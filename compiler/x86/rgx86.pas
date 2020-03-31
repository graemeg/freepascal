{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the x86 specific class for the register
    allocator

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}

unit rgx86;

{$i fpcdefs.inc}

  interface

    uses
      cpubase,cgbase,cgutils,
      aasmtai,aasmdata,aasmsym,aasmcpu,
      rgobj;

    type
       trgx86 = class(trgobj)
         function  get_spill_subreg(r : tregister) : tsubregister;override;
         function  do_spill_replace(list:TAsmList;instr:tai_cpu_abstract_sym;orgreg:tsuperregister;const spilltemp:treference):boolean;override;
       end;

       tpushedsavedloc = record
         case byte of
           0: (pushed: boolean);
           1: (ofs: longint);
       end;

       tpushedsavedfpu = array[tsuperregister] of tpushedsavedloc;

       trgx86fpu = class
          { these counters contain the number of elements in the }
          { unusedregsxxx/usableregsxxx sets                     }
          countunusedregsfpu : byte;

          { Contains the registers which are really used by the proc itself.
            It doesn't take care of registers used by called procedures
          }
          used_in_proc : tcpuregisterset;

          {reg_pushes_other : regvarother_longintarray;
          is_reg_var_other : regvarother_booleanarray;
          regvar_loaded_other : regvarother_booleanarray;}

          fpuvaroffset : byte;

          constructor create;

          function getregisterfpu(list: TAsmList) : tregister;
          procedure ungetregisterfpu(list: TAsmList; r : tregister);

          { pushes and restores registers }
          procedure saveusedfpuregisters(list:TAsmList;
                                         var saved:Tpushedsavedfpu;
                                         const s:Tcpuregisterset);
          procedure restoreusedfpuregisters(list:TAsmList;
                                            const saved:Tpushedsavedfpu);

          { corrects the fpu stack register by ofs }
          function correct_fpuregister(r : tregister;ofs : byte) : tregister;
       end;


implementation

    uses
       verbose;

    const
       { This value is used in tsaved. If the array value is equal
         to this, then this means that this register is not used.}
       reg_not_saved = $7fffffff;


{******************************************************************************
                                    Trgcpu
******************************************************************************}

    function trgx86.get_spill_subreg(r : tregister) : tsubregister;
      begin
        result:=getsubreg(r);
      end;


    { Decide wether a "replace" spill is possible, i.e. wether we can replace a register
      in an instruction by a memory reference. For example, in "mov ireg26d,0", the imaginary
      register ireg26d can be replaced by a memory reference.}
    function trgx86.do_spill_replace(list:TAsmList;instr:tai_cpu_abstract_sym;orgreg:tsuperregister;const spilltemp:treference):boolean;

       { returns true if opcde is an avx opcode which allows only the first (zero) operand might be a memory reference }
       function avx_opcode_only_op0_may_be_memref(opcode : TAsmOp) : boolean;
         begin
           case opcode of
             A_VMULSS,
             A_VMULSD,
             A_VSUBSS,
             A_VSUBSD,
             A_VADDSD,
             A_VADDSS,
             A_VDIVSD,
             A_VDIVSS,
             A_VSQRTSD,
             A_VSQRTSS,
             A_VCVTDQ2PD,
             A_VCVTDQ2PS,
             A_VCVTPD2DQ,
             A_VCVTPD2PS,
             A_VCVTPS2DQ,
             A_VCVTPS2PD,
             A_VCVTSD2SI,
             A_VCVTSD2SS,
             A_VCVTSI2SD,
             A_VCVTSS2SD,
             A_VCVTTPD2DQ,
             A_VCVTTPS2DQ,
             A_VCVTTSD2SI,
             A_VCVTSI2SS,
             A_VCVTSS2SI,
             A_VCVTTSS2SI,
             A_VXORPD,
             A_VXORPS,
             A_VORPD,
             A_VORPS,
             A_VANDPD,
             A_VANDPS,
             A_VUNPCKLPS,
             A_VUNPCKHPS,
             A_VSHUFPD:
               result:=true;
             else
               result:=false;
           end;
         end;


      var
        n,replaceoper : longint;
        is_subh: Boolean;
      begin
        result:=false;
        with taicpu(instr) do
          begin
            replaceoper:=-1;
            case ops of
              1 :
                begin
                  if (oper[0]^.typ=top_reg) and
                     (getregtype(oper[0]^.reg)=regtype) then
                    begin
                      if get_alias(getsupreg(oper[0]^.reg))<>orgreg then
                        internalerror(200410101);
                      replaceoper:=0;
                    end;
                end;
              2,3 :
                begin
                  { avx instruction?
                    currently this rule is sufficient but it might be extended }
                  if (ops=3) and (opcode<>A_SHRD) and (opcode<>A_SHLD) and (opcode<>A_IMUL) then
                    begin
                      { BMI shifting/rotating instructions have special requirements regarding spilling, only
                        the middle operand can be replaced }
                      if ((opcode=A_RORX) or (opcode=A_SHRX) or (opcode=A_SARX) or (opcode=A_SHLX)) then
                        begin
                          if (oper[1]^.typ=top_reg) and (getregtype(oper[1]^.reg)=regtype) and (get_alias(getsupreg(oper[1]^.reg))=orgreg) then
                            replaceoper:=1;
                        end
                      { avx instructions allow only the first operand (at&t counting) to be a register operand
                        all operands must be registers ... }
                      else if (oper[0]^.typ=top_reg) and
                         (oper[1]^.typ=top_reg) and
                         (oper[2]^.typ=top_reg) and
                         { but they must be different }
                         ((getregtype(oper[1]^.reg)<>regtype) or
                          (get_alias(getsupreg(oper[0]^.reg))<>get_alias(getsupreg(oper[1]^.reg)))
                         ) and
                         ((getregtype(oper[2]^.reg)<>regtype) or
                          (get_alias(getsupreg(oper[0]^.reg))<>get_alias(getsupreg(oper[2]^.reg)))
                         ) and
                         (get_alias(getsupreg(oper[0]^.reg))=orgreg) then
                        replaceoper:=0;
                    end
                  else
                    begin
                      { We can handle opcodes with 2 and 3-op imul/shrd/shld the same way, where the 3rd operand is const or CL,
                        that doesn't need spilling.
                        However, due to AT&T order inside the compiler, the 3rd operand is
                        numbered 0, so look at operand no. 1 and 2 if we have 3 operands by
                        adding a "n". }
                      n:=0;
                      if ops=3 then
                        n:=1;
                      { lea is tricky: part of operand 0 can be spilled and the instruction can converted into an
                        add, if base or index shall be spilled and the other one is equal the destination }
                      if (opcode=A_LEA) then
                        begin
                          if (oper[0]^.ref^.offset=0) and
                             (oper[0]^.ref^.scalefactor in [0,1]) and
                             (((getregtype(oper[0]^.ref^.base)=regtype) and
                               (get_alias(getsupreg(oper[0]^.ref^.base))=orgreg) and
                               (getregtype(oper[0]^.ref^.index)=getregtype(oper[1]^.reg)) and
                               (get_alias(getsupreg(oper[0]^.ref^.index))=get_alias(getsupreg(oper[1]^.reg)))) or
                              ((getregtype(oper[0]^.ref^.index)=regtype) and
                               (get_alias(getsupreg(oper[0]^.ref^.index))=orgreg) and
                               (getregtype(oper[0]^.ref^.base)=getregtype(oper[1]^.reg)) and
                               (get_alias(getsupreg(oper[0]^.ref^.base))=get_alias(getsupreg(oper[1]^.reg))))
                             ) then
                             replaceoper:=0;
                        end
                      else if (oper[n+0]^.typ=top_reg) and
                         (oper[n+1]^.typ=top_reg) and
                         ((getregtype(oper[n+0]^.reg)<>regtype) or
                          (getregtype(oper[n+1]^.reg)<>regtype) or
                          (get_alias(getsupreg(oper[n+0]^.reg))<>get_alias(getsupreg(oper[n+1]^.reg)))) then
                        begin
                          if (getregtype(oper[n+0]^.reg)=regtype) and
                             (get_alias(getsupreg(oper[n+0]^.reg))=orgreg) then
                            replaceoper:=0+n
                          else if (getregtype(oper[n+1]^.reg)=regtype) and
                             (get_alias(getsupreg(oper[n+1]^.reg))=orgreg) then
                            replaceoper:=1+n;
                        end
                      else if (oper[n+0]^.typ=top_reg) and
                         (oper[n+1]^.typ=top_const) then
                        begin
                          if (getregtype(oper[0+n]^.reg)=regtype) and
                             (get_alias(getsupreg(oper[0+n]^.reg))=orgreg) then
                            replaceoper:=0+n
                          else
                            internalerror(200704282);
                        end
                      else if (oper[n+0]^.typ=top_const) and
                         (oper[n+1]^.typ=top_reg) then
                        begin
                          if (getregtype(oper[1+n]^.reg)=regtype) and
                             (get_alias(getsupreg(oper[1+n]^.reg))=orgreg) then
                            replaceoper:=1+n
                          else
                            internalerror(200704283);
                        end;
                      case replaceoper of
                        0 :
                          begin
                            { Some instructions don't allow memory references
                              for source }
                            case opcode of
                              A_BT,
                              A_BTS,
                              A_BTC,
                              A_BTR,

                              { shufp*/unpcklp* would require 16 byte alignment for memory locations so we force the source
                                operand into a register }
                              A_SHUFPD,
                              A_SHUFPS,
                              A_UNPCKLPD,
                              A_UNPCKLPS :
                                replaceoper:=-1;

                              { movlhps/movhlps requires the second parameter to be XMM registers }
                              A_MOVHLPS,
                              A_MOVLHPS:
                                replaceoper:=-1;
                              else
                                ;
                            end;
                          end;
                        1 :
                          begin
                            { Some instructions don't allow memory references
                              for destination }
                            case opcode of
                              A_CMOVcc,
                              A_MOVZX,
                              A_MOVSX,
{$ifdef x86_64}
                              A_MOVSXD,
{$endif x86_64}
                              A_MULSS,
                              A_MULSD,
                              A_SUBSS,
                              A_SUBSD,
                              A_ADDSD,
                              A_ADDSS,
                              A_DIVSD,
                              A_DIVSS,
                              A_SQRTSD,
                              A_SQRTSS,
                              A_SHLD,
                              A_SHRD,
                              A_COMISD,
                              A_COMISS,
                              A_CVTDQ2PD,
                              A_CVTDQ2PS,
                              A_CVTPD2DQ,
                              A_CVTPD2PI,
                              A_CVTPD2PS,
                              A_CVTPI2PD,
                              A_CVTPS2DQ,
                              A_CVTPS2PD,
                              A_CVTSD2SI,
                              A_CVTSD2SS,
                              A_CVTSI2SD,
                              A_CVTSS2SD,
                              A_CVTTPD2PI,
                              A_CVTTPD2DQ,
                              A_CVTTPS2DQ,
                              A_CVTTSD2SI,
                              A_CVTPI2PS,
                              A_CVTPS2PI,
                              A_CVTSI2SS,
                              A_CVTSS2SI,
                              A_CVTTPS2PI,
                              A_CVTTSS2SI,
                              A_XORPD,
                              A_XORPS,
                              A_ORPD,
                              A_ORPS,
                              A_ANDPD,
                              A_ANDPS,
                              A_UNPCKLPS,
                              A_UNPCKHPS,
                              A_SHUFPD,
                              A_SHUFPS,
                              A_VCOMISD,
                              A_VCOMISS:
                                replaceoper:=-1;

                              A_IMUL:
                                if ops<>3 then
                                  replaceoper:=-1;
{$ifdef x86_64}
                              A_MOV:
                                 { 64 bit constants can only be moved into registers }
                                 if (oper[0]^.typ=top_const) and
                                    (oper[1]^.typ=top_reg) and
                                    ((oper[0]^.val<low(longint)) or
                                     (oper[0]^.val>high(longint))) then
                                   replaceoper:=-1;
{$endif x86_64}
                              else
                                if avx_opcode_only_op0_may_be_memref(opcode) then
                                  replaceoper:=-1;
                            end;
                          end;
                        2 :
                          begin
                            { Some 3-op instructions don't allow memory references
                              for destination }
                            case instr.opcode of
                              A_IMUL:
                                replaceoper:=-1;
                              else
                                if avx_opcode_only_op0_may_be_memref(opcode) then
                                  replaceoper:=-1;
                            end;
                          end;
                      end;
                    end;
                end;
             end;

{$ifdef x86_64}
            { 32 bit operations on 32 bit registers on x86_64 can result in
              zeroing the upper 32 bits of the register. This does not happen
              with memory operations, so we have to perform these calculations
              in registers.  }
            if (opsize=S_L) then
              replaceoper:=-1;
{$endif x86_64}

            { Replace register with spill reference }
            if replaceoper<>-1 then
              begin
                if opcode=A_LEA then
                  begin
                    opcode:=A_ADD;
                    oper[0]^.ref^:=spilltemp;
                  end
                else
                  begin
                    is_subh:=getsubreg(oper[replaceoper]^.reg)=R_SUBH;
                    oper[replaceoper]^.typ:=top_ref;
                    new(oper[replaceoper]^.ref);
                    oper[replaceoper]^.ref^:=spilltemp;
                    if is_subh then
                      inc(oper[replaceoper]^.ref^.offset);
                    { memory locations aren't guaranteed to be aligned }
                    case opcode of
                      A_MOVAPS:
                        opcode:=A_MOVSS;
                      A_MOVAPD:
                        opcode:=A_MOVSD;
                      A_VMOVAPS:
                        opcode:=A_VMOVSS;
                      A_VMOVAPD:
                        opcode:=A_VMOVSD;
                      else
                        ;
                    end;
                  end;
                result:=true;
              end;
          end;
      end;


{******************************************************************************
                                  Trgx86fpu
******************************************************************************}

    constructor Trgx86fpu.create;
      begin
        used_in_proc:=[];
      end;


    function trgx86fpu.getregisterfpu(list: TAsmList) : tregister;
      begin
        { note: don't return R_ST0, see comments above implementation of }
        { a_loadfpu_* methods in cgcpu (JM)                              }
        result:=NR_ST;
      end;


    procedure trgx86fpu.ungetregisterfpu(list : TAsmList; r : tregister);
      begin
        { nothing to do, fpu stack management is handled by the load/ }
        { store operations in cgcpu (JM)                              }
      end;



    function trgx86fpu.correct_fpuregister(r : tregister;ofs : byte) : tregister;
      begin
        correct_fpuregister:=r;
        setsupreg(correct_fpuregister,ofs);
      end;


    procedure trgx86fpu.saveusedfpuregisters(list: TAsmList;
                                             var saved : tpushedsavedfpu;
                                             const s: tcpuregisterset);
     { var
         r : tregister;
         hr : treference; }
      begin
        used_in_proc:=used_in_proc+s;

{ TODO: firstsavefpureg}
(*
        { don't try to save the fpu registers if not desired (e.g. for }
        { the 80x86)                                                   }
        if firstsavefpureg <> R_NO then
          for r.enum:=firstsavefpureg to lastsavefpureg do
            begin
              saved[r.enum].ofs:=reg_not_saved;
              { if the register is used by the calling subroutine and if }
              { it's not a regvar (those are handled separately)         }
              if not is_reg_var_other[r.enum] and
                 (r.enum in s) and
                 { and is present in use }
                 not(r.enum in unusedregsfpu) then
                begin
                  { then save it }
                  tg.GetTemp(list,extended_size,tt_persistent,hr);
                  saved[r.enum].ofs:=hr.offset;
                  cg.a_loadfpu_reg_ref(list,OS_FLOAT,OS_FLOAT,r,hr);
                  cg.a_reg_dealloc(list,r);
                  include(unusedregsfpu,r.enum);
                  inc(countunusedregsfpu);
                end;
            end;
*)
      end;


    procedure trgx86fpu.restoreusedfpuregisters(list : TAsmList;
                                                const saved : tpushedsavedfpu);
{
      var
         r,r2 : tregister;
         hr : treference;
}
      begin
{ TODO: firstsavefpureg}
(*
        if firstsavefpureg <> R_NO then
          for r.enum:=lastsavefpureg downto firstsavefpureg do
            begin
              if saved[r.enum].ofs <> reg_not_saved then
                begin
                  r2.enum:=R_INTREGISTER;
                  r2.number:=NR_FRAME_POINTER_REG;
                  reference_reset_base(hr,r2,saved[r.enum].ofs);
                  cg.a_reg_alloc(list,r);
                  cg.a_loadfpu_ref_reg(list,OS_FLOAT,OS_FLOAT,hr,r);
                  if not (r.enum in unusedregsfpu) then
                    { internalerror(10)
                      in n386cal we always save/restore the reg *state*
                      using save/restoreunusedstate -> the current state
                      may not be real (JM) }
                  else
                    begin
                      dec(countunusedregsfpu);
                      exclude(unusedregsfpu,r.enum);
                    end;
                  tg.UnGetTemp(list,hr);
                end;
            end;
*)
      end;

(*
    procedure Trgx86fpu.saveotherregvars(list: TAsmList; const s: totherregisterset);
      var
        r: Tregister;
      begin
        if not(cs_opt_regvar in current_settings.optimizerswitches) then
          exit;
        if firstsavefpureg <> NR_NO then
          for r.enum := firstsavefpureg to lastsavefpureg do
            if is_reg_var_other[r.enum] and
               (r.enum in s) then
              store_regvar(list,r);
      end;
*)

end.
