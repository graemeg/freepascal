{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements the code generator for the PowerPC

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
unit cgcpu;

{$i fpcdefs.inc}

  interface

    uses
       globtype,symtype,symdef,
       cgbase,cgobj,cgppc,
       aasmbase,aasmcpu,aasmtai,aasmdata,
       cpubase,cpuinfo,cgutils,cg64f32,rgcpu,
       parabase;

    type
      tcgppc = class(tcgppcgen)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        procedure a_call_name(list : TAsmList;const s : string; weak: boolean);override;
        procedure a_call_reg(list : TAsmList;reg: tregister); override;

        procedure a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister); override;
        procedure a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister); override;

        procedure a_op_const_reg_reg(list: TAsmList; op: TOpCg;
          size: tcgsize; a: tcgint; src, dst: tregister); override;
        procedure a_op_reg_reg_reg(list: TAsmList; op: TOpCg;
          size: tcgsize; src1, src2, dst: tregister); override;

        { move instructions }
        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;
        procedure a_load_reg_reg(list : TAsmList; fromsize, tosize : tcgsize;reg1,reg2 : tregister);override;

        {  comparison operations }
        procedure a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister;
          l : tasmlabel);override;
        procedure a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel); override;

        procedure a_jmp_name(list : TAsmList;const s : string); override;
        procedure a_jmp_always(list : TAsmList;l: tasmlabel); override;

        { 32x32 to 64 bit multiplication }
        procedure a_mul_reg_reg_pair(list: TAsmList;size: tcgsize; src1,src2,dstlo,dsthi: tregister); override;

        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean); override;
        procedure g_save_registers(list:TAsmList); override;
        procedure g_restore_registers(list:TAsmList); override;

        procedure g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);override;

        { find out whether a is of the form 11..00..11b or 00..11...00. If }
        { that's the case, we can use rlwinm to do an AND operation        }
        function get_rlwi_const(a: aint; var l1, l2: longint): boolean;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
      protected
       procedure a_load_regconst_subsetreg_intern(list : TAsmList; fromsize, subsetsize: tcgsize; fromreg: tregister; const sreg: tsubsetregister; slopt: tsubsetloadopt); override;
>>>>>>> graemeg/fixes_2_2
      private

        (* NOT IN USE: *)
        procedure g_stackframe_entry_mac(list : TAsmList;localsize : longint);
        (* NOT IN USE: *)
        procedure g_return_from_proc_mac(list : TAsmList;parasize : tcgint);

        { clear out potential overflow bits from 8 or 16 bit operations  }
        { the upper 24/16 bits of a register after an operation          }
        procedure maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);

        { returns whether a reference can be used immediately in a powerpc }
        { instruction                                                      }
        function issimpleref(const ref: treference): boolean;

        function save_regs(list : TAsmList):longint;
        procedure restore_regs(list : TAsmList);
     end;

     tcg64fppc = class(tcg64f32)
       procedure a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);override;
       procedure a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);override;
       procedure a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64);override;
       procedure a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);override;
     end;

  procedure create_codegen;

const
  TOpCG2AsmOpConstLo: Array[topcg] of TAsmOp = (A_NONE,A_MR,A_ADDI,A_ANDI_,A_DIVWU,
                        A_DIVW,A_MULLW, A_MULLW, A_NONE,A_NONE,A_ORI,
                        A_SRAWI,A_SLWI,A_SRWI,A_SUBI,A_XORI,A_NONE,A_NONE);
  TOpCG2AsmOpConstHi: Array[topcg] of TAsmOp = (A_NONE,A_MR,A_ADDIS,A_ANDIS_,
                        A_DIVWU,A_DIVW, A_MULLW,A_MULLW,A_NONE,A_NONE,
                        A_ORIS,A_NONE, A_NONE,A_NONE,A_SUBIS,A_XORIS,A_NONE,A_NONE);

  implementation

    uses
       globals,verbose,systems,cutils,
       symconst,symsym,fmodule,
       rgobj,tgobj,cpupi,procinfo,paramgr;


    procedure tcgppc.init_register_allocators;
      begin
        inherited init_register_allocators;
        if target_info.system=system_powerpc_darwin then
          begin
{
            if pi_needs_got in current_procinfo.flags then
              begin
                current_procinfo.got:=NR_R31;
                rg[R_INTREGISTER]:=trgcpu.create(R_INTREGISTER,R_SUBWHOLE,
                  [RS_R2,RS_R3,RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,
                   RS_R9,RS_R10,RS_R11,RS_R12,RS_R30,RS_R29,
                   RS_R28,RS_R27,RS_R26,RS_R25,RS_R24,RS_R23,RS_R22,
                   RS_R21,RS_R20,RS_R19,RS_R18,RS_R17,RS_R16,RS_R15,
                   RS_R14,RS_R13],first_int_imreg,[]);
              end
            else}
              rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
                [{$ifdef user0} RS_R0,{$endif} RS_R2,RS_R3,RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,
                 RS_R9,RS_R10,RS_R11,RS_R12,RS_R31,RS_R30,RS_R29,
                 RS_R28,RS_R27,RS_R26,RS_R25,RS_R24,RS_R23,RS_R22,
                 RS_R21,RS_R20,RS_R19,RS_R18,RS_R17,RS_R16,RS_R15,
                 RS_R14,RS_R13],first_int_imreg,[]);
          end
        else
          rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
            [{$ifdef user0} RS_R0,{$endif}RS_R3,RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,
             RS_R9,RS_R10,RS_R11,RS_R12,RS_R31,RS_R30,RS_R29,
             RS_R28,RS_R27,RS_R26,RS_R25,RS_R24,RS_R23,RS_R22,
             RS_R21,RS_R20,RS_R19,RS_R18,RS_R17,RS_R16,RS_R15,
             RS_R14,RS_R13],first_int_imreg,[]);
        rg[R_FPUREGISTER]:=trgcpu.create(R_FPUREGISTER,R_SUBNONE,
            [RS_F0,RS_F1,RS_F2,RS_F3,RS_F4,RS_F5,RS_F6,RS_F7,RS_F8,RS_F9,
             RS_F10,RS_F11,RS_F12,RS_F13,RS_F31,RS_F30,RS_F29,RS_F28,RS_F27,
             RS_F26,RS_F25,RS_F24,RS_F23,RS_F22,RS_F21,RS_F20,RS_F19,RS_F18,
             RS_F17,RS_F16,RS_F15,RS_F14],first_fpu_imreg,[]);
        { TODO: FIX ME}
        rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBNONE,
            [RS_M0,RS_M1,RS_M2],first_mm_imreg,[]);
      end;


    procedure tcgppc.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_MMREGISTER].free;
        inherited done_register_allocators;
      end;

    { calling a procedure by name }
    procedure tcgppc.a_call_name(list : TAsmList;const s : string; weak: boolean);
      begin
         { MacOS: The linker on MacOS (PPCLink) inserts a call to glue code,
           if it is a cross-TOC call. If so, it also replaces the NOP
           with some restore code.}
         if (target_info.system<>system_powerpc_darwin) then
           begin
             if target_info.system<>system_powerpc_aix then
               begin
                 if not(weak) then
                   list.concat(taicpu.op_sym(A_BL,current_asmdata.RefAsmSymbol(s)))
                 else
                   list.concat(taicpu.op_sym(A_BL,current_asmdata.WeakRefAsmSymbol(s)));
               end
             else
               begin
                 if not(weak) then
                   list.concat(taicpu.op_sym(A_BL,current_asmdata.RefAsmSymbol('.'+s)))
                 else
                   list.concat(taicpu.op_sym(A_BL,current_asmdata.WeakRefAsmSymbol('.'+s)));
               end;

             if target_info.system in [system_powerpc_macos,system_powerpc_aix] then
               list.concat(taicpu.op_none(A_NOP));
           end
         else
           list.concat(taicpu.op_sym(A_BL,get_darwin_call_stub(s,weak)));
{
       the compiler does not properly set this flag anymore in pass 1, and
       for now we only need it after pass 2 (I hope) (JM)
         if not(pi_do_call in current_procinfo.flags) then
           internalerror(2003060703);
}
       { not assigned while generating external wrappers }
       if assigned(current_procinfo) then
         include(current_procinfo.flags,pi_do_call);
      end;

    { calling a procedure by address }
    procedure tcgppc.a_call_reg(list : TAsmList;reg: tregister);

      var
        tmpreg : tregister;
        tmpref : treference;

      begin
        if target_info.system=system_powerpc_macos then
          begin
            {Generate instruction to load the procedure address from
            the transition vector.}
            //TODO: Support cross-TOC calls.
            tmpreg := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
            reference_reset(tmpref,4);
            tmpref.offset := 0;
            //tmpref.symaddr := refs_full;
            tmpref.base:= reg;
            list.concat(taicpu.op_reg_ref(A_LWZ,tmpreg,tmpref));
          end
        else
          tmpreg:=reg;
        inherited a_call_reg(list,tmpreg);
      end;


{********************** load instructions ********************}

     procedure tcgppc.a_load_const_reg(list : TAsmList; size: TCGSize; a : tcgint; reg : TRegister);

       begin
          if not(size in [OS_8,OS_S8,OS_16,OS_S16,OS_32,OS_S32]) then
            internalerror(2002090902);
          if (a >= low(smallint)) and
             (a <= high(smallint)) then
            list.concat(taicpu.op_reg_const(A_LI,reg,smallint(a)))
          else if ((a and $ffff) <> 0) then
            begin
              list.concat(taicpu.op_reg_const(A_LI,reg,smallint(a and $ffff)));
              if ((a shr 16) <> 0) or
                 (smallint(a and $ffff) < 0) then
                list.concat(taicpu.op_reg_reg_const(A_ADDIS,reg,reg,
                  smallint((a shr 16)+ord(smallint(a and $ffff) < 0))))
            end
          else
            list.concat(taicpu.op_reg_const(A_LIS,reg,smallint(a shr 16)));
       end;


     procedure tcgppc.a_load_ref_reg(list : TAsmList; fromsize,tosize : tcgsize;const ref: treference;reg : tregister);

       const
         LoadInstr: Array[OS_8..OS_S32,boolean, boolean] of TAsmOp =
                                { indexed? updating?}
                    (((A_LBZ,A_LBZU),(A_LBZX,A_LBZUX)),
                     ((A_LHZ,A_LHZU),(A_LHZX,A_LHZUX)),
                     ((A_LWZ,A_LWZU),(A_LWZX,A_LWZUX)),
                     { 64bit stuff should be handled separately }
                     ((A_NONE,A_NONE),(A_NONE,A_NONE)),
                     { 128bit stuff too }
                     ((A_NONE,A_NONE),(A_NONE,A_NONE)),
                     { there's no load-byte-with-sign-extend :( }
                     ((A_LBZ,A_LBZU),(A_LBZX,A_LBZUX)),
                     ((A_LHA,A_LHAU),(A_LHAX,A_LHAUX)),
                     ((A_LWZ,A_LWZU),(A_LWZX,A_LWZUX)));
       var
         op: tasmop;
         ref2: treference;

       begin
          if target_info.system=system_powerpc_aix then
            g_load_check_simple(list,ref,65536);
          { TODO: optimize/take into consideration fromsize/tosize. Will }
          { probably only matter for OS_S8 loads though                  }
          if not(fromsize in [OS_8,OS_S8,OS_16,OS_S16,OS_32,OS_S32]) then
            internalerror(2002090903);
          ref2 := ref;
          fixref(list,ref2);
          { the caller is expected to have adjusted the reference already }
          { in this case                                                  }
          if (TCGSize2Size[fromsize] >= TCGSize2Size[tosize]) then
            fromsize := tosize;
          op := loadinstr[fromsize,ref2.index<>NR_NO,false];
          a_load_store(list,op,reg,ref2);
<<<<<<< HEAD
          { sign extend shortint if necessary (because there is
	   no load instruction to sign extend an 8 bit value automatically)
	   and mask out extra sign bits when loading from a smaller signed
	   to a larger unsigned type }
          if fromsize = OS_S8 then
	    begin
	      a_load_reg_reg(list, OS_8, OS_S8, reg, reg);
	      a_load_reg_reg(list, OS_S8, tosize, reg, reg);
	    end;
=======
          { sign extend shortint if necessary, since there is no }
          { load instruction that does that automatically (JM)   }
          if (fromsize = OS_S8) then
            begin
              list.concat(taicpu.op_reg_reg(A_EXTSB,reg,reg));
              if (tosize = OS_16) then
                a_load_reg_reg(list,fromsize,tosize,reg,reg);
            end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
       end;


     procedure tcgppc.a_load_reg_reg(list : TAsmList;fromsize, tosize : tcgsize;reg1,reg2 : tregister);

       var
         instr: taicpu;
       begin
         if (tcgsize2size[fromsize] > tcgsize2size[tosize]) or
            ((tcgsize2size[fromsize] = tcgsize2size[tosize]) and
             (fromsize <> tosize)) or
            { needs to mask out the sign in the top 16 bits }
            ((fromsize = OS_S8) and
             (tosize = OS_16)) then
           case tosize of
             OS_8:
               instr := taicpu.op_reg_reg_const_const_const(A_RLWINM,
                 reg2,reg1,0,31-8+1,31);
             OS_S8:
               instr := taicpu.op_reg_reg(A_EXTSB,reg2,reg1);
             OS_16:
               instr := taicpu.op_reg_reg_const_const_const(A_RLWINM,
                 reg2,reg1,0,31-16+1,31);
             OS_S16:
               instr := taicpu.op_reg_reg(A_EXTSH,reg2,reg1);
             OS_32,OS_S32:
               instr := taicpu.op_reg_reg(A_MR,reg2,reg1);
             else internalerror(2002090901);
           end
         else
           instr := taicpu.op_reg_reg(A_MR,reg2,reg1);

         list.concat(instr);
         rg[R_INTREGISTER].add_move_instruction(instr);
       end;


     procedure tcgppc.a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister);

       begin
         a_op_const_reg_reg(list,op,size,a,reg,reg);
       end;


      procedure tcgppc.a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister);

         begin
           a_op_reg_reg_reg(list,op,size,src,dst,dst);
         end;


    procedure tcgppc.maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);
      const
        overflowops = [OP_MUL,OP_SHL,OP_ADD,OP_SUB,OP_NOT,OP_NEG];
      begin
        if (op in overflowops) and
           (size in [OS_8,OS_S8,OS_16,OS_S16]) then
          a_load_reg_reg(list,OS_32,size,dst,dst);
      end;


    procedure tcgppc.a_op_const_reg_reg(list: TAsmList; op: TOpCg;
                       size: tcgsize; a: tcgint; src, dst: tregister);
      var
        l1,l2: longint;
        oplo, ophi: tasmop;
        scratchreg: tregister;
        useReg, gotrlwi: boolean;


        procedure do_lo_hi;
          begin
            list.concat(taicpu.op_reg_reg_const(oplo,dst,src,word(a)));
            list.concat(taicpu.op_reg_reg_const(ophi,dst,dst,word(a shr 16)));
          end;

      begin
        if (op = OP_MOVE) then
          internalerror(2006031401);
        if op = OP_SUB then
          begin
            a_op_const_reg_reg(list,OP_ADD,size,-a,src,dst);
            exit;
          end;
        ophi := TOpCG2AsmOpConstHi[op];
        oplo := TOpCG2AsmOpConstLo[op];
        gotrlwi := get_rlwi_const(aint(a),l1,l2);
        if (op in [OP_AND,OP_OR,OP_XOR]) then
          begin
            if (a = 0) then
              begin
                if op = OP_AND then
                  list.concat(taicpu.op_reg_const(A_LI,dst,0))
                else
                  a_load_reg_reg(list,size,size,src,dst);
                exit;
              end
            else if (a = -1) then
              begin
                case op of
                  OP_OR:
                    list.concat(taicpu.op_reg_const(A_LI,dst,-1));
                  OP_XOR:
                    list.concat(taicpu.op_reg_reg(A_NOT,dst,src));
                  OP_AND:
                    a_load_reg_reg(list,size,size,src,dst);
                end;
                exit;
              end
            else if (aword(a) <= high(word)) and
               ((op <> OP_AND) or
                not gotrlwi) then
              begin
                if ((size = OS_8) and
                    (byte(a) <> a)) or
                   ((size = OS_S8) and
                    (shortint(a) <> a)) then
                  internalerror(200604142);
                list.concat(taicpu.op_reg_reg_const(oplo,dst,src,word(a)));
                { and/or/xor -> cannot overflow in high 16 bits }
                exit;
              end;
            { all basic constant instructions also have a shifted form that }
            { works only on the highest 16bits, so if lo(a) is 0, we can    }
            { use that one                                                  }
            if (word(a) = 0) and
               (not(op = OP_AND) or
                not gotrlwi) then
              begin
                if (size in [OS_8,OS_S8,OS_16,OS_S16]) then
                  internalerror(200604141);
                list.concat(taicpu.op_reg_reg_const(ophi,dst,src,word(a shr 16)));
                exit;
              end;
          end
        else if (op = OP_ADD) then
          if a = 0 then
            begin
              a_load_reg_reg(list,size,size,src,dst);
              exit
            end
          else if (a >= low(smallint)) and
                  (a <= high(smallint)) then
             begin
               list.concat(taicpu.op_reg_reg_const(A_ADDI,dst,src,smallint(a)));
               maybeadjustresult(list,op,size,dst);
               exit;
             end;

        { otherwise, the instructions we can generate depend on the }
        { operation                                                 }
        useReg := false;
        case op of
          OP_DIV,OP_IDIV:
             if (a = 0) then
               internalerror(200208103)
             else if (a = 1) then
               begin
                 a_load_reg_reg(list,OS_INT,OS_INT,src,dst);
                 exit
               end
            else if ispowerof2(a,l1) then
              begin
                case op of
                  OP_DIV:
                    list.concat(taicpu.op_reg_reg_const(A_SRWI,dst,src,l1));
                  OP_IDIV:
                    begin
                       list.concat(taicpu.op_reg_reg_const(A_SRAWI,dst,src,l1));
                       list.concat(taicpu.op_reg_reg(A_ADDZE,dst,dst));
                    end;
                end;
                exit;
              end
            else
              usereg := true;
           OP_IMUL, OP_MUL:
             if (a = 0) then
               begin
                 list.concat(taicpu.op_reg_const(A_LI,dst,0));
                 exit
               end
             else if (a = 1) then
               begin
                 a_load_reg_reg(list,OS_INT,OS_INT,src,dst);
                 exit
               end
             else if ispowerof2(a,l1) then
               list.concat(taicpu.op_reg_reg_const(A_SLWI,dst,src,l1))
             else if (longint(a) >= low(smallint)) and
                (longint(a) <= high(smallint)) then
               list.concat(taicpu.op_reg_reg_const(A_MULLI,dst,src,smallint(a)))
             else
               usereg := true;
          OP_ADD:
            begin
              list.concat(taicpu.op_reg_reg_const(oplo,dst,src,smallint(a)));
              list.concat(taicpu.op_reg_reg_const(ophi,dst,dst,
                smallint((a shr 16) + ord(smallint(a) < 0))));
            end;
          OP_OR:
            { try to use rlwimi }
            if gotrlwi and
               (src = dst) then
              begin
                scratchreg := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
                list.concat(taicpu.op_reg_const(A_LI,scratchreg,-1));
                list.concat(taicpu.op_reg_reg_const_const_const(A_RLWIMI,dst,
                  scratchreg,0,l1,l2));
              end
            else
              do_lo_hi;
          OP_AND:
            { try to use rlwinm }
            if gotrlwi then
              list.concat(taicpu.op_reg_reg_const_const_const(A_RLWINM,dst,
                src,0,l1,l2))
            else
              useReg := true;
          OP_XOR:
            do_lo_hi;
          OP_SHL,OP_SHR,OP_SAR:
            begin
              if (a and 31) <> 0 Then
                list.concat(taicpu.op_reg_reg_const(
                  TOpCG2AsmOpConstLo[Op],dst,src,a and 31))
              else
                a_load_reg_reg(list,size,size,src,dst);
              if (a shr 5) <> 0 then
                internalError(68991);
            end;
	  OP_ROL:
	    begin
	      if (not (size in [OS_32, OS_S32])) then begin
	        internalerror(2008091307);
	      end;
	      list.concat(taicpu.op_reg_reg_const_const_const(A_RLWINM, dst, src, a and 31, 0, 31));
	    end;
	  OP_ROR:
	    begin
	      if (not (size in [OS_32, OS_S32])) then begin
		internalerror(2008091308);
	      end;
	      list.concat(taicpu.op_reg_reg_const_const_const(A_RLWINM, dst, src, (32 - a) and 31, 0, 31));
	    end
          else
            internalerror(200109091);
        end;
        { if all else failed, load the constant in a register and then }
        { perform the operation                                        }
        if useReg then
          begin
            scratchreg := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
            a_load_const_reg(list,OS_32,a,scratchreg);
            a_op_reg_reg_reg(list,op,OS_32,scratchreg,src,dst);
          end;
        maybeadjustresult(list,op,size,dst);
      end;


    procedure tcgppc.a_op_reg_reg_reg(list: TAsmList; op: TOpCg;
      size: tcgsize; src1, src2, dst: tregister);

      const
        op_reg_reg_opcg2asmop: array[TOpCG] of tasmop =
          (A_NONE,A_MR,A_ADD,A_AND,A_DIVWU,A_DIVW,A_MULLW,A_MULLW,A_NEG,A_NOT,A_OR,
           A_SRAW,A_SLW,A_SRW,A_SUB,A_XOR,A_NONE,A_NONE);
      var
        tmpreg : TRegister;

       begin
         if (op = OP_MOVE) then
           internalerror(2006031402);
         case op of
           OP_NEG,OP_NOT:
             begin
               list.concat(taicpu.op_reg_reg(op_reg_reg_opcg2asmop[op],dst,src1));
               if (op = OP_NOT) and
                  not(size in [OS_32,OS_S32]) then
                 { zero/sign extend result again }
                 a_load_reg_reg(list,OS_32,size,dst,dst);
              end;
	   OP_ROL:
	     begin
	       if (not (size in [OS_32, OS_S32])) then begin
	         internalerror(2008091305);
	       end;
	       list.concat(taicpu.op_reg_reg_reg_const_const(A_RLWNM, dst, src2, src1, 0, 31));
	     end;
	   OP_ROR:
	     begin
	       if (not (size in [OS_32, OS_S32])) then begin
	         internalerror(2008091306);
	       end;
	       tmpreg := getintregister(list, OS_INT);
	       list.concat(taicpu.op_reg_reg(A_NEG, tmpreg, src1));
	       list.concat(taicpu.op_reg_reg_reg_const_const(A_RLWNM, dst, src2, tmpreg, 0, 31));
	     end;	
           else
             list.concat(taicpu.op_reg_reg_reg(op_reg_reg_opcg2asmop[op],dst,src2,src1));
         end;
         maybeadjustresult(list,op,size,dst);
       end;


{*************** compare instructructions ****************}

      procedure tcgppc.a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister;
        l : tasmlabel);

        var
          scratch_register: TRegister;
          signed: boolean;

        begin
          signed := cmp_op in [OC_GT,OC_LT,OC_GTE,OC_LTE,OC_EQ,OC_NE];
          { in the following case, we generate more efficient code when }
          { signed is false                                              }
          if (cmp_op in [OC_EQ,OC_NE]) and
             (aword(a) >= $8000) and
             (aword(a) <= $ffff) then
            signed := false;
          if signed then
            if (a >= low(smallint)) and (a <= high(smallint)) Then
              list.concat(taicpu.op_reg_reg_const(A_CMPWI,NR_CR0,reg,a))
            else
              begin
                scratch_register := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
                a_load_const_reg(list,OS_32,a,scratch_register);
                list.concat(taicpu.op_reg_reg_reg(A_CMPW,NR_CR0,reg,scratch_register));
              end
          else
            if (aword(a) <= $ffff) then
              list.concat(taicpu.op_reg_reg_const(A_CMPLWI,NR_CR0,reg,aword(a)))
            else
              begin
                scratch_register := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
                a_load_const_reg(list,OS_32,a,scratch_register);
                list.concat(taicpu.op_reg_reg_reg(A_CMPLW,NR_CR0,reg,scratch_register));
              end;
          a_jmp(list,A_BC,TOpCmp2AsmCond[cmp_op],0,l);
        end;


      procedure tcgppc.a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;
        reg1,reg2 : tregister;l : tasmlabel);

        var
          op: tasmop;

        begin
          if cmp_op in [OC_GT,OC_LT,OC_GTE,OC_LTE] then
            op := A_CMPW
          else
            op := A_CMPLW;
          list.concat(taicpu.op_reg_reg_reg(op,NR_CR0,reg2,reg1));
          a_jmp(list,A_BC,TOpCmp2AsmCond[cmp_op],0,l);
        end;


    procedure tcgppc.a_jmp_name(list : TAsmList;const s : string);
      var
        p : taicpu;
      begin
         if (target_info.system = system_powerpc_darwin) then
           p := taicpu.op_sym(A_B,get_darwin_call_stub(s,false))
        else
          p := taicpu.op_sym(A_B,current_asmdata.RefAsmSymbol(s));
        p.is_jmp := true;
        list.concat(p)
      end;


     procedure tcgppc.a_jmp_always(list : TAsmList;l: tasmlabel);

       begin
         a_jmp(list,A_B,C_None,0,l);
       end;


    procedure tcgppc.a_mul_reg_reg_pair(list: TAsmList;size: tcgsize; src1,src2,dstlo,dsthi: tregister);
      var
        op: tasmop;
      begin
        case size of
          OS_INT:  op:=A_MULHWU;
          OS_SINT: op:=A_MULHW;
        else
          InternalError(2014061501);
        end;
        if (dsthi<>NR_NO) then
          list.concat(taicpu.op_reg_reg_reg(op,dsthi,src1,src2));
        { low word is always unsigned }
        if (dstlo<>NR_NO) then
          list.concat(taicpu.op_reg_reg_reg(A_MULLW,dstlo,src1,src2));
      end;

(*
     procedure tcgppc.g_cond2reg(list: TAsmList; const f: TAsmCond; reg: TRegister);

       var
         testbit: byte;
         bitvalue: boolean;

       begin
         { get the bit to extract from the conditional register + its }
         { requested value (0 or 1)                                   }
         case f.simple of
           false:
             begin
               { we don't generate this in the compiler }
               internalerror(200109062);
             end;
           true:
             case f.cond of
               C_None:
                 internalerror(200109063);
               C_LT..C_NU:
                 begin
                   testbit := (ord(f.cr) - ord(R_CR0))*4;
                   inc(testbit,AsmCondFlag2BI[f.cond]);
                   bitvalue := AsmCondFlagTF[f.cond];
                 end;
               C_T,C_F,C_DNZT,C_DNZF,C_DZT,C_DZF:
                 begin
                   testbit := f.crbit
                   bitvalue := AsmCondFlagTF[f.cond];
                 end;
               else
                 internalerror(200109064);
             end;
         end;
         { load the conditional register in the destination reg }
         list.concat(taicpu.op_reg_reg(A_MFCR,reg));
         { we will move the bit that has to be tested to bit 31 -> rotate }
         { left by bitpos+1 (remember, this is big-endian!)               }
         if bitpos <> 31 then
           inc(bitpos)
         else
           bitpos := 0;
         { extract bit }
         list.concat(taicpu.op_reg_reg_const_const_const(
           A_RLWINM,reg,reg,bitpos,31,31));
         { if we need the inverse, xor with 1 }
         if not bitvalue then
           list.concat(taicpu.op_reg_reg_const(A_XORI,reg,reg,1));
       end;
*)

{ *********** entry/exit code and address loading ************ }

    procedure tcgppc.g_save_registers(list:TAsmList);
      begin
        { this work is done in g_proc_entry }
      end;


    procedure tcgppc.g_restore_registers(list:TAsmList);
      begin
        { this work is done in g_proc_exit }
      end;


    procedure tcgppc.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
     { generated the entry code of a procedure/function. Note: localsize is the }
     { sum of the size necessary for local variables and the maximum possible   }
     { combined size of ALL the parameters of a procedure called by the current }
     { one.                                                                     }
     { This procedure may be called before, as well as after g_return_from_proc }
     { is called. NOTE registers are not to be allocated through the register   }
     { allocator here, because the register colouring has already occured !!    }


     var regcounter,firstregfpu,firstregint: TSuperRegister;
         href : treference;
         usesfpr,usesgpr : boolean;

      begin
        { CR and LR only have to be saved in case they are modified by the current }
        { procedure, but currently this isn't checked, so save them always         }
        { following is the entry code as described in "Altivec Programming }
        { Interface Manual", bar the saving of AltiVec registers           }
        a_reg_alloc(list,NR_STACK_POINTER_REG);

        usesgpr := false;
        usesfpr := false;
        firstregint := RS_NO;
        firstregfpu := RS_NO;

        if not(po_assembler in current_procinfo.procdef.procoptions) then
          begin
            { save link register? }
            if save_lr_in_prologue then
              begin
                a_reg_alloc(list,NR_R0);
                { save return address... }
                { warning: if this is no longer done via r0, or if r0 is       }
                { added to the usable registers, adapt tcgppcgen.g_profilecode }
                list.concat(taicpu.op_reg(A_MFLR,NR_R0));
                { ... in caller's frame }
                case target_info.abi of
                  abi_powerpc_aix,
                  abi_powerpc_darwin:
                    reference_reset_base(href,NR_STACK_POINTER_REG,LA_LR_AIX,4);
                  abi_powerpc_sysv:
                    reference_reset_base(href,NR_STACK_POINTER_REG,LA_LR_SYSV,4);
                end;
                list.concat(taicpu.op_reg_ref(A_STW,NR_R0,href));
                if not(cs_profile in current_settings.moduleswitches) then
                  a_reg_dealloc(list,NR_R0);
              end;

(*
            { save the CR if necessary in callers frame. }
            if target_info.abi in [abi_powerpc_aix,abi_powerpc_darwin]  then
              if false then { Not needed at the moment. }
                begin
                  a_reg_alloc(list,NR_R0);
                  list.concat(taicpu.op_reg_reg(A_MFSPR,NR_R0,NR_CR));
                  reference_reset_base(href,NR_STACK_POINTER_REG,LA_CR_AIX);
                  list.concat(taicpu.op_reg_ref(A_STW,NR_R0,href));
                  a_reg_dealloc(list,NR_R0);
                end;
*)

            firstregfpu := tppcprocinfo(current_procinfo).get_first_save_fpu_reg;
            firstregint := tppcprocinfo(current_procinfo).get_first_save_int_reg;
            usesgpr := firstregint <> 32;
            usesfpr := firstregfpu <> 32;

             if tppcprocinfo(current_procinfo).needs_frame_pointer then
               list.concat(taicpu.op_reg_reg(A_MR,NR_OLD_STACK_POINTER_REG,NR_STACK_POINTER_REG));
          end;

        if usesfpr then
          begin
<<<<<<< HEAD
<<<<<<< HEAD
             reference_reset_base(href,NR_R1,-8,8);
=======
=======
>>>>>>> origin/fixes_2_2
             reference_reset_base(href,NR_R1,-8);
>>>>>>> graemeg/fixes_2_2
             for regcounter:=firstregfpu to RS_F31 do
               begin
                 a_loadfpu_reg_ref(list,OS_F64,OS_F64,newreg(R_FPUREGISTER,regcounter,R_SUBNONE),href);
                 dec(href.offset,8);
               end;
             { compute start of gpr save area }
             inc(href.offset,4);
          end
        else
          { compute start of gpr save area }
          reference_reset_base(href,NR_R1,-4,4);

        { save gprs and fetch GOT pointer }
        if usesgpr then
          begin
            if (firstregint <= RS_R22) or
               ((cs_opt_size in current_settings.optimizerswitches) and
               { with RS_R30 it's also already smaller, but too big a speed trade-off to make }
                (firstregint <= RS_R29)) then
              begin
                { TODO: TODO: 64 bit support }
                dec(href.offset,(RS_R31-firstregint)*sizeof(pint));
                list.concat(taicpu.op_reg_ref(A_STMW,newreg(R_INTREGISTER,firstregint,R_SUBNONE),href));
              end
            else
              for regcounter:=firstregint to RS_R31 do
                begin
                  a_load_reg_ref(list,OS_INT,OS_INT,newreg(R_INTREGISTER,regcounter,R_SUBNONE),href);
                  dec(href.offset,4);
                end;
          end;

{        done in ncgutil because it may only be released after the parameters }
{        have been moved to their final resting place                         }
{        if (tppcprocinfo(current_procinfo).needs_frame_pointer) then }
{          a_reg_dealloc(list,NR_R12); }

        if (not nostackframe) and
           tppcprocinfo(current_procinfo).needstackframe and
           (localsize <> 0) then
          begin
            if (localsize <= high(smallint)) then
              begin
                reference_reset_base(href,NR_STACK_POINTER_REG,-localsize,8);
                a_load_store(list,A_STWU,NR_STACK_POINTER_REG,href);
              end
            else
              begin
                reference_reset_base(href,NR_STACK_POINTER_REG,0,4);
                { can't use getregisterint here, the register colouring }
                { is already done when we get here                      }
                { R12 may hold previous stack pointer, R11  may be in   }
                { use as got => use R0 (but then we can't use           }
                { a_load_const_reg)                                     }
                href.index := NR_R0;
                a_reg_alloc(list,href.index);
                list.concat(taicpu.op_reg_const(A_LI,NR_R0,smallint((-localsize) and $ffff)));
                if (smallint((-localsize) and $ffff) < 0) then
                  { upper 16 bits are now $ffff -> xor with inverse }
                  list.concat(taicpu.op_reg_reg_const(A_XORIS,NR_R0,NR_R0,word(not(((-localsize) shr 16) and $ffff))))
                else
                  list.concat(taicpu.op_reg_reg_const(A_ORIS,NR_R0,NR_R0,word(((-localsize) shr 16) and $ffff)));
                a_load_store(list,A_STWUX,NR_STACK_POINTER_REG,href);
                a_reg_dealloc(list,href.index);
              end;
          end;

        { save the CR if necessary ( !!! never done currently ) }
{       still need to find out where this has to be done for SystemV
        a_reg_alloc(list,R_0);
        list.concat(taicpu.op_reg_reg(A_MFSPR,R_0,R_CR);
        list.concat(taicpu.op_reg_ref(A_STW,scratch_register,
          new_reference(STACK_POINTER_REG,LA_CR)));
        a_reg_dealloc(list,R_0);
}
        { now comes the AltiVec context save, not yet implemented !!! }

      end;


    procedure tcgppc.g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean);
     { This procedure may be called before, as well as after g_stackframe_entry }
     { is called. NOTE registers are not to be allocated through the register   }
     { allocator here, because the register colouring has already occured !!    }

      var
         regcounter,firstregfpu,firstregint: TsuperRegister;
         href : treference;
         usesfpr,usesgpr,genret : boolean;
         localsize: tcgint;
      begin
        { AltiVec context restore, not yet implemented !!! }
        firstregint:=RS_NO;
        firstregfpu:=RS_NO;

        usesfpr:=false;
        usesgpr:=false;
        if not (po_assembler in current_procinfo.procdef.procoptions) then
          begin
            firstregfpu := tppcprocinfo(current_procinfo).get_first_save_fpu_reg;
            firstregint := tppcprocinfo(current_procinfo).get_first_save_int_reg;
            usesgpr := firstregint <> 32;
            usesfpr := firstregfpu <> 32;
          end;

        localsize:= tppcprocinfo(current_procinfo).calc_stackframe_size;

        { adjust r1 }
        { (register allocator is no longer valid at this time and an add of 0   }
        { is translated into a move, which is then registered with the register }
        { allocator, causing a crash                                            }
        if (not nostackframe) and
           tppcprocinfo(current_procinfo).needstackframe and
           (localsize <> 0) then
          a_op_const_reg(list,OP_ADD,OS_ADDR,localsize,NR_R1);

        { no return (blr) generated yet }
        genret:=true;
        if usesfpr then
          begin
            reference_reset_base(href,NR_R1,-8,8);
            for regcounter := firstregfpu to RS_F31 do
              begin
                a_loadfpu_ref_reg(list,OS_F64,OS_F64,href,newreg(R_FPUREGISTER,regcounter,R_SUBNONE));
                dec(href.offset,8);
              end;
            inc(href.offset,4);
          end
        else
          reference_reset_base(href,NR_R1,-4,4);

        if (usesgpr) then
          begin
            if (firstregint <= RS_R22) or
               ((cs_opt_size in current_settings.optimizerswitches) and
                { with RS_R30 it's also already smaller, but too big a speed trade-off to make }
                (firstregint <= RS_R29)) then
              begin
                { TODO: TODO: 64 bit support }
                dec(href.offset,(RS_R31-firstregint)*sizeof(pint));
                list.concat(taicpu.op_reg_ref(A_LMW,newreg(R_INTREGISTER,firstregint,R_SUBNONE),href));
              end
            else
              for regcounter:=firstregint to RS_R31 do
                begin
                  a_load_ref_reg(list,OS_INT,OS_INT,href,newreg(R_INTREGISTER,regcounter,R_SUBNONE));
                  dec(href.offset,4);
                end;
          end;

(*
        { restore fprs and return }
        if usesfpr then
          begin
             { address of fpr save area to r11 }
             r:=NR_R12;
             list.concat(taicpu.op_reg_reg_const(A_ADDI,r,r,(ord(R_F31)-ord(firstregfpu.enum)+1)*8));
             {
             if (pi_do_call in current_procinfo.flags) then
               a_call_name(current_asmdata.RefAsmSymbol('_restfpr_'+tostr(ord(firstregfpu)-ord(R_F14)+14)+'_x'))
             else
               { leaf node => lr haven't to be restored }
               a_call_name('_restfpr_'+tostr(ord(firstregfpu.enum)-ord(R_F14)+14)+'_l');
             genret:=false;
             }
          end;
*)

        { if we didn't generate the return code, we've to do it now }
        if genret then
          begin
            { load link register? }
            if not (po_assembler in current_procinfo.procdef.procoptions) then
              begin
                if (pi_do_call in current_procinfo.flags) then
                  begin
                    case target_info.abi of
                      abi_powerpc_aix,
                      abi_powerpc_darwin:
                        reference_reset_base(href,NR_STACK_POINTER_REG,LA_LR_AIX,4);
                      abi_powerpc_sysv:
                        reference_reset_base(href,NR_STACK_POINTER_REG,LA_LR_SYSV,4);
                    end;
                    a_reg_alloc(list,NR_R0);
                    list.concat(taicpu.op_reg_ref(A_LWZ,NR_R0,href));
                    list.concat(taicpu.op_reg(A_MTLR,NR_R0));
                    a_reg_dealloc(list,NR_R0);
                  end;

(*
                  { restore the CR if necessary from callers frame}
                  if target_info.abi in [abi_powerpc_aix,abi_powerpc_darwin] then
                    if false then { Not needed at the moment. }
                      begin
                        reference_reset_base(href,NR_STACK_POINTER_REG,LA_CR_AIX);
                        list.concat(taicpu.op_reg_ref(A_LWZ,NR_R0,href));
                        list.concat(taicpu.op_reg_reg(A_MTSPR,NR_R0,NR_CR));
                        a_reg_dealloc(list,NR_R0);
                      end;
*)
              end;

            list.concat(taicpu.op_none(A_BLR));
          end;
      end;


    function tcgppc.save_regs(list : TAsmList):longint;
    {Generates code which saves used non-volatile registers in
     the save area right below the address the stackpointer point to.
     Returns the actual used save area size.}

     var regcounter,firstregfpu,firstreggpr: TSuperRegister;
         usesfpr,usesgpr: boolean;
         href : treference;
         offset: tcgint;
         regcounter2, firstfpureg: Tsuperregister;
    begin
      usesfpr:=false;
      firstreggpr:=RS_NO;
      firstregfpu:=RS_NO;
      if not (po_assembler in current_procinfo.procdef.procoptions) then
        begin
            { FIXME: has to be R_F14 instad of R_F8 for SYSV-64bit }
            case target_info.abi of
              abi_powerpc_aix,
              abi_powerpc_darwin:
                firstfpureg := RS_F14;
              abi_powerpc_sysv:
                firstfpureg := RS_F9;
              else
                internalerror(2003122903);
            end;
          for regcounter:=firstfpureg to RS_F31 do
           begin
             if regcounter in rg[R_FPUREGISTER].used_in_proc then
              begin
                 usesfpr:=true;
                 firstregfpu:=regcounter;
                 break;
              end;
           end;
        end;
      usesgpr:=false;
      if not (po_assembler in current_procinfo.procdef.procoptions) then
        for regcounter2:=RS_R13 to RS_R31 do
          begin
            if regcounter2 in rg[R_INTREGISTER].used_in_proc then
              begin
                 usesgpr:=true;
                 firstreggpr:=regcounter2;
                 break;
              end;
          end;
      offset:= 0;

      { save floating-point registers }
      if usesfpr then
        for regcounter := firstregfpu to RS_F31 do
          begin
            offset:= offset - 8;
            reference_reset_base(href, NR_STACK_POINTER_REG, offset, 8);
            list.concat(taicpu.op_reg_ref(A_STFD, tregister(regcounter), href));
          end;
        (* Optimiztion in the future:  a_call_name(list,'_savefXX'); *)

      { save gprs in gpr save area }
      if usesgpr then
        if firstreggpr < RS_R30 then
          begin
            offset:= offset - 4 * (RS_R31 - firstreggpr + 1);
            reference_reset_base(href,NR_STACK_POINTER_REG,offset,4);
            list.concat(taicpu.op_reg_ref(A_STMW,tregister(firstreggpr),href));
              {STMW stores multiple registers}
          end
        else
          begin
            for regcounter := firstreggpr to RS_R31 do
              begin
                offset:= offset - 4;
                reference_reset_base(href, NR_STACK_POINTER_REG, offset, 4);
                list.concat(taicpu.op_reg_ref(A_STW, newreg(R_INTREGISTER,regcounter,R_SUBWHOLE), href));
              end;
          end;

      { now comes the AltiVec context save, not yet implemented !!! }

      save_regs:= -offset;
    end;

    procedure tcgppc.restore_regs(list : TAsmList);
    {Generates code which restores used non-volatile registers from
    the save area right below the address the stackpointer point to.}

     var regcounter,firstregfpu,firstreggpr: TSuperRegister;
         usesfpr,usesgpr: boolean;
         href : treference;
         offset: integer;
         regcounter2, firstfpureg: Tsuperregister;

    begin
      usesfpr:=false;
      firstreggpr:=RS_NO;
      firstregfpu:=RS_NO;

      if not (po_assembler in current_procinfo.procdef.procoptions) then
        begin
          { FIXME: has to be R_F14 instad of R_F8 for SYSV-64bit }
          case target_info.abi of
            abi_powerpc_aix,
            abi_powerpc_darwin:
              firstfpureg := RS_F14;
            abi_powerpc_sysv:
              firstfpureg := RS_F9;
            else
              internalerror(2003122903);
          end;
          for regcounter:=firstfpureg to RS_F31 do
           begin
             if regcounter in rg[R_FPUREGISTER].used_in_proc then
              begin
                 usesfpr:=true;
                 firstregfpu:=regcounter;
                 break;
              end;
           end;
        end;

      usesgpr:=false;
      if not (po_assembler in current_procinfo.procdef.procoptions) then
        for regcounter2:=RS_R13 to RS_R31 do
          begin
            if regcounter2 in rg[R_INTREGISTER].used_in_proc then
              begin
                 usesgpr:=true;
                 firstreggpr:=regcounter2;
                 break;
              end;
          end;

      offset:= 0;

      { restore fp registers }
      if usesfpr then
        for regcounter := firstregfpu to RS_F31 do
          begin
            offset:= offset - 8;
            reference_reset_base(href, NR_STACK_POINTER_REG, offset, 8);
            list.concat(taicpu.op_reg_ref(A_LFD, newreg(R_FPUREGISTER,regcounter,R_SUBWHOLE), href));
          end;
        (* Optimiztion in the future: a_call_name(list,'_restfXX'); *)

      { restore gprs }
      if usesgpr then
        if firstreggpr < RS_R30 then
          begin
            offset:= offset - 4 * (RS_R31 - firstreggpr + 1);
            reference_reset_base(href,NR_STACK_POINTER_REG,offset, 4); //-220
            list.concat(taicpu.op_reg_ref(A_LMW,tregister(firstreggpr),href));
              {LMW loads multiple registers}
          end
        else
          begin
            for regcounter := firstreggpr to RS_R31 do
              begin
                offset:= offset - 4;
                reference_reset_base(href, NR_STACK_POINTER_REG, offset, 4);
                list.concat(taicpu.op_reg_ref(A_LWZ, newreg(R_INTREGISTER,regcounter,R_SUBWHOLE), href));
              end;
          end;

      { now comes the AltiVec context restore, not yet implemented !!! }
    end;


    procedure tcgppc.g_stackframe_entry_mac(list : TAsmList;localsize : longint);
 (* NOT IN USE *)

 { generated the entry code of a procedure/function. Note: localsize is the }
 { sum of the size necessary for local variables and the maximum possible   }
 { combined size of ALL the parameters of a procedure called by the current }
 { one                                                                     }

     const
         macosLinkageAreaSize = 24;

     var
         href : treference;
         registerSaveAreaSize : longint;

      begin
        if (localsize mod 8) <> 0 then
          internalerror(58991);
        { CR and LR only have to be saved in case they are modified by the current }
        { procedure, but currently this isn't checked, so save them always         }
        { following is the entry code as described in "Altivec Programming }
        { Interface Manual", bar the saving of AltiVec registers           }
        a_reg_alloc(list,NR_STACK_POINTER_REG);
        a_reg_alloc(list,NR_R0);

        { save return address in callers frame}
        list.concat(taicpu.op_reg_reg(A_MFSPR,NR_R0,NR_LR));
        { ... in caller's frame }
        reference_reset_base(href,NR_STACK_POINTER_REG,8, 8);
        list.concat(taicpu.op_reg_ref(A_STW,NR_R0,href));
        a_reg_dealloc(list,NR_R0);

        { save non-volatile registers in callers frame}
        registerSaveAreaSize:= save_regs(list);

        { save the CR if necessary in callers frame ( !!! always done currently ) }
        a_reg_alloc(list,NR_R0);
        list.concat(taicpu.op_reg_reg(A_MFSPR,NR_R0,NR_CR));
        reference_reset_base(href,NR_STACK_POINTER_REG,LA_CR_AIX,4);
        list.concat(taicpu.op_reg_ref(A_STW,NR_R0,href));
        a_reg_dealloc(list,NR_R0);

        (*
        { save pointer to incoming arguments }
        list.concat(taicpu.op_reg_reg_const(A_ORI,R_31,STACK_POINTER_REG,0));
        *)

        (*
        a_reg_alloc(list,R_12);

        { 0 or 8 based on SP alignment }
        list.concat(taicpu.op_reg_reg_const_const_const(A_RLWINM,
          R_12,STACK_POINTER_REG,0,28,28));
        { add in stack length }
        list.concat(taicpu.op_reg_reg_const(A_SUBFIC,R_12,R_12,
          -localsize));
        { establish new alignment }
        list.concat(taicpu.op_reg_reg_reg(A_STWUX,STACK_POINTER_REG,STACK_POINTER_REG,R_12));

        a_reg_dealloc(list,R_12);
        *)

        { allocate stack frame }
        localsize:= align(localsize + macosLinkageAreaSize + registerSaveAreaSize, 16);
        inc(localsize,tg.lasttemp);
        localsize:=align(localsize,16);
        //tppcprocinfo(current_procinfo).localsize:=localsize;

        if (localsize <> 0) then
          begin
            if (localsize <= high(smallint)) then
              begin
                reference_reset_base(href,NR_STACK_POINTER_REG,-localsize,8);
                a_load_store(list,A_STWU,NR_STACK_POINTER_REG,href);
              end
            else
              begin
                reference_reset_base(href,NR_STACK_POINTER_REG,0,8);
                href.index := NR_R11;
                a_reg_alloc(list,href.index);
                a_load_const_reg(list,OS_S32,-localsize,href.index);
                a_load_store(list,A_STWUX,NR_STACK_POINTER_REG,href);
                a_reg_dealloc(list,href.index);
              end;
          end;
      end;

    procedure tcgppc.g_return_from_proc_mac(list : TAsmList;parasize : tcgint);
 (* NOT IN USE *)

      var
        href : treference;
      begin
        a_reg_alloc(list,NR_R0);

        { restore stack pointer }
        reference_reset_base(href,NR_STACK_POINTER_REG,LA_SP,4);
        list.concat(taicpu.op_reg_ref(A_LWZ,NR_STACK_POINTER_REG,href));
        (*
        list.concat(taicpu.op_reg_reg_const(A_ORI,NR_STACK_POINTER_REG,R_31,0));
        *)

        { restore the CR if necessary from callers frame
            ( !!! always done currently ) }
        reference_reset_base(href,NR_STACK_POINTER_REG,LA_CR_AIX,4);
        list.concat(taicpu.op_reg_ref(A_LWZ,NR_R0,href));
        list.concat(taicpu.op_reg_reg(A_MTSPR,NR_R0,NR_CR));
        a_reg_dealloc(list,NR_R0);

        (*
        { restore return address from callers frame }
        reference_reset_base(href,STACK_POINTER_REG,8);
        list.concat(taicpu.op_reg_ref(A_LWZ,R_0,href));
        *)

        { restore non-volatile registers from callers frame }
        restore_regs(list);

        (*
        { return to caller }
        list.concat(taicpu.op_reg_reg(A_MTSPR,R_0,R_LR));
        list.concat(taicpu.op_none(A_BLR));
        *)

        { restore return address from callers frame }
        reference_reset_base(href,NR_STACK_POINTER_REG,8,8);
        list.concat(taicpu.op_reg_ref(A_LWZ,NR_R0,href));

        { return to caller }
        list.concat(taicpu.op_reg_reg(A_MTSPR,NR_R0,NR_LR));
        list.concat(taicpu.op_none(A_BLR));
      end;


{ ************* concatcopy ************ }

{$ifdef use8byteconcatcopy}
  const
    maxmoveunit = 8;
{$else use8byteconcatcopy}
  const
    maxmoveunit = 4;
{$endif use8byteconcatcopy}

    procedure tcgppc.g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);

      var
        countreg: TRegister;
        src, dst: TReference;
        lab: tasmlabel;
        count, count2: aint;
        size: tcgsize;
        copyreg: tregister;

      begin
{$ifdef extdebug}
        if len > high(longint) then
          internalerror(2002072704);
{$endif extdebug}

        if (references_equal(source,dest)) then
          exit;

        { make sure short loads are handled as optimally as possible }
        if (len <= maxmoveunit) and
           (byte(len) in [1,2,4,8]) then
          begin
            if len < 8 then
              begin
                size := int_cgsize(len);
                a_load_ref_ref(list,size,size,source,dest);
              end
            else
              begin
                copyreg := getfpuregister(list,OS_F64);
                a_loadfpu_ref_reg(list,OS_F64,OS_F64,source,copyreg);
                a_loadfpu_reg_ref(list,OS_F64,OS_F64,copyreg,dest);
              end;
            exit;
          end;

        count := len div maxmoveunit;

        reference_reset(src,source.alignment);
        reference_reset(dst,dest.alignment);
        { load the address of source into src.base }
        if (count > 4) or
           not issimpleref(source) or
           ((source.index <> NR_NO) and
            ((source.offset + longint(len)) > high(smallint))) then
          begin
            src.base := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
            a_loadaddr_ref_reg(list,source,src.base);
          end
        else
          begin
            src := source;
          end;
        { load the address of dest into dst.base }
        if (count > 4) or
           not issimpleref(dest) or
           ((dest.index <> NR_NO) and
            ((dest.offset + longint(len)) > high(smallint))) then
          begin
            dst.base := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
            a_loadaddr_ref_reg(list,dest,dst.base);
          end
        else
          begin
            dst := dest;
          end;

{$ifdef use8byteconcatcopy}
        if count > 4 then
          { generate a loop }
          begin
            { the offsets are zero after the a_loadaddress_ref_reg and just }
            { have to be set to 8. I put an Inc there so debugging may be   }
            { easier (should offset be different from zero here, it will be }
            { easy to notice in the generated assembler                     }
            inc(dst.offset,8);
            inc(src.offset,8);
            list.concat(taicpu.op_reg_reg_const(A_SUBI,src.base,src.base,8));
            list.concat(taicpu.op_reg_reg_const(A_SUBI,dst.base,dst.base,8));
            countreg := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
            a_load_const_reg(list,OS_32,count,countreg);
            copyreg := getfpuregister(list,OS_F64);
            a_reg_sync(list,copyreg);
            current_asmdata.getjumplabel(lab);
            a_label(list, lab);
            list.concat(taicpu.op_reg_reg_const(A_SUBIC_,countreg,countreg,1));
            list.concat(taicpu.op_reg_ref(A_LFDU,copyreg,src));
            list.concat(taicpu.op_reg_ref(A_STFDU,copyreg,dst));
            a_jmp(list,A_BC,C_NE,0,lab);
            a_reg_sync(list,copyreg);
            len := len mod 8;
          end;

        count := len div 8;
        if count > 0 then
          { unrolled loop }
          begin
            copyreg := getfpuregister(list,OS_F64);
            for count2 := 1 to count do
              begin
                a_loadfpu_ref_reg(list,OS_F64,OS_F64,src,copyreg);
                a_loadfpu_reg_ref(list,OS_F64,OS_F64,copyreg,dst);
                inc(src.offset,8);
                inc(dst.offset,8);
              end;
            len := len mod 8;
          end;

        if (len and 4) <> 0 then
          begin
            a_reg_alloc(list,NR_R0);
            a_load_ref_reg(list,OS_32,OS_32,src,NR_R0);
            a_load_reg_ref(list,OS_32,OS_32,NR_R0,dst);
            inc(src.offset,4);
            inc(dst.offset,4);
            a_reg_dealloc(list,NR_R0);
          end;
{$else use8byteconcatcopy}
        if count > 4 then
          { generate a loop }
          begin
            { the offsets are zero after the a_loadaddress_ref_reg and just }
            { have to be set to 4. I put an Inc there so debugging may be   }
            { easier (should offset be different from zero here, it will be }
            { easy to notice in the generated assembler                     }
            inc(dst.offset,4);
            inc(src.offset,4);
            list.concat(taicpu.op_reg_reg_const(A_SUBI,src.base,src.base,4));
            list.concat(taicpu.op_reg_reg_const(A_SUBI,dst.base,dst.base,4));
            countreg := rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
            a_load_const_reg(list,OS_32,count,countreg);
            { explicitely allocate R_0 since it can be used safely here }
            { (for holding date that's being copied)                    }
            a_reg_alloc(list,NR_R0);
            current_asmdata.getjumplabel(lab);
            a_label(list, lab);
            list.concat(taicpu.op_reg_reg_const(A_SUBIC_,countreg,countreg,1));
            list.concat(taicpu.op_reg_ref(A_LWZU,NR_R0,src));
            list.concat(taicpu.op_reg_ref(A_STWU,NR_R0,dst));
            a_jmp(list,A_BC,C_NE,0,lab);
            a_reg_dealloc(list,NR_R0);
            len := len mod 4;
          end;

        count := len div 4;
        if count > 0 then
          { unrolled loop }
          begin
            a_reg_alloc(list,NR_R0);
            for count2 := 1 to count do
              begin
                a_load_ref_reg(list,OS_32,OS_32,src,NR_R0);
                a_load_reg_ref(list,OS_32,OS_32,NR_R0,dst);
                inc(src.offset,4);
                inc(dst.offset,4);
              end;
            a_reg_dealloc(list,NR_R0);
            len := len mod 4;
          end;
{$endif use8byteconcatcopy}
       { copy the leftovers }
       if (len and 2) <> 0 then
         begin
           a_reg_alloc(list,NR_R0);
           a_load_ref_reg(list,OS_16,OS_16,src,NR_R0);
           a_load_reg_ref(list,OS_16,OS_16,NR_R0,dst);
           inc(src.offset,2);
           inc(dst.offset,2);
           a_reg_dealloc(list,NR_R0);
         end;
       if (len and 1) <> 0 then
         begin
           a_reg_alloc(list,NR_R0);
           a_load_ref_reg(list,OS_8,OS_8,src,NR_R0);
           a_load_reg_ref(list,OS_8,OS_8,NR_R0,dst);
           a_reg_dealloc(list,NR_R0);
         end;
      end;


{***************** This is private property, keep out! :) *****************}

    function tcgppc.issimpleref(const ref: treference): boolean;

      begin
        if (ref.base = NR_NO) and
           (ref.index <> NR_NO) then
          internalerror(200208101);
        result :=
          not(assigned(ref.symbol)) and
          (((ref.index = NR_NO) and
            (ref.offset >= low(smallint)) and
            (ref.offset <= high(smallint))) or
           ((ref.index <> NR_NO) and
            (ref.offset = 0)));
      end;


    { find out whether a is of the form 11..00..11b or 00..11...00. If }
    { that's the case, we can use rlwinm to do an AND operation        }
    function tcgppc.get_rlwi_const(a: aint; var l1, l2: longint): boolean;

      var
        temp : longint;
        testbit : aint;
        compare: boolean;

      begin
        get_rlwi_const := false;
        if (a = 0) or (a = -1) then
          exit;
        { start with the lowest bit }
        testbit := 1;
        { check its value }
        compare := boolean(a and testbit);
        { find out how long the run of bits with this value is            }
        { (it's impossible that all bits are 1 or 0, because in that case }
        { this function wouldn't have been called)                        }
        l1 := 31;
        while (((a and testbit) <> 0) = compare) do
          begin
            testbit := testbit shl 1;
            dec(l1);
          end;

        { check the length of the run of bits that comes next }
        compare := not compare;
        l2 := l1;
        while (((a and testbit) <> 0) = compare) and
               (l2 >= 0) do
          begin
            testbit := testbit shl 1;
            dec(l2);
          end;

        { and finally the check whether the rest of the bits all have the }
        { same value                                                      }
        compare := not compare;
        temp := l2;
        if temp >= 0 then
          if (a shr (31-temp)) <> ((-ord(compare)) shr (31-temp)) then
            exit;

        { we have done "not(not(compare))", so compare is back to its   }
        { initial value. If the lowest bit was 0, a is of the form      }
        { 00..11..00 and we need "rlwinm reg,reg,0,l2+1,l1", (+1        }
        { because l2 now contains the position of the last zero of the  }
        { first run instead of that of the first 1) so switch l1 and l2 }
        { in that case (we will generate "rlwinm reg,reg,0,l1,l2")      }
        if not compare then
          begin
            temp := l1;
            l1 := l2+1;
            l2 := temp;
          end
        else
          { otherwise, l1 currently contains the position of the last   }
          { zero instead of that of the first 1 of the second run -> +1 }
          inc(l1);
        { the following is the same as "if l1 = -1 then l1 := 31;" }
        l1 := l1 and 31;
        l2 := l2 and 31;
        get_rlwi_const := true;
      end;


    procedure tcg64fppc.a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);
      begin
        case op of
          OP_NOT:
            begin
              cg.a_op_reg_reg(list,OP_NOT,OS_32,regsrc.reglo,regdst.reglo);
              cg.a_op_reg_reg(list,OP_NOT,OS_32,regsrc.reghi,regdst.reghi);
            end;
          OP_NEG:
            begin
              list.concat(taicpu.op_reg_reg_const(a_subfic,regdst.reglo,regsrc.reglo,0));
              list.concat(taicpu.op_reg_reg(a_subfze,regdst.reghi,regsrc.reghi));
            end;
          else
            a_op64_reg_reg_reg(list,op,size,regsrc,regdst,regdst);
        end;
      end;


    procedure tcg64fppc.a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);
      begin
        a_op64_const_reg_reg(list,op,size,value,reg,reg);
      end;


    procedure tcg64fppc.a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);
      begin
        case op of
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_reg_reg_reg(list,op,OS_32,regsrc1.reglo,regsrc2.reglo,regdst.reglo);
              cg.a_op_reg_reg_reg(list,op,OS_32,regsrc1.reghi,regsrc2.reghi,regdst.reghi);
            end;
          OP_ADD:
            begin
              list.concat(taicpu.op_reg_reg_reg(A_ADDC,regdst.reglo,regsrc1.reglo,regsrc2.reglo));
              list.concat(taicpu.op_reg_reg_reg(A_ADDE,regdst.reghi,regsrc1.reghi,regsrc2.reghi));
            end;
          OP_SUB:
            begin
              list.concat(taicpu.op_reg_reg_reg(A_SUBC,regdst.reglo,regsrc2.reglo,regsrc1.reglo));
              list.concat(taicpu.op_reg_reg_reg(A_SUBFE,regdst.reghi,regsrc1.reghi,regsrc2.reghi));
            end;
          else
            internalerror(2002072801);
        end;
      end;


    procedure tcg64fppc.a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64);

      const
        ops: array[boolean,1..3] of tasmop = ((A_ADDIC,A_ADDC,A_ADDZE),
                                              (A_SUBIC,A_SUBC,A_ADDME));
      var
        tmpreg: tregister;
        tmpreg64: tregister64;
        issub: boolean;
      begin
        case op of
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_const_reg_reg(list,op,OS_32,aint(value),regsrc.reglo,regdst.reglo);
              cg.a_op_const_reg_reg(list,op,OS_32,aint(value shr 32),regsrc.reghi,
                regdst.reghi);
            end;
          OP_ADD, OP_SUB:
            begin
              if (value < 0) and
                 (value <> low(value)) then
                begin
                  if op = OP_ADD then
                    op := OP_SUB
                  else
                    op := OP_ADD;
                  value := -value;
                end;
              if (longint(value) <> 0) then
                begin
                  issub := op = OP_SUB;
                  if (value > 0) and
                     (value-ord(issub) <= 32767) then
                    begin
                      list.concat(taicpu.op_reg_reg_const(ops[issub,1],
                        regdst.reglo,regsrc.reglo,longint(value)));
                      list.concat(taicpu.op_reg_reg(ops[issub,3],
                        regdst.reghi,regsrc.reghi));
                    end
                  else if ((value shr 32) = 0) then
                    begin
                      tmpreg := tcgppc(cg).rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
                      cg.a_load_const_reg(list,OS_32,aint(value),tmpreg);
                      list.concat(taicpu.op_reg_reg_reg(ops[issub,2],
                        regdst.reglo,regsrc.reglo,tmpreg));
                      list.concat(taicpu.op_reg_reg(ops[issub,3],
                        regdst.reghi,regsrc.reghi));
                    end
                  else
                    begin
                      tmpreg64.reglo := tcgppc(cg).rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
                      tmpreg64.reghi := tcgppc(cg).rg[R_INTREGISTER].getregister(list,R_SUBWHOLE);
                      a_load64_const_reg(list,value,tmpreg64);
                      a_op64_reg_reg_reg(list,op,size,tmpreg64,regsrc,regdst);
                    end
                end
              else
                begin
                  cg.a_load_reg_reg(list,OS_INT,OS_INT,regsrc.reglo,regdst.reglo);
                  cg.a_op_const_reg_reg(list,op,OS_32,aint(value shr 32),regsrc.reghi,
                    regdst.reghi);
                end;
            end;
          else
            internalerror(2002072802);
        end;
      end;


    procedure create_codegen;
      begin
        cg := tcgppc.create;
        cg64 :=tcg64fppc.create;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
      
>>>>>>> graemeg/cpstrnew
=======
      
>>>>>>> graemeg/cpstrnew
=======
      
>>>>>>> graemeg/cpstrnew
=======
      
>>>>>>> origin/cpstrnew
end.
