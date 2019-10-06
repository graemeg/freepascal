{

    Copyright (c) 2008 by Florian Klaempfl
    Member of the Free Pascal development team

    This unit implements the code generator for the AVR

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
       cgbase,cgutils,cgobj,
       aasmbase,aasmcpu,aasmtai,aasmdata,
       parabase,
       cpubase,cpuinfo,node,cg64f32,rgcpu;

    type

      { tcgavr }

      tcgavr = class(tcg)
        { true, if the next arithmetic operation should modify the flags }
        cgsetflags : boolean;
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        function getaddressregister(list:TAsmList):TRegister;override;

        function GetHigh(const r : TRegister) : TRegister;inline;
        function GetOffsetReg(const r: TRegister;ofs : shortint): TRegister;override;
        function GetOffsetReg64(const r,rhi: TRegister;ofs : shortint): TRegister;override;

        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const paraloc : TCGPara);override;
        procedure a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const paraloc : TCGPara);override;
        procedure a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const paraloc : TCGPara);override;
        procedure a_load_reg_cgpara(list : TAsmList; size : tcgsize;r : tregister; const cgpara : tcgpara);override;

        procedure a_call_name(list : TAsmList;const s : string; weak: boolean);override;
        procedure a_call_reg(list : TAsmList;reg: tregister);override;

        procedure a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister); override;
        procedure a_op_reg_reg(list: TAsmList; Op: TOpCG; size: TCGSize; src, dst : TRegister); override;
        procedure a_op_const_reg_reg(list : TAsmList;op : TOpCg;size : tcgsize; a : tcgint;src,dst : tregister); override;

        procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister; setflags: boolean; var ovloc: tlocation); override;
        procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister; setflags: boolean; var ovloc: tlocation); override;

        { move instructions }
        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);override;
        procedure a_load_reg_ref(list : TAsmList; fromsize, tosize: tcgsize; reg : tregister;const ref : treference);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;
        procedure a_load_reg_reg(list : TAsmList; fromsize, tosize : tcgsize;reg1,reg2 : tregister);override;

        { fpu move instructions }
        procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister); override;
        procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister); override;
        procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference); override;

        {  comparison operations }
        procedure a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister;
          l : tasmlabel);override;
        procedure a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel); override;

        procedure a_jmp_name(list : TAsmList;const s : string); override;
        procedure a_jmp_always(list : TAsmList;l: tasmlabel); override;
        procedure a_jmp_flags(list : TAsmList;const f : TResFlags;l: tasmlabel); override;

        procedure g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister); override;

        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean); override;

        procedure a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);override;

        procedure g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);override;
        procedure g_concatcopy_move(list : TAsmList;const source,dest : treference;len : tcgint);

        procedure g_overflowcheck(list: TAsmList; const l: tlocation; def: tdef); override;
        procedure g_overflowCheck_loc(List: TAsmList; const Loc: TLocation; def: TDef; ovloc: tlocation); override;

        procedure g_save_registers(list : TAsmList);override;
        procedure g_restore_registers(list : TAsmList);override;

        procedure a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);
        procedure fixref(list : TAsmList;var ref : treference);
        function normalize_ref(list : TAsmList;ref : treference;
          tmpreg : tregister) : treference;

        procedure emit_mov(list: TAsmList;reg2: tregister; reg1: tregister);

        procedure a_adjust_sp(list: TAsmList; value: longint);
        function GetLoad(const ref : treference) : tasmop;
        function GetStore(const ref: treference): tasmop;

        procedure gen_multiply(list: TAsmList; op: topcg; size: TCgSize; src2, src1, dst: tregister; check_overflow: boolean; var ovloc: tlocation);

      protected
        procedure a_op_reg_reg_internal(list: TAsmList; Op: TOpCG; size: TCGSize; src, srchi, dst, dsthi: TRegister);
        procedure a_op_const_reg_internal(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg, reghi: TRegister);
        procedure maybegetcpuregister(list : tasmlist; reg : tregister);
      end;

      tcg64favr = class(tcg64f32)
        procedure a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);override;
        procedure a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);override;
      end;

    procedure create_codegen;

    const
      TOpCG2AsmOp: Array[topcg] of TAsmOp = (A_NONE,A_MOV,A_ADD,A_AND,A_NONE,
                            A_NONE,A_MULS,A_MUL,A_NEG,A_COM,A_OR,
                            A_ASR,A_LSL,A_LSR,A_SUB,A_EOR,A_ROL,A_ROR);
  implementation

    uses
       globals,verbose,systems,cutils,
       fmodule,
       symconst,symsym,symtable,
       tgobj,rgobj,
       procinfo,cpupi,
       paramgr;


    procedure tcgavr.init_register_allocators;
      begin
        inherited init_register_allocators;
        rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
            [RS_R18,RS_R19,RS_R20,RS_R21,RS_R22,RS_R23,RS_R24,RS_R25,
             RS_R2,RS_R3,RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,RS_R9,
             RS_R10,RS_R11,RS_R12,RS_R13,RS_R14,RS_R15,RS_R16,RS_R17],first_int_imreg,[]);
      end;


    procedure tcgavr.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        // rg[R_ADDRESSREGISTER].free;
        inherited done_register_allocators;
      end;


    function tcgavr.getaddressregister(list: TAsmList): TRegister;
      begin
       Result:=getintregister(list,OS_ADDR);
      end;


    function tcgavr.GetHigh(const r : TRegister) : TRegister;
      begin
        result:=GetNextReg(r);
      end;


    function tcgavr.GetOffsetReg(const r: TRegister;ofs : shortint): TRegister;
      begin
        result:=TRegister(longint(r)+ofs);
      end;


    function tcgavr.GetOffsetReg64(const r,rhi: TRegister;ofs : shortint): TRegister;
      begin
        if ofs>3 then
          result:=TRegister(longint(rhi)+ofs-4)
        else
          result:=TRegister(longint(r)+ofs);
      end;


    procedure tcgavr.a_load_reg_cgpara(list : TAsmList;size : tcgsize;r : tregister;const cgpara : tcgpara);

      procedure load_para_loc(r : TRegister;paraloc : PCGParaLocation);
        var
          ref : treference;
        begin
          paramanager.allocparaloc(list,paraloc);
          case paraloc^.loc of
             LOC_REGISTER,LOC_CREGISTER:
               a_load_reg_reg(list,paraloc^.size,paraloc^.size,r,paraloc^.register);
             LOC_REFERENCE,LOC_CREFERENCE:
               begin
                  reference_reset_base(ref,paraloc^.reference.index,paraloc^.reference.offset,ctempposinvalid,2,[]);
                  a_load_reg_ref(list,paraloc^.size,paraloc^.size,r,ref);
               end;
             else
               internalerror(2002071004);
          end;
        end;

      var
        i, i2 : longint;
        hp : PCGParaLocation;

      begin
{        if use_push(cgpara) then
          begin
            if tcgsize2size[cgpara.Size] > 2 then
              begin
                if tcgsize2size[cgpara.Size] <> 4 then
                  internalerror(2013031101);
                if cgpara.location^.Next = nil then
                  begin
                    if tcgsize2size[cgpara.location^.size] <> 4 then
                      internalerror(2013031101);
                  end
                else
                  begin
                    if tcgsize2size[cgpara.location^.size] <> 2 then
                      internalerror(2013031101);
                    if tcgsize2size[cgpara.location^.Next^.size] <> 2 then
                      internalerror(2013031101);
                    if cgpara.location^.Next^.Next <> nil then
                      internalerror(2013031101);
                  end;

                if tcgsize2size[cgpara.size]>cgpara.alignment then
                  pushsize:=cgpara.size
                else
                  pushsize:=int_cgsize(cgpara.alignment);
                pushsize2 := int_cgsize(tcgsize2size[pushsize] - 2);
                list.concat(taicpu.op_reg(A_PUSH,TCgsize2opsize[pushsize2],makeregsize(list,GetNextReg(r),pushsize2)));
                list.concat(taicpu.op_reg(A_PUSH,S_W,makeregsize(list,r,OS_16)));
              end
            else
              begin
                cgpara.check_simple_location;
                if tcgsize2size[cgpara.location^.size]>cgpara.alignment then
                  pushsize:=cgpara.location^.size
                else
                  pushsize:=int_cgsize(cgpara.alignment);
                list.concat(taicpu.op_reg(A_PUSH,TCgsize2opsize[pushsize],makeregsize(list,r,pushsize)));
              end;

          end
        else }
          begin
            if not(tcgsize2size[cgpara.Size] in [1..4]) then
              internalerror(2014011101);

            hp:=cgpara.location;

            i:=0;
            while i<tcgsize2size[cgpara.Size] do
              begin
                if not(assigned(hp)) then
                  internalerror(2014011102);

                inc(i, tcgsize2size[hp^.Size]);

                if hp^.Loc=LOC_REGISTER then
                  begin
                    load_para_loc(r,hp);
                    hp:=hp^.Next;
                    { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
                    if i<tcgsize2size[cgpara.Size] then
                      r:=GetNextReg(r);
                  end
                else
                  begin
                    load_para_loc(r,hp);

                    if i<tcgsize2size[cgpara.Size] then
                      for i2:=1 to tcgsize2size[hp^.Size] do
                        r:=GetNextReg(r);

                    hp:=hp^.Next;
                  end;
              end;
            if assigned(hp) then
              internalerror(2014011103);
          end;
      end;


    procedure tcgavr.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const paraloc : TCGPara);
      var
        i : longint;
        hp : PCGParaLocation;
        ref: treference;
      begin
        if not(tcgsize2size[paraloc.Size] in [1..4]) then
          internalerror(2014011101);

        hp:=paraloc.location;

        i:=1;
        while i<=tcgsize2size[paraloc.Size] do
          begin
            if not(assigned(hp)) then
              internalerror(2014011105);
            paramanager.allocparaloc(list,hp);
            case hp^.loc of
              LOC_REGISTER,LOC_CREGISTER:
                begin
                  if (tcgsize2size[hp^.size]<>1) or
                    (hp^.shiftval<>0) then
                    internalerror(2015041101);
                  a_load_const_reg(list,hp^.size,(a shr (8*(i-1))) and $ff,hp^.register);

                  inc(i,tcgsize2size[hp^.size]);
                  hp:=hp^.Next;
                end;
              LOC_REFERENCE,LOC_CREFERENCE:
                begin
                  reference_reset(ref,paraloc.alignment,[]);
                  ref.base:=hp^.reference.index;
                  ref.offset:=hp^.reference.offset;
                  a_load_const_ref(list,hp^.size,a shr (8*(i-1)),ref);

                  inc(i,tcgsize2size[hp^.size]);
                  hp:=hp^.Next;
                end;
              else
                internalerror(2002071004);
            end;
          end;
      end;


    procedure tcgavr.a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const paraloc : TCGPara);
      var
        tmpref, ref: treference;
        location: pcgparalocation;
        sizeleft: tcgint;
      begin
        location := paraloc.location;
        tmpref := r;
        sizeleft := paraloc.intsize;
        while assigned(location) do
          begin
            paramanager.allocparaloc(list,location);
            case location^.loc of
              LOC_REGISTER,LOC_CREGISTER:
                a_load_ref_reg(list,location^.size,location^.size,tmpref,location^.register);
              LOC_REFERENCE:
                begin
                  reference_reset_base(ref,location^.reference.index,location^.reference.offset,ctempposinvalid,paraloc.alignment,[]);
                  { doubles in softemu mode have a strange order of registers and references }
                  if location^.size=OS_32 then
                    g_concatcopy(list,tmpref,ref,4)
                  else
                    begin
                      g_concatcopy(list,tmpref,ref,sizeleft);
                      if assigned(location^.next) then
                        internalerror(2005010710);
                    end;
                end;
              LOC_VOID:
                begin
                  // nothing to do
                end;
              else
                internalerror(2002081103);
            end;
            inc(tmpref.offset,tcgsize2size[location^.size]);
            dec(sizeleft,tcgsize2size[location^.size]);
            location := location^.next;
          end;
      end;


    procedure tcgavr.a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const paraloc : TCGPara);
      var
        tmpreg: tregister;
      begin
        tmpreg:=getaddressregister(list);
        a_loadaddr_ref_reg(list,r,tmpreg);
        a_load_reg_cgpara(list,OS_ADDR,tmpreg,paraloc);
      end;


    procedure tcgavr.a_call_name(list : TAsmList;const s : string; weak: boolean);
      var
        sym: TAsmSymbol;
      begin
        if weak then
          sym:=current_asmdata.WeakRefAsmSymbol(s,AT_FUNCTION)
        else
          sym:=current_asmdata.RefAsmSymbol(s,AT_FUNCTION);

        if CPUAVR_HAS_JMP_CALL in cpu_capabilities[current_settings.cputype] then
          list.concat(taicpu.op_sym(A_CALL,sym))
        else
          list.concat(taicpu.op_sym(A_RCALL,sym));

        include(current_procinfo.flags,pi_do_call);
      end;


    procedure tcgavr.a_call_reg(list : TAsmList;reg: tregister);
      begin
        a_reg_alloc(list,NR_ZLO);
        emit_mov(list,NR_ZLO,reg);
        a_reg_alloc(list,NR_ZHI);
        emit_mov(list,NR_ZHI,GetHigh(reg));
        list.concat(taicpu.op_none(A_ICALL));
        a_reg_dealloc(list,NR_ZHI);
        a_reg_dealloc(list,NR_ZLO);

        include(current_procinfo.flags,pi_do_call);
      end;


     procedure tcgavr.a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister);
       begin
         if not(size in [OS_S8,OS_8,OS_S16,OS_16,OS_S32,OS_32]) then
           internalerror(2012102403);
         a_op_const_reg_internal(list,Op,size,a,reg,NR_NO);
       end;


     procedure tcgavr.a_op_reg_reg(list: TAsmList; Op: TOpCG; size: TCGSize; src, dst : TRegister);
       begin
         if not(size in [OS_S8,OS_8,OS_S16,OS_16,OS_S32,OS_32]) then
           internalerror(2012102401);
         a_op_reg_reg_internal(list,Op,size,src,NR_NO,dst,NR_NO);
       end;


     procedure tcgavr.a_op_const_reg_reg(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister);
       var
         tmpSrc, tmpDst, countreg: TRegister;
         b, b2, i, j: byte;
         s1, s2, t1: integer;
         l1: TAsmLabel;
       begin
         if (op in [OP_MUL,OP_IMUL]) and (size in [OS_16,OS_S16]) and (a in [2,4,8]) then
           begin
             emit_mov(list,dst,src);
             emit_mov(list,GetNextReg(dst),GetNextReg(src));
             a:=a shr 1;
             while a>0 do
               begin
                 list.concat(taicpu.op_reg(A_LSL,dst));
                 list.concat(taicpu.op_reg(A_ROL,GetNextReg(dst)));
                 a:=a shr 1;
               end;
           end

         else if (op in [OP_SHL,OP_SHR]) and
           { a=0 get eliminated later by tcg.optimize_op_const }
           (a>0)  then
           begin
             { number of bytes to shift }
             b:=a div 8;

             { Ensure that b is never larger than base type }
             if b>tcgsize2size[size] then
               begin
                 b:=tcgsize2size[size];
                 b2:=0;
               end
             else
               b2:=a mod 8;

             if b < tcgsize2size[size] then
               { copy from src to dst accounting for shift offset }
               for i:=0 to (tcgsize2size[size]-b-1) do
                 if op=OP_SHL then
                   a_load_reg_reg(list,OS_8,OS_8,
                     GetOffsetReg64(src,NR_NO,i),
                     GetOffsetReg64(dst,NR_NO,i+b))
                 else
                   a_load_reg_reg(list,OS_8,OS_8,
                     GetOffsetReg64(src,NR_NO,i+b),
                     GetOffsetReg64(dst,NR_NO,i));

             { remaining bit shifts }
             if b2 > 0 then
               begin
                 { Cost of loop }
                 s1:=3+tcgsize2size[size]-b;
                 t1:=b2*(tcgsize2size[size]-b+3);
                 { Cost of loop unrolling,t2=s2 }
                 s2:=b2*(tcgsize2size[size]-b);

                 if ((cs_opt_size in current_settings.optimizerswitches) and (s1<s2)) or
                    (((s2-s1)-t1/s2)>0) then
                   begin
                     { Shift non-moved bytes in loop }
                     current_asmdata.getjumplabel(l1);
                     countreg:=getintregister(list,OS_8);
                     a_load_const_reg(list,OS_8,b2,countreg);
                     cg.a_label(list,l1);
                     if op=OP_SHL then
                       list.concat(taicpu.op_reg(A_LSL,GetOffsetReg64(dst,NR_NO,b)))
                     else
                       list.concat(taicpu.op_reg(A_LSR,GetOffsetReg64(dst,NR_NO,tcgsize2size[size]-1-b)));

                     if size in [OS_S16,OS_16,OS_S32,OS_32,OS_S64,OS_64] then
                       begin
                         for i:=2+b to tcgsize2size[size] do
                           if op=OP_SHL then
                             list.concat(taicpu.op_reg(A_ROL,GetOffsetReg64(dst,NR_NO,i-1)))
                           else
                             list.concat(taicpu.op_reg(A_ROR,GetOffsetReg64(dst,NR_NO,tcgsize2size[size]-i)));
                       end;
                     list.concat(taicpu.op_reg(A_DEC,countreg));
                     a_jmp_flags(list,F_NE,l1);
                     { keep registers alive }
                     a_reg_sync(list,countreg);
                   end
                 else
                   begin
                     { Unroll shift loop over non-moved bytes }
                     for j:=1 to b2 do
                     begin
                       if op=OP_SHL then
                         list.concat(taicpu.op_reg(A_LSL,
                         GetOffsetReg64(dst,NR_NO,b)))
                       else
                         list.concat(taicpu.op_reg(A_LSR,
                           GetOffsetReg64(dst,NR_NO,tcgsize2size[size]-b-1)));

                       if not(size in [OS_8,OS_S8]) then
                         for i:=2 to tcgsize2size[size]-b do
                           if op=OP_SHL then
                             list.concat(taicpu.op_reg(A_ROL,
                               GetOffsetReg64(dst,NR_NO,b+i-1)))
                           else
                             list.concat(taicpu.op_reg(A_ROR,
                               GetOffsetReg64(dst,NR_NO,tcgsize2size[size]-b-i)));
                     end;
                   end;
               end;

               { fill skipped destination registers with 0
                 Do last,then optimizer can optimize register moves }
               for i:=1 to b do
                 if op=OP_SHL then
                   emit_mov(list,GetOffsetReg64(dst,NR_NO,i-1),NR_R1)
                 else
                   emit_mov(list,GetOffsetReg64(dst,NR_NO,tcgsize2size[size]-i),NR_R1);
           end
         else
           inherited a_op_const_reg_reg(list,op,size,a,src,dst);
       end;

     procedure tcgavr.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister; setflags: boolean; var ovloc: tlocation);
       var
         tmpreg: TRegister;
       begin
         if (op in [OP_MUL,OP_IMUL]) and
            setflags then
           begin
             tmpreg:=getintregister(list,size);
             a_load_const_reg(list,size,a,tmpreg);
             a_op_reg_reg_reg_checkoverflow(list,op,size,tmpreg,src,dst,setflags,ovloc);
           end
         else
           begin
             inherited a_op_const_reg_reg_checkoverflow(list, op, size, a, src, dst, setflags, ovloc);
             ovloc.loc:=LOC_FLAGS;
           end;
       end;

     procedure tcgavr.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister; setflags: boolean; var ovloc: tlocation);
       begin
         if (op in [OP_MUL,OP_IMUL]) and
            setflags then
           gen_multiply(list,op,size,src1,src2,dst,setflags,ovloc)
         else
           begin
             inherited a_op_reg_reg_reg_checkoverflow(list, op, size, src1, src2, dst, setflags, ovloc);
             ovloc.loc:=LOC_FLAGS;
           end;
       end;


     procedure tcgavr.a_op_reg_reg_internal(list : TAsmList; Op: TOpCG; size: TCGSize; src, srchi, dst, dsthi: TRegister);
       var
         countreg,
         tmpreg: tregister;
         i : integer;
         instr : taicpu;
         paraloc1,paraloc2 : TCGPara;
         l1,l2 : tasmlabel;
         pd : tprocdef;
         hovloc: tlocation;

      { NextRegDst* is sometimes called before the register usage and sometimes afterwards }
       procedure NextSrcDstPreInc;
         begin
           if i=5 then
             begin
               dst:=dsthi;
               src:=srchi;
             end
           else
             begin
               dst:=GetNextReg(dst);
               src:=GetNextReg(src);
             end;
         end;

       procedure NextSrcDstPostInc;
         begin
           if i=4 then
             begin
               dst:=dsthi;
               src:=srchi;
             end
           else
             begin
               dst:=GetNextReg(dst);
               src:=GetNextReg(src);
             end;
         end;

       { iterates TmpReg through all registers of dst }
       procedure NextTmp;
         begin
           if i=4 then
             tmpreg:=dsthi
           else
             tmpreg:=GetNextReg(tmpreg);
         end;

      begin
         case op of
           OP_ADD:
             begin
               list.concat(taicpu.op_reg_reg(A_ADD,dst,src));
               for i:=2 to tcgsize2size[size] do
                 begin
                   NextSrcDstPreInc;
                   list.concat(taicpu.op_reg_reg(A_ADC,dst,src));
                 end;
             end;

           OP_SUB:
             begin
               list.concat(taicpu.op_reg_reg(A_SUB,dst,src));
               for i:=2 to tcgsize2size[size] do
                 begin
                   NextSrcDstPreInc;
                   list.concat(taicpu.op_reg_reg(A_SBC,dst,src));
                 end;
             end;

           OP_NEG:
             begin
               if src<>dst then
                 begin
                   if size in [OS_S64,OS_64] then
                     begin
                       a_load_reg_reg(list,OS_32,OS_32,src,dst);
                       a_load_reg_reg(list,OS_32,OS_32,srchi,dsthi);
                     end
                   else
                     a_load_reg_reg(list,size,size,src,dst);
                 end;

               if size in [OS_S16,OS_16,OS_S32,OS_32,OS_S64,OS_64] then
                 begin
                   tmpreg:=GetNextReg(dst);
                   for i:=2 to tcgsize2size[size] do
                     begin
                       list.concat(taicpu.op_reg(A_COM,tmpreg));
                       { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
                       if i<tcgsize2size[size] then
                         NextTmp;
                     end;
                   list.concat(taicpu.op_reg(A_NEG,dst));
                   tmpreg:=GetNextReg(dst);
                   for i:=2 to tcgsize2size[size] do
                     begin
                       list.concat(taicpu.op_reg_const(A_SBCI,tmpreg,-1));
                       { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
                       if i<tcgsize2size[size] then
                         NextTmp;
                   end;
                 end
               else if size in [OS_S8,OS_8] then
                 list.concat(taicpu.op_reg(A_NEG,dst))
               else
                 Internalerror(2018030401);
             end;

           OP_NOT:
             begin
               for i:=1 to tcgsize2size[size] do
                 begin
                   if src<>dst then
                     a_load_reg_reg(list,OS_8,OS_8,src,dst);
                   list.concat(taicpu.op_reg(A_COM,dst));
                   { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
                   if i<tcgsize2size[size] then
                     NextSrcDstPostInc;
                 end;
             end;

           OP_MUL,OP_IMUL:
             begin
               tmpreg:=dst;
               if size in [OS_16,OS_S16] then
                 begin
                   tmpreg:=getintregister(list,size);
                   a_load_reg_reg(list,size,size,dst,tmpreg);
                 end;
               gen_multiply(list,op,size,src,tmpreg,dst,false,hovloc);
             end;

           OP_DIV,OP_IDIV:
             { special stuff, needs separate handling inside code }
             { generator                                          }
             internalerror(2011022001);

           OP_SHR,OP_SHL,OP_SAR,OP_ROL,OP_ROR:
             begin
               current_asmdata.getjumplabel(l1);
               current_asmdata.getjumplabel(l2);
               countreg:=getintregister(list,OS_8);
               a_load_reg_reg(list,size,OS_8,src,countreg);
               list.concat(taicpu.op_reg(A_TST,countreg));
               a_jmp_flags(list,F_EQ,l2);
               cg.a_label(list,l1);
               case op of
                 OP_SHR:
                   list.concat(taicpu.op_reg(A_LSR,GetOffsetReg64(dst,dsthi,tcgsize2size[size]-1)));
                 OP_SHL:
                   list.concat(taicpu.op_reg(A_LSL,dst));
                 OP_SAR:
                   list.concat(taicpu.op_reg(A_ASR,GetOffsetReg64(dst,dsthi,tcgsize2size[size]-1)));
                 OP_ROR:
                   begin
                     { load carry? }
                     if not(size in [OS_8,OS_S8]) then
                       begin
                         list.concat(taicpu.op_none(A_CLC));
                         list.concat(taicpu.op_reg_const(A_SBRC,src,0));
                         list.concat(taicpu.op_none(A_SEC));
                       end;
                     list.concat(taicpu.op_reg(A_ROR,GetOffsetReg64(dst,dsthi,tcgsize2size[size]-1)));
                   end;
                 OP_ROL:
                   begin
                     { load carry? }
                     if not(size in [OS_8,OS_S8]) then
                       begin
                         list.concat(taicpu.op_none(A_CLC));
                         list.concat(taicpu.op_reg_const(A_SBRC,GetOffsetReg64(dst,dsthi,tcgsize2size[size]-1),7));
                         list.concat(taicpu.op_none(A_SEC));
                       end;
                     list.concat(taicpu.op_reg(A_ROL,dst))
                   end;
                 else
                   internalerror(2011030901);
               end;
               if size in [OS_S16,OS_16,OS_S32,OS_32,OS_S64,OS_64] then
                 begin
                   for i:=2 to tcgsize2size[size] do
                     begin
                       case op of
                         OP_ROR,
                         OP_SHR:
                           list.concat(taicpu.op_reg(A_ROR,GetOffsetReg64(dst,dsthi,tcgsize2size[size]-i)));
                         OP_ROL,
                         OP_SHL:
                           list.concat(taicpu.op_reg(A_ROL,GetOffsetReg64(dst,dsthi,i-1)));
                         OP_SAR:
                           list.concat(taicpu.op_reg(A_ROR,GetOffsetReg64(dst,dsthi,tcgsize2size[size]-i)));
                         else
                           internalerror(2011030902);
                       end;
                   end;
                 end;

               list.concat(taicpu.op_reg(A_DEC,countreg));
               a_jmp_flags(list,F_NE,l1);
               { keep registers alive }
               a_reg_sync(list,countreg);
               cg.a_label(list,l2);
             end;

           OP_AND,OP_OR,OP_XOR:
             begin
                for i:=1 to tcgsize2size[size] do
                  begin
                    list.concat(taicpu.op_reg_reg(topcg2asmop[op],dst,src));
                    { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
                    if i<tcgsize2size[size] then
                      NextSrcDstPostInc;
                  end;
             end;
           else
             internalerror(2011022004);
         end;
       end;

     procedure tcgavr.a_op_const_reg_internal(list: TAsmList; Op: TOpCG;
      size: TCGSize; a: tcgint; reg, reghi: TRegister);

       var
         mask : qword;
         shift : byte;
         i,j : byte;
         tmpreg : tregister;
         tmpreg64 : tregister64;

      { NextReg* is sometimes called before the register usage and sometimes afterwards }
      procedure NextRegPreInc;
        begin
          if i=5 then
            reg:=reghi
          else
            reg:=GetNextReg(reg);
        end;


      procedure NextRegPostInc;
        begin
          if i=4 then
            reg:=reghi
          else
            reg:=GetNextReg(reg);
        end;

      var
        curvalue : byte;
        l1: TAsmLabel;

       begin
         optimize_op_const(size,op,a);
         mask:=$ff;
         shift:=0;
         case op of
           OP_NONE:
             begin
               { Opcode is optimized away }
             end;
           OP_MOVE:
             begin
               { Optimized, replaced with a simple load }
               a_load_const_reg(list,size,a,reg);
             end;
           OP_OR:
             begin
               for i:=1 to tcgsize2size[size] do
                 begin
                   if ((qword(a) and mask) shr shift)<>0 then
                     list.concat(taicpu.op_reg_const(A_ORI,reg,(qword(a) and mask) shr shift));
                   { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
                   if i<tcgsize2size[size] then
                     NextRegPostInc;
                   mask:=mask shl 8;
                   inc(shift,8);
                 end;
             end;
           OP_AND:
             begin
               for i:=1 to tcgsize2size[size] do
                 begin
                   if ((qword(a) and mask) shr shift)=0 then
                     list.concat(taicpu.op_reg_reg(A_MOV,reg,NR_R1))
                   else if ((qword(a) and mask) shr shift)<>$ff then
                     list.concat(taicpu.op_reg_const(A_ANDI,reg,(qword(a) and mask) shr shift));
                   { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
                   if i<tcgsize2size[size] then
                     NextRegPostInc;
                   mask:=mask shl 8;
                   inc(shift,8);
                 end;
             end;
           OP_SUB:
             begin
               if ((a and mask)=1) and (tcgsize2size[size]=1) then
                 list.concat(taicpu.op_reg(A_DEC,reg))
               else
                 list.concat(taicpu.op_reg_const(A_SUBI,reg,a and mask));
               if size in [OS_S16,OS_16,OS_S32,OS_32,OS_S64,OS_64] then
                 begin
                   for i:=2 to tcgsize2size[size] do
                     begin
                       NextRegPreInc;
                       mask:=mask shl 8;
                       inc(shift,8);
                       curvalue:=(qword(a) and mask) shr shift;
                       { decrease pressure on upper half of registers by using SBC ...,R1 instead
                         of SBCI ...,0 }
                       if curvalue=0 then
                         list.concat(taicpu.op_reg_reg(A_SBC,reg,NR_R1))
                       else
                         list.concat(taicpu.op_reg_const(A_SBCI,reg,curvalue));
                     end;
                 end;
             end;
           OP_SHR,OP_SHL,OP_SAR,OP_ROL,OP_ROR:
             begin
               if (op=OP_SAR) and (a>=(tcgsize2size[size]*8-1)) then
                 begin
                   current_asmdata.getjumplabel(l1);
                   list.concat(taicpu.op_reg(A_TST,GetOffsetReg64(reg,reghi,tcgsize2size[size]-1)));
                   a_load_const_reg(list,OS_8,0,GetOffsetReg64(reg,reghi,tcgsize2size[size]-1));
                   a_jmp_flags(list,F_PL,l1);
                   list.concat(taicpu.op_reg(A_DEC,GetOffsetReg64(reg,reghi,tcgsize2size[size]-1)));
                   cg.a_label(list,l1);
                   for i:=2 to tcgsize2size[size] do
                     a_load_reg_reg(list,OS_8,OS_8,GetOffsetReg64(reg,reghi,tcgsize2size[size]-1),GetOffsetReg64(reg,reghi,tcgsize2size[size]-i));
                 end
               else if (op=OP_SHR) and (a=(tcgsize2size[size]*8-1)) then
                 begin
                   current_asmdata.getjumplabel(l1);
                   list.concat(taicpu.op_reg(A_TST,GetOffsetReg64(reg,reghi,tcgsize2size[size]-1)));
                   a_load_const_reg(list,OS_8,0,GetOffsetReg64(reg,reghi,0));
                   a_jmp_flags(list,F_PL,l1);
                   list.concat(taicpu.op_reg(A_INC,GetOffsetReg64(reg,reghi,0)));
                   cg.a_label(list,l1);
                   for i:=1 to tcgsize2size[size]-1 do
                     a_load_const_reg(list,OS_8,0,GetOffsetReg64(reg,reghi,i));
                 end
               else if a*tcgsize2size[size]<=8 then
                 begin
                   for j:=1 to a do
                     begin
                       case op of
                         OP_SHR:
                           list.concat(taicpu.op_reg(A_LSR,GetOffsetReg64(reg,reghi,tcgsize2size[size]-1)));
                         OP_SHL:
                           list.concat(taicpu.op_reg(A_LSL,reg));
                         OP_SAR:
                           list.concat(taicpu.op_reg(A_ASR,GetOffsetReg64(reg,reghi,tcgsize2size[size]-1)));
                         OP_ROR:
                           begin
                             { load carry? }
                             if not(size in [OS_8,OS_S8]) then
                               begin
                                 list.concat(taicpu.op_none(A_CLC));
                                 list.concat(taicpu.op_reg_const(A_SBRC,reg,0));
                                 list.concat(taicpu.op_none(A_SEC));
                               end;
                             list.concat(taicpu.op_reg(A_ROR,GetOffsetReg64(reg,reghi,tcgsize2size[size]-1)));
                           end;
                         OP_ROL:
                           begin
                             { load carry? }
                             if not(size in [OS_8,OS_S8]) then
                               begin
                                 list.concat(taicpu.op_none(A_CLC));
                                 list.concat(taicpu.op_reg_const(A_SBRC,GetOffsetReg64(reg,reghi,tcgsize2size[size]-1),7));
                                 list.concat(taicpu.op_none(A_SEC));
                               end;
                             list.concat(taicpu.op_reg(A_ROL,reg))
                           end;
                         else
                           internalerror(2011030901);
                       end;
                       if size in [OS_S16,OS_16,OS_S32,OS_32,OS_S64,OS_64] then
                         begin
                           for i:=2 to tcgsize2size[size] do
                             begin
                               case op of
                                 OP_ROR,
                                 OP_SHR:
                                   list.concat(taicpu.op_reg(A_ROR,GetOffsetReg64(reg,reghi,tcgsize2size[size]-i)));
                                 OP_ROL,
                                 OP_SHL:
                                   list.concat(taicpu.op_reg(A_ROL,GetOffsetReg64(reg,reghi,i-1)));
                                 OP_SAR:
                                   list.concat(taicpu.op_reg(A_ROR,GetOffsetReg64(reg,reghi,tcgsize2size[size]-i)));
                                 else
                                   internalerror(2011030902);
                               end;
                           end;
                         end;
                   end;
                 end
               else
                 begin
                   tmpreg:=getintregister(list,size);
                   a_load_const_reg(list,size,a,tmpreg);
                   a_op_reg_reg(list,op,size,tmpreg,reg);
                 end;
             end;
           OP_ADD:
             begin
               curvalue:=a and mask;
               if curvalue=0 then
                 list.concat(taicpu.op_reg_reg(A_ADD,reg,NR_R1))
               else if (curvalue=1) and (tcgsize2size[size]=1) then
                 list.concat(taicpu.op_reg(A_INC,reg))
               else
                 begin
                   tmpreg:=getintregister(list,OS_8);
                   a_load_const_reg(list,OS_8,curvalue,tmpreg);
                   list.concat(taicpu.op_reg_reg(A_ADD,reg,tmpreg));
                 end;
               if size in [OS_S16,OS_16,OS_S32,OS_32,OS_S64,OS_64] then
                 begin
                   for i:=2 to tcgsize2size[size] do
                     begin
                       NextRegPreInc;
                       mask:=mask shl 8;
                       inc(shift,8);
                       curvalue:=(qword(a) and mask) shr shift;
                       { decrease pressure on upper half of registers by using ADC ...,R1 instead
                         of ADD ...,0 }
                       if curvalue=0 then
                         list.concat(taicpu.op_reg_reg(A_ADC,reg,NR_R1))
                       else
                         begin
                           tmpreg:=getintregister(list,OS_8);
                           a_load_const_reg(list,OS_8,curvalue,tmpreg);
                           list.concat(taicpu.op_reg_reg(A_ADC,reg,tmpreg));
                         end;
                     end;
                 end;
             end;
         else
           begin
             if size in [OS_64,OS_S64] then
               begin
                 tmpreg64.reglo:=getintregister(list,OS_32);
                 tmpreg64.reghi:=getintregister(list,OS_32);
                 cg64.a_load64_const_reg(list,a,tmpreg64);
                 cg64.a_op64_reg_reg(list,op,size,tmpreg64,joinreg64(reg,reghi));
               end
             else
               begin
{$if 0}
                 { code not working yet }
                 if (op=OP_SAR) and (a=31) and (size in [OS_32,OS_S32]) then
                   begin
                     tmpreg:=reg;
                     for i:=1 to 4 do
                       begin
                         list.concat(taicpu.op_reg_reg(A_MOV,tmpreg,NR_R1));
                         tmpreg:=GetNextReg(tmpreg);
                       end;
                   end
                 else
{$endif}
                   begin
                     tmpreg:=getintregister(list,size);
                     a_load_const_reg(list,size,a,tmpreg);
                     a_op_reg_reg(list,op,size,tmpreg,reg);
                   end;
               end;
           end;
       end;
     end;


     procedure tcgavr.a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);
       var
         mask : qword;
         shift : byte;
         i : byte;
       begin
         mask:=$ff;
         shift:=0;
         for i:=1 to tcgsize2size[size] do
           begin
             if ((qword(a) and mask) shr shift)=0 then
               emit_mov(list,reg,NR_R1)
             else
               begin
                 getcpuregister(list,NR_R26);
                 list.concat(taicpu.op_reg_const(A_LDI,NR_R26,(qword(a) and mask) shr shift));
                 a_load_reg_reg(list,OS_8,OS_8,NR_R26,reg);
                 ungetcpuregister(list,NR_R26);
               end;

             mask:=mask shl 8;
             inc(shift,8);
             { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
             if i<tcgsize2size[size] then
               reg:=GetNextReg(reg);
           end;
       end;


    procedure tcgavr.maybegetcpuregister(list:tasmlist;reg : tregister);
      begin
        { allocate the register only, if a cpu register is passed }
        if getsupreg(reg)<first_int_imreg then
          getcpuregister(list,reg);
      end;


    function tcgavr.normalize_ref(list:TAsmList;ref: treference;tmpreg : tregister) : treference;

      var
        tmpref : treference;
        l : tasmlabel;
      begin
        Result:=ref;

         if ref.addressmode<>AM_UNCHANGED then
           internalerror(2011021701);

        { Be sure to have a base register }
        if (ref.base=NR_NO) then
          begin
            { only symbol+offset? }
            if ref.index=NR_NO then
              exit;
            ref.base:=ref.index;
            ref.index:=NR_NO;
          end;

        { can we take advantage of adiw/sbiw? }
        if (current_settings.cputype>=cpu_avr2) and not(assigned(ref.symbol)) and (ref.offset<>0) and (ref.offset>=-63) and (ref.offset<=63) and
          ((tmpreg=NR_R24) or (tmpreg=NR_R26) or (tmpreg=NR_R28) or (tmpreg=NR_R30)) and (ref.base<>NR_NO) then
          begin
            maybegetcpuregister(list,tmpreg);
            emit_mov(list,tmpreg,ref.base);
            maybegetcpuregister(list,GetNextReg(tmpreg));
            emit_mov(list,GetNextReg(tmpreg),GetNextReg(ref.base));
            if ref.index<>NR_NO then
              begin
                list.concat(taicpu.op_reg_reg(A_ADD,tmpreg,ref.index));
                list.concat(taicpu.op_reg_reg(A_ADC,GetNextReg(tmpreg),GetNextReg(ref.index)));
              end;
            if ref.offset>0 then
              list.concat(taicpu.op_reg_const(A_ADIW,tmpreg,ref.offset))
            else
              list.concat(taicpu.op_reg_const(A_SBIW,tmpreg,-ref.offset));
            ref.offset:=0;
            ref.base:=tmpreg;
            ref.index:=NR_NO;
          end
        else if assigned(ref.symbol) or (ref.offset<>0) then
          begin
            reference_reset(tmpref,0,[]);
            tmpref.symbol:=ref.symbol;
            tmpref.offset:=ref.offset;
            if assigned(ref.symbol) and (ref.symbol.typ in [AT_FUNCTION,AT_LABEL]) then
              tmpref.refaddr:=addr_lo8_gs
            else
              tmpref.refaddr:=addr_lo8;
            maybegetcpuregister(list,tmpreg);
            list.concat(taicpu.op_reg_ref(A_LDI,tmpreg,tmpref));

            if assigned(ref.symbol) and (ref.symbol.typ in [AT_FUNCTION,AT_LABEL]) then
              tmpref.refaddr:=addr_hi8_gs
            else
              tmpref.refaddr:=addr_hi8;
            maybegetcpuregister(list,GetNextReg(tmpreg));
            list.concat(taicpu.op_reg_ref(A_LDI,GetNextReg(tmpreg),tmpref));

            if (ref.base<>NR_NO) then
              begin
                list.concat(taicpu.op_reg_reg(A_ADD,tmpreg,ref.base));
                list.concat(taicpu.op_reg_reg(A_ADC,GetNextReg(tmpreg),GetNextReg(ref.base)));
              end;
            if (ref.index<>NR_NO) then
              begin
                list.concat(taicpu.op_reg_reg(A_ADD,tmpreg,ref.index));
                list.concat(taicpu.op_reg_reg(A_ADC,GetNextReg(tmpreg),GetNextReg(ref.index)));
              end;
            ref.symbol:=nil;
            ref.offset:=0;
            ref.base:=tmpreg;
            ref.index:=NR_NO;
          end
        else if (ref.base<>NR_NO) and (ref.index<>NR_NO) then
          begin
            maybegetcpuregister(list,tmpreg);
            emit_mov(list,tmpreg,ref.base);
            maybegetcpuregister(list,GetNextReg(tmpreg));
            emit_mov(list,GetNextReg(tmpreg),GetNextReg(ref.base));
            list.concat(taicpu.op_reg_reg(A_ADD,tmpreg,ref.index));
            list.concat(taicpu.op_reg_reg(A_ADC,GetNextReg(tmpreg),GetNextReg(ref.index)));
            ref.base:=tmpreg;
            ref.index:=NR_NO;
          end
        else if (ref.base<>NR_NO) then
          begin
            maybegetcpuregister(list,tmpreg);
            emit_mov(list,tmpreg,ref.base);
            maybegetcpuregister(list,GetNextReg(tmpreg));
            emit_mov(list,GetNextReg(tmpreg),GetNextReg(ref.base));
            ref.base:=tmpreg;
            ref.index:=NR_NO;
          end
        else if (ref.index<>NR_NO) then
          begin
            maybegetcpuregister(list,tmpreg);
            emit_mov(list,tmpreg,ref.index);
            maybegetcpuregister(list,GetNextReg(tmpreg));
            emit_mov(list,GetNextReg(tmpreg),GetNextReg(ref.index));
            ref.base:=tmpreg;
            ref.index:=NR_NO;
          end;
        Result:=ref;
      end;


     procedure tcgavr.a_load_reg_ref(list : TAsmList; fromsize, tosize: tcgsize; reg : tregister;const ref : treference);
       var
         href : treference;
         conv_done: boolean;
         tmpreg : tregister;
         i : integer;
         QuickRef,ungetcpuregister_z: Boolean;
       begin
         QuickRef:=false;
         ungetcpuregister_z:=false;

         href:=Ref;
         { ensure, href.base contains a valid register if there is any register used }
         if href.base=NR_NO then
           begin
             href.base:=href.index;
             href.index:=NR_NO;
           end;

         { try to use std/sts }
         if not((href.Base=NR_NO) and (href.Index=NR_NO)) then
           begin
             if not((href.addressmode=AM_UNCHANGED) and
                    (href.symbol=nil) and
                     (href.Index=NR_NO) and
                     (href.Offset in [0..64-tcgsize2size[fromsize]])) then
               begin
                 href:=normalize_ref(list,href,NR_R30);
                 getcpuregister(list,NR_R30);
                 getcpuregister(list,NR_R31);
                 ungetcpuregister_z:=true;
               end
             else
               begin
                 if (href.base<>NR_R28) and (href.base<>NR_R30) then
                   begin
                     getcpuregister(list,NR_R30);
                     emit_mov(list,NR_R30,href.base);
                     getcpuregister(list,NR_R31);
                     emit_mov(list,NR_R31,GetNextReg(href.base));
                     href.base:=NR_R30;
                     ungetcpuregister_z:=true;
                   end;
                 QuickRef:=true;
               end;
           end
         else
           QuickRef:=true;

         if (tcgsize2size[fromsize]>32) or (tcgsize2size[tosize]>32) or (fromsize=OS_NO) or (tosize=OS_NO) then
           internalerror(2011021307);

         conv_done:=false;
         if tosize<>fromsize then
           begin
             conv_done:=true;
             if tcgsize2size[tosize]<=tcgsize2size[fromsize] then
               fromsize:=tosize;
             case fromsize of
               OS_8:
                 begin
                   if not(QuickRef) and (tcgsize2size[tosize]>1) then
                     href.addressmode:=AM_POSTINCREMENT;

                   list.concat(taicpu.op_ref_reg(GetStore(href),href,reg));
                   for i:=2 to tcgsize2size[tosize] do
                     begin
                       if QuickRef then
                         inc(href.offset);

                       if not(QuickRef) and (i<tcgsize2size[fromsize]) then
                         href.addressmode:=AM_POSTINCREMENT
                       else
                         href.addressmode:=AM_UNCHANGED;

                       list.concat(taicpu.op_ref_reg(GetStore(href),href,NR_R1));
                     end;
                 end;
               OS_S8:
                 begin
                   if not(QuickRef) and (tcgsize2size[tosize]>1) then
                     href.addressmode:=AM_POSTINCREMENT;
                   list.concat(taicpu.op_ref_reg(GetStore(href),href,reg));

                   if tcgsize2size[tosize]>1 then
                     begin
                       tmpreg:=getintregister(list,OS_8);
                       emit_mov(list,tmpreg,NR_R1);
                       list.concat(taicpu.op_reg_const(A_SBRC,reg,7));
                       list.concat(taicpu.op_reg(A_COM,tmpreg));
                       for i:=2 to tcgsize2size[tosize] do
                         begin
                           if QuickRef then
                             inc(href.offset);

                           if not(QuickRef) and (i<tcgsize2size[fromsize]) then
                             href.addressmode:=AM_POSTINCREMENT
                           else
                             href.addressmode:=AM_UNCHANGED;
                           list.concat(taicpu.op_ref_reg(GetStore(href),href,tmpreg));
                         end;
                     end;
                 end;
               OS_16:
                 begin
                   if not(QuickRef) and (tcgsize2size[tosize]>1) then
                     href.addressmode:=AM_POSTINCREMENT;

                   list.concat(taicpu.op_ref_reg(GetStore(href),href,reg));
                   if QuickRef then
                     inc(href.offset)
                   else if not(QuickRef) and (tcgsize2size[fromsize]>2) then
                     href.addressmode:=AM_POSTINCREMENT
                   else
                     href.addressmode:=AM_UNCHANGED;

                   reg:=GetNextReg(reg);
                   list.concat(taicpu.op_ref_reg(GetStore(href),href,reg));

                   for i:=3 to tcgsize2size[tosize] do
                     begin
                       if QuickRef then
                         inc(href.offset);

                       if not(QuickRef) and (i<tcgsize2size[fromsize]) then
                         href.addressmode:=AM_POSTINCREMENT
                       else
                         href.addressmode:=AM_UNCHANGED;

                       list.concat(taicpu.op_ref_reg(GetStore(href),href,NR_R1));
                     end;
                 end;
               OS_S16:
                 begin
                   if not(QuickRef) and (tcgsize2size[tosize]>1) then
                     href.addressmode:=AM_POSTINCREMENT;

                   list.concat(taicpu.op_ref_reg(GetStore(href),href,reg));
                   if QuickRef then
                     inc(href.offset)
                   else if not(QuickRef) and (tcgsize2size[fromsize]>2) then
                     href.addressmode:=AM_POSTINCREMENT
                   else
                     href.addressmode:=AM_UNCHANGED;

                   reg:=GetNextReg(reg);
                   list.concat(taicpu.op_ref_reg(GetStore(href),href,reg));

                   if tcgsize2size[tosize]>2 then
                     begin
                       tmpreg:=getintregister(list,OS_8);
                       emit_mov(list,tmpreg,NR_R1);
                       list.concat(taicpu.op_reg_const(A_SBRC,reg,7));
                       list.concat(taicpu.op_reg(A_COM,tmpreg));
                       for i:=3 to tcgsize2size[tosize] do
                         begin
                           if QuickRef then
                             inc(href.offset);

                           if not(QuickRef) and (i<tcgsize2size[fromsize]) then
                             href.addressmode:=AM_POSTINCREMENT
                           else
                             href.addressmode:=AM_UNCHANGED;
                           list.concat(taicpu.op_ref_reg(GetStore(href),href,tmpreg));
                         end;
                     end;
                 end;
               else
                 conv_done:=false;
             end;
           end;
         if not conv_done then
           begin
             // CC
             // Write to 16 bit ioreg, first high byte then low byte
             // sequence required for 16 bit timer registers
             // See e.g. atmega328p manual para 15.3 Accessing 16 bit registers
             if (fromsize in [OS_16, OS_S16]) and QuickRef and (href.offset > 31)
               and (href.offset < cpuinfo.embedded_controllers[current_settings.controllertype].srambase) then
               begin
                 tmpreg:=GetNextReg(reg);
                 href.addressmode:=AM_UNCHANGED;
                 inc(href.offset);
                 list.concat(taicpu.op_ref_reg(GetStore(href),href,tmpreg));
                 dec(href.offset);
                 list.concat(taicpu.op_ref_reg(GetStore(href),href,reg));
               end
             else
               begin
                 for i:=1 to tcgsize2size[fromsize] do
                   begin
                       if not(QuickRef) and (i<tcgsize2size[fromsize]) then
                         href.addressmode:=AM_POSTINCREMENT
                       else
                         href.addressmode:=AM_UNCHANGED;

                     list.concat(taicpu.op_ref_reg(GetStore(href),href,reg));

                     if QuickRef then
                       inc(href.offset);

                     { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
                     if i<tcgsize2size[fromsize] then
                       reg:=GetNextReg(reg);
                   end;
               end;
           end;

         if not(QuickRef) or ungetcpuregister_z then
           begin
             ungetcpuregister(list,href.base);
             ungetcpuregister(list,GetNextReg(href.base));
           end;
       end;


     procedure tcgavr.a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;
       const Ref : treference;reg : tregister);
       var
         href : treference;
         conv_done: boolean;
         tmpreg : tregister;
         i : integer;
         QuickRef,ungetcpuregister_z: boolean;
       begin
         QuickRef:=false;
         ungetcpuregister_z:=false;

         href:=Ref;
         { ensure, href.base contains a valid register if there is any register used }
         if href.base=NR_NO then
           begin
             href.base:=href.index;
             href.index:=NR_NO;
           end;

         { try to use ldd/lds }
         if not((href.Base=NR_NO) and (href.Index=NR_NO)) then
           begin
             if not((href.addressmode=AM_UNCHANGED) and
                    (href.symbol=nil) and
                     (href.Index=NR_NO) and
                     (href.Offset in [0..64-tcgsize2size[fromsize]])) then
               begin
                 href:=normalize_ref(list,href,NR_R30);
                 getcpuregister(list,NR_R30);
                 getcpuregister(list,NR_R31);
                 ungetcpuregister_z:=true;
               end
             else
               begin
                 if (href.base<>NR_R28) and (href.base<>NR_R30) then
                   begin
                     getcpuregister(list,NR_R30);
                     emit_mov(list,NR_R30,href.base);
                     getcpuregister(list,NR_R31);
                     emit_mov(list,NR_R31,GetNextReg(href.base));
                     href.base:=NR_R30;
                     ungetcpuregister_z:=true;
                   end;
                 QuickRef:=true;
               end;
           end
         else
           QuickRef:=true;

         if (tcgsize2size[fromsize]>32) or (tcgsize2size[tosize]>32) or (fromsize=OS_NO) or (tosize=OS_NO) then
           internalerror(2011021307);

         conv_done:=false;
         if tosize<>fromsize then
           begin
             conv_done:=true;
             if tcgsize2size[tosize]<=tcgsize2size[fromsize] then
               fromsize:=tosize;
             case fromsize of
               OS_8:
                 begin
                   list.concat(taicpu.op_reg_ref(GetLoad(href),reg,href));
                   for i:=2 to tcgsize2size[tosize] do
                     begin
                       reg:=GetNextReg(reg);
                       emit_mov(list,reg,NR_R1);
                     end;
                 end;
               OS_S8:
                 begin
                   list.concat(taicpu.op_reg_ref(GetLoad(href),reg,href));
                   tmpreg:=reg;

                   if tcgsize2size[tosize]>1 then
                     begin
                       reg:=GetNextReg(reg);
                       emit_mov(list,reg,NR_R1);
                       list.concat(taicpu.op_reg_const(A_SBRC,tmpreg,7));
                       list.concat(taicpu.op_reg(A_COM,reg));
                       tmpreg:=reg;
                       for i:=3 to tcgsize2size[tosize] do
                         begin
                           reg:=GetNextReg(reg);
                           emit_mov(list,reg,tmpreg);
                         end;
                     end;
                 end;
               OS_16:
                 begin
                   if not(QuickRef) then
                     href.addressmode:=AM_POSTINCREMENT;
                   list.concat(taicpu.op_reg_ref(GetLoad(href),reg,href));

                   if QuickRef then
                     inc(href.offset);
                   href.addressmode:=AM_UNCHANGED;

                   reg:=GetNextReg(reg);
                   list.concat(taicpu.op_reg_ref(GetLoad(href),reg,href));

                   for i:=3 to tcgsize2size[tosize] do
                     begin
                       reg:=GetNextReg(reg);
                       emit_mov(list,reg,NR_R1);
                     end;
                 end;
               OS_S16:
                 begin
                   if not(QuickRef) then
                     href.addressmode:=AM_POSTINCREMENT;
                   list.concat(taicpu.op_reg_ref(GetLoad(href),reg,href));
                   if QuickRef then
                     inc(href.offset);
                   href.addressmode:=AM_UNCHANGED;

                   reg:=GetNextReg(reg);
                   list.concat(taicpu.op_reg_ref(GetLoad(href),reg,href));
                   tmpreg:=reg;

                   reg:=GetNextReg(reg);
                   emit_mov(list,reg,NR_R1);
                   list.concat(taicpu.op_reg_const(A_SBRC,tmpreg,7));
                   list.concat(taicpu.op_reg(A_COM,reg));
                   tmpreg:=reg;
                   for i:=4 to tcgsize2size[tosize] do
                     begin
                       reg:=GetNextReg(reg);
                       emit_mov(list,reg,tmpreg);
                     end;
                 end;
               else
                 conv_done:=false;
             end;
           end;
         if not conv_done then
           begin
             for i:=1 to tcgsize2size[fromsize] do
               begin
                 if not(QuickRef) and (i<tcgsize2size[fromsize]) then
                   href.addressmode:=AM_POSTINCREMENT
                 else
                   href.addressmode:=AM_UNCHANGED;

                 list.concat(taicpu.op_reg_ref(GetLoad(href),reg,href));

                 if QuickRef then
                   inc(href.offset);

                 { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
                 if i<tcgsize2size[fromsize] then
                   reg:=GetNextReg(reg);
               end;
           end;

         if ungetcpuregister_z then
           begin
             ungetcpuregister(list,href.base);
             ungetcpuregister(list,GetNextReg(href.base));
           end;
       end;


     procedure tcgavr.a_load_reg_reg(list : TAsmList; fromsize, tosize : tcgsize;reg1,reg2 : tregister);
       var
         conv_done: boolean;
         tmpreg : tregister;
         i : integer;
       begin
         if (tcgsize2size[fromsize]>32) or (tcgsize2size[tosize]>32) or (fromsize=OS_NO) or (tosize=OS_NO) then
           internalerror(2011021310);

         conv_done:=false;
         if tosize<>fromsize then
           begin
             conv_done:=true;
             if tcgsize2size[tosize]<=tcgsize2size[fromsize] then
               fromsize:=tosize;
             case fromsize of
               OS_8:
                 begin
                   emit_mov(list,reg2,reg1);
                   for i:=2 to tcgsize2size[tosize] do
                     begin
                       reg2:=GetNextReg(reg2);
                       emit_mov(list,reg2,NR_R1);
                     end;
                 end;
               OS_S8:
                 begin
                   emit_mov(list,reg2,reg1);

                   if tcgsize2size[tosize]>1 then
                     begin
                       reg2:=GetNextReg(reg2);
                       emit_mov(list,reg2,NR_R1);
                       list.concat(taicpu.op_reg_const(A_SBRC,reg1,7));
                       list.concat(taicpu.op_reg(A_COM,reg2));
                       tmpreg:=reg2;
                       for i:=3 to tcgsize2size[tosize] do
                         begin
                           reg2:=GetNextReg(reg2);
                           emit_mov(list,reg2,tmpreg);
                         end;
                     end;
                 end;
               OS_16:
                 begin
                   emit_mov(list,reg2,reg1);

                   reg1:=GetNextReg(reg1);
                   reg2:=GetNextReg(reg2);
                   emit_mov(list,reg2,reg1);

                   for i:=3 to tcgsize2size[tosize] do
                     begin
                       reg2:=GetNextReg(reg2);
                       emit_mov(list,reg2,NR_R1);
                     end;
                 end;
               OS_S16:
                 begin
                   emit_mov(list,reg2,reg1);

                   reg1:=GetNextReg(reg1);
                   reg2:=GetNextReg(reg2);
                   emit_mov(list,reg2,reg1);

                   if tcgsize2size[tosize]>2 then
                     begin
                       reg2:=GetNextReg(reg2);
                       emit_mov(list,reg2,NR_R1);
                       list.concat(taicpu.op_reg_const(A_SBRC,reg1,7));
                       list.concat(taicpu.op_reg(A_COM,reg2));
                       tmpreg:=reg2;
                       for i:=4 to tcgsize2size[tosize] do
                         begin
                           reg2:=GetNextReg(reg2);
                           emit_mov(list,reg2,tmpreg);
                         end;
                     end;
                 end;
               else
                 conv_done:=false;
             end;
           end;
         if not conv_done and (reg1<>reg2) then
           begin
             for i:=1 to tcgsize2size[fromsize] do
               begin
                 emit_mov(list,reg2,reg1);
                 { check if we are not in the last iteration to avoid an internalerror in GetNextReg }
                 if i<tcgsize2size[fromsize] then
                   begin
                     reg1:=GetNextReg(reg1);
                     reg2:=GetNextReg(reg2);
                   end;
               end;
           end;
       end;


     procedure tcgavr.a_loadfpu_reg_reg(list: TAsmList; fromsize,tosize: tcgsize; reg1, reg2: tregister);
       begin
         internalerror(2012010702);
       end;


     procedure tcgavr.a_loadfpu_ref_reg(list: TAsmList; fromsize,tosize: tcgsize; const ref: treference; reg: tregister);
       begin
         internalerror(2012010703);
       end;


     procedure tcgavr.a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference);
       begin
         internalerror(2012010704);
       end;


    {  comparison operations }
    procedure tcgavr.a_cmp_const_reg_label(list : TAsmList;size : tcgsize;
      cmp_op : topcmp;a : tcgint;reg : tregister;l : tasmlabel);
      var
        swapped , test_msb: boolean;
        tmpreg : tregister;
        i : byte;
      begin
        if a=0 then
          begin
            swapped:=false;
            { swap parameters? }
            case cmp_op of
              OC_GT:
                begin
                  swapped:=true;
                  cmp_op:=OC_LT;
                end;
              OC_LTE:
                begin
                  swapped:=true;
                  cmp_op:=OC_GTE;
                end;
              OC_BE:
                begin
                  swapped:=true;
                  cmp_op:=OC_AE;
                end;
              OC_A:
                begin
                  swapped:=true;
                  cmp_op:=OC_B;
                end;
            end;

            { If doing a signed test for x<0, we can simply test the sign bit
              of the most significant byte }
            if (cmp_op in [OC_LT,OC_GTE]) and
               (not swapped) then
              begin
                for i:=2 to tcgsize2size[size] do
                  reg:=GetNextReg(reg);

                list.concat(taicpu.op_reg_reg(A_CP,reg,NR_R1));
              end
            else
              begin
                if swapped then
                  list.concat(taicpu.op_reg_reg(A_CP,NR_R1,reg))
                else
                  list.concat(taicpu.op_reg_reg(A_CP,reg,NR_R1));

                for i:=2 to tcgsize2size[size] do
                  begin
                    reg:=GetNextReg(reg);
                    if swapped then
                      list.concat(taicpu.op_reg_reg(A_CPC,NR_R1,reg))
                    else
                      list.concat(taicpu.op_reg_reg(A_CPC,reg,NR_R1));
                  end;
              end;

            a_jmp_cond(list,cmp_op,l);
          end
        else
          inherited a_cmp_const_reg_label(list,size,cmp_op,a,reg,l);
      end;


    procedure tcgavr.a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;
      cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel);
      var
        swapped : boolean;
        tmpreg : tregister;
        i : byte;
      begin
        swapped:=false;
        { swap parameters? }
        case cmp_op of
          OC_GT:
            begin
              swapped:=true;
              cmp_op:=OC_LT;
            end;
          OC_LTE:
            begin
              swapped:=true;
              cmp_op:=OC_GTE;
            end;
          OC_BE:
            begin
              swapped:=true;
              cmp_op:=OC_AE;
            end;
          OC_A:
            begin
              swapped:=true;
              cmp_op:=OC_B;
            end;
        end;
        if swapped then
          begin
            tmpreg:=reg1;
            reg1:=reg2;
            reg2:=tmpreg;
          end;
        list.concat(taicpu.op_reg_reg(A_CP,reg2,reg1));

        for i:=2 to tcgsize2size[size] do
          begin
            reg1:=GetNextReg(reg1);
            reg2:=GetNextReg(reg2);
            list.concat(taicpu.op_reg_reg(A_CPC,reg2,reg1));
          end;

        a_jmp_cond(list,cmp_op,l);
      end;


    procedure tcgavr.a_jmp_name(list : TAsmList;const s : string);
      var
        ai : taicpu;
      begin
        if CPUAVR_HAS_JMP_CALL in cpu_capabilities[current_settings.cputype] then
          ai:=taicpu.op_sym(A_JMP,current_asmdata.RefAsmSymbol(s,AT_FUNCTION))
        else
          ai:=taicpu.op_sym(A_RJMP,current_asmdata.RefAsmSymbol(s,AT_FUNCTION));
        ai.is_jmp:=true;
        list.concat(ai);
      end;


    procedure tcgavr.a_jmp_always(list : TAsmList;l: tasmlabel);
      var
        ai : taicpu;
      begin
        if CPUAVR_HAS_JMP_CALL in cpu_capabilities[current_settings.cputype] then
          ai:=taicpu.op_sym(A_JMP,l)
        else
          ai:=taicpu.op_sym(A_RJMP,l);
        ai.is_jmp:=true;
        list.concat(ai);
      end;


    procedure tcgavr.a_jmp_flags(list : TAsmList;const f : TResFlags;l: tasmlabel);
      var
        ai : taicpu;
      begin
        ai:=setcondition(taicpu.op_sym(A_BRxx,l),flags_to_cond(f));
        ai.is_jmp:=true;
        list.concat(ai);
      end;


    procedure tcgavr.g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister);
      var
        l : TAsmLabel;
        tmpflags : TResFlags;
        i: Integer;
        hreg: TRegister;
      begin
        current_asmdata.getjumplabel(l);
        {
        if flags_to_cond(f) then
          begin
            tmpflags:=f;
            inverse_flags(tmpflags);
            emit_mov(reg,NR_R1);
            a_jmp_flags(list,tmpflags,l);
            list.concat(taicpu.op_reg_const(A_LDI,reg,1));
          end
        else
        }
          begin
            list.concat(taicpu.op_reg_const(A_LDI,reg,1));
            hreg:=reg;
            for i:=2 to tcgsize2size[size] do
              begin
                hreg:=GetNextReg(hreg);
                emit_mov(list,hreg,NR_R1);
              end;

            a_jmp_flags(list,f,l);
            emit_mov(list,reg,NR_R1);
          end;
        cg.a_label(list,l);
      end;


    procedure tcgavr.a_adjust_sp(list : TAsmList; value : longint);
      var
        i : integer;
      begin
        case value of
          0:
            ;
          {-14..-1:
            begin
              if ((-value) mod 2)<>0 then
                list.concat(taicpu.op_reg(A_PUSH,NR_R0));
              for i:=1 to (-value) div 2 do
                list.concat(taicpu.op_const(A_RCALL,0));
            end;
          1..7:
            begin
              for i:=1 to value do
                list.concat(taicpu.op_reg(A_POP,NR_R0));
            end;}
          else
            begin
              list.concat(taicpu.op_reg_const(A_SUBI,NR_R28,lo(word(-value))));
              list.concat(taicpu.op_reg_const(A_SBCI,NR_R29,hi(word(-value))));
              // get SREG
              list.concat(taicpu.op_reg_const(A_IN,NR_R0,NIO_SREG));

              // block interrupts
              list.concat(taicpu.op_none(A_CLI));

              // write high SP
              list.concat(taicpu.op_const_reg(A_OUT,NIO_SP_HI,NR_R29));

              // release interrupts
              list.concat(taicpu.op_const_reg(A_OUT,NIO_SREG,NR_R0));

              // write low SP
              list.concat(taicpu.op_const_reg(A_OUT,NIO_SP_LO,NR_R28));
            end;
        end;
      end;


    function tcgavr.GetLoad(const ref: treference) : tasmop;
      begin
        if (ref.base=NR_NO) and (ref.index=NR_NO) then
          result:=A_LDS
        else if (ref.base<>NR_NO) and (ref.offset<>0) then
          result:=A_LDD
        else
          result:=A_LD;
      end;


    function tcgavr.GetStore(const ref: treference) : tasmop;
      begin
        if (ref.base=NR_NO) and (ref.index=NR_NO) then
          result:=A_STS
        else if (ref.base<>NR_NO) and (ref.offset<>0) then
          result:=A_STD
        else
          result:=A_ST;
      end;

    procedure tcgavr.gen_multiply(list: TAsmList; op: topcg; size: TCgSize; src2, src1, dst: tregister; check_overflow: boolean; var ovloc: tlocation);

      procedure perform_r1_check(overflow_label: TAsmLabel; other_reg: TRegister=NR_R1);
        var
          ai: taicpu;
        begin
          if check_overflow then
            begin
              list.concat(taicpu.op_reg_reg(A_OR,NR_R1,other_reg));

              ai:=Taicpu.Op_Sym(A_BRxx,overflow_label);
              ai.SetCondition(C_NE);
              ai.is_jmp:=true;
              list.concat(ai);
            end;
        end;

      procedure perform_ovf_check(overflow_label: TAsmLabel);
        var
          ai: taicpu;
        begin
          if check_overflow then
            begin
              ai:=Taicpu.Op_Sym(A_BRxx,overflow_label);
              ai.SetCondition(C_CS);
              ai.is_jmp:=true;
              list.concat(ai);
            end;
        end;

      var
        pd: tprocdef;
        paraloc1, paraloc2: tcgpara;
        ai: taicpu;
        hl, no_overflow: TAsmLabel;
        name: String;
      begin
        ovloc.loc:=LOC_VOID;
        if size in [OS_8,OS_S8] then
          begin
            if (CPUAVR_HAS_MUL in cpu_capabilities[current_settings.cputype]) and
               (op=OP_MUL) then
              begin
                cg.a_reg_alloc(list,NR_R0);
                cg.a_reg_alloc(list,NR_R1);
                list.concat(taicpu.op_reg_reg(topcg2asmop[op],src1,src2));
                // Check overflow
                if check_overflow then
                  begin
                    current_asmdata.getjumplabel(hl);
                    list.concat(taicpu.op_reg_reg(A_AND,NR_R1,NR_R1));

                     { Clear carry as it's not affected by any of the instructions }
                    list.concat(taicpu.op_none(A_CLC));

                    ai:=Taicpu.Op_Sym(A_BRxx,hl);
                    ai.SetCondition(C_EQ);
                    ai.is_jmp:=true;
                    list.concat(ai);

                    list.concat(taicpu.op_reg(A_CLR,NR_R1));
                    list.concat(taicpu.op_none(A_SEC));

                    a_label(list,hl);

                    ovloc.loc:=LOC_FLAGS;
                  end
                else
                  list.concat(taicpu.op_reg(A_CLR,NR_R1));
                cg.a_reg_dealloc(list,NR_R1);

                list.concat(taicpu.op_reg_reg(A_MOV,dst,NR_R0));
                cg.a_reg_dealloc(list,NR_R0);
              end
            else if (CPUAVR_HAS_MUL in cpu_capabilities[current_settings.cputype]) and
               (op=OP_IMUL) then
              begin
                cg.a_reg_alloc(list,NR_R0);
                cg.a_reg_alloc(list,NR_R1);
                list.concat(taicpu.op_reg_reg(A_MULS,src1,src2));
                list.concat(taicpu.op_reg_reg(A_MOV,dst,NR_R0));

                // Check overflow
                if check_overflow then
                  begin
                    current_asmdata.getjumplabel(no_overflow);

                    list.concat(taicpu.op_reg_const(A_SBRC,NR_R0,7));
                    list.concat(taicpu.op_reg(A_INC,NR_R1));
                    list.concat(taicpu.op_reg(A_TST,NR_R1));

                    ai:=Taicpu.Op_Sym(A_BRxx,no_overflow);
                    ai.SetCondition(C_EQ);
                    ai.is_jmp:=true;
                    list.concat(ai);

                    list.concat(taicpu.op_reg(A_CLR,NR_R1));

                    a_call_name(list,'FPC_OVERFLOW',false);

                    a_label(list,no_overflow);

                    ovloc.loc:=LOC_VOID;
                  end
                else
                  list.concat(taicpu.op_reg(A_CLR,NR_R1));
                cg.a_reg_dealloc(list,NR_R1);
                cg.a_reg_dealloc(list,NR_R0);
              end
            else
              begin
                if size=OS_8 then
                  name:='fpc_mul_byte'
                else
                  name:='fpc_mul_shortint';

                if check_overflow then
                  name:=name+'_checkoverflow';

                pd:=search_system_proc(name);
                paraloc1.init;
                paraloc2.init;
                paramanager.getintparaloc(list,pd,1,paraloc1);
                paramanager.getintparaloc(list,pd,2,paraloc2);
                a_load_reg_cgpara(list,OS_8,src1,paraloc2);
                a_load_reg_cgpara(list,OS_8,src2,paraloc1);
                paramanager.freecgpara(list,paraloc2);
                paramanager.freecgpara(list,paraloc1);
                alloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
                a_call_name(list,upper(name),false);
                dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
                cg.a_reg_alloc(list,NR_R24);
                cg.a_load_reg_reg(list,OS_8,OS_8,NR_R24,dst);
                cg.a_reg_dealloc(list,NR_R24);
                paraloc2.done;
                paraloc1.done;
              end;
          end
        else if size in [OS_16,OS_S16] then
          begin
            if (CPUAVR_HAS_MUL in cpu_capabilities[current_settings.cputype]) and
               ((not check_overflow) or
                (size=OS_16)) then
              begin
                if check_overflow then
                  begin
                    current_asmdata.getjumplabel(hl);
                    current_asmdata.getjumplabel(no_overflow);
                  end;
                cg.a_reg_alloc(list,NR_R0);
                cg.a_reg_alloc(list,NR_R1);
                list.concat(taicpu.op_reg_reg(A_MUL,src2,src1));
                emit_mov(list,dst,NR_R0);
                emit_mov(list,GetNextReg(dst),NR_R1);
                list.concat(taicpu.op_reg_reg(A_MUL,GetNextReg(src1),src2));
                perform_r1_check(hl);
                list.concat(taicpu.op_reg_reg(A_ADD,GetNextReg(dst),NR_R0));
                perform_ovf_check(hl);
                list.concat(taicpu.op_reg_reg(A_MUL,src1,GetNextReg(src2)));
                perform_r1_check(hl);
                list.concat(taicpu.op_reg_reg(A_ADD,GetNextReg(dst),NR_R0));
                perform_ovf_check(hl);

                if check_overflow then
                  begin
                    list.concat(taicpu.op_reg_reg(A_MUL,GetNextReg(src1),GetNextReg(src2)));
                    perform_r1_check(hl,NR_R0);
                  end;

                cg.a_reg_dealloc(list,NR_R0);
                list.concat(taicpu.op_reg(A_CLR,NR_R1));

                if check_overflow then
                  begin
                    {
                      CLV/CLC
                      JMP no_overflow
                    .hl:
                      CLR R1
                      SEV/SEC
                    .no_overflow:
                    }

                    if op=OP_MUL then
                      list.concat(taicpu.op_none(A_CLC))
                    else
                      list.concat(taicpu.op_none(A_CLV));

                    a_jmp_always(list,no_overflow);

                    a_label(list,hl);

                    list.concat(taicpu.op_reg(A_CLR,NR_R1));

                    if op=OP_MUL then
                      list.concat(taicpu.op_none(A_SEC))
                    else
                      list.concat(taicpu.op_none(A_SEV));

                    a_label(list,no_overflow);

                    ovloc.loc:=LOC_FLAGS;
                  end;

                cg.a_reg_dealloc(list,NR_R1);
              end
            else
              begin
                if size=OS_16 then
                  name:='fpc_mul_word'
                else
                  name:='fpc_mul_integer';

                if check_overflow then
                  name:=name+'_checkoverflow';

                pd:=search_system_proc(name);
                paraloc1.init;
                paraloc2.init;
                paramanager.getintparaloc(list,pd,1,paraloc1);
                paramanager.getintparaloc(list,pd,2,paraloc2);
                a_load_reg_cgpara(list,OS_16,src1,paraloc2);
                a_load_reg_cgpara(list,OS_16,src2,paraloc1);
                paramanager.freecgpara(list,paraloc2);
                paramanager.freecgpara(list,paraloc1);
                alloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
                a_call_name(list,upper(name),false);
                dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
                cg.a_reg_alloc(list,NR_R24);
                cg.a_reg_alloc(list,NR_R25);
                cg.a_load_reg_reg(list,OS_8,OS_8,NR_R24,dst);
                cg.a_reg_dealloc(list,NR_R24);
                cg.a_load_reg_reg(list,OS_8,OS_8,NR_R25,GetNextReg(dst));
                cg.a_reg_dealloc(list,NR_R25);
                paraloc2.done;
                paraloc1.done;
              end;
          end
        else
          internalerror(2011022002);
      end;


    procedure tcgavr.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
      var
         regs : tcpuregisterset;
         reg : tsuperregister;
      begin
        if current_procinfo.procdef.isempty then
          exit;
        if (po_interrupt in current_procinfo.procdef.procoptions) and
          (not nostackframe) then
          begin
            { check if the framepointer is actually used, this is done here because
              we have to know the size of the locals (must be 0), avr does not know
              an sp based stack }

            if not(current_procinfo.procdef.stack_tainting_parameter(calleeside)) and
              (localsize=0) then
              current_procinfo.framepointer:=NR_NO;

            { save int registers,
              but only if the procedure returns }
            if not(po_noreturn in current_procinfo.procdef.procoptions) then
              regs:=rg[R_INTREGISTER].used_in_proc
            else
              regs:=[];
            { if the framepointer is potentially used, save it always because we need a proper stack frame,
              even if the procedure never returns, the procedure could be e.g. a nested one accessing
              an outer stackframe }
            if current_procinfo.framepointer<>NR_NO then
              regs:=regs+[RS_R28,RS_R29];

            { we clear r1 }
            include(regs,RS_R1);

            regs:=regs+[RS_R0];

            for reg:=RS_R31 downto RS_R0 do
              if reg in regs then
                list.concat(taicpu.op_reg(A_PUSH,newreg(R_INTREGISTER,reg,R_SUBWHOLE)));

            { Save SREG }
            cg.getcpuregister(list,NR_R0);
            list.concat(taicpu.op_reg_const(A_IN, NR_R0, $3F));
            list.concat(taicpu.op_reg(A_PUSH, NR_R0));
            cg.ungetcpuregister(list,NR_R0);

            list.concat(taicpu.op_reg(A_CLR,NR_R1));

            if current_procinfo.framepointer<>NR_NO then
              begin
                cg.getcpuregister(list,NR_R28);
                list.concat(taicpu.op_reg_const(A_IN,NR_R28,NIO_SP_LO));
                cg.getcpuregister(list,NR_R29);
                list.concat(taicpu.op_reg_const(A_IN,NR_R29,NIO_SP_HI));
                a_adjust_sp(list,-localsize);
              end;
          end
        else if not(nostackframe) then
          begin
            { check if the framepointer is actually used, this is done here because
              we have to know the size of the locals (must be 0), avr does not know
              an sp based stack }

            if not(current_procinfo.procdef.stack_tainting_parameter(calleeside)) and
              (localsize=0) then
              current_procinfo.framepointer:=NR_NO;

            { save int registers,
              but only if the procedure returns }
            if not(po_noreturn in current_procinfo.procdef.procoptions) then
              regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall)
            else
              regs:=[];
            { if the framepointer is potentially used, save it always because we need a proper stack frame,
              even if the procedure never returns, the procedure could be e.g. a nested one accessing
              an outer stackframe }
            if current_procinfo.framepointer<>NR_NO then
              regs:=regs+[RS_R28,RS_R29];

            for reg:=RS_R31 downto RS_R0 do
              if reg in regs then
                list.concat(taicpu.op_reg(A_PUSH,newreg(R_INTREGISTER,reg,R_SUBWHOLE)));

            if current_procinfo.framepointer<>NR_NO then
              begin
                cg.getcpuregister(list,NR_R28);
                list.concat(taicpu.op_reg_const(A_IN,NR_R28,NIO_SP_LO));
                cg.getcpuregister(list,NR_R29);
                list.concat(taicpu.op_reg_const(A_IN,NR_R29,NIO_SP_HI));
                a_adjust_sp(list,-localsize);
              end;
          end;
      end;


    procedure tcgavr.g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean);
      var
        regs : tcpuregisterset;
        reg : TSuperRegister;
        LocalSize : longint;
      begin
        { every byte counts for avr, so if a subroutine is marked as non-returning, we do
          not generate any exit code, so we really trust the noreturn directive
        }
        if po_noreturn in current_procinfo.procdef.procoptions then
          exit;
        if po_interrupt in current_procinfo.procdef.procoptions then
          begin
            if not(current_procinfo.procdef.isempty) and
              (not nostackframe) then
              begin
                regs:=rg[R_INTREGISTER].used_in_proc;
                if current_procinfo.framepointer<>NR_NO then
                  begin
                    regs:=regs+[RS_R28,RS_R29];
                    LocalSize:=current_procinfo.calc_stackframe_size;
                    a_adjust_sp(list,LocalSize);
                  end;

                { we clear r1 }
                include(regs,RS_R1);

                { Reload SREG }
                regs:=regs+[RS_R0];

                cg.getcpuregister(list,NR_R0);
                list.concat(taicpu.op_reg(A_POP, NR_R0));
                list.concat(taicpu.op_const_reg(A_OUT, $3F, NR_R0));
                cg.ungetcpuregister(list,NR_R0);

                for reg:=RS_R0 to RS_R31 do
                  if reg in regs then
                    list.concat(taicpu.op_reg(A_POP,newreg(R_INTREGISTER,reg,R_SUBWHOLE)));
              end;
            list.concat(taicpu.op_none(A_RETI));
          end
        else if not(nostackframe) and not(current_procinfo.procdef.isempty) then
          begin
            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);
            if current_procinfo.framepointer<>NR_NO then
              begin
                regs:=regs+[RS_R28,RS_R29];
                LocalSize:=current_procinfo.calc_stackframe_size;
                a_adjust_sp(list,LocalSize);
              end;
            for reg:=RS_R0 to RS_R31 do
              if reg in regs then
                list.concat(taicpu.op_reg(A_POP,newreg(R_INTREGISTER,reg,R_SUBWHOLE)));
            list.concat(taicpu.op_none(A_RET));
          end
        else
          list.concat(taicpu.op_none(A_RET));
      end;


    procedure tcgavr.a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);
      var
        tmpref : treference;
      begin
         if ref.addressmode<>AM_UNCHANGED then
           internalerror(2011021701);

        if assigned(ref.symbol) or (ref.offset<>0) then
          begin
            reference_reset(tmpref,0,[]);
            tmpref.symbol:=ref.symbol;
            tmpref.offset:=ref.offset;

            if assigned(ref.symbol) and (ref.symbol.typ in [AT_FUNCTION,AT_LABEL]) then
              tmpref.refaddr:=addr_lo8_gs
            else
              tmpref.refaddr:=addr_lo8;
            list.concat(taicpu.op_reg_ref(A_LDI,r,tmpref));

            if assigned(ref.symbol) and (ref.symbol.typ in [AT_FUNCTION,AT_LABEL]) then
              tmpref.refaddr:=addr_hi8_gs
            else
              tmpref.refaddr:=addr_hi8;
            list.concat(taicpu.op_reg_ref(A_LDI,GetNextReg(r),tmpref));

            if (ref.base<>NR_NO) then
              begin
                list.concat(taicpu.op_reg_reg(A_ADD,r,ref.base));
                list.concat(taicpu.op_reg_reg(A_ADC,GetNextReg(r),GetNextReg(ref.base)));
              end;
            if (ref.index<>NR_NO) then
              begin
                list.concat(taicpu.op_reg_reg(A_ADD,r,ref.index));
                list.concat(taicpu.op_reg_reg(A_ADC,GetNextReg(r),GetNextReg(ref.index)));
              end;
          end
        else if (ref.base<>NR_NO)then
          begin
            emit_mov(list,r,ref.base);
            emit_mov(list,GetNextReg(r),GetNextReg(ref.base));
            if (ref.index<>NR_NO) then
              begin
                list.concat(taicpu.op_reg_reg(A_ADD,r,ref.index));
                list.concat(taicpu.op_reg_reg(A_ADC,GetNextReg(r),GetNextReg(ref.index)));
              end;
          end
        else if (ref.index<>NR_NO) then
          begin
            emit_mov(list,r,ref.index);
            emit_mov(list,GetNextReg(r),GetNextReg(ref.index));
          end;
      end;


    procedure tcgavr.fixref(list : TAsmList;var ref : treference);
      begin
        internalerror(2011021320);
      end;


    procedure tcgavr.g_concatcopy_move(list : TAsmList;const source,dest : treference;len : tcgint);
      var
        paraloc1,paraloc2,paraloc3 : TCGPara;
        pd : tprocdef;
      begin
        pd:=search_system_proc('MOVE');
        paraloc1.init;
        paraloc2.init;
        paraloc3.init;
        paramanager.getintparaloc(list,pd,1,paraloc1);
        paramanager.getintparaloc(list,pd,2,paraloc2);
        paramanager.getintparaloc(list,pd,3,paraloc3);
        a_load_const_cgpara(list,OS_SINT,len,paraloc3);
        a_loadaddr_ref_cgpara(list,dest,paraloc2);
        a_loadaddr_ref_cgpara(list,source,paraloc1);
        paramanager.freecgpara(list,paraloc3);
        paramanager.freecgpara(list,paraloc2);
        paramanager.freecgpara(list,paraloc1);
        alloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        a_call_name_static(list,'FPC_MOVE');
        dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        paraloc3.done;
        paraloc2.done;
        paraloc1.done;
      end;


    procedure tcgavr.g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);
      var
        countreg,tmpreg,tmpreg2: tregister;
        srcref,dstref : treference;
        copysize,countregsize : tcgsize;
        l : TAsmLabel;
        i : longint;
        SrcQuickRef, DestQuickRef : Boolean;
      begin
        if len>16 then
          begin
            current_asmdata.getjumplabel(l);

            reference_reset(srcref,source.alignment,source.volatility);
            reference_reset(dstref,dest.alignment,source.volatility);
            srcref.base:=NR_R30;
            srcref.addressmode:=AM_POSTINCREMENT;
            dstref.base:=NR_R26;
            dstref.addressmode:=AM_POSTINCREMENT;

            copysize:=OS_8;
            if len<256 then
              countregsize:=OS_8
            else if len<65536 then
              countregsize:=OS_16
            else
              internalerror(2011022007);
            countreg:=getintregister(list,countregsize);
            a_load_const_reg(list,countregsize,len,countreg);
            cg.getcpuregister(list,NR_R30);
            cg.getcpuregister(list,NR_R31);
            a_loadaddr_ref_reg(list,source,NR_R30);

            { only base or index register in dest? }
            if ((dest.addressmode=AM_UNCHANGED) and (dest.offset=0) and not(assigned(dest.symbol))) and
              ((dest.base<>NR_NO) xor (dest.index<>NR_NO)) then
              begin
                if dest.base<>NR_NO then
                  tmpreg:=dest.base
                else if dest.index<>NR_NO then
                  tmpreg:=dest.index
                else
                  internalerror(2016112001);
              end
            else
              begin
                tmpreg:=getaddressregister(list);
                a_loadaddr_ref_reg(list,dest,tmpreg);
              end;

            { X is used for spilling code so we can load it
              only by a push/pop sequence, this can be
              optimized later on by the peephole optimizer
            }
            list.concat(taicpu.op_reg(A_PUSH,tmpreg));
            list.concat(taicpu.op_reg(A_PUSH,GetNextReg(tmpreg)));
            cg.getcpuregister(list,NR_R27);
            list.concat(taicpu.op_reg(A_POP,NR_R27));
            cg.getcpuregister(list,NR_R26);
            list.concat(taicpu.op_reg(A_POP,NR_R26));
            cg.a_label(list,l);
            cg.getcpuregister(list,NR_R0);
            list.concat(taicpu.op_reg_ref(GetLoad(srcref),NR_R0,srcref));
            list.concat(taicpu.op_ref_reg(GetStore(dstref),dstref,NR_R0));
            cg.ungetcpuregister(list,NR_R0);
            if tcgsize2size[countregsize] = 1 then
              list.concat(taicpu.op_reg(A_DEC,countreg))
            else
              begin
                list.concat(taicpu.op_reg_const(A_SUBI,countreg,1));
                list.concat(taicpu.op_reg_reg(A_SBC,GetNextReg(countreg),NR_R1));
              end;

            a_jmp_flags(list,F_NE,l);
            cg.ungetcpuregister(list,NR_R26);
            cg.ungetcpuregister(list,NR_R27);
            cg.ungetcpuregister(list,NR_R30);
            cg.ungetcpuregister(list,NR_R31);
            { keep registers alive }
            a_reg_sync(list,countreg);
          end
        else
          begin
            SrcQuickRef:=false;
            DestQuickRef:=false;
            if not((source.addressmode=AM_UNCHANGED) and
                   (source.symbol=nil) and
                   ((source.base=NR_R28) or
                    (source.base=NR_R30)) and
                    (source.Index=NR_NO) and
                    (source.Offset in [0..64-len])) and
              not((source.Base=NR_NO) and (source.Index=NR_NO)) then
              begin
                cg.getcpuregister(list,NR_R30);
                cg.getcpuregister(list,NR_R31);
                srcref:=normalize_ref(list,source,NR_R30)
              end
            else
              begin
                SrcQuickRef:=true;
                srcref:=source;
              end;

            if not((dest.addressmode=AM_UNCHANGED) and
                   (dest.symbol=nil) and
                   ((dest.base=NR_R28) or
                    (dest.base=NR_R30)) and
                    (dest.Index=NR_No) and
                    (dest.Offset in [0..64-len])) and
              not((dest.Base=NR_NO) and (dest.Index=NR_NO)) then
              begin
                if not(SrcQuickRef) then
                  begin
                    { only base or index register in dest? }
                    if ((dest.addressmode=AM_UNCHANGED) and (dest.offset=0) and not(assigned(dest.symbol))) and
                      ((dest.base<>NR_NO) xor (dest.index<>NR_NO)) then
                      begin
                        if dest.base<>NR_NO then
                          tmpreg:=dest.base
                        else if dest.index<>NR_NO then
                          tmpreg:=dest.index
                        else
                          internalerror(2016112002);
                      end
                    else
                      tmpreg:=getaddressregister(list);

                    dstref:=normalize_ref(list,dest,tmpreg);

                    { X is used for spilling code so we can load it
                      only by a push/pop sequence, this can be
                      optimized later on by the peephole optimizer
                    }
                    list.concat(taicpu.op_reg(A_PUSH,tmpreg));
                    list.concat(taicpu.op_reg(A_PUSH,GetNextReg(tmpreg)));
                    cg.getcpuregister(list,NR_R27);
                    list.concat(taicpu.op_reg(A_POP,NR_R27));
                    cg.getcpuregister(list,NR_R26);
                    list.concat(taicpu.op_reg(A_POP,NR_R26));
                    dstref.base:=NR_R26;
                  end
                else
                  begin
                    cg.getcpuregister(list,NR_R30);
                    cg.getcpuregister(list,NR_R31);
                    dstref:=normalize_ref(list,dest,NR_R30);
                  end;
              end
            else
              begin
                DestQuickRef:=true;
                dstref:=dest;
              end;

              // CC
              // If dest is an ioreg (31 < offset < srambase) and size = 16 bit then
              // load high byte first, then low byte
              if (len = 2) and DestQuickRef
                and (dest.offset > 31)
                and (dest.offset < cpuinfo.embedded_controllers[current_settings.controllertype].srambase) then
                begin
                  // If src is also a 16 bit ioreg then read low byte then high byte
                  if SrcQuickRef and (srcref.offset > 31)
                    and (srcref.offset < cpuinfo.embedded_controllers[current_settings.controllertype].srambase) then
                    begin
                      // First read source into temp registers
                      tmpreg:=getintregister(list, OS_16);
                      list.concat(taicpu.op_reg_ref(GetLoad(srcref),tmpreg,srcref));
                      inc(srcref.offset);
                      tmpreg2:=GetNextReg(tmpreg);
                      list.concat(taicpu.op_reg_ref(GetLoad(srcref),tmpreg2,srcref));

                      // then move temp registers to dest in reverse order
                      inc(dstref.offset);
                      list.concat(taicpu.op_ref_reg(GetStore(dstref),dstref,tmpreg2));
                      dec(dstref.offset);
                      list.concat(taicpu.op_ref_reg(GetStore(dstref),dstref,tmpreg));
                    end
                  else
                    begin
                      srcref.addressmode:=AM_UNCHANGED;
                      inc(srcref.offset);
                      dstref.addressmode:=AM_UNCHANGED;
                      inc(dstref.offset);

                      cg.getcpuregister(list,NR_R0);
                      list.concat(taicpu.op_reg_ref(GetLoad(srcref),NR_R0,srcref));
                      list.concat(taicpu.op_ref_reg(GetStore(dstref),dstref,NR_R0));
                      cg.ungetcpuregister(list,NR_R0);

                      if not(SrcQuickRef) then
                        srcref.addressmode:=AM_POSTINCREMENT
                      else
                        srcref.addressmode:=AM_UNCHANGED;

                      dec(srcref.offset);
                      dec(dstref.offset);

                      cg.getcpuregister(list,NR_R0);
                      list.concat(taicpu.op_reg_ref(GetLoad(srcref),NR_R0,srcref));
                      list.concat(taicpu.op_ref_reg(GetStore(dstref),dstref,NR_R0));
                      cg.ungetcpuregister(list,NR_R0);
                    end;
                end
              else
              for i:=1 to len do
                begin
                  if not(SrcQuickRef) and (i<len) then
                    srcref.addressmode:=AM_POSTINCREMENT
                  else
                    srcref.addressmode:=AM_UNCHANGED;

                  if not(DestQuickRef) and (i<len) then
                    dstref.addressmode:=AM_POSTINCREMENT
                  else
                    dstref.addressmode:=AM_UNCHANGED;

                  cg.getcpuregister(list,NR_R0);
                  list.concat(taicpu.op_reg_ref(GetLoad(srcref),NR_R0,srcref));
                  list.concat(taicpu.op_ref_reg(GetStore(dstref),dstref,NR_R0));
                  cg.ungetcpuregister(list,NR_R0);

                  if SrcQuickRef then
                    inc(srcref.offset);
                  if DestQuickRef then
                    inc(dstref.offset);
                end;
              if not(SrcQuickRef) then
                begin
                  ungetcpuregister(list,srcref.base);
                  ungetcpuregister(list,TRegister(ord(srcref.base)+1));
                end;
              if not(DestQuickRef) then
                begin
                  ungetcpuregister(list,dstref.base);
                  ungetcpuregister(list,TRegister(ord(dstref.base)+1));
                end;
            end;
        end;


    procedure tcgavr.g_overflowcheck(list: TAsmList; const l: tlocation; def: tdef);
      var
        hl : tasmlabel;
        ai : taicpu;
        cond : TAsmCond;
      begin
        if not(cs_check_overflow in current_settings.localswitches) then
         exit;
        current_asmdata.getjumplabel(hl);
        if not ((def.typ=pointerdef) or
               ((def.typ=orddef) and
                (torddef(def).ordtype in [u64bit,u16bit,u32bit,u8bit,uchar,
                                          pasbool1,pasbool8,pasbool16,pasbool32,pasbool64]))) then
          cond:=C_VC
        else
          cond:=C_CC;
        ai:=Taicpu.Op_Sym(A_BRxx,hl);
        ai.SetCondition(cond);
        ai.is_jmp:=true;
        list.concat(ai);

        a_call_name(list,'FPC_OVERFLOW',false);
        a_label(list,hl);
      end;


    procedure tcgavr.g_overflowCheck_loc(List: TAsmList; const Loc: TLocation; def: TDef; ovloc: tlocation);
      var
        hl : tasmlabel;
        ai : taicpu;
        cond : TAsmCond;
      begin
        if not(cs_check_overflow in current_settings.localswitches) then
         exit;

        case ovloc.loc of
          LOC_FLAGS:
            begin
              current_asmdata.getjumplabel(hl);
              if not ((def.typ=pointerdef) or
                     ((def.typ=orddef) and
                      (torddef(def).ordtype in [u64bit,u16bit,u32bit,u8bit,uchar,
                                                pasbool1,pasbool8,pasbool16,pasbool32,pasbool64]))) then
                cond:=C_VC
              else
                cond:=C_CC;
              ai:=Taicpu.Op_Sym(A_BRxx,hl);
              ai.SetCondition(cond);
              ai.is_jmp:=true;
              list.concat(ai);

              a_call_name(list,'FPC_OVERFLOW',false);
              a_label(list,hl);
            end;
        end;
      end;


    procedure tcgavr.g_save_registers(list: TAsmList);
      begin
        { this is done by the entry code }
      end;


    procedure tcgavr.g_restore_registers(list: TAsmList);
      begin
        { this is done by the exit code }
      end;


    procedure tcgavr.a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);
      var
        ai1,ai2 : taicpu;
        hl : TAsmLabel;
      begin
        ai1:=Taicpu.Op_sym(A_BRxx,l);
        ai1.is_jmp:=true;
        hl:=nil;
        case cond of
          OC_EQ:
            ai1.SetCondition(C_EQ);
          OC_GT:
            begin
              { emulate GT }
              current_asmdata.getjumplabel(hl);
              ai2:=Taicpu.Op_Sym(A_BRxx,hl);
              ai2.SetCondition(C_EQ);
              ai2.is_jmp:=true;
              list.concat(ai2);

              ai1.SetCondition(C_GE);
            end;
          OC_LT:
            ai1.SetCondition(C_LT);
          OC_GTE:
            ai1.SetCondition(C_GE);
          OC_LTE:
            begin
              { emulate LTE }
              ai2:=Taicpu.Op_Sym(A_BRxx,l);
              ai2.SetCondition(C_EQ);
              ai2.is_jmp:=true;
              list.concat(ai2);

              ai1.SetCondition(C_LT);
            end;
          OC_NE:
            ai1.SetCondition(C_NE);
          OC_BE:
            begin
              { emulate BE }
              ai2:=Taicpu.Op_Sym(A_BRxx,l);
              ai2.SetCondition(C_EQ);
              ai2.is_jmp:=true;
              list.concat(ai2);

              ai1.SetCondition(C_LO);
            end;
          OC_B:
            ai1.SetCondition(C_LO);
          OC_AE:
            ai1.SetCondition(C_SH);
          OC_A:
            begin
              { emulate A (unsigned GT) }
              current_asmdata.getjumplabel(hl);
              ai2:=Taicpu.Op_Sym(A_BRxx,hl);
              ai2.SetCondition(C_EQ);
              ai2.is_jmp:=true;
              list.concat(ai2);

              ai1.SetCondition(C_SH);
            end;
          else
            internalerror(2011082501);
        end;
        list.concat(ai1);
        if assigned(hl) then
          a_label(list,hl);
      end;


    procedure tcgavr.emit_mov(list: TAsmList;reg2: tregister; reg1: tregister);
      var
         instr: taicpu;
      begin
       instr:=taicpu.op_reg_reg(A_MOV,reg2,reg1);
       list.Concat(instr);
       { Notify the register allocator that we have written a move instruction so
         it can try to eliminate it. }
       add_move_instruction(instr);
      end;


    procedure tcg64favr.a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);
      begin
         if not(size in [OS_S64,OS_64]) then
           internalerror(2012102402);
         tcgavr(cg).a_op_reg_reg_internal(list,Op,size,regsrc.reglo,regsrc.reghi,regdst.reglo,regdst.reghi);
      end;


    procedure tcg64favr.a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);
      begin
        tcgavr(cg).a_op_const_reg_internal(list,Op,size,value,reg.reglo,reg.reghi);
      end;


    procedure create_codegen;
      begin
        cg:=tcgavr.create;
        cg64:=tcg64favr.create;
      end;

end.
