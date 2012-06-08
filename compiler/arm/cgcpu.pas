{

    Copyright (c) 2003 by Florian Klaempfl
    Member of the Free Pascal development team

    This unit implements the code generator for the ARM

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
      tcgarm = class(tcg)
        { true, if the next arithmetic operation should modify the flags }
        cgsetflags : boolean;

        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const paraloc : TCGPara);override;
        procedure a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const paraloc : TCGPara);override;
        procedure a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const paraloc : TCGPara);override;

        procedure a_call_name(list : TAsmList;const s : string; weak: boolean);override;
        procedure a_call_reg(list : TAsmList;reg: tregister);override;
        procedure a_call_ref(list : TAsmList;ref: treference);override;

        procedure a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister); override;
        procedure a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister); override;

        procedure a_op_const_reg_reg(list: TAsmList; op: TOpCg;
          size: tcgsize; a: tcgint; src, dst: tregister); override;
        procedure a_op_reg_reg_reg(list: TAsmList; op: TOpCg;
          size: tcgsize; src1, src2, dst: tregister); override;
        procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);override;
        procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);override;

        { move instructions }
        procedure a_load_reg_ref(list : TAsmList; fromsize, tosize: tcgsize; reg : tregister;const ref : treference);override;
        procedure a_load_reg_reg(list : TAsmList; fromsize, tosize : tcgsize;reg1,reg2 : tregister);override;
        function a_internal_load_reg_ref(list : TAsmList; fromsize, tosize: tcgsize; reg : tregister;const ref : treference):treference;
        function a_internal_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister):treference;

        { fpu move instructions }
        procedure a_loadfpu_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister); override;
        procedure a_loadfpu_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister); override;
        procedure a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference); override;

        procedure a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);override;
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
        procedure g_concatcopy_unaligned(list : TAsmList;const source,dest : treference;len : tcgint);override;
        procedure g_concatcopy_move(list : TAsmList;const source,dest : treference;len : tcgint);
        procedure g_concatcopy_internal(list : TAsmList;const source,dest : treference;len : tcgint;aligned : boolean);

        procedure g_overflowcheck(list: TAsmList; const l: tlocation; def: tdef); override;
        procedure g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;ovloc : tlocation);override;

        procedure g_save_registers(list : TAsmList);override;
        procedure g_restore_registers(list : TAsmList);override;

        procedure a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);
        procedure fixref(list : TAsmList;var ref : treference);
        function handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference; virtual;

        procedure g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);override;
        procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint); override;
        procedure g_stackpointer_alloc(list : TAsmList;size : longint);override;

        procedure a_loadmm_reg_reg(list: TAsmList; fromsize, tosize : tcgsize;reg1, reg2: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_ref_reg(list: TAsmList; fromsize, tosize : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_reg_ref(list: TAsmList; fromsize, tosize : tcgsize;reg: tregister; const ref: treference;shuffle : pmmshuffle); override;
        procedure a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize : tcgsize;intreg, mmreg: tregister; shuffle: pmmshuffle); override;
        procedure a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize : tcgsize;mmreg, intreg: tregister; shuffle : pmmshuffle); override;

        procedure a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;src,dst: tregister;shuffle : pmmshuffle); override;
        { Transform unsupported methods into Internal errors }
        procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; size: TCGSize; src, dst: TRegister); override;
      private
        { clear out potential overflow bits from 8 or 16 bit operations  }
        { the upper 24/16 bits of a register after an operation          }
        procedure maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);
        function get_darwin_call_stub(const s: string; weak: boolean): tasmsymbol;
      end;

      tarmcgarm = class(tcgarm)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;
      end;

      tcg64farm = class(tcg64f32)
        procedure a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);override;
        procedure a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);override;
        procedure a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64);override;
        procedure a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);override;
        procedure a_op64_const_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64;setflags : boolean;var ovloc : tlocation);override;
        procedure a_op64_reg_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64;setflags : boolean;var ovloc : tlocation);override;
        procedure a_loadmm_intreg64_reg(list: TAsmList; mmsize: tcgsize; intreg: tregister64; mmreg: tregister);override;
        procedure a_loadmm_reg_intreg64(list: TAsmList; mmsize: tcgsize; mmreg: tregister; intreg: tregister64);override;
      end;

      Tthumb2cgarm = class(tcgarm)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        procedure a_call_reg(list : TAsmList;reg: tregister);override;

        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;

        procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);override;
        procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);override;

        procedure g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister); override;

        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean); override;

        function handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference; override;
      end;

      tthumb2cg64farm = class(tcg64farm)
        procedure a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);override;
      end;

    const
      OpCmp2AsmCond : Array[topcmp] of TAsmCond = (C_NONE,C_EQ,C_GT,
                           C_LT,C_GE,C_LE,C_NE,C_LS,C_CC,C_CS,C_HI);

      winstackpagesize = 4096;

    function get_fpu_postfix(def : tdef) : toppostfix;
    procedure create_codegen;

  implementation


    uses
       globals,verbose,systems,cutils,sysutils,
       aopt,aoptcpu,
       fmodule,
       symconst,symsym,
       tgobj,
       procinfo,cpupi,
       paramgr;


    function get_fpu_postfix(def : tdef) : toppostfix;
      begin
        if def.typ=floatdef then
          begin
            case tfloatdef(def).floattype of
              s32real:
                result:=PF_S;
              s64real:
                result:=PF_D;
              s80real:
                result:=PF_E;
              else
                internalerror(200401272);
            end;
          end
        else
          internalerror(200401271);
      end;


    procedure tarmcgarm.init_register_allocators;
      begin
        inherited init_register_allocators;
        { currently, we always save R14, so we can use it }
        if (target_info.system<>system_arm_darwin) then
          rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
              [RS_R0,RS_R1,RS_R2,RS_R3,RS_R12,RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,
               RS_R9,RS_R10,RS_R14],first_int_imreg,[])
        else
          { r7 is not available on Darwin, it's used as frame pointer (always,
            for backtrace support -- also in gcc/clang -> R11 can be used).
            r9 is volatile }
          rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
              [RS_R0,RS_R1,RS_R2,RS_R3,RS_R9,RS_R12,RS_R4,RS_R5,RS_R6,RS_R8,
               RS_R10,RS_R11,RS_R14],first_int_imreg,[]);
        rg[R_FPUREGISTER]:=trgcpu.create(R_FPUREGISTER,R_SUBNONE,
            [RS_F0,RS_F1,RS_F2,RS_F3,RS_F4,RS_F5,RS_F6,RS_F7],first_fpu_imreg,[]);
        { The register allocator currently cannot deal with multiple
          non-overlapping subregs per register, so we can only use
          half the single precision registers for now (as sub registers of the
          double precision ones). }
        if current_settings.fputype=fpu_vfpv3 then
          rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBFD,
              [RS_D0,RS_D1,RS_D2,RS_D3,RS_D4,RS_D5,RS_D6,RS_D7,
               RS_D16,RS_D17,RS_D18,RS_D19,RS_D20,RS_D21,RS_D22,RS_D23,RS_D24,RS_D25,RS_D26,RS_D27,RS_D28,RS_D29,RS_D30,RS_D31,
               RS_D8,RS_D9,RS_D10,RS_D11,RS_D12,RS_D13,RS_D14,RS_D15
              ],first_mm_imreg,[])
        else
          rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBFD,
              [RS_D0,RS_D1,RS_D2,RS_D3,RS_D4,RS_D5,RS_D6,RS_D7,RS_D8,RS_D9,RS_D10,RS_D11,RS_D12,RS_D13,RS_D14,RS_D15],first_mm_imreg,[]);
      end;


    procedure tarmcgarm.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_MMREGISTER].free;
        inherited done_register_allocators;
      end;


     procedure tarmcgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);
       var
          imm_shift : byte;
          l : tasmlabel;
          hr : treference;
       begin
          if not(size in [OS_8,OS_S8,OS_16,OS_S16,OS_32,OS_S32]) then
            internalerror(2002090902);
          if is_shifter_const(a,imm_shift) then
            list.concat(taicpu.op_reg_const(A_MOV,reg,a))
          else if is_shifter_const(not(a),imm_shift) then
            list.concat(taicpu.op_reg_const(A_MVN,reg,not(a)))
          { loading of constants with mov and orr }
          else if (is_shifter_const(a-byte(a),imm_shift)) then
            begin
              list.concat(taicpu.op_reg_const(A_MOV,reg,a-byte(a)));
              list.concat(taicpu.op_reg_reg_const(A_ORR,reg,reg,byte(a)));
            end
          else if (is_shifter_const(a-word(a),imm_shift)) and (is_shifter_const(word(a),imm_shift)) then
            begin
              list.concat(taicpu.op_reg_const(A_MOV,reg,a-word(a)));
              list.concat(taicpu.op_reg_reg_const(A_ORR,reg,reg,word(a)));
            end
          else if (is_shifter_const(a-(dword(a) shl 8) shr 8,imm_shift)) and (is_shifter_const((dword(a) shl 8) shr 8,imm_shift)) then
            begin
              list.concat(taicpu.op_reg_const(A_MOV,reg,a-(dword(a) shl 8) shr 8));
              list.concat(taicpu.op_reg_reg_const(A_ORR,reg,reg,(dword(a) shl 8) shr 8));
            end
          else
            begin
               reference_reset(hr,4);

               current_asmdata.getjumplabel(l);
               cg.a_label(current_procinfo.aktlocaldata,l);
               hr.symboldata:=current_procinfo.aktlocaldata.last;
               current_procinfo.aktlocaldata.concat(tai_const.Create_32bit(longint(a)));

               hr.symbol:=l;
               hr.base:=NR_PC;
               list.concat(taicpu.op_reg_ref(A_LDR,reg,hr));
            end;
       end;


     procedure tarmcgarm.a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);
       var
         oppostfix:toppostfix;
         usedtmpref: treference;
         tmpreg,tmpreg2 : tregister;
         so : tshifterop;
         dir : integer;
       begin
         if (TCGSize2Size[FromSize] >= TCGSize2Size[ToSize]) then
           FromSize := ToSize;
         case FromSize of
           { signed integer registers }
           OS_8:
             oppostfix:=PF_B;
           OS_S8:
             oppostfix:=PF_SB;
           OS_16:
             oppostfix:=PF_H;
           OS_S16:
             oppostfix:=PF_SH;
           OS_32,
           OS_S32:
             oppostfix:=PF_None;
           else
             InternalError(200308297);
         end;
         if (ref.alignment in [1,2]) and (ref.alignment<tcgsize2size[fromsize]) then
           begin
             if target_info.endian=endian_big then
               dir:=-1
             else
               dir:=1;
             case FromSize of
               OS_16,OS_S16:
                 begin
                   { only complicated references need an extra loadaddr }
                   if assigned(ref.symbol) or
                     (ref.index<>NR_NO) or
                     (ref.offset<-4095) or
                     (ref.offset>4094) or
                     { sometimes the compiler reused registers }
                     (reg=ref.index) or
                     (reg=ref.base) then
                     begin
                       tmpreg2:=getintregister(list,OS_INT);
                       a_loadaddr_ref_reg(list,ref,tmpreg2);
                       reference_reset_base(usedtmpref,tmpreg2,0,ref.alignment);
                     end
                   else
                     usedtmpref:=ref;

                   if target_info.endian=endian_big then
                     inc(usedtmpref.offset,1);
                   shifterop_reset(so);so.shiftmode:=SM_LSL;so.shiftimm:=8;
                   tmpreg:=getintregister(list,OS_INT);
                   a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,reg);
                   inc(usedtmpref.offset,dir);
                   if FromSize=OS_16 then
                     a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg)
                   else
                     a_internal_load_ref_reg(list,OS_S8,OS_S8,usedtmpref,tmpreg);
                   list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg,so));
                 end;
               OS_32,OS_S32:
                 begin
                   tmpreg:=getintregister(list,OS_INT);

                   { only complicated references need an extra loadaddr }
                   if assigned(ref.symbol) or
                     (ref.index<>NR_NO) or
                     (ref.offset<-4095) or
                     (ref.offset>4092) or
                     { sometimes the compiler reused registers }
                     (reg=ref.index) or
                     (reg=ref.base) then
                     begin
                       tmpreg2:=getintregister(list,OS_INT);
                       a_loadaddr_ref_reg(list,ref,tmpreg2);
                       reference_reset_base(usedtmpref,tmpreg2,0,ref.alignment);
                     end
                   else
                     usedtmpref:=ref;

                   shifterop_reset(so);so.shiftmode:=SM_LSL;
                   if ref.alignment=2 then
                     begin
                       if target_info.endian=endian_big then
                         inc(usedtmpref.offset,2);
                       a_internal_load_ref_reg(list,OS_16,OS_16,usedtmpref,reg);
                       inc(usedtmpref.offset,dir*2);
                       a_internal_load_ref_reg(list,OS_16,OS_16,usedtmpref,tmpreg);
                       so.shiftimm:=16;
                       list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg,so));
                     end
                   else
                     begin
                       tmpreg2:=getintregister(list,OS_INT);
                       if target_info.endian=endian_big then
                         inc(usedtmpref.offset,3);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,reg);

                       inc(usedtmpref.offset,dir);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);

                       inc(usedtmpref.offset,dir);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg2);

                       so.shiftimm:=8;
                       list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg,so));

                       inc(usedtmpref.offset,dir);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);

                       so.shiftimm:=16;
                       list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg2,so));

                       so.shiftimm:=24;
                       list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg,so));
                     end;
                 end
               else
                 handle_load_store(list,A_LDR,oppostfix,reg,ref);
             end;
           end
         else
           handle_load_store(list,A_LDR,oppostfix,reg,ref);

         if (fromsize=OS_S8) and (tosize = OS_16) then
           a_load_reg_reg(list,OS_16,OS_32,reg,reg);
       end;


    procedure tcgarm.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const paraloc : TCGPara);
      var
        ref: treference;
      begin
        paraloc.check_simple_location;
        paramanager.allocparaloc(list,paraloc.location);
        case paraloc.location^.loc of
          LOC_REGISTER,LOC_CREGISTER:
            a_load_const_reg(list,size,a,paraloc.location^.register);
          LOC_REFERENCE:
            begin
               reference_reset(ref,paraloc.alignment);
               ref.base:=paraloc.location^.reference.index;
               ref.offset:=paraloc.location^.reference.offset;
               a_load_const_ref(list,size,a,ref);
            end;
          else
            internalerror(2002081101);
        end;
      end;


    procedure tcgarm.a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const paraloc : TCGPara);
      var
        tmpref, ref: treference;
        location: pcgparalocation;
        sizeleft: aint;
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
                  reference_reset_base(ref,location^.reference.index,location^.reference.offset,paraloc.alignment);
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
              LOC_FPUREGISTER,LOC_CFPUREGISTER:
                case location^.size of
                   OS_F32, OS_F64:
                     a_loadfpu_ref_reg(list,location^.size,location^.size,tmpref,location^.register);
                   else
                     internalerror(2002072801);
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


    procedure tcgarm.a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const paraloc : TCGPara);
      var
        ref: treference;
        tmpreg: tregister;
      begin
        paraloc.check_simple_location;
        paramanager.allocparaloc(list,paraloc.location);
        case paraloc.location^.loc of
          LOC_REGISTER,LOC_CREGISTER:
            a_loadaddr_ref_reg(list,r,paraloc.location^.register);
          LOC_REFERENCE:
            begin
              reference_reset(ref,paraloc.alignment);
              ref.base := paraloc.location^.reference.index;
              ref.offset := paraloc.location^.reference.offset;
              tmpreg := getintregister(list,OS_ADDR);
              a_loadaddr_ref_reg(list,r,tmpreg);
              a_load_reg_ref(list,OS_ADDR,OS_ADDR,tmpreg,ref);
            end;
          else
            internalerror(2002080701);
        end;
      end;


    procedure tcgarm.a_call_name(list : TAsmList;const s : string; weak: boolean);
      var
        branchopcode: tasmop;
      begin
        { check not really correct: should only be used for non-Thumb cpus }
        if (current_settings.cputype<cpu_armv5) or
           (current_settings.cputype in cpu_thumb2) then
          branchopcode:=A_BL
        else
          branchopcode:=A_BLX;
        if target_info.system<>system_arm_darwin then
          if not weak then
            list.concat(taicpu.op_sym(branchopcode,current_asmdata.RefAsmSymbol(s)))
          else
            list.concat(taicpu.op_sym(branchopcode,current_asmdata.WeakRefAsmSymbol(s)))
        else
          list.concat(taicpu.op_sym(branchopcode,get_darwin_call_stub(s,weak)));
{
        the compiler does not properly set this flag anymore in pass 1, and
        for now we only need it after pass 2 (I hope) (JM)
          if not(pi_do_call in current_procinfo.flags) then
            internalerror(2003060703);
}
        include(current_procinfo.flags,pi_do_call);
      end;


    procedure tcgarm.a_call_reg(list : TAsmList;reg: tregister);
      begin
        { check not really correct: should only be used for non-Thumb cpus }
        if (current_settings.cputype<cpu_armv5) then
          begin
            list.concat(taicpu.op_reg_reg(A_MOV,NR_R14,NR_PC));
            list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,reg));
          end
        else
          list.concat(taicpu.op_reg(A_BLX, reg));
{
        the compiler does not properly set this flag anymore in pass 1, and
        for now we only need it after pass 2 (I hope) (JM)
          if not(pi_do_call in current_procinfo.flags) then
            internalerror(2003060703);
}
        include(current_procinfo.flags,pi_do_call);
      end;


    procedure tcgarm.a_call_ref(list : TAsmList;ref: treference);
      begin
        a_reg_alloc(list,NR_R12);
        a_load_ref_reg(list,OS_ADDR,OS_ADDR,ref,NR_R12);
        a_call_reg(list,NR_R12);
        a_reg_dealloc(list,NR_R12);
        include(current_procinfo.flags,pi_do_call);
      end;


     procedure tcgarm.a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister);
       begin
          a_op_const_reg_reg(list,op,size,a,reg,reg);
       end;


     procedure tcgarm.a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister);
       begin
         case op of
           OP_NEG:
             list.concat(taicpu.op_reg_reg_const(A_RSB,dst,src,0));
           OP_NOT:
             begin
               list.concat(taicpu.op_reg_reg(A_MVN,dst,src));
               case size of
                 OS_8 :
                   a_op_const_reg_reg(list,OP_AND,OS_INT,$ff,dst,dst);
                 OS_16 :
                   a_op_const_reg_reg(list,OP_AND,OS_INT,$ffff,dst,dst);
               end;
             end
           else
             a_op_reg_reg_reg(list,op,OS_32,src,dst,dst);
         end;
       end;


    const
      op_reg_reg_opcg2asmop: array[TOpCG] of tasmop =
        (A_NONE,A_MOV,A_ADD,A_AND,A_NONE,A_NONE,A_MUL,A_MUL,A_NONE,A_NONE,A_ORR,
         A_NONE,A_NONE,A_NONE,A_SUB,A_EOR,A_NONE,A_NONE);


    procedure tcgarm.a_op_const_reg_reg(list: TAsmList; op: TOpCg;
      size: tcgsize; a: tcgint; src, dst: tregister);
      var
        ovloc : tlocation;
      begin
        a_op_const_reg_reg_checkoverflow(list,op,size,a,src,dst,false,ovloc);
      end;


    procedure tcgarm.a_op_reg_reg_reg(list: TAsmList; op: TOpCg;
      size: tcgsize; src1, src2, dst: tregister);
      var
        ovloc : tlocation;
      begin
        a_op_reg_reg_reg_checkoverflow(list,op,size,src1,src2,dst,false,ovloc);
      end;


    procedure tcgarm.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
      var
        shift : byte;
        tmpreg : tregister;
        so : tshifterop;
        l1 : longint;
      begin
        ovloc.loc:=LOC_VOID;
        if {$ifopt R+}(a<>-2147483648) and{$endif} is_shifter_const(-a,shift) then
          case op of
            OP_ADD:
              begin
                op:=OP_SUB;
                a:=aint(dword(-a));
              end;
            OP_SUB:
              begin
                op:=OP_ADD;
                a:=aint(dword(-a));
              end
          end;

        if is_shifter_const(a,shift) and not(op in [OP_IMUL,OP_MUL]) then
          case op of
            OP_NEG,OP_NOT:
              internalerror(200308281);
            OP_SHL:
              begin
                if a>32 then
                  internalerror(200308294);
                if a<>0 then
                  begin
                    shifterop_reset(so);
                    so.shiftmode:=SM_LSL;
                    so.shiftimm:=a;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                  end
                else
                 list.concat(taicpu.op_reg_reg(A_MOV,dst,src));
              end;
            OP_ROL:
              begin
                if a>32 then
                  internalerror(200308294);
                if a<>0 then
                  begin
                    shifterop_reset(so);
                    so.shiftmode:=SM_ROR;
                    so.shiftimm:=32-a;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                  end
                else
                 list.concat(taicpu.op_reg_reg(A_MOV,dst,src));
              end;
            OP_ROR:
              begin
                if a>32 then
                  internalerror(200308294);
                if a<>0 then
                  begin
                    shifterop_reset(so);
                    so.shiftmode:=SM_ROR;
                    so.shiftimm:=a;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                  end
                else
                 list.concat(taicpu.op_reg_reg(A_MOV,dst,src));
              end;
            OP_SHR:
              begin
                if a>32 then
                  internalerror(200308292);
                shifterop_reset(so);
                if a<>0 then
                  begin
                    so.shiftmode:=SM_LSR;
                    so.shiftimm:=a;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                  end
                else
                 list.concat(taicpu.op_reg_reg(A_MOV,dst,src));
              end;
            OP_SAR:
              begin
                if a>32 then
                  internalerror(200308298);
                if a<>0 then
                  begin
                    shifterop_reset(so);
                    so.shiftmode:=SM_ASR;
                    so.shiftimm:=a;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                  end
                else
                 list.concat(taicpu.op_reg_reg(A_MOV,dst,src));
              end;
            else
              {if (op in [OP_SUB, OP_ADD]) and
                 ((a < 0) or
                  (a > 4095)) then
                begin
                  tmpreg:=getintregister(list,size);
                  list.concat(taicpu.op_reg_const(A_MOVT, tmpreg, (a shr 16) and $FFFF));
                  list.concat(taicpu.op_reg_const(A_MOV, tmpreg, a and $FFFF));
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(op_reg_reg_opcg2asmop[op],dst,src,tmpreg),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))
                   ));
                end
              else}
              list.concat(setoppostfix(
                  taicpu.op_reg_reg_const(op_reg_reg_opcg2asmop[op],dst,src,a),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))
              ));
              if (cgsetflags or setflags) and (size in [OS_8,OS_16,OS_32]) then
                begin
                  ovloc.loc:=LOC_FLAGS;
                  case op of
                    OP_ADD:
                      ovloc.resflags:=F_CS;
                    OP_SUB:
                      ovloc.resflags:=F_CC;
                  end;
                end;
          end
        else
          begin
            { there could be added some more sophisticated optimizations }
            if (op in [OP_MUL,OP_IMUL,OP_DIV,OP_IDIV]) and (a=1) then
              a_load_reg_reg(list,size,size,src,dst)
            else if (op in [OP_MUL,OP_IMUL]) and (a=0) then
              a_load_const_reg(list,size,0,dst)
            else if (op in [OP_IMUL,OP_IDIV]) and (a=-1) then
              a_op_reg_reg(list,OP_NEG,size,src,dst)
            { we do this here instead in the peephole optimizer because
              it saves us a register }
            else if (op in [OP_MUL,OP_IMUL]) and ispowerof2(a,l1) and not(cgsetflags or setflags) then
              a_op_const_reg_reg(list,OP_SHL,size,l1,src,dst)
            { for example : b=a*5 -> b=a*4+a with add instruction and shl }
            else if (op in [OP_MUL,OP_IMUL]) and ispowerof2(a-1,l1) and not(cgsetflags or setflags) then
              begin
                if l1>32 then{roozbeh does this ever happen?}
                  internalerror(200308296);
                shifterop_reset(so);
                so.shiftmode:=SM_LSL;
                so.shiftimm:=l1;
                list.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,dst,src,src,so));
              end
            { for example : b=a*7 -> b=a*8-a with rsb instruction and shl }
            else if (op in [OP_MUL,OP_IMUL]) and ispowerof2(a+1,l1) and not(cgsetflags or setflags) then
              begin
                if l1>32 then{does this ever happen?}
                  internalerror(201205181);
                shifterop_reset(so);
                so.shiftmode:=SM_LSL;
                so.shiftimm:=l1;
                list.concat(taicpu.op_reg_reg_reg_shifterop(A_RSB,dst,src,src,so));
              end
            { BIC clears the specified bits, while AND keeps them, using BIC allows to use a
              broader range of shifterconstants.}
            else if (op = OP_AND) and is_shifter_const(not(dword(a)),shift) then
              list.concat(taicpu.op_reg_reg_const(A_BIC,dst,src,not(dword(a))))
            else
              begin
                tmpreg:=getintregister(list,size);
                a_load_const_reg(list,size,a,tmpreg);
                a_op_reg_reg_reg_checkoverflow(list,op,size,tmpreg,src,dst,setflags,ovloc);
              end;
          end;
        maybeadjustresult(list,op,size,dst);
      end;


    procedure tcgarm.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);
      var
        so : tshifterop;
        tmpreg,overflowreg : tregister;
        asmop : tasmop;
      begin
        ovloc.loc:=LOC_VOID;
        case op of
          OP_NEG,OP_NOT,
          OP_DIV,OP_IDIV:
            internalerror(200308281);
          OP_SHL:
            begin
              shifterop_reset(so);
              so.rs:=src1;
              so.shiftmode:=SM_LSL;
              list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src2,so));
            end;
          OP_SHR:
            begin
              shifterop_reset(so);
              so.rs:=src1;
              so.shiftmode:=SM_LSR;
              list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src2,so));
            end;
          OP_SAR:
            begin
              shifterop_reset(so);
              so.rs:=src1;
              so.shiftmode:=SM_ASR;
              list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src2,so));
            end;
          OP_ROL:
            begin
              if not(size in [OS_32,OS_S32]) then
                internalerror(2008072801);
              { simulate ROL by ror'ing 32-value }
              tmpreg:=getintregister(list,OS_32);
              list.concat(taicpu.op_reg_const(A_MOV,tmpreg,32));
              list.concat(taicpu.op_reg_reg_reg(A_SUB,src1,tmpreg,src1));
              shifterop_reset(so);
              so.rs:=src1;
              so.shiftmode:=SM_ROR;
              list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src2,so));
            end;
          OP_ROR:
            begin
              if not(size in [OS_32,OS_S32]) then
                internalerror(2008072802);
              shifterop_reset(so);
              so.rs:=src1;
              so.shiftmode:=SM_ROR;
              list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src2,so));
            end;
          OP_IMUL,
          OP_MUL:
            begin
              if cgsetflags or setflags then
                begin
                  overflowreg:=getintregister(list,size);
                  if op=OP_IMUL then
                    asmop:=A_SMULL
                  else
                    asmop:=A_UMULL;
                  { the arm doesn't allow that rd and rm are the same }
                  if dst=src2 then
                    begin
                      if dst<>src1 then
                        list.concat(taicpu.op_reg_reg_reg_reg(asmop,dst,overflowreg,src1,src2))
                      else
                        begin
                          tmpreg:=getintregister(list,size);
                          a_load_reg_reg(list,size,size,src2,dst);
                          list.concat(taicpu.op_reg_reg_reg_reg(asmop,dst,overflowreg,tmpreg,src1));
                        end;
                    end
                  else
                    list.concat(taicpu.op_reg_reg_reg_reg(asmop,dst,overflowreg,src2,src1));
                  if op=OP_IMUL then
                    begin
                      shifterop_reset(so);
                      so.shiftmode:=SM_ASR;
                      so.shiftimm:=31;
                      list.concat(taicpu.op_reg_reg_shifterop(A_CMP,overflowreg,dst,so));
                    end
                  else
                    list.concat(taicpu.op_reg_const(A_CMP,overflowreg,0));

                   ovloc.loc:=LOC_FLAGS;
                   ovloc.resflags:=F_NE;
                end
              else
                begin
                  { the arm doesn't allow that rd and rm are the same }
                  if dst=src2 then
                    begin
                      if dst<>src1 then
                        list.concat(taicpu.op_reg_reg_reg(A_MUL,dst,src1,src2))
                      else
                        begin
                          tmpreg:=getintregister(list,size);
                          a_load_reg_reg(list,size,size,src2,dst);
                          list.concat(taicpu.op_reg_reg_reg(A_MUL,dst,tmpreg,src1));
                        end;
                    end
                  else
                    list.concat(taicpu.op_reg_reg_reg(A_MUL,dst,src2,src1));
                end;
            end;
          else
            list.concat(setoppostfix(
                taicpu.op_reg_reg_reg(op_reg_reg_opcg2asmop[op],dst,src2,src1),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))
              ));
        end;
        maybeadjustresult(list,op,size,dst);
      end;


    function tcgarm.handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference;
      var
        tmpreg : tregister;
        tmpref : treference;
        l : tasmlabel;
      begin
        tmpreg:=NR_NO;

        { Be sure to have a base register }
        if (ref.base=NR_NO) then
          begin
            if ref.shiftmode<>SM_None then
              internalerror(200308294);
            ref.base:=ref.index;
            ref.index:=NR_NO;
          end;

        { absolute symbols can't be handled directly, we've to store the symbol reference
          in the text segment and access it pc relative

          For now, we assume that references where base or index equals to PC are already
          relative, all other references are assumed to be absolute and thus they need
          to be handled extra.

          A proper solution would be to change refoptions to a set and store the information
          if the symbol is absolute or relative there.
        }

        if (assigned(ref.symbol) and
            not(is_pc(ref.base)) and
            not(is_pc(ref.index))
           ) or
           { [#xxx] isn't a valid address operand }
           ((ref.base=NR_NO) and (ref.index=NR_NO)) or
           (ref.offset<-4095) or
           (ref.offset>4095) or
           ((oppostfix in [PF_SB,PF_H,PF_SH]) and
            ((ref.offset<-255) or
             (ref.offset>255)
            )
           ) or
           ((op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) and
            ((ref.offset<-1020) or
             (ref.offset>1020) or
             ((abs(ref.offset) mod 4)<>0) or
             { the usual pc relative symbol handling assumes possible offsets of +/- 4095 }
             assigned(ref.symbol)
            )
           ) then
          begin
            reference_reset(tmpref,4);

            { load symbol }
            tmpreg:=getintregister(list,OS_INT);
            if assigned(ref.symbol) then
              begin
                current_asmdata.getjumplabel(l);
                cg.a_label(current_procinfo.aktlocaldata,l);
                tmpref.symboldata:=current_procinfo.aktlocaldata.last;

                current_procinfo.aktlocaldata.concat(tai_const.create_sym_offset(ref.symbol,ref.offset));

                { load consts entry }
                tmpref.symbol:=l;
                tmpref.base:=NR_R15;
                list.concat(taicpu.op_reg_ref(A_LDR,tmpreg,tmpref));

                { in case of LDF/STF, we got rid of the NR_R15 }
                if is_pc(ref.base) then
                  ref.base:=NR_NO;
                if is_pc(ref.index) then
                  ref.index:=NR_NO;
              end
            else
              a_load_const_reg(list,OS_ADDR,ref.offset,tmpreg);

            if (ref.base<>NR_NO) then
              begin
                if ref.index<>NR_NO then
                  begin
                    list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,tmpreg));
                    ref.base:=tmpreg;
                  end
                else
                  begin
                    ref.index:=tmpreg;
                    ref.shiftimm:=0;
                    ref.signindex:=1;
                    ref.shiftmode:=SM_None;
                  end;
              end
            else
              ref.base:=tmpreg;
            ref.offset:=0;
            ref.symbol:=nil;
          end;

        if (ref.base<>NR_NO) and (ref.index<>NR_NO) and (ref.offset<>0) then
          begin
            if tmpreg<>NR_NO then
              a_op_const_reg_reg(list,OP_ADD,OS_ADDR,ref.offset,tmpreg,tmpreg)
            else
              begin
                tmpreg:=getintregister(list,OS_ADDR);
                a_op_const_reg_reg(list,OP_ADD,OS_ADDR,ref.offset,ref.base,tmpreg);
                ref.base:=tmpreg;
              end;
            ref.offset:=0;
          end;

        { floating point operations have only limited references
          we expect here, that a base is already set }
        if (op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) and (ref.index<>NR_NO) then
          begin
            if ref.shiftmode<>SM_none then
              internalerror(200309121);
            if tmpreg<>NR_NO then
              begin
                if ref.base=tmpreg then
                  begin
                    if ref.signindex<0 then
                      list.concat(taicpu.op_reg_reg_reg(A_SUB,tmpreg,tmpreg,ref.index))
                    else
                      list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,tmpreg,ref.index));
                    ref.index:=NR_NO;
                  end
                else
                  begin
                    if ref.index<>tmpreg then
                      internalerror(200403161);
                    if ref.signindex<0 then
                      list.concat(taicpu.op_reg_reg_reg(A_SUB,tmpreg,ref.base,tmpreg))
                    else
                      list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,tmpreg));
                    ref.base:=tmpreg;
                    ref.index:=NR_NO;
                  end;
              end
            else
              begin
                tmpreg:=getintregister(list,OS_ADDR);
                list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,ref.index));
                ref.base:=tmpreg;
                ref.index:=NR_NO;
              end;
          end;
        list.concat(setoppostfix(taicpu.op_reg_ref(op,reg,ref),oppostfix));
        Result := ref;
      end;


     procedure tcgarm.a_load_reg_ref(list : TAsmList; fromsize, tosize: tcgsize; reg : tregister;const ref : treference);
       var
         oppostfix:toppostfix;
         usedtmpref: treference;
         tmpreg : tregister;
         so : tshifterop;
         dir : integer;
       begin
         if (TCGSize2Size[FromSize] >= TCGSize2Size[ToSize]) then
           FromSize := ToSize;
         case ToSize of
           { signed integer registers }
           OS_8,
           OS_S8:
             oppostfix:=PF_B;
           OS_16,
           OS_S16:
             oppostfix:=PF_H;
           OS_32,
           OS_S32,
           { for vfp value stored in integer register }
           OS_F32:
             oppostfix:=PF_None;
           else
             InternalError(200308299);
         end;
         if (ref.alignment in [1,2]) and (ref.alignment<tcgsize2size[tosize]) then
           begin
             if target_info.endian=endian_big then
               dir:=-1
             else
               dir:=1;
             case FromSize of
               OS_16,OS_S16:
                 begin
                   shifterop_reset(so);so.shiftmode:=SM_LSR;so.shiftimm:=8;
                   tmpreg:=getintregister(list,OS_INT);
                   usedtmpref:=ref;
                   if target_info.endian=endian_big then
                     inc(usedtmpref.offset,1);
                   usedtmpref:=a_internal_load_reg_ref(list,OS_8,OS_8,reg,usedtmpref);
                   inc(usedtmpref.offset,dir);
                   list.concat(taicpu.op_reg_reg_shifterop(A_MOV,tmpreg,reg,so));
                   a_internal_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref);
                 end;
               OS_32,OS_S32:
                 begin
                   tmpreg:=getintregister(list,OS_INT);
                   usedtmpref:=ref;
                   shifterop_reset(so);so.shiftmode:=SM_LSR;
                   if ref.alignment=2 then
                     begin
                       so.shiftimm:=16;
                       if target_info.endian=endian_big then
                         inc(usedtmpref.offset,2);
                       usedtmpref:=a_internal_load_reg_ref(list,OS_16,OS_16,reg,usedtmpref);
                       list.concat(taicpu.op_reg_reg_shifterop(A_MOV,tmpreg,reg,so));
                       inc(usedtmpref.offset,dir*2);
                       a_internal_load_reg_ref(list,OS_16,OS_16,tmpreg,usedtmpref);
                     end
                   else
                     begin
                       so.shiftimm:=8;
                       if target_info.endian=endian_big then
                         inc(usedtmpref.offset,3);
                       usedtmpref:=a_internal_load_reg_ref(list,OS_8,OS_8,reg,usedtmpref);
                       list.concat(taicpu.op_reg_reg_shifterop(A_MOV,tmpreg,reg,so));
                       inc(usedtmpref.offset,dir);
                       a_internal_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref);
                       list.concat(taicpu.op_reg_reg_shifterop(A_MOV,tmpreg,tmpreg,so));
                       inc(usedtmpref.offset,dir);
                       a_internal_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref);
                       list.concat(taicpu.op_reg_reg_shifterop(A_MOV,tmpreg,tmpreg,so));
                       inc(usedtmpref.offset,dir);
                       a_internal_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref);
                     end;
                 end
               else
                 handle_load_store(list,A_STR,oppostfix,reg,ref);
             end;
           end
         else
           handle_load_store(list,A_STR,oppostfix,reg,ref);
       end;


     function tcgarm.a_internal_load_reg_ref(list : TAsmList; fromsize, tosize: tcgsize; reg : tregister;const ref : treference):treference;
       var
         oppostfix:toppostfix;
       begin
         case ToSize of
           { signed integer registers }
           OS_8,
           OS_S8:
             oppostfix:=PF_B;
           OS_16,
           OS_S16:
             oppostfix:=PF_H;
           OS_32,
           OS_S32:
             oppostfix:=PF_None;
           else
             InternalError(2003082910);
         end;
         result:=handle_load_store(list,A_STR,oppostfix,reg,ref);
       end;


     function tcgarm.a_internal_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister):treference;
       var
         oppostfix:toppostfix;
       begin
         case FromSize of
           { signed integer registers }
           OS_8:
             oppostfix:=PF_B;
           OS_S8:
             oppostfix:=PF_SB;
           OS_16:
             oppostfix:=PF_H;
           OS_S16:
             oppostfix:=PF_SH;
           OS_32,
           OS_S32:
             oppostfix:=PF_None;
           else
             InternalError(200308291);
         end;
         result:=handle_load_store(list,A_LDR,oppostfix,reg,ref);
       end;

     procedure tcgarm.a_load_reg_reg(list : TAsmList; fromsize, tosize : tcgsize;reg1,reg2 : tregister);
       var
         so : tshifterop;

       procedure do_shift(shiftmode : tshiftmode; shiftimm : byte; reg : tregister);
         begin
           so.shiftmode:=shiftmode;
           so.shiftimm:=shiftimm;
           list.concat(taicpu.op_reg_reg_shifterop(A_MOV,reg2,reg,so));
         end;

       var
         instr: taicpu;
         conv_done: boolean;
       begin
         if (tcgsize2size[fromsize]>32) or (tcgsize2size[tosize]>32) or (fromsize=OS_NO) or (tosize=OS_NO) then
           internalerror(2002090901);

         conv_done:=false;
         if tosize<>fromsize then
           begin
             shifterop_reset(so);
             conv_done:=true;
             if tcgsize2size[tosize]<=tcgsize2size[fromsize] then
               fromsize:=tosize;
             case fromsize of
               OS_8:
                 list.concat(taicpu.op_reg_reg_const(A_AND,reg2,reg1,$ff));
               OS_S8:
                 begin
                   do_shift(SM_LSL,24,reg1);
                   if tosize=OS_16 then
                     begin
                       do_shift(SM_ASR,8,reg2);
                       do_shift(SM_LSR,16,reg2);
                     end
                   else
                     do_shift(SM_ASR,24,reg2);
                 end;
               OS_16:
                 begin
                   do_shift(SM_LSL,16,reg1);
                   do_shift(SM_LSR,16,reg2);
                 end;
               OS_S16:
                 begin
                   do_shift(SM_LSL,16,reg1);
                   do_shift(SM_ASR,16,reg2)
                 end;
               else
                 conv_done:=false;
             end;
           end;
         if not conv_done and (reg1<>reg2) then
           begin
             { same size, only a register mov required }
             instr:=taicpu.op_reg_reg(A_MOV,reg2,reg1);
             list.Concat(instr);
             { Notify the register allocator that we have written a move instruction so
               it can try to eliminate it. }
             add_move_instruction(instr);
           end;
       end;


    procedure tcgarm.a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);
      var
         href,href2 : treference;
         hloc : pcgparalocation;
      begin
        href:=ref;
        hloc:=paraloc.location;
        while assigned(hloc) do
          begin
            case hloc^.loc of
              LOC_FPUREGISTER,LOC_CFPUREGISTER:
                begin
                  paramanager.allocparaloc(list,paraloc.location);
                  a_loadfpu_ref_reg(list,size,size,ref,hloc^.register);
                end;
              LOC_REGISTER :
                case hloc^.size of
                  OS_32,
                  OS_F32:
                    begin
                      paramanager.allocparaloc(list,paraloc.location);
                      a_load_ref_reg(list,OS_32,OS_32,href,hloc^.register);
                    end;
                  OS_64,
                  OS_F64:
                    cg64.a_load64_ref_cgpara(list,href,paraloc);
                  else
                    a_load_ref_reg(list,hloc^.size,hloc^.size,href,hloc^.register);
                end;
              LOC_REFERENCE :
                begin
                  reference_reset_base(href2,hloc^.reference.index,hloc^.reference.offset,paraloc.alignment);
                  { concatcopy should choose the best way to copy the data }
                  g_concatcopy(list,href,href2,tcgsize2size[hloc^.size]);
                end;
              else
                internalerror(200408241);
           end;
           inc(href.offset,tcgsize2size[hloc^.size]);
           hloc:=hloc^.next;
         end;
      end;


     procedure tcgarm.a_loadfpu_reg_reg(list: TAsmList; fromsize,tosize: tcgsize; reg1, reg2: tregister);
       begin
         list.concat(setoppostfix(taicpu.op_reg_reg(A_MVF,reg2,reg1),cgsize2fpuoppostfix[tosize]));
       end;


     procedure tcgarm.a_loadfpu_ref_reg(list: TAsmList; fromsize,tosize: tcgsize; const ref: treference; reg: tregister);
       var
         oppostfix:toppostfix;
       begin
         case fromsize of
           OS_32,
           OS_F32:
             oppostfix:=PF_S;
           OS_64,
           OS_F64:
             oppostfix:=PF_D;
           OS_F80:
             oppostfix:=PF_E;
           else
             InternalError(200309021);
         end;
         handle_load_store(list,A_LDF,oppostfix,reg,ref);
         if fromsize<>tosize then
           a_loadfpu_reg_reg(list,fromsize,tosize,reg,reg);
       end;


     procedure tcgarm.a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference);
       var
         oppostfix:toppostfix;
       begin
         case tosize of
           OS_F32:
             oppostfix:=PF_S;
           OS_F64:
             oppostfix:=PF_D;
           OS_F80:
             oppostfix:=PF_E;
           else
             InternalError(200309022);
         end;
         handle_load_store(list,A_STF,oppostfix,reg,ref);
       end;


    {  comparison operations }
    procedure tcgarm.a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister;
      l : tasmlabel);
      var
        tmpreg : tregister;
        b : byte;
      begin
        if is_shifter_const(a,b) then
          list.concat(taicpu.op_reg_const(A_CMP,reg,a))
        { CMN reg,0 and CMN reg,$80000000 are different from CMP reg,$ffffffff
          and CMP reg,$7fffffff regarding the flags according to the ARM manual }
        else if (a<>$7fffffff) and (a<>-1) and is_shifter_const(-a,b) then
          list.concat(taicpu.op_reg_const(A_CMN,reg,-a))
        else
          begin
            tmpreg:=getintregister(list,size);
            a_load_const_reg(list,size,a,tmpreg);
            list.concat(taicpu.op_reg_reg(A_CMP,reg,tmpreg));
          end;
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure tcgarm.a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; size: TCGSize; src, dst: TRegister);
      begin
        Comment(V_Error,'tcgarm.a_bit_scan_reg_reg method not implemented');
      end;

    procedure tcgarm.a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel);
      begin
        list.concat(taicpu.op_reg_reg(A_CMP,reg2,reg1));
        a_jmp_cond(list,cmp_op,l);
      end;


    procedure tcgarm.a_jmp_name(list : TAsmList;const s : string);
      var
        ai : taicpu;
      begin
        ai:=taicpu.op_sym(A_B,current_asmdata.RefAsmSymbol(s));
        ai.is_jmp:=true;
        list.concat(ai);
      end;


    procedure tcgarm.a_jmp_always(list : TAsmList;l: tasmlabel);
      var
        ai : taicpu;
      begin
        ai:=taicpu.op_sym(A_B,l);
        ai.is_jmp:=true;
        list.concat(ai);
      end;


    procedure tcgarm.a_jmp_flags(list : TAsmList;const f : TResFlags;l: tasmlabel);
      var
        ai : taicpu;
      begin
        ai:=setcondition(taicpu.op_sym(A_B,l),flags_to_cond(f));
        ai.is_jmp:=true;
        list.concat(ai);
      end;


    procedure tcgarm.g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister);
      begin
        list.concat(setcondition(taicpu.op_reg_const(A_MOV,reg,1),flags_to_cond(f)));
        list.concat(setcondition(taicpu.op_reg_const(A_MOV,reg,0),inverse_cond(flags_to_cond(f))));
      end;


    procedure tcgarm.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
      var
         ref : treference;
         shift : byte;
         firstfloatreg,lastfloatreg,
         r : byte;
         mmregs,
         regs, saveregs : tcpuregisterset;
         r7offset,
         stackmisalignment : pint;
         postfix: toppostfix;
      begin
        LocalSize:=align(LocalSize,4);
        { call instruction does not put anything on the stack }
        stackmisalignment:=0;
        if not(nostackframe) then
          begin
            firstfloatreg:=RS_NO;
            mmregs:=[];
            case current_settings.fputype of
              fpu_fpa,
              fpu_fpa10,
              fpu_fpa11:
                begin
                  { save floating point registers? }
                  regs:=rg[R_FPUREGISTER].used_in_proc-paramanager.get_volatile_registers_fpu(pocall_stdcall);
                  for r:=RS_F0 to RS_F7 do
                    if r in regs then
                      begin
                        if firstfloatreg=RS_NO then
                          firstfloatreg:=r;
                        lastfloatreg:=r;
                        inc(stackmisalignment,12);
                      end;
                end;
              fpu_vfpv2,
              fpu_vfpv3,
              fpu_vfpv3_d16:
                begin;
                  mmregs:=rg[R_MMREGISTER].used_in_proc-paramanager.get_volatile_registers_mm(pocall_stdcall);
                end;
            end;
            a_reg_alloc(list,NR_STACK_POINTER_REG);
            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              a_reg_alloc(list,NR_FRAME_POINTER_REG);
            { save int registers }
            reference_reset(ref,4);
            ref.index:=NR_STACK_POINTER_REG;
            ref.addressmode:=AM_PREINDEXED;
            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);
            if not(target_info.system in systems_darwin) then
              begin
                a_reg_alloc(list,NR_STACK_POINTER_REG);
                if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
                  begin
                    a_reg_alloc(list,NR_R12);
                    list.concat(taicpu.op_reg_reg(A_MOV,NR_R12,NR_STACK_POINTER_REG));
                  end;
                { the (old) ARM APCS requires saving both the stack pointer (to
                  crawl the stack) and the PC (to identify the function this
                  stack frame belongs to) -> also save R12 (= copy of R13 on entry)
                  and R15 -- still needs updating for EABI and Darwin, they don't
                  need that }
                if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
                  regs:=regs+[RS_FRAME_POINTER_REG,RS_R12,RS_R14,RS_R15]
                else
                  if (regs<>[]) or (pi_do_call in current_procinfo.flags) then
                    include(regs,RS_R14);
                if regs<>[] then
                   begin
                     for r:=RS_R0 to RS_R15 do
                       if r in regs then
                         inc(stackmisalignment,4);
                     list.concat(setoppostfix(taicpu.op_ref_regset(A_STM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_FD));
                   end;

                if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
                  begin
                    { the framepointer now points to the saved R15, so the saved
                      framepointer is at R11-12 (for get_caller_frame) }
                    list.concat(taicpu.op_reg_reg_const(A_SUB,NR_FRAME_POINTER_REG,NR_R12,4));
                    a_reg_dealloc(list,NR_R12);
                  end;
              end
            else
              begin
                { always save r14 if we use r7 as the framepointer, because
                  the parameter offsets are hardcoded in advance and always
                  assume that r14 sits on the stack right behind the saved r7
                }
                if current_procinfo.framepointer=NR_FRAME_POINTER_REG then
                  include(regs,RS_FRAME_POINTER_REG);
                if (regs<>[]) or (pi_do_call in current_procinfo.flags) then
                    include(regs,RS_R14);
                if regs<>[] then
                  begin
                    { on Darwin, you first have to save [r4-r7,lr], and then
                      [r8,r10,r11] and make r7 point to the previously saved
                      r7 so that you can perform a stack crawl based on it
                      ([r7] is previous stack frame, [r7+4] is return address
                    }
                    include(regs,RS_FRAME_POINTER_REG);
                    saveregs:=regs-[RS_R8,RS_R10,RS_R11];
                    r7offset:=0;
                    for r:=RS_R0 to RS_R15 do
                      if r in saveregs then
                        begin
                          inc(stackmisalignment,4);
                          if r<RS_FRAME_POINTER_REG then
                            inc(r7offset,4);
                        end;
                    { save the registers }
                    list.concat(setoppostfix(taicpu.op_ref_regset(A_STM,ref,R_INTREGISTER,R_SUBWHOLE,saveregs),PF_FD));
                    { make r7 point to the saved r7 (regardless of whether this
                      frame uses the framepointer, for backtrace purposes) }
                    if r7offset<>0 then
                      list.concat(taicpu.op_reg_reg_const(A_ADD,NR_FRAME_POINTER_REG,NR_R13,r7offset))
                    else
                      list.concat(taicpu.op_reg_reg(A_MOV,NR_R7,NR_R13));
                    { now save the rest (if any) }
                    saveregs:=regs-saveregs;
                    if saveregs<>[] then
                      begin
                        for r:=RS_R8 to RS_R11 do
                          if r in saveregs then
                            inc(stackmisalignment,4);
                        list.concat(setoppostfix(taicpu.op_ref_regset(A_STM,ref,R_INTREGISTER,R_SUBWHOLE,saveregs),PF_FD));
                      end;
                  end;
              end;

            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (LocalSize<>0) or
               ((stackmisalignment<>0) and
                ((pi_do_call in current_procinfo.flags) or
                 (po_assembler in current_procinfo.procdef.procoptions))) then
              begin
                localsize:=align(localsize+stackmisalignment,current_settings.alignment.localalignmax)-stackmisalignment;
                if not(is_shifter_const(localsize,shift)) then
                  begin
                    if current_procinfo.framepointer=NR_STACK_POINTER_REG then
                      a_reg_alloc(list,NR_R12);
                    a_load_const_reg(list,OS_ADDR,LocalSize,NR_R12);
                    list.concat(taicpu.op_reg_reg_reg(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_R12));
                    a_reg_dealloc(list,NR_R12);
                  end
                else
                  begin
                    a_reg_dealloc(list,NR_R12);
                    list.concat(taicpu.op_reg_reg_const(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize));
                  end;
              end;

            if (mmregs<>[]) or
               (firstfloatreg<>RS_NO) then
             begin
               reference_reset(ref,4);
               if (tg.direction*tarmprocinfo(current_procinfo).floatregstart>=1023) or
                  (current_settings.fputype in [fpu_vfpv2,fpu_vfpv3,fpu_vfpv3_d16]) then
                 begin
                   if not is_shifter_const(tarmprocinfo(current_procinfo).floatregstart,shift) then
                     begin
                       a_reg_alloc(list,NR_R12);
                       a_load_const_reg(list,OS_ADDR,-tarmprocinfo(current_procinfo).floatregstart,NR_R12);
                       list.concat(taicpu.op_reg_reg_reg(A_SUB,NR_R12,current_procinfo.framepointer,NR_R12));
                       a_reg_dealloc(list,NR_R12);
                     end
                   else
                     list.concat(taicpu.op_reg_reg_const(A_SUB,NR_R12,current_procinfo.framepointer,-tarmprocinfo(current_procinfo).floatregstart));
                   ref.base:=NR_R12;
                 end
               else
                 begin
                   ref.base:=current_procinfo.framepointer;
                   ref.offset:=tarmprocinfo(current_procinfo).floatregstart;
                 end;

               case current_settings.fputype of
                 fpu_fpa,
                 fpu_fpa10,
                 fpu_fpa11:
                   begin
                     list.concat(taicpu.op_reg_const_ref(A_SFM,newreg(R_FPUREGISTER,firstfloatreg,R_SUBWHOLE),
                       lastfloatreg-firstfloatreg+1,ref));
                   end;
                 fpu_vfpv2,
                 fpu_vfpv3,
                 fpu_vfpv3_d16:
                   begin
                     ref.index:=ref.base;
                     ref.base:=NR_NO;
                     { FSTMX is deprecated on ARMv6 and later }
                     if (current_settings.cputype<cpu_armv6) then
                       postfix:=PF_IAX
                     else
                       postfix:=PF_IAD;
                     list.concat(setoppostfix(taicpu.op_ref_regset(A_FSTM,ref,R_MMREGISTER,R_SUBFD,mmregs),postfix));
                   end;
               end;
             end;
        end;
      end;


    procedure tcgarm.g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean);
      var
         ref : treference;
         LocalSize : longint;
         firstfloatreg,lastfloatreg,
         r,
         shift : byte;
         mmregs,
         saveregs,
         regs : tcpuregisterset;
         stackmisalignment: pint;
         mmpostfix: toppostfix;
      begin
        if not(nostackframe) then
          begin
            stackmisalignment:=0;
            firstfloatreg:=RS_NO;
            mmregs:=[];
            case current_settings.fputype of
              fpu_fpa,
              fpu_fpa10,
              fpu_fpa11:
                begin
                  { restore floating point registers? }
                  regs:=rg[R_FPUREGISTER].used_in_proc-paramanager.get_volatile_registers_fpu(pocall_stdcall);
                  for r:=RS_F0 to RS_F7 do
                    if r in regs then
                      begin
                        if firstfloatreg=RS_NO then
                          firstfloatreg:=r;
                        lastfloatreg:=r;
                        { floating point register space is already included in
                          localsize below by calc_stackframe_size
                         inc(stackmisalignment,12);
                        }
                      end;
                end;
              fpu_vfpv2,
              fpu_vfpv3,
              fpu_vfpv3_d16:
                begin;
                  { restore vfp registers? }
                  mmregs:=rg[R_MMREGISTER].used_in_proc-paramanager.get_volatile_registers_mm(pocall_stdcall);
                end;
            end;

            if (firstfloatreg<>RS_NO) or
               (mmregs<>[]) then
              begin
                reference_reset(ref,4);
                if (tg.direction*tarmprocinfo(current_procinfo).floatregstart>=1023) or
                   (current_settings.fputype in [fpu_vfpv2,fpu_vfpv3,fpu_vfpv3_d16]) then
                  begin
                    if not is_shifter_const(tarmprocinfo(current_procinfo).floatregstart,shift) then
                      begin
                        a_reg_alloc(list,NR_R12);
                        a_load_const_reg(list,OS_ADDR,-tarmprocinfo(current_procinfo).floatregstart,NR_R12);
                        list.concat(taicpu.op_reg_reg_reg(A_SUB,NR_R12,current_procinfo.framepointer,NR_R12));
                        a_reg_dealloc(list,NR_R12);
                      end
                    else
                      list.concat(taicpu.op_reg_reg_const(A_SUB,NR_R12,current_procinfo.framepointer,-tarmprocinfo(current_procinfo).floatregstart));
                    ref.base:=NR_R12;
                  end
                else
                  begin
                    ref.base:=current_procinfo.framepointer;
                    ref.offset:=tarmprocinfo(current_procinfo).floatregstart;
                  end;
                case current_settings.fputype of
                  fpu_fpa,
                  fpu_fpa10,
                  fpu_fpa11:
                    begin
                      list.concat(taicpu.op_reg_const_ref(A_LFM,newreg(R_FPUREGISTER,firstfloatreg,R_SUBWHOLE),
                        lastfloatreg-firstfloatreg+1,ref));
                    end;
                  fpu_vfpv2,
                  fpu_vfpv3,
                  fpu_vfpv3_d16:
                    begin
                      ref.index:=ref.base;
                      ref.base:=NR_NO;
                      { FLDMX is deprecated on ARMv6 and later }
                      if (current_settings.cputype<cpu_armv6) then
                        mmpostfix:=PF_IAX
                      else
                        mmpostfix:=PF_IAD;
                      list.concat(setoppostfix(taicpu.op_ref_regset(A_FLDM,ref,R_MMREGISTER,R_SUBFD,mmregs),mmpostfix));
                    end;
                end;
              end;

            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall)        ;
            if (pi_do_call in current_procinfo.flags) or
               (regs<>[]) or
               ((target_info.system in systems_darwin) and
                (current_procinfo.framepointer<>NR_STACK_POINTER_REG)) then
              begin
                exclude(regs,RS_R14);
                include(regs,RS_R15);
                if (target_info.system in systems_darwin) then
                  include(regs,RS_FRAME_POINTER_REG);
              end;

            if not(target_info.system in systems_darwin) then
              begin
                { restore saved stack pointer to SP (R13) and saved lr to PC (R15).
                  The saved PC came after that but is discarded, since we restore
                  the stack pointer }
                if (current_procinfo.framepointer<>NR_STACK_POINTER_REG) then
                  regs:=regs+[RS_FRAME_POINTER_REG,RS_R13,RS_R15];
              end
            else
              begin
                { restore R8-R11 already if necessary (they've been stored
                  before the others) }
                saveregs:=regs*[RS_R8,RS_R10,RS_R11];
                if saveregs<>[] then
                  begin
                    reference_reset(ref,4);
                    ref.index:=NR_STACK_POINTER_REG;
                    ref.addressmode:=AM_PREINDEXED;
                    for r:=RS_R8 to RS_R11 do
                      if r in saveregs then
                        inc(stackmisalignment,4);
                    regs:=regs-saveregs;
                  end;
              end;
            for r:=RS_R0 to RS_R15 do
              if r in regs then
                inc(stackmisalignment,4);
            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) or
               (target_info.system in systems_darwin) then
              begin
                LocalSize:=current_procinfo.calc_stackframe_size;
                if (LocalSize<>0) or
                   ((stackmisalignment<>0) and
                    ((pi_do_call in current_procinfo.flags) or
                     (po_assembler in current_procinfo.procdef.procoptions))) then
                  begin
                    localsize:=align(localsize+stackmisalignment,current_settings.alignment.localalignmax)-stackmisalignment;
                    if not(is_shifter_const(LocalSize,shift)) then
                      begin
                        a_reg_alloc(list,NR_R12);
                        a_load_const_reg(list,OS_ADDR,LocalSize,NR_R12);
                        list.concat(taicpu.op_reg_reg_reg(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_R12));
                        a_reg_dealloc(list,NR_R12);
                      end
                    else
                      begin
                        list.concat(taicpu.op_reg_reg_const(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize));
                      end;
                  end;

                if (target_info.system in systems_darwin) and
                   (saveregs<>[]) then
                  list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,saveregs),PF_FD));

                if regs=[] then
                  begin
                    if (current_settings.cputype<cpu_armv5) then
                      list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14))
                    else
                      list.concat(taicpu.op_reg(A_BX,NR_R14))
                  end
                else
                  begin
                    reference_reset(ref,4);
                    ref.index:=NR_STACK_POINTER_REG;
                    ref.addressmode:=AM_PREINDEXED;
                    list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_FD));
                  end;
              end
            else
              begin
                { restore int registers and return }
                reference_reset(ref,4);
                ref.index:=NR_FRAME_POINTER_REG;
                list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_EA));
              end;
          end
        else if (current_settings.cputype<cpu_armv5) then
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14))
        else
          list.concat(taicpu.op_reg(A_BX,NR_R14))
      end;


    procedure tcgarm.a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);
      var
        b : byte;
        tmpref : treference;
        instr : taicpu;
      begin
        if ref.addressmode<>AM_OFFSET then
          internalerror(200309071);
        tmpref:=ref;
        { Be sure to have a base register }
        if (tmpref.base=NR_NO) then
          begin
            if tmpref.shiftmode<>SM_None then
              internalerror(200308294);
            if tmpref.signindex<0 then
              internalerror(200312023);
            tmpref.base:=tmpref.index;
            tmpref.index:=NR_NO;
          end;

        if assigned(tmpref.symbol) or
           not((is_shifter_const(tmpref.offset,b)) or
               (is_shifter_const(-tmpref.offset,b))
              ) then
          fixref(list,tmpref);

        { expect a base here if there is an index }
        if (tmpref.base=NR_NO) and (tmpref.index<>NR_NO) then
          internalerror(200312022);

        if tmpref.index<>NR_NO then
          begin
            if tmpref.shiftmode<>SM_None then
              internalerror(200312021);
            if tmpref.signindex<0 then
              a_op_reg_reg_reg(list,OP_SUB,OS_ADDR,tmpref.base,tmpref.index,r)
            else
              a_op_reg_reg_reg(list,OP_ADD,OS_ADDR,tmpref.base,tmpref.index,r);
            if tmpref.offset<>0 then
              a_op_const_reg_reg(list,OP_ADD,OS_ADDR,tmpref.offset,r,r);
          end
        else
          begin
            if tmpref.base=NR_NO then
              a_load_const_reg(list,OS_ADDR,tmpref.offset,r)
            else
              if tmpref.offset<>0 then
                a_op_const_reg_reg(list,OP_ADD,OS_ADDR,tmpref.offset,tmpref.base,r)
              else
                begin
                  instr:=taicpu.op_reg_reg(A_MOV,r,tmpref.base);
                  list.concat(instr);
                  add_move_instruction(instr);
                end;
          end;
      end;


    procedure tcgarm.fixref(list : TAsmList;var ref : treference);
      var
        tmpreg : tregister;
        tmpref : treference;
        l : tasmlabel;
      begin
        { absolute symbols can't be handled directly, we've to store the symbol reference
          in the text segment and access it pc relative

          For now, we assume that references where base or index equals to PC are already
          relative, all other references are assumed to be absolute and thus they need
          to be handled extra.

          A proper solution would be to change refoptions to a set and store the information
          if the symbol is absolute or relative there.
        }
        { create consts entry }
        reference_reset(tmpref,4);
        current_asmdata.getjumplabel(l);
        cg.a_label(current_procinfo.aktlocaldata,l);
        tmpref.symboldata:=current_procinfo.aktlocaldata.last;

        if assigned(ref.symbol) then
          current_procinfo.aktlocaldata.concat(tai_const.create_sym_offset(ref.symbol,ref.offset))
        else
          current_procinfo.aktlocaldata.concat(tai_const.Create_32bit(ref.offset));

        { load consts entry }
        tmpreg:=getintregister(list,OS_INT);
        tmpref.symbol:=l;
        tmpref.base:=NR_PC;
        list.concat(taicpu.op_reg_ref(A_LDR,tmpreg,tmpref));

        if (ref.base<>NR_NO) then
          begin
            if ref.index<>NR_NO then
              begin
                list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,tmpreg));
                ref.base:=tmpreg;
              end
            else
              if ref.base<>NR_PC then
                begin
                  ref.index:=tmpreg;
                  ref.shiftimm:=0;
                  ref.signindex:=1;
                  ref.shiftmode:=SM_None;
                end
                else
                  ref.base:=tmpreg;
          end
        else
          ref.base:=tmpreg;
        ref.offset:=0;
        ref.symbol:=nil;
      end;


    procedure tcgarm.g_concatcopy_move(list : TAsmList;const source,dest : treference;len : tcgint);
      var
        paraloc1,paraloc2,paraloc3 : TCGPara;
      begin
        paraloc1.init;
        paraloc2.init;
        paraloc3.init;
        paramanager.getintparaloc(pocall_default,1,paraloc1);
        paramanager.getintparaloc(pocall_default,2,paraloc2);
        paramanager.getintparaloc(pocall_default,3,paraloc3);
        a_load_const_cgpara(list,OS_INT,len,paraloc3);
        a_loadaddr_ref_cgpara(list,dest,paraloc2);
        a_loadaddr_ref_cgpara(list,source,paraloc1);
        paramanager.freecgpara(list,paraloc3);
        paramanager.freecgpara(list,paraloc2);
        paramanager.freecgpara(list,paraloc1);
        alloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        alloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
        a_call_name(list,'FPC_MOVE',false);
        dealloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
        dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        paraloc3.done;
        paraloc2.done;
        paraloc1.done;
      end;


    procedure tcgarm.g_concatcopy_internal(list : TAsmList;const source,dest : treference;len : tcgint;aligned : boolean);
      const
        maxtmpreg=10;{roozbeh: can be reduced to 8 or lower if might conflick with reserved ones,also +2 is used becouse of regs required for referencing}

      var
        srcref,dstref,usedtmpref,usedtmpref2:treference;
        srcreg,destreg,countreg,r,tmpreg:tregister;
        helpsize:aint;
        copysize:byte;
        cgsize:Tcgsize;
        tmpregisters:array[1..maxtmpreg] of tregister;
        tmpregi,tmpregi2:byte;

      { will never be called with count<=4 }
      procedure genloop(count : aword;size : byte);
        const
          size2opsize : array[1..4] of tcgsize = (OS_8,OS_16,OS_NO,OS_32);
        var
          l : tasmlabel;
        begin
          current_asmdata.getjumplabel(l);
          if count<size then size:=1;
          a_load_const_reg(list,OS_INT,count div size,countreg);
          cg.a_label(list,l);
          srcref.addressmode:=AM_POSTINDEXED;
          dstref.addressmode:=AM_POSTINDEXED;
          srcref.offset:=size;
          dstref.offset:=size;
          r:=getintregister(list,size2opsize[size]);
          a_load_ref_reg(list,size2opsize[size],size2opsize[size],srcref,r);
          list.concat(setoppostfix(taicpu.op_reg_reg_const(A_SUB,countreg,countreg,1),PF_S));
          a_load_reg_ref(list,size2opsize[size],size2opsize[size],r,dstref);
          a_jmp_flags(list,F_NE,l);
          srcref.offset:=1;
          dstref.offset:=1;
          case count mod size of
            1:
              begin
                a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                a_load_reg_ref(list,OS_8,OS_8,r,dstref);
              end;
            2:
              if aligned then
                begin
                  a_load_ref_reg(list,OS_16,OS_16,srcref,r);
                  a_load_reg_ref(list,OS_16,OS_16,r,dstref);
                end
              else
                begin
                  a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                  a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                end;
            3:
              if aligned then
                begin
                  srcref.offset:=2;
                  dstref.offset:=2;
                  a_load_ref_reg(list,OS_16,OS_16,srcref,r);
                  a_load_reg_ref(list,OS_16,OS_16,r,dstref);
                  a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                end
              else
                begin
                  a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                  a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                  a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                end;
          end;
          { keep the registers alive }
          list.concat(taicpu.op_reg_reg(A_MOV,countreg,countreg));
          list.concat(taicpu.op_reg_reg(A_MOV,srcreg,srcreg));
          list.concat(taicpu.op_reg_reg(A_MOV,destreg,destreg));
        end;

      begin
        if len=0 then
          exit;
        helpsize:=12+maxtmpreg*4;//52 with maxtmpreg=10
        dstref:=dest;
        srcref:=source;
        if cs_opt_size in current_settings.optimizerswitches then
          helpsize:=8;
        if aligned and (len=4) then
          begin
            tmpreg:=getintregister(list,OS_32);
            a_load_ref_reg(list,OS_32,OS_32,source,tmpreg);
            a_load_reg_ref(list,OS_32,OS_32,tmpreg,dest);
          end
        else if (len<=helpsize) and aligned then
          begin
            tmpregi:=0;

            srcreg:=getintregister(list,OS_ADDR);

            { explicit pc relative addressing, could be
              e.g. a floating point constant }
            if source.base=NR_PC then
              begin
                { ... then we don't need a loadaddr }
                srcref:=source;
              end
            else
              begin
                a_loadaddr_ref_reg(list,source,srcreg);
                reference_reset_base(srcref,srcreg,0,source.alignment);
              end;

            while (len div 4 <> 0) and (tmpregi<maxtmpreg) do
              begin
                inc(tmpregi);
                tmpregisters[tmpregi]:=getintregister(list,OS_32);
                a_load_ref_reg(list,OS_32,OS_32,srcref,tmpregisters[tmpregi]);
                inc(srcref.offset,4);
                dec(len,4);
              end;

            destreg:=getintregister(list,OS_ADDR);
            a_loadaddr_ref_reg(list,dest,destreg);
            reference_reset_base(dstref,destreg,0,dest.alignment);
            tmpregi2:=1;
            while (tmpregi2<=tmpregi) do
              begin
                a_load_reg_ref(list,OS_32,OS_32,tmpregisters[tmpregi2],dstref);
                inc(dstref.offset,4);
                inc(tmpregi2);
              end;

            copysize:=4;
            cgsize:=OS_32;
            while len<>0 do
              begin
                if len<2 then
                  begin
                    copysize:=1;
                    cgsize:=OS_8;
                  end
                else if len<4 then
                  begin
                    copysize:=2;
                    cgsize:=OS_16;
                  end;
                dec(len,copysize);
                r:=getintregister(list,cgsize);
                a_load_ref_reg(list,cgsize,cgsize,srcref,r);
                a_load_reg_ref(list,cgsize,cgsize,r,dstref);
                inc(srcref.offset,copysize);
                inc(dstref.offset,copysize);
              end;{end of while}
          end
        else
          begin
            cgsize:=OS_32;
            if (len<=4) then{len<=4 and not aligned}
              begin
                r:=getintregister(list,cgsize);
                usedtmpref:=a_internal_load_ref_reg(list,OS_8,OS_8,srcref,r);
                if Len=1 then
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref)
                else
                  begin
                    tmpreg:=getintregister(list,cgsize);
                    usedtmpref2:=a_internal_load_reg_ref(list,OS_8,OS_8,r,dstref);
                    inc(usedtmpref.offset,1);
                    a_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                    inc(usedtmpref2.offset,1);
                    a_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref2);
                    if len>2 then
                      begin
                        inc(usedtmpref.offset,1);
                        a_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                        inc(usedtmpref2.offset,1);
                        a_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref2);
                        if len>3 then
                          begin
                            inc(usedtmpref.offset,1);
                            a_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                            inc(usedtmpref2.offset,1);
                            a_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref2);
                          end;
                        end;
                      end;
            end{end of if len<=4}
            else
              begin{unaligned & 4<len<helpsize **or** aligned/unaligned & len>helpsize}
                destreg:=getintregister(list,OS_ADDR);
                a_loadaddr_ref_reg(list,dest,destreg);
                reference_reset_base(dstref,destreg,0,dest.alignment);

                srcreg:=getintregister(list,OS_ADDR);
                a_loadaddr_ref_reg(list,source,srcreg);
                reference_reset_base(srcref,srcreg,0,source.alignment);

                countreg:=getintregister(list,OS_32);

//            if cs_opt_size in current_settings.optimizerswitches  then
                { roozbeh : it seems loading 1 byte is faster becouse of caching/fetching(?) }
                {if aligned then
                genloop(len,4)
                else}
                genloop(len,1);
            end;
          end;
    end;

    procedure tcgarm.g_concatcopy_unaligned(list : TAsmList;const source,dest : treference;len : tcgint);
      begin
        g_concatcopy_internal(list,source,dest,len,false);
      end;


    procedure tcgarm.g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);
      begin
        if (source.alignment in [1..3]) or
          (dest.alignment in [1..3]) then
          g_concatcopy_internal(list,source,dest,len,false)
        else
          g_concatcopy_internal(list,source,dest,len,true);
      end;


    procedure tcgarm.g_overflowCheck(list : TAsmList;const l : tlocation;def : tdef);
      var
        ovloc : tlocation;
      begin
        ovloc.loc:=LOC_VOID;
        g_overflowCheck_loc(list,l,def,ovloc);
      end;


    procedure tcgarm.g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;ovloc : tlocation);
      var
        hl : tasmlabel;
        ai:TAiCpu;
        hflags : tresflags;
      begin
        if not(cs_check_overflow in current_settings.localswitches) then
          exit;
        current_asmdata.getjumplabel(hl);
        case ovloc.loc of
          LOC_VOID:
            begin
              ai:=taicpu.op_sym(A_B,hl);
              ai.is_jmp:=true;

              if not((def.typ=pointerdef) or
                    ((def.typ=orddef) and
                     (torddef(def).ordtype in [u64bit,u16bit,u32bit,u8bit,uchar,
                                               pasbool8,pasbool16,pasbool32,pasbool64]))) then
                 ai.SetCondition(C_VC)
              else
                if TAiCpu(List.Last).opcode in [A_RSB,A_RSC,A_SBC,A_SUB] then
                  ai.SetCondition(C_CS)
                else
                  ai.SetCondition(C_CC);

              list.concat(ai);
            end;
          LOC_FLAGS:
            begin
              hflags:=ovloc.resflags;
              inverse_flags(hflags);
              cg.a_jmp_flags(list,hflags,hl);
            end;
          else
            internalerror(200409281);
        end;

        a_call_name(list,'FPC_OVERFLOW',false);
        a_label(list,hl);
      end;


    procedure tcgarm.g_save_registers(list : TAsmList);
      begin
        { this work is done in g_proc_entry }
      end;


    procedure tcgarm.g_restore_registers(list : TAsmList);
      begin
        { this work is done in g_proc_exit }
      end;


    procedure tcgarm.a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);
      var
        ai : taicpu;
      begin
        ai:=Taicpu.Op_sym(A_B,l);
        ai.SetCondition(OpCmp2AsmCond[cond]);
        ai.is_jmp:=true;
        list.concat(ai);
      end;


    procedure tcgarm.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);
      var
        hsym : tsym;
        href : treference;
        paraloc : Pcgparalocation;
        shift : byte;
      begin
        { calculate the parameter info for the procdef }
        procdef.init_paraloc_info(callerside);
        hsym:=tsym(procdef.parast.Find('self'));
        if not(assigned(hsym) and
          (hsym.typ=paravarsym)) then
          internalerror(200305251);
        paraloc:=tparavarsym(hsym).paraloc[callerside].location;
        while paraloc<>nil do
          with paraloc^ do
            begin
              case loc of
                LOC_REGISTER:
                  begin
                    if is_shifter_const(ioffset,shift) then
                      a_op_const_reg(list,OP_SUB,size,ioffset,register)
                    else
                      begin
                        a_load_const_reg(list,OS_ADDR,ioffset,NR_R12);
                        a_op_reg_reg(list,OP_SUB,size,NR_R12,register);
                      end;
                  end;
                LOC_REFERENCE:
                  begin
                    { offset in the wrapper needs to be adjusted for the stored
                      return address }
                    reference_reset_base(href,reference.index,reference.offset+sizeof(aint),sizeof(pint));
                    if is_shifter_const(ioffset,shift) then
                      a_op_const_ref(list,OP_SUB,size,ioffset,href)
                    else
                      begin
                        a_load_const_reg(list,OS_ADDR,ioffset,NR_R12);
                        a_op_reg_ref(list,OP_SUB,size,NR_R12,href);
                      end;
                  end
                else
                  internalerror(200309189);
              end;
              paraloc:=next;
            end;
      end;

    procedure tcgarm.g_stackpointer_alloc(list: TAsmList; size: longint);
      begin
        internalerror(200807237);
      end;


    function get_scalar_mm_op(fromsize,tosize : tcgsize) : tasmop;
      const
        convertop : array[OS_F32..OS_F128,OS_F32..OS_F128] of tasmop = (
          (A_FCPYS,A_FCVTSD,A_NONE,A_NONE,A_NONE),
          (A_FCVTDS,A_FCPYD,A_NONE,A_NONE,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_NONE,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_NONE,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_NONE,A_NONE));
      begin
        result:=convertop[fromsize,tosize];
        if result=A_NONE then
          internalerror(200312205);
      end;


    procedure tcgarm.a_loadmm_reg_reg(list: tasmlist; fromsize,tosize: tcgsize; reg1,reg2: tregister; shuffle: pmmshuffle);
      var
        instr: taicpu;
      begin
        if shuffle=nil then
          begin
            if fromsize=tosize then
              { needs correct size in case of spilling }
              case fromsize of
                OS_F32:
                  instr:=taicpu.op_reg_reg(A_FCPYS,reg2,reg1);
                OS_F64:
                  instr:=taicpu.op_reg_reg(A_FCPYD,reg2,reg1);
                else
                  internalerror(2009112405);
              end
            else
              internalerror(2009112406);
          end
        else if shufflescalar(shuffle) then
          instr:=taicpu.op_reg_reg(get_scalar_mm_op(tosize,fromsize),reg2,reg1)
        else
          internalerror(2009112407);
        list.concat(instr);
        case instr.opcode of
          A_FCPYS,
          A_FCPYD:
            add_move_instruction(instr);
        end;
      end;


    procedure tcgarm.a_loadmm_ref_reg(list: tasmlist; fromsize,tosize: tcgsize; const ref: treference; reg: tregister; shuffle: pmmshuffle);
      var
        intreg,
        tmpmmreg : tregister;
        reg64    : tregister64;
        op       : tasmop;
      begin
        if assigned(shuffle) and
           not(shufflescalar(shuffle)) then
          internalerror(2009112413);

        case fromsize of
          OS_32,OS_S32:
            begin
              fromsize:=OS_F32;
              { since we are loading an integer, no conversion may be required }
              if (fromsize<>tosize) then
                internalerror(2009112801);
            end;
          OS_64,OS_S64:
            begin
              fromsize:=OS_F64;
              { since we are loading an integer, no conversion may be required }
              if (fromsize<>tosize) then
                internalerror(2009112901);
            end;
        end;

        if (fromsize<>tosize) then
          tmpmmreg:=getmmregister(list,fromsize)
        else
          tmpmmreg:=reg;
        if (ref.alignment in [1,2]) then
          begin
            case fromsize of
              OS_F32:
                begin
                  intreg:=getintregister(list,OS_32);
                  a_load_ref_reg(list,OS_32,OS_32,ref,intreg);
                  a_loadmm_intreg_reg(list,OS_32,OS_F32,intreg,tmpmmreg,mms_movescalar);
                end;
              OS_F64:
                begin
                  reg64.reglo:=getintregister(list,OS_32);
                  reg64.reghi:=getintregister(list,OS_32);
                  cg64.a_load64_ref_reg(list,ref,reg64);
                  cg64.a_loadmm_intreg64_reg(list,OS_F64,reg64,tmpmmreg);
                end;
              else
                internalerror(2009112412);
            end;
          end
        else
          begin
             case fromsize of
               OS_F32:
                 op:=A_FLDS;
               OS_F64:
                 op:=A_FLDD;
               else
                 internalerror(2009112415);
             end;
             handle_load_store(list,op,PF_None,tmpmmreg,ref);
          end;

        if (tmpmmreg<>reg) then
          a_loadmm_reg_reg(list,fromsize,tosize,tmpmmreg,reg,shuffle);
      end;


    procedure tcgarm.a_loadmm_reg_ref(list: tasmlist; fromsize,tosize: tcgsize; reg: tregister; const ref: treference; shuffle: pmmshuffle);
      var
        intreg,
        tmpmmreg : tregister;
        reg64    : tregister64;
        op       : tasmop;
      begin
        if assigned(shuffle) and
           not(shufflescalar(shuffle)) then
          internalerror(2009112416);

        case tosize of
          OS_32,OS_S32:
            begin
              tosize:=OS_F32;
              { since we are loading an integer, no conversion may be required }
              if (fromsize<>tosize) then
                internalerror(2009112801);
            end;
          OS_64,OS_S64:
            begin
              tosize:=OS_F64;
              { since we are loading an integer, no conversion may be required }
              if (fromsize<>tosize) then
                internalerror(2009112901);
            end;
        end;

        if (fromsize<>tosize) then
          begin
            tmpmmreg:=getmmregister(list,tosize);
            a_loadmm_reg_reg(list,fromsize,tosize,reg,tmpmmreg,shuffle);
          end
        else
          tmpmmreg:=reg;
        if (ref.alignment in [1,2]) then
          begin
            case tosize of
              OS_F32:
                begin
                  intreg:=getintregister(list,OS_32);
                  a_loadmm_reg_intreg(list,OS_F32,OS_32,tmpmmreg,intreg,shuffle);
                  a_load_reg_ref(list,OS_32,OS_32,intreg,ref);
                end;
              OS_F64:
                begin
                  reg64.reglo:=getintregister(list,OS_32);
                  reg64.reghi:=getintregister(list,OS_32);
                  cg64.a_loadmm_reg_intreg64(list,OS_F64,tmpmmreg,reg64);
                  cg64.a_load64_reg_ref(list,reg64,ref);
                end;
              else
                internalerror(2009112417);
            end;
          end
        else
          begin
             case fromsize of
               OS_F32:
                 op:=A_FSTS;
               OS_F64:
                 op:=A_FSTD;
               else
                 internalerror(2009112418);
             end;
             handle_load_store(list,op,PF_None,tmpmmreg,ref);
          end;
      end;


    procedure tcgarm.a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize : tcgsize; intreg, mmreg: tregister; shuffle: pmmshuffle);
      begin
        { this code can only be used to transfer raw data, not to perform
          conversions }
        if (tosize<>OS_F32) then
          internalerror(2009112419);
        if not(fromsize in [OS_32,OS_S32]) then
          internalerror(2009112420);
        if assigned(shuffle) and
           not shufflescalar(shuffle) then
          internalerror(2009112516);
        list.concat(taicpu.op_reg_reg(A_FMSR,mmreg,intreg));
      end;


    procedure tcgarm.a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize : tcgsize; mmreg, intreg: tregister;shuffle : pmmshuffle);
      begin
        { this code can only be used to transfer raw data, not to perform
          conversions }
        if (fromsize<>OS_F32) then
          internalerror(2009112430);
        if not(tosize in [OS_32,OS_S32]) then
          internalerror(2009112420);
        if assigned(shuffle) and
           not shufflescalar(shuffle) then
          internalerror(2009112514);
        list.concat(taicpu.op_reg_reg(A_FMRS,intreg,mmreg));
      end;


      procedure tcgarm.a_opmm_reg_reg(list: tasmlist; op: topcg; size: tcgsize; src, dst: tregister; shuffle: pmmshuffle);
        var
          tmpreg: tregister;
        begin
          { the vfp doesn't support xor nor any other logical operation, but
            this routine is used to initialise global mm regvars. We can
            easily initialise an mm reg with 0 though. }
          case op of
            OP_XOR:
              begin
                if (src<>dst) or
                   (reg_cgsize(src)<>size) or
                   assigned(shuffle) then
                  internalerror(2009112907);
                tmpreg:=getintregister(list,OS_32);
                a_load_const_reg(list,OS_32,0,tmpreg);
                case size of
                  OS_F32:
                    list.concat(taicpu.op_reg_reg(A_FMSR,dst,tmpreg));
                  OS_F64:
                    list.concat(taicpu.op_reg_reg_reg(A_FMDRR,dst,tmpreg,tmpreg));
                  else
                    internalerror(2009112908);
                end;
              end
            else
              internalerror(2009112906);
          end;
        end;


    procedure tcgarm.g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);

      procedure loadvmttor12;
        var
          href : treference;
        begin
          reference_reset_base(href,NR_R0,0,sizeof(pint));
          cg.a_load_ref_reg(list,OS_ADDR,OS_ADDR,href,NR_R12);
        end;


      procedure op_onr12methodaddr;
        var
          href : treference;
        begin
          if (procdef.extnumber=$ffff) then
            Internalerror(200006139);
          { call/jmp  vmtoffs(%eax) ; method offs }
          reference_reset_base(href,NR_R12,tobjectdef(procdef.struct).vmtmethodoffset(procdef.extnumber),sizeof(pint));
          cg.a_load_ref_reg(list,OS_ADDR,OS_ADDR,href,NR_R12);
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R12));
        end;

      var
        make_global : boolean;
      begin
        if not(procdef.proctypeoption in [potype_function,potype_procedure]) then
          Internalerror(200006137);
        if not assigned(procdef.struct) or
           (procdef.procoptions*[po_classmethod, po_staticmethod,
             po_methodpointer, po_interrupt, po_iocheck]<>[]) then
          Internalerror(200006138);
        if procdef.owner.symtabletype<>ObjectSymtable then
          Internalerror(200109191);

        make_global:=false;
        if (not current_module.is_unit) or
           create_smartlink or
           (procdef.owner.defowner.owner.symtabletype=globalsymtable) then
          make_global:=true;

        if make_global then
          list.concat(Tai_symbol.Createname_global(labelname,AT_FUNCTION,0))
        else
          list.concat(Tai_symbol.Createname(labelname,AT_FUNCTION,0));

        { the wrapper might need aktlocaldata for the additional data to
          load the constant }
        current_procinfo:=cprocinfo.create(nil);

        { set param1 interface to self  }
        g_adjust_self_value(list,procdef,ioffset);

        { case 4 }
        if (po_virtualmethod in procdef.procoptions) and
            not is_objectpascal_helper(procdef.struct) then
          begin
            loadvmttor12;
            op_onr12methodaddr;
          end
        { case 0 }
        else
          list.concat(taicpu.op_sym(A_B,current_asmdata.RefAsmSymbol(procdef.mangledname)));
        list.concatlist(current_procinfo.aktlocaldata);

        current_procinfo.Free;
        current_procinfo:=nil;

        list.concat(Tai_symbol_end.Createname(labelname));
      end;


    procedure tcgarm.maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);
      const
        overflowops = [OP_MUL,OP_SHL,OP_ADD,OP_SUB,OP_NOT,OP_NEG];
      begin
        if (op in overflowops) and
           (size in [OS_8,OS_S8,OS_16,OS_S16]) then
          a_load_reg_reg(list,OS_32,size,dst,dst);
      end;


    function tcgarm.get_darwin_call_stub(const s: string; weak: boolean): tasmsymbol;
      var
        stubname: string;
        l1: tasmsymbol;
        href: treference;
      begin
        stubname := 'L'+s+'$stub';
        result := current_asmdata.getasmsymbol(stubname);
        if assigned(result) then
          exit;

        if current_asmdata.asmlists[al_imports]=nil then
          current_asmdata.asmlists[al_imports]:=TAsmList.create;

        new_section(current_asmdata.asmlists[al_imports],sec_stub,'',4);
        result := current_asmdata.RefAsmSymbol(stubname);
        current_asmdata.asmlists[al_imports].concat(Tai_symbol.Create(result,0));
        { register as a weak symbol if necessary }
        if weak then
          current_asmdata.weakrefasmsymbol(s);
        current_asmdata.asmlists[al_imports].concat(tai_directive.create(asd_indirect_symbol,s));

        if not(cs_create_pic in current_settings.moduleswitches) then
          begin
            l1 := current_asmdata.RefAsmSymbol('L'+s+'$slp');
            reference_reset_symbol(href,l1,0,sizeof(pint));
            href.refaddr:=addr_full;
            current_asmdata.asmlists[al_imports].concat(taicpu.op_reg_ref(A_LDR,NR_R12,href));
            reference_reset_base(href,NR_R12,0,sizeof(pint));
            current_asmdata.asmlists[al_imports].concat(taicpu.op_reg_ref(A_LDR,NR_R15,href));
            current_asmdata.asmlists[al_imports].concat(Tai_symbol.Create(l1,0));
            l1 := current_asmdata.RefAsmSymbol('L'+s+'$lazy_ptr');
            current_asmdata.asmlists[al_imports].concat(tai_const.create_sym(l1));
          end
        else
          internalerror(2008100401);

        new_section(current_asmdata.asmlists[al_imports],sec_data_lazy,'',sizeof(pint));
        current_asmdata.asmlists[al_imports].concat(Tai_symbol.Create(l1,0));
        current_asmdata.asmlists[al_imports].concat(tai_directive.create(asd_indirect_symbol,s));
        current_asmdata.asmlists[al_imports].concat(tai_const.createname('dyld_stub_binding_helper',0));
      end;


    procedure tcg64farm.a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);
      begin
        case op of
          OP_NEG:
            begin
              list.concat(setoppostfix(taicpu.op_reg_reg_const(A_RSB,regdst.reglo,regsrc.reglo,0),PF_S));
              list.concat(taicpu.op_reg_reg_const(A_RSC,regdst.reghi,regsrc.reghi,0));
            end;
          OP_NOT:
            begin
              cg.a_op_reg_reg(list,OP_NOT,OS_INT,regsrc.reglo,regdst.reglo);
              cg.a_op_reg_reg(list,OP_NOT,OS_INT,regsrc.reghi,regdst.reghi);
            end;
          else
            a_op64_reg_reg_reg(list,op,size,regsrc,regdst,regdst);
        end;
      end;


    procedure tcg64farm.a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);
      begin
        a_op64_const_reg_reg(list,op,size,value,reg,reg);
      end;


    procedure tcg64farm.a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64);
      var
        ovloc : tlocation;
      begin
        a_op64_const_reg_reg_checkoverflow(list,op,size,value,regsrc,regdst,false,ovloc);
      end;


    procedure tcg64farm.a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);
      var
        ovloc : tlocation;
      begin
        a_op64_reg_reg_reg_checkoverflow(list,op,size,regsrc1,regsrc2,regdst,false,ovloc);
      end;


    procedure tcg64farm.a_loadmm_intreg64_reg(list: TAsmList; mmsize: tcgsize; intreg: tregister64; mmreg: tregister);
      begin
        { this code can only be used to transfer raw data, not to perform
          conversions }
        if (mmsize<>OS_F64) then
          internalerror(2009112405);
        list.concat(taicpu.op_reg_reg_reg(A_FMDRR,mmreg,intreg.reglo,intreg.reghi));
      end;


    procedure tcg64farm.a_loadmm_reg_intreg64(list: TAsmList; mmsize: tcgsize; mmreg: tregister; intreg: tregister64);
      begin
        { this code can only be used to transfer raw data, not to perform
          conversions }
        if (mmsize<>OS_F64) then
          internalerror(2009112406);
        list.concat(taicpu.op_reg_reg_reg(A_FMRRD,intreg.reglo,intreg.reghi,mmreg));
      end;


    procedure tcg64farm.a_op64_const_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64;setflags : boolean;var ovloc : tlocation);
      var
        tmpreg : tregister;
        b : byte;
      begin
        ovloc.loc:=LOC_VOID;
        case op of
          OP_NEG,
          OP_NOT :
            internalerror(200306017);
        end;
        if (setflags or tcgarm(cg).cgsetflags) and (op in [OP_ADD,OP_SUB]) then
          begin
            case op of
              OP_ADD:
                begin
                  if is_shifter_const(lo(value),b) then
                    list.concat(setoppostfix(taicpu.op_reg_reg_const(A_ADD,regdst.reglo,regsrc.reglo,lo(value)),PF_S))
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,lo(value),tmpreg);
                      list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ADD,regdst.reglo,regsrc.reglo,tmpreg),PF_S));
                    end;

                  if is_shifter_const(hi(value),b) then
                    list.concat(setoppostfix(taicpu.op_reg_reg_const(A_ADC,regdst.reghi,regsrc.reghi,hi(value)),PF_S))
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,hi(value),tmpreg);
                      list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ADC,regdst.reghi,regsrc.reghi,tmpreg),PF_S));
                    end;
                end;
              OP_SUB:
                begin
                  if is_shifter_const(lo(value),b) then
                    list.concat(setoppostfix(taicpu.op_reg_reg_const(A_SUB,regdst.reglo,regsrc.reglo,lo(value)),PF_S))
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,lo(value),tmpreg);
                      list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_SUB,regdst.reglo,regsrc.reglo,tmpreg),PF_S));
                    end;

                  if is_shifter_const(hi(value),b) then
                    list.concat(setoppostfix(taicpu.op_reg_reg_const(A_SBC,regdst.reghi,regsrc.reghi,aint(hi(value))),PF_S))
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,hi(value),tmpreg);
                      list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_SBC,regdst.reghi,regsrc.reghi,tmpreg),PF_S));
                    end;
                end;
              else
                internalerror(200502131);
            end;
            if size=OS_64 then
              begin
                { the arm has an weired opinion how flags for SUB/ADD are handled }
                ovloc.loc:=LOC_FLAGS;
                case op of
                  OP_ADD:
                    ovloc.resflags:=F_CS;
                  OP_SUB:
                    ovloc.resflags:=F_CC;
                end;
              end;
          end
        else
          begin
            case op of
              OP_AND,OP_OR,OP_XOR:
                begin
                  cg.a_op_const_reg_reg(list,op,OS_32,aint(lo(value)),regsrc.reglo,regdst.reglo);
                  cg.a_op_const_reg_reg(list,op,OS_32,aint(hi(value)),regsrc.reghi,regdst.reghi);
                end;
              OP_ADD:
                begin
                  if is_shifter_const(aint(lo(value)),b) then
                    list.concat(setoppostfix(taicpu.op_reg_reg_const(A_ADD,regdst.reglo,regsrc.reglo,aint(lo(value))),PF_S))
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,aint(lo(value)),tmpreg);
                      list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ADD,regdst.reglo,regsrc.reglo,tmpreg),PF_S));
                    end;

                  if is_shifter_const(aint(hi(value)),b) then
                    list.concat(taicpu.op_reg_reg_const(A_ADC,regdst.reghi,regsrc.reghi,aint(hi(value))))
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,aint(hi(value)),tmpreg);
                      list.concat(taicpu.op_reg_reg_reg(A_ADC,regdst.reghi,regsrc.reghi,tmpreg));
                    end;
                end;
              OP_SUB:
                begin
                  if is_shifter_const(aint(lo(value)),b) then
                    list.concat(setoppostfix(taicpu.op_reg_reg_const(A_SUB,regdst.reglo,regsrc.reglo,aint(lo(value))),PF_S))
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,aint(lo(value)),tmpreg);
                      list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_SUB,regdst.reglo,regsrc.reglo,tmpreg),PF_S));
                    end;

                  if is_shifter_const(aint(hi(value)),b) then
                    list.concat(taicpu.op_reg_reg_const(A_SBC,regdst.reghi,regsrc.reghi,aint(hi(value))))
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,hi(value),tmpreg);
                      list.concat(taicpu.op_reg_reg_reg(A_SBC,regdst.reghi,regsrc.reghi,tmpreg));
                    end;
                end;
            else
              internalerror(2003083101);
          end;
        end;
      end;


    procedure tcg64farm.a_op64_reg_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64;setflags : boolean;var ovloc : tlocation);
      begin
        ovloc.loc:=LOC_VOID;
        case op of
          OP_NEG,
          OP_NOT :
            internalerror(200306017);
        end;
        if (setflags or tcgarm(cg).cgsetflags) and (op in [OP_ADD,OP_SUB]) then
          begin
            case op of
              OP_ADD:
                begin
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ADD,regdst.reglo,regsrc1.reglo,regsrc2.reglo),PF_S));
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ADC,regdst.reghi,regsrc1.reghi,regsrc2.reghi),PF_S));
                end;
              OP_SUB:
                begin
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_SUB,regdst.reglo,regsrc2.reglo,regsrc1.reglo),PF_S));
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_SBC,regdst.reghi,regsrc2.reghi,regsrc1.reghi),PF_S));
                end;
              else
                internalerror(2003083101);
            end;
            if size=OS_64 then
              begin
                { the arm has an weired opinion how flags for SUB/ADD are handled }
                ovloc.loc:=LOC_FLAGS;
                case op of
                  OP_ADD:
                    ovloc.resflags:=F_CS;
                  OP_SUB:
                    ovloc.resflags:=F_CC;
                end;
              end;
          end
        else
          begin
            case op of
              OP_AND,OP_OR,OP_XOR:
                begin
                  cg.a_op_reg_reg_reg(list,op,OS_32,regsrc1.reglo,regsrc2.reglo,regdst.reglo);
                  cg.a_op_reg_reg_reg(list,op,OS_32,regsrc1.reghi,regsrc2.reghi,regdst.reghi);
                end;
              OP_ADD:
                begin
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ADD,regdst.reglo,regsrc1.reglo,regsrc2.reglo),PF_S));
                  list.concat(taicpu.op_reg_reg_reg(A_ADC,regdst.reghi,regsrc1.reghi,regsrc2.reghi));
                end;
              OP_SUB:
                begin
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_SUB,regdst.reglo,regsrc2.reglo,regsrc1.reglo),PF_S));
                  list.concat(taicpu.op_reg_reg_reg(A_SBC,regdst.reghi,regsrc2.reghi,regsrc1.reghi));
                end;
              else
                internalerror(2003083101);
            end;
          end;
      end;


    procedure Tthumb2cgarm.init_register_allocators;
      begin
        inherited init_register_allocators;
        { currently, we save R14 always, so we can use it }
        if (target_info.system<>system_arm_darwin) then
          rg[R_INTREGISTER]:=trgintcputhumb2.create(R_INTREGISTER,R_SUBWHOLE,
              [RS_R0,RS_R1,RS_R2,RS_R3,RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,
               RS_R9,RS_R10,RS_R12,RS_R14],first_int_imreg,[])
        else
          { r9 is not available on Darwin according to the llvm code generator }
          rg[R_INTREGISTER]:=trgintcputhumb2.create(R_INTREGISTER,R_SUBWHOLE,
              [RS_R0,RS_R1,RS_R2,RS_R3,RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,
               RS_R10,RS_R12,RS_R14],first_int_imreg,[]);
        rg[R_FPUREGISTER]:=trgcputhumb2.create(R_FPUREGISTER,R_SUBNONE,
            [RS_F0,RS_F1,RS_F2,RS_F3,RS_F4,RS_F5,RS_F6,RS_F7],first_fpu_imreg,[]);
        rg[R_MMREGISTER]:=trgcputhumb2.create(R_MMREGISTER,R_SUBNONE,
            [RS_S0,RS_S1,RS_R2,RS_R3,RS_R4,RS_S31],first_mm_imreg,[]);
      end;


    procedure Tthumb2cgarm.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_MMREGISTER].free;
        inherited done_register_allocators;
      end;


    procedure Tthumb2cgarm.a_call_reg(list : TAsmList;reg: tregister);
      begin
        list.concat(taicpu.op_reg(A_BLX, reg));
{
        the compiler does not properly set this flag anymore in pass 1, and
        for now we only need it after pass 2 (I hope) (JM)
          if not(pi_do_call in current_procinfo.flags) then
            internalerror(2003060703);
}
        include(current_procinfo.flags,pi_do_call);
      end;


     procedure Tthumb2cgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);
       var
          imm_shift : byte;
          l : tasmlabel;
          hr : treference;
       begin
          if not(size in [OS_8,OS_S8,OS_16,OS_S16,OS_32,OS_S32]) then
            internalerror(2002090902);
          if is_shifter_const(a,imm_shift) then
            list.concat(taicpu.op_reg_const(A_MOV,reg,a))
          { loading of constants with mov and orr }
          else if (is_shifter_const(a-byte(a),imm_shift)) then
            begin
              list.concat(taicpu.op_reg_const(A_MOV,reg,a-byte(a)));
              list.concat(taicpu.op_reg_reg_const(A_ORR,reg,reg,byte(a)));
            end
          else if (is_shifter_const(a-word(a),imm_shift)) and (is_shifter_const(word(a),imm_shift)) then
            begin
              list.concat(taicpu.op_reg_const(A_MOV,reg,a-word(a)));
              list.concat(taicpu.op_reg_reg_const(A_ORR,reg,reg,word(a)));
            end
          else if (is_shifter_const(a-(dword(a) shl 8) shr 8,imm_shift)) and (is_shifter_const((dword(a) shl 8) shr 8,imm_shift)) then
            begin
              list.concat(taicpu.op_reg_const(A_MOV,reg,a-(dword(a) shl 8) shr 8));
              list.concat(taicpu.op_reg_reg_const(A_ORR,reg,reg,(dword(a) shl 8) shr 8));
            end
          else
            begin
               reference_reset(hr,4);

               current_asmdata.getjumplabel(l);
               cg.a_label(current_procinfo.aktlocaldata,l);
               hr.symboldata:=current_procinfo.aktlocaldata.last;
               current_procinfo.aktlocaldata.concat(tai_const.Create_32bit(longint(a)));

               hr.symbol:=l;
               list.concat(taicpu.op_reg_ref(A_LDR,reg,hr));
            end;
       end;


     procedure Tthumb2cgarm.a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);
       var
         oppostfix:toppostfix;
         usedtmpref: treference;
         tmpreg,tmpreg2 : tregister;
         so : tshifterop;
         dir : integer;
       begin
         if (TCGSize2Size[FromSize] >= TCGSize2Size[ToSize]) then
           FromSize := ToSize;
         case FromSize of
           { signed integer registers }
           OS_8:
             oppostfix:=PF_B;
           OS_S8:
             oppostfix:=PF_SB;
           OS_16:
             oppostfix:=PF_H;
           OS_S16:
             oppostfix:=PF_SH;
           OS_32,
           OS_S32:
             oppostfix:=PF_None;
           else
             InternalError(200308297);
         end;
         if (ref.alignment in [1,2]) and (ref.alignment<tcgsize2size[fromsize]) then
           begin
             if target_info.endian=endian_big then
               dir:=-1
             else
               dir:=1;
             case FromSize of
               OS_16,OS_S16:
                 begin
                   { only complicated references need an extra loadaddr }
                   if assigned(ref.symbol) or
                     (ref.index<>NR_NO) or
                     (ref.offset<-255) or
                     (ref.offset>4094) or
                     { sometimes the compiler reused registers }
                     (reg=ref.index) or
                     (reg=ref.base) then
                     begin
                       tmpreg2:=getintregister(list,OS_INT);
                       a_loadaddr_ref_reg(list,ref,tmpreg2);
                       reference_reset_base(usedtmpref,tmpreg2,0,ref.alignment);
                     end
                   else
                     usedtmpref:=ref;

                   if target_info.endian=endian_big then
                     inc(usedtmpref.offset,1);
                   shifterop_reset(so);so.shiftmode:=SM_LSL;so.shiftimm:=8;
                   tmpreg:=getintregister(list,OS_INT);
                   a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,reg);
                   inc(usedtmpref.offset,dir);
                   if FromSize=OS_16 then
                     a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg)
                   else
                     a_internal_load_ref_reg(list,OS_S8,OS_S8,usedtmpref,tmpreg);
                   list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg,so));
                 end;
               OS_32,OS_S32:
                 begin
                   tmpreg:=getintregister(list,OS_INT);

                   { only complicated references need an extra loadaddr }
                   if assigned(ref.symbol) or
                     (ref.index<>NR_NO) or
                     (ref.offset<-255) or
                     (ref.offset>4092) or
                     { sometimes the compiler reused registers }
                     (reg=ref.index) or
                     (reg=ref.base) then
                     begin
                       tmpreg2:=getintregister(list,OS_INT);
                       a_loadaddr_ref_reg(list,ref,tmpreg2);
                       reference_reset_base(usedtmpref,tmpreg2,0,ref.alignment);
                     end
                   else
                     usedtmpref:=ref;

                   shifterop_reset(so);so.shiftmode:=SM_LSL;
                   if ref.alignment=2 then
                     begin
                       if target_info.endian=endian_big then
                         inc(usedtmpref.offset,2);
                       a_internal_load_ref_reg(list,OS_16,OS_16,usedtmpref,reg);
                       inc(usedtmpref.offset,dir*2);
                       a_internal_load_ref_reg(list,OS_16,OS_16,usedtmpref,tmpreg);
                       so.shiftimm:=16;
                       list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg,so));
                     end
                   else
                     begin
                       if target_info.endian=endian_big then
                         inc(usedtmpref.offset,3);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,reg);
                       inc(usedtmpref.offset,dir);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                       so.shiftimm:=8;
                       list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg,so));
                       inc(usedtmpref.offset,dir);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                       so.shiftimm:=16;
                       list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg,so));
                       inc(usedtmpref.offset,dir);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                       so.shiftimm:=24;
                       list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg,so));
                     end;
                 end
               else
                 handle_load_store(list,A_LDR,oppostfix,reg,ref);
             end;
           end
         else
           handle_load_store(list,A_LDR,oppostfix,reg,ref);

         if (fromsize=OS_S8) and (tosize = OS_16) then
           a_load_reg_reg(list,OS_16,OS_32,reg,reg);
       end;


    procedure Tthumb2cgarm.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
      var
        shift : byte;
        tmpreg : tregister;
        so : tshifterop;
        l1 : longint;
      begin
        ovloc.loc:=LOC_VOID;
        if {$ifopt R+}(a<>-2147483648) and{$endif} is_shifter_const(-a,shift) then
          case op of
            OP_ADD:
              begin
                op:=OP_SUB;
                a:=aint(dword(-a));
              end;
            OP_SUB:
              begin
                op:=OP_ADD;
                a:=aint(dword(-a));
              end
          end;

        if is_shifter_const(a,shift) and not(op in [OP_IMUL,OP_MUL]) then
          case op of
            OP_NEG,OP_NOT,
            OP_DIV,OP_IDIV:
              internalerror(200308281);
            OP_SHL:
              begin
                if a>32 then
                  internalerror(200308294);
                if a<>0 then
                  begin
                    shifterop_reset(so);
                    so.shiftmode:=SM_LSL;
                    so.shiftimm:=a;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                  end
                else
                 list.concat(taicpu.op_reg_reg(A_MOV,dst,src));
              end;
            OP_ROL:
              begin
                if a>32 then
                  internalerror(200308294);
                if a<>0 then
                  begin
                    shifterop_reset(so);
                    so.shiftmode:=SM_ROR;
                    so.shiftimm:=32-a;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                  end
                else
                 list.concat(taicpu.op_reg_reg(A_MOV,dst,src));
              end;
            OP_ROR:
              begin
                if a>32 then
                  internalerror(200308294);
                if a<>0 then
                  begin
                    shifterop_reset(so);
                    so.shiftmode:=SM_ROR;
                    so.shiftimm:=a;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                  end
                else
                 list.concat(taicpu.op_reg_reg(A_MOV,dst,src));
              end;
            OP_SHR:
              begin
                if a>32 then
                  internalerror(200308292);
                shifterop_reset(so);
                if a<>0 then
                  begin
                    so.shiftmode:=SM_LSR;
                    so.shiftimm:=a;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                  end
                else
                 list.concat(taicpu.op_reg_reg(A_MOV,dst,src));
              end;
            OP_SAR:
              begin
                if a>32 then
                  internalerror(200308295);
                if a<>0 then
                  begin
                    shifterop_reset(so);
                    so.shiftmode:=SM_ASR;
                    so.shiftimm:=a;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                  end
                else
                 list.concat(taicpu.op_reg_reg(A_MOV,dst,src));
              end;
            else
              if (op in [OP_SUB, OP_ADD]) and
                 ((a < 0) or
                  (a > 4095)) then
                begin
                  tmpreg:=getintregister(list,size);
                  a_load_const_reg(list, size, a, tmpreg);
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(op_reg_reg_opcg2asmop[op],dst,src,tmpreg),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))
                   ));
                end
              else
              list.concat(setoppostfix(
                  taicpu.op_reg_reg_const(op_reg_reg_opcg2asmop[op],dst,src,a),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))
              ));
              if (cgsetflags or setflags) and (size in [OS_8,OS_16,OS_32]) then
                begin
                  ovloc.loc:=LOC_FLAGS;
                  case op of
                    OP_ADD:
                      ovloc.resflags:=F_CS;
                    OP_SUB:
                      ovloc.resflags:=F_CC;
                  end;
                end;
          end
        else
          begin
            { there could be added some more sophisticated optimizations }
            if (op in [OP_MUL,OP_IMUL]) and (a=1) then
              a_load_reg_reg(list,size,size,src,dst)
            else if (op in [OP_MUL,OP_IMUL]) and (a=0) then
              a_load_const_reg(list,size,0,dst)
            else if (op in [OP_IMUL]) and (a=-1) then
              a_op_reg_reg(list,OP_NEG,size,src,dst)
            { we do this here instead in the peephole optimizer because
              it saves us a register }
            else if (op in [OP_MUL,OP_IMUL]) and ispowerof2(a,l1) and not(cgsetflags or setflags) then
              a_op_const_reg_reg(list,OP_SHL,size,l1,src,dst)
            { for example : b=a*5 -> b=a*4+a with add instruction and shl }
            else if (op in [OP_MUL,OP_IMUL]) and ispowerof2(a-1,l1) and not(cgsetflags or setflags) then
              begin
                if l1>32 then{roozbeh does this ever happen?}
                  internalerror(200308296);
                shifterop_reset(so);
                so.shiftmode:=SM_LSL;
                so.shiftimm:=l1;
                list.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,dst,src,src,so));
              end
            else
              begin
                tmpreg:=getintregister(list,size);
                a_load_const_reg(list,size,a,tmpreg);
                a_op_reg_reg_reg_checkoverflow(list,op,size,tmpreg,src,dst,setflags,ovloc);
              end;
          end;
        maybeadjustresult(list,op,size,dst);
      end;


    const
      op_reg_reg_opcg2asmopThumb2: array[TOpCG] of tasmop =
        (A_NONE,A_MOV,A_ADD,A_AND,A_UDIV,A_SDIV,A_MUL,A_MUL,A_NONE,A_MVN,A_ORR,
         A_ASR,A_LSL,A_LSR,A_SUB,A_EOR,A_NONE,A_ROR);


    procedure Tthumb2cgarm.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);
      var
        so : tshifterop;
        tmpreg,overflowreg : tregister;
        asmop : tasmop;
      begin
        ovloc.loc:=LOC_VOID;
        case op of
           OP_NEG,OP_NOT:
              internalerror(200308281);
           OP_ROL:
              begin
                if not(size in [OS_32,OS_S32]) then
                   internalerror(2008072801);
                { simulate ROL by ror'ing 32-value }
                tmpreg:=getintregister(list,OS_32);
                list.concat(taicpu.op_reg_const(A_MOV,tmpreg,32));
                list.concat(taicpu.op_reg_reg_reg(A_SUB,src1,tmpreg,src1));
                list.concat(taicpu.op_reg_reg_reg(A_ROR, dst, src2, src1));
              end;
           OP_ROR:
              begin
                if not(size in [OS_32,OS_S32]) then
                   internalerror(2008072802);
                list.concat(taicpu.op_reg_reg_reg(A_ROR, dst, src2, src1));
              end;
           OP_IMUL,
           OP_MUL:
              begin
                if cgsetflags or setflags then
                   begin
                      overflowreg:=getintregister(list,size);
                      if op=OP_IMUL then
                        asmop:=A_SMULL
                      else
                        asmop:=A_UMULL;
                      { the arm doesn't allow that rd and rm are the same }
                      if dst=src2 then
                        begin
                           if dst<>src1 then
                              list.concat(taicpu.op_reg_reg_reg_reg(asmop,dst,overflowreg,src1,src2))
                           else
                              begin
                                tmpreg:=getintregister(list,size);
                                a_load_reg_reg(list,size,size,src2,dst);
                                list.concat(taicpu.op_reg_reg_reg_reg(asmop,dst,overflowreg,tmpreg,src1));
                              end;
                        end
                      else
                        list.concat(taicpu.op_reg_reg_reg_reg(asmop,dst,overflowreg,src2,src1));
                      if op=OP_IMUL then
                        begin
                           shifterop_reset(so);
                           so.shiftmode:=SM_ASR;
                           so.shiftimm:=31;
                           list.concat(taicpu.op_reg_reg_shifterop(A_CMP,overflowreg,dst,so));
                        end
                      else
                        list.concat(taicpu.op_reg_const(A_CMP,overflowreg,0));

                       ovloc.loc:=LOC_FLAGS;
                       ovloc.resflags:=F_NE;
                   end
                else
                   begin
                      { the arm doesn't allow that rd and rm are the same }
                      if dst=src2 then
                        begin
                           if dst<>src1 then
                              list.concat(taicpu.op_reg_reg_reg(A_MUL,dst,src1,src2))
                           else
                              begin
                                tmpreg:=getintregister(list,size);
                                a_load_reg_reg(list,size,size,src2,dst);
                                list.concat(taicpu.op_reg_reg_reg(A_MUL,dst,tmpreg,src1));
                              end;
                        end
                      else
                        list.concat(taicpu.op_reg_reg_reg(A_MUL,dst,src2,src1));
                   end;
              end;
           else
              list.concat(setoppostfix(
                   taicpu.op_reg_reg_reg(op_reg_reg_opcg2asmopThumb2[op],dst,src2,src1),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))
                ));
        end;
        maybeadjustresult(list,op,size,dst);
      end;


    procedure Tthumb2cgarm.g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister);
      var item: taicpu;
      begin
        item := setcondition(taicpu.op_reg_const(A_MOV,reg,1),flags_to_cond(f));
        list.concat(item);
        list.insertbefore(taicpu.op_cond(A_IT, flags_to_cond(f)), item);

        item := setcondition(taicpu.op_reg_const(A_MOV,reg,0),inverse_cond(flags_to_cond(f)));
        list.concat(item);
        list.insertbefore(taicpu.op_cond(A_IT, inverse_cond(flags_to_cond(f))), item);
      end;


    procedure Tthumb2cgarm.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
      var
         ref : treference;
         shift : byte;
         firstfloatreg,lastfloatreg,
         r : byte;
         regs : tcpuregisterset;
         stackmisalignment: pint;
      begin
        LocalSize:=align(LocalSize,4);
        { call instruction does not put anything on the stack }
        stackmisalignment:=0;
        if not(nostackframe) then
          begin
            firstfloatreg:=RS_NO;
            { save floating point registers? }
            for r:=RS_F0 to RS_F7 do
              if r in rg[R_FPUREGISTER].used_in_proc-paramanager.get_volatile_registers_fpu(pocall_stdcall) then
                begin
                  if firstfloatreg=RS_NO then
                    firstfloatreg:=r;
                  lastfloatreg:=r;
                  inc(stackmisalignment,12);
                end;

            a_reg_alloc(list,NR_STACK_POINTER_REG);
            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              begin
                a_reg_alloc(list,NR_FRAME_POINTER_REG);
                a_reg_alloc(list,NR_R12);

                list.concat(taicpu.op_reg_reg(A_MOV,NR_R12,NR_STACK_POINTER_REG));
              end;
            { save int registers }
            reference_reset(ref,4);
            ref.index:=NR_STACK_POINTER_REG;
            ref.addressmode:=AM_PREINDEXED;

            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              regs:=regs+[RS_FRAME_POINTER_REG,RS_R14]
            else if (regs<>[]) or (pi_do_call in current_procinfo.flags) then
              include(regs,RS_R14);

            if regs<>[] then
              begin
                for r:=RS_R0 to RS_R15 do
                  if (r in regs) then
                    inc(stackmisalignment,4);
                list.concat(setoppostfix(taicpu.op_ref_regset(A_STM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_FD));
              end;

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              begin
                { the framepointer now points to the saved R15, so the saved
                  framepointer is at R11-12 (for get_caller_frame) }
                list.concat(taicpu.op_reg_reg_const(A_SUB,NR_FRAME_POINTER_REG,NR_R12,4));
                a_reg_dealloc(list,NR_R12);
              end;

            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (LocalSize<>0) or
               ((stackmisalignment<>0) and
                ((pi_do_call in current_procinfo.flags) or
                 (po_assembler in current_procinfo.procdef.procoptions))) then
              begin
                localsize:=align(localsize+stackmisalignment,current_settings.alignment.localalignmax)-stackmisalignment;
                if not(is_shifter_const(localsize,shift)) then
                  begin
                    if current_procinfo.framepointer=NR_STACK_POINTER_REG then
                      a_reg_alloc(list,NR_R12);
                    a_load_const_reg(list,OS_ADDR,LocalSize,NR_R12);
                    list.concat(taicpu.op_reg_reg_reg(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_R12));
                    a_reg_dealloc(list,NR_R12);
                  end
                else
                  begin
                    if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
                      a_reg_dealloc(list,NR_R12);
                    list.concat(taicpu.op_reg_reg_const(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize));
                  end;
              end;

            if firstfloatreg<>RS_NO then
              begin
                reference_reset(ref,4);
                if tg.direction*tarmprocinfo(current_procinfo).floatregstart>=1023 then
                  begin
                    a_load_const_reg(list,OS_ADDR,-tarmprocinfo(current_procinfo).floatregstart,NR_R12);
                    list.concat(taicpu.op_reg_reg_reg(A_SUB,NR_R12,current_procinfo.framepointer,NR_R12));
                    ref.base:=NR_R12;
                  end
                else
                  begin
                    ref.base:=current_procinfo.framepointer;
                    ref.offset:=tarmprocinfo(current_procinfo).floatregstart;
                  end;
                list.concat(taicpu.op_reg_const_ref(A_SFM,newreg(R_FPUREGISTER,firstfloatreg,R_SUBWHOLE),
                  lastfloatreg-firstfloatreg+1,ref));
              end;
          end;
      end;


    procedure Tthumb2cgarm.g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean);
      var
         ref : treference;
         firstfloatreg,lastfloatreg,
         r : byte;
         shift : byte;
         regs : tcpuregisterset;
         LocalSize : longint;
         stackmisalignment: pint;
      begin
        if not(nostackframe) then
          begin
            stackmisalignment:=0;
            { restore floating point register }
            firstfloatreg:=RS_NO;
            { save floating point registers? }
            for r:=RS_F0 to RS_F7 do
              if r in rg[R_FPUREGISTER].used_in_proc-paramanager.get_volatile_registers_fpu(pocall_stdcall) then
                begin
                  if firstfloatreg=RS_NO then
                    firstfloatreg:=r;
                  lastfloatreg:=r;
                  { floating point register space is already included in
                    localsize below by calc_stackframe_size
                   inc(stackmisalignment,12);
                  }
                end;

            if firstfloatreg<>RS_NO then
              begin
                reference_reset(ref,4);
                if tg.direction*tarmprocinfo(current_procinfo).floatregstart>=1023 then
                  begin
                    a_load_const_reg(list,OS_ADDR,-tarmprocinfo(current_procinfo).floatregstart,NR_R12);
                    list.concat(taicpu.op_reg_reg_reg(A_SUB,NR_R12,current_procinfo.framepointer,NR_R12));
                    ref.base:=NR_R12;
                  end
                else
                  begin
                    ref.base:=current_procinfo.framepointer;
                    ref.offset:=tarmprocinfo(current_procinfo).floatregstart;
                  end;
                list.concat(taicpu.op_reg_const_ref(A_LFM,newreg(R_FPUREGISTER,firstfloatreg,R_SUBWHOLE),
                  lastfloatreg-firstfloatreg+1,ref));
              end;

            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);
            if (pi_do_call in current_procinfo.flags) or (regs<>[]) then
              begin
                exclude(regs,RS_R14);
                include(regs,RS_R15);
              end;
            if (current_procinfo.framepointer<>NR_STACK_POINTER_REG) then
              regs:=regs+[RS_FRAME_POINTER_REG,RS_R15];

            for r:=RS_R0 to RS_R15 do
              if (r in regs) then
                inc(stackmisalignment,4);

            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            //if (current_procinfo.framepointer<>NR_STACK_POINTER_REG) then
              begin
                LocalSize:=current_procinfo.calc_stackframe_size;
                if (LocalSize<>0) or
                   ((stackmisalignment<>0) and
                    ((pi_do_call in current_procinfo.flags) or
                     (po_assembler in current_procinfo.procdef.procoptions))) then
                  begin
                    localsize:=align(localsize+stackmisalignment,current_settings.alignment.localalignmax)-stackmisalignment;
                    if not(is_shifter_const(LocalSize,shift)) then
                      begin
                        a_reg_alloc(list,NR_R12);
                        a_load_const_reg(list,OS_ADDR,LocalSize,NR_R12);
                        list.concat(taicpu.op_reg_reg_reg(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_R12));
                        a_reg_dealloc(list,NR_R12);
                      end
                    else
                      begin
                        if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
                          a_reg_dealloc(list,NR_R12);
                        list.concat(taicpu.op_reg_reg_const(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize));
                      end;
                  end;
              end;

            if regs=[] then
              list.concat(taicpu.op_reg(A_BX,NR_R14))
            else
              begin
                reference_reset(ref,4);
                ref.index:=NR_STACK_POINTER_REG;
                ref.addressmode:=AM_PREINDEXED;
                list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_FD));
              end;
          end
        else
          list.concat(taicpu.op_reg(A_BX,NR_R14));
      end;


   function Tthumb2cgarm.handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference;
      var
        tmpreg : tregister;
        tmpref : treference;
        l : tasmlabel;
        so: tshifterop;
      begin
        tmpreg:=NR_NO;

        { Be sure to have a base register }
        if (ref.base=NR_NO) then
          begin
            if ref.shiftmode<>SM_None then
              internalerror(200308294);
            ref.base:=ref.index;
            ref.index:=NR_NO;
          end;

        { absolute symbols can't be handled directly, we've to store the symbol reference
          in the text segment and access it pc relative

          For now, we assume that references where base or index equals to PC are already
          relative, all other references are assumed to be absolute and thus they need
          to be handled extra.

          A proper solution would be to change refoptions to a set and store the information
          if the symbol is absolute or relative there.
        }

        if (assigned(ref.symbol) and
            not(is_pc(ref.base)) and
            not(is_pc(ref.index))
           ) or
           { [#xxx] isn't a valid address operand }
           ((ref.base=NR_NO) and (ref.index=NR_NO)) or
           //(ref.offset<-4095) or
           (ref.offset<-255) or
           (ref.offset>4095) or
           ((oppostfix in [PF_SB,PF_H,PF_SH]) and
            ((ref.offset<-255) or
             (ref.offset>255)
            )
           ) or
           ((op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) and
            ((ref.offset<-1020) or
             (ref.offset>1020) or
             { the usual pc relative symbol handling assumes possible offsets of +/- 4095 }
             assigned(ref.symbol)
            )
           ) then
          begin
            reference_reset(tmpref,4);

            { load symbol }
            tmpreg:=getintregister(list,OS_INT);
            if assigned(ref.symbol) then
              begin
                current_asmdata.getjumplabel(l);
                cg.a_label(current_procinfo.aktlocaldata,l);
                tmpref.symboldata:=current_procinfo.aktlocaldata.last;

                current_procinfo.aktlocaldata.concat(tai_const.create_sym_offset(ref.symbol,ref.offset));

                { load consts entry }
                tmpref.symbol:=l;
                tmpref.base:=NR_R15;
                list.concat(taicpu.op_reg_ref(A_LDR,tmpreg,tmpref));

                { in case of LDF/STF, we got rid of the NR_R15 }
                if is_pc(ref.base) then
                  ref.base:=NR_NO;
                if is_pc(ref.index) then
                  ref.index:=NR_NO;
              end
            else
              a_load_const_reg(list,OS_ADDR,ref.offset,tmpreg);

            if (ref.base<>NR_NO) then
              begin
                if ref.index<>NR_NO then
                  begin
                    list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,tmpreg));
                    ref.base:=tmpreg;
                  end
                else
                  begin
                    ref.index:=tmpreg;
                    ref.shiftimm:=0;
                    ref.signindex:=1;
                    ref.shiftmode:=SM_None;
                  end;
              end
            else
              ref.base:=tmpreg;
            ref.offset:=0;
            ref.symbol:=nil;
          end;

        if (ref.base<>NR_NO) and (ref.index<>NR_NO) and (ref.offset<>0) then
          begin
            if tmpreg<>NR_NO then
              a_op_const_reg_reg(list,OP_ADD,OS_ADDR,ref.offset,tmpreg,tmpreg)
            else
              begin
                tmpreg:=getintregister(list,OS_ADDR);
                a_op_const_reg_reg(list,OP_ADD,OS_ADDR,ref.offset,ref.base,tmpreg);
                ref.base:=tmpreg;
              end;
            ref.offset:=0;
          end;

        { Hack? Thumb2 doesn't allow PC indexed addressing modes(although it does in the specification) }
        if (ref.base=NR_R15) and (ref.index<>NR_NO) and (ref.shiftmode <> sm_none) then
          begin
            tmpreg:=getintregister(list,OS_ADDR);

            list.concat(taicpu.op_reg_reg(A_MOV, tmpreg, NR_R15));

            ref.base := tmpreg;
          end;

        { floating point operations have only limited references
          we expect here, that a base is already set }
        if (op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) and (ref.index<>NR_NO) then
          begin
            if ref.shiftmode<>SM_none then
              internalerror(200309121);
            if tmpreg<>NR_NO then
              begin
                if ref.base=tmpreg then
                  begin
                    if ref.signindex<0 then
                      list.concat(taicpu.op_reg_reg_reg(A_SUB,tmpreg,tmpreg,ref.index))
                    else
                      list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,tmpreg,ref.index));
                    ref.index:=NR_NO;
                  end
                else
                  begin
                    if ref.index<>tmpreg then
                      internalerror(200403161);
                    if ref.signindex<0 then
                      list.concat(taicpu.op_reg_reg_reg(A_SUB,tmpreg,ref.base,tmpreg))
                    else
                      list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,tmpreg));
                    ref.base:=tmpreg;
                    ref.index:=NR_NO;
                  end;
              end
            else
              begin
                tmpreg:=getintregister(list,OS_ADDR);
                list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,ref.index));
                ref.base:=tmpreg;
                ref.index:=NR_NO;
              end;
          end;
        list.concat(setoppostfix(taicpu.op_reg_ref(op,reg,ref),oppostfix));
        Result := ref;
      end;


    procedure tthumb2cg64farm.a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);
      var tmpreg: tregister;
      begin
        case op of
          OP_NEG:
            begin
              list.concat(setoppostfix(taicpu.op_reg_reg_const(A_RSB,regdst.reglo,regsrc.reglo,0),PF_S));
              tmpreg:=cg.getintregister(list,OS_32);
              list.concat(taicpu.op_reg_const(A_MOV,tmpreg,0));
              list.concat(taicpu.op_reg_reg_reg(A_SBC,regdst.reghi,tmpreg,regsrc.reghi));
            end;
          else
            inherited a_op64_reg_reg(list, op, size, regsrc, regdst);
        end;
      end;


    procedure create_codegen;
      begin
        if current_settings.cputype in cpu_thumb2 then
          begin
            cg:=tthumb2cgarm.create;
            cg64:=tthumb2cg64farm.create;

            casmoptimizer:=TCpuThumb2AsmOptimizer;
          end
        else
          begin
            cg:=tarmcgarm.create;
            cg64:=tcg64farm.create;

            casmoptimizer:=TCpuAsmOptimizer;
          end;
      end;

end.
