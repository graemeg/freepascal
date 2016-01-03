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
       cpubase,cpuinfo,cg64f32,rgcpu;


    type
      { tbasecgarm is shared between all arm architectures }
      tbasecgarm = class(tcg)
        { true, if the next arithmetic operation should modify the flags }
        cgsetflags : boolean;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const paraloc : TCGPara);override;
=======
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const paraloc : TCGPara);override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const paraloc : TCGPara);override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const paraloc : TCGPara);override;
>>>>>>> graemeg/cpstrnew
=======
        procedure a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const paraloc : TCGPara);override;
>>>>>>> origin/cpstrnew
        procedure a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const paraloc : TCGPara);override;
        procedure a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const paraloc : TCGPara);override;

        procedure a_call_name(list : TAsmList;const s : string; weak: boolean);override;
        procedure a_call_reg(list : TAsmList;reg: tregister);override;

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

        procedure g_profilecode(list : TAsmList); override;

        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean); override;
        procedure g_maybe_got_init(list : TAsmList); override;

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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

        procedure a_loadmm_reg_reg(list: TAsmList; fromsize, tosize : tcgsize;reg1, reg2: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_ref_reg(list: TAsmList; fromsize, tosize : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_reg_ref(list: TAsmList; fromsize, tosize : tcgsize;reg: tregister; const ref: treference;shuffle : pmmshuffle); override;
        procedure a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize : tcgsize;intreg, mmreg: tregister; shuffle: pmmshuffle); override;
        procedure a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize : tcgsize;mmreg, intreg: tregister; shuffle : pmmshuffle); override;

        procedure a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;src,dst: tregister;shuffle : pmmshuffle); override;
        { Transform unsupported methods into Internal errors }
        procedure a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: TCGSize; src, dst: TRegister); override;

        { try to generate optimized 32 Bit multiplication, returns true if successful generated }
        function try_optimized_mul32_const_reg_reg(list: TAsmList; a: tcgint; src, dst: tregister) : boolean;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

        procedure g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);override;
<<<<<<< HEAD
<<<<<<< HEAD
        procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint); override;
        procedure g_stackpointer_alloc(list : TAsmList;size : longint);override;

        procedure a_loadmm_reg_reg(list: TAsmList; fromsize, tosize : tcgsize;reg1, reg2: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_ref_reg(list: TAsmList; fromsize, tosize : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_reg_ref(list: TAsmList; fromsize, tosize : tcgsize;reg: tregister; const ref: treference;shuffle : pmmshuffle); override;
        procedure a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize : tcgsize;intreg, mmreg: tregister; shuffle: pmmshuffle); override;
        procedure a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize : tcgsize;mmreg, intreg: tregister; shuffle : pmmshuffle); override;

        procedure a_opmm_reg_reg(list: TAsmList; Op: TOpCG; size : tcgsize;src,dst: tregister;shuffle : pmmshuffle); override;
      private
>>>>>>> graemeg/cpstrnew
        { clear out potential overflow bits from 8 or 16 bit operations  }
        { the upper 24/16 bits of a register after an operation          }
        procedure maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);

        { mla for thumb requires that none of the registers is equal to r13/r15, this method ensures this }
        procedure safe_mla(list: TAsmList;op1,op2,op3,op4 : TRegister);

      end;

      { tcgarm is shared between normal arm and thumb-2 }
      tcgarm = class(tbasecgarm)
        procedure a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister); override;
        procedure a_op_const_ref(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; const ref: TReference); override;
        procedure a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister); override;

        procedure a_op_const_reg_reg(list: TAsmList; op: TOpCg;
          size: tcgsize; a: tcgint; src, dst: tregister); override;
        procedure a_op_reg_reg_reg(list: TAsmList; op: TOpCg;
          size: tcgsize; src1, src2, dst: tregister); override;
        procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);override;
        procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);override;

        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;

        procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint); override;

        {Multiply two 32-bit registers into lo and hi 32-bit registers}
        procedure a_mul_reg_reg_pair(list: tasmlist; size: tcgsize; src1,src2,dstlo,dsthi: tregister); override;
      end;

      { normal arm cg }
      tarmcgarm = class(tcgarm)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;
      end;

<<<<<<< HEAD
      { 64 bit cg for all arm flavours }
      tbasecg64farm = class(tcg64f32)
      end;

      { tcg64farm is shared between normal arm and thumb-2 }
      tcg64farm = class(tbasecg64farm)
=======
      tarmcgarm = class(tcgarm)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;
      end;

      tarmcgarm = class(tcgarm)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;
      end;

      tarmcgarm = class(tcgarm)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;
      end;

      tarmcgarm = class(tcgarm)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;
=======
=======
>>>>>>> origin/fixes_2_2
        procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);
      private
        { clear out potential overflow bits from 8 or 16 bit operations  }
        { the upper 24/16 bits of a register after an operation          }
        procedure maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;

      tcg64farm = class(tcg64f32)
>>>>>>> graemeg/cpstrnew
        procedure a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);override;
        procedure a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);override;
        procedure a_op64_const_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64);override;
        procedure a_op64_reg_reg_reg(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64);override;
        procedure a_op64_const_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;value : int64;regsrc,regdst : tregister64;setflags : boolean;var ovloc : tlocation);override;
        procedure a_op64_reg_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64;setflags : boolean;var ovloc : tlocation);override;
        procedure a_loadmm_intreg64_reg(list: TAsmList; mmsize: tcgsize; intreg: tregister64; mmreg: tregister);override;
        procedure a_loadmm_reg_intreg64(list: TAsmList; mmsize: tcgsize; mmreg: tregister; intreg: tregister64);override;
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      tarmcg64farm = class(tcg64farm)
      end;

      tthumbcgarm = class(tbasecgarm)
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        procedure g_proc_entry(list: TAsmList; localsize: longint; nostackframe: boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean); override;

        procedure a_op_reg_reg(list: TAsmList; Op: TOpCG; size: TCGSize; src,dst: TRegister);override;
        procedure a_op_const_reg(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; dst: tregister);override;
        procedure a_op_const_reg_reg(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister); override;

        procedure g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister); override;

        procedure a_load_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const Ref: treference; reg: tregister);override;
        procedure a_load_const_reg(list: TAsmList; size: tcgsize; a: tcgint; reg: tregister);override;

        procedure g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint); override;

        function handle_load_store(list: TAsmList; op: tasmop; oppostfix: toppostfix; reg: tregister; ref: treference): treference; override;
      end;

      tthumbcg64farm = class(tbasecg64farm)
        procedure a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);override;
        procedure a_op64_const_reg(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;reg : tregister64);override;
      end;

      tthumb2cgarm = class(tcgarm)
=======
      Tthumb2cgarm = class(tcgarm)
>>>>>>> graemeg/cpstrnew
=======
      Tthumb2cgarm = class(tcgarm)
>>>>>>> graemeg/cpstrnew
=======
      Tthumb2cgarm = class(tcgarm)
>>>>>>> graemeg/cpstrnew
=======
      Tthumb2cgarm = class(tcgarm)
>>>>>>> origin/cpstrnew
        procedure init_register_allocators;override;
        procedure done_register_allocators;override;

        procedure a_call_reg(list : TAsmList;reg: tregister);override;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;

        procedure a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister); override;
        procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        procedure a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);override;
        procedure a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);override;

        procedure a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: aint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        procedure a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);override;

        procedure g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister); override;

        procedure g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);override;
        procedure g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean); override;

        function handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

        procedure a_loadmm_reg_reg(list: TAsmList; fromsize, tosize : tcgsize;reg1, reg2: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_ref_reg(list: TAsmList; fromsize, tosize : tcgsize;const ref: treference; reg: tregister;shuffle : pmmshuffle); override;
        procedure a_loadmm_reg_ref(list: TAsmList; fromsize, tosize : tcgsize;reg: tregister; const ref: treference;shuffle : pmmshuffle); override;
        procedure a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize : tcgsize;intreg, mmreg: tregister; shuffle: pmmshuffle); override;
        procedure a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize : tcgsize;mmreg, intreg: tregister; shuffle : pmmshuffle); override;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
       globals,verbose,systems,cutils,
       aopt,aoptcpu,
       fmodule,
       symconst,symsym,symtable,
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
            begin
              if assigned(current_procinfo) and (current_procinfo.framepointer<>NR_R11) then
                rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
                    [RS_R0,RS_R1,RS_R2,RS_R3,RS_R12,RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,
                     RS_R9,RS_R10,RS_R11,RS_R14],first_int_imreg,[])
              else
                rg[R_INTREGISTER]:=trgintcpu.create(R_INTREGISTER,R_SUBWHOLE,
                    [RS_R0,RS_R1,RS_R2,RS_R3,RS_R12,RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,
                     RS_R9,RS_R10,RS_R14],first_int_imreg,[])
            end
        else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          { r7 is not available on Darwin, it's used as frame pointer (always,
            for backtrace support -- also in gcc/clang -> R11 can be used).
            r9 is volatile }
=======
          { r9 is not (always) available on Darwin according to the llvm code
            generator. }
>>>>>>> graemeg/cpstrnew
=======
          { r9 is not (always) available on Darwin according to the llvm code
            generator. }
>>>>>>> graemeg/cpstrnew
=======
          { r9 is not (always) available on Darwin according to the llvm code
            generator. }
>>>>>>> graemeg/cpstrnew
=======
          { r9 is not (always) available on Darwin according to the llvm code
            generator. }
>>>>>>> origin/cpstrnew
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


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     procedure tcgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);
=======
     procedure tarmcgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);
>>>>>>> graemeg/cpstrnew
=======
     procedure tarmcgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);
>>>>>>> graemeg/cpstrnew
=======
     procedure tarmcgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);
>>>>>>> graemeg/cpstrnew
=======
     procedure tarmcgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);
>>>>>>> origin/cpstrnew
       var
          imm_shift : byte;
          l : tasmlabel;
          hr : treference;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          imm1, imm2: DWord;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       begin
          if not(size in [OS_8,OS_S8,OS_16,OS_S16,OS_32,OS_S32]) then
            internalerror(2002090902);
          if is_shifter_const(a,imm_shift) then
            list.concat(taicpu.op_reg_const(A_MOV,reg,a))
          else if is_shifter_const(not(a),imm_shift) then
            list.concat(taicpu.op_reg_const(A_MVN,reg,not(a)))
          { loading of constants with mov and orr }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          else if (split_into_shifter_const(a,imm1, imm2)) then
            begin
              list.concat(taicpu.op_reg_const(A_MOV,reg, imm1));
              list.concat(taicpu.op_reg_reg_const(A_ORR,reg,reg, imm2));
            end
          { loading of constants with mvn and bic }
          else if (split_into_shifter_const(not(a), imm1, imm2)) then
=======
          else if (is_shifter_const(a-byte(a),imm_shift)) then
=======
          else if (is_shifter_const(a-byte(a),imm_shift)) then
=======
          else if (is_shifter_const(a-byte(a),imm_shift)) then
=======
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


    procedure tcgarm.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const paraloc : TCGPara);
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
>>>>>>> origin/cpstrnew
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


<<<<<<< HEAD
     procedure tarmcgarm.a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);
       var
         oppostfix:toppostfix;
         usedtmpref: treference;
         tmpreg,tmpreg2 : tregister;
         so : tshifterop;
         dir : integer;
=======
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


     procedure tcgarm.a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: aint; reg: TRegister);
       begin
          a_op_const_reg_reg(list,op,size,a,reg,reg);
       end;


     procedure tcgarm.a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister);
>>>>>>> origin/cpstrnew
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


    procedure tcgarm.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const paraloc : TCGPara);
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
<<<<<<< HEAD
        paraloc.check_simple_location;
        paramanager.allocparaloc(list,paraloc.location);
        case paraloc.location^.loc of
          LOC_REGISTER,LOC_CREGISTER:
            a_loadaddr_ref_reg(list,r,paraloc.location^.register);
          LOC_REFERENCE:
>>>>>>> graemeg/cpstrnew
=======
        ovloc.loc:=LOC_VOID;
        if is_shifter_const(-a,shift) then
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


     procedure tcgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);
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
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
               reference_reset(hr,4);
=======
               reference_reset(hr);
>>>>>>> graemeg/fixes_2_2

               current_asmdata.getjumplabel(l);
               cg.a_label(current_procinfo.aktlocaldata,l);
               hr.symboldata:=current_procinfo.aktlocaldata.last;
               current_procinfo.aktlocaldata.concat(tai_const.Create_32bit(longint(a)));

<<<<<<< HEAD
               hr.symbol:=l;
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
=======
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


     procedure tcgarm.a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: aint; reg: TRegister);
       begin
          a_op_const_reg_reg(list,op,size,a,reg,reg);
       end;


<<<<<<< HEAD
     procedure tcgarm.a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister);
>>>>>>> graemeg/cpstrnew
       begin
         if (TCGSize2Size[FromSize] >= TCGSize2Size[ToSize]) then
           FromSize := ToSize;
=======
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
        if (op in [A_LDF,A_STF]) and (ref.index<>NR_NO) then
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
             InternalError(200308295);
         end;
         if ref.alignment<>0 then
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
                   shifterop_reset(so);so.shiftmode:=SM_LSR;so.shiftimm:=8;
                   tmpreg:=getintregister(list,OS_INT);
                   usedtmpref:=ref;
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
                 end
               else
                 handle_load_store(list,A_STR,oppostfix,reg,ref);
             end;
           end
         else
           handle_load_store(list,A_STR,oppostfix,reg,ref);
       end;


     procedure tcgarm.a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);
       var
         oppostfix:toppostfix;
         usedtmpref: treference;
         tmpreg,tmpreg2,tmpreg3 : tregister;
         so : tshifterop;
         dir : integer;
       begin
>>>>>>> graemeg/fixes_2_2
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
<<<<<<< HEAD
         if (ref.alignment in [1,2]) and (ref.alignment<tcgsize2size[fromsize]) then
=======
         if Ref.alignment<>0 then
>>>>>>> graemeg/fixes_2_2
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
<<<<<<< HEAD
                       tmpreg2:=getintregister(list,OS_INT);
                       a_loadaddr_ref_reg(list,ref,tmpreg2);
                       reference_reset_base(usedtmpref,tmpreg2,0,ref.alignment);
=======
                       tmpreg3:=getintregister(list,OS_INT);
                       a_loadaddr_ref_reg(list,ref,tmpreg3);
                       reference_reset_base(usedtmpref,tmpreg3,0);
>>>>>>> graemeg/fixes_2_2
                     end
                   else
                     usedtmpref:=ref;

                   if target_info.endian=endian_big then
                     inc(usedtmpref.offset,1);
                   shifterop_reset(so);so.shiftmode:=SM_LSL;so.shiftimm:=8;
                   tmpreg:=getintregister(list,OS_INT);
<<<<<<< HEAD
                   a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,reg);
                   inc(usedtmpref.offset,dir);
=======
                   a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                   inc(usedtmpref.offset,dir);
                   tmpreg2:=getintregister(list,OS_INT);
>>>>>>> graemeg/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
                   if target_info.endian=endian_big then
                     inc(usedtmpref.offset,3);
                   shifterop_reset(so);so.shiftmode:=SM_LSL;so.shiftimm:=8;
                   a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,reg);
                   inc(usedtmpref.offset,dir);
                   a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                   list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,tmpreg2,reg,tmpreg,so));
                   inc(usedtmpref.offset,dir);
                   a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,reg);
                   so.shiftimm:=16;
                   list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,tmpreg,tmpreg2,reg,so));
                   inc(usedtmpref.offset,dir);
                   a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg2);
                   so.shiftimm:=24;
                   list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,tmpreg,tmpreg2,so));
>>>>>>> graemeg/fixes_2_2
                 end
               else
                 handle_load_store(list,A_LDR,oppostfix,reg,ref);
             end;
           end
         else
           handle_load_store(list,A_LDR,oppostfix,reg,ref);
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

         if (fromsize=OS_S8) and (tosize = OS_16) then
           a_load_reg_reg(list,OS_16,OS_32,reg,reg);
       end;
>>>>>>> graemeg/fixes_2_2

         if (fromsize=OS_S8) and (tosize = OS_16) then
           a_load_reg_reg(list,OS_16,OS_32,reg,reg);
       end;


<<<<<<< HEAD
    procedure tcgarm.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const paraloc : TCGPara);
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
=======
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
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2


<<<<<<< HEAD
    procedure tcgarm.a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const paraloc : TCGPara);
=======
=======

       procedure do_shift(shiftmode : tshiftmode; shiftimm : byte; reg : tregister);
         begin
           so.shiftmode:=shiftmode;
           so.shiftimm:=shiftimm;
           list.concat(taicpu.op_reg_reg_shifterop(A_MOV,reg2,reg,so));
         end;

>>>>>>> origin/fixes_2_2
       var
         instr: taicpu;
         conv_done: boolean;
       begin
         if (tcgsize2size[fromsize]>32) or (tcgsize2size[tosize]>32) then
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


    procedure tcgarm.a_paramfpu_ref(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);
>>>>>>> graemeg/fixes_2_2
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
>>>>>>> graemeg/cpstrnew
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

<<<<<<< HEAD
               current_asmdata.getjumplabel(l);
               cg.a_label(current_procinfo.aktlocaldata,l);
               hr.symboldata:=current_procinfo.aktlocaldata.last;
               current_procinfo.aktlocaldata.concat(tai_const.Create_32bit(longint(a)));
=======
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
>>>>>>> graemeg/cpstrnew

               hr.symbol:=l;
               list.concat(taicpu.op_reg_ref(A_LDR,reg,hr));
            end;
       end;

<<<<<<< HEAD
=======
    procedure tcgarm.a_call_ref(list : TAsmList;ref: treference);
      begin
        a_reg_alloc(list,NR_R12);
        a_load_ref_reg(list,OS_ADDR,OS_ADDR,ref,NR_R12);
        a_call_reg(list,NR_R12);
        a_reg_dealloc(list,NR_R12);
        include(current_procinfo.flags,pi_do_call);
      end;
>>>>>>> graemeg/cpstrnew

     procedure tarmcgarm.a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);
       var
         oppostfix:toppostfix;
         usedtmpref: treference;
         tmpreg,tmpreg2 : tregister;
         so : tshifterop;
         dir : integer;
       begin
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
         case fromsize of
>>>>>>> graemeg/fixes_2_2
=======
         case fromsize of
>>>>>>> origin/fixes_2_2
           OS_32,
           OS_S32:
             oppostfix:=PF_None;
           else
             InternalError(200308297);
         end;
<<<<<<< HEAD
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
=======
         handle_load_store(list,A_LDF,oppostfix,reg,ref);
         if fromsize<>tosize then
           a_loadfpu_reg_reg(list,fromsize,tosize,reg,reg);
       end;
>>>>>>> graemeg/fixes_2_2

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


    procedure tcgarm.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : aint;const paraloc : TCGPara);
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
>>>>>>> graemeg/cpstrnew
            begin
              list.concat(taicpu.op_reg_const(A_MVN,reg, imm1));
              list.concat(taicpu.op_reg_reg_const(A_BIC,reg,reg, imm2));
            end
          else
<<<<<<< HEAD
            begin
               reference_reset(hr,4);
=======
            list.concat(taicpu.op_sym(A_BL,current_asmdata.WeakRefAsmSymbol(s)))
        else
          list.concat(taicpu.op_sym(A_BL,get_darwin_call_stub(s,weak)));
{
        the compiler does not properly set this flag anymore in pass 1, and
        for now we only need it after pass 2 (I hope) (JM)
          if not(pi_do_call in current_procinfo.flags) then
            internalerror(2003060703);
}
        include(current_procinfo.flags,pi_do_call);
      end;

<<<<<<< HEAD

    procedure tcgarm.a_call_reg(list : TAsmList;reg: tregister);
      begin
        { check not really correct: should only be used for non-Thumb cpus }
        if (current_settings.cputype<cpu_armv5) then
          begin
            list.concat(taicpu.op_reg_reg(A_MOV,NR_R14,NR_PC));
            list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,reg));
=======
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
<<<<<<< HEAD
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
=======
                reference_reset(ref);
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
>>>>>>> graemeg/fixes_2_2
              end;
            OP_ROR:
              begin
<<<<<<< HEAD
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
=======
                reference_reset(ref);
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
>>>>>>> graemeg/fixes_2_2
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
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew

               current_asmdata.getjumplabel(l);
               cg.a_label(current_procinfo.aktlocaldata,l);
               hr.symboldata:=current_procinfo.aktlocaldata.last;
               current_procinfo.aktlocaldata.concat(tai_const.Create_32bit(longint(a)));

               hr.symbol:=l;
               hr.base:=NR_PC;
               list.concat(taicpu.op_reg_ref(A_LDR,reg,hr));
            end;
       end;


     procedure tcgarm.a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);
       var
         oppostfix:toppostfix;
         usedtmpref: treference;
         tmpreg,tmpreg2 : tregister;
=======


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
>>>>>>> graemeg/cpstrnew
         so : tshifterop;
         dir : integer;
       begin
         if (TCGSize2Size[FromSize] >= TCGSize2Size[ToSize]) then
           FromSize := ToSize;
<<<<<<< HEAD
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

         if (fromsize=OS_S8) and
            (not (CPUARM_HAS_ALL_MEM in cpu_capabilities[current_settings.cputype])) then
           oppostfix:=PF_B;

         if ((ref.alignment in [1,2]) and (ref.alignment<tcgsize2size[fromsize])) or
            ((not (CPUARM_HAS_ALL_MEM in cpu_capabilities[current_settings.cputype])) and
             (oppostfix in [PF_SH,PF_H])) then
=======
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
             InternalError(200308295);
         end;
         if (ref.alignment in [1,2]) and (ref.alignment<tcgsize2size[tosize]) then
>>>>>>> graemeg/cpstrnew
           begin
             if target_info.endian=endian_big then
               dir:=-1
             else
               dir:=1;
             case FromSize of
               OS_16,OS_S16:
                 begin
<<<<<<< HEAD
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
=======
                   shifterop_reset(so);so.shiftmode:=SM_LSR;so.shiftimm:=8;
                   tmpreg:=getintregister(list,OS_INT);
                   usedtmpref:=ref;
                   if target_info.endian=endian_big then
                     inc(usedtmpref.offset,1);
                   usedtmpref:=a_internal_load_reg_ref(list,OS_8,OS_8,reg,usedtmpref);
                   inc(usedtmpref.offset,dir);
                   list.concat(taicpu.op_reg_reg_shifterop(A_MOV,tmpreg,reg,so));
                   a_internal_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref);
>>>>>>> graemeg/cpstrnew
                 end;
               OS_32,OS_S32:
                 begin
                   tmpreg:=getintregister(list,OS_INT);
<<<<<<< HEAD

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

         if (fromsize=OS_S8) and
            (not (CPUARM_HAS_ALL_MEM in cpu_capabilities[current_settings.cputype])) then
           a_load_reg_reg(list,OS_S8,OS_32,reg,reg)
         else if (fromsize=OS_S8) and (tosize = OS_16) then
           a_load_reg_reg(list,OS_16,OS_32,reg,reg);
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
=======
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
>>>>>>> graemeg/cpstrnew


    procedure tbasecgarm.a_load_const_cgpara(list : TAsmList;size : tcgsize;a : tcgint;const paraloc : TCGPara);
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


    procedure tbasecgarm.a_load_ref_cgpara(list : TAsmList;size : tcgsize;const r : treference;const paraloc : TCGPara);
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


    procedure tbasecgarm.a_loadaddr_ref_cgpara(list : TAsmList;const r : treference;const paraloc : TCGPara);
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


    procedure tbasecgarm.a_call_name(list : TAsmList;const s : string; weak: boolean);
      var
        branchopcode: tasmop;
        r : treference;
        sym : TAsmSymbol;
      begin
        { check not really correct: should only be used for non-Thumb cpus }
        // if (CPUARM_HAS_BLX_LABEL in cpu_capabilities[current_settings.cputype]) and
        //   { WinCE GNU AS (not sure if this applies in general) does not support BLX imm }
        // (target_info.system<>system_arm_wince) then
        //   branchopcode:=A_BLX
        // else
        { use always BL as newer binutils do not translate blx apparently
          generating BL is also what clang and gcc do by default }
          branchopcode:=A_BL;
        if not(weak) then
          sym:=current_asmdata.RefAsmSymbol(s)
        else
          sym:=current_asmdata.WeakRefAsmSymbol(s);
        reference_reset_symbol(r,sym,0,sizeof(pint));

        if (tf_pic_uses_got in target_info.flags) and
           (cs_create_pic in current_settings.moduleswitches) then
          begin
            r.refaddr:=addr_pic
          end
        else
          r.refaddr:=addr_full;

        list.concat(taicpu.op_ref(branchopcode,r));
{
        the compiler does not properly set this flag anymore in pass 1, and
        for now we only need it after pass 2 (I hope) (JM)
          if not(pi_do_call in current_procinfo.flags) then
            internalerror(2003060703);
}
        include(current_procinfo.flags,pi_do_call);
      end;


    procedure tbasecgarm.a_call_reg(list : TAsmList;reg: tregister);
      begin
        { check not really correct: should only be used for non-Thumb cpus }
        if not(CPUARM_HAS_BLX in cpu_capabilities[current_settings.cputype]) then
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


     procedure tcgarm.a_op_const_reg(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; reg: TRegister);
       begin
          a_op_const_reg_reg(list,op,size,a,reg,reg);
       end;

     procedure tcgarm.a_op_const_ref(list : TAsmList; Op: TOpCG; size: TCGSize; a: tcgint; const ref: TReference);
       var
         tmpreg,tmpresreg : tregister;
         tmpref : treference;
       begin
         tmpreg:=getintregister(list,size);
         tmpresreg:=getintregister(list,size);
         tmpref:=a_internal_load_ref_reg(list,size,size,ref,tmpreg);
         a_op_const_reg_reg(list,op,size,a,tmpreg,tmpresreg);
         a_load_reg_ref(list,size,size,tmpresreg,tmpref);
       end;

     procedure tcgarm.a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister);
       var
         so : tshifterop;
       begin
         if op = OP_NEG then
           begin
             list.concat(taicpu.op_reg_reg_const(A_RSB,dst,src,0));
             maybeadjustresult(list,OP_NEG,size,dst);
           end
         else if op = OP_NOT then
           begin
             if size in [OS_8, OS_16, OS_S8, OS_S16] then
               begin
                 shifterop_reset(so);
                 so.shiftmode:=SM_LSL;
                 if size in [OS_8, OS_S8] then
                   so.shiftimm:=24
                 else
                   so.shiftimm:=16;
                 list.concat(taicpu.op_reg_reg_shifterop(A_MVN,dst,src,so));
                 {Using a shift here allows this to be folded into another instruction}
                 if size in [OS_S8, OS_S16] then
                   so.shiftmode:=SM_ASR
                 else
                   so.shiftmode:=SM_LSR;
                 list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,dst,so));
               end
             else
               list.concat(taicpu.op_reg_reg(A_MVN,dst,src));
           end
         else
           a_op_reg_reg_reg(list,op,size,src,dst,dst);
       end;


    const
      op_reg_reg_opcg2asmop: array[TOpCG] of tasmop =
        (A_NONE,A_MOV,A_ADD,A_AND,A_NONE,A_NONE,A_MUL,A_MUL,A_NONE,A_NONE,A_ORR,
         A_NONE,A_NONE,A_NONE,A_SUB,A_EOR,A_NONE,A_NONE);

      op_reg_opcg2asmop: array[TOpCG] of tasmop =
        (A_NONE,A_MOV,A_ADD,A_AND,A_NONE,A_NONE,A_MUL,A_MUL,A_NONE,A_NONE,A_ORR,
         A_ASR,A_LSL,A_LSR,A_SUB,A_EOR,A_NONE,A_ROR);

      op_reg_postfix: array[TOpCG] of TOpPostfix =
        (PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,
         PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,PF_None);

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

    function opshift2shiftmode(op: TOpCg): tshiftmode;
      begin
        case op of
          OP_SHL: Result:=SM_LSL;
          OP_SHR: Result:=SM_LSR;
          OP_ROR: Result:=SM_ROR;
          OP_ROL: Result:=SM_ROR;
          OP_SAR: Result:=SM_ASR;
          else internalerror(2012070501);
        end
      end;


    function tbasecgarm.try_optimized_mul32_const_reg_reg(list: TAsmList; a: tcgint; src, dst: tregister) : boolean;
      var
        multiplier : dword;
        power : longint;
        shifterop : tshifterop;
        bitsset : byte;
        negative : boolean;
        first : boolean;
        b,
        cycles : byte;
        maxeffort : byte;
      begin
        result:=true;
        cycles:=0;
        negative:=a<0;
        shifterop.rs:=NR_NO;
        shifterop.shiftmode:=SM_LSL;
        if negative then
          inc(cycles);
        multiplier:=dword(abs(a));
        bitsset:=popcnt(multiplier and $fffffffe);

        { heuristics to estimate how much instructions are reasonable to replace the mul,
          this is currently based on XScale timings }
        { in the simplest case, we need a mov to load the constant and a mul to carry out the
          actual multiplication, this requires min. 1+4 cycles

          because the first shift imm. might cause a stall and because we need more instructions
          when replacing the mul we generate max. 3 instructions to replace this mul }
        maxeffort:=3;

        { if the constant is not a shifter op, we need either some mov/mvn/bic/or sequence or
          a ldr, so generating one more operation to replace this is beneficial }
        if not(is_shifter_const(dword(a),b)) and not(is_shifter_const(not(dword(a)),b)) then
          inc(maxeffort);

        { if the upper 5 bits are all set or clear, mul is one cycle faster }
        if ((dword(a) and $f8000000)=0) or ((dword(a) and $f8000000)=$f8000000) then
          dec(maxeffort);

        { if the upper 17 bits are all set or clear, mul is another cycle faster }
        if ((dword(a) and $ffff8000)=0) or ((dword(a) and $ffff8000)=$ffff8000) then
          dec(maxeffort);

        { most simple cases }
        if a=1 then
          a_load_reg_reg(list,OS_32,OS_32,src,dst)
        else if a=0 then
          a_load_const_reg(list,OS_32,0,dst)
        else if a=-1 then
          a_op_reg_reg(list,OP_NEG,OS_32,src,dst)
        { add up ?

          basically, one add is needed for each bit being set in the constant factor
          however, the least significant bit is for free, it can be hidden in the initial
          instruction
        }
        else if (bitsset+cycles<=maxeffort) and
          (bitsset<=popcnt(dword(nextpowerof2(multiplier,power)-multiplier) and $fffffffe)) then
          begin
            first:=true;
            while multiplier<>0 do
              begin
                shifterop.shiftimm:=BsrDWord(multiplier);
                if odd(multiplier) then
                  begin
                    list.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,dst,src,src,shifterop));
                    dec(multiplier);
                  end
                else
                  if first then
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,shifterop))
                  else
                    list.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,dst,dst,src,shifterop));
                first:=false;
                dec(multiplier,1 shl shifterop.shiftimm);
              end;
            if negative then
              list.concat(taicpu.op_reg_reg_const(A_RSB,dst,dst,0));
          end
        { subtract from the next greater power of two? }
        else if popcnt(dword(nextpowerof2(multiplier,power)-multiplier) and $fffffffe)+cycles+1<=maxeffort then
          begin
            first:=true;
            while multiplier<>0 do
              begin
                if first then
                  begin
                    multiplier:=(1 shl power)-multiplier;
                    shifterop.shiftimm:=power;
                  end
                else
                  shifterop.shiftimm:=BsrDWord(multiplier);

                if odd(multiplier) then
                  begin
                    list.concat(taicpu.op_reg_reg_reg_shifterop(A_RSB,dst,src,src,shifterop));
                    dec(multiplier);
                  end
                else
                  if first then
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,shifterop))
                  else
                    begin
                      list.concat(taicpu.op_reg_reg_reg_shifterop(A_SUB,dst,dst,src,shifterop));
                      dec(multiplier,1 shl shifterop.shiftimm);
                    end;
                first:=false;
              end;
            if negative then
              list.concat(taicpu.op_reg_reg_const(A_RSB,dst,dst,0));
          end
        else
          result:=false;
      end;


    procedure tcgarm.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
      var
        shift, lsb, width : byte;
        tmpreg : tregister;
        so : tshifterop;
        l1 : longint;
        imm1, imm2: DWord;
      begin
        optimize_op_const(size, op, a);
        case op of
          OP_NONE:
            begin
              if src <> dst then
                a_load_reg_reg(list, size, size, src, dst);
              exit;
            end;
          OP_MOVE:
            begin
              a_load_const_reg(list, size, a, dst);
              exit;
            end;
        end;
        ovloc.loc:=LOC_VOID;
        if {$ifopt R+}(a<>-2147483648) and{$endif} not setflags and is_shifter_const(-a,shift) then
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
            OP_SHL,
            OP_SHR,
            OP_ROL,
            OP_ROR,
            OP_SAR:
              begin
                if a>32 then
                  internalerror(200308294);
                shifterop_reset(so);
                so.shiftmode:=opshift2shiftmode(op);
                if op = OP_ROL then
                  so.shiftimm:=32-a
                else
                  so.shiftimm:=a;
                list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
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
                begin
                  if cgsetflags or setflags then
                    a_reg_alloc(list,NR_DEFAULTFLAGS);
                  list.concat(setoppostfix(
                    taicpu.op_reg_reg_const(op_reg_reg_opcg2asmop[op],dst,src,a),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))));
                end;
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
            if (op in [OP_IMUL,OP_IDIV]) and (a=-1) then
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
            else if (op in [OP_MUL,OP_IMUL]) and not(cgsetflags or setflags) and try_optimized_mul32_const_reg_reg(list,a,src,dst) then
              begin
                { nothing to do on success }
              end
            { BIC clears the specified bits, while AND keeps them, using BIC allows to use a
              broader range of shifterconstants.}
            else if (op = OP_AND) and is_shifter_const(not(dword(a)),shift) then
              list.concat(taicpu.op_reg_reg_const(A_BIC,dst,src,not(dword(a))))
            { Doing two shifts instead of two bics might allow the peephole optimizer to fold the second shift
              into the following instruction}
            else if (op = OP_AND) and
                    is_continuous_mask(a, lsb, width) and
                    ((lsb = 0) or ((lsb + width) = 32)) then
              begin
                shifterop_reset(so);
                if (width = 16) and
                   (lsb = 0) and
                   (current_settings.cputype >= cpu_armv6) then
                  list.concat(taicpu.op_reg_reg(A_UXTH,dst,src))
                else if (width = 8) and
                   (lsb = 0) and
                   (current_settings.cputype >= cpu_armv6) then
                  list.concat(taicpu.op_reg_reg(A_UXTB,dst,src))
                else if lsb = 0 then
                  begin
                    so.shiftmode:=SM_LSL;
                    so.shiftimm:=32-width;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                    so.shiftmode:=SM_LSR;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,dst,so));
                  end
                else
                  begin
                    so.shiftmode:=SM_LSR;
                    so.shiftimm:=lsb;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src,so));
                    so.shiftmode:=SM_LSL;
                    list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,dst,so));
                  end;
              end
            else if (op = OP_AND) and split_into_shifter_const(not(dword(a)), imm1, imm2) then
              begin
                list.concat(taicpu.op_reg_reg_const(A_BIC,dst,src,imm1));
                list.concat(taicpu.op_reg_reg_const(A_BIC,dst,dst,imm2));
              end
            else if (op in [OP_ADD, OP_SUB, OP_OR, OP_XOR]) and
                    not(cgsetflags or setflags) and
                    split_into_shifter_const(a, imm1, imm2) then
              begin
                list.concat(taicpu.op_reg_reg_const(op_reg_reg_opcg2asmop[op],dst,src,imm1));
                list.concat(taicpu.op_reg_reg_const(op_reg_reg_opcg2asmop[op],dst,dst,imm2));
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
            internalerror(200308283);
          OP_SHL,
          OP_SHR,
          OP_SAR,
          OP_ROR:
            begin
              if (op = OP_ROR) and not(size in [OS_32,OS_S32]) then
                internalerror(2008072801);
              shifterop_reset(so);
              so.rs:=src1;
              so.shiftmode:=opshift2shiftmode(op);
              list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src2,so));
            end;
          OP_ROL:
            begin
              if not(size in [OS_32,OS_S32]) then
                internalerror(2008072801);
              { simulate ROL by ror'ing 32-value }
              tmpreg:=getintregister(list,OS_32);
              list.concat(taicpu.op_reg_reg_const(A_RSB,tmpreg,src1, 32));
              shifterop_reset(so);
              so.rs:=tmpreg;
              so.shiftmode:=SM_ROR;
              list.concat(taicpu.op_reg_reg_shifterop(A_MOV,dst,src2,so));
            end;
          OP_IMUL,
          OP_MUL:
            begin
              if (cgsetflags or setflags) and
                 (CPUARM_HAS_UMULL in cpu_capabilities[current_settings.cputype]) then
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
                  a_reg_alloc(list,NR_DEFAULTFLAGS);
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
            begin
              if cgsetflags or setflags then
                a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(setoppostfix(
                taicpu.op_reg_reg_reg(op_reg_reg_opcg2asmop[op],dst,src2,src1),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))));
            end;
        end;
        maybeadjustresult(list,op,size,dst);
      end;

    procedure tcgarm.a_mul_reg_reg_pair(list: tasmlist; size: tcgsize; src1,src2,dstlo,dsthi: tregister);
    var
      asmop: tasmop;
    begin
      if CPUARM_HAS_UMULL in cpu_capabilities[current_settings.cputype] then
        begin
          list.concat(tai_comment.create(strpnew('tcgarm.a_mul_reg_reg_pair called')));
          case size of
            OS_32:  asmop:=A_UMULL;
            OS_S32: asmop:=A_SMULL;
            else
              InternalError(2014060802);
          end;
          { The caller might omit dstlo or dsthi, when he is not interested in it, we still
            need valid registers everywhere. In case of dsthi = NR_NO we could fall back to
            32x32=32 bit multiplication}
          if (dstlo = NR_NO) then
            dstlo:=getintregister(list,size);
          if (dsthi = NR_NO) then
            dsthi:=getintregister(list,size);
          list.concat(taicpu.op_reg_reg_reg_reg(asmop, dstlo, dsthi, src1,src2));
        end
      else if dsthi=NR_NO then
        begin
          if (dstlo = NR_NO) then
            dstlo:=getintregister(list,size);
          list.concat(taicpu.op_reg_reg_reg(A_MUL, dstlo, src1,src2));
        end
      else
        begin
          internalerror(2015083022);
        end;
    end;

    function tbasecgarm.handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference;
      var
        tmpreg1,tmpreg2 : tregister;
      begin
        tmpreg1:=NR_NO;

        { Be sure to have a base register }
        if (ref.base=NR_NO) then
          begin
            if ref.shiftmode<>SM_None then
              internalerror(2014020701);
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
           (((op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) or (op=A_VSTR) or (op=A_VLDR)) and
            ((ref.offset<-1020) or
             (ref.offset>1020) or
             ((abs(ref.offset) mod 4)<>0)
            )
           ) or
           ((GenerateThumbCode) and
            (((oppostfix in [PF_SB,PF_SH]) and (ref.offset<>0)) or
             ((oppostfix=PF_None) and ((ref.offset<0) or ((ref.base<>NR_STACK_POINTER_REG) and (ref.offset>124)) or
               ((ref.base=NR_STACK_POINTER_REG) and (ref.offset>1020)) or ((ref.offset mod 4)<>0))) or
             ((oppostfix=PF_H) and ((ref.offset<0) or (ref.offset>62) or ((ref.offset mod 2)<>0) or ((getsupreg(ref.base) in [RS_R8..RS_R15]) and (ref.offset<>0)))) or
             ((oppostfix=PF_B) and ((ref.offset<0) or (ref.offset>31) or ((getsupreg(ref.base) in [RS_R8..RS_R15]) and (ref.offset<>0))))
            )
           ) then
          begin
            fixref(list,ref);
          end;

        if GenerateThumbCode then
          begin
            { certain thumb load require base and index }
            if (oppostfix in [PF_SB,PF_SH]) and
              (ref.base<>NR_NO) and (ref.index=NR_NO) then
              begin
                tmpreg1:=getintregister(list,OS_ADDR);
                a_load_const_reg(list,OS_ADDR,0,tmpreg1);
                ref.index:=tmpreg1;
              end;

            { "hi" registers cannot be used as base or index }
            if (getsupreg(ref.base) in [RS_R8..RS_R12,RS_R14]) or
              ((ref.base=NR_R13) and (ref.index<>NR_NO)) then
              begin
                tmpreg1:=getintregister(list,OS_ADDR);
                a_load_reg_reg(list,OS_ADDR,OS_ADDR,ref.base,tmpreg1);
                ref.base:=tmpreg1;
              end;
            if getsupreg(ref.index) in [RS_R8..RS_R14] then
              begin
                tmpreg1:=getintregister(list,OS_ADDR);
                a_load_reg_reg(list,OS_ADDR,OS_ADDR,ref.index,tmpreg1);
                ref.index:=tmpreg1;
              end;
          end;

        { fold if there is base, index and offset, however, don't fold
          for vfp memory instructions because we later fold the index }
        if not((op in [A_FLDS,A_FLDD,A_FSTS,A_FSTD]) or (op=A_VSTR) or (op=A_VLDR)) and
           (ref.base<>NR_NO) and (ref.index<>NR_NO) and (ref.offset<>0) then
          begin
            if tmpreg1<>NR_NO then
              begin
                tmpreg2:=getintregister(list,OS_ADDR);
                a_op_const_reg_reg(list,OP_ADD,OS_ADDR,ref.offset,tmpreg1,tmpreg2);
                tmpreg1:=tmpreg2;
              end
            else
              begin
                tmpreg1:=getintregister(list,OS_ADDR);
                a_op_const_reg_reg(list,OP_ADD,OS_ADDR,ref.offset,ref.base,tmpreg1);
                ref.base:=tmpreg1;
              end;
            ref.offset:=0;
          end;

        { floating point operations have only limited references
          we expect here, that a base is already set }
        if ((op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) or (op=A_VSTR) or (op=A_VLDR)) and (ref.index<>NR_NO) then
          begin
            if ref.shiftmode<>SM_none then
              internalerror(200309121);
            if tmpreg1<>NR_NO then
              begin
                if ref.base=tmpreg1 then
                  begin
                    if ref.signindex<0 then
                      list.concat(taicpu.op_reg_reg_reg(A_SUB,tmpreg1,tmpreg1,ref.index))
                    else
                      list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg1,tmpreg1,ref.index));
                    ref.index:=NR_NO;
                  end
                else
                  begin
                    if ref.index<>tmpreg1 then
                      internalerror(200403161);
                    if ref.signindex<0 then
                      list.concat(taicpu.op_reg_reg_reg(A_SUB,tmpreg1,ref.base,tmpreg1))
                    else
                      list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg1,ref.base,tmpreg1));
                    ref.base:=tmpreg1;
                    ref.index:=NR_NO;
                  end;
              end
            else
              begin
                tmpreg1:=getintregister(list,OS_ADDR);
                list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg1,ref.base,ref.index));
                ref.base:=tmpreg1;
                ref.index:=NR_NO;
              end;
          end;
        list.concat(setoppostfix(taicpu.op_reg_ref(op,reg,ref),oppostfix));
        Result := ref;
      end;


     procedure tbasecgarm.a_load_reg_ref(list : TAsmList; fromsize, tosize: tcgsize; reg : tregister;const ref : treference);
       var
         oppostfix:toppostfix;
         usedtmpref: treference;
         tmpreg : tregister;
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

         if ((ref.alignment in [1,2]) and (ref.alignment<tcgsize2size[tosize])) or
            ((not (CPUARM_HAS_ALL_MEM in cpu_capabilities[current_settings.cputype])) and
             (oppostfix =PF_H)) then
           begin
             if target_info.endian=endian_big then
               dir:=-1
             else
               dir:=1;
             case FromSize of
               OS_16,OS_S16:
                 begin
                   tmpreg:=getintregister(list,OS_INT);
                   usedtmpref:=ref;
                   if target_info.endian=endian_big then
                     inc(usedtmpref.offset,1);
                   usedtmpref:=a_internal_load_reg_ref(list,OS_8,OS_8,reg,usedtmpref);
                   inc(usedtmpref.offset,dir);
                   a_op_const_reg_reg(list,OP_SHR,OS_INT,8,reg,tmpreg);
                   a_internal_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref);
                 end;
               OS_32,OS_S32:
                 begin
                   tmpreg:=getintregister(list,OS_INT);
                   usedtmpref:=ref;
                   if ref.alignment=2 then
                     begin
                       if target_info.endian=endian_big then
                         inc(usedtmpref.offset,2);
                       usedtmpref:=a_internal_load_reg_ref(list,OS_16,OS_16,reg,usedtmpref);
                       a_op_const_reg_reg(list,OP_SHR,OS_INT,16,reg,tmpreg);
                       inc(usedtmpref.offset,dir*2);
                       a_internal_load_reg_ref(list,OS_16,OS_16,tmpreg,usedtmpref);
                     end
                   else
                     begin
                       if target_info.endian=endian_big then
                         inc(usedtmpref.offset,3);
                       usedtmpref:=a_internal_load_reg_ref(list,OS_8,OS_8,reg,usedtmpref);
                       a_op_const_reg_reg(list,OP_SHR,OS_INT,8,reg,tmpreg);
                       inc(usedtmpref.offset,dir);
                       a_internal_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref);
                       a_op_const_reg(list,OP_SHR,OS_INT,8,tmpreg);
                       inc(usedtmpref.offset,dir);
                       a_internal_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref);
                       a_op_const_reg(list,OP_SHR,OS_INT,8,tmpreg);
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


     function tbasecgarm.a_internal_load_reg_ref(list : TAsmList; fromsize, tosize: tcgsize; reg : tregister;const ref : treference):treference;
       var
         oppostfix:toppostfix;
         href: treference;
         tmpreg: TRegister;
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

         if (tosize in [OS_S16,OS_16]) and
            (not (CPUARM_HAS_ALL_MEM in cpu_capabilities[current_settings.cputype])) then
           begin
             result:=handle_load_store(list,A_STR,PF_B,reg,ref);

             tmpreg:=getintregister(list,OS_INT);
             a_op_const_reg_reg(list,OP_SHR,OS_INT,8,reg,tmpreg);

             href:=result;
             inc(href.offset);

             handle_load_store(list,A_STR,PF_B,tmpreg,href);
           end
         else
           result:=handle_load_store(list,A_STR,oppostfix,reg,ref);
       end;


     function tbasecgarm.a_internal_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister):treference;
       var
         oppostfix:toppostfix;
         so: tshifterop;
         tmpreg: TRegister;
         href: treference;
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

         if (tosize=OS_S8) and
            (not (CPUARM_HAS_ALL_MEM in cpu_capabilities[current_settings.cputype])) then
           begin
             result:=handle_load_store(list,A_LDR,PF_B,reg,ref);
             a_load_reg_reg(list,OS_S8,OS_32,reg,reg);
           end
         else if (tosize in [OS_S16,OS_16]) and
            (not (CPUARM_HAS_ALL_MEM in cpu_capabilities[current_settings.cputype])) then
           begin
             result:=handle_load_store(list,A_LDR,PF_B,reg,ref);

             tmpreg:=getintregister(list,OS_INT);

             href:=result;
             inc(href.offset);

             handle_load_store(list,A_LDR,PF_B,tmpreg,href);

             shifterop_reset(so);
             so.shiftmode:=SM_LSL;
             so.shiftimm:=8;

             list.concat(taicpu.op_reg_reg_reg_shifterop(A_ORR,reg,reg,tmpreg,so));
           end
         else
           result:=handle_load_store(list,A_LDR,oppostfix,reg,ref);
       end;

     procedure tbasecgarm.a_load_reg_reg(list : TAsmList; fromsize, tosize : tcgsize;reg1,reg2 : tregister);
       var
         so : tshifterop;

       procedure do_shift(shiftmode : tshiftmode; shiftimm : byte; reg : tregister);
         begin
           if GenerateThumbCode then
             begin
               case shiftmode of
                 SM_ASR:
                   a_op_const_reg_reg(list,OP_SAR,OS_32,shiftimm,reg,reg2);
                 SM_LSR:
                   a_op_const_reg_reg(list,OP_SHR,OS_32,shiftimm,reg,reg2);
                 SM_LSL:
                   a_op_const_reg_reg(list,OP_SHL,OS_32,shiftimm,reg,reg2);
                 else
                   internalerror(2013090301);
               end;
             end
           else
             begin
               so.shiftmode:=shiftmode;
               so.shiftimm:=shiftimm;
               list.concat(taicpu.op_reg_reg_shifterop(A_MOV,reg2,reg,so));
             end;
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
             if current_settings.cputype<cpu_armv6 then
               case fromsize of
                 OS_8:
                   if GenerateThumbCode then
                     a_op_const_reg_reg(list,OP_AND,OS_32,$ff,reg1,reg2)
                   else
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
               end
             else
               case fromsize of
                 OS_8:
                   if GenerateThumbCode then
                     list.concat(taicpu.op_reg_reg(A_UXTB,reg2,reg1))
                   else
                     list.concat(taicpu.op_reg_reg_const(A_AND,reg2,reg1,$ff));
                 OS_S8:
                   begin
                     if tosize=OS_16 then
                       begin
                         so.shiftmode:=SM_ROR;
                         so.shiftimm:=16;
                         list.concat(taicpu.op_reg_reg_shifterop(A_SXTB16,reg2,reg1,so));
                         do_shift(SM_LSR,16,reg2);
                       end
                     else
                       list.concat(taicpu.op_reg_reg(A_SXTB,reg2,reg1));
                   end;
                 OS_16:
                   list.concat(taicpu.op_reg_reg(A_UXTH,reg2,reg1));
                 OS_S16:
                   list.concat(taicpu.op_reg_reg(A_SXTH,reg2,reg1));
                 else
                   conv_done:=false;
               end

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


    procedure tbasecgarm.a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);
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


     procedure tbasecgarm.a_loadfpu_reg_reg(list: TAsmList; fromsize,tosize: tcgsize; reg1, reg2: tregister);
       begin
         list.concat(setoppostfix(taicpu.op_reg_reg(A_MVF,reg2,reg1),cgsize2fpuoppostfix[tosize]));
       end;


     procedure tbasecgarm.a_loadfpu_ref_reg(list: TAsmList; fromsize,tosize: tcgsize; const ref: treference; reg: tregister);
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


     procedure tbasecgarm.a_loadfpu_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference);
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
    procedure tbasecgarm.a_cmp_const_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;a : tcgint;reg : tregister;
      l : tasmlabel);
      var
        tmpreg : tregister;
        b : byte;
      begin
        a_reg_alloc(list,NR_DEFAULTFLAGS);
        if (not(GenerateThumbCode) and is_shifter_const(a,b)) or
          ((GenerateThumbCode) and is_thumb_imm(a)) then
          list.concat(taicpu.op_reg_const(A_CMP,reg,a))
        { CMN reg,0 and CMN reg,$80000000 are different from CMP reg,$ffffffff
          and CMP reg,$7fffffff regarding the flags according to the ARM manual }
        else if (a<>$7fffffff) and (a<>-1) and not(GenerateThumbCode) and is_shifter_const(-a,b) then
          list.concat(taicpu.op_reg_const(A_CMN,reg,-a))
        else
          begin
            tmpreg:=getintregister(list,size);
            a_load_const_reg(list,size,a,tmpreg);
            list.concat(taicpu.op_reg_reg(A_CMP,reg,tmpreg));
          end;
        a_jmp_cond(list,cmp_op,l);
        a_reg_dealloc(list,NR_DEFAULTFLAGS);
      end;


    procedure tbasecgarm.a_bit_scan_reg_reg(list: TAsmList; reverse: boolean; srcsize, dstsize: TCGSize; src, dst: TRegister);
      begin
        if reverse then
          begin
            list.Concat(taicpu.op_reg_reg(A_CLZ,dst,src));
            list.Concat(taicpu.op_reg_reg_const(A_RSB,dst,dst,31));
            list.Concat(taicpu.op_reg_reg_const(A_AND,dst,dst,255));
          end
        { it is decided during the compilation of the system unit if this code is used or not
          so no additional check for rbit is needed                                           }
        else
          begin
            list.Concat(taicpu.op_reg_reg(A_RBIT,dst,src));
            list.Concat(taicpu.op_reg_reg(A_CLZ,dst,dst));
            a_reg_alloc(list,NR_DEFAULTFLAGS);
            list.Concat(taicpu.op_reg_const(A_CMP,dst,32));
            if GenerateThumb2Code then
              list.Concat(taicpu.op_cond(A_IT, C_EQ));
            list.Concat(setcondition(taicpu.op_reg_const(A_MOV,dst,$ff),C_EQ));
            a_reg_dealloc(list,NR_DEFAULTFLAGS);
          end;
      end;


    procedure tbasecgarm.a_cmp_reg_reg_label(list : TAsmList;size : tcgsize;cmp_op : topcmp;reg1,reg2 : tregister;l : tasmlabel);
      begin
        a_reg_alloc(list,NR_DEFAULTFLAGS);
        list.concat(taicpu.op_reg_reg(A_CMP,reg2,reg1));
        a_jmp_cond(list,cmp_op,l);
        a_reg_dealloc(list,NR_DEFAULTFLAGS);
      end;


    procedure tbasecgarm.a_jmp_name(list : TAsmList;const s : string);
      var
        ai : taicpu;
      begin
        { generate far jump, leave it to the optimizer to get rid of it }
        if GenerateThumbCode then
          ai:=taicpu.op_sym(A_BL,current_asmdata.RefAsmSymbol(s))
        else
          ai:=taicpu.op_sym(A_B,current_asmdata.RefAsmSymbol(s));
        ai.is_jmp:=true;
        list.concat(ai);
      end;


    procedure tbasecgarm.a_jmp_always(list : TAsmList;l: tasmlabel);
      var
        ai : taicpu;
      begin
        { generate far jump, leave it to the optimizer to get rid of it }
        if GenerateThumbCode then
          ai:=taicpu.op_sym(A_BL,l)
        else
          ai:=taicpu.op_sym(A_B,l);
        ai.is_jmp:=true;
        list.concat(ai);
      end;


    procedure tbasecgarm.a_jmp_flags(list : TAsmList;const f : TResFlags;l: tasmlabel);
      var
        ai : taicpu;
        inv_flags : TResFlags;
        hlabel : TAsmLabel;
      begin
        if GenerateThumbCode then
          begin
            inv_flags:=f;
            inverse_flags(inv_flags);
            { the optimizer has to fix this if jump range is sufficient short }
            current_asmdata.getjumplabel(hlabel);
            ai:=setcondition(taicpu.op_sym(A_B,hlabel),flags_to_cond(inv_flags));
            ai.is_jmp:=true;
            list.concat(ai);
            a_jmp_always(list,l);
            a_label(list,hlabel);
          end
        else
          begin
            ai:=setcondition(taicpu.op_sym(A_B,l),flags_to_cond(f));
            ai.is_jmp:=true;
            list.concat(ai);
          end;
      end;


    procedure tbasecgarm.g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister);
      begin
        list.concat(setcondition(taicpu.op_reg_const(A_MOV,reg,1),flags_to_cond(f)));
        list.concat(setcondition(taicpu.op_reg_const(A_MOV,reg,0),inverse_cond(flags_to_cond(f))));
      end;

    procedure tbasecgarm.g_profilecode(list : TAsmList);
      begin
        if target_info.system = system_arm_linux then
          begin
            list.concat(taicpu.op_regset(A_PUSH,R_INTREGISTER,R_SUBWHOLE,[RS_R14]));
            a_call_name(list,'__gnu_mcount_nc',false);
          end
        else
          internalerror(2014091201);
      end;

    procedure tbasecgarm.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
      var
         ref : treference;
         shift : byte;
         firstfloatreg,lastfloatreg,
         r : byte;
         mmregs,
         regs, saveregs : tcpuregisterset;
         registerarea,
         r7offset,
         stackmisalignment : pint;
         imm1, imm2: DWord;
         stack_parameters : Boolean;
      begin
        LocalSize:=align(LocalSize,4);
        stack_parameters:=current_procinfo.procdef.stack_tainting_parameter(calleeside);

        { call instruction does not put anything on the stack }
        registerarea:=0;
        tarmprocinfo(current_procinfo).stackpaddingreg:=High(TSuperRegister);
        lastfloatreg:=RS_NO;
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
                        inc(registerarea,12);
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
                         inc(registerarea,4);

                     { if the stack is not 8 byte aligned, try to add an extra register,
                       so we can avoid the extra sub/add ...,#4 later (KB) }
                     if ((registerarea mod current_settings.alignment.localalignmax) <> 0) then
                       for r:=RS_R3 downto RS_R0 do
                         if not(r in regs) then
                           begin
                             regs:=regs+[r];
                             inc(registerarea,4);
                             tarmprocinfo(current_procinfo).stackpaddingreg:=r;
                             break;
                           end;
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
                          inc(registerarea,4);
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
                            inc(registerarea,4);
                        list.concat(setoppostfix(taicpu.op_ref_regset(A_STM,ref,R_INTREGISTER,R_SUBWHOLE,saveregs),PF_FD));
                      end;
                  end;
              end;

            stackmisalignment:=registerarea mod current_settings.alignment.localalignmax;
            if (LocalSize<>0) or
               ((stackmisalignment<>0) and
                ((pi_do_call in current_procinfo.flags) or
                 (po_assembler in current_procinfo.procdef.procoptions))) then
              begin
                localsize:=align(localsize+stackmisalignment,current_settings.alignment.localalignmax)-stackmisalignment;
                if stack_parameters and (pi_estimatestacksize in current_procinfo.flags) then
                  begin
                    if localsize>tarmprocinfo(current_procinfo).stackframesize then
                      internalerror(2014030901)
                    else
                      localsize:=tarmprocinfo(current_procinfo).stackframesize-registerarea;
                  end;
                if is_shifter_const(localsize,shift) then
                  begin
                    a_reg_dealloc(list,NR_R12);
                    list.concat(taicpu.op_reg_reg_const(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize));
                  end
                else if split_into_shifter_const(localsize, imm1, imm2) then
                  begin
                    a_reg_dealloc(list,NR_R12);
                    list.concat(taicpu.op_reg_reg_const(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,imm1));
                    list.concat(taicpu.op_reg_reg_const(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,imm2));
                  end
                else
                  begin
                    if current_procinfo.framepointer=NR_STACK_POINTER_REG then
                      a_reg_alloc(list,NR_R12);
                    a_load_const_reg(list,OS_ADDR,LocalSize,NR_R12);
                    list.concat(taicpu.op_reg_reg_reg(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_R12));
                    a_reg_dealloc(list,NR_R12);
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
                     {if (current_settings.cputype<cpu_armv6) then
                       postfix:=PF_IAX
                     else
                       postfix:=PF_IAD;}
                     list.concat(taicpu.op_ref_regset(A_VSTM,ref,R_MMREGISTER,R_SUBFD,mmregs));
                   end;
               end;
             end;
        end;
      end;


    procedure tbasecgarm.g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean);
      var
         ref : treference;
         LocalSize : longint;
         firstfloatreg,lastfloatreg,
         r,
         shift : byte;
         mmregs,
         saveregs,
         regs : tcpuregisterset;
         registerarea,
         stackmisalignment: pint;
         paddingreg: TSuperRegister;
         imm1, imm2: DWord;
      begin
        if not(nostackframe) then
          begin
            registerarea:=0;
            firstfloatreg:=RS_NO;
            lastfloatreg:=RS_NO;
            mmregs:=[];
            saveregs:=[];
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
                         inc(registerarea,12);
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
                      {if (current_settings.cputype<cpu_armv6) then
                        mmpostfix:=PF_IAX
                      else
                        mmpostfix:=PF_IAD;}
                      list.concat(taicpu.op_ref_regset(A_VLDM,ref,R_MMREGISTER,R_SUBFD,mmregs));
                    end;
                end;
              end;

            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);
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
                        inc(registerarea,4);
                    regs:=regs-saveregs;
                  end;
              end;
            for r:=RS_R0 to RS_R15 do
              if r in regs then
                inc(registerarea,4);

            { reapply the stack padding reg, in case there was one, see the complimentary
              comment in g_proc_entry() (KB) }
            paddingreg:=tarmprocinfo(current_procinfo).stackpaddingreg;
            if paddingreg < RS_R4 then
              if paddingreg in regs then
                internalerror(201306190)
              else
                begin
                  regs:=regs+[paddingreg];
                  inc(registerarea,4);
                end;
            stackmisalignment:=registerarea mod current_settings.alignment.localalignmax;
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) or
               (target_info.system in systems_darwin) then
              begin
                LocalSize:=current_procinfo.calc_stackframe_size;
                if (LocalSize<>0) or
                   ((stackmisalignment<>0) and
                    ((pi_do_call in current_procinfo.flags) or
                     (po_assembler in current_procinfo.procdef.procoptions))) then
                  begin
                    if pi_estimatestacksize in current_procinfo.flags then
                      LocalSize:=tarmprocinfo(current_procinfo).stackframesize-registerarea
                    else
                      localsize:=align(localsize+stackmisalignment,current_settings.alignment.localalignmax)-stackmisalignment;

                    if is_shifter_const(LocalSize,shift) then
                      list.concat(taicpu.op_reg_reg_const(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize))
                    else if split_into_shifter_const(localsize, imm1, imm2) then
                      begin
                        list.concat(taicpu.op_reg_reg_const(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,imm1));
                        list.concat(taicpu.op_reg_reg_const(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,imm2));
                      end
                    else
                      begin
                        a_reg_alloc(list,NR_R12);
                        a_load_const_reg(list,OS_ADDR,LocalSize,NR_R12);
                        list.concat(taicpu.op_reg_reg_reg(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_R12));
                        a_reg_dealloc(list,NR_R12);
                      end;
                  end;

                if (target_info.system in systems_darwin) and
                   (saveregs<>[]) then
                  list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,saveregs),PF_FD));

                if regs=[] then
                  begin
                    if not(CPUARM_HAS_BX in cpu_capabilities[current_settings.cputype]) then
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
        else if not(CPUARM_HAS_BX in cpu_capabilities[current_settings.cputype]) then
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14))
        else
          list.concat(taicpu.op_reg(A_BX,NR_R14))
      end;


    procedure tbasecgarm.g_maybe_got_init(list : TAsmList);
      var
        ref : treference;
        l : TAsmLabel;
        regs : tcpuregisterset;
        r: byte;
      begin
        if (cs_create_pic in current_settings.moduleswitches) and
           (pi_needs_got in current_procinfo.flags) and
           (tf_pic_uses_got in target_info.flags) then
          begin
            { Procedure parametrs are not initialized at this stage.
              Before GOT initialization code, allocate registers used for procedure parameters
              to prevent usage of these registers for temp operations in later stages of code
              generation. }
            regs:=rg[R_INTREGISTER].used_in_proc;
            for r:=RS_R0 to RS_R3 do
              if r in regs then
                a_reg_alloc(list, newreg(R_INTREGISTER,r,R_SUBWHOLE));
            { Allocate scratch register R12 and use it for GOT calculations directly.
              Otherwise the init code can be distorted in later stages of code generation. }
            a_reg_alloc(list,NR_R12);

            reference_reset(ref,4);
            current_asmdata.getglobaldatalabel(l);
            cg.a_label(current_procinfo.aktlocaldata,l);
            ref.symbol:=l;
            ref.base:=NR_PC;
            ref.symboldata:=current_procinfo.aktlocaldata.last;
            list.concat(Taicpu.op_reg_ref(A_LDR,NR_R12,ref));
            current_asmdata.getaddrlabel(l);
            current_procinfo.aktlocaldata.concat(tai_const.Create_rel_sym_offset(aitconst_32bit,l,current_asmdata.RefAsmSymbol('_GLOBAL_OFFSET_TABLE_'),-8));
            cg.a_label(list,l);
            list.concat(Taicpu.op_reg_reg_reg(A_ADD,NR_R12,NR_PC,NR_R12));
            list.concat(Taicpu.op_reg_reg(A_MOV,current_procinfo.got,NR_R12));

            { Deallocate registers }
            a_reg_dealloc(list,NR_R12);
            for r:=RS_R3 downto RS_R0 do
              if r in regs then
                a_reg_dealloc(list, newreg(R_INTREGISTER,r,R_SUBWHOLE));
          end;
      end;


    procedure tbasecgarm.a_loadaddr_ref_reg(list : TAsmList;const ref : treference;r : tregister);
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
              internalerror(2014020702);
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


    procedure tbasecgarm.fixref(list : TAsmList;var ref : treference);
      var
        tmpreg, tmpreg2 : tregister;
        tmpref : treference;
        l, piclabel : tasmlabel;
        indirection_done : boolean;
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
        piclabel:=nil;
        tmpreg:=NR_NO;

        indirection_done:=false;
        if assigned(ref.symbol) then
          begin
            if (target_info.system=system_arm_darwin) and
               (ref.symbol.bind in [AB_EXTERNAL,AB_WEAK_EXTERNAL,AB_PRIVATE_EXTERN,AB_COMMON]) then
              begin
                tmpreg:=g_indirect_sym_load(list,ref.symbol.name,asmsym2indsymflags(ref.symbol));
                if ref.offset<>0 then
                    a_op_const_reg(list,OP_ADD,OS_ADDR,ref.offset,tmpreg);
                indirection_done:=true;
              end
            else if (cs_create_pic in current_settings.moduleswitches) then
              if (tf_pic_uses_got in target_info.flags) then
                current_procinfo.aktlocaldata.concat(tai_const.Create_type_sym(aitconst_got,ref.symbol))
              else
                begin
                  { ideally, we would want to generate

                      ldr       r1, LPICConstPool
                    LPICLocal:
                      ldr/str   r2,[pc,r1]

                    ...
                      LPICConstPool:
                        .long _globsym-(LPICLocal+8)

                    However, we cannot be sure that the ldr/str will follow
                    right after the call to fixref, so we have to load the
                    complete address already in a register.
                  }
                  current_asmdata.getaddrlabel(piclabel);
                  current_procinfo.aktlocaldata.concat(tai_const.Create_rel_sym_offset(aitconst_ptr,piclabel,ref.symbol,ref.offset-8));
                end
            else
              current_procinfo.aktlocaldata.concat(tai_const.create_sym_offset(ref.symbol,ref.offset))
          end
        else
            current_procinfo.aktlocaldata.concat(tai_const.Create_32bit(ref.offset));

        { load consts entry }
        if not indirection_done then
          begin
            tmpreg:=getintregister(list,OS_INT);
            tmpref.symbol:=l;
            tmpref.base:=NR_PC;
            list.concat(taicpu.op_reg_ref(A_LDR,tmpreg,tmpref));

            if (cs_create_pic in current_settings.moduleswitches) and
               (tf_pic_uses_got in target_info.flags) and
               assigned(ref.symbol) then
              begin
                reference_reset(tmpref,4);
                tmpref.base:=current_procinfo.got;
                tmpref.index:=tmpreg;
                list.concat(taicpu.op_reg_ref(A_LDR,tmpreg,tmpref));
                if ref.offset<>0 then
                  a_op_const_reg(list,OP_ADD,OS_ADDR,ref.offset,tmpreg);
              end;
          end;

        if assigned(piclabel) then
          begin
            cg.a_label(list,piclabel);
            tmpreg2:=getaddressregister(list);
            a_op_reg_reg_reg(list,OP_ADD,OS_ADDR,tmpreg,NR_PC,tmpreg2);
            tmpreg:=tmpreg2
          end;

        { This routine can be called with PC as base/index in case the offset
          was too large to encode in a load/store. In that case, the entire
          absolute expression has been re-encoded in a new constpool entry, and
          we have to remove the use of PC from the original reference (the code
          above made everything relative to the value loaded from the new
          constpool entry) }
        if is_pc(ref.base) then
          ref.base:=NR_NO;
        if is_pc(ref.index) then
          ref.index:=NR_NO;

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


    procedure tbasecgarm.g_concatcopy_move(list : TAsmList;const source,dest : treference;len : tcgint);
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
        alloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
        a_call_name(list,'FPC_MOVE',false);
        dealloccpuregisters(list,R_FPUREGISTER,paramanager.get_volatile_registers_fpu(pocall_default));
        dealloccpuregisters(list,R_INTREGISTER,paramanager.get_volatile_registers_int(pocall_default));
        paraloc3.done;
        paraloc2.done;
        paraloc1.done;
      end;


    procedure tbasecgarm.g_concatcopy_internal(list : TAsmList;const source,dest : treference;len : tcgint;aligned : boolean);
      const
        maxtmpreg_arm = 10; {roozbeh: can be reduced to 8 or lower if might conflick with reserved ones,also +2 is used becouse of regs required for referencing}
        maxtmpreg_thumb = 5;

      var
        srcref,dstref,usedtmpref,usedtmpref2:treference;
        srcreg,destreg,countreg,r,tmpreg:tregister;
        helpsize:aint;
        copysize:byte;
        cgsize:Tcgsize;
        tmpregisters:array[1..maxtmpreg_arm] of tregister;
        maxtmpreg,
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
          a_reg_alloc(list,NR_DEFAULTFLAGS);
          list.concat(setoppostfix(taicpu.op_reg_reg_const(A_SUB,countreg,countreg,1),PF_S));
          a_load_reg_ref(list,size2opsize[size],size2opsize[size],r,dstref);
          a_jmp_flags(list,F_NE,l);
          a_reg_dealloc(list,NR_DEFAULTFLAGS);
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

      { will never be called with count<=4 }
      procedure genloop_thumb(count : aword;size : byte);

        procedure refincofs(const ref : treference;const value : longint = 1);
          begin
            a_op_const_reg(list,OP_ADD,OS_ADDR,value,ref.base);
          end;

        const
          size2opsize : array[1..4] of tcgsize = (OS_8,OS_16,OS_NO,OS_32);
        var
          l : tasmlabel;
        begin
          current_asmdata.getjumplabel(l);
          if count<size then size:=1;
          a_load_const_reg(list,OS_INT,count div size,countreg);
          cg.a_label(list,l);
          r:=getintregister(list,size2opsize[size]);
          a_load_ref_reg(list,size2opsize[size],size2opsize[size],srcref,r);
          refincofs(srcref);
          a_load_reg_ref(list,size2opsize[size],size2opsize[size],r,dstref);
          refincofs(dstref);
          a_reg_alloc(list,NR_DEFAULTFLAGS);
          list.concat(taicpu.op_reg_reg_const(A_SUB,countreg,countreg,1));
          a_jmp_flags(list,F_NE,l);
          a_reg_dealloc(list,NR_DEFAULTFLAGS);
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
                  refincofs(srcref);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                  refincofs(dstref);
                  a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                end;
            3:
              if aligned then
                begin
                  a_load_ref_reg(list,OS_16,OS_16,srcref,r);
                  refincofs(srcref,2);
                  a_load_reg_ref(list,OS_16,OS_16,r,dstref);
                  refincofs(dstref,2);
                  a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                end
              else
                begin
                  a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                  refincofs(srcref);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                  refincofs(dstref);
                  a_load_ref_reg(list,OS_8,OS_8,srcref,r);
                  refincofs(srcref);
                  a_load_reg_ref(list,OS_8,OS_8,r,dstref);
                  refincofs(dstref);
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
        if GenerateThumbCode then
          maxtmpreg:=maxtmpreg_thumb
        else
          maxtmpreg:=maxtmpreg_arm;

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
        else if aligned and (len=2) then
          begin
            tmpreg:=getintregister(list,OS_16);
            a_load_ref_reg(list,OS_16,OS_16,source,tmpreg);
            a_load_reg_ref(list,OS_16,OS_16,tmpreg,dest);
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
                if GenerateThumbCode then
                  genloop_thumb(len,1)
                else
                  genloop(len,1);
            end;
          end;
    end;

    procedure tbasecgarm.g_concatcopy_unaligned(list : TAsmList;const source,dest : treference;len : tcgint);
      begin
        g_concatcopy_internal(list,source,dest,len,false);
      end;


    procedure tbasecgarm.g_concatcopy(list : TAsmList;const source,dest : treference;len : tcgint);
      begin
        if (source.alignment in [1,3]) or
          (dest.alignment in [1,3]) then
          g_concatcopy_internal(list,source,dest,len,false)
        else
          g_concatcopy_internal(list,source,dest,len,true);
      end;


    procedure tbasecgarm.g_overflowCheck(list : TAsmList;const l : tlocation;def : tdef);
      var
        ovloc : tlocation;
      begin
        ovloc.loc:=LOC_VOID;
        g_overflowCheck_loc(list,l,def,ovloc);
      end;


    procedure tbasecgarm.g_overflowCheck_loc(List:TAsmList;const Loc:TLocation;def:TDef;ovloc : tlocation);
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
              cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
            end;
          else
            internalerror(200409281);
        end;

        a_call_name(list,'FPC_OVERFLOW',false);
        a_label(list,hl);
      end;


    procedure tbasecgarm.g_save_registers(list : TAsmList);
      begin
        { this work is done in g_proc_entry }
      end;


    procedure tbasecgarm.g_restore_registers(list : TAsmList);
      begin
        { this work is done in g_proc_exit }
      end;


    procedure tbasecgarm.a_jmp_cond(list : TAsmList;cond : TOpCmp;l: tasmlabel);
      var
        ai : taicpu;
        hlabel : TAsmLabel;
      begin
        if GenerateThumbCode then
          begin
            { the optimizer has to fix this if jump range is sufficient short }
            current_asmdata.getjumplabel(hlabel);
            ai:=Taicpu.Op_sym(A_B,hlabel);
            ai.SetCondition(inverse_cond(OpCmp2AsmCond[cond]));
            ai.is_jmp:=true;
            list.concat(ai);
            a_jmp_always(list,l);
            a_label(list,hlabel);
          end
        else
          begin
            ai:=Taicpu.Op_sym(A_B,l);
            ai.SetCondition(OpCmp2AsmCond[cond]);
            ai.is_jmp:=true;
            list.concat(ai);
          end;
      end;


    function get_scalar_mm_op(fromsize,tosize : tcgsize) : tasmop;
      const
        convertop : array[OS_F32..OS_F128,OS_F32..OS_F128] of tasmop = (
          (A_VMOV,A_VCVT,A_NONE,A_NONE,A_NONE),
          (A_VCVT,A_VMOV,A_NONE,A_NONE,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_NONE,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_NONE,A_NONE),
          (A_NONE,A_NONE,A_NONE,A_NONE,A_NONE));
      begin
        result:=convertop[fromsize,tosize];
        if result=A_NONE then
          internalerror(200312205);
      end;


    function get_scalar_mm_prefix(fromsize,tosize : tcgsize) : TOpPostfix;
      const
        convertop : array[OS_F32..OS_F128,OS_F32..OS_F128] of TOpPostfix = (
          (PF_F32,   PF_F32F64,PF_None,PF_None,PF_None),
          (PF_F64F32,PF_F64,   PF_None,PF_None,PF_None),
          (PF_None,  PF_None,  PF_None,PF_None,PF_None),
          (PF_None,  PF_None,  PF_None,PF_None,PF_None),
          (PF_None,  PF_None,  PF_None,PF_None,PF_None));
      begin
        result:=convertop[fromsize,tosize];
      end;


    procedure tbasecgarm.a_loadmm_reg_reg(list: tasmlist; fromsize,tosize: tcgsize; reg1,reg2: tregister; shuffle: pmmshuffle);
      var
        instr: taicpu;
      begin
        if (shuffle=nil) or shufflescalar(shuffle) then
          instr:=setoppostfix(taicpu.op_reg_reg(get_scalar_mm_op(tosize,fromsize),reg2,reg1),get_scalar_mm_prefix(tosize,fromsize))
        else
          internalerror(2009112407);
        list.concat(instr);
        case instr.opcode of
          A_VMOV:
            add_move_instruction(instr);
        end;
      end;


    procedure tbasecgarm.a_loadmm_ref_reg(list: tasmlist; fromsize,tosize: tcgsize; const ref: treference; reg: tregister; shuffle: pmmshuffle);
      var
        intreg,
        tmpmmreg : tregister;
        reg64    : tregister64;
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
             handle_load_store(list,A_VLDR,PF_None,tmpmmreg,ref);
          end;

        if (tmpmmreg<>reg) then
          a_loadmm_reg_reg(list,fromsize,tosize,tmpmmreg,reg,shuffle);
      end;


    procedure tbasecgarm.a_loadmm_reg_ref(list: tasmlist; fromsize,tosize: tcgsize; reg: tregister; const ref: treference; shuffle: pmmshuffle);
      var
        intreg,
        tmpmmreg : tregister;
        reg64    : tregister64;
      begin
        if assigned(shuffle) and
           not(shufflescalar(shuffle)) then
          internalerror(2009112416);

<<<<<<< HEAD
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
=======
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
>>>>>>> graemeg/cpstrnew
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
             handle_load_store(list,A_VSTR,PF_None,tmpmmreg,ref);
          end;
      end;


    procedure tbasecgarm.a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize : tcgsize; intreg, mmreg: tregister; shuffle: pmmshuffle);
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
        list.concat(taicpu.op_reg_reg(A_VMOV,mmreg,intreg));
      end;


    procedure tbasecgarm.a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize : tcgsize; mmreg, intreg: tregister;shuffle : pmmshuffle);
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
        list.concat(taicpu.op_reg_reg(A_VMOV,intreg,mmreg));
      end;


      procedure tbasecgarm.a_opmm_reg_reg(list: tasmlist; op: topcg; size: tcgsize; src, dst: tregister; shuffle: pmmshuffle);
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
                    list.concat(taicpu.op_reg_reg(A_VMOV,dst,tmpreg));
                  OS_F64:
                    list.concat(taicpu.op_reg_reg_reg(A_VMOV,dst,tmpreg,tmpreg));
                  else
                    internalerror(2009112908);
                end;
              end
            else
              internalerror(2009112906);
          end;
        end;


    procedure tbasecgarm.maybeadjustresult(list: TAsmList; op: TOpCg; size: tcgsize; dst: tregister);
      const
        overflowops = [OP_MUL,OP_SHL,OP_ADD,OP_SUB,OP_NEG];
      begin
        if (op in overflowops) and
           (size in [OS_8,OS_S8,OS_16,OS_S16]) then
          a_load_reg_reg(list,OS_32,size,dst,dst);
      end;


    procedure tbasecgarm.safe_mla(list : TAsmList; op1,op2,op3,op4 : TRegister);

      procedure checkreg(var reg : TRegister);
        var
          tmpreg : TRegister;
        begin
          if ((GenerateThumbCode or GenerateThumb2Code) and (getsupreg(reg)=RS_R13)) or
            (getsupreg(reg)=RS_R15) then
            begin
              tmpreg:=getintregister(list,OS_INT);
              a_load_reg_reg(list,OS_INT,OS_INT,reg,tmpreg);
              reg:=tmpreg;
            end;
        end;

      begin
        checkreg(op1);
        checkreg(op2);
        checkreg(op3);
        checkreg(op4);
        list.concat(taicpu.op_reg_reg_reg_reg(A_MLA,op1,op2,op3,op4));
      end;


    procedure tcg64farm.a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);
      begin
        case op of
          OP_NEG:
            begin
              cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(setoppostfix(taicpu.op_reg_reg_const(A_RSB,regdst.reglo,regsrc.reglo,0),PF_S));
              list.concat(taicpu.op_reg_reg_const(A_RSC,regdst.reghi,regsrc.reghi,0));
              cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
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
        list.concat(taicpu.op_reg_reg_reg(A_VMOV,mmreg,intreg.reglo,intreg.reghi));
      end;


    procedure tcg64farm.a_loadmm_reg_intreg64(list: TAsmList; mmsize: tcgsize; mmreg: tregister; intreg: tregister64);
      begin
        { this code can only be used to transfer raw data, not to perform
          conversions }
        if (mmsize<>OS_F64) then
          internalerror(2009112406);
        list.concat(taicpu.op_reg_reg_reg(A_VMOV,intreg.reglo,intreg.reghi,mmreg));
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
            internalerror(2012022501);
        end;
        if (setflags or tbasecgarm(cg).cgsetflags) and (op in [OP_ADD,OP_SUB]) then
          begin
            case op of
              OP_ADD:
                begin
                  if is_shifter_const(lo(value),b) then
                    begin
                      cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                      list.concat(setoppostfix(taicpu.op_reg_reg_const(A_ADD,regdst.reglo,regsrc.reglo,lo(value)),PF_S))
                    end
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,lo(value),tmpreg);
                      cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
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
                    begin
                      cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                      list.concat(setoppostfix(taicpu.op_reg_reg_const(A_SUB,regdst.reglo,regsrc.reglo,lo(value)),PF_S))
                    end
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,lo(value),tmpreg);
                      cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
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
                    begin
                      cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                      list.concat(setoppostfix(taicpu.op_reg_reg_const(A_ADD,regdst.reglo,regsrc.reglo,aint(lo(value))),PF_S))
                    end
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,aint(lo(value)),tmpreg);
                      cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
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
                    begin
                      cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                      list.concat(setoppostfix(taicpu.op_reg_reg_const(A_SUB,regdst.reglo,regsrc.reglo,aint(lo(value))),PF_S))
                    end
                  else
                    begin
                      tmpreg:=cg.getintregister(list,OS_32);
                      cg.a_load_const_reg(list,OS_32,aint(lo(value)),tmpreg);
                      cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
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


<<<<<<< HEAD
    procedure tcg64farm.a_op64_reg_reg_reg_checkoverflow(list: TAsmList;op:TOpCG;size : tcgsize;regsrc1,regsrc2,regdst : tregister64;setflags : boolean;var ovloc : tlocation);
      begin
        ovloc.loc:=LOC_VOID;
        case op of
          OP_NEG,
          OP_NOT :
            internalerror(2012022502);
        end;
        if (setflags or tbasecgarm(cg).cgsetflags) and (op in [OP_ADD,OP_SUB]) then
          begin
            case op of
              OP_ADD:
                begin
                  cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ADD,regdst.reglo,regsrc1.reglo,regsrc2.reglo),PF_S));
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ADC,regdst.reghi,regsrc1.reghi,regsrc2.reghi),PF_S));
                end;
              OP_SUB:
                begin
                  cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
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
                  cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_ADD,regdst.reglo,regsrc1.reglo,regsrc2.reglo),PF_S));
                  list.concat(taicpu.op_reg_reg_reg(A_ADC,regdst.reghi,regsrc1.reghi,regsrc2.reghi));
                  cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
                end;
              OP_SUB:
                begin
                  cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                  list.concat(setoppostfix(taicpu.op_reg_reg_reg(A_SUB,regdst.reglo,regsrc2.reglo,regsrc1.reglo),PF_S));
                  list.concat(taicpu.op_reg_reg_reg(A_SBC,regdst.reghi,regsrc2.reghi,regsrc1.reghi));
                  cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
                end;
              else
                internalerror(2003083101);
            end;
          end;
      end;


    procedure tthumbcgarm.init_register_allocators;
=======
    function tcgarm.handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference;
      var
        tmpreg : tregister;
        tmpref : treference;
        l : tasmlabel;
>>>>>>> graemeg/cpstrnew
      begin
        inherited init_register_allocators;
        if assigned(current_procinfo) and (current_procinfo.framepointer=NR_R7) then
          rg[R_INTREGISTER]:=trgintcputhumb.create(R_INTREGISTER,R_SUBWHOLE,
              [RS_R0,RS_R1,RS_R2,RS_R3,RS_R4,RS_R5,RS_R6],first_int_imreg,[])
        else
          rg[R_INTREGISTER]:=trgintcputhumb.create(R_INTREGISTER,R_SUBWHOLE,
              [RS_R0,RS_R1,RS_R2,RS_R3,RS_R4,RS_R5,RS_R6,RS_R7],first_int_imreg,[]);
      end;


    procedure tthumbcgarm.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_MMREGISTER].free;
        inherited done_register_allocators;
      end;


<<<<<<< HEAD
    procedure tthumbcgarm.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
      var
         ref : treference;
         r : byte;
         regs : tcpuregisterset;
         stackmisalignment : pint;
         registerarea: DWord;
         stack_parameters: Boolean;
      begin
        stack_parameters:=current_procinfo.procdef.stack_tainting_parameter(calleeside);
        LocalSize:=align(LocalSize,4);
        { call instruction does not put anything on the stack }
        stackmisalignment:=0;
        if not(nostackframe) then
=======
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
             { the usual pc relative symbol handling assumes possible offsets of +/- 4095 }
             assigned(ref.symbol)
            )
           ) then
>>>>>>> graemeg/cpstrnew
          begin
            a_reg_alloc(list,NR_STACK_POINTER_REG);
            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              a_reg_alloc(list,NR_FRAME_POINTER_REG);
            { save int registers }
            reference_reset(ref,4);
            ref.index:=NR_STACK_POINTER_REG;
            ref.addressmode:=AM_PREINDEXED;
            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              begin
                //!!!! a_reg_alloc(list,NR_R12);
                //!!!! list.concat(taicpu.op_reg_reg(A_MOV,NR_R12,NR_STACK_POINTER_REG));
              end;
            { the (old) ARM APCS requires saving both the stack pointer (to
              crawl the stack) and the PC (to identify the function this
              stack frame belongs to) -> also save R12 (= copy of R13 on entry)
              and R15 -- still needs updating for EABI and Darwin, they don't
              need that }
            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              regs:=regs+[RS_R7,RS_R14]
            else
              // if (regs<>[]) or (pi_do_call in current_procinfo.flags) then
              include(regs,RS_R14);

            { safely estimate stack size }
            if localsize+current_settings.alignment.localalignmax+4>508 then
              begin
                include(rg[R_INTREGISTER].used_in_proc,RS_R4);
                include(regs,RS_R4);
              end;

            registerarea:=0;
            if regs<>[] then
               begin
                 for r:=RS_R0 to RS_R15 do
                   if r in regs then
                     inc(registerarea,4);
                 list.concat(taicpu.op_regset(A_PUSH,R_INTREGISTER,R_SUBWHOLE,regs));
               end;

            stackmisalignment:=registerarea mod current_settings.alignment.localalignmax;

            if stack_parameters or (LocalSize<>0) or
               ((stackmisalignment<>0) and
                ((pi_do_call in current_procinfo.flags) or
                 (po_assembler in current_procinfo.procdef.procoptions))) then
              begin
                { do we access stack parameters?
                  if yes, the previously estimated stacksize must be used }
                if stack_parameters then
                  begin
                    if localsize>tarmprocinfo(current_procinfo).stackframesize then
                      begin
                        writeln(localsize);
                        writeln(tarmprocinfo(current_procinfo).stackframesize);
                        internalerror(2013040601);
                      end
                    else
                      localsize:=tarmprocinfo(current_procinfo).stackframesize-registerarea;
                  end
                else
                  localsize:=align(localsize+stackmisalignment,current_settings.alignment.localalignmax)-stackmisalignment;

<<<<<<< HEAD
                if localsize<508 then
=======
        { floating point operations have only limited references
          we expect here, that a base is already set }
        if (op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) and (ref.index<>NR_NO) then
          begin
            if ref.shiftmode<>SM_none then
              internalerror(200309121);
            if tmpreg<>NR_NO then
              begin
                if ref.base=tmpreg then
>>>>>>> graemeg/cpstrnew
                  begin
                    list.concat(taicpu.op_reg_reg_const(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize));
                  end
                else if localsize<=1016 then
                  begin
                    list.concat(taicpu.op_reg_reg_const(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,508));
                    list.concat(taicpu.op_reg_reg_const(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize-508));
                  end
                else
                  begin
                    a_load_const_reg(list,OS_ADDR,-localsize,NR_R4);
                    list.concat(taicpu.op_reg_reg_reg(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_R4));
                    include(regs,RS_R4);

                    //!!!! if current_procinfo.framepointer=NR_STACK_POINTER_REG then
                    //!!!!   a_reg_alloc(list,NR_R12);
                    //!!!! a_load_const_reg(list,OS_ADDR,LocalSize,NR_R12);
                    //!!!! list.concat(taicpu.op_reg_reg_reg(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_R12));
                    //!!!! a_reg_dealloc(list,NR_R12);
                  end;
<<<<<<< HEAD
              end;

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              begin
                list.concat(taicpu.op_reg_reg_const(A_ADD,current_procinfo.framepointer,NR_STACK_POINTER_REG,0));
              end;
          end;
      end;


    procedure tthumbcgarm.g_proc_exit(list: TAsmList; parasize: longint; nostackframe: boolean);
      var
         LocalSize : longint;
         r: byte;
         regs : tcpuregisterset;
         registerarea : DWord;
         stackmisalignment: pint;
         stack_parameters : Boolean;
      begin
        if not(nostackframe) then
          begin
            stack_parameters:=current_procinfo.procdef.stack_tainting_parameter(calleeside);
            regs:=rg[R_INTREGISTER].used_in_proc-paramanager.get_volatile_registers_int(pocall_stdcall);

            include(regs,RS_R15);

            if current_procinfo.framepointer<>NR_STACK_POINTER_REG then
              include(regs,getsupreg(current_procinfo.framepointer));

            registerarea:=0;
            for r:=RS_R0 to RS_R15 do
              if r in regs then
                inc(registerarea,4);

            stackmisalignment:=registerarea mod current_settings.alignment.localalignmax;

            LocalSize:=current_procinfo.calc_stackframe_size;
            if stack_parameters then
              localsize:=tarmprocinfo(current_procinfo).stackframesize-registerarea
            else
              localsize:=align(localsize+stackmisalignment,current_settings.alignment.localalignmax)-stackmisalignment;

            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) or
               (target_info.system in systems_darwin) then
              begin
                if (LocalSize<>0) or
                   ((stackmisalignment<>0) and
                    ((pi_do_call in current_procinfo.flags) or
                     (po_assembler in current_procinfo.procdef.procoptions))) then
                  begin
                    if LocalSize=0 then
                    else if LocalSize<=508 then
                      list.concat(taicpu.op_reg_reg_const(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize))
                    else if LocalSize<=1016 then
                      begin
                        list.concat(taicpu.op_reg_reg_const(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,508));
                        list.concat(taicpu.op_reg_reg_const(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,localsize-508));
                      end
                    else
                      begin
                        a_reg_alloc(list,NR_R3);
                        a_load_const_reg(list,OS_ADDR,LocalSize,NR_R3);
                        list.concat(taicpu.op_reg_reg_reg(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,NR_R3));
                        a_reg_dealloc(list,NR_R3);
                      end;
                  end;

                if regs=[] then
                  begin
                    if not(CPUARM_HAS_BX in cpu_capabilities[current_settings.cputype]) then
                      list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14))
                    else
                      list.concat(taicpu.op_reg(A_BX,NR_R14))
                  end
                else
                  list.concat(taicpu.op_regset(A_POP,R_INTREGISTER,R_SUBWHOLE,regs));
              end;
          end
        else if not(CPUARM_HAS_BX in cpu_capabilities[current_settings.cputype]) then
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14))
        else
          list.concat(taicpu.op_reg(A_BX,NR_R14))
      end;


     procedure tthumbcgarm.a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);
       var
         oppostfix:toppostfix;
         usedtmpref: treference;
         tmpreg,tmpreg2 : tregister;
         dir : integer;
       begin
         if (TCGSize2Size[FromSize] >= TCGSize2Size[ToSize]) then
           FromSize := ToSize;
=======
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
             InternalError(200308295);
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
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
             InternalError(200308298);
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
                     (ref.offset<-124) or
                     (ref.offset>124) or
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
                   tmpreg:=getintregister(list,OS_INT);
                   a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,reg);
                   inc(usedtmpref.offset,dir);
                   if FromSize=OS_16 then
                     a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg)
                   else
                     a_internal_load_ref_reg(list,OS_S8,OS_S8,usedtmpref,tmpreg);
                   list.concat(taicpu.op_reg_const(A_LSL,tmpreg,8));

                   list.concat(taicpu.op_reg_reg(A_ORR,reg,tmpreg));
                 end;
               OS_32,OS_S32:
                 begin
                   tmpreg:=getintregister(list,OS_INT);

                   { only complicated references need an extra loadaddr }
                   if assigned(ref.symbol) or
                     (ref.index<>NR_NO) or
                     (ref.offset<-124) or
                     (ref.offset>124) or
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

                   if ref.alignment=2 then
                     begin
                       if target_info.endian=endian_big then
                         inc(usedtmpref.offset,2);
                       a_internal_load_ref_reg(list,OS_16,OS_16,usedtmpref,reg);
                       inc(usedtmpref.offset,dir*2);
                       a_internal_load_ref_reg(list,OS_16,OS_16,usedtmpref,tmpreg);
                       list.concat(taicpu.op_reg_const(A_LSL,tmpreg,16));
                       list.concat(taicpu.op_reg_reg(A_ORR,reg,tmpreg));
                     end
                   else
                     begin
                       if target_info.endian=endian_big then
                         inc(usedtmpref.offset,3);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,reg);
                       inc(usedtmpref.offset,dir);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                       list.concat(taicpu.op_reg_const(A_LSL,tmpreg,8));
                       list.concat(taicpu.op_reg_reg(A_ORR,reg,tmpreg));
                       inc(usedtmpref.offset,dir);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                       list.concat(taicpu.op_reg_const(A_LSL,tmpreg,16));
                       list.concat(taicpu.op_reg_reg(A_ORR,reg,tmpreg));
                       inc(usedtmpref.offset,dir);
                       a_internal_load_ref_reg(list,OS_8,OS_8,usedtmpref,tmpreg);
                       list.concat(taicpu.op_reg_const(A_LSL,tmpreg,24));
                       list.concat(taicpu.op_reg_reg(A_ORR,reg,tmpreg));
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


     procedure tthumbcgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);
=======
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
>>>>>>> graemeg/cpstrnew
       var
          l : tasmlabel;
          hr : treference;
       begin
          if not(size in [OS_8,OS_S8,OS_16,OS_S16,OS_32,OS_S32]) then
            internalerror(2002090902);
          if is_thumb_imm(a) then
            list.concat(taicpu.op_reg_const(A_MOV,reg,a))
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


    procedure tthumbcgarm.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: tcgint);
      var
        hsym : tsym;
        href,
        tmpref : treference;
        paraloc : Pcgparalocation;
        l : TAsmLabel;
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
                    if is_thumb_imm(ioffset) then
                      a_op_const_reg(list,OP_SUB,size,ioffset,register)
                    else
                      begin
                        list.concat(taicpu.op_regset(A_PUSH,R_INTREGISTER,R_SUBWHOLE,[RS_R4]));
                        reference_reset(tmpref,4);
                        current_asmdata.getjumplabel(l);
                        current_procinfo.aktlocaldata.Concat(tai_align.Create(4));
                        cg.a_label(current_procinfo.aktlocaldata,l);
                        tmpref.symboldata:=current_procinfo.aktlocaldata.last;
                        current_procinfo.aktlocaldata.concat(tai_const.Create_32bit(ioffset));
                        tmpref.symbol:=l;
                        tmpref.base:=NR_PC;
                        list.concat(taicpu.op_reg_ref(A_LDR,NR_R4,tmpref));
                        a_op_reg_reg(list,OP_SUB,size,NR_R4,register);
                        list.concat(taicpu.op_regset(A_POP,R_INTREGISTER,R_SUBWHOLE,[RS_R4]));
                      end;
                  end;
                LOC_REFERENCE:
                  begin
                    { offset in the wrapper needs to be adjusted for the stored
                      return address }
                    reference_reset_base(href,reference.index,reference.offset+sizeof(aint),sizeof(pint));
                    if is_thumb_imm(ioffset) then
                      a_op_const_ref(list,OP_SUB,size,ioffset,href)
                    else
                      begin
                        list.concat(taicpu.op_regset(A_PUSH,R_INTREGISTER,R_SUBWHOLE,[RS_R4]));
                        reference_reset(tmpref,4);
                        current_asmdata.getjumplabel(l);
                        current_procinfo.aktlocaldata.Concat(tai_align.Create(4));
                        cg.a_label(current_procinfo.aktlocaldata,l);
                        tmpref.symboldata:=current_procinfo.aktlocaldata.last;
                        current_procinfo.aktlocaldata.concat(tai_const.Create_32bit(ioffset));
                        tmpref.symbol:=l;
                        tmpref.base:=NR_PC;
                        list.concat(taicpu.op_reg_ref(A_LDR,NR_R4,tmpref));
                        a_op_reg_ref(list,OP_SUB,size,NR_R4,href);
                        list.concat(taicpu.op_regset(A_POP,R_INTREGISTER,R_SUBWHOLE,[RS_R4]));
                      end;
                  end
                else
                  internalerror(200309189);
              end;
              paraloc:=next;
            end;
      end;


    function tthumbcgarm.handle_load_store(list: TAsmList; op: tasmop; oppostfix: toppostfix; reg: tregister; ref: treference): treference;
      var
        href : treference;
        tmpreg : TRegister;
      begin
        href:=ref;
        if { LDR/STR limitations }
           (
            (((op=A_LDR) and (oppostfix=PF_None)) or
             ((op=A_STR) and (oppostfix=PF_None))) and
            (ref.base<>NR_STACK_POINTER_REG) and
            (abs(ref.offset)>124)
           ) or
           { LDRB/STRB limitations }
           (
           (((op=A_LDR) and (oppostfix=PF_B)) or
            ((op=A_LDRB) and (oppostfix=PF_None)) or
            ((op=A_STR) and (oppostfix=PF_B)) or
            ((op=A_STRB) and (oppostfix=PF_None))) and
            ((ref.base=NR_STACK_POINTER_REG) or
             (ref.index=NR_STACK_POINTER_REG) or
             (abs(ref.offset)>31)
            )
           ) or
           { LDRH/STRH limitations }
           (
            (((op=A_LDR) and (oppostfix=PF_H)) or
             ((op=A_LDRH) and (oppostfix=PF_None)) or
             ((op=A_STR) and (oppostfix=PF_H)) or
             ((op=A_STRH) and (oppostfix=PF_None))) and
            ((ref.base=NR_STACK_POINTER_REG) or
             (ref.index=NR_STACK_POINTER_REG) or
             (abs(ref.offset)>62) or
             ((abs(ref.offset) mod 2)<>0)
            )
           ) then
          begin
            tmpreg:=getintregister(list,OS_ADDR);
            a_loadaddr_ref_reg(list,ref,tmpreg);

            reference_reset_base(href,tmpreg,0,ref.alignment);
          end
        else if (op=A_LDR) and
           (oppostfix in [PF_None]) and
           (ref.base=NR_STACK_POINTER_REG) and
           (abs(ref.offset)>1020) then
          begin
            tmpreg:=getintregister(list,OS_ADDR);
            a_loadaddr_ref_reg(list,ref,tmpreg);

            reference_reset_base(href,tmpreg,0,ref.alignment);
          end
        else if (op=A_LDR) and
           ((oppostfix in [PF_SH,PF_SB]) or
            (abs(ref.offset)>124)) then
          begin
            tmpreg:=getintregister(list,OS_ADDR);
            a_loadaddr_ref_reg(list,ref,tmpreg);

            reference_reset_base(href,tmpreg,0,ref.alignment);
          end;

        Result:=inherited handle_load_store(list, op, oppostfix, reg, href);
      end;


    procedure tthumbcgarm.a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister);
      var
<<<<<<< HEAD
        tmpreg : tregister;
      begin
        case op of
          OP_NEG:
            list.concat(taicpu.op_reg_reg(A_NEG,dst,src));
          OP_NOT:
            list.concat(taicpu.op_reg_reg(A_MVN,dst,src));
          OP_DIV,OP_IDIV:
            internalerror(200308284);
          OP_ROL:
            begin
              if not(size in [OS_32,OS_S32]) then
                internalerror(2008072801);
              { simulate ROL by ror'ing 32-value }
              tmpreg:=getintregister(list,OS_32);
              a_load_const_reg(list,OS_32,32,tmpreg);
              list.concat(taicpu.op_reg_reg(A_SUB,tmpreg,src));
              list.concat(taicpu.op_reg_reg(A_ROR,dst,src));
            end;
          else
            begin
              a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(setoppostfix(
                taicpu.op_reg_reg(op_reg_opcg2asmop[op],dst,src),op_reg_postfix[op]));
            end;
        end;
        maybeadjustresult(list,op,size,dst);
      end;

=======
         ref : treference;
         shift : byte;
         firstfloatreg,lastfloatreg,
         r : byte;
         mmregs,
         regs : tcpuregisterset;
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
              fpu_vfpv3:
                begin;
                  mmregs:=rg[R_MMREGISTER].used_in_proc-paramanager.get_volatile_registers_mm(pocall_stdcall);
                end;
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
>>>>>>> graemeg/cpstrnew

    procedure tthumbcgarm.a_op_const_reg(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; dst: tregister);
      var
        tmpreg : tregister;
        {$ifdef DUMMY}
        l1 : longint;
        {$endif DUMMY}
      begin
        //!!! ovloc.loc:=LOC_VOID;
        if {$ifopt R+}(a<>-2147483648) and{$endif} {!!!!!! not setflags and } is_thumb_imm(-a) then
          case op of
            OP_ADD:
              begin
                op:=OP_SUB;
                a:=aint(dword(-a));
              end;
<<<<<<< HEAD
            OP_SUB:
              begin
                op:=OP_ADD;
                a:=aint(dword(-a));
              end
          end;

        if is_thumb_imm(a) and (op in [OP_ADD,OP_SUB]) then
          begin
             // if cgsetflags or setflags then
             a_reg_alloc(list,NR_DEFAULTFLAGS);
            list.concat(setoppostfix(
              taicpu.op_reg_const(op_reg_opcg2asmop[op],dst,a),op_reg_postfix[op]));

            if (cgsetflags {!!! or setflags }) and (size in [OS_8,OS_16,OS_32]) then
              begin
                //!!! ovloc.loc:=LOC_FLAGS;
                case op of
                  OP_ADD:
                    //!!! ovloc.resflags:=F_CS;
                    ;
                  OP_SUB:
                    //!!! ovloc.resflags:=F_CC;
                    ;
                end;
              end;
          end
        else
          begin
            { there could be added some more sophisticated optimizations }
            if (op in [OP_MUL,OP_IMUL,OP_DIV,OP_IDIV]) and (a=1) then
              a_load_reg_reg(list,size,size,dst,dst)
            else if (op in [OP_MUL,OP_IMUL]) and (a=0) then
              a_load_const_reg(list,size,0,dst)
            else if (op in [OP_IMUL,OP_IDIV]) and (a=-1) then
              a_op_reg_reg(list,OP_NEG,size,dst,dst)
            { we do this here instead in the peephole optimizer because
              it saves us a register }
{$ifdef DUMMY}
            else if (op in [OP_MUL,OP_IMUL]) and ispowerof2(a,l1) and not(cgsetflags or setflags) then
              a_op_const_reg_reg(list,OP_SHL,size,l1,dst,dst)
            { for example : b=a*5 -> b=a*4+a with add instruction and shl }
            else if (op in [OP_MUL,OP_IMUL]) and ispowerof2(a-1,l1) and not(cgsetflags or setflags) then
              begin
                if l1>32 then{roozbeh does this ever happen?}
                  internalerror(200308296);
                shifterop_reset(so);
                so.shiftmode:=SM_LSL;
                so.shiftimm:=l1;
                list.concat(taicpu.op_reg_reg_reg_shifterop(A_ADD,dst,dst,dst,so));
              end
            { for example : b=a*7 -> b=a*8-a with rsb instruction and shl }
            else if (op in [OP_MUL,OP_IMUL]) and ispowerof2(a+1,l1) and not(cgsetflags or setflags) then
              begin
                if l1>32 then{does this ever happen?}
                  internalerror(201205181);
                shifterop_reset(so);
                so.shiftmode:=SM_LSL;
                so.shiftimm:=l1;
                list.concat(taicpu.op_reg_reg_reg_shifterop(A_RSB,dst,dst,dst,so));
              end
            else if (op in [OP_MUL,OP_IMUL]) and not(cgsetflags or setflags) and try_optimized_mul32_const_reg_reg(list,a,dst,dst) then
              begin
                { nothing to do on success }
              end
{$endif DUMMY}
            { x := y and 0; just clears a register, this sometimes gets generated on 64bit ops.
              Just using mov x, #0 might allow some easier optimizations down the line. }
            else if (op = OP_AND) and (dword(a)=0) then
              list.concat(taicpu.op_reg_const(A_MOV,dst,0))
            { x := y AND $FFFFFFFF just copies the register, so use mov for better optimizations }
            else if (op = OP_AND) and (not(dword(a))=0) then
              // do nothing
            { BIC clears the specified bits, while AND keeps them, using BIC allows to use a
              broader range of shifterconstants.}
{$ifdef DUMMY}
            else if (op = OP_AND) and is_shifter_const(not(dword(a)),shift) then
              list.concat(taicpu.op_reg_reg_const(A_BIC,dst,dst,not(dword(a))))
            else if (op = OP_AND) and split_into_shifter_const(not(dword(a)), imm1, imm2) then
              begin
                list.concat(taicpu.op_reg_reg_const(A_BIC,dst,dst,imm1));
                list.concat(taicpu.op_reg_reg_const(A_BIC,dst,dst,imm2));
              end
            else if (op in [OP_ADD, OP_SUB, OP_OR]) and
                    not(cgsetflags or setflags) and
                    split_into_shifter_const(a, imm1, imm2) then
              begin
                list.concat(taicpu.op_reg_reg_const(op_reg_reg_opcg2asmop[op],dst,dst,imm1));
                list.concat(taicpu.op_reg_reg_const(op_reg_reg_opcg2asmop[op],dst,dst,imm2));
              end
{$endif DUMMY}
            else if (op in [OP_SHL, OP_SHR, OP_SAR]) then
              begin
                list.concat(taicpu.op_reg_reg_const(op_reg_opcg2asmop[op],dst,dst,a));
              end
            else
              begin
                tmpreg:=getintregister(list,size);
                a_load_const_reg(list,size,a,tmpreg);
                a_op_reg_reg(list,op,size,tmpreg,dst);
              end;
          end;
        maybeadjustresult(list,op,size,dst);
      end;


    procedure tthumbcgarm.a_op_const_reg_reg(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister);
      begin
        if (op=OP_ADD) and (src=NR_R13) and (dst<>NR_R13) and ((a mod 4)=0) and (a>0) and (a<=1020) then
          list.concat(taicpu.op_reg_reg_const(A_ADD,dst,src,a))
        else
          inherited a_op_const_reg_reg(list,op,size,a,src,dst);
=======

            if (mmregs<>[]) or
               (firstfloatreg<>RS_NO) then
             begin
               reference_reset(ref,4);
               if (tg.direction*tarmprocinfo(current_procinfo).floatregstart>=1023) or
                  (current_settings.fputype in [fpu_vfpv2,fpu_vfpv3]) then
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
                 fpu_vfpv3:
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
              fpu_vfpv3:
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
                   (current_settings.fputype in [fpu_vfpv2,fpu_vfpv3]) then
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
                  fpu_vfpv3:
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
            if (pi_do_call in current_procinfo.flags) or (regs<>[]) then
              begin
                exclude(regs,RS_R14);
                include(regs,RS_R15);
              end;
            { restore saved stack pointer to SP (R13) and saved lr to PC (R15).
              The saved PC came after that but is discarded, since we restore
              the stack pointer }
            if (current_procinfo.framepointer<>NR_STACK_POINTER_REG) then
              regs:=regs+[RS_FRAME_POINTER_REG,RS_R13,RS_R15];

            for r:=RS_R0 to RS_R15 do
              if (r in regs) then
                inc(stackmisalignment,4);
            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) then
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

                if regs=[] then
                  begin
                    if (current_settings.cputype<cpu_armv6) then
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
        else if (current_settings.cputype<cpu_armv6) then
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14))
        else
          list.concat(taicpu.op_reg(A_BX,NR_R14))
>>>>>>> graemeg/cpstrnew
      end;


    procedure tthumbcgarm.g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister);
      var
        l1,l2 : tasmlabel;
        ai : taicpu;
      begin
        current_asmdata.getjumplabel(l1);
        current_asmdata.getjumplabel(l2);
        ai:=setcondition(taicpu.op_sym(A_B,l1),flags_to_cond(f));
        ai.is_jmp:=true;
        list.concat(ai);
        list.concat(taicpu.op_reg_const(A_MOV,reg,0));
        list.concat(taicpu.op_sym(A_B,l2));
        cg.a_label(list,l1);
        list.concat(taicpu.op_reg_const(A_MOV,reg,1));
        a_reg_dealloc(list,NR_DEFAULTFLAGS);
        cg.a_label(list,l2);
      end;


    procedure tthumb2cgarm.init_register_allocators;
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
        rg[R_FPUREGISTER]:=trgcpu.create(R_FPUREGISTER,R_SUBNONE,
            [RS_F0,RS_F1,RS_F2,RS_F3,RS_F4,RS_F5,RS_F6,RS_F7],first_fpu_imreg,[]);

        if current_settings.fputype=fpu_vfpv3 then
          rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBFD,
              [RS_D0,RS_D1,RS_D2,RS_D3,RS_D4,RS_D5,RS_D6,RS_D7,
               RS_D16,RS_D17,RS_D18,RS_D19,RS_D20,RS_D21,RS_D22,RS_D23,RS_D24,RS_D25,RS_D26,RS_D27,RS_D28,RS_D29,RS_D30,RS_D31,
               RS_D8,RS_D9,RS_D10,RS_D11,RS_D12,RS_D13,RS_D14,RS_D15
              ],first_mm_imreg,[])
        else if current_settings.fputype in [fpu_fpv4_s16,fpu_vfpv3_d16] then
          rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBFD,
              [RS_D0,RS_D1,RS_D2,RS_D3,RS_D4,RS_D5,RS_D6,RS_D7,
               RS_D8,RS_D9,RS_D10,RS_D11,RS_D12,RS_D13,RS_D14,RS_D15
              ],first_mm_imreg,[])
        else
          rg[R_MMREGISTER]:=trgcpu.create(R_MMREGISTER,R_SUBNONE,
              [RS_S0,RS_S1,RS_R2,RS_R3,RS_R4,RS_S31],first_mm_imreg,[]);
      end;


    procedure tthumb2cgarm.done_register_allocators;
      begin
        rg[R_INTREGISTER].free;
        rg[R_FPUREGISTER].free;
        rg[R_MMREGISTER].free;
        inherited done_register_allocators;
      end;


    procedure tthumb2cgarm.a_call_reg(list : TAsmList;reg: tregister);
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


     procedure tthumb2cgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : tcgint;reg : tregister);
       var
          l : tasmlabel;
          hr : treference;
       begin
          if not(size in [OS_8,OS_S8,OS_16,OS_S16,OS_32,OS_S32]) then
            internalerror(2002090902);
          if is_thumb32_imm(a) then
            list.concat(taicpu.op_reg_const(A_MOV,reg,a))
          else if is_thumb32_imm(not(a)) then
            list.concat(taicpu.op_reg_const(A_MVN,reg,not(a)))
          else if (a and $FFFF)=a then
            list.concat(taicpu.op_reg_const(A_MOVW,reg,a))
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


     procedure tthumb2cgarm.a_load_ref_reg(list : TAsmList; fromsize, tosize : tcgsize;const Ref : treference;reg : tregister);
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
             InternalError(200308299);
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


    procedure tthumb2cgarm.a_op_reg_reg(list : TAsmList; Op: TOpCG; size: TCGSize; src, dst: TRegister);
      begin
        if op = OP_NOT then
          begin
            list.concat(taicpu.op_reg_reg(A_MVN,dst,src));
            case size of
              OS_8: list.concat(taicpu.op_reg_reg(A_UXTB,dst,dst));
              OS_S8: list.concat(taicpu.op_reg_reg(A_SXTB,dst,dst));
              OS_16: list.concat(taicpu.op_reg_reg(A_UXTH,dst,dst));
              OS_S16: list.concat(taicpu.op_reg_reg(A_SXTH,dst,dst));
            end;
          end
        else
          inherited a_op_reg_reg(list, op, size, src, dst);
      end;


    procedure tthumb2cgarm.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: tcgint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
      var
        shift, width : byte;
        tmpreg : tregister;
        so : tshifterop;
        l1 : longint;
      begin
<<<<<<< HEAD
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
=======
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


    procedure tcgarm.g_concatcopy_internal(list : TAsmList;const source,dest : treference;len : aint;aligned : boolean);
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
>>>>>>> graemeg/cpstrnew

        if is_shifter_const(a,shift) and not(op in [OP_IMUL,OP_MUL]) then
          case op of
            OP_NEG,OP_NOT,
            OP_DIV,OP_IDIV:
              internalerror(200308285);
            OP_SHL:
              begin
                if a>32 then
                  internalerror(2014020703);
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
                  internalerror(2014020704);
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
                  internalerror(2014020705);
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
                  if cgsetflags or setflags then
                    a_reg_alloc(list,NR_DEFAULTFLAGS);
                  list.concat(setoppostfix(
                    taicpu.op_reg_reg_reg(op_reg_reg_opcg2asmop[op],dst,src,tmpreg),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))));
                end
              else
                begin
                  if cgsetflags or setflags then
                    a_reg_alloc(list,NR_DEFAULTFLAGS);
                  list.concat(setoppostfix(
                    taicpu.op_reg_reg_const(op_reg_reg_opcg2asmop[op],dst,src,a),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))));
                end;
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
            else if (op in [OP_MUL,OP_IMUL]) and not(cgsetflags or setflags) and try_optimized_mul32_const_reg_reg(list,a,src,dst) then
              begin
                { nothing to do on success }
              end
            { x := y and 0; just clears a register, this sometimes gets generated on 64bit ops.
              Just using mov x, #0 might allow some easier optimizations down the line. }
            else if (op = OP_AND) and (dword(a)=0) then
              list.concat(taicpu.op_reg_const(A_MOV,dst,0))
            { x := y AND $FFFFFFFF just copies the register, so use mov for better optimizations }
            else if (op = OP_AND) and (not(dword(a))=0) then
              list.concat(taicpu.op_reg_reg(A_MOV,dst,src))
            { BIC clears the specified bits, while AND keeps them, using BIC allows to use a
              broader range of shifterconstants.}
            {else if (op = OP_AND) and is_shifter_const(not(dword(a)),shift) then
              list.concat(taicpu.op_reg_reg_const(A_BIC,dst,src,not(dword(a))))}
            else if (op = OP_AND) and is_thumb32_imm(a) then
              list.concat(taicpu.op_reg_reg_const(A_AND,dst,src,dword(a)))
            else if (op = OP_AND) and (a = $FFFF) then
              list.concat(taicpu.op_reg_reg(A_UXTH,dst,src))
            else if (op = OP_AND) and is_thumb32_imm(not(dword(a))) then
              list.concat(taicpu.op_reg_reg_const(A_BIC,dst,src,not(dword(a))))
            else if (op = OP_AND) and is_continuous_mask(not(a), shift, width) then
              begin
                a_load_reg_reg(list,size,size,src,dst);
                list.concat(taicpu.op_reg_const_const(A_BFC,dst,shift,width))
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


    procedure tthumb2cgarm.a_op_reg_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; src1, src2, dst: tregister;setflags : boolean;var ovloc : tlocation);
      var
        so : tshifterop;
        tmpreg,overflowreg : tregister;
        asmop : tasmop;
      begin
        ovloc.loc:=LOC_VOID;
        case op of
           OP_NEG,OP_NOT:
              internalerror(200308286);
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
                      a_reg_alloc(list,NR_DEFAULTFLAGS);
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
             begin
               if cgsetflags or setflags then
                 a_reg_alloc(list,NR_DEFAULTFLAGS);
{$ifdef dummy}
               { R13 is not allowed for certain instruction operands }
               if op_reg_reg_opcg2asmopThumb2[op] in [A_ADD,A_SUB,A_AND,A_BIC,A_EOR] then
                 begin
                   if getsupreg(dst)=RS_R13 then
                     begin
                       tmpreg:=getintregister(list,OS_INT);
                       a_load_reg_reg(list,OS_INT,OS_INT,dst,tmpreg);
                       dst:=tmpreg;
                     end;
                   if getsupreg(src1)=RS_R13 then
                     begin
                       tmpreg:=getintregister(list,OS_INT);
                       a_load_reg_reg(list,OS_INT,OS_INT,src1,tmpreg);
                       src1:=tmpreg;
                     end;
                 end;
{$endif}
               list.concat(setoppostfix(
                 taicpu.op_reg_reg_reg(op_reg_reg_opcg2asmopThumb2[op],dst,src2,src1),toppostfix(ord(cgsetflags or setflags)*ord(PF_S))));
             end;
        end;
        maybeadjustresult(list,op,size,dst);
      end;


    procedure tthumb2cgarm.g_flags2reg(list: TAsmList; size: TCgSize; const f: TResFlags; reg: TRegister);
      begin
        list.concat(taicpu.op_cond(A_ITE, flags_to_cond(f)));
        list.concat(setcondition(taicpu.op_reg_const(A_MOV,reg,1),flags_to_cond(f)));
        list.concat(setcondition(taicpu.op_reg_const(A_MOV,reg,0),inverse_cond(flags_to_cond(f))));
      end;


    procedure tthumb2cgarm.g_proc_entry(list : TAsmList;localsize : longint;nostackframe:boolean);
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
            lastfloatreg:=RS_NO;
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


    procedure tthumb2cgarm.g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean);
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
            lastfloatreg:=RS_NO;
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
                    list.concat(taicpu.op_reg_reg(A_ADD,NR_STACK_POINTER_REG,NR_R12));
                    a_reg_dealloc(list,NR_R12);
                  end
                else
                  begin
                    a_reg_dealloc(list,NR_R12);
                    list.concat(taicpu.op_reg_const(A_ADD,NR_STACK_POINTER_REG,LocalSize));
                  end;
              end;

            if regs=[] then
              list.concat(taicpu.op_reg_reg(A_MOV,NR_R15,NR_R14))
            else
              begin
                reference_reset(ref,4);
                ref.index:=NR_STACK_POINTER_REG;
                ref.addressmode:=AM_PREINDEXED;
                list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_FD));
              end;
          end
        else
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14));
      end;


    function tthumb2cgarm.handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference;
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
              internalerror(2014020706);
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
           (((op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) or (op=A_VSTR) or (op=A_VLDR)) and
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

<<<<<<< HEAD
        { Hack? Thumb2 doesn't allow PC indexed addressing modes(although it does in the specification) }
        if (ref.base=NR_R15) and (ref.index<>NR_NO) and (ref.shiftmode <> sm_none) then
          begin
            tmpreg:=getintregister(list,OS_ADDR);

            list.concat(taicpu.op_reg_reg(A_MOV, tmpreg, NR_R15));

            ref.base := tmpreg;
          end;

        { floating point operations have only limited references
          we expect here, that a base is already set }
        if ((op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) or (op=A_VSTR) or (op=A_VLDR)) and (ref.index<>NR_NO) then
          begin
            if ref.shiftmode<>SM_none then
              internalerror(200309121);
            if tmpreg<>NR_NO then
              begin
                if ref.base=tmpreg then
=======
    procedure tcgarm.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);
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
>>>>>>> graemeg/cpstrnew
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


<<<<<<< HEAD
    procedure tthumb2cgarm.a_loadmm_reg_reg(list: TAsmList; fromsize, tosize: tcgsize; reg1, reg2: tregister; shuffle: pmmshuffle);
      var
        instr: taicpu;
      begin
        if (fromsize=OS_F32) and
          (tosize=OS_F32) then
=======
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


<<<<<<< HEAD
    procedure tcgarm.a_loadmm_reg_ref(list: tasmlist; fromsize,tosize: tcgsize; reg: tregister; const ref: treference; shuffle: pmmshuffle);
=======
    procedure tcgarm.a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
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
=======
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
>>>>>>> graemeg/cpstrnew
                end;
              OS_F64:
                begin
<<<<<<< HEAD
                  reg64.reglo:=getintregister(list,OS_32);
                  reg64.reghi:=getintregister(list,OS_32);
                  cg64.a_loadmm_reg_intreg64(list,OS_F64,tmpmmreg,reg64);
                  cg64.a_load64_reg_ref(list,reg64,ref);
=======
                  reference_reset_base(href2,hloc^.reference.index,hloc^.reference.offset,paraloc.alignment);
                  { concatcopy should choose the best way to copy the data }
                  g_concatcopy(list,href,href2,tcgsize2size[hloc^.size]);
>>>>>>> graemeg/cpstrnew
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
        if po_virtualmethod in procdef.procoptions then
>>>>>>> graemeg/cpstrnew
          begin
            instr:=setoppostfix(taicpu.op_reg_reg(A_VMOV,reg2,reg1), PF_F32);
            list.Concat(instr);
            add_move_instruction(instr);
          end
        else if (fromsize=OS_F64) and
          (tosize=OS_F64) then
          begin
            //list.Concat(setoppostfix(taicpu.op_reg_reg(A_VMOV,tregister(longint(reg2)+1),tregister(longint(reg1)+1)), PF_F32));
            //list.Concat(setoppostfix(taicpu.op_reg_reg(A_VMOV,reg2,reg1), PF_F32));
          end
        else if (fromsize=OS_F32) and
          (tosize=OS_F64) then
          //list.Concat(setoppostfix(taicpu.op_reg_reg(A_VCVT,reg2,reg1), PF_F32))
          begin
            //list.concat(nil);
          end;
      end;


    procedure tthumb2cgarm.a_loadmm_ref_reg(list: TAsmList; fromsize, tosize: tcgsize; const ref: treference; reg: tregister; shuffle: pmmshuffle);
      begin
        handle_load_store(list,A_VLDR,PF_None,reg,ref);
      end;


    procedure tthumb2cgarm.a_loadmm_reg_ref(list: TAsmList; fromsize, tosize: tcgsize; reg: tregister; const ref: treference; shuffle: pmmshuffle);
      begin
        handle_load_store(list,A_VSTR,PF_None,reg,ref);
      end;


    procedure tthumb2cgarm.a_loadmm_intreg_reg(list: TAsmList; fromsize, tosize: tcgsize; intreg, mmreg: tregister; shuffle: pmmshuffle);
      begin
        if //(shuffle=nil) and
          (tosize=OS_F32) then
          list.Concat(taicpu.op_reg_reg(A_VMOV,mmreg,intreg))
        else
          internalerror(2012100813);
      end;

<<<<<<< HEAD

    procedure tthumb2cgarm.a_loadmm_reg_intreg(list: TAsmList; fromsize, tosize: tcgsize; mmreg, intreg: tregister; shuffle: pmmshuffle);
      begin
        if //(shuffle=nil) and
          (fromsize=OS_F32) then
          list.Concat(taicpu.op_reg_reg(A_VMOV,intreg,mmreg))
        else
          internalerror(2012100814);
=======
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
>>>>>>> graemeg/cpstrnew
      end;


    procedure tthumb2cg64farm.a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);
      var tmpreg: tregister;
      begin
        case op of
          OP_NEG:
            begin
              cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(setoppostfix(taicpu.op_reg_reg_const(A_RSB,regdst.reglo,regsrc.reglo,0),PF_S));
              tmpreg:=cg.getintregister(list,OS_32);
              list.concat(taicpu.op_reg_const(A_MOV,tmpreg,0));
              list.concat(taicpu.op_reg_reg_reg(A_SBC,regdst.reghi,tmpreg,regsrc.reghi));
              cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
            end;
          else
            inherited a_op64_reg_reg(list, op, size, regsrc, regdst);
        end;
      end;


    procedure tthumbcg64farm.a_op64_reg_reg(list: TAsmList; op: TOpCG; size: tcgsize; regsrc, regdst: tregister64);
      begin
        case op of
          OP_NEG:
            begin
              list.concat(taicpu.op_reg_const(A_MOV,regdst.reglo,0));
              list.concat(taicpu.op_reg_const(A_MOV,regdst.reghi,0));
              cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(taicpu.op_reg_reg(A_SUB,regdst.reglo,regsrc.reglo));
              list.concat(taicpu.op_reg_reg(A_SBC,regdst.reghi,regsrc.reghi));
              cg.a_reg_dealloc(list,NR_DEFAULTFLAGS);
            end;
          OP_NOT:
            begin
              cg.a_op_reg_reg(list,OP_NOT,OS_INT,regsrc.reglo,regdst.reglo);
              cg.a_op_reg_reg(list,OP_NOT,OS_INT,regsrc.reghi,regdst.reghi);
            end;
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_reg_reg(list,op,OS_32,regsrc.reglo,regdst.reglo);
              cg.a_op_reg_reg(list,op,OS_32,regsrc.reghi,regdst.reghi);
            end;
          OP_ADD:
            begin
              cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(taicpu.op_reg_reg(A_ADD,regdst.reglo,regsrc.reglo));
              list.concat(taicpu.op_reg_reg(A_ADC,regdst.reghi,regsrc.reghi));
            end;
          OP_SUB:
            begin
              cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
              list.concat(taicpu.op_reg_reg(A_SUB,regdst.reglo,regsrc.reglo));
              list.concat(taicpu.op_reg_reg(A_SBC,regdst.reghi,regsrc.reghi));
            end;
          else
            internalerror(2003083101);
        end;
      end;


<<<<<<< HEAD
    procedure tthumbcg64farm.a_op64_const_reg(list: TAsmList; op: TOpCG; size: tcgsize; value: int64; reg: tregister64);
=======
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
>>>>>>> graemeg/cpstrnew
      var
<<<<<<< HEAD
        tmpreg : tregister;
      begin
        case op of
          OP_AND,OP_OR,OP_XOR:
            begin
              cg.a_op_const_reg(list,op,OS_32,aint(lo(value)),reg.reglo);
              cg.a_op_const_reg(list,op,OS_32,aint(hi(value)),reg.reghi);
            end;
          OP_ADD:
            begin
              if (aint(lo(value))>=0) and (aint(lo(value))<=255) then
                begin
                  cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                  list.concat(taicpu.op_reg_const(A_ADD,reg.reglo,aint(lo(value))));
                end
              else
                begin
                  tmpreg:=cg.getintregister(list,OS_32);
                  cg.a_load_const_reg(list,OS_32,aint(lo(value)),tmpreg);
                  cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                  list.concat(taicpu.op_reg_reg(A_ADD,reg.reglo,tmpreg));
                end;

               tmpreg:=cg.getintregister(list,OS_32);
               cg.a_load_const_reg(list,OS_32,aint(hi(value)),tmpreg);
               list.concat(taicpu.op_reg_reg(A_ADC,reg.reghi,tmpreg));
            end;
          OP_SUB:
            begin
              if (aint(lo(value))>=0) and (aint(lo(value))<=255) then
                begin
                  cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                  list.concat(taicpu.op_reg_const(A_SUB,reg.reglo,aint(lo(value))))
                end
              else
                begin
                  tmpreg:=cg.getintregister(list,OS_32);
                  cg.a_load_const_reg(list,OS_32,aint(lo(value)),tmpreg);
                  cg.a_reg_alloc(list,NR_DEFAULTFLAGS);
                  list.concat(taicpu.op_reg_reg(A_SUB,reg.reglo,tmpreg));
=======
         ref : treference;
         shift : byte;
         firstfloatreg,lastfloatreg,
         r : byte;
         mmregs,
         regs : tcpuregisterset;
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
              fpu_vfpv3:
                begin;
                  mmregs:=rg[R_MMREGISTER].used_in_proc-paramanager.get_volatile_registers_mm(pocall_stdcall);
                end;
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
                    a_reg_dealloc(list,NR_R12);
                    list.concat(taicpu.op_reg_reg_const(A_SUB,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize));
                  end;
              end;

            if (mmregs<>[]) or
               (firstfloatreg<>RS_NO) then
             begin
               reference_reset(ref,4);
               if (tg.direction*tarmprocinfo(current_procinfo).floatregstart>=1023) or
                  (current_settings.fputype in [fpu_vfpv2,fpu_vfpv3]) then
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
                 fpu_vfpv3:
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
              fpu_vfpv3:
                begin;
                  { restore vfp registers? }
                  mmregs:=rg[R_MMREGISTER].used_in_proc-paramanager.get_volatile_registers_mm(pocall_stdcall);
>>>>>>> graemeg/cpstrnew
                end;
            end;

<<<<<<< HEAD
              tmpreg:=cg.getintregister(list,OS_32);
              cg.a_load_const_reg(list,OS_32,hi(value),tmpreg);
              list.concat(taicpu.op_reg_reg(A_SBC,reg.reghi,tmpreg));
            end;
          else
            internalerror(2003083101);
        end;
      end;
=======
            if (firstfloatreg<>RS_NO) or
               (mmregs<>[]) then
              begin
                reference_reset(ref,4);
                if (tg.direction*tarmprocinfo(current_procinfo).floatregstart>=1023) or
                   (current_settings.fputype in [fpu_vfpv2,fpu_vfpv3]) then
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
                  fpu_vfpv3:
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
            if (pi_do_call in current_procinfo.flags) or (regs<>[]) then
              begin
                exclude(regs,RS_R14);
                include(regs,RS_R15);
              end;
            { restore saved stack pointer to SP (R13) and saved lr to PC (R15).
              The saved PC came after that but is discarded, since we restore
              the stack pointer }
            if (current_procinfo.framepointer<>NR_STACK_POINTER_REG) then
              regs:=regs+[RS_FRAME_POINTER_REG,RS_R13,RS_R15];

            for r:=RS_R0 to RS_R15 do
              if (r in regs) then
                inc(stackmisalignment,4);
            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) then
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

                if regs=[] then
                  begin
                    if (current_settings.cputype<cpu_armv6) then
                      list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14))
                    else
                      list.concat(taicpu.op_reg(A_BX,NR_R14))
                  end
                else
<<<<<<< HEAD
                  begin
                    reference_reset(ref,4);
                    ref.index:=NR_STACK_POINTER_REG;
                    ref.addressmode:=AM_PREINDEXED;
                    list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_FD));
=======
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
>>>>>>> graemeg/cpstrnew
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
        else if (current_settings.cputype<cpu_armv6) then
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
>>>>>>> graemeg/cpstrnew


    procedure create_codegen;
      begin
        if GenerateThumb2Code then
          begin
            cg:=tthumb2cgarm.create;
            cg64:=tthumb2cg64farm.create;

            casmoptimizer:=TCpuThumb2AsmOptimizer;
          end
        else if GenerateThumbCode then
          begin
            cg:=tthumbcgarm.create;
            cg64:=tthumbcg64farm.create;

            // casmoptimizer:=TCpuThumbAsmOptimizer;
          end
        else
          begin
            cg:=tarmcgarm.create;
            cg64:=tarmcg64farm.create;

            casmoptimizer:=TCpuAsmOptimizer;
          end;
      end;

<<<<<<< HEAD
=======

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
<<<<<<< HEAD
        list.concat(taicpu.op_reg(A_BLX, reg));
{
        the compiler does not properly set this flag anymore in pass 1, and
        for now we only need it after pass 2 (I hope) (JM)
          if not(pi_do_call in current_procinfo.flags) then
            internalerror(2003060703);
}
        include(current_procinfo.flags,pi_do_call);
=======
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
>>>>>>> graemeg/cpstrnew
      end;


<<<<<<< HEAD
     procedure Tthumb2cgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);
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
=======
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
>>>>>>> graemeg/cpstrnew
         so : tshifterop;
         dir : integer;
       begin
         if (TCGSize2Size[FromSize] >= TCGSize2Size[ToSize]) then
           FromSize := ToSize;
<<<<<<< HEAD
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
=======
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
             InternalError(200308295);
         end;
         if (ref.alignment in [1,2]) and (ref.alignment<tcgsize2size[tosize]) then
>>>>>>> graemeg/cpstrnew
           begin
             if target_info.endian=endian_big then
               dir:=-1
             else
               dir:=1;
             case FromSize of
               OS_16,OS_S16:
                 begin
<<<<<<< HEAD
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
=======
                   shifterop_reset(so);so.shiftmode:=SM_LSR;so.shiftimm:=8;
                   tmpreg:=getintregister(list,OS_INT);
                   usedtmpref:=ref;
                   if target_info.endian=endian_big then
                     inc(usedtmpref.offset,1);
                   usedtmpref:=a_internal_load_reg_ref(list,OS_8,OS_8,reg,usedtmpref);
                   inc(usedtmpref.offset,dir);
                   list.concat(taicpu.op_reg_reg_shifterop(A_MOV,tmpreg,reg,so));
                   a_internal_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref);
>>>>>>> graemeg/cpstrnew
                 end;
               OS_32,OS_S32:
                 begin
                   tmpreg:=getintregister(list,OS_INT);
<<<<<<< HEAD

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


    procedure Tthumb2cgarm.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: aint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
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
=======
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
>>>>>>> graemeg/cpstrnew


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
           OP_NEG,OP_NOT,
           OP_DIV,OP_IDIV:
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


<<<<<<< HEAD
    procedure Tthumb2cgarm.g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean);
=======
    procedure tcgarm.a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);
>>>>>>> graemeg/cpstrnew
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
<<<<<<< HEAD
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
=======
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
>>>>>>> graemeg/cpstrnew
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

<<<<<<< HEAD
            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) then
              begin
                LocalSize:=current_procinfo.calc_stackframe_size;
                if (LocalSize<>0) or
                   ((stackmisalignment<>0) and
                    ((pi_do_call in current_procinfo.flags) or
                     (po_assembler in current_procinfo.procdef.procoptions))) then
=======
    procedure tcgarm.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);
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
>>>>>>> graemeg/cpstrnew
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

                if regs=[] then
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_R15,NR_R14))
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
                list.concat(taicpu.op_reg_reg(A_MOV, NR_STACK_POINTER_REG, NR_FRAME_POINTER_REG));
                { Add 4 to SP to make it point to an "imaginary PC" which the paramanager assumes is there(for normal ARM) }
                list.concat(taicpu.op_reg_const(A_ADD, NR_STACK_POINTER_REG, 4));

                reference_reset(ref,4);
                ref.index:=NR_STACK_POINTER_REG;
                list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_DB));
              end;
          end
        else
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14));
      end;


<<<<<<< HEAD
   function Tthumb2cgarm.handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference;
      var
        tmpreg : tregister;
        tmpref : treference;
        l : tasmlabel;
        so: tshifterop;
      begin
        tmpreg:=NR_NO;
=======
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
<<<<<<< HEAD
        make_global : boolean;
=======
         ref : treference;
         shift : byte;
         firstfloatreg,lastfloatreg,
         r : byte;
         mmregs,
         regs : tcpuregisterset;
         stackmisalignment : pint;
         postfix: toppostfix;
>>>>>>> graemeg/cpstrnew
      begin
        if not(procdef.proctypeoption in [potype_function,potype_procedure]) then
          Internalerror(200006137);
        if not assigned(procdef.struct) or
           (procdef.procoptions*[po_classmethod, po_staticmethod,
             po_methodpointer, po_interrupt, po_iocheck]<>[]) then
          Internalerror(200006138);
        if procdef.owner.symtabletype<>ObjectSymtable then
          Internalerror(200109191);
>>>>>>> graemeg/cpstrnew

        { Be sure to have a base register }
        if (ref.base=NR_NO) then
          begin
<<<<<<< HEAD
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
=======
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
              fpu_vfpv3:
                begin;
                  mmregs:=rg[R_MMREGISTER].used_in_proc-paramanager.get_volatile_registers_mm(pocall_stdcall);
                end;
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
>>>>>>> graemeg/cpstrnew

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

<<<<<<< HEAD
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

<<<<<<< HEAD
        { floating point operations have only limited references
          we expect here, that a base is already set }
        if (op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) and (ref.index<>NR_NO) then
          begin
            if ref.shiftmode<>SM_none then
              internalerror(200309121);
            if tmpreg<>NR_NO then
=======
            if (mmregs<>[]) or
               (firstfloatreg<>RS_NO) then
             begin
               reference_reset(ref,4);
               if (tg.direction*tarmprocinfo(current_procinfo).floatregstart>=1023) or
                  (current_settings.fputype in [fpu_vfpv2,fpu_vfpv3]) then
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
                 fpu_vfpv3:
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
              fpu_vfpv3:
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
                   (current_settings.fputype in [fpu_vfpv2,fpu_vfpv3]) then
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
                  fpu_vfpv3:
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
            if (pi_do_call in current_procinfo.flags) or (regs<>[]) then
              begin
                exclude(regs,RS_R14);
                include(regs,RS_R15);
              end;
            { restore saved stack pointer to SP (R13) and saved lr to PC (R15).
              The saved PC came after that but is discarded, since we restore
              the stack pointer }
            if (current_procinfo.framepointer<>NR_STACK_POINTER_REG) then
              regs:=regs+[RS_FRAME_POINTER_REG,RS_R13,RS_R15];

            for r:=RS_R0 to RS_R15 do
              if (r in regs) then
                inc(stackmisalignment,4);
            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) then
>>>>>>> graemeg/cpstrnew
              begin
                if ref.base=tmpreg then
                  begin
                    if ref.signindex<0 then
                      list.concat(taicpu.op_reg_reg_reg(A_SUB,tmpreg,tmpreg,ref.index))
                    else
<<<<<<< HEAD
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
=======
                      begin
                        list.concat(taicpu.op_reg_reg_const(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize));
                      end;
                  end;

                if regs=[] then
                  begin
                    if (current_settings.cputype<cpu_armv6) then
                      list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14))
                    else
                      list.concat(taicpu.op_reg(A_BX,NR_R14))
                  end
                else
<<<<<<< HEAD
                  begin
                    reference_reset(ref,4);
                    ref.index:=NR_STACK_POINTER_REG;
                    ref.addressmode:=AM_PREINDEXED;
                    list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_FD));
>>>>>>> graemeg/cpstrnew
=======
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
>>>>>>> origin/cpstrnew
                  end;
              end
            else
              begin
<<<<<<< HEAD
                tmpreg:=getintregister(list,OS_ADDR);
                list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,ref.index));
                ref.base:=tmpreg;
                ref.index:=NR_NO;
=======
                { restore int registers and return }
                reference_reset(ref,4);
                ref.index:=NR_FRAME_POINTER_REG;
                list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_EA));
>>>>>>> graemeg/cpstrnew
              end;
          end;
        list.concat(setoppostfix(taicpu.op_reg_ref(op,reg,ref),oppostfix));
        Result := ref;
=======
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
        else if (current_settings.cputype<cpu_armv6) then
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14))
        else
<<<<<<< HEAD
          internalerror(2008100401);

        new_section(current_asmdata.asmlists[al_imports],sec_data_lazy,'',sizeof(pint));
        current_asmdata.asmlists[al_imports].concat(Tai_symbol.Create(l1,0));
        current_asmdata.asmlists[al_imports].concat(tai_directive.create(asd_indirect_symbol,s));
        current_asmdata.asmlists[al_imports].concat(tai_const.createname('dyld_stub_binding_helper',0));
>>>>>>> graemeg/cpstrnew
=======
          list.concat(taicpu.op_reg(A_BX,NR_R14))
>>>>>>> graemeg/cpstrnew
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


<<<<<<< HEAD
<<<<<<< HEAD
    procedure create_codegen;
=======
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
>>>>>>> graemeg/cpstrnew
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

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======

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
<<<<<<< HEAD
        list.concat(taicpu.op_reg(A_BLX, reg));
{
        the compiler does not properly set this flag anymore in pass 1, and
        for now we only need it after pass 2 (I hope) (JM)
          if not(pi_do_call in current_procinfo.flags) then
            internalerror(2003060703);
}
        include(current_procinfo.flags,pi_do_call);
=======
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
>>>>>>> graemeg/cpstrnew
      end;


     procedure Tthumb2cgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);
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
=======
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
>>>>>>> origin/cpstrnew
         so : tshifterop;
         dir : integer;
       begin
         if (TCGSize2Size[FromSize] >= TCGSize2Size[ToSize]) then
           FromSize := ToSize;
<<<<<<< HEAD
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
=======
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
             InternalError(200308295);
         end;
         if (ref.alignment in [1,2]) and (ref.alignment<tcgsize2size[tosize]) then
>>>>>>> origin/cpstrnew
           begin
             if target_info.endian=endian_big then
               dir:=-1
             else
               dir:=1;
             case FromSize of
               OS_16,OS_S16:
                 begin
<<<<<<< HEAD
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
=======
                   shifterop_reset(so);so.shiftmode:=SM_LSR;so.shiftimm:=8;
                   tmpreg:=getintregister(list,OS_INT);
                   usedtmpref:=ref;
                   if target_info.endian=endian_big then
                     inc(usedtmpref.offset,1);
                   usedtmpref:=a_internal_load_reg_ref(list,OS_8,OS_8,reg,usedtmpref);
                   inc(usedtmpref.offset,dir);
                   list.concat(taicpu.op_reg_reg_shifterop(A_MOV,tmpreg,reg,so));
                   a_internal_load_reg_ref(list,OS_8,OS_8,tmpreg,usedtmpref);
>>>>>>> origin/cpstrnew
                 end;
               OS_32,OS_S32:
                 begin
                   tmpreg:=getintregister(list,OS_INT);
<<<<<<< HEAD

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


    procedure Tthumb2cgarm.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: aint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
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
=======
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
>>>>>>> origin/cpstrnew


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
           OP_NEG,OP_NOT,
           OP_DIV,OP_IDIV:
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


<<<<<<< HEAD
    procedure Tthumb2cgarm.g_proc_exit(list : TAsmList;parasize : longint;nostackframe:boolean);
=======
    procedure tcgarm.a_loadfpu_ref_cgpara(list : TAsmList;size : tcgsize;const ref : treference;const paraloc : TCGPara);
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
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
=======
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
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) then
              begin
                LocalSize:=current_procinfo.calc_stackframe_size;
                if (LocalSize<>0) or
                   ((stackmisalignment<>0) and
                    ((pi_do_call in current_procinfo.flags) or
                     (po_assembler in current_procinfo.procdef.procoptions))) then
=======
    procedure tcgarm.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);
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
>>>>>>> graemeg/cpstrnew
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

                if regs=[] then
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_R15,NR_R14))
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
                list.concat(taicpu.op_reg_reg(A_MOV, NR_STACK_POINTER_REG, NR_FRAME_POINTER_REG));
                { Add 4 to SP to make it point to an "imaginary PC" which the paramanager assumes is there(for normal ARM) }
                list.concat(taicpu.op_reg_const(A_ADD, NR_STACK_POINTER_REG, 4));

                reference_reset(ref,4);
                ref.index:=NR_STACK_POINTER_REG;
                list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_DB));
              end;
          end
        else
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14));
      end;


<<<<<<< HEAD
   function Tthumb2cgarm.handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference;
      var
        tmpreg : tregister;
        tmpref : treference;
        l : tasmlabel;
        so: tshifterop;
      begin
        tmpreg:=NR_NO;
=======
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
<<<<<<< HEAD
        make_global : boolean;
=======
         ref : treference;
         shift : byte;
         firstfloatreg,lastfloatreg,
         r : byte;
         mmregs,
         regs : tcpuregisterset;
         stackmisalignment : pint;
         postfix: toppostfix;
>>>>>>> origin/cpstrnew
      begin
        if not(procdef.proctypeoption in [potype_function,potype_procedure]) then
          Internalerror(200006137);
        if not assigned(procdef.struct) or
           (procdef.procoptions*[po_classmethod, po_staticmethod,
             po_methodpointer, po_interrupt, po_iocheck]<>[]) then
          Internalerror(200006138);
        if procdef.owner.symtabletype<>ObjectSymtable then
          Internalerror(200109191);
>>>>>>> graemeg/cpstrnew

        { Be sure to have a base register }
        if (ref.base=NR_NO) then
          begin
<<<<<<< HEAD
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
=======
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
              fpu_vfpv3:
                begin;
                  mmregs:=rg[R_MMREGISTER].used_in_proc-paramanager.get_volatile_registers_mm(pocall_stdcall);
                end;
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
>>>>>>> origin/cpstrnew

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

<<<<<<< HEAD
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

<<<<<<< HEAD
        { floating point operations have only limited references
          we expect here, that a base is already set }
        if (op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) and (ref.index<>NR_NO) then
          begin
            if ref.shiftmode<>SM_none then
              internalerror(200309121);
            if tmpreg<>NR_NO then
=======
            if (mmregs<>[]) or
               (firstfloatreg<>RS_NO) then
             begin
               reference_reset(ref,4);
               if (tg.direction*tarmprocinfo(current_procinfo).floatregstart>=1023) or
                  (current_settings.fputype in [fpu_vfpv2,fpu_vfpv3]) then
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
                 fpu_vfpv3:
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
              fpu_vfpv3:
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
                   (current_settings.fputype in [fpu_vfpv2,fpu_vfpv3]) then
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
                  fpu_vfpv3:
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
            if (pi_do_call in current_procinfo.flags) or (regs<>[]) then
              begin
                exclude(regs,RS_R14);
                include(regs,RS_R15);
              end;
            { restore saved stack pointer to SP (R13) and saved lr to PC (R15).
              The saved PC came after that but is discarded, since we restore
              the stack pointer }
            if (current_procinfo.framepointer<>NR_STACK_POINTER_REG) then
              regs:=regs+[RS_FRAME_POINTER_REG,RS_R13,RS_R15];

            for r:=RS_R0 to RS_R15 do
              if (r in regs) then
                inc(stackmisalignment,4);
            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) then
>>>>>>> origin/cpstrnew
              begin
                if ref.base=tmpreg then
                  begin
                    if ref.signindex<0 then
                      list.concat(taicpu.op_reg_reg_reg(A_SUB,tmpreg,tmpreg,ref.index))
                    else
<<<<<<< HEAD
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
=======
                      begin
                        list.concat(taicpu.op_reg_reg_const(A_ADD,NR_STACK_POINTER_REG,NR_STACK_POINTER_REG,LocalSize));
                      end;
                  end;

                if regs=[] then
                  begin
                    if (current_settings.cputype<cpu_armv6) then
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
>>>>>>> origin/cpstrnew
                  end;
              end
            else
              begin
<<<<<<< HEAD
                tmpreg:=getintregister(list,OS_ADDR);
                list.concat(taicpu.op_reg_reg_reg(A_ADD,tmpreg,ref.base,ref.index));
                ref.base:=tmpreg;
                ref.index:=NR_NO;
=======
                { restore int registers and return }
                reference_reset(ref,4);
                ref.index:=NR_FRAME_POINTER_REG;
                list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_EA));
>>>>>>> origin/cpstrnew
              end;
          end;
        list.concat(setoppostfix(taicpu.op_reg_ref(op,reg,ref),oppostfix));
        Result := ref;
=======
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
        else if (current_settings.cputype<cpu_armv6) then
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14))
        else
<<<<<<< HEAD
          internalerror(2008100401);

        new_section(current_asmdata.asmlists[al_imports],sec_data_lazy,'',sizeof(pint));
        current_asmdata.asmlists[al_imports].concat(Tai_symbol.Create(l1,0));
        current_asmdata.asmlists[al_imports].concat(tai_directive.create(asd_indirect_symbol,s));
        current_asmdata.asmlists[al_imports].concat(tai_const.createname('dyld_stub_binding_helper',0));
>>>>>>> graemeg/cpstrnew
=======
          list.concat(taicpu.op_reg(A_BX,NR_R14))
>>>>>>> origin/cpstrnew
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


<<<<<<< HEAD
    procedure create_codegen;
=======
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
>>>>>>> graemeg/cpstrnew
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

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======

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
<<<<<<< HEAD
        list.concat(taicpu.op_reg(A_BLX, reg));
{
        the compiler does not properly set this flag anymore in pass 1, and
        for now we only need it after pass 2 (I hope) (JM)
          if not(pi_do_call in current_procinfo.flags) then
            internalerror(2003060703);
}
        include(current_procinfo.flags,pi_do_call);
=======
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
>>>>>>> origin/cpstrnew
      end;


     procedure Tthumb2cgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);
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


    procedure Tthumb2cgarm.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: aint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
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
           OP_NEG,OP_NOT,
           OP_DIV,OP_IDIV:
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

<<<<<<< HEAD
            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) then
              begin
                LocalSize:=current_procinfo.calc_stackframe_size;
                if (LocalSize<>0) or
                   ((stackmisalignment<>0) and
                    ((pi_do_call in current_procinfo.flags) or
                     (po_assembler in current_procinfo.procdef.procoptions))) then
=======
    procedure tcgarm.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);
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
>>>>>>> origin/cpstrnew
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

                if regs=[] then
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_R15,NR_R14))
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
                list.concat(taicpu.op_reg_reg(A_MOV, NR_STACK_POINTER_REG, NR_FRAME_POINTER_REG));
                { Add 4 to SP to make it point to an "imaginary PC" which the paramanager assumes is there(for normal ARM) }
                list.concat(taicpu.op_reg_const(A_ADD, NR_STACK_POINTER_REG, 4));

                reference_reset(ref,4);
                ref.index:=NR_STACK_POINTER_REG;
                list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_DB));
              end;
          end
        else
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14));
      end;


<<<<<<< HEAD
   function Tthumb2cgarm.handle_load_store(list:TAsmList;op: tasmop;oppostfix : toppostfix;reg:tregister;ref: treference):treference;
      var
        tmpreg : tregister;
        tmpref : treference;
        l : tasmlabel;
        so: tshifterop;
      begin
        tmpreg:=NR_NO;
=======
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
>>>>>>> origin/cpstrnew

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

<<<<<<< HEAD
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
=======
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
>>>>>>> origin/cpstrnew
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


<<<<<<< HEAD
    procedure create_codegen;
=======
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
>>>>>>> origin/cpstrnew
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

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======

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


     procedure Tthumb2cgarm.a_load_const_reg(list : TAsmList; size: tcgsize; a : aint;reg : tregister);
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


    procedure Tthumb2cgarm.a_op_const_reg_reg_checkoverflow(list: TAsmList; op: TOpCg; size: tcgsize; a: aint; src, dst: tregister;setflags : boolean;var ovloc : tlocation);
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
           OP_NEG,OP_NOT,
           OP_DIV,OP_IDIV:
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

<<<<<<< HEAD
            stackmisalignment:=stackmisalignment mod current_settings.alignment.localalignmax;
            if (current_procinfo.framepointer=NR_STACK_POINTER_REG) then
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
=======
    procedure tcgarm.g_save_registers(list : TAsmList);
      begin
        { this work is done in g_proc_entry }
      end;
>>>>>>> graemeg/fixes_2_2

                if regs=[] then
                  list.concat(taicpu.op_reg_reg(A_MOV,NR_R15,NR_R14))
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
                list.concat(taicpu.op_reg_reg(A_MOV, NR_STACK_POINTER_REG, NR_FRAME_POINTER_REG));
                { Add 4 to SP to make it point to an "imaginary PC" which the paramanager assumes is there(for normal ARM) }
                list.concat(taicpu.op_reg_const(A_ADD, NR_STACK_POINTER_REG, 4));

<<<<<<< HEAD
                reference_reset(ref,4);
                ref.index:=NR_STACK_POINTER_REG;
                list.concat(setoppostfix(taicpu.op_ref_regset(A_LDM,ref,R_INTREGISTER,R_SUBWHOLE,regs),PF_DB));
              end;
          end
        else
          list.concat(taicpu.op_reg_reg(A_MOV,NR_PC,NR_R14));
=======
    procedure tcgarm.g_restore_registers(list : TAsmList);
      begin
        { this work is done in g_proc_exit }
>>>>>>> graemeg/fixes_2_2
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

<<<<<<< HEAD
        { absolute symbols can't be handled directly, we've to store the symbol reference
          in the text segment and access it pc relative
=======
    procedure tcgarm.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);
      var
        hsym : tsym;
        href : treference;
        paraloc : Pcgparalocation;
        shift : byte;
      begin
        { calculate the parameter info for the procdef }
        if not procdef.has_paraloc_info then
          begin
            procdef.requiredargarea:=paramanager.create_paraloc_info(procdef,callerside);
            procdef.has_paraloc_info:=true;
          end;
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
                    reference_reset_base(href,reference.index,reference.offset+sizeof(aint));
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


<<<<<<< HEAD
    procedure tcgarm.g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);
>>>>>>> graemeg/fixes_2_2

          For now, we assume that references where base or index equals to PC are already
          relative, all other references are assumed to be absolute and thus they need
          to be handled extra.
=======
    procedure tcgarm.g_save_registers(list : TAsmList);
      begin
        { this work is done in g_proc_entry }
      end;


    procedure tcgarm.g_restore_registers(list : TAsmList);
      begin
        { this work is done in g_proc_exit }
      end;
>>>>>>> origin/fixes_2_2

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

<<<<<<< HEAD
                current_procinfo.aktlocaldata.concat(tai_const.create_sym_offset(ref.symbol,ref.offset));
=======
    procedure tcgarm.g_adjust_self_value(list:TAsmList;procdef: tprocdef;ioffset: aint);
      var
        hsym : tsym;
        href : treference;
        paraloc : Pcgparalocation;
        shift : byte;
      begin
        { calculate the parameter info for the procdef }
        if not procdef.has_paraloc_info then
          begin
            procdef.requiredargarea:=paramanager.create_paraloc_info(procdef,callerside);
            procdef.has_paraloc_info:=true;
          end;
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
                    reference_reset_base(href,reference.index,reference.offset+sizeof(aint));
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


    procedure tcgarm.g_intf_wrapper(list: TAsmList; procdef: tprocdef; const labelname: string; ioffset: longint);
>>>>>>> origin/fixes_2_2

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

<<<<<<< HEAD
        { Hack? Thumb2 doesn't allow PC indexed addressing modes(although it does in the specification) }
        if (ref.base=NR_R15) and (ref.index<>NR_NO) and (ref.shiftmode <> sm_none) then
          begin
            tmpreg:=getintregister(list,OS_ADDR);
=======
        make_global:=false;
        if (not current_module.is_unit) or
           create_smartlink or
           (procdef.owner.defowner.owner.symtabletype=globalsymtable) then
          make_global:=true;
>>>>>>> graemeg/fixes_2_2

            list.concat(taicpu.op_reg_reg(A_MOV, tmpreg, NR_R15));

<<<<<<< HEAD
            ref.base := tmpreg;
          end;
=======
        { the wrapper might need aktlocaldata for the additional data to
          load the constant }
        current_procinfo:=cprocinfo.create(nil);

        { the wrapper might need aktlocaldata for the additional data to
          load the constant }
        current_procinfo:=cprocinfo.create(nil);

        { set param1 interface to self  }
        g_adjust_self_value(list,procdef,ioffset);
>>>>>>> graemeg/fixes_2_2

        { floating point operations have only limited references
          we expect here, that a base is already set }
        if (op in [A_LDF,A_STF,A_FLDS,A_FLDD,A_FSTS,A_FSTD]) and (ref.index<>NR_NO) then
          begin
<<<<<<< HEAD
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
=======
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


    procedure tcg64farm.a_op64_reg_reg(list : TAsmList;op:TOpCG;size : tcgsize;regsrc,regdst : tregister64);
>>>>>>> graemeg/fixes_2_2
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

>>>>>>> origin/cpstrnew
end.
