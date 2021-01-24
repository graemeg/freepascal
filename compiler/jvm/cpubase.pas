{
    Copyright (c) 2010 by Jonas Maebe

    Contains the base types for the Java VM

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
{ This Unit contains the base types for the Java Virtual Machine
}
unit cpubase;

{$i fpcdefs.inc}

interface

uses
  globtype,
  aasmbase,cpuinfo,cgbase;


{*****************************************************************************
                                Assembler Opcodes
*****************************************************************************}

    type
      TAsmOp=(A_None,
        a_aaload, a_aastore, a_aconst_null,
        a_aload, a_aload_0, a_aload_1, a_aload_2, a_aload_3,
        a_anewarray, a_areturn, a_arraylength,
        a_astore, a_astore_0, a_astore_1, a_astore_2, a_astore_3,
        a_athrow, a_baload, a_bastore, a_bipush, a_breakpoint,
        a_caload, a_castore, a_checkcast,
        a_d2f, a_d2i, a_d2l, a_dadd, a_daload, a_dastore, a_dcmpg, a_dcmpl,
        a_dconst_0, a_dconst_1, a_ddiv,
        a_dload, a_dload_0, a_dload_1, a_dload_2, a_dload_3,
        a_dmul, a_dneg, a_drem, a_dreturn,
        a_dstore, a_dstore_0, a_dstore_1, a_dstore_2, a_dstore_3,
        a_dsub,
        a_dup, a_dup2, a_dup2_x1, a_dup2_x2, a_dup_x1, a_dup_x2,
        a_f2d, a_f2i, a_f2l, a_fadd, a_faload, a_fastore, a_fcmpg, a_fcmpl,
        a_fconst_0, a_fconst_1, a_fconst_2, a_fdiv,
        a_fload, a_fload_0, a_fload_1, a_fload_2, a_fload_3,
        a_fmul, a_fneg, a_frem, a_freturn,
        a_fstore, a_fstore_0, a_fstore_1, a_fstore_2, a_fstore_3,
        a_fsub,
        a_getfield, a_getstatic,
        a_goto, a_goto_w,
        a_i2b, a_i2c, a_i2d, a_i2f, a_i2l, a_i2s,
        a_iadd, a_iaload, a_iand, a_iastore,
        a_iconst_m1, a_iconst_0, a_iconst_1, a_iconst_2, a_iconst_3,
        a_iconst_4, a_iconst_5,
        a_idiv,
        a_if_acmpeq, a_if_acmpne, a_if_icmpeq, a_if_icmpge, a_if_icmpgt,
        a_if_icmple, a_if_icmplt, a_if_icmpne,
        a_ifeq, a_ifge, a_ifgt, a_ifle, a_iflt, a_ifne, a_ifnonnull, a_ifnull,
        a_iinc,
        a_iload, a_iload_0, a_iload_1, a_iload_2, a_iload_3,
        a_imul, a_ineg,
        a_instanceof,
        a_invokeinterface, a_invokespecial, a_invokestatic, a_invokevirtual,
        a_ior, a_irem, a_ireturn, a_ishl, a_ishr,
        a_istore, a_istore_0, a_istore_1, a_istore_2, a_istore_3,
        a_isub, a_iushr, a_ixor,
        a_jsr, a_jsr_w,
        a_l2d, a_l2f, a_l2i, a_ladd, a_laload, a_land, a_lastore, a_lcmp,
        a_lconst_0, a_lconst_1,
        a_ldc, a_ldc2_w, a_ldc_w, a_ldiv,
        a_lload, a_lload_0, a_lload_1, a_lload_2, a_lload_3,
        a_lmul, a_lneg,
        a_lookupswitch,
        a_lor, a_lrem,
        a_lreturn,
        a_lshl, a_lshr,
        a_lstore, a_lstore_0, a_lstore_1, a_lstore_2, a_lstore_3,
        a_lsub, a_lushr, a_lxor,
        a_monitorenter,
        a_monitorexit,
        a_multianewarray,
        a_new,
        a_newarray,
        a_nop,
        a_pop, a_pop2,
        a_putfield, a_putstatic,
        a_ret, a_return,
        a_saload, a_sastore, a_sipush,
        a_swap,
        a_tableswitch,
        a_wide
      );

      {# This should define the array of instructions as string }
      op2strtable=array[tasmop] of string[8];

    Const
      {# First value of opcode enumeration }
      firstop = low(tasmop);
      {# Last value of opcode enumeration  }
      lastop  = high(tasmop);


{*****************************************************************************
                                  Registers
*****************************************************************************}

    type
      { Number of registers used for indexing in tables }
      tregisterindex=0..{$i rjvmnor.inc}-1;

    const
      { Available Superregisters }
      {$i rjvmsup.inc}

      { No Subregisters }
      R_SUBWHOLE = R_SUBNONE;

      { Available Registers }
      {$i rjvmcon.inc}

      { aliases }
      { used as base register in references for parameters passed to
        subroutines: these are passed on the evaluation stack, but this way we
        can use the offset field to indicate the order, which is used by ncal
        to sort the parameters }
      NR_EVAL_STACK_BASE = NR_R0;

      maxvarregs = 1;
      maxfpuvarregs = 1;

      { Integer Super registers first and last }
      first_int_imreg = 2;

      { Float Super register first and last }
      first_fpu_imreg     = 2;

      { MM Super register first and last }
      first_mm_imreg     = 2;

      regnumber_table : array[tregisterindex] of tregister = (
        {$i rjvmnum.inc}
      );

     EVALSTACKLOCS = [LOC_REGISTER,LOC_CREGISTER,LOC_FPUREGISTER,LOC_CFPUREGISTER,
       LOC_MMREGISTER,LOC_CMMREGISTER,LOC_SUBSETREG,LOC_CSUBSETREG];


{*****************************************************************************
                               References
*****************************************************************************}

   type
     { array reference types }
     tarrayreftype = (art_none,art_indexreg,art_indexref,art_indexconst);

{*****************************************************************************
                                Conditions
*****************************************************************************}

   type
     // not used by jvm target
     TAsmCond=(C_None);

{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      max_operands = 2;


{*****************************************************************************
                          Default generic sizes
*****************************************************************************}

{$ifdef cpu64bitaddr}
      {# Defines the default address size for a processor,
        -- fake for JVM, only influences default width of
           arithmetic calculations }
      OS_ADDR = OS_64;
      {# the natural int size for a processor,
         has to match osuinttype/ossinttype as initialized in psystem }
      OS_INT = OS_64;
      OS_SINT = OS_S64;
{$else}
      {# Defines the default address size for a processor,
        -- fake for JVM, only influences default width of
           arithmetic calculations }
      OS_ADDR = OS_32;
      {# the natural int size for a processor,
         has to match osuinttype/ossinttype as initialized in psystem }
      OS_INT = OS_32;
      OS_SINT = OS_S32;
{$endif}
      {# the maximum float size for a processor,           }
      OS_FLOAT = OS_F64;
      {# the size of a vector register for a processor     }
      OS_VECTOR = OS_M128;

{*****************************************************************************
                          Generic Register names
*****************************************************************************}

      { dummies, not used for JVM }

      {# Stack pointer register }
      { used as base register in references to indicate that it's a local }
      NR_STACK_POINTER_REG = NR_R1;
      RS_STACK_POINTER_REG = RS_R1;
      {# Frame pointer register }
      NR_FRAME_POINTER_REG = NR_STACK_POINTER_REG;
      RS_FRAME_POINTER_REG = RS_STACK_POINTER_REG;

      { Java results are returned on the evaluation stack, not via a register }

      { Results are returned in this register (32-bit values) }
      NR_FUNCTION_RETURN_REG = NR_NO;
      RS_FUNCTION_RETURN_REG = RS_NO;
      { Low part of 64bit return value }
      NR_FUNCTION_RETURN64_LOW_REG = NR_NO;
      RS_FUNCTION_RETURN64_LOW_REG = RS_NO;
      { High part of 64bit return value }
      NR_FUNCTION_RETURN64_HIGH_REG = NR_NO;
      RS_FUNCTION_RETURN64_HIGH_REG = RS_NO;
      { The value returned from a function is available in this register }
      NR_FUNCTION_RESULT_REG = NR_FUNCTION_RETURN_REG;
      RS_FUNCTION_RESULT_REG = RS_FUNCTION_RETURN_REG;
      { The lowh part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_LOW_REG = NR_FUNCTION_RETURN64_LOW_REG;
      RS_FUNCTION_RESULT64_LOW_REG = RS_FUNCTION_RETURN64_LOW_REG;
      { The high part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_HIGH_REG = NR_FUNCTION_RETURN64_HIGH_REG;
      RS_FUNCTION_RESULT64_HIGH_REG = RS_FUNCTION_RETURN64_HIGH_REG;

      NR_FPU_RESULT_REG = NR_NO;
      NR_MM_RESULT_REG = NR_NO;


{*****************************************************************************
                       GCC /ABI linking information
*****************************************************************************}

      { dummies, not used for JVM }

      {# Required parameter alignment when calling a routine
      }
      std_param_align = 1;


{*****************************************************************************
                            CPU Dependent Constants
*****************************************************************************}

      maxfpuregs = 0;

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
    function reg_cgsize(const reg: tregister) : tcgsize;

    function std_regnum_search(const s:string):Tregister;
    function std_regname(r:Tregister):string;
    function findreg_by_number(r:Tregister):tregisterindex;

    function eh_return_data_regno(nr: longint): longint;

    { since we don't use tasmconds, don't call this routine
      (it will internalerror). We need it anyway to get aoptobj
      to compile (but it won't execute it).
    }
    function inverse_cond(const c: TAsmCond): Tasmcond; {$ifdef USEINLINE}inline;{$endif USEINLINE}

implementation

uses
  verbose,
  rgbase;

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    const
      std_regname_table : array[tregisterindex] of string[15] = (
        {$i rjvmstd.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i rjvmrni.inc}
      );
(*
      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i rjvmsri.inc}
      );
*)
    function reg_cgsize(const reg: tregister): tcgsize;
      begin
        result:=OS_NO;
      end;


    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
      begin
        cgsize2subreg:=R_SUBNONE;
      end;


    function std_regnum_search(const s:string):Tregister;
      begin
        result:=NR_NO;
      end;


    function findreg_by_number(r:Tregister):tregisterindex;
      begin
        result:=findreg_by_number_table(r,regnumber_index);
      end;

    function std_regname(r:Tregister):string;
      var
        p : tregisterindex;
      begin
        p:=findreg_by_number_table(r,regnumber_index);
        if p<>0 then
          result:=std_regname_table[p]
        else
          result:=generic_regname(r);
      end;

    function eh_return_data_regno(nr: longint): longint;
      begin
        result:=-1;
      end;

    function inverse_cond(const c: TAsmCond): Tasmcond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      begin
        result:=C_None;
        internalerror(2015082701);
      end;

end.
