{
    Copyright (c) 2016-2017 by Karoly Balogh

    Contains the base types for the WebAssembly

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
      // control flow
      a_block, a_loop, a_br, a_br_if, a_br_table, a_if, a_else, a_end_block,
      a_end_loop, a_end_if, a_end_function, a_return, a_unreachable,
      // basic
      a_nop, a_drop, a_i32_const, a_i64_const, a_f32_const, a_f64_const,
      a_local_get, a_local_set, a_local_tee, a_global_get, a_global_set,
      a_select, a_call, a_call_indirect,
      // integer
      a_i32_add, a_i64_add, a_i32_sub, a_i64_sub, a_i32_mul, a_i64_mul,
      a_i32_div_s, a_i64_div_s, a_i32_div_u, a_i64_div_u, a_i32_rem_s, a_i64_rem_s,
      a_i32_rem_u, a_i64_rem_u, a_i32_and, a_i64_and, a_i32_or, a_i64_or,
      a_i32_xor, a_i64_xor, a_i32_shl, a_i64_shl, a_i32_shr_s, a_i64_shr_s,
      a_i32_shr_u, a_i64_shr_u, a_i32_rotl, a_i64_rotl, a_i32_rotr, a_i64_rotr,
      a_i32_clz, a_i64_clz, a_i32_ctz, a_i64_ctz, a_i32_popcnt, a_i64_popcnt,
      a_i32_eqz, a_i64_eqz,
      // floating point
      a_f32_add, a_f64_add, a_f32_sub, a_f64_sub, a_f32_mul, a_f64_mul,
      a_f32_div, a_f64_div, a_f32_sqrt, a_f64_sqrt, a_f32_min, a_f64_min,
      a_f32_max, a_f64_max, a_f32_ceil, a_f64_ceil, a_f32_floor, a_f64_floor,
      a_f32_trunc, a_f64_trunc, a_f32_nearest, a_f64_nearest, a_f32_abs, a_f64_abs,
      a_f32_neg, a_f64_neg, a_f32_copysign, a_f64_copysign,
      // integer compare
      a_i32_eq, a_i64_eq, a_i32_ne, a_i64_ne, a_i32_lt_s, a_i64_lt_s,
      a_i32_lt_u, a_i64_lt_u, a_i32_le_s, a_i64_le_s, a_i32_le_u, a_i64_le_u,
      a_i32_gt_s, a_i64_gt_s, a_i32_gt_u, a_i64_gt_u, a_i32_ge_s, a_i64_ge_s,
      a_i32_ge_u, a_i64_ge_u,
      // floating point compare
      a_f32_eq, a_f64_eq, a_f32_ne, a_f64_ne, a_f32_lt, a_f64_lt,
      a_f32_le, a_f64_le, a_f32_gt, a_f64_gt, a_f32_ge, a_f64_ge,
      // conversion
      a_i32_wrap_i64, a_i64_extend_i32_s, a_i64_extend_i32_u,
      a_i32_extend8_s,a_i32_extend16_s,a_i64_extend8_s,a_i64_extend16_s,a_i64_extend32_s,
      a_i32_trunc_f32_s, a_i32_trunc_f64_s, a_i64_trunc_f32_s, a_i64_trunc_f64_s,
      a_i32_trunc_f32_u, a_i32_trunc_f64_u, a_i64_trunc_f32_u, a_i64_trunc_f64_u,
      a_f32_demote_f64, a_f64_promote_f32,
      a_f32_convert_i32_s, a_f32_convert_i64_s,a_f64_convert_i32_s,a_f64_convert_i64_s,
      a_f32_convert_i32_u, a_f32_convert_i64_u,a_f64_convert_i32_u,a_f64_convert_i64_u,
      a_i32_reinterpret_f32, a_i64_reinterpret_f64, a_f32_reinterpret_i32, a_f64_reinterpret_i64,
      // load/store
      a_i32_load, a_i64_load, a_f32_load, a_f64_load,
      a_i32_store, a_i64_store, a_f32_store, a_f64_store,
      a_i32_load8_s, a_i32_load16_s, a_i64_load8_s, a_i64_load16_s, a_i64_load32_s,
      a_i32_load8_u, a_i32_load16_u, a_i64_load8_u, a_i64_load16_u, a_i64_load32_u,
      a_i32_store8, a_i32_store16, a_i64_store8, a_i64_store16, a_i64_store32,
      // additional memory
      a_memory_grow, a_memory_size
      );

      TWasmBasicType = (wbt_i32, wbt_i64, wbt_f32, wbt_f64);
      TWasmResultType = array of TWasmBasicType;

      { TWasmFuncType }

      TWasmFuncType = class
        params: TWasmResultType;
        results: TWasmResultType;
        constructor Create(aparams, aresults: TWasmResultType);
        procedure add_param(param: TWasmBasicType);
        procedure add_result(res: TWasmBasicType);
      end;

      {# This should define the array of instructions as string }
      op2strtable=array[tasmop] of string[31];

    Const
      {# First value of opcode enumeration }
      firstop = low(tasmop);
      {# Last value of opcode enumeration  }
      lastop  = high(tasmop);

      AsmOp_Store = [
         a_i32_store, a_i32_store16, a_i32_store8
        ,a_i64_store, a_i64_store16, a_i64_store8, a_i64_store32
        ,a_f32_store, a_f64_store
      ];

      AsmOp_Load = [
         a_i32_load,
         a_i32_load8_s,  a_i32_load8_u,
         a_i32_load16_s, a_i32_load16_u,
         a_i64_load,
         a_i64_load8_s,  a_i64_load8_u,
         a_i64_load16_s, a_i64_load16_u,
         a_i64_load32_s, a_i64_load32_u,
         a_f32_load, a_f64_load
      ];

      AsmOp_LoadStore = AsmOp_Load + AsmOp_Store;


{*****************************************************************************
                                  Registers
*****************************************************************************}

    type
      { Number of registers used for indexing in tables }
      tregisterindex=0..{$i rwasmnor.inc}-1; // no registers in wasm
      totherregisterset = set of tregisterindex;

    const
      { Available Superregisters }
      // there's no registers in wasm
      {$i rwasmsup.inc}

      { No Subregisters }
      R_SUBWHOLE = R_SUBNONE;

      { Available Registers }
      // there's no registers in wasm
      {$i rwasmcon.inc}

      { aliases }
      { used as base register in references for parameters passed to
        subroutines: these are passed on the evaluation stack, but this way we
        can use the offset field to indicate the order, which is used by ncal
        to sort the parameters }
      NR_EVAL_STACK_BASE = NR_R0;
      RS_EVAL_STACK_BASE = RS_R0;
      { used as base register in references to indicate that it's a local }
      NR_LOCAL_STACK_POINTER_REG = NR_R1;
      RS_LOCAL_STACK_POINTER_REG = RS_R1;
      { fake register, representing the local frame pointer. Used for accessing
        address-taken local variables on the linear stack: (localframeptr+offset). }
      NR_LOCAL_FRAME_POINTER_REG = NR_R3;
      RS_LOCAL_FRAME_POINTER_REG = RS_R3;

      maxvarregs = 1;
      maxfpuvarregs = 1;

      { Integer Super registers first and last }
      first_int_imreg = 4;

      { Float Super register first and last }
      first_fpu_imreg     = 4;

      { MM Super register first and last }
      first_mm_imreg     = 4;

      regnumber_table : array[tregisterindex] of tregister = (
        {$i rwasmnum.inc}
      );

     EVALSTACKLOCS = [LOC_REGISTER,LOC_CREGISTER,LOC_FPUREGISTER,LOC_CFPUREGISTER,
       LOC_MMREGISTER,LOC_CMMREGISTER,LOC_SUBSETREG,LOC_CSUBSETREG];


{*****************************************************************************
                                Conditions
*****************************************************************************}

   type
     // not used by wasm target
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
        -- fake for wasm, only influences default width of
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

      { dummies, not used for Wasm }

      {# Stack pointer register }
      { used as base register in references to indicate that it's a local }
      NR_STACK_POINTER_REG = NR_R1;
      RS_STACK_POINTER_REG = RS_R1;
      {# Frame pointer register }
      NR_FRAME_POINTER_REG = NR_LOCAL_FRAME_POINTER_REG;
      RS_FRAME_POINTER_REG = RS_LOCAL_FRAME_POINTER_REG;

      { WebAssembly results are returned on the evaluation stack, not via a register }

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

      { dummies, not used for Wasm }

      {# Required parameter alignment when calling a routine
      }
      std_param_align = 1;


{*****************************************************************************
                            CPU Dependent Constants
*****************************************************************************}

      maxfpuregs = 0;

      { Global variable, that acts as the stack pointer in linear memory
        (also called the "linear stack"). This stack is used for address-taken
        local variables. This separate stack is needed, because the WASM
        implementation's runtime call stack (which includes return addresses and
        function parameters) is not visible in linear memory. }
      STACK_POINTER_SYM = '__stack_pointer';

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
    function reg_cgsize(const reg: tregister) : tcgsize;

    function std_regnum_search(const s:string):Tregister;
    function std_regname(r:Tregister):string;
    function findreg_by_number(r:Tregister):tregisterindex;

    function dwarf_reg(r:tregister):byte;
    function dwarf_reg_no_error(r:tregister):shortint;
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
        {$i rwasmstd.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i rwasmrni.inc}
      );

      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i rwasmsri.inc}
      );

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

    function dwarf_reg(r:tregister):byte;
      begin
        result:=0;
        internalerror(200603251);
      end;

    function dwarf_reg_no_error(r:tregister):shortint;
      begin
        result:=-1;
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

{*****************************************************************************
                                  TWasmFuncType
*****************************************************************************}

    constructor TWasmFuncType.Create(aparams, aresults: TWasmResultType);
      begin
        inherited Create;
        params:=aparams;
        results:=aresults;
      end;

    procedure TWasmFuncType.add_param(param: TWasmBasicType);
      begin
        SetLength(params,Length(params)+1);
        params[High(params)]:=param;
      end;

    procedure TWasmFuncType.add_result(res: TWasmBasicType);
      begin
        SetLength(results,Length(results)+1);
        results[High(results)]:=res;
      end;

end.
