{
    Copyright (c) 1998-2002 by Florian Klaempfl and Peter Vreman

    Contains the base types for MIPS

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
{# Base unit for processor information. This unit contains
   enumerations of registers, opcodes, sizes, and other
   such things which are processor specific.
}
unit cpubase;

{$i fpcdefs.inc}

  interface

    uses
      cutils,cclasses,
      globtype,globals,
      cpuinfo,
      aasmbase,
      cgbase
      ;


{*****************************************************************************
                                Assembler Opcodes
*****************************************************************************}

    type
      TAsmOp=({$i opcode.inc});

      { This should define the array of instructions as string }
      op2strtable=array[tasmop] of string[11];

    const
      { First value of opcode enumeration }
      firstop = low(tasmop);
      { Last value of opcode enumeration  }
      lastop  = high(tasmop);

{*****************************************************************************
                                  Registers
*****************************************************************************}

    type
      { Number of registers used for indexing in tables }
      tregisterindex=0..{$i rmipsnor.inc}-1;

    const
      { Available Superregisters }
      {$i rmipssup.inc}

      { No Subregisters }
      R_SUBWHOLE = R_SUBD;

      { Available Registers }
      {$i rmipscon.inc}

      { Integer Super registers first and last }
      first_int_supreg = RS_R0;
      first_int_imreg = $20;

      { Float Super register first and last }
      first_fpu_supreg    = RS_F0;
      first_fpu_imreg     = $20;

      { MM Super register first and last }
      first_mm_supreg    = 0;
      first_mm_imreg     = 1;

{ TODO: Calculate bsstart}
      regnumber_count_bsstart = 64;

      regnumber_table : array[tregisterindex] of tregister = (
        {$i rmipsnum.inc}
      );

      regstabs_table : array[tregisterindex] of shortint = (
        {$i rmipssta.inc}
      );

      regdwarf_table : array[tregisterindex] of shortint = (
        {$i rmipsdwf.inc}
      );
      { registers which may be destroyed by calls }
      VOLATILE_INTREGISTERS = [RS_R0..RS_R3,RS_R12..RS_R15];
      VOLATILE_FPUREGISTERS = [RS_F0..RS_F3];

{*****************************************************************************
                                Conditions
*****************************************************************************}

    type
      TAsmCond=(C_None,
        C_EQ, C_NE, C_LT, C_LE, C_GT, C_GE, C_LTU, C_LEU, C_GTU, C_GEU,
        C_LTZ, C_LEZ, C_GTZ, C_GEZ,
        C_COP1TRUE,
        C_COP1FALSE
      );

    const
      cond2str : array[TAsmCond] of string[3]=('',
        'eq','ne','lt','le','gt','ge','ltu','leu','gtu','geu',
        'ltz','lez','gtz','gez',
        'c1t','c1f'
      );

    type
      TResFlags=record
        reg1: TRegister;
        cond: TOpCmp;
      case use_const: boolean of
        False: (reg2: TRegister);
        True: (value: aint);
      end;

{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      max_operands = 4;

      maxintregs = 31;
      maxfpuregs = 8;
      maxaddrregs = 0;

{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      maxvarregs = 7;
      varregs : Array [1..maxvarregs] of tsuperregister =
                (RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,RS_R9,RS_R10);

      maxfpuvarregs = 4;
      fpuvarregs : Array [1..maxfpuvarregs] of tsuperregister =
                (RS_F4,RS_F5,RS_F6,RS_F7);

{*****************************************************************************
                          Default generic sizes
*****************************************************************************}

      { Defines the default address size for a processor, }
      OS_ADDR = OS_32;
      {# the natural int size for a processor,
         has to match osuinttype/ossinttype as initialized in psystem }
      OS_INT = OS_32;
      OS_SINT = OS_S32;
      { the maximum float size for a processor,           }
      OS_FLOAT = OS_F64;
      { the size of a vector register for a processor     }
      OS_VECTOR = OS_M32;

{*****************************************************************************
                          Generic Register names
*****************************************************************************}


      { PIC Code }
      NR_GP = NR_R28;
      NR_PIC_FUNC = NR_R25;
      RS_GP = RS_R28;
      RS_PIC_FUNC = RS_R25;

      { VMT code }
      NR_VMT = NR_R24;
      RS_VMT = RS_R24;

      NR_SP = NR_R29;
      NR_S8 = NR_R30;
      NR_FP = NR_R30;
      NR_RA = NR_R31;

      RS_SP = RS_R29;
      RS_S8 = RS_R30;
      RS_FP = RS_R30;
      RS_RA = RS_R31;

      {# Stack pointer register }
      NR_STACK_POINTER_REG = NR_SP;
      RS_STACK_POINTER_REG = RS_SP;
      {# Frame pointer register }
      NR_FRAME_POINTER_REG = NR_FP;
      RS_FRAME_POINTER_REG = RS_FP;

      NR_RETURN_ADDRESS_REG = NR_R7;
      { the return_result_reg, is used inside the called function to store its return
      value when that is a scalar value otherwise a pointer to the address of the
      result is placed inside it }

      { Results are returned in this register (32-bit values) }
      NR_FUNCTION_RETURN_REG = NR_R2;
      RS_FUNCTION_RETURN_REG = RS_R2;
      { Low part of 64bit return value }
      NR_FUNCTION_RETURN64_LOW_REG = NR_R2;
      RS_FUNCTION_RETURN64_LOW_REG = RS_R2;
      { High part of 64bit return value }
      NR_FUNCTION_RETURN64_HIGH_REG = NR_R3;
      RS_FUNCTION_RETURN64_HIGH_REG = RS_R3;
      { The value returned from a function is available in this register }
      NR_FUNCTION_RESULT_REG = NR_R2;
      RS_FUNCTION_RESULT_REG = RS_R2;
      { The lowh part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_LOW_REG = NR_R2;
      RS_FUNCTION_RESULT64_LOW_REG = RS_R2;
      { The high part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_HIGH_REG = NR_R3;
      RS_FUNCTION_RESULT64_HIGH_REG = RS_R3;

      NR_FPU_RESULT_REG = NR_F0;
      NR_MM_RESULT_REG  = NR_NO;

      NR_DEFAULTFLAGS = NR_NO;

{*****************************************************************************
                       GCC /ABI linking information
*****************************************************************************}

    const
      { Required parameter alignment when calling a routine declared as
        stdcall and cdecl. The alignment value should be the one defined
        by GCC or the target ABI.

        The value of this constant is equal to the constant
        PARM_BOUNDARY / BITS_PER_UNIT in the GCC source.
      }
      std_param_align = 4;

{*****************************************************************************
                            CPU Dependent Constants
*****************************************************************************}

    const
      simm16lo =  -32768;
      simm16hi =   32767;

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    function conditions_equal(const c1, c2: TAsmCond): boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}

    { Checks if Subset is a subset of c (e.g. "less than" is a subset of "less than or equal" }
    function condition_in(const Subset, c: TAsmCond): Boolean;

    { Returns the tcgsize corresponding with the size of reg.}
    function reg_cgsize(const reg: tregister) : tcgsize;
    function cgsize2subreg(regtype: tregistertype; s:tcgsize):tsubregister;
    function is_calljmp(o:tasmop):boolean;
    function findreg_by_number(r:Tregister):tregisterindex;
    function std_regnum_search(const s:string):Tregister;
    function std_regname(r:Tregister):string;
    function dwarf_reg(r:tregister):shortint;
    function dwarf_reg_no_error(r:tregister):shortint;
    function eh_return_data_regno(nr: longint): longint;

  implementation

    uses
      rgBase,verbose;


    const
      std_regname_table : TRegNameTable = (
        {$i rmipsstd.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i rmipsrni.inc}
      );

      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i rmipssri.inc}
      );


    function cgsize2subreg(regtype: tregistertype; s:tcgsize):tsubregister;
      begin
        case regtype of
          R_FPUREGISTER:
            if s=OS_F32 then
              result:=R_SUBFS
            else if s=OS_F64 then
              result:=R_SUBFD
            else
              internalerror(2013021301);
        else
          result:=R_SUBWHOLE;
        end;
      end;


    function reg_cgsize(const reg: tregister): tcgsize;
      begin
        case getregtype(reg) of
          R_INTREGISTER :
            reg_cgsize:=OS_32;
          R_FPUREGISTER :
            begin
              if getsubreg(reg)=R_SUBFD then
                result:=OS_F64
              else
                result:=OS_F32;
            end;
          else
            internalerror(200303181);
          end;
        end;


    function is_calljmp(o:tasmop):boolean;
      begin
        is_calljmp:= o in [A_J,A_JAL,A_JALR,{ A_JALX, }A_JR, A_BA, A_BC];
      end;


    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      const
        inverse: array[TAsmCond] of TAsmCond=(C_None,
        C_NE, C_EQ, C_GE, C_GT, C_LE, C_LT, C_GEU, C_GTU, C_LEU, C_LTU,
        C_GEZ, C_GTZ, C_LEZ, C_LTZ,
        C_COP1FALSE,
        C_COP1TRUE
        );
      begin
        result := inverse[c];
      end;      
      function findreg_by_number(r:Tregister):tregisterindex;
      begin
        { the register table for MIPS cpu only contains 
          R_SUBFS and R_SUBD register types.
          This function is called by dbgstabs unit,
          here were are only interested in register,
          not its subtype, thus we change subreg to
          R_SUBFS or R_SUBD.  }
        case getsubreg(r) of
          R_SUBFD:
            setsubreg(r, R_SUBFS);
          R_SUBL, R_SUBW, R_SUBD, R_SUBQ:
            setsubreg(r, R_SUBD);
          else
            ;
        end;
        result:=rgBase.findreg_by_number_table(r,regnumber_index);
      end;


    function conditions_equal(const c1, c2: TAsmCond): boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      begin
        result := c1 = c2;
      end;


    { Checks if Subset is a subset of c (e.g. "less than" is a subset of "less than or equal" }
    function condition_in(const Subset, c: TAsmCond): Boolean;
      begin
        Result := (c = C_None) or conditions_equal(Subset, c);

        { Please update as necessary. [Kit] }
        if not Result then
          case Subset of
            C_EQ:
              Result := (c in [C_GE, C_LE, C_GEU, C_LEU]);
            C_LT:
              Result := (c in [C_LE]);
            C_LTU:
              Result := (c in [C_LEU]);
            C_GT:
              Result := (c in [C_GE]);
            C_GTU:
              Result := (c in [C_GEU]);
            else
              Result := False;
          end;
      end;


    function std_regnum_search(const s:string):Tregister;
      begin
        result:=regnumber_table[findreg_by_name_table(s,std_regname_table,std_regname_index)];
      end;


    function std_regname(r:Tregister):string;
      var
        p : tregisterindex;
        hr : tregister;
      begin
        hr:=r;
        case getsubreg(hr) of
          R_SUBFD:
            setsubreg(hr, R_SUBFS);
          R_SUBL, R_SUBW, R_SUBD, R_SUBQ:
            setsubreg(hr, R_SUBD);
          else
            ;
        end;
        p:=findreg_by_number_table(hr,regnumber_index);
        if p<>0 then
          result:=std_regname_table[p]
        else if getregtype(r)=R_SPECIALREGISTER then
          result:=tostr(getsupreg(r))
        else
          result:=generic_regname(r);
      end;

    function dwarf_reg(r:tregister):shortint;
      begin
        case getsubreg(r) of
          R_SUBFD:
            setsubreg(r, R_SUBFS);
          R_SUBL, R_SUBW, R_SUBD, R_SUBQ:
            setsubreg(r, R_SUBD);
          else
            ;
        end;
        result:=regdwarf_table[findreg_by_number(r)];
        if result=-1 then
          internalerror(200603251);
      end;

    function dwarf_reg_no_error(r:tregister):shortint;
      begin
        case getsubreg(r) of
          R_SUBFD:
            setsubreg(r, R_SUBFS);
          R_SUBL, R_SUBW, R_SUBD, R_SUBQ:
            setsubreg(r, R_SUBD);
          else
            ;
        end;
        result:=regdwarf_table[findreg_by_number(r)];
      end;

    function eh_return_data_regno(nr: longint): longint;
      begin
        if (nr>=0) and (nr<2) then
          result:=nr+4
        else
          result:=-1;
      end;


begin
end.
