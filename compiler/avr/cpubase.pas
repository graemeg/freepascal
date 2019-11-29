{
    Copyright (c) 2006 by Florian Klaempfl

    Contains the base types for the AVR

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
      TAsmOp=(A_None,
        A_ADD,A_ADC,A_ADIW,A_SUB,A_SUBI,A_SBC,A_SBCI,A_SBRC,A_SBRS,A_SBIW,A_AND,A_ANDI,
        A_OR,A_ORI,A_EOR,A_COM,A_NEG,A_SBR,A_CBR,A_INC,A_DEC,A_TST,
        A_MUL,A_MULS,A_MULSU,A_FMUL,A_FMULS,A_FMULSU,A_RJMP,A_IJMP,
        A_EIJMP,A_JMP,A_RCALL,A_ICALL,R_EICALL,A_CALL,A_RET,A_RETI,A_CPSE,
        A_CP,A_CPC,A_CPI,A_SBIC,A_SBIS,A_BRxx,A_MOV,A_MOVW,A_LDI,A_LDS,A_LD,A_LDD,
        A_STS,A_ST,A_STD,A_LPM,A_ELPM,A_SPM,A_IN,A_OUT,A_PUSH,A_POP,
        A_LSL,A_LSR,A_ROL,A_ROR,A_ASR,A_SWAP,A_BSET,A_BCLR,A_SBI,A_CBI,
        A_SEC,A_SEH,A_SEI,A_SEN,A_SER,A_SES,A_SET,A_SEV,A_SEZ,
        A_CLC,A_CLH,A_CLI,A_CLN,A_CLR,A_CLS,A_CLT,A_CLV,A_CLZ,
        A_BST,A_BLD,A_BREAK,A_NOP,A_SLEEP,A_WDR,A_XCH);


      { This should define the array of instructions as string }
      op2strtable=array[tasmop] of string[11];

    const
      { First value of opcode enumeration }
      firstop = low(tasmop);
      { Last value of opcode enumeration  }
      lastop  = high(tasmop);

      { call/reg instructions (A_RCALL,A_ICALL,A_CALL,A_RET,A_RETI) are not considered as jmp instructions for the usage cases of
        this set }
      jmp_instructions = [A_BRxx,A_SBIC,A_SBIS,A_JMP,A_EIJMP,A_RJMP,A_CPSE,A_IJMP];
      call_jmp_instructions = [A_ICALL,A_RCALL,A_CALL,A_RET,A_RETI]+jmp_instructions;

{*****************************************************************************
                                  Registers
*****************************************************************************}

    type
      { Number of registers used for indexing in tables }
      tregisterindex=0..{$i ravrnor.inc}-1;

    const
      { Available Superregisters }
      {$i ravrsup.inc}

      { No Subregisters }
      R_SUBWHOLE = R_SUBNONE;

      { Available Registers }
      {$i ravrcon.inc}

      NR_XLO = NR_R26;
      NR_XHI = NR_R27;
      NR_YLO = NR_R28;
      NR_YHI = NR_R29;
      NR_ZLO = NR_R30;
      NR_ZHI = NR_R31;

      NIO_SREG = $3f;
      NIO_SP_LO = $3d;
      NIO_SP_HI = $3e;

      { Integer Super registers first and last }
      first_int_supreg = RS_R0;
      first_int_imreg = $20;

      { Float Super register first and last }
      first_fpu_supreg    = RS_INVALID;
      first_fpu_imreg     = 0;

      { MM Super register first and last }
      first_mm_supreg    = RS_INVALID;
      first_mm_imreg     = 0;

      regnumber_count_bsstart = 32;

      regnumber_table : array[tregisterindex] of tregister = (
        {$i ravrnum.inc}
      );

      regstabs_table : array[tregisterindex] of shortint = (
        {$i ravrsta.inc}
      );

      regdwarf_table : array[tregisterindex] of shortint = (
        {$i ravrdwa.inc}
      );
      { registers which may be destroyed by calls }
      VOLATILE_INTREGISTERS = [RS_R0,RS_R1,RS_R18..RS_R27,RS_R30,RS_R31];
      VOLATILE_FPUREGISTERS = [];

{*****************************************************************************
                                Conditions
*****************************************************************************}

    type
      TAsmCond=(C_None,
        C_CC,C_CS,C_EQ,C_GE,C_HC,C_HS,C_ID,C_IE,C_LO,C_LT,
        C_MI,C_NE,C_PL,C_SH,C_TC,C_TS,C_VC,C_VS
      );

    const
      cond2str : array[TAsmCond] of string[2]=('',
        'cc','cs','eq','ge','hc','hs','id','ie','lo','lt',
        'mi','ne','pl','sh','tc','ts','vc','vs'
      );

      uppercond2str : array[TAsmCond] of string[2]=('',
        'CC','CS','EQ','GE','HC','HS','ID','IE','LO','LT',
        'MI','NE','PL','SH','TC','TS','VC','VS'
      );

{*****************************************************************************
                                   Flags
*****************************************************************************}

    type
      TResFlags = (F_NotPossible,F_CC,F_CS,F_EQ,F_GE,F_LO,F_LT,
        F_NE,F_SH,F_VC,F_VS,F_PL,F_MI);

{*****************************************************************************
                                Operands
*****************************************************************************}

      taddressmode = (AM_UNCHANGED,AM_POSTINCREMENT,AM_PREDRECEMENT);

{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      max_operands = 2;

      maxintregs = 15;
      maxfpuregs = 0;
      maxaddrregs = 0;

{*****************************************************************************
                                Operand Sizes
*****************************************************************************}

    type
      topsize = (S_NO,
        S_B,S_W,S_L,S_BW,S_BL,S_WL,
        S_IS,S_IL,S_IQ,
        S_FS,S_FL,S_FX,S_D,S_Q,S_FV,S_FXX
      );

{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      firstsaveintreg = RS_R4;
      lastsaveintreg  = RS_R10;
      firstsavefpureg = RS_INVALID;
      lastsavefpureg  = RS_INVALID;
      firstsavemmreg  = RS_INVALID;
      lastsavemmreg   = RS_INVALID;

      maxvarregs = 7;
      varregs : Array [1..maxvarregs] of tsuperregister =
                (RS_R4,RS_R5,RS_R6,RS_R7,RS_R8,RS_R9,RS_R10);

      maxfpuvarregs = 1;
      fpuvarregs : Array [1..maxfpuvarregs] of tsuperregister =
                (RS_INVALID);

{*****************************************************************************
                          Default generic sizes
*****************************************************************************}

      { Defines the default address size for a processor, }
      OS_ADDR = OS_16;
      { the natural int size for a processor,
        has to match osuinttype/ossinttype as initialized in psystem,
        initially, this was OS_16/OS_S16 on avr, but experience has
        proven that it is better to make it 8 Bit thus having the same
        size as a register.
      }
      OS_INT = OS_8;
      OS_SINT = OS_S8;
      { the maximum float size for a processor,           }
      OS_FLOAT = OS_F64;
      { the size of a vector register for a processor     }
      OS_VECTOR = OS_M32;

{*****************************************************************************
                          Generic Register names
*****************************************************************************}

      { Stack pointer register }
      NR_STACK_POINTER_REG = NR_R13;
      RS_STACK_POINTER_REG = RS_R13;
      { Frame pointer register }
      RS_FRAME_POINTER_REG = RS_R28;
      NR_FRAME_POINTER_REG = NR_R28;
      { Register for addressing absolute data in a position independant way,
        such as in PIC code. The exact meaning is ABI specific. For
        further information look at GCC source : PIC_OFFSET_TABLE_REGNUM
      }
      NR_PIC_OFFSET_REG = NR_R9;
      { Results are returned in this register (32-bit values) }
      NR_FUNCTION_RETURN_REG = NR_R24;
      RS_FUNCTION_RETURN_REG = RS_R24;
      { Low part of 64bit return value }
      NR_FUNCTION_RETURN64_LOW_REG = NR_R22;
      RS_FUNCTION_RETURN64_LOW_REG = RS_R22;
      { High part of 64bit return value }
      NR_FUNCTION_RETURN64_HIGH_REG = NR_R1;
      RS_FUNCTION_RETURN64_HIGH_REG = RS_R1;
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

      NR_MM_RESULT_REG  = NR_NO;

      NR_RETURN_ADDRESS_REG = NR_FUNCTION_RETURN_REG;

      { Offset where the parent framepointer is pushed }
      PARENT_FRAMEPOINTER_OFFSET = 0;

      NR_DEFAULTFLAGS = NR_SREG;
      RS_DEFAULTFLAGS = RS_SREG;

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
                                  Helpers
*****************************************************************************}

    { Returns the tcgsize corresponding with the size of reg.}
    function reg_cgsize(const reg: tregister) : tcgsize;
    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
    procedure inverse_flags(var f: TResFlags);
    function flags_to_cond(const f: TResFlags) : TAsmCond;
    function findreg_by_number(r:Tregister):tregisterindex;
    function std_regnum_search(const s:string):Tregister;
    function std_regname(r:Tregister):string;

    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    function conditions_equal(const c1, c2: TAsmCond): boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}

    { Checks if Subset is a subset of c (e.g. "less than" is a subset of "less than or equal" }
    function condition_in(const Subset, c: TAsmCond): Boolean;

    function dwarf_reg(r:tregister):byte;
    function dwarf_reg_no_error(r:tregister):shortint;
    function eh_return_data_regno(nr: longint): longint;


    function is_calljmp(o:tasmop):boolean;{$ifdef USEINLINE}inline;{$endif USEINLINE}

  implementation

    uses
      rgBase,verbose;


    const
      std_regname_table : TRegNameTable = (
        {$i ravrstd.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i ravrrni.inc}
      );

      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i ravrsri.inc}
      );


    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
      begin
        cgsize2subreg:=R_SUBWHOLE;
      end;


    function reg_cgsize(const reg: tregister): tcgsize;
      begin
        case getregtype(reg) of
          R_INTREGISTER :
            reg_cgsize:=OS_8;
          R_ADDRESSREGISTER :
            reg_cgsize:=OS_16;
          else
            internalerror(2011021905);
          end;
        end;


    procedure inverse_flags(var f: TResFlags);
      const
        inv_flags: array[TResFlags] of TResFlags =
          (F_NotPossible,F_CS,F_CC,F_NE,F_LT,F_SH,F_GE,
           F_NE,F_LO,F_VS,F_VC,F_MI,F_PL);
      begin
        f:=inv_flags[f];
      end;


    function flags_to_cond(const f: TResFlags) : TAsmCond;
      const
        flag_2_cond: array[F_CC..F_MI] of TAsmCond =
          (C_CC,C_CS,C_EQ,C_GE,C_LO,C_LT,
           C_NE,C_SH,C_VC,C_VS,C_PL,C_MI);
      begin
        if f=F_NotPossible then
          internalerror(2011022101);
        if f>high(flag_2_cond) then
          internalerror(200112301);
        result:=flag_2_cond[f];
      end;


    function findreg_by_number(r:Tregister):tregisterindex;
      begin
        result:=rgBase.findreg_by_number_table(r,regnumber_index);
      end;


    function std_regnum_search(const s:string):Tregister;
      begin
        result:=regnumber_table[findreg_by_name_table(s,std_regname_table,std_regname_index)];
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


    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      const
        inverse: array[TAsmCond] of TAsmCond=(C_None,
          C_CS,C_CC,C_NE,C_LT,C_HS,C_HC,C_IE,C_ID,C_SH,C_GE,
          C_PL,C_EQ,C_MI,C_LO,C_TS,C_TC,C_VS,C_VC);
      begin
        result := inverse[c];
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
              Result := (c in [C_GE]);
            C_LT:
              Result := (c in [C_NE]);
            else
              Result := False;
          end;
      end;


    function rotl(d : dword;b : byte) : dword;
      begin
         result:=(d shr (32-b)) or (d shl b);
      end;


    function dwarf_reg(r:tregister):byte;
      var
        reg : shortint;
      begin
        reg:=regdwarf_table[findreg_by_number(r)];
        if reg=-1 then
          internalerror(200603251);
        result:=reg;
      end;

    function dwarf_reg_no_error(r:tregister):shortint;
      begin
        result:=regdwarf_table[findreg_by_number(r)];
      end;

    function eh_return_data_regno(nr: longint): longint;
      begin
        result:=-1;
      end;


    function is_calljmp(o:tasmop):boolean;{$ifdef USEINLINE}inline;{$endif USEINLINE}
      begin
        is_calljmp:= o in call_jmp_instructions;
      end;


end.
