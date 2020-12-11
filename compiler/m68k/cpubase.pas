{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Contains the base types for the m68k

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
{ This Unit contains the base types for the m68k
}
unit cpubase;

{$i fpcdefs.inc}

  interface

  uses
    globtype,globals,
    strings,cutils,cclasses,aasmbase,cpuinfo,cgbase;

{*****************************************************************************
                                Assembler Opcodes
*****************************************************************************}

    type
      tasmop = {$i m68kop.inc}

      {# This should define the array of instructions as string }
      op2strtable=array[tasmop] of string[11];

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
      tregisterindex=0..{$i r68knor.inc}-1;

    const
      { Available Superregisters }
      {$i r68ksup.inc}
      RS_SP = RS_A7;

      R_SUBWHOLE = R_SUBD;

      { Available Registers }
      {$i r68kcon.inc}
      NR_SP = NR_A7;

      { Integer Super registers first and last }
      first_int_imreg = 8;

      { Float Super register first and last }
      first_fpu_imreg     = 8;

      { Integer Super registers first and last }
      first_addr_imreg = 8;

      { MM Super register first and last }
      first_mm_supreg    = 0;
      first_mm_imreg     = 0;

      maxfpuregs = 8;

      { include regnumber_count_bsstart }
      {$i r68kbss.inc}

      regnumber_table : array[tregisterindex] of tregister = (
        {$i r68knum.inc}
      );

      regstabs_table : array[tregisterindex] of shortint = (
        {$i r68ksta.inc}
      );

      regdwarf_table : array[tregisterindex] of shortint = (
{ TODO: reused stabs values!}
        {$i r68ksta.inc}
      );

      { registers which may be destroyed by calls }
      VOLATILE_INTREGISTERS = [RS_D0,RS_D1];
      VOLATILE_FPUREGISTERS = [RS_FP0,RS_FP1];
      VOLATILE_ADDRESSREGISTERS = [RS_A0,RS_A1];

{*****************************************************************************
                                Conditions
*****************************************************************************}

    type
      TAsmCond=(C_None,
         C_CC,C_LS,C_CS,C_LT,C_EQ,C_MI,C_F,C_NE,
         C_GE,C_PL,C_GT,C_T,C_HI,C_VC,C_LE,C_VS
      );


    const
      cond2str:array[TAsmCond] of string[3]=('',
        'cc','ls','cs','lt','eq','mi','f','ne',
        'ge','pl','gt','t','hi','vc','le','vs'
      );

{*****************************************************************************
                                   Flags
*****************************************************************************}

    type
      TResFlags = (
          F_E,F_NE,
          F_G,F_L,F_GE,F_LE,F_C,F_NC,F_A,F_AE,F_B,F_BE,
          F_FE,F_FNE,
          F_FG,F_FL,F_FGE,F_FLE
      );

    const
      FloatResFlags = [F_FE..F_FLE];

{*****************************************************************************
                                Reference
*****************************************************************************}

    type
      { direction of address register :      }
      {              (An)     (An)+   -(An)  }
      tdirection = (dir_none,dir_inc,dir_dec);


{*****************************************************************************
                                Operand Sizes
*****************************************************************************}

       { S_NO = No Size of operand   }
       { S_B  = 8-bit size operand   }
       { S_W  = 16-bit size operand  }
       { S_L  = 32-bit size operand  }
       { Floating point types        }
       { S_FS  = single type (32 bit) }
       { S_FD  = double/64bit integer }
       { S_FX  = Extended type      }
       topsize = (S_NO,S_B,S_W,S_L,S_FS,S_FD,S_FX,S_IQ);

       TOpSizes = set of topsize;

{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      {# maximum number of operands in assembler instruction }
      max_operands = 4;

{*****************************************************************************
                          Default generic sizes
*****************************************************************************}

      {# Defines the default address size for a processor, }
      OS_ADDR = OS_32;
      {# the natural int size for a processor,
         has to match osuinttype/ossinttype as initialized in psystem }
      OS_INT = OS_32;
      OS_SINT = OS_S32;
      {# the maximum float size for a processor,           }
      OS_FLOAT = OS_F64;
      {# the size of a vector register for a processor     }
      OS_VECTOR = OS_M128;


{*****************************************************************************
                               GDB Information
*****************************************************************************}

      {# Register indexes for stabs information, when some
         parameters or variables are stored in registers.

         Taken from m68kelf.h (DBX_REGISTER_NUMBER)
         from GCC 3.x source code.

         This is not compatible with the m68k-sun
         implementation.
      }
      stab_regindex : array[tregisterindex] of shortint =
      (
        {$i r68ksta.inc}
      );

{*****************************************************************************
                          Generic Register names
*****************************************************************************}

      {# Stack pointer register }
      NR_STACK_POINTER_REG = NR_SP;
      RS_STACK_POINTER_REG = RS_SP;
      {# Frame pointer register }

      { Frame pointer register (initialized in tcpuprocinfo.init_framepointer) }
      RS_FRAME_POINTER_REG: tsuperregister = RS_NO;
      NR_FRAME_POINTER_REG: tregister = NR_NO;

      {# Register for addressing absolute data in a position independant way,
         such as in PIC code. The exact meaning is ABI specific. For
         further information look at GCC source : PIC_OFFSET_TABLE_REGNUM
      }
      RS_PIC_OFFSET_REG: tsuperregister = RS_NO;
      NR_PIC_OFFSET_REG: tregister = NR_NO;

      { Return address for DWARF }
      NR_RETURN_ADDRESS_REG = NR_A0;
      RS_RETURN_ADDRESS_REG = RS_A0;
      { Results are returned in this register (32-bit values) }
      NR_FUNCTION_RETURN_REG = NR_D0;
      RS_FUNCTION_RETURN_REG = RS_D0;
      { Low part of 64bit return value }
      NR_FUNCTION_RETURN64_LOW_REG = NR_D0;
      RS_FUNCTION_RETURN64_LOW_REG = RS_D0;
      { High part of 64bit return value }
      NR_FUNCTION_RETURN64_HIGH_REG = NR_D1;
      RS_FUNCTION_RETURN64_HIGH_REG = RS_D1;
      { The value returned from a function is available in this register }
      NR_FUNCTION_RESULT_REG = NR_FUNCTION_RETURN_REG;
      RS_FUNCTION_RESULT_REG = RS_FUNCTION_RETURN_REG;
      { The lowh part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_LOW_REG = NR_FUNCTION_RETURN64_LOW_REG;
      RS_FUNCTION_RESULT64_LOW_REG = RS_FUNCTION_RETURN64_LOW_REG;
      { The high part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_HIGH_REG = NR_FUNCTION_RETURN64_HIGH_REG;
      RS_FUNCTION_RESULT64_HIGH_REG = RS_FUNCTION_RETURN64_HIGH_REG;

      {# Floating point results will be placed into this register }
      NR_FPU_RESULT_REG = NR_FP0;

      {# This is m68k C ABI specific. Some ABIs expect the address of the
         return struct result value in this register. Note that it could be
         either A0 or A1, so later it must be decided on target/ABI specific
         basis. We start with A1 now, because that's what Linux/m68k does
         currently. (KB) }
      RS_M68K_STRUCT_RESULT_REG: tsuperregister = RS_A1;
      NR_M68K_STRUCT_RESULT_REG: tregister = NR_A1;

      NR_DEFAULTFLAGS = NR_SR;
      RS_DEFAULTFLAGS = RS_SR;

{*****************************************************************************
                       GCC /ABI linking information
*****************************************************************************}

      {# Required parameter alignment when calling a routine declared as
         stdcall and cdecl. The alignment value should be the one defined
         by GCC or the target ABI.

         The value of this constant is equal to the constant
         PARM_BOUNDARY / BITS_PER_UNIT in the GCC source.
      }
      std_param_align = 4;  { for 32-bit version only }


{*****************************************************************************
                            CPU Dependent Constants
*****************************************************************************}


{*****************************************************************************
                                  Helpers
*****************************************************************************}

    const
      tcgsize2opsize: Array[tcgsize] of topsize =
        (S_NO,S_B,S_W,S_L,S_L,S_NO,S_B,S_W,S_L,S_L,S_NO,
         S_FS,S_FD,S_FX,S_NO,S_NO,
         S_NO,S_NO,S_NO,S_NO,S_NO,S_NO,S_NO);

    function  is_calljmp(o:tasmop):boolean;

    procedure inverse_flags(var r : TResFlags);
    function  flags_to_cond(const f: TResFlags) : TAsmCond;
    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
    function reg_cgsize(const reg: tregister): tcgsize;

    function findreg_by_number(r:Tregister):tregisterindex;
    function std_regnum_search(const s:string):Tregister;
    function std_regname(r:Tregister):string;

    function isaddressregister(reg : tregister) : boolean;
    function isintregister(reg : tregister) : boolean;
    function fpuregopsize: TOpSize; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    function fpuregsize: aint; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    function needs_unaligned(const refalignment: aint; const size: tcgsize): boolean;
    function isregoverlap(reg1: tregister; reg2: tregister): boolean;

    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    function conditions_equal(const c1, c2: TAsmCond): boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}

    { Checks if Subset is a subset of c (e.g. "less than" is a subset of "less than or equal" }
    function condition_in(const Subset, c: TAsmCond): Boolean;

    function dwarf_reg(r:tregister):shortint;
    function dwarf_reg_no_error(r:tregister):shortint;
    function eh_return_data_regno(nr: longint): longint;

    function isvalue8bit(val: tcgint): boolean;
    function isvalue16bit(val: tcgint): boolean;
    function isvalueforaddqsubq(val: tcgint): boolean;

implementation

    uses
      verbose,
      rgbase;


    const
      std_regname_table : TRegNameTable = (
        {$i r68kstd.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i r68krni.inc}
      );

      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i r68ksri.inc}
      );


{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function is_calljmp(o:tasmop):boolean;
      begin
        case o of
          A_BXX,A_FBXX,A_DBXX,

          A_BHS,A_BLO,A_BHI,A_BLS,A_BCC,A_BCS,A_BNE,A_BEQ,A_BVC,A_BVS,
          A_BPL,A_BMI,A_BGE,A_BLT,A_BGT,A_BLE,A_BRA,
          A_DBT,A_DBF,A_DBHI,A_DBLS,A_DBCC,A_DBHS,A_DBCS,A_DBLO,A_DBNE,
          A_DBEQ,A_DBVC,A_DBVS,A_DBPL,A_DBMI,A_DBGE,A_DBLT,A_DBGT,A_DBLE,
          A_DBRA,

          A_FBF,A_FBEQ,A_FBOGT,A_FBOGE,A_FBOLT,A_FBOLE,A_FBOGL,A_FBOR,A_FBUN,
          A_FBUEQ,A_FBUGT,A_FBUGE,A_FBULT,A_FBULE,A_FBNE,A_FBT,A_FBSF,A_FBSEQ,
          A_FBGT,A_FBGE,A_FBLT,A_FBLE,A_FBGL,A_FBGLE,A_FBNGLE,A_FBNGL,A_FBNLE,
          A_FBNLT,A_FBNGE,A_FBNGT,A_FBSNE,A_FBST,

          A_FDBF,A_FDBEQ,A_FDBOGT,A_FDBOGE,A_FDBOLT,A_FDBOLE,A_FDBOGL,A_FDBOR,
          A_FDBUN,A_FDBUEQ,A_FDBUGT,A_FDBUGE,A_FDBULT,A_FDBULE,A_FDBNE,A_FDBT,
          A_FDBSF,A_FDBSEQ,A_FDBGT,A_FDBGE,A_FDBLT,A_FDBLE,A_FDBGL,A_FDBGLE,
          A_FDBNGLE,A_FDBNGL,A_FDBNLE,A_FDBNLT,A_FDBNGE,A_FDBNGT,A_FDBSNE,
          A_FDBST,

          A_JSR,A_BSR,A_JMP:
            is_calljmp:=true;
          else
            is_calljmp:=false;
        end;
      end;


    procedure inverse_flags(var r: TResFlags);
      const flagsinvers : array[F_E..F_FLE] of tresflags =
            (F_NE,F_E,
             F_LE,F_GE,
             F_L,F_G,
             F_NC,F_C,
             F_BE,F_B,
             F_AE,F_A,
             F_FNE,F_FE,
             F_FLE,F_FGE,
             F_FL,F_G);
      begin
         r:=flagsinvers[r];
      end;



    function flags_to_cond(const f: TResFlags) : TAsmCond;
      const flags2cond: array[tresflags] of tasmcond = (
          C_EQ,{F_E     equal}
          C_NE,{F_NE    not equal}
          C_GT,{F_G     gt signed}
          C_LT,{F_L     lt signed}
          C_GE,{F_GE    ge signed}
          C_LE,{F_LE    le signed}
          C_CS,{F_C     carry set}
          C_CC,{F_NC    carry clear}
          C_HI,{F_A     gt unsigned}
          C_CC,{F_AE    ge unsigned}
          C_CS,{F_B     lt unsigned}
          C_LS,{F_BE    le unsigned}
          C_EQ,{F_FEQ }
          C_NE,{F_FNE }
          C_GT,{F_FG  }
          C_LT,{F_FL  }
          C_GE,{F_FGE }
          C_LE);{F_FLE }
      begin
        flags_to_cond := flags2cond[f];
      end;

    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
      begin
        case regtype of
          R_INTREGISTER:
            if (CPUM68K_HAS_BYTEWORDMATH in cpu_capabilities[current_settings.cputype]) then
              case s of
                OS_8,OS_S8:
                  cgsize2subreg:=R_SUBL;
                OS_16,OS_S16:
                  cgsize2subreg:=R_SUBW;
                OS_32,OS_S32:
                  cgsize2subreg:=R_SUBD;
                OS_64,OS_S64:
                  cgsize2subreg:=R_SUBWHOLE;
                OS_NO:
                  cgsize2subreg:=R_SUBNONE;
              else
                internalerror(2019090801);
              end
            else
              case s of
                OS_8,OS_S8,
                OS_16,OS_S16,
                OS_32,OS_S32,
                OS_64,OS_S64:
                  cgsize2subreg:=R_SUBWHOLE;
                OS_NO:
                  cgsize2subreg:=R_SUBNONE;
              else
                internalerror(2019090803);
              end;
          R_ADDRESSREGISTER:
            cgsize2subreg:=R_SUBWHOLE;
          R_FPUREGISTER:
            cgsize2subreg:=R_SUBNONE;

          else
            internalerror(2019090802);
        end;
      end;


    function reg_cgsize(const reg: tregister): tcgsize;
      { 68881 & compatibles -> 80 bit }
      { CF FPU -> 64 bit }
      const
        fpureg_cgsize: array[boolean] of tcgsize = ( OS_F80, OS_F64 );
      begin
        case getregtype(reg) of
          R_ADDRESSREGISTER,
          R_INTREGISTER :
            result:=OS_32;
          R_FPUREGISTER :
            result:=fpureg_cgsize[current_settings.fputype = fpu_coldfire];
          else
            internalerror(200303181);
        end;
      end;


    function findreg_by_number(r:Tregister):tregisterindex;
      begin
        result:=findreg_by_number_table(r,regnumber_index);
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


    function isaddressregister(reg : tregister) : boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      begin
        result:=getregtype(reg)=R_ADDRESSREGISTER;
      end;

    function isintregister(reg : tregister) : boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      begin
        result:=getregtype(reg)=R_INTREGISTER;
      end;

    function fpuregopsize: TOpSize; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      const
        fpu_regopsize: array[boolean] of TOpSize = ( S_FX, S_FD );
      begin
        result:=fpu_regopsize[current_settings.fputype = fpu_coldfire];
      end;

    function fpuregsize: aint; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      const
        fpu_regsize: array[boolean] of aint = ( 12, 8 ); { S_FX is 12 bytes on '881 }
      begin
        result:=fpu_regsize[current_settings.fputype = fpu_coldfire];
      end;

    function needs_unaligned(const refalignment: aint; const size: tcgsize): boolean;
      begin
        result:=not(CPUM68K_HAS_UNALIGNED in cpu_capabilities[current_settings.cputype]) and
                (refalignment = 1) and
                (tcgsize2size[size] > 1);
      end;

    // the function returns true, if the registers overlap (subreg of the same superregister and same type)
    function isregoverlap(reg1: tregister; reg2: tregister): boolean;
      begin
        tregisterrec(reg1).subreg:=R_SUBNONE;
        tregisterrec(reg2).subreg:=R_SUBNONE;
        result:=reg1=reg2;
      end;

    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      const
        inverse:array[TAsmCond] of TAsmCond=(C_None,
         //C_CC,C_LS,C_CS,C_LT,C_EQ,C_MI,C_F,C_NE,
           C_CS,C_HI,C_CC,C_GE,C_NE,C_PL,C_T,C_EQ,
         //C_GE,C_PL,C_GT,C_T,C_HI,C_VC,C_LE,C_VS
           C_LT,C_MI,C_LE,C_F,C_LS,C_VS,C_GT,C_VC
        );
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
              Result := (c in [C_GE, C_LE]);
            C_LT:
              Result := (c in [C_LE]);
            C_GT:
              Result := (c in [C_GE]);
            else
              Result := False;
          end;
      end;


    function dwarf_reg(r:tregister):shortint;
      begin
        result:=regdwarf_table[findreg_by_number(r)];
        if result=-1 then
          internalerror(200603251);
      end;

    function dwarf_reg_no_error(r:tregister):shortint;
      begin
        result:=regdwarf_table[findreg_by_number(r)];
      end;

    function eh_return_data_regno(nr: longint): longint;
      begin
        result:=-1;
      end;

    { returns true if given value fits to an 8bit signed integer }
    function isvalue8bit(val: tcgint): boolean;
      begin
        isvalue8bit := (val >= low(shortint)) and (val <= high(shortint));
      end;

    { returns true if given value fits to a 16bit signed integer }
    function isvalue16bit(val: tcgint): boolean;
      begin
        isvalue16bit := (val >= low(smallint)) and (val <= high(smallint));
      end;

    { returns true if given value fits addq/subq argument, so in 1 - 8 range }
    function isvalueforaddqsubq(val: tcgint): boolean;
      begin
        isvalueforaddqsubq := (val >= 1) and (val <= 8);
      end;

end.
