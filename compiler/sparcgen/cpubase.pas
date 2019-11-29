{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Contains the base types for the SPARC

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
unit cpubase;

{$i fpcdefs.inc}

{$ModeSwitch advancedrecords}

interface

uses
  globtype,strings,cutils,cclasses,aasmbase,cpuinfo,cgbase;


{*****************************************************************************
                                Assembler Opcodes
*****************************************************************************}

    type
{ TODO: CPU32 opcodes do not fully include the Ultra SPRAC instruction set.}
      { don't change the order of these opcodes! }
      TAsmOp=({$i opcode.inc});

      {# This should define the array of instructions as string }
      op2strtable=array[tasmop] of string[11];

    Const
      {# First value of opcode enumeration }
      firstop = low(tasmop);
      {# Last value of opcode enumeration  }
      lastop  = high(tasmop);

      std_op2str:op2strtable=({$i strinst.inc});

{*****************************************************************************
                                  Registers
*****************************************************************************}

{$ifdef SPARC}
    type
      { Number of registers used for indexing in tables }
      tregisterindex=0..{$i rspnor.inc}-1;

    const
      { Available Superregisters }
      {$i rspsup.inc}

      { No Subregisters }
      R_SUBWHOLE = R_SUBNONE;

      { Available Registers }
      {$i rspcon.inc}

      first_int_imreg = $20;
      first_fpu_imreg = $20;

      { MM Super register first and last }
      first_mm_supreg    = 0;
      first_mm_imreg     = 1;

{ TODO: Calculate bsstart}
      regnumber_count_bsstart = 128;

      regnumber_table : array[tregisterindex] of tregister = (
        {$i rspnum.inc}
      );

      regstabs_table : array[tregisterindex] of ShortInt = (
        {$i rspstab.inc}
      );

      regdwarf_table : array[tregisterindex] of ShortInt = (
        {$i rspdwrf.inc}
      );

      { Aliases for full register LoadStore instructions }
      A_ST_R = A_ST;
      A_LD_R = A_LD;
{$endif SPARC}

{$ifdef SPARC64}
    type
      { Number of registers used for indexing in tables }
      tregisterindex=0..{$i rsp64nor.inc}-1;

    const
      { Available Superregisters }
      {$i rsp64sup.inc}

      { No Subregisters }
      R_SUBWHOLE = R_SUBNONE;

      { Available Registers }
      {$i rsp64con.inc}

      first_int_imreg = $20;
      first_fpu_imreg = $20;

      { MM Super register first and last }
      first_mm_supreg    = 0;
      first_mm_imreg     = 1;

{ TODO: Calculate bsstart}
      regnumber_count_bsstart = 128;

      regnumber_table : array[tregisterindex] of tregister = (
        {$i rsp64num.inc}
      );

      regstabs_table : array[tregisterindex] of ShortInt = (
        {$i rsp64stab.inc}
      );

      regdwarf_table : array[tregisterindex] of ShortInt = (
        {$i rsp64dwrf.inc}
      );
      { Aliases for full register LoadStore instructions }
      A_ST_R = A_STX;
      A_LD_R = A_LDX;
{$endif SPARC64}

{*****************************************************************************
                                Conditions
*****************************************************************************}

    type
      TAsmCond=(C_None,
        C_A,C_AE,C_B,C_BE,
        C_G,C_GE,C_L,C_LE,
        C_E,C_NE,
        C_POS,C_NEG,C_VC,C_VS,
        C_FE,C_FG,C_FL,C_FGE,C_FLE,C_FNE,
        C_FU,C_FUG,C_FUL,C_FUGE,C_FULE,C_FO,C_FUE,C_FLG
      );

    const
      firstIntCond=C_A;
      lastIntCond=C_VS;
      firstFloatCond=C_FE;
      lastFloatCond=C_FNE;
      floatAsmConds=[C_FE..C_FLG];

      cond2str:array[TAsmCond] of string[3]=('',
        'gu','cc','cs','leu',
        'g','ge','l','le',
        'e','ne',
        'pos','neg','vc','vs',
        'e','g','l','ge','le','ne',
        'u','ug','ul','uge','ule','o','ue','lg'
      );


{*****************************************************************************
                                   Flags
*****************************************************************************}

    type
        TSparcFlags = (
          { Integer results }
          F_E,  {Equal}
          F_NE, {Not Equal}
          F_G,  {Greater}
          F_L,  {Less}
          F_GE, {Greater or Equal}
          F_LE, {Less or Equal}
          F_A,  {Above}
          F_AE, {Above or Equal, synonym: Carry Clear}
          F_B,  {Below, synonym: Carry Set}
          F_BE, {Below or Equal}
          { Floating point results }
          F_FE,  {Equal}
          F_FNE, {Not Equal}
          F_FG,  {Greater}
          F_FL,  {Less}
          F_FGE, {Greater or Equal}
          F_FLE  {Less or Equal}
          );
      TResFlags = record
        { either icc or xcc (64 bit }
        FlagReg : TRegister;
        Flags : TSparcFlags;
        procedure Init(r : TRegister;f : TSparcFlags);
      end;

{*****************************************************************************
                                Operand Sizes
*****************************************************************************}


{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      max_operands = 3;

      maxintregs = 8;
      maxfpuregs = 8;
      maxaddrregs = 0;

      maxvarregs = 8;
      varregs : Array [1..maxvarregs] of Tsuperregister =
                (RS_L0,RS_L1,RS_L2,RS_L3,RS_L4,RS_L5,RS_L6,RS_L7);

      maxfpuvarregs = 1;
      fpuvarregs : Array [1..maxfpuvarregs] of TsuperRegister =
                (RS_F2);

{*****************************************************************************
                          Default generic sizes
*****************************************************************************}

{$ifdef SPARC64}
      {# Defines the default address size for a processor, }
      OS_ADDR = OS_64;
      {# the natural int size for a processor,
         has to match osuinttype/ossinttype as initialized in psystem }
      OS_INT = OS_64;
      OS_SINT = OS_S64;
{$else SPARC64}
      {# Defines the default address size for a processor, }
      OS_ADDR = OS_32;
      {# the natural int size for a processor,
         has to match osuinttype/ossinttype as initialized in psystem }
      OS_INT = OS_32;
      OS_SINT = OS_S32;
{$endif SPARC64}
      {# the maximum float size for a processor,           }
      OS_FLOAT = OS_F64;
      {# the size of a vector register for a processor     }
      OS_VECTOR = OS_M64;

{*****************************************************************************
                          Generic Register names
*****************************************************************************}

      {# Stack pointer register }
      NR_STACK_POINTER_REG = NR_O6;
      RS_STACK_POINTER_REG = RS_O6;
      {# Frame pointer register }
      NR_FRAME_POINTER_REG = NR_I6;
      RS_FRAME_POINTER_REG = RS_I6;
      {# Register for addressing absolute data in a position independant way,
         such as in PIC code. The exact meaning is ABI specific. For
         further information look at GCC source : PIC_OFFSET_TABLE_REGNUM

         Taken from GCC rs6000.h
      }
{ TODO: As indicated in rs6000.h, but can't find it anywhere else!}
      {PIC_OFFSET_REG = R_30;}
      { Return address for DWARF }
      NR_RETURN_ADDRESS_REG = NR_I7;
      { the return_result_reg, is used inside the called function to store its return
      value when that is a scalar value otherwise a pointer to the address of the
      result is placed inside it }
      { Results are returned in this register (32-bit values) }
      NR_FUNCTION_RETURN_REG = NR_I0;
      RS_FUNCTION_RETURN_REG = RS_I0;
      { Low part of 64bit return value }
      NR_FUNCTION_RETURN64_LOW_REG = NR_I1;
      RS_FUNCTION_RETURN64_LOW_REG = RS_I1;
      { High part of 64bit return value }
      NR_FUNCTION_RETURN64_HIGH_REG = NR_I0;
      RS_FUNCTION_RETURN64_HIGH_REG = RS_I0;
      { The value returned from a function is available in this register }
      NR_FUNCTION_RESULT_REG = NR_O0;
      RS_FUNCTION_RESULT_REG = RS_O0;
      { The lowh part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_LOW_REG = NR_O1;
      RS_FUNCTION_RESULT64_LOW_REG = RS_O1;
      { The high part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_HIGH_REG = NR_O0;
      RS_FUNCTION_RESULT64_HIGH_REG = RS_O0;

      NR_FPU_RESULT_REG = NR_F0;
      NR_MM_RESULT_REG  = NR_NO;

      PARENT_FRAMEPOINTER_OFFSET = 68; { o0 }

      NR_DEFAULTFLAGS = NR_PSR;
      RS_DEFAULTFLAGS = RS_PSR;

{*****************************************************************************
                       GCC /ABI linking information
*****************************************************************************}

      {# Required parameter alignment when calling a routine declared as
         stdcall and cdecl. The alignment value should be the one defined
         by GCC or the target ABI.

         The value of this constant is equal to the constant
         PARM_BOUNDARY / BITS_PER_UNIT in the GCC source.
      }
      std_param_align = sizeof(AWord);

{$ifdef SPARC64}
      STACK_BIAS = 2047;
{$endif SPARC64}


{*****************************************************************************
                            CPU Dependent Constants
*****************************************************************************}

    const
      simm13lo=-4096;
      simm13hi=4095;

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function  is_calljmp(o:tasmop):boolean;

    procedure inverse_flags(var f: TResFlags);
    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    function conditions_equal(const c1, c2: TAsmCond): boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}

    { Checks if Subset is a subset of c (e.g. "less than" is a subset of "less than or equal" }
    function condition_in(const Subset, c: TAsmCond): Boolean;

    function  flags_to_cond(const f: TResFlags) : TAsmCond;
    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
    function reg_cgsize(const reg: tregister): tcgsize;
    function std_regname(r:Tregister):string;
    function std_regnum_search(const s:string):Tregister;
    function findreg_by_number(r:Tregister):tregisterindex;
    function dwarf_reg(r:tregister):shortint;
    function dwarf_reg_no_error(r:tregister):shortint;
    function eh_return_data_regno(nr: longint): longint;

implementation

    uses
      rgBase,verbose;

{$ifdef SPARC}
    const
      std_regname_table : TRegNameTAble = (
        {$i rspstd.inc}
      );

      regnumber_index : TRegisterIndexTable = (
        {$i rsprni.inc}
      );

      std_regname_index : TRegisterIndexTable = (
        {$i rspsri.inc}
      );
{$endif SPARC}

{$ifdef SPARC64}
    const
      std_regname_table : TRegNameTAble = (
        {$i rsp64std.inc}
      );

      regnumber_index : TRegisterIndexTable = (
        {$i rsp64rni.inc}
      );

      std_regname_index : TRegisterIndexTable = (
        {$i rsp64sri.inc}
      );
{$endif SPARC64}

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function is_calljmp(o:tasmop):boolean;
      const
        CallJmpOp=[A_JMPL..A_CBccc];
      begin
        is_calljmp:=(o in CallJmpOp);
      end;


    procedure inverse_flags(var f: TResFlags);
      const
        inv_flags: array[TSparcFlags] of TSparcFlags =
          (F_NE,F_E,F_LE,F_GE,F_L,F_G,F_BE,F_B,F_AE,F_A,
           F_FNE,F_FE,F_FLE,F_FGE,F_FL,F_FG);
      begin
        f.Flags:=inv_flags[f.Flags];
      end;


   function flags_to_cond(const f:TResFlags):TAsmCond;
      const
        flags_2_cond:array[TSparcFlags] of TAsmCond=
          (C_E,C_NE,C_G,C_L,C_GE,C_LE,C_A,C_AE,C_B,C_BE,
           C_FE,C_FNE,C_FG,C_FL,C_FGE,C_FLE);
      begin
        result:=flags_2_cond[f.Flags];
      end;


    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
      begin
        case regtype of
          R_FPUREGISTER:
            case s of
              OS_F32:
                cgsize2subreg:=R_SUBFS;
              OS_F64:
                cgsize2subreg:=R_SUBFD;
              OS_F128:
                cgsize2subreg:=R_SUBFQ;
              else
                internalerror(2009071903);
            end;
          else
            begin
{$ifdef SPARC32}
              if s in [OS_64,OS_S64] then
                cgsize2subreg:=R_SUBQ
              else
{$endif SPARC32}
                cgsize2subreg:=R_SUBWHOLE;
            end;
        end;
      end;


    function reg_cgsize(const reg: tregister): tcgsize;
      begin
        case getregtype(reg) of
          R_INTREGISTER :
            result:=OS_INT;
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


    function findreg_by_number(r:Tregister):tregisterindex;
      begin
        result:=findreg_by_number_table(r,regnumber_index);
      end;


    function std_regname(r:Tregister):string;
      var
        p : tregisterindex;
      begin
        { For double floats show a pair like %f0:%f1 }
        if (getsubreg(r)=R_SUBFD) and
           (getsupreg(r)<first_fpu_imreg) then
          begin
            setsubreg(r,R_SUBFS);
            p:=findreg_by_number(r);
            if p<>0 then
              result:=std_regname_table[p]
            else
              result:=generic_regname(r);
            setsupreg(r,getsupreg(r)+1);
            p:=findreg_by_number(r);
            if p<>0 then
              result:=result+':'+std_regname_table[p]
            else
              result:=result+':'+generic_regname(r);
          end
        else
          begin
            p:=findreg_by_number(r);
            if p<>0 then
              result:=std_regname_table[p]
            else
              result:=generic_regname(r);
          end;
      end;


    function std_regnum_search(const s:string):Tregister;
      begin
        result:=regnumber_table[findreg_by_name_table(s,std_regname_table,std_regname_index)];
      end;


    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      const
        inverse: array[TAsmCond] of TAsmCond=(C_None,
          C_BE,C_B,C_AE,C_A,
          C_LE,C_L,C_GE,C_G,
          C_NE,C_E,
          C_NEG,C_POS,C_VS,C_VC,
          C_FNE,C_FULE,C_FUGE,C_FUL,C_FUG,C_FE,
          C_FO,C_FLE,C_FGE,C_FL,C_FG,C_FU,C_FLG,C_FUE
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

        { TODO: Can a SparcGEN programmer please update this procedure to
          detect all subsets? Thanks. [Kit] }
        if not Result then
          case Subset of
            C_A:
              Result := (c in [C_A,  C_AE]);
            C_B:
              Result := (c in [C_B,  C_BE]);
            C_E:
              Result := (c in [C_AE, C_BE]);
            C_FE:
              Result := (c in [C_FLE,C_FGE]);
            C_FL:
              Result := (c in [C_FLE]);
            C_FG:
              Result := (c in [C_FGE]);
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
        if (nr>=0) and (nr<2) then
          result:=nr+24
        else
          result:=-1;
      end;


    procedure TResFlags.Init(r : TRegister; f : TSparcFlags);
      begin
        FlagReg:=r;
        Flags:=f;
      end;

end.
