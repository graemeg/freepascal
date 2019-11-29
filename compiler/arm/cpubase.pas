{
    Copyright (c) 1998-2002 by Florian Klaempfl and Peter Vreman

    Contains the base types for the ARM

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

{$define USEINLINE}

{$i fpcdefs.inc}

  interface

    uses
      globtype,globals,
      cpuinfo,
      cgbase
      ;


{*****************************************************************************
                                Assembler Opcodes
*****************************************************************************}

    type
      TAsmOp= {$i armop.inc}
      {This is a bit of a hack, because there are more than 256 ARM Assembly Ops
       But FPC currently can't handle more than 256 elements in a set.}
      TCommonAsmOps = Set of A_None .. A_UADD16;

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
      tregisterindex=0..{$i rarmnor.inc}-1;

    const
      { Available Superregisters }
      {$i rarmsup.inc}

      RS_PC = RS_R15;

      { No Subregisters }
      R_SUBWHOLE = R_SUBNONE;

      { Available Registers }
      {$i rarmcon.inc}

      { aliases }
      NR_PC = NR_R15;

      { Integer Super registers first and last }
      first_int_supreg = RS_R0;
      first_int_imreg = $10;

      { Float Super register first and last }
      first_fpu_supreg    = RS_F0;
      first_fpu_imreg     = $08;

      { MM Super register first and last }
      first_mm_supreg    = RS_S0;
      first_mm_imreg     = $30;

{ TODO: Calculate bsstart}
      regnumber_count_bsstart = 128;

      regnumber_table : array[tregisterindex] of tregister = (
        {$i rarmnum.inc}
      );

      regstabs_table : array[tregisterindex] of shortint = (
        {$i rarmsta.inc}
      );

      regdwarf_table : array[tregisterindex] of shortint = (
        {$i rarmdwa.inc}
      );
      { registers which may be destroyed by calls }
      VOLATILE_INTREGISTERS = [RS_R0..RS_R3,RS_R12..RS_R14];
      VOLATILE_FPUREGISTERS = [RS_F0..RS_F3];
      VOLATILE_MMREGISTERS =  [RS_D0..RS_D7,RS_D16..RS_D31];

      VOLATILE_INTREGISTERS_DARWIN = [RS_R0..RS_R3,RS_R9,RS_R12..RS_R14];

{*****************************************************************************
                          Instruction post fixes
*****************************************************************************}
    type
      { ARM instructions load/store and arithmetic instructions
        can have several instruction post fixes which are collected
        in this enumeration
      }
      TOpPostfix = (PF_None,
        { update condition flags
          or floating point single }
        PF_S,
        { floating point size }
        PF_D,PF_E,PF_P,PF_EP,
        { exchange }
        PF_X,
        { rounding }
        PF_R,
        { load/store }
        PF_B,PF_SB,PF_BT,PF_H,PF_SH,PF_T,
        { multiple load/store address modes }
        PF_IA,PF_IB,PF_DA,PF_DB,PF_FD,PF_FA,PF_ED,PF_EA,
        { multiple load/store vfp address modes }
        PF_IAD,PF_DBD,PF_FDD,PF_EAD,
        PF_IAS,PF_DBS,PF_FDS,PF_EAS,
        PF_IAX,PF_DBX,PF_FDX,PF_EAX,
        { VFP postfixes }
        PF_8,PF_16,PF_32,PF_64,
        PF_I8,PF_I16,PF_I32,PF_I64,
        PF_S8,PF_S16,PF_S32,PF_S64,
        PF_U8,PF_U16,PF_U32,PF_U64,
        PF_P8, // polynomial
        PF_F32,PF_F64,
        PF_F32F64,PF_F64F32,
        PF_F32S16,PF_F32U16,PF_S16F32,PF_U16F32,
        PF_F64S16,PF_F64U16,PF_S16F64,PF_U16F64,
        PF_F32S32,PF_F32U32,PF_S32F32,PF_U32F32,
        PF_F64S32,PF_F64U32,PF_S32F64,PF_U32F64
      );

      TOpPostfixes = set of TOpPostfix;

      TRoundingMode = (RM_None,RM_P,RM_M,RM_Z);

    const
      cgsize2fpuoppostfix : array[OS_NO..OS_F128] of toppostfix = (
        PF_None,
        PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,PF_None,
        PF_S,PF_D,PF_E,PF_None,PF_None);

      oppostfix2str : array[TOpPostfix] of string[8] = ('',
        's',
        'd','e','p','ep',
        'x',
        'r',
        'b','sb','bt','h','sh','t',
        'ia','ib','da','db','fd','fa','ed','ea',
        'iad','dbd','fdd','ead',
        'ias','dbs','fds','eas',
        'iax','dbx','fdx','eax',
        '.8','.16','.32','.64',
        '.i8','.i16','.i32','.i64',
        '.s8','.s16','.s32','.s64',
        '.u8','.u16','.u32','.u64',
        '.p8',
        '.f32','.f64',
        '.f32.f64','.f64.f32',
        '.f32.s16','.f32.u16','.s16.f32','.u16.f32',
        '.f64.s16','.f64.u16','.s16.f64','.u16.f64',
        '.f32.s32','.f32.u32','.s32.f32','.u32.f32',
        '.f64.s32','.f64.u32','.s32.f64','.u32.f64');

      roundingmode2str : array[TRoundingMode] of string[1] = ('',
        'p','m','z');

{*****************************************************************************
                                Conditions
*****************************************************************************}

    type
      TAsmCond=(C_None,
        C_EQ,C_NE,C_CS,C_CC,C_MI,C_PL,C_VS,C_VC,C_HI,C_LS,
        C_GE,C_LT,C_GT,C_LE,C_AL,C_NV
      );

      TAsmConds = set of TAsmCond;

    const
      cond2str : array[TAsmCond] of string[2]=('',
        'eq','ne','cs','cc','mi','pl','vs','vc','hi','ls',
        'ge','lt','gt','le','al','nv'
      );

      uppercond2str : array[TAsmCond] of string[2]=('',
        'EQ','NE','CS','CC','MI','PL','VS','VC','HI','LS',
        'GE','LT','GT','LE','AL','NV'
      );

{*****************************************************************************
                                   Flags
*****************************************************************************}

    type
      TResFlags = (F_EQ,F_NE,F_CS,F_CC,F_MI,F_PL,F_VS,F_VC,F_HI,F_LS,
        F_GE,F_LT,F_GT,F_LE);

{*****************************************************************************
                                Operands
*****************************************************************************}

      taddressmode = (AM_OFFSET,AM_PREINDEXED,AM_POSTINDEXED);
      tshiftmode = (SM_None,SM_LSL,SM_LSR,SM_ASR,SM_ROR,SM_RRX);

      tupdatereg = (UR_None,UR_Update);

      pshifterop = ^tshifterop;

      tshifterop = record
        shiftmode : tshiftmode;
        rs : tregister;
        shiftimm : byte;
      end;

      tcpumodeflag = (mfA, mfI, mfF);
      tcpumodeflags = set of tcpumodeflag;

      tspecialregflag = (srC, srX, srS, srF);
      tspecialregflags = set of tspecialregflag;

{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      max_operands = 6;

      maxintregs = 15;
      maxfpuregs = 8;
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
      { the natural int size for a processor,
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

      { Stack pointer register }
      NR_STACK_POINTER_REG = NR_R13;
      RS_STACK_POINTER_REG = RS_R13;
      { Frame pointer register (initialized in tcpuprocinfo.init_framepointer) }
      RS_FRAME_POINTER_REG: tsuperregister = RS_NO;
      NR_FRAME_POINTER_REG: tregister = NR_NO;
      { Register for addressing absolute data in a position independant way,
        such as in PIC code. The exact meaning is ABI specific. For
        further information look at GCC source : PIC_OFFSET_TABLE_REGNUM
      }
      NR_PIC_OFFSET_REG = NR_R9;
      { Results are returned in this register (32-bit values) }
      NR_FUNCTION_RETURN_REG = NR_R0;
      RS_FUNCTION_RETURN_REG = RS_R0;
      { The value returned from a function is available in this register }
      NR_FUNCTION_RESULT_REG = NR_FUNCTION_RETURN_REG;
      RS_FUNCTION_RESULT_REG = RS_FUNCTION_RETURN_REG;

      NR_FPU_RESULT_REG = NR_F0;

      NR_MM_RESULT_REG  = NR_D0;

      NR_RETURN_ADDRESS_REG = NR_FUNCTION_RETURN_REG;

      { Offset where the parent framepointer is pushed }
      PARENT_FRAMEPOINTER_OFFSET = 0;

      NR_DEFAULTFLAGS = NR_CPSR;
      RS_DEFAULTFLAGS = RS_CPSR;

      { Low part of 64bit return value }
      function NR_FUNCTION_RESULT64_LOW_REG: tregister;{$ifdef USEINLINE}inline;{$endif USEINLINE}
      function RS_FUNCTION_RESULT64_LOW_REG: shortint;{$ifdef USEINLINE}inline;{$endif USEINLINE}
      { High part of 64bit return value }
      function NR_FUNCTION_RESULT64_HIGH_REG: tregister;{$ifdef USEINLINE}inline;{$endif USEINLINE}
      function RS_FUNCTION_RESULT64_HIGH_REG: shortint;{$ifdef USEINLINE}inline;{$endif USEINLINE}

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
    function is_calljmp(o:tasmop):boolean;{$ifdef USEINLINE}inline;{$endif USEINLINE}
    procedure inverse_flags(var f: TResFlags);
    function flags_to_cond(const f: TResFlags) : TAsmCond;
    function findreg_by_number(r:Tregister):tregisterindex;
    function std_regnum_search(const s:string):Tregister;
    function std_regname(r:Tregister):string;

    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    function conditions_equal(const c1, c2: TAsmCond): boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}

    { Checks if Subset is a subset of c (e.g. "less than" is a subset of "less than or equal" }
    function condition_in(const Subset, c: TAsmCond): Boolean;

    procedure shifterop_reset(var so : tshifterop); {$ifdef USEINLINE}inline;{$endif USEINLINE}
    function is_pc(const r : tregister) : boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}

    function is_shifter_const(d : aint;var imm_shift : byte) : boolean;
    function is_thumb_imm(d: aint): boolean;
    { Returns true if d is a valid constant for thumb 32 bit,
      doesn't handle ROR_C detection }
    function is_thumb32_imm(d : aint) : boolean;
    function split_into_shifter_const(value : aint;var imm1: dword; var imm2: dword):boolean;
    function is_continuous_mask(d : aword;var lsb, width: byte) : boolean;
    function dwarf_reg(r:tregister):shortint;
    function dwarf_reg_no_error(r:tregister):shortint;
    function eh_return_data_regno(nr: longint): longint;


    function IsIT(op: TAsmOp) : boolean;
    function GetITLevels(op: TAsmOp) : longint;

    function GenerateARMCode : boolean;
    function GenerateThumbCode : boolean;
    function GenerateThumb2Code : boolean;

    function IsVFPFloatImmediate(ft : tfloattype;value : bestreal) : boolean;

  implementation

    uses
      systems,rgBase,verbose;


    const
      std_regname_table : TRegNameTable = (
        {$i rarmstd.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i rarmrni.inc}
      );

      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i rarmsri.inc}
      );


    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
      begin
        case regtype of
          R_MMREGISTER:
            begin
              case s of
                { records passed in MM registers }
                OS_32,
                OS_F32:
                  cgsize2subreg:=R_SUBFS;
                OS_64,
                OS_F64:
                  cgsize2subreg:=R_SUBFD;
                else
                  internalerror(2009112701);
              end;
            end;
          else
            cgsize2subreg:=R_SUBWHOLE;
        end;
      end;


    function reg_cgsize(const reg: tregister): tcgsize;
      begin
        case getregtype(reg) of
          R_INTREGISTER :
            reg_cgsize:=OS_32;
          R_FPUREGISTER :
            reg_cgsize:=OS_F80;
          R_MMREGISTER :
            begin
              case getsubreg(reg) of
                R_SUBFD,
                R_SUBWHOLE:
                  result:=OS_F64;
                R_SUBFS:
                  result:=OS_F32;
                else
                  internalerror(2009112903);
              end;
            end;
          else
            internalerror(200303181);
          end;
        end;


    function is_calljmp(o:tasmop):boolean;{$ifdef USEINLINE}inline;{$endif USEINLINE}
      begin
        { This isn't 100% perfect because the arm allows jumps also by writing to PC=R15.
          To overcome this problem we simply forbid that FPC generates jumps by loading R15 }
        is_calljmp:= o in [A_B,A_BL,A_BX,A_BLX];
      end;


    procedure inverse_flags(var f: TResFlags);
      const
        inv_flags: array[TResFlags] of TResFlags =
          (F_NE,F_EQ,F_CC,F_CS,F_PL,F_MI,F_VC,F_VS,F_LS,F_HI,
          F_LT,F_GE,F_LE,F_GT);
      begin
        f:=inv_flags[f];
      end;


    function flags_to_cond(const f: TResFlags) : TAsmCond;
      const
        flag_2_cond: array[F_EQ..F_LE] of TAsmCond =
          (C_EQ,C_NE,C_CS,C_CC,C_MI,C_PL,C_VS,C_VC,C_HI,C_LS,
           C_GE,C_LT,C_GT,C_LE);
      begin
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


    procedure shifterop_reset(var so : tshifterop);{$ifdef USEINLINE}inline;{$endif USEINLINE}
      begin
        FillChar(so,sizeof(so),0);
      end;


    function is_pc(const r : tregister) : boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      begin
        is_pc:=(r=NR_R15);
      end;


    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      const
        inverse: array[TAsmCond] of TAsmCond=(C_None,
          C_NE,C_EQ,C_CC,C_CS,C_PL,C_MI,C_VC,C_VS,C_LS,C_HI,
          C_LT,C_GE,C_LE,C_GT,C_None,C_None
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


    function is_shifter_const(d : aint;var imm_shift : byte) : boolean;
      var
         i : longint;
      begin
        if GenerateThumb2Code then
          begin
            for i:=0 to 24 do
              begin
                 if (dword(d) and not($ff shl i))=0 then
                   begin
                     imm_shift:=i;
                     result:=true;
                     exit;
                   end;
              end;
          end
        else
          begin
            for i:=0 to 15 do
              begin
                 if (dword(d) and not(roldword($ff,i*2)))=0 then
                   begin
                      imm_shift:=i*2;
                      result:=true;
                      exit;
                   end;
              end;
          end;
        result:=false;
      end;


    function is_thumb_imm(d: aint): boolean;
      begin
        result:=(d and $FF) = d;
      end;


    function is_thumb32_imm(d: aint): boolean;
      var
        t : aint;
        i : longint;
      begin
        {Loading 0-255 is simple}
        if (d and $FF) = d then
          result:=true
        { If top and bottom are equal, check if either all 4 bytes are equal
          or byte 0 and 2 or byte 1 and 3 are equal }
        else if ((d shr 16)=(d and $FFFF)) and
                (
                  ((d and $FF00FF00) = 0) or
                  ((d and $00FF00FF) = 0) or
                  ((d shr 8)=(d and $FF))
                ) then
          result:=true
        {Can an 8-bit value be shifted accordingly?}
        else
          begin
            result:=false;
            for i:=8 to 31 do
              begin
                t:=RolDWord(d,i);
                if ((t and $FF)=t) and
                   ((t and $80)=$80) then
                  begin
                    result:=true;
                    exit;
                  end;
              end;
          end;
      end;
    
    function is_continuous_mask(d : aword;var lsb, width: byte) : boolean;
      var
        msb : byte;
      begin
        lsb:=BsfDword(d);
        msb:=BsrDword(d);
        
        width:=msb-lsb+1;
        
        result:=(lsb<>255) and (msb<>255) and (aword(((1 shl (msb-lsb+1))-1) shl lsb) = d);
      end;


    function split_into_shifter_const(value : aint;var imm1: dword; var imm2: dword) : boolean;
      var
        d, i, i2: Dword;
      begin
        Result:=false;
        {Thumb2 is not supported (YET?)}
        if GenerateThumb2Code then exit;
        d:=DWord(value);
        for i:=0 to 15 do
          begin
            imm1:=d and rordword($FF, I*2);
            imm2:=d and not (imm1); {remove already found bits}
            {is the remainder a shifterconst? YAY! we've done it!}
            {Could we start from i instead of 0?}
            for i2:=0 to 15 do
              begin
                 if (imm2 and not(rordword($FF,i2*2)))=0 then
                   begin
                      result:=true;
                      exit;
                   end;
              end;
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
          result:=nr
        else
          result:=-1;
      end;

      { Low part of 64bit return value }
    function NR_FUNCTION_RESULT64_LOW_REG: tregister; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    begin
      if target_info.endian=endian_little then
        result:=NR_R0
      else
        result:=NR_R1;
    end;

    function RS_FUNCTION_RESULT64_LOW_REG: shortint; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    begin
      if target_info.endian=endian_little then
        result:=RS_R0
      else
        result:=RS_R1;
    end;

      { High part of 64bit return value }
    function NR_FUNCTION_RESULT64_HIGH_REG: tregister; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    begin
      if target_info.endian=endian_little then
        result:=NR_R1
      else
        result:=NR_R0;
    end;

    function RS_FUNCTION_RESULT64_HIGH_REG: shortint; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    begin
      if target_info.endian=endian_little then
        result:=RS_R1
      else
        result:=RS_R0;
    end;

    function IsIT(op: TAsmOp) : boolean;
      begin
        case op of
          A_IT,
          A_ITE, A_ITT,
          A_ITEE, A_ITTE, A_ITET, A_ITTT,
          A_ITEEE, A_ITTEE, A_ITETE, A_ITTTE,
          A_ITEET, A_ITTET, A_ITETT, A_ITTTT:
            result:=true;
        else
          result:=false;
        end;
      end;

    function GetITLevels(op: TAsmOp) : longint;
      begin
        case op of
          A_IT:
            result:=1;
          A_ITE, A_ITT:
            result:=2;
          A_ITEE, A_ITTE, A_ITET, A_ITTT:
            result:=3;
          A_ITEEE, A_ITTEE, A_ITETE, A_ITTTE,
          A_ITEET, A_ITTET, A_ITETT, A_ITTTT:
            result:=4;
        else
          result:=0;
        end;
      end;


    function GenerateARMCode : boolean;
      begin
        Result:=current_settings.instructionset=is_arm;
      end;


    function GenerateThumbCode : boolean;
      begin
        Result:=(current_settings.instructionset=is_thumb) and not(CPUARM_HAS_THUMB2 in cpu_capabilities[current_settings.cputype]);
      end;


    function GenerateThumb2Code : boolean;
      begin
        Result:=(current_settings.instructionset=is_thumb) and (CPUARM_HAS_THUMB2 in cpu_capabilities[current_settings.cputype]);
      end;


    function IsVFPFloatImmediate(ft : tfloattype;value : bestreal) : boolean;
      var
        singlerec : tcompsinglerec;
        doublerec : tcompdoublerec;
      begin
        Result:=false;
        case ft of
          s32real:
            begin
              singlerec.value:=value;
              singlerec:=tcompsinglerec(NtoLE(DWord(singlerec)));
              Result:=(singlerec.bytes[0]=0) and (singlerec.bytes[1]=0) and ((singlerec.bytes[2] and 7)=0)  and
                (((singlerec.bytes[3] and $7e)=$40) or ((singlerec.bytes[3] and $7e)=$3e));
            end;
          s64real:
            begin
              doublerec.value:=value;
              doublerec:=tcompdoublerec(NtoLE(QWord(doublerec)));
              Result:=(doublerec.bytes[0]=0) and (doublerec.bytes[1]=0) and (doublerec.bytes[2]=0) and
                      (doublerec.bytes[3]=0) and (doublerec.bytes[4]=0) and (doublerec.bytes[5]=0) and
                      ((((doublerec.bytes[6] and $c0)=$0) and ((doublerec.bytes[7] and $7f)=$40)) or
                       (((doublerec.bytes[6] and $c0)=$c0) and ((doublerec.bytes[7] and $7f)=$3f)));
            end;
          else
            ;
        end;
      end;


end.

