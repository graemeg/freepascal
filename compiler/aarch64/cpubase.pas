{
    Copyright (c) 1998-2012 by Florian Klaempfl and Peter Vreman
    Copyright (c) 2014 by Jonas Maebe and Florian Klaempfl

    Contains the base types for Aarch64

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
{ Base unit for processor information. This unit contains
  enumerations of registers, opcodes, sizes, and other
  such things which are processor specific.
}
unit cpubase;

{$define USEINLINE}

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
      TAsmOp= {$i a64op.inc}

      TAsmOps = set of TAsmOp;

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
      tregisterindex=0..{$i ra64nor.inc}-1;

    const
      { Available Superregisters }
      {$i ra64sup.inc}

      RS_IP0 = RS_X16;
      RS_IP1 = RS_X17;

      R_SUBWHOLE = R_SUBQ;

      { Available Registers }
      {$i ra64con.inc}

      NR_IP0 = NR_X16;
      NR_IP1 = NR_X17;

      { Integer Super registers first and last }
      first_int_supreg = RS_X0;
      { xzr and sp take up a separate super register because some instructions
        are ambiguous otherwise }
      first_int_imreg = $21;

      { Integer Super registers first and last }
      first_fpu_supreg = RS_S0;
      first_fpu_imreg = $20;

      { MM Super register first and last }
      first_mm_supreg    = RS_S0;
      first_mm_imreg     = $20;

      { Required parameter alignment when calling a routine declared as
        stdcall and cdecl. The alignment value should be the one defined
        by GCC or the target ABI.

        The value of this constant is equal to the constant
        PARM_BOUNDARY / BITS_PER_UNIT in the GCC source.
      }
      std_param_align = 8;

      { TODO: Calculate bsstart}
      regnumber_count_bsstart = 256;

      regnumber_table : array[tregisterindex] of tregister = (
        {$i ra64num.inc}
      );

      regstabs_table : array[tregisterindex] of shortint = (
        {$i ra64sta.inc}
      );

      regdwarf_table : array[tregisterindex] of shortint = (
        {$i ra64dwa.inc}
      );
      { registers which may be destroyed by calls }
      VOLATILE_INTREGISTERS = [RS_X0..RS_X18,RS_X30];
      VOLATILE_MMREGISTERS =  [RS_D0..RS_D7,RS_D16..RS_D31];

{*****************************************************************************
                          Instruction post fixes
*****************************************************************************}
    type
      { ARM instructions load/store and arithmetic instructions
        can have several instruction post fixes which are collected
        in this enumeration
      }
      TOpPostfix = (PF_None,
        { update condition flags }
        PF_S,
        { load/store sizes }
        PF_B,PF_SB,PF_H,PF_SH,PF_W,PF_SW
      );

      TOpPostfixes = set of TOpPostfix;

    const
      tcgsizep2size: array[OS_NO..OS_F128] of byte =
        {OS_NO }
        (0,
        {OS_8,OS_16,OS_32,OS_64,OS_128,OS_S8,OS_S16,OS_S32,OS_S64,OS_S128}
            0,    1,    2,    3,     4,    0,     1,     2,     3,      4,
        {OS_F32,OS_F64,OS_F80,OS_C64,OS_F128,}
             2,      3,     0,     3,      4);
      oppostfix2str: array[TOpPostfix] of string[2] = ('',
        's',
        'b','sb','h','sh','w','sw');

{*****************************************************************************
                                Conditions
*****************************************************************************}

    type
      TAsmCond=(C_None,
        C_EQ,C_NE,C_HS,C_LO,C_MI,C_PL,C_VS,C_VC,C_HI,C_LS,
        C_GE,C_LT,C_GT,C_LE,C_AL,C_NV
      );

      TAsmConds = set of TAsmCond;

    const
      C_CS = C_HS;
      C_CC = C_LO;
      cond2str : array[TAsmCond] of string[2]=('',
        'eq','ne','hs','lo','mi','pl','vs','vc','hi','ls',
        'ge','lt','gt','le','al','nv'
      );

      uppercond2str : array[TAsmCond] of string[2]=('',
        'EQ','NE','HS','LO','MI','PL','VS','VC','HI','LS',
        'GE','LT','GT','LE','AL','NV'
      );

{*****************************************************************************
                                   Flags
*****************************************************************************}

    type
      TResFlags = (F_EQ,F_NE,F_CS,F_CC,F_MI,F_PL,F_VS,F_VC,F_HI,F_LS,
        F_GE,F_LT,F_GT,F_LE);

    const
      F_HS = F_CS;
      F_LO = F_CC;

{*****************************************************************************
                                Operands
*****************************************************************************}

    type
      taddressmode = (AM_OFFSET,AM_PREINDEXED,AM_POSTINDEXED);

      tshiftmode = (SM_None,
                    { shifted register instructions. LSL can also be used for
                      the index register of certain loads/stores }
                    SM_LSL,SM_LSR,SM_ASR,SM_ROR,
                    { extended register instructions: zero/sign extension +
                        optional shift (interpreted as LSL after extension)
                       -- the index register of certain loads/stores can be
                          extended via (s|u)xtw with a shiftval of either 0 or
                          log2(transfer size of the load/store)
                    }
                    SM_UXTB,SM_UXTH,SM_UXTW,SM_UXTX,SM_SXTB,SM_SXTH,SM_SXTW,SM_SXTX);

      tupdatereg = (UR_None,UR_Update);

      pshifterop = ^tshifterop;

      tshifterop = record
        shiftmode : tshiftmode;
        shiftimm : byte;
      end;

{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      max_operands = 6;

      maxintregs = 32;
      maxfpuregs = 32;
      maxaddrregs = 0;

      shiftedregmodes = [SM_LSL,SM_UXTB,SM_UXTH,SM_UXTW,SM_UXTX,SM_SXTB,SM_SXTH,SM_SXTW,SM_SXTX];
      extendedregmodes = [SM_LSL,SM_LSR,SM_ASR];


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
                          Default generic sizes
*****************************************************************************}

   const
      { Defines the default address size for a processor, }
      OS_ADDR = OS_64;
      { the natural int size for a processor,
        has to match osuinttype/ossinttype as initialized in psystem }
      OS_INT = OS_64;
      OS_SINT = OS_S64;
      { the maximum float size for a processor,           }
      OS_FLOAT = OS_F64;
      { the size of a vector register for a processor     }
      OS_VECTOR = OS_M128;

{*****************************************************************************
                          Generic Register names
*****************************************************************************}


      NR_FP = NR_X29;
      RS_FP = RS_X29;
      NR_WFP = NR_W29;
      RS_WFP = RS_W29;

      NR_LR = NR_X30;
      RS_LR = RS_X30;
      NR_WLR = NR_W30;
      RS_WLR = RS_W30;

      { Stack pointer register }
      NR_STACK_POINTER_REG = NR_SP;
      RS_STACK_POINTER_REG = RS_SP;
      { Frame pointer register }
      NR_FRAME_POINTER_REG = NR_X29;
      RS_FRAME_POINTER_REG = RS_X29;
      { Register for addressing absolute data in a position independant way,
        such as in PIC code. The exact meaning is ABI specific. For
        further information look at GCC source : PIC_OFFSET_TABLE_REGNUM
      }
      NR_PIC_OFFSET_REG = NR_X18;
      { Results are returned in this register (32-bit values) }
      NR_FUNCTION_RETURN_REG = NR_X0;
      RS_FUNCTION_RETURN_REG = RS_X0;
      { The value returned from a function is available in this register }
      NR_FUNCTION_RESULT_REG = NR_FUNCTION_RETURN_REG;
      RS_FUNCTION_RESULT_REG = RS_FUNCTION_RETURN_REG;

      NR_FPU_RESULT_REG = NR_NO;

      NR_MM_RESULT_REG  = NR_D0;

      NR_RETURN_ADDRESS_REG = NR_FUNCTION_RETURN_REG;

      { Offset where the parent framepointer is pushed }
      PARENT_FRAMEPOINTER_OFFSET = 0;

      NR_DEFAULTFLAGS = NR_NZCV;
      RS_DEFAULTFLAGS = RS_NZCV;

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

    function dwarf_reg(r:tregister):shortint;
    function dwarf_reg_no_error(r:tregister):shortint;
    function eh_return_data_regno(nr: longint): longint;

    function is_shifter_const(d: aint; size: tcgsize): boolean;
    function IsFloatImmediate(ft : tfloattype;value : bestreal) : boolean;


  implementation

    uses
      systems,rgBase,verbose;

    const
      std_regname_table : TRegNameTable = (
        {$i ra64std.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i ra64rni.inc}
      );

      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i ra64sri.inc}
      );


    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
      begin
        case regtype of
          R_INTREGISTER:
            begin
              case s of
                { there's only Wn and Xn }
                OS_64,
                OS_S64:
                  cgsize2subreg:=R_SUBWHOLE;
                else
                  cgsize2subreg:=R_SUBD;
                end;
            end;
          R_MMREGISTER:
            begin
              case s of
                { records }
                OS_32,
                OS_F32:
                  cgsize2subreg:=R_SUBMMS;
                OS_64,
                OS_F64:
                  cgsize2subreg:=R_SUBMMD;
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
          R_INTREGISTER:
            case getsubreg(reg) of
              R_SUBD:
                result:=OS_32
              else
                result:=OS_64;
            end;
          R_MMREGISTER :
            begin
              case getsubreg(reg) of
                R_SUBMMD:
                  result:=OS_F64;
                R_SUBMMS:
                  result:=OS_F32;
                R_SUBMMWHOLE:
                  result:=OS_M128;
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
        is_calljmp:=o in [A_B,A_BL,A_BLR,A_RET,A_CBNZ,A_CBZ,A_TBNZ,A_TBZ];
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
        flag_2_cond: array[TResFlags] of TAsmCond =
          (C_EQ,C_NE,C_HS,C_LO,C_MI,C_PL,C_VS,C_VC,C_HI,C_LS,
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


    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      const
        inverse: array[TAsmCond] of TAsmCond=(C_None,
          C_NE,C_EQ,C_LO,C_HS,C_PL,C_MI,C_VC,C_VS,C_LS,C_HI,
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

    function is_shifter_const(d: aint; size: tcgsize): boolean;
      var
         pattern, checkpattern: qword;
         patternlen, maxbits, replicatedlen: longint;
         rightmostone, rightmostzero, checkbit, secondrightmostbit: longint;
      begin
        result:=false;
        { patterns with all bits 0 or 1 cannot be represented this way }
        if (d=0) then
          exit;
        case size of
          OS_64,
          OS_S64:
            begin
              if d=-1 then
                exit;
              maxbits:=64;
            end
          else
            begin
              if longint(d)=-1 then
                exit;
              { we'll generate a 32 bit pattern -> ignore upper sign bits in
                case of negative longint value }
              d:=cardinal(d);
              maxbits:=32;
            end;
        end;
        { "The Logical (immediate) instructions accept a bitmask immediate value
          that is a 32-bit pattern or a 64-bit pattern viewed as a vector of
          identical elements of size e = 2, 4, 8, 16, 32 or, 64 bits. Each
          element contains the same sub-pattern, that is a single run of
          1 to (e - 1) nonzero bits from bit 0 followed by zero bits, then
          rotated by 0 to (e - 1) bits." (ARMv8 ARM)

          Rather than generating all possible patterns and checking whether they
          match our constant, we check whether the lowest 2/4/8/... bits are
          a valid pattern, and if so whether the constant consists of a
          replication of this pattern. Such a valid pattern has the form of
          either (regexp notation)
            * 1+0+1*
            * 0+1+0* }
        patternlen:=2;
        while patternlen<=maxbits do
          begin
            { try lowest <patternlen> bits of d as pattern }
            if patternlen<>64 then
              pattern:=qword(d) and ((qword(1) shl patternlen)-1)
            else
              pattern:=qword(d);
            { valid pattern? If it contains too many 1<->0 transitions, larger
              parts of d cannot be a valid pattern either }
            rightmostone:=BsfQWord(pattern);
            rightmostzero:=BsfQWord(not(pattern));
            { pattern all ones or zeroes -> not a valid pattern (but larger ones
              can still be valid, since we have too few transitions) }
            if (rightmostone<patternlen) and
               (rightmostzero<patternlen) then
              begin
                if rightmostone>rightmostzero then
                  begin
                    { we have .*1*0* -> check next zero position by shifting
                      out the existing zeroes (shr rightmostone), inverting and
                      then again looking for the rightmost one position }
                    checkpattern:=not(pattern);
                    checkbit:=rightmostone;
                  end
                else
                  begin
                    { same as above, but for .*0*1* }
                    checkpattern:=pattern;
                    checkbit:=rightmostzero;
                  end;
                secondrightmostbit:=BsfQWord(checkpattern shr checkbit)+checkbit;
                { if this position is >= patternlen -> ok (1 transition),
                  otherwise we now have 2 transitions and have to check for a
                  third (if there is one, abort)

                  bsf returns 255 if no 1 bit is found, so in that case it's
                  also ok
                  }
                if secondrightmostbit<patternlen then
                  begin
                    secondrightmostbit:=BsfQWord(not(checkpattern) shr secondrightmostbit)+secondrightmostbit;
                    if secondrightmostbit<patternlen then
                      exit;
                  end;
                { ok, this is a valid pattern, now does d consist of a
                  repetition of this pattern? }
                replicatedlen:=patternlen;
                checkpattern:=pattern;
                while replicatedlen<maxbits do
                  begin
                    { douplicate current pattern }
                    checkpattern:=checkpattern or (checkpattern shl replicatedlen);
                    replicatedlen:=replicatedlen*2;
                  end;
                if qword(d)=checkpattern then
                  begin
                    { yes! }
                    result:=true;
                    exit;
                  end;
              end;
            patternlen:=patternlen*2;
          end;
      end;


  function eh_return_data_regno(nr: longint): longint;
    begin
      if (nr>=0) and (nr<2) then
        result:=nr
      else
        result:=-1;
    end;


  function IsFloatImmediate(ft : tfloattype;value : bestreal) : boolean;
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
