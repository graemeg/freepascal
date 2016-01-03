{
    Copyright (c) 1998-2002 by Florian Klaempfl and Peter Vreman

    Contains the base types for the i8086, i386 and x86-64 architecture

    * This code was inspired by the NASM sources
      The Netwide Assembler is Copyright (c) 1996 Simon Tatham and
      Julian Hall. All rights reserved.

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
  cgbase
  ;


{*****************************************************************************
                                Assembler Opcodes
*****************************************************************************}

    type
{$if defined(x86_64)}
      TAsmOp={$i x8664op.inc}
{$elseif defined(i386)}
      TAsmOp={$i i386op.inc}
{$elseif defined(i8086)}
      TAsmOp={$i i8086op.inc}
{$endif}

      { This should define the array of instructions as string }
        op2strtable=array[tasmop] of string[16];

    const
      { First value of opcode enumeration }
      firstop = low(tasmop);
      { Last value of opcode enumeration  }
      lastop  = high(tasmop);

{*****************************************************************************
                                  Registers
*****************************************************************************}

   const
      { Integer Super registers }
      RS_NO         = $ffffffff;
      RS_RAX        = $00;      {EAX}
      RS_RCX        = $01;      {ECX}
      RS_RDX        = $02;      {EDX}
      RS_RBX        = $03;      {EBX}
      RS_RSI        = $04;      {ESI}
      RS_RDI        = $05;      {EDI}
      RS_RBP        = $06;      {EBP}
      RS_RSP        = $07;      {ESP}
      RS_R8         = $08;      {R8}
      RS_R9         = $09;      {R9}
      RS_R10        = $0a;      {R10}
      RS_R11        = $0b;      {R11}
      RS_R12        = $0c;      {R12}
      RS_R13        = $0d;      {R13}
      RS_R14        = $0e;      {R14}
      RS_R15        = $0f;      {R15}
      { create aliases to allow code sharing between x86-64 and i386 }
      RS_EAX        = RS_RAX;
      RS_EBX        = RS_RBX;
      RS_ECX        = RS_RCX;
      RS_EDX        = RS_RDX;
      RS_ESI        = RS_RSI;
      RS_EDI        = RS_RDI;
      RS_EBP        = RS_RBP;
      RS_ESP        = RS_RSP;
      { create aliases to allow code sharing between i386 and i8086 }
      RS_AX        = RS_RAX;
      RS_BX        = RS_RBX;
      RS_CX        = RS_RCX;
      RS_DX        = RS_RDX;
      RS_SI        = RS_RSI;
      RS_DI        = RS_RDI;
      RS_BP        = RS_RBP;
      RS_SP        = RS_RSP;

      { Number of first imaginary register }
      first_int_imreg     = $10;

      { Float Super registers }
      RS_ST0        = $00;
      RS_ST1        = $01;
      RS_ST2        = $02;
      RS_ST3        = $03;
      RS_ST4        = $04;
      RS_ST5        = $05;
      RS_ST6        = $06;
      RS_ST7        = $07;
      RS_ST         = $08;

      { Number of first imaginary register }
      first_fpu_imreg     = $09;

      { MM Super registers }
      RS_XMM0        = $00;
      RS_XMM1        = $01;
      RS_XMM2        = $02;
      RS_XMM3        = $03;
      RS_XMM4        = $04;
      RS_XMM5        = $05;
      RS_XMM6        = $06;
      RS_XMM7        = $07;
      RS_XMM8        = $08;
      RS_XMM9        = $09;
      RS_XMM10       = $0a;
      RS_XMM11       = $0b;
      RS_XMM12       = $0c;
      RS_XMM13       = $0d;
      RS_XMM14       = $0e;
      RS_XMM15       = $0f;

      RS_FLAGS       = $07;

      { Number of first imaginary register }
{$ifdef x86_64}
      first_mm_imreg     = $10;
{$else x86_64}
      first_mm_imreg     = $08;
{$endif x86_64}

      { The subregister that specifies the entire register and an address }
{$if defined(x86_64)}
      { Hammer }
      R_SUBWHOLE    = R_SUBQ;
      R_SUBADDR     = R_SUBQ;
{$elseif defined(i386)}
      { i386 }
      R_SUBWHOLE    = R_SUBD;
      R_SUBADDR     = R_SUBD;
{$elseif defined(i8086)}
      { i8086 }
      R_SUBWHOLE    = R_SUBW;
      R_SUBADDR     = R_SUBW;
{$endif}

      { Available Registers }
{$if defined(x86_64)}
      {$i r8664con.inc}
{$elseif defined(i386)}
      {$i r386con.inc}
{$elseif defined(i8086)}
      {$i r8086con.inc}
{$endif}

    type
      { Number of registers used for indexing in tables }
{$if defined(x86_64)}
      tregisterindex=0..{$i r8664nor.inc}-1;
{$elseif defined(i386)}
      tregisterindex=0..{$i r386nor.inc}-1;
{$elseif defined(i8086)}
      tregisterindex=0..{$i r8086nor.inc}-1;
{$endif}

    const
{ TODO: Calculate bsstart}
      regnumber_count_bsstart = 64;

      regnumber_table : array[tregisterindex] of tregister = (
{$if defined(x86_64)}
        {$i r8664num.inc}
{$elseif defined(i386)}
        {$i r386num.inc}
{$elseif defined(i8086)}
        {$i r8086num.inc}
{$endif}
      );

      regstabs_table : array[tregisterindex] of shortint = (
{$if defined(x86_64)}
        {$i r8664stab.inc}
{$elseif defined(i386)}
        {$i r386stab.inc}
{$elseif defined(i8086)}
        {$i r8086stab.inc}
{$endif}
      );

      regdwarf_table : array[tregisterindex] of shortint = (
{$if defined(x86_64)}
        {$i r8664dwrf.inc}
{$elseif defined(i386)}
        {$i r386dwrf.inc}
{$elseif defined(i8086)}
        {$i r8086dwrf.inc}
{$endif}
      );

      RS_DEFAULTFLAGS = RS_FLAGS;
      NR_DEFAULTFLAGS = NR_FLAGS;

   type
      totherregisterset = set of tregisterindex;


{*****************************************************************************
                                Conditions
*****************************************************************************}

    type
      TAsmCond=(C_None,
        C_A,C_AE,C_B,C_BE,C_C,C_E,C_G,C_GE,C_L,C_LE,C_NA,C_NAE,
        C_NB,C_NBE,C_NC,C_NE,C_NG,C_NGE,C_NL,C_NLE,C_NO,C_NP,
        C_NS,C_NZ,C_O,C_P,C_PE,C_PO,C_S,C_Z
      );

    const
      cond2str:array[TAsmCond] of string[3]=('',
        'a','ae','b','be','c','e','g','ge','l','le','na','nae',
        'nb','nbe','nc','ne','ng','nge','nl','nle','no','np',
        'ns','nz','o','p','pe','po','s','z'
      );

{*****************************************************************************
                                   Flags
*****************************************************************************}

    type
      TResFlags = (F_E,F_NE,F_G,F_L,F_GE,F_LE,F_C,F_NC,
                   F_A,F_AE,F_B,F_BE,
                   F_S,F_NS,F_O,F_NO,
                   { For IEEE-compliant floating-point compares,
                     same as normal counterparts but additionally check PF }
                   F_FE,F_FNE,F_FA,F_FAE,F_FB,F_FBE);

    const
      FPUFlags = [F_FE,F_FNE,F_FA,F_FAE,F_FB,F_FBE];
      FPUFlags2Flags: array[F_FE..F_FBE] of TResFlags = (
        F_E,F_NE,F_A,F_AE,F_B,F_BE
      );

{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      { declare aliases }
      LOC_SSEREGISTER = LOC_MMREGISTER;
      LOC_CSSEREGISTER = LOC_CMMREGISTER;

      max_operands = 4;
      maxfpuregs = 8;

{*****************************************************************************
                            CPU Dependent Constants
*****************************************************************************}

    {$i cpubase.inc}

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
    function reg2opsize(r:Tregister):topsize;
    function reg_cgsize(const reg: tregister): tcgsize;
    function is_calljmp(o:tasmop):boolean;
    procedure inverse_flags(var f: TResFlags);
    function flags_to_cond(const f: TResFlags) : TAsmCond;
    function is_segment_reg(r:tregister):boolean;
    function findreg_by_number(r:Tregister):tregisterindex;
    function std_regnum_search(const s:string):Tregister;
    function std_regname(r:Tregister):string;
    function dwarf_reg(r:tregister):shortint;

    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    function conditions_equal(const c1, c2: TAsmCond): boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}

    { checks whether two segment registers are normally equal in the current memory model }
    function segment_regs_equal(r1,r2:tregister):boolean;

{$ifdef i8086}
    { returns the next virtual register }
    function GetNextReg(const r : TRegister) : TRegister;

    { return whether we need to add an extra FWAIT instruction before the given
      instruction, when we're targeting the i8087. This includes almost all x87
      instructions, but certain ones, which always have or have not a built in
      FWAIT prefix are excluded (e.g. FINIT,FNINIT,etc.). }
    function requires_fwait_on_8087(op: TAsmOp): boolean;
{$endif i8086}

implementation

    uses
      rgbase,verbose;

    const
    {$if defined(x86_64)}
      std_regname_table : TRegNameTable = (
        {$i r8664std.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i r8664rni.inc}
      );
      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i r8664sri.inc}
      );
    {$elseif defined(i386)}
      std_regname_table : TRegNameTable = (
        {$i r386std.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i r386rni.inc}
      );

      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i r386sri.inc}
      );
    {$elseif defined(i8086)}
      std_regname_table : TRegNameTable = (
        {$i r8086std.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i r8086rni.inc}
      );

      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i r8086sri.inc}
      );
    {$endif}


{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
      begin
        case s of
          OS_8,OS_S8:
            cgsize2subreg:=R_SUBL;
          OS_16,OS_S16:
            cgsize2subreg:=R_SUBW;
          OS_32,OS_S32:
            cgsize2subreg:=R_SUBD;
          OS_64,OS_S64:
            cgsize2subreg:=R_SUBQ;
          OS_M64:
            cgsize2subreg:=R_SUBNONE;
          OS_F32,OS_F64,OS_C64:
            case regtype of
              R_FPUREGISTER:
                cgsize2subreg:=R_SUBWHOLE;
              R_MMREGISTER:
                case s of
                  OS_F32:
                    cgsize2subreg:=R_SUBMMS;
                  OS_F64:
                    cgsize2subreg:=R_SUBMMD;
                  else
                    internalerror(2009071901);
                end;
              else
                internalerror(2009071902);
            end;
          OS_M128,OS_MS128:
            cgsize2subreg:=R_SUBMMX;
          OS_M256,OS_MS256:
            cgsize2subreg:=R_SUBMMY;
          else
            internalerror(200301231);
        end;
      end;


    function reg_cgsize(const reg: tregister): tcgsize;
      const subreg2cgsize:array[Tsubregister] of Tcgsize =
            (OS_NO,OS_8,OS_8,OS_16,OS_32,OS_64,OS_NO,OS_NO,OS_NO,OS_F32,OS_F64,OS_NO,OS_M128,OS_M256);
      begin
        case getregtype(reg) of
          R_INTREGISTER :
            reg_cgsize:=subreg2cgsize[getsubreg(reg)];
          R_FPUREGISTER :
            reg_cgsize:=OS_F80;
          R_MMXREGISTER:
            reg_cgsize:=OS_M64;
          R_MMREGISTER:
            reg_cgsize:=subreg2cgsize[getsubreg(reg)];
          R_SPECIALREGISTER :
            case reg of
              NR_CS,NR_DS,NR_ES,NR_SS,NR_FS,NR_GS:
                reg_cgsize:=OS_16;
{$ifdef x86_64}
              NR_DR0..NR_TR7:
                reg_cgsize:=OS_64;
{$endif x86_64}
              else
                reg_cgsize:=OS_32
            end
          else
            internalerror(2003031801);
          end;
        end;


    function reg2opsize(r:Tregister):topsize;
      const
        subreg2opsize : array[tsubregister] of topsize =
          (S_NO,S_B,S_B,S_W,S_L,S_Q,S_NO,S_NO,S_NO,S_NO,S_NO,S_NO,S_NO,S_NO);
      begin
        reg2opsize:=S_L;
        case getregtype(r) of
          R_INTREGISTER :
            reg2opsize:=subreg2opsize[getsubreg(r)];
          R_FPUREGISTER :
            reg2opsize:=S_FL;
          R_MMXREGISTER,
          R_MMREGISTER :
            reg2opsize:=S_MD;
          R_SPECIALREGISTER :
            begin
              case r of
                NR_CS,NR_DS,NR_ES,
                NR_SS,NR_FS,NR_GS :
                  reg2opsize:=S_W;
              end;
            end;
          else
            internalerror(200303181);
        end;
      end;


    function is_calljmp(o:tasmop):boolean;
      begin
        case o of
          A_CALL,
{$if defined(i386) or defined(i8086)}
          A_JCXZ,
{$endif defined(i386) or defined(i8086)}
          A_JECXZ,
{$ifdef x86_64}
          A_JRCXZ,
{$endif x86_64}
          A_JMP,
          A_LOOP,
          A_LOOPE,
          A_LOOPNE,
          A_LOOPNZ,
          A_LOOPZ,
          A_LCALL,
          A_LJMP,
          A_Jcc :
            is_calljmp:=true;
          else
            is_calljmp:=false;
        end;
      end;


    procedure inverse_flags(var f: TResFlags);
      const
        inv_flags: array[TResFlags] of TResFlags =
          (F_NE,F_E,F_LE,F_GE,F_L,F_G,F_NC,F_C,
           F_BE,F_B,F_AE,F_A,
           F_NS,F_S,F_NO,F_O,
           F_FNE,F_FE,F_FBE,F_FB,F_FAE,F_FA);
      begin
        f:=inv_flags[f];
      end;


    function flags_to_cond(const f: TResFlags) : TAsmCond;
      const
        flags_2_cond : array[TResFlags] of TAsmCond =
          (C_E,C_NE,C_G,C_L,C_GE,C_LE,C_C,C_NC,C_A,C_AE,C_B,C_BE,C_S,C_NS,C_O,C_NO,
           C_None,C_None,C_None,C_None,C_None,C_None);
      begin
        result := flags_2_cond[f];
        if (result=C_None) then
          InternalError(2014041301);
      end;


    function is_segment_reg(r:tregister):boolean;
      begin
        result:=false;
        case r of
          NR_CS,NR_DS,NR_ES,
          NR_SS,NR_FS,NR_GS :
            result:=true;
        end;
      end;


    function findreg_by_number(r:Tregister):tregisterindex;
      var
        hr : tregister;
      begin
        { for the name the sub reg doesn't matter }
        hr:=r;
        if (getregtype(hr)=R_MMREGISTER) and
           (getsubreg(hr)<>R_SUBMMY) then
          setsubreg(hr,R_SUBMMX);
        result:=findreg_by_number_table(hr,regnumber_index);
      end;


    function std_regnum_search(const s:string):Tregister;
      begin
        result:=regnumber_table[findreg_by_name_table(s,std_regname_table,std_regname_index)];
      end;


    function std_regname(r:Tregister):string;
      var
        p : tregisterindex;
      begin
        if getregtype(r) in [R_MMREGISTER,R_MMXREGISTER] then
          r:=newreg(getregtype(r),getsupreg(r),R_SUBNONE);
<<<<<<< HEAD
<<<<<<< HEAD
        p:=findreg_by_number(r);
=======
=======
>>>>>>> origin/fixes_2_2
        p:=findreg_by_number_table(r,regnumber_index);
>>>>>>> graemeg/fixes_2_2
        if p<>0 then
          result:=std_regname_table[p]
        else
          result:=generic_regname(r);
      end;


    function inverse_cond(const c: TAsmCond): TAsmCond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      const
        inverse: array[TAsmCond] of TAsmCond=(C_None,
          C_NA,C_NAE,C_NB,C_NBE,C_NC,C_NE,C_NG,C_NGE,C_NL,C_NLE,C_A,C_AE,
          C_B,C_BE,C_C,C_E,C_G,C_GE,C_L,C_LE,C_O,C_P,
          C_S,C_Z,C_NO,C_NP,C_NP,C_P,C_NS,C_NZ
        );
      begin
        result := inverse[c];
      end;


    function conditions_equal(const c1, c2: TAsmCond): boolean; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      begin
        result := c1 = c2;
      end;


    function dwarf_reg(r:tregister):shortint;
      begin
        result:=regdwarf_table[findreg_by_number(r)];
        if result=-1 then
          internalerror(200603251);
      end;


    function segment_regs_equal(r1, r2: tregister): boolean;
      begin
        if not is_segment_reg(r1) or not is_segment_reg(r2) then
          internalerror(2013062301);
        { every segment register is equal to itself }
        if r1=r2 then
          exit(true);
{$if defined(i8086)}
        case current_settings.x86memorymodel of
          mm_tiny:
            begin
              { CS=DS=SS }
              if ((r1=NR_CS) or (r1=NR_DS) or (r1=NR_SS)) and
                 ((r2=NR_CS) or (r2=NR_DS) or (r2=NR_SS)) then
                exit(true);
              { the remaining are distinct from each other }
              exit(false);
            end;
          mm_small,mm_medium:
            begin
              { DS=SS }
              if ((r1=NR_DS) or (r1=NR_SS)) and
                 ((r2=NR_DS) or (r2=NR_SS)) then
                exit(true);
              { the remaining are distinct from each other }
              exit(false);
            end;
          mm_compact,mm_large,mm_huge:
            { all segment registers are different in these models }
            exit(false);
          else
            internalerror(2013062302);
        end;
{$elseif defined(i386) or defined(x86_64)}
        { DS=SS=ES }
        if ((r1=NR_DS) or (r1=NR_SS) or (r1=NR_ES)) and
           ((r2=NR_DS) or (r2=NR_SS) or (r2=NR_ES)) then
          exit(true);
        { the remaining are distinct from each other }
        exit(false);
{$endif}
      end;


{$ifdef i8086}
    function GetNextReg(const r: TRegister): TRegister;
      begin
        if getsupreg(r)<first_int_imreg then
          internalerror(2013051401);
        result:=TRegister(longint(r)+1);
      end;

    function requires_fwait_on_8087(op: TAsmOp): boolean;
      begin
        case op of
            A_F2XM1,A_FABS,A_FADD,A_FADDP,A_FBLD,A_FBSTP,A_FCHS,A_FCOM,A_FCOMP,
            A_FCOMPP,A_FDECSTP,A_FDIV,A_FDIVP,A_FDIVR,A_FDIVRP,
            A_FFREE,A_FIADD,A_FICOM,A_FICOMP,A_FIDIV,A_FIDIVR,A_FILD,
            A_FIMUL,A_FINCSTP,A_FIST,A_FISTP,A_FISUB,A_FISUBR,A_FLD,A_FLD1,
            A_FLDCW,A_FLDENV,A_FLDL2E,A_FLDL2T,A_FLDLG2,A_FLDLN2,A_FLDPI,A_FLDZ,
            A_FMUL,A_FMULP,A_FNOP,A_FPATAN,A_FPREM,A_FPTAN,A_FRNDINT,
            A_FRSTOR,A_FSCALE,A_FSQRT,A_FST,
            A_FSTP,A_FSUB,A_FSUBP,A_FSUBR,A_FSUBRP,A_FTST,
            A_FXAM,A_FXCH,A_FXTRACT,A_FYL2X,A_FYL2XP1:
              result:=true;
          else
            result:=false;
        end;
      end;
{$endif i8086}


end.
