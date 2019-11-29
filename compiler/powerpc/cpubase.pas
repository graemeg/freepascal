{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Contains the base types for the PowerPC

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
{ This Unit contains the base types for the PowerPC
}
unit cpubase;

{$i fpcdefs.inc}

interface

uses
  strings,globtype,
  cutils,cclasses,aasmbase,cpuinfo,cgbase;


{*****************************************************************************
                                Assembler Opcodes
*****************************************************************************}

    type
      TAsmOp=(A_None,
        { normal opcodes }
        a_add, a_add_, a_addo, a_addo_, a_addc, a_addc_, a_addco, a_addco_,
        a_adde, a_adde_, a_addeo, a_addeo_, a_addi, a_addic, a_addic_, a_addis,
        a_addme, a_addme_, a_addmeo, a_addmeo_, a_addze, a_addze_, a_addzeo,
        a_addzeo_, a_and, a_and_, a_andc, a_andc_, a_andi_, a_andis_, a_b,
        a_ba, a_bl, a_bla, a_bc, a_bca, a_bcl, a_bcla, a_bcctr, a_bcctrl, a_bclr,
        a_bclrl, a_cmp, a_cmpi, a_cmpl, a_cmpli, a_cntlzw, a_cntlzw_, a_crand,
        a_crandc, a_creqv, a_crnand, a_crnor, a_cror, a_crorc, a_crxor, a_dcba,
        a_dcbf, a_dcbi, a_dcbst, a_dcbt, a_dcbtst, a_dcbz, a_divw, a_divw_, a_divwo, a_divwo_,
        a_divwu, a_divwu_, a_divwuo, a_divwuo_, a_eciwx, a_ecowx, a_eieio, a_eqv,
        a_eqv_, a_extsb, a_extsb_, a_extsh, a_extsh_, a_fabs, a_fabs_, a_fadd,
        a_fadd_, a_fadds, a_fadds_, a_fcmpo, a_fcmpu, a_fctid, a_fctid_,
        a_fctidz, a_fctidz_, a_fctiw, a_fctiw_, a_fctiwz, a_fctiwz_,
        a_fdiv, a_fdiv_, a_fdivs, a_fdivs_, a_fmadd, a_fmadd_, a_fmadds,
        a_fmadds_, a_fmr, a_fmsub, a_fmsub_, a_fmsubs, a_fmsubs_, a_fmul, a_fmul_,
        a_fmuls, a_fmuls_, a_fnabs, a_fnabs_, a_fneg, a_fneg_, a_fnmadd,
        a_fnmadd_, a_fnmadds, a_fnmadds_, a_fnmsub, a_fnmsub_, a_fnmsubs,
        a_fnmsubs_, a_fres, a_fres_, a_frsp, a_frsp_, a_frsqrte, a_frsqrte_,
        a_fsel, a_fsel_, a_fsqrt, a_fsqrt_, a_fsqrts, a_fsqrts_, a_fsub, a_fsub_,
        a_fsubs, a_fsubs_, a_icbi, a_isync, a_lbz, a_lbzu, a_lbzux, a_lbzx,
        a_lfd, a_lfdu, a_lfdux, a_lfdx, a_lfs, a_lfsu, a_lfsux, a_lfsx, a_lha,
        a_lhau, a_lhaux, a_lhax, a_lhbrx, a_lhz, a_lhzu, a_lhzux, a_lhzx, a_lmw,
        a_lswi, a_lswx, a_lwarx, a_lwbrx, a_lwz, a_lwzu, a_lwzux, a_lwzx, a_mcrf,
        a_mcrfs, a_mcrxr, a_mfcr, a_mffs, a_mffs_, a_mfmsr, a_mfspr, a_mfsr,
        a_mfsrin, a_mftb, a_mtcrf, a_mtfsb0, a_mtfsb1, a_mtfsf, a_mtfsf_,
        a_mtfsfi, a_mtfsfi_, a_mtmsr, a_mtspr, a_mtsr, a_mtsrin, a_mulhw,
        a_mulhw_, a_mulhwu, a_mulhwu_, a_mulli, a_mullw, a_mullw_, a_mullwo,
        a_mullwo_, a_nand, a_nand_, a_neg, a_neg_, a_nego, a_nego_, a_nor, a_nor_,
        a_or, a_or_, a_orc, a_orc_, a_ori, a_oris, a_rfi, a_rlwimi, a_rlwimi_,
        a_rlwinm, a_rlwinm_, a_rlwnm, a_rlwnm_, a_sc, a_slw, a_slw_, a_sraw, a_sraw_,
        a_srawi, a_srawi_,a_srw, a_srw_, a_stb, a_stbu, a_stbux, a_stbx, a_stfd,
        a_stfdu, a_stfdux, a_stfdx, a_stfiwx, a_stfs, a_stfsu, a_stfsux, a_stfsx,
        a_sth, a_sthbrx, a_sthu, a_sthux, a_sthx, a_stmw, a_stswi, a_stswx, a_stw,
        a_stwbrx, a_stwcx_, a_stwu, a_stwux, a_stwx, a_subf, a_subf_, a_subfo,
        a_subfo_, a_subfc, a_subfc_, a_subfco, a_subfco_, a_subfe, a_subfe_,
        a_subfeo, a_subfeo_, a_subfic, a_subfme, a_subfme_, a_subfmeo, a_subfmeo_,
        a_subfze, a_subfze_, a_subfzeo, a_subfzeo_, a_sync, a_tlbia, a_tlbie,
        a_tlbsync, a_tw, a_twi, a_xor, a_xor_, a_xori, a_xoris,
        { simplified mnemonics }
        a_subi, a_subis, a_subic, a_subic_, a_sub, a_sub_, a_subo, a_subo_,
        a_subc, a_subc_, a_subco, a_subco_, a_cmpwi, a_cmpw, a_cmplwi, a_cmplw,
        a_extlwi, a_extlwi_, a_extrwi, a_extrwi_, a_inslwi, a_inslwi_, a_insrwi,
        a_insrwi_, a_rotlwi, a_rotlwi_, a_rotlw, a_rotlw_, a_slwi, a_slwi_,
        a_srwi, a_srwi_, a_clrlwi, a_clrlwi_, a_clrrwi, a_clrrwi_, a_clrslwi,
        a_clrslwi_, a_blr, a_bctr, a_blrl, a_bctrl, a_crset, a_crclr, a_crmove,
        a_crnot, a_mt {move to special purpose reg}, a_mf {move from special purpose reg},
        a_nop, a_li, a_lis, a_la, a_mr, a_mr_, a_not, a_not_, a_mtcr, a_mtlr, a_mflr,
        a_mtctr, a_mfctr, a_mftbu, a_mfxer,
        a_mfpvr, a_mfdcr, a_mtdcr, a_mfdec, mtdec);

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
      tregisterindex=0..{$i rppcnor.inc}-1;

    const
      maxvarregs = 32-6; { 32 int registers - r0 - stackpointer - r2 - 3 scratch registers }
      maxfpuvarregs = 28; { 32 fpuregisters - some scratch registers (minimally 2) }
      { Available Superregisters }
      {$i rppcsup.inc}

      { No Subregisters }
      R_SUBWHOLE=R_SUBNONE;

      { Available Registers }
      {$i rppccon.inc}

      { Integer Super registers first and last }
      first_int_imreg = $20;

      { Float Super register first and last }
      first_fpu_imreg     = $20;

      { MM Super register first and last }
      first_mm_imreg     = $20;

{ TODO: Calculate bsstart}
      regnumber_count_bsstart = 64;

      regnumber_table : array[tregisterindex] of tregister = (
        {$i rppcnum.inc}
      );

      regstabs_table : array[tregisterindex] of shortint = (
        {$i rppcstab.inc}
      );

      regdwarf_table : array[tregisterindex] of shortint = (
        {$i rppcdwrf.inc}
      );

{*****************************************************************************
                                Conditions
*****************************************************************************}

    type
      TAsmCondFlag = (C_None { unconditional jumps },
        { conditions when not using ctr decrement etc }
        C_LT,C_LE,C_EQ,C_GE,C_GT,C_NL,C_NE,C_NG,C_SO,C_NS,C_UN,C_NU,
        { conditions when using ctr decrement etc }
        C_T,C_F,C_DNZ,C_DNZT,C_DNZF,C_DZ,C_DZT,C_DZF);

      TDirHint = (DH_None,DH_Minus,DH_Plus);

    const
      { these are in the XER, but when moved to CR_x they correspond with the }
      { bits below                                                            }
      C_OV = C_GT;
      C_CA = C_EQ;
      C_NO = C_NG;
      C_NC = C_NE;

    type
      TAsmCond = packed record
                   dirhint : tdirhint;
                   case simple: boolean of
                     false: (BO, BI: byte);
                     true: (
                       cond: TAsmCondFlag;
                       case byte of
                         0: ();
                         { specifies in which part of the cr the bit has to be }
                         { tested for blt,bgt,beq,..,bnu                       }
                         1: (cr: RS_CR0..RS_CR7);
                         { specifies the bit to test for bt,bf,bdz,..,bdzf }
                         2: (crbit: byte)
                       );
                 end;

    const
      AsmCondFlag2BO: Array[C_T..C_DZF] of Byte =
        (12,4,16,8,0,18,10,2);

      AsmCondFlag2BOLT_NU: Array[C_LT..C_NU] of Byte =
        (12,4,12,4,12,4,4,4,12,4,12,4);

      AsmCondFlag2BI: Array[C_LT..C_NU] of Byte =
        (0,1,2,0,1,0,2,1,3,3,3,3);

      AsmCondFlagTF: Array[TAsmCondFlag] of Boolean =
        (false,true,false,true,false,true,false,false,false,true,false,true,false,
         true,false,false,true,false,false,true,false);

      AsmCondFlag2Str: Array[TAsmCondFlag] of string[4] = ({cf_none}'',
        { conditions when not using ctr decrement etc}
        'lt','le','eq','ge','gt','nl','ne','ng','so','ns','un','nu',
        't','f','dnz','dnzt','dnzf','dz','dzt','dzf');

      UpperAsmCondFlag2Str: Array[TAsmCondFlag] of string[4] = ({cf_none}'',
        { conditions when not using ctr decrement etc}
        'LT','LE','EQ','GE','GT','NL','NE','NG','SO','NS','UN','NU',
        'T','F','DNZ','DNZT','DNZF','DZ','DZT','DZF');


{*****************************************************************************
                                   Flags
*****************************************************************************}

    type
      TResFlagsEnum = (F_EQ,F_NE,F_LT,F_LE,F_GT,F_GE,F_SO,F_FX,F_FEX,F_VX,F_OX,
                       { For IEEE-compliant floating-point compares, only <= and >=
                         are actually needed but the other two are for inverse. }
                       F_FA,F_FAE,F_FB,F_FBE);
      TResFlags = record
        cr: RS_CR0..RS_CR7;
        flag: TResFlagsEnum;
      end;

    (*
    const
      { arrays for boolean location conversions }

      flag_2_cond : array[TResFlags] of TAsmCond =
         (C_E,C_NE,C_LT,C_LE,C_GT,C_GE,???????????????);
    *)

{*****************************************************************************
                                Reference
*****************************************************************************}

    const
      { MacOS only. Whether the direct data area (TOC) directly contain
        global variables. Otherwise it contains pointers to global variables. }
      macos_direct_globals = false;

{*****************************************************************************
                                Operand Sizes
*****************************************************************************}


{*****************************************************************************
                                 Constants
*****************************************************************************}

    const
      max_operands = 5;


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

         Taken from rs6000.h (DBX_REGISTER_NUMBER)
         from GCC 3.x source code. PowerPC has 1:1 mapping
         according to the order of the registers defined
         in GCC

      }

      stab_regindex : array[tregisterindex] of shortint = (
        {$i rppcstab.inc}
      );


{*****************************************************************************
                          Generic Register names
*****************************************************************************}

      {# Stack pointer register }
      NR_STACK_POINTER_REG = NR_R1;
      RS_STACK_POINTER_REG = RS_R1;
      { old stack pointer register used during copying variables from the caller
        stack frame
      }
      NR_OLD_STACK_POINTER_REG = NR_R12;
      {# Frame pointer register }
      NR_FRAME_POINTER_REG = NR_STACK_POINTER_REG;
      RS_FRAME_POINTER_REG = RS_STACK_POINTER_REG;
      {# Register for addressing absolute data in a position independant way,
         such as in PIC code. The exact meaning is ABI specific. For
         further information look at GCC source : PIC_OFFSET_TABLE_REGNUM

         Taken from GCC rs6000.h
      }
{ TODO: As indicated in rs6000.h, but can't find it anywhere else!}
      NR_PIC_OFFSET_REG = NR_R30;
      { Return address of a function }
      NR_RETURN_ADDRESS_REG = NR_R0;
      { Results are returned in this register (32-bit values) }
      NR_FUNCTION_RETURN_REG = NR_R3;
      RS_FUNCTION_RETURN_REG = RS_R3;
      { Low part of 64bit return value }
      NR_FUNCTION_RETURN64_LOW_REG = NR_R4;
      RS_FUNCTION_RETURN64_LOW_REG = RS_R4;
      { High part of 64bit return value }
      NR_FUNCTION_RETURN64_HIGH_REG = NR_R3;
      RS_FUNCTION_RETURN64_HIGH_REG = RS_R3;
      { The value returned from a function is available in this register }
      NR_FUNCTION_RESULT_REG = NR_FUNCTION_RETURN_REG;
      RS_FUNCTION_RESULT_REG = RS_FUNCTION_RETURN_REG;
      { The lowh part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_LOW_REG = NR_FUNCTION_RETURN64_LOW_REG;
      RS_FUNCTION_RESULT64_LOW_REG = RS_FUNCTION_RETURN64_LOW_REG;
      { The high part of 64bit value returned from a function }
      NR_FUNCTION_RESULT64_HIGH_REG = NR_FUNCTION_RETURN64_HIGH_REG;
      RS_FUNCTION_RESULT64_HIGH_REG = RS_FUNCTION_RETURN64_HIGH_REG;

      NR_FPU_RESULT_REG = NR_F1;
      NR_MM_RESULT_REG = NR_M0;

      NR_DEFAULTFLAGS = NR_CR;
      RS_DEFAULTFLAGS = RS_CR;


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

      LinkageAreaSizeAIX = 24;
      LinkageAreaSizeSYSV = 8;
      { offset in the linkage area for the saved stack pointer }
      LA_SP = 0;
      { offset in the linkage area for the saved conditional register}
      LA_CR_AIX = 4;
      { offset in the linkage area for the saved link register}
      LA_LR_AIX = 8;
      LA_LR_SYSV = 4;
      { offset in the linkage area for the saved RTOC register}
      LA_RTOC_AIX = 20;
      LA_RTOC_ELFV2 = 12;

      PARENT_FRAMEPOINTER_OFFSET = 12;

      NR_RTOC = NR_R2;

      maxfpuregs = 8;

      { minimum size of the stack frame if one exists }
      MINIMUM_STACKFRAME_SIZE = 56;

{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function  is_calljmp(o:tasmop):boolean;

    procedure inverse_flags(var r : TResFlags);
    function  flags_to_cond(const f: TResFlags) : TAsmCond;
    procedure create_cond_imm(BO,BI:byte;var r : TAsmCond);
    procedure create_cond_norm(cond: TAsmCondFlag; cr: byte;var r : TasmCond);

    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
    { Returns the tcgsize corresponding with the size of reg.}
    function reg_cgsize(const reg: tregister) : tcgsize;

    function findreg_by_number(r:Tregister):tregisterindex;
    function std_regnum_search(const s:string):Tregister;
    function std_regname(r:Tregister):string;
    function is_condreg(r : tregister):boolean;

    function inverse_cond(const c: TAsmCond): Tasmcond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
    function conditions_equal(const c1, c2: TAsmCond): boolean;

    { Checks if Subset is a subset of c (e.g. "less than" is a subset of "less than or equal" }
    function condition_in(const Subset, c: TAsmCond): Boolean;

    function dwarf_reg(r:tregister):shortint;
    function dwarf_reg_no_error(r:tregister):shortint;
    function eh_return_data_regno(nr: longint): longint;

implementation

    uses
      rgbase,verbose;

    const
      std_regname_table : TRegNameTable = (
        {$i rppcstd.inc}
      );

      regnumber_index : array[tregisterindex] of tregisterindex = (
        {$i rppcrni.inc}
      );

      std_regname_index : array[tregisterindex] of tregisterindex = (
        {$i rppcsri.inc}
      );


{*****************************************************************************
                                  Helpers
*****************************************************************************}

    function is_calljmp(o:tasmop):boolean;
      begin
       is_calljmp:=false;
        case o of
          A_B,A_BA,A_BL,A_BLA,A_BC,A_BCA,A_BCL,A_BCLA,A_BCCTR,A_BCCTRL,A_BCLR,
            A_BCLRL,A_TW,A_TWI: is_calljmp:=true;
          else
            ;
        end;
      end;


    procedure inverse_flags(var r: TResFlags);
      const
        inv_flags: array[F_EQ..F_GE] of TResFlagsEnum =
          (F_NE,F_EQ,F_GE,F_GE,F_LE,F_LT);
        inv_fpuflags: array[F_FA..F_FBE] of TResFlagsEnum =
          (F_FBE,F_FB,F_FAE,F_FA);
      begin
        if r.flag in [F_EQ..F_GE] then
          r.flag := inv_flags[r.flag]
        else if r.flag in [F_FA..F_FBE] then
          r.flag := inv_fpuflags[r.flag]
        else
          internalerror(2014041901);
      end;


    function inverse_cond(const c: TAsmCond): Tasmcond; {$ifdef USEINLINE}inline;{$endif USEINLINE}
      const
        inv_condflags:array[TAsmCondFlag] of TAsmCondFlag=(C_None,
          C_GE,C_GT,C_NE,C_LT,C_LE,C_LT,C_EQ,C_GT,C_NS,C_SO,C_NU,C_UN,
          C_F,C_T,C_DNZ,C_DNZF,C_DNZT,C_DZ,C_DZF,C_DZT);
      begin
        if (c.cond in [C_DNZ,C_DZ]) then
          internalerror(2005022501);
        result := c;
        result.cond := inv_condflags[c.cond];
      end;


    function conditions_equal(const c1, c2: TAsmCond): boolean;
      begin
        result :=
          (c1.simple and c2.simple) and
          (c1.cond = c2.cond) and
          ((not(c1.cond in [C_T..C_DZF]) and
           (c1.cr = c2.cr)) or
           (c1.crbit = c2.crbit));
      end;


    { Checks if Subset is a subset of c (e.g. "less than" is a subset of "less than or equal" }
    function condition_in(const Subset, c: TAsmCond): Boolean;
      begin
        Result := (c.cond = C_None) or conditions_equal(Subset, c);

        { TODO: Can a PowerPC programmer please update this procedure to
          actually detect subsets? Thanks. [Kit] }
      end;


    function flags_to_cond(const f: TResFlags) : TAsmCond;
      const
        flag_2_cond: array[F_EQ..F_SO] of TAsmCondFlag =
          (C_EQ,C_NE,C_LT,C_LE,C_GT,C_GE,C_SO);
      begin
        if f.flag > high(flag_2_cond) then
          internalerror(200112301);
        result.simple := true;
        result.cr := f.cr;
        result.cond := flag_2_cond[f.flag];
      end;


    procedure create_cond_imm(BO,BI:byte;var r : TAsmCond);
      begin
        r.simple := false;
        r.bo := bo;
        r.bi := bi;
      end;


    procedure create_cond_norm(cond: TAsmCondFlag; cr: byte;var r : TasmCond);
      begin
        r.dirhint := DH_None;
        r.simple := true;
        r.cond := cond;
        case cond of
          C_NONE:;
          C_T..C_DZF: r.crbit := cr
          else r.cr := RS_CR0+cr;
        end;
      end;


    function is_condreg(r : tregister):boolean;
      var
        supreg: tsuperregister;
      begin
        result := false;
        if (getregtype(r) = R_SPECIALREGISTER) then
          begin
             supreg := getsupreg(r);
             result := (supreg >= RS_CR0) and (supreg <= RS_CR7);
          end;
      end;


    function reg_cgsize(const reg: tregister): tcgsize;
      begin
        case getregtype(reg) of
          R_INTREGISTER :
            result:=OS_32;
          R_MMREGISTER:
            result:=OS_M128;
          R_FPUREGISTER:
            result:=OS_F64;
          else
            internalerror(200303181);
        end;
      end;


    function cgsize2subreg(regtype: tregistertype; s:Tcgsize):Tsubregister;
      begin
        cgsize2subreg:=R_SUBWHOLE;
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
          result:=nr+3
        else
          result:=-1;
      end;

end.
