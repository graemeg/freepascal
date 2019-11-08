{

  Copyright (C) <avx-testfile-generator> <Torsten Grundke>

  This source is free software; you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This code is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  A copy of the GNU General Public License is available on the World Wide Web
  at <http://www.gnu.org/copyleft/gpl.html>. You can also obtain it by writing
  to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
  MA 02110-1301, USA.
}

{$mode objfpc}

unit asmtestgenerator;

interface

uses BaseList, Classes;

type
  TOpType = (otUnknown, otXMMReg, otXMMRM, otXMMRM16, otXMMRM8, otYMMReg, otYMMRM, otZMMReg, otZMMRM, otEAX, otRAX, otMem32,
             otMem8, otMem16, otMem64, otMem128, otMem256, otMem512, otREG64, otREG32, otREG16, otREG8, otRM32, otRM64, otIMM8,
             otXMEM32, otXMEM64, otYMEM32, otYMEM64, otZMEM32, otZMEM64,
             otB32, otB64, otKREG);

  TOperandListItem = class(TObject)
  private
    FOpActive: boolean;
    FOpNumber: integer;
    FOpTyp: TOpType;
    FValues: TStringList;
  public
    constructor Create;
    destructor Destroy; override;

    property OpNumber: integer read FOpNumber write FOpNumber;
    property OpTyp: TOpType read FOpTyp write FOpTyp;
    property OpActive: boolean read FOpActive write FOpActive;

    property Values: TStringList read FValues;
  end;

  TOperandList = class(TBaseList)
  private
    function GetItems(aIndex: integer): TOperandListItem;

  public
    function Add(aItem: TOperandListItem): integer;

    property Items[aIndex: integer]: TOperandListItem read GetItems;
  end;


  { TAsmTestGenerator }

  TAsmTestGenerator = class(TObject)
  private
    FReg8          : TStringList;
    FReg16         : TStringList;
    FReg32Base     : TStringList;
    FReg32Index    : TStringList;
    FReg64Base     : TStringList;
    FReg64Index    : TStringList;
    FReg6432Base   : TStringList;
    FReg6432Index  : TStringList;
    FReg32XMMIndex : TStringList;
    FReg32YMMIndex : TStringList;
    FReg32ZMMIndex : TStringList;
    FReg64XMMIndex : TStringList;
    FReg64YMMIndex : TStringList;
    FReg64ZMMIndex : TStringList;
    FRegKREG       : TStringList;

    Fx64: boolean;
    FAVX512: boolean;
    FSAE: boolean;

    procedure MemRegBaseIndexCombi(const aPrefix, aSuffix: String; aSLBaseReg, aSLIndexReg, aRList: TStringList);
    procedure VectorMemRegBaseIndexCombi(const aPrefix, aSuffix: String; aSLBaseReg, aSLIndexReg, aRList: TStringList);

    function InternalCalcTestData(const aInst, aOp1, aOp2, aOp3, aOp4: String): TStringList;
  public
    constructor Create;
    destructor Destroy; override;

    class procedure CalcTestData(aX64, aAVX512, aSAE: boolean; const aInst, aOp1, aOp2, aOp3, aOp4: String; aSL: TStringList);

    class procedure CalcTestInstFile;

    property x64: boolean read Fx64;
  end;

implementation

uses SysUtils, Dialogs;

const
  instabentries = {$i ../../../compiler/x86_64/x8664nop.inc}

type
      TAsmOp={$i ../../../compiler/x86_64/x8664op.inc}

    op2strtable=array[tasmop] of string[16];

    {Instruction flags }
    tinsflag = (
      { please keep these in order and in sync with IF_SMASK }
      IF_SM,                  { size match first two operands  }
      IF_SM2,
      IF_SB,                  { unsized operands can't be non-byte  }
      IF_SW,                  { unsized operands can't be non-word  }
      IF_SD,                  { unsized operands can't be nondword  }

      { unsized argument spec }
      { please keep these in order and in sync with IF_ARMASK }
      IF_AR0,                 { SB, SW, SD applies to argument 0  }
      IF_AR1,                 { SB, SW, SD applies to argument 1  }
      IF_AR2,                 { SB, SW, SD applies to argument 2  }

      IF_PRIV,                { it's a privileged instruction  }
      IF_SMM,                 { it's only valid in SMM  }
      IF_PROT,                { it's protected mode only  }
      IF_NOX86_64,            { removed instruction in x86_64  }
      IF_UNDOC,               { it's an undocumented instruction  }
      IF_FPU,                 { it's an FPU instruction  }
      IF_MMX,                 { it's an MMX instruction  }
      { it's a 3DNow! instruction  }
      IF_3DNOW,
      { it's a SSE (KNI, MMX2) instruction  }
      IF_SSE,
      { SSE2 instructions  }
      IF_SSE2,
      { SSE3 instructions  }
      IF_SSE3,
      { SSE64 instructions  }
      IF_SSE64,
      { SVM instructions  }
      IF_SVM,
      { SSE4 instructions  }
      IF_SSE4,
      IF_SSSE3,
      IF_SSE41,
      IF_SSE42,
      IF_MOVBE,
      IF_CLMUL,
      IF_AVX,
      IF_AVX2,
      IF_AVX512,
      IF_BMI1,
      IF_BMI2,
      { Intel ADX (Multi-Precision Add-Carry Instruction Extensions) }
      IF_ADX,
      IF_16BITONLY,
      IF_FMA,
      IF_FMA4,
      IF_TSX,
      IF_RAND,
      IF_XSAVE,
      IF_PREFETCHWT1,

      { mask for processor level }
      { please keep these in order and in sync with IF_PLEVEL }
      IF_8086,                { 8086 instruction  }
      IF_186,                 { 186+ instruction  }
      IF_286,                 { 286+ instruction  }
      IF_386,                 { 386+ instruction  }
      IF_486,                 { 486+ instruction  }
      IF_PENT,                { Pentium instruction  }
      IF_P6,                  { P6 instruction  }
      IF_KATMAI,              { Katmai instructions  }
      IF_WILLAMETTE,          { Willamette instructions }
      IF_PRESCOTT,            { Prescott instructions }
      IF_X86_64,
      IF_SANDYBRIDGE,         { Sandybridge-specific instruction }
      IF_NEC,                 { NEC V20/V30 instruction }

      { the following are not strictly part of the processor level, because
        they are never used standalone, but always in combination with a
        separate processor level flag. Therefore, they use bits outside of
        IF_PLEVEL, otherwise they would mess up the processor level they're
        used in combination with.
        The following combinations are currently used:
        [IF_AMD, IF_P6],
        [IF_CYRIX, IF_486],
        [IF_CYRIX, IF_PENT],
        [IF_CYRIX, IF_P6] }
      IF_CYRIX,               { Cyrix, Centaur or VIA-specific instruction }
      IF_AMD,                 { AMD-specific instruction  }

      { added flags }
      IF_PRE,                 { it's a prefix instruction }
      IF_PASS2,               { if the instruction can change in a second pass }
      IF_IMM4,                { immediate operand is a nibble (must be in range [0..15]) }
      IF_IMM3,                 { immediate operand is a triad (must be in range [0..7]) }

      IF_BCST2,
      IF_BCST4,
      IF_BCST8,
      IF_BCST16,
      IF_T2,                  { disp8 - tuple - 2 }
      IF_T4,                  { disp8 - tuple - 4 }
      IF_T8,                  { disp8 - tuple - 8 }
      IF_T1S,                 { disp8 - tuple - 1 scalar }
      IF_T1F32,
      IF_T1F64,
      IF_TMDDUP,
      IF_TFV,                 { disp8 - tuple - full vector }
      IF_TFVM,                { disp8 - tuple - full vector memory }
      IF_TQVM,
      IF_TMEM128,
      IF_THV,
      IF_THVM,
      IF_TOVM,

      IF_SCL32,
      IF_SCL64


    );
    tinsflags=set of tinsflag;


      tinsentry=packed record
        opcode  : tasmop;
        ops     : byte;
        //optypes : array[0..max_operands-1] of longint;
        optypes : array[0..3] of int64; //TG
        code    : array[0..11] of char;
        flags   : tinsflags;
      end;
      pinsentry=^tinsentry;

const
      OT_NONE      = $00000000;

      { Bits 0..7: sizes }
      OT_BITS8     = $00000001;
      OT_BITS16    = $00000002;
      OT_BITS32    = $00000004;
      OT_BITS64    = $00000008;  { x86_64 and FPU }
      //OT_BITS128   = $10000000;  { 16 byte SSE }
      //OT_BITS256   = $20000000;  { 32 byte AVX }
      //OT_BITS512   = $40000000;  { 64 byte AVX512 }
      OT_BITS128   = $20000000;  { 16 byte SSE }
      OT_BITS256   = $40000000;  { 32 byte AVX }
      OT_BITS512   = $80000000;  { 64 byte AVX512 }

      OT_VECTORMASK = $1000000000;  { OPTIONAL VECTORMASK AVX512}
      OT_VECTORZERO = $2000000000;  { OPTIONAL ZERO-FLAG  AVX512}
      OT_VECTORBCST = $4000000000;  { BROADCAST-MEM-FLAG  AVX512}
      OT_VECTORSAE  = $8000000000;  { OPTIONAL SAE-FLAG  AVX512}
      OT_VECTORER   = $10000000000; { OPTIONAL ER-FLAG-FLAG  AVX512}


      OT_BITSB32    = OT_BITS32 or OT_VECTORBCST;
      OT_BITSB64    = OT_BITS64 or OT_VECTORBCST;


      OT_VECTOR_EXT_MASK = OT_VECTORMASK or OT_VECTORZERO or OT_VECTORBCST;

      OT_BITS80    = $00000010;  { FPU only  }
      OT_FAR       = $00000020;  { this means 16:16 or 16:32, like in CALL/JMP }
      OT_NEAR      = $00000040;
      OT_SHORT     = $00000080;

      { TODO: FAR/NEAR/SHORT are sizes too, they should be included into size mask,
        but this requires adjusting the opcode table }
      //OT_SIZE_MASK = $3000001F;  { all the size attributes  }
      OT_SIZE_MASK = $E000001F;  { all the size attributes  }
      OT_NON_SIZE  = int64(not int64(OT_SIZE_MASK));

      { Bits 8..11: modifiers }
      OT_SIGNED    = $00000100;  { the operand need to be signed -128-127 }
      OT_TO        = $00000200;  { reverse effect in FADD, FSUB &c  }
      OT_COLON     = $00000400;  { operand is followed by a colon  }
      OT_MODIFIER_MASK = $00000F00;

      { Bits 12..15: type of operand }
      OT_REGISTER  = $00001000;
      OT_IMMEDIATE = $00002000;
      OT_MEMORY    = $0000C000;  { always includes 'OT_REGMEM' bit as well }
      OT_REGMEM    = $00008000;  { for r/m, ie EA, operands  }
      OT_TYPE_MASK = OT_REGISTER or OT_IMMEDIATE or OT_MEMORY or OT_REGMEM;

      OT_REGNORM   = OT_REGISTER or OT_REGMEM;  { 'normal' reg, qualifies as EA  }

      { Bits 20..22, 24..26: register classes
        otf_* consts are not used alone, only to build other constants. }
      otf_reg_cdt  = $00100000;
      otf_reg_gpr  = $00200000;
      otf_reg_sreg = $00400000;
      otf_reg_k    = $00800000;
      otf_reg_fpu  = $01000000;
      otf_reg_mmx  = $02000000;
      otf_reg_xmm  = $04000000;
      otf_reg_ymm  = $08000000;
      otf_reg_zmm  = $10000000;


      otf_reg_extra_mask = $0F000000;
      { Bits 16..19: subclasses, meaning depends on classes field }
      otf_sub0     = $00010000;
      otf_sub1     = $00020000;
      otf_sub2     = $00040000;
      otf_sub3     = $00080000;
      OT_REG_SMASK = otf_sub0 or otf_sub1 or otf_sub2 or otf_sub3;

      //OT_REG_EXTRA_MASK = $0F000000;
      OT_REG_EXTRA_MASK = $1F000000;

      OT_REG_TYPMASK = otf_reg_cdt or otf_reg_gpr or otf_reg_sreg or otf_reg_k or otf_reg_extra_mask;
      { register class 0: CRx, DRx and TRx }
      OT_REG_CDT   = OT_REGISTER or otf_reg_cdt or OT_BITS64;
      OT_REG_CREG  = OT_REG_CDT or otf_sub0;  { CRn  }
      OT_REG_DREG  = OT_REG_CDT or otf_sub1;  { DRn  }
      OT_REG_TREG  = OT_REG_CDT or otf_sub2;  { TRn  }
      OT_REG_CR4   = OT_REG_CDT or otf_sub3;  { CR4 (Pentium only)  }

      { register class 1: general-purpose registers }
      OT_REG_GPR   = OT_REGNORM or otf_reg_gpr;
      OT_RM_GPR    = OT_REGMEM or otf_reg_gpr;
      OT_REG8      = OT_REG_GPR or OT_BITS8;  { 8-bit GPR }
      OT_REG16     = OT_REG_GPR or OT_BITS16;
      OT_REG32     = OT_REG_GPR or OT_BITS32;
      OT_REG64     = OT_REG_GPR or OT_BITS64;

      { GPR subclass 0: accumulator: AL, AX, EAX or RAX }
      OT_REG_ACCUM = OT_REG_GPR or otf_sub0;
      OT_REG_AL    = OT_REG_ACCUM or OT_BITS8;
      OT_REG_AX    = OT_REG_ACCUM or OT_BITS16;
      OT_REG_EAX   = OT_REG_ACCUM or OT_BITS32;
      OT_REG_RAX   = OT_REG_ACCUM or OT_BITS64;
      { GPR subclass 1: counter: CL, CX, ECX or RCX }
      OT_REG_COUNT = OT_REG_GPR or otf_sub1;
      OT_REG_CL    = OT_REG_COUNT or OT_BITS8;
      OT_REG_CX    = OT_REG_COUNT or OT_BITS16;
      OT_REG_ECX   = OT_REG_COUNT or OT_BITS32;
      OT_REG_RCX   = OT_REG_COUNT or OT_BITS64;
      { GPR subclass 2: data register: DL, DX, EDX or RDX }
      OT_REG_DX    = OT_REG_GPR or otf_sub2 or OT_BITS16;
      OT_REG_EDX   = OT_REG_GPR or otf_sub2 or OT_BITS32;

      { register class 2: Segment registers }
      OT_REG_SREG  = OT_REGISTER or otf_reg_sreg or OT_BITS16;
      OT_REG_CS    = OT_REG_SREG or otf_sub0;  { CS  }
      OT_REG_DESS  = OT_REG_SREG or otf_sub1;  { DS, ES, SS (non-CS 86 registers)  }
      OT_REG_FSGS  = OT_REG_SREG or otf_sub2;  { FS, GS (386 extended registers)  }

      { register class 3: FPU registers }
      OT_FPUREG    = OT_REGISTER or otf_reg_fpu;
      OT_FPU0      = OT_FPUREG or otf_sub0;    { FPU stack register zero  }

      { register class 4: MMX (both reg and r/m) }
      OT_MMXREG    = OT_REGNORM or otf_reg_mmx;
      OT_MMXRM     = OT_REGMEM or otf_reg_mmx;

      { register class 5: XMM (both reg and r/m) }
      OT_XMMREG    = OT_REGNORM or otf_reg_xmm;
      OT_XMMRM     = OT_REGMEM or otf_reg_xmm;
      OT_XMEM32    = OT_REGNORM or otf_reg_xmm or otf_reg_gpr or OT_BITS32;
      OT_XMEM32_M  = OT_XMEM32 or OT_VECTORMASK;
      OT_XMEM64    = OT_REGNORM or otf_reg_xmm or otf_reg_gpr or OT_BITS64;
      OT_XMEM64_M  = OT_XMEM64 or OT_VECTORMASK;

      OT_XMMREG_M   = OT_XMMREG or OT_VECTORMASK;
      OT_XMMREG_MZ  = OT_XMMREG or OT_VECTORMASK or OT_VECTORZERO;
      OT_XMMRM_MZ   = OT_XMMRM  or OT_VECTORMASK or OT_VECTORZERO;
      OT_XMMREG_SAE = OT_XMMREG or OT_VECTORSAE;
      OT_XMMRM_SAE  = OT_XMMRM  or OT_VECTORSAE;
      OT_XMMREG_ER  = OT_XMMREG or OT_VECTORER;
      OT_XMMRM_ER   = OT_XMMRM  or OT_VECTORER;



      { register class 5: YMM (both reg and r/m) }
      OT_YMMREG     = OT_REGNORM or otf_reg_ymm;
      OT_YMMRM      = OT_REGMEM or otf_reg_ymm;
      OT_YMEM32     = OT_REGNORM or otf_reg_ymm or otf_reg_gpr or OT_BITS32;
      OT_YMEM32_M   = OT_YMEM32 or OT_VECTORMASK;
      OT_YMEM64     = OT_REGNORM or otf_reg_ymm or otf_reg_gpr or OT_BITS64;
      OT_YMEM64_M   = OT_YMEM64 or OT_VECTORMASK;

      OT_YMMREG_M   = OT_YMMREG or OT_VECTORMASK;
      OT_YMMREG_MZ  = OT_YMMREG or OT_VECTORMASK or OT_VECTORZERO;
      OT_YMMRM_MZ   = OT_YMMRM  or OT_VECTORMASK or OT_VECTORZERO;
      OT_YMMREG_SAE = OT_YMMREG or OT_VECTORSAE;
      OT_YMMRM_SAE  = OT_YMMRM  or OT_VECTORSAE;
      OT_YMMREG_ER  = OT_YMMREG or OT_VECTORER;
      OT_YMMRM_ER   = OT_YMMRM  or OT_VECTORER;


      { register class 5: ZMM (both reg and r/m) }
      OT_ZMMREG     = OT_REGNORM or otf_reg_zmm;
      OT_ZMMRM      = OT_REGMEM or otf_reg_zmm;
      OT_ZMEM32     = OT_REGNORM or otf_reg_zmm or otf_reg_gpr or OT_BITS32;
      OT_ZMEM32_M   = OT_ZMEM32 or OT_VECTORMASK;
      OT_ZMEM64     = OT_REGNORM or otf_reg_zmm or otf_reg_gpr or OT_BITS64;
      OT_ZMEM64_M   = OT_ZMEM64 or OT_VECTORMASK;

      OT_ZMMREG_M   = OT_ZMMREG or OT_VECTORMASK;
      OT_ZMMREG_MZ  = OT_ZMMREG or OT_VECTORMASK or OT_VECTORZERO;
      OT_ZMMRM_MZ   = OT_ZMMRM  or OT_VECTORMASK or OT_VECTORZERO;
      OT_ZMMREG_SAE = OT_ZMMREG or OT_VECTORSAE;
      OT_ZMMRM_SAE  = OT_ZMMRM  or OT_VECTORSAE;
      OT_ZMMREG_ER  = OT_ZMMREG or OT_VECTORER;
      OT_ZMMRM_ER   = OT_ZMMRM  or OT_VECTORER;


      OT_KREG       = OT_REGNORM or otf_reg_k;
      OT_KREG_M     = OT_KREG or OT_VECTORMASK;

      { Vector-Memory operands }
      OT_VMEM_ANY  = OT_XMEM32 or OT_XMEM64 or OT_YMEM32 or OT_YMEM64 or OT_ZMEM32 or OT_ZMEM64;

      { Memory operands }
      OT_MEM8      = OT_MEMORY or OT_BITS8;
      OT_MEM16     = OT_MEMORY or OT_BITS16;
      OT_MEM16_M   = OT_MEM16  or OT_VECTORMASK;
      OT_MEM32     = OT_MEMORY or OT_BITS32;
      OT_MEM32_M   = OT_MEMORY or OT_BITS32 or OT_VECTORMASK;
      OT_BMEM32    = OT_MEMORY or OT_BITS32 or OT_VECTORBCST;
      OT_BMEM32_SAE= OT_MEMORY or OT_BITS32 or OT_VECTORBCST or OT_VECTORSAE;
      OT_MEM64     = OT_MEMORY or OT_BITS64;
      OT_MEM64_M   = OT_MEMORY or OT_BITS64 or OT_VECTORMASK;
      OT_BMEM64    = OT_MEMORY or OT_BITS64 or OT_VECTORBCST;
      OT_BMEM64_SAE= OT_MEMORY or OT_BITS64 or OT_VECTORBCST or OT_VECTORSAE;
      OT_MEM128    = OT_MEMORY or OT_BITS128;
      OT_MEM128_M  = OT_MEMORY or OT_BITS128 or OT_VECTORMASK;
      OT_MEM256    = OT_MEMORY or OT_BITS256;
      OT_MEM256_M  = OT_MEMORY or OT_BITS256 or OT_VECTORMASK;
      OT_MEM512    = OT_MEMORY or OT_BITS512;
      OT_MEM512_M  = OT_MEMORY or OT_BITS512 or OT_VECTORMASK;
      OT_MEM80     = OT_MEMORY or OT_BITS80;




      OT_MEM_OFFS  = OT_MEMORY or otf_sub0;  { special type of EA  }
                                             { simple [address] offset  }

      { Matches any type of r/m operand }
      OT_MEMORY_ANY = OT_MEMORY or OT_RM_GPR or OT_XMMRM or OT_MMXRM or OT_YMMRM or OT_ZMMRM or OT_REG_EXTRA_MASK;

      { Immediate operands }
      OT_IMM8      = OT_IMMEDIATE or OT_BITS8;
      OT_IMM16     = OT_IMMEDIATE or OT_BITS16;
      OT_IMM32     = OT_IMMEDIATE or OT_BITS32;
      OT_IMM64     = OT_IMMEDIATE or OT_BITS64;

      OT_ONENESS   = otf_sub0;  { special type of immediate operand  }
      OT_UNITY     = OT_IMMEDIATE or OT_ONENESS;  { for shift/rotate instructions  }

      std_op2str:op2strtable={$i ../../../compiler/x86_64/x8664int.inc}

  InsTab:array[0..instabentries-1] of TInsEntry={$i ../../../compiler/x86_64/x8664tab.inc}

{ TOperandListItem }

constructor TOperandListItem.Create;
begin
  inherited;

  FOpActive := false;
  FOpNumber := -1;
  FOpTyp    := otUnknown;
  FValues := TStringList.Create;
end;

destructor TOperandListItem.Destroy;
begin
  FreeAndNil(FValues);
  inherited;
end;

{ TOperandList }

function TOperandList.Add(aItem: TOperandListItem): integer;
begin
  result := FList.Add(aItem);
end;

function TOperandList.GetItems(aIndex: integer): TOperandListItem;
begin
  result := TOperandListItem(FList[aIndex]);
end;

{ TAsmTestGenerator }

function TAsmTestGenerator.InternalCalcTestData(const aInst, aOp1, aOp2, aOp3,
  aOp4: String): TStringList;
var
  i: integer;
  Item: TOperandListItem;
  OItem1: TOperandListItem;
  OItem2: TOperandListItem;
  OItem3: TOperandListItem;
  OItem4: TOperandListItem;

  il_Op: integer;
  il_Op1: integer;
  il_Op2: integer;
  il_Op3: integer;
  il_Op4: integer;

  sSuffix: string;
  sl_Operand: String;
  sl_Inst   : String;
  sl_RegCombi: String;
  sl_Prefix: String;
  UsePrefix: boolean;
  il_Operands: integer;
  UsedParams: cardinal;
  UseDefault: boolean;
  sl_RegCombi1: string;
  sl_RegCombi2: string;
  sl_RegCombi3: string;

  function PrepareOperandTyp(const aTyp: String): String;
  begin
    result := aTyp;
    if copy(result, length(result), 1) = '*' then result := copy(result, 1, length(result) - 1);
    if result = 'XMMRM128' then result := 'XMMRM';
    if result = 'YMMRM256' then result := 'YMMRM';
  end;


begin
  result := TStringList.Create;

  OItem1 := TOperandListItem.Create;
  try
    OItem2 := TOperandListItem.Create;
    try
      OItem3 := TOperandListItem.Create;
      try
        OItem4 := TOperandListItem.Create;
        try

          UsePrefix := (UpperCase(aInst) = 'VCVTPD2DQ') OR
                       (UpperCase(aInst) = 'VCVTPD2PS') OR
                       (UpperCase(aInst) = 'VCVTSI2SD') OR
                       (UpperCase(aInst) = 'VCVTSI2SS') OR
                       (UpperCase(aInst) = 'VCVTTPD2DQ') or
                       (UpperCase(aInst) = 'VPMOVZXWQ') or
                       (UpperCase(aInst) = 'VCVTPD2UDQ') or
                       (UpperCase(aInst) = 'VCVTPD2UDQ') or
                       (UpperCase(aInst) = 'VCVTTPD2UDQ') or
                       (UpperCase(aInst) = 'VCVTUQQ2PS') or
                       (UpperCase(aInst) = 'VCVTQQ2PS') or
                       (UpperCase(aInst) = 'VCVTUSI2SD') or
                       (UpperCase(aInst) = 'VCVTUSI2SS') or
                       (UpperCase(aInst) = 'VFPCLASSPD') or
                       (UpperCase(aInst) = 'VFPCLASSPS') or
                       (UpperCase(aInst) = 'VCMPSS')

                       ;


          //TG TODO delete
          if aInst = 'vpmovw2m' then
          begin
            sSuffix := sSuffix;
          end;

          for il_Op := 1 to 4 do
          begin
            sl_Prefix := '';

            case il_Op of
              1: begin
                   Item := OItem1;
                   sl_Operand := aOp1;
                 end;
              2: begin
                   Item := OItem2;
                   sl_Operand := aOp2;
                 end;
              3: begin
                   Item := OItem3;
                   sl_Operand := aOp3;
                 end;
              4: begin
                   Item := OItem4;
                   sl_Operand := aOp4;
                 end;
            end;

            sl_Operand := PrepareOperandTyp(sl_Operand);

            if (AnsiSameText(sl_Operand, 'XMMREG')) or
               (AnsiSameText(sl_Operand, 'XMMREG_M')) or
               (AnsiSameText(sl_Operand, 'XMMREG_MZ')) or
               (AnsiSameText(sl_Operand, 'XMMREG_ER')) or
               (AnsiSameText(sl_Operand, 'XMMREG_SAE')) then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otXMMReg;
              Item.OpActive := true;

              sSuffix := '';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}'
               else if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if Pos('_ER', sl_Operand) > 0 then sSuffix := ', {ru-sae}'
               else if FSAE and (Pos('_SAE', AnsiUppercase(sl_Operand)) > 0) then sSuffix := ', {sae}';

              Item.Values.Add('XMM0' + sSuffix);
              Item.Values.Add('XMM1' + sSuffix);
              Item.Values.Add('XMM2' + sSuffix);
              Item.Values.Add('XMM3' + sSuffix);
              Item.Values.Add('XMM4' + sSuffix);
              Item.Values.Add('XMM5' + sSuffix);
              Item.Values.Add('XMM6' + sSuffix);
              Item.Values.Add('XMM7' + sSuffix);

              if x64 then
              begin
                Item.Values.Clear;
                if FAVX512 then
                begin
                  Item.Values.Add('XMM0' + sSuffix);
                  Item.Values.Add('XMM9' + sSuffix);
                  Item.Values.Add('XMM18' + sSuffix);
                  Item.Values.Add('XMM27' + sSuffix);
                  Item.Values.Add('XMM31' + sSuffix);
                end
                else
                begin
                  Item.Values.Add('XMM0' + sSuffix);
                  Item.Values.Add('XMM4' + sSuffix);
                  Item.Values.Add('XMM8' + sSuffix);
                  Item.Values.Add('XMM12' + sSuffix);
                  Item.Values.Add('XMM15' + sSuffix);
                end;
              end;
            end
            else if (AnsiSameText(sl_Operand, 'XMMRM')) or
                    (AnsiSameText(sl_Operand, 'XMMRM_M')) or
                    (AnsiSameText(sl_Operand, 'XMMRM_MZ')) or
                    (AnsiSameText(sl_Operand, 'XMMRM_ER')) or
                    (AnsiSameText(sl_Operand, 'XMMRM_SAE')) then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otXMMRM;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'oword ';

              sSuffix := '';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}'
               else if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if Pos('_ER', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ', {rd-sae}'
               else if FSAE and (Pos('_SAE', AnsiUppercase(sl_Operand)) > 0) then sSuffix := ', {sae}';

              Item.Values.Add('XMM0' + sSuffix);
              Item.Values.Add('XMM1' + sSuffix);
              Item.Values.Add('XMM2' + sSuffix);
              Item.Values.Add('XMM3' + sSuffix);
              Item.Values.Add('XMM4' + sSuffix);
              Item.Values.Add('XMM5' + sSuffix);
              Item.Values.Add('XMM6' + sSuffix);
              Item.Values.Add('XMM7' + sSuffix);

              if x64 then
              begin
                Item.Values.Clear;
                if FAVX512 then
                begin
                  Item.Values.Add('XMM0' + sSuffix);
                  Item.Values.Add('XMM9' + sSuffix);
                  Item.Values.Add('XMM18' + sSuffix);
                  Item.Values.Add('XMM27' + sSuffix);
                  Item.Values.Add('XMM31' + sSuffix);
                end
                else
                begin
                  Item.Values.Add('XMM0' + sSuffix);
                  Item.Values.Add('XMM4' + sSuffix);
                  Item.Values.Add('XMM8' + sSuffix);
                  Item.Values.Add('XMM12' + sSuffix);
                  Item.Values.Add('XMM15' + sSuffix);
                end;

                MemRegBaseIndexCombi(sl_Prefix, '', FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else
              begin
                MemRegBaseIndexCombi(sl_Prefix, '', FReg32Base, FReg32Index, Item.Values);
              end;
            end
            else if (AnsiSameText(sl_Operand, 'XMMRM8')) or
                    (AnsiSameText(sl_Operand, 'XMMRM8_M')) or
                    (AnsiSameText(sl_Operand, 'XMMRM8_MZ')) or
                    (AnsiSameText(sl_Operand, 'XMMRM8_ER')) or
                    (AnsiSameText(sl_Operand, 'XMMRM8_SAE')) then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otXMMRM8;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'byte ';

              sSuffix := '';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}'
               else if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if Pos('_ER', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ', {rd-sae}'
               else if FSAE and (Pos('_SAE', AnsiUppercase(sl_Operand)) > 0) then sSuffix := ', {sae}';

              Item.Values.Add('XMM0' + sSuffix);
              Item.Values.Add('XMM1' + sSuffix);
              Item.Values.Add('XMM2' + sSuffix);
              Item.Values.Add('XMM3' + sSuffix);
              Item.Values.Add('XMM4' + sSuffix);
              Item.Values.Add('XMM5' + sSuffix);
              Item.Values.Add('XMM6' + sSuffix);
              Item.Values.Add('XMM7' + sSuffix);

              if x64 then
              begin
                Item.Values.Clear;
                if FAVX512 then
                begin
                  Item.Values.Add('XMM0' + sSuffix);
                  Item.Values.Add('XMM9' + sSuffix);
                  Item.Values.Add('XMM18' + sSuffix);
                  Item.Values.Add('XMM27' + sSuffix);
                  Item.Values.Add('XMM31' + sSuffix);
                end
                else
                begin
                  Item.Values.Add('XMM0' + sSuffix);
                  Item.Values.Add('XMM4' + sSuffix);
                  Item.Values.Add('XMM8' + sSuffix);
                  Item.Values.Add('XMM12' + sSuffix);
                  Item.Values.Add('XMM15' + sSuffix);
                end;

                //Item.Values.Add('[RIP]');
                //Item.Values.Add('[RIP + 16]');

                MemRegBaseIndexCombi(sl_Prefix, '', FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else
              begin
                MemRegBaseIndexCombi(sl_Prefix, '', FReg32Base, FReg32Index, Item.Values);
              end;
            end
            else if (AnsiSameText(sl_Operand, 'XMMRM16')) or
                    (AnsiSameText(sl_Operand, 'XMMRM16_M')) or
                    (AnsiSameText(sl_Operand, 'XMMRM16_MZ')) or
                    (AnsiSameText(sl_Operand, 'XMMRM16_ER')) or
                    (AnsiSameText(sl_Operand, 'XMMRM16_SAE'))
                    then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otXMMRM16;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'word ';

              sSuffix := '';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}'
               else if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if Pos('_ER', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ', {rd-sae}'
               else if FSAE and (Pos('_SAE', AnsiUppercase(sl_Operand)) > 0) then sSuffix := ', {sae}';

              Item.Values.Add('XMM0' + sSuffix);
              Item.Values.Add('XMM1' + sSuffix);
              Item.Values.Add('XMM2' + sSuffix);
              Item.Values.Add('XMM3' + sSuffix);
              Item.Values.Add('XMM4' + sSuffix);
              Item.Values.Add('XMM5' + sSuffix);
              Item.Values.Add('XMM6' + sSuffix);
              Item.Values.Add('XMM7' + sSuffix);

              if x64 then
              begin
                Item.Values.Clear;
                if FAVX512 then
                begin
                  Item.Values.Add('XMM0' + sSuffix);
                  Item.Values.Add('XMM9' + sSuffix);
                  Item.Values.Add('XMM18' + sSuffix);
                  Item.Values.Add('XMM27' + sSuffix);
                  Item.Values.Add('XMM31' + sSuffix);
                end
                else
                begin
                  Item.Values.Add('XMM0' + sSuffix);
                  Item.Values.Add('XMM4' + sSuffix);
                  Item.Values.Add('XMM8' + sSuffix);
                  Item.Values.Add('XMM12' + sSuffix);
                  Item.Values.Add('XMM15' + sSuffix);
                end;

                //Item.Values.Add('[RIP]');
                //Item.Values.Add('[RIP + 16]');

                MemRegBaseIndexCombi(sl_prefix, '', FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else
              begin
                MemRegBaseIndexCombi(sl_prefix, '', FReg32Base, FReg32Index, Item.Values);
              end;
            end
            else if (AnsiSameText(sl_Operand, 'YMMREG')) or
                    (AnsiSameText(sl_Operand, 'YMMREG_M')) or
                    (AnsiSameText(sl_Operand, 'YMMREG_MZ')) or
                    (AnsiSameText(sl_Operand, 'YMMREG_ER')) or
                    (AnsiSameText(sl_Operand, 'YMMREG_SAE'))
                    then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otYMMReg;
              Item.OpActive := true;

              sSuffix := '';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}'
               else if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if Pos('_ER', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ', {rd-sae}'
               else if FSAE and (Pos('_SAE', AnsiUppercase(sl_Operand)) > 0) then sSuffix := ', {sae}';

              Item.Values.Add('YMM0' + sSuffix);
              Item.Values.Add('YMM1' + sSuffix);
              Item.Values.Add('YMM2' + sSuffix);
              Item.Values.Add('YMM3' + sSuffix);
              Item.Values.Add('YMM4' + sSuffix);
              Item.Values.Add('YMM5' + sSuffix);
              Item.Values.Add('YMM6' + sSuffix);
              Item.Values.Add('YMM7' + sSuffix);

              if x64 then
              begin
                Item.Values.Clear;
                if FAVX512 then
                begin
                  Item.Values.Add('YMM0' + sSuffix);
                  Item.Values.Add('YMM9' + sSuffix);
                  Item.Values.Add('YMM18' + sSuffix);
                  Item.Values.Add('YMM27' + sSuffix);
                  Item.Values.Add('YMM31' + sSuffix);
                end
                else
                begin
                  Item.Values.Add('YMM0' + sSuffix);
                  Item.Values.Add('YMM4' + sSuffix);
                  Item.Values.Add('YMM8' + sSuffix);
                  Item.Values.Add('YMM12' + sSuffix);
                  Item.Values.Add('YMM15' + sSuffix);
                end;
              end;
            end
            else if (AnsiSameText(sl_Operand, 'YMMRM'))  or
                    (AnsiSameText(sl_Operand, 'YMMRM_M')) or
                    (AnsiSameText(sl_Operand, 'YMMRM_MZ')) or
                    (AnsiSameText(sl_Operand, 'YMMRM_ER')) or
                    (AnsiSameText(sl_Operand, 'YMMRM_SAE'))
                    then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otYMMRM;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'yword ';

              sSuffix := '';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}'
               else if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if Pos('_ER', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ', {rd-sae}'
               else if FSAE and (Pos('_SAE', AnsiUppercase(sl_Operand)) > 0) then sSuffix := ', {sae}';

              Item.Values.Add('YMM0' + sSuffix);
              Item.Values.Add('YMM1' + sSuffix);
              Item.Values.Add('YMM2' + sSuffix);
              Item.Values.Add('YMM3' + sSuffix);
              Item.Values.Add('YMM4' + sSuffix);
              Item.Values.Add('YMM5' + sSuffix);
              Item.Values.Add('YMM6' + sSuffix);
              Item.Values.Add('YMM7' + sSuffix);

              if x64 then
              begin
                Item.Values.Clear;
                if FAVX512 then
                begin
                  Item.Values.Add('YMM0' + sSuffix);
                  Item.Values.Add('YMM9' + sSuffix);
                  Item.Values.Add('YMM18' + sSuffix);
                  Item.Values.Add('YMM27' + sSuffix);
                  Item.Values.Add('YMM31' + sSuffix);
                end
                else
                begin
                  Item.Values.Add('YMM0' + sSuffix);
                  Item.Values.Add('YMM4' + sSuffix);
                  Item.Values.Add('YMM8' + sSuffix);
                  Item.Values.Add('YMM12' + sSuffix);
                  Item.Values.Add('YMM15' + sSuffix);
                end;

                MemRegBaseIndexCombi(sl_prefix, '', FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else
              begin
                MemRegBaseIndexCombi(sl_Prefix, '', FReg32Base, FReg32Index, Item.Values);
              end;
            end
            else if (AnsiSameText(sl_Operand, 'ZMMREG')) or
                    (AnsiSameText(sl_Operand, 'ZMMREG_M')) or
                    (AnsiSameText(sl_Operand, 'ZMMREG_MZ')) or
                    (AnsiSameText(sl_Operand, 'ZMMREG_ER')) or
                    (AnsiSameText(sl_Operand, 'ZMMREG_SAE'))
                    then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otZMMReg;
              Item.OpActive := true;

              sSuffix := '';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}'
               else if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if Pos('_ER', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ', {rd-sae}'
               else if FSAE and (Pos('_SAE', AnsiUppercase(sl_Operand)) > 0) then sSuffix := ', {sae}';

              Item.Values.Add('ZMM0' + sSuffix);
              Item.Values.Add('ZMM1' + sSuffix);
              Item.Values.Add('ZMM2' + sSuffix);
              Item.Values.Add('ZMM3' + sSuffix);
              Item.Values.Add('ZMM4' + sSuffix);
              Item.Values.Add('ZMM5' + sSuffix);
              Item.Values.Add('ZMM6' + sSuffix);
              Item.Values.Add('ZMM7' + sSuffix);

              if x64 then
              begin
                Item.Values.Clear;
                if FAVX512 then
                begin
                  Item.Values.Add('ZMM0' + sSuffix);
                  Item.Values.Add('ZMM9' + sSuffix);
                  Item.Values.Add('ZMM18' + sSuffix);
                  Item.Values.Add('ZMM27' + sSuffix);
                  Item.Values.Add('ZMM31' + sSuffix);
                end
                else
                begin
                  Item.Values.Add('ZMM0' + sSuffix);
                  Item.Values.Add('ZMM4' + sSuffix);
                  Item.Values.Add('ZMM8' + sSuffix);
                  Item.Values.Add('ZMM12' + sSuffix);
                  Item.Values.Add('ZMM15' + sSuffix);
                end;
              end;
            end
            else if (AnsiSameText(sl_Operand, 'ZMMRM')) or
                    (AnsiSameText(sl_Operand, 'ZMMRM_M')) or
                    (AnsiSameText(sl_Operand, 'ZMMRM_MZ')) or
                    (AnsiSameText(sl_Operand, 'XMMRM_ER')) or
                    (AnsiSameText(sl_Operand, 'XMMRM_SAE'))
                    then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otZMMRM;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'zword ';

              sSuffix := '';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}'
               else if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if Pos('_ER', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ', {rd-sae}'
               else if FSAE and (Pos('_SAE', AnsiUppercase(sl_Operand)) > 0) then sSuffix := ', {sae}';

              Item.Values.Add('ZMM0' + sSuffix);
              Item.Values.Add('ZMM1' + sSuffix);
              Item.Values.Add('ZMM2' + sSuffix);
              Item.Values.Add('ZMM3' + sSuffix);
              Item.Values.Add('ZMM4' + sSuffix);
              Item.Values.Add('ZMM5' + sSuffix);
              Item.Values.Add('ZMM6' + sSuffix);
              Item.Values.Add('ZMM7' + sSuffix);

              if x64 then
              begin
                Item.Values.Clear;
                if FAVX512 then
                begin
                  Item.Values.Add('ZMM0' + sSuffix);
                  Item.Values.Add('ZMM9' + sSuffix);
                  Item.Values.Add('ZMM18' + sSuffix);
                  Item.Values.Add('ZMM27' + sSuffix);
                  Item.Values.Add('ZMM31' + sSuffix);
                end
                else
                begin
                  Item.Values.Add('ZMM0' + sSuffix);
                  Item.Values.Add('ZMM4' + sSuffix);
                  Item.Values.Add('ZMM8' + sSuffix);
                  Item.Values.Add('ZMM12' + sSuffix);
                  Item.Values.Add('ZMM15' + sSuffix);
                end;

                MemRegBaseIndexCombi(sl_prefix, '', FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else
              begin
                MemRegBaseIndexCombi(sl_Prefix, '', FReg32Base, FReg32Index, Item.Values);
              end;
            end
            else if AnsiSameText(sl_Operand, 'MEM8') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otMEM8;
              Item.OpActive := true;



	      if UsePrefix then sl_Prefix := 'byte ';

              
              sSuffix := '';
	      
	      if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, '', FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, '', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, 'MEM16') or
                    AnsiSameText(sl_Operand, 'MEM16_M') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otMEM16;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'word ';

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, sSuffix, FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_Prefix, sSuffix, FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, 'MEM32') or
                    AnsiSameText(sl_Operand, 'MEM32_M') or
                    AnsiSameText(sl_Operand, 'MEM32_MZ') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otMEM32;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'dword ';

              sSuffix := '';
              
	      if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}';


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_prefix, sSuffix, FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32Index, Item.Values);
            end
            else if (AnsiSameText(sl_Operand, 'MEM64')) or
                    (AnsiSameText(sl_Operand, 'MEM64_M')) or
                    (AnsiSameText(sl_Operand, 'MEM64_MZ')) then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otMEM64;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'qword ';

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}';

              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, sSuffix, FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32Index, Item.Values);
            end
            else if (AnsiSameText(sl_Operand, 'MEM128')) or
                    (AnsiSameText(sl_Operand, 'MEM128_M')) or
                    (AnsiSameText(sl_Operand, 'MEM128_MZ')) then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otMEM128;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'oword ';

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}';

              if x64 then
              begin
                MemRegBaseIndexCombi(sl_prefix, sSuffix, FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32Index, Item.Values);
            end
            else if (AnsiSameText(sl_Operand, 'MEM256')) or
                    (AnsiSameText(sl_Operand, 'MEM256_M')) or
                    (AnsiSameText(sl_Operand, 'MEM256_MZ')) then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otMEM256;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'yword ';

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}';


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_prefix, sSuffix, FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32Index, Item.Values);
            end
            else if (AnsiSameText(sl_Operand, 'MEM512')) or
                    (AnsiSameText(sl_Operand, 'MEM512_M')) or
                    (AnsiSameText(sl_Operand, 'MEM512_MZ')) then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otMEM512;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'zword ';

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';
              if Pos('_MZ', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1} {z}';


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_prefix, sSuffix, FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, 'REG8') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otREG8;
              Item.OpActive := true;

              if x64 then
              begin
                Item.Values.AddStrings(FReg8);
              end
              else Item.Values.AddStrings(FReg8);
            end
            else if AnsiSameText(sl_Operand, 'REG16') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otREG16;
              Item.OpActive := true;

              if x64 then
              begin
                Item.Values.AddStrings(FReg16);
              end
              else Item.Values.AddStrings(FReg16);
            end
            else if AnsiSameText(sl_Operand, 'REG32') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otREG32;
              Item.OpActive := true;

              if x64 then
              begin
                Item.Values.AddStrings(FReg32Base);
              end
              else Item.Values.AddStrings(FReg32Base);
            end
            else if AnsiSameText(sl_Operand, 'REG64') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otREG64;
              Item.OpActive := true;

              if x64 then
              begin
                Item.Values.AddStrings(FReg64Base);
              end;
            end
            else if AnsiSameText(sl_Operand, 'RM32') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otRM32;
              Item.OpActive := true;

              Item.Values.AddStrings(FReg32Base);

              if UsePrefix then sl_Prefix := 'dword ';

              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, '', FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, '', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, 'RM64') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otRM32;
              Item.OpActive := true;



              if UsePrefix then sl_Prefix := 'qword ';

              if x64 then
              begin
                Item.Values.AddStrings(FReg64Base);
                MemRegBaseIndexCombi(sl_Prefix, '', FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, '', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, 'IMM8') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otIMM8;
              Item.OpActive := true;

              Item.Values.Add('0');
            end
            else if AnsiSameText(sl_Operand, 'XMEM32') or
                    AnsiSameText(sl_Operand, 'XMEM32_M') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otXMEM32;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'oword ';

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if x64 then
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg64Base, FReg64XMMIndex, Item.Values);
              end
              else
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32XMMIndex, Item.Values);
              end;
            end
            else if AnsiSameText(sl_Operand, 'XMEM64') or
                    AnsiSameText(sl_Operand, 'XMEM64_M') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otXMEM64;
              Item.OpActive := true;

              //if UsePrefix then sl_Prefix := 'oword ';
              //
              //if x64 then
              //begin
              //  VectorMemRegBaseIndexCombi(sl_prefix, FReg64Base, FReg64XMMIndex, Item.Values);
              //end
              //else
              //begin
              //  VectorMemRegBaseIndexCombi(sl_prefix, FReg32Base, FReg32XMMIndex, Item.Values);
              //end;

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if x64 then
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg64Base, FReg64XMMIndex, Item.Values);
              end
              else
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32XMMIndex, Item.Values);
              end;

            end
            else if AnsiSameText(sl_Operand, 'YMEM32') or
                    AnsiSameText(sl_Operand, 'YMEM32_M') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otYMEM32;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'yword ';

              //if x64 then
              //begin
              //  VectorMemRegBaseIndexCombi(sl_prefix, FReg64Base, FReg64YMMIndex, Item.Values);
              //end
              //else
              //begin
              //  VectorMemRegBaseIndexCombi(sl_prefix, FReg32Base, FReg32YMMIndex, Item.Values);
              //end;

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if x64 then
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg64Base, FReg64YMMIndex, Item.Values);
              end
              else
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32YMMIndex, Item.Values);
              end;

            end
            else if AnsiSameText(sl_Operand, 'YMEM64') or
                    AnsiSameText(sl_Operand, 'YMEM64_M') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otYMEM64;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'yword ';

              //if x64 then
              //begin
              //  VectorMemRegBaseIndexCombi(sl_prefix, FReg64Base, FReg64YMMIndex, Item.Values);
              //end
              //else
              //begin
              //  VectorMemRegBaseIndexCombi(sl_prefix, FReg32Base, FReg32YMMIndex, Item.Values);
              //end;

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if x64 then
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg64Base, FReg64YMMIndex, Item.Values);
              end
              else
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32YMMIndex, Item.Values);
              end;

            end
            else if AnsiSameText(sl_Operand, 'ZMEM32') or
                    AnsiSameText(sl_Operand, 'ZMEM32_M')  then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otZMEM32;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'zword ';

              //if x64 then
              //begin
              //  VectorMemRegBaseIndexCombi(sl_prefix, FReg64Base, FReg64ZMMIndex, Item.Values);
              //end
              //else
              //begin
              //  VectorMemRegBaseIndexCombi(sl_prefix, FReg32Base, FReg32ZMMIndex, Item.Values);
              //end;

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if x64 then
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg64Base, FReg64ZMMIndex, Item.Values);
              end
              else
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32ZMMIndex, Item.Values);
              end;

            end
            else if AnsiSameText(sl_Operand, 'ZMEM64') or
                    AnsiSameText(sl_Operand, 'ZMEM64_M') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otZMEM64;
              Item.OpActive := true;

              if UsePrefix then sl_Prefix := 'zword ';

              //if x64 then
              //begin
              //  VectorMemRegBaseIndexCombi(sl_prefix, FReg64Base, FReg64ZMMIndex, Item.Values);
              //end
              //else
              //begin
              //  VectorMemRegBaseIndexCombi(sl_prefix, FReg32Base, FReg32ZMMIndex, Item.Values);
              //end;

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if x64 then
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg64Base, FReg64ZMMIndex, Item.Values);
              end
              else
              begin
                VectorMemRegBaseIndexCombi(sl_prefix, sSuffix, FReg32Base, FReg32ZMMIndex, Item.Values);
              end;

            end
            else if AnsiSameText(sl_Operand, '2B32') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otB32;
              Item.OpActive := true;


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, ' {1to2}',  FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, ' {1to2}', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, '4B32') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otB32;
              Item.OpActive := true;


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, ' {1to4}',  FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, ' {1to4}', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, '8B32') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otB32;
              Item.OpActive := true;


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, ' {1to8}',  FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, ' {1to8}', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, '16B32') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otB32;
              Item.OpActive := true;


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, ' {1to16}',  FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, ' {1to16}', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, '2B64') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otB64;
              Item.OpActive := true;


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, ' {1to2}',  FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, ' {1to2}', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, '4B64') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otB64;
              Item.OpActive := true;


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, ' {1to4}',  FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, ' {1to4}', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, '8B64') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otB64;
              Item.OpActive := true;


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, ' {1to8}',  FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, ' {1to8}', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, '16B64') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otB64;
              Item.OpActive := true;


              if x64 then
              begin
                MemRegBaseIndexCombi(sl_Prefix, ' {1to16}',  FReg64Base, FReg64Index, Item.Values);
                //MemRegBaseIndexCombi(FReg6432Base, FReg6432Index, Item.Values);
              end
              else MemRegBaseIndexCombi(sl_prefix, ' {1to16}', FReg32Base, FReg32Index, Item.Values);
            end
            else if AnsiSameText(sl_Operand, 'KREG') or
                    AnsiSameText(sl_Operand, 'KREG_M') then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otKREG;
              Item.OpActive := true;

              sSuffix := '';
              if Pos('_M', AnsiUppercase(sl_Operand)) > 0 then sSuffix := ' {k1}';

              if UsePrefix then sl_Prefix := '';

              for i := 0 to FRegKREG.Count - 1 do
               Item.Values.Add(FRegKREG[i] + sSuffix);
            end
            else if trim(sl_Operand) = '' then
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otUnknown;
              Item.OpActive := false;

              Item.Values.Add('');
            end
            else
            begin
              Item.OpNumber := il_Op;
              Item.OpTyp    := otUnknown;
              Item.OpActive := false;

              Item.Values.Add('?' + sl_Operand);
            end

          end;

          sl_RegCombi := '';


          il_Operands := 0;
          UsedParams  := 0;

          if OItem1.OpActive then
          begin
            inc(il_Operands);
            UsedParams := UsedParams or 1;
          end;

          if OItem2.OpActive then
          begin
            inc(il_Operands);
            UsedParams := UsedParams or 2;
          end;

          if OItem3.OpActive then
          begin
            inc(il_Operands);
            UsedParams := UsedParams or 4;
          end;

          if OItem4.OpActive then
          begin
            inc(il_Operands);
            UsedParams := UsedParams or 8;
          end;

          case il_Operands of
              1: UseDefault := UsedParams <> 1;
              2: UseDefault := UsedParams <> 3;
              3: UseDefault := UsedParams <> 7;
              4: UseDefault := UsedParams <> 15;
            else UseDefault := true;
          end;

          //UseDefault := true;

          if UseDefault then
          begin
            sl_Inst := format('%-20s', [aInst]);

            for il_Op1 := 0 to OItem1.Values.Count - 1 do
            begin
              for il_Op2 := 0 to OItem2.Values.Count - 1 do
              begin
                for il_Op3 := 0 to OItem3.Values.Count - 1 do
                begin
                  for il_Op4 := 0 to OItem4.Values.Count - 1 do
                  begin
                    sl_RegCombi := '';

                    if OItem1.OpActive then
                    begin
                      if sl_RegCombi <> '' then sl_RegCombi := sl_RegCombi + ', ';
                      sl_RegCombi := sl_RegCombi + OItem1.Values[il_Op1];
                    end;

                    if OItem2.OpActive then
                    begin
                      if sl_RegCombi <> '' then sl_RegCombi := sl_RegCombi + ', ';
                      sl_RegCombi := sl_RegCombi + OItem2.Values[il_Op2];
                    end;

                    if OItem3.OpActive then
                    begin
                      if sl_RegCombi <> '' then sl_RegCombi := sl_RegCombi + ', ';
                      sl_RegCombi := sl_RegCombi + OItem3.Values[il_Op3];
                    end;

                    if OItem4.OpActive then
                    begin
                      if sl_RegCombi <> '' then sl_RegCombi := sl_RegCombi + ', ';
                      sl_RegCombi := sl_RegCombi + OItem4.Values[il_Op4];
                    end;

                    if sl_RegCombi <> '' then
                    begin
                      //result.Add(format('%-20s%s', [aInst, sl_RegCombi]));
                      result.Add(sl_Inst + sl_RegCombi);
                      sl_RegCombi := '';
                    end;
                  end;
                end;
              end;
            end;
          end
          else
          begin
            sl_Inst := format('%-20s', [aInst]);

            for il_Op1 := 0 to OItem1.Values.Count - 1 do
            begin
              if OItem1.OpActive then
              begin
                sl_RegCombi1 := OItem1.Values[il_Op1];
              end
              else sl_RegCombi1 := '';

              for il_Op2 := 0 to OItem2.Values.Count - 1 do
              begin
                if OItem2.OpActive then
                begin
                  sl_RegCombi2 := sl_RegCombi1 + ', ' + OItem2.Values[il_Op2];
                end
                else sl_RegCombi2 := sl_RegCombi1;

                for il_Op3 := 0 to OItem3.Values.Count - 1 do
                begin
                  if OItem3.OpActive then
                  begin
                    sl_RegCombi3 := sl_RegCombi2 + ', ' + OItem3.Values[il_Op3];
                  end
                  else sl_RegCombi3 := sl_RegCombi2;

                  for il_Op4 := 0 to OItem4.Values.Count - 1 do
                  begin
                    if OItem4.OpActive then
                    begin
                      sl_RegCombi := sl_RegCombi3 + ', ' + OItem4.Values[il_Op4];
                    end
                    else sl_RegCombi := sl_RegCombi3;

                    if sl_RegCombi <> '' then
                    begin
                      //result.Add(format('%-20s%s', [aInst, sl_RegCombi]));
                      result.Add(sl_Inst + sl_RegCombi);
                      sl_RegCombi := '';
                    end;
                  end;
                end;
              end;
            end;
          end;
        finally
          FreeAndNil(OItem4);
        end;
      finally
        FreeAndNil(OItem3);
      end;
    finally
      FreeAndNil(OItem2);
    end;
  finally
    FreeAndNil(OItem1);
  end;
end;

constructor TAsmTestGenerator.Create;
begin
  inherited;

  FX64 := true;
  FAVX512 := false;

  FReg8          := TStringList.Create;
  FReg16         := TStringList.Create;

  FReg32Base     := TStringList.Create;
  FReg32Index    := TStringList.Create;
  FReg64Base     := TStringList.Create;
  FReg64Index    := TStringList.Create;
  FReg6432Base   := TStringList.Create;
  FReg6432Index  := TStringList.Create;
  FReg32XMMIndex := TStringList.Create;
  FReg32YMMIndex := TStringList.Create;
  FReg32ZMMIndex := TStringList.Create;
  FReg64XMMIndex := TStringList.Create;
  FReg64YMMIndex := TStringList.Create;
  FReg64ZMMIndex := TStringList.Create;
  FRegKREG       := TStringList.Create;

  FReg8.Add('AL');
  FReg8.Add('BL');
  FReg8.Add('CL');
  FReg8.Add('DL');

  
  FReg16.Add('AX');
  FReg16.Add('BX');
  FReg16.Add('CX');
  FReg16.Add('DX');


  FReg32Base.Add('EAX');
  FReg32Base.Add('EBX');
  FReg32Base.Add('ECX');
  FReg32Base.Add('EDX');
  //FReg32Base.Add('ESP');
  //FReg32Base.Add('EBP');
  FReg32Base.Add('EDI');
  FReg32Base.Add('ESI');


  FReg32Index.Add('EAX');
  FReg32Index.Add('EBX');
  FReg32Index.Add('ECX');
  FReg32Index.Add('EDX');
  //FReg32Index.Add('EBP');
  FReg32Index.Add('EDI');
  FReg32Index.Add('ESI');


  FReg64Base.Add('RAX');
  FReg64Base.Add('RBX');
  FReg64Base.Add('RCX');
  FReg64Base.Add('RDX');
  //FReg64Base.Add('RSP');
  //FReg64Base.Add('RBP');
  FReg64Base.Add('RDI');
  FReg64Base.Add('RSI');
  FReg64Base.Add('R8');
  FReg64Base.Add('R9');
  FReg64Base.Add('R10');
  FReg64Base.Add('R11');
  FReg64Base.Add('R12');
  FReg64Base.Add('R13');
  FReg64Base.Add('R14');
  FReg64Base.Add('R15');

  FReg64Index.Add('RAX');
  FReg64Index.Add('RBX');
  FReg64Index.Add('RCX');
  FReg64Index.Add('RDX');
  //FReg64Index.Add('RBP');
  FReg64Index.Add('RDI');
  FReg64Index.Add('RSI');
  FReg64Index.Add('R8');
  FReg64Index.Add('R9');
  FReg64Index.Add('R10');
  FReg64Index.Add('R11');
  FReg64Index.Add('R12');
  FReg64Index.Add('R13');
  FReg64Index.Add('R14');
  FReg64Index.Add('R15');

  FReg6432Base.Add('EAX');
  FReg6432Base.Add('EBX');
  FReg6432Base.Add('ECX');
  FReg6432Base.Add('EDX');
  FReg6432Base.Add('ESP');
  //FReg6432Base.Add('EBP');
  FReg6432Base.Add('EDI');
  FReg6432Base.Add('ESI');
  FReg6432Base.Add('R8D');
  FReg6432Base.Add('R9D');
  FReg6432Base.Add('R10D');
  FReg6432Base.Add('R11D');
  FReg6432Base.Add('R12D');
  FReg6432Base.Add('R13D');
  FReg6432Base.Add('R14D');
  FReg6432Base.Add('R15D');

  FReg6432Index.Add('EAX');
  FReg6432Index.Add('EBX');
  FReg6432Index.Add('ECX');
  FReg6432Index.Add('EDX');
  //FReg6432Index.Add('EBP');
  FReg6432Index.Add('EDI');
  FReg6432Index.Add('ESI');
  FReg6432Index.Add('R8D');
  FReg6432Index.Add('R9D');
  FReg6432Index.Add('R10D');
  FReg6432Index.Add('R11D');
  FReg6432Index.Add('R12D');
  FReg6432Index.Add('R13D');
  FReg6432Index.Add('R14D');
  FReg6432Index.Add('R15D');

  FReg32XMMIndex.ADD('XMM0');
  FReg32XMMIndex.ADD('XMM1');
  FReg32XMMIndex.ADD('XMM2');
  FReg32XMMIndex.ADD('XMM3');
  FReg32XMMIndex.ADD('XMM4');
  FReg32XMMIndex.ADD('XMM5');
  FReg32XMMIndex.ADD('XMM6');
  FReg32XMMIndex.ADD('XMM7');

  FReg32YMMIndex.ADD('YMM0');
  FReg32YMMIndex.ADD('YMM1');
  FReg32YMMIndex.ADD('YMM2');
  FReg32YMMIndex.ADD('YMM3');
  FReg32YMMIndex.ADD('YMM4');
  FReg32YMMIndex.ADD('YMM5');
  FReg32YMMIndex.ADD('YMM6');
  FReg32YMMIndex.ADD('YMM7');

  FReg32ZMMIndex.ADD('ZMM0');
  FReg32ZMMIndex.ADD('ZMM1');
  FReg32ZMMIndex.ADD('ZMM2');
  FReg32ZMMIndex.ADD('ZMM3');
  FReg32ZMMIndex.ADD('ZMM4');
  FReg32ZMMIndex.ADD('ZMM5');
  FReg32ZMMIndex.ADD('ZMM6');
  FReg32ZMMIndex.ADD('ZMM7');


  FReg64XMMIndex.ADD('XMM0');
  FReg64XMMIndex.ADD('XMM1');
  FReg64XMMIndex.ADD('XMM2');
  FReg64XMMIndex.ADD('XMM3');
  FReg64XMMIndex.ADD('XMM4');
  FReg64XMMIndex.ADD('XMM5');
  FReg64XMMIndex.ADD('XMM6');
  FReg64XMMIndex.ADD('XMM7');
  FReg64XMMIndex.ADD('XMM8');
  FReg64XMMIndex.ADD('XMM9');
  FReg64XMMIndex.ADD('XMM10');
  FReg64XMMIndex.ADD('XMM11');
  FReg64XMMIndex.ADD('XMM12');
  FReg64XMMIndex.ADD('XMM13');
  FReg64XMMIndex.ADD('XMM14');
  FReg64XMMIndex.ADD('XMM15');


  FReg64YMMIndex.ADD('YMM0');
  FReg64YMMIndex.ADD('YMM1');
  FReg64YMMIndex.ADD('YMM2');
  FReg64YMMIndex.ADD('YMM3');
  FReg64YMMIndex.ADD('YMM4');
  FReg64YMMIndex.ADD('YMM5');
  FReg64YMMIndex.ADD('YMM6');
  FReg64YMMIndex.ADD('YMM7');
  FReg64YMMIndex.ADD('YMM8');
  FReg64YMMIndex.ADD('YMM9');
  FReg64YMMIndex.ADD('YMM10');
  FReg64YMMIndex.ADD('YMM11');
  FReg64YMMIndex.ADD('YMM12');
  FReg64YMMIndex.ADD('YMM13');
  FReg64YMMIndex.ADD('YMM14');
  FReg64YMMIndex.ADD('YMM15');

  FReg64ZMMIndex.ADD('ZMM0');
  FReg64ZMMIndex.ADD('ZMM1');
  FReg64ZMMIndex.ADD('ZMM2');
  FReg64ZMMIndex.ADD('ZMM3');
  FReg64ZMMIndex.ADD('ZMM4');
  FReg64ZMMIndex.ADD('ZMM5');
  FReg64ZMMIndex.ADD('ZMM6');
  FReg64ZMMIndex.ADD('ZMM7');
  FReg64ZMMIndex.ADD('ZMM8');
  FReg64ZMMIndex.ADD('ZMM9');
  FReg64ZMMIndex.ADD('ZMM10');
  FReg64ZMMIndex.ADD('ZMM11');
  FReg64ZMMIndex.ADD('ZMM12');
  FReg64ZMMIndex.ADD('ZMM13');
  FReg64ZMMIndex.ADD('ZMM14');
  FReg64ZMMIndex.ADD('ZMM15');
  FReg64ZMMIndex.ADD('ZMM16');
  FReg64ZMMIndex.ADD('ZMM17');
  FReg64ZMMIndex.ADD('ZMM18');
  FReg64ZMMIndex.ADD('ZMM19');
  FReg64ZMMIndex.ADD('ZMM20');
  FReg64ZMMIndex.ADD('ZMM21');
  FReg64ZMMIndex.ADD('ZMM22');
  FReg64ZMMIndex.ADD('ZMM23');
  FReg64ZMMIndex.ADD('ZMM24');
  FReg64ZMMIndex.ADD('ZMM25');
  FReg64ZMMIndex.ADD('ZMM26');
  FReg64ZMMIndex.ADD('ZMM27');
  FReg64ZMMIndex.ADD('ZMM28');
  FReg64ZMMIndex.ADD('ZMM29');
  FReg64ZMMIndex.ADD('ZMM30');
  FReg64ZMMIndex.ADD('ZMM31');

  FRegKREG.ADD('K0');
  FRegKREG.ADD('K1');
  FRegKREG.ADD('K2');
  FRegKREG.ADD('K3');
  FRegKREG.ADD('K4');
  FRegKREG.ADD('K5');
  FRegKREG.ADD('K6');
  FRegKREG.ADD('K7');

end;

destructor TAsmTestGenerator.Destroy;
begin
  FreeAndNil(FReg8);
  FreeAndNil(FReg16);

  FreeAndNil(FReg32Base);
  FreeAndNil(FReg32Index);
  FreeAndNil(FReg64Base);
  FreeAndNil(FReg64Index);
  FreeAndNil(FReg6432Base);
  FreeAndNil(FReg6432Index);

  FreeAndNil(FReg32XMMIndex);
  FreeAndNil(FReg32YMMIndex);
  FreeAndNil(FReg32ZMMIndex);
  FreeAndNil(FReg64XMMIndex);
  FreeAndNil(FReg64YMMIndex);
  FreeAndNil(FReg64ZMMIndex);

  FreeAndnil(FRegKREG);

  inherited;
end;

procedure TAsmTestGenerator.MemRegBaseIndexCombi(const aPrefix, aSuffix: String; aSLBaseReg,
  aSLIndexReg, aRList: TStringList);
var
  il_Base: integer;
  il_Index: integer;
begin

  for il_Base := 0 to aSLBaseReg.Count - 1 do
  begin
    aRList.Add(format(aPrefix + '[%s]%s', [aSLBaseReg[il_Base], aSuffix]));

    for il_Index := 0 to aSLIndexReg.Count - 1 do
    begin
      aRList.Add(format(aPrefix + '[%s + %s]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
      aRList.Add(format(aPrefix + '[%s + %s + $10]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
      aRList.Add(format(aPrefix + '[%s + %s + $40]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
      aRList.Add(format(aPrefix + '[%s + %s - $10]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
      aRList.Add(format(aPrefix + '[%s + %s - $40]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));


      aRList.Add(format(aPrefix + '[%s + %s * 2]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
      aRList.Add(format(aPrefix + '[%s + %s * 4]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
      aRList.Add(format(aPrefix + '[%s + %s * 8]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));

      aRList.Add(format(aPrefix + '[%s + %s * 2 + 16]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
      aRList.Add(format(aPrefix + '[%s + %s * 4 + 32]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
      aRList.Add(format(aPrefix + '[%s + %s * 8 + 64]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
    end;
  end;
end;

procedure TAsmTestGenerator.VectorMemRegBaseIndexCombi(const aPrefix, aSuffix: String;
  aSLBaseReg, aSLIndexReg, aRList: TStringList);
var
  il_Base: integer;
  il_Index: integer;
begin

  //for il_Index := 0 to aSLIndexReg.Count - 1 do
  //begin
  //  aRList.Add(format(aPrefix + '[%s]', [aSLIndexReg[il_Index]]));
  //
  //  aRList.Add(format(aPrefix + '[%s * 2]', [aSLIndexReg[il_Index]]));
  //  aRList.Add(format(aPrefix + '[%s * 4]', [aSLIndexReg[il_Index]]));
  //  aRList.Add(format(aPrefix + '[%s * 8]', [aSLIndexReg[il_Index]]));
  //
  //  aRList.Add(format(aPrefix + '[%s * 2 + 16]', [aSLIndexReg[il_Index]]));
  //  aRList.Add(format(aPrefix + '[%s * 4 + 32]', [aSLIndexReg[il_Index]]));
  //  aRList.Add(format(aPrefix + '[%s * 8 + 48]', [aSLIndexReg[il_Index]]));
  //end;


  for il_Base := 0 to aSLBaseReg.Count - 1 do
  begin
    //aRList.Add(format(aPrefix + '[%s]', [aSLBaseReg[il_Base]]));

    for il_Index := 0 to aSLIndexReg.Count - 1 do
    begin
      aRList.Add(format(aPrefix + '[%s + %s]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));

      aRList.Add(format(aPrefix + '[%s + %s * 2]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
      aRList.Add(format(aPrefix + '[%s + %s * 4]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));
      aRList.Add(format(aPrefix + '[%s + %s * 8]%s', [aSLBaseReg[il_Base], aSLIndexReg[il_Index], aSuffix]));

      //aRList.Add(format(aPrefix + '[%s + %s * 2 + 16]', [aSLBaseReg[il_Base], aSLIndexReg[il_Index]]));
      //aRList.Add(format(aPrefix + '[%s + %s * 4 + 32]', [aSLBaseReg[il_Base], aSLIndexReg[il_Index]]));
      //aRList.Add(format(aPrefix + '[%s + %s * 8 + 48]', [aSLBaseReg[il_Base], aSLIndexReg[il_Index]]));


      aRList.Add(format(aPrefix + '[%s + %s]%s', [aSLIndexReg[il_Index], aSLBaseReg[il_Base], aSuffix]));

      //aRList.Add(format(aPrefix + '[%s + %s + 16]', [aSLIndexReg[il_Index], aSLBaseReg[il_Base]]));
    end;
  end;
end;

class procedure TAsmTestGenerator.CalcTestData(aX64, aAVX512, aSAE: boolean; const aInst, aOp1, aOp2, aOp3,
  aOp4: String; aSL: TStringList);
var
  sl: TStringList;
begin
  with TAsmTestGenerator.Create do
  try
    Fx64 := aX64;
    FAVX512 := aAVX512;
    FSAE    := aSAE;

    sl := InternalCalcTestData(aInst, aOp1, aOp2, aOp3, aOp4);
    try
      aSL.AddStrings(sl);
    finally
      FreeAndNil(sl);
    end;
  finally
    Free;
  end;
end;

class procedure TAsmTestGenerator.CalcTestInstFile;
var
  i,j: integer;
  sInst: string;
  sI386: string;
  sX8664: string;
  sAVX512: string;
  sOperands: string;

  sLine: string;


  sl: TStringList;
  bVEX: boolean;
  bEVEX: boolean;
  b256 : boolean;
  b512 : boolean;
begin
  sl := TStringList.Create;
  try

    //tinsentry=packed record
    //  opcode  : tasmop;
    //  ops     : byte;
    //  //optypes : array[0..max_operands-1] of longint;
    //  optypes : array[0..3] of int64; //TG
    //  code    : array[0..11] of char;
    //  flags   : tinsflags;
    //end;

    for i := 0 to length(InsTab) - 1 do
    begin
      bVEX := false;
      bEVEX := false;
      b256 := false;
      b512 := false;

      //TG TODO delete
      if instab[i].opcode = a_vtestps then
      begin
        b512 := b512;
      end;

      for j := 0 to length(InsTab[i].code) - 1 do
      begin
        case ord(InsTab[i].code[j]) of
            0: break;
            1,2,3: break;
          232: bEVEX := true;
          233: b512 := true;
          242: bVEX := true;
          244: b256 := true;
        end;
      end;

      if bVEX or bEVEX then
      begin
        sInst  :=  std_op2str[InsTab[i].opcode];
        sI386  := '1';
        sX8664 := '1';
        if IF_X86_64 in InsTab[i].flags then
        begin
          sI386  := '0';
        end;

        if bEVEX then sAVX512 := '1'
         else sAVX512 := '0';

        sOperands := '';
        for j := 0 to 3 do
        begin
          case InsTab[i].optypes[j] of
               OT_XMMREG: sOperands := sOperands + 'XMMREG,';
             OT_XMMREG_M: sOperands := sOperands + 'XMMREG_M,';
            OT_XMMREG_MZ: sOperands := sOperands + 'XMMREG_MZ,';
            OT_XMMREG_ER: sOperands := sOperands + 'XMMREG_ER,';
           OT_XMMREG_SAE: sOperands := sOperands + 'XMMREG_SAE,';
                OT_XMMRM: sOperands := sOperands + 'XMMRM,';
             OT_XMMRM_MZ: sOperands := sOperands + 'XMMRM_MZ,';

               OT_YMMREG: sOperands := sOperands + 'YMMREG,';
             OT_YMMREG_M: sOperands := sOperands + 'YMMREG_M,';
            OT_YMMREG_MZ: sOperands := sOperands + 'YMMREG_MZ,';
            OT_YMMREG_ER: sOperands := sOperands + 'YMMREG_ER,';
           OT_YMMREG_SAE: sOperands := sOperands + 'YMMREG_SAE,';
                OT_YMMRM: sOperands := sOperands + 'YMMRM,';
             OT_YMMRM_MZ: sOperands := sOperands + 'YMMRM_MZ,';

               OT_ZMMREG: sOperands := sOperands + 'ZMMREG,';
             OT_ZMMREG_M: sOperands := sOperands + 'ZMMREG_M,';
            OT_ZMMREG_MZ: sOperands := sOperands + 'ZMMREG_MZ,';
            OT_ZMMREG_ER: sOperands := sOperands + 'ZMMREG_ER,';
           OT_ZMMREG_SAE: sOperands := sOperands + 'ZMMREG_SAE,';
                OT_ZMMRM: sOperands := sOperands + 'ZMMRM,';
             OT_ZMMRM_MZ: sOperands := sOperands + 'ZMMRM_MZ,';

                OT_MEM32: sOperands := sOperands + 'MEM32,';
                OT_MEM64: sOperands := sOperands + 'MEM64,';
               OT_MEM128: sOperands := sOperands + 'MEM128,';
               OT_MEM256: sOperands := sOperands + 'MEM256,';
               OT_MEM512: sOperands := sOperands + 'MEM512,';

                OT_REG32: sOperands := sOperands + 'REG32,';
                OT_REG64: sOperands := sOperands + 'REG64,';
                 ot_rm_gpr or ot_bits32:
                          sOperands := sOperands + 'RM32,';
                 ot_rm_gpr or ot_bits64:
                          sOperands := sOperands + 'RM64,';

               OT_XMEM32: sOperands := sOperands + 'XMEM32,';
               OT_XMEM64: sOperands := sOperands + 'XMEM64,';

               OT_YMEM32: sOperands := sOperands + 'YMEM32,';
               OT_YMEM64: sOperands := sOperands + 'YMEM64,';

                 OT_IMM8: sOperands := sOperands + 'IMM8,';
                 OT_NONE: sOperands := sOperands + ',';

               OT_BMEM32: if b512 then sOperands := sOperands + '16B32,'
                           else if b256 then sOperands := sOperands + '8B32,'
                           else sOperands := sOperands + '4B32,';
               OT_BMEM64: if b512 then sOperands := sOperands + '8B32,'
                           else if b256 then sOperands := sOperands + '4B32,'
                           else sOperands := sOperands + '2B64,';

                 OT_KREG: sOperands := sOperands + 'KREG,';
               OT_KREG_M: sOperands := sOperands + 'KREG_M,';

            else  sOperands := sOperands;
          end;
        end;

        sOperands := copy(sOperands, 1, length(sOperands) - 1);

        sl.Add(format('FOpCodeList.Add(''%s,%s,%s,%s,%s'');', [sInst, sI386, sX8664, sAVX512, sOperands]));
      end;

    end;

    sl.Savetofile('/tmp/fpcavx512.txt');

   // std_op2str

  finally
    FreeAndnil(sl);
  end;
end;

end.
