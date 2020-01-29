{
    Copyright (c) 1998-2002 by Florian Klaempfl and Peter Vreman

    Contains the abstract assembler implementation for the i386

    * Portions of this code was inspired by the NASM sources
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
unit aasmcpu;

{$i fpcdefs.inc}

interface

    uses
      globtype,verbose,
      cpubase,
      cgbase,cgutils,
      aasmbase,aasmtai,aasmsym,
      ogbase;

    const
      { "mov reg,reg" source operand number }
      O_MOV_SOURCE = 0;
      { "mov reg,reg" destination operand number }
      O_MOV_DEST = 1;

    { Operand types }
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

      OT_VECTOR_EXT = OT_VECTORMASK or OT_VECTORZERO or OT_VECTORBCST or OT_VECTORSAE or OT_VECTORER;

      OT_BITSB32    = OT_BITS32 or OT_VECTORBCST;
      OT_BITSB64    = OT_BITS64 or OT_VECTORBCST;

      OT_BITS80    = $00000010;  { FPU only  }
      OT_FAR       = $00000020;  { this means 16:16 or 16:32, like in CALL/JMP }
      OT_NEAR      = $00000040;
      OT_SHORT     = $00000080;

      { TODO: FAR/NEAR/SHORT are sizes too, they should be included into size mask,
        but this requires adjusting the opcode table }
      //OT_SIZE_MASK = $3000001F;  { all the size attributes  }
      OT_SIZE_MASK = $E000001F;  { all the size attributes  }
      OT_NON_SIZE  = longint(not(longint(OT_SIZE_MASK)));

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
{$ifdef x86_64}
      OT_REG_CDT   = OT_REGISTER or otf_reg_cdt or OT_BITS64;
{$else x86_64}
      OT_REG_CDT   = OT_REGISTER or otf_reg_cdt or OT_BITS32;
{$endif x86_64}
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
{$ifdef x86_64}
      OT_REG_RAX   = OT_REG_ACCUM or OT_BITS64;
{$endif x86_64}
      { GPR subclass 1: counter: CL, CX, ECX or RCX }
      OT_REG_COUNT = OT_REG_GPR or otf_sub1;
      OT_REG_CL    = OT_REG_COUNT or OT_BITS8;
      OT_REG_CX    = OT_REG_COUNT or OT_BITS16;
      OT_REG_ECX   = OT_REG_COUNT or OT_BITS32;
{$ifdef x86_64}
      OT_REG_RCX   = OT_REG_COUNT or OT_BITS64;
{$endif x86_64}
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

      OTVE_VECTOR_SAE          = 1 shl 8;
      OTVE_VECTOR_ER           = 1 shl 9;
      OTVE_VECTOR_ZERO         = 1 shl 10;
      OTVE_VECTOR_WRITEMASK    = 1 shl 11;
      OTVE_VECTOR_BCST         = 1 shl 12;
      OTVE_VECTOR_BCST2        = 0;
      OTVE_VECTOR_BCST4        = 1 shl 4;
      OTVE_VECTOR_BCST8        = 1 shl 5;
      OTVE_VECTOR_BCST16       = 3 shl 4;
      OTVE_VECTOR_RNSAE        = OTVE_VECTOR_ER or 0;
      OTVE_VECTOR_RDSAE        = OTVE_VECTOR_ER or 1 shl 6;
      OTVE_VECTOR_RUSAE        = OTVE_VECTOR_ER or 1 shl 7;
      OTVE_VECTOR_RZSAE        = OTVE_VECTOR_ER or 3 shl 6;


      OTVE_VECTOR_BCST_MASK    = OTVE_VECTOR_BCST2 or OTVE_VECTOR_BCST4 or OTVE_VECTOR_BCST8 or OTVE_VECTOR_BCST16;
      OTVE_VECTOR_ER_MASK      = OTVE_VECTOR_RNSAE or OTVE_VECTOR_RDSAE or OTVE_VECTOR_RUSAE or OTVE_VECTOR_RZSAE;

      OTVE_VECTOR_MASK = OTVE_VECTOR_SAE or OTVE_VECTOR_ER or OTVE_VECTOR_ZERO or OTVE_VECTOR_WRITEMASK or OTVE_VECTOR_BCST;

      { Size of the instruction table converted by nasmconv.pas }
{$if defined(x86_64)}
      instabentries = {$i x8664nop.inc}
{$elseif defined(i386)}
      instabentries = {$i i386nop.inc}
{$elseif defined(i8086)}
      instabentries = {$i i8086nop.inc}
{$endif}
      maxinfolen    = 10;

    type
      { What an instruction can change. Needed for optimizer and spilling code.

        Note: The order of this enumeration is should not be changed! }
      TInsChange = (Ch_None,
        {Read from a register}
        Ch_REAX, Ch_RECX, Ch_REDX, Ch_REBX, Ch_RESP, Ch_REBP, Ch_RESI, Ch_REDI,
        {write from a register}
        Ch_WEAX, Ch_WECX, Ch_WEDX, Ch_WEBX, Ch_WESP, Ch_WEBP, Ch_WESI, Ch_WEDI,
        {read and write from/to a register}
        Ch_RWEAX, Ch_RWECX, Ch_RWEDX, Ch_RWEBX, Ch_RWESP, Ch_RWEBP, Ch_RWESI, Ch_RWEDI,
        {modify the contents of a register with the purpose of using
         this changed content afterwards (add/sub/..., but e.g. not rep
         or movsd)}
        Ch_MEAX, Ch_MECX, Ch_MEDX, Ch_MEBX, Ch_MESP, Ch_MEBP, Ch_MESI, Ch_MEDI,
        {read individual flag bits from the flags register}
        Ch_RCarryFlag,Ch_RParityFlag,Ch_RAuxiliaryFlag,Ch_RZeroFlag,Ch_RSignFlag,Ch_ROverflowFlag,
        {write individual flag bits to the flags register}
        Ch_WCarryFlag,Ch_WParityFlag,Ch_WAuxiliaryFlag,Ch_WZeroFlag,Ch_WSignFlag,Ch_WOverflowFlag,
        {set individual flag bits to 0 in the flags register}
        Ch_W0CarryFlag,Ch_W0ParityFlag,Ch_W0AuxiliaryFlag,Ch_W0ZeroFlag,Ch_W0SignFlag,Ch_W0OverflowFlag,
        {set individual flag bits to 1 in the flags register}
        Ch_W1CarryFlag,Ch_W1ParityFlag,Ch_W1AuxiliaryFlag,Ch_W1ZeroFlag,Ch_W1SignFlag,Ch_W1OverflowFlag,
        {write an undefined value to individual flag bits in the flags register}
        Ch_WUCarryFlag,Ch_WUParityFlag,Ch_WUAuxiliaryFlag,Ch_WUZeroFlag,Ch_WUSignFlag,Ch_WUOverflowFlag,
        {read and write flag bits}
        Ch_RWCarryFlag,Ch_RWParityFlag,Ch_RWAuxiliaryFlag,Ch_RWZeroFlag,Ch_RWSignFlag,Ch_RWOverflowFlag,
        {more specialized flag bits (not considered part of NR_DEFAULTFLAGS by the compiler)}
        Ch_RDirFlag,Ch_W0DirFlag,Ch_W1DirFlag,Ch_W0IntFlag,Ch_W1IntFlag,
        {instruction reads flag bits, according to its condition (used by Jcc/SETcc/CMOVcc)}
        Ch_RFLAGScc,
        {read/write/read+write the entire flags/eflags/rflags register}
        Ch_RFlags, Ch_WFlags, Ch_RWFlags,
        Ch_FPU,
        Ch_Rop1, Ch_Wop1, Ch_RWop1, Ch_Mop1,
        Ch_Rop2, Ch_Wop2, Ch_RWop2, Ch_Mop2,
        Ch_Rop3, Ch_WOp3, Ch_RWOp3, Ch_Mop3,
        Ch_Rop4, Ch_WOp4, Ch_RWOp4, Ch_Mop4,
        { instruction doesn't read it's input register, in case both parameters
          are the same register (e.g. xor eax,eax; sub eax,eax; sbb eax,eax (reads flags only), etc.) }
        Ch_NoReadIfEqualRegs,
        Ch_RMemEDI,Ch_WMemEDI,
        Ch_All,
        { x86_64 registers }
        Ch_RRAX, Ch_RRCX, Ch_RRDX, Ch_RRBX, Ch_RRSP, Ch_RRBP, Ch_RRSI, Ch_RRDI,
        Ch_WRAX, Ch_WRCX, Ch_WRDX, Ch_WRBX, Ch_WRSP, Ch_WRBP, Ch_WRSI, Ch_WRDI,
        Ch_RWRAX, Ch_RWRCX, Ch_RWRDX, Ch_RWRBX, Ch_RWRSP, Ch_RWRBP, Ch_RWRSI, Ch_RWRDI,
        Ch_MRAX, Ch_MRCX, Ch_MRDX, Ch_MRBX, Ch_MRSP, Ch_MRBP, Ch_MRSI, Ch_MRDI
      );

      TInsProp = packed record
        Ch : set of TInsChange;
      end;

      TMemRefSizeInfo = (msiUnknown, msiUnsupported, msiNoSize, msiNoMemRef,
                         msiMultiple, msiMultipleMinSize8, msiMultipleMinSize16, msiMultipleMinSize32,
                         msiMultipleMinSize64, msiMultipleMinSize128, msiMultipleminSize256, msiMultipleMinSize512,
                         msiMemRegSize, msiMemRegx16y32, msiMemRegx16y32z64, msiMemRegx32y64, msiMemRegx32y64z128, msiMemRegx64y128, msiMemRegx64y128z256,
                         msiMemRegx64y256, msiMemRegx64y256z512,
                         msiMem8, msiMem16, msiMem32, msiBMem32, msiMem64, msiBMem64, msiMem128, msiMem256, msiMem512,
                         msiXMem32, msiXMem64, msiYMem32, msiYMem64, msiZMem32, msiZMem64,
                         msiVMemMultiple, msiVMemRegSize,
                         msiMemRegConst128,msiMemRegConst256,msiMemRegConst512);

      TMemRefSizeInfoBCST = (msbUnknown, msbBCST32, msbBCST64, msbMultiple);
      TMemRefSizeInfoBCSTType = (btUnknown, bt1to2, bt1to4, bt1to8, bt1to16);

      TEVEXTupleState = (etsUnknown, etsIsTuple, etsNotTuple);
      TConstSizeInfo  = (csiUnknown, csiMultiple, csiNoSize, csiMem8, csiMem16, csiMem32, csiMem64);

      TInsTabMemRefSizeInfoRec = record
        MemRefSize               : TMemRefSizeInfo;
        MemRefSizeBCST           : TMemRefSizeInfoBCST;
        BCSTXMMMultiplicator     : byte;
        ExistsSSEAVX             : boolean;
        ConstSize                : TConstSizeInfo;
        BCSTTypes                : Set of TMemRefSizeInfoBCSTType;
      end;



    const
      MemRefMultiples: set of TMemRefSizeInfo = [msiMultiple, msiMultipleMinSize8,
                                                 msiMultipleMinSize16, msiMultipleMinSize32,
                                                 msiMultipleMinSize64, msiMultipleMinSize128,
                                                 msiMultipleMinSize256, msiMultipleMinSize512,
                                                 msiVMemMultiple];

      MemRefSizeInfoVMems: Set of TMemRefSizeInfo = [msiXMem32, msiXMem64, msiYMem32, msiYMem64,
                                                     msiZMem32, msiZMem64,
                                                     msiVMemMultiple, msiVMemRegSize];



      InsProp : array[tasmop] of TInsProp =
{$if defined(x86_64)}
        {$i x8664pro.inc}
{$elseif defined(i386)}
        {$i i386prop.inc}
{$elseif defined(i8086)}
        {$i i8086prop.inc}
{$endif}

    type
      TOperandOrder = (op_intel,op_att);

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
        IF_IMM3,                { immediate operand is a triad (must be in range [0..7]) }

        { avx512 flags }
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
        IF_TOVM

      );
      tinsflags=set of tinsflag;

    const
      IF_SMASK=[IF_SM,IF_SM2,IF_SB,IF_SW,IF_SD];
      IF_ARMASK=[IF_AR0,IF_AR1,IF_AR2];  { mask for unsized argument spec  }
      IF_PLEVEL=[IF_8086..IF_NEC]; { mask for processor level }

      IF_TUPLEMASK=[IF_T2..IF_TOVM]; { mask for AVX512 disp8-tuples }


    type
      tinsentry=packed record
        opcode  : tasmop;
        ops     : byte;
        optypes : array[0..max_operands-1] of int64;
        code    : array[0..maxinfolen] of char;
        flags   : tinsflags;
      end;
      pinsentry=^tinsentry;

      { alignment for operator }
      tai_align = class(tai_align_abstract)
         function calculatefillbuf(var buf : tfillbuffer;executable : boolean):pchar;override;
      end;

      { taicpu }

      taicpu = class(tai_cpu_abstract_sym)
         opsize    : topsize;
         constructor op_none(op : tasmop);
         constructor op_none(op : tasmop;_size : topsize);

         constructor op_reg(op : tasmop;_size : topsize;_op1 : tregister);
         constructor op_const(op : tasmop;_size : topsize;_op1 : aint);
         constructor op_ref(op : tasmop;_size : topsize;const _op1 : treference);

         constructor op_reg_reg(op : tasmop;_size : topsize;_op1,_op2 : tregister);
         constructor op_reg_ref(op : tasmop;_size : topsize;_op1 : tregister;const _op2 : treference);
         constructor op_reg_const(op:tasmop; _size: topsize; _op1: tregister; _op2: aint);

         constructor op_const_reg(op : tasmop;_size : topsize;_op1 : aint;_op2 : tregister);
         constructor op_const_const(op : tasmop;_size : topsize;_op1,_op2 : aint);
         constructor op_const_ref(op : tasmop;_size : topsize;_op1 : aint;const _op2 : treference);

         constructor op_ref_reg(op : tasmop;_size : topsize;const _op1 : treference;_op2 : tregister);

         constructor op_reg_reg_reg(op : tasmop;_size : topsize;_op1,_op2,_op3 : tregister);
         constructor op_const_reg_reg(op : tasmop;_size : topsize;_op1 : aint;_op2 : tregister;_op3 : tregister);
         constructor op_const_ref_reg(op : tasmop;_size : topsize;_op1 : aint;const _op2 : treference;_op3 : tregister);
         constructor op_ref_reg_reg(op : tasmop;_size : topsize;const _op1 : treference;_op2,_op3 : tregister);
         constructor op_const_reg_ref(op : tasmop;_size : topsize;_op1 : aint;_op2 : tregister;const _op3 : treference);
         constructor op_reg_reg_ref(op : tasmop;_size : topsize;_op1,_op2 : tregister;const _op3 : treference);
         constructor op_const_reg_reg_reg(op : tasmop;_size : topsize;_op1 : aint;_op2, _op3, _op4 : tregister);

         { this is for Jmp instructions }
         constructor op_cond_sym(op : tasmop;cond:TAsmCond;_size : topsize;_op1 : tasmsymbol);

         constructor op_sym(op : tasmop;_size : topsize;_op1 : tasmsymbol);
         constructor op_sym_ofs(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint);
         constructor op_sym_ofs_reg(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint;_op2 : tregister);
         constructor op_sym_ofs_ref(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint;const _op2 : treference);

         procedure changeopsize(siz:topsize);

         function  GetString:string;

         { This is a workaround for the GAS non commutative fpu instruction braindamage.
           Early versions of the UnixWare assembler had a bug where some fpu instructions
           were reversed and GAS still keeps this "feature" for compatibility.
           for details: http://sourceware.org/binutils/docs/as/i386_002dBugs.html#i386_002dBugs
                        http://bugs.debian.org/cgi-bin/bugreport.cgi?bug=372528
                        http://en.wikibooks.org/wiki/X86_Assembly/GAS_Syntax#Caveats

           Since FPC is "GAS centric" due to its history it generates instructions with the same operand order so
           when generating output for other assemblers, the opcodes must be fixed before writing them.
           This function returns the fixed opcodes. Changing the opcodes permanently is no good idea
           because in case of smartlinking assembler is generated twice so at the second run wrong
           assembler is generated.
           }
         function FixNonCommutativeOpcodes: tasmop;
      private
         FOperandOrder : TOperandOrder;
         procedure init(_size : topsize); { this need to be called by all constructor }
      public
         { the next will reset all instructions that can change in pass 2 }
         procedure ResetPass1;override;
         procedure ResetPass2;override;
         function  CheckIfValid:boolean;
         function  Pass1(objdata:TObjData):longint;override;
         procedure Pass2(objdata:TObjData);override;
         procedure SetOperandOrder(order:TOperandOrder);
         function is_same_reg_move(regtype: Tregistertype):boolean;override;
         { register spilling code }
         function spilling_get_operation_type(opnr: longint): topertype;override;
{$ifdef i8086}
         procedure loadsegsymbol(opidx:longint;s:tasmsymbol);
{$endif i8086}
         property OperandOrder : TOperandOrder read FOperandOrder;
      private
         { next fields are filled in pass1, so pass2 is faster }
         insentry  : PInsEntry;
         insoffset : longint;
         LastInsOffset : longint; { need to be public to be reset }
         inssize   : shortint;
         EVEXTupleState: TEVEXTupleState; { AVX512 disp8*N }
{$ifdef x86_64}
         rex       : byte;
{$endif x86_64}
         function  InsEnd:longint;
         procedure create_ot(objdata:TObjData);
         function  Matches(p:PInsEntry):boolean;
         function  calcsize(p:PInsEntry):shortint;
         procedure gencode(objdata:TObjData);
         function  NeedAddrPrefix(opidx:byte):boolean;
         function  NeedAddrPrefix:boolean;
         procedure write0x66prefix(objdata:TObjData);
         procedure write0x67prefix(objdata:TObjData);
         procedure Swapoperands;
         function  FindInsentry(objdata:TObjData):boolean;
         function  CheckUseEVEX: boolean;
         procedure CheckEVEXTuple(const aInput:toper; aInsEntry: pInsentry; aIsVector128, aIsVector256, aIsVector512, aIsEVEXW1: boolean);
      end;

    function is_64_bit_ref(const ref:treference):boolean;
    function is_32_bit_ref(const ref:treference):boolean;
    function is_16_bit_ref(const ref:treference):boolean;
    function get_ref_address_size(const ref:treference):byte;
    function get_default_segment_of_ref(const ref:treference):tregister;
    procedure optimize_ref(var ref:treference; inlineasm: boolean);

    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
    function spilling_create_store(r:tregister; const ref:treference):Taicpu;

    function MemRefInfo(aAsmop: TAsmOp): TInsTabMemRefSizeInfoRec;
    function MightHaveExtension(AsmOp : TAsmOp) : Boolean;

    procedure InitAsm;
    procedure DoneAsm;


{*****************************************************************************
                              External Symbol Chain
                              used for agx86nsm and agx86int
*****************************************************************************}

    type
      PExternChain = ^TExternChain;

      TExternChain = Record
        psym : pshortstring;
        is_defined : boolean;
        next : PExternChain;
      end;

    const
      FEC : PExternChain = nil;

    procedure AddSymbol(symname : string; defined : boolean);
    procedure FreeExternChainList;

implementation

     uses
       cutils,
       globals,
       systems,
       itcpugas,
       cpuinfo;

    procedure AddSymbol(symname : string; defined : boolean);
    var
       EC : PExternChain;
    begin
      EC:=FEC;
      while assigned(EC) do
        begin
          if EC^.psym^=symname then
            begin
              if defined then
                EC^.is_defined:=true;
              exit;
            end;
          EC:=EC^.next;
        end;
      New(EC);
      EC^.next:=FEC;
      FEC:=EC;
      FEC^.psym:=stringdup(symname);
      FEC^.is_defined := defined;
    end;

    procedure FreeExternChainList;
    var
       EC : PExternChain;
    begin
      EC:=FEC;
      while assigned(EC) do
        begin
          FEC:=EC^.next;
          stringdispose(EC^.psym);
          Dispose(EC);
          EC:=FEC;
        end;
    end;

{*****************************************************************************
                              Instruction table
*****************************************************************************}

     type
       TInsTabCache=array[TasmOp] of longint;
       PInsTabCache=^TInsTabCache;

       TInsTabMemRefSizeInfoCache=array[TasmOp] of TInsTabMemRefSizeInfoRec;
       PInsTabMemRefSizeInfoCache=^TInsTabMemRefSizeInfoCache;

     const
{$if defined(x86_64)}
       InsTab:array[0..instabentries-1] of TInsEntry={$i x8664tab.inc}
{$elseif defined(i386)}
       InsTab:array[0..instabentries-1] of TInsEntry={$i i386tab.inc}
{$elseif defined(i8086)}
       InsTab:array[0..instabentries-1] of TInsEntry={$i i8086tab.inc}
{$endif}
     var
       InsTabCache : PInsTabCache;
       InsTabMemRefSizeInfoCache: PInsTabMemRefSizeInfoCache;
     const
{$if defined(x86_64)}
       { Intel style operands ! }
         opsize_2_type:array[0..2,topsize] of int64=(
         (OT_NONE,
          OT_BITS8,OT_BITS16,OT_BITS32,OT_BITS64,OT_BITS16,OT_BITS32,OT_BITS32,OT_BITS64,OT_BITS64,OT_BITS64,
          OT_BITS16,OT_BITS32,OT_BITS64,
          OT_BITS32,OT_BITS64,OT_BITS80,OT_BITS64,OT_NONE,
          OT_BITS64,
          OT_NEAR,OT_FAR,OT_SHORT,
          OT_NONE,
          OT_BITS128,
          OT_BITS256,
          OT_BITS512
         ),
         (OT_NONE,
          OT_BITS8,OT_BITS16,OT_BITS32,OT_BITS64,OT_BITS8,OT_BITS8,OT_BITS16,OT_BITS8,OT_BITS16,OT_BITS32,
          OT_BITS16,OT_BITS32,OT_BITS64,
          OT_BITS32,OT_BITS64,OT_BITS80,OT_BITS64,OT_NONE,
          OT_BITS64,
          OT_NEAR,OT_FAR,OT_SHORT,
          OT_NONE,
          OT_BITS128,
          OT_BITS256,
          OT_BITS512
         ),
         (OT_NONE,
          OT_BITS8,OT_BITS16,OT_BITS32,OT_BITS64,OT_NONE,OT_NONE,OT_NONE,OT_NONE,OT_NONE,OT_NONE,
          OT_BITS16,OT_BITS32,OT_BITS64,
          OT_BITS32,OT_BITS64,OT_BITS80,OT_BITS64,OT_NONE,
          OT_BITS64,
          OT_NEAR,OT_FAR,OT_SHORT,
          OT_NONE,
          OT_BITS128,
          OT_BITS256,
          OT_BITS512
         )
       );

      reg_ot_table : array[tregisterindex] of longint = (
        {$i r8664ot.inc}
      );
{$elseif defined(i386)}
       { Intel style operands ! }
       opsize_2_type:array[0..2,topsize] of int64=(
         (OT_NONE,
          OT_BITS8,OT_BITS16,OT_BITS32,OT_BITS64,OT_BITS16,OT_BITS32,OT_BITS32,
          OT_BITS16,OT_BITS32,OT_BITS64,
          OT_BITS32,OT_BITS64,OT_BITS80,OT_BITS64,OT_NONE,
          OT_BITS64,
          OT_NEAR,OT_FAR,OT_SHORT,
          OT_NONE,
          OT_BITS128,
          OT_BITS256,
          OT_BITS512
         ),
         (OT_NONE,
          OT_BITS8,OT_BITS16,OT_BITS32,OT_BITS64,OT_BITS8,OT_BITS8,OT_BITS16,
          OT_BITS16,OT_BITS32,OT_BITS64,
          OT_BITS32,OT_BITS64,OT_BITS80,OT_BITS64,OT_NONE,
          OT_BITS64,
          OT_NEAR,OT_FAR,OT_SHORT,
          OT_NONE,
          OT_BITS128,
          OT_BITS256,
          OT_BITS512
         ),
         (OT_NONE,
          OT_BITS8,OT_BITS16,OT_BITS32,OT_BITS64,OT_NONE,OT_NONE,OT_NONE,
          OT_BITS16,OT_BITS32,OT_BITS64,
          OT_BITS32,OT_BITS64,OT_BITS80,OT_BITS64,OT_NONE,
          OT_BITS64,
          OT_NEAR,OT_FAR,OT_SHORT,
          OT_NONE,
          OT_BITS128,
          OT_BITS256,
          OT_BITS512
         )
      );

      reg_ot_table : array[tregisterindex] of longint = (
        {$i r386ot.inc}
      );
{$elseif defined(i8086)}
       { Intel style operands ! }
       opsize_2_type:array[0..2,topsize] of int64=(
         (OT_NONE,
          OT_BITS8,OT_BITS16,OT_BITS32,OT_BITS64,OT_BITS16,OT_BITS32,OT_BITS32,
          OT_BITS16,OT_BITS32,OT_BITS64,
          OT_BITS32,OT_BITS64,OT_BITS80,OT_BITS64,OT_NONE,
          OT_BITS64,
          OT_NEAR,OT_FAR,OT_SHORT,
          OT_NONE,
          OT_BITS128,
          OT_BITS256,
          OT_BITS512
         ),
         (OT_NONE,
          OT_BITS8,OT_BITS16,OT_BITS32,OT_BITS64,OT_BITS8,OT_BITS8,OT_BITS16,
          OT_BITS16,OT_BITS32,OT_BITS64,
          OT_BITS32,OT_BITS64,OT_BITS80,OT_BITS64,OT_NONE,
          OT_BITS64,
          OT_NEAR,OT_FAR,OT_SHORT,
          OT_NONE,
          OT_BITS128,
          OT_BITS256,
          OT_BITS512
         ),
         (OT_NONE,
          OT_BITS8,OT_BITS16,OT_BITS32,OT_BITS64,OT_NONE,OT_NONE,OT_NONE,
          OT_BITS16,OT_BITS32,OT_BITS64,
          OT_BITS32,OT_BITS64,OT_BITS80,OT_BITS64,OT_NONE,
          OT_BITS64,
          OT_NEAR,OT_FAR,OT_SHORT,
          OT_NONE,
          OT_BITS128,
          OT_BITS256,
          OT_BITS512
         )
      );

      reg_ot_table : array[tregisterindex] of longint = (
        {$i r8086ot.inc}
      );
{$endif}

    function MemRefInfo(aAsmop: TAsmOp): TInsTabMemRefSizeInfoRec;
    begin
      result := InsTabMemRefSizeInfoCache^[aAsmop];
    end;


    function MightHaveExtension(AsmOp : TAsmOp): Boolean;
      var
        i,j: LongInt;
        insentry: pinsentry;
      begin
        Result:=true;
        i:=InsTabCache^[AsmOp];
        if i>=0 then
          begin
            insentry:=@instab[i];
            while insentry^.opcode=AsmOp do
              begin
                for j:=0 to insentry^.ops-1 do
                  begin
                    if (insentry^.optypes[j] and OT_VECTOR_EXT)<>0 then
                      exit;
                  end;
                inc(i);
                insentry:=@instab[i];
              end;
          end;
        Result:=false;
      end;

    { Operation type for spilling code }
    type
      toperation_type_table=array[tasmop,0..Max_Operands] of topertype;
    var
      operation_type_table : ^toperation_type_table;


{****************************************************************************
                              TAI_ALIGN
 ****************************************************************************}

    function tai_align.calculatefillbuf(var buf : tfillbuffer;executable : boolean):pchar;
      const
        { Updated according to
          Software Optimization Guide for AMD Family 15h Processors, Verison 3.08, January 2014
          and
          Intel 64 and IA-32 Architectures Software Developer’s Manual
            Volume 2B: Instruction Set Reference, N-Z, January 2015
        }
        alignarray_cmovcpus:array[0..10] of string[11]=(
          #$66#$66#$66#$0F#$1F#$84#$00#$00#$00#$00#$00,
          #$66#$66#$0F#$1F#$84#$00#$00#$00#$00#$00,
          #$66#$0F#$1F#$84#$00#$00#$00#$00#$00,
          #$0F#$1F#$84#$00#$00#$00#$00#$00,
          #$0F#$1F#$80#$00#$00#$00#$00,
          #$66#$0F#$1F#$44#$00#$00,
          #$0F#$1F#$44#$00#$00,
          #$0F#$1F#$40#$00,
          #$0F#$1F#$00,
          #$66#$90,
          #$90);
{$ifdef i8086}
        alignarray:array[0..5] of string[8]=(
          #$90#$90#$90#$90#$90#$90#$90,
          #$90#$90#$90#$90#$90#$90,
          #$90#$90#$90#$90,
          #$90#$90#$90,
          #$90#$90,
          #$90);
{$else i8086}
        alignarray:array[0..5] of string[8]=(
          #$8D#$B4#$26#$00#$00#$00#$00,
          #$8D#$B6#$00#$00#$00#$00,
          #$8D#$74#$26#$00,
          #$8D#$76#$00,
          #$89#$F6,
          #$90);
{$endif i8086}
      var
        bufptr : pchar;
        j : longint;
        localsize: byte;
      begin
        inherited calculatefillbuf(buf,executable);
        if not(use_op) and executable then
         begin
           bufptr:=pchar(@buf);
           { fillsize may still be used afterwards, so don't modify }
           { e.g. writebytes(hp.calculatefillbuf(buf)^,hp.fillsize) }
           localsize:=fillsize;
           while (localsize>0) do
            begin
{$ifndef i8086}
              if CPUX86_HAS_CMOV in cpu_capabilities[current_settings.cputype] then
                begin
                  for j:=low(alignarray_cmovcpus) to high(alignarray_cmovcpus) do
                   if (localsize>=length(alignarray_cmovcpus[j])) then
                    break;
                  move(alignarray_cmovcpus[j][1],bufptr^,length(alignarray_cmovcpus[j]));
                  inc(bufptr,length(alignarray_cmovcpus[j]));
                  dec(localsize,length(alignarray_cmovcpus[j]));
                end
              else
{$endif not i8086}
                begin
                  for j:=low(alignarray) to high(alignarray) do
                   if (localsize>=length(alignarray[j])) then
                    break;
                  move(alignarray[j][1],bufptr^,length(alignarray[j]));
                  inc(bufptr,length(alignarray[j]));
                  dec(localsize,length(alignarray[j]));
                end
            end;
         end;
        calculatefillbuf:=pchar(@buf);
      end;


{*****************************************************************************
                                 Taicpu Constructors
*****************************************************************************}

    procedure taicpu.changeopsize(siz:topsize);
      begin
        opsize:=siz;
      end;


    procedure taicpu.init(_size : topsize);
      begin
         { default order is att }
         FOperandOrder:=op_att;
         segprefix:=NR_NO;
         opsize:=_size;
         insentry:=nil;
         LastInsOffset:=-1;
         InsOffset:=0;
         InsSize:=0;
         EVEXTupleState := etsUnknown;
      end;


    constructor taicpu.op_none(op : tasmop);
      begin
         inherited create(op);
         init(S_NO);
      end;


    constructor taicpu.op_none(op : tasmop;_size : topsize);
      begin
         inherited create(op);
         init(_size);
      end;


    constructor taicpu.op_reg(op : tasmop;_size : topsize;_op1 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=1;
         loadreg(0,_op1);
      end;


    constructor taicpu.op_const(op : tasmop;_size : topsize;_op1 : aint);
      begin
         inherited create(op);
         init(_size);
         ops:=1;
         loadconst(0,_op1);
      end;


    constructor taicpu.op_ref(op : tasmop;_size : topsize;const _op1 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=1;
         loadref(0,_op1);
      end;


    constructor taicpu.op_reg_reg(op : tasmop;_size : topsize;_op1,_op2 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadreg(0,_op1);
         loadreg(1,_op2);
      end;


    constructor taicpu.op_reg_const(op:tasmop; _size: topsize; _op1: tregister; _op2: aint);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadreg(0,_op1);
         loadconst(1,_op2);
      end;


    constructor taicpu.op_reg_ref(op : tasmop;_size : topsize;_op1 : tregister;const _op2 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadreg(0,_op1);
         loadref(1,_op2);
      end;


    constructor taicpu.op_const_reg(op : tasmop;_size : topsize;_op1 : aint;_op2 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadconst(0,_op1);
         loadreg(1,_op2);
      end;


    constructor taicpu.op_const_const(op : tasmop;_size : topsize;_op1,_op2 : aint);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadconst(0,_op1);
         loadconst(1,_op2);
      end;


    constructor taicpu.op_const_ref(op : tasmop;_size : topsize;_op1 : aint;const _op2 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadconst(0,_op1);
         loadref(1,_op2);
      end;


    constructor taicpu.op_ref_reg(op : tasmop;_size : topsize;const _op1 : treference;_op2 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadref(0,_op1);
         loadreg(1,_op2);
      end;


    constructor taicpu.op_reg_reg_reg(op : tasmop;_size : topsize;_op1,_op2,_op3 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=3;
         loadreg(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
      end;


    constructor taicpu.op_const_reg_reg(op : tasmop;_size : topsize;_op1 : aint;_op2 : tregister;_op3 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=3;
         loadconst(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
      end;


    constructor taicpu.op_ref_reg_reg(op : tasmop;_size : topsize;const _op1 : treference;_op2,_op3 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=3;
         loadref(0,_op1);
         loadreg(1,_op2);
         loadreg(2,_op3);
      end;


    constructor taicpu.op_const_ref_reg(op : tasmop;_size : topsize;_op1 : aint;const _op2 : treference;_op3 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=3;
         loadconst(0,_op1);
         loadref(1,_op2);
         loadreg(2,_op3);
      end;


    constructor taicpu.op_const_reg_ref(op : tasmop;_size : topsize;_op1 : aint;_op2 : tregister;const _op3 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=3;
         loadconst(0,_op1);
         loadreg(1,_op2);
         loadref(2,_op3);
      end;


    constructor taicpu.op_reg_reg_ref(op : tasmop;_size : topsize;_op1,_op2 : tregister;const _op3 : treference);
      begin
        inherited create(op);
        init(_size);
        ops:=3;
        loadreg(0,_op1);
        loadreg(1,_op2);
        loadref(2,_op3);
      end;

    constructor taicpu.op_const_reg_reg_reg(op : tasmop; _size : topsize; _op1 : aint; _op2, _op3, _op4 : tregister);
      begin
        inherited create(op);
        init(_size);
        ops:=4;
        loadconst(0,_op1);
        loadreg(1,_op2);
        loadreg(2,_op3);
        loadreg(3,_op4);
      end;


    constructor taicpu.op_cond_sym(op : tasmop;cond:TAsmCond;_size : topsize;_op1 : tasmsymbol);
      begin
         inherited create(op);
         init(_size);
         condition:=cond;
         ops:=1;
         loadsymbol(0,_op1,0);
      end;


    constructor taicpu.op_sym(op : tasmop;_size : topsize;_op1 : tasmsymbol);
      begin
         inherited create(op);
         init(_size);
         ops:=1;
         loadsymbol(0,_op1,0);
      end;


    constructor taicpu.op_sym_ofs(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint);
      begin
         inherited create(op);
         init(_size);
         ops:=1;
         loadsymbol(0,_op1,_op1ofs);
      end;


    constructor taicpu.op_sym_ofs_reg(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint;_op2 : tregister);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadsymbol(0,_op1,_op1ofs);
         loadreg(1,_op2);
      end;


    constructor taicpu.op_sym_ofs_ref(op : tasmop;_size : topsize;_op1 : tasmsymbol;_op1ofs:longint;const _op2 : treference);
      begin
         inherited create(op);
         init(_size);
         ops:=2;
         loadsymbol(0,_op1,_op1ofs);
         loadref(1,_op2);
      end;


    function taicpu.GetString:string;
      var
        i : longint;
        s : string;
        regnr: string;
        addsize : boolean;
      begin
        s:='['+std_op2str[opcode];
        for i:=0 to ops-1 do
         begin
           with oper[i]^ do
             begin
               if i=0 then
                s:=s+' '
               else
                s:=s+',';
               { type }
               addsize:=false;

               regnr := '';
               if getregtype(reg) = R_MMREGISTER then
                str(getsupreg(reg),regnr);

               if (ot and OT_XMMREG)=OT_XMMREG then
                s:=s+'xmmreg' + regnr
               else
                 if (ot and OT_YMMREG)=OT_YMMREG then
                  s:=s+'ymmreg' + regnr
               else
                 if (ot and OT_ZMMREG)=OT_ZMMREG then
                  s:=s+'zmmreg' + regnr

               else
                 if (ot and OT_REG_EXTRA_MASK)=OT_MMXREG then
                  s:=s+'mmxreg'
               else
                 if (ot and OT_REG_EXTRA_MASK)=OT_FPUREG then
                  s:=s+'fpureg'
               else
                if (ot and OT_REGISTER)=OT_REGISTER then
                 begin
                   s:=s+'reg';
                   addsize:=true;
                 end
               else
                if (ot and OT_IMMEDIATE)=OT_IMMEDIATE then
                 begin
                   s:=s+'imm';
                   addsize:=true;
                 end
               else
                if (ot and OT_MEMORY)=OT_MEMORY then
                 begin
                   s:=s+'mem';
                   addsize:=true;
                 end
               else
                 s:=s+'???';
               { size }
               if addsize then
                begin
                  if (ot and OT_BITS8)<>0 then
                    s:=s+'8'
                  else
                   if (ot and OT_BITS16)<>0 then
                    s:=s+'16'
                  else
                   if (ot and OT_BITS32)<>0 then
                    s:=s+'32'
                  else
                   if (ot and OT_BITS64)<>0 then
                    s:=s+'64'
                  else
                   if (ot and OT_BITS128)<>0 then
                    s:=s+'128'
                  else
                   if (ot and OT_BITS256)<>0 then
                    s:=s+'256'
                   else
                    if (ot and OT_BITS512)<>0 then
                     s:=s+'512'
                  else
                    s:=s+'??';
                  { signed }
                  if (ot and OT_SIGNED)<>0 then
                   s:=s+'s';
                end;

               if vopext <> 0 then
                begin
                  str(vopext and $07, regnr);
                  if vopext and OTVE_VECTOR_WRITEMASK = OTVE_VECTOR_WRITEMASK then
                    s := s + ' {k' + regnr + '}';

                  if vopext and OTVE_VECTOR_ZERO = OTVE_VECTOR_ZERO then
                    s := s + ' {z}';

                  if vopext and OTVE_VECTOR_SAE = OTVE_VECTOR_SAE then
                    s := s + ' {sae}';


                  if vopext and OTVE_VECTOR_BCST = OTVE_VECTOR_BCST then
                   case vopext and OTVE_VECTOR_BCST_MASK of
                      OTVE_VECTOR_BCST2: s := s + ' {1to2}';
                      OTVE_VECTOR_BCST4: s := s + ' {1to4}';
                      OTVE_VECTOR_BCST8: s := s + ' {1to8}';
                     OTVE_VECTOR_BCST16: s := s + ' {1to16}';
                   end;

                  if vopext and OTVE_VECTOR_ER = OTVE_VECTOR_ER then
                   case vopext and OTVE_VECTOR_ER_MASK of
                      OTVE_VECTOR_RNSAE: s := s + ' {rn-sae}';
                      OTVE_VECTOR_RDSAE: s := s + ' {rd-sae}';
                      OTVE_VECTOR_RUSAE: s := s + ' {ru-sae}';
                      OTVE_VECTOR_RZSAE: s := s + ' {rz-sae}';
                   end;

                end;
             end;
         end;
        GetString:=s+']';
      end;


    procedure taicpu.Swapoperands;
      var
        p : POper;
      begin
        { Fix the operands which are in AT&T style and we need them in Intel style }
        case ops of
          0,1:
            ;
          2 : begin
                { 0,1 -> 1,0 }
                p:=oper[0];
                oper[0]:=oper[1];
                oper[1]:=p;
              end;
          3 : begin
                { 0,1,2 -> 2,1,0 }
                p:=oper[0];
                oper[0]:=oper[2];
                oper[2]:=p;
              end;
          4 : begin
                { 0,1,2,3 -> 3,2,1,0 }
                p:=oper[0];
                oper[0]:=oper[3];
                oper[3]:=p;
                p:=oper[1];
                oper[1]:=oper[2];
                oper[2]:=p;
              end;
          else
            internalerror(201108141);
        end;
      end;


    procedure taicpu.SetOperandOrder(order:TOperandOrder);
      begin
        if FOperandOrder<>order then
         begin
           Swapoperands;
           FOperandOrder:=order;
         end;
      end;


    function taicpu.FixNonCommutativeOpcodes: tasmop;
      begin
        result:=opcode;

        { we need ATT order }
        SetOperandOrder(op_att);

        if (
            (ops=2) and
            (oper[0]^.typ=top_reg) and
            (oper[1]^.typ=top_reg) and
           { if the first is ST and the second is also a register
             it is necessarily ST1 .. ST7 }
            ((oper[0]^.reg=NR_ST) or
             (oper[0]^.reg=NR_ST0))
           ) or
           { ((ops=1) and
            (oper[0]^.typ=top_reg) and
            (oper[0]^.reg in [R_ST1..R_ST7]))  or}
           (ops=0) then
          begin
            if opcode=A_FSUBR then
              result:=A_FSUB
            else if opcode=A_FSUB then
              result:=A_FSUBR
            else if opcode=A_FDIVR then
              result:=A_FDIV
            else if opcode=A_FDIV then
              result:=A_FDIVR
            else if opcode=A_FSUBRP then
              result:=A_FSUBP
            else if opcode=A_FSUBP then
              result:=A_FSUBRP
            else if opcode=A_FDIVRP then
              result:=A_FDIVP
            else if opcode=A_FDIVP then
              result:=A_FDIVRP;
          end;
        if (
            (ops=1) and
            (oper[0]^.typ=top_reg) and
            (getregtype(oper[0]^.reg)=R_FPUREGISTER) and
            (oper[0]^.reg<>NR_ST)
           ) then
         begin
           if opcode=A_FSUBRP then
             result:=A_FSUBP
           else if opcode=A_FSUBP then
             result:=A_FSUBRP
           else if opcode=A_FDIVRP then
             result:=A_FDIVP
           else if opcode=A_FDIVP then
             result:=A_FDIVRP;
         end;
      end;


{*****************************************************************************
                                Assembler
*****************************************************************************}

    type
      ea = packed record
        sib_present : boolean;
        bytes : byte;
        size  : byte;
        modrm : byte;
        sib   : byte;
{$ifdef x86_64}
        rex   : byte;
{$endif x86_64}
      end;

    procedure taicpu.create_ot(objdata:TObjData);
      {
        this function will also fix some other fields which only needs to be once
      }
      var
        i,l,relsize : longint;
        currsym : TObjSymbol;
      begin
        if ops=0 then
         exit;
        { update oper[].ot field }
        for i:=0 to ops-1 do
         with oper[i]^ do
          begin
            case typ of
              top_reg :
                begin
                  ot:=reg_ot_table[findreg_by_number(reg)];
                end;
              top_ref :
                begin
                  if (ref^.refaddr in [addr_no{$ifdef x86_64},addr_tpoff{$endif x86_64}{$ifdef i386},addr_ntpoff{$endif i386}])
{$ifdef i386}
                     or (
                         (ref^.refaddr in [addr_pic,addr_tlsgd]) and
                         ((ref^.base<>NR_NO) or (ref^.index<>NR_NO))
                        )
{$endif i386}
{$ifdef x86_64}
                     or (
                         (ref^.refaddr in [addr_pic,addr_pic_no_got,addr_tlsgd]) and
                         (ref^.base<>NR_NO)
                        )
{$endif x86_64}
                     then
                    begin

                      { create ot field }
                      if (reg_ot_table[findreg_by_number(ref^.base)] and OT_REG_GPR = OT_REG_GPR) and
                         ((reg_ot_table[findreg_by_number(ref^.index)] = OT_XMMREG) or
                          (reg_ot_table[findreg_by_number(ref^.index)] = OT_YMMREG) or
                          (reg_ot_table[findreg_by_number(ref^.index)] = OT_ZMMREG)
                         ) then
                        // AVX2 - vector-memory-referenz (e.g. vgatherdpd xmm0, [rax  xmm1], xmm2)
                        ot := (reg_ot_table[findreg_by_number(ref^.base)] and OT_REG_GPR) or
                              (reg_ot_table[findreg_by_number(ref^.index)])
                      else if (ref^.base = NR_NO) and
                              ((reg_ot_table[findreg_by_number(ref^.index)] = OT_XMMREG) or
                               (reg_ot_table[findreg_by_number(ref^.index)] = OT_YMMREG) or
                               (reg_ot_table[findreg_by_number(ref^.index)] = OT_ZMMREG)
                              ) then
                        // AVX2 - vector-memory-referenz without base-register (e.g. vgatherdpd xmm0, [xmm1], xmm2)
                        ot := (OT_REG_GPR) or
                              (reg_ot_table[findreg_by_number(ref^.index)])

                      else if (ot and OT_SIZE_MASK)=0 then
                        ot:=OT_MEMORY_ANY or opsize_2_type[i,opsize]
                      else
                        ot:=OT_MEMORY_ANY or (ot and OT_SIZE_MASK);
                      if (ref^.base=NR_NO) and (ref^.index=NR_NO) then
                        ot:=ot or OT_MEM_OFFS;
                      { fix scalefactor }
                      if (ref^.index=NR_NO) then
                       ref^.scalefactor:=0
                      else
                       if (ref^.scalefactor=0) then
                        ref^.scalefactor:=1;
                    end
                  else
                    begin
                      { Jumps use a relative offset which can be 8bit,
                        for other opcodes we always need to generate the full
                        32bit address }
                      if assigned(objdata) and
                         is_jmp then
                        begin
                          currsym:=objdata.symbolref(ref^.symbol);
                          l:=ref^.offset;
{$push}
{$r-,q-} { disable also overflow as address returns a qword for x86_64 }
                          if assigned(currsym) then
                            inc(l,currsym.address);
{$pop}
                          { when it is a forward jump we need to compensate the
                            offset of the instruction since the previous time,
                            because the symbol address is then still using the
                            'old-style' addressing.
                            For backwards jumps this is not required because the
                            address of the symbol is already adjusted to the
                            new offset }
                          if (l>InsOffset) and (LastInsOffset<>-1) then
                            inc(l,InsOffset-LastInsOffset);
                          { instruction size will then always become 2 (PFV) }
                          relsize:=(InsOffset+2)-l;
                          if (relsize>=-128) and (relsize<=127) and
                             (
                              not assigned(currsym) or
                              (currsym.objsection=objdata.currobjsec)
                             ) then
                            ot:=OT_IMM8 or OT_SHORT
                          else
{$ifdef i8086}
                            ot:=OT_IMM16 or OT_NEAR;
{$else i8086}
                            ot:=OT_IMM32 or OT_NEAR;
{$endif i8086}
                        end
                      else
{$ifdef i8086}
                        if opsize=S_FAR then
                          ot:=OT_IMM16 or OT_FAR
                        else
                          ot:=OT_IMM16 or OT_NEAR;
{$else i8086}
                        ot:=OT_IMM32 or OT_NEAR;
{$endif i8086}
                    end;
                end;
              top_local :
                begin
                  if (ot and OT_SIZE_MASK)=0 then
                    ot:=OT_MEMORY or opsize_2_type[i,opsize]
                  else
                    ot:=OT_MEMORY or (ot and OT_SIZE_MASK);
                end;
              top_const :
                begin
                  // if opcode is a SSE or AVX-instruction then we need a
                  // special handling (opsize can different from const-size)
                  // (e.g. "pextrw  reg/m16, xmmreg, imm8" =>> opsize (16 bit), const-size (8 bit)
                  if (InsTabMemRefSizeInfoCache^[opcode].ExistsSSEAVX) and
                     (not(InsTabMemRefSizeInfoCache^[opcode].ConstSize in [csiMultiple, csiUnknown])) then
                  begin
                    case InsTabMemRefSizeInfoCache^[opcode].ConstSize of
                      csiNoSize: ot := ot and OT_NON_SIZE or OT_IMMEDIATE;
                        csiMem8: ot := ot and OT_NON_SIZE or OT_IMMEDIATE or OT_BITS8;
                       csiMem16: ot := ot and OT_NON_SIZE or OT_IMMEDIATE or OT_BITS16;
                       csiMem32: ot := ot and OT_NON_SIZE or OT_IMMEDIATE or OT_BITS32;
                       csiMem64: ot := ot and OT_NON_SIZE or OT_IMMEDIATE or OT_BITS64;
                       else
                         ;
                    end;
                  end
                  else
                  begin
                    { allow 2nd, 3rd or 4th operand being a constant and expect no size for shuf* etc. }
                    { further, allow AAD and AAM with imm. operand }
                    if (opsize=S_NO) and not((i in [1,2,3])
{$ifndef x86_64}
                      or ((i=0) and (opcode in [A_AAD,A_AAM]))
{$endif x86_64}
                      ) then
                      message(asmr_e_invalid_opcode_and_operand);
                    if
{$ifdef i8086}
                       (longint(val)>=-128) and (val<=127) then
{$else i8086}
                       (opsize<>S_W) and
                       (aint(val)>=-128) and (val<=127) then
{$endif not i8086}
                      ot:=OT_IMM8 or OT_SIGNED
                    else
                      ot:=OT_IMMEDIATE or opsize_2_type[i,opsize];
                    if (val=1) and (i=1) then
                      ot := ot or OT_ONENESS;
                  end;
                end;
              top_none :
                begin
                  { generated when there was an error in the
                    assembler reader. It never happends when generating
                    assembler }
                end;
              else
                internalerror(200402266);
            end;
          end;
      end;


    function taicpu.InsEnd:longint;
      begin
        InsEnd:=InsOffset+InsSize;
      end;


      function taicpu.Matches(p:PInsEntry):boolean;
      { * IF_SM stands for Size Match: any operand whose size is not
       * explicitly specified by the template is `really' intended to be
       * the same size as the first size-specified operand.
       * Non-specification is tolerated in the input instruction, but
       * _wrong_ specification is not.
       *
       * IF_SM2 invokes Size Match on only the first _two_ operands, for
       * three-operand instructions such as SHLD: it implies that the
       * first two operands must match in size, but that the third is
       * required to be _unspecified_.
       *
       * IF_SB invokes Size Byte: operands with unspecified size in the
       * template are really bytes, and so no non-byte specification in
       * the input instruction will be tolerated. IF_SW similarly invokes
       * Size Word, and IF_SD invokes Size Doubleword.
       *
       * (The default state if neither IF_SM nor IF_SM2 is specified is
       * that any operand with unspecified size in the template is
       * required to have unspecified size in the instruction too...)
      }
      var
        insot,
        currot: int64;
        i,j,asize,oprs : longint;
        insflags:tinsflags;
        vopext: int64;
        siz : array[0..max_operands-1] of longint;
      begin
        result:=false;

        { Check the opcode and operands }
        if (p^.opcode<>opcode) or (p^.ops<>ops) then
          exit;

{$ifdef i8086}
        { On i8086, we need to skip the i386+ version of Jcc near, if the target
          cpu is earlier than 386. There's another entry, later in the table for
          i8086, which simulates it with i8086 instructions:
            JNcc short +3
            JMP near target }
        if (p^.opcode=A_Jcc) and (current_settings.cputype<cpu_386) and
          (IF_386 in p^.flags) then
          exit;
{$endif i8086}

        for i:=0 to p^.ops-1 do
         begin
           insot:=p^.optypes[i];
           currot:=oper[i]^.ot;

           { Check the operand flags }
           if (insot and (not currot) and OT_NON_SIZE)<>0 then
             exit;

           // IGNORE VECTOR-MEMORY-SIZE
           if insot and OT_TYPE_MASK = OT_MEMORY then
            insot := insot and not(int64(OT_BITS128 or OT_BITS256 or OT_BITS512));


           { Check if the passed operand size matches with one of
             the supported operand sizes }
           if ((insot and OT_SIZE_MASK)<>0) and
              ((insot and currot and OT_SIZE_MASK)<>(currot and OT_SIZE_MASK)) then
             exit;
           { "far" matches only with "far" }
           if (insot and OT_FAR)<>(currot and OT_FAR) then
             exit;
         end;

        { Check operand sizes }
        insflags:=p^.flags;
        if (insflags*IF_SMASK)<>[] then
          begin
            { as default an untyped size can get all the sizes, this is different
              from nasm, but else we need to do a lot checking which opcodes want
              size or not with the automatic size generation }
            asize:=-1;
            if IF_SB in insflags then
              asize:=OT_BITS8
            else if IF_SW in insflags then
              asize:=OT_BITS16
            else if IF_SD in insflags then
              asize:=OT_BITS32;
            if insflags*IF_ARMASK<>[] then
             begin
               siz[0]:=-1;
               siz[1]:=-1;
               siz[2]:=-1;
               if IF_AR0 in insflags then
                 siz[0]:=asize
               else if IF_AR1 in insflags then
                 siz[1]:=asize
               else if IF_AR2 in insflags then
                 siz[2]:=asize
               else
                 internalerror(2017092101);
             end
            else
             begin
               siz[0]:=asize;
               siz[1]:=asize;
               siz[2]:=asize;
             end;

            if insflags*[IF_SM,IF_SM2]<>[] then
             begin
               if IF_SM2 in insflags then
                oprs:=2
               else
                oprs:=p^.ops;
               for i:=0 to oprs-1 do
                if ((p^.optypes[i] and OT_SIZE_MASK) <> 0) then
                 begin
                   for j:=0 to oprs-1 do
                    siz[j]:=p^.optypes[i] and OT_SIZE_MASK;
                   break;
                 end;
              end
             else
              oprs:=2;

            { Check operand sizes }
            for i:=0 to p^.ops-1 do
              begin
                insot:=p^.optypes[i];
                currot:=oper[i]^.ot;
                if ((insot and OT_SIZE_MASK)=0) and
                   ((currot and OT_SIZE_MASK and (not siz[i]))<>0) and
                   { Immediates can always include smaller size }
                   ((currot and OT_IMMEDIATE)=0) and
                    (((insot and OT_SIZE_MASK) or siz[i])<(currot and OT_SIZE_MASK)) then
                  exit;
                if (insot and OT_FAR)<>(currot and OT_FAR) then
                  exit;
              end;
          end;

        if (InsTabMemRefSizeInfoCache^[opcode].MemRefSize in MemRefMultiples) and
           (InsTabMemRefSizeInfoCache^[opcode].ExistsSSEAVX) then
        begin
          for i:=0 to p^.ops-1 do
           begin
             insot:=p^.optypes[i];
             if ((insot and (OT_XMMRM or OT_REG_EXTRA_MASK)) = OT_XMMRM) OR
                ((insot and (OT_YMMRM or OT_REG_EXTRA_MASK)) = OT_YMMRM) OR
                ((insot and (OT_ZMMRM or OT_REG_EXTRA_MASK)) = OT_ZMMRM) then
             begin
               if (insot and OT_SIZE_MASK) = 0 then
               begin
                 case insot and (OT_XMMRM or OT_YMMRM or OT_ZMMRM or OT_REG_EXTRA_MASK) of
                   OT_XMMRM: insot := insot or OT_BITS128;
                   OT_YMMRM: insot := insot or OT_BITS256;
                   OT_ZMMRM: insot := insot or OT_BITS512;
                   else
                     ;
                 end;
               end;
             end;

             currot:=oper[i]^.ot;
             { Check the operand flags }
             if (insot and (not currot) and OT_NON_SIZE)<>0 then
               exit;
             { Check if the passed operand size matches with one of
               the supported operand sizes }
             if ((insot and OT_SIZE_MASK)<>0) and
                ((insot and currot and OT_SIZE_MASK)<>(currot and OT_SIZE_MASK)) then
               exit;
           end;
        end;

        if (InsTabMemRefSizeInfoCache^[opcode].ExistsSSEAVX) then
        begin
          for i:=0 to p^.ops-1 do
           begin
             // check vectoroperand-extention e.g. {k1} {z}

             vopext := 0;
             if (oper[i]^.vopext and OTVE_VECTOR_WRITEMASK) = OTVE_VECTOR_WRITEMASK then
             begin
               vopext := vopext or OT_VECTORMASK;

               if (oper[i]^.vopext and OTVE_VECTOR_ZERO) = OTVE_VECTOR_ZERO then
                vopext := vopext or OT_VECTORZERO;
             end;

             if (oper[i]^.vopext and OTVE_VECTOR_BCST) = OTVE_VECTOR_BCST then
             begin
               vopext := vopext or OT_VECTORBCST;

               if (InsTabMemRefSizeInfoCache^[opcode].BCSTTypes <> []) then
               begin
                 // any opcodes needs a special handling

                 // default broadcast calculation is
                 // bmem32
                 //         xmmreg: {1to4}
                 //         ymmreg: {1to8}
                 //         zmmreg: {1to16}

                 // bmem64
                 //         xmmreg: {1to2}
                 //         ymmreg: {1to4}
                 //         zmmreg: {1to8}

                 // in any opcodes not exists a mmregister
                 // e.g. vfpclasspd  k1, [RAX] {1to8}, 0
                 // =>> check flags


                 case oper[i]^.vopext and (OTVE_VECTOR_BCST2 or OTVE_VECTOR_BCST4 or OTVE_VECTOR_BCST8 or OTVE_VECTOR_BCST16) of
                    OTVE_VECTOR_BCST2: if not(IF_BCST2 in p^.flags) then exit;
                    OTVE_VECTOR_BCST4: if not(IF_BCST4 in p^.flags) then exit;
                    OTVE_VECTOR_BCST8: if not(IF_BCST8 in p^.flags) then exit;
                   OTVE_VECTOR_BCST16: if not(IF_BCST16 in p^.flags) then exit;
                               else exit;
                 end;
               end;

             end;

             if (oper[i]^.vopext and OTVE_VECTOR_ER) = OTVE_VECTOR_ER then
              vopext := vopext or OT_VECTORER;

             if (oper[i]^.vopext and OTVE_VECTOR_SAE) = OTVE_VECTOR_SAE then
              vopext := vopext or OT_VECTORSAE;

             if p^.optypes[i] and vopext <> vopext then
              exit;
           end;
        end;

        result:=true;
      end;


    procedure taicpu.ResetPass1;
      begin
        { we need to reset everything here, because the choosen insentry
          can be invalid for a new situation where the previously optimized
          insentry is not correct }
        InsEntry:=nil;
        InsSize:=0;
        LastInsOffset:=-1;
      end;


    procedure taicpu.ResetPass2;
      begin
        { we are here in a second pass, check if the instruction can be optimized }
        if assigned(InsEntry) and
           (IF_PASS2 in InsEntry^.flags) then
         begin
           InsEntry:=nil;
           InsSize:=0;
         end;
        LastInsOffset:=-1;
      end;


    function taicpu.CheckIfValid:boolean;
      begin
        result:=FindInsEntry(nil);
      end;


    function taicpu.FindInsentry(objdata:TObjData):boolean;
      var
        i : longint;
      begin
        result:=false;
      { Things which may only be done once, not when a second pass is done to
        optimize }

        if (Insentry=nil) or (IF_PASS2 in InsEntry^.flags) then
         begin
           current_filepos:=fileinfo;
           { We need intel style operands }
           SetOperandOrder(op_intel);
           { create the .ot fields }

           create_ot(objdata);
           { set the file postion }
         end
        else
         begin
           { we've already an insentry so it's valid }
           result:=true;
           exit;
         end;
        { Lookup opcode in the table }
        InsSize:=-1;
        i:=instabcache^[opcode];
        if i=-1 then
         begin
           Message1(asmw_e_opcode_not_in_table,gas_op2str[opcode]);
           exit;
         end;
        insentry:=@instab[i];
        while (insentry^.opcode=opcode) do
         begin
           if matches(insentry) then
             begin
               result:=true;
               exit;
             end;
           inc(insentry);
         end;
        Message1(asmw_e_invalid_opcode_and_operands,GetString);
        { No instruction found, set insentry to nil and inssize to -1 }
        insentry:=nil;
        inssize:=-1;
      end;

    function taicpu.CheckUseEVEX: boolean;
    var
      i: integer;
    begin
      result := false;
      for i := 0 to ops - 1 do
      begin
        if (oper[i]^.typ=top_reg) and
           (getregtype(oper[i]^.reg) = R_MMREGISTER) then
         if getsupreg(oper[i]^.reg)>=16 then
          result := true;

        if (oper[i]^.vopext and OTVE_VECTOR_MASK) <> 0 then
         result := true;
      end;
    end;

    procedure taicpu.CheckEVEXTuple(const aInput:toper; aInsEntry: pInsentry; aIsVector128, aIsVector256, aIsVector512, aIsEVEXW1: boolean);
    var
      i: integer;
      tuplesize: integer;
      memsize: integer;
    begin
      if EVEXTupleState = etsUnknown then
      begin
        EVEXTupleState := etsNotTuple;

        if aInsEntry^.Flags * IF_TUPLEMASK <> [] then
        begin
          tuplesize := 0;

          if IF_TFV in aInsEntry^.Flags then
          begin
            for i :=  0 to aInsEntry^.ops - 1 do
             if (aInsEntry^.optypes[i] and OT_BMEM32 = OT_BMEM32) then
             begin
               tuplesize := 4;
               break;
             end
             else if (aInsEntry^.optypes[i] and OT_BMEM64 = OT_BMEM64) then
             begin
               tuplesize := 8;
               break;
             end
             else if (aInsEntry^.optypes[i] and OT_MEMORY = OT_MEMORY) then
             begin
               if aIsVector512 then tuplesize := 64
                else if aIsVector256 then tuplesize := 32
                else tuplesize := 16;

               break;
             end
             else if (aInsEntry^.optypes[i] and OT_REGNORM = OT_REGMEM) then
             begin
               if aIsVector512 then tuplesize := 64
                else if aIsVector256 then tuplesize := 32
                else tuplesize := 16;

               break;
             end;


          end
          else if IF_THV in aInsEntry^.Flags then
          begin
            for i :=  0 to aInsEntry^.ops - 1 do
             if (aInsEntry^.optypes[i] and OT_BMEM32 = OT_BMEM32) then
             begin
               tuplesize := 4;
               break;
             end
             else if (aInsEntry^.optypes[i] and OT_REGNORM = OT_REGMEM) then
             begin
               if aIsVector512 then tuplesize := 32
                else if aIsVector256 then tuplesize := 16
                else tuplesize := 8;

               break;
             end
          end
          else if IF_TFVM in aInsEntry^.Flags then
          begin
            if aIsVector512 then tuplesize := 64
             else if aIsVector256 then tuplesize := 32
             else tuplesize := 16;
          end
          else
          begin
            memsize := 0;

            for i :=  0 to aInsEntry^.ops - 1 do
            begin
              if aInsEntry^.optypes[i] and (OT_REGNORM or OT_MEMORY) = OT_REGMEM then
              begin
                case aInsEntry^.optypes[i] and (OT_BITS32 or OT_BITS64) of
                  OT_BITS32: begin
                               memsize := 32;
                               break;
                             end;
                  OT_BITS64: begin
                               memsize := 64;
                               break;
                             end;
                end;
              end
              else
              case aInsEntry^.optypes[i] and (OT_MEM8 or OT_MEM16 or OT_MEM32 or OT_MEM64) of
                  OT_MEM8: begin
                             memsize := 8;
                             break;
                           end;
                 OT_MEM16: begin
                             memsize := 16;
                             break;
                           end;
                 OT_MEM32: begin
                             memsize := 32;
                             break;
                           end;
                 OT_MEM64: //if aIsEVEXW1 then
                           begin
                             memsize := 64;
                             break;
                           end;
              end;
            end;

            if IF_T1S in aInsEntry^.Flags then
            begin
              case memsize of
                 8: tuplesize := 1;
                16: tuplesize := 2;
                else if aIsEVEXW1 then tuplesize := 8
                      else tuplesize := 4;
              end;
            end
            else if IF_T1F32 in aInsEntry^.Flags then tuplesize := 4
            else if IF_T1F64 in aInsEntry^.Flags then tuplesize := 8
            else if IF_T2 in aInsEntry^.Flags then
            begin
              case aIsEVEXW1 of
                false: tuplesize := 8;
                  else if aIsVector256 or aIsVector512 then tuplesize := 16;
              end;
            end
            else if IF_T4 in aInsEntry^.Flags then
            begin
              case aIsEVEXW1 of
                false: if aIsVector256 or aIsVector512 then tuplesize := 16;
                  else if aIsVector512 then tuplesize := 32;
              end;
            end
            else if IF_T8 in aInsEntry^.Flags then
            begin
              case aIsEVEXW1 of
                false: if aIsVector512 then tuplesize := 32;
                else
                  Internalerror(2019081003);
              end;
            end
            else if IF_THVM in aInsEntry^.Flags then
            begin
              tuplesize := 8; // default 128bit-vectorlength
              if aIsVector256 then tuplesize := 16
               else if aIsVector512 then tuplesize := 32;
            end
            else if IF_TQVM in aInsEntry^.Flags then
            begin
              tuplesize := 4; // default 128bit-vectorlength
              if aIsVector256 then tuplesize := 8
               else if aIsVector512 then tuplesize := 16;
            end
            else if IF_TOVM in aInsEntry^.Flags then
            begin
              tuplesize := 2; // default 128bit-vectorlength
              if aIsVector256 then tuplesize := 4
               else if aIsVector512 then tuplesize := 8;
            end
            else if IF_TMEM128 in aInsEntry^.Flags then tuplesize := 16
            else if IF_TMDDUP in aInsEntry^.Flags then
            begin
              tuplesize := 8; // default 128bit-vectorlength
              if aIsVector256 then tuplesize := 32
               else if aIsVector512 then tuplesize := 64;
            end;
          end;;

          if tuplesize > 0 then
          begin
            if aInput.typ = top_ref then
            begin
              if (aInput.ref^.offset <> 0) and
                 ((aInput.ref^.offset mod tuplesize) = 0) and
                 (abs(aInput.ref^.offset) div tuplesize <= 127) then
              begin
                aInput.ref^.offset := aInput.ref^.offset div tuplesize;
                EVEXTupleState := etsIsTuple;
              end;
            end;
          end;
        end;
      end;
    end;



    function taicpu.Pass1(objdata:TObjData):longint;
      begin
        Pass1:=0;
        { Save the old offset and set the new offset }
        InsOffset:=ObjData.CurrObjSec.Size;
        { Error? }
        if (Insentry=nil) and (InsSize=-1) then
          exit;
        { set the file postion }
        current_filepos:=fileinfo;
        { Get InsEntry }
        if FindInsEntry(ObjData) then
         begin

           { Calculate instruction size }
           InsSize:=calcsize(insentry);
           if segprefix<>NR_NO then
            inc(InsSize);
           if NeedAddrPrefix then
            inc(InsSize);
           { Fix opsize if size if forced }
           if insentry^.flags*[IF_SB,IF_SW,IF_SD]<>[] then
             begin
               if insentry^.flags*IF_ARMASK=[] then
                 begin
                   if IF_SB in insentry^.flags then
                     begin
                       if opsize=S_NO then
                         opsize:=S_B;
                     end
                   else if IF_SW in insentry^.flags then
                     begin
                       if opsize=S_NO then
                         opsize:=S_W;
                     end
                   else if IF_SD in insentry^.flags then
                     begin
                       if opsize=S_NO then
                         opsize:=S_L;
                     end;
                 end;
             end;
           LastInsOffset:=InsOffset;
           Pass1:=InsSize;
           exit;
         end;
        LastInsOffset:=-1;
      end;

    const
      segprefixes: array[NR_ES..NR_GS] of Byte=(
      // es  cs   ss   ds   fs   gs
        $26, $2E, $36, $3E, $64, $65
      );

    procedure taicpu.Pass2(objdata:TObjData);
      begin
        { error in pass1 ? }
        if insentry=nil then
         exit;
        current_filepos:=fileinfo;
        { Segment override }
        if (segprefix>=NR_ES) and (segprefix<=NR_GS) then
         begin
{$ifdef i8086}
           if (objdata.CPUType<>cpu_none) and (objdata.CPUType<cpu_386) and
              ((segprefix=NR_FS) or (segprefix=NR_GS)) then
             Message(asmw_e_instruction_not_supported_by_cpu);
{$endif i8086}
           objdata.writebytes(segprefixes[segprefix],1);
           { fix the offset for GenNode }
           inc(InsOffset);
         end
        else if segprefix<>NR_NO then
          InternalError(201001071);
        { Address size prefix? }
        if NeedAddrPrefix then
        begin
          write0x67prefix(objdata);
          { fix the offset for GenNode }
          inc(InsOffset);
        end;
        { Generate the instruction }
        GenCode(objdata);
      end;


    function is_64_bit_ref(const ref:treference):boolean;
      begin
{$if defined(x86_64)}
        result:=not is_32_bit_ref(ref);
{$elseif defined(i386) or defined(i8086)}
        result:=false;
{$endif}
      end;


    function is_32_bit_ref(const ref:treference):boolean;
      begin
{$if defined(x86_64)}
        result:=(ref.refaddr=addr_no) and
                (ref.base<>NR_RIP) and
                (
                 ((ref.index<>NR_NO) and (getsubreg(ref.index)=R_SUBD)) or
                 ((ref.base<>NR_NO) and (getsubreg(ref.base)=R_SUBD))
                );
{$elseif defined(i386) or defined(i8086)}
        result:=not is_16_bit_ref(ref);
{$endif}
      end;


    function is_16_bit_ref(const ref:treference):boolean;
      var
        ir,br : Tregister;
        isub,bsub : tsubregister;
      begin
        if (ref.index<>NR_NO) and (getregtype(ref.index)=R_MMREGISTER) then
          exit(false);
        ir:=ref.index;
        br:=ref.base;
        isub:=getsubreg(ir);
        bsub:=getsubreg(br);
        { it's a direct address }
        if (br=NR_NO) and (ir=NR_NO) then
          begin
            {$ifdef i8086}
            result:=true;
            {$else i8086}
            result:=false;
            {$endif}
          end
        else
          { it's an indirection }
          begin
            result := ((ir<>NR_NO) and (isub=R_SUBW)) or
                      ((br<>NR_NO) and (bsub=R_SUBW));
          end;
      end;


    function get_ref_address_size(const ref:treference):byte;
      begin
        if is_64_bit_ref(ref) then
          result:=64
        else if is_32_bit_ref(ref) then
          result:=32
        else if is_16_bit_ref(ref) then
          result:=16
        else
          internalerror(2017101601);
      end;


    function get_default_segment_of_ref(const ref:treference):tregister;
      begin
        { for 16-bit registers, we allow base and index to be swapped, that's
        why we also we check whether ref.index=NR_BP. For 32-bit registers,
        however, index=NR_EBP is encoded differently than base=NR_EBP and has
        a different default segment. }
        if (ref.base=NR_BP) or (ref.index=NR_BP) or
           (ref.base=NR_EBP) or (ref.base=NR_ESP)
{$ifdef x86_64}
        or (ref.base=NR_RBP) or (ref.base=NR_RSP)
{$endif x86_64}
           then
          result:=NR_SS
        else
          result:=NR_DS;
      end;


    procedure optimize_ref(var ref:treference; inlineasm: boolean);
      var
        ss_equals_ds: boolean;
        tmpreg: TRegister;
      begin
{$ifdef x86_64}
        { x86_64 in long mode ignores all segment base, limit and access rights
          checks for the DS, ES and SS registers, so we can set ss_equals_ds to
          true (and thus, perform stronger optimizations on the reference),
          regardless of whether this is inline asm or not (so, even if the user
          is doing tricks by loading different values into DS and SS, it still
          doesn't matter while the processor is in long mode) }
        ss_equals_ds:=True;
{$else x86_64}
        { for i8086 and i386 inline asm, we assume SS<>DS, even if we're
          compiling for a memory model, where SS=DS, because the user might be
          doing something tricky with the segment registers (and may have
          temporarily set them differently) }
        if inlineasm then
          ss_equals_ds:=False
        else
          ss_equals_ds:=segment_regs_equal(NR_DS,NR_SS);
{$endif x86_64}
        { remove redundant segment overrides }
        if (ref.segment<>NR_NO) and
           ((inlineasm and (ref.segment=get_default_segment_of_ref(ref))) or
            ((not inlineasm) and (segment_regs_equal(ref.segment,get_default_segment_of_ref(ref))))) then
          ref.segment:=NR_NO;
        if not is_16_bit_ref(ref) then
          begin
            { Switching index to base position gives shorter assembler instructions.
              Converting index*2 to base+index also gives shorter instructions. }
            if (ref.base=NR_NO) and (ref.index<>NR_NO) and (ref.scalefactor<=2) and
               (ss_equals_ds or (ref.segment<>NR_NO) or (ref.index<>NR_EBP))
               { do not mess with tls references, they have the (,reg,1) format on purpose
                 else the linker cannot resolve/replace them }
               {$ifdef i386} and (ref.refaddr<>addr_tlsgd) {$endif i386} then
              begin
                ref.base:=ref.index;
                if ref.scalefactor=2 then
                  ref.scalefactor:=1
                else
                  begin
                    ref.index:=NR_NO;
                    ref.scalefactor:=0;
                  end;
              end;
            { Switching rBP+reg to reg+rBP sometimes gives shorter instructions (if there's no offset)
              On x86_64 this also works for switching r13+reg to reg+r13. }
            if ((ref.base=NR_EBP) {$ifdef x86_64}or (ref.base=NR_RBP) or (ref.base=NR_R13) or (ref.base=NR_R13D){$endif}) and
               (ref.index<>NR_NO) and
               (ref.index<>NR_EBP) and {$ifdef x86_64}(ref.index<>NR_RBP) and (ref.index<>NR_R13) and (ref.index<>NR_R13D) and{$endif}
               (ref.scalefactor<=1) and (ref.offset=0) and (ref.refaddr=addr_no) and
               (ss_equals_ds or (ref.segment<>NR_NO)) then
              begin
                tmpreg:=ref.base;
                ref.base:=ref.index;
                ref.index:=tmpreg;
              end;
          end;
        { remove redundant segment overrides again }
        if (ref.segment<>NR_NO) and
           ((inlineasm and (ref.segment=get_default_segment_of_ref(ref))) or
            ((not inlineasm) and (segment_regs_equal(ref.segment,get_default_segment_of_ref(ref))))) then
          ref.segment:=NR_NO;
      end;


        function taicpu.NeedAddrPrefix(opidx: byte): boolean;
      begin
{$if defined(x86_64)}
        result:=(oper[opidx]^.typ=top_ref) and is_32_bit_ref(oper[opidx]^.ref^);
{$elseif defined(i386)}
        result:=(oper[opidx]^.typ=top_ref) and is_16_bit_ref(oper[opidx]^.ref^);
{$elseif defined(i8086)}
        result:=(oper[opidx]^.typ=top_ref) and is_32_bit_ref(oper[opidx]^.ref^);
{$endif}
      end;


    function taicpu.NeedAddrPrefix:boolean;
      var
        i: Integer;
      begin
        for i:=0 to ops-1 do
          if needaddrprefix(i) then
            exit(true);
        result:=false;
      end;


    procedure badreg(r:Tregister);
      begin
        Message1(asmw_e_invalid_register,generic_regname(r));
      end;


    function regval(r:Tregister):byte;
      const
        intsupreg2opcode: array[0..7] of byte=
        // ax cx dx bx si di bp sp   -- in x86reg.dat
        // ax cx dx bx sp bp si di   -- needed order
          (0, 1, 2, 3, 6, 7, 5, 4);
        maxsupreg: array[tregistertype] of tsuperregister=
{$ifdef x86_64}
          (0, 16, 9, 8, 32, 32, 8, 0, 0);
{$else x86_64}
          (0,  8, 9, 8,  8, 32, 8, 0, 0);
{$endif x86_64}
      var
        rs: tsuperregister;
        rt: tregistertype;
      begin
        rs:=getsupreg(r);
        rt:=getregtype(r);

        if (rs>=maxsupreg[rt]) then
         badreg(r);

        result:=rs and 7;
        if (rt=R_INTREGISTER) then
          begin
            if (rs<8) then
              result:=intsupreg2opcode[rs];
            if getsubreg(r)=R_SUBH then
              inc(result,4);
          end;
      end;


{$if defined(x86_64)}
    function rexbits(r: tregister): byte;
      begin
        result:=0;
        case getregtype(r) of
          R_INTREGISTER:
            if (getsupreg(r)>=RS_R8) then
          { Either B,X or R bits can be set, depending on register role in instruction.
            Set all three bits here, caller will discard unnecessary ones. }
              result:=result or $47
            else if (getsubreg(r)=R_SUBL) and
              (getsupreg(r) in [RS_RDI,RS_RSI,RS_RBP,RS_RSP]) then
              result:=result or $40
            else if (getsubreg(r)=R_SUBH) then
          { Not an actual REX bit, used to detect incompatible usage of
            AH/BH/CH/DH }
              result:=result or $80;
          R_MMREGISTER:
            //if getsupreg(r)>=RS_XMM8 then
            // AVX512 = 32 register
            //    rexbit = 0 => MMRegister 0..7  or 16..23
            //    rexbit = 1 => MMRegister 8..15 or 24..31
            if (getsupreg(r) and $08) = $08 then
              result:=result or $47;
          else
            ;
        end;
      end;



    function process_ea_ref_64_32(const input:toper;var output:ea;rfield:longint; uselargeoffset: boolean):boolean;
      var
        sym   : tasmsymbol;
        md,s  : byte;
        base,index,scalefactor,
        o     : longint;
        ir,br : Tregister;
        isub,bsub : tsubregister;
      begin
        result:=false;
        ir:=input.ref^.index;
        br:=input.ref^.base;
        isub:=getsubreg(ir);
        bsub:=getsubreg(br);
        s:=input.ref^.scalefactor;
        o:=input.ref^.offset;
        sym:=input.ref^.symbol;

        //if ((ir<>NR_NO) and (getregtype(ir)<>R_INTREGISTER)) or
        //   ((br<>NR_NO) and (br<>NR_RIP) and (getregtype(br)<>R_INTREGISTER)) then
        if ((ir<>NR_NO) and (getregtype(ir)=R_MMREGISTER) and (br<>NR_NO) and (getregtype(br)<>R_INTREGISTER)) or // vector memory (AVX2)
           ((ir<>NR_NO) and (getregtype(ir)<>R_INTREGISTER) and (getregtype(ir)<>R_MMREGISTER)) or
           ((br<>NR_NO) and (br<>NR_RIP) and (getregtype(br)<>R_INTREGISTER)) then
          internalerror(200301081);
        { it's direct address }
        if (br=NR_NO) and (ir=NR_NO) then
         begin
           output.sib_present:=true;
           output.bytes:=4;
           output.modrm:=4 or (rfield shl 3);
           output.sib:=$25;
         end
        else if (br=NR_RIP) and (ir=NR_NO) then
          begin
            { rip based }
            output.sib_present:=false;
            output.bytes:=4;
            output.modrm:=5 or (rfield shl 3);
          end
        else
        { it's an indirection }
         begin
           if ((br=NR_RIP) and (ir<>NR_NO)) or
             (ir=NR_RIP) then
             message(asmw_e_illegal_use_of_rip);
           { 16 bit? }

           if ((ir<>NR_NO) and (isub in [R_SUBMMX,R_SUBMMY,R_SUBMMZ]) and
               (br<>NR_NO) and (bsub=R_SUBQ)
              ) then
           begin
             // vector memory (AVX2) =>> ignore
           end
           else if ((ir<>NR_NO) and (isub<>R_SUBQ) and (isub<>R_SUBD)) or
                   ((br<>NR_NO) and (bsub<>R_SUBQ) and (bsub<>R_SUBD)) then
           begin
             message(asmw_e_16bit_32bit_not_supported);
           end;

           { wrong, for various reasons }
           if (ir=NR_ESP) or ((s<>1) and (s<>2) and (s<>4) and (s<>8) and (ir<>NR_NO)) then
            exit;

           output.rex:=output.rex or (rexbits(br) and $F1) or (rexbits(ir) and $F2);
           result:=true;


           { base }
           case br of
             NR_R8D,
             NR_EAX,
             NR_R8,
             NR_RAX : base:=0;
             NR_R9D,
             NR_ECX,
             NR_R9,
             NR_RCX : base:=1;
             NR_R10D,
             NR_EDX,
             NR_R10,
             NR_RDX : base:=2;
             NR_R11D,
             NR_EBX,
             NR_R11,
             NR_RBX : base:=3;
             NR_R12D,
             NR_ESP,
             NR_R12,
             NR_RSP : base:=4;
             NR_R13D,
             NR_EBP,
             NR_R13,
             NR_NO,
             NR_RBP : base:=5;
             NR_R14D,
             NR_ESI,
             NR_R14,
             NR_RSI : base:=6;
             NR_R15D,
             NR_EDI,
             NR_R15,
             NR_RDI : base:=7;
           else
             exit;
           end;
           { index }
           case ir of
             NR_R8D,
             NR_EAX,
             NR_R8,
             NR_RAX,
             NR_XMM0,
             NR_XMM8,
             NR_XMM16,
             NR_XMM24,
             NR_YMM0,
             NR_YMM8,
             NR_YMM16,
             NR_YMM24,
             NR_ZMM0,
             NR_ZMM8,
             NR_ZMM16,
             NR_ZMM24: index:=0;
             NR_R9D,
             NR_ECX,
             NR_R9,
             NR_RCX,
             NR_XMM1,
             NR_XMM9,
             NR_XMM17,
             NR_XMM25,
             NR_YMM1,
             NR_YMM9,
             NR_YMM17,
             NR_YMM25,
             NR_ZMM1,
             NR_ZMM9,
             NR_ZMM17,
             NR_ZMM25: index:=1;
             NR_R10D,
             NR_EDX,
             NR_R10,
             NR_RDX,
             NR_XMM2,
             NR_XMM10,
             NR_XMM18,
             NR_XMM26,
             NR_YMM2,
             NR_YMM10,
             NR_YMM18,
             NR_YMM26,
             NR_ZMM2,
             NR_ZMM10,
             NR_ZMM18,
             NR_ZMM26: index:=2;
             NR_R11D,
             NR_EBX,
             NR_R11,
             NR_RBX,
             NR_XMM3,
             NR_XMM11,
             NR_XMM19,
             NR_XMM27,
             NR_YMM3,
             NR_YMM11,
             NR_YMM19,
             NR_YMM27,
             NR_ZMM3,
             NR_ZMM11,
             NR_ZMM19,
             NR_ZMM27: index:=3;
             NR_R12D,
             NR_ESP,
             NR_R12,
             NR_NO,
             NR_XMM4,
             NR_XMM12,
             NR_XMM20,
             NR_XMM28,
             NR_YMM4,
             NR_YMM12,
             NR_YMM20,
             NR_YMM28,
             NR_ZMM4,
             NR_ZMM12,
             NR_ZMM20,
             NR_ZMM28: index:=4;
             NR_R13D,
             NR_EBP,
             NR_R13,
             NR_RBP,
             NR_XMM5,
             NR_XMM13,
             NR_XMM21,
             NR_XMM29,
             NR_YMM5,
             NR_YMM13,
             NR_YMM21,
             NR_YMM29,
             NR_ZMM5,
             NR_ZMM13,
             NR_ZMM21,
             NR_ZMM29: index:=5;
             NR_R14D,
             NR_ESI,
             NR_R14,
             NR_RSI,
             NR_XMM6,
             NR_XMM14,
             NR_XMM22,
             NR_XMM30,
             NR_YMM6,
             NR_YMM14,
             NR_YMM22,
             NR_YMM30,
             NR_ZMM6,
             NR_ZMM14,
             NR_ZMM22,
             NR_ZMM30: index:=6;
             NR_R15D,
             NR_EDI,
             NR_R15,
             NR_RDI,
             NR_XMM7,
             NR_XMM15,
             NR_XMM23,
             NR_XMM31,
             NR_YMM7,
             NR_YMM15,
             NR_YMM23,
             NR_YMM31,
             NR_ZMM7,
             NR_ZMM15,
             NR_ZMM23,
             NR_ZMM31: index:=7;
           else
             exit;
           end;
           case s of
            0,
            1 : scalefactor:=0;
            2 : scalefactor:=1;
            4 : scalefactor:=2;
            8 : scalefactor:=3;
           else
            exit;
           end;
           { If rbp or r13 is used we must always include an offset }
           if (br=NR_NO) or
              ((br<>NR_RBP) and (br<>NR_R13) and (br<>NR_EBP) and (br<>NR_R13D) and (o=0) and (sym=nil)) then
            md:=0
           else
            if ((o>=-128) and (o<=127) and (sym=nil) and (not(uselargeoffset) or (o = 0))) then
             md:=1
            else
             md:=2;
           if (br=NR_NO) or (md=2) then
            output.bytes:=4
           else
            output.bytes:=md;
           { SIB needed ? }
           if (ir=NR_NO) and (br<>NR_RSP) and (br<>NR_R12) and (br<>NR_ESP) and (br<>NR_R12D)  then
            begin
              output.sib_present:=false;
              output.modrm:=(md shl 6) or (rfield shl 3) or base;
            end
           else
            begin
              output.sib_present:=true;
              output.modrm:=(md shl 6) or (rfield shl 3) or 4;
              output.sib:=(scalefactor shl 6) or (index shl 3) or base;
            end;
         end;
        output.size:=1+ord(output.sib_present)+output.bytes;
        result:=true;
      end;


{$elseif defined(i386) or defined(i8086)}

    function process_ea_ref_32(const input:toper;out output:ea;rfield:longint; uselargeoffset: boolean):boolean;
      var
        sym   : tasmsymbol;
        md,s  : byte;
        base,index,scalefactor,
        o     : longint;
        ir,br : Tregister;
        isub,bsub : tsubregister;
      begin
        result:=false;
        if ((input.ref^.index<>NR_NO) and (getregtype(input.ref^.index)=R_MMREGISTER) and (input.ref^.base<>NR_NO) and (getregtype(input.ref^.base)<>R_INTREGISTER)) or // vector memory (AVX2)
           ((input.ref^.index<>NR_NO) and (getregtype(input.ref^.index)<>R_INTREGISTER) and (getregtype(input.ref^.index)<>R_MMREGISTER)) or
           ((input.ref^.base<>NR_NO) and (getregtype(input.ref^.base)<>R_INTREGISTER)) then
         internalerror(200301081);


        ir:=input.ref^.index;
        br:=input.ref^.base;
        isub:=getsubreg(ir);
        bsub:=getsubreg(br);
        s:=input.ref^.scalefactor;
        o:=input.ref^.offset;
        sym:=input.ref^.symbol;
      { it's direct address }
        if (br=NR_NO) and (ir=NR_NO) then
         begin
           { it's a pure offset }
           output.sib_present:=false;
           output.bytes:=4;
           output.modrm:=5 or (rfield shl 3);
         end
        else
        { it's an indirection }
         begin
           { 16 bit address? }

           if ((ir<>NR_NO) and (isub in [R_SUBMMX,R_SUBMMY,R_SUBMMZ]) and
               (br<>NR_NO) and (bsub=R_SUBD)
              ) then
           begin
             // vector memory (AVX2) =>> ignore
           end
           else if ((ir<>NR_NO) and (isub<>R_SUBD)) or
                   ((br<>NR_NO) and (bsub<>R_SUBD)) then
             message(asmw_e_16bit_not_supported);
{$ifdef OPTEA}
           { make single reg base }
           if (br=NR_NO) and (s=1) then
            begin
              br:=ir;
              ir:=NR_NO;
            end;
           { convert [3,5,9]*EAX to EAX+[2,4,8]*EAX }
           if (br=NR_NO) and
              (((s=2) and (ir<>NR_ESP)) or
                (s=3) or (s=5) or (s=9)) then
            begin
              br:=ir;
              dec(s);
            end;
           { swap ESP into base if scalefactor is 1 }
           if (s=1) and (ir=NR_ESP) then
            begin
              ir:=br;
              br:=NR_ESP;
            end;
{$endif OPTEA}
           { wrong, for various reasons }
           if (ir=NR_ESP) or ((s<>1) and (s<>2) and (s<>4) and (s<>8) and (ir<>NR_NO)) then
            exit;
           { base }
           case br of
             NR_EAX : base:=0;
             NR_ECX : base:=1;
             NR_EDX : base:=2;
             NR_EBX : base:=3;
             NR_ESP : base:=4;
             NR_NO,
             NR_EBP : base:=5;
             NR_ESI : base:=6;
             NR_EDI : base:=7;
           else
             exit;
           end;
           { index }
           case ir of
             NR_EAX,
             NR_XMM0,
             NR_YMM0,
	     NR_ZMM0: index:=0;
             NR_ECX,
             NR_XMM1,
             NR_YMM1,
	     NR_ZMM1: index:=1;
             NR_EDX,
             NR_XMM2,
             NR_YMM2,
	     NR_ZMM2: index:=2;
             NR_EBX,
             NR_XMM3,
             NR_YMM3,
	     NR_ZMM3: index:=3;
             NR_NO,
             NR_XMM4,
             NR_YMM4,
	     NR_ZMM4: index:=4;
             NR_EBP,
             NR_XMM5,
             NR_YMM5,
	     NR_ZMM5: index:=5;
             NR_ESI,
             NR_XMM6,
             NR_YMM6,
	     NR_ZMM6: index:=6;
             NR_EDI,
             NR_XMM7,
             NR_YMM7,
	     NR_ZMM7: index:=7;
           else
             exit;
           end;
           case s of
            0,
            1 : scalefactor:=0;
            2 : scalefactor:=1;
            4 : scalefactor:=2;
            8 : scalefactor:=3;
           else
            exit;
           end;
           if (br=NR_NO) or
              ((br<>NR_EBP) and (o=0) and (sym=nil)) then
            md:=0
           else
            if ((o>=-128) and (o<=127) and (sym=nil) and (not(uselargeoffset) or (o = 0))) then
             md:=1
            else
             md:=2;
           if (br=NR_NO) or (md=2) then
            output.bytes:=4
           else
            output.bytes:=md;
           { SIB needed ? }
           if (ir=NR_NO) and (br<>NR_ESP) then
            begin
              output.sib_present:=false;
              output.modrm:=(longint(md) shl 6) or (rfield shl 3) or base;
            end
           else
            begin
              output.sib_present:=true;
              output.modrm:=(longint(md) shl 6) or (rfield shl 3) or 4;
              output.sib:=(scalefactor shl 6) or (index shl 3) or base;
            end;
         end;
        if output.sib_present then
         output.size:=2+output.bytes
        else
         output.size:=1+output.bytes;
        result:=true;
      end;

    procedure maybe_swap_index_base(var br,ir:Tregister);
      var
        tmpreg: Tregister;
      begin
        if ((br=NR_NO) or (br=NR_SI) or (br=NR_DI)) and
           ((ir=NR_NO) or (ir=NR_BP) or (ir=NR_BX)) then
          begin
            tmpreg:=br;
            br:=ir;
            ir:=tmpreg;
          end;
      end;

    function process_ea_ref_16(const input:toper;out output:ea;rfield:longint; uselargeoffset: boolean):boolean;
      var
        sym   : tasmsymbol;
        md,s  : byte;
        base,
        o     : longint;
        ir,br : Tregister;
        isub,bsub : tsubregister;
      begin
        result:=false;
        if ((input.ref^.index<>NR_NO) and (getregtype(input.ref^.index)<>R_INTREGISTER)) or
           ((input.ref^.base<>NR_NO) and (getregtype(input.ref^.base)<>R_INTREGISTER)) then
          internalerror(200301081);


        ir:=input.ref^.index;
        br:=input.ref^.base;
        isub:=getsubreg(ir);
        bsub:=getsubreg(br);
        s:=input.ref^.scalefactor;
        o:=input.ref^.offset;
        sym:=input.ref^.symbol;
        { it's a direct address }
        if (br=NR_NO) and (ir=NR_NO) then
          begin
            { it's a pure offset }
            output.bytes:=2;
            output.modrm:=6 or (rfield shl 3);
          end
        else
          { it's an indirection }
          begin
            { 32 bit address? }

            if ((ir<>NR_NO) and (isub<>R_SUBW)) or
               ((br<>NR_NO) and (bsub<>R_SUBW)) then
              message(asmw_e_32bit_not_supported);
            { scalefactor can only be 1 in 16-bit addresses }
            if (s<>1) and (ir<>NR_NO) then
              exit;
            maybe_swap_index_base(br,ir);
            if (br=NR_BX) and (ir=NR_SI) then
              base:=0
            else if (br=NR_BX) and (ir=NR_DI) then
              base:=1
            else if (br=NR_BP) and (ir=NR_SI) then
              base:=2
            else if (br=NR_BP) and (ir=NR_DI) then
              base:=3
            else if (br=NR_NO) and (ir=NR_SI) then
              base:=4
            else if (br=NR_NO) and (ir=NR_DI) then
              base:=5
            else if (br=NR_BP) and (ir=NR_NO) then
              base:=6
            else if (br=NR_BX) and (ir=NR_NO) then
              base:=7
            else
              exit;
            if (base<>6) and (o=0) and (sym=nil) then
              md:=0
            else if ((o>=-128) and (o<=127) and (sym=nil) and (not(uselargeoffset) or (o = 0))) then
              md:=1
            else
              md:=2;
            output.bytes:=md;
            output.modrm:=(longint(md) shl 6) or (rfield shl 3) or base;
          end;
        output.size:=1+output.bytes;
        output.sib_present:=false;
        result:=true;
      end;
{$endif}

    function process_ea(const input:toper;out output:ea;rfield:longint; uselargeoffset: boolean):boolean;
      var
        rv  : byte;
      begin
        result:=false;
        fillchar(output,sizeof(output),0);
        {Register ?}
        if (input.typ=top_reg) then
          begin
            rv:=regval(input.reg);
            output.modrm:=$c0 or (rfield shl 3) or rv;
            output.size:=1;
{$ifdef x86_64}
            output.rex:=output.rex or (rexbits(input.reg) and $F1);
{$endif x86_64}
            result:=true;
            exit;
          end;
        {No register, so memory reference.}
        if input.typ<>top_ref then
          internalerror(200409263);
{$if defined(x86_64)}
        result:=process_ea_ref_64_32(input,output,rfield, uselargeoffset);
{$elseif defined(i386) or defined(i8086)}
        if is_16_bit_ref(input.ref^) then
          result:=process_ea_ref_16(input,output,rfield, uselargeoffset)
        else
          result:=process_ea_ref_32(input,output,rfield, uselargeoffset);
{$endif}
      end;

    function taicpu.calcsize(p:PInsEntry):shortint;
      var
        codes : pchar;
        c     : byte;
        len     : shortint;
        ea_data : ea;
        exists_evex: boolean;
        exists_vex: boolean;
        exists_vex_extension: boolean;
        exists_prefix_66: boolean;
        exists_prefix_F2: boolean;
        exists_prefix_F3: boolean;
        exists_l256: boolean;
        exists_l512: boolean;
        exists_EVEXW1: boolean;
{$ifdef x86_64}
        omit_rexw : boolean;
{$endif x86_64}
      begin

        len:=0;

        codes:=@p^.code[0];
        exists_vex := false;
        exists_vex_extension := false;
        exists_prefix_66 := false;
        exists_prefix_F2 := false;
        exists_prefix_F3 := false;
        exists_evex      := false;
        exists_l256      := false;
        exists_l512      := false;
        exists_EVEXW1    := false;
{$ifdef x86_64}
        rex:=0;
        omit_rexw:=false;
{$endif x86_64}
        repeat
          c:=ord(codes^);
          inc(codes);
          case c of
            &0 :
              break;
            &1,&2,&3 :
              begin
                inc(codes,c);
                inc(len,c);
              end;
            &10,&11,&12 :
              begin
{$ifdef x86_64}
                rex:=rex or (rexbits(oper[c-&10]^.reg) and $F1);
{$endif x86_64}
                inc(codes);
                inc(len);
              end;
            &13,&23 :
              begin
                inc(codes);
                inc(len);
              end;
            &4,&5,&6,&7 :
              begin
                if opsize={$ifdef i8086}S_L{$else}S_W{$endif} then
                  inc(len,2)
                else
                  inc(len);
              end;
            &14,&15,&16,
            &20,&21,&22,
            &24,&25,&26,&27,
            &50,&51,&52 :
              inc(len);
            &30,&31,&32,
            &37,
            &60,&61,&62 :
              inc(len,2);
            &34,&35,&36:
              begin
{$ifdef i8086}
                inc(len,2);
{$else i8086}
                if opsize=S_Q then
                  inc(len,8)
                else
                  inc(len,4);
{$endif i8086}
              end;
            &44,&45,&46:
              inc(len,sizeof(pint));
            &54,&55,&56:
              inc(len,8);
            &40,&41,&42,
            &70,&71,&72,
            &254,&255,&256 :
              inc(len,4);
            &64,&65,&66:
{$ifdef i8086}
              inc(len,2);
{$else i8086}
              inc(len,4);
{$endif i8086}
            &74,&75,&76,&77: ; // ignore vex-coded operand-idx
            &320,&321,&322 :
              begin
                case (oper[c-&320]^.ot and OT_SIZE_MASK) of
{$if defined(i386) or defined(x86_64)}
                  OT_BITS16 :
{$elseif defined(i8086)}
                  OT_BITS32 :
{$endif}
                    inc(len);
{$ifdef x86_64}
                  OT_BITS64:
                    begin
                      rex:=rex or $48;
                    end;
{$endif x86_64}
                end;
              end;
            &310 :
{$if defined(x86_64)}
              { every insentry with code 0310 must be marked with NOX86_64 }
              InternalError(2011051301);
{$elseif defined(i386)}
              inc(len);
{$elseif defined(i8086)}
              {nothing};
{$endif}
            &311 :
{$if defined(x86_64) or defined(i8086)}
              inc(len)
{$endif x86_64 or i8086}
              ;
            &324 :
{$ifndef i8086}
              inc(len)
{$endif not i8086}
              ;
            &326 :
              begin
{$ifdef x86_64}
                rex:=rex or $48;
{$endif x86_64}
              end;
            &312,
            &323,
            &327,
            &331,&332: ;
            &325:
{$ifdef i8086}
                inc(len)
{$endif i8086}
              ;

            &333:
              begin
                inc(len);
                exists_prefix_F2 := true;
              end;
            &334:
              begin
                inc(len);
                exists_prefix_F3 := true;
              end;
            &361:
              begin
{$ifndef i8086}
                inc(len);
                exists_prefix_66 := true;
{$endif not i8086}
              end;
            &335:
{$ifdef x86_64}
              omit_rexw:=true
{$endif x86_64}
              ;
            &336,
            &337: {nothing};

            &100..&227 :
              begin
{$ifdef x86_64}
                 if (c<&177) then
                  begin
                    if (oper[c and 7]^.typ=top_reg) then
                      begin
                        rex:=rex or (rexbits(oper[c and 7]^.reg) and $F4);
                      end;
                  end;

{$endif x86_64}
                if (oper[(c shr 3) and 7]^.typ = top_ref) and
                   (oper[(c shr 3) and 7]^.ref^.offset <> 0) then
                begin
                  if (exists_vex and exists_evex and CheckUseEVEX) or
                     (not(exists_vex) and exists_evex) then
                  begin
                    CheckEVEXTuple(oper[(c shr 3) and 7]^, p, not(exists_l256 or exists_l512), exists_l256, exists_l512, exists_EVEXW1);
                    //const aInput:toper; aInsEntry: pInsentry; aIsVector128, aIsVector256, aIsVector512, aIsEVEXW1: boolean);
                  end;
                end;

                if process_ea(oper[(c shr 3) and 7]^, ea_data, 0, EVEXTupleState = etsNotTuple) then
                 inc(len,ea_data.size)
                  else Message(asmw_e_invalid_effective_address);

{$ifdef x86_64}
                rex:=rex or ea_data.rex;
{$endif x86_64}

              end;
            &350:
              begin
                exists_evex := true;
              end;
            &351: exists_l512 := true; // EVEX length bit 512
            &352: exists_EVEXW1 := true; // EVEX W1
            &362: // VEX prefix for AVX (length = 2 or 3 bytes, dependens on REX.XBW or opcode-prefix ($0F38 or $0F3A))
                  // =>> DEFAULT = 2 Bytes
              begin
                //if not(exists_vex) then
                //begin
                //  inc(len, 2);
                //end;

                exists_vex := true;
              end;
            &363: // REX.W = 1
                  // =>> VEX prefix length = 3
              begin
                if not(exists_vex_extension) then
                begin
                  //inc(len);
                  exists_vex_extension := true;
                end;
              end;
            &364: exists_l256 := true; // VEX length bit 256

            &366, // operand 2 (ymmreg) encoded immediate byte (bit 4-7)
            &367: inc(len); // operand 3 (ymmreg) encoded immediate byte (bit 4-7)
            &370: // VEX-Extension prefix $0F
                  // ignore for calculating length
                  ;
            &371, // VEX-Extension prefix $0F38
            &372: // VEX-Extension prefix $0F3A
              begin
                if not(exists_vex_extension) then
                begin
                  //inc(len);
                  exists_vex_extension := true;
                end;
              end;
            &300,&301,&302:
              begin
{$if defined(x86_64) or defined(i8086)}
                if (oper[c and 3]^.ot and OT_SIZE_MASK)=OT_BITS32 then
                  inc(len);
{$endif x86_64 or i8086}
              end;
            else
             InternalError(200603141);
          end;
        until false;
{$ifdef x86_64}
        if ((rex and $80)<>0) and ((rex and $4F)<>0) then
          Message(asmw_e_bad_reg_with_rex);
        rex:=rex and $4F;      { reset extra bits in upper nibble }
        if omit_rexw then
          begin
            if rex=$48 then    { remove rex entirely? }
              rex:=0
            else
              rex:=rex and $F7;
          end;
        if not(exists_vex or exists_evex) then
        begin
          if rex<>0 then
            Inc(len);
        end;
{$endif}
        if exists_evex and
           exists_vex then
        begin
          if CheckUseEVEX then
          begin
            inc(len, 4);
          end
          else
          begin
            inc(len, 2);
            if exists_vex_extension then inc(len);

            {$ifdef x86_64}
              if not(exists_vex_extension) then
              if rex and $0B <> 0 then inc(len);  // REX.WXB <> 0 =>> needed VEX-Extension
            {$endif x86_64}
          end;

          if exists_prefix_66 then dec(len);
          if exists_prefix_F2 then dec(len);
          if exists_prefix_F3 then dec(len);

        end
        else if exists_evex then
        begin
          inc(len, 4);

          if exists_prefix_66 then dec(len);
          if exists_prefix_F2 then dec(len);
          if exists_prefix_F3 then dec(len);
        end
        else
        begin
          if exists_vex then
          begin
            inc(len,2);

            if exists_prefix_66 then dec(len);
            if exists_prefix_F2 then dec(len);
            if exists_prefix_F3 then dec(len);

            if exists_vex_extension then inc(len);
    {$ifdef x86_64}
            if not(exists_vex_extension) then
              if rex and $0B <> 0 then inc(len);  // REX.WXB <> 0 =>> needed VEX-Extension
    {$endif x86_64}

          end;

        end;
        calcsize:=len;
      end;


    procedure taicpu.write0x66prefix(objdata:TObjData);
      const
        b66: Byte=$66;
      begin
{$ifdef i8086}
        if (objdata.CPUType<>cpu_none) and (objdata.CPUType<cpu_386) then
          Message(asmw_e_instruction_not_supported_by_cpu);
{$endif i8086}
        objdata.writebytes(b66,1);
      end;


    procedure taicpu.write0x67prefix(objdata:TObjData);
      const
        b67: Byte=$67;
      begin
{$ifdef i8086}
        if (objdata.CPUType<>cpu_none) and (objdata.CPUType<cpu_386) then
          Message(asmw_e_instruction_not_supported_by_cpu);
{$endif i8086}
        objdata.writebytes(b67,1);
      end;


        procedure taicpu.gencode(objdata: TObjData);
      {
       * the actual codes (C syntax, i.e. octal):
       * \0            - terminates the code. (Unless it's a literal of course.)
       * \1, \2, \3    - that many literal bytes follow in the code stream
       * \4, \6        - the POP/PUSH (respectively) codes for CS, DS, ES, SS
       *                 (POP is never used for CS) depending on operand 0
       * \5, \7        - the second byte of POP/PUSH codes for FS, GS, depending
       *                 on operand 0
       * \10, \11, \12 - a literal byte follows in the code stream, to be added
       *                 to the register value of operand 0, 1 or 2
       * \13           - a literal byte follows in the code stream, to be added
       *                 to the condition code value of the instruction.
       * \14, \15, \16 - a signed byte immediate operand, from operand 0, 1 or 2
       * \20, \21, \22 - a byte immediate operand, from operand 0, 1 or 2
       * \23           - a literal byte follows in the code stream, to be added
       *                 to the inverted condition code value of the instruction
       *                 (inverted version of \13).
       * \24, \25, \26, \27 - an unsigned byte immediate operand, from operand 0, 1, 2 or 3
       * \30, \31, \32 - a word immediate operand, from operand 0, 1 or 2
       * \34, \35, \36 - select between \3[012] and \4[012] depending on 16/32 bit
       *                 assembly mode or the address-size override on the operand
       * \37           - a word constant, from the _segment_ part of operand 0
       * \40, \41, \42 - a long immediate operand, from operand 0, 1 or 2
       * \44, \45, \46 - select between \3[012], \4[012] or \5[456] depending
                         on the address size of instruction
       * \50, \51, \52 - a byte relative operand, from operand 0, 1 or 2
       * \54, \55, \56 - a qword immediate, from operand 0, 1 or 2
       * \60, \61, \62 - a word relative operand, from operand 0, 1 or 2
       * \64, \65, \66 - select between \6[012] and \7[012] depending on 16/32 bit
       *                 assembly mode or the address-size override on the operand
       * \70, \71, \72 - a long relative operand, from operand 0, 1 or 2
       * \74, \75, \76 - a vex-coded vector operand, from operand 0, 1 or 2
       * \1ab          - a ModRM, calculated on EA in operand a, with the spare
       *                 field the register value of operand b.
       * \2ab          - a ModRM, calculated on EA in operand a, with the spare
       *                 field equal to digit b.
       * \254,\255,\256 - a signed 32-bit immediate to be extended to 64 bits
       * \300,\301,\302 - might be an 0x67, depending on the address size of
       *                 the memory reference in operand x.
       * \310          - indicates fixed 16-bit address size, i.e. optional 0x67.
       * \311          - indicates fixed 32-bit address size, i.e. optional 0x67.
       * \312          - (disassembler only) invalid with non-default address size.
       * \320,\321,\322 - might be an 0x66 or 0x48 byte, depending on the operand
       *                 size of operand x.
       * \324          - indicates fixed 16-bit operand size, i.e. optional 0x66.
       * \325          - indicates fixed 32-bit operand size, i.e. optional 0x66.
       * \326          - indicates fixed 64-bit operand size, i.e. optional 0x48.
       * \327          - indicates that this instruction is only valid when the
       *                 operand size is the default (instruction to disassembler,
       *                 generates no code in the assembler)
       * \331          - instruction not valid with REP prefix.  Hint for
       *                 disassembler only; for SSE instructions.
       * \332	       - disassemble a rep (0xF3 byte) prefix as repe not rep.
       * \333          - 0xF3 prefix for SSE instructions
       * \334          - 0xF2 prefix for SSE instructions
       * \335          - Indicates 64-bit operand size with REX.W not necessary / 64-bit scalar vector operand size
       * \336          - Indicates 32-bit scalar vector operand size
       * \337          - Indicates 64-bit scalar vector operand size

       * \350          - EVEX prefix for AVX instructions
       * \351          - EVEX Vector length 512
       * \352          - EVEX W1

       * \361          - 0x66 prefix for SSE instructions

       * \362          - VEX prefix for AVX instructions
       * \363          - VEX W1
       * \364          - VEX Vector length 256

       * \366          - operand 2 (ymmreg,zmmreg) encoded in bit 4-7 of the immediate byte
       * \367          - operand 3 (ymmreg,zmmreg) encoded in bit 4-7 of the immediate byte

       * \370          - VEX 0F-FLAG
       * \371          - VEX 0F38-FLAG
       * \372          - VEX 0F3A-FLAG


      }

      var
{$ifdef i8086}
        currval : longint;
{$else i8086}
        currval : aint;
{$endif i8086}
        currsym : tobjsymbol;
        currrelreloc,
        currabsreloc,
        currabsreloc32 : TObjRelocationType;
{$ifdef x86_64}
        rexwritten : boolean;
{$endif x86_64}

        procedure getvalsym(opidx:longint);
          begin
            case oper[opidx]^.typ of
              top_ref :
                begin
                  currval:=oper[opidx]^.ref^.offset;
                  currsym:=ObjData.symbolref(oper[opidx]^.ref^.symbol);
{$ifdef i8086}
                  if oper[opidx]^.ref^.refaddr=addr_seg then
                    begin
                      currrelreloc:=RELOC_SEGREL;
                      currabsreloc:=RELOC_SEG;
                      currabsreloc32:=RELOC_SEG;
                    end
                  else if oper[opidx]^.ref^.refaddr=addr_dgroup then
                    begin
                      currrelreloc:=RELOC_DGROUPREL;
                      currabsreloc:=RELOC_DGROUP;
                      currabsreloc32:=RELOC_DGROUP;
                    end
                  else if oper[opidx]^.ref^.refaddr=addr_fardataseg then
                    begin
                      currrelreloc:=RELOC_FARDATASEGREL;
                      currabsreloc:=RELOC_FARDATASEG;
                      currabsreloc32:=RELOC_FARDATASEG;
                    end
                  else
{$endif i8086}
{$ifdef i386}
                  if (oper[opidx]^.ref^.refaddr=addr_pic) and
                     (tf_pic_uses_got in target_info.flags) then
                    begin
                      currrelreloc:=RELOC_PLT32;
                      currabsreloc:=RELOC_GOT32;
                      currabsreloc32:=RELOC_GOT32;
                    end
                  else if oper[opidx]^.ref^.refaddr=addr_ntpoff then
                    begin
                      currrelreloc:=RELOC_NTPOFF;
                      currabsreloc:=RELOC_NTPOFF;
                      currabsreloc32:=RELOC_NTPOFF;
                    end
                  else if oper[opidx]^.ref^.refaddr=addr_tlsgd then
                    begin
                      currrelreloc:=RELOC_TLSGD;
                      currabsreloc:=RELOC_TLSGD;
                      currabsreloc32:=RELOC_TLSGD;
                    end
                  else
{$endif i386}
{$ifdef x86_64}
                  if oper[opidx]^.ref^.refaddr=addr_pic then
                    begin
                      currrelreloc:=RELOC_PLT32;
                      currabsreloc:=RELOC_GOTPCREL;
                      currabsreloc32:=RELOC_GOTPCREL;
                    end
                  else if oper[opidx]^.ref^.refaddr=addr_pic_no_got then
                    begin
                      currrelreloc:=RELOC_RELATIVE;
                      currabsreloc:=RELOC_RELATIVE;
                      currabsreloc32:=RELOC_RELATIVE;
                    end
                  else if oper[opidx]^.ref^.refaddr=addr_tpoff then
                    begin
                      currrelreloc:=RELOC_TPOFF;
                      currabsreloc:=RELOC_TPOFF;
                      currabsreloc32:=RELOC_TPOFF;
                    end
                  else if oper[opidx]^.ref^.refaddr=addr_tlsgd then
                    begin
                      currrelreloc:=RELOC_TLSGD;
                      currabsreloc:=RELOC_TLSGD;
                      currabsreloc32:=RELOC_TLSGD;
                    end
                  else
{$endif x86_64}
                    begin
                      currrelreloc:=RELOC_RELATIVE;
                      currabsreloc:=RELOC_ABSOLUTE;
                      currabsreloc32:=RELOC_ABSOLUTE32;
                    end;
                end;
              top_const :
                begin
{$ifdef i8086}
                  currval:=longint(oper[opidx]^.val);
{$else i8086}
                  currval:=aint(oper[opidx]^.val);
{$endif i8086}
                  currsym:=nil;
                  currabsreloc:=RELOC_ABSOLUTE;
                  currabsreloc32:=RELOC_ABSOLUTE32;
                end;
              else
                Message(asmw_e_immediate_or_reference_expected);
            end;
          end;

{$ifdef x86_64}
       procedure maybewriterex;
       begin
          if (rex<>0) and not(rexwritten) then
            begin
              rexwritten:=true;
              objdata.writebytes(rex,1);
            end;
        end;
{$endif x86_64}

       procedure objdata_writereloc(Data:TRelocDataInt;len:aword;p:TObjSymbol;Reloctype:TObjRelocationType);
         begin
{$ifdef i386}
           { Special case of '_GLOBAL_OFFSET_TABLE_'
             which needs a special relocation type R_386_GOTPC }
           if assigned (p) and
              (p.name='_GLOBAL_OFFSET_TABLE_') and
              (tf_pic_uses_got in target_info.flags) then
             begin
               { nothing else than a 4 byte relocation should occur
                 for GOT }
               if len<>4 then
                 Message1(asmw_e_invalid_opcode_and_operands,GetString);
               Reloctype:=RELOC_GOTPC;
               { We need to add the offset of the relocation
                 of _GLOBAL_OFFSET_TABLE symbol within
                 the current instruction }
               inc(data,objdata.currobjsec.size-insoffset);
             end;
{$endif i386}
           objdata.writereloc(data,len,p,Reloctype);
         end;


      const
        CondVal:array[TAsmCond] of byte=($0,
         $7, $3, $2, $6, $2, $4, $F, $D, $C, $E, $6, $2,
         $3, $7, $3, $5, $E, $C, $D, $F, $1, $B, $9, $5,
         $0, $A, $A, $B, $8, $4);
      var
        i: integer;
        c : byte;
        pb : pbyte;
        codes : pchar;
        bytes : array[0..3] of byte;
        rfield,
        data,s,opidx : longint;
        ea_data : ea;
        relsym : TObjSymbol;

        needed_VEX_Extension: boolean;
        needed_VEX: boolean;
        needed_EVEX: boolean;
        needed_VSIB: boolean;
        opmode: integer;
        VEXvvvv: byte;
        VEXmmmmm: byte;
        VEXw    : byte;
        VEXpp   : byte;
        VEXll   : byte;
        EVEXvvvv: byte;
        EVEXpp: byte;
        EVEXr: byte;
        EVEXx: byte;
        EVEXv: byte;
        EVEXll: byte;
        EVEXw1: byte;
        EVEXz   : byte;
        EVEXaaa : byte;
        EVEXb   : byte;
        EVEXmm  : byte;

      begin
        { safety check }
        if objdata.currobjsec.size<>longword(insoffset) then
          internalerror(200130121);

        { those variables are initialized inside local procedures, the dfa cannot handle this yet }
        currsym:=nil;
        currabsreloc:=RELOC_NONE;
        currabsreloc32:=RELOC_NONE;
        currrelreloc:=RELOC_NONE;
        currval:=0;

        { check instruction's processor level }
        { todo: maybe adapt and enable this code for i386 and x86_64 as well }
{$ifdef i8086}
        if objdata.CPUType<>cpu_none then
          begin
            if IF_8086 in insentry^.flags then
            else if IF_186 in insentry^.flags then
              begin
                if objdata.CPUType<cpu_186 then
                  Message(asmw_e_instruction_not_supported_by_cpu);
              end
            else if IF_286 in insentry^.flags then
              begin
                if objdata.CPUType<cpu_286 then
                  Message(asmw_e_instruction_not_supported_by_cpu);
              end
            else if IF_386 in insentry^.flags then
              begin
                if objdata.CPUType<cpu_386 then
                  Message(asmw_e_instruction_not_supported_by_cpu);
              end
            else if IF_486 in insentry^.flags then
              begin
                if objdata.CPUType<cpu_486 then
                  Message(asmw_e_instruction_not_supported_by_cpu);
              end
            else if IF_PENT in insentry^.flags then
              begin
                if objdata.CPUType<cpu_Pentium then
                  Message(asmw_e_instruction_not_supported_by_cpu);
              end
            else if IF_P6 in insentry^.flags then
              begin
                if objdata.CPUType<cpu_Pentium2 then
                  Message(asmw_e_instruction_not_supported_by_cpu);
              end
            else if IF_KATMAI in insentry^.flags then
              begin
                if objdata.CPUType<cpu_Pentium3 then
                  Message(asmw_e_instruction_not_supported_by_cpu);
              end
            else if insentry^.flags*[IF_WILLAMETTE,IF_PRESCOTT]<>[] then
              begin
                if objdata.CPUType<cpu_Pentium4 then
                  Message(asmw_e_instruction_not_supported_by_cpu);
              end
            else if IF_NEC in insentry^.flags then
              begin
              { the NEC V20/V30 extensions are incompatible with 386+, due to overlapping opcodes }
                if objdata.CPUType>=cpu_386 then
                  Message(asmw_e_instruction_not_supported_by_cpu);
              end
            else if IF_SANDYBRIDGE in insentry^.flags then
              begin
              { todo: handle these properly }
            end;
          end;
{$endif i8086}

        { load data to write }
        codes:=insentry^.code;
{$ifdef x86_64}
        rexwritten:=false;
{$endif x86_64}
        { Force word push/pop for registers }
        if (opsize={$ifdef i8086}S_L{$else}S_W{$endif}) and ((codes[0]=#4) or (codes[0]=#6) or
            ((codes[0]=#1) and ((codes[2]=#5) or (codes[2]=#7)))) then
          write0x66prefix(objdata);

        // needed VEX Prefix (for AVX etc.)

        needed_VEX    := false;
        needed_EVEX   := false;
        needed_VEX_Extension := false;
        needed_VSIB   := false;
        opmode   := -1;
        VEXvvvv  := 0;
        VEXmmmmm := 0;

        VEXll    := 0;
        VEXw     := 0;
        VEXpp    := 0;
        EVEXpp   := 0;
        EVEXvvvv := 0;
        EVEXr    := 0;
        EVEXx    := 0;
        EVEXv    := 0;
        EVEXll   := 0;
        EVEXw1   := 0;
        EVEXz    := 0;
        EVEXaaa  := 0;
        EVEXb    := 0;
        EVEXmm   := 0;

        repeat
          c:=ord(codes^);
          inc(codes);

          case c of
             &0: break;
             &1,
             &2,
             &3: inc(codes,c);
            &10,
            &11,
            &12: inc(codes, 1);
            &74: opmode := 0;
            &75: opmode := 1;
            &76: opmode := 2;
     &100..&227: begin
                   // AVX 512 - EVEX
                   // check operands

                   if (c shr 6) = 1 then
                   begin
                     opidx := c and 7;
                     if ops > opidx then
                     begin
                       if (oper[opidx]^.typ=top_reg) then
                        if getsupreg(oper[opidx]^.reg) and $10 = $0 then EVEXr := 1;
                     end
                   end
                   else EVEXr := 1; // modrm:reg not used =>> 1

                   opidx := (c shr 3) and 7;
                   if ops > opidx then
                    case oper[opidx]^.typ of
                      top_reg: if getsupreg(oper[opidx]^.reg) and $10 = $0 then EVEXx := 1;
                      top_ref: begin
                                 if getsupreg(oper[opidx]^.ref^.index) and $08 = $0 then EVEXx := 1;
                                 if getsubreg(oper[opidx]^.ref^.index) in [R_SUBMMX,R_SUBMMY,R_SUBMMZ] then
                                 begin
                                   // VSIB memory addresing
                                   if getsupreg(oper[opidx]^.ref^.index) and $10 = $0 then EVEXv := 1; // VECTOR-Index
                                   needed_VSIB := true;
                                 end;
                               end;
                      else
                        Internalerror(2019081004);
                    end;


                 end;
           &333: begin
                   VEXvvvv              := VEXvvvv  OR $02; // set SIMD-prefix $F3
                   VEXpp                := $02;             // set SIMD-prefix $F3
                   EVEXpp               := $02;             // set SIMD-prefix $F3
                 end;
           &334: begin
                   VEXvvvv              := VEXvvvv  OR $03; // set SIMD-prefix $F2
                   VEXpp                := $03;             // set SIMD-prefix $F2
                   EVEXpp               := $03;             // set SIMD-prefix $F2
                 end;
           &350: needed_EVEX            := true;            // AVX512 instruction or AVX128/256/512-instruction (depended on operands [x,y,z]mm16..)
           &351: EVEXll                 := $02;             // vectorlength = 512 bits AND no scalar
           &352: EVEXw1                 := $01;
           &361: begin
                   VEXvvvv              := VEXvvvv  OR $01; // set SIMD-prefix $66
                   VEXpp                := $01;             // set SIMD-prefix $66
                   EVEXpp               := $01;             // set SIMD-prefix $66
                 end;
           &362: needed_VEX             := true;
           &363: begin
                   needed_VEX_Extension := true;
                   VEXvvvv              := VEXvvvv  OR (1 shl 7); // set REX.W
                   VEXw                 := 1;
                 end;
           &364: begin
                   VEXvvvv              := VEXvvvv  OR $04; // vectorlength = 256 bits AND no scalar
                   VEXll                := $01;
                   EVEXll               := $01;
                 end;
           &366,
           &367: begin
                   opidx:=c-&364;  { 0366->operand 2, 0367->operand 3 }
                   if (ops > opidx) and
                      (oper[opidx]^.typ=top_reg) and
                      ((oper[opidx]^.ot and OT_REG_EXTRA_MASK = otf_reg_xmm) or
                       (oper[opidx]^.ot and OT_REG_EXTRA_MASK = otf_reg_ymm) or
                       (oper[opidx]^.ot and OT_REG_EXTRA_MASK = otf_reg_zmm)) then
                    if (getsupreg(oper[opidx]^.reg) and $10 = $0) then EVEXx := 1;
                 end;
           &370: begin
                   VEXmmmmm             := VEXmmmmm OR $01; // set leading opcode byte $0F
                   EVEXmm               := $01;
                 end;
           &371: begin
                   needed_VEX_Extension := true;
                   VEXmmmmm             := VEXmmmmm OR $02; // set leading opcode byte $0F38
                   EVEXmm               := $02;
                 end;
           &372: begin
                   needed_VEX_Extension := true;
                   VEXmmmmm             := VEXmmmmm OR $03; // set leading opcode byte $0F3A
                   EVEXmm               := $03;
                 end;

          end;
        until false;

        {$ifndef x86_64}
          EVEXv := 1;
          EVEXx := 1;
          EVEXr := 1;
        {$endif}

        if needed_VEX or needed_EVEX then
        begin
          if (opmode > ops) or
             (opmode < -1) then
          begin
            Internalerror(777100);
          end
          else if opmode = -1 then
          begin
            VEXvvvv  := VEXvvvv or ($0F shl 3); // set VEXvvvv bits (bits 6-3) to 1
            EVEXvvvv := $0F;
            {$ifdef x86_64}
              if not(needed_vsib) then EVEXv    := 1;
            {$endif x86_64}
          end
          else if oper[opmode]^.typ = top_reg then
          begin
            VEXvvvv  := VEXvvvv or ((not(regval(oper[opmode]^.reg)) and $07) shl 3);
            EVEXvvvv := not(regval(oper[opmode]^.reg)) and $07;

            {$ifdef x86_64}
              if rexbits(oper[opmode]^.reg) = 0 then VEXvvvv := VEXvvvv or (1 shl 6);

              if rexbits(oper[opmode]^.reg) = 0 then EVEXvvvv := EVEXvvvv or (1 shl 3);
              if getsupreg(oper[opmode]^.reg) and $10 = 0 then EVEXv := 1;
            {$else}
              VEXvvvv := VEXvvvv or (1 shl 6);
              EVEXvvvv := EVEXvvvv or (1 shl 3);
            {$endif x86_64}
          end
          else Internalerror(777101);

          if not(needed_VEX_Extension) then
          begin
            {$ifdef x86_64}
              if rex and $0B <> 0 then needed_VEX_Extension := true;
            {$endif x86_64}
          end;

          //TG
          if needed_EVEX and needed_VEX then
          begin
            needed_EVEX := false;


            if CheckUseEVEX then
            begin
              // EVEX-Flags r,v,x indicate extended-MMregister
              // Flag = 0 =>> [x,y,z]mm16..[x,y,z]mm31
              // Flag = 1 =>> [x,y,z]mm00..[x,y,z]mm15

              needed_EVEX := true;

              needed_VEX := false;
              needed_VEX_Extension := false;
            end;
          end;

          if needed_EVEX then
          begin
            EVEXaaa:= 0;
            EVEXz  := 0;

            for i := 0 to ops - 1 do
             if (oper[i]^.vopext and OTVE_VECTOR_MASK) <> 0 then
             begin
               if oper[i]^.vopext and OTVE_VECTOR_WRITEMASK = OTVE_VECTOR_WRITEMASK then
               begin
                 EVEXaaa := oper[i]^.vopext and $07;
                 if oper[i]^.vopext and OTVE_VECTOR_ZERO = OTVE_VECTOR_ZERO then EVEXz := 1;
               end;

               if oper[i]^.vopext and OTVE_VECTOR_BCST = OTVE_VECTOR_BCST then
               begin
                 EVEXb := 1;
               end;

               // flag EVEXb is multiple use (broadcast, sae and er)
               if oper[i]^.vopext and OTVE_VECTOR_SAE = OTVE_VECTOR_SAE then
               begin
                 EVEXb := 1;
               end;

               if oper[i]^.vopext and OTVE_VECTOR_ER = OTVE_VECTOR_ER then
               begin
                 EVEXb := 1;

                 case oper[i]^.vopext and OTVE_VECTOR_ER_MASK of
                   OTVE_VECTOR_RNSAE: EVEXll := 0;
                   OTVE_VECTOR_RDSAE: EVEXll := 1;
                   OTVE_VECTOR_RUSAE: EVEXll := 2;
                   OTVE_VECTOR_RZSAE: EVEXll := 3;
                                 else EVEXll := 0;
                 end;
               end;
             end;


            bytes[0] := $62;

            bytes[1] := ((EVEXmm   and $03) shl 0)  or
                      {$ifdef x86_64}
                        ((not(rex) and $05) shl 5)  or
                      {$else}
                        (($05) shl 5)               or
                      {$endif x86_64}
                        ((EVEXr    and $01) shl 4)  or
                        ((EVEXx    and $01) shl 6);

            bytes[2] := ((EVEXpp   and $03) shl 0)  or
                        ((1        and $01) shl 2)  or  // fixed in AVX512
                        ((EVEXvvvv and $0F) shl 3)  or
                        ((EVEXw1   and $01) shl 7);

            bytes[3] := ((EVEXaaa  and $07) shl 0)  or
                        ((EVEXv    and $01) shl 3)  or
                        ((EVEXb    and $01) shl 4)  or
                        ((EVEXll   and $03) shl 5)  or
                        ((EVEXz    and $01) shl 7);

            objdata.writebytes(bytes,4);
          end
          else if needed_VEX_Extension then
          begin
            // VEX-Prefix-Length = 3 Bytes
            {$ifdef x86_64}
              VEXmmmmm := VEXmmmmm or ((not(rex) and $07) shl 5);  // set REX.rxb
              VEXvvvv  := VEXvvvv or ((rex and $08) shl 7);        // set REX.w
            {$else}
              VEXmmmmm := VEXmmmmm or (7 shl 5);  //
            {$endif x86_64}

            bytes[0]:=$C4;
            bytes[1]:=VEXmmmmm;
            bytes[2]:=VEXvvvv;
            objdata.writebytes(bytes,3);
          end
          else
          begin
            // VEX-Prefix-Length = 2 Bytes
            {$ifdef x86_64}
              if rex and $04 = 0 then
            {$endif x86_64}
            begin
              VEXvvvv := VEXvvvv or (1 shl 7);
            end;

            bytes[0]:=$C5;
            bytes[1]:=VEXvvvv;
            objdata.writebytes(bytes,2);
          end;
        end
        else
        begin
          needed_VEX_Extension := false;
          opmode := -1;
        end;

        if not(needed_EVEX) then
        begin
          for opidx := 0 to ops - 1 do
          begin
            if ops > opidx then
             if (oper[opidx]^.typ=top_reg) and
                (getregtype(oper[opidx]^.reg) = R_MMREGISTER) then
              if getsupreg(oper[opidx]^.reg) and $10 = $10 then
              begin
                Message1(asmw_e_invalid_opcode_and_operands,GetString);
                break;
              end;
              //badreg(oper[opidx]^.reg);
          end;
        end;

        { load data to write }
        codes:=insentry^.code;
        repeat
          c:=ord(codes^);
          inc(codes);
          case c of
            &0 :
              break;
            &1,&2,&3 :
              begin
{$ifdef x86_64}
                if not(needed_VEX or needed_EVEX) then  // TG
                  maybewriterex;
{$endif x86_64}
                objdata.writebytes(codes^,c);
                inc(codes,c);
              end;
            &4,&6 :
              begin
                case oper[0]^.reg of
                  NR_CS:
                    bytes[0]:=$e;
                  NR_NO,
                  NR_DS:
                    bytes[0]:=$1e;
                  NR_ES:
                    bytes[0]:=$6;
                  NR_SS:
                    bytes[0]:=$16;
                  else
                    internalerror(777004);
                end;
                if c=&4 then
                  inc(bytes[0]);
                objdata.writebytes(bytes,1);
              end;
            &5,&7 :
              begin
                case oper[0]^.reg of
                  NR_FS:
                    bytes[0]:=$a0;
                  NR_GS:
                    bytes[0]:=$a8;
                  else
                    internalerror(777005);
                end;
                if c=&5 then
                  inc(bytes[0]);
                objdata.writebytes(bytes,1);
              end;
            &10,&11,&12 :
              begin
{$ifdef x86_64}
                if not(needed_VEX or needed_EVEX) then  // TG
                  maybewriterex;
{$endif x86_64}
                bytes[0]:=ord(codes^)+regval(oper[c-&10]^.reg);
                inc(codes);
                objdata.writebytes(bytes,1);
              end;
            &13 :
              begin
                bytes[0]:=ord(codes^)+condval[condition];
                inc(codes);
                objdata.writebytes(bytes,1);
              end;
            &14,&15,&16 :
              begin
                getvalsym(c-&14);
                if (currval<-128) or (currval>127) then
                 Message2(asmw_e_value_exceeds_bounds,'signed byte',tostr(currval));
                if assigned(currsym) then
                  objdata_writereloc(currval,1,currsym,currabsreloc)
                else
                  objdata.writebytes(currval,1);
              end;
            &20,&21,&22 :
              begin
                getvalsym(c-&20);
                if (currval<-256) or (currval>255) then
                 Message2(asmw_e_value_exceeds_bounds,'byte',tostr(currval));
                if assigned(currsym) then
                 objdata_writereloc(currval,1,currsym,currabsreloc)
                else
                 objdata.writebytes(currval,1);
              end;
            &23 :
              begin
                bytes[0]:=ord(codes^)+condval[inverse_cond(condition)];
                inc(codes);
                objdata.writebytes(bytes,1);
              end;
            &24,&25,&26,&27 :
              begin
                getvalsym(c-&24);
                if IF_IMM3 in insentry^.flags then
                  begin
                    if (currval<0) or (currval>7) then
                      Message2(asmw_e_value_exceeds_bounds,'unsigned triad',tostr(currval));
                  end
                else if IF_IMM4 in insentry^.flags then
                  begin
                    if (currval<0) or (currval>15) then
                      Message2(asmw_e_value_exceeds_bounds,'unsigned nibble',tostr(currval));
                  end
                else
                  if (currval<0) or (currval>255) then
                    Message2(asmw_e_value_exceeds_bounds,'unsigned byte',tostr(currval));
                if assigned(currsym) then
                 objdata_writereloc(currval,1,currsym,currabsreloc)
                else
                 objdata.writebytes(currval,1);
              end;
            &30,&31,&32 :     // 030..032
              begin
                getvalsym(c-&30);
{$ifndef i8086}
                { currval is an aint so this cannot happen on i8086 and causes only a warning }
                if (currval<-65536) or (currval>65535) then
                 Message2(asmw_e_value_exceeds_bounds,'word',tostr(currval));
{$endif i8086}
                if assigned(currsym)
{$ifdef i8086}
                   or (currabsreloc in [RELOC_DGROUP,RELOC_FARDATASEG])
{$endif i8086}
                then
                 objdata_writereloc(currval,2,currsym,currabsreloc)
                else
                 objdata.writebytes(currval,2);
              end;
            &34,&35,&36 :     // 034..036
              { !!! These are intended (and used in opcode table) to select depending
                    on address size, *not* operand size. Works by coincidence only. }
              begin
                getvalsym(c-&34);
{$ifdef i8086}
                if assigned(currsym) then
                  objdata_writereloc(currval,2,currsym,currabsreloc)
                else
                  objdata.writebytes(currval,2);
{$else i8086}
                if opsize=S_Q then
                  begin
                    if assigned(currsym) then
                     objdata_writereloc(currval,8,currsym,currabsreloc)
                    else
                     objdata.writebytes(currval,8);
                  end
                else
                  begin
                    if assigned(currsym) then
                      objdata_writereloc(currval,4,currsym,currabsreloc32)
                    else
                      objdata.writebytes(currval,4);
                  end
{$endif i8086}
              end;
            &40,&41,&42 :    // 040..042
              begin
                getvalsym(c-&40);
                if assigned(currsym)
{$ifdef i8086}
                   or (currabsreloc in [RELOC_DGROUP,RELOC_FARDATASEG])
{$endif i8086}
                then
                 objdata_writereloc(currval,4,currsym,currabsreloc32)
                else
                 objdata.writebytes(currval,4);
              end;
            &44,&45,&46 :// 044..046 - select between word/dword/qword depending on
              begin      // address size (we support only default address sizes).
                getvalsym(c-&44);
{$if defined(x86_64)}
                if assigned(currsym) then
                  objdata_writereloc(currval,8,currsym,currabsreloc)
                else
                  objdata.writebytes(currval,8);
{$elseif defined(i386)}
                if assigned(currsym) then
                  objdata_writereloc(currval,4,currsym,currabsreloc32)
                else
                  objdata.writebytes(currval,4);
{$elseif defined(i8086)}
                if assigned(currsym) then
                  objdata_writereloc(currval,2,currsym,currabsreloc)
                else
                  objdata.writebytes(currval,2);
{$endif}
              end;
            &50,&51,&52 :   // 050..052 - byte relative operand
              begin
                getvalsym(c-&50);
                data:=currval-insend;
{$push}
{$r-,q-} { disable also overflow as address returns a qword for x86_64 }
                if assigned(currsym) then
                 inc(data,currsym.address);
{$pop}
                if (data>127) or (data<-128) then
                 Message1(asmw_e_short_jmp_out_of_range,tostr(data));
                objdata.writebytes(data,1);
              end;
            &54,&55,&56:   // 054..056 - qword immediate operand
              begin
                getvalsym(c-&54);
                if assigned(currsym) then
                  objdata_writereloc(currval,8,currsym,currabsreloc)
                else
                  objdata.writebytes(currval,8);
              end;
            &60,&61,&62 :
              begin
                getvalsym(c-&60);
{$ifdef i8086}
                if assigned(currsym) then
                 objdata_writereloc(currval,2,currsym,currrelreloc)
                else
                 objdata_writereloc(currval-insend,2,nil,currabsreloc)
{$else i8086}
                InternalError(777006);
{$endif i8086}
              end;
            &64,&65,&66 :  // 064..066 - select between 16/32 address mode, but we support only 32 (only 16 on i8086)
              begin
                getvalsym(c-&64);
{$ifdef i8086}
                if assigned(currsym) then
                 objdata_writereloc(currval,2,currsym,currrelreloc)
                else
                 objdata_writereloc(currval-insend,2,nil,currabsreloc)
{$else i8086}
                if assigned(currsym) then
                 objdata_writereloc(currval,4,currsym,currrelreloc)
                else
                 objdata_writereloc(currval-insend,4,nil,currabsreloc32)
{$endif i8086}
              end;
            &70,&71,&72 :  // 070..072 - long relative operand
              begin
                getvalsym(c-&70);
                if assigned(currsym) then
                 objdata_writereloc(currval,4,currsym,currrelreloc)
                else
                 objdata_writereloc(currval-insend,4,nil,currabsreloc32)
              end;
            &74,&75,&76 : ; // 074..076 - vex-coded vector operand
                            // ignore
            &254,&255,&256 :  // 0254..0256 - dword implicitly sign-extended to 64-bit (x86_64 only)
              begin
                getvalsym(c-&254);
{$ifdef x86_64}
                { for i386 as aint type is longint the
                  following test is useless }
                if (currval<low(longint)) or (currval>high(longint)) then
                  Message2(asmw_e_value_exceeds_bounds,'signed dword',tostr(currval));
{$endif x86_64}

                if assigned(currsym) then
                  objdata_writereloc(currval,4,currsym,currabsreloc32)
                else
                  objdata.writebytes(currval,4);
              end;
            &300,&301,&302:
              begin
{$if defined(x86_64) or defined(i8086)}
                if (oper[c and 3]^.ot and OT_SIZE_MASK)=OT_BITS32 then
                  write0x67prefix(objdata);
{$endif x86_64 or i8086}
              end;
            &310 :   { fixed 16-bit addr }
{$if defined(x86_64)}
              { every insentry having code 0310 must be marked with NOX86_64 }
              InternalError(2011051302);
{$elseif defined(i386)}
              write0x67prefix(objdata);
{$elseif defined(i8086)}
              {nothing};
{$endif}
            &311 :   { fixed 32-bit addr }
{$if defined(x86_64) or defined(i8086)}
              write0x67prefix(objdata)
{$endif x86_64 or i8086}
              ;
            &320,&321,&322 :
              begin
                case oper[c-&320]^.ot and OT_SIZE_MASK of
{$if defined(i386) or defined(x86_64)}
                  OT_BITS16 :
{$elseif defined(i8086)}
                  OT_BITS32 :
{$endif}
                    write0x66prefix(objdata);
{$ifndef x86_64}
                  OT_BITS64 :
                      Message(asmw_e_64bit_not_supported);
{$endif x86_64}
                end;
              end;
            &323 : {no action needed};
            &325:
{$ifdef i8086}
               write0x66prefix(objdata);
{$else i8086}
              {no action needed};
{$endif i8086}

            &324,
            &361:
              begin
{$ifndef i8086}
                if not(needed_VEX or needed_EVEX) then
                  write0x66prefix(objdata);
{$endif not i8086}
              end;
            &326 :
              begin
{$ifndef x86_64}
                Message(asmw_e_64bit_not_supported);
{$endif x86_64}
              end;
            &333 :
              begin
                if not(needed_VEX or needed_EVEX) then
                begin
                  bytes[0]:=$f3;
                  objdata.writebytes(bytes,1);
                end;
              end;
            &334 :
              begin
                if not(needed_VEX or needed_EVEX) then
                begin
                  bytes[0]:=$f2;
                  objdata.writebytes(bytes,1);
                end;
              end;
            &335:
              ;
            &336: ; // indicates 32-bit scalar vector operand {no action needed}
            &337: ; // indicates 64-bit scalar vector operand {no action needed}
            &312,
            &327,
            &331,&332 :
              begin
                { these are dissambler hints or 32 bit prefixes which
                  are not needed }
              end;
            &362..&364: ; // VEX flags =>> nothing todo
            &366, &367:
              begin
                opidx:=c-&364;  { 0366->operand 2, 0367->operand 3 }
                if (needed_VEX or needed_EVEX) and
                  (ops=4) and
                  (oper[opidx]^.typ=top_reg) and
                  (
                   ((oper[opidx]^.ot and OT_REG_EXTRA_MASK)=otf_reg_xmm) or
                   ((oper[opidx]^.ot and OT_REG_EXTRA_MASK)=otf_reg_ymm) or
                   ((oper[opidx]^.ot and OT_REG_EXTRA_MASK)=otf_reg_zmm)
                  ) then
                  begin
                    bytes[0] := ((getsupreg(oper[opidx]^.reg) and 15) shl 4);
                    objdata.writebytes(bytes,1);
                  end
                else
                  Internalerror(2014032001);
              end;
            &350..&352: ; // EVEX flags =>> nothing todo
            &370..&372: ; // VEX flags =>> nothing todo
            &37:
              begin
{$ifdef i8086}
                if assigned(currsym) then
                  objdata_writereloc(0,2,currsym,RELOC_SEG)
                else
                  InternalError(2015041503);
{$else i8086}
                InternalError(777006);
{$endif i8086}
              end;
            else
              begin
                { rex should be written at this point }
{$ifdef x86_64}
                if not(needed_VEX or needed_EVEX) then  // TG
                  if (rex<>0) and not(rexwritten) then
                    internalerror(200603191);
{$endif x86_64}
                if (c>=&100) and (c<=&227) then  // 0100..0227
                 begin
                   if (c<&177) then            // 0177
                    begin
                      if (oper[c and 7]^.typ=top_reg) then
                        rfield:=regval(oper[c and 7]^.reg)
                      else
                        rfield:=regval(oper[c and 7]^.ref^.base);
                    end
                   else
                    rfield:=c and 7;
                   opidx:=(c shr 3) and 7;

                   if not process_ea(oper[opidx]^,ea_data,rfield, EVEXTupleState = etsNotTuple) then
                    Message(asmw_e_invalid_effective_address);


                   pb:=@bytes[0];
                   pb^:=ea_data.modrm;
                   inc(pb);
                   if ea_data.sib_present then
                    begin
                      pb^:=ea_data.sib;
                      inc(pb);
                    end;

                   s:=pb-@bytes[0];
                   objdata.writebytes(bytes,s);

                   case ea_data.bytes of
                     0 : ;
                     1 :
                       begin
                         if (oper[opidx]^.ot and OT_MEMORY)=OT_MEMORY then
                           begin
                             currsym:=objdata.symbolref(oper[opidx]^.ref^.symbol);
{$ifdef i386}
                             if (oper[opidx]^.ref^.refaddr=addr_pic) and
                                (tf_pic_uses_got in target_info.flags) then
                               currabsreloc:=RELOC_GOT32
                             else
{$endif i386}
{$ifdef x86_64}
                             if oper[opidx]^.ref^.refaddr=addr_pic then
                               currabsreloc:=RELOC_GOTPCREL
                             else
{$endif x86_64}
                               currabsreloc:=RELOC_ABSOLUTE;
                             objdata_writereloc(oper[opidx]^.ref^.offset,1,currsym,currabsreloc);
                           end
                         else
                          begin
                            bytes[0]:=oper[opidx]^.ref^.offset;
                            objdata.writebytes(bytes,1);
                          end;
                         inc(s);
                       end;
                     2,4 :
                       begin
                         currsym:=objdata.symbolref(oper[opidx]^.ref^.symbol);
                         currval:=oper[opidx]^.ref^.offset;
{$ifdef x86_64}
                         if oper[opidx]^.ref^.refaddr=addr_pic then
                           currabsreloc:=RELOC_GOTPCREL
                         else if oper[opidx]^.ref^.refaddr=addr_tlsgd then
                           currabsreloc:=RELOC_TLSGD
                         else if oper[opidx]^.ref^.refaddr=addr_tpoff then
                           currabsreloc:=RELOC_TPOFF
                         else
                           if oper[opidx]^.ref^.base=NR_RIP then
                             begin
                               currabsreloc:=RELOC_RELATIVE;
                               { Adjust reloc value by number of bytes following the displacement,
                                 but not if displacement is specified by literal constant }
                               if Assigned(currsym) then
                                 Dec(currval,InsEnd-objdata.CurrObjSec.Size-ea_data.bytes);
                             end
                           else
{$endif x86_64}
{$ifdef i386}
                         if (oper[opidx]^.ref^.refaddr=addr_pic) and
                            (tf_pic_uses_got in target_info.flags) then
                           currabsreloc:=RELOC_GOT32
                         else if oper[opidx]^.ref^.refaddr=addr_tlsgd then
                           currabsreloc:=RELOC_TLSGD
                         else if oper[opidx]^.ref^.refaddr=addr_ntpoff then
                           currabsreloc:=RELOC_NTPOFF
                         else
{$endif i386}
{$ifdef i8086}
                         if ea_data.bytes=2 then
                           currabsreloc:=RELOC_ABSOLUTE
                         else
{$endif i8086}
                             currabsreloc:=RELOC_ABSOLUTE32;

                           if (currabsreloc in [RELOC_ABSOLUTE32{$ifdef i8086},RELOC_ABSOLUTE{$endif}]) and
                            (Assigned(oper[opidx]^.ref^.relsymbol)) then
                           begin
                             relsym:=objdata.symbolref(oper[opidx]^.ref^.relsymbol);
                             if relsym.objsection=objdata.CurrObjSec then
                               begin
                                 currval:=objdata.CurrObjSec.size+ea_data.bytes-relsym.offset+currval;
{$ifdef i8086}
                                 if ea_data.bytes=4 then
                                   currabsreloc:=RELOC_RELATIVE32
                                 else
{$endif i8086}
                                   currabsreloc:=RELOC_RELATIVE;
                               end
                             else
                               begin
                                 currabsreloc:=RELOC_PIC_PAIR;
                                 currval:=relsym.offset;
                               end;
                           end;
                         objdata_writereloc(currval,ea_data.bytes,currsym,currabsreloc);
                         inc(s,ea_data.bytes);
                       end;
                   end;
                 end
                else
                 InternalError(777007);
              end;
          end;
        until false;
      end;


    function taicpu.is_same_reg_move(regtype: Tregistertype):boolean;
      begin
        result:=(((opcode=A_MOV) or (opcode=A_XCHG)) and
                 (regtype = R_INTREGISTER) and
                 (ops=2) and
                 (oper[0]^.typ=top_reg) and
                 (oper[1]^.typ=top_reg) and
                 (oper[0]^.reg=oper[1]^.reg)
                ) or
                ({ checking the opcodes is a long "or" chain, so check first the registers which is more selective }
                 ((regtype = R_MMREGISTER) and
                  (ops=2) and
                  (oper[0]^.typ=top_reg) and
                  (oper[1]^.typ=top_reg) and
                  (oper[0]^.reg=oper[1]^.reg)) and
                  (
                   (opcode=A_MOVSS) or (opcode=A_MOVSD) or
                   (opcode=A_MOVQ) or (opcode=A_MOVD) or
                   (opcode=A_MOVAPS) or (opcode=A_MOVAPD) or
                   (opcode=A_MOVUPS) or (opcode=A_MOVUPD) or
                   (opcode=A_MOVDQA) or (opcode=A_MOVDQU) or
                   (opcode=A_VMOVSS) or (opcode=A_VMOVSD) or
                   (opcode=A_VMOVQ) or (opcode=A_VMOVD) or
                   (opcode=A_VMOVAPS) or (opcode=A_VMOVAPD) or
                   (opcode=A_VMOVUPS) or (opcode=A_VMOVUPD) or
                   (opcode=A_VMOVDQA) or (opcode=A_VMOVDQU)
                  )
                );
      end;


    procedure build_spilling_operation_type_table;
      var
        opcode : tasmop;
      begin
        new(operation_type_table);
        fillchar(operation_type_table^,sizeof(toperation_type_table),byte(operand_read));
        for opcode:=low(tasmop) to high(tasmop) do
          with InsProp[opcode] do
          begin
              if Ch_Rop1 in Ch then
                    operation_type_table^[opcode,0]:=operand_read;
              if Ch_Wop1 in Ch then
                    operation_type_table^[opcode,0]:=operand_write;
              if [Ch_RWop1,Ch_Mop1]*Ch<>[] then
                    operation_type_table^[opcode,0]:=operand_readwrite;
              if Ch_Rop2 in Ch then
                    operation_type_table^[opcode,1]:=operand_read;
              if Ch_Wop2 in Ch then
                    operation_type_table^[opcode,1]:=operand_write;
              if [Ch_RWop2,Ch_Mop2]*Ch<>[] then
                    operation_type_table^[opcode,1]:=operand_readwrite;
              if Ch_Rop3 in Ch then
                    operation_type_table^[opcode,2]:=operand_read;
              if Ch_Wop3 in Ch then
                    operation_type_table^[opcode,2]:=operand_write;
              if [Ch_RWop3,Ch_Mop3]*Ch<>[] then
                    operation_type_table^[opcode,2]:=operand_readwrite;
              if Ch_Rop4 in Ch then
                operation_type_table^[opcode,3]:=operand_read;
              if Ch_Wop4 in Ch then
                operation_type_table^[opcode,3]:=operand_write;
              if [Ch_RWop4,Ch_Mop4]*Ch<>[] then
                operation_type_table^[opcode,3]:=operand_readwrite;
                end;
              end;


    function taicpu.spilling_get_operation_type(opnr: longint): topertype;
      begin
        { the information in the instruction table is made for the string copy
          operation MOVSD so hack here (FK)

          VMOVSS and VMOVSD has two and three operand flavours, this cannot modelled by x86ins.dat
          so fix it here (FK)
        }
        if ((opcode=A_MOVSD) or (opcode=A_VMOVSS) or (opcode=A_VMOVSD)) and (ops=2) then
          begin
            case opnr of
              0:
                result:=operand_read;
              1:
                result:=operand_write;
              else
                internalerror(200506055);
            end
          end
        { IMUL has 1, 2 and 3-operand forms }
        else if opcode=A_IMUL then
          begin
            case ops of
              1:
                if opnr=0 then
                  result:=operand_read
                else
                  internalerror(2014011802);
              2:
                begin
                  case opnr of
                    0:
                      result:=operand_read;
                    1:
                      result:=operand_readwrite;
                    else
                      internalerror(2014011803);
                  end;
                end;
              3:
                begin
                  case opnr of
                    0,1:
                      result:=operand_read;
                    2:
                      result:=operand_write;
                    else
                      internalerror(2014011804);
                  end;
                end;
              else
                internalerror(2014011805);
            end;
          end
        else
          result:=operation_type_table^[opcode,opnr];
      end;


    function spilling_create_load(const ref:treference;r:tregister):Taicpu;
      var
        tmpref: treference;
      begin
        tmpref:=ref;
{$ifdef i8086}
        if tmpref.segment=NR_SS then
          tmpref.segment:=NR_NO;
{$endif i8086}
        case getregtype(r) of
          R_INTREGISTER :
            begin
              if getsubreg(r)=R_SUBH then
                inc(tmpref.offset);
              { we don't need special code here for 32 bit loads on x86_64, since
                those will automatically zero-extend the upper 32 bits. }
              result:=taicpu.op_ref_reg(A_MOV,reg2opsize(r),tmpref,r);
            end;
          R_MMREGISTER :
            if current_settings.fputype in fpu_avx_instructionsets then
              case getsubreg(r) of
                R_SUBMMD:
                  result:=taicpu.op_ref_reg(A_VMOVSD,S_NO,tmpref,r);
                R_SUBMMS:
                  result:=taicpu.op_ref_reg(A_VMOVSS,S_NO,tmpref,r);
                R_SUBQ,
                R_SUBMMWHOLE:
                  result:=taicpu.op_ref_reg(A_VMOVQ,S_NO,tmpref,r);
                else
                  internalerror(200506043);
              end
            else
              case getsubreg(r) of
                R_SUBMMD:
                  result:=taicpu.op_ref_reg(A_MOVSD,S_NO,tmpref,r);
                R_SUBMMS:
                  result:=taicpu.op_ref_reg(A_MOVSS,S_NO,tmpref,r);
                R_SUBQ,
                R_SUBMMWHOLE:
                  result:=taicpu.op_ref_reg(A_MOVQ,S_NO,tmpref,r);
                else
                  internalerror(200506043);
              end;
          else
            internalerror(200401041);
        end;
      end;


    function spilling_create_store(r:tregister; const ref:treference):Taicpu;
      var
        size: topsize;
        tmpref: treference;
      begin
        tmpref:=ref;
{$ifdef i8086}
        if tmpref.segment=NR_SS then
          tmpref.segment:=NR_NO;
{$endif i8086}
        case getregtype(r) of
          R_INTREGISTER :
            begin
              if getsubreg(r)=R_SUBH then
                inc(tmpref.offset);
              size:=reg2opsize(r);
{$ifdef x86_64}
              { even if it's a 32 bit reg, we still have to spill 64 bits
                because we often perform 64 bit operations on them }
              if (size=S_L) then
                begin
                  size:=S_Q;
                  r:=newreg(getregtype(r),getsupreg(r),R_SUBWHOLE);
                end;
{$endif x86_64}
              result:=taicpu.op_reg_ref(A_MOV,size,r,tmpref);
            end;
          R_MMREGISTER :
            if current_settings.fputype in fpu_avx_instructionsets then
              case getsubreg(r) of
                R_SUBMMD:
                  result:=taicpu.op_reg_ref(A_VMOVSD,S_NO,r,tmpref);
                R_SUBMMS:
                  result:=taicpu.op_reg_ref(A_VMOVSS,S_NO,r,tmpref);
                R_SUBQ,
                R_SUBMMWHOLE:
                  result:=taicpu.op_reg_ref(A_VMOVQ,S_NO,r,tmpref);
                else
                  internalerror(200506042);
              end
            else
              case getsubreg(r) of
                R_SUBMMD:
                  result:=taicpu.op_reg_ref(A_MOVSD,S_NO,r,tmpref);
                R_SUBMMS:
                  result:=taicpu.op_reg_ref(A_MOVSS,S_NO,r,tmpref);
                R_SUBQ,
                R_SUBMMWHOLE:
                  result:=taicpu.op_reg_ref(A_MOVQ,S_NO,r,tmpref);
                else
                  internalerror(200506042);
              end;
          else
            internalerror(200401041);
        end;
      end;


{$ifdef i8086}
    procedure taicpu.loadsegsymbol(opidx:longint;s:tasmsymbol);
      var
        r: treference;
      begin
        reference_reset_symbol(r,s,0,1,[]);
        r.refaddr:=addr_seg;
        loadref(opidx,r);
      end;
{$endif i8086}

{*****************************************************************************
                              Instruction table
*****************************************************************************}

    procedure BuildInsTabCache;
      var
        i : longint;
      begin
        new(instabcache);
        FillChar(instabcache^,sizeof(tinstabcache),$ff);
        i:=0;
        while (i<InsTabEntries) do
         begin
           if InsTabCache^[InsTab[i].OPcode]=-1 then
            InsTabCache^[InsTab[i].OPcode]:=i;
           inc(i);
         end;
      end;


    procedure BuildInsTabMemRefSizeInfoCache;
    var
      AsmOp: TasmOp;
      i,j: longint;
      insentry  : PInsEntry;

      MRefInfo: TMemRefSizeInfo;
      SConstInfo: TConstSizeInfo;
      actRegSize: int64;
      actMemSize: int64;
      actConstSize: int64;
      actRegCount: integer;
      actMemCount: integer;
      actConstCount: integer;
      actRegTypes  : int64;
      actRegMemTypes: int64;
      NewRegSize: int64;

      actVMemCount  : integer;
      actVMemTypes  : int64;

      RegMMXSizeMask: int64;
      RegXMMSizeMask: int64;
      RegYMMSizeMask: int64;
      RegZMMSizeMask: int64;

      RegMMXConstSizeMask: int64;
      RegXMMConstSizeMask: int64;
      RegYMMConstSizeMask: int64;
      RegZMMConstSizeMask: int64;

      RegBCSTSizeMask: int64;
      RegBCSTXMMSizeMask: int64;
      RegBCSTYMMSizeMask: int64;
      RegBCSTZMMSizeMask: int64;
      ExistsMemRef      : boolean;

      bitcount          : integer;
      ExistsCode336     : boolean;
      ExistsCode337     : boolean;
      ExistsSSEAVXReg   : boolean;

      function bitcnt(aValue: int64): integer;
      var
        i: integer;
      begin
        result := 0;

        for i := 0 to 63 do
        begin
          if (aValue mod 2) = 1 then
          begin
            inc(result);
          end;

          aValue := aValue shr 1;
        end;
      end;

    begin
      new(InsTabMemRefSizeInfoCache);
      FillChar(InsTabMemRefSizeInfoCache^,sizeof(TInsTabMemRefSizeInfoCache),0);

      for AsmOp := low(TAsmOp) to high(TAsmOp) do
      begin
        i := InsTabCache^[AsmOp];

        if i >= 0 then
        begin
          InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize           := msiUnknown;
          InsTabMemRefSizeInfoCache^[AsmOp].MemRefSizeBCST       := msbUnknown;
          InsTabMemRefSizeInfoCache^[AsmOp].BCSTXMMMultiplicator := 0;
          InsTabMemRefSizeInfoCache^[AsmOp].ConstSize            := csiUnknown;
          InsTabMemRefSizeInfoCache^[AsmOp].ExistsSSEAVX         := false;
          InsTabMemRefSizeInfoCache^[AsmOp].BCSTTypes            := [];

          insentry:=@instab[i];
          RegMMXSizeMask := 0;
          RegXMMSizeMask := 0;
          RegYMMSizeMask := 0;
          RegZMMSizeMask := 0;

          RegMMXConstSizeMask := 0;
          RegXMMConstSizeMask := 0;
          RegYMMConstSizeMask := 0;
          RegZMMConstSizeMask := 0;

          RegBCSTSizeMask:= 0;
          RegBCSTXMMSizeMask := 0;
          RegBCSTYMMSizeMask := 0;
          RegBCSTZMMSizeMask := 0;
          ExistsMemRef       := false;

          while (insentry^.opcode=AsmOp) do
          begin
            MRefInfo         := msiUnknown;

            actRegSize       := 0;
            actRegCount      := 0;
            actRegTypes      := 0;
            NewRegSize       := 0;

            actMemSize       := 0;
            actMemCount      := 0;
            actRegMemTypes   := 0;

            actVMemCount     := 0;
            actVMemTypes     := 0;

            actConstSize     := 0;
            actConstCount    := 0;

            ExistsCode336   := false; // indicate fixed operand size 32 bit
            ExistsCode337   := false; // indicate fixed operand size 64 bit
            ExistsSSEAVXReg := false;

            // parse insentry^.code for &336 and &337
            // &336 (octal) = 222 (decimal) == fixed operand size 32 bit
            // &337 (octal) = 223 (decimal) == fixed operand size 64 bit
            for i := low(insentry^.code) to high(insentry^.code) do
            begin
              case insentry^.code[i] of
                #222: ExistsCode336 := true;
                #223: ExistsCode337 := true;
                #0,#1,#2,#3: break;
              end;
            end;

            for i := 0 to insentry^.ops -1 do
            begin
              if (insentry^.optypes[i] and OT_REGISTER) = OT_REGISTER then
               case insentry^.optypes[i] and (OT_XMMREG or OT_YMMREG or OT_ZMMREG or OT_KREG or OT_REG_EXTRA_MASK) of
                  OT_XMMREG,
                  OT_YMMREG,
                  OT_ZMMREG: ExistsSSEAVXReg := true;
                        else;
               end;
            end;


            for j := 0 to insentry^.ops -1 do
            begin
              if ((insentry^.optypes[j] and OT_XMEM32) = OT_XMEM32) OR
                 ((insentry^.optypes[j] and OT_XMEM64) = OT_XMEM64) OR
                 ((insentry^.optypes[j] and OT_YMEM32) = OT_YMEM32) OR
                 ((insentry^.optypes[j] and OT_YMEM64) = OT_YMEM64) OR
                 ((insentry^.optypes[j] and OT_ZMEM32) = OT_ZMEM32) OR
                 ((insentry^.optypes[j] and OT_ZMEM64) = OT_ZMEM64) then
              begin
                inc(actVMemCount);

                case insentry^.optypes[j] and (OT_XMEM32 OR OT_XMEM64 OR OT_YMEM32 OR OT_YMEM64 OR OT_ZMEM32 OR OT_ZMEM64) of
                  OT_XMEM32: actVMemTypes := actVMemTypes or OT_XMEM32;
                  OT_XMEM64: actVMemTypes := actVMemTypes or OT_XMEM64;
                  OT_YMEM32: actVMemTypes := actVMemTypes or OT_YMEM32;
                  OT_YMEM64: actVMemTypes := actVMemTypes or OT_YMEM64;
                  OT_ZMEM32: actVMemTypes := actVMemTypes or OT_ZMEM32;
                  OT_ZMEM64: actVMemTypes := actVMemTypes or OT_ZMEM64;
                        else InternalError(777206);
                end;
              end
              else if (insentry^.optypes[j] and OT_REGISTER) = OT_REGISTER then
              begin
                inc(actRegCount);

                  NewRegSize := (insentry^.optypes[j] and OT_SIZE_MASK);
                  if NewRegSize = 0 then
                    begin
                      case insentry^.optypes[j] and (OT_MMXREG or OT_XMMREG or OT_YMMREG or OT_ZMMREG or OT_KREG or OT_REG_EXTRA_MASK) of
                        OT_MMXREG: begin
                                     NewRegSize := OT_BITS64;
                                   end;
                        OT_XMMREG: begin
                                     NewRegSize := OT_BITS128;
                                     InsTabMemRefSizeInfoCache^[AsmOp].ExistsSSEAVX := true;
                                   end;
                        OT_YMMREG: begin
                                     NewRegSize := OT_BITS256;
                                     InsTabMemRefSizeInfoCache^[AsmOp].ExistsSSEAVX := true;
                                   end;
                        OT_ZMMREG: begin
                                     NewRegSize := OT_BITS512;
                                     InsTabMemRefSizeInfoCache^[AsmOp].ExistsSSEAVX := true;
                                   end;
                          OT_KREG: begin
                                     InsTabMemRefSizeInfoCache^[AsmOp].ExistsSSEAVX := true;
                                   end;

                              else NewRegSize := not(0);
                      end;
                  end;

                actRegSize  := actRegSize or NewRegSize;
                actRegTypes := actRegTypes or (insentry^.optypes[j] and (OT_MMXREG or OT_XMMREG or OT_YMMREG or OT_ZMMREG or OT_KREG or OT_REG_EXTRA_MASK));
                end
              else if ((insentry^.optypes[j] and OT_MEMORY) <> 0) then
                begin
                  inc(actMemCount);


                  if ExistsSSEAVXReg and ExistsCode336 then
                    actMemSize := actMemSize or OT_BITS32
                  else if ExistsSSEAVXReg and ExistsCode337 then
                    actMemSize := actMemSize or OT_BITS64
                  else
                    actMemSize:=actMemSize or (insentry^.optypes[j] and (OT_SIZE_MASK OR OT_VECTORBCST));

                  if (insentry^.optypes[j] and OT_REGMEM) = OT_REGMEM then
                    begin
                      actRegMemTypes  := actRegMemTypes or insentry^.optypes[j];
                    end;
                end
              else if ((insentry^.optypes[j] and OT_IMMEDIATE) = OT_IMMEDIATE) then
              begin
                inc(actConstCount);

                actConstSize    := actConstSize or (insentry^.optypes[j] and OT_SIZE_MASK);
              end
            end;

            if actConstCount > 0 then
            begin
              case actConstSize of
                0: SConstInfo := csiNoSize;
                OT_BITS8: SConstInfo := csiMem8;
                OT_BITS16: SConstInfo := csiMem16;
                OT_BITS32: SConstInfo := csiMem32;
                OT_BITS64: SConstInfo := csiMem64;
                else SConstInfo := csiMultiple;
              end;

              if InsTabMemRefSizeInfoCache^[AsmOp].ConstSize = csiUnknown then
              begin
                InsTabMemRefSizeInfoCache^[AsmOp].ConstSize := SConstInfo;
              end
              else if InsTabMemRefSizeInfoCache^[AsmOp].ConstSize <> SConstInfo then
              begin
                InsTabMemRefSizeInfoCache^[AsmOp].ConstSize := csiMultiple;
              end;
            end;

            if actVMemCount > 0 then
            begin
              if actVMemCount = 1 then
              begin
                if actVMemTypes > 0 then
                begin
                  case actVMemTypes of
                    OT_XMEM32: MRefInfo := msiXMem32;
                    OT_XMEM64: MRefInfo := msiXMem64;
                    OT_YMEM32: MRefInfo := msiYMem32;
                    OT_YMEM64: MRefInfo := msiYMem64;
                    OT_ZMEM32: MRefInfo := msiZMem32;
                    OT_ZMEM64: MRefInfo := msiZMem64;
                          else InternalError(777208);
                  end;

                  case actRegTypes of
                    OT_XMMREG: case MRefInfo of
                                 msiXMem32,
                                 msiXMem64: RegXMMSizeMask := RegXMMSizeMask or OT_BITS128;
                                 msiYMem32,
                                 msiYMem64: RegXMMSizeMask := RegXMMSizeMask or OT_BITS256;
                                 msiZMem32,
                                 msiZMem64: RegXMMSizeMask := RegXMMSizeMask or OT_BITS512;
                                       else InternalError(777210);
                               end;
                    OT_YMMREG: case MRefInfo of
                                 msiXMem32,
                                 msiXMem64: RegYMMSizeMask := RegYMMSizeMask or OT_BITS128;
                                 msiYMem32,
                                 msiYMem64: RegYMMSizeMask := RegYMMSizeMask or OT_BITS256;
                                 msiZMem32,
                                 msiZMem64: RegYMMSizeMask := RegYMMSizeMask or OT_BITS512;
                                       else InternalError(777211);
                               end;
                    OT_ZMMREG: case MRefInfo of
                                 msiXMem32,
                                 msiXMem64: RegZMMSizeMask := RegZMMSizeMask or OT_BITS128;
                                 msiYMem32,
                                 msiYMem64: RegZMMSizeMask := RegZMMSizeMask or OT_BITS256;
                                 msiZMem32,
                                 msiZMem64: RegZMMSizeMask := RegZMMSizeMask or OT_BITS512;
                                       else InternalError(777211);
                               end;

                          //else InternalError(777209);
                  end;


                  if InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize = msiUnknown then
                  begin
                    InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := MRefInfo;
                  end
                  else if InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize <> MRefInfo then
                  begin
                    if InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize in [msiXMem32, msiXMem64, msiYMem32, msiYMem64, msiZMem32, msiZMem64] then
                    begin
                      InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiVMemMultiple;
                    end
                    else if InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize <> msiVMemMultiple then InternalError(777212);
                  end;

                end;
              end
              else InternalError(777207);
            end
            else
              begin
                if (actMemCount=2) and ((AsmOp=A_MOVS) or (AsmOp=A_CMPS)) then actMemCount:=1;

                ExistsMemRef := ExistsMemRef or (actMemCount > 0);

                case actMemCount of
                  0: ; // nothing todo
                  1: begin
                       MRefInfo := msiUnknown;

                       if not(ExistsCode336 or ExistsCode337) then
                         case actRegMemTypes and (OT_MMXRM or OT_XMMRM or OT_YMMRM or OT_ZMMRM or OT_REG_EXTRA_MASK) of
                           OT_MMXRM: actMemSize := actMemSize or OT_BITS64;
                           OT_XMMRM: actMemSize := actMemSize or OT_BITS128;
                           OT_YMMRM: actMemSize := actMemSize or OT_BITS256;
                           OT_ZMMRM: actMemSize := actMemSize or OT_BITS512;
                         end;

                       case actMemSize of
                                  0: MRefInfo := msiNoSize;
                           OT_BITS8: MRefInfo := msiMem8;
                          OT_BITS16: MRefInfo := msiMem16;
                          OT_BITS32: MRefInfo := msiMem32;
                         OT_BITSB32: MRefInfo := msiBMem32;
                          OT_BITS64: MRefInfo := msiMem64;
                         OT_BITSB64: MRefInfo := msiBMem64;
                         OT_BITS128: MRefInfo := msiMem128;
                         OT_BITS256: MRefInfo := msiMem256;
                         OT_BITS512: MRefInfo := msiMem512;
                         OT_BITS80,
                         OT_FAR,
                         OT_NEAR,
                         OT_SHORT: ; // ignore
                         else
                           begin
                             bitcount := bitcnt(actMemSize);

                             if bitcount > 1 then MRefInfo := msiMultiple
                             else InternalError(777203);
                           end;
                       end;

                       if InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize = msiUnknown then
                         begin
                           InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := MRefInfo;
                         end
                       else
                       begin
                         // ignore broadcast-memory
                         if not(MRefInfo in [msiBMem32, msiBMem64]) then
                         begin
                           if InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize <> MRefInfo then
                           begin
                             with InsTabMemRefSizeInfoCache^[AsmOp] do
                             begin
                               if ((MemRefSize in [msiMem8, msiMULTIPLEMinSize8]) OR (MRefInfo = msiMem8))   then MemRefSize := msiMultipleMinSize8
                               else if ((MemRefSize in [ msiMem16,  msiMULTIPLEMinSize16])  OR (MRefInfo =  msiMem16)) then MemRefSize := msiMultipleMinSize16
                               else if ((MemRefSize in [ msiMem32,  msiMULTIPLEMinSize32])  OR (MRefInfo =  msiMem32)) then MemRefSize := msiMultipleMinSize32
                               else if ((MemRefSize in [ msiMem64,  msiMULTIPLEMinSize64])  OR (MRefInfo =  msiMem64)) then MemRefSize := msiMultipleMinSize64
                               else if ((MemRefSize in [msiMem128, msiMULTIPLEMinSize128])  OR (MRefInfo = msiMem128)) then MemRefSize := msiMultipleMinSize128
                               else if ((MemRefSize in [msiMem256, msiMULTIPLEMinSize256])  OR (MRefInfo = msiMem256)) then MemRefSize := msiMultipleMinSize256
                               else if ((MemRefSize in [msiMem512, msiMULTIPLEMinSize512])  OR (MRefInfo = msiMem512)) then MemRefSize := msiMultipleMinSize512
                               else MemRefSize := msiMultiple;
                             end;
                           end;
                         end;
                       end;

                       //if not(MRefInfo in [msiBMem32, msiBMem64]) and (actRegCount > 0) then
                       if actRegCount > 0 then
                       begin
                         if MRefInfo in [msiBMem32, msiBMem64] then
                         begin
                           if IF_BCST2  in insentry^.flags then InsTabMemRefSizeInfoCache^[AsmOp].BCSTTypes := InsTabMemRefSizeInfoCache^[AsmOp].BCSTTypes + [bt1to2];
                           if IF_BCST4  in insentry^.flags then InsTabMemRefSizeInfoCache^[AsmOp].BCSTTypes := InsTabMemRefSizeInfoCache^[AsmOp].BCSTTypes + [bt1to4];
                           if IF_BCST8  in insentry^.flags then InsTabMemRefSizeInfoCache^[AsmOp].BCSTTypes := InsTabMemRefSizeInfoCache^[AsmOp].BCSTTypes + [bt1to8];
                           if IF_BCST16 in insentry^.flags then InsTabMemRefSizeInfoCache^[AsmOp].BCSTTypes := InsTabMemRefSizeInfoCache^[AsmOp].BCSTTypes + [bt1to16];

                           //InsTabMemRefSizeInfoCache^[AsmOp].BCSTTypes

                           // BROADCAST - OPERAND
                           RegBCSTSizeMask := RegBCSTSizeMask or actMemSize;

                           case actRegTypes and (OT_XMMREG or OT_YMMREG or OT_ZMMREG or OT_REG_EXTRA_MASK) of
                             OT_XMMREG: RegBCSTXMMSizeMask := RegBCSTXMMSizeMask or actMemSize;
                             OT_YMMREG: RegBCSTYMMSizeMask := RegBCSTYMMSizeMask or actMemSize;
                             OT_ZMMREG: RegBCSTZMMSizeMask := RegBCSTZMMSizeMask or actMemSize;
                                   else begin

                                          RegBCSTXMMSizeMask := not(0);
                                          RegBCSTYMMSizeMask := not(0);
                                          RegBCSTZMMSizeMask := not(0);
                                        end;
                           end;
                         end
                         else
                         case actRegTypes and (OT_MMXREG or OT_XMMREG or OT_YMMREG or OT_ZMMREG or OT_REG_EXTRA_MASK) of
                           OT_MMXREG: if actConstCount > 0 then RegMMXConstSizeMask := RegMMXConstSizeMask or actMemSize
                                       else RegMMXSizeMask := RegMMXSizeMask or actMemSize;
                           OT_XMMREG: if actConstCount > 0 then RegXMMConstSizeMask := RegXMMConstSizeMask or actMemSize
                                       else RegXMMSizeMask := RegXMMSizeMask or actMemSize;
                           OT_YMMREG: if actConstCount > 0 then RegYMMConstSizeMask := RegYMMConstSizeMask or actMemSize
                                       else RegYMMSizeMask := RegYMMSizeMask or actMemSize;
                           OT_ZMMREG: if actConstCount > 0 then RegZMMConstSizeMask := RegZMMConstSizeMask or actMemSize
                                       else RegZMMSizeMask := RegZMMSizeMask or actMemSize;
                                 else begin
                                        RegMMXSizeMask := not(0);
                                        RegXMMSizeMask := not(0);
                                        RegYMMSizeMask := not(0);
                                        RegZMMSizeMask := not(0);

                                        RegMMXConstSizeMask := not(0);
                                        RegXMMConstSizeMask := not(0);
                                        RegYMMConstSizeMask := not(0);
                                        RegZMMConstSizeMask := not(0);
                                      end;
                         end;
                       end
                       else


                     end
                  else InternalError(777202);
                end;
              end;

            inc(insentry);
          end;
          if InsTabMemRefSizeInfoCache^[AsmOp].ExistsSSEAVX then
          begin
            case RegBCSTSizeMask of
                      0: ; // ignore;
              OT_BITSB32: begin
                            InsTabMemRefSizeInfoCache^[AsmOp].MemRefSizeBCST       := msbBCST32;
                            InsTabMemRefSizeInfoCache^[AsmOp].BCSTXMMMultiplicator := 4;
                          end;
              OT_BITSB64: begin
                            InsTabMemRefSizeInfoCache^[AsmOp].MemRefSizeBCST       := msbBCST64;
                            InsTabMemRefSizeInfoCache^[AsmOp].BCSTXMMMultiplicator := 2;
                          end;
                    else begin
                           InsTabMemRefSizeInfoCache^[AsmOp].MemRefSizeBCST := msbMultiple;
                         end;;
            end;
          end;


          if (InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize in MemRefMultiples) and
             (InsTabMemRefSizeInfoCache^[AsmOp].ExistsSSEAVX)then
          begin
            if InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize = msiVMemMultiple then
            begin
              if ((RegXMMSizeMask = OT_BITS128) or (RegXMMSizeMask = 0))     and
                 ((RegYMMSizeMask = OT_BITS256) or (RegYMMSizeMask = 0))     and
                 ((RegZMMSizeMask = OT_BITS512) or (RegZMMSizeMask = 0))     and
                 ((RegXMMSizeMask or RegYMMSizeMask or RegZMMSizeMask) <> 0) then
              begin
                InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiVMemRegSize;
              end;
            end
            else if (RegMMXSizeMask or RegMMXConstSizeMask) <> 0 then
            begin
              if ((RegMMXSizeMask or RegMMXConstSizeMask) = OT_BITS64)  and
                 ((RegXMMSizeMask or RegXMMConstSizeMask) = OT_BITS128) and
                 ((RegYMMSizeMask or RegYMMConstSizeMask) = 0)          and
                 ((RegZMMSizeMask or RegZMMConstSizeMask) = 0) then
              begin
                InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegSize;
              end;
            end
            else if (((RegXMMSizeMask or RegXMMConstSizeMask) = OT_BITS128) or ((RegXMMSizeMask or RegXMMConstSizeMask) = 0)) and
                    (((RegYMMSizeMask or RegYMMConstSizeMask) = OT_BITS256) or ((RegYMMSizeMask or RegYMMConstSizeMask) = 0)) and
                    (((RegZMMSizeMask or RegZMMConstSizeMask) = OT_BITS512) or ((RegZMMSizeMask or RegZMMConstSizeMask) = 0)) and
                    (((RegXMMSizeMask or RegXMMConstSizeMask or
                       RegYMMSizeMask or RegYMMConstSizeMask or
                       RegZMMSizeMask or RegZMMConstSizeMask)) <> 0) then
            begin
              InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegSize;
            end
            else if (RegXMMSizeMask or RegXMMConstSizeMask = OT_BITS16) and
                    (RegYMMSizeMask or RegYMMConstSizeMask = OT_BITS32) and
                    (RegZMMSizeMask or RegZMMConstSizeMask = 0) then
            begin
              InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegx16y32;
            end
            else if (RegXMMSizeMask or RegXMMConstSizeMask = OT_BITS16) and
                    (RegYMMSizeMask or RegYMMConstSizeMask = OT_BITS32) and
                    (RegZMMSizeMask or RegZMMConstSizeMask = OT_BITS64) then
            begin
              InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegx16y32z64;
            end
            else if ((RegXMMSizeMask or RegXMMConstSizeMask) = OT_BITS32) and
                    ((RegYMMSizeMask or RegYMMConstSizeMask) = OT_BITS64) then
            begin
              if ((RegZMMSizeMask or RegZMMConstSizeMask) = 0) then
              begin
                InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegx32y64;
              end
              else if ((RegZMMSizeMask or RegZMMConstSizeMask) = OT_BITS128) then
              begin
                InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegx32y64z128;
              end;
            end
            else if ((RegXMMSizeMask or RegXMMConstSizeMask) = OT_BITS64)  and
                    ((RegYMMSizeMask or RegYMMConstSizeMask) = OT_BITS128) and
                    ((RegZMMSizeMask or RegZMMConstSizeMask) = 0) then
            begin
              InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegx64y128;
            end
            else if ((RegXMMSizeMask or RegXMMConstSizeMask) = OT_BITS64)  and
                    ((RegYMMSizeMask or RegYMMConstSizeMask) = OT_BITS128) and
                    ((RegZMMSizeMask or RegZMMConstSizeMask) = OT_BITS256) then
            begin
              InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegx64y128z256;
            end
            else if ((RegXMMSizeMask or RegXMMConstSizeMask) = OT_BITS64)  and
                    ((RegYMMSizeMask or RegYMMConstSizeMask) = OT_BITS256) and
                    ((RegZMMSizeMask or RegZMMConstSizeMask) = 0) then
            begin
              InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegx64y256;
            end
            else if ((RegXMMSizeMask or RegXMMConstSizeMask) = OT_BITS64)  and
                    ((RegYMMSizeMask or RegYMMConstSizeMask) = OT_BITS256) and
                    ((RegZMMSizeMask or RegZMMConstSizeMask) = OT_BITS512) then
            begin
              InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegx64y256z512;
            end
            else if ((RegXMMConstSizeMask = 0) or (RegXMMConstSizeMask = OT_BITS128))     and
                    ((RegYMMConstSizeMask = 0) or (RegYMMConstSizeMask = OT_BITS256))     and
                    ((RegZMMConstSizeMask = 0) or (RegZMMConstSizeMask = OT_BITS512))     and
                    ((RegXMMConstSizeMask or RegYMMConstSizeMask or RegZMMConstSizeMask) <> 0) and
                    (
                     ((RegXMMSizeMask or RegYMMSizeMask or RegZMMSizeMask) = OT_BITS128) or
                     ((RegXMMSizeMask or RegYMMSizeMask or RegZMMSizeMask) = OT_BITS256) or
                     ((RegXMMSizeMask or RegYMMSizeMask or RegZMMSizeMask) = OT_BITS512)
                    ) then
            begin
              case RegXMMSizeMask or RegYMMSizeMask or RegZMMSizeMask of
                OT_BITS128: InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegConst128;
                OT_BITS256: InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegConst256;
                OT_BITS512: InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMemRegConst512;
                       else InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiMultiple;
              end;
            end
            else
            begin
              if not(
                     (AsmOp = A_CVTSI2SS) or
                     (AsmOp = A_CVTSI2SD) or
                     (AsmOp = A_CVTPD2DQ) or
                     (AsmOp = A_VCVTPD2DQ) or
                     (AsmOp = A_VCVTPD2PS) or
                     (AsmOp = A_VCVTSI2SD) or
                     (AsmOp = A_VCVTSI2SS) or
                     (AsmOp = A_VCVTTPD2DQ) or
                     (AsmOp = A_VCVTPD2UDQ) or
                     (AsmOp = A_VCVTQQ2PS) or
                     (AsmOp = A_VCVTTPD2UDQ) or
                     (AsmOp = A_VCVTUQQ2PS) or
                     (AsmOp = A_VCVTUSI2SD) or
                     (AsmOp = A_VCVTUSI2SS) or


                     // TODO check
                     (AsmOp = A_VCMPSS)


                    ) then

              InternalError(777205);
            end;

          end
          else if (InsTabMemRefSizeInfoCache^[AsmOp].ExistsSSEAVX) and
                  (InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize = msiUnknown) and
                  (not(ExistsMemRef)) then
          begin
            InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize := msiNoMemRef;
          end;
        end;
      end;

      for AsmOp := low(TAsmOp) to high(TAsmOp) do
      begin


        // only supported intructiones with SSE- or AVX-operands
        if not(InsTabMemRefSizeInfoCache^[AsmOp].ExistsSSEAVX) then
        begin
          InsTabMemRefSizeInfoCache^[AsmOp].MemRefSize  := msiUnknown;
          InsTabMemRefSizeInfoCache^[AsmOp].ConstSize   := csiUnknown;
        end;
      end;
    end;

    procedure InitAsm;
      begin
        build_spilling_operation_type_table;
        if not assigned(instabcache) then
          BuildInsTabCache;

        if not assigned(InsTabMemRefSizeInfoCache) then
          BuildInsTabMemRefSizeInfoCache;

      end;


    procedure DoneAsm;
      begin
        if assigned(operation_type_table) then
          begin
            dispose(operation_type_table);
            operation_type_table:=nil;
          end;
        if assigned(instabcache) then
          begin
            dispose(instabcache);
            instabcache:=nil;
          end;

        if assigned(InsTabMemRefSizeInfoCache) then
        begin
          dispose(InsTabMemRefSizeInfoCache);
          InsTabMemRefSizeInfoCache:=nil;
        end;
      end;


begin
  cai_align:=tai_align;
  cai_cpu:=taicpu;
end.
