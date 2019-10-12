{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004 by Florian Klaempfl
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

 { for bootstrapping with 3.0.x/3.2.x }
{$if defined(darwin) or defined(FPUVFPV2) or defined(FPUVFPV3) or defined(FPUVFPV4) or defined(FPUVFPV3_d16) or defined(FPUFPV4_s16)}
{$define FPUARM_HAS_VFP_EXTENSION}
{$endif}

{$if defined(wince)}

const
  _DN_SAVE  = $00000000;
  _DN_FLUSH = $01000000;

  _EM_INVALID    = $00000010;
  _EM_DENORMAL   = $00080000;
  _EM_ZERODIVIDE = $00000008;
  _EM_OVERFLOW   = $00000004;
  _EM_UNDERFLOW  = $00000002;
  _EM_INEXACT    = $00000001;

  _IC_AFFINE     = $00040000;
  _IC_PROJECTIVE = $00000000;

  _RC_CHOP       = $00000300;
  _RC_UP         = $00000200;
  _RC_DOWN       = $00000100;
  _RC_NEAR       = $00000000;

  _PC_24         = $00020000;
  _PC_53         = $00010000;
  _PC_64         = $00000000;

  _MCW_DN        = $03000000;
  _MCW_EM        = $0008001F;
  _MCW_IC        = $00040000;
  _MCW_RC        = $00000300;
  _MCW_PC        = $00030000;

function _controlfp(new: DWORD; mask: DWORD): DWORD; cdecl; external 'coredll';

function GetRoundMode: TFPURoundingMode;
var
  c: dword;
begin
  c:=_controlfp(0, 0);
  Result:=TFPURoundingMode((c shr 16) and 3);
end;

function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
var
  c: dword;
begin
  c:=Ord(RoundMode) shl 16;
  c:=_controlfp(c, _MCW_RC);
  Result:=TFPURoundingMode((c shr 16) and 3);
end;

function GetPrecisionMode: TFPUPrecisionMode;
var
  c: dword;
begin
  c:=_controlfp(0, 0);
  if c and _MCW_PC = _PC_64 then
    Result:=pmDouble
  else
    Result:=pmSingle;
end;

function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
var
  c: dword;
begin
  Result:=GetPrecisionMode;
  if Precision = pmSingle then
    c:=_PC_24
  else
    c:=_PC_64;
  _controlfp(c, _MCW_PC);
end;

function ConvertExceptionMask(em: dword): TFPUExceptionMask;
begin
  Result:=[];
  if em and _EM_INVALID = 0 then
    Result:=Result + [exInvalidOp];
  if em and _EM_DENORMAL = 0 then
    Result:=Result + [exDenormalized];
  if em and _EM_ZERODIVIDE = 0 then
    Result:=Result + [exZeroDivide];
  if em and _EM_OVERFLOW = 0 then
    Result:=Result + [exOverflow];
  if em and _EM_UNDERFLOW = 0 then
    Result:=Result + [exUnderflow];
  if em and _EM_INEXACT = 0 then
    Result:=Result + [exPrecision];
end;

function GetExceptionMask: TFPUExceptionMask;
begin
  Result:=ConvertExceptionMask(_controlfp(0, 0));
end;

function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
var
  c: dword;
begin
  c:=0;
  if not(exInvalidOp in Mask) then
    c:=c or _EM_INVALID;
  if not(exDenormalized in Mask) then
    c:=c or _EM_DENORMAL;
  if not(exZeroDivide in Mask) then
    c:=c or _EM_ZERODIVIDE;
  if not(exOverflow in Mask) then
    c:=c or _EM_OVERFLOW;
  if not(exUnderflow in Mask) then
    c:=c or _EM_UNDERFLOW;
  if not(exPrecision in Mask) then
    c:=c or _EM_INEXACT;
  c:=_controlfp(c, _MCW_EM);
  Result:=ConvertExceptionMask(c);
end;

procedure ClearExceptions(RaisePending: Boolean =true);
begin
end;

{$elseif defined(FPUARM_HAS_VFP_EXTENSION)}

const
  _VFP_ENABLE_IM  =  1 shl 8;         { invalid operation      }
  _VFP_ENABLE_ZM  =  1 shl 9;         { divide by zero         }
  _VFP_ENABLE_OM  =  1 shl 10;        { overflow               }
  _VFP_ENABLE_UM  =  1 shl 11;        { underflow              }
  _VFP_ENABLE_PM  =  1 shl 12;        { inexact                }
  _VFP_ENABLE_DM  =  1 shl 15;        { denormalized operation }
  _VFP_ENABLE_ALL =  _VFP_ENABLE_IM or
                   _VFP_ENABLE_ZM or
                   _VFP_ENABLE_OM or
                   _VFP_ENABLE_UM or
                   _VFP_ENABLE_PM or
                   _VFP_ENABLE_DM;    { mask for all flags     }
                   
  _VFP_ROUNDINGMODE_MASK_SHIFT = 22;
  _VFP_ROUNDINGMODE_MASK = 3 shl _VFP_ROUNDINGMODE_MASK_SHIFT;

  _VFP_EXCEPTIONS_PENDING_MASK =
    (1 shl 0) or
    (1 shl 1) or
    (1 shl 2) or
    (1 shl 3) or
    (1 shl 4) or
    (1 shl 7);

function VFP_GetCW : dword; nostackframe; assembler;
  asm
    fmrx r0,fpscr
  end;


procedure VFP_SetCW(cw : dword); nostackframe; assembler;
  asm
    fmxr fpscr,r0
  end;


function VFPCw2RoundingMode(cw: dword): TFPURoundingMode;
  begin
    case (cw and _VFP_ROUNDINGMODE_MASK) shr _VFP_ROUNDINGMODE_MASK_SHIFT of
      0 : result := rmNearest;
      1 : result := rmUp;
      2 : result := rmDown;
      3 : result := rmTruncate;
    end;
  end;


function GetRoundMode: TFPURoundingMode; 
  begin
    result:=VFPCw2RoundingMode(VFP_GetCW);
  end;


function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
  var
    mode: dword;
    oldcw: dword;
  begin
    softfloat_rounding_mode:=RoundMode;
    oldcw:=VFP_GetCW;
    case (RoundMode) of
      rmNearest :      mode := 0;
      rmUp :           mode := 1;
      rmDown :         mode := 2;
      rmTruncate :     mode := 3;
    end;
    mode:=mode shl _VFP_ROUNDINGMODE_MASK_SHIFT;
    VFP_SetCW((oldcw and (not _VFP_ROUNDINGMODE_MASK)) or mode);
    result := VFPCw2RoundingMode(oldcw);
  end;


function GetPrecisionMode: TFPUPrecisionMode;
  begin
    result := pmDouble;
  end;


function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
  begin
    { nothing to do, not supported }
    result := pmDouble;
  end;


function VFPCw2ExceptionMask(cw: dword): TFPUExceptionMask;
  begin
    Result:=[];
    if (cw and _VFP_ENABLE_IM)=0 then
      include(Result,exInvalidOp);

    if (cw and _VFP_ENABLE_DM)=0 then
      include(Result,exDenormalized);

    if (cw and _VFP_ENABLE_ZM)=0 then
      include(Result,exZeroDivide);

    if (cw and _VFP_ENABLE_OM)=0 then
      include(Result,exOverflow);

    if (cw and _VFP_ENABLE_UM)=0 then
      include(Result,exUnderflow);

    if (cw and _VFP_ENABLE_PM)=0 then
      include(Result,exPrecision);
  end;


function GetExceptionMask: TFPUExceptionMask;
  begin
    { some ARM CPUs ignore writing to the hardware mask and just return 0, so we need to return
      the softfloat mask which should be in sync with the hard one }
    if VFP_GetCW=0 then
      Result:=softfloat_exception_mask
    else
      Result:=VFPCw2ExceptionMask(VFP_GetCW);
  end;


function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
  var
    cw : dword;
  begin
    cw:=VFP_GetCW;
    Result:=VFPCw2ExceptionMask(cw);
    cw:=cw and not(_VFP_ENABLE_ALL);

{$ifndef darwin}
    if not(exInvalidOp in Mask) then
      cw:=cw or _VFP_ENABLE_IM;

    if not(exDenormalized in Mask) then
      cw:=cw or _VFP_ENABLE_DM;

    if not(exZeroDivide in Mask) then
      cw:=cw or _VFP_ENABLE_ZM;

    if not(exOverflow in Mask) then
      cw:=cw or _VFP_ENABLE_OM;

    if not(exUnderflow in Mask) then
      cw:=cw or _VFP_ENABLE_UM;

    if not(exPrecision in Mask) then
      cw:=cw or _VFP_ENABLE_PM;
{$endif}
    VFP_SetCW(cw);
    softfloat_exception_mask:=Mask;
  end;


procedure ClearExceptions(RaisePending: Boolean =true);
  begin
    { RaisePending has no effect on ARM, it always raises them at the correct location }
    VFP_SetCW(VFP_GetCW and (not _VFP_EXCEPTIONS_PENDING_MASK));
    softfloat_exception_flags:=[];
  end;

{$else FPUARM_HAS_VFP_EXTENSION}

{*****************************************************************************
                                   FPA code
 *****************************************************************************}
{
 Docs from uclib

 * We have a slight terminology confusion here.  On the ARM, the register
 * we're interested in is actually the FPU status word - the FPU control
 * word is something different (which is implementation-defined and only
 * accessible from supervisor mode.)
 *
 * The FPSR looks like this:
 *
 *     31-24        23-16          15-8              7-0
 * | system ID | trap enable | system control | exception flags |
 *
 * We ignore the system ID bits; for interest's sake they are:
 *
 *  0000	"old" FPE
 *  1000	FPPC hardware
 *  0001	FPE 400
 *  1001	FPA hardware
 *
 * The trap enable and exception flags are both structured like this:
 *
 *     7 - 5     4     3     2     1     0
 * | reserved | INX | UFL | OFL | DVZ | IVO |
 *
 * where a `1' bit in the enable byte means that the trap can occur, and
 * a `1' bit in the flags byte means the exception has occurred.
 *
 * The exceptions are:
 *
 *  IVO - invalid operation
 *  DVZ - divide by zero
 *  OFL - overflow
 *  UFL - underflow
 *  INX - inexact (do not use; implementations differ)
 *
 * The system control byte looks like this:
 *
 *     7-5      4    3    2    1    0
 * | reserved | AC | EP | SO | NE | ND |
 *
 * where the bits mean
 *
 *  ND - no denormalised numbers (force them all to zero)
 *  NE - enable NaN exceptions
 *  SO - synchronous operation
 *  EP - use expanded packed-decimal format
 *  AC - use alternate definition for C flag on compare operations
 */

/* masking of interrupts */
#define _FPU_MASK_IM	0x00010000	/* invalid operation */
#define _FPU_MASK_ZM	0x00020000	/* divide by zero */
#define _FPU_MASK_OM	0x00040000	/* overflow */
#define _FPU_MASK_UM	0x00080000	/* underflow */
#define _FPU_MASK_PM	0x00100000	/* inexact */
#define _FPU_MASK_DM	0x00000000	/* denormalized operation */

/* The system id bytes cannot be changed.
   Only the bottom 5 bits in the trap enable byte can be changed.
   Only the bottom 5 bits in the system control byte can be changed.
   Only the bottom 5 bits in the exception flags are used.
   The exception flags are set by the fpu, but can be zeroed by the user. */
#define _FPU_RESERVED	0xffe0e0e0	/* These bits are reserved.  */

/* The fdlibm code requires strict IEEE double precision arithmetic,
   no interrupts for exceptions, rounding to nearest.  Changing the
   rounding mode will break long double I/O.  Turn on the AC bit,
   the compiler generates code that assumes it is on.  */
#define _FPU_DEFAULT	0x00001000	/* Default value.  */
#define _FPU_IEEE	0x001f1000	/* Default + exceptions enabled. */
}


{$if not(defined(gba)) and not(defined(nds)) and not(defined(FPUSOFT)) and not(defined(FPULIBGCC))}
const
  _FPU_MASK_IM  =  $00010000;      { invalid operation      }
  _FPU_MASK_ZM  =  $00020000;      { divide by zero         }
  _FPU_MASK_OM  =  $00040000;      { overflow               }
  _FPU_MASK_UM  =  $00080000;      { underflow              }
  _FPU_MASK_PM  =  $00100000;      { inexact                }
  _FPU_MASK_DM  =  $00000000;      { denormalized operation }
  _FPU_MASK_ALL =  $001f0000;      { mask for all flags     }

function FPU_GetCW : dword; nostackframe; assembler;
  asm
    rfs r0
  end;


procedure FPU_SetCW(cw : dword); nostackframe; assembler;
  asm
    wfs r0
  end;


function FPUCw2ExceptionMask(cw: dword): TFPUExceptionMask;
  begin
    Result:=[];

    if (cw and _FPU_MASK_IM)=0 then
      include(Result,exInvalidOp);

    if (cw and _FPU_MASK_DM)=0 then
      include(Result,exDenormalized);

    if (cw and _FPU_MASK_ZM)=0 then
      include(Result,exZeroDivide);

    if (cw and _FPU_MASK_OM)=0 then
      include(Result,exOverflow);

    if (cw and _FPU_MASK_UM)=0 then
      include(Result,exUnderflow);

    if (cw and _FPU_MASK_PM)=0 then
      include(Result,exPrecision);
  end;
{$endif}


function GetRoundMode: TFPURoundingMode;
  begin
    GetRoundMode:=softfloat_rounding_mode;
  end;


function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
  begin
    result:=softfloat_rounding_mode;
    softfloat_rounding_mode:=RoundMode;
  end;


function GetPrecisionMode: TFPUPrecisionMode;
  begin
    result := pmDouble;
  end;


function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
  begin
    { does not apply }
    result := pmDouble;
  end;


function GetExceptionMask: TFPUExceptionMask;
  begin
{$if not(defined(gba)) and not(defined(nds)) and not(defined(FPUSOFT)) and not(defined(FPULIBGCC))}
    Result:=FPUCw2ExceptionMask(FPU_GetCW);
{$else}
    Result:=softfloat_exception_mask;
{$endif}
  end;


function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
{$if not(defined(gba)) and not(defined(nds)) and not(defined(FPUSOFT)) and not(defined(FPULIBGCC))}
  var
    cw : dword;
{$endif}
  begin
{$if not(defined(gba)) and not(defined(nds)) and not(defined(FPUSOFT)) and not(defined(FPULIBGCC))}
    cw:=FPU_GetCW;
    Result:=FPUCw2ExceptionMask(cw);
    cw:=cw or _FPU_MASK_ALL;

    if exInvalidOp in Mask then
      cw:=cw and not(_FPU_MASK_IM);

    if exDenormalized in Mask then
      cw:=cw and not(_FPU_MASK_DM);

    if exZeroDivide in Mask then
      cw:=cw and not(_FPU_MASK_ZM);

    if exOverflow in Mask then
      cw:=cw and not(_FPU_MASK_OM);

    if exUnderflow in Mask then
      cw:=cw and not(_FPU_MASK_UM);

    if exPrecision in Mask then
      cw:=cw and not(_FPU_MASK_PM);

    FPU_SetCW(cw);
{$else}
    Result:=softfloat_exception_mask;
{$endif}
    softfloat_exception_mask:=Mask;
  end;


procedure ClearExceptions(RaisePending: Boolean =true);
  begin
    softfloat_exception_flags:=[];
  end;

{$endif wince}
