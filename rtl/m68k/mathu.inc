{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2012 by Sven Barth
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$if defined(FPU68881) or defined(FPUCOLDFIRE)}

{               68881/2 FPCR Encodings
   Rounding Mode                 Rounding Precision
    (RND Field)        Encoding    (PREC Field)
 To Nearest (RN)         0 0        Extend (X)
 To Zero (RZ)            0 1        Single (S)
 To Minus Infinity (RM)  1 0        Double (D)
 To Plus Infinity (RP)   1 1        Undefined
}

{ 68881/2 FPCR layout }
{ Exception Enable Byte: }
{ 15 - BSUN  - Branch/Set on Unordered }
{ 14 - SNAN  - Signal Not A Number }
{ 13 - OPERR - Operand Error }
{ 12 - OVFL  - Overflow }
{ 11 - UNFL  - Underflow }
{ 10 - DZ    - Divide by Zero }
{ 09 - INEX2 - Inexact Operation }
{ 08 - INEX1 - Inexact Decimal Input }
{ Mode Control Byte: }
{ 07 - PREC  - Rounding Precision }
{ 06 - PREC  - Rounding Precision }
{ 05 - RND   - Rounding Mode }
{ 04 - RND   - Rounding Mode }
{ 03 - 0     - Reserved, Set to zero }
{ 02 - 0     - Reserved, Set to zero }
{ 01 - 0     - Reserved, Set to zero }
{ 00 - 0     - Reserved, Set to zero }

{
  Please note that the rounding mode setting via FPCR in most emulators is broken.
  The list includes most versions and incarnations of UAE, MorphOS' Trance emulator,
  and others. The following code was verified to work on real hardware. (KB) 
}

const
  FPU68K_ROUND_MASK_SHIFT = 4;
  FPU68K_ROUND_MASK       = 3 shl FPU68K_ROUND_MASK_SHIFT;

  FPU68K_ROUND_NEAREST    = 0 shl FPU68K_ROUND_MASK_SHIFT;
  FPU68K_ROUND_ZERO       = 1 shl FPU68K_ROUND_MASK_SHIFT;
  FPU68K_ROUND_MINUSINF   = 2 shl FPU68K_ROUND_MASK_SHIFT;
  FPU68K_ROUND_PLUSINF    = 3 shl FPU68K_ROUND_MASK_SHIFT;

const
  FPU68K_PREC_MASK_SHIFT  = 6;
  FPU68K_PREC_MASK        = 3 shl FPU68K_PREC_MASK_SHIFT;

  FPU68K_PREC_EXTENDED    = 0 shl FPU68K_PREC_MASK_SHIFT;
  FPU68K_PREC_SINGLE      = 1 shl FPU68K_PREC_MASK_SHIFT;
  FPU68K_PREC_DOUBLE      = 2 shl FPU68K_PREC_MASK_SHIFT;

const
  FPU68K_EXCEPT_MASK_SHIFT = 8;
  FPU68K_EXCEPT_MASK       = 255 shl FPU68K_EXCEPT_MASK_SHIFT;

  FPU68K_EXCEPT_INEX1      = 1 shl (FPU68K_EXCEPT_MASK_SHIFT + 0);
  FPU68K_EXCEPT_INEX2      = 1 shl (FPU68K_EXCEPT_MASK_SHIFT + 1);
  FPU68K_EXCEPT_DZ         = 1 shl (FPU68K_EXCEPT_MASK_SHIFT + 2);
  FPU68K_EXCEPT_UNFL       = 1 shl (FPU68K_EXCEPT_MASK_SHIFT + 3);
  FPU68K_EXCEPT_OVFL       = 1 shl (FPU68K_EXCEPT_MASK_SHIFT + 4);
  FPU68K_EXCEPT_OPERR      = 1 shl (FPU68K_EXCEPT_MASK_SHIFT + 5);
  FPU68K_EXCEPT_SNAN       = 1 shl (FPU68K_EXCEPT_MASK_SHIFT + 6);
  FPU68K_EXCEPT_BSUN       = 1 shl (FPU68K_EXCEPT_MASK_SHIFt + 7);

  FPU68K_AE_MASK           = $F8;


function GetExceptionMask: TFPUExceptionMask;
begin
  Result := softfloat_exception_mask;
end;

function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
const
  FPCToFPUExceptionFlags: array[TFPUException] of DWord =
      ( {exInvalidOp,} FPU68K_EXCEPT_OPERR or FPU68K_EXCEPT_SNAN or FPU68K_EXCEPT_BSUN,
        {exDenormalized,} 0,
        {exZeroDivide,} FPU68K_EXCEPT_DZ,
        {exOverflow,} FPU68K_EXCEPT_OVFL,
        {exUnderflow,} FPU68K_EXCEPT_UNFL,
        {exPrecision} FPU68K_EXCEPT_INEX1 or FPU68K_EXCEPT_INEX2 );
  FPUToFPCExceptionFlags: array[0..7] of TFPUExceptionMask =
      ( [exPrecision], [exPrecision], [exZeroDivide], [exUnderflow], [exOverflow], [exInvalidOp], [exInvalidOp], [exInvalidOp] );
var
  oldMode, Mode: DWord;
  e: TFPUException;
  i: longint;
begin
  result:=[];

  oldMode:=(GetFPCR and FPU68K_EXCEPT_MASK) shr FPU68K_EXCEPT_MASK_SHIFT;
  for i:=low(FPUToFPCExceptionFlags) to high(FPUToFPCExceptionFlags) do
    if ((1 shl i) and oldMode) > 0 then
      result:=result+FPUToFPCExceptionFlags[i];

  mode:=0;
  { The bits set inside FPCR register are the enabled exceptions,
    not the masked exceptions, thus we need to invert list }
  for e:=low(TFPUException) to high(TFPUException) do
    if not (e in Mask) then
      mode:=mode or FPCToFPUExceptionFlags[e];

  SetFPCR((GetFPCR and not FPU68K_EXCEPT_MASK) or (mode and FPU68K_EXCEPT_MASK));
  { Wipe out any previous exception }
  SetFPSR(GetFPSR and (not (FPU68K_AE_MASK or FPU68K_EXCEPT_MASK)));
  softfloat_exception_mask:=mask;
end;

function GetRoundMode: TFPURoundingMode;
const
  FPUToFPCRoundingMode: array[0..3] of TFPURoundingMode = ( rmNearest, rmTruncate, rmUp, rmDown );
begin
  Result:=FPUToFPCRoundingMode[(GetFPCR and FPU68K_ROUND_MASK) shr FPU68K_ROUND_MASK_SHIFT];
end;

function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
const
  FPCToFPURoundingMode: array[TFPURoundingMode] of DWord =
      ( FPU68K_ROUND_NEAREST, FPU68K_ROUND_MINUSINF, FPU68K_ROUND_PLUSINF, FPU68K_ROUND_ZERO );
var
  FPCR: DWord;
begin
  FPCR:=GetFPCR and not FPU68K_ROUND_MASK;
  SetFPCR(FPCR or FPCToFPURoundingMode[RoundMode]);
  softfloat_rounding_mode:=RoundMode;
  Result:=RoundMode;
end;

function GetPrecisionMode: TFPUPrecisionMode;
begin
  result:=pmDouble;
end;

function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
begin
  result:=pmDouble;
end;

procedure ClearExceptions(RaisePending: Boolean);
begin
  SetFPCR(GetFPCR and not FPU68K_EXCEPT_MASK);
  SetFPSR(0);
  softfloat_exception_flags:=[];
end;
{$else}

function GetExceptionMask: TFPUExceptionMask;
begin
  Result := softfloat_exception_mask;
end;

function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
begin
  result:=softfloat_exception_mask;
  softfloat_exception_mask:=mask;
end;

function GetRoundMode: TFPURoundingMode;
begin
  Result:=softfloat_rounding_mode;
end;

function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
begin
  Result:=softfloat_rounding_mode;
  softfloat_rounding_mode:=RoundMode;
end;

function GetPrecisionMode: TFPUPrecisionMode;
begin
  result:=pmDouble;
end;

function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
begin
  result:=pmDouble;
end;

procedure ClearExceptions(RaisePending: Boolean);
begin
  softfloat_exception_flags:=[];
end;
{$endif}
