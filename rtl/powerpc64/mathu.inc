{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Thomas Schatzl
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
  RoundModeMask        = %00000011;
  NonIEEEModeMask      = %00000100;

  InvalidOperationMask = %10000000;
  OverflowMask         = %01000000;
  UnderflowMask        = %00100000;
  ZeroDivideMask       = %00010000;
  InexactMask          = %00001000;
  AllExceptionsMask    = %11111000;
  ExceptionsPendingMask = %11111111111111100000011100000000;

  ExceptionMask        = InvalidOperationMask or OverflowMask or UnderflowMask or ZeroDivideMask or InexactMask;

  AllConfigBits        = ExceptionMask or NonIEEEModeMask or RoundModeMask;

function getFPSCR : DWord; assembler; nostackframe;
asm
  mffs f0
  stfd f0, -16(r1)
{$ifdef FPC_BIG_ENDIAN}
  lwz r3, -12(r1)
{$else}
  lwz r3, -16(r1)
{$endif}
end;

procedure setFPSCR(newFPSCR : DWord); assembler; nostackframe;
asm
{$ifdef FPC_BIG_ENDIAN}
  stw r3, -12(r1)
{$else}
  stw r3, -16(r1)
{$endif}
  lfd f0, -16(r1)
  mtfsf 255, f0
end;

{$ifdef aix}
const
  FP_RND_RZ = 0;
  FP_RND_RN = 1;
  FP_RND_RP = 2;
  FP_RND_RM = 3;

function fp_is_enabled(Mask: DWord): boolean;cdecl;external;
procedure fp_enable(Mask: DWord);cdecl;external;
function feclearexcept(Mask: DWord):DWord;cdecl;external;
procedure fp_disable(Mask: DWord);cdecl;external;
function fp_read_rnd: word;cdecl;external;
function fp_swap_rnd(RoundMode: word): word;cdecl;external;

{$else aix}
const
  FP_RND_RZ = 1;
  FP_RND_RN = 0;
  FP_RND_RP = 2;
  FP_RND_RM = 3;
{$endif aix}

function GetRoundMode: TFPURoundingMode;
begin
{$ifndef aix}
  case (getFPSCR and RoundModeMask) of
{$else not aix}
  case fp_read_rnd of
{$endif not aix}
    FP_RND_RN : result := rmNearest;
    FP_RND_RZ : result := rmTruncate;
    FP_RND_RP : result := rmUp;
    FP_RND_RM : result := rmDown;
  end;
end;

function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
var
  mode : DWord;
begin
  softfloat_rounding_mode:=RoundMode;
  case (RoundMode) of
    rmNearest :
      begin
        mode := FP_RND_RN;
      end;
    rmTruncate :
      begin
        mode := FP_RND_RZ;
      end;
    rmUp :
      begin
        mode := FP_RND_RP;
      end;
    rmDown :
      begin
        mode := FP_RND_RM;
      end;
  end;
  result := GetRoundMode;
{$ifndef aix}
  setFPSCR((getFPSCR and (not RoundModeMask)) or mode);
{$else not aix}
  fp_swap_rnd(mode);
{$endif not aix}
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


function GetExceptionMask: TFPUExceptionMask;
begin
  result := [];
{$ifndef aix}
  if ((getFPSCR and InvalidOperationMask) = 0) then
    result := result + [exInvalidOp];
  if ((getFPSCR and OverflowMask) = 0) then
    result := result + [exOverflow];
  if ((getFPSCR and UnderflowMask) = 0) then
    result := result + [exUnderflow];
  if ((getFPSCR and ZeroDivideMask) = 0) then
    result := result + [exZeroDivide];
  if ((getFPSCR and InexactMask) = 0) then
    result := result + [exPrecision];
{$else not aix}
  if not fp_is_enabled(InvalidOperationMask) then
    result := result + [exInvalidOp];
  if not fp_is_enabled(OverflowMask) then
    result := result + [exOverflow];
  if not fp_is_enabled(UnderflowMask) then
    result := result + [exUnderflow];
  if not fp_is_enabled(ZeroDivideMask) then
    result := result + [exZeroDivide];
  if not fp_is_enabled(InexactMask) then
    result := result + [exPrecision];
{$endif not aix}
end;

function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
var
  mode : DWord;
begin
  mode := 0;
  softfloat_exception_mask := mask;
  if (exInvalidOp in Mask) then
    begin
      mode := mode or InvalidOperationMask;
    end;
  if (exOverflow in Mask) then
    begin
      mode := mode or OverflowMask;
    end;
  if (exUnderflow in Mask) then
    begin
      mode := mode or UnderflowMask;
    end;
  if (exZeroDivide in Mask) then
    begin
      mode := mode or ZeroDivideMask;
    end;
  if (exPrecision in Mask) then
    begin
      mode := mode or InexactMask;
    end;

  setFPSCR((getFPSCR or ExceptionMask) and not mode and not ExceptionsPendingMask);
  softfloat_exception_flags := [];;
  { also clear out pending exceptions on AIX }
{$ifdef aix}
  { clear pending exceptions }
  feclearexcept(AllExceptionsMask);
  { enable the exceptions that are not disabled }
  fp_enable(mode xor AllExceptionsMask);
  { and disable the rest }
  fp_disable(mode);
{$endif}
  result := Mask - [exDenormalized];
end;


procedure ClearExceptions(RaisePending: Boolean = true);
begin
{$ifdef aix}
  { clear pending exceptions }
  feclearexcept(AllExceptionsMask);
{$endif}
  softfloat_exception_flags := [];
  { RaisePending has no effect on PPC, always raises them at the correct location }
  setFPSCR(getFPSCR and (not ExceptionsPendingMask));
end;

