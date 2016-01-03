{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ exported by the system unit }
function get_fsr : dword;external name 'FPC_GETFSR';
procedure set_fsr(fsr : dword);external name 'FPC_SETFSR';

function FPUExceptionMaskToSoftFloatMask(const Mask: TFPUExceptionMask): byte;
begin
    result:=0;
    if exInvalidOp in Mask then
      result:=result or (1 shl ord(exInvalidOp));
    if exDenormalized in Mask then
      result:=result or (1 shl ord(exDenormalized));
    if exZeroDivide in Mask then
      result:=result or (1 shl ord(exZeroDivide));
    if exOverflow in Mask then
      result:=result or (1 shl ord(exOverflow));
    if exUnderflow in Mask then
      result:=result or (1 shl ord(exUnderflow));
    if exPrecision in Mask then
      result:=result or (1 shl ord(exPrecision));
end;

function GetRoundMode: TFPURoundingMode;
  begin
    result:=TFPURoundingMode(get_fsr shr 30);
  end;

function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
  var
    cw: dword;
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    cw:=get_fsr;
    result:=TFPURoundingMode(cw shr 30);
    set_fsr((cw and $3fffffff) or (dword(RoundMode) shl 30));
=======
=======
>>>>>>> origin/fixes_2_2
    case (RoundMode) of
      rmNearest :
        softfloat_rounding_mode := float_round_nearest_even;
      rmTruncate :
        softfloat_rounding_mode := float_round_to_zero;
      rmUp :
        softfloat_rounding_mode := float_round_up;
      rmDown :
        softfloat_rounding_mode := float_round_down;
    end;
    set_fsr((get_fsr and $3fffffff) or (dword(RoundMode) shl 30));
    result:=TFPURoundingMode(get_fsr shr 30);
>>>>>>> graemeg/fixes_2_2
  end;


function GetPrecisionMode: TFPUPrecisionMode;
  begin
    result:=pmDouble;
  end;


function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
  begin
    result:=pmDouble;
  end;


function FSR2ExceptionMask(fsr: dword): TFPUExceptionMask;
  begin
    result:=[];
    { invalid operation: bit 27 }
    if (fsr and (1 shl 27))=0 then
      include(result,exInvalidOp);

    { zero divide: bit 24 }
    if (fsr and (1 shl 24))=0 then
      include(result,exZeroDivide);

    { overflow: bit 26 }
    if (fsr and (1 shl 26))=0 then
      include(result,exOverflow);

    { underflow: bit 25 }
    if (fsr and (1 shl 25))=0 then
      include(result,exUnderflow);

    { Precision (inexact result): bit 23 }
    if (fsr and (1 shl 23))=0 then
      include(result,exPrecision);
  end;


function GetExceptionMask: TFPUExceptionMask;
  begin
    result:=FSR2ExceptionMask(get_fsr);
  end;


function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
  var
    fsr : dword;
  begin
    fsr:=get_fsr;
    result:=FSR2ExceptionMask(fsr);

    { invalid operation: bit 27 }
    if (exInvalidOp in mask) then
      fsr:=fsr and not(1 shl 27)
    else
      fsr:=fsr or (1 shl 27);

    { zero divide: bit 24 }
    if (exZeroDivide in mask) then
      fsr:=fsr and not(1 shl 24)
    else
      fsr:=fsr or (1 shl 24);

    { overflow: bit 26 }
    if (exOverflow in mask) then
      fsr:=fsr and not(1 shl 26)
    else
      fsr:=fsr or (1 shl 26);

    { underflow: bit 25 }
    if (exUnderflow in mask) then
      fsr:=fsr and not(1 shl 25)
    else
      fsr:=fsr or (1 shl 25);

    { Precision (inexact result): bit 23 }
    if (exPrecision in mask) then
      fsr:=fsr and not(1 shl 23)
    else
      fsr:=fsr or (1 shl 23);

    { update control register contents }
    set_fsr(fsr);

    softfloat_exception_mask:=FPUExceptionMaskToSoftFloatMask(mask);
  end;


procedure ClearExceptions(RaisePending: Boolean =true);
  begin
    set_fsr(get_fsr and $fffffc1f);
  end;

