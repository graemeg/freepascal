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


function GetRoundMode: TFPURoundingMode;
  const
    bits2rm: array[0..3] of TFPURoundingMode = (rmNearest,rmTruncate,rmUp,rmDown);
  begin
    result:=TFPURoundingMode(bits2rm[(get_fsr shr 30) and 3])
  end;


function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
  const
    rm2bits: array[TFPURoundingMode] of byte = (0,3,2,1);
  var
    cw: dword;
  begin
    cw:=get_fsr;
    softfloat_rounding_mode:=RoundMode;
    result:=TFPURoundingMode(cw shr 30);
    set_fsr((cw and $3fffffff) or (rm2bits[RoundMode] shl 30));
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
  end;


procedure ClearExceptions(RaisePending: Boolean =true);
  begin
    set_fsr(get_fsr and $fffffc1f);
  end;

