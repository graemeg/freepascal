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

const
  { FPU enable exception bits for FCSR register }
  fpu_enable_inexact   =  $80;
  fpu_enable_underflow = $100;
  fpu_enable_overflow  = $200;
  fpu_enable_div_zero  = $400;
  fpu_enable_invalid   = $800;
  fpu_enable_mask      = $F80;
  default_fpu_enable = fpu_enable_div_zero or fpu_enable_invalid;

  fpu_flags_mask = $7C;
  fpu_cause_mask = $3F000;

  { FPU rounding mask and values }
  fpu_rounding_mask    = $3;
  fpu_rounding_nearest = 0;
  fpu_rounding_towards_zero = 1;
  fpu_rounding_plus_inf = 2;
  fpu_rounding_minus_inf = 3;

const
  roundmode2fsr : array [TFPURoundingMode] of byte=(
    fpu_rounding_nearest,
    fpu_rounding_minus_inf,
    fpu_rounding_plus_inf,
    fpu_rounding_towards_zero
  );

  fsr2roundmode : array [0..3] of TFPURoundingMode = (
    rmNearest,
    rmTruncate,
    rmUp,
    rmDown
  );

function GetRoundMode: TFPURoundingMode;
  begin
    result:=fsr2roundmode[get_fsr and fpu_rounding_mask];
  end;

function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
  var
    fsr: longword;
  begin
    fsr:=get_fsr;
    result:=fsr2roundmode[fsr and fpu_rounding_mask];
    softfloat_rounding_mode:=RoundMode;
    set_fsr((fsr and not fpu_rounding_mask) or roundmode2fsr[RoundMode]);
  end;


function GetPrecisionMode: TFPUPrecisionMode;
  begin
    result:=pmDouble;
  end;


function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
  begin
    result:=pmDouble;
  end;


function fsr2ExceptionMask(fsr: longword): TFPUExceptionMask;
  begin
    result:=[];
    { invalid operation }
    if (fsr and fpu_enable_invalid)=0 then
      include(result,exInvalidOp);

    { zero divide }
    if (fsr and fpu_enable_div_zero)=0 then
      include(result,exZeroDivide);

    { overflow }
    if (fsr and fpu_enable_overflow)=0 then
      include(result,exOverflow);

    { underflow: }
    if (fsr and fpu_enable_underflow)=0 then
      include(result,exUnderflow);

    { Precision (inexact result) }
    if (fsr and fpu_enable_inexact)=0 then
      include(result,exPrecision);
  end;


function GetExceptionMask: TFPUExceptionMask;
  begin
    result:=fsr2ExceptionMask(get_fsr);
  end;


function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
  var
    fsr : longword;
  begin
    fsr:=get_fsr;
    result:=fsr2ExceptionMask(fsr);

    { Reset flags, cause and enables }
    fsr := fsr and not (fpu_flags_mask or fpu_cause_mask or fpu_enable_mask);

    { invalid operation }
    if not (exInvalidOp in mask) then
      fsr:=fsr or (fpu_enable_invalid);

    { zero divide }
    if not (exZeroDivide in mask) then
      fsr:=fsr or (fpu_enable_div_zero);

    { overflow }
    if not (exOverflow in mask) then
      fsr:=fsr or (fpu_enable_overflow);

    { underflow }
    if not (exUnderflow in mask) then
      fsr:=fsr or (fpu_enable_underflow);

    { Precision (inexact result) }
    if not (exPrecision in mask) then
      fsr:=fsr or (fpu_enable_inexact);

    { update control register contents }
    set_fsr(fsr);
  end;


procedure ClearExceptions(RaisePending: Boolean =true);
  begin
    set_fsr(get_fsr and not (fpu_flags_mask or fpu_cause_mask));
  end;

