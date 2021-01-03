{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 by Jonas Maebe
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}

{$ifdef FPUFD}
function getrm: dword; nostackframe; assembler;
  asm
    frrm a0
  end;


procedure setrm(val: dword); nostackframe; assembler;
  asm
    fsrm a0
  end;


function getfflags: dword; nostackframe; assembler;
  asm
    frflags a0
  end;


procedure setfflags(flags : dword); nostackframe; assembler;
  asm
    fsflags a0
  end;


function GetRoundMode: TFPURoundingMode;
  const
    bits2rm: array[0..3] of TFPURoundingMode = (rmNearest,rmTruncate,rmDown,rmUp);
  begin
    result:=TFPURoundingMode(bits2rm[getrm])
  end;


function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
  const
    rm2bits : array[TFPURoundingMode] of byte = (0,2,3,1);
  begin
    softfloat_rounding_mode:=RoundMode;
    SetRoundMode:=GetRoundMode;
    setrm(rm2bits[RoundMode]);
  end;


function GetPrecisionMode: TFPUPrecisionMode;
  begin
    result:=pmDouble;
  end;


function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
  begin
    result:=pmDouble;
  end;

const
  fpu_nx = 1 shl 0;
  fpu_uf = 1 shl 1;
  fpu_of = 1 shl 2;
  fpu_dz = 1 shl 3;
  fpu_nv = 1 shl 4;


function GetExceptionMask: TFPUExceptionMask;
  begin
    Result:=softfloat_exception_mask;
  end;


function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
  begin
    Result:=softfloat_exception_mask;
    { clear "exception happened" flags }
    ClearExceptions(false);
    softfloat_exception_mask:=Mask;
  end;


procedure RaisePendingExceptions;
  var
    fflags : dword;
    f: TFPUException;
  begin
    fflags:=getfflags;
    if (fflags and fpu_dz) <> 0 then
      float_raise(exZeroDivide);
    if (fflags and fpu_of) <> 0 then
      float_raise(exOverflow);
    if (fflags and fpu_uf) <> 0 then
      float_raise(exUnderflow);
    if (fflags and fpu_nv) <> 0 then
      float_raise(exInvalidOp);
    if (fflags and fpu_nx) <> 0 then
      float_raise(exPrecision);
    { now the soft float exceptions }
    for f in softfloat_exception_flags do
      float_raise(f);
  end;


procedure ClearExceptions(RaisePending: Boolean);
  begin
    if raisepending then
      RaisePendingExceptions;
    softfloat_exception_flags:=[];
    setfflags(0);
  end;
{$else}
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
    Result:=softfloat_exception_mask;
  end;


function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
  begin
    Result:=softfloat_exception_mask;
    softfloat_exception_mask:=Mask;
  end;


procedure ClearExceptions(RaisePending: Boolean =true);
  begin
    softfloat_exception_flags:=[];
  end;
{$endif}
