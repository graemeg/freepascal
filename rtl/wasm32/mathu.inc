{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2003 by Florian Klaempfl
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}

function GetRoundMode: TFPURoundingMode;
begin
  GetRoundMode:=rmNearest;
end;

function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
begin
  SetRoundMode:=rmNearest;
end;

function GetPrecisionMode: TFPUPrecisionMode;
begin
  GetPrecisionMode:=pmDouble;
end;

function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
begin
  SetPrecisionMode:=pmDouble;
end;

function GetExceptionMask: TFPUExceptionMask;
begin
  GetExceptionMask:=[exDenormalized,exOverflow,exPrecision];
end;

function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
begin
  SetExceptionMask:=[exDenormalized,exOverflow,exPrecision];
end;

procedure ClearExceptions(RaisePending: Boolean);
begin
end;
