{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by the Free Pascal development team.

    Processor dependent implementation for the system unit for
    RiscV which is common to all RiscV types

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{****************************************************************************
                       fpu exception related stuff
****************************************************************************}

{$ifdef FPUFD}
const
  fpu_nx = 1 shl 0;
  fpu_uf = 1 shl 1;
  fpu_of = 1 shl 2;
  fpu_dz = 1 shl 3;
  fpu_nv = 1 shl 4;

function getfflags: sizeuint; nostackframe; assembler;
  asm
    frflags a0
  end;


procedure setfflags(flags : sizeuint); nostackframe; assembler;
  asm
    fsflags a0
  end;


procedure RaisePendingExceptions;
  var
    fflags : sizeuint;
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


procedure fpc_throwfpuexception;[public,alias:'FPC_THROWFPUEXCEPTION'];
  var
    fflags : sizeuint;
  begin
    fflags:=getfflags;
    { check, if the exception is masked }
    if ((fflags and fpu_dz) <> 0) and (exZeroDivide in softfloat_exception_mask) then
      fflags:=fflags and not(fpu_dz);
    if ((fflags and fpu_of) <> 0) and (exOverflow in softfloat_exception_mask) then
      fflags:=fflags and not(fpu_of);
    if ((fflags and fpu_uf) <> 0) and (exUnderflow in softfloat_exception_mask) then
      fflags:=fflags and not(fpu_uf);
    if ((fflags and fpu_nv) <> 0) and (exInvalidOp in softfloat_exception_mask) then
      fflags:=fflags and not(fpu_nv);
    if ((fflags and fpu_nx) <> 0) and (exPrecision in softfloat_exception_mask) then
      fflags:=fflags and not(fpu_nx);
    setfflags(fflags);
    if fflags<>0 then
      RaisePendingExceptions;
  end;
{$endif FPUFD}

procedure fpc_cpuinit;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    softfloat_exception_mask:=[exPrecision,exUnderflow];
  end;
