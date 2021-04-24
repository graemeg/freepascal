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

{$asmmode gas}

function getfpcr: qword; nostackframe; assembler;
  asm
    mrs x0,fpcr
  end;


procedure setfpcr(val: qword); nostackframe; assembler;
  asm
    msr fpcr,x0
  end;


function getfpsr: qword; nostackframe; assembler;
  asm
    mrs x0,fpsr
  end;


procedure setfpsr(val: qword); nostackframe; assembler;
  asm
    msr fpsr, x0
  end;


function GetRoundMode: TFPURoundingMode;
  const
    bits2rm: array[0..3] of TFPURoundingMode = (rmNearest,rmUp,rmDown,rmTruncate);
  begin
    result:=TFPURoundingMode(bits2rm[(getfpcr shr 22) and 3])
  end;


function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
  const
    rm2bits: array[TFPURoundingMode] of byte = (0,2,1,3);
  begin
    softfloat_rounding_mode:=RoundMode;
    SetRoundMode:=GetRoundMode;
    setfpcr((getfpcr and $ff3fffff) or (rm2bits[RoundMode] shl 22));
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
  fpu_ioe = 1 shl 8;
  fpu_dze = 1 shl 9;
  fpu_ofe = 1 shl 10;
  fpu_ufe = 1 shl 11;
  fpu_ixe = 1 shl 12;
  fpu_ide = 1 shl 15;
  fpu_exception_mask = qword(fpu_ioe or fpu_dze or fpu_ofe or fpu_ufe or fpu_ixe or fpu_ide);
  fpu_exception_mask_to_status_mask_shift = 8;


function GetExceptionMask: TFPUExceptionMask;
  {
  var
    fpcr: dword;
  }
  begin
    { as I am not aware of any hardware exception supporting AArch64 implementation,
      and else the trapping enable flags are RAZ, return the softfloat exception mask (FK)

    fpcr:=getfpcr;
    result:=[];
    if ((fpcr and fpu_ioe)=0) then
      result := result+[exInvalidOp];
    if ((fpcr and fpu_ofe)=0) then
      result := result+[exOverflow];
    if ((fpcr and fpu_ufe)=0) then
      result := result+[exUnderflow];
    if ((fpcr and fpu_dze)=0) then
      result := result+[exZeroDivide];
    if ((fpcr and fpu_ixe)=0) then
      result := result+[exPrecision];
    if ((fpcr and fpu_ide)=0) then
      result := result+[exDenormalized];
    }
    { as the fpcr flags might be RAZ, the softfloat exception mask
      is considered as the authoritative mask }
    result:=softfloat_exception_mask;
  end;


function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
  var
    newfpcr: qword;
  begin
    { clear "exception happened" flags }
    ClearExceptions(false);
    softfloat_exception_mask:=mask;

    { at least the ThunderX AArch64 support apparently hardware exceptions,
      so set fpcr correctly, thought it might be WI on most implementations it does not hurt
    }
    newfpcr:=fpu_exception_mask;
    if exInvalidOp in Mask then
      newfpcr:=newfpcr and not(fpu_ioe);
    if exOverflow in Mask then
      newfpcr:=newfpcr and not(fpu_ofe);
    if exUnderflow in Mask then
      newfpcr:=newfpcr and not(fpu_ufe);
    if exZeroDivide in Mask then
      newfpcr:=newfpcr and not(fpu_dze);
    if exPrecision in Mask then
      newfpcr:=newfpcr and not(fpu_ixe);
    if exDenormalized in Mask then
      newfpcr:=newfpcr and not(fpu_ide);
    setfpcr((getfpcr and not(fpu_exception_mask)) or newfpcr);

    { as the fpcr flags might be RAZ, the softfloat exception mask
      is considered as the authoritative mask }
    result:=softfloat_exception_mask;
  end;


procedure ClearExceptions(RaisePending: Boolean);
  var
    fpsr: qword;
    f: TFPUException;
  begin
    fpsr:=getfpsr;
    if raisepending then
      begin
        if (fpsr and (fpu_dze shr fpu_exception_mask_to_status_mask_shift)) <> 0 then
          float_raise(exZeroDivide);
        if (fpsr and (fpu_ofe shr fpu_exception_mask_to_status_mask_shift)) <> 0 then
          float_raise(exOverflow);
        if (fpsr and (fpu_ufe shr fpu_exception_mask_to_status_mask_shift)) <> 0 then
          float_raise(exUnderflow);
        if (fpsr and (fpu_ioe shr fpu_exception_mask_to_status_mask_shift)) <> 0 then
          float_raise(exInvalidOp);
        if (fpsr and (fpu_ixe shr fpu_exception_mask_to_status_mask_shift)) <> 0 then
          float_raise(exPrecision);
        if (fpsr and (fpu_ide shr fpu_exception_mask_to_status_mask_shift)) <> 0 then
          float_raise(exDenormalized);
        { now the soft float exceptions }
        for f in  softfloat_exception_flags do
          float_raise(f);
      end;
    softfloat_exception_flags:=[];
    setfpsr(fpsr and not(fpu_exception_mask shr fpu_exception_mask_to_status_mask_shift));
  end;
