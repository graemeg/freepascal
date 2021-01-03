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

{$ASMMODE ATT}
{$define FPC_MATH_HAS_ARCTAN2}
function arctan2(y,x : float) : float;assembler;
  asm
     fldt y
     fldt x
     fpatan
     fwait
  end;


{$define FPC_MATH_HAS_SINCOS}
procedure sincos(theta : extended;out sinus,cosinus : extended);assembler;
  asm
    fldt theta
    fsincos
    fstpt (%edx)
    fstpt (%eax)
    fwait
  end;


procedure sincos(theta : double;out sinus,cosinus : double);assembler;
  asm
    fldl theta
    fsincos
    fstpl (%edx)
    fstpl (%eax)
    fwait
  end;


procedure sincos(theta : single;out sinus,cosinus : single);assembler;
  asm
    flds theta
    fsincos
    fstps (%edx)
    fstps (%eax)
    fwait
  end;


{$define FPC_MATH_HAS_TAN}
function tan(x : float) : float;assembler;
  asm
    fldt X
    fptan
    fstp %st
    fwait
  end;


{$define FPC_MATH_HAS_COTAN}
function cotan(x : float) : float;assembler;
  asm
    fldt X
    fptan
    fdivp %st,%st(1)
    fwait
  end;


{$define FPC_MATH_HAS_LOG2}
function log2(x : float) : float;assembler;
  asm
    fld1
    fldt x
    fyl2x
    fwait
  end;


{$define FPC_MATH_HAS_DIVMOD}
procedure DivMod(Dividend: Integer; Divisor: Word; var Result, Remainder: Word);assembler;
asm
  pushl  %edi
  movzwl %dx,%edi
  cltd
  idiv   %edi
  movw   %ax,(%ecx)
  movl   Remainder,%ecx
  movw   %dx,(%ecx)
  popl   %edi
end;


procedure DivMod(Dividend: Integer; Divisor: Word; var Result, Remainder: SmallInt);assembler;
asm
  pushl  %edi
  movzwl %dx,%edi
  cltd
  idiv   %edi
  movw   %ax,(%ecx)
  movl   Remainder,%ecx
  movw   %dx,(%ecx)
  popl   %edi
end;


procedure DivMod(Dividend: DWord; Divisor: DWord; var Result, Remainder: DWord);assembler;
asm
  pushl %edi
  movl %edx,%edi
  xorl %edx,%edx
  div  %edi
  movl %eax,(%ecx)
  movl Remainder,%ecx
  movl %edx,(%ecx)
  popl %edi
end;


procedure DivMod(Dividend: Integer; Divisor: Integer; var Result, Remainder: Integer);assembler;
asm
  pushl %edi
  movl %edx,%edi
  cltd
  idiv %edi
  movl %eax,(%ecx)
  movl Remainder,%ecx
  movl %edx,(%ecx)
  popl %edi
end;


function GetRoundMode: TFPURoundingMode;
begin
  Result := TFPURoundingMode((Get8087CW shr 10) and 3);
end;


function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
var
  CtlWord: Word;
begin
  softfloat_rounding_mode:=RoundMode;
  CtlWord := Get8087CW;
  Set8087CW((CtlWord and $F3FF) or (Ord(RoundMode) shl 10));
  if has_sse_support then
    SetMXCSR((GetMXCSR and $ffff9fff) or (dword(RoundMode) shl 13));
  Result := TFPURoundingMode((CtlWord shr 10) and 3);
end;

function GetPrecisionMode: TFPUPrecisionMode;
begin
  Result := TFPUPrecisionMode((Get8087CW shr 8) and 3);
end;

function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
var
  CtlWord: Word;
begin
  CtlWord := Get8087CW;
  Set8087CW((CtlWord and $FCFF) or (Ord(Precision) shl 8));
  Result := TFPUPrecisionMode((CtlWord shr 8) and 3);
end;

function GetExceptionMask: TFPUExceptionMask;
begin
  Result := TFPUExceptionMask(Longint(Get8087CW and $3F));
end;

function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
var
  CtlWord: Word;
begin
  CtlWord := Get8087CW;
  Set8087CW( (CtlWord and $FFC0) or Byte(Longint(Mask)) );
  if has_sse_support then
    SetMXCSR((GetMXCSR and $ffffe07f) or (dword(Mask) shl 7));
  Result := TFPUExceptionMask(Longint(CtlWord and $3F));
end;

procedure ClearExceptions(RaisePending: Boolean);assembler;
asm
  cmpb $0,RaisePending
  je .Lclear
  fwait
.Lclear:
  fnclex
end;
