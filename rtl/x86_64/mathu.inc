{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2004 by Florian Klaempfl
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}

{$ASMMODE GAS}

{$ifdef FPC_HAS_TYPE_EXTENDED}
{$define FPC_MATH_HAS_ARCTAN2}
function arctan2(y,x : float) : float;assembler;
  asm
     fldt y
     fldt x
     fpatan
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
{$endif FPC_HAS_TYPE_EXTENDED}


{$define FPC_MATH_HAS_SINCOS}
{$ifdef FPC_HAS_TYPE_EXTENDED}
procedure sincos(theta : extended;out sinus,cosinus : extended);assembler;
asm
    fldt theta
    fsincos
{$ifdef WIN64}
    fstpl (%r8)
    fstpl (%rdx)
{$else WIN64}
    fstpt (%rsi)
    fstpt (%rdi)
{$endif WIN64}
    fwait
  end;
{$endif FPC_HAS_TYPE_EXTENDED}


{$asmmode intel}
procedure sincos(theta : double;out sinus,cosinus : double);assembler;
  var
    t : double;
  asm
    movsd qword ptr t,xmm0
    fld qword ptr t
    fsincos
    fstp qword ptr [cosinus]
    fstp qword ptr [sinus]
    fwait
  end;


procedure sincos(theta : single;out sinus,cosinus : single);assembler;
  var
    t : single;
  asm
    movss dword ptr t,xmm0
    fld dword ptr t
    fsincos
    fstp dword ptr [cosinus]
    fstp dword ptr [sinus]
    fwait
  end;


{$define FPC_MATH_HAS_DIVMOD}
{$asmmode intel}
procedure DivMod(Dividend: Integer; Divisor: Word; var Result, Remainder: Word);assembler;
asm
{$ifdef WIN64}
  mov eax, ecx
  movzx ecx, dx
  cdq
  idiv ecx
  mov [r8], ax
  mov [r9], dx
{$else WIN64}
  mov eax, edi
  movzx esi, si
  mov rdi, rdx
  cdq
  idiv esi
  mov [rdi], ax
  mov [rcx], dx
{$endif WIN64}
end;


procedure DivMod(Dividend: Integer; Divisor: Word; var Result, Remainder: SmallInt);assembler;
asm
{$ifdef WIN64}
  mov eax, ecx
  movzx ecx, dx
  cdq
  idiv ecx
  mov [r8], ax
  mov [r9], dx
{$else WIN64}
  mov eax, edi
  movzx esi, si
  mov rdi, rdx
  cdq
  idiv esi
  mov [rdi], ax
  mov [rcx], dx
{$endif WIN64}
end;


procedure DivMod(Dividend: DWord; Divisor: DWord; var Result, Remainder: DWord);assembler;
asm
{$ifdef WIN64}
  mov eax, ecx
  mov ecx, edx
  xor edx, edx
  div ecx
  mov [r8], eax
  mov [r9], edx
{$else WIN64}
  mov eax, edi
  mov rdi, rdx
  xor edx, edx
  div esi
  mov [rdi], eax
  mov [rcx], edx
{$endif WIN64}
end;


procedure DivMod(Dividend: Integer; Divisor: Integer; var Result, Remainder: Integer);assembler;
asm
{$ifdef WIN64}
  mov eax, ecx
  mov ecx, edx
  cdq
  idiv ecx
  mov [r8], eax
  mov [r9], edx
{$else WIN64}
  mov eax, edi
  mov rdi, rdx
  cdq
  idiv esi
  mov [rdi], eax
  mov [rcx], edx
{$endif WIN64}
end;


{$asmmode gas}
function GetRoundMode: TFPURoundingMode;
begin
{$ifndef FPC_HAS_TYPE_EXTENDED}
  Result:=TFPURoundingMode((GetMXCSR shr 13) and $3);
{$else win64}
  Result:=TFPURoundingMode((Get8087CW shr 10) and $3);
{$endif win64}
end;

function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
var
  CtlWord: Word;
  SSECSR: dword;
begin
  CtlWord:=Get8087CW;
  SSECSR:=GetMXCSR;
  softfloat_rounding_mode:=RoundMode;
  Set8087CW((CtlWord and $F3FF) or (Ord(RoundMode) shl 10));
  SetMXCSR((SSECSR and $ffff9fff) or (dword(RoundMode) shl 13));
{$ifdef FPC_HAS_TYPE_EXTENDED}
  Result:=TFPURoundingMode((CtlWord shr 10) and 3);
{$else}
  Result:=TFPURoundingMode((SSECSR shr 13) and 3);
{$endif FPC_HAS_TYPE_EXTENDED}
end;

function GetPrecisionMode: TFPUPrecisionMode;
begin
  Result:=TFPUPrecisionMode((Get8087CW shr 8) and 3);
end;

function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
var
  CtlWord: Word;
begin
  CtlWord:=Get8087CW;
  Set8087CW((CtlWord and $FCFF) or (Ord(Precision) shl 8));
  Result:=TFPUPrecisionMode((CtlWord shr 8) and 3);
end;

function GetExceptionMask: TFPUExceptionMask;
begin
{$ifndef FPC_HAS_TYPE_EXTENDED}
  Result:=TFPUExceptionMask(dword((GetMXCSR shr 7) and $3f));
{$else win64}
  Result:=TFPUExceptionMask(dword(Get8087CW and $3F));
{$endif win64}
end;

function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
var
  CtlWord: Word;
  SSECSR: dword;
begin
  CtlWord:=Get8087CW;
  SSECSR:=GetMXCSR;
  Set8087CW((CtlWord and $FFC0) or Byte(Longint(Mask)));
  SetMXCSR((SSECSR and $ffffe07f) or (dword(Mask) shl 7));
{$ifdef FPC_HAS_TYPE_EXTENDED}
  Result:=TFPUExceptionMask(dword(CtlWord and $3F));
{$else}
  Result:=TFPUExceptionMask((SSECSR shr 7) and $3F);
{$endif FPC_HAS_TYPE_EXTENDED}
end;


procedure ClearExceptions(RaisePending: Boolean);assembler;
asm
  cmpb $0,RaisePending
  je .Lclear
  fwait
.Lclear:
  fnclex
end;
