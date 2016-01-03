{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2000,2001 by the Free Pascal development team

    Interface and OS-dependent part of variant support

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
Procedure VariantTypeMismatch; overload;
begin
  Raise EVariantError.CreateCode(VAR_TYPEMISMATCH);
end;

<<<<<<< HEAD
=======
Procedure VariantTypeMismatch; overload;
=======
Resourcestring

  SNoInterfaces  = 'No interfaces supported';

Procedure NoInterfaces;

>>>>>>> origin/fixes_2.4
begin
  Raise EVariantError.CreateCode(VAR_TYPEMISMATCH);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
Procedure VariantTypeMismatch; overload;
begin
  Raise EVariantError.CreateCode(VAR_TYPEMISMATCH);
end;

>>>>>>> origin/fixes_2_2
Procedure VariantTypeMismatch(const SourceType, DestType: TVarType);
begin
  { ignore the types for now ... }
  Raise EVariantError.CreateCode(VAR_TYPEMISMATCH);
end;


Function ExceptionToVariantError (E : Exception): HResult;

begin
  If E is EoutOfMemory then
    Result:=VAR_OUTOFMEMORY
  else
    Result:=VAR_EXCEPTION;
end;

{ ---------------------------------------------------------------------
    OS-independent functions not present in Windows
  ---------------------------------------------------------------------}

{--- SmallInt ---}

Function WStrToSmallInt(p: Pointer) : SmallInt;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varSmallInt);
end;

Function LStrToSmallInt(p: Pointer) : SmallInt;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varSmallInt);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function UStrToSmallInt(p: Pointer): SmallInt;
var
  Error: Word;
begin
  Val(UnicodeString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varUString, varSmallInt);
end;

Function VariantToSmallInt(const VargSrc : TVarData) : SmallInt;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToSmallInt', VargSrc);
  end; {$ENDIF}

=======
Function VariantToSmallInt(const VargSrc : TVarData) : SmallInt;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToSmallInt', VargSrc);
  end; {$ENDIF}

>>>>>>> graemeg/fixes_2_2
=======
Function VariantToSmallInt(const VargSrc : TVarData) : SmallInt;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToSmallInt', VargSrc);
  end; {$ENDIF}

>>>>>>> origin/fixes_2_2
  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := smallint(vInteger);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
        varSingle   : Result := smallint(Round(vSingle));
        varDouble   : Result := smallint(Round(vDouble));
        varDate     : Result := smallint(Round(vDate));
{$endif}
        varCurrency : Result := smallint(Round(vCurrency));
=======
=======
>>>>>>> origin/fixes_2_2
        varSingle   : Result := smallint(Round(vSingle));
        varDouble   : Result := smallint(Round(vDouble));
        varCurrency : Result := smallint(Round(vCurrency));
        varDate     : Result := smallint(Round(vDate));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varBoolean  : Result := smallint(SmallInt(vBoolean));
        varVariant  : Result := VariantToSmallInt(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := smallint(vWord);
        varLongWord : Result := smallint(vLongWord);
        varInt64    : Result := smallint(vInt64);
        varQword    : Result := smallint(vQWord);
        varOleStr   : Result := WStrToSmallInt(vOleStr);
        varString   : Result := LStrToSmallInt(vString);
<<<<<<< HEAD
<<<<<<< HEAD
        varUString  : Result := UStrToSmallInt(vString);
      else
        VariantTypeMismatch(vType, varSmallInt);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := smallint(PInteger(vPointer)^);
{$ifndef FPUNONE}
        varSingle   : Result := smallint(Round(PSingle(vPointer)^));
        varDouble   : Result := smallint(Round(PDouble(vPointer)^));
        varDate     : Result := smallint(Round(PDate(vPointer)^));
{$endif}
        varCurrency : Result := smallint(Round(PCurrency(vPointer)^));
=======
=======
>>>>>>> origin/fixes_2_2
      else
        VariantTypeMismatch(vType, varSmallInt);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := smallint(PInteger(vPointer)^);
        varSingle   : Result := smallint(Round(PSingle(vPointer)^));
        varDouble   : Result := smallint(Round(PDouble(vPointer)^));
        varCurrency : Result := smallint(Round(PCurrency(vPointer)^));
        varDate     : Result := smallint(Round(PDate(vPointer)^));
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToSmallInt(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := smallint(PWord(vPointer)^);
        varLongWord : Result := smallint(PLongWord(vPointer)^);
        varInt64    : Result := smallint(PInt64(vPointer)^);
        varQword    : Result := smallint(PQWord(vPointer)^);
        varOleStr   : Result := WStrToSmallInt(PPointer(vPointer)^);
        varString   : Result := LStrToSmallInt(PPointer(vPointer)^);
<<<<<<< HEAD
<<<<<<< HEAD
        varUString  : Result := UStrToSmallInt(PPointer(vPointer)^);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      else { other vtype }
        VariantTypeMismatch(vType, varSmallInt);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varSmallInt);
    else { array or something like that }
      VariantTypeMismatch(vType, varSmallInt);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToSmallInt -> ', Result);
  end; {$ENDIF}
end;
<<<<<<< HEAD
<<<<<<< HEAD

{--- ShortInt ---}

Function WStrToShortInt(p: Pointer) : ShortInt;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varShortInt);
end;

Function LStrToShortInt(p: Pointer) : ShortInt;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varShortInt);
end;

Function UStrToShortInt(p: Pointer) : ShortInt;
var
  Error : Word;
begin
  Val(UnicodeString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varUString, varShortInt);
=======

{--- ShortInt ---}

Function WStrToShortInt(p: Pointer) : ShortInt;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varShortInt);
end;

Function LStrToShortInt(p: Pointer) : ShortInt;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varShortInt);
>>>>>>> graemeg/fixes_2_2
end;

Function VariantToShortInt(const VargSrc : TVarData) : ShortInt;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToShortInt', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := shortint(vSmallInt);
        varShortInt : Result := vShortInt;
        varInteger  : Result := shortint(vInteger);
<<<<<<< HEAD
{$ifndef FPUNONE}
        varSingle   : Result := shortint(Round(vSingle));
        varDouble   : Result := shortint(Round(vDouble));
        varDate     : Result := shortint(Round(vDate));
{$endif}
        varCurrency : Result := shortint(Round(vCurrency));
        varBoolean  : Result := shortint(vBoolean);
=======
        varSingle   : Result := shortint(Round(vSingle));
        varDouble   : Result := shortint(Round(vDouble));
        varCurrency : Result := shortint(Round(vCurrency));
        varDate     : Result := shortint(Round(vDate));
        varBoolean  : Result := SmallInt(vBoolean);
>>>>>>> graemeg/fixes_2_2
        varVariant  : Result := VariantToShortInt(PVarData(vPointer)^);
        varByte     : Result := shortint(vByte);
        varWord     : Result := shortint(vWord);
        varLongWord : Result := shortint(vLongWord);
        varInt64    : Result := shortint(vInt64);
        varQword    : Result := shortint(vQWord);
        varOleStr   : Result := WStrToShortInt(vOleStr);
        varString   : Result := LStrToShortInt(vString);
<<<<<<< HEAD
        varUString  : Result := UStrToShortInt(vString);
      else
        VariantTypeMismatch(vType, varShortInt);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := shortint(PSmallInt(vPointer)^);
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := shortint(PInteger(vPointer)^);
{$ifndef FPUNONE}
        varSingle   : Result := shortint(Round(PSingle(vPointer)^));
        varDouble   : Result := shortint(Round(PDouble(vPointer)^));
        varDate     : Result := shortint(Round(PDate(vPointer)^));
{$endif}
        varCurrency : Result := shortint(Round(PCurrency(vPointer)^));
=======
      else
        VariantTypeMismatch(vType, varShortInt);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := shortint(PSmallInt(vPointer)^);
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := shortint(PInteger(vPointer)^);
        varSingle   : Result := shortint(Round(PSingle(vPointer)^));
        varDouble   : Result := shortint(Round(PDouble(vPointer)^));
        varCurrency : Result := shortint(Round(PCurrency(vPointer)^));
        varDate     : Result := shortint(Round(PDate(vPointer)^));
>>>>>>> graemeg/fixes_2_2
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToShortInt(PVarData(vPointer)^);
        varByte     : Result := shortint(PByte(vPointer)^);
        varWord     : Result := shortint(PWord(vPointer)^);
        varLongWord : Result := shortint(PLongWord(vPointer)^);
        varInt64    : Result := shortint(PInt64(vPointer)^);
        varQword    : Result := shortint(PQWord(vPointer)^);
        varOleStr   : Result := WStrToShortInt(PPointer(vPointer)^);
        varString   : Result := LStrToShortInt(PPointer(vPointer)^);
<<<<<<< HEAD
        varUString  : Result := UStrToShortInt(PPointer(vPointer)^);
=======
>>>>>>> graemeg/fixes_2_2
      else { other vtype }
        VariantTypeMismatch(vType, varShortInt);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varShortInt);
    else { array or something like that }
      VariantTypeMismatch(vType, varShortInt);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToShortInt -> ', Result);
  end; {$ENDIF}
end;

{--- LongInt ---}

Function WStrToLongInt(p: Pointer) : LongInt;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varInteger);
end;

Function LStrToLongInt(p: Pointer) : LongInt;
=======

{--- ShortInt ---}

Function WStrToShortInt(p: Pointer) : ShortInt;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varShortInt);
end;

Function LStrToShortInt(p: Pointer) : ShortInt;
>>>>>>> origin/fixes_2_2
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
<<<<<<< HEAD
    VariantTypeMismatch(varString, varInteger);
end;

<<<<<<< HEAD
Function UStrToLongInt(p: Pointer) : LongInt;
var
  Error : Word;
begin
  Val(UnicodeString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varUString, varInteger);
end;

=======
>>>>>>> graemeg/fixes_2_2
Function VariantToLongInt(const VargSrc : TVarData) : LongInt;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToLongInt', VargSrc);
=======
    VariantTypeMismatch(varString, varShortInt);
end;

Function VariantToShortInt(const VargSrc : TVarData) : ShortInt;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToShortInt', VargSrc);
>>>>>>> origin/fixes_2_2
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
<<<<<<< HEAD
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := vInteger;
<<<<<<< HEAD
{$ifndef FPUNONE}
        varSingle   : Result := longint(Round(vSingle));
        varDouble   : Result := longint(Round(vDouble));
        varDate     : Result := longint(Round(vDate));
{$endif}
        varCurrency : Result := longint(Round(vCurrency));
=======
        varSingle   : Result := longint(Round(vSingle));
        varDouble   : Result := longint(Round(vDouble));
        varCurrency : Result := longint(Round(vCurrency));
        varDate     : Result := longint(Round(vDate));
>>>>>>> graemeg/fixes_2_2
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToLongInt(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := longint(vLongWord);
        varInt64    : Result := longint(vInt64);
        varQword    : Result := longint(vQWord);
        varOleStr   : Result := WStrToLongInt(vOleStr);
        varString   : Result := LStrToLongInt(vString);
<<<<<<< HEAD
        varUString  : Result := UStrToLongInt(vString);
      else
        VariantTypeMismatch(vType, varInteger);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
{$ifndef FPUNONE}
        varSingle   : Result := longint(Round(PSingle(vPointer)^));
        varDouble   : Result := longint(Round(PDouble(vPointer)^));
        varDate     : Result := longint(Round(PDate(vPointer)^));
{$endif}
        varCurrency : Result := longint(Round(PCurrency(vPointer)^));
=======
      else
        VariantTypeMismatch(vType, varInteger);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
        varSingle   : Result := longint(Round(PSingle(vPointer)^));
        varDouble   : Result := longint(Round(PDouble(vPointer)^));
        varCurrency : Result := longint(Round(PCurrency(vPointer)^));
        varDate     : Result := longint(Round(PDate(vPointer)^));
>>>>>>> graemeg/fixes_2_2
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToLongInt(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := longint(PLongWord(vPointer)^);
        varInt64    : Result := longint(PInt64(vPointer)^);
        varQword    : Result := longint(PQWord(vPointer)^);
        varOleStr   : Result := WStrToLongInt(PPointer(vPointer)^);
        varString   : Result := LStrToLongInt(PPointer(vPointer)^);
<<<<<<< HEAD
        varUString  : Result := UStrToLongInt(PPointer(vPointer)^);
=======
>>>>>>> graemeg/fixes_2_2
      else { other vtype }
        VariantTypeMismatch(vType, varInteger);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varInteger);
    else { array or something like that }
      VariantTypeMismatch(vType, varInteger);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToLongInt -> ', Result);
  end; {$ENDIF}
end;

{--- Cardinal ---}
<<<<<<< HEAD

Function WStrToCardinal(p: Pointer) : Cardinal;
=======
        varSmallInt : Result := shortint(vSmallInt);
        varShortInt : Result := vShortInt;
        varInteger  : Result := shortint(vInteger);
        varSingle   : Result := shortint(Round(vSingle));
        varDouble   : Result := shortint(Round(vDouble));
        varCurrency : Result := shortint(Round(vCurrency));
        varDate     : Result := shortint(Round(vDate));
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToShortInt(PVarData(vPointer)^);
        varByte     : Result := shortint(vByte);
        varWord     : Result := shortint(vWord);
        varLongWord : Result := shortint(vLongWord);
        varInt64    : Result := shortint(vInt64);
        varQword    : Result := shortint(vQWord);
        varOleStr   : Result := WStrToShortInt(vOleStr);
        varString   : Result := LStrToShortInt(vString);
      else
        VariantTypeMismatch(vType, varShortInt);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := shortint(PSmallInt(vPointer)^);
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := shortint(PInteger(vPointer)^);
        varSingle   : Result := shortint(Round(PSingle(vPointer)^));
        varDouble   : Result := shortint(Round(PDouble(vPointer)^));
        varCurrency : Result := shortint(Round(PCurrency(vPointer)^));
        varDate     : Result := shortint(Round(PDate(vPointer)^));
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToShortInt(PVarData(vPointer)^);
        varByte     : Result := shortint(PByte(vPointer)^);
        varWord     : Result := shortint(PWord(vPointer)^);
        varLongWord : Result := shortint(PLongWord(vPointer)^);
        varInt64    : Result := shortint(PInt64(vPointer)^);
        varQword    : Result := shortint(PQWord(vPointer)^);
        varOleStr   : Result := WStrToShortInt(PPointer(vPointer)^);
        varString   : Result := LStrToShortInt(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varShortInt);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varShortInt);
    else { array or something like that }
      VariantTypeMismatch(vType, varShortInt);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToShortInt -> ', Result);
  end; {$ENDIF}
end;

{--- LongInt ---}

Function WStrToLongInt(p: Pointer) : LongInt;
>>>>>>> origin/fixes_2_2
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
<<<<<<< HEAD
    VariantTypeMismatch(varOleStr, varLongWord);
end;

Function LStrToCardinal(p: Pointer) : Cardinal;
=======
    VariantTypeMismatch(varOleStr, varInteger);
end;

Function LStrToLongInt(p: Pointer) : LongInt;
>>>>>>> origin/fixes_2_2
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
<<<<<<< HEAD
    VariantTypeMismatch(varString, varLongWord);
end;

Function UStrToCardinal(p: Pointer) : Cardinal;
var
  Error : Word;
begin
  Val(UnicodeString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varUString, varLongWord);
end;

Function VariantToCardinal(const VargSrc : TVarData) : Cardinal;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToCardinal', VargSrc);
  end; {$ENDIF}

=======

Function WStrToCardinal(p: Pointer) : Cardinal;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varLongWord);
end;

Function LStrToCardinal(p: Pointer) : Cardinal;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varLongWord);
end;

Function VariantToCardinal(const VargSrc : TVarData) : Cardinal;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToCardinal', VargSrc);
  end; {$ENDIF}

>>>>>>> graemeg/fixes_2_2
=======
    VariantTypeMismatch(varString, varInteger);
end;

Function VariantToLongInt(const VargSrc : TVarData) : LongInt;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToLongInt', VargSrc);
  end; {$ENDIF}

>>>>>>> origin/fixes_2_2
  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
<<<<<<< HEAD
        varInteger  : Result := cardinal(vInteger);
<<<<<<< HEAD
{$ifndef FPUNONE}
        varSingle   : Result := cardinal(Round(vSingle));
        varDouble   : Result := cardinal(Round(vDouble));
        varDate     : Result := cardinal(Round(vDate));
{$endif}
        varCurrency : Result := cardinal(Round(vCurrency));
        varBoolean  : Result := cardinal(SmallInt(vBoolean));
=======
        varSingle   : Result := cardinal(Round(vSingle));
        varDouble   : Result := cardinal(Round(vDouble));
        varCurrency : Result := cardinal(Round(vCurrency));
        varDate     : Result := cardinal(Round(vDate));
        varBoolean  : Result := SmallInt(vBoolean);
>>>>>>> graemeg/fixes_2_2
        varVariant  : Result := VariantToCardinal(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := cardinal(vInt64);
        varQword    : Result := cardinal(vQWord);
        varOleStr   : Result := WStrToCardinal(vOleStr);
        varString   : Result := LStrToCardinal(vString);
<<<<<<< HEAD
        varUString  : Result := UStrToCardinal(vString);
      else
        VariantTypeMismatch(vType, varLongWord);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := cardinal(PSmallInt(vPointer)^);
        varShortInt : Result := cardinal(PShortInt(vPointer)^);
        varInteger  : Result := cardinal(PInteger(vPointer)^);
{$ifndef FPUNONE}
        varSingle   : Result := cardinal(Round(PSingle(vPointer)^));
        varDouble   : Result := cardinal(Round(PDouble(vPointer)^));
        varDate     : Result := cardinal(Round(PDate(vPointer)^));
{$endif}
        varCurrency : Result := cardinal(Round(PCurrency(vPointer)^));
        varBoolean  : Result := cardinal(SmallInt(PWordBool(vPointer)^));
=======
      else
        VariantTypeMismatch(vType, varLongWord);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := cardinal(PSmallInt(vPointer)^);
        varShortInt : Result := cardinal(PShortInt(vPointer)^);
        varInteger  : Result := cardinal(PInteger(vPointer)^);
        varSingle   : Result := cardinal(Round(PSingle(vPointer)^));
        varDouble   : Result := cardinal(Round(PDouble(vPointer)^));
        varCurrency : Result := cardinal(Round(PCurrency(vPointer)^));
        varDate     : Result := cardinal(Round(PDate(vPointer)^));
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
>>>>>>> graemeg/fixes_2_2
        varVariant  : Result := VariantToCardinal(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := cardinal(PInt64(vPointer)^);
        varQword    : Result := cardinal(PQWord(vPointer)^);
        varOleStr   : Result := WStrToCardinal(PPointer(vPointer)^);
        varString   : Result := LStrToCardinal(PPointer(vPointer)^);
<<<<<<< HEAD
        varUString  : Result := UStrToCardinal(PPointer(vPointer)^);
=======
>>>>>>> graemeg/fixes_2_2
      else { other vtype }
        VariantTypeMismatch(vType, varLongWord);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varLongWord);
    else { array or something like that }
      VariantTypeMismatch(vType, varLongWord);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToCardinal -> ', Result);
  end; {$ENDIF}
end;

procedure PrepareFloatStr(var s: ShortString);
var
  i, j  : Byte;
begin
  j := 1;
  for i := 1 to Length(s) do
<<<<<<< HEAD
    if s[i] <> DefaultFormatSettings.ThousandSeparator then begin
      if s[i] = DefaultFormatSettings.DecimalSeparator then
=======
    if s[i] <> ThousandSeparator then begin
      if s[i] = DecimalSeparator then
>>>>>>> graemeg/fixes_2_2
        s[j] := '.'
      else
        s[j] := s[i];
      Inc(j);
    end;
  SetLength(s, Pred(j));
end;

{--- Single ---}

<<<<<<< HEAD
{$ifndef FPUNONE}

=======
>>>>>>> graemeg/fixes_2_2
Function WStrToSingle(p: Pointer) : Single;
var
  s     : ShortString;
  Error : Word;
begin
  if Length(WideString(p)) > 255 then
    VariantTypeMismatch(varOleStr, varSingle);

  s := WideString(p);
  PrepareFloatStr(s);

  Val(s, Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varSingle);
end;

Function LStrToSingle(p: Pointer) : Single;
var
  s     : ShortString;
  Error : Word;
begin
  if Length(AnsiString(p)) > 255 then
    VariantTypeMismatch(varString, varSingle);

  s := AnsiString(p);
  PrepareFloatStr(s);

  Val(s, Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varSingle);
end;

<<<<<<< HEAD
Function UStrToSingle(p: Pointer) : Single;
var
  s     : ShortString;
  Error : Word;
begin
  if Length(UnicodeString(p)) > 255 then
    VariantTypeMismatch(varUString, varSingle);

  s := UnicodeString(p);
  PrepareFloatStr(s);

  Val(s, Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varUString, varSingle);
end;


=======
>>>>>>> graemeg/fixes_2_2
Function VariantToSingle(const VargSrc : TVarData) : Single;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToSingle', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := vInteger;
        varSingle   : Result := vSingle;
        varDouble   : Result := vDouble;
        varCurrency : Result := vCurrency;
        varDate     : Result := vDate;
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToSingle(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := vInt64;
        varQword    : Result := vQWord;
        varOleStr   : Result := WStrToSingle(vOleStr);
        varString   : Result := LStrToSingle(vString);
<<<<<<< HEAD
        varUString  : Result := UStrToSingle(vString);
      else
        VariantTypeMismatch(vType, varSingle);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
=======
      else
        VariantTypeMismatch(vType, varSingle);
      end;
<<<<<<< HEAD
      varByRef: if Assigned(vPointer) then case vType of
>>>>>>> graemeg/fixes_2_2
=======
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
>>>>>>> origin/fixes_2.4
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
        varSingle   : Result := PSingle(vPointer)^;
        varDouble   : Result := PDouble(vPointer)^;
        varCurrency : Result := PCurrency(vPointer)^;
        varDate     : Result := PDate(vPointer)^;
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToSingle(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := PInt64(vPointer)^;
        varQword    : Result := PQWord(vPointer)^;
        varOleStr   : Result := WStrToSingle(PPointer(vPointer)^);
        varString   : Result := LStrToSingle(PPointer(vPointer)^);
<<<<<<< HEAD
        varUString  : Result := UStrToSingle(PPointer(vPointer)^);
=======
>>>>>>> graemeg/fixes_2_2
      else { other vtype }
        VariantTypeMismatch(vType, varSingle);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varSingle);
    else { array or something like that }
      VariantTypeMismatch(vType, varSingle);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToSingle -> ', Result);
  end; {$ENDIF}
end;

{--- Double ---}

Function WStrToDouble(p: Pointer) : Double;
=======
        varInteger  : Result := vInteger;
        varSingle   : Result := longint(Round(vSingle));
        varDouble   : Result := longint(Round(vDouble));
        varCurrency : Result := longint(Round(vCurrency));
        varDate     : Result := longint(Round(vDate));
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToLongInt(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := longint(vLongWord);
        varInt64    : Result := longint(vInt64);
        varQword    : Result := longint(vQWord);
        varOleStr   : Result := WStrToLongInt(vOleStr);
        varString   : Result := LStrToLongInt(vString);
      else
        VariantTypeMismatch(vType, varInteger);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
        varSingle   : Result := longint(Round(PSingle(vPointer)^));
        varDouble   : Result := longint(Round(PDouble(vPointer)^));
        varCurrency : Result := longint(Round(PCurrency(vPointer)^));
        varDate     : Result := longint(Round(PDate(vPointer)^));
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToLongInt(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := longint(PLongWord(vPointer)^);
        varInt64    : Result := longint(PInt64(vPointer)^);
        varQword    : Result := longint(PQWord(vPointer)^);
        varOleStr   : Result := WStrToLongInt(PPointer(vPointer)^);
        varString   : Result := LStrToLongInt(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varInteger);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varInteger);
    else { array or something like that }
      VariantTypeMismatch(vType, varInteger);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToLongInt -> ', Result);
  end; {$ENDIF}
end;

{--- Cardinal ---}

Function WStrToCardinal(p: Pointer) : Cardinal;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varLongWord);
end;

Function LStrToCardinal(p: Pointer) : Cardinal;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varLongWord);
end;

Function VariantToCardinal(const VargSrc : TVarData) : Cardinal;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToCardinal', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := cardinal(vInteger);
        varSingle   : Result := cardinal(Round(vSingle));
        varDouble   : Result := cardinal(Round(vDouble));
        varCurrency : Result := cardinal(Round(vCurrency));
        varDate     : Result := cardinal(Round(vDate));
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToCardinal(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := cardinal(vInt64);
        varQword    : Result := cardinal(vQWord);
        varOleStr   : Result := WStrToCardinal(vOleStr);
        varString   : Result := LStrToCardinal(vString);
      else
        VariantTypeMismatch(vType, varLongWord);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := cardinal(PSmallInt(vPointer)^);
        varShortInt : Result := cardinal(PShortInt(vPointer)^);
        varInteger  : Result := cardinal(PInteger(vPointer)^);
        varSingle   : Result := cardinal(Round(PSingle(vPointer)^));
        varDouble   : Result := cardinal(Round(PDouble(vPointer)^));
        varCurrency : Result := cardinal(Round(PCurrency(vPointer)^));
        varDate     : Result := cardinal(Round(PDate(vPointer)^));
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToCardinal(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := cardinal(PInt64(vPointer)^);
        varQword    : Result := cardinal(PQWord(vPointer)^);
        varOleStr   : Result := WStrToCardinal(PPointer(vPointer)^);
        varString   : Result := LStrToCardinal(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varLongWord);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varLongWord);
    else { array or something like that }
      VariantTypeMismatch(vType, varLongWord);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToCardinal -> ', Result);
  end; {$ENDIF}
end;

procedure PrepareFloatStr(var s: ShortString);
var
  i, j  : Byte;
begin
  j := 1;
  for i := 1 to Length(s) do
    if s[i] <> ThousandSeparator then begin
      if s[i] = DecimalSeparator then
        s[j] := '.'
      else
        s[j] := s[i];
      Inc(j);
    end;
  SetLength(s, Pred(j));
end;

{--- Single ---}

Function WStrToSingle(p: Pointer) : Single;
>>>>>>> origin/fixes_2_2
var
  s     : ShortString;
  Error : Word;
begin
  if Length(WideString(p)) > 255 then
<<<<<<< HEAD
    VariantTypeMismatch(varOleStr, varDouble);
=======
    VariantTypeMismatch(varOleStr, varSingle);
>>>>>>> origin/fixes_2_2

  s := WideString(p);
  PrepareFloatStr(s);

  Val(s, Result, Error);
  if Error <> 0 then
<<<<<<< HEAD
    VariantTypeMismatch(varOleStr, varDouble);
<<<<<<< HEAD
end;

Function LStrToDouble(p: Pointer) : Double;
=======
    VariantTypeMismatch(varOleStr, varSingle);
end;

Function LStrToSingle(p: Pointer) : Single;
>>>>>>> origin/fixes_2_2
var
  s     : ShortString;
  Error : Word;
begin
  if Length(AnsiString(p)) > 255 then
<<<<<<< HEAD
    VariantTypeMismatch(varString, varDouble);
=======
    VariantTypeMismatch(varString, varSingle);
>>>>>>> origin/fixes_2_2

  s := AnsiString(p);
  PrepareFloatStr(s);

  Val(s, Result, Error);
  if Error <> 0 then
<<<<<<< HEAD
    VariantTypeMismatch(varString, varDouble);
end;

Function UStrToDouble(p: Pointer) : Double;
var
  s     : ShortString;
  Error : Word;
begin
  if Length(UnicodeString(p)) > 255 then
    VariantTypeMismatch(varUString, varDouble);

  s := UnicodeString(p);
  PrepareFloatStr(s);

  Val(s, Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varUString, varDouble);
end;

Function VariantToDouble(const VargSrc : TVarData) : Double;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToDouble', VargSrc);
=======
    VariantTypeMismatch(varString, varSingle);
end;

Function VariantToSingle(const VargSrc : TVarData) : Single;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToSingle', VargSrc);
>>>>>>> origin/fixes_2_2
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := vInteger;
        varSingle   : Result := vSingle;
        varDouble   : Result := vDouble;
        varCurrency : Result := vCurrency;
        varDate     : Result := vDate;
        varBoolean  : Result := SmallInt(vBoolean);
<<<<<<< HEAD
        varVariant  : Result := VariantToDouble(PVarData(vPointer)^);
=======
        varVariant  : Result := VariantToSingle(PVarData(vPointer)^);
>>>>>>> origin/fixes_2_2
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := vInt64;
        varQword    : Result := vQWord;
<<<<<<< HEAD
        varOleStr   : Result := WStrToDouble(vOleStr);
        varString   : Result := LStrToDouble(vString);
        varUString  : Result := UStrToDouble(vString);
      else
        VariantTypeMismatch(vType, varDouble);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
<<<<<<< HEAD
=======
        varOleStr   : Result := WStrToSingle(vOleStr);
        varString   : Result := LStrToSingle(vString);
      else
        VariantTypeMismatch(vType, varSingle);
      end;
      varByRef: if Assigned(vPointer) then case vType of
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
        varSingle   : Result := PSingle(vPointer)^;
        varDouble   : Result := PDouble(vPointer)^;
        varCurrency : Result := PCurrency(vPointer)^;
        varDate     : Result := PDate(vPointer)^;
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
<<<<<<< HEAD
        varVariant  : Result := VariantToDouble(PVarData(vPointer)^);
=======
        varVariant  : Result := VariantToSingle(PVarData(vPointer)^);
>>>>>>> origin/fixes_2_2
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := PInt64(vPointer)^;
        varQword    : Result := PQWord(vPointer)^;
<<<<<<< HEAD
        varOleStr   : Result := WStrToDouble(PPointer(vPointer)^);
        varString   : Result := LStrToDouble(PPointer(vPointer)^);
        varUString  : Result := UStrToDouble(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varDouble);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varDouble);
    else { array or something like that }
      VariantTypeMismatch(vType, varDouble);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToDouble -> ', Result);
  end; {$ENDIF}
end;

{$endif FPUNONE}

=======
=======
        varOleStr   : Result := WStrToSingle(PPointer(vPointer)^);
        varString   : Result := LStrToSingle(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varSingle);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varSingle);
    else { array or something like that }
      VariantTypeMismatch(vType, varSingle);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToSingle -> ', Result);
  end; {$ENDIF}
end;

{--- Double ---}

Function WStrToDouble(p: Pointer) : Double;
var
  s     : ShortString;
  Error : Word;
begin
  if Length(WideString(p)) > 255 then
    VariantTypeMismatch(varOleStr, varDouble);

  s := WideString(p);
  PrepareFloatStr(s);

  Val(s, Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varDouble);
>>>>>>> origin/fixes_2_2
end;

Function LStrToDouble(p: Pointer) : Double;
var
  s     : ShortString;
  Error : Word;
begin
  if Length(AnsiString(p)) > 255 then
    VariantTypeMismatch(varString, varDouble);

  s := AnsiString(p);
  PrepareFloatStr(s);

  Val(s, Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varDouble);
end;

Function VariantToDouble(const VargSrc : TVarData) : Double;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToDouble', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := vInteger;
        varSingle   : Result := vSingle;
        varDouble   : Result := vDouble;
        varCurrency : Result := vCurrency;
        varDate     : Result := vDate;
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToDouble(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := vInt64;
        varQword    : Result := vQWord;
        varOleStr   : Result := WStrToDouble(vOleStr);
        varString   : Result := LStrToDouble(vString);
      else
        VariantTypeMismatch(vType, varDouble);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
        varSingle   : Result := PSingle(vPointer)^;
        varDouble   : Result := PDouble(vPointer)^;
        varCurrency : Result := PCurrency(vPointer)^;
        varDate     : Result := PDate(vPointer)^;
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToDouble(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := PInt64(vPointer)^;
        varQword    : Result := PQWord(vPointer)^;
        varOleStr   : Result := WStrToDouble(PPointer(vPointer)^);
        varString   : Result := LStrToDouble(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varDouble);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varDouble);
    else { array or something like that }
      VariantTypeMismatch(vType, varDouble);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToDouble -> ', Result);
  end; {$ENDIF}
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{--- Currency ---}

Function WStrToCurrency(p: Pointer) : Currency;
var
  s     : ShortString;
  Error : Word;
  {$IFNDEF FPC_HAS_STR_CURRENCY}
  Temp  : Extended;
  {$ENDIF FPC_HAS_STR_CURRENCY}
begin
  if Length(WideString(p)) > 255 then
    VariantTypeMismatch(varOleStr, varCurrency);

  s := WideString(p);
  PrepareFloatStr(s);

  {$IFDEF FPC_HAS_STR_CURRENCY}
  Val(s, Result, Error);
  {$ELSE FPC_HAS_STR_CURRENCY} { needed for platforms where Currency = Int64 }
  Val(s, Temp, Error);
  Result := Temp;
  {$ENDIF FPC_HAS_STR_CURRENCY}

  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varCurrency);
end;

Function LStrToCurrency(p: Pointer) : Currency;
var
  s     : ShortString;
  Error : Word;
  {$IFNDEF FPC_HAS_STR_CURRENCY}
  Temp  : Extended;
  {$ENDIF FPC_HAS_STR_CURRENCY}
begin
  if Length(AnsiString(p)) > 255 then
    VariantTypeMismatch(varString, varCurrency);

  s := AnsiString(p);
  PrepareFloatStr(s);

  {$IFDEF FPC_HAS_STR_CURRENCY}
  Val(s, Result, Error);
  {$ELSE FPC_HAS_STR_CURRENCY} { needed for platforms where Currency = Int64 }
  Val(s, Temp, Error);
  Result := Temp;
  {$ENDIF FPC_HAS_STR_CURRENCY}

  if Error <> 0 then
    VariantTypeMismatch(varString, varCurrency);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function UStrToCurrency(p: Pointer) : Currency;
var
  s     : ShortString;
  Error : Word;
  {$IFNDEF FPC_HAS_STR_CURRENCY}
  Temp  : Extended;
  {$ENDIF FPC_HAS_STR_CURRENCY}
begin
  if Length(UnicodeString(p)) > 255 then
    VariantTypeMismatch(varUString, varCurrency);

  s := UnicodeString(p);
  PrepareFloatStr(s);

  {$IFDEF FPC_HAS_STR_CURRENCY}
  Val(s, Result, Error);
  {$ELSE FPC_HAS_STR_CURRENCY} { needed for platforms where Currency = Int64 }
  Val(s, Temp, Error);
  Result := Temp;
  {$ENDIF FPC_HAS_STR_CURRENCY}

  if Error <> 0 then
    VariantTypeMismatch(varUString, varCurrency);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function VariantToCurrency(const VargSrc : TVarData) : Currency;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToCurrency', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := vInteger;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varSingle   : begin
          if (vSingle > MaxCurrency) or (vSingle < MinCurrency) then
            VariantTypeMismatch(vType, varCurrency);
          Result := vSingle;
        end;
        varDouble   : begin
          if (vDouble > MaxCurrency) or (vDouble < MinCurrency) then
            VariantTypeMismatch(vType, varCurrency);
          Result := vDouble;
        end;
<<<<<<< HEAD
<<<<<<< HEAD
        varDate     : begin
          if (vDate > MaxCurrency) or (vDate < MinCurrency) then
            VariantTypeMismatch(vType, varCurrency);
          Result := vDate;
        end;
{$endif}
        varCurrency : Result := vCurrency;
=======
=======
>>>>>>> origin/fixes_2_2
        varCurrency : Result := vCurrency;
        varDate     : begin
          if (varDate > MaxCurrency) or (varDate < MinCurrency) then
            VariantTypeMismatch(vType, varCurrency);
          Result := vDate;
        end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToCurrency(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := vInt64;
        varQword    : Result := currency(vQWord);
        varOleStr   : Result := WStrToCurrency(vOleStr);
        varString   : Result := LStrToCurrency(vString);
<<<<<<< HEAD
<<<<<<< HEAD
        varUString  : Result := UStrToCurrency(vString);
      else
        VariantTypeMismatch(vType, varCurrency);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
{$ifndef FPUNONE}
=======
=======
>>>>>>> origin/fixes_2_2
      else
        VariantTypeMismatch(vType, varCurrency);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varSingle   : begin
          if (PSingle(vPointer)^ > MaxCurrency) or (PSingle(vPointer)^ < MinCurrency) then
            VariantTypeMismatch(vType, varCurrency);
          Result := PSingle(vPointer)^;
        end;
        varDouble   : begin
          if (PDouble(vPointer)^ > MaxCurrency) or (PDouble(vPointer)^ < MinCurrency) then
            VariantTypeMismatch(vType, varCurrency);
          Result := PDouble(vPointer)^;
        end;
<<<<<<< HEAD
<<<<<<< HEAD
=======
        varCurrency : Result := PCurrency(vPointer)^;
>>>>>>> graemeg/fixes_2_2
=======
        varCurrency : Result := PCurrency(vPointer)^;
>>>>>>> origin/fixes_2_2
        varDate     : begin
          if (PDate(vPointer)^ > MaxCurrency) or (PDate(vPointer)^ < MinCurrency) then
            VariantTypeMismatch(vType, varCurrency);
          Result := PDate(vPointer)^;
        end;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
        varCurrency : Result := PCurrency(vPointer)^;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToCurrency(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := PInt64(vPointer)^;
        varQword    : Result := currency(PQWord(vPointer)^);
        varOleStr   : Result := WStrToCurrency(PPointer(vPointer)^);
        varString   : Result := LStrToCurrency(PPointer(vPointer)^);
<<<<<<< HEAD
<<<<<<< HEAD
        varUString  : Result := UStrToCurrency(PPointer(vPointer)^);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      else { other vtype }
        VariantTypeMismatch(vType, varCurrency);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varCurrency);
    else { array or something like that }
      VariantTypeMismatch(vType, varCurrency);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToCurrency -> ', Result);
  end; {$ENDIF}
end;

{--- Date ---}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function WStrToDate(p: Pointer) : TDateTime;
var
  s: string;
begin
  s := WideString(p);

<<<<<<< HEAD
<<<<<<< HEAD
  if not TryStrToDateTime(s, Result) then
=======
  if not (TryStrToDateTime(s, Result) or
    TryStrToDate(s, Result) or
    TryStrToTime(s, Result)) then
>>>>>>> graemeg/fixes_2_2
=======
  if not (TryStrToDateTime(s, Result) or
    TryStrToDate(s, Result) or
    TryStrToTime(s, Result)) then
>>>>>>> origin/fixes_2_2
    VariantTypeMismatch(varOleStr, varDate);
end;

Function LStrToDate(p: Pointer) : TDateTime;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  if not TryStrToDateTime(AnsiString(p), Result) then
    VariantTypeMismatch(varString, varDate);
end;

Function UStrToDate(p: Pointer) : TDateTime;
begin
  if not TryStrToDateTime(UnicodeString(p), Result) then
    VariantTypeMismatch(varUString, varDate);
end;

=======
=======
>>>>>>> origin/fixes_2_2
  if not (TryStrToDateTime(AnsiString(p), Result) or
    TryStrToDate(AnsiString(p), Result) or
    TryStrToTime(AnsiString(p), Result)) then
    VariantTypeMismatch(varString, varDate);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function VariantToDate(const VargSrc : TVarData) : TDateTime;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToDate', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := vInteger;
        varSingle   : Result := vSingle;
        varDouble   : Result := vDouble;
        varCurrency : Result := vCurrency;
        varDate     : Result := vDate;
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToDate(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := vInt64;
        varQword    : Result := vQWord;
        varOleStr   : Result := WStrToDate(vOleStr);
        varString   : Result := LStrToDate(vString);
<<<<<<< HEAD
<<<<<<< HEAD
        varUString  : Result := UStrToDate(vString);
      else
        VariantTypeMismatch(vType, varDate);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
=======
=======
>>>>>>> origin/fixes_2_2
      else
        VariantTypeMismatch(vType, varDate);
      end;
<<<<<<< HEAD
      varByRef: if Assigned(vPointer) then case vType of
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
>>>>>>> origin/fixes_2.4
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
        varSingle   : Result := PSingle(vPointer)^;
        varDouble   : Result := PDouble(vPointer)^;
        varCurrency : Result := PCurrency(vPointer)^;
        varDate     : Result := PDate(vPointer)^;
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToDate(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := PInt64(vPointer)^;
        varQword    : Result := PQWord(vPointer)^;
        varOleStr   : Result := WStrToDate(PPointer(vPointer)^);
        varString   : Result := LStrToDate(PPointer(vPointer)^);
<<<<<<< HEAD
<<<<<<< HEAD
        varUString  : Result := UStrToDate(PPointer(vPointer)^);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      else { other vtype }
        VariantTypeMismatch(vType, varDate);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varDate);
    else { array or something like that }
      VariantTypeMismatch(vType, varDate);
    end;
  if (Result < MinDateTime) or (Result > MaxDateTime) then
    VariantTypeMismatch(VargSrc.vType, varDate);

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToDate -> ', Result);
  end; {$ENDIF}
end;

<<<<<<< HEAD
<<<<<<< HEAD
{$endif}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{--- Boolean ---}

Function WStrToBoolean(p: Pointer) : Boolean;
begin
  if not TryStrToBool(WideString(p), Result) then
    VariantTypeMismatch(varOleStr, varBoolean);
end;

Function LStrToBoolean(p: Pointer) : Boolean;
begin
  if not TryStrToBool(AnsiString(p), Result) then
    VariantTypeMismatch(varString, varBoolean);
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function UStrToBoolean(p: Pointer) : Boolean;
begin
  if not TryStrToBool(UnicodeString(p), Result) then
    VariantTypeMismatch(varUString, varBoolean);
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function VariantToBoolean(const VargSrc : TVarData) : Boolean;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToBoolean', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := False;
        varSmallInt : Result := vSmallInt <> 0;
        varShortInt : Result := vShortInt <> 0;
        varInteger  : Result := vInteger <> 0;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varSingle   : Result := vSingle <> 0;
        varDouble   : Result := vDouble <> 0;
        varCurrency : Result := vCurrency <> 0;
        varDate     : Result := vDate <> 0;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varBoolean  : Result := vBoolean;
        varVariant  : Result := VariantToBoolean(PVarData(vPointer)^);
        varByte     : Result := vByte <> 0;
        varWord     : Result := vWord <> 0;
        varLongWord : Result := vLongWord <> 0;
        varInt64    : Result := vInt64 <> 0;
        varQword    : Result := vQWord <> 0;
        varOleStr   : Result := WStrToBoolean(vOleStr);
        varString   : Result := LStrToBoolean(vString);
<<<<<<< HEAD
<<<<<<< HEAD
        varUString  : Result := UStrToBoolean(vString);
      else
        VariantTypeMismatch(vType, varBoolean);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := PSmallInt(vPointer)^ <> 0;
        varShortInt : Result := PShortInt(vPointer)^ <> 0;
        varInteger  : Result := PInteger(vPointer)^ <> 0;
{$ifndef FPUNONE}
=======
=======
>>>>>>> origin/fixes_2_2
      else
        VariantTypeMismatch(vType, varBoolean);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := PSmallInt(vPointer)^ <> 0;
        varShortInt : Result := PShortInt(vPointer)^ <> 0;
        varInteger  : Result := PInteger(vPointer)^ <> 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varSingle   : Result := PSingle(vPointer)^ <> 0;
        varDouble   : Result := PDouble(vPointer)^ <> 0;
        varCurrency : Result := PCurrency(vPointer)^ <> 0;
        varDate     : Result := PDate(vPointer)^ <> 0;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^) <> 0;
        varVariant  : Result := VariantToBoolean(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^ <> 0;
        varWord     : Result := PWord(vPointer)^ <> 0;
        varLongWord : Result := PLongWord(vPointer)^ <> 0;
        varInt64    : Result := PInt64(vPointer)^ <> 0;
        varQword    : Result := PQWord(vPointer)^ <> 0;
        varOleStr   : Result := WStrToBoolean(PPointer(vPointer)^);
        varString   : Result := LStrToBoolean(PPointer(vPointer)^);
<<<<<<< HEAD
<<<<<<< HEAD
        varUString  : Result := UStrToBoolean(PPointer(vPointer)^);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      else { other vtype }
        VariantTypeMismatch(vType, varBoolean);
      end else { pointer is nil }
        Result := False;
    else { array or something like that }
      VariantTypeMismatch(vType, varBoolean);
<<<<<<< HEAD
<<<<<<< HEAD
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToBoolean -> ', Result);
  end; {$ENDIF}
end;

{--- Byte ---}

Function WStrToByte(p: Pointer) : Byte;
=======
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToBoolean -> ', Result);
  end; {$ENDIF}
end;

{--- Byte ---}

Function WStrToByte(p: Pointer) : Byte;
=======
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToBoolean -> ', Result);
  end; {$ENDIF}
end;

{--- Byte ---}

Function WStrToByte(p: Pointer) : Byte;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varByte);
end;

Function LStrToByte(p: Pointer) : Byte;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varByte);
end;

Function VariantToByte(const VargSrc : TVarData) : Byte;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToByte', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := byte(vSmallInt);
        varShortInt : Result := byte(vShortInt);
        varInteger  : Result := byte(vInteger);
        varSingle   : Result := byte(Round(vSingle));
        varDouble   : Result := byte(Round(vDouble));
        varCurrency : Result := byte(Round(vCurrency));
        varDate     : Result := byte(Round(vDate));
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToByte(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := byte(vWord);
        varLongWord : Result := byte(vLongWord);
        varInt64    : Result := byte(vInt64);
        varQword    : Result := byte(vQWord);
        varOleStr   : Result := WStrToByte(vOleStr);
        varString   : Result := LStrToByte(vString);
      else
        VariantTypeMismatch(vType, varByte);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := byte(PSmallInt(vPointer)^);
        varShortInt : Result := byte(PShortInt(vPointer)^);
        varInteger  : Result := byte(PInteger(vPointer)^);
        varSingle   : Result := byte(Round(PSingle(vPointer)^));
        varDouble   : Result := byte(Round(PDouble(vPointer)^));
        varCurrency : Result := byte(Round(PCurrency(vPointer)^));
        varDate     : Result := byte(Round(PDate(vPointer)^));
        varBoolean  : Result := byte(SmallInt(PWordBool(vPointer)^));
        varVariant  : Result := byte(VariantToByte(PVarData(vPointer)^));
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := byte(PWord(vPointer)^);
        varLongWord : Result := byte(PLongWord(vPointer)^);
        varInt64    : Result := byte(PInt64(vPointer)^);
        varQword    : Result := byte(PQWord(vPointer)^);
        varOleStr   : Result := WStrToByte(PPointer(vPointer)^);
        varString   : Result := LStrToByte(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varByte);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varByte);
    else { array or something like that }
      VariantTypeMismatch(vType, varByte);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToByte -> ', Result);
  end; {$ENDIF}
end;

{--- Int64 ---}

Function WStrToInt64(p: Pointer) : Int64;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varInt64);
end;

Function LStrToInt64(p: Pointer) : Int64;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varInt64);
end;

Function VariantToInt64(const VargSrc : TVarData) : Int64;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToInt64', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := vInteger;
        varSingle   : Result := Round(vSingle);
        varDouble   : Result := Round(vDouble);
        varCurrency : Result := Round(vCurrency);
        varDate     : Result := Round(vDate);
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToInt64(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := vInt64;
        varQword    : Result := int64(vQWord);
        varOleStr   : Result := WStrToInt64(vOleStr);
        varString   : Result := LStrToInt64(vString);
      else
        VariantTypeMismatch(vType, varInt64);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
        varSingle   : Result := Round(PSingle(vPointer)^);
        varDouble   : Result := Round(PDouble(vPointer)^);
        varCurrency : Result := Round(PCurrency(vPointer)^);
        varDate     : Result := Round(PDate(vPointer)^);
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToInt64(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := PInt64(vPointer)^;
        varQword    : Result := PQWord(vPointer)^;
        varOleStr   : Result := WStrToInt64(PPointer(vPointer)^);
        varString   : Result := LStrToInt64(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varInt64);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varInt64);
    else { array or something like that }
      VariantTypeMismatch(vType, varInt64);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToInt64 -> ', Result);
  end; {$ENDIF}
end;

{--- QWord ---}

Function WStrToQWord(p: Pointer) : QWord;
>>>>>>> origin/fixes_2_2
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
<<<<<<< HEAD
    VariantTypeMismatch(varOleStr, varByte);
end;

Function LStrToByte(p: Pointer) : Byte;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varByte);
end;

Function VariantToByte(const VargSrc : TVarData) : Byte;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToByte', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := byte(vSmallInt);
        varShortInt : Result := byte(vShortInt);
        varInteger  : Result := byte(vInteger);
        varSingle   : Result := byte(Round(vSingle));
        varDouble   : Result := byte(Round(vDouble));
        varCurrency : Result := byte(Round(vCurrency));
        varDate     : Result := byte(Round(vDate));
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToByte(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := byte(vWord);
        varLongWord : Result := byte(vLongWord);
        varInt64    : Result := byte(vInt64);
        varQword    : Result := byte(vQWord);
        varOleStr   : Result := WStrToByte(vOleStr);
        varString   : Result := LStrToByte(vString);
      else
        VariantTypeMismatch(vType, varByte);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := byte(PSmallInt(vPointer)^);
        varShortInt : Result := byte(PShortInt(vPointer)^);
        varInteger  : Result := byte(PInteger(vPointer)^);
        varSingle   : Result := byte(Round(PSingle(vPointer)^));
        varDouble   : Result := byte(Round(PDouble(vPointer)^));
        varCurrency : Result := byte(Round(PCurrency(vPointer)^));
        varDate     : Result := byte(Round(PDate(vPointer)^));
        varBoolean  : Result := byte(SmallInt(PWordBool(vPointer)^));
        varVariant  : Result := byte(VariantToByte(PVarData(vPointer)^));
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := byte(PWord(vPointer)^);
        varLongWord : Result := byte(PLongWord(vPointer)^);
        varInt64    : Result := byte(PInt64(vPointer)^);
        varQword    : Result := byte(PQWord(vPointer)^);
        varOleStr   : Result := WStrToByte(PPointer(vPointer)^);
        varString   : Result := LStrToByte(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varByte);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varByte);
    else { array or something like that }
      VariantTypeMismatch(vType, varByte);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToByte -> ', Result);
  end; {$ENDIF}
end;

{--- Int64 ---}

Function WStrToInt64(p: Pointer) : Int64;
>>>>>>> graemeg/fixes_2_2
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
<<<<<<< HEAD
    VariantTypeMismatch(varOleStr, varByte);
end;

Function LStrToByte(p: Pointer) : Byte;
=======
    VariantTypeMismatch(varOleStr, varInt64);
end;

Function LStrToInt64(p: Pointer) : Int64;
>>>>>>> graemeg/fixes_2_2
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
<<<<<<< HEAD
    VariantTypeMismatch(varString, varByte);
end;

Function UStrToByte(p: Pointer) : Byte;
var
  Error : Word;
begin
  Val(UnicodeString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varUString, varByte);
end;

Function VariantToByte(const VargSrc : TVarData) : Byte;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToByte', VargSrc);
=======
    VariantTypeMismatch(varString, varInt64);
end;

Function VariantToInt64(const VargSrc : TVarData) : Int64;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToInt64', VargSrc);
>>>>>>> graemeg/fixes_2_2
=======
    VariantTypeMismatch(varOleStr, varQWord);
end;

Function LStrToQWord(p: Pointer) : QWord;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varQWord);
end;

Function VariantToQWord(const VargSrc : TVarData) : QWord;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToQWord', VargSrc);
>>>>>>> origin/fixes_2_2
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
<<<<<<< HEAD
<<<<<<< HEAD
        varSmallInt : Result := byte(vSmallInt);
        varShortInt : Result := byte(vShortInt);
        varInteger  : Result := byte(vInteger);
{$ifndef FPUNONE}
        varSingle   : Result := byte(Round(vSingle));
        varDouble   : Result := byte(Round(vDouble));
        varCurrency : Result := byte(Round(vCurrency));
        varDate     : Result := byte(Round(vDate));
{$endif}
        varBoolean  : Result := byte(SmallInt(vBoolean));
        varVariant  : Result := VariantToByte(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := byte(vWord);
        varLongWord : Result := byte(vLongWord);
        varInt64    : Result := byte(vInt64);
        varQword    : Result := byte(vQWord);
        varOleStr   : Result := WStrToByte(vOleStr);
        varString   : Result := LStrToByte(vString);
        varUString  : Result := UStrToByte(vString);
      else
        VariantTypeMismatch(vType, varByte);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := byte(PSmallInt(vPointer)^);
        varShortInt : Result := byte(PShortInt(vPointer)^);
        varInteger  : Result := byte(PInteger(vPointer)^);
{$ifndef FPUNONE}
        varSingle   : Result := byte(Round(PSingle(vPointer)^));
        varDouble   : Result := byte(Round(PDouble(vPointer)^));
        varCurrency : Result := byte(Round(PCurrency(vPointer)^));
        varDate     : Result := byte(Round(PDate(vPointer)^));
{$endif}
        varBoolean  : Result := byte(SmallInt(PWordBool(vPointer)^));
        varVariant  : Result := byte(VariantToByte(PVarData(vPointer)^));
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := byte(PWord(vPointer)^);
        varLongWord : Result := byte(PLongWord(vPointer)^);
        varInt64    : Result := byte(PInt64(vPointer)^);
        varQword    : Result := byte(PQWord(vPointer)^);
        varOleStr   : Result := WStrToByte(PPointer(vPointer)^);
        varString   : Result := LStrToByte(PPointer(vPointer)^);
        varUString  : Result := UStrToByte(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varByte);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varByte);
    else { array or something like that }
      VariantTypeMismatch(vType, varByte);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToByte -> ', Result);
  end; {$ENDIF}
end;

{--- Int64 ---}

Function WStrToInt64(p: Pointer) : Int64;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varInt64);
end;

Function LStrToInt64(p: Pointer) : Int64;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varInt64);
end;

Function UStrToInt64(p: Pointer) : Int64;
var
  Error : Word;
begin
  Val(UnicodeString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varUString, varInt64);
end;

Function VariantToInt64(const VargSrc : TVarData) : Int64;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToInt64', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := vInteger;
{$ifndef FPUNONE}
        varSingle   : Result := Round(vSingle);
        varDouble   : Result := Round(vDouble);
        varCurrency : Result := Round(vCurrency);
        varDate     : Result := Round(vDate);
{$endif}
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToInt64(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := vInt64;
        varQword    : Result := int64(vQWord);
        varOleStr   : Result := WStrToInt64(vOleStr);
        varString   : Result := LStrToInt64(vString);
        varUString  : Result := UStrToInt64(vString);
      else
        VariantTypeMismatch(vType, varInt64);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
{$ifndef FPUNONE}
        varSingle   : Result := Round(PSingle(vPointer)^);
        varDouble   : Result := Round(PDouble(vPointer)^);
        varCurrency : Result := Round(PCurrency(vPointer)^);
        varDate     : Result := Round(PDate(vPointer)^);
{$endif}
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToInt64(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := PInt64(vPointer)^;
        varQword    : Result := PQWord(vPointer)^;
        varOleStr   : Result := WStrToInt64(PPointer(vPointer)^);
        varString   : Result := LStrToInt64(PPointer(vPointer)^);
        varUString  : Result := UStrToInt64(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varInt64);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varInt64);
    else { array or something like that }
      VariantTypeMismatch(vType, varInt64);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
=======
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := vInteger;
        varSingle   : Result := Round(vSingle);
        varDouble   : Result := Round(vDouble);
        varCurrency : Result := Round(vCurrency);
        varDate     : Result := Round(vDate);
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToInt64(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := vInt64;
        varQword    : Result := int64(vQWord);
        varOleStr   : Result := WStrToInt64(vOleStr);
        varString   : Result := LStrToInt64(vString);
      else
        VariantTypeMismatch(vType, varInt64);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
        varSingle   : Result := Round(PSingle(vPointer)^);
        varDouble   : Result := Round(PDouble(vPointer)^);
        varCurrency : Result := Round(PCurrency(vPointer)^);
        varDate     : Result := Round(PDate(vPointer)^);
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToInt64(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := PInt64(vPointer)^;
        varQword    : Result := PQWord(vPointer)^;
        varOleStr   : Result := WStrToInt64(PPointer(vPointer)^);
        varString   : Result := LStrToInt64(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varInt64);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varInt64);
    else { array or something like that }
      VariantTypeMismatch(vType, varInt64);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
>>>>>>> graemeg/fixes_2_2
  WriteLn('VariantToInt64 -> ', Result);
  end; {$ENDIF}
end;

{--- QWord ---}

Function WStrToQWord(p: Pointer) : QWord;
var
  Error : Word;
begin
  Val(WideString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varOleStr, varQWord);
<<<<<<< HEAD
end;

Function LStrToQWord(p: Pointer) : QWord;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varQWord);
end;

Function UStrToQWord(p: Pointer) : QWord;
var
  Error : Word;
begin
  Val(UnicodeString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varUString, varQWord);
=======
end;

Function LStrToQWord(p: Pointer) : QWord;
var
  Error : Word;
begin
  Val(AnsiString(p), Result, Error);
  if Error <> 0 then
    VariantTypeMismatch(varString, varQWord);
>>>>>>> graemeg/fixes_2_2
end;

Function VariantToQWord(const VargSrc : TVarData) : QWord;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToQWord', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := qword(vSmallInt);
        varShortInt : Result := qword(vShortInt);
        varInteger  : Result := qword(vInteger);
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
>>>>>>> graemeg/fixes_2_2
        varSingle   : Result := qword(Round(vSingle));
        varDouble   : Result := qword(Round(vDouble));
        varCurrency : Result := qword(Round(vCurrency));
        varDate     : Result := qword(Round(vDate));
<<<<<<< HEAD
{$endif}
        varBoolean  : Result := qword(SmallInt(vBoolean));
=======
        varBoolean  : Result := SmallInt(vBoolean);
>>>>>>> graemeg/fixes_2_2
        varVariant  : Result := VariantToQWord(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := qword(vInt64);
        varQword    : Result := vQWord;
        varOleStr   : Result := WStrToQWord(vOleStr);
        varString   : Result := LStrToQWord(vString);
<<<<<<< HEAD
        varUString  : Result := UStrToQWord(vString);
      else
        VariantTypeMismatch(vType, varQWord);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := qword(PSmallInt(vPointer)^);
        varShortInt : Result := qword(PShortInt(vPointer)^);
        varInteger  : Result := qword(PInteger(vPointer)^);
{$ifndef FPUNONE}
=======
      else
        VariantTypeMismatch(vType, varQWord);
      end;
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := qword(PSmallInt(vPointer)^);
        varShortInt : Result := qword(PShortInt(vPointer)^);
        varInteger  : Result := qword(PInteger(vPointer)^);
>>>>>>> graemeg/fixes_2_2
        varSingle   : Result := qword(Round(PSingle(vPointer)^));
        varDouble   : Result := qword(Round(PDouble(vPointer)^));
        varCurrency : Result := qword(Round(PCurrency(vPointer)^));
        varDate     : Result := qword(Round(PDate(vPointer)^));
<<<<<<< HEAD
{$endif}
        varBoolean  : Result := qword(SmallInt(PWordBool(vPointer)^));
=======
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
>>>>>>> graemeg/fixes_2_2
        varVariant  : Result := VariantToQWord(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := qword(PInt64(vPointer)^);
        varQword    : Result := PQWord(vPointer)^;
        varOleStr   : Result := WStrToQWord(PPointer(vPointer)^);
        varString   : Result := LStrToQWord(PPointer(vPointer)^);
<<<<<<< HEAD
        varUString  : Result := UStrToQWord(PPointer(vPointer)^);
=======
>>>>>>> graemeg/fixes_2_2
      else { other vtype }
        VariantTypeMismatch(vType, varQWord);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varQWord);
    else { array or something like that }
      VariantTypeMismatch(vType, varQWord);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToQWord -> ', Result);
  end; {$ENDIF}
end;

<<<<<<< HEAD
function VarDateToString(DT: TDateTime): AnsiString;
begin
  if Trunc(DT) = 0 then
    Result := TimeToStr(DT)
  else
    Result := DateTimeToStr(DT);
end;

{--- WideString ---}

Function VariantToWideString(const VargSrc : TVarData) : WideString;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToWideString', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := '';
        varSmallInt : Result := IntToStr(vSmallInt);
        varShortInt : Result := IntToStr(vShortInt);
        varInteger  : Result := IntToStr(vInteger);
{$ifndef FPUNONE}
        varSingle   : Result := FloatToStr(vSingle);
        varDouble   : Result := FloatToStr(vDouble);
        varCurrency : Result := FloatToStr(vCurrency);
        varDate     : Result := VarDateToString(vDate);
{$endif}
        varBoolean  : Result := BoolToStr(vBoolean, True);
        varVariant  : Result := VariantToWideString(PVarData(vPointer)^);
        varByte     : Result := IntToStr(vByte);
        varWord     : Result := IntToStr(vWord);
        varLongWord : Result := IntToStr(vLongWord);
        varInt64    : Result := IntToStr(vInt64);
        varQword    : Result := IntToStr(vQWord);
        varOleStr   : Result := WideString(Pointer(vOleStr));
        varString   : Result := AnsiString(vString);
        varUString  : Result := UnicodeString(vString);
      else
        VariantTypeMismatch(vType, varOleStr);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := IntToStr(PSmallInt(vPointer)^);
        varShortInt : Result := IntToStr(PShortInt(vPointer)^);
        varInteger  : Result := IntToStr(PInteger(vPointer)^);
{$ifndef FPUNONE}
        varSingle   : Result := FloatToStr(PSingle(vPointer)^);
        varDouble   : Result := FloatToStr(PDouble(vPointer)^);
        varCurrency : Result := FloatToStr(PCurrency(vPointer)^);
        varDate     : Result := VarDateToString(PDate(vPointer)^);
{$endif}
        varBoolean  : Result := BoolToStr(PWordBool(vPointer)^, True);
        varVariant  : Result := VariantToWideString(PVarData(vPointer)^);
        varByte     : Result := IntToStr(PByte(vPointer)^);
        varWord     : Result := IntToStr(PWord(vPointer)^);
        varLongWord : Result := IntToStr(PLongWord(vPointer)^);
        varInt64    : Result := IntToStr(PInt64(vPointer)^);
        varQword    : Result := IntToStr(PQWord(vPointer)^);
        varOleStr   : Result := WideString(PPointer(vPointer)^);
        varString   : Result := AnsiString(PPointer(vPointer)^);
        varUString  : Result := UnicodeString(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varOleStr);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varOleStr);
    else { array or something like that }
      VariantTypeMismatch(vType, varOleStr);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToWideString -> ', Result);
  end; {$ENDIF}
end;

{--- AnsiString ---}

Function VariantToAnsiString(const VargSrc : TVarData) : AnsiString;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToAnsiString', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := '';
        varSmallInt : Result := IntToStr(vSmallInt);
        varShortInt : Result := IntToStr(vShortInt);
        varInteger  : Result := IntToStr(vInteger);
{$ifndef FPUNONE}
        varSingle   : Result := FloatToStr(vSingle);
        varDouble   : Result := FloatToStr(vDouble);
        varCurrency : Result := FloatToStr(vCurrency);
        varDate     : Result := VarDateToString(vDate);
{$endif}
        varBoolean  : Result := BoolToStr(vBoolean, True);
        varVariant  : Result := VariantToAnsiString(PVarData(vPointer)^);
        varByte     : Result := IntToStr(vByte);
        varWord     : Result := IntToStr(vWord);
        varLongWord : Result := IntToStr(vLongWord);
        varInt64    : Result := IntToStr(vInt64);
        varQword    : Result := IntToStr(vQWord);
        varOleStr   : Result := WideString(Pointer(vOleStr));
        varString   : Result := AnsiString(vString);
        varUString  : Result := UnicodeString(vString);
      else
        VariantTypeMismatch(vType, varString);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
<<<<<<< HEAD
        varSmallInt : Result := IntToStr(PSmallInt(vPointer)^);
        varShortInt : Result := IntToStr(PShortInt(vPointer)^);
        varInteger  : Result := IntToStr(PInteger(vPointer)^);
=======
        varSmallInt : Result := byte(PSmallInt(vPointer)^);
        varShortInt : Result := byte(PShortInt(vPointer)^);
        varInteger  : Result := byte(PInteger(vPointer)^);
>>>>>>> origin/fixes_2.4
{$ifndef FPUNONE}
        varSingle   : Result := FloatToStr(PSingle(vPointer)^);
        varDouble   : Result := FloatToStr(PDouble(vPointer)^);
        varCurrency : Result := FloatToStr(PCurrency(vPointer)^);
        varDate     : Result := VarDateToString(PDate(vPointer)^);
{$endif}
        varBoolean  : Result := BoolToStr(PWordBool(vPointer)^, True);
        varVariant  : Result := VariantToAnsiString(PVarData(vPointer)^);
        varByte     : Result := IntToStr(PByte(vPointer)^);
        varWord     : Result := IntToStr(PWord(vPointer)^);
        varLongWord : Result := IntToStr(PLongWord(vPointer)^);
        varInt64    : Result := IntToStr(PInt64(vPointer)^);
        varQword    : Result := IntToStr(PQWord(vPointer)^);
        varOleStr   : Result := WideString(PPointer(vPointer)^);
        varString   : Result := AnsiString(PPointer(vPointer)^);
        varUString  : Result := UnicodeString(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varString);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varString);
    else { array or something like that }
      VariantTypeMismatch(vType, varString);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToAnsiString -> ', Result);
  end; {$ENDIF}
end;


Function VariantToShortString(const VargSrc : TVarData) : ShortString;
begin
=======
{--- WideString ---}

Function VariantToWideString(const VargSrc : TVarData) : WideString;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToWideString', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := '';
        varSmallInt : Result := IntToStr(vSmallInt);
        varShortInt : Result := IntToStr(vShortInt);
        varInteger  : Result := IntToStr(vInteger);
        varSingle   : Result := FloatToStr(vSingle);
        varDouble   : Result := FloatToStr(vDouble);
        varCurrency : Result := FloatToStr(vCurrency);
        varDate     : Result := FloatToStr(vDate);
        varBoolean  : Result := BoolToStr(vBoolean, True);
        varVariant  : Result := VariantToWideString(PVarData(vPointer)^);
        varByte     : Result := IntToStr(vByte);
        varWord     : Result := IntToStr(vWord);
        varLongWord : Result := IntToStr(vLongWord);
        varInt64    : Result := IntToStr(vInt64);
        varQword    : Result := IntToStr(vQWord);
        varOleStr   : Result := WideString(Pointer(vOleStr));
        varString   : Result := AnsiString(vString);
      else
        VariantTypeMismatch(vType, varOleStr);
      end;
<<<<<<< HEAD
      varByRef: if Assigned(vPointer) then case vType of
        varSmallInt : Result := IntToStr(PSmallInt(vPointer)^);
        varShortInt : Result := IntToStr(PShortInt(vPointer)^);
        varInteger  : Result := IntToStr(PInteger(vPointer)^);
        varSingle   : Result := FloatToStr(PSingle(vPointer)^);
        varDouble   : Result := FloatToStr(PDouble(vPointer)^);
        varCurrency : Result := FloatToStr(PCurrency(vPointer)^);
        varDate     : Result := FloatToStr(PDate(vPointer)^);
        varBoolean  : Result := BoolToStr(PWordBool(vPointer)^, True);
        varVariant  : Result := VariantToWideString(PVarData(vPointer)^);
        varByte     : Result := IntToStr(PByte(vPointer)^);
        varWord     : Result := IntToStr(PWord(vPointer)^);
        varLongWord : Result := IntToStr(PLongWord(vPointer)^);
        varInt64    : Result := IntToStr(PInt64(vPointer)^);
        varQword    : Result := IntToStr(PQWord(vPointer)^);
        varOleStr   : Result := WideString(PPointer(vPointer)^);
        varString   : Result := AnsiString(PPointer(vPointer)^);
=======
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
{$ifndef FPUNONE}
        varSingle   : Result := Round(PSingle(vPointer)^);
        varDouble   : Result := Round(PDouble(vPointer)^);
        varCurrency : Result := Round(PCurrency(vPointer)^);
        varDate     : Result := Round(PDate(vPointer)^);
{$endif}
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToInt64(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := PInt64(vPointer)^;
        varQword    : Result := PQWord(vPointer)^;
        varOleStr   : Result := WStrToInt64(PPointer(vPointer)^);
        varString   : Result := LStrToInt64(PPointer(vPointer)^);
>>>>>>> origin/fixes_2.4
      else { other vtype }
        VariantTypeMismatch(vType, varOleStr);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varOleStr);
    else { array or something like that }
      VariantTypeMismatch(vType, varOleStr);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToWideString -> ', Result);
  end; {$ENDIF}
end;

{--- AnsiString ---}

Function VariantToAnsiString(const VargSrc : TVarData) : AnsiString;
begin
=======
        varSmallInt : Result := qword(vSmallInt);
        varShortInt : Result := qword(vShortInt);
        varInteger  : Result := qword(vInteger);
        varSingle   : Result := qword(Round(vSingle));
        varDouble   : Result := qword(Round(vDouble));
        varCurrency : Result := qword(Round(vCurrency));
        varDate     : Result := qword(Round(vDate));
        varBoolean  : Result := SmallInt(vBoolean);
        varVariant  : Result := VariantToQWord(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := qword(vInt64);
        varQword    : Result := vQWord;
        varOleStr   : Result := WStrToQWord(vOleStr);
        varString   : Result := LStrToQWord(vString);
      else
        VariantTypeMismatch(vType, varQWord);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := qword(PSmallInt(vPointer)^);
        varShortInt : Result := qword(PShortInt(vPointer)^);
        varInteger  : Result := qword(PInteger(vPointer)^);
        varSingle   : Result := qword(Round(PSingle(vPointer)^));
        varDouble   : Result := qword(Round(PDouble(vPointer)^));
        varCurrency : Result := qword(Round(PCurrency(vPointer)^));
        varDate     : Result := qword(Round(PDate(vPointer)^));
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToQWord(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := qword(PInt64(vPointer)^);
        varQword    : Result := PQWord(vPointer)^;
        varOleStr   : Result := WStrToQWord(PPointer(vPointer)^);
        varString   : Result := LStrToQWord(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varQWord);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varQWord);
    else { array or something like that }
      VariantTypeMismatch(vType, varQWord);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToQWord -> ', Result);
  end; {$ENDIF}
end;

{--- WideString ---}

Function VariantToWideString(const VargSrc : TVarData) : WideString;
begin
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToWideString', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := '';
        varSmallInt : Result := IntToStr(vSmallInt);
        varShortInt : Result := IntToStr(vShortInt);
        varInteger  : Result := IntToStr(vInteger);
        varSingle   : Result := FloatToStr(vSingle);
        varDouble   : Result := FloatToStr(vDouble);
        varCurrency : Result := FloatToStr(vCurrency);
        varDate     : Result := FloatToStr(vDate);
        varBoolean  : Result := BoolToStr(vBoolean, True);
        varVariant  : Result := VariantToWideString(PVarData(vPointer)^);
        varByte     : Result := IntToStr(vByte);
        varWord     : Result := IntToStr(vWord);
        varLongWord : Result := IntToStr(vLongWord);
        varInt64    : Result := IntToStr(vInt64);
        varQword    : Result := IntToStr(vQWord);
        varOleStr   : Result := WideString(Pointer(vOleStr));
        varString   : Result := AnsiString(vString);
      else
        VariantTypeMismatch(vType, varOleStr);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := IntToStr(PSmallInt(vPointer)^);
        varShortInt : Result := IntToStr(PShortInt(vPointer)^);
        varInteger  : Result := IntToStr(PInteger(vPointer)^);
        varSingle   : Result := FloatToStr(PSingle(vPointer)^);
        varDouble   : Result := FloatToStr(PDouble(vPointer)^);
        varCurrency : Result := FloatToStr(PCurrency(vPointer)^);
        varDate     : Result := FloatToStr(PDate(vPointer)^);
        varBoolean  : Result := BoolToStr(PWordBool(vPointer)^, True);
        varVariant  : Result := VariantToWideString(PVarData(vPointer)^);
        varByte     : Result := IntToStr(PByte(vPointer)^);
        varWord     : Result := IntToStr(PWord(vPointer)^);
        varLongWord : Result := IntToStr(PLongWord(vPointer)^);
        varInt64    : Result := IntToStr(PInt64(vPointer)^);
        varQword    : Result := IntToStr(PQWord(vPointer)^);
        varOleStr   : Result := WideString(PPointer(vPointer)^);
        varString   : Result := AnsiString(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varOleStr);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varOleStr);
    else { array or something like that }
      VariantTypeMismatch(vType, varOleStr);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToWideString -> ', Result);
  end; {$ENDIF}
end;

{--- AnsiString ---}

Function VariantToAnsiString(const VargSrc : TVarData) : AnsiString;
begin
>>>>>>> origin/fixes_2_2
  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  DumpVariant('VariantToAnsiString', VargSrc);
  end; {$ENDIF}

  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := '';
        varSmallInt : Result := IntToStr(vSmallInt);
        varShortInt : Result := IntToStr(vShortInt);
        varInteger  : Result := IntToStr(vInteger);
        varSingle   : Result := FloatToStr(vSingle);
        varDouble   : Result := FloatToStr(vDouble);
        varCurrency : Result := FloatToStr(vCurrency);
        varDate     : Result := DateToStr(vDate);
        varBoolean  : Result := BoolToStr(vBoolean, True);
        varVariant  : Result := VariantToAnsiString(PVarData(vPointer)^);
        varByte     : Result := IntToStr(vByte);
        varWord     : Result := IntToStr(vWord);
        varLongWord : Result := IntToStr(vLongWord);
        varInt64    : Result := IntToStr(vInt64);
        varQword    : Result := IntToStr(vQWord);
        varOleStr   : Result := WideString(Pointer(vOleStr));
        varString   : Result := AnsiString(vString);
      else
        VariantTypeMismatch(vType, varString);
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := IntToStr(PSmallInt(vPointer)^);
        varShortInt : Result := IntToStr(PShortInt(vPointer)^);
        varInteger  : Result := IntToStr(PInteger(vPointer)^);
        varSingle   : Result := FloatToStr(PSingle(vPointer)^);
        varDouble   : Result := FloatToStr(PDouble(vPointer)^);
        varCurrency : Result := FloatToStr(PCurrency(vPointer)^);
        varDate     : Result := DateToStr(PDate(vPointer)^);
        varBoolean  : Result := BoolToStr(PWordBool(vPointer)^, True);
        varVariant  : Result := VariantToAnsiString(PVarData(vPointer)^);
        varByte     : Result := IntToStr(PByte(vPointer)^);
        varWord     : Result := IntToStr(PWord(vPointer)^);
        varLongWord : Result := IntToStr(PLongWord(vPointer)^);
        varInt64    : Result := IntToStr(PInt64(vPointer)^);
        varQword    : Result := IntToStr(PQWord(vPointer)^);
        varOleStr   : Result := WideString(PPointer(vPointer)^);
        varString   : Result := AnsiString(PPointer(vPointer)^);
      else { other vtype }
        VariantTypeMismatch(vType, varString);
      end else { pointer is nil }
        VariantTypeMismatch(vType, varString);
    else { array or something like that }
      VariantTypeMismatch(vType, varString);
    end;

  {$IFDEF DEBUG_VARUTILS} if __DEBUG_VARUTILS then begin
  WriteLn('VariantToAnsiString -> ', Result);
  end; {$ENDIF}
end;


Function VariantToShortString(const VargSrc : TVarData) : ShortString;
begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result:=VariantToAnsiString(VargSrc);
end;

{ ---------------------------------------------------------------------
    Some debug routines
  ---------------------------------------------------------------------}


Procedure DumpVariant(const VSrc : Variant);
begin
  DumpVariant(Output, '', TVarData(VSrc));
end;

Procedure DumpVariant(const aName: string; const VSrc : Variant);
begin
  DumpVariant(Output, aName, TVarData(VSrc));
end;

Procedure DumpVariant(Var F : Text; const VSrc : Variant);
begin
  DumpVariant(F, '', TVarData(VSrc));
end;
<<<<<<< HEAD
<<<<<<< HEAD

procedure DumpVariant(var F : Text; const aName: string; const VSrc : Variant); 
begin
  DumpVariant(F, aName, TVarData(VSrc));
end;

Procedure DumpVariant(const VargSrc : TVarData);
begin
  DumpVariant(Output, '', VargSrc);
end;

Procedure DumpVariant(const aName: string; const VargSrc : TVarData);
begin
  DumpVariant(Output, aName, VargSrc);
end;

=======
=======
>>>>>>> origin/fixes_2_2

procedure DumpVariant(var F : Text; const aName: string; const VSrc : Variant); 
begin
  DumpVariant(F, aName, TVarData(VSrc));
end;

Procedure DumpVariant(const VargSrc : TVarData);
begin
  DumpVariant(Output, '', VargSrc);
end;

Procedure DumpVariant(const aName: string; const VargSrc : TVarData);
begin
  DumpVariant(Output, aName, VargSrc);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Procedure DumpVariant(Var F : Text; const VargSrc : TVarData);
begin
  DumpVariant(F, '', VargSrc);
end;

const
  VarTypeStrings : array [varEmpty..varQword] of string = (
    'empty',     { varempty    = 0 }
    'null',      { varnull     = 1 }
    'smallint',  { varsmallint = 2 }
    'integer',   { varinteger  = 3 }
    'single',    { varsingle   = 4 }
    'double',    { vardouble   = 5 }
    'currency',  { varcurrency = 6 }
    'date',      { vardate     = 7 }
    'olestr',    { varolestr   = 8 }
    'dispatch',  { vardispatch = 9 }
    'error',     { varerror    = 10 }
    'boolean',   { varboolean  = 11 }
    'variant',   { varvariant  = 12 }
    'unknown',   { varunknown  = 13 }
    'decimal',   { vardecimal  = 14 }
    'undefined',
    'shortint',  { varshortint = 16 }
    'byte',      { varbyte     = 17 }
    'word',      { varword     = 18 }
    'longword',  { varlongword = 19 }
    'int64',     { varint64    = 20 }
    'qword');    { varqword    = 21 }

Procedure DumpVariant(Var F : Text; const aName: string; const VargSrc : TVarData);

Var
  i: Integer;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Writeln(F,'---> ', aName, ' at $', HexStr(@VargSrc), ' <----------------');
=======
  Writeln(F,'---> ', aName, ' at $', IntToHex(Cardinal(@VargSrc), 8), ' <----------------');
>>>>>>> graemeg/fixes_2_2
=======
  Writeln(F,'---> ', aName, ' at $', IntToHex(Cardinal(@VargSrc), 8), ' <----------------');
>>>>>>> origin/fixes_2_2
  with VargSrc do begin

    if vType and varByRef = varByRef then
      Writeln(F,'Variant is by reference.');

    if vType and varArray = varArray then
      Writeln(F,'Variant is an array.');

    if vType and not (varTypeMask or varArray or varByRef) <> 0 then
      Writeln(F,'Variant has unknown flags set in type: $', IntToHex(vType, 4));


    If (vType and varTypeMask) in [varEmpty..varQword] then
      Writeln(F,'Variant has type : ', VarTypeStrings[vType and varTypeMask])
    else If (vType and varTypeMask) = varString then
      Writeln(F,'Variant has type : string')
<<<<<<< HEAD
<<<<<<< HEAD
    else if (vType and varTypeMask) = varUString then
      Writeln(F,'Variant has type : UnicodeString')
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    else
      Writeln(F,'Variant has unknown type : $', IntToHex(vType and varTypeMask, 4));

    Write('Bytes :');
    for i := 0 to 13 do
      Write(IntToHex(VBytes[i], 2),' ');
    WriteLn;

    if vType and varArray = varArray then begin
<<<<<<< HEAD
<<<<<<< HEAD
      Writeln(F,'---< ', aName, ' at $', HexStr(@VargSrc), ' >----------------');
=======
      Writeln(F,'---< ', aName, ' at $', IntToHex(Cardinal(@VargSrc), 8), ' >----------------');
>>>>>>> graemeg/fixes_2_2
=======
      Writeln(F,'---< ', aName, ' at $', IntToHex(Cardinal(@VargSrc), 8), ' >----------------');
>>>>>>> origin/fixes_2_2
      Writeln(F);
      Exit;
    end;

    If vType <> varEmpty then begin
      Write(F,'Value is: [');

      if (vType and varByRef = varByRef) or (vType and varTypeMask = varVariant) then
        if not Assigned(vPointer) then begin
          WriteLn(F, 'nil]');
<<<<<<< HEAD
<<<<<<< HEAD
          Writeln(F,'---< ', aName, ' at $', HexStr(@VargSrc), ' >----------------');
=======
          Writeln(F,'---< ', aName, ' at $', IntToHex(Cardinal(@VargSrc), 8), ' >----------------');
>>>>>>> graemeg/fixes_2_2
=======
          Writeln(F,'---< ', aName, ' at $', IntToHex(Cardinal(@VargSrc), 8), ' >----------------');
>>>>>>> origin/fixes_2_2
          Writeln(F);
          Exit;
        end;

      case vType of
        varNull                 : Write(F, 'Null');
        varSmallInt             : Write(F, vSmallInt);
        varInteger              : Write(F, vInteger);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varSingle               : Write(F, vSingle);
        varDouble               : Write(F, vDouble);
        varCurrency             : Write(F, vCurrency);
        varDate                 : Write(F, vDate);
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varOleStr               : Write(F, WideString(Pointer(vOleStr)));
        varError                : Write(F, IntToHex(Cardinal(vError), 8));
        varBoolean              : Write(F, vBoolean);
        varVariant, varVariant or varByRef : begin
          WriteLn(' dereferencing -> ]');
          DumpVariant(F, aName+'^', PVarData(vPointer)^);
          Exit;
        end;
        varShortInt             : Write(F, vShortInt);
        varByte                 : Write(F, vByte);
        varWord                 : Write(F, vWord);
        varLongWord             : Write(F, vLongWord);
        varInt64                : Write(F, vInt64);
        varQword                : Write(F, vQWord);
        varString               : Write(F, AnsiString(vString));
        varNull     or varByRef : Write(F, 'Null');
        varSmallInt or varByRef : Write(F, PSmallInt(vPointer)^);
        varInteger  or varByRef : Write(F, PInteger(vPointer)^);
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varSingle   or varByRef : Write(F, PSingle(vPointer)^);
        varDouble   or varByRef : Write(F, PDouble(vPointer)^);
        varCurrency or varByRef : Write(F, PCurrency(vPointer)^);
        varDate     or varByRef : Write(F, PDate(vPointer)^);
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        varOleStr   or varByRef : Write(F, WideString(PPointer(vPointer)^));
        varError    or varByRef : Write(F, IntToHex(Cardinal(PLongWord(vPointer)^), 8));
        varBoolean  or varByRef : Write(F, PWordBool(vPointer)^);
        varShortInt or varByRef : Write(F, PShortInt(vPointer)^);
        varByte     or varByRef : Write(F, PByte(vPointer)^);
        varWord     or varByRef : Write(F, PWord(vPointer)^);
        varLongWord or varByRef : Write(F, PLongWord(vPointer)^);
        varInt64    or varByRef : Write(F, PInt64(vPointer)^);
        varQword    or varByRef : Write(F, PQWord(vPointer)^);
        varString   or varByRef : Write(F, AnsiString(PPointer(vPointer)^));
      else
        Write(F, 'Unsupported');
      end;
      WriteLn(F, ']');
    end;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  Writeln(F,'---< ', aName, ' at $', HexStr(@VargSrc), ' >----------------');
=======
  Writeln(F,'---< ', aName, ' at $', IntToHex(Cardinal(@VargSrc), 8), ' >----------------');
>>>>>>> graemeg/fixes_2_2
=======
  Writeln(F,'---< ', aName, ' at $', IntToHex(Cardinal(@VargSrc), 8), ' >----------------');
>>>>>>> origin/fixes_2_2
  Writeln(F);
end;



