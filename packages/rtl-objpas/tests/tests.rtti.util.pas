unit Tests.Rtti.Util;

{$mode objfpc}{$H+}

interface

uses
  TypInfo, Rtti;

{$ifndef fpc}
type
  CodePointer = Pointer;

  TValueHelper = record helper for TValue
    function AsUnicodeString: UnicodeString;
    function AsAnsiString: AnsiString;
    function AsChar: Char; inline;
    function AsAnsiChar: AnsiChar;
    function AsWideChar: WideChar;
  end;
{$endif}

const
{$if defined(cpui386) or defined(cpux86_64) or defined(cpum68k)}
  DefaultCC = ccReg;
{$else}
  DefaultCC = ccStdCall;
{$endif}

function CopyValue({$ifdef fpc}constref{$else}const [ref]{$endif} aValue: TValue): TValue;
function EqualValues({$ifdef fpc}constref{$else}const [ref]{$endif} aValue1, aValue2: TValue): Boolean;

function TypeKindToStr(aTypeKind: TTypeKind): String; inline;

function GetInstValue(aValue: TObject): TValue;
function GetPointerValue(aValue: Pointer): TValue;
function GetIntValue(aValue: SizeInt): TValue;
function GetAnsiString(const aValue: AnsiString): TValue;
function GetShortString(const aValue: ShortString): TValue;
function GetSingleValue(aValue: Single): TValue;
function GetDoubleValue(aValue: Double): TValue;
function GetExtendedValue(aValue: Extended): TValue;
function GetCompValue(aValue: Comp): TValue;
function GetCurrencyValue(aValue: Currency): TValue;
function GetArray(const aArg: array of SizeInt): TValue;

implementation

uses
  SysUtils, Math;

{$ifndef fpc}
function TValueHelper.AsUnicodeString: UnicodeString;
begin
  Result := UnicodeString(AsString);
end;

function TValueHelper.AsAnsiString: AnsiString;
begin
  Result := AnsiString(AsString);
end;

function TValue.AsWideChar: WideChar;
begin
  if Kind <> tkWideChar then
    raise EInvalidCast.Create('Invalid cast');
  Result := WideChar(Word(AsOrdinal));
end;

function TValue.AsAnsiChar: AnsiChar;
begin
  if Kind <> tkChar then
    raise EInvalidCast.Create('Invalid cast');
  Result := AnsiChar(Byte(AsOrdinal));
end;

function TValue.AsChar: Char;
begin
  Result := AsWideChar;
end;
{$endif}

function CopyValue({$ifdef fpc}constref{$else}const [ref]{$endif} aValue: TValue): TValue;
var
  arrptr: Pointer;
  len, i: SizeInt;
begin
  if aValue.Kind = tkDynArray then begin
    { we need to decouple the source reference, so we're going to be a bit
      cheeky here }
    len := aValue.GetArrayLength;
    arrptr := Nil;
    DynArraySetLength(arrptr, aValue.TypeInfo, 1, @len);
    TValue.Make(@arrptr, aValue.TypeInfo, Result);
    for i := 0 to len - 1 do
      Result.SetArrayElement(i, aValue.GetArrayElement(i));
  end else
    TValue.Make(aValue.GetReferenceToRawData, aValue.TypeInfo, Result);
end;

function EqualValues({$ifdef fpc}constref{$else}const [ref]{$endif} aValue1, aValue2: TValue): Boolean;
var
  td1, td2: PTypeData;
  i: SizeInt;
begin
{$ifdef debug}
  Writeln('Empty: ', aValue1.IsEmpty, ' ', aValue2.IsEmpty);
  Writeln('Kind: ', aValue1.Kind, ' ', aValue2.Kind);
  Writeln('Array: ', aValue1.IsArray, ' ', aValue2.IsArray);
{$endif}
  if aValue1.IsEmpty and aValue2.IsEmpty then
    Result := True
  else if aValue1.IsEmpty and not aValue2.IsEmpty then
    Result := False
  else if not aValue1.IsEmpty and aValue2.IsEmpty then
    Result := False
  else if aValue1.IsArray and aValue2.IsArray then begin
    if aValue1.GetArrayLength = aValue2.GetArrayLength then begin
      Result := True;
      for i := 0 to aValue1.GetArrayLength - 1 do
        if not EqualValues(aValue1.GetArrayElement(i), aValue2.GetArrayElement(i)) then begin
{$ifdef debug}
          Writeln('Element ', i, ' differs: ', HexStr(aValue1.GetArrayElement(i).AsOrdinal, 4), ' ', HexStr(aValue2.GetArrayElement(i).AsOrdinal, 4));
{$endif}
          Result := False;
          Break;
        end;
    end else
      Result := False;
  end else if aValue1.Kind = aValue2.Kind then begin
    td1 := aValue1.TypeData;
    td2 := aValue2.TypeData;
    case aValue1.Kind of
      tkBool:
        Result := aValue1.AsBoolean xor not aValue2.AsBoolean;
      tkSet:
        if td1^.SetSize = td2^.SetSize then
          if td1^.SetSize < SizeOf(SizeInt) then
            Result := aValue1.AsOrdinal = aValue2.AsOrdinal
          else
            Result := CompareMem(aValue1.GetReferenceToRawData, aValue2.GetReferenceToRawData, td1^.SetSize)
        else
          Result := False;
      tkEnumeration,
      tkChar,
      tkWChar,
      tkUChar,
      tkInt64,
      tkInteger:
        Result := aValue1.AsOrdinal = aValue2.AsOrdinal;
      tkQWord:
        Result := aValue1.AsUInt64 = aValue2.AsUInt64;
      tkFloat:
        if td1^.FloatType <> td2^.FloatType then
          Result := False
        else begin
          case td1^.FloatType of
            ftSingle:
              Result := SameValue(Single(aValue1.AsExtended), Single(aValue2.AsExtended));
            ftDouble:
              Result := SameValue(Double(aValue1.AsExtended), Double(aValue2.AsExtended));
            ftExtended:
              Result := SameValue(aValue1.AsExtended, aValue2.AsExtended);
            ftComp:
              Result := aValue1.AsInt64 = aValue2.AsInt64;
            ftCurr:
              Result := aValue1.AsCurrency = aValue2.AsCurrency;
          end;
        end;
      tkSString,
      tkUString,
      tkAString,
      tkWString:
        Result := aValue1.AsString = aValue2.AsString;
      tkDynArray,
      tkArray:
        if aValue1.GetArrayLength = aValue2.GetArrayLength then begin
          Result := True;
          for i := 0 to aValue1.GetArrayLength - 1 do
            if not EqualValues(aValue1.GetArrayElement(i), aValue2.GetArrayElement(i)) then begin
              Result := False;
              Break;
            end;
        end else
          Result := False;
      tkClass,
      tkClassRef,
      tkInterface,
      tkInterfaceRaw,
      tkPointer:
        Result := PPointer(aValue1.GetReferenceToRawData)^ = PPointer(aValue2.GetReferenceToRawData)^;
      tkProcVar:
        Result := PCodePointer(aValue1.GetReferenceToRawData)^ = PCodePointer(aValue2.GetReferenceToRawData)^;
      tkRecord,
      tkObject,
      tkMethod,
      tkVariant: begin
        if aValue1.DataSize = aValue2.DataSize then
          Result := CompareMem(aValue1.GetReferenceToRawData, aValue2.GetReferenceToRawData, aValue1.DataSize)
        else
          Result := False;
      end
      else
        Result := False;
    end;
  end else
    Result := False;
end;

function TypeKindToStr(aTypeKind: TTypeKind): String;
begin
{$ifdef fpc}
  Str(aTypeKind, Result);
{$else}
  Result := GetEnumName(TypeInfo(TTypeKind), Ord(aTypeKind));
{$endif}
end;

function GetInstValue(aValue: TObject): TValue;
begin
  Result := TValue.{$ifdef fpc}specialize{$endif}From<TObject>(aValue);
end;

function GetPointerValue(aValue: Pointer): TValue;
begin
  Result := TValue.{$ifdef fpc}specialize{$endif}From<Pointer>(aValue);
end;

function GetIntValue(aValue: SizeInt): TValue;
begin
  Result := TValue.{$ifdef fpc}specialize{$endif}From<SizeInt>(aValue);
end;

function GetAnsiString(const aValue: AnsiString): TValue;
begin
  Result := TValue.{$ifdef fpc}specialize{$endif}From<AnsiString>(aValue);
end;

function GetShortString(const aValue: ShortString): TValue;
begin
  Result := TValue.{$ifdef fpc}specialize{$endif}From<ShortString>(aValue);
end;

function GetSingleValue(aValue: Single): TValue;
begin
  Result := TValue.{$ifdef fpc}specialize{$endif}From<Single>(aValue);
end;

function GetDoubleValue(aValue: Double): TValue;
begin
  Result := TValue.{$ifdef fpc}specialize{$endif}From<Double>(aValue);
end;

function GetExtendedValue(aValue: Extended): TValue;
begin
  Result := TValue.{$ifdef fpc}specialize{$endif}From<Extended>(aValue);
end;

function GetCompValue(aValue: Comp): TValue;
begin
  Result := TValue.{$ifdef fpc}specialize{$endif}From<Comp>(aValue);
end;

function GetCurrencyValue(aValue: Currency): TValue;
begin
  Result := TValue.{$ifdef fpc}specialize{$endif}From<Currency>(aValue);
end;

{$ifdef fpc}
function GetArray(const aArg: array of SizeInt): TValue;
begin
  Result := specialize OpenArrayToDynArrayValue<SizeInt>(aArg);
end;
{$endif}

end.

