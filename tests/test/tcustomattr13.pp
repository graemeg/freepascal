program tcustomattr13;

{$mode objfpc}
{$modeswitch prefixedattributes}

uses
  TypInfo, Classes, SysUtils, Math;

type
  TString8 = String[8];
  TSet = set of (One, Two, Three);

const
  StrHelloWorld = 'Hello World';
  StrFoobar = 'Foobar';
  StrBlubb = 'Blubb';

  ByteVal = $5a;
  CurrVal = Currency(33.51);
  CompVal = 1234;
  SingleVal = 3.14156;
  SetVal = [One, Three];

type
  TMyAttr = class(TCustomAttribute)
    constructor Create(aByte: Byte; aStr: TString8; aFlt: Single);
    constructor Create(aStr: AnsiString; aSet: TSet; aPtr: Pointer);
    constructor Create(aComp: Comp; aCurr: Currency; aGuid: TGUID; aStr: UnicodeString);
  end;

  [TMyAttr(ByteVal, StrHelloWorld, SingleVal)]
  [TMyAttr(StrFoobar, SetVal, Nil)]
  [TMyAttr(CompVal, CurrVal, IInterface, StrBlubb)]
  TMyClass = class
  end;

constructor TMyAttr.CReate(aByte: Byte; aStr: TString8; aFlt: Single);
begin
end;

constructor TMyAttr.Create(aStr: AnsiString; aSet: TSet; aPtr: Pointer);
begin
end;

constructor TMyAttr.Create(aComp: Comp; aCurr: Currency; aGuid: TGUID; aStr: UnicodeString);
begin

end;

procedure DumpData(aData: Pointer; aSize: SizeInt);
var
  i: SizeInt;
  chars: String[16];
begin
  chars := '                ';
  for i := 0 to aSize - 1 do begin
    if i mod 16 = 0 then begin
      if i > 0 then begin
        Writeln('   ', chars);
        chars := '                ';
      end;
      Write(HexStr(PtrUInt(aData) + i, SizeOF(PtrUInt) * 2), '   ');
    end;
    Write(HexStr((PByte(aData) + i)^, 2), ' ');
    if (PByte(aData)[i] >= $20) and (PByte(aData)[i] < $7F) then
      chars[(i mod 16) + 1] := Chr(PByte(aData)[i])
    else
      chars[(i mod 16) + 1] := '.';
  end;
  while aSize mod 16 <> 0 do begin
    Write('   ');
    Inc(aSize);
  end;
  Writeln('   ', chars);
end;

procedure CheckAttr1(aStrm: TStream);
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
{$push}
{$packrecords c}
type
  TAlignDummy = record
    b: Byte;
    s: Single;
  end;
{$pop}
{$endif}
var
  b: Byte;
  ss: ShortString;
  s: Single;
begin
  if aStrm.Read(b, SizeOf(b)) <> SizeOf(b) then
    Halt(20);
  if b <> ByteVal then
    Halt(21);
  if aStrm.Read(b, SizeOf(b)) <> SizeOf(b) then
    Halt(22);
  if b <> Length(StrHelloWorld) then
    Halt(23);
  SetLength(ss, b);
  if aStrm.Read(ss[1], b) <> b then
    Halt(24);
  if ss <> StrHelloWorld then
    Halt(25);
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
  aStrm.Position := Align(PtrUInt(aStrm.Position), PtrInt(@TAlignDummy(nil^).s)));
{$endif}
  if aStrm.Read(s, SizeOf(Single)) <> SizeOf(Single) then
    Halt(26);
  if s <> Single(SingleVal) then
    Halt(27);
end;

procedure CheckAttr2(aStrm: TStream);
var
  p: Pointer;
  s: TSet;
begin
  if aStrm.Read(p, SizeOf(p)) <> SizeOf(p) then
    Halt(40);
  if AnsiString(p) <> StrFoobar then
    Halt(41);
  if aStrm.Read(s, SizeOf(s)) <> SizeOf(s) then
    Halt(42);
  if s <> SetVal then
    Halt(43);
  if aStrm.Read(p, SizeOf(p)) <> SizeOf(p) then
    Halt(44);
  if Assigned(p) then
    Halt(45);
end;

procedure CheckAttr3(aStrm: TStream);
var
  co: Comp;
  cu: Currency;
  p: Pointer;
  g: TGUID;
begin
  if aStrm.Read(co, SizeOf(co)) <> SizeOf(co) then
    Halt(60);
  if co <> CompVal then
    Halt(61);
  if aStrm.Read(cu, SizeOf(cu)) <> SizeOf(cu) then
    Halt(62);
  if cu <> CurrVal then
    Halt(63);
  if aStrm.Read(g, SizeOf(g)) <> SizeOf(g) then
    Halt(64);
  if not IsEqualGUID(g,TGuid(IInterface)) then
    Halt(65);
  if aStrm.Read(p, SizeOf(p)) <> SizeOf(p) then
    Halt(66);
  if UnicodeString(p) <> StrBlubb then
    Halt(67);
end;

type
  TCheckProc = procedure(aStrm: TStream);

const
  CheckProcs: array[0..2] of TCheckProc = (
    @CheckAttr1,
    @CheckAttr2,
    @CheckAttr3
  );

var
  at: PAttributeTable;
  ae: TAttributeEntry;
  i: SizeInt;
  strm: TMemoryStream;
begin
  at := GetAttributeTable(TypeInfo(TMyClass));
  if at^.AttributeCount = 0 then
    Halt(1);
  if at^.AttributeCount > Length(CheckProcs) then
    Halt(2);

  for i := 0 to at^.AttributeCount - 1 do begin
    ae := at^.AttributesList[i];
    if ae.AttrType^ <> TMyAttr.ClassInfo then
      Halt(3);
    if not Assigned(ae.AttrCtor) then
      Halt(4);
    if not Assigned(ae.AttrProc) then
      Halt(5);
    strm:=TMemoryStream.Create;
    strm.SetSize(ae.ArgLen);
    Move(ae.ArgData^, strm.Memory^, ae.ArgLen);
    CheckProcs[i](strm);
    strm.Free;
  end;
  Writeln('ok');
end.
