
{$ifdef fpc}{$mode objfpc}{$h+}{$endif}
{$apptype console}

uses Variants, SysUtils;

type
  TTest = class(TCustomVariantType)
    procedure Clear(var V: TVarData); override;
    procedure Copy(var Dest: TVarData; const Source: TVarData; const Indirect: Boolean); override;
    procedure DispInvoke(Dest: PVarData; var Source: TVarData; CallDesc: PCallDesc; Params: Pointer); override;
  end;

procedure TTest.Clear(var V: TVarData);
begin
end;

procedure TTest.Copy(var Dest: TVarData; const Source: TVarData; const Indirect: Boolean);
begin
end;

procedure TTest.DispInvoke(Dest: PVarData; var Source: TVarData; CallDesc: PCallDesc; Params: Pointer);
var
  tmp: Word;
begin
  if (CallDesc^.ArgCount =2) and Assigned(Dest) then
  begin
    //writeln(HexStr(PPointer(Params)^), ' ', HexStr(PPointer(Params)[1]));
    WordRec(tmp).Lo := CallDesc^.ArgTypes[0];
    WordRec(tmp).Hi := CallDesc^.ArgTypes[1];
    // !! FPC passes args right-to-left, Delphi does same left-to-right
    // Moreover, IDispatch needs args right-to-left, and Variant Dispatch needs left-to-right. Nice, huh?
    {$ifdef fpc}
    tmp := Swap(tmp);
    {$endif}
    Variant(Dest^) := tmp;
  end;
end;

type
  TTestClass=class
    u8: byte;
    u16: word;
    u32: longword;
{$ifdef fpc}
    u64: qword;
{$endif}
    s8: shortint;
    s16: smallint;
    s32: longint;
    s64: int64;

    cy: currency;

    b: boolean;
    bb: bytebool;
    wb: wordbool;
    lb: longbool;

    sgl: single;
    dbl: double;
    ext: extended;
    dt: TDateTime;

    fsstr: shortstring;
    fastr: ansistring;
    fwstr: widestring;
{$ifdef fpc}
    fustr: unicodestring;
{$endif}

    fvar: Variant;
    fintf: IInterface;
    fdisp: IDispatch;

    property u8prop: Byte read u8;
    property u16prop: Word read u16;
    property u32prop: LongWord read u32;
{$ifdef fpc}
    property u64prop: QWord read u64;
{$endif}
    property s8prop: ShortInt read s8;
    property s16prop: SmallInt read s16;
    property s32prop: LongInt read s32;
    property s64prop: Int64 read s64;

    property cyprop: currency read cy;
    property bprop: boolean read b;
    property bbprop: bytebool read bb;
    property wbprop: wordbool read wb;
    property lbprop: longbool read lb;

    property sglprop: single read sgl;
    property dblprop: double read dbl;
    property extprop: extended read ext;
    property dtprop: TDateTime read dt;

    property varprop: Variant read fvar;
    property intfprop: IInterface read fintf;
    property dispprop: IDispatch read fdisp;

    property sstr: shortstring read fsstr;
    property astr: ansistring read fastr;
    property wstr: widestring read fwstr;
{$ifdef fpc}
    property ustr: unicodestring read fustr;
{$endif}
  end;

var
  cv: TCustomVariantType;
  code: Integer;
  cl: TTestClass;
  v: Variant;

// using negative values of Expected to check that arg is passed by-value only
procedure test(const id: string; const act: Variant; expected: Integer);
var
  tmp: word;
  absexp: Integer;
begin
  tmp := act;
  absexp := abs(expected);
  write(id, WordRec(tmp).Lo,', ', WordRec(tmp).Hi);
  if (expected >= 0) and (WordRec(tmp).Lo <> (expected or $80)) then
  begin
    write(' BYREF failed');
    Code := Code or 1;
  end;
  if WordRec(tmp).Hi <> absexp then
  begin
    write(' BYVAL failed');
    Code := Code or 2;
  end;
  writeln;
end;

begin
  Code := 0;
  cv := TTest.Create;
  cl := TTestClass.Create;
  TVarData(v).vType := cv.VarType;

  test('u8:    ', v.foo(cl.u8, cl.u8prop), varbyte);

  test('u16:    ', v.foo(cl.u16, cl.u16prop), varword);       // (Uncertain) D7: treated as Integer
  test('u32:    ', v.foo(cl.u32, cl.u32prop), varlongword);   // (Uncertain) D7: treated as Integer ByRef
  test('s8:     ', v.foo(cl.s8, cl.s8prop), varshortint);     // (Uncertain) D7: treated as Integer

  test('s16:    ', v.foo(cl.s16, cl.s16prop), varsmallint);
  test('s32:    ', v.foo(cl.s32, cl.s32prop), varinteger);
  test('s64:    ', v.foo(cl.s64, cl.s64prop), varint64);
{$ifdef fpc}
  test('u64:    ', v.foo(cl.u64, cl.u64prop), varword64);
{$endif}

  test('wordbool:', v.foo(cl.wb, cl.wbprop), varBoolean);
  test('curncy:  ', v.foo(cl.cy, cl.cyprop), varCurrency);

  test('single:  ', v.foo(cl.sgl, cl.sglprop), varSingle);
  test('double:  ', v.foo(cl.dbl, cl.dblprop), varDouble);
  test('extended:', v.foo(cl.ext, cl.extprop), -varDouble);  // not a COM type, passed by value

  test('date:    ', v.foo(cl.dt, cl.dtprop), varDate);

  test('ansistr: ', v.foo(cl.fastr, cl.astr), varStrArg);
{$ifdef FPC_WINLIKEWIDESTRING}
  test('widestr: ', v.foo(cl.fwstr, cl.wstr), varOleStr);
{$else FPC_WINLIKEWIDESTRING}
  test('widestr: ', v.foo(cl.fwstr, cl.wstr), varUStrArg);
{$endif FPC_WINLIKEWIDESTRING}
{$ifdef fpc}
  test('unistr:  ', v.foo(cl.fustr, cl.ustr), varUStrArg);
{$endif}
  test('variant: ', v.foo(cl.fvar, cl.varprop), varVariant);

  test('IUnknown:', v.foo(cl.fintf, cl.intfprop), varUnknown);
  test('IDispatch:', v.foo(cl.fdisp, cl.dispprop), varDispatch);

  // not an COM type, passed by value; Delphi uses varStrArg
{$ifdef FPC_WINLIKEWIDESTRING}
  test('shortstr:', v.foo(cl.fsstr, cl.sstr), -varOleStr);
{$else FPC_WINLIKEWIDESTRING}
  test('shortstr:', v.foo(cl.fsstr, cl.sstr), -varUStrArg);
{$endif FPC_WINLIKEWIDESTRING}
// not an COM type, passed by value
  test('longbool:', v.foo(cl.lb, cl.lbprop), -varBoolean);

  // typecasted ordinals (only one arg is actually used)
  test('u8+cast: ', v.foo(byte(55), byte(55)), -varByte);
  test('u16+cast:', v.foo(word(55), word(55)), -varWord);
  test('u32+cast:', v.foo(longword(55), longword(55)), -varLongWord);
{$ifdef fpc}
  test('u64+cast:', v.foo(qword(55), qword(55)), -varQWord);
{$endif}
  test('s8+cast:', v.foo(shortint(55), shortint(55)), -varShortInt);
  test('s16+cast:', v.foo(smallint(55), smallint(55)), -varSmallInt);
  test('s32+cast:', v.foo(longint(55), longint(55)), -varInteger);
  test('s64+cast:', v.foo(int64(55), int64(55)), -varInt64);

  cl.Free;
  if Code <> 0 then
    writeln('Errors: ', Code);
  Halt(Code);

end.
