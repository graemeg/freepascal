{ %TARGET = win32,win64,wince }
{ tests that the different string types are converted correctly when dispatching }

program tdispvar2;

{$mode objfpc}{$H+}

uses
  SysUtils, Variants, ComObj, ActiveX, Windows;

type
  { TTest }

  TTest = class(TInterfacedObject, IDispatch)
    function GetTypeInfoCount(out count : longint) : HResult;stdcall;
    function GetTypeInfo(Index,LocaleID : longint;
      out TypeInfo): HResult;stdcall;
    function GetIDsOfNames(const iid: TGUID; names: Pointer;
      NameCount, LocaleID: LongInt; DispIDs: Pointer) : HResult;stdcall;
    function Invoke(DispID: LongInt;const iid : TGUID;
      LocaleID : longint; Flags: Word;var params;
      VarResult,ExcepInfo,ArgErr : pointer) : HResult;stdcall;
  end;

var
  TestStr: WideString;

{ TTest }

function TTest.GetTypeInfoCount(out count: longint): HResult; stdcall;
begin
  Count := 0;
  Result := S_OK;
end;

function TTest.GetTypeInfo(Index, LocaleID: longint; out TypeInfo): HResult;
  stdcall;
begin
  Result := E_NOTIMPL;
end;

function TTest.GetIDsOfNames(const iid: TGUID; names: Pointer; NameCount,
  LocaleID: LongInt; DispIDs: Pointer): HResult; stdcall;
var
  n: ^PWideChar absolute names;
  d: PDispIDList absolute DispIDs;
begin
  if (WideString(n^) = 'SomeFunction') then begin
    d^[0] := 1;
    Result := S_OK;
  end else
    Result := DISP_E_UNKNOWNNAME;
end;

function TTest.Invoke(DispID: LongInt; const iid: TGUID; LocaleID: longint;
  Flags: Word; var params; VarResult, ExcepInfo, ArgErr: pointer): HResult;
  stdcall;
var
  args: TDispParams absolute params;
  i: UINT;
begin
  //Writeln('Call to Invoke');
  if (DispID = 1) then begin
    //Writeln(HexStr(Flags, 4));
    //Writeln(args.cArgs, ' ', args.cNamedArgs);
    for i := 0 to args.cArgs - 1 do begin
      //Writeln(HexStr(args.rgvarg^[i].vt, 4));
      if args.rgvarg^[i].vt = VT_BSTR then begin
        //Writeln(WideString(args.rgvarg^[i].bstrVal));
        TestStr := WideString(args.rgvarg^[i].bstrVal);
      end else if args.rgvarg^[i].vt = VT_BSTR or VT_BYREF then begin
        //Writeln(args.rgvarg^[i].pbstrVal^);
        TestStr := args.rgvarg^[i].pbstrVal^;
      end;
    end;
    Result := S_OK;
  end else
    Result := E_NOTIMPL;
end;

procedure Test;
{$push}
{$J-}
const
  cs: AnsiString = 'Constant AnsiString';
  cus: UnicodeString = 'Constant UnicodeString';
  cws: WideString = 'Constant WideString';
{$pop}
var
  i: IDispatch;
  w: OleVariant;
  s: AnsiString;
  us: UnicodeString;
  ws: WideString;
begin
  w := Null;
  i := TTest.Create;
  try
    s := 'AnsiString';
    us := 'UnicodeString';
    ws := 'WideString';
    w := i;

    TestStr := '';
    w.SomeFunction('Constant');
    if TestStr <> 'Constant' then
      Halt(1);

    TestStr := '';
    w.SomeFunction(s);
    if TestStr <> 'AnsiString' then
      Halt(2);

    TestStr := '';
    w.SomeFunction(us);
    if TestStr <> 'UnicodeString' then
      Halt(3);

    TestStr := '';
    w.SomeFunction(ws);
    if TestStr <> 'WideString' then
      Halt(4);

    TestStr := '';
    w.SomeFunction(cs);
    if TestStr <> 'Constant AnsiString' then
      Halt(5);

    TestStr := '';
    w.SomeFunction(cus);
    if TestStr <> 'Constant UnicodeString' then
      Halt(6);

    TestStr := '';
    w.SomeFunction(cws);
    if TestStr <> 'Constant WideString' then
      Halt(7);
  finally
    w := Null;
    i := Nil;
  end;
end;

begin
  CoInitializeEx(Nil, COINIT_MULTITHREADED);
  try
    Test;
  finally
    CoUninitialize;
  end;
end.
