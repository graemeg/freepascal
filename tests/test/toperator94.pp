program toperator94;

{$mode objfpc}
{$modeswitch advancedrecords}

type
  TString80 = String[80];
  TString90 = String[90];
  TString40 = String[40];
  TString100 = String[100];

  TTest1 = record
    class operator :=(const aArg: TTest1): TString80;
  end;

  TTest2 = record
    class operator :=(const aArg: TTest2): ShortString;
  end;

var
  ImplicitTest1ShortString: LongInt;
  ImplicitTest1String80: LongInt;
  ImplicitTest2ShortString: LongInt;
  ImplicitTest2String80: LongInt;

class operator TTest1.:=(const aArg: TTest1): TString80;
begin
  Writeln('TTest1 Implicit TString80');
  Inc(ImplicitTest1String80);
  Result := '';
end;

class operator TTest2.:=(const aArg: TTest2): ShortString;
begin
  Writeln('TTest2 Implicit ShortString');
  Inc(ImplicitTest2ShortString);
  Result := '';
end;

operator :=(const aArg: TTest1): ShortString;
begin
  Writeln('TTest1 Implicit ShortString');
  Inc(ImplicitTest1ShortString);
  Result := '';
end;

operator :=(const aArg: TTest2): TString80;
begin
  Writeln('TTest2 Implicit TString80');
  Inc(ImplicitTest2String80);
  Result := '';
end;

var
  t1: TTest1;
  t2: TTest2;
  s80: TString80;
begin
  s80 := t1;
  if ImplicitTest1ShortString <> 1 then
    Halt(1);
  s80 := t2;
  if ImplicitTest2ShortString <> 1 then
    Halt(2);
  Writeln('ok');
end.
