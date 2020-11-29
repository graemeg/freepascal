program toperator91;

{$mode delphi}

type
  TString80 = String[80];
  TString90 = String[90];
  TString40 = String[40];
  TString100 = String[100];

  TTest = record
    class operator Explicit(const aArg: TTest): TString80;
    class operator Explicit(const aArg: TTest): TString90;
    class operator Explicit(const aArg: TTest): ShortString;
    class operator Implicit(const aArg: TTest): TString80;
    class operator Implicit(const aArg: TTest): TString90;
    class operator Implicit(const aArg: TTest): ShortString;
  end;

var
  ExplicitString80: LongInt;
  ExplicitString90: LongInt;
  ExplicitShortString: LongInt;
  ImplicitString80: LongInt;
  ImplicitString90: LongInt;
  ImplicitShortString: LongInt;

class operator TTest.Explicit(const aArg: TTest): TString80;
begin
  Writeln('TString80 Explicit');
  Inc(ExplicitString80);
  Result := '';
end;

class operator TTest.Explicit(const aArg: TTest): TString90;
begin
  Writeln('TString90 Explicit');
  Inc(ExplicitString90);
  Result := '';
end;

class operator TTest.Explicit(const aArg: TTest): ShortString;
begin
  Writeln('ShortString Explicit');
  Inc(ExplicitShortString);
  Result := '';
end;

class operator TTest.Implicit(const aArg: TTest): TString80;
begin
  Writeln('TString80 Implicit');
  Inc(ImplicitString80);
  Result := '';
end;

class operator TTest.Implicit(const aArg: TTest): TString90;
begin
  Writeln('TString90 Implicit');
  Inc(ImplicitString90);
  Result := '';
end;

class operator TTest.Implicit(const aArg: TTest): ShortString;
begin
  Writeln('ShortString Implicit');
  Inc(ImplicitShortString);
  Result := '';
end;

var
  s80: TString80;
  s90: TString90;
  s40: TString40;
  s100: TString100;
  t: TTest;
begin
  // Explicit
  s80 := TString80(t);
  if ExplicitString80 <> 1 then
    Halt(1);
  s90 := TString90(t);
  if ExplicitString90 <> 1 then
    Halt(2);
  s40 := TString40(t);
  if ImplicitShortString <> 1 then
    Halt(3);
  s100 := TString100(t);
  if ImplicitShortString <> 2 then
    Halt(4);
  // Implicit
  s80 := t;
  if ImplicitShortString <> 3 then
    Halt(5);
  s90 := t;
  if ImplicitShortString <> 4 then
    Halt(6);
  s40 := t;
  if ImplicitShortString <> 5 then
    Halt(7);
  s100 := t;
  if ImplicitShortString <> 6 then
    Halt(8);
  Writeln('ok');
end.
