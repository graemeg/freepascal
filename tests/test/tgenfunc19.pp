program tgenfunc19;

{$mode objfpc}

uses
  ugenfunc19;

type
  TTest2 = class(TTest)
    class function Test: LongInt;
  end;

  TTest2Helper = class helper for TTest2
    class function Test: LongInt;
  end;

class function TTest2.Test: LongInt;
begin
  Result := 3;
end;

class function TTest2Helper.Test: LongInt;
begin
  Result := 4;
end;

begin
  if specialize DoTest<TTest> <> 2 then
    Halt(1);
  if specialize DoTest<TTest2> <> 3 then
    Halt(2);
  Writeln('Ok');
end.
