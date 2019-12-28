unit ugenfunc19;

{$mode objfpc}{$H+}

interface

type
  TTest = class
    class function Test: LongInt; static;
  end;

  TTestHelper = class helper for TTest
    class function Test: LongInt; static;
  end;

generic function DoTest<T: TTest>: LongInt;

implementation

class function TTest.Test: LongInt;
begin
  Result := 1;
end;

class function TTestHelper.Test: LongInt;
begin
  Result := 2;
end;

generic function DoTest<T>: LongInt;
begin
  Result := T.Test;
end;


end.

