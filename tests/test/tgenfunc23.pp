program tgenfunc;

{$mode objfpc}

var
  TestTCalled: LongInt;
  TestArrayOfTCalled: LongInt;

generic procedure Test<T>(const aArg: T);
begin
  Inc(TestTCalled);
end;

generic procedure Test<T>(const aArg: array of T);
var
  i: SizeInt;
begin
  for i := 0 to High(aArg) do begin
    specialize Test<T>(aArg[i]);
  end;
  Inc(TestArrayOfTCalled);
end;

begin
  TestTCalled := 0;
  TestArrayOfTCalled := 0;
  specialize Test<LongInt>(1);
  if TestTCalled <> 1 then
    Halt(1);
  specialize Test<LongInt>([1, 2, 3]);
  if TestArrayOfTCalled <> 1 then
    Halt(2);
  if TestTCalled <> 4 then
    Halt(3);
end.
