{$R-}
{$Q-}

const
  A1 = 1000000;
  B1 = 60;
  C1 = 6000000;
  T1 = A1*B1;
  T2 = A1*C1;

procedure dotest(P: longint);
var
  i64: Int64;
begin
  i64 := 0;
  i64 := i64 + P * B1 * int64(A1);
  writeln(i64);
  if i64 <> P*int64(T1) then
    Halt(1);

  i64 := 0;
  i64 := i64 + P * B1 * A1;
  writeln(i64);
  if i64 <> P*T1 then
    Halt(2);

  i64 := 0;
  i64 := i64 + P * C1 * A1;
  writeln(i64);
  if i64 <> P*T2 then
    Halt(3);
end;

begin
  dotest(1000000);
  writeln('OK.');
end.
