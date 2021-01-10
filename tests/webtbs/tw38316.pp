{ %opt=-gh }

program project1;

procedure P1(A: array of Integer);
begin
end;

procedure P2(A: array of Integer);
begin
  P1(A);
end;

var
  A: array [0..2] of Integer;
  i: Integer;
begin
  HaltOnNotReleased := true;
  for i := 0 to 10 do
    P2(A);
end.
