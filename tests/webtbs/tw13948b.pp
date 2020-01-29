{ %opt=-O2 }

type
  tr = array[0..1] of longint;

function f: tr;
begin
  f[0]:=5;
  f[1]:=6;
end;

procedure test;
var
  r: tr;
begin
  r[0]:=1;
  r[1]:=2;
  r[0]:=f[0];
  writeln(r[0]);
  writeln(r[1]);
  if (r[0]<>5) then
    halt(1);
  if (r[1]<>2) then
    halt(2);
  r[0]:=f[1];
  writeln(r[0]);
  writeln(r[1]);
  if (r[0]<>6) then
    halt(1);
  if (r[1]<>2) then
    halt(2);
end;

begin
  test;
end.
