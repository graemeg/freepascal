{ %OPT=-O3 -Sew -vw }
{$mode objfpc}
{$inline on}

procedure test; inline;
begin
  exit;
end;

function f: longint;
begin
  test; // tt.pp(11,3) Warning: Function result variable does not seem to be initialized
  result:=4;
end;

begin
end.
