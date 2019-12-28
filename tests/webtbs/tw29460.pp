program project1;

{$mode objfpc}{$H+}
{$modeswitch advancedrecords}

operator = (z1: LongInt; z2 : ansistring) b : boolean;
begin
  b := false;
end;

var
  i: longint;
  s: string;
begin
  if i = s then
    halt(1);
  writeln('ok');
end.
