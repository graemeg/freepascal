{$mode objfpc}
{$inline on}
function f : longint;inline;
  begin
    result:=result*result;
  end;
  
begin
  writeln(f);
end.
