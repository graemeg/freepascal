{ %fail }
function f(s: string): string;
begin
  f := '''' + s + '''';
end;

function f(s: string): integer;
begin
  Val(s,f);
end;

begin
end.
