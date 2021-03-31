{ %opt=-O3 }
{$mode objfpc}
{$H+}
function Bar(const progress: single; divs: uint32): string;
const
    BarSym: array[boolean] of char = ('.', '#');
var
    i: int32;
begin
    SetLength(result, divs);
    for i := 0 to int32(divs) - 1 do
        pChar(result)[i] := BarSym[(progress >= (0.75 + i) / divs) or (i = int32(divs) - 1) and (progress >= 1)];
end;

var
    s: string;

begin
  if Bar(0.7, 10)<>'#######...' then
    halt(1);
end.
