{$mode objfpc}
program Project1;

{$ifdef cpu64}
const
    lowlimit = 536879040;

procedure test(i : integer);
const
  myarray : array[lowlimit..lowlimit+2] of integer = (1,2,3);
begin
  if @myarray[i]<>@myarray then
    halt(1);
end;

begin
  test(lowlimit);
{$else}
begin
{$endif}
end.
