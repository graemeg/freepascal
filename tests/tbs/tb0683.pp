{$ifndef SKIP_CURRENCY_TEST}
var
  c: currency;
  co: comp;
  i: int64;
begin
  c:=10.25;
  co:=12;
  i:=trunc(c);
  if i<>10 then
    halt(1);
  i:=trunc(co);
  if i<>12 then
    halt(2);
  i:=round(co);
  if i<>12 then
   halt(3);
end.
{$else}
begin
end.
{$endif}
