uses
{$ifdef unix}
  cwstring,
{$endif unix}
  SysUtils;
procedure testansi;
var
  s, q: ansistring;
begin
  s := 'hell';
  s := s+'o';
  q := '"';
  q := q+'''';
  s := s + q + s + 'abc';
  if (s <> 'hello"''helloabc') then
    halt(1);
  s := 'hell';
  s := s+'o';
  s := q+s+q;
  if (s <> '"''hello"''') then
    halt(2);
end;


procedure testshort;
var
  s, q: shortstring;
begin
  s := 'hell';
  s := s+'o';
  q := '"';
  q := q+'''';
  s := s + q + s + 'abc';
  if (s <> 'hello"''helloabc') then
    halt(3);
  s := 'hell';
  s := s+'o';
  s := q+s+q;
  if (s <> '"''hello"''') then
    halt(4);
end;

begin
  testansi;
  testshort;
end.
