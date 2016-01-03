{$mode fpc}
uses
{$ifdef unix}
  cwstring,
{$endif unix}
  SysUtils;

resourcestring
  s = 'OK';

var t:ansistring;

begin
  t:=s;
  if t<>'OK' then
    begin
      writeln('Resourcestring error!');
      halt(1);
    end
  else
    writeln(s);
end.
