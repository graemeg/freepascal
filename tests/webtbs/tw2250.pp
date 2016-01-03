{ Source provided for Free Pascal Bug Report 2250 }
{ Submitted by "Konstantin Seiler" on  2002-12-04 }
{ e-mail: list@kseiler.de }
uses
{$ifdef unix}
  cwstring,
{$endif unix}
  SysUtils;
procedure stringbug;
var env:ansistring;
  procedure addenv(s:ansistring);
  begin
  // Uncomment next line and everything works as espected.
  //writeln(length(env));
      if length(env)=0 then env:=s
                       else env:=env+'|'+s;
  end;

begin
  env:='';
  addenv('first');
  addenv('second');
  addenv('third');
  // It schould write "first|second|third",
  // but only writes "third".
  writeln(env);
  if env<>'first|second|third' then
   begin
     writeln('ERROR!');
     halt(1);
   end;
end;

begin
  stringbug;
end.
