{$ifdef fpc}{$mode delphi}{$H+}{$endif}
var C: Currency;
begin
  c:= 1000;
  c:= c*1.05;
  // at this point C=1050
  writeln(c:4:2);
  if c<>1050 then
    halt(1);
  writeln('ok');
end.
