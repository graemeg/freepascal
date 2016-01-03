<<<<<<< HEAD
<<<<<<< HEAD
{ %skiptarget=os2,emx }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$mode objfpc}
{$H+}

uses sysutils;

Function AnsiEndsStr(const ASubText, AText: string): Boolean;
begin
  Writeln('ZZ',ASubText,'XX ',AText,'YY');
 Result := AnsiCompareStr(Copy(AText,length(AText)-length(ASubText)+1,length(ASubText)),ASubText)=0;
end;

VAR
  s: WideString;
  t: WideString;
  err : boolean;
BEGIN
  s := 'This is a test.'#961#967;
  t := 'test.'#961#967;
  IF AnsiEndsStr(t, s) THEN
    WriteLn('OK.')
  ELSE
    err:=true;
  IF AnsiEndsStr('test.'#961#967, s) THEN
    WriteLn('OK.')
  ELSE
    err:=true;
  if err then
    WriteLn('Not OK.');
  if err then
    halt(1);
END.
