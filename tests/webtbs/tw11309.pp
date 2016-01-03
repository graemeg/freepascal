{$ifdef fpc}
{$mode delphi}
{$endif}

uses
  SysUtils;

<<<<<<< HEAD
<<<<<<< HEAD
var
  csMoney : string;
=======
const
  csMoney = '58.195';
>>>>>>> graemeg/fixes_2_2
=======
const
  csMoney = '58.195';
>>>>>>> origin/fixes_2_2

Function Format_Currency_String1(sMoney : string) : string;
var
aCurrency : Currency;
begin
  TRY
aCurrency := strtoCurr(sMoney);
  EXCEPT
   on E: EConvertError do aCurrency := 0;
  END;
//result := CurrToStrF(currBetrag,ffFixed,2);
result := FloatToStrF(aCurrency,ffFixed,19,2);
end;

Function Format_Currency_String2(sMoney : string) : string;
var
aCurrency : real;
begin
  TRY
aCurrency := strtofloat(sMoney);
  EXCEPT
   on E: EConvertError do aCurrency := 0;
  END;
result := FloatToStrF(aCurrency,ffFixed,19,2);
end;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  csMoney:='58'+DecimalSeparator+'195';
  writeln(Format_Currency_String1(csMoney));
  writeln(Format_Currency_String2(csMoney));
  if Format_Currency_String1(csMoney)<>'58'+DecimalSeparator+'20' then
    halt(1);
  if Format_Currency_String2(csMoney)<>'58'+DecimalSeparator+'20' then
    halt(2);
  writeln('ok');
=======
=======
>>>>>>> origin/fixes_2_2
  writeln(Format_Currency_String1(csMoney));
  writeln(Format_Currency_String2(csMoney));
  if Format_Currency_String1(csMoney)<>'58.20' then
    halt(1);
  if Format_Currency_String2(csMoney)<>'58.20' then
    halt(2);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end.
