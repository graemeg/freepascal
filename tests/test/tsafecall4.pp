{ %TARGET=win32,win64,wince,linux,android }
{ %OPT=-CE }
program tsafecall4;

{$mode objfpc}{$H+}

uses
  Classes, SysUtils;

procedure SafecallProcedure(AParam1,AParam2: integer); safecall;
var i,j: double;
begin
  if (AParam1<>$123456) or (AParam2<>$654321) then
    halt(1);
  i := 1;
  j := 0;
  // division by zero, but no exception should be raised. Instead the function
  // result has to be <> 0
  i := i/j;
end;

function SafecallFunction(AParam1,AParam2: integer): string; safecall;
begin
  if (AParam1<>$123456) or (AParam2<>$654321) then
    halt(2);
  raise exception.create('Ignore and return non-zero');
end;

var
  s : string;
  pass : boolean;

begin
  pass := false;
  try
    SafecallProcedure($123456,$654321);
  except
    on E: ESafecallException do
      pass := true;
  end;
  if not pass then
    halt(11);

  pass := false;
  try
    s := SafecallFunction($123456,$654321);
  except
    on E: ESafecallException do
      pass := true;
  end;
  if not pass then
    halt(12);
end.

