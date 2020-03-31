{ %TARGET=win32,win64,wince,linux,android }
{ %OPT=-CE }
program tsafecall3;

{$mode objfpc}{$H+}

uses
  Classes, SysUtils;

function SafecallProcedureAlias(AParam1,AParam2: integer):HRESULT; {$IFDEF windows}stdcall{$ELSE}cdecl{$ENDIF}; [external name '_SAFECALLPROCEDURE'];
procedure SafecallProcedure(AParam1,AParam2: integer); safecall; [public, alias: '_SAFECALLPROCEDURE'];
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

function SafecallFunctionAlias(AParam1,AParam2: integer; out _result: string):HRESULT; {$IFDEF windows}stdcall{$ELSE}cdecl{$ENDIF}; [external name '_SAFECALLFUNCTION'];
function SafecallFunction(AParam1,AParam2: integer): string; safecall; [public, alias: '_SAFECALLFUNCTION'];
begin
  if (AParam1<>$123456) or (AParam2<>$654321) then
    halt(2);
  raise exception.create('Ignore and return non-zero');
end;

var
  s : string;

begin
  if SafecallProcedureAlias($123456,$654321) = 0 then
    halt(11);
  if SafecallFunctionAlias($123456,$654321,s) = 0 then
    halt(12);
end.

