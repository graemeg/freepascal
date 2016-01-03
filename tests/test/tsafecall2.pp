<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %TARGET=win32,win64,wince,linux,android }
=======
{ %TARGET=win32,win64,wince}
>>>>>>> graemeg/cpstrnew
=======
{ %TARGET=win32,win64,wince}
>>>>>>> graemeg/cpstrnew
=======
{ %TARGET=win32,win64,wince}
>>>>>>> graemeg/cpstrnew
=======
{ %TARGET=win32,win64,wince}
>>>>>>> origin/cpstrnew
program tsafecall2;

{$mode objfpc}{$H+}

uses
  Classes, SysUtils;

// On Windows, safecall should behave like a stdcall, so it can be used for COM/
// ActiveX. On Unix systems, it should be cdecl so that it could be used with
// xpcom.

function SafecallProcedureAlias(AParam1,AParam2: integer):HRESULT; {$IFDEF windows}stdcall{$ELSE}cdecl{$ENDIF}; [external name '_SAFECALLPROCEDURE'];
procedure SafecallProcedure(AParam1,AParam2: integer); safecall; [public, alias: '_SAFECALLPROCEDURE'];
begin
  if (AParam1<>$123456) or (AParam2<>$654321) then
    halt(1);
end;

function SafecallFunctionAlias(AParam1,AParam2: integer; out _result: string):HRESULT; {$IFDEF windows}stdcall{$ELSE}cdecl{$ENDIF}; [external name '_SAFECALLFUNCTION'];
function SafecallFunction(AParam1,AParam2: integer): string; safecall; [public, alias: '_SAFECALLFUNCTION'];
begin
  if (AParam1<>$123456) or (AParam2<>$654321) then
    halt(2)
  else
    result := 'hello';
end;

var s: string;

begin
  try
    // The call is in a try..finally block. This is to test if the stack is
    // cleaned succesfully after the call. Without the try..finally it could
    // be that the stack is corrupted, but that this does not lead to any
    // detectable problems (exception)
    if SafecallProcedureAlias($123456,$654321) <> 0 then
      halt(11);
    if SafecallFunctionAlias($123456,$654321,s) <> 0 then
      halt(12);
    if s<>'hello' then halt(13);
  finally
    writeln('Ok');
  end;
end.

