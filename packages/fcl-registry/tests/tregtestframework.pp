{ %TARGET=win32,win64,wince,linux,solaris,openbsd }
program tregtestframework;

{$IFDEF FPC}
{$mode objfpc}{$H+}
{$ENDIF}

{$IFDEF WINDOWS}
{$APPTYPE CONSOLE}
{$ENDIF}

uses
  {$ifdef unix}
  cwstring,
  {$endif}
  SysUtils,
  fpcunit,  testreport, testregistry, consoletestrunner,
// Units wich contains the tests
  regtcxmlreg,
  regtestbasics;

Var
  A : TTestRunner;

begin
  DefaultFormat:=fPlain;
  DefaultRunAllTests:=True;
  A:=TTestRunner.Create(Nil);
  try
    A.Initialize;
    A.Run;
  finally
    A.Free;
  end;
end.
