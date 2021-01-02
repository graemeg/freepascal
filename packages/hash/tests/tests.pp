{ %CONFIGFILE=fpcunit-console-defaults.ini testdefaults.ini }

program tests;

{$mode objfpc}

uses
  consoletestrunner, TestsHMAC, HMAC;

var
  Application: TTestRunner;
begin
  Application := TTestRunner.Create(nil);
  Application.Initialize;
  Application.Run;
  Application.Free;
end.

