{ %CONFIGFILE=fpcunit-console-defaults.ini testdefaults.ini }

program testregexpr;

{$mode objfpc}{$H+}

uses
  Classes, consoletestrunner, tcregexp;

type

  { TMyTestRunner }

  TMyTestRunner = class(TTestRunner)
  protected
  // override the protected methods of TTestRunner to customize its behavior
  end;

var
  Application: TMyTestRunner;

begin
  DefaultFormat:=fPlain;
  DefaultRunAllTests:=True;
  Application := TMyTestRunner.Create(nil);
  Application.Initialize;
  Application.Title:='testregexpr';
  Application.Run;
  Application.Free;
end.
