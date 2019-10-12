program testsqlscanner;

{$mode objfpc}{$H+}

uses
  Classes, consoletestrunner, tcsqlscanner, 
  fpsqltree, fpsqlscanner, fpsqlparser,
  tcparser, tcgensql;

type

  { TLazTestRunner }

  TMyTestRunner = class(TTestRunner)
  protected
  // override the protected methods of TTestRunner to customize its behavior
  end;

var
  Application: TMyTestRunner;


{$R *.res}

begin
  DefaultFormat:=fPlain;
  DefaultRunAllTests:=True;
  Application := TMyTestRunner.Create(nil);
  Application.Initialize;
  Application.Run;
  Application.Free;
end.
