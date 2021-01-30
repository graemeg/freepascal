program tresolvertests;

{$mode objfpc}{$H+}

uses
  Classes, consoletestrunner, netdbtest;

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
  Application.Title:='resolvertests';
  Application.Run;
  Application.Free;
end.
