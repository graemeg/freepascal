program testjsonconf;

{$mode objfpc}{$H+}

uses
  Classes, consoletestrunner, jsonconftest, jsonconf;

type

  { TLazTestRunner }

  TMyTestRunner = class(TTestRunner)
  protected
  // override the protected methods of TTestRunner to customize its behavior
  end;

var
  Application: TMyTestRunner;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  DefaultFormat:=fPlain;
  DefaultRunAllTests:=True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Application := TMyTestRunner.Create(nil);
  Application.Initialize;
  Application.Run;
  Application.Free;
end.
