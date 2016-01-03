program testjs;

{$mode objfpc}{$H+}

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  cwstring,Classes, consoletestrunner, tcscanner, jsparser, jsscanner, jstree, jsbase,
  tcparser, jswriter, tctextwriter, tcwriter, jstoken;

var
  Application: TTestRunner;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Classes, consoletestrunner, tcscanner, jsparser, jsscanner, jstree, jsbase,
  tcparser;

type

  { TLazTestRunner }

  TMyTestRunner = class(TTestRunner)
  protected
  // override the protected methods of TTestRunner to customize its behavior
  end;

var
  Application: TMyTestRunner;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$IFDEF WINDOWS}{$R testjs.rc}{$ENDIF}

begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  DefaultFormat:=fplain;
  DefaultRunAllTests:=True;
  Application := TTestRunner.Create(nil);
  Application.Initialize;
=======
  Application := TMyTestRunner.Create(nil);
  Application.Initialize;
  Application.Title := 'FPCUnit Console test runner';
>>>>>>> graemeg/cpstrnew
=======
  Application := TMyTestRunner.Create(nil);
  Application.Initialize;
  Application.Title := 'FPCUnit Console test runner';
>>>>>>> graemeg/cpstrnew
=======
  Application := TMyTestRunner.Create(nil);
  Application.Initialize;
  Application.Title := 'FPCUnit Console test runner';
>>>>>>> graemeg/cpstrnew
=======
  Application := TMyTestRunner.Create(nil);
  Application.Initialize;
  Application.Title := 'FPCUnit Console test runner';
>>>>>>> origin/cpstrnew
  Application.Run;
  Application.Free;
end.
