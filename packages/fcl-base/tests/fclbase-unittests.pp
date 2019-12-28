program fclbase_unittests;

{$mode objfpc}{$H+}

uses
  Classes, consoletestrunner, tests_fptemplate, tchashlist,
  testexprpars, tcmaskutils, tcinifile, tccsvreadwrite,tcbufferedfilestream, tccsvdocument;

var
  Application: TTestRunner;

begin
  DefaultFormat:=fPlain;
  DefaultRunAllTests:=True;
  Application := TTestRunner.Create(nil);
  Application.Initialize;
  Application.Title := 'FCL-Base unittests';
  Application.Run;
  Application.Free;
end.
