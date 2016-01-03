{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  P : TPackage;
  T : TTarget;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('zorba');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes := [linux,win32];
=======
    P.Directory:='zorba';
{$endif ALLPACKAGES}
    P.Version:='0.9.9';
>>>>>>> graemeg/cpstrnew
=======
    P.Directory:='zorba';
{$endif ALLPACKAGES}
    P.Version:='0.9.9';
>>>>>>> graemeg/cpstrnew
=======
    P.Directory:='zorba';
{$endif ALLPACKAGES}
    P.Version:='0.9.9';
>>>>>>> graemeg/cpstrnew
=======
    P.Directory:='zorba';
{$endif ALLPACKAGES}
    P.Version:='0.9.9';
>>>>>>> origin/cpstrnew
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('xqc.pas');
      with T.Dependencies do
        begin
          AddInclude('xqc_error.inc');
          AddInclude('xqc_static_context_consts.inc');
        end;
    T:=P.Targets.AddUnit('zorbadyn.pas');
      with T.Dependencies do
        begin
          AddUnit('xqc');
          AddInclude('zorba.inc');
          AddInclude('zorba_options.inc');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    T:=P.Targets.AddUnit('zorba.pas');
      with T.Dependencies do
        begin
          AddUnit('xqc');
          AddInclude('zorba.inc');
          AddInclude('zorba_options.inc');
        end;

//    P.ExamplePath.Add('tests/');
//    P.Targets.AddExampleProgram('testapiv3x.pp');
//    P.Targets.AddExampleProgram('test.pas');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
