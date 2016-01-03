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

    P:=AddPackage('tcl');
<<<<<<< HEAD
<<<<<<< HEAD
    P.Description := 'Interface unit for invoking Tcl interpreter using a library.';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.OSes := AllUnixOSes+AllWindowsOSes+[os2,emx]-[qnx,win16];

    T:=P.Targets.AddUnit('tcl80.pp');

    P.ExamplePath.Add('tests/');
    P.Targets.AddExampleProgram('tcl_demo.pp');
    // 'test.tcl
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='tcl';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('tcl80.pp');

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
