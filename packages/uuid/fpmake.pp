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

    P:=AddPackage('uuid');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.OSes := [linux];
    P.Dependencies.Add('rtl-extra');
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='uuid';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('libuuid.pp');
    T:=P.Targets.AddUnit('macuuid.pp');

<<<<<<< HEAD
<<<<<<< HEAD
    P.Sources.AddSrc('README.txt');

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('testlibuid.pp');
    P.Targets.AddExampleProgram('testuid.pp');
    P.Sources.AddExampleFiles('examples/*',P.Directory,false,'.');


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
