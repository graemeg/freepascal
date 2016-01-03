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

    P:=AddPackage('rexx');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.Oses:=[emx,os2];
    p.Targets.AddUnit('rexxsaa.pp');

    P.Sources.AddSrc('readme.txt');

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('callrexx.pas');
    // 'backward.fnc
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='rexx';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
    P.Oses:=[emx,os2];
//    P.Dependencies.Add('x11');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
