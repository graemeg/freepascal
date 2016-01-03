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

    P:=AddPackage('libgd');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='lgd';
    P.Description := 'Interface unit for library libgd - image processing';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.OSes := P.OSes - [embedded,nativent,msdos,win16];

    T:=P.Targets.AddUnit('gd.pas');

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('gdtestcgi.pp');
    P.Targets.AddExampleProgram('gdtest.pp');
    P.Sources.AddExampleFiles('examples/*',P.Directory,false,'.');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='libgd';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('gd.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
