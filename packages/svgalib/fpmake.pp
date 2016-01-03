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

    P:=AddPackage('svgalib');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes := [beos,haiku,linux,freebsd,solaris,netbsd,openbsd,dragonfly];
=======
    P.Directory:='svgalib';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
>>>>>>> graemeg/fixes_2_2
=======
    P.Directory:='svgalib';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
>>>>>>> origin/fixes_2_2
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('svgalib.pp');
    T:=P.Targets.AddUnit('vgamouse.pp');

<<<<<<< HEAD
<<<<<<< HEAD
    P.Sources.AddSrc('README.txt');
 
    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('testvga.pp');
    P.Targets.AddExampleProgram('vgatest.pp');
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
