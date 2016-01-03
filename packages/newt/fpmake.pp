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

    P:=AddPackage('newt');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.OSes:=[Linux];

    T:=P.Targets.AddUnit('newt.pp');

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('newt3.pas');
    P.Targets.AddExampleProgram('newt2.pas');
    P.Targets.AddExampleProgram('newt1.pas');

=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='newt';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('newt.pp');

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
