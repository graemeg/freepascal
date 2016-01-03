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

    P:=AddPackage('unixutil');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
<<<<<<< HEAD
    P.Version:='3.1.1';
    P.OSes:=[Linux];
    P.CPUs:=[i386];
    P.Dependencies.add('libc');
=======
    P.Version:='2.2.2-0';
    P.OSes:=[Linux];
    P.CPUs:=[i386];
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
    P.SourcePath.Add('src');
    T:=P.Targets.AddUnit('unixutils.pp');
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='unixutil';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('unixutils.pp');
      with T.Dependencies do
        begin
          AddUnit('libc');
        end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
