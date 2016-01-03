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

    P:=AddPackage('libpng');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='lpng';
    P.Description := 'Interface unit for libpng - working with PNG image format.';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
<<<<<<< HEAD
    P.Version:='3.1.1';
=======
    P.Version:='2.2.2-0';
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
    P.Dependencies.Add('zlib');
    P.SourcePath.Add('src');
    P.OSes := AllUnixOSes-[qnx]+[win32,os2,emx];

    T:=P.Targets.AddUnit('png.pp');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='libpng';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('png.pp');
      with T.Dependencies do
        begin
          AddUnit('zlib');
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
