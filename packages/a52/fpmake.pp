{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  P : TPackage;
  T : TTarget;
begin
<<<<<<< HEAD
<<<<<<< HEAD


  With Installer do
    begin
{$endif ALLPACKAGES}
    P:=AddPackage('a52');
    P.Author := 'Library: Michel Lespinasse and Aaron Holtzman, header: Ivo Steimann';
    P.License := 'Library: GPL2 or later, header: LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'A free library for decoding ATSC A/52 streams.';
    P.NeedLibC:= true;
    P.OSes := [linux,win32];

{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
=======
=======
>>>>>>> origin/fixes_2_2
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('a52');
{$ifdef ALLPACKAGES}
    P.Directory:='a52';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('a52.pas');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
