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

    P:=AddPackage('dts');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';

    P.Author := 'Library: Gildas Bazin, header: Ivo Steinmann';
    P.License := 'Library: GPL2 or later, header: LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'a low-level interface to decoding audio frames encoded using DTS Coherent Acoustics';
    P.NeedLibC:= true;
    P.OSes := [linux];

=======
    P.Directory:='dts';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
>>>>>>> graemeg/fixes_2_2
=======
    P.Directory:='dts';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
>>>>>>> origin/fixes_2_2
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('dts.pas');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
