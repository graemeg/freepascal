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

    P:=AddPackage('bfd');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Author := 'Library: Cygnus Support, header: by Uli Tessel';
    P.License := 'Library: GPL2 or later, header: LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Binary File Descriptor library.';
    P.NeedLibC:= true;
    P.OSes := [beos,haiku,freebsd,darwin,iphonesim,solaris,netbsd,openbsd,linux,aix,dragonfly];

=======
    P.Directory:='bfd';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
>>>>>>> graemeg/fixes_2_2
=======
    P.Directory:='bfd';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
>>>>>>> origin/fixes_2_2
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('bfd.pas');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
