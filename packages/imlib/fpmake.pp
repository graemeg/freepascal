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

    P:=AddPackage('imlib');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='imlb';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';

    P.Author := 'Library: Carsten Haitzler, header: ?';
    P.License := 'Library: LGPL 2 or later, header: LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Headers to imlib, an efficient bitmap manipulation program';
    P.NeedLibC:= true;  // true for headers that indirectly link to libc?

    P.OSes:=AllUnixOSes+[OS2,EMX]-[darwin,iphonesim,Android];
    if Defaults.CPU<>arm then
      P.OSes := P.OSes + [darwin];

=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='imlib';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.OSes:=AllUnixOSes;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    
    P.Dependencies.Add('gtk1');
    P.Dependencies.Add('x11');

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('gdk_imlib.pp');
    T:=P.Targets.AddUnit('imlib.pp');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
