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

    P:=AddPackage('aspell');
<<<<<<< HEAD
<<<<<<< HEAD

{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}

    P.Version:='3.1.1';

    P.Author := 'header:Aleš Katona, library: Kevin Atkinson';
    P.License := 'header: LGPL with modification, library: LGPL 2.0 or 2.1';
    P.HomepageURL := 'www.freepascal.org';
    P.OSes := [beos,haiku,freebsd,darwin,iphonesim,netbsd,openbsd,linux,win32,aix,dragonfly];
    P.Email := '';
    P.Description := 'The New Aspell, spelling library';
    P.NeedLibC:= true;

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('aspell.pp');
    T.Dependencies.AddInclude('src/aspelltypes.inc');
    T:=P.Targets.AddUnit('aspelldyn.pp');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='aspell';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('aspell.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('spellcheck.pp');
    with T.Dependencies do
      begin
        AddUnit('aspell');
       end;

    P.Sources.AddSrc('LICENSE');
    P.Sources.AddSrc('LICENSE.ADDON');
<<<<<<< HEAD
<<<<<<< HEAD

    P.ExamplePath.Add('examples');
    T:=P.Targets.AddExampleProgram('example.pas');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
