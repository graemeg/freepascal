{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  T : TTarget;
  P : TPackage;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('fcl-async');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='fcla';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';

    P.Author := 'Mostly Sebastian Guenther';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Asynchronous event management of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllUnixOses;

=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-async';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    P.SourcePath.Add('src');
    P.SourcePath.Add('src/unix',AllUnixOSes);
    P.SourcePath.Add('src/win',AllWindowsOSes);
    P.SourcePath.Add('src/$(OS)',AllOSes-AllWindowsOSes-AllUnixOSes);
    P.IncludePath.Add('src');
    T:=P.Targets.AddUnit('fpasync.pp',AllUnixOSes);
      with T.Dependencies do
        begin
          AddUnit('libasync');
        end;
    T:=P.Targets.AddUnit('libasync.pp',AllUnixOSes);
      with T.Dependencies do
        begin
          AddInclude('libasynch.inc');
          AddInclude('libasync.inc');
        end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
