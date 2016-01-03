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

    P:=AddPackage('fcl-extra');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Dependencies.Add('fcl-base');
    P.OSes:=[Win32,Win64]+AllUnixOSes;
    P.Dependencies.Add('winunits-jedi',[Win32,Win64]);
    P.Dependencies.Add('winunits-base',[Win32,Win64]);
    P.Dependencies.Add('univint',[darwin,iPhoneSim]);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    P.Directory:='fcl-extra';
{$endif ALLPACKAGES}
    P.Version:='2.2.2-0';
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('winunits-jedi',[Win32,Win64]);
    P.Dependencies.Add('winunits-base',[Win32,Win64]);
    P.Dependencies.Add('univint',[darwin]);
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

    P.Author := '<various>';
    P.License := 'LGPL with modification, ';
    P.Email := '';
    P.Description := 'Extra libraries of Free Component Libraries(FCL), FPC''s OOP library.';
    P.NeedLibC:= false;

    P.SourcePath.Add('src');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    P.SourcePath.Add('src/win',AllWindowsOSes);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    P.IncludePath.Add('src/$(OS)',AllOSes-AllWindowsOSes-AllUnixOSes);
    P.IncludePath.Add('src/unix',AllUnixOSes);
    P.IncludePath.Add('src/win',AllWindowsOSes);
    P.IncludePath.Add('src/dummy',AllOSes);

    T:=P.Targets.AddUnit('daemonapp.pp',AllWindowsOSes+AllUnixOSes);
      with T.Dependencies do
        begin
          AddInclude('daemonapp.inc');
        end;
      T.ResourceStrings:=true;

    // Windows units
    T:=P.Targets.AddUnit('ServiceManager.pas',[Win32,Win64]);
      T.ResourceStrings:=true;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}

