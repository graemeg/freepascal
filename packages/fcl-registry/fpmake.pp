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

    P:=AddPackage('fcl-registry');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='fclr';
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
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');

    P.Author := 'FPC development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Windows registry + emulation parts of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-registry';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('registry.pp');
      with T.Dependencies do
        begin
          AddInclude('regdef.inc');
<<<<<<< HEAD
<<<<<<< HEAD
          AddInclude('winreg.inc',AllWindowsOSes);
          AddInclude('xregreg.inc',AllOSes-AllWindowsOSes);
          AddInclude('regini.inc');
          AddUnit('xmlreg');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('xmlreg.pp');

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('tests/testbasics.pp');
    P.Targets.AddExampleProgram('tests/regtestframework.pp');
    // 'tests/Makefile
    // 'tests/Makefile.fpc
=======
=======
>>>>>>> origin/fixes_2_2
          AddInclude('xregreg.inc');
          AddInclude('regini.inc');
          AddUnit('inifiles');
          AddUnit('xmlreg');
        end;
    T:=P.Targets.AddUnit('xmlreg.pp');
      with T.Dependencies do
        begin
          AddUnit('dom');
          AddUnit('xmlread');
          AddUnit('xmlwrite');
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
