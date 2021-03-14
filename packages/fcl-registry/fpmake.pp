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
    P.ShortName:='fclr';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');

    P.Author := 'FPC development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Windows registry + emulation parts of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
    T:=P.Targets.AddUnit('registry.pp');
      with T.Dependencies do
        begin
          AddInclude('regdef.inc');
          AddInclude('winreg.inc',AllWindowsOSes);
          AddInclude('xregreg.inc',AllOSes-AllWindowsOSes);
          AddInclude('regini.inc');
          AddUnit('xmlreg');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('xmlreg.pp');

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('tests/tregtestframework.pp');
    // 'tests/Makefile
    // 'tests/Makefile.fpc

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
