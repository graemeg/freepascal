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

    P:=AddPackage('pastojs');
    P.ShortName := 'p2js';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}

    P.Version:='3.3.1';
    P.OSes:=AllUnixOSes+AllBSDOSes+AllWindowsOSes-[WinCE];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Dependencies.Add('paszlib');
    P.Dependencies.Add('fcl-js');
    P.Dependencies.Add('fcl-json');
    P.Dependencies.Add('fcl-passrc');
    P.Dependencies.Add('fcl-process');
    Defaults.Options.Add('-Sc');

    P.Author := 'Free Pascal development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
    P.SupportBuildModes := [bmOneByOne];

    P.Options.Add('-S2h');

    T:=P.Targets.AddUnit('pas2jsfiler.pp');
    T:=P.Targets.AddUnit('fppas2js.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('fppjssrcmap.pp');
    T:=P.Targets.AddUnit('pas2jsfs.pp');
    T:=P.Targets.AddUnit('pas2jsutils.pp');
    T:=P.Targets.AddUnit('pas2jsfilecache.pp');
      T.Dependencies.AddUnit('pas2jsfs');
      T.Dependencies.AddUnit('pas2jsutils');
    T:=P.Targets.AddUnit('pas2jsfileutils.pp');
      T.Dependencies.AddInclude('pas2js_defines.inc');
      T.Dependencies.AddInclude('pas2jsfileutilsunix.inc',AllUnixOSes);
      T.Dependencies.AddInclude('pas2jsfileutilswin.inc',AllWindowsOSes);
    T:=P.Targets.AddUnit('pas2jslogger.pp');
    T:=P.Targets.AddUnit('pas2jspparser.pp');
    T:=P.Targets.AddUnit('pas2jsuseanalyzer.pp');
    T:=P.Targets.AddUnit('pas2jscompiler.pp');
    T:=P.Targets.AddUnit('pas2jsresstrfile.pp');
      T.ResourceStrings := True;
    T:=P.Targets.AddUnit('pas2jsresources.pp');
    T:=P.Targets.AddUnit('pas2jshtmlresources.pp');
    T:=P.Targets.AddUnit('pas2jsjsresources.pp');
    T:=P.Targets.AddUnit('pas2jsfscompiler.pp');
      T.Dependencies.AddUnit('pas2jscompiler');
    T:=P.Targets.AddUnit('pas2jspcucompiler.pp');
      T.Dependencies.AddUnit('pas2jsfscompiler');
    T:=P.Targets.AddUnit('pas2jscompilercfg.pp');
      T.Dependencies.AddUnit('pas2jscompiler');
    T:=P.Targets.AddUnit('pas2jscompilerpp.pp');
      T.Dependencies.AddUnit('pas2jscompiler');
    T:=P.Targets.AddUnit('pas2jslibcompiler.pp');
      T.Dependencies.AddUnit('pas2jspcucompiler');
      T.Dependencies.AddUnit('pas2jscompilercfg');
      T.Dependencies.AddUnit('pas2jscompilerpp');
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
