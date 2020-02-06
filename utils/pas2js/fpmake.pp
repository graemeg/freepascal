{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_pas2js(const ADirectory: string);

Var
  P : TPackage;
  PT,T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('utils-pas2js');

    P.Author := 'Free Pascal Team';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Description := 'Convert pascal sources to javascript.';
    P.Email := 'michael@freepascal.org';
    P.NeedLibC:= false;
    P.ShortName:='p2js';

    P.Directory:=ADirectory;
    P.Version:='3.3.1';
    P.OSes:=AllUnixOSes+AllBSDOSes+AllWindowsOSes-[WinCE];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];
    P.Dependencies.Add('fcl-json');
    P.Dependencies.Add('fcl-js');
    P.Dependencies.Add('fcl-passrc');
    P.Dependencies.Add('pastojs');
    P.Dependencies.Add('fcl-web');
    P.Dependencies.Add('webidl');
    PT:=P.Targets.AddProgram('pas2js.pp');
    PT:=P.Targets.AddLibrary('pas2jslib.pp');
    PT:=P.Targets.AddUnit('dirwatch.pp');
    PT:=P.Targets.AddUnit('httpcompiler.pp');
    PT.Dependencies.AddUnit('dirwatch');
    PT:=P.Targets.AddProgram('compileserver.pp');
    PT.Dependencies.AddUnit('httpcompiler');
    PT:=P.Targets.AddProgram('webidl2pas.pp');
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_pas2js('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




