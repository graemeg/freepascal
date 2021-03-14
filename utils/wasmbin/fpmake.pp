{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_wasmbin(const ADirectory: string);

Var
  P : TPackage;
  PT,T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('utils-wasmbin');
    P.ShortName := 'wsbn';

    P.Author := 'Free Pascal Team';
    P.License := 'GPL';
    P.HomepageURL := 'www.freepascal.org';
    P.Description := 'Collection of WebAssembly binary utils.';
    P.Email := '';
    P.NeedLibC:= false;

    P.Directory:=ADirectory;
    P.Version:='3.3.1';
    P.OSes:=AllUnixOSes+AllBSDOSes+AllWindowsOSes-[WinCE];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];
    P.Dependencies.Add('fcl-base');
    PT:=P.Targets.AddProgram('wasa.pas');
    PT:=P.Targets.AddProgram('wasmtool.lpr');
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_wasmbin('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




