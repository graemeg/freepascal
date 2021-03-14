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

    P:=AddPackage('webidl');
    P.ShortName:='widl';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-base');
    P.Author := 'Michael Van Canneyt';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'WEB IDL parser and converter to Object Pascal classes';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('webidldefs.pp');
    T:=P.Targets.AddUnit('webidlscanner.pp');
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('webidlparser.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('webidldefs');
      AddUnit('webidlscanner');
      end;
    T:=P.Targets.AddUnit('webidltopas.pp');
    with T.Dependencies do
      begin
      AddUnit('webidldefs');
      AddUnit('webidlscanner');
      AddUnit('webidlparser');
      end;
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
