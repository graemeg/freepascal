{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_ihxutil(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('utils-ihxutil');
    P.ShortName:='ihxu';
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Author := 'Nikolay Nikolov';
    P.License := 'GPL';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'A tool to convert Intel HEX Format files different formats used on various Z80 systems.';
    P.NeedLibC:= false;

    P.Directory:=ADirectory;
    P.Version:='3.3.1';

    P.Dependencies.Add('fcl-base');

    T:=P.Targets.AddProgram('ihxutil.lpr');
    T.Dependencies.AddUnit('ihxreader');
    T.Dependencies.AddUnit('tzxwriter');
    T.Dependencies.AddUnit('zxbasic');
    P.Targets.AddUnit('ihxreader.pas').Install := false;
    P.Targets.AddUnit('tzxwriter.pas').Install := false;
    P.Targets.AddUnit('zxbasic.pas').Install := false;
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_ihxutil('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




