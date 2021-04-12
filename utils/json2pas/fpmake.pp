{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_json2pas(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('utils-json2pas');
    P.Dependencies.Add('fcl-json');

    P.ShortName:='jsnp';
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Author := 'Michael Van Canneyt';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'A utility to create Object Pascal classes files from sample JSON files.';
    P.NeedLibC:= false;

    P.Directory:=ADirectory;
    P.Version:='3.3.1';

    T:=P.Targets.AddProgram('json2pas.pp');
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_json2pas('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




