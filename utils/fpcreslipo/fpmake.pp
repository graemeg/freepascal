{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_fpcreslipo(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('utils-fpcreslipo');
    P.ShortName:='fprl';
    P.Description:='Free Pascal External Resource Thinner';
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,watcom,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Author := 'Giulio Bernardi';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';

    P.Directory:=ADirectory;
    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-res');

    // P.OSes:=[darwin, iphonesim,ios];

    P.Targets.AddImplicitUnit('msghandler.pp').install := false;
    P.Targets.AddImplicitUnit('paramparser.pp').install := false;
    P.Targets.AddImplicitUnit('sourcehandler.pp').install := false;

    T:=P.Targets.AddProgram('fpcreslipo.pp');

    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_fpcreslipo('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




