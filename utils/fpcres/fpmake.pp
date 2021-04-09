{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_fpcres(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('utils-fpcres');
    P.ShortName:='fprs';
    P.OSes:=AllOSes-[atari,embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql];
    //P.OSes:=[win32,win64,wince,haiku,linux,freebsd,openbsd,netbsd,darwin,iphonesim,ios,solaris,os2,emx,aix,aros,amiga,morphos];

    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Author := 'Giulio Bernardi';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Free Pascal Resource Converter.';
    P.NeedLibC:= false;

    P.Directory:=ADirectory;
    P.Version:='3.3.1';

    P.Dependencies.Add('fcl-res');
    P.Dependencies.Add('paszlib');

    T:=P.Targets.AddProgram('fpcres.pas');

    T.Dependencies.AddUnit('closablefilestream');
    T.Dependencies.AddUnit('msghandler');
    T.Dependencies.AddUnit('paramparser');
    T.Dependencies.AddUnit('sourcehandler');
    T.Dependencies.AddUnit('target');

    T:=P.Targets.AddProgram('fpcjres.pas');

    T.Dependencies.AddUnit('closablefilestream');
    T.Dependencies.AddUnit('msghandler');
    T.Dependencies.AddUnit('paramparser');
    T.Dependencies.AddUnit('sourcehandler');
    T.Dependencies.AddUnit('target');
    T.Dependencies.AddUnit('jarsourcehandler');

    P.Targets.AddUnit('closablefilestream.pas').install:=false;
    P.Targets.AddUnit('msghandler.pas').install:=false;
    P.Targets.AddUnit('paramparser.pas').install:=false;
    P.Targets.AddUnit('sourcehandler.pas').install:=false;
    P.Targets.AddUnit('target.pas').install:=false;
    P.Targets.AddUnit('jarsourcehandler.pas').install:=false;
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_fpcres('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




