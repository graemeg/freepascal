{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_unicode(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('utils-unicode');
    P.ShortName:='ucd';
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Author := 'Inoussa OUEDRAOGO';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Utilities for processing the Unicode consortium data files for use with FPC.';

{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('rtl');
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');

    // P.OSes:=[win32, win64, linux, darwin, os2, emx];

    T := P.Targets.AddImplicitUnit('helper.pas');
    T.ResourceStrings := true;
    T.Install := false;
    T := P.Targets.AddImplicitUnit('cldrtxt.pas');
    T.Install := false;
    T := P.Targets.AddImplicitUnit('cldrxml.pas');
    T.Install := false;
    T := P.Targets.AddImplicitUnit('unicodeset.pas');
    T.ResourceStrings := true;
    T.Install := false;
    T := P.Targets.AddImplicitUnit('uca_test.pas');
    T.Install := false;
    T := P.Targets.AddImplicitUnit('cldrhelper.pas');
    T.Install := false;
    T.ResourceStrings:=true;
    T := P.Targets.AddImplicitUnit('cldrtest.pas');
    T.Install := false;
    T := P.Targets.AddImplicitUnit('grbtree.pas');
    T.Install := false;
    T := P.Targets.AddImplicitUnit('trie.pas');
    T.Install := false;

    T:=P.Targets.AddProgram('cldrparser.lpr');
    T:=P.Targets.AddProgram('unihelper.lpr');
    T:=P.Targets.AddProgram('gbpparser.lpr');
    T:=P.Targets.AddProgram('eawparser.lpr');

    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_unicode('');
  Installer.Run;
end.
{$endif ALLPACKAGES}
