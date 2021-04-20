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
    P:=AddPackage('fcl-mustache');
    P.ShortName:='mustache';

{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}

    P.Author := 'Michael Van Canneyt';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Mustache templates for FPC';
    P.NeedLibC:= false;
    P.SourcePath.Add('src');
    P.OSes:=P.OSes-[embedded,win16,msdos,nativent,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Dependencies.Add('rtl-objpas');
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-json');
    P.Dependencies.Add('fcl-db');
    P.Version:='3.3.1';
    T:=P.Targets.AddUnit('src/fpmustache.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('src/fpdbmustache.pp');
      T.ResourceStrings:=true;
      T.Dependencies.AddUnit('fpmustache');
    T:=P.Targets.AddUnit('src/fpexmustache.pp');
      T.ResourceStrings:=true;
      T.Dependencies.AddUnit('fpmustache');
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
