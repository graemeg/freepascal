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

    P:=AddPackage('fcl-js');
    P.ShortName:='fcjs';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Author := 'Michael Van Canneyt';
    P.License := 'LGPL with FPC modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := 'michael@freepascal.org';
    P.Description := 'Javascript scanner/parser/syntax tree units';
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-json');

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('jsbase.pp');
    T:=P.Targets.AddUnit('jstoken.pp');
    T:=P.Targets.AddUnit('jstree.pp');
    T:=P.Targets.AddUnit('jssrcmap.pas');
    T:=P.Targets.AddUnit('jsscanner.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('jsparser.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('jswriter.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('jsminifier.pp');
      T.ResourceStrings:=true;
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
