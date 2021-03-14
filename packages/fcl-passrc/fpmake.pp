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

    P:=AddPackage('fcl-passrc');
    P.ShortName:='fcls';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-base');
    P.Author := 'Sebastian Guenther';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Pascal parsing parts of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('pastree.pp');
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('pscanner.pp');
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('pparser.pp');
      T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('pastree');
          AddUnit('pscanner');
        end;
    T:=P.Targets.AddUnit('pasresolver.pp');
      with T.Dependencies do
        begin
          AddUnit('pastree');
          AddUnit('pscanner');
          AddUnit('pparser');
        end;
    T:=P.Targets.AddUnit('pastounittest.pp');
      with T.Dependencies do
        begin
          AddUnit('pparser');
          AddUnit('pastree');
          AddUnit('pscanner');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('passrcutil.pp');
      with T.Dependencies do
        begin
          AddUnit('pparser');
          AddUnit('pastree');
          AddUnit('pscanner');
        end;
    T.ResourceStrings := False;

    T:=P.Targets.AddUnit('paswrite.pp');
      with T.Dependencies do
        begin
          AddUnit('pastree');
        end;
    T:=P.Targets.AddUnit('pasresolveeval.pas');
      with T.Dependencies do
        begin
          AddUnit('pastree');
          AddUnit('pscanner');
        end;
    T.ResourceStrings := True;
    T:=P.Targets.AddUnit('pasuseanalyzer.pas');
      with T.Dependencies do
        begin
          AddUnit('pastree');
          AddUnit('pasresolver');
        end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
