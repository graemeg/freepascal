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

    P:=AddPackage('fcl-fpcunit');
    P.ShortName:='fclu';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('paszlib');
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('libtar');
    P.Dependencies.Add('univint',[Darwin,iPhoneSim,ios]);

    P.Author := ' Dean Zobec, Michael van Canneyt';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Unit testing system inspired by JUnit of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes := P.OSes - [embedded,nativent,msdos,win16,macosclassic,palmos,symbian,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('digesttestreport.pp');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
          AddUnit('fpcunitreport');
          AddUnit('testutils');
        end;
    T:=P.Targets.AddUnit('fpcunit.pp');
      T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddInclude('DUnitCompatibleInterface.inc');
          AddInclude('DUnitCompatibleInterface.inc');
          AddUnit('testutils');
        end;
    T:=P.Targets.AddUnit('fpcunitreport.pp');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
        end;
    T:=P.Targets.AddUnit('latextestreport.pp');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
          AddUnit('fpcunitreport');
        end;
    T:=P.Targets.AddUnit('plaintestreport.pp');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
          AddUnit('fpcunitreport');
        end;
    T:=P.Targets.AddUnit('testdecorator.pp');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
        end;
    T:=P.Targets.AddUnit('testregistry.pp');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
          AddUnit('testdecorator');
        end;
    T:=P.Targets.AddUnit('testreport.pp');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
          AddUnit('testutils');
        end;
    T:=P.Targets.AddUnit('testutils.pp');
    T:=P.Targets.AddUnit('ubmockobject.pp');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
        end;
    T:=P.Targets.AddUnit('xmlreporter.pas');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
          AddUnit('testutils');
        end;
    T:=P.Targets.AddUnit('xmltestreport.pp');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
          AddUnit('fpcunitreport');
          AddUnit('testutils');
        end;
    T:=P.Targets.AddUnit('consoletestrunner.pas');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
          AddUnit('fpcunitreport');
          AddUnit('testutils');
          AddUnit('xmltestreport');
          AddUnit('latextestreport');
          AddUnit('plaintestreport');
        end;
    T:=P.Targets.AddUnit('simpletestrunner.pas');
      with T.Dependencies do
        begin
          AddUnit('fpcunit');
          AddUnit('fpcunitreport');
          AddUnit('plaintestreport');
        end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
