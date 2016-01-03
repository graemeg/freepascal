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
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='fclu';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Dependencies.Add('paszlib');
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('libtar');
    P.Dependencies.Add('univint',[Darwin,iPhoneSim]);

    P.Author := ' Dean Zobec, Michael van Canneyt';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Unit testing system inspired by JUnit of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes := P.OSes - [embedded,nativent,msdos,win16];
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-fpcunit';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';

    P.Dependencies.Add('paszlib');
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-xml');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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
<<<<<<< HEAD
<<<<<<< HEAD
      T.ResourceStrings:=true;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
