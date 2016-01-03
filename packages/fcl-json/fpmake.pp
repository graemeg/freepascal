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

    P:=AddPackage('fcl-json');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='fclj';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('rtl-objpas');
    P.Author := 'Michael van Canneyt';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Json interfacing, part of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16];
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-json';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';

    P.Dependencies.Add('fcl-base');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('fpjson.pp');
<<<<<<< HEAD
<<<<<<< HEAD
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('jsonconf.pp');
      T.ResourceStrings:=true;
=======
    T:=P.Targets.AddUnit('jsonconf.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('jsonconf.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('fpjson');
          AddUnit('jsonparser');
        end;
    T:=P.Targets.AddUnit('jsonparser.pp');
<<<<<<< HEAD
<<<<<<< HEAD
      T.ResourceStrings:=true;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('fpjson');
          AddUnit('jsonscanner');
        end;
    T:=P.Targets.AddUnit('jsonscanner.pp');
<<<<<<< HEAD
<<<<<<< HEAD
      T.ResourceStrings:=true;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpjsonrtti.pp');
      T.ResourceStrings:=true;
      with T.Dependencies do
        begin
          AddUnit('fpjson');
          AddUnit('jsonparser');
        end;
      T.ResourceStrings:=true;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    P.ExamplePath.Add('examples');
    T:=P.Targets.AddExampleProgram('confdemo.pp');
    T:=P.Targets.AddExampleProgram('parsedemo.pp');
    T:=P.Targets.AddExampleProgram('simpledemo.pp');

    // simpledemo.lpi
    // confdemo.lpi
    // parsedemo.lpi
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD



=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
