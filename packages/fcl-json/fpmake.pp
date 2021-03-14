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
    P.ShortName:='fclj';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('rtl-objpas');
    P.Author := 'Michael van Canneyt';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Json interfacing, part of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('fpjson.pp');
    T.ResourceStrings:=true;

    T:=P.Targets.AddUnit('jsonconf.pp');
    T.ResourceStrings:=true;
    with T.Dependencies do
      begin
      AddUnit('fpjson');
      AddUnit('jsonparser');
     end;
        
    T:=P.Targets.AddUnit('jsonparser.pp');
    T.ResourceStrings:=true;
    with T.Dependencies do
      begin
      AddUnit('fpjson');
      AddUnit('jsonscanner');
      AddUnit('jsonreader');
      end;
    T:=P.Targets.AddUnit('jsonreader.pp');
    T.ResourceStrings:=true;
    with T.Dependencies do
      begin
      AddUnit('fpjson');
      AddUnit('jsonscanner');
      end;
        
    T:=P.Targets.AddUnit('jsonscanner.pp');
    T.ResourceStrings:=true;
    
    T:=P.Targets.AddUnit('fpjsonrtti.pp');
    T.ResourceStrings:=true;
    with T.Dependencies do
      begin
      AddUnit('fpjson');
      AddUnit('jsonparser');
      end;
      
    T:=P.Targets.AddUnit('fpjsontopas.pp');
    T.ResourceStrings:=true;
    with T.Dependencies do
      begin
      AddUnit('fpjson');
      AddUnit('jsonparser');
      end;

    T:=P.Targets.AddUnit('jsonini.pp');
    with T.Dependencies do
      begin
      AddUnit('fpjson');
      AddUnit('jsonparser');
     end;
    T:=P.Targets.AddUnit('json2yaml.pp');
    with T.Dependencies do
      begin
      AddUnit('fpjson');
     end;

    P.ExamplePath.Add('examples');
    T:=P.Targets.AddExampleProgram('confdemo.pp');
    T:=P.Targets.AddExampleProgram('parsedemo.pp');
    T:=P.Targets.AddExampleProgram('simpledemo.pp');
    T:=P.Targets.AddExampleProgram('ini2json.pp');
    T:=P.Targets.AddExampleProgram('j2y.pp');

    // simpledemo.lpi
    // confdemo.lpi
    // parsedemo.lpi

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}



