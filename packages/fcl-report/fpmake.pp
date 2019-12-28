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

    P:=AddPackage('fcl-report');
    P.ShortName:='fpre';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-image');
    P.Dependencies.Add('fcl-xml');
    P.Dependencies.Add('fcl-pdf');
    P.Dependencies.Add('fcl-json');
    P.Dependencies.Add('fcl-db');
    P.Dependencies.Add('fcl-web');
    P.Author := 'Michael Van Canneyt';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'GUI-independent Reporting Engine';
    P.NeedLibC:= false;
    P.OSes:=[linux, win32, win64, darwin, freebsd];
    P.SourcePath.Add('src');
{$IFDEF VER2_6}    
    T:=P.Targets.AddUnit('fprepexprpars.pp');
    T.ResourceStrings := True;
{$ENDIF}
    T:=P.Targets.AddUnit('fpreportstreamer.pp');
    T.ResourceStrings := True;
    
    T:=P.Targets.AddUnit('fpreporthtmlparser.pp');
   
    T:=P.Targets.AddUnit('fpreport.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('fpreportstreamer');
      AddUnit('fpreporthtmlparser');
      end;

    T:=P.Targets.AddUnit('fpreportdata.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      AddUnit('fpreport');
      
    T:=P.Targets.AddUnit('fpreportdatacsv.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('fpreport');
      AddUnit('fpreportdata');
      end;

    T:=P.Targets.AddUnit('fpreportdatadbf.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('fpreport');
      AddUnit('fpreportdata');
      end;

    T:=P.Targets.AddUnit('fpreportdatajson.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('fpreport');
      AddUnit('fpreportdata');
      end;


    T:=P.Targets.AddUnit('fpreportdatasqldb.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('fpreport');
      AddUnit('fpreportdata');
      end;
      
    T:=P.Targets.AddUnit('fpjsonreport.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('fpreport');
      AddUnit('fpreportdata');
      end; 

    T:=P.Targets.AddUnit('fplazreport.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('fpreport');
      AddUnit('fpjsonreport');
      AddUnit('fpreportdb');
      end; 
      
    T:=P.Targets.AddUnit('fpreportjson.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('fpreportstreamer');
      AddUnit('fpreport');
      end;
    {  
    T:=P.Targets.AddUnit('fpreportdom.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      begin
      AddUnit('fpreportstreamer');
      AddUnit('fpreport');
      end;
    }
    T:=P.Targets.AddUnit('fpreportdb.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      AddUnit('fpreport');

    T:=P.Targets.AddUnit('fpextfuncs.pp');
    with T.Dependencies do
      AddUnit('fpreport');

    T:=P.Targets.AddUnit('fpreportcontnr.pp');
    with T.Dependencies do
      AddUnit('fpreport');

    T:=P.Targets.AddUnit('fpreportcanvashelper.pp');
    with T.Dependencies do
      AddUnit('fpreport');
      
    T:=P.Targets.AddUnit('fpreporthtmlutil.pp');
    T.ResourceStrings := True;
    with T.Dependencies do
      AddUnit('fpreport');

    T:=P.Targets.AddUnit('fpreportpdfexport.pp');
    with T.Dependencies do
      AddUnit('fpreport');

    T:=P.Targets.AddUnit('fpreporthtmlexport.pp');
    with T.Dependencies do
      begin
      AddUnit('fpreport');
      AddUnit('fpreporthtmlutil');
      end;

    T:=P.Targets.AddUnit('fpreportfpimageexport.pp');
    with T.Dependencies do
      begin
      AddUnit('fpreport');
      AddUnit('fpreporthtmlutil');
      end;
    T:=P.Targets.AddUnit('fpreportbarcode.pp');
    with T.Dependencies do
      begin
      AddUnit('fpreport');
      end;
   T:=P.Targets.AddUnit('fpreportqrcode.pp');
    with T.Dependencies do
      begin
      AddUnit('fpreport');
      end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
