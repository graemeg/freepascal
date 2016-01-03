{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  T,TBuild : TTarget;
  P : TPackage;
  i : Integer;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('fpvectorial');
{$ifdef ALLPACKAGES}
    P.Directory:='fpvectorial';
{$endif ALLPACKAGES}
    P.Version:='2.2.2-0';
    P.Author := 'Felipe Monteiro de Carvalho, Pedro Sol Pegorini L de Lima';
    P.License := 'LGPL with static linking modification ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := '???';
    P.NeedLibC:= true;

    P.Dependencies.Add('fcl-image');

    P.SourcePath.Add('src');

    T:=P.Targets.AddImplicitUnit('jwazmouse.pas');
    T:=P.Targets.AddImplicitUnit('avisocncgcodereader'); 
    T:=P.Targets.AddImplicitUnit('avisocncgcodewriter'); 
    T:=P.Targets.AddImplicitUnit('avisozlib'); 
    T:=P.Targets.AddImplicitUnit('fpvectorial'); 
    T:=P.Targets.AddImplicitUnit('fpvtocanvas');  
    T:=P.Targets.AddImplicitUnit('pdfvectorialreader'); 
    T:=P.Targets.AddImplicitUnit('pdfvrlexico'); 
    T:=P.Targets.AddImplicitUnit('pdfvrsemantico');
    T:=P.Targets.AddImplicitUnit('pdfvrsintaticoa');

    // Build unit depending on all implicit units
    TBuild:=P.Targets.AddUnit('fpvectbuildunit.pas');
    TBuild.Install:=False;
    For I:=0 to P.Targets.Count-1 do
      begin
        T:=P.Targets.TargetItems[I];
        if T.TargetType=ttImplicitUnit then
          TBuild.Dependencies.AddUnit(T.Name);
      end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
