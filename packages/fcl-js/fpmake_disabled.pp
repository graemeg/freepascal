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
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-js';
{$endif ALLPACKAGES}
    P.Version:='1.0';
    P.Author := 'Michael Van Canneyt';
    P.License := 'LGPL with FPC modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := 'michael@freepascal.org';
    P.Description := 'Javascript scanner/parser/syntax tree units';

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('jsbase.pp');
    T:=P.Targets.AddUnit('jstree.pp');
    T:=P.Targets.AddUnit('jsscanner.pp');
      T.ResourceStrings:=true;
    T:=P.Targets.AddUnit('jsparser.pp');
      T.ResourceStrings:=true;
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
