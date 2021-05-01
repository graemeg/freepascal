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

    P:=AddPackage('qlunits');
    P.ShortName := 'qlun';

    P.Author := 'FPC core team';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Description := 'qlunits, OS interface and utility units for the Sinclair QL';

{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    P.OSes:=[sinclairql];

    T:=P.Targets.AddUnit('qdos.pas');
    T:=P.Targets.AddUnit('qlfloat.pas');
    T:=P.Targets.AddUnit('qlutil.pas');
    T:=P.Targets.AddUnit('sms.pas');

    P.ExamplePath.Add('examples');
    T:=P.Targets.AddExampleProgram('qlcube.pas');
    T:=P.Targets.AddExampleProgram('mtinf.pas');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
