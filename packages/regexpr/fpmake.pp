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

    P:=AddPackage('regexpr');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='reg';
    P.Description := 'Library for working with regular expressions.';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes:=P.OSes-[embedded,win16];
    P.Dependencies.Add('rtl-objpas');
    P.SourcePath.Add('src');

    // Sorokin's RegExpr
    T:=P.Targets.AddUnit('regexpr.pas');

    // RegEx from Joost
    T:=P.Targets.AddUnit('oldregexpr.pp');
    T:=P.Targets.AddUnit('regex.pp');

    T.ResourceStrings := True;

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('testreg1.pp');
    P.Sources.AddExampleFiles('examples/*',P.Directory,false,'.');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='regexpr';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('regexpr.pp');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
