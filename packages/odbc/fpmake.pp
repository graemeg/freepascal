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

    P:=AddPackage('odbc');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes := AllUnixOSes+AllWindowsOSes-[qnx];
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='odbc';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('odbcsqldyn.pas');
      with T.Dependencies do
        begin
          AddInclude('odbcsql.inc');
        end;
    T:=P.Targets.AddUnit('odbcsql.pas');
      with T.Dependencies do
        begin
          AddInclude('odbcsql.inc');
        end;

<<<<<<< HEAD
<<<<<<< HEAD
    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('testodbc.pp');
    P.Sources.AddExampleFiles('examples/*',P.Directory,false,'.');

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
