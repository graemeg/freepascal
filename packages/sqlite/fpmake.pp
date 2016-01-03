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

    P:=AddPackage('sqlite');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='sqlt';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.OSes := AllUnixOSes+AllWindowsOSes-[qnx,win16];
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='sqlite';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('sqlite3db.pas');
      with T.Dependencies do
        begin
          AddUnit('sqlite3');
        end;
    T:=P.Targets.AddUnit('sqlite3dyn.pp');
      with T.Dependencies do
        begin
          AddInclude('sqlite3.inc');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('sqlite3.pp');
      with T.Dependencies do
        begin
          AddInclude('sqlite3.inc');
        end;
    T:=P.Targets.AddUnit('sqlitedb.pas');
      with T.Dependencies do
        begin
          AddUnit('sqlite');
        end;
    T:=P.Targets.AddUnit('sqlite.pp');

<<<<<<< HEAD
<<<<<<< HEAD
    P.ExamplePath.Add('tests/');
    P.Targets.AddExampleProgram('testapiv3x.pp');
    P.Targets.AddExampleProgram('test.pas');
    // 'testapiv3x.README
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
