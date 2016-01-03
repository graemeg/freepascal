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

    P:=AddPackage('ibase');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='ibas';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Author := 'Library: (Codegear?), header: ?';
    P.License := 'Library: Interbase License, header: LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Headers for the Interbase/Firebird RDBMS';
    P.NeedLibC:= true;  // true for headers that indirectly link to libc?
    P.OSes := AllUnixOSes+AllWindowsOSes-[qnx];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='ibase';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('ibase40.pp');
    T:=P.Targets.AddUnit('ibase60dyn.pp');
      with T.Dependencies do
        begin
          AddInclude('ibase60.inc');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T.ResourceStrings := True;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('ibase60.pp');
      with T.Dependencies do
        begin
          AddInclude('ibase60.inc');
        end;

<<<<<<< HEAD
<<<<<<< HEAD
    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('testib40.pp');
    P.Targets.AddExampleProgram('testib60.pp');
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
