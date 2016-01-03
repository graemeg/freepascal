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

    P:=AddPackage('oracle');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='ora';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
    P.OSes := AllUnixOSes+AllWindowsOSes-[qnx];
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='oracle';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    T:=P.Targets.AddUnit('ocidyn.pp');
      with T.Dependencies do
        begin
          AddInclude('oci.inc');
          AddInclude('ocidfn.inc');
          AddInclude('oci1.inc');
          AddInclude('oro_interface.inc');
          AddInclude('orl.inc');
          AddInclude('ort.inc');
          AddInclude('ociap.inc');
          AddInclude('nzt.inc');
          AddInclude('nzerror.inc');
          AddInclude('oro_implementation.inc');
          AddUnit('oratypes');
        end;
    T:=P.Targets.AddUnit('oci.pp');
      with T.Dependencies do
        begin
          AddInclude('oci.inc');
          AddInclude('ocidfn.inc');
          AddInclude('oci1.inc');
          AddInclude('oro_interface.inc');
          AddInclude('orl.inc');
          AddInclude('ort.inc');
          AddInclude('ociap.inc');
          AddInclude('nzt.inc');
          AddInclude('nzerror.inc');
          AddInclude('oro_implementation.inc');
          AddUnit('oratypes');
        end;
    T:=P.Targets.AddUnit('oraoci.pp');
    T:=P.Targets.AddUnit('oratypes.pp');

<<<<<<< HEAD
<<<<<<< HEAD
    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('test01.pp');
    P.Targets.AddExampleProgram('oraclew.pp');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
