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

    P:=AddPackage('libenet');
    P.ShortName:='lnet';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.SourcePath.Add('src');
    P.SourcePath.Add('examples');
    P.IncludePath.Add('src');
    { only enable for darwin after testing }
    P.OSes := AllUnixOSes+AllWindowsOSes-[qnx,darwin,iphonesim];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Dependencies.Add('rtl-extra'); // winsock2
    
    T:=P.Targets.AddUnit('enet.pp');
    T:=P.Targets.AddUnit('uenetclass.pp');
    with T.Dependencies do
      AddUnit('enet');
    // Examples
    P.ExamplePath.Add('examples');
      P.Targets.AddExampleProgram('serverapp.pp');
      P.Targets.AddExampleProgram('clientapp.pp');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
