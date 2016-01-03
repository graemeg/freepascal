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

    P:=AddPackage('openssl');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='ossl';
    P.Description := 'Interface units for OpenSSL libraries supporting SSL-encrypted network communication.';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.OSes := AllUnixOSes+AllWindowsOSes+[OS2,EMX]-[qnx];
    P.Dependencies.Add('rtl-extra',[OS2,EMX]);

    T:=P.Targets.AddUnit('openssl.pas');
    T:=P.Targets.AddUnit('fpopenssl.pp');

    P.ExamplePath.Add('examples');
    P.Targets.AddExampleProgram('test1.pas');
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='openssl';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
//    P.Dependencies.Add('x11');

    T:=P.Targets.AddUnit('openssl.pas');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
