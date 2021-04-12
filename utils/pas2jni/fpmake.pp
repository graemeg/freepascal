{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_pas2jni(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('utils-pas2jni');
    P.ShortName:='p2jn';

    P.Author := 'Yury Sidorov';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Description := 'The pas2jni utility generates a JNI (Java Native Interface) bridge for a Pascal code.';
    P.Email := '';
    P.NeedLibC:= false;

    P.OSes:=AllOSes-[embedded,msdos,win16,go32v2,nativent,macosclassic,palmos,atari,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Directory:=ADirectory;
    P.Version:='3.3.1';
    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fcl-process');
    P.Dependencies.Add('fcl-json');

    T:=P.Targets.AddImplicitUnit('def.pas');
    T.Install := false;
    T:=P.Targets.AddImplicitUnit('ppuparser.pas');
    T.Install := false;
    T:=P.Targets.AddImplicitUnit('writer.pas');
    T.Install := false;
    T:=P.Targets.AddProgram('pas2jni.pas');
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_pas2jni('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




