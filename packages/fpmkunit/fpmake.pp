{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  P : TPackage;
  D : TDependency;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('fpmkunit');
    P.ShortName:='fpmk';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Description:='Free Pascal Make Tool';
    P.Author := 'Peter Vreman';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Basic library of the fpmake/fppkg build system.';
    P.NeedLibC:= false;  // true for headers that indirectly link to libc?
    P.OSes := P.OSes - [embedded,nativent,msdos,win16,atari,macos,palmos];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    // All dependencies (including implicit) are listed
    // here to be able to update all requirements to
    // compile fpmake from a single place
    D:=P.Dependencies.Add('hash');
    D:=P.Dependencies.Add('paszlib');
    D:=P.Dependencies.Add('fcl-process',AllOSes-[morphos,aros,amiga,go32v2,os2]);
    D:=P.Dependencies.Add('libtar');

    with P.Targets.AddUnit('src/fpmkunit.pp') do
      ResourceStrings:=true;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
