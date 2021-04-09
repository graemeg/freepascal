{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_fprcp(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('utils-fprcp');
    P.ShortName:='fprc';
    { java and jvm-android do not support 
      getmem/freemem and new/dispose used in
      these sources }
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];
    { palmos does not have classes }
    P.OSes := P.OSes - [palmos];
    { Program does not fit in 16-bit memory constraints }
    P.OSes := P.OSes - [msdos,win16,zxspectrum,msxdos,amstradcpc,sinclairql];
    { avr-embedded and i8086-embedded have not floating point support by default }
    if Defaults.CPU in [avr,i8086,z80] then
      P.OSes := P.OSes - [embedded];

    P.Author := '<various>';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'fprcp.exe extracts from C header and Pascal files included into resource '+
                     'scripts numerical constants and replaces these constants to its values '+
                     'in resource script.';
    P.NeedLibC:= false;

    P.Directory:=ADirectory;
    P.Version:='3.3.1';

    T:=P.Targets.AddProgram('fprcp.pp');
    T.Dependencies.AddUnit('comments');
    T.Dependencies.AddUnit('expr');
    T.Dependencies.AddUnit('pasprep');

    P.Targets.AddUnit('comments.pp').install:=false;
    P.Targets.AddUnit('expr.pp').install:=false;
    P.Targets.AddUnit('pasprep.pp').install:=false;
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_fprcp('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




