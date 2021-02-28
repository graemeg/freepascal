{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  T : TTarget;
  P : TPackage;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('tplylib');
    P.ShortName:='tpll';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';

    { java and jvm-android do not support
      fpc_get_output used in these sources }
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];
    { palmos does not support command line parameters }
    P.OSes := P.OSes - [palmos];
    { Program does not fit in 16-bit memory constraints }
    P.OSes := P.OSes - [msdos,win16];
    { avr-embedded and i8086-embedded do not meet needed requirements }
    if Defaults.CPU in [avr,i8086] then
      P.OSes := P.OSes - [embedded];

    P.Author := '<various>';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Library units for a compiler generator for Turbo Pascal and compatibles.';
    P.NeedLibC:= false;

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    P.Options.Add('-Sg');

    P.Targets.AddUnit('lexlib.pas');
    P.Targets.AddUnit('yacclib.pas');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}

