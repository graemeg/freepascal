{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_fpcmkcfg(const ADirectory: string);

Var
  P : TPackage;
  T : TTarget;
  Data2IncBin : string;
  VS: string;

begin
  With Installer do
    begin
    P:=AddPackage('utils-fpcmkcfg');
    P.ShortName:='fcmk';
    P.OSes:=AllOSes-[embedded,msdos,nativent,win16,atari,macosclassic,palmos,zxspectrum,msxdos,amstradcpc,sinclairql];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Author := '<various>';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'A utility to create the Free Pascal configuration files.';
    P.NeedLibC:= false;

    P.Directory:=ADirectory;
    P.Version:='3.3.1';

    P.Options.Add('-Sm');
    Str(P.PackageVersion.Major, VS);
    P.Options.Add('-dpackage_version_major:='+VS);
    Str(P.PackageVersion.Minor, VS);
    P.Options.Add('-dpackage_version_minor:='+VS);
    Str(P.PackageVersion.Micro, VS);
    P.Options.Add('-dpackage_version_micro:='+VS);
    Str(P.PackageVersion.Build, VS);
    P.Options.Add('-dpackage_version_build:='+VS);

    P.Dependencies.Add('fcl-base');
    P.Dependencies.Add('fpmkunit');

    Data2IncBin := AddProgramExtension('data2inc',Defaults.BuildOS);
    p.Commands.AddCommand(caBeforeCompile, Data2IncBin, '-b -s fpc.cft fpccfg.inc DefaultConfig','fpccfg.inc','fpc.cft');
    p.Commands.AddCommand(caBeforeCompile, Data2IncBin, '-b -s fpinc.cfg fpcfg.inc fpcfg','fpcfg.inc','fpinc.cfg');
    p.Commands.AddCommand(caBeforeCompile, Data2IncBin, '-b -s fpinc.ini fpini.inc fpini','fpini.inc','fpinc.ini');
    p.Commands.AddCommand(caBeforeCompile, Data2IncBin, '-b -s fppkg.cfg fppkg.inc fppkg','fppkg.inc','fppkg.cfg');
    p.Commands.AddCommand(caBeforeCompile, Data2IncBin, '-b -s default.cft default.inc fppkg_default','default.inc','default.cft');

    T:=P.Targets.AddProgram('fpcmkcfg.pp');
    T.ResourceStrings:=true;
    T.Dependencies.AddInclude('fpccfg.inc');
    T.Dependencies.AddInclude('fpcfg.inc');
    T.Dependencies.AddInclude('fpini.inc');
    T.Dependencies.AddInclude('fppkg.inc');
    T.Dependencies.AddInclude('default.inc');
    end;
end;

{$ifndef ALLPACKAGES}
begin
  add_fpcmkcfg('');
  Installer.Run;
end.
{$endif ALLPACKAGES}




