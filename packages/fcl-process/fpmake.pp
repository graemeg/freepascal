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

    P:=AddPackage('fcl-process');
    P.ShortName:='fclp';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Author := 'Michael van Canneyt and Free Pascal Development team';
    P.License := 'LGPL with modification';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Process (execution) related parts of Free Component Libraries (FCL), FPC''s OOP library.';
    P.Options.Add('-S2h');
    P.NeedLibC:= false;
    P.OSes:=AllOSes-[embedded,msdos,win16,go32v2,nativent,macosclassic,palmos,atari,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.SourcePath.Add('src');
    P.IncludePath.Add('src',AllOSes);
    P.IncludePath.Add('src/unix',AllUnixOSes);
    P.IncludePath.Add('src/winall',AllWindowsOSes);
    P.IncludePath.Add('src/win',[win32,win64]);
    P.IncludePath.Add('src/amicommon',AllAmigaLikeOSes);
    P.IncludePath.Add('src/$(OS)',AllOSes-[win32,win64]-AllUnixOSes-AllAmigaLikeOSes);
    P.IncludePath.Add('src/dummy',AllOSes-[win32,win64]-AllUnixOSes-AllAmigaLikeOSes);

    P.Dependencies.add('morphunits',[morphos]);
    P.Dependencies.add('arosunits',[aros]);
    if Defaults.CPU=powerpc then
      P.Dependencies.add('os4units',[amiga])
    else
      P.Dependencies.add('amunits',[amiga]);
    P.Dependencies.add('fcl-base');

    T:=P.Targets.AddUnit('pipes.pp');
      T.Dependencies.AddInclude('pipes.inc');
    T:=P.Targets.AddUnit('process.pp');
      T.Dependencies.AddInclude('processbody.inc');
      T.Dependencies.AddInclude('process.inc');
      T.ResourceStrings:=True;
    T:=P.Targets.AddUnit('processunicode.pp',[win32,win64]);
      T.Dependencies.AddInclude('processbody.inc');
      T.Dependencies.AddInclude('process.inc');
      T.ResourceStrings:=True;
    T:=P.Targets.AddUnit('simpleipc.pp');
      T.Dependencies.AddInclude('simpleipc.inc');
      T.ResourceStrings:=True;
    T:=P.Targets.AddUnit('pipesipc.pp',AllUnixOSes);
      T.Dependencies.AddInclude('simpleipc.inc');
      T.ResourceStrings:=True;
    T:=P.Targets.AddUnit('dbugmsg.pp');
      T.ResourceStrings:=True;
    T:=P.Targets.AddUnit('dbugintf.pp');
      T.ResourceStrings:=True;
    P.ExamplePath.Add('examples');
      T:=P.Targets.AddExampleProgram('ipcclient.pp');
      T:=P.Targets.AddExampleProgram('ipcserver.pp');
      T:=P.Targets.AddExampleProgram('dbugsrv.pp');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
