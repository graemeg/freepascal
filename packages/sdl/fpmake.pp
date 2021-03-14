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

    P:=AddPackage('sdl');
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
    P.Dependencies.Add('x11',AllUnixOSes);
    P.Dependencies.Add('pthreads',AllUnixOSes);
    P.Dependencies.Add('morphunits',[morphos]);
    if Defaults.CPU=arm then
       P.OSes := P.OSes - [darwin];
    P.OSes := P.OSes - [iphonesim,ios,java,os2,emx,go32v2,watcom,netware,netwlibc,nativent,embedded,
                        android,amiga,aros,msdos,gba,nds,win16,atari,macosclassic,palmos,symbian,wii,
                        freertos,zxspectrum,msxdos,amstradcpc,sinclairql,wasi];

    T:=P.Targets.AddUnit('logger.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
        end;
    T:=P.Targets.AddUnit('sdl_gfx.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('sdl');
        end;
    T:=P.Targets.AddUnit('sdl_image.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('sdl');
        end;
    T:=P.Targets.AddUnit('sdl_mixer_nosmpeg.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('sdl');
        end;
    T:=P.Targets.AddUnit('sdl_mixer.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('smpeg');
          AddUnit('sdl');
        end;
    T:=P.Targets.AddUnit('sdl_net.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('sdl');
        end;
    T:=P.Targets.AddUnit('sdl.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
        end;
    T:=P.Targets.AddUnit('sdl_ttf.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('sdl');
        end;
    T:=P.Targets.AddUnit('sdlutils.pas',[i386,powerpc],[linux,freebsd,win32,darwin,iphonesim,ios]);
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('sdl');
        end;
    T:=P.Targets.AddUnit('smpeg.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('sdl');
        end;
    P.Sources.AddSrc('LGPL');
    P.Sources.AddSrc('LGPL.addon');
    P.Sources.AddSrc('MPL-1.1');
    P.Sources.AddSrc('README.txt');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}





