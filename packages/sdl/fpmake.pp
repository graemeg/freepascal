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
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
    P.Dependencies.Add('x11',AllUnixOSes);
    P.Dependencies.Add('pthreads',AllUnixOSes);
    P.Dependencies.Add('morphunits',[morphos]);
    if Defaults.CPU=arm then
       P.OSes := P.OSes - [darwin];
    P.OSes := P.OSes - [iphonesim,os2,emx,go32v2,watcom,nativent,embedded,android,amiga,aros,msdos,gba,nds,win16];
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='sdl';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
    P.Dependencies.Add('x11');
    P.Dependencies.Add('pthreads');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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
<<<<<<< HEAD
<<<<<<< HEAD
=======
          AddUnit('pthreads');
          AddUnit('xlib');
>>>>>>> graemeg/fixes_2_2
=======
          AddUnit('pthreads');
          AddUnit('xlib');
>>>>>>> origin/fixes_2_2
        end;
    T:=P.Targets.AddUnit('sdl_ttf.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('sdl');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('sdlutils.pas',[i386,powerpc],[linux,freebsd,win32,darwin,iphonesim]);
=======
    T:=P.Targets.AddUnit('sdlutils.pas',[i386,powerpc],[linux,freebsd,win32,darwin]);
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('sdlutils.pas',[i386,powerpc],[linux,freebsd,win32,darwin]);
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('sdlutils.pas',[i386,powerpc],[linux,freebsd,win32,darwin]);
>>>>>>> graemeg/cpstrnew
=======
    T:=P.Targets.AddUnit('sdlutils.pas',[i386,powerpc],[linux,freebsd,win32,darwin]);
>>>>>>> origin/cpstrnew
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
=======
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('sdlutils.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('xlib');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          AddUnit('sdl');
        end;
    T:=P.Targets.AddUnit('smpeg.pas');
      with T.Dependencies do
        begin
          AddInclude('jedi-sdl.inc');
          AddUnit('sdl');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    P.Sources.AddSrc('LGPL');
    P.Sources.AddSrc('LGPL.addon');
    P.Sources.AddSrc('MPL-1.1');
    P.Sources.AddSrc('README.txt');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD




<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
 
>>>>>>> graemeg/cpstrnew
=======
 
>>>>>>> graemeg/cpstrnew
=======
 
>>>>>>> graemeg/cpstrnew
=======
 
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
