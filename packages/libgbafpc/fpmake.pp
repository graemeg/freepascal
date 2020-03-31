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

    P:=AddPackage('libgbafpc');
    P.ShortName := 'lgba';
    P.OSes:=[gba];
    P.CPUs:=[arm];
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Author := 'Library: libgba and maxmod from devkitARM; headers: Francesco Lombardi';
    P.License := 'LGPL';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Some libraries for Nintendo Gameboy Advance.';
    P.NeedLibC:= false;

    P.SourcePath.Add('src');
    T:=P.Targets.AddUnit('gba.pp');
     T.IncludePath.Add('src/gba');
     with T.Dependencies do
     begin
       AddInclude('boyscout.inc');
       AddInclude('core_asm.as');
       AddInclude('disc.inc');
       AddInclude('disc_io.inc');
       AddInclude('dldi.inc');
       AddInclude('fade.inc');
       AddInclude('gba.inc');
       AddInclude('gba_affine.inc');
       AddInclude('gba_base.inc');
       AddInclude('gba_compression.inc');
       AddInclude('gba_console.inc');
       AddInclude('gba_dma.inc');
       AddInclude('gba_helper.inc');
       AddInclude('gba_input.inc');
       AddInclude('gba_interrupt.inc');
       AddInclude('gba_multiboot.inc');
       AddInclude('gba_sio.inc');
       AddInclude('gba_sound.inc');
       AddInclude('gba_sprites.inc');
       AddInclude('gba_systemcalls.inc');
       AddInclude('gba_timers.inc');
       AddInclude('gba_types.inc');
       AddInclude('gba_video.inc');
       AddInclude('helper.inc');
       AddInclude('mappy.inc');
       AddInclude('mbv2.inc');
       AddInclude('pcx.inc');
     end;


    P.SourcePath.Add('src/maxmod');
    T:=P.Targets.AddUnit('maxmod.pp');
     T.IncludePath.Add('src/maxmod/inc');
     with T.Dependencies do
     begin
       AddInclude('maxmod.inc');
       AddInclude('mm_types.inc');
     end;

    P.Sources.AddExampleFiles('examples/*',P.Directory,true,'.');

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
