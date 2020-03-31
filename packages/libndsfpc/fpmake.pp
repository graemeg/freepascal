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

    P:=AddPackage('libndsfpc');
    P.ShortName := 'lnds';
    P.OSes:=[nds];
    P.CPUs:=[arm];
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.3.1';
    P.Author := 'Library: libnds, dswifi, maxmod and gl2d from devkitARM; headers: Francesco Lombardi';
    P.License := 'LGPL';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Some libraries for Nintendo DS.';
    P.NeedLibC:= false;

    P.SourcePath.Add('src');

    T:=P.Targets.AddUnit('nds9.pp');
    T:=P.Targets.AddUnit('nds7.pp');
     T.IncludePath.Add('src/nds');
     with T.Dependencies do
     begin
       AddInclude('bios.inc');
       AddInclude('card.inc');
       AddInclude('debug.inc');
       AddInclude('disc_io.inc');
       AddInclude('dma.inc');
       AddInclude('fifocommon.inc');
       AddInclude('fifomessages.inc');
       AddInclude('helper.inc');
       AddInclude('input.inc');
       AddInclude('interrupts.inc');
       AddInclude('ipc.inc');
       AddInclude('jtypes.inc');
       AddInclude('libversion.inc');
       AddInclude('memory.inc');
       AddInclude('nds.inc');
       AddInclude('ndsinclude.inc');
       AddInclude('ndstypes.inc');
       AddInclude('registers_alt.inc');
       AddInclude('system.inc');
       AddInclude('timers.inc');
       AddInclude('touch.inc');
     end;
     T.IncludePath.Add('src/nds/arm7');
     with T.Dependencies do
     begin
       AddInclude('audio.inc');
       AddInclude('clock.inc');
       AddInclude('i2c.inc');
       AddInclude('input.inc');
       AddInclude('sdmmc.inc');
       AddInclude('serial.inc');
       AddInclude('touch.inc');
     end;
     T.IncludePath.Add('src/nds/arm9');
     with T.Dependencies do
     begin
       AddInclude('background.inc');
       AddInclude('boxtest.inc');
       AddInclude('cache.inc');
       AddInclude('console.inc');
       AddInclude('decompress.inc');
       AddInclude('dldi.inc');
       AddInclude('dynamicArray.inc');
       AddInclude('exceptions.inc');
       AddInclude('guitarGrip.inc');
       AddInclude('image.inc');
       AddInclude('input.inc');
       AddInclude('keyboard.inc');
       AddInclude('linkedlist.inc');
       AddInclude('math.inc');
       AddInclude('ndsmotion.inc');
       AddInclude('paddle.inc');
       AddInclude('pcx.inc');
       AddInclude('piano.inc');
       AddInclude('postest.inc');
       AddInclude('rumble.inc');
       AddInclude('sassert.inc');
       AddInclude('sound.inc');
       AddInclude('sprite.inc');
       AddInclude('trig_lut.inc');
       AddInclude('video.inc');
       AddInclude('videoGL.inc');
       AddInclude('window.inc');
     end;

    // dswifi
    P.SourcePath.Add('src/dswifi');
    T:=P.Targets.AddUnit('dswifi9.pp');
    T:=P.Targets.AddUnit('dswifi7.pp');
     T.IncludePath.Add('src/dswifi/inc');
     with T.Dependencies do
     begin
       AddInclude('dswifi_version.inc');
       AddInclude('dswifi9.inc');
       AddInclude('dswifi7.inc');
       AddInclude('netdb.inc');
       AddInclude('sgIP_errno.inc');
     end;
     T.IncludePath.Add('src/dswifi/inc/netinet');
       T.Dependencies.AddInclude('in.inc');
     T.IncludePath.Add('src/dswifi/inc/sys');
       T.Dependencies.AddInclude('socket.inc');

    // fat
    P.SourcePath.Add('src/fat');
    T:=P.Targets.AddUnit('fat.pp');
     T.IncludePath.Add('src/fat');
     T.Dependencies.AddInclude('fat.inc');
    T:=P.Targets.AddUnit('filesystem.pp');
     T.IncludePath.Add('src/fat');
     T.Dependencies.AddInclude('filesystem.inc');

    // gl2d
    P.SourcePath.Add('src/gl2d');
    T:=P.Targets.AddUnit('gl2d.pp');

    // maxmod
    P.SourcePath.Add('src/maxmod');
    T:=P.Targets.AddUnit('maxmod9.pp');
    T:=P.Targets.AddUnit('maxmod7.pp');
     T.IncludePath.Add('src/maxmod/inc');
     with T.Dependencies do
     begin
       AddInclude('maxmod.inc');
       AddInclude('maxmod7.inc');
       AddInclude('maxmod9.inc');
       AddInclude('mm_mas.inc');
       AddInclude('mm_msl.inc');
       AddInclude('mm_types.inc');
     end;

    P.Sources.AddExampleFiles('examples/*',P.Directory,true,'.');
{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
