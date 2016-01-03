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

    P:=AddPackage('libogcfpc');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    P.OSes:=[wii];
    P.CPUs:=[powerpc];
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Author := 'Library: libogc, libfat, libmad and libmodplay from devkitPPC; headers: Francesco Lombardi';
    P.License := 'LGPL';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Some libraries for Nintendo Wii.';
    P.NeedLibC:= false;
    P.Dependencies.Add('mad');

    P.SourcePath.Add('src');
    T:=P.Targets.AddUnit('aesndlib.pp');
    T:=P.Targets.AddUnit('asndlib.pp');
    T:=P.Targets.AddUnit('debug.pp');
    T:=P.Targets.AddUnit('fat.pp');
    T:=P.Targets.AddUnit('gccore.pp');
    T:=P.Targets.AddUnit('gcmodplay.pp');
    T:=P.Targets.AddUnit('gctypes.pp');
    T:=P.Targets.AddUnit('iso9660.pp');
    T:=P.Targets.AddUnit('mp3player.pp');
    T:=P.Targets.AddUnit('network.pp');

     T.IncludePath.Add('src/bte');
     with T.Dependencies do
     begin
       AddInclude('bd_addr.inc');
       AddInclude('bte.inc');
     end;

     T.IncludePath.Add('src/di');
     with T.Dependencies do
     begin
       AddInclude('di.inc');
     end;

     T.IncludePath.Add('src/ogc');
     with T.Dependencies do
     begin
       AddInclude('aram.inc');
       AddInclude('arqmgr.inc');
       AddInclude('arqueue.inc');
       AddInclude('audio.inc');
       AddInclude('cache.inc');
       AddInclude('card.inc');
       AddInclude('cast.inc');
       AddInclude('color.inc');
       AddInclude('cond.inc');
       AddInclude('conf.inc');
       AddInclude('consol.inc'); 
       AddInclude('context.inc');
       AddInclude('disc_io.inc');
       AddInclude('dsp.inc');
       AddInclude('dvd.inc');
       AddInclude('es.inc');
       AddInclude('exi.inc');
       AddInclude('gu.inc');
       AddInclude('gx.inc');
       AddInclude('gx_struct.inc');
       AddInclude('ios.inc');
       AddInclude('ipc.inc');
       AddInclude('irq.inc');
       AddInclude('isfs.inc');
       AddInclude('libversion.inc');
       AddInclude('lwp.inc');
       AddInclude('lwp_config.inc');
       AddInclude('lwp_heap.inc');
       AddInclude('lwp_messages.inc');
       AddInclude('lwp_mutex.inc');
       AddInclude('lwp_objmgr.inc');
       AddInclude('lwp_priority.inc');
       AddInclude('lwp_queue.inc');
       AddInclude('lwp_sema.inc');
       AddInclude('lwp_stack.inc');
       AddInclude('lwp_states.inc');
       AddInclude('lwp_threadq.inc'); 
       AddInclude('lwp_threads.inc');
       AddInclude('lwp_tqdata.inc');
       AddInclude('lwp_watchdog.inc');
       AddInclude('lwp_wkspace.inc');
       AddInclude('message.inc');
       AddInclude('mutex.inc');
       AddInclude('pad.inc');
       AddInclude('semaphore.inc');
       AddInclude('si.inc');
       AddInclude('stm.inc');
       AddInclude('system.inc');
       AddInclude('sys_state.inc');
       AddInclude('texconv.inc');
       AddInclude('tpl.inc');
       AddInclude('usb.inc');
       AddInclude('usbgecko.inc');
       AddInclude('usbmouse.inc');
       AddInclude('usbstorage.inc');
       AddInclude('video.inc');
       AddInclude('video_types.inc');
       AddInclude('wiilaunch.inc');
     end;
       
     T.IncludePath.Add('src/ogc/machine');
     with T.Dependencies do
     begin
       AddInclude('asm.inc');
       AddInclude('processor.inc');
       AddInclude('spinlock.inc');
     end;


     T.IncludePath.Add('src/sdcard');
     with T.Dependencies do
     begin
       AddInclude('card_buf.inc');
       AddInclude('card_cmn.inc');
       AddInclude('card_io.inc');
       AddInclude('gcsd.inc');
       AddInclude('wiisd_io.inc');
     end;

     T.IncludePath.Add('src/wiikeyboard');
     with T.Dependencies do
     begin
       AddInclude('keyboard.inc');
       AddInclude('usbkeyboard.inc');
       AddInclude('wsksymdef.inc');
     end;

     T.IncludePath.Add('src/wiiuse');
     with T.Dependencies do
     begin
       AddInclude('wiiuse.inc');
       AddInclude('wpad.inc');
     end;

     P.Sources.AddExampleFiles('examples/*',P.Directory,true,'.');
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifdef ALLPACKAGES}
    P.Directory:='libogcfpc';
{$endif ALLPACKAGES}
    P.Version:='2.5.1';
    P.SourcePath.Add('src');
    P.Dependencies.Add('mad');

    // not linux compilable, skip
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
