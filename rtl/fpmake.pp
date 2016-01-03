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

    P:=AddPackage('rtl');
{$ifdef ALLPACKAGES}
    P.Directory:='rtl';
{$endif ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Version:='3.1.1';
=======
    P.Version:='2.2.0';
>>>>>>> graemeg/fixes_2_2
=======
    P.Version:='2.2.0';
>>>>>>> origin/fixes_2_2

    // Where to find the sources using firstmatch
    P.SourcePath.Add('$(OS)');
    P.SourcePath.Add('$(CPU)');
    P.SourcePath.Add('bsd',AllBSDOSes);
    P.SourcePath.Add('unix',AllUnixOSes);
    P.SourcePath.Add('win',AllWindowsOSes);
    P.SourcePath.Add('inc');
    P.SourcePath.Add('objpas');

    // Where to find the include files using firstmatch
    P.IncludePath.Add('$(OS)/$(CPU)',AllUnixOSes);
    P.IncludePath.Add('$(OS)');
    P.IncludePath.Add('$(CPU)');
    P.IncludePath.Add('bsd',AllBSDOSes);
    P.IncludePath.Add('bsd/$(CPU)',AllBSDOSes-[Darwin]);
    P.IncludePath.Add('unix',AllUnixOSes);
    P.IncludePath.Add('win',AllWindowsOSes);
    P.IncludePath.Add('inc');
    P.IncludePath.Add('objpas');

    // System unit
    T:=P.Targets.AddUnit('system.pp');
      With T.Dependencies do
        begin
          // Headers
          AddInclude('setjumph.inc');
          AddInclude('systemh.inc');
          AddInclude('objpash.inc');
          AddInclude('mathh.inc');
          AddInclude('wstringh.inc');
          AddInclude('dynarrh.inc');
          AddInclude('compproc.inc');
          AddInclude('heaph.inc');
          AddInclude('threadh.inc');
          AddInclude('varianth.inc');
          AddInclude('sysosh.inc');
          AddInclude('resh.inc');
          AddInclude('currh.inc');
          // Implementations
          AddInclude('set.inc');
          AddInclude('int64p.inc');
          AddInclude('setjump.inc');
          AddInclude('sysos.inc');
          AddInclude('sysheap.inc');
          AddInclude('sysdir.inc');
          AddInclude('sysfile.inc');
          AddInclude('sysres.inc');
          AddInclude('except.inc');
          AddInclude('threadvr.inc');
          AddInclude('filerec.inc');
          AddInclude('textrec.inc');
          AddInclude('generic.inc');
          AddInclude('genset.inc');
          AddInclude('genmath.inc');
          AddInclude('gencurr.inc');
          AddInclude('sstrings.inc');
          AddInclude('int64.inc');
          AddInclude('astrings.inc');
          AddInclude('wstrings.inc');
<<<<<<< HEAD
<<<<<<< HEAD
=======
          AddInclude('wustrings.inc');
>>>>>>> graemeg/fixes_2_2
=======
          AddInclude('wustrings.inc');
>>>>>>> origin/fixes_2_2
          AddInclude('aliases.inc');
          AddInclude('dynarr.inc');
          AddInclude('objpas.inc');
          AddInclude('variant.inc');
          AddInclude('rtti.inc');
          AddInclude('heap.inc');
          AddInclude('thread.inc');
          AddInclude('text.inc');
          AddInclude('file.inc');
          AddInclude('typefile.inc');
          AddInclude('innr.inc');
          AddInclude('$(CPU).inc');
          AddInclude('fastmove.inc',[i386],AllOSes);
<<<<<<< HEAD
<<<<<<< HEAD
=======
          AddInclude('strpas.inc',[i386,powerpc,powerpc64],AllOSes);
>>>>>>> graemeg/fixes_2_2
=======
          AddInclude('strpas.inc',[i386,powerpc,powerpc64],AllOSes);
>>>>>>> origin/fixes_2_2
          AddInclude('math.inc');
          AddInclude('real2str.inc');
          AddInclude('systhrd.inc',AllWindowsOSes+[Netware,Netwlibc,EMX,OS2]);
          // Unix implementations
          AddInclude('osdefs.inc',AllUnixOSes);
          AddInclude('sysunixh.inc',AllUnixOSes);
<<<<<<< HEAD
<<<<<<< HEAD
=======
          AddInclude('elfres32.inc',AllUnixOSes);
>>>>>>> graemeg/fixes_2_2
=======
          AddInclude('elfres32.inc',AllUnixOSes);
>>>>>>> origin/fixes_2_2
          AddInclude('system.inc',AllUnixOSes);
          AddInclude('errno.inc',AllUnixOSes);
          AddInclude('ostypes.inc',AllUnixOSes);
          AddInclude('ptypes.inc',AllUnixOSes);
          AddInclude('ctypes.inc',AllUnixOSes);
          AddInclude('stat.inc',[Linux]);
          AddInclude('signal.inc',AllUnixOSes);
          AddInclude('sighnd.inc',AllUnixOSes-[Beos]);
          AddInclude('sighndh.inc',[Linux,Solaris]);
          AddInclude('syscallh.inc',[Linux,Beos,FreeBSD]);
          AddInclude('syscall.inc',[Linux,Beos,FreeBSD]);
          AddInclude('sysnr.inc',[Linux,Beos,FreeBSD]);
          AddInclude('ossysc.inc',AllUnixOSes-[Solaris]);
          AddInclude('osmacro.inc',AllUnixOSes);
          // Windows implementations
          AddInclude('winres.inc',AllWindowsOSes);
        end;

    // Compile mode units
    T:=P.Targets.AddUnit('objpas.pp');
      T.Dependencies.AddUnit('system');
    T:=P.Targets.AddUnit('macpas.pp');
      T.Dependencies.AddUnit('system');

    // Unix units
    T:=P.Targets.AddUnit('unixtype.pp',AllUnixOSes);
      With T.Dependencies do
        begin
          AddUnit('system');
        end;
    T:=P.Targets.AddUnit('unixutil.pp',AllUnixOSes);
      with T.Dependencies do
        begin
          AddInclude('textrec.inc');
          AddInclude('filerec.inc');
          AddUnit('system');
        end;
    T:=P.Targets.AddUnit('baseunix.pp',AllUnixOSes);
      With T.Dependencies do
        begin
          AddUnit('unixtype');
          AddInclude('osdefs.inc');
          AddInclude('aliasptp.inc');
          AddInclude('aliasctp.inc');
          AddInclude('errno.inc');
          AddInclude('ostypes.inc');
          AddInclude('stat.inc',[Linux]);
          AddInclude('signal.inc');
          AddInclude('sighndh.inc',[Linux,Solaris]);
          AddInclude('bunxh.inc');
          AddInclude('bunxovlh.inc');
          AddInclude('genfunch.inc');
          AddInclude('genfuncs.inc');
          AddInclude('gensigset.inc');
          AddInclude('genfdset.inc');
          AddInclude('syscallh.inc',[Linux,Beos,FreeBSD]);
          AddInclude('sysnr.inc',[Linux,Beos,FreeBSD]);
          AddInclude('bsyscall.inc',[Linux,FreeBSD]);
          AddInclude('bunxsysc.inc',[Linux,FreeBSD]);
          AddInclude('settimeo.inc');
          AddInclude('osmacro.inc');
          AddInclude('bunxovl.inc');
          AddInclude('textrec.inc');
          AddInclude('filerec.inc');
        end;
    T:=P.Targets.AddUnit('unix.pp',AllUnixOSes);
      with T.Dependencies do
        begin
          AddUnit('baseunix');
          AddUnit('unixutil');
          AddInclude('aliasptp.inc');
          AddInclude('aliasctp.inc');
          AddInclude('unxconst.inc');
          AddInclude('unxsysch.inc',[Linux,FreeBSD]);
          AddInclude('unxsysc.inc',[Linux,FreeBSD]);
          AddInclude('unxovlh.inc');
          AddInclude('unxovl.inc');
          AddInclude('syscallh.inc',[Linux,Beos,FreeBSD]);
          AddInclude('textrec.inc');
          AddInclude('filerec.inc');
          AddInclude('unxfunc.inc');
          AddInclude('timezone.inc');
        end;
    T:=P.Targets.AddUnit('termio.pp',AllUnixOSes);
      with T.Dependencies do
        begin
          AddUnit('baseunix');
          AddInclude('termios.inc');
          AddInclude('termiosh.inc');
          AddInclude('textrec.inc');
          AddInclude('termiosproc.inc');
        end;
    T:=P.Targets.AddUnit('unix/errors.pp',AllUnixOSes);
      with T.Dependencies do
        begin
          AddUnit('unixtype');
          AddInclude('errnostr.inc');
        end;
    T:=P.Targets.AddUnit('unix/syscall.pp',[Linux,Beos,FreeBSD]);
      with T.Dependencies do
        begin
          AddInclude('sysnr.inc');
          AddInclude('syscallh.inc');
        end;
    T:=P.Targets.AddUnit('unix/terminfo.pp',AllUnixOSes);
      T.Dependencies.AddUnit('baseunix',AllUnixOSes);
    T:=P.Targets.AddUnit('unix/dl.pp',AllUnixOSes);
      T.Dependencies.AddUnit('system');
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('unix/ipc.pp',AllUnixOSes - [Android]);
=======
    T:=P.Targets.AddUnit('unix/ipc.pp',AllUnixOSes);
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('unix/ipc.pp',AllUnixOSes);
>>>>>>> origin/fixes_2_2
      With T.Dependencies do
        begin
          AddUnit('baseunix');
          AddUnit('syscall',[Linux,Beos,FreeBSD]);
          AddInclude('ipccall.inc',[Linux]);
          AddInclude('ipcbsd.inc',[FreeBSD]);
        end;

    // Linux units
    T:=P.Targets.AddUnit('si_c21g.pp',[i386],[Linux]);
      With T.Dependencies do
        begin
          AddUnit('system');
          AddInclude('si_intf.inc');
          AddInclude('sysnr.inc');
          AddInclude('si_c21g.inc');
        end;
    T:=P.Targets.AddUnit('si_c21.pp',[i386],[Linux]);
      With T.Dependencies do
        begin
          AddUnit('system');
          AddInclude('si_intf.inc');
          AddInclude('sysnr.inc');
          AddInclude('si_c21.inc');
        end;
    T:=P.Targets.AddUnit('si_c.pp',[i386,x86_64],[Linux]);
      With T.Dependencies do
        begin
          AddUnit('system');
          AddInclude('si_intf.inc');
          AddInclude('sysnr.inc');
          AddInclude('si_c.inc');
        end;
    T:=P.Targets.AddUnit('si_dll.pp',[i386],[Linux]);
      With T.Dependencies do
        begin
          AddUnit('system');
          AddInclude('si_intf.inc');
          AddInclude('sysnr.inc');
          AddInclude('si_dll.inc');
        end;
    T:=P.Targets.AddUnit('si_prc.pp',[i386,x86_64],[Linux]);
      With T.Dependencies do
        begin
          AddUnit('system');
          AddInclude('si_intf.inc');
          AddInclude('sysnr.inc');
          AddInclude('si_prc.inc');
        end;
    T:=P.Targets.AddUnit('si_uc.pp',[i386],[Linux]);
      With T.Dependencies do
        begin
          AddUnit('system');
          AddInclude('si_intf.inc');
          AddInclude('sysnr.inc');
          AddInclude('si_uc.inc');
        end;
    T:=P.Targets.AddUnit('linux.pp',[Linux]);
      with T.Dependencies do
        begin
          AddUnit('baseunix');
          AddUnit('syscall');
        end;
    T:=P.Targets.AddUnit('gpm.pp',[Linux]);
      With T.Dependencies do
        begin
          AddUnit('termio');
          AddUnit('sockets');
          AddUnit('strings');
          AddUnit('unix');
        end;
    T:=P.Targets.AddUnit('linuxvcs.pp',[Linux]);
      with T.Dependencies do
        begin
          AddUnit('baseunix');
          AddUnit('strings');
        end;
    T:=P.Targets.AddUnit('x86.pp',[i386,x86_64],[Linux]);
      T.Dependencies.AddUnit('system');
      T.Dependencies.AddUnit('baseunix');
    T:=P.Targets.AddUnit('ports.pp',[i386,x86_64],[Linux]);
      with T.Dependencies do
        begin
          AddUnit('objpas');
          AddUnit('x86');
        end;
    T:=P.Targets.AddUnit('fpcylix.pp',[i386],[Linux]);
      With T.Dependencies do
        begin
          AddUnit('cthreads');
          AddUnit('cwstring');
          AddUnit('dynlibs');
        end;

    // Windows units
    T:=P.Targets.AddUnit('sysinitcyg.pp',AllWindowsOSes-[WinCE]);
      T.Dependencies.AddUnit('system');
    T:=P.Targets.AddUnit('sysinitgprof.pp',AllWindowsOSes-[WinCE]);
      T.Dependencies.AddUnit('system');
    T:=P.Targets.AddUnit('sysinitpas.pp',AllWindowsOSes-[WinCE]);
      T.Dependencies.AddUnit('system');
    T:=P.Targets.AddUnit('windows.pp',AllWindowsOSes);
      T.IncludePath.Add('win/wininc');
      with T.Dependencies do
        begin
          AddInclude('base.inc');
          AddInclude('errors.inc');
          AddInclude('defines.inc');
          AddInclude('struct.inc');
          AddInclude('messages.inc');
          AddInclude('ascfun.inc');
          AddInclude('unifun.inc');
          AddInclude('ascdef.inc');
          AddInclude('func.inc');
          AddInclude('redef.inc');
          AddInclude('base.inc');
          AddInclude('errors.inc');
          AddInclude('defines.inc');
          AddInclude('struct.inc');
          AddInclude('messages.inc');
          AddInclude('ascfun.inc');
          AddInclude('unifun.inc');
          AddInclude('ascdef.inc');
          AddInclude('func.inc');
          AddInclude('redef.inc');
          AddUnit('objpas');
        end;
    T:=P.Targets.AddUnit('winevent.pp',AllWindowsOSes);
      with T.Dependencies do
        begin
          AddUnit('windows');
        end;
    T:=P.Targets.AddUnit('winsock2.pp',AllWindowsOSes);
      with T.Dependencies do
        begin
          AddUnit('windows');
        end;
    T:=P.Targets.AddUnit('winsock.pp',AllWindowsOSes);
      with T.Dependencies do
        begin
          AddUnit('windows');
        end;
    T:=P.Targets.AddUnit('winsysut.pp',AllWindowsOSes-[WinCE]);
      with T.Dependencies do
        begin
          AddUnit('windows');
          AddUnit('sysutils');
        end;
    T:=P.Targets.AddUnit('sharemem.pp',AllWindowsOSes);
      with T.Dependencies do
        begin
          AddUnit('system');
        end;
    T:=P.Targets.AddUnit('signals.pp',[Win32]);
      with T.Dependencies do
        begin
          AddUnit('system');
        end;
    T:=P.Targets.AddUnit('messages.pp',AllWindowsOSes);
      T.IncludePath.Add('win/wininc');
      with T.Dependencies do
        begin
          AddInclude('messages.inc');
          AddUnit('windows');
        end;

    // Turbo Pascal RTL units
    T:=P.Targets.AddUnit('strings.pp');
      With T.Dependencies do
        begin
          AddUnit('system');
          AddInclude('strings.inc');
          AddInclude('stringss.inc');
          AddInclude('genstr.inc');
          AddInclude('genstrs.inc');
          AddInclude('stringsi.inc');
        end;
    T:=P.Targets.AddUnit('dos.pp');
      With T.Dependencies do
        begin
          AddUnit('strings');
          AddUnit('unix',AllUnixOSes);
          AddInclude('dosh.inc');
          AddInclude('filerec.inc');
          AddInclude('textrec.inc');
          AddInclude('dos.inc');
          AddInclude('fexpand.inc');
        end;
    T:=P.Targets.AddUnit('crt.pp');
      With T.Dependencies do
        begin
          AddUnit('unix',AllUnixOSes);
          AddUnit('termio',AllUnixOSes);
          AddInclude('crth.inc');
          AddInclude('textrec.inc');
        end;
    T:=P.Targets.AddUnit('objects.pp');
      T.Dependencies.AddUnit('dos');
    T:=P.Targets.AddUnit('printer.pp');
      with T.Dependencies do
        begin
          AddInclude('printerh.inc');
          AddInclude('printer.inc');
          AddInclude('textrec.inc');
          AddUnit('unix',AllUnixOSes);
          AddUnit('strings');
        end;

    // Object Pascal RTL units
    T:=P.Targets.AddUnit('rtlconsts.pp');
      T.Dependencies.AddUnit('objpas');
      T.Dependencies.AddInclude('rtlconst.inc');
    T:=P.Targets.AddUnit('sysconst.pp');
      T.Dependencies.AddUnit('objpas');
    T:=P.Targets.AddUnit('sysutils.pp');
      T.IncludePath.Add('objpas/sysutils');
      With T.Dependencies do
        begin
          AddUnit('sysconst');
          AddUnit('unix',AllUnixOSes);
          AddUnit('errors',AllUnixOSes);
          AddUnit('windows',AllWindowsOSes);
          AddInclude('sysutilh.inc');
          AddInclude('sysinth.inc');
          AddInclude('osutilsh.inc');
          AddInclude('datih.inc');
          AddInclude('sysstrh.inc');
          AddInclude('filerec.inc');
          AddInclude('textrec.inc');
          AddInclude('syspchh.inc');
          AddInclude('sysansih.inc');
          AddInclude('syswideh.inc');
<<<<<<< HEAD
<<<<<<< HEAD
          AddInclude('sysunih.inc');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          AddInclude('finah.inc');
          AddInclude('filutilh.inc');
          AddInclude('diskh.inc');
          AddInclude('systhrdh.inc');
          AddInclude('intfh.inc');
          AddInclude('sysutils.inc');
          AddInclude('fina.inc');
          AddInclude('fexpand.inc');
          AddInclude('varerror.inc');
          AddInclude('sysstr.inc');
          AddInclude('sysformt.inc');
          AddInclude('dati.inc');
          AddInclude('syspch.inc');
          AddInclude('strings.inc');
          AddInclude('genstr.inc');
          AddInclude('stringsi.inc');
          AddInclude('sysint.inc');
          AddInclude('sysansi.inc');
          AddInclude('syswide.inc');
<<<<<<< HEAD
<<<<<<< HEAD
          AddInclude('sysuni.inc');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          AddInclude('sysformt.inc');
          AddInclude('sysuthrd.inc');
          AddInclude('osutil.inc');
          AddInclude('sysuintf.inc');
          AddInclude('suuid.inc',AllUnixOSes);
        end;
    T:=P.Targets.AddUnit('types.pp');
      With T.Dependencies do
        begin
          AddUnit('objpas');
          AddUnit('windows',AllWindowsOSes);
        end;
    T:=P.Targets.AddUnit('typinfo.pp');
      T.Dependencies.AddUnit('sysutils');
    T:=P.Targets.AddUnit('classes.pp');
      T.IncludePath.Add('objpas/classes');
      With T.Dependencies do
        begin
          AddUnit('sysutils');
          AddUnit('types');
          AddUnit('typinfo');
          AddUnit('unix',AllUnixOSes);
          AddUnit('windows',AllWindowsOSes);
          AddInclude('classesh.inc');
          AddInclude('classes.inc');
          AddInclude('resref.inc');
          AddInclude('sllist.inc');
          AddInclude('util.inc');
          AddInclude('bits.inc');
          AddInclude('streams.inc');
          AddInclude('parser.inc');
          AddInclude('collect.inc');
          AddInclude('lists.inc');
          AddInclude('stringl.inc');
          AddInclude('tthread.inc');
          AddInclude('persist.inc');
          AddInclude('compon.inc');
          AddInclude('action.inc');
          AddInclude('dm.inc');
          AddInclude('cregist.inc');
          AddInclude('intf.inc');
          AddInclude('filer.inc');
          AddInclude('reader.inc');
          AddInclude('writer.inc');
          AddInclude('twriter.inc');
        end;
    T:=P.Targets.AddUnit('convutils.pp');
      T.Dependencies.AddUnit('objpas');
      T.Dependencies.AddInclude('convutil.inc');
    T:=P.Targets.AddUnit('dateutils.pp');
      With T.Dependencies do
        begin
          AddUnit('sysutils');
          AddUnit('math');
          AddUnit('types');
          AddUnit('sysconst');
          AddInclude('dateutil.inc');
        end;
    T:=P.Targets.AddUnit('stdconvs.pp');
      T.Dependencies.AddUnit('convutils');
    T:=P.Targets.AddUnit('strutils.pp');
      T.Dependencies.AddUnit('sysutils');
    T:=P.Targets.AddUnit('varutils.pp');
      With T.Dependencies do
        begin
          AddUnit('sysutils');
          AddInclude('varutilh.inc');
          AddInclude('varerror.inc');
          AddInclude('cvarutil.inc');
          AddInclude('varutils.inc',AllOSes-AllWindowsOSes);
        end;
    T:=P.Targets.AddUnit('variants.pp');
      With T.Dependencies do
        begin
          AddUnit('sysutils');
          AddUnit('rtlconsts');
          AddUnit('typinfo');
        end;
    T:=P.Targets.AddUnit('fgl.pp');
      T.Dependencies.AddUnit('sysutils');
      T.Dependencies.AddUnit('types');
    T:=P.Targets.AddUnit('fmtbcd.pp');
      T.Dependencies.AddUnit('sysutils');
      T.Dependencies.AddUnit('variants');
    T:=P.Targets.AddUnit('math.pp');
      With T.Dependencies do
        begin
          AddUnit('sysutils');
<<<<<<< HEAD
<<<<<<< HEAD
=======
          AddInclude('mathuh.inc');
>>>>>>> graemeg/fixes_2_2
=======
          AddInclude('mathuh.inc');
>>>>>>> origin/fixes_2_2
          AddInclude('mathu.inc');
        end;

    // CPU dependent units
    T:=P.Targets.AddUnit('cpu.pp',[i386],AllOSes);
      T.Dependencies.AddUnit('system');
    T:=P.Targets.AddUnit('mmx.pp',[i386],AllOSes);
      T.Dependencies.AddUnit('cpu');

    // C Interfacing units
    T:=P.Targets.AddUnit('ctypes.pp');
      With T.Dependencies do
        begin
          AddUnit('system');
          AddUnit('unixtype',AllUnixOSes);
          AddInclude('aliasctp.inc',AllUnixOSes);
        end;
    T:=P.Targets.AddUnit('initc.pp',AllOSes-[WinCE]);
      T.Dependencies.AddUnit('ctypes');
    T:=P.Targets.AddUnit('cmem.pp');
      T.Dependencies.AddUnit('system');
    T:=P.Targets.AddUnit('cthreads.pp',AllUnixOSes);
      With T.Dependencies do
        begin
          AddUnit('objpas');
          AddUnit('initc');
          AddUnit('unix');
          AddUnit('dl');
          AddInclude('pthread.inc');
        end;
    T:=P.Targets.AddUnit('cwstring.pp',AllUnixOSes);
      With T.Dependencies do
        begin
          AddUnit('objpas');
          AddUnit('initc');
          AddUnit('unix');
          AddUnit('ctypes');
        end;

    // Misc units
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpextres.pp',[Darwin]);
      with T.Dependencies do
        begin
          AddInclude('extres_multiarch.inc',[Darwin]);
          AddInclude('extres.inc',AllOSes-[Darwin]);
        end;
    T:=P.Targets.AddUnit('fpintres.pp',AllUnixOSes+AllWindowsOSes);
      with T.Dependencies do
        begin
          AddUnit('baseunix',AllUnixOSes);
          AddInclude('intres.inc',AllOSes-AllWindowsOSes);
          AddInclude('winres.inc',AllWindowsOSes);
        end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('charset.pp');
      with T.Dependencies do
        begin
          AddUnit('objpas');
        end;
    T:=P.Targets.AddUnit('ucomplex.pp');
      with T.Dependencies do
        begin
          AddUnit('math');
        end;
    T:=P.Targets.AddUnit('matrix.pp');
      with T.Dependencies do
        begin
          AddUnit('system');
          AddInclude('mvecimp.inc');
          AddInclude('mmatimp.inc');
        end;
    T:=P.Targets.AddUnit('getopts.pp');
      with T.Dependencies do
        begin
          AddUnit('system');
        end;
    T:=P.Targets.AddUnit('dynlibs.pas');
      With T.Dependencies do
        begin
          AddUnit('objpas');
          AddUnit('dl',AllUnixOSes);
          AddInclude('dynlibs.inc');
        end;

    // Debugging units
    T:=P.Targets.AddUnit('exeinfo.pp');
      with T.Dependencies do
        begin
          AddUnit('objpas');
          AddUnit('strings');
        end;
    T:=P.Targets.AddUnit('heaptrc.pp');
      with T.Dependencies do
        begin
          AddUnit('system');
        end;
    T:=P.Targets.AddUnit('lineinfo.pp');
      with T.Dependencies do
        begin
          AddUnit('exeinfo');
        end;
    T:=P.Targets.AddUnit('lnfodwrf.pp');
      with T.Dependencies do
        begin
          AddUnit('exeinfo');
        end;

    // IO units
    T:=P.Targets.AddUnit('mouse.pp');
      with T.Dependencies do
        begin
          AddUnit('video');
          AddUnit('gpm',[Linux]);
          AddInclude('mouseh.inc');
          AddInclude('mouse.inc');
        end;
    T:=P.Targets.AddUnit('video.pp');
      with T.Dependencies do
        begin
          AddUnit('baseunix',AllUnixOSes);
          AddUnit('termio',AllUnixOSes);
          AddUnit('strings');
          AddUnit('linuxvcs',[Linux]);
          AddUnit('windows',AllWindowsOSes);
          AddUnit('dos',AllWindowsOSes);
          AddInclude('videoh.inc');
          AddInclude('video.inc');
          AddInclude('convert.inc',AllUnixOSes);
        end;
    T:=P.Targets.AddUnit('keyboard.pp');
       with T.Dependencies do
         begin
           AddInclude('keybrdh.inc');
           AddInclude('keyboard.inc');
           AddInclude('keyscan.inc');
           AddUnit('mouse');
         end;
    T:=P.Targets.AddUnit('sockets.pp',AllUnixOSes+AllWindowsOSes+[OS2,MorphOS,Netware,Netwlibc]);
      with T.Dependencies do
        begin
          AddUnit('baseunix',AllUnixOSes);
          AddUnit('winsock2',AllWindowsOSes);
          AddUnit('ctypes');
          AddInclude('socketsh.inc');
          AddInclude('filerec.inc');
          AddInclude('textrec.inc');
          AddInclude('sockovl.inc');
          AddInclude('sockets.inc');
          AddInclude('unxsockh.inc',AllUnixOSes);
          AddInclude('unixsock.inc',AllUnixOSes-[Solaris,Darwin]);
          AddInclude('fpwinsockh.inc',AllWindowsOSes);
        end;
    T:=P.Targets.AddUnit('serial.pp',AllUnixOSes);
      with T.Dependencies do
        begin
          AddUnit('objpas');
          AddUnit('termio',AllUnixOSes);
          AddUnit('unix',AllUnixOSes);
        end;

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
