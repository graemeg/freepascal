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

    P:=AddPackage('libc');
{$ifdef ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';

    P.Author := 'Peter Vreman and Michael van Canneyt (?)';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Kylix compatibility libc header, (linux/x86 only, deprecated for new development)';
    P.NeedLibC:= true;

    P.SourcePath.Add('src');
    P.IncludePath.Add('src');
=======
=======
>>>>>>> origin/fixes_2_2
    P.Directory:='libc';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';
    P.SourcePath.Add('src');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    P.OSES:=[linux];
    p.cpus:=[i386];
    T:=P.Targets.AddUnit('kerneldefs.pp');
    T:=P.Targets.AddUnit('kernelioctl.pp');
    T:=P.Targets.AddUnit('libc.pp');
      with T.Dependencies do
        begin
          AddInclude('glue.inc');
          AddInclude('endianh.inc');
          AddInclude('typesh.inc');
          AddInclude('posixopth.inc');
          AddInclude('stdinth.inc');
          AddInclude('wordsizeh.inc');
          AddInclude('limitsh.inc');
          AddInclude('posix1_limh.inc');
          AddInclude('posix2_limh.inc');
          AddInclude('xopen_limh.inc');
          AddInclude('local_limh.inc');
          AddInclude('inttypesh.inc');
          AddInclude('errnoh.inc');
          AddInclude('pathsh.inc');
          AddInclude('lib_namesh.inc');
          AddInclude('xlocaleh.inc');
          AddInclude('sigcontexth.inc');
          AddInclude('sigseth.inc');
          AddInclude('signumh.inc');
          AddInclude('siginfoh.inc');
          AddInclude('sigstackh.inc');
          AddInclude('sigactionh.inc');
          AddInclude('signalh.inc');
          AddInclude('btimeh.inc');
          AddInclude('timeh.inc');
          AddInclude('stimeh.inc');
          AddInclude('timexh.inc');
          AddInclude('timesh.inc');
          AddInclude('bschedh.inc');
          AddInclude('schedh.inc');
          AddInclude('pthreadtypesh.inc');
          AddInclude('pthreadh.inc');
          AddInclude('sigthreadh.inc');
          AddInclude('semaphoreh.inc');
          AddInclude('spawnh.inc');
          AddInclude('bfcntlh.inc');
          AddInclude('fcntlh.inc');
          AddInclude('fileh.inc');
          AddInclude('bdirenth.inc');
          AddInclude('direnth.inc');
          AddInclude('bstath.inc');
          AddInclude('sstath.inc');
          AddInclude('fnmatchh.inc');
          AddInclude('gconvh.inc');
          AddInclude('gconfigh.inc');
          AddInclude('libioh.inc');
          AddInclude('stdioh.inc');
          AddInclude('stdio_limh.inc');
          AddInclude('stdio_exth.inc');
          AddInclude('bconfnameh.inc');
          AddInclude('unistdh.inc');
          AddInclude('fstabh.inc');
          AddInclude('mntenth.inc');
          AddInclude('ioctlsh.inc');
          AddInclude('ioctl_typesh.inc');
          AddInclude('btermiosh.inc');
          AddInclude('termiosh.inc');
          AddInclude('sttydefaultsh.inc');
          AddInclude('sioctlh.inc');
          AddInclude('srawh.inc');
          AddInclude('ptyh.inc');
          AddInclude('smounth.inc');
          AddInclude('ssysctlh.inc');
          AddInclude('stringh.inc');
          AddInclude('stdlibh.inc');
          AddInclude('malloch.inc');
          AddInclude('ssysinfoh.inc');
          AddInclude('bdlfcnh.inc');
          AddInclude('dlfcnh.inc');
          AddInclude('localeh.inc');
          AddInclude('nl_typesh.inc');
          AddInclude('langinfoh.inc');
          AddInclude('wordexph.inc');
          AddInclude('iconvh.inc');
          AddInclude('bresourceh.inc');
          AddInclude('sresourceh.inc');
          AddInclude('argzh.inc');
          AddInclude('envzh.inc');
          AddInclude('ctypeh.inc');
          AddInclude('wctypeh.inc');
          AddInclude('wcharh.inc');
          AddInclude('bwaitflagsh.inc');
          AddInclude('bwaitstatush.inc');
          AddInclude('swaith.inc');
          AddInclude('butsnameh.inc');
          AddInclude('sutsnameh.inc');
          AddInclude('bmmanh.inc');
          AddInclude('smmaph.inc');
          AddInclude('ssyslogh.inc');
          AddInclude('glibc_versionh.inc');
          AddInclude('buioh.inc');
          AddInclude('suioh.inc');
          AddInclude('asockiosh.inc');
          AddInclude('asocketh.inc');
          AddInclude('bsockaddrh.inc');
          AddInclude('bsocketh.inc');
          AddInclude('ssocketh.inc');
          AddInclude('sunh.inc');
          AddInclude('ninh.inc');
          AddInclude('binh.inc');
          AddInclude('aineth.inc');
          AddInclude('bnetdbh.inc');
          AddInclude('netdbh.inc');
          AddInclude('sselecth.inc');
          AddInclude('pwdh.inc');
          AddInclude('grph.inc');
          AddInclude('sptraceh.inc');
          AddInclude('ulimith.inc');
          AddInclude('bpollh.inc');
          AddInclude('spollh.inc');
          AddInclude('utimeh.inc');
          AddInclude('sysexitsh.inc');
          AddInclude('bustath.inc');
          AddInclude('sustath.inc');
          AddInclude('errh.inc');
          AddInclude('errorh.inc');
          AddInclude('bfenvh.inc');
          AddInclude('fenvh.inc');
          AddInclude('bipch.inc');
          AddInclude('sipch.inc');
          AddInclude('bshmh.inc');
          AddInclude('sshmh.inc');
          AddInclude('bsemh.inc');
          AddInclude('ssemh.inc');
          AddInclude('libgenh.inc');
          AddInclude('butmph.inc');
          AddInclude('utmph.inc');
          AddInclude('butmpxh.inc');
          AddInclude('utmpxh.inc');
          AddInclude('svtimesh.inc');
          AddInclude('svlimith.inc');
          AddInclude('sucontexth.inc');
          AddInclude('ucontexth.inc');
          AddInclude('bmsqh.inc');
          AddInclude('smsgh.inc');
          AddInclude('bstatfsh.inc');
          AddInclude('sstatfsh.inc');
          AddInclude('bstatvfsh.inc');
          AddInclude('sstatvfsh.inc');
          AddInclude('monetaryh.inc');
          AddInclude('mcheckh.inc');
          AddInclude('printfh.inc');
          AddInclude('libintlh.inc');
          AddInclude('shadowh.inc');
          AddInclude('fmtmsgh.inc');
          AddInclude('squotah.inc');
          AddInclude('stimebh.inc');
          AddInclude('spermh.inc');
          AddInclude('sswaph.inc');
          AddInclude('ssendfileh.inc');
          AddInclude('srebooth.inc');
          AddInclude('aioh.inc');
          AddInclude('aliasesh.inc');
          AddInclude('globh.inc');
          AddInclude('crypth.inc');
          AddInclude('sfsuidh.inc');
          AddInclude('sklogh.inc');
          AddInclude('skdaemonh.inc');
          AddInclude('saccth.inc');
          AddInclude('bstroptsh.inc');
          AddInclude('stroptsh.inc');
          AddInclude('allocah.inc');
          AddInclude('getopth.inc');
          AddInclude('argph.inc');
          AddInclude('nssh.inc');
          AddInclude('regexh.inc');
          AddInclude('netherneth.inc');
          AddInclude('nifh.inc');
          AddInclude('nif_arph.inc');
          AddInclude('nif_packeth.inc');
          AddInclude('nif_ppph.inc');
          AddInclude('nif_shaperh.inc');
          AddInclude('nrouteh.inc');
          AddInclude('nashh.inc');
          AddInclude('nath.inc');
          AddInclude('nax25h.inc');
          AddInclude('nech.inc');
          AddInclude('nipxh.inc');
          AddInclude('npacketh.inc');
          AddInclude('nnetromh.inc');
          AddInclude('nroseh.inc');
          AddInclude('nif_etherh.inc');
          AddInclude('netherh.inc');
          AddInclude('nicmp6h.inc');
          AddInclude('nif_fddih.inc');
          AddInclude('nif_trh.inc');
          AddInclude('nigmph.inc');
          AddInclude('nin_systmh.inc');
          AddInclude('niph.inc');
          AddInclude('nip6h.inc');
          AddInclude('nip_icmph.inc');
          AddInclude('ntcph.inc');
          AddInclude('nudph.inc');
          AddInclude('proutedh.inc');
          AddInclude('prwhodh.inc');
          AddInclude('ptalkdh.inc');
          AddInclude('ptimedh.inc');
          AddInclude('sscsih.inc');
          AddInclude('sscsi_ioctlh.inc');
          AddInclude('ssgh.inc');
          AddInclude('ttyenth.inc');
          AddInclude('sgttyh.inc');
          AddInclude('searchh.inc');
          AddInclude('types.inc');
          AddInclude('cerrno.inc');
          AddInclude('time.inc');
          AddInclude('stime.inc');
          AddInclude('dirent.inc');
          AddInclude('sstat.inc');
          AddInclude('libio.inc');
          AddInclude('termios.inc');
          AddInclude('sttydefaults.inc');
          AddInclude('sraw.inc');
          AddInclude('bwaitstatus.inc');
          AddInclude('ssyslog.inc');
          AddInclude('bsocket.inc');
          AddInclude('sun.inc');
          AddInclude('nin.inc');
          AddInclude('sselect.inc');
          AddInclude('squota.inc');
          AddInclude('nethernet.inc');
          AddInclude('nif_ppp.inc');
          AddInclude('nroute.inc');
          AddInclude('nip.inc');
          AddInclude('nif_ether.inc');
          AddInclude('nicmp6.inc');
          AddInclude('nip_icmp.inc');
          AddInclude('pthread.inc');
          AddUnit('kerneldefs');
          AddUnit('kernelioctl');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
     p.Sources.Adddoc('README.txt');
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
