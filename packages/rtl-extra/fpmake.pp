{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;
{$endif ALLPACKAGES}

procedure add_rtl_extra(const ADirectory: string);

Const
  // All Unices have full set of KVM+Crt in unix/ except QNX which is not
  // in workable state atm.
  UnixLikes = AllUnixOSes -[QNX]; // qnx never was active in 2.x afaik

  ClocaleOSes   = UnixLikes -[android];
  CLocaleIncOSes= [Aix,freebsd,netbsd,openbsd,solaris,darwin,iphonesim,dragonfly];

  IPCOSes       = UnixLikes-[aix,android,beos,haiku];
  IPCBSDs       = [FreeBSD,NetBSD,OpenBSD,DragonFly];
//  IPCcdeclOSes  = [Darwin,iphonesim];

  PrinterOSes   = [go32v2,msdos,os2,win32,win64]+unixlikes-[beos,haiku,morphos];
  SerialOSes    = [android,linux,netbsd,openbsd,win32,win64];
  UComplexOSes  = [atari,embedded,emx,gba,go32v2,msdos,nativent,nds,netware,netwlibc,os2,symbian,watcom,wii,wince,win32,win64,freertos]+UnixLikes+AllAmigaLikeOSes;
  MatrixOSes    = [atari,embedded,emx,gba,go32v2,msdos,nativent,nds,netware,netwlibc,os2,symbian,watcom,wii,win32,win64,wince,freertos]+UnixLikes+AllAmigaLikeOSes;
  ObjectsOSes   = [atari,embedded,emx,gba,go32v2,macos,msdos,nds,netware,netwlibc,os2,symbian,watcom,wii,win16,win32,win64,wince,freertos]+UnixLikes+AllAmigaLikeOSes;
  WinsockOSes   = [win32,win64,wince,os2,emx,netware,netwlibc];
  WinSock2OSes  = [win32,win64,wince];
  SocketsOSes   = UnixLikes+AllAmigaLikeOSes+[netware,netwlibc,os2,emx,wince,win32,win64];
  Socksyscall   = [beos,freebsd,haiku,linux,netbsd,openbsd,dragonfly];
  Socklibc  = unixlikes-socksyscall;
  gpmOSes = [Linux,Android];
  AllTargetsextra = ObjectsOSes + UComplexOSes + MatrixOSes+
                      SerialOSes +PrinterOSes+SocketsOSes+gpmOSes;

Var
  P : TPackage;
  T : TTarget;

begin
  With Installer do
    begin
    P:=AddPackage('rtl-extra');
    P.ShortName:='rtle';
    P.Directory:=ADirectory;
    P.Version:='3.3.1';
    P.Author := 'FPC core team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.OSes:=AllTargetsextra;
    if Defaults.CPU=jvm then
      P.OSes := P.OSes - [java,android];

    P.Email := '';
    P.Description := 'Rtl-extra, RTL not needed for bootstrapping';
    P.NeedLibC:= false;
    P.Dependencies.Add('morphunits',[morphos]);
    P.Dependencies.Add('arosunits',[aros]);
    if Defaults.CPU=m68k then
      P.Dependencies.Add('amunits',[amiga]);
    if Defaults.CPU=powerpc then
      P.Dependencies.Add('os4units',[amiga]);

    P.SourcePath.Add('src/inc');
    P.SourcePath.Add('src/$(OS)');
    P.SourcePath.Add('src/darwin',[iphonesim]);
    P.SourcePath.Add('src/unix',AllUnixOSes);
    P.SourcePath.Add('src/bsd',AllBSDOSes);
    P.SourcePath.Add('src/os2commn',[os2,emx]);
    P.SourcePath.Add('src/netwcomn',[netware,netwlibc]);
    // We also need the win/ directory for WinCE as this uses the sockets
    // unit from that directory. Maybe we should try to merge the WinSock(2)
    // units to remove the wince directory completely...
    P.SourcePath.Add('src/win',[win32,win64,wince]);
    P.SourcePath.Add('src/amiga',[morphos]);

    P.IncludePath.Add('src/bsd',AllBSDOSes);
    P.IncludePath.Add('src/inc');
    P.IncludePath.Add('src/os2commn',[os2,emx]);
    P.IncludePath.Add('../../rtl/os2',[os2,emx]);
    P.IncludePath.Add('src/netwcomn',[netware,netwlibc]);
    P.IncludePath.Add('src/unix',AllUnixOSes);
    P.IncludePath.Add('src/$(OS)');
    P.IncludePath.Add('src/darwin',[iphonesim]);
    P.IncludePath.Add('src/win',AllWindowsOSes);

    // Add clocale for Android first in order to compile the source file
    // from the 'android' dir, not the 'unix' dir.
    T:=P.Targets.AddUnit('real48utils.pp',AllTargetsextra-[msdos,win16]  { msdos,win16 excluded temporarily, until bitpacked records containing longints on 16-bit targets are fixed }
                                                         -[embedded]);   { at least avr has no floats }
    if Defaults.CPU<>jvm then
      T:=P.Targets.AddUnit('clocale.pp',[android]);

    { Ideally, we should check if rtl contians math unit,
      I do know how that can be checked. PM 2019/11/27 }
    if (Defaults.CPU<>i8086) or (Defaults.OS<>embedded) then
      T:=P.Targets.AddUnit('ucomplex.pp',UComplexOSes);

    T:=P.Targets.AddUnit('objects.pp',ObjectsOSes);

    T:=P.Targets.AddUnit('printer.pp',PrinterOSes);
    T.Dependencies.AddInclude('printerh.inc',PrinterOSes);
    T.Dependencies.AddInclude('printer.inc',PrinterOSes);

    { Ideally, we should check if rtl contians math unit,
      I do know how that can be checked. PM 2019/11/27 }
    if (Defaults.CPU<>i8086) or (Defaults.OS<>embedded) then
      begin
        T:=P.Targets.AddUnit('matrix.pp',MatrixOSes);
        with T.Dependencies do
          begin
            AddInclude('mvecimp.inc');
            AddInclude('mmatimp.inc');
          end;
      end;
    T:=P.Targets.AddUnit('winsock.pp',WinSockOSes);
    with T.Dependencies do
     begin
       AddInclude('qos.inc',[netware,netwlibc]);
       AddInclude('netwsockh.inc',[netware,netwlibc]);
     end;
    T:=P.Targets.AddUnit('winsock2.pp',WinSock2OSes);
    T:=P.Targets.AddUnit('gpm.pp',gpmOSes);
    with T.Dependencies do
      AddUnit('sockets');

    T:=P.Targets.AddUnit('serial.pp',SerialOSes);
    T:=P.Targets.AddUnit('sockets.pp',SocketsOSes);
    with T.Dependencies do
     begin
       addinclude('osdefs.inc',AllUnixOSes);
       addinclude('socketsh.inc');
       addinclude('fpwinsockh.inc',AllWindowsOSes);
       addinclude('sockets.inc');
       addinclude('sockovl.inc');
       addinclude('unxsockh.inc',UnixLikes);
       addinclude('stdsock.inc',socklibc);
       addinclude('unixsock.inc',socksyscall);
     end;

    T:=P.Targets.AddUnit('ipc.pp',IPCOSes);
    with T.Dependencies do
     begin
       addinclude('osdefs.inc');
       addinclude('ipcbsd.inc',IPCBSDs);
       addinclude('ipcsys.inc',[Linux]);
       addinclude('ipccall.inc',[Linux]);
//       addinclude('ipccdecl.inc',IPCcdeclOSes); // not used?
     end;
    T:=P.Targets.AddUnit('src/unix/clocale.pp',CLocaleOSes);
    with T.Dependencies do
     begin
       addinclude('clocale.inc',clocaleincOSes);
     end;
    T:=P.Targets.AddUnit('sortalgs.pp');
  end
end;

{$ifndef ALLPACKAGES}
begin
  add_rtl_extra('');
  Installer.Run;
end.
{$endif ALLPACKAGES}

