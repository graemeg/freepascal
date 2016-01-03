{$mode objfpc}
{$h+}

Unit kernelioctl;

Interface

const
   _IOC_NRBITS = 8;
   _IOC_TYPEBITS = 8;
   _IOC_SIZEBITS = 14;
   _IOC_DIRBITS = 2;
   _IOC_NRMASK = (1 shl _IOC_NRBITS) - 1;
   _IOC_TYPEMASK = (1 shl _IOC_TYPEBITS) - 1;
   _IOC_SIZEMASK = (1 shl _IOC_SIZEBITS) - 1;
   _IOC_DIRMASK = (1 shl _IOC_DIRBITS) - 1;
   _IOC_NRSHIFT = 0;
   _IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS;
   _IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS;
   _IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS;
   _IOC_NONE = 0;
   _IOC_WRITE = 1;
   _IOC_READ = 2;

Function _IOC(dir, __type, nr, size: cardinal): cardinal;
Function _IOR(_type,nr,size : cardinal) : cardinal;
Function _IOW(_type,nr,size : cardinal) : cardinal;
Function _IOWR(_type,nr,size : cardinal) : cardinal;
Function _IOC_DIR(nr : cardinal) : cardinal;
Function _IOC_TYPE(nr : cardinal) : cardinal;
Function _IOC_NR(nr : cardinal) : cardinal;
Function _IOC_SIZE(nr : cardinal) : cardinal;
Function _IO(__type, nr: Cardinal): cardinal;


const
  IOC_IN = _IOC_WRITE shl _IOC_DIRSHIFT;
  IOC_OUT = _IOC_READ shl _IOC_DIRSHIFT;
  IOC_INOUT = (_IOC_WRITE or _IOC_READ) shl _IOC_DIRSHIFT;
  IOCSIZE_MASK = _IOC_SIZEMASK shl _IOC_SIZESHIFT;
  IOCSIZE_SHIFT = _IOC_SIZESHIFT;

const
   TCGETS = $5401;
   TCSETS = $5402;
   TCSETSW = $5403;
   TCSETSF = $5404;
   TCGETA = $5405;
   TCSETA = $5406;
   TCSETAW = $5407;
   TCSETAF = $5408;
   TCSBRK = $5409;
   TCXONC = $540A;
   TCFLSH = $540B;
   TIOCEXCL = $540C;
   TIOCNXCL = $540D;
   TIOCSCTTY = $540E;
   TIOCGPGRP = $540F;
   TIOCSPGRP = $5410;
   TIOCOUTQ = $5411;
   TIOCSTI = $5412;
   TIOCGWINSZ = $5413;
   TIOCSWINSZ = $5414;
   TIOCMGET = $5415;
   TIOCMBIS = $5416;
   TIOCMBIC = $5417;
   TIOCMSET = $5418;
   TIOCGSOFTCAR = $5419;
   TIOCSSOFTCAR = $541A;
   FIONREAD = $541B;
   TIOCINQ = FIONREAD;
   TIOCLINUX = $541C;
   TIOCCONS = $541D;
   TIOCGSERIAL = $541E;
   TIOCSSERIAL = $541F;
   TIOCPKT = $5420;
   FIONBIO = $5421;
   TIOCNOTTY = $5422;
   TIOCSETD = $5423;
   TIOCGETD = $5424;
   TCSBRKP = $5425;
   TIOCTTYGSTRUCT = $5426;
   TIOCSBRK = $5427;
   TIOCCBRK = $5428;
   TIOCGSID = $5429;

Function TIOCGPTN : Cardinal;
Function TIOCSPTLCK : Cardinal;
Function TIOCGDEV : Cardinal;

const
   FIONCLEX = $5450;
   FIOCLEX = $5451;
   FIOASYNC = $5452;
   TIOCSERCONFIG = $5453;
   TIOCSERGWILD = $5454;
   TIOCSERSWILD = $5455;
   TIOCGLCKTRMIOS = $5456;
   TIOCSLCKTRMIOS = $5457;
   TIOCSERGSTRUCT = $5458;
   TIOCSERGETLSR = $5459;
   TIOCSERGETMULTI = $545A;
   TIOCSERSETMULTI = $545B;
   TIOCMIWAIT = $545C;
   TIOCGICOUNT = $545D;
   TIOCGHAYESESP = $545E;
   TIOCSHAYESESP = $545F;
   TIOCPKT_DATA = 0;
   TIOCPKT_FLUSHREAD = 1;
   TIOCPKT_FLUSHWRITE = 2;
   TIOCPKT_STOP = 4;
   TIOCPKT_START = 8;
   TIOCPKT_NOSTOP = 16;
   TIOCPKT_DOSTOP = 32;
   TIOCSER_TEMT = $01;

Implementation

Function _IOC(dir, __type, nr, size: cardinal): cardinal;

begin
  Result:=(dir shl _IOC_DIRSHIFT) or (__type shl _IOC_TYPESHIFT) or
          (nr shl _IOC_NRSHIFT) or (size shl _IOC_SIZESHIFT);
end;


Function _IO(__type, nr: Cardinal): cardinal;
begin
  Result := _IOC(_IOC_NONE,__type,nr,0);
end;


Function _IOR(_type,nr,size : cardinal) : cardinal;
begin
   _IOR:=_IOC(_IOC_READ,_type,nr,size);
end;


Function _IOW(_type,nr,size : cardinal) : cardinal;
begin
   _IOW:=_IOC(_IOC_WRITE,_type,nr,size);
end;


Function _IOWR(_type,nr,size : cardinal) : cardinal;
begin
   _IOWR:=_IOC(_IOC_READ or _IOC_WRITE,_type,nr,size);
end;


Function _IOC_DIR(nr : cardinal) : cardinal;
begin
   _IOC_DIR:=(nr shr _IOC_DIRSHIFT) and _IOC_DIRMASK;
end;


Function _IOC_TYPE(nr : cardinal) : cardinal;
begin
   _IOC_TYPE:=(nr shr _IOC_TYPESHIFT) and _IOC_TYPEMASK;
end;


Function _IOC_NR(nr : cardinal) : cardinal;
begin
   _IOC_NR:=(nr shr _IOC_NRSHIFT) and _IOC_NRMASK;
end;


Function _IOC_SIZE(nr : cardinal) : cardinal;
begin
   _IOC_SIZE:=(nr shr _IOC_SIZESHIFT) and _IOC_SIZEMASK;
end;


Function TIOCGPTN : cardinal;
begin
   TIOCGPTN:=_IOR(ord('T'),$30,4);
end;

Function TIOCSPTLCK : cardinal;

begin
   TIOCSPTLCK:=_IOW(ord('T'),$31,4);
end;

Function TIOCGDEV : cardinal;

begin
   TIOCGDEV:=_IOR(ord('T'),$32,4);
end;

end.
