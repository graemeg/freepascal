{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2004 by Marco van de Voort
    member of the Free Pascal development team

    ioctls constants for linux

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{********************
   IOCtl(TermIOS)
********************}

Const
  { Amount of Control Chars }
  NCCS = 32;
  NCC = 8;

{$ifdef cpuaarch64}

{ from Linux 4.0, include/uapi/asm-generic/ioctls.h }

  { For Terminal handling }
  TCGETS          = $5401;
  TCSETS          = $5402;
  TCSETSW         = $5403;
  TCSETSF         = $5404;
  TCGETA          = $5405;
  TCSETA          = $5406;
  TCSETAW         = $5407;
  TCSETAF         = $5408;
  TCSBRK          = $5409;
  TCXONC          = $540A;
  TCFLSH          = $540B;
  TIOCEXCL        = $540C;
  TIOCNXCL        = $540D;
  TIOCSCTTY       = $540E;
  TIOCGPGRP       = $540F;
  TIOCSPGRP       = $5410;
  TIOCOUTQ        = $5411;
  TIOCSTI         = $5412;
  TIOCGWINSZ      = $5413;
  TIOCSWINSZ      = $5414;
  TIOCMGET        = $5415;
  TIOCMBIS        = $5416;
  TIOCMBIC        = $5417;
  TIOCMSET        = $5418;
  TIOCGSOFTCAR    = $5419;
  TIOCSSOFTCAR    = $541A;
  FIONREAD        = $541B;
  TIOCINQ         = FIONREAD;
  TIOCLINUX       = $541C;
  TIOCCONS        = $541D;
  TIOCGSERIAL     = $541E;
  TIOCSSERIAL     = $541F;
  TIOCPKT         = $5420;
  FIONBIO         = $5421;
  TIOCNOTTY       = $5422;
  TIOCSETD        = $5423;
  TIOCGETD        = $5424;
  TCSBRKP         = $5425;

  TIOCSBRK        = $5427;
  TIOCCBRK        = $5428;
  TIOCGSID        = $5429;

  TIOCGRS485      = $542E;
  TIOCSRS485      = $542F;

  TCGETX          = $5432;
  TCSETX          = $5433;
  TCSETXF         = $5434;
  TCSETXW         = $5435;
  TIOCVHANGUP     = $5437;

  FIONCLEX        = $5450;
  FIOCLEX         = $5451;
  FIOASYNC        = $5452;
  TIOCSERCONFIG   = $5453;
  TIOCSERGWILD    = $5454;
  TIOCSERSWILD    = $5455;
  TIOCGLCKTRMIOS  = $5456;
  TIOCSLCKTRMIOS  = $5457;
  TIOCSERGSTRUCT  = $5458;
  TIOCSERGETLSR   = $5459;
  TIOCSERGETMULTI = $545A;
  TIOCSERSETMULTI = $545B;
  TIOCMIWAIT      = $545C;
  TIOCGICOUNT     = $545D;

  FIOQSIZE        = $5460;

  TIOCPKT_DATA       = 0;
  TIOCPKT_FLUSHREAD  = 1;
  TIOCPKT_FLUSHWRITE = 2;
  TIOCPKT_STOP       = 4;
  TIOCPKT_START      = 8;
  TIOCPKT_NOSTOP     = 16;
  TIOCPKT_DOSTOP     = 32;
  TIOCPKT_IOCTL      = 64;

  TIOCSER_TEMT       = $01;

{ from Linux 4.0, include/uapi/asm-generic/termbits.h }

  { c_cc characters }
  VINTR    = 0;
  VQUIT    = 1;
  VERASE   = 2;
  VKILL    = 3;
  VEOF     = 4;
  VTIME    = 5;
  VMIN     = 6;
  VSWTC    = 7;
  VSTART   = 8;
  VSTOP    = 9;
  VSUSP    = 10;
  VEOL     = 11;
  VREPRINT = 12;
  VDISCARD = 13;
  VWERASE  = 14;
  VLNEXT   = 15;
  VEOL2    = 16;

  { c_iflag bits }
  IGNBRK  = &0000001;
  BRKINT  = &0000002;
  IGNPAR  = &0000004;
  PARMRK  = &0000010;
  INPCK   = &0000020;
  ISTRIP  = &0000040;
  INLCR   = &0000100;
  IGNCR   = &0000200;
  ICRNL   = &0000400;
  IUCLC   = &0001000;
  IXON    = &0002000;
  IXANY   = &0004000;
  IXOFF   = &0010000;
  IMAXBEL = &0020000;
  IUTF8   = &0040000;

  { c_oflag bits }
  OPOST  = &0000001;
  OLCUC  = &0000002;
  ONLCR  = &0000004;
  OCRNL  = &0000010;
  ONOCR  = &0000020;
  ONLRET = &0000040;
  OFILL  = &0000100;
  OFDEL  = &0000200;
  NLDLY  = &0000400;
  NL0    = &0000000;
  NL1    = &0000400;
  CRDLY  = &0003000;
  CR0    = &0000000;
  CR1    = &0001000;
  CR2    = &0002000;
  CR3    = &0003000;
  TABDLY = &0014000;
  TAB0   = &0000000;
  TAB1   = &0004000;
  TAB2   = &0010000;
  TAB3   = &0014000;
  XTABS  = &0014000;
  BSDLY  = &0020000;
  BS0    = &0000000;
  BS1    = &0020000;
  VTDLY  = &0040000;
  VT0    = &0000000;
  VT1    = &0040000;
  FFDLY  = &0100000;
  FF0    = &0000000;
  FF1    = &0100000;

  { c_cflag bits }
  CBAUD    = &0010017;
  B0       = &0000000;
  B50      = &0000001;
  B75      = &0000002;
  B110     = &0000003;
  B134     = &0000004;
  B150     = &0000005;
  B200     = &0000006;
  B300     = &0000007;
  B600     = &0000010;
  B1200    = &0000011;
  B1800    = &0000012;
  B2400    = &0000013;
  B4800    = &0000014;
  B9600    = &0000015;
  B19200   = &0000016;
  B38400   = &0000017;
  EXTA     = B19200;
  EXTB     = B38400;
  CSIZE    = &0000060;
  CS5      = &0000000;
  CS6      = &0000020;
  CS7      = &0000040;
  CS8      = &0000060;
  CSTOPB   = &0000100;
  CREAD    = &0000200;
  PARENB   = &0000400;
  PARODD   = &0001000;
  HUPCL    = &0002000;
  CLOCAL   = &0004000;
  CBAUDEX  = &0010000;
  BOTHER   = &0010000;
  B57600   = &0010001;
  B115200  = &0010002;
  B230400  = &0010003;
  B460800  = &0010004;
  B500000  = &0010005;
  B576000  = &0010006;
  B921600  = &0010007;
  B1000000 = &0010010;
  B1152000 = &0010011;
  B1500000 = &0010012;
  B2000000 = &0010013;
  B2500000 = &0010014;
  B3000000 = &0010015;
  B3500000 = &0010016;
  B4000000 = &0010017;

  CIBAUD   = &002003600000;
  CMSPAR   = &010000000000;
  CRTSCTS  = &020000000000;

  IBSHIFT  = 16;

  { c_lflag bits }
  ISIG    = &0000001;
  ICANON  = &0000002;
  XCASE   = &0000004;
  ECHO    = &0000010;
  ECHOE   = &0000020;
  ECHOK   = &0000040;
  ECHONL  = &0000100;
  NOFLSH  = &0000200;
  TOSTOP  = &0000400;
  ECHOCTL = &0001000;
  ECHOPRT = &0002000;
  ECHOKE  = &0004000;
  FLUSHO  = &0010000;
  PENDIN  = &0040000;
  IEXTEN  = &0100000;
  EXTPROC = &0200000;

  { TCFlow }
  TCOOFF = 0;
  TCOON  = 1;
  TCIOFF = 2;
  TCION  = 3;

  { TCFlush }
  TCIFLUSH  = 0;
  TCOFLUSH  = 1;
  TCIOFLUSH = 2;

  { TCSetAttr }
  TCSANOW   = 0;
  TCSADRAIN = 1;
  TCSAFLUSH = 2;

{ from Linux 4.0, include/uapi/asm-generic/termios.h }

  { c_line bits }
  TIOCM_LE   = $001;
  TIOCM_DTR  = $002;
  TIOCM_RTS  = $004;
  TIOCM_ST   = $008;
  TIOCM_SR   = $010;
  TIOCM_CTS  = $020;
  TIOCM_CAR  = $040;
  TIOCM_RNG  = $080;
  TIOCM_DSR  = $100;
  TIOCM_CD   = TIOCM_CAR;
  TIOCM_RI   = TIOCM_RNG;
  TIOCM_OUT1 = $2000;
  TIOCM_OUT2 = $4000;
  TIOCM_LOOP = $8000;

{$endif cpuaarch64}

{$ifdef cpupowerpc}
  TCGETS            = $402c7413;
  TCSETS            = $802c7414;
  TCSETSW           = $802c7415;
  TCSETSF           = $802c7416;
  TCGETA            = $40147417;
  TCSETA            = $80147418;
  TCSETAW           = $80147419;
  TCSETAF           = $8014741c;
  TCSBRK            = $2000741d;
  TCXONC            = $2000741e;
  TCFLSH            = $2000741f;
  TIOCEXCL          = $540c;
  TIOCNXCL          = $540d;
  TIOCSCTTY         = $540e;
  TIOCGPGRP         = $40047477;
  TIOCSPGRP         = $80047476;
  TIOCOUTQ          = $40047473;
  TIOCSTI           = $5412;
  TIOCGWINSZ        = $40087468;
  TIOCSWINSZ        = $80087467;
  TIOCMGET          = $5415;
  TIOCMBIS          = $5416;
  TIOCMBIC          = $5417;
  TIOCMSET          = $5418;
  TIOCGSOFTCAR      = $5419;
  TIOCSSOFTCAR      = $541a;
  FIONREAD          = $4004667f;
  TIOCINQ           = $4004667f;
  TIOCLINUX         = $541c;
  TIOCCONS          = $541d;
  TIOCGSERIAL       = $541e;
  TIOCSSERIAL       = $541f;
  TIOCPKT           = $5420;
  FIONBIO           = $8004667e;
  TIOCNOTTY         = $5422;
  TIOCSETD          = $5423;
  TIOCGETD          = $5424;
  TCSBRKP           = $5425;
  TIOCTTYGSTRUCT    = $5426;
  FIONCLEX          = $20006602;
  FIOCLEX           = $20006601;
  FIOASYNC          = $8004667d;
  TIOCSERCONFIG     = $5453;
  TIOCSERGWILD      = $5454;
  TIOCSERSWILD      = $5455;
  TIOCGLCKTRMIOS    = $5456;
  TIOCSLCKTRMIOS    = $5457;
  TIOCSERGSTRUCT    = $5458;
  TIOCSERGETLSR     = $5459;
  TIOCSERGETMULTI   = $545a;
  TIOCSERSETMULTI   = $545b;
  TIOCMIWAIT        = $545c;
  TIOCGICOUNT       = $545d;

{c_cc characters}
  VINTR = 0;
  VQUIT = 1;
  VERASE = 2;
  VKILL = 3;
  VEOF = 4;
  VMIN = 5;
  VEOL = 6;
  VTIME = 7;
  VEOL2 = 8;
  VSWTC = 9;
  VWERASE = 10;
  VREPRINT = 11;
  VSUSP = 12;
  VSTART = 13;
  VSTOP = 14;
  VLNEXT = 15;
  VDISCARD = 16;

{ c_iflag bits }
	 IGNBRK	= &0000001;
	 BRKINT	= &0000002;
	 IGNPAR	= &0000004;
	 PARMRK	= &0000010;
	 INPCK	= &0000020;
	 ISTRIP	= &0000040;
	 INLCR	= &0000100;
	 IGNCR	= &0000200;
	 ICRNL	= &0000400;
	 IXON	= &0001000;
	 IXOFF	= &0002000;
	 IXANY	= &0004000;
	 IUCLC	= &0010000;
	 IMAXBEL= &0020000;

{ c_oflag bits }
	 OPOST	= &0000001;
	 ONLCR	= &0000002;
	 OLCUC	= &0000004;
	 OCRNL	= &0000010;
	 ONOCR	= &0000020;
	 ONLRET	= &0000040;

	 OFILL	= &00000100;
	 OFDEL	= &00000200;
	 NLDLY	= &00001400;
	   NL0	= &00000000;
	   NL1	= &00000400;
	   NL2	= &00001000;
	   NL3	= &00001400;
	 TABDLY	= &00006000;
	   TAB0	= &00000000;
	   TAB1	= &00002000;
	   TAB2	= &00004000;
	   TAB3	= &00006000;
	 CRDLY	= &00030000;
	   CR0	= &00000000;
	   CR1	= &00010000;
	   CR2	= &00020000;
	   CR3	= &00030000;
	 FFDLY	= &00040000;
	   FF0	= &00000000;
	   FF1	= &00040000;
	 BSDLY	= &00100000;
	   BS0	= &00000000;
	   BS1	= &00100000;
	 VTDLY	= &00200000;
	   VT0	= &00000000;
	   VT1	= &00200000;
	 XTABS	= &01000000; { Hmm.. Linux/i386 considers this part of TABDLY.. }

{ c_cflag bit meaning }
	 CBAUD	= &0000377;
	  B0	= &0000000;		{ hang up }
	  B50	= &0000001;
	  B75	= &0000002;
	  B110	= &0000003;
	  B134	= &0000004;
	  B150	= &0000005;
	  B200	= &0000006;
	  B300	= &0000007;
	  B600	= &0000010;
	  B1200	= &0000011;
	  B1800	= &0000012;
	  B2400	= &0000013;
	  B4800	= &0000014;
	  B9600	= &0000015;
	  B19200= &0000016;
	  B38400= &0000017;
	  EXTA  = B19200;
	  EXTB  = B38400;
	  CBAUDEX  = &0000020;
	  B57600   = &00020;
	  B115200  = &00021;
	  B230400  = &00022;
	  B460800  = &00023;
	  B500000  = &00024;
	  B576000  = &00025;
	  B921600  = &00026;
	  B1000000 = &00027;
	  B1152000 = &00030;
	  B1500000 = &00031;
	  B2000000 = &00032;
	  B2500000 = &00033;
	  B3000000 = &00034;
	  B3500000 = &00035;
	  B4000000 = &00036;
	 __MAX_BAUD=  B4000000;

	 CSIZE	= &00001400;
	   CS5	= &00000000;
	   CS6	= &00000400;
	   CS7	= &00001000;
	   CS8	= &00001400;

	 CSTOPB	= &00002000;
	 CREAD	= &00004000;
	 PARENB	= &00010000;
	 PARODD	= &00020000;
	 HUPCL	= &00040000;
	 CLOCAL	= &00100000;
	 CRTSCTS= &020000000000;		{ flow control }

{ c_lflag bits }
	 ISIG	= $00000080;
	 ICANON	= $00000100;
	 XCASE	= $00004000;
	 ECHO	= $00000008;
	 ECHOE	= $00000002;
	 ECHOK	= $00000004;
	 ECHONL	= $00000010;
	 NOFLSH	= $80000000;
	 TOSTOP	= $00400000;
	 ECHOCTL= $00000040;
	 ECHOPRT= $00000020;
	 ECHOKE	= $00000001;
	 FLUSHO	= $00800000;
	 PENDIN	= $20000000;
	 IEXTEN	= $00000400;

{ Values for the ACTION argument to `tcflow'.  }
		TCOOFF		= 0;
		TCOON		= 1;
		TCIOFF		= 2;
		TCION		= 3;

{ Values for the QUEUE_SELECTOR argument to `tcflush'.  }
		TCIFLUSH	= 0;
		TCOFLUSH	= 1;
		TCIOFLUSH	= 2;

{ Values for the OPTIONAL_ACTIONS argument to `tcsetattr'.  }
		TCSANOW		= 0;
		TCSADRAIN	= 1;
		TCSAFLUSH	= 2;

{ Used for packet mode }
	 TIOCPKT_DATA		=  0;
	 TIOCPKT_FLUSHREAD	=  1;
	 TIOCPKT_FLUSHWRITE	=  2;
	 TIOCPKT_STOP		=  4;
	 TIOCPKT_START		=  8;
	 TIOCPKT_NOSTOP		= 16;
	 TIOCPKT_DOSTOP		= 32;

{ c_cc characters }
	 _VINTR	= 0;
	 _VQUIT	= 1;
	 _VERASE= 2;
	 _VKILL	= 3;
	 _VEOF	= 4;
	 _VMIN	= 5;
	 _VEOL	= 6;
	 _VTIME	= 7;
	 _VEOL2	= 8;
	 _VSWTC	= 9;

{ modem lines }
	 TIOCM_LE	= $001;
	 TIOCM_DTR	= $002;
	 TIOCM_RTS	= $004;
	 TIOCM_ST	= $008;
	 TIOCM_SR	= $010;
	 TIOCM_CTS	= $020;
	 TIOCM_CAR	= $040;
	 TIOCM_RNG	= $080;
	 TIOCM_DSR	= $100;
	 TIOCM_CD	= TIOCM_CAR;
	 TIOCM_RI	= TIOCM_RNG;

{ ioctl (fd, TIOCSERGETLSR, &result) where result may be as below }
	 TIOCSER_TEMT    = $01;	{ Transmitter physically empty }

{ line disciplines }
	 N_TTY		= 0;
	 N_SLIP		= 1;
	 N_MOUSE	= 2;
	 N_PPP		= 3;
	 N_STRIP	= 4;
	 N_AX25		= 5;
	 N_X25		= 6;	{ X.25 async  }
	 N_6PACK	= 7;
	 N_MASC		= 8;	{ Mobitex module  }
	 N_R3964	= 9;	{ Simatic R3964 module  }
	 N_PROFIBUS_FDL	= 10;	{ Profibus  }
	 N_IRDA		= 11;	{ Linux IR  }
	 N_SMSBLOCK	= 12;	{ SMS block mode  }
	 N_HDLC		= 13;	{ synchronous HDLC  }
	 N_SYNC_PPP	= 14;	{ synchronous PPP  }
	 N_HCI		= 15;	{ Bluetooth HCI UART  }


{$endif cpupowerpc}

{$ifdef cpuriscv32}

{ from Linux 4.0, include/uapi/asm-generic/ioctls.h }

  { For Terminal handling }
  TCGETS          = $5401;
  TCSETS          = $5402;
  TCSETSW         = $5403;
  TCSETSF         = $5404;
  TCGETA          = $5405;
  TCSETA          = $5406;
  TCSETAW         = $5407;
  TCSETAF         = $5408;
  TCSBRK          = $5409;
  TCXONC          = $540A;
  TCFLSH          = $540B;
  TIOCEXCL        = $540C;
  TIOCNXCL        = $540D;
  TIOCSCTTY       = $540E;
  TIOCGPGRP       = $540F;
  TIOCSPGRP       = $5410;
  TIOCOUTQ        = $5411;
  TIOCSTI         = $5412;
  TIOCGWINSZ      = $5413;
  TIOCSWINSZ      = $5414;
  TIOCMGET        = $5415;
  TIOCMBIS        = $5416;
  TIOCMBIC        = $5417;
  TIOCMSET        = $5418;
  TIOCGSOFTCAR    = $5419;
  TIOCSSOFTCAR    = $541A;
  FIONREAD        = $541B;
  TIOCINQ         = FIONREAD;
  TIOCLINUX       = $541C;
  TIOCCONS        = $541D;
  TIOCGSERIAL     = $541E;
  TIOCSSERIAL     = $541F;
  TIOCPKT         = $5420;
  FIONBIO         = $5421;
  TIOCNOTTY       = $5422;
  TIOCSETD        = $5423;
  TIOCGETD        = $5424;
  TCSBRKP         = $5425;

  TIOCSBRK        = $5427;
  TIOCCBRK        = $5428;
  TIOCGSID        = $5429;

  TIOCGRS485      = $542E;
  TIOCSRS485      = $542F;

  TCGETX          = $5432;
  TCSETX          = $5433;
  TCSETXF         = $5434;
  TCSETXW         = $5435;
  TIOCVHANGUP     = $5437;

  FIONCLEX        = $5450;
  FIOCLEX         = $5451;
  FIOASYNC        = $5452;
  TIOCSERCONFIG   = $5453;
  TIOCSERGWILD    = $5454;
  TIOCSERSWILD    = $5455;
  TIOCGLCKTRMIOS  = $5456;
  TIOCSLCKTRMIOS  = $5457;
  TIOCSERGSTRUCT  = $5458;
  TIOCSERGETLSR   = $5459;
  TIOCSERGETMULTI = $545A;
  TIOCSERSETMULTI = $545B;
  TIOCMIWAIT      = $545C;
  TIOCGICOUNT     = $545D;

  FIOQSIZE        = $5460;

  TIOCPKT_DATA       = 0;
  TIOCPKT_FLUSHREAD  = 1;
  TIOCPKT_FLUSHWRITE = 2;
  TIOCPKT_STOP       = 4;
  TIOCPKT_START      = 8;
  TIOCPKT_NOSTOP     = 16;
  TIOCPKT_DOSTOP     = 32;
  TIOCPKT_IOCTL      = 64;

  TIOCSER_TEMT       = $01;

{ from Linux 4.0, include/uapi/asm-generic/termbits.h }

  { c_cc characters }
  VINTR    = 0;
  VQUIT    = 1;
  VERASE   = 2;
  VKILL    = 3;
  VEOF     = 4;
  VTIME    = 5;
  VMIN     = 6;
  VSWTC    = 7;
  VSTART   = 8;
  VSTOP    = 9;
  VSUSP    = 10;
  VEOL     = 11;
  VREPRINT = 12;
  VDISCARD = 13;
  VWERASE  = 14;
  VLNEXT   = 15;
  VEOL2    = 16;

  { c_iflag bits }
  IGNBRK  = &0000001;
  BRKINT  = &0000002;
  IGNPAR  = &0000004;
  PARMRK  = &0000010;
  INPCK   = &0000020;
  ISTRIP  = &0000040;
  INLCR   = &0000100;
  IGNCR   = &0000200;
  ICRNL   = &0000400;
  IUCLC   = &0001000;
  IXON    = &0002000;
  IXANY   = &0004000;
  IXOFF   = &0010000;
  IMAXBEL = &0020000;
  IUTF8   = &0040000;

  { c_oflag bits }
  OPOST  = &0000001;
  OLCUC  = &0000002;
  ONLCR  = &0000004;
  OCRNL  = &0000010;
  ONOCR  = &0000020;
  ONLRET = &0000040;
  OFILL  = &0000100;
  OFDEL  = &0000200;
  NLDLY  = &0000400;
  NL0    = &0000000;
  NL1    = &0000400;
  CRDLY  = &0003000;
  CR0    = &0000000;
  CR1    = &0001000;
  CR2    = &0002000;
  CR3    = &0003000;
  TABDLY = &0014000;
  TAB0   = &0000000;
  TAB1   = &0004000;
  TAB2   = &0010000;
  TAB3   = &0014000;
  XTABS  = &0014000;
  BSDLY  = &0020000;
  BS0    = &0000000;
  BS1    = &0020000;
  VTDLY  = &0040000;
  VT0    = &0000000;
  VT1    = &0040000;
  FFDLY  = &0100000;
  FF0    = &0000000;
  FF1    = &0100000;

  { c_cflag bits }
  CBAUD    = &0010017;
  B0       = &0000000;
  B50      = &0000001;
  B75      = &0000002;
  B110     = &0000003;
  B134     = &0000004;
  B150     = &0000005;
  B200     = &0000006;
  B300     = &0000007;
  B600     = &0000010;
  B1200    = &0000011;
  B1800    = &0000012;
  B2400    = &0000013;
  B4800    = &0000014;
  B9600    = &0000015;
  B19200   = &0000016;
  B38400   = &0000017;
  EXTA     = B19200;
  EXTB     = B38400;
  CSIZE    = &0000060;
  CS5      = &0000000;
  CS6      = &0000020;
  CS7      = &0000040;
  CS8      = &0000060;
  CSTOPB   = &0000100;
  CREAD    = &0000200;
  PARENB   = &0000400;
  PARODD   = &0001000;
  HUPCL    = &0002000;
  CLOCAL   = &0004000;
  CBAUDEX  = &0010000;
  BOTHER   = &0010000;
  B57600   = &0010001;
  B115200  = &0010002;
  B230400  = &0010003;
  B460800  = &0010004;
  B500000  = &0010005;
  B576000  = &0010006;
  B921600  = &0010007;
  B1000000 = &0010010;
  B1152000 = &0010011;
  B1500000 = &0010012;
  B2000000 = &0010013;
  B2500000 = &0010014;
  B3000000 = &0010015;
  B3500000 = &0010016;
  B4000000 = &0010017;

  CIBAUD   = &002003600000;
  CMSPAR   = &010000000000;
  CRTSCTS  = &020000000000;

  IBSHIFT  = 16;

  { c_lflag bits }
  ISIG    = &0000001;
  ICANON  = &0000002;
  XCASE   = &0000004;
  ECHO    = &0000010;
  ECHOE   = &0000020;
  ECHOK   = &0000040;
  ECHONL  = &0000100;
  NOFLSH  = &0000200;
  TOSTOP  = &0000400;
  ECHOCTL = &0001000;
  ECHOPRT = &0002000;
  ECHOKE  = &0004000;
  FLUSHO  = &0010000;
  PENDIN  = &0040000;
  IEXTEN  = &0100000;
  EXTPROC = &0200000;

  { TCFlow }
  TCOOFF = 0;
  TCOON  = 1;
  TCIOFF = 2;
  TCION  = 3;

  { TCFlush }
  TCIFLUSH  = 0;
  TCOFLUSH  = 1;
  TCIOFLUSH = 2;

  { TCSetAttr }
  TCSANOW   = 0;
  TCSADRAIN = 1;
  TCSAFLUSH = 2;

{ from Linux 4.0, include/uapi/asm-generic/termios.h }

  { c_line bits }
  TIOCM_LE   = $001;
  TIOCM_DTR  = $002;
  TIOCM_RTS  = $004;
  TIOCM_ST   = $008;
  TIOCM_SR   = $010;
  TIOCM_CTS  = $020;
  TIOCM_CAR  = $040;
  TIOCM_RNG  = $080;
  TIOCM_DSR  = $100;
  TIOCM_CD   = TIOCM_CAR;
  TIOCM_RI   = TIOCM_RNG;
  TIOCM_OUT1 = $2000;
  TIOCM_OUT2 = $4000;
  TIOCM_LOOP = $8000;

{$endif cpuriscv32}

{$ifdef cpuriscv64}

{ from Linux 4.0, include/uapi/asm-generic/ioctls.h }

  { For Terminal handling }
  TCGETS          = $5401;
  TCSETS          = $5402;
  TCSETSW         = $5403;
  TCSETSF         = $5404;
  TCGETA          = $5405;
  TCSETA          = $5406;
  TCSETAW         = $5407;
  TCSETAF         = $5408;
  TCSBRK          = $5409;
  TCXONC          = $540A;
  TCFLSH          = $540B;
  TIOCEXCL        = $540C;
  TIOCNXCL        = $540D;
  TIOCSCTTY       = $540E;
  TIOCGPGRP       = $540F;
  TIOCSPGRP       = $5410;
  TIOCOUTQ        = $5411;
  TIOCSTI         = $5412;
  TIOCGWINSZ      = $5413;
  TIOCSWINSZ      = $5414;
  TIOCMGET        = $5415;
  TIOCMBIS        = $5416;
  TIOCMBIC        = $5417;
  TIOCMSET        = $5418;
  TIOCGSOFTCAR    = $5419;
  TIOCSSOFTCAR    = $541A;
  FIONREAD        = $541B;
  TIOCINQ         = FIONREAD;
  TIOCLINUX       = $541C;
  TIOCCONS        = $541D;
  TIOCGSERIAL     = $541E;
  TIOCSSERIAL     = $541F;
  TIOCPKT         = $5420;
  FIONBIO         = $5421;
  TIOCNOTTY       = $5422;
  TIOCSETD        = $5423;
  TIOCGETD        = $5424;
  TCSBRKP         = $5425;

  TIOCSBRK        = $5427;
  TIOCCBRK        = $5428;
  TIOCGSID        = $5429;

  TIOCGRS485      = $542E;
  TIOCSRS485      = $542F;

  TCGETX          = $5432;
  TCSETX          = $5433;
  TCSETXF         = $5434;
  TCSETXW         = $5435;
  TIOCVHANGUP     = $5437;

  FIONCLEX        = $5450;
  FIOCLEX         = $5451;
  FIOASYNC        = $5452;
  TIOCSERCONFIG   = $5453;
  TIOCSERGWILD    = $5454;
  TIOCSERSWILD    = $5455;
  TIOCGLCKTRMIOS  = $5456;
  TIOCSLCKTRMIOS  = $5457;
  TIOCSERGSTRUCT  = $5458;
  TIOCSERGETLSR   = $5459;
  TIOCSERGETMULTI = $545A;
  TIOCSERSETMULTI = $545B;
  TIOCMIWAIT      = $545C;
  TIOCGICOUNT     = $545D;

  FIOQSIZE        = $5460;

  TIOCPKT_DATA       = 0;
  TIOCPKT_FLUSHREAD  = 1;
  TIOCPKT_FLUSHWRITE = 2;
  TIOCPKT_STOP       = 4;
  TIOCPKT_START      = 8;
  TIOCPKT_NOSTOP     = 16;
  TIOCPKT_DOSTOP     = 32;
  TIOCPKT_IOCTL      = 64;

  TIOCSER_TEMT       = $01;

{ from Linux 4.0, include/uapi/asm-generic/termbits.h }

  { c_cc characters }
  VINTR    = 0;
  VQUIT    = 1;
  VERASE   = 2;
  VKILL    = 3;
  VEOF     = 4;
  VTIME    = 5;
  VMIN     = 6;
  VSWTC    = 7;
  VSTART   = 8;
  VSTOP    = 9;
  VSUSP    = 10;
  VEOL     = 11;
  VREPRINT = 12;
  VDISCARD = 13;
  VWERASE  = 14;
  VLNEXT   = 15;
  VEOL2    = 16;

  { c_iflag bits }
  IGNBRK  = &0000001;
  BRKINT  = &0000002;
  IGNPAR  = &0000004;
  PARMRK  = &0000010;
  INPCK   = &0000020;
  ISTRIP  = &0000040;
  INLCR   = &0000100;
  IGNCR   = &0000200;
  ICRNL   = &0000400;
  IUCLC   = &0001000;
  IXON    = &0002000;
  IXANY   = &0004000;
  IXOFF   = &0010000;
  IMAXBEL = &0020000;
  IUTF8   = &0040000;

  { c_oflag bits }
  OPOST  = &0000001;
  OLCUC  = &0000002;
  ONLCR  = &0000004;
  OCRNL  = &0000010;
  ONOCR  = &0000020;
  ONLRET = &0000040;
  OFILL  = &0000100;
  OFDEL  = &0000200;
  NLDLY  = &0000400;
  NL0    = &0000000;
  NL1    = &0000400;
  CRDLY  = &0003000;
  CR0    = &0000000;
  CR1    = &0001000;
  CR2    = &0002000;
  CR3    = &0003000;
  TABDLY = &0014000;
  TAB0   = &0000000;
  TAB1   = &0004000;
  TAB2   = &0010000;
  TAB3   = &0014000;
  XTABS  = &0014000;
  BSDLY  = &0020000;
  BS0    = &0000000;
  BS1    = &0020000;
  VTDLY  = &0040000;
  VT0    = &0000000;
  VT1    = &0040000;
  FFDLY  = &0100000;
  FF0    = &0000000;
  FF1    = &0100000;

  { c_cflag bits }
  CBAUD    = &0010017;
  B0       = &0000000;
  B50      = &0000001;
  B75      = &0000002;
  B110     = &0000003;
  B134     = &0000004;
  B150     = &0000005;
  B200     = &0000006;
  B300     = &0000007;
  B600     = &0000010;
  B1200    = &0000011;
  B1800    = &0000012;
  B2400    = &0000013;
  B4800    = &0000014;
  B9600    = &0000015;
  B19200   = &0000016;
  B38400   = &0000017;
  EXTA     = B19200;
  EXTB     = B38400;
  CSIZE    = &0000060;
  CS5      = &0000000;
  CS6      = &0000020;
  CS7      = &0000040;
  CS8      = &0000060;
  CSTOPB   = &0000100;
  CREAD    = &0000200;
  PARENB   = &0000400;
  PARODD   = &0001000;
  HUPCL    = &0002000;
  CLOCAL   = &0004000;
  CBAUDEX  = &0010000;
  BOTHER   = &0010000;
  B57600   = &0010001;
  B115200  = &0010002;
  B230400  = &0010003;
  B460800  = &0010004;
  B500000  = &0010005;
  B576000  = &0010006;
  B921600  = &0010007;
  B1000000 = &0010010;
  B1152000 = &0010011;
  B1500000 = &0010012;
  B2000000 = &0010013;
  B2500000 = &0010014;
  B3000000 = &0010015;
  B3500000 = &0010016;
  B4000000 = &0010017;

  CIBAUD   = &002003600000;
  CMSPAR   = &010000000000;
  CRTSCTS  = &020000000000;

  IBSHIFT  = 16;

  { c_lflag bits }
  ISIG    = &0000001;
  ICANON  = &0000002;
  XCASE   = &0000004;
  ECHO    = &0000010;
  ECHOE   = &0000020;
  ECHOK   = &0000040;
  ECHONL  = &0000100;
  NOFLSH  = &0000200;
  TOSTOP  = &0000400;
  ECHOCTL = &0001000;
  ECHOPRT = &0002000;
  ECHOKE  = &0004000;
  FLUSHO  = &0010000;
  PENDIN  = &0040000;
  IEXTEN  = &0100000;
  EXTPROC = &0200000;

  { TCFlow }
  TCOOFF = 0;
  TCOON  = 1;
  TCIOFF = 2;
  TCION  = 3;

  { TCFlush }
  TCIFLUSH  = 0;
  TCOFLUSH  = 1;
  TCIOFLUSH = 2;

  { TCSetAttr }
  TCSANOW   = 0;
  TCSADRAIN = 1;
  TCSAFLUSH = 2;

{ from Linux 4.0, include/uapi/asm-generic/termios.h }

  { c_line bits }
  TIOCM_LE   = $001;
  TIOCM_DTR  = $002;
  TIOCM_RTS  = $004;
  TIOCM_ST   = $008;
  TIOCM_SR   = $010;
  TIOCM_CTS  = $020;
  TIOCM_CAR  = $040;
  TIOCM_RNG  = $080;
  TIOCM_DSR  = $100;
  TIOCM_CD   = TIOCM_CAR;
  TIOCM_RI   = TIOCM_RNG;
  TIOCM_OUT1 = $2000;
  TIOCM_OUT2 = $4000;
  TIOCM_LOOP = $8000;

{$endif cpuriscv64}

{$ifdef cpui386}
  { For Terminal handling }
  TCGETS          = $5401;
  TCSETS          = $5402;
  TCSETSW         = $5403;
  TCSETSF         = $5404;
  TCGETA          = $5405;
  TCSETA          = $5406;
  TCSETAW         = $5407;
  TCSETAF         = $5408;
  TCSBRK          = $5409;
  TCXONC          = $540A;
  TCFLSH          = $540B;
  TIOCEXCL        = $540C;
  TIOCNXCL        = $540D;
  TIOCSCTTY       = $540E;
  TIOCGPGRP       = $540F;
  TIOCSPGRP       = $5410;
  TIOCOUTQ        = $5411;
  TIOCSTI         = $5412;
  TIOCGWINSZ      = $5413;
  TIOCSWINSZ      = $5414;
  TIOCMGET        = $5415;
  TIOCMBIS        = $5416;
  TIOCMBIC        = $5417;
  TIOCMSET        = $5418;
  TIOCGSOFTCAR    = $5419;
  TIOCSSOFTCAR    = $541A;
  FIONREAD        = $541B;
  TIOCINQ         = FIONREAD;
  TIOCLINUX       = $541C;
  TIOCCONS        = $541D;
  TIOCGSERIAL     = $541E;
  TIOCSSERIAL     = $541F;
  TIOCPKT         = $5420;
  FIONBIO         = $5421;
  TIOCNOTTY       = $5422;
  TIOCSETD        = $5423;
  TIOCGETD        = $5424;
  TCSBRKP         = $5425;
  TIOCTTYGSTRUCT  = $5426;
  FIONCLEX        = $5450;
  FIOCLEX         = $5451;
  FIOASYNC        = $5452;
  TIOCSERCONFIG   = $5453;
  TIOCSERGWILD    = $5454;
  TIOCSERSWILD    = $5455;
  TIOCGLCKTRMIOS  = $5456;
  TIOCSLCKTRMIOS  = $5457;
  TIOCSERGSTRUCT  = $5458;
  TIOCSERGETLSR   = $5459;
  TIOCSERGETMULTI = $545A;
  TIOCSERSETMULTI = $545B;

  TIOCMIWAIT      = $545C;
  TIOCGICOUNT     = $545D;
  TIOCGHAYESESP   = $545E;
  TIOCSHAYESESP   = $545F;
  FIOQSIZE        = $5460;


  TIOCPKT_DATA       = 0;
  TIOCPKT_FLUSHREAD  = 1;
  TIOCPKT_FLUSHWRITE = 2;
  TIOCPKT_STOP       = 4;
  TIOCPKT_START      = 8;
  TIOCPKT_NOSTOP     = 16;
  TIOCPKT_DOSTOP     = 32;

{c_cc characters}
  VINTR    = 0;
  VQUIT    = 1;
  VERASE   = 2;
  VKILL    = 3;
  VEOF     = 4;
  VTIME    = 5;
  VMIN     = 6;
  VSWTC    = 7;
  VSTART   = 8;
  VSTOP    = 9;
  VSUSP    = 10;
  VEOL     = 11;
  VREPRINT = 12;
  VDISCARD = 13;
  VWERASE  = 14;
  VLNEXT   = 15;
  VEOL2    = 16;

{c_iflag bits}
   IGNBRK  = $0000001;
   BRKINT  = $0000002;
   IGNPAR  = $0000004;
   PARMRK  = $0000008;
   INPCK   = $0000010;
   ISTRIP  = $0000020;
   INLCR   = $0000040;
   IGNCR   = $0000080;
   ICRNL   = $0000100;
   IUCLC   = $0000200;
   IXON    = $0000400;
   IXANY   = $0000800;
   IXOFF   = $0001000;
   IMAXBEL = $0002000;
   IUTF8   = $0004000;

{c_oflag bits}
   OPOST  = $0000001;
   OLCUC  = $0000002;
   ONLCR  = $0000004;
   OCRNL  = $0000008;
   ONOCR  = $0000010;
   ONLRET = $0000020;
   OFILL  = $0000040;
   OFDEL  = $0000080;
   NLDLY  = $0000100;
     NL0  = $0000000;
     NL1  = $0000100;
   CRDLY  = $0000600;
     CR0  = $0000000;
     CR1  = $0000200;
     CR2  = $0000400;
     CR3  = $0000600;
   TABDLY = $0001800;
     TAB0 = $0000000;
     TAB1 = $0000800;
     TAB2 = $0001000;
     TAB3 = $0001800;
    XTABS = $0001800;
   BSDLY  = $0002000;
     BS0  = $0000000;
     BS1  = $0002000;
   VTDLY  = $0004000;
     VT0  = $0000000;
     VT1  = $0004000;
   FFDLY  = $0008000;
     FF0  = $0000000;
     FF1  = $0008000;

{c_cflag bits}
   CBAUD   = $000100F;
   B0      = $0000000;
   B50     = $0000001;
   B75     = $0000002;
   B110    = $0000003;
   B134    = $0000004;
   B150    = $0000005;
   B200    = $0000006;
   B300    = $0000007;
   B600    = $0000008;
   B1200   = $0000009;
   B1800   = $000000A;
   B2400   = $000000B;
   B4800   = $000000C;
   B9600   = $000000D;
   B19200  = $000000E;
   B38400  = $000000F;
   EXTA    = B19200;
   EXTB    = B38400;
   CSIZE   = $0000030;
     CS5   = $0000000;
     CS6   = $0000010;
     CS7   = $0000020;
     CS8   = $0000030;
   CSTOPB  = $0000040;
   CREAD   = $0000080;
   PARENB  = $0000100;
   PARODD  = $0000200;
   HUPCL   = $0000400;
   CLOCAL  = $0000800;
   CBAUDEX = $0001000;
   B57600  = $0001001;
   B115200 = $0001002;
   B230400 = $0001003;
   B460800 = $0001004;
   B500000 = $0001005;
   B576000 = $0001006;
   B921600 = $0001007;
   B1000000= $0001008;
   B1152000= $0001009;
   B1500000= $000100A;
   B2000000= $000100B;
   B2500000= $000100C;
   B3000000= $000100D;
   B3500000= $000100E;
   B4000000= $000100F;

   CIBAUD  = $100F0000;
   CMSPAR  = $40000000;
   CRTSCTS = $80000000;

{c_lflag bits}
   ISIG    = $0000001;
   ICANON  = $0000002;
   XCASE   = $0000004;
   ECHO    = $0000008;
   ECHOE   = $0000010;
   ECHOK   = $0000020;
   ECHONL  = $0000040;
   NOFLSH  = $0000080;
   TOSTOP  = $0000100;
   ECHOCTL = $0000200;
   ECHOPRT = $0000400;
   ECHOKE  = $0000800;
   FLUSHO  = $0001000;
   PENDIN  = $0004000;
   IEXTEN  = $0008000;

{c_line bits}
   TIOCM_LE   = $001;
   TIOCM_DTR  = $002;
   TIOCM_RTS  = $004;
   TIOCM_ST   = $008;
   TIOCM_SR   = $010;
   TIOCM_CTS  = $020;
   TIOCM_CAR  = $040;
   TIOCM_RNG  = $080;
   TIOCM_DSR  = $100;
   TIOCM_CD   = TIOCM_CAR;
   TIOCM_RI   = TIOCM_RNG;
   TIOCM_OUT1 = $2000;
   TIOCM_OUT2 = $4000;

{TCSetAttr}
   TCSANOW   = 0;
   TCSADRAIN = 1;
   TCSAFLUSH = 2;

{TCFlow}
   TCOOFF = 0;
   TCOON  = 1;
   TCIOFF = 2;
   TCION  = 3;

{TCFlush}
   TCIFLUSH  = 0;
   TCOFLUSH  = 1;
   TCIOFLUSH = 2;

{$endif cpui386}

{$if defined(cpusparc) or defined(cpusparc64)}
  TCGETA = $40125401;
  TCSETA = $80125402;
  TCSETAW = $80125403;
  TCSETAF = $80125404;
  TCSBRK = $20005405;
  TCXONC = $20005406;
  TCFLSH = $20005407;
  TCGETS = $40245408;
  TCSETS = $80245409;
  TCSETSW = $8024540a;
  TCSETSF = $8024540b;
  TIOCGETD = $40047400;
  TIOCSETD = $80047401;
  TIOCEXCL = $2000740d;
  TIOCNXCL = $2000740e;
  TIOCCONS = $20007424;
  TIOCGSOFTCAR = $40047464;
  TIOCSSOFTCAR = $80047465;
  TIOCSWINSZ = $80087467;
  TIOCGWINSZ = $40087468;
  TIOCMGET = $4004746a;
  TIOCMBIC = $8004746b;
  TIOCMBIS = $8004746c;
  TIOCMSET = $8004746d;
  TIOCSTART = $2000746e;
  TIOCSTOP = $2000746f;
  TIOCPKT = $80047470;
  TIOCNOTTY = $20007471;
  TIOCSTI = $80017472;
  TIOCOUTQ = $40047473;
  TIOCCBRK = $2000747a;
  TIOCSBRK = $2000747b;
  TIOCSPGRP = $80047482;
  TIOCGPGRP = $40047483;
  TIOCSCTTY = $20007484;
  TIOCGSID = $40047485;
  TIOCGPTN = $40047486;
  TIOCSPTLCK = $80047487;
  FIOCLEX = $20006601;
  FIONCLEX = $20006602;
  FIOASYNC = $8004667d;
  FIONBIO = $8004667e;
  FIONREAD = $4004667f;
  TIOCINQ = $4004667f;
  TIOCLINUX = $541c;
  TIOCGSERIAL = $541e;
  TIOCSSERIAL = $541f;
  TCSBRKP = $5425;
  TIOCTTYGSTRUCT = $5426;
  TIOCSERCONFIG = $5453;
  TIOCSERGWILD = $5454;
  TIOCSERSWILD = $5455;
  TIOCGLCKTRMIOS = $5456;
  TIOCSLCKTRMIOS = $5457;
  TIOCSERGSTRUCT = $5458;
  TIOCSERGETLSR = $5459;
  TIOCSERGETMULTI = $545a;
  TIOCSERSETMULTI = $545b;
  TIOCMIWAIT = $545c;
  TIOCGICOUNT = $545d;
  TIOCPKT_DATA = $0;
  TIOCPKT_FLUSHREAD = $1;
  TIOCPKT_FLUSHWRITE = $2;
  TIOCPKT_STOP = $4;
  TIOCPKT_START = $8;
  TIOCPKT_NOSTOP = $10;
  TIOCPKT_DOSTOP = $20;

{c_cc characters}
  VINTR = 0;
  VQUIT = 1;
  VERASE = 2;
  VKILL = 3;
  VEOF = 4;
  VEOL = 5;
  VEOL2 = 6;
  VSWTC = 7;
  VSTART = 8;
  VSTOP = 9;
  VSUSP = 10;
  VDSUSP = 11;
  VREPRINT = 12;
  VDISCARD = 13;
  VWERASE = 14;
  VLNEXT = 15;
  VMIN = 4;
  VTIME = 5;

{c_iflag bits}
   IGNBRK  = $0000001;
   BRKINT  = $0000002;
   IGNPAR  = $0000004;
   PARMRK  = $0000008;
   INPCK   = $0000010;
   ISTRIP  = $0000020;
   INLCR   = $0000040;
   IGNCR   = $0000080;
   ICRNL   = $0000100;
   IUCLC   = $0000200;
   IXON    = $0000400;
   IXANY   = $0000800;
   IXOFF   = $0001000;
   IMAXBEL = $0002000;

{c_oflag bits}
   OPOST  = $0000001;
   OLCUC  = $0000002;
   ONLCR  = $0000004;
   OCRNL  = $0000008;
   ONOCR  = $0000010;
   ONLRET = $0000020;
   OFILL  = $0000040;
   OFDEL  = $0000080;
   NLDLY  = $0000100;
     NL0  = $0000000;
     NL1  = $0000100;
   CRDLY  = $0000600;
     CR0  = $0000000;
     CR1  = $0000200;
     CR2  = $0000400;
     CR3  = $0000600;
   TABDLY = $0001800;
     TAB0 = $0000000;
     TAB1 = $0000800;
     TAB2 = $0001000;
     TAB3 = $0001800;
    XTABS = $0001800;
   BSDLY  = $0002000;
     BS0  = $0000000;
     BS1  = $0002000;
   VTDLY  = $0004000;
     VT0  = $0000000;
     VT1  = $0004000;
   FFDLY  = $0008000;
     FF0  = $0000000;
     FF1  = $0008000;

{c_cflag bits}
   CBAUD   = $000100F;
   B0      = $0000000;
   B50     = $0000001;
   B75     = $0000002;
   B110    = $0000003;
   B134    = $0000004;
   B150    = $0000005;
   B200    = $0000006;
   B300    = $0000007;
   B600    = $0000008;
   B1200   = $0000009;
   B1800   = $000000A;
   B2400   = $000000B;
   B4800   = $000000C;
   B9600   = $000000D;
   B19200  = $000000E;
   B38400  = $000000F;
   EXTA    = B19200;
   EXTB    = B38400;
   CSIZE   = $0000030;
     CS5   = $0000000;
     CS6   = $0000010;
     CS7   = $0000020;
     CS8   = $0000030;
   CSTOPB  = $0000040;
   CREAD   = $0000080;
   PARENB  = $0000100;
   PARODD  = $0000200;
   HUPCL   = $0000400;
   CLOCAL  = $0000800;
   CBAUDEX = $0001000;
   B57600  = $0001001;
   B115200 = $0001002;
   B230400 = $0001003;
   B460800 = $0001004;
   CIBAUD  = $100F0000;
   CMSPAR  = $40000000;
   CRTSCTS = $80000000;

{c_lflag bits}
   ISIG    = $0000001;
   ICANON  = $0000002;
   XCASE   = $0000004;
   ECHO    = $0000008;
   ECHOE   = $0000010;
   ECHOK   = $0000020;
   ECHONL  = $0000040;
   NOFLSH  = $0000080;
   TOSTOP  = $0000100;
   ECHOCTL = $0000200;
   ECHOPRT = $0000400;
   ECHOKE  = $0000800;
   FLUSHO  = $0001000;
   PENDIN  = $0004000;
   IEXTEN  = $0008000;

{c_line bits}
   TIOCM_LE   = $001;
   TIOCM_DTR  = $002;
   TIOCM_RTS  = $004;
   TIOCM_ST   = $008;
   TIOCM_SR   = $010;
   TIOCM_CTS  = $020;
   TIOCM_CAR  = $040;
   TIOCM_RNG  = $080;
   TIOCM_DSR  = $100;
   TIOCM_CD   = TIOCM_CAR;
   TIOCM_RI   = TIOCM_RNG;
   TIOCM_OUT1 = $2000;
   TIOCM_OUT2 = $4000;

{TCSetAttr}
   TCSANOW   = 0;
   TCSADRAIN = 1;
   TCSAFLUSH = 2;

{TCFlow}
   TCOOFF = 0;
   TCOON  = 1;
   TCIOFF = 2;
   TCION  = 3;

{TCFlush}
   TCIFLUSH  = 0;
   TCOFLUSH  = 1;
   TCIOFLUSH = 2;
{$endif  defined(cpusparc) or defined(cpusparc64)}

{$ifdef cpux86_64}
  TCGETS =              $5401;
  TCSETS =              $5402;
  TCSETSW =             $5403;
  TCSETSF =             $5404;
  TCGETA =              $5405;
  TCSETA =              $5406;
  TCSETAW =             $5407;
  TCSETAF =             $5408;
  TCSBRK =              $5409;
  TCXONC =              $540A;
  TCFLSH =              $540B;
  TIOCEXCL =    $540C;
  TIOCNXCL =    $540D;
  TIOCSCTTY =   $540E;
  TIOCGPGRP =   $540F;
  TIOCSPGRP =   $5410;
  TIOCOUTQ =    $5411;
  TIOCSTI =             $5412;
  TIOCGWINSZ =  $5413;
  TIOCSWINSZ =  $5414;
  TIOCMGET =    $5415;
  TIOCMBIS =    $5416;
  TIOCMBIC =    $5417;
  TIOCMSET =    $5418;
  TIOCGSOFTCAR =        $5419;
  TIOCSSOFTCAR =        $541A;
  FIONREAD =    $541B;
  TIOCINQ =             FIONREAD;
  TIOCLINUX =   $541C;
  TIOCCONS =    $541D;
  TIOCGSERIAL = $541E;
  TIOCSSERIAL = $541F;
  TIOCPKT =             $5420;
  FIONBIO =             $5421;
  TIOCNOTTY =   $5422;
  TIOCSETD =    $5423;
  TIOCGETD =    $5424;
  TCSBRKP =             $5425;
  TIOCSBRK =    $5427;
  TIOCCBRK =    $5428;
  TIOCGSID =    $5429;
  FIONCLEX =    $5450;
  FIOCLEX =             $5451;
  FIOASYNC =    $5452;
  TIOCSERCONFIG =       $5453;
  TIOCSERGWILD =        $5454;
  TIOCSERSWILD =        $5455;
  TIOCGLCKTRMIOS =      $5456;
  TIOCSLCKTRMIOS =      $5457;
  TIOCSERGSTRUCT =      $5458;
  TIOCSERGETLSR =   $5459;
  TIOCSERGETMULTI = $545A;
  TIOCSERSETMULTI = $545B;
  TIOCMIWAIT =  $545C;
  TIOCGICOUNT = $545D;
  TIOCGHAYESESP =   $545E;
  TIOCSHAYESESP =   $545F;
  FIOQSIZE =       $5460;
  TIOCPKT_DATA =                 0;
  TIOCPKT_FLUSHREAD =    1;
  TIOCPKT_FLUSHWRITE =   2;
  TIOCPKT_STOP =                 4;
  TIOCPKT_START =                8;
  TIOCPKT_NOSTOP =              16;
  TIOCPKT_DOSTOP =              32;
  TIOCSER_TEMT =    $01;

{c_cc characters}
  VINTR    = 0;
  VQUIT    = 1;
  VERASE   = 2;
  VKILL    = 3;
  VEOF     = 4;
  VTIME    = 5;
  VMIN     = 6;
  VSWTC    = 7;
  VSTART   = 8;
  VSTOP    = 9;
  VSUSP    = 10;
  VEOL     = 11;
  VREPRINT = 12;
  VDISCARD = 13;
  VWERASE  = 14;
  VLNEXT   = 15;
  VEOL2    = 16;

{c_iflag bits}
   IGNBRK  = $0000001;
   BRKINT  = $0000002;
   IGNPAR  = $0000004;
   PARMRK  = $0000008;
   INPCK   = $0000010;
   ISTRIP  = $0000020;
   INLCR   = $0000040;
   IGNCR   = $0000080;
   ICRNL   = $0000100;
   IUCLC   = $0000200;
   IXON    = $0000400;
   IXANY   = $0000800;
   IXOFF   = $0001000;
   IMAXBEL = $0002000;

{c_oflag bits}
   OPOST  = $0000001;
   OLCUC  = $0000002;
   ONLCR  = $0000004;
   OCRNL  = $0000008;
   ONOCR  = $0000010;
   ONLRET = $0000020;
   OFILL  = $0000040;
   OFDEL  = $0000080;
   NLDLY  = $0000100;
     NL0  = $0000000;
     NL1  = $0000100;
   CRDLY  = $0000600;
     CR0  = $0000000;
     CR1  = $0000200;
     CR2  = $0000400;
     CR3  = $0000600;
   TABDLY = $0001800;
     TAB0 = $0000000;
     TAB1 = $0000800;
     TAB2 = $0001000;
     TAB3 = $0001800;
    XTABS = $0001800;
   BSDLY  = $0002000;
     BS0  = $0000000;
     BS1  = $0002000;
   VTDLY  = $0004000;
     VT0  = $0000000;
     VT1  = $0004000;
   FFDLY  = $0008000;
     FF0  = $0000000;
     FF1  = $0008000;

{c_cflag bits}
   CBAUD   = $000100F;
   B0      = $0000000;
   B50     = $0000001;
   B75     = $0000002;
   B110    = $0000003;
   B134    = $0000004;
   B150    = $0000005;
   B200    = $0000006;
   B300    = $0000007;
   B600    = $0000008;
   B1200   = $0000009;
   B1800   = $000000A;
   B2400   = $000000B;
   B4800   = $000000C;
   B9600   = $000000D;
   B19200  = $000000E;
   B38400  = $000000F;
   EXTA    = B19200;
   EXTB    = B38400;
   CSIZE   = $0000030;
     CS5   = $0000000;
     CS6   = $0000010;
     CS7   = $0000020;
     CS8   = $0000030;
   CSTOPB  = $0000040;
   CREAD   = $0000080;
   PARENB  = $0000100;
   PARODD  = $0000200;
   HUPCL   = $0000400;
   CLOCAL  = $0000800;
   CBAUDEX = $0001000;
   B57600  = $0001001;
   B115200 = $0001002;
   B230400 = $0001003;
   B460800 = $0001004;
   CIBAUD  = $100F0000;
   CMSPAR  = $40000000;
   CRTSCTS = $80000000;
   B500000 = $0001005;
   B576000 = $0001006;
   B921600 = $0001007;
   B1000000= $0001008;
   B1152000= $0001009;
   B1500000= $000100A;
   B2000000= $000100B;
   B2500000= $000100C;
   B3000000= $000100D;
   B3500000= $000100E;
   B4000000= $000100F;
   __MAX_BAUD=  B4000000;

{c_lflag bits}
   ISIG    = $0000001;
   ICANON  = $0000002;
   XCASE   = $0000004;
   ECHO    = $0000008;
   ECHOE   = $0000010;
   ECHOK   = $0000020;
   ECHONL  = $0000040;
   NOFLSH  = $0000080;
   TOSTOP  = $0000100;
   ECHOCTL = $0000200;
   ECHOPRT = $0000400;
   ECHOKE  = $0000800;
   FLUSHO  = $0001000;
   PENDIN  = $0004000;
   IEXTEN  = $0008000;

{c_line bits}
   TIOCM_LE   = $001;
   TIOCM_DTR  = $002;
   TIOCM_RTS  = $004;
   TIOCM_ST   = $008;
   TIOCM_SR   = $010;
   TIOCM_CTS  = $020;
   TIOCM_CAR  = $040;
   TIOCM_RNG  = $080;
   TIOCM_DSR  = $100;
   TIOCM_CD   = TIOCM_CAR;
   TIOCM_RI   = TIOCM_RNG;
   TIOCM_OUT1 = $2000;
   TIOCM_OUT2 = $4000;

{TCSetAttr}
   TCSANOW   = 0;
   TCSADRAIN = 1;
   TCSAFLUSH = 2;

{TCFlow}
   TCOOFF = 0;
   TCOON  = 1;
   TCIOFF = 2;
   TCION  = 3;

{TCFlush}
   TCIFLUSH  = 0;
   TCOFLUSH  = 1;
   TCIOFLUSH = 2;

{$endif cpux86_64}

{$ifdef cpuarm}
  { For Terminal handling }
  TCGETS          = $5401;
  TCSETS          = $5402;
  TCSETSW         = $5403;
  TCSETSF         = $5404;
  TCGETA          = $5405;
  TCSETA          = $5406;
  TCSETAW         = $5407;
  TCSETAF         = $5408;
  TCSBRK          = $5409;
  TCXONC          = $540A;
  TCFLSH          = $540B;
  TIOCEXCL        = $540C;
  TIOCNXCL        = $540D;
  TIOCSCTTY       = $540E;
  TIOCGPGRP       = $540F;
  TIOCSPGRP       = $5410;
  TIOCOUTQ        = $5411;
  TIOCSTI         = $5412;
  TIOCGWINSZ      = $5413;
  TIOCSWINSZ      = $5414;
  TIOCMGET        = $5415;
  TIOCMBIS        = $5416;
  TIOCMBIC        = $5417;
  TIOCMSET        = $5418;
  TIOCGSOFTCAR    = $5419;
  TIOCSSOFTCAR    = $541A;
  FIONREAD        = $541B;
  TIOCINQ         = FIONREAD;
  TIOCLINUX       = $541C;
  TIOCCONS        = $541D;
  TIOCGSERIAL     = $541E;
  TIOCSSERIAL     = $541F;
  TIOCPKT         = $5420;
  FIONBIO         = $5421;
  TIOCNOTTY       = $5422;
  TIOCSETD        = $5423;
  TIOCGETD        = $5424;
  TCSBRKP         = $5425;
  TIOCTTYGSTRUCT  = $5426;
  FIONCLEX        = $5450;
  FIOCLEX         = $5451;
  FIOASYNC        = $5452;
  TIOCSERCONFIG   = $5453;
  TIOCSERGWILD    = $5454;
  TIOCSERSWILD    = $5455;
  TIOCGLCKTRMIOS  = $5456;
  TIOCSLCKTRMIOS  = $5457;
  TIOCSERGSTRUCT  = $5458;
  TIOCSERGETLSR   = $5459;
  TIOCSERGETMULTI = $545A;
  TIOCSERSETMULTI = $545B;

  TIOCMIWAIT      = $545C;
  TIOCGICOUNT     = $545D;
  FIOQSIZE        = $545E;

  TIOCPKT_DATA       = 0;
  TIOCPKT_FLUSHREAD  = 1;
  TIOCPKT_FLUSHWRITE = 2;
  TIOCPKT_STOP       = 4;
  TIOCPKT_START      = 8;
  TIOCPKT_NOSTOP     = 16;
  TIOCPKT_DOSTOP     = 32;

{c_cc characters}
  VINTR    = 0;
  VQUIT    = 1;
  VERASE   = 2;
  VKILL    = 3;
  VEOF     = 4;
  VTIME    = 5;
  VMIN     = 6;
  VSWTC    = 7;
  VSTART   = 8;
  VSTOP    = 9;
  VSUSP    = 10;
  VEOL     = 11;
  VREPRINT = 12;
  VDISCARD = 13;
  VWERASE  = 14;
  VLNEXT   = 15;
  VEOL2    = 16;

{c_iflag bits}
   IGNBRK  = $0000001;
   BRKINT  = $0000002;
   IGNPAR  = $0000004;
   PARMRK  = $0000008;
   INPCK   = $0000010;
   ISTRIP  = $0000020;
   INLCR   = $0000040;
   IGNCR   = $0000080;
   ICRNL   = $0000100;
   IUCLC   = $0000200;
   IXON    = $0000400;
   IXANY   = $0000800;
   IXOFF   = $0001000;
   IMAXBEL = $0002000;

{c_oflag bits}
   OPOST  = $0000001;
   OLCUC  = $0000002;
   ONLCR  = $0000004;
   OCRNL  = $0000008;
   ONOCR  = $0000010;
   ONLRET = $0000020;
   OFILL  = $0000040;
   OFDEL  = $0000080;
   NLDLY  = $0000100;
     NL0  = $0000000;
     NL1  = $0000100;
   CRDLY  = $0000600;
     CR0  = $0000000;
     CR1  = $0000200;
     CR2  = $0000400;
     CR3  = $0000600;
   TABDLY = $0001800;
     TAB0 = $0000000;
     TAB1 = $0000800;
     TAB2 = $0001000;
     TAB3 = $0001800;
    XTABS = $0001800;
   BSDLY  = $0002000;
     BS0  = $0000000;
     BS1  = $0002000;
   VTDLY  = $0004000;
     VT0  = $0000000;
     VT1  = $0004000;
   FFDLY  = $0008000;
     FF0  = $0000000;
     FF1  = $0008000;

{c_cflag bits}
   CBAUD   = $000100F;
   B0      = $0000000;
   B50     = $0000001;
   B75     = $0000002;
   B110    = $0000003;
   B134    = $0000004;
   B150    = $0000005;
   B200    = $0000006;
   B300    = $0000007;
   B600    = $0000008;
   B1200   = $0000009;
   B1800   = $000000A;
   B2400   = $000000B;
   B4800   = $000000C;
   B9600   = $000000D;
   B19200  = $000000E;
   B38400  = $000000F;
   EXTA    = B19200;
   EXTB    = B38400;
   CSIZE   = $0000030;
     CS5   = $0000000;
     CS6   = $0000010;
     CS7   = $0000020;
     CS8   = $0000030;
   CSTOPB  = $0000040;
   CREAD   = $0000080;
   PARENB  = $0000100;
   PARODD  = $0000200;
   HUPCL   = $0000400;
   CLOCAL  = $0000800;
   CBAUDEX = $0001000;
   B57600  = $0001001;
   B115200 = $0001002;
   B230400 = $0001003;
   B460800 = $0001004;

   CIBAUD  = $100F0000;
   CMSPAR  = $40000000;
   CRTSCTS = $80000000;

{c_lflag bits}
   ISIG    = $0000001;
   ICANON  = $0000002;
   XCASE   = $0000004;
   ECHO    = $0000008;
   ECHOE   = $0000010;
   ECHOK   = $0000020;
   ECHONL  = $0000040;
   NOFLSH  = $0000080;
   TOSTOP  = $0000100;
   ECHOCTL = $0000200;
   ECHOPRT = $0000400;
   ECHOKE  = $0000800;
   FLUSHO  = $0001000;
   PENDIN  = $0004000;
   IEXTEN  = $0008000;

{c_line bits}
   TIOCM_LE   = $001;
   TIOCM_DTR  = $002;
   TIOCM_RTS  = $004;
   TIOCM_ST   = $008;
   TIOCM_SR   = $010;
   TIOCM_CTS  = $020;
   TIOCM_CAR  = $040;
   TIOCM_RNG  = $080;
   TIOCM_DSR  = $100;
   TIOCM_CD   = TIOCM_CAR;
   TIOCM_RI   = TIOCM_RNG;
   TIOCM_OUT1 = $2000;
   TIOCM_OUT2 = $4000;

{TCSetAttr}
   TCSANOW   = 0;
   TCSADRAIN = 1;
   TCSAFLUSH = 2;

{TCFlow}
   TCOOFF = 0;
   TCOON  = 1;
   TCIOFF = 2;
   TCION  = 3;

{TCFlush}
   TCIFLUSH  = 0;
   TCOFLUSH  = 1;
   TCIOFLUSH = 2;

{$endif cpuarm}


{$if defined(cpumips) or defined(cpumipsel)}
const
  TCGETA = $5401;
  TCSETA = $5402;
  TCSETAW = $5403;
  TCSETAF = $5404;
  TCSBRK = $5405;
  TCXONC = $5406;
  TCFLSH = $5407;
  TCGETS = $540d;
  TCSETS = $540e;
  TCSETSW = $540f;
  TCSETSF = $5410;

  TIOCEXCL = $740d;
  TIOCNXCL = $740e;
  TIOCOUTQ = $7472;
  TIOCSTI = $5472;
  TIOCMGET = $741d;
  TIOCMBIS = $741b;
  TIOCMBIC = $741c;
  TIOCMSET = $741a;
  TIOCPKT = $5470;
  TIOCPKT_DATA = $00;
  TIOCPKT_FLUSHREAD = $01;
  TIOCPKT_FLUSHWRITE = $02;
  TIOCPKT_STOP = $04;
  TIOCPKT_START = $08;
  TIOCPKT_NOSTOP = $10;
  TIOCPKT_DOSTOP = $20;
  TIOCPKT_IOCTL = $40;
  {Values checked: TIOCGWINSZ and TIOCSWINSZ on a real mips system }
  TIOCGWINSZ = $40087468;
  TIOCSWINSZ = $80087467;

  TIOCNOTTY = $5471;
  TIOCSETD = $7401;
  TIOCGETD = $7400;
  FIOCLEX = $6601;
  FIONCLEX = $6602;
  FIOASYNC = $667d;
  FIONBIO = $667e;
  FIOQSIZE = $667f;

  TIOCGLTC = $7474;
  TIOCSLTC = $7475;
  {Values checked: TIOCGPGRP and TIOCSPGRP on a real mips system }
  TIOCGPGRP = $40047477;
  TIOCSPGRP = $80047476;

  FIONREAD = $467f;
  TIOCINQ = FIONREAD;
  TIOCGETP = $7408;
  TIOCSETP = $7409;
  TIOCSETN = $740a;
  TIOCSBRK = $5427;
  TIOCCBRK = $5428;
  TIOCGSID = $7416;
  TIOCVHANGUP = $5437;
  TIOCSCTTY = $5480;
  TIOCGSOFTCAR = $5481;
  TIOCSSOFTCAR = $5482;
  TIOCLINUX = $5483;
  TIOCGSERIAL = $5484;
  TIOCSSERIAL = $5485;
  TCSBRKP = $5486;
  TIOCSERCONFIG = $5488;
  TIOCSERGWILD = $5489;
  TIOCSERSWILD = $548a;
  TIOCGLCKTRMIOS = $548b;
  TIOCSLCKTRMIOS = $548c;
  TIOCSERGSTRUCT = $548d;
  TIOCSERGETLSR = $548e;
  TIOCSERGETMULTI = $548f;
  TIOCSERSETMULTI = $5490;
  TIOCMIWAIT = $5491;
  TIOCGICOUNT = $5492;

  VINTR = 0;
  VQUIT = 1;
  VERASE = 2;
  VKILL = 3;
  VMIN = 4;
  VTIME = 5;
  VEOL2 = 6;
  VSWTC = 7;
  VSWTCH = VSWTC;
  VSTART = 8;
  VSTOP = 9;
  VSUSP = 10;

  VREPRINT = 12;
  VDISCARD = 13;
  VWERASE = 14;
  VLNEXT = 15;
  VEOF = 16;
  VEOL = 17;

  IGNBRK = &0000001;
  BRKINT = &0000002;
  IGNPAR = &0000004;
  PARMRK = &0000010;
  INPCK = &0000020;
  ISTRIP = &0000040;
  INLCR = &0000100;
  IGNCR = &0000200;
  ICRNL = &0000400;
  IUCLC = &0001000;
  IXON = &0002000;
  IXANY = &0004000;
  IXOFF = &0010000;
  IMAXBEL = &0020000;
  IUTF8 = &0040000;

  OPOST = &0000001;
  OLCUC = &0000002;

  ONLCR = &0000004;
  OCRNL = &0000010;
  ONOCR = &0000020;
  ONLRET = &0000040;
  OFILL = &0000100;
  OFDEL = &0000200;
  NLDLY = &0000400;
  NL0 = &0000000;
  NL1 = &0000400;
  CRDLY = &0003000;
  CR0 = &0000000;
  CR1 = &0001000;
  CR2 = &0002000;
  CR3 = &0003000;
  TABDLY = &0014000;
  TAB0 = &0000000;
  TAB1 = &0004000;
  TAB2 = &0010000;
  TAB3 = &0014000;
  XTABS = &0014000;
  BSDLY = &0020000;
  BS0 = &0000000;
  BS1 = &0020000;
  VTDLY = &0040000;
  VT0 = &0000000;
  VT1 = &0040000;
  FFDLY = &0100000;
  FF0 = &0000000;
  FF1 = &0100000;


  CBAUD = &0010017;

  B0 = &0000000;
  B50 = &0000001;
  B75 = &0000002;
  B110 = &0000003;
  B134 = &0000004;
  B150 = &0000005;
  B200 = &0000006;
  B300 = &0000007;
  B600 = &0000010;
  B1200 = &0000011;
  B1800 = &0000012;
  B2400 = &0000013;
  B4800 = &0000014;
  B9600 = &0000015;
  B19200 = &0000016;
  B38400 = &0000017;
  EXTA = $B19200;
  EXTB = $B38400;

  CSIZE = &0000060;
  CS5 = &0000000;
  CS6 = &0000020;
  CS7 = &0000040;
  CS8 = &0000060;
  CSTOPB = &0000100;
  CREAD = &0000200;
  PARENB = &0000400;
  PARODD = &0001000;
  HUPCL = &0002000;
  CLOCAL = &0004000;
  CBAUDEX = &0010000;
  BOTHER = &0010000;
  B57600 = &0010001;
  B115200 = &0010002;
  B230400 = &0010003;
  B460800 = &0010004;
  B500000 = &0010005;
  B576000 = &0010006;
  B921600 = &0010007;
  B1000000 = &0010010;
  B1152000 = &0010011;
  B1500000 = &0010012;
  B2000000 = &0010013;
  B2500000 = &0010014;
  B3000000 = &0010015;
  B3500000 = &0010016;
  B4000000 = &0010017;

  CIBAUD = &002003600000;
  CMSPAR = &010000000000;
  CRTSCTS = &020000000000;
  IBSHIFT = 16;

  ISIG = &0000001;
  ICANON = &0000002;
  XCASE = &0000004;
  ECHO = &0000010;
  ECHOE = &0000020;
  ECHOK = &0000040;
  ECHONL = &0000100;
  NOFLSH = &0000200;
  IEXTEN = &0000400;
  ECHOCTL = &0001000;
  ECHOPRT = &0002000;
  ECHOKE = &0004000;
  FLUSHO = &0020000;
  PENDIN = &0040000;
  TOSTOP = &0100000;
  ITOSTOP = TOSTOP;
  EXTPROC = &0200000;
  TIOCSER_TEMT = $01;

  TCOOFF = 0;
  TCOON = 1;
  TCIOFF = 2;
  TCION = 3;

  TCIFLUSH = 0;
  TCOFLUSH = 1;
  TCIOFLUSH = 2;

  TIOCM_LE   = $001;
  TIOCM_DTR  = $002;
  TIOCM_RTS  = $004;
  TIOCM_ST   = $010;
  TIOCM_SR   = $020;
  TIOCM_CTS  = $040;
  TIOCM_CAR  = $100;
  TIOCM_RNG  = $200;
  TIOCM_DSR  = $400;
  TIOCM_CD   = TIOCM_CAR;
  TIOCM_RI   = TIOCM_RNG;
  TIOCM_OUT1 = $2000;
  TIOCM_OUT2 = $4000;
  TIOCM_LOOP = $8000;

  TCSANOW = TCSETS;
  TCSADRAIN = TCSETSW;
  TCSAFLUSH = TCSETSF;
{$endif CPUMIPS}

{$ifdef CPUM68K}

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
  TIOCSBRK = $5427;
  TIOCCBRK = $5428;
  TIOCGSID = $5429;
  TIOCGRS485 = $542E;
  TIOCSRS485 = $542F;
  TCGETX = $5432;
  TCSETX = $5433;
  TCSETXF = $5434;
  TCSETXW = $5435;
  TIOCVHANGUP = $5437;
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

  { c_cc characters  }
  VINTR = 0;
  VQUIT = 1;
  VERASE = 2;
  VKILL = 3;
  VEOF = 4;
  VTIME = 5;
  VMIN = 6;
  VSWTC = 7;
  VSTART = 8;
  VSTOP = 9;
  VSUSP = 10;
  VEOL = 11;
  VREPRINT = 12;
  VDISCARD = 13;
  VWERASE = 14;
  VLNEXT = 15;
  VEOL2 = 16;

  { c_iflag bits  }
  IGNBRK = &000001;
  BRKINT = &000002;
  IGNPAR = &000004;
  PARMRK = &000010;
  INPCK = &000020;
  ISTRIP = &000040;
  INLCR = &000100;
  IGNCR = &000200;
  ICRNL = &000400;
  IUCLC = &001000;
  IXON = &002000;
  IXANY = &004000;
  IXOFF = &010000;
  IMAXBEL = &020000;
  IUTF8 = &040000;

  { c_oflag bits  }
  OPOST = &000001;
  OLCUC = &000002;
  ONLCR = &000004;
  OCRNL = &000010;
  ONOCR = &000020;
  ONLRET = &000040;
  OFILL = &000100;
  OFDEL = &000200;
  NLDLY = &000400;
  NL0 = &000000;
  NL1 = &000400;
  CRDLY = &003000;
  CR0 = &000000;
  CR1 = &001000;
  CR2 = &002000;
  CR3 = &003000;
  TABDLY = &014000;
  TAB0 = &000000;
  TAB1 = &004000;
  TAB2 = &010000;
  TAB3 = &014000;
  XTABS = &014000;
  BSDLY = &020000;
  BS0 = &000000;
  BS1 = &020000;
  VTDLY = &040000;
  VT0 = &000000;
  VT1 = &040000;
  FFDLY = &100000;
  FF0 = &000000;
  FF1 = &100000;

  { c_cflag bit meaning  }
  CBAUD = &010017;

  { hang up  }
  B0 = &000000;
  B50 = &000001;
  B75 = &000002;
  B110 = &000003;
  B134 = &000004;
  B150 = &000005;
  B200 = &000006;
  B300 = &000007;
  B600 = &000010;
  B1200 = &000011;
  B1800 = &000012;
  B2400 = &000013;
  B4800 = &000014;
  B9600 = &000015;
  B19200 = &000016;
  B38400 = &000017;
  EXTA = B19200;
  EXTB = B38400;
  CSIZE = &000060;
  CS5 = &000000;
  CS6 = &000020;
  CS7 = &000040;
  CS8 = &000060;
  CSTOPB = &000100;
  CREAD = &000200;
  PARENB = &000400;
  PARODD = &001000;
  HUPCL = &002000;
  CLOCAL = &004000;
  CBAUDEX = &010000;
  BOTHER = &010000;
  B57600 = &010001;
  B115200 = &010002;
  B230400 = &010003;
  B460800 = &010004;
  B500000 = &010005;
  B576000 = &010006;
  B921600 = &010007;
  B1000000 = &010010;
  B1152000 = &010011;
  B1500000 = &010012;
  B2000000 = &010013;
  B2500000 = &010014;
  B3000000 = &010015;
  B3500000 = &010016;
  B4000000 = &010017;
  CIBAUD = &02003600000;
  CMSPAR = &10000000000;
  CRTSCTS = &20000000000;
  IBSHIFT = 16;

  { c_lflag bits  }
  ISIG = &000001;
  ICANON = &000002;
  XCASE = &000004;
  ECHO = &000010;
  ECHOE = &000020;
  ECHOK = &000040;
  ECHONL = &000100;
  NOFLSH = &000200;
  TOSTOP = &000400;
  ECHOCTL = &001000;
  ECHOPRT = &002000;
  ECHOKE = &004000;
  FLUSHO = &010000;
  PENDIN = &040000;
  IEXTEN = &100000;
  EXTPROC = &200000;

  TCOOFF = 0;
  TCOON = 1;
  TCIOFF = 2;
  TCION = 3;

  TCIFLUSH = 0;
  TCOFLUSH = 1;
  TCIOFLUSH = 2;

  TCSANOW = 0;
  TCSADRAIN = 1;
  TCSAFLUSH = 2;

  { modem lines }
  TIOCM_LE = $001;
  TIOCM_DTR = $002;
  TIOCM_RTS = $004;
  TIOCM_ST = $008;
  TIOCM_SR = $010;
  TIOCM_CTS = $020;
  TIOCM_CAR = $040;
  TIOCM_RNG = $080;
  TIOCM_DSR = $100;
  TIOCM_CD = TIOCM_CAR;
  TIOCM_RI = TIOCM_RNG;
  TIOCM_OUT1 = $2000;
  TIOCM_OUT2 = $4000;
  TIOCM_LOOP = $8000;

{$endif CPUM68K}


{$ifdef cpuxtensa}
  { For Terminal handling }
  TCGETS          = $5401;
  TCSETS          = $5402;
  TCSETSW         = $5403;
  TCSETSF         = $5404;
  TCGETA          = $5405;
  TCSETA          = $5406;
  TCSETAW         = $5407;
  TCSETAF         = $5408;
  TCSBRK          = $5409;
  TCXONC          = $540A;
  TCFLSH          = $540B;
  TIOCEXCL        = $540C;
  TIOCNXCL        = $540D;
  TIOCSCTTY       = $540E;
  TIOCGPGRP       = $540F;
  TIOCSPGRP       = $5410;
  TIOCOUTQ        = $5411;
  TIOCSTI         = $5412;
  TIOCGWINSZ      = $5413;
  TIOCSWINSZ      = $5414;
  TIOCMGET        = $5415;
  TIOCMBIS        = $5416;
  TIOCMBIC        = $5417;
  TIOCMSET        = $5418;
  TIOCGSOFTCAR    = $5419;
  TIOCSSOFTCAR    = $541A;
  FIONREAD        = $541B;
  TIOCINQ         = FIONREAD;
  TIOCLINUX       = $541C;
  TIOCCONS        = $541D;
  TIOCGSERIAL     = $541E;
  TIOCSSERIAL     = $541F;
  TIOCPKT         = $5420;
  FIONBIO         = $5421;
  TIOCNOTTY       = $5422;
  TIOCSETD        = $5423;
  TIOCGETD        = $5424;
  TCSBRKP         = $5425;
  TIOCTTYGSTRUCT  = $5426;
  FIONCLEX        = $5450;
  FIOCLEX         = $5451;
  FIOASYNC        = $5452;
  TIOCSERCONFIG   = $5453;
  TIOCSERGWILD    = $5454;
  TIOCSERSWILD    = $5455;
  TIOCGLCKTRMIOS  = $5456;
  TIOCSLCKTRMIOS  = $5457;
  TIOCSERGSTRUCT  = $5458;
  TIOCSERGETLSR   = $5459;
  TIOCSERGETMULTI = $545A;
  TIOCSERSETMULTI = $545B;

  TIOCMIWAIT      = $545C;
  TIOCGICOUNT     = $545D;
  TIOCGHAYESESP   = $545E;
  TIOCSHAYESESP   = $545F;
  FIOQSIZE        = $5460;


  TIOCPKT_DATA       = 0;
  TIOCPKT_FLUSHREAD  = 1;
  TIOCPKT_FLUSHWRITE = 2;
  TIOCPKT_STOP       = 4;
  TIOCPKT_START      = 8;
  TIOCPKT_NOSTOP     = 16;
  TIOCPKT_DOSTOP     = 32;

{c_cc characters}
  VINTR    = 0;
  VQUIT    = 1;
  VERASE   = 2;
  VKILL    = 3;
  VEOF     = 4;
  VTIME    = 5;
  VMIN     = 6;
  VSWTC    = 7;
  VSTART   = 8;
  VSTOP    = 9;
  VSUSP    = 10;
  VEOL     = 11;
  VREPRINT = 12;
  VDISCARD = 13;
  VWERASE  = 14;
  VLNEXT   = 15;
  VEOL2    = 16;

{c_iflag bits}
   IGNBRK  = $0000001;
   BRKINT  = $0000002;
   IGNPAR  = $0000004;
   PARMRK  = $0000008;
   INPCK   = $0000010;
   ISTRIP  = $0000020;
   INLCR   = $0000040;
   IGNCR   = $0000080;
   ICRNL   = $0000100;
   IUCLC   = $0000200;
   IXON    = $0000400;
   IXANY   = $0000800;
   IXOFF   = $0001000;
   IMAXBEL = $0002000;
   IUTF8   = $0004000;

{c_oflag bits}
   OPOST  = $0000001;
   OLCUC  = $0000002;
   ONLCR  = $0000004;
   OCRNL  = $0000008;
   ONOCR  = $0000010;
   ONLRET = $0000020;
   OFILL  = $0000040;
   OFDEL  = $0000080;
   NLDLY  = $0000100;
     NL0  = $0000000;
     NL1  = $0000100;
   CRDLY  = $0000600;
     CR0  = $0000000;
     CR1  = $0000200;
     CR2  = $0000400;
     CR3  = $0000600;
   TABDLY = $0001800;
     TAB0 = $0000000;
     TAB1 = $0000800;
     TAB2 = $0001000;
     TAB3 = $0001800;
    XTABS = $0001800;
   BSDLY  = $0002000;
     BS0  = $0000000;
     BS1  = $0002000;
   VTDLY  = $0004000;
     VT0  = $0000000;
     VT1  = $0004000;
   FFDLY  = $0008000;
     FF0  = $0000000;
     FF1  = $0008000;

{c_cflag bits}
   CBAUD   = $000100F;
   B0      = $0000000;
   B50     = $0000001;
   B75     = $0000002;
   B110    = $0000003;
   B134    = $0000004;
   B150    = $0000005;
   B200    = $0000006;
   B300    = $0000007;
   B600    = $0000008;
   B1200   = $0000009;
   B1800   = $000000A;
   B2400   = $000000B;
   B4800   = $000000C;
   B9600   = $000000D;
   B19200  = $000000E;
   B38400  = $000000F;
   EXTA    = B19200;
   EXTB    = B38400;
   CSIZE   = $0000030;
     CS5   = $0000000;
     CS6   = $0000010;
     CS7   = $0000020;
     CS8   = $0000030;
   CSTOPB  = $0000040;
   CREAD   = $0000080;
   PARENB  = $0000100;
   PARODD  = $0000200;
   HUPCL   = $0000400;
   CLOCAL  = $0000800;
   CBAUDEX = $0001000;
   B57600  = $0001001;
   B115200 = $0001002;
   B230400 = $0001003;
   B460800 = $0001004;
   B500000 = $0001005;
   B576000 = $0001006;
   B921600 = $0001007;
   B1000000= $0001008;
   B1152000= $0001009;
   B1500000= $000100A;
   B2000000= $000100B;
   B2500000= $000100C;
   B3000000= $000100D;
   B3500000= $000100E;
   B4000000= $000100F;

   CIBAUD  = $100F0000;
   CMSPAR  = $40000000;
   CRTSCTS = $80000000;

{c_lflag bits}
   ISIG    = $0000001;
   ICANON  = $0000002;
   XCASE   = $0000004;
   ECHO    = $0000008;
   ECHOE   = $0000010;
   ECHOK   = $0000020;
   ECHONL  = $0000040;
   NOFLSH  = $0000080;
   TOSTOP  = $0000100;
   ECHOCTL = $0000200;
   ECHOPRT = $0000400;
   ECHOKE  = $0000800;
   FLUSHO  = $0001000;
   PENDIN  = $0004000;
   IEXTEN  = $0008000;

{c_line bits}
   TIOCM_LE   = $001;
   TIOCM_DTR  = $002;
   TIOCM_RTS  = $004;
   TIOCM_ST   = $008;
   TIOCM_SR   = $010;
   TIOCM_CTS  = $020;
   TIOCM_CAR  = $040;
   TIOCM_RNG  = $080;
   TIOCM_DSR  = $100;
   TIOCM_CD   = TIOCM_CAR;
   TIOCM_RI   = TIOCM_RNG;
   TIOCM_OUT1 = $2000;
   TIOCM_OUT2 = $4000;

{TCSetAttr}
   TCSANOW   = 0;
   TCSADRAIN = 1;
   TCSAFLUSH = 2;

{TCFlow}
   TCOOFF = 0;
   TCOON  = 1;
   TCIOFF = 2;
   TCION  = 3;

{TCFlush}
   TCIFLUSH  = 0;
   TCOFLUSH  = 1;
   TCIOFLUSH = 2;

{$endif cpuxtensa}

Type
  winsize = record
    ws_row,
    ws_col,
    ws_xpixel,
    ws_ypixel : word;
  end;
  TWinSize=winsize;

{$PACKRECORDS C}
  Termios = record
    c_iflag,
    c_oflag,
    c_cflag,
    c_lflag  : cardinal;
    c_line   : char;
    c_cc     : array[0..NCCS-1] of byte;
    c_ispeed,
    c_ospeed : cardinal;
  end;
  TTermios=Termios;
{$PACKRECORDS Default}

