{
   This file is part of the Free Pascal run time library.
   (c) 2000-2003 by Marco van de Voort
   member of the Free Pascal development team.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   Termios header for FreeBSD

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

CONST

{
 * Special Control Characters
 *
 * Index into c_cc[] character array.
 *
 *      Name         Subscript  Enabled by
 }
 { control characters }
VINTR	= 0;
VQUIT	= 1;
VERASE	= 2;
VKILL	= 3;
VEOF	= 4;
VEOL	= 5;
VMIN	= 4;
VTIME	= 5;
VEOL2	= 6;
VSWTCH	= 7;
VSTART  = 8;
VSTOP   = 9;
VSUSP   = 10;

{ number of control characters }
 NCC	= 11;
 NCCS           =NCC;

Type
  winsize = packed record
    ws_row,
    ws_col,
    ws_xpixel,
    ws_ypixel : word;
  end;
  TWinSize=winsize;

// typedef unsigned long tcflag_t;
// typedef unsigned char speed_t;
// typedef unsigned char cc_t;
 tcflag_t = Cardinal;
 speed_t = byte;
 cc_t = char;

type
  Termios = packed record
    c_iflag,
    c_oflag,
    c_cflag,
    c_lflag  : tcflag_t;
    c_line   : char;
    c_ixxxxx : speed_t;
    c_oxxxxx : speed_t;
    c_cc     : array[0..NCCS-1] of speed_t;
  end;
  TTermios=Termios;

CONST


 POSIX_VDISABLE=Chr($ff);
{

#define CCEQ(val, c)    ((c) == (val) ? (val) != _POSIX_VDISABLE : 0)
}

{ * Input flags - software input processing}

        IGNBRK          =       $1;     { ignore BREAK condition }
        BRKINT          =       $2;     { map BREAK to SIGINTR }
        IGNPAR          =       $4;     { ignore (discard) parity errors }
        PARMRK          =       $8;     { mark parity and framing errors }
        INPCK           =      $10;     { enable checking of parity errors }
        ISTRIP          =      $20;     { strip 8th bit off chars }
        INLCR           =      $40;     { map NL into CR }
        IGNCR           =      $80;     { ignore CR }
        ICRNL           =     $100;     { map CR to NL (ala CRMOD) }
        IUCLC			=	  $200;		{ maps all upper case to lower }
        IXON            =     $400;     { enable output flow control }
        IXANY           =     $800;     { enable input flow control }
        IXOFF           =    $1000;	    { any char will restart after stop }

{
 * Output flags - software output processing
}

OPOST		= $01;		{ enable postprocessing of output }
OLCUC		= $02;		{ maps lowercase to uppercase }
ONLCR		= $04;		{ maps NL to CR-NL on output }
OCRNL		= $08;		{ maps CR to NL on output }
ONOCR		= $10;		{ no CR output when at column 0 }
ONLRET		= $20;		{ newline performs CR function }
OFILL		= $40;		{ uses fill characters for delays }
OFDEL		= $80;		{ Fills are DEL, otherwise NUL }
NLDLY		= $100;		{ Newline delays: }
NL0			= $000;
NL1			= $100;
CRDLY		= $600;		{ Carriage return delays: }
CR0			= $000;
CR1			= $200;
CR2			= $400;
CR3			= $600;
TABDLY		= $1800;		{ Tab delays: }
TAB0		= $0000;
TAB1		= $0800;
TAB2		= $1000;
TAB3		= $1800;
BSDLY		= $2000;		{ Backspace delays: }
BS0			= $0000;
BS1			= $2000;
VTDLY		= $4000;		{ Vertical tab delays: }
VT0			= $0000;
VT1			= $4000;
FFDLY		= $8000;		{ Form feed delays: }
FF0			= $0000;
FF1			= $8000;

{
  c_cflag - control modes
}

CBAUD		= $1F;			{ line speed definitions }

B0			= $00;
B50			= $01;
B75			= $02;
B110		= $03;
B134		= $04;
B150		= $05;
B200		= $06;
B300		= $07;
B600		= $08;
B1200		= $09;
B1800		= $0A;
B2400		= $0B;
B4800		= $0C;
B9600		= $0D;
B19200		= $0E;
B38400		= $0F;
B57600		= $10;
B115200		= $11;
B230400		= $12;
B31250		= $13;			{ for MIDI }

CSIZE		= $20;			{ character size }
CS5			= $00;			{ only 7 and 8 bits supported }
CS6			= $00;
CS7			= $00;
CS8			= $20;
CSTOPB		= $40;			{ send 2 stop bits, not 1 }
CREAD		= $80;			{ enables receiver }
PARENB		= $100;			{ xmit parity enable }
PARODD		= $200;			{ odd parity, else even }
HUPCL		= $400;			{ hangs up on last close }
CLOCAL		= $800;			{ indicates local line }
XLOBLK		= $1000;			{ block layer output ?}
CTSFLOW		= $2000;			{ enable CTS flow }
RTSFLOW		= $4000;			{ enable RTS flow }
CRTSCTS		= RTSFLOW or CTSFLOW;



{
 * "Local" flags - dumping ground for other state
 *
 * Warning: some flags in this structure begin with
 * the letter "I" and look like they belong in the
 * input flag.
 }
 
{
  c_lflag - local modes
}

ISIG		= $01;			{ enable signals }
ICANON		= $02;			{ Canonical input }
XCASE		= $04;			{ Canonical u/l case }
ECHO		= $08;			{ Enable echo }
ECHOE		= $10;			{ Echo erase as bs-sp-bs }
ECHOK		= $20;			{ Echo nl after kill }
ECHONL		= $40;			{ Echo nl }
NOFLSH		= $80;			{ Disable flush after int or quit }
TOSTOP      = $100;         { stop bg processes that write to tty }
IEXTEN      = $200;         { implementation defined extensions }

{
  Event codes.  Returned from TCWAITEVENT
}
EV_RING			= $0001;
EV_BREAK		= $0002;
EV_CARRIER		= $0004;
EV_CARRIERLOST	= $0008;
 
{
 * Commands passed to tcsetattr() for setting the termios structure.
}

CONST

        TCSANOW         = $01;             { make change immediate }
        TCSADRAIN       = $02;             { drain output, then change }
        TCSAFLUSH       = $04;             { drain output, flush input }
        
        // TCASOFT undefined under BeOS
        TCSASOFT        = $10;           { flag - don't alter h.w. state }


        TCIFLUSH        = $01;
        TCOFLUSH        = $02;
        TCIOFLUSH       = (TCIFLUSH or TCOFLUSH);
        TCOOFF          = $01;
        TCOON           = $02;
        TCIOFF          = $04;
        TCION           = $08;

{
#include <sys/cdefs.h>

__BEGIN_DECLS
speed_t cfgetispeed __P((const struct termios *));
speed_t cfgetospeed __P((const struct termios *));
int     cfsetispeed __P((struct termios *, speed_t));
int     cfsetospeed __P((struct termios *, speed_t));
int     tcgetattr __P((int, struct termios *));
int     tcsetattr __P((int, int, const struct termios *));
int     tcdrain __P((int));
int     tcflow __P((int, int));
int     tcflush __P((int, int));
int     tcsendbreak __P((int, int));

#ifndef _POSIX_SOURCE
void    cfmakeraw __P((struct termios *));
int     cfsetspeed __P((struct termios *, speed_t));
#endif { !_POSIX_SOURCE }
__END_DECLS

#endif { !_KERNEL }



struct winsize {
        unsigned short  ws_row;         { rows, in characters }
        unsigned short  ws_col;         { columns, in characters }
        unsigned short  ws_xpixel;      { horizontal size, pixels }
        unsigned short  ws_ypixel;      { vertical size, pixels }
};

}
(*       IOCTLREAD        = $40000000;
       IOCTLWRITE       = $80000000;
       IOCTLVOID        = $20000000;

        TIOCMODG        = IOCTLREAD+$47400+ 3;  { get modem control state }
        TIOCMODS        = IOCTLWRITE+$47400+ 4; { set modem control state }
                TIOCM_LE        =$0001;         { line enable }
                TIOCM_DTR       =$0002;         { data terminal ready }
                TIOCM_RTS       =$0004;         { request to send }
                TIOCM_ST        =$0010;         { secondary transmit }
                TIOCM_SR        =$0020;         { secondary receive }
                TIOCM_CTS       =$0040;         { clear to send }
                TIOCM_CAR       =$0100;         { carrier detect }
                TIOCM_CD        =TIOCM_CAR;
                TIOCM_RNG       =$0200;         { ring }
                TIOCM_RI        =TIOCM_RNG;
                TIOCM_DSR       =$0400;         { data set ready }
                                                { 8-10 compat }
        TIOCEXCL         =IOCTLVOID+$7400+ 13;          { set exclusive use of tty }
        TIOCNXCL         =IOCTLVOID+$7400+ 14;          { reset exclusive use of tty }
*)                                                { 15 unused }
//        TIOCFLUSH        =IOCTLWRITE+$47400+ 16;        { flush buffers }
                                                { 17-18 compat }
//        TIOCGETA         =IOCTLREAD+$2C7400+ 19; { get termios struct }
//        TIOCSETA         =IOCTLWRITE+$2C7400+ 20; { set termios struct }
//        TIOCSETAW        =IOCTLWRITE+$2C7400+ 21; { drain output, set }
//        TIOCSETAF        =IOCTLWRITE+$2C7400+ 22; { drn out, fls in, set }
//        TIOCGETD         =IOCTLREAD+$47400+ 26; { get line discipline }
//        TIOCSETD         =IOCTLWRITE+$47400+ 27;        { set line discipline }
	                                                { 127-124 compat }

// BeOS values
		TIOCGETA		= $8000;		
		TIOCSETA		= TIOCGETA + 1;
		TIOCSETAF		= TIOCGETA + 2;
		TIOCSETAW		= TIOCGETA + 3;
		TCWAITEVENT		= TIOCGETA + 4;
		TIOCSBRK		= TIOCGETA + 5;
		TIOCFLUSH		= TIOCGETA + 6;
		TCXONC			= TIOCGETA + 7;
		TCQUERYCONNECTED= TIOCGETA + 8;
		TCGETBITS		= TIOCGETA + 9;
		TIOCSDTR		= TIOCGETA + 10;
		TCSETRTS		= TIOCGETA + 11;
		TIOCGWINSZ		= TIOCGETA + 12;
		TIOCSWINSZ		= TIOCGETA + 13;
		TCVTIME			= TIOCGETA + 14;
		
		
//		TIOCTIMESTAMP 	= TCVTIME;
// end BeOS values
(*		      
//        TIOCSBRK         =IOCTLVOID+$7400+ 123;         { set break bit }
        TIOCCBRK         =IOCTLVOID+$7400+ 122;         { clear break bit }
//        TIOCSDTR         =IOCTLVOID+$7400+ 121;         { set data terminal ready }
        TIOCCDTR         =IOCTLVOID+$7400+ 120;         { clear data terminal ready }
        TIOCGPGRP        =IOCTLREAD+$47400+ 119;        { get pgrp of tty }
        TIOCSPGRP        =IOCTLWRITE+$47400+ 118;       { set pgrp of tty }
                                                { 117-116 compat }
        TIOCOUTQ         =IOCTLREAD+$47400+ 115;        { output queue size }
        TIOCSTI          =IOCTLWRITE+$17400+ 114;       { simulate terminal input }
        TIOCNOTTY        =IOCTLVOID+$7400+ 113;         { void tty association }
        TIOCPKT          =IOCTLWRITE+$47400+ 112;       { pty: set/clear packet mode }
                TIOCPKT_DATA            =$00;   { data packet }
                TIOCPKT_FLUSHREAD       =$01;   { flush packet }
                TIOCPKT_FLUSHWRITE      =$02;   { flush packet }
                TIOCPKT_STOP            =$04;   { stop output }
                TIOCPKT_START           =$08;   { start output }
                TIOCPKT_NOSTOP          =$10;   { no more ^S, ^Q }
                TIOCPKT_DOSTOP          =$20;   { now do ^S ^Q }
                TIOCPKT_IOCTL           =$40;   { state change of pty driver }
        TIOCSTOP         =IOCTLVOID+$7400+ 111;         { stop output, like ^S }
        TIOCSTART        =IOCTLVOID+$7400+ 110;         { start output, like ^Q }
        TIOCMSET         =IOCTLWRITE+$47400+ 109;       { set all modem bits }
        TIOCMBIS         =IOCTLWRITE+$47400+ 108;       { bis modem bits }
        TIOCMBIC         =IOCTLWRITE+$47400+ 107;       { bic modem bits }
        TIOCMGET         =IOCTLREAD+$47400+ 106;        { get all modem bits }
        TIOCREMOTE       =IOCTLWRITE+$47400+ 105;       { remote input editing }
//        TIOCGWINSZ       =IOCTLREAD+$87400+ 104;        { get window size }
//        TIOCSWINSZ       =IOCTLWRITE+$87400+ 103;       { set window size }
        TIOCUCNTL        =IOCTLWRITE+$47400+ 102;       { pty: set/clr usr cntl mode }
        TIOCSTAT         =IOCTLVOID+$7400+ 101;         { simulate ^T status message }
  //                       UIOCCMD(n)   _IO('u', n)     { usr cntl op "n" }
        TIOCCONS         =IOCTLWRITE+$47400+ 98;        { become virtual console }
        TIOCSCTTY        =IOCTLVOID+$7400+ 97;          { become controlling tty }
        TIOCEXT          =IOCTLWRITE+$47400+ 96;        { pty: external processing }
        TIOCSIG          =IOCTLVOID+$7400+ 95;          { pty: generate signal }
        TIOCDRAIN        =IOCTLVOID+$7400+ 94;          { wait till output drained }
        TIOCMSDTRWAIT    =IOCTLWRITE+$47400+ 91;        { modem: set wait on close }
        TIOCMGDTRWAIT    =IOCTLREAD+$47400+ 90; { modem: get wait on close }
//        TIOCTIMESTAMP    =IOCTLREAD+$87400+ 89;         { enable/get timestamp
//                                                 * of last input event }
        TIOCDCDTIMESTAMP =IOCTLREAD+$87400+ 88; { enable/get timestamp
                                                 * of last DCd rise }
        TIOCSDRAINWAIT   =IOCTLWRITE+$47400+ 87;        { set ttywait timeout }
        TIOCGDRAINWAIT   =IOCTLREAD+$47400+ 86; { get ttywait timeout }

        TTYDISC          =0;            { termios tty line discipline }
        SLIPDISC         =4;            { serial IP discipline }
        PPPDISC          =5;            { PPP discipline }
        NETGRAPHDISC     =6;            { Netgraph tty node discipline }

		// OCO 31/10/2005 For compatiblity (defined to compile ShiftState function
		// in keyboard.pp)
		// Maybe, it should not work but it compile at least...
		TIOCLINUX        = $541C;

*)
{
 * Defaults on "first" open.
 }
        TTYDEF_IFLAG     =(BRKINT       or ICRNL        or IXON or IXANY);
       TTYDEF_OFLAG      =(OPOST or ONLCR);
       TTYDEF_LFLAG      =(ECHO or ICANON or ISIG or IEXTEN or ECHOE );
        TTYDEF_CFLAG     =(CREAD or CS8 or HUPCL);
       TTYDEF_SPEED      =(B9600);



{
 * Control Character Defaults
 }
(*        CtrlMask        = $1f;  {\037}
        CEOF            =chr( ORD('d') and CtrlMask);
        CEOL            =chr( $ff and CtrlMask);{ XXX avoid _POSIX_VDISABLE }
        CERASE          =chr( $7F and CtrlMask);
        CINTR           =chr(ORD('c') and CtrlMask);
        CSTATUS         =chr(ORD('t') and CtrlMask);
        CKILL           =chr(ORD('u') and CtrlMask);
        CMIN            =chr(1);
        CQUIT           =chr(034  and CtrlMask);        { FS, ^\ }
        CSUSP           =chr(ORD('z') and CtrlMask);
        CTIME           =chr(0);
        CDSUSP          =chr(ORD('y') and CtrlMask);
        CSTART          =chr(ORD('q') and CtrlMask);
        CSTOP           =chr(ORD('s') and CtrlMask);
        CLNEXT          =chr(ORD('v') and CtrlMask);
        CDISCARD        =chr(ORD('o') and CtrlMask);
        CWERASE         =chr(ORD('w') and CtrlMask);
        CREPRINT        =chr(ORD('r') and CtrlMask);
        CEOT            =CEOF;
{ compat }
        CBRK            =CEOL;
        CRPRNT          =CREPRINT;
        CFLUSH          =CDISCARD;
*)

{
 *        TTYDEFCHARS to include an array of default control characters.
}
    ttydefchars : array[0..NCCS-1] OF char =(
        Chr(VINTR), Chr(VQUIT), Chr(VERASE), Chr(VKILL), Chr(VEOF), Chr(VEOL),
        Chr(VEOL2), Chr(VSWTCH), Chr(VSTART), Chr(VSTOP), Chr(VSUSP));

<<<<<<< HEAD
<<<<<<< HEAD
{
  According to posix/sys/ioctl.h
  /* these currently work only on sockets */	
}
	FIONBIO  = $be000000;
	FIONREAD = $be000001;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
