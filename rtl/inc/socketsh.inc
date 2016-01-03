{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$INLINE ON}

type
{$ifdef SOCK_HAS_SINLEN}
  sa_family_t = cuchar;
{$else}
  sa_family_t = cushort;
{$endif}

const
  { Socket types }
{$ifdef sunos}
  SOCK_STREAM     = 2;               { stream socket }
  SOCK_DGRAM      = 1;               { datagram socket }
  SOCK_RAW        = 4;               { raw-protocol interface }
  SOCK_RDM        = 5;               { reliably-delivered message }
  SOCK_SEQPACKET  = 6;               { sequenced packet stream }
{$else}
  SOCK_STREAM     = 1;               { stream (connection) socket   }
  SOCK_DGRAM      = 2;               { datagram (conn.less) socket  }
  SOCK_RAW        = 3;               { raw socket                   }
  SOCK_RDM        = 4;               { reliably-delivered message   }
  SOCK_SEQPACKET  = 5;               { sequential packet socket     }
{$endif}

  INADDR_ANY   = CARDINAL(0);
  INADDR_NONE  = CARDINAL($FFFFFFFF);

const
  { Two constants to determine whether part of soket is for in or output }
  S_IN = 0;
  S_OUT = 1;

type
  pin_addr = ^in_addr;
  in_addr = packed record
    case boolean of
       true: (s_addr  : cuint32);         // inaddr_t=cuint32
       false: (s_bytes : packed array[1..4] of byte);
  end;

  TIn_addr = in_addr;

  TInAddr = in_addr;
  PInAddr = pin_addr;

  {pin_addrbytes = ^in_addrbytes;
  in_addrbytes = packed array [1..4] of byte;}

  psockaddr_in = ^sockaddr_in;
  sockaddr_in = packed record
    case boolean of
     false : (
  {$ifdef SOCK_HAS_SINLEN}
     sin_len     : cuchar;
  {$endif}
     sin_family  : sa_family_t;
     sin_port    : cushort;
     sin_addr    : in_addr;
     xpad         : array [0..7] of char; { to get to the size of sockaddr... }
      );
   true: (
  {$ifdef SOCK_HAS_SINLEN}
     len     : cuchar deprecated;
  {$endif}
     family  : sa_family_t deprecated;
     port    : cushort deprecated;
     addr    : cardinal deprecated;
     pad         : array [0..7] of char deprecated; { to get to the size of sockaddr... }
      );
  end;

  TInetSockAddr = sockaddr_in;
  PInetSockAddr = psockaddr_in;

  psockaddr = ^sockaddr;
  sockaddr = packed record // if sa_len is defined, sa_family_t is smaller
  {$ifdef SOCK_HAS_SINLEN}
     sa_len     : cuchar;
  {$endif}
    case integer of
      0: (sa_family: sa_family_t;
          sa_data: packed array[0..13] of cuint8);
      1: (sin_family: sa_family_t;
          sin_port: cushort;
          sin_addr: in_addr;
          sin_zero: packed array[0..7] of cuint8);
  end;

  TSockAddr = sockaddr;

  plinger = ^linger;
  linger = packed record
    l_onoff  : cint;	(* Linger active		*)
    l_linger : cint;	(* How long to linger for	*)
  end;

  TLinger = linger;

  pin6_addr = ^in6_addr;
  in6_addr = packed record
  case byte of
    0: (u6_addr8  : array[0..15] of byte);
    1: (u6_addr16 : array[0..7] of Word);
    2: (u6_addr32 : array[0..3] of Cardinal);
    3: (s6_addr8  : array[0..15] of shortint);
    4: (s6_addr   : array[0..15] of shortint);
    5: (s6_addr16 : array[0..7] of smallint);
    6: (s6_addr32 : array[0..3] of LongInt);
  end;

  Tin6_addr = in6_addr;

  TIn6Addr = in6_addr;
  PIn6Addr = pin6_addr;

  psockaddr_in6 = ^sockaddr_in6;
  sockaddr_in6 = packed Record
    {$ifdef SOCK_HAS_SINLEN}  // as per RFC 2553
      sin6_len    : cuint8;
    {$endif}
    sin6_family   : sa_family_t;
    sin6_port     : cuint16;
    sin6_flowinfo : cuint32;
    sin6_addr     : in6_addr;
    sin6_scope_id : cuint32;
  end;

  TInetSockAddr6 = sockaddr_in6;
  PInetSockAddr6 = psockaddr_in6;

  TSockPairArray = Array[0..1] of Longint;
  TSockArray     = Array[1..2] of Longint;              //legacy

  psockaddr_un = ^sockaddr_un;
  sockaddr_un = packed record
    {$ifdef SOCK_HAS_SINLEN}
      sun_len     : cuint8;
    {$endif}
    sun_family    : sa_family_t;
    sun_path      : array[0..107] of char;
  end;

  Tsocket=longint;   {To easy porting code from Kylix libc unit to sockets unit.}
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2

function socketerror : cint; 

function  fpsocket      (domain:cint; xtype:cint; protocol: cint):cint;
function  fprecv        (s:cint; buf: pointer; len: size_t; flags: cint):ssize_t;
function  fprecvfrom    (s:cint; buf: pointer; len: size_t; flags: cint; from : psockaddr; fromlen : psocklen):ssize_t;
function  fpsend        (s:cint; msg:pointer; len:size_t; flags:cint):ssize_t;
function  fpsendto      (s:cint; msg:pointer; len:size_t; flags:cint; tox :psockaddr; tolen: tsocklen):ssize_t;
function  fpbind        (s:cint; addrx : psockaddr; addrlen : tsocklen):cint;
function  fplisten      (s:cint; backlog : cint):cint;
function  fpaccept      (s:cint; addrx : psockaddr; addrlen : psocklen):cint;
function  fpconnect     (s:cint; name  : psockaddr; namelen : tsocklen):cint;
function  fpshutdown    (s:cint; how:cint):cint;
function  fpgetsockname (s:cint; name  : psockaddr; namelen : psocklen):cint;
function  fpgetpeername (s:cint; name  : psockaddr; namelen : psocklen):cint;
function  fpgetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : psocklen):cint;
function  fpsetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : tsocklen):cint;
function  fpsocketpair  (d:cint; xtype:cint; protocol:cint; sv:pcint):cint;

Function CloseSocket(Sock:Longint):Longint; // vote is still out on this one

{Basic Socket Functions}
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef legacysocket}
Function Socket(Domain,SocketType,Protocol:Longint):Longint; deprecated;
=======
Function Socket(Domain,SocketType,Protocol:Longint):Longint; deprecated;
Function CloseSocket(Sock:Longint):Longint; // vote is still out on this one
>>>>>>> graemeg/fixes_2_2
=======
Function Socket(Domain,SocketType,Protocol:Longint):Longint; deprecated;
Function CloseSocket(Sock:Longint):Longint; // vote is still out on this one
>>>>>>> origin/fixes_2_2
Function Send(Sock:Longint;Const Buf;BufLen,Flags:Longint):Longint; deprecated;
Function SendTo(Sock:Longint;Const Buf;BufLen,Flags:Longint;Var Addr; AddrLen : Longint):Longint; deprecated;
Function Recv(Sock:Longint;Var Buf;BufLen,Flags:Longint):Longint; deprecated;
Function RecvFrom(Sock : Longint; Var Buf; Buflen,Flags : Longint; Var Addr; var AddrLen : longInt) : longint;deprecated; 
Function Bind(Sock:Longint;Const Addr;AddrLen:Longint):Boolean;deprecated;
Function Listen (Sock,MaxConnect:Longint):Boolean;deprecated;
Function Accept(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;deprecated;
Function Connect(Sock:Longint;Const Addr;Addrlen:Longint):boolean;deprecated;
Function Shutdown(Sock:Longint;How:Longint):Longint;deprecated;
Function GetSocketName(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;deprecated;
Function GetPeerName(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;deprecated;
Function SetSocketOptions(Sock,Level,OptName:Longint;const OptVal;optlen:longint):Longint;deprecated;
Function GetSocketOptions(Sock,Level,OptName:Longint;Var OptVal;Var optlen:longint):Longint;deprecated;
Function SocketPair(Domain,SocketType,Protocol:Longint;var Pair:TSockArray):Longint;deprecated;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}

{ untyped/text filetype support }
=======
=======
>>>>>>> origin/fixes_2_2

{Text Support}
Procedure Sock2Text(Sock:Longint;Var SockIn,SockOut:Text);deprecated;

{Untyped File Support}
Procedure Sock2File(Sock:Longint;Var SockIn,SockOut:File);deprecated;

Function Accept(Sock:longint;var addr:TInetSockAddr;var SockIn,SockOut:File):Boolean; deprecated;
Function Accept(Sock:longint;var addr:TInetSockAddr;var SockIn,SockOut:text):Boolean;deprecated;
Function Connect(Sock:longint;const addr:TInetSockAddr;var SockIn,SockOut:text):Boolean; deprecated;
Function Connect(Sock:longint;const addr:TInetSockAddr;var SockIn,SockOut:file):Boolean;deprecated;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2

Procedure Sock2Text(Sock:Longint;Var SockIn,SockOut:Text);deprecated;
Function Accept(Sock:longint;var addr:TInetSockAddr;var SockIn,SockOut:File):Boolean; deprecated;
Function Accept(Sock:longint;var addr:TInetSockAddr;var SockIn,SockOut:text):Boolean;deprecated;
Function Connect(Sock:longint;const addr:TInetSockAddr;var SockIn,SockOut:text):Boolean; deprecated;
Function Connect(Sock:longint;const addr:TInetSockAddr;var SockIn,SockOut:file):Boolean;deprecated;
Procedure Sock2File(Sock:Longint;Var SockIn,SockOut:File);deprecated;
=======

>>>>>>> origin/fixes_2_2

{ Utility routines}
function htonl( host : longint):longint; inline;
Function NToHl( Net : Longint) : Longint; inline;
function htons( host : word):word; inline;
Function NToHs( Net : word):word; inline;

function NetAddrToStr (Entry : in_addr) : AnsiString;
function HostAddrToStr(Entry : in_addr) : AnsiString;
function StrToHostAddr(IP : AnsiString) : in_addr ;
function StrToNetAddr (IP : AnsiString) : in_addr;

{ these for are for netdb legacy compat}
Function HostToNet     (Host : in_addr) : in_addr;
Function NetToHost     (Net  : in_addr) : in_addr;
Function HostToNet     (Host : Longint) : Longint;
Function NetToHost     (Net  : Longint) : Longint;
Function ShortHostToNet(Host : Word) : Word;
Function ShortNetToHost(Net  : Word) : Word;

// ipv6
function HostAddrToStr6(Entry : Tin6_addr) : AnsiString;
function StrToHostAddr6(IP   : String) : Tin6_addr;     // not implemented?!?
function NetAddrToStr6 (Entry: Tin6_addr) : AnsiString;
function StrToNetAddr6 (IP   : AnsiString) : TIn6_Addr;

CONST
	NoAddress : in_addr  = (s_addr:0);
	NoNet     : in_addr  = (s_addr:0);
 	NoAddress6: in6_addr = (u6_addr16:(0,0,0,0,0,0,0,0));
  	NoNet6    : in6_addr = (u6_addr16:(0,0,0,0,0,0,0,0));

