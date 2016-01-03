{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2007 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$PACKRECORDS 2}
unit Sockets;
Interface

Uses ctypes,exec;

type
    size_t   = cuint32;         { as definied in the C standard}
    ssize_t  = cint32;          { used by function for returning number of bytes}

    socklen_t= cuint32;
    TSocklen = socklen_t;
    pSocklen = ^socklen_t;


//{ $i unxsockh.inc}
{$define BSD}
{$define SOCK_HAS_SINLEN}
{$i socketsh.inc}

type
  TUnixSockAddr = packed Record
                  sa_len     : cuchar;
                  family       : sa_family_t;
                  path:array[0..107] of char;    //104 total for freebsd.
                  end;

type
  hostent = record
    h_name     : PChar;
    h_aliases  : PPChar;
    h_addrtype : LongInt;
    h_Length   : LongInt;
    h_addr_list: ^PDWord;
  end;
  THostEnt = hostent;
  PHostEnt = ^THostEnt;


const
  AF_UNSPEC      = 0;               {* unspecified *}
  AF_LOCAL       = 1;               {* local to host (pipes, portals) *}
  AF_UNIX        = AF_LOCAL;        {* backward compatibility *}
  AF_INET        = 2;               {* internetwork: UDP, TCP, etc. *}
  AF_IMPLINK     = 3;               {* arpanet imp addresses *}
  AF_PUP         = 4;               {* pup protocols: e.g. BSP *}
  AF_CHAOS       = 5;               {* mit CHAOS protocols *}
  AF_NS          = 6;               {* XEROX NS protocols *}
  AF_ISO         = 7;               {* ISO protocols *}
  AF_OSI         = AF_ISO;
  AF_ECMA        = 8;               {* european computer manufacturers *}
  AF_DATAKIT     = 9;               {* datakit protocols *}
  AF_CCITT       = 10;              {* CCITT protocols, X.25 etc *}
  AF_SNA         = 11;              {* IBM SNA *}
  AF_DECnet      = 12;              {* DECnet *}
  AF_DLI         = 13;              {* DEC Direct data link interface *}
  AF_LAT         = 14;              {* LAT *}
  AF_HYLINK      = 15;              {* NSC Hyperchannel *}
  AF_APPLETALK   = 16;              {* Apple Talk *}
  AF_ROUTE       = 17;              {* Internal Routing Protocol *}
  AF_LINK        = 18;              {* Link layer interface *}
  pseudo_AF_XTP  = 19;              {* eXpress Transfer Protocol (no AF) *}
  AF_COIP        = 20;              {* connection-oriented IP, aka ST II *}
  AF_CNT         = 21;              {* Computer Network Technology *}
  pseudo_AF_RTIP = 22;              {* Help Identify RTIP packets *}
  AF_IPX         = 23;              {* Novell Internet Protocol *}
  AF_SIP         = 24;              {* Simple Internet Protocol *}
  pseudo_AF_PIP  = 25;              {* Help Identify PIP packets *}

  AF_MAX         = 26;


const
  EsockEINTR            = 4; // EsysEINTR;   
  EsockEBADF            = 9; // EsysEBADF;
  EsockEFAULT           = 14; // EsysEFAULT;
  EsockEINVAL           = 22; //EsysEINVAL;
  EsockEACCESS          = 13; //ESysEAcces;
  EsockEMFILE           = 24; //ESysEmfile;
  EsockENOBUFS          = 55; //ESysENoBufs;
  EsockENOTCONN         = 57; //ESysENotConn;
  EsockEPROTONOSUPPORT  = 43; //ESysEProtoNoSupport;
  EsockEWOULDBLOCK      = 35; //ESysEWouldBlock; // same as eagain on morphos

{ unix socket specific functions }
{*
Procedure Str2UnixSockAddr(const addr:string;var t:TUnixSockAddr;var len:longint); deprecated;
Function Bind(Sock:longint;const addr:string):boolean; deprecated;
Function Connect(Sock:longint;const addr:string;var SockIn,SockOut:text):Boolean; deprecated;
Function Connect(Sock:longint;const addr:string;var SockIn,SockOut:file):Boolean; deprecated;
Function Accept(Sock:longint;var addr:string;var SockIn,SockOut:text):Boolean;    deprecated;
Function Accept(Sock:longint;var addr:string;var SockIn,SockOut:File):Boolean;    deprecated;
*}
//function  fpaccept      (s:cint; addrx : psockaddr; addrlen : psocklen):cint; maybelibc
//function  fpbind      (s:cint; addrx : psockaddr; addrlen : tsocklen):cint;  maybelibc
//function  fpconnect     (s:cint; name  : psockaddr; namelen : tsocklen):cint;  maybelibc

var SocketBase : pLibrary;

function bsd_socket(domain : LongInt location 'd0'; type_ : LongInt location 'd1'; protocol : LongInt location 'd2') : LongInt; syscall legacy SocketBase 030;
function bsd_bind(s : LongInt location 'd0'; const name : psockaddr location 'a0'; namelen : LongInt location 'd1') : LongInt; syscall legacy SocketBase 036;
function bsd_listen(s : LongInt location 'd0'; backlog : LongInt location 'd1') : LongInt; syscall legacy SocketBase 042;
function bsd_accept(s : LongInt location 'd0'; addr : psockaddr location 'a0'; var addrlen : LongInt location 'a1') : LongInt; syscall legacy SocketBase 048;
function bsd_connect(s : LongInt location 'd0'; const name : psockaddr location 'a0'; namelen : LongInt location 'd1') : LongInt; syscall legacy SocketBase 054;
function bsd_sendto(s : LongInt location 'd0'; const msg : pChar location 'a0'; len : LongInt location 'd1'; flags : LongInt location 'd2'; const to_ : psockaddr location 'a1'; tolen : LongInt location 'd3') : LongInt; syscall legacy SocketBase 060;
function bsd_send(s : LongInt location 'd0'; const msg : pChar location 'a0'; len : LongInt location 'd1'; flags : LongInt location 'd2') : LongInt; syscall legacy SocketBase 066;
function bsd_recvfrom(s : LongInt location 'd0'; buf : pChar location 'a0'; len : LongInt location 'd1'; flags : LongInt location 'd2'; from : psockaddr location 'a1'; var fromlen : LongInt location 'a2') : LongInt; syscall legacy SocketBase 072;
function bsd_recv(s : LongInt location 'd0'; buf : pChar location 'a0'; len : LongInt location 'd1'; flags : LongInt location 'd2') : LongInt; syscall legacy SocketBase 078;
function bsd_shutdown(s : LongInt location 'd0'; how : LongInt location 'd1') : LongInt; syscall legacy SocketBase 084;
function bsd_closesocket(d : LongInt location 'd0') : LongInt; syscall legacy SocketBase 120;
function bsd_inet_ntoa(in_ : DWord location 'd0') : pChar; syscall legacy SocketBase 174;
function bsd_inet_addr(const cp : pChar location 'a0') : DWord; syscall legacy SocketBase 180;
function bsd_gethostbyname(const name : pChar location 'a0') : phostent; syscall legacy SocketBase 210;
function bsd_gethostbyaddr(const addr : pChar location 'a0'; len : LongInt location 'd0'; type_ : LongInt location 'd1') : phostent; syscall legacy SocketBase 216;


Implementation

//Uses {$ifndef FPC_USE_LIBC}SysCall{$else}initc{$endif};

threadvar internal_socketerror : cint;

{ Include filerec and textrec structures }
{$i filerec.inc}
{$i textrec.inc}
{******************************************************************************
                          Kernel Socket Callings
******************************************************************************}

function socketerror:cint;

begin
  socketerror:=internal_socketerror;
end;

//{$define uselibc:=cdecl; external;}

//const libname='c';
{
function cfpaccept      (s:cint; addrx : psockaddr; addrlen : psocklen):cint; cdecl; external libname name 'accept';
function cfpbind        (s:cint; addrx : psockaddr; addrlen : tsocklen):cint;  cdecl; external libname name 'bind';
function cfpconnect     (s:cint; name  : psockaddr; namelen : tsocklen):cint;  cdecl; external libname name 'connect';
function cfpgetpeername (s:cint; name  : psockaddr; namelen : psocklen):cint; cdecl; external libname name 'getpeername';
function cfpgetsockname (s:cint; name  : psockaddr; namelen : psocklen):cint; cdecl; external libname name 'getsockname';
function cfpgetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : psocklen):cint; cdecl; external libname name 'getsockopt';
function cfplisten      (s:cint; backlog : cint):cint;                          cdecl; external libname name 'listen';
function cfprecv        (s:cint; buf: pointer; len: size_t; flags: cint):ssize_t; cdecl; external libname name 'recv';
function cfprecvfrom    (s:cint; buf: pointer; len: size_t; flags: cint; from : psockaddr; fromlen : psocklen):ssize_t; cdecl; external libname name 'recvfrom';
//function cfprecvmsg     (s:cint; msg: pmsghdr; flags:cint):ssize_t; cdecl; external libname name '';
function cfpsend        (s:cint; msg:pointer; len:size_t; flags:cint):ssize_t; cdecl; external libname name 'send';
function cfpsendto      (s:cint; msg:pointer; len:size_t; flags:cint; tox :psockaddr; tolen: tsocklen):ssize_t; cdecl; external libname name 'sendto';
//function cfpsendmsg   (s:cint; hdr: pmsghdr; flags:cint):ssize; cdecl; external libname name '';
function cfpsetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen :tsocklen):cint; cdecl; external libname name 'setsockopt';
function cfpshutdown    (s:cint; how:cint):cint; cdecl; external libname name 'shutdown';
function cfpsocket      (domain:cint; xtype:cint; protocol: cint):cint; cdecl; external libname name 'socket';
function cfpsocketpair  (d:cint; xtype:cint; protocol:cint; sv:pcint):cint; cdecl; external libname name 'socketpair';
}

function cfpaccept(s : LongInt location 'd0'; addr : psockaddr location 'a0';  addrlen : pSocklen location 'a1') : LongInt; syscall legacy SocketBase 048;
function cfpbind(s : LongInt location 'd0'; const name : psockaddr location 'a0'; namelen : LongInt location 'd1') : LongInt; syscall legacy SocketBase 036;
function cfpconnect(s : LongInt location 'd0'; const name : psockaddr location 'a0'; namelen : LongInt location 'd1') : LongInt; syscall legacy SocketBase 054;
function cfpsendto(s : LongInt location 'd0'; const msg : pChar location 'a0'; len : LongInt location 'd1'; flags : LongInt location 'd2'; const to_ : psockaddr location 'a1'; tolen : LongInt location 'd3') : LongInt; syscall legacy SocketBase 060;
function cfpsend(s : LongInt location 'd0'; const msg : pChar location 'a0'; len : LongInt location 'd1'; flags : LongInt location 'd2') : LongInt; syscall legacy SocketBase 066;
function cfprecvfrom(s : LongInt location 'd0'; buf : pChar location 'a0'; len : LongInt location 'd1'; flags : LongInt location 'd2'; from : psockaddr location 'a1'; fromlen : pSockLen location 'a2') : LongInt; syscall legacy SocketBase 072;
function cfprecv(s : LongInt location 'd0'; buf : pChar location 'a0'; len : LongInt location 'd1'; flags : LongInt location 'd2') : LongInt; syscall legacy SocketBase 078;
function cfpgetsockopt(s : LongInt location 'd0'; level : LongInt location 'd1'; optname : LongInt location 'd2'; optval : Pointer location 'a0'; optlen : pSockLen location 'a1') : LongInt; syscall legacy SocketBase 096;
function cfpgetsockname(s : LongInt location 'd0'; hostname : psockaddr location 'a0'; namelen : pSockLen location 'a1') : LongInt; syscall legacy SocketBase 102;
function cfpgetpeername(s : LongInt location 'd0'; hostname : psockaddr location 'a0'; namelen : pSockLen location 'a1') : LongInt; syscall legacy SocketBase 108;
function cfpsetsockopt(s : LongInt location 'd0'; level : LongInt location 'd1'; optname : LongInt location 'd2'; const optval : Pointer location 'a0'; optlen : LongInt location 'd3') : LongInt; syscall legacy SocketBase 090;
function cfplisten(s : LongInt location 'd0'; backlog : LongInt location 'd1') : LongInt; syscall legacy SocketBase 042;
function cfpsocket(domain : LongInt location 'd0'; type_ : LongInt location 'd1'; protocol : LongInt location 'd2') : LongInt; syscall legacy SocketBase 030;
function cfpshutdown(s : LongInt location 'd0'; how : LongInt location 'd1') : LongInt; syscall legacy SocketBase 084;
function cfpCloseSocket(d : LongInt location 'd0') : LongInt; syscall legacy SocketBase 120;

function cfpErrno : LongInt; syscall legacy SocketBase 162;

function fpgeterrno: longint; inline;
begin
  fpgeterrno:=cfpErrno;
end;

function fpClose(d: LongInt): LongInt; inline;
begin
  fpClose:=cfpCloseSocket(d);
end;

function fpaccept      (s:cint; addrx : psockaddr; addrlen : psocklen):cint;

begin
 fpaccept:=cfpaccept(s,addrx,addrlen);
 internal_socketerror:=fpgeterrno; 
end;

function fpbind (s:cint; addrx : psockaddr; addrlen : tsocklen):cint;
begin
  fpbind:=cfpbind (s,addrx,addrlen);
  internal_socketerror:=fpgeterrno;
end;

function fpconnect     (s:cint; name  : psockaddr; namelen : tsocklen):cint;
begin
  fpconnect:=cfpconnect (s,name,namelen);
  internal_socketerror:=fpgeterrno;
end;

function fpgetpeername (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpgetpeername:=cfpgetpeername (s,name,namelen);
  internal_socketerror:=fpgeterrno;
end;

function fpgetsockname (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpgetsockname:=cfpgetsockname(s,name,namelen);
  internal_socketerror:=fpgeterrno;
end;

function fpgetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : psocklen):cint;
begin
  fpgetsockopt:=cfpgetsockopt(s,level,optname,optval,optlen);
  internal_socketerror:=fpgeterrno;
end;

function fplisten      (s:cint; backlog : cint):cint;
begin
  fplisten:=cfplisten(s,backlog);
  internal_socketerror:=fpgeterrno;
end;

function fprecv         (s:cint; buf: pointer; len: size_t; flags:cint):ssize_t;
begin
  fprecv:= cfprecv      (s,buf,len,flags);
  internal_socketerror:=fpgeterrno;
end;

function fprecvfrom    (s:cint; buf: pointer; len: size_t; flags: cint; from : psockaddr; fromlen : psocklen):ssize_t;
begin
  fprecvfrom:= cfprecvfrom (s,buf,len,flags,from,fromlen);
  internal_socketerror:=fpgeterrno;
end;

function fpsend         (s:cint; msg:pointer; len:size_t; flags:cint):ssize_t;
begin
  fpsend:=cfpsend (s,msg,len,flags);
  internal_socketerror:=fpgeterrno;
end;

function fpsendto       (s:cint; msg:pointer; len:size_t; flags:cint; tox :psockaddr; tolen: tsocklen):ssize_t;
begin
  fpsendto:=cfpsendto (s,msg,len,flags,tox,tolen);
  internal_socketerror:=fpgeterrno;
end;

function fpsetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen :tsocklen):cint;
begin
  fpsetsockopt:=cfpsetsockopt(s,level,optname,optval,optlen);
  internal_socketerror:=fpgeterrno;
end;

function fpshutdown     (s:cint; how:cint):cint;
begin
  fpshutdown:=cfpshutdown(s,how);
  internal_socketerror:=fpgeterrno;
end;

function fpsocket       (domain:cint; xtype:cint; protocol: cint):cint;
begin
  fpsocket:=cfpsocket(domain,xtype,protocol);
  internal_socketerror:=fpgeterrno;
end;


function fpsocketpair  (d:cint; xtype:cint; protocol:cint; sv:pcint):cint;
begin
{
  fpsocketpair:=cfpsocketpair(d,xtype,protocol,sv);
  internal_socketerror:=fpgeterrno;
}
  fpsocketpair:=-1;
end;


{$i sockovl.inc}
{$i sockets.inc}

// FIXME: this doesn't make any sense here, because SocketBase should be task-specific
// but FPC doesn't support that yet (TODO)
{$WARNING FIX ME, TODO}

initialization

  SocketBase:=NIL;
  SocketBase:=OpenLibrary('bsdsocket.library',4);

finalization

  if (SocketBase<>NIL) then CloseLibrary(SocketBase);

end.
