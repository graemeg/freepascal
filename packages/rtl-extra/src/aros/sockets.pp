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
{.$DEFINE SOCKETS_DEBUG}
{$ModeSwitch out}

unit Sockets;
Interface

uses
  ctypes,exec;

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
  SO_LINGER     = $0080;
  SOL_SOCKET    = $FFFF;

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

threadvar
  SocketBase: PLibrary;

function bsd_socket(Domain: LongInt; Type_: LongInt; Protocol: LongInt): LongInt; syscall SocketBase 5;
function bsd_bind(s: LongInt; const name: PSockAddr; NameLen: LongInt): LongInt; syscall SocketBase 6;
function bsd_listen(s: LongInt; BackLog: LongInt): LongInt; syscall SocketBase 7;
function bsd_accept(s: LongInt; Addr: PSockaddr; AddrLen: PSockLen): LongInt; syscall SocketBase 8;
function bsd_connect(s : LongInt; const Name: PSockaddr; NameLen: LongInt): LongInt; syscall SocketBase 9;
function bsd_sendto(s: LongInt; const Msg: PChar; Len: LongInt; Flags: LongInt; const To_: PSockaddr; ToLen: LongInt): LongInt; syscall SocketBase 10;
function bsd_send(s: LongInt; const msg: PChar; Len: LongInt; Flags: LongInt): LongInt; syscall SocketBase 11;
function bsd_recvfrom(s: LongInt; Buf: PChar; Len: LongInt; Flags: LongInt; From: PSockaddr; FromLen: PSockLen): LongInt; syscall SocketBase 12;
function bsd_recv(s: LongInt; buf: PChar; Len: LongInt; Flags: LongInt): LongInt; syscall SocketBase 13;
function bsd_shutdown(s: LongInt; How: LongInt): LongInt; syscall SocketBase 14;
function bsd_setsockopt(s: LongInt; level: LongInt; optname: LongInt; const optval: Pointer; optlen: LongInt) : LongInt; syscall SocketBase 15;
function bsd_getsockopt(s: LongInt; Level: LongInt; OptName: LongInt; OptVal: Pointer; OptLen: PSockLen): LongInt; syscall SocketBase 16;
function bsd_getsockname(s: LongInt; HostName: PSockaddr; NameLen: PSockLen): LongInt; syscall SocketBase 17;
function bsd_getpeername(s: LongInt; HostName: PSockaddr; NameLen: PSockLen): LongInt; syscall SocketBase 18;
function bsd_closesocket(s: LongInt): LongInt; syscall SocketBase 20;
function bsd_Errno: LongInt; syscall SocketBase 27;
function bsd_inet_ntoa(in_: LongWord): PChar; syscall SocketBase 29;
function bsd_inet_addr(const cp: PChar): LongWord; syscall SocketBase 30;
function bsd_gethostbyname(const Name: PChar): PHostEnt; syscall SocketBase 35;
function bsd_gethostbyaddr(const Addr: PByte; Len: LongInt; Type_: LongInt): PHostEnt; syscall SocketBase 36;

Implementation

threadvar internal_socketerror: cint;

{ Include filerec and textrec structures }
{.$i filerec.inc}
{.$i textrec.inc}

{******************************************************************************
                          Kernel Socket Callings
******************************************************************************}

function socketerror: cint;
begin
  socketerror := internal_socketerror;
end;

function fpgeterrno: longint; inline;
begin
  if Assigned(SocketBase) then
    fpgeterrno := bsd_Errno
  else
    fpgeterrno := 0;
end;

function fpClose(d: LongInt): LongInt; inline;
begin
  if Assigned(SocketBase) then
    fpClose := bsd_CloseSocket(d)
  else
    fpClose := -1;
end;

function fpaccept(s: cint; addrx: PSockaddr; Addrlen: PSocklen): cint;
begin
  fpaccept := bsd_accept(s,addrx,addrlen);
  internal_socketerror := fpgeterrno;
end;

function fpbind(s:cint; addrx: psockaddr; addrlen: tsocklen): cint;
begin
  fpbind := bsd_bind(s, addrx, addrlen);
  internal_socketerror := fpgeterrno;
end;

function fpconnect(s:cint; name: psockaddr; namelen: tsocklen): cint;
begin
  if Assigned(SocketBase) then
  begin
    fpconnect := bsd_connect(s, name, namelen);
    internal_socketerror := fpgeterrno;
  end
  else
  begin
    fpconnect := -1;
    internal_socketerror := ESockEPROTONOSUPPORT;
  end;
end;

function fpgetpeername (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpgetpeername := bsd_getpeername(s,name,namelen);
  internal_socketerror := fpgeterrno;
end;

function fpgetsockname(s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpgetsockname := bsd_getsockname(s,name,namelen);
  internal_socketerror := fpgeterrno;
end;

function fpgetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : psocklen):cint;
begin
  fpgetsockopt := bsd_getsockopt(s,level,optname,optval,optlen);
  internal_socketerror := fpgeterrno;
end;

function fplisten(s:cint; backlog : cint):cint;
begin
  fplisten := bsd_listen(s, backlog);
  internal_socketerror := fpgeterrno;
end;

function fprecv(s:cint; buf: pointer; len: size_t; Flags: cint): ssize_t;
begin
  fprecv := bsd_recv(s,buf,len,flags);
  internal_socketerror := fpgeterrno;
end;

function fprecvfrom(s:cint; buf: pointer; len: size_t; flags: cint; from : psockaddr; fromlen : psocklen):ssize_t;
begin
  fprecvfrom := bsd_recvfrom(s, buf, len, flags, from, fromlen);
  internal_socketerror := fpgeterrno;
end;

function fpsend(s:cint; msg:pointer; len:size_t; flags:cint):ssize_t;
begin
  fpsend := bsd_send(s, msg, len, flags);
  internal_socketerror := fpgeterrno;
end;

function fpsendto(s:cint; msg:pointer; len:size_t; flags:cint; tox :psockaddr; tolen: tsocklen):ssize_t;
begin
  fpsendto := bsd_sendto(s, msg, len, flags, tox, tolen);
  internal_socketerror := fpgeterrno;
end;

function fpsetsockopt(s:cint; level:cint; optname:cint; optval:pointer; optlen :tsocklen):cint;
begin
  fpsetsockopt := bsd_setsockopt(s, level, optname, optval, optlen);
  internal_socketerror := fpgeterrno;
end;

function fpshutdown(s: cint; how: cint): cint;
begin
  fpshutdown := bsd_shutdown(s, how);
  internal_socketerror := fpgeterrno;
end;

function fpsocket(domain: cint; xtype: cint; protocol: cint): cint;
begin
  if Assigned(SocketBase) then
  begin
    fpsocket := bsd_socket(domain, xtype, protocol);
    internal_socketerror := fpgeterrno;
  end
  else
  begin
    internal_socketerror := ESockEPROTONOSUPPORT;
    fpsocket := -1;
  end;
end;


function fpsocketpair(d:cint; xtype:cint; protocol:cint; sv:pcint):cint;
begin
{
  fpsocketpair:=cfpsocketpair(d,xtype,protocol,sv);
  internal_socketerror:=fpgeterrno;
}
  fpsocketpair:=-1;
end;


{$i sockovl.inc}
{$i sockets.inc}

const
  BSDSOCKET_LIBRARY_VER = 4;

procedure BSDSocketOpen;
begin
{$IFDEF SOCKETS_DEBUG}
  SysDebugLn('FPC Sockets: Opening bsdsocket.library...');
{$ENDIF}
  SocketBase:=OpenLibrary('bsdsocket.library', BSDSOCKET_LIBRARY_VER);
{$IFDEF SOCKETS_DEBUG}
  if SocketBase = nil then
    SysDebugLn('FPC Sockets: FAILED to open bsdsocket.library.')
  else
    SysDebugLn('FPC Sockets: bsdsocket.library opened successfully.');
{$ENDIF}
end;

procedure BSDSocketClose;
begin
  if (SocketBase<>NIL) then CloseLibrary(SocketBase);
  SocketBase:=NIL;
{$IFDEF SOCKETS_DEBUG}
  SysDebugLn('FPC Sockets: bsdsocket.library closed.');
{$ENDIF}
end;

initialization
  AddThreadInitProc(@BSDSocketOpen);
  AddThreadExitProc(@BSDSocketClose);
  BSDSocketOpen;

finalization
  BSDSocketClose;
end.
