{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 Yuri Prokushev
    Copyright (c) 2005 Soren Ager

    Sockets implementation for OS/2

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$MODE ObjFPC}
{ $DEFINE notUnix}      // To make ssockets.pp compile
unit Sockets;

Interface

Uses
  so32dll,ctypes;

Const
//  AF_LOCAL       = so32dll.AF_LOCAL;
  AF_UNSPEC      = so32dll.AF_UNSPEC;
  AF_LOCAL       = so32dll.AF_LOCAL;
  AF_UNIX        = so32dll.AF_UNIX;
  AF_OS2         = so32dll.AF_OS2;
  AF_INET        = so32dll.AF_INET;
  AF_IMPLINK     = so32dll.AF_IMPLINK;     // arpanet imp addresses
  AF_PUP         = so32dll.AF_PUP;         // pup protocols: e.g. BSP
  AF_CHAOS       = so32dll.AF_CHAOS;       // mit CHAOS protocols
  AF_NS          = so32dll.AF_NS;          // XEROX NS protocols
  AF_ISO         = so32dll.AF_ISO;         // ISO protocols
  AF_OSI         = so32dll.AF_OSI;
  AF_ECMA        = so32dll.AF_ECMA;        // european computer manufacturers
  AF_DATAKIT     = so32dll.AF_DATAKIT;     // datakit protocols
  AF_CCITT       = so32dll.AF_CCITT;       // CCITT protocols, X.25 etc
  AF_SNA         = so32dll.AF_SNA;         // IBM SNA
  AF_DECnet      = so32dll.AF_DECnet;      // DECnet
  AF_DLI         = so32dll.AF_DLI;         // DEC Direct data link interface
  AF_LAT         = so32dll.AF_LAT;         // LAT
  AF_HYLINK      = so32dll.AF_HYLINK;      // NSC Hyperchannel
  AF_APPLETALK   = so32dll.AF_APPLETALK;   // Apple Talk
  AF_NB          = so32dll.AF_NB;          // Netbios
  AF_NETBIOS     = so32dll.AF_NETBIOS;     // Netbios
  AF_LINK        = so32dll.AF_LINK;        // Link layer interface
  pseudo_AF_XTP  = so32dll.pseudo_AF_XTP;  // eXpress Transfer Protocol (no AF)
  AF_COIP        = so32dll.AF_COIP;        // connection-oriented IP, aka ST II
  AF_CNT         = so32dll.AF_CNT;         // Computer Network Technology
  pseudo_AF_RTIP = so32dll.pseudo_AF_RTIP; // Help Identify RTIP packets
  AF_IPX         = so32dll.AF_IPX;         // Novell Internet Protocol
  AF_SIP         = so32dll.AF_SIP;         // Simple Internet Protocol
  AF_INET6       = so32dll.AF_INET6;
  pseudo_AF_PIP  = so32dll.pseudo_AF_PIP;  // Help Identify PIP packets
  AF_ROUTE       = so32dll.AF_ROUTE;       // Internal Routing Protocol
  AF_FWIP        = so32dll.AF_FWIP;        // firewall support
  AF_IPSEC       = so32dll.AF_IPSEC;       // IPSEC and encryption techniques
  AF_DES         = so32dll.AF_DES;         // DES
  AF_MD5         = so32dll.AF_MD5;
  AF_CDMF        = so32dll.AF_CDMF;

  AF_MAX         = so32dll.AF_MAX;

//  PF_LOCAL     = so32dll.PF_LOCAL;
  PF_OS2       = so32dll.PF_OS2;
  PF_IMPLINK   = so32dll.PF_IMPLINK;
  PF_PUP       = so32dll.PF_PUP;
  PF_CHAOS     = so32dll.PF_CHAOS;
  PF_NS        = so32dll.PF_NS;
  PF_ISO       = so32dll.PF_ISO;
  PF_OSI       = so32dll.PF_OSI;
  PF_ECMA      = so32dll.PF_ECMA;
  PF_DATAKIT   = so32dll.PF_DATAKIT;
  PF_CCITT     = so32dll.PF_CCITT;
  PF_SNA       = so32dll.PF_SNA;
  PF_DECnet    = so32dll.PF_DECnet;
  PF_DLI       = so32dll.PF_DLI;
  PF_LAT       = so32dll.PF_LAT;
  PF_HYLINK    = so32dll.PF_HYLINK;
  PF_APPLETALK = so32dll.PF_APPLETALK;
  PF_NETBIOS   = so32dll.PF_NB;
  PF_NB        = so32dll.PF_NB;
  PF_ROUTE     = so32dll.PF_ROUTE;
  PF_LINK      = so32dll.PF_LINK;
  PF_XTP       = so32dll.PF_XTP;  // really just proto family, no AF
  PF_COIP      = so32dll.PF_COIP;
  PF_CNT       = so32dll.PF_CNT;
  PF_SIP       = so32dll.PF_SIP;
  PF_INET6     = so32dll.PF_INET6;
  PF_IPX       = so32dll.PF_IPX;     // same format as AF_NS
  PF_RTIP      = so32dll.PF_RTIP;    // same format as AF_INET
  PF_PIP       = so32dll.PF_PIP;

  PF_MAX       = so32dll.PF_MAX;

const EsockEINTR  = SOCEINTR;
      EsockEBADF  = SOCEBADF;
      EsockEFAULT = SOCEFAULT;
      EsockEINVAL = SOCEINVAL;
      EsockEACCESS = SOCEACCES;
      EsockEMFILE  = SOCEMFILE;
      EsockEMSGSIZE = SOCEMSGSIZE;
      EsockENOBUFS = SOCENOBUFS;
      EsockENOTCONN = SOCENOTCONN;
      EsockENOTSOCK = SOCENOTSOCK;
      EsockEPROTONOSUPPORT = SOCEPROTONOSUPPORT;
      EsockEWOULDBLOCK = SOCEWOULDBLOCK;


Type
  cushort=word;
  cuint16=word;
  cuint32=cardinal;
  size_t =cuint32;
  ssize_t=cuint16;
  cint   =longint;
  pcint  =^cint;
  tsocklen=cint;
  psocklen=^tsocklen;

// OS/2 stack based on BSD stack
{$DEFINE BSD}
{$I socketsh.inc}
  INVALID_SOCKET = TSocket(not(0));
  SOCKET_ERROR = -1;


Implementation

{Include filerec and textrec structures}
{$I filerec.inc}
{$I textrec.inc}

{******************************************************************************
                          Basic Socket Functions
******************************************************************************}

function SocketError: cint;
begin
  SocketError := so32dll.Sock_ErrNo;
end;

Function socket(Domain,SocketType,Protocol:Longint):Longint;
begin
  Socket:=so32dll.Socket(Domain,SocketType,ProtoCol);
end;

Function Send(Sock:Longint;Const Buf;BufLen,Flags:Longint):Longint;
begin
  Send:=fpSend(Sock,@Buf,BufLen,Flags);
end;

Function SendTo(Sock:Longint;Const Buf;BufLen,Flags:Longint;Var Addr; AddrLen : Longint):Longint;
begin
  SendTo:=fpSendTo(Sock,@Buf,BufLen,Flags,@Addr,AddrLen);
end;

Function Recv(Sock:Longint;Var Buf;BufLen,Flags:Longint):Longint;
begin
  Recv:=so32dll.Recv(Sock,Buf,BufLen,Flags);
end;

Function RecvFrom(Sock : Longint; Var Buf; Buflen,Flags : Longint; Var Addr; var AddrLen : longInt) : longint;
begin
  RecvFrom:=so32dll.RecvFrom(Sock,Buf,BufLen,Flags,so32dll.SockAddr(Addr),AddrLen);
end;

Function Bind(Sock:Longint;Const Addr;AddrLen:Longint):Boolean;
begin
  Bind:=fpBind(Sock,@Addr,AddrLen)=0;
end;

Function Listen(Sock,MaxConnect:Longint):Boolean;
begin
  Listen := so32dll.Listen(Sock,MaxConnect) = 0;
end;

Function Accept(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;
begin
  Accept:=so32dll.Accept(Sock,so32dll.SockAddr(Addr), AddrLen);
end;

Function Connect(Sock:Longint;const Addr; Addrlen:Longint):Boolean;
begin
  Connect:=fpConnect(Sock,@Addr,AddrLen)=0;
end;

Function Shutdown(Sock:Longint;How:Longint):Longint;
begin
  ShutDown:=so32dll.ShutDown(Sock,How);
end;

Function GetSocketName(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;
begin
  GetSocketName:=so32dll.GetSockName(Sock, so32dll.SockAddr(Addr),AddrLen);
end;

Function GetPeerName(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;
begin
  GetPeerName:=so32dll.GetPeerName(Sock,so32dll.SockAddr(Addr),AddrLen);
end;

Function SetSocketOptions(Sock,Level,OptName:Longint;Const OptVal;optlen:longint):Longint;
begin
  SetSocketOptions:=fpSetSockOpt(Sock,Level,OptName,@OptVal,OptLen);
end;

Function GetSocketOptions(Sock,Level,OptName:Longint;Var OptVal;Var optlen:longint):Longint;
begin
  GetSocketOptions:=so32dll.GetSockOpt(Sock,Level,OptName,OptVal,OptLen);
end;

Function SocketPair(Domain,SocketType,Protocol:Longint;var Pair:TSockArray):Longint;
begin
{!!TODO!!
  SocketPair:=so32dll.socketpair(Domain,SocketType,Protocol,Pair);}
  //SocketCall(Socket_Sys_SocketPair,Domain,SocketType,Protocol,longint(@Pair),0,0);
  SocketPair:=-1;
end;

{ mimic the linux fpWrite/fpRead calls for the file/text socket wrapper }
function fpWrite(handle : longint;Const bufptr;size : dword) : dword;
begin
  fpWrite := dword(fpsend(handle, @bufptr, size, 0));
  if fpWrite = dword(-1) then
    fpWrite := 0;
end;

function fpRead(handle : longint;var bufptr;size : dword) : dword;
var
  d : dword;
begin
  d:=dword(so32dll.os2_ioctl(handle,FIONREAD,d,SizeOf(d)));
  if d=dword(-1) then
   fpRead:=0
  else
   begin
    if size>d then
      size:=d;
    fpRead := dword(so32dll.recv(handle, bufptr, size, 0));
    if fpRead = dword(-1) then
     fpRead := 0
   end;
end;

{$i sockets.inc}

function fpsocket       (domain:cint; xtype:cint; protocol: cint):cint;
begin
  fpSocket:=so32dll.Socket(Domain,xtype,ProtoCol);
end;

function fpsend (s:cint; msg:pointer; len:size_t; flags:cint):ssize_t;
begin
  fpSend:=so32dll.Send(S,msg^,len,flags);
end;

function fpsendto (s:cint; msg:pointer; len:size_t; flags:cint; tox :psockaddr; tolen: tsocklen):ssize_t;
begin
  // Dubious construct, this should be checked. (IPV6 fails ?)
  fpSendTo:=so32dll.SendTo(S,msg^,Len,Flags,so32dll.SockAddr(tox^),toLen);
end;

function fprecv         (s:cint; buf: pointer; len: size_t; flags: cint):ssize_t;
begin
  fpRecv:=so32dll.Recv(S,Buf,Len,Flags);
end;

function fprecvfrom    (s:cint; buf: pointer; len: size_t; flags: cint; from : psockaddr; fromlen : psocklen):ssize_t;
begin
  fpRecvFrom:=so32dll.RecvFrom(S,Buf,Len,Flags,so32dll.SockAddr(from^),FromLen^);
end;

function fpconnect     (s:cint; name  : psockaddr; namelen : tsocklen):cint;
begin
  fpConnect:=so32dll.Connect(S,so32dll.SockAddr(name^),nameLen);
end;

function fpshutdown     (s:cint; how:cint):cint;
begin
  fpShutDown:=so32dll.ShutDown(S,How);
end;

function fpbind (s:cint; addrx : psockaddr; addrlen : tsocklen):cint;
begin
  fpbind:=so32dll.Bind(S,so32dll.SockAddr(Addrx^),AddrLen);
end;

function fplisten      (s:cint; backlog : cint):cint;
begin
  fplisten:=so32dll.Listen(S,backlog);
end;

function fpaccept      (s:cint; addrx : psockaddr; addrlen : psocklen):cint;
begin
  fpAccept:=so32dll.Accept(S,so32dll.SockAddr(Addrx^),longint(@AddrLen));
end;

function fpgetsockname (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpGetSockName:=so32dll.GetSockName(S,so32dll.SockAddr(name^),nameLen^);
end;

function fpgetpeername (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpGetPeerName:=so32dll.GetPeerName(S,so32dll.SockAddr(name^),NameLen^);
end;

function fpgetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : psocklen):cint;
begin
  fpGetSockOpt:=so32dll.GetSockOpt(S,Level,OptName,OptVal,OptLen^);
end;

function fpsetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen :tsocklen):cint;
begin
  fpSetSockOpt:=so32dll.SetSockOpt(S,Level,OptName,OptVal,OptLen);
end;

function fpsocketpair  (d:cint; xtype:cint; protocol:cint; sv:pcint):cint;
begin
  fpsocketpair:=-1;
end;

Function CloseSocket(Sock:Longint):Longint;
begin
  CloseSocket:=so32dll.soclose (Sock);
end;


Begin
  so32dll.sock_init;
End.
