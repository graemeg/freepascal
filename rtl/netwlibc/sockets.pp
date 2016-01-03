{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2004 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}
unit Sockets;

Interface

{$macro on}
{$define maybelibc:=}

{$R-}

Uses
  winsock;

Type
  cushort=word;
  cuint8 =byte;
  cuint16=word;
  cuint32=cardinal;
  size_t =cuint32;
  ssize_t=cuint16;
  cint   =longint;
  pcint  =^cint;
  tsocklen=cint;
  psocklen=^tsocklen;

const
  EsockEINTR            = WSAEINTR;
  EsockEBADF            = WSAEBADF;
  EsockEFAULT           = WSAEFAULT;
  EsockEINVAL           = WSAEINVAL;
  EsockEACCESS         = WSAEACCES;
  EsockEMFILE          = WSAEMFILE;
  EsockEMSGSIZE        = WSAEMSGSIZE;
  EsockENOBUFS         = WSAENOBUFS;
  EsockENOTCONN        = WSAENOTCONN;
  EsockENOTSOCK        = WSAENOTSOCK;
  EsockEPROTONOSUPPORT = WSAEPROTONOSUPPORT;
  EsockEWOULDBLOCK     = WSAEWOULDBLOCK;

{$i netwsockh.inc}
{$i socketsh.inc}

Implementation

{ Include filerec and textrec structures }
{$i filerec.inc}
{$i textrec.inc}

{******************************************************************************
                          Basic Socket Functions
******************************************************************************}



//function fprecvmsg     (s:cint; msg: pmsghdr; flags:cint):ssize_t;
//function fpsendmsg    (s:cint; hdr: pmsghdr; flags:cint):ssize;

//function fpsocket     (domain:cint; xtype:cint; protocol: cint):cint;


function SocketError: cint;
begin
 SocketError := WSAGetLastError;
end;

function fpsocket       (domain:cint; xtype:cint; protocol: cint):cint;
begin
  fpSocket:=WinSock.Socket(Domain,xtype,ProtoCol);
end;

function fpsend (s:cint; msg:pointer; len:size_t; flags:cint):ssize_t;
begin
  fpSend:=WinSock.Send(S,msg,len,flags);
end;

function fpsendto (s:cint; msg:pointer; len:size_t; flags:cint; tox :psockaddr; tolen: tsocklen):ssize_t;
begin
  // Dubious construct, this should be checked. (IPV6 fails ?)
  fpSendTo:=WinSock.SendTo(S,msg,Len,Flags,Winsock.TSockAddr(tox^),toLen);
end;

function fprecv         (s:cint; buf: pointer; len: size_t; flags: cint):ssize_t;
begin
  fpRecv:=WinSock.Recv(S,Buf,Len,Flags);
end;

function fprecvfrom    (s:cint; buf: pointer; len: size_t; flags: cint; from : psockaddr; fromlen : psocklen):ssize_t;
begin
fpRecvFrom:=WinSock.RecvFrom(S,Buf,Len,Flags,Winsock.TSockAddr(from^),FromLen^);
end;

function fpconnect     (s:cint; name  : psockaddr; namelen : tsocklen):cint;
begin
  fpConnect:=WinSock.Connect(S,WinSock.TSockAddr(name^),nameLen);
end;

function fpshutdown     (s:cint; how:cint):cint;
begin
  fpShutDown:=WinSock.ShutDown(S,How);
end;

Function socket(Domain,SocketType,Protocol:Longint):Longint;
begin
  socket:=fpsocket(Domain,sockettype,protocol);
end;

Function Send(Sock:Longint;Const Buf;BufLen,Flags:Longint):Longint;
begin
  send:=fpsend(sock,@buf,buflen,flags);
end;

Function SendTo(Sock:Longint;Const Buf;BufLen,Flags:Longint;Var Addr; AddrLen : Longint):Longint;
begin
  sendto:=fpsendto(sock,@buf,buflen,flags,@addr,addrlen);
end;

Function Recv(Sock:Longint;Var Buf;BufLen,Flags:Longint):Longint;
begin
  Recv:=fpRecv(Sock,@Buf,BufLen,Flags);
end;

Function RecvFrom(Sock : Longint; Var Buf; Buflen,Flags : Longint; Var Addr; var AddrLen : longint) : longint;
begin
  RecvFrom:=fpRecvFrom(Sock,@Buf,BufLen,Flags,@Addr,@AddrLen);
end;

function fpbind (s:cint; addrx : psockaddr; addrlen : tsocklen):cint;
begin
  fpbind:=WinSock.Bind(S,WinSock.PSockAddr(Addrx),AddrLen);
end;

function fplisten      (s:cint; backlog : cint):cint;
begin
  fplisten:=WinSock.Listen(S,backlog);
end;

function fpaccept      (s:cint; addrx : psockaddr; addrlen : psocklen):cint;
begin
  fpAccept:=WinSock.Accept(S,WinSock.PSockAddr(Addrx),plongint(AddrLen));
end;

function fpgetsockname (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpGetSockName:=WinSock.GetSockName(S,WinSock.TSockAddr(name^),nameLen^);
end;

function fpgetpeername (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpGetPeerName:=WinSock.GetPeerName(S,WinSock.TSockAddr(name^),NameLen^);
end;

function fpgetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : psocklen):cint;
begin
  fpGetSockOpt:=WinSock.GetSockOpt(S,Level,OptName,OptVal,OptLen^);
end;

function fpsetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen :tsocklen):cint;
begin
  fpSetSockOpt:=WinSock.SetSockOpt(S,Level,OptName,OptVal,OptLen);
end;

function fpsocketpair  (d:cint; xtype:cint; protocol:cint; sv:pcint):cint;
begin
  fpSocketPair := -1;
end;

Function CloseSocket(Sock:Longint):Longint;
begin
  CloseSocket := Winsock.CloseSocket (Sock);
end;

Function Bind(Sock:Longint;Const Addr;AddrLen:Longint):Boolean;
begin
  bind:=fpBind(Sock,@Addr,AddrLen)=0;
end;

Function Listen(Sock,MaxConnect:Longint):Boolean;
begin
  Listen:=fplisten(Sock,MaxConnect)=0;
end;

Function Accept(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;
begin
  Accept:=FPAccept(sock,@addr,@addrlen);
end;

Function Shutdown(Sock:Longint;How:Longint):Longint;
begin
 shutdown:=fpshutdown(sock,how);
end;

Function Connect(Sock:Longint;Const Addr;Addrlen:Longint):Boolean;
begin
 connect:=fpconnect(sock,@addr,addrlen)=0;
end;

Function GetSocketName(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;
begin
 GetSocketName:=fpGetSockName(sock,@addr,@addrlen);
end;

Function GetPeerName(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;
begin
 GetPeerName:=fpGetPeerName(Sock,@addr,@addrlen);
end;

Function GetSocketOptions(Sock,Level,OptName:Longint;Var OptVal;Var optlen:longint):Longint;
begin
 GetSocketOptions:=fpGetSockOpt(sock,level,optname,@optval,@optlen);
end;

Function SetSocketOptions(Sock,Level,OptName:Longint;Const OptVal;optlen:longint):Longint;
begin
 SetSocketOptions:=fpsetsockopt(sock,level,optname,@optval,optlen);
end;

Function SocketPair(Domain,SocketType,Protocol:Longint;var Pair:TSockArray):Longint;
begin
  // SocketPair:=SocketCall(Socket_Sys_SocketPair,Domain,SocketType,Protocol,longint(@Pair),0,0);
  SocketPair := -1;
end;


{$ifdef unix}
{ mimic the linux fpWrite/fpRead calls for the file/text socket wrapper }
function fpWrite(handle : longint;Const bufptr;size : dword) : dword;
begin
  fpWrite := dword(WinSock.send(handle, bufptr, size, 0));
  if fpWrite = dword(SOCKET_ERROR) then
    fpWrite := 0;
end;

function fpRead(handle : longint;var bufptr;size : dword) : dword;
var
  d : dword;
begin
  if ioctlsocket(handle,FIONREAD,@d) = SOCKET_ERROR then
    begin
      fpRead:=0;
      exit;
    end;
  if d>0 then
    begin
      if size>d then
        size:=d;
      fpRead := dword(WinSock.recv(handle, bufptr, size, 0));
      if fpRead = dword(SOCKET_ERROR) then
        fpRead := 0;
    end;
  end;
{$else}
{ mimic the linux fpWrite/fpRead calls for the file/text socket wrapper }
function fpWrite(handle : longint;Const bufptr;size : dword) : dword;
begin
  fpWrite := dword(WinSock.send(handle, bufptr, size, 0));
  if fpWrite = dword(SOCKET_ERROR) then
    fpWrite := 0;
end;

function fpRead(handle : longint;var bufptr;size : dword) : dword;
var
  d : dword;
begin
  if ioctlsocket(handle,FIONREAD,@d) = SOCKET_ERROR then
    begin
      fpRead:=0;
      exit;
    end;
  if d>0 then
    begin
      if size>d then
        size:=d;
      fpRead := dword(WinSock.recv(handle, bufptr, size, 0));
      if fpRead = dword(SOCKET_ERROR) then
        fpRead := 0;
    end;
  end;
{$endif}

{$i sockets.inc}

{ winsocket stack needs an init. and cleanup code }
var
  wsadata : twsadata;

initialization
  WSAStartUp($2,wsadata);
finalization
  WSACleanUp;
end.
