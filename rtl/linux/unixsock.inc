{
   This file is part of the Free Pascal run time library.
   (c) 2004 by Marco van de Voort
   member of the Free Pascal development team.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   socket call implementations for Linux

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

{$if not defined(cpux86_64) and not defined(NO_SYSCALL_SOCKETCALL)}
  {$define NEED_SOCKETCALL}
{$endif}

{******************************************************************************
                          Basic Socket Functions
******************************************************************************}

{$ifdef NEED_SOCKETCALL}

Const
  {
    Arguments to the Linux Kernel system call for sockets. All
    Socket Connected calls go through the same system call,
    with an extra argument to determine what action to take.
  }
  Socket_Sys_SOCKET      = 1;
  Socket_Sys_BIND        = 2;
  Socket_Sys_CONNECT     = 3;
  Socket_Sys_LISTEN      = 4;
  Socket_Sys_ACCEPT      = 5;
  Socket_Sys_GETSOCKNAME = 6;
  Socket_Sys_GETPEERNAME = 7;
  Socket_Sys_SOCKETPAIR  = 8;
  Socket_Sys_SEND        = 9;
  Socket_Sys_RECV        = 10;
  Socket_Sys_SENDTO      = 11;
  Socket_Sys_RECVFROM    = 12;
  Socket_Sys_SHUTDOWN    = 13;
  Socket_Sys_SETSOCKOPT  = 14;
  Socket_Sys_GETSOCKOPT  = 15;
  Socket_Sys_SENDMSG     = 16;
  Socket_Sys_RECVMSG     = 17;


Function SocketCall(SockCallNr,a1,a2,a3,a4,a5,a6:TSysParam):cint; inline;
var
  Args:array[1..6] of TSysParam;
begin
  args[1]:=a1;
  args[2]:=a2;
  args[3]:=a3;
  args[4]:=a4;
  args[5]:=a5;
  args[6]:=a6;
  SocketCall:=do_Syscall(syscall_nr_socketcall,sockcallnr,TSysParam(@args));
  internal_socketerror:=fpgeterrno;
end;


function SocketCall(SockCallNr,a1,a2,a3:TSysParam):cint;inline;
begin
  SocketCall:=SocketCall(SockCallNr,a1,a2,a3,0,0,0);
end;

function  fpsocket (domain:cint; xtype:cint; protocol: cint):cint;
begin
  fpSocket:=SocketCall(Socket_Sys_socket,Domain,xtype,Protocol);
end;

function  fpsend (s:cint; msg:pointer; len:size_t; flags:cint):ssize_t;
begin
  fpSend:=SocketCall(Socket_Sys_sendto,S,TSysParam(msg),Len,Flags,0,0);
end;

function  fpsendto (s:cint; msg:pointer; len:size_t; flags:cint; tox :psockaddr; tolen: tsocklen):ssize_t;
begin
  fpSendto:=SocketCall(Socket_Sys_sendto,S,TSysParam(msg),Len,Flags,TSysParam(tox),tolen);
end;

function  fprecv (s:cint; buf: pointer; len: size_t; flags:cint):ssize_t;
begin
  fpRecv:=SocketCall(Socket_Sys_Recvfrom,S,tsysparam(buf),len,flags,0,0);
end;

function  fprecvfrom (s:cint; buf: pointer; len: size_t; flags: cint; from : psockaddr; fromlen : psocklen):ssize_t;
begin
  fpRecvFrom:=SocketCall(Socket_Sys_Recvfrom,S,TSysParam(buf),len,flags,TSysParam(from),TSysParam(fromlen));
end;

function  fpbind (s:cint; addrx : psockaddr; addrlen : tsocklen):cint;
begin
  fpBind:=SocketCall(Socket_Sys_Bind,S,TSysParam(addrx),addrlen);
end;

function  fplisten (s:cint; backlog : cint):cint;
begin
  fpListen:=SocketCall(Socket_Sys_Listen,S,backlog,0);
end;

function  fpaccept (s:cint; addrx : psockaddr; addrlen : psocklen):cint;
begin
  fpAccept:=SocketCall(Socket_Sys_accept,S,TSysParam(addrx),TSysParam(addrlen));
end;

function  fpconnect (s:cint; name  : psockaddr; namelen : tsocklen):cint;
begin
  fpConnect:=SocketCall(Socket_Sys_connect,S,TSysParam(name),namelen);
end;

function  fpshutdown (s:cint; how:cint):cint;
begin
  fpShutDown:=SocketCall(Socket_Sys_shutdown,S,how,0);
end;

function  fpgetsockname (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpGetSockName:=SocketCall(Socket_Sys_GetSockName,S,TSysParam(name),TSysParam(namelen));
end;

function  fpgetpeername (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpGetPeerName:=SocketCall(Socket_Sys_GetPeerName,S,TSysParam(name),TSysParam(namelen));
end;

function  fpsetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : tsocklen):cint;
begin
  fpSetSockOpt:=SocketCall(Socket_Sys_SetSockOpt,S,level,optname,TSysParam(optval),optlen,0);
end;

function  fpgetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : psocklen):cint;
begin
  fpGetSockOpt:=SocketCall(Socket_Sys_GetSockOpt,S,level,TSysParam(optname),TSysParam(optval),TSysParam(optlen),0);
end;

function  fpsocketpair (d:cint; xtype:cint; protocol:cint; sv:pcint):cint;
begin
  fpSocketPair:=SocketCall(Socket_Sys_SocketPair,d,xtype,protocol,TSysParam(sv),0,0);
end;

{$else NEED_SOCKETCALL}

function  fpsocket (domain:cint; xtype:cint; protocol: cint):cint;
begin
  fpSocket:=do_syscall(syscall_nr_socket,Domain,xtype,Protocol);
  internal_socketerror:=fpgeterrno;
end;

function  fpsend (s:cint; msg:pointer; len:size_t; flags:cint):ssize_t;
begin
  fpSend:=do_syscall(syscall_nr_sendto,S,TSysParam(msg),Len,Flags,0,0);
  internal_socketerror:=fpgeterrno;
end;

function  fpsendto (s:cint; msg:pointer; len:size_t; flags:cint; tox :psockaddr; tolen: tsocklen):ssize_t;
begin
  fpSendto:=do_syscall(syscall_nr_sendto,S,TSysParam(msg),Len,Flags,TSysParam(tox),tolen);
  internal_socketerror:=fpgeterrno;
end;

function  fprecv (s:cint; buf: pointer; len: size_t; flags:cint):ssize_t;
begin
  fpRecv:=do_syscall(syscall_nr_Recvfrom,S,tsysparam(buf),len,flags,0,0);
  internal_socketerror:=fpgeterrno;
end;

function  fprecvfrom (s:cint; buf: pointer; len: size_t; flags: cint; from : psockaddr; fromlen : psocklen):ssize_t;
begin
  fpRecvFrom:=do_syscall(syscall_nr_Recvfrom,S,TSysParam(buf),len,flags,TSysParam(from),TSysParam(fromlen));
  internal_socketerror:=fpgeterrno;
end;

function  fpbind (s:cint; addrx : psockaddr; addrlen : tsocklen):cint;
begin
  fpBind:=do_syscall(syscall_nr_Bind,S,TSysParam(addrx),addrlen);
  internal_socketerror:=fpgeterrno;
end;

function  fplisten (s:cint; backlog : cint):cint;
begin
  fpListen:=do_syscall(syscall_nr_Listen,S,backlog);
  internal_socketerror:=fpgeterrno;
end;

function  fpaccept (s:cint; addrx : psockaddr; addrlen : psocklen):cint;
begin
  fpAccept:=do_syscall(syscall_nr_accept,S,TSysParam(addrx),TSysParam(addrlen));
  internal_socketerror:=fpgeterrno;
end;

function  fpconnect (s:cint; name  : psockaddr; namelen : tsocklen):cint;
begin
  fpConnect:=do_syscall(syscall_nr_connect,S,TSysParam(name),namelen);
  internal_socketerror:=fpgeterrno;
end;

function  fpshutdown (s:cint; how:cint):cint;
begin
  fpShutDown:=do_syscall(syscall_nr_shutdown,S,how);
  internal_socketerror:=fpgeterrno;
end;

function  fpgetsockname (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpGetSockName:=do_syscall(syscall_nr_GetSockName,S,TSysParam(name),TSysParam(namelen));
  internal_socketerror:=fpgeterrno;
end;

function  fpgetpeername (s:cint; name  : psockaddr; namelen : psocklen):cint;
begin
  fpGetPeerName:=do_syscall(syscall_nr_GetPeerName,S,TSysParam(name),TSysParam(namelen));
  internal_socketerror:=fpgeterrno;
end;

function  fpsetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : tsocklen):cint;
begin
  fpSetSockOpt:=do_syscall(syscall_nr_SetSockOpt,S,level,optname,TSysParam(optval),optlen);
  internal_socketerror:=fpgeterrno;
end;

function  fpgetsockopt  (s:cint; level:cint; optname:cint; optval:pointer; optlen : psocklen):cint;
begin
  fpGetSockOpt:=do_syscall(syscall_nr_GetSockOpt,S,level,TSysParam(optname),TSysParam(optval),TSysParam(optlen));
  internal_socketerror:=fpgeterrno;
end;

function  fpsocketpair (d:cint; xtype:cint; protocol:cint; sv:pcint):cint;
begin
  fpSocketPair:=do_syscall(syscall_nr_SocketPair,d,xtype,protocol,TSysParam(sv));
  internal_socketerror:=fpgeterrno;
end;

{$endif NEED_do_syscall}

