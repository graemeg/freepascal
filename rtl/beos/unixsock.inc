{
   This file is part of the Free Pascal run time library.
   (c) 2000-2003 by Marco van de Voort
   member of the Free Pascal development team.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   socket call implementations for FreeBSD

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

{******************************************************************************
                          Basic Socket Functions
******************************************************************************}

function  fpsocket (domain:cint; xtype:cint; protocol: cint):cint;
Begin
  fpSocket:=Do_Syscall(syscall_nr_socket,Domain,xtype,Protocol);
  internal_socketerror:=fpgeterrno;
End;

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

