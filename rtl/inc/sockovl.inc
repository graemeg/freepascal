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

{$ifdef legacysocket}
Function socket(Domain,SocketType,Protocol:Longint):Longint;
begin
  Socket:=fpsocket(domain,sockettype,protocol);
end;


Function Send(Sock:Longint;Const Buf;BufLen,Flags:Longint):Longint;
begin
  Send:=fpsend(sock,@buf,buflen,flags);
end;

Function SendTo(Sock:Longint;Const Buf;BufLen,Flags:Longint;Var Addr; AddrLen : Longint):Longint;
begin
  Sendto:=fpsendto(sock,@buf,buflen,flags,@addr,addrlen);
end;

Function Recv(Sock:Longint;Var Buf;BufLen,Flags:Longint):Longint;
begin
  Recv:=fprecv(sock,@buf,buflen,flags);
end;

Function RecvFrom(Sock : Longint; Var Buf; Buflen,Flags : Longint; Var Addr ; var AddrLen :longint) : longint;

begin
  RecvFrom:=fprecvfrom(Sock,@buf,buflen,flags,@Addr,psocklen(@AddrLen));
end;

Function Bind(Sock:Longint;Const Addr;AddrLen:Longint):Boolean;
begin
  Bind:=fpbind(Sock,@Addr,AddrLen)=0;
end;

Function Listen(Sock,MaxConnect:Longint):Boolean;
begin
  Listen:=fplisten(sock,maxconnect)=0;
end;

Function Accept(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;
begin
  Accept:=fpaccept(sock,@addr,psocklen(@addrlen));
end;

Function Connect(Sock:Longint;Const Addr;Addrlen:Longint): boolean;

begin
  Connect:=fpConnect(sock,@addr,addrlen)=0;
end;

Function Shutdown(Sock:Longint;How:Longint):Longint;
begin
  ShutDown:=fpshutdown(sock,how);
end;

Function GetSocketName(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;
begin
  GetSocketName:=fpgetsockname(sock,@addr,psocklen(@addrlen));
end;

Function GetPeerName(Sock:Longint;Var Addr;Var Addrlen:Longint):Longint;
begin
  GetPeerName:=fpgetPeerName(sock,@addr,psocklen(@addrlen));
end;

Function SetSocketOptions(Sock,Level,OptName:Longint;const OptVal;optlen:longint):Longint;
begin
  SetSocketOptions:=fpsetsockopt(Sock,Level,OptName,pointer(@OptVal),optlen);
end;

Function GetSocketOptions(Sock,Level,OptName:Longint;Var OptVal;Var optlen:longint):Longint;
begin
  GetSocketOptions:=fpgetsockopt(Sock,Level,OptName,@OptVal,psocklen(@OptLen));
end;

Function SocketPair(Domain,SocketType,Protocol:Longint;var Pair:TSockArray):Longint;
begin
  SocketPair:=fpsocketpair(domain,sockettype,protocol,@pair[1]);
end;
{$endif}

{******************************************************************************
                               UnixSock
******************************************************************************}

Procedure Str2UnixSockAddr(const addr:string;var t:TUnixSockAddr;var len:longint);
begin
  Move(Addr[1],t.Path,length(Addr));
  t.Family:=AF_UNIX;
  t.Path[length(Addr)]:=#0;
  Len:=Length(Addr)+3;
end;


Function Bind(Sock:longint;const addr:string):boolean;
var
  UnixAddr : TUnixSockAddr;
  AddrLen  : longint;
begin
  Str2UnixSockAddr(addr,UnixAddr,AddrLen);
<<<<<<< HEAD
<<<<<<< HEAD
  Bind := fpBind(Sock, @UnixAddr, AddrLen)=0; // what the flying fuck were you thinking?
=======
  Bind := Bind(Sock, UnixAddr, AddrLen); // what the flying fuck were you thinking?
>>>>>>> graemeg/fixes_2_2
=======
  Bind := Bind(Sock, UnixAddr, AddrLen); // what the flying fuck were you thinking?
>>>>>>> origin/fixes_2_2
end;


Function DoAccept(Sock:longint;var addr:string):longint;
var
  UnixAddr : TUnixSockAddr;
  AddrLen  : longint;
begin
  AddrLen:=length(addr)+3;
  DoAccept:=fpAccept(Sock,@UnixAddr,@AddrLen);
  Move(UnixAddr.Path,Addr[1],AddrLen);
  SetLength(Addr,AddrLen);
end;



Function DoConnect(Sock:longint;const addr:string):Boolean;
var
  UnixAddr : TUnixSockAddr;
  AddrLen  : longint;
begin
  Str2UnixSockAddr(addr,UnixAddr,AddrLen);
  DoConnect:=fpConnect(Sock,@UnixAddr,AddrLen)=0;
end;

Function Accept(Sock:longint;var addr:string;var SockIn,SockOut:text):Boolean;
var
  s : longint;
begin
  S:=DoAccept(Sock,addr);
  if S>0 then
   begin
     Sock2Text(S,SockIn,SockOut);
     Accept:=true;
   end
  else
   Accept:=false;
end;

Function Accept(Sock:longint;var addr:string;var SockIn,SockOut:File):Boolean;
var
  s : longint;
begin
  S:=DoAccept(Sock,addr);
  if S>0 then
   begin
     Sock2File(S,SockIn,SockOut);
     Accept:=true;
   end
  else
   Accept:=false;
end;

Function Connect(Sock:longint;const addr:string;var SockIn,SockOut:text):Boolean;
begin
  Connect:=DoConnect(Sock,addr);
  If Connect then
     Sock2Text(Sock,SockIn,SockOut);
end;

Function Connect(Sock:longint;const addr:string;var SockIn,SockOut:file):Boolean;
begin
  Connect:=DoConnect(Sock,addr);
  if Connect then
     Sock2File(Sock,SockIn,SockOut);
end;

Function CloseSocket (Sock:Longint):Longint;
begin
  if fpclose(Sock)=0 then
    CloseSocket := 0 else
    CloseSocket := -1;
end;


