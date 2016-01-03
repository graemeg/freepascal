(*
  $Id: socket.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  DSWifi Project - socket emulation layer defines/prototypes (sys/socket.h)
  (C) 2005-2006 Stephen Stair - sgstair@akkit.org - http://www.akkit.org
  ****************************************************************************** 
  DSWifi Lib and test materials are licenced under the MIT open source licence:
  Copyright (c) 2005-2006 Stephen Stair
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of
  this software and associated documentation files (the "Software"), to deal in
  the Software without restriction, including without limitation the rights to
  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is furnished to do
  so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  ******************************************************************************
    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  Comments:

*)


{$ifdef NDS_INTERFACE}

const
//#include <sys/time.h>
  SOL_SOCKET  = $fff;    (* options for socket level *)
  SOL_TCP     = 6;       (* TCP level *)
  PF_UNSPEC		= 0;
  PF_INET			= 2;
  PF_INET6		= 10;

  AF_UNSPEC		= PF_UNSPEC;
  AF_INET			= PF_INET;
  AF_INET6		= PF_INET6;

  SOCK_STREAM		= 1;
  SOCK_DGRAM		= 2;

// need to sync FIO* values with commonly accepted ones sometime
  FIONBIO			= 1;
  FIONREAD		= 2;

  SOCKET_ERROR	= -1;

// send()/recv()/etc flags
// at present, only MSG_PEEK is implemented though.
  MSG_WAITALL		= $40000000;
  MSG_TRUNC		= $20000000;
  MSG_PEEK		= $10000000;
  MSG_OOB			= $08000000;
  MSG_EOR			= $04000000;
  MSG_DONTROUTE	= $02000000;
  MSG_CTRUNC		= $01000000;

// shutdown() flags:
  SHUT_RD			= 1;
  SHUT_WR			= 2;
  SHUT_RDWR		= 3;


  SO_DEBUG        = $0001;    (* turn on debugging info recording *)
  SO_ACCEPTCONN   = $0002;    (* socket has had listen() *)
  SO_REUSEADDR    = $0004;    (* allow local address reuse *)
  SO_KEEPALIVE    = $0008;    (* keep connections alive *)
  SO_DONTROUTE    = $0010;    (* just use interface addresses *)
  SO_BROADCAST    = $0020;    (* permit sending of broadcast msgs *)
  SO_USELOOPBACK  = $0040;    (* bypass hardware when possible *)
  SO_LINGER       = $0080;    (* linger on close if data present *)
  SO_OOBINLINE    = $0100;    (* leave received OOB data in line *)
  SO_REUSEPORT    = $0200;    (* allow local address & port reuse *)
 	  	 
  SO_DONTLINGER   = not SO_LINGER;
 	  	 
  SO_SNDBUF    = $1001;    (* send buffer size *)
  SO_RCVBUF    = $1002;    (* receive buffer size *)
  SO_SNDLOWAT  = $1003;    (* send low-water mark *)
  SO_RCVLOWAT  = $1004;    (* receive low-water mark *)
  SO_SNDTIMEO  = $1005;    (* send timeout *)
  SO_RCVTIMEO  = $1006;    (* receive timeout *)
  SO_ERROR     = $1007;    (* get error status and clear *)
  SO_TYPE      = $1008;    (* get socket type *)
type
  sockaddr = record
    _sa_family: cushort;
    sa_data: array [0..13] of char;
  end;
  psockaddr = ^sockaddr;


function socket(domain, _type, protocol: cint): cint; cdecl; external;
function bind(socket: cint; {const} addr: psockaddr; addr_len: cint): cint; cdecl; external;
function connect(socket: cint; {const} addr: psockaddr; addr_len: cint): cint; cdecl; external;
function send(socket: cint; {const} data: pointer; sendlength, flags: cint): cint; cdecl; external;
function recv(socket: cint; data: pointer; recvlength, flags: cint): cint; cdecl; external;
function sendto(socket: cint; {const} data: pointer; sendlength, flags: cint; const addr: psockaddr; addr_len: cint): cint; cdecl; external;
function recvfrom(socket: cint; data: pointer; recvlength, flags: cint; addr: psockaddr; addr_len: pcint): cint; cdecl; external;
function listen(socket, max_connections: cint): cint; cdecl; external;
function accept(socket: cint; addr: psockaddr; addr_len: pcint): cint; cdecl; external;
function shutdown(socket, shutdown_type: cint): cint; cdecl; external;
function closesocket(socket: cint): cint; cdecl; external;

function ioctl(socket: cint; cmd: clong; arg: pointer): cint; cdecl; external;

function setsockopt(socket, level, option_name: cint; const data: pointer; data_len: cint): cint; cdecl; external;
function getsockopt(socket, level, option_name: cint; data: pointer; data_len: pcint): cint; cdecl; external;

function getpeername(socket: cint; addr: psockaddr; addr_len: pcint): cint; cdecl; external;
function getsockname(socket: cint; addr: psockaddr; addr_len: pcint): cint; cdecl; external;

function gethostname(name: pcchar; {size_t} len: cuint): cint; cdecl; external;
function sethostname(const name: pcchar; {size_t} len: cuint): cint; cdecl; external;

function 	htons(num: cushort): cushort; cdecl; external;
function 	htonl(num: culong): culong; cdecl; external;

function ntohs(num: cushort): cushort; inline;
function ntohl(num: culong): culong; inline;


type
  fd_set = array [0..7] of clong;
  pfd_set = ^fd_set;

  timeval = packed record
    sec: clong;
    usec: clong;
  end;
  ptimeval = ^timeval;


function select(nfds: cint; readfds, writefds, errorfds: pfd_set; timeout: ptimeval): cint; cdecl; external;
{$endif NDS_INTERFACE}


{$ifdef NDS_IMPLEMENTATION}
function ntohs(num: cushort): cushort; inline;
begin
  ntohs := htons(num);
end;

function ntohl(num: culong): culong; inline;
begin
  ntohl := htonl(num);
end;

{$endif NDS_IMPLEMENTATION}
