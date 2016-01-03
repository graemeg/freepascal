unit network;
{$mode objfpc} 
{$J+}
{$INLINE ON}
{$MACRO ON}
{$ASSERTIONS ON}

interface

uses
  ctypes, gctypes;

const
  INVALID_SOCKET = ( not 0 );
  SOCKET_ERROR = ( - 1 );
  SOCK_STREAM = 1;
  SOCK_DGRAM = 2;
  SOCK_RAW = 3;
  SO_DEBUG = $0001;  (* turn on debugging info recording  *)
  SO_ACCEPTCONN = $0002;  (* socket has had listen()  *)
  SO_REUSEADDR = $0004;  (* allow local address reuse  *)
  SO_KEEPALIVE = $0008;  (* keep connections alive  *)
  SO_DONTROUTE = $0010;  (* just use interface addresses  *)
  SO_BROADCAST = $0020;  (* permit sending of broadcast msgs  *)
  SO_USELOOPBACK = $0040;  (* bypass hardware when possible  *)
  SO_LINGER = $0080;  (* linger on close if data present  *)
  SO_OOBINLINE = $0100;  (* leave received OOB data in line  *)
  SO_REUSEPORT = $0200;  (* allow local address & port reuse  *)
  SO_DONTLINGER = ( not SO_LINGER );
  SO_SNDBUF = $1001;  (* send buffer size  *)
  SO_RCVBUF = $1002;  (* receive buffer size  *)
  SO_SNDLOWAT = $1003;  (* send low-water mark  *)
  SO_RCVLOWAT = $1004;  (* receive low-water mark  *)
  SO_SNDTIMEO = $1005;  (* send timeout  *)
  SO_RCVTIMEO = $1006;  (* receive timeout  *)
  SO_ERROR = $1007;  (* get error status and clear  *)
  SO_TYPE = $1008;  (* get socket type  *)

type
  linger = record
    l_onoff : cint;  (* option on/off  *)
    l_linger : cint;  (* linger time  *)
  end;

const
  SOL_SOCKET = $ffff;  (* options for socket level  *)
  AF_UNSPEC = 0;
  AF_INET = 2;
  PF_INET = AF_INET;
  PF_UNSPEC = AF_UNSPEC;
  IPPROTO_IP = 0;
  IPPROTO_TCP = 6;
  IPPROTO_UDP = 17;
  INADDR_ANY = 0;
  INADDR_BROADCAST = $ffffffff;
  MSG_DONTWAIT = $40;  (* Nonblocking i/o for this operation only  *)
  IP_TOS = 1;
  IP_TTL = 2;
  IPTOS_TOS_MASK = $1E;

function IPTOS_TOS(tos: longint): longint; inline;

const
  IPTOS_LOWDELAY = $10;
  IPTOS_THROUGHPUT = $08;
  IPTOS_RELIABILITY = $04;
  IPTOS_LOWCOST = $02;
  IPTOS_MINCOST = IPTOS_LOWCOST;
  IPTOS_PREC_MASK = $e0;

function IPTOS_PREC(tos: longint): longint; inline;

const
  IPTOS_PREC_NETCONTROL = $e0;
  IPTOS_PREC_INTERNETCONTROL = $c0;
  IPTOS_PREC_CRITIC_ECP = $a0;
  IPTOS_PREC_FLASHOVERRIDE = $80;
  IPTOS_PREC_FLASH = $60;
  IPTOS_PREC_IMMEDIATE = $40;
  IPTOS_PREC_PRIORITY = $20;
  IPTOS_PREC_ROUTINE = $00;


{$if not defined(FIONREAD) or not defined(FIONBIO)}
const
  IOCPARM_MASK    = $7f;
  IOC_VOID        = $20000000;
  IOC_OUT         = $40000000;
  IOC_IN          = $80000000;
  IOC_INOUT       = (IOC_IN or IOC_OUT);

function _IO(x, y: cardinal): cardinal; inline;
function _IOR(x, y, t: cardinal): cardinal; inline;
function _IOW(x, y, t: cardinal): cardinal; inline;
{$endif}

{$ifndef FIONREAD}
  {$define FIONREAD := _IOR(ord('f'), 127, culong);}
{$endif}

{$ifndef FIONBIO}
  {$define FIONBIO := _IOW(ord('f'), 126, culong);}
{$endif}


{$ifndef SIOCSHIWAT}
  {$define SIOCSHIWAT  := _IOW(ord('s'),  0, High(culong))}
  {$define SIOCGHIWAT  := _IOR(ord('s'),  1, High(culong))}
  {$define SIOCSLOWAT  := _IOW(ord('s'),  2, High(culong))}
  {$define SIOCGLOWAT  := _IOR(ord('s'),  3, High(culong))}
  {$define SIOCATMARK  := _IOR(ord('s'),  7, High(culong))}
{$endif}

{$ifndef O_NONBLOCK}
  {$define O_NONBLOCK := &04000}
{$endif}

{$ifndef FD_SET}
  {$undef  FD_SETSIZE}
  {$define FD_SETSIZE := 16}
type

{$ifndef HAVE_IN_ADDR}
  {$define HAVE_IN_ADDR}
  in_addr = record
    s_addr: cuint32;
  end;
  pin_addr = ^in_addr;
{$endif}

  _fd_set = record
    fd_bits: array [0..((FD_SETSIZE + 7) div 8) - 1] of cuint8;
  end;
  P_fd_set = ^_fd_set;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  timeval = record
    tv_sec: cint32;
    tv_usec: cint32;
  end;
  Ptimeval = ^timeval;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure FD_SET(n: longint; var p: _fd_set); inline;
procedure FD_CLR(n: longint; var p: _fd_set); inline;
function FD_ISSET(n: longint; p: _fd_set): boolean; inline;
procedure FD_ZERO(var p: _fd_set); inline;
{$endif}

{$ifndef TCP_NODELAY}
  {$define	TCP_NODELAY := $01}
{$endif}

{$ifndef TCP_KEEPALIVE}
  {$define TCP_KEEPALIVE := $02}
{$endif}

{$ifndef socklen_t}
  {$define socklen_t := cuint32}
  {$define psocklen_t := pcuint32}
{$endif}

{$ifndef htons}
  {$define htons(x) := (x)}
{$endif}

{$ifndef ntohs}
  {$define ntohs(x) := (x)}
{$endif}

{$ifndef htonl}
  {$define htonl(x) := (x)}
{$endif}

{$ifndef ntohl}
  {$define ntohl(x) := (x)}
{$endif}

{$ifndef h_addr}
  {$define h_addr := h_addr_list[0]}
{$endif}

{$ifndef IP4_ADDR}
procedure IP4_ADDR(var ipaddr: in_addr; a,b,c,d: cuint32); inline;
function ip4_addr1(ipaddr: in_addr): cuint32; inline;
function ip4_addr2(ipaddr: in_addr): cuint32; inline;
function ip4_addr3(ipaddr: in_addr): cuint32; inline;
function ip4_addr4(ipaddr: in_addr): cuint32; inline;
{$endif}

const
  POLLIN				= $0001;
  POLLPRI				= $0002;
  POLLOUT				= $0004;
  POLLERR				= $0008;
  POLLHUP				= $0010;
  POLLNVAL			= $0020;

type
  sockaddr_in = record
    sin_len: cuint8;
    sin_family: cuint8;
    sin_port: cuint16;
    sin_addr: in_addr;
    sin_zero: array [0..7] of cint8;
  end;
  psockaddr_in = ^sockaddr_in;

  sockaddr = record
    sa_len: cuint8;
    sa_family: cuint8;
    sa_data: array [0..13] of cint8;
  end;
  psockaddr = ^sockaddr;

  hostent = record
    h_name: pcchar;
    h_aliases: ppcchar;
    h_addrtype: cuint16;
    h_length: cuint16;
    h_addr_list: ppcchar;
  end;
  phostent = ^hostent;

  pollsd = record
    socket: cint32;
    events: cuint32;
    revents: cuint32;
  end;
  ppollsd = ^pollsd;

function inet_addr(const cp: pcchar): cuint32; cdecl; external;
function inet_aton(const cp: pcchar; addr: pin_addr): cint8; cdecl; external;
function inet_ntoa(addr: in_addr): pcchar; cdecl; external;
function if_config(local_ip, netmask, gateway: pcchar; use_dhcp: cbool): cint32; cdecl; external;
function if_configex(local_ip, netmask, gateway: pin_addr; use_dhcp: cbool): cint32; cdecl; external;
function net_init(): cint32; cdecl; external;

{$ifdef HW_RVL}
type
  netcallback = function(result: cint32; usrdata: pointer): cint32; cdecl;

function net_init_async(cb: netcallback; usrdata: pointer): cint32; cdecl; external;
function net_get_status: cint32; cdecl; external;
procedure net_wc24cleanup; cdecl; external;
function net_get_mac_address(mac_buf: pointer): cint32; cdecl; external;
{$endif}

procedure net_deinit; cdecl; external;
function net_gethostip: cuint32; cdecl; external;
function net_socket(domain, type_, protocol: cuint32): cint32; cdecl; external;
function net_bind(s: cint32; name_: Psockaddr; namelen: socklen_t): cint32; cdecl; external;
function net_listen(s: cint32; backlog: cuint32): cint32; cdecl; external;
function net_accept(s: cint32; addr: Psockaddr; addrlen: Psocklen_t): cint32; cdecl; external;
function net_connect(s: cint32; par1: Psockaddr; par2: socklen_t): cint32; cdecl; external;
function net_write(s: cint32; data: pointer; size: cint32): cint32; cdecl; external;
function net_send(s: cint32; data: pointer; size: cint32; flags: cuint32): cint32; cdecl; external;
function net_sendto(s: cint32; data: pointer; len: cint32; flags: cuint32; to_: Psockaddr; tolen: socklen_t): cint32; cdecl; external;
function net_recv(s: cint32; mem: pointer; len: cint32; flags: cuint32): cint32; cdecl; external;
function net_recvfrom(s: cint32; mem: pointer; len: cint32; flags: cuint32; from: Psockaddr; fromlen: Psocklen_t): cint32; cdecl; external;
function net_read(s: cint32; mem: pointer; len: cint32): cint32; cdecl; external;
function net_close(s: cint32): cint32; cdecl; external;
function net_select(maxfdp1: cint32; readset, writeset, exceptset: P_fd_set; timeout: Ptimeval): cint32; cdecl; external;
function net_setsockopt(s: cint32; level, optname: cuint32; optval: pointer; optlen: socklen_t): cint32; cdecl; external;
function net_ioctl(s: cint32; cmd: cuint32; argp: pointer): cint32; cdecl; external;
function net_fcntl(s: cint32; cmd, flags: cuint32): cint32; cdecl; external;
function net_poll(sds: Ppollsd; nsds, timeout: cint32): cint32; cdecl; external;
function net_shutdown(s: cint32; how: cuint32): cint32; cdecl; external;
function net_gethostbyname(addrString: pcchar): Phostent; cdecl; external;

implementation

function IPTOS_TOS(tos: cint): cint; inline;
begin
   IPTOS_TOS:=tos and IPTOS_TOS_MASK;
end;

function IPTOS_PREC(tos: cint): cint; inline;
begin
   IPTOS_PREC:=tos and IPTOS_PREC_MASK;
end;

{$if defined(FIONREAD) or defined(FIONBIO)}
function _IO(x, y: cardinal): cardinal; inline;
begin
  Result := IOC_VOID or ((x shl 8) or y);
end;

function _IOR(x, y, t: cardinal): cardinal; inline;
begin
  result := IOC_OUT or ((clong(sizeof(t)) and IOCPARM_MASK) shl 16) or (x shl 8) or y;
end;

function _IOW(x, y, t: cardinal): cardinal; inline;
begin
  result := IOC_IN or ((clong(sizeof(t)) and IOCPARM_MASK) shl 16) or (x shl 8) or y;
end;
{$endif}


{$ifndef FD_SET}
procedure FD_SET(n: longint; var p: _fd_set); inline;
begin
  p.fd_bits[n div 8] :=  p.fd_bits[n div 8]  or (1 shl (n and 7));
end;

procedure FD_CLR(n: longint; var p: _fd_set); inline;
begin
  p.fd_bits[n div 8] :=  p.fd_bits[n div 8]  and not (1 shl (n and 7));
end;

function FD_ISSET(n: longint; p: _fd_set): boolean; inline;
begin
  result := (p.fd_bits[n div 8] and (1 shl (n and 7))) <> 0;
end;

procedure FD_ZERO(var p: _fd_set); inline;
begin
  memset(@p, 0, sizeof(p));
end;
{$endif}

{$ifndef IP4_ADDR}
procedure IP4_ADDR(var ipaddr: in_addr; a,b,c,d: cuint32); inline;
begin
  ipaddr.s_addr := {htonl}((cuint32(a and $ff)<<24) or (cuint32(b and $ff) shl 16) or (cuint32(c and $ff) shl 8) or cuint32(d and $ff));
end;

function ip4_addr1(ipaddr: in_addr): cuint32; inline;
begin
  result := cuint32({ntohl}(ipaddr.s_addr) shr 24) and $ff;
end;

function ip4_addr2(ipaddr: in_addr): cuint32; inline;
begin
  result := cuint32({ntohl}(ipaddr.s_addr) shr 16) and $ff;
end;

function ip4_addr3(ipaddr: in_addr): cuint32; inline;
begin
  result := cuint32({ntohl}(ipaddr.s_addr) shr 8) and $ff;
end;

function ip4_addr4(ipaddr: in_addr): cuint32; inline;
begin
  result := cuint32({ntohl}(ipaddr.s_addr)) and $ff;
end;
{$endif}

initialization

end.
