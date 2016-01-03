{
    This file is part of the Free Pascal run time library.
    This unit contains the declarations for the Win32 Socket Library

    Copyright (c) 1999-2000 by Florian Klaempfl,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$ifndef NO_SMART_LINK}
{$smartlink on}
{$endif}
unit winsock;

  interface

    uses
       windows;

    const
       WINSOCK_VERSION = $0101;
       {
         Default maximium number of sockets.
         this does not
         mean that the underlying Windows Sockets implementation has to
         support that many!
       }
       FD_SETSIZE = 64;

    type
       tOS_INT  = LongInt;
       tOS_UINT = DWord;
       ptOS_INT = ^tOS_INT;
       ptOS_UINT = ^tOS_UINT;
       u_char = char;
       u_short = word;
       u_int = tOS_UINT;
       u_long = dword;
       pu_long = ^u_long;
       plongint = ^longint;
       TSocket = UINT_PTR;

       { there is already a procedure called FD_SET, so this
         record was renamed (FK) }
       fdset = record
          fd_count : u_int;
          fd_array : array[0..(FD_SETSIZE)-1] of TSocket;
       end;

       TFDSet = fdset;
       PFDSet = ^fdset;

       timeval = record
          tv_sec : longint;
          tv_usec : longint;
       end;

       TTimeVal = timeval;
       PTimeVal = ^TTimeVal;

       { found no reference to this type in c header files and here. AlexS }
       { minutes west of Greenwich  }
       { type of dst correction  }
       timezone = record
          tz_minuteswest : longint;
          tz_dsttime : longint;
       end;
       TTimeZone = timezone;
       PTimeZone = ^TTimeZone;

    const
       IOCPARM_MASK = $7f;
       IOC_VOID = $20000000;
       IOC_OUT = $40000000;
       IOC_IN = $80000000;
       IOC_INOUT = IOC_IN or IOC_OUT;
       FIONREAD =cardinal( IOC_OUT or
         ((4 and IOCPARM_MASK) shl 16) or
         (102 shl 8) or 127);
       FIONBIO = cardinal(IOC_IN or
         ((4 and IOCPARM_MASK) shl 16) or
         (102 shl 8) or 126);
       FIOASYNC     = cardinal(IOC_IN or
         ((4 and IOCPARM_MASK) shl 16) or
         (102 shl 8) or 125);
       {
         Structures returned by network data base library, taken from the
         BSD file netdb.h.  All addresses are supplied in host order, and
         returned in network order (suitable for use in system calls).
         Slight modifications for differences between Linux and winsock.h
      }
    type
       hostent = record
          { official name of host  }
          h_name: pchar;
          { alias list  }
          h_aliases: ^pchar;
          { host address type  }
          h_addrtype: SmallInt;
          { length of address  }
          h_length: SmallInt;
          { list of addresses  }
          case byte of
             0: (h_addr_list: ^pchar);
             1: (h_addr: ^pchar)
       end;
       THostEnt = hostent;
       PHostEnt = ^THostEnt;

       {
         Assumption here is that a network number
         fits in an unsigned long -- someday that won't be true!
       }
       netent = record
          { official name of net  }
          n_name : ^char;
          { alias list  }
          n_aliases : ^pchar;
          { net address type  }
          n_addrtype : SmallInt;
          n_pad1 : SmallInt;    { ensure right packaging }
          { network #  }
          n_net : u_long;
       end;
       TNetEnt = netent;
       PNetEnt = ^TNetEnt;

       servent = record
          { official service name  }
          s_name : ^char;
          { alias list  }
          s_aliases : ^pchar;
{$ifdef WIN64}
          { protocol to use  }
          s_proto : ^char;
          { port #  }
          s_port : SmallInt;
{$else WIN64}
          { port #  }
          s_port : SmallInt;
          n_pad1 : SmallInt;    { ensure right packaging }
          { protocol to use  }
          s_proto : ^char;
{$endif WIN64}
       end;
       TServEnt = servent;
       PServEnt = ^TServEnt;

       protoent = record
          { official protocol name  }
          p_name : ^char;
          { alias list  }
          p_aliases : ^pchar;
          { protocol #  }
          p_proto : SmallInt;
          p_pad1 : SmallInt;    { ensure packaging }
       end;
       TProtoEnt = protoent;
       PProtoEnt = ^TProtoEnt;

    const
       {
         Standard well-known IP protocols.
         For some reason there are differences between Linx and winsock.h
       }
       IPPROTO_IP = 0;
       IPPROTO_ICMP = 1;
       IPPROTO_IGMP = 2;
       IPPROTO_GGP = 3;
       IPPROTO_TCP = 6;
       IPPORT_ECHO = 7;
       IPPORT_DISCARD = 9;
       IPPORT_SYSTAT = 11;
       IPPROTO_PUP = 12;
       IPPORT_DAYTIME = 13;
       IPPORT_NETSTAT = 15;
       IPPROTO_UDP = 17;
       IPPROTO_IDP = 22;
       IPPROTO_ND = 77;
       IPPROTO_RAW = 255;
       IPPROTO_MAX = 256;
       IPPORT_FTP = 21;
       IPPORT_TELNET = 23;
       IPPORT_SMTP = 25;
       IPPORT_TIMESERVER = 37;
       IPPORT_NAMESERVER = 42;
       IPPORT_WHOIS = 43;
       IPPORT_MTP = 57;
       IPPORT_TFTP = 69;
       IPPORT_RJE = 77;
       IPPORT_FINGER = 79;
       IPPORT_TTYLINK = 87;
       IPPORT_SUPDUP = 95;
       IPPORT_EXECSERVER = 512;
       IPPORT_LOGINSERVER = 513;
       IPPORT_CMDSERVER = 514;
       IPPORT_EFSSERVER = 520;
       IPPORT_BIFFUDP = 512;
       IPPORT_WHOSERVER = 513;
       IPPORT_ROUTESERVER = 520;
       IPPORT_RESERVED = 1024;

    const
       IMPLINK_IP = 155;
       IMPLINK_LOWEXPER = 156;
       IMPLINK_HIGHEXPER = 158;

    type
       SunB = packed record
          s_b1,s_b2,s_b3,s_b4 : u_char;
       end;

       SunW = packed record
         s_w1,s_w2 : u_short;
       end;

       in_addr = record
          case integer of
             0 : (S_un_b : SunB);
             1 : (S_un_w : SunW);
             2 : (S_addr : u_long);
       end;
       TInAddr = in_addr;
       PInAddr = ^TInAddr;

       sockaddr_in = record
          case integer of
             0 : ( (* equals to sockaddr_in, size is 16 byte *)
                  sin_family : SmallInt;                (* 2 byte *)
                  sin_port : u_short;                   (* 2 byte *)
                  sin_addr : TInAddr;                   (* 4 byte *)
                  sin_zero : array[0..8-1] of char;     (* 8 byte *)
                 );
             1 : ((* equals to sockaddr, size is 16 byte *)
                  sa_family : Smallint; (* 2 byte *)
                  sa_data : array[0..14-1] of char;    (* 14 byte *)
                 );
       end;
       TSockAddrIn = sockaddr_in;
       PSockAddrIn = ^TSockAddrIn;
       TSockAddr = sockaddr_in;
       PSockAddr = ^TSockAddr;

    const
       INADDR_ANY = $00000000;
       INADDR_LOOPBACK = $7F000001;
       INADDR_BROADCAST = $FFFFFFFF;

       IN_CLASSA_NET = $ff000000;
       IN_CLASSA_NSHIFT = 24;
       IN_CLASSA_HOST = $00ffffff;
       IN_CLASSA_MAX = 128;
       IN_CLASSB_NET = $ffff0000;
       IN_CLASSB_NSHIFT = 16;
       IN_CLASSB_HOST = $0000ffff;
       IN_CLASSB_MAX = 65536;
       IN_CLASSC_NET = $ffffff00;
       IN_CLASSC_NSHIFT = 8;
       IN_CLASSC_HOST = $000000ff;
       INADDR_NONE = $ffffffff;

       WSADESCRIPTION_LEN = 256;
       WSASYS_STATUS_LEN = 128;

    type
       WSADATA = record
          wVersion : WORD;              { 2 byte, ofs 0 }
          wHighVersion : WORD;          { 2 byte, ofs 2 }
{$ifdef win64}
          iMaxSockets : word;           { 2 byte, ofs 390 }
          iMaxUdpDg : word;             { 2 byte, ofs 392 }
          lpVendorInfo : pchar;         { 4 byte, ofs 396 }
          szDescription : array[0..WSADESCRIPTION_LEN] of char; { 257 byte, ofs 4 }
          szSystemStatus : array[0..WSASYS_STATUS_LEN] of char; { 129 byte, ofs 261 }
{$else win64}
          szDescription : array[0..WSADESCRIPTION_LEN] of char; { 257 byte, ofs 4 }
          szSystemStatus : array[0..WSASYS_STATUS_LEN] of char; { 129 byte, ofs 261 }
          iMaxSockets : word;           { 2 byte, ofs 390 }
          iMaxUdpDg : word;             { 2 byte, ofs 392 }
          lpVendorInfo : pchar;         { 4 byte, ofs 396 }
{$endif win64}
       end;
       TWSAData = WSADATA;
       PWSAData = ^TWSAData;

    const
       IP_OPTIONS = 1;
       IP_MULTICAST_IF = 2;
       IP_MULTICAST_TTL = 3;
       IP_MULTICAST_LOOP = 4;
       IP_ADD_MEMBERSHIP = 5;
       IP_DROP_MEMBERSHIP = 6;
       IP_DEFAULT_MULTICAST_TTL = 1;
       IP_DEFAULT_MULTICAST_LOOP = 1;
       IP_MAX_MEMBERSHIPS = 20;

    type
       ip_mreq = record
            imr_multiaddr : in_addr;
            imr_interface : in_addr;
         end;

    {
       Definitions related to sockets: types, address families, options,
       taken from the BSD file sys/socket.h.
    }
    const
       INVALID_SOCKET = TSocket(not(1));
       SOCKET_ERROR = -1;
       SOCK_STREAM = 1;
       SOCK_DGRAM = 2;
       SOCK_RAW = 3;
       SOCK_RDM = 4;
       SOCK_SEQPACKET = 5;

      { For setsockoptions(2)  }
       SO_DEBUG = $0001;
       SO_ACCEPTCONN = $0002;
       SO_REUSEADDR = $0004;
       SO_KEEPALIVE = $0008;
       SO_DONTROUTE = $0010;
       SO_BROADCAST = $0020;
       SO_USELOOPBACK = $0040;
       SO_LINGER = $0080;
       SO_OOBINLINE = $0100;
       {
         Additional options.
       }
       { send buffer size  }
       SO_SNDBUF = $1001;
       { receive buffer size  }
       SO_RCVBUF = $1002;
       { send low-water mark  }
       SO_SNDLOWAT = $1003;
       { receive low-water mark  }
       SO_RCVLOWAT = $1004;
       { send timeout  }
       SO_SNDTIMEO = $1005;
       { receive timeout  }
       SO_RCVTIMEO = $1006;
       { get error status and clear  }
       SO_ERROR = $1007;
       { get socket type  }
       SO_TYPE = $1008;

       {
         Options for connect and disconnect data and options.  Used only by
         non-TCP/IP transports such as DECNet, OSI TP4, etc.
       }
       SO_CONNDATA = $7000;
       SO_CONNOPT = $7001;
       SO_DISCDATA = $7002;
       SO_DISCOPT = $7003;
       SO_CONNDATALEN = $7004;
       SO_CONNOPTLEN = $7005;
       SO_DISCDATALEN = $7006;
       SO_DISCOPTLEN = $7007;

       {
         Option for opening sockets for synchronous access.
       }
       SO_OPENTYPE = $7008;
       SO_SYNCHRONOUS_ALERT = $10;
       SO_SYNCHRONOUS_NONALERT = $20;

       {
         Other NT-specific options.
       }
       SO_MAXDG = $7009;
       SO_MAXPATHDG = $700A;
       SO_UPDATE_ACCEPT_CONTEXT = $700B;
       SO_CONNECT_TIME = $700C;

       {
         TCP options.
       }
       TCP_NODELAY = $0001;
       TCP_BSDURGENT = $7000;

       {
         Address families.
       }
       { unspecified  }
       AF_UNSPEC = 0;
       { local to host (pipes, portals)  }
       AF_UNIX = 1;
       { internetwork: UDP, TCP, etc.  }
       AF_INET = 2;
       { arpanet imp addresses  }
       AF_IMPLINK = 3;
       { pup protocols: e.g. BSP  }
       AF_PUP = 4;
       { mit CHAOS protocols  }
       AF_CHAOS = 5;
       { IPX and SPX  }
       AF_IPX = 6;
       { XEROX NS protocols  }
       AF_NS = 6;
       { ISO protocols  }
       AF_ISO = 7;
       { OSI is ISO  }
       AF_OSI = AF_ISO;
       { european computer manufacturers  }
       AF_ECMA = 8;
       { datakit protocols  }
       AF_DATAKIT = 9;
       { CCITT protocols, X.25 etc  }
       AF_CCITT = 10;
       { IBM SNA  }
       AF_SNA = 11;
       { DECnet  }
       AF_DECnet = 12;
       { Direct data link interface  }
       AF_DLI = 13;
       { LAT  }
       AF_LAT = 14;
       { NSC Hyperchannel  }
       AF_HYLINK = 15;
       { AppleTalk  }
       AF_APPLETALK = 16;
       { NetBios-style addresses  }
       AF_NETBIOS = 17;
       { VoiceView }
       AF_VOICEVIEW = 18;
       { FireFox }
       AF_FIREFOX = 19;
       { Somebody is using this! }
       AF_UNKNOWN1 = 20;
       { Banyan }
       AF_BAN = 21;

       AF_MAX = 22;

    type
       {
         Structure used by kernel to pass protocol
         information in raw sockets.
       }
       sockproto = record
          sp_family : u_short;
          sp_protocol : u_short;
       end;
       TSockProto = sockproto;
       PSockProto = ^TSockProto;

    const
       {
         Protocol families, same as address families for now.
       }
       PF_UNSPEC = AF_UNSPEC;
       PF_UNIX = AF_UNIX;
       PF_INET = AF_INET;
       PF_IMPLINK = AF_IMPLINK;
       PF_PUP = AF_PUP;
       PF_CHAOS = AF_CHAOS;
       PF_NS = AF_NS;
       PF_IPX = AF_IPX;
       PF_ISO = AF_ISO;
       PF_OSI = AF_OSI;
       PF_ECMA = AF_ECMA;
       PF_DATAKIT = AF_DATAKIT;
       PF_CCITT = AF_CCITT;
       PF_SNA = AF_SNA;
       PF_DECnet = AF_DECnet;
       PF_DLI = AF_DLI;
       PF_LAT = AF_LAT;
       PF_HYLINK = AF_HYLINK;
       PF_APPLETALK = AF_APPLETALK;
       PF_VOICEVIEW = AF_VOICEVIEW;
       PF_FIREFOX = AF_FIREFOX;
       PF_UNKNOWN1 = AF_UNKNOWN1;
       PF_BAN = AF_BAN;
       PF_MAX = AF_MAX;

    type
       {
         Structure used for manipulating linger option.
       }
       linger = record
          l_onoff : u_short;
          l_linger : u_short;
       end;
       TLinger = linger;
       PLinger = ^TLinger;

    const
       {
         Level number for (get/set)sockopt() to apply to socket itself.
       }
       { options for socket level  }
       SOL_SOCKET = $ffff;
       {
         Maximum queue length specifiable by listen.
       }
       SOMAXCONN = 5;
       { process out-of-band data  }
       MSG_OOB = $1;
       { peek at incoming message  }
       MSG_PEEK = $2;
       { send without using routing tables  }
       MSG_DONTROUTE = $4;
       MSG_MAXIOVLEN = 16;
       { partial send or recv for message xport  }
       MSG_PARTIAL = $8000;

       {
         Define constant based on rfc883, used by gethostbyxxxx() calls.
       }
       MAXGETHOSTSTRUCT = 1024;
       MAXHOSTNAMELEN = MAXGETHOSTSTRUCT;

       {
         Define flags to be used with the WSAAsyncSelect() call.
       }
       FD_READ = $01;
       FD_WRITE = $02;
       FD_OOB = $04;
       FD_ACCEPT = $08;
       FD_CONNECT = $10;
       FD_CLOSE = $20;

       {
         All Windows Sockets error constants are biased by WSABASEERR from
         the "normal"
       }
       WSABASEERR = 10000;

       {
         Windows Sockets definitions of regular Microsoft C error constants
       }
       WSAEINTR = WSABASEERR + 4;
       WSAEBADF = WSABASEERR + 9;
       WSAEACCES = WSABASEERR + 13;
       WSAEFAULT = WSABASEERR + 14;
       WSAEINVAL = WSABASEERR + 22;
       WSAEMFILE = WSABASEERR + 24;

       {
         Windows Sockets definitions of regular Berkeley error constants
       }
       WSAEWOULDBLOCK = WSABASEERR + 35;
       WSAEINPROGRESS = WSABASEERR + 36;
       WSAEALREADY = WSABASEERR + 37;
       WSAENOTSOCK = WSABASEERR + 38;
       WSAEDESTADDRREQ = WSABASEERR + 39;
       WSAEMSGSIZE = WSABASEERR + 40;
       WSAEPROTOTYPE = WSABASEERR + 41;
       WSAENOPROTOOPT = WSABASEERR + 42;
       WSAEPROTONOSUPPORT = WSABASEERR + 43;
       WSAESOCKTNOSUPPORT = WSABASEERR + 44;
       WSAEOPNOTSUPP = WSABASEERR + 45;
       WSAEPFNOSUPPORT = WSABASEERR + 46;
       WSAEAFNOSUPPORT = WSABASEERR + 47;
       WSAEADDRINUSE = WSABASEERR + 48;
       WSAEADDRNOTAVAIL = WSABASEERR + 49;
       WSAENETDOWN = WSABASEERR + 50;
       WSAENETUNREACH = WSABASEERR + 51;
       WSAENETRESET = WSABASEERR + 52;
       WSAECONNABORTED = WSABASEERR + 53;
       WSAECONNRESET = WSABASEERR + 54;
       WSAENOBUFS = WSABASEERR + 55;
       WSAEISCONN = WSABASEERR + 56;
       WSAENOTCONN = WSABASEERR + 57;
       WSAESHUTDOWN = WSABASEERR + 58;
       WSAETOOMANYREFS = WSABASEERR + 59;
       WSAETIMEDOUT = WSABASEERR + 60;
       WSAECONNREFUSED = WSABASEERR + 61;
       WSAELOOP = WSABASEERR + 62;
       WSAENAMETOOLONG = WSABASEERR + 63;
       WSAEHOSTDOWN = WSABASEERR + 64;
       WSAEHOSTUNREACH = WSABASEERR + 65;
       WSAENOTEMPTY = WSABASEERR + 66;
       WSAEPROCLIM = WSABASEERR + 67;
       WSAEUSERS = WSABASEERR + 68;
       WSAEDQUOT = WSABASEERR + 69;
       WSAESTALE = WSABASEERR + 70;
       WSAEREMOTE = WSABASEERR + 71;
       WSAEDISCON = WSABASEERR + 101;

       {
         Extended Windows Sockets error constant definitions
       }
       WSASYSNOTREADY = WSABASEERR + 91;
       WSAVERNOTSUPPORTED = WSABASEERR + 92;
       WSANOTINITIALISED = WSABASEERR + 93;
       {
         Error return codes from gethostbyname() and gethostbyaddr()
         (when using the resolver). Note that these errors are
         retrieved via WSAGetLastError() and must therefore follow
         the rules for avoiding clashes with error numbers from
         specific implementations or language run-time systems.
         For this reason the codes are based at WSABASEERR+1001.
         Note also that [WSA]NO_ADDRESS is defined only for
         compatibility purposes.
        }
       WSAHOST_NOT_FOUND = WSABASEERR + 1001;
       HOST_NOT_FOUND = WSAHOST_NOT_FOUND;
       { Non-Authoritative: Host not found, or SERVERFAIL  }
       WSATRY_AGAIN = WSABASEERR + 1002;
       TRY_AGAIN = WSATRY_AGAIN;

       { Non recoverable errors, FORMERR, REFUSED, NOTIMP  }
       WSANO_RECOVERY = WSABASEERR + 1003;
       NO_RECOVERY = WSANO_RECOVERY;

       { Valid name, no data record of requested type  }
       WSANO_DATA = WSABASEERR + 1004;
       NO_DATA = WSANO_DATA;

       { no address, look for MX record  }
       WSANO_ADDRESS = WSANO_DATA;
       NO_ADDRESS = WSANO_ADDRESS;

    const
       {
         Windows Sockets errors redefined as regular Berkeley error constants.
       }
       EWOULDBLOCK = WSAEWOULDBLOCK;
       EINPROGRESS = WSAEINPROGRESS;
       EALREADY = WSAEALREADY;
       ENOTSOCK = WSAENOTSOCK;
       EDESTADDRREQ = WSAEDESTADDRREQ;
       EMSGSIZE = WSAEMSGSIZE;
       EPROTOTYPE = WSAEPROTOTYPE;
       ENOPROTOOPT = WSAENOPROTOOPT;
       EPROTONOSUPPORT = WSAEPROTONOSUPPORT;
       ESOCKTNOSUPPORT = WSAESOCKTNOSUPPORT;
       EOPNOTSUPP = WSAEOPNOTSUPP;
       EPFNOSUPPORT = WSAEPFNOSUPPORT;
       EAFNOSUPPORT = WSAEAFNOSUPPORT;
       EADDRINUSE = WSAEADDRINUSE;
       EADDRNOTAVAIL = WSAEADDRNOTAVAIL;
       ENETDOWN = WSAENETDOWN;
       ENETUNREACH = WSAENETUNREACH;
       ENETRESET = WSAENETRESET;
       ECONNABORTED = WSAECONNABORTED;
       ECONNRESET = WSAECONNRESET;
       ENOBUFS = WSAENOBUFS;
       EISCONN = WSAEISCONN;
       ENOTCONN = WSAENOTCONN;
       ESHUTDOWN = WSAESHUTDOWN;
       ETOOMANYREFS = WSAETOOMANYREFS;
       ETIMEDOUT = WSAETIMEDOUT;
       ECONNREFUSED = WSAECONNREFUSED;
       ELOOP = WSAELOOP;
       ENAMETOOLONG = WSAENAMETOOLONG;
       EHOSTDOWN = WSAEHOSTDOWN;
       EHOSTUNREACH = WSAEHOSTUNREACH;
       ENOTEMPTY = WSAENOTEMPTY;
       EPROCLIM = WSAEPROCLIM;
       EUSERS = WSAEUSERS;
       EDQUOT = WSAEDQUOT;
       ESTALE = WSAESTALE;
       EREMOTE = WSAEREMOTE;

       TF_DISCONNECT = $01;
       TF_REUSE_SOCKET = $02;
       TF_WRITE_BEHIND = $04;

       {
         Options for use with [gs]etsockopt at the IP level.
       }
       IP_TTL = 7;
       IP_TOS = 8;
       IP_DONTFRAGMENT = 9;

    type
       _TRANSMIT_FILE_BUFFERS = record
          Head : Pointer;
          HeadLength : dword;
          Tail : Pointer;
          TailLength : dword;
       end;
       TRANSMIT_FILE_BUFFERS = _TRANSMIT_FILE_BUFFERS;
       TTransmitFileBuffers = _TRANSMIT_FILE_BUFFERS;
       PTransmitFileBuffers = ^TTransmitFileBuffers;

    { Socket function prototypes  }
    const
       winsockdll = 'wsock32.dll';

{
Winsock types all buffers as pchar (char *), modern POSIX does it the ANSI
C way with pointer (void *). If the pointer overloaded version doesn't exist,
a "pointer" will be passed to the "var" version. (bug 3142).
So if there are var/const versions:
- To keep ported unix code working, there must be "pointer" variants (ANSI)
- To keep Delphi/ported C Winsock code working there must be pchar variants
        (K&R)
IOW, there _must_ be 3 versions then: var/const, pchar and pointer}

    function accept(s:TSocket; addr: PSockAddr; addrlen : ptOS_INT) : TSocket;stdcall;external winsockdll name 'accept';
    function accept(s:TSocket; addr: PSockAddr; var addrlen : tOS_INT) : TSocket;stdcall;external winsockdll name 'accept';
    function bind(s:TSocket; addr: PSockaddr;namelen:tOS_INT):tOS_INT;stdcall;external winsockdll name 'bind';
    function bind(s:TSocket; const addr: TSockaddr;namelen:tOS_INT):tOS_INT;stdcall;external winsockdll name 'bind';
    function closesocket(s:TSocket):tOS_INT;stdcall;external winsockdll name 'closesocket';
    function connect(s:TSocket; addr:PSockAddr; namelen:tOS_INT):tOS_INT;stdcall;external winsockdll name 'connect';
    function connect(s:TSocket; Const name:TSockAddr; namelen:tOS_INT):tOS_INT;stdcall;external winsockdll name 'connect';
    function ioctlsocket(s:TSocket; cmd:longint; var arg:u_long):tOS_INT;stdcall;external winsockdll name 'ioctlsocket'; { really a c-long }
    function ioctlsocket(s:TSocket; cmd:longint; var arg:longint):tOS_INT;stdcall;external winsockdll name 'ioctlsocket'; { really a c-long }
    function ioctlsocket(s:TSocket; cmd:longint; argp:pu_long):tOS_INT;stdcall;external winsockdll name 'ioctlsocket'; { really a c-long }
    function getpeername(s:TSocket; var name:TSockAddr;var namelen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'getpeername';
    function getsockname(s:TSocket; var name:TSockAddr;var namelen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'getsockname';
    function getsockopt(s:TSocket; level:tOS_INT; optname:tOS_INT;optval:pchar;var optlen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'getsockopt';
    function getsockopt(s:TSocket; level:tOS_INT; optname:tOS_INT;optval:pointer;var optlen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'getsockopt';
    function getsockopt(s:TSocket; level:tOS_INT; optname:tOS_INT;var optval;var optlen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'getsockopt';
    function htonl(hostlong:u_long):u_long;stdcall;external winsockdll name 'htonl';
    function htons(hostshort:u_short):u_short;stdcall;external winsockdll name 'htons';
    function inet_addr(cp:pchar):cardinal;stdcall;external winsockdll name 'inet_addr';
    function inet_ntoa(i : TInAddr):pchar;stdcall;external winsockdll name 'inet_ntoa';
    function listen(s:TSocket; backlog:tOS_INT):tOS_INT;stdcall;external winsockdll name 'listen';
    function ntohl(netlong:u_long):u_long;stdcall;external winsockdll name 'ntohl';
    function ntohs(netshort:u_short):u_short;stdcall;external winsockdll name 'ntohs';
    function recv(s:TSocket;buf:pchar; len:tOS_INT; flags:tOS_INT):tOS_INT;stdcall;external winsockdll name 'recv';
    function recv(s:TSocket;buf:pointer; len:tOS_INT; flags:tOS_INT):tOS_INT;stdcall;external winsockdll name 'recv';
    function recv(s:TSocket;var buf; len:tOS_INT; flags:tOS_INT):tOS_INT;stdcall;external winsockdll name 'recv';
    function recvfrom(s:TSocket;buf:pchar; len:tOS_INT; flags:tOS_INT;from:PSockAddr; fromlen:ptOS_INT):tOS_INT;stdcall;
      external winsockdll name 'recvfrom';
    function recvfrom(s:TSocket;buf:pointer; len:tOS_INT; flags:tOS_INT;from:PSockAddr; fromlen:ptOS_INT):tOS_INT;stdcall;
      external winsockdll name 'recvfrom';
    function recvfrom(s:TSocket;var buf; len:tOS_INT; flags:tOS_INT;Const from:TSockAddr; var fromlen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'recvfrom';
    function select(nfds:tOS_INT; readfds,writefds,exceptfds : PFDSet;timeout: PTimeVal):tOS_INT;stdcall;
      external winsockdll name 'select';
    function send(s:TSocket;Const buf; len:tOS_INT; flags:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'send';
    function send(s:TSocket; buf:pchar; len:tOS_INT; flags:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'send';
    function send(s:TSocket;buf:pointer; len:tOS_INT; flags:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'send';
    function sendto(s:TSocket; buf:pchar; len:tOS_INT; flags:tOS_INT;toaddr:PSockAddr; tolen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'sendto';
    function sendto(s:TSocket; buf:pointer; len:tOS_INT; flags:tOS_INT;toaddr:PSockAddr; tolen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'sendto';
    function sendto(s:TSocket; Const buf; len:tOS_INT; flags:tOS_INT;Const toaddr:TSockAddr; tolen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'sendto';
    function setsockopt(s:TSocket; level:tOS_INT; optname:tOS_INT; optval:pchar; optlen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'setsockopt';
    function setsockopt(s:TSocket; level:tOS_INT; optname:tOS_INT;optval:pointer; optlen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'setsockopt';
    function setsockopt(s:TSocket; level:tOS_INT; optname:tOS_INT; Const optval; optlen:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'setsockopt';
    function shutdown(s:TSocket; how:tOS_INT):tOS_INT;stdcall;
      external winsockdll name 'shutdown';
    function socket(af:tOS_INT; t:tOS_INT; protocol:tOS_INT):TSocket;stdcall;
      external winsockdll name 'socket';

    { Database function prototypes  }
    function gethostbyaddr(addr:pchar; len:tOS_INT; t:tOS_INT): PHostEnt;stdcall;external winsockdll name 'gethostbyaddr';
    function gethostbyname(name:pchar):PHostEnt;stdcall;external winsockdll name 'gethostbyname';
    function gethostname(name:pchar; namelen:tOS_INT):tOS_INT;stdcall;external winsockdll name 'gethostname';
    function getservbyport(port:tOS_INT; proto:pchar):PServEnt;stdcall;external winsockdll name 'getservbyport';
    function getservbyname(name:pchar; proto:pchar):PServEnt;stdcall;external winsockdll name 'getservbyname';
    function getprotobynumber(proto:tOS_INT):PProtoEnt;stdcall;external winsockdll name 'getprotobynumber';
    function getprotobyname(name:pchar):PProtoEnt;stdcall;external winsockdll name 'getprotobyname';

    { Microsoft Windows Extension function prototypes  }
    function WSAStartup(wVersionRequired:word;var WSAData:TWSADATA):tOS_INT;stdcall;
      external winsockdll name 'WSAStartup';
    function WSACleanup:tOS_INT;stdcall;external winsockdll name 'WSACleanup';
    procedure WSASetLastError(iError:tOS_INT);stdcall;external winsockdll name 'WSASetLastError';
    function WSAGetLastError:tOS_INT;stdcall;external winsockdll name 'WSAGetLastError';
    function WSAIsBlocking:BOOL;stdcall;external winsockdll name 'WSAIsBlocking';
    function WSAUnhookBlockingHook:tOS_INT;stdcall;external winsockdll name 'WSAUnhookBlockingHook';
    function WSASetBlockingHook(lpBlockFunc:TFarProc):TFarProc;stdcall;external winsockdll name 'WSASetBlockingHook';
    function WSACancelBlockingCall:tOS_INT;stdcall;external winsockdll name 'WSACancelBlockingCall';
    function WSAAsyncGetServByName(hWnd:HWND; wMsg:u_int; name:pchar; proto:pchar; buf:pchar;
                                   buflen:tOS_INT):THandle;stdcall;external winsockdll name 'WSAAsyncGetServByName';
    function WSAAsyncGetServByPort(hWnd:HWND; wMsg:u_int; port:tOS_INT; proto:pchar; buf:pchar;
                                   buflen:tOS_INT):THandle;stdcall;external winsockdll name 'WSAAsyncGetServByPort';
    function WSAAsyncGetProtoByName(hWnd:HWND; wMsg:u_int; name:pchar; buf:pchar; buflen:tOS_INT):THandle;stdcall;
      external winsockdll name 'WSAAsyncGetProtoByName';
    function WSAAsyncGetProtoByNumber(hWnd:HWND; wMsg:u_int; number:tOS_INT; buf:pchar; buflen:tOS_INT):THandle;stdcall;
      external winsockdll name 'WSAAsyncGetProtoByNumber';
    function WSAAsyncGetHostByName(hWnd:HWND; wMsg:u_int; name:pchar; buf:pchar; buflen:tOS_INT):THandle;stdcall;
      external winsockdll name 'WSAAsyncGetHostByName';
    function WSAAsyncGetHostByAddr(hWnd:HWND; wMsg:u_int; addr:pchar; len:tOS_INT; t:tOS_INT;
                                   buf:pchar; buflen:tOS_INT):THandle;stdcall;
                                   external winsockdll name 'WSAAsyncGetHostByAddr';
    function WSACancelAsyncRequest(hAsyncTaskHandle:THandle):tOS_INT;stdcall;
      external winsockdll name 'WSACancelAsyncRequest';
    function WSAAsyncSelect(s:TSocket; hWnd:HWND; wMsg:u_int; lEvent:longint):tOS_INT;stdcall; { really a c-long }
      external winsockdll name 'WSAAsyncSelect';
    function WSARecvEx(s:TSocket;var buf; len:tOS_INT; flags:ptOS_INT):tOS_INT;stdcall;
      external winsockdll name 'WSARecvEx';
    function __WSAFDIsSet(s:TSocket; var FDSet:TFDSet):Bool;stdcall;
      external winsockdll name '__WSAFDIsSet';
    function __WSAFDIsSet_(s:TSocket; var FDSet:TFDSet):tOS_INT;stdcall;
      external winsockdll name '__WSAFDIsSet';
    function TransmitFile(hSocket:TSocket; hFile:THandle; nNumberOfBytesToWrite:dword;
                          nNumberOfBytesPerSend:DWORD; lpOverlapped:POverlapped;
                          lpTransmitBuffers:PTransmitFileBuffers; dwReserved:dword):Bool;stdcall;
                          external winsockdll name 'TransmitFile';

    function AcceptEx(sListenSocket,sAcceptSocket:TSocket;
                      lpOutputBuffer:Pointer; dwReceiveDataLength,dwLocalAddressLength,
                      dwRemoteAddressLength:dword; var lpdwBytesReceived:dword;
                      lpOverlapped:POverlapped):Bool;stdcall;
                      external winsockdll name 'AcceptEx';

    procedure GetAcceptExSockaddrs(lpOutputBuffer:Pointer;
                                   dwReceiveDataLength,dwLocalAddressLength,dwRemoteAddressLength:dword;
                                   var LocalSockaddr:PSockAddr; var LocalSockaddrLength:tOS_INT;
                                   var RemoteSockaddr:PSockAddr; var RemoteSockaddrLength:tOS_INT);stdcall;
                                   external winsockdll name 'GetAcceptExSockaddrs';

    function WSAMakeSyncReply(Buflen,Error:Word):dword;
    function WSAMakeSelectReply(Event,Error:Word):dword;
    function WSAGetAsyncBuflen(Param:dword):Word;
    function WSAGetAsyncError(Param:dword):Word;
    function WSAGetSelectEvent(Param:dword):Word;
    function WSAGetSelectError(Param:dword):Word;
    procedure FD_CLR(Socket:TSocket; var FDSet:TFDSet);
    function FD_ISSET(Socket:TSocket; var FDSet:TFDSet):Boolean;
    procedure FD_SET(Socket:TSocket; var FDSet:TFDSet);
    procedure FD_ZERO(var FDSet:TFDSet);

  implementation

    {
      Implementation of the helper routines
    }
    function WSAMakeSyncReply(Buflen,Error:Word):dword;

      begin
         WSAMakeSyncReply:=MakeLong(Buflen, Error);
      end;

    function WSAMakeSelectReply(Event,Error:Word):dword;

      begin
         WSAMakeSelectReply:=MakeLong(Event,Error);
      end;

    function WSAGetAsyncBuflen(Param:dword):Word;

      begin
         WSAGetAsyncBuflen:=lo(Param);
      end;

    function WSAGetAsyncError(Param:dword):Word;

      begin
         WSAGetAsyncError:=hi(Param);
      end;

    function WSAGetSelectEvent(Param:dword):Word;

      begin
         WSAGetSelectEvent:=lo(Param);
      end;

    function WSAGetSelectError(Param:dword):Word;

      begin
         WSAGetSelectError:=hi(Param);
      end;

    procedure FD_CLR(Socket:TSocket; var FDSet:TFDSet);

      var
         i : u_int;

      begin
         i:=0;
         while i<FDSet.fd_count do
           begin
              if FDSet.fd_array[i]=Socket then
                begin
                   while i<FDSet.fd_count-1 do
                     begin
                        FDSet.fd_array[i]:=FDSet.fd_array[i+1];
                        inc(i);
                     end;
                   dec(FDSet.fd_count);
                   break;
                end;
              inc(i);
           end;
      end;

    function FD_ISSET(Socket:TSocket; var FDSet:TFDSet):Boolean;

      begin
         FD_ISSET:=__WSAFDIsSet(Socket,FDSet);
      end;

    procedure FD_SET(Socket:TSocket; var FDSet:TFDSet);

      begin
         if FDSet.fd_count<FD_SETSIZE then
           begin
              FDSet.fd_array[FDSet.fd_count]:=Socket;
              Inc(FDSet.fd_count);
           end;
      end;

    procedure FD_ZERO(var FDSet:TFDSet);

      begin
         FDSet.fd_count:=0;
      end;

end.
