{       Winsock2.h -- definitions to be used with the WinSock 2 DLL and WinSock 2 applications.
        This header file corresponds to version 2.2.x of the WinSock API specification.
        This file includes parts which are Copyright (c) 1982-1986 Regents
        of the University of California. All rights reserved.
        The Berkeley Software License Agreement specifies the terms and
        conditions for redistribution. }

// converted by Alex Konshin, mailto:alexk@msmt.spb.su
// added FreePascal stuff: AlexS@freepage.de

unit WinSock2;

{$ifndef NO_SMART_LINK}
{$smartlink on}
{$endif}

interface

Uses Windows;

{       Define the current Winsock version. To build an earlier Winsock version
        application redefine this value prior to including Winsock2.h. }
Const
  WINSOCK_VERSION = $0202;
  WINSOCK2_DLL = 'ws2_32.dll';

Type
  u_char = Char;
  u_short = Word;
  u_int = DWord;
  u_long = DWord;
  pu_char = ^u_char;
  pu_short = ^u_short;
  pu_int = ^u_int;
  pu_long = ^u_long;

  TSocket = UINT_PTR;      { The new type to be used in all instances which refer to sockets. }

  WSAEVENT = THandle;
  PWSAEVENT = ^WSAEVENT;
  LPWSAEVENT = PWSAEVENT;
{$IFDEF UNICODE}
  PMBChar = PWideChar;
{$ELSE}
  PMBChar = PChar;
{$ENDIF}

const
  FD_SETSIZE     =   64;

type
  PFDSet = ^TFDSet;
  TFDSet = record
    fd_count: u_int;
    fd_array: array[0..FD_SETSIZE-1] of TSocket;
  end;
  fdset = TFDSet;

  PTimeVal = ^TTimeVal;
  TTimeVal = record
    tv_sec: Longint;
    tv_usec: Longint;
  end;
  timeval = TTimeVal;

       timezone = record
          tz_minuteswest : longint;
          tz_dsttime : longint;
       end;
       TTimeZone = timezone;
       PTimeZone = ^TTimeZone;

const
  IOCPARM_MASK = $7f;
  IOC_VOID     = $20000000;
  IOC_OUT      = $40000000;
  IOC_IN       = $80000000;
  IOC_INOUT    = (IOC_IN or IOC_OUT);

  FIONREAD     = IOC_OUT or { get # bytes to read }
    ((Longint(SizeOf(Longint)) and IOCPARM_MASK) shl 16) or
    (Longint(Byte('f')) shl 8) or 127;
  FIONBIO      = IOC_IN or { set/clear non-blocking i/o }
    ((Longint(SizeOf(Longint)) and IOCPARM_MASK) shl 16) or
    (Longint(Byte('f')) shl 8) or 126;
  FIOASYNC     = IOC_IN or { set/clear async i/o }
    ((Longint(SizeOf(Longint)) and IOCPARM_MASK) shl 16) or
    (Longint(Byte('f')) shl 8) or 125;

type
  PHostEnt = ^THostEnt;
  THostEnt = record
    h_name: PChar;
    h_aliases: ^PChar;
    h_addrtype: Smallint;
    h_length: Smallint;
    case Byte of
      0: (h_addr_list: ^PChar);
      1: (h_addr: ^PChar)
  end;
  hostent = THostEnt;

  PNetEnt = ^TNetEnt;
  TNetEnt = record
    n_name: PChar;
    n_aliases: ^PChar;
    n_addrtype: Smallint;
    n_net: u_long;
  end;
  netent = TNetEnt;

  PServEnt = ^TServEnt;
  TServEnt = record
    s_name: PChar;
    s_aliases: ^PChar;
{$ifdef WIN64}
    s_proto: PChar;
    s_port: Smallint;
{$else WIN64}
    s_port: Smallint;
    s_proto: PChar;
{$endif WIN64}
  end;
  servent = TServEnt;

  PProtoEnt = ^TProtoEnt;
  TProtoEnt = record
    p_name: PChar;
    p_aliases: ^Pchar;
    p_proto: Smallint;
  end;
  protoent = TProtoEnt;

const

{ Protocols }
  IPPROTO_IP     =   0;             { dummy for IP }
  IPPROTO_ICMP   =   1;             { control message protocol }
  IPPROTO_IGMP   =   2;             { group management protocol }
  IPPROTO_GGP    =   3;             { gateway^2 (deprecated) }
  IPPROTO_TCP    =   6;             { tcp }
  IPPROTO_PUP    =  12;             { pup }
  IPPROTO_UDP    =  17;             { user datagram protocol }
  IPPROTO_IDP    =  22;             { xns idp }
  IPPROTO_ND     =  77;             { UNOFFICIAL net disk proto }

  IPPROTO_RAW    =  255;            { raw IP packet }
  IPPROTO_MAX    =  256;

{ Port/socket numbers: network standard functions}

  IPPORT_ECHO    =   7;
  IPPORT_DISCARD =   9;
  IPPORT_SYSTAT  =   11;
  IPPORT_DAYTIME =   13;
  IPPORT_NETSTAT =   15;
  IPPORT_FTP     =   21;
  IPPORT_TELNET  =   23;
  IPPORT_SMTP    =   25;
  IPPORT_TIMESERVER  =  37;
  IPPORT_NAMESERVER  =  42;
  IPPORT_WHOIS       =  43;
  IPPORT_MTP         =  57;

{ Port/socket numbers: host specific functions }

  IPPORT_TFTP        =  69;
  IPPORT_RJE         =  77;
  IPPORT_FINGER      =  79;
  IPPORT_TTYLINK     =  87;
  IPPORT_SUPDUP      =  95;

{ UNIX TCP sockets }

  IPPORT_EXECSERVER  =  512;
  IPPORT_LOGINSERVER =  513;
  IPPORT_CMDSERVER   =  514;
  IPPORT_EFSSERVER   =  520;

{ UNIX UDP sockets }

  IPPORT_BIFFUDP     =  512;
  IPPORT_WHOSERVER   =  513;
  IPPORT_ROUTESERVER =  520;

{ Ports < IPPORT_RESERVED are reserved for
  privileged processes (e.g. root). }

  IPPORT_RESERVED    =  1024;

{ Link numbers }

  IMPLINK_IP         =  155;
  IMPLINK_LOWEXPER   =  156;
  IMPLINK_HIGHEXPER  =  158;

  TF_DISCONNECT           = $01;
  TF_REUSE_SOCKET         = $02;
  TF_WRITE_BEHIND         = $04;

{ Options for use with [gs]etsockopt at the IP level. }

  IP_OPTIONS          = 1;
  IP_MULTICAST_IF     = 2;           { set/get IP multicast interface   }
  IP_MULTICAST_TTL    = 3;           { set/get IP multicast timetolive  }
  IP_MULTICAST_LOOP   = 4;           { set/get IP multicast loopback    }
  IP_ADD_MEMBERSHIP   = 5;           { add  an IP group membership      }
  IP_DROP_MEMBERSHIP  = 6;           { drop an IP group membership      }
  IP_TTL              = 7;           { set/get IP Time To Live          }
  IP_TOS              = 8;           { set/get IP Type Of Service       }
  IP_DONTFRAGMENT     = 9;           { set/get IP Don't Fragment flag   }


  IP_DEFAULT_MULTICAST_TTL   = 1;    { normally limit m'casts to 1 hop  }
  IP_DEFAULT_MULTICAST_LOOP  = 1;    { normally hear sends if a member  }
  IP_MAX_MEMBERSHIPS         = 20;   { per socket; must fit in one mbuf }

{ This is used instead of -1, since the
  TSocket type is unsigned.}

  INVALID_SOCKET                = TSocket(NOT(0));
  SOCKET_ERROR                  = -1;

{ Types }

  SOCK_STREAM     = 1;               { stream socket }
  SOCK_DGRAM      = 2;               { datagram socket }
  SOCK_RAW        = 3;               { raw-protocol interface }
  SOCK_RDM        = 4;               { reliably-delivered message }
  SOCK_SEQPACKET  = 5;               { sequenced packet stream }

{ Option flags per-socket. }

  SO_DEBUG        = $0001;          { turn on debugging info recording }
  SO_ACCEPTCONN   = $0002;          { socket has had listen() }
  SO_REUSEADDR    = $0004;          { allow local address reuse }
  SO_KEEPALIVE    = $0008;          { keep connections alive }
  SO_DONTROUTE    = $0010;          { just use interface addresses }
  SO_BROADCAST    = $0020;          { permit sending of broadcast msgs }
  SO_USELOOPBACK  = $0040;          { bypass hardware when possible }
  SO_LINGER       = $0080;          { linger on close if data present }
  SO_OOBINLINE    = $0100;          { leave received OOB data in line }

  SO_DONTLINGER       = Integer(not SO_LINGER);
  SO_EXCLUSIVEADDRUSE = Integer(not SO_REUSEADDR); { disallow local address reuse }

{ Additional options. }

  SO_SNDBUF       = $1001;          { send buffer size }
  SO_RCVBUF       = $1002;          { receive buffer size }
  SO_SNDLOWAT     = $1003;          { send low-water mark }
  SO_RCVLOWAT     = $1004;          { receive low-water mark }
  SO_SNDTIMEO     = $1005;          { send timeout }
  SO_RCVTIMEO     = $1006;          { receive timeout }
  SO_ERROR        = $1007;          { get error status and clear }
  SO_TYPE         = $1008;          { get socket type }

{ Options for connect and disconnect data and options.  Used only by
  non-TCP/IP transports such as DECNet, OSI TP4, etc. }

  SO_CONNDATA     = $7000;
  SO_CONNOPT      = $7001;
  SO_DISCDATA     = $7002;
  SO_DISCOPT      = $7003;
  SO_CONNDATALEN  = $7004;
  SO_CONNOPTLEN   = $7005;
  SO_DISCDATALEN  = $7006;
  SO_DISCOPTLEN   = $7007;

{ Option for opening sockets for synchronous access. }
  SO_OPENTYPE     = $7008;
  SO_SYNCHRONOUS_ALERT    = $10;
  SO_SYNCHRONOUS_NONALERT = $20;

{ Other NT-specific options. }
  SO_MAXDG        = $7009;
  SO_MAXPATHDG    = $700A;
  SO_UPDATE_ACCEPT_CONTEXT     = $700B;
  SO_CONNECT_TIME = $700C;

{ TCP options. }
  TCP_NODELAY     = $0001;
  TCP_BSDURGENT   = $7000;

{ WinSock 2 extension -- new options }
        SO_GROUP_ID = $2001; // ID of a socket group
        SO_GROUP_PRIORITY = $2002; // the relative priority within a group
        SO_MAX_MSG_SIZE = $2003; // maximum message size
        SO_PROTOCOL_INFOA = $2004; // WSAPROTOCOL_INFOA structure
        SO_PROTOCOL_INFOW = $2005; // WSAPROTOCOL_INFOW structure
{$IFDEF UNICODE}
        SO_PROTOCOL_INFO = SO_PROTOCOL_INFOW;
{$ELSE}
        SO_PROTOCOL_INFO = SO_PROTOCOL_INFOA;
{$ENDIF}
        PVD_CONFIG = $3001; // configuration info for service provider
        SO_CONDITIONAL_ACCEPT = $3002;   { enable true conditional accept: 
                                           connection is not ack-ed to the 
                                           other side until conditional 
                                           function returns CF_ACCEPT }

{ Address families. }
  AF_UNSPEC       = 0;               { unspecified }
  AF_UNIX         = 1;               { local to host (pipes, portals) }
  AF_INET         = 2;               { internetwork: UDP, TCP, etc. }
  AF_IMPLINK      = 3;               { arpanet imp addresses }
  AF_PUP          = 4;               { pup protocols: e.g. BSP }
  AF_CHAOS        = 5;               { mit CHAOS protocols }
  AF_IPX          = 6;               { IPX and SPX }
  AF_NS           = 6;               { XEROX NS protocols }
  AF_ISO          = 7;               { ISO protocols }
  AF_OSI          = AF_ISO;          { OSI is ISO }
  AF_ECMA         = 8;               { european computer manufacturers }
  AF_DATAKIT      = 9;               { datakit protocols }
  AF_CCITT        = 10;              { CCITT protocols, X.25 etc }
  AF_SNA          = 11;              { IBM SNA }
  AF_DECnet       = 12;              { DECnet }
  AF_DLI          = 13;              { Direct data link interface }
  AF_LAT          = 14;              { LAT }
  AF_HYLINK       = 15;              { NSC Hyperchannel }
  AF_APPLETALK    = 16;              { AppleTalk }
  AF_NETBIOS      = 17;              { NetBios-style addresses }
  AF_VOICEVIEW    = 18;              { VoiceView }
  AF_FIREFOX      = 19;              { FireFox }
  AF_UNKNOWN1     = 20;              { Somebody is using this! }
  AF_BAN          = 21;              { Banyan }
  AF_ATM          = 22;              { Native ATM Services }
  AF_INET6        = 23;              { Internetwork Version 6 }
  AF_CLUSTER      = 24;              { Microsoft Wolfpack }
  AF_12844        = 25;              { IEEE 1284.4 WG AF }
  AF_IRDA         = 26;              { IrDA }
  AF_NETDES       = 28;              { Network Designers OSI & gateway
                                            enabled protocols }
  AF_TCNPROCESS   = 29;
  AF_TCNMESSAGE   = 30;
  AF_ICLFXBM      = 31;

  AF_MAX          = 32;

{       Socket I/O Controls }
{Const
        SIOCSHIWAT = _IOW('s', 0, u_long); // set high watermark
        SIOCGHIWAT = _IOR('s', 1, u_long); // get high watermark
        SIOCSLOWAT = _IOW('s', 2, u_long); // set low watermark
        SIOCGLOWAT = _IOR('s', 3, u_long); // get low watermark
        SIOCATMARK = _IOR('s', 7, u_long); // at oob mark? }

{ Protocol families, same as address families for now. }

  PF_UNSPEC       = AF_UNSPEC;
  PF_UNIX         = AF_UNIX;
  PF_INET         = AF_INET;
  PF_IMPLINK      = AF_IMPLINK;
  PF_PUP          = AF_PUP;
  PF_CHAOS        = AF_CHAOS;
  PF_NS           = AF_NS;
  PF_IPX          = AF_IPX;
  PF_ISO          = AF_ISO;
  PF_OSI          = AF_OSI;
  PF_ECMA         = AF_ECMA;
  PF_DATAKIT      = AF_DATAKIT;
  PF_CCITT        = AF_CCITT;
  PF_SNA          = AF_SNA;
  PF_DECnet       = AF_DECnet;
  PF_DLI          = AF_DLI;
  PF_LAT          = AF_LAT;
  PF_HYLINK       = AF_HYLINK;
  PF_APPLETALK    = AF_APPLETALK;
  PF_VOICEVIEW    = AF_VOICEVIEW;
  PF_FIREFOX      = AF_FIREFOX;
  PF_UNKNOWN1     = AF_UNKNOWN1;
  PF_BAN          = AF_BAN;
  PF_ATM          = AF_ATM;
  PF_INET6        = AF_INET6;
  PF_CLUSTER      = AF_CLUSTER;
  PF_12844        = AF_12844;
  PF_IRDA         = AF_IRDA;
  PF_NETDES       = AF_NETDES;
  PF_TCNPROCESS   = AF_TCNPROCESS;
  PF_TCNMESSAGE   = AF_TCNMESSAGE; 
  PF_ICLFXBM      = AF_ICLFXBM;

  PF_MAX          = AF_MAX;

type

  SunB = record
    s_b1, s_b2, s_b3, s_b4: u_char;
  end;

  SunW = record
    s_w1, s_w2: u_short;
  end;

  PInAddr = ^TInAddr;
  TInAddr = record
    case integer of
      0: (S_un_b: SunB);
      1: (S_un_w: SunW);
      2: (S_addr: u_long);
    end;
  in_addr = TInAddr;

  PIn6Addr = ^TIn6Addr;
  TIn6Addr = record
    case byte of
      0: (u6_addr8  : array[0..15] of byte);
      1: (u6_addr16 : array[0..7] of Word);
      2: (u6_addr32 : array[0..3] of Cardinal);
      3: (s6_addr8  : array[0..15] of shortint);
      4: (s6_addr   : array[0..15] of shortint);
      5: (s6_addr16 : array[0..7] of smallint);
      6: (s6_addr32 : array[0..3] of LongInt);
    end;

  PSockAddrIn = ^TSockAddrIn;
  TSockAddrIn = record
    case Integer of
      0: (sin_family: u_short;
          sin_port: u_short;
          sin_addr: TInAddr;
          sin_zero: array[0..7] of Char);
      1: (sa_family: u_short;
          sa_data: array[0..13] of Char)
  end;
  sockaddr_in = TSockAddrIn;

  PSockAddrIn6 = ^TSockAddrIn6;
  TSockAddrIn6 = record
    sin6_family   : u_short;
    sin6_port     : u_short;
    sin6_flowinfo : u_long;
    sin6_addr     : TIn6Addr;
    sin6_scope_id : u_long;
  end;
  sockaddr_in6 = TSockAddrIn6;

  { Structure used by kernel to store most addresses. }

  PSockAddr = ^TSockAddr;
  TSockAddr = TSockAddrIn;

  { Structure used by kernel to pass protocol information in raw sockets. }
  PSockProto = ^TSockProto;
  TSockProto = record
    sp_family: u_short;
    sp_protocol: u_short;
  end;
  sockproto = TSockProto;

{ Structure used for manipulating linger option. }
  PLinger = ^TLinger;
  TLinger = record
    l_onoff: u_short;
    l_linger: u_short;
  end;

const
  INADDR_ANY       = $00000000;
  INADDR_LOOPBACK  = $7F000001;
  INADDR_BROADCAST = $FFFFFFFF;
  INADDR_NONE      = $FFFFFFFF;

  { options for socket level  }
  SOL_SOCKET      = $ffff;

  MSG_OOB         = $1;             {process out-of-band data }
  MSG_PEEK        = $2;             {peek at incoming message }
  MSG_DONTROUTE   = $4;             {send without using routing tables }

{ WinSock 2 extension -- new flags for WSASend(), WSASendTo(), WSARecv() and WSARecvFrom() }
  MSG_INTERRUPT = $10; {/* send/recv in the interrupt context */}
  MSG_MAXIOVLEN = 16;

  MSG_PARTIAL     = $8000;          {partial send or recv for message xport }

{ Define constant based on rfc883, used by gethostbyxxxx() calls. }

  MAXGETHOSTSTRUCT        = 1024;

{ Maximum queue length specifiable by listen. }
        SOMAXCONN = $7fffffff;

{ WinSock 2 extension -- bit values and indices for FD_XXX network events }
        FD_READ_BIT = 0;
        FD_READ = (1  shl  FD_READ_BIT);
        FD_WRITE_BIT = 1;
        FD_WRITE = (1  shl  FD_WRITE_BIT);
        FD_OOB_BIT = 2;
        FD_OOB = (1  shl  FD_OOB_BIT);
        FD_ACCEPT_BIT = 3;
        FD_ACCEPT = (1  shl  FD_ACCEPT_BIT);
        FD_CONNECT_BIT = 4;
        FD_CONNECT = (1  shl  FD_CONNECT_BIT);
        FD_CLOSE_BIT = 5;
        FD_CLOSE = (1  shl  FD_CLOSE_BIT);
        FD_QOS_BIT = 6;
        FD_QOS = (1  shl  FD_QOS_BIT);
        FD_GROUP_QOS_BIT = 7;
        FD_GROUP_QOS = (1  shl  FD_GROUP_QOS_BIT);
        FD_MAX_EVENTS = 8;
        FD_ALL_EVENTS = ((1  shl  FD_MAX_EVENTS) - 1);

{ All Windows Sockets error constants are biased by WSABASEERR from the "normal" }

  WSABASEERR              = 10000;

{ Windows Sockets definitions of regular Microsoft C error constants }

  WSAEINTR                = (WSABASEERR+4);
  WSAEBADF                = (WSABASEERR+9);
  WSAEACCES               = (WSABASEERR+13);
  WSAEFAULT               = (WSABASEERR+14);
  WSAEINVAL               = (WSABASEERR+22);
  WSAEMFILE               = (WSABASEERR+24);

{ Windows Sockets definitions of regular Berkeley error constants }

  WSAEWOULDBLOCK          = (WSABASEERR+35);
  WSAEINPROGRESS          = (WSABASEERR+36);
  WSAEALREADY             = (WSABASEERR+37);
  WSAENOTSOCK             = (WSABASEERR+38);
  WSAEDESTADDRREQ         = (WSABASEERR+39);
  WSAEMSGSIZE             = (WSABASEERR+40);
  WSAEPROTOTYPE           = (WSABASEERR+41);
  WSAENOPROTOOPT          = (WSABASEERR+42);
  WSAEPROTONOSUPPORT      = (WSABASEERR+43);
  WSAESOCKTNOSUPPORT      = (WSABASEERR+44);
  WSAEOPNOTSUPP           = (WSABASEERR+45);
  WSAEPFNOSUPPORT         = (WSABASEERR+46);
  WSAEAFNOSUPPORT         = (WSABASEERR+47);
  WSAEADDRINUSE           = (WSABASEERR+48);
  WSAEADDRNOTAVAIL        = (WSABASEERR+49);
  WSAENETDOWN             = (WSABASEERR+50);
  WSAENETUNREACH          = (WSABASEERR+51);
  WSAENETRESET            = (WSABASEERR+52);
  WSAECONNABORTED         = (WSABASEERR+53);
  WSAECONNRESET           = (WSABASEERR+54);
  WSAENOBUFS              = (WSABASEERR+55);
  WSAEISCONN              = (WSABASEERR+56);
  WSAENOTCONN             = (WSABASEERR+57);
  WSAESHUTDOWN            = (WSABASEERR+58);
  WSAETOOMANYREFS         = (WSABASEERR+59);
  WSAETIMEDOUT            = (WSABASEERR+60);
  WSAECONNREFUSED         = (WSABASEERR+61);
  WSAELOOP                = (WSABASEERR+62);
  WSAENAMETOOLONG         = (WSABASEERR+63);
  WSAEHOSTDOWN            = (WSABASEERR+64);
  WSAEHOSTUNREACH         = (WSABASEERR+65);
  WSAENOTEMPTY            = (WSABASEERR+66);
  WSAEPROCLIM             = (WSABASEERR+67);
  WSAEUSERS               = (WSABASEERR+68);
  WSAEDQUOT               = (WSABASEERR+69);
  WSAESTALE               = (WSABASEERR+70);
  WSAEREMOTE              = (WSABASEERR+71);

{ Extended Windows Sockets error constant definitions }

  WSASYSNOTREADY          = (WSABASEERR+91);
  WSAVERNOTSUPPORTED      = (WSABASEERR+92);
  WSANOTINITIALISED       = (WSABASEERR+93);
  WSAEDISCON = (WSABASEERR+101);
  WSAENOMORE = (WSABASEERR+102);
  WSAECANCELLED = (WSABASEERR+103);
  WSAEINVALIDPROCTABLE = (WSABASEERR+104);
  WSAEINVALIDPROVIDER = (WSABASEERR+105);
  WSAEPROVIDERFAILEDINIT = (WSABASEERR+106);
  WSASYSCALLFAILURE = (WSABASEERR+107);
  WSASERVICE_NOT_FOUND = (WSABASEERR+108);
  WSATYPE_NOT_FOUND = (WSABASEERR+109);
  WSA_E_NO_MORE = (WSABASEERR+110);
  WSA_E_CANCELLED = (WSABASEERR+111);
  WSAEREFUSED = (WSABASEERR+112);


{ Error return codes from gethostbyname() and gethostbyaddr()
  (when using the resolver). Note that these errors are
  retrieved via WSAGetLastError() and must therefore follow
  the rules for avoiding clashes with error numbers from
  specific implementations or language run-time systems.
  For this reason the codes are based at WSABASEERR+1001.
  Note also that [WSA]NO_ADDRESS is defined only for
  compatibility purposes. }

{ Authoritative Answer: Host not found }
  WSAHOST_NOT_FOUND       = (WSABASEERR+1001);
  HOST_NOT_FOUND          = WSAHOST_NOT_FOUND;

{ Non-Authoritative: Host not found, or SERVERFAIL }
  WSATRY_AGAIN            = (WSABASEERR+1002);
  TRY_AGAIN               = WSATRY_AGAIN;

{ Non recoverable errors, FORMERR, REFUSED, NOTIMP }
  WSANO_RECOVERY          = (WSABASEERR+1003);
  NO_RECOVERY             = WSANO_RECOVERY;

{ Valid name, no data record of requested type }
  WSANO_DATA              = (WSABASEERR+1004);
  NO_DATA                 = WSANO_DATA;

{ no address, look for MX record }
  WSANO_ADDRESS           = WSANO_DATA;
  NO_ADDRESS              = WSANO_ADDRESS;

{ WinSock 2 extension -- new error codes and type definition }
  WSA_IO_PENDING = ERROR_IO_PENDING;
  WSA_IO_INCOMPLETE = ERROR_IO_INCOMPLETE;
  WSA_INVALID_HANDLE = ERROR_INVALID_HANDLE;
  WSA_INVALID_PARAMETER = ERROR_INVALID_PARAMETER;
  WSA_NOT_ENOUGH_MEMORY = ERROR_NOT_ENOUGH_MEMORY;
  WSA_OPERATION_ABORTED = ERROR_OPERATION_ABORTED;
{$ifndef FPC}{TODO}
  WSA_INVALID_EVENT = WSAEVENT(nil);
{$endif}
  WSA_MAXIMUM_WAIT_EVENTS = MAXIMUM_WAIT_OBJECTS;
  WSA_WAIT_FAILED = $ffffffff;
  WSA_WAIT_EVENT_0 = WAIT_OBJECT_0;
  WSA_WAIT_IO_COMPLETION = WAIT_IO_COMPLETION;
  WSA_WAIT_TIMEOUT = WAIT_TIMEOUT;
  WSA_INFINITE = INFINITE;

{ Windows Sockets errors redefined as regular Berkeley error constants.
  These are commented out in Windows NT to avoid conflicts with errno.h.
  Use the WSA constants instead. }

  EWOULDBLOCK        =  WSAEWOULDBLOCK;
  EINPROGRESS        =  WSAEINPROGRESS;
  EALREADY           =  WSAEALREADY;
  ENOTSOCK           =  WSAENOTSOCK;
  EDESTADDRREQ       =  WSAEDESTADDRREQ;
  EMSGSIZE           =  WSAEMSGSIZE;
  EPROTOTYPE         =  WSAEPROTOTYPE;
  ENOPROTOOPT        =  WSAENOPROTOOPT;
  EPROTONOSUPPORT    =  WSAEPROTONOSUPPORT;
  ESOCKTNOSUPPORT    =  WSAESOCKTNOSUPPORT;
  EOPNOTSUPP         =  WSAEOPNOTSUPP;
  EPFNOSUPPORT       =  WSAEPFNOSUPPORT;
  EAFNOSUPPORT       =  WSAEAFNOSUPPORT;
  EADDRINUSE         =  WSAEADDRINUSE;
  EADDRNOTAVAIL      =  WSAEADDRNOTAVAIL;
  ENETDOWN           =  WSAENETDOWN;
  ENETUNREACH        =  WSAENETUNREACH;
  ENETRESET          =  WSAENETRESET;
  ECONNABORTED       =  WSAECONNABORTED;
  ECONNRESET         =  WSAECONNRESET;
  ENOBUFS            =  WSAENOBUFS;
  EISCONN            =  WSAEISCONN;
  ENOTCONN           =  WSAENOTCONN;
  ESHUTDOWN          =  WSAESHUTDOWN;
  ETOOMANYREFS       =  WSAETOOMANYREFS;
  ETIMEDOUT          =  WSAETIMEDOUT;
  ECONNREFUSED       =  WSAECONNREFUSED;
  ELOOP              =  WSAELOOP;
  ENAMETOOLONG       =  WSAENAMETOOLONG;
  EHOSTDOWN          =  WSAEHOSTDOWN;
  EHOSTUNREACH       =  WSAEHOSTUNREACH;
  ENOTEMPTY          =  WSAENOTEMPTY;
  EPROCLIM           =  WSAEPROCLIM;
  EUSERS             =  WSAEUSERS;
  EDQUOT             =  WSAEDQUOT;
  ESTALE             =  WSAESTALE;
  EREMOTE            =  WSAEREMOTE;


  WSADESCRIPTION_LEN     =   256;
  WSASYS_STATUS_LEN      =   128;
  MAX_PROTOCOL_CHAIN = 7;
  BASE_PROTOCOL = 1;
  LAYERED_PROTOCOL = 0;
  WSAPROTOCOL_LEN = 255;

type
  PWSAData = ^TWSAData;
  TWSAData = record
     wVersion : WORD;              { 2 byte, ofs 0 }
     wHighVersion : WORD;          { 2 byte, ofs 2 }
{$ifdef win64}
     iMaxSockets : word;
     iMaxUdpDg : word;
     lpVendorInfo : pchar;
     szDescription : array[0..WSADESCRIPTION_LEN] of char;
     szSystemStatus : array[0..WSASYS_STATUS_LEN] of char;
{$else win64}
     szDescription : array[0..WSADESCRIPTION_LEN] of char; { 257 byte, ofs 4 }
     szSystemStatus : array[0..WSASYS_STATUS_LEN] of char; { 129 byte, ofs 261 }
     iMaxSockets : word;           { 2 byte, ofs 390 }
     iMaxUdpDg : word;             { 2 byte, ofs 392 }
     lpVendorInfo : pchar;         { 4 byte, ofs 396 }
{$endif win64}
  end;
  WSAData = TWSAData;

{       WSAOVERLAPPED = Record
                Internal: LongInt;
                InternalHigh: LongInt;
                Offset: LongInt;
                OffsetHigh: LongInt;
                hEvent: WSAEVENT;
        end;}
        WSAOVERLAPPED = TOverlapped;
        TWSAOverlapped = WSAOverlapped;
        PWSAOverlapped = ^WSAOverlapped;
        LPWSAOVERLAPPED = PWSAOverlapped;

{       WinSock 2 extension -- WSABUF and QOS struct, include qos.h }
{ to pull in FLOWSPEC and related definitions }


        WSABUF = record
                len: U_LONG;    { the length of the buffer }
                buf: PChar;     { the pointer to the buffer }
        end {WSABUF};
        PWSABUF = ^WSABUF;
        LPWSABUF = PWSABUF;

        TServiceType = LongInt;

        TFlowSpec = Record
                TokenRate,               // In Bytes/sec
                TokenBucketSize,         // In Bytes
                PeakBandwidth,           // In Bytes/sec
                Latency,                 // In microseconds
                DelayVariation : LongInt;// In microseconds
                ServiceType : TServiceType;
                MaxSduSize,     MinimumPolicedSize : LongInt;// In Bytes
        end;
        PFlowSpec = ^TFLOWSPEC;
        flowspec = TFlowSpec;

        TQualityOfService = record
                SendingFlowspec: TFlowSpec;     { the flow spec for data sending }
                ReceivingFlowspec: TFlowSpec;   { the flow spec for data receiving }
                ProviderSpecific: WSABUF; { additional provider specific stuff }
        end {TQualityOfService};
        PQOS = ^TQualityOfService;
        LPQOS = PQOS;
        
Const
        SERVICETYPE_NOTRAFFIC             =  $00000000;  // No data in this direction
        SERVICETYPE_BESTEFFORT            =  $00000001;  // Best Effort
        SERVICETYPE_CONTROLLEDLOAD        =  $00000002;  // Controlled Load
        SERVICETYPE_GUARANTEED            =  $00000003;  // Guaranteed
        SERVICETYPE_NETWORK_UNAVAILABLE   =  $00000004;  // Used to notify change to user
        SERVICETYPE_GENERAL_INFORMATION   =  $00000005;  // corresponds to "General Parameters" defined by IntServ
        SERVICETYPE_NOCHANGE              =  $00000006;  // used to indicate that the flow spec contains no change from any previous one
// to turn on immediate traffic control, OR this flag with the ServiceType field in teh FLOWSPEC
        SERVICE_IMMEDIATE_TRAFFIC_CONTROL =  $80000000;

{       WinSock 2 extension -- manifest constants for return values of the condition function }
        CF_ACCEPT = $0000;
        CF_REJECT = $0001;
        CF_DEFER = $0002;

{       WinSock 2 extension -- manifest constants for shutdown() }
        SD_RECEIVE = $00;
        SD_SEND = $01;
        SD_BOTH = $02;

{       WinSock 2 extension -- data type and manifest constants for socket groups }
        SG_UNCONSTRAINED_GROUP = $01;
        SG_CONSTRAINED_GROUP = $02;
Type
        GROUP = u_long;

{       WinSock 2 extension -- data type for WSAEnumNetworkEvents() }
        TWSANetworkEvents = record
                lNetworkEvents: LongInt;
                iErrorCode: Array[0..FD_MAX_EVENTS-1] of Longint;
        end {TWSANetworkEvents};
        PWSANetworkEvents = ^TWSANetworkEvents;
        LPWSANetworkEvents = PWSANetworkEvents;

        TWSAProtocolChain = record
                ChainLen: Longint;      // the length of the chain,
                // length = 0 means layered protocol,
                // length = 1 means base protocol,
                // length > 1 means protocol chain
                ChainEntries: Array[0..MAX_PROTOCOL_CHAIN-1] of LongInt; { a list of dwCatalogEntryIds }
        end {TWSAPROTOCOLCHAIN};

Type
        TWSAProtocol_InfoA = record
                dwServiceFlags1: LongInt;
                dwServiceFlags2: LongInt;
                dwServiceFlags3: LongInt;
                dwServiceFlags4: LongInt;
                dwProviderFlags: LongInt;
                ProviderId: TGUID;
                dwCatalogEntryId: LongInt;
                ProtocolChain: TWSAProtocolChain;
                iVersion: Longint;
                iAddressFamily: Longint;
                iMaxSockAddr: Longint;
                iMinSockAddr: Longint;
                iSocketType: Longint;
                iProtocol: Longint;
                iProtocolMaxOffset: Longint;
                iNetworkByteOrder: Longint;
                iSecurityScheme: Longint;
                dwMessageSize: LongInt;
                dwProviderReserved: LongInt;
                szProtocol: Array[0..WSAPROTOCOL_LEN+1-1] of Char;
        end {TWSAProtocol_InfoA};
        PWSAProtocol_InfoA = ^TWSAProtocol_InfoA;
        LPWSAProtocol_InfoA = PWSAProtocol_InfoA;

        TWSAProtocol_InfoW = record
                dwServiceFlags1: LongInt;
                dwServiceFlags2: LongInt;
                dwServiceFlags3: LongInt;
                dwServiceFlags4: LongInt;
                dwProviderFlags: LongInt;
                ProviderId: TGUID;
                dwCatalogEntryId: LongInt;
                ProtocolChain: TWSAProtocolChain;
                iVersion: Longint;
                iAddressFamily: Longint;
                iMaxSockAddr: Longint;
                iMinSockAddr: Longint;
                iSocketType: Longint;
                iProtocol: Longint;
                iProtocolMaxOffset: Longint;
                iNetworkByteOrder: Longint;
                iSecurityScheme: Longint;
                dwMessageSize: LongInt;
                dwProviderReserved: LongInt;
                szProtocol: Array[0..(WSAPROTOCOL_LEN+1-1)] of WideChar;
        end {TWSAProtocol_InfoW};
        PWSAProtocol_InfoW = ^TWSAProtocol_InfoW;
        LPWSAProtocol_InfoW = PWSAProtocol_InfoW;

{$IFDEF UNICODE}
        TWSAProtocol_Info = TWSAProtocol_InfoW;
        LPWSAProtocol_Info = PWSAProtocol_InfoW;
{$ELSE}
        TWSAProtocol_Info = TWSAProtocol_InfoA;
        LPWSAProtocol_Info = PWSAProtocol_InfoA;
{$ENDIF}

{       Flag bit definitions for dwProviderFlags */ }
Const
        PFL_MULTIPLE_PROTO_ENTRIES = $00000001;
        PFL_RECOMMENDED_PROTO_ENTRY = $00000002;
        PFL_HIDDEN = $00000004;
        PFL_MATCHES_PROTOCOL_ZERO = $00000008;

{       Flag bit definitions for dwServiceFlags1 */ }
        XP1_CONNECTIONLESS = $00000001;
        XP1_GUARANTEED_DELIVERY = $00000002;
        XP1_GUARANTEED_ORDER = $00000004;
        XP1_MESSAGE_ORIENTED = $00000008;
        XP1_PSEUDO_STREAM = $00000010;
        XP1_GRACEFUL_CLOSE = $00000020;
        XP1_EXPEDITED_DATA = $00000040;
        XP1_CONNECT_DATA = $00000080;
        XP1_DISCONNECT_DATA = $00000100;
        XP1_SUPPORT_BROADCAST = $00000200;
        XP1_SUPPORT_MULTIPOINT = $00000400;
        XP1_MULTIPOINT_CONTROL_PLANE = $00000800;
        XP1_MULTIPOINT_DATA_PLANE = $00001000;
        XP1_QOS_SUPPORTED = $00002000;
        XP1_INTERRUPT = $00004000;
        XP1_UNI_SEND = $00008000;
        XP1_UNI_RECV = $00010000;
        XP1_IFS_HANDLES = $00020000;
        XP1_PARTIAL_MESSAGE = $00040000;

        BIGENDIAN = $0000;
        LITTLEENDIAN = $0001;

        SECURITY_PROTOCOL_NONE = $0000;

{       WinSock 2 extension -- manifest constants for WSAJoinLeaf() }
        JL_SENDER_ONLY = $01;
        JL_RECEIVER_ONLY = $02;
        JL_BOTH = $04;

{ WinSock 2 extension -- manifest constants for WSASocket() }
        WSA_FLAG_OVERLAPPED = $01;
        WSA_FLAG_MULTIPOINT_C_ROOT = $02;
        WSA_FLAG_MULTIPOINT_C_LEAF = $04;
        WSA_FLAG_MULTIPOINT_D_ROOT = $08;
        WSA_FLAG_MULTIPOINT_D_LEAF = $10;

{ WinSock 2 extension -- manifest constants for WSAIoctl() }
        IOC_UNIX = $00000000;
        IOC_WS2 = $08000000;
        IOC_PROTOCOL = $10000000;
        IOC_VENDOR = $18000000;

        SIO_ASSOCIATE_HANDLE = IOC_IN or IOC_WS2 or 1;
        SIO_ENABLE_CIRCULAR_QUEUEING = IOC_WS2 or 2;
        SIO_FIND_ROUTE = IOC_OUT or IOC_WS2 or 3;
        SIO_FLUSH = IOC_WS2 or 4;
        SIO_GET_BROADCAST_ADDRESS = IOC_OUT or IOC_WS2 or 5;
        SIO_GET_EXTENSION_FUNCTION_POINTER = IOC_INOUT or IOC_WS2 or 6;
        SIO_GET_QOS = IOC_INOUT or IOC_WS2 or 7;
        SIO_GET_GROUP_QOS = IOC_INOUT or IOC_WS2 or 8;
        SIO_MULTIPOINT_LOOPBACK = IOC_IN or IOC_WS2 or 9;
        SIO_MULTICAST_SCOPE = IOC_IN or IOC_WS2 or 10;
        SIO_SET_QOS = IOC_IN or IOC_WS2 or 11;
        SIO_SET_GROUP_QOS = IOC_IN or IOC_WS2 or 12;
        SIO_TRANSLATE_HANDLE = IOC_INOUT or IOC_WS2 or 13;

{WinSock 2 extension -- manifest constants for SIO_TRANSLATE_HANDLE ioctl }
        TH_NETDEV = $00000001;
        TH_TAPI = $00000002;

Const
        SERVICE_MULTIPLE = $00000001;

{ & Name Spaces }
        NS_ALL = (0);

        NS_SAP = (1);
        NS_NDS = (2);
        NS_PEER_BROWSE = (3);

        NS_TCPIP_LOCAL = (10);
        NS_TCPIP_HOSTS = (11);
        NS_DNS = (12);
        NS_NETBT = (13);
        NS_WINS = (14);

        NS_NBP = (20);

        NS_MS = (30);
        NS_STDA = (31);
        NS_NTDS = (32);

        NS_X500 = (40);
        NS_NIS = (41);
        NS_NISPLUS = (42);

        NS_WRQ = (50);

{       Resolution flags for WSAGetAddressByName().
        Note these are also used by the 1.1 API GetAddressByName, so leave them around. }
        RES_UNUSED_1 = $00000001;
        RES_FLUSH_CACHE = $00000002;
        RES_SERVICE = $00000004;

{       Well known value names for Service Types }
        SERVICE_TYPE_VALUE_IPXPORTA = 'IpxSocket';
{$ifndef FPC}{TODO}
        SERVICE_TYPE_VALUE_IPXPORTW : PWideChar = 'IpxSocket';
        SERVICE_TYPE_VALUE_SAPIDA = 'SapId';
        SERVICE_TYPE_VALUE_SAPIDW : PWideChar = 'SapId';

        SERVICE_TYPE_VALUE_TCPPORTA = 'TcpPort';
        SERVICE_TYPE_VALUE_TCPPORTW : PWideChar = 'TcpPort';

        SERVICE_TYPE_VALUE_UDPPORTA = 'UdpPort';
        SERVICE_TYPE_VALUE_UDPPORTW : PWideChar = 'UdpPort';

        SERVICE_TYPE_VALUE_OBJECTIDA = 'ObjectId';
        SERVICE_TYPE_VALUE_OBJECTIDW : PWideChar = 'ObjectId';

{$IFDEF UNICODE}
        SERVICE_TYPE_VALUE_SAPID = SERVICE_TYPE_VALUE_SAPIDW;
        SERVICE_TYPE_VALUE_TCPPORT = SERVICE_TYPE_VALUE_TCPPORTW;
        SERVICE_TYPE_VALUE_UDPPORT = SERVICE_TYPE_VALUE_UDPPORTW;
        SERVICE_TYPE_VALUE_OBJECTID = SERVICE_TYPE_VALUE_OBJECTIDW;
{$ELSE}
        SERVICE_TYPE_VALUE_SAPID = SERVICE_TYPE_VALUE_SAPIDA;
        SERVICE_TYPE_VALUE_TCPPORT = SERVICE_TYPE_VALUE_TCPPORTA;
        SERVICE_TYPE_VALUE_UDPPORT = SERVICE_TYPE_VALUE_UDPPORTA;
        SERVICE_TYPE_VALUE_OBJECTID = SERVICE_TYPE_VALUE_OBJECTIDA;
{$ENDIF}

{$endif}{FPC}

{ SockAddr Information }


Type
        SOCKET_ADDRESS = record
                lpSockaddr : PSockAddr;
                iSockaddrLength : Longint;
        end {SOCKET_ADDRESS};
        PSOCKET_ADDRESS = ^SOCKET_ADDRESS;

{ CSAddr Information }
        CSADDR_INFO = record
                LocalAddr, RemoteAddr: SOCKET_ADDRESS;
                iSocketType, iProtocol : LongInt;
        end {CSADDR_INFO};
        PCSADDR_INFO = ^CSADDR_INFO;

{       Address Family/Protocol Tuples }
        TAFProtocols = record
                iAddressFamily: Longint;
                iProtocol: Longint;
        end {AFPROTOCOLS};
        PAFProtocols = ^TAFProtocols;

{       Client Query API Typedefs }

{ The comparators }
        TWSAEComparator = (COMP_EQUAL {= 0}, COMP_NOTLESS );

        TWSAVersion = record
                dwVersion: LongInt;
                ecHow: TWSAEComparator;
        end {TWSAVersion};
        PWSAVersion = ^TWSAVersion;

        TWSAQuerySetA = record
                dwSize: LongInt;
                lpszServiceInstanceName: PChar;
                lpServiceClassId: PGUID;
                lpVersion: PWSAVERSION;
                lpszComment: PChar;
                dwNameSpace: LongInt;
                lpNSProviderId: PGUID;
                lpszContext: PChar;
                dwNumberOfProtocols: LongInt;
                lpafpProtocols: PAFProtocols;
                lpszQueryString: PChar;
                dwNumberOfCsAddrs: LongInt;
                lpcsaBuffer: PCSADDR_INFO;
                dwOutputFlags: LongInt;
                lpBlob: PBLOB;
        end {TWSAQuerySetA};
        PWSAQuerySetA = ^TWSAQuerySetA;
        LPWSAQuerySetA = PWSAQuerySetA;
        TWSAQuerySetW = record
                dwSize: LongInt;
                lpszServiceInstanceName: PWideChar;
                lpServiceClassId: PGUID;
                lpVersion: PWSAVERSION;
                lpszComment: PWideChar;
                dwNameSpace: LongInt;
                lpNSProviderId: PGUID;
                lpszContext: PWideChar;
                dwNumberOfProtocols: LongInt;
                lpafpProtocols: PAFProtocols;
                lpszQueryString: PWideChar;
                dwNumberOfCsAddrs: LongInt;
                lpcsaBuffer: PCSADDR_INFO;
                dwOutputFlags: LongInt;
                lpBlob: PBLOB;
        end {TWSAQuerySetW};
        PWSAQuerySetW = ^TWSAQuerySetW;
        LPWSAQuerySetW = PWSAQuerySetW;
{$IFDEF UNICODE}
        PWSAQuerySet = PWSAQuerySetW;
        LPWSAQuerySet = PWSAQuerySetW;
{$ELSE}
        PWSAQuerySet = PWSAQuerySetA;
        LPWSAQuerySet = PWSAQuerySetA;
{$ENDIF}

  PWSAMSG = ^TWSAMSG;
  TWSAMSG = record
    name: PSOCKET_ADDRESS;
    namelen: Longint;
    lpBuffers: LPWSABUF;
    dwBufferCount: DWORD;
    Control: WSABUF;
    dwFlags: DWORD;
  end;
  WSAMSG = TWSAMSG;
  LPWSAMSG = PWSAMSG;

Const
        LUP_DEEP = $0001;
        LUP_CONTAINERS = $0002;
        LUP_NOCONTAINERS = $0004;
        LUP_NEAREST = $0008;
        LUP_RETURN_NAME = $0010;
        LUP_RETURN_TYPE = $0020;
        LUP_RETURN_VERSION = $0040;
        LUP_RETURN_COMMENT = $0080;
        LUP_RETURN_ADDR = $0100;
        LUP_RETURN_BLOB = $0200;
        LUP_RETURN_ALIASES = $0400;
        LUP_RETURN_QUERY_STRING = $0800;
        LUP_RETURN_ALL = $0FF0;
        LUP_RES_SERVICE = $8000;

        LUP_FLUSHCACHE = $1000;
        LUP_FLUSHPREVIOUS = $2000;


{       Return flags }
        RESULT_IS_ALIAS = $0001;

Type
{ Service Address Registration and Deregistration Data Types. }
        TWSAeSetServiceOp = (RNRSERVICE_REGISTER{=0},RNRSERVICE_DEREGISTER,RNRSERVICE_DELETE);

{       Service Installation/Removal Data Types. }
        TWSANSClassInfoA = record
                lpszName: PChar;
                dwNameSpace: LongInt;
                dwValueType: LongInt;
                dwValueSize: LongInt;
                lpValue: Pointer;
        end {_WSANSClassInfoA};
        PWSANSClassInfoA = ^TWSANSClassInfoA;
        TWSANSClassInfoW = record
                lpszName: PWideChar;
                dwNameSpace: LongInt;
                dwValueType: LongInt;
                dwValueSize: LongInt;
                lpValue: Pointer;
        end {TWSANSClassInfoW};
        PWSANSClassInfoW = ^TWSANSClassInfoW;
{$IFDEF UNICODE}
        TWSANSClassInfo = TWSANSClassInfoW;
        PWSANSClassInfo = PWSANSClassInfoW;
        LPWSANSClassInfo = PWSANSClassInfoW;
{$ELSE}
        TWSANSClassInfo = TWSANSClassInfoA;
        PWSANSClassInfo = PWSANSClassInfoA;
        LPWSANSClassInfo = PWSANSClassInfoA;
{$ENDIF // UNICODE}

        TWSAServiceClassInfoA = record
                lpServiceClassId: PGUID;
                lpszServiceClassName: PChar;
                dwCount: LongInt;
                lpClassInfos: PWSANSClassInfoA;
        end {TWSAServiceClassInfoA};
        PWSAServiceClassInfoA = ^TWSAServiceClassInfoA;
        LPWSAServiceClassInfoA = PWSAServiceClassInfoA;
        TWSAServiceClassInfoW = record
                lpServiceClassId: PGUID;
                lpszServiceClassName: PWideChar;
                dwCount: LongInt;
                lpClassInfos: PWSANSClassInfoW;
        end {TWSAServiceClassInfoW};
        PWSAServiceClassInfoW = ^TWSAServiceClassInfoW;
        LPWSAServiceClassInfoW = PWSAServiceClassInfoW;
{$IFDEF UNICODE}
        TWSAServiceClassInfo = TWSAServiceClassInfoW;
        PWSAServiceClassInfo = PWSAServiceClassInfoW;
        LPWSAServiceClassInfo = PWSAServiceClassInfoW;
{$ELSE}
        TWSAServiceClassInfo = TWSAServiceClassInfoA;
        PWSAServiceClassInfo = PWSAServiceClassInfoA;
        LPWSAServiceClassInfo = PWSAServiceClassInfoA;
{$ENDIF}

        TWSANameSpace_InfoA = record
                NSProviderId: TGUID;
                dwNameSpace: LongInt;
                fActive: LongInt{Bool};
                dwVersion: LongInt;
                lpszIdentifier: PChar;
        end {TWSANameSpace_InfoA};
        PWSANameSpace_InfoA = ^TWSANameSpace_InfoA;
        LPWSANameSpace_InfoA = PWSANameSpace_InfoA;
        TWSANameSpace_InfoW = record
                NSProviderId: TGUID;
                dwNameSpace: LongInt;
                fActive: LongInt{Bool};
                dwVersion: LongInt;
                lpszIdentifier: PWideChar;
        end {TWSANameSpace_InfoW};
        PWSANameSpace_InfoW = ^TWSANameSpace_InfoW;
        LPWSANameSpace_InfoW = PWSANameSpace_InfoW;
{$IFDEF UNICODE}
        TWSANameSpace_Info = TWSANameSpace_InfoW;
        PWSANameSpace_Info = PWSANameSpace_InfoW;
        LPWSANameSpace_Info = PWSANameSpace_InfoW;
{$ELSE}
        TWSANameSpace_Info = TWSANameSpace_InfoA;
        PWSANameSpace_Info = PWSANameSpace_InfoA;
        LPWSANameSpace_Info = PWSANameSpace_InfoA;
{$ENDIF}

{       WinSock 2 extensions -- data types for the condition function in }
{ WSAAccept() and overlapped I/O completion routine. }
Type
        LPCONDITIONPROC = function (lpCallerId: LPWSABUF; lpCallerData : LPWSABUF; lpSQOS,lpGQOS : LPQOS; lpCalleeId,lpCalleeData : LPWSABUF;
                g : GROUP; dwCallbackData : DWORD ) : Longint; stdcall;
        LPWSAOVERLAPPED_COMPLETION_ROUTINE = procedure ( const dwError, cbTransferred : DWORD; const lpOverlapped : LPWSAOVERLAPPED; const dwFlags : DWORD ); stdcall;

function accept( const s: TSocket; addr: PSockAddr; addrlen: PLongint ): TSocket; stdcall;external WINSOCK2_DLL name 'accept';
function accept( const s: TSocket; addr: PSockAddr; var addrlen: Longint ): TSocket; stdcall;external WINSOCK2_DLL name 'accept';
function bind( const s: TSocket; addr: PSockAddr; const namelen: Longint ): Longint; stdcall;external WINSOCK2_DLL name 'bind';
function bind( const s: TSocket; const addr: TSockAddr; namelen: Longint ): Longint; stdcall;external WINSOCK2_DLL name 'bind';
function closesocket( const s: TSocket ): Longint; stdcall;external WINSOCK2_DLL name 'closesocket';
function connect( const s: TSocket; name: PSockAddr; namelen: Longint): Longint; stdcall;external WINSOCK2_DLL name 'connect';
function connect( const s: TSocket; const name: TSockAddr; namelen: Longint): Longint; stdcall;external WINSOCK2_DLL name 'connect';
function ioctlsocket( const s: TSocket; cmd: Longint; var arg: u_long ): Longint; stdcall;external WINSOCK2_DLL name 'ioctlsocket';
function ioctlsocket( const s: TSocket; cmd: Longint; argp: pu_long ): Longint; stdcall;external WINSOCK2_DLL name 'ioctlsocket';
function getpeername( const s: TSocket; var name: TSockAddr; var namelen: Longint ): Longint; stdcall;external WINSOCK2_DLL name 'getpeername';
function getsockname( const s: TSocket; var name: TSockAddr; var namelen: Longint ): Longint; stdcall;external WINSOCK2_DLL name 'getsockname';
function getsockopt( const s: TSocket; const level, optname: Longint; optval: PChar; var optlen: Longint ): Longint; stdcall;external WINSOCK2_DLL name 'getsockopt';
function getsockopt( const s: TSocket; const level, optname: Longint; optval: Pointer; var optlen: Longint ): Longint; stdcall;external WINSOCK2_DLL name 'getsockopt';
function getsockopt( const s: TSocket; const level, optname: Longint; var optval; var optlen: Longint ): Longint; stdcall;external WINSOCK2_DLL name 'getsockopt';
function htonl(hostlong: u_long): u_long; stdcall;external WINSOCK2_DLL name 'htonl';
function htons(hostshort: u_short): u_short; stdcall;external WINSOCK2_DLL name 'htons';
function inet_addr(cp: PChar): u_long; stdcall;external WINSOCK2_DLL name 'inet_addr';
function inet_ntoa(inaddr: TInAddr): PChar; stdcall;external WINSOCK2_DLL name 'inet_ntoa';
function listen(s: TSocket; backlog: Longint): Longint; stdcall;external WINSOCK2_DLL name 'listen';
function ntohl(netlong: u_long): u_long; stdcall;external WINSOCK2_DLL name 'ntohl';
function ntohs(netshort: u_short): u_short; stdcall;external WINSOCK2_DLL name 'ntohs';
function recv(s: TSocket; var Buf; len, flags: Longint): Longint; stdcall;external WINSOCK2_DLL name 'recv';
function recv(s: TSocket; Buf: PChar; len, flags: Longint): Longint; stdcall;external WINSOCK2_DLL name 'recv';
function recv(s: TSocket; Buf: Pointer; len, flags: Longint): Longint; stdcall;external WINSOCK2_DLL name 'recv';
function recvfrom(s: TSocket; Buf: PChar; len, flags: Longint; from: PSockAddr; fromlen: PLongint): Longint; stdcall;external WINSOCK2_DLL name 'recvfrom';
function recvfrom(s: TSocket; Buf: Pointer; len, flags: Longint; from: PSockAddr; fromlen: PLongint): Longint; stdcall;external WINSOCK2_DLL name 'recvfrom';
function recvfrom(s: TSocket; var Buf; len, flags: Longint; const from: TSockAddr; var fromlen: Longint): Longint; stdcall;external WINSOCK2_DLL name 'recvfrom';
function select(nfds: Longint; readfds, writefds, exceptfds: PFDSet; timeout: PTimeVal): Longint; stdcall;external WINSOCK2_DLL name 'select';
function send(s: TSocket; const Buf; len, flags: Longint): Longint; stdcall;external WINSOCK2_DLL name 'send';
function send(s: TSocket; Buf: PChar; len, flags: Longint): Longint; stdcall;external WINSOCK2_DLL name 'send';
function send(s: TSocket; Buf: Pointer; len, flags: Longint): Longint; stdcall;external WINSOCK2_DLL name 'send';
function sendto(s: TSocket; const Buf; len, flags: Longint; const addrto: TSockAddr; tolen: Longint): Longint; stdcall;external WINSOCK2_DLL name 'sendto';
function sendto(s: TSocket; Buf: PChar; len, flags: Longint; addrto: PSockAddr; tolen: Longint): Longint; stdcall;external WINSOCK2_DLL name 'sendto';
function sendto(s: TSocket; Buf: Pointer; len, flags: Longint; addrto: PSockAddr; tolen: Longint): Longint; stdcall;external WINSOCK2_DLL name 'sendto';
function setsockopt(s: TSocket; level, optname: Longint; const optval; optlen: Longint): Longint; stdcall;external WINSOCK2_DLL name 'setsockopt';
function setsockopt(s: TSocket; level, optname: Longint; optval: PChar; optlen: Longint): Longint; stdcall;external WINSOCK2_DLL name 'setsockopt';
function setsockopt(s: TSocket; level, optname: Longint; optval: Pointer; optlen: Longint): Longint; stdcall;external WINSOCK2_DLL name 'setsockopt';
function shutdown(s: TSocket; how: Longint): Longint; stdcall;external WINSOCK2_DLL name 'shutdown';
function socket(af, struct, protocol: Longint): TSocket; stdcall;external WINSOCK2_DLL name 'socket';

function gethostbyaddr(addr: Pointer; len, struct: Longint): PHostEnt; stdcall;external WINSOCK2_DLL name 'gethostbyaddr';
function gethostbyname(name: PChar): PHostEnt; stdcall;external WINSOCK2_DLL name 'gethostbyname';
function gethostname(name: PChar; len: Longint): Longint; stdcall;external WINSOCK2_DLL name 'gethostname';
function getservbyport(port: Longint; proto: PChar): PServEnt; stdcall;external WINSOCK2_DLL name 'getservbyport';
function getservbyname(name, proto: PChar): PServEnt; stdcall;external WINSOCK2_DLL name 'getservbyname';
function getprotobynumber(proto: Longint): PProtoEnt; stdcall;external WINSOCK2_DLL name 'getprotobynumber';
function getprotobyname(name: PChar): PProtoEnt; stdcall;external WINSOCK2_DLL name 'getprotobyname';

function WSAStartup(wVersionRequired: word; var WSData: TWSAData): Longint; stdcall; external WINSOCK2_DLL name 'WSAStartup';
function WSACleanup: Longint; stdcall; external WINSOCK2_DLL name 'WSACleanup';
procedure WSASetLastError(iError: Longint); stdcall; external WINSOCK2_DLL name 'WSASetLastError';
function WSAGetLastError: Longint; stdcall; external WINSOCK2_DLL name 'WSAGetLastError';
function WSAIsBlocking: BOOL; stdcall; external WINSOCK2_DLL name 'WSAIsBlocking';
function WSAUnhookBlockingHook: Longint; stdcall; external WINSOCK2_DLL name 'WSAUnhookBlockingHook';
function WSASetBlockingHook(lpBlockFunc: TFarProc): TFarProc; stdcall; external WINSOCK2_DLL name 'WSASetBlockingHook';
function WSACancelBlockingCall: Longint; stdcall; external WINSOCK2_DLL name 'WSACancelBlockingCall';
function WSAAsyncGetServByName(HWindow: HWND; wMsg: u_int; name, proto, buf: PChar; buflen: Longint): THandle; stdcall; external WINSOCK2_DLL name 'WSAAsyncGetServByName';
function WSAAsyncGetServByPort( HWindow: HWND; wMsg, port: u_int; proto, buf: PChar; buflen: Longint): THandle; stdcall; external WINSOCK2_DLL name 'WSAAsyncGetServByPort';
function WSAAsyncGetProtoByName(HWindow: HWND; wMsg: u_int; name, buf: PChar; buflen: Longint): THandle; stdcall; external WINSOCK2_DLL name 'WSAAsyncGetProtoByName';
function WSAAsyncGetProtoByNumber(HWindow: HWND; wMsg: u_int; number: Longint; buf: PChar; buflen: Longint): THandle; stdcall; external WINSOCK2_DLL name 'WSAAsyncGetProtoByNumber';
function WSAAsyncGetHostByName(HWindow: HWND; wMsg: u_int; name, buf: PChar; buflen: Longint): THandle; stdcall; external WINSOCK2_DLL name 'WSAAsyncGetHostByName';
function WSAAsyncGetHostByAddr(HWindow: HWND; wMsg: u_int; addr: PChar; len, struct: Longint; buf: PChar; buflen: Longint): THandle; stdcall; external WINSOCK2_DLL name 'WSAAsyncGetHostByAddr';
function WSACancelAsyncRequest(hAsyncTaskHandle: THandle): Longint; stdcall; external WINSOCK2_DLL name 'WSACancelAsyncRequest';
function WSAAsyncSelect(s: TSocket; HWindow: HWND; wMsg: u_int; lEvent: Longint): Longint; stdcall; external WINSOCK2_DLL name 'WSAAsyncSelect';
function __WSAFDIsSet(s: TSOcket; var FDSet: TFDSet): Bool; stdcall; external WINSOCK2_DLL name '__WSAFDIsSet';

{       WinSock 2 API new function prototypes }
function WSAAccept( s : TSocket; addr : TSockAddr; addrlen : PLongint; lpfnCondition : LPCONDITIONPROC; dwCallbackData : DWORD ): TSocket; stdcall; external WINSOCK2_DLL name 'WSAAccept';
function WSACloseEvent( hEvent : WSAEVENT) : WordBool; stdcall; external WINSOCK2_DLL name 'WSACloseEvent';
function WSAConnect( s : TSocket; const name : PSockAddr; namelen : Longint; lpCallerData,lpCalleeData : LPWSABUF; lpSQOS,lpGQOS : LPQOS ) : Longint; stdcall; external WINSOCK2_DLL name 'WSAConnect';
function WSACreateEvent : WSAEVENT; stdcall; external WINSOCK2_DLL name 'WSACreateEvent';
function WSADuplicateSocketA( s : TSocket; dwProcessId : DWORD; lpProtocolInfo : LPWSAProtocol_InfoA ) : Longint; stdcall; external WINSOCK2_DLL name 'WSADuplicateSocketA';
function WSADuplicateSocketW( s : TSocket; dwProcessId : DWORD; lpProtocolInfo : LPWSAProtocol_InfoW ) : Longint; stdcall; external WINSOCK2_DLL name 'WSADuplicateSocketW';
function WSADuplicateSocket( s : TSocket; dwProcessId : DWORD; lpProtocolInfo : LPWSAProtocol_Info ) : Longint; stdcall; external WINSOCK2_DLL name 'WSADuplicateSocket';
function WSAEnumNetworkEvents( const s : TSocket; const hEventObject : WSAEVENT; lpNetworkEvents : LPWSANETWORKEVENTS ) :Longint; stdcall; external WINSOCK2_DLL name 'WSAEnumNetworkEvents';
function WSAEnumProtocolsA( lpiProtocols : PLongint; lpProtocolBuffer : LPWSAProtocol_InfoA; var lpdwBufferLength : DWORD ) : Longint; stdcall; external WINSOCK2_DLL name 'WSAEnumProtocolsA';
function WSAEnumProtocolsW( lpiProtocols : PLongint; lpProtocolBuffer : LPWSAProtocol_InfoW; var lpdwBufferLength : DWORD ) : Longint; stdcall; external WINSOCK2_DLL name 'WSAEnumProtocolsW';
function WSAEnumProtocols( lpiProtocols : PLongint; lpProtocolBuffer : LPWSAProtocol_Info; var lpdwBufferLength : DWORD ) : Longint; stdcall; external WINSOCK2_DLL name 'WSAEnumProtocols';
function WSAEventSelect( s : TSocket; hEventObject : WSAEVENT; lNetworkEvents : LongInt ): Longint; stdcall; external WINSOCK2_DLL name 'WSAEventSelect';
function WSAGetOverlappedResult( s : TSocket; lpOverlapped : LPWSAOVERLAPPED; lpcbTransfer : LPDWORD; fWait : BOOL; var lpdwFlags : DWORD ) : WordBool; stdcall; external WINSOCK2_DLL name 'WSAGetOverlappedResult';
function WSAGetQosByName( s : TSocket; lpQOSName : LPWSABUF; lpQOS : LPQOS ): WordBool; stdcall; external WINSOCK2_DLL name 'WSAGetQosByName';
function WSAhtonl( s : TSocket; hostlong : u_long; var lpnetlong : DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSAhtonl';
function WSAhtons( s : TSocket; hostshort : u_short; var lpnetshort : WORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSAhtons';
function WSAIoctl( s : TSocket; dwIoControlCode : DWORD; lpvInBuffer : Pointer; cbInBuffer : DWORD; lpvOutBuffer : Pointer; cbOutBuffer : DWORD;
        lpcbBytesReturned : LPDWORD; lpOverlapped : LPWSAOVERLAPPED; lpCompletionRoutine : LPWSAOVERLAPPED_COMPLETION_ROUTINE ) : Longint; stdcall; external WINSOCK2_DLL name 'WSAIoctl';
function WSAJoinLeaf( s : TSocket; name : PSockAddr; namelen : Longint; lpCallerData,lpCalleeData : LPWSABUF;
        lpSQOS,lpGQOS : LPQOS; dwFlags : DWORD ) : TSocket; stdcall; external WINSOCK2_DLL name 'WSAJoinLeaf';
function WSANtohl( s : TSocket; netlong : u_long; var lphostlong : DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSANtohl';
function WSANtohs( s : TSocket; netshort : u_short; var lphostshort : WORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSANtohs';
function WSARecv( s : TSocket; lpBuffers : LPWSABUF; dwBufferCount : DWORD; var lpNumberOfBytesRecvd : DWORD; var lpFlags : DWORD;
        lpOverlapped : LPWSAOVERLAPPED; lpCompletionRoutine : LPWSAOVERLAPPED_COMPLETION_ROUTINE ): Longint; stdcall; external WINSOCK2_DLL name 'WSARecv';
function WSARecvDisconnect( s : TSocket; lpInboundDisconnectData : LPWSABUF ): Longint; stdcall; external WINSOCK2_DLL name 'WSARecvDisconnect';
function WSARecvFrom( s : TSocket; lpBuffers : LPWSABUF; dwBufferCount : DWORD; var lpNumberOfBytesRecvd : DWORD; var lpFlags : DWORD;
        lpFrom : PSockAddr; lpFromlen : PLongint; lpOverlapped : LPWSAOVERLAPPED; lpCompletionRoutine : LPWSAOVERLAPPED_COMPLETION_ROUTINE ): Longint; stdcall; external WINSOCK2_DLL name 'WSARecvFrom';
function WSARecvMsg( s : TSocket; lpMsg : LPWSAMSG; lpdwNumberOfBytesRecvd : DWORD; lpOverlapped : LPWSAOVERLAPPED; lpCompletionRoutine : LPWSAOVERLAPPED_COMPLETION_ROUTINE) : Longint; stdcall; external WINSOCK2_DLL name 'WSARecvMsg';
function WSAResetEvent( hEvent : WSAEVENT ): WordBool; stdcall; external WINSOCK2_DLL name 'WSAResetEvent';
function WSASend( s : TSocket; lpBuffers : LPWSABUF; dwBufferCount : DWORD; var lpNumberOfBytesSent : DWORD; dwFlags : DWORD;
        lpOverlapped : LPWSAOVERLAPPED; lpCompletionRoutine : LPWSAOVERLAPPED_COMPLETION_ROUTINE ): Longint; stdcall; external WINSOCK2_DLL name 'WSASend';
function WSASendDisconnect( s : TSocket; lpOutboundDisconnectData : LPWSABUF ): Longint; stdcall; external WINSOCK2_DLL name 'WSASendDisconnect';
function WSASendTo( s : TSocket; lpBuffers : LPWSABUF; dwBufferCount : DWORD; var lpNumberOfBytesSent : DWORD; dwFlags : DWORD;
        lpTo : PSockAddr; iTolen : Longint; lpOverlapped : LPWSAOVERLAPPED; lpCompletionRoutine : LPWSAOVERLAPPED_COMPLETION_ROUTINE ): Longint; stdcall; external WINSOCK2_DLL name 'WSASendTo';
function WSASendMsg( s : TSocket; lpMsg : LPWSAMSG; dwFlags : DWORD; lpNumberOfBytesSent : DWORD; lpOverlapped : LPWSAOVERLAPPED; lpCompletionRoutine : LPWSAOVERLAPPED_COMPLETION_ROUTINE) : Longint; stdcall; external WINSOCK2_DLL name 'WSASendMsg';
function WSASetEvent( hEvent : WSAEVENT ): WordBool; stdcall; external WINSOCK2_DLL name 'WSASetEvent';
function WSASocketA( af, iType, protocol : Longint; lpProtocolInfo : LPWSAProtocol_InfoA; g : GROUP; dwFlags : DWORD ): TSocket; stdcall; external WINSOCK2_DLL name 'WSASocketA';
function WSASocketW( af, iType, protocol : Longint; lpProtocolInfo : LPWSAProtocol_InfoW; g : GROUP; dwFlags : DWORD ): TSocket; stdcall; external WINSOCK2_DLL name 'WSASocketW';
function WSASocket( af, iType, protocol : Longint; lpProtocolInfo : LPWSAProtocol_Info; g : GROUP; dwFlags : DWORD ): TSocket; stdcall; external WINSOCK2_DLL name 'WSASocket';
function WSAWaitForMultipleEvents( cEvents : DWORD; lphEvents : PWSAEVENT; fWaitAll : LongBool;
        dwTimeout : DWORD; fAlertable : LongBool ): DWORD; stdcall; external WINSOCK2_DLL name 'WSAWaitForMultipleEvents';
function WSAAddressToStringA( var lpsaAddress : TSockAddr; const dwAddressLength : DWORD; const lpProtocolInfo : LPWSAProtocol_InfoA;
        const lpszAddressString : PChar; var lpdwAddressStringLength : DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSAAddressToStringA';
function WSAAddressToStringW( var lpsaAddress : TSockAddr; const dwAddressLength : DWORD; const lpProtocolInfo : LPWSAProtocol_InfoW;
        const lpszAddressString : PWideChar; var lpdwAddressStringLength : DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSAAddressToStringW';
function WSAAddressToString( var lpsaAddress : TSockAddr; const dwAddressLength : DWORD; const lpProtocolInfo : LPWSAProtocol_Info;
        const lpszAddressString : PMBChar; var lpdwAddressStringLength : DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSAAddressToString';
function WSAStringToAddressA( const AddressString : PChar; const AddressFamily: Longint; const lpProtocolInfo : LPWSAProtocol_InfoA;
        var lpAddress : TSockAddr; var lpAddressLength : Longint ): Longint; stdcall; external WINSOCK2_DLL name 'WSAStringToAddressA';
function WSAStringToAddressW( const AddressString : PWideChar; const AddressFamily: Longint; const lpProtocolInfo : LPWSAProtocol_InfoA;
        var lpAddress : TSockAddr; var lpAddressLength : Longint ): Longint; stdcall; external WINSOCK2_DLL name 'WSAStringToAddressW';
function WSAStringToAddress( const AddressString : PMBChar; const AddressFamily: Longint; const lpProtocolInfo : LPWSAProtocol_Info;
        var lpAddress : TSockAddr; var lpAddressLength : Longint ): Longint; stdcall; external WINSOCK2_DLL name 'WSAStringToAddress';

{       Registration and Name Resolution API functions }
function WSALookupServiceBeginA( const lpqsRestrictions : LPWSAQuerySetA; const dwControlFlags : DWORD; lphLookup : PHANDLE ): Longint; stdcall; external WINSOCK2_DLL name 'WSALookupServiceBeginA';
function WSALookupServiceBeginW( const lpqsRestrictions : LPWSAQuerySetW; const dwControlFlags : DWORD; lphLookup : PHANDLE ): Longint; stdcall; external WINSOCK2_DLL name 'WSALookupServiceBeginW';
function WSALookupServiceBegin( const lpqsRestrictions : LPWSAQuerySet; const dwControlFlags : DWORD; lphLookup : PHANDLE ): Longint; stdcall; external WINSOCK2_DLL name 'WSALookupServiceBegin';
function WSALookupServiceNextA( const hLookup : THandle; const dwControlFlags : DWORD; var lpdwBufferLength : DWORD; lpqsResults : LPWSAQuerySetA ): Longint; stdcall; external WINSOCK2_DLL name 'WSALookupServiceNextA';
function WSALookupServiceNextW( const hLookup : THandle; const dwControlFlags : DWORD; var lpdwBufferLength : DWORD; lpqsResults : LPWSAQuerySetW ): Longint; stdcall; external WINSOCK2_DLL name 'WSALookupServiceNextW';
function WSALookupServiceNext( const hLookup : THandle; const dwControlFlags : DWORD; var lpdwBufferLength : DWORD; lpqsResults : LPWSAQuerySet ): Longint; stdcall; external WINSOCK2_DLL name 'WSALookupServiceNext';
function WSALookupServiceEnd( const hLookup : THandle ): Longint; stdcall; external WINSOCK2_DLL name 'WSALookupServiceEnd';
function WSAInstallServiceClassA( const lpServiceClassInfo : LPWSAServiceClassInfoA ) : Longint; stdcall; external WINSOCK2_DLL name 'WSAInstallServiceClassA';
function WSAInstallServiceClassW( const lpServiceClassInfo : LPWSAServiceClassInfoW ) : Longint; stdcall; external WINSOCK2_DLL name 'WSAInstallServiceClassW';
function WSAInstallServiceClass( const lpServiceClassInfo : LPWSAServiceClassInfo ) : Longint; stdcall; external WINSOCK2_DLL name 'WSAInstallServiceClass';
function WSARemoveServiceClass( const lpServiceClassId : PGUID ) : Longint; stdcall; external WINSOCK2_DLL name 'WSARemoveServiceClass';
function WSAGetServiceClassInfoA( const lpProviderId : PGUID; const lpServiceClassId : PGUID; var lpdwBufSize : DWORD;
        lpServiceClassInfo : LPWSAServiceClassInfoA ): Longint; stdcall; external WINSOCK2_DLL name 'WSAGetServiceClassInfoA';
function WSAGetServiceClassInfoW( const lpProviderId : PGUID; const lpServiceClassId : PGUID; var lpdwBufSize : DWORD;
        lpServiceClassInfo : LPWSAServiceClassInfoW ): Longint; stdcall; external WINSOCK2_DLL name 'WSAGetServiceClassInfoW';
function WSAGetServiceClassInfo( const lpProviderId : PGUID; const lpServiceClassId : PGUID; var lpdwBufSize : DWORD;
        lpServiceClassInfo : LPWSAServiceClassInfo ): Longint; stdcall; external WINSOCK2_DLL name 'WSAGetServiceClassInfo';
function WSAEnumNameSpaceProvidersA( var lpdwBufferLength: DWORD; const lpnspBuffer: LPWSANameSpace_InfoA ): Longint; stdcall; external WINSOCK2_DLL name 'WSAEnumNameSpaceProvidersA';
function WSAEnumNameSpaceProvidersW( var lpdwBufferLength: DWORD; const lpnspBuffer: LPWSANameSpace_InfoW ): Longint; stdcall; external WINSOCK2_DLL name 'WSAEnumNameSpaceProvidersW';
function WSAEnumNameSpaceProviders( var lpdwBufferLength: DWORD; const lpnspBuffer: LPWSANameSpace_Info ): Longint; stdcall; external WINSOCK2_DLL name 'WSAEnumNameSpaceProviders';
function WSAGetServiceClassNameByClassIdA( const lpServiceClassId: PGUID; lpszServiceClassName: PChar;
        var lpdwBufferLength: DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSAGetServiceClassNameByClassIdA';
function WSAGetServiceClassNameByClassIdW( const lpServiceClassId: PGUID; lpszServiceClassName: PWideChar;
        var lpdwBufferLength: DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSAGetServiceClassNameByClassIdW';
function WSAGetServiceClassNameByClassId( const lpServiceClassId: PGUID; lpszServiceClassName: PMBChar;
        var lpdwBufferLength: DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSAGetServiceClassNameByClassId';
function WSASetServiceA( const lpqsRegInfo: LPWSAQuerySetA; const essoperation: TWSAeSetServiceOp;
        const dwControlFlags: DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSASetServiceA';
function WSASetServiceW( const lpqsRegInfo: LPWSAQuerySetW; const essoperation: TWSAeSetServiceOp;
        const dwControlFlags: DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSASetServiceW';
function WSASetService( const lpqsRegInfo: LPWSAQuerySet; const essoperation: TWSAeSetServiceOp;
        const dwControlFlags: DWORD ): Longint; stdcall; external WINSOCK2_DLL name 'WSASetService';

{ Macros }
function WSAMakeSyncReply(Buflen, Error: Word): Longint;
function WSAMakeSelectReply(Event, Error: Word): Longint;
function WSAGetAsyncBuflen(Param: Longint): Word;
function WSAGetAsyncError(Param: Longint): Word;
function WSAGetSelectEvent(Param: Longint): Word;
function WSAGetSelectError(Param: Longint): Word;

procedure FD_CLR(Socket: TSocket; var FDSet: TFDSet);
function FD_ISSET(Socket: TSocket; var FDSet: TFDSet): Boolean;
procedure FD_SET(Socket: TSocket; var FDSet: TFDSet);
procedure FD_ZERO(var FDSet: TFDSet);

//=============================================================
implementation
//=============================================================

function WSAMakeSyncReply(Buflen, Error: Word): Longint;
begin
  WSAMakeSyncReply:= MakeLong(Buflen, Error);
end;

function WSAMakeSelectReply(Event, Error: Word): Longint;
begin
  WSAMakeSelectReply:= MakeLong(Event, Error);
end;

function WSAGetAsyncBuflen(Param: Longint): Word;
begin
  WSAGetAsyncBuflen:= LOWORD(Param);
end;

function WSAGetAsyncError(Param: Longint): Word;
begin
  WSAGetAsyncError:= HIWORD(Param);
end;

function WSAGetSelectEvent(Param: Longint): Word;
begin
  WSAGetSelectEvent:= LOWORD(Param);
end;

function WSAGetSelectError(Param: Longint): Word;
begin
  WSAGetSelectError:= HIWORD(Param);
end;

procedure FD_CLR(Socket: TSocket; var FDSet: TFDSet);
var
  I: cardinal;
begin
  I := 0;
  while I < FDSet.fd_count do
  begin
    if FDSet.fd_array[I] = Socket then
    begin
      while I < FDSet.fd_count - 1 do
      begin
        FDSet.fd_array[I] := FDSet.fd_array[I + 1];
        Inc(I);
      end;
      Dec(FDSet.fd_count);
      Break;
    end;
    Inc(I);
  end;
end;

function FD_ISSET(Socket: TSocket; var FDSet: TFDSet): Boolean;
begin
  FD_ISSET := __WSAFDIsSet(Socket, FDSet);
end;

procedure FD_SET(Socket: TSocket; var FDSet: TFDSet);
begin
  if FDSet.fd_count < FD_SETSIZE then
  begin
    FDSet.fd_array[FDSet.fd_count] := Socket;
    Inc(FDSet.fd_count);
  end;
end;

procedure FD_ZERO(var FDSet: TFDSet);
begin
  FDSet.fd_count := 0;
end;

end.
