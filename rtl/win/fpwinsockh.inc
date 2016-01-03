{ 
  Should be aliases but I copy pasted them from winsock2 for now...
}

const
{ Address families. }
  AF_UNSPEC       = winsock2.AF_UNSPEC;               { unspecified }
  AF_UNIX         = winsock2.AF_UNIX;               { local to host (pipes, portals) }
  AF_INET         = winsock2.AF_INET;               { internetwork: UDP, TCP, etc. }
  AF_IMPLINK      = winsock2.AF_IMPLINK;               { arpanet imp addresses }
  AF_PUP          = winsock2.AF_PUP;               { pup protocols: e.g. BSP }
  AF_CHAOS        = winsock2.AF_CHAOS;               { mit CHAOS protocols }
  AF_IPX          = winsock2.AF_IPX;               { IPX and SPX }
  AF_NS           = winsock2.AF_NS;               { XEROX NS protocols }
  AF_ISO          = winsock2.AF_ISO;               { ISO protocols }
  AF_OSI          = winsock2.AF_OSI;          { OSI is ISO }
  AF_ECMA         = winsock2.AF_ECMA;               { european computer manufacturers }
  AF_DATAKIT      = winsock2.AF_DATAKIT;               { datakit protocols }
  AF_CCITT        = winsock2.AF_CCITT;              { CCITT protocols, X.25 etc }
  AF_SNA          = winsock2.AF_SNA;              { IBM SNA }
  AF_DECnet       = winsock2.AF_DECnet;              { DECnet }
  AF_DLI          = winsock2.AF_DLI;              { Direct data link interface }
  AF_LAT          = winsock2.AF_LAT;              { LAT }
  AF_HYLINK       = winsock2.AF_HYLINK;              { NSC Hyperchannel }
  AF_APPLETALK    = winsock2.AF_APPLETALK;              { AppleTalk }
  AF_NETBIOS      = winsock2.AF_NETBIOS;              { NetBios-style addresses }
  AF_VOICEVIEW    = winsock2.AF_VOICEVIEW;              { VoiceView }
  AF_FIREFOX      = winsock2.AF_FIREFOX;              { FireFox }
  AF_UNKNOWN1     = winsock2.AF_UNKNOWN1;              { Somebody is using this! }
  AF_BAN          = winsock2.AF_BAN;              { Banyan }
  AF_ATM          = winsock2.AF_ATM; // Native ATM Services
  AF_INET6        = winsock2.AF_INET6; // Internetwork Version 6
  AF_MAX          = winsock2.AF_MAX;

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

  PF_MAX          = AF_MAX;

{ Protocols }
  IPPROTO_IP     =  winsock2.IPPROTO_IP;
  IPPROTO_ICMP   =  winsock2.IPPROTO_ICMP;
  IPPROTO_IGMP   =  winsock2.IPPROTO_IGMP;
  IPPROTO_GGP    =  winsock2.IPPROTO_GGP;
  IPPROTO_TCP    =  winsock2.IPPROTO_TCP;
  IPPROTO_PUP    =  winsock2.IPPROTO_PUP;
  IPPROTO_UDP    =  winsock2.IPPROTO_UDP;
  IPPROTO_IDP    =  winsock2.IPPROTO_IDP;
  IPPROTO_ND     =  winsock2.IPPROTO_ND;

  IPPROTO_RAW    =  winsock2.IPPROTO_RAW;
  IPPROTO_MAX    =  winsock2.IPPROTO_MAX;

{ Port/socket numbers: network standard functions}

  IPPORT_ECHO    =  winsock2.IPPORT_ECHO;
  IPPORT_DISCARD =  winsock2.IPPORT_DISCARD;
  IPPORT_SYSTAT  =  winsock2.IPPORT_SYSTAT;
  IPPORT_DAYTIME =  winsock2.IPPORT_DAYTIME;
  IPPORT_NETSTAT =  winsock2.IPPORT_NETSTAT;
  IPPORT_FTP     =  winsock2.IPPORT_FTP;
  IPPORT_TELNET  =  winsock2.IPPORT_TELNET;
  IPPORT_SMTP    =  winsock2.IPPORT_SMTP;
  IPPORT_TIMESERVER  =  winsock2.IPPORT_TIMESERVER;
  IPPORT_NAMESERVER  =  winsock2.IPPORT_NAMESERVER;
  IPPORT_WHOIS       =  winsock2.IPPORT_WHOIS;
  IPPORT_MTP         =  winsock2.IPPORT_MTP;

{ Port/socket numbers: host specific functions }

  IPPORT_TFTP        =  winsock2.IPPORT_TFTP;
  IPPORT_RJE         =  winsock2.IPPORT_RJE;
  IPPORT_FINGER      =  winsock2.IPPORT_FINGER;
  IPPORT_TTYLINK     =  winsock2.IPPORT_TTYLINK;
  IPPORT_SUPDUP      =  winsock2.IPPORT_SUPDUP;

{ UNIX TCP sockets }

  IPPORT_EXECSERVER  =  winsock2.IPPORT_EXECSERVER;
  IPPORT_LOGINSERVER =  winsock2.IPPORT_LOGINSERVER;
  IPPORT_CMDSERVER   =  winsock2.IPPORT_CMDSERVER;
  IPPORT_EFSSERVER   =  winsock2.IPPORT_EFSSERVER;

{ UNIX UDP sockets }

  IPPORT_BIFFUDP     =  winsock2.IPPORT_BIFFUDP;
  IPPORT_WHOSERVER   =  winsock2.IPPORT_WHOSERVER;
  IPPORT_ROUTESERVER =  winsock2.IPPORT_ROUTESERVER;

{ Ports < IPPORT_RESERVED are reserved for
  privileged processes (e.g. root). }

  IPPORT_RESERVED    =  winsock2.IPPORT_RESERVED;

{ Options for use with [gs]etsockopt at the IP level. }

  IP_OPTIONS          = winsock2.IP_OPTIONS;
  IP_MULTICAST_IF     = winsock2.IP_MULTICAST_IF;
  IP_MULTICAST_TTL    = winsock2.IP_MULTICAST_TTL;
  IP_MULTICAST_LOOP   = winsock2.IP_MULTICAST_LOOP;
  IP_ADD_MEMBERSHIP   = winsock2.IP_ADD_MEMBERSHIP;
  IP_DROP_MEMBERSHIP  = winsock2.IP_DROP_MEMBERSHIP;
  IP_TTL              = winsock2.IP_TTL;
  IP_TOS              = winsock2.IP_TOS;
  IP_DONTFRAGMENT     = winsock2.IP_DONTFRAGMENT;

{ This is used instead of -1, since the
  TSocket type is unsigned.}
{
  Tsocket might be unsigned, but the sockets unit uses
  signed types. So, we cannot use these constants, signed
  ones are defined in socketh.inc.
  INVALID_SOCKET                = winsock2.INVALID_SOCKET;
  SOCKET_ERROR                  = winsock2.SOCKET_ERROR;
}

{ Option flags per-socket. }

  SO_DEBUG        = winsock2.SO_DEBUG;
  SO_ACCEPTCONN   = winsock2.SO_ACCEPTCONN;
  SO_REUSEADDR    = winsock2.SO_REUSEADDR;
  SO_KEEPALIVE    = winsock2.SO_KEEPALIVE;
  SO_DONTROUTE    = winsock2.SO_DONTROUTE;
  SO_BROADCAST    = winsock2.SO_BROADCAST;
  SO_USELOOPBACK  = winsock2.SO_USELOOPBACK;
  SO_LINGER       = winsock2.SO_LINGER;
  SO_OOBINLINE    = winsock2.SO_OOBINLINE;

  SO_DONTLINGER  =  winsock2.SO_DONTLINGER;

{ Additional options. }

  SO_SNDBUF       = winsock2.SO_SNDBUF;
  SO_RCVBUF       = winsock2.SO_RCVBUF;
  SO_SNDLOWAT     = winsock2.SO_SNDLOWAT;
  SO_RCVLOWAT     = winsock2.SO_RCVLOWAT;
  SO_SNDTIMEO     = winsock2.SO_SNDTIMEO;
  SO_RCVTIMEO     = winsock2.SO_RCVTIMEO;
  SO_ERROR        = winsock2.SO_ERROR;
  SO_TYPE         = winsock2.SO_TYPE;

{ Options for connect and disconnect data and options.  Used only by
  non-TCP/IP transports such as DECNet, OSI TP4, etc. }

  SO_CONNDATA     = winsock2.SO_CONNDATA;
  SO_CONNOPT      = winsock2.SO_CONNOPT;
  SO_DISCDATA     = winsock2.SO_DISCDATA;
  SO_DISCOPT      = winsock2.SO_DISCOPT;
  SO_CONNDATALEN  = winsock2.SO_CONNDATALEN;
  SO_CONNOPTLEN   = winsock2.SO_CONNOPTLEN;
  SO_DISCDATALEN  = winsock2.SO_DISCDATALEN;
  SO_DISCOPTLEN   = winsock2.SO_DISCOPTLEN;

{ Option for opening sockets for synchronous access. }
  SO_OPENTYPE     = winsock2.SO_OPENTYPE;
  SO_SYNCHRONOUS_ALERT    = winsock2.SO_SYNCHRONOUS_ALERT;
  SO_SYNCHRONOUS_NONALERT = winsock2.SO_SYNCHRONOUS_NONALERT;

{ Other NT-specific options. }
  SO_MAXDG        = winsock2.SO_MAXDG;
  SO_MAXPATHDG    = winsock2.SO_MAXPATHDG;
  SO_UPDATE_ACCEPT_CONTEXT     = winsock2.SO_UPDATE_ACCEPT_CONTEXT;
  SO_CONNECT_TIME = winsock2.SO_CONNECT_TIME;

{ TCP options. }
  TCP_NODELAY     = winsock2.TCP_NODELAY;
  TCP_BSDURGENT   = winsock2.TCP_BSDURGENT;

  MSG_OOB         = winsock2.MSG_OOB;
  MSG_PEEK        = winsock2.MSG_PEEK;
  MSG_DONTROUTE   = winsock2.MSG_DONTROUTE;

{ WinSock 2 extension -- new flags for WSASend(), WSASendTo(), WSARecv() and WSARecvFrom() }
  MSG_INTERRUPT = winsock2.MSG_INTERRUPT;
  MSG_MAXIOVLEN = winsock2.MSG_MAXIOVLEN;

  MSG_PARTIAL     = winsock2.MSG_PARTIAL;

  { options for socket level  }
  SOL_SOCKET = $ffff;
  
  INVALID_SOCKET = winsock2.INVALID_SOCKET;
  SOCKET_ERROR = winsock2.SOCKET_ERROR;

