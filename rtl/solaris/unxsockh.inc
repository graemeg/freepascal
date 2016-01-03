{
   This file is part of the Free Pascal run time library.
   (c) 2000-2003 by Marco van de Voort
   member of the Free Pascal development team.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   OS dependant part of the header.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

{$linklib libsocket.so}
{$linklib libnsl.so}

const
  {
   * Address families.
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
  { XEROX NS protocols  }
     AF_NS = 6;
  { nbs protocols  }
     AF_NBS = 7;
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
  { Apple Talk  }
     AF_APPLETALK = 16;
  { Network Interface Tap  }
     AF_NIT = 17;
  { IEEE 802.2, also ISO 8802  }
     AF_802 = 18;
  { umbrella for all families used  }
     AF_OSI = 19;
  { CCITT X.25 in particular  }
     AF_X25 = 20;
  { AFI = 47, IDI = 4  }
     AF_OSINET = 21;
  { U.S. Government OSI  }
     AF_GOSIP = 22;
  { Novell Internet Protocol  }
     AF_IPX = 23;
  { Internal Routing Protocol  }
     AF_ROUTE = 24;
  { Link-layer interface  }
     AF_LINK = 25;
  { Internet Protocol, Version 6  }
     AF_INET6 = 26;
  { Security Association DB socket  }
     AF_KEY = 27;
  { NCA socket  }
     AF_NCA = 28;
  { Security Policy DB socket  }
     AF_POLICY = 29;
     AF_MAX = 29;
  {
   * Protocol families, same as address families for now.
    }
     PF_UNSPEC = AF_UNSPEC;
     PF_UNIX = AF_UNIX;
     PF_INET = AF_INET;
     PF_IMPLINK = AF_IMPLINK;
     PF_PUP = AF_PUP;
     PF_CHAOS = AF_CHAOS;
     PF_NS = AF_NS;
     PF_NBS = AF_NBS;
     PF_ECMA = AF_ECMA;
     PF_DATAKIT = AF_DATAKIT;
     PF_CCITT = AF_CCITT;
     PF_SNA = AF_SNA;
     PF_DECnet = AF_DECnet;
     PF_DLI = AF_DLI;
     PF_LAT = AF_LAT;
     PF_HYLINK = AF_HYLINK;
     PF_APPLETALK = AF_APPLETALK;
     PF_NIT = AF_NIT;
     PF_802 = AF_802;
     PF_OSI = AF_OSI;
     PF_X25 = AF_X25;
     PF_OSINET = AF_OSINET;
     PF_GOSIP = AF_GOSIP;
     PF_IPX = AF_IPX;
     PF_ROUTE = AF_ROUTE;
     PF_LINK = AF_LINK;
     PF_INET6 = AF_INET6;
     PF_KEY = AF_KEY;
     PF_NCA = AF_NCA;
     PF_POLICY = AF_POLICY;
     PF_MAX = AF_MAX;

{ Maximum queue length specifiable by listen.  }
  SOMAXCONN     = 128;

   SOL_SOCKET = $FFFF;
	
   SO_DEBUG        = $0001;          { turn on debugging info recording }
   SO_ACCEPTCONN   = $0002;          { socket has had listen() }
   SO_REUSEADDR    = $0004;          { allow local address reuse }
   SO_KEEPALIVE    = $0008;          { keep connections alive }
   SO_DONTROUTE    = $0010;          { just use interface addresses }
   SO_BROADCAST    = $0020;          { permit sending of broadcast msgs }
   SO_USELOOPBACK  = $0040;          { bypass hardware when possible }
   SO_LINGER       = $0080;          { linger on close if data present }
   SO_OOBINLINE    = $0100;          { leave received OOB data in line }
   SO_DGRAM_ERRIND = $0200;          { Application wants delayed error }

   SO_DONTLINGER   = not SO_LINGER;    { ~SO_LINGER }

   { * Additional options, not kept in so_options. }
   SO_SNDBUF      = $1001;          { send buffer size }
   SO_RCVBUF      = $1002;          { receive buffer size }
   SO_SNDLOWAT    = $1003;          { send low-water mark }
   SO_RCVLOWAT    = $1004;          { receive low-water mark }
   SO_SNDTIMEO    = $1005;          { send timeout }
   SO_RCVTIMEO    = $1006;          { receive timeout }
   SO_ERROR       = $1007;          { get error status and clear }
   SO_TYPE        = $1008;          { get socket type }
   SO_PROTOTYPE   = $1009;          { get/set protocol type }

   { "Socket"-level control message types: }
   SCM_RIGHTS     = $1010;          { access rights (array of int) }

   SO_STATE       = $2000;          { Internal: get so_state }

   SHUT_RD         = 0;             { shut down the reading side }
   SHUT_WR         = 1;             { shut down the writing side }
   SHUT_RDWR       = 2;             { shut down both sides }

  { Flags for send, recv etc. }
  MSG_OOB          = $0001;              { Process out-of-band data}
  MSG_PEEK         = $0002;              { Peek at incoming messages }
  MSG_DONTROUTE    = $0004;              { Don't use local routing }
  MSG_EOR          = $0008;              { End of record }
  MSG_CTRUNC       = $0010;              { Control data lost before delivery }
  MSG_TRUNC        = $0020;
  MSG_WAITALL      = $0040;              { Wait for a full request }
  MSG_DONTWAIT     = $0080;              { Non-blocking I/O }
  MSG_NOTIFICATION = $0100;           { Notification, not data }
  MSG_XPG4_2       = $8000;	      { Private: XPG4.2 flag }
  MSG_MAXIOVLEN    =    16;
  
     TCP_NODELAY = $01;
  { set maximum segment size  }
     TCP_MAXSEG = $02;
  { set keepalive timer  }
     TCP_KEEPALIVE = $8;
     TCP_NOTIFY_THRESHOLD = $10;
     TCP_ABORT_THRESHOLD = $11;
     TCP_CONN_NOTIFY_THRESHOLD = $12;
     TCP_CONN_ABORT_THRESHOLD = $13;
     TCP_RECVDSTADDR = $14;
     TCP_INIT_CWND = $15;
     TCP_KEEPALIVE_THRESHOLD = $16;
     TCP_KEEPALIVE_ABORT_THRESHOLD = $17;
     TCP_CORK = $18;

