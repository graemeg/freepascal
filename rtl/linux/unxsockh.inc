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

Const
{* Supported address families. *}
  AF_UNSPEC     = 0;
  AF_UNIX       = 1;     { Unix domain sockets          }
  AF_LOCAL      = 1;     { POSIX name for AF_UNIX       }
  AF_INET       = 2;     { Internet IP Protocol         }
  AF_AX25       = 3;     { Amateur Radio AX.25          }
  AF_IPX        = 4;     { Novell IPX                   }
  AF_APPLETALK  = 5;     { AppleTalk DDP                }
  AF_NETROM     = 6;     { Amateur Radio NET/ROM        }
  AF_BRIDGE     = 7;     { Multiprotocol bridge         }
  AF_ATMPVC     = 8;     { ATM PVCs                     }
  AF_X25        = 9;     { Reserved for X.25 project    }
  AF_INET6      = 10;    { IP version 6                 }
  AF_ROSE       = 11;    { Amateur Radio X.25 PLP       }
  AF_DECnet     = 12;    { Reserved for DECnet project  }
  AF_NETBEUI    = 13;    { Reserved for 802.2LLC project}
  AF_SECURITY   = 14;    { Security callback pseudo AF }
  AF_KEY        = 15;    { PF_KEY key management API }
  AF_NETLINK    = 16;
  AF_ROUTE      = AF_NETLINK; { Alias to emulate 4.4BSD }
  AF_PACKET     = 17;    { Packet family                }
  AF_ASH        = 18;    { Ash                          }
  AF_ECONET     = 19;    { Acorn Econet                 }
  AF_ATMSVC     = 20;    { ATM SVCs                     }
  AF_SNA        = 22;	 { Linux SNA Project (nutters!) }
  AF_IRDA       = 23;    { IRDA sockets                 }
  AF_PPPOX	= 24;    { PPPoX sockets                }
  AF_WANPIPE    = 25;    { Wanpipe API Sockets }
  AF_LLC        = 26;    { Linux LLC                    }
  AF_TIPC       = 30;    { TIPC sockets                 }
  AF_BLUETOOTH  = 31;    { Bluetooth sockets            }
  AF_MAX        = 32;    { For now.. }
  
  SOCK_MAXADDRLEN = 255;             { longest possible addresses }

{
* Protocol families, same as address families for now.
}

  PF_UNSPEC     = AF_UNSPEC;
  PF_UNIX       = AF_UNIX;
  PF_LOCAL      = AF_LOCAL;
  PF_INET       = AF_INET;
  PF_AX25       = AF_AX25;
  PF_IPX        = AF_IPX;
  PF_APPLETALK  = AF_APPLETALK;
  PF_NETROM     = AF_NETROM;
  PF_BRIDGE     = AF_BRIDGE;
  PF_ATMPVC     = AF_ATMPVC;
  PF_X25        = AF_X25;
  PF_INET6      = AF_INET6;
  PF_ROSE       = AF_ROSE;
  PF_DECnet     = AF_DECnet;
  PF_NETBEUI    = AF_NETBEUI;
  PF_SECURITY   = AF_SECURITY;
  PF_KEY        = AF_KEY;
  PF_NETLINK    = AF_NETLINK;
  PF_ROUTE      = AF_ROUTE;
  PF_PACKET     = AF_PACKET;
  PF_ASH        = AF_ASH;
  PF_ECONET     = AF_ECONET;
  PF_ATMSVC     = AF_ATMSVC;
  PF_SNA        = AF_SNA;
  PF_IRDA       = AF_IRDA;
  PF_PPPOX	= AF_PPPOX;
  PF_WANPIPE    = AF_WANPIPE;
  PF_LLC        = AF_LLC;
  PF_TIPC       = AF_TIPC;
  PF_BLUETOOTH  = AF_BLUETOOTH;
  PF_MAX        = AF_MAX;


{ Maximum queue length specifiable by listen.  }
  SOMAXCONN     = 128;

{ For setsockoptions(2) }
         SOL_SOCKET  =   1;
         SO_DEBUG    =   1;
         SO_REUSEADDR=   2;
         SO_TYPE     =   3;
         SO_ERROR    =   4;
         SO_DONTROUTE=   5;
         SO_BROADCAST=   6;
         SO_SNDBUF   =   7;
         SO_RCVBUF   =   8;
         SO_KEEPALIVE=   9;
         SO_OOBINLINE=   10;
         SO_NO_CHECK =   11;
         SO_PRIORITY =   12;
         SO_LINGER   =   13;
         SO_BSDCOMPAT=   14;
{ To add :         SO_REUSEPORT 15 }
         SO_PASSCRED=    16;
         SO_PEERCRED=    17;
         SO_RCVLOWAT=    18;
         SO_SNDLOWAT=    19;
         SO_RCVTIMEO=    20;
         SO_SNDTIMEO=    21;

{ Security levels - as per NRL IPv6 - don't actually do anything }

         SO_SECURITY_AUTHENTICATION      =   22;
         SO_SECURITY_ENCRYPTION_TRANSPORT=   23;
         SO_SECURITY_ENCRYPTION_NETWORK  =   24;

         SO_BINDTODEVICE=   25;

{ Socket filtering }

         SO_ATTACH_FILTER=  26;
         SO_DETACH_FILTER=  27;
         SO_PEERNAME     =  28;
         SO_TIMESTAMP    = 29;
         SCM_TIMESTAMP   = SO_TIMESTAMP;
         SO_ACCEPTCONN   = 30;

// Following from kernel 2.6.14-1.1637_FC4

        SHUT_RD          = 0;             { shut down the reading side }
        SHUT_WR          = 1;             { shut down the writing side }
        SHUT_RDWR        = 2;             { shut down both sides }

//from /usr/include/netinet/in.h

        IPPROTO_IP       = 0;       { Dummy protocol for TCP.  }
        IPPROTO_HOPOPTS  = 0;      { IPv6 Hop-by-Hop options.  }

        IPPROTO_ICMP     = 1;       { Internet Control Message Protocol.  }
        IPPROTO_IGMP     = 2;       { Internet Group Management Protocol. }
        IPPROTO_IPIP     = 4;       { IPIP tunnels (older KA9Q tunnels use 94).  }
        IPPROTO_TCP      = 6;       { Transmission Control Protocol.  }
        IPPROTO_EGP      = 8;       { Exterior Gateway Protocol.  }

        IPPROTO_PUP      = 12;       { PUP protocol.  }
        IPPROTO_UDP      = 17;       { User Datagram Protocol.  }
        IPPROTO_IDP      = 22;       { XNS IDP protocol.  }
        IPPROTO_TP       = 29;       { SO Transport Protocol Class 4.  }
        IPPROTO_IPV6     = 41;     { IPv6 header.  }

         IPPROTO_ROUTING = 43;  { IPv6 routing header.  }
         IPPROTO_FRAGMENT = 44; { IPv6 fragmentation header.  }
         IPPROTO_RSVP    = 46;       { Reservation Protocol.  }
         IPPROTO_GRE     = 47;       { General Routing Encapsulation.  }
         IPPROTO_ESP     = 50;     { encapsulating security payload.  }
         IPPROTO_AH      = 51;     { authentication header.  }
         IPPROTO_ICMPV6  = 58;     { ICMPv6.  }
         IPPROTO_NONE    = 59;     { IPv6 no next header.  }
         IPPROTO_DSTOPTS = 60;     { IPv6 destination options.  }
         IPPROTO_MTP     = 92;       { Multicast Transport Protocol.  }
         IPPROTO_ENCAP   = 98;       { Encapsulation Header.  }
         IPPROTO_PIM     = 103;       { Protocol Independent Multicast.  }
         IPPROTO_COMP    = 108;       { Compression Header Protocol.  }
         IPPROTO_SCTP    = 132;       { Stream Control Transmission Protocol.  }
         IPPROTO_RAW     = 255;       { Raw IP packets.  }
         IPPROTO_MAX     = 255;
//from /usr/include/bits/in.h
{{ Options for use with etsockopt' and etsockopt' at the IP level.
   The first word in the comment at the right is the data type used;
   "bool" means a boolean value stored in an 	nt'.  }
}
        IP_OPTIONS              = 4;               { ip_opts; IP per-packet options.  }
        IP_HDRINCL              = 3;               { int; Header is included with data.  }
        IP_TOS                  = 1;               { int; IP type of service and precedence.  }
        IP_TTL                  = 2;               { int; IP time to live.  }
        IP_RECVOPTS             = 6;               { bool; Receive all IP options w/datagram.  }
{ For BSD compatibility.  }
        IP_RETOPTS              = 7;               { ip_opts; Set/get IP per-packet options.  }
        IP_RECVRETOPTS          = IP_RETOPTS;      { bool; Receive IP options for response.  }

        IP_MULTICAST_IF         = 32;                { in_addr; set/get IP multicast i/f }
        IP_MULTICAST_TTL        = 33;               { u_char; set/get IP multicast ttl }
        IP_MULTICAST_LOOP       = 34;              { i_char; set/get IP multicast loopback }
        IP_ADD_MEMBERSHIP       = 35;               { ip_mreq; add an IP group membership }
        IP_DROP_MEMBERSHIP      = 36;            { ip_mreq; drop an IP group membership }
        IP_UNBLOCK_SOURCE       = 37;                 { ip_mreq_source: unblock data from source }
        IP_BLOCK_SOURCE         = 38;              { ip_mreq_source: block data from source }
        IP_ADD_SOURCE_MEMBERSHIP = 39;             { ip_mreq_source: join source group }
        IP_DROP_SOURCE_MEMBERSHIP = 40;            { ip_mreq_source: leave source group }
        IP_MSFILTER             = 41;
        MCAST_JOIN_GROUP        = 42;    { group_req: join any-source group }
        MCAST_BLOCK_SOURCE      = 43;    { group_source_req: block from given group }
        MCAST_UNBLOCK_SOURCE    = 44;    { group_source_req: unblock from given group}
        MCAST_LEAVE_GROUP       = 45;    { group_req: leave any-source group }
        MCAST_JOIN_SOURCE_GROUP = 46;   { group_source_req: join source-spec gr }
        MCAST_LEAVE_SOURCE_GROUP = 47;  { group_source_req: leave source-spec gr}
        MCAST_MSFILTER          = 48;

        MCAST_EXCLUDE           = 0;
        MCAST_INCLUDE           = 1;

        IP_ROUTER_ALERT         = 5;    { bool }
        IP_PKTINFO              = 8;    { bool }
        IP_PKTOPTIONS           = 9;
        IP_PMTUDISC             = 10;    { obsolete name? }
        IP_MTU_DISCOVER         = 10;   { int; see below }
        IP_RECVERR              = 11;    { bool }
        IP_RECVTTL              = 12;   { bool }
        IP_RECVTOS              = 13;    { bool }


{ IP_MTU_DISCOVER arguments.  }
        IP_PMTUDISC_DONT        = 0;    { Never send DF frames.  }
        IP_PMTUDISC_WANT        = 1;    { Use per route hints.  }
        IP_PMTUDISC_DO          = 2;    { Always DF.  }

{ To select the IP level.  }
        SOL_IP                  = 0;

        IP_DEFAULT_MULTICAST_TTL = 1;
        IP_DEFAULT_MULTICAST_LOOP = 1;
        IP_MAX_MEMBERSHIPS       = 20;


{  Options for use with etsockopt' and etsockopt' at the IPv6 level.
   The first word in the comment at the right is the data type used;
   "bool" means a boolean value stored in an 	nt'.  }
        IPV6_ADDRFORM         = 1;
        IPV6_PKTINFO             = 2;
        IPV6_HOPOPTS             = 3;
        IPV6_DSTOPTS             = 4;
        IPV6_RTHDR          = 5;
        IPV6_RXSRCRT             = IPV6_RTHDR;
        //this may be an old name, I couldn't find it in my include files but
        //I found it with google.  It may have been depreciated because I only
        //saw it in earlier files.
        IPV6_PKTOPTIONS         = 6;
        IPV6_CHECKSUM            = 7;
        IPV6_HOPLIMIT            = 8;

        SCM_SRCRT                = IPV6_RXSRCRT;

        IPV6_NEXTHOP         = 9;
        IPV6_AUTHHDR         = 10;
        IPV6_UNICAST_HOPS        = 16;
        IPV6_MULTICAST_IF     = 17;
        IPV6_MULTICAST_HOPS     = 18;
        IPV6_MULTICAST_LOOP      = 19;
        IPV6_JOIN_GROUP             = 20;
        IPV6_LEAVE_GROUP     = 21;
        IPV6_ROUTER_ALERT     = 22;
        IPV6_MTU_DISCOVER     = 23;
        IPV6_MTU         = 24;
        IPV6_RECVERR         = 25;
        IPV6_V6ONLY         = 26;
        IPV6_JOIN_ANYCAST     = 27;
        IPV6_LEAVE_ANYCAST     = 28;
        IPV6_IPSEC_POLICY     = 34;
        IPV6_XFRM_POLICY     = 35;

{ Obsolete synonyms for the above.  }
        IPV6_ADD_MEMBERSHIP     = IPV6_JOIN_GROUP;
        IPV6_DROP_MEMBERSHIP     = IPV6_LEAVE_GROUP;
        IPV6_RXHOPOPTS         = IPV6_HOPOPTS;
        IPV6_RXDSTOPTS           = IPV6_DSTOPTS;

{ IPV6_MTU_DISCOVER values.  }
        IPV6_PMTUDISC_DONT     = 0;    { Never send DF frames.  }
        IPV6_PMTUDISC_WANT      = 1;    { Use per route hints.  }
        IPV6_PMTUDISC_DO     = 2;    { Always DF.  }

{ Socket level values for IPv6.  }
        SOL_IPV6                 = 41;
        SOL_ICMPV6               = 58;

{ Routing header options for IPv6.  }
        IPV6_RTHDR_LOOSE         = 0;   { Hop doesn't need to be neighbour. }
        IPV6_RTHDR_STRICT     = 1;    { Hop must be a neighbour.  }

        IPV6_RTHDR_TYPE_0        = 0;    { IPv6 Routing header type 0.  }
        
  { Flags for send, recv etc. }
  MSG_OOB      = $0001;              { Process out-of-band data}
  MSG_PEEK     = $0002;              { Peek at incoming messages }
  MSG_DONTROUTE= $0004;              { Don't use local routing }
  MSG_TRYHARD  = MSG_DONTROUTE;
  MSG_CTRUNC   = $0008;              { Control data lost before delivery }
  MSG_PROXY    = $0010;              { Supply or ask second address }
  MSG_TRUNC    = $0020;
  MSG_DONTWAIT = $0040;              { Non-blocking I/O }
  MSG_EOR      = $0080;              { End of record }
  MSG_WAITALL  = $0100;              { Wait for a full request }
  MSG_FIN      = $0200;
  MSG_SYN      = $0400;
  MSG_CONFIRM  = $0800;              { Confirm path validity }
  MSG_RST      = $1000;
  MSG_ERRQUERE = $2000;              { Fetch message from error queue }
  MSG_NOSIGNAL = $4000;              { Do not generate SIGPIPE }
  MSG_MORE     = $8000;              { Sender will send more }
  MSG_EOF      = MSG_FIN;
  
     TCP_NODELAY = 1;
  { Limit MSS  }
     TCP_MAXSEG = 2;
  { Never send partially complete segments  }
     TCP_CORK = 3;
  { Start keeplives after this period  }
     TCP_KEEPIDLE = 4;
  { Interval between keepalives  }
     TCP_KEEPINTVL = 5;
  { Number of keepalives before death  }
     TCP_KEEPCNT = 6;
  { Number of SYN retransmits  }
     TCP_SYNCNT = 7;
  { Life time of orphaned FIN-WAIT-2 state  }
     TCP_LINGER2 = 8;
  { Wake up listener only when data arrive  }
     TCP_DEFER_ACCEPT = 9;
  { Bound advertised window  }
     TCP_WINDOW_CLAMP = 10;
  { Information about this connection.  }
     TCP_INFO = 11;
  { Block/reenable quick acks  }
     TCP_QUICKACK = 12;
  { Congestion control algorithm  }
     TCP_CONGESTION = 13;
  { TCP MD5 Signature (RFC2385)  }
     TCP_MD5SIG = 14;
  
     UDP_CORK = 1;
  { Set the socket to accept encapsulated packets  }
     UDP_ENCAP = 100;
  { UDP encapsulation types  }
  { draft-ietf-ipsec-nat-t-ike-00/01  }
     UDP_ENCAP_ESPINUDP_NON_IKE = 1;
  { draft-ietf-ipsec-udp-encaps-06  }
     UDP_ENCAP_ESPINUDP = 2;
  { rfc2661  }
     UDP_ENCAP_L2TPINUDP = 3;

