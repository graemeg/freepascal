{
   This file is part of the Free Pascal run time library.
   (c) 2000-2003 by Marco van de Voort
   member of the Free Pascal development team.
   (c) 2010 by Olivier Coursière.

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   OS dependant part of the header.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

Const
{*
 * Address families.
 *}
  AF_UNSPEC        = 0;			{ unspecified }
  AF_INET          = 1;			{ internetwork: UDP, TCP, etc. }
  AF_APPLETALK     = 2;			{ Apple Talk }
  AF_ROUTE         = 3;			{ Internal Routing Protocol }
  AF_LINK          = 4;			{ Link layer interface }
  AF_INET6         = 5;			{ IPv6 }
  AF_DLI           = 6;			{ DEC Direct data link interface }
  AF_IPX           = 7;			{ Novell Internet Protocol }
  AF_NOTIFY        = 8; 		{ ??? }
  AF_LOCAL         = 9;			{ local to host (pipes, portals) }
  AF_UNIX          = AF_LOCAL;	{ standardized name for AF_LOCAL }
  AF_BLUETOOTH     = 10; 		{ Bluetooth sockets }
  AF_MAX           = 11;

(* Values not yet defined under Haiku (see sys/socket.h)
  AF_IMPLINK       = 3;		{ arpanet imp addresses }
  AF_PUP           = 4;		{ pup protocols: e.g. BSP }
  AF_CHAOS         = 5;		{ mit CHAOS protocols }
  AF_NETBIOS       = 6;		{ SMB protocols }
  AF_ISO           = 7;		{ ISO protocols }
  AF_OSI           = AF_ISO;
  AF_ECMA          = 8;		{ European computer manufacturers }
  AF_DATAKIT       = 9;		{ datakit protocols }
  AF_CCITT         = 10;	{ CCITT protocols, X.25 etc }
  AF_SNA           = 11;	{ IBM SNA }
  AF_DECnet        = 12;	{ DECnet }
  AF_LAT           = 14;	{ LAT }
  AF_HYLINK        = 15;	{ NSC Hyperchannel }
  pseudo_AF_XTP    = 19;	{ eXpress Transfer Protocol (no AF) }
  AF_COIP          = 20;	{ connection-oriented IP, aka ST II }
  AF_CNT           = 21;	{ Computer Network Technology }
  pseudo_AF_RTIP   = 22;	{ Help Identify RTIP packets }
  AF_SIP           = 24;	{ Simple Internet Protocol }
  pseudo_AF_PIP    = 25;	{ Help Identify PIP packets }
  AF_ISDN          = 26;	{ Integrated Services Digital Network}
  AF_E164          = AF_ISDN;	{ CCITT E.164 recommendation }
  pseudo_AF_KEY    = 27;	{ Internal key-management function }
  AF_NATM          = 29;	{ native ATM access }
  AF_ATM           = 30;	{ ATM }
  pseudo_AF_HDRCMPLT = 31;	{ Used by BPF to not rewrite headers
					 * in interface output routine
                                         }
  AF_NETGRAPH      = 32;	{ Netgraph sockets }
  AF_SLOW          = 33;	{ 802.3ad slow protocol }
  AF_SCLUSTER      = 34;	{ Sitara cluster protocol }
  AF_ARP           = 35;
  AF_IEEE80211     = 37;	{ IEEE 802.11 protocol }
*)

  {
   * Protocol families, same as address families for now.
    }

  const
     PF_UNSPEC = AF_UNSPEC;
     PF_INET = AF_INET;
     PF_ROUTE = AF_ROUTE;
     PF_LINK = AF_LINK;
     PF_INET6 = AF_INET6;
     PF_LOCAL = AF_LOCAL;
  { backward compatibility  }
     PF_UNIX = PF_LOCAL;
     PF_BLUETOOTH = AF_BLUETOOTH;
     PF_MAX = AF_MAX;

(* Values not yet defined under Haiku (see sys/socket.h)
     PF_IMPLINK = AF_IMPLINK;
     PF_PUP = AF_PUP;
     PF_CHAOS = AF_CHAOS;
     PF_NETBIOS = AF_NETBIOS;
     PF_ISO = AF_ISO;
     PF_OSI = AF_ISO;
     PF_ECMA = AF_ECMA;
     PF_DATAKIT = AF_DATAKIT;
     PF_CCITT = AF_CCITT;
     PF_SNA = AF_SNA;
     PF_DECnet = AF_DECnet;
     PF_DLI = AF_DLI;
     PF_LAT = AF_LAT;
     PF_HYLINK = AF_HYLINK;
     PF_APPLETALK = AF_APPLETALK;
  { really just proto family, no AF  }
     PF_XTP = pseudo_AF_XTP;
     PF_COIP = AF_COIP;
     PF_CNT = AF_CNT;
     PF_SIP = AF_SIP;
     PF_IPX = AF_IPX;
  { same format as AF_INET  }
     PF_RTIP = pseudo_AF_RTIP;
     PF_PIP = pseudo_AF_PIP;
     PF_ISDN = AF_ISDN;
     PF_KEY = pseudo_AF_KEY;
     PF_NATM = AF_NATM;
     PF_ATM = AF_ATM;
     PF_NETGRAPH = AF_NETGRAPH;
     PF_SLOW = AF_SLOW;
     PF_SCLUSTER = AF_SCLUSTER;
     PF_ARP = AF_ARP;
*)

  SOCK_PACKET     = 10;


{ Maximum queue length specifiable by listen.  }
  SOMAXCONN     = 32;

        SOL_SOCKET 	     = -1;

        SO_ACCEPTCONN    = $00000001;        { socket has had listen() }
        SO_BROADCAST     = $00000002;        { permit sending of broadcast msgs }
        SO_DEBUG         = $00000004;        { turn on debugging info recording }
        SO_DONTROUTE     = $00000008;        { just use interface addresses }
        SO_KEEPALIVE     = $00000010;        { keep connections alive }
        SO_OOBINLINE     = $00000020;        { leave received OOB data in line }
        SO_REUSEADDR     = $00000040;        { allow local address reuse }
        SO_REUSEPORT     = $00000080;        { allow local address & port reuse }
        SO_USELOOPBACK   = $00000100;        { bypass hardware when possible }
        SO_LINGER        = $00000200;        { linger on close if data present }
        
(* Values not yet defined under Haiku (see sys/socket.h)
        SO_TIMESTAMP     = $0400;        { timestamp received dgram traffic }
*)

{
 * Additional options, not kept in so_options.
 }
        SO_SNDBUF        = $40000001;    { send buffer size }
        SO_SNDLOWAT      = $40000003;    { send low-water mark }
        SO_SNDTIMEO      = $40000005;    { send timeout }
        SO_RCVBUF        = $40000002;    { receive buffer size }
        SO_RCVLOWAT      = $40000004;    { receive low-water mark }
        SO_RCVTIMEO      = $40000006;    { receive timeout }
        SO_ERROR         = $40000007;    { get error status and clear }
        SO_TYPE          = $40000008;    { get socket type }
        SO_NONBLOCK		 = $40000009;
        SO_BINDTODEVICE	 = $4000000a;    { binds the socket to a specific device index }
        SO_PEERCRED		 = $4000000b;    { get peer credentials, param: ucred }


        SHUT_RD         =0;             { shut down the reading side }
        SHUT_WR         =1;             { shut down the writing side }
        SHUT_RDWR       =2;             { shut down both sides }

	IPPROTO_IP              = 0;               { dummy for IP }
	IPPROTO_ICMP            = 1;               { control message protocol }
	IPPROTO_TCP             = 6;               { tcp }
	IPPROTO_UDP             = 17;              { user datagram protocol }


	IPPROTO_HOPOPTS		= 0 ; 		{ IP6 hop-by-hop options }
	IPPROTO_IGMP		= 2 ; 		{ group mgmt protocol }
	IPPROTO_GGP		= 3 ; 		{ gateway^2 (deprecated) }
	IPPROTO_IPV4		= 4 ; 		{ IPv4 encapsulation }
	IPPROTO_IPIP		= IPPROTO_IPV4;	{ for compatibility }
	IPPROTO_ST		= 7 ; 		{ Stream protocol II }
	IPPROTO_EGP		= 8 ; 		{ exterior gateway protocol }
	IPPROTO_PIGP		= 9 ; 		{ private interior gateway }
	IPPROTO_RCCMON		= 10; 		{ BBN RCC Monitoring }
	IPPROTO_NVPII		= 11; 		{ network voice protocol}
	IPPROTO_PUP		= 12; 		{ pup }
	IPPROTO_ARGUS		= 13; 		{ Argus }
	IPPROTO_EMCON		= 14; 		{ EMCON }
	IPPROTO_XNET		= 15; 		{ Cross Net Debugger }
	IPPROTO_CHAOS		= 16; 		{ Chaos}
	IPPROTO_MUX		= 18; 		{ Multiplexing }
	IPPROTO_MEAS		= 19; 		{ DCN Measurement Subsystems }
	IPPROTO_HMP		= 20; 		{ Host Monitoring }
	IPPROTO_PRM		= 21; 		{ Packet Radio Measurement }
	IPPROTO_IDP		= 22; 		{ xns idp }
	IPPROTO_TRUNK1		= 23; 		{ Trunk-1 }
	IPPROTO_TRUNK2		= 24; 		{ Trunk-2 }
	IPPROTO_LEAF1		= 25; 		{ Leaf-1 }
	IPPROTO_LEAF2		= 26; 		{ Leaf-2 }
	IPPROTO_RDP		= 27; 		{ Reliable Data }
	IPPROTO_IRTP		= 28; 		{ Reliable Transaction }
	IPPROTO_TP		= 29; 		{ tp-4 w/ class negotiation }
	IPPROTO_BLT		= 30; 		{ Bulk Data Transfer }
	IPPROTO_NSP		= 31; 		{ Network Services }
	IPPROTO_INP		= 32; 		{ Merit Internodal }
	IPPROTO_SEP		= 33; 		{ Sequential Exchange }
	IPPROTO_3PC		= 34; 		{ Third Party Connect }
	IPPROTO_IDPR		= 35; 		{ InterDomain Policy Routing }
	IPPROTO_XTP		= 36; 		{ XTP }
	IPPROTO_DDP		= 37; 		{ Datagram Delivery }
	IPPROTO_CMTP		= 38; 		{ Control Message Transport }
	IPPROTO_TPXX		= 39; 		{ TP++ Transport }
	IPPROTO_IL		= 40; 		{ IL transport protocol }
	IPPROTO_IPV6		= 41; 		{ IP6 header }
	IPPROTO_SDRP		= 42; 		{ Source Demand Routing }
	IPPROTO_ROUTING		= 43; 		{ IP6 routing header }
	IPPROTO_FRAGMENT	= 44; 		{ IP6 fragmentation header }
	IPPROTO_IDRP		= 45; 		{ InterDomain Routing}
	IPPROTO_RSVP		= 46; 		{ resource reservation }
	IPPROTO_GRE		= 47; 		{ General Routing Encap. }
	IPPROTO_MHRP		= 48; 		{ Mobile Host Routing }
	IPPROTO_BHA		= 49; 		{ BHA }
	IPPROTO_ESP		= 50; 		{ IP6 Encap Sec. Payload }
	IPPROTO_AH		= 51; 		{ IP6 Auth Header }
	IPPROTO_INLSP		= 52; 		{ Integ. Net Layer Security }
	IPPROTO_SWIPE		= 53; 		{ IP with encryption }
	IPPROTO_NHRP		= 54; 		{ Next Hop Resolution }
	IPPROTO_MOBILE		= 55; 		{ IP Mobility }
	IPPROTO_TLSP		= 56; 		{ Transport Layer Security }
	IPPROTO_SKIP		= 57; 		{ SKIP }
	IPPROTO_ICMPV6		= 58; 		{ ICMP6 }
	IPPROTO_NONE		= 59; 		{ IP6 no next header }
	IPPROTO_DSTOPTS		= 60; 		{ IP6 destination option }
	IPPROTO_AHIP		= 61; 		{ any host internal protocol }
	IPPROTO_CFTP		= 62; 		{ CFTP }
	IPPROTO_HELLO		= 63; 		{ "hello" routing protocol }
	IPPROTO_SATEXPAK	= 64; 		{ SATNET/Backroom EXPAK }
	IPPROTO_KRYPTOLAN	= 65; 		{ Kryptolan }
	IPPROTO_RVD		= 66; 		{ Remote Virtual Disk }
	IPPROTO_IPPC		= 67; 		{ Pluribus Packet Core }
	IPPROTO_ADFS		= 68; 		{ Any distributed FS }
	IPPROTO_SATMON		= 69; 		{ Satnet Monitoring }
	IPPROTO_VISA		= 70; 		{ VISA Protocol }
	IPPROTO_IPCV		= 71; 		{ Packet Core Utility }
	IPPROTO_CPNX		= 72; 		{ Comp. Prot. Net. Executive }
	IPPROTO_CPHB		= 73; 		{ Comp. Prot. HeartBeat }
	IPPROTO_WSN		= 74; 		{ Wang Span Network }
	IPPROTO_PVP		= 75; 		{ Packet Video Protocol }
	IPPROTO_BRSATMON	= 76; 		{ BackRoom SATNET Monitoring }
	IPPROTO_ND		= 77; 		{ Sun net disk proto (temp.) }
	IPPROTO_WBMON		= 78; 		{ WIDEBAND Monitoring }
	IPPROTO_WBEXPAK		= 79; 		{ WIDEBAND EXPAK }
	IPPROTO_EON		= 80; 		{ ISO cnlp }
	IPPROTO_VMTP		= 81; 		{ VMTP }
	IPPROTO_SVMTP		= 82; 		{ Secure VMTP }
	IPPROTO_VINES		= 83; 		{ Banyon VINES }
	IPPROTO_TTP		= 84; 		{ TTP }
	IPPROTO_IGP		= 85; 		{ NSFNET-IGP }
	IPPROTO_DGP		= 86; 		{ dissimilar gateway prot. }
	IPPROTO_TCF		= 87; 		{ TCF }
	IPPROTO_IGRP		= 88; 		{ Cisco/GXS IGRP }
	IPPROTO_OSPFIGP		= 89; 		{ OSPFIGP }
	IPPROTO_SRPC		= 90; 		{ Strite RPC protocol }
	IPPROTO_LARP		= 91; 		{ Locus Address Resoloution }
	IPPROTO_MTP		= 92; 		{ Multicast Transport }
	IPPROTO_AX25		= 93; 		{ AX.25 Frames }
	IPPROTO_IPEIP		= 94; 		{ IP encapsulated in IP }
	IPPROTO_MICP		= 95; 		{ Mobile Int.ing control }
	IPPROTO_SCCSP		= 96; 		{ Semaphore Comm. security }
	IPPROTO_ETHERIP		= 97; 		{ Ethernet IP encapsulation }
	IPPROTO_ENCAP		= 98; 		{ encapsulation header }
	IPPROTO_APES		= 99; 		{ any private encr. scheme }
	IPPROTO_GMTP		= 100;		{ GMTP}
	IPPROTO_IPCOMP		= 108;		{ payload compression (IPComp) }
{ 101-254: Partly Unassigned }
	IPPROTO_PIM		= 103;		{ Protocol Independent Mcast }
	IPPROTO_CARP		= 112;		{ CARP }
	IPPROTO_PGM		= 113;		{ PGM }
	IPPROTO_PFSYNC		= 240;		{ PFSYNC }

{ last return value of *_input(), meaning "all job for this pkt is done".  }
	IPPROTO_RAW             = 255;
	IPPROTO_MAX		= 256;
	IPPROTO_DONE		= 257;

{
 * Options for use with [gs]etsockopt at the IP level.
 * First word of comment is data type; bool is stored in int.
 }
	IP_OPTIONS		= 1 ;   { buf/ip_opts; set/get IP options }
	IP_HDRINCL		= 2 ;   { int; header is included with data }
	IP_TOS			= 3 ;   { int; IP type of service and preced. }
	IP_TTL			= 4 ;   { int; IP time to live }
	IP_RECVOPTS		= 5 ;   { bool; receive all IP opts w/dgram }
	IP_RECVRETOPTS		= 6 ;   { bool; receive IP opts for response }
	IP_RECVDSTADDR		= 7 ;   { bool; receive IP dst addr w/dgram }
	IP_SENDSRCADDR		= IP_RECVDSTADDR; { cmsg_type to set src addr }
	IP_RETOPTS			= 8 ;   { ip_opts; set/get IP options }
	IP_MULTICAST_IF		= 9 ;   { u_char; set/get IP multicast i/f  }
	IP_MULTICAST_TTL	= 10;   { u_char; set/get IP multicast ttl }
	IP_MULTICAST_LOOP	= 11;   { u_char; set/get IP multicast loopback }
	IP_ADD_MEMBERSHIP	= 12;   { ip_mreq; add an IP group membership }
	IP_DROP_MEMBERSHIP	= 13;   { ip_mreq; drop an IP group membership }
	
	IP_BLOCK_SOURCE				= 14;	{ ip_mreq_source }
	IP_UNBLOCK_SOURCE			= 15;	{ ip_mreq_source }
	IP_ADD_SOURCE_MEMBERSHIP	= 16;	{ ip_mreq_source }
	IP_DROP_SOURCE_MEMBERSHIP	= 17;	{ ip_mreq_source }
	MCAST_JOIN_GROUP			= 18;	{ group_req }
	MCAST_BLOCK_SOURCE			= 19;	{ group_source_req }
	MCAST_UNBLOCK_SOURCE		= 20;	{ group_source_req }
	MCAST_LEAVE_GROUP			= 21;	{ group_req }
	MCAST_JOIN_SOURCE_GROUP		= 22;	{ group_source_req }
	MCAST_LEAVE_SOURCE_GROUP	= 23;	{ group_source_req }
	
(* Values not yet defined under Haiku (see netinet/in.h)
	IP_MULTICAST_VIF	= 14;   { set/get IP mcast virt. iface }
	IP_RSVP_ON		= 15;   { enable RSVP in kernel }
	IP_RSVP_OFF		= 16;   { disable RSVP in kernel }
	IP_RSVP_VIF_ON		= 17;   { set RSVP per-vif socket }
	IP_RSVP_VIF_OFF		= 18;   { unset RSVP per-vif socket }
	IP_PORTRANGE		= 19;   { int; range to choose for unspec port }
	IP_RECVIF		= 20;   { bool; receive reception if w/dgram }

{ for IPSEC }
	IP_IPSEC_POLICY		= 21;   { int; set/get security policy }
	IP_FAITH		= 22;   { bool; accept FAITH'ed connections }

	IP_ONESBCAST		= 23;   { bool: send all-ones broadcast }
                                
	IP_FW_TABLE_ADD		= 40;   { add entry }
	IP_FW_TABLE_DEL		= 41;   { delete entry }
	IP_FW_TABLE_FLUSH	= 42;   { flush table }
	IP_FW_TABLE_GETSIZE	= 43;   { get table size }
	IP_FW_TABLE_LIST	= 44;   { list table contents }

	IP_FW_ADD		= 50;   { add a firewall rule to chain }
	IP_FW_DEL		= 51;   { delete a firewall rule from chain }
	IP_FW_FLUSH		= 52;   { flush firewall rule chain }
	IP_FW_ZERO		= 53;   { clear single/all firewall counter(s) }
	IP_FW_GET		= 54;   { get entire firewall rule chain }
	IP_FW_RESETLOG		= 55;   { reset logging counters }

	IP_DUMMYNET_CONFIGURE	= 60;   { add/configure a dummynet pipe }
	IP_DUMMYNET_DEL		= 61;   { delete a dummynet pipe from chain }
	IP_DUMMYNET_FLUSH	= 62;   { flush dummynet }
	IP_DUMMYNET_GET		= 64;   { get entire dummynet pipes }

	IP_RECVTTL		= 65;   { bool; receive IP TTL w/dgram }

	IPV6_SOCKOPT_RESERVED1	= 3 ; { reserved for future use }
	IPV6_UNICAST_HOPS	= 4 ; { int; IP6 hops }
*)

	IPV6_MULTICAST_IF	= 24; { u_int; setget IP6 multicast if  }
	IPV6_MULTICAST_HOPS	= 25; { int; setget IP6 multicast hops }
	IPV6_MULTICAST_LOOP	= 26; { u_int; setget IP6 multicast loopback }

(* Values not yet defined under Haiku (see netinet/in.h)
	IPV6_JOIN_GROUP		= 12; { ip6_mreq; join a group membership }
	IPV6_LEAVE_GROUP	= 13; { ip6_mreq; leave a group membership }
	IPV6_PORTRANGE		= 14; { int; range to choose for unspec port }

	IPV6_PKTINFO            = 46; { in6_pktinfo; send if, src addr }	
 	IPV6_HOPLIMIT           = 47; { int; send hop limit }
 	IPV6_NEXTHOP            = 48; { sockaddr; next hop addr }
 	IPV6_HOPOPTS            = 49; { ip6_hbh; send hop-by-hop option }
 	IPV6_DSTOPTS            = 50; { ip6_dest; send dst option befor rthdr }
 	IPV6_RTHDR              = 51; { ip6_rthdr; send routing header }
 	IPV6_PKTOPTIONS         = 52; { buf/cmsghdr; set/get IPv6 options }
*)  
  
  { Flags for send, recv etc. }
  MSG_OOB       = $0001;              { process out-of-band data}
  MSG_PEEK      = $0002;              { peek at incoming messages }
  MSG_DONTROUTE = $0004;              { send without using routing tables }
  MSG_EOR       = $0008;              { data completes record }
  MSG_TRUNC     = $0010;              { data discarded before delivery }
  MSG_CTRUNC    = $0020;              { control data lost before delivery }
  MSG_WAITALL   = $0040;              { wait for full request or error }
  MSG_DONTWAIT  = $0080;              { this message should be nonblocking }
  MSG_BCAST     = $0100;              { this message rec'd as broadcast }
  MSG_MCAST     = $0200;              { this message rec'd as multicast }
  MSG_EOF       = $0400;              { data completes connection }
  
(* Values not yet defined under Haiku (see sys/socket.h)
  MSG_NBIO      = $4000;
  MSG_COMPAT    = $8000;
  MSG_SOCALLBCK = $10000;
  MSG_NOSIGNAL  = $20000;              { Do not generate SIGPIPE }
*)

  { SOL_SOCKET control message types }
  SCM_RIGHTS     = $01;
  
  INVALID_SOCKET = -1;
  SOCKET_ERROR = -1;
  
     TCP_NODELAY = $01;
  { don't delay sending smaller packets  }
     TCP_MAXSEG = $02;
  { set maximum segment size  }
     TCP_NOPUSH = $04;
  { don't use TH_PUSH  }
     TCP_NOOPT = $08;
  { don't use any TCP options  }
