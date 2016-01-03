
const
  {
   * Address families.
    }

  { unspecified  }
     AF_UNSPEC = 0;
  { local to host (pipes, portals)  }
     AF_LOCAL = 1;
  { backward compatibility  }
     AF_UNIX = AF_LOCAL;
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
  { ISO protocols  }
     AF_ISO = 7;
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
  { DEC Direct data link interface  }
     AF_DLI = 13;
  { LAT  }
     AF_LAT = 14;
  { NSC Hyperchannel  }
     AF_HYLINK = 15;
  { Apple Talk  }
     AF_APPLETALK = 16;
  { Internal Routing Protocol  }
     AF_ROUTE = 17;
  { Link layer interface  }
     AF_LINK = 18;
  { eXpress Transfer Protocol (no AF)  }
     pseudo_AF_XTP = 19;
  { connection-oriented IP, aka ST II  }
     AF_COIP = 20;
  { Computer Network Technology  }
     AF_CNT = 21;
  { Help Identify RTIP packets  }
     pseudo_AF_RTIP = 22;
  { Novell Internet Protocol  }
     AF_IPX = 23;
  { IPv6  }
     AF_INET6 = 24;
  { Help Identify PIP packets  }
     pseudo_AF_PIP = 25;
  { Integrated Services Digital Network }
     AF_ISDN = 26;
  { CCITT E.164 recommendation  }
     AF_E164 = AF_ISDN;
  { native ATM access  }
     AF_NATM = 27;
     AF_ENCAP = 28;
  { Simple Internet Protocol  }
     AF_SIP = 29;
     AF_KEY = 30;
  { Used by BPF to not rewrite headers
  					   in interface output routine  }
     pseudo_AF_HDRCMPLT = 31;
  { Bluetooth  }
     AF_BLUETOOTH = 32;
     AF_MAX = 33;
  {
   * Protocol families, same as address families for now.
    }
     PF_UNSPEC = AF_UNSPEC;
     PF_LOCAL = AF_LOCAL;
  { backward compatibility  }
     PF_UNIX = PF_LOCAL;
     PF_INET = AF_INET;
     PF_IMPLINK = AF_IMPLINK;
     PF_PUP = AF_PUP;
     PF_CHAOS = AF_CHAOS;
     PF_NS = AF_NS;
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
     PF_ROUTE = AF_ROUTE;
     PF_LINK = AF_LINK;
  { really just proto family, no AF  }
     PF_XTP = pseudo_AF_XTP;
     PF_COIP = AF_COIP;
     PF_CNT = AF_CNT;
  { same format as AF_NS  }
     PF_IPX = AF_IPX;
     PF_INET6 = AF_INET6;
  { same format as AF_INET  }
     PF_RTIP = pseudo_AF_RTIP;
     PF_PIP = pseudo_AF_PIP;
     PF_ISDN = AF_ISDN;
     PF_NATM = AF_NATM;
     PF_ENCAP = AF_ENCAP;
     PF_SIP = AF_SIP;
     PF_KEY = AF_KEY;
     PF_BPF = pseudo_AF_HDRCMPLT;
     PF_BLUETOOTH = AF_BLUETOOTH;
     PF_MAX = AF_MAX;
     
     MSG_OOB = $1;
  { peek at incoming message  }
     MSG_PEEK = $2;
  { send without using routing tables  }
     MSG_DONTROUTE = $4;
  { data completes record  }
     MSG_EOR = $8;
  { data discarded before delivery  }
     MSG_TRUNC = $10;
  { control data lost before delivery  }
     MSG_CTRUNC = $20;
  { wait for full request or error  }
     MSG_WAITALL = $40;
  { this message should be nonblocking  }
     MSG_DONTWAIT = $80;
  { this message rec'd as broadcast  }
     MSG_BCAST = $100;
  { this message rec'd as multicast  }
     MSG_MCAST = $200;
     
     TCP_NODELAY = $01;
  { set maximum segment size  }
     TCP_MAXSEG = $02;
  { enable TCP MD5 signature option  }
     TCP_MD5SIG = $04;
  { enable SACKs (if disabled by def.)  }
     TCP_SACK_ENABLE = $08;

        SHUT_RD         =0;             { shut down the reading side }
        SHUT_WR         =1;             { shut down the writing side }
        SHUT_RDWR       =2;             { shut down both sides }


