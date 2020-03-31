unit nb30;
//  This module contains the definitions for portable NetBIOS 3.0
//  support.
interface

uses windows;
{***************************************************************
 *                                                              *
 *              Data structure templates                        *
 *                                                              *
 ***************************************************************}


const
  nbdllname     = 'netapi32.dll';
  NCBNAMSZ      =  16;    { absolute length of a net name           }
  MAX_LANA      = 254;    { lana's in range 0 to MAX_LANA inclusive }

{
 * Network Control Block
}

Type
 PNCB = ^_NCB;
 NCB_post_type = procedure (p: PNCB); stdcall;
 _NCB = packed record
    ncb_command     : UCHAR;              { command code                   }
    ncb_retcode     : UCHAR;              { return code                    }
    ncb_lsn         : UCHAR;              { local session number           }
    ncb_num         : UCHAR;              { number of our network name     }
    ncb_buffer      : PUCHAR;             { address of message buffer      }
    ncb_length      : WORD;             { size of message buffer           }
    ncb_callname    : array [0..NCBNAMSZ-1] of UCHAR; { blank-padded name of remote    }
    ncb_name        : array [0..NCBNAMSZ-1] of UCHAR; { our blank-padded netname       }
    ncb_rto         : UCHAR;              { rcv timeout/retry count        }
    ncb_sto         : UCHAR;              { send timeout/sys timeout       }
    ncb_post        : ncb_post_type;      { POST routine address        }
    ncb_lana_num    : UCHAR;              { lana (adapter) number          }
    ncb_cmd_cplt    : UCHAR;              { $ff => commmand pending       }
{$ifdef WIN64}
    ncb_reserve     : array[0..17] of UCHAR; { reserved, used by BIOS         }
{$else}
    ncb_reserve     : array[0..9] of UCHAR;  { reserved, used by BIOS         }
{$endif}
    ncb_event       : THandle;            { HANDLE to Win32 event which    }
                                          { will be set to the signalled   }
                                          { state when an ASYNCH command   }
                                          { completes                      }
   end;
   NCB = _NCB;

{
 *  Structure returned to the NCB command NCBASTAT is ADAPTER_STATUS followed
 *  by an array of NAME_BUFFER structures.
 }

 ADAPTER_STATUS  = packed record
   adapter_address             : Array [0..5] of UCHAR;
   rev_major                   : UCHAR;
   reserved0                   : UCHAR;
   adapter_type                : UCHAR;
   rev_minor                   : UCHAR;
   duration                    : WORD;
   frmr_recv                   : WORD;
   frmr_xmit                   : WORD;

   iframe_recv_err             : WORD;

   xmit_aborts                 : WORD;
   xmit_success                : DWORD;
   recv_success                : DWORD;

   iframe_xmit_err             : WORD;

   recv_buff_unavail           : WORD;
   t1_timeouts                 : WORD;
   ti_timeouts                 : WORD;
   reserved1                   : DWORD;
   free_ncbs                   : WORD;
   max_cfg_ncbs                : WORD;
   max_ncbs                    : WORD;
   xmit_buf_unavail            : WORD;
   max_dgram_size              : WORD;
   pending_sess                : WORD;
   max_cfg_sess                : WORD;
   max_sess                    : WORD;
   max_sess_pkt_size           : WORD;
   name_count                  : WORD;
   end;
  TADAPTER_STATUS = ADAPTER_STATUS;
  PADAPTER_STATUS = ^TADAPTER_STATUS;

 NAME_BUFFER = packed record
    name      : array [0..NCBNAMSZ-1] of UCHAR;
    name_num  : UCHAR;
    name_flags: UCHAR;
	end;
 TNAME_BUFFER = NAME_BUFFER;
 PNAME_BUFFER = ^NAME_BUFFER;

//  values for name_flags bits.
Const
       NAME_FLAGS_MASK = $87;

       GROUP_NAME      = $80;
       UNIQUE_NAME     = $00;

       REGISTERING     = $00;
       REGISTERED      = $04;
       DEREGISTERED    = $05;
       DUPLICATE       = $06;
       DUPLICATE_DEREG = $07;

Type
{
 *  Structure returned to the NCB command NCBSSTAT is SESSION_HEADER followed
 *  by an array of SESSION_BUFFER structures. If the NCB_NAME starts with an
 *  asterisk then an array of these structures is returned containing the
 *  status for all names.
 }

 PSESSION_HEADER = ^TSESSION_HEADER;
 TSESSION_HEADER = packed record
        sess_name : UCHAR;
        num_sess  : UCHAR;
        rcv_dg_outstanding  : UCHAR;
        rcv_any_outstanding : UCHAR;
      end;

 PSESSION_BUFFER = ^TSESSION_BUFFER;
 TSESSION_BUFFER = packed record
        lsn   : UCHAR;
        state : UCHAR;
        local_name : array[0..(NCBNAMSZ)-1] of UCHAR;
        remote_name : array[0..(NCBNAMSZ)-1] of UCHAR;
        rcvs_outstanding  : UCHAR;
        sends_outstanding : UCHAR;
      end;

//  Values for state

Const
       LISTEN_OUTSTANDING      = $01;
       CALL_PENDING            = $02;
       SESSION_ESTABLISHED     = $03;
       HANGUP_PENDING          = $04;
       HANGUP_COMPLETE         = $05;
       SESSION_ABORTED         = $06;

{
 *  Structure returned to the NCB command NCBENUM.
 *
 *  On a system containing lana's 0, 2 and 3, a structure with
 *  length =3, lana[0]=0, lana[1]=2 and lana[2]=3 will be returned.
 }
Type
  PLANA_ENUM = ^TLANA_ENUM;
  TLANA_ENUM = packed record
        length : UCHAR;     //  Number of valid entries in lana[]
        lana   : array[0..(MAX_LANA+1)-1] of UCHAR;
      end;

{
 *  Structure returned to the NCB command NCBFINDNAME is FIND_NAME_HEADER followed
 *  by an array of FIND_NAME_BUFFER structures.
 }

  PFIND_NAME_HEADER = ^TFIND_NAME_HEADER;
  TFIND_NAME_HEADER = packed record
        node_count : WORD;
        reserved   : UCHAR;
        unique_group : UCHAR;
      end;

  PFIND_NAME_BUFFER = ^TFIND_NAME_BUFFER;
  TFIND_NAME_BUFFER = packed record
        length         : UCHAR;
        access_control : UCHAR;
        frame_control  : UCHAR;
        destination_addr : array[0..5] of UCHAR;
        source_addr      : array[0..5] of UCHAR;
        routing_info     : array[0..17] of UCHAR;
      end;

{
 *  Structure provided with NCBACTION. The purpose of NCBACTION is to provide
 *  transport specific extensions to netbios.
 }

  PACTION_HEADER = ^TACTION_HEADER;
  TACTION_HEADER = packed record
        transport_id : ULONG;
        action_code  : USHORT;
        reserved     : USHORT;
      end;

//  Values for transport_id

const
       ALL_TRANSPORTS  = 'M'#0#0#0;
       MS_NBF          = 'MNBF'#0;


{***************************************************************
 *                                                              *
 *              Special values and constants                    *
 *                                                              *
 ***************************************************************}

Const
{
 *      NCB Command codes
 }

       NCBCALL         = $10;            { NCB CALL                           }
       NCBLISTEN       = $11;            { NCB LISTEN                         }
       NCBHANGUP       = $12;            { NCB HANG UP                        }
       NCBSEND         = $14;            { NCB SEND                           }
       NCBRECV         = $15;            { NCB RECEIVE                        }
       NCBRECVANY      = $16;            { NCB RECEIVE ANY                    }
       NCBCHAINSEND    = $17;            { NCB CHAIN SEND                     }
       NCBDGSEND       = $20;            { NCB SEND DATAGRAM                  }
       NCBDGRECV       = $21;            { NCB RECEIVE DATAGRAM               }
       NCBDGSENDBC     = $22;            { NCB SEND BROADCAST DATAGRAM        }
       NCBDGRECVBC     = $23;            { NCB RECEIVE BROADCAST DATAGRAM     }
       NCBADDNAME      = $30;            { NCB ADD NAME                       }
       NCBDELNAME      = $31;            { NCB DELETE NAME                    }
       NCBRESET        = $32;            { NCB RESET                          }
       NCBASTAT        = $33;            { NCB ADAPTER STATUS                 }
       NCBSSTAT        = $34;            { NCB SESSION STATUS                 }
       NCBCANCEL       = $35;            { NCB CANCEL                         }
       NCBADDGRNAME    = $36;            { NCB ADD GROUP NAME                 }
       NCBENUM         = $37;            { NCB ENUMERATE LANA NUMBERS         }
       NCBUNLINK       = $70;            { NCB UNLINK                         }
       NCBSENDNA       = $71;            { NCB SEND NO ACK                    }
       NCBCHAINSENDNA  = $72;            { NCB CHAIN SEND NO ACK              }
       NCBLANSTALERT   = $73;            { NCB LAN STATUS ALERT               }
       NCBACTION       = $77;            { NCB ACTION                         }
       NCBFINDNAME     = $78;            { NCB FIND NAME                      }
       NCBTRACE        = $79;            { NCB TRACE                          }


       ASYNCH          = $80;            { high bit set == asynchronous       }

{
 *      NCB Return codes
 }

       NRC_GOODRET     = $00;    { good return                                }
                                { also returned when ASYNCH request accepted }
       NRC_BUFLEN      = $01;    { illegal buffer length                      }
       NRC_ILLCMD      = $03;    { illegal command                            }
       NRC_CMDTMO      = $05;    { command timed out                          }
       NRC_INCOMP      = $06;    { message incomplete, issue another command  }
       NRC_BADDR       = $07;    { illegal buffer address                     }
       NRC_SNUMOUT     = $08;    { session number out of range                }
       NRC_NORES       = $09;    { no resource available                      }
       NRC_SCLOSED     = $0a;    { session closed                             }
       NRC_CMDCAN      = $0b;    { command cancelled                          }
       NRC_DUPNAME     = $0d;    { duplicate name                             }
       NRC_NAMTFUL     = $0e;    { name table full                            }
       NRC_ACTSES      = $0f;    { no deletions, name has active sessions     }
       NRC_LOCTFUL     = $11;    { local session table full                   }
       NRC_REMTFUL     = $12;    { remote session table full                  }
       NRC_ILLNN       = $13;    { illegal name number                        }
       NRC_NOCALL      = $14;    { no callname                                }
       NRC_NOWILD      = $15;    { cannot put * in NCB_NAME                   }
       NRC_INUSE       = $16;    { name in use on remote adapter              }
       NRC_NAMERR      = $17;    { name deleted                               }
       NRC_SABORT      = $18;    { session ended abnormally                   }
       NRC_NAMCONF     = $19;    { name conflict detected                     }
       NRC_IFBUSY      = $21;    { interface busy, IRET before retrying       }
       NRC_TOOMANY     = $22;    { too many commands outstanding, retry later }
       NRC_BRIDGE      = $23;    { ncb_lana_num field invalid                 }
       NRC_CANOCCR     = $24;    { command completed while cancel occurring   }
       NRC_CANCEL      = $26;    { command not valid to cancel                }
       NRC_DUPENV      = $30;    { name defined by anther local process       }
       NRC_ENVNOTDEF   = $34;    { environment undefined. RESET required      }
       NRC_OSRESNOTAV  = $35;    { required OS resources exhausted            }
       NRC_MAXAPPS     = $36;    { max number of applications exceeded        }
       NRC_NOSAPS      = $37;    { no saps available for netbios              }
       NRC_NORESOURCES = $38;    { requested resources are not available      }
       NRC_INVADDRESS  = $39;    { invalid ncb address or length > segment    }
       NRC_INVDDID     = $3B;    { invalid NCB DDID                           }
       NRC_LOCKFAIL    = $3C;    { lock of user area failed                   }
       NRC_OPENERR     = $3f;    { NETBIOS not loaded                         }
       NRC_SYSTEM      = $40;    { system error                               }

       NRC_PENDING     = $ff;    { asynchronous command is not yet finished   }

{***************************************************************
 *                                                              *
 *              main user entry point for NetBIOS 3.0           *
 *                                                              *
 * Usage: result = Netbios( pncb );                             *
 ***************************************************************}

function Netbios(pncb : PNCB):UCHAR; stdcall; external nbdllname name 'Netbios';

implementation
end.
