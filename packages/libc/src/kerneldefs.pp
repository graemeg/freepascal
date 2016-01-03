{$mode objfpc}
{$h+}
unit kerneldefs;

interface
// Translated from asm/types.h (i386)

const
   HZ = 100;
   EXEC_PAGESIZE = 4096;
   NGROUPS = 32;
   NOGROUP = -(1);
   MAXHOSTNAMELEN = 64;
   CLOCKS_PER_SEC = 100;
type
   Pumode_t = ^umode_t;
   umode_t = word;

   P__s8 = ^__s8;
   __s8 = shortint;

   P__u8 = ^__u8;
   __u8 = byte;

   P__s16 = ^__s16;
   __s16 = smallint;

   P__u16 = ^__u16;
   __u16 = word;

   P__s32 = ^__s32;
   __s32 = longint;

   P__u32 = ^__u32;
   __u32 = dword;

   P__s64 = ^__s64;
   __s64 = int64;

   P__u64 = ^__u64;
   __u64 = qword;

type

   Ps8 = ^s8;
   s8 = char;

   Pu8 = ^u8;
   u8 = byte;

   Ps16 = ^s16;
   s16 = smallint;

   Pu16 = ^u16;
   u16 = word;

   Ps32 = ^s32;
   s32 = longint;

   Pu32 = ^u32;
   u32 = dword;

   Ps64 = ^s64;
   s64 = int64;

   Pu64 = ^u64;
   u64 = qword;

const
   BITS_PER_LONG = 32;

// Translated from include/linux/if_ether.h


const
   ETH_ALEN = 6;
   ETH_HLEN = 14;
   ETH_ZLEN = 60;
   ETH_DATA_LEN = 1500;
   ETH_FRAME_LEN = 1514;
   ETH_P_LOOP = $0060;
   ETH_P_PUP = $0200;
   ETH_P_PUPAT = $0201;
   ETH_P_IP = $0800;
   ETH_P_X25 = $0805;
   ETH_P_ARP = $0806;
   ETH_P_BPQ = $08FF;
   ETH_P_IEEEPUP = $0a00;
   ETH_P_IEEEPUPAT = $0a01;
   ETH_P_DEC = $6000;
   ETH_P_DNA_DL = $6001;
   ETH_P_DNA_RC = $6002;
   ETH_P_DNA_RT = $6003;
   ETH_P_LAT = $6004;
   ETH_P_DIAG = $6005;
   ETH_P_CUST = $6006;
   ETH_P_SCA = $6007;
   ETH_P_RARP = $8035;
   ETH_P_ATALK = $809B;
   ETH_P_AARP = $80F3;
   ETH_P_8021Q = $8100;
   ETH_P_IPX = $8137;
   ETH_P_IPV6 = $86DD;
   ETH_P_PPP_DISC = $8863;
   ETH_P_PPP_SES = $8864;
   ETH_P_ATMMPOA = $884c;
   ETH_P_ATMFATE = $8884;
   ETH_P_802_3 = $0001;
   ETH_P_AX25 = $0002;
   ETH_P_ALL = $0003;
   ETH_P_802_2 = $0004;
   ETH_P_SNAP = $0005;
   ETH_P_DDCMP = $0006;
   ETH_P_WAN_PPP = $0007;
   ETH_P_PPP_MP = $0008;
   ETH_P_LOCALTALK = $0009;
   ETH_P_PPPTALK = $0010;
   ETH_P_TR_802_2 = $0011;
   ETH_P_MOBITEX = $0015;
   ETH_P_CONTROL = $0016;
   ETH_P_IRDA = $0017;
   ETH_P_ECONET = $0018;
type
   Pethhdr = ^ethhdr;
   ethhdr = record
        h_dest : array[0..(ETH_ALEN)-1] of byte;
        h_source : array[0..(ETH_ALEN)-1] of byte;
        h_proto : word;
     end;

// Translated from include/linux/if_fddi.h


const
   FDDI_K_ALEN = 6;
   FDDI_K_8022_HLEN = 16;
   FDDI_K_SNAP_HLEN = 21;
   FDDI_K_8022_ZLEN = 16;
   FDDI_K_SNAP_ZLEN = 21;
   FDDI_K_8022_DLEN = 4475;
   FDDI_K_SNAP_DLEN = 4470;
   FDDI_K_LLC_ZLEN = 13;
   FDDI_K_LLC_LEN = 4491;
   FDDI_FC_K_VOID = $00;
   FDDI_FC_K_NON_RESTRICTED_TOKEN = $80;
   FDDI_FC_K_RESTRICTED_TOKEN = $C0;
   FDDI_FC_K_SMT_MIN = $41;
   FDDI_FC_K_SMT_MAX = $4F;
   FDDI_FC_K_MAC_MIN = $C1;
   FDDI_FC_K_MAC_MAX = $CF;
   FDDI_FC_K_ASYNC_LLC_MIN = $50;
   FDDI_FC_K_ASYNC_LLC_DEF = $54;
   FDDI_FC_K_ASYNC_LLC_MAX = $5F;
   FDDI_FC_K_SYNC_LLC_MIN = $D0;
   FDDI_FC_K_SYNC_LLC_MAX = $D7;
   FDDI_FC_K_IMPLEMENTOR_MIN = $60;
   FDDI_FC_K_IMPLEMENTOR_MAX = $6F;
   FDDI_FC_K_RESERVED_MIN = $70;
   FDDI_FC_K_RESERVED_MAX = $7F;
   FDDI_EXTENDED_SAP = $AA;
   FDDI_UI_CMD = $03;

type
   Pfddi_8022_1_hdr = ^fddi_8022_1_hdr;
   fddi_8022_1_hdr = record
        dsap : __u8;
        ssap : __u8;
        ctrl : __u8;
     end;

   Pfddi_8022_2_hdr = ^fddi_8022_2_hdr;
   fddi_8022_2_hdr = record
        dsap : __u8;
        ssap : __u8;
        ctrl_1 : __u8;
        ctrl_2 : __u8;
     end;


const
   FDDI_K_OUI_LEN = 3;
type
   Pfddi_snap_hdr = ^fddi_snap_hdr;
   fddi_snap_hdr = record
        dsap : __u8;
        ssap : __u8;
        ctrl : __u8;
        oui : array[0..(FDDI_K_OUI_LEN)-1] of __u8;
        ethertype : __u16;
     end;

   Pfddihdr = ^fddihdr;
   fddihdr = record
        fc : __u8;
        daddr : array[0..(FDDI_K_ALEN)-1] of __u8;
        saddr : array[0..(FDDI_K_ALEN)-1] of __u8;
        hdr : record
            case longint of
               0 : ( llc_8022_1 : fddi_8022_1_hdr );
               1 : ( llc_8022_2 : fddi_8022_2_hdr );
               2 : ( llc_snap : fddi_snap_hdr );
            end;
     end;

   Pfddi_statistics = ^fddi_statistics;
   fddi_statistics = record
        rx_packets : __u32;
        tx_packets : __u32;
        rx_bytes : __u32;
        tx_bytes : __u32;
        rx_errors : __u32;
        tx_errors : __u32;
        rx_dropped : __u32;
        tx_dropped : __u32;
        multicast : __u32;
        transmit_collision : __u32;
        rx_length_errors : __u32;
        rx_over_errors : __u32;
        rx_crc_errors : __u32;
        rx_frame_errors : __u32;
        rx_fifo_errors : __u32;
        rx_missed_errors : __u32;
        tx_aborted_errors : __u32;
        tx_carrier_errors : __u32;
        tx_fifo_errors : __u32;
        tx_heartbeat_errors : __u32;
        tx_window_errors : __u32;
        rx_compressed : __u32;
        tx_compressed : __u32;
        smt_station_id : array[0..7] of __u8;
        smt_op_version_id : __u32;
        smt_hi_version_id : __u32;
        smt_lo_version_id : __u32;
        smt_user_data : array[0..31] of __u8;
        smt_mib_version_id : __u32;
        smt_mac_cts : __u32;
        smt_non_master_cts : __u32;
        smt_master_cts : __u32;
        smt_available_paths : __u32;
        smt_config_capabilities : __u32;
        smt_config_policy : __u32;
        smt_connection_policy : __u32;
        smt_t_notify : __u32;
        smt_stat_rpt_policy : __u32;
        smt_trace_max_expiration : __u32;
        smt_bypass_present : __u32;
        smt_ecm_state : __u32;
        smt_cf_state : __u32;
        smt_remote_disconnect_flag : __u32;
        smt_station_status : __u32;
        smt_peer_wrap_flag : __u32;
        smt_time_stamp : __u32;
        smt_transition_time_stamp : __u32;
        mac_frame_status_Functions : __u32;
        mac_t_max_capability : __u32;
        mac_tvx_capability : __u32;
        mac_available_paths : __u32;
        mac_current_path : __u32;
        mac_upstream_nbr : array[0..(FDDI_K_ALEN)-1] of __u8;
        mac_downstream_nbr : array[0..(FDDI_K_ALEN)-1] of __u8;
        mac_old_upstream_nbr : array[0..(FDDI_K_ALEN)-1] of __u8;
        mac_old_downstream_nbr : array[0..(FDDI_K_ALEN)-1] of __u8;
        mac_dup_address_test : __u32;
        mac_requested_paths : __u32;
        mac_downstream_port_type : __u32;
        mac_smt_address : array[0..(FDDI_K_ALEN)-1] of __u8;
        mac_t_req : __u32;
        mac_t_neg : __u32;
        mac_t_max : __u32;
        mac_tvx_value : __u32;
        mac_frame_cts : __u32;
        mac_copied_cts : __u32;
        mac_transmit_cts : __u32;
        mac_error_cts : __u32;
        mac_lost_cts : __u32;
        mac_frame_error_threshold : __u32;
        mac_frame_error_ratio : __u32;
        mac_rmt_state : __u32;
        mac_da_flag : __u32;
        mac_una_da_flag : __u32;
        mac_frame_error_flag : __u32;
        mac_ma_unitdata_available : __u32;
        mac_hardware_present : __u32;
        mac_ma_unitdata_enable : __u32;
        path_tvx_lower_bound : __u32;
        path_t_max_lower_bound : __u32;
        path_max_t_req : __u32;
        path_configuration : array[0..7] of __u32;
        port_my_type : array[0..1] of __u32;
        port_neighbor_type : array[0..1] of __u32;
        port_connection_policies : array[0..1] of __u32;
        port_mac_indicated : array[0..1] of __u32;
        port_current_path : array[0..1] of __u32;
        port_requested_paths : array[0..(3 * 2)-1] of __u8;
        port_mac_placement : array[0..1] of __u32;
        port_available_paths : array[0..1] of __u32;
        port_pmd_class : array[0..1] of __u32;
        port_connection_capabilities : array[0..1] of __u32;
        port_bs_flag : array[0..1] of __u32;
        port_lct_fail_cts : array[0..1] of __u32;
        port_ler_estimate : array[0..1] of __u32;
        port_lem_reject_cts : array[0..1] of __u32;
        port_lem_cts : array[0..1] of __u32;
        port_ler_cutoff : array[0..1] of __u32;
        port_ler_alarm : array[0..1] of __u32;
        port_connect_state : array[0..1] of __u32;
        port_pcm_state : array[0..1] of __u32;
        port_pc_withhold : array[0..1] of __u32;
        port_ler_flag : array[0..1] of __u32;
        port_hardware_present : array[0..1] of __u32;
     end;

// Translated from /linux/if_slip.h


const
   SL_MODE_SLIP = 0;
   SL_MODE_CSLIP = 1;
   SL_MODE_KISS = 4;
   SL_OPT_SIXBIT = 2;
   SL_OPT_ADAPTIVE = 8;

   SIOCDEVPRIVATE       = $89F0;
   SIOCPROTOPRIVATE     = $89E0;

   SIOCSKEEPALIVE = SIOCDEVPRIVATE;
   SIOCGKEEPALIVE = SIOCDEVPRIVATE + 1;
   SIOCSOUTFILL = SIOCDEVPRIVATE + 2;
   SIOCGOUTFILL = SIOCDEVPRIVATE + 3;
   SIOCSLEASE = SIOCDEVPRIVATE + 4;
   SIOCGLEASE = SIOCDEVPRIVATE + 5;

// Translated from /linux/if_tr.h


const
   TR_ALEN = 6;
   AC = $10;
   LLC_FRAME = $40;

    EXTENDED_SAP = $AA;
    UI_CMD = $03;


type
       Ptrh_hdr = ^trh_hdr;
       trh_hdr = record
            ac : __u8;
            fc : __u8;
            daddr : array[0..(TR_ALEN)-1] of __u8;
            saddr : array[0..(TR_ALEN)-1] of __u8;
            rcf : __u16;
            rseg : array[0..7] of __u16;
         end;

       Ptrllc = ^trllc;
       trllc = record
            dsap : __u8;
            ssap : __u8;
            llc : __u8;
            protid : array[0..2] of __u8;
            ethertype : __u16;
         end;

       Ptr_statistics = ^tr_statistics;
       tr_statistics = record
            rx_packets : dword;
            tx_packets : dword;
            rx_bytes : dword;
            tx_bytes : dword;
            rx_errors : dword;
            tx_errors : dword;
            rx_dropped : dword;
            tx_dropped : dword;
            multicast : dword;
            transmit_collision : dword;
            line_errors : dword;
            internal_errors : dword;
            burst_errors : dword;
            A_C_errors : dword;
            abort_delimiters : dword;
            lost_frames : dword;
            recv_congest_count : dword;
            frame_copied_errors : dword;
            frequency_errors : dword;
            token_errors : dword;
            dummy1 : dword;
         end;


    const
       TR_RII = $80;
       TR_RCF_DIR_BIT = $80;
       TR_RCF_LEN_MASK = $1f00;
       TR_RCF_BROADCAST = $8000;
       TR_RCF_LIMITED_BROADCAST = $C000;
       TR_RCF_FRAME2K = $20;
       TR_RCF_BROADCAST_MASK = $C000;
       TR_MAXRIFLEN = 18;

const
  TR_HLEN      = (SizeOf(trh_hdr) + SizeOf(trllc));

// Translated from linux/ppp_defs.h

    const
       PPP_HDRLEN = 4;
       PPP_FCSLEN = 2;
       PPP_MRU = 1500;

Function PPP_ADDRESS(const p): __u8;
Function PPP_CONTROL(const p): __u8;
Function PPP_PROTOCOL(const p): __u16;


Const
       PPP_ALLSTATIONS = $ff;
       PPP_UI = $03;
       PPP_FLAG = $7e;
       PPP_ESCAPE = $7d;
       PPP_TRANS = $20;
       PPP_IP = $21;
       PPP_AT = $29;
       PPP_IPX = $2b;
       PPP_VJC_COMP = $2d;
       PPP_VJC_UNCOMP = $2f;
       PPP_MP = $3d;
       PPP_IPV6 = $57;
       PPP_COMPFRAG = $fb;
       PPP_COMP = $fd;
       PPP_IPCP = $8021;
       PPP_ATCP = $8029;
       PPP_IPXCP = $802b;
       PPP_IPV6CP = $8057;
       PPP_CCPFRAG = $80fb;
       PPP_CCP = $80fd;
       PPP_LCP = $c021;
       PPP_PAP = $c023;
       PPP_LQR = $c025;
       PPP_CHAP = $c223;
       PPP_CBCP = $c029;
       PPP_INITFCS = $ffff;
       PPP_GOODFCS = $f0b8;

    type

       Pext_accm = ^ext_accm;
       ext_accm = packed array[0..8-1] of __u32;

       NPmode =  Longint;
       Const
         NPMODE_PASS = 0;
         NPMODE_DROP = 1;
         NPMODE_ERROR = 2;
         NPMODE_QUEUE = 3;

    type
       Ppppstat = ^pppstat;
       pppstat = record
            ppp_discards : __u32;
            ppp_ibytes : __u32;
            ppp_ioctects : __u32;
            ppp_ipackets : __u32;
            ppp_ierrors : __u32;
            ppp_ilqrs : __u32;
            ppp_obytes : __u32;
            ppp_ooctects : __u32;
            ppp_opackets : __u32;
            ppp_oerrors : __u32;
            ppp_olqrs : __u32;
         end;

       Pvjstat = ^vjstat;
       vjstat = record
            vjs_packets : __u32;
            vjs_compressed : __u32;
            vjs_searches : __u32;
            vjs_misses : __u32;
            vjs_uncompressedin : __u32;
            vjs_compressedin : __u32;
            vjs_errorin : __u32;
            vjs_tossed : __u32;
         end;

       Pcompstat = ^compstat;
       compstat = record
            unc_bytes : __u32;
            unc_packets : __u32;
            comp_bytes : __u32;
            comp_packets : __u32;
            inc_bytes : __u32;
            inc_packets : __u32;
            in_count : __u32;
            bytes_out : __u32;
            ratio : double;
         end;

       Pppp_stats = ^ppp_stats;
       ppp_stats = record
            p : pppstat;
            vj : vjstat;
         end;

       Pppp_comp_stats = ^ppp_comp_stats;
       ppp_comp_stats = record
            c : compstat;
            d : compstat;
         end;

      __kernel_time_t = Longint;
      ppp_idle = record
          xmit_idle: __kernel_time_t;
          recv_idle: __kernel_time_t;
      end;
      Pppp_idle = ^ppp_idle;

// Translated from linux/ppp-comp.h

    const
       DO_BSD_COMPRESS = 1;
       DO_DEFLATE = 1;
       DO_PREDICTOR_1 = 0;
       DO_PREDICTOR_2 = 0;

    type
       Pcompressor = ^compressor;
       compressor = record
            compress_proto : longint;
            comp_alloc : Function (options:Pbyte; opt_len:longint):pointer;cdecl;
            comp_free : procedure (state:pointer);
            comp_init : Function (state:pointer; options:Pbyte; opt_len:longint; _unit:longint; opthdr:longint;
                         debug:longint):longint;
            comp_reset : procedure (state:pointer);
            compress : Function (state:pointer; rptr:Pbyte; obuf:Pbyte; isize:longint; osize:longint):longint;
            comp_stat : procedure (state:pointer; stats:Pcompstat);
            decomp_alloc : Function (options:Pbyte; opt_len:longint):pointer;
            decomp_free : procedure (state:pointer);
            decomp_init : Function (state:pointer; options:Pbyte; opt_len:longint; _unit:longint; opthdr:longint;
                         mru:longint; debug:longint):longint;
            decomp_reset : procedure (state:pointer);
            decompress : Function (state:pointer; ibuf:Pbyte; isize:longint; obuf:Pbyte; osize:longint):longint;
            incomp : procedure (state:pointer; ibuf:Pbyte; icnt:longint);
            decomp_stat : procedure (state:pointer; stats:Pcompstat);
         end;


    const
       DECOMP_ERROR = -(1);
       DECOMP_FATALERROR = -(2);
       CCP_CONFREQ = 1;
       CCP_CONFACK = 2;
       CCP_TERMREQ = 5;
       CCP_TERMACK = 6;
       CCP_RESETREQ = 14;
       CCP_RESETACK = 15;
       CCP_MAX_OPTION_LENGTH = 32;


Function CCP_CODE(dp: Pointer): Byte;
Function CCP_ID(dp: Pointer): Byte;
Function CCP_LENGTH(dp: Pointer): Word;
Function CCP_OPT_CODE(dp: Pointer): Byte;
Function CCP_OPT_LENGTH(dp: Pointer): Byte;

const
       CCP_HDRLEN      = 4;
       CCP_OPT_MINLEN = 2;
       CI_BSD_COMPRESS = 21;
       CILEN_BSD_COMPRESS = 3;
       BSD_CURRENT_VERSION = 1;

Function BSD_NBITS(x: longint): longint;
Function BSD_VERSION(x: longint): longint;
Function BSD_MAKE_OPT(v, n: longint): longint;

    const
       BSD_MIN_BITS = 9;
       BSD_MAX_BITS = 15;
       CI_DEFLATE = 26;
       CI_DEFLATE_DRAFT = 24;
       CILEN_DEFLATE = 4;
       DEFLATE_MIN_SIZE = 8;
       DEFLATE_MAX_SIZE = 15;
       DEFLATE_METHOD_VAL = 8;

Function DEFLATE_SIZE(x: longint): longint;
Function DEFLATE_METHOD(x: longint): longint;
Function DEFLATE_MAKE_OPT(w: longint): longint;

    const
       DEFLATE_CHK_SEQUENCE = 0;
       CI_PREDICTOR_1 = 1;
       CILEN_PREDICTOR_1 = 2;
       CI_PREDICTOR_2 = 2;
       CILEN_PREDICTOR_2 = 2;

// Translated from linux/atalk.h

    const
       ATPORT_FIRST = 1;
       ATPORT_RESERVED = 128;
       ATPORT_LAST = 254;
       ATADDR_ANYNET = __u16(0);
       ATADDR_ANYNODE = __u8(0);
       ATADDR_ANYPORT = __u8(0);
       ATADDR_BCAST = __u8(255);

    const
       DDP_MAXSZ = 587;
       DDP_MAXHOPS = 15;
       SIOCATALKDIFADDR = SIOCPROTOPRIVATE + 0;
    type
       Pat_addr = ^at_addr;
       at_addr = record
            s_net : __u16;
            s_node : __u8;
         end;

       sa_family_t = Word; // From libc.

       Psockaddr_at = ^sockaddr_at;
       sockaddr_at = record
            sat_family : sa_family_t;
            sat_port : __u8;
            sat_addr : at_addr;
            sat_zero : array[0..7] of char;
         end;

       Pnetrange = ^netrange;
       netrange = record
            nr_phase : __u8;
            nr_firstnet : __u16;
            nr_lastnet : __u16;
         end;

       Patalk_route = ^atalk_route;
       atalk_route = record
            dev : Pointer; // pnet_device ??
            target : at_addr;
            gateway : at_addr;
            flags : longint;
            next : Patalk_route;
         end;

Const
  ATIF_PROBE      = 1;    { Probing for an address }
  ATIF_PROBE_FAIL = 2;    { Probe collided }

Type
       PATalkIFace = ^TATalkIFace;
       atalk_iface = record
           dev: Pointer;
           address: at_addr;
           status: longint;
           nets: netrange;
           next: PATalkIFace;
       end;
      TATalkIFace = atalk_iface;


       Patalk_sock = ^atalk_sock;
       atalk_sock = record
            dest_net : word;
            src_net : word;
            dest_node : byte;
            src_node : byte;
            dest_port : byte;
            src_port : byte;
         end;

// Translated from linux/igmp.h

    type
       Pigmphdr = ^igmphdr;
       igmphdr = record
            __type : __u8;
            code : __u8;
            csum : __u16;
            group : __u32;
         end;


    const
       IGMP_HOST_MEMBERSHIP_QUERY = $11;
       IGMP_HOST_MEMBERSHIP_REPORT = $12;
       IGMP_DVMRP = $13;
       IGMP_PIM = $14;
       IGMP_TRACE = $15;
       IGMP_HOST_NEW_MEMBERSHIP_REPORT = $16;
       IGMP_HOST_LEAVE_MESSAGE = $17;
       IGMP_MTRACE_RESP = $1e;
       IGMP_MTRACE = $1f;
       IGMP_DELAYING_MEMBER = $01;
       IGMP_IDLE_MEMBER = $02;
       IGMP_LAZY_MEMBER = $03;
       IGMP_SLEEPING_MEMBER = $04;
       IGMP_AWAKENING_MEMBER = $05;
       IGMP_MINLEN = 8;
       IGMP_MAX_HOST_REPORT_DELAY = 10;
       IGMP_TIMER_SCALE = 10;
       IGMP_AGE_THRESHOLD = 400;

{ ---------------------------------------------------------------------
    Borland compatibility types
  ---------------------------------------------------------------------}

Type
  TATalkRoute = atalk_route;

Implementation

Function PPP_ADDRESS(const p): __u8;

begin
  Result:=P__u8(@p)^;
end;

Function PPP_CONTROL(const p): __u8;
begin
  Result:=P__u8(Cardinal(@p) + SizeOf(__u8))^;
end;

Function PPP_PROTOCOL(const p): __u16;
begin
  Result:=P__u8(Cardinal(@p) + SizeOf(__u8)*2)^ shl 8;
  Result:=Result + P__u8(Cardinal(@p) + SizeOf(__u8)*3)^
end;

Function CCP_CODE(dp: Pointer): Byte;
begin
  Result:=PByte(dp)^;
end;

Function CCP_ID(dp: Pointer): Byte;
begin
  Inc(PByte(dp));
  Result:=PByte(dp)^;
end;

Function CCP_LENGTH(dp: Pointer): Word;
begin
  Inc(PByte(dp), 2);
  Result:=(PByte(dp)^ shl 8);
  Inc(PByte(dp), 1);
  Result:=Result + PByte(dp)^;
end;

Function CCP_OPT_CODE(dp: Pointer): Byte;
begin
  Result:=PByte(dp)^;
end;

Function CCP_OPT_LENGTH(dp: Pointer): Byte;
begin
  Inc(PByte(dp));
  Result:=PByte(dp)^;
end;

Function DEFLATE_SIZE(x: longint): longint;
begin
  Result:=(Cardinal(x) shr 4) + DEFLATE_MIN_SIZE;
end;

Function DEFLATE_METHOD(x: longint): longint;
begin
  Result:=x and $0F;
end;

Function DEFLATE_MAKE_OPT(w: longint): longint;
begin
  Result:=((w - DEFLATE_MIN_SIZE) shl 4) + DEFLATE_METHOD_VAL;
end;

Function BSD_NBITS(x: longint): longint;
begin
  Result:=(x and $1F);
end;

Function BSD_VERSION(x: longint): longint;
begin
  Result:=Cardinal(x) shr 5;
end;

Function BSD_MAKE_OPT(v, n: longint): longint;
begin
  Result:= longint((Cardinal(v) shl 5) or Cardinal(n));
end;

end.
