{
    Copyright (c) 1999-2000 by Michael Van Canneyt

    Unit containing definitions from the Linux CDROM kernel interface.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit lincd;

{$mode objfpc}

interface

uses
  baseunix,
  unix;

{ ---------------------------------------------------------------------
    cdrom.h header translation.
  ---------------------------------------------------------------------}

  { Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;

  TU8  = Byte;
  PU8  = ^TU8;
  TU32 = Cardinal;
  PU32 = ^TU32;

{$PACKRECORDS C}

const
   EDRIVE_CANT_DO_THIS = ESysEOPNOTSUPP; // = 95
   CDROMPAUSE = $5301;
   CDROMRESUME = $5302;
   CDROMPLAYMSF = $5303;
   CDROMPLAYTRKIND = $5304;
   CDROMREADTOCHDR = $5305;
   CDROMREADTOCENTRY = $5306;
   CDROMSTOP = $5307;
   CDROMSTART = $5308;
   CDROMEJECT = $5309;
   CDROMVOLCTRL = $530a;
   CDROMSUBCHNL = $530b;
   CDROMREADMODE2 = $530c;
   CDROMREADMODE1 = $530d;
   CDROMREADAUDIO = $530e;
   CDROMEJECT_SW = $530f;
   CDROMMULTISESSION = $5310;
   CDROM_GET_MCN = $5311;
   CDROM_GET_UPC = CDROM_GET_MCN;
   CDROMRESET = $5312;
   CDROMVOLREAD = $5313;
   CDROMREADRAW = $5314;
   CDROMREADCOOKED = $5315;
   CDROMSEEK = $5316;
   CDROMPLAYBLK = $5317;
   CDROMREADALL = $5318;
   CDROMGETSPINDOWN = $531d;
   CDROMSETSPINDOWN = $531e;
   CDROMCLOSETRAY = $5319;
   CDROM_SET_OPTIONS = $5320;
   CDROM_CLEAR_OPTIONS = $5321;
   CDROM_SELECT_SPEED = $5322;
   CDROM_SELECT_DISC = $5323;
   CDROM_MEDIA_CHANGED = $5325;
   CDROM_DRIVE_STATUS = $5326;
   CDROM_DISC_STATUS = $5327;
   CDROM_CHANGER_NSLOTS = $5328;
   CDROM_LOCKDOOR = $5329;
   CDROM_DEBUG = $5330;
   CDROM_GET_CAPABILITY = $5331;
   CDROMAUDIOBUFSIZ = $5382;
   DVD_READ_STRUCT = $5390;
   DVD_WRITE_STRUCT = $5391;
   DVD_AUTH = $5392;
   CDROM_SEND_PACKET = $5393;
   CDROM_NEXT_WRITABLE = $5394;
   CDROM_LAST_WRITTEN = $5395;

type
   Pcdrom_msf0 = ^Tcdrom_msf0;
   Tcdrom_msf0 = record
        minute : Tu8;
        second : Tu8;
        frame : Tu8;
     end;

   Pcdrom_addr = ^Tcdrom_addr;
   Tcdrom_addr = record
       case longint of
          0 : ( msf : Tcdrom_msf0 );
          1 : ( lba : longint );
       end;

   Pcdrom_msf = ^Tcdrom_msf;
   Tcdrom_msf = record
        cdmsf_min0 : Tu8;
        cdmsf_sec0 : Tu8;
        cdmsf_frame0 : Tu8;
        cdmsf_min1 : Tu8;
        cdmsf_sec1 : Tu8;
        cdmsf_frame1 : Tu8;
     end;

   Pcdrom_ti = ^Tcdrom_ti;
   Tcdrom_ti = record
        cdti_trk0 : Tu8;
        cdti_ind0 : Tu8;
        cdti_trk1 : Tu8;
        cdti_ind1 : Tu8;
     end;

   Pcdrom_tochdr = ^Tcdrom_tochdr;
   Tcdrom_tochdr = record
        cdth_trk0 : Tu8;
        cdth_trk1 : Tu8;
     end;

   Pcdrom_volctrl = ^Tcdrom_volctrl;
   Tcdrom_volctrl = record
        channel0 : Tu8;
        channel1 : Tu8;
        channel2 : Tu8;
        channel3 : Tu8;
     end;

   Pcdrom_subchnl = ^Tcdrom_subchnl;
   Tcdrom_subchnl = record
        cdsc_format : Tu8;
        cdsc_audiostatus : Tu8;
        flag0 : word;
        cdsc_trk : Tu8;
        cdsc_ind : Tu8;
        cdsc_absaddr : Tcdrom_addr;
        cdsc_reladdr : Tcdrom_addr;
     end;

const
   bm_Tcdrom_subchnl_cdsc_adr = $F;
   bp_Tcdrom_subchnl_cdsc_adr = 0;
   bm_Tcdrom_subchnl_cdsc_ctrl = $F0;
   bp_Tcdrom_subchnl_cdsc_ctrl = 4;
function cdsc_adr(var a : Tcdrom_subchnl) : Tu8;
procedure set_cdsc_adr(var a : Tcdrom_subchnl; __cdsc_adr : Tu8);
function cdsc_ctrl(var a : Tcdrom_subchnl) : Tu8;
procedure set_cdsc_ctrl(var a : Tcdrom_subchnl; __cdsc_ctrl : Tu8);

type
   Pcdrom_tocentry = ^Tcdrom_tocentry;
   Tcdrom_tocentry = record
        cdte_track : Tu8;
        flag0 : tu8;
        cdte_format : Tu8;
        cdte_addr : Tcdrom_addr;
        cdte_datamode : Tu8;
     end;

const
   bm_Tcdrom_tocentry_cdte_adr = $F;
   bp_Tcdrom_tocentry_cdte_adr = 0;
   bm_Tcdrom_tocentry_cdte_ctrl = $F0;
   bp_Tcdrom_tocentry_cdte_ctrl = 4;
function cdte_adr(var a : Tcdrom_tocentry) : Tu8;
procedure set_cdte_adr(var a : Tcdrom_tocentry; __cdte_adr : Tu8);
function cdte_ctrl(var a : Tcdrom_tocentry) : Tu8;
procedure set_cdte_ctrl(var a : Tcdrom_tocentry; __cdte_ctrl : Tu8);

type
   Pcdrom_read = ^Tcdrom_read;
   Tcdrom_read = record
        cdread_lba : longint;
        cdread_bufaddr : Pchar;
        cdread_buflen : longint;
     end;

   Pcdrom_read_audio = ^Tcdrom_read_audio;
   Tcdrom_read_audio = record
        addr : Tcdrom_addr;
        addr_format : Tu8;
        nframes : longint;
        buf : Pu8;
     end;

   Pcdrom_multisession = ^Tcdrom_multisession;
   Tcdrom_multisession = record
        addr : Tcdrom_addr;
        xa_flag : Tu8;
        addr_format : Tu8;
     end;

   Pcdrom_mcn = ^Tcdrom_mcn;
   Tcdrom_mcn = record
        medium_catalog_number : array[0..13] of Tu8;
     end;

   Pcdrom_blk = ^Tcdrom_blk;
   Tcdrom_blk = record
        from : dword;
        len : word;
     end;


const
   CDROM_PACKET_SIZE = 12;
   CGC_DATA_UNKNOWN = 0;
   CGC_DATA_WRITE = 1;
   CGC_DATA_READ = 2;
   CGC_DATA_NONE = 3;

// Moved here
type
   Prequest_sense = ^Trequest_sense;
   Trequest_sense = record
        flag0 : word;
        segment_number : Tu8;
        flag1 : word;
        information : array[0..3] of Tu8;
        add_sense_len : Tu8;
        command_info : array[0..3] of Tu8;
        asc : Tu8;
        ascq : Tu8;
        fruc : Tu8;
        sks : array[0..2] of Tu8;
        asb : array[0..45] of Tu8;
     end;
   Pcdrom_generic_command = ^Tcdrom_generic_command;
   Tcdrom_generic_command = record
        cmd : array[0..(CDROM_PACKET_SIZE)-1] of byte;
        buffer : Pbyte;
        buflen : dword;
        stat : longint;
        sense : Prequest_sense;
        data_direction : byte;
        quiet : longint;
        timeout : longint;
        reserved : array[0..0] of pointer;
     end;


const
   CD_MINS = 74;
   CD_SECS = 60;
   CD_FRAMES = 75;
   CD_SYNC_SIZE = 12;
   CD_MSF_OFFSET = 150;
   CD_CHUNK_SIZE = 24;
   CD_NUM_OF_CHUNKS = 98;
   CD_FRAMESIZE_SUB = 96;
   CD_HEAD_SIZE = 4;
   CD_SUBHEAD_SIZE = 8;
   CD_EDC_SIZE = 4;
   CD_ZERO_SIZE = 8;
   CD_ECC_SIZE = 276;
   CD_FRAMESIZE = 2048;
   CD_FRAMESIZE_RAW = 2352;
   CD_FRAMESIZE_RAWER = 2646;
   CD_FRAMESIZE_RAW1 = CD_FRAMESIZE_RAW - CD_SYNC_SIZE;
   CD_FRAMESIZE_RAW0 = (CD_FRAMESIZE_RAW - CD_SYNC_SIZE) - CD_HEAD_SIZE;
   CD_XA_HEAD = CD_HEAD_SIZE + CD_SUBHEAD_SIZE;
   CD_XA_TAIL = CD_EDC_SIZE + CD_ECC_SIZE;
   CD_XA_SYNC_HEAD = CD_SYNC_SIZE + CD_XA_HEAD;
   CDROM_LBA = $01;
   CDROM_MSF = $02;
   CDROM_DATA_TRACK = $04;
   CDROM_LEADOUT = $AA;
   CDROM_AUDIO_INVALID = $00;
   CDROM_AUDIO_PLAY = $11;
   CDROM_AUDIO_PAUSED = $12;
   CDROM_AUDIO_COMPLETED = $13;
   CDROM_AUDIO_ERROR = $14;
   CDROM_AUDIO_NO_STATUS = $15;
   CDC_CLOSE_TRAY = $1;
   CDC_OPEN_TRAY = $2;
   CDC_LOCK = $4;
   CDC_SELECT_SPEED = $8;
   CDC_SELECT_DISC = $10;
   CDC_MULTI_SESSION = $20;
   CDC_MCN = $40;
   CDC_MEDIA_CHANGED = $80;
   CDC_PLAY_AUDIO = $100;
   CDC_RESET = $200;
   CDC_IOCTLS = $400;
   CDC_DRIVE_STATUS = $800;
   CDC_GENERIC_PACKET = $1000;
   CDC_CD_R = $2000;
   CDC_CD_RW = $4000;
   CDC_DVD = $8000;
   CDC_DVD_R = $10000;
   CDC_DVD_RAM = $20000;
   CDS_NO_INFO = 0;
   CDS_NO_DISC = 1;
   CDS_TRAY_OPEN = 2;
   CDS_DRIVE_NOT_READY = 3;
   CDS_DISC_OK = 4;
   CDS_AUDIO = 100;
   CDS_DATA_1 = 101;
   CDS_DATA_2 = 102;
   CDS_XA_2_1 = 103;
   CDS_XA_2_2 = 104;
   CDS_MIXED = 105;
   CDO_AUTO_CLOSE = $1;
   CDO_AUTO_EJECT = $2;
   CDO_USE_FFLAGS = $4;
   CDO_LOCK = $8;
   CDO_CHECK_TYPE = $10;
{ was #define dname def_expr }
function CDSL_NONE : longint;
    { return type might be wrong }

{ was #define dname def_expr }
function CDSL_CURRENT : longint;


const
   CD_PART_MAX = 64;
   CD_PART_MASK = CD_PART_MAX - 1;
   GPCMD_BLANK = $a1;
   GPCMD_CLOSE_TRACK = $5b;
   GPCMD_FLUSH_CACHE = $35;
   GPCMD_FORMAT_UNIT = $04;
   GPCMD_GET_CONFIGURATION = $46;
   GPCMD_GET_EVENT_STATUS_NOTIFICATION = $4a;
   GPCMD_GET_PERFORMANCE = $ac;
   GPCMD_INQUIRY = $12;
   GPCMD_LOAD_UNLOAD = $a6;
   GPCMD_MECHANISM_STATUS = $bd;
   GPCMD_MODE_SELECT_10 = $55;
   GPCMD_MODE_SENSE_10 = $5a;
   GPCMD_PAUSE_RESUME = $4b;
   GPCMD_PLAY_AUDIO_10 = $45;
   GPCMD_PLAY_AUDIO_MSF = $47;
   GPCMD_PLAY_AUDIO_TI = $48;
   GPCMD_PLAY_CD = $bc;
   GPCMD_PREVENT_ALLOW_MEDIUM_REMOVAL = $1e;
   GPCMD_READ_10 = $28;
   GPCMD_READ_12 = $a8;
   GPCMD_READ_CDVD_CAPACITY = $25;
   GPCMD_READ_CD = $be;
   GPCMD_READ_CD_MSF = $b9;
   GPCMD_READ_DISC_INFO = $51;
   GPCMD_READ_DVD_STRUCTURE = $ad;
   GPCMD_READ_FORMAT_CAPACITIES = $23;
   GPCMD_READ_HEADER = $44;
   GPCMD_READ_TRACK_RZONE_INFO = $52;
   GPCMD_READ_SUBCHANNEL = $42;
   GPCMD_READ_TOC_PMA_ATIP = $43;
   GPCMD_REPAIR_RZONE_TRACK = $58;
   GPCMD_REPORT_KEY = $a4;
   GPCMD_REQUEST_SENSE = $03;
   GPCMD_RESERVE_RZONE_TRACK = $53;
   GPCMD_SCAN = $ba;
   GPCMD_SEEK = $2b;
   GPCMD_SEND_DVD_STRUCTURE = $ad;
   GPCMD_SEND_EVENT = $a2;
   GPCMD_SEND_KEY = $a3;
   GPCMD_SEND_OPC = $54;
   GPCMD_SET_READ_AHEAD = $a7;
   GPCMD_SET_STREAMING = $b6;
   GPCMD_START_STOP_UNIT = $1b;
   GPCMD_STOP_PLAY_SCAN = $4e;
   GPCMD_TEST_UNIT_READY = $00;
   GPCMD_VERIFY_10 = $2f;
   GPCMD_WRITE_10 = $2a;
   GPCMD_WRITE_AND_VERIFY_10 = $2e;
   GPCMD_SET_SPEED = $bb;
   GPCMD_PLAYAUDIO_TI = $48;
   GPCMD_GET_MEDIA_STATUS = $da;
   GPMODE_R_W_ERROR_PAGE = $01;
   GPMODE_WRITE_PARMS_PAGE = $05;
   GPMODE_AUDIO_CTL_PAGE = $0e;
   GPMODE_POWER_PAGE = $1a;
   GPMODE_FAULT_FAIL_PAGE = $1c;
   GPMODE_TO_PROTECT_PAGE = $1d;
   GPMODE_CAPABILITIES_PAGE = $2a;
   GPMODE_ALL_PAGES = $3f;
   GPMODE_CDROM_PAGE = $0d;
   DVD_STRUCT_PHYSICAL = $00;
   DVD_STRUCT_COPYRIGHT = $01;
   DVD_STRUCT_DISCKEY = $02;
   DVD_STRUCT_BCA = $03;
   DVD_STRUCT_MANUFACT = $04;

type
   Pdvd_layer = ^Tdvd_layer;
   Tdvd_layer = record
        flag0 : longint;
        start_sector : Tu32;
        end_sector : Tu32;
        end_sector_l0 : Tu32;
     end;

const
   bm_Tdvd_layer_book_version = $F;
   bp_Tdvd_layer_book_version = 0;
   bm_Tdvd_layer_book_type = $F0;
   bp_Tdvd_layer_book_type = 4;
   bm_Tdvd_layer_min_rate = $F00;
   bp_Tdvd_layer_min_rate = 8;
   bm_Tdvd_layer_disc_size = $F000;
   bp_Tdvd_layer_disc_size = 12;
   bm_Tdvd_layer_layer_type = $F0000;
   bp_Tdvd_layer_layer_type = 16;
   bm_Tdvd_layer_track_path = $100000;
   bp_Tdvd_layer_track_path = 20;
   bm_Tdvd_layer_nlayers = $600000;
   bp_Tdvd_layer_nlayers = 21;
   bm_Tdvd_layer_track_density = $7800000;
   bp_Tdvd_layer_track_density = 23;
   bm_Tdvd_layer_linear_density = $78000000;
   bp_Tdvd_layer_linear_density = 27;
   bm_Tdvd_layer_bca = $80000000;
   bp_Tdvd_layer_bca = 31;
function book_version(var a : Tdvd_layer) : Tu8;
procedure set_book_version(var a : Tdvd_layer; __book_version : Tu8);
function book_type(var a : Tdvd_layer) : Tu8;
procedure set_book_type(var a : Tdvd_layer; __book_type : Tu8);
function min_rate(var a : Tdvd_layer) : Tu8;
procedure set_min_rate(var a : Tdvd_layer; __min_rate : Tu8);
function disc_size(var a : Tdvd_layer) : Tu8;
procedure set_disc_size(var a : Tdvd_layer; __disc_size : Tu8);
function layer_type(var a : Tdvd_layer) : Tu8;
procedure set_layer_type(var a : Tdvd_layer; __layer_type : Tu8);
function track_path(var a : Tdvd_layer) : Tu8;
procedure set_track_path(var a : Tdvd_layer; __track_path : Tu8);
function nlayers(var a : Tdvd_layer) : Tu8;
procedure set_nlayers(var a : Tdvd_layer; __nlayers : Tu8);
function track_density(var a : Tdvd_layer) : Tu8;
procedure set_track_density(var a : Tdvd_layer; __track_density : Tu8);
function linear_density(var a : Tdvd_layer) : Tu8;
procedure set_linear_density(var a : Tdvd_layer; __linear_density : Tu8);
function bca(var a : Tdvd_layer) : Tu8;
procedure set_bca(var a : Tdvd_layer; __bca : Tu8);

const
   DVD_LAYERS = 4;

type
   Pdvd_physical = ^Tdvd_physical;
   Tdvd_physical = record
        _type : Tu8;
        layer_num : Tu8;
        layer : array[0..(DVD_LAYERS)-1] of Tdvd_layer;
     end;

   Pdvd_copyright = ^Tdvd_copyright;
   Tdvd_copyright = record
        _type : Tu8;
        layer_num : Tu8;
        cpst : Tu8;
        rmi : Tu8;
     end;

   Pdvd_disckey = ^Tdvd_disckey;
   Tdvd_disckey = record
        _type : Tu8;
        flag0 : word;
        value : array[0..2047] of Tu8;
     end;

const
   bm_Tdvd_disckey_agid = $3;
   bp_Tdvd_disckey_agid = 0;
function agid(var a : Tdvd_disckey) : dword;
procedure set_agid(var a : Tdvd_disckey; __agid : dword);

type
   Pdvd_bca = ^Tdvd_bca;
   Tdvd_bca = record
        _type : Tu8;
        len : longint;
        value : array[0..187] of Tu8;
     end;

   Pdvd_manufact = ^Tdvd_manufact;
   Tdvd_manufact = record
        _type : Tu8;
        layer_num : Tu8;
        len : longint;
        value : array[0..2047] of Tu8;
     end;


   Pdvd_struct = ^Tdvd_struct;
   Tdvd_struct = record
       case longint of
          0 : ( _type : Tu8 );
          1 : ( physical : Tdvd_physical );
          2 : ( copyright : Tdvd_copyright );
          3 : ( disckey : Tdvd_disckey );
          4 : ( bca : Tdvd_bca );
          5 : ( manufact : Tdvd_manufact );
       end;

const
   DVD_LU_SEND_AGID = 0;
   DVD_HOST_SEND_CHALLENGE = 1;
   DVD_LU_SEND_KEY1 = 2;
   DVD_LU_SEND_CHALLENGE = 3;
   DVD_HOST_SEND_KEY2 = 4;
   DVD_AUTH_ESTABLISHED = 5;
   DVD_AUTH_FAILURE = 6;
   DVD_LU_SEND_TITLE_KEY = 7;
   DVD_LU_SEND_ASF = 8;
   DVD_INVALIDATE_AGID = 9;
   DVD_LU_SEND_RPC_STATE = 10;
   DVD_HOST_SEND_RPC_STATE = 11;

type

   Pdvd_key = ^Tdvd_key;
   Tdvd_key = Tu8;

   Pdvd_challenge = ^Tdvd_challenge;
   Tdvd_challenge = Tu8;
   Pdvd_lu_send_agid = ^Tdvd_lu_send_agid;
   Tdvd_lu_send_agid = record
        _type : Tu8;
        flag0 : word;
     end;

const
   bm_Tdvd_lu_send_agid_agid = $3;
   bp_Tdvd_lu_send_agid_agid = 0;
function agid(var a : Tdvd_lu_send_agid) : dword;
procedure set_agid(var a : Tdvd_lu_send_agid; __agid : dword);

type
   Pdvd_host_send_challenge = ^Tdvd_host_send_challenge;
   Tdvd_host_send_challenge = record
        _type : Tu8;
        flag0 : word;
        chal : Tdvd_challenge;
     end;

const
   bm_Tdvd_host_send_challenge_agid = $3;
   bp_Tdvd_host_send_challenge_agid = 0;
function agid(var a : Tdvd_host_send_challenge) : dword;
procedure set_agid(var a : Tdvd_host_send_challenge; __agid : dword);

type
   Pdvd_send_key = ^Tdvd_send_key;
   Tdvd_send_key = record
        _type : Tu8;
        flag0 : word;
        key : Tdvd_key;
     end;

const
   bm_Tdvd_send_key_agid = $3;
   bp_Tdvd_send_key_agid = 0;
function agid(var a : Tdvd_send_key) : dword;
procedure set_agid(var a : Tdvd_send_key; __agid : dword);

type
   Pdvd_lu_send_challenge = ^Tdvd_lu_send_challenge;
   Tdvd_lu_send_challenge = record
        _type : Tu8;
        flag0 : word;
        chal : Tdvd_challenge;
     end;

const
   bm_Tdvd_lu_send_challenge_agid = $3;
   bp_Tdvd_lu_send_challenge_agid = 0;
function agid(var a : Tdvd_lu_send_challenge) : dword;
procedure set_agid(var a : Tdvd_lu_send_challenge; __agid : dword);

const
   DVD_CPM_NO_COPYRIGHT = 0;
   DVD_CPM_COPYRIGHTED = 1;
   DVD_CP_SEC_NONE = 0;
   DVD_CP_SEC_EXIST = 1;
   DVD_CGMS_UNRESTRICTED = 0;
   DVD_CGMS_SINGLE = 2;
   DVD_CGMS_RESTRICTED = 3;

type
   Pdvd_lu_send_title_key = ^Tdvd_lu_send_title_key;
   Tdvd_lu_send_title_key = record
        _type : Tu8;
        flag0 : word;
        title_key : Tdvd_key;
        lba : longint;
        flag1 : word;
     end;

const
   bm_Tdvd_lu_send_title_key_agid = $3;
   bp_Tdvd_lu_send_title_key_agid = 0;
   bm_Tdvd_lu_send_title_key_cpm = $1;
   bp_Tdvd_lu_send_title_key_cpm = 0;
   bm_Tdvd_lu_send_title_key_cp_sec = $2;
   bp_Tdvd_lu_send_title_key_cp_sec = 1;
   bm_Tdvd_lu_send_title_key_cgms = $C;
   bp_Tdvd_lu_send_title_key_cgms = 2;
function agid(var a : Tdvd_lu_send_title_key) : dword;
procedure set_agid(var a : Tdvd_lu_send_title_key; __agid : dword);
function cpm(var a : Tdvd_lu_send_title_key) : dword;
procedure set_cpm(var a : Tdvd_lu_send_title_key; __cpm : dword);
function cp_sec(var a : Tdvd_lu_send_title_key) : dword;
procedure set_cp_sec(var a : Tdvd_lu_send_title_key; __cp_sec : dword);
function cgms(var a : Tdvd_lu_send_title_key) : dword;
procedure set_cgms(var a : Tdvd_lu_send_title_key; __cgms : dword);

type
   Pdvd_lu_send_asf = ^Tdvd_lu_send_asf;
   Tdvd_lu_send_asf = record
        _type : Tu8;
        flag0 : word;
     end;

const
   bm_Tdvd_lu_send_asf_agid = $3;
   bp_Tdvd_lu_send_asf_agid = 0;
   bm_Tdvd_lu_send_asf_asf = $4;
   bp_Tdvd_lu_send_asf_asf = 2;
function agid(var a : Tdvd_lu_send_asf) : dword;
procedure set_agid(var a : Tdvd_lu_send_asf; __agid : dword);
function asf(var a : Tdvd_lu_send_asf) : dword;
procedure set_asf(var a : Tdvd_lu_send_asf; __asf : dword);

type
   Pdvd_host_send_rpcstate = ^Tdvd_host_send_rpcstate;
   Tdvd_host_send_rpcstate = record
        _type : Tu8;
        pdrc : Tu8;
     end;

   Pdvd_lu_send_rpcstate = ^Tdvd_lu_send_rpcstate;
   Tdvd_lu_send_rpcstate = record
        flag0 : word;
        region_mask : Tu8;
        rpc_scheme : Tu8;
     end;

const
   bm_Tdvd_lu_send_rpcstate_type = $3;
   bp_Tdvd_lu_send_rpcstate_type = 0;
   bm_Tdvd_lu_send_rpcstate_vra = $1C;
   bp_Tdvd_lu_send_rpcstate_vra = 2;
   bm_Tdvd_lu_send_rpcstate_ucca = $E0;
   bp_Tdvd_lu_send_rpcstate_ucca = 5;
function get_type(var a : Tdvd_lu_send_rpcstate) : Tu8;
procedure set_type(var a : Tdvd_lu_send_rpcstate; __type : Tu8);
function get_vra(var a : Tdvd_lu_send_rpcstate) : Tu8;
procedure set_vra(var a : Tdvd_lu_send_rpcstate; __vra : Tu8);
function ucca(var a : Tdvd_lu_send_rpcstate) : Tu8;
procedure set_ucca(var a : Tdvd_lu_send_rpcstate; __ucca : Tu8);

type

   Pdvd_authinfo = ^Tdvd_authinfo;
   Tdvd_authinfo = record
       case longint of
          0 : ( _type : Tu8 );
          1 : ( lsa : Tdvd_lu_send_agid );
          2 : ( hsc : Tdvd_host_send_challenge );
          3 : ( lsk : Tdvd_send_key );
          4 : ( lsc : Tdvd_lu_send_challenge );
          5 : ( hsk : Tdvd_send_key );
          6 : ( lstk : Tdvd_lu_send_title_key );
          7 : ( lsasf : Tdvd_lu_send_asf );
          8 : ( hrpcs : Tdvd_host_send_rpcstate );
          9 : ( lrpcs : Tdvd_lu_send_rpcstate );
       end;

const
   bm_Trequest_sense_valid = $1;
   bp_Trequest_sense_valid = 0;
   bm_Trequest_sense_error_code = $FE;
   bp_Trequest_sense_error_code = 1;
   bm_Trequest_sense_reserved1 = $3;
   bp_Trequest_sense_reserved1 = 0;
   bm_Trequest_sense_ili = $4;
   bp_Trequest_sense_ili = 2;
   bm_Trequest_sense_reserved2 = $8;
   bp_Trequest_sense_reserved2 = 3;
   bm_Trequest_sense_sense_key = $F0;
   bp_Trequest_sense_sense_key = 4;
function valid(var a : Trequest_sense) : Tu8;
procedure set_valid(var a : Trequest_sense; __valid : Tu8);
function error_code(var a : Trequest_sense) : Tu8;
procedure set_error_code(var a : Trequest_sense; __error_code : Tu8);
function reserved1(var a : Trequest_sense) : Tu8;
procedure set_reserved1(var a : Trequest_sense; __reserved1 : Tu8);
function ili(var a : Trequest_sense) : Tu8;
procedure set_ili(var a : Trequest_sense; __ili : Tu8);
function reserved2(var a : Trequest_sense) : Tu8;
procedure set_reserved2(var a : Trequest_sense; __reserved2 : Tu8);
function sense_key(var a : Trequest_sense) : Tu8;
procedure set_sense_key(var a : Trequest_sense; __sense_key : Tu8);


{ ---------------------------------------------------------------------
    Utility functions
  ---------------------------------------------------------------------}
<<<<<<< HEAD
<<<<<<< HEAD
type
  TCDSearchRec = record
    Name: String;
    i,j: Integer;
  end;

Function IsCDDevice(Device : String) : Boolean;
Function FindFirstCD(var ACDSearchRec: TCDSearchRec): Boolean;
Function FindNextCD(var ACDSearchRec: TCDSearchRec): Boolean;
=======

Function IsCDDevice(Device : String) : Boolean;
>>>>>>> graemeg/fixes_2_2
=======

Function IsCDDevice(Device : String) : Boolean;
>>>>>>> origin/fixes_2_2
Function DetectCd : String;

implementation

uses major,sysutils;

{ ---------------------------------------------------------------------
    Functions from cdrom.h translation.
  ---------------------------------------------------------------------}

function cdsc_adr(var a : Tcdrom_subchnl) : Tu8;
begin
  cdsc_adr:=(a.flag0 and bm_Tcdrom_subchnl_cdsc_adr) shr bp_Tcdrom_subchnl_cdsc_adr;
end;

procedure set_cdsc_adr(var a : Tcdrom_subchnl; __cdsc_adr : Tu8);
begin
  a.flag0:=a.flag0 or ((__cdsc_adr shl bp_Tcdrom_subchnl_cdsc_adr) and bm_Tcdrom_subchnl_cdsc_adr);
end;

function cdsc_ctrl(var a : Tcdrom_subchnl) : Tu8;
begin
  cdsc_ctrl:=(a.flag0 and bm_Tcdrom_subchnl_cdsc_ctrl) shr bp_Tcdrom_subchnl_cdsc_ctrl;
end;

procedure set_cdsc_ctrl(var a : Tcdrom_subchnl; __cdsc_ctrl : Tu8);
begin
  a.flag0:=a.flag0 or ((__cdsc_ctrl shl bp_Tcdrom_subchnl_cdsc_ctrl) and bm_Tcdrom_subchnl_cdsc_ctrl);
end;

function cdte_adr(var a : Tcdrom_tocentry) : Tu8;
begin
  cdte_adr:=(a.flag0 and bm_Tcdrom_tocentry_cdte_adr) shr bp_Tcdrom_tocentry_cdte_adr;
end;

procedure set_cdte_adr(var a : Tcdrom_tocentry; __cdte_adr : Tu8);
begin
  a.flag0:=a.flag0 or ((__cdte_adr shl bp_Tcdrom_tocentry_cdte_adr) and bm_Tcdrom_tocentry_cdte_adr);
end;

function cdte_ctrl(var a : Tcdrom_tocentry) : Tu8;
begin
  cdte_ctrl:=(a.flag0 and bm_Tcdrom_tocentry_cdte_ctrl) shr bp_Tcdrom_tocentry_cdte_ctrl;
end;

procedure set_cdte_ctrl(var a : Tcdrom_tocentry; __cdte_ctrl : Tu8);
begin
  a.flag0:=a.flag0 or ((__cdte_ctrl shl bp_Tcdrom_tocentry_cdte_ctrl) and bm_Tcdrom_tocentry_cdte_ctrl);
end;

{ was #define dname def_expr }
function CDSL_NONE : longint;
    { return type might be wrong }
    begin
    CDSL_NONE:=(longint(( not (0)) shr 1)) - 1;
    end;

{ was #define dname def_expr }
function CDSL_CURRENT : longint;
    begin
    CDSL_CURRENT:=longint(( not (0)) shr 1);
    end;

function book_version(var a : Tdvd_layer) : Tu8;
begin
  book_version:=(a.flag0 and bm_Tdvd_layer_book_version) shr bp_Tdvd_layer_book_version;
end;

procedure set_book_version(var a : Tdvd_layer; __book_version : Tu8);
begin
  a.flag0:=a.flag0 or ((__book_version shl bp_Tdvd_layer_book_version) and bm_Tdvd_layer_book_version);
end;

function book_type(var a : Tdvd_layer) : Tu8;
begin
  book_type:=(a.flag0 and bm_Tdvd_layer_book_type) shr bp_Tdvd_layer_book_type;
end;

procedure set_book_type(var a : Tdvd_layer; __book_type : Tu8);
begin
  a.flag0:=a.flag0 or ((__book_type shl bp_Tdvd_layer_book_type) and bm_Tdvd_layer_book_type);
end;

function min_rate(var a : Tdvd_layer) : Tu8;
begin
  min_rate:=(a.flag0 and bm_Tdvd_layer_min_rate) shr bp_Tdvd_layer_min_rate;
end;

procedure set_min_rate(var a : Tdvd_layer; __min_rate : Tu8);
begin
  a.flag0:=a.flag0 or ((__min_rate shl bp_Tdvd_layer_min_rate) and bm_Tdvd_layer_min_rate);
end;

function disc_size(var a : Tdvd_layer) : Tu8;
begin
  disc_size:=(a.flag0 and bm_Tdvd_layer_disc_size) shr bp_Tdvd_layer_disc_size;
end;

procedure set_disc_size(var a : Tdvd_layer; __disc_size : Tu8);
begin
  a.flag0:=a.flag0 or ((__disc_size shl bp_Tdvd_layer_disc_size) and bm_Tdvd_layer_disc_size);
end;

function layer_type(var a : Tdvd_layer) : Tu8;
begin
  layer_type:=(a.flag0 and bm_Tdvd_layer_layer_type) shr bp_Tdvd_layer_layer_type;
end;

procedure set_layer_type(var a : Tdvd_layer; __layer_type : Tu8);
begin
  a.flag0:=a.flag0 or ((__layer_type shl bp_Tdvd_layer_layer_type) and bm_Tdvd_layer_layer_type);
end;

function track_path(var a : Tdvd_layer) : Tu8;
begin
  track_path:=(a.flag0 and bm_Tdvd_layer_track_path) shr bp_Tdvd_layer_track_path;
end;

procedure set_track_path(var a : Tdvd_layer; __track_path : Tu8);
begin
  a.flag0:=a.flag0 or ((__track_path shl bp_Tdvd_layer_track_path) and bm_Tdvd_layer_track_path);
end;

function nlayers(var a : Tdvd_layer) : Tu8;
begin
  nlayers:=(a.flag0 and bm_Tdvd_layer_nlayers) shr bp_Tdvd_layer_nlayers;
end;

procedure set_nlayers(var a : Tdvd_layer; __nlayers : Tu8);
begin
  a.flag0:=a.flag0 or ((__nlayers shl bp_Tdvd_layer_nlayers) and bm_Tdvd_layer_nlayers);
end;

function track_density(var a : Tdvd_layer) : Tu8;
begin
  track_density:=(a.flag0 and bm_Tdvd_layer_track_density) shr bp_Tdvd_layer_track_density;
end;

procedure set_track_density(var a : Tdvd_layer; __track_density : Tu8);
begin
  a.flag0:=a.flag0 or ((__track_density shl bp_Tdvd_layer_track_density) and bm_Tdvd_layer_track_density);
end;

function linear_density(var a : Tdvd_layer) : Tu8;
begin
  linear_density:=(a.flag0 and bm_Tdvd_layer_linear_density) shr bp_Tdvd_layer_linear_density;
end;

procedure set_linear_density(var a : Tdvd_layer; __linear_density : Tu8);
begin
  a.flag0:=a.flag0 or ((__linear_density shl bp_Tdvd_layer_linear_density) and bm_Tdvd_layer_linear_density);
end;

function bca(var a : Tdvd_layer) : Tu8;
begin
  bca:=(a.flag0 and bm_Tdvd_layer_bca) shr bp_Tdvd_layer_bca;
end;

procedure set_bca(var a : Tdvd_layer; __bca : Tu8);
begin
  a.flag0:=a.flag0 or ((__bca shl bp_Tdvd_layer_bca) and bm_Tdvd_layer_bca);
end;

function agid(var a : Tdvd_disckey) : dword;
begin
  agid:=(a.flag0 and bm_Tdvd_disckey_agid) shr bp_Tdvd_disckey_agid;
end;

procedure set_agid(var a : Tdvd_disckey; __agid : dword);
begin
  a.flag0:=a.flag0 or ((__agid shl bp_Tdvd_disckey_agid) and bm_Tdvd_disckey_agid);
end;

function agid(var a : Tdvd_lu_send_agid) : dword;
begin
  agid:=(a.flag0 and bm_Tdvd_lu_send_agid_agid) shr bp_Tdvd_lu_send_agid_agid;
end;

procedure set_agid(var a : Tdvd_lu_send_agid; __agid : dword);
begin
  a.flag0:=a.flag0 or ((__agid shl bp_Tdvd_lu_send_agid_agid) and bm_Tdvd_lu_send_agid_agid);
end;

function agid(var a : Tdvd_host_send_challenge) : dword;
begin
  agid:=(a.flag0 and bm_Tdvd_host_send_challenge_agid) shr bp_Tdvd_host_send_challenge_agid;
end;

procedure set_agid(var a : Tdvd_host_send_challenge; __agid : dword);
begin
  a.flag0:=a.flag0 or ((__agid shl bp_Tdvd_host_send_challenge_agid) and bm_Tdvd_host_send_challenge_agid);
end;

function agid(var a : Tdvd_send_key) : dword;
begin
  agid:=(a.flag0 and bm_Tdvd_send_key_agid) shr bp_Tdvd_send_key_agid;
end;

procedure set_agid(var a : Tdvd_send_key; __agid : dword);
begin
  a.flag0:=a.flag0 or ((__agid shl bp_Tdvd_send_key_agid) and bm_Tdvd_send_key_agid);
end;

function agid(var a : Tdvd_lu_send_challenge) : dword;
begin
  agid:=(a.flag0 and bm_Tdvd_lu_send_challenge_agid) shr bp_Tdvd_lu_send_challenge_agid;
end;

procedure set_agid(var a : Tdvd_lu_send_challenge; __agid : dword);
begin
  a.flag0:=a.flag0 or ((__agid shl bp_Tdvd_lu_send_challenge_agid) and bm_Tdvd_lu_send_challenge_agid);
end;

function agid(var a : Tdvd_lu_send_title_key) : dword;
begin
  agid:=(a.flag0 and bm_Tdvd_lu_send_title_key_agid) shr bp_Tdvd_lu_send_title_key_agid;
end;

procedure set_agid(var a : Tdvd_lu_send_title_key; __agid : dword);
begin
  a.flag0:=a.flag0 or ((__agid shl bp_Tdvd_lu_send_title_key_agid) and bm_Tdvd_lu_send_title_key_agid);
end;

function cpm(var a : Tdvd_lu_send_title_key) : dword;
begin
  cpm:=(a.flag1 and bm_Tdvd_lu_send_title_key_cpm) shr bp_Tdvd_lu_send_title_key_cpm;
end;

procedure set_cpm(var a : Tdvd_lu_send_title_key; __cpm : dword);
begin
  a.flag1:=a.flag1 or ((__cpm shl bp_Tdvd_lu_send_title_key_cpm) and bm_Tdvd_lu_send_title_key_cpm);
end;

function cp_sec(var a : Tdvd_lu_send_title_key) : dword;
begin
  cp_sec:=(a.flag1 and bm_Tdvd_lu_send_title_key_cp_sec) shr bp_Tdvd_lu_send_title_key_cp_sec;
end;

procedure set_cp_sec(var a : Tdvd_lu_send_title_key; __cp_sec : dword);
begin
  a.flag1:=a.flag1 or ((__cp_sec shl bp_Tdvd_lu_send_title_key_cp_sec) and bm_Tdvd_lu_send_title_key_cp_sec);
end;

function cgms(var a : Tdvd_lu_send_title_key) : dword;
begin
  cgms:=(a.flag1 and bm_Tdvd_lu_send_title_key_cgms) shr bp_Tdvd_lu_send_title_key_cgms;
end;

procedure set_cgms(var a : Tdvd_lu_send_title_key; __cgms : dword);
begin
  a.flag1:=a.flag1 or ((__cgms shl bp_Tdvd_lu_send_title_key_cgms) and bm_Tdvd_lu_send_title_key_cgms);
end;

function agid(var a : Tdvd_lu_send_asf) : dword;
begin
  agid:=(a.flag0 and bm_Tdvd_lu_send_asf_agid) shr bp_Tdvd_lu_send_asf_agid;
end;

procedure set_agid(var a : Tdvd_lu_send_asf; __agid : dword);
begin
  a.flag0:=a.flag0 or ((__agid shl bp_Tdvd_lu_send_asf_agid) and bm_Tdvd_lu_send_asf_agid);
end;

function asf(var a : Tdvd_lu_send_asf) : dword;
begin
  asf:=(a.flag0 and bm_Tdvd_lu_send_asf_asf) shr bp_Tdvd_lu_send_asf_asf;
end;

procedure set_asf(var a : Tdvd_lu_send_asf; __asf : dword);
begin
  a.flag0:=a.flag0 or ((__asf shl bp_Tdvd_lu_send_asf_asf) and bm_Tdvd_lu_send_asf_asf);
end;

function get_type(var a : Tdvd_lu_send_rpcstate) : Tu8;
begin
  get_type:=(a.flag0 and bm_Tdvd_lu_send_rpcstate_type) shr bp_Tdvd_lu_send_rpcstate_type;
end;

procedure set_type(var a : Tdvd_lu_send_rpcstate; __type : Tu8);
begin
  a.flag0:=a.flag0 or ((__type shl bp_Tdvd_lu_send_rpcstate_type) and bm_Tdvd_lu_send_rpcstate_type);
end;

function get_vra(var a : Tdvd_lu_send_rpcstate) : Tu8;
begin
  get_vra:=(a.flag0 and bm_Tdvd_lu_send_rpcstate_vra) shr bp_Tdvd_lu_send_rpcstate_vra;
end;

procedure set_vra(var a : Tdvd_lu_send_rpcstate; __vra : Tu8);
begin
  a.flag0:=a.flag0 or ((__vra shl bp_Tdvd_lu_send_rpcstate_vra) and bm_Tdvd_lu_send_rpcstate_vra);
end;

function ucca(var a : Tdvd_lu_send_rpcstate) : Tu8;
begin
  ucca:=(a.flag0 and bm_Tdvd_lu_send_rpcstate_ucca) shr bp_Tdvd_lu_send_rpcstate_ucca;
end;

procedure set_ucca(var a : Tdvd_lu_send_rpcstate; __ucca : Tu8);
begin
  a.flag0:=a.flag0 or ((__ucca shl bp_Tdvd_lu_send_rpcstate_ucca) and bm_Tdvd_lu_send_rpcstate_ucca);
end;

function valid(var a : Trequest_sense) : Tu8;
begin
  valid:=(a.flag0 and bm_Trequest_sense_valid) shr bp_Trequest_sense_valid;
end;

procedure set_valid(var a : Trequest_sense; __valid : Tu8);
begin
  a.flag0:=a.flag0 or ((__valid shl bp_Trequest_sense_valid) and bm_Trequest_sense_valid);
end;

function error_code(var a : Trequest_sense) : Tu8;
begin
  error_code:=(a.flag0 and bm_Trequest_sense_error_code) shr bp_Trequest_sense_error_code;
end;

procedure set_error_code(var a : Trequest_sense; __error_code : Tu8);
begin
  a.flag0:=a.flag0 or ((__error_code shl bp_Trequest_sense_error_code) and bm_Trequest_sense_error_code);
end;

function reserved1(var a : Trequest_sense) : Tu8;
begin
  reserved1:=(a.flag1 and bm_Trequest_sense_reserved1) shr bp_Trequest_sense_reserved1;
end;

procedure set_reserved1(var a : Trequest_sense; __reserved1 : Tu8);
begin
  a.flag1:=a.flag1 or ((__reserved1 shl bp_Trequest_sense_reserved1) and bm_Trequest_sense_reserved1);
end;

function ili(var a : Trequest_sense) : Tu8;
begin
  ili:=(a.flag1 and bm_Trequest_sense_ili) shr bp_Trequest_sense_ili;
end;

procedure set_ili(var a : Trequest_sense; __ili : Tu8);
begin
  a.flag1:=a.flag1 or ((__ili shl bp_Trequest_sense_ili) and bm_Trequest_sense_ili);
end;

function reserved2(var a : Trequest_sense) : Tu8;
begin
  reserved2:=(a.flag1 and bm_Trequest_sense_reserved2) shr bp_Trequest_sense_reserved2;
end;

procedure set_reserved2(var a : Trequest_sense; __reserved2 : Tu8);
begin
  a.flag1:=a.flag1 or ((__reserved2 shl bp_Trequest_sense_reserved2) and bm_Trequest_sense_reserved2);
end;

function sense_key(var a : Trequest_sense) : Tu8;
begin
  sense_key:=(a.flag1 and bm_Trequest_sense_sense_key) shr bp_Trequest_sense_sense_key;
end;

procedure set_sense_key(var a : Trequest_sense; __sense_key : Tu8);
begin
  a.flag1:=a.flag1 or ((__sense_key shl bp_Trequest_sense_sense_key) and bm_Trequest_sense_sense_key);
end;

{ ---------------------------------------------------------------------
    Implementation of utility functions.
  ---------------------------------------------------------------------}


Const
  NrDevices = 16;
  Devices : Array[1..NrDevices] of string = (
  '/dev/cdrom',
  '/dev/cdroms/cdrom?',
  '/dev/hd?',
  '/dev/sg?',
  '/dev/cdu31a',
  '/dev/cdu535',
  '/dev/sbpcd',
  '/dev/sbpcd?',
  '/dev/sonycd',
  '/dev/mcd',
  '/dev/sjcd',
  '/dev/cm206cd',
  '/dev/gscd',
  '/dev/scd?',
  '/dev/sr?',
  '/dev/optcd');

<<<<<<< HEAD
<<<<<<< HEAD
function FindFirstCD(var ACDSearchRec: TCDSearchRec): Boolean;
begin
  Result := False;
  With ACDSearchRec do
  begin
    I := 1;
    J := 0;
    Name := '';
  end;
  Result := FindNextCD(ACDSearchRec);
end;

function FindNextCD(var ACDSearchRec: TCDSearchRec): Boolean;
var
  L: integer;
  S: String;
  FoundDev: String;
begin

  Result := False;
  FoundDev := '';
  with ACDSearchRec do
  begin
    While (FoundDev='') and (I<NrDevices) do
    begin
      S:=Devices[i];
      L:=Length(S);
      If S[l]='?' then
      begin
        S:=Copy(S,1,L-1);
        if j >= 3 then
          j := 0;
        while (j <= 3) and (Length(FoundDev)=0) do
        begin
          If IsCdDevice(S+Chr(Ord('0')+J)) then
            FoundDev:=S+Chr(Ord('0')+J)
          else If IsCdDevice(S+Chr(Ord('a')+J)) then
            FoundDev:=S+Chr(Ord('a')+J);
          Inc(j);
        end;
        if J >= 3 then
          Inc(i);
      end
      else
      begin
        If IsCdDevice(S) then
          FoundDev:=S;
        Inc(i);
      end;
    end;
  end;
  Result := Length(FoundDev) > 0;
  ACDSearchRec.Name:=FoundDev;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Function DetectCD : String;

Var
  I,J,L : Integer;
  S : String;

begin
  Result:='';
  I:=0;
  While (Result='') and (I<NrDevices) do
    begin
    Inc(I);
    S:=Devices[i];
    L:=Length(S);
    If S[l]='?' then
      begin
      S:=Copy(S,1,L-1);
      For J:=0 to 3 do
        If IsCdDevice(S+Chr(Ord('0')+J)) then
          Result:=S+Chr(Ord('0')+J)
        else If IsCdDevice(S+Chr(Ord('a')+J)) then
          Result:=S+Chr(Ord('a')+J)
      end
    else
      If IsCdDevice(S) then
        Result:=S;
    end;
end;

Const
  IDEMajor = [IDE0_MAJOR,IDE1_MAJOR,IDE2_MAJOR,IDE3_MAJOR];
  CDMajor  = [CDU31A_CDROM_MAJOR,CDU535_CDROM_MAJOR,
              MATSUSHITA_CDROM_MAJOR,MATSUSHITA_CDROM2_MAJOR,
              MATSUSHITA_CDROM3_MAJOR,MATSUSHITA_CDROM4_MAJOR,
              SANYO_CDROM_MAJOR,
              MITSUMI_CDROM_MAJOR,MITSUMI_X_CDROM_MAJOR,
              OPTICS_CDROM_MAJOR,AZTECH_CDROM_MAJOR,
              GOLDSTAR_CDROM_MAJOR,CM206_CDROM_MAJOR];

Function TestCDRomIOCTL(Device : String) : Boolean;forward;


Function IsCDDevice(Device : String) : Boolean;

Var
  Info : stat;
  S : String;
  DeviceMajor,F : Integer;

begin
{$ifdef debug}
  Writeln('Testing device : ',Device);
{$endif}
  Result:=False;
  If fpstat(device,info)<>0 then
    exit;
  if not (fpS_ISCHR(info.st_mode) or
     fpS_ISBLK(info.st_mode)) then
    exit;
  S:=fpReadLink(Device);
  If (S<>'') then
    Device:=S;
  If fpStat(Device,info)<>0 then
    exit;
  DeviceMajor:=info.st_rdev shr 8;
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef debug}
  Writeln('Device major : ',DeviceMajor);
  {$endif}
  If DeviceMajor in [IDE0_MAJOR,IDE1_MAJOR,IDE2_MAJOR,IDE3_MAJOR] then
    Result:=TestCDRomIOCTL(Device)
  else 
    begin
    Result:= DeviceMajor=SCSI_CDROM_MAJOR;
    If not Result then
      begin
      Result:=DeviceMajor in CDMajor;
      If Not Result then
        begin
        // test SCSI
        end
      else
        begin
        F:=fpOpen(Device,OPEN_RDONLY or OPEN_NONBLOCK);
        Result:=(F>=0);
        If Result then
          fpClose(F);
        end;
      end;
    end;  
=======
=======
>>>>>>> origin/fixes_2_2
  If DeviceMajor in [IDE0_MAJOR,IDE1_MAJOR,IDE2_MAJOR,IDE3_MAJOR] then
      Result:=TestCDRomIOCTL(Device)
  else
    begin
    Result:=DeviceMajor in CDMajor;
    If Not Result then
      begin
      // test SCSI
      end
    else
      begin
      F:=fpOpen(Device,OPEN_RDONLY or OPEN_NONBLOCK);
      Result:=(F>=0);
      If Result then
        fpClose(F);
      end;
    end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Function TestCDRomIOCTL(Device : String) : Boolean;

Var
  F : Integer;
  info : Tcdrom_volctrl;

begin
{$ifdef debug}
  Writeln('Testing for ATAPI');
{$endif}
  Result:=False;
  f:=fpOpen(device,OPEN_RDONLY or OPEN_NONBLOCK);
  If (f<0) then
    exit;
  Result:=(fpIOCtl(f,CDROMVOLREAD,@info)=0);
  fpClose(f);
end;

end.
