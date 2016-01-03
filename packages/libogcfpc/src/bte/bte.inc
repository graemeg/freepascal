{$IFDEF OGC_INTERFACE}
const
  ERR_OK = 0;
  ERR_MEM = - 1;
  ERR_BUF = - 2;
  ERR_ABRT = - 3;
  ERR_RST = - 4;
  ERR_CLSD = - 5;
  ERR_CONN = - 6;
  ERR_VAL = - 7;
  ERR_ARG = - 8;
  ERR_RTE = - 9;
  ERR_USE = - 10;
  ERR_IF = - 11;
  ERR_PKTSIZE = - 17;
  HIDP_STATE_READY = $00;
  HIDP_STATE_LISTEN = $01;
  HIDP_STATE_CONNECTING = $02;
  HIDP_STATE_CONNECTED = $04;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  HIDP_CONTROL_CHANNEL = $11;
  HIDP_DATA_CHANNEL = $13;
=======
  HIDP_OUTPUT_CHANNEL = $11;
  HIDP_INPUT_CHANNEL = $13;
>>>>>>> graemeg/cpstrnew
=======
  HIDP_OUTPUT_CHANNEL = $11;
  HIDP_INPUT_CHANNEL = $13;
>>>>>>> graemeg/cpstrnew
=======
  HIDP_OUTPUT_CHANNEL = $11;
  HIDP_INPUT_CHANNEL = $13;
>>>>>>> graemeg/cpstrnew
=======
  HIDP_OUTPUT_CHANNEL = $11;
  HIDP_INPUT_CHANNEL = $13;
>>>>>>> origin/cpstrnew
  HIDP_HDR_TRANS_MASK = $f0;
  HIDP_HDR_PARAM_MASK = $0f;
  HIDP_TRANS_HANDSHAKE = $00;
  HIDP_TRANS_HIDCONTROL = $10;
  HIDP_TRANS_GETREPORT = $40;
  HIDP_TRANS_SETREPORT = $50;
  HIDP_TRANS_GETPROTOCOL = $60;
  HIDP_TRANS_SETPROTOCOL = $70;
  HIDP_TRANS_GETIDLE = $80;
  HIDP_TRANS_SETIDLE = $90;
  HIDP_TRANS_DATA = $a0;
  HIDP_TRANS_DATAC = $b0;
  HIDP_HSHK_SUCCESSFULL = $00;
  HIDP_HSHK_NOTREADY = $01;
  HIDP_HSHK_INV_REPORTID = $02;
  HIDP_HSHK_NOTSUPPORTED = $03;
  HIDP_HSHK_IVALIDPARAM = $04;
  HIDP_HSHK_UNKNOWNERROR = $0e;
  HIDP_HSHK_FATALERROR = $0f;
  HIDP_CTRL_NOP = $00;
  HIDP_CTRL_HARDRESET = $01;
  HIDP_CTRL_SOFTRESET = $02;
  HIDP_CTRL_SUSPEND = $03;
  HIDP_CTRL_RESUME = $04;
  HIDP_CTRL_VC_UNPLUG = $05;
  (* HIDP data transaction headers  *)

  HIDP_DATA_RTYPE_MASK = $03;
  HIDP_DATA_RSRVD_MASK = $0c;
  HIDP_DATA_RTYPE_OTHER = $00;
  HIDP_DATA_RTYPE_INPUT = $01;
  HIDP_DATA_RTYPE_OUPUT = $02;
  HIDP_DATA_RTYPE_FEATURE = $03;
  HIDP_PROTO_BOOT = $00;
  HIDP_PROTO_REPORT = $01;

type
  l2cap_pcb = packed record
  end;
  ctrl_req_t = packed record
  end;

  inquiry_info = packed record
    bdaddr : _bd_addr;
    cod : array [0..2] of cuint8;
  end;
  pinquiry_info = ^inquiry_info;

  inquiry_info_ex = packed record
    bdaddr : _bd_addr;
    cod : array [0..2] of cuint8;
    psrm : cuint8;
    psm : cuint8;
    co : cuint16;
  end;
  pinquiry_info_ex = ^inquiry_info_ex;

  linkkey_info = packed record
    bdaddr : _bd_addr;
    key : array [0..15] of cuint8;
  end;
  plinkkey_info = ^linkkey_info;

  pbte_pcb = ^bte_pcb;
  Trecv = function(arg, buffer: pointer; len: cuint16): cint32; cdecl;
  Tconn_cfm = function(arg: pointer; pcb: Pbte_pcb; err: cuint8): cint32; cdecl;
  Tdisconn_cfm = function(arg: pointer; pcb: Pbte_pcb; err: cuint8): cint32; cdecl;

  Pl2cap_pcb = ^l2cap_pcb;
  Pctrl_req_t = ^ctrl_req_t;
  bte_pcb = packed record
    err : cuint8;
    state : cuint32;
    cbarg : pointer;
    ctrl_req_head : Pctrl_req_t;
    ctrl_req_tail : Pctrl_req_t;
    cmdq : lwpq_t;
    bdaddr : _bd_addr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    ctl_pcb : Pl2cap_pcb;
    data_pcb : Pl2cap_pcb;
=======
    out_pcb : Pl2cap_pcb;
    in_pcb : Pl2cap_pcb;
>>>>>>> graemeg/cpstrnew
=======
    out_pcb : Pl2cap_pcb;
    in_pcb : Pl2cap_pcb;
>>>>>>> graemeg/cpstrnew
=======
    out_pcb : Pl2cap_pcb;
    in_pcb : Pl2cap_pcb;
>>>>>>> graemeg/cpstrnew
=======
    out_pcb : Pl2cap_pcb;
    in_pcb : Pl2cap_pcb;
>>>>>>> origin/cpstrnew
    recv : Trecv;
    conn_cfm : Tconn_cfm;
    disconn_cfm : Tdisconn_cfm;
  end;

type
  btecallback = function(result: cint32; userdata: pointer): cint32; cdecl;


procedure BTE_Init; cdecl; external;

procedure BTE_Shutdown; cdecl; external;

function BTE_InitCore(cb: btecallback): cint32; cdecl; external;

function BTE_ApplyPatch(cb: btecallback): cint32; cdecl; external;

function BTE_InitSub(cb: btecallback): cint32; cdecl; external;

function BTE_ReadStoredLinkKey(keys: Plinkkey_info; max_cnt: cuint8;
  cb: btecallback): cint32; cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function BTE_ReadBdAddr(bdaddr: P_bd_addr; cb: btecallback): cint32; cdecl; external;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
type
  TAuxCCB1 = procedure(bdaddr: P_bd_addr; reason: cuint8); cdecl;
  TAuxCCB0 = procedure(bdaddr: P_bd_addr; reason: cuint8); cdecl;
  function BTE_SetDisconnectCallback(callback: TAuxCCB1): TAuxCCB0; cdecl; external;


function bte_new: Pbte_pcb; cdecl; external;
procedure bte_arg(pcb: Pbte_pcb; arg: pointer); cdecl; external;

procedure bte_received(pcb: Pbte_pcb; recv: Trecv); cdecl; external;

procedure bte_disconnected(pcb: Pbte_pcb; disconn_cfm: cint32); cdecl; external;

function bte_registerdeviceasync(pcb: Pbte_pcb; bdaddr: P_bd_addr; conn_cfm: Tconn_cfm): cint32; cdecl; external;


function bte_disconnect(pcb: Pbte_pcb): cint32; cdecl; external;


//s32 bte_listen(struct bte_pcb *pcb,struct _bd_addr *bdaddr,u8 psm);
//s32 bte_accept(struct bte_pcb *pcb,s32 (*recv)(void *arg,void *buffer,u16 len));
function bte_inquiry(info: Pinquiry_info; max_cnt, flush: cuint8): cint32; cdecl; external;
function bte_inquiry_ex(info: Pinquiry_info_ex; max_cnt, flush: cuint8): cint32; cdecl; external;

//s32 bte_connect(struct bte_pcb *pcb,struct _bd_addr *bdaddr,u8 psm,s32 (*recv)(void *arg,void *buffer,u16 len));
//s32 bte_connect_ex(struct bte_pcb *pcb,struct inquiry_info_ex *info,u8 psm,s32 (*recv)(void *arg,void *buffer,u16 len));
function bte_senddata(pcb: Pbte_pcb; messg: pointer; len: cuint16): cint32; cdecl; external;

function bte_sendmessage(pcb: Pbte_pcb; messg: pointer; len: cuint16): cint32; cdecl; external;

type
  Tsent_cb = function(arg: pointer; pcb: Pbte_pcb; err: cuint8): cint32; cdecl;
function bte_sendmessageasync(pcb: Pbte_pcb; messg: pointer; len: cuint16;
  sent: Tsent_cb): cint32; cdecl; external;
{$ENDIF}



