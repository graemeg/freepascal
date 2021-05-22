{
    This file is part of the Free Pascal Sinclair QL support package.
    Copyright (c) 2020 by Karoly Balogh

    Interface QDOS OS functions for applications

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$PACKRECORDS 2}

unit qdos;


interface

type
  Tchanid = longint;
  Tjobid = longint;
  Ttimeout = smallint;
  Tcolour = byte;

type
  Pqlstr = ^Tqlstr;
  Tqlstr = record
    qs_strlen: word;
    qs_str: array[0..0] of char;
  end;

const
  ERR_NC = -1;   { Operation not complete }
  ERR_NJ = -2;   { Not a (valid) job. }
  ERR_OM = -3;   { Out of memory. }
  ERR_OR = -4;   { Out of range. }
  ERR_BO = -5;   { Buffer overflow. }
  ERR_NO = -6;   { Channel not open. }
  ERR_NF = -7;   { File or device not found. }
  ERR_FX = -8;   { File already exists. }
  ERR_IU = -9;   { File or device already in use. }
  ERR_EF = -10;  { End of file. }
  ERR_DF = -11;  { Drive full. }
  ERR_BN = -12;  { Bad device. }
  ERR_TE = -13;  { Transmission error. }
  ERR_FF = -14;  { Format failed. }
  ERR_BP = -15;  { Bad parameter. }
  ERR_FE = -16;  { File error. }
  ERR_EX = -17;  { Expression error. }
  ERR_OV = -18;  { Arithmetic overflow. }
  ERR_NI = -19;  { Not implemented. }
  ERR_RO = -20;  { Read only. }
  ERR_BL = -21;  { Bad line of Basic. }

const
  Q_OPEN = 0;
  Q_OPEN_IN = 1;
  Q_OPEN_NEW = 2;
  Q_OPEN_OVER = 3;  { Not available on microdrives. }
  Q_OPEN_DIR = 4;

{ sysvars offsets }
const
   SV_IDENT = $00;
   SV_CHEAP = $04;
   SV_CHPFR = $08;
   SV_FREE = $0c;
   SV_BASIC = $10;
   SV_TRNSP = $14;
   SV_TRNFR = $18;
   SV_RESPR = $1c;
   SV_RAMT = $20;
   SV_RAND = $2e;
   SV_POLLM = $30;
   SV_TVMOD = $32;
   SV_SCRST = $33;
   SV_MCSTA = $34;
   SV_PCINT = $35;
   SV_NETNR = $37;
   SV_I2LST = $38;
   SV_PLIST = $3c;
   SV_SHLST = $40;
   SV_DRLST = $44;
   SV_DDLST = $48;
   SV_KEYQ = $4c;
   SV_TRAPV = $50;
   SV_BTPNT = $54;
   SV_BTBAS = $58;
   SV_BTTOP = $5c;
   SV_JBTAG = $60;
   SV_JBMAX = $62;
   SV_JBPNT = $64;
   SV_JBBAS = $68;
   SV_JBTOP = $6c;
   SV_CHTAG = $70;
   SV_CHMAX = $72;
   SV_CHPNT = $74;
   SV_CHBAS = $78;
   SV_CHTOP = $7c;
   SV_CAPS = $88;
   SV_ARBUF = $8a;
   SV_ARDEL = $8c;
   SV_ARFRQ = $8e;
   SV_ARCNT = $90;
   SV_CQCH = $92;
   SV_SOUND = $96;
   SV_SER1C = $98;
   SV_SER2C = $9c;
   SV_TMODE = $a0;
   SV_PTYP = $a1;
   SV_CSUB = $a2;
   SV_TIMO = $a6;
   SV_TIMOV = $a8;
   SV_FSTAT = $aa;
   SV_MDRUN = $ee;
   SV_MDCNT = $ef;
   SV_MDDID = $f0;
   SV_MDSTA = $f8;
   SV_FSDEF = $100;
   SV_FSLST = $140;
   SV_TOP = $180;


type
  Tqlfloat = array[0..5] of byte;
  Pqlfloat = ^Tqlfloat;

type
  TQLRect = record
    q_width : word;
    q_height : word;
    q_x : word;
    q_y : word;
  end;
  PQLRect = ^TQLRect;

type
  TWindowDef = record
    border_colour : byte;
    border_width : byte;
    paper : byte;
    ink : byte;
    width : word;
    height : word;
    x_origin: word;
    y_origin: word;
  end;
  PWindowDef = ^TWindowDef;

type
  Pqdos_queue = ^Tqdos_queue;
  Tqdos_queue = record
    q_nextq: Pqdos_queue;
    q_end: pchar;
    q_nextin: pchar;
    q_nxtout: pchar;
    q_queue: array[0..1] of char;
  end;

const
  QDOSQUEUE_SIZE = $12;

type
  Tchan_defb = record
    ch_len: dword;
    ch_drivr: pbyte;
    ch_owner: Tjobid;
    ch_rflag: pbyte;
    ch_tag: word;
    ch_stat: byte;
    ch_actn: byte;
    ch_jobwt: Tjobid;
  end;

const
  CHAN_DEFBSIZE = $18;

type
  Pser_cdefb = ^Tser_cdefb;
  Tser_cdefb = record
    ser_cdef: Tchan_defb;
    ser_chnq: word;
    ser_par: word;
    ser_thsx: word;
    ser_prot: word;
    ser_rxq: Tqdos_queue;
    ser_dum1: array[0..79] of byte;
    ser_txq: Tqdos_queue;
    ser_dum2: array[0..79] of byte;
  end;

const
  SER_CDEFBSIZE = $E4;

type
  Tnet_cdefb = record
    net_cdef: Tchan_defb;
    net_hedr: byte;
    net_self: byte;
    net_blkl: byte;
    net_blkh: byte;
    net_type: byte;
    net_nbyt: byte;
    net_dchk: byte;
    net_hchk: byte;
    net_data: array[0..254] of byte;
    net_rpnt: byte;
  end;

const
  NET_CDEFBSIZE = $120;

type
  Tpipe_cdefb = record
    ch_cdef: Tchan_defb;
    ch_qin: Pqdos_queue;
    ch_qout: Pqdos_queue;
  end;

const
  PIPE_CDEFBSIZE = $20;

type
  Tscrn_info = record
    sd_xmin: word;
    sd_ymin: word;
    sd_xsize: word;
    sd_ysize: word;
    sd_borwd: word;
    sd_xpos: word;
    sd_ypos: word;
    sd_xinc: word;
    sd_yinc: word;
    sd_font: array[0..1] of pointer;
    sd_scrb: pointer;
    sd_pmask: dword;
    sd_smask: dword;
    sd_imask: dword;
    sd_cattr: byte;
    sd_curf: byte;
    sd_pcolr: Tcolour;
    sd_scolr: Tcolour;
    sd_icolr: Tcolour;
    sd_bcolr: Tcolour;
    sd_nlsta: byte;
    sd_fmod: byte;
    sd_xorg: Tqlfloat;
    sd_yorg: Tqlfloat;
    sd_scal: Tqlfloat;
    sd_fbuf: pointer;
    sd_fuse: pointer;
    sd_linel: word;
  end;

const
  SCRN_INFOSIZE = $4E;

type
  Pscr_cdefb = ^Tscr_cdefb;
  Tscr_cdefb = record
    scr_cdef: Tchan_defb;
    scr_info: Tscrn_info;
  end;

const
  SCR_CDEFBSIZE = CHAN_DEFBSIZE + SCRN_INFOSIZE;

const
  CA_UNDERLINE = $1;
  CA_FLASH = $2;
  CA_TRANS = $4;
  CA_XOR = $8;
  CA_DOUBLE_HEIGHT = $10;
  CA_EXT_WIDTH = $20;
  CA_DBLE_WIDTH = $40;
  CA_GRAF_POS_CHAR = $80;

type
  Tcon_union1 = record
    sdu_linel: longint;
    sdu_kbd: Tqdos_queue;
  end;

  Pcon_cdefb = ^Tcon_cdefb;
  Tcon_cdefb = record
    con_cdef: Tchan_defb;
    con_info: Tscrn_info;
    case boolean of
      false:  ( sd_js: Tcon_union1 );
      true:   ( sd_jm: Tqdos_queue );
  end;

const
  CON_CDEFBSIZE = SCR_CDEFBSIZE + QDOSQUEUE_SIZE + 4;

type
  Pfs_cdefb = ^Tfs_cdefb;
  Tfs_cdefb = record
    fs_cdef: Tchan_defb;
    fs_next: Pfs_cdefb;
    fs_access: byte;
    fs_drive: byte;
    fs_filnr: word;
    fs_nblok: word;
    fs_nbyte: word;
    fs_eblok: word;
    fs_ebyte: word;
    fs_cblock: pointer;
    fs_updt: byte;
    fs_res1: shortint;
    fs_res2: longint;
    fs_name: Tqlstr;
    fs_pad: array[0..105] of byte;
  end;

const
  FS_CDEFBSIZE = $a0;
  FSCDEF_SIZE = FS_CDEFBSIZE; { inconsistently named alias, from C code }


{ Variable/type includes before function declarations }
{$i qdos_sysvars.inc}

{ the functions declared in qdosfuncs.inc are implemented in the system unit. They're included
  here via externals, do avoid double implementation of assembler wrappers. for this reason,
  qdosfuncs.inc in packages/qlunits must be kept identical to the one in rtl/sinclairql (KB). }

{$i qdosfuncs.inc}

{ other functions, not used/implemented by the RTL }

procedure sd_point(chan: Tchanid; timeout: Ttimeout; x: Pqlfloat; y: Pqlfloat);
procedure sd_point(chan: Tchanid; timeout: Ttimeout; x: double; y: double);

procedure sd_line(chan: Tchanid; timeout: Ttimeout; x_start: Pqlfloat; y_start: Pqlfloat; x_end: Pqlfloat; y_end: Pqlfloat);
procedure sd_line(chan: Tchanid; timeout: Ttimeout; x_start: double; y_start: double; x_end: double; y_end: double);


implementation

uses
  qlfloat;

const
  _SD_POINT = $30;
  _SD_LINE = $31;

procedure sd_point(chan: Tchanid; timeout: Ttimeout; x: Pqlfloat; y: Pqlfloat);
var
  stack: array[0..1] of TQLFloat;
begin
  stack[1]:=x^;
  stack[0]:=y^;
  asm
    move.l d3,-(sp)
    move.w timeout,d3
    move.l chan,a0
    lea.l stack,a1
    moveq.l #_SD_POINT,d0
    trap #3
    move.l (sp)+,d3
  end;
end;

procedure sd_point(chan: Tchanid; timeout: Ttimeout; x: double; y: double);
var
  stack: array[0..1] of TQLFloat;
begin
  double_to_qlfp(@stack[1],@x);
  double_to_qlfp(@stack[0],@y);
  asm
    move.l d3,-(sp)
    move.w timeout,d3
    move.l chan,a0
    lea.l stack,a1
    moveq.l #_SD_POINT,d0
    trap #3
    move.l (sp)+,d3
  end;
end;


procedure sd_line(chan: Tchanid; timeout: Ttimeout; x_start: Pqlfloat; y_start: Pqlfloat; x_end: Pqlfloat; y_end: Pqlfloat);
var
  stack: array[0..3] of TQLFloat;
begin
  stack[3]:=x_start^;
  stack[2]:=y_start^;
  stack[1]:=x_end^;
  stack[0]:=y_end^;
  asm
    move.l d3,-(sp)
    move.w timeout,d3
    move.l chan,a0
    lea.l stack,a1
    moveq.l #_SD_LINE,d0
    trap #3
    move.l (sp)+,d3
  end;
end;

procedure sd_line(chan: Tchanid; timeout: Ttimeout; x_start: double; y_start: double; x_end: double; y_end: double);
var
  stack: array[0..3] of TQLFloat;
begin
  double_to_qlfp(@stack[3],@x_start);
  double_to_qlfp(@stack[2],@y_start);
  double_to_qlfp(@stack[1],@x_end);
  double_to_qlfp(@stack[0],@y_end);
  asm
    move.l d3,-(sp)
    move.w timeout,d3
    move.l chan,a0
    lea.l stack,a1
    moveq.l #_SD_LINE,d0
    trap #3
    move.l (sp)+,d3
  end;
end;


end.
