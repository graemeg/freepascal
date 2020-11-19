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

unit qdos;


interface

type
  Tchanid = longint;
  Tjobid = longint;
  Ttimeout = smallint;


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


{ the functions declared as external here are implemented in the system unit. They're included
  here via externals, do avoid double implementation of assembler wrappers (KB) }

function mt_inf(sys_vars: ppchar; ver_ascii: plongint): Tjobid; external name '_mt_inf';

procedure mt_dmode(s_mode: pword; d_type: pword); external name '_mt_dmode';

function mt_alchp(size: dword; sizegot: pdword; jobid: Tjobid): pointer; external name '_mt_alchp';
procedure mt_rechp(area: pointer); external name '_mt_rechp';

function io_open_qlstr(name_qlstr: pointer; mode: longint): Tchanid; external name '_io_open_qlstr';
function io_open(name: pchar; mode: longint): Tchanid; external name '_io_open';
function io_close(chan: Tchanid): longint; external name '_io_close';

function io_sbyte(chan: Tchanid; timeout: Ttimeout; c: char): longint; external name '_io_sbyte';
function io_sstrg(chan: Tchanid; timeout: Ttimeout; buf: pointer; len: smallint): smallint; external name '_io_sstrg';

function sd_wdef(chan: Tchanid; timeout: Ttimeout; border_colour: byte; border_width: word; window: PQLRect): longint; external name '_sd_wdef'; 
function sd_clear(chan: Tchanid; timeout: Ttimeout): longint; external name '_sd_clear';

function ut_con(params: PWindowDef): Tchanid; external name '_ut_con';
function ut_scr(params: PWindowDef): Tchanid; external name '_ut_scr';


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
