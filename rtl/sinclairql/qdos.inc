{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 by Karoly Balogh

    Interface QDOS OS functions used by the Sinclair QL RTL

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$i qdosh.inc}

const
  _MT_INF   = $00;
  _MT_ALCHP = $18;
  _MT_RECHP = $19;

function mt_inf(sys_vars: ppchar; ver_ascii: plongint): Tjobid; assembler; nostackframe; public name '_mt_inf';
asm
  move.l  d2,-(sp)
  move.l  sys_vars,-(sp)
  move.l  ver_ascii,-(sp)
  moveq.l #_MT_INF,d0
  trap #1
  move.l  (sp)+,a1
  move.l  d2,(a1)   { ver_ascii }
  move.l  (sp)+,a1
  move.l  a0,(a1)   { sys_vars }
  move.l  (sp)+,d2
  move.l  d1,d0     { jobid }
end;

function mt_alchp(size: dword; sizegot: pdword; jobid: Tjobid): pointer; assembler; nostackframe; public name '_mt_alchp';
asm
  movem.l d2-d3/a2-a3,-(sp)
  move.l sizegot,-(sp)
  move.l jobid,d2
  move.l size,d1
  moveq.l #_MT_ALCHP,d0
  trap #1
  move.l (sp)+,d2 // sizegot ptr
  tst d0
  bne @quit
  move.l d2,a1
  beq @nosizegot
  move.l d1,(a1)
@nosizegot:
  move.l a0,d0
@quit:
  movem.l (sp)+,d2-d3/a2-a3
end;

procedure mt_rechp(area: pointer); assembler; nostackframe; public name '_mt_rechp';
asm
  movem.l d2-d3/a2-a3,-(sp)
  move.l area,a0
  moveq.l #_MT_RECHP,d0
  trap #1
  movem.l (sp)+,d2-d3/a2-a3
end;


const
  _IO_OPEN = $01;
  _IO_CLOSE = $02;

function io_open_qlstr(name_qlstr: pointer; mode: longint): Tchanid; assembler; nostackframe; public name '_io_open_qlstr';
asm
  movem.l d2-d3,-(sp)
  move.l name_qlstr,a0
  moveq.l #-1,d1
  move.l mode,d3
  moveq.l #_IO_OPEN,d0
  trap #2
  bne @quit
  move.l a0,d0
@quit:
  movem.l (sp)+,d2-d3
end;

function io_open(name: pchar; mode: longint): Tchanid; public name '_io_open';
var
  len: longint;
  name_qlstr: array[0..63] of char;
begin
  len:=length(name);
  if len > length(name_qlstr)-2 then
    len:=length(name_qlstr)-2;

  PWord(@name_qlstr)[0]:=len;
  Move(name^,name_qlstr[2],len);

  result:=io_open_qlstr(@name_qlstr,mode);
end;

function io_close(chan: Tchanid): longint; assembler; nostackframe; public name '_io_close';
asm
  move.l chan,a0
  moveq.l #_IO_CLOSE,d0
  trap #2
end;


const
  _IO_SBYTE = $05;
  _IO_SSTRG = $07;

function io_sbyte(chan: Tchanid; timeout: Ttimeout; c: char): longint; assembler; public name '_io_sbyte';
asm
  move.l d3,-(sp)
  move.w timeout,d3
  clr.l d1
  move.b c,d1
  move.l chan,a0
  moveq.l #_IO_SBYTE,d0
  trap #3
  move.l (sp)+,d3
end;

function io_sstrg(chan: Tchanid; timeout: Ttimeout; buf: pointer; len: smallint): smallint; assembler; public name '_io_sstrg';
asm
  movem.l d2-d3,-(sp)
  move.w len,d2
  move.l buf,a1
  move.w timeout,d3
  move.l chan,a0
  moveq.l #_IO_SSTRG,d0
  trap #3
  tst.l d0
  beq @ok
  cmp.w	#ERR_EF,d0
  beq @eof
  cmp.w #ERR_NC,d0
  bne @quit
@eof:
  tst.w d1
  beq @quit
@ok:
  clr.l d0
  move.w d1,d0
@quit:
  movem.l (sp)+,d2-d3
end;


const
  _UT_CON = $c6;
  _UT_SCR = $c8;

function ut_con(params: PConScrParams): Tchanid; assembler; nostackframe; public name '_ut_con';
asm
  movem.l d2-d3/a2-a3,-(sp)
  move.l params,a1
  move.w _UT_CON,a2
  jsr (a2)
  bne @quit
  move.l a0,d0
@quit:
  movem.l (sp)+,d2-d3/a2-a3
end;

function ut_scr(params: PConScrParams): Tchanid; assembler; nostackframe; public name '_ut_scr';
asm
  movem.l d2-d3/a2-a3,-(sp)
  move.l params,a1
  move.w _UT_SCR,a2
  jsr (a2)
  bne @quit
  move.l a0,d0
@quit:
  movem.l (sp)+,d2-d3/a2-a3
end;
