{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2021 by Karoly Balogh

    Interface to SMS only OS functions used by the Sinclair QL RTL

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
    _IOF_MKDIR = $4d;

function iof_mkdir(chan: Tchanid): longint; assembler; nostackframe; public name '_iof_mkdir';
asm
  move.l d3,-(sp)
  move.l chan,a0
  moveq.l #-1,d3
  moveq.l #0,d1
  moveq.l #_IOF_MKDIR,d0
  trap #3
  move.l (sp)+,d3
end;
