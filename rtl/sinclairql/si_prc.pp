{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 by Karoly Balogh

    System Entry point for the Sinclair QL

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit si_prc;

interface

implementation

{$i qdosfuncs.inc}

var
  binstart: byte; external name '_stext';
  binend: byte; external name '_etext';
  bssstart: byte; external name '_sbss';
  bssend: byte; external name '_ebss';

procedure PascalMain; external name 'PASCALMAIN';
procedure PascalStart; forward;

{ this function must be the first in this unit which contains code }
{$OPTIMIZATION OFF}
function _FPC_proc_start: longint; cdecl; assembler; nostackframe; public name '_start';
asm
    bra   @start
    dc.l  $0
    dc.w  $4afb
    dc.w  3
    dc.l  $46504300   { Job name, just FPC for now }

@start:
    { relocation code }

    { get our actual position in RAM }
    lea.l binstart(pc),a0
    move.l a0,d0
    { get an offset to the end of the binary. this works both
      relocated and not. The decision if to relocate is done
      later then }
    lea.l binend,a1
    lea.l binstart,a0
    sub.l a0,a1
    add.l d0,a1
    move.l d0,a0

    { read the relocation marker, this is always two padding bytes
      ad the end of .text, so we're free to poke there }
    move.w -2(a1),d7
    beq @noreloc

    { zero out the relocation marker, so if our code is called again
      without reload, it won't relocate itself twice }
    move.w #0,-2(a1)

    { first item in the relocation table is the number of relocs }
    move.l (a1)+,d7
    beq @noreloc

@relocloop:
    { we read the offsets and relocate them }
    move.l (a1)+,d1
    add.l d0,(a0,d1)
    subq.l #1,d7
    bne @relocloop

@noreloc:
    jsr PascalStart
end;

procedure _FPC_proc_halt(_ExitCode: longint); public name '_haltproc';
begin
  mt_frjob(-1, _ExitCode);
end;

procedure PascalStart;
begin
  { initialize .bss }
  FillChar(bssstart,PtrUInt(@bssend)-PtrUInt(@bssstart),#0);

  PascalMain;

  Halt; { this should never be reached }
end;


end.
