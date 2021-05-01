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

{$i qdosh.inc}
{$i qdosfuncs.inc}

var
  binstart: byte; external name '_stext';
  binend: byte; external name '_etext';
  bssstart: byte; external name '_sbss';
  bssend: byte; external name '_ebss';
  stackpointer_on_entry: pointer; public name '__stackpointer_on_entry';

procedure PascalMain; external name 'PASCALMAIN';
procedure PascalStart(a7_on_entry: pointer); noreturn; forward;

{ this function must be the first in this unit which contains code }
procedure _FPC_proc_start; cdecl; assembler; nostackframe; noreturn; public name '_start';
asm
    bra   @start
    dc.l  $0
    dc.w  $4afb
    dc.w  8
    dc.l  $4650435f   { Job name buffer. FPC_PROG by default, can be overridden }
    dc.l  $50524f47   { the startup code will inject the main program name here }
    dc.l  $00000000   { user codes is free to use the SetQLJobName() function   }
    dc.l  $00000000   { max. length: 48 characters }
    dc.l  $00000000
    dc.l  $00000000
    dc.l  $00000000
    dc.l  $00000000
    dc.l  $00000000
    dc.l  $00000000
    dc.l  $00000000
    dc.l  $00000000

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

{$DEFINE PACKEDRELOCS}
{$IFNDEF PACKEDRELOCS}
@relocloop:
    { we read the offsets and relocate them }
    move.l (a1)+,d1
    add.l d0,(a0,d1)
    subq.l #1,d7
    bne @relocloop
{$ELSE PACKEDRELOCS}
    moveq #0,d2
@relocloop:
    { we read the offsets and relocate them }
    moveq #0,d1
    move.b (a1)+,d1
    bne @addoffs
    { if byte = 0, we have a long offset following }
    move.b (a1)+,d1
    lsl.w #8,d1
    move.b (a1)+,d1
    swap d1
    move.b (a1)+,d1
    lsl.w #8,d1
    move.b (a1)+,d1
    subq.l #4,d7
@addoffs:
    add.l d1,d2
    add.l d0,(a0,d2)
    subq.l #1,d7
    bgt @relocloop
{$ENDIF PACKEDRELOCS}

@noreloc:
    move.l a7,a0

    bra PascalStart
end;

procedure _FPC_proc_halt(_ExitCode: longint); noreturn; public name '_haltproc';
begin
  mt_frjob(-1, _ExitCode);
end;

procedure PascalStart(a7_on_entry: pointer); noreturn;
begin
  { initialize .bss }
  FillChar(bssstart,PtrUInt(@bssend)-PtrUInt(@bssstart),#0);

  stackpointer_on_entry:=a7_on_entry;

  PascalMain;
end;


end.
