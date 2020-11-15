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
  stacktop: pointer;
  setjmpbuf: jmp_buf;
  stklen: longint; external name '__stklen';
  binstart: byte; external name '_stext';
  binend: byte; external name '_etext';
  bssstart: byte; external name '_sbss';
  bssend: byte; external name '_ebss';

{ this is const, so it will go into the .data section, not .bss }
const
  stackorig: pointer = nil;

procedure PascalMain; external name 'PASCALMAIN';

{ this function must be the first in this unit which contains code }
{$OPTIMIZATION OFF}
function _FPC_proc_start: longint; cdecl; public name '_start';
var
  newstack: pointer;
begin
  _FPC_proc_start:=0;
  asm
    move.l d7,-(sp)
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
    move.l (sp)+,d7

    { save the original stack pointer }
    move.l a7,stackorig
  end;

  { initialize .bss }
  FillChar(bssstart,PtrUInt(@bssend)-PtrUInt(@bssstart),#0);

  newstack:=mt_alchp(stklen,nil,-1);
  if not assigned(newstack) then
    _FPC_proc_start:=ERR_OM
  else
    begin
      stacktop:=pbyte(newstack)+stklen;
      asm
        move.l stacktop,sp
      end;
      if setjmp(setjmpbuf) = 0 then
        PascalMain;
      asm
        move.l stackorig,sp
      end;
      mt_rechp(newstack);
   end;
end;

procedure _FPC_proc_halt(_ExitCode: longint); public name '_haltproc';
begin
  longjmp(setjmpbuf,1); 
end;


end.
