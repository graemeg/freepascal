{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 2000-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
    History:

    Added the define use_amiga_smartlink.
    13 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{
  This unit must be deprecated because at least:
  - It is leaking memory. It allocates a new buffer for each string which won't
    be freed until the program exits.
  - The unit doesn't provide any way to free allocated string buffers manually.
    (Because ReleasePas2C is not a public function.)
  - It does allocations outside the Pascal heap, which the compiler has no control
    over, and makes it very hard to track these allocations, because the heaptrc
    unit doesn't work.
  - The intuition.library documentation states that AllocRemember() is a quite
    ineffective function, because it does two memory allocations and because it
    doesn't use memory pools it has a terrible effect on memory fragmentation.
  - It uses a for loop byte to copy the string contents, which is very slow.
  - It uses a global handle without any protection, therefore it's not thread safe.
  - The strings unit provide equivalent functionality, without the leaking problem.
  - Because of the above reasons, this unit will be removed as soon as nothing
    else in the AmUnits package and among the examples depend on it.
    (KB)
}

unit PasToC
  deprecated 'Pas2C function is leaking memory, don''t use it. StrPCopy in strings unit provides equivalent functionality.';

interface

function Pas2C( s : String): PChar;

implementation

const
   MEMF_ANY      = %000000000000000000000000;   { * Any type of memory will do * }
   MEMF_PUBLIC   = %000000000000000000000001;

   MEMF_CLEAR    = %000000010000000000000000;

Type

    ULONG = Longint;

    pRemember = ^tRemember;
    tRemember = record
        NextRemember    : pRemember;
        RememberSize    : ULONG;
        Memory          : Pointer;
    end;

var
    myrememberkey : pRemember;
    remember_exit : pointer;

FUNCTION fpcAllocRemember(VAR rememberKey : pRemember location 'a0'; size : ULONG location 'd0'; flags : ULONG location 'd1') : POINTER; syscall _IntuitionBase 396;
PROCEDURE fpcFreeRemember(VAR rememberKey : pRemember location 'a0'; reallyForget : LONGINT location 'd0'); syscall _IntuitionBase 408;

Function StringPcharCopy(Dest: PChar; Source: String):PChar;
var
   counter : byte;
Begin
   counter := 0;
  { if empty pascal string  }
  { then setup and exit now }
  if Source = '' then
  Begin
    Dest[0] := #0;
    StringPCharCopy := Dest;
    exit;
  end;
  for counter:=1 to length(Source) do
  begin
    Dest[counter-1] := Source[counter];
  end;
  { terminate the string }
  Dest[counter] := #0;
  StringPcharCopy:=Dest;
end;

function Pas2C(s : string): PChar;
var
    themem : Pointer;
begin
    themem := fpcAllocRemember(myrememberkey,length(s)+1, MEMF_CLEAR or MEMF_PUBLIC);
    if themem = nil then begin
        writeln('Can''t allocate memory for string');
        halt(20);
    end else begin
        StringPCharCopy(themem,s);
        Pas2C := themem;
    end;
end;

procedure ReleasePasToC;
begin
    ExitProc := remember_exit;
    fpcFreeRemember(myrememberkey,1);
end;

begin
    myrememberkey := nil;
    remember_exit := ExitProc;
    ExitProc := @ReleasePasToC;
end.
