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

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit PasToC;


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

FUNCTION fpcAllocRemember(VAR rememberKey : pRemember; size : ULONG; flags : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rememberKey,A0
    MOVE.L  size,D0
    MOVE.L  flags,D1
    MOVEA.L _IntuitionBase,A6
    JSR -396(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE fpcFreeRemember(VAR rememberKey : pRemember; reallyForget : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L rememberKey,A0
    MOVE.L  reallyForget,D0
    MOVEA.L _IntuitionBase,A6
    JSR -408(A6)
    MOVEA.L (A7)+,A6
  END;
END;

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
