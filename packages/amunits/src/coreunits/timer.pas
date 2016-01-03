{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
    History:
    Removed the var for all functions.
    06 Sep 2000.

    Added the define use_amiga_smartlink.
    13 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se

}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit timer;

INTERFACE

uses exec;

Const

{ unit defintions }
    UNIT_MICROHZ        = 0;
    UNIT_VBLANK         = 1;
    UNIT_ECLOCK         = 2;
    UNIT_WAITUNTIL      = 3;
    UNIT_WAITECLOCK     = 4;

    TIMERNAME : PChar   = 'timer.device';

Type

    ptimeval = ^ttimeval;
    ttimeval = record
        tv_secs         : ULONG;
        tv_micro        : ULONG;
    end;

    ptimerequest = ^ttimerequest;
    ttimerequest = record
        tr_node         : tIORequest;
        tr_time         : ttimeval;
    end;

    pEClockVal = ^tEClockVal;
    tEClockVal = record
        ev_hi : ULONG;
        ev_lo : ULONG;
    end;


Const

{ IO_COMMAND to use for adding a timer }
    TR_ADDREQUEST       = CMD_NONSTD;
    TR_GETSYSTIME       = CMD_NONSTD + 1;
    TR_SETSYSTIME       = CMD_NONSTD + 2;

{  To use any of the routines below, TimerBase must be set to point
   to the timer.device, either by calling CreateTimer or by pulling
   the device pointer from a valid TimeRequest, i.e.

        TimerBase := TimeRequest.io_Device;

    _after_ you have called OpenDevice on the timer.
 }

var
    TimerBase   : Pointer;

Procedure AddTime( Dest, Source : ptimeval);
Function CmpTime( Dest, Source : ptimeval) : ULONG;
Procedure SubTime( Dest, Source : ptimeval);
function ReadEClock(Dest : pEClockVal): longint;
procedure GetSysTime( Dest : ptimeval);

IMPLEMENTATION

Procedure AddTime( Dest, Source : ptimeval);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Dest,a0
       MOVE.L  Source,a1
       MOVE.L  TimerBase,A6
       JSR -042(A6)
       MOVE.L  (A7)+,A6
   end;
end;

Function CmpTime( Dest, Source : ptimeval) : ULONG;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Dest,a0
       MOVE.L  Source,a1
       MOVE.L  TimerBase,A6
       JSR -054(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

Procedure SubTime( Dest, Source : ptimeval);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Dest,a0
       MOVE.L  Source,a1
       MOVE.L  TimerBase,A6
       JSR -048(A6)
       MOVE.L  (A7)+,A6
   end;
end;

function ReadEClock(Dest : pEClockVal): longint;
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Dest,a0
       MOVE.L  TimerBase,A6
       JSR -060(A6)
       MOVE.L  (A7)+,A6
       MOVE.L  d0,@RESULT
   end;
end;

procedure GetSysTime( Dest : ptimeval);
begin
   asm
       MOVE.L  A6,-(A7)
       MOVE.L  Dest,a0
       MOVE.L  TimerBase,A6
       JSR -066(A6)
       MOVE.L  (A7)+,A6
   end;
end;


end.
