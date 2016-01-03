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

    Added functions and procedures with array of const.
    For use with fpc 1.0.7. They are in systemvartags.
    11 Nov 2002.

    Added the defines use_amiga_smartlink and
    use_auto_openlib. Implemented autoopening of the
    library.
    14 Jan 2003.

    Changed integer > smallint,
            cardinal > longword.
    Changed startcode for unit.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT realtime;

INTERFACE
USES exec, utility;

{***************************************************************************}

const
{ realtime.library's idea of time is based on a clock which emits a pulse
 * 1200 times a second (1.2kHz). All time values maintained by realtime.library
 * are based on this number. For example, the field RealTimeBase->rtb_Time
 * expresses an amount of time equivalent to (RealTimeBase->rtb_Time/TICK_FREQ)
 * seconds.
 }
 TICK_FREQ = 1200;


{***************************************************************************}


{ Each Conductor represents a group of applications which wish to remain
 * synchronized together.
 *
 * This structure must only be allocated by realtime.library and is
 * READ-ONLY!
 }
Type
 pConductor = ^tConductor;
 tConductor = record
    cdt_Link        : tNode;
    cdt_Reserved0   : WORD;
    cdt_Players     : tMinList;          { this conductor's players      }
    cdt_ClockTime,                       { current time of this sequence }
    cdt_StartTime,                       { start time of this sequence   }
    cdt_ExternalTime,                    { time from external unit       }
    cdt_MaxExternalTime,                 { upper limit on sync'd time    }
    cdt_Metronome   : ULONG;             { MetricTime highest pri node   }
    cdt_Reserved1   : WORD;
    cdt_Flags       : WORD;              { conductor flags               }
    cdt_State       : Byte;              { playing or stopped            }
 end;

const
{ Flag bits for Conductor.cdt_Flags }
 CONDUCTF_EXTERNAL = 1;   { clock is externally driven }
 CONDUCTF_GOTTICK  = 2;   { received 1st external tick }
 CONDUCTF_METROSET = 4;   { cdt_Metronome filled in    }
 CONDUCTF_PRIVATE  = 8;   { conductor is private       }

 CONDUCTB_EXTERNAL = 0;
 CONDUCTB_GOTTICK  = 1;
 CONDUCTB_METROSET = 2;
 CONDUCTB_PRIVATE  = 3;

{ constants for Conductor.cdt_State and SetConductorState() }
 CONDSTATE_STOPPED    = 0;   { clock is stopped              }
 CONDSTATE_PAUSED     = 1;   { clock is paused               }
 CONDSTATE_LOCATE     = 2;   { go to 'running' when ready    }
 CONDSTATE_RUNNING    = 3;   { run clock NOW                 }

{ These do not actually exist as Conductor states, but are used as additional
 * arguments to SetConductorState()
 }
 CONDSTATE_METRIC     = -1;   { ask high node to locate       }
 CONDSTATE_SHUTTLE    = -2;   { time changing but not running }
 CONDSTATE_LOCATE_SET = -3;   { maestro done locating         }


{***************************************************************************}


{ The Player is the connection between a Conductor and an application.
 *
 * This structure must only be allocated by realtime.library and is
 * READ-ONLY!
 }
Type
 pPlayer = ^tPlayer;
 tPlayer = record
    pl_Link             : tNode;
    pl_Reserved0,
    pl_Reserved1        : Shortint;
    pl_Hook             : pHook;         { player's hook function       }
    pl_Source           : pConductor;    { pointer to parent context    }
    pl_Task             : pTask;         { task to signal for alarm     }
    pl_MetricTime       : Longint;       { current time in app's metric }
    pl_AlarmTime        : Longint;       { time to wake up              }
    pl_UserData         : Pointer;       { for application use  }
    pl_PlayerID         : WORD;          { for application use  }
    pl_Flags            : WORD;          { general Player flags         }
 end;

const
{ Flag bits for Player.pl_Flags }
 PLAYERF_READY     = 1;   { player is ready to go!        }
 PLAYERF_ALARMSET  = 2;   { alarm is set                  }
 PLAYERF_QUIET     = 3;   { a dummy player, used for sync }
 PLAYERF_CONDUCTED = 8;   { give me metered time          }
 PLAYERF_EXTSYNC   = 16;   { granted external sync         }

 PLAYERB_READY     = 0;
 PLAYERB_ALARMSET  = 1;
 PLAYERB_QUIET     = 2;
 PLAYERB_CONDUCTED = 3;
 PLAYERB_EXTSYNC   = 4;


{***************************************************************************}


{ Tags for CreatePlayer(), SetPlayerAttrs(), and GetPlayerAttrs() }
 PLAYER_Base         = (TAG_USER+64)   ;
 PLAYER_Hook         = (PLAYER_Base+1) ;  { set address of hook function }
 PLAYER_Name         = (PLAYER_Base+2) ;  { name of player       }
 PLAYER_Priority     = (PLAYER_Base+3) ;  { priority of player           }
 PLAYER_Conductor    = (PLAYER_Base+4) ;  { set conductor for player     }
 PLAYER_Ready        = (PLAYER_Base+5) ;  { the "ready" flag             }
 PLAYER_AlarmTime    = (PLAYER_Base+12);  { alarm time (sets PLAYERF_ALARMSET) }
 PLAYER_Alarm        = (PLAYER_Base+13);  { sets/clears PLAYERF_ALARMSET flag  }
 PLAYER_AlarmSigTask = (PLAYER_Base+6) ;  { task to signal for alarm/notify    }
 PLAYER_AlarmSigBit  = (PLAYER_Base+8) ;  { signal bit for alarm (or -1) }
 PLAYER_Conducted    = (PLAYER_Base+7) ;  { sets/clears PLAYERF_CONDUCTED flag   }
 PLAYER_Quiet        = (PLAYER_Base+9) ;  { don't process time thru this }
 PLAYER_UserData     = (PLAYER_Base+10);
 PLAYER_ID           = (PLAYER_Base+11);
 PLAYER_ExtSync      = (PLAYER_Base+14);  { attempt/release to ext sync  }
 PLAYER_ErrorCode    = (PLAYER_Base+15);  { error return value           }


{***************************************************************************}


{ Method types for messages sent via a Player's hook }
 PM_TICK     = 0;
 PM_STATE    = 1;
 PM_POSITION = 2;
 PM_SHUTTLE  = 3;

Type
{ used for PM_TICK, PM_POSITION and PM_SHUTTLE methods }
 ppmTime = ^tpmTime;
 tpmTime = record
    pmt_Method  : ULONG;        { PM_TICK, PM_POSITION, or PM_SHUTTLE }
    pmt_Time    : ULONG;
 end;

{ used for the PM_STATE method }
 ppmState = ^tpmState;
 tpmState = record
    pms_Method  : ULONG;        { PM_STATE }
    pms_OldState: ULONG;
 end;


{***************************************************************************}

const
{ Possible lock types for LockRealTime() }
 RT_CONDUCTORS = 0;   { conductor list }


{***************************************************************************}


{ realtime.library error codes }
 RTE_NOMEMORY    = 801;   { memory allocation failed      }
 RTE_NOCONDUCTOR = 802;   { player needs a conductor      }
 RTE_NOTIMER     = 803;   { timer (CIA) allocation failed }
 RTE_PLAYING     = 804;   { can't shuttle while playing   }


{***************************************************************************}


{ OpenLibrary("realtime.library",0) returns a pointer to this structure.
 * All fields are READ-ONLY.
 }
Type
 pRealTimeBase = ^tRealTimeBase;
 tRealTimeBase = record
    rtb_LibNode     : tLibrary;
    rtb_Reserved0   : Array[0..1] of Byte;

    rtb_Time,                      { current time                         }
    rtb_TimeFrac    : ULONG;       { fixed-point fraction part of time    }
    rtb_Reserved1   : WORD;
    rtb_TickErr     : smallint;     { nanosecond error from ideal Tick     }
 end;                              { length to real tick length           }

{ Actual tick length is: 1/TICK_FREQ + rtb_TickErr/1e9 }

const
 RealTime_TickErr_Min = -705;
 RealTime_TickErr_Max =  705;

{*--- functions in V37 or higher (Release 2.04) ---*}

VAR RealTimeBase : pRealTimeBase;

const
    REALTIMENAME : PChar = 'realtime.library';

FUNCTION CreatePlayerA(const tagList : pTagItem) : pPlayer;
PROCEDURE DeletePlayer(player : pPlayer);
FUNCTION ExternalSync(player : pPlayer; minTime : LONGINT; maxTime : LONGINT) : BOOLEAN;
FUNCTION FindConductor(const name : pCHAR) : pConductor;
FUNCTION GetPlayerAttrsA(const player : pPlayer;const tagList : pTagItem) : ULONG;
FUNCTION LockRealTime(lockType : ULONG) : POINTER;
FUNCTION NextConductor(const previousConductor : pConductor) : pConductor;
FUNCTION SetConductorState(player : pPlayer; state : ULONG; time : LONGINT) : LONGINT;
FUNCTION SetPlayerAttrsA(player : pPlayer;const tagList : pTagItem) : BOOLEAN;
PROCEDURE UnlockRealTime(lock : POINTER);

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitREALTIMELibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    REALTIMEIsCompiledHow : longint;

IMPLEMENTATION

{$ifndef dont_use_openlib}
uses msgbox;
{$endif dont_use_openlib}

FUNCTION CreatePlayerA(const tagList : pTagItem) : pPlayer;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L tagList,A0
    MOVEA.L RealTimeBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE DeletePlayer(player : pPlayer);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L player,A0
    MOVEA.L RealTimeBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION ExternalSync(player : pPlayer; minTime : LONGINT; maxTime : LONGINT) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L player,A0
    MOVE.L  minTime,D0
    MOVE.L  maxTime,D1
    MOVEA.L RealTimeBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION FindConductor(const name : pCHAR) : pConductor;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L name,A0
    MOVEA.L RealTimeBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetPlayerAttrsA(const player : pPlayer;const tagList : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L player,A0
    MOVEA.L tagList,A1
    MOVEA.L RealTimeBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION LockRealTime(lockType : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  lockType,D0
    MOVEA.L RealTimeBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION NextConductor(const previousConductor : pConductor) : pConductor;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L previousConductor,A0
    MOVEA.L RealTimeBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetConductorState(player : pPlayer; state : ULONG; time : LONGINT) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L player,A0
    MOVE.L  state,D0
    MOVE.L  time,D1
    MOVEA.L RealTimeBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION SetPlayerAttrsA(player : pPlayer;const tagList : pTagItem) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L player,A0
    MOVEA.L tagList,A1
    MOVEA.L RealTimeBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE UnlockRealTime(lock : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L lock,A0
    MOVEA.L RealTimeBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
  END;
END;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of realtime.library}
  {$Info don't forget to use InitREALTIMELibrary in the beginning of your program}

var
    realtime_exit : Pointer;

procedure CloserealtimeLibrary;
begin
    ExitProc := realtime_exit;
    if RealTimeBase <> nil then begin
        CloseLibrary(pLibrary(RealTimeBase));
        RealTimeBase := nil;
    end;
end;

procedure InitREALTIMELibrary;
begin
    RealTimeBase := nil;
    RealTimeBase := pRealTimeBase(OpenLibrary(REALTIMENAME,LIBVERSION));
    if RealTimeBase <> nil then begin
        realtime_exit := ExitProc;
        ExitProc := @CloserealtimeLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open realtime.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    REALTIMEIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of realtime.library}

var
    realtime_exit : Pointer;

procedure CloserealtimeLibrary;
begin
    ExitProc := realtime_exit;
    if RealTimeBase <> nil then begin
        CloseLibrary(pLibrary(RealTimeBase));
        RealTimeBase := nil;
    end;
end;

begin
    RealTimeBase := nil;
    RealTimeBase := pRealTimeBase(OpenLibrary(REALTIMENAME,LIBVERSION));
    if RealTimeBase <> nil then begin
        realtime_exit := ExitProc;
        ExitProc := @CloserealtimeLibrary;
        REALTIMEIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open realtime.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    REALTIMEIsCompiledHow := 3;
   {$Warning No autoopening of realtime.library compiled}
   {$Warning Make sure you open realtime.library yourself}
{$endif dont_use_openlib}


END. (* UNIT REALTIME *)



