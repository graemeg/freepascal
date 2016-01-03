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
    For use with fpc 1.0.7. Thay are in systemvartags.
    11 Nov 2002.

    Added the defines use_amiga_smartlink and
    use_auto_openlib.
    13 Jan 2003.

    Update for AmigaOS 3.9.
    Changed startcode for unit.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

UNIT lowlevel;

INTERFACE
USES exec, utility, timer;

Type
{ structure for use with QueryKeys() }
 pKeyQuery = ^tKeyQuery;
 tKeyQuery = record
    kq_KeyCode  : WORD;
    kq_Pressed  : Boolean;
 end;

{***************************************************************************}

Const

 LOWLEVELNAME : PChar = 'lowlevel.library';

{ bits in the return value of GetKey() }
 LLKB_LSHIFT     = 16;
 LLKB_RSHIFT     = 17;
 LLKB_CAPSLOCK   = 18;
 LLKB_CONTROL    = 19;
 LLKB_LALT       = 20;
 LLKB_RALT       = 21;
 LLKB_LAMIGA     = 22;
 LLKB_RAMIGA     = 23;

 LLKF_LSHIFT     = 65536;
 LLKF_RSHIFT     = 131072;
 LLKF_CAPSLOCK   = 262144;
 LLKF_CONTROL    = 524288;
 LLKF_LALT       = 1048576;
 LLKF_RALT       = 2097152;
 LLKF_LAMIGA     = 4194304;
 LLKF_RAMIGA     = 8388608;

{***************************************************************************}


{ Tags for SetJoyPortAttrs() }
 SJA_Dummy        = (TAG_USER+$c00100);
 SJA_Type         = (SJA_Dummy+1); { force type to mouse, joy, game cntrlr }
 SJA_Reinitialize = (SJA_Dummy+2); { free potgo bits, reset to autosense   }

{ Controller types for SJA_Type tag }
 SJA_TYPE_AUTOSENSE = 0;
 SJA_TYPE_GAMECTLR  = 1;
 SJA_TYPE_MOUSE     = 2;
 SJA_TYPE_JOYSTK    = 3;


{***************************************************************************}


{ ReadJoyPort() return value definitions }

{ Port types }
 JP_TYPE_NOTAVAIL  = 0;          { port data unavailable    }
 JP_TYPE_GAMECTLR  = 268435456;  { port has game controller }
 JP_TYPE_MOUSE     = 536870912;  { port has mouse           }
 JP_TYPE_JOYSTK    = 805306368;  { port has joystick        }
 JP_TYPE_UNKNOWN   = 1073741824; { port has unknown device  }
 JP_TYPE_MASK      = -268435456; { controller type          }

{ Button types, valid for all types except JP_TYPE_NOTAVAIL }
 JPB_BUTTON_BLUE    = 23;     { Blue - Stop; Right Mouse                }
 JPB_BUTTON_RED     = 22;     { Red - Select; Left Mouse; Joystick Fire }
 JPB_BUTTON_YELLOW  = 21;     { Yellow - Repeat                         }
 JPB_BUTTON_GREEN   = 20;     { Green - Shuffle                         }
 JPB_BUTTON_FORWARD = 19;     { Charcoal - Forward              }
 JPB_BUTTON_REVERSE = 18;     { Charcoal - Reverse              }
 JPB_BUTTON_PLAY    = 17;     { Grey - Play/Pause; Middle Mouse         }
 JPF_BUTTON_BLUE    = 8388608;
 JPF_BUTTON_RED     = 4194304;
 JPF_BUTTON_YELLOW  = 2097152;
 JPF_BUTTON_GREEN   = 1048576;
 JPF_BUTTON_FORWARD = 524288;
 JPF_BUTTON_REVERSE = 262144;
 JPF_BUTTON_PLAY    = 131072;
 JP_BUTTON_MASK     = JPF_BUTTON_BLUE OR JPF_BUTTON_RED OR JPF_BUTTON_YELLOW OR JPF_BUTTON_GREEN OR JPF_BUTTON_FORWARD OR JPF_BUTTON_REVERSE OR JPF_BUTTON_PLAY;

{ Direction types, valid for JP_TYPE_GAMECTLR and JP_TYPE_JOYSTK }
 JPB_JOY_UP        = 3;
 JPB_JOY_DOWN      = 2;
 JPB_JOY_LEFT      = 1;
 JPB_JOY_RIGHT     = 0;
 JPF_JOY_UP        = 8;
 JPF_JOY_DOWN      = 4;
 JPF_JOY_LEFT      = 2;
 JPF_JOY_RIGHT     = 1;
 JP_DIRECTION_MASK = JPF_JOY_UP OR JPF_JOY_DOWN OR JPF_JOY_LEFT OR JPF_JOY_RIGHT;

{ Mouse position reports, valid for JP_TYPE_MOUSE }
 JP_MHORZ_MASK     = 255;        { horzizontal position }
 JP_MVERT_MASK     = 65280;      { vertical position    }
 JP_MOUSE_MASK     = JP_MHORZ_MASK OR JP_MVERT_MASK;

{ Obsolete ReadJoyPort() definitions, here for source code compatibility only.
 * Please do NOT use in new code.
 }
 JPB_BTN1  = JPB_BUTTON_BLUE   ;
 JPF_BTN1  = JPF_BUTTON_BLUE   ;
 JPB_BTN2  = JPB_BUTTON_RED    ;
 JPF_BTN2  = JPF_BUTTON_RED    ;
 JPB_BTN3  = JPB_BUTTON_YELLOW ;
 JPF_BTN3  = JPF_BUTTON_YELLOW ;
 JPB_BTN4  = JPB_BUTTON_GREEN  ;
 JPF_BTN4  = JPF_BUTTON_GREEN  ;
 JPB_BTN5  = JPB_BUTTON_FORWARD;
 JPF_BTN5  = JPF_BUTTON_FORWARD;
 JPB_BTN6  = JPB_BUTTON_REVERSE;
 JPF_BTN6  = JPF_BUTTON_REVERSE;
 JPB_BTN7  = JPB_BUTTON_PLAY   ;
 JPF_BTN7  = JPF_BUTTON_PLAY   ;
 JPB_UP    = JPB_JOY_UP        ;
 JPF_UP    = JPF_JOY_UP        ;
 JPB_DOWN  = JPB_JOY_DOWN      ;
 JPF_DOWN  = JPF_JOY_DOWN      ;
 JPB_LEFT  = JPB_JOY_LEFT      ;
 JPF_LEFT  = JPF_JOY_LEFT      ;
 JPB_RIGHT = JPB_JOY_RIGHT     ;
 JPF_RIGHT = JPF_JOY_RIGHT     ;


{***************************************************************************}


{ Tags for SystemControl() }
 SCON_Dummy         = (TAG_USER+$00C00000);
 SCON_TakeOverSys   = (SCON_Dummy+0);
 SCON_KillReq       = (SCON_Dummy+1);
 SCON_CDReboot      = (SCON_Dummy+2);
 SCON_StopInput     = (SCON_Dummy+3);
 SCON_AddCreateKeys = (SCON_Dummy+4);
 SCON_RemCreateKeys = (SCON_Dummy+5);

{ Reboot control values for use with SCON_CDReboot tag }
 CDReboot_On        =     1;
 CDReboot_Off       =     0;
 CDReboot_Default   =     2;


{***************************************************************************}


{ Rawkey codes returned when using SCON_AddCreateKeys with SystemControl() }

 RAWKEY_PORT0_BUTTON_BLUE      =  $72;
 RAWKEY_PORT0_BUTTON_RED       =  $78;
 RAWKEY_PORT0_BUTTON_YELLOW    =  $77;
 RAWKEY_PORT0_BUTTON_GREEN     =  $76;
 RAWKEY_PORT0_BUTTON_FORWARD   =  $75;
 RAWKEY_PORT0_BUTTON_REVERSE   =  $74;
 RAWKEY_PORT0_BUTTON_PLAY      =  $73;
 RAWKEY_PORT0_JOY_UP           =  $79;
 RAWKEY_PORT0_JOY_DOWN         =  $7A;
 RAWKEY_PORT0_JOY_LEFT         =  $7C;
 RAWKEY_PORT0_JOY_RIGHT        =  $7B;

 RAWKEY_PORT1_BUTTON_BLUE      =  $172;
 RAWKEY_PORT1_BUTTON_RED       =  $178;
 RAWKEY_PORT1_BUTTON_YELLOW    =  $177;
 RAWKEY_PORT1_BUTTON_GREEN     =  $176;
 RAWKEY_PORT1_BUTTON_FORWARD   =  $175;
 RAWKEY_PORT1_BUTTON_REVERSE   =  $174;
 RAWKEY_PORT1_BUTTON_PLAY      =  $173;
 RAWKEY_PORT1_JOY_UP           =  $179;
 RAWKEY_PORT1_JOY_DOWN         =  $17A;
 RAWKEY_PORT1_JOY_LEFT         =  $17C;
 RAWKEY_PORT1_JOY_RIGHT        =  $17B;

 RAWKEY_PORT2_BUTTON_BLUE      =  $272;
 RAWKEY_PORT2_BUTTON_RED       =  $278;
 RAWKEY_PORT2_BUTTON_YELLOW    =  $277;
 RAWKEY_PORT2_BUTTON_GREEN     =  $276;
 RAWKEY_PORT2_BUTTON_FORWARD   =  $275;
 RAWKEY_PORT2_BUTTON_REVERSE   =  $274;
 RAWKEY_PORT2_BUTTON_PLAY      =  $273;
 RAWKEY_PORT2_JOY_UP           =  $279;
 RAWKEY_PORT2_JOY_DOWN         =  $27A;
 RAWKEY_PORT2_JOY_LEFT         =  $27C;
 RAWKEY_PORT2_JOY_RIGHT        =  $27B;

 RAWKEY_PORT3_BUTTON_BLUE      =  $372;
 RAWKEY_PORT3_BUTTON_RED       =  $378;
 RAWKEY_PORT3_BUTTON_YELLOW    =  $377;
 RAWKEY_PORT3_BUTTON_GREEN     =  $376;
 RAWKEY_PORT3_BUTTON_FORWARD   =  $375;
 RAWKEY_PORT3_BUTTON_REVERSE   =  $374;
 RAWKEY_PORT3_BUTTON_PLAY      =  $373;
 RAWKEY_PORT3_JOY_UP           =  $379;
 RAWKEY_PORT3_JOY_DOWN         =  $37A;
 RAWKEY_PORT3_JOY_LEFT         =  $37C;
 RAWKEY_PORT3_JOY_RIGHT        =  $37B;


{***************************************************************************}


{ Return values for GetLanguageSelection() }
 LANG_UNKNOWN    = 0 ;
 LANG_AMERICAN   = 1 ;      { American English }
 LANG_ENGLISH    = 2 ;      { British English  }
 LANG_GERMAN     = 3 ;
 LANG_FRENCH     = 4 ;
 LANG_SPANISH    = 5 ;
 LANG_ITALIAN    = 6 ;
 LANG_PORTUGUESE = 7 ;
 LANG_DANISH     = 8 ;
 LANG_DUTCH      = 9 ;
 LANG_NORWEGIAN  = 10;
 LANG_FINNISH    = 11;
 LANG_SWEDISH    = 12;
 LANG_JAPANESE   = 13;
 LANG_CHINESE    = 14;
 LANG_ARABIC     = 15;
 LANG_GREEK      = 16;
 LANG_HEBREW     = 17;
 LANG_KOREAN     = 18;


{***************************************************************************}


{ --- functions in V40 or higher (Release 3.1) --- }

VAR LowLevelBase : pLibrary;

FUNCTION AddKBInt(const intRoutine : POINTER;const intData : POINTER) : POINTER;
FUNCTION AddTimerInt(const intRoutine : POINTER;const intData : POINTER) : POINTER;
FUNCTION AddVBlankInt(const intRoutine : POINTER;const intData : POINTER) : POINTER;
FUNCTION ElapsedTime(context : pEClockVal) : ULONG;
FUNCTION GetKey : ULONG;
FUNCTION GetLanguageSelection : BYTE;
PROCEDURE QueryKeys(queryArray : pKeyQuery; arraySize : ULONG);
FUNCTION ReadJoyPort(port : ULONG) : ULONG;
PROCEDURE RemKBInt(intHandle : POINTER);
PROCEDURE RemTimerInt(intHandle : POINTER);
PROCEDURE RemVBlankInt(intHandle : POINTER);
FUNCTION SetJoyPortAttrsA(portNumber : ULONG;const tagList : pTagItem) : BOOLEAN;
PROCEDURE StartTimerInt(intHandle : POINTER; timeInterval : ULONG; continuous : LONGINT);
PROCEDURE StopTimerInt(intHandle : POINTER);
FUNCTION SystemControlA(const tagList : pTagItem) : ULONG;


{Here we read how to compile this unit}
{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitLOWLEVELLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    LOWLEVELIsCompiledHow : longint;

IMPLEMENTATION

{$ifndef dont_use_openlib}
uses msgbox;
{$endif dont_use_openlib}

FUNCTION AddKBInt(const intRoutine : POINTER;const intData : POINTER) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L intRoutine,A0
    MOVEA.L intData,A1
    MOVEA.L LowLevelBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AddTimerInt(const intRoutine : POINTER;const  intData : POINTER) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L intRoutine,A0
    MOVEA.L intData,A1
    MOVEA.L LowLevelBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AddVBlankInt(const intRoutine : POINTER;const intData : POINTER) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L intRoutine,A0
    MOVEA.L intData,A1
    MOVEA.L LowLevelBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ElapsedTime(context : pEClockVal) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L context,A0
    MOVEA.L LowLevelBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetKey : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L LowLevelBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetLanguageSelection : BYTE;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L LowLevelBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE QueryKeys(queryArray : pKeyQuery; arraySize : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L queryArray,A0
    MOVE.L  arraySize,D1
    MOVEA.L LowLevelBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION ReadJoyPort(port : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  port,D0
    MOVEA.L LowLevelBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE RemKBInt(intHandle : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L intHandle,A1
    MOVEA.L LowLevelBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RemTimerInt(intHandle : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L intHandle,A1
    MOVEA.L LowLevelBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RemVBlankInt(intHandle : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L intHandle,A1
    MOVEA.L LowLevelBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SetJoyPortAttrsA(portNumber : ULONG;const tagList : pTagItem) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  portNumber,D0
    MOVEA.L tagList,A1
    MOVEA.L LowLevelBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE StartTimerInt(intHandle : POINTER; timeInterval : ULONG; continuous : LONGINT);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L intHandle,A1
    MOVE.L  timeInterval,D0
    MOVE.L  continuous,D1
    MOVEA.L LowLevelBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE StopTimerInt(intHandle : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L intHandle,A1
    MOVEA.L LowLevelBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION SystemControlA(const tagList : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L tagList,A1
    MOVEA.L LowLevelBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of lowlevel.library}
  {$Info don't forget to use InitLOWLEVELLibrary in the beginning of your program}

var
    lowlevel_exit : Pointer;

procedure CloselowlevelLibrary;
begin
    ExitProc := lowlevel_exit;
    if LowLevelBase <> nil then begin
        CloseLibrary(LowLevelBase);
        LowLevelBase := nil;
    end;
end;

procedure InitLOWLEVELLibrary;
begin
    LowLevelBase := nil;
    LowLevelBase := OpenLibrary(LOWLEVELNAME,LIBVERSION);
    if LowLevelBase <> nil then begin
        lowlevel_exit := ExitProc;
        ExitProc := @CloselowlevelLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open lowlevel.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    LOWLEVELIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of lowlevel.library}

var
    lowlevel_exit : Pointer;

procedure CloselowlevelLibrary;
begin
    ExitProc := lowlevel_exit;
    if LowLevelBase <> nil then begin
        CloseLibrary(LowLevelBase);
        LowLevelBase := nil;
    end;
end;

begin
    LowLevelBase := nil;
    LowLevelBase := OpenLibrary(LOWLEVELNAME,LIBVERSION);
    if LowLevelBase <> nil then begin
        lowlevel_exit := ExitProc;
        ExitProc := @CloselowlevelLibrary;
        LOWLEVELIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open lowlevel.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    LOWLEVELIsCompiledHow := 3;
   {$Warning No autoopening of lowlevel.library compiled}
   {$Warning Make sure you open lowlevel.library yourself}
{$endif dont_use_openlib}

END. (* UNIT LOWLEVEL *)




