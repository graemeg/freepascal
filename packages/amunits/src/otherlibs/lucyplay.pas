{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 2003 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for lucyplay.library

  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
{
  History:

  First version of this library.
  16 Jan 2003.

  Changed cardinal > longword.
  Changed startcode for unit.
  12 Feb 2003.

  nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}


{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT LUCYPLAY;

INTERFACE
USES Exec;

VAR LucyPlayBase : pLibrary;

const
    LUCYPLAYNAME : PChar = 'lucyplay.library';

  type

     PLucyPlaySample = ^tLucyPlaySample;
     tLucyPlaySample = record
          SampleRate : ULONG;        {  sample frequency                 }
          Size : ULONG;              {  sample size (in bytes)           }
          _Type : ULONG;             {  AHI type (mono/stereo, 8/16bit)  }
          SampleData : pointer;      {  pointer to the sample data       }
       end;

     PLucyPlayJoystick = ^tLucyPlayJoystick;
     tLucyPlayJoystick = record
          Up : UBYTE;        {  0 means FALSE,  1 means TRUE     }
          Down : UBYTE;      {  if (Up == 1) Down = 0;           }
          Left : UBYTE;      {  Don't forget to check for        }
          Right : UBYTE;     {   Up/Down + Left/Right combos!    }
          Red : UBYTE;       {  Standard fire button             }
          Blue : UBYTE;      {  for 2-button sticks/pads         }
          Green : UBYTE;     {  for CD32 compatible pads only    }
          Yellow : UBYTE;    {  for CD32 compatible pads only    }
          Reverse : UBYTE;   {  for CD32 compatible pads only    }
          Forward : UBYTE;   {  for CD32 compatible pads only    }
          Play : UBYTE;      {  for CD32 compatible pads only    }
          Error : UBYTE;     {  hope this is always FALSE ;)     }
       end;

  const
     INVALID_ID =  not (0);

  const
     LUC_JOY_RIGHT = 1;
     LUC_JOY_LEFT = 2;
     LUC_JOY_DOWN = 4;
     LUC_JOY_UP = 8;
     LUC_JOY_PLAY = 16;
     LUC_JOY_REVERSE = 32;
     LUC_JOY_FORWARD = 64;
     LUC_JOY_GREEN = 128;
     LUC_JOY_YELLOW = 256;
     LUC_JOY_RED = 512;
     LUC_JOY_BLUE = 1024;
     LUC_ERR_ALLOCMEM = 1;
     LUC_ERR_CREATEIOREQUEST = 2;
     LUC_ERR_CREATEMSGPORT = 3;
     LUC_ERR_FILETYPE = 4;
     LUC_ERR_INTERNAL = 5;
     LUC_ERR_OPENDEVICE = 6;
     LUC_ERR_OPENFILE = 7;
     LUC_ERR_OPENLIBRARY = 8;
     LUC_ERR_READJOYPORT = 9;
     LUC_ERR_DOIO = 10;




PROCEDURE lucAudioFree(smp : pLucyPlaySample);
FUNCTION lucAudioInit : LONGINT;
PROCEDURE lucAudioKill;
FUNCTION lucAudioLoad(fname : pCHAR) : pLucyPlaySample;
PROCEDURE lucAudioPlay(smp : pLucyPlaySample);
PROCEDURE lucAudioStop;
PROCEDURE lucAudioWait;
FUNCTION lucBestModeID(w : longword; h : longword; d : longword) : longword;
FUNCTION lucError : longword;
FUNCTION lucJoyInit : pLucyPlayJoystick;
FUNCTION lucJoyInitForce : pLucyPlayJoystick;
PROCEDURE lucJoyKill(joy : pLucyPlayJoystick);
PROCEDURE lucJoyRead(joy : pLucyPlayJoystick);
FUNCTION lucJoyReadBool : longword;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitLUCYPLAYLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    LUCYPLAYIsCompiledHow : longint;

IMPLEMENTATION

{$ifndef dont_use_openlib}
uses msgbox;
{$endif dont_use_openlib}

PROCEDURE lucAudioFree(smp : pLucyPlaySample);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L smp,A0
        MOVEA.L LucyPlayBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION lucAudioInit : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L LucyPlayBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE lucAudioKill;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L LucyPlayBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION lucAudioLoad(fname : pCHAR) : pLucyPlaySample;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L fname,A0
        MOVEA.L LucyPlayBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE lucAudioPlay(smp : pLucyPlaySample);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L smp,A0
        MOVEA.L LucyPlayBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE lucAudioStop;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L LucyPlayBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE lucAudioWait;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L LucyPlayBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION lucBestModeID(w : longword; h : longword; d : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  w,D0
        MOVE.L  h,D1
        MOVE.L  d,D2
        MOVEA.L LucyPlayBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION lucError : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L LucyPlayBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION lucJoyInit : pLucyPlayJoystick;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L LucyPlayBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION lucJoyInitForce : pLucyPlayJoystick;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L LucyPlayBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE lucJoyKill(joy : pLucyPlayJoystick);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L joy,A0
        MOVEA.L LucyPlayBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE lucJoyRead(joy : pLucyPlayJoystick);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L joy,A0
        MOVEA.L LucyPlayBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION lucJoyReadBool : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L LucyPlayBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of lucyplay.library}
  {$Info don't forget to use InitLUCYPLAYLibrary in the beginning of your program}

var
    lucyplay_exit : Pointer;

procedure CloselucyplayLibrary;
begin
    ExitProc := lucyplay_exit;
    if LucyPlayBase <> nil then begin
        CloseLibrary(LucyPlayBase);
        LucyPlayBase := nil;
    end;
end;

procedure InitLUCYPLAYLibrary;
begin
    LucyPlayBase := nil;
    LucyPlayBase := OpenLibrary(LUCYPLAYNAME,LIBVERSION);
    if LucyPlayBase <> nil then begin
        lucyplay_exit := ExitProc;
        ExitProc := @CloselucyplayLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open lucyplay.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    LUCYPLAYIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of lucyplay.library}

var
    lucyplay_exit : Pointer;

procedure CloselucyplayLibrary;
begin
    ExitProc := lucyplay_exit;
    if LucyPlayBase <> nil then begin
        CloseLibrary(LucyPlayBase);
        LucyPlayBase := nil;
    end;
end;

begin
    LucyPlayBase := nil;
    LucyPlayBase := OpenLibrary(LUCYPLAYNAME,LIBVERSION);
    if LucyPlayBase <> nil then begin
        lucyplay_exit := ExitProc;
        ExitProc := @CloselucyplayLibrary;
        LUCYPLAYIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open lucyplay.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    LUCYPLAYIsCompiledHow := 3;
   {$Warning No autoopening of lucyplay.library compiled}
   {$Warning Make sure you open lucyplay.library yourself}
{$endif dont_use_openlib}

END. (* UNIT LUCYPLAY *)



