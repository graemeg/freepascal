{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 2003 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for ahi_sub.library

  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}

{
  History:

  First version of this unit.
  15 Jan 2003.

  Changed cardinal > longword.
  Added startcode for unit.
  12 Feb 2003.

  nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT AHI_SUB;

INTERFACE
USES Exec, ahi, utility;


  {
        $VER: ahi_sub.h 4.1 (2.4.97)

        ahi/[driver].audio definitions

        (C) Copyright 1994-1997 Martin Blom
        All Rights Reserved.

     (TAB SIZE: 8)
   }

   { AHIAudioCtrlDrv  }
  type
     PAHIAudioCtrlDrv = ^tAHIAudioCtrlDrv;
     tAHIAudioCtrlDrv = record
          ahiac_AudioCtrl : tAHIAudioCtrl;
          ahiac_Flags : ULONG;              { See below for definition  }
          ahiac_SoundFunc : PHook;          { AHIA_SoundFunc  }
          ahiac_PlayerFunc : PHook;         { AHIA_PlayerFunc  }
          ahiac_PlayerFreq : Fixed;         { AHIA_PlayerFreq  }
          ahiac_MinPlayerFreq : Fixed;      { AHIA_MinPlayerFreq  }
          ahiac_MaxPlayerFreq : Fixed;      { AHIA_MaxPlayerFreq  }
          ahiac_MixFreq : ULONG;            { AHIA_MixFreq  }
          ahiac_Channels : UWORD;           { AHIA_Channels  }
          ahiac_Sounds : UWORD;             { AHIA_Sounds  }
          ahiac_DriverData : APTR;          { Unused. Store whatever you want here.  }
          ahiac_MixerFunc : PHook;          { Mixing routine Hook  }
          ahiac_SamplerFunc : PHook;        { Sampler routine Hook  }
          ahiac_Obsolete : ULONG;
          ahiac_BuffSamples : ULONG;        { Samples to mix this pass.  }
          ahiac_MinBuffSamples : ULONG;     { Min. samples to mix each pass.  }
          ahiac_MaxBuffSamples : ULONG;     { Max. samples to mix each pass.  }
          ahiac_BuffSize : ULONG;           { Buffer size ahiac_MixerFunc needs.  }
          ahiac_BuffType : ULONG;           { Buffer format (V2)  }
          ahiac_PreTimer : function :BOOL;  { Call before mixing (V4)  }
          ahiac_PostTimer : procedure;      { Call after mixing (V4)  }
       end;

       { The rest is PRIVATE! Hands off! They may change any time.
        [lots of private stuff]  }

  {   TAGS  }
  const
     AHIDB_UserBase = AHI_TagBase + 500;    { Use for driver specific tags  }
  {   DEFS  }
  { AHIsub_AllocAudio return flags  }
     AHISF_ERROR = 1 shl 0;
     AHISF_MIXING = 1 shl 1;
     AHISF_TIMING = 1 shl 2;
     AHISF_KNOWSTEREO = 1 shl 3;
     AHISF_KNOWHIFI = 1 shl 4;
     AHISF_CANRECORD = 1 shl 5;
     AHISF_CANPOSTPROCESS = 1 shl 6;
     AHISB_ERROR = 0;
     AHISB_MIXING = 1;
     AHISB_TIMING = 2;
     AHISB_KNOWSTEREO = 3;
     AHISB_KNOWHIFI = 4;
     AHISB_CANRECORD = 5;
     AHISB_CANPOSTPROCESS = 6;
  { AHIsub_Start() and AHIsub_Stop() flags  }
     AHISF_PLAY = 1 shl 0;
     AHISF_RECORD = 1 shl 1;
     AHISB_PLAY = 0;
     AHISB_RECORD = 1;
  { ahiac_Flags  }
     AHIACF_VOL = 1 shl 0;
     AHIACF_PAN = 1 shl 1;
     AHIACF_STEREO = 1 shl 2;
     AHIACF_HIFI = 1 shl 3;
     AHIACF_PINGPONG = 1 shl 4;
     AHIACF_RECORD = 1 shl 5;

     AHIACF_MULTTAB = 1 shl 6;   { Private!  }
     AHIACB_VOL = 0;
     AHIACB_PAN = 1;
     AHIACB_STEREO = 2;
     AHIACB_HIFI = 3;
     AHIACB_PINGPONG = 4;
     AHIACB_RECORD = 5;

     AHIACB_MULTTAB = 6;        { Private!  }

  { AHIsub_Set#? and AHIsub_(Un)LoadSound return code  }
     AHIS_UNKNOWN =  not (0);

  { IFF chunk names for the audio mode file  }
     ID_AHIM = $4148494D;     { AHI Modes  }
     ID_AUDN = $4155544E;     { AUDio driver Name  }
     ID_AUDD = $41554444;     { AUDio driver Data  }
     ID_AUDM = $4155444D;     { AUDio Mode  }




VAR AHIsubBase : pLibrary;

const
    AHI_SUBNAME : PChar = 'ahi_sub.library';


FUNCTION AHIsub_AllocAudio(tagList : pTagItem; AudioCtrl : pAHIAudioCtrlDrv) : longword;
PROCEDURE AHIsub_Disable(AudioCtrl : pAHIAudioCtrlDrv);
PROCEDURE AHIsub_Enable(AudioCtrl : pAHIAudioCtrlDrv);
PROCEDURE AHIsub_FreeAudio(AudioCtrl : pAHIAudioCtrlDrv);
FUNCTION AHIsub_GetAttr(Attribute : longword; Argument : LONGINT; d2arg : LONGINT; tagList : pTagItem; AudioCtrl : pAHIAudioCtrlDrv) : LONGINT;
FUNCTION AHIsub_HardwareControl(Attribute : longword; Argument : LONGINT; AudioCtrl : pAHIAudioCtrlDrv) : LONGINT;
FUNCTION AHIsub_LoadSound(Sound : WORD; _Type : longword; Info : POINTER; AudioCtrl : pAHIAudioCtrlDrv) : longword;
FUNCTION AHIsub_SetEffect(Effect : POINTER; AudioCtrl : pAHIAudioCtrlDrv) : longword;
FUNCTION AHIsub_SetFreq(Channel : WORD; Freq : longword; AudioCtrl : pAHIAudioCtrlDrv; Flags : longword) : longword;
FUNCTION AHIsub_SetSound(Channel : WORD; Sound : WORD; Offset : longword; Length : LONGINT; AudioCtrl : pAHIAudioCtrlDrv; Flags : longword) : longword;
FUNCTION AHIsub_SetVol(Channel : WORD; Volume : LONGINT; Pan : LONGINT; AudioCtrl : pAHIAudioCtrlDrv; Flags : longword) : longword;
FUNCTION AHIsub_Start(Flags : longword; AudioCtrl : pAHIAudioCtrlDrv) : longword;
FUNCTION AHIsub_Stop(Flags : longword; AudioCtrl : pAHIAudioCtrlDrv) : longword;
FUNCTION AHIsub_UnloadSound(Sound : WORD; Audioctrl : pAHIAudioCtrlDrv) : longword;
FUNCTION AHIsub_Update(Flags : longword; AudioCtrl : pAHIAudioCtrlDrv) : longword;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitAHI_SUBLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    AHI_SUBIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray;


FUNCTION AHIsub_AllocAudio(tagList : pTagItem; AudioCtrl : pAHIAudioCtrlDrv) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L tagList,A1
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE AHIsub_Disable(AudioCtrl : pAHIAudioCtrlDrv);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AHIsub_Enable(AudioCtrl : pAHIAudioCtrlDrv);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AHIsub_FreeAudio(AudioCtrl : pAHIAudioCtrlDrv);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AHIsub_GetAttr(Attribute : longword; Argument : LONGINT; d2arg : LONGINT; tagList : pTagItem; AudioCtrl : pAHIAudioCtrlDrv) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Attribute,D0
        MOVE.L  Argument,D1
        MOVE.L  d2arg,D2
        MOVEA.L tagList,A1
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHIsub_HardwareControl(Attribute : longword; Argument : LONGINT; AudioCtrl : pAHIAudioCtrlDrv) : LONGINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Attribute,D0
        MOVE.L  Argument,D1
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHIsub_LoadSound(Sound : WORD; _Type : longword; Info : POINTER; AudioCtrl : pAHIAudioCtrlDrv) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Sound,D0
        MOVE.L  _Type,D1
        MOVEA.L Info,A0
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHIsub_SetEffect(Effect : POINTER; AudioCtrl : pAHIAudioCtrlDrv) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Effect,A0
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHIsub_SetFreq(Channel : WORD; Freq : longword; AudioCtrl : pAHIAudioCtrlDrv; Flags : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Channel,D0
        MOVE.L  Freq,D1
        MOVEA.L AudioCtrl,A2
        MOVE.L  Flags,D2
        MOVEA.L AHIsubBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHIsub_SetSound(Channel : WORD; Sound : WORD; Offset : longword; Length : LONGINT; AudioCtrl : pAHIAudioCtrlDrv; Flags : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Channel,D0
        MOVE.L  Sound,D1
        MOVE.L  Offset,D2
        MOVE.L  Length,D3
        MOVEA.L AudioCtrl,A2
        MOVE.L  Flags,D4
        MOVEA.L AHIsubBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHIsub_SetVol(Channel : WORD; Volume : LONGINT; Pan : LONGINT; AudioCtrl : pAHIAudioCtrlDrv; Flags : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Channel,D0
        MOVE.L  Volume,D1
        MOVE.L  Pan,D2
        MOVEA.L AudioCtrl,A2
        MOVE.L  Flags,D3
        MOVEA.L AHIsubBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHIsub_Start(Flags : longword; AudioCtrl : pAHIAudioCtrlDrv) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Flags,D0
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHIsub_Stop(Flags : longword; AudioCtrl : pAHIAudioCtrlDrv) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Flags,D0
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHIsub_UnloadSound(Sound : WORD; Audioctrl : pAHIAudioCtrlDrv) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Sound,D0
        MOVEA.L Audioctrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHIsub_Update(Flags : longword; AudioCtrl : pAHIAudioCtrlDrv) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Flags,D0
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIsubBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of ahi_sub.library}
  {$Info don't forget to use InitAHI_SUBLibrary in the beginning of your program}

var
    ahi_sub_exit : Pointer;

procedure Closeahi_subLibrary;
begin
    ExitProc := ahi_sub_exit;
    if AHIsubBase <> nil then begin
        CloseLibrary(AHIsubBase);
        AHIsubBase := nil;
    end;
end;

procedure InitAHI_SUBLibrary;
begin
    AHIsubBase := nil;
    AHIsubBase := OpenLibrary(AHI_SUBNAME,LIBVERSION);
    if AHIsubBase <> nil then begin
        ahi_sub_exit := ExitProc;
        ExitProc := @Closeahi_subLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open ahi_sub.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    AHI_SUBIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of ahi_sub.library}

var
    ahi_sub_exit : Pointer;

procedure Closeahi_subLibrary;
begin
    ExitProc := ahi_sub_exit;
    if AHIsubBase <> nil then begin
        CloseLibrary(AHIsubBase);
        AHIsubBase := nil;
    end;
end;

begin
    AHIsubBase := nil;
    AHIsubBase := OpenLibrary(AHI_SUBNAME,LIBVERSION);
    if AHIsubBase <> nil then begin
        ahi_sub_exit := ExitProc;
        ExitProc := @Closeahi_subLibrary;
        AHI_SUBIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open ahi_sub.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    AHI_SUBIsCompiledHow := 3;
   {$Warning No autoopening of ahi_sub.library compiled}
   {$Warning Make sure you open ahi_sub.library yourself}
{$endif dont_use_openlib}


END. (* UNIT AHI_SUB *)



