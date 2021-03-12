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
{$PACKRECORDS 2}

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




VAR AHIsubBase : pLibrary = nil;

const
    AHI_SUBNAME : PChar = 'ahi_sub.library';


FUNCTION AHIsub_AllocAudio(tagList : pTagItem location 'a1'; AudioCtrl : pAHIAudioCtrlDrv location 'a2') : longword; syscall AHIsubBase 30;
PROCEDURE AHIsub_Disable(AudioCtrl : pAHIAudioCtrlDrv location 'a2'); syscall AHIsubBase 42;
PROCEDURE AHIsub_Enable(AudioCtrl : pAHIAudioCtrlDrv location 'a2'); syscall AHIsubBase 48;
PROCEDURE AHIsub_FreeAudio(AudioCtrl : pAHIAudioCtrlDrv location 'a2'); syscall AHIsubBase 36;
FUNCTION AHIsub_GetAttr(Attribute : longword location 'd0'; Argument : LONGINT location 'd1'; d2arg : LONGINT location 'd2'; tagList : pTagItem location 'a1'; AudioCtrl : pAHIAudioCtrlDrv location 'a2') : LONGINT; syscall AHIsubBase 108;
FUNCTION AHIsub_HardwareControl(Attribute : longword location 'd0'; Argument : LONGINT location 'd1'; AudioCtrl : pAHIAudioCtrlDrv location 'a2') : LONGINT; syscall AHIsubBase 114;
FUNCTION AHIsub_LoadSound(Sound : WORD location 'd0'; _Type : longword location 'd1'; Info : POINTER location 'a0'; AudioCtrl : pAHIAudioCtrlDrv location 'a2') : longword; syscall AHIsubBase 96;
FUNCTION AHIsub_SetEffect(Effect : POINTER location 'a0'; AudioCtrl : pAHIAudioCtrlDrv location 'a2') : longword; syscall AHIsubBase 90;
FUNCTION AHIsub_SetFreq(Channel : WORD location 'd0'; Freq : longword location 'd1'; AudioCtrl : pAHIAudioCtrlDrv location 'a2'; Flags : longword location 'd2') : longword; syscall AHIsubBase 78;
FUNCTION AHIsub_SetSound(Channel : WORD location 'd0'; Sound : WORD location 'd1'; Offset : longword location 'd2'; Length : LONGINT location 'd3'; AudioCtrl : pAHIAudioCtrlDrv location 'a2'; Flags : longword location 'd4') : longword; syscall AHIsubBase 84;
FUNCTION AHIsub_SetVol(Channel : WORD location 'd0'; Volume : LONGINT location 'd1'; Pan : LONGINT location 'd2'; AudioCtrl : pAHIAudioCtrlDrv location 'a2'; Flags : longword location 'd3') : longword; syscall AHIsubBase 72;
FUNCTION AHIsub_Start(Flags : longword location 'd0'; AudioCtrl : pAHIAudioCtrlDrv location 'a2') : longword; syscall AHIsubBase 54;
FUNCTION AHIsub_Stop(Flags : longword location 'd0'; AudioCtrl : pAHIAudioCtrlDrv location 'a2') : longword; syscall AHIsubBase 66;
FUNCTION AHIsub_UnloadSound(Sound : WORD location 'd0'; Audioctrl : pAHIAudioCtrlDrv location 'a2') : longword; syscall AHIsubBase 102;
FUNCTION AHIsub_Update(Flags : longword location 'd0'; AudioCtrl : pAHIAudioCtrlDrv location 'a2') : longword; syscall AHIsubBase 60;

IMPLEMENTATION

const
    { Change VERSION and LIBVERSION to proper values }
    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

initialization
  AHIsubBase := OpenLibrary(AHI_SUBNAME,LIBVERSION);
finalization
  if Assigned(AHIsubBase) then
    CloseLibrary(AHIsubBase);
END. (* UNIT AHI_SUB *)



