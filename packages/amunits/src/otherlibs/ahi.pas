{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 2003 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for ahi.library

  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
{
  History:

  First version of this unit.
  14 Jan 2003.

  Changed cardinal > longword.
  Added startcode for unit.
  12 Feb 2003.

  nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT AHI;

INTERFACE

USES Exec,utility;

VAR AHIBase : pLibrary;

{
        $VER: ahi.h 4.2 (27.4.97)

        ahi.device definitions

        (C) Copyright 1994-1997 Martin Blom
        All Rights Reserved.

     (TAB SIZE: 8)
   }

  type

     PFixed = ^Fixed;
     Fixed = LONGINT;
  { A fixed-point value, 16 bits
    to the left of the point and
    16 bits to the right  }

  type

     Psposition = ^sposition;
     sposition = Fixed;

  { AHIAudioCtrl  }
  { Lots of private data follows!  }
     PAHIAudioCtrl = ^tAHIAudioCtrl;
     tAHIAudioCtrl = record
          ahiac_UserData : APTR;
       end;

  { AHISoundMessage  }
     PAHISoundMessage = ^tAHISoundMessage;
     tAHISoundMessage = record
          ahism_Channel : UWORD;
       end;

  { AHIRecordMessage  }
     PAHIRecordMessage = ^tAHIRecordMessage;
     tAHIRecordMessage = record
          ahirm_Type : ULONG;     { Format of buffer (object)  }
          ahirm_Buffer : APTR;    { Pointer to the sample array  }
          ahirm_Length : ULONG;   { Number of sample frames in buffer  }
       end;

  { AHISampleInfo  }
     PAHISampleInfo = ^tAHISampleInfo;
     tAHISampleInfo = record
          ahisi_Type : ULONG;      { Format of samples  }
          ahisi_Address : APTR;    { Address to array of samples  }
          ahisi_Length : ULONG;    { Number of samples in array  }
       end;

  { AHIAudioModeRequester  }
     PAHIAudioModeRequester = ^tAHIAudioModeRequester;
     tAHIAudioModeRequester = record
          ahiam_AudioID : ULONG;      { Selected audio mode  }
          ahiam_MixFreq : ULONG;      { Selected mixing/sampling frequency  }
          ahiam_LeftEdge : WORD;      { Coordinates of requester on exit  }
          ahiam_TopEdge : WORD;
          ahiam_Width : WORD;
          ahiam_Height : WORD;
          ahiam_InfoOpened : BOOL;    { Info window opened on exit?  }
          ahiam_InfoLeftEdge : WORD;  { Last coordinates of Info window  }
          ahiam_InfoTopEdge : WORD;
          ahiam_InfoWidth : WORD;
          ahiam_InfoHeight : WORD;
          ahiam_UserData : APTR;      { You can store your own data here  }
          { Lots of private data follows!  }
       end;

  { AHIEffMasterVolume  }
     PAHIEffMasterVolume = ^tAHIEffMasterVolume;
     tAHIEffMasterVolume = record
          ahie_Effect : ULONG;     { Set to AHIET_MASTERVOLUME  }
          ahiemv_Volume : Fixed;   { See autodocs for range!  }
       end;

  { AHIEffOutputBuffer  }
     PAHIEffOutputBuffer = ^tAHIEffOutputBuffer;
     tAHIEffOutputBuffer = record
          ahie_Effect : ULONG;     { Set to AHIET_OUTPUTBUFFER  }
          ahieob_Func : PHook;
          { These fields are filled by AHI  }
          ahieob_Type : ULONG;     { Format of buffer  }
          ahieob_Buffer : APTR;    { Pointer to the sample array  }
          ahieob_Length : ULONG;   { Number of sample frames in buffer  }
       end;

  { AHIEffDSPMask (V4)  }
     PAHIEffDSPMask = ^tAHIEffDSPMask;
     tAHIEffDSPMask = record
          ahie_Effect : ULONG;       { Set to AHIET_DSPMASK  }
          ahiedm_Channels : UWORD;   { Number of elements in array  }
          ahiedm_Mask : array[0..0] of UBYTE;   { Here follows the array  }
       end;


  const
     AHIEDM_WET = 0;
     AHIEDM_DRY = 1;
  { AHIEffDSPEcho (V4)  }
  type
     PAHIDSPEcho = ^tAHIDSPEcho;
     tAHIDSPEcho = record
          ahie_Effect : ULONG;     { Set to AHIET_DSPECHO  }
          ahiede_Delay : ULONG;    { In samples  }
          ahiede_Feedback : Fixed;
          ahiede_Mix : Fixed;
          ahiede_Cross : Fixed;
       end;

  { AHIEffChannelInfo (V4)  }
     PAHIEffChannelInfo = ^tAHIEffChannelInfo;
     tAHIEffChannelInfo = record
          ahie_Effect : ULONG;    { Set to AHIET_CHANNELINFO  }
          ahieci_Func : PHook;
          ahieci_Channels : UWORD;
          ahieci_Pad : UWORD;
          { The rest is filled by AHI  }
          ahieci_Offset : array[0..0] of ULONG;   { The array follows  }
       end;

  {   TAGS  }

  const
     AHI_TagBase = TAG_USER;
     AHI_TagBaseR = AHI_TagBase or $8000;
  { AHI_AllocAudioA tags  }
  { Desired audio mode  }
     AHIA_AudioID = AHI_TagBase + 1;
  { Suggested mixing frequency  }
     AHIA_MixFreq = AHI_TagBase + 2;
  { Suggested number of channels  }
     AHIA_Channels = AHI_TagBase + 3;
  { Number of sounds to use  }
     AHIA_Sounds = AHI_TagBase + 4;
  { End-of-Sound Hook  }
     AHIA_SoundFunc = AHI_TagBase + 5;
  { Player Hook  }
     AHIA_PlayerFunc = AHI_TagBase + 6;
  { Frequency for player Hook (Fixed) }
     AHIA_PlayerFreq = AHI_TagBase + 7;
  { Minimum Frequency for player Hook  }
     AHIA_MinPlayerFreq = AHI_TagBase + 8;
  { Maximum Frequency for player Hook  }
     AHIA_MaxPlayerFreq = AHI_TagBase + 9;
  { Sample recording Hook  }
     AHIA_RecordFunc = AHI_TagBase + 10;
  { What to put in ahiac_UserData  }
     AHIA_UserData = AHI_TagBase + 11;
  { AHI_PlayA tags (V4)  }
  { All command tags should be...  }
     AHIP_BeginChannel = AHI_TagBase + 40;
  { ... enclosed by these tags.  }
     AHIP_EndChannel = AHI_TagBase + 41;
     AHIP_Freq = AHI_TagBase + 50;
     AHIP_Vol = AHI_TagBase + 51;
     AHIP_Pan = AHI_TagBase + 52;
     AHIP_Sound = AHI_TagBase + 53;
     AHIP_Offset = AHI_TagBase + 54;
     AHIP_Length = AHI_TagBase + 55;
     AHIP_LoopFreq = AHI_TagBase + 60;
     AHIP_LoopVol = AHI_TagBase + 61;
     AHIP_LoopPan = AHI_TagBase + 62;
     AHIP_LoopSound = AHI_TagBase + 63;
     AHIP_LoopOffset = AHI_TagBase + 64;
     AHIP_LoopLength = AHI_TagBase + 65;
  { AHI_ControlAudioA tags  }
  { Boolean  }
     AHIC_Play = AHI_TagBase + 80;
  { Boolean  }
     AHIC_Record = AHI_TagBase + 81;
     AHIC_MonitorVolume = AHI_TagBase + 82;
  { ti_Data is pointer to Fixed (LONG)  }
     AHIC_MonitorVolume_Query = AHI_TagBase + 83;
  { ti_Data is pointer to ULONG  }
     AHIC_MixFreq_Query = AHI_TagBase + 84;
  { --- New for V2, they will be ignored by V1 ---  }
     AHIC_InputGain = AHI_TagBase + 85;
  { ti_Data is pointer to Fixed (LONG)  }
     AHIC_InputGain_Query = AHI_TagBase + 86;
     AHIC_OutputVolume = AHI_TagBase + 87;
  { ti_Data is pointer to Fixed (LONG)  }
     AHIC_OutputVolume_Query = AHI_TagBase + 88;
     AHIC_Input = AHI_TagBase + 89;
  { ti_Data is pointer to ULONG  }
     AHIC_Input_Query = AHI_TagBase + 90;
     AHIC_Output = AHI_TagBase + 91;
  { ti_Data is pointer to ULONG  }
     AHIC_Output_Query = AHI_TagBase + 92;
  { AHI_GetAudioAttrsA tags  }
     AHIDB_AudioID = AHI_TagBase + 100;
  { Pointer to name of driver  }
     AHIDB_Driver = AHI_TagBaseR + 101;
  { Private!  }
     AHIDB_Flags = AHI_TagBase + 102;
  { Boolean  }
     AHIDB_Volume = AHI_TagBase + 103;
  { Boolean  }
     AHIDB_Panning = AHI_TagBase + 104;
  { Boolean  }
     AHIDB_Stereo = AHI_TagBase + 105;
  { Boolean  }
     AHIDB_HiFi = AHI_TagBase + 106;
  { Boolean  }
     AHIDB_PingPong = AHI_TagBase + 107;
  { Private!  }
     AHIDB_MultTable = AHI_TagBase + 108;
  { Pointer to name of this mode  }
     AHIDB_Name = AHI_TagBaseR + 109;
  { Output bits  }
     AHIDB_Bits = AHI_TagBase + 110;
  { Max supported channels  }
     AHIDB_MaxChannels = AHI_TagBase + 111;
  { Min mixing freq. supported  }
     AHIDB_MinMixFreq = AHI_TagBase + 112;
  { Max mixing freq. supported  }
     AHIDB_MaxMixFreq = AHI_TagBase + 113;
  { Boolean  }
     AHIDB_Record = AHI_TagBase + 114;
     AHIDB_Frequencies = AHI_TagBase + 115;
  { ti_Data is frequency index  }
     AHIDB_FrequencyArg = AHI_TagBase + 116;
     AHIDB_Frequency = AHI_TagBase + 117;
  { Pointer to driver author name  }
     AHIDB_Author = AHI_TagBase + 118;
  { Pointer to driver copyright notice  }
     AHIDB_Copyright = AHI_TagBase + 119;
  { Pointer to driver version string  }
     AHIDB_Version = AHI_TagBase + 120;
  { Pointer to driver annotation text  }
     AHIDB_Annotation = AHI_TagBase + 121;
  { Specifies the string buffer size  }
     AHIDB_BufferLen = AHI_TagBase + 122;
  { ti_Data is frequency!  }
     AHIDB_IndexArg = AHI_TagBase + 123;
     AHIDB_Index = AHI_TagBase + 124;
  { Boolean  }
     AHIDB_Realtime = AHI_TagBase + 125;
  { It's sample  frames   }
     AHIDB_MaxPlaySamples = AHI_TagBase + 126;
  { It's sample  frames   }
     AHIDB_MaxRecordSamples = AHI_TagBase + 127;
  { Boolean  }
     AHIDB_FullDuplex = AHI_TagBase + 129;
  { --- New for V2, they will be ignored by V1 ---  }
     AHIDB_MinMonitorVolume = AHI_TagBase + 130;
     AHIDB_MaxMonitorVolume = AHI_TagBase + 131;
     AHIDB_MinInputGain = AHI_TagBase + 132;
     AHIDB_MaxInputGain = AHI_TagBase + 133;
     AHIDB_MinOutputVolume = AHI_TagBase + 134;
     AHIDB_MaxOutputVolume = AHI_TagBase + 135;
     AHIDB_Inputs = AHI_TagBase + 136;
  { ti_Data is input index  }
     AHIDB_InputArg = AHI_TagBase + 137;
     AHIDB_Input = AHI_TagBase + 138;
     AHIDB_Outputs = AHI_TagBase + 139;
  { ti_Data is input index  }
     AHIDB_OutputArg = AHI_TagBase + 140;
     AHIDB_Output = AHI_TagBase + 141;
  { --- New for V4, they will be ignored by V2 and earlier ---  }
  { Private!  }
     AHIDB_Data = AHI_TagBaseR + 142;
  { AHI_BestAudioIDA tags  }
  { --- New for V4, they will be ignored by V2 and earlier ---  }
     AHIB_Dizzy = AHI_TagBase + 190;
  { AHI_AudioRequestA tags  }
  { Window control  }
  { Parent window  }
     AHIR_Window = AHI_TagBase + 200;
  { Screen to open on if no window  }
     AHIR_Screen = AHI_TagBase + 201;
  { Name of public screen  }
     AHIR_PubScreenName = AHI_TagBase + 202;
  { Allocate private IDCMP?  }
     AHIR_PrivateIDCMP = AHI_TagBase + 203;
  { Function to handle IntuiMessages  }
     AHIR_IntuiMsgFunc = AHI_TagBase + 204;
  { Block input in AHIR_Window?  }
     AHIR_SleepWindow = AHI_TagBase + 205;
  { What to put in ahiam_UserData  }
     AHIR_UserData = AHI_TagBase + 206;
  { Text display  }
  { Text font to use for gadget text  }
     AHIR_TextAttr = AHI_TagBase + 220;
  { Locale to use for text  }
     AHIR_Locale = AHI_TagBase + 221;
  { Title of requester  }
     AHIR_TitleText = AHI_TagBase + 222;
  { Positive gadget text  }
     AHIR_PositiveText = AHI_TagBase + 223;
  { Negative gadget text  }
     AHIR_NegativeText = AHI_TagBase + 224;
  { Initial settings  }
  { Initial requester coordinates  }
     AHIR_InitialLeftEdge = AHI_TagBase + 240;
     AHIR_InitialTopEdge = AHI_TagBase + 241;
  { Initial requester dimensions  }
     AHIR_InitialWidth = AHI_TagBase + 242;
     AHIR_InitialHeight = AHI_TagBase + 243;
  { Initial audio mode id  }
     AHIR_InitialAudioID = AHI_TagBase + 244;
  { Initial mixing/sampling frequency  }
     AHIR_InitialMixFreq = AHI_TagBase + 245;
  { Info window initially opened?  }
     AHIR_InitialInfoOpened = AHI_TagBase + 246;
  { Initial Info window coords.  }
     AHIR_InitialInfoLeftEdge = AHI_TagBase + 247;
     AHIR_InitialInfoTopEdge = AHI_TagBase + 248;
  { Not used!  }
     AHIR_InitialInfoWidth = AHI_TagBase + 249;
  { Not used!  }
     AHIR_InitialInfoHeight = AHI_TagBase + 250;
  { Options  }
  { Allow selection of mixing frequency?  }
     AHIR_DoMixFreq = AHI_TagBase + 260;
  { Allow selection of default mode? (V4)  }
     AHIR_DoDefaultMode = AHI_TagBase + 261;
  { Filtering  }
  { Pointer to filter taglist  }
     AHIR_FilterTags = AHI_TagBase + 270;
  { Function to filter mode id's  }
     AHIR_FilterFunc = AHI_TagBase + 271;
  {   DEFS  }
     AHINAME : PChar = 'ahi.device';
  { Invalid Audio ID  }
     AHI_INVALID_ID =  not (0);
  { Only for AHI_AllocAudioA()!  }
     AHI_DEFAULT_ID = $00000000;
  { Special sample render Audio ID  }
     AHI_LOOPBACK_ID = $00000001;
  { Only for AHI_AllocAudioA()!  }
     AHI_DEFAULT_FREQ = 0;
  { Special frequency for AHI_SetFreq()  }
     AHI_MIXFREQ =  not (0);
  { Turns a channel off  }
     AHI_NOSOUND = $ffff;
  { Set#? Flags  }
     AHISF_IMM = 1 shl 0;
     AHISB_IMM = 0;
  { Effect Types  }
  { OR with effect to disable  }
     AHIET_CANCEL = 1 shl 31;
     AHIET_MASTERVOLUME = 1;
     AHIET_OUTPUTBUFFER = 2;
  { --- New for V4 ---  }
     AHIET_DSPMASK = 3;
     AHIET_DSPECHO = 4;
     AHIET_CHANNELINFO = 5;
  { Sound Types  }
  { Private  }
     AHIST_NOTYPE =  not (0);
  { 8 or 16 bit sample  }
     AHIST_SAMPLE = 0;
  { Dynamic sample  }
     AHIST_DYNAMICSAMPLE = 1;
  { The input from your sampler  }
     AHIST_INPUT = 1 shl 29;
  { Private  }
     AHIST_BW = 1 shl 30;
  { Sample types  }
  { Note that only AHIST_M8S, AHIST_S8S, AHIST_M16S and AHIST_S16S
     are supported by AHI_LoadSound().  }
  { Mono, 8 bit signed (BYTE)  }
     AHIST_M8S = 0;
  { Mono, 16 bit signed (WORD)  }
     AHIST_M16S = 1;
  { Stereo, 8 bit signed (2×BYTE)  }
     AHIST_S8S = 2;
  { Stereo, 16 bit signed (2×WORD)  }
     AHIST_S16S = 3;
  { Mono, 32 bit signed (LONG)  }
     AHIST_M32S = 8;
  { Stereo, 32 bit signed (2×LONG)  }
     AHIST_S32S = 10;
  { OBSOLETE!  }
     AHIST_M8U = 4;
  { Error codes  }
  { No error  }
     AHIE_OK = 0;
  { Out of memory  }
     AHIE_NOMEM = 1;
  { Unknown sound type  }
     AHIE_BADSOUNDTYPE = 2;
  { Unknown/unsupported sample type  }
     AHIE_BADSAMPLETYPE = 3;
  { User-triggered abortion  }
     AHIE_ABORTED = 4;
  { Error, but unknown  }
     AHIE_UNKNOWN = 5;
  { CMD_WRITE/CMD_READ failure  }
     AHIE_HALFDUPLEX = 6;
  { DEVICE INTERFACE DEFINITIONS FOLLOWS                                     }
  { Device units  }
     AHI_DEFAULT_UNIT = 0;
     AHI_NO_UNIT = 255;
  { The preference file  }
     ID_AHIU = $41484955;
     ID_AHIG = $41484947;


  type
     PAHIUnitPrefs = ^tAHIUnitPrefs;
     tAHIUnitPrefs = record
          ahiup_Unit : UBYTE;
          ahiup_Pad : UBYTE;
          ahiup_Channels : UWORD;
          ahiup_AudioMode : ULONG;
          ahiup_Frequency : ULONG;
          ahiup_MonitorVolume : Fixed;
          ahiup_InputGain : Fixed;
          ahiup_OutputVolume : Fixed;
          ahiup_Input : ULONG;
          ahiup_Output : ULONG;
       end;



     PAHIGlobalPrefs = ^tAHIGlobalPrefs;
     tAHIGlobalPrefs = record
          ahigp_DebugLevel : UWORD;       { Range: 0-3 (for None, Low,}
          ahigp_DisableSurround : BOOL;   { High and All)  }
          ahigp_DisableEcho : BOOL;
          ahigp_FastEcho : BOOL;
          ahigp_MaxCPU : Fixed;
          ahigp_ClipMasterVolume : BOOL;
       end;

  { Debug levels  }

  const
     AHI_DEBUG_NONE = 0;
     AHI_DEBUG_LOW = 1;
     AHI_DEBUG_HIGH = 2;
     AHI_DEBUG_ALL = 3;

  { AHIRequest  }
  type
     PAHIRequest = ^tAHIRequest;
     tAHIRequest = record
          ahir_Std : tIOStdReq;                  { Standard IO request  }
          ahir_Version : UWORD;                  { Needed version  }
          { --- New for V4, they will be ignored by V2 and earlier ---  }
          ahir_Pad1 : UWORD;
          ahir_Private : array[0..1] of ULONG;   { Hands off!  }
          ahir_Type : ULONG;                     { Sample format  }
          ahir_Frequency : ULONG;                { Sample/Record frequency  }
          ahir_Volume : Fixed;                   { Sample volume  }
          ahir_Position : Fixed;                 { Stereo position  }
          ahir_Link : PAHIRequest;               { For double buffering  }
       end;

  { Flags for OpenDevice()  }

  const
     AHIDF_NOMODESCAN = 1 shl 0;
     AHIDB_NOMODESCAN = 0;



FUNCTION AHI_AddAudioMode(a0arg : pTagItem) : longword;
FUNCTION AHI_AllocAudioA(tagList : pTagItem) : pAHIAudioCtrl;
FUNCTION AHI_AllocAudioRequestA(tagList : pTagItem) : pAHIAudioModeRequester;
FUNCTION AHI_AudioRequestA(Requester : pAHIAudioModeRequester; tagList : pTagItem) : BOOLEAN;
FUNCTION AHI_BestAudioIDA(tagList : pTagItem) : longword;
FUNCTION AHI_ControlAudioA(AudioCtrl : pAHIAudioCtrl; tagList : pTagItem) : longword;
PROCEDURE AHI_FreeAudio(AudioCtrl : pAHIAudioCtrl);
PROCEDURE AHI_FreeAudioRequest(Requester : pAHIAudioModeRequester);
FUNCTION AHI_GetAudioAttrsA(ID : longword; Audioctrl : pAHIAudioCtrl; tagList : pTagItem) : BOOLEAN;
PROCEDURE AHI_KillAudio;
FUNCTION AHI_LoadModeFile(a0arg : pCHAR) : longword;
FUNCTION AHI_LoadSound(Sound : WORD; _Type : longword; Info : POINTER; AudioCtrl : pAHIAudioCtrl) : longword;
FUNCTION AHI_NextAudioID(Last_ID : longword) : longword;
PROCEDURE AHI_PlayA(Audioctrl : pAHIAudioCtrl; tagList : pTagItem);
FUNCTION AHI_RemoveAudioMode(d0arg : longword) : longword;
FUNCTION AHI_SampleFrameSize(SampleType : longword) : longword;
FUNCTION AHI_SetEffect(Effect : POINTER; AudioCtrl : pAHIAudioCtrl) : longword;
PROCEDURE AHI_SetFreq(Channel : WORD; Freq : longword; AudioCtrl : pAHIAudioCtrl; Flags : longword);
PROCEDURE AHI_SetSound(Channel : WORD; Sound : WORD; Offset : longword; len : LONGINT; AudioCtrl : pAHIAudioCtrl; Flags : longword);
PROCEDURE AHI_SetVol(Channel : WORD; Volume : LONGINT; Pan : LONGINT; AudioCtrl : pAHIAudioCtrl; Flags : longword);
PROCEDURE AHI_UnloadSound(Sound : WORD; Audioctrl : pAHIAudioCtrl);
{
 Functions and procedures with array of const go here
}
FUNCTION AHI_AllocAudio(const tagList : Array Of Const) : pAHIAudioCtrl;
FUNCTION AHI_AllocAudioRequest(const tagList : Array Of Const) : pAHIAudioModeRequester;
FUNCTION AHI_AudioRequest(Requester : pAHIAudioModeRequester; const tagList : Array Of Const) : BOOLEAN;
FUNCTION AHI_BestAudioID(const tagList : Array Of Const) : longword;
FUNCTION AHI_ControlAudio(AudioCtrl : pAHIAudioCtrl; const tagList : Array Of Const) : longword;
FUNCTION AHI_GetAudioAttrs(ID : longword; Audioctrl : pAHIAudioCtrl; const tagList : Array Of Const) : BOOLEAN;
PROCEDURE AHI_Play(Audioctrl : pAHIAudioCtrl; const tagList : Array Of Const);

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitAHILibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    AHIIsCompiledHow : longint;
IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray;

FUNCTION AHI_AddAudioMode(a0arg : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L a0arg,A0
        MOVEA.L AHIBase,A6
        JSR     -150(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHI_AllocAudioA(tagList : pTagItem) : pAHIAudioCtrl;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L tagList,A1
        MOVEA.L AHIBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHI_AllocAudioRequestA(tagList : pTagItem) : pAHIAudioModeRequester;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L tagList,A0
        MOVEA.L AHIBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHI_AudioRequestA(Requester : pAHIAudioModeRequester; tagList : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Requester,A0
        MOVEA.L tagList,A1
        MOVEA.L AHIBase,A6
        JSR     -126(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AHI_BestAudioIDA(tagList : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L tagList,A1
        MOVEA.L AHIBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHI_ControlAudioA(AudioCtrl : pAHIAudioCtrl; tagList : pTagItem) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L AudioCtrl,A2
        MOVEA.L tagList,A1
        MOVEA.L AHIBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE AHI_FreeAudio(AudioCtrl : pAHIAudioCtrl);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AHI_FreeAudioRequest(Requester : pAHIAudioModeRequester);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Requester,A0
        MOVEA.L AHIBase,A6
        JSR     -132(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AHI_GetAudioAttrsA(ID : longword; Audioctrl : pAHIAudioCtrl; tagList : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  ID,D0
        MOVEA.L Audioctrl,A2
        MOVEA.L tagList,A1
        MOVEA.L AHIBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE AHI_KillAudio;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L AHIBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AHI_LoadModeFile(a0arg : pCHAR) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L a0arg,A0
        MOVEA.L AHIBase,A6
        JSR     -162(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHI_LoadSound(Sound : WORD; _Type : longword; Info : POINTER; AudioCtrl : pAHIAudioCtrl) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Sound,D0
        MOVE.L  _Type,D1
        MOVEA.L Info,A0
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHI_NextAudioID(Last_ID : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Last_ID,D0
        MOVEA.L AHIBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE AHI_PlayA(Audioctrl : pAHIAudioCtrl; tagList : pTagItem);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Audioctrl,A2
        MOVEA.L tagList,A1
        MOVEA.L AHIBase,A6
        JSR     -138(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AHI_RemoveAudioMode(d0arg : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  d0arg,D0
        MOVEA.L AHIBase,A6
        JSR     -156(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHI_SampleFrameSize(SampleType : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  SampleType,D0
        MOVEA.L AHIBase,A6
        JSR     -144(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AHI_SetEffect(Effect : POINTER; AudioCtrl : pAHIAudioCtrl) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Effect,A0
        MOVEA.L AudioCtrl,A2
        MOVEA.L AHIBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE AHI_SetFreq(Channel : WORD; Freq : longword; AudioCtrl : pAHIAudioCtrl; Flags : longword);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Channel,D0
        MOVE.L  Freq,D1
        MOVEA.L AudioCtrl,A2
        MOVE.L  Flags,D2
        MOVEA.L AHIBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AHI_SetSound(Channel : WORD; Sound : WORD; Offset : longword; len : LONGINT; AudioCtrl : pAHIAudioCtrl; Flags : longword);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Channel,D0
        MOVE.L  Sound,D1
        MOVE.L  Offset,D2
        MOVE.L  len,D3
        MOVEA.L AudioCtrl,A2
        MOVE.L  Flags,D4
        MOVEA.L AHIBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AHI_SetVol(Channel : WORD; Volume : LONGINT; Pan : LONGINT; AudioCtrl : pAHIAudioCtrl; Flags : longword);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Channel,D0
        MOVE.L  Volume,D1
        MOVE.L  Pan,D2
        MOVEA.L AudioCtrl,A2
        MOVE.L  Flags,D3
        MOVEA.L AHIBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE AHI_UnloadSound(Sound : WORD; Audioctrl : pAHIAudioCtrl);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Sound,D0
        MOVEA.L Audioctrl,A2
        MOVEA.L AHIBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
  END;
END;

{
 Functions and procedures with array of const go here
}
FUNCTION AHI_AllocAudio(const tagList : Array Of Const) : pAHIAudioCtrl;
begin
    AHI_AllocAudio := AHI_AllocAudioA(readintags(tagList));
end;

FUNCTION AHI_AllocAudioRequest(const tagList : Array Of Const) : pAHIAudioModeRequester;
begin
    AHI_AllocAudioRequest := AHI_AllocAudioRequestA(readintags(tagList));
end;

FUNCTION AHI_AudioRequest(Requester : pAHIAudioModeRequester; const tagList : Array Of Const) : BOOLEAN;
begin
    AHI_AudioRequest := AHI_AudioRequestA(Requester , readintags(tagList));
end;

FUNCTION AHI_BestAudioID(const tagList : Array Of Const) : longword;
begin
    AHI_BestAudioID := AHI_BestAudioIDA(readintags(tagList));
end;

FUNCTION AHI_ControlAudio(AudioCtrl : pAHIAudioCtrl; const tagList : Array Of Const) : longword;
begin
    AHI_ControlAudio := AHI_ControlAudioA(AudioCtrl , readintags(tagList));
end;

FUNCTION AHI_GetAudioAttrs(ID : longword; Audioctrl : pAHIAudioCtrl; const tagList : Array Of Const) : BOOLEAN;
begin
    AHI_GetAudioAttrs := AHI_GetAudioAttrsA(ID , Audioctrl , readintags(tagList));
end;

PROCEDURE AHI_Play(Audioctrl : pAHIAudioCtrl; const tagList : Array Of Const);
begin
    AHI_PlayA(Audioctrl , readintags(tagList));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of ahi.library}
  {$Info don't forget to use InitAHILibrary in the beginning of your program}

var
    ahi_exit : Pointer;

procedure CloseahiLibrary;
begin
    ExitProc := ahi_exit;
    if AHIBase <> nil then begin
        CloseLibrary(AHIBase);
        AHIBase := nil;
    end;
end;

procedure InitAHILibrary;
begin
    AHIBase := nil;
    AHIBase := OpenLibrary(AHINAME,LIBVERSION);
    if AHIBase <> nil then begin
        ahi_exit := ExitProc;
        ExitProc := @CloseahiLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open ahi.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    AHIIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of ahi.library}

var
    ahi_exit : Pointer;

procedure CloseahiLibrary;
begin
    ExitProc := ahi_exit;
    if AHIBase <> nil then begin
        CloseLibrary(AHIBase);
        AHIBase := nil;
    end;
end;

begin
    AHIBase := nil;
    AHIBase := OpenLibrary(AHINAME,LIBVERSION);
    if AHIBase <> nil then begin
        ahi_exit := ExitProc;
        ExitProc := @CloseahiLibrary;
        AHIIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open ahi.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    AHIIsCompiledHow := 3;
   {$Warning No autoopening of ahi.library compiled}
   {$Warning Make sure you open ahi.library yourself}
{$endif dont_use_openlib}

END. (* UNIT AHI *)



