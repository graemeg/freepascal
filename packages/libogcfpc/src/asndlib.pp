unit asndlib;
{$mode objfpc} 
{$J+}
{$INLINE ON}
{$MACRO ON}
{$PACKRECORDS C}
{$ASSERTIONS ON}

interface

uses
  ctypes, gctypes;

const
  ASND_LIB = $100;
  SND_LIB = ( ASND_LIB + 2 );
  SND_OK = 0;
  SND_INVALID = - 1;
  SND_ISNOTASONGVOICE = - 2;
  SND_BUSY = 1;
  SND_UNUSED = 0;  (*!< This voice is available for use.  *)
  SND_WORKING = 1;  (*!< This voice is currently in progress.  *)
  SND_WAITING = 2;  (*!< This voice is currently in progress and waiting to one SND_AddVoice() function (the voice handler is called continuously)  *)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  VOICE_MONO_8BIT       = 0;
  VOICE_MONO_16BIT      = 1;
  VOICE_MONO_16BIT_BE   = 1;
  VOICE_STEREO_8BIT     = 2;
  VOICE_STEREO_16BIT    = 3;
  VOICE_STEREO_16BIT_BE = 3;
  VOICE_MONO_8BIT_U     = 4;
  VOICE_MONO_16BIT_LE   = 5;
  VOICE_STEREO_8BIT_U   = 6;
  VOICE_STEREO_16BIT_LE = 7;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  VOICE_MONO_8BIT = 0;
  VOICE_MONO_16BIT = 1;
  VOICE_STEREO_8BIT = 2;
  VOICE_STEREO_16BIT = 3;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  MIN_VOLUME = 0;
  MID_VOLUME = 127;
  MAX_VOLUME = 255;
  MIN_PITCH = 1;  (*!< 1Hz  *)
  F44100HZ_PITCH = 44100;  (*!< 44100Hz  *)
  MAX_PITCH = 144000;  (*!< 144000Hz (more process time for pitch>48000)  *)

  NOTE_DO = 0;
  NOTE_DOs = 1;
  NOTE_REb = NOTE_DOs;
  NOTE_RE = NOTE_REb + 1;
  NOTE_REs = NOTE_RE + 1;
  NOTE_MIb = NOTE_REs;
  NOTE_MI = NOTE_MIb + 1;
  NOTE_FA = NOTE_MI + 1;
  NOTE_FAs = NOTE_FA + 1;
  NOTE_SOLb = NOTE_FAs;
  NOTE_SOL = NOTE_SOLb + 1;
  NOTE_SOLs = NOTE_SOL + 1;
  NOTE_LAb = NOTE_SOLs;
  NOTE_LA = NOTE_LAb + 1;
  NOTE_LAs = NOTE_LA + 1;
  NOTE_SIb = NOTE_LAs;
  NOTE_SI = NOTE_SIb + 1;

  NOTE_C = 0;
  NOTE_Cs = 1;
  NOTE_Db = NOTE_Cs;
  NOTE_D = NOTE_Db + 1;
  NOTE_Ds = NOTE_D + 1;
  NOTE_Eb = NOTE_Ds;
  NOTE_E = NOTE_Eb + 1;
  NOTE_F = NOTE_E + 1;
  NOTE_Fs = NOTE_F + 1;
  NOTE_Gb = NOTE_Fs;
  NOTE_G = NOTE_Gb + 1;
  NOTE_Gs = NOTE_G + 1;
  NOTE_Ab = NOTE_Gs;
  NOTE_A = NOTE_Ab + 1;
  NOTE_As = NOTE_A + 1;
  NOTE_Bb = NOTE_As;
  NOTE_B = NOTE_Bb + 1;


function _NOTE(note, octave: cint): cint; inline;

{$define Note2Freq               := ANote2Freq}
{$define SND_Init                := ASND_Init}
{$define SND_End                 := ASND_End}
{$define SND_Pause               := ASND_Pause}
{$define SND_Is_Paused           := ASND_Is_Paused}
{$define SND_GetTime             := ASND_GetTime}
{$define SND_GetSampleCounter    := ASND_GetSampleCounter}
{$define SND_GetSamplesPerTick   := ASND_GetSamplesPerTick}
{$define SND_SetTime             := ASND_SetTime}
{$define SND_SetCallback         := ASND_SetCallback}
{$define SND_GetAudioRate        := ASND_GetAudioRate}
{$define SND_SetVoice            := ASND_SetVoice}
{$define SND_AddVoice            := ASND_AddVoice}
{$define SND_StopVoice           := ASND_StopVoice}
{$define SND_PauseVoice          := ASND_PauseVoice}
{$define SND_StatusVoice         := ASND_StatusVoice}
{$define SND_GetFirstUnusedVoice := ASND_GetFirstUnusedVoice}
{$define SND_ChangePitchVoice    := ASND_ChangePitchVoice}
{$define SND_ChangeVolumeVoice   := ASND_ChangeVolumeVoice}
{$define SND_ChangeVolumeVoice   := ASND_ChangeVolumeVoice}
{$define SND_GetTickCounterVoice := ASND_GetTickCounterVoice}
{$define SND_GetTimerVoice       := ASND_GetTimerVoice}
{$define SND_TestPointer         := ASND_TestPointer}



type
  ASNDVoiceCallback = procedure(voice: cint32); cdecl;

function ANote2Freq(note, freq_base, note_base: cint): cint; cdecl; external;
procedure ASND_Init; cdecl; external;
procedure ASND_End; cdecl; external;
procedure ASND_Pause(paused: cint32); cdecl; external;
function ASND_Is_Paused: cint32; cdecl; external;
function ASND_GetTime: cuint32; cdecl; external;
function ASND_GetSampleCounter: cuint32; cdecl; external;
function ASND_GetSamplesPerTick: cuint32; cdecl; external;
procedure ASND_SetTime(time: cuint32); cdecl; external;

type
  TAuxCallback0 = procedure; cdecl;
procedure ASND_SetCallback(callback: TAuxCallback0); cdecl; external;
function ASND_GetAudioRate: cint32; cdecl; external;
function ASND_SetVoice(voice, format, pitch, delay: cint32; snd: pointer; size_snd, volume_l, volume_r: cint32; callback: ASNDVoiceCallback): cint32; cdecl; external;
function ASND_SetInfiniteVoice(voice, format, pitch, delay: cint32; snd: pointer; size_snd, volume_l, volume_r: cint32): cint32; cdecl; external;
function ASND_AddVoice(voice: cint32; snd: pointer; size_snd: cint32): cint32; cdecl; external;
function ASND_StopVoice(voice: cint32): cint32; cdecl; external;
function ASND_PauseVoice(voice, pause: cint32): cint32; cdecl; external;
function ASND_StatusVoice(voice: cint32): cint32; cdecl; external;
function ASND_GetFirstUnusedVoice: cint32; cdecl; external;
function ASND_ChangePitchVoice(voice, pitch: cint32): cint32; cdecl; external;
function ASND_ChangeVolumeVoice(voice, volume_l, volume_r: cint32): cint32; cdecl; external;
function ASND_GetTickCounterVoice(voice: cint32): cuint32; cdecl; external;
function ASND_GetTimerVoice(voice: cint32): cuint32; cdecl; external;
function ASND_TestPointer(voice: cint32; pointer: pointer): cint32; cdecl; external;
function ASND_TestVoiceBufferReady(voice: cint32): cint32; cdecl; external;
function ASND_GetDSP_PercentUse: cuint32; cdecl; external;
function ASND_GetDSP_ProcessTime: cuint32; cdecl; external;

implementation

function _NOTE(note, octave: cint): cint; inline;
begin
  _NOTE := note + (octave shl 3) + (octave shl 2);
end;

initialization

{$linklib asnd}

end.
