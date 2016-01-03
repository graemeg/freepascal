unit aesndlib;
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
  MAX_VOICES = 32;
  SND_BUFFERSIZE = 384;  // output 2ms sound data at 48KHz
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef HW_DOL}
  DSP_DEFAULT_FREQ = 48044;
{$else}
	DSP_DEFAULT_FREQ = 48000;
{$endif}
=======
  DSP_DEFAULT_FREQ = 48000;
>>>>>>> graemeg/cpstrnew
=======
  DSP_DEFAULT_FREQ = 48000;
>>>>>>> graemeg/cpstrnew
=======
  DSP_DEFAULT_FREQ = 48000;
>>>>>>> graemeg/cpstrnew
=======
  DSP_DEFAULT_FREQ = 48000;
>>>>>>> origin/cpstrnew
  VOICE_STATE_STOPPED = 0;
  VOICE_STATE_RUNNING = 1;
  VOICE_STATE_STREAM = 2;
  VOICE_MONO8 = $00000000;
  VOICE_STEREO8 = $00000001;
  VOICE_MONO16 = $00000002;
  VOICE_STEREO16 = $00000003;
  VOICE_FREQ32KHZ = 32000;
  VOICE_FREQ48KHZ = 48000;

type
  aesndpb_t = record
  end;
  AESNDPB = aesndpb_t;
  PAESNDPB = ^AESNDPB;

  AESNDVoiceCallback = procedure(pb: PAESNDPB; state: cuint32); cdecl;
  AESNDAudioCallback = procedure(audio_buffer: pointer; len: cuint32); cdecl;


procedure AESND_Init; cdecl; external;
procedure AESND_Reset; cdecl; external;
procedure AESND_Pause(pause: cbool); cdecl; external;
function AESND_GetDSPProcessTime: cuint32; cdecl; external;
function AESND_GetDSPProcessUsage: f32; cdecl; external;
function AESND_RegisterAudioCallback(cb: AESNDAudioCallback): AESNDAudioCallback; cdecl; external;
function AESND_AllocateVoice(cb: AESNDVoiceCallback): PAESNDPB; cdecl; external;
procedure AESND_FreeVoice(pb: PAESNDPB); cdecl; external;
procedure AESND_SetVoiceStop(pb: PAESNDPB; stop: cbool); cdecl; external;
procedure AESND_SetVoiceMute(pb: PAESNDPB; mute: cbool); cdecl; external;
procedure AESND_SetVoiceLoop(pb: PAESNDPB; loop: cbool); cdecl; external;
procedure AESND_SetVoiceFormat(pb: PAESNDPB; format: cuint32); cdecl; external;
procedure AESND_SetVoiceStream(pb: PAESNDPB; stream: cbool); cdecl; external;
procedure AESND_SetVoiceFrequency(pb: PAESNDPB; freq: cuint32); cdecl; external;
procedure AESND_SetVoiceVolume(pb: PAESNDPB; volume_l, volume_r: cuint16); cdecl; external;
procedure AESND_SetVoiceBuffer(pb: PAESNDPB; buffer: pointer; len: cuint32); cdecl; external;
procedure AESND_PlayVoice(pb: PAESNDPB; format: cuint32; buffer: pointer; len, freq, delay: cuint32; looped: cbool); cdecl; external;
function AESND_RegisterVoiceCallback(pb: PAESNDPB; cb: AESNDVoiceCallback): AESNDVoiceCallback; cdecl; external;

implementation

initialization

{$linklib aesnd}

end.
