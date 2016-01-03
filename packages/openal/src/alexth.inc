const

(* format base = $10000 *)
  AL_FORMAT_IMA_ADPCM_MONO16_EXT            = $10000;
  AL_FORMAT_IMA_ADPCM_STEREO16_EXT          = $10001;
  AL_FORMAT_WAVE_EXT                        = $10002;
  AL_FORMAT_VORBIS_EXT                      = $10003;

(* four point formats *)
  AL_FORMAT_QUAD8_LOKI                      = $10004;
  AL_FORMAT_QUAD16_LOKI                     = $10005;

(**
 * token extensions, base = $20000
 *)

(* deprecated, use AL_GAIN *)
  AL_GAIN_LINEAR_LOKI                      = $20000;

(*
 * types for special loaders.  This should be deprecated in favor
 * of the special format tags.
 *)
type
  PWaveFMT                          = ^WaveFMT;
  WaveFMT                           = packed record
    encoding        : ALushort;
    channels        : ALushort;      (* 1 = mono, 2 = stereo *)
    frequency       : ALuint;        (* One of 11025, 22050, or 44100 Hz *)
    byterate        : ALuint;        (* Average bytes per second *)
    blockalign      : ALushort;      (* Bytes per sample block *)
    bitspersample   : ALushort;
  end;

  PalWaveFMT_LOKI                   = ^alWaveFMT_LOKI;
  alWaveFMT_LOKI                    = WaveFMT;

  P_MS_ADPCM_decodestate            = ^_MS_ADPCM_decodestate;
  _MS_ADPCM_decodestate             = packed record
    hPredictor      : ALubyte;
    iDelta          : ALushort;
    iSamp1          : ALshort;
    iSamp2          : ALshort;
  end;

  PalMSADPCM_decodestate_LOKI       = ^alMSADPCM_decodestate_LOKI;
  alMSADPCM_decodestate_LOKI        = _MS_ADPCM_decodestate;

  PMS_ADPCM_decoder                 = ^MS_ADPCM_decoder;
  MS_ADPCM_decoder                  = packed record
    wavefmt         : alWaveFMT_LOKI;
    wSamplesPerBlock: ALushort;
    wNumCoef        : ALushort;
    aCoeff          : array[0..6] of array[0..1] of ALshort;
    (* * * *)
    state           : array[0..1] of alMSADPCM_decodestate_LOKI;
  end;

  PalMSADPCM_state_LOKI             = ^alMSADPCM_state_LOKI;
  alMSADPCM_state_LOKI              = MS_ADPCM_decoder;

  PIMA_ADPCM_decodestate_s          = ^IMA_ADPCM_decodestate_s;
  IMA_ADPCM_decodestate_s           = packed record
    valprev         : ALint;      (* Previous output value *)
    index           : ALbyte;       (* Index into stepsize table *)
  end;

  PalIMAADPCM_decodestate_LOKI      = ^alIMAADPCM_decodestate_LOKI;
  alIMAADPCM_decodestate_LOKI       = IMA_ADPCM_decodestate_s;

  PIMA_ADPCM_decoder                = ^IMA_ADPCM_decoder;
  IMA_ADPCM_decoder                 = packed record
    wavefmt         : alWaveFMT_LOKI;
    wSamplesPerBlock: ALushort;
    (* * * *)
    state           : array[0..1] of alIMAADPCM_decodestate_LOKI;
  end;

  PalIMAADPCM_state_LOKI            = ^alIMAADPCM_state_LOKI;
  alIMAADPCM_state_LOKI             = IMA_ADPCM_decoder;


const
(**
 * Context creation extension tokens
 * base = $200000
 *)

(**
 * followed by ### of sources
 *)
  ALC_SOURCES_LOKI                         = $200000;

(**
 * followed by ### of buffers
 *)
  ALC_BUFFERS_LOKI                         = $200001;

(*
 *  Channel operations are probably a big no-no and destined
 *  for obsolesence.
 *
 *  base = $300000
 *)
  ALC_CHAN_MAIN_LOKI                       = $300000;
  ALC_CHAN_PCM_LOKI                        = $300001;
  ALC_CHAN_CD_LOKI                         = $300002;

(* loki *)
type
  CALLBACK_LOKI = function(a: ALuint; b: ALuint; c: PALshort; d: ALEnum; e: ALint; f: ALint): cint; cdecl;

function alcGetAudioChannel_LOKI(channel: ALuint): ALfloat; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
procedure alcSetAudioChannel_LOKI(channel: ALuint; volume: ALfloat); cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
procedure alBombOnError_LOKI; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
procedure alBufferi_LOKI(bid: ALuint; param: ALenum; value: ALint); cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
procedure alBufferDataWithCallback_LOKI(bid: ALuint; cb: CALLBACK_LOKI); cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
procedure alBufferWriteData_LOKI(buffer: ALuint; format: ALenum; data: PALvoid; size: ALsizei; freq: ALsizei; internalFormat: ALenum); cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
procedure alGenStreamingBuffers_LOKI(n: ALsizei; samples: PALuint); cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
function alBufferAppendData_LOKI(buffer: ALuint; format: ALenum; data: PALvoid; size: ALsizei; freq: ALsizei): ALsizei; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
function alBufferAppendWriteData_LOKI(buffer: ALuint; format: ALenum; data: PALvoid; size: ALsizei; freq: ALsizei; internalFormat: ALenum): ALsizei; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};

(* binary compatibility *)
function alBufferAppendData(buffer: ALuint; format: ALenum; data: PALvoid; size: ALsizei; freq: ALsizei): ALsizei; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};

(*
 * Don't use these.  If you're reading this, you should remove these functions
 * and all other reverb functions.  Now.
 *)
procedure alReverbScale_LOKI(sid: ALuint; param: ALfloat); cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
procedure alReverbDelay_LOKI(sid: ALuint; param: ALfloat); cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};

(* Capture api *)

function alCaptureInit_EXT(format: ALenum; rate: ALuint; bufferSize: ALsizei): ALboolean; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
function alCaptureDestroy_EXT: ALboolean; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
function alCaptureStart_EXT: ALboolean; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
function alCaptureStop_EXT: ALboolean; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};

(* Non-blocking device read *)

function alCaptureGetData_EXT(data: PALvoid; n: ALsizei; format: ALenum; rate: ALuint): ALsizei; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};

(* custom loaders *)

function alutLoadVorbis_LOKI(bid: ALuint; const data: PALvoid; size: ALint): ALboolean; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};
function alutLoadMP3_LOKI(bid: ALuint; data: PALvoid; size: ALint): ALboolean; cdecl; external {$IFDEF DYNLINK}openallib{$ENDIF};

(* function pointers *)
{
type
typedef ALfloat   ( ALC_APIENTRY *PFNALCGETAUDIOCHANNELPROC ) ( ALuint channel ): ALfloat; cdecl;
typedef void      ( ALC_APIENTRY *PFNALCSETAUDIOCHANNELPROC ) ( ALuint channel, ALfloat volume ); cdecl;
typedef void      ( AL_APIENTRY *PFNALBOMBONERRORPROC ) ( void ); cdecl;
typedef void      ( AL_APIENTRY *PFNALBUFFERIPROC ) ( ALuint bid, ALenum param, ALint value ); cdecl;
typedef void      ( AL_APIENTRY *PFNALBUFFERDATAWITHCALLBACKPROC ) ( ALuint bid, int ( *Callback ) ( ALuint, ALuint, ALshort *, ALenum, ALint, ALint ) ); cdecl;
typedef void      ( AL_APIENTRY *PFNALBUFFERWRITEDATAPROC ) ( ALuint buffer, ALenum format, ALvoid *data, ALsizei size, ALsizei freq, ALenum internalFormat ); cdecl;
typedef void      ( AL_APIENTRY *PFNALGENSTREAMINGBUFFERSPROC ) ( ALsizei n, ALuint *samples ); cdecl;
typedef ALsizei   ( AL_APIENTRY *PFNALBUFFERAPPENDDATAPROC ) ( ALuint buffer, ALenum format, ALvoid *data, ALsizei size, ALsizei freq ): ALsizei; cdecl;
typedef ALsizei   ( AL_APIENTRY *PFNALBUFFERAPPENDWRITEDATAPROC ) ( ALuint buffer, ALenum format, ALvoid *data, ALsizei size, ALsizei freq, ALenum internalFormat ): ALsizei; cdecl;

typedef ALboolean ( ALC_APIENTRY *PFNALCAPTUREINITPROC ) ( ALenum format, ALuint rate, ALsizei bufferSize ): ALboolean; cdecl;
typedef ALboolean ( ALC_APIENTRY *PFNALCAPTUREDESTROYPROC ) ( void ): ALboolean; cdecl;
typedef ALboolean ( ALC_APIENTRY *PFNALCAPTURESTARTPROC ) ( void ): ALboolean; cdecl;
typedef ALboolean ( ALC_APIENTRY *PFNALCAPTURESTOPPROC ) ( void ): ALboolean; cdecl;

typedef ALsizei   ( ALC_APIENTRY *PFNALCAPTUREGETDATAPROC ) ( ALvoid *data, ALsizei n, ALenum format, ALuint rate ): ALsizei; cdecl;

typedef ALboolean ( AL_APIENTRY *PFNALUTLOADVORBISPROC ) ( ALuint bid, ALvoid *data, ALint size ): ALboolean; cdecl;
typedef ALboolean ( AL_APIENTRY *PFNALUTLOADRAW_ADPCMDATAPROC ) ( ALuint bid, ALvoid *data, ALuint size, ALuint freq, ALenum format ): ALboolean; cdecl;
typedef ALboolean ( AL_APIENTRY *ALUTLOADIMA_ADPCMDATAPROC ) ( ALuint bid, ALvoid *data, ALuint size, alIMAADPCM_state_LOKI *ias ): ALboolean; cdecl;
typedef ALboolean ( AL_APIENTRY *ALUTLOADMS_ADPCMDATAPROC ) ( ALuint bid, void *data, int size, alMSADPCM_state_LOKI *mss ): ALboolean; cdecl;

typedef void      ( AL_APIENTRY *PFNALREVERBSCALEPROC ) ( ALuint sid, ALfloat param ); cdecl;
typedef void      ( AL_APIENTRY *PFNALREVERBDELAYPROC ) ( ALuint sid, ALfloat param ); cdecl;
}