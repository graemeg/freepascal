unit sdl_mixer;
{******************************************************************************}
{
  $Id: sdl_mixer.pas,v 1.18 2007/05/29 21:31:44 savage Exp $
  
}
{                                                                              }
{       Borland Delphi SDL_Mixer - Simple DirectMedia Layer Mixer Library      }
{       Conversion of the Simple DirectMedia Layer Headers                     }
{                                                                              }
{ Portions created by Sam Lantinga <slouken@devolution.com> are                }
{ Copyright (C) 1997, 1998, 1999, 2000, 2001  Sam Lantinga                     }
{ 5635-34 Springhouse Dr.                                                      }
{ Pleasanton, CA 94588 (USA)                                                   }
{                                                                              }
{ All Rights Reserved.                                                         }
{                                                                              }
{ The original files are : SDL_mixer.h                                         }
{                          music_cmd.h                                         }
{                          wavestream.h                                        }
{                          timidity.h                                          }
{                          playmidi.h                                          }
{                          music_ogg.h                                         }
{                          mikmod.h                                            }
{                                                                              }
{ The initial developer of this Pascal code was :                              }
{ Dominqiue Louis <Dominique@SavageSoftware.com.au>                            }
{                                                                              }
{ Portions created by Dominqiue Louis are                                      }
{ Copyright (C) 2000 - 2001 Dominqiue Louis.                                   }
{                                                                              }
{                                                                              }
{ Contributor(s)                                                               }
{ --------------                                                               }
{ Matthias Thoma <ma.thoma@gmx.de>                                             }
{                                                                              }
{ Obtained through:                                                            }
{ Joint Endeavour of Delphi Innovators ( Project JEDI )                        }
{                                                                              }
{ You may retrieve the latest version of this file at the Project              }
{ JEDI home page, located at http://delphi-jedi.org                            }
{                                                                              }
{ The contents of this file are used with permission, subject to               }
{ the Mozilla Public License Version 1.1 (the "License"); you may              }
{ not use this file except in compliance with the License. You may             }
{ obtain a copy of the License at                                              }
{ http://www.mozilla.org/MPL/MPL-1.1.html                                                         }
{                                                                              }
{ Software distributed under the License is distributed on an                  }
{ "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or               }
{ implied. See the License for the specific language governing                 }
{ rights and limitations under the License.                                    }
{                                                                              }
{ Description                                                                  }
{ -----------                                                                  }
{                                                                              }
{                                                                              }
{                                                                              }
{                                                                              }
{                                                                              }
{                                                                              }
{                                                                              }
{ Requires                                                                     }
{ --------                                                                     }
{   SDL.pas & SMPEG.pas somewhere within your search path.                     }
{                                                                              }
{ Programming Notes                                                            }
{ -----------------                                                            }
{   See the Aliens Demo to see how this library is used                        }
{                                                                              }
{ Revision History                                                             }
{ ----------------                                                             }
{   April    02 2001 - DL : Initial Translation                                }
{                                                                              }
{  February  02 2002 - DL : Update to version 1.2.1                            }
{                                                                              }
{   April   03 2003 - DL : Added jedi-sdl.inc include file to support more     }
{                          Pascal compilers. Initial support is now included   }
{                          for GnuPascal, VirtualPascal, TMT and obviously     }
{                          continue support for Delphi Kylix and FreePascal.   }
{                                                                              }
{   April   24 2003 - DL : under instruction from Alexey Barkovoy, I have added}
{                          better TMT Pascal support and under instruction     }
{                          from Prof. Abimbola Olowofoyeku (The African Chief),}
{                          I have added better Gnu Pascal support              }
{                                                                              }
{   April   30 2003 - DL : under instruction from David Mears AKA              }
{                          Jason Siletto, I have added FPC Linux support.      }
{                          This was compiled with fpc 1.1, so remember to set  }
{                          include file path. ie. -Fi/usr/share/fpcsrc/rtl/*   }
{                                                                              }
{
  $Log: sdl_mixer.pas,v $
  Revision 1.18  2007/05/29 21:31:44  savage
  Changes as suggested by Almindor for 64bit compatibility.

  Revision 1.17  2007/05/20 20:31:17  savage
  Initial Changes to Handle 64 Bits

  Revision 1.16  2006/12/02 00:16:17  savage
  Updated to latest version

  Revision 1.15  2005/04/10 11:48:33  savage
  Changes as suggested by Michalis, thanks.

  Revision 1.14  2005/02/24 20:20:07  savage
  Changed definition of MusicType and added GetMusicType function

  Revision 1.13  2005/01/05 01:47:09  savage
  Changed LibName to reflect what MacOS X should have. ie libSDL*-1.2.0.dylib respectively.

  Revision 1.12  2005/01/04 23:14:56  savage
  Changed LibName to reflect what most Linux distros will have. ie libSDL*-1.2.so.0 respectively.

  Revision 1.11  2005/01/01 02:05:19  savage
  Updated to v1.2.6

  Revision 1.10  2004/09/12 21:45:17  savage
  Robert Reed spotted that Mix_SetMusicPosition was missing from the conversion, so this has now been added.

  Revision 1.9  2004/08/27 21:48:24  savage
  IFDEFed out Smpeg support on MacOS X

  Revision 1.8  2004/08/14 22:54:30  savage
  Updated so that Library name defines are correctly defined for MacOS X.

  Revision 1.7  2004/05/10 14:10:04  savage
  Initial MacOS X support. Fixed defines for MACOS ( Classic ) and DARWIN ( MacOS X ).

  Revision 1.6  2004/04/13 09:32:08  savage
  Changed Shared object names back to just the .so extension to avoid conflicts on various Linux/Unix distros. Therefore developers will need to create Symbolic links to the actual Share Objects if necessary.

  Revision 1.5  2004/04/01 20:53:23  savage
  Changed Linux Shared Object names so they reflect the Symbolic Links that are created when installing the RPMs from the SDL site.

  Revision 1.4  2004/03/31 22:20:02  savage
  Windows unit not used in this file, so it was removed to keep the code tidy.

  Revision 1.3  2004/03/31 10:05:08  savage
  Better defines for Endianess under FreePascal and Borland compilers.

  Revision 1.2  2004/03/30 20:23:28  savage
  Tidied up use of UNIX compiler directive.

  Revision 1.1  2004/02/14 23:35:42  savage
  version 1 of sdl_image, sdl_mixer and smpeg.


}
{******************************************************************************}

{$I jedi-sdl.inc}

interface

uses
{$IFDEF __GPC__}
  gpc,
{$ENDIF}
{$IFNDEF DARWIN}
{$IFNDEF no_smpeg}
  smpeg,
{$ENDIF}
{$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF MORPHOS}
  exec,
{$ENDIF}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  sdl;

const
{$IFDEF WINDOWS}
  SDL_MixerLibName = 'SDL_mixer.dll';
{$ENDIF}

{$IFDEF UNIX}
{$IFDEF DARWIN}
  SDL_MixerLibName = 'libSDL_mixer-1.2.0.dylib';
{$ELSE}
  {$IFDEF FPC}
    SDL_MixerLibName = 'libSDL_mixer.so';
  {$ELSE}
    SDL_MixerLibName = 'libSDL_mixer-1.2.so.0';
  {$ENDIF}
{$ENDIF}
{$ENDIF}

{$IFDEF MACOS}
  SDL_MixerLibName = 'SDL_mixer';
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF MACOS}
  SDL_MixerLibName = 'powersdl_mixer.library';
{$ENDIF}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {* Printable format: "%d.%d.%d", MAJOR, MINOR, PATCHLEVEL *}
  SDL_MIXER_MAJOR_VERSION = 1;
{$EXTERNALSYM MIX_MAJOR_VERSION}
  SDL_MIXER_MINOR_VERSION = 2;
{$EXTERNALSYM MIX_MINOR_VERSION}
  SDL_MIXER_PATCHLEVEL    = 7;
{$EXTERNALSYM MIX_PATCHLEVEL}

  // Backwards compatibility
  MIX_MAJOR_VERSION = SDL_MIXER_MAJOR_VERSION;
  MIX_MINOR_VERSION = SDL_MIXER_MINOR_VERSION;
  MIX_PATCHLEVEL    = SDL_MIXER_PATCHLEVEL;
  
  // SDL_Mixer.h constants
  { The default mixer has 8 simultaneous mixing channels }
{$IFNDEF MIX_CHANNELS}
  MIX_CHANNELS = 8;
{$ENDIF}
{$EXTERNALSYM MIX_CHANNELS}
  { Good default values for a PC soundcard }
  MIX_DEFAULT_FREQUENCY = 22050;
{$EXTERNALSYM MIX_DEFAULT_FREQUENCY}

{$IFDEF IA32}
  MIX_DEFAULT_FORMAT = AUDIO_S16LSB;
{$ELSE}
  MIX_DEFAULT_FORMAT = AUDIO_S16MSB;
{$ENDIF}
{$EXTERNALSYM MIX_DEFAULT_FORMAT}

  MIX_DEFAULT_CHANNELS = 2;
{$EXTERNALSYM MIX_DEFAULT_CHANNELS}
  MIX_MAX_VOLUME = 128; { Volume of a chunk }
{$EXTERNALSYM MIX_MAX_VOLUME}

  PATH_MAX = 255;

  // mikmod.h constants
  {*
  * Library version
  *}
  LIBMIKMOD_VERSION_MAJOR = 3;
  LIBMIKMOD_VERSION_MINOR = 1;
  LIBMIKMOD_REVISION = 8;
  LIBMIKMOD_VERSION = ( ( LIBMIKMOD_VERSION_MAJOR shl 16 ) or
    ( LIBMIKMOD_VERSION_MINOR shl 8 ) or
    ( LIBMIKMOD_REVISION ) );

type
  //music_cmd.h types
  PMusicCMD = ^TMusicCMD;
  TMusicCMD = record
    filename : array[ 0..PATH_MAX - 1 ] of char;
    cmd : array[ 0..PATH_MAX - 1 ] of char;
    pid : TSYS_ThreadHandle;
  end;

  //wavestream.h types
  PWAVStream = ^TWAVStream;
  TWAVStream = record
<<<<<<< HEAD
<<<<<<< HEAD
    rw : PSDL_RWops;
    freerw : TSDL_Bool;
=======
    wavefp : Pointer;
>>>>>>> graemeg/fixes_2_2
=======
    wavefp : Pointer;
>>>>>>> origin/fixes_2_2
    start : longint;
    stop : longint;
    cvt : TSDL_AudioCVT;
  end;

  //playmidi.h types
  PMidiEvent = ^TMidiEvent;
  TMidiEvent = record
    time : Longint;
    channel : uint8;
    type_ : uint8;
    a : uint8;
    b : uint8;
  end;

  PMidiSong = ^TMidiSong;
  TMidiSong = record
    samples : Longint;
    events : PMidiEvent;
  end;

  //music_ogg.h types
  POGG_Music = ^TOGG_Music;
  TOGG_Music = record
    playing : integer;
    volume : integer;
    //vf: OggVorbis_File;
    section : integer;
    cvt : TSDL_AudioCVT;
    len_available : integer;
    snd_available : PUint8;
  end;

  // mikmod.h types
  {*
  * Error codes
  *}
  TErrorEnum = (
    MMERR_OPENING_FILE,
    MMERR_OUT_OF_MEMORY,
    MMERR_DYNAMIC_LINKING,
    MMERR_SAMPLE_TOO_BIG,
    MMERR_OUT_OF_HANDLES,
    MMERR_UNKNOWN_WAVE_TYPE,
    MMERR_LOADING_PATTERN,
    MMERR_LOADING_TRACK,
    MMERR_LOADING_HEADER,
    MMERR_LOADING_SAMPLEINFO,
    MMERR_NOT_A_MODULE,
    MMERR_NOT_A_STREAM,
    MMERR_MED_SYNTHSAMPLES,
    MMERR_ITPACK_INVALID_DATA,
    MMERR_DETECTING_DEVICE,
    MMERR_INVALID_DEVICE,
    MMERR_INITIALIZING_MIXER,
    MMERR_OPENING_AUDIO,
    MMERR_8BIT_ONLY,
    MMERR_16BIT_ONLY,
    MMERR_STEREO_ONLY,
    MMERR_ULAW,
    MMERR_NON_BLOCK,
    MMERR_AF_AUDIO_PORT,
    MMERR_AIX_CONFIG_INIT,
    MMERR_AIX_CONFIG_CONTROL,
    MMERR_AIX_CONFIG_START,
    MMERR_GUS_SETTINGS,
    MMERR_GUS_RESET,
    MMERR_GUS_TIMER,
    MMERR_HP_SETSAMPLESIZE,
    MMERR_HP_SETSPEED,
    MMERR_HP_CHANNELS,
    MMERR_HP_AUDIO_OUTPUT,
    MMERR_HP_AUDIO_DESC,
    MMERR_HP_BUFFERSIZE,
    MMERR_OSS_SETFRAGMENT,
    MMERR_OSS_SETSAMPLESIZE,
    MMERR_OSS_SETSTEREO,
    MMERR_OSS_SETSPEED,
    MMERR_SGI_SPEED,
    MMERR_SGI_16BIT,
    MMERR_SGI_8BIT,
    MMERR_SGI_STEREO,
    MMERR_SGI_MONO,
    MMERR_SUN_INIT,
    MMERR_OS2_MIXSETUP,
    MMERR_OS2_SEMAPHORE,
    MMERR_OS2_TIMER,
    MMERR_OS2_THREAD,
    MMERR_DS_PRIORITY,
    MMERR_DS_BUFFER,
    MMERR_DS_FORMAT,
    MMERR_DS_NOTIFY,
    MMERR_DS_EVENT,
    MMERR_DS_THREAD,
    MMERR_DS_UPDATE,
    MMERR_WINMM_HANDLE,
    MMERR_WINMM_ALLOCATED,
    MMERR_WINMM_DEVICEID,
    MMERR_WINMM_FORMAT,
    MMERR_WINMM_UNKNOWN,
    MMERR_MAC_SPEED,
    MMERR_MAC_START,
    MMERR_MAX
    );

  PMODULE = ^TMODULE;
  TMODULE = record
    (* general module information *)
    //CHAR*       songname;    (* name of the song *)
    //CHAR*       modtype;     (* string type of module loaded *)
    //CHAR*       comment;     (* module comments *)
    //UWORD       flags;       (* See module flags above *)
    //UBYTE       numchn;      (* number of module channels *)
    //UBYTE       numvoices;   (* max # voices used for full NNA playback *)
    //UWORD       numpos;      (* number of positions in this song *)
    //UWORD       numpat;      (* number of patterns in this song *)
    //UWORD       numins;      (* number of instruments *)
    //UWORD       numsmp;      (* number of samples *)
    //type = record  INSTRUMENT* instruments; (* all instruments *)
    //type = record  SAMPLE*     samples;     (* all samples *)
    //UBYTE       realchn;     (* real number of channels used *)
    //UBYTE       totalchn;    (* total number of channels used (incl NNAs) *)
    (* playback settings *)
    //UWORD       reppos;      (* restart position *)
    //UBYTE       initspeed;   (* initial song speed *)
    //UWORD       inittempo;   (* initial song tempo *)
    //UBYTE       initvolume;  (* initial global volume (0 - 128) *)
    //UWORD       panning : array[ 0..64- 1 ] of ; (* 64 panning positions *)
    //UBYTE       chanvol : array[ 0..64- 1 ] of ; (* 64 channel positions *)
    //UWORD       bpm;         (* current beats-per-minute speed *)
    //UWORD       sngspd;      (* current song speed *)
    //SWORD       volume;      (* song volume (0-128) (or user volume) *)
    //BOOL        extspd;      (* extended speed flag (default enabled) *)
    //BOOL        panflag;     (* panning flag (default enabled) *)
    //BOOL        wrap;        (* wrap module ? (default disabled) *)
    //BOOL        loop;		 (* allow module to loop ? (default enabled) *)
    //BOOL        fadeout;	 (* volume fade out during last pattern *)
    //UWORD       patpos;      (* current row number *)
    //SWORD       sngpos;      (* current song position *)
    //ULONG       sngtime;     (* current song time in 2^-10 seconds *)
    //SWORD       relspd;      (* relative speed factor *)
    (* internal module representation *)
    //UWORD       numtrk;      (* number of tracks *)
    //UBYTE**     tracks;      (* array of numtrk pointers to tracks *)
    //UWORD*      patterns;    (* array of Patterns *)
    //UWORD*      pattrows;    (* array of number of rows for each pattern *)
    //UWORD*      positions;   (* all positions *)
    //BOOL        forbid;      (* if true, no player updatenot  *)
    //UWORD       numrow;      (* number of rows on current pattern *)
    //UWORD       vbtick;      (* tick counter (counts from 0 to sngspd) *)
    //UWORD       sngremainder;(* used for song time computation *)
    //type = record MP_CONTROL*  control;     (* Effects Channel info (size pf.numchn) *)
    //type = record MP_VOICE*    voice;       (* Audio Voice information (size md_numchn) *)
    //UBYTE       globalslide; (* global volume slide rate *)
    //UBYTE       pat_repcrazy;(* module has just looped to position -1 *)
    //UWORD       patbrk;      (* position where to start a new pattern *)
    //UBYTE       patdly;      (* patterndelay counter (command memory) *)
    //UBYTE       patdly2;     (* patterndelay counter (real one) *)
    //SWORD       posjmp;      (* flag to indicate a jump is needed... *)
  end;

  PUNIMOD = ^TUNIMOD;
  TUNIMOD = TMODULE;

  //SDL_mixer.h types
  { The internal format for an audio chunk }
  PMix_Chunk = ^TMix_Chunk;
  TMix_Chunk = record
    allocated : integer;
    abuf : PUint8;
    alen : Uint32;
    volume : Uint8; { Per-sample volume, 0-128 }
  end;
  Mix_Chunk = TMix_Chunk;

  { The different fading types supported }
  TMix_Fading = (
    MIX_NO_FADING,
    MIX_FADING_OUT,
    MIX_FADING_IN
    );
  Mix_Fading = TMix_Fading;

  TMix_MusicType = (
    MUS_NONE,
    MUS_CMD,
    MUS_WAV,
    MUS_MOD,
    MUS_MID,
    MUS_OGG,
    MUS_MP3
    );
  Mix_MusicType = TMix_MusicType;

  TMusicUnion = record
    case Byte of
      0 : ( cmd : PMusicCMD );
      1 : ( wave : PWAVStream );
      2 : ( module : PUNIMOD );
      3 : ( midi : TMidiSong );
      4 : ( ogg : POGG_music );
      {$IFNDEF DARWIN}
      5 : ( mp3 : PSMPEG );
      {$ENDIF}
  end;

  P_Mix_Music = ^T_Mix_Music;
  T_Mix_Music = record
    type_ : TMix_MusicType;
    data : TMusicUnion;
    fading : TMix_Fading;
    fade_volume : integer;
    fade_step : integer;
    fade_steps : integer;
    error : integer;
  end;

  { The internal format for a music chunk interpreted via mikmod }
  PMix_Music = ^TMix_Music;
  TMix_Music = T_Mix_Music;

  {$IFNDEF __GPC__}
  TMixFunction = function( udata : Pointer; stream : PUint8; len : integer ) : Pointer; cdecl;
  {$ELSE}
  TMixFunction = function( udata : Pointer; stream : PUint8; len : integer ) : Pointer;
  {$ENDIF}

{ This macro can be used to fill a version structure with the compile-time
  version of the SDL_mixer library. }
procedure SDL_MIXER_VERSION(var X: TSDL_Version);
{$EXTERNALSYM SDL_MIXER_VERSION}

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF MORPHOS}
{$INCLUDE powersdl_mixer.inc}
{$ELSE MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ This function gets the version of the dynamically linked SDL_mixer library.
     It should NOT be used to fill a version structure, instead you should use the
     SDL_MIXER_VERSION() macro. }
function Mix_Linked_Version : PSDL_version;
cdecl; external {$IFDEF __GPC__}name 'Mix_Linked_Version'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_Linked_Version}

{ Open the mixer with a certain audio format }
function Mix_OpenAudio( frequency : integer; format : Uint16; channels :
  integer; chunksize : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_OpenAudio'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_OpenAudio}

{ Dynamically change the number of channels managed by the mixer.
   If decreasing the number of channels, the upper channels are
   stopped.
   This function returns the new number of allocated channels.
 }
function Mix_AllocateChannels( numchannels : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_AllocateChannels'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_AllocateChannels}

{ Find out what the actual audio device parameters are.
   This function returns 1 if the audio has been opened, 0 otherwise.
 }
function Mix_QuerySpec( var frequency : integer; var format : Uint16; var channels : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_QuerySpec'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_QuerySpec}

{ Load a wave file or a music (.mod .s3m .it .xm) file }
function Mix_LoadWAV_RW( src : PSDL_RWops; freesrc : integer ) : PMix_Chunk;
cdecl; external {$IFDEF __GPC__}name 'Mix_LoadWAV_RW'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_LoadWAV_RW}

function Mix_LoadWAV( filename : PChar ) : PMix_Chunk;

function Mix_LoadMUS( const filename : PChar ) : PMix_Music;
cdecl; external {$IFDEF __GPC__}name 'Mix_LoadMUS'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_LoadMUS}

(*#if 0 { This hasn't been hooked into music.c yet }
{ Load a music file from an SDL_RWop object (MikMod-specific currently)
   Matt Campbell (matt@campbellhome.dhs.org) April 2000 }
function Mix_LoadMUS_RW(SDL_RWops *rw) : PMix_Music;  cdecl;
#endif*)

{ Load a wave file of the mixer format from a memory buffer }
function Mix_QuickLoad_WAV( mem : PUint8 ) : PMix_Chunk;
cdecl; external {$IFDEF __GPC__}name 'Mix_QuickLoad_WAV'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_QuickLoad_WAV}

{ Free an audio chunk previously loaded }
procedure Mix_FreeChunk( chunk : PMix_Chunk );
cdecl; external {$IFDEF __GPC__}name 'Mix_FreeChunk'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_FreeChunk}

procedure Mix_FreeMusic( music : PMix_Music );
cdecl; external {$IFDEF __GPC__}name 'Mix_FreeMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_FreeMusic}

{ Find out the music format of a mixer music, or the currently playing
   music, if 'music' is NULL.}
function Mix_GetMusicType( music : PMix_Music ) : TMix_MusicType;
cdecl; external {$IFDEF __GPC__}name 'Mix_GetMusicType'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_GetMusicType}

{ Set a function that is called after all mixing is performed.
   This can be used to provide real-time visual display of the audio stream
   or add a custom mixer filter for the stream data.
}
procedure Mix_SetPostMix( mix_func : TMixFunction; arg : Pointer );
cdecl; external {$IFDEF __GPC__}name 'Mix_SetPostMix'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_SetPostMix}

{ Add your own music player or additional mixer function.
   If 'mix_func' is NULL, the default music player is re-enabled.
 }
procedure Mix_HookMusic( mix_func : TMixFunction; arg : Pointer );
 cdecl; external {$IFDEF __GPC__}name 'Mix_HookMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_HookMusic}

{ Add your own callback when the music has finished playing.
 }
procedure Mix_HookMusicFinished( music_finished : Pointer );
cdecl; external {$IFDEF __GPC__}name 'Mix_HookMusicFinished'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_HookMusicFinished}

{ Get a pointer to the user data for the current music hook }
function Mix_GetMusicHookData : Pointer;
cdecl; external {$IFDEF __GPC__}name 'Mix_GetMusicHookData'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_GetMusicHookData}

<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{* Add your own callback when a channel has finished playing. NULL
 * to disable callback.*}
type
  {$IFNDEF __GPC__}
  TChannel_finished = procedure( channel: Integer ); cdecl;
  {$ELSE}
  TChannel_finished = procedure( channel: Integer );
  {$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
{$IFNDEF MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure Mix_ChannelFinished( channel_finished : TChannel_finished );
cdecl; external {$IFDEF __GPC__}name 'Mix_ChannelFinished'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_ChannelFinished}

<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
const
  MIX_CHANNEL_POST = -2;

  {* This is the format of a special effect callback:
   *
   *   myeffect(int chan, void *stream, int len, void *udata);
   *
   * (chan) is the channel number that your effect is affecting. (stream) is
   *  the buffer of data to work upon. (len) is the size of (stream), and
   *  (udata) is a user-defined bit of data, which you pass as the last arg of
   *  Mix_RegisterEffect(), and is passed back unmolested to your callback.
   *  Your effect changes the contents of (stream) based on whatever parameters
   *  are significant, or just leaves it be, if you prefer. You can do whatever
   *  you like to the buffer, though, and it will continue in its changed state
   *  down the mixing pipeline, through any other effect functions, then finally
   *  to be mixed with the rest of the channels and music for the final output
   *  stream.
   *}
type
  {$IFNDEF __GPC__}
  TMix_EffectFunc = function( chan : integer; stream : Pointer; len : integer; udata : Pointer ) : Pointer; cdecl;
  {$ELSE}
  TMix_EffectFunc = function( chan : integer; stream : Pointer; len : integer; udata : Pointer ) : Pointer;
  {$ENDIF}
  {*
   * This is a callback that signifies that a channel has finished all its
   *  loops and has completed playback. This gets called if the buffer
   *  plays out normally, or if you call Mix_HaltChannel(), implicitly stop
   *  a channel via Mix_AllocateChannels(), or unregister a callback while
   *  it's still playing.
   *}
  {$IFNDEF __GPC__}
  TMix_EffectDone = function( chan : integer; udata : Pointer ) : Pointer; cdecl;
  {$ELSE}
  TMix_EffectDone = function( chan : integer; udata : Pointer ) : Pointer;
  {$ENDIF}
  {* Register a special effect function. At mixing time, the channel data is
  *  copied into a buffer and passed through each registered effect function.
  *  After it passes through all the functions, it is mixed into the final
  *  output stream. The copy to buffer is performed once, then each effect
  *  function performs on the output of the previous effect. Understand that
  *  this extra copy to a buffer is not performed if there are no effects
  *  registered for a given chunk, which saves CPU cycles, and any given
  *  effect will be extra cycles, too, so it is crucial that your code run
  *  fast. Also note that the data that your function is given is in the
  *  format of the sound device, and not the format you gave to Mix_OpenAudio(),
  *  although they may in reality be the same. This is an unfortunate but
  *  necessary speed concern. Use Mix_QuerySpec() to determine if you can
  *  handle the data before you register your effect, and take appropriate
  *  actions.
  * You may also specify a callback (Mix_EffectDone_t) that is called when
  *  the channel finishes playing. This gives you a more fine-grained control
  *  than Mix_ChannelFinished(), in case you need to free effect-specific
  *  resources, etc. If you don't need this, you can specify NULL.
  * You may set the callbacks before or after calling Mix_PlayChannel().
  * Things like Mix_SetPanning() are just internal special effect functions,
  *  so if you are using that, you've already incurred the overhead of a copy
  *  to a separate buffer, and that these effects will be in the queue with
  *  any functions you've registered. The list of registered effects for a
  *  channel is reset when a chunk finishes playing, so you need to explicitly
  *  set them with each call to Mix_PlayChannel*().
  * You may also register a special effect function that is to be run after
  *  final mixing occurs. The rules for these callbacks are identical to those
  *  in Mix_RegisterEffect, but they are run after all the channels and the
  *  music have been mixed into a single stream, whereas channel-specific
  *  effects run on a given channel before any other mixing occurs. These
  *  global effect callbacks are call "posteffects". Posteffects only have
  *  their Mix_EffectDone_t function called when they are unregistered (since
  *  the main output stream is never "done" in the same sense as a channel).
  *  You must unregister them manually when you've had enough. Your callback
  *  will be told that the channel being mixed is (MIX_CHANNEL_POST) if the
  *  processing is considered a posteffect.
  *
  * After all these effects have finished processing, the callback registered
  *  through Mix_SetPostMix() runs, and then the stream goes to the audio
  *  device.
  *
  * returns zero if error (no such channel), nonzero if added.
  *  Error messages can be retrieved from Mix_GetError().
  *}
<<<<<<< HEAD
<<<<<<< HEAD

{$IFNDEF MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function Mix_RegisterEffect( chan : integer; f : TMix_EffectFunc; d : TMix_EffectDone; arg : Pointer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_RegisterEffect'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_RegisterEffect}

{* You may not need to call this explicitly, unless you need to stop an
 *  effect from processing in the middle of a chunk's playback.
 * Posteffects are never implicitly unregistered as they are for channels,
 *  but they may be explicitly unregistered through this function by
 *  specifying MIX_CHANNEL_POST for a channel.
 * returns zero if error (no such channel or effect), nonzero if removed.
 *  Error messages can be retrieved from Mix_GetError().
 *}
function Mix_UnregisterEffect( channel : integer; f : TMix_EffectFunc ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_UnregisterEffect'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_UnregisterEffect}

 {* You may not need to call this explicitly, unless you need to stop all
  * effects from processing in the middle of a chunk's playback. Note that
  * this will also shut off some internal effect processing, since
  * Mix_SetPanning( ) and others may use this API under the hood.This is
  * called internally when a channel completes playback.
  * Posteffects are never implicitly unregistered as they are for channels,
  * but they may be explicitly unregistered through this function by
  * specifying MIX_CHANNEL_POST for a channel.
  * returns zero if error( no such channel ), nonzero if all effects removed.
  * Error messages can be retrieved from Mix_GetError( ).
  *}
function Mix_UnregisterAllEffects( channel : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_UnregisterAllEffects'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_UnregisterAllEffects}

<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF MORPHOS}

const
  MIX_EFFECTSMAXSPEED = 'MIX_EFFECTSMAXSPEED';

{$IFNDEF MORPHOS}
=======
const
  MIX_EFFECTSMAXSPEED = 'MIX_EFFECTSMAXSPEED';

>>>>>>> graemeg/fixes_2_2
=======
const
  MIX_EFFECTSMAXSPEED = 'MIX_EFFECTSMAXSPEED';

>>>>>>> origin/fixes_2_2
  {*
  * These are the internally - defined mixing effects.They use the same API that
  * effects defined in the application use, but are provided here as a
  * convenience.Some effects can reduce their quality or use more memory in
  * the name of speed; to enable this, make sure the environment variable
  * MIX_EFFECTSMAXSPEED( see above ) is defined before you call
  * Mix_OpenAudio( ).
  * }

  {* set the panning of a channel.The left and right channels are specified
  * as integers between 0 and 255, quietest to loudest, respectively.
  *
  * Technically, this is just individual volume control for a sample with
  * two( stereo )channels, so it can be used for more than just panning.
  * if you want real panning, call it like this :
  *
  * Mix_SetPanning( channel, left, 255 - left );
  *
  * ...which isn't so hard.
  *
  * Setting( channel ) to MIX_CHANNEL_POST registers this as a posteffect, and
  * the panning will be done to the final mixed stream before passing it on
  * to the audio device.
  *
  * This uses the Mix_RegisterEffect( )API internally, and returns without
  * registering the effect function if the audio device is not configured
  * for stereo output.Setting both( left ) and ( right ) to 255 causes this
  * effect to be unregistered, since that is the data's normal state.
  *
  * returns zero if error( no such channel or Mix_RegisterEffect( )fails ),
  * nonzero if panning effect enabled.Note that an audio device in mono
  * mode is a no - op, but this call will return successful in that case .
  * Error messages can be retrieved from Mix_GetError( ).
  * }
  function Mix_SetPanning( channel : integer; left : Uint8; right : Uint8  ) : integer;
  cdecl; external {$IFDEF __GPC__}name 'Mix_SetPanning'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
  {$EXTERNALSYM Mix_SetPanning}
  
  { * set the position ofa channel.( angle ) is an integer from 0 to 360, that
    * specifies the location of the sound in relation to the listener.( angle )
    * will be reduced as neccesary( 540 becomes 180 degrees, -100 becomes 260 ).
    * Angle 0 is due north, and rotates clockwise as the value increases.
    * for efficiency, the precision of this effect may be limited( angles 1
    * through 7 might all produce the same effect, 8 through 15 are equal, etc ).
    * ( distance ) is an integer between 0 and 255 that specifies the space
    * between the sound and the listener.The larger the number, the further
    * away the sound is .Using 255 does not guarantee that the channel will be
    * culled from the mixing process or be completely silent.For efficiency,
    * the precision of this effect may be limited( distance 0 through 5 might
    * all produce the same effect, 6 through 10 are equal, etc ).Setting( angle )
    * and ( distance ) to 0 unregisters this effect, since the data would be
    * unchanged.
    *
    * if you need more precise positional audio, consider using OpenAL for
    * spatialized effects instead of SDL_mixer.This is only meant to be a
    * basic effect for simple "3D" games.
    *
    * if the audio device is configured for mono output, then you won't get
    * any effectiveness from the angle; however, distance attenuation on the
  * channel will still occur.While this effect will function with stereo
  * voices, it makes more sense to use voices with only one channel of sound,
  * so when they are mixed through this effect, the positioning will sound
  * correct.You can convert them to mono through SDL before giving them to
  * the mixer in the first place if you like.
  *
  * Setting( channel ) to MIX_CHANNEL_POST registers this as a posteffect, and
  * the positioning will be done to the final mixed stream before passing it
  * on to the audio device.
  *
  * This is a convenience wrapper over Mix_SetDistance( ) and Mix_SetPanning( ).
  *
  * returns zero if error( no such channel or Mix_RegisterEffect( )fails ),
  * nonzero if position effect is enabled.
  * Error messages can be retrieved from Mix_GetError( ).
  * }
  function Mix_SetPosition( channel :integer; angle : Sint16; distance : Uint8  ) : integer;
  cdecl; external {$IFDEF __GPC__}name 'Mix_SetPosition'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
  {$EXTERNALSYM Mix_SetPosition}

  {* set the "distance" of a channel.( distance ) is an integer from 0 to 255
  * that specifies the location of the sound in relation to the listener.
  * Distance 0 is overlapping the listener, and 255 is as far away as possible
  * A distance of 255 does not guarantee silence; in such a case , you might
  * want to try changing the chunk's volume, or just cull the sample from the
  * mixing process with Mix_HaltChannel( ).
    * for efficiency, the precision of this effect may be limited( distances 1
    * through 7 might all produce the same effect, 8 through 15 are equal, etc ).
    * ( distance ) is an integer between 0 and 255 that specifies the space
    * between the sound and the listener.The larger the number, the further
    * away the sound is .
    * Setting( distance ) to 0 unregisters this effect, since the data would be
    * unchanged.
    * if you need more precise positional audio, consider using OpenAL for
    * spatialized effects instead of SDL_mixer.This is only meant to be a
    * basic effect for simple "3D" games.
    *
    * Setting( channel ) to MIX_CHANNEL_POST registers this as a posteffect, and
    * the distance attenuation will be done to the final mixed stream before
    * passing it on to the audio device.
    *
  * This uses the Mix_RegisterEffect( )API internally.
  *
  * returns zero if error( no such channel or Mix_RegisterEffect( )fails ),
  * nonzero if position effect is enabled.
    * Error messages can be retrieved from Mix_GetError( ).
    * }
    function Mix_SetDistance( channel : integer; distance : Uint8 ) : integer;
    cdecl; external {$IFDEF __GPC__}name 'Mix_SetDistance'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
    {$EXTERNALSYM Mix_SetDistance}
  { *
    * !!! FIXME : Haven't implemented, since the effect goes past the
  * end of the sound buffer.Will have to think about this.
  * - -ryan.
  * /
  { if 0
  { * Causes an echo effect to be mixed into a sound.( echo ) is the amount
  * of echo to mix.0 is no echo, 255 is infinite( and probably not
  * what you want ).
  *
  * Setting( channel ) to MIX_CHANNEL_POST registers this as a posteffect, and
  * the reverbing will be done to the final mixed stream before passing it on
  * to the audio device.
  *
  * This uses the Mix_RegisterEffect( )API internally.If you specify an echo
  * of zero, the effect is unregistered, as the data is already in that state.
  *
  * returns zero if error( no such channel or Mix_RegisterEffect( )fails ),
  * nonzero if reversing effect is enabled.
    * Error messages can be retrieved from Mix_GetError( ).
    *
    extern no_parse_DECLSPEC int Mix_SetReverb( int channel, Uint8 echo );
  #E ndif}
  { * Causes a channel to reverse its stereo.This is handy if the user has his
    * speakers hooked up backwards, or you would like to have a minor bit of
  * psychedelia in your sound code. : )Calling this function with ( flip )
  * set to non - zero reverses the chunks's usual channels. If (flip) is zero,
  * the effect is unregistered.
  *
  * This uses the Mix_RegisterEffect( )API internally, and thus is probably
  * more CPU intensive than having the user just plug in his speakers
  * correctly.Mix_SetReverseStereo( )returns without registering the effect
  * function if the audio device is not configured for stereo output.
  *
  * if you specify MIX_CHANNEL_POST for ( channel ), then this the effect is used
  * on the final mixed stream before sending it on to the audio device( a
  * posteffect ).
  *
  * returns zero if error( no such channel or Mix_RegisterEffect( )fails ),
  * nonzero if reversing effect is enabled.Note that an audio device in mono
  * mode is a no - op, but this call will return successful in that case .
  * Error messages can be retrieved from Mix_GetError( ).
  * }
  function Mix_SetReverseStereo( channel : integer; flip : integer ) : integer;
  cdecl; external {$IFDEF __GPC__}name 'Mix_SetReverseStereo'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
  {$EXTERNALSYM Mix_SetReverseStereo}
  { end of effects API. - -ryan. *}

{ Reserve the first channels (0 -> n-1) for the application, i.e. don't allocate
   them dynamically to the next sample if requested with a -1 value below.
   Returns the number of reserved channels.
 }
function Mix_ReserveChannels( num : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_ReserveChannels'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_ReserveChannels}

{ Channel grouping functions }

{ Attach a tag to a channel. A tag can be assigned to several mixer
   channels, to form groups of channels.
   If 'tag' is -1, the tag is removed (actually -1 is the tag used to
   represent the group of all the channels).
   Returns true if everything was OK.
 }
function Mix_GroupChannel( which : integer; tag : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_GroupChannel'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_GroupChannel}

{ Assign several consecutive channels to a group }
function Mix_GroupChannels( from : integer; to_ : integer; tag : integer ) :
integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_GroupChannels'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_GroupChannels}

{ Finds the first available channel in a group of channels }
function Mix_GroupAvailable( tag : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_GroupAvailable'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_GroupAvailable}

{ Returns the number of channels in a group. This is also a subtle
   way to get the total number of channels when 'tag' is -1
 }
function Mix_GroupCount( tag : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_GroupCount'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_GroupCount}

{ Finds the "oldest" sample playing in a group of channels }
function Mix_GroupOldest( tag : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_GroupOldest'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_GroupOldest}

{ Finds the "most recent" (i.e. last) sample playing in a group of channels }
function Mix_GroupNewer( tag : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_GroupNewer'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_GroupNewer}

{ The same as above, but the sound is played at most 'ticks' milliseconds }
function Mix_PlayChannelTimed( channel : integer; chunk : PMix_Chunk; loops : integer; ticks : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_PlayChannelTimed'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_PlayChannelTimed}

{ Play an audio chunk on a specific channel.
   If the specified channel is -1, play on the first free channel.
   If 'loops' is greater than zero, loop the sound that many times.
   If 'loops' is -1, loop inifinitely (~65000 times).
   Returns which channel was used to play the sound.
}
function Mix_PlayChannel( channel : integer; chunk : PMix_Chunk; loops : integer ) : integer;

function Mix_PlayMusic( music : PMix_Music; loops : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_PlayMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_PlayMusic}

{ Fade in music or a channel over "ms" milliseconds, same semantics as the "Play" functions }
function Mix_FadeInMusic( music : PMix_Music; loops : integer; ms : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_FadeInMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_FadeInMusic}

function Mix_FadeInChannelTimed( channel : integer; chunk : PMix_Chunk; loops : integer; ms : integer; ticks : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_FadeInChannelTimed'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_FadeInChannelTimed}

function Mix_FadeInChannel( channel : integer; chunk : PMix_Chunk; loops : integer; ms : integer ) : integer;

{ Set the volume in the range of 0-128 of a specific channel or chunk.
   If the specified channel is -1, set volume for all channels.
   Returns the original volume.
   If the specified volume is -1, just return the current volume.
}
function Mix_Volume( channel : integer; volume : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_Volume'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_Volume}

function Mix_VolumeChunk( chunk : PMix_Chunk; volume : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_VolumeChunk'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_VolumeChunk}

function Mix_VolumeMusic( volume : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_VolumeMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_VolumeMusic}

{ Halt playing of a particular channel }
function Mix_HaltChannel( channel : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_HaltChannel'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_HaltChannel}

function Mix_HaltGroup( tag : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_HaltGroup'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_HaltGroup}

function Mix_HaltMusic : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_HaltMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_HaltMusic}

{ Change the expiration delay for a particular channel.
   The sample will stop playing after the 'ticks' milliseconds have elapsed,
   or remove the expiration if 'ticks' is -1
}
function Mix_ExpireChannel( channel : integer; ticks : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_ExpireChannel'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_ExpireChannel}

{ Halt a channel, fading it out progressively till it's silent
   The ms parameter indicates the number of milliseconds the fading
   will take.
 }
function Mix_FadeOutChannel( which : integer; ms : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_FadeOutChannel'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_FadeOutChannel}
function Mix_FadeOutGroup( tag : integer; ms : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_FadeOutGroup'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_FadeOutGroup}
function Mix_FadeOutMusic( ms : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_FadeOutMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_FadeOutMusic}

{ Query the fading status of a channel }
function Mix_FadingMusic : TMix_Fading;
cdecl; external {$IFDEF __GPC__}name 'Mix_FadingMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_FadingMusic}

function Mix_FadingChannel( which : integer ) : TMix_Fading;
cdecl; external {$IFDEF __GPC__}name 'Mix_FadingChannel'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_FadingChannel}

{ Pause/Resume a particular channel }
procedure Mix_Pause( channel : integer );
cdecl; external {$IFDEF __GPC__}name 'Mix_Pause'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_Pause}

procedure Mix_Resume( channel : integer );
cdecl; external {$IFDEF __GPC__}name 'Mix_Resume'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_Resume}

function Mix_Paused( channel : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_Paused'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_Paused}

{ Pause/Resume the music stream }
procedure Mix_PauseMusic;
cdecl; external {$IFDEF __GPC__}name 'Mix_PauseMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_PauseMusic}

procedure Mix_ResumeMusic;
cdecl; external {$IFDEF __GPC__}name 'Mix_ResumeMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_ResumeMusic}

procedure Mix_RewindMusic;
cdecl; external {$IFDEF __GPC__}name 'Mix_RewindMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_RewindMusic}

function Mix_PausedMusic : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_PausedMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_PausedMusic}

{ Set the current position in the music stream.
  This returns 0 if successful, or -1 if it failed or isn't implemented.
  This function is only implemented for MOD music formats (set pattern
  order number) and for OGG music (set position in seconds), at the
  moment.
}
function Mix_SetMusicPosition( position : double ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_SetMusicPosition'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_SetMusicPosition}

{ Check the status of a specific channel.
   If the specified channel is -1, check all channels.
}
function Mix_Playing( channel : integer ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_Playing'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_Playing}

function Mix_PlayingMusic : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_PlayingMusic'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_PlayingMusic}

{ Stop music and set external music playback command }
function Mix_SetMusicCMD( const command : PChar ) : integer;
cdecl; external {$IFDEF __GPC__}name 'Mix_SetMusicCMD'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_SetMusicCMD}

{ Synchro value is set by MikMod from modules while playing }
function Mix_SetSynchroValue( value : integer ) : integer; overload;
cdecl; external {$IFDEF __GPC__}name 'Mix_SetSynchroValue'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_SetSynchroValue}

function Mix_GetSynchroValue : integer; overload;
cdecl; external {$IFDEF __GPC__}name 'Mix_GetSynchroValue'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_SetSynchroValue}

{
  Get the Mix_Chunk currently associated with a mixer channel
    Returns nil if it's an invalid channel, or there's no chunk associated.
}
function Mix_GetChunk( channel : integer ) : PMix_Chunk;
cdecl; external {$IFDEF __GPC__}name 'Mix_GetChunk'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_GetChunk}

{ Close the mixer, halting all playing audio }
procedure Mix_CloseAudio;
cdecl; external {$IFDEF __GPC__}name 'Mix_CloseAudio'{$ELSE} SDL_MixerLibName{$ENDIF __GPC__};
{$EXTERNALSYM Mix_CloseAudio}

<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ We'll use SDL for reporting errors }
procedure Mix_SetError( fmt : PChar );

function Mix_GetError : PChar;

{------------------------------------------------------------------------------}
{ initialization                                                               }
{------------------------------------------------------------------------------}

{------------------------------------------------------------------------------}

implementation

{$IFDEF __GPC__}
  {$L 'sdl_mixer'}  { link sdl_mixer.dll.a or libsdl_mixer.so or libsdl_mixer.a }
{$ENDIF}

procedure SDL_MIXER_VERSION( var X : TSDL_version );
begin
  X.major := SDL_MIXER_MAJOR_VERSION;
  X.minor := SDL_MIXER_MINOR_VERSION;
  X.patch := SDL_MIXER_PATCHLEVEL;
end;

function Mix_LoadWAV( filename : PChar ) : PMix_Chunk;
begin
  result := Mix_LoadWAV_RW( SDL_RWFromFile( filename, 'rb' ), 1 );
end;

function Mix_PlayChannel( channel : integer; chunk : PMix_Chunk; loops : integer ) : integer;
begin
  result := Mix_PlayChannelTimed( channel, chunk, loops, -1 );
end;

function Mix_FadeInChannel( channel : integer; chunk : PMix_Chunk; loops :
  integer; ms : integer ) : integer;
begin
  result := Mix_FadeInChannelTimed( channel, chunk, loops, ms, -1 );
end;

procedure Mix_SetError( fmt : PChar );
begin
  SDL_SetError( fmt );
end;

function Mix_GetError : PChar;
begin
  result := SDL_GetError;
end;

end.