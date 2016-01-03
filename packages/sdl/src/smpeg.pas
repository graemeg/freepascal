unit smpeg;
{******************************************************************************}
{
  $Id: smpeg.pas,v 1.7 2004/08/14 22:54:30 savage Exp $
  
}
{                                                                              }
{       Borland Delphi SMPEG - SDL MPEG Player Library                         }
{       Conversion of the SMPEG - SDL MPEG Player Library                      }
{                                                                              }
{ Portions created by Sam Lantinga <slouken@devolution.com> are                }
{ Copyright (C) 1997, 1998, 1999, 2000, 2001  Sam Lantinga                     }
{ 5635-34 Springhouse Dr.                                                      }
{ Pleasanton, CA 94588 (USA)                                                   }
{                                                                              }
{ All Rights Reserved.                                                         }
{                                                                              }
{ The original files are : smpeg.h                                             }
{                                                                              }
{ The initial developer of this Pascal code was :                              }
{ Matthias Thoma <ma.thoma@gmx.de>                                             }
{                                                                              }
{ Portions created by Matthias Thoma are                                       }
{ Copyright (C) 2000 - 2001 Matthias Thoma.                                    }
{                                                                              }
{                                                                              }
{ Contributor(s)                                                               }
{ --------------                                                               }
{ Tom Jones <tigertomjones@gmx.de>  His Project inspired this conversion       }
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
{ http://www.mozilla.org/MPL/MPL-1.1.html                                      }
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
{   The SDL Runtime libraris on Win32  : SDL.dll on Linux : libSDL-1.2.so.0    }
{   They are available from...                                                 }
{   http://www.libsdl.org .                                                    }
{                                                                              }
{ Programming Notes                                                            }
{ -----------------                                                            }
{                                                                              }
{                                                                              }
{                                                                              }
{                                                                              }
{ Revision History                                                             }
{ ----------------                                                             }
{   May      08 2001 - MT : Initial conversion                                 }
{                                                                              }
{   October  12 2001 - DA : Various changes as suggested by David Acklam       }
{                                                                              }
{   April   03 2003 - DL : Added jedi-sdl.inc include file to support more     }
{                          Pascal compilers. Initial support is now included   }
{                          for GnuPascal, VirtualPascal, TMT and obviously     }
{                          continue support for Delphi Kylix and FreePascal.   }
{                                                                              }
{   April   08 2003 - MK : Aka Mr Kroket - Added Better FPC support            }
{                          Fixed all invalid calls to DLL.                     }
{                          Changed constant names to:                          }
{                          const                                               }
{                          STATUS_SMPEG_ERROR = -1;                            }
{                          STATUS_SMPEG_STOPPED = 0;                           }
{                          STATUS_SMPEG_PLAYING = 1;                           }
{                          because SMPEG_ERROR is a function (_SMPEG_error     }
{                          isn't correct), and cannot be two elements with the }
{                          same name                                           }
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
  $Log: smpeg.pas,v $
  Revision 1.7  2004/08/14 22:54:30  savage
  Updated so that Library name defines are correctly defined for MacOS X.

  Revision 1.6  2004/05/10 14:10:04  savage
  Initial MacOS X support. Fixed defines for MACOS ( Classic ) and DARWIN ( MacOS X ).

  Revision 1.5  2004/04/13 09:32:08  savage
  Changed Shared object names back to just the .so extension to avoid conflicts on various Linux/Unix distros. Therefore developers will need to create Symbolic links to the actual Share Objects if necessary.

  Revision 1.4  2004/04/02 10:40:55  savage
  Changed Linux Shared Object name so they reflect the Symbolic Links that are created when installing the RPMs from the SDL site.

  Revision 1.3  2004/03/31 22:20:02  savage
  Windows unit not used in this file, so it was removed to keep the code tidy.

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
{$IFDEF windows}
  SmpegLibName = 'smpeg.dll';
{$ENDIF}

{$IFDEF UNIX}
{$IFDEF DARWIN}
  SmpegLibName = 'libsmpeg.dylib';
{$ELSE}
  SmpegLibName = 'libsmpeg.so';
{$ENDIF}
{$ENDIF}

{$IFDEF MACOS}
  SmpegLibName = 'smpeg';
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF MORPHOS}
  SmpegLibName = 'smpeg.library';
{$ENDIF}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//------------------------------------------------------------------------------
// MPEGFilter.h
//------------------------------------------------------------------------------
{ SMPEG filter info flags }
const
  SMPEG_FILTER_INFO_MB_ERROR = 1;
  SMPEG_FILTER_INFO_PIXEL_ERROR = 2;

{ Filter info from SMPEG }
type
  SMPEG_FilterInfo = record
    yuv_mb_square_error: PUint16;
    yuv_pixel_square_error: PUint16;
  end;
  TSMPEG_FilterInfo = SMPEG_FilterInfo;
  PSMPEG_FilterInfo = ^SMPEG_FilterInfo;

{ MPEG filter definition }
  PSMPEG_Filter = ^TSMPEG_Filter;

{ Callback functions for the filter }
  {$IFNDEF __GPC__}
  TSMPEG_FilterCallback = function( dest, source: PSDL_Overlay; region: PSDL_Rect; filter_info: PSMPEG_FilterInfo; data: Pointer ): Pointer; cdecl;
  {$ELSE}
  TSMPEG_FilterCallback = function( dest, source: PSDL_Overlay; region: PSDL_Rect; filter_info: PSMPEG_FilterInfo; data: Pointer ): Pointer;
  {$ENDIF}

  {$IFNDEF __GPC__}
  TSMPEG_FilterDestroy = function( Filter: PSMPEG_Filter ): Pointer; cdecl;
  {$ELSE}
  TSMPEG_FilterDestroy = function( Filter: PSMPEG_Filter ): Pointer;
  {$ENDIF}  

{ The filter definition itself }
  TSMPEG_Filter = record
    flags: Uint32;
    data: Pointer;
    callback: TSMPEG_FilterCallback;
    destroy: TSMPEG_FilterDestroy;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
{$IFNDEF MORPHOS}
{* This part is a bit confusing in PowerSDL includes, fix later. KB *}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ The null filter (default). It simply copies the source rectangle to the video overlay. }
function SMPEGfilter_null: PSMPEG_Filter;
cdecl; external {$IFDEF __GPC__}name 'SMPEGfilter_null'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ The bilinear filter. A basic low-pass filter that will produce a smoother image. }
function SMPEGfilter_bilinear: PSMPEG_Filter;
cdecl; external {$IFDEF __GPC__}name 'SMPEGfilter_bilinear'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ The deblocking filter. It filters block borders and non-intra coded blocks to reduce blockiness }
function SMPEGfilter_deblocking: PSMPEG_Filter;
cdecl; external {$IFDEF __GPC__}name 'SMPEGfilter_deblocking'{$ELSE} SmpegLibName{$ENDIF __GPC__};
<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

//------------------------------------------------------------------------------
// SMPEG.h
//------------------------------------------------------------------------------
const
  SMPEG_MAJOR_VERSION = 0;
  SMPEG_MINOR_VERSION = 4;
  SMPEG_PATCHLEVEL = 2;

type
  SMPEG_version = record
    major: UInt8;
    minor: UInt8;
    patch: UInt8;
  end;
  TSMPEG_version = SMPEG_version;
  PSMPEG_version = ^TSMPEG_version;

  // This is the actual SMPEG object
  _SMPEG = record
    //obj: PMPEG;
  end;
  TSMPEG = _SMPEG;
  PSMPEG = ^_SMPEG;

  { Used to get information about the SMPEG object }
  __SMPEG_Info = record
    has_audio: Integer;
    has_video: Integer;
    width: Integer;
    height: Integer;
    current_frame: Integer;
    current_fps: double;
    audio_string: array[0..79] of char;
    audio_current_frame: Integer;
    current_offset: UInt32;
    total_size: UInt32;
    current_time: double;
    total_time: double;
  end;
  _SMPEG_Info = __SMPEG_Info;
  SMPEG_Info = _SMPEG_Info;
  TSMPEG_Info = _SMPEG_Info;
  PSMPEG_Info = ^_SMPEG_Info;
  
{ Possible MPEG status codes }
const
  STATUS_SMPEG_ERROR = -1;
  STATUS_SMPEG_STOPPED = 0;
  STATUS_SMPEG_PLAYING = 1;

type
  SMPEGstatus = Integer;
  TSMPEGstatus = Integer;
  PSMPEGstatus = ^Integer;

  { Matches the declaration of SDL_UpdateRect() }
  {$IFNDEF __GPC__}
  TSMPEG_DisplayCallback = function( dst: PSDL_Surface; x, y: Integer; w, h: Cardinal ): Pointer; cdecl;
  {$ELSE}
  TSMPEG_DisplayCallback = function( dst: PSDL_Surface; x, y: Integer; w, h: Cardinal ): Pointer;
  {$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF MORPHOS}
{$INCLUDE powersdl_smpeg.inc}
{$ELSE MORPHOS}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ Create a new SMPEG object from an MPEG file.
  On return, if 'info' is not NULL, it will be filled with information
  about the MPEG object.
  This function returns a new SMPEG object.  Use SMPEG_error() to find out
  whether or not there was a problem building the MPEG stream.
  The sdl_audio parameter indicates if SMPEG should initialize the SDL audio
  subsystem. If not, you will have to use the SMPEG_playaudio() function below
  to extract the decoded data. }
function SMPEG_new(const _file: PChar; info: PSMPEG_Info; sdl_audio: Integer): PSMPEG;
cdecl; external {$IFDEF __GPC__}name 'SMPEG_new'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ The same as above for a file descriptor }
function SMPEG_new_descr(_file: Integer; info: PSMPEG_Info; sdl_audio: Integer): PSMPEG;
cdecl; external {$IFDEF __GPC__}name 'SMPEG_new_descr'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{  The same as above but for a raw chunk of data.  SMPEG makes a copy of the
   data, so the application is free to delete after a successful call to this
   function. }
function SMPEG_new_data(data: Pointer; size: Integer; info: PSMPEG_Info; sdl_audio: Integer): PSMPEG;
cdecl; external {$IFDEF __GPC__}name 'SMPEG_new_data'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Get current information about an SMPEG object }
procedure SMPEG_getinfo(mpeg: PSMPEG; info: PSMPEG_Info);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_getinfo'{$ELSE} SmpegLibName{$ENDIF __GPC__};

//procedure SMPEG_getinfo(mpeg: PSMPEG; info: Pointer);
//cdecl; external {$IFDEF __GPC__}name 'SMPEG_getinfo'{$ELSE} SmpegLibName{$ENDIF __GPC__};
{ Enable or disable audio playback in MPEG stream }
procedure SMPEG_enableaudio(mpeg: PSMPEG; enable: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_enableaudio'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Enable or disable video playback in MPEG stream }
procedure SMPEG_enablevideo(mpeg: PSMPEG; enable: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_enablevideo'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Delete an SMPEG object }
procedure SMPEG_delete(mpeg: PSMPEG);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_delete'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Get the current status of an SMPEG object }
function SMPEG_status(mpeg: PSMPEG): TSMPEGstatus;
cdecl; external {$IFDEF __GPC__}name 'SMPEG_status'{$ELSE} SmpegLibName{$ENDIF __GPC__};
                                  // status
{ Set the audio volume of an MPEG stream, in the range 0-100 }
procedure SMPEG_setvolume(mpeg: PSMPEG; volume: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_setvolume'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Set the destination surface for MPEG video playback
  'surfLock' is a mutex used to synchronize access to 'dst', and can be NULL.
  'callback' is a function called when an area of 'dst' needs to be updated.
  If 'callback' is NULL, the default function (SDL_UpdateRect) will be used. }
procedure SMPEG_setdisplay(mpeg: PSMPEG; dst: PSDL_Surface; surfLock: PSDL_mutex; callback: TSMPEG_DisplayCallback);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_setdisplay'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Set or clear looping play on an SMPEG object }
procedure SMPEG_loop(mpeg: PSMPEG; _repeat: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_loop'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Scale pixel display on an SMPEG object }
procedure SMPEG_scaleXY(mpeg: PSMPEG; width, height: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_scaleXY'{$ELSE} SmpegLibName{$ENDIF __GPC__};
procedure SMPEG_scale(mpeg: PSMPEG; scale: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_scale'{$ELSE} SmpegLibName{$ENDIF __GPC__};

<<<<<<< HEAD
<<<<<<< HEAD
=======
procedure SMPEG_Double(mpeg : PSMPEG; doubleit : Boolean );

>>>>>>> graemeg/fixes_2_2
=======
procedure SMPEG_Double(mpeg : PSMPEG; doubleit : Boolean );

>>>>>>> origin/fixes_2_2
{ Move the video display area within the destination surface }
procedure SMPEG_move(mpeg: PSMPEG; x, y: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_move'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Set the region of the video to be shown }
procedure SMPEG_setdisplayregion(mpeg: PSMPEG; x, y, w, h: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_setdisplayregion'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Play an SMPEG object }
procedure SMPEG_play(mpeg: PSMPEG);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_play'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Pause/Resume playback of an SMPEG object}
procedure SMPEG_pause(mpeg: PSMPEG);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_pause'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Stop playback of an SMPEG object }
procedure SMPEG_stop(mpeg: PSMPEG);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_stop'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Rewind the play position of an SMPEG object to the beginning of the MPEG }
procedure SMPEG_rewind(mpeg: PSMPEG);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_rewind'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Seek 'bytes' bytes in the MPEG stream }
procedure SMPEG_seek(mpeg: PSMPEG; bytes: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_seek'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Skip 'seconds' seconds in the MPEG stream }
procedure SMPEG_skip(mpeg: PSMPEG; seconds: single);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_skip'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Render a particular frame in the MPEG video
   API CHANGE: This function no longer takes a target surface and position.
               Use SMPEG_setdisplay() and SMPEG_move() to set this information. }
procedure SMPEG_renderFrame(mpeg: PSMPEG; framenum: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_renderFrame'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Render the last frame of an MPEG video }
procedure SMPEG_renderFinal(mpeg: PSMPEG; dst: PSDL_Surface; x, y: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_renderFinal'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Set video filter }
function SMPEG_filter(mpeg: PSMPEG; filter: PSMPEG_Filter): PSMPEG_Filter;
cdecl; external {$IFDEF __GPC__}name 'SMPEG_filter'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Return NULL if there is no error in the MPEG stream, or an error message
   if there was a fatal error in the MPEG stream for the SMPEG object. }
function SMPEG_error(mpeg: PSMPEG): PChar;
cdecl; external {$IFDEF __GPC__}name 'SMPEG_error'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Exported callback function for audio playback.
   The function takes a buffer and the amount of data to fill, and returns
   the amount of data in bytes that was actually written.  This will be the
   amount requested unless the MPEG audio has finished.
}
function SMPEG_playAudio(mpeg: PSMPEG; stream: PUInt8; len: Integer): Integer;
cdecl; external {$IFDEF __GPC__}name 'SMPEG_playAudio'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Wrapper for SMPEG_playAudio() that can be passed to SDL and SDL_mixer }
procedure SMPEG_playAudioSDL(mpeg: Pointer; stream: PUInt8; len: Integer);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_playAudioSDL'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Get the best SDL audio spec for the audio stream }
function SMPEG_wantedSpec(mpeg: PSMPEG; wanted: PSDL_AudioSpec): Integer;
cdecl; external {$IFDEF __GPC__}name 'SMPEG_wantedSpec'{$ELSE} SmpegLibName{$ENDIF __GPC__};

{ Inform SMPEG of the actual SDL audio spec used for sound playback }
procedure SMPEG_actualSpec(mpeg: PSMPEG; spec: PSDL_AudioSpec);
cdecl; external {$IFDEF __GPC__}name 'SMPEG_actualSpec'{$ELSE} SmpegLibName{$ENDIF __GPC__};

<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF MORPHOS}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ This macro can be used to fill a version structure with the compile-time
  version of the SDL library. }
procedure SMPEG_GETVERSION( var X : TSMPEG_version );

<<<<<<< HEAD
<<<<<<< HEAD
procedure SMPEG_Double(mpeg : PSMPEG; doubleit : Boolean );

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
implementation

{$IFDEF __GPC__}
  {$L 'smpeg'}  { link smpeg.dll.a or libsmpeg.so or libsmpeg.a }
{$ENDIF}

procedure SMPEG_double(mpeg : PSMPEG; doubleit : Boolean );
begin
  if doubleit then
    SMPEG_scale( mpeg, 2 )
  else
    SMPEG_scale( mpeg, 1 );
end;

procedure SMPEG_GETVERSION( var X : TSMPEG_version );
begin
  X.major := SMPEG_MAJOR_VERSION;
  X.minor := SMPEG_MINOR_VERSION;
  X.patch := SMPEG_PATCHLEVEL;
end;

end.
