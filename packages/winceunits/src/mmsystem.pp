{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ********************************************************************** }

//
// Module Name:mmsystem.h -- Include file for Multimedia API's
//
// *    If defined, the following flags inhibit inclusion
// *    of the indicated items:
// *
// *      MMNOSOUND       Sound support
// *      MMNOWAVE        Waveform support
// *      MMNOMCI         MCI API
// *      MMNOMMIO        file I/O
// *
// *

//
//  Microsoft Windows Mobile 6.0 for PocketPC SDK.
//

unit mmsystem;

{$CALLING cdecl}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_OS_UNICODE}
  {$define UNICODE}
{$endif}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
interface

uses Windows, mmreg;

{$PACKRECORDS 1} // #include "pshpack1.h"   // Assume byte packing throughout

{$DEFINE MMNOMIDI} // No midi audio support.

{$IFDEF WIN32}
{$DEFINE _WIN32}
{$ENDIF WIN32}

{****************************************************************************

                    General constants and data types

****************************************************************************}
//* general constants */
const
      MAXPNAMELEN      = 32;     // max product name length (including NULL)
      MAXERRORLENGTH   = 128;    // max error text length (including final NULL)

{*
 *  Microsoft Manufacturer and Product ID's
 *
    Used with wMid and wPid fields in WAVEOUTCAPS, WAVEINCAPS,
    MIDIOUTCAPS, MIDIINCAPS, AUXCAPS, JOYCAPS structures.
 *}
{*
 *  (these have been moved to
 *  MMREG.H for Windows 4.00 and above).
 *}
//* manufacturer IDs *
const
      MM_MICROSOFT            = 1;        // Microsoft Corporation


//* general data types */
type
     MMVERSION = UINT;      // major (high byte), minor (low byte)

     MMRESULT = UINT;       // error return code, 0 means no error
                                // call as if(err=xxxx(...)) Error(err); else

     LPUINT = ^UINT;


//* MMTIME data structure */
type
     mmtime_tag = record
       wType:UINT;      //* indicates the contents of the union */
       case UINT of
         0: (ms:DWORD);         //* milliseconds */
         1: (sample:DWORD);     //* samples */
         2: (cb:DWORD);         //* byte count */
         3: (ticks:DWORD);      //* ticks in MIDI stream */
         4: (smpte:record
                     hour:byte;       //* hours */
                     min:byte;        //* minutes */
                     sec:byte;        //* seconds */
                     frame:byte;      //* frames  */
                     fps:byte;        //* frames per second */
                     dummy:byte;      //* pad */
{$IFDEF _WIN32}
                     pad:array[0..1] of byte;
{$ENDIF _WIN32}
                   end);
         5: (midi:record
                    songptrpos:DWORD;   //* song pointer position */
                  end);
     end;
     MMTIME = mmtime_tag;
     PMMTIME = ^mmtime_tag;
     NPMMTIME = ^mmtime_tag;
     LPMMTIME = ^mmtime_tag;

//* types for wType field in MMTIME struct */
const
      TIME_MS         = $0001; //* time in milliseconds */
      TIME_SAMPLES    = $0002; //* number of wave samples */
      TIME_BYTES      = $0004; //* current byte offset */
      TIME_SMPTE      = $0008; //* SMPTE time */
      TIME_MIDI       = $0010; //* MIDI time */
      TIME_TICKS      = $0020; //* Ticks within MIDI stream */

{ Was declared as
      MAKEFOURCC(ch0, ch1, ch2, ch3)                              \
      ((DWORD)(BYTE)(ch0) | ((DWORD)(BYTE)(ch1) << 8) |   \
      ((DWORD)(BYTE)(ch2) << 16) | ((DWORD)(BYTE)(ch3) << 24 ))

      mmioFOURCC(ch0, ch1, ch2, ch3)  MAKEFOURCC(ch0, ch1, ch2, ch3)
}
function MAKEFOURCC(ch0:AnsiChar; ch1:AnsiChar; ch2:AnsiChar; ch3:AnsiChar):FOURCC;
function mmioFOURCC(ch0:AnsiChar; ch1:AnsiChar; ch2:AnsiChar; ch3:AnsiChar):FOURCC;


{****************************************************************************

            Multimedia Extensions Window Messages

****************************************************************************}
const
      MM_MCINOTIFY        = $3B9;           //* MCI */

      MM_WOM_OPEN         = $3BB;           //* waveform output */
      MM_WOM_CLOSE        = $3BC;
      MM_WOM_DONE         = $3BD;
      MM_WOM_ATTENUATED   = $3D8;           //* gain class support - stream attenuated */

      MM_WIM_OPEN         = $3BE;           //* waveform input */
      MM_WIM_CLOSE        = $3BF;
      MM_WIM_DATA         = $3C0;

      MM_MIM_OPEN         = $3C1;           //* MIDI input */
      MM_MIM_CLOSE        = $3C2;
      MM_MIM_DATA         = $3C3;
      MM_MIM_LONGDATA     = $3C4;
      MM_MIM_ERROR        = $3C5;
      MM_MIM_LONGERROR    = $3C6;

      MM_MOM_OPEN         = $3C7;           //* MIDI output */
      MM_MOM_CLOSE        = $3C8;
      MM_MOM_DONE         = $3C9;

//* these are also in msvideo.h */
      MM_DRVM_OPEN       = $3D0;           //* installable drivers */
      MM_DRVM_CLOSE      = $3D1;
      MM_DRVM_DATA       = $3D2;
      MM_DRVM_ERROR      = $3D3;

      MM_MCISYSTEM_STRING = $3CA;       //* internal */

//* these are used by msacm.h */
      MM_STREAM_OPEN      = $3D4;
      MM_STREAM_CLOSE     = $3D5;
      MM_STREAM_DONE      = $3D6;
      MM_STREAM_ERROR     = $3D7;

      MM_MOM_POSITIONCB   = $3CA;           //* Callback for MEVT_POSITIONCB */

      MM_MCISIGNAL        = $3CB;

      MM_MIM_MOREDATA      = $3CC;          //* MIM_DONE w/ pending events */

      MM_MIXM_LINE_CHANGE     = $3D0;       //* mixer line change notify */
      MM_MIXM_CONTROL_CHANGE  = $3D1;       //* mixer control change notify */

{****************************************************************************

        String resource number bases (internal use)

****************************************************************************}
const
      MMSYSERR_BASE          = 0;
      WAVERR_BASE            = 32;
      MIDIERR_BASE           = 64;
      TIMERR_BASE            = 96;
      MCIERR_BASE            = 256;
      MIXERR_BASE            = 1024;

      MCI_STRING_OFFSET      = 512;
      MCI_VD_OFFSET          = 1024;
      MCI_CD_OFFSET          = 1088;
      MCI_WAVE_OFFSET        = 1152;
      MCI_SEQ_OFFSET         = 1216;

{****************************************************************************

            General error return values

****************************************************************************}
const
//* general error return values */
      MMSYSERR_NOERROR      = 0;                  //* no error */
      MMSYSERR_ERROR        = MMSYSERR_BASE + 1;  //* unspecified error */
      MMSYSERR_BADDEVICEID  = MMSYSERR_BASE + 2;  //* device ID out of range */
      MMSYSERR_NOTENABLED   = MMSYSERR_BASE + 3;  //* driver failed enable */
      MMSYSERR_ALLOCATED    = MMSYSERR_BASE + 4;  //* device already allocated */
      MMSYSERR_INVALHANDLE  = MMSYSERR_BASE + 5;  //* device handle is invalid */
      MMSYSERR_NODRIVER     = MMSYSERR_BASE + 6;  //* no device driver present */
      MMSYSERR_NOMEM        = MMSYSERR_BASE + 7;  //* memory allocation error */
      MMSYSERR_NOTSUPPORTED = MMSYSERR_BASE + 8;  //* function isn't supported */
      MMSYSERR_BADERRNUM    = MMSYSERR_BASE + 9;  //* error value out of range */
      MMSYSERR_INVALFLAG    = MMSYSERR_BASE + 10; //* invalid flag passed */
      MMSYSERR_INVALPARAM   = MMSYSERR_BASE + 11; //* invalid parameter passed */
      MMSYSERR_HANDLEBUSY   = MMSYSERR_BASE + 12; //* handle being used */
                                                  //* simultaneously on another */
                                                  //* thread (eg callback) */
      MMSYSERR_INVALIDALIAS = MMSYSERR_BASE + 13; //* specified alias not found */
      MMSYSERR_BADDB        = MMSYSERR_BASE + 14; //* bad registry database */
      MMSYSERR_KEYNOTFOUND  = MMSYSERR_BASE + 15; //* registry key not found */
      MMSYSERR_READERROR    = MMSYSERR_BASE + 16; //* registry read error */
      MMSYSERR_WRITEERROR   = MMSYSERR_BASE + 17; //* registry write error */
      MMSYSERR_DELETEERROR  = MMSYSERR_BASE + 18; //* registry delete error */
      MMSYSERR_VALNOTFOUND  = MMSYSERR_BASE + 19; //* registry value not found */
      MMSYSERR_NODRIVERCB   = MMSYSERR_BASE + 20; //* driver does not call DriverCallback */
      MMSYSERR_LASTERROR    = MMSYSERR_BASE + 20; //* last error in range */

type
     HDRVR = HANDLE;

{$IFNDEF MMNODRV}
//
// Installable driver support was left out.
//
{$ENDIF MMNODRV}


{****************************************************************************

              Driver callback support

****************************************************************************}

//* flags used with waveOutOpen(), waveInOpen(), midiInOpen(), and */
//* midiOutOpen() to specify the type of the dwCallback parameter. */
const
      CALLBACK_TYPEMASK   = $00070000;    //* callback type mask */
      CALLBACK_NULL       = $00000000;    //* no callback */
      CALLBACK_WINDOW     = $00010000;    //* dwCallback is a HWND */
      CALLBACK_TASK       = $00020000;    //* dwCallback is a HTASK */
      CALLBACK_FUNCTION   = $00030000;    //* dwCallback is a FARPROC */
      CALLBACK_THREAD     = CALLBACK_TASK;//* thread ID replaces 16 bit task */
      CALLBACK_EVENT      = $00050000;    //* dwCallback is an EVENT Handle */
      CALLBACK_MSGQUEUE   = $00060000;    //* dwCallback is HANDLE returned by CreateMsgQueue or OpenMsgQueue (new in Windows CE 5.0) */


type
     DRVCALLBACK = procedure(_hdrvr:HANDLE; uMsg:UINT; dwUser:DWORD; dw1:DWORD; dw2:DWORD); cdecl;
     LPDRVCALLBACK = DRVCALLBACK;
     PDRVCALLBACK = DRVCALLBACK;

{* CALLBACK_MSGQUEUE - client process sets up a MsgQueuue that receives WAVEMSG structures.
*  Note that structure fields are identical to arguments to a callback function
*  but we put the message field first to allow for multi-functional message queues.
*}
type
     _tag_WAVEMSG = record
       uMsg:UINT;       // WOM_OPEN, WIM_OPEN, WOM_DONE, WIM_DATA, etc.
       hWav:HANDLE;       // stream handle returned by waveInOpen or waveOutOpen
       dwInstance:DWORD; // value of dwInstance argument passed into waveInOpen or waveOutOpen
       dwParam1:DWORD;   // completed WAVEHDR for WIM_DATA, WOM_DONE, reserved elsewhere
       dwParam2:DWORD;   // reserved
     end;
     WAVEMSG = _tag_WAVEMSG;
     PWAVEMSG = ^_tag_WAVEMSG;

{$IFNDEF MMNOSOUND}

function sndPlaySoundW(lpszSoundName:LPCWSTR; fuSound:UINT):BOOL; external KernelDLL name 'sndPlaySoundW'; // index 266
{$IFDEF UNICODE}
function sndPlaySound(lpszSoundName:LPCWSTR; fuSound:UINT):BOOL; external KernelDLL name 'sndPlaySoundW'; // index 266
{$ELSE} // UNICODE
//#define sndPlaySound  sndPlaySoundA
{$ENDIF} // UNICODE

function PlaySoundW(pszSound:LPCWSTR; hmod:HMODULE; fdwSound:DWORD):BOOL; external KernelDLL name 'PlaySoundW'; // index 267
{$IFDEF UNICODE}
function PlaySound(pszSound:LPCWSTR; hmod:HMODULE; fdwSound:DWORD):BOOL; external KernelDLL name 'PlaySoundW'; // index 267
{$ELSE} // UNICODE
//#define PlaySound  PlaySoundA
{$ENDIF} // UNICODE

{*
 *  flag values for fuSound arguments on [snd]PlaySound
 *  or dwFlags for PlaySound
 *}

//* sndAlias creates the alias identifier */
const
      SND_ALIAS_START  = 0;      // ??? must be > 4096 to keep strings in same section of resource file

function sndAlias(ch0:AnsiChar; ch1:AnsiChar):DWORD;

const
      SND_ALIAS_SYSTEMASTERISK        = SND_ALIAS_START+
                                        (DWORD(AnsiChar('S')) or
                                         DWORD(AnsiChar('*')));

      SND_ALIAS_SYSTEMQUESTION        = SND_ALIAS_START+
                                        (DWORD(AnsiChar('S')) or
                                         DWORD(AnsiChar('?')));

      SND_ALIAS_SYSTEMHAND            = SND_ALIAS_START+
                                        (DWORD(AnsiChar('S')) or
                                         DWORD(AnsiChar('H')));

      SND_ALIAS_SYSTEMEXIT            = SND_ALIAS_START+
                                        (DWORD(AnsiChar('S')) or
                                         DWORD(AnsiChar('E')));

      SND_ALIAS_SYSTEMSTART           = SND_ALIAS_START+
                                        (DWORD(AnsiChar('S')) or
                                         DWORD(AnsiChar('S')));

      SND_ALIAS_SYSTEMWELCOME         = SND_ALIAS_START+
                                        (DWORD(AnsiChar('S')) or
                                         DWORD(AnsiChar('W')));

      SND_ALIAS_SYSTEMEXCLAMATION     = SND_ALIAS_START+
                                        (DWORD(AnsiChar('S')) or
                                         DWORD(AnsiChar('!')));

      SND_ALIAS_SYSTEMDEFAULT         = SND_ALIAS_START+
                                        (DWORD(AnsiChar('S')) or
                                         DWORD(AnsiChar('D')));

      SND_ALIAS      = $00010000;   // name is a WIN.INI [sounds] entry
      SND_FILENAME   = $00020000;   // name is a file name
      SND_RESOURCE   = $00040004;   // name is a resource name or atom

      SND_SYNC       = $00000000;   // play synchronously (default)
      SND_ASYNC      = $00000001;   // play asynchronously
      SND_NODEFAULT  = $00000002;   // silence not default, if sound not found
      SND_MEMORY     = $00000004;   // lpszSoundName points to a memory file
      SND_LOOP       = $00000008;   // loop the sound until next sndPlaySound
      SND_NOSTOP     = $00000010;   // don't stop any currently playing sound

      SND_NOWAIT     = $00002000;   // don't wait if the driver is busy
      SND_VALIDFLAGS = $0017201F;   // Set of valid flag bits.  Anything outside
                                    // this range will raise an error
      SND_RESERVED   = $FF000000;   // In particular these flags are reserved
      SND_TYPE_MASK  = $00170007;
      SND_ALIAS_ID   = $00110000;   // name is a WIN.INI [sounds] entry identifier

{$ENDIF} // MMNOSOUND


{$IFNDEF MMNOWAVE}
{****************************************************************************

                        Waveform audio support

****************************************************************************}
const
//* waveform audio error return values */
      WAVERR_BADFORMAT      = WAVERR_BASE + 0;    // unsupported wave format
      WAVERR_STILLPLAYING   = WAVERR_BASE + 1;    // still something playing
      WAVERR_UNPREPARED     = WAVERR_BASE + 2;    // header not prepared
      WAVERR_SYNC           = WAVERR_BASE + 3;    // device is synchronous
      WAVERR_LASTERROR      = WAVERR_BASE + 3;    // last error in range

//* waveform audio data types */
type
     HWAVE = HANDLE;
     HWAVEIN = HANDLE;
     LPHWAVEIN = ^HWAVEIN;
     HWAVEOUT = HANDLE;
     LPHWAVEOUT = ^HWAVEOUT;

     WAVECALLBACK = DRVCALLBACK;
     LPWAVECALLBACK = WAVECALLBACK;

const
//* wave callback messages */
      WOM_OPEN        = MM_WOM_OPEN;
      WOM_CLOSE       = MM_WOM_CLOSE;
      WOM_DONE        = MM_WOM_DONE;
      WIM_OPEN        = MM_WIM_OPEN;
      WIM_CLOSE       = MM_WIM_CLOSE;
      WIM_DATA        = MM_WIM_DATA;

//* device ID for wave device mapper */
const
      WAVE_MAPPER     = DWORD(-1);

//* flags for dwFlags parameter in waveOutOpen() and waveInOpen() */
const
       WAVE_FORMAT_QUERY     = $00000001;
       WAVE_ALLOWSYNC        = $00000002;
       WAVE_MAPPED           = $00000004;
       WAVE_FORMAT_DIRECT    = $00000008;
       WAVE_FORMAT_DIRECT_QUERY = WAVE_FORMAT_QUERY or WAVE_FORMAT_DIRECT;
       WAVE_NOMIXER          = $00000080;           //* Windows CE only - bypass software mixer */


// Switch to DWORD packing for WAVEHDR
// Warning: This assumes that the start of headers is DWORD aligned, which is a change from previous implementations.
{$PACKRECORDS 4}// #include "pshpack4.h"
//* wave data block header */
type
     LPWAVEHDR = ^wavehdr_tag;
     wavehdr_tag = record
       lpData:LPSTR;                 //* pointer to locked data buffer */
       dwBufferLength:DWORD;         //* length of data buffer */
       dwBytesRecorded:DWORD;        //* used for input only */
       dwUser:DWORD;                 //* for client's use */
       dwFlags:DWORD;                //* assorted flags (see defines) */
       dwLoops:DWORD;                //* loop control counter */
       lpNext:LPWAVEHDR;     //* reserved for driver */
       reserved:DWORD;               //* reserved for driver */
     end;
     WAVEHDR = wavehdr_tag;
     PWAVEHDR = ^wavehdr_tag;
     NPWAVEHDR = ^wavehdr_tag;

// Switch back to previous packing
{$PACKRECORDS 1}// #include "poppack.h"


//* flags for dwFlags field of WAVEHDR */
const
      WHDR_DONE       = $00000001;  //* done bit */
      WHDR_PREPARED   = $00000002;  //* set if this header has been prepared */
      WHDR_BEGINLOOP  = $00000004;  //* loop start block */
      WHDR_ENDLOOP    = $00000008;  //* loop end block */
      WHDR_INQUEUE    = $00000010;  //* reserved for driver */

//* waveform output device capabilities structure */
type
     tagWAVEOUTCAPS = record
       wMid:word;                  //* manufacturer ID */
       wPid:word;                  //* product ID */
       vDriverVersion:MMVERSION;      //* version of the driver */
       szPname:array[0..MAXPNAMELEN-1] of TCHAR;  //* product name (NULL terminated string) */
       dwFormats:DWORD;             //* formats supported */
       wChannels:word;             //* number of sources supported */
       wReserved1:word;            //* packing */
       dwSupport:DWORD;             //* functionality supported by driver */
     end;
     WAVEOUTCAPS = tagWAVEOUTCAPS;
     PWAVEOUTCAPS = ^tagWAVEOUTCAPS;
     NPWAVEOUTCAPS = ^tagWAVEOUTCAPS;
     LPWAVEOUTCAPS = ^tagWAVEOUTCAPS;

//* flags for dwSupport field of WAVEOUTCAPS */
const
      WAVECAPS_PITCH          = $0001;   //* supports pitch control */
      WAVECAPS_PLAYBACKRATE   = $0002;   //* supports playback rate control */
      WAVECAPS_VOLUME         = $0004;   //* supports volume control */
      WAVECAPS_LRVOLUME       = $0008;   //* separate left-right volume control */
//      WAVECAPS_SYNC           = $0010; //* Windows CE only supports asynchronous audio devices */
      WAVECAPS_SAMPLEACCURATE = $0020;
      WAVECAPS_DIRECTSOUND    = $0040;

//* waveform input device capabilities structure */
type
     tagWAVEINCAPS = record
       wMid:word;                    //* manufacturer ID */
       wPid:word;                    //* product ID */
       vDriverVersion:MMVERSION;     //* version of the driver */
       szPname:array[0..MAXPNAMELEN-1] of TCHAR;    //* product name (NULL terminated string) */
       dwFormats:DWORD;              //* formats supported */
       wChannels:word;               //* number of channels supported */
       wReserved1:word;              //* structure packing */
     end;
     WAVEINCAPS = tagWAVEINCAPS;
     PWAVEINCAPS = ^tagWAVEINCAPS;
     NPWAVEINCAPS = ^tagWAVEINCAPS;
     LPWAVEINCAPS = ^tagWAVEINCAPS;

//* defines for dwFormat field of WAVEINCAPS and WAVEOUTCAPS */
const
      WAVE_INVALIDFORMAT     = $00000000;       //* invalid format */
      WAVE_FORMAT_1M08       = $00000001;       //* 11.025 kHz, Mono,   8-bit  */
      WAVE_FORMAT_1S08       = $00000002;       //* 11.025 kHz, Stereo, 8-bit  */
      WAVE_FORMAT_1M16       = $00000004;       //* 11.025 kHz, Mono,   16-bit */
      WAVE_FORMAT_1S16       = $00000008;       //* 11.025 kHz, Stereo, 16-bit */
      WAVE_FORMAT_2M08       = $00000010;       //* 22.05  kHz, Mono,   8-bit  */
      WAVE_FORMAT_2S08       = $00000020;       //* 22.05  kHz, Stereo, 8-bit  */
      WAVE_FORMAT_2M16       = $00000040;       //* 22.05  kHz, Mono,   16-bit */
      WAVE_FORMAT_2S16       = $00000080;       //* 22.05  kHz, Stereo, 16-bit */
      WAVE_FORMAT_4M08       = $00000100;       //* 44.1   kHz, Mono,   8-bit  */
      WAVE_FORMAT_4S08       = $00000200;       //* 44.1   kHz, Stereo, 8-bit  */
      WAVE_FORMAT_4M16       = $00000400;       //* 44.1   kHz, Mono,   16-bit */
      WAVE_FORMAT_4S16       = $00000800;       //* 44.1   kHz, Stereo, 16-bit */

//* property information for audio gain classes */

//* audio gain class property sets */
const
      MM_PROPSET_GAINCLASS_CLASS:GUID = '{E7E569A5-8498-43FE-8075-33D1FDAB15EF}';
      MM_PROPSET_GAINCLASS_STREAM:GUID = '{40E953AE-EE3E-493A-93EE-DA3E30764390}';
(*
#define MM_PROPSET_GAINCLASS_CLASS \
    { 0xe7e569a5, 0x8498, 0x43fe, { 0x80, 0x75, 0x33, 0xd1, 0xfd, 0xab, 0x15, 0xef } }
#define MM_PROPSET_GAINCLASS_STREAM \
    { 0x40e953ae, 0xee3e, 0x493a, { 0x93, 0xee, 0xda, 0x3e, 0x30, 0x76, 0x43, 0x90 } }
*)

const
//* MM_PROPSET_GAINCLASS_CLASS property IDs */
      MM_PROP_GAINCLASS_CLASS     = 1;

//* MM_PROPSET_GAINCLASS_STREAM property IDs */
      MM_PROP_GAINCLASS_STREAM    = 1;

//* structure to describe audio gain classes */
type
     tagAUDIOGAINCLASS = record
       dwPriority:DWORD;
       dwRelativeGain:DWORD;
     end;
     AUDIOGAINCLASS = tagAUDIOGAINCLASS;
     PAUDIOGAINCLASS = ^tagAUDIOGAINCLASS;
     CPAUDIOGAINCLASS = ^AUDIOGAINCLASS;

//* values for AUDIOGAINCLASS.dwPriority */
const
      WAGC_PRIORITY_CRITICAL  = 6;
      WAGC_PRIORITY_HIGH      = 4;
      WAGC_PRIORITY_NORMAL    = 2;

//* wave stream properties structure */
type
     tagSTREAMPROPS = record
       dwClassID:DWORD;
       dwFlags:DWORD;
     end;
     STREAMPROPS = tagSTREAMPROPS;
     PSTREAMPROPS = ^tagSTREAMPROPS;
     CPSTREAMPROPS = ^STREAMPROPS;

//* flag definitions for STREAMPROPS.dwFlags */
const
      SPSFL_NOTIFY_GAIN_CHANGE   = $00000001;      //* callback gets MM_WOM_ATTENUATED messages */
      SPSFL_EXCLUDE              = $00000002;      //* stream does not participate in gain-class operations */

//* values for STREAMPROPS.dwClassid */
const
      WAGC_CLASS_DEFAULT              = 0;
      WAGC_CLASS_NORMAL               = 1;
      WAGC_CLASS_AUDIO_ALERT          = 2;
      WAGC_CLASS_SPEECH_ALERT         = 3;
      WAGC_CLASS_SPEECH_NOTIFICATION  = 4;
      WAGC_CLASS_AUDIO_NOTIFICATION   = 5;
      WAGC_CLASS_FORCE_MUTE           = 6;
//* classes 7-15 are reserved for future use */
      WAGC_CLASS_MAX                  = 15; //* class ID's larger than this are illegal */

//* Discontinuity detection property set */
     MM_PROPSET_DISCONTINUITY:GUID = '{B4389733-868E-4D06-B008-9A6FC8CE852E}';
{
#define MM_PROPSET_DISCONTINUITY \
    { 0xb4389733, 0x868e, 0x4d06, { 0xb0, 0x8, 0x9a, 0x6f, 0xc8, 0xce, 0x85, 0x2e } }
}

//* MM_PROPSET_DISCONTINUITY property IDs */
     MM_PROP_DISCONTINUITY   = 1;

//* bit values for MM_PROP_DISCONTINUITY parameter */
      WDSC_APP        = 1;
      WDSC_SWMIXER    = 2;
      WDSC_DRIVER     = 4;

//* structure to receive discontinuity information */
type
     tagAUDIODISCONTINUITY = record
       dwMask:DWORD;       // bitmask shows which values were set (see WDSC_* values above)
       dwApp:DWORD;        // # discontinuities detected at the application level
       dwSwMixer:DWORD;    // # discontinuities detected at the software mixer level
       dwDriver:DWORD;     // # discontinuities detected at the driver level
     end;
     AUDIODISCONTINUITY = tagAUDIODISCONTINUITY;
     PAUDIODISCONTINUITY = ^tagAUDIODISCONTINUITY;

//* waveform audio function prototypes */
function waveOutGetNumDevs:UINT; external KernelDLL name 'waveOutGetNumDevs'; // index 268
function waveOutGetDevCaps(uDeviceID:UINT; pwoc:LPWAVEOUTCAPS; cbwoc:UINT):MMRESULT; external KernelDLL name 'waveOutGetDevCaps'; // index 269
function waveOutGetVolume(hwo:HWAVEOUT; pdwVolume:LPDWORD):MMRESULT; external KernelDLL name 'waveOutGetVolume'; // index 26A
function waveOutSetVolume(hwo:HWAVEOUT; dwVolume:DWORD):MMRESULT; external KernelDLL name 'waveOutSetVolume'; // index 26B
function waveOutGetErrorText(mmrError:MMRESULT; pszText:PTSTR; cchText:UINT):MMRESULT; external KernelDLL name 'waveOutGetErrorText'; // index 26C
function waveOutClose(hwo:HWAVEOUT):MMRESULT; external KernelDLL name 'waveOutClose'; // index 26D
function waveOutPrepareHeader(hwo:HWAVEOUT; pwh:LPWAVEHDR; cbwh:UINT):MMRESULT; external KernelDLL name 'waveOutPrepareHeader'; // index 26E
function waveOutUnprepareHeader(hwo:HWAVEOUT; pwh:LPWAVEHDR; cbwh:UINT):MMRESULT; external KernelDLL name 'waveOutUnprepareHeader'; // index 26F
function waveOutWrite(hwo:HWAVEOUT; pwh:LPWAVEHDR; cbwh:UINT):MMRESULT; external KernelDLL name 'waveOutWrite'; // index 270
function waveOutPause(hwo:HWAVEOUT):MMRESULT; external KernelDLL name 'waveOutPause'; // index 271
function waveOutRestart(hwo:HWAVEOUT):MMRESULT; external KernelDLL name 'waveOutRestart'; // index 272
function waveOutReset(hwo:HWAVEOUT):MMRESULT; external KernelDLL name 'waveOutReset'; // index 273
function waveOutBreakLoop(hwo:HWAVEOUT):MMRESULT; external KernelDLL name 'waveOutBreakLoop'; // index 274
function waveOutGetPosition(hwo:HWAVEOUT; pmmt:LPMMTIME; cbmmt:UINT):MMRESULT; external KernelDLL name 'waveOutGetPosition'; // index 275
function waveOutGetPitch(hwo:HWAVEOUT; pdwPitch:LPDWORD):MMRESULT; external KernelDLL name 'waveOutGetPitch'; // index 276
function waveOutSetPitch(hwo:HWAVEOUT; dwPitch:DWORD):MMRESULT; external KernelDLL name 'waveOutSetPitch'; // index 277
function waveOutGetPlaybackRate(hwo:HWAVEOUT; pdwRate:LPDWORD):MMRESULT; external KernelDLL name 'waveOutGetPlaybackRate'; // index 278
function waveOutSetPlaybackRate(hwo:HWAVEOUT; dwRate:DWORD):MMRESULT; external KernelDLL name 'waveOutSetPlaybackRate'; // index 279
function waveOutGetID(hwo:HWAVEOUT; puDeviceID:LPUINT):MMRESULT; external KernelDLL name 'waveOutGetID'; // index 27A
function waveOutMessage(hwo:HWAVEOUT; uMsg:UINT; dw1:DWORD; dw2:DWORD):MMRESULT; external KernelDLL name 'waveOutMessage'; // index 27B
function waveOutOpen(Phwo:LPHWAVEOUT;
                     uDeviceID:UINT;
                     pwfx:LPCWAVEFORMATEX;
                     dwCallback:DWORD;
                     dwInstance:DWORD;
                     fdwOpen:DWORD):MMRESULT; external KernelDLL name 'waveOutOpen'; // index 27C
function waveOutGetProperty(uDeviceID:UINT;
                            pPropSetId:PGUID;
                            ulPropId:ULONG;
                            pvPropParams:LPVOID;
                            cbPropParams:ULONG;
                            pvPropData:LPVOID;
                            cbPropData:ULONG;
                            pcbReturn:PULONG):MMRESULT; external KernelDLL name 'waveOutGetProperty'; // index ?
function waveOutSetProperty(uDeviceID:UINT;
                            pPropSetId:PGUID;
                            ulPropId:ULONG;
                            pvPropParams:LPVOID;
                            cbPropParams:ULONG;
                            pvPropData:LPVOID;
                            cbPropData:ULONG):MMRESULT; external KernelDLL name 'waveOutSetProperty'; // index ?


function waveInGetNumDevs:UINT; external KernelDLL name 'waveInGetNumDevs'; // index 27D
function waveInGetDevCaps(uDeviceID:UINT; pwic:LPWAVEINCAPS; cbwic:UINT):MMRESULT; external KernelDLL name 'waveInGetDevCaps'; // index 27E
function waveInGetErrorText(mmrError:MMRESULT; pszText:LPTSTR; cchText:UINT):MMRESULT; external KernelDLL name 'waveInGetErrorText'; // index 27F
function waveInClose(hwi:HWAVEIN):MMRESULT; external KernelDLL name 'waveInClose'; // index 280
function waveInPrepareHeader(hwi:HWAVEIN; pwh:LPWAVEHDR; cbwh:UINT):MMRESULT; external KernelDLL name 'waveInPrepareHeader'; // index 281
function waveInUnprepareHeader(hwi:HWAVEIN; pwh:LPWAVEHDR; cbwh:UINT):MMRESULT; external KernelDLL name 'waveInUnprepareHeader'; // index 282
function waveInAddBuffer(hwi:HWAVEIN; pwh:LPWAVEHDR; cbwh:UINT):MMRESULT; external KernelDLL name 'waveInAddBuffer'; // index 283
function waveInStart(hwi:HWAVEIN):MMRESULT; external KernelDLL name 'waveInStart'; // index 284
function waveInStop(hwi:HWAVEIN):MMRESULT; external KernelDLL name 'waveInStop'; // index 285
function waveInReset(hwi:HWAVEIN):MMRESULT; external KernelDLL name 'waveInReset'; // index 286
function waveInGetPosition(hwi:HWAVEIN; pmmt:LPMMTIME; cbmmt:UINT):MMRESULT; external KernelDLL name 'waveInGetPosition'; // index 287
function waveInGetID(hwi:HWAVEIN; puDeviceID:LPUINT):MMRESULT; external KernelDLL name 'waveInGetID'; // index 288
function waveInMessage(hwi:HWAVEIN; uMsg:UINT; dw1:DWORD; dw2:DWORD):MMRESULT; external KernelDLL name 'waveInMessage'; // index 289
function waveInOpen(phwi:LPHWAVEIN;
                    uDeviceID:UINT;
                    pwfx:LPCWAVEFORMATEX;
                    dwCallback:DWORD;
                    dwInstance:DWORD;
                    fdwOpen:DWORD):MMRESULT; external KernelDLL name 'waveInOpen'; // index 28A
function waveInGetProperty(uDeviceID:UINT;
                           pPropSetId:PGUID;
                           ulPropId:ULONG;
                           pvPropParams:LPVOID;
                           cbPropParams:ULONG;
                           pvPropData:LPVOID;
                           cbPropData:ULONG;
                           pcbReturn:PULONG):MMRESULT; external KernelDLL name 'waveInGetProperty'; // index ?
function waveInSetProperty(uDeviceID:UINT;
                           pPropSetId:PGUID;
                           ulPropId:ULONG;
                           pvPropParams:LPVOID;
                           cbPropParams:ULONG;
                           pvPropData:LPVOID;
                           cbPropData:ULONG):MMRESULT; external KernelDLL name 'waveInSetProperty'; // index ?

{$ENDIF} // MMNOWAVE


{$IFNDEF MMNOMIDI}
{****************************************************************************

                            MIDI audio support

****************************************************************************}
const
//* MIDI error return values */
      MIDIERR_UNPREPARED    = MIDIERR_BASE + 0;   //* header not prepared */
      MIDIERR_STILLPLAYING  = MIDIERR_BASE + 1;   //* still something playing */
      MIDIERR_NOMAP         = MIDIERR_BASE + 2;   //* no configured instruments */
      MIDIERR_NOTREADY      = MIDIERR_BASE + 3;   //* hardware is still busy */
      MIDIERR_NODEVICE      = MIDIERR_BASE + 4;   //* port no longer connected */
      MIDIERR_INVALIDSETUP  = MIDIERR_BASE + 5;   //* invalid MIF */
      MIDIERR_BADOPENMODE   = MIDIERR_BASE + 6;   //* operation unsupported w/ open mode */
      MIDIERR_DONT_CONTINUE = MIDIERR_BASE + 7;   //* thru device 'eating' a message */
      MIDIERR_LASTERROR     = MIDIERR_BASE + 7;   //* last error in range */

//* MIDI audio data types */
type
     HMIDI = HANDLE;
     HMIDIIN = HANDLE;
     HMIDIOUT = HANDLE;
     HMIDISTRM = HANDLE;

     LPHMIDI = ^HMIDI;
     LPHMIDIIN = ^HMIDIIN;
     LPHMIDIOUT = ^HMIDIOUT;
     LPHMIDISTRM = ^HMIDISTRM;

     MIDICALLBACK = DRVCALLBACK;
     LPMIDICALLBACK = MIDICALLBACK;

const
      MIDIPATCHSIZE   = 128;
      
type
     PATCHARRAY = array[0..MIDIPATCHSIZE-1] of word;
     LPPATCHARRAY = ^PATCHARRAY;
     KEYARRAY = array[0..MIDIPATCHSIZE-1] of word;
     LPKEYARRAY = ^KEYARRAY;

//* MIDI callback messages */
const
      MIM_OPEN        = MM_MIM_OPEN;
      MIM_CLOSE       = MM_MIM_CLOSE;
      MIM_DATA        = MM_MIM_DATA;
      MIM_LONGDATA    = MM_MIM_LONGDATA;
      MIM_ERROR       = MM_MIM_ERROR;
      MIM_LONGERROR   = MM_MIM_LONGERROR;
      MOM_OPEN        = MM_MOM_OPEN;
      MOM_CLOSE       = MM_MOM_CLOSE;
      MOM_DONE        = MM_MOM_DONE;

      MIM_MOREDATA      = MM_MIM_MOREDATA;
      MOM_POSITIONCB    = MM_MOM_POSITIONCB;

//* device ID for MIDI mapper */
const
      MIDIMAPPER     = UINT(-1);
      MIDI_MAPPER    = UINT(-1);

//* flags for dwFlags parm of midiInOpen() */
const
      MIDI_IO_STATUS      = $00000020;
      MIDI_IO_CONTROL     = $00000008;                         //* Internal */
      MIDI_IO_INPUT       = $00000010;  //*future*/             /* Internal */
      MIDI_IO_OWNED       = $00004000;                         //* Internal */
      MIDI_IO_SHARED      = $00008000;                         //* Internal */
      MIDI_I_VALID        = $C027;                              //* Internal */
      MIDI_O_VALID        = $C00E;                              //* Internal */


//* flags for wFlags parm of midiOutCachePatches(), midiOutCacheDrumPatches() */
      MIDI_CACHE_ALL      = 1;
      MIDI_CACHE_BESTFIT  = 2;
      MIDI_CACHE_QUERY    = 3;
      MIDI_UNCACHE        = 4;
      MIDI_CACHE_VALID    = MIDI_CACHE_ALL or MIDI_CACHE_BESTFIT or MIDI_CACHE_QUERY or MIDI_UNCACHE;     //* Internal */


//* MIDI output device capabilities structure */
{$IFDEF _WIN32}
type
     tagMIDIOUTCAPSA = record
       wMid:word;                  //* manufacturer ID */
       wPid:word;                  //* product ID */
       vDriverVersion:MMVERSION;      //* version of the driver */
       szPname:array[0..MAXPNAMELEN-1] of AnsiChar;  //* product name (NULL terminated string) */
       wTechnology:word;           //* type of device */
       wVoices:word;               //* # of voices (internal synth only) */
       wNotes:word;                //* max # of notes (internal synth only) */
       wChannelMask:word;          //* channels used (internal synth only) */
       dwSupport:DWORD;             //* functionality supported by driver */
     end;
     MIDIOUTCAPSA = tagMIDIOUTCAPSA;
     PMIDIOUTCAPSA = ^tagMIDIOUTCAPSA;
     NPMIDIOUTCAPSA = ^tagMIDIOUTCAPSA;
     LPMIDIOUTCAPSA = ^tagMIDIOUTCAPSA;

type
     tagMIDIOUTCAPSW = record
       wMid:word;                  //* manufacturer ID */
       wPid:word;                  //* product ID */
       vDriverVersion:MMVERSION;      //* version of the driver */
       szPname:array[0..MAXPNAMELEN-1] of WCHAR;  //* product name (NULL terminated string) */
       wTechnology:word;           //* type of device */
       wVoices:word;               //* # of voices (internal synth only) */
       wNotes:word;                //* max # of notes (internal synth only) */
       wChannelMask:word;          //* channels used (internal synth only) */
       dwSupport:DWORD;             //* functionality supported by driver */
     end;
     MIDIOUTCAPSW = tagMIDIOUTCAPSW;
     PMIDIOUTCAPSW = ^tagMIDIOUTCAPSW;
     NPMIDIOUTCAPSW = ^tagMIDIOUTCAPSW;
     LPMIDIOUTCAPSW = ^tagMIDIOUTCAPSW;

{$IFDEF UNICODE}
     MIDIOUTCAPS = MIDIOUTCAPSW;
     PMIDIOUTCAPS = PMIDIOUTCAPSW;
     NPMIDIOUTCAPS = NPMIDIOUTCAPSW;
     LPMIDIOUTCAPS = LPMIDIOUTCAPSW;
{$ELSE} // UNICODE
     MIDIOUTCAPS = MIDIOUTCAPSA;
     PMIDIOUTCAPS = PMIDIOUTCAPSA;
     NPMIDIOUTCAPS = NPMIDIOUTCAPSA;
     LPMIDIOUTCAPS = LPMIDIOUTCAPSA;
{$ENDIF} // UNICODE

{$ELSE} // _WIN32
type
     midioutcaps_tag = record
       wMid:word;                  //* manufacturer ID */
       wPid:word;                  //* product ID */
       vDriverVersion:MMVERSION;        //* version of the driver */
       szPname:array[0..MAXPNAMELEN-1] of char;  //* product name (NULL terminated string) */
       wTechnology:word;           //* type of device */
       wVoices:word;               //* # of voices (internal synth only) */
       wNotes:word;                //* max # of notes (internal synth only) */
       wChannelMask:word;          //* channels used (internal synth only) */
       dwSupport:DWORD;             //* functionality supported by driver */
     end;
     MIDIOUTCAPS = midioutcaps_tag;
     PMIDIOUTCAPS = ^midioutcaps_tag;
     NPMIDIOUTCAPS = ^midioutcaps_tag;
     LPMIDIOUTCAPS = ^midioutcaps_tag;
{$ENDIF} // _WIN32

const
//* flags for wTechnology field of MIDIOUTCAPS structure */
      MOD_MIDIPORT    = 1;  //* output port */
      MOD_SYNTH       = 2;  //* generic internal synth */
      MOD_SQSYNTH     = 3;  //* square wave internal synth */
      MOD_FMSYNTH     = 4;  //* FM internal synth */
      MOD_MAPPER      = 5;  //* MIDI mapper */

//* flags for dwSupport field of MIDIOUTCAPS structure */
      MIDICAPS_VOLUME          = $0001;  //* supports volume control */
      MIDICAPS_LRVOLUME        = $0002;  //* separate left-right volume control */
      MIDICAPS_CACHE           = $0004;

      MIDICAPS_STREAM          = $0008;  //* driver supports midiStreamOut directly */



//* MIDI input device capabilities structure */
{$IFDEF _WIN32}
type
     tagMIDIINCAPSA = record
       wMid:word;                   //* manufacturer ID */
       wPid:word;                   //* product ID */
       vDriverVersion:MMVERSION;         //* version of the driver */
       szPname:array[0..MAXPNAMELEN-1] of AnsiChar;   //* product name (NULL terminated string) */
//#if (WINVER >= 0x0400)
       dwSupport:DWORD;             //* functionality supported by driver */
//#endif
     end;
     MIDIINCAPSA = tagMIDIINCAPSA;
     PMIDIINCAPSA = ^tagMIDIINCAPSA;
     NPMIDIINCAPSA = ^tagMIDIINCAPSA;
     LPMIDIINCAPSA = ^tagMIDIINCAPSA;

type
     tagMIDIINCAPSW = record
       wMid:word;                   //* manufacturer ID */
       wPid:word;                   //* product ID */
       vDriverVersion:MMVERSION;         //* version of the driver */
       szPname:array[0..MAXPNAMELEN-1] of WCHAR;   //* product name (NULL terminated string) */
//#if (WINVER >= 0x0400)
       dwSupport:DWORD;             //* functionality supported by driver */
//#endif
     end;
     MIDIINCAPSW = tagMIDIINCAPSW;
     PMIDIINCAPSW = ^tagMIDIINCAPSW;
     NPMIDIINCAPSW = ^tagMIDIINCAPSW;
     LPMIDIINCAPSW = ^tagMIDIINCAPSW;

{$IFDEF UNICODE}
     MIDIINCAPS = MIDIINCAPSW;
     PMIDIINCAPS = PMIDIINCAPSW;
     NPMIDIINCAPS = NPMIDIINCAPSW;
     LPMIDIINCAPS = LPMIDIINCAPSW;
{$ELSE} // UNICODE
     MIDIINCAPS = MIDIINCAPSA;
     PMIDIINCAPS = PMIDIINCAPSA;
     NPMIDIINCAPS = NPMIDIINCAPSA;
     LPMIDIINCAPS = LPMIDIINCAPSA;
{$ENDIF} // UNICODE

{$ELSE} // _WIN32
type
     midiincaps_tag = record
       wMid:word;                  //* manufacturer ID */
       wPid:word;                  //* product ID */
       vDriverVersion:MMVERSION;        //* version of the driver */
       szPname:array[0..MAXPNAMELEN-1] of char;  //* product name (NULL terminated string) */
//#if (WINVER >= 0x0400)
       dwSupport:DWORD;             //* functionality supported by driver */
//#endif
     end;
     MIDIINCAPS = midiincaps_tag;
     PMIDIINCAPS = ^midiincaps_tag;
     NPMIDIINCAPS = ^midiincaps_tag;
     LPMIDIINCAPS = ^midiincaps_tag;
{$ENDIF} // _WIN32


//* MIDI data block header */
type
     LPMIDIHDR = ^midihdr_tag;
     midihdr_tag = record
       lpData:LPSTR;               //* pointer to locked data block */
       dwBufferLength:DWORD;       //* length of data in data block */
       dwBytesRecorded:DWORD;      //* used for input only */
       dwUser:DWORD;               //* for client's use */
       dwFlags:DWORD;              //* assorted flags (see defines) */
       lpNext:LPMIDIHDR;           //* reserved for driver */
       reserved:DWORD;             //* reserved for driver */
//#if (WINVER >= 0x0400)
       dwOffset:DWORD;             //* Callback offset into buffer */
       dwReserved:array[0..7] of DWORD;        //* Reserved for MMSYSTEM */
//#endif
     end;
     MIDIHDR = midihdr_tag;
     PMIDIHDR = ^midihdr_tag;
     NPMIDIHDR = ^midihdr_tag;

type
     midievent_tag = record
       dwDeltaTime:DWORD;          //* Ticks since last event */
       dwStreamID:DWORD;           //* Reserved; must be zero */
       dwEvent:DWORD;              //* Event type and parameters */
       dwParms:array[0..0] of DWORD;  //* Parameters if this is a long event */
     end;
     MIDIEVENT = midievent_tag;

type
     midistrmbuffver_tag = record
       dwVersion:DWORD;                  //* Stream buffer format version */
       dwMid:DWORD;                      //* Manufacturer ID as defined in MMREG.H */
       dwOEMVersion:DWORD;               //* Manufacturer version for custom ext */
     end;
     MIDISTRMBUFFVER = midistrmbuffver_tag;

const
//* flags for dwFlags field of MIDIHDR structure */
      MHDR_DONE       = $00000001;       //* done bit */
      MHDR_PREPARED   = $00000002;       //* set if header prepared */
      MHDR_INQUEUE    = $00000004;       //* reserved for driver */
      MHDR_ISSTRM     = $00000008;       //* Buffer is stream buffer */
      MHDR_SENDING    = $00000020;       //* Internal */
      MHDR_MAPPED     = $00001000;       //* thunked header */   /* Internal */
      MHDR_SHADOWHDR  = $00002000;       //* MIDIHDR is 16-bit shadow */ /* Internal */
      MHDR_VALID      = $e000302F;       //* valid flags */      /* Internal */

      MHDR_SAVE       = $e0003000;       //* Save these flags */  /* Internal */
                                         //* past driver calls */ /* Internal */

//* Dreamcast-only flags for tonebank support */
      MHDR_WRITEABLE    = $e0000000;
      MHDR_GENERALMIDI  = $80000000;     //* Indicates a tonebank that is to be used */
                                         //* by General MIDI stream ports */
      MHDR_GMDRUM       = $40000000;     //* In combination with GENERALMIDI, indicates */
                                         //* a GM drum tonebank.  By itself, indicates an */
                                         //* "alternate" GM drum tonebank */
      MHDR_MAPPINGTABLE = $20000000;     //* Indicates that this MIDIHDR is for a */
                                         //* program mapping table rather than an actual */
                                         //* tonebank */

//* */
//* Type codes which go in the high byte of the event DWORD of a stream buffer */
//* */
//* Type codes 00-7F contain parameters within the low 24 bits */
//* Type codes 80-FF contain a length of their parameter in the low 24 */
//* bits, followed by their parameter data in the buffer. The event */
//* DWORD contains the exact byte length; the parm data itself must be */
//* padded to be an even multiple of 4 bytes long. */
//* */
const
      MEVT_F_SHORT        = $00000000;
      MEVT_F_LONG         = $80000000;
      MEVT_F_CALLBACK     = $40000000;

      MEVT_SHORTMSG       = byte($00);    //* parm = shortmsg for midiOutShortMsg */
      MEVT_TEMPO          = byte($01);    //* parm = new tempo in microsec/qn     */
      MEVT_NOP            = byte($02);    //* parm = unused; does nothing         */

function MEVT_EVENTTYPE(x:DWORD):byte;
function MEVT_EVENTPARM(x:DWORD):DWORD;

//* 0x04-0x7F reserved */
const
      MEVT_LONGMSG        = byte($80);    //* parm = bytes to send verbatim       */
      MEVT_COMMENT        = byte($82);    //* parm = comment data                 */
      MEVT_VERSION        = byte($84);    //* parm = MIDISTRMBUFFVER struct       */

//* 0x81-0xFF reserved */

      MIDISTRM_ERROR      = -2;

//* */
//* Structures and defines for midiStreamProperty */
//* */
      MIDIPROP_SET        = $80000000;
      MIDIPROP_GET        = $40000000;

//* These are intentionally both non-zero so the app cannot accidentally */
//* leave the operation off and happen to appear to work due to default */
//* action. */

function MIDIPROP_PROPERTY(mp:DWORD):DWORD;

const
      MIDIPROP_TIMEDIV     = 00000001;
      MIDIPROP_TEMPO       = 00000002;
      MIDIPROP_SMF         = 00000003;
      MIDIPROP_BUFFERED    = 00000004;
      MIDIPROP_LOOPING     = 00000005;
      MIDIPROP_GENERALMIDI = 00000006;

//* Some useful defines to help with Standard MIDI File Format stuff */
      SMF_TIMEDIV_QUARTERNOTE = 0;
      SMF_TIMEDIV_SECONDS     = 1;


function SMF_TIMEDIV(format:DWORD; division:DWORD):DWORD;
function SMF_TIMEDIV_SMPTE(smpte:DWORD; division:DWORD):DWORD;
function SMF_TIMEDIV_ISSMPTE(dw:DWORD):DWORD;
function SMF_TIMEDIV_GETSMPTE(dw:DWORD):byte;
function SMF_TIMEDIV_GETTPF(dw:DWORD):DWORD;
function SMF_TIMEDIV_GETTPQN(dw:DWORD):DWORD;

type
     midiproptimediv_tag = record
       cbStruct:DWORD;
       dwTimeDiv:DWORD;
     end;
     MIDIPROPTIMEDIV = midiproptimediv_tag;
     LPMIDIPROPTIMEDIV = ^midiproptimediv_tag;

type
     midiproptempo_tag = record
       cbStruct:DWORD;
       dwTempo:DWORD;
     end;
     MIDIPROPTEMPO = midiproptempo_tag;
     LPMIDIPROPTEMPO = ^midiproptempo_tag;

const
      MIDIPROP_PROPVAL    = $3FFFFFFF; //* Internal */

// The library exports midi-functions.
{$IFDEF _WIN32}
const
      MIDIDLL = 'winmm.dll';
{$ELSE _WIN32}
const
      MIDIDLL = KernelDLL;
{$ENDIF _WIN32}

//* MIDI function prototypes */
function midiOutGetNumDevs:UINT; external MIDIDLL name 'midiOutGetNumDevs';
function midiStreamOpen(phms:LPHMIDISTRM; puDeviceID:LPUINT; cMidi:DWORD; dwCallback:DWORD; dwInstance:DWORD; fdwOpen:DWORD):MMRESULT; external MIDIDLL name 'midiStreamOpen'; // index
function midiStreamClose(hms:HMIDISTRM):MMRESULT; external MIDIDLL name 'midiStreamClose'; // index

function midiStreamProperty(hms:HMIDISTRM; lppropdata:LPBYTE; dwProperty:DWORD):MMRESULT; external MIDIDLL name 'midiStreamProperty'; // index
function midiStreamPosition(hms:HMIDISTRM; lpmmt:LPMMTIME; cbmmt:UINT):MMRESULT; external MIDIDLL name 'midiStreamPosition'; // index

function midiStreamOut(hms:HMIDISTRM; pmh:LPMIDIHDR; cbmh:UINT):MMRESULT; external MIDIDLL name 'midiStreamOut'; // index
function midiStreamPause(hms:HMIDISTRM):MMRESULT; external MIDIDLL name 'midiStreamPause'; // index
function midiStreamRestart(hms:HMIDISTRM):MMRESULT; external MIDIDLL name 'midiStreamRestart'; // index
function midiStreamStop(hms:HMIDISTRM):MMRESULT; external MIDIDLL name 'midiStreamStop'; // index

{$IFDEF _WIN32}
function midiConnect(hmi:HMIDI; hmo:HMIDIOUT; pReserved:LPVOID):MMRESULT; external MIDIDLL name 'midiConnect'; // index
function midiDisconnect(hmi:HMIDI; hmo:HMIDIOUT; pReserved:LPVOID):MMRESULT; external MIDIDLL name 'midiDisconnect'; // index
{$ENDIF} // _WIN32


{$IFDEF _WIN32}
function midiOutGetDevCapsA(uDeviceID:UINT; pmoc:LPMIDIOUTCAPSA; cbmoc:UINT):MMRESULT; external MIDIDLL name 'midiOutGetDevCapsA'; // index
function midiOutGetDevCapsW(uDeviceID:UINT; pmoc:LPMIDIOUTCAPSW; cbmoc:UINT):MMRESULT; external MIDIDLL name 'midiOutGetDevCapsW'; // index
{$IFDEF UNICODE}
function midiOutGetDevCaps(uDeviceID:UINT; pmoc:LPMIDIOUTCAPSW; cbmoc:UINT):MMRESULT; external MIDIDLL name 'midiOutGetDevCapsW'; // index
{$ELSE} // UNICODE
function midiOutGetDevCaps(uDeviceID:UINT; pmoc:LPMIDIOUTCAPSA; cbmoc:UINT):MMRESULT; external MIDIDLL name 'midiOutGetDevCapsA'; // index
{$ENDIF} // UNICODE

{$ELSE} // _WIN32
function midiOutGetDevCaps(uDeviceID:UINT; pmoc:LPMIDIOUTCAPS; cbmoc:UINT):MMRESULT; external MIDIDLL name 'midiOutGetDevCaps'; // index
{$ENDIF} // _WIN32

function midiOutGetVolume(hmo:HMIDIOUT; pdwVolume:LPDWORD):MMRESULT; external MIDIDLL name 'midiOutGetVolume'; // index
function midiOutSetVolume(hmo:HMIDIOUT; dwVolume:DWORD):MMRESULT; external MIDIDLL name 'midiOutSetVolume'; // index

{$IFDEF _WIN32}
function midiOutGetErrorTextA(mmrError:MMRESULT; pszText:LPSTR; cchText:UINT):MMRESULT; external MIDIDLL name 'midiOutGetErrorTextA'; // index
function midiOutGetErrorTextW(mmrError:MMRESULT; pszText:LPWSTR; cchText:UINT):MMRESULT; external MIDIDLL name 'midiOutGetErrorTextW'; // index
{$IFDEF UNICODE}
function midiOutGetErrorText(mmrError:MMRESULT; pszText:LPWSTR; cchText:UINT):MMRESULT; external MIDIDLL name 'midiOutGetErrorTextW'; // index
{$ELSE} // UNICODE
function midiOutGetErrorText(mmrError:MMRESULT; pszText:LPSTR; cchText:UINT):MMRESULT; external MIDIDLL name 'midiOutGetErrorTextA'; // index
{$ENDIF} // UNICODE
{$ELSE} // _WIN32
function midiOutGetErrorText(mmrError:MMRESULT; pszText:LPSTR; cchText:UINT):MMRESULT; external MIDIDLL name 'midiOutGetErrorText'; // index
{$ENDIF} // _WIN32

function midiOutOpen(phmo:LPHMIDIOUT; uDeviceID:UINT;
    dwCallback:DWORD; dwInstance:DWORD; fdwOpen:DWORD):MMRESULT; external MIDIDLL name 'midiOutOpen'; // index
function midiOutClose(hmo:HMIDIOUT):MMRESULT; external MIDIDLL name 'midiOutClose'; // index
function midiOutPrepareHeader(hmo:HMIDIOUT; pmh:LPMIDIHDR; cbmh:UINT):MMRESULT; external MIDIDLL name 'midiOutPrepareHeader'; // index
function midiOutUnprepareHeader(hmo:HMIDIOUT; pmh:LPMIDIHDR; cbmh:UINT):MMRESULT; external MIDIDLL name 'midiOutUnprepareHeader'; // index
function midiOutShortMsg(hmo:HMIDIOUT; dwMsg:DWORD):MMRESULT; external MIDIDLL name 'midiOutShortMsg'; // index
function midiOutLongMsg(hmo:HMIDIOUT; pmh:LPMIDIHDR; cbmh:UINT):MMRESULT; external MIDIDLL name 'midiOutLongMsg'; // index
function midiOutReset(hmo:HMIDIOUT):MMRESULT; external MIDIDLL name 'midiOutReset'; // index
function midiOutCachePatches(hmo:HMIDIOUT; uBank:UINT; pwpa:LPWORD; fuCache:UINT):MMRESULT; external MIDIDLL name 'midiOutCachePatches'; // index
function midiOutCacheDrumPatches(hmo:HMIDIOUT; uPatch:UINT; pwkya:LPWORD; fuCache:UINT):MMRESULT; external MIDIDLL name 'midiOutCacheDrumPatches'; // index
function midiOutGetID(hmo:HMIDIOUT; puDeviceID:LPUINT):MMRESULT; external MIDIDLL name 'midiOutGetID'; // index

{$IFDEF _WIN32}
function midiOutMessage(hmo:HMIDIOUT; uMsg:UINT; dw1:DWORD; dw2:DWORD):MMRESULT; external MIDIDLL name 'midiOutMessage'; // index
{$ELSE} // _WIN32
function midiOutMessage(hmo:HMIDIOUT; uMsg:UINT; dw1:DWORD; dw2:DWORD):DWORD; external MIDIDLL name 'midiOutMessage'; // index
{$ENDIF} // _WIN32

function midiInGetNumDevs:UINT; external MIDIDLL name 'midiInGetNumDevs'; // index

{$IFDEF _WIN32}
function midiInGetDevCapsA(uDeviceID:UINT; pmic:LPMIDIINCAPSA; cbmic:UINT):MMRESULT; external MIDIDLL name 'midiInGetDevCapsA'; // index
function midiInGetDevCapsW(uDeviceID:UINT; pmic:LPMIDIINCAPSW; cbmic:UINT):MMRESULT; external MIDIDLL name 'midiInGetDevCapsW'; // index
{$IFDEF UNICODE}
function midiInGetDevCaps(uDeviceID:UINT; pmic:LPMIDIINCAPSW; cbmic:UINT):MMRESULT; external MIDIDLL name 'midiInGetDevCapsW'; // index
{$ELSE} // UNICODE
function midiInGetDevCaps(uDeviceID:UINT; pmic:LPMIDIINCAPSA; cbmic:UINT):MMRESULT; external MIDIDLL name 'midiInGetDevCapsA'; // index
{$ENDIF} // UNICODE

function midiInGetErrorTextA(mmrError:MMRESULT; pszText:LPSTR; cchText:UINT):MMRESULT; external MIDIDLL name 'midiInGetErrorTextA'; // index
function midiInGetErrorTextW(mmrError:MMRESULT; pszText:LPWSTR; cchText:UINT):MMRESULT; external MIDIDLL name 'midiInGetErrorTextW'; // index
{$IFDEF UNICODE}
function midiInGetErrorText(mmrError:MMRESULT; pszText:LPWSTR; cchText:UINT):MMRESULT; external MIDIDLL name 'midiInGetErrorTextW'; // index
{$ELSE} // UNICODE
function midiInGetErrorText(mmrError:MMRESULT; pszText:LPSTR; cchText:UINT):MMRESULT; external MIDIDLL name 'midiInGetErrorTextA'; // index
{$ENDIF} // UNICODE

{$ELSE} // _WIN32
function midiInGetDevCaps(uDeviceID:UINT; pmic:LPMIDIINCAPS; cbmic:UINT):MMRESULT; external MIDIDLL name 'midiInGetDevCaps'; // index
function midiInGetErrorText(mmrError:MMRESULT; pszText:LPSTR; cchText:UINT):MMRESULT; external MIDIDLL name 'midiInGetErrorText'; // index
{$ENDIF} // _WIN32

function midiInOpen(phmi:LPHMIDIIN; uDeviceID:UINT;
        dwCallback:DWORD; dwInstance:DWORD; fdwOpen:DWORD):MMRESULT; external MIDIDLL name 'midiInOpen'; // index
function midiInClose(hmi:HMIDIIN):MMRESULT; external MIDIDLL name 'midiInClose'; // index
function midiInPrepareHeader(hmi:HMIDIIN; pmh:LPMIDIHDR; cbmh:UINT):MMRESULT; external MIDIDLL name 'midiInPrepareHeader'; // index
function midiInUnprepareHeader(hmi:HMIDIIN; pmh:LPMIDIHDR; cbmh:UINT):MMRESULT; external MIDIDLL name 'midiInUnprepareHeader'; // index
function midiInAddBuffer(hmi:HMIDIIN; pmh:LPMIDIHDR; cbmh:UINT):MMRESULT; external MIDIDLL name 'midiInAddBuffer'; // index
function midiInStart(hmi:HMIDIIN):MMRESULT; external MIDIDLL name 'midiInStart'; // index
function midiInStop(hmi:HMIDIIN):MMRESULT; external MIDIDLL name 'midiInStop'; // index
function midiInReset(hmi:HMIDIIN):MMRESULT; external MIDIDLL name 'midiInReset'; // index
function midiInGetID(hmi:HMIDIIN; puDeviceID:LPUINT):MMRESULT; external MIDIDLL name 'midiInGetID'; // index

{$IFDEF _WIN32}
function midiInMessage(hmi:HMIDIIN; uMsg:UINT; dw1:DWORD; dw2:DWORD):MMRESULT; external MIDIDLL name 'midiInMessage'; // index
{$ELSE} // _WIN32
function midiInMessage(hmi:HMIDIIN; uMsg:UINT; dw1:DWORD; dw2:DWORD):DWORD; external MIDIDLL name 'midiInMessage'; // index
{$ENDIF} // _WIN32

{$ENDIF} // MMNOMIDI 



{$IFNDEF MMNOMIXER}
{****************************************************************************

                            Mixer Support

****************************************************************************}

type
     HMIXEROBJ = HANDLE;
     LPHMIXEROBJ = ^HMIXEROBJ;

     HMIXER = HANDLE;
     LPHMIXER = ^HMIXER;

const
      MIXER_SHORT_NAME_CHARS   = 16;
      MIXER_LONG_NAME_CHARS    = 64;

//* */
//*  MMRESULT error return values specific to the mixer API */
//* */
//* */
const
      MIXERR_INVALLINE            = MIXERR_BASE + 0;
      MIXERR_INVALCONTROL         = MIXERR_BASE + 1;
      MIXERR_INVALVALUE           = MIXERR_BASE + 2;
      MIXERR_LASTERROR            = MIXERR_BASE + 2;

      MIXER_OBJECTF_HANDLE    = $80000000;
      MIXER_OBJECTF_MIXER     = $00000000;
      MIXER_OBJECTF_HMIXER    = MIXER_OBJECTF_HANDLE or MIXER_OBJECTF_MIXER;
      MIXER_OBJECTF_WAVEOUT   = $10000000;
      MIXER_OBJECTF_HWAVEOUT  = MIXER_OBJECTF_HANDLE or MIXER_OBJECTF_WAVEOUT;
      MIXER_OBJECTF_WAVEIN    = $20000000;
      MIXER_OBJECTF_HWAVEIN   = MIXER_OBJECTF_HANDLE or MIXER_OBJECTF_WAVEIN;
      MIXER_OBJECTF_MIDIOUT   = $30000000;
      MIXER_OBJECTF_HMIDIOUT  = MIXER_OBJECTF_HANDLE or MIXER_OBJECTF_MIDIOUT;
      MIXER_OBJECTF_MIDIIN    = $40000000;
      MIXER_OBJECTF_HMIDIIN   = MIXER_OBJECTF_HANDLE or MIXER_OBJECTF_MIDIIN;
      MIXER_OBJECTF_AUX       = $50000000;

function mixerGetNumDevs:UINT; external KernelDLL name 'mixerGetNumDevs'; // index 2AE

type
     tagMIXERCAPS = record
       wMid:word;                   //* manufacturer id */
       wPid:word;                   //* product id */
       vDriverVersion:MMVERSION;    //* version of the driver */
       szPname:array[0..MAXPNAMELEN-1] of TCHAR;  //* product name */
       fdwSupport:DWORD;             //* misc. support bits */
       cDestinations:DWORD;          //* count of destinations */
     end;
     MIXERCAPS = tagMIXERCAPS;
     PMIXERCAPS = ^tagMIXERCAPS;
     LPMIXERCAPS = ^tagMIXERCAPS;

function mixerGetDevCaps(uMxId:UINT; pmxcaps:LPMIXERCAPS; cbmxcaps:UINT):MMRESULT; external KernelDLL name 'mixerGetDevCaps'; // index 2AA

function mixerOpen(phmx:LPHMIXER; uMxId:UINT; dwCallback:DWORD; dwInstance:DWORD; fdwOpen:DWORD):MMRESULT; external KernelDLL name 'mixerOpen'; // index 2B0

function mixerClose(hmx:HMIXER):MMRESULT; external KernelDLL name 'mixerClose'; // index 2B2

function mixerMessage(hmx:HMIXER; uMsg:UINT; dwParam1:DWORD; dwParam2:DWORD):DWORD; external KernelDLL name 'mixerMessage'; // index 2AF

type
     tMIXERLINE = record
       cbStruct:DWORD;               //* size of MIXERLINE structure */
       dwDestination:DWORD;          //* zero based destination index */
       dwSource:DWORD;               //* zero based source index (if source) */
       dwLineID:DWORD;               //* unique line id for mixer device */
       fdwLine:DWORD;                //* state/information about line */
       dwUser:DWORD;                 //* driver specific information */
       dwComponentType:DWORD;        //* component type line connects to */
       cChannels:DWORD;              //* number of channels line supports */
       cConnections:DWORD;           //* number of connections [possible] */
       cControls:DWORD;              //* number of controls at this line */
       szShortName:array[0..MIXER_SHORT_NAME_CHARS-1] of TCHAR;
       szName:array[0..MIXER_LONG_NAME_CHARS-1] of TCHAR;
       Target:record
         dwType:DWORD;                 //* MIXERLINE_TARGETTYPE_xxxx */
         dwDeviceID:DWORD;             //* target device ID of device type */
         wMid:word;                   //* of target device */
         wPid:word;                   //*      " */
         vDriverVersion:MMVERSION;         //*      " */
         szPname:array[0..MAXPNAMELEN-1] of TCHAR;   //*      " */
       end;
     end;
     MIXERLINE = tMIXERLINE;
     PMIXERLINE = ^tMIXERLINE;
     LPMIXERLINE = ^tMIXERLINE;

//* */
//*  MIXERLINE.fdwLine */
//* */
//* */
const
      MIXERLINE_LINEF_ACTIVE              = $00000001;
      MIXERLINE_LINEF_DISCONNECTED        = $00008000;
      MIXERLINE_LINEF_SOURCE              = $80000000;

//* */
//*  MIXERLINE.dwComponentType */
//* */
//*  component types for destinations and sources */
//* */
//* */
      MIXERLINE_COMPONENTTYPE_DST_FIRST       = $00000000;
      MIXERLINE_COMPONENTTYPE_DST_UNDEFINED   = MIXERLINE_COMPONENTTYPE_DST_FIRST + 0;
      MIXERLINE_COMPONENTTYPE_DST_DIGITAL     = MIXERLINE_COMPONENTTYPE_DST_FIRST + 1;
      MIXERLINE_COMPONENTTYPE_DST_LINE        = MIXERLINE_COMPONENTTYPE_DST_FIRST + 2;
      MIXERLINE_COMPONENTTYPE_DST_MONITOR     = MIXERLINE_COMPONENTTYPE_DST_FIRST + 3;
      MIXERLINE_COMPONENTTYPE_DST_SPEAKERS    = MIXERLINE_COMPONENTTYPE_DST_FIRST + 4;
      MIXERLINE_COMPONENTTYPE_DST_HEADPHONES  = MIXERLINE_COMPONENTTYPE_DST_FIRST + 5;
      MIXERLINE_COMPONENTTYPE_DST_TELEPHONE   = MIXERLINE_COMPONENTTYPE_DST_FIRST + 6;
      MIXERLINE_COMPONENTTYPE_DST_WAVEIN      = MIXERLINE_COMPONENTTYPE_DST_FIRST + 7;
      MIXERLINE_COMPONENTTYPE_DST_VOICEIN     = MIXERLINE_COMPONENTTYPE_DST_FIRST + 8;
      MIXERLINE_COMPONENTTYPE_DST_LAST        = MIXERLINE_COMPONENTTYPE_DST_FIRST + 8;

      MIXERLINE_COMPONENTTYPE_SRC_FIRST       = $00001000;
      MIXERLINE_COMPONENTTYPE_SRC_UNDEFINED   = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 0;
      MIXERLINE_COMPONENTTYPE_SRC_DIGITAL     = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 1;
      MIXERLINE_COMPONENTTYPE_SRC_LINE        = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 2;
      MIXERLINE_COMPONENTTYPE_SRC_MICROPHONE  = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 3;
      MIXERLINE_COMPONENTTYPE_SRC_SYNTHESIZER = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 4;
      MIXERLINE_COMPONENTTYPE_SRC_COMPACTDISC = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 5;
      MIXERLINE_COMPONENTTYPE_SRC_TELEPHONE   = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 6;
      MIXERLINE_COMPONENTTYPE_SRC_PCSPEAKER   = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 7;
      MIXERLINE_COMPONENTTYPE_SRC_WAVEOUT     = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 8;
      MIXERLINE_COMPONENTTYPE_SRC_AUXILIARY   = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 9;
      MIXERLINE_COMPONENTTYPE_SRC_ANALOG      = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 10;
      MIXERLINE_COMPONENTTYPE_SRC_LAST        = MIXERLINE_COMPONENTTYPE_SRC_FIRST + 10;

//* */
//*  MIXERLINE.Target.dwType */
//* */
//* */
      MIXERLINE_TARGETTYPE_UNDEFINED      = 0;
      MIXERLINE_TARGETTYPE_WAVEOUT        = 1;
      MIXERLINE_TARGETTYPE_WAVEIN         = 2;
      MIXERLINE_TARGETTYPE_MIDIOUT        = 3;
      MIXERLINE_TARGETTYPE_MIDIIN         = 4;
      MIXERLINE_TARGETTYPE_AUX            = 5;


function mixerGetLineInfo(hmxobj:HMIXEROBJ; pmxl:LPMIXERLINE; fdwInfo:DWORD):MMRESULT; external KernelDLL name 'mixerGetLineInfo'; // index 2AD

const
      MIXER_GETLINEINFOF_DESTINATION      = $00000000;
      MIXER_GETLINEINFOF_SOURCE           = $00000001;
      MIXER_GETLINEINFOF_LINEID           = $00000002;
      MIXER_GETLINEINFOF_COMPONENTTYPE    = $00000003;
      MIXER_GETLINEINFOF_TARGETTYPE       = $00000004;

      MIXER_GETLINEINFOF_QUERYMASK        = $0000000F;

function mixerGetID(hmxobj:HMIXEROBJ; puMxId:LPUINT; fdwId:DWORD):MMRESULT; external KernelDLL name 'mixerGetID'; // index 2AB

//* */
//*  MIXERCONTROL */
//* */
//* */
type
     tMIXERCONTROL = record
       cbStruct:DWORD;           //* size in bytes of MIXERCONTROL */
       dwControlID:DWORD;        //* unique control id for mixer device */
       dwControlType:DWORD;      //* MIXERCONTROL_CONTROLTYPE_xxx */
       fdwControl:DWORD;         //* MIXERCONTROL_CONTROLF_xxx */
       cMultipleItems:DWORD;     //* if MIXERCONTROL_CONTROLF_MULTIPLE set */
       szShortName:array[0..MIXER_SHORT_NAME_CHARS-1] of TCHAR;
       szName:array[0..MIXER_LONG_NAME_CHARS-1] of TCHAR;
       Bounds:record
         case DWORD of
           0: (lMinimum:LONG;           //* signed minimum for this control */
               lMaximum:LONG);           //* signed maximum for this control */
           1: (dwMinimum:DWORD;          //* unsigned minimum for this control */
               dwMaximum:DWORD);          //* unsigned maximum for this control */
           2: (dwReserved:array[0..5] of DWORD);
       end;
       Metrics:record
         case DWORD of
           0: (cSteps:DWORD);             //* # of steps between min & max */
           1: (cbCustomData:DWORD);       //* size in bytes of custom data */
           2: (dwReserved:array[0..5] of DWORD);
       end;
    end;
    MIXERCONTROL = tMIXERCONTROL;
    PMIXERCONTROL = ^tMIXERCONTROL;
    LPMIXERCONTROL = ^tMIXERCONTROL;

const    
//* */
//*  MIXERCONTROL.fdwControl */
//* */
//* */
      MIXERCONTROL_CONTROLF_UNIFORM   = $00000001;
      MIXERCONTROL_CONTROLF_MULTIPLE  = $00000002;
      MIXERCONTROL_CONTROLF_DISABLED  = $80000000;

//* */
//*  MIXERCONTROL_CONTROLTYPE_xxx building block defines */
//* */
//* */
      MIXERCONTROL_CT_CLASS_MASK          = $F0000000;
      MIXERCONTROL_CT_CLASS_CUSTOM        = $00000000;
      MIXERCONTROL_CT_CLASS_METER         = $10000000;
      MIXERCONTROL_CT_CLASS_SWITCH        = $20000000;
      MIXERCONTROL_CT_CLASS_NUMBER        = $30000000;
      MIXERCONTROL_CT_CLASS_SLIDER        = $40000000;
      MIXERCONTROL_CT_CLASS_FADER         = $50000000;
      MIXERCONTROL_CT_CLASS_TIME          = $60000000;
      MIXERCONTROL_CT_CLASS_LIST          = $70000000;

      MIXERCONTROL_CT_SUBCLASS_MASK       = $0F000000;

      MIXERCONTROL_CT_SC_SWITCH_BOOLEAN   = $00000000;
      MIXERCONTROL_CT_SC_SWITCH_BUTTON    = $01000000;

      MIXERCONTROL_CT_SC_METER_POLLED     = $00000000;

      MIXERCONTROL_CT_SC_TIME_MICROSECS   = $00000000;
      MIXERCONTROL_CT_SC_TIME_MILLISECS   = $01000000;

      MIXERCONTROL_CT_SC_LIST_SINGLE      = $00000000;
      MIXERCONTROL_CT_SC_LIST_MULTIPLE    = $01000000;

      MIXERCONTROL_CT_UNITS_MASK          = $00FF0000;
      MIXERCONTROL_CT_UNITS_CUSTOM        = $00000000;
      MIXERCONTROL_CT_UNITS_BOOLEAN       = $00010000;
      MIXERCONTROL_CT_UNITS_SIGNED        = $00020000;
      MIXERCONTROL_CT_UNITS_UNSIGNED      = $00030000;
      MIXERCONTROL_CT_UNITS_DECIBELS      = $00040000; //* in 10ths */
      MIXERCONTROL_CT_UNITS_PERCENT       = $00050000; //* in 10ths */

//* */
//*  Commonly used control types for specifying MIXERCONTROL.dwControlType */
//* */

      MIXERCONTROL_CONTROLTYPE_CUSTOM         = MIXERCONTROL_CT_CLASS_CUSTOM or MIXERCONTROL_CT_UNITS_CUSTOM;
      MIXERCONTROL_CONTROLTYPE_BOOLEANMETER   = MIXERCONTROL_CT_CLASS_METER or MIXERCONTROL_CT_SC_METER_POLLED or MIXERCONTROL_CT_UNITS_BOOLEAN;
      MIXERCONTROL_CONTROLTYPE_SIGNEDMETER    = MIXERCONTROL_CT_CLASS_METER or MIXERCONTROL_CT_SC_METER_POLLED or MIXERCONTROL_CT_UNITS_SIGNED;
      MIXERCONTROL_CONTROLTYPE_PEAKMETER      = MIXERCONTROL_CONTROLTYPE_SIGNEDMETER + 1;
      MIXERCONTROL_CONTROLTYPE_UNSIGNEDMETER  = MIXERCONTROL_CT_CLASS_METER or MIXERCONTROL_CT_SC_METER_POLLED or MIXERCONTROL_CT_UNITS_UNSIGNED;
      MIXERCONTROL_CONTROLTYPE_BOOLEAN        = MIXERCONTROL_CT_CLASS_SWITCH or MIXERCONTROL_CT_SC_SWITCH_BOOLEAN or MIXERCONTROL_CT_UNITS_BOOLEAN;
      MIXERCONTROL_CONTROLTYPE_ONOFF          = MIXERCONTROL_CONTROLTYPE_BOOLEAN + 1;
      MIXERCONTROL_CONTROLTYPE_MUTE           = MIXERCONTROL_CONTROLTYPE_BOOLEAN + 2;
      MIXERCONTROL_CONTROLTYPE_MONO           = MIXERCONTROL_CONTROLTYPE_BOOLEAN + 3;
      MIXERCONTROL_CONTROLTYPE_LOUDNESS       = MIXERCONTROL_CONTROLTYPE_BOOLEAN + 4;
      MIXERCONTROL_CONTROLTYPE_STEREOENH      = MIXERCONTROL_CONTROLTYPE_BOOLEAN + 5;

// - mmreg.h
      MIXERCONTROL_CONTROLTYPE_SRS_MTS                = MIXERCONTROL_CONTROLTYPE_BOOLEAN + 6;
      MIXERCONTROL_CONTROLTYPE_SRS_ONOFF              = MIXERCONTROL_CONTROLTYPE_BOOLEAN + 7;
      MIXERCONTROL_CONTROLTYPE_SRS_SYNTHSELECT        = MIXERCONTROL_CONTROLTYPE_BOOLEAN + 8;
// - end of mmreg.h

      MIXERCONTROL_CONTROLTYPE_BUTTON         = MIXERCONTROL_CT_CLASS_SWITCH or MIXERCONTROL_CT_SC_SWITCH_BUTTON or MIXERCONTROL_CT_UNITS_BOOLEAN;
      MIXERCONTROL_CONTROLTYPE_DECIBELS       = MIXERCONTROL_CT_CLASS_NUMBER or MIXERCONTROL_CT_UNITS_DECIBELS;
      MIXERCONTROL_CONTROLTYPE_SIGNED         = MIXERCONTROL_CT_CLASS_NUMBER or MIXERCONTROL_CT_UNITS_SIGNED;
      MIXERCONTROL_CONTROLTYPE_UNSIGNED       = MIXERCONTROL_CT_CLASS_NUMBER or MIXERCONTROL_CT_UNITS_UNSIGNED;
      MIXERCONTROL_CONTROLTYPE_PERCENT        = MIXERCONTROL_CT_CLASS_NUMBER or MIXERCONTROL_CT_UNITS_PERCENT;
      MIXERCONTROL_CONTROLTYPE_SLIDER         = MIXERCONTROL_CT_CLASS_SLIDER or MIXERCONTROL_CT_UNITS_SIGNED;
      MIXERCONTROL_CONTROLTYPE_PAN            = MIXERCONTROL_CONTROLTYPE_SLIDER + 1;
      MIXERCONTROL_CONTROLTYPE_QSOUNDPAN      = MIXERCONTROL_CONTROLTYPE_SLIDER + 2;
      MIXERCONTROL_CONTROLTYPE_FADER          = MIXERCONTROL_CT_CLASS_FADER or MIXERCONTROL_CT_UNITS_UNSIGNED;
      MIXERCONTROL_CONTROLTYPE_VOLUME         = MIXERCONTROL_CONTROLTYPE_FADER + 1;
      MIXERCONTROL_CONTROLTYPE_BASS           = MIXERCONTROL_CONTROLTYPE_FADER + 2;
      MIXERCONTROL_CONTROLTYPE_TREBLE         = MIXERCONTROL_CONTROLTYPE_FADER + 3;
      MIXERCONTROL_CONTROLTYPE_EQUALIZER      = MIXERCONTROL_CONTROLTYPE_FADER + 4;
      MIXERCONTROL_CONTROLTYPE_SINGLESELECT   = MIXERCONTROL_CT_CLASS_LIST or MIXERCONTROL_CT_SC_LIST_SINGLE or MIXERCONTROL_CT_UNITS_BOOLEAN;
      MIXERCONTROL_CONTROLTYPE_MUX            = MIXERCONTROL_CONTROLTYPE_SINGLESELECT + 1;
      MIXERCONTROL_CONTROLTYPE_MULTIPLESELECT = MIXERCONTROL_CT_CLASS_LIST or MIXERCONTROL_CT_SC_LIST_MULTIPLE or MIXERCONTROL_CT_UNITS_BOOLEAN;
      MIXERCONTROL_CONTROLTYPE_MIXER          = MIXERCONTROL_CONTROLTYPE_MULTIPLESELECT + 1;
      MIXERCONTROL_CONTROLTYPE_MICROTIME      = MIXERCONTROL_CT_CLASS_TIME or MIXERCONTROL_CT_SC_TIME_MICROSECS or MIXERCONTROL_CT_UNITS_UNSIGNED;
      MIXERCONTROL_CONTROLTYPE_MILLITIME      = MIXERCONTROL_CT_CLASS_TIME or MIXERCONTROL_CT_SC_TIME_MILLISECS or MIXERCONTROL_CT_UNITS_UNSIGNED;

//* */
//*  MIXERLINECONTROLS */
//* */
type
     tMIXERLINECONTROLS = record
       cbStruct:DWORD;       //* size in bytes of MIXERLINECONTROLS */
       dwLineID:DWORD;       //* line id (from MIXERLINE.dwLineID) */
       case DWORD of
         0: (dwControlID:DWORD); //* MIXER_GETLINECONTROLSF_ONEBYID */
         1: (dwControlType:DWORD;  //* MIXER_GETLINECONTROLSF_ONEBYTYPE */
             cControls:DWORD;      //* count of controls pmxctrl points to */
             cbmxctrl:DWORD;       //* size in bytes of _one_ MIXERCONTROL */
             pamxctrl:LPMIXERCONTROL);  //* pointer to first MIXERCONTROL array */
       end;
       MIXERLINECONTROLS = tMIXERLINECONTROLS;
       PMIXERLINECONTROLS = ^tMIXERLINECONTROLS;
       LPMIXERLINECONTROLS = ^tMIXERLINECONTROLS;

function mixerGetLineControls(hmxobj:HMIXEROBJ; pmxlc:LPMIXERLINECONTROLS; fdwControls:DWORD):MMRESULT; external KernelDLL name 'mixerGetLineControls'; // index 2AC

const
      MIXER_GETLINECONTROLSF_ALL          = $00000000;
      MIXER_GETLINECONTROLSF_ONEBYID      = $00000001;
      MIXER_GETLINECONTROLSF_ONEBYTYPE    = $00000002;

      MIXER_GETLINECONTROLSF_QUERYMASK    = $0000000F;

type
     tMIXERCONTROLDETAILS = record
       cbStruct:DWORD;       //* size in bytes of MIXERCONTROLDETAILS */
       dwControlID:DWORD;    //* control id to get/set details on */
       cChannels:DWORD;      //* number of channels in paDetails array */
       case DWORD of
         0: (hwndOwner:HWND);      //* for MIXER_SETCONTROLDETAILSF_CUSTOM */
         1: (cMultipleItems:DWORD; //* if _MULTIPLE, the number of items per channel */
             cbDetails:DWORD;      //* size of _one_ details_XX struct */
             paDetails:LPVOID);      //* pointer to array of details_XX structs */
     end;
     MIXERCONTROLDETAILS = tMIXERCONTROLDETAILS;
     PMIXERCONTROLDETAILS = ^tMIXERCONTROLDETAILS;
     LPMIXERCONTROLDETAILS = ^tMIXERCONTROLDETAILS;

//* */
//*  MIXER_GETCONTROLDETAILSF_LISTTEXT */
//* */
//* */
type
     tMIXERCONTROLDETAILS_LISTTEXT = record
       dwParam1:DWORD;
       dwParam2:DWORD;
       szName:array[0..MIXER_LONG_NAME_CHARS-1] of TCHAR;
     end;
     MIXERCONTROLDETAILS_LISTTEXT = tMIXERCONTROLDETAILS_LISTTEXT;
     PMIXERCONTROLDETAILS_LISTTEXT = ^tMIXERCONTROLDETAILS_LISTTEXT;
     LPMIXERCONTROLDETAILS_LISTTEXT = ^tMIXERCONTROLDETAILS_LISTTEXT;

//* */
//*  MIXER_GETCONTROLDETAILSF_VALUE */
//* */
//* */
type
     tMIXERCONTROLDETAILS_BOOLEAN = record
       fValue:LONG;
     end;
     MIXERCONTROLDETAILS_BOOLEAN = tMIXERCONTROLDETAILS_BOOLEAN;
     PMIXERCONTROLDETAILS_BOOLEAN = ^tMIXERCONTROLDETAILS_BOOLEAN;
     LPMIXERCONTROLDETAILS_BOOLEAN = ^tMIXERCONTROLDETAILS_BOOLEAN;

type
     tMIXERCONTROLDETAILS_SIGNED = record
       lValue:LONG;
     end;
     MIXERCONTROLDETAILS_SIGNED = tMIXERCONTROLDETAILS_SIGNED;
     PMIXERCONTROLDETAILS_SIGNED = ^tMIXERCONTROLDETAILS_SIGNED;
     LPMIXERCONTROLDETAILS_SIGNED = ^tMIXERCONTROLDETAILS_SIGNED;

type
     tMIXERCONTROLDETAILS_UNSIGNED = record
       dwValue:DWORD;
     end;
     MIXERCONTROLDETAILS_UNSIGNED = tMIXERCONTROLDETAILS_UNSIGNED;
     PMIXERCONTROLDETAILS_UNSIGNED = ^tMIXERCONTROLDETAILS_UNSIGNED;
     LPMIXERCONTROLDETAILS_UNSIGNED = ^tMIXERCONTROLDETAILS_UNSIGNED;

function mixerGetControlDetails(hmxobj:HMIXEROBJ; pmxcd:LPMIXERCONTROLDETAILS; fdwDetails:DWORD):MMRESULT; external KernelDLL name 'mixerGetControlDetails'; // index 2A9

const
      MIXER_GETCONTROLDETAILSF_VALUE      = $00000000;
      MIXER_GETCONTROLDETAILSF_LISTTEXT   = $00000001;

      MIXER_GETCONTROLDETAILSF_QUERYMASK  = $0000000F;

function mixerSetControlDetails(hmxobj:HMIXEROBJ; pmxcd:LPMIXERCONTROLDETAILS; fdwDetails:DWORD):MMRESULT; external KernelDLL name 'mixerSetControlDetails'; // index 2B1

const
      MIXER_SETCONTROLDETAILSF_VALUE      = $00000000;
      MIXER_SETCONTROLDETAILSF_CUSTOM     = $00000001;

      MIXER_SETCONTROLDETAILSF_QUERYMASK  = $0000000F;

{$ENDIF} // MMNOMIXER 


{$IFNDEF MMNOTIMER}
{****************************************************************************

                            Timer support

****************************************************************************}

//* timer error return values */
const
      TIMERR_NOERROR        = 0;                  //* no error */
      TIMERR_NOCANDO        = TIMERR_BASE+1;      //* request not completed */
      TIMERR_STRUCT         = TIMERR_BASE+33;     //* time struct size */

//* timer data types */
type
     TIMECALLBACK = procedure(uTimerID:UINT; uMsg:UINT; dwUser:DWORD; dw1:DWORD; dw2:DWORD); cdecl;
     LPTIMECALLBACK = TIMECALLBACK;

//* flags for fuEvent parameter of timeSetEvent() function */
const
      TIME_ONESHOT    = $0000;   //* program timer for single event */
      TIME_PERIODIC   = $0001;   //* program for continuous periodic event */

{.$IFDEF _WIN32}
      TIME_CALLBACK_FUNCTION      = $0000;  //* callback is function */
      TIME_CALLBACK_EVENT_SET     = $0010;  //* callback is event - use SetEvent */
      TIME_CALLBACK_EVENT_PULSE   = $0020;  //* callback is event - use PulseEvent */
      TIME_CALLBACK_TYPEMASK      = $00F0;  //* Internal */
{.$ENDIF} // _WIN32


//* timer device capabilities data structure */
type
     timecaps_tag = record
       wPeriodMin:UINT;     //* minimum period supported  */
       wPeriodMax:UINT;     //* maximum period supported  */
     end;
     TIMECAPS = timecaps_tag;
     PTIMECAPS = ^timecaps_tag;
     NPTIMECAPS = ^timecaps_tag;
     LPTIMECAPS = ^timecaps_tag;

{$IFDEF _WIN32}
const
      MMTimerDLL = 'winmm.dll';
{$ELSE _WIN32}
const
      MMTimerDLL = 'mmtimer.dll';
{$ENDIF _WIN32}

//* timer function prototypes */
function timeGetSystemTime(pmmt:LPMMTIME; cbmmt:UINT):MMRESULT; external MMTimerDLL name 'timeGetSystemTime'; // index
function timeGetTime:DWORD; external MMTimerDLL name 'timeGetTime'; // index
function timeSetEvent(uDelay:UINT;
                      uResolution:UINT;
                      fptc:LPTIMECALLBACK;
                      dwUser:DWORD;
                      fuEvent:UINT):MMRESULT; external MMTimerDLL name 'timeSetEvent'; // index
function timeKillEvent(uTimerID:UINT):MMRESULT; external MMTimerDLL name 'timeKillEvent'; // index
function timeGetDevCaps(ptc:LPTIMECAPS; cbtc:UINT):MMRESULT; external MMTimerDLL name 'timeGetDevCaps'; // index
function timeBeginPeriod(uPeriod:UINT):MMRESULT; external MMTimerDLL name 'timeBeginPeriod'; // index
function timeEndPeriod(uPeriod:UINT):MMRESULT; external MMTimerDLL name 'timeEndPeriod'; // index
function timeGetTimeSinceInterrupt:DWORD; external MMTimerDLL name 'timeGetTimeSinceInterrupt'; // index
function timeGetHardwareFrequency:DWORD; external MMTimerDLL name 'timeGetHardwareFrequency'; // index

{$ENDIF} // MMNOTIMER


{$IFNDEF MMNOMMIO}
{****************************************************************************

                        Multimedia File I/O support

****************************************************************************}

const
//* MMIO error return values */
      MMIOERR_BASE                = 256;
      MMIOERR_FILENOTFOUND        = MMIOERR_BASE + 1;  //* file not found */
      MMIOERR_OUTOFMEMORY         = MMIOERR_BASE + 2;  //* out of memory */
      MMIOERR_CANNOTOPEN          = MMIOERR_BASE + 3;  //* cannot open */
      MMIOERR_CANNOTCLOSE         = MMIOERR_BASE + 4;  //* cannot close */
      MMIOERR_CANNOTREAD          = MMIOERR_BASE + 5;  //* cannot read */
      MMIOERR_CANNOTWRITE         = MMIOERR_BASE + 6;  //* cannot write */
      MMIOERR_CANNOTSEEK          = MMIOERR_BASE + 7;  //* cannot seek */
      MMIOERR_CANNOTEXPAND        = MMIOERR_BASE + 8;  //* cannot expand file */
      MMIOERR_CHUNKNOTFOUND       = MMIOERR_BASE + 9;  //* chunk not found */
      MMIOERR_UNBUFFERED          = MMIOERR_BASE + 10; //*  */
      MMIOERR_PATHNOTFOUND        = MMIOERR_BASE + 11; //* path incorrect */
      MMIOERR_ACCESSDENIED        = MMIOERR_BASE + 12; //* file was protected */
      MMIOERR_SHARINGVIOLATION    = MMIOERR_BASE + 13; //* file in use */
      MMIOERR_NETWORKERROR        = MMIOERR_BASE + 14; //* network not responding */
      MMIOERR_TOOMANYOPENFILES    = MMIOERR_BASE + 15; //* no more file handles  */
      MMIOERR_INVALIDFILE         = MMIOERR_BASE + 16; //* default error file error */

//* MMIO constants */
      CFSEPCHAR       = '+';             //* compound file name separator char. */

//* MMIO data types */
type
      HPSTR = ^char;          //* a huge version of LPSTR */

      HMMIO = HANDLE;                  //* a handle to an open file */

type
     MMIOPROC = function(lpmmioinfo:LPSTR; uMsg:UINT; lParam1:LPARAM; lParam2:LPARAM):LRESULT; cdecl;
     LPMMIOPROC = MMIOPROC;

//* general MMIO information data structure */
type
     _MMIOINFO = record
      //* general fields */
       dwFlags:DWORD;        //* general status flags */
       fccIOProc:FOURCC;      //* pointer to I/O procedure */
       pIOProc:LPMMIOPROC;    //* pointer to I/O procedure */
       wErrorRet:UINT;      //* place for error to be returned */
       htask:HTASK;          //* alternate local task */

      //* fields maintained by MMIO functions during buffered I/O */
       cchBuffer:LONG;      //* size of I/O buffer (or 0L) */
       pchBuffer:HPSTR;      //* start of I/O buffer (or NULL) */
       pchNext:HPSTR;        //* pointer to next byte to read/write */
       pchEndRead:HPSTR;     //* pointer to last valid byte to read */
       pchEndWrite:HPSTR;    //* pointer to last byte to write */
       lBufOffset:LONG;     //* disk offset of start of buffer */

      //* fields maintained by I/O procedure */
       lDiskOffset:LONG;    //* disk offset of next read or write */
       adwInfo:array[0..2] of DWORD;     //* data specific to type of MMIOPROC */

      //* other fields maintained by MMIO */
       dwReserved1:DWORD;    //* reserved for MMIO use */
       dwReserved2:DWORD;    //* reserved for MMIO use */
       hmmio:HMMIO;          //* handle to open file */
     end;
     MMIOINFO = _MMIOINFO;
     PMMIOINFO = ^_MMIOINFO;
     NPMMIOINFO = ^_MMIOINFO;
     LPMMIOINFO = ^_MMIOINFO;
     LPCMMIOINFO = ^MMIOINFO;

//* RIFF chunk information data structure */
type
     _MMCKINFO = record
       ckid:FOURCC;           //* chunk ID */
       cksize:DWORD;         //* chunk size */
       fccType:FOURCC;        //* form type or list type */
       dwDataOffset:DWORD;   //* offset of data portion of chunk */
       dwFlags:DWORD;        //* flags used by MMIO functions */
     end;
     MMCKINFO = _MMCKINFO;
     PMMCKINFO = ^_MMCKINFO;
     NPMMCKINFO = ^_MMCKINFO;
     LPMMCKINFO = ^_MMCKINFO;
     LPCMMCKINFO = ^MMCKINFO;

const
//* bit field masks */
      MMIO_RWMODE     = $00000003;      //* open file for reading/writing/both */
      MMIO_SHAREMODE  = $00000070;      //* file sharing mode number */

//* read/write mode numbers (bit field MMIO_RWMODE) */
      MMIO_READ       = $00000000;      //* open file for reading only */
      MMIO_WRITE      = $00000001;      //* open file for writing only */
      MMIO_READWRITE  = $00000002;      //* open file for reading and writing */
      
//* various MMIO flags */
      MMIO_FHOPEN             = $0010;  //* mmioClose: keep file handle open */
      MMIO_EMPTYBUF           = $0010;  //* mmioFlush: empty the I/O buffer */
      MMIO_TOUPPER            = $0010;  //* mmioStringToFOURCC: to u-case */
      MMIO_INSTALLPROC    = $00010000;  //* mmioInstallIOProc: install MMIOProc */
      MMIO_GLOBALPROC     = $10000000;  //* mmioInstallIOProc: install globally */
      MMIO_REMOVEPROC     = $00020000;  //* mmioInstallIOProc: remove MMIOProc */
      MMIO_UNICODEPROC    = $01000000;  //* mmioInstallIOProc: Unicode MMIOProc */
      MMIO_FINDPROC       = $00040000;  //* mmioInstallIOProc: find an MMIOProc */
      MMIO_FINDCHUNK          = $0010;  //* mmioDescend: find a chunk by ID */
      MMIO_FINDRIFF           = $0020;  //* mmioDescend: find a LIST chunk */
      MMIO_FINDLIST           = $0040;  //* mmioDescend: find a RIFF chunk */
      MMIO_CREATERIFF         = $0020;  //* mmioCreateChunk: make a LIST chunk */
      MMIO_CREATELIST         = $0040;  //* mmioCreateChunk: make a RIFF chunk */

      MMIO_VALIDPROC      = $10070000;  //* valid for mmioInstallIOProc */ /* Internal */

//* constants for dwFlags field of MMIOINFO */
      MMIO_CREATE     = $00001000;      //* create new file (or truncate file) */
      MMIO_PARSE      = $00000100;      //* parse new file returning path */
      MMIO_DELETE     = $00000200;      //* create new file (or truncate file) */
      MMIO_EXIST      = $00004000;      //* checks for existence of file */
      MMIO_ALLOCBUF   = $00010000;      //* mmioOpen() should allocate a buffer */
      MMIO_GETTEMP    = $00020000;      //* mmioOpen() should retrieve temp name */

      MMIO_DIRTY      = $10000000;      //* I/O buffer is dirty */

      MMIO_OPEN_VALID = $0003FFFF;      //* valid flags for mmioOpen */ /* Internal */
      MMIO_FLUSH_VALID = MMIO_EMPTYBUF;  //* valid flags for mmioFlush */ /* Internal */
      MMIO_ADVANCE_VALID = MMIO_WRITE or MMIO_READ;     //* valid flags for mmioAdvance */ /* Internal */
      MMIO_FOURCC_VALID = MMIO_TOUPPER;  //* valid flags for mmioStringToFOURCC */ /* Internal */
      MMIO_DESCEND_VALID = MMIO_FINDCHUNK or MMIO_FINDRIFF or MMIO_FINDLIST; //* Internal */
      MMIO_CREATE_VALID = MMIO_CREATERIFF or MMIO_CREATELIST;   //* Internal */

//* share mode numbers (bit field MMIO_SHAREMODE) */
      MMIO_COMPAT     = $00000000;      //* compatibility mode */
      MMIO_EXCLUSIVE  = $00000010;      //* exclusive-access mode */
      MMIO_DENYWRITE  = $00000020;      //* deny writing to other processes */
      MMIO_DENYREAD   = $00000030;      //* deny reading to other processes */
      MMIO_DENYNONE   = $00000040;      //* deny nothing to other processes */

//* message numbers for MMIOPROC I/O procedure functions */
      MMIOM_READ      = MMIO_READ;       //* read */
      MMIOM_WRITE    = MMIO_WRITE;       //* write */
      MMIOM_SEEK              = 2;       //* seek to a new position in file */
      MMIOM_OPEN              = 3;       //* open file */
      MMIOM_CLOSE             = 4;       //* close file */
      MMIOM_WRITEFLUSH        = 5;       //* write and flush */

      MMIOM_RENAME            = 6;       //* rename specified file */

      MMIOM_USER         = $8000;        //* beginning of user-defined messages */

//* standard four character codes */
const
      FOURCC_RIFF     = FOURCC(byte(AnsiChar('R')) or
                               (byte(AnsiChar('I')) shl 8) or
                               (byte(AnsiChar('F')) shl 16) or
                               (byte(AnsiChar('F')) shl 24)
                              );


      FOURCC_LIST     = FOURCC(byte(AnsiChar('L')) or
                               (byte(AnsiChar('I')) shl 8) or
                               (byte(AnsiChar('S')) shl 16) or
                               (byte(AnsiChar('T')) shl 24)
                              );

//* four character codes used to identify standard built-in I/O procedures */
      FOURCC_DOS      = FOURCC(byte(AnsiChar('D')) or
                               (byte(AnsiChar('O')) shl 8) or
                               (byte(AnsiChar('S')) shl 16) or
                               (byte(AnsiChar(' ')) shl 24)
                              );

      FOURCC_MEM      = FOURCC(byte(AnsiChar('M')) or
                               (byte(AnsiChar('E')) shl 8) or
                               (byte(AnsiChar('M')) shl 16) or
                               (byte(AnsiChar(' ')) shl 24)
                              );

//* flags for mmioSeek() */
const
      SEEK_SET        = 0;               //* seek to an absolute position */
      SEEK_CUR        = 1;               //* seek relative to current position */
      SEEK_END        = 2;               //* seek relative to end of file */

//* other constants */
const
      MMIO_DEFAULTBUFFER      = 8192;    //* default buffer size */

{$ENDIF} // MMNOMMIO

{$PACKRECORDS DEFAULT} // #include "poppack.h"        /* Revert to default packing */

implementation

{ Was declared as
      MAKEFOURCC(ch0, ch1, ch2, ch3)                              \
      ((DWORD)(BYTE)(ch0) | ((DWORD)(BYTE)(ch1) << 8) |   \
      ((DWORD)(BYTE)(ch2) << 16) | ((DWORD)(BYTE)(ch3) << 24 ))

      mmioFOURCC(ch0, ch1, ch2, ch3)  MAKEFOURCC(ch0, ch1, ch2, ch3)
}
function MAKEFOURCC(ch0:AnsiChar; ch1:AnsiChar; ch2:AnsiChar; ch3:AnsiChar):FOURCC; inline;
begin
  MAKEFOURCC:=DWORD(ch0) or
              (DWORD(ch1) shl 8) or
              (DWORD(ch2) shl 16) or
              (DWORD(ch3) shl 24);
end;

function mmioFOURCC(ch0:AnsiChar; ch1:AnsiChar; ch2:AnsiChar; ch3:AnsiChar):FOURCC;
begin
  mmioFOURCC:=MAKEFOURCC(ch0,ch1,ch2,ch3);
end;

{ Was declared as
#define sndAlias( ch0, ch1 ) \
                ( SND_ALIAS_START + (DWORD)(BYTE)(ch0) | ( (DWORD)(BYTE)(ch1) << 8 ))
}
function sndAlias(ch0:AnsiChar; ch1:AnsiChar):DWORD; inline;
begin
  sndAlias:=SND_ALIAS_START+(DWORD(ch0) or (DWORD(ch1) shl 8));
end;

function MEVT_EVENTTYPE(x:DWORD):byte; inline;
begin
  MEVT_EVENTTYPE:=byte(((x shr 24) and $FF));
end;

function MEVT_EVENTPARM(x:DWORD):DWORD; inline;
begin
  MEVT_EVENTPARM:=DWORD(x and $00FFFFFF);
end;

{$IFNDEF MMNOMIDI}
function MIDIPROP_PROPERTY(mp:DWORD):DWORD; inline;
begin
  MIDIPROP_PROPERTY:=mp and (not (MIDIPROP_SET or MIDIPROP_GET));
end;

function SMF_TIMEDIV(format:DWORD; division:DWORD):DWORD; inline;
begin
  SMF_TIMEDIV:=(format shr 15) or division;
end;

function SMF_TIMEDIV_SMPTE(smpte:DWORD; division:DWORD):DWORD; inline;
begin
  SMF_TIMEDIV_SMPTE:=(smpte shl 8) or division;
end;

function SMF_TIMEDIV_ISSMPTE(dw:DWORD):DWORD; inline;
begin
  SMF_TIMEDIV_ISSMPTE:=dw shr 15;
end;

function SMF_TIMEDIV_GETSMPTE(dw:DWORD):byte; inline;
begin
  SMF_TIMEDIV_GETSMPTE:=byte((dw shr 8) and $ff);
end;

function SMF_TIMEDIV_GETTPF(dw:DWORD):DWORD; inline;
begin
  SMF_TIMEDIV_GETTPF:=dw and $ff;
end;

function SMF_TIMEDIV_GETTPQN(dw:DWORD):DWORD; inline;
begin
  SMF_TIMEDIV_GETTPQN:=dw and $7fff;
end;

{$ENDIF MMNOMIDI}

end.
