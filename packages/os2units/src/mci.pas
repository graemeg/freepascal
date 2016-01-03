{
    Copyright (c) 1991, 1992, 1993 International Business Machines Corporation
    Copyright (c) 2002 by Andry Svirgunov (cool2@ngs.ru)
    Copyright (c) 2002-2003 by Yuri Prokushev (prokushev@freemail.ru)

    This is Media Control Interface of MMPM/2

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public License (LGPL) as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version. This program is
    distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.

    See the GNU Library General Public License for more details. You should
    have received a copy of the GNU Library General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 **********************************************************************}

{
@abstract(Media Control Interface of MMPM/2)
@author(Andry Svirgunov (cool2@ngs.ru))
@author(Yuri Prokushev (prokushev@freemail.ru))
@created(29 Nov 2002)
@lastmod(05 Dec 2002)
This is Media Control Interface of MMPM/2.
Warning: This code is alfa. Future versions of this unit will propably
not be compatible.
}
Unit mci;

Interface

Uses
  mmbase,
  os2def,
  pmgpi;

Const
  MCI_TRUE                      = 1;
  MCI_FALSE                     = 0;

  // MCI command message identifiers
  MCI_OPEN                      = 1;
  MCI_CLOSE                     = 2;
  MCI_ESCAPE                    = 3;
  MCI_PLAY                      = 4;
  MCI_SEEK                      = 5;
  MCI_STOP                      = 6;
  MCI_PAUSE                     = 7;
  MCI_INFO                      = 8;
  MCI_GETDEVCAPS                = 9;
  MCI_STATUS                    = 10;
  MCI_SPIN                      = 11;
  MCI_SET                       = 12;
  MCI_STEP                      = 13;
  MCI_RECORD                    = 14;
  MCI_SYSINFO                   = 15;
  MCI_SAVE                      = 16;
  MCI_CUE                       = 17;
  MCI_UPDATE                    = 18;
  MCI_SET_CUEPOINT              = 19;
  MCI_SET_POSITION_ADVISE       = 20;
  MCI_SET_SYNC_OFFSET           = 21;
  MCI_LOAD                      = 22;
  MCI_ACQUIREDEVICE             = 23;
  MCI_RELEASEDEVICE             = 24;
  MCI_MASTERAUDIO               = 25;
  MCI_GETTOC                    = 26;
  MCI_DEVICESETTINGS            = 27;
  MCI_CONNECTOR                 = 28;
  MCI_RESUME                    = 29;
  MCI_CONNECTORINFO             = 31;
  MCI_DEFAULT_CONNECTION        = 32;
  MCI_CONNECTION                = 33;
  MCI_GROUP                     = 34;
  MCI_NETWORD_DEFAULT_CONNECTION= 35;

  // MCI command message identifiers reserved for Digital Video and Video Overlay
  MCI_CAPTURE                   = 40;
  MCI_FREEZE                    = 41;
  MCI_GETIMAGEBUFFER            = 42;
  MCI_GETIMAGEPALETTE           = 43;
  MCI_PUT                       = 44;
  MCI_REALIZE                   = 45;
  MCI_REWIND                    = 46;
  MCI_RESTORE                   = 47;
  MCI_SETIMAGEBUFFER            = 48;
  MCI_SETIMAGEPALETTE           = 49;
  MCI_UNFREEZE                  = 50;
  MCI_WHERE                     = 51;
  MCI_WINDOW                    = 52;

  MCI_DELETE                    = 53;
  MCI_CUT                       = 54;
  MCI_PASTE                     = 55;
  MCI_COPY                      = 56;
  MCI_REDO                      = 57;
  MCI_UNDO                      = 58;
  MCI_MIXNOTIFY                 = 59;
  MCI_SETTUNER                  = 60;
  MCI_FILTER                    = 61;
  MCI_BUFFER                    = 62;
  MCI_MIXSETUP                  = 63;

  MCI_MAX_COMMAND               = 64;

Type
  RECT = LongInt;
  HDCT = LongInt;

Const
  // this and all subsequent message ID's are reserved for the user
  MCI_USER_MESSAGES             = 2000;

  // Special Device ID for "all"
  MCI_ALL_DEVICE_ID             = $FFFF;

  // MCI implementation limits
  MAX_DEVICE_NAME               = 20;
  MAX_ALIAS_NAME                = 20;
  MAX_PRODINFO                  = 40;
  MAX_EXTENSION_NAME            = 4;
  MAX_DEV_PARAMS                = 128;
  MAX_VERSION_NUMBER            = 6;
  MAX_PDD_NAME                  = 9;
  MAX_DLL_NAME                  = 260;
  MAX_CLASSES                   = 10;
  MAX_CONNECTORS                = 10;
  MAX_EXTENSIONS                = 25;
  MAX_TYPEBUFFER                = 256;

  // MCI Device Type Constants
  MCI_DEVTYPE_VIDEOTAPE         = 1;
  MCI_DEVTYPE_VIDEODISC         = 2;
  MCI_DEVTYPE_CD_AUDIO          = 3;
  MCI_DEVTYPE_DAT               = 4;
  MCI_DEVTYPE_AUDIO_TAPE        = 5;
  MCI_DEVTYPE_OTHER             = 6;
  MCI_DEVTYPE_WAVEFORM_AUDIO    = 7;
  MCI_DEVTYPE_SEQUENCER         = 8;
  MCI_DEVTYPE_AUDIO_AMPMIX      = 9;
  MCI_DEVTYPE_OVERLAY           = 10;
  MCI_DEVTYPE_ANIMATION         = 11;
  MCI_DEVTYPE_DIGITAL_VIDEO     = 12;
  MCI_DEVTYPE_SPEAKER           = 13;
  MCI_DEVTYPE_HEADPHONE         = 14;
  MCI_DEVTYPE_MICROPHONE        = 15;
  MCI_DEVTYPE_MONITOR           = 16;
  MCI_DEVTYPE_CDXA              = 17;
  MCI_DEVTYPE_FILTER            = 18;
  MCI_DEVTYPE_TTS               = 19;

  // MCI Device Type Names
  MCI_DEVTYPE_VIDEOTAPE_NAME    = 'Videotape';
  MCI_DEVTYPE_VIDEODISC_NAME    = 'Videodisc';
  MCI_DEVTYPE_CD_AUDIO_NAME     = 'CDaudio';
  MCI_DEVTYPE_DAT_NAME          = 'DAT';
  MCI_DEVTYPE_AUDIO_TAPE_NAME   = 'Audiotape';
  MCI_DEVTYPE_OTHER_NAME        = 'Other';
  MCI_DEVTYPE_WAVEFORM_AUDIO_NAME = 'Waveaudio';
  MCI_DEVTYPE_SEQUENCER_NAME    = 'Sequencer';
  MCI_DEVTYPE_AUDIO_AMPMIX_NAME = 'Ampmix';
  MCI_DEVTYPE_OVERLAY_NAME      = 'Overlay';
  MCI_DEVTYPE_ANIMATION_NAME    = 'Animation';
  MCI_DEVTYPE_DIGITAL_VIDEO_NAME= 'Digitalvideo';
  MCI_DEVTYPE_SPEAKER_NAME      = 'Speaker';
  MCI_DEVTYPE_HEADPHONE_NAME    = 'Headphone';
  MCI_DEVTYPE_MICROPHONE_NAME   = 'Microphone';
  MCI_DEVTYPE_MONITOR_NAME      = 'Monitor';
  MCI_DEVTYPE_CDXA_NAME         = 'CDXA';
  MCI_DEVTYPE_FILTER_NAME       = 'Filter';
  MCI_DEVTYPE_TTS_NAME          = 'Texttospeech';
  MCI_MAX_SYSTEM_DEVICE_NAMES   = 19;


  // Getdevcaps, set, and status item base values
  MCI_AMP_ITEM_BASE             = $1000;
  MCI_CD_ITEM_BASE              = $2000;
  MCI_CDXA_ITEM_BASE            = $3000;
  MCI_VD_ITEM_BASE              = $4000;
  MCI_SEQ_ITEM_BASE             = $5000;
  MCI_WAVE_ITEM_BASE            = $6000;
  MCI_VID_ITEM_BASE             = $7000;
  MCI_DGV_ITEM_BASE             = $8000;
  MCI_OVLY_ITEM_BASE            = $9000;

  // Flags for mciDriverNotify
  MCI_NOTIFY_SUCCESSFUL         = $0000;
  MCI_NOTIFY_SUPERSEDED         = $0001;
  MCI_NOTIFY_ABORTED            = $0002;
  MCI_NOTIFY_ERROR              = $0003;
  // mciDriverNotify  Message Types
  MM_MCINOTIFY                  = $0500;
  MM_MCIPASSDEVICE              = $0501;
  MM_MCIPOSITIONCHANGE          = $0502;
  MM_MCICUEPOINT                = $0503;
  MM_MCIPLAYLISTMESSAGE         = $0504;
  MM_MCIEVENT                   = $0505;
  MM_MCISYNCH                   = $0506;

  MCI_LOSING_USE                = $00000001;
  MCI_GAINING_USE               = $00000002;

  // Common message flags 0x000000XX are reserved for common flags                      */
  MCI_NOTIFY                    = $00000001;
  MCI_WAIT                      = $00000002;
  MCI_FROM                      = $00000004;
  MCI_TO                        = $00000008;
  MCI_MILLISECONDS              = $00000010;
  MCI_TRACK                     = $00000020;
  MCI_OVER                      = $00000040;
  MCI_TEST                      = $00000080;
  MCI_TO_BUFFER                 = $00000100;
  MCI_FROM_BUFFER               = $00000200;
  MCI_CONVERT_FORMAT            = $00000400;

  // Time formats
  MCI_FORMAT_MILLISECONDS       = $00000001;
  MCI_FORMAT_mmTime             = $00000002;
  MCI_FORMAT_MSF                = $00000005;
  MCI_FORMAT_TMSF               = $00000006;
  MCI_FORMAT_CHAPTERS           = $00000007;
  MCI_FORMAT_FRAMES             = $00000008;
  MCI_FORMAT_HMS                = $00000009;
  MCI_FORMAT_TRACKS             = $0000000A;
  MCI_FORMAT_BYTES              = $0000000B;
  MCI_FORMAT_SAMPLES            = $0000000C;
  MCI_FORMAT_HMSF               = $0000000D;
  MCI_FORMAT_SET_SMPTE_24       = $0000000E;
  MCI_FORMAT_SET_SMPTE_25       = $0000000F;
  MCI_FORMAT_SET_SMPTE_30       = $00000010;
  MCI_FORMAT_SET_SMPTE_30DROP   = $00000011;
  MCI_FORMAT_SET_SONGPTR        = $00000012;
  MCI_FORMAT_uSec               = $00000013;

  // Speed formats
  MCI_FORMAT_PERCENTAGE         = $00000003;
  MCI_FORMAT_FPS                = $00000004;

//  Time format conversions functions

function mSecFromMM(Value: mmTime): LongInt;

function mSecToMM(Value: Cardinal): mmTime;

function RedBookToMM(Value: Cardinal): mmTime;

function fps24ToMM(Value: Cardinal): mmTime;

function fps25ToMM(Value: Cardinal): mmTime;

function fps30ToMM(Value: Cardinal): mmTime;

function HMSToMM(Value: Cardinal): mmTime;

// The +20 is used for rounding purposes.  It is derived by:
//   1/2 * ((300 MMTIME/SEC / (75 FRAMES/SEC)) = 20 MMTIME/HALF-FRAME
function RedBookFromMM(Value: mmTime): Cardinal;

function FPS24FromMM(Value: mmTime): Cardinal;

function FPS25FromMM(value: mmTime): Cardinal;

function FPS30FromMM(value: mmTime): Cardinal;

function HMSFromMM(value: mmTime): Cardinal;

function tmsf_track(time: mmTime): Byte;

function tmsf_minute(time: mmTime): Byte;

function tmsf_second(time: mmTime): Byte;

function tmsf_frame(time: mmTime): Byte;

function msf_minute(time: mmTime): Byte;

function msf_second(time: mmTime): Byte;

function msf_frame(time: mmTime): Byte;

//  BYTE ACCESS WITH A DWORD MACROS
function uLong_lwlb(var ul): Byte;   // Low word low byte

function uLong_lwhb(var ul): Byte;   // Low word high byte

function uLong_hwlb(var ul): Byte;   // High word low byte

function uLong_hwhb(var ul): Byte;   // High word high byte

function uLong_lowd(var ul): Word;   // Low word

function uLong_hiwd(var ul): Word;   // High word

// parameters for default command messages with empty parameter lists
type
  mci_Generic_Parms = record
    hwndCallback : hwnd;
  end;
  pmci_Generic_Parms = ^mci_Generic_Parms;

// flags for the MCI_ACQUIREDEVICE message
//     0x00000X00 are reserved for MCI_ACQUIREDEVICE flags
Const
  MCI_EXCLUSIVE                 = $00000100;
  MCI_EXCLUSIVE_INSTANCE        = $00000200;
  MCI_ACQUIRE_QUEUE             = $00000400;

  // flags for the MCI_CAPTURE message                                 */
  //     0x0000XX00 are reserved for MCI_CAPTURE flags                 */
  //     MCI_CONVERT is used by MCI_GETIMAGEBUFFER, MCI_SETIMAGEBUFFER */
  //                            MCI_CAPTURE, and MCI_RESTORE           */
  MCI_CAPTURE_RECT              = $00000100;
  MCI_CONVERT                   = $00001000;
type
  // parameter structure for the MCI_CAPTURE message                   */
  mci_Capture_Parms = record
    hwndCallback : hwnd;
    Rect         : RectL;
  end;
  pmci_Capture_Parms = ^mci_Capture_Parms;

  // flags for MCI_CONNECTOR and MCI_CONNECTION messages               */
Const
  MCI_ENABLE_CONNECTOR          = $00000100;
  MCI_DISABLE_CONNECTOR         = $00000200;
  MCI_QUERY_CONNECTOR_STATUS    = $00000400;
  MCI_CONNECTOR_TYPE            = $00000800;
  MCI_CONNECTOR_INDEX           = $00001000;
  MCI_TO_CONNECTOR_INDEX        = $00002000;
  MCI_TO_CONNECTOR_TYPE         = $00004000;
  MCI_QUERY_CONNECTION          = $00008000;
  MCI_MAKE_CONNECTION           = $00010000;
  MCI_BREAK_CONNECTION          = $00020000;
  MCI_ENUMERATE_CONNECTORS      = $00040000;
  MCI_QUERY_CONNECTOR_TYPE      = $00080000;
  MCI_QUERY_VALID_CONNECTION    = $00100000;
  MCI_CONNECTOR_ALIAS           = $00200000;
  MCI_EXTERNAL_CONNECTION       = $00400000;

  // Connector types
  MCI_MIDI_STREAM_CONNECTOR     = $00000001;
  MCI_CD_STREAM_CONNECTOR       = $00000002;
  MCI_WAVE_STREAM_CONNECTOR     = $00000003;
  MCI_AMP_STREAM_CONNECTOR      = $00000004;
  MCI_XA_STREAM_CONNECTOR       = $00000005;
  MCI_HEADPHONES_CONNECTOR      = $00000006;
  MCI_SPEAKERS_CONNECTOR        = $00000007;
  MCI_MICROPHONE_CONNECTOR      = $00000008;
  MCI_LINE_IN_CONNECTOR         = $00000009;
  MCI_LINE_OUT_CONNECTOR        = $0000000a;
  MCI_VIDEO_IN_CONNECTOR        = $0000000b;
  MCI_VIDEO_OUT_CONNECTOR       = $0000000c;
  MCI_PHONE_SET_CONNECTOR       = $0000000d;
  MCI_PHONE_LINE_CONNECTOR      = $0000000e;
  MCI_AUDIO_IN_CONNECTOR        = $0000000f;
  MCI_AUDIO_OUT_CONNECTOR       = $00000010;
  MCI_UNIVERSAL_CONNECTOR       = $00000011;
  MCI_INTERNAL_AUDIO_CONNECTOR  = $00000012;
  MCI_MIDI_IN_CONNECTOR         = $00000013;
  MCI_MIDI_OUT_CONNECTOR        = $00000014;
  MCI_NULL_CONNECTOR            = $00000015;

Type
  // parameter structure for the MCI_CONNECTION message
  mci_Connection_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    ulConnectorType  : LongInt;                  // Connector type
    ulConnectorIndex : LongInt;                  // Connector index
    pszAlias         : pChar;                  // Connection device alias
    usToDeviceID     : Integer;                 // Connected to device ID
    usReserved0      : Integer;                 // Reserved field
    ulReserved1      : LongInt;                  // Reserved field
    ulReserved2      : LongInt;                  // Reserved field
  end;
  pmci_Connection_Parms = ^mci_Connection_Parms;

      //********************************************************************/
      // parameter structure for the MCI_CONNECTOR message                 */
      //********************************************************************/
  mci_Connector_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    ulReturn         : LongInt;                  // Return information
    ulConnectorType  : LongInt;                  // If specified, ulConnectorIndex is relative
                                               // to the specified connector type
    ulConnectorIndex : LongInt;                  // Connector number
  end;
  pmci_Connector_Parms = ^mci_Connector_Parms;


      //********************************************************************/
      // Flags for use with ulParam1                                       */
      //********************************************************************/

CONST
      MCI_MIXSETUP_INIT            =$00010000;
      MCI_MIXSETUP_DEINIT          =$00020000;
      MCI_MIXSETUP_QUERYMODE       =$00040000;


      //********************************************************************/
      // parameter structure for the MCI_BUFFER    message                 */
      //********************************************************************/
Type
        mci_mix_buffer = record
         ulStructLength:LongInt;   // Length of the structure          */
         pBuffer:Pointer;          // Pointer to a buffer              */
         ulBufferLength:LongInt;   // Length of the buffer             */
         ulFlags:LongInt;          // Flags                            */
         ulUserParm:LongInt;       // Caller parameter                 */
         ulTime:LongInt;           // OUT--Current time in MS          */
         ulReserved1:LongInt;      // Unused.                          */
         ulReserved2:LongInt;      // Unused.                          */
         end;

      PMCI_MIX_BUFFER=^mci_mix_buffer;


      //********************************************************************/
      // valid options for the ulFlags field of MCI_MIX_BUFFER             */
      //********************************************************************/

CONST
      MIX_BUFFER_EOS                =$00000001;

type
  MixerProc=Function(ulHandle:LongInt;pBuffer:PMCI_MIX_BUFFER;ulFlags:LongInt):Longint; cdecl;
  MixerEvent=Function(ulStatus:LongInt;pBuffer:PMCI_MIX_BUFFER;ulFlags:LongInt):LongInt; cdecl;

      //********************************************************************/
      // valid returns for the ulFlags param of the MIXEREVENT entry       */
      //********************************************************************/

CONST
      MIX_STREAM_ERROR             =$00000080;
      MIX_READ_COMPLETE            =$00000001;
      MIX_WRITE_COMPLETE           =$00000002;



      //********************************************************************/
      // parameter structure for the MCI_MIXSETUP_PARMS message            */
      //********************************************************************/

TYPE MCI_MIXSETUP_PARMS = record
         hwndCallback:HWND;     // PM window handle for MCI notify message      */
         ulBitsPerSample:LongInt;  // IN Number of Bits per Sample                 */
         ulFormatTag:LongInt;      // IN Format Tag                                */
         ulSamplesPerSec:LongInt;  // IN Sampling Rate                             */
         ulChannels:LongInt;       // IN Number of channels                        */
         ulFormatMode:LongInt;     // IN Either MCI_RECORD or MCI_PLAY             */
         ulDeviceType:LongInt;     // IN MCI_DEVTYPE (i.e. DEVTYPE_WAVEFORM etc.)  */
         ulMixHandle:LongInt;      // OUT--mixer returns handle for write/read     */
         pmixWrite:MixerProc;        // OUT-Mixer Write Routine entry point          */
         pmixRead:MixerProc;         // OUT-Mixer Read Routine entry point           */
         pmixEvent:MixerEvent;        // IN--Mixer Read Routine entry point           */
         pExtendedInfo:Pointer;    // Ptr to extended wave information             */
         ulBufferSize:longInt;     // OUT--suggested buffer size for current mode  */
         ulNumBuffers:LongInt;     // OUT--suggested # of buffers for current mode */
         end;

pmci_mixsetup_parms=^mci_mixsetup_parms;

      //********************************************************************/
      // Flags for use with ulParam1                                       */
      //********************************************************************/

CONST

      MCI_BUFFER_QUERY         =$00010000;
      MCI_BUFFER_SET           =$00020000;
      MCI_ALLOCATE_MEMORY      =$00040000;
      MCI_DEALLOCATE_MEMORY    =$00080000;


      //********************************************************************/
      // parameter structure for the MCI_BUFFER    message                 */
      //********************************************************************/
type mci_buffer_parms=record
         hwndCallback:hwnd;     // PM window handle for MCI notify message    */
         ulStructLength:longint;   // Length of the MCI Buffer command           */
         ulNumBuffers:longInt;     // Number of buffers MCI driver should use    */
         ulBufferSize:longint;     // Size of buffers MCI driver should use      */
         ulMinToStart:longint;     // Min number of buffers to create a stream.  */
         ulSrcStart:longint;       // # of EMPTY buffers required to start Source*/
         ulTgtStart:longint;       // # of FULL buffers required to start Target */

         pBufList:pointer;         // Pointer to a list of buffers               */

         end;
 pmci_buffer_parms=^mci_buffer_parms;


      //********************************************************************/
      // parameter structure for the MCI_CONNECTORINFO message             */
      //********************************************************************/
  mci_ConnectorInfo_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    ulReturn         : Longint;                  // Return information
    ulDeviceTypeID   : Longint;                  // MCI device type
    ulConnectorType  : Longint;                  // If specified, ulConnectorIndex is relative
                                               // to the specified connector type
    ulConnectorIndex : Longint;                  // Connector number
    ulToConnectorType: Longint;                  // Connector type to test if
                                               // MCI_QUERY_VALID_CONNECTION is specified
  end;
  pmci_ConnectorInfo_Parms = ^mci_ConnectorInfo_Parms;

      //********************************************************************/
      // flags for the MCI_CUE message                                     */
      //     0x00000x00 are reserved for MCI_CUE flags                     */
      //********************************************************************/
CONST
  MCI_CUE_INPUT                 = $00000100;
  MCI_CUE_OUTPUT                = $00000200;
  MCI_SHOW                      = $00000400;
  MCI_NOSHOW                    = $00000800;

      //************************************************/
      // Additional flags for Wave Audio for MCI_CUE   */
      //************************************************/
  MCI_WAVE_INPUT                = $00001000;
  MCI_WAVE_OUTPUT               = $00002000;

      //****************************************************************************/
      // the MCI_CONNLIST structure used in the NETWORK_DEFAULT_CONNECTION message */
      //****************************************************************************/
type mci_connlist=record
         ulFromDevice: LongInt;  // Ordinal position of device within pDevices array */
         ulSrcConnType: LongInt; // Type of source connector                         */
         ulSrcConnNum: LongInt;  // Source Connector Number                          */
         ulToDevice: LongInt;    // Ordinal position of device within pDevices array */
         ulTgtConnType: LongInt; // Type of target connector                         */
         ulTgtConnNum: Longint;  // Target Connector Number                          */
         end;
      pmci_connlist=^mci_connlist;


      //****************************************************************************/
      // parameter structure for the MCI_NETWORK_DEFAULT_CONNECTION message        */
      //****************************************************************************/
type mci_network_default_connection_parms=record
     hwndCallback:HWND;                   // PM window handle for MCI notify message    */
     szInstallName:ARRAY [0..MAX_DEVICE_NAME] of Char; // Name of section containing default conn.   */
     ulNumDevices:LongInt;                   // Number of additional devices in connection */
     ulNumPlayConnections:LongInt;           // Number of Play connections                 */
     ulNumRecordConnections:LongInt;         // Number of Record connections               */
     pDevices:ARRAY [0..MAX_DEVICE_NAME] of ^Char;    // Pointer to array of device names to open   */
     pPlayConnections:PMCI_CONNLIST;               // Pointer to array of connections to "make"  */
     pRecordConnections:PMCI_CONNLIST;             // Pointer to array of connections to "make"  */
     end;
type pmci_network_default_connection_parms=^mci_network_default_connection_parms;


      //******************************************************/
      // Additional flags for MCI_NETWORK_DEFAULT_CONNECTION */
      //******************************************************/
CONST
      MCI_RECORD_DEFAULTS             =$00000004;
      MCI_PLAY_DEFAULTS               =$00000008;


      //********************************************************************/
      // parameter structure for the MCI_DEFAULT_CONNECTION message        */
      //********************************************************************/
type  mci_Default_Connection_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    pszDevice        : pChar;                  // Device name
    ulConnectorType  : LongInt;                  // If specified, ulConnectorIndex is relative
                                               // to the specified connector type
    ulConnectorIndex : LongInt;                  // Connector number
    pszToDevice      : pChar;                  // Return device name to which the connection exists
    ulToConnectorType: LongInt;                  // Connector type
    ulToConnectorIndex:LongInt;                  // Connector number
  end;
  pmci_Default_Connection_Parms = ^mci_Default_Connection_Parms;

      //********************************************************************/
      // parameter structure for the MCI_DEVICESETTINGS message            */
      //********************************************************************/
  mci_DeviceSettings_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    hwndNotebook     : hwnd;                   // Hwhd to notebook window
    usDeviceType     : Integer;                 // Device type
    pszDeviceName    : pChar;                  // Device name
  end;
  pmci_DeviceSettings_Parms = ^mci_DeviceSettings_Parms;

      //*********************************************************************/
      // parameter structure for the MCI_CUT / COPY / DELETE / UNDO / etc...*/
      //*********************************************************************/
  mci_Edit_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    ulStructLen      : Longint;                  // Length of the Structure
    ulFrom           : Longint;                  // Beginning Point of Range
    ulTo             : Longint;                  // ending point of range
    pBuff            : Pointer;                // user buffer
    ulBufLen         : Longint;                  // length of user buffer
    pHeader          : Pointer;                // header which describes the buffer
  end;
  pmci_Edit_Parms = ^mci_Edit_Parms;

      //********************************************************************/
      // flags and parameter structure for the MCI_ESCAPE message          */
      //     0x00000X00 are reserved for MCI_ESCAPE flags                  */
      //********************************************************************/
const
  mci_Escape_String             = $00000100;

type
  mci_Escape_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    pszCommand       : pChar;                  // Command to send to the device
  end;
  pmci_Escape_Parms = ^mci_Escape_Parms;

      //********************************************************************/
      // flags for the MCI_FREEZE message                                  */
      //     0x0000XX00 are reserved for MCI_FREEZE flags                  */
      //********************************************************************/
CONST
  MCI_OVLY_FREEZE_RECT          = $00000100;
  MCI_OVLY_FREEZE_RECT_OUTSIDE  = $00000200;

      //********************************************************************/
      // flags for the MCI_GETDEVCAPS message                              */
      //     0x00XXXX00 are reserved for MCI_GETDEVCAPS flags              */
      //********************************************************************/
  MCI_GETDEVCAPS_MESSAGE        = $00000100;
  MCI_GETDEVCAPS_ITEM           = $00000200;
  MCI_GETDEVCAPS_EXTENDED       = $00000400;
      //************************************************/
      // General MCI_GETDEVCAPS item values            */
      //************************************************/
  MCI_GETDEVCAPS_CAN_EJECT                                                = $00000001;
  MCI_GETDEVCAPS_CAN_LOCKEJECT  = $00000002;
  MCI_GETDEVCAPS_CAN_PLAY       = $00000003;
  MCI_GETDEVCAPS_CAN_PROCESS_INTERNAL = $00000004;
  MCI_GETDEVCAPS_CAN_RECORD     = $00000005;
  MCI_GETDEVCAPS_CAN_RECORD_INSERT    = $00000006;
  MCI_GETDEVCAPS_CAN_SAVE       = $00000007;
  MCI_GETDEVCAPS_CAN_SETVOLUME  = $00000008;
  MCI_GETDEVCAPS_CAN_STREAM     = $00000009;
  MCI_GETDEVCAPS_DEVICE_TYPE    = $0000000A;
  MCI_GETDEVCAPS_HAS_AUDIO      = $0000000B;
  MCI_GETDEVCAPS_HAS_VIDEO      = $0000000C;
  MCI_GETDEVCAPS_PREROLL_TIME   = $0000000D;
  MCI_GETDEVCAPS_PREROLL_TYPE   = $0000000E;
  MCI_GETDEVCAPS_USES_FILES     = $0000000F;
  MCI_GETDEVCAPS_HAS_IMAGE      = $00000010;
  MCI_GETDEVCAPS_WAVE_FORMAT    = $00000011;
  MCI_GETDEVCAPS_CAN_CLOSE_DOOR = $00000012;

      //**********************************************/
      // return ID's for videodisc MCI_GETDEVCAPS    */
      //**********************************************/
  MCI_VD_MEDIA_CLV=$00000001;
  MCI_VD_MEDIA_CAV=$00000002;
  MCI_VD_MEDIA_OTHER=$00000003;

      //************************************************/
      // MCI_GETDEVCAPS item values for video devices  */
      //************************************************/
  MCI_VID_GETDEVCAPS_CAN_STRETCH         = MCI_VID_ITEM_BASE;
  MCI_VID_GETDEVCAPS_CAN_DISTORT         = MCI_VID_ITEM_BASE+1;
  MCI_VID_GETDEVCAPS_VIDEO_X_EXTENT      = MCI_VID_ITEM_BASE+2;
  MCI_VID_GETDEVCAPS_VIDEO_Y_EXTENT      = MCI_VID_ITEM_BASE+3;
  MCI_VID_GETDEVCAPS_IMAGE_X_EXTENT      = MCI_VID_ITEM_BASE+4;
  MCI_VID_GETDEVCAPS_IMAGE_Y_EXTENT      = MCI_VID_ITEM_BASE+5;
  MCI_VID_GETDEVCAPS_MAX_WINDOWS         = MCI_VID_ITEM_BASE+6;
  MCI_VID_GETDEVCAPS_CAN_FREEZE          = MCI_VID_ITEM_BASE+7;
  MCI_VID_GETDEVCAPS_OVERLAY_GRAPHICS    = MCI_VID_ITEM_BASE+8;
  MCI_VID_GETDEVCAPS_CAN_REVERSE         = MCI_VID_ITEM_BASE+9;
  MCI_VID_GETDEVCAPS_FAST_RATE           = MCI_VID_ITEM_BASE+10;
  MCI_VID_GETDEVCAPS_SLOW_RATE           = MCI_VID_ITEM_BASE+11;
  MCI_VID_GETDEVCAPS_NORMAL_RATE         = MCI_VID_ITEM_BASE+12;
  MCI_VID_GETDEVCAPS_MINIMUM_RATE        = MCI_VID_ITEM_BASE+13;
  MCI_VID_GETDEVCAPS_MAXIMUM_RATE        = MCI_VID_ITEM_BASE+14;


      //************************************************/
      // MCI_GETDEVCAPS flag values for Videodisc      */
      //************************************************/
  MCI_VD_GETDEVCAPS_CLV                  = $00000400;
  MCI_VD_GETDEVCAPS_CAV                  = $00000800;

      //************************************************/
      // MCI_GETDEVCAPS item values for Videodisc      */
      //************************************************/

  MCI_VD_GETDEVCAPS_CAN_REVERSE          = MCI_VID_GETDEVCAPS_CAN_REVERSE;
  MCI_VD_GETDEVCAPS_FAST_RATE            = MCI_VID_GETDEVCAPS_FAST_RATE;
  MCI_VD_GETDEVCAPS_SLOW_RATE            = MCI_VID_GETDEVCAPS_SLOW_RATE;
  MCI_VD_GETDEVCAPS_NORMAL_RATE          = MCI_VID_GETDEVCAPS_NORMAL_RATE;
  MCI_VD_GETDEVCAPS_MINIMUM_RATE         = MCI_VID_GETDEVCAPS_MINIMUM_RATE;
  MCI_VD_GETDEVCAPS_MAXIMUM_RATE         = MCI_VID_GETDEVCAPS_MAXIMUM_RATE;
      //***********************************************/
      // MCI_GETDEVCAPS item values for Digital Video */
      //***********************************************/
  MCI_DGV_GETDEVCAPS_CAN_REVERSE         = MCI_VID_GETDEVCAPS_CAN_REVERSE;
  MCI_DGV_GETDEVCAPS_CAN_STRETCH         = MCI_VID_GETDEVCAPS_CAN_STRETCH;
  MCI_DGV_GETDEVCAPS_CAN_DISTORT         = MCI_VID_GETDEVCAPS_CAN_DISTORT;
  MCI_DGV_GETDEVCAPS_FAST_RATE           = MCI_VID_GETDEVCAPS_FAST_RATE;
  MCI_DGV_GETDEVCAPS_SLOW_RATE           = MCI_VID_GETDEVCAPS_SLOW_RATE;
  MCI_DGV_GETDEVCAPS_NORMAL_RATE         = MCI_VID_GETDEVCAPS_NORMAL_RATE;
  MCI_DGV_GETDEVCAPS_MINIMUM_RATE        = MCI_VID_GETDEVCAPS_MINIMUM_RATE;
  MCI_DGV_GETDEVCAPS_MAXIMUM_RATE        = MCI_VID_GETDEVCAPS_MAXIMUM_RATE;
  MCI_DGV_GETDEVCAPS_VIDEO_X_EXTENT      = MCI_VID_GETDEVCAPS_VIDEO_X_EXTENT;
  MCI_DGV_GETDEVCAPS_VIDEO_Y_EXTENT      = MCI_VID_GETDEVCAPS_VIDEO_Y_EXTENT;
  MCI_DGV_GETDEVCAPS_IMAGE_X_EXTENT      = MCI_VID_GETDEVCAPS_IMAGE_X_EXTENT;
  MCI_DGV_GETDEVCAPS_IMAGE_Y_EXTENT      = MCI_VID_GETDEVCAPS_IMAGE_Y_EXTENT;
  MCI_DGV_GETDEVCAPS_MAX_WINDOWS         = MCI_VID_GETDEVCAPS_MAX_WINDOWS;
  MCI_DGV_GETDEVCAPS_OVERLAY_GRAPHICS    = MCI_VID_GETDEVCAPS_OVERLAY_GRAPHICS;
  MCI_DGV_GETDEVCAPS_HAS_TUNER           = MCI_DGV_ITEM_BASE;
  MCI_DGV_GETDEVCAPS_HAS_TELETEX         = MCI_DGV_ITEM_BASE+1;
  MCI_DGV_GETDEVCAPS_HAS_AFC             = MCI_DGV_ITEM_BASE+2;

      //************************************************/
      // MCI_GETDEVCAPS item values for Video Overlay  */
      //************************************************/
  MCI_OVLY_GETDEVCAPS_CAN_STRETCH        = MCI_VID_GETDEVCAPS_CAN_STRETCH;
  MCI_OVLY_GETDEVCAPS_CAN_DISTORT        = MCI_VID_GETDEVCAPS_CAN_DISTORT;
  MCI_OVLY_GETDEVCAPS_VIDEO_X_EXTENT     = MCI_VID_GETDEVCAPS_VIDEO_X_EXTENT;
  MCI_OVLY_GETDEVCAPS_VIDEO_Y_EXTENT     = MCI_VID_GETDEVCAPS_VIDEO_Y_EXTENT;
  MCI_OVLY_GETDEVCAPS_IMAGE_X_EXTENT     = MCI_VID_GETDEVCAPS_IMAGE_X_EXTENT;
  MCI_OVLY_GETDEVCAPS_IMAGE_Y_EXTENT     = MCI_VID_GETDEVCAPS_IMAGE_Y_EXTENT;
  MCI_OVLY_GETDEVCAPS_MAX_WINDOWS        = MCI_VID_GETDEVCAPS_MAX_WINDOWS;
  MCI_OVLY_GETDEVCAPS_CAN_FREEZE         = MCI_VID_GETDEVCAPS_CAN_FREEZE;
  MCI_OVLY_GETDEVCAPS_OVERLAY_GRAPHICS   = MCI_VID_GETDEVCAPS_OVERLAY_GRAPHICS;

  MCI_PREROLL_NOTIFIED                   = $00000001;
  MCI_PREROLL_DETERMINISTIC              = $00000002;
  MCI_PREROLL_NONE                       = $00000003;

      //********************************************************************/
      // parameter structure for the MCI_GETDEVCAPS message                */
      //********************************************************************/
TYPE
  mci_GetDevCaps_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    ulReturn         : Longint;                  // Return field
    ulItem           : Longint;                  // Item field for GETDEVCAPS item to query
    usMessage        : integer;                 // Field to hold MCI message to query
    usReserved0      : integer;                 // Reserved field
  end;
  pmci_GetDevCaps_Parms = ^mci_GetDevCaps_Parms;

      //********************************************************************/
      // parameter structure for the MCI_WAVE_GETDEVCAPS                   */
      //********************************************************************/
  mci_Wave_GetDevCaps_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    ulReturn         : Longint;                  // Return field
    ulItem           : Longint;                  // Item field for GETDEVCAPS item to query
    usMessage        : Integer;                 // Field to hold MCI message to query
    usReserved0      : Integer;                 // Reserved field
    ulLength         : Longint;                  // Length of structure in ULONGS
    ulBitsPerSample  : Longint;                  // Number of Bits per Sample
    ulFormatTag      : Longint;                  // Format Tag
    ulSamplesPerSec  : Longint;                  // Sampling Rate
    ulChannels       : Longint;                  // Number of channels
    ulFormatMode     : Longint;                  // Either MCI_RECORD or MCI_PLAY
  end;
  pmci_Wave_GetDevCaps_Parms = ^mci_Wave_GetDevCaps_Parms;

      //********************************************************************/
      // potential values for the ulValue field of MCI_AMP_GETDEVCAPS  */
      //********************************************************************/
CONST
  MCI_SET_MONITOR                 = $00000001;
  MCI_SET_MONITOR_AUDIO           = $00000002;
  MCI_SET_MONITOR_VIDEO           = $00000003;
  MCI_SET_RECORD                  = $00000004;

  // MCI_SET flags for amp/mixer;
  MCI_AMP_SET_BALANCE             = $00100000;
  MCI_AMP_SET_PITCH               = $00200000;
  MCI_AMP_SET_TREBLE              = $00400000;
  MCI_AMP_SET_BASS                = $00800000;
  MCI_AMP_SET_GAIN                = $01000000;
  MCI_AMP_SET_ALC                 = $00000004;
  MCI_AMP_SET_AUDIO               = $00000008;
  MCI_AMP_SET_CROSSOVER           = $00000010;
  MCI_AMP_SET_LOUDNESS            = $00000020;
  MCI_AMP_SET_MUTE                = $00000040;
  MCI_AMP_SET_REVERB              = $00000080;
  MCI_AMP_SET_STEREOENHANCE       = $00000100;
  MCI_AMP_SET_CUSTOM1             = $00000200;
  MCI_AMP_SET_CUSTOM2             = $00000400;
  MCI_AMP_SET_CUSTOM3             = $00000800;
  MCI_AMP_SET_LRVOLUME            = $00001000;
  MCI_AMP_SET_MID                 = $00020000;
  MCI_AMP_SET_CHORUS              = $00004000;  // typo? 40000?
  MCI_AMP_SET_VOLUME              = $00080000;  // MCI_SET_VOLUME;
  MCI_AMP_SET_MONITOR             = MCI_SET_MONITOR_AUDIO;

  // Potential values for the ulValue field of MCI_AMP_GETDEVCAPS
  MCI_AMP_CAN_SET_MONITOR         = MCI_AMP_SET_MONITOR;
  MCI_AMP_CAN_SET_BALANCE         = MCI_AMP_SET_BALANCE;
  MCI_AMP_CAN_SET_ALC             = MCI_AMP_SET_ALC;
  MCI_AMP_CAN_SET_CROSSOVER       = MCI_AMP_SET_CROSSOVER;
  MCI_AMP_CAN_SET_LOUDNESS        = MCI_AMP_SET_LOUDNESS;
  MCI_AMP_CAN_SET_MUTE            = MCI_AMP_SET_MUTE;
  MCI_AMP_CAN_SET_REVERB          = MCI_AMP_SET_REVERB;
  MCI_AMP_CAN_SET_STEREOENHANCE   = MCI_AMP_SET_STEREOENHANCE;
  MCI_AMP_CAN_SET_CUSTOM1         = MCI_AMP_SET_CUSTOM1;
  MCI_AMP_CAN_SET_CUSTOM2         = MCI_AMP_SET_CUSTOM2;
  MCI_AMP_CAN_SET_CUSTOM3         = MCI_AMP_SET_CUSTOM3;
  MCI_AMP_CAN_SET_LRVOLUME        = MCI_AMP_SET_LRVOLUME;
  MCI_AMP_CAN_SET_BASS            = MCI_AMP_SET_BASS;
  MCI_AMP_CAN_SET_MID             = MCI_AMP_SET_MID;
  MCI_AMP_CAN_SET_TREBLE          = MCI_AMP_SET_TREBLE;
  MCI_AMP_CAN_SET_PITCH           = MCI_AMP_SET_PITCH;
  MCI_AMP_CAN_SET_GAIN            = MCI_AMP_SET_GAIN;
  MCI_AMP_CAN_SET_CHORUS          = MCI_AMP_SET_CHORUS;
  MCI_AMP_CAN_SET_VOLUME          = MCI_AMP_SET_VOLUME;

      //************************************************************************/
      // potential values for the ulExtended field of MCI_AMP_GETDEVCAPS_PARMS */
      //************************************************************************/
  MCI_MIXER_LINE                  = $00000001;


      //********************************************************************/
      // parameter structure for the MCI_AMP_GETDEVCAPS                    */
      //********************************************************************/
type
  mci_Amp_GetDevCaps_Parms = record
    hwndCallback  : Longint;                     // PM window handle for MCI notify message
    ulReturn      : Longint;                     // Return field
    ulItem        : Longint;                     // Item field for GETDEVCAPS item to query
    usMessage     : Integer;                    // Field to hold MCI message to query
    usReserved0   : Integer;                    // Reserved field
    ulLength      : LongInt;                     // Length of structure in uLongS
    ulValue       : LongInt;                     // Value to determine caps
    ulAttribute   : LongInt;                     // Flags to modified the extended parms
    ulExtended    : LongInt;                     // Extended flags field
  end;
  pmci_Amp_GetDevCaps_Parms = ^mci_Amp_GetDevCaps_Parms;


      //********************************************************************/
      // values for the ulFlags field of MCI_MIXEVENT_PARMS                */
      //********************************************************************/
CONST
  MCI_MIX_ATTRIBUTE             = $00000001;
  MCI_MIX_CONNECTOR             = $00000002;
      //********************************************************************/
      // values for the lParam field for mix_notify                        */
      //********************************************************************/

  MCI_MIXNOTIFY_ON              = $00000004;
  MCI_MIXNOTIFY_OFF             = $00000008;

      //********************************************************************/
      // value to indicate type of MM_MCIEVENT                             */
      //********************************************************************/

  MCI_MIXEVENT                  = $00000001;


      //********************************************************************/
      // parameter structure for the MCI_MIXEVENT_PARMS                    */
      //********************************************************************/
type
  mci_MixEvent_Parms = record
    ulLength         : Longint;                  // Length of struct
    hwndMixer        : hwnd;                   // window to inform of mixer changes
    ulFlags          : Longint;                  // Either MCI_MIX_ATTRIBUTE
                                               //      or MCI_MIX_CONNECTOR
    usDeviceID       : Integer;                 // device id to notify of the change
    ulDeviceType     : LongInt;                  // Device Type which generated the change
    ulDeviceOrdinal  : LongInt;                  // Ordinal of device Type
    ulAttribute      : LongInt;                  // Attribute that changed(volume, bass etc.)
    ulValue          : LongInt;                  // new value of the attribute which changed
    ulConnectorType  : LongInt;                  // Connector Type
    ulConnectorIndex : LongInt;                  // Connector Index
    ulConnStatus     : LongInt;                  // Is connector enabled/disabled.
  end;
  pmci_MixEvent_Parms = ^mci_MixEvent_Parms;


      //********************************************************************/
      // flag values for the MCI_GETIMAGEBUFFER and MCI_SETIMAGEBUFFER msg */
      //     0x0000XX00 are reserved for MCI_GETIMAGEBUFFER and            */
      //       MCI_SETIMAGEBUFFER Flags                                    */
      //     MCI_CONVERT is defined by MCI_CAPTURE message as 0x00001000L  */
      //********************************************************************/
CONST
  MCI_USE_HW_BUFFER             = $00000400;
  MCI_GET_HW_BUFFER_PTR         = $00000800;

      //**************************************************/
      // parm structure for MCI_GETIMAGEBUFFER and       */
      //                    MCI_SETIMAGEBUFFER messages  */
      //**************************************************/
type
  mci_Image_Parms = record
    hwndCallback       : hwnd;                 // PM window handle for MCI notify message
    ulPelFormat        : Longint;                // Format of the image data returned
    usBitCount         : Integer;               // Number of bitsimage data returned
    usReserved0        : Integer;               // Reserved field
    ulImageCompression : LongInt;                //
    rect               : RectL;                // Image area to get/set
    pPelBuffer         : Pointer;              // Pel data buffer
    ulPelBufferHeight  : LongInt;                // Number of rows in buffer
    ulPelBufferWidth   : LongInt;                // Pels per row in buffer
    ulBufLen           : LongInt;                // Pel data buffer length in bytes
  end;
  pmci_Image_Parms = ^mci_Image_Parms;

      //********************************************************************/
      // flags for the MCI_GETIMAGEPALETTE message                         */
      //     0x0000XX00 are reserved for MCI_GETIMAGEPALETTE flags         */
      //********************************************************************/
CONST
  MCI_FIND_BEST_REGISTERED      = $00000100;
  MCI_QUERY_REGISTERED_MAP      = $00000200;
  MCI_QUERY_REGISTERED_MAP_SIZE = $00000400;


      //**************************************************/
      // parameter structure for MCI_GETIMAGEPALETTE and */
      //    MCI_SETIMAGEPALETTE messages                 */
      //**************************************************/
type
  mci_Palette_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    usRegisteredMap  : Integer;                 // Id of the palette from the registed color map
    usReserved0      : Integer;                 // Reserved field
    ulPalEntries     : LongInt;                  // size of the palette returned
    pPalette         : Pointer;                // pointer to the palette
  end;
  pmci_Palette_Parms = ^mci_Palette_Parms;


      //********************************************************************/
      // Parameter structure for the MCI_GETTOC message                    */
      //********************************************************************/
  mci_Toc_Rec = record
    TrackNum         : BYTE;                   // Returned Track Number
    ulStartAddr      : LongInt;                  // Starting address of the track in MMTIME format
    ulEndAddr        : LongInt;                  // Ending address of the track in MMTIME format
    Control          : BYTE;                   // Track Control information
    usCountry        : Integer;                 // Country
    ulOwner          : LongInt;                  // Owner
    ulSerialNum      : LongInt;                  // Serial Number
  end;
  pTocRec = ^mci_Toc_Rec;

  mci_Toc_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    pBuf             : pTocRec;                // Pointer to array MCI_TOC_REC structures to
                                               // be filled in by the mci device
    ulBufSize        : LongInt;                  // Size of the array
  end;
  pmci_Toc_Parms = ^mci_Toc_Parms;

      //********************************************************************/
      // parameters for the MCI_GROUP message                              */
      //     0x00XXXX00 are reserved for MCI_GROUP flags                   */
      //********************************************************************/
const
  MCI_SYNCHRONIZE               = $00000100;
  MCI_NOPIECEMEAL               = $00000200;
  MCI_GROUP_MAKE                = $00000400;
  MCI_GROUP_DELETE              = $00000800;
  MCI_GROUP_ALIAS               = $00001000;
  MCI_GROUP_MASTER              = $00002000;
  MCI_GROUP_NONE                = $00004000;   // No group!

type
  mci_Group_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message.
    usGroupID                   : Integer;      // GroupID
    usReserved0                 : Integer;      // RESERVED
    ulStructLength              : LongInt;       // Length of Structure in ULONGS.
    usMasterID                  : Integer;      // ID of Master device in Synchrounous groups.
    usReserved1                 : Integer;      // RESERVED
    pszGroupAlias               : pChar;       // Pointer to Alias Name.
    ulNumDevices                : LongInt;       // Number of Devices in group.
    paulDeviceID                : PLongInt;      // Array of Device ids in the group.
  end;
  pmci_Group_Parms = ^mci_Group_Parms;


      //********************************************************************/
      // flags for the MCI_INFO message                                    */
      //     0x00000X00 are reserved for MCI_INFO flags                    */
      //********************************************************************/
CONST
  MCI_INFO_PRODUCT              = $00000100;
  MCI_INFO_FILE                 = $00000200;
      //***********************************************/
      // CD Audio                MCI_INFO             */
      //***********************************************/
  MCI_CD_INFO_ID                = $00010000;
  MCI_CD_INFO_UPC               = $00020000;

      //***********************************************/
      // Videodisc               MCI_INFO             */
      //***********************************************/
  MCI_VD_INFO_LABEL             = $00001000;

      //***********************************************/
      // Digital Video           MCI_INFO             */
      //***********************************************/
  MCI_DGV_INFO_VIDEO_FILE       = $00001000;
  MCI_DGV_INFO_IMAGE_FILE       = $00002000;
  MCI_DGV_INFO_REGION           = $00004000;
  MCI_DGV_INFO_REGION_TEXT      = $00008000;
  MCI_DGV_INFO_TEXT             = $00010000;

      //***********************************************/
      // Video Overlay           MCI_INFO             */
      //***********************************************/
  MCI_OVLY_INFO_TEXT            = $00010000;


type
  mci_Info_Parms = record
    hwndCallback      : hwnd;                  // PM window handle for MCI notify message
    pszReturn         : pChar;                 // Pointer to return buffer
    ulRetSize         : LongInt;                 // Return buffer size
  end;
  pmci_Info_Parms = ^mci_Info_Parms;


      //********************************************************************/
      // parameters and flags for the MCI_LOAD message                     */
      //     0x00000X00 are reserved for MCI_LOAD flags                    */
      //********************************************************************/
  mci_Load_Parms = record
    hwndCallback      : hwnd;                  // PM window handle for MCI notify message
    pszElementName    : pChar;                 // File name to loads
  end;
  pmci_Load_Parms = ^mci_Load_Parms;

      //********************************************************************/
      // parameters and flags for the MCI_MASTERAUDIO                      */
      //     0x000XXX00 are reserved for MCI_MASTERAUDIO  flags            */
      //********************************************************************/
CONST
  MCI_ON                        = $00000100;
  MCI_OFF                       = $00000200;
  MCI_HEADPHONES                = $00000400;
  MCI_SPEAKERS                  = $00000800;
  MCI_MASTERVOL                 = $00001000;
  MCI_SAVESETTING               = $00002000;
  MCI_QUERYSAVEDSETTING         = $00004000;
  MCI_QUERYCURRENTSETTING       = $00008000;

type
  mci_MasterAudio_Parms = record
    hwndDummyCallback : hwnd;                  // Notify not allowed for this message
    ulReturn          : LongInt;                 // Return field for query information
    ulMasterVolume    : LongInt;                 // Master volume field
  end;
  pmci_MasterAudio_Parms = ^mci_MasterAudio_Parms;

      //********************************************************************/
      // parameters and flags for the MCI_OPEN message                     */
      //     0x0000XX00 are reserved for MCI_OPEN flags                    */
      //********************************************************************/
CONST
  MCI_OPEN_ELEMENT              = $00000100;
  MCI_OPEN_ALIAS                = $00000200;
  MCI_OPEN_ELEMENT_ID           = $00000400;
  MCI_OPEN_PLAYLIST             = $00000800;
  MCI_OPEN_TYPE_ID              = $00001000;
  MCI_OPEN_SHAREABLE            = $00002000;
  MCI_OPEN_MMIO                 = $00004000;
  MCI_READONLY                  = $00008000;

      //**************************************************/
      // parameters and flags for the MCI_OPEN message   */
      // Digital Video, Video Overlay specific           */
      //**************************************************/
  MCI_VID_OPEN_PARENT           = $01000000;
  MCI_DGV_OPEN_PARENT           = MCI_VID_OPEN_PARENT;
  MCI_OVLY_OPEN_PARENT          = MCI_VID_OPEN_PARENT;

type
  mci_Open_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    usDeviceID       : Integer;                 // Device ID returned to user
    usReserved0      : Integer;                 // Reserved
    pszDeviceType    : pChar;                  // Device name from SYSTEM.INI
    pszElementName   : pChar;                  // Typically a file name or NULL
    pszAlias         : pChar;                  // Optional device alias
  end;
  pmci_Open_Parms = ^mci_Open_Parms;

      //*********************************************************/
      // parameters for the AMP MCI_OPEN message                */
      //*********************************************************/
  mci_Amp_Open_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    usDeviceID       : Integer;                 // Device ID returned to user
    usReserved0      : Integer;                 // Reserved field
    pszDeviceType    : pChar;                  // Device name from SYSTEM.INI
    pszElementName   : pChar;                  // Typically a file name or NULL
    pszAlias         : pChar;                  // Optional device alias
    pDevDataPtr      : Pointer;                // Pointer to device data
  end;
  pmci_Amp_Open_Parms = ^mci_Amp_Open_Parms;

      //*********************************************************/
      // parameters for MCI_OPEN message for video devices      */
      //*********************************************************/

  mci_Vid_Open_Parms = record
    hwndCallback     : hwnd;                   // PM window handle for MCI notify message
    usDeviceID       : Integer;                 // Device ID returned to user
    usReserved0      : Integer;                 // Reserved field
    pszDeviceType    : pChar;                  // Device name from SYSTEM.INI
    pszElementName   : pChar;                  // Typically a file name or NULL
    pszAlias         : pChar;                  // Optional device alias
    hwndParent       : hwnd;                   // Parent window handle
  end;
  pmci_Vid_Open_Parms = ^mci_Vid_Open_Parms;

  mci_dgv_Open_Parms   = mci_Vid_Open_Parms;
  pmci_dgv_Open_Parms  = ^mci_dgv_Open_Parms;

  mci_Ovly_Open_Parms  = mci_Vid_Open_Parms;
  pmci_Ovly_Open_Parms = ^mci_Ovly_Open_Parms;

      //**************************************************/
      // MCI_PLAY flag values for videodisc              */
      //**************************************************/
CONST
  MCI_VD_PLAY_REVERSE           = $00001000;
  MCI_VD_PLAY_FAST              = $00002000;
  MCI_VD_PLAY_SPEED             = $00004000;
  MCI_VD_PLAY_SCAN              = $00008000;
  MCI_VD_PLAY_SLOW              = $00010000;

      //**************************************************/
      // MCI_PLAY flag values for digital video          */
      //**************************************************/
  MCI_DGV_PLAY_REVERSE          = $00001000;
  MCI_DGV_PLAY_FAST             = $00002000;
  MCI_DGV_PLAY_SPEED            = $00004000;
  MCI_DGV_PLAY_SCAN             = $00008000;
  MCI_DGV_PLAY_SLOW             = $00010000;
  MCI_DGV_PLAY_REPEAT           = $00020000;

type
  mci_Play_Parms = record
    hwndCallback      : hwnd;
    ulFrom            : LongInt;
    ulTo              : LongInt;
  end;
  pmci_Play_Parms = ^mci_Play_Parms;

  mci_VD_Play_Parms = record
    hwndCallback      : hwnd;
    ulFrom            : LongInt;
    ulTo              : LongInt;
    ulFactor          : LongInt;
  end;
  pmci_VD_Play_Parms = ^mci_VD_Play_Parms;

  mci_DGV_play_Parms = record
    hwndCallback      : hwnd;
    ulFrom            : LongInt;
    ulTo              : LongInt;
    ulSpeed           : LongInt;
  end;
  pmci_DGV_play_Parms = ^mci_DGV_play_Parms;

      //********************************************************************/
      // parameters for the MCI_PUT  message                               */
      //     0x00000X00 are reserved for MCI_PUT  flags                    */
      //********************************************************************/
CONST
  MCI_VID_PUT_RECT              = $00000100;
  MCI_VID_PUT_DESTINATION       = $00000200;
  MCI_VID_PUT_SOURCE            = $00000400;
  MCI_VID_PUT_WINDOW_MOVE       = $00000800;
  MCI_VID_PUT_WINDOW_SIZE       = $00001000;

  MCI_DGV_PUT_RECT              = MCI_VID_PUT_RECT;
  MCI_DGV_PUT_DESTINATION       = MCI_VID_PUT_DESTINATION;
  MCI_DGV_PUT_SOURCE            = MCI_VID_PUT_SOURCE;
  MCI_DGV_PUT_WINDOW_MOVE       = MCI_VID_PUT_WINDOW_MOVE;
  MCI_DGV_PUT_WINDOW_SIZE       = MCI_VID_PUT_WINDOW_SIZE;
  MCI_DGV_MONITOR               = $00010000;
  MCI_DGV_RECORD                = $00020000;

  MCI_OVLY_PUT_RECT             = MCI_VID_PUT_RECT;
  MCI_OVLY_PUT_DESTINATION      = MCI_VID_PUT_DESTINATION;
  MCI_OVLY_PUT_SOURCE           = MCI_VID_PUT_SOURCE;
  MCI_OVLY_PUT_WINDOW_MOVE      = MCI_VID_PUT_WINDOW_MOVE;
  MCI_OVLY_PUT_WINDOW_SIZE      = MCI_VID_PUT_WINDOW_SIZE;

      //**************************************************/
      // Rectangle parameters for MCI_PUT, MCI_WHERE,    */
      // MCI_FREEZE, and MCI_UNFREEZE                    */
      //**************************************************/

type
  mci_Vid_Rect_Parms = record
    hwndCallback      : hwnd;                  // PM window handle for MCI notify message
    rc                : RectL;                 // rectangle array specifying the offset
                                               // and size of a rectangle
  end;

  mci_Dgv_Rect_Parms            = mci_Vid_Rect_Parms;
  pmci_Dgv_Rect_Parms           = ^mci_Dgv_Rect_Parms;

  mci_Ovly_Rect_Parms           = mci_Vid_Rect_Parms;
  pmci_Ovly_Rect_Parms          = ^mci_Ovly_Rect_Parms;

      //********************************************************************/
      // parameters for the MCI_RECORD message                             */
      //     0x00000X00 are reserved for MCI_RECORD flags                  */
      //********************************************************************/
CONST
  MCI_RECORD_INSERT             = $00000100;
  MCI_RECORD_OVERWRITE          = $00000200;

  MCI_DGV_RECORD_RECT           = $00000400;

type
  mci_Record_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulFrom                      : LongInt;       // Record from position
    ulTo                        : LongInt;       // Record to position
  end;
  pmci_Record_Parms = ^mci_Record_Parms;

      //********************************************************************/
      // parameters for the MCI_RELEASEDEVICE message                      */
      //     0xXXXXXX00 are reserved for MCI_RELEASEDEVICE message         */
      //********************************************************************/
CONST
  MCI_RETURN_RESOURCE           = $00000100;

      //********************************************************************/
      // parameters for the MCI_RESTORE message                            */
      //     0x0000XX00 are reserved for MCI_RESTORE flags                 */
      //     MCI_CONVERT is defined by MCI_CAPTURE message as 0x00001000L  */
      //********************************************************************/
  MCI_RESTORE_SRC_RECT          = $00000100;
  MCI_RESTORE_DEST_RECT         = $00000200;

type  mci_Restore_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    SrcRect                     : RectL;       // Rectangle array specifying the source
                                               // area to be restored
    RestRect                    : RectL;       // Rectangle array specifying the destination
                                               // area to be restored
  end;
  pmci_Restore_Parms = ^mci_Restore_Parms;

      //********************************************************************/
      // parameters for the MCI_SAVE  message                              */
      //     0x00000X00 are reserved for MCI_SAVE  flags                   */
      //********************************************************************/
const
  MCI_SAVE_FILE                 = $00000100;

      //************************************************/
      // Digital Video               MCI_SAVE          */
      //************************************************/

  MCI_DGV_SAVE_VIDEO_FILE       = $00001000;
  MCI_DGV_SAVE_IMAGE_FILE       = $00002000;

type
  mci_Save_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    pszFileName                 : pChar;       // Filename to save data to
  end;
  pmci_Save_Parms = ^mci_Save_Parms;

      //********************************************************************/
      // parameters for the MCI_SEEK message                               */
      //     0x00000X00 are reserved for MCI_SEEK flags                    */
      //********************************************************************/
const
  MCI_TO_START                  = $00000100;
  MCI_TO_END                    = $00000200;

      //************************************************/
      // Digital Video               MCI_SEEK          */
      //************************************************/

  MCI_DGV_SEEK_TO_NEAREST_IFRAME = $00100000;

      //************************************************/
      // Videodisc                   MCI_SEEK          */
      //************************************************/
  MCI_VD_SEEK_REVERSE           = $00001000;

type
  mci_Seek_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulTo                        : LongInt;       // Seek to this position
  end;
  pmci_Seek_Parms = ^mci_Seek_Parms;

      //********************************************************************/
      // Flags for the MCI_SET message                                     */
      //     0x00XXXX00 are reserved for MCI_SET  flags                    */
      //********************************************************************/
const
  MCI_SET_ITEM                  = $00000100;
  MCI_SET_ON                    = $00000200;
  MCI_SET_OFF                   = $00000400;
  MCI_SET_VIDEO                 = $00000800;
  MCI_SET_AUDIO                 = $00001000;
  MCI_SET_DOOR_OPEN             = $00002000;
  MCI_SET_DOOR_CLOSED           = $00004000;
  MCI_SET_SPEED_FORMAT          = $00008000;
  MCI_SET_TIME_FORMAT           = $00010000;
  MCI_SET_DOOR_LOCK             = $00020000;
  MCI_SET_DOOR_UNLOCK           = $00040000;
  MCI_SET_VOLUME                = $00080000;
  MCI_SET_ITEM_FOURCC           = $00100000;
  MCI_SET_REVERSE               = $00200000;

      //******************************************************/
      // Values for the ulAudio field                        */
      //******************************************************/
  MCI_SET_AUDIO_ALL             = $00000000;
  MCI_SET_AUDIO_LEFT            = $00000001;
  MCI_SET_AUDIO_RIGHT           = $00000002;




      //******************************************************/
      // MCI_SET flags for CDXA                              */
      //******************************************************/
  MCI_CDXA_SET_CHANNEL          = $01000000;
  MCI_CDXA_AUDIO_DEVICE         = $10000000;
  MCI_CDXA_AUDIO_BUFFER         = $02000000;
  MCI_CDXA_VIDEO_BUFFER         = $04000000;
  MCI_CDXA_DATA_BUFFER          = $08000000;

      //******************************************************/
      // MCI_SET flags for videodisc                         */
      //******************************************************/
  MCI_VD_SET_ON                 = MCI_SET_ON;
  MCI_VD_SET_OFF                = MCI_SET_OFF;
  MCI_VD_SET_VIDEO              = MCI_SET_VIDEO;
  MCI_VD_SET_CHANNEL            = $00100000;
  MCI_VD_SET_DISPLAY            = $00200000;
  MCI_VD_SET_DONTCARE           = $00400000;   // Dont care!

      //******************************************************/
      // Generic MCI_SET item values for video devices       */
      //******************************************************/
  MCI_VID_SET_VIDEO_COMPRESSION = MCI_VID_ITEM_BASE;
  MCI_VID_SET_IMAGE_COMPRESSION = MCI_VID_ITEM_BASE+1;
  MCI_VID_SET_IMAGE_BITSPERPEL  = MCI_VID_ITEM_BASE+2;
  MCI_VID_SET_IMAGE_PELFORMAT   = MCI_VID_ITEM_BASE+3;
  MCI_VID_SET_BRIGHTNESS        = MCI_VID_ITEM_BASE+4;
  MCI_VID_SET_CONTRAST          = MCI_VID_ITEM_BASE+5;
  MCI_VID_SET_HUE               = MCI_VID_ITEM_BASE+6;
  MCI_VID_SET_SATURATION        = MCI_VID_ITEM_BASE+7;
  MCI_VID_SET_SHARPNESS         = MCI_VID_ITEM_BASE+8;
  MCI_VID_SET_GREYSCALE         = MCI_VID_ITEM_BASE+9;
  MCI_VID_SET_IMAGE_QUALITY     = MCI_VID_ITEM_BASE+10;
  MCI_VID_SET_VIDEO_QUALITY     = MCI_VID_ITEM_BASE+11;
  MCI_VID_SET_IMAGE_COMPRESSION_METHOD = MCI_VID_ITEM_BASE+12;
  MCI_VID_SET_MINIMUM_VIDEO_REFRESH_RATE = MCI_VID_ITEM_BASE+13;
  MCI_VID_SET_IMAGE_FILE_FORMAT = MCI_VID_ITEM_BASE+14;
  MCI_VID_SET_REF_INTERVAL      = MCI_VID_ITEM_BASE+15;
  MCI_VID_SET_MAXDATARATE       = MCI_VID_ITEM_BASE+16;
  MCI_VID_SET_VIDEO_FILE_FORMAT = MCI_VID_ITEM_BASE+17;

      //******************************************************/
      // MCI_SET item values for video overlay               */
      //******************************************************/
  MCI_OVLY_SET_IMAGE_COMPRESSION = MCI_VID_SET_IMAGE_COMPRESSION;
  MCI_OVLY_SET_IMAGE_BITSPERPEL = MCI_VID_SET_IMAGE_BITSPERPEL;
  MCI_OVLY_SET_IMAGE_PELFORMAT  = MCI_VID_SET_IMAGE_PELFORMAT;
  MCI_OVLY_SET_BRIGHTNESS       = MCI_VID_SET_BRIGHTNESS;
  MCI_OVLY_SET_CONTRAST         = MCI_VID_SET_CONTRAST;
  MCI_OVLY_SET_HUE              = MCI_VID_SET_HUE;
  MCI_OVLY_SET_SATURATION       = MCI_VID_SET_SATURATION;
  MCI_OVLY_SET_SHARPNESS        = MCI_VID_SET_SHARPNESS;
  MCI_OVLY_SET_GREYSCALE        = MCI_VID_SET_GREYSCALE;
  MCI_OVLY_SET_IMAGE_QUALITY    = MCI_VID_SET_IMAGE_QUALITY;
  MCI_OVLY_SET_IMAGE_COMPRESSION_METHOD = MCI_VID_SET_IMAGE_COMPRESSION_METHOD;
  MCI_OVLY_SET_MINIMUM_VIDEO_REFRESH_RATE = MCI_VID_SET_MINIMUM_VIDEO_REFRESH_RATE;
  MCI_OVLY_SET_IMAGE_FILE_FORMAT = MCI_VID_SET_IMAGE_FILE_FORMAT;

      //******************************************************/
      // MCI_SET item values for digital video               */
      //******************************************************/
  MCI_DGV_SET_VIDEO_COMPRESSION = MCI_VID_SET_VIDEO_COMPRESSION;
  MCI_DGV_SET_IMAGE_COMPRESSION = MCI_VID_SET_IMAGE_COMPRESSION;
  MCI_DGV_SET_IMAGE_BITSPERPEL  = MCI_VID_SET_IMAGE_BITSPERPEL;
  MCI_DGV_SET_IMAGE_PELFORMAT   = MCI_VID_SET_IMAGE_PELFORMAT;
  MCI_DGV_SET_BRIGHTNESS        = MCI_VID_SET_BRIGHTNESS;
  MCI_DGV_SET_CONTRAST          = MCI_VID_SET_CONTRAST;
  MCI_DGV_SET_HUE               = MCI_VID_SET_HUE;
  MCI_DGV_SET_SATURATION        = MCI_VID_SET_SATURATION;
  MCI_DGV_SET_SHARPNESS         = MCI_VID_SET_SHARPNESS;
  MCI_DGV_SET_GREYSCALE         = MCI_VID_SET_GREYSCALE;
  MCI_DGV_SET_IMAGE_QUALITY     = MCI_VID_SET_IMAGE_QUALITY;
  MCI_DGV_SET_VIDEO_QUALITY     = MCI_VID_SET_VIDEO_QUALITY;
  MCI_DGV_SET_IMAGE_COMPRESSION_METHOD = MCI_VID_SET_IMAGE_COMPRESSION_METHOD;
  MCI_DGV_SET_MONITOR           = MCI_SET_MONITOR;
  MCI_DGV_SET_MINIMUM_VIDEO_REFRESH_RATE = MCI_VID_SET_MINIMUM_VIDEO_REFRESH_RATE;
  MCI_DGV_SET_IMAGE_FILE_FORMAT = MCI_VID_SET_IMAGE_FILE_FORMAT;
  MCI_DGV_SET_REF_INTERVAL      = MCI_VID_SET_REF_INTERVAL;
  MCI_DGV_SET_MAXDATARATE       = MCI_VID_SET_MAXDATARATE;
  MCI_DGV_SET_VIDEO_FILE_FORMAT = MCI_VID_SET_VIDEO_FILE_FORMAT;

  MCI_DGV_SET_AUDIO_COMPRESSION = MCI_DGV_ITEM_BASE;
  MCI_DGV_SET_AUDIO_QUALITY     = MCI_DGV_ITEM_BASE+1;
  MCI_DGV_SET_CHANNELS          = MCI_DGV_ITEM_BASE+2;
  MCI_DGV_SET_BITSPERSAMPLE     = MCI_DGV_ITEM_BASE+3;
  MCI_DGV_SET_SAMPLESPERSEC     = MCI_DGV_ITEM_BASE+4;
  MCI_DGV_SET_FORMATTAG         = MCI_DGV_ITEM_BASE+5;
  MCI_DGV_SET_BLOCKALIGN        = MCI_DGV_ITEM_BASE+6;
  MCI_DGV_SET_AVGBYTESPERSEC    = MCI_DGV_ITEM_BASE+7;

  MCI_DGV_SET_VIDEO_COMPRESSION_SUBTYPE = MCI_DGV_ITEM_BASE+8;
  MCI_DGV_SET_VIDEO_RECORD_RATE = MCI_DGV_ITEM_BASE+9;
  MCI_DGV_SET_VIDEO_RECORD_FRAME_DURATION = MCI_DGV_ITEM_BASE+10;
  MCI_DGV_SET_RECORD_AUDIO      = MCI_DGV_ITEM_BASE+11;
  MCI_DGV_SET_TRANSPARENT_COLOR = MCI_DGV_ITEM_BASE+12;
  MCI_DGV_SET_GRAPHIC_TRANSPARENT_COLOR = MCI_DGV_ITEM_BASE+12;
  MCI_DGV_SET_AUDIOSYNC         = MCI_DGV_ITEM_BASE+13;
  MCI_DGV_SET_VIDEO_TRANSPARENT_COLOR = MCI_DGV_ITEM_BASE+14;

      //******************************************************/
      // Video file format                                   */
      //******************************************************/
  MCI_VID_FILE_FORMAT_AVI       = $20495641;

      //******************************************************/
      // Audio/video/image compression types                 */
      //******************************************************/
  MCI_AUD_COMP_ADPCM4           = $00000001;
  MCI_AUD_COMP_PCM8             = $00000002;

  MCI_VID_COMP_RTV_2_0          = $00000101;
  MCI_VID_COMP_RTV_2_1          = $00000102;
  MCI_VID_COMP_ULTI             = $49544C55;
  MCI_VID_COMP_RT21             = $31325452;
  MCI_VID_COMP_NONE             = $20424944;   // This is 'DIB ' now.

  MCI_IMG_COMP_NONE             = $00000201;
  MCI_IMG_COMP_PIC9_1_0         = $00000202;
  MCI_IMG_COMP_PIC16_1_0        = $00000203;
  MCI_IMG_COMP_JPEG9            = $00000204;
  MCI_IMG_COMP_JPEG9_1_0        = MCI_IMG_COMP_JPEG9;
  MCI_IMG_COMP_BMP_RLE4         = $00000205;
  MCI_IMG_COMP_BMP_RLE8         = $00000206;
  MCI_IMG_COMP_JPEGN            = $00000207;

      //******************************************************/
      // PEL Formats (video/image)                           */
      //******************************************************/
  MCI_VID_PALETTE               = $00000001;
  MCI_VID_RGB                   = $00000002;
  MCI_VID_YUV                   = $00000003;

  MCI_IMG_PALETTE               = MCI_VID_PALETTE;
  MCI_IMG_RGB                   = MCI_VID_RGB;
  MCI_IMG_YUV                   = MCI_VID_YUV;


      //******************************************************/
      // Audio/video/image quality levels                    */
      //******************************************************/
  MCI_AUD_QUALITY_HIGH          = $00000001;
  MCI_AUD_QUALITY_MED           = $00000002;
  MCI_AUD_QUALITY_LOW           = $00000003;

  MCI_VID_QUALITY_HIGH          = $00000101;
  MCI_VID_QUALITY_MED           = $00000102;
  MCI_VID_QUALITY_LOW           = $00000103;
  MCI_VID_QUALITY_BEST          = MCI_VID_QUALITY_HIGH;
  MCI_VID_QUALITY_CDROM         = MCI_VID_QUALITY_MED;
  MCI_VID_QUALITY_COMPACT       = MCI_VID_QUALITY_LOW;
  MCI_VID_QUALITY_BETTER        = MCI_VID_QUALITY_MED;
  MCI_VID_QUALITY_GOOD          = MCI_VID_QUALITY_LOW;

  MCI_IMG_QUALITY_HIGH          = $00000201;
  MCI_IMG_QUALITY_MED           = $00000202;
  MCI_IMG_QUALITY_LOW           = $00000203;

      //******************************************************/
      // MCI_SET flags for wave audio                        */
      //******************************************************/
  MCI_WAVE_SET_SAMPLESPERSEC    = $01000000;
  MCI_WAVE_SET_AVGBYTESPERSEC   = $02000000;
  MCI_WAVE_SET_BLOCKALIGN       = $04000000;
  MCI_WAVE_SET_FORMATTAG        = $08000000;
  MCI_WAVE_SET_CHANNELS         = $10000000;
  MCI_WAVE_SET_BITSPERSAMPLE    = $80000000;

      //******************************************************/
      // Wave format tag defines                             */
      //******************************************************/
  MCI_WAVE_FORMAT_PCM           = DATATYPE_WAVEFORM;
  MCI_WAVE_FORMAT_ADPCM         = $0002;
  MCI_WAVE_FORMAT_IBM_CVSD      = $0005;
  MCI_WAVE_FORMAT_ALAW          = DATATYPE_RIFF_ALAW;
  MCI_WAVE_FORMAT_MULAW         = DATATYPE_RIFF_MULAW;
  MCI_WAVE_FORMAT_OKI_ADPCM     = $0010;
  MCI_WAVE_FORMAT_DVI_ADPCM     = $0011;
  MCI_WAVE_FORMAT_DIGISTD       = $0015;
  MCI_WAVE_FORMAT_DIGIFIX       = $0016;
  MCI_WAVE_FORMAT_AVC_ADPCM     = DATATYPE_ADPCM_AVC;
  MCI_WAVE_FORMAT_IBM_ADPCM     = DATATYPE_ADPCM_AVC;
  MCI_WAVE_FORMAT_IBM_MULAW     = DATATYPE_MULAW;
  MCI_WAVE_FORMAT_IBM_ALAW      = DATATYPE_ALAW;
  MCI_WAVE_FORMAT_CT_ADPCM      = DATATYPE_CT_ADPCM;
  MCI_WAVE_FORMAT_MPEG1         = DATATYPE_MPEG1AUDIO;

      //******************************************************/
      // MCI_SET flags for sequencer                         */
      //******************************************************/
  MCI_SEQ_SET_TEMPO             = $00100000;
  MCI_SEQ_SET_PORT              = $00200000;
  MCI_SEQ_ENABLE_PORT_MAPPER    = $00400000;
  MCI_SEQ_DISABLE_PORT_MAPPER   = $00800000;
  MCI_SEQ_SET_OFFSET            = $01000000;
  MCI_SEQ_SET_MASTER            = $02000000;
  MCI_SEQ_SET_SLAVE             = $04000000;

      //*********************************************/
      // SEQ time formats                           */
      //*********************************************/
  MCI_SEQ_SET_SMPTE_24          = $00000100;
  MCI_SEQ_SET_SMPTE_25          = $00000200;
  MCI_SEQ_SET_SMPTE_30          = $00000300;
  MCI_SEQ_SET_SMPTE_30DROP      = $00000400;
  MCI_SEQ_SET_SONGPTR           = $00000500;

      //*********************************************/
      // SEQ synchronization types                  */
      //*********************************************/
  MCI_SEQ_MIDI                  = $00000001;
  MCI_SEQ_SMPTE                 = $00000002;
  MCI_SEQ_FILE                  = $00000003;
  MCI_SEQ_NONE                  = $00000004;

      //*********************************************/
      // SEQ PORT TYPES                             */
      //*********************************************/
  MCI_SET_NONE                  = $10000000;
  MIDI_MAPPER                   = $20000000;
  MCI_MIDI_MAPPER               = MIDI_MAPPER;

type
  mci_Set_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulTimeFormat                : Longint;       // Time format to be used by the device
    ulSpeedFormat               : Longint;       // Speed format used by this device
    ulAudio                     : Longint;       // Channel number for this operation
                                               // (MCI_SET_AUDIO_LEFT, MCI_SET_AUDIO_RIGHT,
                                               //  MCI_SET_AUDIO_ALL)
    ulLevel                     : Longint;       // Volume, treble or bass level as % of max.
    ulOver                      : Longint;       // Delay time for vectored change in millisecond
    ulItem                      : Longint;       // Item field for set item flags
    ulValue                     : Longint;       // Value associated with item flag
  end;
  pmci_Set_Parms = ^mci_Set_Parms;

  mci_Amp_Set_Parms            = mci_Set_Parms;
  pmci_Amp_Set_Parms           = ^mci_Amp_Set_Parms;

  mci_DGV_Set_Parms            = mci_Set_Parms;
  pmci_DGV_Set_Parms           = ^mci_DGV_Set_Parms;

  mci_Ovly_Set_Parms           = mci_Set_Parms;
  pmci_Ovly_Set_Parms          = ^mci_Ovly_Set_Parms;


  mci_CDXA_Set_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulTimeFormat                : Longint;       // Time format to be used by the device
    ulSpeedFormat               : Longint;       // Speed format used by this device
    ulAudio                     : Longint;       // Channel number for this operation
                                               // (MCI_SET_AUDIO_LEFT, MCI_SET_AUDIO_RIGHT,
                                               //  MCI_SET_AUDIO_ALL)
    ulLevel                     : Longint;       // Volume, treble or bass level as % of max.
    ulOver                      : Longint;       // Delay time for vectored change in milliseconds
    ulItem                      : Longint;       // Item field for set item flags
    ulValue                     : Longint;       // Value associated with item flag
    ulChannel                   : Longint;       // Channel number
    pPlayList                   : Pointer;     // Pointer to play list
    ulPlayListSize              : Longint;       // Play list size
  end;
  pmci_CDXA_Set_Parms = ^mci_CDXA_Set_Parms;

  mci_VD_Set_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulTimeFormat                : LongInt;       // Time format to be used by the device
    ulSpeedFormat               : LongInt;       // Speed format used by this device
    ulAudio                     : LongInt;       // Channel number for this operation
                                               // (MCI_SET_AUDIO_LEFT, MCI_SET_AUDIO_RIGHT,
                                               //  MCI_SET_AUDIO_ALL)
    ulLevel                     : LongInt;       // Volume, treble or bass level as % of max.
    ulOver                      : LongInt;       // Delay time for vectored change in milliseconds
    ulItem                      : LongInt;       // Item field for set item flags
    ulValue                     : LongInt;       // Value associated with item flag
    ulChannel                   : LongInt;       // Videodisc channel
  end;
  pmci_VD_Set_Parms = ^mci_VD_Set_Parms;

  mci_Wave_Set_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulTimeFormat                : Longint;       // Time format to be used by the device
    ulSpeedFormat               : Longint;       // Speed format used by this device
    ulAudio                     : Longint;       // Channel number for this operation
                                               // (MCI_SET_AUDIO_LEFT, MCI_SET_AUDIO_RIGHT,
                                               //  MCI_SET_AUDIO_ALL)
    ulLevel                     : Longint;       // Volume, trebble or bass level as % of max.
    ulOver                      : Longint;       // Delay time for vectored change in milliseconds
    ulItem                      : Longint;       // Item field for set item flags
    ulValue                     : Longint;       // Value associated with item flag
    usInput                     : Integer;      // Channel for input
    usReserved0                 : Integer;      // Reserved field
    usOutput                    : Integer;      // Channel for output
    usReserved1                 : Integer;      // Reserved field
    usFormatTag                 : Integer;      // Format tag
    usReserved2                 : Integer;      // Reserved field
    usChannels                  : Integer;      // mono(1) or stereo(2)
    usReserved3                 : Integer;      // Reserved field
    ulSamplesPerSec             : Longint;       // Samples per seconds
    ulAvgBytesPerSec            : LongInt;       // Bytes per seconds
    usBlockAlign                : Integer;      // Block alignment of data
    usReserved4                 : Integer;      // Reserved field
    usBitsPerSample             : Integer;      // Bits per seconds
    usReserved5                 : Integer;      // Reserved field
  end;
  pmci_Wave_Set_Parms = ^mci_Wave_Set_Parms;

  mci_Seq_Set_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulTimeFormat                : Longint;       // Time format to be used by the device
    ulSpeedFormat               : Longint;       // Speed format used by this device
    ulAudio                     : Longint;       // Channel number for this operation
                                               // (MCI_SET_AUDIO_LEFT, MCI_SET_AUDIO_RIGHT,
                                               //  MCI_SET_AUDIO_ALL)
    ulLevel                     : Longint;       // Volume, trebble or bass level as % of max.
    ulOver                      : Longint;       // Delay time for vectored change in millisecond
    ulItem                      : Longint;       // Item field for set item flags
    ulValue                     : Longint;       // Value associated with item flag
    ulTempo                     : Longint;       // Specified the tempo
    ulPort                      : Longint;       // Output port
    ulSlave                     : Longint;       // Unused field
    ulMaster                    : Longint;       // Unused field
    ulOffset                    : Longint;       // Specified the data offset
  end;
  pmci_Seq_Set_Parms = ^mci_Seq_Set_Parms;


      //********************************************************************/
      // parameters for the MCI_SET_CUEPOINT message                       */
      //     0x00000x00 are reserved for MCI_SET_CUEPOINT flags            */
      //********************************************************************/
CONST
  MCI_SET_CUEPOINT_ON           = $00000100;
  MCI_SET_CUEPOINT_OFF          = $00000200;

type
  mci_CuePoint_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulCuepoint                  : Longint;       // Specifies the cuepoint location
    usUserParm                  : Integer;      // User parmameter returned on cuepoint
                                               // notify message
    usReserved0                 : Integer;      // Reserved field
  end;
  pmci_CuePoint_Parms = ^mci_CuePoint_Parms;

      //********************************************************************/
      // parameters for the MCI_SETIMAGEPALETTE message                    */
      //     0x000XXX00 are reserved for MCI_SETIMAGEPALETTE flags         */
      //********************************************************************/
CONST
  MCI_SET_REGISTERED            = $00000100;

      //********************************************************************/
      // flags and parameter structure for the MCI_SET_POSITION_ADVISE msg */
      //     0x00000X00 are reserved for MCI_SET_POSITION_ADVISE flags     */
      //********************************************************************/
  MCI_SET_POSITION_ADVISE_ON    = $00000100;
  MCI_SET_POSITION_ADVISE_OFF   = $00000200;

type
  mci_Position_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulUnits                     : Longint;       // Specifies position change notifiication
                                               // granularity
    usUserParm                  : Integer;      // User parameter returned on position change
                                               // notification message.
    Reserved0                   : Integer;      // Reserved field
    Reserved1                   : LongInt;       // Reserved field
  end;
  pmci_Position_Parms = ^mci_Position_Parms;

      //********************************************************************/
      // parameter structure for the MCI_SET_SYNC_OFFSET message           */
      //********************************************************************/
  mci_Sync_Offset_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulOffset                    : LongInt;       // Specifies the device media position offset
                                               // in the currently specified device units.
  end;
  pmci_Sync_Offset_Parms = ^mci_Sync_Offset_Parms;

      //********************************************************************/
      // flags for the MCI_SPIN message                                    */
      //     0x00000X00 are reserved for MCI_SPIN flags                    */
      //********************************************************************/
CONST
  MCI_SPIN_UP                   = $00000100;
  MCI_SPIN_DOWN                 = $00000200;

      //********************************************************************/
      // MCI_STATUS message flags                                          */
      //     0x000XXX00 are reserved for MCI_STATUS flags                  */
      //     MCI_VOLUME and MCI_VOLUME_DELAY are 0x00000100 and 0x00000200 */
      //********************************************************************/
  MCI_STATUS_ITEM               = $00000100;
  MCI_STATUS_START              = $00000200;
  MCI_STATUS_CONNECTOR          = $00000400;

      //************************************************/
      // General MCI_STATUS item values                */
      //************************************************/
  MCI_STATUS_CURRENT_TRACK      = $00000001;
  MCI_STATUS_LENGTH             = $00000002;
  MCI_STATUS_MODE               = $00000003;
  MCI_STATUS_NUMBER_OF_TRACKS   = $00000004;
  MCI_STATUS_POSITION           = $00000005;
  MCI_STATUS_POSITION_IN_TRACK  = $00000006;
  MCI_STATUS_MEDIA_PRESENT      = $00000007;
  MCI_STATUS_VOLUME             = $00000008;
  MCI_STATUS_READY              = $00000009;
  MCI_STATUS_TIME_FORMAT        = $0000000A;
  MCI_STATUS_SPEED_FORMAT       = $0000000B;
  MCI_STATUS_MONITOR            = $0000000C;
  MCI_STATUS_AUDIO              = $0000000D;
  MCI_STATUS_VIDEO              = $0000000E;
  MCI_STATUS_CLIPBOARD          = $0000000F;
  MCI_STATUS_CAN_PASTE          = $00000010;
  MCI_STATUS_CAN_REDO           = $00000020;
  MCI_STATUS_CAN_UNDO           = $00000030;

      //************************************************/
      // ulValue field values for MCI_STATUS_AUDIO     */
      //************************************************/
  MCI_STATUS_AUDIO_ALL          = $00000000;
  MCI_STATUS_AUDIO_LEFT         = $00000001;
  MCI_STATUS_AUDIO_RIGHT        = $00000002;

      //************************************************/
      // MCI_STATUS item values for amp/mixer          */
      //************************************************/
  MCI_AMP_STATUS_PITCH          = MCI_AMP_ITEM_BASE;
  MCI_AMP_STATUS_TREBLE         = MCI_AMP_ITEM_BASE+1;
  MCI_AMP_STATUS_BASS           = MCI_AMP_ITEM_BASE+2;
  MCI_AMP_STATUS_BALANCE        = MCI_AMP_ITEM_BASE+3;
  MCI_AMP_STATUS_GAIN           = MCI_AMP_ITEM_BASE+4;
  MCI_AMP_STATUS_MONITOR        = MCI_AMP_ITEM_BASE+5;
  MCI_AMP_STATUS_MID            = MCI_AMP_ITEM_BASE+6;
  MCI_AMP_STATUS_VOLUME         = MCI_STATUS_VOLUME;
  MCI_AMP_STATUS_LOUDNESS       = MCI_AMP_ITEM_BASE+7;
  MCI_AMP_STATUS_CROSSOVER      = MCI_AMP_ITEM_BASE+8;
  MCI_AMP_STATUS_REVERB         = MCI_AMP_ITEM_BASE+9;
  MCI_AMP_STATUS_ALC            = MCI_AMP_ITEM_BASE+10;
  MCI_AMP_STATUS_CHORUS         = MCI_AMP_ITEM_BASE+11;
  MCI_AMP_STATUS_CUSTOM1        = MCI_AMP_ITEM_BASE+12;
  MCI_AMP_STATUS_CUSTOM2        = MCI_AMP_ITEM_BASE+13;
  MCI_AMP_STATUS_CUSTOM3        = MCI_AMP_ITEM_BASE+14;
  MCI_AMP_STATUS_MUTE           = MCI_AMP_ITEM_BASE+15;
  MCI_AMP_STATUS_STEREOENHANCE  = MCI_AMP_ITEM_BASE+16;


      //************************************************/
      // MCI_STATUS item values for cd audio           */
      //************************************************/
  MCI_CD_STATUS_TRACK_TYPE      = MCI_CD_ITEM_BASE;
  MCI_CD_STATUS_TRACK_COPYPERMITTED = MCI_CD_ITEM_BASE+1;
  MCI_CD_STATUS_TRACK_CHANNELS  = MCI_CD_ITEM_BASE+2;
  MCI_CD_STATUS_TRACK_PREEMPHASIS = MCI_CD_ITEM_BASE+3;

      //***********************************************/
      // return values for CD MCI_STATUS message with */
      //   MCI_CD_STATUS_TRACK_TYPE set               */
      //***********************************************/
  MCI_CD_TRACK_AUDIO            = $00000001;
  MCI_CD_TRACK_DATA             = $00000002;
  MCI_CD_TRACK_OTHER            = $00000003;

      //************************************************/
      // MCI_STATUS item values for CDXA               */
      //************************************************/
  MCI_CDXA_STATUS_CHANNEL       = MCI_CDXA_ITEM_BASE;

      //***********************************************/
      //returned from call for MCI_CDXA_STATUS_CHANNEL*/
      //***********************************************/
  MCI_CDXA_NONE                 = $00000000;


      //************************************************/
      // MCI_STATUS item values for sequencer          */
      //************************************************/
  MCI_SEQ_STATUS_TEMPO          = MCI_SEQ_ITEM_BASE;
  MCI_SEQ_STATUS_OFFSET         = MCI_SEQ_ITEM_BASE+1;
  MCI_SEQ_STATUS_DIVTYPE        = MCI_SEQ_ITEM_BASE+2;
  MCI_SEQ_STATUS_MASTER         = MCI_SEQ_ITEM_BASE+3;
  MCI_SEQ_STATUS_PORT           = MCI_SEQ_ITEM_BASE+4;
  MCI_SEQ_STATUS_SLAVE          = MCI_SEQ_ITEM_BASE+5;

      //*********************************************/
      // Return value for current division type     */
      //*********************************************/
  MCI_SEQ_DIV_PPQN              = $00000001;
  MCI_SEQ_DIV_SMPTE_24          = $00000002;
  MCI_SEQ_DIV_SMPTE_25          = $00000003;
  MCI_SEQ_DIV_SMPTE_30DROP      = $00000004;
  MCI_SEQ_DIV_SMPTE_30          = $00000005;

      //************************************************/
      // MCI_STATUS items for videodisc                */
      //************************************************/
  MCI_VD_STATUS_SPEED           = MCI_VD_ITEM_BASE;
  MCI_VD_STATUS_FORWARD         = MCI_VD_ITEM_BASE+1;
  MCI_VD_MEDIA_TYPE             = MCI_VD_ITEM_BASE+2;
  MCI_VD_STATUS_SIDE            = MCI_VD_ITEM_BASE+3;
  MCI_VD_STATUS_DISC_SIZE       = MCI_VD_ITEM_BASE+4;

      //**********************************************/
      // return values for videodisc status command  */
      //**********************************************/
  MCI_VD_SPEED_NORMAL           = $00000000;
  MCI_VD_SPEED_FAST             = $00000001;
  MCI_VD_SPEED_SLOW             = $00000002;

      //************************************************/
      // MCI_STATUS items for wave audio               */
      //************************************************/
  MCI_WAVE_STATUS_LEVEL         = MCI_WAVE_ITEM_BASE;
  MCI_WAVE_STATUS_SAMPLESPERSEC = MCI_WAVE_ITEM_BASE+1;
  MCI_WAVE_STATUS_AVGBYTESPERSEC = MCI_WAVE_ITEM_BASE+2;
  MCI_WAVE_STATUS_BLOCKALIGN    = MCI_WAVE_ITEM_BASE+3;
  MCI_WAVE_STATUS_FORMATTAG     = MCI_WAVE_ITEM_BASE+4;
  MCI_WAVE_STATUS_CHANNELS      = MCI_WAVE_ITEM_BASE+5;
  MCI_WAVE_STATUS_BITSPERSAMPLE = MCI_WAVE_ITEM_BASE+6;

      //************************************************/
      // Common video MCI_STATUS items                 */
      //************************************************/
  MCI_VID_STATUS_hwnd                          = MCI_VID_ITEM_BASE;
  MCI_VID_STATUS_AUDIO_COMPRESSION             = MCI_VID_ITEM_BASE+1;
  MCI_VID_STATUS_VIDEO_COMPRESSION             = MCI_VID_ITEM_BASE+2;
  MCI_VID_STATUS_IMAGE_COMPRESSION             = MCI_VID_ITEM_BASE+3;
  MCI_VID_STATUS_AUDIO_QUALITY                 = MCI_VID_ITEM_BASE+4;
  MCI_VID_STATUS_VIDEO_QUALITY                 = MCI_VID_ITEM_BASE+5;
  MCI_VID_STATUS_IMAGE_QUALITY                 = MCI_VID_ITEM_BASE+6;
  MCI_VID_STATUS_IMAGE_BITSPERPEL              = MCI_VID_ITEM_BASE+7;
  MCI_VID_STATUS_IMAGE_PELFORMAT               = MCI_VID_ITEM_BASE+8;
  MCI_VID_STATUS_FORWARD                       = MCI_VID_ITEM_BASE+9;
  MCI_VID_STATUS_NORMAL_RATE                   = MCI_VID_ITEM_BASE+10;
  MCI_VID_STATUS_VIDEO_X_EXTENT                = MCI_VID_ITEM_BASE+11;
  MCI_VID_STATUS_VIDEO_Y_EXTENT                = MCI_VID_ITEM_BASE+12;
  MCI_VID_STATUS_IMAGE_X_EXTENT                = MCI_VID_ITEM_BASE+13;
  MCI_VID_STATUS_IMAGE_Y_EXTENT                = MCI_VID_ITEM_BASE+14;
  MCI_VID_STATUS_BRIGHTNESS                    = MCI_VID_ITEM_BASE+15;
  MCI_VID_STATUS_CONTRAST                      = MCI_VID_ITEM_BASE+16;
  MCI_VID_STATUS_HUE                           = MCI_VID_ITEM_BASE+17;
  MCI_VID_STATUS_SATURATION                    = MCI_VID_ITEM_BASE+18;
  MCI_VID_STATUS_GREYSCALE                     = MCI_VID_ITEM_BASE+19;
  MCI_VID_STATUS_SHARPNESS                     = MCI_VID_ITEM_BASE+20;
  MCI_VID_STATUS_SPEED                         = MCI_VID_ITEM_BASE+21;
  MCI_VID_STATUS_IMAGE_FILE_FORMAT             = MCI_VID_ITEM_BASE+22;
  MCI_VID_STATUS_TRANSPARENT_TYPE              = MCI_VID_ITEM_BASE+23;
  MCI_VID_STATUS_REF_INTERVAL                  = MCI_VID_ITEM_BASE+24;
  MCI_VID_STATUS_MAXDATARATE                   = MCI_VID_ITEM_BASE+25;
  MCI_VID_STATUS_VIDEO_FILE_FORMAT             = MCI_VID_ITEM_BASE+26;

      //************************************************/
      // Status Transparent Type returns               */
      // MCI_VID_PALETTE                               */
      // MCI_VID_RGB                                   */
      // MCI_VID_YUV                                   */
      //************************************************/
  MCI_VID_STATUS_TRANSPARENT_COLOR             = MCI_VID_ITEM_BASE+24;

      //************************************************/
      // MCI_STATUS items for digital video            */
      //************************************************/
  MCI_DGV_STATUS_hwnd                          = MCI_VID_STATUS_hwnd;
  MCI_DGV_STATUS_AUDIO_COMPRESSION             = MCI_VID_STATUS_AUDIO_COMPRESSION;
  MCI_DGV_STATUS_VIDEO_COMPRESSION             = MCI_VID_STATUS_VIDEO_COMPRESSION;
  MCI_DGV_STATUS_IMAGE_COMPRESSION             = MCI_VID_STATUS_IMAGE_COMPRESSION;
  MCI_DGV_STATUS_AUDIO_QUALITY                 = MCI_VID_STATUS_AUDIO_QUALITY;
  MCI_DGV_STATUS_VIDEO_QUALITY                 = MCI_VID_STATUS_VIDEO_QUALITY;
  MCI_DGV_STATUS_IMAGE_QUALITY                 = MCI_VID_STATUS_IMAGE_QUALITY;
  MCI_DGV_STATUS_IMAGE_BITSPERPEL              = MCI_VID_STATUS_IMAGE_BITSPERPEL;
  MCI_DGV_STATUS_IMAGE_PELFORMAT               = MCI_VID_STATUS_IMAGE_PELFORMAT;
  MCI_DGV_STATUS_FORWARD                       = MCI_VID_STATUS_FORWARD;
  MCI_DGV_STATUS_NORMAL_RATE                   = MCI_VID_STATUS_NORMAL_RATE;
  MCI_DGV_STATUS_VIDEO_X_EXTENT                = MCI_VID_STATUS_VIDEO_X_EXTENT;
  MCI_DGV_STATUS_VIDEO_Y_EXTENT                = MCI_VID_STATUS_VIDEO_Y_EXTENT;
  MCI_DGV_STATUS_IMAGE_X_EXTENT                = MCI_VID_STATUS_IMAGE_X_EXTENT;
  MCI_DGV_STATUS_IMAGE_Y_EXTENT                = MCI_VID_STATUS_IMAGE_Y_EXTENT;
  MCI_DGV_STATUS_BRIGHTNESS                    = MCI_VID_STATUS_BRIGHTNESS;
  MCI_DGV_STATUS_CONTRAST                      = MCI_VID_STATUS_CONTRAST;
  MCI_DGV_STATUS_HUE                           = MCI_VID_STATUS_HUE;
  MCI_DGV_STATUS_SATURATION                    = MCI_VID_STATUS_SATURATION;
  MCI_DGV_STATUS_SPEED                         = MCI_VID_STATUS_SPEED;
  MCI_DGV_STATUS_SHARPNESS                     = MCI_VID_STATUS_SHARPNESS;
  MCI_DGV_STATUS_REF_INTERVAL                  = MCI_VID_STATUS_REF_INTERVAL;
  MCI_DGV_STATUS_MAXDATARATE                   = MCI_VID_STATUS_MAXDATARATE;
  MCI_DGV_STATUS_VIDEO_FILE_FORMAT             = MCI_VID_STATUS_VIDEO_FILE_FORMAT;

  MCI_DGV_STATUS_CHANNELS                      = MCI_DGV_ITEM_BASE+13;
  MCI_DGV_STATUS_BITSPERSAMPLE                 = MCI_DGV_ITEM_BASE+14;
  MCI_DGV_STATUS_SAMPLESPERSEC                 = MCI_DGV_ITEM_BASE+15;
  MCI_DGV_STATUS_FORMATTAG                     = MCI_DGV_ITEM_BASE+16;
  MCI_DGV_STATUS_BLOCKALIGN                    = MCI_DGV_ITEM_BASE+17;
  MCI_DGV_STATUS_AVGBYTESPERSEC                = MCI_DGV_ITEM_BASE+18;
  MCI_DGV_STATUS_VIDEO_COMPRESSION_SUBTYPE     = MCI_DGV_ITEM_BASE+19;
  MCI_DGV_STATUS_VIDEO_RECORD_RATE             = MCI_DGV_ITEM_BASE+20;
  MCI_DGV_STATUS_VIDEO_RECORD_FRAME_DURATION   = MCI_DGV_ITEM_BASE+21;
  MCI_DGV_STATUS_RECORD_AUDIO                  = MCI_DGV_ITEM_BASE+22;
  MCI_DGV_STATUS_TRANSPARENT_COLOR             = MCI_DGV_ITEM_BASE+23;
  MCI_DGV_STATUS_GRAPHIC_TRANSPARENT_COLOR     = MCI_DGV_ITEM_BASE+23;  // MUST BE SAME AS TRANSPARENT COLOR
  MCI_DGV_STATUS_hwnd_MONITOR                  = MCI_DGV_ITEM_BASE+24;
  MCI_DGV_STATUS_DROPPED_FRAME_PCT             = MCI_DGV_ITEM_BASE+25;
  MCI_DGV_STATUS_AUDIOSYNC                     = MCI_DGV_ITEM_BASE+26;
  MCI_DGV_STATUS_AUDIOSYNC_DIRECTION           = MCI_DGV_ITEM_BASE+27;
  MCI_DGV_STATUS_VIDEO_TRANSPARENT_COLOR       = MCI_DGV_ITEM_BASE+28;  // MUST BE SAME AS TRANSPARENT COLOR
  MCI_DGV_STATUS_TUNER_TV_CHANNEL              = MCI_DGV_ITEM_BASE+29;
  MCI_DGV_STATUS_TUNER_LOW_TV_CHANNEL          = MCI_DGV_ITEM_BASE+29;
  MCI_DGV_STATUS_TUNER_HIGH_TV_CHANNEL         = MCI_DGV_ITEM_BASE+29;
  MCI_DGV_STATUS_TUNER_REGION                  = MCI_DGV_ITEM_BASE+30;
  MCI_DGV_STATUS_TUNER_FINETUNE                = MCI_DGV_ITEM_BASE+31;
  MCI_DGV_STATUS_TUNER_FREQUENCY               = MCI_DGV_ITEM_BASE+32;
  MCI_DGV_STATUS_TUNER_AUDIO_CHANNEL           = MCI_DGV_ITEM_BASE+33;
  MCI_DGV_STATUS_TUNER_AFC                     = MCI_DGV_ITEM_BASE+34;
  MCI_DGV_STATUS_VALID_SIGNAL                  = MCI_DGV_ITEM_BASE+35;

      //************************************************/
      // MCI_STATUS item values for video overlay      */
      //************************************************/
  MCI_OVLY_STATUS_hwnd                         = MCI_VID_STATUS_hwnd;
  MCI_OVLY_STATUS_IMAGE_COMPRESSION            = MCI_VID_STATUS_IMAGE_COMPRESSION;
  MCI_OVLY_STATUS_IMAGE_BITSPERPEL             = MCI_VID_STATUS_IMAGE_BITSPERPEL;
  MCI_OVLY_STATUS_IMAGE_PELFORMAT              = MCI_VID_STATUS_IMAGE_PELFORMAT;
  MCI_OVLY_STATUS_IMAGE_X_EXTENT               = MCI_VID_STATUS_IMAGE_X_EXTENT;
  MCI_OVLY_STATUS_IMAGE_Y_EXTENT               = MCI_VID_STATUS_IMAGE_Y_EXTENT;
  MCI_OVLY_STATUS_BRIGHTNESS                   = MCI_VID_STATUS_BRIGHTNESS;
  MCI_OVLY_STATUS_CONTRAST                     = MCI_VID_STATUS_CONTRAST;
  MCI_OVLY_STATUS_HUE                          = MCI_VID_STATUS_HUE;
  MCI_OVLY_STATUS_SATURATION                   = MCI_VID_STATUS_SATURATION;
  MCI_OVLY_STATUS_GREYSCALE                    = MCI_VID_STATUS_GREYSCALE;
  MCI_OVLY_STATUS_IMAGE_QUALITY                = MCI_VID_STATUS_IMAGE_QUALITY;
  MCI_OVLY_STATUS_SHARPNESS                    = MCI_VID_STATUS_SHARPNESS;
  MCI_OVLY_STATUS_IMAGE_FILE_FORMAT            = MCI_VID_STATUS_IMAGE_FILE_FORMAT;
  MCI_OVLY_STATUS_TRANSPARENT_TYPE             = MCI_VID_STATUS_TRANSPARENT_TYPE;
  MCI_OVLY_STATUS_TRANSPARENT_COLOR            = MCI_VID_STATUS_TRANSPARENT_COLOR;

      //************************************************/
      // Status Mode return values                     */
      //************************************************/
  MCI_MODE_NOT_READY            = $00000001;
  MCI_MODE_PAUSE                = $00000002;
  MCI_MODE_PLAY                 = $00000003;
  MCI_MODE_STOP                 = $00000004;
  MCI_MODE_RECORD               = $00000005;
  MCI_MODE_SEEK                 = $00000006;

      //************************************************/
      // Status Direction return values                */
      //************************************************/
  MCI_FORWARD                   = $00000000;
  MCI_REVERSE                   = $00000001;

type
  mci_Status_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulReturn                    : LongInt;       // Return field
    ulItem                      : LongInt;       // Item field for STATUS item to query
    ulValue                     : LongInt;       // Status value field (this used to be)
                                               //  ulTrack but was changed in Rel 1.1
                                               //  to extend the status structure.
                                               //  See the programming reference on when
                                               //  ulValue is used and how...
  end;
  pmci_Status_Parms = ^mci_Status_Parms;


  mci_CDXA_Status_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulReturn                    : LongInt;       // Return field
    ulItem                      : LongInt;       // Item field for STATUS item to query
    ulValue                     : LongInt;       // Status value field (this used to be)
                                               //  ulTrack but was changed in Rel 1.1
                                               //  to extend the status structure.
                                               //  See the programming reference on when
                                               //  ulValue is used and how...
    ulChannel                   : LongInt;       // Channel
  end;
  pmci_CDXA_Status_Parms = ^mci_CDXA_Status_Parms;

      //********************************************************************/
      // flags and parameter structure for the MCI_STEP message            */
      //     0x00000X00 are reserved for MCI_STEP flags                    */
      //********************************************************************/
CONST
  MCI_STEP_FRAMES               = $00000100;
  MCI_STEP_REVERSE              = $00000200;

type
  mci_Step_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulStep                      : LongInt;       // amount to step in current time format
  end;
  pmci_Step_Parms = ^mci_Step_Parms;

      //********************************************************************/
      // flags for the MCI_SYSINFO message                                 */
      //     0xXXXXXX00 are reserved for MCI_SYSINFO flags                 */
      //********************************************************************/
CONST
  MCI_SYSINFO_QUANTITY          = $00000100;
  MCI_SYSINFO_OPEN              = $00000200;
  MCI_SYSINFO_NAME              = $00000400;
  MCI_SYSINFO_INSTALLNAME       = $00000800;
  MCI_SYSINFO_ITEM              = $00001000;
  MCI_SYSINFO_INI_LOCK          = $00002000;

      //*******************************************************/
      // Extended SYSINFO functions                           */
      //*******************************************************/
  MCI_SYSINFO_INSTALL_DRIVER    = $00000001;
  MCI_SYSINFO_QUERY_DRIVER      = $00000002;
  MCI_SYSINFO_DELETE_DRIVER     = $00000004;
  MCI_SYSINFO_SET_PARAMS        = $00000008;
  MCI_SYSINFO_QUERY_PARAMS      = $00000010;
  MCI_SYSINFO_SET_CONNECTORS    = $00000020;
  MCI_SYSINFO_QUERY_CONNECTORS  = $00000040;
  MCI_SYSINFO_SET_EXTENSIONS    = $00000080;
  MCI_SYSINFO_QUERY_EXTENSIONS  = $00000100;
  MCI_SYSINFO_SET_ALIAS         = $00000200;
  MCI_SYSINFO_QUERY_NAMES       = $00000400;
  MCI_SYSINFO_SET_DEFAULT       = $00000800;
  MCI_SYSINFO_QUERY_DEFAULT     = $00001000;
  MCI_SYSINFO_SET_TYPES         = $00002000;
  MCI_SYSINFO_QUERY_TYPES       = $00004000;

      //*******************************************************/
      // Device Flags                                         */
      //*******************************************************/
  MCI_SYSINFO_DEV_CONTROLLABLE  = $00000001;
  MCI_SYSINFO_DEV_NONCONTROLLABLE = $00000002;
  MCI_SYSINFO_DEVICESETTINGS    = $00000004;

      //********************************************************************/
      // parameter structures for the MCI_SYSINFO message                  */
      //********************************************************************/
TYPE
  MaxDevNameChar = array [0..Max_Device_Name-1] of CHAR;
  MaxVerNumChar = array [0..Max_Version_Number-1] of CHAR;
  MaxProdInfChar = array [0..Max_ProdInfo-1] of CHAR;
  MaxPddNameChar = array [0..Max_Pdd_Name-1] of CHAR;
  MaxClsShort    = array [0..Max_Classes-1] of integer;
  MaxClsShort2   = array [0..Max_Classes-1,0..Max_Classes-1] of integer;
  MaxDevParmChar = array [0..Max_Dev_Params-1] of CHAR;

  mci_SysInfo_Parms = record
    hwndDummyCallback        : hwnd;           // NOTIFY not allowed for SYSINFO
    pszReturn                : pChar;          // Pointer to return buffer
    ulRetSize                : LongInt;          // Return buffer size
    ulNumber                 : LongInt;          // Ordinal Number
    usDeviceType             : Integer;         // MCI device type number
    usReserved0              : Integer;         // Reserved field
    ulItem                   : LongInt;          // Used to indicate the MCI_SYSINFO
    pSysInfoParm             : Pointer;        // extended function to perform
  end;
  pmci_SysInfo_Parms = ^mci_SysInfo_Parms;

  mci_SysInfo_LogDevice = packed record
    szInstallName            : MaxDevNameChar; // Device install name
    usDeviceType             : Integer;         // Device type number
    ulDeviceFlag             : Longint;          // Flag indicating whether device
                                               //  device is controllable or not
    szVersionNumber          : MaxVerNumChar;  // INI file version number
    szProductInfo            : MaxProdInfChar; // Textual product description
    szMCDDriver              : MaxDevNameChar; // MCI Driver dll name
    szVSDDriver              : MaxDevNameChar; // VSD dll name
    szPDDName                : MaxPddNameChar; // Device PDD name
    szMCDTable               : MaxDevNameChar; // Device type command table
    szVSDTable               : MaxDevNameChar; // Device specific command table
    usShareType              : Integer;         // Device sharing mode
    szResourceName           : MaxDevNameChar; // Resource name
    usResourceUnits          : Integer;         // Total resource units available
                                               //  for this device
    usResourceClasses        : Integer;         // Number of resource classes for
                                               //  this device
    ausClassArray            : MaxClsShort;    // Maximum number of resource
                                               //  units for each class
    ausValidClassArray       : MaxClsShort2;   // Valid class combinations
  end;
  pmci_SysInfo_LogDevice = ^mci_SysInfo_LogDevice;


  mci_SysInfo_DevParams = record
    szInstallName            : MaxDevNameChar; // Device install name
    szDevParams              : MaxDevParmChar; // Device specific parameters
  end;
  pmci_SysInfo_DevParams = ^mci_SysInfo_DevParams;

type
  Connect = record
    usConnectType            : Integer;         // Connector type
    szToInstallName          : MaxDevNameChar; // Install name this connector
                                               //  is connected to
    usToConnectIndex         : Integer;         // Connector index this connector
                                               //  is connected to
  end;
  pConnect = ^Connect;


  MaxCnctCnct      = array[0..MAX_CONNECTORS-1] of Connect;
  MaxExtChar2      = array[0..MAX_EXTENSIONS-1,0..MAX_EXTENSION_NAME-1] of CHAR;
  MaxAliasNameChar = array[0..Max_Alias_Name-1] of Char;
  MaxTypeBufChar   = array[0..Max_TypeBuffer] of Char;

  mci_SysInfo_ConParams = record
    szInstallName            : MaxDevNameChar; // Device install name
    usNumConnectors          : Integer;         // Number of device connectors
    ConnectorList            : MaxCnctCnct;    // Connector list array
  end;
  pmci_SysInfo_ConParams = ^mci_SysInfo_ConParams;

  mci_SysInfo_Extension = record
    szInstallName            : MaxDevNameChar; // Device install name
    usNumExtensions          : Integer;         // Number of extensions
    szExtension              : MaxExtChar2;    // Extension name array
  end;
  pmci_SysInfo_Extension = ^mci_SysInfo_Extension;

  mci_SysInfo_Alias = record
    szInstallName            : MaxDevNameChar;   // Device install name
    szAliasName              : MaxAliasNameChar; // Alias name
  end;
  pmci_SysInfo_Alias = ^mci_SysInfo_Alias;

  mci_SysInfo_DefaultDevice = record
    szInstallName            : MaxDevNameChar; // Device install name
    usDeviceType             : Integer;         // Device type number
  end;
  pmci_SysInfo_DefaultDevice = ^mci_SysInfo_DefaultDevice;

  mci_SysInfo_Query_Name = record
    szInstallName            : MaxDevNameChar; // Device install name
    szLogicalName            : MaxDevNameChar; // Logical device name
    szAliasName              : MaxAliasNameChar; // Alias name
    usDeviceType             : Integer;         // Device type number
    usDeviceOrd              : Integer;         // Device type ordinal
  end;
  pmci_SysInfo_Query_Name = ^mci_SysInfo_Query_Name;

  mci_SysInfo_Types = record
    szInstallName            : MaxDevNameChar; // Device install name
    szTypes                  : MaxTypeBufChar; // EA types
  end;
  pmci_SysInfo_Types = ^mci_SysInfo_Types;


      //********************************************************************/
      // flags for the MCI_UNFREEZE message                                */
      //     0x00000X00 are reserved for MCI_UNFREEZE flags                */
      //********************************************************************/
CONST
  MCI_OVLY_UNFREEZE_RECT         = $00000100;
  MCI_OVLY_UNFREEZE_RECT_OUTSIDE = $00000200;

      //********************************************************************/
      // flags for the MCI_WHERE message                                   */
      //     0x0000XX00 are reserved for MCI_WHERE flags                   */
      //********************************************************************/
  MCI_VID_WHERE_DESTINATION     = $00000100;
  MCI_VID_WHERE_SOURCE          = $00000200;
  MCI_VID_WHERE_WINDOW          = $00000400;

  MCI_DGV_WHERE_DESTINATION     = MCI_VID_WHERE_DESTINATION;
  MCI_DGV_WHERE_SOURCE          = MCI_VID_WHERE_SOURCE;
  MCI_DGV_WHERE_WINDOW          = MCI_VID_WHERE_WINDOW;
  MCI_DGV_WHERE_ADJUSTED        = $00000800;

  MCI_OVLY_WHERE_DESTINATION    = MCI_VID_WHERE_DESTINATION;
  MCI_OVLY_WHERE_SOURCE         = MCI_VID_WHERE_SOURCE;
  MCI_OVLY_WHERE_WINDOW         = MCI_VID_WHERE_WINDOW;
      //********************************************************************/
      // flags and parameter structure for the MCI_WINDOW message          */
      //     0x0000XX00   are reserved for MCI_WINDOW flags                */
      //********************************************************************/
  MCI_VID_WINDOW_hwnd           = $00000100;
  MCI_VID_WINDOW_STATE          = $00000200;
  MCI_VID_WINDOW_TEXT           = $00000400;
  MCI_VID_WINDOW_DEFAULT        = $00000800;

  MCI_DGV_WINDOW_hwnd           = MCI_VID_WINDOW_hwnd;
  MCI_DGV_WINDOW_STATE          = MCI_VID_WINDOW_STATE;
  MCI_DGV_WINDOW_TEXT           = MCI_VID_WINDOW_TEXT;
  MCI_DGV_WINDOW_DEFAULT        = MCI_VID_WINDOW_DEFAULT;

  MCI_OVLY_WINDOW_hwnd          = MCI_VID_WINDOW_hwnd;
  MCI_OVLY_WINDOW_STATE         = MCI_VID_WINDOW_STATE;
  MCI_OVLY_WINDOW_TEXT          = MCI_VID_WINDOW_TEXT;
  MCI_OVLY_WINDOW_DEFAULT       = MCI_VID_WINDOW_DEFAULT;

type
  mci_Vid_Window_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    hwndDest                    : hwnd;        // Handle to the client window used for
                                               // the destination of the video image
    usCmdShow                   : Integer;      // Specifies how the window is displayed
    usReserved1                 : Integer;      // Reserved
    pszText                     : pChar;       // The text to use as the window caption
    pszAlias                    : pChar;       // The window alias for the display window
  end;
  pmci_Vid_Window_Parms = ^mci_Vid_Window_Parms;

  mci_dgv_Window_Parms = mci_Vid_Window_Parms;
  pmci_dgv_Window_Parms = ^mci_dgv_Window_Parms;

  mci_Ovly_Window_Parms = mci_Vid_Window_Parms;
  pmci_Ovly_Window_Parms = ^mci_Ovly_Window_Parms;


      //********************************************************************/
      // flags and parameter structure for the MCI_TUNER  message          */
      //********************************************************************/
CONST
  MCI_DGV_AUDIO_CHANNEL         = $00010000;
  MCI_DGV_TV_CHANNEL            = $00020000;
  MCI_DGV_FREQUENCY             = $00040000;
  MCI_DGV_FINETUNE_PLUS         = $00080000;
  MCI_DGV_FINETUNE_MINUS        = $00100000;
  MCI_DGV_REGION                = $00200000;
  MCI_DGV_AFC_ON                = $00400000;
  MCI_DGV_AFC_OFF               = $00800000;
  MCI_DGV_POLARIZATION          = $00800000;
  MCI_DGV_QUERY                 = $01000000;

type
  mci_Dgv_Tuner_Parms = record
    hwndCallback                : hwnd;        // PM window handle for MCI notify message
    ulFrequency                 : LongInt;       // Tuner Frequency
    ulReserved0                 : LongInt;       // Reserved 0
    ulTVChannel                 : LongInt;       // TV Channel
    lFineTune                   : LongInt;        // Fine tuning adjustments.
    pszRegion                   : pChar;       // TV Channel Region
    ulReserved1                 : LongInt;       // Reserved 1
    ulReserved2                 : LongInt;       // Reserved 2
  end;
  pmci_Dgv_Tuner_Parms = ^mci_Dgv_Tuner_Parms;

      //********************************************************************/
      //                                                                   */
      //  MCI system value defines                                         */
      //                                                                   */
      //********************************************************************/
CONST
  MSV_CLOSEDCAPTION             = $0;
  MSV_MASTERVOLUME              = $1;
  MSV_HEADPHONES                = $2;
  MSV_SPEAKERS                  = $3;
  MSV_WORKPATH                  = $4;
  MSV_SYSQOSVALUE               = $5;
  MSV_SYSQOSERRORFLAG           = $6;

  MSV_MAX                       = $7;


      //********************************************************************/
      // Playlist defines                                                  */
      //********************************************************************/
  DATA_OPERATION                = 0;
  BRANCH_OPERATION              = 1;
  LOOP_OPERATION                = 2;
  CALL_OPERATION                = 3;
  RETURN_OPERATION              = 4;
  EXIT_OPERATION                = 5;
  NOP_OPERATION                 = 6;
  MESSAGE_OPERATION             = 7;
  CUEPOINT_OPERATION            = 8;
  SEMWAIT_OPERATION             = 9;
  SEMPOST_OPERATION             = 10;

      //********************************************************************/
      //                                                                   */
      //  MCI Function prototypes                                          */
      //                                                                   */
      //********************************************************************/


function mciSendCommand(usDeviceID: Integer; usMessage: Integer; ulParam1: LongInt;
         var Param2; usUserParm: Integer): longint; cdecl;

function mciSendString(pszCommandBuf: pChar; pszReturnString: pChar;
         wReturnLength: Integer; hwndCallBack: hwnd; usUserParm: Integer): longInt; cdecl;

function mciGetErrorString(ulError: LongInt; pszBuffer: pChar; usLength: Integer): longint; cdecl;

function mciMakeGroup(var usDeviceGroupID:Integer; usDeviceCount: Integer;
             var ausDeviceList: Integer; ulFlags: LongInt; ulMMTime: LongInt): longint; cdecl;

function mciDeleteGroup(usGroupID:Integer): longint; cdecl;

function mciSetSysValue(iSysValue: Integer; var Value): Boolean; cdecl;

function mciQuerySysValue(iSysValue: Integer;var Value): Boolean; cdecl;

function mciGetDeviceID(pszName: pChar): Longint; cdecl;

Implementation

function mSecToMM(Value: Cardinal): mmTime;
begin
  If Value > $FFFFFFFF div 3 then
    mSecToMM := 0
  else
    mSecToMM := Value div 3;
end;

function mSecFromMM(Value: mmTime): LongInt;
begin
  mSecFromMM := (Value+1) div 3;
end;

function RedBookToMM(Value: Cardinal): mmTime;
begin
  RedBookToMM := ( Value and $0000FF ) * 60 * 3000 +
            ( Value and $00FF00 ) * $100 * 3000 +
            ( Value and $FF0000 ) * $10000 * 3000 div 75;

end;

function fps24ToMM(Value: Cardinal): mmTime;
begin
  fps24toMM := ( Value and $000000FF ) * 60 * 60 * 3000 +
            ( Value and $0000FF00 ) * $100 * 60 * 3000 +
            ( Value and $00FF0000 ) * $10000 * 3000 +
            ( Value and $FF000000 ) * $10000 * 3000 div 24;
end;

function fps25ToMM(Value: Cardinal): mmTime;
begin
  fps25ToMM := ( Value and $000000FF ) * 60 * 60 * 3000 +
            ( Value and $0000FF00 ) * $100 * 60 * 3000 +
            ( Value and $00FF0000 ) * $10000 * 3000 +
            ( Value and $FF000000 ) * $10000 * 3000 div 25;
end;

function fps30ToMM(Value: Cardinal): mmTime;
begin
  fps30ToMM := ( Value and $000000FF ) * 60 * 60 * 3000 +
            ( Value and $0000FF00 ) * $100 * 60 * 3000 +
            ( Value and $00FF0000 ) * $10000 * 3000 +
            ( Value and $FF000000 ) * $10000 * 3000 div 30;
end;

function HMSToMM(Value: Cardinal): mmTime;
begin
  HMSToMM := ( Value and $0000FF ) * 60 * 60 * 3000 +
            ( Value and $00FF00 ) * $100 * 60 * 3000 +
            ( Value and $FF0000 ) * $10000 * 3000;
end;

function RedBookFromMM(Value: mmTime): Cardinal;
begin
  if Value+20 >= $100*60*3000 then
    RedBookFromMM := 0
  else
    RedBookFromMM := (Value+20) div (60*3000) +
              ((Value+20) mod ((60*3000) div 3000)) shl 8 +
              ((Value+20) div ((3000 div 75) mod 75)) shl 16;
end;

function FPS24FromMM(Value: mmTime): Cardinal;
begin
  if Value+63 >= $A4CB8000 then // $100*60*60*3000 then
    FPS24FromMM := 0
  else
    FPS24FromMM := ((((Value+63) mod 3000) div (3000 div 24)) shl 24) and $FF000000 +
              ((((Value+63) div 3000) mod 60) shl 16) and $00FF0000 +
              (((((Value+63) div 3000) div 60) mod 60) shl 8) and $0000FF00 +
              ((((Value+63) div 3000) div 60) div 60) and $000000FF;
end;

function FPS25FromMM(value: mmTime): Cardinal;
begin
  if Value+60 >= $A4CB8000 then // $100*60*60*3000
    FPS25FromMM := 0
  else
    FPS25FromMM := ((((Value+60) mod 3000) div (3000 div 25)) shl 24) and $FF000000 +
              ((((Value+60) div 3000) mod 60) shl 16) and $00FF0000 +
              (((((Value+60) div 3000) div 60) mod 60) shl 8) and $0000FF00 +
              ((((Value+60) div 3000) div 60) div 60) and $000000FF;
end;

function FPS30FromMM(value: mmTime): Cardinal;
begin
  if Value+50 >= $A4CB8000 then // $100*60*60*3000
    FPS30FromMM := 0
  else
    FPS30FromMM := ((((Value+50) mod 3000) div (3000 div 30)) shl 24) and $FF000000 +
              ((((Value+50) div 3000) mod 60) shl 16) and $00FF0000 +
              (((((Value+50) div 3000) div 60) mod 60) shl 8) and $0000FF00 +
              ((((Value+50) div 3000) div 60) div 60) and $000000FF;
end;

function HMSFromMM(value: mmTime): Cardinal;
begin
  if Value+50 >= $A4CB8000 then // $100*60*60*3000
    HMSFromMM := 0
  else
    HMSFromMM := (((( Value+50) div 3000) mod 60) shl 16) and $00FF0000 +
              (((((Value+50) div 3000) div 60) mod 60) shl 8) and $0000FF00 +
              (((( Value+50) div 3000) div 60) div 60) and $000000FF;
end;

function tmsf_track(time: mmTime): Byte;
begin
  tmsf_track := byte(time);
end;

function tmsf_minute(time: mmTime): Byte;
begin
  tmsf_minute := pbyte(pchar(@time)+1)^;
end;

function tmsf_second(time: mmTime): Byte;
begin
  tmsf_second := pbyte(pchar(@time)+2)^;
end;

function tmsf_frame(time: mmTime): Byte;
begin
  tmsf_frame := pbyte(pchar(@time)+3)^;
end;

function msf_minute(time: mmTime): Byte;
begin
  msf_minute := byte(time);
end;

function msf_second(time: mmTime): Byte;
begin
  msf_second := pbyte(pchar(@time)+1)^;
end;

function msf_frame(time: mmTime): Byte;
begin
  msf_frame := pbyte(pchar(@time)+2)^;
end;

function uLong_lwlb(var ul): Byte;   // Low word low byte
begin
  uLong_lwlb := LongInt(ul);
end;

function uLong_lwhb(var ul): Byte;   // Low word high byte
begin
  uLong_lwhb := pbyte(pchar(@ul)+1)^;
end;

function uLong_hwlb(var ul): Byte;   // High word low byte
begin
  uLong_hwlb := pbyte(pchar(@ul)+2)^;
end;

function uLong_hwhb(var ul): Byte;   // High word high byte
begin
  uLong_hwhb := pbyte(pchar(@ul)+3)^;
end;

function uLong_lowd(var ul): Word;   // Low word
begin
  uLong_lowd:=pWord(pchar(@ul))^;
end;

function uLong_hiwd(var ul): Word;   // High word
begin
  uLong_hiwd := pWord(pchar(@ul)+Sizeof(Word))^;
end;

function mciSendCommand(usDeviceID: Integer; usMessage: Integer; ulParam1: LongInt;
         var Param2; usUserParm: Integer): longint; cdecl; external 'MDM' index 1;

function mciSendString(pszCommandBuf: pChar; pszReturnString: pChar;
         wReturnLength: Integer; hwndCallBack: hwnd; usUserParm: Integer): longInt; cdecl; external 'MDM' index 2;

function mciGetErrorString(ulError: LongInt; pszBuffer: pChar; usLength: Integer): longint;  cdecl; external 'MDM' index 3;

function mciMakeGroup(var usDeviceGroupID:Integer; usDeviceCount: Integer;
             var ausDeviceList: Integer; ulFlags: LongInt; ulMMTime: LongInt): longint; cdecl; external 'MDM' index 12;

function mciDeleteGroup(usGroupID:Integer): longint; cdecl; external 'MDM' index 13;

function mciSetSysValue(iSysValue: Integer; var Value): Boolean; cdecl; external 'MDM' index 10;

function mciQuerySysValue(iSysValue: Integer;var Value): Boolean; cdecl; external 'MDM' index 11;

function mciGetDeviceID(pszName: pChar): Longint; cdecl; external 'MDM' index 16;

End.
