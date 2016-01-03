{
    Copyright (c) 1990-1993 International Business Machines Corporation
    Copyright (c) 2002 by Andry Svirgunov (cool2@ngs.ru)
    Copyright (c) 2002-2003 by Yuri Prokushev (prokushev@freemail.ru)

    OS/2 Multimedia structures and definitions

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
@abstract(Multimedia structures and definitions)
@author(Andry Svirgunov (cool2@ngs.ru))
@author(Yuri Prokushev (prokushev@freemail.ru))
@created(17 Dec 2002)
@lastmod(19 Jan 2003)
OS/2 Multimedia structures and definitions
Warning: This code is alfa. Future versions of this unit will propably
not be compatible.
}
//****************************************************************************/
//*                                                                          */
//* Module Name: MMBase.pas                                                  */
//*                                                                          */
//* OS/2 2.0 Multimedia Extensions Base Definitions                          */
//*                                                                          */
//* Copyright (c) International Business Machines Corporation 1991, 1992     */
//*                        All Rights Reserved                               */
//*                                                                          */
//*------------------------------------------------------------------------- */
//* Converted by Andry Svirgunov. Email: cool2@ngs.ru                        */
//* 14.11.2002.                                                              */
//* Revised by Yuri Prokushev (prokushev@freemail.ru)                        */
//****************************************************************************/

Unit mmbase;

{$MODE ObjFPC}

Interface

Uses Os2Def;

Type
        VERSION = WORD;
        LPSTR = ^Char;
        LPTR = Pointer;
        LPRECT = Pointer;
        HANDLE = HWND;
        PW = ^Word;
        PAW = ^Word;
        PFN = Pointer ;
        PPFN = ^PFN;

type
  FNMCI=Function(var p,w:Word;a,b,c:LongInt):Longint;

TYPE
        PFNMCI = ^FNMCI;
        SZ = Char;              //* ASCIIZ char string type*/
        MMTIME = LongInt;       //* universal Chinatown time (1/3000 second)*/
        PMMTIME = ^MMTIME;      //* Ptr to MMTIME unit*/
        HMMIO = LongInt;        //* Handle to an MMIO object*/
        PHMMIO = ^HMMIO;        //* Handle to an MMIO object*/


TYPE
  FOURCC=Cardinal;

//**********************************************
//*
//* SPCBKEY - Stream Protocol Key
//*
//**********************************************/
Type _SPCBKEY = record                  //* spcbkey SPCB key (Stream data type) */
        ulDataType : LongInt;
        ulDataSubType : LongInt;
        ulIntKey : LongInt;             //* generated internal key            */
        end;
TYPE PSPCBKEY = ^_SPCBKEY;

Const

        _SPCBKEY_DEFINED = 1;

//**********************************************
// *
// * MMTRACKINFO - This structure is used to
// *            represent a video, audio or
// *            some other type of track
// *            within a digital video movie
// *            file.
// *
// **********************************************/
TYPE _MMTRACKINFO = record      //* mmtrackinfo */
        ulTrackID : LongInt;    //* track identifier */
        ulMediaType : LongInt;  //* media type */
        ulCountry : LongInt;    //* country code for the track */
        ulCodePage : LongInt;   //* country code page for the track */
        ulReserved1: LongInt;   //* reserved must be 0 */
        ulReserved2 : LongInt;  //* reserved must be 0 */
        end;

TYPE PMMTRACKINFO = ^_MMTRACKINFO; //* Ptr to a track table entry */

//************************
// * Clipboard formats
// ************************/
CONST
        CF_RMID = 10;
        CF_RIFF = 11;
        CF_WAVE = 12;
        CF_AVI = 13;
//************************
// * Resource formats
// ************************/
        RT_RMID = 100;
        RT_RIFF = 101;
        RT_WAVE = 102;
        RT_AVI = 103;
        RT_AVS = 104;


//************************
// * Drag formats
// ************************/
        DRT_WAVE = 'DIGITAL AUDIO';
        DRT_AVI = 'DIGITAL VIDEO';
        DRT_MIDI = 'MIDI';


//************************
// * Sampling Rate
// ************************/
        HZ_8000 = 8000;         //* 8.0 kHz*/
        HZ_11025 = 11025;       //* 11.025 kHz*/
        HZ_14700 = 14700;       //* 14.700 kHz (SPV/2)*/
        HZ_18900 = 18900;       //* 18.900 kHz (CD/XA LVL C)*/
        HZ_22050 = 22050;       //* 22.050 kHz*/
        HZ_37800 = 37800;       //* 37.800 kHz (CD/XA LVL B)*/
        HZ_44100 = 44100;       //* 44.100 kHz*/

//************************
// * Bits per Sample
// ************************/
        BPS_4 = 4;              //* 4 bits/sample (ADPCM)*/
        BPS_8 = 8;              //* 8 bits/sample (PCM)*/
        BPS_16 =16;             //* 16 bits/sample(PCM)*/

//************************
//* Channels
//************************/
        CH_1 = 1;               //* Mono*//
        CH_2 = 2;               //* Stereo*//
        CH_4 = 4;               //* Quad*//


//*************************************************************************
// * Multimedia Datatypes (spcbkey values)
// *************************************************************************/


///****************
// * NULL datatype
// ****************/
        DATATYPE_NULL = $0000;  //* NULL stream data type (NULL)*/
        SUBTYPE_NONE = $0000;   //* No subtype for this data type*/

//*********************
// * WAVEFORM datatypes
// *********************/
        DATATYPE_WAVEFORM = $0001; //* WAVEFORM audio (PCM)*/


//* Subtypes for DATATYPE_WAVEFORM:*/
        WAVE_FORMAT_1M08 = $0001;       //* 11.025kHz, Mono  , 8-bit*/
        WAVE_FORMAT_1S08 = $0002;       //* 11.025kHz, Stereo, 8-bit*/
        WAVE_FORMAT_1M16 = $0003;       //* 11.025kHz, Mono  , 16-bit*/
        WAVE_FORMAT_1S16 = $0004;       //* 11.025kHz, Stereo, 16-bit*/
        WAVE_FORMAT_2M08 = $0005;       //* 22.05kHz , Mono  , 8-bit*/
        WAVE_FORMAT_2S08 = $0006;       //* 22.05kHz , Stereo, 8-bit*/
        WAVE_FORMAT_2M16 = $0007;       //* 22.05kHz , Mono  , 16-bit*/
        WAVE_FORMAT_2S16 = $0008;       //* 22.05kHz , Stereo, 16-bit*/
        WAVE_FORMAT_4M08 = $0009;       //* 44.1kHz  , Mono  , 8-bit*/
        WAVE_FORMAT_4S08 = $000A;       //* 44.1kHz  , Stereo, 8-bit*/
        WAVE_FORMAT_4M16 = $000B;       //* 44.1kHz  , Mono  , 16-bit*/
        WAVE_FORMAT_4S16 = $000C;       //* 44.1kHz  , Stereo, 16-bit*/
        WAVE_FORMAT_8M08 = $000D;       //*  8.0kHz  , Mono  , 8-bit*/
        WAVE_FORMAT_8S08 = $000E;       //*  8.0kHz  , Stereo, 8-bit*/
        WAVE_FORMAT_8M16 = $000F;       //*  8.0kHz  , Mono  , 16-bit*/
        WAVE_FORMAT_8S16 = $0010;       //*  8.0kHz  , Stereo, 16-bit*/

//*********************
// * DIGVIDEO datatypes
// *********************/
        DATATYPE_DIGVIDEO = $0002;      //* Digital Video */

//* Subtypes for DATATYPE_DIGVIDEO:*/
        DIGVIDEO_PHOTOM = $544F4850;    //* Photmotion video, FOURCC value for phio */
        DIGVIDEO_ULTIM = $49544C55;     //* Ultimotion video, FOURCC value for ulio */
        DIGVIDEO_AVI = $20495641;       //* AVI video, FOURCC value for avio */
        DIGVIDEO_MONITOR = $206E6FD;    //* MONITOR video, FOURCC value for monitoring */
        DIGVIDEO_FLC = $63696C66;       //* FLIC video, FOURCC value for flic */
        DIGVIDEO_MPEG = $4745504D;      //* MPEG video, FOURCC value for MPEG */

//******************
// * MuLaw datatypes
//******************/
        DATATYPE_MULAW = $0101;         //* MuLaw*/
        DATATYPE_RIFF_MULAW = $0007;    //* MuLaw*/

//* Subtypes for DATATYPE_MULAW:*/

        MULAW_8B8KS = $0001;            //* 8bit 8kHz stereo*/
        MULAW_8B11KS = $0002;           //* 8bit 11kHz stereo*/
        MULAW_8B22KS = $0003;           //* 8bit 22kHz stereo*/
        MULAW_8B44KS = $0004;           //* 8bit 44kHz stereo*/
        MULAW_8B8KM = $0005;            //* 8bit 8kHz mono*/
        MULAW_8B11KM = $0006;           //* 8bit 11kHz mono*/
        MULAW_8B22KM = $0007;           //* 8bit 22kHz mono*/
        MULAW_8644KM = $0008;           //* 8bit 44kHz mono*/

//*****************
// * ALaw datatypes
// *****************/
        DATATYPE_ALAW = $0102;          //* ALaw*/
        DATATYPE_RIFF_ALAW = $0006;     //* ALaw*/

//* Subtypes for DATATYPE_ALAW:*/
        ALAW_8B8KS = $0001;             //* 8bit 8kHz stereo*/
        ALAW_8B11KS = $0002;            //* 8bit 11kHz stereo*/
        ALAW_8B22KS = $0003;            //* 8bit 22kHz stereo*/
        ALAW_8B44KS = $0004;            //* 8bit 44kHz stereo*/
        ALAW_8B8KM = $0005;             //* 8bit 8kHz mono*/
        ALAW_8B11KM = $0006;            //* 8bit 11kHz mono*/
        ALAW_8B22KM = $0007;            //* 8bit 22kHz mono*/
        ALAW_8B44KM = $0008;            //* 8bit 44kHz mono*/

//*******************************
// * AVC ADPCM WAVEFORM datatypes
// *******************************/
        DATATYPE_ADPCM_AVC = $0103;     //* ADPCM audio*/

//* Subtypes for DATATYPE_ADPCM_AVC:*/
        ADPCM_AVC_VOICE = $0001;
        ADPCM_AVC_MUSIC = $0002;
        ADPCM_AVC_STEREO = $0003;
        ADPCM_AVC_HQ = $0004;

//******************
// * CT_ADPCM datatypes
// ******************/
        DATATYPE_CT_ADPCM = $0200;      //* Creative technologies */

//* Subtypes for DATATYPE_CT_ADPCM:*/

        CD_ADPCM_16B8KS = $0001;        //* 16bit 8kHz stereo*/
        CD_ADPCM_16B11KS = $0002;       //* 16bit 11kHz stereo*/
        CD_ADPCM_16B22KS = $0003;       //* 16bit 22kHz stereo*/
        CD_ADPCM_16B44KS = $0004;       //* 16bit 44kHz stereo*/
        CD_ADPCM_16B8KM = $0005;        //* 16bit 8kHz mono*/
        CD_ADPCM_16B11KM = $0006;       //* 16bit 11kHz mono*/
        CD_ADPCM_16B22KM = $0007;       //* 16bit 22kHz mono*/
        CD_ADPCM_16B44KM = $0008;       //* 16bit 44kHz mono*/

//****************
// * MIDI datatype
// ****************/

        DATATYPE_MIDI = $0201;          //* MIDI*/
//*         SUBTYPE_NONE            0x0000L      No subtype for this data type*/


//********************
// * GENERIC datatypes
// ********************/
        DATATYPE_GENERIC = $0202;       //* Generic for files / memory. This*/
                                        //* datatype will match any other*/
                                        //* datatype at stream creation time.*/

//**************************
// * Speech Viewer datatypes
// **************************/

        DATATYPE_SPV2 = $0203;          //* Speech Viewer*/

//* Subtypes for DATATYPE_SPV2:*/

        SPV2_BPCM = $0001;
        SPV2_PCM = $0002;
        SPV2_NONE = $0003;

//******************
// * CD-XA datatypes
// ******************/

        DATATYPE_CDXA_VIDEO = $0204;    //* CD-XA Video datatype*/
        DATATYPE_CDXA_DATA = $0205;     //* CD-XA sub-band data datatype*/
        DATATYPE_CDXA_AUDIO = $0206;    //* CD-XA ADPCM Audio datatype*/
        DATATYPE_CDXA_AUDIO_HD = $0207; //* CD-XA ADPCM Audio with Headers*/
        DATATYPE_CDXA_HEADERS = $0208;  //* CD-XA Headers and Subheaders*/

//* Subtypes for DATATYPE_CDXA_AUDIO:*/

        DATATYPE_LEVELB = $0000;        //* LEVEL B Audio Stereo*/
        DATATYPE_LEVELC = $0001;        //* LEVEL C Audio Stereo*/
        DATATYPE_LEVELB_MONO = $0002;   //* LEVEL B Audio Mono*/
        DATATYPE_LEVELC_MONO = $0003;   //* LEVEL C Audio Mono*/

//*********************
// * DIGISPEECH datatype
// *********************/
        DATATYPE_DIGISPEECH = $0208;    //* DIGISPEECH */
        DATATYPE_NATIVE = $0001;        //* Native digispeech */

//*********************
// * MPEG-1 Audio datatypes
// *********************/
        DATATYPE_MPEG1AUDIO = $0050;    //* MPEG-1 Audio */

//* Subtypes for DATATYPE_MPEG1AUDIO:*/
        MPEG1_FORMAT_3M16 = $0001;      //* 32.000kHz,  Mono  , 16-bit*/
        MPEG1_FORMAT_3S16 = $0002;      //* 32.000kHz,  Stereo, 16-bit*/
        MPEG1_FORMAT_4M16 = $0003;      //* 44.1/48kHz, Mono  , 16-bit*/
        MPEG1_FORMAT_4S16 = $0004;      //* 44.1/48kHz, Stereo, 16-bit*/

//*******************
// * UNKNOWN datatype
// *******************/
        DATATYPE_UNKNOWN = $FFFFFFFF;   //* Unknown datatype */

//*         SUBTYPE_NONE            0x0000L      No subtype for this data type*/


// ERROR CODES

CONST

NO_ERROR = 0;
//*****************************************/
//* MCI Device Manager Error Return codes */
//*****************************************/

CONST
MCIERR_BASE                     = 5000;
MCIERR_SUCCESS                  = 0;
MCIERR_INVALID_DEVICE_ID        =(MCIERR_BASE + 1);
MCIERR_NO_MASTER                =(MCIERR_BASE + 2);
MCIERR_UNRECOGNIZED_KEYWORD     =(MCIERR_BASE + 3);
MCIERR_MASTER_CONFLICT          =(MCIERR_BASE + 4);
MCIERR_UNRECOGNIZED_COMMAND     =(MCIERR_BASE + 5);
MCIERR_HARDWARE                 =(MCIERR_BASE + 6);
MCIERR_INVALID_DEVICE_NAME      =(MCIERR_BASE + 7);
MCIERR_OUT_OF_MEMORY            =(MCIERR_BASE + 8);
MCIERR_DEVICE_OPEN              =(MCIERR_BASE + 9);
MCIERR_CANNOT_LOAD_DRIVER       =(MCIERR_BASE + 10);
MCIERR_MISSING_COMMAND_STRING   =(MCIERR_BASE + 11);
MCIERR_PARAM_OVERFLOW           =(MCIERR_BASE + 12);
MCIERR_MISSING_STRING_ARGUMENT  =(MCIERR_BASE + 13);
MCIERR_BAD_INTEGER              =(MCIERR_BASE + 14);
MCIERR_PARSER_INTERNAL          =(MCIERR_BASE + 15);
MCIERR_DRIVER_INTERNAL          =(MCIERR_BASE + 16);
MCIERR_MISSING_PARAMETER        =(MCIERR_BASE + 17);
MCIERR_UNSUPPORTED_FUNCTION     =(MCIERR_BASE + 18);
MCIERR_FILE_NOT_FOUND           =(MCIERR_BASE + 19);
MCIERR_DEVICE_NOT_READY         =(MCIERR_BASE + 20);
MCIERR_INTERNAL                 =(MCIERR_BASE + 21);
MCIERR_DRIVER                   =(MCIERR_BASE + 22);
MCIERR_CANNOT_USE_ALL           =(MCIERR_BASE + 23);
MCIERR_MULTIPLE                 =(MCIERR_BASE + 24);
MCIERR_EXTENSION_NOT_FOUND      =(MCIERR_BASE + 25);
MCIERR_OUTOFRANGE               =(MCIERR_BASE + 26);
MCIERR_CANNOT_ADD_ALIAS         =(MCIERR_BASE + 27);
MCIERR_FLAGS_NOT_COMPATIBLE     =(MCIERR_BASE + 28);
MCIERR_CANNOT_USE_NOUNLOAD      =(MCIERR_BASE + 29);
MCIERR_FILE_NOT_SAVED           =(MCIERR_BASE + 30);
MCIERR_DEVICE_TYPE_REQUIRED     =(MCIERR_BASE + 31);
MCIERR_DEVICE_LOCKED            =(MCIERR_BASE + 32);
MCIERR_DUPLICATE_ALIAS          =(MCIERR_BASE + 33);
MCIERR_INSTANCE_INACTIVE        =(MCIERR_BASE + 34);

MCIERR_COMMAND_TABLE            =(MCIERR_BASE + 35);
MCIERR_INI_FILE_LOCKED          =(MCIERR_BASE + 37);

MCIERR_NO_AUDIO_SUPPORT         =(MCIERR_BASE + 40);
MCIERR_NOT_IN_PM_SESSION        =(MCIERR_BASE + 41);
MCIERR_DUPLICATE_KEYWORD        =(MCIERR_BASE + 42);
MCIERR_COMMAND_STRING_OVERFLOW  =(MCIERR_BASE + 43);
MCIERR_DRIVER_PROC_NOT_FOUND    =(MCIERR_BASE + 44);
MCIERR_INVALID_DEVICE_TYPE      =(MCIERR_BASE + 45);
MCIERR_INVALID_DEVICE_ORDINAL   =(MCIERR_BASE + 46);
MCIERR_HEADPHONES_NOT_SET       =(MCIERR_BASE + 47);
MCIERR_SPEAKERS_NOT_SET         =(MCIERR_BASE + 48);
MCIERR_SOUND_NOT_SET            =(MCIERR_BASE + 49);
MCIERR_INVALID_BUFFER           =(MCIERR_BASE + 50);
MCIERR_INVALID_MEDIA_TYPE       =(MCIERR_BASE + 51);
MCIERR_INVALID_CONNECTOR_INDEX  =(MCIERR_BASE + 52);
MCIERR_NO_CONNECTION            =(MCIERR_BASE + 53);
MCIERR_INVALID_FLAG             =(MCIERR_BASE + 54);
MCIERR_CANNOT_LOAD_DSP_MOD      =(MCIERR_BASE + 55);
MCIERR_ALREADY_CONNECTED        =(MCIERR_BASE + 56);
MCIERR_INVALID_CALLBACK_HANDLE  =(MCIERR_BASE + 57);
MCIERR_DRIVER_NOT_FOUND         =(MCIERR_BASE + 58);
MCIERR_DUPLICATE_DRIVER         =(MCIERR_BASE + 59);
MCIERR_INI_FILE                 =(MCIERR_BASE + 60);
MCIERR_INVALID_GROUP_ID         =(MCIERR_BASE + 61);
MCIERR_ID_ALREADY_IN_GROUP      =(MCIERR_BASE + 62);
MCIERR_MEDIA_CHANGED            =(MCIERR_BASE + 63);
MCIERR_MISSING_FLAG             =(MCIERR_BASE + 64);
MCIERR_UNSUPPORTED_FLAG         =(MCIERR_BASE + 65);
MCIERR_DRIVER_NOT_LOADED        =(MCIERR_BASE + 66);
MCIERR_INVALID_MODE             =(MCIERR_BASE + 67);
MCIERR_INVALID_ITEM_FLAG        =(MCIERR_BASE + 68);
MCIERR_INVALID_TIME_FORMAT_FLAG =(MCIERR_BASE + 69);
MCIERR_SPEED_FORMAT_FLAG        =(MCIERR_BASE + 70);
MCIERR_INVALID_AUDIO_FLAG       =(MCIERR_BASE + 71);
MCIERR_NODEFAULT_DEVICE         =(MCIERR_BASE + 72);
MCIERR_DUPLICATE_EXTENSION      =(MCIERR_BASE + 73);
MCIERR_FILE_ATTRIBUTE           =(MCIERR_BASE + 74);
MCIERR_DUPLICATE_CUEPOINT       =(MCIERR_BASE + 75);
MCIERR_INVALID_CUEPOINT         =(MCIERR_BASE + 76);
MCIERR_CUEPOINT_LIMIT_REACHED   =(MCIERR_BASE + 77);
MCIERR_MISSING_ITEM             =(MCIERR_BASE + 78);
MCIERR_MISSING_TIME_FORMAT      =(MCIERR_BASE + 79);
MCIERR_MISSING_SPEED_FORMAT     =(MCIERR_BASE + 80);
MCIERR_INVALID_CONNECTOR_TYPE   =(MCIERR_BASE + 81);
MCIERR_TARGET_DEVICE_FULL       =(MCIERR_BASE + 82);
MCIERR_UNSUPPORTED_CONN_TYPE    =(MCIERR_BASE + 83);
MCIERR_CANNOT_MODIFY_CONNECTOR  =(MCIERR_BASE + 84);
MCIERR_RECORD_ABORTED           =(MCIERR_BASE + 85);
MCIERR_GROUP_COMMAND            =(MCIERR_BASE + 86);
MCIERR_DEVICE_NOT_FOUND         =(MCIERR_BASE + 87);
MCIERR_RESOURCE_NOT_AVAILABLE   =(MCIERR_BASE + 88);
MCIERR_INVALID_IO_PROC          =(MCIERR_BASE + 89);

MCIERR_WAVE_OUTPUTSINUSE        =(MCIERR_BASE + 90);
MCIERR_WAVE_SETOUTPUTINUSE      =(MCIERR_BASE + 91);
MCIERR_WAVE_INPUTSINUSE         =(MCIERR_BASE + 92);
MCIERR_WAVE_SETINPUTINUSE       =(MCIERR_BASE + 93);
MCIERR_WAVE_OUTPUTUNSPECIFIED   =(MCIERR_BASE + 94);
MCIERR_WAVE_INPUTUNSPECIFIED    =(MCIERR_BASE + 95);
MCIERR_WAVE_OUTPUTSUNSUITABLE   =(MCIERR_BASE + 96);
MCIERR_WAVE_SETOUTPUTUNSUITABLE =(MCIERR_BASE + 97);
MCIERR_WAVE_INPUTSUNSUITABLE    =(MCIERR_BASE + 98);
MCIERR_WAVE_SETINPUTUNSUITABLE  =(MCIERR_BASE + 99);

MCIERR_SEQ_DIV_INCOMPATIBLE     =(MCIERR_BASE + 100);
MCIERR_SEQ_PORT_INUSE           =(MCIERR_BASE + 101);
MCIERR_SEQ_PORT_NONEXISTENT     =(MCIERR_BASE + 102);
MCIERR_SEQ_PORT_MAPNODEVICE     =(MCIERR_BASE + 103);
MCIERR_SEQ_PORT_MISCERROR       =(MCIERR_BASE + 104);
MCIERR_SEQ_TIMER                =(MCIERR_BASE + 105);

MCIERR_VDP_COMMANDCANCELLED     =(MCIERR_BASE + 106);
MCIERR_VDP_COMMANDFAILURE       =(MCIERR_BASE + 107);
MCIERR_VDP_NOTSPUNUP            =(MCIERR_BASE + 108);
MCIERR_VDP_NOCHAPTER            =(MCIERR_BASE + 109);
MCIERR_VDP_NOSIDE               =(MCIERR_BASE + 110);
MCIERR_VDP_NOSIZE               =(MCIERR_BASE + 111);
MCIERR_VDP_INVALID_TIMEFORMAT   =(MCIERR_BASE + 112);

MCIERR_CLIPBOARD_ERROR          =(MCIERR_BASE + 114);
MCIERR_CANNOT_CONVERT           =(MCIERR_BASE + 115);
MCIERR_CANNOT_REDO              =(MCIERR_BASE + 116);
MCIERR_CANNOT_UNDO              =(MCIERR_BASE + 117);
MCIERR_CLIPBOARD_EMPTY          =(MCIERR_BASE + 118);

MCIERR_INVALID_WORKPATH         =(MCIERR_BASE + 119);
MCIERR_INDETERMINATE_LENGTH     =(MCIERR_BASE + 120);
MCIERR_DUPLICATE_EA             =(MCIERR_BASE + 121);
MCIERR_INVALID_CONNECTION       =(MCIERR_BASE + 122);
MCIERR_CHANNEL_OFF              =(MCIERR_BASE + 123);
MCIERR_CANNOT_CHANGE_CHANNEL    =(MCIERR_BASE + 124);
MCIERR_FILE_IO                  =(MCIERR_BASE + 125);
MCIERR_SYSTEM_FILE              =(MCIERR_BASE + 126);
MCIERR_DISPLAY_RESOLUTION       =(MCIERR_BASE + 127);
MCIERR_NO_ASYNC_PLAY_ACTIVE     =(MCIERR_BASE + 128);

MCIERR_UNSUPP_FORMAT_TAG        =(MCIERR_BASE + 129);
MCIERR_UNSUPP_SAMPLESPERSEC     =(MCIERR_BASE + 130);
MCIERR_UNSUPP_BITSPERSAMPLE     =(MCIERR_BASE + 131);
MCIERR_UNSUPP_CHANNELS          =(MCIERR_BASE + 132);
MCIERR_UNSUPP_FORMAT_MODE       =(MCIERR_BASE + 133);
MCIERR_NO_DEVICE_DRIVER         =(MCIERR_BASE + 134);
MCIERR_CODEC_NOT_SUPPORTED      =(MCIERR_BASE + 135);

MCIERR_TUNER_NO_HW              =(MCIERR_BASE + 136);
MCIERR_TUNER_NO_AFC             =(MCIERR_BASE + 137);
MCIERR_TUNER_AFC_ON             =(MCIERR_BASE + 138);
MCIERR_TUNER_CHANNEL_SKIPPED    =(MCIERR_BASE + 139);
MCIERR_TUNER_CHANNEL_TOO_LOW    =(MCIERR_BASE + 140);
MCIERR_TUNER_CHANNEL_TOO_HIGH   =(MCIERR_BASE + 141);
MCIERR_AUD_CHANNEL_OUTOFRANGE   =(MCIERR_BASE + 142);
MCIERR_TUNER_INVALID_REGION     =(MCIERR_BASE + 143);
MCIERR_SIGNAL_INVALID           =(MCIERR_BASE + 144);
MCIERR_TUNER_MODE               =(MCIERR_BASE + 145);
MCIERR_TUNER_REGION_NOT_SET     =(MCIERR_BASE + 146);
MCIERR_TUNER_CHANNEL_NOT_SET    =(MCIERR_BASE + 147);
MCIERR_UNSUPP_CLASS             =(MCIERR_BASE + 148);
MCIERR_UNSUPPORTED_ATTRIBUTE    =(MCIERR_BASE + 149);

MCIERR_CUSTOM_DRIVER_BASE       =(MCIERR_BASE + 256);

//******************************************/
//* Sync/Stream Manager Error Return codes */
//******************************************/

MEBASE                          =(MCIERR_BASE + 500);
ERROR_INVALID_STREAM            =(MEBASE + 1);
ERROR_INVALID_HID               =(MEBASE + 2);
ERROR_INVALID_NETWORK           =(MEBASE + 3);
ERROR_INVALID_OBJTYPE           =(MEBASE + 4);
ERROR_INVALID_FLAG              =(MEBASE + 5);
ERROR_INVALID_EVCB              =(MEBASE + 6);
ERROR_INVALID_EVENT             =(MEBASE + 7);
ERROR_INVALID_MMTIME            =(MEBASE + 8);
ERROR_INVALID_NUMSLAVES         =(MEBASE + 9);
ERROR_INVALID_REQUEST           =(MEBASE + 10);
ERROR_INVALID_SPCBKEY           =(MEBASE + 11);
ERROR_INVALID_HNDLR_NAME        =(MEBASE + 12);
ERROR_INVALID_PROTOCOL          =(MEBASE + 13);
ERROR_INVALID_BUFFER_SIZE       =(MEBASE + 14);
ERROR_INVALID_BUFFER_RETURNED   =(MEBASE + 15);
ERROR_INVALID_ACB               =(MEBASE + 16);
ERROR_INVALID_RECORD_RETURNED   =(MEBASE + 17);
ERROR_INVALID_MESSAGE           =(MEBASE + 18);

ERROR_STREAM_NOT_OWNER          =(MEBASE + 99);
ERROR_STREAM_USED               =(MEBASE + 100);
ERROR_STREAM_CREATION           =(MEBASE + 101);
ERROR_STREAM_NOTMASTER          =(MEBASE + 102);
ERROR_STREAM_NOT_STOP           =(MEBASE + 103);
ERROR_STREAM_OPERATION          =(MEBASE + 104);
ERROR_STREAM_STOP_PENDING       =(MEBASE + 105);
ERROR_STREAM_ALREADY_STOP       =(MEBASE + 106);
ERROR_STREAM_ALREADY_PAUSE      =(MEBASE + 107);
ERROR_STREAM_NOT_STARTED        =(MEBASE + 108);
ERROR_STREAM_NOT_ACTIVE         =(MEBASE + 109);
ERROR_START_STREAM              =(MEBASE + 110);
ERROR_MASTER_USED               =(MEBASE + 111);
ERROR_SPCBKEY_MISMATCH          =(MEBASE + 112);
ERROR_INSUFF_BUFFER             =(MEBASE + 113);
ERROR_ALLOC_RESOURCES           =(MEBASE + 114);
ERROR_ACCESS_OBJECT             =(MEBASE + 115);
ERROR_HNDLR_REGISTERED          =(MEBASE + 116);
ERROR_DATA_ITEM_NOT_SPECIFIED   =(MEBASE + 117);
ERROR_INVALID_SEQUENCE          =(MEBASE + 118);
ERROR_INITIALIZATION            =(MEBASE + 119);
ERROR_READING_INI               =(MEBASE + 120);
ERROR_LOADING_HNDLR             =(MEBASE + 121);
ERROR_HNDLR_NOT_FOUND           =(MEBASE + 122);
ERROR_SPCB_NOT_FOUND            =(MEBASE + 123);
ERROR_DEVICE_NOT_FOUND          =(MEBASE + 124);
ERROR_TOO_MANY_EVENTS           =(MEBASE + 125);
ERROR_DEVICE_OVERRUN            =(MEBASE + 126);
ERROR_DEVICE_UNDERRUN           =(MEBASE + 127);
ERROR_HNDLR_NOT_IN_INI          =(MEBASE + 128);
ERROR_QUERY_STREAM_TIME         =(MEBASE + 129);
ERROR_DATA_ITEM_NOT_SEEKABLE    =(MEBASE + 130);
ERROR_NOT_SEEKABLE_BY_TIME      =(MEBASE + 131);
ERROR_NOT_SEEKABLE_BY_BYTES     =(MEBASE + 132);
ERROR_STREAM_NOT_SEEKABLE       =(MEBASE + 133);
ERROR_PLAYLIST_STACK_OVERFLOW   =(MEBASE + 135);
ERROR_PLAYLIST_STACK_UNDERFLOW  =(MEBASE + 136);
ERROR_LOCKING_BUFFER            =(MEBASE + 137);
ERROR_UNLOCKING_BUFFER          =(MEBASE + 138);
ERROR_SEEK_PAST_END             =(MEBASE + 139);
ERROR_SEEK_BACK_NOT_SUPPORTED   =(MEBASE + 140);
ERROR_INTERNAL_ERROR            =(MEBASE + 141);
ERROR_INTERNAL_CORRUPT          =(MEBASE + 142);
ERROR_INSUFF_MEM                =(MEBASE + 143);
ERROR_LARGE_SEEK_BY_TIME        =(MEBASE + 144);
ERROR_STREAM_PREROLLING         =(MEBASE + 145);
ERROR_INI_FILE                  =(MEBASE + 146);
ERROR_SEEK_BEFORE_BEGINNING     =(MEBASE + 147);
ERROR_TOO_MANY_HANDLERS         =(MEBASE + 148);
ERROR_ALLOC_HEAP                =(MEBASE + 149);
ERROR_END_OF_PLAYLIST           =(MEBASE + 150);
ERROR_TOO_MANY_STREAMS          =(MEBASE + 151);
ERROR_FILE_FORMAT_INCORRECT     =(MEBASE + 152);
ERROR_DESTROY_STREAM            =(MEBASE + 153);
ERROR_INVALID_NUMMASTERS        =(MEBASE + 154);
ERROR_MASTER_CONFLICT           =(MEBASE + 155);
ERROR_NO_MASTER                 =(MEBASE + 156);
ERROR_NO_SYNC                   =(MEBASE + 157);
ERROR_STREAM_ALREADY_IN_NETWORK =(MEBASE + 158);
ERROR_NO_STREAMS_IN_NETWORK     =(MEBASE + 159);
ERROR_MISSING_EVENT_ROUTINE     =(MEBASE + 160);
ERROR_CAN_NOT_REMOVE_STREAM     =(MEBASE + 161);

ERROR_BUFFER_NOT_AVAILABLE      =(MEBASE + 400);
ERROR_TOO_MANY_BUFFERS          =(MEBASE + 401);
ERROR_TOO_MANY_RECORDS          =(MEBASE + 402);


//*----- ERROR_INVALID_PROTOCOL ulErrorStatus defines -----*/
//*----- Refer to SHC_NEGOTIATE_RESULT api.*/
PROTOCOL_SPCBLENGTH            =1;
PROTOCOL_SPCBKEY               =2;
PROTOCOL_DATAFLAG              =3;
PROTOCOL_NUMRECORDS            =4;
PROTOCOL_BLOCKSIZE             =5;
PROTOCOL_BUFFERSIZE            =6;
PROTOCOL_MINNUMBUFFERS         =7;
PROTOCOL_MAXNUMBUFFERS         =8;
PROTOCOL_SOURCESTART           =9;
PROTOCOL_TARGETSTART           =10;
PROTOCOL_BUFFERFLAG            =11;
PROTOCOL_HANDLERFLAG           =12;
PROTOCOL_SYNCTOLERANCE         =13;
PROTOCOL_SYNCINTERVAL          =14;
PROTOCOL_INTERNALERROR         =-1;

//***********************************/
//* MMIO Manager Error Return codes */
//***********************************/

MMIOERR_BASE                   =(MEBASE + 1000);
MMIOERR_UNBUFFERED             =(MMIOERR_BASE + 1);
MMIOERR_CANNOTWRITE            =(MMIOERR_BASE + 2);
MMIOERR_CHUNKNOTFOUND          =(MMIOERR_BASE + 3);

MMIOERR_INVALID_HANDLE         =(MMIOERR_BASE + 4);
MMIOERR_INVALID_PARAMETER      =(MMIOERR_BASE + 5);
MMIOERR_INTERNAL_SYSTEM        =(MMIOERR_BASE + 6);
MMIOERR_NO_CORE                =(MMIOERR_BASE + 7);

MMIOERR_INI_OPEN               =(MMIOERR_BASE + 8);
MMIOERR_INI_READ               =(MMIOERR_BASE + 9);

MMIOERR_INVALID_BUFFER_LENGTH  =(MMIOERR_BASE + 10);
MMIOERR_NO_BUFFER_ALLOCATED    =(MMIOERR_BASE + 11);
MMIOERR_NO_FLUSH_FOR_MEM_FILE  =(MMIOERR_BASE + 12);
MMIOERR_NO_FLUSH_NEEDED        =(MMIOERR_BASE + 13);
MMIOERR_READ_ONLY_FILE         =(MMIOERR_BASE + 14);
MMIOERR_WRITE_ONLY_FILE        =(MMIOERR_BASE + 15);
MMIOERR_INSTALL_PROC_FAILED    =(MMIOERR_BASE + 16);
MMIOERR_READ_FAILED            =(MMIOERR_BASE + 17);
MMIOERR_WRITE_FAILED           =(MMIOERR_BASE + 18);
MMIOERR_SEEK_FAILED            =(MMIOERR_BASE + 19);
MMIOERR_CANNOTEXPAND           =(MMIOERR_BASE + 20);
MMIOERR_FREE_FAILED            =(MMIOERR_BASE + 21);
MMIOERR_EOF_SEEN               =(MMIOERR_BASE + 22);
MMIOERR_INVALID_ACCESS_FLAG    =(MMIOERR_BASE + 23);
MMIOERR_INVALID_STRUCTURE      =(MMIOERR_BASE + 24);
MMIOERR_INVALID_SIZE           =(MMIOERR_BASE + 25);
MMIOERR_INVALID_FILENAME       =(MMIOERR_BASE + 26);

MMIOERR_CF_DUPLICATE_SEEN      =(MMIOERR_BASE + 27);
MMIOERR_CF_ENTRY_NO_CORE       =(MMIOERR_BASE + 28);
MMIOERR_CF_WO_UNSUPPORTED      =(MMIOERR_BASE + 29);
MMIOERR_CF_ELEMENTS_OPEN       =(MMIOERR_BASE + 30);
MMIOERR_CF_NON_BND_FILE        =(MMIOERR_BASE + 31);
MMIOERR_CF_ENTRY_NOT_FOUND     =(MMIOERR_BASE + 32);

MMIOERR_DELETE_FAILED          =(MMIOERR_BASE + 33);
MMIOERR_OUTOFMEMORY            =(MMIOERR_BASE + 34);

MMIOERR_INVALID_DLLNAME        =(MMIOERR_BASE + 35);
MMIOERR_INVALID_PROCEDURENAME  =(MMIOERR_BASE + 36);
MMIOERR_MATCH_NOT_FOUND        =(MMIOERR_BASE + 37);

MMIOERR_SEEK_BEFORE_BEGINNING  =(MMIOERR_BASE + 38);
MMIOERR_INVALID_FILE           =(MMIOERR_BASE + 39);
MMIOERR_QOSUNAVAILABLE         =(MMIOERR_BASE + 40);
MMIOERR_MEDIA_NOT_FOUND        =(MMIOERR_BASE + 41);

MMIOERR_ERROR_IN_FRAME_DATA    =(MMIOERR_BASE + 42);
MMIOERR_INVALID_DIM_ALIGN      =(MMIOERR_BASE + 43);
MMIOERR_CODEC_NOT_SUPPORTED    =(MMIOERR_BASE + 44);

MMIOERR_UNSUPPORTED_FUNCTION   =(MMIOERR_BASE + 45);
MMIOERR_CLIPBRD_ERROR          =(MMIOERR_BASE + 46);
MMIOERR_CLIPBRD_ACTIVE         =(MMIOERR_BASE + 47);
MMIOERR_CLIPBRD_EMPTY          =(MMIOERR_BASE + 48);
MMIOERR_NEED_NEW_FILENAME      =(MMIOERR_BASE + 49);
MMIOERR_INVALID_TRACK_OPERATION=(MMIOERR_BASE + 50);
MMIOERR_INCOMPATIBLE_DATA      =(MMIOERR_BASE + 51);
MMIOERR_ACCESS_DENIED          =(MMIOERR_BASE + 52);
MMIOERR_MISSING_FLAG           =(MMIOERR_BASE + 53);
MMIOERR_INVALID_ITEM_FLAG      =(MMIOERR_BASE + 54);

//*************************************/
//* Real-Time MIDI Error Return Codes */
//*************************************/

MIDIERR_BASE                     =(MMIOERR_BASE + 500);

MIDIERR_DUPLICATE_INSTANCE_NAME  =(MIDIERR_BASE + 1);
MIDIERR_HARDWARE_FAILED          =(MIDIERR_BASE + 2);
MIDIERR_INTERNAL_SYSTEM          =(MIDIERR_BASE + 3);
MIDIERR_INVALID_BUFFER_LENGTH    =(MIDIERR_BASE + 4);
MIDIERR_INVALID_CLASS_NUMBER     =(MIDIERR_BASE + 5);
MIDIERR_INVALID_CONFIG_DATA      =(MIDIERR_BASE + 6);
MIDIERR_INVALID_FLAG             =(MIDIERR_BASE + 7);
MIDIERR_INVALID_INSTANCE_NAME    =(MIDIERR_BASE + 8);
MIDIERR_INVALID_INSTANCE_NUMBER  =(MIDIERR_BASE + 9);
MIDIERR_INVALID_PARAMETER        =(MIDIERR_BASE + 10);
MIDIERR_INVALID_SETUP            =(MIDIERR_BASE + 11);
MIDIERR_NO_DRIVER                =(MIDIERR_BASE + 12);
MIDIERR_NO_DEFAULT_HW_NODE       =(MIDIERR_BASE + 13);
MIDIERR_NOT_ALLOWED              =(MIDIERR_BASE + 14);
MIDIERR_NOTIFY_MISSED            =(MIDIERR_BASE + 15);
MIDIERR_RESOURCE_NOT_AVAILABLE   =(MIDIERR_BASE + 16);
MIDIERR_SENDONLY                 =(MIDIERR_BASE + 17);
MIDIERR_RECEIVEONLY              =(MIDIERR_BASE + 18);

TIMERERR_BASE                    =(MIDIERR_BASE + 100);

TIMERERR_INVALID_PARAMETER       =(TIMERERR_BASE + 1);
TIMERERR_INTERNAL_SYSTEM         =(TIMERERR_BASE + 2);


//***********************************/
//* User defined Error Return codes */
//***********************************/

USERERR_BASE                     =(MMIOERR_BASE + 1000);

Const
  FOURCC_ULTI:FOURCC=0;
  FOURCC_RT21:FOURCC=0;
  FOURCC_DIB :FOURCC=0;
  FOURCC_R565:FOURCC=0;
  FOURCC_R555:FOURCC=0;
  FOURCC_R664:FOURCC=0;
  FOURCC_RGB3:FOURCC=0;
  FOURCC_BGR3:FOURCC=0;
  FOURCC_RGB4:FOURCC=0;
  FOURCC_BGR4:FOURCC=0;
  FOURCC_LUT8:FOURCC=0;
  FOURCC_LT12:FOURCC=0;
  FOURCC_GREY:FOURCC=0;
  FOURCC_GY16:FOURCC=0;
  FOURCC_Y888:FOURCC=0;
  FOURCC_Y2X2:FOURCC=0;
  FOURCC_Y4X4:FOURCC=0;
  FOURCC_YUV9:FOURCC=0;
  FOURCC_Y644:FOURCC=0;
  FOURCC_MONO:FOURCC=0;
  FOURCC_Y422:FOURCC=0;
  FOURCC_Y42B:FOURCC=0;
  FOURCC_Y42D:FOURCC=0;
  FOURCC_Y411:FOURCC=0;
  FOURCC_VGA :FOURCC=0;

Const
  // FourCCs will be initialized in Initialization section
  FourCC_RIFF                   : FourCC = 0;
  FourCC_LIST                   : FourCC = 0;
  FourCC_MEM                    : FourCC = 0;
  FourCC_DOS                    : FourCC = 0;
  FourCC_BND                    : FourCC = 0;
  FourCC_FREE                   : FourCC = 0;
  FourCC_DEL                    : FourCC = 0;
  FourCC_CTOC                   : FourCC = 0;
  FourCC_CGRP                   : FourCC = 0;
  FourCC_CF                     : FourCC = 0;


//****************************************************************************/
//*                                                                          */
//* Module Name: AUDIO.H                                                     */
//*                                                                          */
//* OS/2 2.0 Multimedia Extensions Audio Structures and definitions.         */
//*                                                                          */
//* Copyright (c) International Business Machines Corporation 1991, 1992     */
//*                        All Rights Reserved                               */
//*                                                                          */
//* Ported to FPC: CooL/2 Aka Andy Svirgunov.                                */
//****************************************************************************/

//*-------------------------------------------------------------------------*
//* AUDIODD version level
//-------------------------------------------------------------------------*/

CONST
         CURRENT_VERSION=$01020000;

//*-------------------------------------------------------------------------*
//* Control definitions
//*-------------------------------------------------------------------------*/

        AUDIO_IOCTL_CAT   =$80;
        AUDIO_INIT        =$40;
        AUDIO_STATUS      =$41;
        AUDIO_CONTROL     =$42;
        AUDIO_BUFFER      =$43;
        AUDIO_LOAD        =$44;
        AUDIO_WAIT        =$45;
        AUDIO_HPI         =$46;
        AUDIO_CAPABILITY  =$48;

        MIX_GETCONNECTIONS=$60;
        MIX_SETCONNECTIONS=$61;
        MIX_GETLINEINFO   =$62;
        MIX_GETCONTROL    =$63;
        MIX_SETCONTROL    =$64;

//*-------------------------------------------------------------------------*
//* AUDIO_INIT declarations and defines
//*-------------------------------------------------------------------------*/

        LOAD_PATH = 260;

//* Values for AUDIO_INIT.sMode  */

        ADPCM     =1;     //* AVC type ADPCM                */
        PCM       =2;     //* Pulse Coded Modulation        */
        MU_LAW    =3;     //* mu-law                        */
        MIDI      =4;     //* MIDI data                     */
        A_LAW     =5;     //* a-law                         */
        SOURCE_MIX=6;     //* External audio source         */
        SPV2      =7;     //* Speech Viewer/2               */
        ADPCMXA   =8;     //* XA CD ROM                     */
        SPV2BCPCM =25;    //* Speech Viewer/2               */
        SPV2PCM   =26;
        SPV2NONE  =27;
        IDLE      =999;
        CLAIM_HDWR=32000; //* Serialize access to hardware  */

//* Values for AUDIO_INIT.ulFlags        */

        FIXED             =$00000001;    // Fixed length data             */
        LEFT_ALIGNED      =$00000002;    // Left align bits on byte bndry */
        RIGHT_ALIGNED     =$00000004;    // Right align bits on byte bndry*/
        TWOS_COMPLEMENT   =$00000008;    // 2's complement data           */
        SIGNED            =$00000010;    // Signed data                   */
        BIG_ENDIAN        =$00000020;    // MSB's first (motorola format) */
        RIFF_DATATYPE     =$00000040;    // sMode contains a RIFF datatype*/
        PITCH             =$00100000;    // Pitch control is supported    */
        INPUT             =$00200000;    // Input select is supported     */
        OUTPUT            =$00400000;    // Output select is supported    */
        MONITOR           =$00800000;    // Monitor is supported          */
        VOLUME            =$01000000;    // Volume control is supported   */
        VOLUME_DELAY      =$02000000;    // Volume delay is supported     */
        BALANCE           =$04000000;    // Balance control is supported  */
        BALANCE_DELAY     =$08000000;    // Balance delay is supported    */
        TREBLE            =$10000000;    // Treble control is supported   */
        BASS              =$20000000;    // Bass control supported        */
        BESTFIT_PROVIDED  =$40000000;    // bestfit returned              */
        LOAD_CODE         =$80000000;    // DSP load needed               */

//* Values for AUDIO_INIT.ulOperation    */

        OPERATION_PLAY    =1;
        OPERATION_RECORD  =2;
        PLAY_AND_RECORD   =3;
        ANALYSIS          =6;             // Speech Viewer/2               */
        DISTANCE          =7;             // Speech Viewer/2               */
        MIGRATION         =8;             // Speech Viewer/2               */

//* Values for AUDIO_INIT.sReturnCode    */

        NO_PLAY                      =1;
        NO_RECORD                    =2;
        NO_RECORD_AND_PLAY           =3;
        INVALID_REQUEST              =4;
        CONFLICT                     =5;
        OVERLOADED                   =6;
        DOWNLEVEL_DD                 =7; // DD is down level from appl.   */
        DSP_LOAD_PENDING_ON_OTHER_TRK=8; // Other trk hasn't loaded dsp   */

        AUDIO_IGNORE                 =-1;

//* Values for AUDIO_INIT.sDeviceID      */

        MINIDD                 =0;
        ACPA                   =1;
        MACPA                  =2;
        MPU401                 =3;
        SOUND_BLASTER          =4;
        IMF                    =5;
        PS1                    =6;
        PAS16                  =7;

//* AUDIO_INIT structure declaration     */

TYPE MCI_AUDIO_INIT = RECORD
        lSRate:LongInt;
        lBitsPerSRate:Longint;
        lBsize:LongInt;
        sMode:Integer;
        sChannels:Integer;
        lResolution:LongInt;
        abLoadPath:ARRAY [0..LOAD_PATH] of CHAR;
        ulFlags:LongInt;
        ulOperation:LongInt;
        sReturnCode:Integer;
        sSlotNumber:Integer;
        sDeviceID:Integer;
        pvReserved:Pointer;       //* MMPM2 uses this to pass back sysfilenum */
        ulVersionLevel:LongInt;
        end;
TYPE PMCI_AUDIO_INIT = ^MCI_AUDIO_INIT;

//*-------------------------------------------------------------------------*
//* AUDIO_DEVID declarations and defines
//*-------------------------------------------------------------------------*/

TYPE _MCI_AUDIO_DEVID = RECORD
        ulDevType:LongInt;
        ulDevNum:LongInt;
        end;

TYPE PMCI_AUDIO_DEVID = ^_MCI_AUDIO_DEVID;

//* Input devices        */

CONST

        NULL_INPUT              =0;
        STEREO_LINE_INPUT       =1;
        LEFT_LINE_INPUT         =2;
        RIGHT_LINE_INPUT        =3;
        MIC_INPUT               =4;
        BOOSTED_MIC_INPUT       =5;
        PHONE_LINE_INPUT        =6;
        HANDSET_INPUT           =7;
        SYNTH_INPUT             =8;
        DIGITAL_PHONE_LINE_INPUT=9;
        DIGITAL_HANDSET_INPUT   =10;
        MIDI_IN_PORT            =11;
        LOOPBACK                =11;
        DEFAULT_INPUT           =$FFFFFFFF;


//* Output devices       */

        NULL_OUTPUT              = 0;
        STEREO_LINE_OUTPUT       = 1;
        LEFT_LINE_OUTPUT         = 2;
        RIGHT_LINE_OUTPUT        = 3;
        SPEAKER_OUTPUT           = 4;
        HEADSET_OUTPUT           = 5;
        PHONE_LINE_OUTPUT        = 6;
        HANDSET_OUTPUT           = 7;
        SYNTH_OUTPUT             = 8;
        DIGITAL_PHONE_LINE_OUTPUT= 9;
        DIGITAL_HANDSET_OUTPUT   = 10;
        MIDI_OUT_PORT            = 11;
        DEFAULT_OUTPUT           = $FFFFFFFF;

//* Defined values for DevNum    */

        DEFAULT_DEVICE      = 0;
        DEVICE_1            = 1;
        DEVICE_2            = 2;


//*-------------------------------------------------------------------------*
//* Valid Return codes for the ulSupport field of MCI_AUDIO_CAPS
//*-------------------------------------------------------------------------*/
        SUPPORT_SUCCESS         =$00000000;
        UNSUPPORTED_RATE        =$00000001;
        UNSUPPORTED_CHANNELS    =$00000002;
        UNSUPPORTED_BPS         =$00000004;
        UNSUPPORTED_DATATYPE    =$00000008;
        UNSUPPORTED_OPERATION   =$00000010;


TYPE _MCI_AUDIO_CAPS = RECORD
    ulLength:LongInt;                 // in     Structure length                    */
    ulSamplingRate:LongInt;           // in out Sampling rate to query              */
    ulChannels:LongInt;               // in out Channels to query                   */
    ulBitsPerSample:LongInt;          // in out BPS to query                        */
    ulDataType:LongInt;               // in out RIFF Datatype to query              */
    ulOperation:LongInt;              // in out OPERATION_PLAY or OPERATION_RECORD  */
    ulSupport:LongInt;                //    out BOOLEAN-does DD support this mode   */
    ulDataSubType:LongInt;            //    out Data-subtype to use                 */
    ulResourceUnits:LongInt;          //    out Resource units this mode            */
    ulResourceClass:LongInt;          //    out Resource class for this mode        */
    ulBlockAlign:LongInt;             //    out Block alignment for this mode.      */
    fCanRecord:LongInt;               //    out Is recording possbile - this mode   */
    ulFlags:LongInt;                  //    out                                     */
    ulCapability:LongInt;             //    out Capability of the device.           */
    end;

TYPE PAUDIO_CAPS=^_MCI_AUDIO_CAPS;


//*-------------------------------------------------------------------------*
//* Valid Return codes for the ul field of MCI_AUDIO_CAPS
//*-------------------------------------------------------------------------*/

CONST

SUPPORT_MIX                 =$00000001;// supports mixer functions           */
SUPPORT_RIFF_MODES          =$00000002;// supports RIFF modes for AUDIO_INIT */
SUPPORT_CAP                 =$80000000;// Capability IOCTL supported         */


//*-------------------------------------------------------------------------*
//* AUDIO_CHANGE declarations and defines
//*-------------------------------------------------------------------------*/

//* Values for AUDIO_CHANGE.lMonitor     */

MONITOR_OFF                    =0;
MONITOR_UNCOMPRESSED           =1;
MONITOR_COMPRESSED             =2;

//* Values for AUDIO_CHANGE.lInput       */

HIGH_GAIN_MIC          = 0;
LINE_1                 = 1;
LINE_2                 = 2;
LINES_1AND2            = 3;
LOW_GAIN_MIC           = 4;
ALL_LINES              = $FFFF;
INPUTS_LISTED          = $5555;

//* Values for AUDIO_CHANGE.lOutput      */

EXTERNAL_SPEAKER        =1;
INTERNAL_SPEAKER        =2;
OUTPUT_1                =4;
OUTPUTS_LISTED          =$5555;

//* AUDIO_CHANGE structure declaration   */

TYPE _MCI_AUDIO_CHANGE = RECORD

        pvDevInfo:Pointer;
        lInput:LongInt;
        lOutput:LongInt;
        lMonitor:LongInt;
        lVolume:LongInt;
        lVolumeDelay:LongInt;
        lBalance:LongInt;
        lBalanceDelay:LongInt;
        lTreble:LongInt;
        lBass:LongInt;
        lPitch:LongInt;
        rInputList:ARRAY[0..8] OF _MCI_AUDIO_DEVID;
        rOutputList:ARRAY[0..8] OF _MCI_AUDIO_DEVID;

        prMoreInputs:PMCI_AUDIO_DEVID;
        prMoreOutputs:PMCI_AUDIO_DEVID;
        lGain:LongInt;
        pvModeInfo:Pointer;
        end;

TYPE PMCI_AUDIO_CHANGE=^_MCI_AUDIO_CHANGE;

TYPE MIDI_INFO = RECORD
      sTempo:Integer;
      sCPQN:Integer;
      sMidiSwitches:Integer;
      sReserved:ARRAY[0..5] OF Integer;
      end;

//*************************************************************************
//*  MIDI Switches
//**************************************************************************/

CONST

        MIDI_THRU_THRU =0;
        MIDI_THRU_OUT  =1;


//*-------------------------------------------------------------------------*
//* AUDIO_STATUS declarations and defines
//*-------------------------------------------------------------------------*/

//* Values for AUDIO_STATUS.ulOperation  */

        STOPPED                 =0;
        PLAYING                 =1;
        RECORDING               =2;
        PLAYING_AND_RECORDING   =3;
        UNITIALIZED             =$FFFFFFFF;

//* AUDIO_STATUS structure declaration   */

TYPE MCI_AUDIO_STATUS = RECORD
        lSRate:LongInt;
        lBitsPerSRate:LongInt;
        lBsize:LongInt;
        sMode:Integer;
        sChannels:Integer;
        ulFlags:longInt;
        ulOperation:LongInt;
        rAudioChange:_MCI_AUDIO_CHANGE;
        end;

TYPE PMCI_AUDIO_STATUS = MCI_AUDIO_STATUS;

//*-------------------------------------------------------------------------*
//* AUDIO_CONTROL declarations and defines
//*-------------------------------------------------------------------------*/

//* Values for AUDIO_CONTROL.usIOCtlRequest      */

CONST

        AUDIO_CHANGE            =0;
        AUDIO_START             =1;
        AUDIO_STOP              =2;
        AUDIO_PAUSE             =3;
        AUDIO_RESUME            =4;

//* Values for AUDIO_CONTROL.sReturnCode */

AC_UNINITED         = 1;       // Device must be init'ed or loaded first    */
FULL_QUEUE          = 2;       // Maximum # requests exceeded        */
AC_UNPAUSED         = 3;       // Resume issued, but dev not paused  */
AC_UNSTARTED        = 5;       // Device must be started first       */
INVALID_INPUT_LIST  = 7;       // invalid change.input_list entry    */
INVALID_OUTPUT_LIST = 8;       // invalid change.output_list entry   */

//* AUDIO_CONTROL structure declaration  */

TYPE MCI_AUDIO_CONTROL = RECORD
        usIOCtlRequest:Integer;
        pbRequestInfo:Pointer;
        ulPosition:LongInt;
        sReturnCode:Integer;
        end;

TYPE PMCI_AUDIO_CONTROL = MCI_AUDIO_CONTROL;

//*-------------------------------------------------------------------------*
//* AUDIO_BUFFER declarations and defines
//*-------------------------------------------------------------------------*/

//* Values for AUDIO_BUFFER.ulFlags      */

CONST

        AUDIO_UNDERRUN  =1;
        AUDIO_OVERRUN   =2;

//* Values for AUDIO_BUFFER.ulPositionType       */

        POS_MSECS       =0;
        MIDI_CLOCKS     =1;
        SMPTE_24        =24;
        SMPTE_25        =25;
        SMPTE_30DF      =29;
        SMPTE_30        =30;

TYPE MCI_AUDIO_BUFFER = RECORD
        ulFlags:LongInt;
        ulReadBufSize:LongInt;
        ulWriteBufSize:LongInt;
        ulReadBufTime:LongInt;
        ulWriteBufTime:LongInt;
        ulReadBufMax:LongInt;
        ulWriteBufMax:LongInt;
        ulPosition:LongInt;
        ulPositionType:LongInt;
        lReadBufCap:LongInt;
        lWriteBufCap:LongInt;
        lRequestBufCap:LongInt;
        end;

TYPE PMCI_AUDIO_BUFFER =^MCI_AUDIO_BUFFER;

//*-------------------------------------------------------------------------*
//* AUDIO_LOAD declarations and defines
//*-------------------------------------------------------------------------*/

//* Values for AUDIO_LOAD.ulFlags        */

CONST

        LOAD_START              =$01;
        LOAD_END                =$02;
        LOAD_32BIT              =$10;


TYPE MCI_AUDIO_LOAD = RECORD
        pbBuffer:^Char;
        ulSize:LongInt;
        ulFlags:LongInt;
        end;

TYPE PMCI_AUDIO_LOAD = MCI_AUDIO_LOAD;

//*-------------------------------------------------------------------------*
//* Track info declarations and defines
//*-------------------------------------------------------------------------*/

//* Track Info structure declaration     */

TYPE MCI_TRACK_INFO = RECORD
        usMasterVolume:Integer;
        usDitherPct:Integer; //* Percent of a bit dither during record  */
        usMasterVolumeDelay:Integer;
        usMasterBalance:Integer;
        usMasterBalanceDelay:Integer;
        end;

TYPE PMCI_TRACK_INFO = ^MCI_TRACK_INFO;


//*-------------------------------------------------------------------------*
//* IOBUFFER declarations and defines
//*-------------------------------------------------------------------------*/

CONST

        STARTED  =1;
        PAUSED   =2;

TYPE _MCI_AUDIO_IOBUFFER = RECORD
        lSize:LongInt;
        pHead:^Char;
        pTail:^Char;
        lCount:LongInt;
        ulPosition:LongInt;
        lDelay:LongInt;
        usRunFlags:Integer;
        usSelInc:Integer;
        pBuffer:^Char;
        end;

TYPE PMCI_AUDIO_IOBUFFER = ^_MCI_AUDIO_IOBUFFER;

//*-------------------------------------------------------------------------*
//* AUDIO_HPI declarations and defines
//-------------------------------------------------------------------------*/

CONST

        CBXMIT  =1;
        CBREC   =2;
        CRTIMER =4;

        EP_OPEN         =0;
        EP_CLOSE        =1;
        EP_READ         =2;
        EP_WRITE        =3;
        EP_INIT         =4;
        EP_STATUS       =5;
        EP_CONTROL      =6;
        EP_BUFFER       =7;
        EP_LOAD         =8;
        EP_WAIT         =9;



TYPE MCI_AUDIO_HPI = RECORD
        pvEntry :procedure;
        pvCallBack:procedure;
//        VOID (FAR *pvEntry)();
//        VOID (FAR *pvCallBack)();
        prXBuff:PMCI_AUDIO_IOBUFFER;
        prRBuff:PMCI_AUDIO_IOBUFFER;
        usFlags:Integer;
        end;

TYPE PMCI_AUDIO_HPI = ^MCI_AUDIO_HPI;

//**************************/
//* AUDIO_UPDATE Structure */
//**************************/
TYPE AUDIO_UPDATE = RECORD

  iobuf_type:Char;              //* 0 - XMITIO, 1 - RECIO to be updated      */
  buffer_address:^Char;     //* address to buffer to be added to array   */
  buffer_length:LongInt;          //* length of buffer to be added             */
  rc:Integer;                    //* return code                              */
  reserved:Pointer;           //* future use                               */
  end;

TYPE UPDATE = ^AUDIO_UPDATE;

//* audio_update.iobuf_type definitions                                      */

CONST

        XMIT_IOBUF =0;
        REC_IOBUF  =1;

//* audio_update.rc definitions                                              */
        MAX_NUM_BUFFERS_REACHED =9;
        UPDATE_GENERAL_FAILURE  =10;
        INVALID_BUFFER_LENGTH   =11;


//****************************************************************************/
//*                                                                          */
//* Module Name:  CDAUDIO.H                                                  */
//*                                                                          */
//* FUNCTION:  This file contains the macro definition and common record     */
//*            structures used between the CD Audio MCI Driver, its VSDs     */
//*            (Vendor Specific Drivers), and the CD look-up table,          */
//*            MMPMCD.INI.                                                   */
//*                                                                          */
//* Copyright (c) International Business Machines Corporation 1991 - 1993    */
//*                        All Rights Reserved                               */
//****************************************************************************/
//*                                                                          */
//* Ported to FPC: CooL/2 Aka Andy Svirgunov.                                */
//*                                                                          */
//****************************************************************************/

//*******************************************************************/
//* CD MCD and VSD values and inter-modual communications           */
//*******************************************************************/


//**********************************/
//* Macro Definitions              */
//**********************************/

Const

UPC_SIZE             = 7;     //* UPC code size, CD serial number */
CDMCD_CUEPOINT_MAX   = 20;     //* maximum number of cuepoints */
CDROM_SPEC_START     = 6000;     //* 2 seconds, greatest min start address */
MCI_INTERNAL_MESSAGES_START = 1000;

//*************************************************************************/
//* Internal messages between the MCI Driver and the VSD.                 */
//*************************************************************************/

MCIDRV_REGISTER_DISC    =MCI_INTERNAL_MESSAGES_START + 1;
MCIDRV_REGISTER_DRIVE   =MCI_INTERNAL_MESSAGES_START + 2;
MCIDRV_REGISTER_TRACKS  =MCI_INTERNAL_MESSAGES_START + 3;
MCIDRV_CD_READ_LONG     =MCI_INTERNAL_MESSAGES_START + 4;
MCIDRV_CD_SET_VERIFY    =MCI_INTERNAL_MESSAGES_START + 5;
MCIDRV_CD_STATUS_CVOL   =MCI_INTERNAL_MESSAGES_START + 6;


//*************************************************************************/
//* Internal callback routine from the VSD to the MCI Driver.             */
//*************************************************************************/

//typedef VOID (*PFNCDMCD) (DWORD, DWORD, DWORD);


//***********************************/
//* CDAudRegister record structures */
//***********************************/

TYPE MCI_CD_ID = Record   //* ID a disc, used to verify a disc change                 */
                //* must stay at 8 bytes to equal size of UPC               */
   Mode:byte;                           //* mode, 0=UPC, 1=ID              */
   wTrack1:word;                        //* address of track one in MMTIME */
   NumTracks:byte;                      //* number of tracks               */
   dwLeadOut:word;                      //* address of lead out track      */
   end;

TYPE MCI_CD_REGDRIVE_PARMS=RECORD //* CD-ROM Drive information, determines capabilities */

   wCaps:WORD;                          //* capabilities                   */
   dwPrerollType:WORD;                  //* preroll type                   */
   dwPrerollTime:WORD;                  //* preroll time                   */
   dwMinStartTime:WORD;                 //* minimum starting time          */
   dwCDMCDID:WORD;                      //* CD MCD ID for instance         */
//   pCDMCDReturn:PFNCDMCD;                //* addr of CD MCD Return function */
   end;

//*************************************************/
//* These flags are valid for the wCaps Field     */
//*************************************************/
CONST

CDVSD_CAP_CAN_RECORD  =$0001;   //* Can record audio               */
CDVSD_CAP_HAS_AUDIO   =$0002;   //* Can play audio                 */
CDVSD_CAP_HAS_VIDEO   =$0004;    //* Can play video                 */
CDVSD_CAP_CAN_CLSDOOR =$0008;    //* Can retract tray/close door    */
CDVSD_CAP_CAN_EJECT   =$0010;    //* Can eject disc                 */
CDVSD_CAP_CAN_STREAM  =$0020;    //* Can stream                     */
CDVSD_CAP_HAS_DAC     =$0040;    //* Can process internal           */
CDVSD_CAP_CAN_LOCK    =$0080;    //* Can disable manual eject       */
CDVSD_CAP_CAN_VOLUME  =$0100;    //* Can manipulate volume settings */
CDVSD_CAP_CAN_REVERSE =$0200;    //* Can play in reverse            */
CDVSD_CAP_CAN_V_SPEED =$0400;    //* Can vary play speed            */
CDVSD_CAP_CAN_CUE     =$0800;    //* Can read sequent. after break  */
                                        //* Used for Cue, Pause, and Seek  */


TYPE MCI_CD_REGDISC_PARMS = RECORD //* Information about the disc  */

   LowestTrackNum:BYTE;                 //* lowest track number  */
   HighestTrackNum:BYTE;                //* highest track number */
   UPC:ARRAY [0..UPC_SIZE] of BYTE;                  //* upc, 13 BCD + 4bit 0 */
   DiscID:MCI_CD_ID;                      //* Disc ID              */
   end;

TYPE MCI_CD_REGTRACK_REC=RECORD //* Information about each track */

   TrackNum:BYTE;                       //* track number               */
   dwStartAddr:WORD;                    //* starting address in MMTIME */
   dwEndAddr:WORD;                      //* ending address             */
   TrackControl:BYTE;                   //* track control information  */
   end;

TYPE MCI_CD_REGTRACKS_PARMS=RECORD //* Track information, used to verify address */
   TrackRecArr:^MCI_CD_REGTRACK_REC;     //* ptr to array of track recs  */
   dwBufSize:WORD;                      //* size of buffer              */
   end;

//********************************************/
//* Environment settings to save and restore */
//********************************************/

TYPE MCIDRV_CD_SAVE_PARMS=RECORD
   dwPosition:WORD;                      //* current position in MMTIME       */
   dwEndPlay:WORD;                       //* end play position                */
   dwMode:WORD;                          //* Play mode: playing, paused, etc. */
   dwLevel:WORD;                         //* volume levels                    */
   dwVSDData:WORD;                       //* Extra VSD data storage area      */
   end;

//********************************************/
//* Read Long (2352-Byte) Sectors            */
//********************************************/

//**************************************************************************/
//* Flag for the MCIDRV_CD_READ_LONG message                               */
//*   Default addressing mode for the dwFrom field is Hardware Red Book.   */
//*   Absolute sector addressing for ISO-9660, or High Sierra Group        */
//*   requires a flag, as does an MMTIME address.                          */
//**************************************************************************/

CONST

MCI_CD_READLONG_HSG          =$00000100;
MCI_CD_READLONG_MMTIME       =$00000200;

TYPE MCI_CD_READLONG_PARMS=RECORD
   dwFrom:WORD;                        //* read from this position   */
   wCount:WORD;                        //* Number of sectors to read */
   lpstrReturn:^CHAR;                  //* Pointer to return buffer  */
   dwRetSize:WORD;                     //* Return buffer size        */
   end;



//*******************************************************************/
//* CD Table INI file, MMPMCD.INI, values                           */
//*******************************************************************/

//*******************************************************************/
//* Capability Flags I from the CD drive look-up table, MMPMCD.INI  */
//*******************************************************************/

CONST

CDHW_CAP_CAN_EJECT    =$00000001; //* Can software eject disc         */
CDHW_CAP_CAN_LOCK     =$00000002; //* Can lock drive/disable eject    */
CDHW_CAP_READS_RAW    =$00000004; //* Can read raw sectors            */
CDHW_CAP_CAN_WRITE    =$00000008; //* Can write to disc               */
CDHW_CAP_CAN_PLAY     =$00000010; //* Can play CD-DA tracks           */
CDHW_CAP_CAN_INTERLEV =$00000020; //* Supports ISO-9660 interleaving  */
CDHW_CAP_CAN_PREFETCH =$00000080; //* Can prefetch internally         */
CDHW_CAP_MANIP_AUDIO  =$00000100; //* Can manipulte audio channels    */
CDHW_CAP_USES_REDBOOK =$00000200; //* Can use Red Book mode           */
CDHW_CAP_READS_XA     =$00000400; //* Can read CD-ROM/XA data         */
CDHW_CAP_CONT_READS   =$00000800; //* Continues to read DA after stop */
CDHW_CAP_CAN_REVERSE  =$00001000; //* Can play in reverse             */
CDHW_CAP_READS_CDDA   =$40000000; //* Can read CD-DA audio tracks     */


//*******************************************************************/
//* Capability Flags II from the CD drive look-up table, MMPMCD.INI */
//*******************************************************************/

CDHW_VOL_DEP_MASK     =$00000007; //* Volume dependency mask        */
CDHW_VOL_DEP_NONE     =        0; //* Volume without dependency     */
CDHW_VOL_DEP_HIGH     =        2; //* Volume is highest value       */
CDHW_VOL_DEP_LOW      =        3; //* Volume is lowest value        */
CDHW_VOL_DEP_LEFT     =        4; //* Volume is left volume         */
CDHW_VOL_DEP_RIGHT    =        5; //* Volume is right volume        */
CDHW_VOL_INDEP_MUTE   =$00000008; //* Mute has no dependency        */
CDHW_CAP_LOCK_MOUNT   =$00000010; //* Can only lock when mounted    */
CDHW_CAP_PLAY_VIDEO   =$00000020; //* Can play video                */
CDHW_CAP_MODAL_ONLY   =$00000040; //* Stop req to interrupt PLAY    */
CDHW_CAP_SUP_SEEK     =$00000080; //* Supports SEEK IOCTL           */
CDHW_CAP_SUP_UPC      =$00000100; //* Supports UPC IOCTL            */
CDHW_CAP_SUP_SUBCHAN  =$00000200; //* Supports sub-channel IOCTL    */
CDHW_CAP_CAN_CLS_TRAY =$00000400; //* Can close door/retract caddy  */
CDHW_CAP_NO_STREAM    =$00001000; //* Cannot stream CD-DA when PDD  */
                                         //*   says it can                 */
CDHW_CAP_VAR_PLAY     =$80000000;  //* Supports variable speeds      */

//* INI file record structure */
CDINI_APPSIZE              =   8;  //* Application name size    */
CDINI_KEYSIZE              =  16;  //* Key name size            */
CDINI_VSD_NAME_SIZE        =   8;  //* size of file name        */
VOLUME_CONTROL             = 101;  //* num of values, 0% - 100% */


TYPE MMPMCD_REC=RECORD
 usEntryVer:INTEGER;                     //* Entry version                 */
 ulCaps1:LONGINT;                        //* Capability flag I             */
 ulCaps2:LONGINT;                        //* Capability flag II            */
 VSDName:ARRAY [0..CDINI_VSD_NAME_SIZE] of CHAR;   //* DLL name for VSD              */
 ulMinStart:LONGINT;                     //* Min starting addr, 0x00MMSSFF */
 usVolCnt:INTEGER;                       //* volume level counter          */
 ausVolValues:ARRAY [0..VOLUME_CONTROL] of INTEGER;  //* volume control values         */
 end;

//*************************START OF SPECIFICATIONS **************************/
//                                                                          */
// Module NAME:  VIDEO.H                                                    */
//                                                                          */
// OS/2 2.0 Multimedia Extensions Video structures and definitions          */
//                                                                          */
// Copyright (c) International Business Machines Corporation 1993           */
//                         All Rights Reserved                              */
//***************************************************************************/
// Converted by Andry Svirgunov. Email: cool2@ngs.ru                        */
// 14.11.2002                                                               */
//                                                                          */
//************************* END OF SPECIFICATIONS ***************************/

TYPE _VIDEO_FRAME_HDR = RECORD          // vfh  */
        FrameNumber   : LongInt;        // relative frame number       */
        FramesSkipped : LongInt;        // no. frames skipped between this and last frames */
        StreamTime    : LongInt;        // stream time in milliseconds */
        FrameSize     : LongInt;        // size in bytes               */
        SHparm1       : LongInt;        // used by stream handler      */
        reserved3     : LongInt;        // unused                      */
        reserved2     : LongInt;        // unused                      */
        reserved1     : LongInt;        // unused                      */
        end;

TYPE PVIDEO_FRAME_HDR = ^_VIDEO_FRAME_HDR;

//*************************START OF SPECIFICATIONS **************************/
//                                                                          */
// COPYRIGHT:     IBM - International Business Machines                     */
//              Copyright (c) IBM Corporation  1991, 1992, 1993             */
//                        All Rights Reserved                               */
//                                                                          */
//                                                                          */
// SOURCE FILE NAME:  UMFORMAT.H                                            */
//                                                                          */
// DESCRIPTIVE NAME: Ultimotion File Format Headers (Beta)                  */
//                                                                          */
//   An Ultimotion file consists of interlevaed audio and video and text    */
//   chunks within the data list chunk.                                     */
//                                                                          */
//   A file may contain interleaved audio chunks or the soundtrack may      */
//   be contained in a seperate file specified in the header chunk, or      */
//   there may be no associated soundtrack (a silent movie).  Multiple      */
//   audio tracks are supported by this file format, either interleaved     */
//   or in seperate files.  Only one video track, track 0, is supported in  */
//   a single file.  Audio tracks are identified by a track number, with    */
//   the first audio track being track 1.                                   */
//                                                                          */
//   If a file specifies both interleaved audio tracks and external         */
//   audio tracks (.WAV files), the external audio tracks are numbered      */
//   consecutively following the interleaved audio tracks.  For             */
//   example, if a file specifies two interleaved audio tracks and two      */
//   external audio tracks, the interleaved audio tracks will be tracks     */
//   0 and 1, and the external audio tracks will be tracks 2 and 3.         */
//   Note that tracks can carry distinct audio information such as          */
//   multiple languages, and that stereo audio can be carried in a          */
//   single track.                                                          */
//                                                                          */
//************************* END OF SPECIFICATIONS ***************************/

// The hex fourcc value is also the SPCBKEY.ulDataSubType value for the */
// Ultimotion file format - OS2MEDEF.H                                  */
//***************************************************************************/
// Ultimotion File Format:                                                  */
//                                                                          */
//                                                                          */
//    Conventions used in this format:                                      */
//                                                                          */
//    - A file is a collection of chunks                                    */
//    - The first ULONG in a chunk is the chunk type (chunk ID)             */
//    - The second ULONG in a chunk is the length of the chunk,             */
//         including the header and the data, but excluding the chunk       */
//         ID and chunk length.                                             */
//    - For each interleaved audio track there is one audio frame that      */
//         corresponds to each video frame                                  */
//    - An audio frame has the same duration as its corresponding video     */
//         frame                                                            */
//    - Audio frames always follow their corresponding video frame, and     */
//         as such preceed the next video frame                             */
//                                                                          */
// Legend:                                                                  */
//     [<element name>]      optional                                       */
//     <element name>        1                                              */
//     <element name>...     1 or more                                      */
//     [<element name>]...   0 or more                                      */
//                                                                          */
//                                                                          */
//                                                                          */
//  SMV form                                                                */
//  ---------                                                               */
//                                                                          */
// <RIFF( 'ummv'                                                            */
//       <LIST( 'umhl'                             - Header LIST            */
//             <Videoheader-ck>                    - Video header chunk     */
//             <videotitle-ck>...                  - Video title chunk(s)   */
//                                                 -                        */
//             [<LIST( 'umal'                      - Audio LIST(s)          */
//                    {<extaudiofilename-ck> |     - Ext. WAV files         */
//                     <audioheader-ck>}           - Audio header           */
//                    <audiotitle-ck>...)>]...     - Audio title            */
//                                                 -                        */
//             [<LIST( 'umtl'                      - Text LIST              */
//                     <textheader-ck>...)>])>     - Text header            */
//                                                 -                        */
//       <LIST( 'umfd'                             - Frame data LIST        */
//             <LIST( 'umcd' {                     - Chapter data LIST      */
//                   <videoframe-ck>               - Video frame            */
//                   [<audioframe-ck>]             - Audio frame            */
//                   [<textframe-ck>]}...)>...)    - Text frame             */
//                                                 -                        */
//        [<LIST( 'umci'                           - Chap index table LIST  */
//               <LIST( 'umce'                     - Chap index entry LIST  */
//                     <chapterheader-ck>          - Chapter index hdr      */
//                     <chapttitle-ck>...          - Chapter title          */
//                     <frameindex-ck>)>...)>])>   - frame index            */
//                                                 -  table                 */
//                                                                          */
//***************************************************************************/
CONST

 UMAUDIOFILENAMELENGTH   =60;
 SIZEOF_FORM             =4;

// ULTIMOTION FORMS */
        UMFORM_MOVIE                  = 'ummv';
        UMFORM_HEADERLIST             = 'umhl';
        UMFORM_AUDIOHDRLIST           = 'umal';
        UMFORM_TEXTHDRLIST            = 'umtl';
        UMFORM_FRAMEDATALIST          = 'umfd';
        UMFORM_CHAPTERDATALIST        = 'umcd';
        UMFORM_CHAPTERTABLELIST       = 'umci';
        UMFORM_CHAPTERENTRYLIST       = 'umce';

// byte swapped hex defines for ulong assignments... */
        HEX_UMFORM_MOVIE               =$766d6d75;     // vmmu */
        HEX_UMFORM_HEADERLIST          =$6c686d75;     // lhmu */
        HEX_UMFORM_AUDIOHDRLIST        =$6c616d75;     // lamu */
        HEX_UMFORM_TEXTHDRLIST         =$6c746d75;     // ltmu */
        HEX_UMFORM_FRAMEDATALIST       =$64666d75;     // dfmu */
        HEX_UMFORM_CHAPTERDATALIST     =$64636d75;     // dcmu */
        HEX_UMFORM_CHAPTERTABLELIST    =$69636d75;     // icmu */
        HEX_UMFORM_CHAPTERENTRYLIST    =$65636d75;     // ecmu */

// ULTIMOTION CHUNK IDS */
        UMID_VIDEOHEADER               ='umvh';
        UMID_EXTAUDIONAME              ='umea';
        UMID_AUDIOHEADER               ='umah';
        UMID_TEXTHEADER                ='umth';
        UMID_VIDEOTITLE                ='umvt';
        UMID_AUDIOTITLE                ='umat';
        UMID_TEXTFRAME                 ='umtf';
        UMID_VIDEOFRAME                ='umvf';
        UMID_AUDIOFRAME                ='umaf';
        UMID_CHAPTERHEADER             ='umch';
        UMID_CHAPTERTITLE              ='umct';
        UMID_FRAMETABLE                ='umfi';

// byte swapped hex defines for ulong assignments... */
        HEX_UMID_VIDEOHEADER           =$68766d75;      // hvmu */
        HEX_UMID_EXTAUDIONAME          =$61656d75;      // aemu */
        HEX_UMID_AUDIOHEADER           =$68616d75;      // hamu */
        HEX_UMID_TEXTHEADER            =$68746d75;      // htmu */
        HEX_UMID_VIDEOTITLE            =$74766d75;      // tvmu */
        HEX_UMID_AUDIOTITLE            =$74616d75;      // tamu */
        HEX_UMID_TEXTFRAME             =$66746d75;      // ftmu */
        HEX_UMID_VIDEOFRAME            =$66766d75;      // fvmu */
        HEX_UMID_AUDIOFRAME            =$66616d75;      // famu */
        HEX_UMID_CHAPTERHEADER         =$68636d75;      // hcmu */
        HEX_UMID_CHAPTERTITLE          =$74636d75;      // tcmu */
        HEX_UMID_FRAMETABLE            =$69666d75;      // ifmu */



//***************************************************************************/
// CHUNK HEADER STRUCTURE - APPEARS AT START OF EACH CHUNK                  */
//***************************************************************************/
type UMCHUNKHEADER =record  // umch */
   ulChunkID:LongInt;              // ID for this chunk                   */
   ulChunkLength:LongInt;          // Length of chunk that follows        */
   end;
 PUMCHUNKHEADER=^UmChunkHeader;   // pumch */


//***************************************************************************/
// VIDEO HEADER CHUNK -                                                     */
//                                                                          */
// The UMVIDEOHEADER structure is followed by a list of                     */
// usInterleavedTracks null-terminated external audio (.WAV) file           */
// names.  The audio file name fields are fixed size to better enable       */
// changing the file names without re-writing the file.                     */
//***************************************************************************/
   UMVIDEOHEADER = record   // umvh */
   ulTotalDuration:LongInt;        // Video duration in MMTIME            */
   ulMaxFrameSize:LongInt;         // Max video frame size in bytes       */
   ulMaxAvgDataRate:LongInt;       // Max avergage data rate              */
   ulMaxBurstInterval:LongInt;     // Max interval for max data rate      */
   ulCompressionRatioX100:LongInt; // Compression ratio                   */
   ulPosterFrameOffset:LongInt;    // Poster for video (from beginning)   */
   usMaxFrameX:Word;            // Max horizontal frame size           */
   usMaxFrameY:Word;            // Max vertical frame size             */
   usNomFrameDuration:Word;     // Nominal recorded frame duration     */
   usNomIFrameInterval:word;    // I-frame every this many frames      */
   usNumCompressionTypes:Word;  // number of different CODECS used in file */
   aulCompressionTypes:Array [0..20] of LongInt;// List of CODECs found in this file */
   Reserved:Array [0..16] of LongInt;           // Reserved space */
   end;
   PUMVIDEOHEADER=^UMVIDEOHEADER;  // pumvh */


//***************************************************************************/
// EXTERNAL AUDIO FILE NAME CHUNK                                           */
//***************************************************************************/
   EXTAUDIONAME = record    // umea */
     usTrackNumber:Word;          // Audio track number for this format  */
     szFileName:Array [0..UMAUDIOFILENAMELENGTH] of Char;
   end;
  PEXTAUDIONAME=^EXTAUDIONAME;  // pumea */


//***************************************************************************/
// INTERLEAVED AUDIO TRACK HEADER CHUNK                                     */
//***************************************************************************/
   AUDIOHEADER = record   // umah */
           usTrackNumber:Word;          // Audio track number for this format  */
           usFormatTag:Word;            // Type of wave format                 */
           usChannels:Word;             // Number of channels (1=mono 2=stereo)*/
           ulSamplesPerSec:LongInt;        // Sampling rate                       */
           ulAvgBytesPerSec:LongInt;       // Avg bytes per sec                   */
           usBlockAlign:Word;           // Block alignment in bytes            */
           usBitsPerSample:Word;        // Bits per sample                     */
           usCountryCode:Word;          // Country code for this title         */
   end;
   PAUDIOHEADER=^AUDIOHEADER; // pumah */


//***************************************************************************/
// INTERLEAVED TEXT TRACK HEADER CHUNK                                      */
//***************************************************************************/
   TEXTHEADER = record   // umth */
           usTrackNumber:Word;          // Audio track number for this format  */
           usCountryCode:Word;          // Country code for this title         */
   end;
   PTEXTHEADER=^TEXTHEADER; // pumth */


//***************************************************************************/
// TITLE CHUNK                                                              */
//***************************************************************************/
   TITLE =record  // ttl */
           usCountryCode:Word;         // Country code for this title         */
           szTitle:Array [0..1] of Char;            // Video title null-terminated         */
   end;
   PTITLE=^TITLE; // pttl */


//***************************************************************************/
// AUDIO FRAME CHUNK                                                        */
//***************************************************************************/
   AUDIOFRAME =record // umaf */
          usTrackNumber:Word;         // audio track number                  */
          bData:Array [0..1] of Byte;
   end;
   PAUDIOFRAME=^AUDIOFRAME; // pumaf */


//***************************************************************************/
// TEXT FRAME CHUNK                                                         */
//***************************************************************************/
   TEXTFRAME =record  // umtf */
           usTrackNumber:Word;         // Text track number                   */
           szText:Array [0..1] of Char;             // Text null-terminated                */
   end;


//***************************************************************************/
// VIDEO FRAME CHUNK                                                        */
//***************************************************************************/
   VIDEOFRAME = record // umvf */
          usTrackNumber:Word;         // Video track number  (0L only for now) */
          ulDuration:LongInt;            // Frame duration in MMTIME            */
          ulFrameFlags:LongInt;          // Frame flags                         */
          ulCompressionType:LongInt;     // Compression type                    */
          bData:Array [0..1] of Byte;
   end;
  PVIDEOFRAME =^VIDEOFRAME; // pumvf */

// ulCompressionType defines: */
CONST

UM_VIDEO_COMPRESSION_TYPE_RAWRGB565        =1;
UM_VIDEO_COMPRESSION_TYPE_RAWUVY556        =2;
UM_VIDEO_COMPRESSION_TYPE_RAWYUV411        =3;
UM_VIDEO_COMPRESSION_TYPE_BH146           =10;   // BETA-RGB16 */
UM_VIDEO_COMPRESSION_TYPE_BH211SCS4       =11;   // 1.1 YUV16 subsampled chroma sub4 */
UM_VIDEO_COMPRESSION_TYPE_BH211UCS4       =12;   // unique chroma sub4 */

// ulVideoFrameFlags defines: */
  UM_VIDEO_FRAME_FLAG_DELTAFRAME   =$1;   // 1: delta frame, 0: I frame */
  UM_VIDEO_FRAME_FLAG_SCENECHANGE  =$2;


//***************************************************************************/
// CHAPTER INDEX HEADER (header for each entry in chapter index LIST        */
//***************************************************************************/
 TYPE UMCHAPTERINDEX = record  // umch */
       ulChapterOffset:LongInt;            // Offset from beginning of file       */
       ulChapterDuration:LongInt;          // Duration of chapter in MMTIME       */
       ulPosterFrameOffset:LongInt;        // Poster for chapter (offset from chapter)*/
  end;
  PUMCHAPTERINDEX =^UMCHAPTERINDEX;  // pumch */


//***************************************************************************/
// FRAME INDEX ENTRY                                                        */
//***************************************************************************/
   UMFRAMEINDEX = record  // umfi */
           ulFrameOffsets:LongInt;         // Offset from beginning of chapter    */
           ulFrameFlags:LongInt;           // Frame flags (Refer to frame header) */
   end;
    PUMFRAMEINDEX =^UMFRAMEINDEX;  // pumfi */


//***************************************************************************/
//                                                                          */
// Module Name: SPCB.H                                                      */
//                                                                          */
// OS/2 2.0 Multimedia Extensions Sync/Stream Manager Stream Protocol       */
// Control Block Definitions.                                               */
//                                                                          */
// Copyright (c) International Business Machines Corporation 1991, 1992     */
//                        All Rights Reserved                               */
//--------------------------------------------------------------------------*/
// Converted by Andry Svirgunov. Email: cool2@ngs.ru                        */
// 14.11.2002                                                               */
//                                                                          */
//***************************************************************************/

//*********************************************
// *
// * SPCBKEY - Stream Protocol Key
// *
// **********************************************/
//TYPE
//        MMTIME = Cardinal;

  spcbKey = record
    ulDataType    : LongInt;
    ulDataSubType : LongInt;
    ulIntKey      : LongInt;              // Generated internal key
  end;
//  pspcbKey = ^spcbKey;

//*********************************************
// *
// * SPCB - Stream Protocol Control Block
// *
// **********************************************/
  _spcb = record
   ulSPCBLen       : LongInt;       // SPCB struture length
   spcbkey         : SPCBKEY;
   ulDataFlags     : LongInt;       // Data type flags
   ulNumRec        : LongInt;       // Max # records/buffer (Only used for Split streams)
   ulBlockSize     : LongInt;       // Block alignment in bytes.
   ulBufSize       : LongInt;       // Buffer size (SSM allocated buffers) must be
                                  //  a multiple of Block size
   ulMinBuf        : LongInt;       // Minimum number of buffers needed to stream
   ulMaxBuf        : LongInt;       // Maximum number of buffers needed to stream
   ulSrcStart      : LongInt;       // # of EMPTY buffers required to start Source
   ulTgtStart      : LongInt;       // # of FULL buffers required to start Target
   ulBufFlags      : LongInt;       // Handler Protocol negotiation flags
   ulHandFlags     : LongInt;       // Handler Protocol flags
   mmtimeTolerance : mmTime;      // Sync tolerance value...Used as check by SSM
                                  //  to determine whether to send a sync pulse
                                  //  to this specific slave Stream Handler.
                                  //  Algorithm:
                                  //   diff = abs(master time - slave time)
                                  //   if  diff >= tolerance
                                  //     then send sync pulse
                                  // (Valid only for Slave stream handlers)
   mmtimeSync      : mmTime;      // Used to save sync pulse generation granularity
                                  //  if the master uses the Stream Manager Timer.
                                  //  if SpiSetSync passed NULL value for sync gran
                                  //   then use this default value. (since hardware
                                  //   must have it's own interrupt time interval.
                                  // (Valid only for Master stream handlers)
   ulBytesPerUnit  : LongInt;       // Bytes/unit of time. This is used to do seeks
                                  // on linear data that is not compressed or
                                  // of variable length.
   mmtimePerUnit   : mmTime;      // The amount of mmtime each unit represents.
                                  // A unit can be a second, minute or whatever.
  end;
  pspcb = ^_spcb;

const
  spcb_max_buf_Size        = 1024*1024;  // Largest buffer size in bytes


//******************
// * SPCB_ulDataFlags:
// *******************/

  // SPCB_ulDataFlags:
  spcbData_CueTime         = $0002; // This data type can support time cue points
  spcbData_CueData         = $0004; // This data type can support data cue points

  spcbData_Seek            = $0000; // Seeks can be performed on this data type.
  spcbData_NoSeek          = $0008; // NO seeks can be performed on this data type.
  spcbData_YieldTime       = $0010; // The ulBytes per unit field is used for a millisecond value
                                    // It represents the amount of yield time between reads of
                                    // each buffer. If this bit is set, the value of ulBytesPerUnit
                                    // is used as input to DosSleep to yield for that period of time
  ValidDataFlags           = spcbData_CueTime OR spcbData_CueData OR
                             spcbData_YieldTime OR spcbData_NoSeek;


//*******************
//* SPCB_ulBufFlags:
//********************/

  // SPCB_ulBufFlags:
  spcbBuf_USERPROVIDED     = $0001; // User provides buffers for stream
                                    // SSMgr will not allocate buffers,
                                    // but must lock down provided buffers,
                                    // so this will affect performance!!
                                    // (Source Handler only)
  spcbBuf_FIXEDBUF         = $0002; // Buffer size is fixed for this data type
                                    //  in this handler. Can not be used
                                    //  with the spcbBuf_USERPROVIDED flags.
  spcbBuf_NONCONTIGUOUS    = $0004; // Buffer do not need to be contiguous
                                    //  in physical memory.
  spcbBuf_INTERLEAVED      = $0008; // Stream buffers can be interleaved.
                                    //  Can not be use with the
                                    //  spcbBuf_USERPROVIDED flag.
                                    //  (Source Handler only)
  spcbBuf_MAXSIZE          = $0010; // ulBufSize is the maximum size that
                                    //  can be used by this stream handler
  spcbBuf_16MEG            = $0020; // The Stream buffers may be allocated
                                    //  above the 16 Meg line. This is used
                                    //  by stream handlers that can support
                                    //  greater than 16 Megabyte addresses.
  spcbBuf_FIXEDBLOCK       = $0040; // Use the ulBlockSize field to represent
                                    // the size of the IO reads that should
                                    // be performed (Interleaved streams)
  ValidBufFlags            = spcbBuf_UserProvided OR spcbBuf_FixedBuf OR
                             spcbBuf_NonContiguous OR spcbBuf_Interleaved OR
                             spcbBuf_16Meg OR spcbBuf_FixedBlock OR
                             spcbBuf_MaxSize;

//******************
// * SPCB_ulHandFlags:
// *******************/
  // SPCB_ulHandFlags:
  spcbHand_genSync         = $0001; // (INPUT FROM HANDLER ONLY)
                                    // This handler can generate sync pulses
                                    // if it is the master.
  spcbHand_RcvSync         = $0002; // (INPUT FROM HANDLER ONLY)
                                    // This handler can receive sync pulses
                                    // if it is a slave
  spcbHand_Timer           = $0004; // Use Stream Manager Timer for Sync
  spcbHand_NonStream       = $0008; // Stream Handler is non-streaming
  spcbHand_GenTime         = $0010; // This handler contains real
                                    // stream time. The handler that
                                    // supports the SpiGetTime, data/time
                                    // cue point calls for this stream.
  spcbHand_NoPreRoll       = $0020; // This stream can not be prerolled.
                                    // (i.e. recording streams)
  spcbHand_NoSync          = $0040; // This stream can be group into a
                                    // sync group, but does not send or
                                    // receive sync pulses.
  spcbHand_Phys_Seek       = $0080; // This handler does a seek to a physical
                                    // device or memory not just a time adjustment.
                                    // (ssm will always call this handler first on
                                    //  an SpiSeekStream call).

  ValidHandFlags           = spcbHand_GenSync OR spcbHand_RcvSync OR
                             spcbHand_Timer OR spcbHand_NonStream OR
                             spcbHand_GenTime OR spcbHand_NoPreRoll OR
                             spcbHand_NoSync OR spcbHand_Phys_Seek;

//**************************************************************************\
//
// Module Name: SHDD.H
//
// OS/2 2.0 Multimedia Extensions Stream Handler Device Driver Interfaces
// Block defintions.
//
// Copyright (c) International Business Machines Corporation 1990,1991
//                         All Rights Reserved
// -------------------------
// Ported by Andry Svirgunov
//
//**************************************************************************/

//***************************************************************************/
//                         D E F I N E S                                    */
//***************************************************************************/

// Multimedia Extensions Stream Handler Device Driver Interfaces
// Block defintions.

type
  shdfn   = pointer;
  ddcmdfn = pointer;
  hstream = Longint;
  hevent  = LongInt;
  hID     = LongInt;
//  MMTIME  = Cardinal;
type
  time_evcb = record
    ulType         : LongInt;            // Event_CUE_TIME
    ulSubType      : LongInt;            // Not used
    ulFlags        : LongInt;            // Single/Recurring(input/output)
    hstream        : HSTREAM;          // handle to stream for this event (input/output)
    hid            : hID;              // handler Id (input/output)
    ulStatus       : LongInt;            // Event status/error return code (output)
    mmtimeStream   : MMTIME;           // Stream time (input/output)
    unused1        : LongInt;            // 0 (input)
    unused2        : LongInt;            // 0 (input)
  end;
  ptime_evcb = ^time_evcb;
(*type
  spcbKey = record
    ulDataType    : LongInt;
    ulDataSubType : LongInt;
    ulIntKey      : LongInt;              // Generated internal key
  end;*)
//  pspcbKey = ^spcbKey;

//***************************************************************************/
//                       S T R U C T U R E S                                */
//***************************************************************************/


//***************************************************************************/
//***   DDCMD Interface                                                    **/
//***************************************************************************/

const
  // DDCMD Defines
  DDCMD_SETUP                   = 0;
  DDCMD_READ                    = 1;
  DDCMD_WRITE                   = 2;
  DDCMD_STATUS                  = 3;
  DDCMD_CONTROL                 = 4;
  DDCMD_REG_STREAM              = 5;
  DDCMD_DEREG_STREAM            = 6;

type
  ddCmdCommon = record
    ulFunction        : LongInt;                 // Function requested by SH
    Stream            : hStream;               // data stream instance
  end;
  pddCmdCommon = ^ddCmdCommon;

  ddCmd_Setup_parm = record
    ulFunction        : LongInt;                 // Function requested by SH
    hStream           : HSTREAM;
    pSetupParm        : Pointer;               // see SETUP_PARM struct
    ulSetupParmSize   : LongInt;                 // see SETUP_PARM struct
  end;
  pddCmdSetup = ^ddCmd_Setup_parm;


                //*********************************************/
                // SETUP_PARM structure                       */
                //*********************************************/
  // SETUP_PARM structure
  Setup_Parm = record                          //  DDCMDSETUP parameter block
    ulStreamTime      : LongInt;                 // stream time in milliseconds
    ulFlags           : LongInt;                 // various flags (input/output)
    // NOTE: new fields will be added here
  end;
  pSetup_Parm = ^Setup_Parm;

                //*********************************************/
                // ulFlag defines                             */
                //*********************************************/
const
  // ulFlag defines
  SETUP_RECURRING_EVENTS        = $00000001;
                // the device driver sets this flag on return from the
                // DDCMD_SETUP command if the device driver assumes events
                // are recurring events---in this case, the stream handler
                // will not have to re-enable a recurring event each time
                // the event occurs by sending a DDCMD_CONTROL command to
                // the device driver.  this is useful when CUE_TIME or
                // DATA_CUE events are expected to be used as RECURRING

type
  ccCmdReadWrite = record
    ulFunction        : LongInt;                 // Function requested by SH
    hStream           : HSTREAM;
    pBuffer           : Pointer;
    ulBufferSize      : LongInt;
    pProcessLin       : Pointer;
    fEOS              : Boolean;
    ulParm1           : LongInt;
    ulParm2           : LongInt;
    ulLength          : LongInt;
  end;
  pccCmdReadWrite = ^ccCmdReadWrite;
  ppccCmdReadWrite = ^pccCmdReadWrite;

  ccCmdStatus = record
    ulFunction        : LongInt;                 // Function requested by SH
    hStream           : HSTREAM;
    pStatus           : Pointer;               // (output)ptr to current position time
    ulStatusSize      : LongInt;                 // (output)size of position time
  end;
  pccCmdStatus = ^ccCmdStatus;

                //*********************************************/
                // STATUS_PARM structure                      */
                //*********************************************/
  // STATUS_PARM structure
  Status_Parm = record                         // DDCMDSTATUS parameter block
    ulTime            : LongInt;                 // current position time in milliseconds
    // NOTE: new fields will be added here
  end;
  pStatus_Parm = ^Status_Parm;

  ddCmdControl = record
    ulFunction        : LongInt;                 // Function requested by SH
    hStream           : HSTREAM;
    hEvent            : hEvent;                // used for EVENTS only
    ulCmd             : LongInt;
    pParm             : Pointer;               // see CONTROL_PARM structure
    ulParmSize        : LongInt;                 // see CONTROL_PARM structure
  end;
  pddCmdControl = ^ddCmdControl;
                //*********************************************/
                // ulCmd defines                              */
                //*********************************************/
const
  // ulCmd defines
  DDCMD_START                   = 1;           // start device
  DDCMD_STOP                    = 2;           // stop device and return current position in pParm
  DDCMD_PAUSE                   = 3;           // pause device and return current position in pParm
  DDCMD_RESUME                  = 4;           // resume device
  DDCMD_ENABLE_EVENT            = 5;           // ask PDD to create this event
  DDCMD_DISABLE_EVENT           = 6;           // ask PDD to purge this event
  DDCMD_PAUSE_TIME              = 7;           // pause time keeping, but not the stream
  DDCMD_RESUME_TIME             = 8;           // resume time keeping.

                //*********************************************/
                // CONTROL_PARM structure                     */
                //*********************************************/
type
  // CONTROL_PARM structure
  Control_Parm = record                        // DDCMDCONTROL parameter block
    ulTime            : LongInt;                 // time in milliseconds
                                               // SH sets cuetime when ulCmd is ENABLE_EVENT
                                               // PDD returns current time for STOP, PAUSE
    evcb              : TIME_EVCB;
  end;
  pControl_Parm = ^Control_Parm;

                //******************************************************************************/
                // This structure is filled in by the amp mixer based on the instance          */
                // data.                                                                       */
                //******************************************************************************/
TYPE AUDIOMODE = record    // audio mode fields for DDCMDRegister */
        lSRate:LongInt;            // sample rate              */
        lBitsPerSRate:LongInt;     // bits per sample          */
        sChannels:Integer;        // Channels                 */
        end;
TYPE PAUDIOMODE=^AUDIOMODE;


TYPE  ddCmdRegister = record
    ulFunction        : LongInt;                 // Function requested by SH
    hStream           : HSTREAM;               // Stream handle needed @ interrupt time
    ulSysFileNum      : LongInt;                 // Device Handle so pdd can map device instance to hstream
    pSHDEntryPoint    : SHDFN;                 // Stream handler entry point
    ulStreamOperation : LongInt;                 // SH input Record or play
    spcbkey           : SPCBKEY;
    ulBufSize         : LongInt;                 // PDD output (optional) buffer size in bytes for SPCB
    ulNumBufs         : LongInt;                 // PDD output (optional) # of buffers for SPCB
    ulAddressType     : LongInt;                 // PDD output (required) addr ptr type to data buffer
    ulBytesPerUnit    : LongInt;                 // PDD output (required)
    mmtimePerUnit     : MMTIME;                // PDD output (required)
    AudioMode         : AUDIOMODE;                 // SH input Device Control Block
    hid               : HID;                   // SH input stream handler id
  end;
  pddCmdRegister = ^ddCmdRegister;

                //****************************/
                // ulStreamOperation  defines*/
                //****************************/
const
  // ulStreamOperation  defines
  STREAM_OPERATION_MASK         = $C000;
  STREAM_OPERATION_CONSUME      = $8000;
  STREAM_OPERATION_PRODUCE      = $C000;

                //******************************************************************************/
                // ulAddressType  defines                                                      */
                // The PDD will tell the SH what type of address ptr it expects the data buffer*/
                // to be.  The SH will then request this address type to the SSM, so that the  */
                // SSM will send the correct type of each buffer request.                      */
                //******************************************************************************/
  ADDRESS_TYPE_VIRTUAL          = 0;
  ADDRESS_TYPE_PHYSICAL         = 1;              //default
  ADDRESS_TYPE_LINEAR           = 2;

type
  ddCmdDeregister = record
    ulFunction        : LongInt;                 // Function requested by SH
    Stream            : hStream;               // Stream handle needed @ interrupt time
  end;
  pddCmdDeregister = ^ddCmdDeregister;


//***************************************************************************/
//*** RING 0 Stream Handler SHD Interfaces                                ***/
//***************************************************************************/

const
  // RING 0 Stream Handler  SHD's:
  SHD_REPORT_INT                = 0;           // PDD reports interrupt has arrived
  SHD_REPORT_EVENT              = 1;           // PDD reports cuetime has arrived

type
  shd_Common = record
    ulFunction        : LongInt;                 // Function requested by PDD
    Stream            : hStream;               // Stream handle needed @ interrupt time
  end;
  pshd_Common = ^shd_Common;

type shd_ReportInt = record
    ulFunction        : LongInt;                 // Function requested by PDD
    hStream           : HSTREAM;               // so SH knows which stream to process
    pBuffer           : Pointer;               // return ptr to last used buffer
    ulFlag            : LongInt;                 // reason for interrupt
    ulStatus          : LongInt;                 // rc or bytes read/written
    ulStreamTime      : LongInt;                 // time in milliseconds of stream position
  end;
  pshd_ReportInt = ^shd_ReportInt;

                //******************/
                // ulFlag settings */
                //******************/
const
  // ulFlag settings
  ERROR                         = $80;
  STREAM_STOP_NOW               = $40;
  SHD_READ_COMPLETE             = 1;
  SHD_WRITE_COMPLETE            = 2;

type
  shd_ReportEvent = record
    ulFunction        : LongInt;                 // Function requested by PDD
    hStream           : HSTREAM;               // so SH knows which stream to process
    hEvent            : hEvent;                // event handle pass back to stream handler
    ulStreamTime      : LongInt;                 // time in milliseconds of stream position
  end;
  pshd_ReportEvent = ^shd_ReportEvent;



//***************************************************************************/
//*** RING 3 Stream Handler SHD Interfaces                                ***/
//***************************************************************************/

//*** RING 3 Stream Handler SHD Interfaces                                **

const
  // RING 3 Stream Handler  SHD's: (Uses SpiSendMsg as interface)
  SHC_REPORT_INT                = $80000000;   // ulMsgType for SHD_REPORT_INT
  SHC_REPORT_EVENT              = $80000001;   // ulMsgType for SHD_REPORT_EVENT


//*********************************************
// *
// * MSG_REPORTINT Control Block.
// *  pMsg field of SpiSendMsg api
// *
// **********************************************/
type
  // MSG_REPORTINT Control Block.
  // pMsg field of SpiSendMsg api
  msg_ReportInt = record
    ulMsgLen          : LongInt;                 // Length of structure
    pBuffer           : Pointer;               // return ptr to last used buffer
    ulFlag            : LongInt;                 // reason for interrupt
    ulStatus          : LongInt;                 // rc or bytes read/written
    ulStreamTime      : LongInt;                 // time in milliseconds of stream position
  end;
  pmsg_ReportInt = ^msg_ReportInt;

   // For ulFlag defines, refer to the _shd_reportint_parm structure */



//*********************************************
// *
// * MSG_REPORTEVENT Control Block.
// *  pMsg field of SpiSendMsg api
// *
// **********************************************/
type
  // MSG_REPORTEVENT Control Block.
  //  pMsg field of SpiSendMsg api
  msg_ReportEvent = record
    ulMsgLen          : LongInt;                 // Length of structure
    hevent            : hEvent;                // event handle pass back to stream handler
    ulStreamTime      : LongInt;                 // time in milliseconds of stream position
  end;
type   pmsg_ReportEvent = ^msg_ReportEvent;

//***************************************************************************/
//              F U N C T I O N   P R O T O T Y P E S                       */
//***************************************************************************/

//RC  FAR         SHDEntryPoint(PSHD_COMMON pCommon);
//RC  FAR         DDCMDEntryPoint(PDDCMDCOMMON pCommon);

//***************************************************************************\
//*                                                                           *
//*    Multimedia Mixer Software Development Kit Include File                 *
//*                                                                           *
//*                                                                           *
//***************************************************************************/
// Converted by Andry Svirgunov. Email: cool2@ngs.ru                        */
// 14.11.2002                                                               */
//                                                                          */
//***************************************************************************/

        TYPE HMIXER = longint;
        TYPE PHMIXER = ^HMIXER;


TYPE _MIXERLINEINFO = RECORD
        ulLength : LongInt;             //length of the struct */
        ulNumChannels : LongInt;        // number of channels on input */
        ulSupport : LongInt;            // supported functionality (MIXLINEINFO) */
        ulConnectionsPossible : LongInt;// lines connectable to */
        ulLine : LongInt;               // which line to operate on */
        end;

TYPE PMIXERLINEINFO = ^_MIXERLINEINFO;

TYPE _LINECONNECTIONS = RECORD
        ulLength : Longint;             // length of the struct */
        ulConnection : LongInt;
        ulLine : Longint;
        ulFlags : LongInt;
        end;

TYPE PLINECONNECTIONS = ^_LINECONNECTIONS;

// flags for MIXERCONTROL ulFlags field */
CONST
        MIX_MASTER = $000000001;        // indicates that this is master effect for the control */

TYPE _MIXERCONTROL = RECORD
        ulLength : LongInt;             // length of the struct */
        ulLine : LongInt;
        ulControl : LongInt;
        ulSettings : LongInt;
        ulFlags : LongInt;
        end;

TYPE PMIXERCONTROL = ^_MIXERCONTROL;
// -- need masteraudio command for the following. */


// flags for MIXERLINEINFO ulSupport field */
CONST
        MIX_BALANCE       = $00100000;  // separate balance volume control */
        MIX_ALC           = $00000004;  // supports Auto Level Control */
        MIX_MONITOR       = $00000002;  // supports Monitor Control */
        MIX_CROSSOVER     = $00000010;  // supports crossover change */
        MIX_LOUDNESS      = $00000020;  // supports loudness equalization */
        MIX_MUTE          = $00000040;  // supports channel mute */
        MIX_REVERB        = $00000080;  // supports reverb */
        MIX_STEREOENHANCE = $00000100;  // supports stereo enhance */
        MIX_CUSTOM1       = $00000200;  // supports custom effect #1 */
        MIX_CUSTOM2       = $00000400;  // supports custom effect #2 */
        MIX_CUSTOM3       = $00000800;  // supports custom effect #3 */
        MIX_LRVOLUME      = $00001000;  // separate left-right volume control */
        MIX_BASS          = $00800000;  // supports Bass */
        MIX_MID           = $00020000;  // supports Mid */
        MIX_TREBLE        = $00400000;  // supports Treble */
        MIX_PITCH         = $00200000;  // supports pitch modifications */
        MIX_GAIN          = $01000000;  // supports gain modifications */
        MIX_CHORUS        = $00004000;  // supports Bass */
        MIX_VOLUME        = 000800000;  // supports volume controls */


        MIX_STEREO_OFF    = $00000000;
        MIX_STEREO_STUDIO = $40004000;
        MIX_STEREO_HALL   = $80008000;
        MIX_STEREO_STADIUM= $C000C000;

        MCI_STEREO_OFF    = $00000000;
        MCI_STEREO_STUDIO = $00000001;
        MCI_STEREO_HALL   = $00000002;
        MCI_STEREO_STADIUM= $00000004;

        MIX_LOUDNESS_OFF  = $00000000;
        MIX_LOUDNESS_ON   = $FFFFFFFF;

        MCI_LOUDNESS_OFF  = $00000000;
        MCI_LOUDNESS_ON   = $FFFFFFFF;


// ERROR MESSAGES */
CONST
        MIXERR_BASE = 512;
        MIXERR_NOERR = 0;
        MIXERR_INVALIDHANDLE = MIXERR_BASE+0;
        MIXERR_INVALIDINPUT  = MIXERR_BASE+1;
        MIXERR_INVALIDOUTPUT = MIXERR_BASE+2;
        MIXERR_NOTSUPPORTED  = MIXERR_BASE+7;


// INPUT LINES */
// SOURCES */
        IN_SYNTHESIZER = 0;
        IN_MIXER = 1;
        IN_EXTERNAL = 2;
        IN_INTERNAL = 3;
        IN_MICROPHONE = 4;
        IN_PCM = 5;
        IN_PC_SPEAKER = 6;

// SINKS */
        OUT_AMPLIFIER = 0;
        OUT_PCM = 1;


// SOURCES */
        SOURCE_SYNTHESIZER    = $000000001;
        SOURCE_LINE           = $000000002;
        SOURCE_INTERNAL_AUDIO = $000000004;
        SOURCE_MICROPHONE     = $000000008;
        SOURCE_WAVE           = $000000010;
        SOURCE_PC_SPEAKER     = $000000020;
        SOURCE_NULL           = $000000040;
        SOURCE_MIDI           = $000000080;

// SINKS */
// lad--these defines are invalid. */
        SINK_LINE_OUT   = $000100000;
        SINK_SPEAKER    = $000200000;
        SINK_HEADPHONES = $000400000;
        SINK_NULL       = $000800000;
        SINK_ALL        = $001000000;

// lad--wpos addition */
        SINK_WAVE       = $002000000;
        SINK_MIDI       = $004000000;

//*************************START OF SPECIFICATIONS **************************/
//                                                                          */
// Module NAME:  MCD.H                                                      */
//                                                                          */
//                                                                          */
// OS/2 2.0 Multimedia Extensions MCD structures and definitions            */
//                                                                          */
// Copyright (c) International Business Machines Corporation 1990,1991      */
//                         All Rights Reserved                              */
//*--------------------------------------------------------------------------*/
//* Converted by Andry Svirgunov. Email: cool2@ngs.ru                        */
//* 14.11.2002                                                               */
//*                                                                          */
//************************//END OF SPECIFICATIONS ***************************/

CONST

        LIST_LEN=8;

        LINE_IN_ON  =$00000001;
        MIC_ON      =$00000002;
        LINE_OUT_ON =$00000004;

TYPE
        PSZ = PChar;
        SHandle = Word;
        HFile = SHandle;
//        HMTX = Longint;

//-------------------------------------------------------------------------*
//RIFF to ACPA Mode conversion Table
//------------------------------------------------------------------------*/

TYPE RIFFTABLE = Record
        ulDataType:LongInt;
        ulAUDIODDMode:LongInt;
        end;

TYPE _MCI_PORT_LIST = Record   // devid */
      ulDevType:LongInt;
      ulDevNum:LongInt;
        end;
//-------------------------------------------------------------------------*
//Amp/Mixer instance structure declaration
//------------------------------------------------------------------------*/

TYPE _MCI_AMP_INSTANCE = Record
        szDeviceName:Array [0..128] of char;
        szDriverName:Array [0..15] of char;
        hFile:HFile;
        usDeviceID:Integer;
        ulGlobalFile:LongInt;

        usMasterVolume:Integer;
        lLeftVolume:LongInt;
        lRightVolume:LongInt;
        lVolumeDelay:LongInt;
        lBalance:LongInt;

        lBalanceDelay:LongInt;
        lBass:LongInt;

        lTreble:LongInt;

        lPitch:LongInt;
        lGain:LongInt;

        lSRate:LongInt;
        lBitsPerSRate:LongInt;
        lBsize:LongInt;
        sMode:Integer;
        sChannels:Integer;
        ulFlags:LongInt;
        lMonitor:LongInt;
        lInput:LongInt;
        lOutput:LongInt;
        ulOperation:LongInt;
        ulGainLevel:LongInt;

        ulStoreRVolume:LongInt;
        ulStoreLVolume:LongInt;


        ulDosOpened:LongInt;
        ulInitOpen:LongInt;
        ulMasterVolume:LongInt;
        ulActive:LongInt;
        lStoreSRate:LongInt;
        lStoreBitsPerSRate:LongInt;
        ulStoreOperation:LongInt;
        sStoreMode:Integer;
        sStoreChannels:Integer;
        ulSetAmpDefaults:LongInt;
        ulDefaultsSet:LongInt;

        ulOutputDev:LongInt;
        ulInputDev:LongInt;
        ulResourcesUsed:LongInt;
        ulRestoreNotify:LongInt;
        ulCallback:LongInt;
        sUserParm:Integer;
        lNotifyFlag:LongInt;
//        LONG  ( //APIENTRY pfnAUDIOIF) ( PVOID, LONG, LONG, LONG, LONG);
        hmtxInstanceSem:HMTX;
        ulClass:LongInt;
        ulBlockAlignment:LongInt;
        ulHardwareMode:LongInt;
        rInputList:Array [0.. LIST_LEN ] of _MCI_PORT_LIST;
        rOutputList:Array [0.. LIST_LEN ] of _MCI_PORT_LIST;
        ulNumInputs:LongInt;
        ulNumOutputs:LongInt;

        ulDataType:LongInt;
        ulSubType:LongInt;
        ulBytesPerBlock:LongInt;
        ulMMTimePerBlock:LongInt;

//-------------------------------------------------------------------------*
//New resource management stuff
//------------------------------------------------------------------------*/



        pAudioModeData:Pointer;      // ptr to dev. specific resource data */
        pResourceDLL:Array [0.. 128 ] of Char;

        pProdInfo:PSZ;         // pointer to the product name        */
        ulDeviceID:LongInt;        // pointer to the device id.          */
        ulNumMappingRows:LongInt;  // num of RIFF->AUDIODD mapping modes */
        ulMatch:LongInt;
        pMapTable:^RIFFTABLE;        // additional RIFF->AUDIODD mappings  */

        ulNumDataTypes:LongInt;    // number of datatypes to check       */
        pDataTypeTable:Pointer;
        fMute:Boolean;
        ulResourceID:LongInt;      // offset in the resource dll to use  */
        ulConnDefaults:LongInt;    // Default values for connectors      */

        end;
TYPE PMCI_AMP_INSTANCE = ^_MCI_AMP_INSTANCE;


const
  MIDINAMESIZE                  = 40;
  MIDIPATCHNUM                  = 128;
  MIDITYPEAPPNAME               = 'MidiTypes';    // Appname in MIDITYPE.INI

//*******************************************************************/
// These structures describe the format of the MIDI maps that are   */
// located in the MIDITYPE.INI file.                                */
//*******************************************************************/

type
  // These structures describe the format of the MIDI maps that are
  // located in the MIDITYPE.INI file.
  MidiTypeEntry = record
    uGenMidiPatchNumber  : Integer;             // Device To General Midi Conversion
    uDevMidiPatchNumber  : Integer;             // General Midi to Device Conversion
    uVolumePercent       : Integer;             // Channel Patch Volume Scaler
    uGenMidiKeyNumber    : Integer;             // Device To General Midi Perc. Key Conversion
    uDevMidiKeyNumber    : Integer;             // General Midi to Device Perc. Key Conversion
  end;
  pMidiTypeEntry = ^MidiTypeEntry;

  _MidiType = record
    uStyle            : Integer;                // Midi Style
    uDrums10          : Integer;                // Patch 10 Drums if Perckey not supported
    uDrums16          : Integer;                // Patch 16 Drums if Perckey not supported
    ulReserved        : LongInt;                 // Reserved
    MidiTypeEntry     : array[0..midiPatchNum-1] of MIDITYPEENTRY;
                                               // Array of MIDITYPEENTRYs
    szPatchAndPercKeyName: array[0..2*MIDIPATCHNUM*MIDINAMESIZE-1] of char;
                                               // List of 128 Patch Names
                                               // that are null terminated, then a
                                               // list of 128 Percussion key names that
                                               // are double null terminated
                                               // Each item is null terminated
  end;
  MidiTpye=_MidiType;
  pMidiType = ^_MidiType;

const
  // Style Flags
  MT_PERCUSSIONKEYS             = $1;          // Percussion Keys are supported
  MT_MAPPERCKEYPATCHES          = $2;          // Map Percussion Key Patches


CONST
        STATUS_BIT = $80;

        TRACK_OFF = 0;
        TRACK_ON = 1;

        NORMAL_SYSEX = $F0;
        ESCAPE_SYSEX = $F7;

        // User defined messages for IO PROC
        MMIOM_START = $0E00;
        MMIOM_USER = $0F00;
        MMIOM_GETHEADER = MMIOM_START + 9;
        META = $FF;
// Meta stuff */
// user defined messages for IO PROC */
        MMIO_SEQ = MMIOM_USER + 1;
        MMIO_MERGE = MMIOM_USER + 2;
        MMIO_FROM_SEQ = MMIOM_USER + 3;
        MMIO_TIMED_READ = MMIOM_USER + 4;
        MMIO_SET_TRACKS = MMIOM_USER + 5;
        MMIO_FORMAT_O = MMIOM_USER + 6;
        MMIO_GETHEADER = MMIOM_GETHEADER;
        MMIO_GETFILELENTH = MMIOM_USER +10;
        MMIO_GETTRACKLENGTH = MMIOM_USER + 11;


type
  MidiHeader = record // midi header //
    chHeaderChunk               : array[0..3] of Char;
    case Boolean of
    true: (
        dwHeaderLength : Dword;
        wFormat : Word;
        wNumTracks : Word;
        wDivision : Word;
        vpAdditionalInformation_ : ^LongInt;
        );
        false: (
    ulHeaderLength              : LongInt;
    usFormat                    : Integer;
    usNumTracks                 : Integer;
    usDivision                  : Integer;
    vpAdditionalInformation     : Pointer;
    )
  end;
  pMidiHeader = ^MidiHeader;

  mmMidiHeader = record
    case boolean of
    true: (
    ulHeaderLength    : LongInt;                 // Length in Bytes
    ulContentType     : LongInt;                 // Image content
    ulMediaType       : LongInt;                 // type of media
    midiheader        : MidiHeader;            // header
    );
    false: (
        dwHeaderLength : DWORD;         // Length in Bytes */
        dwContentType : DWORD;          // Image content */
        dwMediaType : DWORD;            // type of media */
        midiheader_ : MIDIHEADER;       // header */
        )
  end;
  pmmMidiHeader = ^mmMidiHeader;

//***************************************************************************/
//                                                                          */
// Module Name: EVCB.H                                                      */
//                                                                          */
// OS/2 2.0 Multimedia Extensions Sync/Stream Manager Event Control         */
// Block defintitions.                                                      */
//                                                                          */
// Copyright (c) International Business Machines Corporation 1991, 1992     */
//                        All Rights Reserved                               */
//***************************************************************************/
// Converted by Andry Svirgunov. Email: cool2@ngs.ru                        */
// 14.11.2002                                                               */
//                                                                          */
//***************************************************************************/

//*********************************************
// * List of Implicit events:
// *    EVENT_EOS                 - Use IMPL_EVCB
// *    EVENT_ERROR               - Use IMPL_EVCB
// *    EVENT_STREAM_STOPPED      - Use IMPL_EVCB
// *    EVENT_SYNC_PREROLLED      - Use IMPL_EVCB
// *    EVENT_PLAYLISTMESSAGE     - Use PLAYL_EVCB
// *    EVENT_PLAYLISTCUEPOINT    - Use PLAYL_EVCB
// *    EVENT_QUEUE_OVERFLOW      - Use IMPL_EVCB
// **********************************************/

//*********************************************
// * List of Explicit events:
// *    EVENT_SYNCOVERRUN         - Use OVRU_EVCB
// *    EVENT_CUE_TIME            - Use TIME_EVCB
// *    EVENT_CUE_DATA            - Use DATA_EVCB
// *    EVENT_DATAUNDERRUN        - Use EVCB
// *    EVENT_DATAOVERRUN         - Use EVCB
// *    EVENT_CUE_TIME_PAUSE      - Use TIME_EVCB
// **********************************************/
CONST

        EVENT_SYNCOVERRUN       =$4;
        EVENT_CUE_TIME          =$5;       // Cue point in terms of stream time*/
        EVENT_CUE_DATA          =$6;       // Cue point in terms of data items */
        EVENT_DATAUNDERRUN      =$7;       // data underrun event from SH    */
        EVENT_DATAOVERRUN       =$8;       // data underrun event from SH    */
        EVENT_CUE_TIME_PAUSE    =$9;       // Pause when cue-point reached.  */
        EVENT_STATUS_LEVEL      =$10;      // Report status level            */



//*********************************************
// * SYNC_EVCB - Sync Pulse Event Control Block
// *             (This event is only seen by
// *              stream handlers)
// **********************************************/
        EVENT_SYNC              =$1;     // Synchronization point in stream  */

TYPE _SYNC_EVCB = RECORD         // syevcb                                     */
   ulType:LongInt;                     // EVENT_SYNC                       */
   ulSubType:LongInt;                  // Not used                         */
   ulSyncFlags:LongInt;                // 0 (input),status (output)        */
   hstream:LongInt;                    // handle to stream for this event  */
   hid:LongInt;                        // Handler id                       */
   ulStatus:LongInt;                   // Event status (output)            */
   mmtimeStart:MMTIME;                 // Filled in by Sync/Stream manager */
                                       //  at SpiEnableSync time. (input)  */
   mmtimeMaster:MMTIME;                // Filled in by Master SH (input)   */
   mmtimeSlave:MMTIME;                 // Filled in by slave SH. (output)  */
   end;

TYPE PSYNC_EVCB = ^_SYNC_EVCB;         // Ptr to a  EVCB       */

// ulSyncFlags:
CONST

        SYNCOVERRUN        =$0001;    // Set by SSM if sync pulse comes in before    */
                                      //  a has processed the current sync pulse.    */
        SYNCPOLLING        =$0002;    // SSM set this after filling the handler EVCB.*/
                                      //  Handler resets it after it processes the   */
                                      //  contents of EVCB. The SSM must check this  */
                                      //  bit before modifying EVCB. If bit already  */
                                      //  set, set the SYNC overrun bit.             */


        EVENT_RESERVED     =$2;       // RESERVED                         */


//*********************************************
// * IMPL_EVCB - Implicit Event Control Block
// *             (This is a parameter on the
// *              SpiCreatStrem API)
// **********************************************/
        EVENT_IMPLICIT_TYPE=$3;     // These events are always supported*/


TYPE _IMPL_EVCB = RECORD         // imevcb                                     */
   ulType:LongInt;                     // EVENT_IMPLICIT_TYPE              */
   ulSubType:LongInt;                  // One of the implicit events (input) */
   ulFlags:LongInt;                    // 0 (Input), status (Output)         */
   hstream:LongInt;                    // handle to stream for this event (input/output) */
   hid:LongInt;                        // 0 (input), handler Id (output)     */
   ulStatus:LongInt;                   // Event status/error return code (output) */
   unused1:LongInt;                    // 0 (input)                        */
   unused2:LongInt;                    // 0 (input)                        */
   unused3:LongInt;                    // 0 (input)                        */
   end;
TYPE PIMPL_EVCB = ^_IMPL_EVCB;

// ulSubtype:                                                              */

CONST

        EVENT_EOS               =$1;     // End of Stream                    */
        EVENT_ERROR             =$2;     // Error in Stream handler or device driver        */
        EVENT_STREAM_STOPPED    =$3;     // Stream is in a stopped state (Discard or Flush) */
        EVENT_SYNC_PREROLLED    =$4;     // All streams are prerolled (buffers   */
                                         //  have been filled by the Source SH's.*/
        EVENT_PLAYLISTMESSAGE   =$5;     // Memory SH playlist message event     */
        EVENT_PLAYLISTCUEPOINT  =$6;     // Memory SH playlist cue point         */
        EVENT_QUEUE_OVERFLOW    =$7;     // Event Queue overflow. Indicates lost */
                                         //  events. Application must use this   */
                                         //  to clear any waiting conditions.    */
        EVENT_START             =$8;     // Start stream                         */


//*********************************************
// * PLAYL_EVCB - Playlist Message Event Control Block
// **********************************************/
// (IMPLICIT EVENT)                                                        */
TYPE _PLAYL_EVCB=RECORD       // plevcb                                    */
   ulType:LongInt;                     // EVENT_IMPLICIT_TYPE              */
   ulSubType:LongInt;                  // EVENT_PLAYLISTMESSAGE or EVENT_PLAYLISTCUEPOINT */
   ulFlags:LongInt;                    // 0 (input), status (output)                      */
   hstream:LongInt;                    // handle to stream for this event (input/output)  */
   hid:LongInt;                        // 0 (input), handler Id (output)                  */
   ulStatus:LongInt;                   // Playlist Instruction number (output)            */
   ulMessageParm:LongInt;              // 0 (input), Message from playlist (output)       */
   unused1:LongInt;                    // 0 (input)                        */
   unused2:LongInt;                    // 0 (input)                        */
   end;
TYPE PPLAYL_EVCB = ^_PLAYL_EVCB;             // Ptr to a  EVCB       */


//*********************************************
// * OVRU_EVCB - Sync Overrun Event Control Block
// *             (Applications can request to be
// *              notified whenever a Sync overrun
// *              occurs.
// **********************************************/
TYPE _OVRU_EVCB = RECORD     // ovevcb */
   ulType:LongInt;                     // EVENT_SYNCOVERRUN                */
   ulSubType:LongInt;                  // Not used                         */
   ulFlags:LongInt;                    // 0 (input), status (output)       */
   hstream:LongInt;                    // handle to stream for this event (input/output) */
   hid:LongInt;                        // handler Id (input/output)        */
   ulStatus:LongInt;                   // Event status/error return code (output) */
   mmtimeSlave:MMTIME;                // 0 (input), Slave stream time (output)   */
   mmtimeStart:MMTIME;                // 0 (input), Slave start offset (offset)  */
   mmtimeMaster:MMTIME;               // 0 (input), Master stream time (output)  */
   end;

TYPE POVRU_EVCB = ^_OVRU_EVCB;             // Ptr to a  EVCB       */


//*********************************************
// * TIME_EVCB - Cue Time Event Control Block
// *             (Applications can request to be
// *              notified for a time cue point)
// **********************************************/

TYPE _TIME_EVCB = RECORD     // tievcb                                     */
   ulType:LongInt;                     // EVENT_CUE_TIME                   */
   ulSubType:LongInt;                  // Not used                         */
   ulFlags:LongInt;                    // Single/Recurring(input/output)   */
   hstream:LongInt;                    // handle to stream for this event (input/output)*/
   hid:LongInt;                        // handler Id (input/output)                     */
   ulStatus:LongInt;                   // Event status/error return code (output)       */
   mmtimeStream:MMTIME;               // Stream time (input/output)       */
   unused1:LongInt;                    // 0 (input)                        */
   unused2:LongInt;                    // 0 (input)                        */
   end;

//TYPE PTIME_EVCB = ^_TIME_EVCB;             // Ptr to a  EVCB       */

//                                                                         */
// ulFlags:                                                                */
CONST

        EVENT_SINGLE            =$0;     // This is single one time event    */
        EVENT_RECURRING         =$1;     // This is recurring event          */


//*********************************************
// * DATA_EVCB - Cue Data Event Control Block
// *             (Applications can request to be
// *              notified for a data cue point)
// **********************************************/

TYPE _DATA_EVCB = RECORD     // daevcb                                     */
   ulType:LongInt;                     // EVENT_CUE_DATA                   */
   ulSubType:LongInt;                  // Event SubType                    */
   ulFlags:LongInt;                    // Single/Recurring(input/output)   */
   hstream:LongInt;                    // handle to stream for this event (input/output) */
   hid:LongInt;                        // handler Id (input/output)                      */
   ulStatus:LongInt;                   // Event status/error return code (output)        */
   mmtimeStream:MMTIME;                // Stream time (input/output)                     */
   ulEventParm1:LongInt;               // Data to Cue on (input/output)                  */
                                       //  This is interpreted by the stream handler     */
   ulEventParm2:LongInt;               // Length of data cue buffer (input/output)       */
   end;
TYPE PDATA_EVCB = ^_DATA_EVCB;         // Ptr to a  EVCB       */

// ulFlags:                                                                */
//#define EVENT_SINGLE          0L        This is single one time event    */
//#define EVENT_RECURRING       1L        This is recurring event          */

CONST

        EVENT_DATAPTR           =$2;   // ulEventParm1 if ptr to buffer and*/
                                       //  ulEventParm2 is a length of buffer, */
                                       //  else ulEventParm1 is data.          */


//*****************
// * EVCB_ulType:  (Stream Handler defined types)
// ******************/
        SH_TYPE           =$80000000;  // Stream Handler defined types must */
                                       // have the high order bit set in the*/
                                       // ulType field.                     */
                                       // events: 0H - 7FFFFFFFH are reserved!*/

//*********************************************
// * EVCB - Generic Event Control Block (Use the
// *        specific one in the SSM.H if possible)
// **********************************************/
TYPE _EVCB = RECORD     // evcb */
   ulType:LongInt;                     // Event type (input)               */
   ulSubType:LongInt;                  // Event SubType (input)            */
   ulFlags:LongInt;                    // 0 (Input), status (Output)       */
   hstream:LongInt;                    // handle to stream for this event  */
   hid:LongInt;                        // handler Id (input/output)        */
   ulStatus:LongInt;                   // Event status (output)            */
   ulEventParm1:LongInt;               // Event parameters (input), HID if implicit event */
   ulEventParm2:LongInt;               // Event parameters (input)         */
   ulEventParm3:LongInt;               // Event parameters (input)         */
   end;
TYPE PEVCB = ^_EVCB;             // Ptr to a  EVCB       */

//*********************************************
// * STATUS_LEVEL_EVCB - Status Level Event Control Block
// **********************************************/
TYPE _STATUS_LEVEL_EVCB = RECORD   // stevcb */
   // standard part */
   ulType:LongInt;                     // Event type (input)               */
   ulSubType:LongInt;                  // Event SubType (input)            */
   pMCDInstance:LongInt;               // for MCD use */
   hstream:LongInt;                    // handle to stream for this event  */
   hid:LongInt;                        // handler Id (input/output)        */
   // custom part */
   ulSample:ARRAY [0..4] OF LongInt;                // Samples */
   end;
TYPE PSTATUS_EVCB = ^_STATUS_LEVEL_EVCB;     // Ptr to a  EVCB       */

//***************************************************************************/
//                                                                          */
// Module Name: DCB.H                                                       */
//                                                                          */
// OS/2 2.0 Multimedia Extensions Sync/Stream Manager Device Control        */
// Block defintitions.                                                      */
//                                                                          */
// Copyright (c) International Business Machines Corporation 1991, 1992     */
//                        All Rights Reserved                               */
//***************************************************************************/
// Converted by Andry Svirgunov. Email: cool2@ngs.ru                        */
// 14.11.2002                                                               */
//                                                                          */
//***************************************************************************/

CONST MAX_SPI_NAME = 9;

//**************************************************************************
// *
// * DCB - Common Device Control Block
// *               (*** PACKED STRUCTURE ***)
// ***************************************************************************/
TYPE _DCB = RECORD              // dcb  -  Device Control Block                      */
        ulDCBLen : LongInt;     // length of structure                 */
        szDevName: Array [0..MAX_SPI_NAME] of char;// device driver name                  */
        end;
TYPE PDCB = ^_DCB;


//**************************************************************************
// *
// * E_DCB - Extended Device Control Block
// *
// *         This structure will grow over time as new fields are added
// *         to the end of the structure.  If you manipulate the structure,
// *         be sure to check the length field.
// *
// *               (*** PACKED STRUCTURE ***)
// ***************************************************************************/

TYPE E_DCB = RECORD             // e_dcb  -  Extended Device Control Block          */
        ulDCBLen : LongInt;     // length of structure                 */
        szDevName: Array [0..MAX_SPI_NAME] of char;// device driver name                  */
        ulSysFileNum: LongInt;  // file handle number                  */
        end;
TYPE PE_DCB = ^E_DCB;
TYPE PDCB_AUDIOSH = ^E_DCB;


//**************************************************************************
// *
// * VSD_DCB - VSD Device Control Block
// *
// *         This structure will allow stream handlers to use the VSD DLL
// *         by using by the additional fields in the structure.
// *
// *               (*** PACKED STRUCTURE ***)
// ***************************************************************************/
TYPE _VSD_DCB = RECORD          // vsd_dcb - VSD Device Control Block            */
        ulDCBLen : Longint;     // length of structure                 */
        szDevName: Array [0..MAX_SPI_NAME] of char;// device driver name                  */
        ulSysFileNum: LongInt;  // file handle number                  */
        hvsd : LongInt;         // Handle to VSD instance              */
        pfnEntryPoint : ^PFN;   // Address of VSD entry point          */
        ulReserved1 : LongInt;  // Reserved for system                 */
        ulReserved2 : LongInt;  // Reserved for system                 */
        end;
TYPE PVSD_DCB = ^_VSD_DCB;

//****************************************************************************/
//*                                                                          */
//* Module Name: ACB.H                                                       */
//*                                                                          */
//* OS/2 2.0 Multimedia Extensions Sync/Stream Manager Associate             */
//* Control Block Definitions.                                               */
//*                                                                          */
//* Copyright (c) International Business Machines Corporation 1991, 1992     */
//*                        All Rights Reserved                               */
//*--------------------------------------------------------------------------*/
//* Converted by Andry Svirgunov. Email: cool2@ngs.ru                        */
//* 14.11.2002                                                               */
//*                                                                          */
//****************************************************************************/
CONST

        MAX_PORT_NAME=40;           // Max port name length         */
        MAX_PORTS    =16;           // Max number ports             */


//**********************************************
// *
// * ACB - Associate Control Block
// *
// *       Each stream handler must define the
// *       ACB for each data object type that
// *       expect ACB's from the application.
// *
// **********************************************/
TYPE _ACB = RECORD              //* acb  -  Assoc. Control Block  */
        ulACBLen : LongInt;     //* length of structure */
        ulObjType : LongInt;
        ulParm1 : LongInt;
        ulParm2 : LongInt;
        end;
TYPE PACB = ^_ACB;              //* Ptr to an ACB        */


//******************************************************
// * FSSH - File system stream handler MMIO Object ACB
// ******************************************************/
CONST

        ACBTYPE_MMIO=$0001;  //* MMIO object                      */

TYPE _ACB_MMIO = RECORD         //* acbmmio  - MMIO Assoc. Control Block       */
        ulACBLen : LongInt;     //* length of structure                   */
        ulObjType : LongInt;    //* ACBTYPE_MMIO                          */
        hmmio : HMMIO ;         //* Handle of media element mgr obj       */
        end;
TYPE PACB_MMIO = ^_ACB_MMIO;    //* Ptr to an ACB  */


//******************************************************
// * MSH - Memory stream handler Playlist Object ACB
// ******************************************************/
CONST
        ACBTYPE_MEM_PLAYL=$0003;  //* Memory playlist object           */

TYPE _ACB_MEM_PLAYL = RECORD    //* acbplyl - File system Assoc. Control Block */
        ulACBLen : LongInt;     //* length of structure                   */
        ulObjType : LongInt;    //* ACBTYPE_MEM_PLAYL                     */
        pMemoryAddr : Pointer;  //* Starting address of memory obj        */
        end;
TYPE PACB_MEM_PLAYL = ^_ACB_MEM_PLAYL;  //* Ptr to an ACB  */


//******************************************************
// * CDDASH - CD DA stream handler Object ACB
// ******************************************************/
CONST
        ACBTYPE_CDDA=$0004;  //*  Compact disk - digital audio obj*/


TYPE _ACB_CDDA = RECORD         //* acbcdda - CD Assoc. Control Block          */
        ulACBLen : LongInt;     //* length of structure                   */
        ulObjType : LongInt;    //* ACBTYPE_CDDA                          */
        bCDDrive : Char;        //* CD drive letter                       */
        end;
TYPE PACB_CDDA = ^_ACB_CDDA;    //* Ptr to an ACB  */

//******************************************************
// * MISH - MIDI stream handler port-stream table ACB
// ******************************************************/
CONST
        ACBTYPE_MISH=$0005;      //* MIDI port-stream table       */

TYPE _ACB_MISH = RECORD         //* acbmish - MIDI Assoc. Control Block          */
        ulACBLen : LongInt;     //* length of structure          */
        ulObjType: LongInt;     //* ACBTYPE_MISH                 */
        hStreamDefault : Longint;//* Default hstream to use when mapper */
                                 //* is turned off.               */
        ulDeviceTypeID : LongInt;//* device type id               */
        ulpMapperPorts : LongInt;//* pointer to mapper port table */
        ulNumInStreams : LongInt;
        hStreamIn : Array [0..MAX_PORTS] of Longint;//* Array of Input streams       */
        ulNumOutStreams : LongInt;
        hStreamsOut : Array [0..MAX_PORTS] of Longint;//* Array of Output streams      */
                                                      //* The index into the array is  */
                                                      //* the source channel for that  */
                                                      //* stream.                      */

        end;
TYPE PACB_MISH = ^_ACB_MISH;    //* Ptr to MIDI associate control block */

//******************************************************
// * MISH - MIDI stream handler SET ACB
// ******************************************************/
CONST
        ACBTYPE_SET=$0006;      //* MIDI set function            */

TYPE _ACB_SET = RECORD          //* acbset - Set Assoc. Control Block             */
        ulACBLen :LongInt;      //* length of structure          */
        ulObjType : LongInt;    //* ACBTYPE_SET                  */
        ulFlags : LongInt;      //* Set flags                    */
        ulReserved : Longint;   //* Reserved                     */
        end;
TYPE PACB_SET = ^_ACB_SET;      //* Ptr to set associate control block */

//* ulFlags defines:                                                        */
CONST

        MIDI_MAP_ON =$0000;      //* turn mapping function on in MISH   */
        MIDI_MAP_OFF=$0001;      //* turn mapping function off in MISH  */


//******************************************************
// * NULLSH - Null stream handler user ACB
// ******************************************************/
        ACBTYPE_NULLSH=$0007;  //* User od Null stream handler      */

TYPE _ACB_NULISH = RECORD       //* acbnullsh - NULLSH Assoc. Control Block    */
        ulACBLen : LongInt;     //* length of structure                   */
        ulObjType : LongInt;    //* ACBTYPE_NULLSH                        */
        pfnEntry : PFN ;        //* user callback entry point             */
        ulReserved : LongInt;   //* Reserved                              */
        end;
TYPE PACB_NULISH = ^_ACB_NULISH;//* Ptr to an ACB  */

//******************************************************
// * MTSH - MultiTrack Stream Handler Object ACB
// ******************************************************/
CONST

        ACBTYPE_MTSH=$0008;  //* MTSH object                      */

TYPE _ACB_MTSH = RECORD                 //* acbmtsh  - MTSH Assoc. Control Block       */
        ulACBLen : LongInt;             //* length of structure                   */
        ulObjType : Longint;            //* ACBTYPE_MTSH                          */
        hmmio : HMMIO;                  //* Handle of media element mgr obj       */
        mmtrackinfo : _MMTRACKINFO;     //* track for this stream                 */
        ulFlags : LongInt;              //* flags                                 */
        ulMaxBytesPerSec : LongInt;     //* Max bytes per second                  */
        ulAvgBytesPerSec : Longint;     //* Avg bytes per second                  */
        mmtimePerFrame : MMTIME;        //* frame display time or 0L              */
        ulTotalFrames : LongInt;        //* total number of video frames          */
        end;
TYPE PACB_MTSH = ^_ACB_MTSH;            //* Ptr to an ACB  */

//* ulFlags defines:                                                        */
CONST
        TRACK_ACTIVATE   =$0000;    //* Activate track in stream handler  */
        TRACK_DEACTIVATE =$0001;    //* Deactivate track in stream handler  */


//******************************************************
// * CSH - CODEC stream handler Object ACB
// ******************************************************/
        ACBTYPE_CODECSH=$0009;      //* CODEC object                 */
TYPE _ACB_CODECSH = RECORD      //* acbcodecsh - CODEC SH acb                 */
        ulACBLen : Longint;     //* length of structure          */
        ulObjType : Longint;    //* ACBTYPE_CODECSH              */
        hstreamToPair : LongInt;//* 2nd stream of pair           */
        pMmioInfo : Longint;    //* info for IO Proc             */
        ulInfoLength : Longint; //* length of MmioInfo           */
        pCodecControl : Longint;//* CODEC control info           */
        ulControlLength : Longint;//* length of CodecControl       */
        end;
TYPE PACB_CODECSH = ^_ACB_CODECSH;//* Ptr to associate control block */


Implementation

Function mmioFOURCC(ch0,ch1,ch2,ch3:Char): Cardinal;
begin
  mmioFOURCC:=Cardinal(ord(ch0)) or (Cardinal(ord(ch1)) shl 8) or
              (Cardinal(ord(ch2)) shl 16) or (Cardinal(ord(ch3)) shl 24);
end;

Begin
  FOURCC_R565:=mmioFOURCC( 'R', '5', '6', '5' );
  FOURCC_R555:=mmioFOURCC( 'R', '5', '5', '5' );
  FOURCC_R664:=mmioFOURCC( 'R', '6', '6', '4' );
  FOURCC_RGB3:=mmioFOURCC( 'R', 'G', 'B', '3' );
  FOURCC_BGR3:=mmioFOURCC( 'B', 'G', 'R', '3' );
  FOURCC_RGB4:=mmioFOURCC( 'R', 'G', 'B', '4' );
  FOURCC_BGR4:=mmioFOURCC( 'B', 'G', 'R', '4' );
  FOURCC_LUT8:=mmioFOURCC( 'L', 'U', 'T', '8' );
  FOURCC_LT12:=mmioFOURCC( 'L', 'T', '1', '2' );
  FOURCC_GREY:=mmioFOURCC( 'G', 'R', 'E', 'Y' );
  FOURCC_GY16:=mmioFOURCC( 'G', 'Y', '1', '6' );
  FOURCC_Y888:=mmioFOURCC( 'Y', '8', '8', '8' );
  FOURCC_Y2X2:=mmioFOURCC( 'Y', '2', 'X', '2' );
  FOURCC_Y4X4:=mmioFOURCC( 'Y', '4', 'X', '4' );
  FOURCC_YUV9:=mmioFOURCC( 'Y', 'U', 'V', '9' );
  FOURCC_Y644:=mmioFOURCC( 'Y', '6', '4', '4' );
  FOURCC_MONO:=mmioFOURCC( 'M', 'O', 'N', 'O' );
  FOURCC_Y422:=mmioFOURCC( 'Y', '4', '2', '2' );
  FOURCC_Y42B:=mmioFOURCC( 'Y', '4', '2', 'B' );
  FOURCC_Y42D:=mmioFOURCC( 'Y', '4', '2', 'D' );
  FOURCC_Y411:=mmioFOURCC( 'Y', '4', '1', '1' );
  FOURCC_VGA :=mmioFOURCC( 'V', 'G', 'A', ' ' );

  FOURCC_ULTI:=mmioFOURCC('U', 'L', 'T', 'I');
  FOURCC_RT21:=mmioFOURCC('R', 'T', '2', '1');
  FOURCC_DIB :=mmioFOURCC('D', 'I', 'B', ' ');

  FOURCC_RIFF:=mmioFOURCC( 'R', 'I', 'F', 'F' );
  FOURCC_LIST:=mmioFOURCC( 'L', 'I', 'S', 'T' );
  FOURCC_MEM :=mmioFOURCC( 'M', 'E', 'M', ' ' );
  FOURCC_DOS :=mmioFOURCC( 'D', 'O', 'S', ' ' );
  FOURCC_BND :=mmioFOURCC( 'B', 'N', 'D', ' ' );
  FOURCC_FREE:=mmioFOURCC( 'F', 'R', 'E', 'E' );
  FOURCC_DEL :=mmioFOURCC( 'D', 'E', 'L', ' ' );
  FOURCC_CTOC:=mmioFOURCC( 'C', 'T', 'O', 'C' );
  FOURCC_CGRP:=mmioFOURCC( 'C', 'G', 'R', 'P' );
  FOURCC_CF  :=mmioFOURCC( 'C', 'F', ' ', ' ' );
End.
