{
    Copyright (c) 1990-1993 International Business Machines Corporation
    Copyright (c) 2002 by Andry Svirgunov (cool2@ngs.ru)
    Copyright (c) 2002-2003 by Yuri Prokushev (prokushev@freemail.ru)

    OS/2 2.0 Multimedia Extensions Input/Output Manager

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
@abstract(OS/2 2.0 Multimedia Extensions Input/Output Manager)
@author(Andry Svirgunov (cool2@ngs.ru))
@author(Yuri Prokushev (prokushev@freemail.ru))
@created(17 Dec 2002)
@lastmod(19 Jan 2003)
OS/2 2.0 Multimedia Extensions Input/Output Manager
Warning: This code is alfa. Future versions of this unit will propably
not be compatible.
}
Unit MMIO;

Interface

Uses
  MMBase;

Const
  LibName='MMPM'; // !!TODO!! Subject to replace by MMIO

// MS compat defines.
Type
  HPSTR = PChar;
  Hwnd = LongInt;
  PFourCC = ^FourCC;
  HMMCF = hmmIO;
  HMODULE = LongInt;
  PHModule = ^Hmodule;
  pLong = ^LongInt;

// Define MMIO public data structures.
  mmCkInfo = record
    ckid              : FourCC;                // Chunk id (FourCC)
    ckSize            : LongInt;                 // Chunk Size (bytes)
    fccType           : FourCC;                // FourCC Type (if ckid RIFF or LIST)
    ulDataOffset      : LongInt;                 // File Offset of data portion of chunk
    ulFlags           : LongInt;                 // MMIO_DIRTY (if new chunk)
  end;
  pmmCkInfo = ^mmCkInfo;
  lpmmckinfo = pmmckinfo; // MS compat define */

  mmIOProc = function( pmmIOInfo: Pointer; usMsg: Word; lp1, lp2: LongInt): LongInt;
  pmmIOProc = mmIOProc;
  pCodecProc = mmIOProc;
  ppmmioproc = mmIOProc;
  LPMMIOProc = pmmIOProc;

  mmIOInfo = record
    ulFlags           : LongInt;                 // Open flags
    fccIOProc         : FourCC;                // FourCC of the IOProc to use
    pIOProc           : mmIOProc;              // Function Pointer to IOProc to use
    ulErrorRet        : LongInt;                 // Extended Error return code
    cchBuffer         : LongInt;                  // I/O buff size (if used), Fsize if MEM
    pchBuffer         : pChar;                 // Start of I/O buff
    pchNext           : pChar;                 // Next char to read or write in buff
    pchEndRead        : pChar;                 // Last char in buff can be read + 1
    pchEndWrite       : pChar;                 // Last char in buff can be written + 1
    lBufOffset        : LongInt;                  // Offset in buff to pchNext
    lDiskOffset       : LongInt;                  // Disk offset in file
    aulInfo           : Array[0..3] of LongInt;  // IOProc specific fields
    lLogicalFilePos   : LongInt;                  // Actual file position, buffered or not
    ulTranslate       : LongInt;                 // Translation field
    fccChildIOProc    : FourCC;                // FourCC of Child IOProc
    pExtraInfoStruct  : Pointer;               // Pointer to a structure of related data
    mmio              : hmmIO;                 // Handle to media element
  end;
  pmmIOInfo = ^mmIOInfo;
  lpmmIOInfo = pmmIOInfo;


  mmCfInfo = record
    ulHeaderSize      : LongInt;                 // CTOC header size
    ulEntriesTotal    : LongInt;                 // Num of CTOC table entries
    ulEntriesDeleted  : LongInt;                 // Num of CTOC table entries deleted
    ulEntriesUnused   : LongInt;                 // Num of unused CTOC entries
    ulBytesTotal      : LongInt;                 // Combined byte size of all CGRP elements
    ulBytesDeleted    : LongInt;                 // Byte size of all deleted CGRP elements
    ulHeaderFlags     : LongInt;                 // Info about entire compound file (CF)
    usEntrySize       : Word;                // Size of each CTOC table entry
    usNameSize        : Word;                // Size of name field in entry, default 13
    usExHdrFields     : Word;                // Num CTOC header extra fields
    usExEntFields     : Word;                // Num CTOC entry extra fields
  end;

// Note:  The are variable length DWORD arrays that may be present
//        at the end of the MMCFINFO structure.

//   DWORD    (*adwExHdrFldUsage)[];  */   // Array of header extra usage fields */
//   DWORD    (*adwExtEntFldUsage)[]; */   // Array of entry extra usage fields  */
//   DWORD    (*adwExHdrField)[];     */   // Array of header extra fields       */
  pmmCfInfo = ^mmCfInfo;
  lpmmCfInfo = pmmcfInfo;

  mmCtocEntry = record
    ulOffset          : LongInt;                 // Offset of element within CGRP
    ulSize            : LongInt;                 // Size of element
    ulMedType         : LongInt;                 // Fourcc of element
    ulMedUsage        : LongInt;                 // Possible sub type
    ulCompressTech    : LongInt;                 // Compression technique used
    ulUncompressBytes : LongInt;                 // Actual size of uncompressed element
  end;


// Note:  A variable length name field and possibly a DWORD array may
//        be present at the end of the MMCTOCENTRY structure.

//   DWORD    (*adwExEntField)[]; */   // Array of entry extra fields */
//   PSZ      pszElementName[];   */   // Name of element, variable length */

  pmmCtocEntry = ^mmCtocEntry;
  lpmmCtocEntry = pmmCtocEntry;

// Conversion Flags/Data Structures:

const
  // Conversion Flags/Data Structures:
  MMIO_MEDIATYPE_IMAGE          = $00000001;  // Image media
  MMIO_MEDIATYPE_AUDIO          = $00000002;  // Audio media
  MMIO_MEDIATYPE_MIDI           = $00000004;  // MIDI media
  MMIO_MEDIATYPE_COMPOUND       = $00000008;  // Cmpd media
  MMIO_MEDIATYPE_OTHER          = $00000010;  // Other media
  MMIO_MEDIATYPE_UNKNOWN        = $00000020;  // Unknown media
  MMIO_MEDIATYPE_DIGITALVIDEO   = $00000040;  // Digital Video
  MMIO_MEDIATYPE_ANIMATION      = $00000080;  // Future: Not Supported
  MMIO_MEDIATYPE_MOVIE          = $00000100;  // Movie File

  MMIO_CANREADTRANSLATED        = $00000001;  //IOProc Cpbilty Flgs
  MMIO_CANWRITETRANSLATED       = $00000002;  // "       "       "
  MMIO_CANREADWRITETRANSLATED   = $00000004;  // "       "       "
  MMIO_CANREADUNTRANSLATED      = $00000008;  // "       "       "
  MMIO_CANWRITEUNTRANSLATED     = $00000010;  // "       "       "
  MMIO_CANREADWRITEUNTRANSLATED = $00000020;  // "       "       "
  MMIO_CANSEEKTRANSLATED        = $00000040;  // "       "       "
  MMIO_CANSEEKUNTRANSLATED      = $00000080;  // "       "       "
  MMIO_CANINSERTUNTRANSLATED    = $00000100;  // "       "       "
  MMIO_CANSAVEUNTRANSLATED      = $00000200;  // "       "       "
  MMIO_CANINSERTTRANSLATED      = $00000400;  // "       "       "
  MMIO_CANSAVETRANSLATED        = $00000800;  // "       "       "
  MMIO_CANMULTITRACKREADTRANSLATED = $00001000; // "       "       "
  MMIO_CANMULTITRACKREADUNTRANSLATED = $00002000; // "       "       "
  MMIO_CANMULTITRACKWRITETRANSLATED = $00004000; // "       "       "
  MMIO_CANMULTITRACKWRITEUNTRANSLATED = $00008000; // "       "       "
  MMIO_CANTRACKSEEKTRANSLATED   = $00010000;  // "       "       "
  MMIO_CANTRACKSEEKUNTRANSLATED = $00020000;  // "       "       "
  MMIO_CANTRACKREADTRANSLATED   = $00040000;  // "       "       "
  MMIO_CANTRACKREADUNTRANSLATED = $00080000;  // "       "       "
  MMIO_CANTRACKWRITETRANSLATED  = $00100000;  // "       "       "
  MMIO_CANTRACKWRITEUNTRANSLATED = $00200000; // "       "       "

  MMIO_IOPROC_STORAGESYSTEM     = $00000001;  // IOProc Types
  MMIO_IOPROC_FILEFORMAT        = $00000002;
  MMIO_IOPROC_DATAFORMAT        = $00000004;

Type
  mmFormatInfo = record
    ulStructLen       : LongInt;                 // Length of this structure
    fccIOProc         : FourCC;                // IOProc identifier
    ulIOProcType      : LongInt;                 // Type of IOProc
    ulMediaType       : LongInt;                 // Media Type
    ulFlags           : LongInt;                 // IOProc capability flags
    szDefaultFormatExt : Array[0..Sizeof(FourCC)] of Char;
                                               // Default extension 4 + null
    ulCodePage        : LongInt;                 // Code Page
    ulLanguage        : LongInt;                 // Language
    lNameLength       : LongInt;                  // length of identifier string
  end;
  pmmFormatInfo = ^mmFormatInfo;

Const
  MMIO_IMAGE_UNKNOWN            = $00000000;   // Unknown image content
  MMIO_IMAGE_DRAWING            = $00000001;   // Simple drawing
  MMIO_IMAGE_GRAPH              = $00000002;   // Graphs & Cartoons
  MMIO_IMAGE_PHOTO              = $00000004;   // Varying Color & Shades

  MAX_PALETTE                   = 256;

//*********************************************
// *
// * GENPAL - Generic Header Palette
// *
// **********************************************/

Type
  RGB2 = record
    bBlue:     Byte;            { Blue component of the color definition }
    bGreen:    Byte;            { Green component of the color definition}
    bRed:      Byte;            { Red component of the color definition  }
    fcOptions: Byte;            { Reserved, must be zero                 }
   end;
  PRGB2 = ^RGB2;

Type
  GENPAL = RECORD          // genpal */
    ulStartIndex:LongInt;       // starting RGB index */
    ulNumColors:LongInt;        // number of following entries */
    prgb2Entries:PRGB2;         // 256 RGB entries */
  end;
  PGENPAL = ^GENPAL;          // Ptr to a generic palette */

Type
  BitMapInfoHeader2 = record
    cbFix:           LongInt;     { Length of structure                    }
    cx:              LongInt;     { Bit-map width in pels                  }
    cy:              LongInt;     { Bit-map height in pels                 }
    cPlanes:         Word; { Number of bit planes                   }
    cBitCount:       Word; { Number of bits per pel within a plane  }
    ulCompression:   Longint;     { Compression scheme used to store the bitmap }
    cbImage:         Longint;     { Length of bit-map storage data in bytes}
    cxResolution:    Longint;     { x resolution of target device          }
    cyResolution:    Longint;     { y resolution of target device          }
    cclrUsed:        Longint;     { Number of color indices used           }
    cclrImportant:   Longint;     { Number of important color indices      }
    usUnits:         Word; { Units of measure                       }
    usReserved:      Word; { Reserved                               }
    usRecording:     Word; { Recording algorithm                    }
    usRendering:     Word; { Halftoning algorithm                   }
    cSize1:          Longint;     { Size value 1                           }
    cSize2:          Longint;     { Size value 2                           }
    ulColorEncoding: Longint;     { Color encoding                         }
    ulIdentifier:    Longint;     { Reserved for application use           }
  end;
  pbitmapinfoheader2=^bitmapinfoheader2;

  xdibHdr_Prefix = record
    ulMemSize         : LongInt;                 // Length of bitmap
    ulPelFormat       : LongInt;
    usTransType       : Word;
    ulTransVal        : LongInt;
  end;

  mmxdibHeader = record                        // XDIB Header
    XDIBHeaderPrefix  : XDIBHDR_PREFIX;
    BMPInfoHeader2    : BITMAPINFOHEADER2;
  end;
  pmmxdibHeader = ^mmxdibHeader;

  mmImageHeader = record
    ulHeaderLength    : LongInt;                 // Length in Bytes
    ulContentType     : LongInt;                 // Image content
    ulMediaType       : LongInt;                 // Media Type
    mmXDIBHeader      : mmXDIBHeader;          // OS/2 2.0 PM compat header
    bmiColors         : Array[0..MAX_PALETTE-1] of RGB2; // PM compatible palette
  end;
  pmmImageHeader = ^mmImageHeader;


Const
  MMIO_AUDIO_UNKNOWN            = $00000000;   // Unknown image content
  MMIO_AUDIO_VOICE              = $00000001;   // Limited Range
  MMIO_AUDIO_MUSIC              = $00000002;   // FM Radio or equivalent
  MMIO_AUDIO_HIFI               = $00000004;   // High quality recording

Type
  Wave_Header = record
    usFormatTag       : Word;                // Type of wave format
    usChannels        : Word;                // Number of channels
    ulSamplesPerSec   : LongInt;                 // Sampling rate
    ulAvgBytesPerSec  : LongInt;                 // Avg bytes per sec
    usBlockAlign      : Word;                // Block Alignment in bytes
    usBitsPerSample   : Word;                // Bits per sample
  end;

  xWav_HeaderInfo = record
    ulAudioLengthInMS      : LongInt;            // Audio data in millisecs
    ulAudioLengthInBytes   : LongInt;            // Audio data in bytes
    pAdditionalInformation : Pointer;
  end;

  mmxWav_Header = record
    WAVEHeader        : WAVE_HEADER;           // Per RIFF WAVE Definition
    XWAVHeaderInfo    : XWAV_HEADERINFO;       // Extended wave definition
  end;

  mmAudioHeader = record
    ulHeaderLength    : LongInt;                 // Length in Bytes
    ulContentType     : LongInt;                 // Image content
    ulMediaType       : LongInt;                 // Media Type
    mmXWAVHeader      : MMXWAV_HEADER;         // header
  end;
  pmmAudioHeader = ^mmAudioHeader;

Const
  MMIO_MIDI_UNKNOWN             = $00000000;  // Unknown midi content
  MMIO_MIDI_VOICE               = $00000001;  // Limited Range
  MMIO_MIDI_MUSIC               = $00000002;  // FM Radio or equivalent
  MMIO_MIDI_HIFI                = $00000004;  // High quality recording

// MMPMMMIO.INI file structure and definitions.
  CCHMAXPATH                    = 260;
  DLLNAME_SIZE                  = CCHMAXPATH;
  PROCNAME_SIZE                 = 32;
  MAX_EXTENSION_NAME            = 4;

Type
  mmIniFileInfo = record
    fccIOProc         : FourCC;                // IOProc identifier
    szDLLName         : Array[0..DLLName_Size-1] of Char;  // DLL name string
    szProcName        : Array[0..ProcName_Size-1] of Char; // Procedure name string
    ulFlags           : LongInt;                 // Flags for Preload
    ulExtendLen       : LongInt;                 // Length of ext fields
    ulMediaType       : LongInt;                 // Media type
    ulIOProcType      : LongInt;                 // Type of IOProc
    szDefExt          : Array[0..Max_Extension_Name] of Char;
  end;
  pmmIniFileInfo = ^mmIniFileInfo;

// CODEC Structures and type definitions for Rel. 1.1
Const
  CODEC_INFO_SIZE               = 8;
  CODEC_HW_NAME_SIZE            = 32;

Type
  CodecIniFileInfo = record
    ulStructLen       : LongInt;                 // length of this structure
    fcc               : FourCC;                // File Format ID
    szDLLName         : Array[0..DLLName_Size-1] of Char;       // DLL name string
    szProcName        : Array[0..ProcName_Size-1] of Char;      // Procedure name string
    ulCompressType    : LongInt;                 // Compression Type
    ulCompressSubType : LongInt;                 // Compression SubType
    ulMediaType       : LongInt;                 // Media type
    ulCapsFlags       : LongInt;                 // capabilities flags
    ulFlags           : LongInt;                 // flags
    szHWID            : Array[0..Codec_HW_Name_Size-1] of Char; // specific information
    ulMaxSrcBufLen    : LongInt;                 // max source buffer length
    ulSyncMethod      : LongInt;                 // Synchronization method
    fccPreferredFormat: LongInt;                 // Preferred output format
    ulXalignment      : LongInt;                 // x alignment - video only
    ulYalignment      : LongInt;                 // y alignment - video only
    ulSpecInfo        : Array[0..Codec_Info_Size-1] of LongInt;   // specific information
  end;
  pCodecIniFileInfo = ^CodecIniFileInfo;

Const
  // CODECINIFILEINFO synchronization method (ulSyncMethod) values.
  CODEC_SYNC_METHOD_NO_DROP_FRAMES = 0;
  CODEC_SYNC_METHOD_DROP_FRAMES_IMMEDIATELY = 1;
  CODEC_SYNC_METHOD_DROP_FRAMES_PRECEDING_KEY = 2;
  CODEC_SYNC_METHOD_HARDWARE    = 3;

  // CODECINIFILEINFO capabilities (ulCapsFlags) values.
  CODEC_COMPRESS                = $00000001;
  CODEC_DECOMPRESS              = $00000002;
  CODEC_WINDOW_CLIPPING         = $00000004;
  CODEC_PALETTE_TRANS           = $00000008;
  CODEC_SELFHEAL                = $00000010;
  CODEC_SCALE_PEL_DOUBLE        = $00000020;
  CODEC_SCALE_PEL_HALVED        = $00000040;
  CODEC_SCALE_CONTINUOUS        = $00000080;
  CODEC_MULAPERTURE             = $00000100;
  CODEC_4_BIT_COLOR             = $00000200;
  CODEC_8_BIT_COLOR             = $00000400;
  CODEC_16_BIT_COLOR            = $00000800;
  CODEC_24_BIT_COLOR            = $00001000;
  CODEC_HARDWARE                = $00002000;
  CODEC_SYMMETRIC               = $00004000;
  CODEC_ASYMMETRIC              = $00008000;
  CODEC_DIRECT_DISPLAY          = $00010000;
  CODEC_DEFAULT                 = $00020000;
  CODEC_ORIGIN_LOWERLEFT        = $00040000;
  CODEC_ORIGIN_UPPERLEFT        = $00080000;
  CODEC_SET_QUALITY             = $00100000;  // quality level is settable
  CODEC_DATA_CONSTRAINT         = $00200000;  // data constraint supported
  CODEC_HW_OVERLAY              = $00400000;
  CODEC_MULTI_BUFFER            = $00800000;
  CODEC_DITHER_OUTPUT           = $01000000;

  // Audio related flags
  CODEC_COMP_REALTIME           = $00020000;
  CODEC_DECOMP_REALTIME         = $00040000;

// CODECINIFILEINFO Flag Values (ulFlags) values.

  // Bit definitions for mmioSet()
  MMIO_SET_EXTENDEDINFO         = $0001;
  MMIO_QUERY_EXTENDEDINFO_BASE  = $0002;
  MMIO_QUERY_EXTENDEDINFO_ALL   = $0004;

Type
  // CODECASSOC structure
  CodecAssoc = record
   pCodecOpen         : Pointer;               // codec specific open header
   pCODECIniInfo      : PCodecIniFileInfo;     // codecinifileinfo
  end;
  pCodecAssoc = ^CodecAssoc;

  // MMEXTENDINFO structure
  mmExtendInfo = record
    ulStructLen       : LongInt;             // length of this structure
    ulBufSize         : LongInt;             // total buffer size
    ulFlags           : LongInt;             // flags
    ulTrackID         : LongInt;             // track ID
    ulNumCODECs       : LongInt;             // number of codec entries
    pCODECAssoc       : PCodecAssoc;       // pointer to codec info array
  end;
  pmmExtendInfo = ^mmExtendInfo;

const
  // MMEXTENDINFO operation (ulFlags) values.
  MMIO_TRACK                    = $00000001;
  MMIO_NORMAL_READ              = $00000002;
  MMIO_SCAN_READ                = $00000004;
  MMIO_REVERSE_READ             = $00000008;
  MMIO_CODEC_ASSOC              = $00000100;

  // Audio Related defines
  MMIO_REALTIME_CODEC           = $00000200;

  MMIO_RESETTRACKS              = -1;       //  Turns off the active track number.

  CODEC_START                   = $0EC0;
  CODEC_END                     = $0EFF;

  MMIOM_CODEC_CLOSE             = CODEC_START + 1;
  MMIOM_CODEC_OPEN              = CODEC_START + 2;
  MMIOM_CODEC_QUERYNAME         = CODEC_START + 3;
  MMIOM_CODEC_QUERYNAMELENGTH   = CODEC_START + 4;
  MMIOM_CODEC_COMPRESS          = CODEC_START + 5;
  MMIOM_CODEC_DECOMPRESS        = CODEC_START + 6;

  MMIOMP_CODEC_HW_CONTROL_STOP_DISCARD   =0;
  MMIOMP_CODEC_HW_CONTROL_STOP_FLUSH     =1;
  MMIOMP_CODEC_HW_CONTROL_STOP_PAUSE     =2;
  MMIOMP_CODEC_HW_CONTROL_DATATYPE       =3;

  // Flags:
  MMIO_CREATE                   = $00000001;       // Open
  MMIO_CTOCFIRST                = $00000002;       // Open
  MMIO_READ                     = $00000004;       // Open
  MMIO_WRITE                    = $00000008;       // Open
  MMIO_READWRITE                = $00000010;       // Open
  MMIO_COMPAT                   = $00000020;       // Open
  MMIO_EXCLUSIVE                = $00000040;       // Open
  MMIO_DENYWRITE                = $00000080;       // Open
  MMIO_DENYREAD                 = $00000100;       // Open
  MMIO_DENYNONE                 = $00000200;       // Open
  MMIO_ALLOCBUF                 = $00000400;       // Open
  MMIO_DELETE                   = $00000800;       // Open

  MMIO_USE_TEMP                 = $00001000;       // Open/Close/Save

  MMIO_INSERTON                 = $00000001;       // Insert
  MMIO_INSERTOFF                = $00000002;       // Insert

  MMIO_RWMODE                   = $00001000;       // Open
  MMIO_SHAREMODE                = $00002000;       // Open

  MMIO_DIRTY                    = $00004000;       // Write

  MMIO_VERTBAR                  = $00008000;       // Open
  MMIO_BUFSHARED                = $00010000;       // Open
  MMIO_APPEND                   = $00020000;       // Open
  MMIO_NOIDENTIFY               = $00040000;       // Open

  MMIO_FINDFIRST                = $00000001;       // CF Find Entry
  MMIO_FINDNEXT                 = $00000002;       // CF Find Entry
  MMIO_FINDUNUSED               = $00000004;       // CF Find Entry
  MMIO_FINDDELETED              = $00000008;       // CF Find Entry

  MMIO_CHANGEDELETED            = $0001;         // CF Change Entry

  MMIO_CF_FQNAME                = $0001;         // CF Compact

  MMIO_FHOPEN                   = $0001;           // Close

  MMIO_EMPTYBUF                 = $0001;           // Flush

  MMIO_CREATERIFF               = $0001;           // CreateChunk
  MMIO_CREATELIST               = $0002;           // CreateChunk
  MMIO_FINDCHUNK                = $0004;           // Descend
  MMIO_FINDRIFF                 = $0008;           // Descend
  MMIO_FINDLIST                 = $0010;           // Descend

  CTOC_HF_SEQUENTIAL            = $00000001;    // CTOC ulHeaderFlags
  CTOC_HF_MEDSUBTYPE            = $00000002;    // CTOC ulHeaderFlags

  CTOC_EFU_UNUSED               = $00000000; // CTOC extra usage code
  CTOC_EFU_LASTMODTIME          = $00000001; // CTOC extra usage code
  CTOC_EFU_CODEPAGE             = $00000002; // CTOC extra usage code
  CTOC_EFU_LANGUAGE             = $00000003; // CTOC extra usage code
  CTOC_EFU_COMPRESSPARAM0       = $00000005; // CTOC extra usage code
  CTOC_EFU_COMPRESSPARAM1       = $00000006; // CTOC extra usage code
  CTOC_EFU_COMPRESSPARAM2       = $00000007; // CTOC extra usage code
  CTOC_EFU_COMPRESSPARAM3       = $00000008; // CTOC extra usage code
  CTOC_EFU_COMPRESSPARAM4       = $00000009; // CTOC extra usage code
  CTOC_EFU_COMPRESSPARAM5       = $0000000A; // CTOC extra usage code
  CTOC_EFU_COMPRESSPARAM6       = $0000000B; // CTOC extra usage code
  CTOC_EFU_COMPRESSPARAM7       = $0000000C; // CTOC extra usage code
  CTOC_EFU_COMPRESSPARAM8       = $0000000D; // CTOC extra usage code
  CTOC_EFU_COMPRESSPARAM9       = $0000000E; // CTOC extra usage code
  CTOC_CharSET_STANDARD         = $00000000; // CTOC charset value

  MMIO_INSTALLPROC              = $00000001; // Install IO Proc
  MMIO_REMOVEPROC               = $00000002; // Install IO Proc
  MMIO_FINDPROC                 = $00000004; // Install IO Proc

  MMIO_MATCHFIRST               = $00000010; // Ini File Handler
  MMIO_MATCHNEXT                = $00000020; // Ini File Handler
  MMIO_MATCHFourCC              = $00000040; // Ini File Handler
  MMIO_MATCHDLL                 = $00000080; // Ini File Handler
  MMIO_MATCHPROCEDURENAME       = $00000100; // Ini File Handler
  MMIO_FULLPATH                 = $00000200; // Ini File Handler
  MMIO_NOVERIFY                 = $00000400; // Ini File Handler
  MMIO_MATCHCOMPRESSTYPE        = $00000800; // Ini File Handler
  MMIO_EXTENDED_STRUCT          = $00001000; // Ini File ulFlags
  MMIO_MATCHCOMPRESSSUBTYPE     = $00002000; // Ini File Handler
  MMIO_MATCHHWID                = $00004000; // Ini File Handler
  MMIO_MATCHCAPSFLAGS           = $00008000; // Ini File Handler
  MMIO_SKIPMATCH                = $00010000; // Ini/Load Handler

  MMIO_TOUPPER                  = $0001;      // StringToFourcc

  MMIO_CF_ENTRY_EXISTS          = $00000001; // Add CGRP element

  MMIO_FORCE_IDENTIFY_SS        = $00000001; // Identify
  MMIO_FORCE_IDENTIFY_FF        = $00000002; // Identify

  MMIO_NOTRANSLATE              = $00000000; // Translation
  MMIO_TRANSLATEDATA            = $00000001; // Translation
  MMIO_TRANSLATEHEADER          = $00000002; // Translation
  MMIO_DECOMPRESS               = $00000004; // CODEC Decompress

  MMIO_DEFAULTBUFFER            = 8192;    // two pages under OS/2 2.0

  MMIO_SEEK_IFRAME              = $00010000;  // Seek to nearest previous IFRAME



  // Messages :
  MMIOM_START                   = $0E00;
  MMIOM_END                     = $0EFF;

  MMIOM_GETCF                   = MMIOM_START + 1;
  MMIOM_GETCFENTRY              = MMIOM_START + 2;

  MMIOM_CLOSE                   = MMIOM_START + 3;
  MMIOM_OPEN                    = MMIOM_START + 4;
  MMIOM_READ                    = MMIOM_START + 5;
  MMIOM_SEEK                    = MMIOM_START + 6;
  MMIOM_WRITE                   = MMIOM_START + 7;

  MMIOM_IDENTIFYFILE            = MMIOM_START + 8;
  MMIOM_GETHEADER               = MMIOM_START + 9;
  MMIOM_SETHEADER               = MMIOM_START + 10;
  MMIOM_QUERYHEADERLENGTH       = MMIOM_START + 11;
  MMIOM_GETFORMATNAME           = MMIOM_START + 12;
  MMIOM_GETFORMATINFO           = MMIOM_START + 13;
  MMIOM_SEEKBYTIME              = MMIOM_START + 14;
  MMIOM_TEMPCHANGE              = MMIOM_START + 15;
  MMIOM_BEGININSERT             = MMIOM_START + 16;
  MMIOM_ENDINSERT               = MMIOM_START + 17;
  MMIOM_SAVE                    = MMIOM_START + 18;
  MMIOM_SET                     = MMIOM_START + 19;
  MMIOM_COMPRESS                = MMIOM_START + 20;
  MMIOM_DECOMPRESS              = MMIOM_START + 21;
  MMIOM_MULTITRACKREAD          = MMIOM_START + 22;
  MMIOM_MULTITRACKWRITE         = MMIOM_START + 23;
  MMIOM_DELETE                  = MMIOM_START + 24;
  MMIOM_BEGINGROUP              = MMIOM_START + 25;
  MMIOM_ENDGROUP                = MMIOM_START + 26;
  MMIOM_UNDO                    = MMIOM_START + 27;
  MMIOM_REDO                    = MMIOM_START + 28;
  MMIOM_BEGINSTREAM             = MMIOM_START + 29;
  MMIOM_ENDSTREAM               = MMIOM_START + 30;


  MMIOM_CUT                     = MMIOM_START + 31;
  MMIOM_COPY                    = MMIOM_START + 32;
  MMIOM_PASTE                   = MMIOM_START + 33;
  MMIOM_CLEAR                   = MMIOM_START + 34;
  MMIOM_STATUS                  = MMIOM_START + 35;
  MMIOM_WINMSG                  = MMIOM_START + 36;
  MMIOM_BEGINRECORD             = MMIOM_START + 37;
  MMIOM_ENDRECORD               = MMIOM_START + 38;

  // These 3 new messages were added with feature 11710
  MMIOM_QUERYIMAGE              = MMIOM_START + 39;
  MMIOM_QUERYIMAGECOUNT         = MMIOM_START + 40;
  MMIOM_SETIMAGE                = MMIOM_START + 41;


  MMIO_REALTIME                 = $00000001;
  MMIO_NONREALTIME              = $00000002;

  MMIOM_USER                    = $0F00;
  MMIOM_USER_END                = $0FFF;

Type
  // Parameter structure for MMIOM_STATUS
  mmIO_Status_Parms = record
    hwndWindow        : hwnd;                  // Some items require a window handle
    ulReturn          : LongInt;                 // Return field
    ulItem            : LongInt;                 // Use MCI_STATUS_... flags here
    ulValue           : LongInt;                 // Status value field
    ulType            : LongInt;                 // MCI_FORMAT_... of ulReturn
  end;
  pmmIO_Status_Parms = ^mmIO_Status_Parms;

  // Parameter structure for MMIOM_COPY, MMIOM_CUT, MMIOM_CLEAR and MMIOM_PASTE
  uSec = LongInt;                                 // microsecond time format

  mmIO_mEdit_Parms = record
    ulStrucLen        : LongInt;                 // length of this structure
    hwndWindow        : hwnd;                  // window handle
    ulStartTime       : uSec;                  // starting time in usec
    ulDuration        : uSec;                  // duration in usec
    ulCurrentFilePosition : LongInt;             // current file position in usec
    ulNewFilePosition : LongInt;                 // returned by IO proc in usec, MCD will issue a seek
    ulNewFileLength   : LongInt;                 // return by IO proc in usec, MCD updates its headers
    pBuffer           : Pointer;               // optional buffer
    ulBufferLength    : LongInt;                 // optional buffer's length
    pHeader           : Pointer;               // optional pointer to header for buffer
  end;
  pmmIO_mEdit_Parms = ^mmIO_mEdit_Parms;

  // Parameter structure for MMIOM_WINMSG
  mmIO_WinMsg = record
    hwndWindow        : hwnd;                  // these are the parameters
    usMessage         : Word;                //  ... passed to the
    pParam1           : Pointer;               //  ... window procedure
    pParam2           : Pointer;               //  ... by PM
  end;
  pmmIO_WinMsg = ^mmIO_WinMsg;

// JPEG IOproc specific structure
Const
  YUV_YVU            =$0080;  //v013 Compressed data is YUV///
  DST_Y              =20;
  DST_YY             =25;

Type
  jpegoptions = record            // this comment needed by h2inc        ///
    ulStructLen:Longint;                    // size of this sturcture                  ///
    usQuantization:Array [0..4] of word;              // Each number may be 1 - 65535      ///
    usScale:Word;                        // 1 (1/8 Size) - 8 (Full Size; default)  ///
    ulColorOrder:LongInt;                   // YUV_YVU (Default) or ~YUV_YVU     ///
    usColorSpaceOut:Word;               // DST_YY (Default) or DST_Y           ///
  end;
  pJpegOptions = ^JpegOptions;

// Include error codes for MMIO only.
Const
  MMIO_SUCCESS                  = 0;
  MMIO_WARNING                  = 2;
  MMIO_ERROR                    = -1;
  MMIOERR_UNSUPPORTED_MESSAGE   = -2;

  MMIO_CF_SUCCESS               = 0;
  MMIO_CF_FAILURE               = 1;


  MMIO_NLS_CharSET_INFO         = 8000;   // RCDATA Name ID for NLS
  MMIO_IOPROC_NAME_TABLE        = 8500;   // RCDATA Name ID for string table
  MMIO_CODEC_NAME_TABLE         = 9000;   // RCDATA Name ID for Codec  table

// Numeric equivalents of fourcc's.  These are needed for the resource
// compiler.

  HEX_FourCC_DOS                = $20534f44;
  HEX_FourCC_MEM                = $204d454d;
  HEX_FourCC_BND                = $20444e42;
  HEX_FourCC_CF                 = $20204643;

// Country codes (CC), languages (LC), and dialects (DC).

  MMIO_DEFAULT_CODE_PAGE        = 437;

  MMIO_CC_NONE                  = 000;
  MMIO_CC_USA                   = 001;
  MMIO_CC_CANADA                = 002;
  MMIO_CC_LATIN_AMERICA         = 003;
  MMIO_CC_GREECE                = 030;
  MMIO_CC_NETHERLANDS           = 031;
  MMIO_CC_BELGIUM               = 032;
  MMIO_CC_FRANCE                = 033;
  MMIO_CC_SPAIN                 = 034;
  MMIO_CC_ITALY                 = 039;
  MMIO_CC_SWITZERLAND           = 041;
  MMIO_CC_AUSTRIA               = 043;
  MMIO_CC_UNITED_KINGDOM        = 044;
  MMIO_CC_DENMARK               = 045;
  MMIO_CC_SWEDEN                = 046;
  MMIO_CC_NORWAY                = 047;
  MMIO_CC_WEST_GERMANY          = 049;
  MMIO_CC_MEXICO                = 052;
  MMIO_CC_BRAZIL                = 055;
  MMIO_CC_AUSTRALIA             = 061;
  MMIO_CC_NEW_ZEALAND           = 064;
  MMIO_CC_JAPAN                 = 081;
  MMIO_CC_KOREA                 = 082;
  MMIO_CC_CHINA                 = 086;
  MMIO_CC_TAIWAN                = 088;
  MMIO_CC_TURKEY                = 090;
  MMIO_CC_PORTUGAL              = 351;
  MMIO_CC_LUXEMBOURG            = 352;
  MMIO_CC_ICELAND               = 354;
  MMIO_CC_FINLAND               = 358;

  MMIO_LC_NONE                  = 0;
  MMIO_DC_NONE                  = 0;
  MMIO_LC_ARABIC                = 1;
  MMIO_DC_ARABIC                = 1;
  MMIO_LC_BULGARIAN             = 2;
  MMIO_DC_BULGARIAN             = 1;
  MMIO_LC_CATALAN               = 3;
  MMIO_DC_CATALAN               = 1;
  MMIO_LC_TRADITIONAL_CHINESE   = 4;
  MMIO_DC_TRADITIONAL_CHINESE   = 1;
  MMIO_LC_SIMPLE_CHINESE        = 4;
  MMIO_DC_SIMPLE_CHINESE        = 2;
  MMIO_LC_CZECH                 = 5;
  MMIO_DC_CZECH                 = 1;
  MMIO_LC_DANISH                = 6;
  MMIO_DC_DANISH                = 1;
  MMIO_LC_GERMAN                = 7;
  MMIO_DC_GERMAN                = 1;
  MMIO_LC_SWISS_GERMAN          = 7;
  MMIO_DC_SWISS_GERMAN          = 2;
  MMIO_LC_GREEK                 = 8;
  MMIO_DC_GREEK                 = 1;
  MMIO_LC_US_ENGLISH            = 9;
  MMIO_DC_US_ENGLISH            = 1;
  MMIO_LC_UK_ENGLISH            = 9;
  MMIO_DC_UK_ENGLISH            = 2;
  MMIO_LC_SPANISH               = 10;
  MMIO_DC_SPANISH               = 1;
  MMIO_LC_SPANISH_MEXICAN       = 10;
  MMIO_DC_SPANISH_MEXICAN       = 2;
  MMIO_LC_FINNISH               = 11;
  MMIO_DC_FINNISH               = 1;
  MMIO_LC_FRENCH                = 12;
  MMIO_DC_FRENCH                = 1;
  MMIO_LC_BELGIAN_FRENCH        = 12;
  MMIO_DC_BELGIAN_FRENCH        = 2;
  MMIO_LC_CANADIAN_FRENCH       = 12;
  MMIO_DC_CANADIAN_FRENCH       = 3;
  MMIO_LC_SWISS_FRENCH          = 12;
  MMIO_DC_SWISS_FRENCH          = 4;
  MMIO_LC_HEBREW                = 13;
  MMIO_DC_HEBREW                = 1;
  MMIO_LC_HUNGARIAN             = 14;
  MMIO_DC_HUNGARIAN             = 1;
  MMIO_LC_ICELANDIC             = 15;
  MMIO_DC_ICELANDIC             = 1;
  MMIO_LC_ITALIAN               = 16;
  MMIO_DC_ITALIAN               = 1;
  MMIO_LC_SWISS_ITALIAN         = 16;
  MMIO_DC_SWISS_ITALIAN         = 2;
  MMIO_LC_JAPANESE              = 17;
  MMIO_DC_JAPANESE              = 1;
  MMIO_LC_KOREAN                = 18;
  MMIO_DC_KOREAN                = 1;
  MMIO_LC_DUTCH                 = 19;
  MMIO_DC_DUTCH                 = 1;
  MMIO_LC_BELGIAN_DUTCH         = 19;
  MMIO_DC_BELGIAN_DUTCH         = 2;
  MMIO_LC_NORWEGIAN_BOKMAL      = 20;
  MMIO_DC_NORWEGIAN_BOKMAL      = 1;
  MMIO_LC_NORWEGIAN_NYNORSK     = 20;
  MMIO_DC_NORWEGIAN_NYNORSK     = 2;
  MMIO_LC_POLISH                = 21;
  MMIO_DC_POLISH                = 1;
  MMIO_LC_BRAZILIAN_PORTUGUESE  = 22;
  MMIO_DC_BRAZILIAN_PORTUGUESE  = 1;
  MMIO_LC_PORTUGUESE            = 22;
  MMIO_DC_PORTUGUESE            = 2;
  MMIO_LC_RHAETO_ROMANIC        = 23;
  MMIO_DC_RHAETO_ROMANIC        = 1;
  MMIO_LC_ROMANIAN              = 24;
  MMIO_DC_ROMANIAN              = 1;
  MMIO_LC_RUSSIAN               = 25;
  MMIO_DC_RUSSIAN               = 1;
  MMIO_LC_SERBO_CROATIAN_LATIN  = 26;
  MMIO_DC_SERBO_CROATIAN_LATIN  = 1;
  MMIO_LC_SERBO_CROATIAN_CYRILLIC = 26;
  MMIO_DC_SERBO_CROATIAN_CYRILLIC = 2;
  MMIO_LC_SLOVAK                = 27;
  MMIO_DC_SLOVAK                = 1;
  MMIO_LC_ALBANIAN              = 28;
  MMIO_DC_ALBANIAN              = 1;
  MMIO_LC_SWEDISH               = 29;
  MMIO_DC_SWEDISH               = 1;
  MMIO_LC_THAI                  = 30;
  MMIO_DC_THAI                  = 1;
  MMIO_LC_TURKISH               = 31;
  MMIO_DC_TURKISH               = 1;
  MMIO_LC_URDU                  = 32;
  MMIO_DC_URDU                  = 1;
  MMIO_LC_BAHASA                = 33;
  MMIO_DC_BAHASA                = 1;


// Ultimotion CODEC type for CODECINIFILEINFO ulCompressType */
//#define  FOURCC_ULTI       mmioFOURCC('U', 'L', 'T', 'I')
Const
//  FOURCC_ULTI:FOURCC=0;
  HEX_FOURCC_ULTI=$49544C55;     // ITLU */

// Indeo CODEC type for CODECINIFILEINFO ulCompressType */
//#define  FOURCC_RT21       mmioFOURCC('R', 'T', '2', '1')
//  FOURCC_RT21:FOURCC=0;
  HEX_FOURCC_RT21=$31325452;     // 12TR */

// Mondo CODEC type for CODECINIFILEINFO ulCompressType */
//#define  FOURCC_DIB        mmioFOURCC('D', 'I', 'B', ' ')
//   FOURCC_DIB:FOURCC=0;
   HEX_FOURCC_DIB=$20424944;     //  BID */



// CODECVIDEOHEADER - CODEC video Header
Type
  TCODECVIDEOHEADER=record  // codecvidhdr */
    ulStructLen:LongInt;
    cx:LongInt;
    cy:LongInt;
    cPlanes:Integer;
    cBitCount:Integer;
    ulColorEncoding:LongInt;
    genpal:GENPAL;
  end;
  PCodecVideoHeader=^TCODECVIDEOHEADER;

// ulColorEncoding defines: */
Const
  MMIO_RGB_5_6_5    =$0001;  // Each pixel is a RGB_5_6_5 datatype */
  MMIO_RGB_24       =$0002;  // Each pixel is a RGB_24 datatype */
  MMIO_YUV_4_1_1    =$0004;  // Each pixel is a YUV_4_1_1 datatype */
  MMIO_COMPRESSED   =$0008;  // The data is compressed */
  MMIO_YUV_24       =$0010;  // Each pixel is a YUV_24 datatype */
  MMIO_PALETTIZED   =$0020;  // The data is palettized */
  MMIO_OS2_BITMAP24 =$0020;  // The data is palettized */


//*********************************************
// *
// * MMVIDEOOPEN - Video Open Structure
// *
// * This structure is passed on the CODEC open
// * message when video compression is being done
// * to indicate information such as quality,
// * frame rate, data rate, and key frame rate.
// *
// * Quality:
// *
// * The ulQuality field specifies a scalar value
// * in the range 0 - 10000, where 0 is the lowest
// * quality and 10000 is the highest quality.  A
// * value of -1 specifies the default quality level,
// * and the default quality level (e.g. 5000) is
// * returned in the ulQuality field.
// *
// *
// * Key Frame rate:
//*
// * The ulKeyFrameRate structure specifies the key
// * frame (aka I-frame, reference frame) frequency.
// * Every Nth frame is a key frame as specified.
// * A value of zero specifies that no periodic key
// * are to be compressed.  Additional key frames may
// * be inserted at any point by specifying
// * MMIO_IS_KEY_FRAME in the MMCOMPRESS structure.
// *
// * example:  ulKeyFrameRate = 5  results in:
//*
//*    key delta delta delta delta key delta delta delta delta key delta...
// *
//*
// * Frame rate:
// *
// * Rate = number of time units per second
// * Scale = number of time units per frame
// *
// * examples:  Rate = 30  Scale = 1     =>    30 FPS
// *            Rate = 15  Scale = 1     =>    15 FPS
// *            Rate = 25  Scale = 2     =>    12.5 FPS
// *
// *
// * Data Constraint:
// *
// * Compressors which are capable of constraining the
// * resultant compressed video data rate use the
// * information in the ulDataConstraint and
// * ulConstraintInterval fields.  A non-zero value
//* in ulDataConstraint specifies the number of bytes
// * which is not to be exceeded over an interval of
// * frames in the output data stream, regardless of
// * the requested quality level.  This value only
// * considers video data, i.e. audio data and file format
// * overhead must be considered seperately when determining
// * the final output file data rate.  The interval of
// * frames over which the data is constrained is specified
//* in ulConstraintInterval.  A value of zero for
// * ulDataContraint specifies that the data rate is not
// * to be constrained and is compressed according to
// * the requested quality level.
// *
// * example 1:  ulDataConstraint = 150000   ulConstraintInterval = 15
// *
// *             This results in an output stream wherein the sizes of any 15
// *             consecutive frames does not exceed 150000 bytes.  If the
// *             frame rate is 15 FPS, the resultant data rate will not
// *             exceed 150000 bytes per second.
// *
// * example 2:  ulDataConstraint = 10000    ulConstraintInterval = 1
// *
// *             This results in an output stream wherein any single frame
// *             does not exceed 10000 bytes.  If the frame rate is 15 FPS,
// *             the resultant data rate will not exceed 150000 bytes per
// *             second.  Note the difference between this case and example 1
// *             where individual frames may exceed 10000 bytes (the average)
// *             so long other frames in any 15 frame sequence are sufficiently
// *             smaller to satisfy the constraint within the constraint interval.
// *
// **********************************************/
TYPE _MMVIDEOOPEN = RECORD      // mmvidopen */
       ulStructLen:LongInt;
       ulQuality:LongInt;
       ulKeyFrameRate:LongInt;
       ulScale:LongInt;
       ulRate:LongInt;
       ulDataConstraint:LongInt;
       ulConstraintInterval:LongInt;
       end;
TYPE PMMVIDEOOPEN = ^_MMVIDEOOPEN;


TYPE _MMAUDIOOPEN = RECORD
  ulStructLen:LongInt;         // Length of struct */
  ulSamplesPerBlock:LongInt;   // Samples in each block of compressed data */
  ulBytesPerBlock:LongInt;     // uncompressed bytes in each block */
  ulFlags:LongInt;             // Compression flags */
  ulBestGuess:LongInt;         // Guess at avg. compression ratio */
  ulBlockAlignment:LongInt;    // Block alignment of codec */
  ulLength:LongInt;            // Length of the file */
  hCodec:LongInt;              // Codec handle */
  pfnCodec:PCodecProc;
  end;

TYPE PMMAUDIOOPEN = ^_MMAUDIOOPEN;

// defines for the ulFlags field of the BUFER_INFORMATION */

CONST

        BLOCK_ORIENTED        =$00000001;
        NON_LINEAR            =$00000002;
        INIT_CODEC            =$00000004;

//*********************************************
// *
// * CODECOPEN - CODEC open structure
// *
// **********************************************/
TYPE _CODECOPEN = RECORD       // codecopen */
   ulFlags:LongInt;             // flags & events - Refer to ulCapsFlags in CODECINIFILEINFO */
   pControlHdr:Pointer;         // control header - (codec specific) */
   pSrcHdr:Pointer;             // source header - Ptr CODECVIDEOHEADER */
   pDstHdr:Pointer;             // destination header - Ptr CODECVIDEOHEADER */
   pOtherInfo:Pointer;          // other information - Ptr MMVIDEOOPEN/MMAUDIOOPEN */
   end;
TYPE PCODECOPEN = ^_CODECOPEN;

const
  // CODECINIFILEINFO capabilities (ulCapsFlags) values.

  Valid_CodecOpen_InputFlags = CODEC_DECOMPRESS or
                               CODEC_WINDOW_CLIPPING or
                               CODEC_PALETTE_TRANS or
                               CODEC_SELFHEAL or
                               CODEC_SCALE_PEL_DOUBLE or
                               CODEC_SCALE_PEL_HALVED or
                               CODEC_SCALE_CONTINUOUS or
                               CODEC_MULAPERTURE or
                               CODEC_HARDWARE or
                               CODEC_DIRECT_DISPLAY;
// Stream handler communication */

TYPE _AUDIO_CODEC_INFO = RECORD
  ulStructLen:LongInt;         // Length of struct */
  ulBytesPerBlock:LongInt;     // uncompressed bytes in each block */
  ulBlockAlignment:LongInt;    // Block alignment of codec */
  hCodec:LongInt;              // Codec handle */
  pfnCodec:PCODECPROC;
//  LONG (* APIENTRY pfnCodec) (PVOID, SHORT, LONG, LONG); */
  end;

CONST
        AUDIO_CODEC_INF    =1000;


//*********************************************
// *
// * MMCOMPRESS - Compress structure
// *
// **********************************************/
TYPE _MMCOMPRESS = RECORD    // mmcomp */
   ulStructLen:LongInt;       // length of this structure */
   ulFlags:LongInt;           // command and status flags */
   ulSrcBufLen:LongInt;       // source buffer size */
   pSrcBuf:Pointer;           // source buffer */
   ulDstBufLen:LongInt;       // destination buffer length */
   pDstBuf:Pointer;           // destination buffer */
   pRunTimeInfo:Pointer;      // control information */
   end;
TYPE PMMCOMPRESS = ^_MMCOMPRESS;

// ulFlags Input values for MMCOMPRESS structure:                */
// Note:  MMIO_IS_KEY_FRAME and MMIO_IS_PALETTE are defined      */
// below, but are listed here for information purposes only.     */
// MMIO_IS_KEY_FRAME         This bit is set by the application  */
//                           to instruct the IOProc to compress  */
//                           the pSrcBuf into a key or reference */
//                           frame.  If the bit is not set, a    */
//                           delta frame is compressed.          */
// MMIO_IS_PALETTE           A video palette is provided.  This  */
//                           is set by the application.          */


//*********************************************
// *
// * MMVIDEOCOMPRESS - Video Compress structure
// *
// **********************************************/

TYPE _MMVIDEOCOMPRESS = RECORD // mmvidcomp */
   ulStructLen:LongInt;       // Structure length */
   genpalVideo:GENPAL;       // Video stream palette */
   pControlHdr:Pointer;       // control header (codec specific) */
   end;
TYPE PMMVIDEOCOMPRESS = ^_MMVIDEOCOMPRESS;

CONST

START_DECOMPRESSION     =$00000001;
CONTINUE_DECOMPRESSION  =$00000002;
START_SEEK              =$00000004;
CONTINUE_SEEK           =$00000008;

//*********************************************
// *
// * MMDECOMPRESS - Decompress Structure
// *
// **********************************************/
TYPE _MMDECOMPRESS = Record   // mmdec */
   ulStructLen:LongInt;       // length of this structure */
   ulFlags:LongInt;           // command and status flags */
   ulSrcBufLen:LongInt;       // source buffer size */
   pSrcBuf:Pointer;           // source buffer */
   ulDstBufLen:LongInt;       // destination buffer length */
   pDstBuf:pointer;           // destination buffer */
   pRunTimeInfo:Pointer;      // control information Ptr to MMVIDEODECOMPRESS */
   end;
TYPE PMMDECOMPRESS = ^_MMDECOMPRESS;

// ulFlags defines: */
CONST

        MMIO_DROP_DELTA_FRAME =$0001; // Input/Output - Tells the IOProc to drop the delta */
                                      // frame if the pSrcBuf contains a delta */
                                      // frame.  On return, the bit is reset */
                                      // if the delta frame is dropped. */
        MMIO_IS_KEY_FRAME     =$0002; // Output - This bit is set by the IOProc when */
                                      // the data contained in the pSrcBuf is */
                                      // a key or reference frame. */
        MMIO_IS_PALETTE       =$0004; // Output - A video palette has been found. */
                                      // This is set by the IOProc. */
        MMIO_PALETTE_CHANGE   =$0008; // Input - The physical palette has been changed */
                                      // in...  This is set by the application. */
        MMIO_ORIGIN_LOWERLEFT =$0010; // Input - The video frame origin */
        MMIO_RECTL_CHANGE     =$0020; // Input - The valid rectl list has changed. */
        MMIO_ORIGIN_UPPERLEFT =$0040; // Input - The video frame origin */
        MMIO_DROP_FRAME_DECODE=$0080; // Input - Tells the IOProc to drop decoding  */
        MMIO_HIGH_QUALITY     =$0100; // Input - Tells Codec to render best */
                                      // quality image - not time critical */
        MMIO_IGNORE_CLIPPING  =$0200; // Ignore clipping rectangles used for bitmap capture */
                                      // high performance */
        MMIO_OUTPUT_FULL_IMAGE=$0400; // Output a complete image on decompress, even if this*/
                                      // is a delta frame */
                                      // of the frame.                         */

        VALID_DECOMPRESS_INPUTFLAGS   = MMIO_DROP_DELTA_FRAME or
                                        MMIO_PALETTE_CHANGE or
                                        MMIO_ORIGIN_LOWERLEFT or
                                        MMIO_RECTL_CHANGE or
                                        MMIO_DROP_FRAME_DECODE or
                                        MMIO_ORIGIN_UPPERLEFT or
                                        MMIO_HIGH_QUALITY or
                                        MMIO_IGNORE_CLIPPING or
                                        MMIO_OUTPUT_FULL_IMAGE;


        START_COMPRESSION     =$00000001;
        CONTINUE_COMPRESSION  =$00000002;
        SOURCE_UNUSED         =$00000004;
        TARGET_UNUSED         =$00000008;

//*********************************************
// *
// * MMVIDEODECOMPRESS - Video Decompress structure
// *
// **********************************************/
TYPE  RectL   = record
    xLeft:   Longint;
    yBottom: Longint;
    xRight:  Longint;
    yTop:    Longint;
  end;
TYPE PRECTL = ^RECTL;

TYPE _MMVIDEODECOMPRESS = Record    // mmviddec */
   ulStructLen:LongInt;            // Structure length */
   ulRectlCount:LongInt;           // Valid rectangle count - for clipping */
   prectl:PRECTL;                  // Valid rectangle array - for clipping */
   ulSkipLength:LongInt;           // Skipped line length */
   ulDecodeLines:LongInt;          // Num of lines to decompress */
   genpalPhysical:GENPAL;          // Physical palette */
   genpalVideo:GENPAL;             // Video stream palette */
   rectlSrc:RECTL;                 // Source window rectangle */
   rectlDst:RECTL;                 // Destination window rectangle */
   ulDeltaCount:LongInt;           // Number of remaining delta frames before the next I-Frame */
   ulParm1:LongInt;                // Codec specific parm */
   ulParm2:Longint;                // Codec specific parm */
   ulParm3:LongInt;                // Codec specific parm */
   ulParm4:LongInt;                // Codec specific parm */
   end;
TYPE PMMVIDEODECOMPRESS = ^_MMVIDEODECOMPRESS;

//************************************************
// *
// * RECORDTAB - Record table
// *
// * NOTE: This structure maps to ESRCBUFTAB in ssm.h
// *************************************************/
TYPE _RECORDTAB = record       // recordtab */
   ulReserved1:LongInt;       // reserved for system */
   pRecord:Pointer;           // ptr to record in buffer */
   ulLength:LongInt;          // length of record */
   ulReserved2:LongInt;       // reserved for system */
   ulReserved3:LongInt;       // reserved for system */
   ulParm1:LongInt;           // Record specific data */
   ulParm2:LongInt;           // Record specific data */
   end;
TYPE PRECORDTAB=^_RECORDTAB;  // Ptr to a buffer entry  */


//**************************************************
// *
// * RECORDTABWRITE - Record table for video write
// *
// * NOTE: This structure maps to ETGTBUFTAB in ssm.h
// ***************************************************/
TYPE _RECORDTABWRITE=RECORD       // recordtab */
   pRecord:Pointer;           // ptr to record in buffer */
   ulReserved1:Longint;       // reserved for system */
   ulLength:Longint;          // length of record */
   ulReserved2:longint;       // reserved for system */
   ulReserved3:longint;       // reserved for system */
   ulParm1:longint;           // Record specific data */
   ulParm2:longint;           // Record specific data */
   end;
type precordtabwrite=^_recordtabwrite;// Ptr to a buffer entry  */


// ulParm1 Return values for MULTITRACK_READ only:               */
// Note:  MMIO_IS_KEY_FRAME and MMIO_IS_PALETTE are defined      */
// above, but are listed here for information purposes only      */
// as they are valid ulParm1 Return values for MULTITRACK_READ.  */
// MMIO_IS_KEY_FRAME         Frame is a Key frame                */
// MMIO_IS_PALETTE           Buffer contains a video palette     */

CONST

        MMIO_INVISIBLE_FRAME   =$1000;    // Indicates a invisible video frame */
        MMIO_NULL_FRAME        =$2000;    // Indicates a null video frame (zero length) */

// ulParm2 Return values for MULTITRACK_READ only:               */
//    This field contains the frame number for this video frame  */
//    if this track is a video track.                            */



// ulParm1 Input values for MULTITRACK_WRITE only:               */
// Note:  MMIO_IS_KEY_FRAME and MMIO_IS_PALETTE are defined      */
// above, but are listed here for information purposes only      */
// as they are valid ulParm1 Input values for MULTITRACK_WRITE.  */
// MMIO_IS_KEY_FRAME         Frame is a Key frame                */
// MMIO_IS_PALETTE           Buffer contains a video palette     */

// ulParm2 Input values for MULTITRACK_WRITE only:               */
//    This field contains the number of null frames              */
//    that should be inserted before this frame                  */
//    (this recordtab entry).                                    */


//***********************************************
// *
// *  TRACKMAP - This structure maps a track to
// *             a record table.
// *
// ************************************************/
TYPE _TRACKMAP = RECORD        // trackmap */
   ulTrackID:LongInt;         // Input - track ID */
   ulNumEntries:LongInt;      // Input - number of record entries */
   pRecordTabList:PRecordTab; // Input/Output - Ptr to a record table */
   end;
TYPE PTRACKMAP=^_TRACKMAP;    // Ptr to a track map table entry */

//*********************************************
// *
// * MMMULTITRACKREAD - Multiple Track Read
// *
// **********************************************/
TYPE _MMMULTITRACKREAD=RECORD   // mtread */
   ulLength:LongInt;          // Input - Size of buffer to read.  The IO should be performed on this size of  */
                              //          buffer.  The actual buffer size may be bigger and is given in the   */
                              //          ulBufferLength field below.  Video frames can span pBuffer+ulLength */
                              //          as long as the frame is less than the ulBufferLength in size.       */
                              //          The purpose of this is to break the IO reads into smaller sizes     */
                              //          while still allowing large frame sizes.                             */
   pBuffer:Pointer;           // Input - ptr to read buffer           */
   ulFlags:LongInt;           // Input/Output - read flags            */
   ulNumTracks:LongInt;       // Input - number of track entries      */
   pTrackMapList:PTRACKMAP;   // Input - ptr to track-to-record list  */
// End of old MMMULTITRACKREAD structure */
   ulBufferLength:LongInt;    // Input - Actual length of read buffer */
   ulReserved:longInt;        // Input - Reserved (must be 0)         */
   end;
TYPE PMMMULTITRACKREAD=^_MMMULTITRACKREAD;

// ulFlags Input Values: */

CONST

        MULTITRACKREAD_EXTENDED       =$0004; // Indicates that the new extended multitrack   */
                                              // read structure is passed from caller instead */
                                              // of the previous multitrack read structure.   */

// ulFlags Return Values: */
        MULTITRACKREAD_NOTDONE        =$0001; // Read is not done.  Another read of the same        */
                                              // buffer is necessary.  There were not enough record */
                                              // entries in the record table passed to this api.    */
        MULTITRACKREAD_EOF            =$0002; // End of File.  Used because # bytes read may not    */
                                              // match the length of the buffer in cases of a       */
                                              // record that spans in the next buffer.              */


//*********************************************
// *
// * MMMULTITRACKWRITE - Multiple Track Write
// *
// **********************************************/
TYPE _MMMULTITRACKWRITE=RECORD   // mtwrite */
   ulNumTracks:LongInt;       // Input - number of track entries     */
   pTrackMapList:PTRACKMAP;     // Input - ptr to track-to-record list */
   ulFlags:LongInt;           // Input - write flags (Default = 0)   */
   ulReserved:LongInt;        // Input - Reserved (must be 0)        */
   end;
TYPE PMMMULTITRACKWRITE=^_MMMULTITRACKWRITE;

// ulFlags Input Values: */

CONST

        MULTITRACKWRITE_MERGE        =$0001; // Attempt to interleave the data on the write. */
                                             // The default (without this flag set) is to    */
                                             // write all records for each track then write  */
                                             // all records of the next track and so on.     */


//*********************************************
// *
// * MMMOVIEHEADER - standard movie header data
// *
// **********************************************/
TYPE  MMTrackInfo = record
    ulTrackID     : Longint;              // track identifier
    ulMediaType   : Longint;              // media type
    ulCountry     : Longint;              // country code for the track
    ulCodePage    : Longint;              // country code page for the track
    ulReserved1   : Longint;              // reserved must be 0
    ulReserved2   : Longint;              // reserved must be 0
  end;
//TYPE  pMMTrackInfo = ^MMTrackInfo;

//TYPE
//        PSZ = PChar;

TYPE _MMMOVIEHEADER=RECORD   // mmhdr */
   ulStructLen:LongInt;       // length of this structure */
   ulContentType:LongInt;     // movie content type */
   ulMediaType:LongInt;       // video media type */
   ulMovieCapsFlags:LongInt;  // capabilities */
   ulMaxBytesPerSec:LongInt;  // maximum transfer rate */
   ulPaddingGranularity:LongInt;// pad to a multiple of this size */
   ulSuggestedBufferSize:LongInt;
   ulStart:LongInt;           // delay time marking beginning or start of movie */
   ulLength:LongInt;
   ulNextTrackID:LongInt;     // next available track id */
   ulNumEntries:LongInt;      // number of track entries */
   pmmTrackInfoList:PMMTRACKINFO;  // track information */
   pszMovieTitle:PSZ;     // movie title */
   ulCountry:LongInt;         // country code for the title string */
   ulCodePage:LongInt;        // country code page the title string */
   ulAvgBytesPerSec:LongInt;  // average transfer rate */
   end;
TYPE PMMMOVIEHEADER=^_MMMOVIEHEADER;

// ulMovieCapsFlags Defines: */
CONST

        MOVIE_HAS_VIDEO         =$0001;  // The movie contains video. */
        MOVIE_HAS_AUDIO         =$0002;  // The movie contains audio. */
        MOVIE_CAN_SEEK          =$0004;  // The movie can seek. */
        MOVIE_CAN_SCAN          =$0008;  // The movie can fast scan. */
        MOVIE_HAS_COPYRIGHT     =$0010;  // The movie contains copyrighted data. */
        MOVIE_WAS_CAPTUREFILE   =$0020;  // The movie is a specially allocated  */
                                          // file used for capturing real-time */
                                          // video.  Applications should warn  */
                                          // the user before writing over a file */
                                          // with this flag set because the user  */
                                          // probably defragmented this file. */
                                          // If this flag is set, then there is a chance */
                                          // that not all of the records will be written */
                                          // on the call. Caller must check for this whether */
                                          // this flag is set or not. */

//*********************************************
// *
// * MMVIDEOHEADER - Movie Video Track Header
// *
// **********************************************/

TYPE _MMVIDEOHEADER=RECORD   // mmvhdr */
   ulStructLen:LongInt;       // length of this structure */
   ulContentType:LongInt;     // video content type */
   ulMediaType:LongInt;       // video media type */
   ulVideoCapsFlags:LongInt;  // capabilities */
   ulWidth:LongInt;           // video width in pels */
   ulHeight:LongInt;          // video height in pels */
   ulScale:LongInt;
   ulRate:LongInt;            // Rate / Scale == frames/second */
   ulStart:LongInt;           // delay time marking beginning or start of stream */
   ulLength:LongInt;
   ulTotalFrames:LongInt;     // total number of video frames */
   ulInitialFrames:LongInt;
   mmtimePerFrame:MMTIME;    // frame display time or 0L */
   ulSuggestedBufferSize:LongInt;
   genpalVideo:GENPAL;       // palette */
   pmmXDIBHeader:PMMXDIBHEADER;     // windows DIB compatible header */
   ulHHRWidth:LongInt;        // Actual width of HHR video     */
   fHHR:Boolean;              // HHR flag                      */
   end;
TYPE PMMVIDEOHEADER=_MMVIDEOHEADER;

// ulContentType Defines: */
CONST

        MMIO_VIDEO_UNKNOWN          =$00000000;  // Unknown video content */
        MMIO_VIDEO_DATA             =$00000001;  // Video                 */


//
// Base function prototypes:
///

function mmioAdvance( mmIO: hmmio; Info: pmmioinfo; usFlags: Word ): Word; cdecl;
function mmioAscend( mmIO: hmmio; pckinfo: pmmCkInfo; usFlags: Word ): Word; cdecl;
function mmioClose( mmIO: hmmio; usFlags: Word ): Word; cdecl;
function mmioCreateChunk( mmio: hmmio; pckinfo: pmmCkInfo; usFlags: Word ): Word; cdecl;
function mmioDescend( mmIO: hmmio; pckinfo, pckinfoParent: pMMCkInfo; usFlags: Word ): Word; cdecl;
function mmioFlush( mmIO: hmmio; usFlags: Word ): Word; cdecl;
function mmioGetInfo( mmIO: hmmio; Info: pmmioinfo; usFlags: Word ): Word; cdecl;
function mmioGetLastError( mmIO: hmmio ): Longint; cdecl;
function mmioInstallIOProc( fccIOProc: FourCC; pIOProc: pMMIOProc; ulFlags: Longint ): pmmIOProc; cdecl;
function mmioOpen( pszFileName: pChar; mmIOInfo: pmmioinfo; ulOpenFlags: Longint ): hMMIO; cdecl;
function mmioRead( mmIO: hmmio; pchBuffer: pChar; cBytes: Longint ): Longint; cdecl;
function mmioSeek( mmIO: hmmio; lOffset, lOrigin: Longint ): Longint; cdecl;
function mmioSendMessage( mmIO: hmmio; usMsg: Word; lParam1, lParam2: Longint ): Longint; cdecl;
function mmioSetBuffer( mmIO: hmmio; pchBuffer: pChar; cBytes: Longint; usFlags: Word ): Word; cdecl;
function mmioSetInfo( mmIO: hmmio; mmIOInfo: pmmioinfo; usFlags: Word ): Word; cdecl;
function mmioStringToFourCC( pszString: pChar; usFlags: Word ): FourCC; cdecl;
function mmioWrite( mmIO: hmmio; pchBuffer: pChar; cBytes: Longint ): Longint; cdecl;

//
// Compound File function prototypes:
///

function mmioCFOpen( pszFileName: pChar; CfInfo, IOInfo: pmmcfinfo; ulFlags: Longint ): hMMCF; cdecl;
function mmioCFClose( mmCf: hmmcf; ulFlags: Longint ): Longint; cdecl;
function mmioCFGetInfo( mmCf: hmmcf; CfInfo: pmmcfinfo; cBytes: Longint ): Longint; cdecl;
function mmioCFSetInfo( mmCf: hmmcf; CfInfo: pmmcfinfo; cBytes: Longint ): Longint; cdecl;
function mmioCFFindEntry( mmCf: hmmcf; ctocEntry: mmctocentry; ulFlags: Longint ): Longint; cdecl;
function mmioCFAddEntry( mmCf: hmmcf; cTocEntry: mmctocentry; ulFlags: Longint ): Longint; cdecl;
function mmioCFChangeEntry( mmCf: hmmcf; CTocEntry: pmmctocentry; ulFlags: Longint ): Longint; cdecl;
function mmioCFDeleteEntry( mmCf: hmmcf; CTocEntry: pmmctocentry; ulFlags: Longint ): Longint; cdecl;
function mmioCFAddElement( mmCf: hmmcf; pszElementName: pChar; fccType: FourCC;
  pchBuffer: pChar; cchBytes: LongInt; ulFlags: Longint ): Longint; cdecl;

function mmioCFCopy( mmCfSource: hmmcf; pszDestFileName: pChar; ulFlags: Longint ): Longint; cdecl;
//
// Conversion Utility function prototypes:
///

function mmioQueryFormatCount( FormatInfo: pmmformatinfo;
  plNumFormats: pLong; ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;

function mmioGetFormats( FormatInfo: pmmformatinfo;
  lNumFormats: LongInt; pFormatInfoList: Pointer; plFormatsRead: pLong;
  ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;

function mmioGetFormatName( FormatInfo: pmmformatinfo; pszFormatName: pChar;
  plBytesRead: pLong; ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;

function mmioIdentifyFile( pszFileName: pChar; MMIoInfo: pmmioinfo;
  FormatInfo: pmmformatinfo; pfccStorageSystem: pFourCC; ulReserved: Longint;
  ulFlags: Longint ): Longint; cdecl;

function mmioQueryHeaderLength( mmIO: hmmio; plHeaderLength: pLong;
  ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;

function mmioGetHeader( mmIO: hmmio; pHeader: Pointer; lHeaderLength: LongInt;
  plBytesRead: pLong; ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;

function mmioSetHeader( mmIO: hmmio; pHeader: Pointer; lHeaderLength: LongInt;
  plBytesWritten: pLong; ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;

function mmioIniFileHandler( IniFileInfo: pmminifileinfo; ulFlags: Longint ): Longint; cdecl;

function mmioIdentifyStorageSystem( pszFileName: pChar;
  MMIoInfo: pmmioinfo; pfccStorageSystem: pFourCC ): Longint; cdecl;

function mmioDetermineSSIOProc( pszFileName: pChar; MMIoInfo: pmmioinfo;
  pfccStorageSystem: pFourCC; pszParsedRemainder: pChar ): Longint; cdecl;

function mmioQueryIOProcModuleHandle( IOProc: pMMIOProc;
  IOProcModule: phModule ): Longint; cdecl;

function mmioCFCompact( pszFileName: pChar; ulFlags: Longint ): Longint; cdecl;

//
// MMPMMMIO.INI file migration utility
///

function mmioMigrateIniFile( ulFlags: Longint ): Longint; cdecl;

//
// MMIO CODEC APIs
///

function mmioIniFileCODEC( IniFile: pCODECIniFileInfo; ulFlags: Longint ): Longint; cdecl;
function mmioSet( mmIO: hmmio; ExtendInfo: pmmExtendInfo; ulFlags: Longint): Longint; cdecl;
function mmioQueryCODECName( IniInfo: pCODECIniFileinfo;
  pszCODECName: pChar; pulBytesRead: pLongint ): Longint; cdecl;

function mmioQueryCODECNameLength( IniInfo: pCODECIniFileinfo;
  pulNameLength: pLongint ): Longint; cdecl;

function mmioLoadCODECProc( IniInfo: pCODECIniFileInfo;
  Module: phModule; ulFlags: Longint ): pCodecProc; cdecl;

function mmioGetData( mmIO: hmmio; mmIOInfo: pmmioinfo; usFlags: Word ): Word; cdecl;

Implementation

//-------
function mmioAdvance( mmIO: hmmio; Info: pmmioinfo; usFlags: Word ): Word; cdecl;
    external LibName index 55;

function mmioAscend( mmIO: hmmio; pckinfo: pmmCkInfo; usFlags: Word ): Word; cdecl;
    external LibName index 49;

function mmioClose( mmIO: hmmio; usFlags: Word ): Word; cdecl;
    external LibName index 45;

function mmioCreateChunk( mmio: hmmio; pckinfo: pmmCkInfo; usFlags: Word ): Word; cdecl;
    external LibName index 51;

function mmioDescend( mmIO: hmmio; pckinfo, pckinfoParent: pMMCkInfo; usFlags: Word ): Word; cdecl;
    external LibName index 50;

function mmioFlush( mmIO: hmmio; usFlags: Word ): Word; cdecl;
    external LibName index 44;

function mmioGetInfo( mmIO: hmmio; Info: pmmioinfo; usFlags: Word ): Word; cdecl;
    external LibName index 52;

function mmioGetLastError( mmIO: hmmio ): Longint; cdecl;
    external LibName index 38;

function mmioInstallIOProc( fccIOProc: FourCC; pIOProc: pMMIOProc; ulFlags: Longint ): pmmIOProc; cdecl;
    external LibName index 39;

function mmioOpen( pszFileName: pChar; mmIOInfo: pmmioinfo; ulOpenFlags: Longint ): hMMIO; cdecl;
    external LibName index 40;

function mmioRead( mmIO: hmmio; pchBuffer: pChar; cBytes: Longint ): Longint; cdecl;
    external LibName index 41;

function mmioSeek( mmIO: hmmio; lOffset, lOrigin: Longint ): Longint; cdecl;
    external LibName index 43;

function mmioSendMessage( mmIO: hmmio; usMsg: Word; lParam1, lParam2: Longint ): Longint; cdecl;
    external LibName index 54;

function mmioSetBuffer( mmIO: hmmio; pchBuffer: pChar; cBytes: Longint; usFlags: Word ): Word; cdecl;
    external LibName index 56;

function mmioSetInfo( mmIO: hmmio; mmIOInfo: pmmioinfo; usFlags: Word ): Word; cdecl;
    external LibName index 53;

function mmioStringToFourCC( pszString: pChar; usFlags: Word ): FourCC; cdecl;
    external LibName index 37;

function mmioWrite( mmIO: hmmio; pchBuffer: pChar; cBytes: Longint ): Longint; cdecl;
    external LibName index 42;

function mmioCFOpen( pszFileName: pChar; CfInfo, IOInfo: pmmcfinfo; ulFlags: Longint ): hMMCF; cdecl;
    external LibName index 57;

function mmioCFClose( mmCf: hmmcf; ulFlags: Longint ): Longint; cdecl;
    external LibName index 58;

function mmioCFGetInfo( mmCf: hmmcf; CfInfo: pmmcfinfo; cBytes: Longint ): Longint; cdecl;
    external LibName index 64;

function mmioCFSetInfo( mmCf: hmmcf; CfInfo: pmmcfinfo; cBytes: Longint ): Longint; cdecl;
    external LibName index 65;

function mmioCFFindEntry( mmCf: hmmcf; ctocEntry: mmctocentry; ulFlags: Longint ): Longint; cdecl;
    external LibName index 62;

function mmioCFAddEntry( mmCf: hmmcf; cTocEntry: mmctocentry; ulFlags: Longint ): Longint; cdecl;
    external LibName index 59;

function mmioCFChangeEntry( mmCf: hmmcf; CTocEntry: pmmctocentry; ulFlags: Longint ): Longint; cdecl;
    external LibName index 61;

function mmioCFDeleteEntry( mmCf: hmmcf; CTocEntry: pmmctocentry; ulFlags: Longint ): Longint; cdecl;
    external LibName index 60;

function mmioCFAddElement( mmCf: hmmcf; pszElementName: pChar; fccType: FourCC;
  pchBuffer: pChar; cchBytes: LongInt; ulFlags: Longint ): Longint; cdecl;
    external LibName index 63;

function mmioCFCopy( mmCfSource: hmmcf; pszDestFileName: pChar; ulFlags: Longint ): Longint; cdecl;
    external LibName index 66;

function mmioQueryFormatCount( FormatInfo: pmmformatinfo;
  plNumFormats: pLong; ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;
    external LibName index 87;

function mmioGetFormats( FormatInfo: pmmformatinfo;
  lNumFormats: LongInt; pFormatInfoList: Pointer; plFormatsRead: pLong;
  ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;
    external LibName index 88;

function mmioGetFormatName( FormatInfo: pmmformatinfo; pszFormatName: pChar;
  plBytesRead: pLong; ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;
    external LibName index 93;

function mmioIdentifyFile( pszFileName: pChar; MMIoInfo: pmmioinfo;
  FormatInfo: pmmformatinfo; pfccStorageSystem: pFourCC; ulReserved: Longint;
  ulFlags: Longint ): Longint; cdecl;
    external LibName index 92;

function mmioQueryHeaderLength( mmIO: hmmio; plHeaderLength: pLong;
  ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;
    external LibName index 89;

function mmioGetHeader( mmIO: hmmio; pHeader: Pointer; lHeaderLength: LongInt;
  plBytesRead: pLong; ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;
    external LibName index 90;

function mmioSetHeader( mmIO: hmmio; pHeader: Pointer; lHeaderLength: LongInt;
  plBytesWritten: pLong; ulReserved: Longint; ulFlags: Longint ): Longint; cdecl;
    external LibName index 91;

function mmioIniFileHandler( IniFileInfo: pmminifileinfo; ulFlags: Longint ): Longint; cdecl;
    external LibName index 98;

function mmioIdentifyStorageSystem( pszFileName: pChar;
  MMIoInfo: pmmioinfo; pfccStorageSystem: pFourCC ): Longint; cdecl;
    external LibName index 100;

function mmioDetermineSSIOProc( pszFileName: pChar; MMIoInfo: pmmioinfo;
  pfccStorageSystem: pFourCC; pszParsedRemainder: pChar ): Longint; cdecl;
    external LibName index 101;

function mmioQueryIOProcModuleHandle( IOProc: pMMIOProc;
  IOProcModule: phModule ): Longint; cdecl;
    external LibName index 106;

function mmioCFCompact( pszFileName: pChar; ulFlags: Longint ): Longint; cdecl;
    external LibName index 113;

//--------
function mmioMigrateIniFile( ulFlags: Longint ): Longint; cdecl;
    external LibName index 111;

//
// MMIO CODEC APIs
///

function mmioIniFileCODEC( IniFile: pCODECIniFileInfo; ulFlags: Longint ): Longint; cdecl;
    external LibName index 112;

function mmioSet( mmIO: hmmio; ExtendInfo: pmmExtendInfo; ulFlags: Longint): Longint; cdecl;
    external LibName index 114;

function mmioQueryCODECName( IniInfo: pCODECIniFileinfo;
  pszCODECName: pChar; pulBytesRead: pLongint ): Longint; cdecl;
    external LibName index 115;

function mmioQueryCODECNameLength( IniInfo: pCODECIniFileinfo;
  pulNameLength: pLongint ): Longint; cdecl;
    external LibName index 118;

function mmioLoadCODECProc( IniInfo: pCODECIniFileInfo;
  Module: phModule; ulFlags: Longint ): pCodecProc; cdecl;
    external LibName index 117;

function mmioGetData( mmIO: hmmio; mmIOInfo: pmmioinfo; usFlags: Word ): Word; cdecl;
    external LibName index 119;


end.
