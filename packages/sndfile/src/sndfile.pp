(*
 - Translation for sndfile.h version 1.0.17 by Ido Kanner idokan at gmail dot com
*)
{
** Copyright (C) 1999-2006 Erik de Castro Lopo <erikd@mega-nerd.com>
**
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU Lesser General Public License as published by
** the Free Software Foundation; either version 2.1 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU Lesser General Public License for more details.
**
** You should have received a copy of the GNU Lesser General Public License
** along with this program; if not, write to the Free Software
<<<<<<< HEAD
<<<<<<< HEAD
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
=======
** Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
** Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2

** sndfile.h -- system-wide definitions
**
** API documentation is in the doc/ directory of the source code tarball
** and at http://www.mega-nerd.com/libsndfile/api.html.

 This is the version 1.0.X header file.
 
 For the Metrowerks CodeWarrior Pro Compiler (mainly MacOS)
 
** The following file types can be read and written.
** A file type would consist of a major type (ie SF_FORMAT_WAV) bitwise
** ORed with a minor type (ie SF_FORMAT_PCM). SF_FORMAT_TYPEMASK and
** SF_FORMAT_SUBMASK can be used to separate the major and minor file
** types.
}
{$PACKRECORDS C}{$MACRO ON}
unit sndfile; 

interface

uses
  ctypes, unixtype;

const
  sndfilelib = 'sndfile';

const
  // Major formats.
  SF_FORMAT_WAV   = $010000;		// Microsoft WAV format (little endian default).
	SF_FORMAT_AIFF  = $020000;		// Apple/SGI AIFF format (big endian).
	SF_FORMAT_AU    = $030000;		// Sun/NeXT AU format (big endian).
	SF_FORMAT_RAW   = $040000;		// RAW PCM data.
	SF_FORMAT_PAF   = $050000;		// Ensoniq PARIS file format.
	SF_FORMAT_SVX   = $060000;		// Amiga IFF / SVX8 / SV16 format.
	SF_FORMAT_NIST  = $070000;		// Sphere NIST format.
	SF_FORMAT_VOC		= $080000;		// VOC files.
	SF_FORMAT_IRCAM = $0A0000;		// Berkeley/IRCAM/CARL
	SF_FORMAT_W64   = $0B0000;		// Sonic Foundry's 64 bit RIFF/WAV
	SF_FORMAT_MAT4  = $0C0000;		// Matlab (tm) V4.2 / GNU Octave 2.0
	SF_FORMAT_MAT5  = $0D0000;		// Matlab (tm) V5.0 / GNU Octave 2.1
	SF_FORMAT_PVF   = $0E0000;		// Portable Voice Format
	SF_FORMAT_XI    = $0F0000;		// Fasttracker 2 Extended Instrument
	SF_FORMAT_HTK   = $100000;		// HMM Tool Kit format
	SF_FORMAT_SDS   = $110000;		// Midi Sample Dump Standard
	SF_FORMAT_AVR   = $120000;		// Audio Visual Research
	SF_FORMAT_WAVEX = $130000;		// MS WAVE with WAVEFORMATEX
	SF_FORMAT_SD2   = $160000;		// Sound Designer 2
	SF_FORMAT_FLAC  = $170000;		// FLAC lossless file format
	SF_FORMAT_CAF   = $180000;		// Core Audio File format
 
const
  //Subtypes from here on.
  SF_FORMAT_PCM_S8    = $0001;    // Signed 8 bit data
	SF_FORMAT_PCM_16    = $0002;    // Signed 16 bit data
	SF_FORMAT_PCM_24    = $0003;    // Signed 24 bit data
	SF_FORMAT_PCM_32    = $0004;    // Signed 32 bit data

	SF_FORMAT_PCM_U8    = $0005;    // Unsigned 8 bit data (WAV and RAW only)

	SF_FORMAT_FLOAT     = $0006;    // 32 bit float data
	SF_FORMAT_DOUBLE    = $0007;    // 64 bit float data

	SF_FORMAT_ULAW      = $0010;    // U-Law encoded.
	SF_FORMAT_ALAW      = $0011;    // A-Law encoded.
	SF_FORMAT_IMA_ADPCM = $0012;    // IMA ADPCM.
	SF_FORMAT_MS_ADPCM  = $0013;    // Microsoft ADPCM.

	SF_FORMAT_GSM610    = $0020;    // GSM 6.10 encoding.
	SF_FORMAT_VOX_ADPCM = $0021;    // OKI / Dialogix ADPCM

	SF_FORMAT_G721_32   = $0030;    // 32kbs G721 ADPCM encoding.
	SF_FORMAT_G723_24   = $0031;    // 24kbs G723 ADPCM encoding.
	SF_FORMAT_G723_40   = $0032;    // 40kbs G723 ADPCM encoding.

	SF_FORMAT_DWVW_12   = $0040;    // 12 bit Delta Width Variable Word encoding.
	SF_FORMAT_DWVW_16   = $0041;    // 16 bit Delta Width Variable Word encoding.
	SF_FORMAT_DWVW_24   = $0042;    // 24 bit Delta Width Variable Word encoding.
	SF_FORMAT_DWVW_N    = $0043;    // N bit Delta Width Variable Word encoding.

	SF_FORMAT_DPCM_8    = $0050;    // 8 bit differential PCM (XI only)
	SF_FORMAT_DPCM_16   = $0051;    // 16 bit differential PCM (XI only)

const
  // Endian-ness options.
	SF_ENDIAN_FILE     = $00000000;  // Default file endian-ness.
	SF_ENDIAN_LITTLE   = $10000000;  // Force little endian-ness.
	SF_ENDIAN_BIG      = $20000000;  // Force big endian-ness.
	SF_ENDIAN_CPU      = $30000000;  // Force CPU endian-ness.

	SF_FORMAT_SUBMASK  = $0000FFFF;
	SF_FORMAT_TYPEMASK = $0FFF0000;
	SF_FORMAT_ENDMASK  = $30000000;

{
** The following are the valid command numbers for the sf_command()
** interface.  The use of these commands is documented in the file
** command.html in the doc directory of the source code distribution.
}
const
  SFC_GET_LIB_VERSION            = $1000;
	SFC_GET_LOG_INFO               = $1001;

	SFC_GET_NORM_DOUBLE            = $1010;
	SFC_GET_NORM_FLOAT             = $1011;
	SFC_SET_NORM_DOUBLE            = $1012;
	SFC_SET_NORM_FLOAT             = $1013;
	SFC_SET_SCALE_FLOAT_INT_READ   = $1014;

	SFC_GET_SIMPLE_FORMAT_COUNT    = $1020;
	SFC_GET_SIMPLE_FORMAT          = $1021;

	SFC_GET_FORMAT_INFO            = $1028;

	SFC_GET_FORMAT_MAJOR_COUNT     = $1030;
	SFC_GET_FORMAT_MAJOR           = $1031;
	SFC_GET_FORMAT_SUBTYPE_COUNT   = $1032;
	SFC_GET_FORMAT_SUBTYPE         = $1033;

	SFC_CALC_SIGNAL_MAX            = $1040;
	SFC_CALC_NORM_SIGNAL_MAX       = $1041;
	SFC_CALC_MAX_ALL_CHANNELS      = $1042;
	SFC_CALC_NORM_MAX_ALL_CHANNELS = $1043;
	SFC_GET_SIGNAL_MAX             = $1044;
	SFC_GET_MAX_ALL_CHANNELS       = $1045;

	SFC_SET_ADD_PEAK_CHUNK         = $1050;

	SFC_UPDATE_HEADER_NOW          = $1060;
	SFC_SET_UPDATE_HEADER_AUTO     = $1061;

	SFC_FILE_TRUNCATE	             = $1080;

	SFC_SET_RAW_START_OFFSET       = $1090;

	SFC_SET_DITHER_ON_WRITE        = $10A0;
	SFC_SET_DITHER_ON_READ         = $10A1;

	SFC_GET_DITHER_INFO_COUNT      = $10A2;
	SFC_GET_DITHER_INFO            = $10A3;

	SFC_GET_EMBED_FILE_INFO        = $10B0;

	SFC_SET_CLIPPING               = $10C0;
	SFC_GET_CLIPPING               = $10C1;

	SFC_GET_INSTRUMENT             = $10D0;
	SFC_SET_INSTRUMENT             = $10D1;

	SFC_GET_LOOP_INFO              = $10E0;

	SFC_GET_BROADCAST_INFO         = $10F0;
	SFC_SET_BROADCAST_INFO         = $10F1;

	// Following commands for testing only.
	SFC_TEST_IEEE_FLOAT_REPLACE	   = $6001;

  {
	** SFC_SET_ADD_* values are deprecated and will disappear at some
	** time in the future. They are guaranteed to be here up to and
	** including version 1.0.8 to avoid breakage of existng software.
	** They currently do nothing and will continue to do nothing.
  }
	SFC_SET_ADD_DITHER_ON_WRITE	   = $1070;
	SFC_SET_ADD_DITHER_ON_READ     = $1071;

{
** String types that can be set and read from files. Not all file types
** support this and even the file types which support one, may not support
** all string types.
}
const
  SF_STR_TITLE     = $01;
	SF_STR_COPYRIGHT = $02;
	SF_STR_SOFTWARE  = $03;
	SF_STR_ARTIST    = $04;
	SF_STR_COMMENT   = $05;
	SF_STR_DATE      = $06;
 
{
** Use the following as the start and end index when doing metadata
** transcoding.
}
  SF_STR_FIRST     = SF_STR_TITLE;
  SF_STR_LAST      = SF_STR_DATE;

const
  // True and false
  SF_FALSE = 0;
  SF_TRUE  = 1;

const
  // Modes for opening files.
  SFM_READ  = $10;
	SFM_WRITE = $20;
	SFM_RDWR  = $30;

{
** Public error values. These are guaranteed to remain unchanged for the duration
** of the library major version number.
** There are also a large number of private error numbers which are internal to
** the library which can change at any time.
}
const
  SF_ERR_NO_ERROR             = 0;
	SF_ERR_UNRECOGNISED_FORMAT  = 1;
	SF_ERR_SYSTEM               = 2;
	SF_ERR_MALFORMED_FILE       = 3;
	SF_ERR_UNSUPPORTED_ENCODING = 4;

//A SNDFILE* pointer can be passed around much like stdio.h's FILE* pointer.

type
  PSNDFILE     = pointer;
  PSNDFILE_tag = PSNDFILE;

{
** The following typedef is system specific and is defined when libsndfile is.
** compiled. sf_count_t can be one of loff_t (Linux), off_t (*BSD),
** off64_t (Solaris), __int64_t (Win32) etc.
}
type
  Psf_count_t  = ^Tsf_count_t;
  Tsf_count_t = off_t;

const
  SF_COUNT_MAX = ctypes.clong($7FFFFFFFFFFFFFFF);

{
** A pointer to a SF_INFO structure is passed to sf_open_read () and filled in.
** On write, the SF_INFO structure is filled in by the user and passed into
** sf_open_write ().
}
type
  PSF_INFO = ^TSF_INFO;
  TSF_INFO = record
               frames     : Tsf_count_t; // Used to be called samples.  Changed to avoid confusion.
               samplerate : ctypes.cint;
<<<<<<< HEAD
<<<<<<< HEAD
               channels   : ctypes.cint;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
               format     : ctypes.cint;
               sections   : ctypes.cint;
               seekable   : ctypes.cint;
             end;

{
** The SF_FORMAT_INFO struct is used to retrieve information about the sound
** file formats libsndfile supports using the sf_command () interface.
**
** Using this interface will allow applications to support new file formats
** and encoding types when libsndfile is upgraded, without requiring
** re-compilation of the application.
**
** Please consult the libsndfile documentation (particularly the information
** on the sf_command () interface) for examples of its use.
}
type
  PSF_FORMAT_INFO = ^TSF_FORMAT_INFO;
  TSF_FORMAT_INFO = record
                      format    : ctypes.cint;
                      name      : ctypes.pcchar;
<<<<<<< HEAD
<<<<<<< HEAD
                      extension : ctypes.pcchar;
=======
                      extention : ctypes.pcchar;
>>>>>>> graemeg/fixes_2_2
=======
                      extention : ctypes.pcchar;
>>>>>>> origin/fixes_2_2
                    end;

{
** Enums and typedefs for adding dither on read and write.
** See the html documentation for sf_command(), SFC_SET_DITHER_ON_WRITE
** and SFC_SET_DITHER_ON_READ.
}
const
  SFD_DEFAULT_LEVEL	 = 0;
	SFD_CUSTOM_LEVEL	 = $40000000;

	SFD_NO_DITHER      = 500;
	SFD_WHITE          = 501;
	SFD_TRIANGULAR_PDF = 502;

type
  PSF_DITHER_INFO = ^TSF_DITHER_INFO;
  TSF_DITHER_INFO = record
                      type_ : ctypes.cint;
                      level : ctypes.cdouble;
                      name  : ctypes.pcchar;
                    end;

{
** Struct used to retrieve information about a file embedded within a
** larger file. See SFC_GET_EMBED_FILE_INFO.
}
type
  PSF_EMBED_FILE_INFO = ^TSF_EMBED_FILE_INFO;
  TSF_EMBED_FILE_INFO = record
                          offset : Tsf_count_t;
                          length : Tsf_count_t;
                        end;

// Structs used to retrieve music sample information from a file.

const
// The loop mode field in SF_INSTRUMENT will be one of the following.
  SF_LOOP_NONE        = 800;
  SF_LOOP_FORWARD     = 801;
  SF_LOOP_BACKWARD    = 802;
  SF_LOOP_ALTERNATING = 803;
  
type
  PSF_INSTRUMENT = ^TSF_INSTRUMENT;
  TSF_INSTRUMENT = record
                     gain         : ctypes.cint;
                     basenote,
                     detune       : ctypes.cchar;
                     velocity_lo,
                     velocity_hi  : ctypes.cchar;
                     loop_count   : ctypes.cint;
                     loops        : array[0..15] of record
                                                      mode  : ctypes.cint;
                                                      start : ctypes.cuint;
                                                      end_  : ctypes.cuint;
                                                      count : ctypes.cuint;
                                                    end;
                   end;

// Struct used to retrieve loop information from a file.
type
  PSF_LOOP_INFO = ^TSF_LOOP_INFO;
  TSF_LOOP_INFO = record
                    time_sig_num : ctypes.cushort;             // any positive integer    > 0
                    time_sig_den : ctypes.cushort;             // any positive power of 2 > 0
                    loop_mode    : ctypes.cint;                // see SF_LOOP enum
                    
                    num_beats    : ctypes.cint;                // this is NOT the amount of quarter notes !!!
                                                               // a full bar of 4/4 is 4 beats
                                                               // a full bar of 7/8 is 7 beats
                                                   
                    bpm          : ctypes.cfloat;              // suggestion, as it can be calculated using other fields:
                                                               // file's lenght, file's sampleRate and our time_sig_den
                                                               // -> bpms are always the amount of _quarter notes_ per minute
                                                   
                    root_key     : ctypes.cint;                // MIDI note, or -1 for None
                    future       : array[0..5] of ctypes.cint;
                  end;
                  

{
**	Struct used to retrieve broadcast (EBU) information from a file.
**	Strongly (!) based on EBU "bext" chunk format used in Broadcast WAVE.
}
type
  PSF_BROADCAST_INFO = ^TSF_BROADCAST_INFO;
  TSF_BROADCAST_INFO = record
                         description          : array[0..255] of ctypes.cchar;
                         originator           : array[0..31]  of ctypes.cchar;
                         originator_reference : array[0..31]  of ctypes.cchar;
                         origination_date     : array[0..9]   of ctypes.cchar;
                         origination_time     : array[0..7]   of ctypes.cchar;
                         time_reference_low   : ctypes.cint;
                         time_reference_high  : ctypes.cint;
                         version              : ctypes.cshort;
                         umid                 : array[0..63]  of ctypes.cchar;
                         reserved             : array[0..189] of ctypes.cchar;
                         coding_history_size  : ctypes.cuint;
                         coding_history       : array[0..255] of ctypes.cchar;
                       end;

type
   Tsf_vio_get_filelen = function (user_date : pointer) : Tsf_count_t;           cdecl;
   
   Tsf_vio_seek        = function (offest : Tsf_count_t; whence : ctypes.cint;
                                   user_date : pointer) : Tsf_count_t;           cdecl;
                                   
   Tsf_vio_read        = function (ptr : Pointer; count : Tsf_count_t;
                                   user_date : pointer) : Tsf_count_t;           cdecl;

   Tsf_vio_write       = function (ptr : Pointer; count : Tsf_count_t;
                                   user_date : pointer) : Tsf_count_t;           cdecl;
                                   
   Tsf_vio_tell        = function (user_data : Pointer) : Tsf_count_t;           cdecl;


   PSF_VIRTUAL_IO = ^TSF_VIRTUAL_IO;
   TSF_VIRTUAL_IO = record
                      get_filelen : Tsf_vio_get_filelen;
                      seek        : Tsf_vio_seek;
                      read        : Tsf_vio_read;
                      write       : Tsf_vio_write;
                      tell        : Tsf_vio_tell;
                    end;

{
** Open the specified file for read, write or both. On error, this will
** return a NULL pointer. To find the error number, pass a NULL SNDFILE
** to sf_perror () or sf_error_str ().
** All calls to sf_open() should be matched with a call to sf_close().
}
<<<<<<< HEAD
<<<<<<< HEAD
function sf_open (path : pChar; mode : ctypes.cint; sfinfo : PSF_INFO) : PSNDFILE; cdecl;
=======
function sf_open (path : ctypes.pcchar; mode : ctypes.cint; sfinfo : PSF_INFO) : PSNDFILE; cdecl;
>>>>>>> graemeg/fixes_2_2
=======
function sf_open (path : ctypes.pcchar; mode : ctypes.cint; sfinfo : PSF_INFO) : PSNDFILE; cdecl;
>>>>>>> origin/fixes_2_2
  external sndfilelib  name 'sf_open';

{
** Use the existing file descriptor to create a SNDFILE object. If close_desc
** is TRUE, the file descriptor will be closed when sf_close() is called. If
** it is FALSE, the descritor will not be closed.
** When passed a descriptor like this, the library will assume that the start
** of file header is at the current file offset. This allows sound files within
** larger container files to be read and/or written.
** On error, this will return a NULL pointer. To find the error number, pass a
** NULL SNDFILE to sf_perror () or sf_error_str ().
** All calls to sf_open_fd() should be matched with a call to sf_close().
}

function sf_open_fd (fd     : ctypes.cint; mode       : ctypes.cint;
                     sfinfo : PSF_INFO;    close_desc : ctypes.cint) : PSNDFILE; cdecl;
  external sndfilelib name 'sf_open_fd';

function sf_open_virtual (sfvirtual : PSF_VIRTUAL_IO; mode      : ctypes.cint;
                          sfinfo    : PSF_INFO;       user_data : Pointer) : PSNDFILE; cdecl;
  external sndfilelib name 'sf_open_virtual';

{
** sf_error () returns a error number which can be translated to a text
** string using sf_error_number().
}

function sf_error (sndfile : PSNDFILE) : ctypes.cint; cdecl;
  external sndfilelib name 'sf_error';

{
** sf_strerror () returns to the caller a pointer to the current error message for
** the given SNDFILE.
}

function sf_strerror (sndfile : PSNDFILE) : ctypes.pcchar; cdecl;
  external sndfilelib name 'sf_strerror';

{
** sf_error_number () allows the retrieval of the error string for each internal
** error number.
}

function sf_error_number (errnum : ctypes.cint) : ctypes.pcchar; cdecl;
  external sndfilelib name 'sf_error_number';

{
** The following three error functions are deprecated but they will remain in the
** library for the forseeable future. The function sf_strerror() should be used
** in their place.
}

function sf_perror (sndfile : PSNDFILE) : ctypes.cint; cdecl;
  external sndfilelib name 'sf_perror';

function sf_error_str (sndfile : PSNDFILE; str : ctypes.pcchar;
                       len     : size_t) : ctypes.cint; cdecl;
  external sndfilelib name 'sf_error_str';

// Return TRUE if fields of the SF_INFO struct are a valid combination of values.

function sf_command (sndfile : PSNDFILE; command  : ctypes.cint;
                     data    : Pointer;  datasize : ctypes.cint) : ctypes.cint; cdecl;
  external sndfilelib name 'sf_command';

// Return TRUE if fields of the SF_INFO struct are a valid combination of values.

function sf_format_check (info : PSF_INFO) : ctypes.cint; cdecl;
  external sndfilelib name 'sf_format_check';

{
** Seek within the waveform data chunk of the SNDFILE. sf_seek () uses
** the same values for whence (SEEK_SET, SEEK_CUR and SEEK_END) as
** stdio.h function fseek ().
** An offset of zero with whence set to SEEK_SET will position the
** read / write pointer to the first data sample.
** On success sf_seek returns the current position in (multi-channel)
** samples from the start of the file.
** Please see the libsndfile documentation for moving the read pointer
** separately from the write pointer on files open in mode SFM_RDWR.
** On error all of these functions return -1.
}

function sf_seek (sndfile : PSNDFILE; frame : Tsf_count_t;
                  whence  : ctypes.cint) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_seek';

{
** Functions for retrieving and setting string data within sound files.
** Not all file types support this features; AIFF and WAV do. For both
** functions, the str_type parameter must be one of the SF_STR_* values
** defined above.
** On error, sf_set_string() returns non-zero while sf_get_string()
** returns NULL.
}

function sf_set_string (sndfile : PSNDFILE;      str_type : ctypes.cint;
                        str     : ctypes.pcchar) : ctypes.cint; cdecl;
  external sndfilelib name 'sf_set_string';
  
function sf_get_string (sndfile : PSNDFILE; str_type : ctypes.cint) : ctypes.pcchar; cdecl;
  external sndfilelib name 'sf_get_string';

// Functions for reading/writing the waveform data of a sound file.

function sf_read_raw (sndfile : PSNDFILE; ptr : Pointer;
                      bytes   : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_read_raw';
  
function sf_write_raw (sndfile : PSNDFILE; ptr : Pointer;
                      bytes   : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_write_raw';

{
** Functions for reading and writing the data chunk in terms of frames.
** The number of items actually read/written = frames * number of channels.
**     sf_xxxx_raw		read/writes the raw data bytes from/to the file
**     sf_xxxx_short	passes data in the native short format
**     sf_xxxx_int		passes data in the native int format
**     sf_xxxx_float	passes data in the native float format
**     sf_xxxx_double	passes data in the native double format
** All of these read/write function return number of frames read/written.
}

function sf_readf_short (sndfile : PSNDFILE; ptr : ctypes.pcshort;
                         frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_readf_short';
                         
function sf_writef_short (sndfile : PSNDFILE; ptr : ctypes.pcshort;
                          frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_writef_short';

function sf_readf_int (sndfile : PSNDFILE; ptr : ctypes.pcint;
                       frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_readf_int';

function sf_writef_int (sndfile : PSNDFILE; ptr : ctypes.pcint;
                        frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_writef_int';

function sf_readf_float (sndfile : PSNDFILE; ptr : ctypes.pcfloat;
                         frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_readf_float';

function sf_writef_float (sndfile : PSNDFILE; ptr : ctypes.pcfloat;
                          frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_writef_float';

function sf_readf_double (sndfile : PSNDFILE; ptr : ctypes.pcdouble;
                          frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_readf_double';

function sf_writef_double (sndfile : PSNDFILE; ptr : ctypes.pcdouble;
                           frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_writef_double';

{
** Functions for reading and writing the data chunk in terms of items.
** Otherwise similar to above.
** All of these read/write function return number of items read/written.
}

function sf_read_short (sndfile : PSNDFILE; ptr : ctypes.pcshort;
                        frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_read_short';

function sf_write_short (sndfile : PSNDFILE; ptr : ctypes.pcshort;
                         frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_write_short';

function sf_read_int (sndfile : PSNDFILE; ptr : ctypes.pcint;
                      frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_read_int';

function sf_write_int (sndfile : PSNDFILE; ptr : ctypes.pcint;
                       frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_write_int';

function sf_read_float (sndfile : PSNDFILE; ptr : ctypes.pcfloat;
                        frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_read_float';
  
function sf_write_float (sndfile : PSNDFILE; ptr : ctypes.pcfloat;
                         frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_write_float';

function sf_read_double (sndfile : PSNDFILE; ptr : ctypes.pcdouble;
                         frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_read_double';

function sf_write_double (sndfile : PSNDFILE; ptr : ctypes.pcdouble;
                          frames  : Tsf_count_t) : Tsf_count_t; cdecl;
  external sndfilelib name 'sf_write_double';

{
** Close the SNDFILE and clean up all memory allocations associated with this
** file.
** Returns 0 on success, or an error number.
}

function sf_close (sndfile : PSNDFILE) : ctypes.cint; cdecl;
  external sndfilelib name 'sf_close';

{
** If the file is opened SFM_WRITE or SFM_RDWR, call fsync() on the file
** to force the writing of data to disk. If the file is opened SFM_READ
** no action is taken.
}

procedure sf_write_sync (sndfile : PSNDFILE); cdecl;
  external sndfilelib name 'sf_write_sync';

implementation

end.

