{
  Translation of the libmad headers for FreePascal
  Copyright (C) 2006 by Ivo Steinmann
}

(*
 * libmad - MPEG audio decoder library
 * Copyright (C) 2000-2003 Underbit Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * If you would like to negotiate alternate licensing terms, you may do
 * so by contacting: Underbit Technologies, Inc. <info@underbit.com>
 *)

unit mad;

{$mode objfpc}
{$MINENUMSIZE 4}

interface

uses
  ctypes;

{$IFDEF WINDOWS}
  {$DEFINE DYNLINK}
{$ENDIF}

{$DEFINE MAD_DISABLE_BUILTIN_DECODER}

{$IFDEF DYNLINK}
const
{$IF Defined(WINDOWS)}
  madlib = 'libmad.dll';
{$ELSEIF Defined(UNIX)}
  madlib = 'libmad.so';
{$ELSE}
  {$MESSAGE ERROR 'DYNLINK not supported'}
{$IFEND}
{$ELSE}
  {$LINKLIB mad}
{$ENDIF}


(***********************************************************************)
(* Header : version.h                                                  *)
(***********************************************************************)

const
  MAD_VERSION_MAJOR                 = 0;
  MAD_VERSION_MINOR                 = 15;
  MAD_VERSION_PATCH                 = 1;
  MAD_VERSION_EXTRA                 = ' (beta)';

function MAD_VERSION_STRINGIZE(num: cint): String;
function MAD_VERSION_STRING(num: cint): String;
function MAD_VERSION: String;

const
  MAD_PUBLISHYEAR                   = '2000-2004';
  MAD_AUTHOR                        = 'Underbit Technologies, Inc.';
  MAD_EMAIL                         = 'info@underbit.com';


(***********************************************************************)
(* Header : fixed.h                                                    *)
(***********************************************************************)

type
  mad_fixed_t                       = csint;
  mad_fixed64hi_t                   = csint;
  mad_fixed64lo_t                   = cuint;
  mad_fixed64_t                     = cslonglong;
  mad_sample_t                      = mad_fixed_t;

{*
 * Fixed-point format: 0xABBBBBBB
 * A == whole part      (sign + 3 bits)
 * B == fractional part (28 bits)
 *
 * Values are signed two's complement, so the effective range is:
 * 0x80000000 to 0x7fffffff
 *       -8.0 to +7.9999999962747097015380859375
 *
 * The smallest representable value is:
 * 0x00000001 == 0.0000000037252902984619140625 (i.e. about 3.725e-9)
 *
 * 28 bits of fractional accuracy represent about
 * 8.6 digits of decimal accuracy.
 *
 * Fixed-point numbers can be added or subtracted as normal
 * integers, but multiplication requires shifting the 64-bit result
 * from 56 fractional bits back to 28 (and rounding.)
 *
 * Changing the definition of MAD_F_FRACBITS is only partially
 * supported, and must be done with care.
 *}

const
  MAD_F_FRACBITS                    = 28;
//  MAD_F_MIN                       = mad_fixed_t(-$80000000);
  MAD_F_MAX                         = mad_fixed_t(+$7fffffff);
  MAD_F_ONE                         = mad_fixed_t( $10000000);
  MAD_F_SCALEBITS                   = MAD_F_FRACBITS;

//function  mad_f_tofixed(x: double): mad_fixed_t;


(***********************************************************************)
(* Header : bit.h                                                      *)
(***********************************************************************)

type
  mad_bitptr = record
    byte  : ^cuchar;
    cache : cushort;
    left  : cushort;
  end;

procedure mad_bit_init(var bitptr: mad_bitptr; byte: pcuchar); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_bit_finish(var bitptr: mad_bitptr);
function  mad_bit_length(var begin_, end_: mad_bitptr): cuint; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function  mad_bit_bitsleft(var bitptr: mad_bitptr): cushort;
function  mad_bit_nextbyte(var bitptr: mad_bitptr): pcuchar; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_bit_skip(var bitptr: mad_bitptr); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function  mad_bit_read(var bitptr: mad_bitptr; len: cuint): culong; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_bit_write(var bitptr: mad_bitptr; len: cuint; value: culong); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function  mad_bit_crc(bitptr: mad_bitptr; len: cuint; init: cushort): cushort; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};


(***********************************************************************)
(* Header : timer.h                                                    *)
(***********************************************************************)

type
  mad_timer_t = record
    seconds  : cslong;        { whole seconds }
    fraction : culong;      { 1/MAD_TIMER_RESOLUTION seconds }
  end;

const
  MAD_TIMER_RESOLUTION              = 352800000;
  mad_timer_zero                    : mad_timer_t = (seconds:0; fraction:0);

type
  mad_units = (
    MAD_UNITS_HOURS                 =    -2,
    MAD_UNITS_MINUTES               =    -1,
    MAD_UNITS_SECONDS               =     0,

    { metric units }

    MAD_UNITS_DECISECONDS           =    10,
    MAD_UNITS_CENTISECONDS          =   100,
    MAD_UNITS_MILLISECONDS          =  1000,

    { audio sample units }

    MAD_UNITS_8000_HZ               =  8000,
    MAD_UNITS_11025_HZ              = 11025,
    MAD_UNITS_12000_HZ              = 12000,

    MAD_UNITS_16000_HZ              = 16000,
    MAD_UNITS_22050_HZ              = 22050,
    MAD_UNITS_24000_HZ              = 24000,

    MAD_UNITS_32000_HZ              = 32000,
    MAD_UNITS_44100_HZ              = 44100,
    MAD_UNITS_48000_HZ              = 48000,

    { video frame/field units }

    MAD_UNITS_24_FPS                =    24,
    MAD_UNITS_25_FPS                =    25,
    MAD_UNITS_30_FPS                =    30,
    MAD_UNITS_48_FPS                =    48,
    MAD_UNITS_50_FPS                =    50,
    MAD_UNITS_60_FPS                =    60,

    { CD audio frames }

    MAD_UNITS_75_FPS                =    75,

    { video drop-frame units }

    MAD_UNITS_23_976_FPS            =   -24,
    MAD_UNITS_24_975_FPS            =   -25,
    MAD_UNITS_29_97_FPS             =   -30,
    MAD_UNITS_47_952_FPS            =   -48,
    MAD_UNITS_49_95_FPS             =   -50,
    MAD_UNITS_59_94_FPS             =   -60
  );


procedure mad_timer_reset(var timer: mad_timer_t);
function  mad_timer_compare(timer1: mad_timer_t; timer2: mad_timer_t): cint; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function  mad_timer_sign(timer: mad_timer_t): cint;
procedure mad_timer_negate(var timer: mad_timer_t); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function  mad_timer_abs(timer: mad_timer_t): mad_timer_t; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_timer_set(var timer: mad_timer_t; seconds, numer, denom: culong); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_timer_add(var timer: mad_timer_t; incr: mad_timer_t); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_timer_multiply(var timer: mad_timer_t; scalar: cslong); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function  mad_timer_count(timer: mad_timer_t; units: mad_units): cslong; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function  mad_timer_fraction(timer: mad_timer_t; denom: culong): culong; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_timer_string(timer: mad_timer_t; dest, format: pcchar; units, fracunits: mad_units; subparts: culong); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};


(***********************************************************************)
(* Header : stream.h                                                   *)
(***********************************************************************)

type
  mad_error = (
    MAD_ERROR_NONE                  = $0000,    { no error }

    MAD_ERROR_BUFLEN                = $0001,    { input buffer too small (or EOF) }
    MAD_ERROR_BUFPTR                = $0002,    { invalid (null) buffer pointer }

    MAD_ERROR_NOMEM                 = $0031,    { not enough memory }

    MAD_ERROR_LOSTSYNC              = $0101,    { lost synchronization }
    MAD_ERROR_BADLAYER              = $0102,    { reserved header layer value }
    MAD_ERROR_BADBITRATE            = $0103,    { forbidden bitrate value }
    MAD_ERROR_BADSAMPLERATE         = $0104,    { reserved sample frequency value }
    MAD_ERROR_BADEMPHASIS           = $0105,    { reserved emphasis value }

    MAD_ERROR_BADCRC                = $0201,    { CRC check failed }
    MAD_ERROR_BADBITALLOC           = $0211,    { forbidden bit allocation value }
    MAD_ERROR_BADSCALEFACTOR        = $0221,    { bad scalefactor index }
    MAD_ERROR_BADFRAMELEN           = $0231,    { bad frame length }
    MAD_ERROR_BADBIGVALUES          = $0232,    { bad big_values count }
    MAD_ERROR_BADBLOCKTYPE          = $0233,    { reserved block_type }
    MAD_ERROR_BADSCFSI              = $0234,    { bad scalefactor selection info }
    MAD_ERROR_BADDATAPTR            = $0235,    { bad main_data_begin pointer }
    MAD_ERROR_BADPART3LEN           = $0236,    { bad audio data length }
    MAD_ERROR_BADHUFFTABLE          = $0237,    { bad Huffman table select }
    MAD_ERROR_BADHUFFDATA           = $0238,    { Huffman data overrun }
    MAD_ERROR_BADSTEREO             = $0239     { incompatible block_type for JS }
  );

function MAD_RECOVERABLE(error: mad_error) : Boolean;

type
  mad_stream = record
    buffer       : pointer;             { input bitstream buffer }
    bufend       : pointer;             { end of buffer }
    skiplen      : culong;       { bytes to skip before next frame }
    sync         : cint;                 { stream sync found }
    freerate     : culong;       { free bitrate (fixed) }
    this_frame   : pointer;             { start of current frame }
    next_frame   : pointer;             { start of next frame }
    ptr          : mad_bitptr;          { current processing bit pointer }
    anc_ptr      : mad_bitptr;          { ancillary bits pointer }
    anc_bitlen   : cuint;        { number of ancillary bits }
    main_data    : pointer;             { Layer III main_data() }
    md_len       : cuint;        { bytes in main_data }
    options      : cint;                 { decoding options (see below) }
    error        : mad_error;           { error code (see above) }
  end;

const
  MAD_BUFFER_GUARD                  = 8;
  MAD_BUFFER_MDLEN                  = 511 + 2048 + MAD_BUFFER_GUARD;

  MAD_OPTION_IGNORECRC              = $0001;    { ignore CRC errors }
  MAD_OPTION_HALFSAMPLERATE         = $0002;    { generate PCM at 1/2 sample rate }
{$if defined(false)}                            { not yet implemented }
  MAD_OPTION_LEFTCHANNEL            = $0010;    { decode left channel only }
  MAD_OPTION_RIGHTCHANNEL           = $0020;    { decode right channel only }
  MAD_OPTION_SINGLECHANNEL          = $0030;    { combine channels }
{$ifend}

procedure mad_stream_init(var stream: mad_stream); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_stream_finish(var stream: mad_stream); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_stream_options(stream: mad_stream; opts: cint);
procedure mad_stream_buffer(var stream: mad_stream; buffer: pcuchar; length: culong); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_stream_skip(var stream: mad_stream; length: culong); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function  mad_stream_sync(var stream: mad_stream): cint; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function  mad_stream_errorstr(var stream: mad_stream): pcchar; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};


(***********************************************************************)
(* Header : frame.h                                                    *)
(***********************************************************************)

type
  mad_layer = (
    MAD_LAYER_I                     = 1,    { Layer I }
    MAD_LAYER_II                    = 2,    { Layer II }
    MAD_LAYER_III                   = 3     { Layer III }
  );

   mad_mode = (
    MAD_MODE_SINGLE_CHANNEL         = 0,    { single channel }
    MAD_MODE_DUAL_CHANNEL           = 1,    { dual channel }
    MAD_MODE_JOINT_STEREO           = 2,    { joint (MS/intensity) stereo }
    MAD_MODE_STEREO                 = 3     { normal LR stereo }
  );

  mad_emphasis = (
    MAD_EMPHASIS_NONE               = 0,    { no emphasis }
    MAD_EMPHASIS_50_15_US           = 1,    { 50/15 microseconds emphasis }
    MAD_EMPHASIS_CCITT_J_17         = 3,    { CCITT J.17 emphasis }
    MAD_EMPHASIS_RESERVED           = 2     { unknown emphasis }
  );

  mad_header = record
    layer          : mad_layer;         { audio layer (1, 2, or 3) }
    mode           : mad_mode;          { channel mode (see above) }
    mode_extension : cint;               { additional mode info }
    emphasis       : mad_emphasis;      { de-emphasis to use (see above) }
    bitrate        : culong;     { stream bitrate (bps) }
    samplerate     : cuint;      { sampling frequency (Hz) }
    crc_check      : cushort;    { frame CRC accumulator }
    crc_target     : cushort;    { final target CRC checksum }
    flags          : cint;               { flags (see below) }
    private_bits   : cint;               { private bits (see below) }
    duration       : mad_timer_t;       { audio playing time of frame }
  end;

  mad_overlap = array[0..1, 0..31, 0..17] of mad_fixed_t;

  mad_frame = record
    header         : mad_header;        { MPEG audio header }
    options        : cint;               { decoding options (from stream) }
                                        { synthesis subband filter samples }
    sbsample       : packed array[0..1, 0..35, 0..31] of mad_fixed_t;
    overlap        : ^mad_overlap;      { Layer III block overlap data }
  end;

const
  MAD_FLAG_NPRIVATE_III             = $0007;    { number of Layer III private bits }
  MAD_FLAG_INCOMPLETE               = $0008;    { header but not data is decoded }

  MAD_FLAG_PROTECTION               = $0010;    { frame has CRC protection }
  MAD_FLAG_COPYRIGHT                = $0020;    { frame is copyright }
  MAD_FLAG_ORIGINAL                 = $0040;    { frame is original (else copy) }
  MAD_FLAG_PADDING                  = $0080;    { frame has additional slot }

  MAD_FLAG_I_STEREO                 = $0100;    { uses intensity joint stereo }
  MAD_FLAG_MS_STEREO                = $0200;    { uses middle/side joint stereo }
  MAD_FLAG_FREEFORMAT               = $0400;    { uses free format bitrate }

  MAD_FLAG_LSF_EXT                  = $1000;    { lower sampling freq. extension }
  MAD_FLAG_MC_EXT                   = $2000;    { multichannel audio extension }
  MAD_FLAG_MPEG_2_5_EXT             = $4000;    { MPEG 2.5 (unofficial) extension }

  MAD_PRIVATE_HEADER                = $0100;    { header private bit }
  MAD_PRIVATE_III                   = $001f;    { Layer III private bits (up to 5) }

function  mad_nchannels(header: mad_header): cint;
function  mad_nsbsamples(header: mad_header): cint;
procedure mad_header_init(var header: mad_header); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_header_finish(var header: mad_header);
function  mad_header_decode(var header: mad_header; var stream: mad_stream): cint; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_frame_init(var frame: mad_frame); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_frame_finish(var frame: mad_frame); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function  mad_frame_decode(var frame: mad_frame; var stream: mad_stream): cint; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_frame_mute(var frame: mad_frame); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};


(***********************************************************************)
(* Header : synth.h                                                    *)
(***********************************************************************)

type
  mad_pcm = record
    samplerate : cuint;          { sampling frequency (Hz) }
    channels   : cushort;        { number of channels }
    length     : cushort;        { number of samples per channel }
                                        { PCM output samples [ch][sample] }
    samples    : packed array [0..1, 0..1151] of mad_fixed_t;
  end;

  mad_synth = record
                                        { polyphase filterbank outputs [ch][eo][peo][s][v] }
    filter     : array[0..1, 0..1, 0..1, 0..15, 0..7] of mad_fixed_t;
    phase      : cuint;          { current processing phase }
    pcm        : mad_pcm;               { PCM output }
  end;

const
  { single channel PCM selector }
  MAD_PCM_CHANNEL_SINGLE            = 0;

  { dual channel PCM selector }
  MAD_PCM_CHANNEL_DUAL_1            = 0;
  MAD_PCM_CHANNEL_DUAL_2            = 1;

  { stereo PCM selector }
  MAD_PCM_CHANNEL_STEREO_LEFT       = 0;
  MAD_PCM_CHANNEL_STEREO_RIGHT      = 1;

procedure mad_synth_init(var synth: mad_synth); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_synth_finish(var synth: mad_synth);
procedure mad_synth_mute(var synth: mad_synth); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
procedure mad_synth_frame(var synth: mad_synth; var frame: mad_frame); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};


(***********************************************************************)
(* Header : decoder.h                                                  *)
(***********************************************************************)

{$IFNDEF MAD_DISABLE_BUILTIN_DECODER}
type
  mad_decoder_mode = (
    MAD_DECODER_MODE_SYNC           = 0,
    MAD_DECODER_MODE_ASYNC          = 1
  );

  mad_flow = (
    MAD_FLOW_CONTINUE               = $0000,    { continue normally }
    MAD_FLOW_STOP                   = $0010,    { stop decoding normally }
    MAD_FLOW_BREAK                  = $0011,    { stop decoding and signal an error }
    MAD_FLOW_IGNORE                 = $0020     { ignore the current frame }
  );

  async_struct = record
    pid         : clong;
    _in         : cint;
    _out        : cint;
  end;

  sync_struct = record
    stream      : mad_stream;
    frame       : mad_frame;
    synth       : mad_synth;
  end;

  mad_input_func    = function(user: Pointer; var stream: mad_stream): mad_flow; cdecl;
  mad_header_func   = function(user: Pointer; var header: mad_header): mad_flow; cdecl;
  mad_filter_func   = function(user: Pointer; var frame: mad_frame): mad_flow; cdecl;
  mad_output_func   = function(user: Pointer; var header: mad_header; var pcm: mad_pcm): mad_flow; cdecl;
  mad_error_func    = function(user: Pointer; var stream: mad_stream; var frame: mad_frame): mad_flow; cdecl;
  mad_message_func  = function(user, msg: Pointer; var l: cuint): mad_flow; cdecl;

  mad_decoder = record
    mode        : mad_decoder_mode;
    options     : cint;
    async       : async_struct;
    sync        : ^sync_struct;
    data        : pointer;
    InputFunc   : mad_input_func;
    HeaderFunc  : mad_header_func;
    FilterFunc  : mad_filter_func;
    OutputFunc  : mad_output_func;
    ErrorFunc   : mad_error_func;
    MessageFunc : mad_message_func;
  end;

procedure mad_decoder_init(var decoder: mad_decoder; user: pointer; Input: mad_input_func; Header: mad_header_func; Filter: mad_filter_func; Output: mad_output_func; Error: mad_error_func; Message: mad_message_func); cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function mad_decoder_finish(var decoder: mad_decoder): cint; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function mad_decoder_run(var decoder: mad_decoder; mode: mad_decoder_mode): cint; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
function mad_decoder_message(var decoder: mad_decoder; msg: Pointer; var l: cuint): cint; cdecl; external {$IFDEF DYNLINK}madlib{$ENDIF};
{$ENDIF}


{
  Developer of the MAD helpers for FreePascal
  Copyright (C) 2006 by Ivo Steinmann
}
{$IFDEF MAD_DISABLE_BUILTIN_DECODER}
const
  MAD_INPUT_BUFFER_SIZE = 5*8192;

type
  mad_read_func  = function(user: pointer; ptr: pointer; size: cuint): cuint; cdecl;
  mad_seek_func  = function(user: pointer; offset: clong; whence: cint): clong; cdecl;
  mad_close_func = function(user: pointer): cint; cdecl;
  mad_tell_func  = function(user: pointer): clong; cdecl;

  pmad_decoder = ^mad_decoder;
  mad_decoder = record
    inbuf       : array[0..MAD_INPUT_BUFFER_SIZE-1] of cuint8;
    stream      : mad_stream;
    frame       : mad_frame;
    synth       : mad_synth;
    samplecnt   : cint;
    sampleofs   : cint;
    user        : pointer;
    read        : mad_read_func;
    seek        : mad_seek_func;
    close       : mad_close_func;
    tell        : mad_tell_func;

  // Userinfo
    sample_rate : cint;
  end;

function mad_decoder_init(user: pointer; read: mad_read_func; seek: mad_seek_func; close: mad_close_func; tell: mad_tell_func): pmad_decoder;
function mad_decoder_read(decoder: pmad_decoder; buffer: pointer; length: cint): cint;
procedure mad_decoder_free(decoder: pmad_decoder);
{$ENDIF}

implementation

function MAD_VERSION_STRINGIZE(num: cint): String;
begin
  MAD_VERSION_STRINGIZE := '';
  Str(num, MAD_VERSION_STRINGIZE);
end;

function MAD_VERSION_STRING(num: cint): String;
begin
  MAD_VERSION_STRING := MAD_VERSION_STRINGIZE(num);
end;

function MAD_VERSION: String;
begin
  MAD_VERSION :=
    MAD_VERSION_STRING(MAD_VERSION_MAJOR) + '.' +
    MAD_VERSION_STRING(MAD_VERSION_MINOR) + '.' +
    MAD_VERSION_STRING(MAD_VERSION_PATCH) +
    MAD_VERSION_EXTRA;
end;

{function mad_f_tofixed(x: double): mad_fixed_t;
begin
  Result := mad_fixed_t(x * double(1 shl MAD_F_FRACBITS) + 0.5);
end;}

procedure mad_bit_finish(var bitptr: mad_bitptr);
begin
end;

function mad_bit_bitsleft(var bitptr: mad_bitptr): cushort;
begin
  mad_bit_bitsleft := bitptr.left;
end;

procedure mad_timer_reset(var timer: mad_timer_t);
begin
  timer := mad_timer_zero;
end;

function mad_timer_sign(timer: mad_timer_t): cint;
begin
  mad_timer_sign := mad_timer_compare(timer, mad_timer_zero);
end;

function MAD_RECOVERABLE(error: mad_error): Boolean;
begin
  MAD_RECOVERABLE := word(error) and $ff00 > 0;
end;

procedure mad_stream_options(stream: mad_stream; opts: cint);
begin
  stream.options := opts;
end;

procedure mad_header_finish(var header: mad_header);
begin
  FillChar(header, sizeof(mad_header), 0);
end;

function mad_nchannels(header: mad_header): cint;
begin
  if longword(header.mode) <> 0 then
    mad_nchannels := 2 else
    mad_nchannels := 1;
end;

function mad_nsbsamples(header: mad_header): cint;
begin
  if header.layer = MAD_LAYER_I then mad_nsbsamples := 12 else
  if (header.layer = MAD_LAYER_III) and (header.flags and MAD_FLAG_LSF_EXT > 0)
    then mad_nsbsamples := 18
    else mad_nsbsamples := 36;
end;

procedure mad_synth_finish(var synth: mad_synth);
begin
  FillChar(synth, sizeof(mad_synth), 0);
end;

{$IFDEF MAD_DISABLE_BUILTIN_DECODER}
function mad_decoder_init(user: pointer; read: mad_read_func; seek: mad_seek_func; close: mad_close_func; tell: mad_tell_func): pmad_decoder;
begin
  GetMem(Result, Sizeof(mad_decoder));
  FillChar(Result^, Sizeof(mad_decoder), 0);
  mad_stream_init(Result^.stream);
  mad_frame_init(Result^.frame);
  mad_synth_init(Result^.synth);
  Result^.user := user;
  Result^.read := read;
  Result^.seek := seek;
  Result^.close := close;
  Result^.tell := tell;
end;

procedure mad_decoder_free(decoder: pmad_decoder);
begin
  if not Assigned(decoder) then
    Exit;

  mad_synth_finish(decoder^.synth);
  mad_frame_finish(decoder^.frame);
  mad_stream_finish(decoder^.stream);
  decoder^.close(decoder^.user);
  FreeMem(decoder);
end;

function mad_decoder_read(decoder: pmad_decoder; buffer: pointer; length: cint): cint;
var
  ofs, num, i: cint;
  inbuf_ptr: pointer;
  len, remaining: cint;
begin
  // check blocksize here!

  ofs := 0;
  num := length;

  while num > 0 do
  begin
    if decoder^.samplecnt = 0 then
    begin
      if (decoder^.stream.buffer = nil) or (decoder^.stream.error = MAD_ERROR_BUFLEN) then
      begin
        if Assigned(decoder^.stream.next_frame) then
        begin
          remaining := ptrint(decoder^.stream.bufend) - ptrint(decoder^.stream.next_frame);
          inbuf_ptr := pointer(ptrint(@decoder^.inbuf) + remaining);
          len  := MAD_INPUT_BUFFER_SIZE - remaining;
          Move(decoder^.stream.next_frame^, decoder^.inbuf, remaining);
        end else begin
          remaining := 0;
          len  := MAD_INPUT_BUFFER_SIZE;
          inbuf_ptr := @decoder^.inbuf;
        end;

        len := decoder^.read(decoder^.user, inbuf_ptr, len);
        if len <= 0 then
          Exit(ofs);

        mad_stream_buffer(decoder^.stream, decoder^.inbuf, len+remaining);
        decoder^.stream.error := MAD_ERROR_NONE;
      end;

      if mad_frame_decode(decoder^.frame, decoder^.stream) <> 0 then
      begin
        if MAD_RECOVERABLE(decoder^.stream.error) or (decoder^.stream.error = MAD_ERROR_BUFLEN) then
          Continue;

        Exit(ofs);
      end;

      mad_synth_frame(decoder^.synth, decoder^.frame);

      with decoder^.synth do
      if pcm.channels = 2 then
      begin
        for i := 0 to pcm.length -1 do
        begin
          if pcm.samples[0][i] >= MAD_F_ONE then
            pcm.samples[0][i] := MAD_F_ONE - 1;
          if pcm.samples[0][i] < -MAD_F_ONE then
            pcm.samples[0][i] := -MAD_F_ONE;
          pcm.samples[0][i] := pcm.samples[0][i] shr (MAD_F_FRACBITS + 1 - 16 + 1);

          if pcm.samples[1][i] >= MAD_F_ONE then
            pcm.samples[1][i] := MAD_F_ONE - 1;
          if pcm.samples[1][i] < -MAD_F_ONE then
            pcm.samples[1][i] := -MAD_F_ONE;
          pcm.samples[1][i] := pcm.samples[1][i] shr (MAD_F_FRACBITS + 1 - 16 + 1);
        end;
      end else begin
        for i := 0 to pcm.length -1 do
        begin
          if pcm.samples[0][i] >= MAD_F_ONE then
            pcm.samples[0][i] := MAD_F_ONE - 1;
          if pcm.samples[0][i] < -MAD_F_ONE then
            pcm.samples[0][i] := -MAD_F_ONE;
          pcm.samples[0][i] := pcm.samples[0][i] shr (MAD_F_FRACBITS + 1 - 16 + 1);
          pcm.samples[1][i] := pcm.samples[0][i];
        end;
      end;

      decoder^.sampleofs := 0;
      decoder^.samplecnt := decoder^.synth.pcm.length;
      decoder^.sample_rate := decoder^.synth.pcm.samplerate;
    end;

    len := num div 4;
    if len > decoder^.samplecnt then
      len := decoder^.samplecnt;

    for i := 0 to len - 1 do
    begin
      pcint16(ptrint(buffer) + ofs + 0)^ := decoder^.synth.pcm.samples[0][decoder^.sampleofs];
      pcint16(ptrint(buffer) + ofs + 2)^ := decoder^.synth.pcm.samples[1][decoder^.sampleofs];

      Inc(decoder^.sampleofs);
      Dec(decoder^.samplecnt);
      ofs := ofs + 4;
      num := num - 4;
    end;
  end;

  Result := ofs;
end;
{$ENDIF}

end.
