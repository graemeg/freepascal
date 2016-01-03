{
  Translation of the vorbis headers for FreePascal
  Copyright (C) 2006 by Ivo Steinmann
}

(********************************************************************
 *                                                                  *
 * THIS FILE IS PART OF THE OggVorbis SOFTWARE CODEC SOURCE CODE.   *
 * USE, DISTRIBUTION AND REPRODUCTION OF THIS LIBRARY SOURCE IS     *
 * GOVERNED BY A BSD-STYLE SOURCE LICENSE INCLUDED WITH THIS SOURCE *
 * IN 'COPYING'. PLEASE READ THESE TERMS BEFORE DISTRIBUTING.       *
 *                                                                  *
 * THE OggVorbis SOURCE CODE IS (C) COPYRIGHT 1994-2001             *
 * by the XIPHOPHORUS Company http://www.xiph.org/                  *
 *                                                                  *
 ********************************************************************)

unit vorbis;

{$mode objfpc}
{$MINENUMSIZE 4}
{$PACKRECORDS C}

interface

uses
  ctypes, ogg;

{$IFDEF WINDOWS}
  {$DEFINE DYNLINK}
{$ENDIF}

{$IFDEF DYNLINK}
const
{$IF Defined(WINDOWS)}
  vorbislib     = 'vorbis.dll';
  vorbisfilelib = 'vorbisfile.dll';
  vorbisenclib  = 'vorbisenc.dll';
{$ELSEIF Defined(UNIX)}
  vorbislib     = 'libvorbis.so';
  vorbisfilelib = 'libvorbisfile.so';
  vorbisenclib  = 'libvorbisenc.so';
{$ELSE}
  {$MESSAGE ERROR 'DYNLINK not supported'}
{$IFEND}
{$ELSE}
  {$LINKLIB vorbis}
  {$LINKLIB vorbisfile}
  {$LINKLIB vorbisenc}
{$ENDIF}

(***********************************************************************)
(* Header : codec.h                                                    *)
(***********************************************************************)

type
<<<<<<< HEAD
<<<<<<< HEAD
=======
  csize_t = culong;

>>>>>>> graemeg/fixes_2_2
=======
  csize_t = culong;

>>>>>>> origin/fixes_2_2
  ppcfloat = ^pcfloat;

  pvorbis_info = ^vorbis_info;
  vorbis_info = record
    version         : cint;
    channels        : cint;
    rate            : clong;

  { The below bitrate declarations are *hints*.
     Combinations of the three values carry the following implications:

     all three set to the same value:
       implies a fixed rate bitstream
     only nominal set:
       implies a VBR stream that averages the nominal bitrate.  No hard
       upper/lower limit
     upper and or lower set:
       implies a VBR bitstream that obeys the bitrate limits. nominal
       may also be set to give a nominal rate.
     none set:
       the coder does not care to speculate.
  }

    bitrate_upper   : clong;
    bitrate_nominal : clong;
    bitrate_lower   : clong;
    bitrate_window  : clong;
    codec_setup     : pointer;
  end;

{ vorbis_dsp_state buffers the current vorbis audio analysis/synthesis state.  The DSP state belongs to a specific logical bitstream }

  pvorbis_dsp_state = ^vorbis_dsp_state;
  vorbis_dsp_state = record
    analysisp       : cint;
    vi              : pvorbis_info;

    pcm             : ppcfloat;
    pcmret          : ppcfloat;
    pcm_storage     : cint;
    pcm_current     : cint;
    pcm_returned    : cint;

    preextrapolate  : cint;
    eofflag         : cint;

    lW              : clong;
    W               : clong;
    nW              : clong;
    centerW         : clong;

    granulepos      : ogg_int64_t;
    sequence        : ogg_int64_t;

    glue_bits       : ogg_int64_t;
    time_bits       : ogg_int64_t;
    floor_bits      : ogg_int64_t;
    res_bits        : ogg_int64_t;

    backend_state   : pointer;
  end;

{ vorbis_block is a single block of data to be processed as part of
  the analysis/synthesis stream; it belongs to a specific logical
  bitstream, but is independant from other vorbis_blocks belonging to
  that logical bitstream. }

  palloc_chain = ^alloc_chain;
  alloc_chain = record
    ptr             : pointer;
    next            : palloc_chain;
  end;

  pvorbis_block = ^vorbis_block;
  vorbis_block = record
  { necessary stream state for linking to the framing abstraction }
    pcm             : ppcfloat;            { this is a pointer into local storage }
    opb             : oggpack_buffer;

    lW              : clong;
    W               : clong;
    nW              : clong;
    pcmend          : cint;
    mode            : cint;

    eofflag         : cint;
    granulepos      : ogg_int64_t;
    sequence        : ogg_int64_t;
    vd              : pvorbis_dsp_state; { For read-only access of configuration }

  { local storage to avoid remallocing; it's up to the mapping to structure it }
    localstore      : pointer;
    localtop        : clong;
    localalloc      : clong;
    totaluse        : clong;
    reap            : palloc_chain;

  { bitmetrics for the frame }
    glue_bits       : clong;
    time_bits       : clong;
    floor_bits      : clong;
    res_bits        : clong;

    internal        : pointer;
  end;

{ vorbis_info contains all the setup information specific to the
  specific compression/decompression mode in progress (eg,
  psychoacoustic settings, channel setup, options, codebook
  etc). vorbis_info and substructures are in backends.h. }

{ the comments are not part of vorbis_info so that vorbis_info can be static storage }

  pvorbis_comment = ^vorbis_comment;
  vorbis_comment = record
  { unlimited user comment fields.  libvorbis writes 'libvorbis' whatever vendor is set to in encode }
    user_comments   : ^pcchar;
    comment_lengths : pcint;
    comments        : cint;
    vendor          : pcchar;
  end;


{ libvorbis encodes in two abstraction layers; first we perform DSP
  and produce a packet (see docs/analysis.txt).  The packet is then
  coded into a framed OggSquish bitstream by the second layer (see
  docs/framing.txt).  Decode is the reverse process; we sync/frame
  the bitstream and extract individual packets, then decode the
  packet back into PCM audio.

  The extra framing/packetizing is used in streaming formats, such as
  files.  Over the net (such as with UDP), the framing and
  packetization aren't necessary as they're provided by the transport
  and the streaming layer is not used }

{ Vorbis PRIMITIVES: general }

procedure vorbis_info_init(var vi: vorbis_info); cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
procedure vorbis_info_clear(var vi: vorbis_info); cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_info_blocksize(var vi: vorbis_info; zo: cint): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
procedure vorbis_comment_init(var vc: vorbis_comment); cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
procedure vorbis_comment_add(var vc: vorbis_comment; comment: pchar); cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
procedure vorbis_comment_add_tag(var vc: vorbis_comment; tag: pchar; contents: pchar); cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_comment_query(var vc: vorbis_comment; tag: pchar; count: cint): pchar; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_comment_query_count(var vc: vorbis_comment; tag: pchar): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
procedure vorbis_comment_clear(var vc: vorbis_comment); cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};

function  vorbis_block_init(var v: vorbis_dsp_state; var vb: vorbis_block): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_block_clear(var vb: vorbis_block): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
procedure vorbis_dsp_clear(var v: vorbis_dsp_state); cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_granule_time(var v: vorbis_dsp_state; granulepos: ogg_int64_t): cdouble; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};

{ vorbislib PRIMITIVES: analysis/DSP layer }

function  vorbis_analysis_init(var v: vorbis_dsp_state; var vi: vorbis_info): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_commentheader_out(var vc: vorbis_comment; var op: ogg_packet): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_analysis_headerout(var v:vorbis_dsp_state; var vc: vorbis_comment; var op: ogg_packet; var op_comm: ogg_packet; var op_code: ogg_packet): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_analysis_buffer(var v: vorbis_dsp_state; vals: cint): ppcfloat; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_analysis_wrote(var v: vorbis_dsp_state; vals: cint): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_analysis_blockout(var v: vorbis_dsp_state; var vb: vorbis_block): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_analysis(var vb: vorbis_block; var op: ogg_packet): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};

function  vorbis_bitrate_addblock(var vb: vorbis_block): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_bitrate_flushpacket(var vd: vorbis_dsp_state; var op: ogg_packet): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};

{ vorbislib PRIMITIVES: synthesis layer }

function  vorbis_synthesis_headerin(var vi: vorbis_info; var vc: vorbis_comment; var op: ogg_packet): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};

function  vorbis_synthesis_init(var v: vorbis_dsp_state; var vi: vorbis_info): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_synthesis_restart(var v: vorbis_dsp_state): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_synthesis(var vb: vorbis_block; var op: ogg_packet): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_synthesis_trackonly(var vb: vorbis_block; var op: ogg_packet): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_synthesis_blockin(var v: vorbis_dsp_state; var vb: vorbis_block): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_synthesis_pcmout(var v: vorbis_dsp_state; var pcm: ppcfloat): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_synthesis_lapout(var v: vorbis_dsp_state; var pcm: ppcfloat): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_synthesis_read(var v: vorbis_dsp_state; samples: cint): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_packet_blocksize(var vi: vorbis_info; var op: ogg_packet): clong; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};

function  vorbis_synthesis_halfrate(var v: vorbis_info; flag: cint): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};
function  vorbis_synthesis_halfrate_p(var v: vorbis_info): cint; cdecl; external {$IFDEF DYNLINK}vorbislib{$ENDIF};

{ vorbislib ERRORS and return codes }
Const
  OV_FALSE          = -1;
  OV_EOF            = -2;
  OV_HOLE           = -3;

  OV_EREAD          = -128;
  OV_EFAULT         = -129;
  OV_EIMPL          = -130;
  OV_EINVAL         = -131;
  OV_ENOTVORBIS     = -132;
  OV_EBADHEADER     = -133;
  OV_EVERSION       = -134;
  OV_ENOTAUDIO      = -135;
  OV_EBADPACKET     = -136;
  OV_EBADLINK       = -137;
  OV_ENOSEEK        = -138;


(***********************************************************************)
(* Header : vorbisfile.h                                               *)
(***********************************************************************)

type

{* The function prototypes for the callbacks are basically the same as for

 * the stdio functions fread, fseek, fclose, ftell.
 * The one difference is that the FILE * arguments have been replaced with
 * a void * - this is to be used as a pointer to whatever internal data these
 * functions might need. In the stdio case, it's just a FILE * cast to a void *
 *
 * If you use other functions, check the docs for these functions and return
 * the right values. For seek_func(), you *MUST* return -1 if the stream is
 * unseekable
 *}

  read_func  = function(ptr: pointer; size, nmemb: csize_t; datasource: pointer): csize_t; cdecl;
  seek_func  = function(datasource: pointer; offset: ogg_int64_t; whence: cint): cint; cdecl;
  close_func = function(datasource: pointer): cint; cdecl;
  tell_func  = function(datasource: pointer): clong; cdecl;

  pov_callbacks = ^ov_callbacks;
  ov_callbacks = record
    read            : read_func;
    seek            : seek_func;
    close           : close_func;
    tell            : tell_func;
  end;

const
  NOTOPEN           = 0;
  PARTOPEN          = 1;
  OPENED            = 2;
  STREAMSET         = 3;
  INITSET           = 4;

type
  POggVorbis_File = ^OggVorbis_File;
  OggVorbis_File = record
    datasource      : pointer; { pointer to a FILE *, etc. }
    seekable        : cint;
    offset          : ogg_int64_t;
    end_            : ogg_int64_t;
    oy              : ogg_sync_state;

  { If the FILE handle isn't seekable (eg, a pipe), only the current stream appears }
    links           : cint;
    offsets         : pogg_int64_t;
    dataoffsets     : pogg_int64_t;
    serialnos       : pclong;
    pcmlengths      : pogg_int64_t; { overloaded to maintain binary compatability; x2 size, stores both beginning and end values }
    vi              : pvorbis_info;
    vc              : pvorbis_comment;

  { Decoding working state local storage }
    pcm_offset      : ogg_int64_t;
    ready_state     : cint;
    current_serialno: clong;
    current_link    : cint;

    bittrack        : cdouble;
    samptrack       : cdouble;

    os              : ogg_stream_state; { take physical pages, weld into a logical stream of packets }
    vd              : vorbis_dsp_state; { central working state for the packet->PCM decoder }
    vb              : vorbis_block;     { local working space for packet->PCM decode }

    callbacks       : ov_callbacks;
  end;


function ov_clear(var vf: OggVorbis_File): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_open(f: pointer; var vf: OggVorbis_File; initial: pointer; ibytes: clong): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_open_callbacks(datasource: pointer; var vf: OggVorbis_File; initial: pointer; ibytes: clong; callbacks: ov_callbacks): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};

function ov_test(f: pointer; var vf: OggVorbis_File; initial: pointer; ibytes: clong): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_test_callbacks(datasource: pointer; var vf: OggVorbis_File; initial: pointer; ibytes: clong; callbacks: ov_callbacks): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_test_open(var vf: OggVorbis_File): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};

function ov_bitrate(var vf: OggVorbis_File; i: cint): clong; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_bitrate_instant(var vf: OggVorbis_File): clong; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_streams(var vf: OggVorbis_File): clong; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_seekable(var vf: OggVorbis_File): clong; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_serialnumber(var vf: OggVorbis_File; i: cint): clong; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};

function ov_raw_total(var vf: OggVorbis_File; i: cint): ogg_int64_t; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_pcm_total(var vf: OggVorbis_File; i: cint): ogg_int64_t; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_time_total(var vf: OggVorbis_File; i: cint): cdouble; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};

function ov_raw_seek(var vf: OggVorbis_File; pos: ogg_int64_t): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_pcm_seek(var vf: OggVorbis_File; pos: ogg_int64_t): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_pcm_seek_page(var vf: OggVorbis_File; pos: ogg_int64_t): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_time_seek(var vf: OggVorbis_File; pos: cdouble): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_time_seek_page(var vf: OggVorbis_File; pos: cdouble): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};

function ov_raw_seek_lap(var vf: OggVorbis_File; pos: ogg_int64_t): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_pcm_seek_lap(var vf: OggVorbis_File; pos: ogg_int64_t): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_pcm_seek_page_lap(var vf: OggVorbis_File; pos: ogg_int64_t): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_time_seek_lap(var vf: OggVorbis_File; pos: cdouble): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_time_seek_page_lap(var vf: OggVorbis_File; pos: cdouble): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};

function ov_raw_tell(var vf: OggVorbis_File): ogg_int64_t; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_pcm_tell(var vf: OggVorbis_File): ogg_int64_t; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_time_tell(var vf: OggVorbis_File): cdouble; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};

function ov_info(var vf: OggVorbis_File; link: cint): pvorbis_info; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_comment(var vf: OggVorbis_File; link: cint): pvorbis_comment; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};

function ov_read_float(var vf: OggVorbis_File; var pcm_channels: ppcfloat; samples: cint; bitstream: pcint): clong; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_read(var vf: OggVorbis_File; buffer: pointer; length: cint; bigendianp: cbool; word: cint; sgned: cbool; bitstream: pcint): clong; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_crosslap(var vf1: OggVorbis_File; var vf2: OggVorbis_File): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};

function ov_halfrate(var vf: OggVorbis_File; flag: cint): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};
function ov_halfrate_p(var vf: OggVorbis_File): cint; cdecl; external {$IFDEF DYNLINK}vorbisfilelib{$ENDIF};


{
  Developer of the A52 helpers for FreePascal
  Copyright (C) 2006 by Ivo Steinmann
}

function ov_read_ext(var vf: OggVorbis_File; buffer: pointer; length: cint; bigendianp: cbool; word: cint; sgned: cbool): clong;


(***********************************************************************)
(* Header : vorbisenc.h                                                *)
(***********************************************************************)

const
  OV_ECTL_RATEMANAGE_GET       = $10;

  OV_ECTL_RATEMANAGE_SET       = $11;
  OV_ECTL_RATEMANAGE_AVG       = $12;
  OV_ECTL_RATEMANAGE_HARD      = $13;

  OV_ECTL_LOWPASS_GET          = $20;
  OV_ECTL_LOWPASS_SET          = $21;

  OV_ECTL_IBLOCK_GET           = $30;
  OV_ECTL_IBLOCK_SET           = $31;

type
  povectl_ratemanage_arg = ^ovectl_ratemanage_arg;
  ovectl_ratemanage_arg = record
    management_active        : cint;

    bitrate_hard_min         : clong;
    bitrate_hard_max         : clong;
    bitrate_hard_window      : cdouble;

    bitrate_av_lo            : clong;
    bitrate_av_hi            : clong;
    bitrate_av_window        : cdouble;
    bitrate_av_window_center : cdouble;
  end;

function vorbis_encode_init(var vi: vorbis_info; channels, rate, max_bitrate, nominal_bitrate, min_bitrate: clong): cint; cdecl; external {$IFDEF DYNLINK}vorbisenclib{$ENDIF};
function vorbis_encode_setup_managed(var vi: vorbis_info; channels, rate, max_bitrate, nominal_bitrate, min_bitrate: clong): cint; cdecl; external {$IFDEF DYNLINK}vorbisenclib{$ENDIF};
function vorbis_encode_setup_vbr(var vi: vorbis_info; channels, rate: clong; quality: cfloat): cint; cdecl; external {$IFDEF DYNLINK}vorbisenclib{$ENDIF};
(* quality level from 0. (lo) to 1. (hi) *)
function vorbis_encode_init_vbr(var vi: vorbis_info; channels, rate: clong; base_quality: cfloat): cint; cdecl; external {$IFDEF DYNLINK}vorbisenclib{$ENDIF};
function vorbis_encode_setup_init(var vi: vorbis_info): cint; cdecl; external {$IFDEF DYNLINK}vorbisenclib{$ENDIF};
function vorbis_encode_ctl(var vi: vorbis_info; number: cint; arg: pointer): cint; cdecl; external {$IFDEF DYNLINK}vorbisenclib{$ENDIF};

implementation

function ov_read_ext(var vf: OggVorbis_File; buffer: pointer; length: cint; bigendianp: cbool; word: cint; sgned: cbool): clong;
var
  ofs: cint;
  Num: cint;
  Res: cint;
begin
  // check blocksize here!
  {if length mod 4 <> 0 then
    Exit(0);}

  ofs := 0;
  num := length;

  while num > 0 do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
    res := ov_read(vf, pointer(ptruint(buffer) + ofs), num, bigendianp, word, sgned, nil);
=======
    res := ov_read(vf, pointer(ptrint(buffer) + ofs), num, bigendianp, word, sgned, nil);
>>>>>>> graemeg/fixes_2_2
=======
    res := ov_read(vf, pointer(ptrint(buffer) + ofs), num, bigendianp, word, sgned, nil);
>>>>>>> origin/fixes_2_2
    if res < 0 then
      Exit(res);

    if res = 0 then
      Break;

    ofs := ofs + res;
    num := num - res;
  end;

  Result := ofs;
end;

end.

