{
  Translation of the DTS headers for FreePascal
  Copyright (C) 2006 by Ivo Steinmann
}

(*
 * dts.h
 * Copyright (C) 2004 Gildas Bazin <gbazin@videolan.org>
 *
 * This file is part of dtsdec, a free DTS Coherent Acoustics stream decoder.
 * See http://www.videolan.org/dtsdec.html for updates.
 *
 * dtsdec is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * dtsdec is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *)

{
Using the libdts API
--------------------

libdts provides a low-level interface to decoding audio frames encoded
using DTS Coherent Acoustics. libdts provides downmixing and
dynamic range compression for the following output configurations:

DTS_CHANNEL  : Dual mono. Two independant mono channels.
DTS_CHANNEL1 : First of the two mono channels above.
DTS_CHANNEL2 : Second of the two mono channels above.
DTS_MONO     : Mono.
DTS_STEREO   : Stereo.
DTS_DOLBY    : Dolby surround compatible stereo.
DTS_3F       : 3 front channels (left, center, right)
DTS_2F1R     : 2 front, 1 rear surround channel (L, R, S)
DTS_3F1R     : 3 front, 1 rear surround channel (L, C, R, S)
DTS_2F2R     : 2 front, 2 rear surround channels (L, R, LS, RS)
DTS_3F2R     : 3 front, 2 rear surround channels (L, C, R, LS, RS)

DTS_LFE      : Low frequency effects channel. Normally used to connect a
               subwoofer. Can be combined with any of the above channels.
               For example: DTS_3F2R | DTS_LFE -> 3 front, 2 rear, 1 LFE (5.1)


Initialization
--------------

dts_state_t * dts_init (uint32_t mm_accel);

Initializes the DTS library. Takes as a parameter the acceptable
optimizations which may be used, such as MMX. These are found in the
included header file 'mm_accel', along with an autodetection function
(mm_accel()). Currently, there is no accelleration implemented.

The return value is a pointer to a dts state object.


Probing the bitstream
---------------------

int dts_syncinfo (uint8_t * buf, int * flags,
                  int * sample_rate, int * bit_rate, int * frame_length);

The DTS bitstream is composed of several dts frames concatenated one
after each other. A dts frame is the smallest independantly decodable
unit in the stream.

buf must contain at least 14 bytes from the input stream. If these look
like the start of a valid dts frame, dts_syncinfo() returns the size
of the coded frame in bytes, and fills flags, sample_rate, bit_rate and
frame_length with the information encoded in the stream. The returned size
is guaranteed to be an even number between 96 and 16384 for the 16 bits
version of the bitstream and 109 and 18726 for the 14 bits version.
sample_rate will be the sampling frequency in Hz, bit_rate is for the
compressed stream and is in bits per second, and flags is a description of
the coded channels: the DTS_LFE bit is set if there is an LFE channel coded
in this stream, and by masking flags with DTS_CHANNEL_MASK you will get a
value that describes the full-bandwidth channels, as one of the
DTS_CHANNEL...DTS_3F2R flags.

If this can not possibly be a valid frame, then the function returns
0. You should then try to re-synchronize with the dts stream - one way
to try this would be to advance buf by one byte until its contents
looks like a valid frame, but there might be better
application-specific ways to synchronize.

You need to call this function for each frame, for several
reasons: this function detects errors that the other functions will
not double-check, consecutive frames might have different lengths, and
it helps you re-sync with the stream if you get de-synchronized. It will as
well detect the kind of bitstream it is dealing with (big/little endian,
16/14 bits mode)


Starting to decode a frame
--------------------------

int dts_frame (dts_state_t * state, uint8_t * buf, int * flags,
           sample_t * level, sample_t bias);

This starts the work of decoding the DTS frame (to be completed using
dts_block()). buf should point to the beginning of the complete frame
of the full size returned by dts_syncinfo().

You should pass in the flags the speaker configuration that you
support, and libdts will return the speaker configuration it will use
for its output, based on what is coded in the stream and what you
asked for. For example, if the stream contains 2+2 channels
(dts_syncinfo() returned DTS_2F2R in the flags), and you have 3+1
speakers (you passed DTS_3F1R), then libdts will choose do downmix to
2+1 speakers, since there is no center channel to send to your center
speaker. So in that case the left and right channels will be
essentially unmodified by the downmix, and the two surround channels
will be added together and sent to your surround speaker. libdts will
return DTS_2F1R to indicate this.

The good news is that when you downmix to stereo you dont have to
worry about this, you will ALWAYS get a stereo output no matter what
was coded in the stream. For more complex output configurations you
will have to handle the case where libdts couldnt give you what you
wanted because some of the channels were not encoded in the stream
though.

Level, bias, and DTS_ADJUST_LEVEL:

Before downmixing, samples are floating point values with a range of
[-1,1]. Most types of downmixing will combine channels together, which
will potentially result in a larger range for the output
samples. libdts provides two methods of controlling the range of the
output, either before or after the downmix stage.

If you do not set DTS_ADJUST_LEVEL, libdts will multiply the samples
by your level value, so that they fit in the [-level,level]
range. Then it will apply the standardized downmix equations,
potentially making the samples go out of that interval again. The
level parameter is not modified.

Setting the DTS_ADJUST_LEVEL flag will instruct libdts to treat your
level value as the intended range interval after downmixing. It will
then figure out what level to use before the downmix (what you should
have passed if you hadnt used the DTS_ADJUST_LEVEL flag), and
overwrite the level value you gave it with that new level value.

The bias represents a value which should be added to the result
regardless:

output_sample = (input_sample * level) + bias;

For example, a bias of 384 and a level of 1 tells liba52 you want
samples between 383 and 385 instead of -1 and 1. This is what the
sample program dtsdec does, as it makes it faster to convert the
samples to integer format, using a trick based on the IEEE
floating-point format.

This function also initialises the state for that frame, which will be
reused next when decoding blocks.


Dynamic range compression
-------------------------

void dts_dynrng (dts_state_t * state,
                 sample_t (* call) (sample_t, void *), void * data);

This function is purely optional. If you dont call it, libdts will
provide the default behaviour, which is to apply the full dynamic
range compression as specified in the DTS stream. This basically
makes the loud sounds softer, and the soft sounds louder, so you can
more easily listen to the stream in a noisy environment without
disturbing anyone.

If you do call this function and set a NULL callback, this will
totally disable the dynamic range compression and provide a playback
more adapted to a movie theater or a listening room.

If you call this function and specify a callback function, this
callback might be called up to once for each block, with two
arguments: the compression factor 'c' recommended by the bitstream,
and the private data pointer you specified in dts_dynrng(). The
callback will then return the amount of compression to actually use -
typically pow(c,x) where x is somewhere between 0 and 1. More
elaborate compression functions might want to use a different value
for 'x' depending wether c>1 or c<1 - or even something more complex
if this is what you want.


Finding the number of blocks
----------------------------

int dts_blocks_num (dts_state_t * state);

Every DTS frame is composed of a variable number of blocks. Calling
dts_blocks_num() after dts_frame() will give you the number of blocks in the
current frame.

Decoding blocks
---------------

int dts_block (dts_state_t * state);

Every DTS frame is composed of a variable number of blocks, each with an
output of 256 samples for each channel. The dts_block() function decodes
the next block in the frame, and should be called dts_blocks_num() times to
decode all of the audio in the frame.

Getting the decoded audio samples
---------------------------------

sample_t * dts_samples (dts_state_t * state);

After each call to dts_block(), you should extract the audio data from the
internal samples buffer.

This function returns a pointer to an internal buffer which will contain 256
samples for the first channel, followed by 256 samples for the second
channel, etc... the channel order is center, left, right, left
surround, right surround, LFE. If one of the channels is not present in the
libdts output, as indicated by the flags returned by dts_frame(), then
this channel is skipped and the following channels are shifted so
libdts does not leave an empty space between channels.


Pseudocode example
------------------

dts_state_t * state = dts_init (mm_accel());

loop on input bytes:
  if at least 14 bytes in the buffer:

    bytes_to_get = dts_syncinfo (...)

    if bytes_to_get == 0:
      goto loop to keep looking for sync point
    else
      get rest of bytes

      dts_frame (state, buf, ...)
      [dts_dynrng (state, ...); this is only optional]
      for i = 1 ... dts_blocks_num():
        dts_block (state)
        dts_samples (state)
        convert samples to integer and queue to soundcard
}

unit dts;

{$mode objfpc}
{$MINENUMSIZE 4}

interface

uses
  ctypes;

{$UNDEF LIBA52_DOUBLE}

{$IFDEF WINDOWS}
  {$DEFINE DYNLINK}
{$ENDIF}

{$IFDEF DYNLINK}
const
{$IF Defined(WINDOWS)}
  dtslib = 'dts.dll';
{$ELSEIF Defined(UNIX)}
  dtslib = 'libdts.so';
{$ELSE}
  {$MESSAGE ERROR 'DYNLINK not supported'}
{$IFEND}
{$ELSE}
  {$LINKLIB dts}
{$ENDIF}

(* x86 accelerations *)
const
  MM_ACCEL_X86_MMX    = $80000000;
  MM_ACCEL_X86_3DNOW  = $40000000;
  MM_ACCEL_X86_MMXEXT = $20000000;

function mm_accel: cuint32; cdecl; external {$IFDEF DYNLINK}dtslib{$ENDIF};


type
  pdts_sample_t = ^dts_sample_t;
{$IF Defined(LIBDTS_FIXED)}
  dts_sample_t = cint32;
{$ELSEIF Defined(LIBDTS_DOUBLE)}
  dts_sample_t = cdouble;
{$ELSE}
  dts_sample_t = cfloat;
{$IFEND}

  pdts_level_t = ^dts_level_t;
  dts_level_t = dts_sample_t;

  pdts_state_t = ^dts_state_t;
  dts_state_t = record
  end;

  dts_dynrng_call = function(s: dts_sample_t; data: pointer): dts_level_t; cdecl;

const
  DTS_MONO            = 0;
  DTS_CHANNEL         = 1;
  DTS_STEREO          = 2;
  DTS_STEREO_SUMDIFF  = 3;
  DTS_STEREO_TOTAL    = 4;
  DTS_3F              = 5;
  DTS_2F1R            = 6;
  DTS_3F1R            = 7;
  DTS_2F2R            = 8;
  DTS_3F2R            = 9;
  DTS_4F2R            = 10;

  DTS_DOLBY           = 101; (* FIXME *)

  DTS_CHANNEL_MAX     = DTS_3F2R; (* We don't handle anything above that *)
  DTS_CHANNEL_BITS    = 6;
  DTS_CHANNEL_MASK    = $3F;

  DTS_LFE             = $80;
  DTS_ADJUST_LEVEL    = $100;


function dts_init(mm_accel: cuint32): pdts_state_t; cdecl; external {$IFDEF DYNLINK}dtslib{$ENDIF};
function dts_syncinfo(state: pdts_state_t; buf: pcuint8; var flags: cint; var sample_rate: cint; var bit_rate: cint; var frame_length: cint): cint; cdecl; external {$IFDEF DYNLINK}dtslib{$ENDIF};
function dts_frame(state: pdts_state_t; buf: pcuint8; var flags: cint; var level: dts_level_t; bias: dts_sample_t): cint; cdecl; external {$IFDEF DYNLINK}dtslib{$ENDIF};
procedure dts_dynrng(state: pdts_state_t; call: dts_dynrng_call; data: pointer); cdecl; external {$IFDEF DYNLINK}dtslib{$ENDIF};
function dts_blocks_num(state: pdts_state_t): cint; cdecl; external {$IFDEF DYNLINK}dtslib{$ENDIF};
function dts_block(state: pdts_state_t): cint; cdecl; external {$IFDEF DYNLINK}dtslib{$ENDIF};
function dts_samples(state: pdts_state_t): pdts_sample_t; cdecl; external {$IFDEF DYNLINK}dtslib{$ENDIF};
procedure dts_free(state: pdts_state_t); cdecl; external {$IFDEF DYNLINK}dtslib{$ENDIF};



{
  Developer of the A52 helpers for FreePascal
  Copyright (C) 2006 by Ivo Steinmann
}

type
  dts_read_func  = function(user: pointer; ptr: pointer; size: cuint): cuint; cdecl;
  dts_seek_func  = function(user: pointer; offset: clong; whence: cint): clong; cdecl;
  dts_close_func = function(user: pointer): cint; cdecl;
  dts_tell_func  = function(user: pointer): clong; cdecl;

  pdts_decoder = ^dts_decoder;
  dts_decoder = record
    inbuf       : array[0..24576-1] of cuint8;
    inbuf_ptr   : pcuint8;
    frame_size  : cint;
    state       : pdts_state_t;
    fsamples    : pdts_sample_t; // internal samples buffer of dts (returned by dts_samples)

    samples     : array[0..1,0..6*256-1] of cint16;
    samplecnt   : cint;
    sampleofs   : cint;
    user        : pointer;
    read        : dts_read_func;
    seek        : dts_seek_func;
    close       : dts_close_func;
    tell        : dts_tell_func;

  // Userinfo
    flags       : cint;
    channels    : cint;
    sample_rate : cint;
    bit_rate    : cint;
  end;


function dts_decoder_init(mm_accel: cuint32; user: pointer; read: dts_read_func; seek: dts_seek_func; close: dts_close_func; tell: dts_tell_func): pdts_decoder;
function dts_decoder_read(decoder: pdts_decoder; buffer: pointer; length: cint): cint;
procedure dts_decoder_free(decoder: pdts_decoder);

implementation

function dts_decoder_init(mm_accel: cuint32; user: pointer; read: dts_read_func; seek: dts_seek_func; close: dts_close_func; tell: dts_tell_func): pdts_decoder;
begin
  GetMem(Result, Sizeof(dts_decoder));
  FillChar(Result^, Sizeof(dts_decoder), 0);
  Result^.state := dts_init(mm_accel);
  Result^.fsamples := dts_samples(Result^.state);
  Result^.inbuf_ptr := @Result^.inbuf[0];
  Result^.user := user;
  Result^.read := read;
  Result^.seek := seek;
  Result^.close := close;
  Result^.tell := tell;
end;

procedure dts_decoder_free(decoder: pdts_decoder);
begin
  if not Assigned(decoder) then
    Exit;

  dts_free(decoder^.state);
  decoder^.close(decoder^.user);
  FreeMem(decoder);
end;

function dts_decoder_read(decoder: pdts_decoder; buffer: pointer; length: cint): cint;
const
  HEADER_SIZE = 14;
//  ac3_channels: array[0..7] of cint = (2,1,2,3,3,4,4,5);
var
  num, ofs: cint;
  flags, len, i, j: cint;
  sample_rate, bit_rate: cint;
  level: dts_sample_t;
begin
  // check blocksize here!

  ofs := 0;
  num := length;

  while num > 0 do
  begin
    if decoder^.samplecnt = 0 then
    begin
      len := ptrint(decoder^.inbuf_ptr) - ptrint(@decoder^.inbuf);

      if (len < HEADER_SIZE) or (len < decoder^.frame_size) then
      begin
        (* inbuf too small : enlarge *)
        len := Sizeof(dts_decoder.inbuf) - len;
        len := decoder^.read(decoder^.user, decoder^.inbuf_ptr, len);
        if len <= 0 then
          Exit(ofs);

        Inc(decoder^.inbuf_ptr, len);
      end;

      if decoder^.frame_size = 0 then
      begin
        (* no header seen : find one. We need at least 7 bytes to parse it *)

        len := dts_syncinfo(decoder^.state, @decoder^.inbuf[0], decoder^.flags, sample_rate, bit_rate, i{dummy});
        if len = 0 then
        begin
          (* no sync found : move by one byte (inefficient, but simple!) *)
          Move(decoder^.inbuf[1], decoder^.inbuf[0], ptrint(decoder^.inbuf_ptr) - ptrint(@decoder^.inbuf) - 1);
          Dec(decoder^.inbuf_ptr, 1);
        end else begin
          decoder^.frame_size := len;

          (* update codec info *)
          decoder^.sample_rate := sample_rate;
          decoder^.bit_rate := bit_rate;
          {decoder^.channels := ac3_channels[decoder^.flags and $7];
          if decoder^.flags and A52_LFE <> 0 then
            Inc(decoder^.channels);}

          {WriteLn('  frame_size  : ', decoder^.frame_size);
          WriteLn('  sample_rate : ', sample_rate);
          WriteLn('  bit_rate    : ', bit_rate);
          WriteLn('  channels    : ', decoder^.channels);}
        end;

        Continue;
      end;

      (* decode the frame *)
      flags := DTS_STEREO or DTS_ADJUST_LEVEL;//decoder^.flags;
      level := 0;//High(Smallint)-30;
      (* FIXME dts_frame dont care on level parameters, so I set it to zero and multiply with High(Smallint) later *)

      if dts_frame(decoder^.state, @decoder^.inbuf[0], flags, level, 0) <> 0 then
      begin
        decoder^.inbuf_ptr := @decoder^.inbuf[0];
        decoder^.frame_size := 0;
        Continue;
      end;

      len := dts_blocks_num(decoder^.state);
      for i := 0 to len - 1 do
      begin
        if dts_block(decoder^.state) <> 0 then
        begin
          decoder^.inbuf_ptr := @decoder^.inbuf[0];
          decoder^.frame_size := 0;
          Exit(-1);
        end;

        for j := 0 to 255 do
        begin
          decoder^.samples[0, i*256+j] := Round(High(Smallint)*decoder^.fsamples[j + 000]);
          decoder^.samples[1, i*256+j] := Round(High(Smallint)*decoder^.fsamples[j + 256]);
        end;
      end;

      (* skip decoded frame *)
      Move(decoder^.inbuf[decoder^.frame_size], decoder^.inbuf[0], ptrint(decoder^.inbuf_ptr) - ptrint(@decoder^.inbuf) - decoder^.frame_size);
      Dec(decoder^.inbuf_ptr, decoder^.frame_size);
      decoder^.frame_size := 0;

      decoder^.sampleofs := 0;
      decoder^.samplecnt := len*256;
    end;

    len := num div 4;
    if len > decoder^.samplecnt then
      len := decoder^.samplecnt;

    for i := 0 to len - 1 do
    begin
      pcint16(ptrint(buffer) + ofs + 0)^ := decoder^.samples[0][decoder^.sampleofs];
      pcint16(ptrint(buffer) + ofs + 2)^ := decoder^.samples[1][decoder^.sampleofs];

      Inc(decoder^.sampleofs);
      Dec(decoder^.samplecnt);
      ofs := ofs + 4;
      num := num - 4;
    end;
  end;

  Result := ofs;
end;

end.
