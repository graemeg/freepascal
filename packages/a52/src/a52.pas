{
  Translation of the A52 headers for FreePascal
  Copyright (C) 2006 by Ivo Steinmann
}

(*
 * a52.h
 * Copyright (C) 2000-2002 Michel Lespinasse <walken@zoy.org>
 * Copyright (C) 1999-2000 Aaron Holtzman <aholtzma@ess.engr.uvic.ca>
 *
 * This file is part of a52dec, a free ATSC A-52 stream decoder.
 * See http://liba52.sourceforge.net/ for updates.
 *
 * a52dec is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * a52dec is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *)


{
Using the liba52 API
--------------------

liba52 provides a low-level interface to decoding audio frames encoded
using ATSC standard A/52 aka AC-3. liba52 provides downmixing and
dynamic range compression for the following output configurations:

A52_CHANNEL  : Dual mono. Two independant mono channels.
A52_CHANNEL1 : First of the two mono channels above.
A52_CHANNEL2 : Second of the two mono channels above.
A52_MONO     : Mono.
A52_STEREO   : Stereo.
A52_DOLBY    : Dolby surround compatible stereo.
A52_3F       : 3 front channels (left, center, right)
A52_2F1R     : 2 front, 1 rear surround channel (L, R, S)
A52_3F1R     : 3 front, 1 rear surround channel (L, C, R, S)
A52_2F2R     : 2 front, 2 rear surround channels (L, R, LS, RS)
A52_3F2R     : 3 front, 2 rear surround channels (L, C, R, LS, RS)

A52_LFE      : Low frequency effects channel. Normally used to connect a
               subwoofer. Can be combined with any of the above channels.
               For example: A52_3F2R | A52_LFE -> 3 front, 2 rear, 1 LFE (5.1)


Initialization
--------------

sample_t * a52_init (uint32_t mm_accel);

Initializes the A/52 library. Takes as a parameter the acceptable
optimizations which may be used, such as MMX. These are found in the
included header file 'mm_accel', along with an autodetection function
(mm_accel()). Currently, the only accelleration implemented is
MM_ACCEL_MLIB, which uses the 'mlib' library if installed. mlib is
only available on some Sun Microsystems platforms.

The return value is a pointer to a properly-aligned sample buffer used
for output samples.


Probing the bitstream
---------------------

int a52_syncinfo (uint8_t * buf, int * flags,
                  int * sample_rate, int * bit_rate);

The A/52 bitstream is composed of several a52 frames concatenated one
after each other. An a52 frame is the smallest independantly decodable
unit in the stream.

buf must contain at least 7 bytes from the input stream. If these look
like the start of a valid a52 frame, a52_syncinfo() returns the size
of the coded frame in bytes, and fills flags, sample_rate and bit_rate
with the information encoded in the stream. The returned size is
guaranteed to be an even number between 128 and 3840. sample_rate will
be the sampling frequency in Hz, bit_rate is for the compressed stream
and is in bits per second, and flags is a description of the coded
channels: the A52_LFE bit is set if there is an LFE channel coded in
this stream, and by masking flags with A52_CHANNEL_MASK you will get a
value that describes the full-bandwidth channels, as one of the
A52_CHANNEL...A52_3F2R flags.

If this can not possibly be a valid frame, then the function returns
0. You should then try to re-synchronize with the a52 stream - one way
to try this would be to advance buf by one byte until its contents
looks like a valid frame, but there might be better
application-specific ways to synchronize.

It is recommended to call this function for each frame, for several
reasons: this function detects errors that the other functions will
not double-check, consecutive frames might have different lengths, and
it helps you re-sync with the stream if you get de-synchronized.


Starting to decode a frame
--------------------------

int a52_frame (a52_state_t * state, uint8_t * buf, int * flags,
           sample_t * level, sample_t bias);

This starts the work of decoding the A/52 frame (to be completed using
a52_block()). buf should point to the beginning of the complete frame
of the full size returned by a52_syncinfo().

You should pass in the flags the speaker configuration that you
support, and liba52 will return the speaker configuration it will use
for its output, based on what is coded in the stream and what you
asked for. For example, if the stream contains 2+2 channels
(a52_syncinfo() returned A52_2F2R in the flags), and you have 3+1
speakers (you passed A52_3F1R), then liba52 will choose do downmix to
2+1 speakers, since there is no center channel to send to your center
speaker. So in that case the left and right channels will be
essentially unmodified by the downmix, and the two surround channels
will be added together and sent to your surround speaker. liba52 will
return A52_2F1R to indicate this.

The good news is that when you downmix to stereo you dont have to
worry about this, you will ALWAYS get a stereo output no matter what
was coded in the stream. For more complex output configurations you
will have to handle the case where liba52 couldnt give you what you
wanted because some of the channels were not encoded in the stream
though.

Level, bias, and A52_ADJUST_LEVEL:

Before downmixing, samples are floating point values with a range of
[-1,1]. Most types of downmixing will combine channels together, which
will potentially result in a larger range for the output
samples. liba52 provides two methods of controlling the range of the
output, either before or after the downmix stage.

If you do not set A52_ADJUST_LEVEL, liba52 will multiply the samples
by your level value, so that they fit in the [-level,level]
range. Then it will apply the standardized downmix equations,
potentially making the samples go out of that interval again. The
level parameter is not modified.

Setting the A52_ADJUST_LEVEL flag will instruct liba52 to treat your
level value as the intended range interval after downmixing. It will
then figure out what level to use before the downmix (what you should
have passed if you hadnt used the A52_ADJUST_LEVEL flag), and
overwrite the level value you gave it with that new level value.

The bias represents a value which should be added to the result
regardless:

output_sample = (input_sample * level) + bias;

For example, a bias of 384 and a level of 1 tells liba52 you want
samples between 383 and 385 instead of -1 and 1. This is what the
sample program a52dec does, as it makes it faster to convert the
samples to integer format, using a trick based on the IEEE
floating-point format.

This function also initialises the state for that frame, which will be
reused next when decoding blocks.


Dynamic range compression
-------------------------

void a52_dynrng (a52_state_t * state,
                 sample_t (* call) (sample_t, void *), void * data);

This function is purely optional. If you dont call it, liba52 will
provide the default behaviour, which is to apply the full dynamic
range compression as specified in the A/52 stream. This basically
makes the loud sounds softer, and the soft sounds louder, so you can
more easily listen to the stream in a noisy environment without
disturbing anyone.

If you do call this function and set a NULL callback, this will
totally disable the dynamic range compression and provide a playback
more adapted to a movie theater or a listening room.

If you call this function and specify a callback function, this
callback might be called up to once for each block, with two
arguments: the compression factor 'c' recommended by the bitstream,
and the private data pointer you specified in a52_dynrng(). The
callback will then return the amount of compression to actually use -
typically pow(c,x) where x is somewhere between 0 and 1. More
elaborate compression functions might want to use a different value
for 'x' depending wether c>1 or c<1 - or even something more complex
if this is what you want.


Decoding blocks
---------------

int a52_block (a52_state_t * state, sample_t * samples);

Every A/52 frame is composed of 6 blocks, each with an output of 256
samples for each channel. The a52_block() function decodes the next
block in the frame, and should be called 6 times to decode all of the
audio in the frame. After each call, you should extract the audio data
from the sample buffer.

The sample pointer given should be the one a52_init() returned.

After this function returns, the samples buuffer will contain 256
samples for the first channel, followed by 256 samples for the second
channel, etc... the channel order is LFE, left, center, right, left
surround, right surround. If one of the channels is not present in the
liba52 output, as indicated by the flags returned by a52_frame(), then
this channel is skipped and the following channels are shifted so
liba52 does not leave an empty space between channels.


Pseudocode example
------------------

sample_t * samples = a52_init (mm_accel());

loop on input bytes:
  if at least 7 bytes in the buffer:

    bytes_to_get = a52_syncinfo (...)

    if bytes_to_get == 0:
      goto loop to keep looking for sync point
    else
      get rest of bytes

      a52_frame (state, buf, ...)
      [a52_dynrng (state, ...); this is only optional]
      for i = 1 ... 6:
        a52_block (state, samples)
        convert samples to integer and queue to soundcard
}

unit a52;

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
  a52lib = 'a52.dll';
{$ELSEIF Defined(UNIX)}
  a52lib = 'liba52.so';
{$ELSE}
  {$MESSAGE ERROR 'DYNLINK not supported'}
{$IFEND}
{$ELSE}
  {$LINKLIB a52}
{$ENDIF}

const
(* generic accelerations *)
  MM_ACCEL_DJBFFT     = $00000001;

(* x86 accelerations *)
  MM_ACCEL_X86_MMX    = $80000000;
  MM_ACCEL_X86_3DNOW  = $40000000;
  MM_ACCEL_X86_MMXEXT = $20000000;

function mm_accel: cuint32; cdecl; external {$IFDEF DYNLINK}a52lib{$ENDIF};


// include a52.h

// to avoid type problems with dts.pas, I renamed a52_sample_t to a52_a52_sample_t
type
  pa52_sample_t = ^a52_sample_t;
{$IFDEF LIBA52_DOUBLE}
  a52_sample_t = cdouble;
{$ELSE}
  a52_sample_t = cfloat;
{$ENDIF}

  pa52_state_t = ^a52_state_t;
  a52_state_t = record
  end;

  a52_dynrng_call = function(s: a52_sample_t; data: pointer): a52_sample_t; cdecl;

const
   A52_CHANNEL      = 0;
   A52_MONO         = 1;
   A52_STEREO       = 2;
   A52_3F           = 3;
   A52_2F1R         = 4;
   A52_3F1R         = 5;
   A52_2F2R         = 6;
   A52_3F2R         = 7;
   A52_CHANNEL1     = 8;
   A52_CHANNEL2     = 9;
   A52_DOLBY        = 10;
   A52_CHANNEL_MASK = 15;

   A52_LFE          = 16;
   A52_ADJUST_LEVEL = 32;


function a52_init(mm_accel: cuint32): pa52_state_t; cdecl; external {$IFDEF DYNLINK}a52lib{$ENDIF};
function a52_samples(state: pa52_state_t): pa52_sample_t; cdecl; external {$IFDEF DYNLINK}a52lib{$ENDIF};
function a52_syncinfo(buf: pcuint8; var flags: cint; var sample_rate: cint; var bit_rate: cint): cint; cdecl; external {$IFDEF DYNLINK}a52lib{$ENDIF};
function a52_frame(state: pa52_state_t; buf: pcuint8; var flags: cint; var level: a52_sample_t; bias: a52_sample_t): cint; cdecl; external {$IFDEF DYNLINK}a52lib{$ENDIF};
procedure a52_dynrng(state: pa52_state_t; call: a52_dynrng_call; data: pointer); cdecl; external {$IFDEF DYNLINK}a52lib{$ENDIF};
function a52_block(state: pa52_state_t): cint; cdecl; external {$IFDEF DYNLINK}a52lib{$ENDIF};
procedure a52_free(state: pa52_state_t); cdecl; external {$IFDEF DYNLINK}a52lib{$ENDIF};



{
  Developer of the A52 helpers for FreePascal
  Copyright (C) 2006 by Ivo Steinmann
}

type
  a52_read_func  = function(user: pointer; ptr: pointer; size: cuint): cuint; cdecl;
  a52_seek_func  = function(user: pointer; offset: clong; whence: cint): clong; cdecl;
  a52_close_func = function(user: pointer): cint; cdecl;
  a52_tell_func  = function(user: pointer): clong; cdecl;

  pa52_decoder = ^a52_decoder;
  a52_decoder = record
    inbuf       : array[0..4096-1] of cuint8;
    inbuf_ptr   : pcuint8;
    frame_size  : cint;
    state       : pa52_state_t;
    fsamples    : pa52_sample_t; // internal samples buffer of a52 (returned by a52_samples)

    samples     : array[0..1,0..6*256-1] of cint16;
    samplecnt   : cint;
    sampleofs   : cint;
    user        : pointer;
    read        : a52_read_func;
    seek        : a52_seek_func;
    close       : a52_close_func;
    tell        : a52_tell_func;

  // Userinfo
    flags       : cint;
    channels    : cint;
    sample_rate : cint;
    bit_rate    : cint;
  end;


function a52_decoder_init(mm_accel: cuint32; user: pointer; read: a52_read_func; seek: a52_seek_func; close: a52_close_func; tell: a52_tell_func): pa52_decoder;
function a52_decoder_read(decoder: pa52_decoder; buffer: pointer; length: cint): cint;
procedure a52_decoder_free(decoder: pa52_decoder);

implementation

function a52_decoder_init(mm_accel: cuint32; user: pointer; read: a52_read_func; seek: a52_seek_func; close: a52_close_func; tell: a52_tell_func): pa52_decoder;
begin
  GetMem(Result, Sizeof(a52_decoder));
  FillChar(Result^, Sizeof(a52_decoder), 0);
  Result^.state := a52_init(mm_accel);
  Result^.fsamples := a52_samples(Result^.state);
  Result^.inbuf_ptr := @Result^.inbuf[0];
  Result^.user := user;
  Result^.read := read;
  Result^.seek := seek;
  Result^.close := close;
  Result^.tell := tell;
end;

procedure a52_decoder_free(decoder: pa52_decoder);
begin
  if not Assigned(decoder) then
    Exit;

  a52_free(decoder^.state);
  decoder^.close(decoder^.user);
  FreeMem(decoder);
end;

function a52_decoder_read(decoder: pa52_decoder; buffer: pointer; length: cint): cint;
const
  HEADER_SIZE = 7;
  ac3_channels: array[0..7] of cint = (2,1,2,3,3,4,4,5);
var
  num, ofs: cint;
  flags, len, i, j: cint;
  sample_rate, bit_rate: cint;
  level: a52_sample_t;
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
        len := Sizeof(a52_decoder.inbuf) - len;
        len := decoder^.read(decoder^.user, decoder^.inbuf_ptr, len);
        if len <= 0 then
          Exit(ofs);

        Inc(decoder^.inbuf_ptr, len);
      end;

      if decoder^.frame_size = 0 then
      begin
        (* no header seen : find one. We need at least 7 bytes to parse it *)
        //WriteLn('no header seen (', len, ')');

        len := a52_syncinfo(@decoder^.inbuf[0], decoder^.flags, sample_rate, bit_rate);
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
          decoder^.channels := ac3_channels[decoder^.flags and $7];
          if decoder^.flags and A52_LFE <> 0 then
            Inc(decoder^.channels);

         {WriteLn('  frame_size  : ', decoder^.frame_size);
          WriteLn('  sample_rate : ', sample_rate);
          WriteLn('  bit_rate    : ', bit_rate);
          WriteLn('  channels    : ', decoder^.channels);}
        end;

        Continue;
      end;

      (* decode the frame *)
      flags := A52_STEREO;//decoder^.flags;
      level := High(Smallint)-30;

      if a52_frame(decoder^.state, @decoder^.inbuf[0], flags, level, 0) <> 0 then
      begin
        decoder^.inbuf_ptr := @decoder^.inbuf[0];
        decoder^.frame_size := 0;
        Continue;
      end;

      for i := 0 to 5 do
      begin
        if a52_block(decoder^.state) <> 0 then
        begin
          decoder^.inbuf_ptr := @decoder^.inbuf[0];
          decoder^.frame_size := 0;
          Exit(-1);
        end;

        for j := 0 to 255 do
        begin
          decoder^.samples[0, i*256+j] := Round(decoder^.fsamples[j + 000]);
          decoder^.samples[1, i*256+j] := Round(decoder^.fsamples[j + 256]);
        end;
      end;

      (* skip decoded frame *)
      Move(decoder^.inbuf[decoder^.frame_size], decoder^.inbuf[0], ptrint(decoder^.inbuf_ptr) - ptrint(@decoder^.inbuf) - decoder^.frame_size);
      Dec(decoder^.inbuf_ptr, decoder^.frame_size);
      decoder^.frame_size := 0;

      decoder^.sampleofs := 0;
      decoder^.samplecnt := 6*256;
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
