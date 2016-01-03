{***********************************************************
Copyright 1991 by Digital Equipment Corporation, Maynard, Massachusetts,
and the Massachusetts Institute of Technology, Cambridge, Massachusetts.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its 
documentation for any purpose and without fee is hereby granted, 
provided that the above copyright notice appear in all copies and that
both that copyright notice and this permission notice appear in 
supporting documentation, and that the names of Digital or MIT not be
used in advertising or publicity pertaining to distribution of the
software without specific, written prior permission.  

DIGITAL DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING
ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO EVENT SHALL
DIGITAL BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR
ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
SOFTWARE.

******************************************************************}
{ $XFree86: xc/include/extensions/Xvlib.h,v 1.3 1999/12/11 19:28:48 mvojkovi Exp $ }

{*
** File: 
**
**   Xvlib.h --- Xv library public header file
**
** Author: 
**
**   David Carver (Digital Workstation Engineering/Project Athena)
**
** Revisions:
**
**   26.06.91 Carver
**     - changed XvFreeAdaptors to XvFreeAdaptorInfo
**     - changed XvFreeEncodings to XvFreeEncodingInfo
**
**   11.06.91 Carver
**     - changed SetPortControl to SetPortAttribute
**     - changed GetPortControl to GetPortAttribute
**     - changed QueryBestSize
**
**   05.15.91 Carver
**     - version 2.0 upgrade
**
**   01.24.91 Carver
**     - version 1.4 upgrade
**
*}

unit xvlib;

interface

uses
  ctypes, x, xlib, xshm, xv;

{$PACKRECORDS c}

const
  libXv='Xv';

type
  PXvRational = ^TXvRational;
  TXvRational = record
    numerator : cint;
    denominator : cint;
  end;

  PXvAttribute = ^TXvAttribute;
  TXvAttribute = record
    flags : cint; { XvGettable, XvSettable }
    min_value : cint;
    max_value : cint;
    name : pchar;
  end;

  PPXvEncodingInfo = ^PXvEncodingInfo;
  PXvEncodingInfo = ^TXvEncodingInfo;
  TXvEncodingInfo = record
    encoding_id : TXvEncodingID;
    name : pchar;
    width : culong;
    height : culong;
    rate : TXvRational;
    num_encodings : culong;
  end;

  PXvFormat = ^TXvFormat;
  TXvFormat = record
    depth : cchar;
    visual_id : culong;
  end;

  PPXvAdaptorInfo = ^PXvAdaptorInfo;
  PXvAdaptorInfo = ^TXvAdaptorInfo;
  TXvAdaptorInfo = record
    base_id : TXvPortID;
    num_ports : culong;
    _type : cchar;
    name : pchar;
    num_formats : culong;
    formats : PXvFormat;
    num_adaptors : culong;
  end;

  PXvVideoNotifyEvent = ^TXvVideoNotifyEvent;
  TXvVideoNotifyEvent = record
    _type : cint;
    serial : culong;      { # of last request processed by server }
    send_event : TBool;   { true if this came from a SendEvent request }
    display : PDisplay;   { Display the event was read from }
    drawable : TDrawable; { drawable }
    reason : culong;      { what generated this event }
    port_id : TXvPortID;  { what port }
    time : TTime;         { milliseconds }
  end;

  PXvPortNotifyEvent = ^TXvPortNotifyEvent;
  TXvPortNotifyEvent = record
    _type : cint;
    serial : culong;     { # of last request processed by server }
    send_event : TBool;  { true if this came from a SendEvent request }
    display : PDisplay;  { Display the event was read from }
    port_id : TXvPortID; { what port }
    time : TTime;        { milliseconds }
    attribute : TAtom;   { atom that identifies attribute }
    value : clong;       { value of attribute }
  end;

  PXvEvent = ^TXvEvent;
  TXvEvent = record
    case longint of
      0 : (
            _type : cint;
	  );
      1 : (
            xvvideo : TXvVideoNotifyEvent;
          );
      2 : (
            xvport : TXvPortNotifyEvent;
          );
      3 : (
            pad : array[0..23] of clong;
          );
  end;

  PXvImageFormatValues = ^TXvImageFormatValues;
  TXvImageFormatValues = record
    id : cint;                    { Unique descriptor for the format }
    _type : cint;                 { XvRGB, XvYUV }
    byte_order : cint;            { LSBFirst, MSBFirst }
    guid : array[0..15] of cchar; { Globally Unique IDentifier }
    bits_per_pixel : cint;
    format : cint;                { XvPacked, XvPlanar }
    num_planes : cint;

  { for RGB formats only }
    depth : cint;
    red_mask : cuint;
    green_mask : cuint;
    blue_mask : cuint;

  { for YUV formats only }
    y_sample_bits : cuint;
    u_sample_bits : cuint;
    v_sample_bits : cuint;
    horz_y_period : cuint;
    horz_u_period : cuint;
    horz_v_period : cuint;
    vert_y_period : cuint;
    vert_u_period : cuint;
    vert_v_period : cuint;
    component_order : array[0..31] of char; { eg. UYVY }
    scanline_order : cint;                  { XvTopToBottom, XvBottomToTop }
  end;

  PXvImage = ^TXvImage;
  TXvImage = record
    id : cint;
    width, height : cint;
    data_size : cint;              { bytes }
    num_planes : cint;
    pitches : pcint;               { bytes }
    offsets : pcint;               { bytes }
    data : pcchar;
    obdata : TXPointer;
  end;

function XvQueryExtension(
  display : PDisplay;
  p_version,
  p_revision,
  p_requestBase,
  p_eventBase,
  p_errorBase : pcuint
) : cint; cdecl; external libXv;

function XvQueryAdaptors(
  display : PDisplay;
  window : TWindow;
  p_nAdaptors : pcuint;
  p_pAdaptors : PPXvAdaptorInfo
) : cint; cdecl; external libXv;

function XvQueryEncodings(
  display : PDisplay;
  port : TXvPortID;
  p_nEncoding : pcuint;
  p_pEncoding : PPXvEncodingInfo
) : cint; cdecl; external libXv;

function XvPutVideo(
  display : PDisplay;
  port : TXvPortID;
  d : TDrawable;
  gc : TGC;
  vx,
  vy : cint;
  vw,
  vh : cuint;
  dx,
  dy : cint;
  dw,
  dh : cuint
) : cint; cdecl; external libXv;

function XvPutStill(
  display : PDisplay;
  port : TXvPortID;
  d : TDrawable;
  gc : TGC;
  vx,
  vy : cint;
  vw,
  vh : cuint;
  dx,
  dy : cint;
  dw,
  dh : cuint
) : cint; cdecl; external libXv;

function XvGetVideo(
  display : PDisplay;
  port : TXvPortID;
  d : TDrawable;
  gc : TGC;
  vx,
  vy : cint;
  vw,
  vh : cuint;
  dx,
  dy : cint;
  dw,
  dh : cuint
) : cint; cdecl; external libXv;

function XvGetStill(
  display : PDisplay;
  port : TXvPortID;
  d : TDrawable;
  gc : TGC;
  vx,
  vy : cint;
  vw,
  vh : cuint;
  dx,
  dy : cint;
  dw,
  dh : cuint
) : cint; cdecl; external libXv;

function XvStopVideo(
  display : PDisplay;
  port : TXvPortID;
  drawable : TDrawable
) : cint; cdecl; external libXv;

function XvGrabPort(
  display : PDisplay;
  port : TXvPortID;
  time : TTime
) : cint; cdecl; external libXv;

function XvUngrabPort(
  display : PDisplay;
  port : TXvPortID;
  time : TTime
) : cint; cdecl; external libXv;

function XvSelectVideoNotify(
  display : PDisplay;
  drawable : TDrawable;
  onoff : TBool
) : cint; cdecl; external libXv;

function XvSelectPortNotify(
  display : PDisplay;
  port : TXvPortID;
  onoff : TBool
) : cint; cdecl; external libXv;

function XvSetPortAttribute(
  display : PDisplay;
  port : TXvPortID;
  attribute : TAtom;
  value : cint
) : cint; cdecl; external libXv;

function XvGetPortAttribute(
  display : PDisplay;
  port : TXvPortID;
  attribute : TAtom;
  p_value : pcint
) : cint; cdecl; external libXv;

function XvQueryBestSize(
  display : PDisplay;
  port : TXvPortID;
  motion : TBool;
  vid_w,
  vid_h,
  drw_w,
  drw_h : cuint;
  p_actual_width,
  p_actual_height : pcuint
) : cint; cdecl; external libXv;

function XvQueryPortAttributes(
  display : PDisplay;
  port : TXvPortID;
  number : pcint
) : PXvAttribute; cdecl; external libXv;


procedure XvFreeAdaptorInfo(
  adaptors : PXvAdaptorInfo
); cdecl; external libXv;

procedure XvFreeEncodingInfo(
  encodings : PXvEncodingInfo
); cdecl; external libXv;


function XvListImageFormats (
  display : PDisplay;
  port_id : TXvPortID;
  count_return : pcint
) : PXvImageFormatValues; cdecl; external libXv;

function XvCreateImage (
  display : PDisplay;
  port : TXvPortID;
  id : cint;
  data : pcchar;
  width,
  height : cint
) : PXvImage; cdecl; external libXv;

function XvPutImage (
  display : PDisplay;
  id : TXvPortID;
  d : TDrawable;
  gc : TGC;
  image : PXvImage;
  src_x,
  src_y : cint;
  src_w,
  src_h : cuint;
  dest_x,
  dest_y : cint;
  dest_w,
  dest_h : cuint
) : cint; cdecl; external libXv;

function XvShmPutImage (
  display : PDisplay;
  id : TXvPortID;
  d : TDrawable;
  gc : TGC;
  image : PXvImage;
  src_x,
  src_y : cint;
  src_w,
  src_h : cuint;
  dest_x,
  dest_y : cint;
  dest_w,
  dest_h : cuint;
  send_event : TBool
) : cint; cdecl; external libXv;

function XvShmCreateImage(
  display : PDisplay;
  port : TXvPortID;
  id : cint;
  data : pcchar;
  width,
  height : cint;
  shminfo : PXShmSegmentInfo
) : PXvImage; cdecl; external libXv;

implementation

end.
