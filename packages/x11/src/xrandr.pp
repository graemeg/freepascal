{
  $XFree86: xc/lib/Xrandr/Xrandr.h,v 1.9 2002/09/29 23:39:44 keithp Exp $
 
  Copyright (C) 2000 Compaq Computer Corporation, Inc.
  Copyright (C) 2002 Hewlett-Packard Company, Inc.
 
  Permission to use, copy, modify, distribute, and sell this software and its
  documentation for any purpose is hereby granted without fee, provided that
  the above copyright notice appear in all copies and that both that
  copyright notice and this permission notice appear in supporting
  documentation, and that the name of Compaq not be used in advertising or
  publicity pertaining to distribution of the software without specific,
  written prior permission.  HP makes no representations about the
  suitability of this software for any purpose.  It is provided "as is"
  without express or implied warranty.
 
  HP DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING ALL
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO EVENT SHALL COMPAQ
  BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
  OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN 
  CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 
  Author:  Jim Gettys, HP Labs, HP.
}

unit xrandr;

interface

{$PACKRECORDS c}

uses
  ctypes, x, xlib;

const
  libXrandr = 'Xrandr';

{$I randr.inc}

type
  PXRRScreenSize = ^TXRRScreenSize;
  TXRRScreenSize = record
    width, height : cint;
    mwidth, mheight : cint;
  end;

{
   Events.
}

  TXRRScreenChangeNotifyEvent = record
    _type : cint;                 { event base }
    serial : culong;              { # of last request processed by server }
    send_event : TBool;           { true if this came from a SendEvent request }
    display : PDisplay;           { Display the event was read from }
    window : TWindow;             { window which selected for this event }
    root : TWindow;               { Root window for changed screen }
    timestamp : TTime;            { when the screen change occurred }
    config_timestamp : TTime;     { when the last configuration change }
    size_index : TSizeID;
    subpixel_order : TSubpixelOrder;
    rotation : TRotation;
    width : cint;
    height : cint;
    mwidth : cint;
    mheight : cint;
  end;


{ internal representation is private to the library }
  PXRRScreenConfiguration = ^TXRRScreenConfiguration;
  TXRRScreenConfiguration = record end;

function XRRQueryExtension(
  dpy : PDisplay;
  event_basep,
  error_basep : Pcint) : TBool; cdecl; external libXrandr;

function XRRQueryVersion(
  dpy : PDisplay;
  major_versionp : Pcint;
  minor_versionp : Pcint) : TStatus; cdecl; external libXrandr;

function XRRGetScreenInfo(
  dpy : PDisplay;
  draw : TDrawable) : PXRRScreenConfiguration; cdecl; external libXrandr;

procedure XRRFreeScreenConfigInfo(
  config : PXRRScreenConfiguration); cdecl; external libXrandr;

{
  Note that screen configuration changes are only permitted if the client can
  prove it has up to date configuration information.  We are trying to
  insist that it become possible for screens to change dynamically, so
  we want to ensure the client knows what it is talking about when requesting
  changes.
}
function XRRSetScreenConfig(
  dpy : PDisplay;
  config : PXRRScreenConfiguration;
  draw : TDrawable;
  size_index : cint;
  rotation : TRotation;
  timestamp : TTime) : TStatus; cdecl; external libXrandr;

{ added in v1.1, sorry for the lame name }
function XRRSetScreenConfigAndRate(
  dpy : PDisplay;
  config : PXRRScreenConfiguration;
  draw : TDrawable;
  size_index : cint;
  rotation : TRotation;
  rate : cshort;
  timestamp : TTime) : TStatus; cdecl; external libXrandr;


function XRRConfigRotations(
  config : PXRRScreenConfiguration;
  current_rotation : PRotation) : TRotation; cdecl; external libXrandr;

function XRRConfigTimes(
  config : PXRRScreenConfiguration;
  config_timestamp : PTime) : TTime; cdecl; external libXrandr;

function XRRConfigSizes(
  config : PXRRScreenConfiguration;
  nsizes : Pcint) : PXRRScreenSize; cdecl; external libXrandr;

function XRRConfigRates(
  config : PXRRScreenConfiguration;
  sizeID : cint;
  nrates : Pcint) : Pcshort; cdecl; external libXrandr;

function XRRConfigCurrentConfiguration(
  config : PXRRScreenConfiguration;
  rotation : PRotation) : TSizeID; cdecl; external libXrandr;

function XRRConfigCurrentRate(
  config : PXRRScreenConfiguration) : cshort; cdecl; external libXrandr;

function XRRRootToScreen(
  dpy : PDisplay;
  root : TWindow) : cint; cdecl; external libXrandr;

{
  returns the screen configuration for the specified screen; does a lazy
  evalution to delay getting the information, and caches the result.
  These routines should be used in preference to XRRGetScreenInfo
  to avoid unneeded round trips to the X server.  These are new
  in protocol version 0.1.
}


function XRRScreenConfig(
  dpy : PDisplay;
  screen : cint) : PXRRScreenConfiguration; cdecl; external libXrandr;
function XRRConfig(
  screen : PScreen) : PXRRScreenConfiguration; cdecl; external libXrandr;
procedure XRRSelectInput(
  dpy : PDisplay;
  window : TWindow;
  mask : cint); cdecl; external libXrandr;

{
  the following are always safe to call, even if RandR is not implemented 
  on a screen 
}


function XRRRotations(
  dpy : PDisplay;
  screen : cint;
  current_rotation : PRotation) : TRotation; cdecl; external libXrandr;
function XRRSizes(
  dpy : PDisplay;
  screen : cint;
  nsizes : Pcint) : PXRRScreenSize; cdecl; external libXrandr;
function XRRRates(
  dpy : PDisplay;
  screen : cint;
  sizeID : cint;
  nrates : Pcint) : Pcshort; cdecl; external libXrandr;
function XRRTimes(
  dpy : PDisplay;
  screen : cint;
  config_timestamp : PTime) : TTime; cdecl; external libXrandr;


{
  intended to take RRScreenChangeNotify,  or 
  ConfigureNotify (on the root window)
  returns 1 if it is an event type it understands, 0 if not
}
function XRRUpdateConfiguration(
  event : PXEvent) : cint; cdecl; external libXrandr;

implementation

end.
