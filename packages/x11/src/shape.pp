(************************************************************

Copyright 1989, 1998  The Open Group

Permission to use, copy, modify, distribute, and sell this software and its
documentation for any purpose is hereby granted without fee, provided that
the above copyright notice appear in all copies and that both that
copyright notice and this permission notice appear in supporting
documentation.

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
OPEN GROUP BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Except as contained in this notice, the name of The Open Group shall not be
used in advertising or otherwise to promote the sale, use or other dealings
in this Software without prior written authorization from The Open Group.

********************************************************)

unit shape;

{$PACKRECORDS C}

interface

uses
  ctypes, x, xlib, xutil;

const
  libXext = 'Xext';

{$I shapeconst.inc}

//#include <X11/Xutil.h>

type
  PXShapeEvent = ^TXShapeEvent;
  TXShapeEvent = record
    _type: cint;               { of event }
    serial: culong;            { # of last request processed by server }
    send_event: TBool;         { true if this came frome a SendEvent request }
    display: PDisplay;         { Display the event was read from }
    window: TWindow;           { window of event }
    kind: cint;                { ShapeBounding or ShapeClip }
    x, y: cint;                { extents of new region }
    width, height: cunsigned;
    time: TTime;               { server timestamp when region changed }
    shaped: TBool;             { true if the region exists }
  end;

function XShapeQueryExtension(
    display: PDisplay;
    event_base,
    error_base: Pcint
): TBoolResult; cdecl; external libXext;

function XShapeQueryVersion(
    display: PDisplay;
    major_version,
    minor_version: Pcint
): TStatus; cdecl; external libXext;

procedure XShapeCombineRegion(
    display: PDisplay;
    dest: TWindow;
    dest_kind: cint;
    x_off,
    y_off: cint;
    region: TRegion;
    op: cint
); cdecl; external libXext;

procedure XShapeCombineRectangles(
    display: PDisplay;
    dest: TWindow;
    dest_kind: cint;
    x_off,
    y_off: cint;
    rectangles: PXRectangle;
    n_rects: cint;
    op: cint;
    ordering: cint
); cdecl; external libXext;

procedure XShapeCombineMask(
    display: PDisplay;
    dest: TWindow;
    dest_kind: cint;
    x_off,
    y_off: cint;
    src: TPixmap;
    op: cint
); cdecl; external libXext;

procedure XShapeCombineShape(
    display: PDisplay;
    dest: TWindow;
    dest_kind: cint;
    x_off,
    y_off: cint;
    src: TWindow;
    src_kind: cint;
    op: cint
); cdecl; external libXext;

procedure XShapeOffsetShape(
    display: PDisplay;
    dest: TWindow;
    dest_kind: cint;
    x_off,
    y_off: cint
); cdecl; external libXext;

function XShapeQueryExtents(
    display: PDisplay;
    window: TWindow;
    bounding_shaped: PBool;
    x_bounding,
    y_bounding: Pcint;
    w_bounding,
    h_bounding: Pcuint;
    clip_shaped: PBool;
    x_clip,
    y_clip: Pcint;
    w_clip,
    h_clip: Pcuint
): TStatus; cdecl; external libXext;

procedure XShapeSelectInput(
    display: PDisplay;
    window: TWindow;
    mask: culong
); cdecl; external libXext;

function XShapeInputSelected(
    display: PDisplay;
    window: TWindow
): culong; cdecl; external libXext;

function XShapeGetRectangles(
    display: PDisplay;
    window: TWindow;
    kind: cint;
    count,
    ordering: Pcint
): PXRectangle; cdecl; external libXext;

implementation

end.
