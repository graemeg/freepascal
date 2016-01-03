{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit inputevent;

INTERFACE

uses exec, utility, timer;

const

{------ constants -------------------------------------------------}

{   --- InputEvent.ie_Class --- }
{ A NOP input event }
    IECLASS_NULL        = $00;
{ A raw keycode from the keyboard device }
    IECLASS_RAWKEY      = $01;
{ The raw mouse report from the game port device }
    IECLASS_RAWMOUSE    = $02;
{ A private console event }
    IECLASS_EVENT       = $03;
{ A Pointer Position report }
    IECLASS_POINTERPOS  = $04;
{ A timer event }
    IECLASS_TIMER       = $06;
{ select button pressed down over a Gadget (address in ie_EventAddress) }
    IECLASS_GADGETDOWN  = $07;
{ select button released over the same Gadget (address in ie_EventAddress) }
    IECLASS_GADGETUP    = $08;
{ some Requester activity has taken place.  See Codes REQCLEAR and REQSET }
    IECLASS_REQUESTER   = $09;
{ this is a Menu Number transmission (Menu number is in ie_Code) }
    IECLASS_MENULIST    = $0A;
{ User has selected the active Window's Close Gadget }
    IECLASS_CLOSEWINDOW = $0B;
{ this Window has a new size }
    IECLASS_SIZEWINDOW  = $0C;
{ the Window pointed to by ie_EventAddress needs to be refreshed }
    IECLASS_REFRESHWINDOW = $0D;
{ new preferences are available }
    IECLASS_NEWPREFS    = $0E;
{ the disk has been removed }
    IECLASS_DISKREMOVED = $0F;
{ the disk has been inserted }
    IECLASS_DISKINSERTED = $10;
{ the window is about to be been made active }
    IECLASS_ACTIVEWINDOW = $11;
{ the window is about to be made inactive }
    IECLASS_INACTIVEWINDOW = $12;
{ extended-function pointer position report (V36) }
    IECLASS_NEWPOINTERPOS  = $13;
{ Help key report during Menu session (V36) }
    IECLASS_MENUHELP       = $14;
{ the Window has been modified with move, size, zoom, or change (V36) }
    IECLASS_CHANGEWINDOW   = $15;



{ the last class }

    IECLASS_MAX         = $15;

{  --- InputEvent.ie_SubClass --- }
{  IECLASS_NEWPOINTERPOS }
{      like IECLASS_POINTERPOS }
 IESUBCLASS_COMPATIBLE  = $00;
{      ie_EventAddress points to struct IEPointerPixel }
 IESUBCLASS_PIXEL       = $01;
{      ie_EventAddress points to struct IEPointerTablet }
 IESUBCLASS_TABLET      = $02;

{ pointed to by ie_EventAddress for IECLASS_NEWPOINTERPOS,
 * and IESUBCLASS_PIXEL.
 *
 * You specify a screen and pixel coordinates in that screen
 * at which you'd like the mouse to be positioned.
 * Intuition will try to oblige, but there will be restrictions
 * to positioning the pointer over offscreen pixels.
 *
 * IEQUALIFIER_RELATIVEMOUSE is supported for IESUBCLASS_PIXEL.
 }
Type

   pIEPointerPixel = ^tIEPointerPixel;
   tIEPointerPixel = record
    iepp_Screen : Pointer;   { pointer to an open screen }
    iepp_Position : record
       x,y : smallint;
    end;
   END;

{ pointed to by ie_EventAddress for IECLASS_NEWPOINTERPOS,
 * and IESUBCLASS_TABLET.
 *
 * You specify a range of values and a value within the range
 * independently for each of X and Y (the minimum value of
 * the ranges is always normalized to 0).
 *
 * Intuition will position the mouse proportionally within its
 * natural mouse position rectangle limits.
 *
 * IEQUALIFIER_RELATIVEMOUSE is not supported for IESUBCLASS_TABLET.
 }

   pIEPointerTablet = ^tIEPointerTablet;
   tIEPointerTablet = record
    iept_Range : record       { 0 is min, these are max      }
       x,y : Word;
    end;
    iept_Value : record       { between 0 AND iept_Range     }
       x,y : Word;
    end;
    iept_Pressure : Word;  { -128 to 127 (unused, set to 0)  }
   END;

{ The ie_EventAddress of an IECLASS_NEWPOINTERPOS event of subclass
 * IESUBCLASS_NEWTABLET points at an IENewTablet structure.
 *
 *
 * IEQUALIFIER_RELATIVEMOUSE is not supported for IESUBCLASS_NEWTABLET.
 }

 pIENewTablet = ^tIENewTablet;
 tIENewTablet = record
    { Pointer to a hook you wish to be called back through, in
     * order to handle scaling.  You will be provided with the
     * width and height you are expected to scale your tablet
     * to, perhaps based on some user preferences.
     * If NULL, the tablet's specified range will be mapped directly
     * to that width and height for you, and you will not be
     * called back.
     }
    ient_CallBack : pHook;

    { Post-scaling coordinates and fractional coordinates.
     * DO NOT FILL THESE IN AT THE TIME THE EVENT IS WRITTEN!
     * Your driver will be called back and provided information
     * about the width and height of the area to scale the
     * tablet into.  It should scale the tablet coordinates
     * (perhaps based on some preferences controlling aspect
     * ratio, etc.) and place the scaled result into these
     * fields.  The ient_ScaledX and ient_ScaledY fields are
     * in screen-pixel resolution, but the origin ( [0,0]-point )
     * is not defined.  The ient_ScaledXFraction and
     * ient_ScaledYFraction fields represent sub-pixel position
     * information, and should be scaled to fill a UWORD fraction.
     }
    ient_ScaledX, ient_ScaledY,
    ient_ScaledXFraction, ient_ScaledYFraction : WORD;

    { Current tablet coordinates along each axis: }
    ient_TabletX, ient_TabletY : ULONG;

    { Tablet range along each axis.  For example, if ient_TabletX
     * can take values 0-999, ient_RangeX should be 1000.
     }
    ient_RangeX, ient_RangeY : ULONG;

    { Pointer to tag-list of additional tablet attributes.
     * See <intuition/intuition.h> for the tag values.
     }
    ient_TagList : pTagItem;
 end;


CONST
{   --- InputEvent.ie_Code ---   }
{ IECLASS_RAWKEY }
    IECODE_UP_PREFIX            = $80;
    IECODE_KEY_CODE_FIRST       = $00;
    IECODE_KEY_CODE_LAST        = $77;
    IECODE_COMM_CODE_FIRST      = $78;
    IECODE_COMM_CODE_LAST       = $7F;

{ IECLASS_ANSI }
    IECODE_C0_FIRST             = $00;
    IECODE_C0_LAST              = $1F;
    IECODE_ASCII_FIRST          = $20;
    IECODE_ASCII_LAST           = $7E;
    IECODE_ASCII_DEL            = $7F;
    IECODE_C1_FIRST             = $80;
    IECODE_C1_LAST              = $9F;
    IECODE_LATIN1_FIRST         = $A0;
    IECODE_LATIN1_LAST          = $FF;

{ IECLASS_RAWMOUSE }
    IECODE_LBUTTON              = $68;  { also uses IECODE_UP_PREFIX }
    IECODE_RBUTTON              = $69;
    IECODE_MBUTTON              = $6A;
    IECODE_NOBUTTON             = $FF;

{ IECLASS_EVENT }
    IECODE_NEWACTIVE            = $01;  { active input window changed }
    IECODE_NEWSIZE              = $02;  { resize of window }
    IECODE_REFRESH              = $03;  { refresh of window }

{ IECLASS_REQUESTER Codes }
{ REQSET is broadcast when the first Requester (not subsequent ones) opens
 * in the Window
 }
    IECODE_REQSET               = $01;
{ REQCLEAR is broadcast when the last Requester clears out of the Window }
    IECODE_REQCLEAR             = $00;


{   --- InputEvent.ie_Qualifier --- }
    IEQUALIFIER_LSHIFT          = $0001;
    IEQUALIFIER_RSHIFT          = $0002;
    IEQUALIFIER_CAPSLOCK        = $0004;
    IEQUALIFIER_CONTROL         = $0008;
    IEQUALIFIER_LALT            = $0010;
    IEQUALIFIER_RALT            = $0020;
    IEQUALIFIER_LCOMMAND        = $0040;
    IEQUALIFIER_RCOMMAND        = $0080;
    IEQUALIFIER_NUMERICPAD      = $0100;
    IEQUALIFIER_REPEAT          = $0200;
    IEQUALIFIER_INTERRUPT       = $0400;
    IEQUALIFIER_MULTIBROADCAST  = $0800;
    IEQUALIFIER_MIDBUTTON       = $1000;
    IEQUALIFIER_RBUTTON         = $2000;
    IEQUALIFIER_LEFTBUTTON      = $4000;
    IEQUALIFIER_RELATIVEMOUSE   = $8000;

    IEQUALIFIERB_LSHIFT         = 0;
    IEQUALIFIERB_RSHIFT         = 1;
    IEQUALIFIERB_CAPSLOCK       = 2;
    IEQUALIFIERB_CONTROL        = 3;
    IEQUALIFIERB_LALT           = 4;
    IEQUALIFIERB_RALT           = 5;
    IEQUALIFIERB_LCOMMAND       = 6;
    IEQUALIFIERB_RCOMMAND       = 7;
    IEQUALIFIERB_NUMERICPAD     = 8;
    IEQUALIFIERB_REPEAT         = 9;
    IEQUALIFIERB_INTERRUPT      = 10;
    IEQUALIFIERB_MULTIBROADCAST = 11;
    IEQUALIFIERB_MIDBUTTON      = 12;
    IEQUALIFIERB_RBUTTON        = 13;
    IEQUALIFIERB_LEFTBUTTON     = 14;
    IEQUALIFIERB_RELATIVEMOUSE  = 15;


{------ InputEvent ------------------------------------------------}

    type
       pInputEvent = ^tInputEvent;
       tInputEvent = record
            ie_NextEvent : pInputEvent;
            ie_Class : BYTE;
            ie_SubClass : BYTE;
            ie_Code : WORD;
            ie_Qualifier : WORD;
            ie_position : record
                case longint of
                   0 : ( ie_xy : record
                        ie_x : smallint;
                        ie_y : smallint;
                     end );
                   1 : ( ie_addr : APTR );
                   2 : ( ie_dead : record
                        ie_prev1DownCode : BYTE;
                        ie_prev1DownQual : BYTE;
                        ie_prev2DownCode : BYTE;
                        ie_prev2DownQual : BYTE;
                     end );
                end;
            ie_TimeStamp : tTimeVal;
         end;

IMPLEMENTATION

end.
