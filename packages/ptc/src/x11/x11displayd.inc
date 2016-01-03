<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the PTCPas framebuffer library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2013 Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew
    Original C++ version by Christian Nentwich (c.nentwich@cs.ucl.ac.uk)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version
    with the following modification:

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent modules,and
    to copy and distribute the resulting executable under terms of your choice,
    provided that you also meet, for each linked independent module, the terms
    and conditions of the license of that module. An independent module is a
    module which is not derived from or based on this library. If you modify
    this library, you may extend this exception to your version of the library,
    but you are not obligated to do so. If you do not wish to do so, delete this
    exception statement from your version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

type
  TX11FlagsEnum = (PTC_X11_FULLSCREEN,
                   PTC_X11_LEAVE_DISPLAY,
                   PTC_X11_LEAVE_WINDOW,
                   PTC_X11_TRY_DGA1,
                   PTC_X11_TRY_DGA2,
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TX11FlagsEnum = (PTC_X11_FULLSCREEN,
                   PTC_X11_LEAVE_DISPLAY,
                   PTC_X11_LEAVE_WINDOW,
                   PTC_X11_TRY_DGA,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
                   PTC_X11_TRY_XF86VIDMODE,
                   PTC_X11_TRY_XRANDR,
                   PTC_X11_TRY_XSHM,
                   PTC_X11_DITHER,
                   PTC_X11_FULLSCREEN_CURSOR_VISIBLE,
<<<<<<< HEAD
<<<<<<< HEAD
                   PTC_X11_WINDOWED_CURSOR_INVISIBLE,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   PTC_X11_GRAB_MOUSE,
                   PTC_X11_INTERCEPT_WINDOW_CLOSE,
                   PTC_X11_RESIZABLE_WINDOW,
                   PTC_X11_USE_OPENGL);
=======
                   PTC_X11_GRAB_MOUSE);
>>>>>>> graemeg/cpstrnew
  TX11Flags = set of TX11FlagsEnum;

type
  TX11Display = class
  protected
    { Conversion object }
    FCopy: TPTCCopy;
    FClear: TPTCClear;
<<<<<<< HEAD
    FPalette: IPTCPalette;

    FArea: IPTCArea;
    FClip: IPTCArea;

=======
    FPalette: TPTCPalette;

    FArea: TPTCArea;
    FClip: TPTCArea;

>>>>>>> graemeg/cpstrnew
    FEventQueue: TEventQueue;

    FFlags: TX11Flags;
    FWidth, FHeight: Integer;
<<<<<<< HEAD
    FFormat: IPTCFormat;
=======
    FFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew

    FDisplay: PDisplay;
    FScreen: Integer;

    FCMap: TColormap;
    FColours: PXColor;

    FFunctionKeys: PInteger;
    FNormalKeys: PInteger;

<<<<<<< HEAD
    function GetInterceptClose: Boolean;
    procedure SetInterceptClose(AInterceptClose: Boolean);

    function NextEvent(out AEvent: IPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; virtual; abstract;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): IPTCEvent; virtual; abstract;

    function GetX11DirectFormat(ABitsPerPixel: Integer;
                                AR, AG, AB: DWord;
                                AByteOrder: cint): IPTCFormat;
    function GetX11Format(AFormat: IPTCFormat): IPTCFormat;

    { initialise the keyboard mapping table }
    procedure SetKeyMapping;

    { Data access }
    function GetWidth: Integer;
    function GetHeight: Integer;
    function GetPitch: Integer; virtual; abstract;
    function GetFormat: IPTCFormat;
    function GetArea: IPTCArea;
    procedure HandleKeyEvent(const e: TXKeyEvent);
    property InterceptClose: Boolean read GetInterceptClose write SetInterceptClose;
  public
    constructor Create(ADisplay: PDisplay; AScreen: Integer; const AFlags: TX11Flags); virtual;
    destructor Destroy; override;

    procedure Open(ATitle: string; AWidth, AHeight: Integer; AFormat: IPTCFormat; const AOpenGLAttributes: IPTCOpenGLAttributes); virtual; abstract;

    { This will always return a windowed console. The first version
      fills the whole window, the second one has a custom size }
    procedure Open(AWindow: TWindow; AFormat: IPTCFormat); virtual; abstract;
    procedure Open(AWindow: TWindow; AFormat: IPTCFormat; AX, AY, AWidth, AHeight: Integer); virtual; abstract;

    procedure Close; virtual; abstract;

    procedure InternalResize(AWidth, AHeight: Integer); virtual;

    procedure Update; virtual; abstract;
    procedure Update(AArea: IPTCArea); virtual; abstract;

=======
=======
                   PTC_X11_GRAB_MOUSE);
  TX11Flags = set of TX11FlagsEnum;

type
  TX11Display = class
  protected
    { Conversion object }
    FCopy: TPTCCopy;
    FClear: TPTCClear;
    FPalette: TPTCPalette;

    FArea: TPTCArea;
    FClip: TPTCArea;

    FEventQueue: TEventQueue;

=======
                   PTC_X11_GRAB_MOUSE);
  TX11Flags = set of TX11FlagsEnum;

type
  TX11Display = class
  protected
    { Conversion object }
    FCopy: TPTCCopy;
    FClear: TPTCClear;
    FPalette: TPTCPalette;

    FArea: TPTCArea;
    FClip: TPTCArea;

    FEventQueue: TEventQueue;

>>>>>>> graemeg/cpstrnew
    FFlags: TX11Flags;
    FWidth, FHeight: Integer;
    FFormat: TPTCFormat;

    FDisplay: PDisplay;
    FScreen: Integer;

    FCMap: TColormap;
    FColours: PXColor;

    FFunctionKeys: PInteger;
    FNormalKeys: PInteger;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
    function NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; virtual; abstract;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent; virtual; abstract;

    function GetX11DirectFormat(ABitsPerPixel: Integer;
                                AR, AG, AB: DWord;
                                AByteOrder: cint): TPTCFormat;
    function GetX11Format(const AFormat: TPTCFormat): TPTCFormat;

    { initialise the keyboard mapping table }
    procedure SetKeyMapping;

    { Data access }
    function GetWidth: Integer;
    function GetHeight: Integer;
    function GetPitch: Integer; virtual; abstract;
    function GetFormat: TPTCFormat;
    function GetArea: TPTCArea;
  public
    constructor Create(ADisplay: PDisplay; AScreen: Integer; const AFlags: TX11Flags); virtual;
    destructor Destroy; override;

=======
                   PTC_X11_GRAB_MOUSE);
  TX11Flags = set of TX11FlagsEnum;

type
  TX11Display = class
  protected
    { Conversion object }
    FCopy: TPTCCopy;
    FClear: TPTCClear;
    FPalette: TPTCPalette;

    FArea: TPTCArea;
    FClip: TPTCArea;

    FEventQueue: TEventQueue;

    FFlags: TX11Flags;
    FWidth, FHeight: Integer;
    FFormat: TPTCFormat;

    FDisplay: PDisplay;
    FScreen: Integer;

    FCMap: TColormap;
    FColours: PXColor;

    FFunctionKeys: PInteger;
    FNormalKeys: PInteger;

    function NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; virtual; abstract;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent; virtual; abstract;

    function GetX11DirectFormat(ABitsPerPixel: Integer;
                                AR, AG, AB: DWord;
                                AByteOrder: cint): TPTCFormat;
    function GetX11Format(const AFormat: TPTCFormat): TPTCFormat;

    { initialise the keyboard mapping table }
    procedure SetKeyMapping;

    { Data access }
    function GetWidth: Integer;
    function GetHeight: Integer;
    function GetPitch: Integer; virtual; abstract;
    function GetFormat: TPTCFormat;
    function GetArea: TPTCArea;
  public
    constructor Create(ADisplay: PDisplay; AScreen: Integer; const AFlags: TX11Flags); virtual;
    destructor Destroy; override;

>>>>>>> origin/cpstrnew
    procedure Open(ATitle: string; AWidth, AHeight: Integer; const AFormat: TPTCFormat); virtual; abstract;

    { This will always return a windowed console. The first version
      fills the whole window, the second one has a custom size }
    procedure Open(AWindow: TWindow; const AFormat: TPTCFormat); virtual; abstract;
    procedure Open(AWindow: TWindow; const AFormat: TPTCFormat; AX, AY, AWidth, AHeight: Integer); virtual; abstract;

    procedure Close; virtual; abstract;

    procedure Update; virtual; abstract;
    procedure Update(const AArea: TPTCArea); virtual; abstract;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function Lock: Pointer; virtual; abstract;
    procedure Unlock; virtual; abstract;

    { load pixels to console }
    procedure Load(const APixels: Pointer; AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   AFormat: IPTCFormat; APalette: IPTCPalette); virtual;
    procedure Load(const APixels: Pointer; AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat; APalette: IPTCPalette;
                   ASource, ADestination: IPTCArea); virtual;

    { save console pixels }
    procedure Save(APixels: Pointer; AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat; APalette: IPTCPalette); virtual;
    procedure Save(APixels: Pointer; AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat; APalette: IPTCPalette;
                   ASource, ADestination: IPTCArea); virtual;

    { clear surface }
    procedure Clear(AColor: IPTCColor); virtual;
    procedure Clear(AColor: IPTCColor; AArea: IPTCArea); virtual;

    { Console palette }
    procedure Palette(APalette: IPTCPalette); virtual; abstract;
    function Palette: IPTCPalette; virtual;

    { console clip area }
    procedure Clip(AArea: IPTCArea);
=======
=======
>>>>>>> graemeg/cpstrnew
                   const AFormat: TPTCFormat; const APalette: TPTCPalette); virtual;
    procedure Load(const APixels: Pointer; AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat; const APalette: TPTCPalette;
                   const ASource, ADestination: TPTCArea); virtual;

    { save console pixels }
    procedure Save(APixels: Pointer; AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat; const APalette: TPTCPalette); virtual;
    procedure Save(APixels: Pointer; AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat; const APalette: TPTCPalette;
                   const ASource, ADestination: TPTCArea); virtual;

    { clear surface }
    procedure Clear(const AColor: TPTCColor); virtual;
    procedure Clear(const AColor: TPTCColor; const AArea: TPTCArea); virtual;

    { Console palette }
    procedure Palette(const APalette: TPTCPalette); virtual; abstract;
    function Palette: TPTCPalette; virtual;

    { console clip area }
    procedure Clip(const AArea: TPTCArea);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

    { cursor control }
    procedure SetCursor(AVisible: Boolean); virtual; abstract;

    { mouse grab control }
    procedure SetMouseGrab(AGrabMouse: Boolean); virtual; abstract;

    { Data access }
<<<<<<< HEAD
<<<<<<< HEAD
    function Clip: IPTCArea;

    function IsFullScreen: Boolean; virtual; abstract;
    function IsOpen: Boolean; virtual; abstract;
=======
    function Clip: TPTCArea;

    function IsFullScreen: Boolean; virtual; abstract;
>>>>>>> graemeg/cpstrnew
=======
    function Clip: TPTCArea;

    function IsFullScreen: Boolean; virtual; abstract;
>>>>>>> graemeg/cpstrnew

    { set flags (only used internally now!) }
    procedure SetFlags(AFlags: TX11Flags);

<<<<<<< HEAD
<<<<<<< HEAD
    procedure GetModes(var AModes: TPTCModeList); virtual; abstract;
=======
    procedure GetModes(var AModes: TPTCModeDynArray); virtual; abstract;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray); virtual; abstract;
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
                   const AFormat: TPTCFormat; const APalette: TPTCPalette); virtual;
    procedure Load(const APixels: Pointer; AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat; const APalette: TPTCPalette;
                   const ASource, ADestination: TPTCArea); virtual;

    { save console pixels }
    procedure Save(APixels: Pointer; AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat; const APalette: TPTCPalette); virtual;
    procedure Save(APixels: Pointer; AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat; const APalette: TPTCPalette;
                   const ASource, ADestination: TPTCArea); virtual;

    { clear surface }
    procedure Clear(const AColor: TPTCColor); virtual;
    procedure Clear(const AColor: TPTCColor; const AArea: TPTCArea); virtual;

    { Console palette }
    procedure Palette(const APalette: TPTCPalette); virtual; abstract;
    function Palette: TPTCPalette; virtual;

    { console clip area }
    procedure Clip(const AArea: TPTCArea);

    { cursor control }
    procedure SetCursor(AVisible: Boolean); virtual; abstract;

    { mouse grab control }
    procedure SetMouseGrab(AGrabMouse: Boolean); virtual; abstract;

    { Data access }
    function Clip: TPTCArea;

    function IsFullScreen: Boolean; virtual; abstract;

    { set flags (only used internally now!) }
    procedure SetFlags(AFlags: TX11Flags);

    procedure GetModes(var AModes: TPTCModeDynArray); virtual; abstract;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
                   PTC_X11_WINDOWED_CURSOR_INVISIBLE);
  TX11Flags = Set Of TX11FlagsEnum;

Type
  TX11Display = Class(TObject)
  Protected
    Function NextEvent(Var AEvent : TPTCEvent; AWait : Boolean; Const AEventMask : TPTCEventMask) : Boolean; Virtual; Abstract;
    Function PeekEvent(AWait : Boolean; Const AEventMask : TPTCEventMask) : TPTCEvent; Virtual; Abstract;

    Function GetX11Format(Const AFormat : TPTCFormat) : TPTCFormat;

    { initialise the keyboard mapping table }
    Procedure SetKeyMapping;

    { Data access }
    Function GetWidth : Integer;
    Function GetHeight : Integer;
    Function GetPitch : Integer; Virtual; Abstract;
    Function GetFormat : TPTCFormat;
    Function GetArea : TPTCArea;

    { Conversion object }
    FCopy : TPTCCopy;
    FClear : TPTCClear;
    FPalette : TPTCPalette;

    FArea : TPTCArea;
    FClip : TPTCArea;

    FEventQueue : TEventQueue;

    FFlags : TX11Flags;
    FWidth, FHeight : DWord;
    FFormat : TPTCFormat;

    FDisplay : PDisplay;
    FScreen : Integer;

    FCMap : TColormap;
    FColours : PXColor;

    FFunctionKeys : PInteger;
    FNormalKeys : PInteger;

    {m_thread : pthread_t;}
  Public
    Constructor Create(ADisplay : PDisplay; AScreen : Integer; Const AFlags : TX11Flags); Virtual;
    Destructor Destroy; Override;

    Procedure Open(ATitle : String; AWidth, AHeight : Integer; Const AFormat : TPTCFormat); Virtual; Abstract;

    { This will always return a windowed console. The first version
      fills the whole window, the second one has a custom size }
    Procedure Open(AWindow : TWindow; Const AFormat : TPTCFormat); Virtual; Abstract;
    Procedure Open(AWindow : TWindow; Const AFormat : TPTCFormat; AX, AY, AWidth, AHeight : Integer); Virtual; Abstract;

    Procedure Close; Virtual; Abstract;

    Procedure Update; Virtual; Abstract;
    Procedure Update(Const AArea : TPTCArea); Virtual; Abstract;

    Function Lock : Pointer; Virtual; Abstract;
    Procedure Unlock; Virtual; Abstract;

    { load pixels to console }
    Procedure Load(Const APixels : Pointer; AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat; Const APalette : TPTCPalette); Virtual;
    Procedure Load(Const APixels : Pointer; AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat; Const APalette : TPTCPalette;
                   Const ASource, ADestination : TPTCArea); Virtual;

    { save console pixels }
    Procedure Save(APixels : Pointer; AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat; Const APalette : TPTCPalette); Virtual;
    Procedure Save(APixels : Pointer; AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat; Const APalette : TPTCPalette;
                   Const ASource, ADestination : TPTCArea); Virtual;

    { clear surface }
    Procedure Clear(Const AColor : TPTCColor); Virtual;
    Procedure Clear(Const AColor : TPTCColor; Const AArea : TPTCArea); Virtual;

    { Console palette }
    Procedure Palette(Const APalette : TPTCPalette); Virtual; Abstract;
    Function Palette : TPTCPalette; Virtual;

    { console clip area }
    Procedure Clip(Const AArea : TPTCArea);

    { cursor control }
    Procedure SetCursor(AVisible : Boolean); Virtual; Abstract;

    { Data access }
    Function Clip : TPTCArea;

    Function IsFullScreen : Boolean; Virtual; Abstract;

    { Set flags (only used internally now!) }
    Procedure SetFlags(AFlags : TX11Flags);

    Procedure GetModes(Var AModes : TPTCModeDynArray); Virtual; Abstract;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    { X11 helper functions for your enjoyment }

    { return the display we are using }
<<<<<<< HEAD
<<<<<<< HEAD
    function GetX11Display: PDisplay;

    { return the screen we are using }
    function GetX11Screen: Integer;

    { return our window (0 if DGA) }
    function GetX11Window: TWindow; virtual; abstract;

    property Width: Integer read GetWidth;
    property Height: Integer read GetHeight;
    property Pitch: Integer read GetPitch;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    property Area: IPTCArea read GetArea;
    property Format: IPTCFormat read GetFormat;

    procedure OpenGL_SwapBuffers; virtual;
    procedure OpenGL_SetSwapInterval(AInterval: Integer); virtual;
    function OpenGL_GetSwapInterval: Integer; virtual;
=======
    property Area: TPTCArea read GetArea;
    property Format: TPTCFormat read GetFormat;
>>>>>>> graemeg/cpstrnew
=======
    property Area: TPTCArea read GetArea;
    property Format: TPTCFormat read GetFormat;
>>>>>>> graemeg/cpstrnew
=======
    property Area: TPTCArea read GetArea;
    property Format: TPTCFormat read GetFormat;
>>>>>>> graemeg/cpstrnew
=======
    property Area: TPTCArea read GetArea;
    property Format: TPTCFormat read GetFormat;
>>>>>>> origin/cpstrnew
  end;
=======
=======
>>>>>>> origin/fixes_2_2
    Function GetX11Display : PDisplay;

    { return the screen we are using }
    Function GetX11Screen : Integer;

    { return our window (0 if DGA) }
    Function GetX11Window : TWindow; Virtual; Abstract;

    Property Width : Integer Read GetWidth;
    Property Height : Integer Read GetHeight;
    Property Pitch : Integer Read GetPitch;
    Property Area : TPTCArea Read GetArea;
    Property Format : TPTCFormat Read GetFormat;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
