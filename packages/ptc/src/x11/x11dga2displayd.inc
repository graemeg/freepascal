<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the PTCPas framebuffer library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2012 Nikolay Nikolov (nickysn@users.sourceforge.net)
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

{$IFDEF ENABLE_X11_EXTENSION_XF86DGA2}

type
  TX11DGA2Display = class(TX11Display)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FOpen: Boolean;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FXDGAEventBase: cint;
    FXDGAErrorBase: cint;
  
    { The list of available modes (todo: move to local vars in the open function) }
    FXDGAModes: PXDGAMode;
    FXDGAModesNum: cint;

    { Holds the pointer to the framebuffer and all the other information for
      the current mode (or nil, if a mode isn't open) }
    FXDGADevice: PXDGADevice;

    { Coordinates of upper left frame corner }
    FDestX, FDestY: Integer;

    FModeIsSet: Boolean;
    FFramebufferIsOpen: Boolean;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure HandleEvents;

  protected
    function NextEvent(out event: IPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): IPTCEvent; override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function NextEvent(var event: TPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent; override;

    procedure HandleEvents;
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
  public
    constructor Create(ADisplay: PDisplay; AScreen: Integer; const AFlags: TX11Flags); override;
    destructor Destroy; override;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure Open(ATitle: string; AWidth, AHeight: Integer; AFormat: IPTCFormat; const AOpenGLAttributes: IPTCOpenGLAttributes); override;
    procedure Open(w: TWindow; _format: IPTCFormat); override;
    procedure Open(_window: TWindow; _format: IPTCFormat; x, y, w, h: Integer); override;
    procedure Close; override;
    procedure GetModes(var AModes: TPTCModeList); override;
    procedure Update; override;
    procedure Update(AArea: IPTCArea); override;
    function lock: Pointer; override;
    procedure unlock; override;
    procedure Palette(_palette: IPTCPalette); override;
    function GetPitch: Integer; override;
    function getX11Window: TWindow; override;
    function isFullScreen: Boolean; override;
    function IsOpen: Boolean; override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure open(ATitle: string; AWidth, AHeight: Integer; const AFormat: TPTCFormat); override;
    procedure open(w: TWindow; const _format: TPTCFormat); override;
    procedure open(_window: TWindow; const _format: TPTCFormat; x, y, w, h: Integer); override;
    procedure close; override;
    procedure GetModes(var AModes: TPTCModeDynArray); override;
    procedure update; override;
    procedure update(const _area: TPTCArea); override;
    function lock: Pointer; override;
    procedure unlock; override;
    procedure palette(const _palette: TPTCPalette); override;
    function GetPitch: Integer; override;
    function getX11Window: TWindow; override;
    function isFullScreen: Boolean; override;
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
    procedure SetCursor(visible: Boolean); override;
    procedure SetMouseGrab(AGrabMouse: Boolean); override;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
{$IFDEF ENABLE_X11_EXTENSION_XF86DGA2}

Type
  TX11DGA2Display = Class(TX11Display)
  Private
    Function NextEvent(Var event : TPTCEvent; wait : Boolean; Const EventMask : TPTCEventMask) : Boolean; Override;
    Function PeekEvent(wait : Boolean; Const EventMask : TPTCEventMask) : TPTCEvent; Override;

    Procedure HandleEvents;

    { The list of available modes (todo: move to local vars in the open function) }
    FXDGAModes : PXDGAMode;
    FXDGAModesNum : cint;

    { Holds the pointer to the framebuffer and all the other information for
      the current mode (or nil, if a mode isn't open) }
    FXDGADevice : PXDGADevice;

    { Coordinates of upper left frame corner }
    m_destx, m_desty : Integer;

    FModeIsSet : Boolean;
    FFramebufferIsOpen : Boolean;
  Public
    Constructor Create(ADisplay : PDisplay; AScreen : Integer; Const AFlags : TX11Flags); Override;
    Destructor Destroy; Override;

    Procedure open(title : String; _width, _height : Integer; Const _format : TPTCFormat); Override;
    Procedure open(w : TWindow; Const _format : TPTCFormat); Override;
    Procedure open(_window : TWindow; Const _format : TPTCFormat; x, y, w, h : Integer); Override;
    Procedure close; Override;
    Procedure GetModes(Var AModes : TPTCModeDynArray); Override;
    Procedure update; Override;
    Procedure update(Const _area : TPTCArea); Override;
    Function lock : Pointer; Override;
    Procedure unlock; Override;
    Procedure palette(Const _palette : TPTCPalette); Override;
    Function GetPitch : Integer; Override;
    Function getX11Window : TWindow; Override;
    Function isFullScreen : Boolean; Override;
    Procedure SetCursor(visible : Boolean); Override;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ENDIF ENABLE_X11_EXTENSION_XF86DGA2}
