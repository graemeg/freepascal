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
  TX11WindowDisplay = class(TX11Display)
  private
    {eventHandler}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FOpen: Boolean;
    FWindow: TWindow;
    FVisual: PVisual;
=======
    FWindow: TWindow;
>>>>>>> graemeg/cpstrnew
=======
    FWindow: TWindow;
>>>>>>> graemeg/cpstrnew
=======
    FWindow: TWindow;
>>>>>>> graemeg/cpstrnew
=======
    FWindow: TWindow;
>>>>>>> origin/cpstrnew
    FPrimary: TX11Image;
    FDestX, FDestY: Integer;
    FGC: TGC;
    FAtomClose: TAtom; { X Atom for close window button }
    FCursorVisible: Boolean;
    FGrabMouse: Boolean;
    FMouseGrabbed: Boolean;
    FX11InvisibleCursor: TCursor; { Blank cursor }
    FFullScreen: Boolean; { Keeps a snapshot of the PTC_X11_FULLSCREEN option
                             taken at the time 'open' was called }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FResizable: Boolean;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FFocus: Boolean;
    FModeSwitcher: TX11Modes;

    FPreviousMouseButtonState: TPTCMouseButtonState;
    FPreviousMouseX, FPreviousMouseY: Integer; { for calculating the deltas }
    FPreviousMousePositionSaved: Boolean; { true, if FPreviousMouseX,
           FPreviousMouseY and FPreviousMouseButtonState contain valid values }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FPreviousWidth, FPreviousHeight: Integer;

{$IFDEF ENABLE_X11_EXTENSION_GLX}
    FGLXFBConfig: TX11GLXFBConfig;
{$ENDIF ENABLE_X11_EXTENSION_GLX}
=======
    function NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent; override;
>>>>>>> graemeg/cpstrnew
=======
    function NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent; override;
>>>>>>> graemeg/cpstrnew
=======
    function NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent; override;
>>>>>>> graemeg/cpstrnew
=======
    function NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent; override;
>>>>>>> origin/cpstrnew

    procedure EnterFullScreen;
    procedure LeaveFullScreen;
    procedure internal_ShowCursor(AVisible: Boolean);
    procedure internal_GrabMouse(AGrabMouse: Boolean);
    procedure HandleChangeFocus(ANewFocus: Boolean);
    procedure HandleEvents;
    procedure Draw;
    function CreateImage(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                         AFormat: IPTCFormat): TX11Image; { Factory method }
    function CreateModeSwitcher: TX11Modes; { Factory method }
    procedure CreateColormap; { Register colour maps }

  protected
    function NextEvent(out AEvent: IPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): IPTCEvent; override;
=======
                         AFormat: TPTCFormat): TX11Image; { Factory method }
    function CreateModeSwitcher: TX11Modes; { Factory method }
    procedure CreateColormap; { Register colour maps }
>>>>>>> graemeg/cpstrnew
=======
                         AFormat: TPTCFormat): TX11Image; { Factory method }
    function CreateModeSwitcher: TX11Modes; { Factory method }
    procedure CreateColormap; { Register colour maps }
>>>>>>> graemeg/cpstrnew
=======
                         AFormat: TPTCFormat): TX11Image; { Factory method }
    function CreateModeSwitcher: TX11Modes; { Factory method }
    procedure CreateColormap; { Register colour maps }
>>>>>>> graemeg/cpstrnew
=======
                         AFormat: TPTCFormat): TX11Image; { Factory method }
    function CreateModeSwitcher: TX11Modes; { Factory method }
    procedure CreateColormap; { Register colour maps }
>>>>>>> origin/cpstrnew
  public
    constructor Create(ADisplay: PDisplay; AScreen: Integer; const AFlags: TX11Flags); override;
    destructor Destroy; override;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure Open(ATitle: string; AWidth, AHeight: Integer; AFormat: IPTCFormat; const AOpenGLAttributes: IPTCOpenGLAttributes); override;
    procedure Open(AWindow: TWindow; AFormat: IPTCFormat); override;
    procedure Open(AWindow: TWindow; AFormat: IPTCFormat; AX, AY, AWidth, AHeight: Integer); override;
    procedure Close; override;
    procedure InternalResize(AWidth, AHeight: Integer); override;
    procedure Update; override;
    procedure Update(AArea: IPTCArea); override;
    function Lock: Pointer; override;
    procedure Unlock; override;
    procedure GetModes(var AModes: TPTCModeList); override;
    procedure Palette(APalette: IPTCPalette); override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure Open(ATitle: string; AWidth, AHeight: Integer; const AFormat: TPTCFormat); override;
    procedure Open(AWindow: TWindow; const AFormat: TPTCFormat); override;
    procedure Open(AWindow: TWindow; const AFormat: TPTCFormat; AX, AY, AWidth, AHeight: Integer); override;
    procedure Close; override;
    procedure Update; override;
    procedure Update(const AArea: TPTCArea); override;
    function Lock: Pointer; override;
    procedure Unlock; override;
    procedure GetModes(var AModes: TPTCModeDynArray); override;
    procedure Palette(const APalette: TPTCPalette); override;
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
    function GetPitch: Integer; override;
    function GetX11Window: TWindow; override;
    function GetX11GC: TGC; virtual;
    function IsFullScreen: Boolean; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function IsOpen: Boolean; override;
    procedure SetCursor(AVisible: Boolean); override;
    procedure SetMouseGrab(AGrabMouse: Boolean); override;
{$IFDEF ENABLE_X11_EXTENSION_GLX}
    procedure OpenGL_SwapBuffers; override;
    procedure OpenGL_SetSwapInterval(AInterval: Integer); override;
    function OpenGL_GetSwapInterval: Integer; override;
{$ENDIF ENABLE_X11_EXTENSION_GLX}
=======
    procedure SetCursor(AVisible: Boolean); override;
    procedure SetMouseGrab(AGrabMouse: Boolean); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure SetCursor(AVisible: Boolean); override;
    procedure SetMouseGrab(AGrabMouse: Boolean); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure SetCursor(AVisible: Boolean); override;
    procedure SetMouseGrab(AGrabMouse: Boolean); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure SetCursor(AVisible: Boolean); override;
    procedure SetMouseGrab(AGrabMouse: Boolean); override;
>>>>>>> origin/cpstrnew
  end;
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TX11WindowDisplay = Class(TX11Display)
  Private
    Function NextEvent(Var AEvent : TPTCEvent; AWait : Boolean; Const AEventMask : TPTCEventMask) : Boolean; Override;
    Function PeekEvent(AWait : Boolean; Const AEventMask : TPTCEventMask) : TPTCEvent; Override;

    Procedure EnterFullScreen;
    Procedure LeaveFullScreen;
    Procedure internal_ShowCursor(AVisible : Boolean);
    Procedure HandleChangeFocus(ANewFocus : Boolean);
    Procedure HandleEvents;
    Function CreateImage(ADisplay : PDisplay; AScreen, AWidth, AHeight : Integer;
                         AFormat : TPTCFormat) : TX11Image; { Factory method }
    Function CreateModeSwitcher : TX11Modes; { Factory method }
    Procedure CreateColormap; { Register colour maps }
    {eventHandler}
    FWindow : TWindow;
    FPrimary : TX11Image;
    FDestX, FDestY : Integer;
    FGC : TGC;
    FAtomClose : TAtom; { X Atom for close window button }
    FCursorVisible : Boolean;
    FX11InvisibleCursor : TCursor; { Blank cursor }
    FFullScreen : Boolean; { Keeps a snapshot of the PTC_X11_FULLSCREEN option
                             taken at the time 'open' was called }
    FFocus : Boolean;
    FModeSwitcher : TX11Modes;

    FPreviousMouseButtonState : TPTCMouseButtonState;
    FPreviousMouseX, FPreviousMouseY : Integer; { for calculating the deltas }
    FPreviousMousePositionSaved : Boolean; { true, if FPreviousMouseX,
           FPreviousMouseY and FPreviousMouseButtonState contain valid values }
  Public
    Constructor Create(ADisplay : PDisplay; AScreen : Integer; Const AFlags : TX11Flags); Override;
    Destructor Destroy; Override;

    Procedure Open(ATitle : String; AWidth, AHeight : Integer; Const AFormat : TPTCFormat); Override;
    Procedure Open(AWindow : TWindow; Const AFormat : TPTCFormat); Override;
    Procedure Open(AWindow : TWindow; Const AFormat : TPTCFormat; AX, AY, AWidth, AHeight : Integer); Override;
    Procedure Close; Override;
    Procedure Update; Override;
    Procedure Update(Const AArea : TPTCArea); Override;
    Function Lock : Pointer; Override;
    Procedure Unlock; Override;
    Procedure GetModes(Var AModes : TPTCModeDynArray); Override;
    Procedure Palette(Const APalette : TPTCPalette); Override;
    Function GetPitch : Integer; Override;
    Function GetX11Window : TWindow; Override;
    Function GetX11GC : TGC; Virtual;
    Function IsFullScreen : Boolean; Override;
    Procedure SetCursor(AVisible : Boolean); Override;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
