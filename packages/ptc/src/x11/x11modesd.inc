<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the PTCPas framebuffer library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2011 Nikolay Nikolov (nickysn@users.sourceforge.net)
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TX11Modes = class(TObject)
=======
  TX11Modes = Class(TObject)
>>>>>>> graemeg/cpstrnew
=======
  TX11Modes = Class(TObject)
>>>>>>> graemeg/cpstrnew
=======
  TX11Modes = Class(TObject)
>>>>>>> graemeg/cpstrnew
=======
  TX11Modes = Class(TObject)
>>>>>>> origin/cpstrnew
  private
    FDisplay: PDisplay;
    FScreen: cint;
  protected
    function GetWidth: Integer; virtual; abstract;
    function GetHeight: Integer; virtual; abstract;
  public
    constructor Create(ADisplay: PDisplay; AScreen: cint); virtual;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure GetModes(var AModes: TPTCModeList; ACurrentDesktopFormat: IPTCFormat); virtual; abstract;
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); virtual; abstract;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); virtual; abstract;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); virtual; abstract;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); virtual; abstract;
>>>>>>> origin/cpstrnew
    procedure SetBestMode(AWidth, AHeight: Integer); virtual; abstract;
    procedure RestorePreviousMode; virtual; abstract;
    property Width: Integer read GetWidth;
    property Height: Integer read GetHeight;
  end;

  TX11ModesNoModeSwitching = Class(TX11Modes)
  private
    FWidth, FHeight: Integer;
  protected
    function GetWidth: Integer; override;
    function GetHeight: Integer; override;
  public
    constructor Create(ADisplay: PDisplay; AScreen: cint); override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure GetModes(var AModes: TPTCModeList; ACurrentDesktopFormat: IPTCFormat); override;
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> origin/cpstrnew
    procedure SetBestMode(AWidth, AHeight: Integer); override;
    procedure RestorePreviousMode; override;
  end;

{$IFDEF ENABLE_X11_EXTENSION_XRANDR}
  TX11ModesXrandr = Class(TX11Modes)
  private
    FRoot: TWindow;
    FXRRConfig: PXRRScreenConfiguration;
    FWidth, FHeight: Integer;

    FInMode: Boolean;
    FSavedMode: record
      ScreenSize: TXRRScreenSize;
      Rotation: TRotation;
      Rate: cshort;
    end;

    procedure CreateScreenConfig;
    procedure DestroyScreenConfig;

    class function FindBestMode(AScreenSizes: PXRRScreenSize;
                                AScreenSizesNum: Integer;
                                ARequestedWidth,
                                ARequestedHeight: Integer): Integer;
    procedure SaveCurrentMode;
  protected
    function GetWidth: Integer; override;
    function GetHeight: Integer; override;
  public
    constructor Create(ADisplay: PDisplay; AScreen: cint); override;
    destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure GetModes(var AModes: TPTCModeList; ACurrentDesktopFormat: IPTCFormat); override;
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> origin/cpstrnew
    procedure SetBestMode(AWidth, AHeight: Integer); override;
    procedure RestorePreviousMode; override;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TX11Modes = Class(TObject)
  Private
    FDisplay : PDisplay;
    FScreen : cint;
  Protected
    Function GetWidth : Integer; Virtual; Abstract;
    Function GetHeight : Integer; Virtual; Abstract;
  Public
    Constructor Create(ADisplay : PDisplay; AScreen : cint); Virtual;
    Procedure GetModes(Var AModes : TPTCModeDynArray; ACurrentDesktopFormat : TPTCFormat); Virtual; Abstract;
    Procedure SetBestMode(AWidth, AHeight : Integer); Virtual; Abstract;
    Procedure RestorePreviousMode; Virtual; Abstract;
    Property Width : Integer Read GetWidth;
    Property Height : Integer Read GetHeight;
  End;

  TX11ModesNoModeSwitching = Class(TX11Modes)
  Private
    FWidth, FHeight : Integer;
  Protected
    Function GetWidth : Integer; Override;
    Function GetHeight : Integer; Override;
  Public
    Constructor Create(ADisplay : PDisplay; AScreen : cint); Override;
    Procedure GetModes(Var AModes : TPTCModeDynArray; ACurrentDesktopFormat : TPTCFormat); Override;
    Procedure SetBestMode(AWidth, AHeight : Integer); Override;
    Procedure RestorePreviousMode; Override;
  End;

{$IFDEF ENABLE_X11_EXTENSION_XRANDR}
  TX11ModesXrandr = Class(TX11Modes)
  Private
    FRoot : TWindow;
    FXRRConfig : PXRRScreenConfiguration;
  Protected
    Function GetWidth : Integer; Override;
    Function GetHeight : Integer; Override;
  Public
    Constructor Create(ADisplay : PDisplay; AScreen : cint); Override;
    Destructor Destroy; Override;
    Procedure GetModes(Var AModes : TPTCModeDynArray; ACurrentDesktopFormat : TPTCFormat); Override;
    Procedure SetBestMode(AWidth, AHeight : Integer); Override;
    Procedure RestorePreviousMode; Override;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF ENABLE_X11_EXTENSION_XRANDR}

{$IFDEF ENABLE_X11_EXTENSION_XF86VIDMODE}
  TX11ModesXF86VidMode = Class(TX11Modes)
<<<<<<< HEAD
<<<<<<< HEAD
  private
    FModeList: PPXF86VidModeModeInfo;
    FModeListCount: cint;
    FSavedMode: PXF86VidModeModeLine;
    FSavedDotClock: cint;
    FWidth, FHeight: Integer;

    procedure RetrieveModeList;
    function FindNumberOfBestMode(AWidth, AHeight: Integer): Integer;
  protected
    function GetWidth: Integer; override;
    function GetHeight: Integer; override;
  public
    constructor Create(ADisplay: PDisplay; AScreen: cint); override;
    destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure GetModes(var AModes: TPTCModeList; ACurrentDesktopFormat: IPTCFormat); override;
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat); override;
>>>>>>> origin/cpstrnew
    procedure SetBestMode(AWidth, AHeight: Integer); override;
    procedure RestorePreviousMode; override;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
  Private
    FModeList : PPXF86VidModeModeInfo;
    FModeListCount : cint;
    FSavedMode : PXF86VidModeModeLine;
    FSavedDotClock : cint;
    FWidth, FHeight : Integer;

    Procedure RetrieveModeList;
    Function FindNumberOfBestMode(AWidth, AHeight : Integer) : Integer;
  Protected
    Function GetWidth : Integer; Override;
    Function GetHeight : Integer; Override;
  Public
    Constructor Create(ADisplay : PDisplay; AScreen : cint); Override;
    Destructor Destroy; Override;
    Procedure GetModes(Var AModes : TPTCModeDynArray; ACurrentDesktopFormat : TPTCFormat); Override;
    Procedure SetBestMode(AWidth, AHeight : Integer); Override;
    Procedure RestorePreviousMode; Override;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF ENABLE_X11_EXTENSION_XF86VIDMODE}
