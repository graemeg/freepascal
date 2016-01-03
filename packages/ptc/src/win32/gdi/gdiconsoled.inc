{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
    version 2.1 of the License, or (at your option) any later version.
>>>>>>> graemeg/fixes_2_2
=======
    version 2.1 of the License, or (at your option) any later version.
>>>>>>> origin/fixes_2_2

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

<<<<<<< HEAD
<<<<<<< HEAD
type
  TGDIConsole = class(TPTCBaseConsole)
  private
    FWindow: TWin32Window;
    FWin32DIB: TWin32DIB;
    FKeyboard: TWin32Keyboard;
    FMouse: TWin32Mouse;

    FCopy: TPTCCopy;
    FClear: TPTCClear;
    FEventQueue: TEventQueue;
    FArea: TPTCArea;
    FClip: TPTCArea;
    FPalette: TPTCPalette;
    FModes: array [0..1] of TPTCMode;

    FOpen: Boolean;
    FLocked: Boolean;
    FCursor: Boolean;
    FCursorMode: TWin32CursorMode;

    FTitle: string;

    FDisplayWidth: Integer;
    FDisplayHeight: Integer;

    FDefaultWidth: Integer;
    FDefaultHeight: Integer;
    FDefaultFormat: TPTCFormat;

    procedure UpdateCursor;

    function GetWidth: Integer; override;
    function GetHeight: Integer; override;
    function GetPitch: Integer; override;
    function GetArea: TPTCArea; override;
    function GetFormat: TPTCFormat; override;
    function GetPages: Integer; override;
    function GetName: string; override;
    function GetTitle: string; override;
    function GetInformation: string; override;

    procedure CheckOpen(const AMessage: string);
    procedure CheckUnlocked(const AMessage: string);
  public
    constructor Create; override;
    destructor Destroy; override;

    procedure Open(const ATitle: string; APages: Integer = 0); overload; override;
    procedure Open(const ATitle: string; const AFormat: TPTCFormat;
                   APages: Integer = 0); overload; override;
    procedure Open(const ATitle: string; AWidth, AHeight: Integer;
                   const AFormat: TPTCFormat; APages: Integer = 0); overload; override;
    procedure Open(const ATitle: string; const AMode: TPTCMode;
                   APages: Integer = 0); overload; override;
    procedure Close; override;

    procedure Copy(ASurface: TPTCBaseSurface); override;
    procedure Copy(ASurface: TPTCBaseSurface;
                   const ASource, ADestination: TPTCArea); override;

    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat;
                   const APalette: TPTCPalette); override;
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat;
                   const APalette: TPTCPalette;
                   const ASource, ADestination: TPTCArea); override;
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat;
                   const APalette: TPTCPalette); override;
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat;
                   const APalette: TPTCPalette;
                   const ASource, ADestination: TPTCArea); override;

    function Lock: Pointer; override;
    procedure Unlock; override;

    procedure Clear; override;
    procedure Clear(const AColor: TPTCColor); override;
    procedure Clear(const AColor: TPTCColor;
                    const AArea: TPTCArea); override;

    procedure Configure(const AFileName: String); override;
    function Option(const AOption: String): Boolean; override;

    procedure Palette(const APalette: TPTCPalette); override;
    procedure Clip(const AArea: TPTCArea); override;
    function Clip: TPTCArea; override;
    function Palette: TPTCPalette; override;
    function Modes: PPTCMode; override;

    procedure Flush; override;
    procedure Finish; override;
    procedure Update; override;
    procedure Update(const AArea: TPTCArea); override;

    function NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent; override;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TGDIConsole = Class(TPTCBaseConsole)
  Private
    FWindow : TWin32Window;
    FWin32DIB : TWin32DIB;
    FKeyboard : TWin32Keyboard;
    FMouse : TWin32Mouse;

    FCopy : TPTCCopy;
    FClear : TPTCClear;
    FEventQueue : TEventQueue;
    FArea : TPTCArea;
    FClip : TPTCArea;
    FPalette : TPTCPalette;

    FOpen : Boolean;
    FLocked : Boolean;

    FTitle : String;

    FDefaultWidth : Integer;
    FDefaultHeight : Integer;
    FDefaultFormat : TPTCFormat;

    Function GetWidth : Integer; Override;
    Function GetHeight : Integer; Override;
    Function GetPitch : Integer; Override;
    Function GetArea : TPTCArea; Override;
    Function GetFormat : TPTCFormat; Override;
    Function GetPages : Integer; Override;
    Function GetName : String; Override;
    Function GetTitle : String; Override;
    Function GetInformation : String; Override;

    Procedure CheckOpen(    AMessage : String);
    Procedure CheckUnlocked(AMessage : String);
  Public
    Constructor Create; Override;
    Destructor Destroy; Override;

    Procedure Open(Const ATitle : String; APages : Integer = 0); Overload; Override;
    Procedure Open(Const ATitle : String; Const AFormat : TPTCFormat;
                   APages : Integer = 0); Overload; Override;
    Procedure Open(Const ATitle : String; AWidth, AHeight : Integer;
                   Const AFormat : TPTCFormat; APages : Integer = 0); Overload; Override;
    Procedure Open(Const ATitle : String; Const AMode : TPTCMode;
                   APages : Integer = 0); Overload; Override;
    Procedure Close; Override;

    Procedure Copy(Var ASurface : TPTCBaseSurface); Override;
    Procedure Copy(Var ASurface : TPTCBaseSurface;
                   Const ASource, ADestination : TPTCArea); Override;

    Procedure Load(Const APixels : Pointer;
                   AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat;
                   Const APalette : TPTCPalette); Override;
    Procedure Load(Const APixels : Pointer;
                   AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat;
                   Const APalette : TPTCPalette;
                   Const ASource, ADestination : TPTCArea); Override;
    Procedure Save(APixels : Pointer;
                   AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat;
                   Const APalette : TPTCPalette); Override;
    Procedure Save(APixels : Pointer;
                   AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat;
                   Const APalette : TPTCPalette;
                   Const ASource, ADestination : TPTCArea); Override;

    Function Lock : Pointer; Override;
    Procedure Unlock; Override;

    Procedure Clear; Override;
    Procedure Clear(Const AColor : TPTCColor); Override;
    Procedure Clear(Const AColor : TPTCColor;
                    Const AArea : TPTCArea); Override;

    Procedure Configure(Const AFileName : String); Override;
    Function Option(Const AOption : String) : Boolean; Override;

    Procedure Palette(Const APalette : TPTCPalette); Override;
    Procedure Clip(Const AArea : TPTCArea); Override;
    Function Clip : TPTCArea; Override;
    Function Palette : TPTCPalette; Override;
    Function Modes : PPTCMode; Override;

    Procedure Flush; Override;
    Procedure Finish; Override;
    Procedure Update; Override;
    Procedure Update(Const AArea : TPTCArea); Override;

    Function NextEvent(Var AEvent : TPTCEvent; AWait : Boolean; Const AEventMask : TPTCEventMask) : Boolean; Override;
    Function PeekEvent(AWait : Boolean; Const AEventMask : TPTCEventMask) : TPTCEvent; Override;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
