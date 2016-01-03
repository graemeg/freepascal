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
  TVESAConsole = class(TPTCOpenGLLessConsole)
  private
    { data }
    FModes: array of IPTCMode;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TVESAConsole = class(TPTCBaseConsole)
  private
    { data }
    FModes: array of TPTCMode;
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
    FModesLast: Integer;
    FModesN: array of record
      Index: Integer;
      SupportsWindowed: Boolean;
      SupportsLFB: Boolean;
    end;
    FTitle: string;
    FInformation: string;
    FCurrentMode: Integer;
    FVESACurrentMode: Integer;
    FWidth, FHeight, FPitch{, FPages}: Integer;

    FVideoPageSize: Integer;
    FVideoPagesCount: Integer;
    FCurrentVideoPage: Integer; { 0-based }
    FNextVideoPage: Integer; { 0-based }
    FVideoPageHeight: Integer;

    FPrimary: Pointer;
    FLFBNearPtrAccessAvailable: Boolean;

    { flags }
    FOpen: Boolean;
    FLocked: Boolean;

    FTryLFB: Boolean;
    FTryWindowed: Boolean;

    { option data }
    FDefaultWidth: Integer;
    FDefaultHeight: Integer;
//    FDefaultPages: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FDefaultFormat: IPTCFormat;

    { objects }
    FCopy: TPTCCopy;
    FArea: IPTCArea;
    FClip: IPTCArea;
//    FFormat: IPTCFormat;

//    FClear: TPTCClear;
    FPalette: IPTCPalette;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FDefaultFormat: TPTCFormat;

    { objects }
    FCopy: TPTCCopy;
    FArea: TPTCArea;
    FClip: TPTCArea;
//    FFormat: TPTCFormat;

//    FClear: TPTCClear;
    FPalette: TPTCPalette;
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

    FEventQueue: TEventQueue;

    { Dos objects }
    FKeyboard: TDosKeyboard;
    FMouse: TDosMouse;

    { internal console management routines }
    procedure internal_close;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function FindBestMode(const AMode: IPTCMode): Integer;
=======
    function FindBestMode(const AMode: TPTCMode): Integer;
>>>>>>> graemeg/cpstrnew
=======
    function FindBestMode(const AMode: TPTCMode): Integer;
>>>>>>> graemeg/cpstrnew
=======
    function FindBestMode(const AMode: TPTCMode): Integer;
>>>>>>> graemeg/cpstrnew
=======
    function FindBestMode(const AMode: TPTCMode): Integer;
>>>>>>> origin/cpstrnew
    
    procedure UpdateModeList;
    procedure EnableLFB;
    procedure DisableLFB;
    procedure EnableWindowed;
    procedure DisableWindowed;

    procedure HandleEvents;

    { console debug checks }
    procedure check_open;
    procedure check_unlocked;
  public
    constructor Create; override;
    destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure Configure(const AFileName: string); override;
    function Option(const AOption: string): Boolean; override;
    function Modes: TPTCModeList; override;
    procedure Open(const ATitle: string; APages: Integer); overload; override;
    procedure Open(const ATitle: string; AFormat: IPTCFormat;
                   APages: Integer); overload; override;
    procedure Open(const ATitle: string; AWidth, AHeight: Integer;
                   AFormat: IPTCFormat; APages: Integer); overload; override;
    procedure Open(const ATitle: string; AMode: IPTCMode;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure Configure(const AFileName: String); override;
    function Option(const AOption: String): Boolean; override;
    function Modes: PPTCMode; override;
    procedure Open(const ATitle: string; APages: Integer); overload; override;
    procedure Open(const ATitle: string; const AFormat: TPTCFormat;
                   APages: Integer); overload; override;
    procedure Open(const ATitle: string; AWidth, AHeight: Integer;
                   const AFormat: TPTCFormat; APages: Integer); overload; override;
    procedure Open(const ATitle: string; const AMode: TPTCMode;
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
                   APages: Integer); overload; override;
    procedure Close; override;
    procedure Flush; override;
    procedure Finish; override;
    procedure Update; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure Update(AArea: IPTCArea); override;
    procedure Copy(ASurface: IPTCSurface); override;
    procedure Copy(ASurface: IPTCSurface;
                   ASource, ADestination: IPTCArea); override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure Update(const AArea: TPTCArea); override;
    procedure Copy(ASurface: TPTCBaseSurface); override;
    procedure Copy(ASurface: TPTCBaseSurface;
                   const ASource, ADestination: TPTCArea); override;
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
    function Lock: Pointer; override;
    procedure Unlock; override;
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette); override;
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette;
                   ASource, ADestination: IPTCArea); override;
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette); override;
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette;
                   ASource, ADestination: IPTCArea); override;
    procedure Clear; override;
    procedure Clear(AColor: IPTCColor); override;
    procedure Clear(AColor: IPTCColor;
                    AArea: IPTCArea); override;
    procedure Palette(APalette: IPTCPalette); override;
    function Palette: IPTCPalette; override;
    procedure Clip(AArea: IPTCArea); override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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
    procedure Clear; override;
    procedure Clear(const AColor: TPTCColor); override;
    procedure Clear(const AColor: TPTCColor;
                    const AArea: TPTCArea); override;
    procedure Palette(const APalette: TPTCPalette); override;
    function Palette: TPTCPalette; override;
    procedure Clip(const AArea: TPTCArea); override;
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
    function GetWidth: Integer; override;
    function GetHeight: Integer; override;
    function GetPitch: Integer; override;
    function GetPages: Integer; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    function GetArea: IPTCArea; override;
    function Clip: IPTCArea; override;
    function GetFormat: IPTCFormat; override;
    function GetName: string; override;
    function GetTitle: string; override;
    function GetInformation: string; override;
    function NextEvent(out AEvent: IPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): IPTCEvent; override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    function GetArea: TPTCArea; override;
    function Clip: TPTCArea; override;
    function GetFormat: TPTCFormat; override;
    function GetName: string; override;
    function GetTitle: string; override;
    function GetInformation: string; override;
    function NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent; override;
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
  end;
