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
  TTextFX2Console = class(TPTCOpenGLLessConsole)
  private
    { data }
    FModes: array of IPTCMode;
    FTitle: string;
    FInformation: string;

    { flags }
    FOpen: Boolean;
    FLocked: Boolean;

    { option data }
    FDefaultWidth: Integer;
    FDefaultHeight: Integer;
    FDefaultFormat: IPTCFormat;

    { objects }
    FCopy: TPTCCopy;
    FClear: TPTCClear;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TTextFX2Console = Class(TPTCBaseConsole)
  private
    { data }
    m_modes: array [0..255] of TPTCMode;
    m_title: string;
    m_information: string;

    { flags }
    m_open: Boolean;
    m_locked: Boolean;

    { option data }
    m_default_width: Integer;
    m_default_height: Integer;
    m_default_format: TPTCFormat;

    { objects }
    m_copy: TPTCCopy;
    m_clear: TPTCClear;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FKeyboard: TDosKeyboard;
    FMouse: TDosMouse;
    FPrimary: TPTCSurface;
    F160x100buffer: TPTCSurface;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    m_keyboard: TDosKeyboard;
    FMouse: TDosMouse;
    m_primary: TPTCSurface;
    m_160x100buffer: TPTCSurface;
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

    { internal console management routines }
    procedure internal_pre_open_setup(const _title: String);
    procedure internal_open_fullscreen_start;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure internal_open_fullscreen(_width, _height: Integer; _format: IPTCFormat);
=======
    procedure internal_open_fullscreen(_width, _height: Integer; const _format: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
    procedure internal_open_fullscreen(_width, _height: Integer; const _format: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
    procedure internal_open_fullscreen(_width, _height: Integer; const _format: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
    procedure internal_open_fullscreen(_width, _height: Integer; const _format: TPTCFormat);
>>>>>>> origin/cpstrnew
    procedure internal_open_fullscreen_finish(_pages: Integer);
    procedure internal_post_open_setup;
    procedure internal_reset;
    procedure internal_close;

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
    procedure open(const _title: string; _pages: Integer); overload; override;
    procedure open(const _title: string; _format: IPTCFormat;
                   _pages: Integer); overload; override;
    procedure open(const _title: string; _width, _height: Integer;
                   _format: IPTCFormat; _pages: Integer); overload; override;
    procedure open(const _title: string; _mode: IPTCMode;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure Configure(const AFileName: String); override;
    function option(const _option: String): Boolean; override;
    function modes: PPTCMode; override;
    procedure open(const _title: string; _pages: Integer); overload; override;
    procedure open(const _title: string; const _format: TPTCFormat;
                   _pages: Integer); overload; override;
    procedure open(const _title: string; _width, _height: Integer;
                   const _format: TPTCFormat; _pages: Integer); overload; override;
    procedure open(const _title: string; const _mode: TPTCMode;
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
                   _pages: Integer); overload; override;
    procedure close; override;
    procedure flush; override;
    procedure finish; override;
    procedure update; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure update(_area: IPTCArea); override;
    procedure copy(surface: IPTCSurface); override;
    procedure copy(surface: IPTCSurface;
                   source, destination: IPTCArea); override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure update(const _area: TPTCArea); override;
    procedure copy(surface: TPTCBaseSurface); override;
    procedure copy(surface: TPTCBaseSurface;
                   const source, destination: TPTCArea); override;
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
    function lock: Pointer; override;
    procedure unlock; override;
    procedure load(const pixels: Pointer;
                   _width, _height, _pitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                   _format: IPTCFormat;
                   _palette: IPTCPalette); override;
    procedure load(const pixels: Pointer;
                   _width, _height, _pitch: Integer;
                   _format: IPTCFormat;
                   _palette: IPTCPalette;
                   source, destination: IPTCArea); override;
    procedure save(pixels: Pointer;
                   _width, _height, _pitch: Integer;
                   _format: IPTCFormat;
                   _palette: IPTCPalette); override;
    procedure save(pixels: Pointer;
                   _width, _height, _pitch: Integer;
                   _format: IPTCFormat;
                   _palette: IPTCPalette;
                   source, destination: IPTCArea); override;
    procedure clear; override;
    procedure Clear(AColor: IPTCColor); override;
    procedure clear(color: IPTCColor;
                    _area: IPTCArea); override;
    procedure Palette(_palette: IPTCPalette); override;
    function Palette: IPTCPalette; override;
    procedure Clip(_area: IPTCArea); override;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                   const _format: TPTCFormat;
                   const _palette: TPTCPalette); override;
    procedure load(const pixels: Pointer;
                   _width, _height, _pitch: Integer;
                   const _format: TPTCFormat;
                   const _palette: TPTCPalette;
                   const source, destination: TPTCArea); override;
    procedure save(pixels: Pointer;
                   _width, _height, _pitch: Integer;
                   const _format: TPTCFormat;
                   const _palette: TPTCPalette); override;
    procedure save(pixels: Pointer;
                   _width, _height, _pitch: Integer;
                   const _format: TPTCFormat;
                   const _palette: TPTCPalette;
                   const source, destination: TPTCArea); override;
    procedure clear; override;
    procedure clear(const color: TPTCColor); override;
    procedure clear(const color: TPTCColor;
                    const _area: TPTCArea); override;
    procedure Palette(const _palette: TPTCPalette); override;
    function Palette: TPTCPalette; override;
    procedure Clip(const _area: TPTCArea); override;
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
    function NextEvent(out event: IPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): IPTCEvent; override;
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
    function NextEvent(var event: TPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent; override;
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
