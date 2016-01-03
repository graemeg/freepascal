{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

Type
  TDirectXConsole = Class(TPTCBaseConsole)
  Private
    { internal console management routines }
    Procedure internal_open(Const _title : String; window : HWND; Const mode : TPTCMode; _pages : Integer; exact : Boolean);
    Procedure internal_recycle(Const _title : String; window : HWND; Const mode : TPTCMode; _pages : Integer; exact : Boolean);
    Procedure internal_close;
    Procedure internal_shutdown;

    { internal console open management routines }
    Procedure internal_open_start(Const _title : String; window : HWND);
    Procedure internal_open_finish;
    Procedure internal_open_reset;

    { internal fullscreen open routines }
    Procedure internal_open_fullscreen_start(window : HWND; Const mode : TPTCMode; exact : Boolean);
    Procedure internal_open_fullscreen_change(Const mode : TPTCMode; exact : Boolean);
    Procedure internal_open_fullscreen_surface(Const mode : TPTCMode; _pages : Integer);
    Procedure internal_open_fullscreen_finish;

    { internal windowed open routines }
    Procedure internal_open_windowed_start(window : HWND; Const mode : TPTCMode; exact : Boolean);
    Procedure internal_open_windowed_change(Const mode : TPTCMode; exact : Boolean);
    Procedure internal_open_windowed_surface(Const mode : TPTCMode; _pages : Integer);
    Procedure internal_open_windowed_finish;

    { internal console open recycling routines }
    Procedure internal_recycle_fullscreen(Const _title : String; window : HWND; Const mode : TPTCMode; _pages : Integer; exact : Boolean);
    Procedure internal_recycle_windowed(Const _title : String; window : HWND; Const mode : TPTCMode; _pages : Integer; exact : Boolean);

{$IFDEF DEBUG}
    { debug }
    Procedure CHECK_OPEN(msg : String);
    Procedure CHECK_LOCK(msg : String);
{$ENDIF}

    { painting }
    Procedure paint;

    { cursor state }
    Procedure update_cursor;

    { title data }
{    m_title : Array[0..1023] Of Char;}
    m_title : AnsiString;

    { flags }
    m_open : Boolean;
    m_locked : Boolean;
    m_cursor : Boolean;

    { option data }
    m_frequency : Integer;
    m_default_width : Integer;
    m_default_height : Integer;
    m_default_pages : Integer;
    m_center_window : Boolean;
    m_synchronized_update : Boolean;
    m_default_format : TPTCFormat;
    m_output_mode : Integer; {Output}
    m_window_mode : Integer; {Window}
    m_primary_mode_windowed : Integer; {Primary}
    m_primary_mode_fullscreen : Integer; {Primary}
    m_nearest_mode : Integer; {Nearest}
    m_cursor_mode : Integer; {Cursor}

    { objects }
    m_copy : TPTCCopy;

    { Win32 objects }
    m_window : TWin32Window;
    m_keyboard : TWin32Keyboard;

    { DirectX objects }
    m_hook : TDirectXHook;
    m_library : TDirectXLibrary;
    m_display : TDirectXDisplay;
    m_primary : TDirectXPrimary;
  Protected
    Procedure internal_ReadKey(k : TPTCKey); Override;
    Function internal_PeekKey(k : TPTCKey) : Boolean; Override;
  Public
    Constructor Create;
    Destructor Destroy; Override;
    Procedure configure(Const _file : String); Override;
    Function option(Const _option : String) : Boolean; Override;
    Function modes : PPTCMode; Override;
    Procedure open(Const _title : String; _pages : Integer); Overload; Override;
    Procedure open(Const _title : String; Const _format : TPTCFormat;
                   _pages : Integer); Overload; Override;
    Procedure open(Const _title : String; _width, _height : Integer;
                   Const _format : TPTCFormat; _pages : Integer); Overload; Override;
    Procedure open(Const _title : String; Const _mode : TPTCMode;
                   _pages : Integer); Overload; Override;
    Procedure close; Override;
    Procedure flush; Override;
    Procedure finish; Override;
    Procedure update; Override;
    Procedure update(Const _area : TPTCArea); Override;
    Procedure copy(Var surface : TPTCBaseSurface); Override;
    Procedure copy(Var surface : TPTCBaseSurface;
                   Const source, destination : TPTCArea); Override;
    Function lock : Pointer; Override;
    Procedure unlock; Override;
    Procedure load(Const pixels : Pointer;
                   _width, _height, _pitch : Integer;
                   Const _format : TPTCFormat;
                   Const _palette : TPTCPalette); Override;
    Procedure load(Const pixels : Pointer;
                   _width, _height, _pitch : Integer;
                   Const _format : TPTCFormat;
                   Const _palette : TPTCPalette;
                   Const source, destination : TPTCArea); Override;
    Procedure save(pixels : Pointer;
                   _width, _height, _pitch : Integer;
                   Const _format : TPTCFormat;
                   Const _palette : TPTCPalette); Override;
    Procedure save(pixels : Pointer;
                   _width, _height, _pitch : Integer;
                   Const _format : TPTCFormat;
                   Const _palette : TPTCPalette;
                   Const source, destination : TPTCArea); Override;
    Procedure clear; Override;
    Procedure clear(Const color : TPTCColor); Override;
    Procedure clear(Const color : TPTCColor;
                    Const _area : TPTCArea); Override;
    Procedure palette(Const _palette : TPTCPalette); Override;
    Function palette : TPTCPalette; Override;
    Procedure clip(Const _area : TPTCArea); Override;
    Function width : Integer; Override;
    Function height : Integer; Override;
    Function pitch : Integer; Override;
    Function pages : Integer; Override;
    Function area : TPTCArea; Override;
    Function clip : TPTCArea; Override;
    Function format : TPTCFormat; Override;
    Function name : String; Override;
    Function title : String; Override;
    Function information : String; Override;
  End;
