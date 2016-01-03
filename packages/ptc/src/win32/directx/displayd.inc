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
  TDirectXDisplay = Class(TObject)
  Private
    Procedure internal_open(Const _mode : TPTCMode; exact : Boolean; frequency : Integer);
    Procedure internal_open_nearest(Const _mode : TPTCMode; exact : Boolean; frequency : Integer);
    Procedure internal_dispose_modes;
    Procedure internal_dispose_resolutions;
    m_open : Boolean;
    m_fullscreen : Boolean;
    m_mode : TPTCMode;
    m_window : HWND;
    m_ddraw : LPDIRECTDRAW2;
    m_modes_count : Integer;
    m_resolutions_count : Integer;
    m_modes : Array[0..255] Of TPTCMode;
    m_resolutions : Array[0..255] Of TPTCMode;
    m_information : String;

    m_cursorsaved : Boolean;
    m_cursorX, m_cursorY : Integer;
{    m_foreground : HWND;
    m_foreground_rect : RECT;
    m_foreground_placement : WINDOWPLACEMENT;}
  Public
    Constructor Create;
    Destructor Destroy; Override;
    Procedure setup(ddraw : LPDIRECTDRAW2);
    Function modes : PPTCMode;
    Function test(Const _mode : TPTCMode; exact : Boolean) : Boolean;
    Procedure cooperative(window : HWND; _fullscreen : Boolean);
    Procedure open;
    Procedure open(Const _mode : TPTCMode; exact : Boolean; frequency : Integer);
    Procedure close;
    Procedure save;
    Procedure restore;
    Function mode : TPTCMode;
    Function fullscreen : Boolean;
    Function information : String;
  End;
