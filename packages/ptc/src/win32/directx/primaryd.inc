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
  TDirectXPrimary = Class(TObject)
  Private
    Function pack(Const color : TPTCColor; Const _format : TPTCFormat) : int32;
    Procedure analyse(mask : int32; Var base, size : Integer);

    m_width : Integer;
    m_height : Integer;
    m_pages : Integer;
    m_area : TPTCArea;
    m_clip : TPTCArea;
    m_format : TPTCFormat;

    m_active : Boolean;
    m_blocking : Boolean;
    m_centering : Boolean;
    m_fullscreen : Boolean;
    m_synchronize : Boolean;

    m_clear : TPTCClear;

    m_window : TWin32Window;

    m_locked : Pointer;

    m_palette : TPTCPalette;

    m_primary_width : Integer;
    m_primary_height : Integer;

    m_secondary_width : Integer;
    m_secondary_height : Integer;

    m_lpDD2 : LPDIRECTDRAW2;

    m_lpDDS : LPDIRECTDRAWSURFACE;
    m_lpDDS_primary : LPDIRECTDRAWSURFACE;
    m_lpDDS_primary_back : LPDIRECTDRAWSURFACE;
    m_lpDDS_primary_page : Array[0..31] Of LPDIRECTDRAWSURFACE;

    m_lpDDS_secondary : LPDIRECTDRAWSURFACE;

    m_lpDDC : LPDIRECTDRAWCLIPPER;

    m_back, m_front : LPDIRECTDRAWSURFACE;
  Public
    Constructor Create;
    Destructor Destroy; Override;

    Procedure initialize(Window : TWin32Window; lpDD2 : LPDIRECTDRAW2);
    Procedure primary(_pages : Integer; video, fullscreen, _palette, complex : Boolean);
    Procedure secondary(_width, _height : Integer);
    Procedure synchronize(_update : Boolean);
    Procedure centering(center : Boolean);
    Procedure close;

    Procedure update;

    Function lock : Pointer;
    Procedure unlock;

    Procedure clear;
    Procedure clear(Const color : TPTCColor; Const _area : TPTCArea);

    Procedure palette(Const _palette : TPTCPalette);
    Function palette : TPTCPalette;

    Procedure clip(Const _area : TPTCArea);

    Function width : Integer;
    Function height : Integer;
    Function pages : Integer;
    Function pitch : Integer;
    Function area : TPTCArea;
    Function clip : TPTCArea;
    Function format : TPTCFormat;

    Function lpDDS : LPDIRECTDRAWSURFACE;
    Function lpDDS_primary : LPDIRECTDRAWSURFACE;
    Function lpDDS_secondary : LPDIRECTDRAWSURFACE;

    Procedure activate;
    Procedure deactivate;
    Function active : Boolean;
    Procedure block;
    Procedure save;
    Procedure restore;

    Procedure paint;

    Procedure blocking(_blocking : Boolean);
  End;
