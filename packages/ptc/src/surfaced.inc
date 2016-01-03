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
  TPTCSurface=Class(TPTCBaseSurface)
  Private
    {data}
    m_width : Integer;
    m_height : Integer;
    m_pitch : Integer;
    m_area : TPTCArea;
    m_clip : TPTCArea;
    m_format : TPTCFormat;
    m_locked : Boolean;
    m_pixels : Pointer;
    {objects}
    m_copy : TPTCCopy;
    m_clear : TPTCClear;
    m_palette : TPTCPalette;
  Public
    Constructor Create(_width, _height : Integer; Const _format : TPTCFormat);
    Destructor Destroy; Override;
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
    Procedure clear(Const color : TPTCColor; Const _area : TPTCArea); Override;
    Procedure palette(Const _palette : TPTCPalette); Override;
    Function Palette : TPTCPalette; Override;
    Procedure clip(Const _area : TPTCArea); Override;
    Function GetWidth : Integer; Override;
    Function GetHeight : Integer; Override;
    Function GetPitch : Integer; Override;
    Function GetArea : TPTCArea; Override;
    Function Clip : TPTCArea; Override;
    Function GetFormat : TPTCFormat; Override;
    Function option(Const _option : String) : Boolean; Override;
  End;
