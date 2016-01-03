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
  TPTCConsole=Class(TPTCBaseConsole)
  Private
    Function ConsoleCreate(index : Integer) : TPTCBaseConsole;
    Function ConsoleCreate(Const AName : String) : TPTCBaseConsole;
    Procedure check;
    console : TPTCBaseConsole;
    m_modes : Array[0..1023] Of TPTCMode;
    hacky_option_console_flag : Boolean;
  Public
    Constructor Create; Override;
    Destructor Destroy; Override;
    Procedure configure(Const _file : String); Override;
    Function option(Const _option : String) : Boolean; Override;
    Function modes : PPTCMode; Override;
    Procedure open(Const _title : String; _pages : Integer = 0); Overload; Override;
    Procedure open(Const _title : String; Const _format : TPTCFormat;
                   _pages : Integer = 0); Overload; Override;
    Procedure open(Const _title : String; _width, _height : Integer;
                   Const _format : TPTCFormat; _pages : Integer = 0); Overload; Override;
    Procedure open(Const _title : String; Const _mode : TPTCMode;
                   _pages : Integer = 0); Overload; Override;

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
    Function Palette : TPTCPalette; Override;
    Procedure Clip(Const _area : TPTCArea); Override;
    Function GetWidth : Integer; Override;
    Function GetHeight : Integer; Override;
    Function GetPitch : Integer; Override;
    Function GetPages : Integer; Override;
    Function GetArea : TPTCArea; Override;
    Function Clip : TPTCArea; Override;
    Function GetFormat : TPTCFormat; Override;
    Function GetName : String; Override;
    Function GetTitle : String; Override;
    Function GetInformation : String; Override;
    Function NextEvent(Var event : TPTCEvent; wait : Boolean; Const EventMask : TPTCEventMask) : Boolean; Override;
    Function PeekEvent(wait : Boolean; Const EventMask : TPTCEventMask) : TPTCEvent; Override;
  End;
