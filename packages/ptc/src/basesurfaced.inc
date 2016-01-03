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
  TPTCBaseSurface=Class(TObject)
  Private
    Function GetWidth : Integer; Virtual; Abstract;
    Function GetHeight : Integer; Virtual; Abstract;
    Function GetPitch : Integer; Virtual; Abstract;
    Function GetArea : TPTCArea; Virtual; Abstract;
    Function GetFormat : TPTCFormat; Virtual; Abstract;
  Public
    Procedure Copy(Var ASurface : TPTCBaseSurface); Virtual; Abstract;
    Procedure Copy(Var ASurface : TPTCBaseSurface;
                   Const ASource, ADestination : TPTCArea); Virtual; Abstract;
    Function Lock : Pointer; Virtual; Abstract;
    Procedure Unlock; Virtual; Abstract;
    Procedure Load(Const APixels : Pointer;
                   AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat;
                   Const APalette : TPTCPalette); Virtual; Abstract;
    Procedure Load(Const APixels : Pointer;
                   AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat;
                   Const APalette : TPTCPalette;
                   Const ASource, ADestination : TPTCArea); Virtual; Abstract;
    Procedure Save(APixels : Pointer;
                   AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat;
                   Const APalette : TPTCPalette); Virtual; Abstract;
    Procedure Save(APixels : Pointer;
                   AWidth, AHeight, APitch : Integer;
                   Const AFormat : TPTCFormat;
                   Const APalette : TPTCPalette;
                   Const ASource, ADestination : TPTCArea); Virtual; Abstract;
    Procedure Clear; Virtual; Abstract;
    Procedure Clear(Const AColor : TPTCColor); Virtual; Abstract;
    Procedure Clear(Const AColor : TPTCColor;
                    Const AArea : TPTCArea); Virtual; Abstract;
    Procedure Palette(Const APalette : TPTCPalette); Virtual; Abstract;
    Procedure Clip(Const AArea : TPTCArea); Virtual; Abstract;
    Function Option(Const AOption : String) : Boolean; Virtual; Abstract;
    Function Clip : TPTCArea; Virtual; Abstract;
    Function Palette : TPTCPalette; Virtual; Abstract;
    Property Width : Integer Read GetWidth;
    Property Height : Integer Read GetHeight;
    Property Pitch : Integer Read GetPitch;
    Property Area : TPTCArea Read GetArea;
    Property Format : TPTCFormat Read GetFormat;
  End;
