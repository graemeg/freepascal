{
    Free Pascal port of the OpenPTC C++ library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2003, 2006, 2007, 2009-2012  Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

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
  IPTCSurface = interface
    function GetWidth: Integer;
    function GetHeight: Integer;
    function GetPitch: Integer;
    function GetArea: IPTCArea;
    function GetFormat: IPTCFormat;

    procedure Copy(ASurface: IPTCSurface);
    procedure Copy(ASurface: IPTCSurface;
                   ASource, ADestination: IPTCArea);
    function Lock: Pointer;
    procedure Unlock;
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette);
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette;
                   ASource, ADestination: IPTCArea);
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette);
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette;
                   ASource, ADestination: IPTCArea);
    procedure Clear;
    procedure Clear(AColor: IPTCColor);
    procedure Clear(AColor: IPTCColor;
                    AArea: IPTCArea);
    procedure Palette(APalette: IPTCPalette);
    procedure Clip(AArea: IPTCArea);
    function Option(const AOption: String): Boolean;
    function Clip: IPTCArea;
    function Palette: IPTCPalette;
    property Width: Integer read GetWidth;
    property Height: Integer read GetHeight;
    property Pitch: Integer read GetPitch;
    property Area: IPTCArea read GetArea;
    property Format: IPTCFormat read GetFormat;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TPTCBaseSurface = class
  private
    function GetWidth: Integer; virtual; abstract;
    function GetHeight: Integer; virtual; abstract;
    function GetPitch: Integer; virtual; abstract;
    function GetArea: TPTCArea; virtual; abstract;
    function GetFormat: TPTCFormat; virtual; abstract;
  public
    procedure Copy(ASurface: TPTCBaseSurface); virtual; abstract;
    procedure Copy(ASurface: TPTCBaseSurface;
                   const ASource, ADestination: TPTCArea); virtual; abstract;
    function Lock: Pointer; virtual; abstract;
    procedure Unlock; virtual; abstract;
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat;
                   const APalette: TPTCPalette); virtual; abstract;
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat;
                   const APalette: TPTCPalette;
                   const ASource, ADestination: TPTCArea); virtual; abstract;
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat;
                   const APalette: TPTCPalette); virtual; abstract;
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   const AFormat: TPTCFormat;
                   const APalette: TPTCPalette;
                   const ASource, ADestination: TPTCArea); virtual; abstract;
    procedure Clear; virtual; abstract;
    procedure Clear(const AColor: TPTCColor); virtual; abstract;
    procedure Clear(const AColor: TPTCColor;
                    const AArea: TPTCArea); virtual; abstract;
    procedure Palette(const APalette: TPTCPalette); virtual; abstract;
    procedure Clip(const AArea: TPTCArea); virtual; abstract;
    function Option(const AOption: String): Boolean; virtual; abstract;
    function Clip: TPTCArea; virtual; abstract;
    function Palette: TPTCPalette; virtual; abstract;
    property Width: Integer read GetWidth;
    property Height: Integer read GetHeight;
    property Pitch: Integer read GetPitch;
    property Area: TPTCArea read GetArea;
    property Format: TPTCFormat read GetFormat;
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
