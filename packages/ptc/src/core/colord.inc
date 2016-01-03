{
    Free Pascal port of the OpenPTC C++ library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2007, 2009-2011  Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
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
  IPTCColor = interface
    function GetIndex: Integer;
    function GetR: Single;
    function GetG: Single;
    function GetB: Single;
    function GetA: Single;
    function GetDirect: Boolean;
    function GetIndexed: Boolean;

    function Equals(AColor: IPTCColor): Boolean;

    property Index: Integer read GetIndex;
    property R: Single read GetR;
    property G: Single read GetG;
    property B: Single read GetB;
    property A: Single read GetA;
    property Direct: Boolean read GetDirect;
    property Indexed: Boolean read GetIndexed;
  end;

  TPTCColorFactory = class
  public
    class function CreateNew: IPTCColor;
    class function CreateNew(AIndex: Integer): IPTCColor;
    class function CreateNew(ARed, AGreen, ABlue: Single; AAlpha: Single = 1): IPTCColor;
    class function CreateNew(AColor: IPTCColor): IPTCColor;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TPTCColor = class
  private
    FIndex: Integer;
    FRed, FGreen, FBlue, FAlpha: Single;
    FDirect: Boolean;
    FIndexed: Boolean;
  public
    constructor Create;
    constructor Create(AIndex: Integer);
    constructor Create(ARed, AGreen, ABlue: Single; AAlpha: Single = 1);
    constructor Create(const AColor: TPTCColor);
    procedure Assign(const AColor: TPTCColor);
    function Equals(const AColor: TPTCColor): Boolean;
    property Index: Integer read FIndex;
    property R: Single read FRed;
    property G: Single read FGreen;
    property B: Single read FBlue;
    property A: Single read FAlpha;
    property Direct: Boolean read FDirect;
    property Indexed: Boolean read FIndexed;
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
