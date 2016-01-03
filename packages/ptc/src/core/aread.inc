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
  IPTCArea = interface
    function GetLeft: Integer;
    function GetTop: Integer;
    function GetRight: Integer;
    function GetBottom: Integer;
    function GetWidth: Integer;
    function GetHeight: Integer;

    function Equals(AArea: IPTCArea): Boolean;

    property Left: Integer read GetLeft;
    property Top: Integer read GetTop;
    property Right: Integer read GetRight;
    property Bottom: Integer read GetBottom;
    property Width: Integer read GetWidth;
    property Height: Integer read GetHeight;
  end;

  TPTCAreaFactory = class
  public
    class function CreateNew: IPTCArea;
    class function CreateNew(ALeft, ATop, ARight, ABottom: Integer): IPTCArea;
    class function CreateNew(AArea: IPTCArea): IPTCArea;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TPTCArea = class
  private
    FLeft, FTop, FRight, FBottom: Integer;
    function GetWidth: Integer;
    function GetHeight: Integer;
  public
    constructor Create;
    constructor Create(ALeft, ATop, ARight, ABottom: Integer);
    constructor Create(const AArea: TPTCArea);
    procedure Assign(const AArea: TPTCArea);
    function Equals(const AArea: TPTCArea): Boolean;
    property Left: Integer read FLeft;
    property Top: Integer read FTop;
    property Right: Integer read FRight;
    property Bottom: Integer read FBottom;
    property Width: Integer read GetWidth;
    property Height: Integer read GetHeight;
  end;
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
