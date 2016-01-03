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
  IPTCFormat = interface
    function GetHermesFormat: PHermesFormat;

    function Equals(AFormat: IPTCFormat): Boolean;

    function GetR: Uint32;
    function GetG: Uint32;
    function GetB: Uint32;
    function GetA: Uint32;
    function GetBits: Integer;
    function GetIndexed: Boolean;
    function GetDirect: Boolean;
    function GetBytes: Integer;

    property R: Uint32 read GetR;
    property G: Uint32 read GetG;
    property B: Uint32 read GetB;
    property A: Uint32 read GetA;
    property Bits: Integer read GetBits;
    property Indexed: Boolean read GetIndexed;
    property Direct: Boolean read GetDirect;
    property Bytes: Integer read GetBytes;
  end;

  TPTCFormatFactory = class
  public
    class function CreateNew: IPTCFormat;
    class function CreateNew(ABits: Integer): IPTCFormat;
    class function CreateNew(ABits: Integer;
                             ARedMask, AGreenMask, ABlueMask: Uint32;
                             AAlphaMask: Uint32 = 0): IPTCFormat;
    class function CreateNew(AFormat: IPTCFormat): IPTCFormat;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TPTCFormat = class
  private
    FFormat: THermesFormat;
    function GetDirect: Boolean;
    function GetBytes: Integer;
  public
    constructor Create;
    constructor Create(ABits: Integer);
    constructor Create(ABits: Integer;
                       ARedMask, AGreenMask, ABlueMask: Uint32;
                       AAlphaMask: Uint32 = 0);
    constructor Create(const format: TPTCFormat);
    destructor Destroy; override;
    procedure Assign(const format: TPTCFormat);
    function Equals(const format: TPTCFormat): Boolean;
    property R: Uint32 read FFormat.r;
    property G: Uint32 read FFormat.g;
    property B: Uint32 read FFormat.b;
    property A: Uint32 read FFormat.a;
    property Bits: Integer read FFormat.bits;
    property Indexed: Boolean read FFormat.indexed;
    property Direct: Boolean read GetDirect;
    property Bytes: Integer read GetBytes;
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
