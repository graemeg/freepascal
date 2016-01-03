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
  IPTCMode = interface
    function GetValid: Boolean;
    function GetWidth: Integer;
    function GetHeight: Integer;
    function GetFormat: IPTCFormat;

    function Equals(AMode: IPTCMode): Boolean;

    property Valid: Boolean read GetValid;
    property Width: Integer read GetWidth;
    property Height: Integer read GetHeight;
    property Format: IPTCFormat read GetFormat;
  end;

  TPTCModeList = array of IPTCMode;

  TPTCModeFactory = class
  public
    class function CreateNew: IPTCMode;
    class function CreateNew(AWidth, AHeight: Integer; AFormat: IPTCFormat): IPTCMode;
    class function CreateNew(AMode: IPTCMode): IPTCMode;
  end;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  PPTCMode=^TPTCMode;
  TPTCMode = class
  private
    FValid: Boolean;
    FWidth: Integer;
    FHeight: Integer;
    FFormat: TPTCFormat;
  public
    constructor Create;
    constructor Create(AWidth, AHeight: Integer; const AFormat: TPTCFormat);
    constructor Create(const mode: TPTCMode);
    destructor Destroy; override;
    procedure Assign(const mode: TPTCMode);
    function Equals(const mode: TPTCMode): Boolean;
    property Valid: Boolean read FValid;
    property Width: Integer read FWidth;
    property Height: Integer read FHeight;
    property Format: TPTCFormat read FFormat;
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
