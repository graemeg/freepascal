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
  TPTCPalette=Class(TObject)
  Private
    m_locked : Boolean;
    m_handle : THermesHandle;
  Public
    Constructor Create;
    Constructor Create(Const _data : Array Of Uint32);
    Constructor Create(Const palette : TPTCPalette);
    Destructor Destroy; Override;
    Procedure Assign(Const palette : TPTCPalette);
    Function Equals(Const palette : TPTCPalette) : Boolean;
    Function lock : PUint32;
    Procedure unlock;
    Procedure load(Const _data : Array Of Uint32);
    Procedure load(_data : Pointer);
    Procedure save(Var _data : Array Of Uint32);
    Procedure save(_data : Pointer);
    Function data : PUint32;
  End;
