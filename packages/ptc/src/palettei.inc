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

Constructor TPTCPalette.Create;

Var
  zero : Array[0..255] Of Uint32;

Begin
  m_locked := False;
  If Not Hermes_Init Then
    Raise TPTCError.Create('could not initialize hermes');
  m_handle := Hermes_PaletteInstance;
  If m_handle = 0 Then
    Raise TPTCError.Create('could not create hermes palette instance');
  FillChar(zero, SizeOf(zero), 0);
  load(zero);
End;

Constructor TPTCPalette.Create(Const _data : Array Of Uint32);

Begin
  m_locked := False;
  If Not Hermes_Init Then
    Raise TPTCError.Create('could not initialize hermes');
  m_handle := Hermes_PaletteInstance;
  If m_handle = 0 Then
    Raise TPTCError.Create('could not create hermes palette instance');
  load(_data);
End;

Constructor TPTCPalette.Create(Const palette : TPTCPalette);

Begin
  m_locked := False;
  If Not Hermes_Init Then
    Raise TPTCError.Create('could not initialize hermes');
  m_handle := Hermes_PaletteInstance;
  If m_handle = 0 Then
    Raise TPTCError.Create('could not create hermes palette instance');
  Assign(palette);
End;

Destructor TPTCPalette.Destroy;

Begin
  If m_locked Then
    Raise TPTCError.Create('palette is still locked');
  Hermes_PaletteReturn(m_handle);
  Hermes_Done;
  Inherited Destroy;
End;

Procedure TPTCPalette.Assign(Const palette : TPTCPalette);

Begin
  If Self = palette Then
    Raise TPTCError.Create('self assignment is not allowed');
  Hermes_PaletteSet(m_handle, Hermes_PaletteGet(palette.m_handle));
End;

Function TPTCPalette.Equals(Const palette : TPTCPalette) : Boolean;

Begin
  Equals := CompareDWord(Hermes_PaletteGet(m_handle)^, Hermes_PaletteGet(palette.m_handle)^, 1024 Div 4) = 0;
End;

Function TPTCPalette.lock : PUint32;

Begin
  If m_locked Then
    Raise TPTCError.Create('palette is already locked');
  m_locked := True;
  lock := Hermes_PaletteGet(m_handle);
End;

Procedure TPTCPalette.unlock;

Begin
  If Not m_locked Then
    Raise TPTCError.Create('palette is not locked');
  m_locked := False;
End;

Procedure TPTCPalette.load(Const _data : Array Of Uint32);

Begin
  Hermes_PaletteSet(m_handle, @_data);
End;

Procedure TPTCPalette.load(_data : Pointer);

Begin
  Hermes_PaletteSet(m_handle, _data);
End;

Procedure TPTCPalette.save(Var _data : Array Of Uint32);

Begin
  Move(Hermes_PaletteGet(m_handle)^, _data, 1024);
End;

Procedure TPTCPalette.save(_data : Pointer);

Begin
  Move(Hermes_PaletteGet(m_handle)^, _data^, 1024);
End;

Function TPTCPalette.data : PUint32;

Begin
  data := Hermes_PaletteGet(m_handle);
End;
