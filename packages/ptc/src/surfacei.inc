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

Constructor TPTCSurface.Create(_width, _height : Integer; Const _format : TPTCFormat);

Var
  size : Integer;

Begin
  m_pixels := Nil;
  m_copy := Nil;
  m_clear := Nil;
  m_palette := Nil;
  m_format := Nil;
  m_area := Nil;
  m_clip := Nil;
  m_locked := False;
  LOG('creating surface');
  LOG('width', _width);
  LOG('height', _height);
  LOG('format', _format);
  m_width := _width;
  m_height := _height;
  m_format := TPTCFormat.Create(_format);
  m_area := TPTCArea.Create(0, 0, width, height);
  m_clip := TPTCArea.Create(m_area);
  m_pitch := width * _format.bytes;
  size := width * height * _format.bytes;
  If size = 0 Then
    Raise TPTCError.Create('zero surface size');
  m_pixels := GetMem(size);
  If m_pixels = Nil Then
    Raise TPTCError.Create('could not allocate surface pixels');
  m_copy := TPTCCopy.Create;
  m_clear := TPTCClear.Create;
  m_palette := TPTCPalette.Create;
  clear;
End;

Destructor TPTCSurface.Destroy;

Begin
  If m_locked Then
    Raise TPTCError.Create('surface is still locked');
  m_copy.Free;
  m_clear.Free;
  m_palette.Free;
  m_clip.Free;
  m_area.Free;
  m_format.Free;
  If m_pixels <> Nil Then
    FreeMem(m_pixels);
  Inherited Destroy;
End;

Procedure TPTCSurface.copy(Var surface : TPTCBaseSurface);

Begin
  surface.load(m_pixels, m_width, m_height, m_pitch, m_format, m_palette);
End;

Procedure TPTCSurface.copy(Var surface : TPTCBaseSurface;
                           Const source, destination : TPTCArea);

Begin
  surface.load(m_pixels, m_width, m_height, m_pitch, m_format, m_palette,
               source, destination);
End;

Function TPTCSurface.lock : Pointer;

Begin
  If m_locked Then
    Raise TPTCError.Create('surface is already locked');
  m_locked := True;
  lock := m_pixels;
End;

Procedure TPTCSurface.unlock;

Begin
  If Not m_locked Then
    Raise TPTCError.Create('surface is not locked');
  m_locked := False;
End;

Procedure TPTCSurface.load(Const pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette);

Var
  Area_ : TPTCArea;

Begin
  If m_clip.Equals(m_area) Then
  Begin
    m_copy.request(_format, m_format);
    m_copy.palette(_palette, m_palette);
    m_copy.copy(pixels, 0, 0, _width, _height, _pitch, m_pixels, 0, 0,
                m_width, m_height, m_pitch);
  End
  Else
  Begin
    Area_ := TPTCArea.Create(0, 0, _width, _height);
    Try
      load(pixels, _width, _height, _pitch, _format, _palette, Area_, m_area);
    Finally
      Area_.Free;
    End;
  End;
End;

Procedure TPTCSurface.load(Const pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette;
                           Const source, destination : TPTCArea);

Var
  clipped_source, clipped_destination : TPTCArea;
  area_ : TPTCArea;

Begin
  clipped_source := Nil;
  clipped_destination := Nil;
  area_ := Nil;
  Try
    clipped_source := TPTCArea.Create;
    clipped_destination := TPTCArea.Create;
    area_ := TPTCArea.Create(0, 0, _width, _height);
    TPTCClipper.clip(source, area_, clipped_source, destination, m_clip,
                     clipped_destination);
    m_copy.request(_format, m_format);
    m_copy.palette(_palette, m_palette);
    m_copy.copy(pixels, clipped_source.left, clipped_source.top,
                clipped_source.width, clipped_source.height, _pitch,
                m_pixels, clipped_destination.left, clipped_destination.top,
                clipped_destination.width, clipped_destination.height, m_pitch);
  Finally
    clipped_source.Free;
    clipped_destination.Free;
    area_.Free;
  End;
End;

Procedure TPTCSurface.save(pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette);

Var
  area_ : TPTCArea;

Begin
  If m_clip.Equals(m_area) Then
  Begin
    m_copy.request(m_format, _format);
    m_copy.palette(m_palette, _palette);
    m_copy.copy(m_pixels, 0, 0, m_width, m_height, m_pitch, pixels, 0, 0,
                _width, _height, _pitch);
  End
  Else
  Begin
    area_ := TPTCArea.Create(0, 0, width, height);
    Try
      save(pixels, _width, _height, _pitch, _format, _palette, m_area, area_);
    Finally
      area_.Free;
    End;
  End;
End;

Procedure TPTCSurface.save(pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette;
                           Const source, destination : TPTCArea);

Var
  clipped_source, clipped_destination : TPTCArea;
  area_ : TPTCArea;

Begin
  clipped_source := Nil;
  clipped_destination := Nil;
  area_ := Nil;
  Try
    clipped_source := TPTCArea.Create;
    clipped_destination := TPTCArea.Create;
    area_ := TPTCArea.Create(0, 0, _width, _height);
    TPTCClipper.clip(source, m_clip, clipped_source, destination, area_,
                     clipped_destination);
    m_copy.request(m_format, _format);
    m_copy.palette(m_palette, _palette);
    m_copy.copy(m_pixels, clipped_source.left, clipped_source.top,
                clipped_source.width, clipped_source.height, m_pitch,
                pixels, clipped_destination.left, clipped_destination.top,
                clipped_destination.width, clipped_destination.height, _pitch);
  Finally
    clipped_source.Free;
    clipped_destination.Free;
    area_.Free;
  End;
End;

Procedure TPTCSurface.clear;

Var
  Color : TPTCColor;

Begin
  If format.direct Then
    Color := TPTCColor.Create(0, 0, 0, 0)
  Else
    Color := TPTCColor.Create(0);
  Try
    clear(Color);
  Finally
    Color.Free;
  End;
End;

Procedure TPTCSurface.clear(Const color : TPTCColor);

Begin
  clear(color, m_area);
End;

Procedure TPTCSurface.clear(Const color : TPTCColor; Const _area : TPTCArea);

Var
  clipped_area : TPTCArea;

Begin
  clipped_area := TPTCClipper.clip(_area, m_clip);
  Try
    m_clear.request(m_format);
    m_clear.clear(m_pixels, clipped_area.left, clipped_area.top,
                  clipped_area.width, clipped_area.height, m_pitch, color);
  Finally
    clipped_area.Free;
  End;
End;

Procedure TPTCSurface.palette(Const _palette : TPTCPalette);

Begin
  m_palette.load(_palette.data^);
End;

Function TPTCSurface.Palette : TPTCPalette;

Begin
  Result := m_palette;
End;

Procedure TPTCSurface.clip(Const _area : TPTCArea);

Var
  tmp : TPTCArea;

Begin
  tmp := TPTCClipper.clip(_area, m_area);
  Try
    m_clip.Assign(tmp);
  Finally
    tmp.Free;
  End;
End;

Function TPTCSurface.GetWidth : Integer;

Begin
  Result := m_width;
End;

Function TPTCSurface.GetHeight : Integer;

Begin
  Result := m_height;
End;

Function TPTCSurface.GetPitch : Integer;

Begin
  Result := m_pitch;
End;

Function TPTCSurface.GetArea : TPTCArea;

Begin
  Result := m_area;
End;

Function TPTCSurface.Clip : TPTCArea;

Begin
  Result := m_clip;
End;

Function TPTCSurface.GetFormat : TPTCFormat;

Begin
  Result := m_format;
End;

Function TPTCSurface.option(Const _option : String) : Boolean;

Begin
  Result := m_copy.option(_option);
End;
