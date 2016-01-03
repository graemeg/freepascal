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

Constructor TPTCClear.Create;

Begin
  FFormat := Nil;
  { initialize hermes }
  If Not Hermes_Init Then
    Raise TPTCError.Create('could not initialize hermes');

  { default current format }
  FFormat := TPTCFormat.Create;
  { create hermes clearer instance }
  FHandle := Hermes_ClearerInstance;
  { check hermes clearer instance }
  If FHandle = 0 Then
    Raise TPTCError.Create('could not create hermes clearer instance');
End;

Destructor TPTCClear.Destroy;

Begin
  { return the clearer instance }
  Hermes_ClearerReturn(FHandle);
  FFormat.Free;

  { free hermes }
  Hermes_Done;

  Inherited Destroy;
End;

Procedure TPTCClear.Request(Const AFormat : TPTCFormat);

Var
  hermes_format : PHermesFormat;

Begin
  hermes_format := @AFormat.FFormat;
  { request surface clear for this format }
  If Not Hermes_ClearerRequest(FHandle, hermes_format) Then
    Raise TPTCError.Create('unsupported clear format');

  { update current format }
  FFormat.Assign(AFormat);
End;

Procedure TPTCClear.Clear(APixels : Pointer; AX, AY, AWidth, AHeight, APitch : Integer; Const AColor : TPTCColor);

Var
  r, g, b, a : LongInt;
  index : LongInt;

Begin
  If APixels = Nil Then
    Raise TPTCError.Create('nil pixels pointer in clear');

  { check format type }
  If FFormat.direct Then
  Begin
    { check color type }
    If Not AColor.direct Then
      Raise TPTCError.Create('direct pixel formats can only be cleared with direct color');

    { setup clear color }
    r := Trunc(AColor.R * 255);
    g := Trunc(AColor.G * 255);
    b := Trunc(AColor.B * 255);
    a := Trunc(AColor.A * 255);

    { clamp red }
    If r > 255 Then
      r := 255
    Else
      If r < 0 Then
        r := 0;

    { clamp green }
    If g > 255 Then
      g := 255
    Else
      If g < 0 Then
        g := 0;

    { clamp blue }
    If b > 255 Then
      b := 255
    Else
      If b < 0 Then
        b := 0;

    { clamp alpha }
    If a > 255 Then
      a := 255
    Else
      If a < 0 Then
        a := 0;

    { perform the clearing }
    Hermes_ClearerClear(FHandle, APixels, AX, AY, AWidth, AHeight, APitch,
                        r, g, b, a);
  End
  Else
  Begin
    { check color type }
    If Not AColor.indexed Then
      Raise TPTCError.Create('indexed pixel formats can only be cleared with indexed color');

    { setup clear index }
    index := AColor.index;

    { clamp color index }
    If index > 255 Then
      index := 255
    Else
      If index < 0 Then
        index := 0;

    { perform the clearing }
    Hermes_ClearerClear(FHandle, APixels, AX, AY, AWidth, AHeight, APitch,
                        0, 0, 0, index);
  End;
End;
