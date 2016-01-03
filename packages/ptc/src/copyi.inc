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

Constructor TPTCCopy.Create;

Begin
  If Not Hermes_Init Then
    Raise TPTCError.Create('could not initialize hermes');
  FFlags := HERMES_CONVERT_NORMAL;
  FHandle := Hermes_ConverterInstance(FFlags);
  If FHandle = 0 Then
    Raise TPTCError.Create('could not create hermes converter instance');
End;

Destructor TPTCCopy.Destroy;

Begin
  Hermes_ConverterReturn(FHandle);
  Hermes_Done;
  Inherited Destroy;
End;

Procedure TPTCCopy.Request(Const ASource, ADestination : TPTCFormat);

Var
  hermes_source_format, hermes_destination_format : PHermesFormat;

Begin
  hermes_source_format := @ASource.FFormat;
  hermes_destination_format := @ADestination.FFormat;
  If Not Hermes_ConverterRequest(FHandle, hermes_source_format,
     hermes_destination_format) Then
    Raise TPTCError.Create('unsupported hermes pixel format conversion');
End;

Procedure TPTCCopy.Palette(Const ASource, ADestination : TPTCPalette);

Begin
  If Not Hermes_ConverterPalette(FHandle, ASource.m_handle,
         ADestination.m_handle) Then
    Raise TPTCError.Create('could not set hermes conversion palettes');
End;

Procedure TPTCCopy.copy(Const ASourcePixels : Pointer; ASourceX, ASourceY,
                   ASourceWidth, ASourceHeight, ASourcePitch : Integer;
                   ADestinationPixels : Pointer; ADestinationX, ADestinationY,
                   ADestinationWidth, ADestinationHeight, ADestinationPitch : Integer);

Begin
{$IFDEF DEBUG}
{
  This checking is performed only when DEBUG is defined,
  and can be used to track down errors early caused by passing
  nil pointers to surface and console functions.

  Even though technicially it is the users responsibility
  to ensure that all pointers are non-nil, it is useful
  to provide a check here in debug build to prevent such
  bugs from ever occuring.

  The checking function also tests that the source and destination
  pointers are not the same, a bug that can be caused by copying
  a surface to itself. The nature of the copy routine is that
  this operation is undefined if the source and destination memory
  areas overlap.
}
  If ASourcePixels = Nil Then
    Raise TPTCError.Create('nil source pointer in copy');
  If ADestinationPixels = Nil Then
    Raise TPTCError.Create('nil destination pointer in copy');
  If ASourcePixels = ADestinationPixels Then
    Raise TPTCError.Create('identical source and destination pointers in copy');
{$ELSE DEBUG}
    { in release build no checking is performed for the sake of efficiency. }
{$ENDIF DEBUG}
  If Not Hermes_ConverterCopy(FHandle, ASourcePixels, ASourceX, ASourceY,
          ASourceWidth, ASourceHeight, ASourcePitch, ADestinationPixels,
          ADestinationX, ADestinationY, ADestinationWidth, ADestinationHeight,
          ADestinationPitch) Then
    Raise TPTCError.Create('hermes conversion failure');
End;

Function TPTCCopy.Option(Const AOption : String) : Boolean;

Begin
  If (AOption = 'attempt dithering') And ((FFlags And HERMES_CONVERT_DITHER) = 0) Then
  Begin
    FFlags := FFlags Or HERMES_CONVERT_DITHER;
    Update;
    Result := True;
    Exit;
  End;
  If (AOption = 'disable dithering') And ((FFlags And HERMES_CONVERT_DITHER) <> 0) Then
  Begin
    FFlags := FFlags And (Not HERMES_CONVERT_DITHER);
    Update;
    Result := True;
    Exit;
  End;
  Result := False;
End;

Procedure TPTCCopy.Update;

Begin
  Hermes_ConverterReturn(FHandle);
  FHandle := Hermes_ConverterInstance(FFlags);
  If FHandle = 0 Then
    Raise TPTCError.Create('could not update hermes converter instance');
End;
