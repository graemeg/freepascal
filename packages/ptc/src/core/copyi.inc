{
    Free Pascal port of the OpenPTC C++ library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2003, 2006, 2007, 2009-2011  Nikolay Nikolov (nickysn@users.sourceforge.net)
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

constructor TPTCCopy.Create;
begin
  if not Hermes_Init then
    raise TPTCError.Create('could not initialize hermes');
  FFlags := HERMES_CONVERT_NORMAL;
  FHandle := Hermes_ConverterInstance(FFlags);
  if FHandle = nil then
    raise TPTCError.Create('could not create hermes converter instance');
end;

destructor TPTCCopy.Destroy;
begin
  Hermes_ConverterReturn(FHandle);
  Hermes_Done;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPTCCopy.Request(ASource, ADestination: IPTCFormat);
var
  hermes_source_format, hermes_destination_format: PHermesFormat;
begin
  hermes_source_format := ASource.GetHermesFormat;
  hermes_destination_format := ADestination.GetHermesFormat;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TPTCCopy.Request(const ASource, ADestination: TPTCFormat);
var
  hermes_source_format, hermes_destination_format: PHermesFormat;
begin
  hermes_source_format := @ASource.FFormat;
  hermes_destination_format := @ADestination.FFormat;
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
  if not Hermes_ConverterRequest(FHandle, hermes_source_format,
     hermes_destination_format) then
    raise TPTCError.Create('unsupported hermes pixel format conversion');
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPTCCopy.Palette(ASource, ADestination: IPTCPalette);
begin
  if not Hermes_ConverterPalette(FHandle, ASource.GetHermesPaletteHandle,
         ADestination.GetHermesPaletteHandle) then
    raise TPTCError.Create('could not set hermes conversion palettes');
end;

procedure TPTCCopy.Copy(const ASourcePixels: Pointer; ASourceX, ASourceY,
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TPTCCopy.Palette(const ASource, ADestination: TPTCPalette);
begin
  if not Hermes_ConverterPalette(FHandle, ASource.FHandle,
         ADestination.FHandle) then
    raise TPTCError.Create('could not set hermes conversion palettes');
end;

procedure TPTCCopy.copy(const ASourcePixels: Pointer; ASourceX, ASourceY,
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
                   ASourceWidth, ASourceHeight, ASourcePitch: Integer;
                   ADestinationPixels: Pointer; ADestinationX, ADestinationY,
                   ADestinationWidth, ADestinationHeight, ADestinationPitch: Integer);
begin
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
  if ASourcePixels = nil then
    raise TPTCError.Create('nil source pointer in copy');
  if ADestinationPixels = nil then
    raise TPTCError.Create('nil destination pointer in copy');
  if ASourcePixels = ADestinationPixels then
    raise TPTCError.Create('identical source and destination pointers in copy');
{$ELSE DEBUG}
    { in release build no checking is performed for the sake of efficiency. }
{$ENDIF DEBUG}
  if not Hermes_ConverterCopy(FHandle, ASourcePixels, ASourceX, ASourceY,
          ASourceWidth, ASourceHeight, ASourcePitch, ADestinationPixels,
          ADestinationX, ADestinationY, ADestinationWidth, ADestinationHeight,
          ADestinationPitch) then
    raise TPTCError.Create('hermes conversion failure');
end;

function TPTCCopy.Option(const AOption: String): Boolean;
begin
  if (AOption = 'attempt dithering') and ((FFlags and HERMES_CONVERT_DITHER) = 0) then
  begin
    FFlags := FFlags or HERMES_CONVERT_DITHER;
    Update;
    Result := True;
    exit;
  end;
  if (AOption = 'disable dithering') and ((FFlags and HERMES_CONVERT_DITHER) <> 0) then
  begin
    FFlags := FFlags and (not HERMES_CONVERT_DITHER);
    Update;
    Result := True;
    exit;
  end;
  Result := False;
end;

procedure TPTCCopy.Update;
begin
  Hermes_ConverterReturn(FHandle);
  FHandle := Hermes_ConverterInstance(FFlags);
  if FHandle = nil then
    raise TPTCError.Create('could not update hermes converter instance');
end;
