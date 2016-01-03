{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
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

constructor TPTCClear.Create;
begin
  FFormat := nil;
  { initialize hermes }
  if not Hermes_Init then
    raise TPTCError.Create('could not initialize hermes');

  { default current format }
  FFormat := TPTCFormat.Create;
  { create hermes clearer instance }
  FHandle := Hermes_ClearerInstance;
  { check hermes clearer instance }
  if FHandle = nil then
    raise TPTCError.Create('could not create hermes clearer instance');
end;

destructor TPTCClear.Destroy;
begin
  { return the clearer instance }
  Hermes_ClearerReturn(FHandle);
  FFormat.Free;

  { free hermes }
  Hermes_Done;

  inherited Destroy;
end;

procedure TPTCClear.Request(const AFormat: TPTCFormat);
var
  hermes_format: PHermesFormat;
begin
  hermes_format := @AFormat.FFormat;
  { request surface clear for this format }
  if not Hermes_ClearerRequest(FHandle, hermes_format) then
    raise TPTCError.Create('unsupported clear format');

  { update current format }
  FFormat.Assign(AFormat);
end;

procedure TPTCClear.Clear(APixels: Pointer; AX, AY, AWidth, AHeight, APitch: Integer; const AColor: TPTCColor);
var
  r, g, b, a: LongInt;
  index: LongInt;
begin
  if APixels = nil then
    raise TPTCError.Create('nil pixels pointer in clear');

  { check format type }
  if FFormat.direct then
  begin
    { check color type }
    if not AColor.direct then
      raise TPTCError.Create('direct pixel formats can only be cleared with direct color');

    { setup clear color }
    r := Trunc(AColor.R * 255);
    g := Trunc(AColor.G * 255);
    b := Trunc(AColor.B * 255);
    a := Trunc(AColor.A * 255);

    { clamp red }
    if r > 255 then
      r := 255
    else
      if r < 0 then
        r := 0;

    { clamp green }
    if g > 255 then
      g := 255
    else
      if g < 0 then
        g := 0;

    { clamp blue }
    if b > 255 then
      b := 255
    else
      if b < 0 then
        b := 0;

    { clamp alpha }
    if a > 255 then
      a := 255
    else
      if a < 0 then
        a := 0;

    { perform the clearing }
    Hermes_ClearerClear(FHandle, APixels, AX, AY, AWidth, AHeight, APitch,
                        r, g, b, a);
  end
  else
  begin
    { check color type }
    if not AColor.indexed then
      raise TPTCError.Create('indexed pixel formats can only be cleared with indexed color');

    { setup clear index }
    index := AColor.index;

    { clamp color index }
    if index > 255 then
      index := 255
    else
      if index < 0 then
        index := 0;

    { perform the clearing }
    Hermes_ClearerClear(FHandle, APixels, AX, AY, AWidth, AHeight, APitch,
                        0, 0, 0, index);
  end;
end;
