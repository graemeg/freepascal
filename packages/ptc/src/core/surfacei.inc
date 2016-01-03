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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
  TPTCSurface = class(TInterfacedObject, IPTCSurface)
  private
    {data}
    FWidth: Integer;
    FHeight: Integer;
    FPitch: Integer;
    FArea: IPTCArea;
    FClip: IPTCArea;
    FFormat: IPTCFormat;
    FLocked: Boolean;
    FPixels: Pointer;
    {objects}
    FCopy: TPTCCopy;
    FClear: TPTCClear;
    FPalette: IPTCPalette;
  public
    constructor Create(AWidth, AHeight: Integer; AFormat: IPTCFormat);
    destructor Destroy; override;
    procedure Copy(ASurface: IPTCSurface);
    procedure Copy(ASurface: IPTCSurface;
                   ASource, ADestination: IPTCArea);
    function Lock: Pointer;
    procedure Unlock;
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette);
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette;
                   ASource, ADestination: IPTCArea);
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette);
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette;
                   ASource, ADestination: IPTCArea);
    procedure Clear;
    procedure Clear(AColor: IPTCColor);
    procedure Clear(AColor: IPTCColor; AArea: IPTCArea);
    procedure Palette(APalette: IPTCPalette);
    function Palette: IPTCPalette;
    procedure Clip(AArea: IPTCArea);
    function GetWidth: Integer;
    function GetHeight: Integer;
    function GetPitch: Integer;
    function GetArea: IPTCArea;
    function Clip: IPTCArea;
    function GetFormat: IPTCFormat;
    function Option(const AOption: string): Boolean;

    property Width: Integer read GetWidth;
    property Height: Integer read GetHeight;
    property Pitch: Integer read GetPitch;
    property Area: IPTCArea read GetArea;
    property Format: IPTCFormat read GetFormat;
  end;

class function TPTCSurfaceFactory.CreateNew(AWidth, AHeight: Integer; AFormat: IPTCFormat): IPTCSurface;
begin
  Result := TPTCSurface.Create(AWidth, AHeight, AFormat);
end;

constructor TPTCSurface.Create(AWidth, AHeight: Integer; AFormat: IPTCFormat);
=======
constructor TPTCSurface.Create(AWidth, AHeight: Integer; const AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TPTCSurface.Create(AWidth, AHeight: Integer; const AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TPTCSurface.Create(AWidth, AHeight: Integer; const AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
constructor TPTCSurface.Create(AWidth, AHeight: Integer; const AFormat: TPTCFormat);
>>>>>>> origin/cpstrnew
var
  size: Integer;
begin
  FLocked := False;
  LOG('creating surface');
  LOG('width', AWidth);
  LOG('height', AHeight);
  LOG('format', AFormat);
  FWidth := AWidth;
  FHeight := AHeight;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FFormat := AFormat;
=======
  FFormat := TPTCFormat.Create(AFormat);
>>>>>>> graemeg/cpstrnew
=======
  FFormat := TPTCFormat.Create(AFormat);
>>>>>>> graemeg/cpstrnew
=======
  FFormat := TPTCFormat.Create(AFormat);
>>>>>>> graemeg/cpstrnew
=======
  FFormat := TPTCFormat.Create(AFormat);
>>>>>>> origin/cpstrnew
  FArea := TPTCArea.Create(0, 0, AWidth, AHeight);
  FClip := TPTCArea.Create(FArea);
  FPitch := AWidth * AFormat.Bytes;
  size := AWidth * AHeight * AFormat.Bytes;
  if size = 0 then
    raise TPTCError.Create('zero surface size');
  FPixels := GetMem(size);
  FCopy := TPTCCopy.Create;
  FClear := TPTCClear.Create;
  FPalette := TPTCPalette.Create;
  clear;
end;

destructor TPTCSurface.Destroy;
begin
  if FLocked then
  begin
    LOG('destroying surface that is still locked!!!');
  end;
  FCopy.Free;
  FClear.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FPalette.Free;
  FClip.Free;
  FArea.Free;
  FFormat.Free;
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
  FreeMem(FPixels);
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPTCSurface.Copy(ASurface: IPTCSurface);
=======
procedure TPTCSurface.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCSurface.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCSurface.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCSurface.Copy(ASurface: TPTCBaseSurface);
>>>>>>> origin/cpstrnew
begin
  ASurface.Load(FPixels, FWidth, FHeight, FPitch, FFormat, FPalette);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPTCSurface.Copy(ASurface: IPTCSurface;
                           ASource, ADestination: IPTCArea);
=======
procedure TPTCSurface.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCSurface.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCSurface.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCSurface.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  ASurface.Load(FPixels, FWidth, FHeight, FPitch, FFormat, FPalette,
                ASource, ADestination);
end;

function TPTCSurface.Lock: Pointer;
begin
  if FLocked then
    raise TPTCError.Create('surface is already locked');
  FLocked := True;
  Result := FPixels;
end;

procedure TPTCSurface.Unlock;
begin
  if not FLocked then
    raise TPTCError.Create('surface is not locked');
  FLocked := False;
end;

procedure TPTCSurface.Load(const APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat;
                           APalette: IPTCPalette);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
var
  Area_: TPTCArea;
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
begin
  if FClip.Equals(FArea) then
  begin
    FCopy.Request(AFormat, FFormat);
    FCopy.Palette(APalette, FPalette);
    FCopy.Copy(APixels, 0, 0, AWidth, AHeight, APitch, FPixels, 0, 0,
               FWidth, FHeight, FPitch);
  end
  else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Load(APixels, AWidth, AHeight, APitch, AFormat, APalette,
         TPTCArea.Create(0, 0, AWidth, AHeight), FArea);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  begin
    Area_ := TPTCArea.Create(0, 0, AWidth, AHeight);
    try
      Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, Area_, FArea);
    finally
      Area_.Free;
    end;
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
end;

procedure TPTCSurface.Load(const APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat;
                           APalette: IPTCPalette;
                           ASource, ADestination: IPTCArea);
var
  clipped_source: IPTCArea;
  clipped_destination: IPTCArea;
begin
  TPTCClipper.Clip(ASource, TPTCArea.Create(0, 0, AWidth, AHeight),
                   clipped_source,
                   ADestination, FClip,
                   clipped_destination);
  FCopy.Request(AFormat, FFormat);
  FCopy.Palette(APalette, FPalette);
  FCopy.Copy(APixels, clipped_source.left, clipped_source.top,
             clipped_source.width, clipped_source.height, APitch,
             FPixels, clipped_destination.left, clipped_destination.top,
             clipped_destination.width, clipped_destination.height, FPitch);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
var
  clipped_source: TPTCArea = nil;
  clipped_destination: TPTCArea = nil;
  area_: TPTCArea = nil;
begin
  try
    clipped_source := TPTCArea.Create;
    clipped_destination := TPTCArea.Create;
    area_ := TPTCArea.Create(0, 0, AWidth, AHeight);
    TPTCClipper.Clip(ASource, area_, clipped_source, ADestination, FClip,
                     clipped_destination);
    FCopy.Request(AFormat, FFormat);
    FCopy.Palette(APalette, FPalette);
    FCopy.Copy(APixels, clipped_source.left, clipped_source.top,
               clipped_source.width, clipped_source.height, APitch,
               FPixels, clipped_destination.left, clipped_destination.top,
               clipped_destination.width, clipped_destination.height, FPitch);
  finally
    clipped_source.Free;
    clipped_destination.Free;
    area_.Free;
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
end;

procedure TPTCSurface.Save(APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat;
                           APalette: IPTCPalette);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
var
  area_: TPTCArea;
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
begin
  if FClip.Equals(FArea) then
  begin
    FCopy.Request(FFormat, AFormat);
    FCopy.Palette(FPalette, APalette);
    FCopy.Copy(FPixels, 0, 0, FWidth, FHeight, FPitch, APixels, 0, 0,
               AWidth, AHeight, APitch);
  end
  else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Save(APixels, AWidth, AHeight, APitch, AFormat, APalette,
         FArea, TPTCArea.Create(0, 0, width, height));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  begin
    area_ := TPTCArea.Create(0, 0, width, height);
    try
      Save(APixels, AWidth, AHeight, APitch, AFormat, APalette, FArea, area_);
    finally
      area_.Free;
    end;
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
end;

procedure TPTCSurface.Save(APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat;
                           APalette: IPTCPalette;
                           ASource, ADestination: IPTCArea);
var
  clipped_source: IPTCArea;
  clipped_destination: IPTCArea;
begin
  TPTCClipper.Clip(ASource, FClip,
                   clipped_source,
                   ADestination, TPTCArea.Create(0, 0, AWidth, AHeight),
                   clipped_destination);
  FCopy.Request(FFormat, AFormat);
  FCopy.Palette(FPalette, APalette);
  FCopy.Copy(FPixels, clipped_source.left, clipped_source.top,
             clipped_source.width, clipped_source.height, FPitch,
             APixels, clipped_destination.left, clipped_destination.top,
             clipped_destination.width, clipped_destination.height, APitch);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
var
  clipped_source: TPTCArea = nil;
  clipped_destination: TPTCArea = nil;
  area_: TPTCArea = nil;
begin
  try
    clipped_source := TPTCArea.Create;
    clipped_destination := TPTCArea.Create;
    area_ := TPTCArea.Create(0, 0, AWidth, AHeight);
    TPTCClipper.Clip(ASource, FClip, clipped_source, ADestination, area_,
                     clipped_destination);
    FCopy.Request(FFormat, AFormat);
    FCopy.Palette(FPalette, APalette);
    FCopy.Copy(FPixels, clipped_source.left, clipped_source.top,
               clipped_source.width, clipped_source.height, FPitch,
               APixels, clipped_destination.left, clipped_destination.top,
               clipped_destination.width, clipped_destination.height, APitch);
  finally
    clipped_source.Free;
    clipped_destination.Free;
    area_.Free;
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
end;

procedure TPTCSurface.Clear;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Color: IPTCColor;
=======
  Color: TPTCColor;
>>>>>>> graemeg/cpstrnew
=======
  Color: TPTCColor;
>>>>>>> graemeg/cpstrnew
=======
  Color: TPTCColor;
>>>>>>> graemeg/cpstrnew
=======
  Color: TPTCColor;
>>>>>>> origin/cpstrnew
begin
  if Format.Direct then
    Color := TPTCColor.Create(0, 0, 0, 0)
  else
    Color := TPTCColor.Create(0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  Clear(Color);
end;

procedure TPTCSurface.Clear(AColor: IPTCColor);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  try
    Clear(Color);
  finally
    Color.Free;
  end;
end;

procedure TPTCSurface.Clear(const AColor: TPTCColor);
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
begin
  Clear(AColor, FArea);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPTCSurface.Clear(AColor: IPTCColor; AArea: IPTCArea);
var
  clipped_area: IPTCArea;
begin
  clipped_area := TPTCClipper.Clip(AArea, FClip);
  FClear.Request(FFormat);
  FClear.Clear(FPixels, clipped_area.left, clipped_area.top,
               clipped_area.width, clipped_area.height, FPitch, AColor);
end;

procedure TPTCSurface.Palette(APalette: IPTCPalette);
begin
  FPalette.Load(APalette.Data^);
end;

function TPTCSurface.Palette: IPTCPalette;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TPTCSurface.Clear(const AColor: TPTCColor; const AArea: TPTCArea);
var
  clipped_area: TPTCArea;
begin
  clipped_area := TPTCClipper.clip(AArea, FClip);
  try
    FClear.Request(FFormat);
    FClear.Clear(FPixels, clipped_area.left, clipped_area.top,
                 clipped_area.width, clipped_area.height, FPitch, AColor);
  finally
    clipped_area.Free;
  end;
end;

procedure TPTCSurface.Palette(const APalette: TPTCPalette);
begin
  FPalette.Load(APalette.data^);
end;

function TPTCSurface.Palette: TPTCPalette;
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
begin
  Result := FPalette;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPTCSurface.Clip(AArea: IPTCArea);
begin
  FClip := TPTCClipper.Clip(AArea, FArea);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TPTCSurface.Clip(const AArea: TPTCArea);
var
  tmp: TPTCArea;
begin
  tmp := TPTCClipper.Clip(AArea, FArea);
  try
    FClip.Assign(tmp);
  finally
    tmp.Free;
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
end;

function TPTCSurface.GetWidth: Integer;
begin
  Result := FWidth;
end;

function TPTCSurface.GetHeight: Integer;
begin
  Result := FHeight;
end;

function TPTCSurface.GetPitch: Integer;
begin
  Result := FPitch;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCSurface.GetArea: IPTCArea;
=======
function TPTCSurface.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCSurface.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCSurface.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCSurface.GetArea: TPTCArea;
>>>>>>> origin/cpstrnew
begin
  Result := FArea;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCSurface.Clip: IPTCArea;
=======
function TPTCSurface.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCSurface.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCSurface.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCSurface.Clip: TPTCArea;
>>>>>>> origin/cpstrnew
begin
  Result := FClip;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCSurface.GetFormat: IPTCFormat;
=======
function TPTCSurface.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TPTCSurface.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TPTCSurface.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TPTCSurface.GetFormat: TPTCFormat;
>>>>>>> origin/cpstrnew
begin
  Result := FFormat;
end;

function TPTCSurface.Option(const AOption: string): Boolean;
begin
  Result := FCopy.Option(AOption);
end;
