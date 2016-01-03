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
  TPTCPalette = class(TInterfacedObject, IPTCPalette)
  private
    FLocked: Boolean;
    FHandle: THermesPaletteHandle;
  public
    constructor Create;
    constructor Create(const AData: array of Uint32);
    constructor Create(APalette: IPTCPalette);
    destructor Destroy; override;
//    procedure Assign(const APalette: TPTCPalette);
//    function Equals(const APalette: TPTCPalette): Boolean;
    function Lock: PUint32;
    procedure Unlock;
    procedure Load(const AData: array of Uint32);
    procedure Load(AData: Pointer);
    procedure Save(var AData: array of Uint32);
    procedure Save(AData: Pointer);
    function Data: PUint32;
    function GetHermesPaletteHandle: THermesPaletteHandle;
  end;

class function TPTCPaletteFactory.CreateNew: IPTCPalette;
begin
  Result := TPTCPalette.Create;
end;

class function TPTCPaletteFactory.CreateNew(const AData: array of Uint32): IPTCPalette;
begin
  Result := TPTCPalette.Create(AData);
end;

class function TPTCPaletteFactory.CreateNew(APalette: IPTCPalette): IPTCPalette;
begin
  Result := TPTCPalette.Create(APalette);
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
constructor TPTCPalette.Create;
var
  zero: array [0..255] of Uint32;
begin
  FLocked := False;
  if not Hermes_Init then
    raise TPTCError.Create('could not initialize hermes');
  FHandle := Hermes_PaletteInstance;
  if FHandle = nil then
    raise TPTCError.Create('could not create hermes palette instance');
  FillChar(zero, SizeOf(zero), 0);
  Load(zero);
end;

constructor TPTCPalette.Create(const AData: array of Uint32);
begin
  FLocked := False;
  if not Hermes_Init then
    raise TPTCError.Create('could not initialize hermes');
  FHandle := Hermes_PaletteInstance;
  if FHandle = nil then
    raise TPTCError.Create('could not create hermes palette instance');
  Load(AData);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
constructor TPTCPalette.Create(APalette: IPTCPalette);
=======
constructor TPTCPalette.Create(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
constructor TPTCPalette.Create(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
constructor TPTCPalette.Create(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
constructor TPTCPalette.Create(const APalette: TPTCPalette);
>>>>>>> origin/cpstrnew
begin
  FLocked := False;
  if not Hermes_Init then
    raise TPTCError.Create('could not initialize hermes');
  FHandle := Hermes_PaletteInstance;
  if FHandle = nil then
    raise TPTCError.Create('could not create hermes palette instance');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Hermes_PaletteSet(FHandle, Hermes_PaletteGet(APalette.GetHermesPaletteHandle));
=======
  Assign(APalette);
>>>>>>> graemeg/cpstrnew
=======
  Assign(APalette);
>>>>>>> graemeg/cpstrnew
=======
  Assign(APalette);
>>>>>>> graemeg/cpstrnew
=======
  Assign(APalette);
>>>>>>> origin/cpstrnew
end;

destructor TPTCPalette.Destroy;
begin
  if FLocked then
    raise TPTCError.Create('palette is still locked');
  Hermes_PaletteReturn(FHandle);
  Hermes_Done;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{procedure TPTCPalette.Assign(const APalette: TPTCPalette);
=======
procedure TPTCPalette.Assign(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCPalette.Assign(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCPalette.Assign(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCPalette.Assign(const APalette: TPTCPalette);
>>>>>>> origin/cpstrnew
begin
  if Self = APalette then
    exit;

  Hermes_PaletteSet(FHandle, Hermes_PaletteGet(APalette.FHandle));
end;

function TPTCPalette.Equals(const APalette: TPTCPalette): Boolean;
begin
  Equals := CompareDWord(Hermes_PaletteGet(FHandle)^, Hermes_PaletteGet(APalette.FHandle)^, 1024 div 4) = 0;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
}
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
function TPTCPalette.Lock: PUint32;
begin
  if FLocked then
    raise TPTCError.Create('palette is already locked');
  FLocked := True;
  Result := Hermes_PaletteGet(FHandle);
end;

procedure TPTCPalette.Unlock;
begin
  if not FLocked then
    raise TPTCError.Create('palette is not locked');
  FLocked := False;
end;

procedure TPTCPalette.Load(const AData: array of Uint32);
begin
  Hermes_PaletteSet(FHandle, @AData);
end;

procedure TPTCPalette.Load(AData: Pointer);
begin
  Hermes_PaletteSet(FHandle, AData);
end;

procedure TPTCPalette.Save(var AData: array of Uint32);
begin
  Move(Hermes_PaletteGet(FHandle)^, AData, 1024);
end;

procedure TPTCPalette.Save(AData: Pointer);
begin
  Move(Hermes_PaletteGet(FHandle)^, AData^, 1024);
end;

function TPTCPalette.Data: PUint32;
begin
  Result := Hermes_PaletteGet(FHandle);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TPTCPalette.GetHermesPaletteHandle: THermesPaletteHandle;
begin
  Result := FHandle;
end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
