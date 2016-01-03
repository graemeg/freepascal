{
    This file is part of the PTCPas framebuffer library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2011 Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew

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

{$MACRO ON}

{$DEFINE DEFAULT_WIDTH:=320}
{$DEFINE DEFAULT_HEIGHT:=200}
{$DEFINE DEFAULT_FORMAT:=TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF)}

constructor TCGAConsole.Create;
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

var
  I: Integer;

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
  inherited Create;

  m_open := False;
  m_locked := False;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  FillChar(m_modes, SizeOf(m_modes), 0);
>>>>>>> graemeg/cpstrnew
=======
  FillChar(m_modes, SizeOf(m_modes), 0);
>>>>>>> graemeg/cpstrnew
=======
  FillChar(m_modes, SizeOf(m_modes), 0);
>>>>>>> graemeg/cpstrnew
=======
  FillChar(m_modes, SizeOf(m_modes), 0);
>>>>>>> origin/cpstrnew
  m_title := '';
  m_information := '';
  m_default_width := DEFAULT_WIDTH;
  m_default_height := DEFAULT_HEIGHT;
  m_default_format := DEFAULT_FORMAT;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  for I := 0 to 255 do
    m_modes[I] := TPTCMode.Create;

>>>>>>> graemeg/cpstrnew
=======
  for I := 0 to 255 do
    m_modes[I] := TPTCMode.Create;

>>>>>>> graemeg/cpstrnew
=======
  for I := 0 to 255 do
    m_modes[I] := TPTCMode.Create;

>>>>>>> graemeg/cpstrnew
=======
  for I := 0 to 255 do
    m_modes[I] := TPTCMode.Create;

>>>>>>> origin/cpstrnew
  m_copy := TPTCCopy.Create;
  m_clear := TPTCClear.Create;
  Configure('ptcpas.cfg');
end;

destructor TCGAConsole.Destroy;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  close;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  I: Integer;

begin
  close;
  for I := 0 to 255 do
    m_modes[I].Free;
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
  m_keyboard.Free;
  FMouse.Free;
  FEventQueue.Free;
  m_copy.Free;
  m_clear.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  inherited Destroy;
end;

procedure TCGAConsole.Configure(const AFileName: string);
var
  F: TextFile;
  S: string;
begin
  AssignFile(F, AFileName);
  {$push}{$I-}
  Reset(F);
  {$pop}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  m_default_format.Free;
  inherited Destroy;
end;

procedure TCGAConsole.Configure(const AFileName: String);

var
  F: Text;
  S: string;

begin
  AssignFile(F, AFileName);
  {$I-}
  Reset(F);
  {$I+}
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
  if IOResult <> 0 then
    exit;
  while not EoF(F) do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    {$push}{$I-}
    Readln(F, S);
    {$pop}
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> graemeg/cpstrnew
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> graemeg/cpstrnew
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> graemeg/cpstrnew
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> origin/cpstrnew
    if IOResult <> 0 then
      Break;
    Option(S);
  end;
  CloseFile(F);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCGAConsole.option(const _option: string): Boolean;
=======
function TCGAConsole.option(const _option: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.option(const _option: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.option(const _option: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.option(const _option: String): Boolean;

>>>>>>> origin/cpstrnew
begin
  {...}
  if _option = 'enable logging' then
  begin
    LOG_enabled := True;
    Result := True;
    exit;
  end;
  if _option = 'disable logging' then
  begin
    LOG_enabled := False;
    Result := True;
    exit;
  end;

  Result := m_copy.option(_option);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCGAConsole.modes: TPTCModeList;
begin
  Result := m_modes;
end;

procedure TCGAConsole.Open(const _title: string; _pages: Integer); overload;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TCGAConsole.modes: PPTCMode;

begin
  Result := @m_modes;
end;

procedure TCGAConsole.open(const _title: string; _pages: Integer); overload;

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
  open(_title, m_default_format, _pages);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCGAConsole.open(const _title: string; _format: IPTCFormat;
                           _pages: Integer); overload;
=======
procedure TCGAConsole.open(const _title: string; const _format: TPTCFormat;
                           _pages: Integer); overload;

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.open(const _title: string; const _format: TPTCFormat;
                           _pages: Integer); overload;

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.open(const _title: string; const _format: TPTCFormat;
                           _pages: Integer); overload;

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.open(const _title: string; const _format: TPTCFormat;
                           _pages: Integer); overload;

>>>>>>> origin/cpstrnew
begin
  open(_title, m_default_width, m_default_height, _format, _pages);
end;

procedure TCGAConsole.open(const _title: string; _width, _height: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           _format: IPTCFormat; _pages: Integer); overload;
begin
  open(_title, TPTCMode.Create(_width, _height, _format), _pages);
end;

procedure TCGAConsole.open(const _title: string; _mode: IPTCMode;
                           _pages: Integer); overload;
var
  _width, _height: Integer;
  _format: IPTCFormat;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                           const _format: TPTCFormat; _pages: Integer); overload;

var
  m: TPTCMode;

begin
  m := TPTCMode.Create(_width, _height, _format);
  try
    open(_title, m, _pages);
  finally
    m.Free;
  end;
end;

procedure TCGAConsole.open(const _title: string; const _mode: TPTCMode;
                           _pages: Integer); overload;

var
  _width, _height: Integer;
  _format: TPTCFormat;

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
  if not _mode.valid then
    raise TPTCError.Create('invalid mode');

  _width := _mode.width;
  _height := _mode.height;
  _format := _mode.format;

  internal_pre_open_setup(_title);
  internal_open_fullscreen_start;
  internal_open_fullscreen(_width, _height, _format);
  internal_open_fullscreen_finish(_pages);
  internal_post_open_setup;
end;

procedure TCGAConsole.close;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  if m_open then
  begin
    if m_locked then
      raise TPTCError.Create('console is still locked');
    {flush all key presses}
    while KeyPressed do ReadKey;
    internal_close;
    m_open := False;
  end;
end;

procedure TCGAConsole.flush;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  check_open;
  check_unlocked;
end;

procedure TCGAConsole.finish;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  check_open;
  check_unlocked;
end;

procedure TCGAConsole.update;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  framebuffer: PByte;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  framebuffer: PByte;

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
  check_open;
  check_unlocked;
  framebuffer := m_primary.lock;
  try
{    vrc;}
    CGADump(framebuffer);
  finally
    m_primary.unlock;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCGAConsole.update(_area: IPTCArea);
=======
procedure TCGAConsole.update(const _area: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.update(const _area: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.update(const _area: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.update(const _area: TPTCArea);

>>>>>>> origin/cpstrnew
begin
  update;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCGAConsole.Copy(surface: IPTCSurface);
var
  pixels: Pointer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TCGAConsole.copy(surface: TPTCBaseSurface);

var
  pixels: Pointer;

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
  check_open;
  check_unlocked;
  pixels := lock;
  try
    try
      surface.load(pixels, width, height, pitch, format, palette);
    finally
      unlock;
    end;
  except
    on error: TPTCError do
      raise TPTCError.Create('failed to copy console to surface', error);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  end;
end;

procedure TCGAConsole.Copy(surface: IPTCSurface;
                           source, destination: IPTCArea);
var
  pixels: Pointer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  end;
end;

procedure TCGAConsole.copy(surface: TPTCBaseSurface;
                           const source, destination: TPTCArea);

var
  pixels: Pointer;

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
  check_open;
  check_unlocked;
  pixels := lock;
  try
    try
      surface.load(pixels, width, height, pitch, format, palette, source, destination);
    finally
      unlock;
    end;
  except
    on error: TPTCError do
      raise TPTCError.Create('failed to copy console to surface', error);

  end;
end;

function TCGAConsole.lock: Pointer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  pixels: Pointer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  pixels: Pointer;

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
  check_open;
  if m_locked then
    raise TPTCError.Create('console is already locked');

  pixels := m_primary.lock;
  m_locked := True;
  Result := pixels;
end;

procedure TCGAConsole.unlock;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  check_open;
  if not m_locked then
    raise TPTCError.Create('console is not locked');

  m_primary.unlock;
  m_locked := False;
end;

procedure TCGAConsole.load(const pixels: Pointer;
                           _width, _height, _pitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           _format: IPTCFormat;
                           _palette: IPTCPalette);
var
  console_pixels: Pointer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                           const _format: TPTCFormat;
                           const _palette: TPTCPalette);
var
  Area_: TPTCArea;
  console_pixels: Pointer;

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
  check_open;
  check_unlocked;
  if clip.Equals(area) then
  begin
    try
      console_pixels := lock;
      try
        m_copy.request(_format, format);
        m_copy.palette(_palette, palette);
        m_copy.copy(pixels, 0, 0, _width, _height, _pitch, console_pixels, 0, 0,
                    width, height, pitch);
      finally
        unlock;
      end;
    except
      on error: TPTCError do
        raise TPTCError.Create('failed to load pixels to console', error);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    end;
  end
  else
    Load(pixels, _width, _height, _pitch, _format, _palette, TPTCArea.Create(0, 0, width, height), area);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

    end;
  end
  else
  begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    try
      load(pixels, _width, _height, _pitch, _format, _palette, Area_, area);
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

procedure TCGAConsole.load(const pixels: Pointer;
                           _width, _height, _pitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           _format: IPTCFormat;
                           _palette: IPTCPalette;
                           source, destination: IPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: IPTCArea;
begin
  check_open;
  check_unlocked;
  try
    console_pixels := lock;
    try
      TPTCClipper.clip(source, TPTCArea.Create(0, 0, _width, _height), clipped_source, destination, clip, clipped_destination);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                           const _format: TPTCFormat;
                           const _palette: TPTCPalette;
                           const source, destination: TPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: TPTCArea;
  tmp: TPTCArea;

begin
  check_open;
  check_unlocked;
  clipped_source := nil;
  clipped_destination := nil;
  try
    console_pixels := lock;
    try
      clipped_source := TPTCArea.Create;
      clipped_destination := TPTCArea.Create;
      tmp := TPTCArea.Create(0, 0, _width, _height);
      try
        TPTCClipper.clip(source, tmp, clipped_source, destination, clip, clipped_destination);
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
      m_copy.request(_format, format);
      m_copy.palette(_palette, palette);
      m_copy.copy(pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, _pitch,
                  console_pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, pitch);
    finally
      unlock;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
      clipped_source.Free;
      clipped_destination.Free;
>>>>>>> graemeg/cpstrnew
=======
      clipped_source.Free;
      clipped_destination.Free;
>>>>>>> graemeg/cpstrnew
=======
      clipped_source.Free;
      clipped_destination.Free;
>>>>>>> graemeg/cpstrnew
=======
      clipped_source.Free;
      clipped_destination.Free;
>>>>>>> origin/cpstrnew
    end;
  except
    on error:TPTCError do
      raise TPTCError.Create('failed to load pixels to console area', error);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
  end;
end;

procedure TCGAConsole.save(pixels: Pointer;
                           _width, _height, _pitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           _format: IPTCFormat;
                           _palette: IPTCPalette);
var
  console_pixels: Pointer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                           const _format: TPTCFormat;
                           const _palette: TPTCPalette);
var
  Area_: TPTCArea;
  console_pixels: Pointer;

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
  check_open;
  check_unlocked;
  if clip.Equals(area) then
  begin
    try
      console_pixels := lock;
      try
        m_copy.request(format, _format);
        m_copy.palette(palette, _palette);
        m_copy.copy(console_pixels, 0, 0, width, height, pitch, pixels, 0, 0,
                    _width, _height, _pitch);
      finally
        unlock;
      end;
    except
      on error: TPTCError do
        raise TPTCError.Create('failed to save console pixels', error);

    end;
  end
  else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Save(pixels, _width, _height, _pitch, _format, _palette, area, TPTCArea.Create(0, 0, width, height));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    try
      save(pixels, _width, _height, _pitch, _format, _palette, area, Area_);
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

procedure TCGAConsole.save(pixels: Pointer;
                           _width, _height, _pitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           _format: IPTCFormat;
                           _palette: IPTCPalette;
                           source, destination: IPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: IPTCArea;
begin
  check_open;
  check_unlocked;
  try
    console_pixels := lock;
    try
      TPTCClipper.clip(source, clip, clipped_source, destination, TPTCArea.Create(0, 0, _width, _height), clipped_destination);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                           const _format: TPTCFormat;
                           const _palette: TPTCPalette;
                           const source, destination: TPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: TPTCArea;
  tmp: TPTCArea;

begin
  check_open;
  check_unlocked;
  clipped_source := nil;
  clipped_destination := nil;
  try
    console_pixels := lock;
    try
      clipped_source := TPTCArea.Create;
      clipped_destination := TPTCArea.Create;
      tmp := TPTCArea.Create(0, 0, _width, _height);
      try
        TPTCClipper.clip(source, clip, clipped_source, destination, tmp, clipped_destination);
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
      m_copy.request(format, _format);
      m_copy.palette(palette, _palette);
      m_copy.copy(console_pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, pitch,
                  pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, _pitch);
    finally
      unlock;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
      clipped_source.Free;
      clipped_destination.Free;
>>>>>>> graemeg/cpstrnew
=======
      clipped_source.Free;
      clipped_destination.Free;
>>>>>>> graemeg/cpstrnew
=======
      clipped_source.Free;
      clipped_destination.Free;
>>>>>>> graemeg/cpstrnew
=======
      clipped_source.Free;
      clipped_destination.Free;
>>>>>>> origin/cpstrnew
    end;
  except
    on error:TPTCError do
      raise TPTCError.Create('failed to save console area pixels', error);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  end;
end;

procedure TCGAConsole.Clear;
var
  Color: IPTCColor;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  end;
end;

procedure TCGAConsole.clear;

var
  tmp: TPTCColor;

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
  check_open;
  check_unlocked;
  if format.direct then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Color := TPTCColor.Create(0, 0, 0, 0)
  else
    Color := TPTCColor.Create(0);
  Clear(Color);
end;

procedure TCGAConsole.Clear(color: IPTCColor);
begin
  check_open;
  check_unlocked;
  Clear(color, TPTCArea.Create);
end;

procedure TCGAConsole.clear(color: IPTCColor;
                            _area: IPTCArea);
var
  pixels: Pointer;
  clipped_area: IPTCArea;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    tmp := TPTCColor.Create(0, 0, 0, 0)
  else
    tmp := TPTCColor.Create(0);
  try
    clear(tmp);
  finally
    tmp.Free;
  end;
end;

procedure TCGAConsole.clear(const color: TPTCColor);

var
  tmp: TPTCArea;

begin
  check_open;
  check_unlocked;
  tmp := TPTCArea.Create;
  try
    clear(color, tmp);
  finally
    tmp.Free;
  end;
end;

procedure TCGAConsole.clear(const color: TPTCColor;
                            const _area: TPTCArea);

var
  pixels: Pointer;
  clipped_area: TPTCArea;

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
  check_open;
  check_unlocked;
  try
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    clipped_area := nil;
>>>>>>> graemeg/cpstrnew
=======
    clipped_area := nil;
>>>>>>> graemeg/cpstrnew
=======
    clipped_area := nil;
>>>>>>> graemeg/cpstrnew
=======
    clipped_area := nil;
>>>>>>> origin/cpstrnew
    pixels := lock;
    try
      clipped_area := TPTCClipper.clip(_area, clip);
      m_clear.request(format);
      m_clear.clear(pixels, clipped_area.left, clipped_area.right, clipped_area.width, clipped_area.height, pitch, color);
    finally
      unlock;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
      clipped_area.Free;
>>>>>>> graemeg/cpstrnew
=======
      clipped_area.Free;
>>>>>>> graemeg/cpstrnew
=======
      clipped_area.Free;
>>>>>>> graemeg/cpstrnew
=======
      clipped_area.Free;
>>>>>>> origin/cpstrnew
    end;
  except
    on error: TPTCError do
      raise TPTCError.Create('failed to clear console area', error);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  end;
end;

procedure TCGAConsole.Palette(_palette: IPTCPalette);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  end;
end;

procedure TCGAConsole.Palette(const _palette: TPTCPalette);

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
  check_open;
  m_primary.palette(_palette);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCGAConsole.Palette: IPTCPalette;
=======
function TCGAConsole.Palette: TPTCPalette;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.Palette: TPTCPalette;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.Palette: TPTCPalette;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.Palette: TPTCPalette;

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_primary.palette;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCGAConsole.Clip(_area: IPTCArea);
=======
procedure TCGAConsole.Clip(const _area: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.Clip(const _area: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.Clip(const _area: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.Clip(const _area: TPTCArea);

>>>>>>> origin/cpstrnew
begin
  check_open;
  m_primary.clip(_area);
end;

function TCGAConsole.GetWidth: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_primary.width;
end;

function TCGAConsole.GetHeight: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_primary.height;
end;

function TCGAConsole.GetPitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_primary.pitch;
end;

function TCGAConsole.GetPages: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := 2;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCGAConsole.GetArea: IPTCArea;
=======
function TCGAConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.GetArea: TPTCArea;

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_primary.area;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCGAConsole.Clip: IPTCArea;
=======
function TCGAConsole.Clip: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.Clip: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.Clip: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.Clip: TPTCArea;

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_primary.clip;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCGAConsole.GetFormat: IPTCFormat;
=======
function TCGAConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.GetFormat: TPTCFormat;

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_primary.format;
end;

function TCGAConsole.GetName: string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  Result := 'CGA';
end;

function TCGAConsole.GetTitle: string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  Result := m_title;
end;

function TCGAConsole.GetInformation: string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  Result := m_information;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCGAConsole.internal_pre_open_setup(const _title: string);
=======
procedure TCGAConsole.internal_pre_open_setup(const _title: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.internal_pre_open_setup(const _title: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.internal_pre_open_setup(const _title: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.internal_pre_open_setup(const _title: String);

>>>>>>> origin/cpstrnew
begin
  m_title := _title;
end;

procedure TCGAConsole.internal_open_fullscreen_start;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  CGAPrecalc;

  m_primary := TPTCSurface.Create(320, 200, TPTCFormat.Create(32, $FF0000, $00FF00, $0000FF));
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  f: TPTCFormat;

begin
  CGAPrecalc;

  f := TPTCFormat.Create(32, $FF0000, $00FF00, $0000FF);
  try
    m_primary := TPTCSurface.Create(320, 200, f);
  finally
    f.Free;
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

  CGA320;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TCGAConsole.internal_open_fullscreen(_width, _height: Integer; const _format: IPTCFormat);
=======
procedure TCGAConsole.internal_open_fullscreen(_width, _height: Integer; const _format: TPTCFormat);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.internal_open_fullscreen(_width, _height: Integer; const _format: TPTCFormat);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.internal_open_fullscreen(_width, _height: Integer; const _format: TPTCFormat);

>>>>>>> graemeg/cpstrnew
=======
procedure TCGAConsole.internal_open_fullscreen(_width, _height: Integer; const _format: TPTCFormat);

>>>>>>> origin/cpstrnew
begin
end;

procedure TCGAConsole.internal_open_fullscreen_finish(_pages: Integer);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
end;

procedure TCGAConsole.internal_post_open_setup;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  FreeAndNil(m_keyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);
  m_keyboard := TDosKeyboard.Create;
  FMouse := TDosMouse.Create(m_primary.width, m_primary.height);
  FEventQueue := TEventQueue.Create;

  { temporary platform dependent information fudge }
  m_information := 'dos version x.xx.x, CGA, 320x200 - 4 colors';

  { set open flag }
  m_open := True;
end;

procedure TCGAConsole.internal_reset;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  FreeAndNil(m_primary);
  FreeAndNil(m_keyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);
end;

procedure TCGAConsole.internal_close;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  FreeAndNil(m_primary);
  FreeAndNil(m_keyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);

  CGAText;
end;

procedure TCGAConsole.HandleEvents;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  m_keyboard.GetPendingEvents(FEventQueue);
  FMouse.GetPendingEvents(FEventQueue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCGAConsole.NextEvent(out event: IPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean;
begin
  check_open;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TCGAConsole.NextEvent(var event: TPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean;

begin
  check_open;

  FreeAndNil(event);
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
  repeat
    { get events }
    HandleEvents;

    { try to find an event that matches the EventMask }
    event := FEventQueue.NextEvent(EventMask);
  until (not Wait) or (event <> Nil);
  Result := event <> nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TCGAConsole.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): IPTCEvent;
=======
function TCGAConsole.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======
function TCGAConsole.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

>>>>>>> origin/cpstrnew
begin
  check_open;

  repeat
    { get events }
    HandleEvents;

    { try to find an event that matches the EventMask }
    Result := FEventQueue.PeekEvent(EventMask);
  until (not Wait) or (Result <> Nil);
end;

procedure TCGAConsole.check_open;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  if not m_open then
    raise TPTCError.Create('console is not open');
end;

procedure TCGAConsole.check_unlocked;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  if m_locked then
    raise TPTCError.Create('console is not unlocked');
end;
