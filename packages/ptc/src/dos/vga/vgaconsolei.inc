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

{$ASMMODE intel}

constructor TVGAConsole.Create;
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
  fmt1, fmt2, fmt3: TPTCFormat;

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
  m_title := '';
  m_information := '';
  m_default_width := DEFAULT_WIDTH;
  m_default_height := DEFAULT_HEIGHT;
  m_default_format := DEFAULT_FORMAT;

  m_primary := nil;

  m_area := TPTCArea.Create;
  m_clip := TPTCArea.Create;
  m_copy := TPTCCopy.Create;
  m_clear := TPTCClear.Create;
  m_palette := TPTCPalette.Create;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SetLength(m_modes, 3);
  m_modes[0] := TPTCMode.Create(320, 200, TPTCFormat.Create(8));
  m_modes[1] := TPTCMode.Create(320, 200, TPTCFormat.Create(8, $E0, $1C, $03));
  m_modes[2] := TPTCMode.Create(320, 200, TPTCFormat.Create(16, $F800, $7E0, $1F));

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  fmt1 := nil;
  fmt2 := nil;
  fmt3 := nil;
  try
    fmt1 := TPTCFormat.Create(8);
    fmt2 := TPTCFormat.Create(8, $E0, $1C, $03);
    fmt3 := TPTCFormat.Create(16, $F800, $7E0, $1F);
    m_modes[0] := TPTCMode.Create(320, 200, fmt1);
    m_modes[1] := TPTCMode.Create(320, 200, fmt2);
    m_modes[2] := TPTCMode.Create(320, 200, fmt3);
    m_modes[3] := TPTCMode.Create;
  finally
    fmt1.Free;
    fmt2.Free;
    fmt3.Free;
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
  m_faketype := FAKEMODE2A;

  configure('ptcpas.cfg');
end;

destructor TVGAConsole.Destroy;
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
  close;
  internal_clear_mode_list;
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

procedure TVGAConsole.Configure(const AFileName: string);
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
  m_palette.Free;
  m_clip.Free;
  m_area.Free;
  inherited Destroy;
end;

procedure TVGAConsole.Configure(const AFileName: String);

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
function TVGAConsole.option(const _option: string): Boolean;
=======
function TVGAConsole.option(const _option: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.option(const _option: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.option(const _option: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.option(const _option: String): Boolean;

>>>>>>> origin/cpstrnew
begin
  {...}
  if (System.Copy(_option, 1, 8) = 'FAKEMODE') and (Length(_option) = 10) and
    (_option[9] >= '1') and (_option[9] <= '3') and
    (_option[10] >= 'A') and (_option[10] <= 'C') then
  begin
    case _option[9] of
      '1': case _option[10] of
        'A': m_faketype := FAKEMODE1A;
        'B': m_faketype := FAKEMODE1B;
        'C': m_faketype := FAKEMODE1C;
      end;
      '2': case _option[10] of
        'A': m_faketype := FAKEMODE2A;
        'B': m_faketype := FAKEMODE2B;
        'C': m_faketype := FAKEMODE2C;
      end;
      '3': case _option[10] of
        'A': m_faketype := FAKEMODE3A;
        'B': m_faketype := FAKEMODE3B;
        'C': m_faketype := FAKEMODE3C;
      end;
    end;
    Result := True;
    exit;
  end;
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

procedure TVGAConsole.internal_clear_mode_list;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  SetLength(m_modes, 0);
end;

function TVGAConsole.modes: TPTCModeList;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  I: Integer;
  Done: Boolean;

begin
  I := 0;
  Done := False;
  repeat
    Done := not m_modes[I].valid;
    m_modes[I].Free;
    Inc(I);
  until Done;
end;

function TVGAConsole.modes: PPTCMode;

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
  Result := m_modes;
end;

procedure TVGAConsole.open(const _title: string; _pages: Integer); overload;
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
  open(_title, m_default_format, _pages);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVGAConsole.open(const _title: string; _format: IPTCFormat;
                           _pages: Integer); overload;
=======
procedure TVGAConsole.open(const _title: string; const _format: TPTCFormat;
                           _pages: Integer); overload;

>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.open(const _title: string; const _format: TPTCFormat;
                           _pages: Integer); overload;

>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.open(const _title: string; const _format: TPTCFormat;
                           _pages: Integer); overload;

>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.open(const _title: string; const _format: TPTCFormat;
                           _pages: Integer); overload;

>>>>>>> origin/cpstrnew
begin
  open(_title, m_default_width, m_default_height, _format, _pages);
end;

procedure TVGAConsole.open(const _title: string; _width, _height: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           _format: IPTCFormat; _pages: Integer); overload;
begin
  open(_title, TPTCMode.Create(_width, _height, _format), _pages);
end;

procedure TVGAConsole.open(const _title: string; _mode: IPTCMode;
                           _pages: Integer); overload;
var
  I: Integer;
  modetype: Integer;
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

procedure TVGAConsole.open(const _title: string; const _mode: TPTCMode;
                           _pages: Integer); overload;

var
  I: Integer;
  modetype: Integer;

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
  if _mode.format.indexed then
    modetype := INDEX8
  else
    if _mode.format.bits = 8 then
      modetype := RGB332
    else
      modetype := FAKEMODE;
  internal_pre_open_setup(_title);
  internal_open_fullscreen_start;
  internal_open_fullscreen(modetype);
  internal_open_fullscreen_finish(_pages);
  internal_post_open_setup;
end;

procedure TVGAConsole.close;
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
    { flush all key presses }
    while KeyPressed do ReadKey;
    internal_close;
    m_open := False;
  end;
end;

procedure TVGAConsole.flush;
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

procedure TVGAConsole.finish;
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVGAConsole.vga_load(data: Pointer); ASSembler; register;
asm
=======
procedure TVGAConsole.vga_load(data: Pointer); ASSembler; Register;

Asm
>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.vga_load(data: Pointer); ASSembler; Register;

Asm
>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.vga_load(data: Pointer); ASSembler; Register;

Asm
>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.vga_load(data: Pointer); ASSembler; Register;

Asm
>>>>>>> origin/cpstrnew
  push es
  mov esi, data
  mov ax, fs
  mov es, ax
  mov ecx, 64000/4
  mov edi, 0A0000h
  cld
  rep movsd
  pop es
end;

procedure TVGAConsole.update;
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
  case m_CurrentMode of
    0, 1: begin
      while (inportb($3DA) and 8) <> 0 Do;
      while (inportb($3DA) and 8) = 0 Do;
      vga_load(m_primary);
    end;
    2: fakemode_load(m_primary, True);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVGAConsole.update(_area: IPTCArea);
=======
procedure TVGAConsole.update(const _area: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.update(const _area: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.update(const _area: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.update(const _area: TPTCArea);

>>>>>>> origin/cpstrnew
begin
  update;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVGAConsole.copy(surface: IPTCSurface);
var
  pixels: Pointer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TVGAConsole.copy(surface: TPTCBaseSurface);

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

procedure TVGAConsole.copy(surface: IPTCSurface;
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

procedure TVGAConsole.copy(surface: TPTCBaseSurface;
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

function TVGAConsole.lock: Pointer;
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
  if m_locked then
    raise TPTCError.Create('console is already locked');

  m_locked := True;
  Result := m_primary;
end;

procedure TVGAConsole.unlock;
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

  m_locked := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVGAConsole.Load(const pixels: Pointer;
                           _width, _height, _pitch: Integer;
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
procedure TVGAConsole.load(const pixels: Pointer;
                           _width, _height, _pitch: Integer;
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
end;

procedure TVGAConsole.Load(const pixels: Pointer;
                           _width, _height, _pitch: Integer;
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
end;

procedure TVGAConsole.load(const pixels: Pointer;
                           _width, _height, _pitch: Integer;
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

procedure TVGAConsole.save(pixels: Pointer;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    end;
  end
  else
    Save(pixels, _width, _height, _pitch, _format, _palette, area, TPTCArea.Create(0, 0, width, height));
end;

procedure TVGAConsole.Save(pixels: Pointer;
                           _width, _height, _pitch: Integer;
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

    end;
  end
  else
  begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    try
      save(pixels, _width, _height, _pitch, _format, _palette, area, Area_);
    finally
      Area_.Free;
    end;
  end;
end;

procedure TVGAConsole.save(pixels: Pointer;
                           _width, _height, _pitch: Integer;
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

procedure TVGAConsole.clear;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  Color: IPTCColor;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

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

procedure TVGAConsole.clear(AColor: IPTCColor);
begin
  check_open;
  check_unlocked;
  Clear(AColor, TPTCArea.Create);
end;

procedure TVGAConsole.clear(color: IPTCColor;
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

procedure TVGAConsole.clear(const color: TPTCColor);

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

procedure TVGAConsole.clear(const color: TPTCColor;
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

procedure TVGAConsole.Palette(_palette: IPTCPalette);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  end;
end;

procedure TVGAConsole.Palette(const _palette: TPTCPalette);

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
  if format.indexed then
  begin
    m_palette.load(_palette.data);
    internal_SetPalette(_palette.data);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVGAConsole.Palette: IPTCPalette;
=======
function TVGAConsole.Palette: TPTCPalette;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.Palette: TPTCPalette;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.Palette: TPTCPalette;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.Palette: TPTCPalette;

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_palette;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVGAConsole.Clip(_area: IPTCArea);
begin
  check_open;
  m_clip := TPTCClipper.clip(_area, m_area);
end;

function TVGAConsole.GetWidth: Integer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TVGAConsole.Clip(const _area: TPTCArea);

var
  tmp: TPTCArea;

begin
  check_open;
  tmp := TPTCClipper.clip(_area, m_area);
  try
    m_clip.Assign(tmp);
  finally
    tmp.Free;
  end;
end;

function TVGAConsole.GetWidth: Integer;

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
  Result := m_width;
end;

function TVGAConsole.GetHeight: Integer;
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
  Result := m_height;
end;

function TVGAConsole.GetPitch: Integer;
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
  Result := m_pitch;
end;

function TVGAConsole.GetPages: Integer;
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
function TVGAConsole.GetArea: IPTCArea;
=======
function TVGAConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.GetArea: TPTCArea;

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_area;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVGAConsole.Clip: IPTCArea;
=======
function TVGAConsole.Clip: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.Clip: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.Clip: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.Clip: TPTCArea;

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_clip;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVGAConsole.GetFormat: IPTCFormat;
=======
function TVGAConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.GetFormat: TPTCFormat;

>>>>>>> origin/cpstrnew
begin
  check_open;
  Result := m_modes[m_CurrentMode].format;
end;

function TVGAConsole.GetName: string;
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
  Result := 'VGA';
end;

function TVGAConsole.GetTitle: string;
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

function TVGAConsole.GetInformation: string;
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
procedure TVGAConsole.internal_pre_open_setup(const _title: string);
=======
procedure TVGAConsole.internal_pre_open_setup(const _title: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.internal_pre_open_setup(const _title: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.internal_pre_open_setup(const _title: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TVGAConsole.internal_pre_open_setup(const _title: String);

>>>>>>> origin/cpstrnew
begin
  m_title := _title;
end;

procedure TVGAConsole.internal_open_fullscreen_start;
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

procedure TVGAConsole.internal_open_fullscreen(ModeType: Integer);
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
  tmp: TPTCArea;

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
  VGASetMode(320, 200, ModeType, m_faketype);
  case ModeType of
    INDEX8: begin
      m_CurrentMode := 0;
      m_pitch := 320;
    end;
    RGB332: begin
      m_CurrentMode := 1;
      m_pitch := 320;
    end;
    FAKEMODE: begin
      m_CurrentMode := 2;
      m_pitch := 640;
    end;
  end;
  m_width := 320;
  m_height := 200;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  m_area := TPTCArea.Create(0, 0, m_width, m_height);
  m_clip := m_area;
end;

procedure TVGAConsole.internal_open_fullscreen_finish(_pages: Integer);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  tmp := TPTCArea.Create(0, 0, m_width, m_height);
  try
    m_area.Assign(tmp);
    m_clip.Assign(tmp);
  finally
    tmp.Free;
  end;
end;

procedure TVGAConsole.internal_open_fullscreen_finish(_pages: Integer);

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
  FreeMemAndNil(m_primary);
  m_primary := GetMem(m_height * m_pitch);
  FillChar(m_primary^, m_height * m_pitch, 0);
end;

procedure TVGAConsole.internal_post_open_setup;
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
  FMouse := TDosMouse.Create(m_width, m_height);
  FEventQueue := TEventQueue.Create;

  { temporary platform dependent information fudge }
  m_information := 'dos version x.xx.x, VGA, 320x200 FAKEMODExx/8-bit indexed/8-bit RGB332';

  { set open flag }
  m_open := True;
end;

procedure TVGAConsole.internal_reset;
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
  FreeMemAndNil(m_primary);
  FreeAndNil(m_keyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);
end;

procedure TVGAConsole.internal_close;
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
  FreeMemAndNil(m_primary);
  FreeAndNil(m_keyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);

  RestoreTextMode;
end;

procedure TVGAConsole.internal_SetPalette(data: PUint32);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  i: Integer;
  c: DWord;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  i: Integer;
  c: DWord;

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
  outportb($3C8, 0);
  for i := 0 to 255 do
  begin
    c := (data^ shr 2) and $003F3F3F;
    outportb($3C9, Byte(c shr 16));
    outportb($3C9, Byte(c shr 8));
    outportb($3C9, Byte(c));
    Inc(data);
  end;
end;

procedure TVGAConsole.HandleEvents;
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
function TVGAConsole.NextEvent(out event: IPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean;
begin
  check_open;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TVGAConsole.NextEvent(var event: TPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean;

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
function TVGAConsole.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): IPTCEvent;
=======
function TVGAConsole.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======
function TVGAConsole.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

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

procedure TVGAConsole.check_open;
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

procedure TVGAConsole.check_unlocked;
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
