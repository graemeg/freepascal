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

constructor TTextFX2Console.Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  inherited Create;

  FOpen := False;
  FLocked := False;
  FTitle := '';
  FInformation := '';
  FDefaultWidth := DEFAULT_WIDTH;
  FDefaultHeight := DEFAULT_HEIGHT;
  FDefaultFormat := DEFAULT_FORMAT;
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
  inherited Create;

  m_open := False;
  m_locked := False;
  FillChar(m_modes, SizeOf(m_modes), 0);
  m_title := '';
  m_information := '';
  m_default_width := DEFAULT_WIDTH;
  m_default_height := DEFAULT_HEIGHT;
  m_default_format := DEFAULT_FORMAT;

  for I := Low(m_modes) to High(m_modes) do
    m_modes[I] := TPTCMode.Create;
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

  calcpal := @calcpal_colorbase;
  use_charset := @charset_b7asc;
  build_colormap(0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FCopy := TPTCCopy.Create;
  FClear := TPTCClear.Create;
  Configure('ptcpas.cfg');
end;

destructor TTextFX2Console.Destroy;
begin
  Close;
  F160x100buffer.Free;
  FPrimary.Free;

  FKeyboard.Free;
  FMouse.Free;
  FEventQueue.Free;
  FCopy.Free;
  FClear.Free;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  m_copy := TPTCCopy.Create;
  m_clear := TPTCClear.Create;
  configure('ptcpas.cfg');
end;

destructor TTextFX2Console.Destroy;

var
  I: Integer;

begin
  close;
  m_160x100buffer.Free;
  m_primary.Free;

  for I := Low(m_modes) to High(m_modes) do
    m_modes[I].Free;
  m_keyboard.Free;
  FMouse.Free;
  FEventQueue.Free;
  m_copy.Free;
  m_clear.Free;
  m_default_format.Free;
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
  dispose_colormap;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TTextFX2Console.Configure(const AFileName: string);
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
procedure TTextFX2Console.Configure(const AFileName: String);

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
function TTextFX2Console.Option(const AOption: string): Boolean;
begin
  {...}
  Result := True;
  if AOption = 'charset_b8ibm' then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TTextFX2Console.option(const _option: String): Boolean;

begin
  {...}
  Result := True;
  if _option = 'charset_b8ibm' then
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
    use_charset := @charset_b8ibm;
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AOption = 'charset_b7asc' then
=======
  if _option = 'charset_b7asc' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b7asc' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b7asc' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b7asc' then
>>>>>>> origin/cpstrnew
  begin
    use_charset := @charset_b7asc;
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AOption = 'charset_b7sml' then
=======
  if _option = 'charset_b7sml' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b7sml' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b7sml' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b7sml' then
>>>>>>> origin/cpstrnew
  begin
    use_charset := @charset_b7sml;
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AOption = 'charset_b8gry' then
=======
  if _option = 'charset_b8gry' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b8gry' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b8gry' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b8gry' then
>>>>>>> origin/cpstrnew
  begin
    use_charset := @charset_b8gry;
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AOption = 'charset_b7nws' then
=======
  if _option = 'charset_b7nws' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b7nws' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b7nws' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'charset_b7nws' then
>>>>>>> origin/cpstrnew
  begin
    use_charset := @charset_b7nws;
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AOption = 'calcpal_colorbase' then
=======
  if _option = 'calcpal_colorbase' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'calcpal_colorbase' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'calcpal_colorbase' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'calcpal_colorbase' then
>>>>>>> origin/cpstrnew
  begin
    calcpal := @calcpal_colorbase;
    build_colormap(0);
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AOption = 'calcpal_lightbase' then
=======
  if _option = 'calcpal_lightbase' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'calcpal_lightbase' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'calcpal_lightbase' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'calcpal_lightbase' then
>>>>>>> origin/cpstrnew
  begin
    calcpal := @calcpal_lightbase;
    build_colormap(0);
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AOption = 'calcpal_lightbase_g' then
=======
  if _option = 'calcpal_lightbase_g' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'calcpal_lightbase_g' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'calcpal_lightbase_g' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'calcpal_lightbase_g' then
>>>>>>> origin/cpstrnew
  begin
    calcpal := @calcpal_lightbase_g;
    build_colormap(0);
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AOption = 'enable logging' then
=======
  if _option = 'enable logging' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'enable logging' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'enable logging' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'enable logging' then
>>>>>>> origin/cpstrnew
  begin
    LOG_enabled := True;
    Result := True;
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AOption = 'disable logging' then
=======
  if _option = 'disable logging' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'disable logging' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'disable logging' then
>>>>>>> graemeg/cpstrnew
=======
  if _option = 'disable logging' then
>>>>>>> origin/cpstrnew
  begin
    LOG_enabled := False;
    Result := True;
    exit;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result := FCopy.Option(AOption);
end;

function TTextFX2Console.Modes: TPTCModeList;
begin
  Result := FModes;
end;

procedure TTextFX2Console.Open(const _title: string; _pages: Integer); overload;
begin
  Open(_title, FDefaultFormat, _pages);
end;

procedure TTextFX2Console.open(const _title: string; _format: IPTCFormat;
                               _pages: Integer); overload;
begin
  open(_title, FDefaultWidth, FDefaultHeight, _format, _pages);
end;

procedure TTextFX2Console.open(const _title: string; _width, _height: Integer;
                               _format: IPTCFormat; _pages: Integer); overload;
begin
  open(_title, TPTCMode.Create(_width, _height, _format), _pages);
end;

procedure TTextFX2Console.open(const _title: string; _mode: IPTCMode;
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
  Result := m_copy.option(_option);
end;

function TTextFX2Console.modes: PPTCMode;

begin
  Result := @m_modes;
end;

procedure TTextFX2Console.open(const _title: string; _pages: Integer); overload;

begin
  open(_title, m_default_format, _pages);
end;

procedure TTextFX2Console.open(const _title: string; const _format: TPTCFormat;
                               _pages: Integer); overload;

begin
  open(_title, m_default_width, m_default_height, _format, _pages);
end;

procedure TTextFX2Console.open(const _title: string; _width, _height: Integer;
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

procedure TTextFX2Console.open(const _title: string; const _mode: TPTCMode;
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TTextFX2Console.Close;
begin
  if FOpen then
  begin
    if FLocked then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TTextFX2Console.close;

begin
  if m_open then
  begin
    if m_locked then
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
      raise TPTCError.Create('console is still locked');
    {flush all key presses}
    while KeyPressed do ReadKey;
    internal_close;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FOpen := False;
=======
    m_open := False;
>>>>>>> graemeg/cpstrnew
=======
    m_open := False;
>>>>>>> graemeg/cpstrnew
=======
    m_open := False;
>>>>>>> graemeg/cpstrnew
=======
    m_open := False;
>>>>>>> origin/cpstrnew
  end;
end;

procedure TTextFX2Console.flush;
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

procedure TTextFX2Console.finish;
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

procedure TTextFX2Console.update;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  framebuffer: PInteger;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  framebuffer: PInteger;

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FPrimary.copy(F160x100buffer);
  framebuffer := F160x100buffer.Lock;
  vrc;
  dump_160x(0, 50, framebuffer);
  F160x100buffer.Unlock;
end;

procedure TTextFX2Console.update(_area: IPTCArea);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  m_primary.copy(m_160x100buffer);
  framebuffer := m_160x100buffer.lock;
  vrc;
  dump_160x(0, 50, framebuffer);
  m_160x100buffer.unlock;
end;

procedure TTextFX2Console.update(const _area: TPTCArea);

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
  update;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TTextFX2Console.copy(surface: IPTCSurface);
var
  pixels: Pointer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TTextFX2Console.copy(surface: TPTCBaseSurface);

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

procedure TTextFX2Console.copy(surface: IPTCSurface;
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

procedure TTextFX2Console.copy(surface: TPTCBaseSurface;
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
  end;
end;

function TTextFX2Console.Lock: Pointer;
var
  pixels: Pointer;
begin
  check_open;
  if FLocked then
    raise TPTCError.Create('console is already locked');
  pixels := FPrimary.lock;
  FLocked := True;
  Result := pixels;
end;

procedure TTextFX2Console.Unlock;
begin
  check_open;
  if not FLocked then
    raise TPTCError.Create('console is not locked');

  FPrimary.unlock;
  FLocked := False;
end;

procedure TTextFX2Console.Load(const pixels: Pointer;
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

  end;
end;

function TTextFX2Console.lock: Pointer;

var
  pixels: Pointer;

begin
  check_open;
  if m_locked then
    raise TPTCError.Create('console is already locked');
  pixels := m_primary.lock;
  m_locked := True;
  Result := pixels;
end;

procedure TTextFX2Console.unlock;

begin
  check_open;
  if not m_locked then
    raise TPTCError.Create('console is not locked');

  m_primary.unlock;
  m_locked := False;
end;

procedure TTextFX2Console.load(const pixels: Pointer;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        FCopy.request(_format, format);
        FCopy.palette(_palette, palette);
        FCopy.copy(pixels, 0, 0, _width, _height, _pitch, console_pixels, 0, 0,
=======
        m_copy.request(_format, format);
        m_copy.palette(_palette, palette);
        m_copy.copy(pixels, 0, 0, _width, _height, _pitch, console_pixels, 0, 0,
>>>>>>> graemeg/cpstrnew
=======
        m_copy.request(_format, format);
        m_copy.palette(_palette, palette);
        m_copy.copy(pixels, 0, 0, _width, _height, _pitch, console_pixels, 0, 0,
>>>>>>> graemeg/cpstrnew
=======
        m_copy.request(_format, format);
        m_copy.palette(_palette, palette);
        m_copy.copy(pixels, 0, 0, _width, _height, _pitch, console_pixels, 0, 0,
>>>>>>> graemeg/cpstrnew
=======
        m_copy.request(_format, format);
        m_copy.palette(_palette, palette);
        m_copy.copy(pixels, 0, 0, _width, _height, _pitch, console_pixels, 0, 0,
>>>>>>> origin/cpstrnew
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

procedure TTextFX2Console.load(const pixels: Pointer;
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
      FCopy.request(_format, format);
      FCopy.palette(_palette, palette);
      FCopy.copy(pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, _pitch,
                  console_pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, pitch);
    finally
      unlock;
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
      m_copy.request(_format, format);
      m_copy.palette(_palette, palette);
      m_copy.copy(pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, _pitch,
                  console_pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, pitch);
    finally
      unlock;
      clipped_source.Free;
      clipped_destination.Free;
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
  except
    on error:TPTCError do
      raise TPTCError.Create('failed to load pixels to console area', error);

  end;
end;

procedure TTextFX2Console.save(pixels: Pointer;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        FCopy.request(format, _format);
        FCopy.palette(palette, _palette);
        FCopy.copy(console_pixels, 0, 0, width, height, pitch, pixels, 0, 0,
=======
        m_copy.request(format, _format);
        m_copy.palette(palette, _palette);
        m_copy.copy(console_pixels, 0, 0, width, height, pitch, pixels, 0, 0,
>>>>>>> graemeg/cpstrnew
=======
        m_copy.request(format, _format);
        m_copy.palette(palette, _palette);
        m_copy.copy(console_pixels, 0, 0, width, height, pitch, pixels, 0, 0,
>>>>>>> graemeg/cpstrnew
=======
        m_copy.request(format, _format);
        m_copy.palette(palette, _palette);
        m_copy.copy(console_pixels, 0, 0, width, height, pitch, pixels, 0, 0,
>>>>>>> graemeg/cpstrnew
=======
        m_copy.request(format, _format);
        m_copy.palette(palette, _palette);
        m_copy.copy(console_pixels, 0, 0, width, height, pitch, pixels, 0, 0,
>>>>>>> origin/cpstrnew
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

procedure TTextFX2Console.save(pixels: Pointer;
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
      FCopy.request(format, _format);
      FCopy.palette(palette, _palette);
      FCopy.copy(console_pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, pitch,
                 pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, _pitch);
    finally
      unlock;
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
      m_copy.request(format, _format);
      m_copy.palette(palette, _palette);
      m_copy.copy(console_pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, pitch,
                  pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, _pitch);
    finally
      unlock;
      clipped_source.Free;
      clipped_destination.Free;
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

procedure TTextFX2Console.clear;
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

procedure TTextFX2Console.Clear(AColor: IPTCColor);
begin
  check_open;
  check_unlocked;
  Clear(AColor, TPTCArea.Create);
end;

procedure TTextFX2Console.clear(color: IPTCColor;
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

procedure TTextFX2Console.clear(const color: TPTCColor);

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

procedure TTextFX2Console.clear(const color: TPTCColor;
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
    pixels := lock;
    try
      clipped_area := TPTCClipper.clip(_area, clip);
      FClear.request(format);
      FClear.clear(pixels, clipped_area.left, clipped_area.right, clipped_area.width, clipped_area.height, pitch, color);
    finally
      unlock;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    clipped_area := nil;
    pixels := lock;
    try
      clipped_area := TPTCClipper.clip(_area, clip);
      m_clear.request(format);
      m_clear.clear(pixels, clipped_area.left, clipped_area.right, clipped_area.width, clipped_area.height, pitch, color);
    finally
      unlock;
      clipped_area.Free;
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
  except
    on error: TPTCError do
      raise TPTCError.Create('failed to clear console area', error);

  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TTextFX2Console.Palette(_palette: IPTCPalette);
begin
  check_open;
  FPrimary.palette(_palette);
end;

function TTextFX2Console.Palette: IPTCPalette;
begin
  check_open;
  Result := FPrimary.palette;
end;

procedure TTextFX2Console.Clip(_area: IPTCArea);
begin
  check_open;
  FPrimary.clip(_area);
end;

function TTextFX2Console.GetWidth: Integer;
begin
  check_open;
  Result := FPrimary.width;
end;

function TTextFX2Console.GetHeight: Integer;
begin
  check_open;
  Result := FPrimary.height;
end;

function TTextFX2Console.GetPitch: Integer;
begin
  check_open;
  Result := FPrimary.pitch;
end;

function TTextFX2Console.GetPages: Integer;
begin
  check_open;
  Result := 2;{FPrimary.pages;}
end;

function TTextFX2Console.GetArea: IPTCArea;
begin
  check_open;
  Result := FPrimary.area;
end;

function TTextFX2Console.Clip: IPTCArea;
begin
  check_open;
  Result := FPrimary.clip;
end;

function TTextFX2Console.GetFormat: IPTCFormat;
begin
  check_open;
  Result := FPrimary.format;
end;

function TTextFX2Console.GetName: string;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TTextFX2Console.Palette(const _palette: TPTCPalette);

begin
  check_open;
  m_primary.palette(_palette);
end;

function TTextFX2Console.Palette: TPTCPalette;

begin
  check_open;
  Result := m_primary.palette;
end;

procedure TTextFX2Console.Clip(const _area: TPTCArea);

begin
  check_open;
  m_primary.clip(_area);
end;

function TTextFX2Console.GetWidth: Integer;

begin
  check_open;
  Result := m_primary.width;
end;

function TTextFX2Console.GetHeight: Integer;

begin
  check_open;
  Result := m_primary.height;
end;

function TTextFX2Console.GetPitch: Integer;

begin
  check_open;
  Result := m_primary.pitch;
end;

function TTextFX2Console.GetPages: Integer;

begin
  check_open;
  Result := 2;{m_primary.pages;}
end;

function TTextFX2Console.GetArea: TPTCArea;

begin
  check_open;
  Result := m_primary.area;
end;

function TTextFX2Console.Clip: TPTCArea;

begin
  check_open;
  Result := m_primary.clip;
end;

function TTextFX2Console.GetFormat: TPTCFormat;

begin
  check_open;
  Result := m_primary.format;
end;

function TTextFX2Console.GetName: string;

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
  Result := 'TextFX2';
end;

function TTextFX2Console.GetTitle: string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result := FTitle;
end;

function TTextFX2Console.GetInformation: string;
begin
  Result := FInformation;
end;

procedure TTextFX2Console.internal_pre_open_setup(const _title: string);
begin
  FTitle := _title;
end;

procedure TTextFX2Console.internal_open_fullscreen_start;
begin
  F160x100buffer := TPTCSurface.Create(160, 100, TPTCFormat.Create(32, $0000FF, $00FF00, $FF0000));
  set80x50;
end;

procedure TTextFX2Console.internal_open_fullscreen(_width, _height: Integer; _format: IPTCFormat);
begin
  FPrimary := TPTCSurface.Create(_width, _height, _format);
end;

procedure TTextFX2Console.internal_open_fullscreen_finish(_pages: Integer);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

begin
  Result := m_title;
end;

function TTextFX2Console.GetInformation: string;

begin
  Result := m_information;
end;

procedure TTextFX2Console.internal_pre_open_setup(const _title: String);

begin
  m_title := _title;
end;

procedure TTextFX2Console.internal_open_fullscreen_start;

var
  f: TPTCFormat;

begin
  f := TPTCFormat.Create(32, $0000FF, $00FF00, $FF0000);
  try
    m_160x100buffer := TPTCSurface.Create(160, 100, f);
  finally
    f.Free;
  end;
  set80x50;
end;

procedure TTextFX2Console.internal_open_fullscreen(_width, _height: Integer; const _format: TPTCFormat);

begin
  m_primary := TPTCSurface.Create(_width, _height, _format);
end;

procedure TTextFX2Console.internal_open_fullscreen_finish(_pages: Integer);

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
end;

procedure TTextFX2Console.internal_post_open_setup;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  FreeAndNil(FKeyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);
  FKeyboard := TDosKeyboard.Create;
  FMouse := TDosMouse.Create(FPrimary.width, FPrimary.height);
  FEventQueue := TEventQueue.Create;

  { temporary platform dependent information fudge }
  FInformation := 'dos version x.xx.x, TextFX2, ...';

  { set open flag }
  FOpen := True;
end;

procedure TTextFX2Console.internal_reset;
begin
  FreeAndNil(FPrimary);
  FreeAndNil(FKeyboard);
=======
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
  m_information := 'dos version x.xx.x, TextFX2, ...';

  { set open flag }
  m_open := True;
end;

procedure TTextFX2Console.internal_reset;

begin
  FreeAndNil(m_primary);
  FreeAndNil(m_keyboard);
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
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);
end;

procedure TTextFX2Console.internal_close;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  FreeAndNil(FPrimary);
  FreeAndNil(F160x100buffer);
  FreeAndNil(FKeyboard);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

begin
  FreeAndNil(m_primary);
  FreeAndNil(m_160x100buffer);
  FreeAndNil(m_keyboard);
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
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);
  set80x25;
end;

procedure TTextFX2Console.HandleEvents;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  FKeyboard.GetPendingEvents(FEventQueue);
  FMouse.GetPendingEvents(FEventQueue);
end;

function TTextFX2Console.NextEvent(out event: IPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean;
begin
  check_open;

=======
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

function TTextFX2Console.NextEvent(var event: TPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean;

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
function TTextFX2Console.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): IPTCEvent;
=======
function TTextFX2Console.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======
function TTextFX2Console.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======
function TTextFX2Console.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======
function TTextFX2Console.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;

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

procedure TTextFX2Console.check_open;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  if not FOpen then
=======

begin
  if not m_open then
>>>>>>> graemeg/cpstrnew
=======

begin
  if not m_open then
>>>>>>> graemeg/cpstrnew
=======

begin
  if not m_open then
>>>>>>> graemeg/cpstrnew
=======

begin
  if not m_open then
>>>>>>> origin/cpstrnew
    raise TPTCError.Create('console is not open');
end;

procedure TTextFX2Console.check_unlocked;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  if FLocked then
=======

begin
  if m_locked then
>>>>>>> graemeg/cpstrnew
=======

begin
  if m_locked then
>>>>>>> graemeg/cpstrnew
=======

begin
  if m_locked then
>>>>>>> graemeg/cpstrnew
=======

begin
  if m_locked then
>>>>>>> origin/cpstrnew
    raise TPTCError.Create('console is not unlocked');
end;
