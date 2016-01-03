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

{$MACRO ON}

{$DEFINE DEFAULT_WIDTH:=320}
{$DEFINE DEFAULT_HEIGHT:=200}
{$DEFINE DEFAULT_FORMAT:=TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF)}
{$IFDEF DEBUG}
{$DEFINE DEFAULT_OUTPUT:=WINDOWED}
{$ELSE}
{$DEFINE DEFAULT_OUTPUT:=DEFAULT}
{$ENDIF}
{$IFNDEF DEBUG}
{$DEFINE CHECK_OPEN:=//}
{$DEFINE CHECK_LOCK:=//}
{$ENDIF}

Const
  {Output}
  DEFAULT = 0;
  WINDOWED = 1;
  FULLSCREEN = 2;

  {Window}
  RESIZABLE = 0;
  FIXED = 1;

  {Primary}
  DIRECT = 0;
  SECONDARY = 1;

  {Nearest}
  NEAREST_DEFAULT = 0;
  NEAREST_CENTERING = 1;
  NEAREST_STRETCHING = 2;

  {Cursor}
  CURSOR_DEFAULT = 0;
  CURSOR_SHOW = 1;
  CURSOR_HIDE = 2;

Function PChar2String(Q : PChar) : String;

Var
  I : Integer;
  S : String;

Begin
  S := '';
  I := 0;
  While Q[I] <> #0 Do
  Begin
    S := S + Q[I];
    Inc(I);
  End;
  PChar2String := S;
End;

Constructor TDirectXConsole.Create;

Begin
  { clear objects }
  m_default_format := Nil;
  m_hook := Nil;
  m_window := Nil;
  m_keyboard := Nil;
  m_copy := Nil;
  m_library := Nil;
  m_display := Nil;
  m_primary := Nil;
  m_copy := TPTCCopy.Create;
  m_library := TDirectXLibrary.Create;
  m_display := TDirectXDisplay.Create;
  m_primary := TDirectXPrimary.Create;

  { defaults }
  m_open := False;
  m_locked := False;
  m_cursor := True;

  { clear strings }
{  m_title[0] := #0;}
  m_title := '';

  { default option data }
  m_frequency := 0;
  m_default_width := DEFAULT_WIDTH;
  m_default_height := DEFAULT_HEIGHT;
  m_default_format := DEFAULT_FORMAT;
  m_center_window := False;
  m_synchronized_update := True;
  m_output_mode := DEFAULT_OUTPUT;
  m_window_mode := RESIZABLE;
  m_primary_mode_windowed := SECONDARY;
  m_primary_mode_fullscreen := DIRECT;
  m_nearest_mode := NEAREST_DEFAULT;
  m_cursor_mode := CURSOR_DEFAULT;

  { configure console }
  configure('ptc.cfg');

  { setup display object }
  m_display.setup(m_library.lpDD2);
End;

Destructor TDirectXConsole.Destroy;

Begin
  { close }
  close;

  m_hook.Free;
  m_keyboard.Free;
  m_window.Free;

  m_primary.Free;
  m_display.Free;
  m_library.Free;
  m_copy.Free;
  m_default_format.Free;
  Inherited Destroy;
End;

Procedure TDirectXConsole.configure(Const _file : String);

Var
  F : Text;
  S : String;

Begin
  ASSignFile(F, _file);
  {$I-}
  Reset(F);
  {$I+}
  If IOResult <> 0 Then
    Exit;
  While Not EoF(F) Do
  Begin
    {$I-}
    Readln(F, S);
    {$I+}
    If IOResult <> 0 Then
      Break;
    option(S);
  End;
  CloseFile(F);
End;

Function TDirectXConsole.option(Const _option : String) : Boolean;

Var
  tmp, tmp2 : Integer;
  tmpformat : TPTCFormat;

Begin
  LOG('console option', _option);
  option := True;
  If _option = 'default output' Then
  Begin
    m_output_mode := DEFAULT;
    Exit;
  End;
  If _option = 'windowed output' Then
  Begin
    m_output_mode := WINDOWED;
    Exit;
  End;
  If _option = 'fullscreen output' Then
  Begin
    m_output_mode := FULLSCREEN;
    Exit;
  End;
  If System.Copy(_option, 1, 13) = 'default width' Then
  Begin
    If Length(_option) > 13 Then
    Begin
      Val(System.Copy(_option, 14, Length(_option)-13), m_default_width, tmp);
      If m_default_width = 0 Then
        m_default_width := DEFAULT_WIDTH;
    End
    Else
    Begin
      m_default_width := DEFAULT_WIDTH;
    End;
  End;
  If System.Copy(_option, 1, 14) = 'default height' Then
  Begin
    If Length(_option) > 14 Then
    Begin
      Val(System.Copy(_option, 15, Length(_option)-14), m_default_height, tmp);
      If m_default_height = 0 Then
        m_default_height := DEFAULT_HEIGHT;
    End
    Else
    Begin
      m_default_height := DEFAULT_HEIGHT;
    End;
  End;
  If System.Copy(_option, 1, 12) = 'default bits' Then
  Begin
    If Length(_option) > 12 Then
    Begin
      Val(System.Copy(_option, 13, Length(_option)-12), tmp, tmp2);
      Case tmp Of
        8 : tmpformat := TPTCFormat.Create(8);
        16 : tmpformat := TPTCFormat.Create(16, $F800, $07E0, $001F);
        24 : tmpformat := TPTCFormat.Create(24, $00FF0000, $0000FF00, $000000FF);
        32 : tmpformat := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);
        Else
          Exit(False);
      End;
      Try
        m_default_format.ASSign(tmpformat);
      Finally
        tmpformat.Free;
      End;
    End
    Else
    Begin
      tmpformat := DEFAULT_FORMAT;
      Try
        m_default_format.ASSign(tmpformat);
      Finally
        tmpformat.Free;
      End;
    End;
  End;
  If _option = 'resizable window' Then
  Begin
    m_window_mode := RESIZABLE;
    Exit;
  End;
  If _option = 'fixed window' Then
  Begin
    m_window_mode := FIXED;
    Exit;
  End;
  If _option = 'windowed primary direct' Then
  Begin
    m_primary_mode_windowed := DIRECT;
    Exit;
  End;
  If _option = 'windowed primary secondary' Then
  Begin
    m_primary_mode_windowed := SECONDARY;
    Exit;
  End;
  If _option = 'fullscreen primary direct' Then
  Begin
    m_primary_mode_fullscreen := DIRECT;
    Exit;
  End;
  If _option = 'fullscreen primary secondary' Then
  Begin
    m_primary_mode_fullscreen := SECONDARY;
    Exit;
  End;
  If _option = 'center window' Then
  Begin
    m_center_window := True;
    Exit;
  End;
  If _option = 'default window position' Then
  Begin
    m_center_window := False;
    Exit;
  End;
  If _option = 'synchronized update' Then
  Begin
    m_synchronized_update := True;
    Exit;
  End;
  If _option = 'unsynchronized update' Then
  Begin
    m_synchronized_update := False;
    Exit;
  End;
  If _option = 'default nearest' Then
  Begin
    m_nearest_mode := NEAREST_DEFAULT;
    Exit;
  End;
  If _option = 'center nearest' Then
  Begin
    m_nearest_mode := NEAREST_CENTERING;
    Exit;
  End;
  If _option = 'default stretch' Then
  Begin
    m_nearest_mode := NEAREST_STRETCHING;
    Exit;
  End;
  If _option = 'default cursor' Then
  Begin
    m_cursor_mode := CURSOR_DEFAULT;
    update_cursor;
    Exit;
  End;
  If _option = 'show cursor' Then
  Begin
    m_cursor_mode := CURSOR_SHOW;
    update_cursor;
    Exit;
  End;
  If _option = 'hide cursor' Then
  Begin
    m_cursor_mode := CURSOR_HIDE;
    update_cursor;
    Exit;
  End;
  If System.Copy(_option, 1, 9) = 'frequency' Then
  Begin
    If Length(_option) > 9 Then
    Begin
      Val(System.Copy(_option, 10, Length(_option)-9), m_frequency, tmp);
    End
    Else
      m_frequency := 0;
  End;
  If _option = 'enable key buffering' Then
  Begin
    If m_keyboard = Nil Then
    Begin
      option := False;
      Exit;
    End;
    m_keyboard.enable;
  End;
  If _option = 'disable key buffering' Then
  Begin
    If m_keyboard = Nil Then
    Begin
      option := False;
      Exit;
    End;
    m_keyboard.disable;
  End;
  If _option = 'enable blocking' Then
  Begin
    m_primary.blocking(True);
    Exit;
  End;
  If _option = 'disable blocking' Then
  Begin
    m_primary.blocking(False);
    Exit;
  End;
{$IFDEF PTC_LOGGING}
  If _option = 'enable logging' Then
  Begin
    LOG_enabled := True;
    option := True;
    Exit;
  End;
  If _option = 'disable logging' Then
  Begin
    LOG_enabled := False;
    option := True;
    Exit;
  End;
{$ENDIF}

  option := m_copy.option(_option);
End;

Function TDirectXConsole.modes : PPTCMode;

Begin
  modes := m_display.modes;
End;

Procedure TDirectXConsole.open(Const _title : String; _pages : Integer);

Begin
  open(_title, m_default_format, _pages);
End;

Procedure TDirectXConsole.open(Const _title : String; Const _format : TPTCFormat;
                           _pages : Integer);

Begin
  open(_title, m_default_width, m_default_height, _format, _pages);
End;

Procedure TDirectXConsole.open(Const _title : String; _width, _height : Integer;
                           Const _format : TPTCFormat; _pages : Integer);

Var
  m : TPTCMode;

Begin
  { internal open nearest mode }
  m := TPTCMode.Create(_width, _height, _format);
  Try
    internal_open(_title, 0, m, _pages, False);
  Finally
    m.Free;
  End;
End;

Procedure TDirectXConsole.open(Const _title : String; Const _mode : TPTCMode;
                           _pages : Integer);

Begin
  { internal open exact mode }
  internal_open(_title, 0, _mode, _pages, True);
End;

Procedure TDirectXConsole.close;

Begin
  If m_open Then
  Begin
    If m_locked Then
      Raise TPTCError.Create('console is still locked');

    { flush all key presses }
    While KeyPressed Do
      ReadKey;
  End;
  internal_close;
  Win32Cursor_resurrect;
End;

Procedure TDirectXConsole.flush;

Begin
  CHECK_OPEN('TDirectXConsole.flush');
  CHECK_LOCK('TDirectXConsole.flush');
  { [platform dependent code to flush all console operations] }

  { handle cursor show flag }
  If Not m_cursor Then
    SetCursor(0);

  { update window }
  m_window.update;
End;

Procedure TDirectXConsole.finish;

Begin
  CHECK_OPEN('TDirectXConsole.finish');
  CHECK_LOCK('TDirectXConsole.finish');
  { [platform dependent code to finish all console operations] }

  { handle cursor show flag }
  If Not m_cursor Then
    SetCursor(0);

  { update window }
  m_window.update;
End;

Procedure TDirectXConsole.update;

Begin
  CHECK_OPEN('TDirectXConsole.update');
  CHECK_LOCK('TDirectXConsole.update');

  { update primary surface }
  m_primary.update;

  { handle cursor show flag }
  If Not m_cursor Then
    SetCursor(0);

  { update window }
  m_window.update;
End;

Procedure TDirectXConsole.update(Const _area : TPTCArea);

Begin
  { update }
  update;
End;

Procedure TDirectXConsole.internal_ReadKey(k : TPTCKey);

Begin
  CHECK_OPEN('TDirectXConsole.internal_ReadKey');
  m_keyboard.internal_ReadKey(m_window, k);
End;

Function TDirectXConsole.internal_PeekKey(k : TPTCKey) : Boolean;

Begin
  CHECK_OPEN('TDirectXConsole.internal_PeekKey');
  Result := m_keyboard.internal_PeekKey(m_window, k);
End;

Procedure TDirectXConsole.copy(Var surface : TPTCBaseSurface);

Var
  pixels : Pointer;

Begin
  CHECK_OPEN('TDirectXConsole.copy(surface)');
  CHECK_LOCK('TDirectXConsole.copy(surface)');
  pixels := lock;
  Try
    surface.load(pixels, width, height, pitch, format, palette);
    unlock;
  Except
    On error : TPTCError Do
    Begin
      unlock;
      Raise TPTCError.Create('failed to copy console to surface', error);
    End;
  End;
End;

Procedure TDirectXConsole.copy(Var surface : TPTCBaseSurface;
                           Const source, destination : TPTCArea);

Var
  pixels : Pointer;

Begin
  CHECK_OPEN('TDirectXConsole.flush(surface, source, destination)');
  CHECK_LOCK('TDirectXConsole.flush(surface, source, destination)');
  pixels := lock;
  Try
    surface.load(pixels, width, height, pitch, format, palette, source, destination);
    unlock;
  Except
    On error : TPTCError Do
    Begin
      unlock;
      Raise TPTCError.Create('failed to copy console to surface', error);
    End;
  End;
End;

Function TDirectXConsole.lock : Pointer;

Begin
  CHECK_OPEN('TDirectXConsole.lock');
  { fail if the console is already locked }
  If m_locked Then
    Raise TPTCError.Create('console is already locked');

  { lock primary surface }
  lock := m_primary.lock;

  { surface is locked }
  m_locked := True;
End;

Procedure TDirectXConsole.unlock;

Begin
  CHECK_OPEN('TDirectXConsole.unlock');
  { fail if the console is not locked }
  If Not m_locked Then
    Raise TPTCError.Create('console is not locked');

  { unlock primary surface }
  m_primary.unlock;

  { we are unlocked }
  m_locked := False;
End;

Procedure TDirectXConsole.load(Const pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette);
Var
  Area_ : TPTCArea;
  console_pixels : Pointer;

Begin
  CHECK_OPEN('TDirectXConsole.load(pixels, width, height, pitch, format, palette)');
  CHECK_LOCK('TDirectXConsole.load(pixels, width, height, pitch, format, palette)');
  If clip.Equals(area) Then
  Begin
    console_pixels := lock;
    Try
      Try
        m_copy.request(_format, format);
        m_copy.palette(_palette, palette);
        m_copy.copy(pixels, 0, 0, _width, _height, _pitch, console_pixels, 0, 0,
                    width, height, pitch);
      Except
        On error : TPTCError Do
        Begin
          Raise TPTCError.Create('failed to load pixels to console', error);
        End;
      End;
    Finally
      unlock;
    End;
  End
  Else
  Begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    Try
      load(pixels, _width, _height, _pitch, _format, _palette, Area_, area);
    Finally
      Area_.Free;
    End;
  End;
End;

Procedure TDirectXConsole.load(Const pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette;
                           Const source, destination : TPTCArea);
Var
  console_pixels : Pointer;
  clipped_source, clipped_destination : TPTCArea;
  tmp : TPTCArea;

Begin
  CHECK_OPEN('TDirectXConsole.load(pixels, width, height, pitch, format, palette, source, destination)');
  CHECK_LOCK('TDirectXConsole.load(pixels, width, height, pitch, format, palette, source, destination)');
  clipped_destination := Nil;
  clipped_source := TPTCArea.Create;
  Try
    clipped_destination := TPTCArea.Create;
    console_pixels := lock;
    Try
      Try
        tmp := TPTCArea.Create(0, 0, _width, _height);
        Try
          TPTCClipper.clip(source, tmp, clipped_source, destination, clip, clipped_destination);
        Finally
          tmp.Free;
        End;
        m_copy.request(_format, format);
        m_copy.palette(_palette, palette);
        m_copy.copy(pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, _pitch,
                    console_pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, pitch);
      Except
        On error:TPTCError Do
        Begin
          Raise TPTCError.Create('failed to load pixels to console area', error);
        End;
      End;
    Finally
      unlock;
    End;
  Finally
    clipped_source.Free;
    clipped_destination.Free;
  End;
End;

Procedure TDirectXConsole.save(pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette);
Var
  Area_ : TPTCArea;
  console_pixels : Pointer;

Begin
  CHECK_OPEN('TDirectXConsole.save(pixels, width, height, pitch, format, palette)');
  CHECK_LOCK('TDirectXConsole.save(pixels, width, height, pitch, format, palette)');
  If clip.Equals(area) Then
  Begin
    console_pixels := lock;
    Try
      Try
        m_copy.request(format, _format);
        m_copy.palette(palette, _palette);
        m_copy.copy(console_pixels, 0, 0, width, height, pitch, pixels, 0, 0,
                    _width, _height, _pitch);
      Except
        On error : TPTCError Do
        Begin
          Raise TPTCError.Create('failed to save console pixels', error);
        End;
      End;
    Finally
      unlock;
    End;
  End
  Else
  Begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    Try
      save(pixels, _width, _height, _pitch, _format, _palette, area, Area_);
    Finally
      Area_.Free;
    End;
  End;
End;

Procedure TDirectXConsole.save(pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette;
                           Const source, destination : TPTCArea);
Var
  console_pixels : Pointer;
  clipped_source, clipped_destination : TPTCArea;
  tmp : TPTCArea;

Begin
  CHECK_OPEN('TDirectXConsole.save(pixels, width, height, pitch, format, palette, source, destination)');
  CHECK_LOCK('TDirectXConsole.save(pixels, width, height, pitch, format, palette, source, destination)');
  clipped_destination := Nil;
  clipped_source := TPTCArea.Create;
  Try
    clipped_destination := TPTCArea.Create;
    console_pixels := lock;
    Try
      Try
        tmp := TPTCArea.Create(0, 0, _width, _height);
        Try
          TPTCClipper.clip(source, clip, clipped_source, destination, tmp, clipped_destination);
        Finally
          tmp.Free;
        End;
        m_copy.request(format, _format);
        m_copy.palette(palette, _palette);
        m_copy.copy(console_pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, pitch,
                    pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, _pitch);
      Except
        On error:TPTCError Do
        Begin
          Raise TPTCError.Create('failed to save console area pixels', error);
        End;
      End;
    Finally
      unlock;
    End;
  Finally
    clipped_source.Free;
    clipped_destination.Free;
  End;
End;

Procedure TDirectXConsole.clear;

Var
  tmp : TPTCColor;

Begin
  CHECK_OPEN('TDirectXConsole.clear');
  CHECK_LOCK('TDirectXConsole.clear');
  If format.direct Then
    tmp := TPTCColor.Create(0, 0, 0, 0)
  Else
    tmp := TPTCColor.Create(0);
  Try
    clear(tmp);
  Finally
    tmp.Free;
  End;
End;

Procedure TDirectXConsole.clear(Const color : TPTCColor);

Var
  tmp : TPTCArea;

Begin
  CHECK_OPEN('TDirectXConsole.clear(color)');
  CHECK_LOCK('TDirectXConsole.clear(color)');
  tmp := TPTCArea.Create;
  Try
    clear(color, tmp);
  Finally
    tmp.Free;
  End;
End;

Procedure TDirectXConsole.clear(Const color : TPTCColor;
                           Const _area : TPTCArea);

Begin
  CHECK_OPEN('TDirectXConsole.clear(color, area)');
  CHECK_LOCK('TDirectXConsole.clear(color, area)');
  m_primary.clear(color, _area);
End;

Procedure TDirectXConsole.palette(Const _palette : TPTCPalette);

Begin
  CHECK_OPEN('TDirectXConsole.palette(palette)');
  m_primary.palette(_palette);
End;

Function TDirectXConsole.palette : TPTCPalette;

Begin
  CHECK_OPEN('TDirectXConsole.palette');
  palette := m_primary.palette;
End;

Procedure TDirectXConsole.clip(Const _area : TPTCArea);

Begin
  CHECK_OPEN('TDirectXConsole.clip(area)');
  m_primary.clip(_area);
End;

Function TDirectXConsole.width : Integer;

Begin
  CHECK_OPEN('TDirectXConsole.width');
  width := m_primary.width;
End;

Function TDirectXConsole.height : Integer;

Begin
  CHECK_OPEN('TDirectXConsole.height');
  height := m_primary.height;
End;

Function TDirectXConsole.pitch : Integer;

Begin
  CHECK_OPEN('TDirectXConsole.pitch');
  pitch := m_primary.pitch;
End;

Function TDirectXConsole.pages : Integer;

Begin
  CHECK_OPEN('TDirectXConsole.pages');
  pages := m_primary.pages;
End;

Function TDirectXConsole.area : TPTCArea;

Begin
  CHECK_OPEN('TDirectXConsole.area');
  area := m_primary.area;
End;

Function TDirectXConsole.clip : TPTCArea;

Begin
  CHECK_OPEN('TDirectXConsole.clip');
  clip := m_primary.clip;
End;

Function TDirectXConsole.format : TPTCFormat;

Begin
  CHECK_OPEN('TDirectXConsole.format');
  format := m_primary.format;
End;

Function TDirectXConsole.name : String;

Begin
  name := 'DirectX';
End;

Function TDirectXConsole.title : String;

Begin
  CHECK_OPEN('TDirectXConsole.title');
  title := m_title;
End;

Function TDirectXConsole.information : String;

Begin
  CHECK_OPEN('TDirectXConsole.information');
  information := m_display.information;
End;

Procedure TDirectXConsole.internal_open(Const _title : String; window : HWND; Const mode : TPTCMode; _pages : Integer; exact : Boolean);

Var
  _width, _height : Integer;
  _format : TPTCFormat;

Begin
  Try
    { recycle an already open console }
    internal_recycle(_title, window, mode, _pages, exact);
    Exit;
  Except
    On TPTCError Do
    { could not recycle };
  End;

  { check that the mode is valid }
  If Not mode.valid Then
    Raise TPTCError.Create('invalid mode');

  { get mode information }
  _width := mode.width;
  _height := mode.height;
  _format := mode.format;

  { start internal open }
  internal_open_start(_title, window);

  { check output mode }
  Case m_output_mode Of
    DEFAULT :
    Try
      { start fullscreen open }
      internal_open_fullscreen_start(window, mode, exact);

      { change fullscreen display }
      internal_open_fullscreen_change(mode, exact);

      { setup fullscreen display surfaces }
      internal_open_fullscreen_surface(mode, _pages);

      { finish fullscreen open }
      internal_open_fullscreen_finish;
    Except
      On TPTCError Do
      Begin
        { internal open reset }
        internal_open_reset;

        { start windowed open }
        internal_open_windowed_start(window, mode, exact);

        { change windowed display display mode }
        internal_open_windowed_change(mode, exact);

        { setup windowed display }
        internal_open_windowed_surface(mode, _pages);

        { finish windowed open }
        internal_open_windowed_finish;
      End;
    End;
    WINDOWED : Begin
      { start windowed open }
      internal_open_windowed_start(window, mode, exact);

      { change windowed display display mode }
      internal_open_windowed_change(mode, exact);

      { setup windowed display }
      internal_open_windowed_surface(mode, _pages);

      { finish windowed open }
      internal_open_windowed_finish;
    End;
    FULLSCREEN : Begin
      { start fullscreen open }
      internal_open_fullscreen_start(window, mode, exact);

      { change fullscreen display }
      internal_open_fullscreen_change(mode, exact);

      { setup fullscreen display surfaces }
      internal_open_fullscreen_surface(mode, _pages);

      { finish fullscreen open }
      internal_open_fullscreen_finish;
    End;
  End;

  { finish internal open }
  internal_open_finish;
End;

Procedure TDirectXConsole.internal_recycle(Const _title : String; window : HWND; Const mode : TPTCMode; _pages : Integer; exact : Boolean);

Begin
  { Check if the console is open }
  If not m_open Then
    Raise TPTCError.Create('cannot recycle because it is not already open');
  If window <> 0 Then
  Begin
    If (m_window.handle <> window) Or (Not (m_window.managed)) Then
      Raise TPTCError.Create('cannot recycle with this user window');
  End;
  Case m_output_mode Of
    DEFAULT :
      If m_display.fullscreen Then
      Begin
        Try
          internal_recycle_fullscreen(_title, window, mode, _pages, exact);
        Except
          On TPTCError Do
            Raise TPTCError.Create('recycling fullscreen to windowed is not implemented');
        End;
      End
      Else
        Raise TPTCError.Create('recycling windowed to fullscreen is not implemented');
    FULLSCREEN : internal_recycle_fullscreen(_title, window, mode, _pages, exact);
    WINDOWED : internal_recycle_fullscreen(_title, window, mode, _pages, exact);
  End;
End;

Procedure TDirectXConsole.internal_close;

Begin
  m_open := False;
  FreeAndNil(m_keyboard);
  FreeAndNil(m_hook);
  If m_primary <> Nil Then
    m_primary.close;
  If m_display <> Nil Then
    m_display.close;
  FreeAndNil(m_window);
  If m_display <> Nil Then
    m_display.restore;
End;

Procedure TDirectXConsole.internal_shutdown;

Begin
  m_library.close;
End;

Procedure TDirectXConsole.internal_open_start(Const _title : String; window : HWND);

Var
  tmp : Array[0..1023] Of Char;

Begin
  { close_down }
  internal_close;

  { check window }
  If window = 0 Then
  Begin
    m_title := _title;
  End
  Else
  Begin
    GetWindowText(window, @tmp, SizeOf(tmp));
    m_title := PChar2String(@tmp);
  End;
End;

Procedure TDirectXConsole.internal_open_finish;

Begin
  FreeAndNil(m_keyboard);
  m_keyboard := TWin32Keyboard.Create(m_window.handle, m_window.thread, False);
  m_window.update;
  m_open := True;
End;

Procedure TDirectXConsole.internal_open_reset;

Begin
  FreeAndNil(m_keyboard);
  FreeAndNil(m_hook);
  m_primary.close;
  FreeAndNil(m_window);
  m_display.restore;
End;

Procedure TDirectXConsole.internal_open_fullscreen_start(window : HWND; Const mode : TPTCMode; exact : Boolean);

Begin
  { test if display mode exists... }
  If Not m_display.test(mode, exact) Then
    Raise TPTCError.Create('display mode test failed!');

  { handle cursor show mode }
  If m_cursor_mode = CURSOR_SHOW Then
    m_cursor := True
  Else
    m_cursor := False;

  { save display }
  m_display.save;

  { check window }
  If window = 0 Then
    m_window := TWin32Window.Create('PTC_DIRECTX_FULLSCREEN', m_title, WS_EX_TOPMOST, WS_POPUP Or WS_SYSMENU Or WS_VISIBLE, SW_NORMAL, 0, 0, GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN), False, False)
  Else
    m_window := TWin32Window.Create(window);

  { set window cursor }
  m_window.cursor(m_cursor);

  { set cooperative level }
  m_display.cooperative(m_window.handle, True);
End;

Procedure TDirectXConsole.internal_open_fullscreen_change(Const mode : TPTCMode; exact : Boolean);

Begin
  m_display.open(mode, exact, m_frequency);
  m_primary.blocking(True);
End;

Procedure TDirectXConsole.internal_open_fullscreen_surface(Const mode : TPTCMode; _pages : Integer);

Var
  primary : Boolean;
  _secondary : Boolean;
  _palette : Boolean;
  complex : Boolean;

Begin
  _secondary := (m_primary_mode_fullscreen = SECONDARY) Or (Not m_display.mode.Equals(mode));
  _palette := m_display.mode.format.indexed;
  m_primary.initialize(m_window, m_library.lpDD2);
  complex := False;
  primary := False;

  { randy heit's primary setup }
  While (Not primary) And (Not complex) Do
  Begin
    If _pages >= 1 Then
    Try
      m_primary.primary(_pages, True, True, _palette, complex);
      primary := True;
    Except
      On TPTCError Do;
    End;
    If Not primary Then
    Try
      m_primary.primary(3, True, True, _palette, complex);
      primary := True;
    Except
      On TPTCError Do
      Try
        m_primary.primary(2, True, True, _palette, complex);
        primary := True;
      Except
        On TPTCError Do
        Try
          If Not _secondary Then
            m_primary.primary(2, False, True, _palette, complex)
          Else
            m_primary.primary(1, False, True, _palette, complex);
          primary := True;
        Except
          On TPTCError Do
            complex := Not complex;
        End;
      End;
    End;
  End;
  If _secondary Then
    m_primary.secondary(mode.width, mode.height);
  If m_nearest_mode = NEAREST_CENTERING Then
    m_primary.centering(True);
  If m_nearest_mode = NEAREST_STRETCHING Then
    m_primary.centering(False);
  {
  original primary setup code (1.0.17)
  ...
  }

  m_primary.synchronize(m_synchronized_update);
End;

Procedure TDirectXConsole.internal_open_fullscreen_finish;

Begin
  FreeAndNil(m_hook);

  { create hook on window }
  m_hook := TDirectXHook.Create(Self, m_window.handle, GetCurrentThreadId, m_cursor, m_window.managed, True);
End;

Procedure TDirectXConsole.internal_open_windowed_start(window : HWND; Const mode : TPTCMode; exact : Boolean);

Var
  extended : Integer;

Begin
  If m_cursor_mode = CURSOR_HIDE Then
    m_cursor := False
  Else
    m_cursor := True;
  FreeAndNil(m_window);
  If window <> 0 Then
  Begin
    m_window := TWin32Window.Create(window);
  End
  Else
  Begin
    extended := 0;
    If m_primary_mode_windowed = DIRECT Then
      extended := WS_EX_TOPMOST;
    Case m_window_mode Of
      RESIZABLE : m_window := TWin32Window.Create('PTC_DIRECTX_WINDOWED_RESIZABLE', m_title,
                                              extended, WS_OVERLAPPEDWINDOW Or WS_VISIBLE, SW_NORMAL, CW_USEDEFAULT, CW_USEDEFAULT, mode.width, mode.height, m_center_window, False);
      FIXED : m_window := TWin32Window.Create('PTC_DIRECTX_WINDOWED_FIXED', m_title,
                                              extended, WS_VISIBLE Or WS_SYSMENU Or WS_CAPTION Or WS_MINIMIZE, SW_NORMAL, CW_USEDEFAULT, CW_USEDEFAULT, mode.width, mode.height, m_center_window, False);
    End;
  End;
  m_window.cursor(m_cursor);
  m_display.cooperative(m_window.handle, False);
End;

Procedure TDirectXConsole.internal_open_windowed_change(Const mode : TPTCMode; exact : Boolean);

Begin
  m_display.open;
  If m_primary_mode_windowed = DIRECT Then
    m_primary.blocking(True)
  Else
    m_primary.blocking(False);
End;

Procedure TDirectXConsole.internal_open_windowed_surface(Const mode : TPTCMode; _pages : Integer);

Begin
  m_primary.initialize(m_window, m_library.lpDD2);
  m_primary.primary(1, False, False, False, False);
  If m_primary_mode_windowed = SECONDARY Then
    m_primary.secondary(mode.width, mode.height);
End;

Procedure TDirectXConsole.internal_open_windowed_finish;

Begin
  FreeAndNil(m_hook);

  { create hook on window }
  m_hook := TDirectXHook.Create(Self, m_window.handle, GetCurrentThreadId, m_cursor, m_window.managed, False);
End;

Procedure TDirectXConsole.internal_recycle_fullscreen(Const _title : String; window : HWND; Const mode : TPTCMode; _pages : Integer; exact : Boolean);

Begin
  LOG('fullscreen open recycle');
  m_primary.close;
  internal_open_fullscreen_change(mode, exact);
  internal_open_fullscreen_surface(mode, _pages);
End;

Procedure TDirectXConsole.internal_recycle_windowed(Const _title : String; window : HWND; Const mode : TPTCMode; _pages : Integer; exact : Boolean);

Begin
  LOG('windowed open recycle');
  m_primary.close;
  m_window.resize(mode.width, mode.height);
  internal_open_windowed_change(mode, exact);
  internal_open_windowed_surface(mode, _pages);
End;

Procedure TDirectXConsole.paint;

Begin
  If m_locked Or (Not m_open) Then
    Exit;
  m_primary.paint;
End;

Procedure TDirectXConsole.update_cursor;

Begin
  If Not m_open Then
    Exit;
  If m_display.fullscreen Then
    If m_cursor_mode = CURSOR_SHOW Then
      m_cursor := True
    Else
      m_cursor := False
  Else
    If m_cursor_mode = CURSOR_HIDE Then
      m_cursor := False
    Else
      m_cursor := True;

  { update hook cursor }
  m_hook.cursor(m_cursor);

  { update window cursor }
  m_window.cursor(m_cursor);
End;

{$IFDEF DEBUG}
Procedure TDirectXConsole.CHECK_OPEN(msg : String);

Begin
  If Not m_open Then
  Try
    Raise TPTCError.Create('console is not open');
  Except
    On error : TPTCError Do
      Raise TPTCError.Create(msg, error);
  End;
End;

Procedure TDirectXConsole.CHECK_LOCK(msg : String);

Begin
  If m_locked Then
  Try
    Raise TPTCError.Create('console is locked');
  Except
    On error : TPTCError Do
      Raise TPTCError.Create(msg, error);
  End;
End;
{$ENDIF}
