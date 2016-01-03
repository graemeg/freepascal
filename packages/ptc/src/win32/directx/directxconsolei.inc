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

//const
  {Output}
//  DEFAULT = 0;
//  WINDOWED = 1;
//  FULLSCREEN = 2;

  {Window}
//  RESIZABLE = 0;
//  FIXED = 1;

  {Primary}
//  DIRECT = 0;
//  SECONDARY = 1;

  {Nearest}
//  NEAREST_DEFAULT = 0;
//  NEAREST_CENTERING = 1;
//  NEAREST_STRETCHING = 2;

function PChar2String(Q: PChar): string;
var
  I: Integer;
  S: string;
begin
  S := '';
  I := 0;
  while Q[I] <> #0 do
  begin
    S := S + Q[I];
    Inc(I);
  end;
  PChar2String := S;
end;

constructor TDirectXConsole.Create;
begin
  inherited Create;

  FCopy := TPTCCopy.Create;
  FWin32Cursor := TWin32Cursor.Create;
  FLibrary := TDirectXLibrary.Create;
  FDisplay := TDirectXDisplay.Create;
  FPrimary := TDirectXPrimary.Create;

  { defaults }
  FOpen := False;
  FLocked := False;
  FCursor := True;
  FGrabMouse := False;

  { clear strings }
  FTitle := '';

  { default option data }
  FFrequency := 0;
  FDefaultWidth := DEFAULT_WIDTH;
  FDefaultHeight := DEFAULT_HEIGHT;
  FDefaultFormat := DEFAULT_FORMAT;
  FCenterWindow := False;
  FSynchronizedUpdate := True;
  FOutputMode := DEFAULT_OUTPUT;
  FWindowMode := RESIZABLE;
  FPrimaryModeWindowed := SECONDARY;
  FPrimaryModeFullscreen := DIRECT;
  FNearestMode := NEAREST_DEFAULT;
  FCursorMode := CURSOR_DEFAULT;

  { configure console }
  Configure('ptcpas.cfg');

  { setup display object }
  FDisplay.Setup(FLibrary.lpDD2);
end;

destructor TDirectXConsole.Destroy;
begin
  { close }
  Close;

  FHook.Free;
  FMouse.Free;
  FKeyboard.Free;
  FWindow.Free;

  FPrimary.Free;
  FDisplay.Free;
  FLibrary.Free;
  FEventQueue.Free;
  FWin32Cursor.Free;
  FCopy.Free;
  FDefaultFormat.Free;
  inherited Destroy;
end;

procedure TDirectXConsole.Configure(const AFileName: String);
var
  F: TextFile;
  S: string;
begin
  AssignFile(F, AFileName);
  {$I-}
  Reset(F);
  {$I+}
  if IOResult <> 0 then
    exit;
  while not EoF(F) do
  begin
    {$I-}
    Readln(F, S);
    {$I+}
    if IOResult <> 0 then
      Break;
    Option(S);
  end;
  CloseFile(F);
end;

function TDirectXConsole.Option(const AOption: String): Boolean;
var
  tmp, tmp2: Integer;
  tmpformat: TPTCFormat;
begin
  LOG('console option', AOption);
  Result := True;
  if AOption = 'default output' then
  begin
    FOutputMode := DEFAULT;
    exit;
  end;
  if AOption = 'windowed output' then
  begin
    FOutputMode := WINDOWED;
    exit;
  end;
  if AOption = 'fullscreen output' then
  begin
    FOutputMode := FULLSCREEN;
    exit;
  end;
  if System.Copy(AOption, 1, 13) = 'default width' then
  begin
    if Length(AOption) > 13 then
    begin
      Val(System.Copy(AOption, 14, Length(AOption)-13), FDefaultWidth, tmp);
      if FDefaultWidth = 0 then
        FDefaultWidth := DEFAULT_WIDTH;
    end
    else
    begin
      FDefaultWidth := DEFAULT_WIDTH;
    end;
  end;
  if System.Copy(AOption, 1, 14) = 'default height' then
  begin
    if Length(AOption) > 14 then
    begin
      Val(System.Copy(AOption, 15, Length(AOption)-14), FDefaultHeight, tmp);
      if FDefaultHeight = 0 then
        FDefaultHeight := DEFAULT_HEIGHT;
    end
    else
    begin
      FDefaultHeight := DEFAULT_HEIGHT;
    end;
  end;
  if System.Copy(AOption, 1, 12) = 'default bits' then
  begin
    if Length(AOption) > 12 then
    begin
      Val(System.Copy(AOption, 13, Length(AOption)-12), tmp, tmp2);
      case tmp of
        8: tmpformat := TPTCFormat.Create(8);
        16: tmpformat := TPTCFormat.Create(16, $F800, $07E0, $001F);
        24: tmpformat := TPTCFormat.Create(24, $00FF0000, $0000FF00, $000000FF);
        32: tmpformat := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);
        else
          exit(False);
      end;
      try
        FDefaultFormat.Assign(tmpformat);
      finally
        tmpformat.Free;
      end;
    end
    else
    begin
      tmpformat := DEFAULT_FORMAT;
      try
        FDefaultFormat.Assign(tmpformat);
      finally
        tmpformat.Free;
      end;
    end;
  end;
  if AOption = 'resizable window' then
  begin
    FWindowMode := RESIZABLE;
    exit;
  end;
  if AOption = 'fixed window' then
  begin
    FWindowMode := FIXED;
    exit;
  end;
  if AOption = 'windowed primary direct' then
  begin
    FPrimaryModeWindowed := DIRECT;
    exit;
  end;
  if AOption = 'windowed primary secondary' then
  begin
    FPrimaryModeWindowed := SECONDARY;
    exit;
  end;
  if AOption = 'fullscreen primary direct' then
  begin
    FPrimaryModeFullscreen := DIRECT;
    exit;
  end;
  if AOption = 'fullscreen primary secondary' then
  begin
    FPrimaryModeFullscreen := SECONDARY;
    exit;
  end;
  if AOption = 'center window' then
  begin
    FCenterWindow := True;
    exit;
  end;
  if AOption = 'default window position' then
  begin
    FCenterWindow := False;
    exit;
  end;
  if AOption = 'synchronized update' then
  begin
    FSynchronizedUpdate := True;
    exit;
  end;
  if AOption = 'unsynchronized update' then
  begin
    FSynchronizedUpdate := False;
    exit;
  end;
  if AOption = 'default nearest' then
  begin
    FNearestMode := NEAREST_DEFAULT;
    exit;
  end;
  if AOption = 'center nearest' then
  begin
    FNearestMode := NEAREST_CENTERING;
    exit;
  end;
  if AOption = 'default stretch' then
  begin
    FNearestMode := NEAREST_STRETCHING;
    exit;
  end;
  if AOption = 'default cursor' then
  begin
    FCursorMode := CURSOR_DEFAULT;
    UpdateCursor;
    exit;
  end;
  if AOption = 'show cursor' then
  begin
    FCursorMode := CURSOR_SHOW;
    UpdateCursor;
    exit;
  end;
  if AOption = 'hide cursor' then
  begin
    FCursorMode := CURSOR_HIDE;
    UpdateCursor;
    exit;
  end;
  if AOption = 'grab mouse' then
  begin
    if FOpen and (not FFullscreen) then
    begin
      FWindow.ConfineCursor(True);
    end;
    FGrabMouse := True;
    exit;
  end;
  if AOption = 'ungrab mouse' then
  begin
    if FOpen and (not FFullscreen) then
    begin
      FWindow.ConfineCursor(False);
    end;
    FGrabMouse := False;
    exit;
  end;
  if System.Copy(AOption, 1, 9) = 'frequency' then
  begin
    if Length(AOption) > 9 then
    begin
      Val(System.Copy(AOption, 10, Length(AOption)-9), FFrequency, tmp);
    end
    else
      FFrequency := 0;
  end;
  if AOption = 'enable key buffering' then
  begin
    if FKeyboard = nil then
    begin
      Result := False;
      exit;
    end;
    FKeyboard.Enable;
  end;
  if AOption = 'disable key buffering' then
  begin
    if FKeyboard = nil then
    begin
      Result := False;
      exit;
    end;
    FKeyboard.Disable;
  end;
  if AOption = 'enable blocking' then
  begin
    FPrimary.blocking(True);
    exit;
  end;
  if AOption = 'disable blocking' then
  begin
    FPrimary.blocking(False);
    exit;
  end;
  if AOption = 'enable logging' then
  begin
    LOG_enabled := True;
    Result := True;
    exit;
  end;
  if AOption = 'disable logging' then
  begin
    LOG_enabled := False;
    Result := True;
    exit;
  end;

  Result := FCopy.Option(AOption);
end;

function TDirectXConsole.Modes: PPTCMode;
begin
  Result := FDisplay.Modes;
end;

procedure TDirectXConsole.Open(const ATitle: string; APages: Integer = 0);
begin
  Open(ATitle, FDefaultFormat, APages);
end;

procedure TDirectXConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
                           APages: Integer = 0);
begin
  Open(ATitle, FDefaultWidth, FDefaultHeight, AFormat, APages);
end;

procedure TDirectXConsole.Open(const ATitle: string; AWidth, AHeight: Integer;
                           const AFormat: TPTCFormat; APages: Integer = 0);
var
  m: TPTCMode;
begin
  { internal open nearest mode }
  m := TPTCMode.Create(AWidth, AHeight, AFormat);
  try
    internal_open(ATitle, 0, m, APages, False);
  finally
    m.Free;
  end;
end;

procedure TDirectXConsole.Open(const ATitle: string; const AMode: TPTCMode;
                           APages: Integer = 0);
begin
  { internal open exact mode }
  internal_open(ATitle, 0, AMode, APages, True);
end;

procedure TDirectXConsole.Close;
begin
  if FOpen then
  begin
    if FLocked then
      raise TPTCError.Create('console is still locked');

    { flush all key presses }
    while KeyPressed do
      ReadKey;
  end;
  internal_close;
  FWin32Cursor.Show;
end;

procedure TDirectXConsole.Flush;
begin
  CHECK_OPEN('TDirectXConsole.Flush');
  CHECK_LOCK('TDirectXConsole.Flush');
  { [platform dependent code to flush all console operations] }

  { handle cursor show flag }
{  if not FCursor then
    SetCursor(0);}

  { update window }
  FWindow.Update;
end;

procedure TDirectXConsole.Finish;
begin
  CHECK_OPEN('TDirectXConsole.Finish');
  CHECK_LOCK('TDirectXConsole.Finish');
  { [platform dependent code to finish all console operations] }

  { handle cursor show flag }
{  if not FCursor then
    SetCursor(0);}

  { update window }
  FWindow.Update;
end;

procedure TDirectXConsole.Update;
begin
  CHECK_OPEN('TDirectXConsole.Update');
  CHECK_LOCK('TDirectXConsole.Update');

  { update primary surface }
  FPrimary.Update;

  { handle cursor show flag }
{  if not FCursor then
    SetCursor(0);}

  { update window }
  FWindow.Update;
end;

procedure TDirectXConsole.Update(const AArea: TPTCArea);
begin
  { update }
  Update;
end;

function TDirectXConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
var
  UseGetMessage: Boolean;
begin
  CHECK_OPEN('TDirectXConsole.NextEvent');
//  CHECK_LOCK('TDirectXConsole.NextEvent');

  FreeAndNil(AEvent);
  UseGetMessage := False;
  repeat
    FPrimary.Block;

    { update window }
    FWindow.Update(False, UseGetMessage);

    { try to find an event that matches the EventMask }
    AEvent := FEventQueue.NextEvent(AEventMask);
    if AWait then
      UseGetMessage := True;
  until (not AWait) or (AEvent <> nil);
  Result := AEvent <> nil;
end;

function TDirectXConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
var
  UseGetMessage: Boolean;
begin
  CHECK_OPEN('TDirectXConsole.PeekEvent');
//  CHECK_LOCK('TDirectXConsole.PeekEvent');

  UseGetMessage := False;
  repeat
    FPrimary.Block;

    { update window }
    FWindow.Update(False, UseGetMessage);

    { try to find an event that matches the EventMask }
    Result := FEventQueue.PeekEvent(AEventMask);
    if AWait then
      UseGetMessage := True;
  until (not AWait) or (Result <> nil);
end;

procedure TDirectXConsole.Copy(ASurface: TPTCBaseSurface);
var
  pixels: Pointer;
begin
  CHECK_OPEN('TDirectXConsole.Copy(ASurface)');
  CHECK_LOCK('TDirectXConsole.Copy(ASurface)');
  pixels := Lock;
  try
    ASurface.Load(Pixels, Width, Height, Pitch, Format, Palette);
    Unlock;
  except
    on error: TPTCError do
    begin
      Unlock;
      raise TPTCError.Create('failed to copy console to surface', error);
    end;
  end;
end;

procedure TDirectXConsole.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
var
  pixels: Pointer;
begin
  CHECK_OPEN('TDirectXConsole.Copy(ASurface, ASource, ADestination)');
  CHECK_LOCK('TDirectXConsole.Copy(ASurface, ASource, ADestination)');
  pixels := Lock;
  try
    ASurface.Load(Pixels, Width, Height, Pitch, Format, Palette, ASource, ADestination);
    Unlock;
  except
    on error: TPTCError do
    begin
      Unlock;
      raise TPTCError.Create('failed to copy console to surface', error);
    end;
  end;
end;

function TDirectXConsole.Lock: Pointer;
begin
  CHECK_OPEN('TDirectXConsole.Lock');
  { fail if the console is already locked }
  if FLocked then
    raise TPTCError.Create('console is already locked');

  { lock primary surface }
  Result := FPrimary.Lock;

  { surface is locked }
  FLocked := True;
end;

procedure TDirectXConsole.Unlock;
begin
  CHECK_OPEN('TDirectXConsole.Unlock');
  { fail if the console is not locked }
  if not FLocked then
    raise TPTCError.Create('console is not locked');

  { unlock primary surface }
  FPrimary.Unlock;

  { we are unlocked }
  FLocked := False;
end;

procedure TDirectXConsole.Load(const APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
var
  Area_: TPTCArea;
  console_pixels: Pointer;
begin
  CHECK_OPEN('TDirectXConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  CHECK_LOCK('TDirectXConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  if Clip.Equals(Area) then
  begin
    console_pixels := Lock;
    try
      try
        FCopy.Request(AFormat, Format);
        FCopy.Palette(APalette, Palette);
        FCopy.Copy(APixels, 0, 0, AWidth, AHeight, APitch, console_pixels, 0, 0,
                    Width, Height, Pitch);
      except
        on error: TPTCError do
        begin
          raise TPTCError.Create('failed to load pixels to console', error);
        end;
      end;
    finally
      Unlock;
    end;
  end
  else
  begin
    Area_ := TPTCArea.Create(0, 0, Width, Height);
    try
      Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, Area_, area);
    finally
      Area_.Free;
    end;
  end;
end;

procedure TDirectXConsole.Load(const APixels: Pointer;
                               AWidth, AHeight, APitch: Integer;
                               const AFormat: TPTCFormat;
                               const APalette: TPTCPalette;
                               const ASource, ADestination: TPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: TPTCArea;
  tmp: TPTCArea;
begin
  CHECK_OPEN('TDirectXConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  CHECK_LOCK('TDirectXConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  clipped_destination := nil;
  clipped_source := TPTCArea.Create;
  try
    clipped_destination := TPTCArea.Create;
    console_pixels := Lock;
    try
      try
        tmp := TPTCArea.Create(0, 0, AWidth, AHeight);
        try
          TPTCClipper.clip(ASource, tmp, clipped_source, ADestination, Clip, clipped_destination);
        finally
          tmp.Free;
        end;
        FCopy.Request(AFormat, Format);
        FCopy.Palette(APalette, Palette);
        FCopy.Copy(APixels, clipped_source.Left, clipped_source.Top, clipped_source.Width, clipped_source.Height, APitch,
                    console_pixels, clipped_destination.Left, clipped_destination.Top, clipped_destination.Width, clipped_destination.Height, Pitch);
      except
        on error:TPTCError do
        begin
          raise TPTCError.Create('failed to load pixels to console area', error);
        end;
      end;
    finally
      Unlock;
    end;
  finally
    clipped_source.Free;
    clipped_destination.Free;
  end;
end;

procedure TDirectXConsole.Save(APixels: Pointer;
                               AWidth, AHeight, APitch: Integer;
                               const AFormat: TPTCFormat;
                               const APalette: TPTCPalette);
var
  Area_: TPTCArea;
  console_pixels: Pointer;
begin
  CHECK_OPEN('TDirectXConsole.Save(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  CHECK_LOCK('TDirectXConsole.Save(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  if Clip.Equals(Area) then
  begin
    console_pixels := Lock;
    try
      try
        FCopy.Request(Format, AFormat);
        FCopy.Palette(Palette, APalette);
        FCopy.Copy(console_pixels, 0, 0, Width, Height, Pitch, APixels, 0, 0,
                    AWidth, AHeight, APitch);
      except
        on error: TPTCError do
        begin
          raise TPTCError.Create('failed to save console pixels', error);
        end;
      end;
    finally
      Unlock;
    end;
  end
  else
  begin
    Area_ := TPTCArea.Create(0, 0, Width, Height);
    try
      Save(APixels, AWidth, AHeight, APitch, AFormat, APalette, Area, Area_);
    finally
      Area_.Free;
    end;
  end;
end;

procedure TDirectXConsole.Save(APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: TPTCArea;
  tmp: TPTCArea;
begin
  CHECK_OPEN('TDirectXConsole.Save(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  CHECK_LOCK('TDirectXConsole.Save(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  clipped_destination := nil;
  clipped_source := TPTCArea.Create;
  try
    clipped_destination := TPTCArea.Create;
    console_pixels := Lock;
    try
      try
        tmp := TPTCArea.Create(0, 0, AWidth, AHeight);
        try
          TPTCClipper.Clip(ASource, Clip, clipped_source, ADestination, tmp, clipped_destination);
        finally
          tmp.Free;
        end;
        FCopy.Request(Format, AFormat);
        FCopy.Palette(Palette, APalette);
        FCopy.Copy(console_pixels, clipped_source.Left, clipped_source.Top, clipped_source.Width, clipped_source.Height, Pitch,
                    APixels, clipped_destination.Left, clipped_destination.Top, clipped_destination.Width, clipped_destination.Height, APitch);
      except
        on error:TPTCError do
        begin
          raise TPTCError.Create('failed to save console area pixels', error);
        end;
      end;
    finally
      Unlock;
    end;
  finally
    clipped_source.Free;
    clipped_destination.Free;
  end;
end;

procedure TDirectXConsole.Clear;
var
  tmp: TPTCColor;
begin
  CHECK_OPEN('TDirectXConsole.Clear');
  CHECK_LOCK('TDirectXConsole.Clear');
  if format.direct then
    tmp := TPTCColor.Create(0, 0, 0, 0)
  else
    tmp := TPTCColor.Create(0);
  try
    Clear(tmp);
  finally
    tmp.Free;
  end;
end;

procedure TDirectXConsole.Clear(const AColor: TPTCColor);
var
  tmp: TPTCArea;
begin
  CHECK_OPEN('TDirectXConsole.Clear(AColor)');
  CHECK_LOCK('TDirectXConsole.Clear(AColor)');
  tmp := TPTCArea.Create;
  try
    Clear(AColor, tmp);
  finally
    tmp.Free;
  end;
end;

procedure TDirectXConsole.Clear(const AColor: TPTCColor;
                                const AArea: TPTCArea);
begin
  CHECK_OPEN('TDirectXConsole.Clear(AColor, AArea)');
  CHECK_LOCK('TDirectXConsole.Clear(AColor, AArea)');
  FPrimary.Clear(AColor, AArea);
end;

procedure TDirectXConsole.palette(const APalette: TPTCPalette);
begin
  CHECK_OPEN('TDirectXConsole.Palette(APalette)');
  FPrimary.Palette(APalette);
end;

function TDirectXConsole.Palette: TPTCPalette;
begin
  CHECK_OPEN('TDirectXConsole.Palette');
  Result := FPrimary.palette;
end;

procedure TDirectXConsole.Clip(const AArea: TPTCArea);
begin
  CHECK_OPEN('TDirectXConsole.Clip(AArea)');
  FPrimary.Clip(AArea);
end;

function TDirectXConsole.GetWidth: Integer;
begin
  CHECK_OPEN('TDirectXConsole.GetWidth');
  Result := FPrimary.width;
end;

function TDirectXConsole.GetHeight: Integer;
begin
  CHECK_OPEN('TDirectXConsole.GetHeight');
  Result := FPrimary.height;
end;

function TDirectXConsole.GetPitch: Integer;
begin
  CHECK_OPEN('TDirectXConsole.GetPitch');
  Result := FPrimary.pitch;
end;

function TDirectXConsole.GetPages: Integer;
begin
  CHECK_OPEN('TDirectXConsole.GetPages');
  Result := FPrimary.pages;
end;

function TDirectXConsole.GetArea: TPTCArea;
begin
  CHECK_OPEN('TDirectXConsole.GetArea');
  Result := FPrimary.area;
end;

function TDirectXConsole.Clip: TPTCArea;
begin
  CHECK_OPEN('TDirectXConsole.Clip');
  Result := FPrimary.clip;
end;

function TDirectXConsole.GetFormat: TPTCFormat;
begin
  CHECK_OPEN('TDirectXConsole.GetFormat');
  Result := FPrimary.format;
end;

function TDirectXConsole.GetName: string;
begin
  Result := 'DirectX';
end;

function TDirectXConsole.GetTitle: string;
begin
  CHECK_OPEN('TDirectXConsole.GetTitle');
  Result := FTitle;
end;

function TDirectXConsole.GetInformation: string;
begin
  CHECK_OPEN('TDirectXConsole.GetInformation');
  Result := FDisplay.information;
end;

procedure TDirectXConsole.internal_open(const _title: string; window: HWND; const mode: TPTCMode; _pages: Integer; exact: Boolean);
var
  _width, _height: Integer;
  _format: TPTCFormat;
begin
  try
    { recycle an already open console }
    internal_recycle(_title, window, mode, _pages, exact);
    exit;
  except
    on TPTCError do
    { could not recycle };
  end;

  { check that the mode is valid }
  if not mode.valid then
    raise TPTCError.Create('invalid mode');

  { get mode information }
  _width := mode.width;
  _height := mode.height;
  _format := mode.format;

  { start internal open }
  internal_open_start(_title, window);

  { check output mode }
  case FOutputMode of
    DEFAULT :
    try
      { start fullscreen open }
      internal_open_fullscreen_start(window, mode, exact);

      { change fullscreen display }
      internal_open_fullscreen_change(mode, exact);

      { setup fullscreen display surfaces }
      internal_open_fullscreen_surface(mode, _pages);

      { finish fullscreen open }
      internal_open_fullscreen_finish;
    except
      on TPTCError do
      begin
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
      end;
    end;
    WINDOWED: begin
      { start windowed open }
      internal_open_windowed_start(window, mode, exact);

      { change windowed display display mode }
      internal_open_windowed_change(mode, exact);

      { setup windowed display }
      internal_open_windowed_surface(mode, _pages);

      { finish windowed open }
      internal_open_windowed_finish;
    end;
    FULLSCREEN: begin
      { start fullscreen open }
      internal_open_fullscreen_start(window, mode, exact);

      { change fullscreen display }
      internal_open_fullscreen_change(mode, exact);

      { setup fullscreen display surfaces }
      internal_open_fullscreen_surface(mode, _pages);

      { finish fullscreen open }
      internal_open_fullscreen_finish;
    end;
  end;

  { finish internal open }
  internal_open_finish;
end;

procedure TDirectXConsole.internal_recycle(const _title: string; window: HWND; const mode: TPTCMode; _pages: Integer; exact: Boolean);
begin
  { Check if the console is open }
  if not FOpen then
    raise TPTCError.Create('cannot recycle because it is not already open');
  if window <> 0 then
  begin
    if (FWindow.handle <> window) or (not (FWindow.managed)) then
      raise TPTCError.Create('cannot recycle with this user window');
  end;
  case FOutputMode of
    DEFAULT :
      if FDisplay.fullscreen then
      begin
        try
          internal_recycle_fullscreen(_title, window, mode, _pages, exact);
        except
          on TPTCError do
            raise TPTCError.Create('recycling fullscreen to windowed is not implemented');
        end;
      end
      else
        raise TPTCError.Create('recycling windowed to fullscreen is not implemented');
    FULLSCREEN: internal_recycle_fullscreen(_title, window, mode, _pages, exact);
    WINDOWED: internal_recycle_fullscreen(_title, window, mode, _pages, exact);
  end;
end;

procedure TDirectXConsole.internal_close;
begin
  FOpen := False;
  FreeAndNil(FMouse);
  FreeAndNil(FKeyboard);
  FreeAndNil(FHook);
  FreeAndNil(FEventQueue);
  if FPrimary <> nil then
    FPrimary.Close;
  if FDisplay <> nil then
    FDisplay.Close;
  FreeAndNil(FWindow);
  if FDisplay <> nil then
    FDisplay.Restore;
end;

procedure TDirectXConsole.internal_shutdown;
begin
  FLibrary.Close;
end;

procedure TDirectXConsole.internal_open_start(const _title: string; window: HWND);
var
  tmp: array [0..1023] of Char;
begin
  { close_down }
  internal_close;

  { check window }
  if window = 0 then
  begin
    FTitle := _title;
  end
  else
  begin
    GetWindowText(window, @tmp, SizeOf(tmp));
    FTitle := PChar2String(@tmp);
  end;
end;

procedure TDirectXConsole.internal_open_finish;
begin
  FreeAndNil(FMouse);
  FreeAndNil(FKeyboard);
  FreeAndNil(FEventQueue);
  FEventQueue := TEventQueue.Create;
  FKeyboard := TWin32Keyboard.Create(FWindow.handle, FWindow.thread, False, FEventQueue);
  FMouse := TWin32Mouse.Create(FWindow.handle, FWindow.thread, False, FEventQueue, FPrimary.Fullscreen, FPrimary.width, FPrimary.height);
  if FPrimary.Fullscreen then
    FMouse.SetWindowArea(0, 0, FDisplay.Mode.Width, FDisplay.Mode.Height);
  FWindow.update;
  FOpen := True;
end;

procedure TDirectXConsole.internal_open_reset;
begin
  FreeAndNil(FMouse);
  FreeAndNil(FKeyboard);
  FreeAndNil(FHook);
  FPrimary.close;
  FreeAndNil(FWindow);
  FreeAndNil(FEventQueue);
  FDisplay.restore;
end;

procedure TDirectXConsole.internal_open_fullscreen_start(window: HWND; const mode: TPTCMode; exact: Boolean);
begin
  FFullscreen := True;

  { test if display mode exists... }
  if not FDisplay.test(mode, exact) then
    raise TPTCError.Create('display mode test failed!');

  { handle cursor show mode }
  if FCursorMode = CURSOR_SHOW then
    FCursor := True
  else
    FCursor := False;

  { save display }
  FDisplay.save;

  { check window }
  if window = 0 then
    FWindow := TWin32Window.Create('PTC_DIRECTX_FULLSCREEN',
                                    FTitle,
                                    WS_EX_TOPMOST,
                                    DWord(WS_POPUP or WS_SYSMENU or WS_VISIBLE), // fpc windows RTL bug - WS_POPUP should be a DWord!!!
                                    SW_NORMAL,
                                    0, 0,
                                    GetSystemMetrics(SM_CXSCREEN),
                                    GetSystemMetrics(SM_CYSCREEN),
                                    False, False)
  else
    FWindow := TWin32Window.Create(window);

  { set window cursor }
  FWindow.Cursor(FCursor);

  if FCursor then
    FWin32Cursor.Show
  else
    FWin32Cursor.Hide;

  { set cooperative level }
  FDisplay.cooperative(FWindow.handle, True);
end;

procedure TDirectXConsole.internal_open_fullscreen_change(const mode: TPTCMode; exact: Boolean);
begin
  FDisplay.open(mode, exact, FFrequency);
  FPrimary.blocking(True);
end;

procedure TDirectXConsole.internal_open_fullscreen_surface(const mode: TPTCMode; _pages: Integer);
var
  primary: Boolean;
  _secondary: Boolean;
  _palette: Boolean;
  complex: Boolean;
begin
  _secondary := (FPrimaryModeFullscreen = SECONDARY) or (not FDisplay.mode.Equals(mode));
  _palette := FDisplay.mode.format.indexed;
  FPrimary.initialize(FWindow, FLibrary.lpDD2);
  complex := True; { Complex = false sucks; according to the DirectX 7 docs, AddAttachedSurface must be used only for Z-buffer surfaces }
  primary := False;

  { todo: log errors }
  if _pages >= 1 then
  try
    FPrimary.primary(_pages, True, True, _palette, complex);
    primary := True;
  except
    on TPTCError Do;
  end;

  if not primary then
  try
    FPrimary.primary(3, True, True, _palette, complex);
    primary := True;
  except
    on TPTCError Do;
  end;

  if not primary then
  try
    FPrimary.primary(2, True, True, _palette, complex);
    primary := True;
  except
    on TPTCError Do;
  end;

  if not Primary then
  try
    if not _secondary then
      FPrimary.primary(2, False, True, _palette, complex)
    else
      FPrimary.primary(1, False, True, _palette, complex);
    primary := True;
  except
    on TPTCError do
      raise TPTCError.Create('Could not create primary surface');
  end;

  if _secondary then
    FPrimary.secondary(mode.width, mode.height);
  if FNearestMode = NEAREST_CENTERING then
    FPrimary.centering(True);
  if FNearestMode = NEAREST_STRETCHING then
    FPrimary.centering(False);
  {
  original primary setup code (1.0.17)
  ...
  }

  FPrimary.synchronize(FSynchronizedUpdate);
end;

procedure TDirectXConsole.internal_open_fullscreen_finish;
begin
  FreeAndNil(FHook);

  { create hook on window }
  FHook := TDirectXHook.Create(Self, FWindow.handle, GetCurrentThreadId, FCursor, FWindow.managed, True);
end;

procedure TDirectXConsole.internal_open_windowed_start(window: HWND; const mode: TPTCMode; exact: Boolean);
var
  extended: Integer;
begin
  FFullscreen := False;
  if FCursorMode = CURSOR_HIDE then
    FCursor := False
  else
    FCursor := True;
  FreeAndNil(FWindow);
  if window <> 0 then
  begin
    FWindow := TWin32Window.Create(window);
  end
  else
  begin
    extended := 0;
    if FPrimaryModeWindowed = DIRECT then
      extended := WS_EX_TOPMOST;
    case FWindowMode of
      RESIZABLE: FWindow := TWin32Window.Create('PTC_DIRECTX_WINDOWED_RESIZABLE', FTitle,
                                              extended, WS_OVERLAPPEDWINDOW or WS_VISIBLE, SW_NORMAL, CW_USEDEFAULT, CW_USEDEFAULT, mode.width, mode.height, FCenterWindow, False);
      FIXED: FWindow := TWin32Window.Create('PTC_DIRECTX_WINDOWED_FIXED', FTitle,
                                              extended, WS_VISIBLE or WS_SYSMENU or WS_CAPTION or WS_MINIMIZEBOX, SW_NORMAL, CW_USEDEFAULT, CW_USEDEFAULT, mode.width, mode.height, FCenterWindow, False);
    end;
  end;
  FWindow.Cursor(FCursor);
  FDisplay.cooperative(FWindow.handle, False);
end;

procedure TDirectXConsole.internal_open_windowed_change(const mode: TPTCMode; exact: Boolean);
begin
  FDisplay.open;
  if FPrimaryModeWindowed = DIRECT then
    FPrimary.blocking(True)
  else
    FPrimary.blocking(False);
end;

procedure TDirectXConsole.internal_open_windowed_surface(const mode: TPTCMode; _pages: Integer);
begin
  FPrimary.initialize(FWindow, FLibrary.lpDD2);
  FPrimary.primary(1, False, False, False, False);
  if FPrimaryModeWindowed = SECONDARY then
    FPrimary.secondary(mode.width, mode.height);
end;

procedure TDirectXConsole.internal_open_windowed_finish;
begin
  FreeAndNil(FHook);

  { create hook on window }
  FHook := TDirectXHook.Create(Self, FWindow.handle, GetCurrentThreadId, FCursor, FWindow.managed, False);
end;

procedure TDirectXConsole.internal_recycle_fullscreen(const _title: string; window: HWND; const mode: TPTCMode; _pages: Integer; exact: Boolean);
begin
  LOG('fullscreen open recycle');
  FPrimary.close;
  internal_open_fullscreen_change(mode, exact);
  internal_open_fullscreen_surface(mode, _pages);
end;

procedure TDirectXConsole.internal_recycle_windowed(const _title: string; window: HWND; const mode: TPTCMode; _pages: Integer; exact: Boolean);
begin
  LOG('windowed open recycle');
  FPrimary.close;
  FWindow.resize(mode.width, mode.height);
  internal_open_windowed_change(mode, exact);
  internal_open_windowed_surface(mode, _pages);
end;

procedure TDirectXConsole.Paint;
begin
  if FLocked or (not FOpen) then
    exit;
  FPrimary.Paint;
end;

procedure TDirectXConsole.UpdateCursor;
begin
  if not FOpen then
    exit;
  if FDisplay.Fullscreen then
    if FCursorMode = CURSOR_SHOW then
      FCursor := True
    else
      FCursor := False
  else
    if FCursorMode = CURSOR_HIDE then
      FCursor := False
    else
      FCursor := True;

  { update hook cursor }
  FHook.Cursor(FCursor);

  { update window cursor }
  FWindow.Cursor(FCursor);

  { hide/show cursor globally if running fullscreen }
  if FDisplay.Fullscreen then
    if FCursor then
      FWin32Cursor.Show
    else
      FWin32Cursor.Hide;
end;

{$IFDEF DEBUG}
procedure TDirectXConsole.CHECK_OPEN(AMsg: String);
begin
  if not FOpen then
  try
    raise TPTCError.Create('console is not open');
  except
    on error: TPTCError do
      raise TPTCError.Create(AMsg, error);
  end;
end;

procedure TDirectXConsole.CHECK_LOCK(AMsg: String);
begin
  if FLocked then
  try
    raise TPTCError.Create('console is locked');
  except
    on error: TPTCError do
      raise TPTCError.Create(AMsg, error);
  end;
end;
{$ENDIF}
