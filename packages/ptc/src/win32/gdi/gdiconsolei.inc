<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the PTCPas framebuffer library
    Copyright (C) 2001-2010 Nikolay Nikolov (nickysn@users.sourceforge.net)

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

constructor TGDIConsole.Create;
begin
  inherited Create;

  FCursorMode := CURSOR_DEFAULT;

  FDisplayWidth := GetSystemMetrics(SM_CXSCREEN);
  FDisplayHeight := GetSystemMetrics(SM_CYSCREEN);
=======
=======
>>>>>>> origin/fixes_2_2
Constructor TGDIConsole.Create;

Begin
  Inherited Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  FDefaultWidth := 320;
  FDefaultHeight := 200;
  FDefaultFormat := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

  FCopy := TPTCCopy.Create;
  FClear := TPTCClear.Create;
  FArea := TPTCArea.Create;
  FClip := TPTCArea.Create;
  FPalette := TPTCPalette.Create;

  FOpen := False;

<<<<<<< HEAD
<<<<<<< HEAD
  FModes[0] := TPTCMode.Create(FDisplayWidth, FDisplayHeight, FDefaultFormat);
  FModes[1] := TPTCMode.Create;

  { configure console }
  Configure('ptcpas.cfg');
end;

destructor TGDIConsole.Destroy;
var
  I: Integer;
begin
=======
=======
>>>>>>> origin/fixes_2_2
  { configure console }
  Configure('ptcpas.cfg');
End;

Destructor TGDIConsole.Destroy;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Close;

  {...}

  FWin32DIB.Free;
  FWindow.Free;
  FPalette.Free;
  FEventQueue.Free;
  FCopy.Free;
  FClear.Free;
  FArea.Free;
  FClip.Free;
  FDefaultFormat.Free;

<<<<<<< HEAD
<<<<<<< HEAD
  for I := Low(FModes) to High(FModes) do
    FModes[I].Free;

  inherited Destroy;
end;

procedure TGDIConsole.Open(const ATitle: string; APages: Integer = 0);
begin
  Open(ATitle, FDefaultFormat, APages);
end;

procedure TGDIConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
               APages: Integer = 0);
begin
  Open(ATitle, FDefaultWidth, FDefaultHeight, AFormat, APages);
end;

procedure TGDIConsole.Open(const ATitle: string; const AMode: TPTCMode;
                           APages: Integer = 0);
begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
end;

procedure TGDIConsole.Open(const ATitle: string; AWidth, AHeight: Integer;
               const AFormat: TPTCFormat; APages: Integer = 0);
var
  tmp: TPTCArea;
begin
  if FOpen then
=======
=======
>>>>>>> origin/fixes_2_2
  Inherited Destroy;
End;

Procedure TGDIConsole.Open(Const ATitle : String; APages : Integer = 0);

Begin
  Open(ATitle, FDefaultFormat, APages);
End;

Procedure TGDIConsole.Open(Const ATitle : String; Const AFormat : TPTCFormat;
               APages : Integer = 0);

Begin
  Open(ATitle, FDefaultWidth, FDefaultHeight, AFormat, APages);
End;

Procedure TGDIConsole.Open(Const ATitle : String; Const AMode : TPTCMode;
                           APages : Integer = 0);

Begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
End;

Procedure TGDIConsole.Open(Const ATitle : String; AWidth, AHeight : Integer;
               Const AFormat : TPTCFormat; APages : Integer = 0);

Var
  tmp : TPTCArea;

Begin
  If FOpen Then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Close;

(*  FWindow := TWin32Window.Create('PTC_GDI_FULLSCREEN',
                                 ATitle,
                                 WS_EX_TOPMOST,
<<<<<<< HEAD
<<<<<<< HEAD
                                 DWord(WS_POPUP or WS_SYSMENU or WS_VISIBLE), // fpc windows RTL bug - WS_POPUP should be a DWord!!!
=======
                                 DWord(WS_POPUP Or WS_SYSMENU Or WS_VISIBLE), // fpc windows RTL bug - WS_POPUP should be a DWord!!!
>>>>>>> graemeg/fixes_2_2
=======
                                 DWord(WS_POPUP Or WS_SYSMENU Or WS_VISIBLE), // fpc windows RTL bug - WS_POPUP should be a DWord!!!
>>>>>>> origin/fixes_2_2
                                 SW_NORMAL,
                                 0, 0,
                                 GetSystemMetrics(SM_CXSCREEN),
                                 GetSystemMetrics(SM_CYSCREEN),
                                 False, False);*)

  FWindow := TWin32Window.Create('PTC_GDI_WINDOWED_FIXED',
                                 ATitle,
                                 0,
<<<<<<< HEAD
<<<<<<< HEAD
                                 WS_VISIBLE or WS_SYSMENU or WS_CAPTION or WS_MINIMIZEBOX,
=======
                                 WS_VISIBLE Or WS_SYSMENU Or WS_CAPTION Or WS_MINIMIZEBOX,
>>>>>>> graemeg/fixes_2_2
=======
                                 WS_VISIBLE Or WS_SYSMENU Or WS_CAPTION Or WS_MINIMIZEBOX,
>>>>>>> origin/fixes_2_2
                                 SW_NORMAL,
                                 CW_USEDEFAULT, CW_USEDEFAULT,
                                 AWidth, AHeight,
                                 {m_center_window}False,
                                 False);

(*  FWindow := TWin32Window.Create('PTC_GDI_WINDOWED_RESIZABLE',
                                 ATitle,
                                 0,
<<<<<<< HEAD
<<<<<<< HEAD
                                 WS_OVERLAPPEDWINDOW or WS_VISIBLE,
=======
                                 WS_OVERLAPPEDWINDOW Or WS_VISIBLE,
>>>>>>> graemeg/fixes_2_2
=======
                                 WS_OVERLAPPEDWINDOW Or WS_VISIBLE,
>>>>>>> origin/fixes_2_2
                                 SW_NORMAL,
                                 CW_USEDEFAULT, CW_USEDEFAULT,
                                 AWidth, AHeight,
                                 {m_center_window}False,
                                 False);*)

  FWin32DIB := TWin32DIB.Create(AWidth, AHeight);

  FreeAndNil(FKeyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);
  FEventQueue := TEventQueue.Create;
  FKeyboard := TWin32Keyboard.Create(FWindow.Handle, FWindow.Thread, False, FEventQueue);
  FMouse := TWin32Mouse.Create(FWindow.Handle, FWindow.Thread, False, FEventQueue, {FFullScreen}False, AWidth, AHeight);

  tmp := TPTCArea.Create(0, 0, AWidth, AHeight);
<<<<<<< HEAD
<<<<<<< HEAD
  try
    FArea.Assign(tmp);
    FClip.Assign(tmp);
  finally
    tmp.Free;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
  Try
    FArea.Assign(tmp);
    FClip.Assign(tmp);
  Finally
    tmp.Free;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  FWindow.Update;

  FTitle := ATitle;

  FOpen := True;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TGDIConsole.Close;
begin
  if not FOpen then
    exit;
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TGDIConsole.Close;

Begin
  If Not FOpen Then
    Exit;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  {...}

  FreeAndNil(FKeyboard);
  FreeAndNil(FMouse);

  FreeAndNil(FWin32DIB);
  FreeAndNil(FWindow);

  FreeAndNil(FEventQueue);

  FTitle := '';

  FOpen := False;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TGDIConsole.Copy(ASurface: TPTCBaseSurface);
begin
  // todo...
end;

procedure TGDIConsole.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
begin
  // todo...
end;

procedure TGDIConsole.Load(const APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
var
  Area_: TPTCArea;
  console_pixels: Pointer;
begin
  CheckOpen(    'TGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  CheckUnlocked('TGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  if Clip.Equals(Area) then
  begin
    try
      console_pixels := Lock;
      try
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TGDIConsole.Copy(Var ASurface : TPTCBaseSurface);

Begin
  // todo...
End;

Procedure TGDIConsole.Copy(Var ASurface : TPTCBaseSurface;
                           Const ASource, ADestination : TPTCArea);

Begin
  // todo...
End;

Procedure TGDIConsole.Load(Const APixels : Pointer;
                           AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat;
                           Const APalette : TPTCPalette);
Var
  Area_ : TPTCArea;
  console_pixels : Pointer;

Begin
  CheckOpen(    'TGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  CheckUnlocked('TGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  If Clip.Equals(Area) Then
  Begin
    Try
      console_pixels := Lock;
      Try
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        FCopy.Request(AFormat, Format);
        FCopy.Palette(APalette, Palette);
        FCopy.Copy(APixels, 0, 0, AWidth, AHeight, APitch, console_pixels, 0, 0,
                    Width, Height, Pitch);
<<<<<<< HEAD
<<<<<<< HEAD
      finally
        Unlock;
      end;
    except
      on error: TPTCError do
        raise TPTCError.Create('failed to load pixels to console', error);
    end;
  end
  else
  begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    try
      Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, Area_, Area);
    finally
      Area_.Free;
    end;
  end;
end;

procedure TGDIConsole.Load(const APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: TPTCArea;
  tmp: TPTCArea;
begin
  CheckOpen(    'TGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  CheckUnlocked('TGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  clipped_source := nil;
  clipped_destination := nil;
  try
    console_pixels := Lock;
    try
      clipped_source := TPTCArea.Create;
      clipped_destination := TPTCArea.Create;
      tmp := TPTCArea.Create(0, 0, AWidth, AHeight);
      try
        TPTCClipper.Clip(ASource, tmp, clipped_source, ADestination, Clip, clipped_destination);
      finally
        tmp.Free;
      end;
=======
=======
>>>>>>> origin/fixes_2_2
      Finally
        Unlock;
      End;
    Except
      On error : TPTCError Do
        Raise TPTCError.Create('failed to load pixels to console', error);
    End;
  End
  Else
  Begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    Try
      Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, Area_, Area);
    Finally
      Area_.Free;
    End;
  End;
End;

Procedure TGDIConsole.Load(Const APixels : Pointer;
                           AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat;
                           Const APalette : TPTCPalette;
                           Const ASource, ADestination : TPTCArea);
Var
  console_pixels : Pointer;
  clipped_source, clipped_destination : TPTCArea;
  tmp : TPTCArea;

Begin
  CheckOpen(    'TGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  CheckUnlocked('TGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  clipped_source := Nil;
  clipped_destination := Nil;
  Try
    console_pixels := Lock;
    Try
      clipped_source := TPTCArea.Create;
      clipped_destination := TPTCArea.Create;
      tmp := TPTCArea.Create(0, 0, AWidth, AHeight);
      Try
        TPTCClipper.Clip(ASource, tmp, clipped_source, ADestination, Clip, clipped_destination);
      Finally
        tmp.Free;
      End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      FCopy.request(AFormat, Format);
      FCopy.palette(APalette, Palette);
      FCopy.copy(APixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, APitch,
                 console_pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, Pitch);
<<<<<<< HEAD
<<<<<<< HEAD
    finally
      Unlock;
      clipped_source.Free;
      clipped_destination.Free;
    end;
  except
    on error: TPTCError do
      raise TPTCError.Create('failed to load pixels to console area', error);
  end;
end;

procedure TGDIConsole.Save(APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
begin
  // todo...
end;

procedure TGDIConsole.Save(APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
begin
  // todo...
end;

function TGDIConsole.Lock: Pointer;
begin
  Result := FWin32DIB.Pixels; // todo...
  FLocked := True;
end;

procedure TGDIConsole.Unlock;
begin
  FLocked := False;
end;

procedure TGDIConsole.UpdateCursor;
begin
  if not FOpen then
    exit;

  case FCursorMode of
    CURSOR_DEFAULT:
      FCursor := {Not FFullScreen}True;
    CURSOR_SHOW:
      FCursor := True;
    CURSOR_HIDE:
      FCursor := False;
  end;

  { update window cursor }
  FWindow.Cursor(FCursor);

  { hide/show cursor globally if running fullscreen }
{  if FFullscreen then
    if FCursor then
      Win32Cursor_resurrect
    else
      Win32Cursor_kill;}
end;

procedure TGDIConsole.Clear;
begin
  // todo...
end;

procedure TGDIConsole.Clear(const AColor: TPTCColor);
begin
  // todo...
end;

procedure TGDIConsole.Clear(const AColor: TPTCColor;
                            const AArea: TPTCArea);
begin
  // todo...
end;

procedure TGDIConsole.Configure(const AFileName: String);
var
  F: TextFile;
  S: string;
begin
=======
=======
>>>>>>> origin/fixes_2_2
    Finally
      Unlock;
      clipped_source.Free;
      clipped_destination.Free;
    End;
  Except
    On error : TPTCError Do
      Raise TPTCError.Create('failed to load pixels to console area', error);
  End;
End;

Procedure TGDIConsole.Save(APixels : Pointer;
                           AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat;
                           Const APalette : TPTCPalette);

Begin
  // todo...
End;

Procedure TGDIConsole.Save(APixels : Pointer;
                           AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat;
                           Const APalette : TPTCPalette;
                           Const ASource, ADestination : TPTCArea);

Begin
  // todo...
End;

Function TGDIConsole.Lock : Pointer;

Begin
  Result := FWin32DIB.Pixels; // todo...
  FLocked := True;
End;

Procedure TGDIConsole.Unlock;

Begin
  FLocked := False;
End;

Procedure TGDIConsole.Clear;

Begin
  // todo...
End;

Procedure TGDIConsole.Clear(Const AColor : TPTCColor);

Begin
  // todo...
End;

Procedure TGDIConsole.Clear(Const AColor : TPTCColor;
                            Const AArea : TPTCArea);

Begin
  // todo...
End;

Procedure TGDIConsole.Configure(Const AFileName : String);

Var
  F : Text;
  S : String;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  AssignFile(F, AFileName);
  {$I-}
  Reset(F);
  {$I+}
<<<<<<< HEAD
<<<<<<< HEAD
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

function TGDIConsole.Option(const AOption: String): Boolean;
begin
  LOG('console option', AOption);
  Result := True;
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
    FWindow.ConfineCursor(true);
    exit;
  end;
  if AOption = 'ungrab mouse' then
  begin
    FWindow.ConfineCursor(false);
    exit;
  end;

  Result := FCopy.Option(AOption);
end;

procedure TGDIConsole.Palette(const APalette: TPTCPalette);
begin
  // todo...
end;

procedure TGDIConsole.Clip(const AArea: TPTCArea);
var
  tmp: TPTCArea;
begin
  CheckOpen('TGDIConsole.Clip(AArea)');

  tmp := TPTCClipper.Clip(AArea, FArea);
  try
    FClip.Assign(tmp);
  finally
    tmp.Free;
  end;
end;

function TGDIConsole.Clip: TPTCArea;
begin
  CheckOpen('TGDIConsole.Clip');
  Result := FClip;
end;

function TGDIConsole.Palette: TPTCPalette;
begin
  CheckOpen('TGDIConsole.Palette');
  Result := FPalette;
end;

function TGDIConsole.Modes: PPTCMode;
begin
  Result := @FModes[0];
end;

procedure TGDIConsole.Flush;
begin
=======
=======
>>>>>>> origin/fixes_2_2
  If IOResult <> 0 Then
    Exit;
  While Not EoF(F) Do
  Begin
    {$I-}
    Readln(F, S);
    {$I+}
    If IOResult <> 0 Then
      Break;
    Option(S);
  End;
  CloseFile(F);
End;

Function TGDIConsole.Option(Const AOption : String) : Boolean;

Begin
  // todo...

  Result := FCopy.Option(AOption);
End;

Procedure TGDIConsole.Palette(Const APalette : TPTCPalette);

Begin
  // todo...
End;

Procedure TGDIConsole.Clip(Const AArea : TPTCArea);

Var
  tmp : TPTCArea;

Begin
  CheckOpen('TGDIConsole.Clip(AArea)');

  tmp := TPTCClipper.Clip(AArea, FArea);
  Try
    FClip.Assign(tmp);
  Finally
    tmp.Free;
  End;
End;

Function TGDIConsole.Clip : TPTCArea;

Begin
  CheckOpen('TGDIConsole.Clip');
  Result := FClip;
End;

Function TGDIConsole.Palette : TPTCPalette;

Begin
  CheckOpen('TGDIConsole.Palette');
  Result := FPalette;
End;

Function TGDIConsole.Modes : PPTCMode;

Begin
  // todo...
  Result := Nil;
End;

Procedure TGDIConsole.Flush;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckOpen(    'TGDIConsole.Flush');
  CheckUnlocked('TGDIConsole.Flush');

  // todo...
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TGDIConsole.Finish;
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TGDIConsole.Finish;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckOpen(    'TGDIConsole.Finish');
  CheckUnlocked('TGDIConsole.Finish');

  // todo...
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TGDIConsole.Update;
var
  ClientRect: RECT;
  DeviceContext: HDC;
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TGDIConsole.Update;

Var
  ClientRect : RECT;
  DeviceContext : HDC;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckOpen(    'TGDIConsole.Update');
  CheckUnlocked('TGDIConsole.Update');

  FWindow.Update;

<<<<<<< HEAD
<<<<<<< HEAD
  DeviceContext := GetDC(FWindow.Handle);

  if DeviceContext <> 0 then
  begin
    if GetClientRect(FWindow.Handle, @ClientRect) then
    begin
=======
=======
>>>>>>> origin/fixes_2_2
  DeviceContext := GetDC(FWindow.m_window);

  If DeviceContext <> 0 Then
  Begin
    If GetClientRect(FWindow.m_window, @ClientRect) Then
    Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      StretchDIBits(DeviceContext,
                    0, 0, ClientRect.right, ClientRect.bottom,
                    0, 0, FWin32DIB.Width, FWin32DIB.Height,
                    FWin32DIB.Pixels,
                    FWin32DIB.BMI^,
                    DIB_RGB_COLORS,
                    SRCCOPY);
<<<<<<< HEAD
<<<<<<< HEAD
    end;

    ReleaseDC(FWindow.Handle, DeviceContext);
  end;
end;

procedure TGDIConsole.Update(const AArea: TPTCArea);
begin
  Update;
end;

function TGDIConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
var
  UseGetMessage: Boolean;
begin
=======
=======
>>>>>>> origin/fixes_2_2
    End;

    ReleaseDC(FWindow.m_window, DeviceContext);
  End;
End;

Procedure TGDIConsole.Update(Const AArea : TPTCArea);

Begin
  Update;
End;

Function TGDIConsole.NextEvent(Var AEvent : TPTCEvent; AWait : Boolean; Const AEventMask : TPTCEventMask) : Boolean;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckOpen('TGDIConsole.NextEvent');
//  CheckUnlocked('TGDIConsole.NextEvent');

  FreeAndNil(AEvent);
<<<<<<< HEAD
<<<<<<< HEAD
  UseGetMessage := False;
  repeat
    { update window }
    FWindow.Update(False, UseGetMessage);

    { try to find an event that matches the EventMask }
    AEvent := FEventQueue.NextEvent(AEventMask);
    if AWait then
      UseGetMessage := True;
  until (not AWait) or (AEvent <> Nil);
  Result := AEvent <> nil;
end;

function TGDIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
var
  UseGetMessage: Boolean;
begin
  CheckOpen('TGDIConsole.PeekEvent');
//  CheckUnlocked('TGDIConsole.PeekEvent');

  UseGetMessage := False;
  repeat
    { update window }
    FWindow.Update(False, UseGetMessage);

    { try to find an event that matches the EventMask }
    Result := FEventQueue.PeekEvent(AEventMask);
    if AWait then
      UseGetMessage := True;
  until (not AWait) or (Result <> Nil);
end;

function TGDIConsole.GetWidth: Integer;
begin
  CheckOpen('TGDIConsole.GetWidth');
  Result := FWin32DIB.Width;
end;

function TGDIConsole.GetHeight: Integer;
begin
  CheckOpen('TGDIConsole.GetHeight');
  Result := FWin32DIB.Height;
end;

function TGDIConsole.GetPitch: Integer;
begin
  CheckOpen('TGDIConsole.GetPitch');
  Result := FWin32DIB.Pitch;
end;

function TGDIConsole.GetArea: TPTCArea;
begin
  CheckOpen('TGDIConsole.GetArea');
  Result := FArea;
end;

function TGDIConsole.GetFormat: TPTCFormat;
begin
  CheckOpen('TGDIConsole.GetFormat');
  Result := FWin32DIB.Format;
end;

function TGDIConsole.GetPages: Integer;
begin
  CheckOpen('TGDIConsole.GetPages');
  Result := 2;
end;

function TGDIConsole.GetName: string;
begin
  Result := 'GDI';
end;

function TGDIConsole.GetTitle: string;
begin
  CheckOpen('TGDIConsole.GetTitle');
  Result := FTitle;
end;

function TGDIConsole.GetInformation: string;
begin
  CheckOpen('TGDIConsole.GetInformation');
  Result := ''; // todo...
end;

procedure TGDIConsole.CheckOpen(const AMessage: String);
begin
  if not FOpen then
  try
    raise TPTCError.Create('console is not open');
  except
    on error: TPTCError do
      raise TPTCError.Create(AMessage, error);
  end;
end;

procedure TGDIConsole.CheckUnlocked(const AMessage: String);
begin
  if FLocked then
  try
    raise TPTCError.Create('console is locked');
  except
    on error: TPTCError do
      raise TPTCError.Create(AMessage, error);
  end;
end;
=======
=======
>>>>>>> origin/fixes_2_2
  Repeat
    { update window }
    FWindow.Update;

    { try to find an event that matches the EventMask }
    AEvent := FEventQueue.NextEvent(AEventMask);
  Until (Not AWait) Or (AEvent <> Nil);
  Result := AEvent <> Nil;
End;

Function TGDIConsole.PeekEvent(AWait : Boolean; Const AEventMask : TPTCEventMask) : TPTCEvent;

Begin
  CheckOpen('TGDIConsole.PeekEvent');
//  CheckUnlocked('TGDIConsole.PeekEvent');

  Repeat
    { update window }
    FWindow.Update;

    { try to find an event that matches the EventMask }
    Result := FEventQueue.PeekEvent(AEventMask);
  Until (Not AWait) Or (Result <> Nil);
End;

Function TGDIConsole.GetWidth : Integer;

Begin
  CheckOpen('TGDIConsole.GetWidth');
  Result := FWin32DIB.Width;
End;

Function TGDIConsole.GetHeight : Integer;

Begin
  CheckOpen('TGDIConsole.GetHeight');
  Result := FWin32DIB.Height;
End;

Function TGDIConsole.GetPitch : Integer;

Begin
  CheckOpen('TGDIConsole.GetPitch');
  Result := FWin32DIB.Pitch;
End;

Function TGDIConsole.GetArea : TPTCArea;

Begin
  CheckOpen('TGDIConsole.GetArea');
  Result := FArea;
End;

Function TGDIConsole.GetFormat : TPTCFormat;

Begin
  CheckOpen('TGDIConsole.GetFormat');
  Result := FWin32DIB.Format;
End;

Function TGDIConsole.GetPages : Integer;

Begin
  CheckOpen('TGDIConsole.GetPages');
  Result := 2;
End;

Function TGDIConsole.GetName : String;

Begin
  Result := 'GDI';
End;

Function TGDIConsole.GetTitle : String;

Begin
  CheckOpen('TGDIConsole.GetTitle');
  Result := FTitle;
End;

Function TGDIConsole.GetInformation : String;

Begin
  CheckOpen('TGDIConsole.GetInformation');
  Result := ''; // todo...
End;

Procedure TGDIConsole.CheckOpen(AMessage : String);

Begin
  If Not FOpen Then
  Try
    Raise TPTCError.Create('console is not open');
  Except
    On error : TPTCError Do
      Raise TPTCError.Create(AMessage, error);
  End;
End;

Procedure TGDIConsole.CheckUnlocked(AMessage : String);

Begin
  If FLocked Then
  Try
    Raise TPTCError.Create('console is locked');
  Except
    On error : TPTCError Do
      Raise TPTCError.Create(AMessage, error);
  End;
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
