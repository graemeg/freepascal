<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the PTCPas framebuffer library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2007, 2009-2011  Nikolay Nikolov (nickysn@users.sourceforge.net)
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

constructor TWinCEGDIConsole.Create;
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
  inherited Create;

  FDisplayWidth := GetSystemMetrics(SM_CXSCREEN);
  FDisplayHeight := GetSystemMetrics(SM_CYSCREEN);
=======
=======
>>>>>>> origin/fixes_2_2
Constructor TWinCEGDIConsole.Create;

Begin
  Inherited Create;

  FDefaultWidth := 320;
  FDefaultHeight := 200;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FDefaultFormat := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);

  FCopy := TPTCCopy.Create;
  FClear := TPTCClear.Create;
  FArea := TPTCArea.Create;
  FClip := TPTCArea.Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
  SetLength(FModes, 1);
  FModes[0] := TPTCMode.Create(FDisplayWidth, FDisplayHeight, FDefaultFormat);
end;

destructor TWinCEGDIConsole.Destroy;
=======
=======
>>>>>>> graemeg/cpstrnew
  FModes[0] := TPTCMode.Create(FDisplayWidth, FDisplayHeight, FDefaultFormat);
  FModes[1] := TPTCMode.Create;
end;

destructor TWinCEGDIConsole.Destroy;

var
  I: Integer;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

  FModes[0] := TPTCMode.Create(FDisplayWidth, FDisplayHeight, FDefaultFormat);
  FModes[1] := TPTCMode.Create;
end;

destructor TWinCEGDIConsole.Destroy;

var
  I: Integer;

>>>>>>> graemeg/cpstrnew
=======

  FModes[0] := TPTCMode.Create(FDisplayWidth, FDisplayHeight, FDefaultFormat);
  FModes[1] := TPTCMode.Create;
end;

destructor TWinCEGDIConsole.Destroy;

var
  I: Integer;

>>>>>>> origin/cpstrnew
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Destructor TWinCEGDIConsole.Destroy;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Close;

  FWindow.Free;

  FEventQueue.Free;
  FCopy.Free;
  FClear.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  inherited Destroy;
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; APages: Integer = 0);
=======
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FArea.Free;
  FClip.Free;
  FDefaultFormat.Free;

<<<<<<< HEAD
<<<<<<< HEAD
  for I := Low(FModes) to High(FModes) do
    FModes[I].Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew

  inherited Destroy;
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; APages: Integer = 0);

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
begin
  Open(ATitle, FDefaultFormat, APages);
end;

<<<<<<< HEAD
procedure TWinCEGDIConsole.Open(const ATitle: string; AFormat: IPTCFormat;
                                APages: Integer = 0);
begin
  Open(ATitle, FDisplayWidth, FDisplayHeight, AFormat, APages);
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; AMode: IPTCMode;
                                APages: Integer = 0);
begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; AWidth, AHeight: Integer;
                                AFormat: IPTCFormat; APages: Integer = 0);
var
  DeviceContext: HDC;
=======
=======

  inherited Destroy;
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; APages: Integer = 0);

=======

  inherited Destroy;
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; APages: Integer = 0);

>>>>>>> graemeg/cpstrnew
begin
  Open(ATitle, FDefaultFormat, APages);
end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
procedure TWinCEGDIConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
                                APages: Integer = 0);

begin
  Open(ATitle, FDisplayWidth, FDisplayHeight, AFormat, APages);
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; const AMode: TPTCMode;
                                APages: Integer = 0);

begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; AWidth, AHeight: Integer;
                                const AFormat: TPTCFormat; APages: Integer = 0);

var
  DeviceContext: HDC;
  tmp: TPTCArea;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
begin
  Open(ATitle, FDefaultFormat, APages);
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
                                APages: Integer = 0);

begin
  Open(ATitle, FDisplayWidth, FDisplayHeight, AFormat, APages);
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; const AMode: TPTCMode;
                                APages: Integer = 0);

begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
end;

procedure TWinCEGDIConsole.Open(const ATitle: string; AWidth, AHeight: Integer;
                                const AFormat: TPTCFormat; APages: Integer = 0);

var
  DeviceContext: HDC;
  tmp: TPTCArea;

>>>>>>> origin/cpstrnew
begin
  LOG('TWinCEGDIConsole.Open');

  if FBitmap <> 0 then
  begin
    DeleteObject(FBitmap);
    FBitmap := 0;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
  Inherited Destroy;
End;

Procedure TWinCEGDIConsole.Open(Const ATitle : String; APages : Integer = 0);

Begin
  Open(ATitle, FDefaultFormat, APages);
End;

Procedure TWinCEGDIConsole.Open(Const ATitle : String; Const AFormat : TPTCFormat;
                                APages : Integer = 0);

Begin
  Open(ATitle, FDefaultWidth, FDefaultHeight, AFormat, APages);
End;

Procedure TWinCEGDIConsole.Open(Const ATitle : String; Const AMode : TPTCMode;
                                APages : Integer = 0);

Begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
End;

Procedure TWinCEGDIConsole.Open(Const ATitle : String; AWidth, AHeight : Integer;
                                Const AFormat : TPTCFormat; APages : Integer = 0);

Var
  DeviceContext : HDC;
  tmp : TPTCArea;

Begin
  LOG('TWinCEGDIConsole.Open');

  If FBitmap <> 0 Then
  Begin
    DeleteObject(FBitmap);
    FBitmap := 0;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FreeAndNil(FWindow);
  FreeAndNil(FBitmapInfo);
  FreeAndNil(FKeyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);

  LOG('creating window');
  FWindow := TWinCEWindow.Create('PTC_GDI_WINDOWED_FIXED',
                                 ATitle,
                                 0,
<<<<<<< HEAD
<<<<<<< HEAD
                                 WS_VISIBLE {Or WS_SYSMENU or WS_CAPTION},
                                 SW_SHOWNORMAL,
                                 0, 0,
                                 FDisplayWidth, FDisplayHeight,
				 @WndProc);
  LOG('window created successfully');

  FBitmapInfo := TWinCEBitmapInfo.Create(FDisplayWidth, FDisplayHeight);

  LOG('trying to create a dib section');
  DeviceContext := GetDC(FWindow.WindowHandle);
  if DeviceContext = 0 then
    raise TPTCError.Create('could not get device context of window');
=======
=======
>>>>>>> origin/fixes_2_2
                                 WS_VISIBLE {Or WS_SYSMENU Or WS_CAPTION},
                                 SW_SHOWNORMAL,
                                 CW_USEDEFAULT, CW_USEDEFAULT,
                                 AWidth, AHeight,
				 @WndProc);
  LOG('window created successfully');

  FBitmapInfo := TWinCEBitmapInfo.Create(AWidth, AHeight);

  LOG('trying to create a dib section');
  DeviceContext := GetDC(FWindow.WindowHandle);
  If DeviceContext = 0 Then
    Raise TPTCError.Create('could not get device context of window');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FBitmap := CreateDIBSection(DeviceContext,
                              FBitmapInfo.BMI^,
                              DIB_RGB_COLORS,
                              FBitmapPixels,
                              0, 0);
  ReleaseDC(FWindow.WindowHandle, DeviceContext);
<<<<<<< HEAD
<<<<<<< HEAD
  if FBitmap = 0 then
    raise TPTCError.Create('could not create dib section');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FArea := TPTCArea.Create(0, 0, FDisplayWidth, FDisplayHeight);
  FClip := FArea;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  tmp := TPTCArea.Create(0, 0, FDisplayWidth, FDisplayHeight);
  try
    FArea.Assign(tmp);
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

  FEventQueue := TEventQueue.Create;
  FKeyboard := TWinCEKeyboard.Create(FEventQueue);
  FMouse := TWinCEMouse.Create(FEventQueue, False, FDisplayWidth, FDisplayHeight);
=======
=======
>>>>>>> origin/fixes_2_2
  If FBitmap = 0 Then
    Raise TPTCError.Create('could not create dib section');

  tmp := TPTCArea.Create(0, 0, AWidth, AHeight);
  Try
    FArea.Assign(tmp);
    FClip.Assign(tmp);
  Finally
    tmp.Free;
  End;

  FEventQueue := TEventQueue.Create;
  FKeyboard := TWinCEKeyboard.Create(FEventQueue);
  FMouse := TWinCEMouse.Create(FEventQueue, False, AWidth, AHeight);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  FWindow.Update;

  {todo...}
  FOpen := True;
  LOG('console open succeeded');
<<<<<<< HEAD
<<<<<<< HEAD
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGDIConsole.Close;
=======

procedure TWinCEGDIConsole.Close;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGDIConsole.Close;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGDIConsole.Close;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGDIConsole.Close;

>>>>>>> origin/cpstrnew
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWinCEGDIConsole.Close;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  LOG('TWinCEGDIConsole.Close');

  FreeAndNil(FKeyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FEventQueue);

<<<<<<< HEAD
<<<<<<< HEAD
  FBitmapPixels := nil; { just in case... }
  FreeAndNil(FBitmapInfo);
  if FBitmap <> 0 then
  begin
    DeleteObject(FBitmap);
    FBitmap := 0;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
  FBitmapPixels := Nil; { just in case... }
  FreeAndNil(FBitmapInfo);
  If FBitmap <> 0 Then
  Begin
    DeleteObject(FBitmap);
    FBitmap := 0;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FreeAndNil(FWindow);

  {todo...}

  FOpen := False;
<<<<<<< HEAD
<<<<<<< HEAD
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGDIConsole.Copy(ASurface: IPTCSurface);
=======

procedure TWinCEGDIConsole.Copy(ASurface: TPTCBaseSurface);

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGDIConsole.Copy(ASurface: TPTCBaseSurface);

=======

procedure TWinCEGDIConsole.Copy(ASurface: TPTCBaseSurface);

=======

procedure TWinCEGDIConsole.Copy(ASurface: TPTCBaseSurface);

>>>>>>> origin/cpstrnew
begin
  {todo...}
end;

procedure TWinCEGDIConsole.Copy(ASurface: TPTCBaseSurface;
                                const ASource, ADestination: TPTCArea);

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  {todo...}
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TWinCEGDIConsole.Copy(ASurface: TPTCBaseSurface;
                                const ASource, ADestination: TPTCArea);

>>>>>>> graemeg/cpstrnew
begin
  {todo...}
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TWinCEGDIConsole.Copy(ASurface: IPTCSurface;
                                ASource, ADestination: IPTCArea);
=======
procedure TWinCEGDIConsole.Copy(ASurface: TPTCBaseSurface;
                                const ASource, ADestination: TPTCArea);

>>>>>>> graemeg/cpstrnew
begin
  {todo...}
end;

procedure TWinCEGDIConsole.Load(const APixels: Pointer;
                                AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
                                AFormat: IPTCFormat;
                                APalette: IPTCPalette);
var
  console_pixels: Pointer;
=======
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette);
var
  Area_: TPTCArea;
  console_pixels: Pointer;

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGDIConsole.Load(const APixels: Pointer;
                                AWidth, AHeight, APitch: Integer;
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette);
var
  Area_: TPTCArea;
  console_pixels: Pointer;

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGDIConsole.Load(const APixels: Pointer;
                                AWidth, AHeight, APitch: Integer;
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette);
var
  Area_: TPTCArea;
  console_pixels: Pointer;

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGDIConsole.Load(const APixels: Pointer;
                                AWidth, AHeight, APitch: Integer;
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette);
var
  Area_: TPTCArea;
  console_pixels: Pointer;

>>>>>>> origin/cpstrnew
begin
  CheckOpen(    'TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  CheckUnlocked('TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  if Clip.Equals(Area) then
  begin
    try
      console_pixels := Lock;
      try
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWinCEGDIConsole.Copy(Var ASurface : TPTCBaseSurface);

Begin
  {todo...}
End;

Procedure TWinCEGDIConsole.Copy(Var ASurface : TPTCBaseSurface;
                                Const ASource, ADestination : TPTCArea);

Begin
  {todo...}
End;

Procedure TWinCEGDIConsole.Load(Const APixels : Pointer;
                                AWidth, AHeight, APitch : Integer;
                                Const AFormat : TPTCFormat;
                                Const APalette : TPTCPalette);
Var
  Area_ : TPTCArea;
  console_pixels : Pointer;

Begin
  CheckOpen(    'TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  CheckUnlocked('TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, TPTCArea.Create(0, 0, width, height), Area);
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
      Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, Area_, Area);
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

procedure TWinCEGDIConsole.Load(const APixels: Pointer;
                                AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                AFormat: IPTCFormat;
                                APalette: IPTCPalette;
                                ASource, ADestination: IPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: IPTCArea;
begin
  CheckOpen(    'TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  CheckUnlocked('TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  try
    console_pixels := Lock;
    try
      TPTCClipper.Clip(ASource, TPTCArea.Create(0, 0, AWidth, AHeight), clipped_source, ADestination, Clip, clipped_destination);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette;
                                const ASource, ADestination: TPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: TPTCArea;
  tmp: TPTCArea;

begin
  CheckOpen(    'TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  CheckUnlocked('TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  clipped_source := nil;
  clipped_destination := nil;
  try
    console_pixels := Lock;
    try
=======
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette;
                                const ASource, ADestination: TPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: TPTCArea;
  tmp: TPTCArea;

begin
  CheckOpen(    'TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  CheckUnlocked('TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  clipped_source := nil;
  clipped_destination := nil;
  try
    console_pixels := Lock;
    try
>>>>>>> origin/cpstrnew
      clipped_source := TPTCArea.Create;
      clipped_destination := TPTCArea.Create;
      tmp := TPTCArea.Create(0, 0, AWidth, AHeight);
      try
        TPTCClipper.Clip(ASource, tmp, clipped_source, ADestination, Clip, clipped_destination);
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

Procedure TWinCEGDIConsole.Load(Const APixels : Pointer;
                                AWidth, AHeight, APitch : Integer;
                                Const AFormat : TPTCFormat;
                                Const APalette : TPTCPalette;
                                Const ASource, ADestination : TPTCArea);
Var
  console_pixels : Pointer;
  clipped_source, clipped_destination : TPTCArea;
  tmp : TPTCArea;

Begin
  CheckOpen(    'TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  CheckUnlocked('TWinCEGDIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
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
<<<<<<< HEAD
=======
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
    on error: TPTCError do
      raise TPTCError.Create('failed to load pixels to console area', error);
  end;
end;

procedure TWinCEGDIConsole.Save(APixels: Pointer;
                                AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                AFormat: IPTCFormat;
                                APalette: IPTCPalette);
=======
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette);

>>>>>>> graemeg/cpstrnew
=======
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette);

>>>>>>> graemeg/cpstrnew
begin
  {todo...}
end;

procedure TWinCEGDIConsole.Save(APixels: Pointer;
                                AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
                                AFormat: IPTCFormat;
                                APalette: IPTCPalette;
                                ASource, ADestination: IPTCArea);
=======
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette);

>>>>>>> graemeg/cpstrnew
begin
  {todo...}
end;

<<<<<<< HEAD
function TWinCEGDIConsole.Lock: Pointer;
=======
=======
procedure TWinCEGDIConsole.Save(APixels: Pointer;
                                AWidth, AHeight, APitch: Integer;
>>>>>>> graemeg/cpstrnew
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette;
                                const ASource, ADestination: TPTCArea);

=======
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette);

>>>>>>> origin/cpstrnew
begin
  {todo...}
end;

<<<<<<< HEAD
function TWinCEGDIConsole.Lock: Pointer;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette;
                                const ASource, ADestination: TPTCArea);

=======
procedure TWinCEGDIConsole.Save(APixels: Pointer;
                                AWidth, AHeight, APitch: Integer;
                                const AFormat: TPTCFormat;
                                const APalette: TPTCPalette;
                                const ASource, ADestination: TPTCArea);

>>>>>>> origin/cpstrnew
begin
  {todo...}
end;

function TWinCEGDIConsole.Lock: Pointer;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  Result := FBitmapPixels; // todo...
  FLocked := True;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGDIConsole.Unlock;
<<<<<<< HEAD
=======

procedure TWinCEGDIConsole.Unlock;

>>>>>>> origin/cpstrnew
begin
  FLocked := False;
end;

procedure TWinCEGDIConsole.Clear;
<<<<<<< HEAD
=======

>>>>>>> origin/cpstrnew
begin
  {todo...}
end;

<<<<<<< HEAD
procedure TWinCEGDIConsole.Clear(AColor: IPTCColor);
=======
=======

procedure TWinCEGDIConsole.Unlock;
>>>>>>> graemeg/cpstrnew

=======

procedure TWinCEGDIConsole.Unlock;

>>>>>>> graemeg/cpstrnew
begin
  FLocked := False;
end;

procedure TWinCEGDIConsole.Clear;

begin
  {todo...}
end;

procedure TWinCEGDIConsole.Clear(const AColor: TPTCColor);

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
begin
  {todo...}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TWinCEGDIConsole.Clear(AColor: IPTCColor;
                                 AArea: IPTCArea);
begin
  {todo...}
end;

procedure TWinCEGDIConsole.Configure(const AFileName: string);
var
  F: Text;
  S: string;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
procedure TWinCEGDIConsole.Clear(const AColor: TPTCColor;
                                 const AArea: TPTCArea);

begin
  {todo...}
end;

procedure TWinCEGDIConsole.Configure(const AFileName: String);

var
  F: Text;
  S: string;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
begin
=======
procedure TWinCEGDIConsole.Clear(const AColor: TPTCColor);

begin
  {todo...}
end;

procedure TWinCEGDIConsole.Clear(const AColor: TPTCColor;
                                 const AArea: TPTCArea);

begin
  {todo...}
end;

procedure TWinCEGDIConsole.Configure(const AFileName: String);

var
  F: Text;
  S: string;

begin
>>>>>>> origin/cpstrnew
  AssignFile(F, AFileName);
  {$push}{$I-}
  Reset(F);
<<<<<<< HEAD
  {$pop}
=======
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
function TWinCEGDIConsole.Option(const AOption: string): Boolean;
=======
function TWinCEGDIConsole.Option(const AOption: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TWinCEGDIConsole.Option(const AOption: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.Option(const AOption: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.Option(const AOption: String): Boolean;

>>>>>>> origin/cpstrnew
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

Procedure TWinCEGDIConsole.Save(APixels : Pointer;
                                AWidth, AHeight, APitch : Integer;
                                Const AFormat : TPTCFormat;
                                Const APalette : TPTCPalette);

Begin
  {todo...}
End;

Procedure TWinCEGDIConsole.Save(APixels : Pointer;
                                AWidth, AHeight, APitch : Integer;
                                Const AFormat : TPTCFormat;
                                Const APalette : TPTCPalette;
                                Const ASource, ADestination : TPTCArea);

Begin
  {todo...}
End;

Function TWinCEGDIConsole.Lock : Pointer;

Begin
  Result := FBitmapPixels; // todo...
  FLocked := True;
End;

Procedure TWinCEGDIConsole.Unlock;

Begin
  FLocked := False;
End;

Procedure TWinCEGDIConsole.Clear;

Begin
  {todo...}
End;

Procedure TWinCEGDIConsole.Clear(Const AColor : TPTCColor);

Begin
  {todo...}
End;

Procedure TWinCEGDIConsole.Clear(Const AColor : TPTCColor;
                                 Const AArea : TPTCArea);

Begin
  {todo...}
End;

Procedure TWinCEGDIConsole.Configure(Const AFileName : String);

Var
  F : Text;
  S : String;

Begin
  AssignFile(F, AFileName);
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
    Option(S);
  End;
  CloseFile(F);
End;

Function TWinCEGDIConsole.Option(Const AOption : String) : Boolean;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  LOG('console option', AOption);

  // todo...

  Result := FCopy.Option(AOption);
<<<<<<< HEAD
<<<<<<< HEAD
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGDIConsole.Palette(APalette: IPTCPalette);
begin
  {todo...}
end;

procedure TWinCEGDIConsole.Clip(AArea: IPTCArea);
begin
  CheckOpen('TWinCEGDIConsole.Clip(AArea)');

  FClip := TPTCClipper.Clip(AArea, FArea);
end;

function TWinCEGDIConsole.Clip: IPTCArea;
=======

=======

>>>>>>> graemeg/cpstrnew
procedure TWinCEGDIConsole.Palette(const APalette: TPTCPalette);

=======

procedure TWinCEGDIConsole.Palette(const APalette: TPTCPalette);

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGDIConsole.Palette(const APalette: TPTCPalette);

>>>>>>> origin/cpstrnew
begin
  {todo...}
end;

procedure TWinCEGDIConsole.Clip(const AArea: TPTCArea);

var
  tmp: TPTCArea;

begin
  CheckOpen('TWinCEGDIConsole.Clip(AArea)');

  tmp := TPTCClipper.Clip(AArea, FArea);
  try
    FClip.Assign(tmp);
  finally
    tmp.Free;
  end;
end;

function TWinCEGDIConsole.Clip: TPTCArea;

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
  CheckOpen('TWinCEGDIConsole.Clip');
  Result := FClip;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGDIConsole.Palette: IPTCPalette;
=======

function TWinCEGDIConsole.Palette: TPTCPalette;

>>>>>>> origin/cpstrnew
begin
  {todo...}
end;

<<<<<<< HEAD
function TWinCEGDIConsole.Modes: TPTCModeList;
begin
  Result := FModes;
end;

procedure TWinCEGDIConsole.Flush;
=======

function TWinCEGDIConsole.Palette: TPTCPalette;

=======

function TWinCEGDIConsole.Palette: TPTCPalette;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.Palette: TPTCPalette;

>>>>>>> graemeg/cpstrnew
begin
  {todo...}
end;

function TWinCEGDIConsole.Modes: PPTCMode;

begin
  Result := @FModes[0];
end;
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGDIConsole.Flush;

>>>>>>> graemeg/cpstrnew
begin
  {todo...}
end;

procedure TWinCEGDIConsole.Finish;
<<<<<<< HEAD
=======

procedure TWinCEGDIConsole.Flush;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGDIConsole.Flush;

>>>>>>> graemeg/cpstrnew
begin
  {todo...}
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TWinCEGDIConsole.WndProc(Ahwnd: HWND; AuMsg: UINT; AwParam: WPARAM; AlParam: LPARAM): LRESULT;
=======
=======
procedure TWinCEGDIConsole.Finish;
>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGDIConsole.Finish;
>>>>>>> graemeg/cpstrnew

begin
  {todo...}
end;

function TWinCEGDIConsole.WndProc(Ahwnd: HWND; AuMsg: UINT; AwParam: WPARAM; AlParam: LPARAM): LRESULT;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
function TWinCEGDIConsole.Modes: PPTCMode;

begin
  Result := @FModes[0];
end;

procedure TWinCEGDIConsole.Flush;

begin
  {todo...}
end;

procedure TWinCEGDIConsole.Finish;

begin
  {todo...}
end;

function TWinCEGDIConsole.WndProc(Ahwnd: HWND; AuMsg: UINT; AwParam: WPARAM; AlParam: LPARAM): LRESULT;

>>>>>>> origin/cpstrnew
begin
  case AuMsg of
  WM_CLOSE: begin
    LOG('TWinCEGDIConsole.WndProc: WM_CLOSE');
    Halt(0);
  end;
  WM_KEYDOWN, WM_KEYUP: begin
    if FKeyboard <> nil then
      Result := FKeyboard.WndProc(Ahwnd, AuMsg, AwParam, AlParam)
    else
      Result := 0;
    exit;
  end;
  WM_MOUSEMOVE,
  WM_LBUTTONDOWN, WM_LBUTTONUP, WM_LBUTTONDBLCLK,
  WM_MBUTTONDOWN, WM_MBUTTONUP, WM_MBUTTONDBLCLK,
  WM_RBUTTONDOWN, WM_RBUTTONUP, WM_RBUTTONDBLCLK: begin
    if FMouse <> nil then
      Result := FMouse.WndProc(Ahwnd, AuMsg, AwParam, AlParam)
    else
      Result := 0;
    exit;
  end;
  else
    Result := DefWindowProcW(Ahwnd, AuMsg, AwParam, AlParam);
  end;
end;

procedure TWinCEGDIConsole.Update;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var
  ClientRect: RECT;
  DeviceContext, DeviceContext2: HDC;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  ClientRect: RECT;
  DeviceContext, DeviceContext2: HDC;

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
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWinCEGDIConsole.Palette(Const APalette : TPTCPalette);

Begin
  {todo...}
End;

Procedure TWinCEGDIConsole.Clip(Const AArea : TPTCArea);

Var
  tmp : TPTCArea;

Begin
  CheckOpen('TWinCEGDIConsole.Clip(AArea)');

  tmp := TPTCClipper.Clip(AArea, FArea);
  Try
    FClip.Assign(tmp);
  Finally
    tmp.Free;
  End;
End;

Function TWinCEGDIConsole.Clip : TPTCArea;

Begin
  CheckOpen('TWinCEGDIConsole.Clip');
  Result := FClip;
End;

Function TWinCEGDIConsole.Palette : TPTCPalette;

Begin
  {todo...}
End;

Function TWinCEGDIConsole.Modes : PPTCMode;

Begin
  // todo...
  Result := Nil;
End;

Procedure TWinCEGDIConsole.Flush;

Begin
  {todo...}
End;

Procedure TWinCEGDIConsole.Finish;

Begin
  {todo...}
End;

Function TWinCEGDIConsole.WndProc(Ahwnd : HWND; AuMsg : UINT; AwParam : WPARAM; AlParam : LPARAM) : LRESULT;

Begin
  Case AuMsg Of
  WM_CLOSE : Begin
    LOG('TWinCEGDIConsole.WndProc: WM_CLOSE');
    Halt(0);
  End;
  WM_KEYDOWN, WM_KEYUP : Begin
    If FKeyboard <> Nil Then
      Result := FKeyboard.WndProc(Ahwnd, AuMsg, AwParam, AlParam)
    Else
      Result := 0;
    Exit;
  End;
  WM_MOUSEMOVE,
  WM_LBUTTONDOWN, WM_LBUTTONUP, WM_LBUTTONDBLCLK,
  WM_MBUTTONDOWN, WM_MBUTTONUP, WM_MBUTTONDBLCLK,
  WM_RBUTTONDOWN, WM_RBUTTONUP, WM_RBUTTONDBLCLK : Begin
    If FMouse <> Nil Then
      Result := FMouse.WndProc(Ahwnd, AuMsg, AwParam, AlParam)
    Else
      Result := 0;
    Exit;
  End;
  Else
    Result := DefWindowProcW(Ahwnd, AuMsg, AwParam, AlParam);
  End;
End;

Procedure TWinCEGDIConsole.Update;

Var
  ClientRect : RECT;
  DeviceContext, DeviceContext2 : HDC;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckOpen(    'TWinCEGDIConsole.Update');
  CheckUnlocked('TWinCEGDIConsole.Update');

  FWindow.Update;

  DeviceContext := GetDC(FWindow.WindowHandle);

<<<<<<< HEAD
<<<<<<< HEAD
  if DeviceContext <> 0 then
  begin
    if GetClientRect(FWindow.WindowHandle, @ClientRect) then
    begin
      DeviceContext2 := CreateCompatibleDC(DeviceContext);
      if DeviceContext2 <> 0 then
      begin
        SelectObject(DeviceContext2, FBitmap);

        StretchBlt(DeviceContext,
                   0, 0, ClientRect.right, ClientRect.bottom,
                   DeviceContext2,
                   0, 0, FBitmapInfo.Width, FBitmapInfo.Height,
                   SRCCOPY);

        DeleteDC(DeviceContext2);
      end;
    end;

    ReleaseDC(FWindow.WindowHandle, DeviceContext);
  end;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGDIConsole.Update(AArea: IPTCArea);
=======

procedure TWinCEGDIConsole.Update(const AArea: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGDIConsole.Update(const AArea: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGDIConsole.Update(const AArea: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGDIConsole.Update(const AArea: TPTCArea);

>>>>>>> origin/cpstrnew
begin
  {todo...}
  Update;
end;
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
function TWinCEGDIConsole.NextEvent(out AEvent: IPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
=======
function TWinCEGDIConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TWinCEGDIConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGDIConsole.NextEvent');
//  CheckUnlocked('TWinCEGDIConsole.NextEvent');

<<<<<<< HEAD
=======
  FreeAndNil(AEvent);
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
=======
=======
>>>>>>> origin/fixes_2_2
  If DeviceContext <> 0 Then
  Begin
    If GetClientRect(FWindow.WindowHandle, @ClientRect) Then
    Begin
      DeviceContext2 := CreateCompatibleDC(DeviceContext);
      If DeviceContext2 <> 0 Then
      Begin
        SelectObject(DeviceContext2, FBitmap);

	StretchBlt(DeviceContext,
	           0, 0, ClientRect.right, ClientRect.bottom,
		   DeviceContext2,
		   0, 0, FBitmapInfo.Width, FBitmapInfo.Height,
		   SRCCOPY);

        DeleteDC(DeviceContext2);
      End;
    End;

    ReleaseDC(FWindow.WindowHandle, DeviceContext);
  End;
End;

Procedure TWinCEGDIConsole.Update(Const AArea : TPTCArea);

Begin
  {todo...}
  Update;
End;

Function TWinCEGDIConsole.NextEvent(Var AEvent : TPTCEvent; AWait : Boolean; Const AEventMask : TPTCEventMask) : Boolean;

Begin
  CheckOpen('TWinCEGDIConsole.NextEvent');
//  CheckUnlocked('TWinCEGDIConsole.NextEvent');

  FreeAndNil(AEvent);
  Repeat
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    { update window }
    FWindow.Update;

    { try to find an event that matches the EventMask }
    AEvent := FEventQueue.NextEvent(AEventMask);
<<<<<<< HEAD
<<<<<<< HEAD
  until (not AWait) or (AEvent <> Nil);
  Result := AEvent <> nil;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGDIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): IPTCEvent;
=======

function TWinCEGDIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGDIConsole.PeekEvent');
//  CheckUnlocked('TWinCEGDIConsole.PeekEvent');

  repeat
=======
=======
>>>>>>> origin/fixes_2_2
  Until (Not AWait) Or (AEvent <> Nil);
  Result := AEvent <> Nil;
End;

Function TWinCEGDIConsole.PeekEvent(AWait : Boolean; Const AEventMask : TPTCEventMask) : TPTCEvent;

Begin
  CheckOpen('TWinCEGDIConsole.PeekEvent');
//  CheckUnlocked('TWinCEGDIConsole.PeekEvent');

  Repeat
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    { update window }
    FWindow.Update;

    { try to find an event that matches the EventMask }
    Result := FEventQueue.PeekEvent(AEventMask);
<<<<<<< HEAD
<<<<<<< HEAD
  until (not AWait) or (Result <> Nil);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGDIConsole.GetWidth: Integer;
=======

function TWinCEGDIConsole.GetWidth: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetWidth: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetWidth: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetWidth: Integer;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGDIConsole.GetWidth');
  Result := FBitmapInfo.Width;
end;

function TWinCEGDIConsole.GetHeight: Integer;
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
  CheckOpen('TWinCEGDIConsole.GetHeight');
  Result := FBitmapInfo.Height;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGDIConsole.GetPitch: Integer;
=======

function TWinCEGDIConsole.GetPitch: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetPitch: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetPitch: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetPitch: Integer;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGDIConsole.GetPitch');
  Result := FBitmapInfo.Pitch;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGDIConsole.GetFormat: IPTCFormat;
=======

function TWinCEGDIConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetFormat: TPTCFormat;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGDIConsole.GetFormat');
  Result := FBitmapInfo.Format;
end;
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
function TWinCEGDIConsole.GetArea: IPTCArea;
=======
function TWinCEGDIConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TWinCEGDIConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetArea: TPTCArea;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGDIConsole.GetArea');
  Result := FArea;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGDIConsole.GetPages: Integer;
=======

function TWinCEGDIConsole.GetPages: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetPages: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetPages: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetPages: Integer;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGDIConsole.GetPages');
  Result := 2;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGDIConsole.GetName: string;
=======

function TWinCEGDIConsole.GetName: string;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetName: string;

>>>>>>> graemeg/cpstrnew
begin
  Result := 'WinCE';
end;

function TWinCEGDIConsole.GetTitle: string;
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetName: string;

begin
  Result := 'WinCE';
end;

function TWinCEGDIConsole.GetTitle: string;

>>>>>>> graemeg/cpstrnew
begin
  CheckOpen('TWinCEGDIConsole.GetTitle');
  Result := FTitle;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGDIConsole.GetInformation: string;
=======

function TWinCEGDIConsole.GetInformation: string;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetInformation: string;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetInformation: string;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGDIConsole.GetName: string;

begin
  Result := 'WinCE';
end;

function TWinCEGDIConsole.GetTitle: string;

begin
  CheckOpen('TWinCEGDIConsole.GetTitle');
  Result := FTitle;
end;

function TWinCEGDIConsole.GetInformation: string;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGDIConsole.GetInformation');
  Result := ''; // todo...
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TWinCEGDIConsole.CheckOpen(AMessage: string);
=======
procedure TWinCEGDIConsole.CheckOpen(AMessage: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGDIConsole.CheckOpen(AMessage: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGDIConsole.CheckOpen(AMessage: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGDIConsole.CheckOpen(AMessage: String);

>>>>>>> origin/cpstrnew
begin
  if not FOpen then
  try
    raise TPTCError.Create('console is not open');
  except
    on error: TPTCError do
      raise TPTCError.Create(AMessage, error);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TWinCEGDIConsole.CheckUnlocked(AMessage: string);
=======
procedure TWinCEGDIConsole.CheckUnlocked(AMessage: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGDIConsole.CheckUnlocked(AMessage: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGDIConsole.CheckUnlocked(AMessage: String);

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGDIConsole.CheckUnlocked(AMessage: String);

>>>>>>> origin/cpstrnew
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
  Until (Not AWait) Or (Result <> Nil);
End;

Function TWinCEGDIConsole.GetWidth : Integer;

Begin
  CheckOpen('TWinCEGDIConsole.GetWidth');
  Result := FBitmapInfo.Width;
End;

Function TWinCEGDIConsole.GetHeight : Integer;

Begin
  CheckOpen('TWinCEGDIConsole.GetHeight');
  Result := FBitmapInfo.Height;
End;

Function TWinCEGDIConsole.GetPitch : Integer;

Begin
  CheckOpen('TWinCEGDIConsole.GetPitch');
  Result := FBitmapInfo.Pitch;
End;

Function TWinCEGDIConsole.GetFormat : TPTCFormat;

Begin
  CheckOpen('TWinCEGDIConsole.GetFormat');
  Result := FBitmapInfo.Format;
End;

Function TWinCEGDIConsole.GetArea : TPTCArea;

Begin
  CheckOpen('TWinCEGDIConsole.GetArea');
  Result := FArea;
End;

Function TWinCEGDIConsole.GetPages : Integer;

Begin
  CheckOpen('TWinCEGDIConsole.GetPages');
  Result := 2;
End;

Function TWinCEGDIConsole.GetName : String;

Begin
  Result := 'WinCE';
End;

Function TWinCEGDIConsole.GetTitle : String;

Begin
  CheckOpen('TWinCEGDIConsole.GetTitle');
  Result := FTitle;
End;

Function TWinCEGDIConsole.GetInformation : String;

Begin
  CheckOpen('TWinCEGDIConsole.GetInformation');
  Result := ''; // todo...
End;

Procedure TWinCEGDIConsole.CheckOpen(AMessage : String);

Begin
  If Not FOpen Then
  Try
    Raise TPTCError.Create('console is not open');
  Except
    On error : TPTCError Do
      Raise TPTCError.Create(AMessage, error);
  End;
End;

Procedure TWinCEGDIConsole.CheckUnlocked(AMessage : String);

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
