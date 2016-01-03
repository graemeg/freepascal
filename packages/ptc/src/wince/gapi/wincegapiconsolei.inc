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

constructor TWinCEGAPIConsole.Create;
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
=======
=======
>>>>>>> origin/fixes_2_2
Constructor TWinCEGAPIConsole.Create;

Begin
  Inherited Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  FCopy := TPTCCopy.Create;
  FClear := TPTCClear.Create;
  FArea := TPTCArea.Create;
  FClip := TPTCArea.Create;

  LOG('getting display properties');
  FGXDisplayProperties := GXGetDisplayProperties;
  LOG('width='  + IntToStr(FGXDisplayProperties.cxWidth ));
  LOG('height=' + IntToStr(FGXDisplayProperties.cyHeight));
  LOG('xpitch=' + IntToStr(FGXDisplayProperties.cbxPitch));
  LOG('ypitch=' + IntToStr(FGXDisplayProperties.cbyPitch));
  LOG('BPP='    + IntToStr(FGXDisplayProperties.cBPP    ));
  LOG('format=' + IntToStr(FGXDisplayProperties.ffFormat));

  FDisplayWidth := FGXDisplayProperties.cxWidth;
  FDisplayHeight := FGXDisplayProperties.cyHeight;
  FDisplayPitch := FGXDisplayProperties.cbyPitch;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  if (FGXDisplayProperties.ffFormat and kfDirect565) <> 0 then
    FDisplayFormat := TPTCFormat.Create(FGXDisplayProperties.cBPP,
                                        $F800, $07E0, $001F);

  if (FGXDisplayProperties.ffFormat and kfDirect888) <> 0 then
    FDisplayFormat := TPTCFormat.Create(FGXDisplayProperties.cBPP,
                                        $FF0000, $FF00, $FF);

  if (FGXDisplayProperties.ffFormat and kfDirect555) <> 0 then
    FDisplayFormat := TPTCFormat.Create(FGXDisplayProperties.cBPP,
                                        $7C00, $03E0, $001F);

  if (FGXDisplayProperties.ffFormat and kfDirect444) <> 0 then
    FDisplayFormat := TPTCFormat.Create(FGXDisplayProperties.cBPP,
                                        $0F00, $00F0, $000F);

  if FDisplayFormat = nil then
    raise TPTCError.Create('GAPI: Unknown/unsupported pixel format');

  SetLength(FModes, 1);
  FModes[0] := TPTCMode.Create(FDisplayWidth, FDisplayHeight, FDisplayFormat);
end;

destructor TWinCEGAPIConsole.Destroy;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FDisplayFormat := nil;

  if (FGXDisplayProperties.ffFormat and kfDirect565) <> 0 then
    FDisplayFormat := TPTCFormat.Create(FGXDisplayProperties.cBPP,
                                        $F800, $07E0, $001F);

  if (FGXDisplayProperties.ffFormat and kfDirect888) <> 0 then
    FDisplayFormat := TPTCFormat.Create(FGXDisplayProperties.cBPP,
                                        $FF0000, $FF00, $FF);

  if (FGXDisplayProperties.ffFormat and kfDirect555) <> 0 then
    FDisplayFormat := TPTCFormat.Create(FGXDisplayProperties.cBPP,
                                        $7C00, $03E0, $001F);

  if (FGXDisplayProperties.ffFormat and kfDirect444) <> 0 then
    FDisplayFormat := TPTCFormat.Create(FGXDisplayProperties.cBPP,
                                        $0F00, $00F0, $000F);

  if FDisplayFormat = nil then
    raise TPTCError.Create('GAPI: Unknown/unsupported pixel format');

  FModes[0] := TPTCMode.Create(FDisplayWidth, FDisplayHeight, FDisplayFormat);
  FModes[1] := TPTCMode.Create;
end;

destructor TWinCEGAPIConsole.Destroy;

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
=======
=======
>>>>>>> origin/fixes_2_2
//  FDisplayFormat := TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF);
  FDisplayFormat := TPTCFormat.Create(16, $F800, $07E0, $001F); {hardcoded for now...}

  FModes[0] := TPTCMode.Create(FDisplayWidth, FDisplayHeight, FDisplayFormat);
  FModes[1] := TPTCMode.Create;
End;

Destructor TWinCEGAPIConsole.Destroy;

Var
  I : Integer;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Close;

  FCopy.Free;
  FClear.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FArea.Free;
  FClip.Free;
  FDisplayFormat.Free;

<<<<<<< HEAD
<<<<<<< HEAD
  for I := Low(FModes) to High(FModes) do
    FModes[I].Free;
>>>>>>> graemeg/cpstrnew

  inherited Destroy;
end;
<<<<<<< HEAD

<<<<<<< HEAD
procedure TWinCEGAPIConsole.Open(const ATitle: string; APages: Integer = 0);
<<<<<<< HEAD
<<<<<<< HEAD
=======

procedure TWinCEGAPIConsole.Open(const ATitle: string; APages: Integer = 0);

>>>>>>> graemeg/cpstrnew
=======
  inherited Destroy;
end;

procedure TWinCEGAPIConsole.Open(const ATitle: string; APages: Integer = 0);

>>>>>>> origin/cpstrnew
begin
  Open(ATitle, FDisplayFormat, APages);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TWinCEGAPIConsole.Open(const ATitle: string; AFormat: IPTCFormat;
                                 APages: Integer = 0);
=======
procedure TWinCEGAPIConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
                                 APages: Integer = 0);

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGAPIConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
                                 APages: Integer = 0);

>>>>>>> origin/cpstrnew
begin
  Open(ATitle, FDisplayWidth, FDisplayHeight, AFormat, APages);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TWinCEGAPIConsole.Open(const ATitle: string; AMode: IPTCMode;
                                 APages: Integer = 0);
=======
=======
>>>>>>> graemeg/cpstrnew

begin
  Open(ATitle, FDisplayFormat, APages);
end;

procedure TWinCEGAPIConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
                                 APages: Integer = 0);

begin
  Open(ATitle, FDisplayWidth, FDisplayHeight, AFormat, APages);
end;

procedure TWinCEGAPIConsole.Open(const ATitle: string; const AMode: TPTCMode;
                                 APages: Integer = 0);

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
end;

procedure TWinCEGAPIConsole.Open(const ATitle: string; AWidth, AHeight: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
                                 AFormat: IPTCFormat; APages: Integer = 0);
=======
=======
>>>>>>> graemeg/cpstrnew
                                 const AFormat: TPTCFormat; APages: Integer = 0);

var
  tmp: TPTCArea;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGAPIConsole.Open(const ATitle: string; const AMode: TPTCMode;
                                 APages: Integer = 0);

=======
procedure TWinCEGAPIConsole.Open(const ATitle: string; const AMode: TPTCMode;
                                 APages: Integer = 0);

>>>>>>> origin/cpstrnew
begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
end;

procedure TWinCEGAPIConsole.Open(const ATitle: string; AWidth, AHeight: Integer;
                                 const AFormat: TPTCFormat; APages: Integer = 0);

var
  tmp: TPTCArea;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  LOG('TWinCEGAPIConsole.Open');

  if FOpen then
    Close;

  try
=======
=======
>>>>>>> origin/fixes_2_2
  For I := Low(FModes) To High(FModes) Do
    FModes[I].Free;

  Inherited Destroy;
End;

Procedure TWinCEGAPIConsole.Open(Const ATitle : String; APages : Integer = 0);

Begin
  Open(ATitle, FDisplayFormat, APages);
End;

Procedure TWinCEGAPIConsole.Open(Const ATitle : String; Const AFormat : TPTCFormat;
                                 APages : Integer = 0);

Begin
  Open(ATitle, FDisplayWidth, FDisplayHeight, AFormat, APages);
End;

Procedure TWinCEGAPIConsole.Open(Const ATitle : String; Const AMode : TPTCMode;
                                 APages : Integer = 0);

Begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
End;

Procedure TWinCEGAPIConsole.Open(Const ATitle : String; AWidth, AHeight : Integer;
                                 Const AFormat : TPTCFormat; APages : Integer = 0);

Var
  tmp : TPTCArea;

Begin
  LOG('TWinCEGAPIConsole.Open');

  If FOpen Then
    Close;

  Try
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    LOG('creating window');
    FWindow := TWinCEWindow.Create('PTC_GAPI_FULLSCREEN',
                                   ATitle,
                                   0,
<<<<<<< HEAD
<<<<<<< HEAD
                                   WS_VISIBLE {Or WS_SYSMENU or WS_CAPTION},
                                   SW_SHOWNORMAL,
                                   0, 0,
=======
                                   WS_VISIBLE {Or WS_SYSMENU Or WS_CAPTION},
                                   SW_SHOWNORMAL,
                                   CW_USEDEFAULT, CW_USEDEFAULT,
>>>>>>> graemeg/fixes_2_2
=======
                                   WS_VISIBLE {Or WS_SYSMENU Or WS_CAPTION},
                                   SW_SHOWNORMAL,
                                   CW_USEDEFAULT, CW_USEDEFAULT,
>>>>>>> origin/fixes_2_2
                                   FDisplayWidth, FDisplayHeight,
				   @WndProc);
    LOG('window created successfully');

    LOG('opening display');
<<<<<<< HEAD
<<<<<<< HEAD
    if GXOpenDisplay(FWindow.WindowHandle, GX_FULLSCREEN) <> 0 then
      FGXDisplayIsOpen := True {success!!!}
    else
      raise TPTCError.Create('could not open display');

<<<<<<< HEAD
	FArea := TPTCArea.Create(0, 0, FDisplayWidth, FDisplayHeight);
	FClip := FArea;
=======
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
=======
=======
>>>>>>> origin/fixes_2_2
    If GXOpenDisplay(FWindow.WindowHandle, GX_FULLSCREEN) <> 0 Then
      FGXDisplayIsOpen := True {success!!!}
    Else
      Raise TPTCError.Create('could not open display');

    tmp := TPTCArea.Create(0, 0, FDisplayWidth, FDisplayHeight);
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

    FEventQueue := TEventQueue.Create;
    FKeyboard := TWinCEKeyboard.Create(FEventQueue);
    FMouse := TWinCEMouse.Create(FEventQueue, True, FDisplayWidth, FDisplayHeight);

<<<<<<< HEAD
<<<<<<< HEAD
    if {m_primary.m_fullscreen}True then
=======
    If {m_primary.m_fullscreen}True Then
>>>>>>> graemeg/fixes_2_2
=======
    If {m_primary.m_fullscreen}True Then
>>>>>>> origin/fixes_2_2
      FMouse.SetWindowArea(0, 0, FDisplayWidth, FDisplayHeight);

    FWindow.Update;

    FOpen := True;
<<<<<<< HEAD
<<<<<<< HEAD
  except
    on error: TObject do
    begin
      Close;
<<<<<<< HEAD
      raise;
    end;
  end;
end;

procedure TWinCEGAPIConsole.Close;
=======
      Raise;
    end;
  end;
end;

procedure TWinCEGAPIConsole.Close;

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
  LOG('TWinCEGAPIConsole.Close');

  if FGXDisplayIsOpen Then;
=======
=======
>>>>>>> origin/fixes_2_2
  Except
    On error : TObject Do
    Begin
      Close;
      Raise;
    End;
  End;
End;

Procedure TWinCEGAPIConsole.Close;

Begin
  LOG('TWinCEGAPIConsole.Close');

  If FGXDisplayIsOpen Then;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    GXCloseDisplay;
  FGXDisplayIsOpen := False;

  FreeAndNil(FKeyboard);
  FreeAndNil(FMouse);
  FreeAndNil(FWindow);
  FreeAndNil(FEventQueue);

  FOpen := False;
<<<<<<< HEAD
<<<<<<< HEAD
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TWinCEGAPIConsole.Copy(ASurface: IPTCSurface);
begin
end;

procedure TWinCEGAPIConsole.Copy(ASurface: IPTCSurface;
                                 ASource, ADestination: IPTCArea);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TWinCEGAPIConsole.Copy(ASurface: TPTCBaseSurface);

begin
end;

procedure TWinCEGAPIConsole.Copy(ASurface: TPTCBaseSurface;
                                 const ASource, ADestination: TPTCArea);

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

procedure TWinCEGAPIConsole.Load(const APixels: Pointer;
                                 AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                 AFormat: IPTCFormat;
                                 APalette: IPTCPalette);
var
  console_pixels: Pointer;
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
  CheckOpen(    'TWinCEGAPIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  CheckUnlocked('TWinCEGAPIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  if Clip.Equals(Area) then
  begin
    try
      console_pixels := Lock;
      try
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWinCEGAPIConsole.Copy(Var ASurface : TPTCBaseSurface);

Begin
End;

Procedure TWinCEGAPIConsole.Copy(Var ASurface : TPTCBaseSurface;
                                 Const ASource, ADestination : TPTCArea);

Begin
End;

Procedure TWinCEGAPIConsole.Load(Const APixels : Pointer;
                                 AWidth, AHeight, APitch : Integer;
                                 Const AFormat : TPTCFormat;
                                 Const APalette : TPTCPalette);
Var
  Area_ : TPTCArea;
  console_pixels : Pointer;

Begin
  CheckOpen(    'TWinCEGAPIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
  CheckUnlocked('TWinCEGAPIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette)');
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

procedure TWinCEGAPIConsole.Load(const APixels: Pointer;
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
  console_pixels: Pointer;
  clipped_source, clipped_destination: TPTCArea;
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
  CheckOpen(    'TWinCEGAPIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  CheckUnlocked('TWinCEGAPIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  clipped_source := nil;
  clipped_destination := nil;
  try
    console_pixels := Lock;
    try
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      TPTCClipper.Clip(ASource, TPTCArea.Create(0, 0, AWidth, AHeight), clipped_source, ADestination, Clip, clipped_destination);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
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

Procedure TWinCEGAPIConsole.Load(Const APixels : Pointer;
                                 AWidth, AHeight, APitch : Integer;
                                 Const AFormat : TPTCFormat;
                                 Const APalette : TPTCPalette;
                                 Const ASource, ADestination : TPTCArea);
Var
  console_pixels : Pointer;
  clipped_source, clipped_destination : TPTCArea;
  tmp : TPTCArea;

Begin
  CheckOpen(    'TWinCEGAPIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
  CheckUnlocked('TWinCEGAPIConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination)');
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
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGAPIConsole.Save(APixels: Pointer;
                                 AWidth, AHeight, APitch: Integer;
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
end;

procedure TWinCEGAPIConsole.Save(APixels: Pointer;
                                 AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
                                 AFormat: IPTCFormat;
                                 APalette: IPTCPalette;
                                 ASource, ADestination: IPTCArea);
=======
=======
>>>>>>> graemeg/cpstrnew
                                 const AFormat: TPTCFormat;
                                 const APalette: TPTCPalette;
                                 const ASource, ADestination: TPTCArea);

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
begin
end;

function TWinCEGAPIConsole.Lock: Pointer;
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
begin
  CheckUnlocked('display already locked');
  Result := GXBeginDraw;

  if Result = nil then
    raise TPTCError.Create('the display cannot be locked');

  FLocked := True;
end;
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGAPIConsole.Unlock;
=======

procedure TWinCEGAPIConsole.Unlock;

>>>>>>> graemeg/cpstrnew
begin
  if not FLocked then
    raise TPTCError.Create('display is not locked');

=======

procedure TWinCEGAPIConsole.Unlock;

begin
  if not FLocked then
    raise TPTCError.Create('display is not locked');

>>>>>>> graemeg/cpstrnew
  if GXEndDraw = 0 then
    raise TPTCError.Create('could not unlock display');

  FLocked := False;
end;
<<<<<<< HEAD

procedure TWinCEGAPIConsole.Clear;
<<<<<<< HEAD
begin
end;

procedure TWinCEGAPIConsole.Clear(AColor: IPTCColor);
begin
end;

procedure TWinCEGAPIConsole.Clear(AColor: IPTCColor;
                                  AArea: IPTCArea);
begin
end;

procedure TWinCEGAPIConsole.Configure(const AFileName: string);
var
  F: Text;
  S: string;
=======
=======

procedure TWinCEGAPIConsole.Clear;

begin
end;

procedure TWinCEGAPIConsole.Clear(const AColor: TPTCColor);
>>>>>>> graemeg/cpstrnew

begin
end;

<<<<<<< HEAD
procedure TWinCEGAPIConsole.Clear(const AColor: TPTCColor);
=======
procedure TWinCEGAPIConsole.Clear(const AColor: TPTCColor;
                    const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew

begin
end;

<<<<<<< HEAD
procedure TWinCEGAPIConsole.Clear(const AColor: TPTCColor;
                    const AArea: TPTCArea);

begin
end;

procedure TWinCEGAPIConsole.Configure(const AFileName: String);

var
  F: Text;
  S: string;

>>>>>>> graemeg/cpstrnew
=======
procedure TWinCEGAPIConsole.Configure(const AFileName: String);

var
  F: Text;
  S: string;

>>>>>>> graemeg/cpstrnew
begin
=======

procedure TWinCEGAPIConsole.Save(APixels: Pointer;
                                 AWidth, AHeight, APitch: Integer;
                                 const AFormat: TPTCFormat;
                                 const APalette: TPTCPalette);

begin
end;

procedure TWinCEGAPIConsole.Save(APixels: Pointer;
                                 AWidth, AHeight, APitch: Integer;
                                 const AFormat: TPTCFormat;
                                 const APalette: TPTCPalette;
                                 const ASource, ADestination: TPTCArea);

begin
end;

function TWinCEGAPIConsole.Lock: Pointer;

=======

procedure TWinCEGAPIConsole.Save(APixels: Pointer;
                                 AWidth, AHeight, APitch: Integer;
                                 const AFormat: TPTCFormat;
                                 const APalette: TPTCPalette);

begin
end;

procedure TWinCEGAPIConsole.Save(APixels: Pointer;
                                 AWidth, AHeight, APitch: Integer;
                                 const AFormat: TPTCFormat;
                                 const APalette: TPTCPalette;
                                 const ASource, ADestination: TPTCArea);

begin
end;

function TWinCEGAPIConsole.Lock: Pointer;

>>>>>>> origin/cpstrnew
begin
  CheckUnlocked('display already locked');
  Result := GXBeginDraw;

  if Result = nil then
    raise TPTCError.Create('the display cannot be locked');

  FLocked := True;
end;

procedure TWinCEGAPIConsole.Unlock;

begin
  if not FLocked then
    raise TPTCError.Create('display is not locked');

  if GXEndDraw = 0 then
    raise TPTCError.Create('could not unlock display');

  FLocked := False;
end;

procedure TWinCEGAPIConsole.Clear;

begin
end;

procedure TWinCEGAPIConsole.Clear(const AColor: TPTCColor);

begin
end;

procedure TWinCEGAPIConsole.Clear(const AColor: TPTCColor;
                    const AArea: TPTCArea);

begin
end;

procedure TWinCEGAPIConsole.Configure(const AFileName: String);

var
  F: Text;
  S: string;

begin
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
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
    if IOResult <> 0 then
      break;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    {$I-}
    Readln(F, S);
    {$I+}
    if IOResult <> 0 then
      Break;
>>>>>>> graemeg/cpstrnew
    Option(S);
  end;
  CloseFile(F);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGAPIConsole.Option(const AOption: string): Boolean;
=======

function TWinCEGAPIConsole.Option(const AOption: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.Option(const AOption: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.Option(const AOption: String): Boolean;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.Option(const AOption: String): Boolean;

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

Procedure TWinCEGAPIConsole.Save(APixels : Pointer;
                                 AWidth, AHeight, APitch : Integer;
                                 Const AFormat : TPTCFormat;
                                 Const APalette : TPTCPalette);

Begin
End;

Procedure TWinCEGAPIConsole.Save(APixels : Pointer;
                                 AWidth, AHeight, APitch : Integer;
                                 Const AFormat : TPTCFormat;
                                 Const APalette : TPTCPalette;
                                 Const ASource, ADestination : TPTCArea);

Begin
End;

Function TWinCEGAPIConsole.Lock : Pointer;

Begin
  CheckUnlocked('display already locked');
  Result := GXBeginDraw;

  If Result = Nil Then
    Raise TPTCError.Create('the display cannot be locked');

  FLocked := True;
End;

Procedure TWinCEGAPIConsole.Unlock;

Begin
  If Not FLocked Then
    Raise TPTCError.Create('display is not locked');

  If GXEndDraw = 0 Then
    Raise TPTCError.Create('could not unlock display');

  FLocked := False;
End;

Procedure TWinCEGAPIConsole.Clear;

Begin
End;

Procedure TWinCEGAPIConsole.Clear(Const AColor : TPTCColor);

Begin
End;

Procedure TWinCEGAPIConsole.Clear(Const AColor : TPTCColor;
                    Const AArea : TPTCArea);

Begin
End;

Procedure TWinCEGAPIConsole.Configure(Const AFileName : String);

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

Function TWinCEGAPIConsole.Option(Const AOption : String) : Boolean;

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

procedure TWinCEGAPIConsole.Palette(APalette: IPTCPalette);
begin
end;

procedure TWinCEGAPIConsole.Clip(AArea: IPTCArea);
begin
  CheckOpen('TWinCEGAPIConsole.Clip(AArea)');

  FClip := TPTCClipper.Clip(AArea, FArea);
end;

function TWinCEGAPIConsole.Clip: IPTCArea;
=======

procedure TWinCEGAPIConsole.Palette(const APalette: TPTCPalette);

begin
end;

procedure TWinCEGAPIConsole.Clip(const AArea: TPTCArea);

var
  tmp: TPTCArea;

=======
=======
>>>>>>> origin/cpstrnew

procedure TWinCEGAPIConsole.Palette(const APalette: TPTCPalette);

begin
end;

procedure TWinCEGAPIConsole.Clip(const AArea: TPTCArea);

var
  tmp: TPTCArea;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Palette(const APalette: TPTCPalette);

begin
end;

procedure TWinCEGAPIConsole.Clip(const AArea: TPTCArea);

var
  tmp: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGAPIConsole.Clip(AArea)');

  tmp := TPTCClipper.Clip(AArea, FArea);
  try
    FClip.Assign(tmp);
  finally
    tmp.Free;
  end;
end;

function TWinCEGAPIConsole.Clip: TPTCArea;

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
  CheckOpen('TWinCEGAPIConsole.Clip');
  Result := FClip;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGAPIConsole.Palette: IPTCPalette;
begin
end;

function TWinCEGAPIConsole.Modes: TPTCModeList;
begin
  Result := FModes;
end;

function TWinCEGAPIConsole.WndProc(Ahwnd: HWND; AuMsg: UINT; AwParam: WPARAM; AlParam: LPARAM): LRESULT;
=======

function TWinCEGAPIConsole.Palette: TPTCPalette;

begin
end;

function TWinCEGAPIConsole.Modes: PPTCMode;

begin
=======

function TWinCEGAPIConsole.Palette: TPTCPalette;

begin
end;

function TWinCEGAPIConsole.Modes: PPTCMode;

begin
>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.Palette: TPTCPalette;

begin
end;

function TWinCEGAPIConsole.Modes: PPTCMode;

begin
>>>>>>> graemeg/cpstrnew
  Result := @FModes[0];
end;

function TWinCEGAPIConsole.WndProc(Ahwnd: HWND; AuMsg: UINT; AwParam: WPARAM; AlParam: LPARAM): LRESULT;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.Palette: TPTCPalette;

begin
end;

function TWinCEGAPIConsole.Modes: PPTCMode;

begin
  Result := @FModes[0];
end;

function TWinCEGAPIConsole.WndProc(Ahwnd: HWND; AuMsg: UINT; AwParam: WPARAM; AlParam: LPARAM): LRESULT;

>>>>>>> origin/cpstrnew
begin
  case AuMsg of
  WM_CLOSE: begin
    LOG('TWinCEGAPIConsole.WndProc: WM_CLOSE');
    Halt(0);
  end;
  WM_KILLFOCUS: begin
    LOG('TWinCEGAPIConsole.WndProc: WM_KILLFOCUS');
    if FGXDisplayIsOpen then
      GXSuspend;
    Result := 0;
    exit;
  end;
  WM_SETFOCUS: begin
    LOG('TWinCEGAPIConsole.WndProc: WM_SETFOCUS');
    if FGXDisplayIsOpen then
      GXResume;
    Result := 0;
    exit;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGAPIConsole.Flush;
=======

procedure TWinCEGAPIConsole.Flush;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Flush;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Flush;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Flush;

>>>>>>> origin/cpstrnew
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWinCEGAPIConsole.Palette(Const APalette : TPTCPalette);

Begin
End;

Procedure TWinCEGAPIConsole.Clip(Const AArea : TPTCArea);

Var
  tmp : TPTCArea;

Begin
  CheckOpen('TWinCEGAPIConsole.Clip(AArea)');

  tmp := TPTCClipper.Clip(AArea, FArea);
  Try
    FClip.Assign(tmp);
  Finally
    tmp.Free;
  End;
End;

Function TWinCEGAPIConsole.Clip : TPTCArea;

Begin
  CheckOpen('TWinCEGAPIConsole.Clip');
  Result := FClip;
End;

Function TWinCEGAPIConsole.Palette : TPTCPalette;

Begin
End;

Function TWinCEGAPIConsole.Modes : PPTCMode;

Begin
  Result := @FModes[0];
End;

Function TWinCEGAPIConsole.WndProc(Ahwnd : HWND; AuMsg : UINT; AwParam : WPARAM; AlParam : LPARAM) : LRESULT;

Begin
  Case AuMsg Of
  WM_CLOSE : Begin
    LOG('TWinCEGAPIConsole.WndProc: WM_CLOSE');
    Halt(0);
  End;
  WM_KILLFOCUS : Begin
    LOG('TWinCEGAPIConsole.WndProc: WM_KILLFOCUS');
    If FGXDisplayIsOpen Then
      GXSuspend;
    Result := 0;
    Exit;
  End;
  WM_SETFOCUS : Begin
    LOG('TWinCEGAPIConsole.WndProc: WM_SETFOCUS');
    If FGXDisplayIsOpen Then
      GXResume;
    Result := 0;
    Exit;
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

Procedure TWinCEGAPIConsole.Flush;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckOpen    ('TWinCEGAPIConsole.Flush');
  CheckUnlocked('TWinCEGAPIConsole.Flush');

  Update;
<<<<<<< HEAD
<<<<<<< HEAD
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGAPIConsole.Finish;
=======

procedure TWinCEGAPIConsole.Finish;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Finish;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Finish;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Finish;

>>>>>>> origin/cpstrnew
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWinCEGAPIConsole.Finish;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckOpen    ('TWinCEGAPIConsole.Finish');
  CheckUnlocked('TWinCEGAPIConsole.Finish');

  Update;
<<<<<<< HEAD
<<<<<<< HEAD
end;
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGAPIConsole.Update;
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Update;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Update;

>>>>>>> origin/cpstrnew
begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWinCEGAPIConsole.Update;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CheckOpen    ('TWinCEGAPIConsole.Update');
  CheckUnlocked('TWinCEGAPIConsole.Update');

  FWindow.Update;
<<<<<<< HEAD
<<<<<<< HEAD
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TWinCEGAPIConsole.Update(AArea: IPTCArea);
=======

procedure TWinCEGAPIConsole.Update(const AArea: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Update(const AArea: TPTCArea);

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Update(const AArea: TPTCArea);

>>>>>>> graemeg/cpstrnew
begin
  Update;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TWinCEGAPIConsole.NextEvent(out AEvent: IPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
=======
function TWinCEGAPIConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TWinCEGAPIConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;

>>>>>>> graemeg/cpstrnew
=======
function TWinCEGAPIConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;

>>>>>>> graemeg/cpstrnew
=======

procedure TWinCEGAPIConsole.Update(const AArea: TPTCArea);

begin
  Update;
end;

function TWinCEGAPIConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGAPIConsole.NextEvent');
//  CheckUnlocked('TWinCEGAPIConsole.NextEvent');

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
End;

Procedure TWinCEGAPIConsole.Update(Const AArea : TPTCArea);

Begin
  Update;
End;

Function TWinCEGAPIConsole.NextEvent(Var AEvent : TPTCEvent; AWait : Boolean; Const AEventMask : TPTCEventMask) : Boolean;

Begin
  CheckOpen('TWinCEGAPIConsole.NextEvent');
//  CheckUnlocked('TWinCEGAPIConsole.NextEvent');

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

function TWinCEGAPIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): IPTCEvent;
=======

function TWinCEGAPIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGAPIConsole.PeekEvent');
//  CheckUnlocked('TWinCEGAPIConsole.PeekEvent');

  repeat
=======
=======
>>>>>>> origin/fixes_2_2
  Until (Not AWait) Or (AEvent <> Nil);
  Result := AEvent <> Nil;
End;

Function TWinCEGAPIConsole.PeekEvent(AWait : Boolean; Const AEventMask : TPTCEventMask) : TPTCEvent;

Begin
  CheckOpen('TWinCEGAPIConsole.PeekEvent');
//  CheckUnlocked('TWinCEGAPIConsole.PeekEvent');

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
  until (not AWait) or (Result <> nil);
end;
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGAPIConsole.GetWidth: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetWidth: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetWidth: Integer;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGAPIConsole.GetWidth');
  Result := FDisplayWidth;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGAPIConsole.GetHeight: Integer;
=======

function TWinCEGAPIConsole.GetHeight: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetHeight: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetHeight: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetHeight: Integer;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGAPIConsole.GetHeight');
  Result := FDisplayHeight;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGAPIConsole.GetPitch: Integer;
=======

function TWinCEGAPIConsole.GetPitch: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetPitch: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetPitch: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetPitch: Integer;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGAPIConsole.GetPitch');
  Result := FDisplayPitch;
end;
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
function TWinCEGAPIConsole.GetArea: IPTCArea;
=======
function TWinCEGAPIConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======
function TWinCEGAPIConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetArea: TPTCArea;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetArea: TPTCArea;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGAPIConsole.GetArea');
  Result := FArea;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGAPIConsole.GetFormat: IPTCFormat;
=======

function TWinCEGAPIConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetFormat: TPTCFormat;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetFormat: TPTCFormat;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGAPIConsole.GetFormat');
  Result := FDisplayFormat;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGAPIConsole.GetPages: Integer;
=======

function TWinCEGAPIConsole.GetPages: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetPages: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetPages: Integer;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetPages: Integer;

>>>>>>> origin/cpstrnew
begin
  CheckOpen('TWinCEGAPIConsole.GetPages');
  Result := 1; {???}
end;
<<<<<<< HEAD

function TWinCEGAPIConsole.GetName: string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

function TWinCEGAPIConsole.GetName: string;

>>>>>>> origin/cpstrnew
begin
  Result := 'GAPI';
end;

function TWinCEGAPIConsole.GetTitle: string;
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew

begin
  Result := 'GAPI';
end;

function TWinCEGAPIConsole.GetTitle: string;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
begin
  CheckOpen('TWinCEGAPIConsole.GetTitle');
  Result := FTitle;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TWinCEGAPIConsole.GetInformation: string;
=======

function TWinCEGAPIConsole.GetInformation: string;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetInformation: string;

>>>>>>> graemeg/cpstrnew
=======

function TWinCEGAPIConsole.GetInformation: string;

>>>>>>> graemeg/cpstrnew
=======

begin
  CheckOpen('TWinCEGAPIConsole.GetTitle');
  Result := FTitle;
end;

function TWinCEGAPIConsole.GetInformation: string;

>>>>>>> origin/cpstrnew
begin
  Result := ''; // todo...
end;

procedure TWinCEGAPIConsole.CheckOpen(    AMessage: String);
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
  if not FOpen then
  try
    raise TPTCError.Create('console is not open');
  except
    on error: TPTCError do
      raise TPTCError.Create(AMessage, error);
  end;
end;

procedure TWinCEGAPIConsole.CheckUnlocked(AMessage: String);
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

Function TWinCEGAPIConsole.GetWidth : Integer;

Begin
  CheckOpen('TWinCEGAPIConsole.GetWidth');
  Result := FDisplayWidth;
End;

Function TWinCEGAPIConsole.GetHeight : Integer;

Begin
  CheckOpen('TWinCEGAPIConsole.GetHeight');
  Result := FDisplayHeight;
End;

Function TWinCEGAPIConsole.GetPitch : Integer;

Begin
  CheckOpen('TWinCEGAPIConsole.GetPitch');
  Result := FDisplayPitch;
End;

Function TWinCEGAPIConsole.GetArea : TPTCArea;

Begin
  CheckOpen('TWinCEGAPIConsole.GetArea');
  Result := FArea;
End;

Function TWinCEGAPIConsole.GetFormat : TPTCFormat;

Begin
  CheckOpen('TWinCEGAPIConsole.GetFormat');
  Result := FDisplayFormat;
End;

Function TWinCEGAPIConsole.GetPages : Integer;

Begin
  CheckOpen('TWinCEGAPIConsole.GetPages');
  Result := 1; {???}
End;

Function TWinCEGAPIConsole.GetName : String;

Begin
  Result := 'GAPI';
End;

Function TWinCEGAPIConsole.GetTitle : String;

Begin
  CheckOpen('TWinCEGAPIConsole.GetTitle');
  Result := FTitle;
End;

Function TWinCEGAPIConsole.GetInformation : String;

Begin
  Result := ''; // todo...
End;

Procedure TWinCEGAPIConsole.CheckOpen(    AMessage : String);

Begin
  If Not FOpen Then
  Try
    Raise TPTCError.Create('console is not open');
  Except
    On error : TPTCError Do
      Raise TPTCError.Create(AMessage, error);
  End;
End;

Procedure TWinCEGAPIConsole.CheckUnlocked(AMessage : String);

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
