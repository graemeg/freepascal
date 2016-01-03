<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the PTCPas framebuffer library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2012 Nikolay Nikolov (nickysn@users.sourceforge.net)
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
    Original C++ version by Christian Nentwich (c.nentwich@cs.ucl.ac.uk)

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

constructor TX11Modes.Create(ADisplay: PDisplay; AScreen: cint);
begin
  FDisplay := ADisplay;
  FScreen := AScreen;
end;

constructor TX11ModesNoModeSwitching.Create(ADisplay: PDisplay; AScreen: cint);
begin
  inherited;

  FWidth := DisplayWidth(FDisplay, FScreen);
  FHeight := DisplayHeight(FDisplay, FScreen);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11ModesNoModeSwitching.GetModes(var AModes: TPTCModeList; ACurrentDesktopFormat: IPTCFormat);
begin
  SetLength(AModes, 1);
  AModes[0] := TPTCMode.Create(FWidth,
                               FHeight,
                               ACurrentDesktopFormat);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TX11ModesNoModeSwitching.GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat);
begin
=======
=======
>>>>>>> origin/fixes_2_2
Constructor TX11Modes.Create(ADisplay : PDisplay; AScreen : cint);

Begin
  FDisplay := ADisplay;
  FScreen := AScreen;
End;

Constructor TX11ModesNoModeSwitching.Create(ADisplay : PDisplay; AScreen : cint);

Begin
  Inherited;

  FWidth := DisplayWidth(FDisplay, FScreen);
  FHeight := DisplayHeight(FDisplay, FScreen);
End;

Procedure TX11ModesNoModeSwitching.GetModes(Var AModes : TPTCModeDynArray; ACurrentDesktopFormat : TPTCFormat);

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  SetLength(AModes, 2);
  AModes[0] := TPTCMode.Create(FWidth,
                               FHeight,
                               ACurrentDesktopFormat);
  AModes[1] := TPTCMode.Create;
<<<<<<< HEAD
<<<<<<< HEAD
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

procedure TX11ModesNoModeSwitching.SetBestMode(AWidth, AHeight: Integer);
begin
  FWidth := DisplayWidth(FDisplay, FScreen);
  FHeight := DisplayHeight(FDisplay, FScreen);
end;

procedure TX11ModesNoModeSwitching.RestorePreviousMode;
begin
  { do nothing }
end;

function TX11ModesNoModeSwitching.GetWidth: Integer;
begin
  Result := FWidth;
end;

function TX11ModesNoModeSwitching.GetHeight: Integer;
begin
  Result := FHeight;
end;

{$IFDEF ENABLE_X11_EXTENSION_XRANDR}
constructor TX11ModesXrandr.Create(ADisplay: PDisplay; AScreen: cint);
var
  dummy1, dummy2: cint;
  Major, Minor: cint;
begin
  inherited;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  FInMode := False;

=======

  FInMode := False;

>>>>>>> graemeg/cpstrnew
=======

  FInMode := False;

>>>>>>> graemeg/cpstrnew
=======

  FInMode := False;

>>>>>>> origin/cpstrnew
  if not XRRQueryExtension(FDisplay, @dummy1, @dummy2) then
    raise TPTCError.Create('Xrandr extension not available');
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TX11ModesNoModeSwitching.SetBestMode(AWidth, AHeight : Integer);

Begin
  FWidth := DisplayWidth(FDisplay, FScreen);
  FHeight := DisplayHeight(FDisplay, FScreen);
End;

Procedure TX11ModesNoModeSwitching.RestorePreviousMode;

Begin
  { do nothing }
End;

Function TX11ModesNoModeSwitching.GetWidth : Integer;

Begin
  Result := FWidth;
End;

Function TX11ModesNoModeSwitching.GetHeight : Integer;

Begin
  Result := FHeight;
End;

{$IFDEF ENABLE_X11_EXTENSION_XRANDR}
Constructor TX11ModesXrandr.Create(ADisplay : PDisplay; AScreen : cint);

Var
  dummy1, dummy2 : cint;
  Major, Minor : cint;

Begin
  Inherited;

  If Not XRRQueryExtension(FDisplay, @dummy1, @dummy2) Then
    Raise TPTCError.Create('Xrandr extension not available');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  XRRQueryVersion(FDisplay, @Major, @Minor); // todo: check
  LOG('Xrandr version: ' + IntToStr(Major) + '.' + IntToStr(Minor));

  FRoot := RootWindow(FDisplay, FScreen);

<<<<<<< HEAD
<<<<<<< HEAD
  { is this necessary? }
  CreateScreenConfig;
  DestroyScreenConfig;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

destructor TX11ModesXrandr.Destroy;
begin
  RestorePreviousMode;

  inherited;
end;

procedure TX11ModesXrandr.CreateScreenConfig;
begin
  DestroyScreenConfig;

=======

destructor TX11ModesXrandr.Destroy;
begin
  RestorePreviousMode;

  inherited;
end;

procedure TX11ModesXrandr.CreateScreenConfig;
begin
  DestroyScreenConfig;

>>>>>>> graemeg/cpstrnew
=======

destructor TX11ModesXrandr.Destroy;
begin
  RestorePreviousMode;

  inherited;
end;

procedure TX11ModesXrandr.CreateScreenConfig;
begin
  DestroyScreenConfig;

>>>>>>> graemeg/cpstrnew
  FXRRConfig := XRRGetScreenInfo(FDisplay, FRoot);
  if FXRRConfig = nil then
    raise TPTCError.Create('XRRGetScreenInfo failed');
end;

procedure TX11ModesXrandr.DestroyScreenConfig;
var
  tmp: PXRRScreenConfiguration;
begin
  if FXRRConfig <> nil then
  begin
    tmp := FXRRConfig;
    FXRRConfig := nil;
    XRRFreeScreenConfigInfo(tmp);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11ModesXrandr.GetModes(var AModes: TPTCModeList; ACurrentDesktopFormat: IPTCFormat);
=======
procedure TX11ModesXrandr.GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11ModesXrandr.GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11ModesXrandr.GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
var
  ScreenSizes: PXRRScreenSize;
  ScreenSizesNum: cint;
  I: Integer;
begin
  ScreenSizes := XRRConfigSizes(FXRRConfig, @ScreenSizesNum);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SetLength(AModes, ScreenSizesNum);
=======
  SetLength(AModes, ScreenSizesNum + 1);
  AModes[ScreenSizesNum] := TPTCMode.Create;
>>>>>>> graemeg/cpstrnew
=======
  SetLength(AModes, ScreenSizesNum + 1);
  AModes[ScreenSizesNum] := TPTCMode.Create;
>>>>>>> graemeg/cpstrnew
=======
  SetLength(AModes, ScreenSizesNum + 1);
  AModes[ScreenSizesNum] := TPTCMode.Create;
>>>>>>> graemeg/cpstrnew
  for I := 0 to ScreenSizesNum - 1 do
    AModes[I] := TPTCMode.Create(ScreenSizes[I].width,
                                 ScreenSizes[I].height,
                                 ACurrentDesktopFormat);
end;

class function TX11ModesXrandr.FindBestMode(AScreenSizes: PXRRScreenSize;
                                            AScreenSizesNum: Integer;
                                            ARequestedWidth,
                                            ARequestedHeight: Integer): Integer;
var
  I: Integer;
  min_diff: Integer;
  d_x, d_y: Integer;
  found_mode: Integer;
begin
  { Find exact match }
  for I := 0 to AScreenSizesNum - 1 do
    if (AScreenSizes[I].width = ARequestedWidth) and
       (AScreenSizes[I].height = ARequestedHeight) then
      exit(I);

  { try to find a mode that matches the width first }
  for I := 0 to AScreenSizesNum - 1 do
    if (AScreenSizes[I].width = ARequestedWidth) and
       (AScreenSizes[I].height >= ARequestedHeight) then
      exit(I);

  { Next try to match the height }
  for I := 0 to AScreenSizesNum - 1 do
    if (AScreenSizes[I].width >= ARequestedWidth) and
       (AScreenSizes[I].height = ARequestedHeight) then
      exit(I);

  { Finally, find the mode that is bigger than the requested one and makes }
  { the least difference }
  found_mode := -1;
  min_diff := High(Integer);
  for I := 0 to AScreenSizesNum - 1 do
    if (AScreenSizes[I].width >= ARequestedWidth) and
       (AScreenSizes[I].height >= ARequestedHeight) then
    begin
      d_x := Sqr(AScreenSizes[I].width - ARequestedWidth);
      d_y := Sqr(AScreenSizes[I].height - ARequestedHeight);
      if (d_x + d_y) < min_diff then
      begin
        min_diff := d_x + d_y;
        found_mode := I;
      end;
    end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if found_mode <> -1 then
    exit(found_mode);

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
  raise TPTCError.Create('Cannot find matching video mode');
end;

procedure TX11ModesXrandr.SetBestMode(AWidth, AHeight: Integer);
var
  ScreenSizes: PXRRScreenSize;
  ScreenSizesNum: cint;
  BestMode: Integer;
  CurrentRotation: TRotation;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  I: Integer;
>>>>>>> graemeg/cpstrnew
=======
  I: Integer;
>>>>>>> graemeg/cpstrnew
=======
  I: Integer;
>>>>>>> graemeg/cpstrnew
begin
  CreateScreenConfig;
  try
    if not FInMode then
      SaveCurrentMode;

    ScreenSizes := XRRConfigSizes(FXRRConfig, @ScreenSizesNum);

    BestMode := FindBestMode(ScreenSizes, ScreenSizesNum, AWidth, AHeight);
    FWidth := ScreenSizes[BestMode].width;
    FHeight := ScreenSizes[BestMode].height;

    XRRConfigCurrentConfiguration(FXRRConfig, @CurrentRotation);

    X11CheckSuccess(XRRSetScreenConfig(FDisplay, FXRRConfig, FRoot, BestMode,
                                       CurrentRotation, CurrentTime),
                    'XRRSetScreenConfig failed');

    FInMode := True;
  finally
    DestroyScreenConfig;
  end;
end;

procedure TX11ModesXrandr.SaveCurrentMode;
var
  SizeIndex: TSizeID;
  ScreenSizes: PXRRScreenSize;
  ScreenSizesNum: cint;
begin
  LOG('xrandr: saving previous mode');

  SizeIndex := XRRConfigCurrentConfiguration(FXRRConfig, @FSavedMode.Rotation);
  FSavedMode.Rate := XRRConfigCurrentRate(FXRRConfig);

  ScreenSizes := XRRConfigSizes(FXRRConfig, @ScreenSizesNum);
  if (SizeIndex < 0) or (SizeIndex >= ScreenSizesNum) then
    raise TPTCError.Create('XRRConfigCurrentConfiguration returned a size index outside the range of screen sizes');
  FSavedMode.ScreenSize := ScreenSizes[SizeIndex];
end;

procedure TX11ModesXrandr.RestorePreviousMode;
var
  I: Integer;
  SizeIndex: cint;
  ScreenSizes: PXRRScreenSize;
  ScreenSizesNum: cint;
begin
  if not FInMode then
    exit;

  LOG('xrandr: restoring previous mode');

  CreateScreenConfig;
  try
    SizeIndex := -1;
    ScreenSizes := XRRConfigSizes(FXRRConfig, @ScreenSizesNum);
    for I := 0 to ScreenSizesNum - 1 do
      if (ScreenSizes[I].width   = FSavedMode.ScreenSize.width  ) and
         (ScreenSizes[I].height  = FSavedMode.ScreenSize.height ) and
         (ScreenSizes[I].mwidth  = FSavedMode.ScreenSize.mwidth ) and
         (ScreenSizes[I].mheight = FSavedMode.ScreenSize.mheight) then
      begin
        SizeIndex := I;
        Break;
      end;
    if SizeIndex = -1 then
      for I := 0 to ScreenSizesNum - 1 do
        if (ScreenSizes[I].width  = FSavedMode.ScreenSize.width ) and
           (ScreenSizes[I].height = FSavedMode.ScreenSize.height) then
        begin
          SizeIndex := I;
          Break;
        end;
    if SizeIndex = -1 then
      raise TPTCError.Create('xrandr: saved mode size not found in size list');

    X11CheckSuccess(XRRSetScreenConfigAndRate(FDisplay,
                                              FXRRConfig,
                                              FRoot,
                                              SizeIndex,
                                              FSavedMode.Rotation,
                                              FSavedMode.Rate,
                                              CurrentTime),
                    'XRRSetScreenConfigAndRate failed');

    FInMode := False;
  finally
    DestroyScreenConfig;
  end;
end;

function TX11ModesXrandr.GetWidth: Integer;
begin
  Result := FWidth;
end;

=======

destructor TX11ModesXrandr.Destroy;
begin
  RestorePreviousMode;

  inherited;
end;

procedure TX11ModesXrandr.CreateScreenConfig;
begin
  DestroyScreenConfig;

  FXRRConfig := XRRGetScreenInfo(FDisplay, FRoot);
  if FXRRConfig = nil then
    raise TPTCError.Create('XRRGetScreenInfo failed');
end;

procedure TX11ModesXrandr.DestroyScreenConfig;
var
  tmp: PXRRScreenConfiguration;
begin
  if FXRRConfig <> nil then
  begin
    tmp := FXRRConfig;
    FXRRConfig := nil;
    XRRFreeScreenConfigInfo(tmp);
  end;
end;

procedure TX11ModesXrandr.GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat);
var
  ScreenSizes: PXRRScreenSize;
  ScreenSizesNum: cint;
  I: Integer;
begin
  ScreenSizes := XRRConfigSizes(FXRRConfig, @ScreenSizesNum);

  SetLength(AModes, ScreenSizesNum + 1);
  AModes[ScreenSizesNum] := TPTCMode.Create;
  for I := 0 to ScreenSizesNum - 1 do
    AModes[I] := TPTCMode.Create(ScreenSizes[I].width,
                                 ScreenSizes[I].height,
                                 ACurrentDesktopFormat);
end;

class function TX11ModesXrandr.FindBestMode(AScreenSizes: PXRRScreenSize;
                                            AScreenSizesNum: Integer;
                                            ARequestedWidth,
                                            ARequestedHeight: Integer): Integer;
var
  I: Integer;
  min_diff: Integer;
  d_x, d_y: Integer;
  found_mode: Integer;
begin
  { Find exact match }
  for I := 0 to AScreenSizesNum - 1 do
    if (AScreenSizes[I].width = ARequestedWidth) and
       (AScreenSizes[I].height = ARequestedHeight) then
      exit(I);

  { try to find a mode that matches the width first }
  for I := 0 to AScreenSizesNum - 1 do
    if (AScreenSizes[I].width = ARequestedWidth) and
       (AScreenSizes[I].height >= ARequestedHeight) then
      exit(I);

  { Next try to match the height }
  for I := 0 to AScreenSizesNum - 1 do
    if (AScreenSizes[I].width >= ARequestedWidth) and
       (AScreenSizes[I].height = ARequestedHeight) then
      exit(I);

  { Finally, find the mode that is bigger than the requested one and makes }
  { the least difference }
  found_mode := -1;
  min_diff := High(Integer);
  for I := 0 to AScreenSizesNum - 1 do
    if (AScreenSizes[I].width >= ARequestedWidth) and
       (AScreenSizes[I].height >= ARequestedHeight) then
    begin
      d_x := Sqr(AScreenSizes[I].width - ARequestedWidth);
      d_y := Sqr(AScreenSizes[I].height - ARequestedHeight);
      if (d_x + d_y) < min_diff then
      begin
        min_diff := d_x + d_y;
        found_mode := I;
      end;
    end;

  raise TPTCError.Create('Cannot find matching video mode');
end;

procedure TX11ModesXrandr.SetBestMode(AWidth, AHeight: Integer);
var
  ScreenSizes: PXRRScreenSize;
  ScreenSizesNum: cint;
  BestMode: Integer;
  CurrentRotation: TRotation;
  I: Integer;
begin
  CreateScreenConfig;
  try
    if not FInMode then
      SaveCurrentMode;

    ScreenSizes := XRRConfigSizes(FXRRConfig, @ScreenSizesNum);

    BestMode := FindBestMode(ScreenSizes, ScreenSizesNum, AWidth, AHeight);
    FWidth := ScreenSizes[BestMode].width;
    FHeight := ScreenSizes[BestMode].height;

    XRRConfigCurrentConfiguration(FXRRConfig, @CurrentRotation);

    X11CheckSuccess(XRRSetScreenConfig(FDisplay, FXRRConfig, FRoot, BestMode,
                                       CurrentRotation, CurrentTime),
                    'XRRSetScreenConfig failed');

    FInMode := True;
  finally
    DestroyScreenConfig;
  end;
end;

procedure TX11ModesXrandr.SaveCurrentMode;
var
  SizeIndex: TSizeID;
  ScreenSizes: PXRRScreenSize;
  ScreenSizesNum: cint;
begin
  LOG('xrandr: saving previous mode');

  SizeIndex := XRRConfigCurrentConfiguration(FXRRConfig, @FSavedMode.Rotation);
  FSavedMode.Rate := XRRConfigCurrentRate(FXRRConfig);

  ScreenSizes := XRRConfigSizes(FXRRConfig, @ScreenSizesNum);
  if (SizeIndex < 0) or (SizeIndex >= ScreenSizesNum) then
    raise TPTCError.Create('XRRConfigCurrentConfiguration returned a size index outside the range of screen sizes');
  FSavedMode.ScreenSize := ScreenSizes[SizeIndex];
end;

procedure TX11ModesXrandr.RestorePreviousMode;
var
  I: Integer;
  SizeIndex: cint;
  ScreenSizes: PXRRScreenSize;
  ScreenSizesNum: cint;
begin
  if not FInMode then
    exit;

  LOG('xrandr: restoring previous mode');

  CreateScreenConfig;
  try
    SizeIndex := -1;
    ScreenSizes := XRRConfigSizes(FXRRConfig, @ScreenSizesNum);
    for I := 0 to ScreenSizesNum - 1 do
      if (ScreenSizes[I].width   = FSavedMode.ScreenSize.width  ) and
         (ScreenSizes[I].height  = FSavedMode.ScreenSize.height ) and
         (ScreenSizes[I].mwidth  = FSavedMode.ScreenSize.mwidth ) and
         (ScreenSizes[I].mheight = FSavedMode.ScreenSize.mheight) then
      begin
        SizeIndex := I;
        Break;
      end;
    if SizeIndex = -1 then
      for I := 0 to ScreenSizesNum - 1 do
        if (ScreenSizes[I].width  = FSavedMode.ScreenSize.width ) and
           (ScreenSizes[I].height = FSavedMode.ScreenSize.height) then
        begin
          SizeIndex := I;
          Break;
        end;
    if SizeIndex = -1 then
      raise TPTCError.Create('xrandr: saved mode size not found in size list');

    X11CheckSuccess(XRRSetScreenConfigAndRate(FDisplay,
                                              FXRRConfig,
                                              FRoot,
                                              SizeIndex,
                                              FSavedMode.Rotation,
                                              FSavedMode.Rate,
                                              CurrentTime),
                    'XRRSetScreenConfigAndRate failed');

    FInMode := False;
  finally
    DestroyScreenConfig;
  end;
end;

function TX11ModesXrandr.GetWidth: Integer;
begin
  Result := FWidth;
end;

>>>>>>> origin/cpstrnew
function TX11ModesXrandr.GetHeight: Integer;
begin
  Result := FHeight;
end;
{$ENDIF ENABLE_X11_EXTENSION_XRANDR}

{$IFDEF ENABLE_X11_EXTENSION_XF86VIDMODE}
constructor TX11ModesXF86VidMode.Create(ADisplay: PDisplay; AScreen: Integer);
var
  dummy1, dummy2: cint;
begin
  inherited;

  FSavedMode := nil;
  FSavedDotClock := 0;
  FModeList := nil;
  FModeListCount := 0;

  if not XF86VidModeQueryExtension(FDisplay, @dummy1, @dummy2) then
    raise TPTCError.Create('VidMode extension not available');
end;

destructor TX11ModesXF86VidMode.Destroy;
begin
  if FSavedMode <> nil then
  begin
    RestorePreviousMode;
    if FSavedMode^.privsize <> 0 then
      XFree(FSavedMode^.c_private);
    Dispose(FSavedMode);
  end;

  if FModeList <> nil then
    XFree(FModeList);

  inherited Destroy;
end;

{todo: move the saving of the previous mode to a separate function...}
procedure TX11ModesXF86VidMode.RetrieveModeList;
begin
  { if we have been called before, do nothing }
  if FModeList <> nil then
    exit;
=======
=======
>>>>>>> origin/fixes_2_2
  FXRRConfig := XRRGetScreenInfo(FDisplay, FRoot);
  If FXRRConfig = Nil Then
    Raise TPTCError.Create('XRRGetScreenInfo failed');

  Raise TPTCError.Create('Xrandr mode switcher is not yet implemented...');
End;

Destructor TX11ModesXrandr.Destroy;

Begin
  If FXRRConfig <> Nil Then
    XRRFreeScreenConfigInfo(FXRRConfig);

  Inherited;
End;

Procedure TX11ModesXrandr.GetModes(Var AModes : TPTCModeDynArray; ACurrentDesktopFormat : TPTCFormat);

Begin
  {...}
End;

Procedure TX11ModesXrandr.SetBestMode(AWidth, AHeight : Integer);

Begin
  {todo...}
End;

Procedure TX11ModesXrandr.RestorePreviousMode;

Begin
  {todo...}
End;

Function TX11ModesXrandr.GetWidth : Integer;

Begin
  // todo...
End;

Function TX11ModesXrandr.GetHeight : Integer;

Begin
  // todo...
End;
{$ENDIF ENABLE_X11_EXTENSION_XRANDR}

{$IFDEF ENABLE_X11_EXTENSION_XF86VIDMODE}
Constructor TX11ModesXF86VidMode.Create(ADisplay : PDisplay; AScreen : Integer);

Var
  dummy1, dummy2 : cint;

Begin
  Inherited;

  FSavedMode := Nil;
  FSavedDotClock := 0;
  FModeList := Nil;
  FModeListCount := 0;

  If Not XF86VidModeQueryExtension(FDisplay, @dummy1, @dummy2) Then
    Raise TPTCError.Create('VidMode extension not available');
End;

Destructor TX11ModesXF86VidMode.Destroy;

Begin
  If FSavedMode <> Nil Then
  Begin
    RestorePreviousMode;
    If FSavedMode^.privsize <> 0 Then
      XFree(FSavedMode^.c_private);
    Dispose(FSavedMode);
  End;

  If FModeList <> Nil Then
    XFree(FModeList);

  Inherited Destroy;
End;

{todo: move the saving of the previous mode to a separate function...}
Procedure TX11ModesXF86VidMode.RetrieveModeList;

Begin
  { If we have been called before, do nothing }
  If FModeList <> Nil Then
    Exit;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { Save previous mode }
  New(FSavedMode);
  FillChar(FSavedMode^, SizeOf(FSavedMode^), 0);
  XF86VidModeGetModeLine(FDisplay, FScreen, @FSavedDotClock, FSavedMode);

  { Get all available video modes }
  XF86VidModeGetAllModeLines(FDisplay, FScreen, @FModeListCount, @FModeList);
<<<<<<< HEAD
<<<<<<< HEAD
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11ModesXF86VidMode.GetModes(var AModes: TPTCModeList; ACurrentDesktopFormat: IPTCFormat);
=======
procedure TX11ModesXF86VidMode.GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11ModesXF86VidMode.GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11ModesXF86VidMode.GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11ModesXF86VidMode.GetModes(var AModes: TPTCModeDynArray; ACurrentDesktopFormat: TPTCFormat);
>>>>>>> origin/cpstrnew
var
  I: Integer;
begin
  RetrieveModeList;

<<<<<<< HEAD
  SetLength(AModes, FModeListCount);
=======
  SetLength(AModes, FModeListCount + 1);
  AModes[FModeListCount] := TPTCMode.Create;
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
  for I := 0 to FModeListCount - 1 do
    AModes[I] := TPTCMode.Create(FModeList[I]^.hdisplay, FModeList[I]^.vdisplay, ACurrentDesktopFormat);
end;

function TX11ModesXF86VidMode.FindNumberOfBestMode(AWidth, AHeight: Integer): Integer;
var
  min_diff: Integer;
  d_x, d_y: Integer;
  found_mode: Integer;
  I: Integer;
begin
  { try an exact match }
  for I := 0 to FModeListCount - 1 do
    if (FModeList[I]^.hdisplay = AWidth) and (FModeList[I]^.vdisplay = AHeight) then
      exit(I);

  { try to find a mode that matches the width first }
  for I := 0 to FModeListCount - 1 do
    if (FModeList[I]^.hdisplay = AWidth) and (FModeList[I]^.vdisplay >= AHeight) then
      exit(I);

  { Next try to match the height }
  for I := 0 to FModeListCount - 1 do
    if (FModeList[I]^.hdisplay >= AWidth) and (FModeList[I]^.vdisplay = AHeight) then
      exit(I);
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TX11ModesXF86VidMode.GetModes(Var AModes : TPTCModeDynArray; ACurrentDesktopFormat : TPTCFormat);

Var
  I : Integer;

Begin
  RetrieveModeList;

  SetLength(AModes, FModeListCount + 1);
  AModes[FModeListCount] := TPTCMode.Create;
  For I := 0 To FModeListCount - 1 Do
    AModes[I] := TPTCMode.Create(FModeList[I]^.hdisplay, FModeList[I]^.vdisplay, ACurrentDesktopFormat);
End;

Function TX11ModesXF86VidMode.FindNumberOfBestMode(AWidth, AHeight : Integer) : Integer;

Var
  min_diff : Integer;
  d_x, d_y : Integer;
  found_mode : Integer;
  I : Integer;

Begin
  { Try an exact match }
  For I := 0 To FModeListCount - 1 Do
    If (FModeList[I]^.hdisplay = AWidth) And (FModeList[I]^.vdisplay = AHeight) Then
      Exit(I);

  { Try to find a mode that matches the width first }
  For I := 0 To FModeListCount - 1 Do
    If (FModeList[I]^.hdisplay = AWidth) And (FModeList[I]^.vdisplay >= AHeight) Then
      Exit(I);

  { Next try to match the height }
  For I := 0 To FModeListCount - 1 Do
    If (FModeList[I]^.hdisplay >= AWidth) And (FModeList[I]^.vdisplay = AHeight) Then
      Exit(I);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { Finally, find the mode that is bigger than the requested one and makes }
  { the least difference }
  found_mode := -1;
  min_diff := High(Integer);
<<<<<<< HEAD
<<<<<<< HEAD
  for I := 0 to FModeListCount - 1 do
    if (FModeList[I]^.hdisplay >= AWidth) and (FModeList[I]^.vdisplay >= AHeight) then
    begin
      d_x := Sqr(FModeList[I]^.hdisplay - AWidth);
      d_y := Sqr(FModeList[I]^.vdisplay - AHeight);
      if (d_x + d_y) < min_diff then
      begin
        min_diff := d_x + d_y;
        found_mode := I;
      end;
    end;

  if found_mode <> -1 then
    Result := found_mode
  else
    raise TPTCError.Create('Cannot find matching video mode');
end;

procedure TX11ModesXF86VidMode.SetBestMode(AWidth, AHeight: Integer);
var
  BestMode: Integer;
begin
  RetrieveModeList;

  BestMode := FindNumberOfBestMode(AWidth, AHeight);
  if not XF86VidModeSwitchToMode(FDisplay, FScreen, FModeList[BestMode]) then
    raise TPTCError.Create('Error switching to the requested video mode');
=======
=======
>>>>>>> origin/fixes_2_2
  For I := 0 To FModeListCount - 1 Do
    If (FModeList[I]^.hdisplay >= AWidth) And (FModeList[I]^.vdisplay >= AHeight) Then
    Begin
      d_x := Sqr(FModeList[I]^.hdisplay - AWidth);
      d_y := Sqr(FModeList[I]^.vdisplay - AHeight);
      If (d_x + d_y) < min_diff Then
      Begin
        min_diff := d_x + d_y;
        found_mode := I;
      End;
    End;

  If found_mode <> -1 Then
    Result := found_mode
  Else
    Raise TPTCError.Create('Cannot find matching video mode');
End;

Procedure TX11ModesXF86VidMode.SetBestMode(AWidth, AHeight : Integer);

Var
  BestMode : Integer;

Begin
  RetrieveModeList;

  BestMode := FindNumberOfBestMode(AWidth, AHeight);
  If Not XF86VidModeSwitchToMode(FDisplay, FScreen, FModeList[BestMode]) Then
    Raise TPTCError.Create('Error switching to the requested video mode');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  FWidth := FModeList[BestMode]^.hdisplay;
  FHeight := FModeList[BestMode]^.vdisplay;

  XWarpPointer(FDisplay, None, RootWindow(FDisplay, FScreen), 0, 0, 0, 0,
<<<<<<< HEAD
<<<<<<< HEAD
               FWidth div 2,
               FHeight div 2);

  if not XF86VidModeSetViewPort(FDisplay, FScreen, 0, 0) then
    raise TPTCError.Create('Error moving the viewport to the upper-left corner');
end;

procedure TX11ModesXF86VidMode.RestorePreviousMode;
var
  ModeInfo: TXF86VidModeModeInfo;
begin
  if FSavedMode <> nil then
  begin
=======
=======
>>>>>>> origin/fixes_2_2
               FWidth Div 2,
               FHeight Div 2);

  If Not XF86VidModeSetViewPort(FDisplay, FScreen, 0, 0) Then
    Raise TPTCError.Create('Error moving the viewport to the upper-left corner');
End;

Procedure TX11ModesXF86VidMode.RestorePreviousMode;

Var
  ModeInfo : TXF86VidModeModeInfo;

Begin
  If FSavedMode <> Nil Then
  Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    {FSavedMode is a TXF86VidModeModeLine, but XF86VidModeSwitchToMode wants a
                     TXF86VidModeModeInfo :}
    FillChar(ModeInfo, SizeOf(ModeInfo), 0);
    ModeInfo.dotclock := FSavedDotClock;
    ModeInfo.hdisplay := FSavedMode^.hdisplay;
    ModeInfo.hsyncstart := FSavedMode^.hsyncstart;
    ModeInfo.hsyncend := FSavedMode^.hsyncend;
    ModeInfo.htotal := FSavedMode^.htotal;
    ModeInfo.vdisplay := FSavedMode^.vdisplay;
    ModeInfo.vsyncstart := FSavedMode^.vsyncstart;
    ModeInfo.vsyncend := FSavedMode^.vsyncend;
    ModeInfo.vtotal := FSavedMode^.vtotal;
    ModeInfo.flags := FSavedMode^.flags;
    ModeInfo.privsize := FSavedMode^.privsize;
    ModeInfo.c_private := FSavedMode^.c_private;

    XF86VidModeSwitchToMode(FDisplay, FScreen, @ModeInfo);
<<<<<<< HEAD
<<<<<<< HEAD
  end;
end;

function TX11ModesXF86VidMode.GetWidth: Integer;
begin
  Result := FWidth;
end;

function TX11ModesXF86VidMode.GetHeight: Integer;
begin
  Result := FHeight;
end;
=======
=======
>>>>>>> origin/fixes_2_2
  End;
End;

Function TX11ModesXF86VidMode.GetWidth : Integer;

Begin
  Result := FWidth;
End;

Function TX11ModesXF86VidMode.GetHeight : Integer;

Begin
  Result := FHeight;
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF ENABLE_X11_EXTENSION_XF86VIDMODE}
