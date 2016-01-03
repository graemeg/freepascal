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

{$IFDEF ENABLE_X11_EXTENSION_XF86DGA2}

constructor TX11DGA2Display.Create(ADisplay: PDisplay; AScreen: Integer; const AFlags: TX11Flags);
var
  MajorVersion, MinorVersion: cint;
begin
  inherited;
=======
=======
>>>>>>> origin/fixes_2_2
{$IFDEF ENABLE_X11_EXTENSION_XF86DGA2}

Constructor TX11DGA2Display.Create(ADisplay : PDisplay; AScreen : Integer; Const AFlags : TX11Flags);

Var
  dummy1, dummy2 : cint;

Begin
  Inherited;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  LOG('trying to open a DGA 2.0 display');

  { Check if the DGA extension can be used }
  LOG('checking if the DGA extension can be used (XDGAQueryExtension)');
<<<<<<< HEAD
<<<<<<< HEAD
  if not XDGAQueryExtension(FDisplay, @FXDGAEventBase, @FXDGAErrorBase) then
    raise TPTCError.Create('DGA extension not available');
<<<<<<< HEAD

  LOG('checking DGA version (XDGAQueryVersion)');
  if not XDGAQueryVersion(FDisplay, @MajorVersion, @MinorVersion) then
    raise TPTCError.Create('Unable to query DGA version');

  LOG('DGA version ' + IntToStr(MajorVersion) + '.' + IntToStr(MinorVersion));

=======

  LOG('checking DGA version (XDGAQueryVersion)');
  if not XDGAQueryVersion(FDisplay, @MajorVersion, @MinorVersion) then
    raise TPTCError.Create('Unable to query DGA version');

  LOG('DGA version ' + IntToStr(MajorVersion) + '.' + IntToStr(MinorVersion));

>>>>>>> origin/cpstrnew
  if MajorVersion < 2 then
    raise TPTCError.Create('DGA version older than 2.0');
end;

destructor TX11DGA2Display.Destroy;
begin
  Close;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11DGA2Display.Open(ATitle: string; AWidth, AHeight: Integer; AFormat: IPTCFormat; const AOpenGLAttributes: IPTCOpenGLAttributes);
=======
procedure TX11DGA2Display.open(ATitle: string; AWidth, AHeight: Integer; const AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11DGA2Display.open(ATitle: string; AWidth, AHeight: Integer; const AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11DGA2Display.open(ATitle: string; AWidth, AHeight: Integer; const AFormat: TPTCFormat);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11DGA2Display.open(ATitle: string; AWidth, AHeight: Integer; const AFormat: TPTCFormat);
>>>>>>> origin/cpstrnew

  function ModeFlagsToString(AFlags: cint): string;
  begin
    Result := '';
    if (AFlags and XDGAConcurrentAccess) <> 0 then
      Result := Result + ' XDGAConcurrentAccess';
    if (AFlags and XDGASolidFillRect) <> 0 then
      Result := Result + ' XDGASolidFillRect';
    if (AFlags and XDGABlitRect) <> 0 then
      Result := Result + ' XDGABlitRect';
    if (AFlags and XDGABlitTransRect) <> 0 then
      Result := Result + ' XDGABlitTransRect';
    if (AFlags and XDGAPixmap) <> 0 then
      Result := Result + ' XDGAPixmap';
    if (AFlags and XDGAInterlaced) <> 0 then
      Result := Result + ' XDGAInterlaced';
    if (AFlags and XDGADoublescan) <> 0 then
      Result := Result + ' XDGADoublescan';
    if Result <> '' then
      Delete(Result, 1, 1);
  end;
  
  function ViewportFlagsToString(AViewportFlags: cint): string;
  begin
    Result := '';
    if (AViewportFlags and XDGAFlipRetrace) <> 0 then
      Result := Result + ' XDGAFlipRetrace';
    if (AViewportFlags and XDGAFlipImmediate) <> 0 then
      Result := Result + ' XDGAFlipImmediate';
    if Result <> '' then
      Delete(Result, 1, 1);
  end;
  
  function ByteOrderToString(AByteOrder: cint): string;
  begin
    case AByteOrder of
      LSBFirst: Result := ' (LSBFirst)';
      MSBFirst: Result := ' (MSBFirst)';
    else
      Result := '';
    end;
  end;

  function VisualClassToString(AVisualClass: cshort): string;
  begin
    case AVisualClass of
      StaticGray: Result := ' (StaticGray)';
      GrayScale: Result := ' (GrayScale)';
      StaticColor: Result := ' (StaticColor)';
      PseudoColor: Result := ' (PseudoColor)';
      TrueColor: Result := ' (TrueColor)';
      DirectColor: Result := ' (DirectColor)';
    else
      Result := '';
    end;
  end;

  function FindMode: Integer;
  var
    I: Integer;
  begin
    { this function is currently a hack }
    { todo: implement generic algorithm for mode selection }

    for I := 0 to FXDGAModesNum - 1 do
    begin
      with FXDGAModes[I] do
      begin
        if (viewportWidth = AWidth) and (viewportHeight = AHeight) and
          (bitsPerPixel = AFormat.Bits) then
          exit(num);
      end;
    end;

    for I := 0 to FXDGAModesNum - 1 do
    begin
      with FXDGAModes[I] do
      begin
        if (viewportWidth = AWidth) and (viewportHeight = AHeight) then
          exit(num);
      end;
    end;

    Result := 1;  // todo: fix...
  end;

var
  i: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
//  e: TXEvent;
=======
  e: TXEvent;
  tmpArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
  e: TXEvent;
  tmpArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
  e: TXEvent;
  tmpArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
  e: TXEvent;
  tmpArea: TPTCArea;
>>>>>>> origin/cpstrnew
  r, g, b: Single;
  found_mode: Integer;
begin
  FWidth := AWidth;
  FHeight := AHeight;

  LOG('trying to open framebuffer (XDGAOpenFramebuffer)');
  if not XDGAOpenFramebuffer(FDisplay, FScreen) then
    raise TPTCError.Create('Cannot open framebuffer - insufficient privileges?');
  FFramebufferIsOpen := True;

  { Get all available video modes }
=======
=======
>>>>>>> origin/fixes_2_2
  If Not XDGAQueryExtension(FDisplay, @dummy1, @dummy2) Then
    Raise TPTCError.Create('DGA extension not available');
End;

Destructor TX11DGA2Display.Destroy;

Begin
  Close;
  Inherited Destroy;
End;

Procedure TX11DGA2Display.open(title : String; _width, _height : Integer; Const _format : TPTCFormat);

Var
  vml : PXF86VidModeModeLine;
  dotclock : Integer;
  i : Integer;
  found : Boolean;
  root : TWindow;
  e : TXEvent;
  tmpArea : TPTCArea;
  r, g, b : Single;
  found_mode : Integer;
  min_diff : Integer;
  d_x, d_y : Integer;

Begin
  FWidth := _width;
  FHeight := _height;

  LOG('trying to open framebuffer (XDGAOpenFramebuffer)');
  If Not XDGAOpenFramebuffer(FDisplay, FScreen) Then
    Raise TPTCError.Create('Cannot open framebuffer - insufficient privileges?');
  FFramebufferIsOpen := True;

  { Get all availabe video modes }
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  LOG('querying available display modes (XDGAQueryModes)');
  FXDGAModes := XDGAQueryModes(FDisplay, FScreen, @FXDGAModesNum);

  LOG('number of display modes', FXDGAModesNum);

<<<<<<< HEAD
<<<<<<< HEAD
  for I := 0 to FXDGAModesNum - 1 do
  begin
    LOG('mode#', I);
    with FXDGAModes[I] do
    begin
      LOG('num', num);
      LOG('name', name);
      LOG('verticalRefresh', verticalRefresh);
      LOG('flags', IntToStr(flags) + ' (' + ModeFlagsToString(flags) + ')');
      LOG('imageWidth', imageWidth);
      LOG('imageHeight', imageHeight);
      LOG('pixmapWidth', pixmapWidth);
      LOG('pixmapHeight', pixmapHeight);
      LOG('bytesPerScanline', bytesPerScanline);
      LOG('byteOrder', IntToStr(byteOrder) + ByteOrderToString(byteOrder));
      LOG('depth', depth);
      LOG('bitsPerPixel', bitsPerPixel);
      LOG('redMask', '$' + HexStr(redMask, 8));
      LOG('greenMask', '$' + HexStr(greenMask, 8));
      LOG('blueMask', '$' + HexStr(blueMask, 8));
      LOG('visualClass', IntToStr(visualClass) + VisualClassToString(visualClass));
      LOG('viewportWidth', viewportWidth);
      LOG('viewportHeight', viewportHeight);
      LOG('xViewportStep', xViewportStep);
      LOG('yViewportStep', yViewportStep);
      LOG('maxViewportX', maxViewportX);
      LOG('maxViewportY', maxViewportY);
      LOG('viewportFlags', IntToStr(viewportFlags) + ' (' + ViewportFlagsToString(viewportFlags) + ')');
      LOG('reserved1', reserved1);
      LOG('reserved2', reserved2);
      LOG('----------');
    end;
  end;

  found_mode := FindMode;

  FXDGADevice := XDGASetMode(FDisplay, FScreen, found_mode);
  if FXDGADevice = nil then
    raise TPTCError.Create('XDGASetMode failed (returned nil)');
  if FXDGADevice^.data = nil then
    raise TPTCError.Create('The pointer to the framebuffer, returned by XDGA is nil?!');
  FModeIsSet := True;

  if FWidth > FXDGADevice^.mode.viewportWidth then
    FWidth := FXDGADevice^.mode.viewportWidth;
  if FHeight > FXDGADevice^.mode.viewportHeight then
    FHeight := FXDGADevice^.mode.viewportHeight;

  FDestX := (FXDGADevice^.mode.viewportWidth div 2) - (FWidth div 2);
  FDestY := (FXDGADevice^.mode.viewportHeight div 2) - (FHeight div 2);

  { Check if the requested colour mode is available }
  FFormat := GetX11Format(AFormat);

(*  { Grab exclusive control over the keyboard and mouse }
  root := XRootWindow(FDisplay, FScreen);
  XGrabKeyboard(FDisplay, root, True, GrabModeAsync, GrabModeAsync, CurrentTime);
  XGrabPointer(FDisplay, root, True, PointerMotionMask or ButtonPressMask or
               ButtonReleaseMask, GrabModeAsync, GrabModeAsync, None, None,
               CurrentTime);*)
=======
=======
>>>>>>> origin/fixes_2_2
  For I := 0 To FXDGAModesNum - 1 Do
  Begin
    LOG('mode#', I);
    LOG('num', FXDGAModes[I].num);
    LOG('name: ' + FXDGAModes[I].name);
  End;

  found_mode := 0; // todo: find a video mode

  Raise TPTCError.Create('break! dga 2.0 code unfinished');

  FXDGADevice := XDGASetMode(FDisplay, FScreen, found_mode);
  If FXDGADevice = Nil Then
    Raise TPTCError.Create('XDGASetMode failed (returned nil)');
  If FXDGADevice^.data = Nil Then
    Raise TPTCError.Create('The pointer to the framebuffer, returned by XDGA is nil?!');
  FModeIsSet := True;

  { Check if the requested colour mode is available }
  FFormat := GetX11Format(_format);

  { Grab exclusive control over the keyboard and mouse }
{  root := XRootWindow(FDisplay, FScreen);
  XGrabKeyboard(FDisplay, root, True, GrabModeAsync, GrabModeAsync, CurrentTime);
  XGrabPointer(FDisplay, root, True, PointerMotionMask Or ButtonPressMask Or
               ButtonReleaseMask, GrabModeAsync, GrabModeAsync, None, None,
               CurrentTime);}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  XFlush(FDisplay);

  { Get Display information }
{  XF86DGAGetVideo(FDisplay, FScreen, @dga_addr, @dga_linewidth,
                  @dga_banksize, @dga_memsize);}

  { Don't have to be root anymore }
{  setuid(getuid);...}

//  XF86DGAGetViewPortSize(FDisplay, FScreen, @dga_width, @dga_height);

<<<<<<< HEAD
<<<<<<< HEAD
{  if XF86DGAForkApp(FScreen) <> 0 then
    raise TPTCError.Create('cannot do safety fork')
  else
  begin
    if XF86DGADirectVideo(FDisplay, FScreen, XF86DGADirectGraphics or
      XF86DGADirectKeyb or XF86DGADirectMouse) = 0 then
      raise TPTCError.Create('cannot switch to DGA mode');
  end;}

//  m_indirect := True;
//  FillChar(dga_addr^, dga_linewidth * dga_height * (FFormat.bits div 8), 0);

  XDGASelectInput(FDisplay, FScreen, KeyPressMask or KeyReleaseMask or
                                     ButtonPressMask or ButtonReleaseMask or
                                     PointerMotionMask);

  { Important.. sort of =) }
//  XDGASetViewport(FDisplay, FScreen, 0, 0, {XDGAFlipImmediate}XDGAFlipRetrace);

(*  found := False;
  repeat
    { Stupid loop. The key }
    { events were causing }
    { problems.. }
    found := XCheckMaskEvent(FDisplay, KeyPressMask or KeyReleaseMask, @e);
  until not found;*)

  { Create colour map in 8 bit mode }
  if FFormat.bits = 8 {todo:fix!} then
  begin
    FColours := GetMem(256 * SizeOf(TXColor));
    if FColours = nil then
      raise TPTCError.Create('Cannot allocate colour map cells');
//    FCMap := XCreateColormap(FDisplay, RootWindow(FDisplay, FScreen),
//                              DefaultVisual(FDisplay, FScreen), AllocAll);
    FCMap := XDGACreateColormap(FDisplay, FScreen, FXDGADevice, AllocAll);
    if FCMap = 0 then
      raise TPTCError.Create('Cannot create colour map');
  end
  else
    FCMap := 0;

  { Set 332 palette, for now }
  if (FFormat.bits = 8) and FFormat.direct then
  begin
    {Taken from PTC 0.72, i hope it's fine}
    for i := 0 to 255 do
    begin
      r := ((i and $E0) shr 5) * 255 / 7;
      g := ((i and $1C) shr 2) * 255 / 7;
      b := (i and $03) * 255 / 3;

      FColours[i].pixel := i;

      FColours[i].red := Round(r) shl 8;
      FColours[i].green := Round(g) shl 8;
      FColours[i].blue := Round(b) shl 8;

      Byte(FColours[i].flags) := DoRed or DoGreen or DoBlue;
    end;
    XStoreColors(FDisplay, FCMap, FColours, 256);
    XDGAInstallColormap(FDisplay, FScreen, FCMap);
  end;

  { Set clipping area }
<<<<<<< HEAD
  FClip := TPTCArea.Create(0, 0, FWidth, FHeight);
=======
  tmpArea := TPTCArea.Create(0, 0, FWidth, FHeight);
  try
    FClip.Assign(tmpArea);
  finally
    tmpArea.Free;
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

  XDGASync(FDisplay, FScreen);

  FillChar(FXDGADevice^.data^,
    FXDGADevice^.mode.bytesPerScanline * FXDGADevice^.mode.imageHeight, 0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  FOpen := True;
end;

{ not in DGA mode }
procedure TX11DGA2Display.open(w: TWindow; _format: IPTCFormat);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

{ not in DGA mode }
procedure TX11DGA2Display.open(w: TWindow; const _format: TPTCFormat);
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
  if w = 0 then; { Prevent warnings }
  if _format = nil then;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11DGA2Display.open(_window: TWindow; _format: IPTCFormat; x, y, w, h: Integer);
=======
procedure TX11DGA2Display.open(_window: TWindow; const _format: TPTCFormat; x, y, w, h: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11DGA2Display.open(_window: TWindow; const _format: TPTCFormat; x, y, w, h: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11DGA2Display.open(_window: TWindow; const _format: TPTCFormat; x, y, w, h: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11DGA2Display.open(_window: TWindow; const _format: TPTCFormat; x, y, w, h: Integer);
>>>>>>> origin/cpstrnew
begin
  if (_window = 0) or (_format = nil) or (x = 0) or
     (y = 0) or (w = 0) or (h = 0) then;
end;

procedure TX11DGA2Display.close;
var
  tmp: Pointer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FOpen := False;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if FModeIsSet then
  begin
=======
=======
>>>>>>> origin/fixes_2_2
{  If XF86DGAForkApp(FScreen) <> 0 Then
    Raise TPTCError.Create('cannot do safety fork')
  Else
  Begin
    If XF86DGADirectVideo(FDisplay, FScreen, XF86DGADirectGraphics Or
      XF86DGADirectKeyb Or XF86DGADirectMouse) = 0 Then
      Raise TPTCError.Create('cannot switch to DGA mode');
  End;}

//  m_indirect := True;
//  FillChar(dga_addr^, dga_linewidth * dga_height * (FFormat.bits Div 8), 0);

  XSelectInput(FDisplay, DefaultRootWindow(FDisplay),
               KeyPressMask Or KeyReleaseMask);

  XF86DGASetViewPort(FDisplay, FScreen, 0, 0); { Important.. sort of =) }

  found := False;
  Repeat
    { Stupid loop. The key }
    { events were causing }
    { problems.. }
    found := XCheckMaskEvent(FDisplay, KeyPressMask Or KeyReleaseMask, @e);
  Until Not found;

  { Create colour map in 8 bit mode }
  If FFormat.bits = 8 Then
  Begin
    FColours := GetMem(256 * SizeOf(TXColor));
    If FColours = Nil Then
      Raise TPTCError.Create('Cannot allocate colour map cells');
    FCMap := XCreateColormap(FDisplay, RootWindow(FDisplay, FScreen),
                              DefaultVisual(FDisplay, FScreen), AllocAll);
    If FCMap = 0 Then
      Raise TPTCError.Create('Cannot create colour map');
  End
  Else
    FCMap := 0;

  { Set 332 palette, for now }
  If (FFormat.bits = 8) And FFormat.direct Then
  Begin
    {Taken from PTC 0.72, i hope it's fine}
    For i := 0 To 255 Do
    Begin
      r := ((i And $E0) Shr 5) * 255 / 7;
      g := ((i And $1C) Shr 2) * 255 / 7;
      b := (i And $03) * 255 / 3;

      FColours[i].pixel := i;

      FColours[i].red := Round(r) Shl 8;
      FColours[i].green := Round(g) Shl 8;
      FColours[i].blue := Round(b) Shl 8;

      Byte(FColours[i].flags) := DoRed Or DoGreen Or DoBlue;
    End;
    XStoreColors(FDisplay, FCMap, FColours, 256);
    XF86DGAInstallColormap(FDisplay, FScreen, FCMap);
  End;

  { Set clipping area }
  tmpArea := TPTCArea.Create(0, 0, FWidth, FHeight);
  Try
    FClip.Assign(tmpArea);
  Finally
    tmpArea.Free;
  End;
End;

{ Not in DGA mode }
Procedure TX11DGA2Display.open(w : TWindow; Const _format : TPTCFormat);

Begin
  If w = 0 Then; { Prevent warnings }
  If _format = Nil Then;
End;

Procedure TX11DGA2Display.open(_window : TWindow; Const _format : TPTCFormat; x, y, w, h : Integer);

Begin
  If (_window = 0) Or (_format = Nil) Or (x = 0) Or
     (y = 0) Or (w = 0) Or (h = 0) Then;
End;

Procedure TX11DGA2Display.close;

Var
  tmp : Pointer;

Begin
  If FModeIsSet Then
  Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FModeIsSet := False;

    { restore the original mode }
    XDGASetMode(FDisplay, FScreen, 0); { returns PXDGADevice }
{    XUngrabKeyboard(FDisplay, CurrentTime);
    XUngrabPointer(FDisplay, CurrentTime);}
<<<<<<< HEAD
<<<<<<< HEAD
  end;

  if FFramebufferIsOpen then
  begin
    FFramebufferIsOpen := False;
    XDGACloseFramebuffer(FDisplay, FScreen);
  end;

  if FDisplay <> nil then
    XFlush(FDisplay);

  if FCMap <> 0 then
  begin
    XFreeColormap(FDisplay, FCMap);
    FCMap := 0;
  end;

  FreeMemAndNil(FColours);

  if FXDGADevice <> nil then
  begin
    tmp := FXDGADevice;
    FXDGADevice := nil;
    XFree(tmp);
  end;

  if FXDGAModes <> nil then
  begin
    tmp := FXDGAModes;
    FXDGAModes := nil;
    XFree(tmp);
  end;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
procedure TX11DGA2Display.GetModes(var AModes: TPTCModeList);
begin
  SetLength(AModes, 0);
  {todo...}
end;

procedure TX11DGA2Display.Update;
begin
end;

procedure TX11DGA2Display.Update(AArea: IPTCArea);
=======
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
procedure TX11DGA2Display.GetModes(var AModes: TPTCModeDynArray);
begin
  SetLength(AModes, 1);
  AModes[0] := TPTCMode.Create;
  {todo...}
end;

=======

procedure TX11DGA2Display.GetModes(var AModes: TPTCModeDynArray);
begin
  SetLength(AModes, 1);
  AModes[0] := TPTCMode.Create;
  {todo...}
end;

>>>>>>> origin/cpstrnew
procedure TX11DGA2Display.update;
begin
end;

procedure TX11DGA2Display.update(const _area: TPTCArea);
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

function TX11DGA2Display_MatchAnyEvent(display: PDisplay; event: PXEvent; arg: TXPointer): LongBool; cdecl;
begin
  Result := LongBool(1);
end;

procedure TX11DGA2Display.HandleEvents;
var
  e: TXEvent;
  NewFocus: Boolean;
  NewFocusSpecified: Boolean;

  function UsefulEventsPending: Boolean;
  var
    tmpEvent: TXEvent;
  begin
    if XCheckIfEvent(FDisplay, @tmpEvent, @TX11DGA2Display_MatchAnyEvent, nil) then
    begin
      Result := True;
      XPutBackEvent(FDisplay, @tmpEvent);
      exit;
    end;

    Result := False;
  end;

  procedure HandleXDGAKeyEvent;
  var
    XDGAKeyEvent: TXDGAKeyEvent absolute e;
    XKeyEvent: TXKeyEvent;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  begin
    XDGAKeyEventToXKeyEvent(@XDGAKeyEvent, @XKeyEvent);
    HandleKeyEvent(XKeyEvent);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    sym: TKeySym;
    sym_modded: TKeySym; { modifiers like shift are taken into account here }
    press: Boolean;
    alt, shift, ctrl: Boolean;
    uni: Integer;
    key: TPTCKeyEvent;
    buf: array [1..16] of Char;
  begin
    XDGAKeyEventToXKeyEvent(@XDGAKeyEvent, @XKeyEvent);
    sym := XLookupKeySym(@XKeyEvent, 0);
    XLookupString(@XKeyEvent, @buf, SizeOf(buf), @sym_modded, nil);
    uni := X11ConvertKeySymToUnicode(sym_modded);
    alt := (XDGAKeyEvent.state and Mod1Mask) <> 0;
    shift := (XDGAKeyEvent.state and ShiftMask) <> 0;
    ctrl := (XDGAKeyEvent.state and ControlMask) <> 0;
    if XDGAKeyEvent._type = (KeyPress + FXDGAEventBase) then
      press := True
    else
      press := False;

    key := nil;
    case sym shr 8 of
      0: key := TPTCKeyEvent.Create(FNormalKeys[sym and $FF], uni, alt, shift, ctrl, press);
      $FF: key := TPTCKeyEvent.Create(FFunctionKeys[sym and $FF], uni, alt, shift, ctrl, press);
      else
        key := TPTCKeyEvent.Create;
    end;
    FEventQueue.AddEvent(key);
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

begin
  NewFocusSpecified := False;
  while UsefulEventsPending do
  begin
    XNextEvent(FDisplay, @e);
(*    case e._type of
      FocusIn: begin
        NewFocus := True;
        NewFocusSpecified := True;
      end;
      FocusOut: begin
        NewFocus := False;
        NewFocusSpecified := True;
      end;
      ClientMessage: begin
{        if (e.xclient.format = 32) and (TAtom(e.xclient.data.l[0]) = m_atom_close) then
          Halt(0);}
      end;
      Expose: begin
        {...}
      end;
    end;*)
    case e._type - FXDGAEventBase of
      KeyPress, KeyRelease: HandleXDGAKeyEvent;
      ButtonPress, ButtonRelease: begin
        {...}
      end;
      MotionNotify: begin
        {...}
      end;
    end;
  end;
//  HandleChangeFocus(NewFocus);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11DGA2Display.NextEvent(out event: IPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean;
var
  tmpEvent: TXEvent;
begin
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TX11DGA2Display.NextEvent(var event: TPTCEvent; wait: Boolean; const EventMask: TPTCEventMask): Boolean;
var
  tmpEvent: TXEvent;
begin
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
=======
=======
>>>>>>> origin/fixes_2_2
  End;

  If FFramebufferIsOpen Then
  Begin
    FFramebufferIsOpen := False;
    XDGACloseFramebuffer(FDisplay, FScreen);
  End;

  If FDisplay <> Nil Then
    XFlush(FDisplay);

  If FCMap <> 0 Then
  Begin
    XFreeColormap(FDisplay, FCMap);
    FCMap := 0;
  End;

  FreeMemAndNil(FColours);

  If FXDGADevice <> Nil Then
  Begin
    tmp := FXDGADevice;
    FXDGADevice := Nil;
    XFree(tmp);
  End;

  If FXDGAModes <> Nil Then
  Begin
    tmp := FXDGAModes;
    FXDGAModes := Nil;
    XFree(tmp);
  End;
End;

Procedure TX11DGA2Display.GetModes(Var AModes : TPTCModeDynArray);

Begin
  SetLength(AModes, 1);
  AModes[0] := TPTCMode.Create;
  {todo...}
End;

Procedure TX11DGA2Display.update;

Begin
End;

Procedure TX11DGA2Display.update(Const _area : TPTCArea);

Begin
End;

Procedure TX11DGA2Display.HandleEvents;

Var
  e : TXEvent;
  NewFocus : Boolean;
  NewFocusSpecified : Boolean;

  Function UsefulEventsPending : Boolean;

  Var
    tmpEvent : TXEvent;

  Begin
    If XCheckTypedEvent(FDisplay, ClientMessage, @tmpEvent) Then
    Begin
      Result := True;
      XPutBackEvent(FDisplay, @tmpEvent);
      Exit;
    End;

    If XCheckMaskEvent(FDisplay, FocusChangeMask Or
                       KeyPressMask Or KeyReleaseMask Or
                       ButtonPressMask Or ButtonReleaseMask Or
                       PointerMotionMask Or ExposureMask, @tmpEvent) Then
    Begin
      Result := True;
      XPutBackEvent(FDisplay, @tmpEvent);
      Exit;
    End;

    Result := False;
  End;

  Procedure HandleKeyEvent;

  Var
    sym : TKeySym;
    sym_modded : TKeySym; { modifiers like shift are taken into account here }
    press : Boolean;
    alt, shift, ctrl : Boolean;
    uni : Integer;
    key : TPTCKeyEvent;
    buf : Array[1..16] Of Char;

  Begin
    sym := XLookupKeySym(@e.xkey, 0);
    XLookupString(@e.xkey, @buf, SizeOf(buf), @sym_modded, Nil);
    uni := X11ConvertKeySymToUnicode(sym_modded);
    alt := (e.xkey.state And Mod1Mask) <> 0;
    shift := (e.xkey.state And ShiftMask) <> 0;
    ctrl := (e.xkey.state And ControlMask) <> 0;
    If e._type = KeyPress Then
      press := True
    Else
      press := False;

    key := Nil;
    Case sym Shr 8 Of
      0 : key := TPTCKeyEvent.Create(FNormalKeys[sym And $FF], uni, alt, shift, ctrl, press);
      $FF : key := TPTCKeyEvent.Create(FFunctionKeys[sym And $FF], uni, alt, shift, ctrl, press);
      Else
        key := TPTCKeyEvent.Create;
    End;
    FEventQueue.AddEvent(key);
  End;

Begin
  NewFocusSpecified := False;
  While UsefulEventsPending Do
  Begin
    XNextEvent(FDisplay, @e);
    Case e._type Of
      FocusIn : Begin
        NewFocus := True;
        NewFocusSpecified := True;
      End;
      FocusOut : Begin
        NewFocus := False;
        NewFocusSpecified := True;
      End;
      ClientMessage : Begin
{        If (e.xclient.format = 32) And (TAtom(e.xclient.data.l[0]) = m_atom_close) Then
          Halt(0);}
      End;
      Expose : Begin
        {...}
      End;
      KeyPress, KeyRelease : HandleKeyEvent;
      ButtonPress, ButtonRelease : Begin
        {...}
      End;
      MotionNotify : Begin
        {...}
      End;
    End;
  End;
//  HandleChangeFocus(NewFocus);
End;

Function TX11DGA2Display.NextEvent(Var event : TPTCEvent; wait : Boolean; Const EventMask : TPTCEventMask) : Boolean;

Var
  tmpEvent : TXEvent;

Begin
  FreeAndNil(event);
  Repeat
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    { process all events from the X queue and put them on our FEventQueue }
    HandleEvents;

    { try to find an event that matches the EventMask }
    event := FEventQueue.NextEvent(EventMask);

<<<<<<< HEAD
<<<<<<< HEAD
    if wait and (event = nil) then
    begin
      { if the X event queue is empty, block until an event is received }
      XPeekEvent(FDisplay, @tmpEvent);
    end;
  until (not Wait) or (event <> nil);
  Result := event <> nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11DGA2Display.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): IPTCEvent;
=======
function TX11DGA2Display.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TX11DGA2Display.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TX11DGA2Display.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TX11DGA2Display.PeekEvent(wait: Boolean; const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> origin/cpstrnew
var
  tmpEvent: TXEvent;
begin
  repeat
=======
=======
>>>>>>> origin/fixes_2_2
    If wait And (event = Nil) Then
    Begin
      { if the X event queue is empty, block until an event is received }
      XPeekEvent(FDisplay, @tmpEvent);
    End;
  Until (Not Wait) Or (event <> Nil);
  Result := event <> Nil;
End;

Function TX11DGA2Display.PeekEvent(wait : Boolean; Const EventMask : TPTCEventMask) : TPTCEvent;

Var
  tmpEvent : TXEvent;

Begin
  Repeat
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    { process all events from the X queue and put them on our FEventQueue }
    HandleEvents;

    { try to find an event that matches the EventMask }
    Result := FEventQueue.PeekEvent(EventMask);

<<<<<<< HEAD
<<<<<<< HEAD
    if wait and (Result = nil) then
    begin
      { if the X event queue is empty, block until an event is received }
      XPeekEvent(FDisplay, @tmpEvent);
    end;
  until (not Wait) or (Result <> nil);
end;

function TX11DGA2Display.lock: Pointer;
begin
  lock := PByte(FXDGADevice^.data) +
          FXDGADevice^.mode.bytesPerScanline * FDestY +
          FDestX * (FXDGADevice^.mode.bitsPerPixel div 8);
end;

procedure TX11DGA2Display.unlock;
begin
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11DGA2Display.Palette(_palette: IPTCPalette);
=======
procedure TX11DGA2Display.palette(const _palette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11DGA2Display.palette(const _palette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11DGA2Display.palette(const _palette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11DGA2Display.palette(const _palette: TPTCPalette);
>>>>>>> origin/cpstrnew
var
  pal: PUint32;
  i: Integer;
begin
  pal := _palette.data;
  if not FFormat.indexed then
    exit;
  for i := 0 to 255 do
  begin
    FColours[i].pixel := i;

    FColours[i].red := ((pal[i] shr 16) and $FF) shl 8;
    FColours[i].green := ((pal[i] shr 8) and $FF) shl 8;
    FColours[i].blue := (pal[i] and $FF) shl 8;

    Byte(FColours[i].flags) := DoRed or DoGreen or DoBlue;
  end;
  XStoreColors(FDisplay, FCMap, FColours, 256);
  XDGAInstallColormap(FDisplay, FScreen, FCMap);
end;

function TX11DGA2Display.GetPitch: Integer;
begin
  Result := FXDGADevice^.mode.bytesPerScanline;
end;

function TX11DGA2Display.getX11Window: TWindow;
begin
  Result := DefaultRootWindow(FDisplay);
end;

function TX11DGA2Display.isFullScreen: Boolean;
begin
  { DGA is always fullscreen }
  Result := True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11DGA2Display.IsOpen: Boolean;
begin
  Result := FOpen;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TX11DGA2Display.SetCursor(visible: Boolean);
begin
  {nothing... raise exception if visible=true?}
end;

procedure TX11DGA2Display.SetMouseGrab(AGrabMouse: Boolean);
begin
  {...}
end;
=======
=======
>>>>>>> origin/fixes_2_2
    If wait And (Result = Nil) Then
    Begin
      { if the X event queue is empty, block until an event is received }
      XPeekEvent(FDisplay, @tmpEvent);
    End;
  Until (Not Wait) Or (Result <> Nil);
End;


Function TX11DGA2Display.lock : Pointer;

Begin
  lock := PByte(FXDGADevice^.data) +
          FXDGADevice^.mode.bytesPerScanline * m_desty +
          m_destx * (FXDGADevice^.mode.bitsPerPixel Div 8);
End;

Procedure TX11DGA2Display.unlock;

Begin
End;

Procedure TX11DGA2Display.palette(Const _palette : TPTCPalette);

Var
  pal : PUint32;
  i : Integer;

Begin
  pal := _palette.data;
  If Not FFormat.indexed Then
    Exit;
  For i := 0 To 255 Do
  Begin
    FColours[i].pixel := i;

    FColours[i].red := ((pal[i] Shr 16) And $FF) Shl 8;
    FColours[i].green := ((pal[i] Shr 8) And $FF) Shl 8;
    FColours[i].blue := (pal[i] And $FF) Shl 8;

    Byte(FColours[i].flags) := DoRed Or DoGreen Or DoBlue;
  End;
  XStoreColors(FDisplay, FCMap, FColours, 256);
  XF86DGAInstallColormap(FDisplay, FScreen, FCMap);
End;

Function TX11DGA2Display.GetPitch : Integer;

Begin
  Result := FXDGADevice^.mode.bytesPerScanline;
End;

Function TX11DGA2Display.getX11Window : TWindow;

Begin
  Result := DefaultRootWindow(FDisplay);
End;

Function TX11DGA2Display.isFullScreen : Boolean;

Begin
  { DGA is always fullscreen }
  Result := True;
End;

Procedure TX11DGA2Display.SetCursor(visible : Boolean);

Begin
  {nothing... raise exception if visible=true?}
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ENDIF ENABLE_X11_EXTENSION_XF86DGA2}
