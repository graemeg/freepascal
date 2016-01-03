<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the PTCPas framebuffer library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2013 Nikolay Nikolov (nickysn@users.sourceforge.net)
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

constructor TX11WindowDisplay.Create(ADisplay: PDisplay; AScreen: Integer; const AFlags: TX11Flags);
begin
  inherited;
  FFocus := True;
  FX11InvisibleCursor := None;
  FCursorVisible := True;
  FGrabMouse := False;
  FMouseGrabbed := False;
end;

destructor TX11WindowDisplay.Destroy;
begin
  Close;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11WindowDisplay.Open(ATitle: AnsiString; AWidth, AHeight: Integer; AFormat: IPTCFormat; const AOpenGLAttributes: IPTCOpenGLAttributes);
var
=======
procedure TX11WindowDisplay.Open(ATitle: AnsiString; AWidth, AHeight: Integer; const AFormat: TPTCFormat);
var
  tmpFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
procedure TX11WindowDisplay.Open(ATitle: AnsiString; AWidth, AHeight: Integer; const AFormat: TPTCFormat);
var
  tmpFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
procedure TX11WindowDisplay.Open(ATitle: AnsiString; AWidth, AHeight: Integer; const AFormat: TPTCFormat);
var
  tmpFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
procedure TX11WindowDisplay.Open(ATitle: AnsiString; AWidth, AHeight: Integer; const AFormat: TPTCFormat);
var
  tmpFormat: TPTCFormat;
>>>>>>> origin/cpstrnew
  xgcv: TXGCValues;
  textprop: TXTextProperty;
  e: TXEvent;
  found: Boolean;
  attr: TXSetWindowAttributes;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AttrMask: culong;
  size_hints: PXSizeHints;
=======
  size_hints: PXSizeHints;
  tmpArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
  size_hints: PXSizeHints;
  tmpArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
  size_hints: PXSizeHints;
  tmpArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
  size_hints: PXSizeHints;
  tmpArea: TPTCArea;
>>>>>>> origin/cpstrnew
  tmppchar: PChar;
  tmpArrayOfCLong: array [1..1] of clong;
  tmpPixmap: TPixmap;
  BlackColor: TXColor;
  BlankCursorData: array [1..8] of Byte = (0, 0, 0, 0, 0, 0, 0, 0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  CreateWindow_Depth: cint;
  CreateWindow_Visual: PVisual;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  FHeight := AHeight;
  FWidth := AWidth;
  FPreviousWidth := FWidth;
  FPreviousHeight := FHeight;
  FDestX := 0;
  FDestY := 0;

  FFullScreen := PTC_X11_FULLSCREEN in FFlags;
  FResizable := (PTC_X11_RESIZABLE_WINDOW in FFlags) and not FFullScreen;
=======
=======
>>>>>>> origin/fixes_2_2
Constructor TX11WindowDisplay.Create(ADisplay : PDisplay; AScreen : Integer; Const AFlags : TX11Flags);

Begin
  Inherited;
  FFocus := True;
  FX11InvisibleCursor := None;
  FCursorVisible := True;
End;

Destructor TX11WindowDisplay.Destroy;

Begin
  Close;
  Inherited Destroy;
End;

Procedure TX11WindowDisplay.Open(ATitle : AnsiString; AWidth, AHeight : Integer; Const AFormat : TPTCFormat);

Var
  tmpFormat : TPTCFormat;
  xgcv : TXGCValues;
  textprop : TXTextProperty;
  e : TXEvent;
  found : Boolean;
  attr : TXSetWindowAttributes;
  size_hints : PXSizeHints;
  tmpArea : TPTCArea;
  tmppchar : PChar;
  tmpArrayOfCLong : Array[1..1] Of clong;
  tmpPixmap : TPixmap;
  BlackColor : TXColor;
  BlankCursorData : Array[1..8] Of Byte = (0, 0, 0, 0, 0, 0, 0, 0);

Begin
  FHeight := AHeight;
  FWidth := AWidth;
  FDestX := 0;
  FDestY := 0;

  FFullScreen := PTC_X11_FULLSCREEN In FFlags;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  FFocus := True;

  FPreviousMousePositionSaved := False;

  FillChar(BlackColor, SizeOf(BlackColor), 0);
  BlackColor.red := 0;
  BlackColor.green := 0;
  BlackColor.blue := 0;

  { Create the mode switcher object }
<<<<<<< HEAD
<<<<<<< HEAD
  if (FModeSwitcher = Nil) and FFullScreen then
=======
  If (FModeSwitcher = Nil) And FFullScreen Then
>>>>>>> graemeg/fixes_2_2
=======
  If (FModeSwitcher = Nil) And FFullScreen Then
>>>>>>> origin/fixes_2_2
    FModeSwitcher := CreateModeSwitcher;

  { Create the invisible cursor }
  tmpPixmap := XCreateBitmapFromData(FDisplay, RootWindow(FDisplay, FScreen), @BlankCursorData, 8, 8);
<<<<<<< HEAD
<<<<<<< HEAD
  try
    FX11InvisibleCursor := XCreatePixmapCursor(FDisplay, tmpPixmap, tmpPixmap, @BlackColor, @BlackColor, 0, 0);
  finally
    if tmpPixmap <> None then
      XFreePixmap(FDisplay, tmpPixmap);
  end;

  { Check if we have that colour depth available.. Easy as there is no
    format conversion yet }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FFormat := GetX11Format(AFormat);

  attr.border_pixel := BlackPixel(FDisplay, FScreen);
  attr.background_pixel := BlackPixel(FDisplay, FScreen);
  AttrMask := CWBackPixel or CWBorderPixel;

  CreateWindow_Depth := CopyFromParent;
  CreateWindow_Visual := PVisual(CopyFromParent);
  FVisual := DefaultVisual(FDisplay, FScreen);

  if PTC_X11_USE_OPENGL in FFlags then
  begin
{$IFDEF ENABLE_X11_EXTENSION_GLX}
    FGLXFBConfig := TX11GLXFBConfig.Create(FDisplay, FScreen, AOpenGLAttributes);
    CreateWindow_Depth := FGLXFBConfig.FVisInfo.depth;
    CreateWindow_Visual := FGLXFBConfig.FVisInfo.visual;
    FVisual := CreateWindow_Visual;
    FScreen := FGLXFBConfig.FVisInfo.screen; //?
    attr.colormap := XCreateColormap(FDisplay, RootWindow(FDisplay, FScreen), CreateWindow_Visual, AllocNone);
    AttrMask := AttrMask or CWColormap;
{$ELSE ENABLE_X11_EXTENSION_GLX}
    raise TPTCError.Create('Console does not support OpenGL');
{$ENDIF ENABLE_X11_EXTENSION_GLX}
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  tmpFormat := nil;
  try
    tmpFormat := GetX11Format(AFormat);
    FFormat.Assign(tmpFormat);
  finally
    tmpFormat.Free;
  end;
  tmpFormat := nil;
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

  { Create a window }
  FWindow := XCreateWindow(FDisplay, RootWindow(FDisplay, FScreen), 0, 0,
                AWidth, AHeight, 0, CreateWindow_Depth, InputOutput, CreateWindow_Visual,
                AttrMask, @attr);
  { Register the delete atom }
  FAtomClose := XInternAtom(FDisplay, 'WM_DELETE_WINDOW', False);
  X11Check(XSetWMProtocols(FDisplay, FWindow, @FAtomClose, 1), 'XSetWMProtocols');
<<<<<<< HEAD

  if not (PTC_X11_USE_OPENGL in FFlags) then
  begin
    { Get graphics context }
    xgcv.graphics_exposures := Ord(False);
    FGC := XCreateGC(FDisplay, FWindow, GCGraphicsExposures, @xgcv);
    if FGC = nil then
      raise TPTCError.Create('can''t create graphics context');
  end;

=======
  { Get graphics context }
  xgcv.graphics_exposures := Ord(False);
  FGC := XCreateGC(FDisplay, FWindow, GCGraphicsExposures, @xgcv);
  if FGC = nil then
    raise TPTCError.Create('can''t create graphics context');
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
  { Set window title }
  tmppchar := PChar(ATitle);
  X11Check(XStringListToTextProperty(@tmppchar, 1, @textprop), 'XStringListToTextProperty');
  try
    XSetWMName(FDisplay, FWindow, @textprop);
    XFlush(FDisplay);
  finally
    XFree(textprop.value);
  end;

  { Set normal hints }
  size_hints := XAllocSizeHints;
  try
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    size_hints^.flags := PBaseSize;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    size_hints^.flags := PMinSize or PBaseSize;
    size_hints^.min_width := AWidth;
    size_hints^.min_height := AHeight;
>>>>>>> graemeg/cpstrnew
    size_hints^.base_width := AWidth;
    size_hints^.base_height := AHeight;
    if FFullScreen then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      size_hints^.flags := size_hints^.flags or PMinSize or PWinGravity;
      size_hints^.min_width := AWidth;
      size_hints^.min_height := AHeight;
=======
      size_hints^.flags := size_hints^.flags or PWinGravity;
>>>>>>> graemeg/cpstrnew
=======
      size_hints^.flags := size_hints^.flags or PWinGravity;
>>>>>>> graemeg/cpstrnew
=======
      size_hints^.flags := size_hints^.flags or PWinGravity;
>>>>>>> graemeg/cpstrnew
=======
      size_hints^.flags := size_hints^.flags or PWinGravity;
>>>>>>> origin/cpstrnew
      size_hints^.win_gravity := StaticGravity;
    end
    else
    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      if FResizable then
      begin
        size_hints^.flags := size_hints^.flags or PMinSize;
        size_hints^.min_width := 0;
        size_hints^.min_height := 0;
      end
      else
      begin
        { not fullscreen and not resizable: maxsize=minsize=basesize }
        size_hints^.flags := size_hints^.flags or PMinSize or PMaxSize;
        size_hints^.min_width := AWidth;
        size_hints^.min_height := AHeight;
        size_hints^.max_width := AWidth;
        size_hints^.max_height := AHeight;
      end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      { not fullscreen - add maxsize limit=minsize, i.e. make window not resizable }
      size_hints^.flags := size_hints^.flags or PMaxSize;
      size_hints^.max_width := AWidth;
      size_hints^.max_height := AHeight;
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
    XSetWMNormalHints(FDisplay, FWindow, size_hints);
    XFlush(FDisplay);
  finally
    XFree(size_hints);
  end;

  { Set the _NET_WM_STATE property }
  if FFullScreen then
  begin
=======
=======
>>>>>>> origin/fixes_2_2
  Try
    FX11InvisibleCursor := XCreatePixmapCursor(FDisplay, tmpPixmap, tmpPixmap, @BlackColor, @BlackColor, 0, 0);
  Finally
    If tmpPixmap <> None Then
      XFreePixmap(FDisplay, tmpPixmap);
  End;

  { Check if we have that colour depth available.. Easy as there is no
    format conversion yet }
  tmpFormat := Nil;
  Try
    tmpFormat := GetX11Format(AFormat);
    FFormat.Assign(tmpFormat);
  Finally
    tmpFormat.Free;
  End;
  tmpFormat := Nil;

  { Create a window }
  FWindow := XCreateSimpleWindow(FDisplay, RootWindow(FDisplay, FScreen), 0, 0,
                AWidth, AHeight, 0, BlackPixel(FDisplay, FScreen),
                                    BlackPixel(FDisplay, FScreen));
  { Register the delete atom }
  FAtomClose := XInternAtom(FDisplay, 'WM_DELETE_WINDOW', False);
  X11Check(XSetWMProtocols(FDisplay, FWindow, @FAtomClose, 1), 'XSetWMProtocols');
  { Get graphics context }
  xgcv.graphics_exposures := False;
  FGC := XCreateGC(FDisplay, FWindow, GCGraphicsExposures, @xgcv);
  If FGC = Nil Then
    Raise TPTCError.Create('can''t create graphics context');
  { Set window title }
  tmppchar := PChar(ATitle);
  X11Check(XStringListToTextProperty(@tmppchar, 1, @textprop), 'XStringListToTextProperty');
  Try
    XSetWMName(FDisplay, FWindow, @textprop);
    XFlush(FDisplay);
  Finally
    XFree(textprop.value);
  End;

  { Set normal hints }
  size_hints := XAllocSizeHints;
  Try
    size_hints^.flags := PMinSize Or PBaseSize;
    size_hints^.min_width := AWidth;
    size_hints^.min_height := AHeight;
    size_hints^.base_width := AWidth;
    size_hints^.base_height := AHeight;
    If FFullScreen Then
    Begin
      size_hints^.flags := size_hints^.flags Or PWinGravity;
      size_hints^.win_gravity := StaticGravity;
    End
    Else
    Begin
      { not fullscreen - add maxsize limit=minsize, i.e. make window not resizable }
      size_hints^.flags := size_hints^.flags Or PMaxSize;
      size_hints^.max_width := AWidth;
      size_hints^.max_height := AHeight;
    End;
    XSetWMNormalHints(FDisplay, FWindow, size_hints);
    XFlush(FDisplay);
  Finally
    XFree(size_hints);
  End;

  { Set the _NET_WM_STATE property }
  If FFullScreen Then
  Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    tmpArrayOfCLong[1] := XInternAtom(FDisplay, '_NET_WM_STATE_FULLSCREEN', False);

    XChangeProperty(FDisplay, FWindow,
                    XInternAtom(FDisplay, '_NET_WM_STATE', False),
                    XA_ATOM,
                    32, PropModeReplace, @tmpArrayOfCLong, 1);
<<<<<<< HEAD
<<<<<<< HEAD
  end;
=======
  End;
>>>>>>> graemeg/fixes_2_2
=======
  End;
>>>>>>> origin/fixes_2_2

  { Map the window and wait for success }
  XSelectInput(FDisplay, FWindow, StructureNotifyMask);
  XMapRaised(FDisplay, FWindow);
<<<<<<< HEAD
<<<<<<< HEAD
  repeat
    XNextEvent(FDisplay, @e);
    if e._type = MapNotify then
      Break;
  until False;
  { Get keyboard input and sync }
  XSelectInput(FDisplay, FWindow, KeyPressMask or KeyReleaseMask or
                                  StructureNotifyMask or FocusChangeMask or
                                  ButtonPressMask or ButtonReleaseMask or
                                  PointerMotionMask or ExposureMask);
  XSync(FDisplay, False);

  if not (PTC_X11_USE_OPENGL in FFlags) then
  begin
    { Create XImage using factory method }
    FPrimary := CreateImage(FDisplay, FScreen, FWidth, FHeight, FFormat);
  end;

  found := False;
  repeat
    { Stupid loop. The key }
    { events were causing }
    { problems.. }
    found := XCheckMaskEvent(FDisplay, KeyPressMask or KeyReleaseMask, @e);
  until not found;
=======
=======
>>>>>>> origin/fixes_2_2
  Repeat
    XNextEvent(FDisplay, @e);
    If e._type = MapNotify Then
      Break;
  Until False;
  { Get keyboard input and sync }
  XSelectInput(FDisplay, FWindow, KeyPressMask Or KeyReleaseMask Or
                                  StructureNotifyMask Or FocusChangeMask Or
                                  ButtonPressMask Or ButtonReleaseMask Or
                                  PointerMotionMask);
  XSync(FDisplay, False);
  { Create XImage using factory method }
  FPrimary := CreateImage(FDisplay, FScreen, FWidth, FHeight, FFormat);

  found := False;
  Repeat
    { Stupid loop. The key }
    { events were causing }
    { problems.. }
    found := XCheckMaskEvent(FDisplay, KeyPressMask Or KeyReleaseMask, @e);
  Until Not found;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  attr.backing_store := Always;
  XChangeWindowAttributes(FDisplay, FWindow, CWBackingStore, @attr);

  { Set clipping area }
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> origin/fixes_2_2
  tmpArea := TPTCArea.Create(0, 0, FWidth, FHeight);
  Try
    FClip.Assign(tmpArea);
  Finally
    tmpArea.Free;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { Installs the right colour map for 8 bit modes }
  CreateColormap;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF ENABLE_X11_EXTENSION_GLX}
  if PTC_X11_USE_OPENGL in FFlags then
  begin
    FGLXFBConfig.AttachToWindow(FWindow);
    FGLXFBConfig.MakeRenderingContextCurrent;
  end;
{$ENDIF ENABLE_X11_EXTENSION_GLX}

  if FFullScreen then
    EnterFullScreen;

  FOpen := True;
end;

procedure TX11WindowDisplay.Open(AWindow: TWindow; AFormat: IPTCFormat);
begin
end;

procedure TX11WindowDisplay.Open(AWindow: TWindow; AFormat: IPTCFormat; AX, AY, AWidth, AHeight: Integer);
=======
  if FFullScreen then
    EnterFullScreen;
end;

=======
  if FFullScreen then
    EnterFullScreen;
end;

>>>>>>> graemeg/cpstrnew
procedure TX11WindowDisplay.Open(AWindow: TWindow; const AFormat: TPTCFormat);
begin
end;

procedure TX11WindowDisplay.Open(AWindow: TWindow; const AFormat: TPTCFormat; AX, AY, AWidth, AHeight: Integer);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
begin
end;

procedure TX11WindowDisplay.Close;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  FOpen := False;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
  if FFullScreen then
    EnterFullScreen;
end;

procedure TX11WindowDisplay.Open(AWindow: TWindow; const AFormat: TPTCFormat);
begin
end;

procedure TX11WindowDisplay.Open(AWindow: TWindow; const AFormat: TPTCFormat; AX, AY, AWidth, AHeight: Integer);
begin
end;

procedure TX11WindowDisplay.Close;
begin
>>>>>>> graemeg/cpstrnew
=======
  if FFullScreen then
    EnterFullScreen;
end;

procedure TX11WindowDisplay.Open(AWindow: TWindow; const AFormat: TPTCFormat);
begin
end;

procedure TX11WindowDisplay.Open(AWindow: TWindow; const AFormat: TPTCFormat; AX, AY, AWidth, AHeight: Integer);
begin
end;

procedure TX11WindowDisplay.Close;
begin
>>>>>>> origin/cpstrnew
  FreeAndNil(FModeSwitcher);

{$IFDEF ENABLE_X11_EXTENSION_GLX}
  FreeAndNil(FGLXFBConfig);
{$ENDIF ENABLE_X11_EXTENSION_GLX}

  {pthreads?!}
  if FCMap <> 0 then
  begin
    XFreeColormap(FDisplay, FCMap);
    FCMap := 0;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
  If FFullScreen Then
    EnterFullScreen;
End;

Procedure TX11WindowDisplay.Open(AWindow : TWindow; Const AFormat : TPTCFormat);

Begin
End;

Procedure TX11WindowDisplay.Open(AWindow : TWindow; Const AFormat : TPTCFormat; AX, AY, AWidth, AHeight : Integer);

Begin
End;

Procedure TX11WindowDisplay.Close;

Begin
  FreeAndNil(FModeSwitcher);

  {pthreads?!}
  If FCMap <> 0 Then
  Begin
    XFreeColormap(FDisplay, FCMap);
    FCMap := 0;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { Destroy XImage and buffer }
  FreeAndNil(FPrimary);
  FreeMemAndNil(FColours);

<<<<<<< HEAD
<<<<<<< HEAD
  if FGC <> nil then
  begin
    XFreeGC(FDisplay, FGC);
    FGC := nil;
  end;

  { Hide and destroy window }
  if (FWindow <> 0) and (not (PTC_X11_LEAVE_WINDOW In FFlags)) then
  begin
=======
  { Hide and destroy window }
  If (FWindow <> 0) And (Not (PTC_X11_LEAVE_WINDOW In FFlags)) Then
  Begin
>>>>>>> graemeg/fixes_2_2
=======
  { Hide and destroy window }
  If (FWindow <> 0) And (Not (PTC_X11_LEAVE_WINDOW In FFlags)) Then
  Begin
>>>>>>> origin/fixes_2_2
    XUnmapWindow(FDisplay, FWindow);
    XSync(FDisplay, False);

    XDestroyWindow(FDisplay, FWindow);
<<<<<<< HEAD
<<<<<<< HEAD
  end;

  { Free the invisible cursor }
  if FX11InvisibleCursor <> None then
  begin
    XFreeCursor(FDisplay, FX11InvisibleCursor);
    FX11InvisibleCursor := None;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11WindowDisplay.InternalResize(AWidth, AHeight: Integer);
begin
  if FFullScreen then
    raise TPTCError.Create('Internal resize not supported in fullscreen mode');
  if not FResizable then
    raise TPTCError.Create('Internal resize cannot be used on a non-resizable window');

  if not (PTC_X11_USE_OPENGL in FFlags) then
  begin
    { destroy previous XImage }
    FreeAndNil(FPrimary);
  end;

  FWidth := AWidth;
  FHeight := AHeight;

  if not (PTC_X11_USE_OPENGL in FFlags) then
  begin
    { Create XImage using factory method }
    FPrimary := CreateImage(FDisplay, FScreen, FWidth, FHeight, FFormat);
  end;

  { Set clipping area }
  FClip := TPTCArea.Create(0, 0, FWidth, FHeight);
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TX11WindowDisplay.internal_ShowCursor(AVisible: Boolean);
var
  attr: TXSetWindowAttributes;
begin
  if AVisible then
    attr.cursor := None { Use the normal cursor }
  else
    attr.cursor := FX11InvisibleCursor; { Set the invisible cursor }

  XChangeWindowAttributes(FDisplay, FWindow, CWCursor, @attr);
end;

procedure TX11WindowDisplay.internal_GrabMouse(AGrabMouse: Boolean);
var
  GrabResult: cint;
begin
  if AGrabMouse then
  begin
    if not FMouseGrabbed then
    begin
      //Writeln('XGrabPointer...');
      GrabResult :=XGrabPointer(FDisplay, FWindow, True,
                                PointerMotionMask or ButtonPressMask or ButtonReleaseMask,
                                GrabModeAsync, GrabModeAsync,
                                FWindow, None, CurrentTime);
      if GrabResult <> GrabSuccess then
      begin
        LOG('XGrabPointer failed, result=' + IntToStr(GrabResult));
        //Writeln(GrabResult);
      end;

      { XGrabPointer may fail if someone else is already holding a mouse grab.
        It happens e.g. sometimes when alt+tabbing away and back to the application
        in GNOME. That's why we check the result and try again on the next call.
        And that's why this method is called everytime HandleEvents is called. }
      FMouseGrabbed := GrabResult = GrabSuccess;
    end;
  end
  else
  begin
    if FMouseGrabbed then
    begin
      //Writeln('XUngrabPointer');
      XUngrabPointer(FDisplay, CurrentTime);
      FMouseGrabbed := False;
    end;
  end;
end;

procedure TX11WindowDisplay.SetCursor(AVisible: Boolean);
begin
  FCursorVisible := AVisible;

  if FFocus then
    internal_ShowCursor(FCursorVisible);
end;
<<<<<<< HEAD

procedure TX11WindowDisplay.SetMouseGrab(AGrabMouse: Boolean);
begin
  FGrabMouse := AGrabMouse;

  if FFocus then
    internal_GrabMouse(FGrabMouse);
end;

=======

procedure TX11WindowDisplay.SetMouseGrab(AGrabMouse: Boolean);
begin
  FGrabMouse := AGrabMouse;

  if FFocus then
    internal_GrabMouse(FGrabMouse);
end;

>>>>>>> origin/cpstrnew
procedure TX11WindowDisplay.EnterFullScreen;
begin
  { try to switch mode }
  if Assigned(FModeSwitcher) then
=======
=======
>>>>>>> origin/fixes_2_2
  End;

  { Free the invisible cursor }
  If FX11InvisibleCursor <> None Then
  Begin
    XFreeCursor(FDisplay, FX11InvisibleCursor);
    FX11InvisibleCursor := None;
  End;
End;

Procedure TX11WindowDisplay.internal_ShowCursor(AVisible : Boolean);

Var
  attr : TXSetWindowAttributes;

Begin
  If AVisible Then
    attr.cursor := None { Use the normal cursor }
  Else
    attr.cursor := FX11InvisibleCursor; { Set the invisible cursor }

  XChangeWindowAttributes(FDisplay, FWindow, CWCursor, @attr);
End;

Procedure TX11WindowDisplay.SetCursor(AVisible : Boolean);

Begin
  FCursorVisible := AVisible;

  If FFocus Then
    internal_ShowCursor(FCursorVisible);
End;

Procedure TX11WindowDisplay.EnterFullScreen;

Begin
  { Try to switch mode }
  If Assigned(FModeSwitcher) Then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FModeSwitcher.SetBestMode(FWidth, FHeight);

  XSync(FDisplay, False);

  { Center the image }
<<<<<<< HEAD
<<<<<<< HEAD
  FDestX := FModeSwitcher.Width div 2 - FWidth div 2;
  FDestY := FModeSwitcher.Height div 2 - FHeight div 2;
end;

procedure TX11WindowDisplay.LeaveFullScreen;
begin
  { Restore previous mode }
  if Assigned(FModeSwitcher) then
    FModeSwitcher.RestorePreviousMode;

  XSync(FDisplay, False);
end;

procedure TX11WindowDisplay.HandleChangeFocus(ANewFocus: Boolean);
begin
  { No change? }
  if ANewFocus = FFocus then
    exit;

  FFocus := ANewFocus;
  if FFocus then
  begin
    { focus in }
    if FFullScreen then
      EnterFullScreen;

    internal_ShowCursor(FCursorVisible);
//    internal_GrabMouse(FGrabMouse);
  end
  else
  begin
    { focus out }

    internal_GrabMouse(False);

    if FFullScreen then
      LeaveFullScreen;

    internal_ShowCursor(True);
  end;

  XSync(FDisplay, False);
end;

function TX11WindowDisplay_MatchAnyEvent(display: PDisplay; event: PXEvent; arg: TXPointer): LongBool; cdecl;
begin
  Result := LongBool(1);
end;

procedure TX11WindowDisplay.HandleEvents;
var
  e: TXEvent;
  NewFocus: Boolean;
  NewFocusSpecified: Boolean;

  function UsefulEventsPending: Boolean;
  var
    tmpEvent: TXEvent;
  begin
    if XCheckIfEvent(FDisplay, @tmpEvent, @TX11WindowDisplay_MatchAnyEvent, nil) then
    begin
      Result := True;
      XPutBackEvent(FDisplay, @tmpEvent);
      exit;
    end;

    Result := False;
  end;

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
  procedure HandleKeyEvent;
  var
    sym: TKeySym;
    sym_modded: TKeySym; { modifiers like shift are taken into account here }
    press: Boolean;
    alt, shift, ctrl: Boolean;
    uni: Integer;
    key: TPTCKeyEvent;
    buf: array [1..16] of Char;
  begin
    sym := XLookupKeySym(@e.xkey, 0);
    XLookupString(@e.xkey, @buf, SizeOf(buf), @sym_modded, nil);
    uni := X11ConvertKeySymToUnicode(sym_modded);
    alt := (e.xkey.state and Mod1Mask) <> 0;
    shift := (e.xkey.state and ShiftMask) <> 0;
    ctrl := (e.xkey.state and ControlMask) <> 0;
    if e._type = KeyPress then
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
  procedure HandleMouseEvent;
  var
    x, y: cint;
    state: cuint;
    PTCMouseButtonState: TPTCMouseButtonState;

    button: TPTCMouseButton;
    before, after: Boolean;
    cstate: TPTCMouseButtonState;
  begin
    case e._type of
      MotionNotify: begin
        x := e.xmotion.x;
        y := e.xmotion.y;
        state := e.xmotion.state;
      end;
      ButtonPress, ButtonRelease: begin
        x := e.xbutton.x;
        y := e.xbutton.y;
        state := e.xbutton.state;
        if e._type = ButtonPress then
        begin
          case e.xbutton.button of
            Button1: state := state or Button1Mask;
            Button2: state := state or Button2Mask;
            Button3: state := state or Button3Mask;
            Button4: state := state or Button4Mask;
            Button5: state := state or Button5Mask;
          end;
        end
        else
        begin
          case e.xbutton.button of
            Button1: state := state and (not Button1Mask);
            Button2: state := state and (not Button2Mask);
            Button3: state := state and (not Button3Mask);
            Button4: state := state and (not Button4Mask);
            Button5: state := state and (not Button5Mask);
          end;
        end;
      end;
      else
        raise TPTCError.Create('Internal Error');
    end;

    if (state and Button1Mask) = 0 then
      PTCMouseButtonState := []
    else
      PTCMouseButtonState := [PTCMouseButton1];
    if (state and Button2Mask) <> 0 then
      PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton2];
    if (state and Button3Mask) <> 0 then
      PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton3];
    if (state and Button4Mask) <> 0 then
      PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton4];
    if (state and Button5Mask) <> 0 then
      PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton5];

    if (x >= 0) and (x < FWidth) and (y >= 0) and (y < FHeight) then
    begin
      if not FPreviousMousePositionSaved then
      begin
        FPreviousMouseX := x; { first DeltaX will be 0 }
        FPreviousMouseY := y; { first DeltaY will be 0 }
        FPreviousMouseButtonState := [];
      end;

      { movement? }
      if (x <> FPreviousMouseX) or (y <> FPreviousMouseY) then
=======
=======
>>>>>>> origin/fixes_2_2
  FDestX := FModeSwitcher.Width Div 2 - FWidth Div 2;
  FDestY := FModeSwitcher.Height Div 2 - FHeight Div 2;
End;

Procedure TX11WindowDisplay.LeaveFullScreen;

Begin
  { Restore previous mode }
  If Assigned(FModeSwitcher) Then
    FModeSwitcher.RestorePreviousMode;

  XSync(FDisplay, False);
End;

Procedure TX11WindowDisplay.HandleChangeFocus(ANewFocus : Boolean);

Begin
  { No change? }
  If ANewFocus = FFocus Then
    Exit;

  FFocus := ANewFocus;
  If FFocus Then
  Begin
    { focus in }
    If FFullScreen Then
      EnterFullScreen;

    internal_ShowCursor(FCursorVisible);
  End
  Else
  Begin
    { focus out }
    If FFullScreen Then
      LeaveFullScreen;

    internal_ShowCursor(True);
  End;

  XSync(FDisplay, False);
End;

Procedure TX11WindowDisplay.HandleEvents;

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

  Procedure HandleMouseEvent;

  Var
    x, y : cint;
    state : cuint;
    PTCMouseButtonState : TPTCMouseButtonState;

    button : TPTCMouseButton;
    before, after : Boolean;
    cstate : TPTCMouseButtonState;

  Begin
    Case e._type Of
      MotionNotify : Begin
        x := e.xmotion.x;
        y := e.xmotion.y;
        state := e.xmotion.state;
      End;
      ButtonPress, ButtonRelease : Begin
        x := e.xbutton.x;
        y := e.xbutton.y;
        state := e.xbutton.state;
        If e._type = ButtonPress Then
        Begin
          Case e.xbutton.button Of
            Button1 : state := state Or Button1Mask;
            Button2 : state := state Or Button2Mask;
            Button3 : state := state Or Button3Mask;
            Button4 : state := state Or Button4Mask;
            Button5 : state := state Or Button5Mask;
          End;
        End
        Else
        Begin
          Case e.xbutton.button Of
            Button1 : state := state And (Not Button1Mask);
            Button2 : state := state And (Not Button2Mask);
            Button3 : state := state And (Not Button3Mask);
            Button4 : state := state And (Not Button4Mask);
            Button5 : state := state And (Not Button5Mask);
          End;
        End;
      End;
      Else
        Raise TPTCError.Create('Internal Error');
    End;

    If (state And Button1Mask) = 0 Then
      PTCMouseButtonState := []
    Else
      PTCMouseButtonState := [PTCMouseButton1];
    If (state And Button2Mask) <> 0 Then
      PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton2];
    If (state And Button3Mask) <> 0 Then
      PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton3];
    If (state And Button4Mask) <> 0 Then
      PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton4];
    If (state And Button5Mask) <> 0 Then
      PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton5];

    If (x >= 0) And (x < FWidth) And (y >= 0) And (y < FHeight) Then
    Begin
      If Not FPreviousMousePositionSaved Then
      Begin
        FPreviousMouseX := x; { first DeltaX will be 0 }
        FPreviousMouseY := y; { first DeltaY will be 0 }
        FPreviousMouseButtonState := [];
      End;

      { movement? }
      If (x <> FPreviousMouseX) Or (y <> FPreviousMouseY) Then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        FEventQueue.AddEvent(TPTCMouseEvent.Create(x, y, x - FPreviousMouseX, y - FPreviousMouseY, FPreviousMouseButtonState));

      { button presses/releases? }
      cstate := FPreviousMouseButtonState;
<<<<<<< HEAD
<<<<<<< HEAD
      for button := Low(button) to High(button) do
      begin
        before := button In FPreviousMouseButtonState;
        after := button In PTCMouseButtonState;
        if after and (not before) then
        begin
          { button was pressed }
          cstate := cstate + [button];
          FEventQueue.AddEvent(TPTCMouseButtonEvent.Create(x, y, 0, 0, cstate, True, button));
        end
        else
          if before and (not after) then
          begin
            { button was released }
            cstate := cstate - [button];
            FEventQueue.AddEvent(TPTCMouseButtonEvent.Create(x, y, 0, 0, cstate, False, button));
          end;
      end;
=======
=======
>>>>>>> origin/fixes_2_2
      For button := Low(button) To High(button) Do
      Begin
        before := button In FPreviousMouseButtonState;
        after := button In PTCMouseButtonState;
        If after And (Not before) Then
        Begin
          { button was pressed }
          cstate := cstate + [button];
          FEventQueue.AddEvent(TPTCMouseButtonEvent.Create(x, y, 0, 0, cstate, True, button));
        End
        Else
          If before And (Not after) Then
          Begin
            { button was released }
            cstate := cstate - [button];
            FEventQueue.AddEvent(TPTCMouseButtonEvent.Create(x, y, 0, 0, cstate, False, button));
          End;
      End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

      FPreviousMouseX := x;
      FPreviousMouseY := y;
      FPreviousMouseButtonState := PTCMouseButtonState;
      FPreviousMousePositionSaved := True;
<<<<<<< HEAD
<<<<<<< HEAD
    end;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  procedure HandleConfigureNotifyEvent;
  begin
    if FFullScreen or not FResizable then
      exit;

    if (FPreviousWidth <> e.xconfigure.width) or (FPreviousHeight <> e.xconfigure.height) then
      FEventQueue.AddEvent(TPTCResizeEvent.Create(e.xconfigure.width, e.xconfigure.height));

    FPreviousWidth := e.xconfigure.width;
    FPreviousHeight := e.xconfigure.height;
  end;

=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
begin
  NewFocusSpecified := False;
  while UsefulEventsPending do
  begin
    XNextEvent(FDisplay, @e);
    case e._type of
      FocusIn: begin
        NewFocus := True;
        NewFocusSpecified := True;
      end;
      FocusOut: begin
        NewFocus := False;
        NewFocusSpecified := True;
      end;
      ClientMessage: begin
        if (e.xclient.format = 32) and (TAtom(e.xclient.data.l[0]) = FAtomClose) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          if InterceptClose then
            FEventQueue.AddEvent(TPTCCloseEvent.Create)
          else
            Halt(0);
=======
          Halt(0);
>>>>>>> graemeg/cpstrnew
=======
          Halt(0);
>>>>>>> graemeg/cpstrnew
=======
          Halt(0);
>>>>>>> graemeg/cpstrnew
=======
          Halt(0);
>>>>>>> origin/cpstrnew
      end;
      Expose: begin
        if e.xexpose.count = 0 then
          Draw;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      ConfigureNotify: HandleConfigureNotifyEvent;
      KeyPress, KeyRelease: HandleKeyEvent(e.xkey);
=======
      KeyPress, KeyRelease: HandleKeyEvent;
>>>>>>> graemeg/cpstrnew
=======
      KeyPress, KeyRelease: HandleKeyEvent;
>>>>>>> graemeg/cpstrnew
=======
      KeyPress, KeyRelease: HandleKeyEvent;
>>>>>>> graemeg/cpstrnew
=======
      KeyPress, KeyRelease: HandleKeyEvent;
>>>>>>> origin/cpstrnew
      ButtonPress, ButtonRelease, MotionNotify: HandleMouseEvent;
    end;
  end;
  if NewFocusSpecified then
    HandleChangeFocus(NewFocus);

  if FFocus then
    internal_GrabMouse(FGrabMouse);
end;
<<<<<<< HEAD

procedure TX11WindowDisplay.Draw;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if not (PTC_X11_USE_OPENGL in FFlags) then
  begin
    FPrimary.Put(FWindow, FGC, FDestX, FDestY);
  end;
=======
  FPrimary.Put(FWindow, FGC, FDestX, FDestY);
>>>>>>> graemeg/cpstrnew
end;

procedure TX11WindowDisplay.Update;
begin
  Draw;
<<<<<<< HEAD

  HandleEvents;
end;

procedure TX11WindowDisplay.Update(AArea: IPTCArea);
var
  UpdateArea: IPTCArea;
begin
  UpdateArea := TPTCClipper.Clip(TPTCArea.Create(0, 0, FWidth, FHeight), AArea);
  FPrimary.Put(FWindow, FGC, UpdateArea.Left, UpdateArea.Top,
               FDestX + UpdateArea.Left, FDestY + UpdateArea.Top,
               UpdateArea.Width, UpdateArea.Height);

  HandleEvents;
end;

function TX11WindowDisplay.NextEvent(out AEvent: IPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
var
  tmpEvent: TXEvent;
begin
=======
=======
  FPrimary.Put(FWindow, FGC, FDestX, FDestY);
end;

procedure TX11WindowDisplay.Update;
begin
  Draw;
>>>>>>> graemeg/cpstrnew

  HandleEvents;
end;

=======
  FPrimary.Put(FWindow, FGC, FDestX, FDestY);
end;

procedure TX11WindowDisplay.Update;
begin
  Draw;

  HandleEvents;
end;

>>>>>>> graemeg/cpstrnew
=======

procedure TX11WindowDisplay.Draw;
begin
  FPrimary.Put(FWindow, FGC, FDestX, FDestY);
end;

procedure TX11WindowDisplay.Update;
begin
  Draw;

  HandleEvents;
end;

>>>>>>> origin/cpstrnew
procedure TX11WindowDisplay.Update(const AArea: TPTCArea);
var
  updatearea: TPTCArea;
  tmparea: TPTCArea;
begin
  tmparea := TPTCArea.Create(0, 0, FWidth, FHeight);
  try
    updatearea := TPTCClipper.Clip(tmparea, AArea);
    try
      FPrimary.Put(FWindow, FGC, updatearea.Left, updatearea.Top,
                   FDestX + updatearea.Left, FDestY + updatearea.Top,
                   updatearea.Width, updatearea.Height);
    finally
      updatearea.Free;
    end;
  finally
    tmparea.Free;
  end;

  HandleEvents;
end;

function TX11WindowDisplay.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
var
  tmpEvent: TXEvent;
begin
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
        If (e.xclient.format = 32) And (TAtom(e.xclient.data.l[0]) = FAtomClose) Then
          Halt(0);
      End;
      Expose : Begin
        {...}
      End;
      KeyPress, KeyRelease : HandleKeyEvent;
      ButtonPress, ButtonRelease, MotionNotify : HandleMouseEvent;
    End;
  End;
  If NewFocusSpecified Then
    HandleChangeFocus(NewFocus);
End;

Procedure TX11WindowDisplay.Update;

Begin
  FPrimary.Put(FWindow, FGC, FDestX, FDestY);

  HandleEvents;
End;

Procedure TX11WindowDisplay.Update(Const AArea : TPTCArea);

Var
  updatearea : TPTCArea;
  tmparea : TPTCArea;

Begin
  tmparea := TPTCArea.Create(0, 0, FWidth, FHeight);
  Try
    updatearea := TPTCClipper.Clip(tmparea, AArea);
    Try
      FPrimary.Put(FWindow, FGC, updatearea.Left, updatearea.Top,
                   FDestX + updatearea.Left, FDestY + updatearea.Top,
                   updatearea.Width, updatearea.Height);
    Finally
      updatearea.Free;
    End;
  Finally
    tmparea.Free;
  End;

  HandleEvents;
End;

Function TX11WindowDisplay.NextEvent(Var AEvent : TPTCEvent; AWait : Boolean; Const AEventMask : TPTCEventMask) : Boolean;

Var
  tmpEvent : TXEvent;

Begin
  FreeAndNil(AEvent);
  Repeat
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    { process all events from the X queue and put them on our FEventQueue }
    HandleEvents;

    { try to find an event that matches the EventMask }
    AEvent := FEventQueue.NextEvent(AEventMask);

<<<<<<< HEAD
<<<<<<< HEAD
    if AWait and (AEvent = Nil) then
    begin
      { if the X event queue is empty, block until an event is received }
      XPeekEvent(FDisplay, @tmpEvent);
    end;
  until (not AWait) or (AEvent <> Nil);
  Result := AEvent <> nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11WindowDisplay.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): IPTCEvent;
=======
function TX11WindowDisplay.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TX11WindowDisplay.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TX11WindowDisplay.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TX11WindowDisplay.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> origin/cpstrnew
var
  tmpEvent: TXEvent;
begin
  repeat
=======
=======
>>>>>>> origin/fixes_2_2
    If AWait And (AEvent = Nil) Then
    Begin
      { if the X event queue is empty, block until an event is received }
      XPeekEvent(FDisplay, @tmpEvent);
    End;
  Until (Not AWait) Or (AEvent <> Nil);
  Result := AEvent <> Nil;
End;

Function TX11WindowDisplay.PeekEvent(AWait : Boolean; Const AEventMask : TPTCEventMask) : TPTCEvent;

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
    Result := FEventQueue.PeekEvent(AEventMask);

<<<<<<< HEAD
<<<<<<< HEAD
    if AWait and (Result = Nil) then
    begin
      { if the X event queue is empty, block until an event is received }
      XPeekEvent(FDisplay, @tmpEvent);
    end;
  until (not AWait) or (Result <> nil);
end;

function TX11WindowDisplay.Lock: Pointer;
begin
  Result := FPrimary.Lock;
end;
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11WindowDisplay.Unlock;
begin
end;

procedure TX11WindowDisplay.GetModes(var AModes: TPTCModeList);
var
  current_desktop_format: IPTCFormat;
=======
procedure TX11WindowDisplay.unlock;
begin
end;

procedure TX11WindowDisplay.GetModes(var AModes: TPTCModeDynArray);
var
  current_desktop_format, tmpfmt: TPTCFormat;
>>>>>>> graemeg/cpstrnew
begin
  if FModeSwitcher = nil then
    FModeSwitcher := CreateModeSwitcher;

<<<<<<< HEAD
  current_desktop_format := GetX11Format(TPTCFormat.Create(8));

  FModeSwitcher.GetModes(AModes, current_desktop_format);
end;

procedure TX11WindowDisplay.Palette(APalette: IPTCPalette);
=======
=======
=======

>>>>>>> origin/cpstrnew
procedure TX11WindowDisplay.unlock;
begin
end;

<<<<<<< HEAD
=======

procedure TX11WindowDisplay.unlock;
begin
end;

>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TX11WindowDisplay.GetModes(var AModes: TPTCModeDynArray);
var
  current_desktop_format, tmpfmt: TPTCFormat;
begin
  if FModeSwitcher = nil then
    FModeSwitcher := CreateModeSwitcher;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  current_desktop_format := nil;
  tmpfmt := TPTCFormat.Create(8);
  try
    current_desktop_format := GetX11Format(tmpfmt);

    FModeSwitcher.GetModes(AModes, current_desktop_format);
  finally
    tmpfmt.Free;
    current_desktop_format.Free;
  end;
end;

procedure TX11WindowDisplay.Palette(const APalette: TPTCPalette);
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
var
  pal: PUint32;
  i: Integer;
begin
  pal := APalette.Data;
  if not FFormat.Indexed then
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
end;

function TX11WindowDisplay.GetPitch: Integer;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result := FPrimary.Pitch;
end;

function TX11WindowDisplay.CreateImage(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer;
                                       AFormat: IPTCFormat): TX11Image;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Result := FPrimary.pitch;
end;

function TX11WindowDisplay.CreateImage(ADisplay: PDisplay; AScreen, AWidth, AHeight: Integer;
                                       AFormat: TPTCFormat): TX11Image;
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
  {$IFDEF ENABLE_X11_EXTENSION_XSHM}
  if (PTC_X11_TRY_XSHM In FFlags) and XShmQueryExtension(ADisplay) then
  begin
    try
      LOG('trying to create a XShm image');
      Result := TX11ShmImage.Create(ADisplay, AScreen, AWidth, AHeight, AFormat);
      exit;
    except
      LOG('XShm failed');
    end;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
    If AWait And (Result = Nil) Then
    Begin
      { if the X event queue is empty, block until an event is received }
      XPeekEvent(FDisplay, @tmpEvent);
    End;
  Until (Not AWait) Or (Result <> Nil);
End;

Function TX11WindowDisplay.Lock : Pointer;

Begin
  Result := FPrimary.Lock;
End;

Procedure TX11WindowDisplay.unlock;

Begin
End;

Procedure TX11WindowDisplay.GetModes(Var AModes : TPTCModeDynArray);

Var
  current_desktop_format, tmpfmt : TPTCFormat;

Begin
  If FModeSwitcher = Nil Then
    FModeSwitcher := CreateModeSwitcher;

  current_desktop_format := Nil;
  tmpfmt := TPTCFormat.Create(8);
  Try
    current_desktop_format := GetX11Format(tmpfmt);

    FModeSwitcher.GetModes(AModes, current_desktop_format);
  Finally
    tmpfmt.Free;
    current_desktop_format.Free;
  End;
End;

Procedure TX11WindowDisplay.Palette(Const APalette : TPTCPalette);

Var
  pal : PUint32;
  i : Integer;

Begin
  pal := APalette.Data;
  If Not FFormat.Indexed Then
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
End;

Function TX11WindowDisplay.GetPitch : Integer;

Begin
  Result := FPrimary.pitch;
End;

Function TX11WindowDisplay.CreateImage(ADisplay : PDisplay; AScreen, AWidth, AHeight : Integer;
                                       AFormat : TPTCFormat) : TX11Image;

Begin
  {$IFDEF ENABLE_X11_EXTENSION_XSHM}
  If (PTC_X11_TRY_XSHM In FFlags) And XShmQueryExtension(ADisplay) Then
  Begin
    Try
      LOG('trying to create a XShm image');
      Result := TX11ShmImage.Create(ADisplay, AScreen, AWidth, AHeight, AFormat);
      Exit;
    Except
      LOG('XShm failed');
    End;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$ENDIF ENABLE_X11_EXTENSION_XSHM}

  LOG('trying to create a normal image');
  Result := TX11NormalImage.Create(ADisplay, AScreen, AWidth, AHeight, AFormat);
<<<<<<< HEAD
<<<<<<< HEAD
end;

function TX11WindowDisplay.CreateModeSwitcher: TX11Modes;
begin
{$IFDEF ENABLE_X11_EXTENSION_XRANDR}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if PTC_X11_TRY_XRANDR in FFlags then
=======
  if PTC_X11_TRY_XRANDR In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_XRANDR In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_XRANDR In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_XRANDR In FFlags then
>>>>>>> origin/cpstrnew
    try
      LOG('trying to initialize the Xrandr mode switcher');
      Result := TX11ModesXrandr.Create(FDisplay, FScreen);
      exit;
    except
      LOG('Xrandr failed');
    end;
{$ENDIF ENABLE_X11_EXTENSION_XRANDR}

{$IFDEF ENABLE_X11_EXTENSION_XF86VIDMODE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if PTC_X11_TRY_XF86VIDMODE in FFlags then
=======
  if PTC_X11_TRY_XF86VIDMODE In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_XF86VIDMODE In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_XF86VIDMODE In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_XF86VIDMODE In FFlags then
>>>>>>> origin/cpstrnew
    try
      LOG('trying to initialize the XF86VidMode mode switcher');
      Result := TX11ModesXF86VidMode.Create(FDisplay, FScreen);
      exit;
    except
      LOG('XF86VidMode failed');
    end;
=======
=======
>>>>>>> origin/fixes_2_2
End;

Function TX11WindowDisplay.CreateModeSwitcher : TX11Modes;

Begin
{$IFDEF ENABLE_X11_EXTENSION_XRANDR}
  If PTC_X11_TRY_XRANDR In FFlags Then
    Try
      LOG('trying to initialize the Xrandr mode switcher');
      Result := TX11ModesXrandr.Create(FDisplay, FScreen);
      Exit;
    Except
      LOG('Xrandr failed');
    End;
{$ENDIF ENABLE_X11_EXTENSION_XRANDR}

{$IFDEF ENABLE_X11_EXTENSION_XF86VIDMODE}
  If PTC_X11_TRY_XF86VIDMODE In FFlags Then
    Try
      LOG('trying to initialize the XF86VidMode mode switcher');
      Result := TX11ModesXF86VidMode.Create(FDisplay, FScreen);
      Exit;
    Except
      LOG('XF86VidMode failed');
    End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ENDIF ENABLE_X11_EXTENSION_XF86VIDMODE}

  LOG('creating the standard NoModeSwitching mode switcher');
  Result := TX11ModesNoModeSwitching.Create(FDisplay, FScreen);
<<<<<<< HEAD
<<<<<<< HEAD
end;

function TX11WindowDisplay.GetX11Window: TWindow;
begin
  Result := FWindow;
end;

function TX11WindowDisplay.GetX11GC: TGC;
begin
  Result := FGC;
end;

function TX11WindowDisplay.IsFullScreen: Boolean;
begin
  Result := FFullScreen;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11WindowDisplay.IsOpen: Boolean;
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
procedure TX11WindowDisplay.CreateColormap; { Register colour maps }
var
  i: Integer;
  r, g, b: Single;
begin
  if FFormat.Bits = 8 then
  begin
    FColours := GetMem(256 * SizeOf(TXColor));
    if FColours = nil then
      raise TPTCError.Create('Cannot allocate colour map cells');
    FCMap := XCreateColormap(FDisplay, RootWindow(FDisplay, FScreen),
                             DefaultVisual(FDisplay, FScreen), AllocAll);
    if FCMap = 0 then
      raise TPTCError.Create('Cannot create colour map');
    XInstallColormap(FDisplay, FCMap);
    XSetWindowColormap(FDisplay, FWindow, FCMap);
  end
  else
    FCMap := 0;

  { Set 332 palette, for now }
  if (FFormat.Bits = 8) and FFormat.Direct then
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
  end;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$IFDEF ENABLE_X11_EXTENSION_GLX}
procedure TX11WindowDisplay.OpenGL_SwapBuffers;
begin
  FGLXFBConfig.SwapBuffers;
end;

procedure TX11WindowDisplay.OpenGL_SetSwapInterval(AInterval: Integer);
begin
  FGLXFBConfig.SetSwapInterval(AInterval);
end;

function TX11WindowDisplay.OpenGL_GetSwapInterval: Integer;
begin
  Result := FGLXFBConfig.GetSwapInterval;
end;
{$ENDIF ENABLE_X11_EXTENSION_GLX}
=======
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
End;

Function TX11WindowDisplay.GetX11Window : TWindow;

Begin
  Result := FWindow;
End;

Function TX11WindowDisplay.GetX11GC : TGC;

Begin
  Result := FGC;
End;

Function TX11WindowDisplay.IsFullScreen : Boolean;

Begin
  Result := FFullScreen;
End;

Procedure TX11WindowDisplay.CreateColormap; { Register colour maps }

Var
  i : Integer;
  r, g, b : Single;

Begin
  If FFormat.Bits = 8 Then
  Begin
    FColours := GetMem(256 * SizeOf(TXColor));
    If FColours = Nil Then
      Raise TPTCError.Create('Cannot allocate colour map cells');
    FCMap := XCreateColormap(FDisplay, RootWindow(FDisplay, FScreen),
                             DefaultVisual(FDisplay, FScreen), AllocAll);
    If FCMap = 0 Then
      Raise TPTCError.Create('Cannot create colour map');
    XInstallColormap(FDisplay, FCMap);
    XSetWindowColormap(FDisplay, FWindow, FCMap);
  End
  Else
    FCMap := 0;

  { Set 332 palette, for now }
  If (FFormat.Bits = 8) And FFormat.Direct Then
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
  End;
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
