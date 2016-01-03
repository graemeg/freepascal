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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
{$INCLUDE x11unikey.inc}
=======
{$INCLUDE xunikey.inc}
>>>>>>> graemeg/cpstrnew

=======

{$INCLUDE xunikey.inc}

>>>>>>> graemeg/cpstrnew
=======

{$INCLUDE xunikey.inc}

>>>>>>> graemeg/cpstrnew
=======

{$INCLUDE xunikey.inc}

>>>>>>> origin/cpstrnew
constructor TX11Display.Create(ADisplay: PDisplay; AScreen: Integer; const AFlags: TX11Flags);
begin
=======
=======
>>>>>>> origin/fixes_2_2
{$INCLUDE xunikey.inc}

Constructor TX11Display.Create(ADisplay : PDisplay; AScreen : Integer; Const AFlags : TX11Flags);

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FFlags := AFlags;

  FDisplay := ADisplay;
  FScreen := AScreen;

  FCopy := TPTCCopy.Create;
  FClear := TPTCClear.Create;
  FPalette := TPTCPalette.Create;
  FClip := TPTCArea.Create;
  FArea := TPTCArea.Create;
  FFormat := TPTCFormat.Create;
  FEventQueue := TEventQueue.Create;

  SetKeyMapping;
<<<<<<< HEAD
<<<<<<< HEAD
end;

destructor TX11Display.Destroy;
begin
  { Just close the display, everything else is done by the subclasses }
  if (FDisplay <> Nil) and (not (PTC_X11_LEAVE_DISPLAY In FFlags)) then
  begin
    XFlush(FDisplay);
    XCloseDisplay(FDisplay);
    FDisplay := nil;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
End;

Destructor TX11Display.Destroy;

Begin
  { Just close the display, everything else is done by the subclasses }
  If (FDisplay <> Nil) And (Not (PTC_X11_LEAVE_DISPLAY In FFlags)) Then
  Begin
    XFlush(FDisplay);
    XCloseDisplay(FDisplay);
    FDisplay := Nil;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FreeMemAndNil(FNormalKeys);
  FreeMemAndNil(FFunctionKeys);

  FCopy.Free;
  FClear.Free;
<<<<<<< HEAD
<<<<<<< HEAD
  FEventQueue.Free;

  inherited Destroy;
end;

procedure TX11Display.Load(const APixels: Pointer; AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat; APalette: IPTCPalette);
var
=======
                           const AFormat: TPTCFormat; const APalette: TPTCPalette);
var
  Area_: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; const APalette: TPTCPalette);
var
  Area_: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; const APalette: TPTCPalette);
var
  Area_: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; const APalette: TPTCPalette);
var
  Area_: TPTCArea;
>>>>>>> origin/cpstrnew
  console_pixels: Pointer;
begin
  if Clip.Equals(Area) then
  begin
    try
      console_pixels := Lock;
      try
=======
=======
>>>>>>> origin/fixes_2_2
  FPalette.Free;
  FClip.Free;
  FArea.Free;
  FFormat.Free;
  FEventQueue.Free;

  Inherited Destroy;
End;

Procedure TX11Display.Load(Const APixels : Pointer; AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat; Const APalette : TPTCPalette);
Var
  Area_ : TPTCArea;
  console_pixels : Pointer;

Begin
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
    Load(APixels, AWidth, AHeight, APitch, AFormat, APalette,
         TPTCArea.Create(0, 0, width, height), Area);
end;

procedure TX11Display.Load(const APixels: Pointer; AWidth, AHeight, APitch: Integer;
                           AFormat: IPTCFormat; APalette: IPTCPalette;
                           ASource, ADestination: IPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: IPTCArea;
begin
  try
    console_pixels := Lock;
    try
      TPTCClipper.Clip(ASource, TPTCArea.Create(0, 0, AWidth, AHeight),
                       clipped_source,
                       ADestination, Clip,
                       clipped_destination);
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
end;

procedure TX11Display.Load(const APixels: Pointer; AWidth, AHeight, APitch: Integer;
                           const AFormat: TPTCFormat; const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
var
  console_pixels: Pointer;
  clipped_source, clipped_destination: TPTCArea;
  tmp: TPTCArea;
begin
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

Procedure TX11Display.Load(Const APixels : Pointer; AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat; Const APalette : TPTCPalette;
                           Const ASource, ADestination : TPTCArea);
Var
  console_pixels : Pointer;
  clipped_source, clipped_destination : TPTCArea;
  tmp : TPTCArea;

Begin
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

procedure TX11Display.Save(APixels: Pointer; AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat; APalette: IPTCPalette);
=======
                           const AFormat: TPTCFormat; const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; const APalette: TPTCPalette);
>>>>>>> origin/cpstrnew
begin
end;

procedure TX11Display.Save(APixels: Pointer; AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat; APalette: IPTCPalette;
                           ASource, ADestination: IPTCArea);
begin
end;

procedure TX11Display.Clear(AColor: IPTCColor);
begin
end;

procedure TX11Display.Clear(AColor: IPTCColor; AArea: IPTCArea);
begin
end;

function TX11Display.Palette: IPTCPalette;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                           const AFormat: TPTCFormat; const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
begin
end;

procedure TX11Display.Clear(const AColor: TPTCColor);
begin
end;

procedure TX11Display.Clear(const AColor: TPTCColor; const AArea: TPTCArea);
begin
end;

function TX11Display.Palette: TPTCPalette;
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
  Result := FPalette;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11Display.Clip(AArea: IPTCArea);
begin
  FClip := AArea;
=======
procedure TX11Display.Clip(const AArea: TPTCArea);
begin
  FClip.Assign(AArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Display.Clip(const AArea: TPTCArea);
begin
  FClip.Assign(AArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Display.Clip(const AArea: TPTCArea);
begin
  FClip.Assign(AArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Display.Clip(const AArea: TPTCArea);
begin
  FClip.Assign(AArea);
>>>>>>> origin/cpstrnew
end;

function TX11Display.GetWidth: Integer;
begin
  Result := FWidth;
end;

function TX11Display.GetHeight: Integer;
begin
  Result := FHeight;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11Display.Clip: IPTCArea;
=======
function TX11Display.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TX11Display.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TX11Display.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TX11Display.Clip: TPTCArea;
>>>>>>> origin/cpstrnew
begin
  Result := FClip;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11Display.GetArea: IPTCArea;
begin
  FArea := TPTCArea.Create(0, 0, FWidth, FHeight);
  Result := FArea;
end;

function TX11Display.GetFormat: IPTCFormat;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TX11Display.GetArea: TPTCArea;
var
  tmp: TPTCArea;
begin
  tmp := TPTCArea.Create(0, 0, FWidth, FHeight);
  try
    FArea.Assign(tmp);
  finally
    tmp.Free;
  end;
  Result := FArea;
end;

function TX11Display.GetFormat: TPTCFormat;
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
  Result := FFormat;
end;

procedure TX11Display.SetFlags(AFlags: TX11Flags);
begin
  FFlags := AFlags;
end;

function TX11Display.GetX11Display: PDisplay;
begin
  Result := FDisplay;
end;

function TX11Display.GetX11Screen: Integer;
begin
  Result := FScreen;
end;

function TX11Display.GetX11DirectFormat(ABitsPerPixel: Integer;
                                        AR, AG, AB: DWord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                        AByteOrder: cint): IPTCFormat;
=======
                                        AByteOrder: cint): TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
                                        AByteOrder: cint): TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
                                        AByteOrder: cint): TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
                                        AByteOrder: cint): TPTCFormat;
>>>>>>> origin/cpstrnew

  procedure Swap16(var S: DWord);
  begin
    S := ((S and $FF) shl 8) or
         ((S shr 8) and $FF);
  end;

  procedure Swap24(var S: DWord);
  begin
    S := ((S and $FF) shl 16) or
          (S and $FF00) or
         ((S shr 16) and $FF);
  end;

  procedure Swap32(var S: DWord);
  begin
    S := ((S and $FF) shl 24) or
         ((S and $FF00) shl 8) or
         ((S shr 8) and $FF00) or
          (S shr 24);
  end;

const
  {$IFDEF FPC_LITTLE_ENDIAN}
  ClientByteOrder = LSBFirst;
  {$ELSE FPC_LITTLE_ENDIAN}
  ClientByteOrder = MSBFirst;
  {$ENDIF FPC_LITTLE_ENDIAN}
begin
  if AByteOrder <> ClientByteOrder then
  begin
    { server has different endianness than us - swap masks }
    { TODO: fix - this will probably create bad masks (non-contiguous bits) for 16 bpp }

    case ABitsPerPixel of
      8: {do nothing};
      16:
        begin
          Swap16(AR);
          Swap16(AG);
          Swap16(AB);
        end;
      24:
        begin
          Swap24(AR);
          Swap24(AG);
          Swap24(AB);
        end;
      32:
        begin
          Swap32(AR);
          Swap32(AG);
          Swap32(AB);
        end;
    end;
  end;

  Result := TPTCFormat.Create(ABitsPerPixel, AR, AG, AB);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11Display.GetX11Format(AFormat: IPTCFormat): IPTCFormat;
=======
function TX11Display.GetX11Format(const AFormat: TPTCFormat): TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TX11Display.GetX11Format(const AFormat: TPTCFormat): TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TX11Display.GetX11Format(const AFormat: TPTCFormat): TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TX11Display.GetX11Format(const AFormat: TPTCFormat): TPTCFormat;
>>>>>>> origin/cpstrnew
var
  TmpDepth: Integer;
  NumFound: Integer;
  I: Integer;
  PixmapFormatValues: PXPixmapFormatValues;
  ServerByteOrder: cint;
  R, G, B: DWord;
begin
  ServerByteOrder := ImageByteOrder(FDisplay);

  { Check if our screen has the same format available. I hate how X }
  { keeps bits_per_pixel and depth different }
  TmpDepth := DisplayPlanes(FDisplay, FScreen);

  PixmapFormatValues := XListPixmapFormats(FDisplay, @NumFound);
  try
    for I := 0 to NumFound - 1 do
    begin
      if PixmapFormatValues[I].depth = TmpDepth then
      begin
        TmpDepth := PixmapFormatValues[I].bits_per_pixel;
        break;
      end;
    end;
  finally
    XFree(PixmapFormatValues);
  end;

  if (TmpDepth = 8) and AFormat.Indexed then
    Result := TPTCFormat.Create(8)
  else
    if (TmpDepth = 8) and AFormat.Direct then
      Result := TPTCFormat.Create(8, $E0, $1C, $03)
    else
    begin
      R := DefaultVisual(FDisplay, FScreen)^.red_mask;
      G := DefaultVisual(FDisplay, FScreen)^.green_mask;
      B := DefaultVisual(FDisplay, FScreen)^.blue_mask;

      Result := GetX11DirectFormat(TmpDepth, R, G, B, ServerByteOrder);
    end;
end;

procedure TX11Display.SetKeyMapping;
var
  _I: Integer;
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

Procedure TX11Display.Save(APixels : Pointer; AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat; Const APalette : TPTCPalette);

Begin
End;

Procedure TX11Display.Save(APixels : Pointer; AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat; Const APalette : TPTCPalette;
                           Const ASource, ADestination : TPTCArea);

Begin
End;

Procedure TX11Display.Clear(Const AColor : TPTCColor);

Begin
End;

Procedure TX11Display.Clear(Const AColor : TPTCColor; Const AArea : TPTCArea);

Begin
End;

Function TX11Display.Palette : TPTCPalette;

Begin
  Result := FPalette;
End;

Procedure TX11Display.Clip(Const AArea : TPTCArea);

Begin
  FClip.Assign(AArea);
End;

Function TX11Display.GetWidth : Integer;

Begin
  Result := FWidth;
End;

Function TX11Display.GetHeight : Integer;

Begin
  Result := FHeight;
End;

Function TX11Display.Clip : TPTCArea;

Begin
  Result := FClip;
End;

Function TX11Display.GetArea : TPTCArea;

Var
  tmp : TPTCArea;

Begin
  tmp := TPTCArea.Create(0, 0, FWidth, FHeight);
  Try
    FArea.Assign(tmp);
  Finally
    tmp.Free;
  End;
  Result := FArea;
End;

Function TX11Display.GetFormat : TPTCFormat;

Begin
  Result := FFormat;
End;

Procedure TX11Display.SetFlags(AFlags : TX11Flags);

Begin
  FFlags := AFlags;
End;

Function TX11Display.GetX11Display : PDisplay;

Begin
  Result := FDisplay;
End;

Function TX11Display.GetX11Screen : Integer;

Begin
  Result := FScreen;
End;

Function TX11Display.GetX11Format(Const AFormat : TPTCFormat) : TPTCFormat;

Var
  tmp_depth : Integer;
  numfound : Integer;
  i : Integer;
  pfv : PXPixmapFormatValues;

Begin
  Result := Nil;

  { Check if our screen has the same format available. I hate how X }
  { keeps bits_per_pixel and depth different }
  tmp_depth := DisplayPlanes(FDisplay, FScreen);

  pfv := XListPixmapFormats(FDisplay, @numfound);
  Try
    For i := 0 To numfound - 1 Do
    Begin
      If pfv[i].depth = tmp_depth Then
      Begin
        tmp_depth := pfv[i].bits_per_pixel;
        Break;
      End;
    End;
  Finally
    XFree(pfv);
  End;

  If (tmp_depth = 8) And AFormat.Indexed Then
    Result := TPTCFormat.Create(8)
  Else
    If (tmp_depth = 8) And AFormat.Direct Then
      Result := TPTCFormat.Create(8, $E0, $1C, $03)
    Else
      Result := TPTCFormat.Create(tmp_depth,
                                 DefaultVisual(FDisplay, FScreen)^.red_mask,
                                 DefaultVisual(FDisplay, FScreen)^.green_mask,
                                 DefaultVisual(FDisplay, FScreen)^.blue_mask);
End;

Procedure TX11Display.SetKeyMapping;

Var
  _I : Integer;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FreeMemAndNil(FFunctionKeys);
  FreeMemAndNil(FNormalKeys);
  FFunctionKeys := GetMem(256 * SizeOf(Integer));
  FNormalKeys := GetMem(256 * SizeOf(Integer));

<<<<<<< HEAD
<<<<<<< HEAD
  for _I := 0 to 255 do
  begin
    FFunctionKeys[_I] := Integer(PTCKEY_UNDEFINED);
    FNormalKeys[_I] := Integer(PTCKEY_UNDEFINED);
  end;

  { Assign function key indices from X definitions }
  FFunctionKeys[$FF and XK_BackSpace] := Integer(PTCKEY_BACKSPACE);
  FFunctionKeys[$FF and XK_Tab] := Integer(PTCKEY_TAB);
  FFunctionKeys[$FF and XK_Clear] := Integer(PTCKEY_CLEAR);
  FFunctionKeys[$FF and XK_Return] := Integer(PTCKEY_ENTER);
  FFunctionKeys[$FF and XK_Pause] := Integer(PTCKEY_PAUSE);
  FFunctionKeys[$FF and XK_Scroll_Lock] := Integer(PTCKEY_SCROLLLOCK);
  FFunctionKeys[$FF and XK_Escape] := Integer(PTCKEY_ESCAPE);
  FFunctionKeys[$FF and XK_Delete] := Integer(PTCKEY_DELETE);

  FFunctionKeys[$FF and XK_Kanji] := Integer(PTCKEY_KANJI);

  FFunctionKeys[$FF and XK_Home] := Integer(PTCKEY_HOME);
  FFunctionKeys[$FF and XK_Left] := Integer(PTCKEY_LEFT);
  FFunctionKeys[$FF and XK_Up] := Integer(PTCKEY_UP);
  FFunctionKeys[$FF and XK_Right] := Integer(PTCKEY_RIGHT);
  FFunctionKeys[$FF and XK_Down] := Integer(PTCKEY_DOWN);
  FFunctionKeys[$FF and XK_Page_Up] := Integer(PTCKEY_PAGEUP);
  FFunctionKeys[$FF and XK_Page_Down] := Integer(PTCKEY_PAGEDOWN);
  FFunctionKeys[$FF and XK_End] := Integer(PTCKEY_END);

  FFunctionKeys[$FF and XK_Print] := Integer(PTCKEY_PRINTSCREEN);
  FFunctionKeys[$FF and XK_Insert] := Integer(PTCKEY_INSERT);
  FFunctionKeys[$FF and XK_Num_Lock] := Integer(PTCKEY_NUMLOCK);

  FFunctionKeys[$FF and XK_KP_0] := Integer(PTCKEY_NUMPAD0);
  FFunctionKeys[$FF and XK_KP_1] := Integer(PTCKEY_NUMPAD1);
  FFunctionKeys[$FF and XK_KP_2] := Integer(PTCKEY_NUMPAD2);
  FFunctionKeys[$FF and XK_KP_3] := Integer(PTCKEY_NUMPAD3);
  FFunctionKeys[$FF and XK_KP_4] := Integer(PTCKEY_NUMPAD4);
  FFunctionKeys[$FF and XK_KP_5] := Integer(PTCKEY_NUMPAD5);
  FFunctionKeys[$FF and XK_KP_6] := Integer(PTCKEY_NUMPAD6);
  FFunctionKeys[$FF and XK_KP_7] := Integer(PTCKEY_NUMPAD7);
  FFunctionKeys[$FF and XK_KP_8] := Integer(PTCKEY_NUMPAD8);
  FFunctionKeys[$FF and XK_KP_9] := Integer(PTCKEY_NUMPAD9);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FFunctionKeys[$FF and XK_KP_Home] := Integer(PTCKEY_HOME);
  FFunctionKeys[$FF and XK_KP_Left] := Integer(PTCKEY_LEFT);
  FFunctionKeys[$FF and XK_KP_Up] := Integer(PTCKEY_UP);
  FFunctionKeys[$FF and XK_KP_Right] := Integer(PTCKEY_RIGHT);
  FFunctionKeys[$FF and XK_KP_Down] := Integer(PTCKEY_DOWN);
  FFunctionKeys[$FF and XK_KP_Page_Up] := Integer(PTCKEY_PAGEUP);
  FFunctionKeys[$FF and XK_KP_Page_Down] := Integer(PTCKEY_PAGEDOWN);
  FFunctionKeys[$FF and XK_KP_End] := Integer(PTCKEY_END);
  FFunctionKeys[$FF and XK_KP_Begin] := Integer(PTCKEY_CLEAR);
  FFunctionKeys[$FF and XK_KP_Insert] := Integer(PTCKEY_INSERT);
  FFunctionKeys[$FF and XK_KP_Delete] := Integer(PTCKEY_DELETE);
  FFunctionKeys[$FF and XK_KP_Multiply] := Integer(PTCKEY_MULTIPLY);
  FFunctionKeys[$FF and XK_KP_Divide] := Integer(PTCKEY_DIVIDE);
  FFunctionKeys[$FF and XK_KP_Add] := Integer(PTCKEY_ADD);
  FFunctionKeys[$FF and XK_KP_Subtract] := Integer(PTCKEY_SUBTRACT);
  FFunctionKeys[$FF and XK_KP_Decimal] := Integer(PTCKEY_DECIMAL);
  FFunctionKeys[$FF and XK_KP_Enter] := Integer(PTCKEY_ENTER);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  FFunctionKeys[$FF and XK_F1] := Integer(PTCKEY_F1);
  FFunctionKeys[$FF and XK_F2] := Integer(PTCKEY_F2);
  FFunctionKeys[$FF and XK_F3] := Integer(PTCKEY_F3);
  FFunctionKeys[$FF and XK_F4] := Integer(PTCKEY_F4);
  FFunctionKeys[$FF and XK_F5] := Integer(PTCKEY_F5);
  FFunctionKeys[$FF and XK_F6] := Integer(PTCKEY_F6);
  FFunctionKeys[$FF and XK_F7] := Integer(PTCKEY_F7);
  FFunctionKeys[$FF and XK_F8] := Integer(PTCKEY_F8);
  FFunctionKeys[$FF and XK_F9] := Integer(PTCKEY_F9);
  FFunctionKeys[$FF and XK_F10] := Integer(PTCKEY_F10);
  FFunctionKeys[$FF and XK_F11] := Integer(PTCKEY_F11);
  FFunctionKeys[$FF and XK_F12] := Integer(PTCKEY_F12);

  FFunctionKeys[$FF and XK_Shift_L] := Integer(PTCKEY_SHIFT);
  FFunctionKeys[$FF and XK_Shift_R] := Integer(PTCKEY_SHIFT);
  FFunctionKeys[$FF and XK_Control_L] := Integer(PTCKEY_CONTROL);
  FFunctionKeys[$FF and XK_Control_R] := Integer(PTCKEY_CONTROL);
  FFunctionKeys[$FF and XK_Caps_Lock] := Integer(PTCKEY_CAPSLOCK);
  FFunctionKeys[$FF and XK_Meta_L] := Integer(PTCKEY_META);
  FFunctionKeys[$FF and XK_Meta_R] := Integer(PTCKEY_META);
  FFunctionKeys[$FF and XK_Alt_L] := Integer(PTCKEY_ALT);
  FFunctionKeys[$FF and XK_Alt_R] := Integer(PTCKEY_ALT);

  { Assign normal key indices }
  FNormalKeys[$FF and XK_space] := Integer(PTCKEY_SPACE);
  FNormalKeys[$FF and XK_comma] := Integer(PTCKEY_COMMA);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FNormalKeys[$FF and XK_minus] := Integer(PTCKEY_MINUS);
=======
  FNormalKeys[$FF and XK_minus] := Integer(PTCKEY_SUBTRACT);
>>>>>>> graemeg/cpstrnew
=======
  FNormalKeys[$FF and XK_minus] := Integer(PTCKEY_SUBTRACT);
>>>>>>> graemeg/cpstrnew
=======
  FNormalKeys[$FF and XK_minus] := Integer(PTCKEY_SUBTRACT);
>>>>>>> graemeg/cpstrnew
=======
  FNormalKeys[$FF and XK_minus] := Integer(PTCKEY_SUBTRACT);
>>>>>>> origin/cpstrnew
  FNormalKeys[$FF and XK_period] := Integer(PTCKEY_PERIOD);
  FNormalKeys[$FF and XK_slash] := Integer(PTCKEY_SLASH);
  FNormalKeys[$FF and XK_0] := Integer(PTCKEY_ZERO);
  FNormalKeys[$FF and XK_1] := Integer(PTCKEY_ONE);
  FNormalKeys[$FF and XK_2] := Integer(PTCKEY_TWO);
  FNormalKeys[$FF and XK_3] := Integer(PTCKEY_THREE);
  FNormalKeys[$FF and XK_4] := Integer(PTCKEY_FOUR);
  FNormalKeys[$FF and XK_5] := Integer(PTCKEY_FIVE);
  FNormalKeys[$FF and XK_6] := Integer(PTCKEY_SIX);
  FNormalKeys[$FF and XK_7] := Integer(PTCKEY_SEVEN);
  FNormalKeys[$FF and XK_8] := Integer(PTCKEY_EIGHT);
  FNormalKeys[$FF and XK_9] := Integer(PTCKEY_NINE);
  FNormalKeys[$FF and XK_semicolon] := Integer(PTCKEY_SEMICOLON);
  FNormalKeys[$FF and XK_equal] := Integer(PTCKEY_EQUALS);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FNormalKeys[$FF and XK_grave] := Integer(PTCKEY_BACKQUOTE);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  FNormalKeys[$FF and XK_bracketleft] := Integer(PTCKEY_OPENBRACKET);
  FNormalKeys[$FF and XK_backslash] := Integer(PTCKEY_BACKSLASH);
  FNormalKeys[$FF and XK_bracketright] := Integer(PTCKEY_CLOSEBRACKET);

  FNormalKeys[$FF and XK_a] := Integer(PTCKEY_A);
  FNormalKeys[$FF and XK_b] := Integer(PTCKEY_B);
  FNormalKeys[$FF and XK_c] := Integer(PTCKEY_C);
  FNormalKeys[$FF and XK_d] := Integer(PTCKEY_D);
  FNormalKeys[$FF and XK_e] := Integer(PTCKEY_E);
  FNormalKeys[$FF and XK_f] := Integer(PTCKEY_F);
  FNormalKeys[$FF and XK_g] := Integer(PTCKEY_G);
  FNormalKeys[$FF and XK_h] := Integer(PTCKEY_H);
  FNormalKeys[$FF and XK_i] := Integer(PTCKEY_I);
  FNormalKeys[$FF and XK_j] := Integer(PTCKEY_J);
  FNormalKeys[$FF and XK_k] := Integer(PTCKEY_K);
  FNormalKeys[$FF and XK_l] := Integer(PTCKEY_L);
  FNormalKeys[$FF and XK_m] := Integer(PTCKEY_M);
  FNormalKeys[$FF and XK_n] := Integer(PTCKEY_N);
  FNormalKeys[$FF and XK_o] := Integer(PTCKEY_O);
  FNormalKeys[$FF and XK_p] := Integer(PTCKEY_P);
  FNormalKeys[$FF and XK_q] := Integer(PTCKEY_Q);
  FNormalKeys[$FF and XK_r] := Integer(PTCKEY_R);
  FNormalKeys[$FF and XK_s] := Integer(PTCKEY_S);
  FNormalKeys[$FF and XK_t] := Integer(PTCKEY_T);
  FNormalKeys[$FF and XK_u] := Integer(PTCKEY_U);
  FNormalKeys[$FF and XK_v] := Integer(PTCKEY_V);
  FNormalKeys[$FF and XK_w] := Integer(PTCKEY_W);
  FNormalKeys[$FF and XK_x] := Integer(PTCKEY_X);
  FNormalKeys[$FF and XK_y] := Integer(PTCKEY_Y);
  FNormalKeys[$FF and XK_z] := Integer(PTCKEY_Z);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TX11Display.HandleKeyEvent(const e: TXKeyEvent);
var
  sym: TKeySym;
  sym_modded: TKeySym; { modifiers like shift are taken into account here }
  press: Boolean;
  alt, shift, ctrl: Boolean;
  uni: Integer;
  key: TPTCKeyEvent;
  buf: array [1..16] of Char;
begin
  sym := XLookupKeySym(@e, 0);
  XLookupString(@e, @buf, SizeOf(buf), @sym_modded, nil);
//  Writeln('sym_modded = ', sym_modded);
  uni := X11ConvertKeySymToUnicode(sym_modded);
  alt := (e.state and Mod1Mask) <> 0;
  shift := (e.state and ShiftMask) <> 0;
  ctrl := (e.state and ControlMask) <> 0;
  if e._type = KeyPress then
    press := True
  else
    press := False;

  // XK_ISO_Left_Tab is Shift-Tab
  if sym_modded = XK_ISO_Left_Tab then
  begin
    sym_modded := XK_Tab;
    uni := 9;
    shift := True;
  end;

  // Hack, used for handling the code of Shift-Key combinations
  case sym_modded shr 8 of
    0:
      begin
        if FNormalKeys[sym_modded and $FF] = 0 then
          sym_modded := sym;
      end;
    $FF:
      begin
        if FFunctionKeys[sym_modded and $FF] = 0 then
          sym_modded := sym;
      end;
  end;

  key := nil;
  case sym_modded shr 8 of
    0: key := TPTCKeyEvent.Create(FNormalKeys[sym_modded and $FF], uni, alt, shift, ctrl, press);
    $FF: key := TPTCKeyEvent.Create(FFunctionKeys[sym_modded and $FF], uni, alt, shift, ctrl, press);
    else
      key := TPTCKeyEvent.Create;
  end;
  FEventQueue.AddEvent(key);
end;

function TX11Display.GetInterceptClose: Boolean;
begin
  Result := PTC_X11_INTERCEPT_WINDOW_CLOSE in FFlags;
end;

procedure TX11Display.SetInterceptClose(AInterceptClose: Boolean);
begin
  if AInterceptClose then
    FFlags := FFlags + [PTC_X11_INTERCEPT_WINDOW_CLOSE]
  else
    FFlags := FFlags - [PTC_X11_INTERCEPT_WINDOW_CLOSE];
end;

procedure TX11Display.InternalResize(AWidth, AHeight: Integer);
begin
  raise TPTCError.Create('Console not in windowed mode');
end;

procedure TX11Display.OpenGL_SwapBuffers;
begin
  raise TPTCError.Create('Not in OpenGL mode');
end;

procedure TX11Display.OpenGL_SetSwapInterval(AInterval: Integer);
begin
  raise TPTCError.Create('Not in OpenGL mode');
end;

function TX11Display.OpenGL_GetSwapInterval: Integer;
begin
  Result := -1;
  raise TPTCError.Create('Not in OpenGL mode');
end;
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
  For _I := 0 To 255 Do
  Begin
    FFunctionKeys[_I] := Integer(PTCKEY_UNDEFINED);
    FNormalKeys[_I] := Integer(PTCKEY_UNDEFINED);
  End;

  { Assign function key indices from X definitions }
  FFunctionKeys[$FF And XK_BackSpace] := Integer(PTCKEY_BACKSPACE);
  FFunctionKeys[$FF And XK_Tab] := Integer(PTCKEY_TAB);
  FFunctionKeys[$FF And XK_Clear] := Integer(PTCKEY_CLEAR);
  FFunctionKeys[$FF And XK_Return] := Integer(PTCKEY_ENTER);
  FFunctionKeys[$FF And XK_Pause] := Integer(PTCKEY_PAUSE);
  FFunctionKeys[$FF And XK_Scroll_Lock] := Integer(PTCKEY_SCROLLLOCK);
  FFunctionKeys[$FF And XK_Escape] := Integer(PTCKEY_ESCAPE);
  FFunctionKeys[$FF And XK_Delete] := Integer(PTCKEY_DELETE);

  FFunctionKeys[$FF And XK_Kanji] := Integer(PTCKEY_KANJI);

  FFunctionKeys[$FF And XK_Home] := Integer(PTCKEY_HOME);
  FFunctionKeys[$FF And XK_Left] := Integer(PTCKEY_LEFT);
  FFunctionKeys[$FF And XK_Up] := Integer(PTCKEY_UP);
  FFunctionKeys[$FF And XK_Right] := Integer(PTCKEY_RIGHT);
  FFunctionKeys[$FF And XK_Down] := Integer(PTCKEY_DOWN);
  FFunctionKeys[$FF And XK_Page_Up] := Integer(PTCKEY_PAGEUP);
  FFunctionKeys[$FF And XK_Page_Down] := Integer(PTCKEY_PAGEDOWN);
  FFunctionKeys[$FF And XK_End] := Integer(PTCKEY_END);

  FFunctionKeys[$FF And XK_Print] := Integer(PTCKEY_PRINTSCREEN);
  FFunctionKeys[$FF And XK_Insert] := Integer(PTCKEY_INSERT);
  FFunctionKeys[$FF And XK_Num_Lock] := Integer(PTCKEY_NUMLOCK);

  FFunctionKeys[$FF And XK_KP_0] := Integer(PTCKEY_NUMPAD0);
  FFunctionKeys[$FF And XK_KP_1] := Integer(PTCKEY_NUMPAD1);
  FFunctionKeys[$FF And XK_KP_2] := Integer(PTCKEY_NUMPAD2);
  FFunctionKeys[$FF And XK_KP_3] := Integer(PTCKEY_NUMPAD3);
  FFunctionKeys[$FF And XK_KP_4] := Integer(PTCKEY_NUMPAD4);
  FFunctionKeys[$FF And XK_KP_5] := Integer(PTCKEY_NUMPAD5);
  FFunctionKeys[$FF And XK_KP_6] := Integer(PTCKEY_NUMPAD6);
  FFunctionKeys[$FF And XK_KP_7] := Integer(PTCKEY_NUMPAD7);
  FFunctionKeys[$FF And XK_KP_8] := Integer(PTCKEY_NUMPAD8);
  FFunctionKeys[$FF And XK_KP_9] := Integer(PTCKEY_NUMPAD9);

  FFunctionKeys[$FF And XK_F1] := Integer(PTCKEY_F1);
  FFunctionKeys[$FF And XK_F2] := Integer(PTCKEY_F2);
  FFunctionKeys[$FF And XK_F3] := Integer(PTCKEY_F3);
  FFunctionKeys[$FF And XK_F4] := Integer(PTCKEY_F4);
  FFunctionKeys[$FF And XK_F5] := Integer(PTCKEY_F5);
  FFunctionKeys[$FF And XK_F6] := Integer(PTCKEY_F6);
  FFunctionKeys[$FF And XK_F7] := Integer(PTCKEY_F7);
  FFunctionKeys[$FF And XK_F8] := Integer(PTCKEY_F8);
  FFunctionKeys[$FF And XK_F9] := Integer(PTCKEY_F9);
  FFunctionKeys[$FF And XK_F10] := Integer(PTCKEY_F10);
  FFunctionKeys[$FF And XK_F11] := Integer(PTCKEY_F11);
  FFunctionKeys[$FF And XK_F12] := Integer(PTCKEY_F12);

  FFunctionKeys[$FF And XK_Shift_L] := Integer(PTCKEY_SHIFT);
  FFunctionKeys[$FF And XK_Shift_R] := Integer(PTCKEY_SHIFT);
  FFunctionKeys[$FF And XK_Control_L] := Integer(PTCKEY_CONTROL);
  FFunctionKeys[$FF And XK_Control_R] := Integer(PTCKEY_CONTROL);
  FFunctionKeys[$FF And XK_Caps_Lock] := Integer(PTCKEY_CAPSLOCK);
  FFunctionKeys[$FF And XK_Meta_L] := Integer(PTCKEY_META);
  FFunctionKeys[$FF And XK_Meta_R] := Integer(PTCKEY_META);
  FFunctionKeys[$FF And XK_Alt_L] := Integer(PTCKEY_ALT);
  FFunctionKeys[$FF And XK_Alt_R] := Integer(PTCKEY_ALT);

  { Assign normal key indices }
  FNormalKeys[$FF And XK_space] := Integer(PTCKEY_SPACE);
  FNormalKeys[$FF And XK_comma] := Integer(PTCKEY_COMMA);
  FNormalKeys[$FF And XK_minus] := Integer(PTCKEY_SUBTRACT);
  FNormalKeys[$FF And XK_period] := Integer(PTCKEY_PERIOD);
  FNormalKeys[$FF And XK_slash] := Integer(PTCKEY_SLASH);
  FNormalKeys[$FF And XK_0] := Integer(PTCKEY_ZERO);
  FNormalKeys[$FF And XK_1] := Integer(PTCKEY_ONE);
  FNormalKeys[$FF And XK_2] := Integer(PTCKEY_TWO);
  FNormalKeys[$FF And XK_3] := Integer(PTCKEY_THREE);
  FNormalKeys[$FF And XK_4] := Integer(PTCKEY_FOUR);
  FNormalKeys[$FF And XK_5] := Integer(PTCKEY_FIVE);
  FNormalKeys[$FF And XK_6] := Integer(PTCKEY_SIX);
  FNormalKeys[$FF And XK_7] := Integer(PTCKEY_SEVEN);
  FNormalKeys[$FF And XK_8] := Integer(PTCKEY_EIGHT);
  FNormalKeys[$FF And XK_9] := Integer(PTCKEY_NINE);
  FNormalKeys[$FF And XK_semicolon] := Integer(PTCKEY_SEMICOLON);
  FNormalKeys[$FF And XK_equal] := Integer(PTCKEY_EQUALS);

  FNormalKeys[$FF And XK_bracketleft] := Integer(PTCKEY_OPENBRACKET);
  FNormalKeys[$FF And XK_backslash] := Integer(PTCKEY_BACKSLASH);
  FNormalKeys[$FF And XK_bracketright] := Integer(PTCKEY_CLOSEBRACKET);

  FNormalKeys[$FF And XK_a] := Integer(PTCKEY_A);
  FNormalKeys[$FF And XK_b] := Integer(PTCKEY_B);
  FNormalKeys[$FF And XK_c] := Integer(PTCKEY_C);
  FNormalKeys[$FF And XK_d] := Integer(PTCKEY_D);
  FNormalKeys[$FF And XK_e] := Integer(PTCKEY_E);
  FNormalKeys[$FF And XK_f] := Integer(PTCKEY_F);
  FNormalKeys[$FF And XK_g] := Integer(PTCKEY_G);
  FNormalKeys[$FF And XK_h] := Integer(PTCKEY_H);
  FNormalKeys[$FF And XK_i] := Integer(PTCKEY_I);
  FNormalKeys[$FF And XK_j] := Integer(PTCKEY_J);
  FNormalKeys[$FF And XK_k] := Integer(PTCKEY_K);
  FNormalKeys[$FF And XK_l] := Integer(PTCKEY_L);
  FNormalKeys[$FF And XK_m] := Integer(PTCKEY_M);
  FNormalKeys[$FF And XK_n] := Integer(PTCKEY_N);
  FNormalKeys[$FF And XK_o] := Integer(PTCKEY_O);
  FNormalKeys[$FF And XK_p] := Integer(PTCKEY_P);
  FNormalKeys[$FF And XK_q] := Integer(PTCKEY_Q);
  FNormalKeys[$FF And XK_r] := Integer(PTCKEY_R);
  FNormalKeys[$FF And XK_s] := Integer(PTCKEY_S);
  FNormalKeys[$FF And XK_t] := Integer(PTCKEY_T);
  FNormalKeys[$FF And XK_u] := Integer(PTCKEY_U);
  FNormalKeys[$FF And XK_v] := Integer(PTCKEY_V);
  FNormalKeys[$FF And XK_w] := Integer(PTCKEY_W);
  FNormalKeys[$FF And XK_x] := Integer(PTCKEY_X);
  FNormalKeys[$FF And XK_y] := Integer(PTCKEY_Y);
  FNormalKeys[$FF And XK_z] := Integer(PTCKEY_Z);
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
