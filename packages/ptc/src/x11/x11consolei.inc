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

constructor TX11Console.Create;
var
  s: AnsiString;
begin
  inherited Create;
=======
=======
>>>>>>> origin/fixes_2_2
Constructor TX11Console.Create;

Var
  s : AnsiString;

Begin
  Inherited Create;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { default flags }
  FFlags := [PTC_X11_TRY_XSHM, PTC_X11_TRY_XF86VIDMODE];

  FTitle := '';

  Configure('/usr/share/ptcpas/ptcpas.conf');
  s := fpgetenv('HOME');
<<<<<<< HEAD
<<<<<<< HEAD
  if s = '' then
    s := '/';
  if s[Length(s)] <> '/' then
    s := s + '/';
  s := s + '.ptcpas.conf';
  Configure(s);
end;

destructor TX11Console.Destroy;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Close;
  FreeAndNil(FX11Display);
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
  Close;
  FreeAndNil(FX11Display);
  for I := Low(FModes) to High(FModes) do
    FreeAndNil(FModes[I]);
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
  inherited Destroy;
end;

procedure TX11Console.Configure(const AFileName: String);
var
  F: TextFile;
  S: string;
begin
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

function TX11Console.Option(const AOption: string): Boolean;
begin
  Result := True;
  if AOption = 'default output' then
  begin
    { default is windowed for now }
    FFlags := FFlags - [PTC_X11_FULLSCREEN];
    exit;
  end;
  if AOption = 'windowed output' then
  begin
    FFlags := FFlags - [PTC_X11_FULLSCREEN];
    exit;
  end;
  if AOption = 'fullscreen output' then
  begin
    FFlags := FFlags + [PTC_X11_FULLSCREEN];
    exit;
  end;
  if AOption = 'leave window open' then
  begin
    FFlags := FFlags + [PTC_X11_LEAVE_WINDOW];
    exit;
  end;
  if AOption = 'leave display open' then
  begin
    FFlags := FFlags + [PTC_X11_LEAVE_DISPLAY];
    exit;
  end;
  if (AOption = 'dga') or (AOption = 'dga on') then
  begin
    FFlags := FFlags + [PTC_X11_TRY_DGA1, PTC_X11_TRY_DGA2];
    exit;
  end;
  if AOption = 'dga off' then
  begin
    FFlags := FFlags - [PTC_X11_TRY_DGA1, PTC_X11_TRY_DGA2];
    exit;
  end;
  if (AOption = 'dga1') or (AOption = 'dga1 on') then
  begin
    FFlags := FFlags + [PTC_X11_TRY_DGA1];
    exit;
  end;
  if AOption = 'dga1 off' then
  begin
    FFlags := FFlags - [PTC_X11_TRY_DGA1];
    exit;
  end;
  if (AOption = 'dga2') or (AOption = 'dga2 on') then
  begin
    FFlags := FFlags + [PTC_X11_TRY_DGA2];
    exit;
  end;
  if AOption = 'dga2 off' then
  begin
    FFlags := FFlags - [PTC_X11_TRY_DGA2];
    exit;
  end;
  if (AOption = 'xf86vidmode') or (AOption = 'xf86vidmode on') then
  begin
    FFlags := FFlags + [PTC_X11_TRY_XF86VIDMODE];
    exit;
  end;
  if AOption = 'xf86vidmode off' then
  begin
    FFlags := FFlags - [PTC_X11_TRY_XF86VIDMODE];
    exit;
  end;
  if (AOption = 'xrandr') or (AOption = 'xrandr on') then
  begin
    FFlags := FFlags + [PTC_X11_TRY_XRANDR];
    exit;
  end;
  if AOption = 'xrandr off' then
  begin
    FFlags := FFlags - [PTC_X11_TRY_XRANDR];
    exit;
  end;
  if (AOption = 'xshm') or (AOption = 'xshm on') then
  begin
    FFlags := FFlags + [PTC_X11_TRY_XSHM];
    exit;
  end;
  if AOption = 'xshm off' then
  begin
    FFlags := FFlags - [PTC_X11_TRY_XSHM];
    exit;
  end;
  if AOption = 'default cursor' then
  begin
    FFlags := FFlags - [PTC_X11_FULLSCREEN_CURSOR_VISIBLE, PTC_X11_WINDOWED_CURSOR_INVISIBLE];
    UpdateCursor;
    exit;
  end;
  if AOption = 'show cursor' then
  begin
    FFlags := (FFlags - [PTC_X11_WINDOWED_CURSOR_INVISIBLE]) + [PTC_X11_FULLSCREEN_CURSOR_VISIBLE];
    UpdateCursor;
    exit;
  end;
  if AOption = 'hide cursor' then
  begin
    FFlags := (FFlags - [PTC_X11_FULLSCREEN_CURSOR_VISIBLE]) + [PTC_X11_WINDOWED_CURSOR_INVISIBLE];
    UpdateCursor;
    exit;
  end;
  if AOption = 'grab mouse' then
  begin
    FFlags := FFlags + [PTC_X11_GRAB_MOUSE];
    UpdateMouseGrab;
    exit;
  end;
  if AOption = 'ungrab mouse' then
  begin
    FFlags := FFlags - [PTC_X11_GRAB_MOUSE];
    UpdateMouseGrab;
    exit;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if AOption = 'intercept window close' then
  begin
    FFlags := FFlags + [PTC_X11_INTERCEPT_WINDOW_CLOSE];
    if Assigned(FX11Display) then
      FX11Display.InterceptClose := True;
    exit;
  end;
  if AOption = 'resizable window' then
  begin
    FFlags := FFlags + [PTC_X11_RESIZABLE_WINDOW];
    exit;
  end;
  if AOption = 'fixed window' then
  begin
    FFlags := FFlags - [PTC_X11_RESIZABLE_WINDOW];
    exit;
  end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

  if Assigned(FX11Display) then
    Result := FX11Display.FCopy.Option(AOption)
  else
    Result := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11Console.GetOpenGL_Enabled: Boolean;
begin
  Result := PTC_X11_USE_OPENGL in FFlags;
end;

procedure TX11Console.SetOpenGL_Enabled(AValue: Boolean);
begin
  if (FX11Display <> nil) and FX11Display.IsOpen then
    raise TPTCError.Create('Value of UseOpenGL cannot be modified while the console is open');
  if AValue then
    FFlags := FFlags + [PTC_X11_USE_OPENGL]
  else
    FFlags := FFlags - [PTC_X11_USE_OPENGL];
end;

function TX11Console.Modes: TPTCModeList;
begin
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TX11Console.Modes: PPTCMode;
var
  I: Integer;
begin
  for I := Low(FModes) to High(FModes) do
    FreeAndNil(FModes[I]);

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
  if FX11Display = nil then
=======
=======
>>>>>>> origin/fixes_2_2
  If s = '' Then
    s := '/';
  If s[Length(s)] <> '/' Then
    s := s + '/';
  s := s + '.ptcpas.conf';
  Configure(s);
End;

Destructor TX11Console.Destroy;

Var
  I : Integer;

Begin
  Close;
  FreeAndNil(FX11Display);
  For I := Low(FModes) To High(FModes) Do
    FreeAndNil(FModes[I]);
  Inherited Destroy;
End;

Procedure TX11Console.Configure(Const AFileName : String);

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

Function TX11Console.Option(Const AOption : String) : Boolean;

Begin
  Result := True;
  If AOption = 'default output' Then
  Begin
    { default is windowed for now }
    FFlags := FFlags - [PTC_X11_FULLSCREEN];
    Exit;
  End;
  If AOption = 'windowed output' Then
  Begin
    FFlags := FFlags - [PTC_X11_FULLSCREEN];
    Exit;
  End;
  If AOption = 'fullscreen output' Then
  Begin
    FFlags := FFlags + [PTC_X11_FULLSCREEN];
    Exit;
  End;
  If AOption = 'leave window open' Then
  Begin
    FFlags := FFlags + [PTC_X11_LEAVE_WINDOW];
    Exit;
  End;
  If AOption = 'leave display open' Then
  Begin
    FFlags := FFlags + [PTC_X11_LEAVE_DISPLAY];
    Exit;
  End;
  If AOption = 'dga' Then
  Begin
    FFlags := FFlags + [PTC_X11_TRY_DGA];
    Exit;
  End;
  If AOption = 'dga off' Then
  Begin
    FFlags := FFlags - [PTC_X11_TRY_DGA];
    Exit;
  End;
  If AOption = 'xf86vidmode' Then
  Begin
    FFlags := FFlags + [PTC_X11_TRY_XF86VIDMODE];
    Exit;
  End;
  If AOption = 'xf86vidmode off' Then
  Begin
    FFlags := FFlags - [PTC_X11_TRY_XF86VIDMODE];
    Exit;
  End;
  If AOption = 'xrandr' Then
  Begin
    FFlags := FFlags + [PTC_X11_TRY_XRANDR];
    Exit;
  End;
  If AOption = 'xrandr off' Then
  Begin
    FFlags := FFlags - [PTC_X11_TRY_XRANDR];
    Exit;
  End;
  If AOption = 'xshm' Then
  Begin
    FFlags := FFlags + [PTC_X11_TRY_XSHM];
    Exit;
  End;
  If AOption = 'xshm off' Then
  Begin
    FFlags := FFlags - [PTC_X11_TRY_XSHM];
    Exit;
  End;
  If AOption = 'default cursor' Then
  Begin
    FFlags := FFlags - [PTC_X11_FULLSCREEN_CURSOR_VISIBLE, PTC_X11_WINDOWED_CURSOR_INVISIBLE];
    UpdateCursor;
    Exit;
  End;
  If AOption = 'show cursor' Then
  Begin
    FFlags := (FFlags - [PTC_X11_WINDOWED_CURSOR_INVISIBLE]) + [PTC_X11_FULLSCREEN_CURSOR_VISIBLE];
    UpdateCursor;
    Exit;
  End;
  If AOption = 'hide cursor' Then
  Begin
    FFlags := (FFlags - [PTC_X11_FULLSCREEN_CURSOR_VISIBLE]) + [PTC_X11_WINDOWED_CURSOR_INVISIBLE];
    UpdateCursor;
    Exit;
  End;
  If AOption = 'enable logging' Then
  Begin
    LOG_enabled := True;
    Result := True;
    Exit;
  End;
  If AOption = 'disable logging' Then
  Begin
    LOG_enabled := False;
    Result := True;
    Exit;
  End;

  If Assigned(FX11Display) Then
    Result := FX11Display.FCopy.Option(AOption)
  Else
    Result := False;
End;

Function TX11Console.Modes : PPTCMode;

Var
  I : Integer;

Begin
  For I := Low(FModes) To High(FModes) Do
    FreeAndNil(FModes[I]);

  If FX11Display = Nil Then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FX11Display := CreateDisplay;

  FX11Display.GetModes(FModes);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Result := FModes;
=======
  Result := @FModes[0];
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

{TODO: Find current pixel depth}
procedure TX11Console.Open(const ATitle: string; APages: Integer = 0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Open(ATitle, TPTCFormat.Create(32, $FF0000, $FF00, $FF), APages);
end;

procedure TX11Console.Open(const ATitle: string; AFormat: IPTCFormat;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var
  tmp: TPTCFormat;
begin
  tmp := TPTCFormat.Create(32, $FF0000, $FF00, $FF);
  try
    Open(ATitle, tmp, APages);
  finally
    tmp.Free;
  end;
end;

procedure TX11Console.Open(const ATitle: string; const AFormat: TPTCFormat;
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
                           APages: Integer = 0);
begin
  Open(ATitle, 640, 480, AFormat, APages);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11Console.Open(const ATitle: string; AMode: IPTCMode;
=======
procedure TX11Console.Open(const ATitle: string; const AMode: TPTCMode;
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Open(const ATitle: string; const AMode: TPTCMode;
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Open(const ATitle: string; const AMode: TPTCMode;
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Open(const ATitle: string; const AMode: TPTCMode;
>>>>>>> origin/cpstrnew
                           APages: Integer = 0);
begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
end;

function TX11Console.CreateDisplay: TX11Display;
var
  display: PDisplay;
  screen: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  DetectableAutoRepeatSupported: Boolean;
begin
  { Check if we can open an X display }
  display := XOpenDisplay(nil);
=======
begin
  { Check if we can open an X display }
  display := XOpenDisplay(Nil);
>>>>>>> graemeg/cpstrnew
=======
begin
  { Check if we can open an X display }
  display := XOpenDisplay(Nil);
>>>>>>> graemeg/cpstrnew
=======
begin
  { Check if we can open an X display }
  display := XOpenDisplay(Nil);
>>>>>>> graemeg/cpstrnew
=======
begin
  { Check if we can open an X display }
  display := XOpenDisplay(Nil);
>>>>>>> origin/cpstrnew
  if display = nil then
    raise TPTCError.Create('Cannot open X display');
=======
=======
>>>>>>> origin/fixes_2_2
  Result := @FModes[0];
End;

{TODO: Find current pixel depth}
Procedure TX11Console.Open(Const ATitle : String; APages : Integer = 0);

Var
  tmp : TPTCFormat;

Begin
  tmp := TPTCFormat.Create(32, $FF0000, $FF00, $FF);
  Try
    Open(ATitle, tmp, APages);
  Finally
    tmp.Free;
  End;
End;

Procedure TX11Console.Open(Const ATitle : String; Const AFormat : TPTCFormat;
                           APages : Integer = 0);

Begin
  Open(ATitle, 640, 480, AFormat, APages);
End;

Procedure TX11Console.Open(Const ATitle : String; Const AMode : TPTCMode;
                           APages : Integer = 0);

Begin
  Open(ATitle, AMode.Width, AMode.Height, AMode.Format, APages);
End;

Function TX11Console.CreateDisplay : TX11Display;

Var
  display : PDisplay;
  screen : Integer;

Begin
  { Check if we can open an X display }
  display := XOpenDisplay(Nil);
  If display = Nil Then
    Raise TPTCError.Create('Cannot open X display');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

  { DefaultScreen should be fine }
  screen := DefaultScreen(display);

<<<<<<< HEAD
<<<<<<< HEAD
  if not XkbSetDetectableAutoRepeat(display, True, @DetectableAutoRepeatSupported) then
    LOG('DetectableAutoRepeat not supported')
  else
    if not DetectableAutoRepeatSupported then
      LOG('DetectableAutoRepeat not supported');

  {$IFDEF ENABLE_X11_EXTENSION_XF86DGA2}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if (PTC_X11_TRY_DGA2 in FFlags) and not (PTC_X11_USE_OPENGL in FFlags) then
=======
  if PTC_X11_TRY_DGA2 In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_DGA2 In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_DGA2 In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_DGA2 In FFlags then
>>>>>>> origin/cpstrnew
  begin
    try
      Result := TX11DGA2Display.Create(display, screen, FFlags + [PTC_X11_LEAVE_DISPLAY]);
      Result.SetFlags(FFlags);
      exit;
    except
      LOG('DGA 2.0 failed');
    end;
  end;
  {$ENDIF ENABLE_X11_EXTENSION_XF86DGA2}

  {$IFDEF ENABLE_X11_EXTENSION_XF86DGA1}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if (PTC_X11_TRY_DGA1 in FFlags) and not (PTC_X11_USE_OPENGL in FFlags) then
=======
  if PTC_X11_TRY_DGA1 In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_DGA1 In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_DGA1 In FFlags then
>>>>>>> graemeg/cpstrnew
=======
  if PTC_X11_TRY_DGA1 In FFlags then
>>>>>>> origin/cpstrnew
  begin
    try
      Result := TX11DGA1Display.Create(display, screen, FFlags + [PTC_X11_LEAVE_DISPLAY]);
      Result.SetFlags(FFlags);
    except
      LOG('DGA 1.0 failed');
    end;
  end;
  {$ENDIF ENABLE_X11_EXTENSION_XF86DGA1}

  Result := TX11WindowDisplay.Create(display, screen, FFlags);
end;

procedure TX11Console.Open(const ATitle: string; AWidth, AHeight: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat; APages: Integer = 0);
=======
                           const AFormat: TPTCFormat; APages: Integer = 0);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; APages: Integer = 0);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; APages: Integer = 0);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; APages: Integer = 0);
>>>>>>> origin/cpstrnew
begin
  Close;
  FTitle := ATitle;

  if FX11Display = nil then
    FX11Display := CreateDisplay;
  FX11Display.Open(ATitle, AWidth, AHeight, AFormat, FOpenGLAttributes);

  UpdateCursor;
end;

procedure TX11Console.Close;
begin
  FreeAndNil(FX11Display);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TX11Console.InternalResize(AWidth, AHeight: Integer);
begin
  FX11Display.InternalResize(AWidth, AHeight);
end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

procedure TX11Console.Flush;
begin
  Update;
end;

procedure TX11Console.Finish;
begin
  Update;
end;

procedure TX11Console.Update;
begin
  FX11Display.Update;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11Console.Update(AArea: IPTCArea);
=======
procedure TX11Console.Update(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Update(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Update(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Update(const AArea: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  FX11Display.Update(AArea);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11Console.NextEvent(out AEvent: IPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
=======
function TX11Console.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
>>>>>>> origin/cpstrnew
begin
  Result := FX11Display.NextEvent(AEvent, AWait, AEventMask);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11Console.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): IPTCEvent;
=======
function TX11Console.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> origin/cpstrnew
begin
  Result := FX11Display.PeekEvent(AWait, AEventMask);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11Console.Copy(ASurface: IPTCSurface);
=======
procedure TX11Console.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Copy(ASurface: TPTCBaseSurface);
>>>>>>> origin/cpstrnew
begin
  {todo!...}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11Console.Copy(ASurface: IPTCSurface;
                           ASource, ADestination: IPTCArea);
=======
procedure TX11Console.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  {todo!...}
end;

function TX11Console.Lock: Pointer;
begin
  Result := FX11Display.Lock;
end;

procedure TX11Console.Unlock;
begin
  FX11Display.Unlock;
end;

procedure TX11Console.Load(const APixels: Pointer;
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
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> origin/cpstrnew
begin
  FX11Display.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette);
end;

procedure TX11Console.Load(const APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat;
                           APalette: IPTCPalette;
                           ASource, ADestination: IPTCArea);
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  FX11Display.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination);
end;

procedure TX11Console.Save(APixels: Pointer;
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
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> origin/cpstrnew
begin
  {todo!...}
end;

procedure TX11Console.Save(APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat;
                           APalette: IPTCPalette;
                           ASource, ADestination: IPTCArea);
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  {todo!...}
end;

procedure TX11Console.Clear;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  tmp: IPTCColor;
=======
  tmp: TPTCColor;
>>>>>>> graemeg/cpstrnew
=======
  tmp: TPTCColor;
>>>>>>> graemeg/cpstrnew
=======
  tmp: TPTCColor;
>>>>>>> graemeg/cpstrnew
=======
  tmp: TPTCColor;
>>>>>>> origin/cpstrnew
begin
  if Format.Direct then
    tmp := TPTCColor.Create(0, 0, 0, 0)
  else
    tmp := TPTCColor.Create(0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  Clear(tmp);
end;

procedure TX11Console.Clear(AColor: IPTCColor);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  try
    Clear(tmp);
  finally
    tmp.Free;
  end;
end;

procedure TX11Console.Clear(const AColor: TPTCColor);
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
  try
    Clear(tmp);
  finally
    tmp.Free;
  end;
end;

procedure TX11Console.Clear(const AColor: TPTCColor);
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  FX11Display.Clear(AColor);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11Console.Clear(AColor: IPTCColor;
                            AArea: IPTCArea);
=======
procedure TX11Console.Clear(const AColor: TPTCColor;
                            const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Clear(const AColor: TPTCColor;
                            const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Clear(const AColor: TPTCColor;
                            const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Clear(const AColor: TPTCColor;
                            const AArea: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  FX11Display.Clear(AColor, AArea);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11Console.Palette(APalette: IPTCPalette);
=======
procedure TX11Console.Palette(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Palette(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Palette(const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Palette(const APalette: TPTCPalette);
>>>>>>> origin/cpstrnew
begin
  FX11Display.Palette(APalette);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11Console.Palette: IPTCPalette;
=======
function TX11Console.Palette: TPTCPalette;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.Palette: TPTCPalette;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.Palette: TPTCPalette;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.Palette: TPTCPalette;
>>>>>>> origin/cpstrnew
begin
  Result := FX11Display.Palette;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TX11Console.Clip(AArea: IPTCArea);
=======
procedure TX11Console.Clip(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Clip(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Clip(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TX11Console.Clip(const AArea: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  FX11Display.Clip(AArea);
end;

function TX11Console.GetWidth: Integer;
begin
  Result := FX11Display.Width;
end;

function TX11Console.GetHeight: Integer;
begin
  Result := FX11Display.Height;
end;

function TX11Console.GetPitch: Integer;
begin
  Result := FX11Display.Pitch;
end;

function TX11Console.GetPages: Integer;
begin
  Result := 2;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11Console.GetArea: IPTCArea;
=======
function TX11Console.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.GetArea: TPTCArea;
>>>>>>> origin/cpstrnew
begin
  Result := FX11Display.Area;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11Console.Clip: IPTCArea;
=======
function TX11Console.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.Clip: TPTCArea;
>>>>>>> origin/cpstrnew
begin
  Result := FX11Display.Clip;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TX11Console.GetFormat: IPTCFormat;
=======
function TX11Console.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TX11Console.GetFormat: TPTCFormat;
>>>>>>> origin/cpstrnew
begin
  Result := FX11Display.Format;
end;

function TX11Console.GetName: string;
begin
  Result := 'X11';
end;

function TX11Console.GetTitle: string;
begin
  Result := FTitle;
end;

function TX11Console.GetInformation: string;
begin
  if FX11Display = nil then
    exit('PTC X11');
  Result := 'PTC X11, ';
  if FX11Display.IsFullScreen then
    Result := Result + 'fullscreen '
  else
    Result := Result + 'windowed ';

  { TODO: use virtual methods, instead of "is" }
  if FX11Display Is TX11WindowDisplay then
  begin
    if TX11WindowDisplay(FX11Display).FPrimary <> nil then
      Result := Result + '(' + TX11WindowDisplay(FX11Display).FPrimary.Name + ') '
    else
      Result := Result + '';
  end
  else
  begin
    {$IFDEF ENABLE_X11_EXTENSION_XF86DGA2}
    if FX11Display Is TX11DGA2Display then
      Result := Result + '(DGA) '
    else
    {$ENDIF ENABLE_X11_EXTENSION_XF86DGA2}
    {$IFDEF ENABLE_X11_EXTENSION_XF86DGA1}
    if FX11Display Is TX11DGA1Display then
      Result := Result + '(DGA) '
    else
    {$ENDIF ENABLE_X11_EXTENSION_XF86DGA1}
    begin
      {...}
    end;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
  {$IFDEF ENABLE_X11_EXTENSION_XF86DGA2}
  If PTC_X11_TRY_DGA In FFlags Then
  Begin
    Try
      Result := TX11DGA2Display.Create(display, screen, FFlags + [PTC_X11_LEAVE_DISPLAY]);
      Result.SetFlags(FFlags);
      Exit;
    Except
      LOG('DGA 2.0 failed');
    End;
  End;
  {$ENDIF ENABLE_X11_EXTENSION_XF86DGA2}

  {$IFDEF ENABLE_X11_EXTENSION_XF86DGA1}
  If PTC_X11_TRY_DGA In FFlags Then
  Begin
    Try
      Result := TX11DGA1Display.Create(display, screen, FFlags + [PTC_X11_LEAVE_DISPLAY]);
      Result.SetFlags(FFlags);
    Except
      LOG('DGA 1.0 failed');
    End;
  End;
  {$ENDIF ENABLE_X11_EXTENSION_XF86DGA1}

  Result := TX11WindowDisplay.Create(display, screen, FFlags);
End;

Procedure TX11Console.Open(Const ATitle : String; AWidth, AHeight : Integer;
                           Const AFormat : TPTCFormat; APages : Integer = 0);

Begin
  Close;
  FTitle := ATitle;

  If FX11Display = Nil Then
    FX11Display := CreateDisplay;
  FX11Display.Open(ATitle, AWidth, AHeight, AFormat);

  UpdateCursor;
End;

Procedure TX11Console.Close;

Begin
  FreeAndNil(FX11Display);
End;

Procedure TX11Console.Flush;

Begin
  Update;
End;

Procedure TX11Console.Finish;

Begin
  Update;
End;

Procedure TX11Console.Update;

Begin
  FX11Display.Update;
End;

Procedure TX11Console.Update(Const AArea : TPTCArea);

Begin
  FX11Display.Update(AArea);
End;

Function TX11Console.NextEvent(Var AEvent : TPTCEvent; AWait : Boolean; Const AEventMask : TPTCEventMask) : Boolean;

Begin
  Result := FX11Display.NextEvent(AEvent, AWait, AEventMask);
End;

Function TX11Console.PeekEvent(AWait : Boolean; Const AEventMask : TPTCEventMask) : TPTCEvent;

Begin
  Result := FX11Display.PeekEvent(AWait, AEventMask);
End;

Procedure TX11Console.Copy(Var ASurface : TPTCBaseSurface);

Begin
  {todo!...}
End;

Procedure TX11Console.Copy(Var ASurface : TPTCBaseSurface;
                           Const ASource, ADestination : TPTCArea);

Begin
  {todo!...}
End;

Function TX11Console.Lock : Pointer;

Begin
  Result := FX11Display.Lock;
End;

Procedure TX11Console.Unlock;

Begin
  FX11Display.Unlock;
End;

Procedure TX11Console.Load(Const APixels : Pointer;
                           AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat;
                           Const APalette : TPTCPalette);

Begin
  FX11Display.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette);
End;

Procedure TX11Console.Load(Const APixels : Pointer;
                           AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat;
                           Const APalette : TPTCPalette;
                           Const ASource, ADestination : TPTCArea);

Begin
  FX11Display.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette, ASource, ADestination);
End;

Procedure TX11Console.Save(APixels : Pointer;
                           AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat;
                           Const APalette : TPTCPalette);

Begin
  {todo!...}
End;

Procedure TX11Console.Save(APixels : Pointer;
                           AWidth, AHeight, APitch : Integer;
                           Const AFormat : TPTCFormat;
                           Const APalette : TPTCPalette;
                           Const ASource, ADestination : TPTCArea);

Begin
  {todo!...}
End;

Procedure TX11Console.Clear;

Var
  tmp : TPTCColor;

Begin
  If Format.Direct Then
    tmp := TPTCColor.Create(0, 0, 0, 0)
  Else
    tmp := TPTCColor.Create(0);
  Try
    Clear(tmp);
  Finally
    tmp.Free;
  End;
End;

Procedure TX11Console.Clear(Const AColor : TPTCColor);

Begin
  FX11Display.Clear(AColor);
End;

Procedure TX11Console.Clear(Const AColor : TPTCColor;
                            Const AArea : TPTCArea);

Begin
  FX11Display.Clear(AColor, AArea);
End;

Procedure TX11Console.Palette(Const APalette : TPTCPalette);

Begin
  FX11Display.Palette(APalette);
End;

Function TX11Console.Palette : TPTCPalette;

Begin
  Result := FX11Display.Palette;
End;

Procedure TX11Console.Clip(Const AArea : TPTCArea);

Begin
  FX11Display.Clip(AArea);
End;

Function TX11Console.GetWidth : Integer;

Begin
  Result := FX11Display.Width;
End;

Function TX11Console.GetHeight : Integer;

Begin
  Result := FX11Display.Height;
End;

Function TX11Console.GetPitch : Integer;

Begin
  Result := FX11Display.Pitch;
End;

Function TX11Console.GetPages : Integer;

Begin
  Result := 2;
End;

Function TX11Console.GetArea : TPTCArea;

Begin
  Result := FX11Display.Area;
End;

Function TX11Console.Clip : TPTCArea;

Begin
  Result := FX11Display.Clip;
End;

Function TX11Console.GetFormat : TPTCFormat;

Begin
  Result := FX11Display.Format;
End;

Function TX11Console.GetName : String;

Begin
  Result := 'X11';
End;

Function TX11Console.GetTitle : String;

Begin
  Result := FTitle;
End;

Function TX11Console.GetInformation : String;

Begin
  If FX11Display = Nil Then
    Exit('PTC X11');
  Result := 'PTC X11, ';
  If FX11Display.IsFullScreen Then
    Result := Result + 'fullscreen '
  Else
    Result := Result + 'windowed ';

  { TODO: use virtual methods, instead of "is" }
  If FX11Display Is TX11WindowDisplay Then
  Begin
    If TX11WindowDisplay(FX11Display).FPrimary <> Nil Then
      Result := Result + '(' + TX11WindowDisplay(FX11Display).FPrimary.Name + ') '
    Else
      Result := Result + '';
  End
  Else
  Begin
    {$IFDEF ENABLE_X11_EXTENSION_XF86DGA2}
    If FX11Display Is TX11DGA2Display Then
      Result := Result + '(DGA) '
    Else
    {$ENDIF ENABLE_X11_EXTENSION_XF86DGA2}
    {$IFDEF ENABLE_X11_EXTENSION_XF86DGA1}
    If FX11Display Is TX11DGA1Display Then
      Result := Result + '(DGA) '
    Else
    {$ENDIF ENABLE_X11_EXTENSION_XF86DGA1}
    Begin
      {...}
    End;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Result := Result + 'mode, ' +
            IntToStr(FX11Display.Width) + 'x' +
            IntToStr(FX11Display.Height) + ', ' +
            IntToStr(FX11Display.Format.Bits) + ' bit';
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TX11Console.UpdateCursor;
begin
  if Assigned(FX11Display) then
  begin
    if FX11Display.IsFullScreen then
      FX11Display.SetCursor(PTC_X11_FULLSCREEN_CURSOR_VISIBLE in FFlags)
    else
      FX11Display.SetCursor(not (PTC_X11_WINDOWED_CURSOR_INVISIBLE in FFlags));
  end;
end;

procedure TX11Console.UpdateMouseGrab;
begin
  if Assigned(FX11Display) then
    FX11Display.SetMouseGrab(PTC_X11_GRAB_MOUSE in FFlags);
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TX11Console.OpenGL_SwapBuffers;
begin
  FX11Display.OpenGL_SwapBuffers;
end;

procedure TX11Console.OpenGL_SetSwapInterval(AInterval: Integer);
begin
  FX11Display.OpenGL_SetSwapInterval(AInterval);
end;

function TX11Console.OpenGL_GetSwapInterval: Integer;
begin
  Result := FX11Display.OpenGL_GetSwapInterval;
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
End;

Procedure TX11Console.UpdateCursor;

Begin
  If Assigned(FX11Display) Then
  Begin
    If FX11Display.IsFullScreen Then
      FX11Display.SetCursor(PTC_X11_FULLSCREEN_CURSOR_VISIBLE In FFlags)
    Else
      FX11Display.SetCursor(Not (PTC_X11_WINDOWED_CURSOR_INVISIBLE In FFlags));
  End;
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
