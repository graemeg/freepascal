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

Constructor TDirectXDisplay.Create;

Begin
  m_information := '';
  m_mode := Nil;
  m_cursorsaved := False;
  m_open := False;
  m_fullscreen := False;
  m_ddraw := Nil;
  m_window := 0;
//  m_foreground := 0;
  FillChar(m_modes, SizeOf(m_modes), 0);
  FillChar(m_resolutions, SizeOf(m_resolutions), 0);
  m_mode := TPTCMode.Create;
End;

Destructor TDirectXDisplay.Destroy;

Begin
  close;
  m_mode.Free;
  internal_dispose_modes;
  internal_dispose_resolutions;
  Inherited Destroy;
End;

Procedure TDirectXDisplay.internal_dispose_modes;

Var
  i : Integer;

Begin
  For i := Low(m_modes) To High(m_modes) Do
    FreeAndNil(m_modes[i]);
End;

Procedure TDirectXDisplay.internal_dispose_resolutions;

Var
  i : Integer;

Begin
  For i := Low(m_resolutions) To High(m_resolutions) Do
    FreeAndNil(m_resolutions[i]);
End;

Function TDirectXDisplay_callback(descriptor : LPDDSURFACEDESC; Context : Pointer) : HRESULT; StdCall;

Var
  display : TDirectXDisplay;
  tmp : TPTCFormat;

Begin
  If (descriptor = Nil) Or (Context = Nil) Then
  Begin
    TDirectXDisplay_callback := DDENUMRET_CANCEL;
    Exit;
  End;
  display := TDirectXDisplay(Context);
  If ((descriptor^.dwFlags And DDSD_WIDTH) <> 0) And
     ((descriptor^.dwFlags And DDSD_HEIGHT) <> 0) And
     ((descriptor^.dwFlags And DDSD_PIXELFORMAT) <> 0) Then
  Begin
    tmp := DirectXTranslate(descriptor^.ddpfPixelFormat);
    Try
      FreeAndNil(display.m_modes[display.m_modes_count]);
      display.m_modes[display.m_modes_count] :=
        TPTCMode.Create(descriptor^.dwWidth, descriptor^.dwHeight, tmp);
    Finally
      tmp.Free;
    End;
    Inc(display.m_modes_count);
  End
  Else
  Begin
    LOG('EnumDisplayModesCallback was not given enough mode information');
  End;
  TDirectXDisplay_callback := DDENUMRET_OK;
End;

Procedure TDirectXDisplay.setup(ddraw : LPDIRECTDRAW2);

Var
  version : OSVERSIONINFO;
  match, found : Boolean;
  i, j : Integer;
  temp : TPTCMode;
  temp2 : TPTCFormat;
  S, S2 : String;

Begin
  LOG('setting up display lpDD2');
  m_ddraw := ddraw;
  m_information := 'windows version x.xx.x' + #13 + #10 + 'ddraw version x.xx' + #13 + #10 + 'display driver name xxxxx' +
    #13 + #10 + 'display driver vendor xxxxx' + #13 + #10 + 'certified driver? x' + #13 + #10;
  m_modes_count := 0;
  DirectXCheck(m_ddraw^.lpVtbl^.EnumDisplayModes(m_ddraw, 0, Nil, {this}Self, LPDDENUMMODESCALLBACK(@TDirectXDisplay_callback)));
  version.dwOSVersionInfoSize := SizeOf(version);
  If GetVersionEx(version) Then
  Begin
    If version.dwPlatformId = VER_PLATFORM_WIN32_WINDOWS Then
    Begin
      LOG('detected windows 95/98');
      temp2 := TPTCFormat.Create(8);
      Try
        found := False;
        For i := 0 To m_modes_count - 1 Do
          If (m_modes[i].width = 320) And (m_modes[i].height = 200) And
              m_modes[i].format.Equals(temp2) Then
            found := True;
        If Not found Then
        Begin
          LOG('adding 320x200x8 to mode list');
          FreeAndNil(m_modes[m_modes_count]);
          m_modes[m_modes_count] := TPTCMode.Create(320, 200, temp2);
          Inc(m_modes_count);
        End;
        found := False;
        For i := 0 To m_modes_count - 1 Do
          If (m_modes[i].width = 320) And (m_modes[i].height = 240) And
              m_modes[i].format.Equals(temp2) Then
            found := True;
        If Not found Then
        Begin
          LOG('adding 320x240x8 to mode list');
          FreeAndNil(m_modes[m_modes_count]);
          m_modes[m_modes_count] := TPTCMode.Create(320, 240, temp2);
          Inc(m_modes_count);
        End;
      Finally
        temp2.Free;
      End;
    End
    Else
      If version.dwPlatformId = VER_PLATFORM_WIN32_NT Then
      Begin
        LOG('detected windows nt');
      End;
  End;
  LOG('number of display modes', m_modes_count);
  FreeAndNil(m_modes[m_modes_count]);
  m_modes[m_modes_count] := TPTCMode.Create;
  m_resolutions_count := 0;
  For i := 0 To m_modes_count - 1 Do
  Begin
    match := False;
    For j := 0 To m_resolutions_count - 1 Do
      If (m_modes[i].width = m_resolutions[j].width) And
         (m_modes[i].height = m_resolutions[j].height) Then
      Begin
        match := True;
        Break;
      End;
    If Not match Then
    Begin
      FreeAndNil(m_resolutions[m_resolutions_count]);
      m_resolutions[m_resolutions_count] := TPTCMode.Create(m_modes[i]);
      Inc(m_resolutions_count);
    End;
  End;
  FreeAndNil(m_resolutions[m_resolutions_count]);
  m_resolutions[m_resolutions_count] := TPTCMode.Create;

  { kludge sort... :) }
  For i := 0 To m_resolutions_count - 1 Do
    For j := i + 1 To m_resolutions_count - 1 Do
      If (m_resolutions[i].width > m_resolutions[j].width) Or
         (m_resolutions[i].height > m_resolutions[j].height) Then
      Begin
        temp := m_resolutions[i];
        m_resolutions[i] := m_resolutions[j];
        m_resolutions[j] := temp;
      End;
  LOG('number of unique resolutions', m_resolutions_count);
  For i := 0 To m_resolutions_count - 1 Do
  Begin
    Str(m_resolutions[i].width, S);
    Str(m_resolutions[i].height, S2);
    LOG(S + ' x ' + S2);
  End;
End;

Function TDirectXDisplay.modes : PPTCMode;

Begin
  modes := @m_modes;
End;

Function TDirectXDisplay.test(Const _mode : TPTCMode; exact : Boolean) : Boolean;

Var
  i : Integer;

Begin
  If m_modes_count = 0 Then
  Begin
    LOG('mode test success with empty mode list');
    test := True;
    Exit;
  End;
  If exact Then
  Begin
    For i := 0 To m_modes_count - 1 Do
      If m_modes[i].Equals(_mode) Then
      Begin
        LOG('test exact mode success');
        test := True;
        Exit;
      End;
    LOG('test exact mode failure');
    test := False;
  End
  Else
  Begin
    For i := 0 To m_resolutions_count - 1 Do
      If (_mode.width <= m_resolutions[i].width) And
         (_mode.height <= m_resolutions[i].height) Then
      Begin
        LOG('test nearest mode success');
        test := True;
        Exit;
      End;
    LOG('test nearest mode failure');
    test := False;
  End;
End;

Procedure TDirectXDisplay.cooperative(window : HWND; _fullscreen : Boolean);

Begin
  If _fullscreen Then
  Begin
    LOG('entering exclusive mode');
    DirectXCheck(m_ddraw^.lpVtbl^.SetCooperativeLevel(m_ddraw, window, DDSCL_EXCLUSIVE Or DDSCL_FULLSCREEN Or DDSCL_ALLOWMODEX));
  End
  Else
  Begin
    LOG('entering normal cooperative mode');
    DirectXCheck(m_ddraw^.lpVtbl^.SetCooperativeLevel(m_ddraw, window, DDSCL_NORMAL));
  End;
  m_window := window;
  m_fullscreen := _fullscreen;
End;

Procedure TDirectXDisplay.open;

Begin
  FreeAndNil(m_mode);
  m_mode := TPTCMode.Create;
  m_open := True;
  LOG('opening windowed display');
End;

Procedure TDirectXDisplay.open(Const _mode : TPTCMode; exact : Boolean; frequency : Integer);

Begin
  If exact Then
  Begin
    LOG('opening exact fullscreen display mode');
    internal_open(_mode, True, frequency);
  End
  Else
  Begin
    LOG('opening nearest fullscreen mode');
    internal_open_nearest(_mode, False, frequency);
  End;
  LOG('successfully opened fullscreen display mode');
End;

Procedure TDirectXDisplay.close;

Begin
  If m_open And (m_ddraw <> Nil) Then
  Begin
    LOG('closing display');
    If m_fullscreen Then
    Begin
      LOG('restoring display mode');
      m_ddraw^.lpVtbl^.RestoreDisplayMode(m_ddraw);
      LOG('leaving exclusive mode');
      m_ddraw^.lpVtbl^.SetCooperativeLevel(m_ddraw, m_window, DDSCL_NORMAL);
    End;
  End;
  m_open := False;
  m_fullscreen := False;
End;

Procedure TDirectXDisplay.save;

Var
  p : POINT;

Begin
  LOG('saving desktop');

  m_cursorsaved := GetCursorPos(p);
  m_cursorX := p.x;
  m_cursorY := p.y;

{  m_foreground := GetForegroundWindow;
  GetWindowRect(m_foreground, m_foreground_rect);
  m_foreground_placement.length := SizeOf(WINDOWPLACEMENT);
  GetWindowPlacement(m_foreground, m_foreground_placement);}
End;

Procedure TDirectXDisplay.restore;

Begin
{  If (m_foreground <> 0) And IsWindow(m_foreground) And (m_foreground <> m_window) Then
  Begin}
  LOG('restoring desktop');
  If IsWindow(m_window) And m_fullscreen Then
    SetWindowPos(m_window, HWND_BOTTOM, 0, 0, 0, 0, SWP_NOMOVE Or SWP_NOSIZE Or SWP_NOACTIVATE);
  If m_cursorsaved Then
  Begin
    SetCursorPos(m_cursorX, m_cursorY);
    m_cursorsaved := False;
  End;
{    SetForegroundWindow(m_foreground);
    SetWindowPlacement(m_foreground, m_foreground_placement);
    SetWindowPos(m_foreground, HWND_TOP, m_foreground_rect.left, m_foreground_rect.top, m_foreground_rect.right - m_foreground_rect.left, m_foreground_rect.bottom - m_foreground_rect.top, SWP_FRAMECHANGED Or SWP_NOCOPYBITS);
    m_foreground := 0;
  End;}
End;

Function TDirectXDisplay.mode : TPTCMode;

Begin
  mode := m_mode;
End;

Function TDirectXDisplay.fullscreen : Boolean;

Begin
  fullscreen := m_fullscreen;
End;

Function TDirectXDisplay.information : String;

Begin
  information := m_information;
End;

Procedure TDirectXDisplay.internal_open(Const _mode : TPTCMode; exact : Boolean; frequency : Integer);

Begin
  LOG('internal display open');
  LOG('mode width', _mode.width);
  LOG('mode height', _mode.height);
  LOG('mode format', _mode.format);
  LOG('mode frequency', frequency);
  If exact Then
  Begin
    LOG('setting exact mode');
    DirectXCheck(m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, _mode.format.bits, frequency, 0));
  End
  Else
    Case _mode.format.bits Of
      32 : Begin
        LOG('setting virtual 32 mode');
        If m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 32, frequency, 0) <> DD_OK Then
          If m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 24, frequency, 0) <> DD_OK Then
            DirectXCheck(m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 16, frequency, 0));
      End;
      24 : Begin
        LOG('setting virtual 24 mode');
        If m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 24, frequency, 0) <> DD_OK Then
          If m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 32, frequency, 0) <> DD_OK Then
            DirectXCheck(m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 16, frequency, 0));
      End;
      16 : Begin
        LOG('setting virtual 16 mode');
        If m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 16, frequency, 0) <> DD_OK Then
          If m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 24, frequency, 0) <> DD_OK Then
            DirectXCheck(m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 32, frequency, 0));
      End;
      8 : Begin
        LOG('setting virtual 8 mode');
        If m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 8, frequency, 0) <> DD_OK Then
          If m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 24, frequency, 0) <> DD_OK Then {yes, 24bit is now faster than 32bit!}
            If m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 32, frequency, 0) <> DD_OK Then
              DirectXCheck(m_ddraw^.lpVtbl^.SetDisplayMode(m_ddraw, _mode.width, _mode.height, 16, frequency, 0));
      End;
      Else
        Raise TPTCError.Create('unsupported pixel format');
    End;
  LOG('internal display open success');
  FreeAndNil(m_mode);
  m_mode := TPTCMode.Create(_mode);
  m_open := True;
End;

Procedure TDirectXDisplay.internal_open_nearest(Const _mode : TPTCMode; exact : Boolean; frequency : Integer);

Var
  index : Integer;
  match, match_exact : TPTCMode;
  tmp : TPTCMode;
  i : Integer;
  width, height : Integer;
  dx1, dy1, dx2, dy2 : Integer;

Begin
  If m_resolutions_count <> 0 Then
  Begin
    LOG('searching for nearest mode in resolutions list');
    index := 0;
    match_exact := Nil;
    match := TPTCMode.Create;
    Try
      match_exact := TPTCMode.Create;
      For i := 0 To m_resolutions_count - 1 Do
      Begin
        width := m_resolutions[i].width;
        height := m_resolutions[i].height;
        If (_mode.width <= width) And (_mode.height <= height) Then
        Begin
          If (width = _mode.width) And (height = _mode.height) Then
          Begin
            LOG('found an exact match');
            tmp := TPTCMode.Create(width, height, _mode.format);
            Try
              match_exact.ASSign(tmp);
            Finally
              tmp.Free;
            End;
          End;
          If match.valid Then
          Begin
            dx1 := match.width - _mode.width;
            dy1 := match.height - _mode.height;
            dx2 := width - _mode.width;
            dy2 := height - _mode.height;
            If (dx2 <= dx1) And (dy2 <= dy1) Then
            Begin
              LOG('found a better nearest match');
              tmp := TPTCMode.Create(width, height, _mode.format);
              Try
                match.ASSign(tmp);
              Finally
                tmp.Free;
              End;
            End;
          End
          Else
          Begin
            LOG('found first nearest match');
            tmp := TPTCMode.Create(width, height, _mode.format);
            Try
              match.ASSign(tmp);
            Finally
              tmp.Free;
            End;
          End;
        End
        Else
        Begin
{          LOG('stopping nearest mode search');
          Break;}
        End;
      End;

      If match_exact.valid Then
      Try
        LOG('trying an exact match');
        internal_open(match_exact, exact, frequency);
        Exit;
      Except
        On TPTCError Do;
      End;
      If match.valid Then
      Try
        LOG('trying nearest match');
        internal_open(match, exact, frequency);
        Exit;
      Except
        On TPTCError Do;
      End;
    Finally
      match.Free;
      match_exact.Free;
    End;
  End
  Else
  Begin
    LOG('no resolutions list for nearest mode search');
  End;
{  match.Free;
  match_exact.Free;}
  LOG('could not find a nearest match in first pass');
  Try
    LOG('trying requested mode first');
    internal_open(_mode, exact, frequency);
    Exit;
  Except
    On TPTCError Do
    Begin
      LOG('falling back to nearest standard mode');
      If (_mode.width <= 320) And (_mode.height <= 200) Then
      Try
        tmp := TPTCMode.Create(320, 200, _mode.format);
        Try
          internal_open(tmp, exact, frequency);
        Finally
          tmp.Free;
        End;
        Exit;
      Except
        On TPTCError Do;
      End;
      If (_mode.width <= 320) And (_mode.height <= 240) Then
      Try
        tmp := TPTCMode.Create(320, 240, _mode.format);
        Try
          internal_open(tmp, exact, frequency);
        Finally
          tmp.Free;
        End;
        Exit;
      Except
        On TPTCError Do;
      End;
      If (_mode.width <= 512) And (_mode.height <= 384) Then
      Try
        tmp := TPTCMode.Create(512, 384, _mode.format);
        Try
          internal_open(tmp, exact, frequency);
        Finally
          tmp.Free;
        End;
        Exit;
      Except
        On TPTCError Do;
      End;
      If (_mode.width <= 640) And (_mode.height <= 400) Then
      Try
        tmp := TPTCMode.Create(640, 400, _mode.format);
        Try
          internal_open(tmp, exact, frequency);
        Finally
          tmp.Free;
        End;
        Exit;
      Except
        On TPTCError Do;
      End;
      If (_mode.width <= 640) And (_mode.height <= 480) Then
      Try
        tmp := TPTCMode.Create(640, 480, _mode.format);
        Try
          internal_open(tmp, exact, frequency);
        Finally
          tmp.Free;
        End;
        Exit;
      Except
        On TPTCError Do;
      End;
      If (_mode.width <= 800) And (_mode.height <= 600) Then
      Try
        tmp := TPTCMode.Create(800, 600, _mode.format);
        Try
          internal_open(tmp, exact, frequency);
        Finally
          tmp.Free;
        End;
        Exit;
      Except
        On TPTCError Do;
      End;
      If (_mode.width <= 1024) And (_mode.height <= 768) Then
      Try
        tmp := TPTCMode.Create(1024, 768, _mode.format);
        Try
          internal_open(tmp, exact, frequency);
        Finally
          tmp.Free;
        End;
        Exit;
      Except
        On TPTCError Do;
      End;
      If (_mode.width <= 1280) And (_mode.height <= 1024) Then
      Try
        tmp := TPTCMode.Create(1280, 1024, _mode.format);
        Try
          internal_open(tmp, exact, frequency);
        Finally
          tmp.Free;
        End;
        Exit;
      Except
        On TPTCError Do;
      End;
      If (_mode.width <= 1600) And (_mode.height <= 1200) Then
      Try
        tmp := TPTCMode.Create(1600, 1200, _mode.format);
        Try
          internal_open(tmp, exact, frequency);
        Finally
          tmp.Free;
        End;
        Exit;
      Except
        On TPTCError Do;
      End;
    End;
  End;
  Raise TPTCError.Create('could not set display mode');
End;
