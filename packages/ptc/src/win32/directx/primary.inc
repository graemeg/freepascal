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

Constructor TDirectXPrimary.Create;

Begin
  m_area := Nil;
  m_clip := Nil;
  m_format := Nil;
  m_clear := Nil;
  m_palette := Nil;
  m_area := TPTCArea.Create;
  m_clip := TPTCArea.Create;
  m_format := TPTCFormat.Create;
  m_clear := TPTCClear.Create;
  m_palette := TPTCPalette.Create;

  m_locked := Nil;
  m_window := Nil;
  m_width := 0;
  m_height := 0;
  m_back := Nil;
  m_front := Nil;
  m_pages := 0;
  m_lpDD2 := Nil;
  m_lpDDC := Nil;
  m_lpDDS_primary := Nil;
  m_lpDDS_primary_back := Nil;
  m_lpDDS_secondary := Nil;
  m_active := True;
  m_blocking := True;
  m_centering := True;
  m_synchronize := True;
  m_fullscreen := False;
  m_primary_width := 0;
  m_primary_height := 0;
  m_secondary_width := 0;
  m_secondary_height := 0;
  FillChar(m_lpDDS_primary_page, SizeOf(m_lpDDS_primary_page), 0);
End;

Destructor TDirectXPrimary.Destroy;

Begin
  { close }
  close;
  m_area.Free;
  m_clip.Free;
  m_format.Free;
  m_clear.Free;
  m_palette.Free;
  Inherited Destroy;
End;

Procedure TDirectXPrimary.initialize(window : TWin32Window; lpDD2 : LPDIRECTDRAW2);

Begin
  LOG('initializing primary surface');
  close;
  m_window := window;
  m_lpDD2 := lpDD2;
End;

Procedure TDirectXPrimary.primary(_pages : Integer; video, fullscreen, _palette, complex : Boolean);

Var
  attach_primary_pages : Boolean;
  descriptor : DDSURFACEDESC;
  ddpf : DDPIXELFORMAT;
  capabilities : DDSCAPS;
  tmp : TPTCPalette;
  i : Integer;
  rectangle : RECT;

Begin
  Try
    LOG('creating primary surface');
    LOG('pages', _pages);
    LOG('video', video);
    LOG('fullscreen', fullscreen);
    LOG('palette', _palette);
    LOG('complex', complex);
    If _pages <= 0 Then
      Raise TPTCError.Create('invalid number of pages');
    m_fullscreen := fullscreen;
    attach_primary_pages := False;
    If complex Or (Not _palette) Or (_pages = 1) Then
    Begin
      LOG('creating a complex primary flipping surface');
      FillChar(descriptor, SizeOf(descriptor), 0);
      descriptor.dwSize := SizeOf(descriptor);
      descriptor.dwFlags := DDSD_CAPS;
      If _pages > 1 Then
	descriptor.dwFlags := descriptor.dwFlags Or DDSD_BACKBUFFERCOUNT;
      descriptor.dwBackBufferCount := _pages - 1;
      descriptor.ddsCaps.dwCaps := DDSCAPS_PRIMARYSURFACE;
      If video Then
	descriptor.ddsCaps.dwCaps := descriptor.ddsCaps.dwCaps Or DDSCAPS_VIDEOMEMORY;
      If _pages > 1 Then
	descriptor.ddsCaps.dwCaps := descriptor.ddsCaps.dwCaps Or DDSCAPS_COMPLEX Or DDSCAPS_FLIP;
      DirectXCheck(m_lpDD2^.lpVtbl^.CreateSurface(m_lpDD2, @descriptor, @m_lpDDS_primary, Nil), 'm_lpDD2^.CreateSurface failed in TDirectXPrimary.primary');
    End
    Else
    Begin
      LOG('creating a simple primary surface');
      FillChar(descriptor, SizeOf(descriptor), 0);
      descriptor.dwSize := SizeOf(descriptor);
      descriptor.dwFlags := DDSD_CAPS;
      descriptor.ddsCaps.dwCaps := DDSCAPS_PRIMARYSURFACE;
      If video Then
	descriptor.ddsCaps.dwCaps := descriptor.ddsCaps.dwCaps Or DDSCAPS_VIDEOMEMORY;
      DirectXCheck(m_lpDD2^.lpVtbl^.CreateSurface(m_lpDD2, @descriptor, @m_lpDDS_primary, Nil), 'm_lpDD2^.CreateSurface failed in TDirectXPrimary.primary (palette)');
      attach_primary_pages := True;
    End;
    FillChar(descriptor, SizeOf(descriptor), 0);
    descriptor.dwSize := SizeOf(descriptor);
    DirectXCheck(m_lpDDS_primary^.lpVtbl^.GetSurfaceDesc(m_lpDDS_primary, @descriptor), 'm_lpDDS_primary^.GetSurfaceDesc failed in TDirectXPrimary.primary');
    If (descriptor.ddsCaps.dwCaps And DDSCAPS_VIDEOMEMORY) <> 0 Then
    Begin
      LOG('primary surface is in video memory');
    End
    Else
    Begin
      LOG('primary surface is in system memory');
    End;
    FillChar(ddpf, SizeOf(ddpf), 0);
    ddpf.dwSize := SizeOf(ddpf);
    DirectXCheck(m_lpDDS_primary^.lpVtbl^.GetPixelFormat(m_lpDDS_primary, @ddpf), 'm_lpDDS_primary^.GetPixelFormat failed in TDirectXPrimary.primary');
    m_front := m_lpDDS_primary;
    m_pages := _pages;
    m_width := descriptor.dwWidth;
    m_height := descriptor.dwHeight;
    FreeAndNil(m_format);
    m_format := DirectXTranslate(ddpf);
    LOG('primary width', m_width);
    LOG('primary height', m_height);
    LOG('primary pages', m_pages);
    LOG('primary format', m_format);
    If _palette Then
    Begin
      LOG('clearing primary palette');
      tmp := TPTCPalette.Create;
      Try
	palette(tmp);
      Finally
	tmp.Free;
      End;
    End;
    If attach_primary_pages Then
    Begin
      If (_pages - 1) > High(m_lpDDS_primary_page) Then
	Raise TPTCError.Create('too many primary pages');
      For i := 0 To _pages - 2 Do
      Begin
	LOG('creating primary page surface');
	FillChar(descriptor, SizeOf(descriptor), 0);
	descriptor.dwSize := SizeOf(descriptor);
	descriptor.dwFlags := DDSD_CAPS Or DDSD_WIDTH Or DDSD_HEIGHT;
	descriptor.dwWidth := m_width;
	descriptor.dwHeight := m_height;
	descriptor.ddsCaps.dwCaps := DDSCAPS_OFFSCREENPLAIN;
	If video Then
	  descriptor.ddsCaps.dwCaps := descriptor.ddsCaps.dwCaps Or DDSCAPS_VIDEOMEMORY;
	DirectXCheck(m_lpDD2^.lpVtbl^.CreateSurface(m_lpDD2, @descriptor, @m_lpDDS_primary_page[i], Nil), 'm_lpDD2^.CreateSurface failed in TDirectXPrimary.primary (primary page)');

	FillChar(descriptor, SizeOf(descriptor), 0);
	descriptor.dwSize := SizeOf(descriptor);
	DirectXCheck(m_lpDDS_primary_page[i]^.lpVtbl^.GetSurfaceDesc(m_lpDDS_primary_page[i], @descriptor), 'm_lpDDS_primary_page^.GetSurfaceDesc failed in TDirectXPrimary.primary');

	If (descriptor.ddsCaps.dwCaps And DDSCAPS_VIDEOMEMORY) <> 0 Then
	Begin
	  LOG('primary surface page is in video memory');
	End
	Else
	Begin
	  LOG('primary surface page is in system memory');
	End;
	LOG('attaching page to primary surface');
	DirectXCheck(m_lpDDS_primary^.lpVtbl^.AddAttachedSurface(m_lpDDS_primary, m_lpDDS_primary_page[i]), 'm_lpDDS_primary^.AddAttachedSurface failed in TDirectXPrimary.primary');
      End;
    End;
    m_primary_width := m_width;
    m_primary_height := m_height;
    If Not fullscreen Then
    Begin
      GetClientRect(m_window.handle, rectangle);
      m_width := rectangle.right;
      m_height := rectangle.bottom;
    End;
    FreeAndNil(m_area);
    m_area := TPTCArea.Create(0, 0, m_width, m_height);
    FreeAndNil(m_clip);
    m_clip := TPTCArea.Create(m_area);
    If _pages > 1 Then
    Begin
      capabilities.dwCaps := DDSCAPS_BACKBUFFER;
      DirectXCheck(m_front^.lpVtbl^.GetAttachedSurface(m_front, @capabilities, @m_lpDDS_primary_back), 'm_front^.GetAttachedSurface failed in TDirectXPrimary.primary');

      FillChar(descriptor, SizeOf(descriptor), 0);
      descriptor.dwSize := SizeOf(descriptor);
      DirectXCheck(m_lpDDS_primary_back^.lpVtbl^.GetSurfaceDesc(m_lpDDS_primary_back, @descriptor), 'm_lpDDS_primary_back^.GetSurfaceDesc failed in TDirectXPrimary.primary');

      If (descriptor.ddsCaps.dwCaps And DDSCAPS_VIDEOMEMORY) <> 0 Then
      Begin
	LOG('primary back surface is in video memory');
      End
      Else
      Begin
	LOG('primary back surface is in system memory');
      End;
    End
    Else
      m_lpDDS_primary_back := m_front;
    m_back := m_lpDDS_primary_back;
    If fullscreen Then
      While _pages > 0 Do
      Begin
	Dec(_pages);
	LOG('clearing primary page');
	clear;
	update;
      End;
  Except
    On error : TPTCError Do
    Begin
      If m_lpDDS_primary <> Nil Then
      Begin
	m_lpDDS_primary^.lpVtbl^.Release(m_lpDDS_primary);
	m_lpDDS_primary := Nil;
      End;
      Raise TPTCError.Create('could not create primary surface', error);
    End;
  End;
End;

Procedure TDirectXPrimary.secondary(_width, _height : Integer);

Var
  descriptor : DDSURFACEDESC;
  hel : DDCAPS;
  driver : DDCAPS;
  capabilities : DDSCAPS;

Begin
  LOG('creating secondary surface');
  LOG('width', _width);
  LOG('height', _height);
  FillChar(descriptor, SizeOf(descriptor), 0);
  descriptor.dwSize := SizeOf(descriptor);
  descriptor.dwFlags := DDSD_CAPS Or DDSD_HEIGHT Or DDSD_WIDTH;
  descriptor.ddsCaps.dwCaps := DDSCAPS_OFFSCREENPLAIN;
  descriptor.dwHeight := _height;
  descriptor.dwWidth := _width;
  DirectXCheck(m_lpDD2^.lpVtbl^.CreateSurface(m_lpDD2, @descriptor, @m_lpDDS_secondary, Nil), 'm_lpDD2^.CreateSurface failed in TDirectXPrimary.secondary');

  FillChar(descriptor, SizeOf(descriptor), 0);
  descriptor.dwSize := SizeOf(descriptor);
  DirectXCheck(m_lpDDS_secondary^.lpVtbl^.GetSurfaceDesc(m_lpDDS_secondary, @descriptor), 'm_lpDDS_secondary^.GetSurfaceDesc failed in TDirectXPrimary.secondary');

  If (descriptor.ddsCaps.dwCaps And DDSCAPS_VIDEOMEMORY) <> 0 Then
  Begin
    LOG('secondary surface is in video memory');
  End
  Else
  Begin
    LOG('secondary surface is in system memory');
  End;

  If Not m_fullscreen Then
  Begin
    LOG('attaching clipper to primary surface');
    DirectXCheck(m_lpDD2^.lpVtbl^.CreateClipper(m_lpDD2, 0, @m_lpDDC, Nil), 'm_lpDD2^.CreateClipper failed in TDirectXPrimary.secondary');
    DirectXCheck(m_lpDDC^.lpVtbl^.SetHWnd(m_lpDDC, 0, m_window.handle), 'm_lpDDC^.SetHWnd failed in TDirectXPrimary.secondary');
    DirectXCheck(m_lpDDS_primary^.lpVtbl^.SetClipper(m_lpDDS_primary, m_lpDDC), 'm_lpDDS_primary^.SetClipper failed in TDirectXPrimary.secondary');
  End;
  m_width := _width;
  m_height := _height;
  FreeAndNil(m_area);
  m_area := TPTCArea.Create(0, 0, m_width, m_height);
  FreeAndNil(m_clip);
  m_clip := TPTCArea.Create(m_area);
  m_secondary_width := m_width;
  m_secondary_height := m_height;
  m_back := m_lpDDS_secondary;

{  hel.dwSize := SizeOf(hel);
  driver.dwSize := SizeOf(driver);
  DirectXCheck(m_lpDD2^.GetCaps(@driver, @hel));}
  {
    auto stretching support is disabled below because in almost 100% of cases
    centering is faster and we must choose the fastest option by default!
  }
  {todo: DDCAPS!!!!!!!!!!!}
{  If ((driver.dwCaps And DDCAPS_BLTSTRETCH) <> 0) And
     ((driver.dwFXCaps And DDFXCAPS_BLTSTRETCHY) <> 0) Then
  Begin
    LOG('found hardware stretching support');
  End
  Else
  Begin
    LOG('no hardware stretching support');
  End;}

  m_lpDDS_secondary^.lpVtbl^.GetCaps(m_lpDDS_secondary, @capabilities);
  If (capabilities.dwCaps And DDSCAPS_SYSTEMMEMORY) <> 0 Then
  Begin
    LOG('secondary surface is in system memory');
  End;

  centering(True);

  LOG('clearing secondary page');

  clear;

  update;
End;

Procedure TDirectXPrimary.synchronize(_update : Boolean);

Begin
  m_synchronize := _update;
  If m_pages > 1 Then
    m_synchronize := False;
  LOG('primary synchronize', _update);
End;

Procedure TDirectXPrimary.centering(center : Boolean);

Begin
  m_centering := center;
  LOG('primary centering', m_centering);
End;

Procedure TDirectXPrimary.close;

Var
  i : Integer;
  lost : Boolean;
  tmp : TPTCPalette;

Begin
  Try
    LOG('closing primary surface');
    lost := False;
    If (m_lpDDS_primary <> Nil) And (m_lpDDS_primary^.lpVtbl^.IsLost(m_lpDDS_primary) <> DD_OK) Then
      lost := True;
    If (m_lpDDS_secondary <> Nil) And (m_lpDDS_secondary^.lpVtbl^.IsLost(m_lpDDS_secondary) <> DD_OK) Then
      lost := True;
    If (m_back <> Nil) And (m_lpDDS_primary <> Nil) And m_fullscreen And (Not lost) Then
    Begin
      tmp := TPTCPalette.Create;
      Try
	LOG('clearing primary palette');
	palette(tmp);
      Finally
	tmp.Free;
      End;
      LOG('clearing primary pages');
      For i := 0 To m_pages - 1 Do
      Begin
	clear;
	update;
      End;
    End;
  Except
    On TPTCError Do
    Begin
      LOG('primary close clearing failed');
    End;
  End;

  If m_lpDDC <> Nil Then
  Begin
    LOG('releasing clipper');
    m_lpDDC^.lpVtbl^.Release(m_lpDDC);
    m_lpDDC := Nil;
  End;
  If m_lpDDS_secondary <> Nil Then
  Begin
    LOG('releasing secondary surface');
    m_lpDDS_secondary^.lpVtbl^.Release(m_lpDDS_secondary);
    m_lpDDS_secondary := Nil;
  End;
  i := 0;
  While m_lpDDS_primary_page[i] <> Nil Do
  Begin
    LOG('releasing attached primary surface page');
    m_lpDDS_primary_page[i]^.lpVtbl^.Release(m_lpDDS_primary_page[i]);
    m_lpDDS_primary_page[i] := Nil;
    Inc(i);
  End;
  If m_lpDDS_primary <> Nil Then
  Begin
    LOG('releasing primary surface');
    m_lpDDS_primary^.lpVtbl^.Release(m_lpDDS_primary);
    m_lpDDS_primary := Nil;
  End;

  m_back := Nil;
  m_front := Nil;
  m_lpDDS_primary_back := Nil;
End;

Procedure TDirectXPrimary.update;

Begin
  block;
  paint;
  If m_pages > 1 Then
    DirectXCheck(m_front^.lpVtbl^.Flip(m_front, Nil, DDFLIP_WAIT), 'm_front^.Flip failed in TDirectXPrimary.update');
End;

Function TDirectXPrimary.lock : Pointer;

Var
  descriptor : DDSURFACEDESC;
  pnt : POINT;
  rct : RECT;

Begin
  block;
  descriptor.dwSize := SizeOf(descriptor);
  If m_fullscreen Or (m_back = m_lpDDS_secondary) Then
  Begin
    DirectXCheck(m_back^.lpVtbl^.Lock(m_back, Nil, @descriptor, DDLOCK_WAIT, 0), 'm_back^.Lock failed in TDirectXPrimary.lock');
    m_locked := descriptor.lpSurface;
  End
  Else
  Begin
    pnt.x := 0;
    pnt.y := 0;
    ClientToScreen(m_window.handle, pnt);
    rct.left := pnt.x;
    rct.top := pnt.y;
    rct.right := pnt.x + m_width;
    rct.bottom := pnt.y + m_height;
    DirectXCheck(m_back^.lpVtbl^.Lock(m_back, @rct, @descriptor, DDLOCK_WAIT, 0), 'm_back^.Lock(rect) failed in TDirectXPrimary.lock');
    m_locked := descriptor.lpSurface;
  End;
  lock := m_locked;
End;

Procedure TDirectXPrimary.unlock;

Begin
  block;
  DirectXCheck(m_back^.lpVtbl^.Unlock(m_back, m_locked), 'm_back^.Unlock failed in TDirectXPrimary.unlock');
End;

Procedure TDirectXPrimary.clear;

Var
  fx : DDBLTFX;
  tmp : TPTCColor;

Begin
  block;
  If m_fullscreen Or (m_back = m_lpDDS_secondary) Then
  Begin
    fx.dwSize := SizeOf(fx);
    fx.dwFillColor := 0;
    DirectXCheck(m_back^.lpVtbl^.Blt(m_back, Nil, Nil, Nil, DDBLT_COLORFILL Or DDBLT_WAIT, @fx), 'm_back^.Blt failed in TDirectXPrimary.clear');
  End
  Else
  Begin
    { todo: replace with hardware clear! }
    If format.direct Then
    Begin
      tmp := TPTCColor.Create(0, 0, 0, 0);
      Try
	clear(tmp, m_area);
      Finally
	tmp.Free;
      End;
    End
    Else
    Begin
      tmp := TPTCColor.Create(0);
      Try
	clear(tmp, m_area);
      Finally
	tmp.Free;
      End;
    End;
  End;
End;

Procedure TDirectXPrimary.clear(Const color : TPTCColor; Const _area : TPTCArea);

Var
  clipped, clipped_area : TPTCArea;
  clear_color : DWord;
  rct : RECT;
  fx : DDBLTFX;
  pixels : Pointer;


Begin
  block;
  If m_fullscreen Or (m_back = m_lpDDS_secondary) Then
  Begin
    clipped := TPTCClipper.clip(_area, m_clip);
    Try
      clear_color := pack(color, m_format);
      With rct Do
      Begin
	left := clipped.left;
	top := clipped.top;
	right := clipped.right;
	bottom := clipped.bottom;
      End;
      fx.dwSize := SizeOf(fx);
      fx.dwFillColor := clear_color;
      DirectXCheck(m_back^.lpVtbl^.Blt(m_back, @rct, Nil, Nil, DDBLT_COLORFILL Or DDBLT_WAIT, @fx), 'm_back^.Blt(rect) failed in TDirectXPrimary.clear');
    Finally
      clipped.Free;
    End;
  End
  Else
  Begin
    { todo: replace with accelerated clearing code! }
    pixels := lock;
    clipped_area := Nil;
    Try
      Try
	clipped_area := TPTCClipper.clip(_area, clip);
	m_clear.request(format);
	m_clear.clear(pixels, clipped_area.left, clipped_area.right, clipped_area.width, clipped_area.height, pitch, color);
	unlock;
      Except
	On error : TPTCError Do
	Begin
	  unlock;
	  Raise TPTCError.Create('failed to clear console area', error);
	End;
      End;
    Finally
      If clipped_area <> Nil Then
	clipped_area.Free;
    End;
  End;
End;

Procedure TDirectXPrimary.palette(Const _palette : TPTCPalette);

Var
  data : Pint32;
  temp : Array[0..255] Of PALETTEENTRY;
  i : Integer;
  lpDDP : LPDIRECTDRAWPALETTE;

Begin
  block;

  m_palette.load(_palette.data);
  If Not m_format.indexed Then
  Begin
    LOG('palette set in direct color');
  End
  Else
  Begin
    data := _palette.data;
    For i := 0 To 255 Do
    Begin
      temp[i].peRed := (data[i] And $00FF0000) Shr 16;
      temp[i].peGreen := (data[i] And $0000FF00) Shr 8;
      temp[i].peBlue := data[i] And $000000FF;
      temp[i].peFlags := 0;
    End;
    lpDDP := Nil;
    If m_lpDDS_primary^.lpVtbl^.GetPalette(m_lpDDS_primary, @lpDDP) <> DD_OK Then
    Begin
      DirectXCheck(m_lpDD2^.lpVtbl^.CreatePalette(m_lpDD2, DDPCAPS_8BIT Or DDPCAPS_ALLOW256 Or DDPCAPS_INITIALIZE, @temp, @lpDDP, Nil), 'm_lpDD2^.CreatePalette failed in TDirectXPrimary.palette');
      DirectXCheck(m_lpDDS_primary^.lpVtbl^.SetPalette(m_lpDDS_primary, lpDDP), 'm_lpDDS_primary^.SetPalette failed in TDirectXPrimary.palette');
    End
    Else
      DirectXCheck(lpDDP^.lpVtbl^.SetEntries(lpDDP, 0, 0, 256, @temp), 'lpDDP^.SetEntries failed in TDirectXPrimary.palette');
  End;
End;

Function TDirectXPrimary.palette : TPTCPalette;

Begin
  palette := m_palette;
End;

Procedure TDirectXPrimary.clip(Const _area : TPTCArea);

Var
  tmp : TPTCArea;

Begin
  tmp := TPTCClipper.clip(_area, m_area);
  Try
    m_clip.ASSign(tmp);
  Finally
    tmp.Free;
  End;
End;

Function TDirectXPrimary.width : Integer;

Begin
  width := m_width;
End;

Function TDirectXPrimary.height : Integer;

Begin
  height := m_height;
End;

Function TDirectXPrimary.pages : Integer;

Begin
  pages := m_pages;
End;

Function TDirectXPrimary.pitch : Integer;

Var
  descriptor : DDSURFACEDESC;

Begin
  Block;
  descriptor.dwSize := SizeOf(descriptor);
  DirectXCheck(m_back^.lpVtbl^.GetSurfaceDesc(m_back, @descriptor), 'm_back^.GetSurfaceDesc failed in TDirectXPrimary.pitch');
  pitch := descriptor.lPitch;
End;

Function TDirectXPrimary.area : TPTCArea;

Begin
  area := m_area;
End;

Function TDirectXPrimary.clip : TPTCArea;

Begin
  clip := m_clip;
End;

Function TDirectXPrimary.format : TPTCFormat;

Begin
  format := m_format;
End;

Function TDirectXPrimary.lpDDS : LPDIRECTDRAWSURFACE;

Begin
  If m_lpDDS_secondary <> Nil Then
    lpDDS := m_lpDDS_secondary
  Else
    lpDDS := m_lpDDS_primary_back;
End;

Function TDirectXPrimary.lpDDS_primary : LPDIRECTDRAWSURFACE;

Begin
  lpDDS_primary := m_lpDDS_primary;
End;

Function TDirectXPrimary.lpDDS_secondary : LPDIRECTDRAWSURFACE;

Begin
  lpDDS_secondary := m_lpDDS_secondary;
End;

Procedure TDirectXPrimary.activate;

Begin
  LOG('primary activated');
  m_active := True;
End;

Procedure TDirectXPrimary.deactivate;

Begin
  LOG('primary deactivated');
  If m_blocking Then
    m_active := False
  Else
    {no deactivation when not blocking};
End;

Function TDirectXPrimary.active : Boolean;

Begin
  active := m_active;
End;

Procedure TDirectXPrimary.block;

Var
  restored : Boolean;

Begin
  If Not m_blocking Then
    Exit;
  If Not active Then
  Begin
    restored := False;
    While Not restored Do
    Begin
      LOG('blocking until activated');
      While Not active Do
      Begin
	m_window.update(True);
	Sleep(10);
      End;
      LOG('primary is active');
      m_window.update(True);
      Try
	restore;
	restored := True;
	LOG('successful restore');
      Except
	On TPTCError Do
	Begin
	  LOG('application is active but cannot restore');
	End;
      End;
      Sleep(10);
    End;
  End;
  If m_lpDDS_primary^.lpVtbl^.IsLost(m_lpDDS_primary) <> DD_OK Then
    Raise TPTCError.Create('primary surface lost unexpectedly!');
  If (m_lpDDS_secondary <> Nil) And (m_lpDDS_secondary^.lpVtbl^.IsLost(m_lpDDS_secondary) <> DD_OK) Then
    Raise TPTCError.Create('secondary surface lost unexpectedly!');
End;

Procedure TDirectXPrimary.save;

Begin
  If m_lpDDS_primary^.lpVtbl^.IsLost(m_lpDDS_primary) = DD_OK Then
  Begin
    LOG('saving contents of primary surface');

    { todo: save contents of primary surface }
  End
  Else
  Begin
    LOG('could not save primary surface');
  End;

  If (m_lpDDS_secondary <> Nil) And (m_lpDDS_secondary^.lpVtbl^.IsLost(m_lpDDS_secondary) = DD_OK) Then
  Begin
    LOG('saving contents of secondary surface');

    { todo: save contents of secondary surface }
  End
  Else
    If m_lpDDS_secondary <> Nil Then
    Begin
      LOG('could not save secondary surface');
    End;
End;

Procedure TDirectXPrimary.restore;

Var
  i : Integer;
  rct : RECT;
  fx : DDBLTFX;

Begin
  DirectXCheck(m_lpDDS_primary^.lpVtbl^.Restore(m_lpDDS_primary), 'm_lpDDS_primary^.Restore failed in TDirectXConsole.restore');
  If m_lpDDS_secondary <> Nil Then
    DirectXCheck(m_lpDDS_secondary^.lpVtbl^.Restore(m_lpDDS_secondary), 'm_lpDDS_secondary^.Restore failed in TDirectXConsole.restore');
  LOG('restoring contents of primary surface');
  { todo: restore palette object on primary surface ? }
  { todo: restore contents of primary surface }
  If m_lpDDS_primary_page[0] <> Nil Then
  Begin
    LOG('restoring attached pages');
    For i := 0 To m_pages - 2 Do
      DirectXCheck(m_lpDDS_primary_page[i]^.lpVtbl^.Restore(m_lpDDS_primary_page[i]), 'm_lpDDS_primary_page^.Restore failed in TDirectXConsole.restore');
  End;

  If m_lpDDS_secondary <> Nil Then
  Begin
    If m_fullscreen Then
    Begin
      LOG('temporary primary surface clear');

      { temporary: clear primary surface }
      With rct Do
      Begin
	left := 0;
	top := 0;
	right := m_primary_width;
	bottom := m_primary_height;
      End;
      fx.dwSize := SizeOf(fx);
      fx.dwFillColor := 0;
      DirectXCheck(m_lpDDS_primary^.lpVtbl^.Blt(m_lpDDS_primary, @rct, Nil, Nil, DDBLT_COLORFILL Or DDBLT_WAIT, @fx), 'm_lpDDS_primary^.Blt failed in TDirectXPrimary.restore');
    End;
    LOG('restoring contents of secondary surface');
    { todo: restore contents of secondary surface }
  End;
End;

Procedure TDirectXPrimary.paint;

Var
  source, destination : RECT;
  pnt : POINT;
  x, y : Integer;
  fx : DDBLTFX;

Begin
  If Not active Then
  Begin
    LOG('paint when not active');
    Exit;
  End;
  If m_lpDDS_secondary <> Nil Then
  Begin
    If (m_lpDDS_primary^.lpVtbl^.IsLost(m_lpDDS_primary) <> DD_OK) Or
       (m_lpDDS_secondary^.lpVtbl^.IsLost(m_lpDDS_secondary) <> DD_OK) Then
    Begin
      LOG('paint when surfaces are lost');
      Exit;
    End;
    source.left := 0;
    source.top := 0;
    source.right := m_secondary_width;
    source.bottom := m_secondary_height;
    destination.left := 0;
    destination.top := 0;
    destination.right := m_primary_width;
    destination.bottom := m_primary_height;

    { note: code below assumes secondary is smaller than primary }
    If m_centering And m_fullscreen Then
    Begin
      x := (destination.right - source.right) Div 2;
      y := (destination.bottom - source.bottom) Div 2;

      destination.left := x;
      destination.top := y;
      destination.right := x + source.right;
      destination.bottom := y + source.bottom;
    End;
    If Not m_fullscreen Then
    Begin
      pnt.x := 0;
      pnt.y := 0;
      ClientToScreen(m_window.handle, pnt);

      GetClientRect(m_window.handle, destination);
      Inc(destination.left, pnt.x);
      Inc(destination.top, pnt.y);
      Inc(destination.right, pnt.x);
      Inc(destination.bottom, pnt.y);
    End;

    If ((source.right - source.left) = 0) Or
       ((source.bottom - source.top) = 0) Or
       ((destination.right - destination.left) = 0) Or
       ((destination.bottom - destination.top) = 0) Then
    Begin
      LOG('zero area in primary paint');
      Exit;
    End;

    If m_synchronize Then
    Begin
      fx.dwSize := SizeOf(fx);
      fx.dwDDFX := DDBLTFX_NOTEARING;
      Try
	DirectXCheck(m_lpDDS_primary_back^.lpVtbl^.Blt(m_lpDDS_primary_back, @destination, m_lpDDS_secondary, @source, DDBLT_WAIT Or DDBLT_DDFX, @fx), 'm_lpDDS_primary^.Blt (synchronized) failed in TDirectXPrimary.paint');
      Except
	On TPTCError Do
	Begin
	  LOG('falling back to unsynchronized blt');
	  m_synchronize := False;
	End;
      End;
    End;
    If Not m_synchronize Then
      DirectXCheck(m_lpDDS_primary_back^.lpVtbl^.Blt(m_lpDDS_primary_back, @destination, m_lpDDS_secondary, @source, DDBLT_WAIT, Nil), 'm_lpDDS_primary^.Blt (unsynchronized) failed in TDirectXPrimary.paint');
  End;
End;

Procedure TDirectXPrimary.blocking(_blocking : Boolean);

Begin
  m_blocking := _blocking;
End;

Function TDirectXPrimary.pack(Const color : TPTCColor; Const _format : TPTCFormat) : int32;

Var
  r_base, g_base, b_base, a_base : Integer;
  r_size, g_size, b_size, a_size : Integer;
  r_scale, g_scale, b_scale, a_scale : Single;

Begin
  If color.direct And _format.direct Then
  Begin
    r_base := 0;
    g_base := 0;
    b_base := 0;
    a_base := 0;
    r_size := 0;
    g_size := 0;
    b_size := 0;
    a_size := 0;
    analyse(_format.r, r_base, r_size);
    analyse(_format.g, g_base, g_size);
    analyse(_format.b, b_base, b_size);
    analyse(_format.a, a_base, a_size);
    r_scale := 1 Shl r_size;
    g_scale := 1 Shl g_size;
    b_scale := 1 Shl b_size;
    a_scale := 1 Shl a_size;
    pack := (Trunc(color.r * r_scale) Shl r_base) Or
	    (Trunc(color.g * g_scale) Shl g_base) Or
	    (Trunc(color.b * b_scale) Shl b_base) Or
	    (Trunc(color.a * a_scale) Shl a_base);
  End
  Else
    If color.indexed And _format.indexed Then
      pack := color.index
    Else
      Raise TPTCError.Create('color format type mismatch');
End;

Procedure TDirectXPrimary.analyse(mask : int32; Var base, size : Integer);

Begin
  base := 0;
  size := 0;
  If mask = 0 Then
    Exit;
  While (mask And 1) = 0 Do
  Begin
    mask := mask Shr 1;
    Inc(base);
  End;
  While (mask And 1) <> 0 Do
  Begin
    mask := mask Shr 1;
    Inc(size);
  End;
End;
