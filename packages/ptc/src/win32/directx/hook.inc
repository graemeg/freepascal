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

Constructor TDirectXHook.Create(console : Pointer; window : HWND; thread : DWord; _cursor, managed, fullscreen : Boolean);

Begin
  m_console := console;

  m_cursor := _cursor;
  m_managed := managed;
  m_fullscreen := fullscreen;

  LOG('creating window hook');

  Inherited Create(window, thread);
End;

Destructor TDirectXHook.Destroy;

Begin
  LOG('destroying window hook');
  Inherited Destroy;
End;

Procedure TDirectXHook.cursor(flag : Boolean);

Begin
  m_cursor := flag;
End;

Function TDirectXHook.WndProc(hWnd : HWND; message : DWord; wParam : WPARAM; lParam : LPARAM) : LRESULT;

Var
  active : Boolean;
  thread : DWord;
  placement : WINDOWPLACEMENT;
  console : TDirectXConsole;

Begin
  Case message Of
    WM_PAINT : Begin
      LOG('TDirectXHook WM_PAINT');

      { paint console }
      TDirectXConsole(m_console).paint;
    End;
    WM_ACTIVATEAPP : Begin
      LOG('TDirectXHook WM_ACTIVATEAPP');

      { get window message data }
      active := Boolean(wParam);
      thread := lParam;

      { check active flag }
      If active = False Then
      Begin
        { deactivate }
        deactivate;

        { check cursor and fullscreen }
        If (Not m_cursor) And m_fullscreen Then
          { show cursor }
          Win32Cursor_resurrect;
      End
      Else
      Begin
        { check cursor and fullscreen }
        If (Not m_cursor) And m_fullscreen Then
        Begin
          { get window placement for active app }
          If Not GetWindowPlacement(hWnd, placement) Then
            { on failure set to normal show }
            placement.showCmd := SW_SHOWNORMAL;

          { check show command is not minimize }
          If placement.showCmd <> SW_SHOWMINIMIZED Then
            {hide cursor}
            Win32Cursor_kill;
        End;

        { activate }
        activate;
      End;

      { handled }
      WndProc := 1;
      Exit;
    End;
    WM_SETCURSOR : Begin
      { check cursor }
      If Not m_cursor Then
        { hide cursor }
        SetCursor(12);

      { handled }
      WndProc := 1;
      Exit;
    End;
    WM_CLOSE : Begin
      LOG('TDirectXHook WM_CLOSE');

      If m_managed Then
      Begin
        console := TDirectXConsole(m_console);

        { close console }
        console.close;

        { note: at this point the hook object has been destroyed by the console! }

        { internal console shutdown }
        console.internal_shutdown;

        { halt }
        Halt(0);
      End;

      { handled }
      WndProc := 1;
      Exit;
    End;
  End;

  { unhandled }
  WndProc := 0;
End;

Procedure TDirectXHook.activate;

Var
  console : TDirectXConsole;
  display : TDirectXDisplay;
  primary : TDirectXPrimary;

Begin
  console := TDirectXConsole(m_console);
  { check if open }
  If console.m_open Then
  Begin
    LOG('activate');

    { get console object references }
    display := console.m_display;
    primary := console.m_primary;

    { check if primary is not active }
    If Not primary.active Then
    Begin
      { save display }
      display.save;

      { activate primary }
      primary.activate;
    End;
  End;
End;

Procedure TDirectXHook.deactivate;

Var
  console : TDirectXConsole;
  display : TDirectXDisplay;
  primary : TDirectXPrimary;

Begin
  console := TDirectXConsole(m_console);
  { check if open }
  If console.m_open Then
  Begin
    LOG('deactivate');

    { get console object references }
    display := console.m_display;
    primary := console.m_primary;

    { check if primary is not active }
    If primary.active Then
    Begin
      { save primary }
      primary.save;

      { deactivate primary }
      primary.deactivate;

      { restore display }
      display.restore;
    End;
  End;
End;
