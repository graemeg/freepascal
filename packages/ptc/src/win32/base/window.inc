{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
    version 2.1 of the License, or (at your option) any later version.
>>>>>>> graemeg/fixes_2_2
=======
    version 2.1 of the License, or (at your option) any later version.
>>>>>>> origin/fixes_2_2

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

{ $R win32\base\ptcres.res}

{ bug in the compiler???}
{ $LINKLIB ptc.owr}

<<<<<<< HEAD
<<<<<<< HEAD
constructor TWin32Window.Create(window: HWND);
begin
  LOG('attaching to user managed window');
  defaults;
  FWindow := window;
  FManaged := False;
end;

constructor TWin32Window.Create(const AWndClass, ATitle: string; AExtra, AStyle: DWord;
                                AShow, AX, AY, AWidth, AHeight: Integer; ACenter, AMultithreaded: Boolean;
				AData: Pointer);
begin
  internal_create(AWndClass, ATitle, AExtra, AStyle, AShow, AX, AY, AWidth, AHeight, ACenter, AMultithreaded, AData);
end;

destructor TWin32Window.Destroy;
begin
  Close;
  inherited Destroy;
end;

procedure TWin32Window.Cursor(AFlag: Boolean);
begin
  if AFlag then
  begin
    {$WARNING GCLP_HCURSOR not defined in windows unit}
//    SetClassLong(FWindow, GCL_HCURSOR, LoadCursor(0, IDC_ARROW));
    SetClassLongPtr(FWindow, {GCLP_HCURSOR}GCL_HCURSOR, LoadCursor(0, IDC_ARROW));
  end
  else
  begin
    {$WARNING GCLP_HCURSOR not defined in windows unit}
//    SetClassLong(FWindow, GCL_HCURSOR, 0);
    SetClassLongPtr(FWindow, {GCLP_HCURSOR}GCL_HCURSOR, 0);
  end;
  SendMessage(FWindow, WM_SETCURSOR, 0, 0);
end;

procedure TWin32Window.ConfineCursor(AFlag: Boolean);
var
  rct: TRECT;
  p1, p2: TPOINT;
begin
  if AFlag then
  begin
    GetClientRect(FWindow, @rct);
    p1.x := rct.left;
    p1.y := rct.top;
    p2.x := rct.right;
    p2.y := rct.bottom;
    ClientToScreen(FWindow, @p1);
    ClientToScreen(FWindow, @p2);
    rct.left := p1.x;
    rct.top := p1.y;
    rct.right := p2.x;
    rct.bottom := p2.y;
    ClipCursor(@rct);
    FCursorConfineInEffect := True;
  end
  else
  begin
    if FCursorConfineInEffect then
    begin
      ClipCursor(nil);
      FCursorConfineInEffect := False;
    end;
  end;
end;

procedure TWin32Window.Resize(AWidth, AHeight: Integer);
var
  window_rectangle: RECT;
  rectangle: RECT;
begin
  GetWindowRect(FWindow, window_rectangle);
  with rectangle do
  begin
    left := 0;
    top := 0;
    right := AWidth;
    bottom := AHeight;
  end;
  AdjustWindowRectEx(rectangle, FStyle, False, FExtra);
  SetWindowPos(FWindow, HWND_TOP, window_rectangle.left,
=======
=======
>>>>>>> origin/fixes_2_2
Constructor TWin32Window.Create(window : HWND);

Begin
  LOG('attaching to user managed window');
  defaults;
  m_window := window;
  m_managed := False;
End;

Constructor TWin32Window.Create(wndclass, title : String; extra, style : DWord; show, x, y, width, height : Integer; center, _multithreaded : Boolean; data : Pointer);

Begin
  internal_create(wndclass, title, extra, style, show, x, y, width, height, center, _multithreaded, data);
End;

Constructor TWin32Window.Create(wndclass, title : String; extra, style : DWord; show, x, y, width, height : Integer; center, _multithreaded : Boolean);

Begin
  internal_create(wndclass, title, extra, style, show, x, y, width, height, center, _multithreaded, Nil);
End;

Destructor TWin32Window.Destroy;

Begin
  close;
  Inherited Destroy;
End;

Procedure TWin32Window.cursor(flag : Boolean);

Begin
  If flag Then
  Begin
    SetClassLong(m_window, GCL_HCURSOR, LoadCursor(0, IDC_ARROW));
  End
  Else
  Begin
    SetClassLong(m_window, GCL_HCURSOR, 0);
  End;
  SendMessage(m_window, WM_SETCURSOR, 0, 0);
End;

Procedure TWin32Window.resize(width, height : Integer);

Var
  window_rectangle : RECT;
  rectangle : RECT;

Begin
  GetWindowRect(m_window, window_rectangle);
  With rectangle Do
  Begin
    left := 0;
    top := 0;
    right := width;
    bottom := height;
  End;
  AdjustWindowRectEx(rectangle, m_style, False, m_extra);
  SetWindowPos(m_window, HWND_TOP, window_rectangle.left,
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      window_rectangle.top, rectangle.right - rectangle.left,
      rectangle.bottom - rectangle.top, 0);
  {
  todo: detect if the window is resized off the screen and let windows reposition it correctly... ?
  }
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TWin32Window.Update(AForce: Boolean = False; AWaitForMessage: Boolean = False);
var
  message: MSG;
begin
  if (not FManaged) and (not AForce) then
    exit;
  if not FMultithreaded then
  begin
    { updated to pump all window messages, and not just for our FWindow;
      this fixes keyboard layout switching and maybe other bugs and side effects...
      Seems like Windows wants everything pumped :) }
    if AWaitForMessage then
    begin
      GetMessage(message, {FWindow}0, 0, 0);
      TranslateMessage(message);
      DispatchMessage(message);
    end
    else
      while PeekMessage(message, {FWindow}0, 0, 0, PM_REMOVE) do
      begin
        TranslateMessage(message);
        DispatchMessage(message);
      end;
  end
  else
    Sleep(0);
end;

function TWin32Window.GetThread: DWord;
begin
  if FMultithreaded then
    Result := FID
  else
    Result := GetCurrentThreadId;
end;

function WndProcSingleThreaded(hWnd: HWND; message: UINT; wParam: WPARAM; lParam: LPARAM): LRESULT; stdcall;
begin
  case message of
    WM_SETCURSOR: begin
      {$WARNING GCLP_HCURSOR not defined in windows unit}
      if (LOWORD(lParam) = HTCLIENT) and (GetClassLongPtr(hWnd, {GCLP_HCURSOR}GCL_HCURSOR) = 0) then
      begin
        SetCursor(0);
	Result := 1;
      end
      else
        Result := DefWindowProc(hWnd, message, wParam, lParam);
    end;
    WM_CLOSE: begin
      LOG('TWin32Window WM_CLOSE');
      Halt(0);
    end;
    else
      Result := DefWindowProc(hWnd, message, wParam, lParam);
  end;
end;

function WndProcMultiThreaded(hWnd: HWND; message: UINT; wParam: WPARAM; lParam: LPARAM): LRESULT; stdcall;
begin
  Result := 0;
  case message of
    WM_SETCURSOR: begin
      {$WARNING GCLP_HCURSOR not defined in windows unit}
      if (LOWORD(lParam) = HTCLIENT) and (GetClassLongPtr(hWnd, {GCLP_HCURSOR}GCL_HCURSOR) = 0) then
      begin
        SetCursor(0);
	Result := 1;
      end
      else
        Result := DefWindowProc(hWnd, message, wParam, lParam);
    end;
    WM_DESTROY: begin
      LOG('TWin32Window WM_DESTROY');
      PostQuitMessage(0);
    end;
    WM_CLOSE: begin
      LOG('TWin32Window WM_CLOSE');
      Halt(0);
    end;
    else
      Result := DefWindowProc(hWnd, message, wParam, lParam);
  end;
end;

procedure TWin32Window.internal_create(const AWndClass, ATitle: string; AExtra, AStyle: DWord;
                                       AShow, AX, AY, AWidth, AHeight: Integer; ACenter, AMultithreaded: Boolean;
				       AData: Pointer);
var
  program_instance{, library_instance}: DWord;
  rectangle: RECT;
  display_width, display_height: Integer;
  wc: WNDCLASSEX;
begin
  LOG('creating managed window');
  Defaults;
  FMultithreaded := AMultithreaded;
  try
    program_instance := GetModuleHandle(nil);
{    library_instance := program_instance;}
    wc.cbSize := SizeOf(WNDCLASSEX);
    wc.hInstance := program_instance;
    wc.lpszClassName := PChar(AWndClass);
    wc.style := CS_VREDRAW or CS_HREDRAW;
    wc.hIcon := 0{LoadIcon(library_instance, 'IDI_PTC_ICON')};
    wc.hIconSm := 0;
    wc.lpszMenuName := nil;
    wc.cbClsExtra := 0;
    wc.cbWndExtra := 0;
    wc.hbrBackground := 0;{(HBRUSH) GetStockObject(BLACK_BRUSH)}
    if AMultithreaded then
      wc.lpfnWndProc := @WndProcMultiThreaded
    else
      wc.lpfnWndProc := @WndProcSingleThreaded;
    wc.hCursor := LoadCursor(0, IDC_ARROW);
    RegisterClassEx(wc);
    with rectangle do
    begin
      left := 0;
      top := 0;
      right := AWidth;
      bottom := AHeight;
    end;
    AdjustWindowRectEx(rectangle, AStyle, False, AExtra);
    if ACenter then
    begin
      LOG('centering window');
      display_width := GetSystemMetrics(SM_CXSCREEN);
      display_height := GetSystemMetrics(SM_CYSCREEN);
      AX := (display_width - (rectangle.right - rectangle.left)) div 2;
      AY := (display_height - (rectangle.bottom - rectangle.top)) div 2;
    end;
    FName := AWndClass;
    FTitle := ATitle;
    FExtra := AExtra;
    FStyle := AStyle;
    FShow := AShow;
    FX := AX;
    FY := AY;
    FWidth := rectangle.right - rectangle.left;
    FHeight := rectangle.bottom - rectangle.top;
    FData := AData;
    if AMultithreaded then
    begin
      {...}
    end
    else
    begin
      FWindow := CreateWindowEx(FExtra, PChar(FName), PChar(FTitle), FStyle, FX, FY, FWidth, FHeight, 0, 0, 0, FData);
      if not IsWindow(FWindow) then
        raise TPTCError.Create('could not create window');
      ShowWindow(FWindow, FShow);
      SetFocus(FWindow);
      SetActiveWindow(FWindow);
      SetForegroundWindow(FWindow);
    end;
  except
    on error: TPTCError do
      raise TPTCError.Create('could not create window', error);
  end;
end;

procedure TWin32Window.Defaults;
begin
  FWindow := 0;
  FEvent := 0;
  FThread := 0;
  FID := 0;
  FName := '';
  FTitle := '';
  FExtra := 0;
  FStyle := 0;
  FShow := 0;
  FX := 0;
  FY := 0;
  FWidth := 0;
  FHeight := 0;
  FData := nil;
  FManaged := True;
  FMultithreaded := False;
end;

procedure TWin32Window.Close;
begin
  if FCursorConfineInEffect then
    ConfineCursor(False);

  if not FManaged then
  begin
    LOG('detaching from user managed window');
    FWindow := 0;
  end
  else
  begin
    LOG('closing managed window');
    if FMultithreaded then
    begin
      if (FThread <> 0) and IsWindow(FWindow) then
      begin
        PostMessage(FWindow, WM_DESTROY, 0, 0);
        WaitForSingleObject(FThread, INFINITE);
      end;
      if FEvent <> 0 then
        CloseHandle(FEvent);
      if FThread <> 0 then
        CloseHandle(FThread);
      SetPriorityClass(GetCurrentProcess, NORMAL_PRIORITY_CLASS);
    end
    else
      if (FWindow <> 0) and IsWindow(FWindow) then
        DestroyWindow(FWindow);
    FWindow := 0;
    FEvent := 0;
    FThread := 0;
    FID := 0;
    UnregisterClass(PChar(FName), GetModuleHandle(Nil));
  end;
end;

class procedure TWin32Window.ThreadFunction(AOwner: TWin32Window);
var
  message: MSG;
begin
  with AOwner do
  begin
    FWindow := CreateWindowEx(FExtra, PChar(FName), PChar(FTitle), FStyle, FX, FY, FWidth, FHeight, 0, 0, 0, FData);
    if IsWindow(FWindow) then
    begin
      ShowWindow(FWindow, FShow);
      SetFocus(FWindow);
      SetForegroundWindow(FWindow);
      SetEvent(FEvent);
      while GetMessage(message, 0, 0, 0) = True do
      begin
        TranslateMessage(message);
        DispatchMessage(message);
      end;
    end
    else
      SetEvent(FEvent);
  end;
end;
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWin32Window.update(force : Boolean);

Var
  message : MSG;

Begin
  If (Not m_managed) And (Not force) Then
    Exit;
  If Not m_multithreaded Then
  Begin
    While PeekMessage(message, m_window, 0, 0, PM_REMOVE) Do
    Begin
      TranslateMessage(message);
      DispatchMessage(message);
    End;
  End
  Else
    Sleep(0);
End;

Procedure TWin32Window.update; {force = False}

Begin
  update(False);
End;

Function TWin32Window.handle : HWND;

Begin
  handle := m_window;
End;

Function TWin32Window.thread : DWord;

Begin
  If m_multithreaded Then
    thread := m_id
  Else
    thread := GetCurrentThreadId;
End;

Function TWin32Window.managed : Boolean;

Begin
  managed := m_managed;
End;

Function TWin32Window.multithreaded : Boolean;

Begin
  multithreaded := m_multithreaded;
End;

Function WndProcSingleThreaded(hWnd : HWND; message : UINT; wParam : WPARAM; lParam : LPARAM) : LRESULT; StdCall;

Begin
  Case message Of
    WM_CLOSE : Begin
      LOG('TWin32Window WM_CLOSE');
      Halt(0);
    End;
    Else
      WndProcSingleThreaded := DefWindowProc(hWnd, message, wParam, lParam);
  End;
End;

Function WndProcMultiThreaded(hWnd : HWND; message : UINT; wParam : WPARAM; lParam : LPARAM) : LRESULT; StdCall;

Begin
  WndProcMultiThreaded := 0;
  Case message Of
    WM_DESTROY : Begin
      LOG('TWin32Window WM_DESTROY');
      PostQuitMessage(0);
    End;
    WM_CLOSE : Begin
      LOG('TWin32Window WM_CLOSE');
      Halt(0);
    End;
    Else
      WndProcMultiThreaded := DefWindowProc(hWnd, message, wParam, lParam);
  End;
End;

Procedure TWin32Window.internal_create(wndclass, title : String; extra, style : DWord; show, x, y, width, height : Integer; center, _multithreaded : Boolean; data : Pointer);

Var
  program_instance{, library_instance} : DWord;
  rectangle : RECT;
  display_width, display_height : Integer;
  wc : WNDCLASSEX;

Begin
  LOG('creating managed window');
  defaults;
  m_multithreaded := _multithreaded;
  wndclass := wndclass + #0;
  title := title + #0;
  Try
    program_instance := GetModuleHandle(Nil);
{    library_instance := program_instance;}
    wc.cbSize := SizeOf(WNDCLASSEX);
    wc.hInstance := program_instance;
    wc.lpszClassName := @wndclass[1];
    wc.style := CS_VREDRAW Or CS_HREDRAW;
    wc.hIcon := 0{LoadIcon(library_instance, 'IDI_PTC_ICON')};
    wc.hIconSm := 0;
    wc.lpszMenuName := Nil;
    wc.cbClsExtra := 0;
    wc.cbWndExtra := 0;
    wc.hbrBackground := 0;{(HBRUSH) GetStockObject(BLACK_BRUSH)}
    If multithreaded Then
      wc.lpfnWndProc := @WndProcMultiThreaded
    Else
      wc.lpfnWndProc := @WndProcSingleThreaded;
    wc.hCursor := LoadCursor(0, IDC_ARROW);
    RegisterClassEx(wc);
    With rectangle Do
    Begin
      left := 0;
      top := 0;
      right := width;
      bottom := height;
    End;
    AdjustWindowRectEx(rectangle, style, False, extra);
    If center Then
    Begin
      LOG('centering window');
      display_width := GetSystemMetrics(SM_CXSCREEN);
      display_height := GetSystemMetrics(SM_CYSCREEN);
      x := (display_width - (rectangle.right - rectangle.left)) Div 2;
      y := (display_height - (rectangle.bottom - rectangle.top)) Div 2;
    End;
    m_name := wndclass;
    m_title := title;
    m_extra := extra;
    m_style := style;
    m_show := show;
    m_x := x;
    m_y := y;
    m_width := rectangle.right - rectangle.left;
    m_height := rectangle.bottom - rectangle.top;
    m_data := data;
    If multithreaded Then
    Begin
      {...}
    End
    Else
    Begin
      m_window := CreateWindowEx(m_extra, PChar(m_name), PChar(m_title), m_style, m_x, m_y, m_width, m_height, 0, 0, 0, m_data);
      If Not IsWindow(m_window) Then
        Raise TPTCError.Create('could not create window');
      ShowWindow(m_window, m_show);
      SetFocus(m_window);
      SetActiveWindow(m_window);
      SetForegroundWindow(m_window);
    End;
  Except
    On error : TPTCError Do
      Raise TPTCError.Create('could not create window', error);
  End;
End;

Procedure TWin32Window.defaults;

Begin
  m_window := 0;
  m_event := 0;
  m_thread := 0;
  m_id := 0;
  m_name := '';
  m_title := '';
  m_extra := 0;
  m_style := 0;
  m_show := 0;
  m_x := 0;
  m_y := 0;
  m_width := 0;
  m_height := 0;
  m_data := Nil;
  m_managed := True;
  m_multithreaded := False;
End;

Procedure TWin32Window.close;

Begin
  If Not m_managed Then
  Begin
    LOG('detaching from user managed window');
    m_window := 0;
  End
  Else
  Begin
    LOG('closing managed window');
    If m_multithreaded Then
    Begin
      If (m_thread <> 0) And IsWindow(m_window) Then
      Begin
        PostMessage(m_window, WM_DESTROY, 0, 0);
        WaitForSingleObject(m_thread, INFINITE);
      End;
      If m_event <> 0 Then
        CloseHandle(m_event);
      If m_thread <> 0 Then
        CloseHandle(m_thread);
      SetPriorityClass(GetCurrentProcess, NORMAL_PRIORITY_CLASS);
    End
    Else
      If (m_window <> 0) And IsWindow(m_window) Then
        DestroyWindow(m_window);
    m_window := 0;
    m_event := 0;
    m_thread := 0;
    m_id := 0;
    UnregisterClass(PChar(m_name), GetModuleHandle(Nil));
  End;
End;

Class Procedure TWin32Window.ThreadFunction(owner : TWin32Window);

Var
  message : MSG;

Begin
  With owner Do
  Begin
    m_window := CreateWindowEx(m_extra, PChar(m_name), PChar(m_title), m_style, m_x, m_y, m_width, m_height, 0, 0, 0, m_data);
    If IsWindow(m_window) Then
    Begin
      ShowWindow(m_window, m_show);
      SetFocus(m_window);
      SetForegroundWindow(m_window);
      SetEvent(m_event);
      While GetMessage(message, 0, 0, 0) = True Do
      Begin
        TranslateMessage(message);
        DispatchMessage(message);
      End;
    End
    Else
      SetEvent(m_event);
  End;
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
