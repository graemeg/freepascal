<<<<<<< HEAD
<<<<<<< HEAD
{
    This file is part of the PTCPas framebuffer library
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2007, 2009, 2010  Nikolay Nikolov (nickysn@users.sourceforge.net)
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

type
  PWndProcRegEntry = ^TWndProcRegEntry;
  TWndProcRegEntry = record
    WindowHandle: HWND;
    Handler: TWinCEWndProc;
  end;

ThreadVar
  WndProcRegistry: array of TWndProcRegEntry;
  WndProcRegistryCache: Integer;

procedure WndProcAdd(AWindowHandle: HWND; AHandler: TWinCEWndProc);

var
  I: Integer;

begin
=======
=======
>>>>>>> origin/fixes_2_2
Type
  PWndProcRegEntry = ^TWndProcRegEntry;
  TWndProcRegEntry = Record
    WindowHandle : HWND;
    Handler : TWinCEWndProc;
  End;

ThreadVar
  WndProcRegistry : Array Of TWndProcRegEntry;
  WndProcRegistryCache : Integer;

Procedure WndProcAdd(AWindowHandle : HWND; AHandler : TWinCEWndProc);

Var
  I : Integer;

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  I := Length(WndProcRegistry);
  SetLength(WndProcRegistry, I + 1);
  WndProcRegistry[I].WindowHandle := AWindowHandle;
  WndProcRegistry[I].Handler := AHandler;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure WndProcRemove(AWindowHandle: HWND);

var
  I, J: Integer;

begin
  J := 0;
  for I := Low(WndProcRegistry) to High(WndProcRegistry) do
    if WndProcRegistry[I].WindowHandle <> AWindowHandle then
    begin
      WndProcRegistry[J] := WndProcRegistry[I];
      Inc(J);
    end;
  SetLength(WndProcRegistry, J);
end;

function WndProcFind(AWindowHandle: HWND): TWinCEWndProc;

var
  I: Integer;

begin
  if (WndProcRegistryCache >= Low(WndProcRegistry)) and
     (WndProcRegistryCache <= High(WndProcRegistry)) and
     (WndProcRegistry[WndProcRegistryCache].WindowHandle = AWindowHandle) then
  begin
    Result := WndProcRegistry[WndProcRegistryCache].Handler;
    exit;
  end;

  for I := Low(WndProcRegistry) to High(WndProcRegistry) do
    if WndProcRegistry[I].WindowHandle = AWindowHandle then
    begin
      Result := WndProcRegistry[I].Handler;
      WndProcRegistryCache := I;
      exit;
    end;
  Result := nil;
end;

function WinCEWindowProc(Ahwnd: HWND; AuMsg: UINT; AwParam: WPARAM; AlParam: LPARAM): LRESULT; cdecl;

var
  Handler: TWinCEWndProc;

begin
  Handler := WndProcFind(Ahwnd);
  if Handler <> nil then
    Result := Handler(Ahwnd, AuMsg, AwParam, AlParam)
  else
    Result := DefWindowProcW(Ahwnd, AuMsg, AwParam, AlParam);
end;

constructor TWinCEWindow.Create(const AClassName, ATitle: WideString;
                                AExStyle, AStyle: DWord;
				AShow, AX, AY, AWidth, AHeight: Integer;
                                AWndProc: TWinCEWndProc;
				AData: Pointer = Nil);

var
  ClassAtom: ATOM;
  wc: WNDCLASSW;
  ProgramInstance: HANDLE;
  Rectangle: RECT;
  X, Y, Width, Height: Integer;

begin
  ProgramInstance := GetModuleHandleW(Nil);
  if ProgramInstance = 0 then
    raise TPTCError.Create('could not get module handle');

  LOG('registering window class');
  FillChar(wc, SizeOf(wc), 0);
  wc.style := CS_DBLCLKS{ or CS_HREDRAW or CS_VREDRAW};
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure WndProcRemove(AWindowHandle : HWND);

Var
  I, J : Integer;

Begin
  J := 0;
  For I := Low(WndProcRegistry) To High(WndProcRegistry) Do
    If WndProcRegistry[I].WindowHandle <> AWindowHandle Then
    Begin
      WndProcRegistry[J] := WndProcRegistry[I];
      Inc(J);
    End;
  SetLength(WndProcRegistry, J);
End;

Function WndProcFind(AWindowHandle : HWND) : TWinCEWndProc;

Var
  I : Integer;

Begin
  If (WndProcRegistryCache >= Low(WndProcRegistry)) And
     (WndProcRegistryCache <= High(WndProcRegistry)) And
     (WndProcRegistry[WndProcRegistryCache].WindowHandle = AWindowHandle) Then
  Begin
    Result := WndProcRegistry[WndProcRegistryCache].Handler;
    Exit;
  End;

  For I := Low(WndProcRegistry) To High(WndProcRegistry) Do
    If WndProcRegistry[I].WindowHandle = AWindowHandle Then
    Begin
      Result := WndProcRegistry[I].Handler;
      WndProcRegistryCache := I;
      Exit;
    End;
  Result := Nil;
End;

Function WinCEWindowProc(Ahwnd : HWND; AuMsg : UINT; AwParam : WPARAM; AlParam : LPARAM) : LRESULT; CDecl;

Var
  Handler : TWinCEWndProc;

Begin
  Handler := WndProcFind(Ahwnd);
  If Handler <> Nil Then
    Result := Handler(Ahwnd, AuMsg, AwParam, AlParam)
  Else
    Result := DefWindowProcW(Ahwnd, AuMsg, AwParam, AlParam);
End;

Constructor TWinCEWindow.Create(Const AClassName, ATitle : WideString;
                                AExStyle, AStyle : DWord;
				AShow, AX, AY, AWidth, AHeight : Integer;
                                AWndProc : TWinCEWndProc;
				AData : Pointer = Nil);

Var
  ClassAtom : ATOM;
  wc : WNDCLASSW;
  ProgramInstance : HANDLE;
  Rectangle : RECT;
  X, Y, Width, Height : Integer;

Begin
  ProgramInstance := GetModuleHandleW(Nil);
  If ProgramInstance = 0 Then
    Raise TPTCError.Create('could not get module handle');

  LOG('registering window class');
  FillChar(wc, SizeOf(wc), 0);
  wc.style := CS_DBLCLKS{ Or CS_HREDRAW Or CS_VREDRAW};
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  wc.lpfnWndProc := @WinCEWindowProc;
  wc.cbClsExtra := 0;
  wc.cbWndExtra := 0;
  wc.hInstance := ProgramInstance;
  wc.hIcon := 0; { not supported by WinCE }
  wc.hCursor := 0;
  wc.hbrBackground := 0;
<<<<<<< HEAD
<<<<<<< HEAD
  wc.lpszMenuName := nil;
  wc.lpszClassName := PWideChar(AClassName);
  ClassAtom := RegisterClassW(@wc);
  if ClassAtom = 0 then
    raise TPTCError.Create('could not register window class');
  FClassName := AClassName;
  FClassHInstance := wc.hInstance;

  With Rectangle do
  begin
=======
=======
>>>>>>> origin/fixes_2_2
  wc.lpszMenuName := Nil;
  wc.lpszClassName := PWideChar(AClassName);
  ClassAtom := RegisterClassW(@wc);
  If ClassAtom = 0 Then
    Raise TPTCError.Create('could not register window class');
  FClassName := AClassName;
  FClassHInstance := wc.hInstance;

  With Rectangle Do
  Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    left := 0;
    top := 0;
    right := AWidth;
    bottom := AHeight;
<<<<<<< HEAD
<<<<<<< HEAD
  end;
  if not AdjustWindowRectEx(@Rectangle, AStyle, False, AExStyle) then
    raise TPTCError.Create('could not AdjustWindowRectEx');
=======
  End;
  If Not AdjustWindowRectEx(@Rectangle, AStyle, False, AExStyle) Then
    Raise TPTCError.Create('could not AdjustWindowRectEx');
>>>>>>> graemeg/fixes_2_2
=======
  End;
  If Not AdjustWindowRectEx(@Rectangle, AStyle, False, AExStyle) Then
    Raise TPTCError.Create('could not AdjustWindowRectEx');
>>>>>>> origin/fixes_2_2

  X := AX;
  Y := AY;
  Width := Rectangle.right - Rectangle.left;
  Height := Rectangle.bottom - Rectangle.top;

  FWindow := CreateWindowExW(AExStyle,
                             PWideChar(AClassName),
			     PWideChar(ATitle),
			     AStyle,
			     X, Y, Width, Height,
			     0, 0, 0,
			     AData);
<<<<<<< HEAD
<<<<<<< HEAD
  if (FWindow = 0) or not IsWindow(FWindow) then
    raise TPTCError.Create('could not create window');
  LOG('installing window message handler');
  WndProcAdd(FWindow, AWndProc);
  ShowWindow(FWindow, AShow);
  if SetFocus(FWindow) = 0 then
    raise TPTCError.Create('could not set focus to the new window');
  if SetActiveWindow(FWindow) = 0 then
    raise TPTCError.Create('could not set active window');
  if not SetForegroundWindow(FWindow) then
    raise TPTCError.Create('could not set foreground window');
  {...}
end;

destructor TWinCEWindow.Destroy;

begin
  Close;
  inherited Destroy;
end;

procedure TWinCEWindow.Close;

begin
  if (FWindow <> 0) and IsWindow(FWindow) then
  begin
    WndProcRemove(FWindow);
    DestroyWindow(FWindow);
  end;
  FWindow := 0;

  if FClassName <> '' then
    UnregisterClass(PWideChar(FClassName), FClassHInstance);
  FClassName := '';
end;

procedure TWinCEWindow.Update;

var
  Message: MSG;

begin
  while PeekMessage(@Message, FWindow, 0, 0, PM_REMOVE) do
  begin
    TranslateMessage(@Message);
    DispatchMessage(@Message);
  end;
end;
=======
=======
>>>>>>> origin/fixes_2_2
  If (FWindow = 0) Or Not IsWindow(FWindow) Then
    Raise TPTCError.Create('could not create window');
  LOG('installing window message handler');
  WndProcAdd(FWindow, AWndProc);
  ShowWindow(FWindow, AShow);
  If SetFocus(FWindow) = 0 Then
    Raise TPTCError.Create('could not set focus to the new window');
  If SetActiveWindow(FWindow) = 0 Then
    Raise TPTCError.Create('could not set active window');
  If Not SetForegroundWindow(FWindow) Then
    Raise TPTCError.Create('could not set foreground window');
  {...}
End;

Destructor TWinCEWindow.Destroy;

Begin
  Close;
  Inherited Destroy;
End;

Procedure TWinCEWindow.Close;

Begin
  If (FWindow <> 0) And IsWindow(FWindow) Then
  Begin
    WndProcRemove(FWindow);
    DestroyWindow(FWindow);
  End;
  FWindow := 0;

  If FClassName <> '' Then
    UnregisterClass(PWideChar(FClassName), FClassHInstance);
  FClassName := '';
End;

Procedure TWinCEWindow.Update;

Var
  Message : MSG;

Begin
  While PeekMessage(@Message, FWindow, 0, 0, PM_REMOVE) Do
  Begin
    TranslateMessage(@Message);
    DispatchMessage(@Message);
  End;
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
