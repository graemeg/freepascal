{
<<<<<<< HEAD
<<<<<<< HEAD
    This file is part of the PTCPas framebuffer library
    Copyright (C) 2007, 2009, 2010  Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)
>>>>>>> graemeg/fixes_2_2
=======
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)
>>>>>>> origin/fixes_2_2

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

<<<<<<< HEAD
<<<<<<< HEAD
constructor TWinCEMouse.Create(EventQueue: TEventQueue; FullScreen: Boolean; ConsoleWidth, ConsoleHeight: Integer);

begin
=======
Constructor TWinCEMouse.Create(EventQueue : TEventQueue; FullScreen : Boolean; ConsoleWidth, ConsoleHeight : Integer);

Begin
>>>>>>> graemeg/fixes_2_2
=======
Constructor TWinCEMouse.Create(EventQueue : TEventQueue; FullScreen : Boolean; ConsoleWidth, ConsoleHeight : Integer);

Begin
>>>>>>> origin/fixes_2_2
  FEventQueue := EventQueue;

  FFullScreen := FullScreen;
  FConsoleWidth := ConsoleWidth;
  FConsoleHeight := ConsoleHeight;

  FPreviousMousePositionSaved := False;

  { enable buffering }
  FEnabled := True;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TWinCEMouse.SetWindowArea(WindowX1, WindowY1, WindowX2, WindowY2: Integer);

begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWinCEMouse.SetWindowArea(WindowX1, WindowY1, WindowX2, WindowY2 : Integer);

Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  FWindowX1 := WindowX1;
  FWindowY1 := WindowY1;
  FWindowX2 := WindowX2;
  FWindowY2 := WindowY2;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TWinCEMouse.enable;

begin
  { enable buffering }
  FEnabled := True;
end;

procedure TWinCEMouse.disable;

begin
  { disable buffering }
  FEnabled := False;
end;

function TWinCEMouse.WndProc(hWnd: HWND; message: DWord; wParam: WPARAM; lParam: LPARAM): LRESULT;

var
  fwKeys: Integer;
  xPos, yPos: Integer;
  LButton, MButton, RButton: Boolean;
  TranslatedXPos, TranslatedYPos: Integer;
  PTCMouseButtonState: TPTCMouseButtonState;
  WindowRect: RECT;

  button: TPTCMouseButton;
  before, after: Boolean;
  cstate: TPTCMouseButtonState;

begin
  Result := 0;
  { check enabled flag }
  if not FEnabled then
    exit;

  if (message = WM_MOUSEMOVE) or
     (message = WM_LBUTTONDOWN) or (message = WM_LBUTTONUP) or (message = WM_LBUTTONDBLCLK) or
     (message = WM_MBUTTONDOWN) or (message = WM_MBUTTONUP) or (message = WM_MBUTTONDBLCLK) or
     (message = WM_RBUTTONDOWN) or (message = WM_RBUTTONUP) or (message = WM_RBUTTONDBLCLK) then
  begin
    fwKeys := wParam; {MK_LBUTTON or MK_MBUTTON or MK_RBUTTON or MK_CONTROL or MK_SHIFT}
    xPos := lParam and $FFFF;
    yPos := (lParam shr 16) and $FFFF;

    LButton := (fwKeys and MK_LBUTTON) <> 0;
    MButton := (fwKeys and MK_MBUTTON) <> 0;
    RButton := (fwKeys and MK_RBUTTON) <> 0;

    if not FFullScreen then
    begin
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWinCEMouse.enable;

Begin
  { enable buffering }
  FEnabled := True;
End;

Procedure TWinCEMouse.disable;

Begin
  { disable buffering }
  FEnabled := False;
End;

Function TWinCEMouse.WndProc(hWnd : HWND; message : DWord; wParam : WPARAM; lParam : LPARAM) : LRESULT;

Var
  fwKeys : Integer;
  xPos, yPos : Integer;
  LButton, MButton, RButton : Boolean;
  TranslatedXPos, TranslatedYPos : Integer;
  PTCMouseButtonState : TPTCMouseButtonState;
  WindowRect : RECT;

  button : TPTCMouseButton;
  before, after : Boolean;
  cstate : TPTCMouseButtonState;

Begin
  Result := 0;
  { check enabled flag }
  If Not FEnabled Then
    Exit;

  If (message = WM_MOUSEMOVE) Or
     (message = WM_LBUTTONDOWN) Or (message = WM_LBUTTONUP) Or (message = WM_LBUTTONDBLCLK) Or
     (message = WM_MBUTTONDOWN) Or (message = WM_MBUTTONUP) Or (message = WM_MBUTTONDBLCLK) Or
     (message = WM_RBUTTONDOWN) Or (message = WM_RBUTTONUP) Or (message = WM_RBUTTONDBLCLK) Then
  Begin
    fwKeys := wParam; {MK_LBUTTON or MK_MBUTTON or MK_RBUTTON or MK_CONTROL or MK_SHIFT}
    xPos := lParam And $FFFF;
    yPos := (lParam Shr 16) And $FFFF;

    LButton := (fwKeys And MK_LBUTTON) <> 0;
    MButton := (fwKeys And MK_MBUTTON) <> 0;
    RButton := (fwKeys And MK_RBUTTON) <> 0;

    If Not FFullScreen Then
    Begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      GetClientRect(hWnd, WindowRect);

      FWindowX1 := WindowRect.left;
      FWindowY1 := WindowRect.top;
      FWindowX2 := WindowRect.right - 1;
      FWindowY2 := WindowRect.bottom - 1;
<<<<<<< HEAD
<<<<<<< HEAD
    end;

    if (xPos >= FWindowX1) and (yPos >= FWindowY1) and
       (xPos <= FWindowX2) and (yPos <= FWindowY2) then
    begin
      if FWindowX2 <> FWindowX1 then
        TranslatedXPos := (xPos - FWindowX1) * (FConsoleWidth  - 1) div (FWindowX2 - FWindowX1)
      else { avoid div by zero }
        TranslatedXPos := 0;

      if FWindowY2 <> FWindowY1 then
        TranslatedYPos := (yPos - FWindowY1) * (FConsoleHeight - 1) div (FWindowY2 - FWindowY1)
      else { avoid div by zero }
        TranslatedYPos := 0;

      { Just in case... }
      if TranslatedXPos < 0 then
        TranslatedXPos := 0;
      if TranslatedYPos < 0 then
        TranslatedYPos := 0;
      if TranslatedXPos >= FConsoleWidth then
        TranslatedXPos := FConsoleWidth - 1;
      if TranslatedYPos >= FConsoleHeight then
        TranslatedYPos := FConsoleHeight - 1;

      if not LButton then
        PTCMouseButtonState := []
      else
        PTCMouseButtonState := [PTCMouseButton1];

      if RButton then
        PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton2];

      if MButton then
        PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton3];

      if not FPreviousMousePositionSaved then
      begin
        FPreviousMouseX := TranslatedXPos; { first DeltaX will be 0 }
        FPreviousMouseY := TranslatedYPos; { first DeltaY will be 0 }
        FPreviousMouseButtonState := [];
      end;

      { movement? }
      if (TranslatedXPos <> FPreviousMouseX) or (TranslatedYPos <> FPreviousMouseY) then
=======
=======
>>>>>>> origin/fixes_2_2
    End;

    If (xPos >= FWindowX1) And (yPos >= FWindowY1) And
       (xPos <= FWindowX2) And (yPos <= FWindowY2) Then
    Begin
      If FWindowX2 <> FWindowX1 Then
        TranslatedXPos := (xPos - FWindowX1) * (FConsoleWidth  - 1) Div (FWindowX2 - FWindowX1)
      Else { avoid div by zero }
        TranslatedXPos := 0;

      If FWindowY2 <> FWindowY1 Then
        TranslatedYPos := (yPos - FWindowY1) * (FConsoleHeight - 1) Div (FWindowY2 - FWindowY1)
      Else { avoid div by zero }
        TranslatedYPos := 0;

      { Just in case... }
      If TranslatedXPos < 0 Then
        TranslatedXPos := 0;
      If TranslatedYPos < 0 Then
        TranslatedYPos := 0;
      If TranslatedXPos >= FConsoleWidth Then
        TranslatedXPos := FConsoleWidth - 1;
      If TranslatedYPos >= FConsoleHeight Then
        TranslatedYPos := FConsoleHeight - 1;

      If Not LButton Then
        PTCMouseButtonState := []
      Else
        PTCMouseButtonState := [PTCMouseButton1];

      If RButton Then
        PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton2];

      If MButton Then
        PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton3];

      If Not FPreviousMousePositionSaved Then
      Begin
        FPreviousMouseX := TranslatedXPos; { first DeltaX will be 0 }
        FPreviousMouseY := TranslatedYPos; { first DeltaY will be 0 }
        FPreviousMouseButtonState := [];
      End;

      { movement? }
      If (TranslatedXPos <> FPreviousMouseX) Or (TranslatedYPos <> FPreviousMouseY) Then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        FEventQueue.AddEvent(TPTCMouseEvent.Create(TranslatedXPos, TranslatedYPos, TranslatedXPos - FPreviousMouseX, TranslatedYPos - FPreviousMouseY, FPreviousMouseButtonState));

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
          FEventQueue.AddEvent(TPTCMouseButtonEvent.Create(TranslatedXPos, TranslatedYPos, 0, 0, cstate, True, button));
        end
        else
          if before and (not after) then
          begin
            { button was released }
            cstate := cstate - [button];
            FEventQueue.AddEvent(TPTCMouseButtonEvent.Create(TranslatedXPos, TranslatedYPos, 0, 0, cstate, False, button));
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
          FEventQueue.AddEvent(TPTCMouseButtonEvent.Create(TranslatedXPos, TranslatedYPos, 0, 0, cstate, True, button));
        End
        Else
          If before And (Not after) Then
          Begin
            { button was released }
            cstate := cstate - [button];
            FEventQueue.AddEvent(TPTCMouseButtonEvent.Create(TranslatedXPos, TranslatedYPos, 0, 0, cstate, False, button));
          End;
      End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

      FPreviousMouseX := TranslatedXPos;
      FPreviousMouseY := TranslatedYPos;
      FPreviousMouseButtonState := PTCMouseButtonState;
      FPreviousMousePositionSaved := True;
<<<<<<< HEAD
<<<<<<< HEAD
    end;
  end;
end;
=======
    End;
  End;
End;
>>>>>>> graemeg/fixes_2_2
=======
    End;
  End;
End;
>>>>>>> origin/fixes_2_2
