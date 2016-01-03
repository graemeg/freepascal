{
<<<<<<< HEAD
<<<<<<< HEAD
    This file is part of the PTCPas framebuffer library
    Copyright (C) 2007, 2009, 2010  Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)
>>>>>>> graemeg/fixes_2_2
=======
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
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
constructor TWinCEKeyboard.Create(EventQueue: TEventQueue);

begin
//  m_monitor := nil;
//  m_event := nil;
//  inherited Create(window, thread);
=======
=======
>>>>>>> origin/fixes_2_2
Constructor TWinCEKeyboard.Create(EventQueue : TEventQueue);

Begin
//  m_monitor := Nil;
//  m_event := Nil;
//  Inherited Create(window, thread);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//  m_monitor := TWin32Monitor.Create;
//  m_event := TWin32Event.Create;

  { setup defaults }
  m_alt := False;
  m_shift := False;
  m_control := False;

  { setup data }
  FEventQueue := EventQueue;
//  m_multithreaded := multithreaded;

  { enable buffering }
  m_enabled := True;
<<<<<<< HEAD
<<<<<<< HEAD
end;

procedure TWinCEKeyboard.enable;

begin
  { enable buffering }
  m_enabled := True;
end;

procedure TWinCEKeyboard.disable;

begin
  { disable buffering }
  m_enabled := False;
end;

function TWinCEKeyboard.WndProc(hWnd: HWND; message: DWord; wParam: WPARAM; lParam: LPARAM): LRESULT;

var
  i: Integer;
  scancode: Integer;
  KeyStateArray: array [0..255] of Byte;
  AsciiBuf: Word;
  press: Boolean;
  uni: Integer;
  tmp: Integer;

begin
  WndProc := 0;
  { check enabled flag }
  if not m_enabled then
    exit;

  { process key message }
  if (message = WM_KEYDOWN) or (message = WM_KEYUP) {Or ((message = WM_SYSKEYDOWN) and ((lParam and (1 shl 29)) <> 0))} then
  begin
    if message = WM_KEYUP then
      press := False
    else
      press := True;

    { update modifiers }
    if wParam = VK_MENU then
      { alt }
      m_alt := press
    else
      if wParam = VK_SHIFT then
        { shift }
        m_shift := press
      else
        if wParam = VK_CONTROL then
=======
=======
>>>>>>> origin/fixes_2_2
End;

Procedure TWinCEKeyboard.enable;

Begin
  { enable buffering }
  m_enabled := True;
End;

Procedure TWinCEKeyboard.disable;

Begin
  { disable buffering }
  m_enabled := False;
End;

Function TWinCEKeyboard.WndProc(hWnd : HWND; message : DWord; wParam : WPARAM; lParam : LPARAM) : LRESULT;

Var
  i : Integer;
  scancode : Integer;
  KeyStateArray : Array[0..255] Of Byte;
  AsciiBuf : Word;
  press : Boolean;
  uni : Integer;
  tmp : Integer;

Begin
  WndProc := 0;
  { check enabled flag }
  If Not m_enabled Then
    Exit;

  { process key message }
  If (message = WM_KEYDOWN) Or (message = WM_KEYUP) {Or ((message = WM_SYSKEYDOWN) And ((lParam And (1 Shl 29)) <> 0))} Then
  Begin
    If message = WM_KEYUP Then
      press := False
    Else
      press := True;

    { update modifiers }
    If wParam = VK_MENU Then
      { alt }
      m_alt := press
    Else
      If wParam = VK_SHIFT Then
        { shift }
        m_shift := press
      Else
        If wParam = VK_CONTROL Then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          { control }
          m_control := press;

    { enter monitor if multithreaded }
<<<<<<< HEAD
<<<<<<< HEAD
(*    if m_multithreaded then
=======
(*    If m_multithreaded Then
>>>>>>> graemeg/fixes_2_2
=======
(*    If m_multithreaded Then
>>>>>>> origin/fixes_2_2
      m_monitor.enter;*)

    uni := -1;

<<<<<<< HEAD
<<<<<<< HEAD
(*    if GetKeyboardState(@KeyStateArray) then
    begin
      scancode := (lParam shr 16) and $FF;
      {todo: ToUnicode (Windows NT)}
      tmp := ToAscii(wParam, scancode, @KeyStateArray, @AsciiBuf, 0);
      if (tmp = 1) or (tmp = 2) then
      begin
        if tmp = 2 then
        begin
//          Writeln('[', AsciiBuf, ']'); {???? todo: dead keys ????}
        end
        else
        begin
//          Write(Chr(AsciiBuf));
          {todo: codepage -> unicode}
          if AsciiBuf <= 126 then
            uni := AsciiBuf;
        end;

      end;
    End;*)

    { handle key repeat count }
    for i := 1 to lParam and $FFFF do
=======
=======
>>>>>>> origin/fixes_2_2
(*    If GetKeyboardState(@KeyStateArray) Then
    Begin
      scancode := (lParam Shr 16) And $FF;
      {todo: ToUnicode (Windows NT)}
      tmp := ToAscii(wParam, scancode, @KeyStateArray, @AsciiBuf, 0);
      If (tmp = 1) Or (tmp = 2) Then
      Begin
        If tmp = 2 Then
        Begin
//          Writeln('[', AsciiBuf, ']'); {???? todo: dead keys ????}
        End
        Else
        Begin
//          Write(Chr(AsciiBuf));
          {todo: codepage -> unicode}
          If AsciiBuf <= 126 Then
            uni := AsciiBuf;
        End;

      End;
    End;*)

    { handle key repeat count }
    For i := 1 To lParam And $FFFF Do
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      { create and insert key object }
      FEventQueue.AddEvent(TPTCKeyEvent.Create(wParam, uni, m_alt, m_shift, m_control, press));

    { check multithreaded flag }
<<<<<<< HEAD
<<<<<<< HEAD
(*    if m_multithreaded then
    begin
=======
(*    If m_multithreaded Then
    Begin
>>>>>>> graemeg/fixes_2_2
=======
(*    If m_multithreaded Then
    Begin
>>>>>>> origin/fixes_2_2
      { set event }
      m_event._set;

      { leave monitor }
      m_monitor.leave;
    End;*)
<<<<<<< HEAD
<<<<<<< HEAD
  end;
end;
=======
  End;
End;
>>>>>>> graemeg/fixes_2_2
=======
  End;
End;
>>>>>>> origin/fixes_2_2
