{
    Free Pascal port of the OpenPTC C++ library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2006, 2009, 2010  Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

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

constructor TDosMouse.Create(Width, Height: Integer);

begin
  FWidth := Width;
  FHeight := Height;

  FPreviousMousePositionSaved := False;

  mouse33h.InitMouse;
  FMousePresent := mouse33h.MousePresent;

  if FMousePresent then
  begin
    mouse33h.SetCursorRange(0, 0, FWidth - 1, FHeight - 1);
    mouse33h.MoveMouseTo(FWidth div 2, FHeight div 2);
  end;
end;

procedure TDosMouse.GetPendingEvents(queue: TEventQueue);

var
  X, Y: Integer;
  Left, Right, Middle: Boolean;
  PTCMouseButtonState: TPTCMouseButtonState;

  button: TPTCMouseButton;
  before, after: Boolean;
  cstate: TPTCMouseButtonState;

begin
  if not FMousePresent then
    exit;

  mouse33h.ReadMouse(X, Y, Left, Right, Middle);

  PTCMouseButtonState := [];
  if Left then
    PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton1];
  if Right then
    PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton2];
  if Middle then
    PTCMouseButtonState := PTCMouseButtonState + [PTCMouseButton3];

  if X >= FWidth - 1 then
    X := FWidth - 1;
  if Y >= FHeight - 1 then
    Y := FHeight - 1;
  if X <= 0 then
    X := 0;
  if Y <= 0 then
    Y := 0;

  { has something new happened? }
  if (not FPreviousMousePositionSaved) or
     (X <> FPreviousMouseX) or (Y <> FPreviousMouseY) or
     (PTCMouseButtonState <> FPreviousMouseButtonState) then
  begin
    if not FPreviousMousePositionSaved then
    begin
      FPreviousMouseX := X; { first DeltaX will be 0 }
      FPreviousMouseY := Y; { first DeltaY will be 0 }
      FPreviousMouseButtonState := [];
    end;

    { movement? }
    if (X <> FPreviousMouseX) or (Y <> FPreviousMouseY) then
      queue.AddEvent(TPTCMouseEvent.Create(X, Y, X - FPreviousMouseX, Y - FPreviousMouseY, FPreviousMouseButtonState));

    { button presses/releases? }
    cstate := FPreviousMouseButtonState;
    for button := Low(button) to High(button) do
    begin
      before := button In FPreviousMouseButtonState;
      after := button In PTCMouseButtonState;
      if after and (not before) then
      begin
        { button was pressed }
        cstate := cstate + [button];
        queue.AddEvent(TPTCMouseButtonEvent.Create(x, y, 0, 0, cstate, True, button));
      end
      else
        if before and (not after) then
        begin
          { button was released }
          cstate := cstate - [button];
          queue.AddEvent(TPTCMouseButtonEvent.Create(x, y, 0, 0, cstate, False, button));
        end;
    end;

    FPreviousMouseX := X;
    FPreviousMouseY := Y;
    FPreviousMouseButtonState := PTCMouseButtonState;
    FPreviousMousePositionSaved := True;
  end;
end;
