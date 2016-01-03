{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
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

<<<<<<< HEAD
<<<<<<< HEAD
type
  TWin32Mouse = class(TWin32Hook)
  private
    FEventQueue: TEventQueue;

    FFullScreen: Boolean;

    { the actual image area, inside the window (top left and bottom right corner) }
    FWindowX1, FWindowY1, FWindowX2, FWindowY2: Integer;
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TWin32Mouse = Class(TWin32Hook)
  Private
    { window procedure }
    Function WndProc(hWnd : HWND; message : DWord; wParam : WPARAM; lParam : LPARAM) : LRESULT; Override;

    FEventQueue : TEventQueue;

    FFullScreen : Boolean;

    { the actual image area, inside the window (top left and bottom right corner) }
    FWindowX1, FWindowY1, FWindowX2, FWindowY2 : Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    { console resolution
      - mouse cursor position as seen by the user must always be in range:
        [0..FConsoleWidth-1, 0..FConsoleHeight-1] }
<<<<<<< HEAD
<<<<<<< HEAD
    FConsoleWidth, FConsoleHeight: Integer;

    FPreviousMouseButtonState: TPTCMouseButtonState;
    FPreviousMouseX, FPreviousMouseY: Integer; { for calculating the deltas }
    FPreviousMousePositionSaved: Boolean; { true, if FPreviousMouseX,
           FPreviousMouseY and FPreviousMouseButtonState contain valid values }

    { flag data }
    FEnabled: Boolean;

    { window procedure }
    function WndProc(hWnd: HWND; message: DWord; wParam: WPARAM; lParam: LPARAM): LRESULT; override;
  public
    { setup }
    constructor Create(AWindow: HWND; AThread: DWord; AMultithreaded: Boolean; AEventQueue: TEventQueue;
        AFullScreen: Boolean; AConsoleWidth, AConsoleHeight: Integer);

    procedure SetWindowArea(AWindowX1, AWindowY1, AWindowX2, AWindowY2: Integer);

    { control }
    procedure Enable;
    procedure Disable;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
    FConsoleWidth, FConsoleHeight : Integer;

    FPreviousMouseButtonState : TPTCMouseButtonState;
    FPreviousMouseX, FPreviousMouseY : Integer; { for calculating the deltas }
    FPreviousMousePositionSaved : Boolean; { true, if FPreviousMouseX,
           FPreviousMouseY and FPreviousMouseButtonState contain valid values }

    { flag data }
    FEnabled : Boolean;
  Public
    { setup }
    Constructor Create(window : HWND; thread : DWord; multithreaded : Boolean; EventQueue : TEventQueue; FullScreen : Boolean; ConsoleWidth, ConsoleHeight : Integer);

    Procedure SetWindowArea(WindowX1, WindowY1, WindowX2, WindowY2 : Integer);

    { control }
    Procedure enable;
    Procedure disable;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
