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

<<<<<<< HEAD
<<<<<<< HEAD
type
  TWin32Keyboard = class(TWin32Hook)
  private
    { data }
    FMultithreaded: Boolean;
    FEvent: TWin32Event;
    FMonitor: TWin32Monitor;
    FEventQueue: TEventQueue;

    { flag data }
    FEnabled: Boolean;

    { modifiers }
    FAlt: Boolean;
    FShift: Boolean;
    FControl: Boolean;

    { window procedure }
    function WndProc(hWnd: HWND; message: DWord; wParam: WPARAM; lParam: LPARAM): LRESULT; override;
  public
    { setup }
    constructor Create(AWindow: HWND; AThread: DWord; AMultithreaded: Boolean; AEventQueue: TEventQueue);
    destructor Destroy; override;

    { control }
    procedure Enable;
    procedure Disable;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TWin32Keyboard = Class(TWin32Hook)
  Private
    { window procedure }
    Function WndProc(hWnd : HWND; message : DWord; wParam : WPARAM; lParam : LPARAM) : LRESULT; Override;

    { internal key functions }
{    Procedure insert(_key : TPTCKeyEvent);
    Function remove : TPTCKeyEvent;
    Function ready : Boolean;}

    { data }
{    m_key : Boolean;}
    m_multithreaded : Boolean;
    m_event : TWin32Event;
    m_monitor : TWin32Monitor;
    FEventQueue : TEventQueue;

    { flag data }
    m_enabled : Boolean;

    { modifiers }
    m_alt : Boolean;
    m_shift : Boolean;
    m_control : Boolean;

    { key buffer }
{    m_head : Integer;
    m_tail : Integer;
    m_buffer : Array[0..1023] Of TPTCKeyEvent;}
  Public
    { setup }
    Constructor Create(window : HWND; thread : DWord; multithreaded : Boolean; EventQueue : TEventQueue);
    Destructor Destroy; Override;

    { input }
{    Function internal_PeekKey(window : TWin32Window; k : TPTCKeyEvent) : Boolean;
    Procedure internal_ReadKey(window : TWin32Window; k : TPTCKeyEvent);}

    { control }
    Procedure enable;
    Procedure disable;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
