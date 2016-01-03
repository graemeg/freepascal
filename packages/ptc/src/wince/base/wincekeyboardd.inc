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
type
  TWinCEKeyboard = Class(TObject)
  private
    { data }
    FEventQueue: TEventQueue;

    { flag data }
    m_enabled: Boolean;

    { modifiers }
    m_alt: Boolean;
    m_shift: Boolean;
    m_control: Boolean;
  public
    { setup }
    constructor Create(EventQueue: TEventQueue);

    { window procedure }
    function WndProc(hWnd: HWND; message: DWord; wParam: WPARAM; lParam: LPARAM): LRESULT;

    { control }
    procedure enable;
    procedure disable;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TWinCEKeyboard = Class(TObject)
  Private
    { data }
    FEventQueue : TEventQueue;

    { flag data }
    m_enabled : Boolean;

    { modifiers }
    m_alt : Boolean;
    m_shift : Boolean;
    m_control : Boolean;
  Public
    { setup }
    Constructor Create(EventQueue : TEventQueue);

    { window procedure }
    Function WndProc(hWnd : HWND; message : DWord; wParam : WPARAM; lParam : LPARAM) : LRESULT;

    { control }
    Procedure enable;
    Procedure disable;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
