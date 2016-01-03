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

Type
  TWin32Hook = Class(TObject)
  Private
    Procedure Add(window : HWND; thread : DWord);
    Procedure Remove(window : HWND; thread : DWord);

    m_window : HWND;
    m_thread : DWord;

    {m_hook : HHOOK;
    m_count : Integer;
    m_cached : PWin32Hook_Lookup;
    m_registry : Array[0..15] Of TWin32Hook_Lookup;
    m_monitor : TWin32Monitor;}
  Protected
    Function WndProc(hWnd : HWND; message : DWord; wParam : WPARAM; lParam : LPARAM) : LRESULT; Virtual; Abstract;
  Public
    Constructor Create(window : HWND; thread : DWord);
    Destructor Destroy; Override;
  End;
