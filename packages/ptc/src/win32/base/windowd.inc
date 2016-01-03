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
  TWin32Window = class
  private
    FWindow: HWND;
    FEvent: THANDLE;
    FThread: THANDLE;
    FID: DWord;
    FName: AnsiString;
    FTitle: AnsiString;
    FExtra: DWord;
    FStyle: DWord;
    FShow: Integer;
    FX, FY: Integer;
    FWidth, FHeight: Integer;
    FData: Pointer;
    FManaged: Boolean;
    FMultithreaded: Boolean;
    FCursorConfineInEffect: Boolean;

{    class function WndProcSingleThreaded(hWnd: HWND; message: UINT; wParam: WPARAM; lParam: LPARAM): LRESULT; StdCall;
    class function WndProcMultiThreaded(hWnd: HWND; message: UINT; wParam: WPARAM; lParam: LPARAM): LRESULT; StdCall;}
    procedure internal_create(const AWndClass, ATitle: string; AExtra, AStyle: DWord;
                              AShow, AX, AY, AWidth, AHeight: Integer; ACenter, AMultithreaded: Boolean;
			      AData: Pointer);

    procedure Defaults;
    procedure Close;
    function GetThread: DWord;
    class procedure ThreadFunction(AOwner: TWin32Window);
  public
    constructor Create(window: HWND);
    constructor Create(const AWndClass, ATitle: string; AExtra, AStyle: DWord;
                       AShow, AX, AY, AWidth, AHeight: Integer; ACenter, AMultithreaded: Boolean;
		       AData: Pointer = nil);
    destructor Destroy; override;
    procedure Cursor(AFlag: Boolean);
    procedure ConfineCursor(AFlag: Boolean);
    procedure Resize(AWidth, AHeight: Integer);
    procedure Update(AForce: Boolean = False; AWaitForMessage: Boolean = False);
    property Handle: HWND read FWindow;
    property Thread: DWord read GetThread;
    property Managed: Boolean read FManaged;
    property Multithreaded: Boolean read FMultithreaded;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TWin32Window = Class(TObject)
  Private
    Procedure internal_create(wndclass, title : String; extra, style : DWord; show, x, y, width, height : Integer; center, _multithreaded : Boolean; data : Pointer);

    Procedure defaults;
    Procedure close;
    Class Procedure ThreadFunction(owner : TWin32Window);
{    Class Function WndProcSingleThreaded(hWnd : HWND; message : UINT; wParam : WPARAM; lParam : LPARAM) : LRESULT; StdCall;
    Class Function WndProcMultiThreaded(hWnd : HWND; message : UINT; wParam : WPARAM; lParam : LPARAM) : LRESULT; StdCall;}
    m_window : HWND;
    m_event : THANDLE;
    m_thread : THANDLE;
    m_id : DWord;
    m_name : AnsiString;
    m_title : AnsiString;
    m_extra : DWord;
    m_style : DWord;
    m_show : Integer;
    m_x, m_y : Integer;
    m_width, m_height : Integer;
    m_data : Pointer;
    m_managed : Boolean;
    m_multithreaded : Boolean;
  Public
    Constructor Create(window : HWND);
    Constructor Create(wndclass, title : String; extra, style : DWord; show, x, y, width, height : Integer; center, _multithreaded : Boolean; data : Pointer);
    Constructor Create(wndclass, title : String; extra, style : DWord; show, x, y, width, height : Integer; center, _multithreaded : Boolean);
    Destructor Destroy; Override;
    Procedure cursor(flag : Boolean);
    Procedure resize(width, height : Integer);
    Procedure update(force : Boolean);
    Procedure update; {force = False}
    Function handle : HWND;
    Function thread : DWord;
    Function managed : Boolean;
    Function multithreaded : Boolean;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
