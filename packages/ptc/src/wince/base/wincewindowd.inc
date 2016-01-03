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
  TWinCEWndProc = Function(Ahwnd: HWND; AuMsg: UINT; AwParam: WPARAM; AlParam: LPARAM): LRESULT of Object;

  TWinCEWindow = Class(TObject)
  private
    FWindow: HWND;
    FClassName: WideString;
    FClassHInstance: HINST;
  public
    constructor Create(const AClassName, ATitle: WideString;
                       AExStyle, AStyle: DWord;
                       AShow, AX, AY, AWidth, AHeight: Integer;
                       AWndProc: TWinCEWndProc;
                       AData: Pointer = Nil);
    destructor Destroy; override;

    procedure Close;
    procedure Update;

    property WindowHandle: HWND read FWindow;
  end;
=======
=======
>>>>>>> origin/fixes_2_2
Type
  TWinCEWndProc = Function(Ahwnd : HWND; AuMsg : UINT; AwParam : WPARAM; AlParam : LPARAM) : LRESULT Of Object;

  TWinCEWindow = Class(TObject)
  Private
    FWindow : HWND;
    FClassName : WideString;
    FClassHInstance : HINST;
  Public
    Constructor Create(Const AClassName, ATitle : WideString;
                       AExStyle, AStyle : DWord;
                       AShow, AX, AY, AWidth, AHeight : Integer;
                       AWndProc : TWinCEWndProc;
                       AData : Pointer = Nil);
    Destructor Destroy; Override;

    Procedure Close;
    Procedure Update;

    Property WindowHandle : HWND Read FWindow;
  End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
