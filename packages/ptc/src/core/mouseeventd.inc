{
    Free Pascal port of the OpenPTC C++ library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2007, 2009-2011  Nikolay Nikolov (nickysn@users.sourceforge.net)
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

type
{todo  TPTCMouseCursor = (PTCMouseCursorDefault,
                     PTCMouseCursorAlwaysVisible,
                     PTCMouseCursorAlwaysInvisible);}
  TPTCMouseButton = (PTCMouseButton1, { left mouse button }
                     PTCMouseButton2, { right mouse button }
                     PTCMouseButton3, { middle mouse button }
                     PTCMouseButton4,
                     PTCMouseButton5);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TPTCMouseButtonState = set of TPTCMouseButton;
  IPTCMouseEvent = interface(IPTCEvent)
    ['{4D093608-6F27-4578-B41E-3492A4C7FEED}']
    function GetX: Integer;
    function GetY: Integer;
    function GetDeltaX: Integer;
    function GetDeltaY: Integer;
    function GetButtonState: TPTCMouseButtonState;

    property X: Integer read GetX;
    property Y: Integer read GetY;
    property DeltaX: Integer read GetDeltaX;
    property DeltaY: Integer read GetDeltaY;
    property ButtonState: TPTCMouseButtonState read GetButtonState;
  end;
  IPTCMouseButtonEvent = interface(IPTCMouseEvent)
    ['{363B9ACC-4DEB-4031-8BD9-0B6788C6CFA7}']
    function GetPress: Boolean;
    function GetRelease: Boolean;
    function GetButton: TPTCMouseButton;

    property Press: Boolean read GetPress;
    property Release: Boolean read GetRelease;
    property Button: TPTCMouseButton read GetButton;
  end;

  TPTCMouseEventFactory = class
  public
    class function CreateNew(AX, AY, ADeltaX, ADeltaY: Integer; AButtonState: TPTCMouseButtonState): IPTCMouseEvent;
  end;
  TPTCMouseButtonEventFactory = class
  public
    class function CreateNew(AX, AY, ADeltaX, ADeltaY: Integer; AButtonState: TPTCMouseButtonState; APress: Boolean; AButton: TPTCMouseButton): IPTCMouseButtonEvent;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TPTCMouseButtonState = Set of TPTCMouseButton;
  TPTCMouseEvent = Class(TPTCEvent)
  private
    FX, FY: Integer;
    FDeltaX, FDeltaY: Integer;
    FButtonState: TPTCMouseButtonState;
  protected
    function GetType: TPTCEventType; override;
  public
    constructor Create(AX, AY, ADeltaX, ADeltaY: Integer; AButtonState: TPTCMouseButtonState);
    property X: Integer read FX;
    property Y: Integer read FY;
    property DeltaX: Integer read FDeltaX;
    property DeltaY: Integer read FDeltaY;
    property ButtonState: TPTCMouseButtonState read FButtonState;
  end;
  TPTCMouseButtonEvent = Class(TPTCMouseEvent)
  private
    FPress: Boolean;
    FButton: TPTCMouseButton;
    function GetRelease: Boolean;
  public
    constructor Create(AX, AY, ADeltaX, ADeltaY: Integer; AButtonState: TPTCMouseButtonState; APress: Boolean; AButton: TPTCMouseButton);
    property Press: Boolean read FPress;
    property Release: Boolean read GetRelease;
    property Button: TPTCMouseButton read FButton;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;
