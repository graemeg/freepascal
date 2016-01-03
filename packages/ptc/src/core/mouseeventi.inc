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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
  TPTCMouseEvent = class(TPTCEvent, IPTCMouseEvent)
  private
    FX, FY: Integer;
    FDeltaX, FDeltaY: Integer;
    FButtonState: TPTCMouseButtonState;
    function GetX: Integer;
    function GetY: Integer;
    function GetDeltaX: Integer;
    function GetDeltaY: Integer;
    function GetButtonState: TPTCMouseButtonState;
  protected
    function GetEventType: TPTCEventType; override;
  public
    constructor Create(AX, AY, ADeltaX, ADeltaY: Integer; AButtonState: TPTCMouseButtonState);
{    property X: Integer read GetX;
    property Y: Integer read GetY;
    property DeltaX: Integer read GetDeltaX;
    property DeltaY: Integer read GetDeltaY;
    property ButtonState: TPTCMouseButtonState read GetButtonState;}
  end;
  TPTCMouseButtonEvent = class(TPTCMouseEvent, IPTCMouseButtonEvent)
  private
    FPress: Boolean;
    FButton: TPTCMouseButton;
    function GetPress: Boolean;
    function GetRelease: Boolean;
    function GetButton: TPTCMouseButton;
  public
    constructor Create(AX, AY, ADeltaX, ADeltaY: Integer; AButtonState: TPTCMouseButtonState; APress: Boolean; AButton: TPTCMouseButton);
{    property Press: Boolean read GetPress;
    property Release: Boolean read GetRelease;
    property Button: TPTCMouseButton read GetButton;}
  end;

class function TPTCMouseEventFactory.CreateNew(AX, AY, ADeltaX, ADeltaY: Integer; AButtonState: TPTCMouseButtonState): IPTCMouseEvent;
begin
  Result := TPTCMouseEvent.Create(AX, AY, ADeltaX, ADeltaY, AButtonState);
end;

class function TPTCMouseButtonEventFactory.CreateNew(AX, AY, ADeltaX, ADeltaY: Integer; AButtonState: TPTCMouseButtonState; APress: Boolean; AButton: TPTCMouseButton): IPTCMouseButtonEvent;
begin
  Result := TPTCMouseButtonEvent.Create(AX, AY, ADeltaX, ADeltaY, AButtonState, APress, AButton);
end;

function TPTCMouseEvent.GetEventType: TPTCEventType;
=======
function TPTCMouseEvent.GetType: TPTCEventType;
>>>>>>> graemeg/cpstrnew
=======
function TPTCMouseEvent.GetType: TPTCEventType;
>>>>>>> graemeg/cpstrnew
=======
function TPTCMouseEvent.GetType: TPTCEventType;
>>>>>>> graemeg/cpstrnew
=======
function TPTCMouseEvent.GetType: TPTCEventType;
>>>>>>> origin/cpstrnew
begin
  Result := PTCMouseEvent;
end;

constructor TPTCMouseEvent.Create(AX, AY, ADeltaX, ADeltaY: Integer; AButtonState: TPTCMouseButtonState);
begin
  FX := AX;
  FY := AY;
  FDeltaX := ADeltaX;
  FDeltaY := ADeltaY;
  FButtonState := AButtonState;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCMouseEvent.GetX: Integer;
begin
  Result := FX;
end;

function TPTCMouseEvent.GetY: Integer;
begin
  Result := FY;
end;

function TPTCMouseEvent.GetDeltaX: Integer;
begin
  Result := FDeltaX;
end;

function TPTCMouseEvent.GetDeltaY: Integer;
begin
  Result := FDeltaY;
end;

function TPTCMouseEvent.GetButtonState: TPTCMouseButtonState;
begin
  Result := FButtonState;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
constructor TPTCMouseButtonEvent.Create(AX, AY, ADeltaX, ADeltaY: Integer; AButtonState: TPTCMouseButtonState; APress: Boolean; AButton: TPTCMouseButton);
begin
  if APress xor (AButton In AButtonState) then
    raise TPTCError.Create('Invalid ButtonState');

  inherited Create(AX, AY, ADeltaX, ADeltaY, AButtonState);

  FPress := APress;
  FButton := AButton;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCMouseButtonEvent.GetPress: Boolean;
begin
  Result := FPress;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TPTCMouseButtonEvent.GetRelease: Boolean;
begin
  Result := not FPress;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

function TPTCMouseButtonEvent.GetButton: TPTCMouseButton;
begin
  Result := FButton;
end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
