{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2006  Nikolay Nikolov (nickysn@users.sourceforge.net)
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
{todo  TPTCMouseCursor = (PTCMouseCursorDefault,
                     PTCMouseCursorAlwaysVisible,
                     PTCMouseCursorAlwaysInvisible);}
  TPTCMouseButton = (PTCMouseButton1, { left mouse button }
                     PTCMouseButton2, { right mouse button }
                     PTCMouseButton3, { middle mouse button }
                     PTCMouseButton4,
                     PTCMouseButton5);
  TPTCMouseButtonState = Set Of TPTCMouseButton;
  TPTCMouseEvent = Class(TPTCEvent)
  Private
    FX, FY : Integer;
    FDeltaX, FDeltaY : Integer;
    FButtonState : TPTCMouseButtonState;
  Protected
    Function GetType : TPTCEventType; Override;
  Public
    Constructor Create(AX, AY, ADeltaX, ADeltaY : Integer; AButtonState : TPTCMouseButtonState);
    Property X : Integer Read FX;
    Property Y : Integer Read FY;
    Property DeltaX : Integer Read FDeltaX;
    Property DeltaY : Integer Read FDeltaY;
    Property ButtonState : TPTCMouseButtonState Read FButtonState;
  End;
  TPTCMouseButtonEvent = Class(TPTCMouseEvent)
  Private
    FPress : Boolean;
    FButton : TPTCMouseButton;
    Function GetRelease : Boolean;
  Public
    Constructor Create(AX, AY, ADeltaX, ADeltaY : Integer; AButtonState : TPTCMouseButtonState; APress : Boolean; AButton : TPTCMouseButton);
    Property Press : Boolean Read FPress;
    Property Release : Boolean Read GetRelease;
    Property Button : TPTCMouseButton Read FButton;
  End;
