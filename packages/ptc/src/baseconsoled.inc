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
  TPTCBaseConsole=Class(TPTCBaseSurface)
  Private
    FReleaseEnabled : Boolean;
    Function GetPages : Integer; Virtual; Abstract;
    Function GetName : String; Virtual; Abstract;
    Function GetTitle : String; Virtual; Abstract;
    Function GetInformation : String; Virtual; Abstract;
  Public
    Constructor Create; Virtual;
    Procedure Configure(Const AFileName : String); Virtual; Abstract;
    Function Modes : PPTCMode; Virtual; Abstract;
    Procedure Open(Const ATitle : String; APages : Integer = 0); Overload; Virtual; Abstract;
    Procedure Open(Const ATitle : String; Const AFormat : TPTCFormat;
                   APages : Integer = 0); Overload; Virtual; Abstract;
    Procedure Open(Const ATitle : String; AWidth, AHeight : Integer;
                   Const AFormat : TPTCFormat; APages : Integer = 0); Overload; Virtual; Abstract;
    Procedure Open(Const ATitle : String; Const AMode : TPTCMode;
                   APages : Integer = 0); Overload; Virtual; Abstract;
    Procedure Close; Virtual; Abstract;
    Procedure Flush; Virtual; Abstract;
    Procedure Finish; Virtual; Abstract;
    Procedure Update; Virtual; Abstract;
    Procedure Update(Const AArea : TPTCArea); Virtual; Abstract;

    { event handling }
    Function NextEvent(Var AEvent : TPTCEvent; AWait : Boolean; Const AEventMask : TPTCEventMask) : Boolean; Virtual; Abstract;
    Function PeekEvent(AWait : Boolean; Const AEventMask : TPTCEventMask) : TPTCEvent; Virtual; Abstract;

    { key handling }
    Function KeyPressed : Boolean;
    Function PeekKey(Var AKey : TPTCKeyEvent) : Boolean;
    Procedure ReadKey(Var AKey : TPTCKeyEvent);
    Procedure ReadKey;
    Property KeyReleaseEnabled : Boolean Read FReleaseEnabled Write FReleaseEnabled;

    Property Pages : Integer Read GetPages;
    Property Name : String Read GetName;
    Property Title : String Read GetTitle;
    Property Information : String Read GetInformation;
  End;
