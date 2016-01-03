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

{Function TPTCExposeEvent.GetType : TPTCEventType;

Begin
  Result := PTCExposeEvent;
End;}

Type
  PEventLinkedList = ^TEventLinkedList;
  TEventLinkedList = Record
    Event : TPTCEvent;
    Next : PEventLinkedList;
  End;
  TEventQueue = Class(TObject)
  Private
    FHead, FTail : PEventLinkedList;
  Public
    Constructor Create;
    Destructor Destroy; Override;
    Procedure AddEvent(event : TPTCEvent);
    Function PeekEvent(Const EventMask : TPTCEventMask) : TPTCEvent;
    Function NextEvent(Const EventMask : TPTCEventMask) : TPTCEvent;
  End;

Constructor TEventQueue.Create;

Begin
  FHead := Nil;
  FTail := Nil;
End;

Destructor TEventQueue.Destroy;

Var
  p, pnext : PEventLinkedList;

Begin
  p := FHead;
  While p <> Nil Do
  Begin
    FreeAndNil(p^.Event);
    pnext := p^.Next;
    Dispose(p);
    p := pnext;
  End;
  Inherited Destroy;
End;

Procedure TEventQueue.AddEvent(event : TPTCEvent);

Var
  tmp : PEventLinkedList;

Begin
  New(tmp);
  FillChar(tmp^, SizeOf(tmp^), 0);
  tmp^.Next := Nil;
  tmp^.Event := event;

  If FTail <> Nil Then
  Begin
    FTail^.Next := tmp;
    FTail := tmp;
  End
  Else
  Begin { FTail = Nil }
    FHead := tmp;
    FTail := tmp;
  End;
End;

Function TEventQueue.PeekEvent(Const EventMask : TPTCEventMask) : TPTCEvent;

Var
  p : PEventLinkedList;

Begin
  p := FHead;
  While p <> Nil Do
  Begin
    If p^.Event.EventType In EventMask Then
    Begin
      Result := p^.Event;
      Exit;
    End;
    p := p^.Next;
  End;

  Result := Nil;
End;

Function TEventQueue.NextEvent(Const EventMask : TPTCEventMask) : TPTCEvent;

Var
  prev, p : PEventLinkedList;

Begin
  prev := Nil;
  p := FHead;
  While p <> Nil Do
  Begin
    If p^.Event.EventType In EventMask Then
    Begin
      Result := p^.Event;

      { delete the element from the linked list }
      If prev <> Nil Then
        prev^.Next := p^.Next
      Else
        FHead := p^.Next;
      If p^.Next = Nil Then
        FTail := prev;
      Dispose(p);

      Exit;
    End;
    prev := p;
    p := p^.Next;
  End;

  Result := Nil;
End;
