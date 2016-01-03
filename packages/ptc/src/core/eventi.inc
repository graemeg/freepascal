{
    Free Pascal port of the OpenPTC C++ library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2003, 2006, 2007, 2009-2011  Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
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

{function TPTCExposeEvent.GetType: TPTCEventType;
begin
  Result := PTCExposeEvent;
end;}

type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TPTCEvent = class(TInterfacedObject, IPTCEvent)
  protected
    function GetEventType: TPTCEventType; virtual; abstract;
  public
    property EventType: TPTCEventType read GetEventType;
  end;
{  TPTCExposeEvent = Class(TPTCEvent)
  protected
    function GetType: TPTCEventType; override;
  End;}

  PEventLinkedList = ^TEventLinkedList;
  TEventLinkedList = record
    Event: IPTCEvent;
=======
  PEventLinkedList = ^TEventLinkedList;
  TEventLinkedList = record
    Event: TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
  PEventLinkedList = ^TEventLinkedList;
  TEventLinkedList = record
    Event: TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
  PEventLinkedList = ^TEventLinkedList;
  TEventLinkedList = record
    Event: TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
  PEventLinkedList = ^TEventLinkedList;
  TEventLinkedList = record
    Event: TPTCEvent;
>>>>>>> origin/cpstrnew
    Next: PEventLinkedList;
  end;
  TEventQueue = class
  private
    FHead, FTail: PEventLinkedList;
  public
    constructor Create;
    destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure AddEvent(const event: IPTCEvent);
    function PeekEvent(const EventMask: TPTCEventMask): IPTCEvent;
    function NextEvent(const EventMask: TPTCEventMask): IPTCEvent;
=======
    procedure AddEvent(event: TPTCEvent);
    function PeekEvent(const EventMask: TPTCEventMask): TPTCEvent;
    function NextEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
    procedure AddEvent(event: TPTCEvent);
    function PeekEvent(const EventMask: TPTCEventMask): TPTCEvent;
    function NextEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
    procedure AddEvent(event: TPTCEvent);
    function PeekEvent(const EventMask: TPTCEventMask): TPTCEvent;
    function NextEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
    procedure AddEvent(event: TPTCEvent);
    function PeekEvent(const EventMask: TPTCEventMask): TPTCEvent;
    function NextEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> origin/cpstrnew
  end;

constructor TEventQueue.Create;
begin
  FHead := nil;
  FTail := nil;
end;

destructor TEventQueue.Destroy;
var
  p, pnext: PEventLinkedList;
begin
  p := FHead;
  while p <> nil do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    p^.Event := nil;
=======
    FreeAndNil(p^.Event);
>>>>>>> graemeg/cpstrnew
=======
    FreeAndNil(p^.Event);
>>>>>>> graemeg/cpstrnew
=======
    FreeAndNil(p^.Event);
>>>>>>> graemeg/cpstrnew
=======
    FreeAndNil(p^.Event);
>>>>>>> origin/cpstrnew
    pnext := p^.Next;
    Dispose(p);
    p := pnext;
  end;
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TEventQueue.AddEvent(const event: IPTCEvent);
=======
procedure TEventQueue.AddEvent(event: TPTCEvent);
>>>>>>> graemeg/cpstrnew
=======
procedure TEventQueue.AddEvent(event: TPTCEvent);
>>>>>>> graemeg/cpstrnew
=======
procedure TEventQueue.AddEvent(event: TPTCEvent);
>>>>>>> graemeg/cpstrnew
=======
procedure TEventQueue.AddEvent(event: TPTCEvent);
>>>>>>> origin/cpstrnew
var
  tmp: PEventLinkedList;
begin
  New(tmp);
  FillChar(tmp^, SizeOf(tmp^), 0);
  tmp^.Next := nil;
  tmp^.Event := event;

  if FTail <> nil then
  begin
    FTail^.Next := tmp;
    FTail := tmp;
  end
  else
  begin { FTail = nil }
    FHead := tmp;
    FTail := tmp;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TEventQueue.PeekEvent(const EventMask: TPTCEventMask): IPTCEvent;
=======
function TEventQueue.PeekEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TEventQueue.PeekEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TEventQueue.PeekEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TEventQueue.PeekEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> origin/cpstrnew
var
  p: PEventLinkedList;
begin
  p := FHead;
  while p <> nil do
  begin
    if p^.Event.EventType In EventMask then
    begin
      Result := p^.Event;
      exit;
    end;
    p := p^.Next;
  end;

  Result := nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TEventQueue.NextEvent(const EventMask: TPTCEventMask): IPTCEvent;
=======
function TEventQueue.NextEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TEventQueue.NextEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TEventQueue.NextEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TEventQueue.NextEvent(const EventMask: TPTCEventMask): TPTCEvent;
>>>>>>> origin/cpstrnew
var
  prev, p: PEventLinkedList;
begin
  prev := nil;
  p := FHead;
  while p <> nil do
  begin
    if p^.Event.EventType In EventMask then
    begin
      Result := p^.Event;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      p^.Event := nil;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

      { delete the element from the linked list }
      if prev <> nil then
        prev^.Next := p^.Next
      else
        FHead := p^.Next;
      if p^.Next = nil then
        FTail := prev;
      Dispose(p);

      exit;
    end;
    prev := p;
    p := p^.Next;
  end;

  Result := nil;
end;
