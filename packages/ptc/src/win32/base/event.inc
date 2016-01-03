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
constructor TWin32Event.Create;
begin
  { create event handle }
  FEvent := CreateEvent(nil, True, False, nil);

  { check event handle }
  if FEvent = 0 then
    raise TPTCError.Create('could not create event');
end;

destructor TWin32Event.Destroy;
begin
  { close handle }
  CloseHandle(FEvent);

  inherited Destroy;
end;

procedure TWin32Event.SetEvent;
begin
  { set event }
  windows.SetEvent(FEvent);
end;

procedure TWin32Event.ResetEvent;
begin
  { reset event }
  windows.ResetEvent(FEvent);
end;

procedure TWin32Event.WaitForEvent;
begin
  { wait for event }
  WaitForSingleObject(FEvent, INFINITE);
end;
=======
=======
>>>>>>> origin/fixes_2_2
Constructor TWin32Event.Create;

Begin
  { create event handle }
  m_event := CreateEvent(Nil, True, False, Nil);

  { check event handle }
  If m_event = 0 Then
    Raise TPTCError.Create('could not create event');
End;

Destructor TWin32Event.Destroy;

Begin
  { close handle }
  CloseHandle(m_event);

  Inherited Destroy;
End;

Procedure TWin32Event._set;

Begin
  { set event }
  SetEvent(m_event);
End;

Procedure TWin32Event.reset;

Begin
  { reset event }
  ResetEvent(m_event);
End;

Procedure TWin32Event.wait;

Begin
  { wait for event }
  WaitForSingleObject(m_event, INFINITE);
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
