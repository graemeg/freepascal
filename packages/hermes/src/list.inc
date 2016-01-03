{
    Free Pascal port of the Hermes C library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C version by Christian Nentwich (c.nentwich@cs.ucl.ac.uk)

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
  TListFreeCallback = Procedure(p : Pointer);
  PHermesListElementStruct = ^THermesListElementStruct;
  THermesListElementStruct = Record
    handle : THermesHandle;
    data : Pointer;
    next : PHermesListElementStruct;
  End;
  PHermesListElement = ^THermesListElement;
  THermesListElement = THermesListElementStruct;
  PHermesList = ^THermesList;
  THermesList = Record
    first, last : PHermesListElement;
  End;

{Function Hermes_ListNew : PHermesList;
Procedure Hermes_ListDestroy(list : PHermesList);
Function Hermes_ListElementNew(handle : THermesHandle) : PHermesListElement;
Procedure Hermes_ListAdd(list : PHermesList; element : PHermesListElement);
Procedure Hermes_ListAddFront(list : PHermesList; element : PHermesListElement);
Function Hermes_ListDeleteElement(list : PHermesList; handle : THermesHandle) : Boolean;
Function Hermes_ListLookup(list : PHermesList;
                           handle : THermesHandle) : PHermesListElement;}

Function Hermes_ListNew : PHermesList;

Var
  tmp : PHermesList;

Begin
{  New(tmp);}
  tmp := malloc(SizeOf(THermesList));
  If tmp = Nil Then
  Begin
    Hermes_ListNew := Nil;
    Exit;
  End;
  tmp^.first := Nil;
  tmp^.last := Nil;
  Hermes_ListNew := tmp;
End;

Procedure Hermes_ListDestroy(list : PHermesList);

Var
  tmp, run : PHermesListElement;

Begin
  If list = Nil Then
    Exit;
  run := list^.first;
  While run <> Nil Do
  Begin
    tmp := run;
    run := run^.next;
    If tmp^.data <> Nil Then
    Begin
      {to do: free(tmp->data)}
      free(tmp^.data);
    End;
    free(tmp);
  End;
  free(list);
  list := Nil;
End;

Function Hermes_ListElementNew(handle : THermesHandle) : PHermesListElement;

Var
  tmp : PHermesListElement;

Begin
  tmp := malloc(SizeOf(THermesListElement));
  If tmp = Nil Then
  Begin
    Hermes_ListElementNew := Nil;
    Exit;
  End;
  tmp^.handle := handle;
  tmp^.next := Nil;
  tmp^.data := Nil;
  Hermes_ListElementNew := tmp;
End;

Procedure Hermes_ListAdd(list : PHermesList; element : PHermesListElement);

Begin
  If (list = Nil) Or (element = Nil) Then
    Exit;
  If list^.first = Nil Then
  Begin
    list^.first := element;
    list^.last := element;
    element^.next := Nil;
    Exit;
  End;
  list^.last^.next := element;
  list^.last := element;
  element^.next := Nil;
End;

Procedure Hermes_ListAddFront(list : PHermesList; element : PHermesListElement);

Begin
  If (list = Nil) Or (element = Nil) Then
    Exit;
  If list^.first = Nil Then
  Begin
    list^.first := element;
    list^.last := element;
    element^.next := Nil;
    Exit;
  End;
  element^.next := list^.first;
  list^.first := element;
End;

Function Hermes_ListDeleteElement(list : PHermesList; handle : THermesHandle; user_free : TListFreeCallback) : Boolean;

Var
  run, previous : PHermesListElement;

Begin
  If list = Nil Then
  Begin
    Hermes_ListDeleteElement := False;
    Exit;
  End;
  previous := Nil;
  run := list^.first;
  While run <> Nil Do
  Begin
    If run^.handle = handle Then
    Begin
      If run = list^.first Then
        list^.first := run^.next
      Else
        previous^.next := run^.next;
      If run = list^.last Then
      Begin
        list^.last := previous;
        If list^.last <> Nil Then
          list^.last^.next := Nil;
      End;
      If run^.data <> Nil Then
      Begin
        If user_free <> Nil Then
        Begin
          user_free(run^.data);
        End;
        free(run^.data);
      End;
      free(run);
      Hermes_ListDeleteElement := True;
      Exit;
    End;
    previous := run;
    run := run^.next;
  End;
  Hermes_ListDeleteElement := False;
End;

Function Hermes_ListDeleteElement(list : PHermesList; handle : THermesHandle) : Boolean;

Begin
  Hermes_ListDeleteElement := Hermes_ListDeleteElement(list, handle, Nil);
End;

Function Hermes_ListLookup(list : PHermesList;
                           handle : THermesHandle) : PHermesListElement;

Var
  run : PHermesListElement;

Begin
  If list = Nil Then
  Begin
    Hermes_ListLookup := Nil;
    Exit;
  End;
  run := list^.first;
  While run <> Nil Do
  Begin
    If run^.handle = handle Then
    Begin
      Hermes_ListLookup := run;
      Exit;
    End;
    run := run^.next;
  End;
  Hermes_ListLookup := Nil;
End;
