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

{Function Hermes_ClearerInstance : THermesHandle;
Procedure Hermes_ClearerReturn(handle : THermesHandle);
Function Hermes_ClearerRequest(handle : THermesHandle; format : PHermesFormat) : Boolean;
Function Hermes_ClearerClear(handle : THermesHandle; pixels : Pointer;
                             x1, y1, width, height, pitch : Integer;
                             r, g, b : int32; index : char8) : Boolean;}

Type
  PClearerInstance = ^TClearerInstance;
  TClearerInstance = Record
    format : PHermesFormat;
    func : THermesClearPtr;
  End;

Const
{ClearerList is a list of TClearerInstance}
  ClearerList : PHermesList = Nil;
  CLEARrefcount : Integer = 0;
  ClearCurrenthandle : THermesHandle = 0;

Function Hermes_ClearerInstance : THermesHandle;

Var
  element : PHermesListElement;
  newinstance : PClearerInstance;

Begin
  If CLEARrefcount = 0 Then
  Begin
    ClearerList := Hermes_ListNew;
    If ClearerList = Nil Then
    Begin
      Hermes_ClearerInstance := 0;
      Exit;
    End;
  End;
  element := Hermes_ListElementNew(ClearCurrenthandle + 1);
  If element = Nil Then
  Begin
    Hermes_ClearerInstance := 0;
    Exit;
  End;
  newinstance := malloc(SizeOf(TClearerInstance));
  If newinstance = Nil Then
  Begin
    Hermes_ClearerInstance := 0;
    Exit;
  End;
  newinstance^.func := Nil;
  newinstance^.format := Hermes_FormatNewEmpty;
  If newinstance^.format = Nil Then
  Begin
    Hermes_ClearerInstance := 0;
    Exit;
  End;
  element^.data := newinstance;
  Hermes_ListAdd(ClearerList, element);
  Inc(CLEARrefcount);
  Inc(ClearCurrenthandle);
  Hermes_ClearerInstance := ClearCurrenthandle;
End;

Procedure Hermes_ClearerFreeHandleCallback(q : Pointer);

Begin
  free(PClearerInstance(q)^.format);
End;

Procedure Hermes_ClearerReturn(handle : THermesHandle);

Var
  element : PHermesListElement;
  instance : PClearerInstance;

Begin
  Dec(CLEARrefcount);
  If Hermes_ListDeleteElement(ClearerList, handle, @Hermes_ClearerFreeHandleCallback) = False Then
    Exit;
  If CLEARrefcount = 0 Then
  Begin
    { Dirty fix: Free the format pointers in all the clearer instances }
    { The list functions need updating to allow member deletion! }
    element := ClearerList^.first;
    While element <> Nil Do
    Begin
      instance := element^.data;
      free(instance^.format);
      element := element^.next;
    End;
    Hermes_ListDestroy(ClearerList);
  End;
End;

Function Hermes_ClearerRequest(handle : THermesHandle; format : PHermesFormat) : Boolean;

Var
  element : PHermesListElement;
  clr : PClearerInstance;
  i : Integer;

Begin
  { Look up this clearer in the list }
  element := Hermes_ListLookup(ClearerList, handle);
  If element = Nil Then
  Begin
    Hermes_ClearerRequest := False;
    Exit;
  End;
  clr := element^.data;

  { If the clearer is the same, return 1 }
  If Hermes_FormatEquals(clr^.format, format) Then
  Begin
    Hermes_ClearerRequest := True;
    Exit;
  End;

  { Otherwise look for a new clearer }
  clr^.func := Nil;
  For i := 0 To numClearers - 1 Do
  Begin
    If Clearers[i]^.bits = format^.bits Then
    Begin
      clr^.func := Clearers[i]^.func;
      Hermes_FormatCopy(format, clr^.format);
      Hermes_ClearerRequest := True;
      Exit;
    End;
  End;
  Hermes_ClearerRequest := False;
End;

Function Hermes_ClearerClear(handle : THermesHandle; pixels : Pointer;
                             x1, y1, width, height, pitch : Integer;
                             r, g, b : int32; index : char8) : Boolean;

Var
  element : PHermesListElement;
  info : THermesGenericInfo;
  clr : PClearerInstance;
  pixelval, d_r, d_g, d_b, d_a : int32;
  iface : THermesClearInterface;

Begin
  If (height <= 0) Or (width <= 0) Then
  Begin
    Hermes_ClearerClear := True;
    Exit;
  End;

  { Look up this clearer in the list }
  element := Hermes_ListLookup(ClearerList, handle);
  If (element = Nil) Or (element^.data = Nil) Then
  Begin
    Hermes_ClearerClear := False;
    Exit;
  End;

  { Get clearer instance from list element data }
  clr := element^.data;

  { No conversion function assigned }
  If clr^.func = Nil Then
  Begin
    Hermes_ClearerClear := False;
    Exit;
  End;

  If clr^.format^.indexed Then
    pixelval := index
  Else
  Begin
    Hermes_Calculate_Generic_Info(24, 16, 8, 32,
                                  Hermes_Topbit(clr^.format^.r),
                                  Hermes_Topbit(clr^.format^.g),
                                  Hermes_Topbit(clr^.format^.b),
                                  Hermes_Topbit(clr^.format^.a), @info);
    pixelval := (index Shl 24) Or (r Shl 16) Or (g Shl 8) Or b;
    d_r := ((pixelval Shr info.r_right) Shl info.r_left) And clr^.format^.r;
    d_g := ((pixelval Shr info.g_right) Shl info.g_left) And clr^.format^.g;
    d_b := ((pixelval Shr info.b_right) Shl info.b_left) And clr^.format^.b;
    d_a := ((pixelval Shr info.a_right) Shl info.a_left) And clr^.format^.a;
    pixelval := d_r Or d_g Or d_b Or d_a;
  End;
  iface.dest := pixels;
  Inc(iface.dest, y1*pitch + x1*(clr^.format^.bits Shr 3));

  iface.width := width;
  iface.height := height;
  iface.add := pitch - width * (clr^.format^.bits Shr 3);

  iface.value := pixelval;

  { Optimization }
  If iface.add = 0 Then
  Begin
    iface.width := iface.width * iface.height;
    iface.height := 1;
  End;

  clr^.func(@iface);
  Hermes_ClearerClear := True;
End;
