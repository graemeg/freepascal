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
  PHermesLookupTable = ^THermesLookupTable;
  THermesLookupTable = Record
    data : ^int32;             { Actual lookup table }
    valid : Boolean;           { Is this table up to date? }
    format : THermesFormat;    { Format of lookup table }
  End;
  PHermesPalette = ^THermesPalette;
  THermesPalette = Record
    data : ^int32;             { Palette data }
    tables : PHermesList;      { Linked list of HermesLookupTables }
  End;

Const
  PaletteList : PHermesList = Nil;
  PALETTErefcount : Integer = 0;
  currenthandle : THermesHandle = 0;

{Function Hermes_PaletteGetTable(palette : THermesHandle; format : PHermesFormat) : Pointer;
Procedure Hermes_PaletteMakeLookup(lookup, palette : Pint32;
                                   format : PHermesFormat);

Function Hermes_PaletteInstance : THermesHandle;
Procedure Hermes_PaletteReturn(handle : THermesHandle);
Procedure Hermes_PaletteSet(handle : THermesHandle; palette : Pointer);
Function Hermes_PaletteGet(handle : THermesHandle) : Pointer;
Procedure Hermes_PaletteInvalidateCache(handle : THermesHandle);}

Function Hermes_PaletteInstance : THermesHandle;

Var
  newinstance : PHermesPalette;
  newelement : PHermesListElement;

Begin
  If PaletteList = Nil Then
  Begin
    PaletteList := Hermes_ListNew;
    { Could not create a new list }
    If PaletteList = Nil Then
    Begin
      Hermes_PaletteInstance := 0;
      Exit;
    End;
  End;
  { Create a new palette structure }
  newinstance := malloc(SizeOf(THermesPalette));
  If newinstance = Nil Then
  Begin
    Hermes_PaletteInstance := 0;
    Exit;
  End;
  { Create palette data }
  newinstance^.data := malloc(256*SizeOf(int32));
  If newinstance^.data = Nil Then
  Begin
    free(newinstance);
    Hermes_PaletteInstance := 0;
    Exit;
  End;
  { Create lookup table list }
  newinstance^.tables := Hermes_ListNew;
  If newinstance^.tables = Nil Then
  Begin
    free(newinstance^.data);
    free(newinstance);
    Hermes_PaletteInstance := 0;
    Exit;
  End;
  { Everything fine so far, create a new list element }
  newelement := Hermes_ListElementNew(currenthandle+1);
  If newelement = Nil Then
  Begin
    Hermes_ListDestroy(newinstance^.tables);
    free(newinstance^.data);
    free(newinstance);
    Hermes_PaletteInstance := 0;
    Exit;
  End;

  { No errors, put current palette structure into the list element and add
    that to the list }
  newelement^.data := newinstance;

  Hermes_ListAdd(PaletteList, newelement);

  Inc(PALETTErefcount);
  Inc(currenthandle);
  Hermes_PaletteInstance := currenthandle;
End;

Procedure Hermes_PaletteReturn(handle : THermesHandle);

Var
  element : PHermesListElement;
  pal : PHermesPalette;
  table : PHermesLookupTable;

Begin
  element := Hermes_ListLookup(PaletteList, handle);
  If element = Nil Then
    Exit;

  pal := element^.data;


  { Free palette data and lookup tables }
  free(pal^.data);

  element := pal^.tables^.first;
  While element <> Nil Do
  Begin
    table := element^.data;
    If (table <> Nil) And (table^.data <> Nil) Then
    Begin
      free(table^.data);
      table^.data := Nil;
    End;
    element := element^.next;
  End;
  Hermes_ListDestroy(pal^.tables);


  { Delete list element that holds this palette }
  Hermes_ListDeleteElement(PaletteList, handle);


  { Decrease reference count. If down to zero, delete palette list }
  Dec(PALETTErefcount);
  If PALETTErefcount = 0 Then
  Begin
    Hermes_ListDestroy(PaletteList);
    PaletteList := Nil;
  End;
End;

Procedure Hermes_PaletteSet(handle : THermesHandle; palette : Pointer);

Var
  element : PHermesListElement;
  pal : PHermesPalette;

Begin
{  DebugMSG('Hermes_PaletteSet('+C2Str(handle)+','+C2Str(DWord(palette))+')');}
  element := Hermes_ListLookup(PaletteList, handle);
  If element = Nil Then
    Exit;

  pal := element^.data;
  element := pal^.tables^.first;

  { Invalidate all lookup tables }

  While element <> Nil Do
  Begin
    (PHermesLookupTable(element^.data))^.valid := False;
    element := element^.next;
  End;

{  FillChar(palette^, 256*4, $7f);}
  Move(palette^, pal^.data^, 256*4);
End;

Function Hermes_PaletteGet(handle : THermesHandle) : Pointer;

Var
  element : PHermesListElement;
  pal : PHermesPalette;

Begin
  element := Hermes_ListLookup(PaletteList, handle);
  If element = Nil Then
  Begin
    Hermes_PaletteGet := Nil;
    Exit;
  End;

  pal := element^.data;
  Hermes_PaletteGet := pal^.data;
End;

Procedure Hermes_PaletteMakeLookup(lookup, palette : Pint32;
                                   format : PHermesFormat);

Var
  info : THermesGenericInfo;
  I : Integer;
  r, g, b : int32;

Begin
{  DebugMSG('Yo! Hermes_PaletteMakeLookup');}
  r := 0;
  g := 0;
  b := 0;
  If format^.indexed Then
    Exit;

  Hermes_Calculate_Generic_Info(24,16,8,32,
                                Hermes_Topbit(format^.r),
                                Hermes_Topbit(format^.g),
                                Hermes_Topbit(format^.b),
                                Hermes_Topbit(format^.a),
                                @info);

  { Optimised loop if there are no left shifts }
  If (info.r_left = 0) And (info.g_left = 0) And (info.b_left = 0) Then
    For I := 0 To 255 Do
    Begin
      r := (palette[i] Shr info.r_right) And format^.r;
      g := (palette[i] Shr info.g_right) And format^.g;
      b := (palette[i] Shr info.b_right) And format^.b;
      lookup[i] := r Or g Or b;
    End
  Else
    For I := 0 To 255 Do
    Begin
      r := ((palette[i] Shr info.r_right) Shl info.r_left) And format^.r;
      g := ((palette[i] Shr info.g_right) Shl info.g_left) And format^.g;
      b := ((palette[i] Shr info.b_right) Shl info.b_left) And format^.b;
      lookup[i] := r Or g Or b;
    End;
End;

Function Hermes_PaletteGetTable(palette : THermesHandle; format : PHermesFormat) : Pointer;

Var
  element : PHermesListElement;
  pal : PHermesPalette;
  table : PHermesLookupTable;

Begin
  element := Hermes_ListLookup(PaletteList, palette);
  If element = Nil Then
  Begin
    Hermes_PaletteGetTable := Nil;
    Exit;
  End;

  pal := element^.data;

  { Go to the first table in the list }
  element := pal^.tables^.first;

  { Search for correct table using format }
  While element <> Nil Do
  Begin
    table := element^.data;

    If Hermes_FormatEquals(@table^.format, format) Then
    Begin
      If table^.valid Then
      Begin
        Hermes_PaletteGetTable := table^.data;
        Exit;
      End;

      { Have to recreate the lookup table }
      Hermes_PaletteMakeLookup(table^.data, pal^.data, format);
      table^.valid := True;

      Hermes_PaletteGetTable := table^.data;
      Exit;
    End;

    element := element^.next;
  End;

  { Format not found, have to create a new table (need no handle) }
  table := malloc(SizeOf(THermesLookupTable));
  If table = Nil Then
  Begin
    Hermes_PaletteGetTable := Nil;
    Exit;
  End;

  table^.data := malloc(1024);
  If table^.data = Nil Then
  Begin
    Hermes_PaletteGetTable := Nil;
    Exit;
  End;

  { Create lookup table }
  Hermes_PaletteMakeLookup(table^.data, pal^.data, format);
  Hermes_FormatCopy(format, @table^.format);

  table^.valid := True;

  { Create a new list element }
  element := Hermes_ListElementNew(0);
  If element = Nil Then
  Begin
    Hermes_PaletteGetTable := Nil;
    Exit;
  End;
  element^.data := table;

  { Add to the front of the list }
  Hermes_ListAddFront(pal^.tables, element);

  { Return lookup data }
  Hermes_PaletteGetTable := table^.data;

End;

Procedure Hermes_PaletteInvalidateCache(handle : THermesHandle);

Var
  element : PHermesListElement;
  pal : PHermesPalette;

Begin
  element := Hermes_ListLookup(PaletteList, handle);
  If element = Nil Then
    Exit;

  pal := element^.data;

  element := pal^.tables^.first;

  { Invalidate all lookup tables }

  While element <> Nil Do
  Begin
    (PHermesLookupTable(element^.data))^.valid := False;
    element := element^.next;
  End;
End;
