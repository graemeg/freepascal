{
    Copyright (c) 2007 by Pierre Muller

    Common reference types

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ****************************************************************************
}

unit crefs;

{$i fpcdefs.inc}

interface

uses
  globtype,
  cclasses;

  type

   TrefItem = class (TLinkedListItem)
     refinfo  : tfileposinfo;
     constructor create(const ARefInfo : tfileposinfo);
     Function GetCopy:TLinkedListItem;virtual;reintroduce;
   end;

   TRefLinkedList = class(TLinkedList)
     procedure WriteToPPU;
   end;

implementation

constructor TRefItem.Create(const ARefInfo : tfileposinfo);
begin
  Inherited Create;
  RefInfo:=ARefInfo;
end;

Function TRefItem.GetCopy : TLinkedListItem;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Result:=TRefItem.Create(RefInfo);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var
  NR : TRefItem;
begin
  NR.Create(RefInfo);
  GetCopy:=NR;
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

procedure TRefLinkedList.WriteToPPU;
begin
end;

begin
end.
