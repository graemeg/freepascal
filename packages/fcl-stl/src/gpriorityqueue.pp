{
   This file is part of the Free Pascal FCL library.
   BSD parts (c) 2011 Vlado Boza

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
{$mode objfpc}

unit gpriorityqueue;

interface 

uses gvector;

{TCompare is comparing class, which should have class method c(a,b:T):boolean, which returns true is a is less than b}

type 
  generic TPriorityQueue<T, TCompare>=class
  private
  type 
    TContainer=specialize TVector<T>;
  var
    FData:TContainer;
    
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure PushUp();
=======
    procedure PushUp(position:SizeUInt);
>>>>>>> graemeg/cpstrnew
=======
    procedure PushUp(position:SizeUInt);
>>>>>>> graemeg/cpstrnew
=======
    procedure PushUp(position:SizeUInt);
>>>>>>> graemeg/cpstrnew
=======
    procedure PushUp(position:SizeUInt);
>>>>>>> origin/cpstrnew
    function Left(a:SizeUInt):SizeUInt;inline;
    function Right(a:SizeUInt):SizeUInt;inline;
    procedure Heapify(position:SizeUInt);
    function Parent(a:SizeUInt):SizeUInt;inline;
  public
    constructor Create;
    destructor Destroy;override;
    function Top:T;inline;
    procedure Pop;inline;
    procedure Push(value:T);inline;
    function Size:SizeUInt;inline;
    function IsEmpty:boolean;inline;
  end;

implementation

constructor TPriorityQueue.Create;
begin
  FData:=TContainer.Create;
end;

destructor TPriorityQueue.Destroy;
begin;
  FData.Destroy;
end;

function TPriorityQueue.Size:SizeUInt;inline;
begin
  Size:=FData.Size;
end;

function TPriorityQueue.IsEmpty:boolean;inline;
begin
  IsEmpty:=FData.Size=0;
end;

function TPriorityQueue.Top:T;inline;
begin
  Top:=FData[0];
end;

procedure TPriorityQueue.Pop;inline;
begin
  if not IsEmpty then begin
    FData[0]:=FData.back;
    FData.PopBack;
    Heapify(0);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPriorityQueue.PushUp();
var position,np:SizeUInt; temp:T;
begin
  position:=FData.Size-1;
=======
procedure TPriorityQueue.PushUp(position:SizeUInt);
var np:SizeUInt; temp:T;
begin
>>>>>>> graemeg/cpstrnew
=======
procedure TPriorityQueue.PushUp(position:SizeUInt);
var np:SizeUInt; temp:T;
begin
>>>>>>> graemeg/cpstrnew
=======
procedure TPriorityQueue.PushUp(position:SizeUInt);
var np:SizeUInt; temp:T;
begin
>>>>>>> graemeg/cpstrnew
=======
procedure TPriorityQueue.PushUp(position:SizeUInt);
var np:SizeUInt; temp:T;
begin
>>>>>>> origin/cpstrnew
  while(position>0) do
  begin
    np := Parent(position);
    if(TCompare.c(FData[np],FData[position])) then
    begin
      temp:=FData[np];
      FData[np]:=FData[position];
      FData[position]:=temp;
      position:=np;
    end else
      break;
  end;
end;

procedure TPriorityQueue.Push(value:T);inline;
begin
  FData.PushBack(value);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  PushUp();
=======
  PushUp(FData.Size-1);
>>>>>>> graemeg/cpstrnew
=======
  PushUp(FData.Size-1);
>>>>>>> graemeg/cpstrnew
=======
  PushUp(FData.Size-1);
>>>>>>> graemeg/cpstrnew
=======
  PushUp(FData.Size-1);
>>>>>>> origin/cpstrnew
end;

function TPriorityQueue.Left(a:SizeUInt):SizeUInt;inline;
begin
  Left:=((a+1)shl 1)-1;
end;

function TPriorityQueue.Right(a:SizeUInt):SizeUInt;inline;
begin
  Right:=(a+1) shl 1;
end;

function TPriorityQueue.Parent(a:SizeUInt):SizeUInt;inline;
begin
  Parent:=(a-1)shr 1;
end;

procedure TPriorityQueue.Heapify(position:SizeUInt);
var mpos,l,r:SizeUInt; temp:T;
begin
  while(true) do
  begin
    mpos:=position;
    l:=Left(position);
    r:=Right(position);
    if (l<FData.Size) AND (TCompare.c(FData[mpos],FData[l])) then
      mpos:=l;
    if (r<FData.Size) AND (TCompare.c(FData[mpos],FData[r])) then
      mpos:=r;
    if mpos = position then break;

    temp:=FData[position];
    FData[position]:=FData[mpos];
    FData[mpos]:=temp;
    position:=mpos;
  end;
end;

end.
