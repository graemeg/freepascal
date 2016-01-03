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

unit garrayutils;

interface

const MaxDepth=60;
const InsertSortThreshold=16;

{TCompare is comparing class, which should have class method c(a,b:TValue):boolean, which returns true if a is less than b}
type
  generic TOrderingArrayUtils<TArr, Tvalue, TCompare>=class
  private
    class procedure Sortrange(var Arr:TArr; Start,Fin,d:SizeUInt);
    class procedure HeapSort(var Arr:TArr; Start,Fin:SizeUInt);
    class procedure InsertSort(var Arr:TArr; Start,Fin:SizeUInt);
    class function Left(a:SizeUInt):SizeUInt;inline;
    class function Right(a:SizeUInt):SizeUInt;inline;
    class procedure Heapify(var Arr: TArr; Position:SizeUInt; Start,Fin:SizeUInt);
    class function Parent(a:SizeUInt):SizeUInt;inline;
  public
    class procedure Sort(var Arr: TArr; size:SizeUInt);
    class function NextPermutation(var Arr: TArr; size:SizeUInt):boolean;
  end;

  generic TArrayUtils<TArr, Tvalue>=class
  public
    class procedure RandomShuffle(Arr: TArr; size: SizeUInt);
  end;

implementation

class function TOrderingArrayUtils.Left(a:SizeUInt):SizeUInt;inline;
begin
  Left:=((a+1)shl 1)-1;
end;

class function TOrderingArrayUtils.Right(a:SizeUInt):SizeUInt;inline;
begin
  Right:=(a+1) shl 1;
end;

class function TOrderingArrayUtils.Parent(a:SizeUInt):SizeUInt;inline;
begin
  Parent:=(a-1)shr 1;
end;

class procedure TOrderingArrayUtils.Heapify(var Arr: TArr; Position:SizeUInt; Start,Fin:SizeUInt);
var mpos,l,r:SizeUInt; temp:TValue;
begin
  while(true) do 
  begin
    mpos:=Position;
    l:=Left(Position-Start)+Start;
    r:=Right(Position-Start)+Start;
    if (l<Fin) AND (TCompare.c(Arr[mpos],Arr[l])) then
      mpos:=l;
    if (r<Fin) AND (TCompare.c(Arr[mpos],Arr[r])) then
      mpos:=r;
    if mpos = Position then break;
    
    temp:=Arr[Position];
    Arr[Position]:=Arr[mpos];
    Arr[mpos]:=temp;
    Position:=mpos;
  end;
end;

class procedure TOrderingArrayUtils.Sort(var Arr:TArr; size:SizeUInt);inline;
begin
  Sortrange(Arr,0,size,0);
  InsertSort(Arr,0,size);
end;

class procedure TOrderingArrayUtils.Sortrange(var Arr:TArr; Start,Fin,d:SizeUInt);
var pivot,temp:Tvalue; i,j,k,l:SizeUInt;
begin
  if (Fin-Start) <= InsertSortThreshold then
  begin
    InsertSort(Arr,Start,Fin);
    exit;
  end;
  if d>=maxdepth then
  begin
    HeapSort(Arr, Start, Fin);
    exit;
  end;
{median of 3} 
  j:=Start;
  k:=Fin-1;
  l:=(Start+Fin)div 2;
  if(TCompare.c(Arr[j],Arr[k])) and (TCompare.c(Arr[j],Arr[l])) then
  begin
    if(TCompare.c(Arr[k],Arr[l])) then
    begin
      temp:=Arr[k];
      Arr[k]:=Arr[j];
      Arr[j]:=temp;
    end else 
    begin
      temp:=Arr[l];
      Arr[l]:=Arr[j];
      Arr[j]:=temp;
    end;
  end
  else if(TCompare.c(Arr[k],Arr[j])) and (TCompare.c(Arr[l],Arr[j])) then
  begin
    if(TCompare.c(Arr[l],Arr[k])) then
    begin
      temp:=Arr[k];
      Arr[k]:=Arr[j];
      Arr[j]:=temp;
    end else
    begin
      temp:=Arr[l];
      Arr[l]:=Arr[j];
      Arr[j]:=temp;
    end;
  end;

{partition} 
  pivot:=Arr[Start];

  i:=Start-1;
  j:=Fin;
  repeat 
    repeat
      dec(j);
    until (not (TCompare.c(pivot,Arr[j])));
   
    
    repeat
      inc(i);
    until (not (TCompare.c(Arr[i],pivot)));
    if(i < j) then
    begin
      temp:=Arr[i];
      Arr[i]:=Arr[j];
      Arr[j]:=temp;
    end;
  until (i>=j);

  Sortrange(Arr, Start, j+1, d+1);
  Sortrange(Arr, j+1, Fin, d+1);
end;

class procedure TOrderingArrayUtils.InsertSort(var Arr:TArr; Start,Fin:SizeUInt);inline;
var i,j:SizeUInt; temp:Tvalue;
begin
  for i:=Start+1 to Fin-1 do
  begin
    j:=i;
    temp:=Arr[i];
    while (j>0) and (TCompare.c(temp,Arr[j-1])) do
    begin
      Arr[j]:=Arr[j-1];
      dec(j);
    end;
    Arr[j]:=temp;
  end;
end;

class procedure TOrderingArrayUtils.HeapSort(var Arr: TArr; Start,Fin:SizeUInt);
var i,cur,next,l,r,size:SizeUInt; temp:Tvalue;
begin
{buildHeap}
  size:=Fin-Start;
  for i:=((size div 2)-1) downto 0 do 
    Heapify(Arr, i+Start, Start, Fin);
{bottomup HeapSort}
  for i:=size-1 downto 1 do
  begin
    Fin:=Fin-1;
    cur:=Start;
    temp:=Arr[Start];
    while(true) do
    begin
      l:=Left(cur-Start)+Start;
      if l>=Fin then 
        break;
      next:=l;
      r:=Right(cur-Start)+Start;
      if (r<Fin) AND (TCompare.c(Arr[l],Arr[r])) then
        next:=r;
      Arr[cur]:=Arr[next];
      cur:=next;
    end;
    Arr[cur]:=temp;
    temp:=Arr[i+Start];
    Arr[i+Start]:=Arr[cur];
    Arr[cur]:=temp;
    l:=Parent(cur-Start)+Start;
    while (cur <> 0) AND (TCompare.c(Arr[l],Arr[cur])) do
    begin
      temp:=Arr[cur];
      Arr[cur]:=Arr[l];
      Arr[l]:=temp;
      cur:=l;
      l:=Parent(cur-Start)+Start;
    end;
  end;
end;

class function TOrderingArrayUtils.NextPermutation(var Arr: TArr; size: SizeUInt):boolean;
var i,f:SizeUInt; temp:TValue;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  f := size;
=======
  f := -1;
>>>>>>> graemeg/cpstrnew
=======
  f := -1;
>>>>>>> graemeg/cpstrnew
=======
  f := -1;
>>>>>>> graemeg/cpstrnew
=======
  f := -1;
>>>>>>> origin/cpstrnew
  for i:=size-1 downto 1 do begin 
    if (TCompare.c(arr[i-1], arr[i])) then begin
      f := i-1;
      break;
    end;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if f = size then exit(false);
=======
  if f = -1 then exit(false);
>>>>>>> graemeg/cpstrnew
=======
  if f = -1 then exit(false);
>>>>>>> graemeg/cpstrnew
=======
  if f = -1 then exit(false);
>>>>>>> graemeg/cpstrnew
=======
  if f = -1 then exit(false);
>>>>>>> origin/cpstrnew
  for i:=size-1 downto 1 do begin
    if (TCompare.c(arr[f], arr[i])) then begin
      temp:=arr[f]; arr[f] := arr[i]; arr[i] := temp;
      break;
    end;
  end;
  i:= size-1;
  inc(f);
  while (i > f) do begin
    temp:=arr[f]; arr[f] := arr[i]; arr[i] := temp;
    dec(i); inc(f);
  end;
  NextPermutation := true;
end;

class procedure TArrayUtils.RandomShuffle(Arr: TArr; size: SizeUInt);
var i,r:SizeUInt; temp:Tvalue;
begin
  for i:=size-1 downto 1 do begin
    r:=random(Int64(i));
    temp:=Arr[r];
    Arr[r]:=Arr[i];
    Arr[i]:=temp;
  end;
end;


end.
