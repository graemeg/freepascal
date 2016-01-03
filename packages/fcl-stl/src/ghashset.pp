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

unit ghashset;

interface
uses gvector, gutil, garrayutils;

const baseFDataSize = 8;

{Thash should have one class function hash(a:T, n:longint):longint which return uniformly distributed
value in range <0,n-1> base only on arguments, n will be always power of 2}

type
    generic THashSetIterator<T, TTable>=class
    public
    var
      Fh,Fp:SizeUInt;
      FData:TTable;
      function Next:boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      function GetData:T;
      property Data:T read GetData;
 end;
=======
      function GetValue:T;
  end;
>>>>>>> graemeg/cpstrnew
=======
      function GetValue:T;
  end;
>>>>>>> graemeg/cpstrnew
=======
      function GetValue:T;
  end;
>>>>>>> graemeg/cpstrnew
=======
      function GetValue:T;
  end;
>>>>>>> origin/cpstrnew

  generic THashSet<T, Thash>=class
    private 
    type 
      TContainer = specialize TVector<T>;
      TTable = specialize TVector<TContainer>;
    var 
      FData:TTable;
      FDataSize:SizeUInt; 
      procedure EnlargeTable;
    public 
    type
      TIterator = specialize THashSetIterator<T, TTable>;
      constructor create;
      destructor destroy;override;
      procedure insert(value:T);inline;
      function contains(value:T):boolean;inline;
      function size:SizeUInt;inline;
      procedure delete(value:T);inline;
      function IsEmpty:boolean;inline;

      function Iterator:TIterator;
  end;

implementation

function THashSet.Size:SizeUInt;inline;
begin
  Size:=FDataSize;
end;

destructor THashSet.Destroy;
var i:SizeUInt;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  for i:=0 to FData.size-1 do
=======
  for i:=0 to FData.size do
>>>>>>> graemeg/cpstrnew
=======
  for i:=0 to FData.size do
>>>>>>> graemeg/cpstrnew
=======
  for i:=0 to FData.size do
>>>>>>> graemeg/cpstrnew
=======
  for i:=0 to FData.size do
>>>>>>> origin/cpstrnew
    (FData[i]).Destroy;
  FData.Destroy;
end;

function THashSet.IsEmpty():boolean;inline;
begin
  if Size()=0 then 
    IsEmpty:=true
  else 
    IsEmpty:=false;
end;

procedure THashSet.EnlargeTable;
var i,j,h,oldDataSize:SizeUInt; 
    value:T;
begin
  oldDataSize:=FData.size;
  FData.resize(FData.size*2);
  for i:=oldDataSize to FData.size-1 do
    FData[i] := TContainer.create;
  for i:=oldDataSize-1 downto 0 do begin
    j := 0;
    while j < (FData[i]).size do begin
      value := (FData[i])[j];
      h:=Thash.hash(value,FData.size);
      if (h <> i) then begin
        (FData[i])[j] := (FData[i]).back;
        (FData[i]).popback;
        (FData[h]).pushback(value);
      end else
        inc(j);
    end;
  end;
end;

constructor THashSet.create;
var i:longint;
begin
  FDataSize:=0;
  FData:=TTable.create;
  FData.resize(baseFDataSize);
  for i:=0 to baseFDataSize-1 do
    FData[i]:=TContainer.create;
end;

function THashSet.contains(value:T):boolean;inline;
var i,h,bs:longint;
begin
  h:=Thash.hash(value,FData.size);
  bs:=(FData[h]).size;
  for i:=0 to bs-1 do begin
    if ((FData[h])[i]=value) then exit(true);
  end;
  exit(false);
end;

procedure THashSet.insert(value:T);inline;
begin
  if (contains(value)) then exit;
  inc(FDataSize);
  (FData[Thash.hash(value,FData.size)]).pushback(value);

  if (FDataSize > 2*FData.size) then
    EnlargeTable;
end;

procedure THashSet.delete(value:T);inline;
var h,i:SizeUInt;
begin
  h:=Thash.hash(value,FData.size);
  i:=0;
  while i < (FData[h]).size do begin
    if ((FData[h])[i]=value) then begin
      (FData[h])[i] := (FData[h]).back;
      (FData[h]).popback;
      dec(FDataSize);
      exit;
    end;
    inc(i);
  end;
end;

function THashSetIterator.Next:boolean;
begin
  inc(Fp);
  if (Fp = (FData[Fh]).size) then begin
    Fp:=0; inc(Fh);
    while Fh < FData.size do begin
      if ((FData[Fh]).size > 0) then break;
      inc(Fh);
    end;
    if (Fh = FData.size) then exit(false);
  end;
  Next := true;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function THashSetIterator.GetData:T;
begin
  GetData:=(FData[Fh])[Fp];
=======
function THashSetIterator.GetValue:T;
begin
  GetValue:=(FData[Fh])[Fp];
>>>>>>> graemeg/cpstrnew
=======
function THashSetIterator.GetValue:T;
begin
  GetValue:=(FData[Fh])[Fp];
>>>>>>> graemeg/cpstrnew
=======
function THashSetIterator.GetValue:T;
begin
  GetValue:=(FData[Fh])[Fp];
>>>>>>> graemeg/cpstrnew
=======
function THashSetIterator.GetValue:T;
begin
  GetValue:=(FData[Fh])[Fp];
>>>>>>> origin/cpstrnew
end;

function THashSet.Iterator:TIterator;
var h,p:SizeUInt;
begin
  h:=0;
  p:=0;
  while h < FData.size do begin
    if ((FData[h]).size > 0) then break;
    inc(h);
  end;
  if (h = FData.size) then exit(nil);
  Iterator := TIterator.create;
  Iterator.Fh := h;
  Iterator.Fp := p;
  Iterator.FData := FData;
end;

end.
