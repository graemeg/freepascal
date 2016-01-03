<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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

  unit ghashmap;

  interface
  uses gvector, gutil, garrayutils;

  const baseFDataSize = 8;

  {Thash should have one class function hash(a:TKey, n:longint):longint which return uniformly distributed
  value in range <0,n-1> base only on arguments, n will be always power of 2}

  type
    generic THashmapIterator<TKey, TValue, T, TTable>=class
      public
      type PValue=^TValue;
      var
        Fh,Fp:SizeUInt;
        FData:TTable;
        function Next:boolean;inline;
        function GetData:T;inline;
        function GetKey:TKey;inline;
        function GetValue:TValue;inline;
        function GetMutable:PValue;inline;
        procedure SetValue(value:TValue);inline;
        property Data:T read GetData;
        property Key:TKey read GetKey;
        property Value:TValue read GetValue write SetValue;
        property MutableValue:PValue read GetMutable;
    end;

    generic THashmap<TKey, TValue, Thash>=class
      public
      type
        TPair=record
          Value:TValue;
          Key:TKey;
        end;
      var
      private 
      type
        TContainer = specialize TVector<TPair>;
        TTable = specialize TVector<TContainer>;
      var 
        FData:TTable;
        FDataSize:SizeUInt; 
        procedure EnlargeTable;
      public 
      type
        TIterator = specialize THashmapIterator<TKey, TValue, TPair, TTable>;
        constructor create;
        destructor destroy;override;
        procedure insert(key:TKey;value:TValue);inline;
        function contains(key:TKey):boolean;inline;
        function size:SizeUInt;inline;
        procedure delete(key:TKey);inline;
        function IsEmpty:boolean;inline;
        function GetData(key:TKey):TValue;inline;

        property Items[i : TKey]: TValue read GetData write Insert; default;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

unit ghashmap;

interface
uses gvector, gutil, garrayutils;

const baseFDataSize = 8;

{Thash should have one class function hash(a:TKey, n:longint):longint which return uniformly distributed
value in range <0,n-1> base only on arguments, n will be always power of 2}

type
  generic THashmapIterator<T, TTable>=class
    public
    var
      Fh,Fp:SizeUInt;
      FData:TTable;
      function Next:boolean;
      function GetValue:T;
  end;

  generic THashmap<TKey, TValue, Thash>=class
    public
    type
      TPair=record
        Value:TValue;
        Key:TKey;
      end;
    var
    private 
    type
      TContainer = specialize TVector<TPair>;
      TTable = specialize TVector<TContainer>;
    var 
      FData:TTable;
      FDataSize:SizeUInt; 
      procedure EnlargeTable;
    public 
    type
      TIterator = specialize THashmapIterator<TPair, TTable>;
      constructor create;
      destructor destroy;override;
      procedure insert(key:TKey;value:TValue);inline;
      function contains(key:TKey):boolean;inline;
      function size:SizeUInt;inline;
      procedure delete(key:TKey);inline;
      function IsEmpty:boolean;inline;
      function GetValue(key:TKey):TValue;inline;

      property Items[i : TKey]: TValue read GetValue write Insert; default;
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

      function Iterator:TIterator;
  end;

implementation

function THashmap.Size:SizeUInt;inline;
begin
  Size:=FDataSize;
end;

destructor THashmap.Destroy;
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

function THashmap.IsEmpty():boolean;inline;
begin
  if Size()=0 then 
    IsEmpty:=true
  else 
    IsEmpty:=false;
end;

procedure THashmap.EnlargeTable;
var i,j,h,oldDataSize:SizeUInt; 
    value:TPair;
begin
  oldDataSize:=FData.size;
  FData.resize(FData.size*2);
  for i:=oldDataSize to FData.size-1 do
    FData[i] := TContainer.create;
  for i:=oldDataSize-1 downto 0 do begin
    j := 0;
    while j < (FData[i]).size do begin
      value := (FData[i])[j];
      h:=Thash.hash(value.key,FData.size);
      if (h <> i) then begin
        (FData[i])[j] := (FData[i]).back;
        (FData[i]).popback;
        (FData[h]).pushback(value);
      end else
        inc(j);
    end;
  end;
end;

constructor THashmap.create;
var i:longint;
begin
  FDataSize:=0;
  FData:=TTable.create;
  FData.resize(baseFDataSize);
  for i:=0 to baseFDataSize-1 do
    FData[i]:=TContainer.create;
end;

function THashmap.contains(key:TKey):boolean;inline;
var i,h,bs:longint;
begin
  h:=Thash.hash(key,FData.size);
  bs:=(FData[h]).size;
  for i:=0 to bs-1 do begin
    if (((FData[h])[i]).Key=key) then exit(true);
  end;
  exit(false);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function THashmap.GetData(key:TKey):TValue;inline;
=======
function THashmap.GetValue(key:TKey):TValue;inline;
>>>>>>> graemeg/cpstrnew
=======
function THashmap.GetValue(key:TKey):TValue;inline;
>>>>>>> graemeg/cpstrnew
=======
function THashmap.GetValue(key:TKey):TValue;inline;
>>>>>>> graemeg/cpstrnew
=======
function THashmap.GetValue(key:TKey):TValue;inline;
>>>>>>> origin/cpstrnew
var i,h,bs:longint;
begin
  h:=Thash.hash(key,FData.size);
  bs:=(FData[h]).size;
  for i:=0 to bs-1 do begin
    if (((FData[h])[i]).Key=key) then exit(((FData[h])[i]).Value);
  end;
end;

procedure THashmap.insert(key:TKey;value:TValue);inline;
var pair:TPair; i,h,bs:longint;
begin
  h:=Thash.hash(key,FData.size);
  bs:=(FData[h]).size;
  for i:=0 to bs-1 do begin
    if (((FData[h])[i]).Key=key) then begin
      ((FData[h]).mutable[i])^.value := value;
      exit;
    end;
  end;
  pair.Key := key;
  pair.Value := value;
  inc(FDataSize);
  (FData[h]).pushback(pair);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if (FDataSize > 5*FData.size) then
=======
  if (FDataSize > 2*FData.size) then
>>>>>>> graemeg/cpstrnew
=======
  if (FDataSize > 2*FData.size) then
>>>>>>> graemeg/cpstrnew
=======
  if (FDataSize > 2*FData.size) then
>>>>>>> graemeg/cpstrnew
=======
  if (FDataSize > 2*FData.size) then
>>>>>>> origin/cpstrnew
    EnlargeTable;
end;

procedure THashmap.delete(key:TKey);inline;
var h,i:SizeUInt;
begin
  h:=Thash.hash(key,FData.size);
  i:=0;
  while i < (FData[h]).size do begin
    if (((FData[h])[i]).key=key) then begin
      (FData[h])[i] := (FData[h]).back;
      (FData[h]).popback;
      dec(FDataSize);
      exit;
    end;
    inc(i);
  end;
end;

function THashmapIterator.Next:boolean;
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
function THashmapIterator.GetData:T;
begin
  GetData:=(FData[Fh])[Fp];
=======
function THashmapIterator.GetValue:T;
begin
  GetValue:=(FData[Fh])[Fp];
>>>>>>> graemeg/cpstrnew
=======
function THashmapIterator.GetValue:T;
begin
  GetValue:=(FData[Fh])[Fp];
>>>>>>> graemeg/cpstrnew
=======
function THashmapIterator.GetValue:T;
begin
  GetValue:=(FData[Fh])[Fp];
>>>>>>> graemeg/cpstrnew
=======
function THashmapIterator.GetValue:T;
begin
  GetValue:=(FData[Fh])[Fp];
>>>>>>> origin/cpstrnew
end;

function THashmap.Iterator:TIterator;
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function THashmapIterator.GetKey:TKey;inline;
begin
  GetKey:=((FData[Fh])[Fp]).Key;
end;

function THashmapIterator.GetValue:TValue;inline;
begin
  GetValue:=((FData[Fh])[Fp]).Value;
end;

function THashmapIterator.GetMutable:PValue;inline;
begin
  GetMutable:=@((FData[Fh]).Mutable[Fp]^.Value);
end;

procedure THashmapIterator.SetValue(value:TValue);inline;
begin
  ((FData[Fh]).mutable[Fp])^.Value := value;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end.
