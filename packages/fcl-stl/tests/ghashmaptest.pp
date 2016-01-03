{$mode objfpc}

unit ghashmaptest;

interface

uses fpcunit, testregistry, ghashmap;

type hint=class
  class function hash(a,n:SizeUInt):SizeUInt;
end;

type THashmaplli=specialize THashMap<longint, longint, hint>;

type TGHashmapTest = class(TTestCase)
  Published
    procedure HashmapTest1;
    procedure HashmapTest2;
    procedure HashmapTest3;
  public
    procedure Setup;override;
  private 
    data:THashmaplli;
  end;

implementation

class function hint.hash(a,n:SizeUInt):SizeUInt;
begin
  hash:= (a xor (a shr 5) xor (a shl 7)) and (n-1);
end;

procedure TGHashmapTest.HashMapTest1;
var i:longint;
begin
  AssertEquals('Not IsEmpty', true, data.IsEmpty);
  data.insert(47, 42);
  AssertEquals('47 not found', true, data.contains(47));
  AssertEquals('39 found', false, data.contains(39));
  data[39]:=33;
  data[47]:=22;
  AssertEquals('bad size', 2, data.size);
  AssertEquals('bad 47', 22, data[47]);
  for i:=0 to 10000 do
    data[20*i+42] := 47+i;
  for i:=0 to 10000 do
    AssertEquals('bad number found', false, data.contains(i*5+101));
  for i:=0 to 10000 do
    AssertEquals('bad number', i+47, data[i*20+42]);
  AssertEquals('IsEmpty', false, data.IsEmpty);
end;

procedure TGHashmapTest.HashMapTest2;
var i:longint;
begin
  for i:=0 to 1000 do
    data[3*i] := 7*i;
  for i:=0 to 1000 do
    data.delete(3*i+1);
  AssertEquals('bad size before delete', 1001, data.size);
  for i:=500 to 1000 do
    data.delete(3*i);
  AssertEquals('bad size after delete', 500, data.size);
  for i:=0 to 499 do
    AssertEquals('element not found', true, data.contains(3*i));
  for i:=500 to 1000 do
    AssertEquals('deleted element found', false, data.contains(3*i));
end;

procedure TGHashmapTest.HashMapTest3;
var i:longint;
    x:array[0..1000] of longint;
    it:THashmaplli.TIterator;
begin
  it:=data.Iterator;
  if it <> nil then
    AssertEquals('it not null', 0, 1);
  for i:=0 to 1000 do begin
    data[i]:=47*i;
    x[i]:=0;
  end;
  it:=data.Iterator;
  repeat
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    inc(x[it.Data.key]);
    AssertEquals('bad value', it.Data.key*47, it.Data.value);
    AssertEquals('bad value2', it.Key*47, it.Value);
    it.Value := it.Key+23;
    it.Value := it.Value*2;
    AssertEquals('bad value3', it.Key*2+46, it.Value);
    it.MutableValue^ := 222;
    AssertEquals('bad value4', 222, it.Value);
=======
    inc(x[it.GetValue.key]);
    AssertEquals('bad value', it.GetValue.key*47, it.GetValue.value);
>>>>>>> graemeg/cpstrnew
=======
    inc(x[it.GetValue.key]);
    AssertEquals('bad value', it.GetValue.key*47, it.GetValue.value);
>>>>>>> graemeg/cpstrnew
=======
    inc(x[it.GetValue.key]);
    AssertEquals('bad value', it.GetValue.key*47, it.GetValue.value);
>>>>>>> graemeg/cpstrnew
=======
    inc(x[it.GetValue.key]);
    AssertEquals('bad value', it.GetValue.key*47, it.GetValue.value);
>>>>>>> origin/cpstrnew
  until not it.next;
  for i:=0 to 1000 do begin
    AssertEquals('som not 1', 1, x[i]);
  end;
end;

procedure TGHashmapTest.Setup;
begin
  data:=THashmaplli.create;
end;

initialization
  RegisterTest(TGHashmapTest);
end.
