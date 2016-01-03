{$mode objfpc}

unit ghashsettest;

interface

uses fpcunit, testregistry, ghashset;

type hint=class
  class function hash(a,n:SizeUInt):SizeUInt;
end;

type THashsetlli=specialize THashSet<longint, hint>;

type TGHashSetTest = class(TTestCase)
  Published
    procedure HashSetTest1;
    procedure HashSetTest2;
    procedure HashSetTest3;
  public
    procedure Setup;override;
  private 
    data:THashsetlli;
  end;

implementation

class function hint.hash(a,n:SizeUInt):SizeUInt;
begin
  hash:= (a xor (a shr 5) xor (a shl 7)) and (n-1);
end;

procedure TGHashSetTest.HashSetTest1;
var i:longint;
begin
  AssertEquals('Not IsEmpty', true, data.IsEmpty);
  data.insert(47);
  AssertEquals('47 not found', true, data.contains(47));
  AssertEquals('39 found', false, data.contains(39));
  data.insert(39);
  data.insert(47);
  AssertEquals('bad size', 2, data.size);
  for i:=0 to 10000 do
    data.insert(20*i+42);
  for i:=0 to 10000 do
    AssertEquals('bad number found', false, data.contains(i*5+101));
  for i:=0 to 10000 do
    AssertEquals('number not found', true, data.contains(i*20+42));
  AssertEquals('IsEmpty', false, data.IsEmpty);
end;

procedure TGHashSetTest.HashSetTest2;
var i:longint;
begin
  for i:=0 to 1000 do
    data.insert(3*i);
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

procedure TGHashSetTest.HashSetTest3;
var i:longint;
    x:array[0..1000] of longint;
    it:THashSetlli.TIterator;
begin
  it:=data.Iterator;
  if it <> nil then
    AssertEquals('it not null', 0, 1);
  for i:=0 to 1000 do begin
    data.insert(i);
    x[i]:=0;
  end;
  it:=data.Iterator;
  repeat
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    inc(x[it.Data]);
=======
    inc(x[it.GetValue]);
>>>>>>> graemeg/cpstrnew
=======
    inc(x[it.GetValue]);
>>>>>>> graemeg/cpstrnew
=======
    inc(x[it.GetValue]);
>>>>>>> graemeg/cpstrnew
=======
    inc(x[it.GetValue]);
>>>>>>> origin/cpstrnew
  until not it.next;
  for i:=0 to 1000 do begin
    AssertEquals('som not 1', 1, x[i]);
  end;
end;

procedure TGHashSetTest.Setup;
begin
  data:=THashSetlli.create;
end;

initialization
  RegisterTest(TGHashSetTest);
end.
