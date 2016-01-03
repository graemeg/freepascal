{$mode objfpc}

unit gmaptest;

interface

uses fpcunit, testregistry, gmap, gutil;

type lesslli=specialize TLess<longint>;
     maplli=specialize TMap<longint,longint, lesslli>;

type TGMapTest = class(TTestCase)
  Published
    procedure MapTest;
  public
    procedure Setup;override;
  private 
    data:maplli;
  end;

implementation

procedure TGMapTest.MapTest;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
var it:maplli.TIterator;
=======
var it:maplli.TMSet.pnode;
>>>>>>> graemeg/cpstrnew
=======
var it:maplli.TMSet.pnode;
>>>>>>> graemeg/cpstrnew
=======
var it:maplli.TMSet.pnode;
>>>>>>> graemeg/cpstrnew
=======
var it:maplli.TMSet.pnode;
>>>>>>> origin/cpstrnew
begin
  data[3]:=3;
  data[5]:=5;
  data[7]:=7;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Wrong min key', 3, data.min().GetData.key);
  AssertEquals('Wrong max key', 7, data.max().GetData.key);
  AssertEquals('Wrong min val', 3, data.min().GetData.value);
  AssertEquals('Wrong max val', 7, data.max().GetData.value);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AssertEquals('Wrong min key', 3, data.min()^.data.key);
  AssertEquals('Wrong max key', 7, data.max()^.data.key);
  AssertEquals('Wrong min val', 3, data.min()^.data.value);
  AssertEquals('Wrong max val', 7, data.max()^.data.value);
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

  AssertEquals('Wrong val', 5, data[5]);

  data.delete(3);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Wrong min key', 5, data.min().GetData.key);
  AssertEquals('Wrong max key', 7, data.max().GetData.key);
  AssertEquals('Wrong min val', 5, data.min().GetData.value);
  AssertEquals('Wrong max val', 7, data.max().GetData.value);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AssertEquals('Wrong min key', 5, data.min()^.data.key);
  AssertEquals('Wrong max key', 7, data.max()^.data.key);
  AssertEquals('Wrong min val', 5, data.min()^.data.value);
  AssertEquals('Wrong max val', 7, data.max()^.data.value);
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


  data[3]:=3;
  data[3]:=47;
  AssertEquals('Wrong val 2', 47, data[3]);

  if(data.find(4)<>nil) then 
    AssertEquals('Found key which not there', 0, 1);

  data[17]:=42;

  it:=data.min;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Wrong min', 3, it.Key);
  AssertEquals('Next not true', true, it.Next);
  AssertEquals('Wrong next', 5, it.Key);
  AssertEquals('Wrong next value', 5, it.Value);
  it.Value := it.Value + 17;
  AssertEquals('Wrong value update', 22, it.Value);
  it.MutableValue^:= 444;
  AssertEquals('Wrong mutable value update', 444, it.Value);
  AssertEquals('Next not true', true, it.Next);
  AssertEquals('Wrong next', 7, it.GetData.key);
  AssertEquals('Next not true', true, it.Next);
  AssertEquals('Wrong next', 17, it.GetData.key);
  AssertEquals('Next not false', false, it.Next);

  it:=data.max;
  AssertEquals('Wrong max', 17, it.GetData.key);
  AssertEquals('Prev not true', true, it.Prev);
  AssertEquals('Wrong prev', 7, it.GetData.key);
  AssertEquals('Prev not true', true, it.Prev);
  AssertEquals('Wrong prev', 5, it.GetData.key);
  AssertEquals('Prev not true', true, it.Prev);
  AssertEquals('Wrong prev', 3, it.GetData.key);
  AssertEquals('Prev not false', false, it.Prev);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AssertEquals('Wrong min', 3, it^.Data.key);
  it:=data.next(it);
  AssertEquals('Wrong next', 5, it^.Data.key);
  it:=data.next(it);
  AssertEquals('Wrong next', 7, it^.Data.key);
  it:=data.next(it);
  AssertEquals('Wrong next', 17, it^.Data.key);
  it:=data.next(it);
  if(it<>nil) then
    AssertEquals('Last not nil', 0, 1);

  it:=data.max;
  AssertEquals('Wrong max', 17, it^.Data.key);
  it:=data.prev(it);
  AssertEquals('Wrong prev', 7, it^.Data.key);
  it:=data.prev(it);
  AssertEquals('Wrong prev', 5, it^.Data.key);
  it:=data.prev(it);
  AssertEquals('Wrong prev', 3, it^.Data.key);
  it:=data.prev(it);
  if(it<>nil) then
    AssertEquals('First not nil', 0, 1);
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

procedure TGMapTest.Setup;
begin
  data:=maplli.create;
end;

initialization
  RegisterTest(TGMapTest);
end.
