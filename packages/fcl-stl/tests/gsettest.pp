{$mode objfpc}

unit gsettest;

interface

uses fpcunit, testregistry, gset, gutil;

type lesslli=specialize TLess<longint>;
     setlli=specialize TSet<longint,lesslli>;

type TGSetTest = class(TTestCase)
  Published
    procedure SetTest;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure IteratorTest;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  public
    procedure Setup;override;
  private 
    data:setlli;
  end;

implementation

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TGSetTest.IteratorTest;
var it:setlli.TIterator;
begin
  it:=data.min;
  if (it <> nil) then
    AssertEquals('not null min', 0, 1);
  data.insert(3);
  data.insert(5);
  data.insert(7);
  it:=data.min;
  AssertEquals('bad value', 3, it.Data);
  AssertEquals('next not true', true, it.Next);
  AssertEquals('bad value', 5, it.Data);
  AssertEquals('next not true', true, it.Next);
  AssertEquals('bad value', 7, it.Data);
  AssertEquals('next not false', false, it.Next);
end;

procedure TGSetTest.SetTest;
var it:setlli.TIterator;
=======
procedure TGSetTest.SetTest;
var it:setlli.pnode;
>>>>>>> graemeg/cpstrnew
=======
procedure TGSetTest.SetTest;
var it:setlli.pnode;
>>>>>>> graemeg/cpstrnew
=======
procedure TGSetTest.SetTest;
var it:setlli.pnode;
>>>>>>> graemeg/cpstrnew
=======
procedure TGSetTest.SetTest;
var it:setlli.pnode;
>>>>>>> origin/cpstrnew
begin
  data.insert(3);
  data.insert(5);
  data.insert(7);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Wrong min', 3, data.min().Data);
  AssertEquals('Wrong max', 7, data.max().Data);
  data.delete(3);
  AssertEquals('Wrong size', 2, data.size);
  AssertEquals('Wrong min', 5, data.min().Data);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AssertEquals('Wrong min', 3, data.min()^.data);
  AssertEquals('Wrong max', 7, data.max()^.data);
  data.delete(3);
  AssertEquals('Wrong size', 2, data.size);
  AssertEquals('Wrong min', 5, data.min()^.data);
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
  data.insert(3);
  data.insert(3);
  data.insert(3);
  AssertEquals('Wrong size', 3, data.size);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Wrong min', 3, data.min().Data);
=======
  AssertEquals('Wrong min', 3, data.min()^.data);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Wrong min', 3, data.min()^.data);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Wrong min', 3, data.min()^.data);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Wrong min', 3, data.min()^.data);
>>>>>>> origin/cpstrnew
  if(data.find(4)<>nil) then 
    Fail('Found key which not there');
  if(data.find(5)=nil) then
    Fail('Not found key which was there');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if(data.FindLess(8).Data<>7) then
    Fail('Wrong less than 8');
  if(data.FindLess(7).Data<>5) then
=======
  if(data.FindLess(8)^.data<>7) then
    Fail('Wrong less than 8');
  if(data.FindLess(7)^.data<>5) then
>>>>>>> graemeg/cpstrnew
=======
  if(data.FindLess(8)^.data<>7) then
    Fail('Wrong less than 8');
  if(data.FindLess(7)^.data<>5) then
>>>>>>> graemeg/cpstrnew
=======
  if(data.FindLess(8)^.data<>7) then
    Fail('Wrong less than 8');
  if(data.FindLess(7)^.data<>5) then
>>>>>>> graemeg/cpstrnew
=======
  if(data.FindLess(8)^.data<>7) then
    Fail('Wrong less than 8');
  if(data.FindLess(7)^.data<>5) then
>>>>>>> origin/cpstrnew
    Fail('Wrong less than 7');
  if(data.FindLess(3)<>nil) then
    Fail('Wrong less than 3');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if(data.FindLessEqual(8).Data<>7) then
    Fail('Wrong less equal than 8');
  if(data.FindLessEqual(7).Data<>7) then
    Fail('Wrong less equal than 7');
  if(data.FindLessEqual(6).Data<>5) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if(data.FindLessEqual(8)^.data<>7) then
    Fail('Wrong less equal than 8');
  if(data.FindLessEqual(7)^.data<>7) then
    Fail('Wrong less equal than 7');
  if(data.FindLessEqual(6)^.data<>5) then
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
    Fail('Wrong less equal than 6');
  if(data.FindLessEqual(2)<>nil) then
    Fail('Wrong less equal than 2');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if(data.FindGreater(2).Data<>3) then
    Fail('Wrong greater than 2');
  if(data.Findgreater(3).Data<>5) then
=======
  if(data.FindGreater(2)^.data<>3) then
    Fail('Wrong greater than 2');
  if(data.Findgreater(3)^.data<>5) then
>>>>>>> graemeg/cpstrnew
=======
  if(data.FindGreater(2)^.data<>3) then
    Fail('Wrong greater than 2');
  if(data.Findgreater(3)^.data<>5) then
>>>>>>> graemeg/cpstrnew
=======
  if(data.FindGreater(2)^.data<>3) then
    Fail('Wrong greater than 2');
  if(data.Findgreater(3)^.data<>5) then
>>>>>>> graemeg/cpstrnew
=======
  if(data.FindGreater(2)^.data<>3) then
    Fail('Wrong greater than 2');
  if(data.Findgreater(3)^.data<>5) then
>>>>>>> origin/cpstrnew
    Fail('Wrong greater than 3');
  if(data.Findgreater(7)<>nil) then
    Fail('Wrong greater than 7');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if(data.FindGreaterEqual(2).Data<>3) then
    Fail('Wrong greater equal than 2');
  if(data.FindGreaterEqual(3).Data<>3) then
    Fail('Wrong greater equal than 3');
  if(data.FindGreaterEqual(4).Data<>5) then
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if(data.FindGreaterEqual(2)^.data<>3) then
    Fail('Wrong greater equal than 2');
  if(data.FindGreaterEqual(3)^.data<>3) then
    Fail('Wrong greater equal than 3');
  if(data.FindGreaterEqual(4)^.data<>5) then
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
    Fail('Wrong greater equal than 4');
  if(data.FindGreaterEqual(8)<>nil) then
    Fail('Wrong greater equal than 8');

  data.insert(17);

  it:=data.min;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Wrong min', 3, it.Data);
  AssertEquals('Next not true', true, it.next);
  AssertEquals('Wrong next', 5, it.Data);
  AssertEquals('Next not true', true, it.next);
  AssertEquals('Wrong next', 7, it.Data);
  AssertEquals('Next not true', true, it.next);
  AssertEquals('Wrong next', 17, it.Data);
  AssertEquals('Last next not fail', false, it.next);

  it:=data.max;
  AssertEquals('Wrong max', 17, it.Data);
  AssertEquals('Prev not true', true, it.prev);
  AssertEquals('Wrong prev', 7, it.Data);
  AssertEquals('Prev not true', true, it.prev);
  AssertEquals('Wrong prev', 5, it.Data);
  AssertEquals('Prev not true', true, it.prev);
  AssertEquals('Wrong prev', 3, it.Data);
  AssertEquals('First prev not fail', false, it.prev);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AssertEquals('Wrong min', 3, it^.data);
  it:=data.next(it);
  AssertEquals('Wrong next', 5, it^.data);
  it:=data.next(it);
  AssertEquals('Wrong next', 7, it^.data);
  it:=data.next(it);
  AssertEquals('Wrong next', 17, it^.data);
  it:=data.next(it);
  if(it<>nil) then
    AssertEquals('Last not nil', 0, 1);

  it:=data.max;
  AssertEquals('Wrong max', 17, it^.data);
  it:=data.prev(it);
  AssertEquals('Wrong prev', 7, it^.data);
  it:=data.prev(it);
  AssertEquals('Wrong prev', 5, it^.data);
  it:=data.prev(it);
  AssertEquals('Wrong prev', 3, it^.data);
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

procedure TGSetTest.Setup;
begin
  data:=setlli.create;
end;

initialization
  RegisterTest(TGSetTest);
end.
