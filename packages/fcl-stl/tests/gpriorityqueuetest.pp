{$mode objfpc}

unit gpriorityqueuetest;

interface

uses fpcunit, testregistry, gpriorityqueue, gutil;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type lesslli=specialize TLess<longint>;
     queuelli=specialize TPriorityQueue<longint,lesslli>;
=======
{type lesslli=specialize TLess<longint>;
     queuelli=specialize TPriorityQueue<longint,lesslli>;}
>>>>>>> graemeg/cpstrnew
=======
{type lesslli=specialize TLess<longint>;
     queuelli=specialize TPriorityQueue<longint,lesslli>;}
>>>>>>> graemeg/cpstrnew
=======
{type lesslli=specialize TLess<longint>;
     queuelli=specialize TPriorityQueue<longint,lesslli>;}
>>>>>>> graemeg/cpstrnew
=======
{type lesslli=specialize TLess<longint>;
     queuelli=specialize TPriorityQueue<longint,lesslli>;}
>>>>>>> origin/cpstrnew

type TGPQueueTest = class(TTestCase)
  Published
    procedure QueueTest;
  public
    procedure Setup;override;
  private 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    data:queuelli;
=======
   { data:queuelli;}
>>>>>>> graemeg/cpstrnew
=======
   { data:queuelli;}
>>>>>>> graemeg/cpstrnew
=======
   { data:queuelli;}
>>>>>>> graemeg/cpstrnew
=======
   { data:queuelli;}
>>>>>>> origin/cpstrnew
  end;

implementation

procedure TGPQueueTest.QueueTest;
var i,last:longint;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Not IsEmpty', true, data.IsEmpty);
=======
{  AssertEquals('Not IsEmpty', true, data.IsEmpty);
>>>>>>> graemeg/cpstrnew
=======
{  AssertEquals('Not IsEmpty', true, data.IsEmpty);
>>>>>>> graemeg/cpstrnew
=======
{  AssertEquals('Not IsEmpty', true, data.IsEmpty);
>>>>>>> graemeg/cpstrnew
=======
{  AssertEquals('Not IsEmpty', true, data.IsEmpty);
>>>>>>> origin/cpstrnew
  for i:=0 to 10 do
    data.push(random(10000));
  last:=data.top;
  data.pop;
  for i:=0 to 9 do begin
    AssertEquals('Wrong order', true, data.top<last);
    AssertEquals('Wrong size', 10-i, data.size);
    last:=data.top;
    data.pop;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Not IsEmpty', true, data.IsEmpty);
=======
  AssertEquals('Not IsEmpty', true, data.IsEmpty);}
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Not IsEmpty', true, data.IsEmpty);}
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Not IsEmpty', true, data.IsEmpty);}
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Not IsEmpty', true, data.IsEmpty);}
>>>>>>> origin/cpstrnew
end;

procedure TGPQueueTest.Setup;
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  data:=queuelli.create;
=======
{  data:=queuelli.create;}
>>>>>>> graemeg/cpstrnew
=======
{  data:=queuelli.create;}
>>>>>>> graemeg/cpstrnew
=======
{  data:=queuelli.create;}
>>>>>>> graemeg/cpstrnew
=======
{  data:=queuelli.create;}
>>>>>>> origin/cpstrnew
end;

initialization
  RegisterTest(TGPQueueTest);
end.
