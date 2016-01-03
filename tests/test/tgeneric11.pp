{$mode objfpc}

type
   generic TList<_T>=class(TObject)
<<<<<<< HEAD
<<<<<<< HEAD
   public
     var
       data : _T;
=======
=======
>>>>>>> origin/fixes_2_2
   var public
     data : _T;
>>>>>>> graemeg/fixes_2_2
     procedure Add(item: _T);
     procedure Assign(Source: specialize TList<_T>);
   end;

procedure TList.Add(item: _T);
begin
  data:=item;
end;

procedure TList.Assign(Source: specialize TList<_T>);
begin
  data:=Source.data;
end;

type
  TMyIntList = specialize TList<integer>;
  TMyStringList = specialize TList<string>;

var
  ilist1, ilist2 : TMyIntList;
  slist1, slist2 : TMyStringList;
begin
  ilist1 := TMyIntList.Create;
  ilist1.add(10);
  ilist2 := TMyIntList.Create;
  ilist2.add(20);
  ilist2.assign(ilist1);
  if ilist2.data <> 10 then
    halt(1);
  slist1 := TMyStringList.Create;
  slist1.add('test');
  slist2 := TMyStringList.Create;
  slist2.add('hello');
  slist2.assign(slist1);
  if slist2.data <> 'test' then
    halt(1);
end.

