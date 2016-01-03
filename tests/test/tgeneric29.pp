<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %NORUN }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
program tgeneric29;

{$mode delphi}

type
  IGenericInterface<T> = interface
    function DoSomething(Arg: T): T;
  end;

  TGenericClass<T> = class(TInterfacedObject, IGenericInterface<T>)
    F: T;
    type
      Intf = IGenericInterface<Integer>;
    function DoSomething(Arg: T): T;
    function Test(Arg: Intf): Intf;
  end;

  TGenericRecord<T> = record
    F: T;
  end;

  TGenericArray<T> = array of T;

function TGenericClass<T>.DoSomething(Arg: T): T;
begin
  Result := Arg;
end;

function TGenericClass<T>.Test(Arg: Intf): Intf;
begin
  Result := Arg;
end;

var
  ClassSpecialize: TGenericClass<Integer>;
  RecordSpecialize: TGenericRecord<Integer>;
  ArraySpecialize: TGenericArray<Integer>;
begin
end.
