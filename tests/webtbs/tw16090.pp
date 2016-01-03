{ %norun }

program tw16090;

{$mode objfpc}{$H+}

type
  TClass1 = class
  public type
    generic TNestedClass<T> = class
    end;
    // Fatal: Internal error 200705152
    TSpecialization1 = specialize TNestedClass<Integer>;
  end;

  // Fatal: Internal error 200705152
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TSpecialization1 = TClass1.specialize TNestedClass<Integer>;
=======
  TSpecialization1 = specialize TClass1.TNestedClass<Integer>;
>>>>>>> graemeg/cpstrnew
=======
  TSpecialization1 = specialize TClass1.TNestedClass<Integer>;
>>>>>>> graemeg/cpstrnew
=======
  TSpecialization1 = specialize TClass1.TNestedClass<Integer>;
>>>>>>> graemeg/cpstrnew
=======
  TSpecialization1 = specialize TClass1.TNestedClass<Integer>;
>>>>>>> origin/cpstrnew

begin
end.

