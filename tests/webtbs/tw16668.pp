{ %opt=-g-t }

program Project1;

{$mode objfpc}{$H+}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure Foo1;
var
  a: TObject;
  r: record
    b : array[0..10] of byte
  end;
=======
procedure Foo;
var
  a: TObject;
>>>>>>> graemeg/cpstrnew
=======
procedure Foo;
var
  a: TObject;
>>>>>>> graemeg/cpstrnew
=======
procedure Foo;
var
  a: TObject;
>>>>>>> origin/cpstrnew
begin
{$ifdef cpu32}
  if ptruint(a)<>$55555555 then
    halt(1);
{$else}
  if ptruint(a)<>$5555555555555555 then
    halt(1);
{$endif}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure Foo2;inline;
var
  a: TObject;
  r: record
    b : array[0..10] of byte
  end;
=======
procedure Foo;
var
  a: TObject;
>>>>>>> graemeg/cpstrnew
begin
{$ifdef cpu32}
  if ptruint(a)<>$55555555 then
    halt(1);
{$else}
  if ptruint(a)<>$5555555555555555 then
    halt(1);
{$endif}
end;

<<<<<<< HEAD

procedure Foo3;inline;
var
  r: record
    b : array[0..10] of byte
  end;
begin
  if r.b[3]<>$55 then
    halt(1);
end;

begin
  Foo1;
  Foo2;
  Foo3;
  writeln('ok');
=======
begin
  Foo;
>>>>>>> graemeg/cpstrnew
=======
begin
  Foo;
>>>>>>> graemeg/cpstrnew
=======
begin
  Foo;
>>>>>>> graemeg/cpstrnew
=======
begin
  Foo;
>>>>>>> origin/cpstrnew
end.
