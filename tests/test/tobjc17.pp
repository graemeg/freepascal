{ %target=darwin }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %cpu=powerpc,powerpc64,i386,x86_64,arm,aarch64 }
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> origin/cpstrnew

{ Written by Jonas Maebe in 2009, released into the public domain }

program project1;

{$mode objfpc}{$H+}
{$modeswitch objectivec1}
type
 MyObject = objcclass(NSObject)
 private
   data : Integer;
 public
   procedure setData_(aData: Integer); message 'setData:';
 end;

procedure MyObject.setData_(aData: Integer);
begin
 data := aData;
end;

var
 m : MyObject;

begin
 m := MyObject.alloc;
 m.setData_(5);
 if (m.data<>5) then
   halt(1);
 m.release;
end.
