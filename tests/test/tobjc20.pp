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
  tr = record
    s: shortstring;
  end;

 MyObject = objcclass(NSObject)
   fss: shortstring;
   fsingle: single;
   fdouble: double;
   fbool: boolean;

   function getss: shortstring ; message 'getss';
   function getsspara(l1,l2: longint): shortstring ; message 'getss:l1:';
   function getsingle(l1,l2: longint): single; message 'getsingle:l1:';
   function getdouble(l1,l2: longint; d: double): double; message 'getdouble:l1:l2:';

   function getbool: boolean; message 'getbool';
 end;

function MyObject.getss: shortstring;
begin
  result:=fss;
end;


function MyObject.getsspara(l1,l2: longint): shortstring;
begin
  if (l1<>1) or
     (l2<>2) then
    halt(1);
  result:=fss;
end;


function MyObject.getsingle(l1,l2: longint): single;
begin
  if (l1<>1) or
     (l2<>2) then
    halt(2);
  result:=fsingle;
end;


function MyObject.getdouble(l1,l2: longint; d: double): double;
begin
  writeln(d);
  if (l1<>1) or
     (l2<>2) or
     (d<>1.5) then
    halt(3);
  result:=fdouble;
end;

function MyObject.getbool: boolean;
begin
  result:=fbool;
end;

var
  m: MyObject;
  b: boolean;
begin
 m := MyObject.alloc;
 m:=m.init;
 m.fss:='hello!';
 m.fsingle:=123.625;
 m.fdouble:=9876.0625;

 if m.getss<>'hello!' then
   halt(4);
 m.fss:='gij ook';
 if m.getsspara(1,2)<>'gij ook' then
   halt(5);
 if m.getsingle(1,2)<>123.625 then
   halt(6);
 if m.getdouble(1,2,1.5)<>9876.0625 then
   halt(7);

 m.fbool:=true;
 b:=m.getbool;
 if ord(b)<>ord(true) then
   halt(8);

 m.fbool:=false;
 b:=m.getbool;
 if ord(b)<>ord(false) then
   halt(9);

 m.release;
end.
