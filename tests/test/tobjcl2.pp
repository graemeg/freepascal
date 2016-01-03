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
{ %NEEDLIBRARY }

{ Written by Jonas Maebe in 2009, released into the public domain }

{$mode objfpc}
{$modeswitch objectivec1}

const
{$ifdef windows}
  libname='tobjcl1.dll';
{$else}
  libname='tobjcl1';
  {$linklib tobjcl1}
{$endif}

type
  MyLibObjCClass = objcclass external (NSObject)
   public
    fa: byte;
    function publicfun: byte; message 'publicfun';
   protected
    fb: byte;
    function protectedfun: byte; message 'protectedfun';
   private
    fc: byte;
    function privatefun: byte; message 'privatefun';
  end;

  MyDerivedClass = objcclass(MyLibObjCClass)
    l: longint;
    function callprotectedfun: byte; message 'callprotectedfun';
  end;


function MyDerivedClass.callprotectedfun: byte;
  begin
    result:=protectedfun;
  end;


var
  a: MyLibObjCClass;
begin
  a:=NSObject(MyDerivedClass.alloc).init;
  a.fa:=55;
  a.fb:=66;
  if a.publicfun<>55 then
    halt(1);
  if MyDerivedClass(a).callprotectedfun<>66 then
    halt(2);
  a.release;
end.
