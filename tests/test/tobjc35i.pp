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
{ %norun }

{ Written by Jonas Maebe in 2010, released into the public domain }

{$mode objfpc}
{$modeswitch objectivec1}

// will refer to the real NSObject in objcbase
procedure mytest(a: NSObject);
begin
end;

// define external incomplete version
type
  NSObject = objcclass external;

  // NSObject should still resolve to the full definition in objcbase
  MyObject = objcclass(NSObject)
  end;

var
  // refers to external incomplete version
  a: NSObject;
begin
  // compiler should treat external and real type as compatible
  mytest(a);
end.
