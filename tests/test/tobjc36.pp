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

{ Written by Jonas Maebe in 2010, released into the public domain }

{$mode objfpc}
{$modeswitch objectivec1}

// check whether we can override methods added to a class via a category
// (mainly required because the way Apple deprecates methods is by moving
//  them from class definitions to NSDeprecated category definitions)

type
  MyCategory = objccategory(NSObject)
    procedure extraproc(a: longint); message 'extraproc:';
  end;

  MyObject = objcclass(NSObject)
    // overrides extraproc added to NSObject
    procedure extraproc(a: longint); override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  MyObject2 = objcclass(NSObject)
    // overrides extraproc added to NSObject
    procedure extraproc(a: longint); override;
  end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure MyCategory.extraproc(a: longint);
  begin
    if a<>1 then
      halt(1);
  end;

procedure MyObject.extraproc(a: longint);
  begin
    if a<>2 then
      halt(2);
    inherited extraproc(1);
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure MyObject2.extraproc(a: longint);
  begin
    if a<>3 then
      halt(3);
    inherited extraproc(1);
  end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  a: NSObject;
  b: MyObject;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  c: MyObject2;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  a:=NSObject.alloc.init;
  a.extraproc(1);
  a.release;
  b:=MyObject.alloc.init;
  b.extraproc(2);
  b.release;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  c:=MyObject.alloc.init;
  c.extraproc(2);
  c.release;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end.
