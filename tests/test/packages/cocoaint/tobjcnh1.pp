{ %target=darwin }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %cpu=powerpc,i386,powerpc64,x86_64 }
=======
{ %cpu=powerpc,i386 }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,i386 }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,i386 }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,i386 }
>>>>>>> origin/cpstrnew

{$mode objfpc}{$H+}
{$modeswitch objectivec1}
program Start;
uses
ctypes,
CFBase, CFString,
CocoaAll;

type
MyObject = objcclass(NSObject)
  function initMyObject : id; message 'initMyObject';
  function testFunction : cint; message 'testFunction';
end;

MySubobject = objcclass(MyObject)
  function initMyObject : id; message 'initMyObject'; override;
  function testFunction : cint; message 'testFunction'; override;
end;

procedure NSLog(fmt : CFStringRef); cdecl; varargs; external name 'NSLog';

function MyObject.initMyObject : id;
  var
    temp: id;
  begin
    Result:=nil;
    NSLog(CFSTR('MyObject.initMyObject entry, self = %p'), self);
    Result := inherited init;
    { default NSObject.init does not return anything different,
      so should be safe in test program }
    if result<>self then
      halt(1);
    NSLog(CFSTR('Result assigned by inherited init = %p'), Result);
    NSLog(CFSTR('self after inherited init = %p'), self);
    Result := self;
    NSLog(CFSTR('returning result = %p'), Result)
  end;

function MyObject.testFunction : cint;
  begin
    Result := 1;
    NSLog(CFSTR('MyObject.testFunction returning %d'), Result)
  end;

function MySubobject.initMyObject : id;
  begin
    Result:=nil;
    NSLog(CFSTR('MySubobject.initMyObject entry, self = %p'), self);
    Result := inherited initMyObject;
    if (result<>self) then
      halt(2);
    NSLog(CFSTR('Result assigned by inherited initMyObject = %p'), Result);
    NSLog(CFSTR('self after inherited init = %p'), self);
    Result := self;
    NSLog(CFSTR('returning result = %p'), Result)
  end;

function MySubobject.testFunction : cint;
  begin
    Result:=-1;
    writeln('MySubobject.testFunction calling inherited...');
    Result := inherited testFunction;
    if (result<>1) then
      halt(3);
    NSLog(CFSTR('Return from inherited = %d'), Result);
    Result := 2;
    NSLog(CFSTR('MySubobject.testFunction returning %d'), Result)
  end;


procedure MyTest;
  var
    ap: NSAutoreleasePool;
    o: MyObject;
    oo: MySubobject;
    n: cint;
  begin
    ap := NSAutoreleasePool.new;
    writeln('========== Initializing MyObject and MySubobject ==========');
    o := MyObject(MyObject.alloc).initMyObject;
    writeln;
    oo := MySubobject(MySubobject.alloc).initMyObject;
    writeln; writeln;
    writeln('========== Testing testFunction ==========');
    n := o.testFunction;
    writeln('MyObject.testFunction returned ', n);
    writeln;
    n := oo.testFunction;
    writeln('MySubobject.testFunction returned ', n);
    o.release;
    oo.release;
    ap.drain
  end;

begin
  MyTest;
end.
