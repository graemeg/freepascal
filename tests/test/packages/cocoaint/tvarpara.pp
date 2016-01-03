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
  CocoaAll;

type
  unichar=word;
  punichar = ^unichar;
  MyObject = objcclass(NSObject)
    procedure receiveByVar_length(var chars : unichar; length : culong); message 'receiveByVar:length:';
    procedure receiveByPtr_length(chars : punichar; length : culong); message 'receiveByPtr:length:';
  end;

var
  c: array[0..1] of unichar;

procedure MyObject.receiveByVar_length(var chars : unichar; length : culong);
  begin
    Writeln('MyObject.receiveByVar_length');
    writeln('address of `chars`: 0x', HexStr(Pointer(@chars)));
    Writeln;
    if @chars<>@c[0] then
      halt(1);
  end;

procedure MyObject.receiveByPtr_length(chars : punichar; length : culong);
  begin
    Writeln('MyObject.receiveByPtr_length');
    writeln('address of `chars`: 0x', HexStr(Pointer(chars)));
    Writeln;
    if chars<>@c[0] then
      halt(2);
  end;

procedure passByVar(var chars : unichar);
  begin
    Writeln('passByVar');
    writeln('address of `chars`: 0x', HexStr(Pointer(@chars)));
    Writeln;
    if @chars<>@c[0] then
      halt(3);
  end;

procedure passByPtr(chars : punichar);
  begin
    Writeln('passByPtr');
    writeln('address of `chars`: 0x', HexStr(Pointer(chars)));
    Writeln;
    if chars<>@c[0] then
      halt(1);
  end;

procedure MyVarTest;
  var
    o: MyObject;
  begin
    o := MyObject(MyObject.alloc).init;
    o.receiveByVar_length(c[0], 1);
    o.receiveByPtr_length(@c[0], 1);
    passByVar(c[0]);
    passByPtr(@c[0]);
    o.release
  end;


begin
  c[0]:=unichar(widechar('c'));
  c[1]:=unichar(widechar(#0));
  MyVarTest;
end.
