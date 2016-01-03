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

unit tobjc37;
interface
uses
	CocoaAll;

type
	MyWindow = objcclass(NSWindow)
		procedure awakeFromNib; override;
	end;
	
	
implementation

type
	MyView = objcclass(NSView)
		procedure awakeFromNib; override;
	end;

procedure MyWindow.awakeFromNib;
begin
end;

procedure MyView.awakeFromNib;
begin
end;

procedure test;
var
  w: mywindow;
  v: myview;
begin
  w:=mywindow.alloc.init;
  w.awakefromnib;
  w.release;
  v:=myview.alloc.init;
  v.awakefromnib;
  v.release;
end;
	
end.
