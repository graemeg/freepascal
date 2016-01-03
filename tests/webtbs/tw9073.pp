<<<<<<< HEAD
<<<<<<< HEAD
{ %opt=-CN }

{ the -CN is to generated null pointer load checks for AIX,
  ignored on other platforms because there the OS performs
  this checking for you }

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$mode objfpc}

uses
{$ifdef unix}
  cthreads,
{$endif}
  SysUtils, Classes;

type
  tc = class(tthread)
    procedure execute; override;
  end;

var
  caught: boolean;

procedure tc.execute;
type
  plongint = ^longint;
var
  p: plongint;
begin
  p:=nil;
  try
    writeln(p^);
  except
    caught:=true;
  end;
end;

var
  c: tc;
begin
  caught:=false;
  c:=tc.create(false);
  c.waitfor;
  c.free;
  halt(ord(not caught));
end.

