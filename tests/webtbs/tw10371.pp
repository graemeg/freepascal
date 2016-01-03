program bug;

{$MODE OBJFPC} {$H+}
{$INLINE ON}

uses
  SysUtils, Classes;

type
  TBug = class
  protected
<<<<<<< HEAD
<<<<<<< HEAD
    fL: longword;
    function InlinedMethod : longword; inline;
=======
    function InlinedMethod : longword; inline;
    fL: longword;
>>>>>>> graemeg/fixes_2_2
=======
    function InlinedMethod : longword; inline;
    fL: longword;
>>>>>>> origin/fixes_2_2
  public
    procedure Method1(var Buf); 
    procedure Method2;
  end;

function TBug.InlinedMethod : longword; inline;
begin
  Method1(Result);
end;

procedure TBug.Method2;
var aValue: longword;
begin
  aValue := InlinedMethod;
  fL:=aValue;
end;

procedure TBug.Method1(var Buf);
type
  plongword=^longword;
begin
  plongword(@buf)^:=$12345678;
end;

var
  b: tbug;
begin
  b:=tbug.create;
  b.method2;
  if (b.fl<>$12345678) then
    halt(1);
  b.free;
end.
