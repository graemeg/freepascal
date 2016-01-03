{$ifdef fpc}
{$mode delphi}
{$endif}

uses
  variants;

type
  tdynarr = array of byte;

  tc = class
   private
    fda: tdynarr;
    fva: variant;
   public
    property da: tdynarr read fda write fda;
    property va: variant read fva write fva;
  end;

var
  c: tc;
  v: variant;
  d: tdynarr;
begin
  c:=tc.create;

  v:=5;
  c.va:=v;
  if (c.fva <> 5) then
    halt(1);
  v:='abc';
  v:=c.va;
  if (v <> 5) then
    halt(2);

  setlength(d,4);
  d[0]:=245;
  d[1]:=1;
  d[2]:=38;
  d[3]:=115;
  c.da:=d;
  if (length(c.fda)<>4) or
     (c.fda[0]<>245) or
     (c.fda[1]<>1) or
     (c.fda[2]<>38) or
     (c.fda[3]<>115) then
    halt(3);
  d:=nil;
  d:=c.da;
<<<<<<< HEAD
<<<<<<< HEAD
  c.da:=nil;
=======
  c.fda:=nil;
>>>>>>> graemeg/fixes_2_2
=======
  c.fda:=nil;
>>>>>>> origin/fixes_2_2
  if (length(d)<>4) or
     (d[0]<>245) or
     (d[1]<>1) or
     (d[2]<>38) or
     (d[3]<>115) then
    halt(4);
  
  c.free;
end.
