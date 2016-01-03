uses
{$ifdef unix}
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef darwin}iosxwstr{$else}cwstring{$endif},
{$endif}
  variants,sysutils;
var a:variant;
    x,y: array of byte;
=======
=======
>>>>>>> origin/fixes_2_2
  cwstring,
{$endif}
  variants,sysutils;
var a:variant;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  a:=VarArrayCreate([0,2,0,2],varVariant);
  if VarArrayDimCount(a)<>2 then
    halt(1);
  VarArrayPut(a,'b',[1,1]);
  if String(VarArrayGet(a,[1,1]))<>'b' then
    halt(2);
  a[2,1]:='asdf';
  if VarArrayGet(a,[2,1])<>'asdf' then
    halt(2);
<<<<<<< HEAD
<<<<<<< HEAD
  setlength(x,3);
  x[0]:=77;
  x[1]:=88;
  x[2]:=99;
  a[2,2]:=x;
  y:=VarArrayGet(a,[2,2]);
  if (y[0]<>x[0]) or
     (y[1]<>x[1]) or
     (y[2]<>x[2]) then
    halt(3);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  a:='';
  writeln('ok');
end.


