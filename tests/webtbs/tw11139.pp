function f(c: char): char; overload;
begin
  halt(1);
end;

function f(const s: shortstring): shortstring; overload;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  halt(3);
=======
  f:=lowercase(s);
>>>>>>> graemeg/fixes_2_2
=======
  f:=lowercase(s);
>>>>>>> origin/fixes_2_2
end;

function f(const a: ansistring): ansistring; overload;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  f:=lowercase(a);
=======
  halt(3);
>>>>>>> graemeg/fixes_2_2
=======
  halt(3);
>>>>>>> origin/fixes_2_2
end;

Procedure DoIt;
var avar:variant;
      txt:String;
Begin
avar:='Hello';
txt:=f(avar);//this line causes the compilation error
if (txt<>'hello') then
  halt(4);
end;

begin
  doit;
end.
