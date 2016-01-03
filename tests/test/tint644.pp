<<<<<<< HEAD
<<<<<<< HEAD
{ %OPT=-Oonoconstprop }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ %result=215 }

{$q+}
var
  c: qword;
begin
  c:=2;
  c:=c*qword($ffffffffffffffff);
end.
