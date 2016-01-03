{ %fail }
<<<<<<< HEAD
<<<<<<< HEAD
{ %opt=-Se1w2 -vw }
=======
{ %opt=-Se1w2 }
>>>>>>> graemeg/fixes_2_2
=======
{ %opt=-Se1w2 }
>>>>>>> origin/fixes_2_2

var
  pc: pchar;
  m,m1: longword;
begin
  m:=1;
  pc^:=char(m-(m1*10)+byte('0'));
end.
