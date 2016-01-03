{ %fail }
<<<<<<< HEAD
<<<<<<< HEAD
{ %opt=-Sew -vw }
=======
{ %opt=-Sew }
>>>>>>> graemeg/fixes_2_2
=======
{ %opt=-Sew }
>>>>>>> origin/fixes_2_2

procedure test;
var
  b: byte;
begin
  case b of
    1: b:=1;
  end;
end;

begin
  test;
end.
