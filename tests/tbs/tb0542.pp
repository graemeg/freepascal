{ %norun }
<<<<<<< HEAD
<<<<<<< HEAD
{ %opt=-Sew -vw }
=======
{ %opt=-Sew }
>>>>>>> graemeg/fixes_2_2
=======
{ %opt=-Sew }
>>>>>>> origin/fixes_2_2

procedure t;
var
  f:array[1..10] of text;
begin
  assign(f[1],paramstr(0));
end;

begin
  t;
end.
