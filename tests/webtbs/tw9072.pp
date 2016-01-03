<<<<<<< HEAD
<<<<<<< HEAD
{ %opt=-Sew -vw }
=======
{ %opt=-Sew }
>>>>>>> graemeg/fixes_2_2
=======
{ %opt=-Sew }
>>>>>>> origin/fixes_2_2

{$mode tp}

var
  a: array[1..2] of longint;
  l: longint;
begin
  l:=0;
  for a[1]:=1 to 10 do
    for a[2]:=1 to 10 do
      inc(l);
  if (l<>100) then
    halt(1);
end.
