var
  s: string;
  d: Double;
begin
  d := 5.9999999999999991;
<<<<<<< HEAD
<<<<<<< HEAD
  Str(d:23,s);
  if (pos('6',s)<>0) then
=======
  Str(d:23,s); 
  if (pos('9',s)<>0) or (pos('5',s)<>0) then
>>>>>>> graemeg/fixes_2_2
=======
  Str(d:23,s); 
  if (pos('9',s)<>0) or (pos('5',s)<>0) then
>>>>>>> origin/fixes_2_2
    halt(1);
end.
