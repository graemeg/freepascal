{ %opt=-O- }
var
  q1,q2,q3 : qword;
begin
  q1:=$0000FFFFFFFEFFFF;
  q2:=$FFFEFFFF;
  q3:=$FFFF00000000;
  if q1<>q2 or q3 then
    halt(1);
end.
