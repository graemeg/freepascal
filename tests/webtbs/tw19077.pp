uses
  math;
var
  c : currency;
begin
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_TYPE_EXTENDED}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
  c:=1.5625;
  c:=RoundTo(c,-1);
  if c<>1.6 then
    halt(1);
  writeln('ok');
<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_HAS_TYPE_EXTENDED}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
end.


