{$mode delphi}

type
 TMyClass = class
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   i, i2 :LongInt;
=======
   i, i2 :Integer;
>>>>>>> graemeg/cpstrnew
=======
   i, i2 :Integer;
>>>>>>> graemeg/cpstrnew
=======
   i, i2 :Integer;
>>>>>>> graemeg/cpstrnew
=======
   i, i2 :Integer;
>>>>>>> origin/cpstrnew
 end;

begin
 if ptruint(@TMyClass(pointer(5)).i2)<>(5+sizeof(pointer)+4) then
   halt(1);
end.

