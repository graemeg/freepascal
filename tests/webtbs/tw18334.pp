{ %cpu=arm }
{ %norun }

procedure test; assembler;
  asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(* not sure which CPUs really support it (FK) *)
{$ifdef CPU_ARMV7M}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    cps #0
    cpsie aif, #0
    cpsid aif, #0
    cpsie aif
    cpsid aif
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif CPU_ARMV7M}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;

begin
end.
