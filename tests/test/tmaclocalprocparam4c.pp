{ %norun }

{$mode tp}
{$modeswitch nestedprocvars}

type
  tprocedure = procedure;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure test(p: codepointer);
=======
procedure test(p: pointer);
>>>>>>> graemeg/cpstrnew
=======
procedure test(p: pointer);
>>>>>>> graemeg/cpstrnew
=======
procedure test(p: pointer);
>>>>>>> graemeg/cpstrnew
=======
procedure test(p: pointer);
>>>>>>> origin/cpstrnew

  procedure nested;
    begin
    end;

begin
  { this should be a plain pointer }
  test(@nested);
end;

begin
end.
