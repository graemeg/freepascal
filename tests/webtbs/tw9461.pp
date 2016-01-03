{ %CPU=arm }
procedure p; assembler;
  var
    i : longint;
  asm
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef CPUTHUMB}
    mla r0,r1,r2,r3
{$endif CPUTHUMB}
=======
    mla r0,r1,r2,r3
>>>>>>> graemeg/fixes_2_2
=======
    mla r0,r1,r2,r3
>>>>>>> origin/fixes_2_2
  end;

begin
end.
