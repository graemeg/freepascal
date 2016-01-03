{ the reason this compiles is that tdef.size and tabstractvarsym.getsize }
{ both return an aint, and then the size of ta is reported as low(aint)  }

type
  ta = array[1..high(ptrint)-4] of byte;
  tr = packed record
    l: longint;
    case byte of
<<<<<<< HEAD
<<<<<<< HEAD
      0: (x: longint);
=======
      0: (l: longint);
>>>>>>> graemeg/fixes_2_2
=======
      0: (l: longint);
>>>>>>> origin/fixes_2_2
      1: (e: ta);
  end;

begin
  writeln(sizeof(ta));
end.
