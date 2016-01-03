<<<<<<< HEAD
<<<<<<< HEAD
{ %cpu=i386,powerpc}
{ %target=linux,win32,go32v2,os2,beos,haiku,morphos }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ %OPT=-ghc }
//test.pp
type o=object
      constructor init;
     end;

constructor o.init; begin end;

var o1 : ^o;

begin
  HaltOnNotReleased := true;;
  New(o1,init);
  // New(o1); o1^.init; <- no error
  dispose(o1);
end.
