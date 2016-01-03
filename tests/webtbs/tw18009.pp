{ %CPU=i386 }
{ %TARGET=go32v2,win32,linux }
{ %NOTE=This test requires an installed Nasm }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %OPT=-Cg- }
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ASMMODE INTEL}
var
q:qword;

begin
asm
  paddb mm0,q
  paddw mm0,q
  paddd mm0,q
  paddusb mm0,q
  paddusw mm0,q
  paddsb mm0,q
  paddsw mm0,q
end;
end.
