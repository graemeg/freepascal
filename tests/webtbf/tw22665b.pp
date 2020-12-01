{ %skiptarget=win64 }
{ %cpu=x86_64 }
{ %opt=-vw -Sew -Cg }
{ %fail }

{$asmmode intel}

var
  val: qword; public;

Function Test: QWord; Assembler; NoStackFrame;
Asm
 { global symbols must be accessed via the GOT on non-win64 }
 mov RAX, [RIP+val]
End;

BEGIN
  Val := $12345678901;
  if test<>$12345678901 then
    halt(1);
END.

