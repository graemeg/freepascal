{ %cpu=i386,x86_64 }
{ %norun }
{ %opt=-Rintel -O4 -a }

function foo ( w : word):byte; assembler;
asm
     mov ax, w

end;

begin
     foo(3);
end.
