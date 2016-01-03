<<<<<<< HEAD
<<<<<<< HEAD
{ %skiptarget=darwin}
{ %cpu=i386 }
{ %OPT=-Cg- }
=======
{ %cpu=i386 }
>>>>>>> graemeg/fixes_2_2
=======
{ %cpu=i386 }
>>>>>>> origin/fixes_2_2
var
  w : word;
  b1,b2,b3 : byte;
begin
  b1:=$aa;
  b2:=$55;
  b3:=$aa;
{$asmmode intel}
  asm
    mov eax,$deadbeef
    mov ax,w
    shr eax,16
    mov w,ax
    mov bl,$55
    lea edx,b2
    test [edx],bl
    setnz b1
  end;
  if w<>$dead then
    halt(1);
  if b1<>1 then
    halt(1);
  writeln('ok');
end.
