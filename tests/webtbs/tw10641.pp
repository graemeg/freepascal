{ %cpu=x86_64 }
<<<<<<< HEAD
<<<<<<< HEAD
{ %opt=-vw -Sew }

{$ifdef windows}
  {$imagebase $10000}
{$endif}
var
  test : dword;
  test2 : dword;
begin
  test:=$deadbeef;
  test2:=$deadbeef;
  ASM
    MOVL $16,%EAX
    LEA .LLT(%RIP),%RBX
    JMP (%RBX,%RAX)
    .balign 16
.LLT:
    .quad .L3,.L2,.L1
.L2:
    MOVL $12341234,test2(%RIP)
    JMP  .L3
.L1:
    MOVL $0,test(%RIP)
.L3:
  END;
  if (test<>0) or (test2<>$deadbeef) then
=======
=======
>>>>>>> origin/fixes_2_2
var
  test : dword;

begin
  test:=$deadbeef;
  ASM
    MOVL $0,%EAX
    JMP .LLT(%RAX)
    .align 16
.LLT:
    .quad .L1,.L2
.L2:
    MOVL $12341234,test
.L1:
    MOVL $0,test
  END;
  if test<>0 then
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    halt(1);
  writeln('ok');
end.
