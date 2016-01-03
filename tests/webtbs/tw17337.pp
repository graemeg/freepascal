{ %target=win64, linux, freebsd }
{ %cpu=x86_64 }
{ %opt=-Aas }

{$asmmode att}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure test_gas;
=======
procedure test_gas;   
>>>>>>> graemeg/cpstrnew
=======
procedure test_gas;   
>>>>>>> graemeg/cpstrnew
=======
procedure test_gas;   
>>>>>>> graemeg/cpstrnew
=======
procedure test_gas;   
>>>>>>> origin/cpstrnew
var
  test : qword;
begin
  test:=$5ffffffff;
  if (test < qword($2ffffffff)) then
    runerror(1);
  if (test < qword($ffffffff)) then
    runerror(2);
asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  movq $0xffffffff,%rax
  movq %rax,test
=======
  movq $0xffffffff,test
>>>>>>> graemeg/cpstrnew
=======
  movq $0xffffffff,test
>>>>>>> graemeg/cpstrnew
=======
  movq $0xffffffff,test
>>>>>>> graemeg/cpstrnew
=======
  movq $0xffffffff,test
>>>>>>> origin/cpstrnew
end;
  if test <> $ffffffff then
    runerror(5);
end ;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/cpstrnew
=======
  
>>>>>>> graemeg/cpstrnew
=======
  
>>>>>>> graemeg/cpstrnew
=======
  
>>>>>>> origin/cpstrnew
var
  test : qword;
begin
  test:=$5ffffffff;
  if (test < qword($2ffffffff)) then
    runerror(3);
  if (test < qword($ffffffff)) then
    runerror(4);
  test_gas;
end.
