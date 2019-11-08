{ %cpu=x86_64 }
{ %norun }
program project1;

{$mode objfpc}{$H+}
{$ASMMODE intel}

begin
  asm
    vaddsd	xmm0, xmm0, qword ptr [rax]
    vaddsd	xmm0, xmm0, qword ptr [rax + 8]
  end;
end.
