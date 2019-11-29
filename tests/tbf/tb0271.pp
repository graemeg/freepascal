{ %fail }
{ %cpu=x86_64 }
begin
  asm
    leaq (%rip,%rax),%rax
  end;
end.
