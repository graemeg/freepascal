{ %fail }
{ %CPU=x86_64 }
begin
  asm
    movq (%rdi,%rsp),%rax
  end;
end.
