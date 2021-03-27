{ %cpu=x86_64 }
function test : int64;assembler;
  asm
        MOV $'=TXEHTAP',%RAX // FAIL: LONG STRING "PATHEXT="
  end;
var
  s : string[8];
  l : int64 absolute s[1];
begin
  l:=test;
  s[0]:=#8;
  if s<>'PATHEXT=' then
    halt(1);
end.
