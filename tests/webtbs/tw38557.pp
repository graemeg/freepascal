{ %cpu=x86_64 }
{ %fail }
{$asmmode intel}
type
     gfxImage = record
          data: pointer;
          width, height: dWord;
     end;

function putPixel(where:gfxImage;x,y,col:dword):dword; assembler; nostackframe;
//begin
asm
     xor eax,eax
     mov eax,x
     mov eax,y
     mov eax,col
     mov rax,rdi
     mov rax,rsi
     rol rax,32

     mov rax, where
     //mov rax, where.data
     //mov rax, where
end;
//end;

var a : gfxImage;
    z : dword;
begin
     a.data:= pointer(5);
     a.width := 8;
     a.height := 7;
     z:= putPixel (a, 1, 2, 3);
     writeln;
     writeln(' ',z,' ');
     writeln;
end.
