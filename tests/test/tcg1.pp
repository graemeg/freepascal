{ %CPU=i386 }
{$R-}
program test_register_pushing;

const
  haserror : boolean = false;
  
procedure dotest;
var
  wpush,lpush: longint;
begin
{$asmmode att}
  asm
<<<<<<< HEAD
<<<<<<< HEAD
    movl   %esp,wpush
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_PIC}
    movl   %esp,before
>>>>>>> graemeg/fixes_2_2
    pushw  %es
    subl   %esp,wpush
    popw   %es
{$else not FPC_PIC}
    call   .LPIC
.LPIC:
    popl   %ecx
{$ifdef darwin}
    movl   %esp,before-.LPIC(%ecx)
    pushw  %es
    movl   %esp,after-.LPIC(%ecx)
    popw   %es
{$else darwin}
    addl   $_GLOBAL_OFFSET_TABLE_,%ecx
    movl   %esp,before@GOT(%ecx)
    pushw  %es
    movl   %esp,after@GOT(%ecx)
    popw   %es
<<<<<<< HEAD
=======
{$else not FPC_PIC}
    call   .LPIC
.LPIC:
    popl   %ecx
{$ifdef darwin}
    movl   %esp,before-.LPIC(%ecx)
    pushw  %es
    movl   %esp,after-.LPIC(%ecx)
    popw   %es
{$else darwin}
    addl   $_GLOBAL_OFFSET_TABLE_,%ecx
    movl   %esp,before@GOT(%ecx)
    pushw  %es
    movl   %esp,after@GOT(%ecx)
    popw   %es
>>>>>>> origin/fixes_2_2
{$endif darwin}
{$endif not FPC_PIC}
  end;
  if wpush<>2 then
    begin
      Writeln('Compiler does not push "pushw %es" into 2 bytes');
      haserror:=true;
    end;
    
  asm
<<<<<<< HEAD
<<<<<<< HEAD
    movl   %esp,lpush
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_PIC}
    movl   %esp,before
>>>>>>> graemeg/fixes_2_2
    pushl  %es
    subl   %esp,lpush
    popl   %es
{$else not FPC_PIC}
    call   .LPIC
.LPIC:
    popl   %ecx
{$ifdef darwin}
    movl   %esp,before-.LPIC(%ecx)
    pushl  %es
    movl   %esp,after-.LPIC(%ecx)
    popl   %es
{$else darwin}
    addl   $_GLOBAL_OFFSET_TABLE_,%ecx
    movl   %esp,before@GOT(%ecx)
    pushl  %es
    movl   %esp,after@GOT(%ecx)
    popl   %es
{$endif darwin}
{$endif not FPC_PIC}
  end;
  if lpush<>4 then
    begin
      Writeln('Compiler does not push "pushl %es" into 4 bytes');
      haserror:=true;
    end;

  asm
<<<<<<< HEAD
<<<<<<< HEAD
    movl   %esp,wpush
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_PIC}
    movl   %esp,before
>>>>>>> graemeg/fixes_2_2
    pushw  %gs
    subl   %esp,wpush
    popw   %gs
{$else not FPC_PIC}
    call   .LPIC
.LPIC:
    popl   %ecx
{$ifdef darwin}
    movl   %esp,before-.LPIC(%ecx)
    pushw  %gs
    movl   %esp,after-.LPIC(%ecx)
    popw   %gs
<<<<<<< HEAD
=======
{$else not FPC_PIC}
    call   .LPIC
.LPIC:
    popl   %ecx
{$ifdef darwin}
    movl   %esp,before-.LPIC(%ecx)
    pushw  %gs
    movl   %esp,after-.LPIC(%ecx)
    popw   %gs
>>>>>>> origin/fixes_2_2
{$else darwin}
    addl   $_GLOBAL_OFFSET_TABLE_,%ecx
    movl   %esp,before@GOT(%ecx)
    pushw  %gs
    movl   %esp,after@GOT(%ecx)
    popw   %gs
{$endif darwin}
{$endif not FPC_PIC}
  end;
  if wpush<>2 then
    begin
      Writeln('Compiler does not push "pushw %gs" into 2 bytes');
      haserror:=true;
    end;
  asm
<<<<<<< HEAD
<<<<<<< HEAD
    movl   %esp,lpush
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_PIC}
    movl   %esp,before
>>>>>>> graemeg/fixes_2_2
    pushl  %gs
    subl   %esp,lpush
    popl   %gs
{$else not FPC_PIC}
    call   .LPIC
.LPIC:
    popl   %ecx
{$ifdef darwin}
    movl   %esp,before-.LPIC(%ecx)
    pushl  %gs
    movl   %esp,after-.LPIC(%ecx)
    popl   %gs
<<<<<<< HEAD
=======
{$else not FPC_PIC}
    call   .LPIC
.LPIC:
    popl   %ecx
{$ifdef darwin}
    movl   %esp,before-.LPIC(%ecx)
    pushl  %gs
    movl   %esp,after-.LPIC(%ecx)
    popl   %gs
>>>>>>> origin/fixes_2_2
{$else darwin}
    addl   $_GLOBAL_OFFSET_TABLE_,%ecx
    movl   %esp,before@GOT(%ecx)
    pushl  %gs
    movl   %esp,after@GOT(%ecx)
    popl   %gs
{$endif darwin}
{$endif not FPC_PIC}
  end;
  if lpush<>4 then
    begin
      Writeln('Compiler does not push "pushl %gs" into 4 bytes');
      haserror:=true;
    end;
{$asmmode intel}
  asm
<<<<<<< HEAD
<<<<<<< HEAD
    mov    lpush,esp
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_PIC}
    mov    before,esp
>>>>>>> graemeg/fixes_2_2
    push   es
    sub    lpush,esp
    pop    es
{$else not FPC_PIC}
    call   @@LPIC
@@LPIC:
    pop    ecx
{$ifdef darwin}
    mov    [before-@@LPIC+ecx],esp
    push   es
    mov    [after-@@LPIC+ecx],esp
    pop    es
{$else darwin}
    add    ecx,@_GLOBAL_OFFSET_TABLE_
    mov    [ecx].OFFSET before,esp
    push   es
    mov    [ecx].OFFSET after,esp
    pop    es
{$endif darwin}
{$endif not FPC_PIC}
  end;
  Writeln('Intel "push es" uses ',lpush,' bytes');
  if haserror then
    Halt(1);
end;

begin
  dotest;
end.
