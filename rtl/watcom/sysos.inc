<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{*****************************************************************************
                             Watcom Helpers
*****************************************************************************}
const
  carryflag = 1;

type
  tseginfo=packed record
    offset  : pointer;
    segment : word;
  end;

var
  old_int00 : tseginfo;cvar;
  old_int75 : tseginfo;cvar;


procedure getinoutres(def : word);
var
  regs : trealregs;
begin
  regs.realeax:=$5900;
  regs.realebx:=$0;
  sysrealintr($21,regs);
  InOutRes:=lo(regs.realeax);
  case InOutRes of
   19 : InOutRes:=150;
   21 : InOutRes:=152;
   32 : InOutRes:=5;
  end;
  if InOutRes=0 then
    InOutRes:=Def;
end;


function far_strlen(selector : word;linear_address : sizeuint) : longint;assembler;
asm
        movl linear_address,%edx
        movl %edx,%ecx
        movw selector,%gs
.Larg19:
        movb %gs:(%edx),%al
        testb %al,%al
        je .Larg20
        incl %edx
        jmp .Larg19
.Larg20:
        movl %edx,%eax
        subl %ecx,%eax
end;


function get_ds : word;assembler;
asm
        movw    %ds,%ax
end;


function get_cs : word;assembler;
asm
        movw    %cs,%ax
end;

function dos_selector : word; assembler;
asm
   movw %ds,%ax  { no separate selector needed }
end;

procedure alloc_tb; assembler;
{ allocate 8kB real mode transfer buffer }
asm
   pushl %ebx
   movw $0x100,%ax
   movw $512,%bx
   int $0x31
   movw %ax,tb_segment
   shll $16,%eax
   shrl $12,%eax
   movl %eax,tb
   popl %ebx
end;

procedure sysseg_move(sseg : word;source : sizeuint;dseg : word;dest : sizeuint;count : longint);
begin
   if count=0 then
     exit;
   if (sseg<>dseg) or ((sseg=dseg) and (source>dest)) then
     asm
        pushl %esi
        pushl %edi
        pushw %es
        pushw %ds
        cld
        movl count,%ecx
        movl source,%esi
        movl dest,%edi
        movw dseg,%ax
        movw %ax,%es
        movw sseg,%ax
        movw %ax,%ds
        movl %ecx,%eax
        shrl $2,%ecx
        rep
        movsl
        movl %eax,%ecx
        andl $3,%ecx
        rep
        movsb
        popw %ds
        popw %es
        popl %edi
        popl %esi
     end
   else if (source<dest) then
     { copy backward for overlapping }
     asm
        pushl %esi
        pushl %edi
        pushw %es
        pushw %ds
        std
        movl count,%ecx
        movl source,%esi
        movl dest,%edi
        movw dseg,%ax
        movw %ax,%es
        movw sseg,%ax
        movw %ax,%ds
        addl %ecx,%esi
        addl %ecx,%edi
        movl %ecx,%eax
        andl $3,%ecx
        orl %ecx,%ecx
        jz .LSEG_MOVE1

        { calculate esi and edi}
        decl %esi
        decl %edi
        rep
        movsb
        incl %esi
        incl %edi
     .LSEG_MOVE1:
        subl $4,%esi
        subl $4,%edi
        movl %eax,%ecx
        shrl $2,%ecx
        rep
        movsl
        cld
        popw %ds
        popw %es
        popl %edi
        popl %esi
     end;
end;

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{*****************************************************************************
                             Watcom Helpers
*****************************************************************************}
const
  carryflag = 1;

type
  tseginfo=packed record
    offset  : pointer;
    segment : word;
  end;

var
  old_int00 : tseginfo;cvar;
  old_int75 : tseginfo;cvar;


procedure getinoutres(def : word);
var
  regs : trealregs;
begin
  regs.realeax:=$5900;
  regs.realebx:=$0;
  sysrealintr($21,regs);
  InOutRes:=lo(regs.realeax);
  case InOutRes of
   19 : InOutRes:=150;
   21 : InOutRes:=152;
   32 : InOutRes:=5;
  end;
  if InOutRes=0 then
    InOutRes:=Def;
end;


function far_strlen(selector : word;linear_address : sizeuint) : longint;assembler;
asm
        movl linear_address,%edx
        movl %edx,%ecx
        movw selector,%gs
.Larg19:
        movb %gs:(%edx),%al
        testb %al,%al
        je .Larg20
        incl %edx
        jmp .Larg19
.Larg20:
        movl %edx,%eax
        subl %ecx,%eax
end;


function get_ds : word;assembler;
asm
        movw    %ds,%ax
end;


function get_cs : word;assembler;
asm
        movw    %cs,%ax
end;

function dos_selector : word; assembler;
asm
   movw %ds,%ax  { no separate selector needed }
end;

procedure alloc_tb; assembler;
{ allocate 8kB real mode transfer buffer }
asm
   pushl %ebx
   movw $0x100,%ax
   movw $512,%bx
   int $0x31
   movw %ax,tb_segment
   shll $16,%eax
   shrl $12,%eax
   movl %eax,tb
   popl %ebx
end;

procedure sysseg_move(sseg : word;source : sizeuint;dseg : word;dest : sizeuint;count : longint);
begin
   if count=0 then
     exit;
   if (sseg<>dseg) or ((sseg=dseg) and (source>dest)) then
     asm
        pushl %esi
        pushl %edi
        pushw %es
        pushw %ds
        cld
        movl count,%ecx
        movl source,%esi
        movl dest,%edi
        movw dseg,%ax
        movw %ax,%es
        movw sseg,%ax
        movw %ax,%ds
        movl %ecx,%eax
        shrl $2,%ecx
        rep
        movsl
        movl %eax,%ecx
        andl $3,%ecx
        rep
        movsb
        popw %ds
        popw %es
        popl %edi
        popl %esi
     end
   else if (source<dest) then
     { copy backward for overlapping }
     asm
        pushl %esi
        pushl %edi
        pushw %es
        pushw %ds
        std
        movl count,%ecx
        movl source,%esi
        movl dest,%edi
        movw dseg,%ax
        movw %ax,%es
        movw sseg,%ax
        movw %ax,%ds
        addl %ecx,%esi
        addl %ecx,%edi
        movl %ecx,%eax
        andl $3,%ecx
        orl %ecx,%ecx
        jz .LSEG_MOVE1

        { calculate esi and edi}
        decl %esi
        decl %edi
        rep
        movsb
        incl %esi
        incl %edi
     .LSEG_MOVE1:
        subl $4,%esi
        subl $4,%edi
        movl %eax,%ecx
        shrl $2,%ecx
        rep
        movsl
        cld
        popw %ds
        popw %es
        popl %edi
        popl %esi
     end;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
