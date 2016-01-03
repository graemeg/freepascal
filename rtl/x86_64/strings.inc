{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by Florian Klaempfl, member of the
    Free Pascal development team

    Processor dependent part of strings.pp, that can be shared with
    sysutils unit.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$ASMMODE GAS}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{$ifndef FPC_UNIT_HAS_STRCOPY}
{$define FPC_UNIT_HAS_STRCOPY}
{ Created from glibc: libc/sysdeps/x86_64/strcpy.S Version 1.2 }
function strcopy(dest,source : pchar) : pchar;assembler;
{$ifdef win64}
var
  rdi,rsi : int64;
{$endif win64}
asm
{$ifdef win64}
        movq %rsi,rsi
        movq %rdi,rdi
        movq %rdx, %rsi
        movq %rcx, %rdi
{$endif win64}
        movq %rsi, %rcx                { Source register. }
        andl $7, %ecx                  { mask alignment bits }
        movq %rdi, %rdx                { Duplicate destination pointer.  }

        jz .LFPC_STRCOPY_5              { aligned => start loop }

        neg %ecx                       { We need to align to 8 bytes.  }
        addl $8,%ecx

        { Search the first bytes directly.  }
.LFPC_STRCOPY_0:
        movb        (%rsi), %al        { Fetch a byte }
        testb       %al, %al           { Is it NUL? }
        movb        %al, (%rdx)        { Store it }
        jz          .LFPC_STRCOPY_4     { If it was NUL, done! }
        incq        %rsi
        incq        %rdx
        decl        %ecx
        jnz         .LFPC_STRCOPY_0

.LFPC_STRCOPY_5:
        movq        $0xfefefefefefefeff,%r8

        { Now the sources is aligned.  Unfortunatly we cannot force
           to have both source and destination aligned, so ignore the
           alignment of the destination.  }
        .p2align 4
.LFPC_STRCOPY_1:
        { 1st unroll.  }
        movq        (%rsi), %rax       { Read double word (8 bytes).  }
        addq        $8, %rsi           { Adjust pointer for next word.  }
        movq        %rax, %r9          { Save a copy for NUL finding.  }
        addq        %r8, %r9           { add the magic value to the word.  We get
                                         carry bits reported for each byte which
                                         is *not* 0 }
        jnc         .LFPC_STRCOPY_3     { highest byte is NUL => return pointer }
        xorq        %rax, %r9          { (word+magic)^word }
        orq         %r8, %r9           { set all non-carry bits }
        incq        %r9                { add 1: if one carry bit was *not* set
                                         the addition will not result in 0.  }

        jnz         .LFPC_STRCOPY_3                { found NUL => return pointer }

        movq        %rax, (%rdx)        { Write value to destination.  }
        addq        $8, %rdx        { Adjust pointer.  }

        { 2nd unroll.  }
        movq        (%rsi), %rax        { Read double word (8 bytes).  }
        addq        $8, %rsi        { Adjust pointer for next word.  }
        movq        %rax, %r9        { Save a copy for NUL finding.  }
        addq        %r8, %r9        { add the magic value to the word.  We get
                                   carry bits reported for each byte which
                                   is *not* 0 }
        jnc         .LFPC_STRCOPY_3                { highest byte is NUL => return pointer }
        xorq        %rax, %r9        { (word+magic)^word }
        orq         %r8, %r9        { set all non-carry bits }
        incq        %r9                { add 1: if one carry bit was *not* set
                                   the addition will not result in 0.  }

        jnz         .LFPC_STRCOPY_3                { found NUL => return pointer }

        movq        %rax, (%rdx)        { Write value to destination.  }
        addq        $8, %rdx        { Adjust pointer.  }

        { 3rd unroll.  }
        movq        (%rsi), %rax        { Read double word (8 bytes).  }
        addq        $8, %rsi        { Adjust pointer for next word.  }
        movq        %rax, %r9        { Save a copy for NUL finding.  }
        addq        %r8, %r9        { add the magic value to the word.  We get
                                   carry bits reported for each byte which
                                   is *not* 0 }
        jnc        .LFPC_STRCOPY_3                { highest byte is NUL => return pointer }
        xorq        %rax, %r9        { (word+magic)^word }
        orq        %r8, %r9        { set all non-carry bits }
        incq        %r9                { add 1: if one carry bit was *not* set
                                   the addition will not result in 0.  }

        jnz         .LFPC_STRCOPY_3                { found NUL => return pointer }

        movq        %rax, (%rdx)        { Write value to destination.  }
        addq        $8, %rdx        { Adjust pointer.  }

        { 4th unroll.  }
        movq        (%rsi), %rax        { Read double word (8 bytes).  }
        addq        $8, %rsi        { Adjust pointer for next word.  }
        movq        %rax, %r9        { Save a copy for NUL finding.  }
        addq        %r8, %r9        { add the magic value to the word.  We get
                                   carry bits reported for each byte which
                                   is *not* 0 }
        jnc         .LFPC_STRCOPY_3                { highest byte is NUL => return pointer }
        xorq        %rax, %r9        { (word+magic)^word }
        orq         %r8, %r9        { set all non-carry bits }
        incq        %r9                { add 1: if one carry bit was *not* set
                                   the addition will not result in 0.  }

        jnz         .LFPC_STRCOPY_3                { found NUL => return pointer }

        movq        %rax, (%rdx)        { Write value to destination.  }
        addq        $8, %rdx        { Adjust pointer.  }
        jmp         .LFPC_STRCOPY_1                { Next iteration.  }

        { Do the last few bytes. %rax contains the value to write.
           The loop is unrolled twice.  }
        .p2align 4
.LFPC_STRCOPY_3:
        { Note that stpcpy needs to return with the value of the NUL
           byte.  }
        movb        %al, (%rdx)        { 1st byte.  }
        testb       %al, %al        { Is it NUL.  }
        jz          .LFPC_STRCOPY_4                { yes, finish.  }
        incq        %rdx                { Increment destination.  }
        movb        %ah, (%rdx)        { 2nd byte.  }
        testb       %ah, %ah        { Is it NUL?.  }
        jz          .LFPC_STRCOPY_4                { yes, finish.  }
        incq        %rdx                { Increment destination.  }
        shrq        $16, %rax        { Shift...  }
        jmp         .LFPC_STRCOPY_3                { and look at next two bytes in %rax.  }

.LFPC_STRCOPY_4:
        movq        %rdi, %rax        { Source is return value.  }
{$ifdef win64}
        movq rsi,%rsi
        movq rdi,%rdi
{$endif win64}
end;
{$endif FPC_UNIT_HAS_STRCOPY}

>>>>>>> graemeg/cpstrnew

{$ifndef FPC_UNIT_HAS_STRCOMP}
{$define FPC_UNIT_HAS_STRCOMP}
function StrComp(Str1, Str2: PChar): SizeInt;assembler;nostackframe;
asm
{$ifndef win64}
        movq   %rsi,%rdx
        movq   %rdi,%rcx
{$endif win64}
        subq   %rcx,%rdx
.balign 16
.Lloop:                          { unrolled 4 times }
        movb   (%rcx),%al
        cmpb   (%rdx,%rcx),%al
        jne    .Ldiff
        testb  %al,%al
        jz    .Leq
        movb   1(%rcx),%al
        cmpb   1(%rdx,%rcx),%al
        jne    .Ldiff
        testb  %al,%al
        jz     .Leq
        movb   2(%rcx),%al
        cmpb   2(%rdx,%rcx),%al
        jne    .Ldiff
        testb  %al,%al
        jz     .Leq
        movb   3(%rcx),%al
        add    $4,%rcx
        cmpb   -1(%rdx,%rcx),%al
        jne    .Ldiff
        testb  %al,%al
        jnz    .Lloop
.Leq:
        xorq   %rax,%rax
        jmp    .Lexit

.Ldiff:
        sbbq   %rax,%rax        { -1 if CF was set, 0 otherwise }
        orb    $1,%al           { 0 becomes 1, -1 remains unchanged }
.Lexit:
end;
{$endif FPC_UNIT_HAS_STRCOMP}
