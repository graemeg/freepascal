{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 by Florian Klaempfl and Sergei Gorelkin
    Members of the Free Pascal development team

    Processor dependent implementation for the system unit for
    the x86-64 architecture

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$asmmode GAS}

{****************************************************************************
                               Primitives
****************************************************************************}

{$define FPC_SYSTEM_HAS_SPTR}
Function Sptr : Pointer;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
        movq    %rsp,%rax
end;

{$IFNDEF INTERNAL_BACKTRACE}
{$define FPC_SYSTEM_HAS_GET_FRAME}
function get_frame:pointer;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
        movq    %rbp,%rax
end;
{$ENDIF not INTERNAL_BACKTRACE}

{$define FPC_SYSTEM_HAS_GET_PC_ADDR}
function get_pc_addr:pointer;assembler;nostackframe;
asm
        movq    (%rsp),%rax
end;

{$define FPC_SYSTEM_HAS_GET_CALLER_ADDR}
function get_caller_addr(framebp:pointer;addr:pointer=nil):pointer;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  get_caller_addr:=framebp;
  if assigned(framebp) then
    get_caller_addr:=PPointer(framebp)[1];
end;


{$define FPC_SYSTEM_HAS_GET_CALLER_FRAME}
function get_caller_frame(framebp:pointer;addr:pointer=nil):pointer;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  get_caller_frame:=framebp;
  if assigned(framebp) then
    get_caller_frame:=PPointer(framebp)^;
end;

// The following assembler procedures are disabled for FreeBSD due to
// multiple issues with its old GNU assembler (Mantis #19188).
// Even after fixing them, it can be enabled only for the trunk version,
// otherwise bootstrapping won't be possible.
// Modified to use oldbinutils as in cpu.pp source, to allow easier use for other targets.
{$ifdef freebsd}
  {$ifndef overridebinutils}
    {$define oldbinutils}
  {$endif}
{$endif freebsd}

{$ifndef oldbinutils}


{$ifndef FPC_SYSTEM_HAS_MOVE}
{$define FPC_SYSTEM_HAS_MOVE}
procedure Move(const source;var dest;count:SizeInt);[public, alias: 'FPC_MOVE'];assembler;nostackframe;
{ Linux: rdi source, rsi dest, rdx count
  win64: rcx source, rdx dest, r8 count }
asm
{$ifndef win64}
    mov    %rdx, %r8
    mov    %rsi, %rdx
    mov    %rdi, %rcx
{$endif win64}

    mov    %r8, %rax
    sub    %rdx, %rcx            { rcx = src - dest }
    jz     .Lquit                { exit if src=dest }
    jnb    .L1                   { src>dest => forward move }

    add    %rcx, %rax            { rcx is negative => r8+rcx > 0 if regions overlap }
    jb     .Lback                { if no overlap, still do forward move }

.L1:
    cmp    $8, %r8
    jl     .Lless8f              { signed compare, negative count not allowed }
    test   $7, %dl
    je     .Ldestaligned

    test   $1, %dl               { align dest by moving first 1+2+4 bytes }
    je     .L2f
    mov    (%rcx,%rdx,1),%al
    dec    %r8
    mov    %al, (%rdx)
    add    $1, %rdx
.L2f:
    test   $2, %dl
    je     .L4f
    mov    (%rcx,%rdx,1),%ax
    sub    $2, %r8
    mov    %ax, (%rdx)
    add    $2, %rdx
.L4f:
    test   $4, %dl
    je     .Ldestaligned
    mov    (%rcx,%rdx,1),%eax
    sub    $4, %r8
    mov    %eax, (%rdx)
    add    $4, %rdx

.Ldestaligned:
    mov    %r8, %r9
    shr    $5, %r9
    jne    .Lmore32

.Ltail:
    mov    %r8, %r9
    shr    $3, %r9
    je     .Lless8f

    .balign 16
.Lloop8f:                             { max. 8 iterations }
    mov    (%rcx,%rdx,1),%rax
    mov    %rax, (%rdx)
    add    $8, %rdx
    dec    %r9
    jne    .Lloop8f
    and    $7, %r8

.Lless8f:
    test   %r8, %r8
    jle    .Lquit

    .balign 16
.Lloop1f:
    mov    (%rcx,%rdx,1),%al
    mov    %al,(%rdx)
    inc    %rdx
    dec    %r8
    jne    .Lloop1f
.Lquit:
    retq


.Lmore32:
    cmp    $0x2000, %r9          { this limit must be processor-specific (1/2 L2 cache size) }
    jnae   .Lloop32
    cmp    $0x1000, %rcx         { but don't bother bypassing cache if src and dest }
    jnb    .Lntloopf             { are close to each other}

    .balign 16
.Lloop32:
    add    $32,%rdx
    mov    -32(%rcx,%rdx,1),%rax
    mov    -24(%rcx,%rdx,1),%r10
    mov    %rax,-32(%rdx)
    mov    %r10,-24(%rdx)
    dec    %r9
    mov    -16(%rcx,%rdx,1),%rax
    mov    -8(%rcx,%rdx,1),%r10
    mov    %rax,-16(%rdx)
    mov    %r10,-8(%rdx)
    jne    .Lloop32

    and    $0x1f, %r8
    jmpq   .Ltail

.Lntloopf:
    mov    $32, %eax

    .balign 16
.Lpref:
    prefetchnta (%rcx,%rdx,1)
    prefetchnta 0x40(%rcx,%rdx,1)
    add    $0x80, %rdx
    dec    %eax
    jne    .Lpref

    sub    $0x1000, %rdx
    mov    $64, %eax

    .balign 16
.Loop64:
    add    $64, %rdx
    mov    -64(%rcx,%rdx,1), %r9
    mov    -56(%rcx,%rdx,1), %r10
    movnti %r9, -64(%rdx)
    movnti %r10, -56(%rdx)

    mov    -48(%rcx,%rdx,1), %r9
    mov    -40(%rcx,%rdx,1), %r10
    movnti %r9, -48(%rdx)
    movnti %r10, -40(%rdx)
    dec    %eax
    mov    -32(%rcx,%rdx,1), %r9
    mov    -24(%rcx,%rdx,1), %r10
    movnti %r9, -32(%rdx)
    movnti %r10, -24(%rdx)

    mov    -16(%rcx,%rdx,1), %r9
    mov    -8(%rcx,%rdx,1), %r10
    movnti %r9, -16(%rdx)
    movnti %r10, -8(%rdx)
    jne    .Loop64

    sub    $0x1000, %r8
    cmp    $0x1000, %r8
    jae    .Lntloopf

    mfence
    jmpq    .Ldestaligned        { go handle remaining bytes }

{ backwards move }
.Lback:
    add    %r8, %rdx             { points to the end of dest }
    cmp    $8, %r8
    jl     .Lless8b              { signed compare, negative count not allowed }
    test   $7, %dl
    je     .Ldestalignedb
    test   $1, %dl
    je     .L2b
    dec    %rdx
    mov    (%rcx,%rdx,1), %al
    dec    %r8
    mov    %al, (%rdx)
.L2b:
    test   $2, %dl
    je     .L4b
    sub    $2, %rdx
    mov    (%rcx,%rdx,1), %ax
    sub    $2, %r8
    mov    %ax, (%rdx)
.L4b:
    test   $4, %dl
    je     .Ldestalignedb
    sub    $4, %rdx
    mov    (%rcx,%rdx,1), %eax
    sub    $4, %r8
    mov    %eax, (%rdx)

.Ldestalignedb:
    mov    %r8, %r9
    shr    $5, %r9
    jne    .Lmore32b

.Ltailb:
    mov    %r8, %r9
    shr    $3, %r9
    je     .Lless8b

.Lloop8b:
    sub    $8, %rdx
    mov    (%rcx,%rdx,1), %rax
    dec    %r9
    mov    %rax, (%rdx)
    jne    .Lloop8b
    and    $7, %r8

.Lless8b:
    test   %r8, %r8
    jle    .Lquit2

    .balign 16
.Lsmallb:
    dec   %rdx
    mov   (%rcx,%rdx,1), %al
    dec   %r8
    mov   %al,(%rdx)
    jnz   .Lsmallb
.Lquit2:
    retq

.Lmore32b:
    cmp   $0x2000, %r9
    jnae  .Lloop32b
    cmp    $0xfffffffffffff000,%rcx
    jb     .Lntloopb

    .balign 16
.Lloop32b:
    sub    $32, %rdx
    mov    24(%rcx,%rdx,1), %rax
    mov    16(%rcx,%rdx,1), %r10
    mov    %rax, 24(%rdx)
    mov    %r10, 16(%rdx)
    dec    %r9
    mov    8(%rcx,%rdx,1),%rax
    mov    (%rcx,%rdx,1), %r10
    mov    %rax, 8(%rdx)
    mov    %r10, (%rdx)
    jne    .Lloop32b
    and    $0x1f, %r8
    jmpq   .Ltailb


.Lntloopb:
    mov    $32, %eax

    .balign 16
.Lprefb:
    sub    $0x80, %rdx
    prefetchnta (%rcx,%rdx,1)
    prefetchnta 0x40(%rcx,%rdx,1)
    dec    %eax
    jnz    .Lprefb

    add    $0x1000, %rdx
    mov    $0x40, %eax

    .balign 16
.Lloop64b:
    sub    $64, %rdx
    mov    56(%rcx,%rdx,1), %r9
    mov    48(%rcx,%rdx,1), %r10
    movnti %r9, 56(%rdx)
    movnti %r10, 48(%rdx)

    mov    40(%rcx,%rdx,1), %r9
    mov    32(%rcx,%rdx,1), %r10
    movnti %r9, 40(%rdx)
    movnti %r10, 32(%rdx)
    dec    %eax
    mov    24(%rcx,%rdx,1), %r9
    mov    16(%rcx,%rdx,1), %r10
    movnti %r9, 24(%rdx)
    movnti %r10, 16(%rdx)

    mov    8(%rcx,%rdx,1), %r9
    mov    (%rcx,%rdx,1), %r10
    movnti %r9, 8(%rdx)
    movnti %r10, (%rdx)
    jne    .Lloop64b

    sub    $0x1000, %r8
    cmp    $0x1000, %r8
    jae    .Lntloopb
    mfence
    jmpq   .Ldestalignedb
end;
{$endif FPC_SYSTEM_HAS_MOVE}

{$ifndef FPC_SYSTEM_HAS_FILLCHAR}
{$define FPC_SYSTEM_HAS_FILLCHAR}
Procedure FillChar(var x;count:SizeInt;value:byte);assembler;nostackframe;
  asm
{ win64: rcx dest, rdx count, r8b value
  linux: rdi dest, rsi count, rdx value }
{$ifndef win64}
    mov    %rdx, %r8
    mov    %rsi, %rdx
    mov    %rdi, %rcx
{$endif win64}

    cmp    $8, %rdx
    jl     .Ltiny

// TODO: movz?q and movs?q are not accepted by FPC asmreader, it needs fixing.
// `movzbl' instead is accepted and generates correct code with internal assembler,
// but breaks targets using external GAS (Mantis #19188).
// So use a different instruction for now.

    { expand byte value  }
    andq   $0xff, %r8
{
    movzbq %r8b, %r8
}
    mov    $0x0101010101010101,%r9
    imul   %r9, %r8

    test   $7, %cl
    je     .Laligned

    { align dest to 8 bytes }
    test   $1, %cl
    je     .L2
    movb   %r8b, (%rcx)
    add    $1, %rcx
    sub    $1, %rdx
.L2:
    test   $2, %cl
    je     .L4
    movw   %r8w, (%rcx)
    add    $2, %rcx
    sub    $2, %rdx
.L4:
    test   $4, %cl
    je     .Laligned
    movl   %r8d, (%rcx)
    add    $4, %rcx
    sub    $4, %rdx

.Laligned:
    mov    %rdx, %rax
    and    $0x3f, %rdx
    shr    $6, %rax
    jne    .Lmore64

.Lless64:
    mov    %rdx, %rax
    and    $7, %rdx
    shr    $3, %rax
    je     .Ltiny

    .balign 16
.Lloop8:                               { max. 8 iterations }
    mov    %r8, (%rcx)
    add    $8, %rcx
    dec    %rax
    jne    .Lloop8
.Ltiny:
    test   %rdx, %rdx
    jle    .Lquit
.Lloop1:
    movb   %r8b, (%rcx)
    inc    %rcx
    dec    %rdx
    jnz    .Lloop1
.Lquit:
    retq

.Lmore64:
    cmp    $0x2000,%rax
    jae    .Lloop64nti

    .balign 16
.Lloop64:
    add    $64, %rcx
    mov    %r8, -64(%rcx)
    mov    %r8, -56(%rcx)
    mov    %r8, -48(%rcx)
    mov    %r8, -40(%rcx)
    dec    %rax
    mov    %r8, -32(%rcx)
    mov    %r8, -24(%rcx)
    mov    %r8, -16(%rcx)
    mov    %r8, -8(%rcx)
    jne    .Lloop64
    jmp    .Lless64

    .balign 16
.Lloop64nti:
    add    $64, %rcx
    movnti %r8, -64(%rcx)
    movnti %r8, -56(%rcx)
    movnti %r8, -48(%rcx)
    movnti %r8, -40(%rcx)
    dec    %rax
    movnti %r8, -32(%rcx)
    movnti %r8, -24(%rcx)
    movnti %r8, -16(%rcx)
    movnti %r8, -8(%rcx)
    jnz    .Lloop64nti
    mfence
    jmp    .Lless64
  end;
{$endif FPC_SYSTEM_HAS_FILLCHAR}

{$ifndef FPC_SYSTEM_HAS_INDEXBYTE}
{$define FPC_SYSTEM_HAS_INDEXBYTE}
function IndexByte(Const buf;len:SizeInt;b:byte):SizeInt; assembler; nostackframe;
{ win64: rcx buf, rdx len, r8b word
  linux: rdi buf, rsi len, rdx word }
asm
{$ifdef win64}
    movd   %r8d, %xmm1
{$else}
    movd   %edx, %xmm1
    movq   %rdi, %rcx
    movq   %rsi, %rdx
{$endif}
    mov    %rcx, %r8
    punpcklbw  %xmm1, %xmm1
    and    $-0x10, %rcx                { highest aligned address before buf }
    test   %rdx, %rdx
    punpcklbw  %xmm1, %xmm1
    jz     .Lnotfound                  { exit if len=0 }
    add    $16, %rcx                   { first aligned address after buf }
    pshufd $0, %xmm1, %xmm1
    movdqa -16(%rcx), %xmm0            { Fetch first 16 bytes (up to 15 bytes before target) }
    sub    %r8, %rcx                   { rcx=number of valid bytes, r8=original ptr }

    pcmpeqb %xmm1, %xmm0               { compare with pattern and get bitmask }
    pmovmskb %xmm0, %eax

    shl    %cl, %eax                   { shift valid bits into high word }
    and    $0xffff0000, %eax           { clear low word containing invalid bits }
    shr    %cl, %eax                   { shift back }
    jmp   .Lcontinue

    .balign 16
.Lloop:
    movdqa (%r8,%rcx), %xmm0           { r8 and rcx may have any values, }
    add    $16, %rcx                   { but their sum is evenly divisible by 16. }
    pcmpeqb %xmm1, %xmm0
    pmovmskb %xmm0, %eax
.Lcontinue:
    test   %eax, %eax
    jnz    .Lmatch
    cmp    %rcx, %rdx
    ja     .Lloop
.Lnotfound:
    or     $-1, %rax
    retq

.Lmatch:
    bsf    %eax, %eax
    lea    -16(%rcx,%rax), %rax
    cmp    %rax, %rdx                  { check against the buffer length }
    jbe    .Lnotfound
end;
{$endif FPC_SYSTEM_HAS_INDEXBYTE}

{$ifndef FPC_SYSTEM_HAS_INDEXWORD}
{$define FPC_SYSTEM_HAS_INDEXWORD}
function IndexWord(Const buf;len:SizeInt;b:word):SizeInt; assembler; nostackframe;
{ win64: rcx buf, rdx len, r8b word
  linux: rdi buf, rsi len, rdx word }
asm
{$ifdef win64}
    movd   %r8d, %xmm1
{$else}
    movd   %edx, %xmm1
    movq   %rdi, %rcx
    movq   %rsi, %rdx
{$endif}
    mov    %rcx, %r8
    punpcklwd  %xmm1, %xmm1
    and    $-0x10, %rcx
    test   %rdx, %rdx
    pshufd $0, %xmm1, %xmm1
    jz     .Lnotfound                  { exit if len=0 }
    add    $16, %rcx
    movdqa -16(%rcx), %xmm0            { Fetch first 16 bytes (up to 14 bytes before target) }
    sub    %r8, %rcx                   { rcx=number of valid bytes }

    test   $1, %r8b                    { if buffer isn't aligned to word boundary, }
    jnz    .Lunaligned                 { use a different algorithm }

    pcmpeqw  %xmm1, %xmm0
    pmovmskb %xmm0, %eax

    shl    %cl, %eax
    and    $0xffff0000, %eax
    shr    %cl, %eax
    shr    $1, %ecx                    { bytes->words }
    jmp    .Lcontinue

    .balign 16
.Lloop:
    movdqa (%r8,%rcx,2), %xmm0
    add    $8, %rcx
    pcmpeqw  %xmm1, %xmm0
    pmovmskb %xmm0, %eax
.Lcontinue:
    test   %eax, %eax
    jnz    .Lmatch
    cmp    %rcx, %rdx
    ja     .Lloop

.Lnotfound:
    or    $-1, %rax
    retq

.Lmatch:
    bsf    %eax, %eax
    shr    $1, %eax                    { in words }
    lea    -8(%rcx,%rax), %rax
    cmp    %rax, %rdx
    jbe    .Lnotfound                  { if match is after the specified length, ignore it }
    retq

.Lunaligned:
    movdqa  %xmm1, %xmm2               { (mis)align the pattern (in this particular case: }
    psllw   $8, %xmm1                  {   swap bytes of each word of pattern) }
    psrlw   $8, %xmm2
    por     %xmm2, %xmm1

    pcmpeqb  %xmm1, %xmm0
    pmovmskb %xmm0, %eax

    shl    %cl, %eax
    and    $0xffff0000, %eax
    shr    %cl, %eax

    add    %rdx, %rdx                  { length words -> bytes }
    xor    %r10d, %r10d                { nothing to merge yet }
    jmp    .Lcontinue_u

    .balign 16
.Lloop_u:
    movdqa (%r8,%rcx), %xmm0
    add    $16, %rcx
    pcmpeqb %xmm1, %xmm0               { compare by bytes }
    shr    $16, %r10d                  { bit 16 shifts into 0 }
    pmovmskb %xmm0, %eax
.Lcontinue_u:
    shl    $1, %eax                    { 15:0 -> 16:1 }
    or     %r10d, %eax                 { merge bit 0 from previous round }
    mov    %eax, %r10d
    shr    $1, %eax                    { now AND together adjacent pairs of bits }
    and    %r10d, %eax
    and    $0x5555, %eax               { also reset odd bits }
    jnz    .Lmatch_u
    cmpq   %rcx, %rdx
    ja     .Lloop_u

.Lnotfound_u:
    or     $-1, %rax
    retq
.Lmatch_u:
    bsf    %eax, %eax
    lea    -16(%rcx,%rax), %rax
    cmp    %rax, %rdx
    jbe    .Lnotfound_u                { if match is after the specified length, ignore it }
    sar    $1, %rax                    { in words }
end;
{$endif FPC_SYSTEM_HAS_INDEXWORD}

{$endif freebsd}

{$ifndef FPC_SYSTEM_HAS_COMPAREBYTE}
{$define FPC_SYSTEM_HAS_COMPAREBYTE}
function CompareByte(Const buf1,buf2;len:SizeInt):SizeInt; assembler; nostackframe;
{ win64: rcx buf, rdx buf, r8 len
  linux: rdi buf, rsi buf, rdx len }
asm
{$ifndef win64}
    mov    %rdx, %r8
    mov    %rsi, %rdx
    mov    %rdi, %rcx
{$endif win64}
    negq    %r8
    jz      .LCmpbyteZero

    subq    %r8, %rcx
    subq    %r8, %rdx

    .balign 16
.LCmpbyteLoop:
{$ifdef oldbinutils}
// for the reason why this alternate coding of movzbl is given here
// see the comments in FillChar above
    .byte 0x42,0x0F,0xB6,0x04,0x01
{$else}
    movzbl  (%rcx,%r8), %eax
{$endif}    cmpb    (%rdx,%r8), %al
    jne     .LCmpbyteExitFast
    incq     %r8
    jne     .LCmpbyteLoop
.LCmpbyteZero:
     xorl    %eax, %eax
     retq

.LCmpbyteExitFast:
{$ifdef oldbinutils}
    .byte 0x42,0x0F,0xB6,0x0C,0x02
{$else}
     movzbl  (%rdx,%r8), %ecx    { Compare last position }
{$endif}
     subq    %rcx, %rax
end;
{$endif FPC_SYSTEM_HAS_COMPAREBYTE}


{$define FPC_SYSTEM_HAS_DECLOCKED_LONGINT}
{ does a thread save inc/dec }
function declocked(var l : longint) : boolean;assembler; nostackframe;
  asm
     { this check should be done because a lock takes a lot }
     { of time!                                             }
{$ifdef FPC_PIC}
     movq       IsMultithread@GOTPCREL(%rip),%rax
     cmpl       $0,(%rax)
{$else FPC_PIC}
     cmpl       $0,IsMultithread(%rip)
{$endif FPC_PIC}
{$ifndef win64}
     mov        %rdi, %rcx
{$endif win64}
     jz         .Ldeclockednolock
     lock
     decl       (%rcx)
     jmp        .Ldeclockedend
.Ldeclockednolock:
     decl       (%rcx)
.Ldeclockedend:
     setzb      %al
  end;


{$define FPC_SYSTEM_HAS_DECLOCKED_INT64}
function declocked(var l : int64) : boolean;assembler; nostackframe;
  asm
     { this check should be done because a lock takes a lot }
     { of time!                                             }
{$ifdef FPC_PIC}
     movq       IsMultithread@GOTPCREL(%rip),%rax
     cmpl       $0,(%rax)
{$else FPC_PIC}
     cmpl       $0,IsMultithread(%rip)
{$endif FPC_PIC}
{$ifndef win64}
     mov        %rdi, %rcx
{$endif win64}
     jz         .Ldeclockednolock
     lock
     decq       (%rcx)
     jmp        .Ldeclockedend
.Ldeclockednolock:
     decq       (%rcx)
.Ldeclockedend:
     setzb      %al
  end;


{$define FPC_SYSTEM_HAS_INCLOCKED_LONGINT}
procedure inclocked(var l : longint);assembler; nostackframe;

  asm
     { this check should be done because a lock takes a lot }
     { of time!                                             }
{$ifdef FPC_PIC}
     movq       IsMultithread@GOTPCREL(%rip),%rax
     cmpl       $0,(%rax)
{$else FPC_PIC}
     cmpl       $0,IsMultithread(%rip)
{$endif FPC_PIC}
{$ifndef win64}
     mov        %rdi, %rcx
{$endif win64}
     jz         .Linclockednolock
     lock
     incl       (%rcx)
     jmp        .Linclockedend
.Linclockednolock:
     incl       (%rcx)
.Linclockedend:
  end;


{$define FPC_SYSTEM_HAS_INCLOCKED_INT64}
procedure inclocked(var l : int64);assembler; nostackframe;

  asm
     { this check should be done because a lock takes a lot }
     { of time!                                             }
{$ifdef FPC_PIC}
     movq       IsMultithread@GOTPCREL(%rip),%rax
     cmpl       $0,(%rax)
{$else FPC_PIC}
     cmpl       $0,IsMultithread(%rip)
{$endif FPC_PIC}
{$ifndef win64}
     mov        %rdi, %rcx
{$endif win64}
     jz         .Linclockednolock
     lock
     incq       (%rcx)
     jmp        .Linclockedend
.Linclockednolock:
     incq       (%rcx)
.Linclockedend:
  end;


function InterLockedDecrement (var Target: longint) : longint; assembler; nostackframe;
asm
{$ifdef win64}
        movq    %rcx,%rax
{$else win64}
        movq    %rdi,%rax
{$endif win64}
        movl    $-1,%edx
        xchgq   %rdx,%rax
        lock
        xaddl   %eax, (%rdx)
        decl    %eax
end;


function InterLockedIncrement (var Target: longint) : longint; assembler; nostackframe;
asm
{$ifdef win64}
        movq    %rcx,%rax
{$else win64}
        movq    %rdi,%rax
{$endif win64}
        movl    $1,%edx
        xchgq   %rdx,%rax
        lock
        xaddl   %eax, (%rdx)
        incl    %eax
end;


function InterLockedExchange (var Target: longint;Source : longint) : longint; assembler; nostackframe;
asm
{$ifdef win64}
        xchgl   (%rcx),%edx
        movl    %edx,%eax
{$else win64}
        xchgl   (%rdi),%esi
        movl    %esi,%eax
{$endif win64}
end;


function InterLockedExchangeAdd (var Target: longint;Source : longint) : longint; assembler; nostackframe;
asm
{$ifdef win64}
        xchgq   %rcx,%rdx
        lock
        xaddl   %ecx, (%rdx)
        movl    %ecx,%eax
{$else win64}
        xchgq   %rdi,%rsi
        lock
        xaddl   %edi, (%rsi)
        movl    %edi,%eax
{$endif win64}
end;


function InterLockedCompareExchange(var Target: longint; NewValue, Comperand : longint): longint; assembler; nostackframe;
asm
{$ifdef win64}
        movl            %r8d,%eax
        lock
        cmpxchgl        %edx,(%rcx)
{$else win64}
        movl            %edx,%eax
        lock
        cmpxchgl        %esi,(%rdi)
{$endif win64}
end;


function InterLockedDecrement64 (var Target: int64) : int64; assembler; nostackframe;
asm
{$ifdef win64}
        movq    %rcx,%rax
{$else win64}
        movq    %rdi,%rax
{$endif win64}
        movq    $-1,%rdx
        xchgq   %rdx,%rax
        lock
        xaddq   %rax, (%rdx)
        decq    %rax
end;


function InterLockedIncrement64 (var Target: int64) : int64; assembler; nostackframe;
asm
{$ifdef win64}
        movq    %rcx,%rax
{$else win64}
        movq    %rdi,%rax
{$endif win64}
        movq    $1,%rdx
        xchgq   %rdx,%rax
        lock
        xaddq   %rax, (%rdx)
        incq    %rax
end;


function InterLockedExchange64 (var Target: int64;Source : int64) : int64; assembler; nostackframe;
asm
{$ifdef win64}
        xchgq   (%rcx),%rdx
        movq    %rdx,%rax
{$else win64}
        xchgq   (%rdi),%rsi
        movq    %rsi,%rax
{$endif win64}
end;


function InterLockedExchangeAdd64 (var Target: int64;Source : int64) : int64; assembler; nostackframe;
asm
{$ifdef win64}
        xchgq   %rcx,%rdx
        lock
        xaddq   %rcx, (%rdx)
        movq    %rcx,%rax
{$else win64}
        xchgq   %rdi,%rsi
        lock
        xaddq   %rdi, (%rsi)
        movq    %rdi,%rax
{$endif win64}
end;


function InterLockedCompareExchange64(var Target: int64; NewValue, Comperand : int64): int64; assembler; nostackframe;
asm
{$ifdef win64}
        movq            %r8,%rax
        lock
        cmpxchgq        %rdx,(%rcx)
{$else win64}
        movq            %rdx,%rax
        lock
        cmpxchgq        %rsi,(%rdi)
{$endif win64}
end;


{****************************************************************************
                                  FPU
****************************************************************************}

const
  { Internal constants for use in system unit }
  FPU_Invalid = 1;
  FPU_Denormal = 2;
  FPU_DivisionByZero = 4;
  FPU_Overflow = 8;
  FPU_Underflow = $10;
  FPU_StackUnderflow = $20;
  FPU_StackOverflow = $40;
  FPU_ExceptionMask = $ff;

  MM_Invalid = 1;
  MM_Denormal = 2;
  MM_DivisionByZero = 4;
  MM_Overflow = 8;
  MM_Underflow = $10;
  MM_Precicion = $20;
  MM_ExceptionMask = $3f;

  MM_MaskInvalidOp = %0000000010000000;
  MM_MaskDenorm    = %0000000100000000;
  MM_MaskDivZero   = %0000001000000000;
  MM_MaskOverflow  = %0000010000000000;
  MM_MaskUnderflow = %0000100000000000;
  MM_MaskPrecision = %0001000000000000;

procedure fpc_cpuinit;
  begin
    { don't let libraries influence the FPU cw set by the host program }
    if IsLibrary then
      begin
        Default8087CW:=Get8087CW;
        DefaultMXCSR:=GetMXCSR;
      end;
    SysResetFPU;
  end;

{$define FPC_SYSTEM_HAS_SYSINITFPU}
Procedure SysInitFPU;
begin
end;


{$define FPC_SYSTEM_HAS_SYSRESETFPU}
Procedure SysResetFPU;
  var
    { these locals are so we don't have to hack pic code in the assembler }
    localmxcsr: dword;
    localfpucw: word;
  begin
    localfpucw:=Default8087CW;
    localmxcsr:=DefaultMXCSR;
    asm
      fninit
      fwait
      fldcw   localfpucw
      ldmxcsr localmxcsr
    end;
  end;


{$ifndef FPC_SYSTEM_HAS_MEM_BARRIER}
{$define FPC_SYSTEM_HAS_MEM_BARRIER}

procedure ReadBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
  lfence
end;

procedure ReadDependencyBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
  { reads imply barrier on earlier reads depended on }
end;

procedure ReadWriteBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
  mfence
end;

procedure WriteBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
  sfence
end;

{$endif}

{****************************************************************************
                          Math Routines
****************************************************************************}

{$define FPC_SYSTEM_HAS_SWAPENDIAN}

{ SwapEndian(<16 Bit>) being inlined is faster than using assembler }
function SwapEndian(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    { the extra Word type cast is necessary because the "AValue shr 8" }
    { is turned into "longint(AValue) shr 8", so if AValue < 0 then    }
    { the sign bits from the upper 16 bits are shifted in rather than  }
    { zeroes.                                                          }
    Result := SmallInt((Word(AValue) shr 8) or (Word(AValue) shl 8));
  end;


function SwapEndian(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result := Word((AValue shr 8) or (AValue shl 8));
  end;


function SwapEndian(const AValue: LongInt): LongInt; assembler; nostackframe;
asm
{$ifdef win64}
  movl %ecx, %eax
{$else win64}
  movl %edi, %eax
{$endif win64}
  bswap %eax
end;


function SwapEndian(const AValue: DWord): DWord; assembler; nostackframe;
asm
{$ifdef win64}
  movl %ecx, %eax
{$else win64}
  movl %edi, %eax
{$endif win64}
  bswap %eax
end;


function SwapEndian(const AValue: Int64): Int64; assembler; nostackframe;
asm
{$ifdef win64}
  movq %rcx, %rax
{$else win64}
  movq %rdi, %rax
{$endif win64}
  bswap %rax
end;


function SwapEndian(const AValue: QWord): QWord; assembler; nostackframe;
asm
{$ifdef win64}
  movq %rcx, %rax
{$else win64}
  movq %rdi, %rax
{$endif win64}
  bswap %rax
end;


{$ifndef win64}
{$define FPC_SYSTEM_HAS_U128_DIV_U64_TO_U64}
function u128_div_u64_to_u64( const xh, xl: qword; const y: qword; out quotient, remainder: qword ): boolean;nostackframe;assembler;
{
  SysV:
  xh: RDI
  xl: RSI
  y: RDX
  quotient: RCX
  remainder: R8
}
label
  dodiv;
asm
  cmpq %rdi,%rdx
  ja   dodiv
  xorl %eax,%eax
  ret
dodiv:
  movq %rdx,%r9
  movq %rsi,%rax
  movq %rdi,%rdx
  divq %r9
  movq %rax,(%rcx)
  movq %rdx,(%r8)
  movl $1,%eax
end;
{$endif win64}
