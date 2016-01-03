{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by the Free Pascal development team.

    Processor dependent implementation for the system unit for
    ARM

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$asmmode divided}
=======
{$asmmode gas}
>>>>>>> graemeg/cpstrnew
=======
{$asmmode gas}
>>>>>>> graemeg/cpstrnew
=======
{$asmmode gas}
>>>>>>> graemeg/cpstrnew
=======
{$asmmode gas}
>>>>>>> origin/cpstrnew

{$ifndef FPC_SYSTEM_HAS_MOVE}
{$define FPC_SYSTEM_FPC_MOVE}
{$endif FPC_SYSTEM_HAS_MOVE}

{$ifdef FPC_SYSTEM_FPC_MOVE}
const
  cpu_has_edsp : boolean = false;
  in_edsp_test : boolean = false;
{$endif FPC_SYSTEM_FPC_MOVE}

{$if not(defined(wince)) and not(defined(gba)) and not(defined(nds)) and not(defined(FPUSOFT)) and not(defined(FPULIBGCC))}
{$define FPC_SYSTEM_HAS_SYSINITFPU}
Procedure SysInitFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  { Enable FPU exceptions, but disable INEXACT, UNDERFLOW, DENORMAL }
  asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(FPUFPA) or defined(FPUFPA10) or defined(FPUFPA11)}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    rfs r0
    and r0,r0,#0xffe0ffff
    orr r0,r0,#0x00070000
    wfs r0
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$else}
    movw r0, #(0xed88)
    movt r0, #(0xe000)
    ldr r1, [r0]
    orr r1, r1, #(0xF << 20)
    str r1, [r0]
{$endif}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;
end;
{$endif}

procedure fpc_cpuinit;
begin
  SysInitFPU;
end;

{$ifdef wince}
function _controlfp(new: DWORD; mask: DWORD): DWORD; cdecl; external 'coredll';

{$define FPC_SYSTEM_HAS_SYSRESETFPU}
Procedure SysResetFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  softfloat_exception_flags:=[];
=======
  softfloat_exception_flags:=0;
>>>>>>> graemeg/cpstrnew
=======
  softfloat_exception_flags:=0;
>>>>>>> graemeg/cpstrnew
=======
  softfloat_exception_flags:=0;
>>>>>>> graemeg/cpstrnew
=======
  softfloat_exception_flags:=0;
>>>>>>> origin/cpstrnew
end;

{$define FPC_SYSTEM_HAS_SYSINITFPU}
Procedure SysInitFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  softfloat_exception_mask:=[float_flag_underflow,float_flag_inexact,float_flag_denormal];
=======
  softfloat_exception_mask:=float_flag_underflow or float_flag_inexact or float_flag_denormal;
>>>>>>> graemeg/cpstrnew
=======
  softfloat_exception_mask:=float_flag_underflow or float_flag_inexact or float_flag_denormal;
>>>>>>> graemeg/cpstrnew
=======
  softfloat_exception_mask:=float_flag_underflow or float_flag_inexact or float_flag_denormal;
>>>>>>> graemeg/cpstrnew
=======
  softfloat_exception_mask:=float_flag_underflow or float_flag_inexact or float_flag_denormal;
>>>>>>> origin/cpstrnew
  { Enable FPU exceptions, but disable INEXACT, UNDERFLOW, DENORMAL }
  { FPU precision 64 bit, rounding to nearest, affine infinity }
  _controlfp($000C0003, $030F031F);
end;
{$endif wince}

{****************************************************************************
                       stack frame related stuff
****************************************************************************}

{$IFNDEF INTERNAL_BACKTRACE}
{$define FPC_SYSTEM_HAS_GET_FRAME}
function get_frame:pointer;assembler;nostackframe;
asm
  mov    r0,r11
end;
{$ENDIF not INTERNAL_BACKTRACE}

{
 Stack frame on Thumb2:
  LR    <- FP
  Old FP
}

{$define FPC_SYSTEM_HAS_GET_CALLER_ADDR}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function get_caller_addr(framebp:pointer;addr:pointer=nil):pointer;assembler;
=======
function get_caller_addr(framebp:pointer):pointer;assembler;
>>>>>>> graemeg/cpstrnew
=======
function get_caller_addr(framebp:pointer):pointer;assembler;
>>>>>>> graemeg/cpstrnew
=======
function get_caller_addr(framebp:pointer):pointer;assembler;
>>>>>>> graemeg/cpstrnew
=======
function get_caller_addr(framebp:pointer):pointer;assembler;
>>>>>>> origin/cpstrnew
asm
  movs r0,r0
  beq .Lg_a_null
  ldr r0,[r0]
.Lg_a_null:
end;


{$define FPC_SYSTEM_HAS_GET_CALLER_FRAME}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function get_caller_frame(framebp:pointer;addr:pointer=nil):pointer;assembler;
=======
function get_caller_frame(framebp:pointer):pointer;assembler;
>>>>>>> graemeg/cpstrnew
=======
function get_caller_frame(framebp:pointer):pointer;assembler;
>>>>>>> graemeg/cpstrnew
=======
function get_caller_frame(framebp:pointer):pointer;assembler;
>>>>>>> graemeg/cpstrnew
=======
function get_caller_frame(framebp:pointer):pointer;assembler;
>>>>>>> origin/cpstrnew
asm
  movs r0,r0
  beq .Lgnf_null
  ldr r0,[r0,#-4]
.Lgnf_null:
end;


{$define FPC_SYSTEM_HAS_SPTR}
Function Sptr : pointer;assembler;
asm
  mov    r0,sp
end;


{$ifndef FPC_SYSTEM_HAS_FILLCHAR}
{$define FPC_SYSTEM_HAS_FILLCHAR}
Procedure FillChar(var x;count:longint;value:byte);assembler;nostackframe;
asm
        // less than 0?
        cmp r1,#0
        it lt
        movlt pc,lr
        mov     r3,r0
        cmp     r1,#8           // at least 8 bytes to do?
        blt     .LFillchar2
        orr r2,r2,r2,lsl #8
        orr r2,r2,r2,lsl #16
.LFillchar0:
        tst     r3,#3           // aligned yet?
        itt ne
        strneb r2,[r3],#1
        subne   r1,r1,#1
        bne     .LFillchar0
        mov     ip,r2
.LFillchar1:
        cmp     r1,#8           // 8 bytes still to do?
        blt     .LFillchar2
        stmia   r3!,{r2,ip}
        sub     r1,r1,#8
        cmp     r1,#8           // 8 bytes still to do?
        blt     .LFillchar2
        stmia   r3!,{r2,ip}
        sub     r1,r1,#8
        cmp     r1,#8           // 8 bytes still to do?
        blt     .LFillchar2
        stmia   r3!,{r2,ip}
        sub     r1,r1,#8
        cmp     r1,#8           // 8 bytes still to do?
        itt ge
        stmgeia r3!,{r2,ip}
        subge   r1,r1,#8
        bge     .LFillchar1
.LFillchar2:
        movs r1,r1              // anything left?
        it eq
        moveq pc,lr
        rsb     r1,r1,#7
        mov     r1,r1,lsl #2
        add     pc,r1
        mov     r0,r0
        strb r2,[r3],#1
        strb r2,[r3],#1
        strb r2,[r3],#1
        strb r2,[r3],#1
        strb r2,[r3],#1
        strb r2,[r3],#1
        strb r2,[r3],#1
        mov pc,lr
end;
{$endif FPC_SYSTEM_HAS_FILLCHAR}

{$ifndef FPC_SYSTEM_HAS_MOVE}
{$define FPC_SYSTEM_HAS_MOVE}
procedure Move_pld(const source;var dest;count:longint);assembler;nostackframe;
asm
  pld [r0]
  pld [r1]
  // count <=0 ?
  cmp r2,#0
  it le
  movle pc,lr
  // overlap?
  cmp r1,r0
  bls .Lnooverlap
  add r3,r0,r2
  cmp r3,r1
  bls .Lnooverlap
  // overlap, copy backward
.Loverlapped:
  subs r2,r2,#1
  ldrb r3,[r0,r2]
  strb r3,[r1,r2]
  bne .Loverlapped
  mov pc,lr
.Lnooverlap:
  // less then 16 bytes to copy?
  cmp r2,#8
  // yes, the forget about the whole optimizations
  // and do a bytewise copy
  blt .Lbyteloop

  // both aligned?
  orr r3,r0,r1
  tst r3,#3

  bne .Lbyteloop
(*
  // yes, then align
  // alignment to 4 byte boundries is enough
  ldrb ip,[r0],#1
  sub r2,r2,#1
  stb ip,[r1],#1
  tst r3,#2
  bne .Ldifferentaligned
  ldrh ip,[r0],#2
  sub r2,r2,#2
  sth ip,[r1],#2

.Ldifferentaligned
  // qword aligned?
  orrs r3,r0,r1
  tst r3,#7
  bne .Ldwordloop
*)
  pld [r0,#32]
  pld [r1,#32]
.Ldwordloop:
  sub r2,r2,#4
  ldr r3,[r0],#4
  // preload
  pld [r0,#64]
  pld [r1,#64]
  cmp r2,#4
  str r3,[r1],#4
  bcs .Ldwordloop
  cmp r2,#0
  it eq
  moveq pc,lr
.Lbyteloop:
  subs r2,r2,#1
  ldrb r3,[r0],#1
  strb r3,[r1],#1
  bne .Lbyteloop
  mov pc,lr
end;

procedure Move_blended(const source;var dest;count:longint);assembler;nostackframe;
asm
  // count <=0 ?
  cmp r2,#0
  it le
  movle pc,lr
  // overlap?
  cmp r1,r0
  bls .Lnooverlap
  add r3,r0,r2
  cmp r3,r1
  bls .Lnooverlap
  // overlap, copy backward
.Loverlapped:
  subs r2,r2,#1
  ldrb r3,[r0,r2]
  strb r3,[r1,r2]
  bne .Loverlapped
  mov pc,lr
.Lnooverlap:
  // less then 16 bytes to copy?
  cmp r2,#8
  // yes, the forget about the whole optimizations
  // and do a bytewise copy
  blt .Lbyteloop

  // both aligned?
  orr r3,r0,r1
  tst r3,#3

  bne .Lbyteloop
(*
  // yes, then align
  // alignment to 4 byte boundries is enough
  ldrb ip,[r0],#1
  sub r2,r2,#1
  stb ip,[r1],#1
  tst r3,#2
  bne .Ldifferentaligned
  ldrh ip,[r0],#2
  sub r2,r2,#2
  sth ip,[r1],#2

.Ldifferentaligned
  // qword aligned?
  orrs r3,r0,r1
  tst r3,#7
  bne .Ldwordloop
*)
.Ldwordloop:
  sub r2,r2,#4
  ldr r3,[r0],#4
  cmp r2,#4
  str r3,[r1],#4
  bcs .Ldwordloop
  cmp r2,#0
  it eq
  moveq pc,lr
.Lbyteloop:
  subs r2,r2,#1
  ldrb r3,[r0],#1
  strb r3,[r1],#1
  bne .Lbyteloop
  mov pc,lr
end;


const
  moveproc : pointer = @move_blended;

procedure Move(const source;var dest;count:longint);[public, alias: 'FPC_MOVE'];assembler;nostackframe;
asm
  ldr ip,.Lmoveproc
  ldr pc,[ip]
.Lmoveproc:
  .long moveproc
end;

{$endif FPC_SYSTEM_HAS_MOVE}

{****************************************************************************
                                 String
****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_FPC_SHORTSTR_ASSIGN}
{$define FPC_SYSTEM_HAS_FPC_SHORTSTR_ASSIGN}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure fpc_shortstr_to_shortstr(out res:shortstring;const sstr:shortstring);assembler;nostackframe;[public,alias: 'FPC_SHORTSTR_TO_SHORTSTR'];compilerproc;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ifndef FPC_STRTOSHORTSTRINGPROC}
function fpc_shortstr_to_shortstr(len:longint;const sstr:shortstring):shortstring;assembler;nostackframe;[public,alias: 'FPC_SHORTSTR_TO_SHORTSTR'];compilerproc;
{$else}
procedure fpc_shortstr_to_shortstr(out res:shortstring;const sstr:shortstring);assembler;nostackframe;[public,alias: 'FPC_SHORTSTR_TO_SHORTSTR'];compilerproc;
{$endif}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{r0: __RESULT
 r1: len
 r2: sstr}

asm
    ldrb r12,[r2],#1
    cmp  r12,r1
    it gt
    movgt r12,r1
    strb r12,[r0],#1
    cmp  r12,#6 (* 6 seems to be the break even point. *)
    blt  .LStartTailCopy
    (* Align destination on 32bits. This is the only place where unrolling
       really seems to help, since in the common case, sstr is aligned on
       32 bits, therefore in the common case we need to copy 3 bytes to
       align, i.e. in the case of a loop, you wouldn't branch out early.*)
    rsb  r3,r0,#0
    ands  r3,r3,#3
    sub  r12,r12,r3
    itttt ne
    ldrneb r1,[r2],#1
    strneb r1,[r0],#1
    subnes  r3,r3,#1
    ldrneb r1,[r2],#1
    itttt ne
    strneb r1,[r0],#1
    subnes  r3,r3,#1
    ldrneb r1,[r2],#1
    strneb r1,[r0],#1
    it ne
    subnes  r3,r3,#1
.LDoneAlign:
    (* Destination should be aligned now, but source might not be aligned,
       if this is the case, do a byte-per-byte copy. *)
    tst r2,#3
    bne .LStartTailCopy
    (* Start the main copy, 32 bit at a time. *)
    movs r3,r12,lsr #2
    and r12,r12,#3
    beq  .LStartTailCopy
.LNext4bytes:
    (* Unrolling this loop would save a little bit of time for long strings
       (>20 chars), but alas, it hurts for short strings and they are the
       common case.*)
    ittt ne
    ldrne r1,[r2],#4
    strne r1,[r0],#4
    subnes  r3,r3,#1
    bne .LNext4bytes
.LStartTailCopy:
    (* Do remaining bytes. *)
    cmp r12,#0
    beq .LDoneTail
.LNextChar3:
    ldrb r1,[r2],#1
    strb r1,[r0],#1
    subs  r12,r12,#1
    bne .LNextChar3
.LDoneTail:
end;

procedure fpc_shortstr_assign(len:longint;sstr,dstr:pointer);assembler;nostackframe;[public,alias:'FPC_SHORTSTR_ASSIGN'];compilerproc;

{r0: len
 r1: sstr
 r2: dstr}

asm
    ldrb r12,[r1],#1
    cmp  r12,r0
    it gt
    movgt r12,r0
    strb r12,[r2],#1
    cmp  r12,#6 (* 6 seems to be the break even point. *)
    blt  .LStartTailCopy
    (* Align destination on 32bits. This is the only place where unrolling
       really seems to help, since in the common case, sstr is aligned on
       32 bits, therefore in the common case we need to copy 3 bytes to
       align, i.e. in the case of a loop, you wouldn't branch out early.*)
    rsb  r3,r2,#0
    ands  r3,r3,#3
    sub  r12,r12,r3
    itttt ne
    ldrneb r0,[r1],#1
    strneb r0,[r2],#1
    subnes  r3,r3,#1
    ldrneb r0,[r1],#1
    itttt ne
    strneb r0,[r2],#1
    subnes  r3,r3,#1
    ldrneb r0,[r1],#1
    strneb r0,[r2],#1
    it ne
    subnes  r3,r3,#1
.LDoneAlign:
    (* Destination should be aligned now, but source might not be aligned,
       if this is the case, do a byte-per-byte copy. *)
    tst r1,#3
    bne .LStartTailCopy
    (* Start the main copy, 32 bit at a time. *)
    movs r3,r12,lsr #2
    and r12,r12,#3
    beq  .LStartTailCopy
.LNext4bytes:
    (* Unrolling this loop would save a little bit of time for long strings
       (>20 chars), but alas, it hurts for short strings and they are the
       common case.*)
    ittt ne
    ldrne r0,[r1],#4
    strne r0,[r2],#4
    subnes  r3,r3,#1
    bne .LNext4bytes
.LStartTailCopy:
    (* Do remaining bytes. *)
    cmp r12,#0
    beq .LDoneTail
.LNextChar3:
    ldrb r0,[r1],#1
    strb r0,[r2],#1
    subs  r12,r12,#1
    bne .LNextChar3
.LDoneTail:
end;
{$endif FPC_SYSTEM_HAS_FPC_SHORTSTR_ASSIGN}

{$ifndef FPC_SYSTEM_HAS_FPC_PCHAR_LENGTH}
{$define FPC_SYSTEM_HAS_FPC_PCHAR_LENGTH}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function fpc_Pchar_length(p:Pchar):sizeint;assembler;nostackframe;[public,alias:'FPC_PCHAR_LENGTH'];compilerproc;
=======
function fpc_Pchar_length(p:Pchar):longint;assembler;nostackframe;[public,alias:'FPC_PCHAR_LENGTH'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
function fpc_Pchar_length(p:Pchar):longint;assembler;nostackframe;[public,alias:'FPC_PCHAR_LENGTH'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
function fpc_Pchar_length(p:Pchar):longint;assembler;nostackframe;[public,alias:'FPC_PCHAR_LENGTH'];compilerproc;
>>>>>>> graemeg/cpstrnew
=======
function fpc_Pchar_length(p:Pchar):longint;assembler;nostackframe;[public,alias:'FPC_PCHAR_LENGTH'];compilerproc;
>>>>>>> origin/cpstrnew

asm
    cmp r0,#0
    mov r1,r0
    beq .Ldone
.Lnextchar:
    (*Are we aligned?*)
    tst r1,#3
    bne .Ltest_unaligned    (*No, do byte per byte.*)
    ldr r3,.L01010101
.Ltest_aligned:
    (*Aligned, load 4 bytes at a time.*)
    ldr r12,[r1],#4
    (*Check wether r12 contains a 0 byte.*)
    sub r2,r12,r3
    mvn r12,r12
    and r2,r2,r12
    ands r2,r2,r3,lsl #7    (*r3 lsl 7 = $80808080*)
    beq .Ltest_aligned      (*No 0 byte, repeat.*)
    sub r1,r1,#4
.Ltest_unaligned:
    ldrb r12,[r1],#1
    cmp r12,#1              (*r12<1 same as r12=0, but result in carry flag*)
    bcs .Lnextchar
    (*Dirty trick: we need to subtract 1 extra because we have counted the
      terminating 0, due to the known carry flag sbc can do this.*)
    sbc r0,r1,r0
.Ldone:
    mov pc,lr
.L01010101:
    .long 0x01010101
end;
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function InterLockedDecrement (var Target: longint) : longint; assembler; nostackframe;
asm
.Lloop:
  ldrex ip, [r0]
  sub ip, #1
  strex r3, ip, [r0]
  cmp r3, #0
  bne .Lloop

  mov r0, ip
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

var
  fpc_system_lock: longint; export name 'fpc_system_lock';

function InterLockedDecrement (var Target: longint) : longint; assembler; nostackframe;
asm
// lock
  ldr r3, .Lfpc_system_lock
  mov r1, #1
.Lloop:
  ldrex r2, [r3]
  cmp r2, #0
  itt eq
  strexeq r2, r1, [r3]
  cmpeq r2, #0
  bne .Lloop
// do the job
  ldr r1, [r0]
  sub r1, r1, #1
  str r1, [r0]
  mov r0, r1
// unlock and return
  str r2, [r3]
  mov pc, lr
  
.Lfpc_system_lock:
  .long fpc_system_lock
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;


function InterLockedIncrement (var Target: longint) : longint; assembler; nostackframe;
asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
.Lloop:
  ldrex ip, [r0]
  add ip, #1
  strex r3, ip, [r0]
  cmp r3, #0
  bne .Lloop

  mov r0, ip
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
// lock
  ldr r3, .Lfpc_system_lock
  mov r1, #1
.Lloop:
  ldrex r2, [r3]
  cmp r2, #0
  itt eq
  strexeq r2, r1, [r3]
  cmpeq r2, #0
  bne .Lloop
// do the job
  ldr r1, [r0]
  add r1, r1, #1
  str r1, [r0]
  mov r0, r1
// unlock and return
  str r2, [r3]
  mov pc, lr

.Lfpc_system_lock:
  .long fpc_system_lock
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;


function InterLockedExchange (var Target: longint;Source : longint) : longint; assembler; nostackframe;
asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
.Lloop:
  ldrex ip, [r0]
  strex r3, r1, [r0]
  cmp r3, #0
  bne .Lloop

  mov r0, ip
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

// lock
  ldr r3, .Lfpc_system_lock
  mov r2, #1
.Lloop:
  ldrex r2, [r3]
  cmp r2, #0
  itt eq
  strexeq r2, r12, [r3]
  cmpeq r2, #0
  bne .Lloop
// do the job
  ldr r2, [r0]
  str r1, [r0]
  mov r0, r2
// unlock and return
  mov r2, #0
  str r2, [r3]
  mov pc, lr

.Lfpc_system_lock:
  .long fpc_system_lock
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

function InterLockedExchangeAdd (var Target: longint;Source : longint) : longint; assembler; nostackframe;
asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
.Lloop:
  ldrex ip, [r0]
  add r2, ip, r1
  strex r3, r2, [r0]
  cmp r3, #0
  bne .Lloop

  mov r0, ip
end;

function InterlockedCompareExchange(var Target: longint; NewValue: longint; Comperand: longint): longint; assembler; nostackframe;
asm
.Lloop:
  ldrex ip, [r0]
  cmp ip, r2
  ite eq
  strexeq r3, r1, [r0]
  movne r3, #0
  cmp r3, #0
  bne .Lloop

  mov r0, ip
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
// lock
  ldr r3, .Lfpc_system_lock
  mov r2, #1
.Lloop:
  ldrex r2, [r3]
  cmp r2, #0
  itt eq
  strexeq r2, r12, [r3]
  cmpeq r2, #0
  bne .Lloop
// do the job
  ldr r2, [r0]
  add r1, r1, r2
  str r1, [r0]
  mov r0, r2
// unlock and return
  mov r2, #0
  str r2, [r3]
  mov pc, lr

.Lfpc_system_lock:
  .long fpc_system_lock
end;


function InterlockedCompareExchange(var Target: longint; NewValue: longint; Comperand: longint): longint; assembler; nostackframe;
asm
// lock
  ldr r12, .Lfpc_system_lock
  mov r3, #1
.Lloop:
  ldrex r2, [r12]
  cmp r2, #0
  itt eq
  strexeq r2, r1, [r12]
  cmpeq r2, #0
  bne .Lloop
// do the job
  ldr r3, [r0]
  cmp r3, r2
  it eq
  streq r1, [r0]
  mov r0, r3
// unlock and return
  mov r3, #0
  str r3, [r12]
  mov pc, lr

.Lfpc_system_lock:
  .long fpc_system_lock
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

{$define FPC_SYSTEM_HAS_DECLOCKED_LONGINT}
function declocked(var l: longint) : boolean; inline;
begin
  Result:=InterLockedDecrement(l) = 0;
end;

{$define FPC_SYSTEM_HAS_INCLOCKED_LONGINT}
procedure inclocked(var l: longint); inline;
begin
  InterLockedIncrement(l);
end;

procedure fpc_cpucodeinit;
begin
{$ifdef FPC_SYSTEM_FPC_MOVE}
  cpu_has_edsp:=true;
  in_edsp_test:=true;
  asm
    mov r1,sp
    bic r0,r1,#7
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    ldrd r0,r1,[r0]
=======
    ldrd r0,[r0]
>>>>>>> graemeg/cpstrnew
=======
    ldrd r0,[r0]
>>>>>>> graemeg/cpstrnew
=======
    ldrd r0,[r0]
>>>>>>> graemeg/cpstrnew
=======
    ldrd r0,[r0]
>>>>>>> origin/cpstrnew
  end;
  in_edsp_test:=false;
  if cpu_has_edsp then
    moveproc:=@move_pld
  else
    moveproc:=@move_blended;
{$endif FPC_SYSTEM_FPC_MOVE}
end;
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

{include hand-optimized assembler division code}
{$i divide.inc}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
