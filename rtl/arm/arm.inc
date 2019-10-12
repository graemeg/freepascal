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

{ IMPORTANT!
  Never use the "BLX label" instruction! Use "BL label" instead.
  The linker will always change BL to BLX if necessary, but not vice versa (linker version dependent).
  "BLX label" ALWAYS changes the instruction set. It changes a processor in ARM state to Thumb state,
  or a processor in Thumb state to ARM state.
}

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

{ for bootstrapping with 3.0.x/3.2.x }
{$if not defined(darwin) and not defined(FPUVFPV2) and not defined(FPUVFPV3) and not defined(FPUVFPV4) and not defined(FPUVFPV3_D16) and not defined(FPUARM_HAS_VFP_EXTENSION))}
{$define FPUARM_HAS_FPA}
{$else}
{$define FPUARM_HAS_VFP_EXTENSION}
{$endif}

{$if defined(FPUARM_HAS_FPA)}
Procedure SysInitFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  { Enable FPU exceptions, but disable INEXACT, UNDERFLOW, DENORMAL }
  asm
    rfs r0
    and r0,r0,#0xffe0ffff
    orr r0,r0,#0x00070000
    wfs r0
  end;
end;
{$elseif defined(FPUARM_HAS_VFP_EXTENSION)}


const
  fpu_nx = 1 shl 0;
  fpu_uf = 1 shl 1;
  fpu_of = 1 shl 2;
  fpu_dz = 1 shl 3;
  fpu_nv = 1 shl 4;

function getfpscr: sizeuint; nostackframe; assembler;
  asm
    fmrx r0,fpscr
  end;


procedure setfpscr(flags : sizeuint); nostackframe; assembler;
  asm
    fmxr fpscr,r0
  end;


const
  FPSCR_IOC = 1;
  FPSCR_DZC = 1 shl 1;
  FPSCR_OFC = 1 shl 2;
  FPSCR_UFC = 1 shl 3;
  FPSCR_IXC = 1 shl 4;
  FPSCR_IDC = 1 shl 7;
  FPSCR_EXCEPTIONS = FPSCR_IOC or FPSCR_DZC or FPSCR_OFC or FPSCR_UFC or FPSCR_IXC or FPSCR_IDC;

procedure RaisePendingExceptions;
  var
    fpscr : longint;
    f: TFPUException;
  begin
    { at this point, we know already, that an exception will be risen }
    fpscr:=getfpscr;

    if (fpscr and FPSCR_DZC) <> 0 then
      float_raise(exZeroDivide);
    if (fpscr and FPSCR_OFC) <> 0 then
      float_raise(exOverflow);
    if (fpscr and FPSCR_UFC) <> 0 then
      float_raise(exUnderflow);
    if (fpscr and FPSCR_IOC) <> 0 then
      float_raise(exInvalidOp);
    if (fpscr and FPSCR_IXC) <> 0 then
      float_raise(exPrecision);
    if (fpscr and FPSCR_IDC) <> 0 then
      float_raise(exDenormalized);

    { now the soft float exceptions }
    for f in softfloat_exception_flags do
      float_raise(f);
  end;


procedure fpc_throwfpuexception;[public,alias:'FPC_THROWFPUEXCEPTION'];
  var
    fpscr : dword;
    f: TFPUException;
  begin
    { at this point, we know already, that an exception will be risen }
    fpscr:=getfpscr;

    { check, if the exception is masked, as ARM without hardware exceptions have no masking functionality,
      we use the software mask }
    if ((fpscr and FPSCR_DZC) <> 0) and (exZeroDivide in softfloat_exception_mask) then
      fpscr:=fpscr and not(FPSCR_DZC);
    if ((fpscr and FPSCR_OFC) <> 0) and (exOverflow in softfloat_exception_mask) then
      fpscr:=fpscr and not(FPSCR_OFC);
    if ((fpscr and FPSCR_UFC) <> 0) and (exUnderflow in softfloat_exception_mask) then
      fpscr:=fpscr and not(FPSCR_UFC);
    if ((fpscr and FPSCR_IOC) <> 0) and (exInvalidOp in softfloat_exception_mask) then
      fpscr:=fpscr and not(FPSCR_IOC);
    if ((fpscr and FPSCR_IXC) <> 0) and (exPrecision in softfloat_exception_mask) then
      fpscr:=fpscr and not(FPSCR_IXC);
    if ((fpscr and FPSCR_IDC) <> 0) and (exDenormalized in softfloat_exception_mask) then
      fpscr:=fpscr and not(FPSCR_IDC);
    setfpscr(fpscr);
    if (fpscr and FPSCR_EXCEPTIONS)<>0 then
      RaisePendingExceptions;
  end;


Procedure SysInitFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  { Enable FPU exceptions, but disable INEXACT, UNDERFLOW, DENORMAL }
  asm
    fmrx r0,fpscr
    // set "round to nearest" mode
    and  r0,r0,#0xff3fffff
    // mask "exception happened" and overflow flags
    and  r0,r0,#0xffffff20
    // mask exception flags
    and  r0,r0,#0xffff40ff
{$ifndef darwin}
    // Floating point exceptions cause kernel panics on iPhoneOS 2.2.1...

    // disable flush-to-zero mode (IEEE math compliant)
    and  r0,r0,#0xfeffffff
    // enable invalid operation, div-by-zero and overflow exceptions
    orr  r0,r0,#0x00000700
{$endif}
    fmxr fpscr,r0
  end;
  softfloat_exception_mask:=[float_flag_underflow,float_flag_inexact,float_flag_denormal];
  softfloat_exception_flags:=[];
end;

{$define FPC_SYSTEM_HAS_SYSRESETFPU}
Procedure SysResetFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  softfloat_exception_flags:=[];
  setfpscr(getfpscr and not(FPSCR_EXCEPTIONS));
end;


{$endif}
{$endif}

procedure fpc_cpuinit;
begin
  { don't let libraries influence the FPU cw set by the host program }
  if not IsLibrary then
    SysInitFPU;
end;

{$ifdef wince}
function _controlfp(new: DWORD; mask: DWORD): DWORD; cdecl; external 'coredll';

{$define FPC_SYSTEM_HAS_SYSRESETFPU}
Procedure SysResetFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
end;

{$define FPC_SYSTEM_HAS_SYSINITFPU}
Procedure SysInitFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  { Enable FPU exceptions, but disable INEXACT, UNDERFLOW, DENORMAL }
  { FPU precision 64 bit, rounding to nearest, affine infinity }
  _controlfp($000C0003, $030F031F);
end;
{$endif wince}

{$ifdef linux}
function fpc_read_tp : pointer; [public, alias: 'fpc_read_tp'];assembler; nostackframe;
asm
  // Helper is located at 0xffff0fe0
  mvn r0,#0x0000f000 // mov r0, #0xffff0fff
  sub pc,r0,#0x1f    // Jump to helper
end;
{$endif linux}

{****************************************************************************
                       stack frame related stuff
****************************************************************************}

{$IFNDEF INTERNAL_BACKTRACE}
{$define FPC_SYSTEM_HAS_GET_FRAME}
function get_frame:pointer;assembler;nostackframe;
asm
{$ifndef darwin}
  mov    r0,r11
{$else}
  mov    r0,r7
{$endif}
end;
{$ENDIF not INTERNAL_BACKTRACE}

{$define FPC_SYSTEM_HAS_GET_CALLER_ADDR}
function get_caller_addr(framebp:pointer;addr:pointer=nil):pointer;assembler;nostackframe;
asm
  cmp r0,#0
{$ifndef darwin}
  ldrne r0,[r0,#-4]
{$else}
  ldrne r0,[r0,#4]
{$endif}
end;


{$define FPC_SYSTEM_HAS_GET_CALLER_FRAME}
function get_caller_frame(framebp:pointer;addr:pointer=nil):pointer;assembler;nostackframe;
asm
  cmp r0,#0
{$ifndef darwin}
  ldrne r0,[r0,#-12]
{$else}
  ldrne r0,[r0]
{$endif}
end;


{$define FPC_SYSTEM_HAS_SPTR}
Function Sptr : pointer;assembler;nostackframe;
asm
  mov    r0,sp
end;


{$ifndef FPC_SYSTEM_HAS_FILLCHAR}
{$define FPC_SYSTEM_HAS_FILLCHAR}
Procedure FillChar(var x;count:longint;value:byte);assembler;nostackframe;
asm
        // less than 0?
        cmp     r1,#0
{$ifdef CPUARM_HAS_BX}
        bxle    lr
{$else}
        movle   pc,lr
{$endif}
        mov     r3,r0

        orr     r2,r2,r2,lsl #8
        orr     r2,r2,r2,lsl #16

        tst     r3, #3  // Aligned?
        bne     .LFillchar_do_align

.LFillchar_is_aligned:
        subs    r1,r1,#8
        bmi     .LFillchar_less_than_8bytes

        mov     ip,r2
.LFillchar_at_least_8bytes:
        // Do 16 bytes per loop
        // More unrolling is uncessary, as we'll just stall on the write buffers
        stmia   r3!,{r2,ip}
        subs    r1,r1,#8
        stmplia r3!,{r2,ip}
        subpls  r1,r1,#8
        bpl     .LFillchar_at_least_8bytes

.LFillchar_less_than_8bytes:
        // Do the rest
        adds    r1, r1, #8

{$ifdef CPUARM_HAS_BX}
        bxeq    lr
{$else}
        moveq   pc,lr
{$endif}

        tst     r1, #4
        strne   r2,[r3],#4
{$ifdef CPUARM_HAS_ALL_MEM}
        tst     r1, #2
        strneh  r2,[r3],#2
{$else CPUARM_HAS_ALL_MEM}
        tst     r1, #2
        strneb  r2,[r3],#1
        strneb  r2,[r3],#1
{$endif CPUARM_HAS_ALL_MEM}
        tst     r1, #1
        strneb  r2,[r3],#1
{$ifdef CPUARM_HAS_BX}
        bx  lr
{$else}
        mov pc,lr
{$endif}

// Special case for unaligned start
// We make a maximum of 3 loops here
.LFillchar_do_align:
        strb r2,[r3],#1
        subs r1, r1, #1
{$ifdef CPUARM_HAS_BX}
        bxeq  lr
{$else}
        moveq pc,lr
{$endif}
        tst r3,#3
        bne .LFillchar_do_align
        b .LFillchar_is_aligned
end;
{$endif FPC_SYSTEM_HAS_FILLCHAR}

{$ifndef FPC_SYSTEM_HAS_MOVE}
{$define FPC_SYSTEM_HAS_MOVE}
{$ifdef CPUARM_HAS_EDSP}
procedure Move(const source;var dest;count:longint);[public, alias: 'FPC_MOVE'];assembler;nostackframe;
{$else CPUARM_HAS_EDSP}
procedure Move_pld(const source;var dest;count:longint);assembler;nostackframe;
{$endif CPUARM_HAS_EDSP}
asm
  // pld [r0]
  // encode this using .long so the rtl assembles also with instructions sets not supporting pld
  .long 0xf5d0f000

  // count <=0 ?
  cmp r2,#0
{$ifdef CPUARM_HAS_BX}
  bxle  lr
{$else}
  movle pc,lr
{$endif}
  // overlap?
  subs   r3, r1, r0    // if (dest > source) and
  cmphi  r2, r3        //    (count > dest - src) then
  bhi    .Loverlapped  //   DoReverseByteCopy;

  cmp r2,#8            // if (count < 8) then
  blt .Lbyteloop       //    DoForwardByteCopy;
  // Any way to avoid the above jump and fuse the next two instructions?
  tst   r0, #3         // if (source and 3) <> 0 or
  tsteq r1, #3         //    (dest and 3) <> 0 then
  bne   .Lbyteloop     //   DoForwardByteCopy;

  // pld   [r0,#32]
  // encode this using .long so the rtl assembles also with instructions sets not supporting pld
  .long 0xf5d0f020

.Ldwordloop:
  ldmia r0!, {r3, ip}

  // preload
  // pld   [r0,#64]
  // encode this using .long so the rtl assembles also with instructions sets not supporting pld
  .long 0xf5d0f040

  sub   r2,r2,#8
  cmp   r2, #8
  stmia r1!, {r3, ip}
  bge   .Ldwordloop
  cmp   r2,#0
{$ifdef CPUARM_HAS_BX}
  bxeq  lr
{$else}
  moveq pc,lr
{$endif}
.Lbyteloop:
  subs r2,r2,#1
  ldrb r3,[r0],#1
  strb r3,[r1],#1
  bne .Lbyteloop
{$ifdef CPUARM_HAS_BX}
  bx  lr
{$else}
  mov pc,lr
{$endif}
.Loverlapped:
  subs r2,r2,#1
  ldrb r3,[r0,r2]
  strb r3,[r1,r2]
  bne .Loverlapped
end;

{$ifndef CPUARM_HAS_EDSP}
procedure Move_blended(const source;var dest;count:longint);assembler;nostackframe;
asm
  // count <=0 ?
  cmp r2,#0
{$ifdef CPUARM_HAS_BX}
  bxle  lr
{$else}
  movle pc,lr
{$endif}
  // overlap?
  subs   r3, r1, r0    // if (dest > source) and
  cmphi  r2, r3        //    (count > dest - src) then
  bhi    .Loverlapped  //   DoReverseByteCopy;

  cmp r2,#8            // if (count < 8) then
  blt .Lbyteloop       //    DoForwardByteCopy;
  // Any way to avoid the above jump and fuse the next two instructions?
  tst   r0, #3         // if (source and 3) <> 0 or
  tsteq r1, #3         //    (dest and 3) <> 0 then
  bne   .Lbyteloop     //   DoForwardByteCopy;

.Ldwordloop:
  ldmia r0!, {r3, ip}
  sub   r2,r2,#8
  cmp   r2, #8
  stmia r1!, {r3, ip}
  bge   .Ldwordloop
  cmp   r2,#0
{$ifdef CPUARM_HAS_BX}
  bxeq  lr
{$else}
  moveq pc,lr
{$endif}
.Lbyteloop:
  subs r2,r2,#1
  ldrb r3,[r0],#1
  strb r3,[r1],#1
  bne .Lbyteloop
{$ifdef CPUARM_HAS_BX}
  bx  lr
{$else}
  mov pc,lr
{$endif}
.Loverlapped:
  subs r2,r2,#1
  ldrb r3,[r0,r2]
  strb r3,[r1,r2]
  bne .Loverlapped
end;

const
  moveproc : procedure(const source;var dest;count:longint) = @move_blended;

procedure Move(const source;var dest;count:longint);[public, alias: 'FPC_MOVE']; {$ifndef FPC_PIC} assembler;nostackframe; {$endif FPC_PIC}
{$ifdef FPC_PIC}
begin
  moveproc(source,dest,count);
end;
{$else FPC_PIC}
asm
  ldr ip,.Lmoveproc
  ldr pc,[ip]
.Lmoveproc:
  .long moveproc
end;
{$endif FPC_PIC}

{$endif CPUARM_HAS_EDSP}

{$endif FPC_SYSTEM_HAS_MOVE}

{****************************************************************************
                                 String
****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_FPC_SHORTSTR_ASSIGN}
{$define FPC_SYSTEM_HAS_FPC_SHORTSTR_ASSIGN}

procedure fpc_shortstr_to_shortstr(out res:shortstring;const sstr:shortstring);assembler;nostackframe;[public,alias: 'FPC_SHORTSTR_TO_SHORTSTR'];compilerproc;
{r0: __RESULT
 r1: len
 r2: sstr}

asm
    ldrb r12,[r2],#1
    cmp  r12,r1
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
    ldrneb r1,[r2],#1
    strneb r1,[r0],#1
    subnes  r3,r3,#1
    ldrneb r1,[r2],#1
    strneb r1,[r0],#1
    subnes  r3,r3,#1
    ldrneb r1,[r2],#1
    strneb r1,[r0],#1
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
    ldrneb r0,[r1],#1
    strneb r0,[r2],#1
    subnes  r3,r3,#1
    ldrneb r0,[r1],#1
    strneb r0,[r2],#1
    subnes  r3,r3,#1
    ldrneb r0,[r1],#1
    strneb r0,[r2],#1
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
function fpc_Pchar_length(p:Pchar):sizeint;assembler;nostackframe;[public,alias:'FPC_PCHAR_LENGTH'];compilerproc;

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
{$ifdef CPUARM_HAS_BX}
    bx  lr
{$else}
    mov pc,lr
{$endif}
.L01010101:
    .long 0x01010101
end;
{$endif}


{$ifndef darwin}
{$define FPC_SYSTEM_HAS_ANSISTR_DECR_REF}
Procedure fpc_ansistr_decr_ref (Var S : Pointer); [Public,Alias:'FPC_ANSISTR_DECR_REF'];assembler;nostackframe; compilerproc;
asm
  ldr     r1, [r0]
  // On return the pointer will always be set to zero, so utilize the delay slots
  mov     r2, #0
  str     r2, [r0]

  // Check for a zero string
  cmp     r1, #0
  // Load reference counter
  ldrne   r2, [r1, #-8]
{$ifdef CPUARM_HAS_BX}
  bxeq    lr
{$else}
  moveq   pc,lr
{$endif}
  // Check for a constant string
  cmp     r2, #0
{$ifdef CPUARM_HAS_BX}
  bxlt    lr
{$else}
  movlt   pc,lr
{$endif}
  stmfd   sp!, {r1, lr}
  sub     r0, r1, #8
  bl      InterLockedDecrement
  // InterLockedDecrement is a nice guy and sets the z flag for us
  // if the reference count dropped to 0
  ldmnefd sp!, {r1, pc}
  ldmfd sp!, {r0, lr}
  // We currently can not use constant symbols in ARM-Assembly
  // but we need to stay backward compatible with 2.6
  sub     r0, r0, #12
  // Jump without a link, so freemem directly returns to our caller
  b       FPC_FREEMEM
end;

{$define FPC_SYSTEM_HAS_ANSISTR_INCR_REF}
Procedure fpc_ansistr_incr_ref (S : Pointer); [Public,Alias:'FPC_ANSISTR_INCR_REF'];assembler;nostackframe; compilerproc;
asm
  // Null string?
  cmp     r0, #0
  // Load reference counter
  ldrne   r1, [r0, #-8]
  // pointer to counter, calculate here for delay slot utilization
  subne   r0, r0, #8
{$ifdef CPUARM_HAS_BX}
  bxeq    lr
{$else}
  moveq   pc,lr
{$endif}
  // Check for a constant string
  cmp     r1, #0
  // Tailcall
  // Hopefully the linker will place InterLockedIncrement as layed out here
  bge     InterLockedIncrement
  // Freepascal will generate a proper return here, save some cachespace
end;
{$endif not darwin}

// --- InterLocked functions begin

{$if not defined(CPUARM_HAS_LDREX) and not defined(SYSTEM_HAS_KUSER_CMPXCHG) }
  // Use generic interlock implementation

  var
    fpc_system_lock: longint;

  {$ifdef FPC_PIC}
     // Use generic interlock implementation with PIC

     // A helper function to get a pointer to fpc_system_lock in the PIC compatible way.
     function get_fpc_system_lock_ptr: pointer;
     begin
       get_fpc_system_lock_ptr:=@fpc_system_lock;
     end;

  {$endif FPC_PIC}
{$endif}

function InterLockedDecrement (var Target: longint) : longint; assembler; nostackframe;
asm
{$ifdef CPUARM_HAS_LDREX}
.Lloop:
  ldrex r1, [r0]
  sub   r1, r1, #1
  strex r2, r1, [r0]
  cmp r2, #0
  bne .Lloop
  movs r0, r1
  bx  lr
{$else}
{$ifdef SYSTEM_HAS_KUSER_CMPXCHG}
  stmfd r13!, {lr}
  mov r2, r0   // kuser_cmpxchg does not clobber r2 by definition
.Latomic_dec_loop:
  ldr r0, [r2]   // Load the current value

  // We expect this to work without looping most of the time
  // R3 gets clobbered in kuser_cmpxchg so in the unlikely case that we have to
  // loop here again, we have to reload the value. Normaly this just fills the
  // load stall-cycles from the above ldr so in reality we'll not get any additional
  // delays because of this
  // Don't use ldr to load r3 to avoid cacheline trashing
  // Load 0xffff0fff into r3 and substract to 0xffff0fc0,
  // the kuser_cmpxchg entry point
  mvn r3, #0x0000f000
  sub r3, r3, #0x3F

  sub r1, r0, #1 // Decrement value
{$ifdef CPUARM_HAS_BLX}
  blx r3	 // Call kuser_cmpxchg, sets C-Flag on success
{$else}
  mov lr, pc
{$ifdef CPUARM_HAS_BX}
  bx  r3
{$else}
  mov pc, r3
{$endif}
{$endif}
  // MOVS sets the Z flag when the result reaches zero, this can be used later on
  // The C-Flag will not be modified by this because we're not doing any shifting
  movcss r0, r1	 // We expect that to work most of the time so keep it pipeline friendly
  ldmcsfd r13!, {pc}
  b .Latomic_dec_loop // kuser_cmpxchg sets C flag on error

{$else}
// lock
  {$ifdef FPC_PIC}
    push {r0,lr}
    bl get_fpc_system_lock_ptr
    mov r3,r0
    pop {r0,lr}
  {$else FPC_PIC}
    ldr r3, .Lfpc_system_lock
  {$endif FPC_PIC}
  mov r1, #1
.Lloop:
  swp r2, r1, [r3]
  cmp r2, #0
  bne .Lloop
// do the job
  ldr r1, [r0]
  sub r1, r1, #1
  str r1, [r0]
  movs r0, r1
// unlock and return
  str r2, [r3]
{$ifdef CPUARM_HAS_BX}
  bx  lr
{$else}
  mov pc,lr
{$endif}

{$ifndef FPC_PIC}
.Lfpc_system_lock:
  .long fpc_system_lock
{$endif FPC_PIC}

{$endif}
{$endif}
end;

function InterLockedIncrement (var Target: longint) : longint; assembler; nostackframe;
asm
{$ifdef CPUARM_HAS_LDREX}
.Lloop:
  ldrex r1, [r0]
  add   r1, r1, #1
  strex r2, r1, [r0]
  cmp r2, #0
  bne .Lloop
  mov r0, r1
  bx  lr
{$else}
{$ifdef SYSTEM_HAS_KUSER_CMPXCHG}
  stmfd r13!, {lr}
  mov r2, r0   // kuser_cmpxchg does not clobber r2 by definition
.Latomic_inc_loop:
  ldr r0, [r2]   // Load the current value

  // We expect this to work without looping most of the time
  // R3 gets clobbered in kuser_cmpxchg so in the unlikely case that we have to
  // loop here again, we have to reload the value. Normaly this just fills the
  // load stall-cycles from the above ldr so in reality we'll not get any additional
  // delays because of this
  // Don't use ldr to load r3 to avoid cacheline trashing
  // Load 0xffff0fff into r3 and substract to 0xffff0fc0,
  // the kuser_cmpxchg entry point
  mvn r3, #0x0000f000
  sub r3, r3, #0x3F

  add r1, r0, #1 // Increment value
{$ifdef CPUARM_HAS_BLX}
  blx r3	 // Call kuser_cmpxchg, sets C-Flag on success
{$else}
  mov lr, pc
{$ifdef CPUARM_HAS_BX}
  bx  r3
{$else}
  mov pc, r3
{$endif}
{$endif}
  movcs r0, r1	 // We expect that to work most of the time so keep it pipeline friendly
  ldmcsfd r13!, {pc}
  b .Latomic_inc_loop // kuser_cmpxchg sets C flag on error

{$else}
// lock
  {$ifdef FPC_PIC}
    push {r0,lr}
    bl get_fpc_system_lock_ptr
    mov r3,r0
    pop {r0,lr}
  {$else FPC_PIC}
    ldr r3, .Lfpc_system_lock
  {$endif FPC_PIC}
  mov r1, #1
.Lloop:
  swp r2, r1, [r3]
  cmp r2, #0
  bne .Lloop
// do the job
  ldr r1, [r0]
  add r1, r1, #1
  str r1, [r0]
  mov r0, r1
// unlock and return
  str r2, [r3]
{$ifdef CPUARM_HAS_BX}
  bx  lr
{$else}
  mov pc,lr
{$endif}

{$ifndef FPC_PIC}
.Lfpc_system_lock:
  .long fpc_system_lock
{$endif FPC_PIC}

{$endif}
{$endif}
end;

function InterLockedExchange (var Target: longint;Source : longint) : longint; assembler; nostackframe;
asm
{$ifdef CPUARM_HAS_LDREX}
// swp is deprecated on ARMv6 and above
.Lloop:
  ldrex r2, [r0]
  strex r3, r1, [r0]
  cmp r3, #0
  bne .Lloop
  mov r0, r2
  bx  lr
{$else}
{$ifdef SYSTEM_HAS_KUSER_CMPXCHG}
  stmfd r13!, {r4, lr}
  mov r2, r0   // kuser_cmpxchg does not clobber r2 (and r1) by definition
.Latomic_add_loop:
  ldr r0, [r2]   // Load the current value

  // We expect this to work without looping most of the time
  // R3 gets clobbered in kuser_cmpxchg so in the unlikely case that we have to
  // loop here again, we have to reload the value. Normaly this just fills the
  // load stall-cycles from the above ldr so in reality we'll not get any additional
  // delays because of this
  // Don't use ldr to load r3 to avoid cacheline trashing
  // Load 0xffff0fff into r3 and substract to 0xffff0fc0,
  // the kuser_cmpxchg entry point
  mvn r3, #0x0000f000
  sub r3, r3, #0x3F
  mov r4, r0     // save the current value because kuser_cmpxchg clobbers r0
{$ifdef CPUARM_HAS_BLX}
  blx r3	 // Call kuser_cmpxchg, sets C-Flag on success
{$else}
  mov lr, pc
{$ifdef CPUARM_HAS_BX}
  bx  r3
{$else}
  mov pc, r3
{$endif}
{$endif}
  // restore the original value if needed
  movcs   r0, r4
  ldmcsfd r13!, {r4, pc}

  b .Latomic_add_loop // kuser_cmpxchg failed, loop back
{$else}
// lock
  {$ifdef FPC_PIC}
    push {r0,r1,lr}
    bl get_fpc_system_lock_ptr
    mov r3,r0
    pop {r0,r1,lr}
  {$else FPC_PIC}
    ldr r3, .Lfpc_system_lock
  {$endif FPC_PIC}
  mov r2, #1
.Lloop:
  swp r2, r2, [r3]
  cmp r2, #0
  bne .Lloop
// do the job
  ldr r2, [r0]
  str r1, [r0]
  mov r0, r2
// unlock and return
  mov r2, #0
  str r2, [r3]
{$ifdef CPUARM_HAS_BX}
  bx  lr
{$else}
  mov pc,lr
{$endif}

{$ifndef FPC_PIC}
.Lfpc_system_lock:
  .long fpc_system_lock
{$endif FPC_PIC}

{$endif}
{$endif}
end;

function InterLockedExchangeAdd (var Target: longint;Source : longint) : longint; assembler; nostackframe;
asm
{$ifdef CPUARM_HAS_LDREX}
.Lloop:
  ldrex r2, [r0]
  add   r12, r1, r2
  strex r3, r12, [r0]
  cmp r3, #0
  bne .Lloop
  mov  r0, r2
  bx  lr
{$else}
{$ifdef SYSTEM_HAS_KUSER_CMPXCHG}
  stmfd r13!, {r4, lr}
  mov r2, r0   // kuser_cmpxchg does not clobber r2 by definition
  mov r4, r1   // Save addend
.Latomic_add_loop:
  ldr r0, [r2]   // Load the current value

  // We expect this to work without looping most of the time
  // R3 gets clobbered in kuser_cmpxchg so in the unlikely case that we have to
  // loop here again, we have to reload the value. Normaly this just fills the
  // load stall-cycles from the above ldr so in reality we'll not get any additional
  // delays because of this
  // Don't use ldr to load r3 to avoid cacheline trashing
  // Load 0xffff0fff into r3 and substract to 0xffff0fc0,
  // the kuser_cmpxchg entry point
  mvn r3, #0x0000f000
  sub r3, r3, #0x3F

  add r1, r0, r4 // Add to value
{$ifdef CPUARM_HAS_BLX}
  blx r3	 // Call kuser_cmpxchg, sets C-Flag on success
{$else}
  mov lr, pc
{$ifdef CPUARM_HAS_BX}
  bx  r3
{$else}
  mov pc, r3
{$endif}
{$endif}
  // r1 does not get clobbered, so just get back the original value
  // Otherwise we would have to allocate one more register and store the
  // temporary value
  subcs   r0, r1, r4
  ldmcsfd r13!, {r4, pc}

  b .Latomic_add_loop // kuser_cmpxchg failed, loop back

{$else}
// lock
  {$ifdef FPC_PIC}
    push {r0,r1,lr}
    bl get_fpc_system_lock_ptr
    mov r3,r0
    pop {r0,r1,lr}
  {$else FPC_PIC}
    ldr r3, .Lfpc_system_lock
  {$endif FPC_PIC}
  mov r2, #1
.Lloop:
  swp r2, r2, [r3]
  cmp r2, #0
  bne .Lloop
// do the job
  ldr r2, [r0]
  add r1, r1, r2
  str r1, [r0]
  mov r0, r2
// unlock and return
  mov r2, #0
  str r2, [r3]
{$ifdef CPUARM_HAS_BX}
  bx  lr
{$else}
  mov pc,lr
{$endif}

{$ifndef FPC_PIC}
.Lfpc_system_lock:
  .long fpc_system_lock
{$endif FPC_PIC}

{$endif}
{$endif}
end;


function InterlockedCompareExchange(var Target: longint; NewValue: longint; Comperand: longint): longint; assembler; nostackframe;
asm
{$ifdef CPUARM_HAS_LDREX}
.Lloop:
  ldrex    r3, [r0]
  mov      r12, #0
  cmp      r3, r2
  strexeq  r12, r1, [r0]
  cmp      r12, #0
  bne      .Lloop
  mov      r0, r3
  bx       lr
{$else}
{$ifdef SYSTEM_HAS_KUSER_CMPXCHG}
  stmfd r13!, {r4, lr}

  mov   r4, r2 // Swap parameters around
  mov   r2, r0
  mov   r0, r4 // Use r4 because we'll need the new value for later

  // r1 and r2 will not be clobbered by kuser_cmpxchg
  // If we have to loop, r0 will be set to the original Comperand
  // kuser_cmpxchg is documented to destroy r3, therefore setting
  // r3 must be in the loop
  .Linterlocked_compare_exchange_loop:
  mvn   r3, #0x0000f000
  sub   r3, r3, #0x3F
{$ifdef CPUARM_HAS_BLX}
  blx r3       // Call kuser_cmpxchg, sets C-Flag on success
{$else}
  mov lr, pc
{$ifdef CPUARM_HAS_BX}
  bx  r3
{$else}
  mov pc, r3
{$endif}
{$endif}
  movcs r0, r4   // Return the previous value on success
  ldmcsfd r13!, {r4, pc}
  // The error case is a bit tricky, kuser_cmpxchg does not return the current value
  // So we may need to loop to avoid race conditions
  // The loop case is HIGHLY unlikely, it would require that we got rescheduled between
  // calling kuser_cmpxchg and the ldr. While beeing rescheduled another process/thread
  // would have the set the value to our comperand
  ldr   r0, [r2] // Load the currently set value
  cmp   r0, r4   // Return if Comperand != current value, otherwise loop again
  ldmnefd r13!, {r4, pc}
  // If we need to loop here, we have to
  b    .Linterlocked_compare_exchange_loop

{$else}
// lock
  {$ifdef FPC_PIC}
    push {r0,r1,r2,lr}
    bl get_fpc_system_lock_ptr
    mov r12,r0
    pop {r0,r1,r2,lr}
  {$else FPC_PIC}
    ldr r12, .Lfpc_system_lock
  {$endif FPC_PIC}
  mov r3, #1
.Lloop:
  swp r3, r3, [r12]
  cmp r3, #0
  bne .Lloop
// do the job
  ldr r3, [r0]
  cmp r3, r2
  streq r1, [r0]
  mov r0, r3
// unlock and return
  mov r3, #0
  str r3, [r12]
{$ifdef CPUARM_HAS_BX}
  bx  lr
{$else}
  mov pc,lr
{$endif}

{$ifndef FPC_PIC}
.Lfpc_system_lock:
  .long fpc_system_lock
{$endif FPC_PIC}

{$endif}
{$endif}
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

// --- InterLocked functions end

procedure fpc_cpucodeinit;
begin
{$ifdef FPC_SYSTEM_FPC_MOVE}
{$ifndef CPUARM_HAS_EDSP}
  cpu_has_edsp:=true;
  in_edsp_test:=true;
  asm
    bic r0,sp,#7

    // ldrd r0,r1,[r0]
    // encode this using .long so the rtl assembles also with instructions sets not supporting pld
    .long 0xe1c000d0
  end;
  in_edsp_test:=false;
  if cpu_has_edsp then
    moveproc:=@move_pld
  else
    moveproc:=@move_blended;
{$else CPUARM_HAS_EDSP}
  cpu_has_edsp:=true;
{$endif CPUARM_HAS_EDSP}
{$endif FPC_SYSTEM_FPC_MOVE}
end;

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

(*
This is kept for reference. Thats what the compiler COULD generate in these cases.
But FPC currently does not support inlining of asm-functions, so the whole call-overhead
is bigger than the gain of the optimized function.
function AsmSwapEndian(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif};assembler;nostackframe;
asm
	// We're starting with 4321
{$if defined(CPUARM_HAS_REV)}
	rev r0, r0		// Reverse byteorder    r0 = 1234
	mov r0, r0, shr #16	// Shift down to 16bits r0 = 0012
{$else}
	mov r0, r0, shl #16	// Shift to make that 2100
	mov r0, r0, ror #24	// Rotate to 1002
	orr r0, r0, r0 shr #16  // Shift and combine into 0012
{$endif}
end;

*)

{
  These used to be an assembler-function, but with newer improvements to the compiler this
  generates a perfect 4 cycle code sequence and can be inlined.
}
function SwapEndian(const AValue: LongWord): LongWord;{$ifdef SYSTEMINLINE}inline;{$endif}
var
  Temp: LongWord;
begin
  Temp  := AValue xor rordword(AValue,16);
  Temp  := Temp and $FF00FFFF;
  Result:= (Temp shr 8) xor rordword(AValue,8);
end;

function SwapEndian(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Result:=LongInt(SwapEndian(DWord(AValue)));
end;

{
  Currently freepascal will not generate a good assembler sequence for
  Result:=(SwapEndian(longword(lo(AValue))) shl 32) or
          (SwapEndian(longword(hi(AValue))));

  So we keep an assembly version for now
}

function SwapEndian(const AValue: Int64): Int64; assembler; nostackframe;
asm
// fpc >2.6.0 adds the "rev" instruction in the internal assembler
{$if defined(CPUARM_HAS_REV)}
        rev r2, r0
        rev r0, r1
        mov r1, r2
{$else}
        mov ip, r1

        // We're starting with r0 = $87654321
        eor r1, r0, r0, ror #16          // r1 = $C444C444
        bic r1, r1, #16711680            // r1 = r1 and $ff00ffff = $C400C444
        mov r0, r0, ror #8               // r0 = $21876543
        eor r1, r0, r1, lsr #8           // r1 = $21436587

        eor r0, ip, ip, ror #16
        bic r0, r0, #16711680
        mov ip, ip, ror #8
        eor r0, ip, r0, lsr #8

{$endif}
end;

function SwapEndian(const AValue: QWord): QWord; {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  Result:=QWord(SwapEndian(Int64(AValue)));
end;

{$ifndef FPC_SYSTEM_HAS_MEM_BARRIER}
{$define FPC_SYSTEM_HAS_MEM_BARRIER}

{ Generic read/readwrite barrier code.  }
procedure barrier; assembler; nostackframe;
asm
  // manually encode the instructions to avoid bootstrap and -march external
  // assembler settings 
{$ifdef CPUARM_HAS_DMB}
  .long 0xf57ff05f  // dmb sy
{$else CPUARM_HAS_DMB}
{$ifdef CPUARMV6}
  mov r0, #0
  .long 0xee070fba  // mcr 15, 0, r0, cr7, cr10, {5}
{$else CPUARMV6}
{$ifdef SYSTEM_HAS_KUSER_MEMORY_BARRIER}
  stmfd r13!, {lr}
  mvn   r0, #0x0000f000
  sub   r0, r0, #0x5F
{$ifdef CPUARM_HAS_BLX}
  blx r0       // Call kuser_memory_barrier at address 0xffff0fa0
{$else CPUARM_HAS_BLX}
  mov lr, pc
{$ifdef CPUARM_HAS_BX}
  bx  r0
{$else CPUARM_HAS_BX}
  mov pc, r0
{$endif CPUARM_HAS_BX}
{$endif CPUARM_HAS_BLX}
  ldmfd r13!, {pc}
{$endif SYSTEM_HAS_KUSER_MEMORY_BARRIER}
{$endif CPUARMV6}
{$endif CPUARM_HAS_DMB}
end;

procedure ReadBarrier;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  barrier;
end;

procedure ReadDependencyBarrier;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  { reads imply barrier on earlier reads depended on; not required on ARM }
end;

procedure ReadWriteBarrier;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  barrier;
end;

procedure WriteBarrier; assembler; nostackframe;
asm
  // specialize the write barrier because according to ARM, implementations for
  // "dmb st" may be more optimal than the more generic "dmb sy"
{$ifdef CPUARM_HAS_DMB}
  .long 0xf57ff05e  // dmb st
{$else CPUARM_HAS_DMB}
{$ifdef CPUARMV6}
  mov r0, #0
  .long 0xee070fba  // mcr 15, 0, r0, cr7, cr10, {5} 
{$else CPUARMV6}
{$ifdef SYSTEM_HAS_KUSER_MEMORY_BARRIER}
  stmfd r13!, {lr}
  mvn   r0, #0x0000f000
  sub   r0, r0, #0x5F
{$ifdef CPUARM_HAS_BLX}
  blx r0       // Call kuser_memory_barrier at address 0xffff0fa0
{$else CPUARM_HAS_BLX}
  mov lr, pc
{$ifdef CPUARM_HAS_BX}
  bx  r0
{$else CPUARM_HAS_BX}
  mov pc, r0
{$endif CPUARM_HAS_BX}
{$endif CPUARM_HAS_BLX}
  ldmfd r13!, {pc}
{$endif SYSTEM_HAS_KUSER_MEMORY_BARRIER}
{$endif CPUARMV6}
{$endif CPUARM_HAS_DMB}
end;

{$endif}

{include hand-optimized assembler division code}
{$i divide.inc}
