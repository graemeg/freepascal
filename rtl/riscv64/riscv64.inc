{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by the Free Pascal development team.

    Processor dependent implementation for the system unit for
    RiscV64

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ Common RiscV stuff }
{$I ../riscv/riscv.inc}

{****************************************************************************
                       stack frame related stuff
****************************************************************************}

{$IFNDEF INTERNAL_BACKTRACE}
{$define FPC_SYSTEM_HAS_GET_FRAME}
function get_frame:pointer;assembler;nostackframe;
  asm
    addi a0, fp, 0
  end;
{$ENDIF not INTERNAL_BACKTRACE}


{$define FPC_SYSTEM_HAS_GET_CALLER_ADDR}
function get_caller_addr(framebp:pointer;addr:pointer=nil):pointer;assembler;
  asm
    ld a0, -8*1(a0)
  end;


{$define FPC_SYSTEM_HAS_GET_CALLER_FRAME}
function get_caller_frame(framebp:pointer;addr:pointer=nil):pointer;assembler;
  asm
    ld a0, -8*2(a0)
  end;


{$define FPC_SYSTEM_HAS_SPTR}
Function Sptr : pointer;assembler;nostackframe;
  asm
    addi a0, sp, 0
  end;


function InterLockedDecrement (var Target: longint) : longint; assembler; nostackframe;
  asm
{$ifdef CPURV_HAS_ATOMIC}
    addi a1, x0, -1
    amoadd.w a0, a1, (a0)
    addw a0, a0, a1
{$else CPURV_HAS_ATOMIC}
    lw a1, 0(a0)
    addiw a1, a1, -1
    sw a1, 0(a0)
    addi a0, a1, 0
{$endif CPURV_HAS_ATOMIC}
  end;


function InterLockedIncrement (var Target: longint) : longint; assembler; nostackframe;
  asm
{$ifdef CPURV_HAS_ATOMIC}
    addi a1, x0, 1
    amoadd.w a0, a1, (a0)
    addw a0, a0, a1
{$else CPURV_HAS_ATOMIC}
    lw a1, 0(a0)
    addiw a1, a1, 1
    sw a1, 0(a0)
    addi a0, a1, 0
{$endif CPURV_HAS_ATOMIC}
  end;


function InterLockedExchange (var Target: longint;Source : longint) : longint; assembler; nostackframe;
  asm
{$ifdef CPURV_HAS_ATOMIC}
    amoswap.w a0, a1, (a0)
{$else CPURV_HAS_ATOMIC}
    lw a2, 0(a0)
    sw a1, 0(a0)
    addi a0, a2, 0
{$endif CPURV_HAS_ATOMIC}
  end;


function InterlockedCompareExchange(var Target: longint; NewValue: longint; Comperand: longint): longint; assembler; nostackframe;
  asm
{$ifdef CPURV_HAS_ATOMIC}
  .LLoop:
    lr.w a3, 0(a0)
    bne a3, a2, .LFail
    sc.w a4, a1, 0(a0)
    bne a4, x0, .LLoop
  .LFail:
    addi a0, a3, 0
{$else CPURV_HAS_ATOMIC}
    lw a3, 0(a0)
    bne a3, a2, .LFail
    sw a1, 0(a0)
  .LFail:
    addi a0, a3, 0
{$endif CPURV_HAS_ATOMIC}
  end;


function InterLockedExchangeAdd (var Target: longint;Source : longint) : longint; assembler; nostackframe;
  asm
{$ifdef CPURV_HAS_ATOMIC}
    amoadd.w a0, a1, (a0)
{$else CPURV_HAS_ATOMIC}
    lw a2, 0(a0)
    addw a2, a2, a1
    sw a2, 0(a0)
    addi a0, a2, 0
{$endif CPURV_HAS_ATOMIC}
  end;



function InterLockedDecrement64 (var Target: int64) : int64; assembler; nostackframe;
  asm
{$ifdef CPURV_HAS_ATOMIC}
    addi a1, x0, -1
    amoadd.d a0, a1, (a0)
    add a0, a0, a1
{$else CPURV_HAS_ATOMIC}
    ld a1, 0(a0)
    addi a1, a1, -1
    sd a1, 0(a0)
    addi a0, a1, 0
{$endif CPURV_HAS_ATOMIC}
  end;


function InterLockedIncrement64 (var Target: int64) : int64; assembler; nostackframe;
  asm
{$ifdef CPURV_HAS_ATOMIC}
    addi a1, x0, 1
    amoadd.d a0, a1, (a0)
    add a0, a0, a1
{$else CPURV_HAS_ATOMIC}
    ld a1, 0(a0)
    addi a1, a1, 1
    sd a1, 0(a0)
    addi a0, a1, 0
{$endif CPURV_HAS_ATOMIC}
  end;


function InterLockedExchange64 (var Target: int64;Source : int64) : int64; assembler; nostackframe;
  asm
{$ifdef CPURV_HAS_ATOMIC}
    amoswap.d a0, a1, (a0)
{$else CPURV_HAS_ATOMIC}
    ld a2, 0(a0)
    sd a1, 0(a0)
    addi a0, a2, 0
{$endif CPURV_HAS_ATOMIC}
  end;


function InterlockedCompareExchange64(var Target: int64; NewValue: int64; Comperand: int64): int64; assembler; nostackframe;
  asm
{$ifdef CPURV_HAS_ATOMIC}
  .LLoop:
    lr.d a3, 0(a0)
    bne a3, a2, .LFail
    sc.d a4, a1, 0(a0)
    bne a4, x0, .LLoop
  .LFail:
    addi a0, a3, 0
{$else CPURV_HAS_ATOMIC}
    ld a3, 0(a0)
    bne a3, a2, .LFail
    sd a1, 0(a0)
  .LFail:
    addi a0, a3, 0
{$endif CPURV_HAS_ATOMIC}
  end;


function InterLockedExchangeAdd64 (var Target: int64;Source : int64) : int64; assembler; nostackframe;
  asm
{$ifdef CPURV_HAS_ATOMIC}
    amoadd.d a0, a1, (a0)
{$else CPURV_HAS_ATOMIC}
    ld a2, 0(a0)
    add a2, a2, a1
    sd a2, 0(a0)
    addi a0, a2, 0
{$endif CPURV_HAS_ATOMIC}
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


{$define FPC_SYSTEM_HAS_DECLOCKED_INT64}
function declocked(var l:int64):boolean;
  begin
    Result:=InterLockedDecrement64(l) = 0;
  end;


{$define FPC_SYSTEM_HAS_INCLOCKED_INT64}
procedure inclocked(var l:int64);
  begin
    InterLockedIncrement64(l);
  end;


{$define FPC_SYSTEM_HAS_MEM_BARRIER}

procedure ReadBarrier; assembler; nostackframe;
  asm
    fence ir, ir
  end;


procedure ReadDependencyBarrier;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
  end;

procedure ReadWriteBarrier; assembler; nostackframe;
  asm
    fence iorw, iorw
  end;

procedure WriteBarrier; assembler; nostackframe;
  asm
    fence ow, ow
  end;
