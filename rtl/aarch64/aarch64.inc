{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 by Jonas Maebe, member of
    the Free Pascal development team.

    Processor dependent implementation for the system unit for
    AArch64

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$IFNDEF LINUX}
    {$DEFINE USE_DCBZ}
{$ENDIF LINUX}

{****************************************************************************
                           AArch64 specific stuff
****************************************************************************}
const
  fpu_ioe = 1 shl 8;
  fpu_dze = 1 shl 9;
  fpu_ofe = 1 shl 10;
  fpu_ufe = 1 shl 11;
  fpu_ixe = 1 shl 12;
  fpu_ide = 1 shl 15;
  fpu_exception_mask = fpu_ioe or fpu_dze or fpu_ofe or fpu_ufe or fpu_ixe or fpu_ide;
  fpu_exception_mask_to_status_mask_shift = 8;

function getfpcr: dword; nostackframe; assembler;
  asm
    mrs x0,fpcr
  end;


procedure setfpcr(val: dword); nostackframe; assembler;
  asm
    msr fpcr,x0
  end;


function getfpsr: dword; nostackframe; assembler;
  asm
    mrs x0,fpsr
  end;


procedure setfpsr(val: dword); nostackframe; assembler;
  asm
    msr fpsr, x0
  end;


const
  FPSR_IOC = 1;
  FPSR_DZC = 1 shl 1;
  FPSR_OFC = 1 shl 2;
  FPSR_UFC = 1 shl 3;
  FPSR_IXC = 1 shl 4;
  FPSR_IDC = 1 shl 7;
  FPSR_EXCEPTIONS = FPSR_IOC or FPSR_DZC or FPSR_OFC or FPSR_UFC or FPSR_IXC or FPSR_IDC;


procedure RaisePendingExceptions;
  var
    fpsr : dword;
    f: TFPUException;
  begin
    fpsr:=getfpsr;
    if (fpsr and FPSR_DZC) <> 0 then
      float_raise(exZeroDivide);
    if (fpsr and FPSR_OFC) <> 0 then
      float_raise(exOverflow);
    if (fpsr and FPSR_UFC) <> 0 then
      float_raise(exUnderflow);
    if (fpsr and FPSR_IOC) <> 0 then
      float_raise(exInvalidOp);
    if (fpsr and FPSR_IXC) <> 0 then
      float_raise(exPrecision);
    if (fpsr and FPSR_IDC) <> 0 then
      float_raise(exDenormalized);
    { now the soft float exceptions }
    for f in softfloat_exception_flags do
      float_raise(f);
  end;


{ as so far no AArch64 flavour which supports hard floating point exceptions, we use solely
  the softfloat_exception_mask for masking as the masking flags are RAZ and WI if floating point
  exceptions are not supported }
procedure fpc_throwfpuexception;[public,alias:'FPC_THROWFPUEXCEPTION'];
  var
    fpsr : dword;
    f: TFPUException;
  begin
    { at this point, we know already, that an exception will be risen }
    fpsr:=getfpsr;

    { check, if the exception is masked }
    if ((fpsr and FPSR_DZC) <> 0) and (exZeroDivide in softfloat_exception_mask) then
      fpsr:=fpsr and not(FPSR_DZC);
    if ((fpsr and FPSR_OFC) <> 0) and (exOverflow in softfloat_exception_mask) then
      fpsr:=fpsr and not(FPSR_OFC);
    if ((fpsr and FPSR_UFC) <> 0) and (exUnderflow in softfloat_exception_mask) then
      fpsr:=fpsr and not(FPSR_UFC);
    if ((fpsr and FPSR_IOC) <> 0) and (exInvalidOp in softfloat_exception_mask) then
      fpsr:=fpsr and not(FPSR_IOC);
    if ((fpsr and FPSR_IXC) <> 0) and (exPrecision in softfloat_exception_mask) then
      fpsr:=fpsr and not(FPSR_IXC);
    if ((fpsr and FPSR_IDC) <> 0) and (exDenormalized in softfloat_exception_mask) then
      fpsr:=fpsr and not(FPSR_IDC);
    setfpsr(fpsr);
    if (fpsr and FPSR_EXCEPTIONS)<>0 then
      RaisePendingExceptions;
  end;


{$define FPC_SYSTEM_HAS_SYSINITFPU}
procedure SysInitFPU;
  begin
    softfloat_rounding_mode:=rmNearest;
    { 0 is rmNearest }
    setfpcr(getfpcr and $ff3fffff);
    { clear all "exception happened" flags we care about}
    setfpsr(getfpsr and not(fpu_exception_mask shr fpu_exception_mask_to_status_mask_shift));
    { enable invalid operations and division by zero exceptions. }
    setfpcr(((getfpcr and not(fpu_exception_mask)) or fpu_dze or fpu_ofe or fpu_ioe));
    softfloat_exception_mask:=[float_flag_underflow,float_flag_inexact,float_flag_denormal];
    softfloat_exception_flags:=[];
  end;


{$define FPC_SYSTEM_HAS_SYSRESETFPU}
Procedure SysResetFPU;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  softfloat_exception_flags:=[];
  setfpsr(getfpsr and not(FPSR_EXCEPTIONS));
end;


procedure fpc_cpuinit;
  begin
    { don't let libraries influence the FPU cw set by the host program }
    if not IsLibrary then
      SysInitFPU;
  end;


{****************************************************************************
                                Move / Fill
****************************************************************************}


{****************************************************************************
                                 String
****************************************************************************}

{$define FPC_SYSTEM_HAS_GET_CALLER_ADDR}
function get_caller_addr(framebp:pointer;addr:pointer=nil):pointer;assembler; nostackframe;
  asm
    cbz x0, .Lcaller_addr_invalid
    ldur x0, [x0]
    cbz x0, .Lcaller_addr_invalid
    ldur x0, [x0, #8]
   .Lcaller_addr_invalid:
  end;


{$define FPC_SYSTEM_HAS_GET_CALLER_FRAME}
function get_caller_frame(framebp:pointer;addr:pointer=nil):pointer;assembler; nostackframe;
  asm
    cbz x0, .Lcaller_addr_invalid
    ldur x0, [x0]
   .Lcaller_addr_invalid:
  end;


{$define FPC_SYSTEM_HAS_SPTR}
Function Sptr : Pointer;assembler; nostackframe;
  asm
    mov x0, sp
  end;


{****************************************************************************
                                 Str()
****************************************************************************}

{ int_str: generic implementation is used for now }


{****************************************************************************
                             Multithreading
****************************************************************************}

{ perform a thread-safe inc/dec }

{$define FPC_SYSTEM_HAS_DECLOCKED_LONGINT}
function declocked(var l : longint) : boolean;assembler;nostackframe;
  { input:  address of l in x0                                      }
  { output: boolean indicating whether l is zero after decrementing }
  asm
  .LDecLockedLoop:
    ldxr   w1,[x0]
    sub    w1,w1,#1
    stxr   w2,w1,[x0]
    cbnz   w2,.LDecLockedLoop
    cset   w0, eq
  end;


{$define FPC_SYSTEM_HAS_INCLOCKED_LONGINT}
procedure inclocked(var l : longint);assembler;nostackframe;
  asm
  .LIncLockedLoop:
    ldxr   w1,[x0]
    add    w1,w1,#1
    stxr   w2,w1,[x0]
    cbnz   w2,.LIncLockedLoop
  end;


{$define FPC_SYSTEM_HAS_DECLOCKED_INT64}
function declocked(var l : int64) : boolean;assembler;nostackframe;
  { input:  address of l in x0                                      }
  { output: boolean indicating whether l is zero after decrementing }
  asm
  .LDecLockedLoop:
    ldxr   x1,[x0]
    subs   x1,x1,#1
    stxr   w2,x1,[x0]
    cbnz   w2,.LDecLockedLoop
    cset   w0, eq
  end;


{$define FPC_SYSTEM_HAS_INCLOCKED_INT64}
procedure inclocked(var l : int64);assembler;nostackframe;
  asm
  .LIncLockedLoop:
    ldxr   x1,[x0]
    add    x1,x1,#1
    stxr   w2,x1,[x0]
    cbnz   w2,.LIncLockedLoop
  end;


function InterLockedDecrement (var Target: longint) : longint; assembler; nostackframe;
  { input:  address of target in x0 }
  { output: target-1 in x0          }
  { side-effect: target := target-1 }
  asm
  .LInterDecLockedLoop:
    ldxr   w1,[x0]
    sub    w1,w1,#1
    stxr   w2,w1,[x0]
    cbnz   w2,.LInterDecLockedLoop
    mov    w0,w1
  end;


function InterLockedIncrement (var Target: longint) : longint; assembler; nostackframe;
  { input:  address of target in x0 }
  { output: target+1 in x0          }
  { side-effect: target := target+1 }
  asm
  .LInterIncLockedLoop:
    ldxr   w1,[x0]
    add    w1,w1,#1
    stxr   w2,w1,[x0]
    cbnz   w2,.LInterIncLockedLoop
    mov    w0,w1
  end;


function InterLockedExchange (var Target: longint;Source : longint) : longint; assembler; nostackframe;
  { input:  address of target in x0, source in w1 }
  { output: target in x0                          }
  { side-effect: target := source                 }
  asm
  .LInterLockedXchgLoop:
    ldxr   w2,[x0]
    stxr   w3,w1,[x0]
    cbnz   w3,.LInterLockedXchgLoop
    mov    w0,w2
  end;


function InterLockedExchangeAdd (var Target: longint;Source : longint) : longint; assembler; nostackframe;
  asm
  .LInterLockedXchgAddLoop:
    ldxr   w2,[x0]
    add    w4,w2,w1
    stxr   w3,w4,[x0]
    cbnz   w3,.LInterLockedXchgAddLoop
    mov    w0,w2
  end;


function InterlockedCompareExchange(var Target: longint; NewValue: longint; Comperand: longint): longint; assembler; nostackframe;
  { input:  address of target in x0, newvalue in w1, comparand in w2 }
  { output: value stored in target before entry of the function      }
  { side-effect: NewValue stored in target if (target = comparand)   }
  asm
  .LInterlockedCompareExchangeLoop:
    ldxr   w3,[x0]
    cmp    w3,w2
    csel   w4,w1,w3,eq
    stxr   w5,w4,[x0]
    cbnz   w5,.LInterlockedCompareExchangeLoop
    mov    w0,w3
  end;


function InterLockedDecrement64 (var Target: int64) : int64; assembler; nostackframe;
  asm
  .LInterDecLockedLoop:
    ldxr   x1,[x0]
    sub    x1,x1,#1
    stxr   w2,x1,[x0]
    cbnz   w2,.LInterDecLockedLoop
    mov    x0,x1
  end;


function InterLockedIncrement64 (var Target: int64) : int64; assembler; nostackframe;
  asm
  .LInterIncLockedLoop:
    ldxr   x1,[x0]
    add    x1,x1,#1
    stxr   w2,x1,[x0]
    cbnz   w2,.LInterIncLockedLoop
    mov    x0,x1
  end;


function InterLockedExchange64 (var Target: int64;Source : int64) : int64; assembler; nostackframe;
  asm
  .LInterLockedXchgLoop:
    ldxr   x2,[x0]
    stxr   w3,x1,[x0]
    cbnz   w3,.LInterLockedXchgLoop
    mov    x0,x2
  end;


function InterLockedExchangeAdd64 (var Target: int64;Source : int64) : int64; assembler; nostackframe;
  asm
  .LInterLockedXchgAddLoop:
    ldxr   x2,[x0]
    add    x4,x2,x1
    stxr   w3,x4,[x0]
    cbnz   w3,.LInterLockedXchgAddLoop
    mov    x0,x2
  end;


function InterLockedCompareExchange64(var Target: int64; NewValue, Comperand : int64): int64; assembler; nostackframe;
  asm
  .LInterlockedCompareExchangeLoop:
    ldxr   x3,[x0]
    cmp    x3,x2
    csel   x4,x1,x3,eq
    stxr   w5,x4,[x0]
    cbnz   w5,.LInterlockedCompareExchangeLoop
    mov    x0,x3
  end;


{$ifndef FPC_SYSTEM_HAS_MEM_BARRIER}
{$define FPC_SYSTEM_HAS_MEM_BARRIER}
procedure ReadBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
  asm
    { dmb ishld }
    dmb #9
  end;

procedure ReadDependencyBarrier;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  { reads imply barrier on earlier reads depended on }
end;

procedure ReadWriteBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
  { dmb ish }
  dmb #11
end;

procedure WriteBarrier;assembler;nostackframe;{$ifdef SYSTEMINLINE}inline;{$endif}
asm
  { dmb ishst }
  dmb #10
end;

{$endif}
