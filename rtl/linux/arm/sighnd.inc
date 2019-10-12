{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt,
    member of the Free Pascal development team.

    Signal handler is arch dependant due to processor to language
    exception conversion.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function GetHandleErrorAddrFrameAddr: pointer;
begin
  result:=@HandleErrorAddrFrame;
end;

{$ifndef CPUTHUMB}
Procedure SignalToHandleErrorAddrFrame_ARM(Errno : longint;addr : CodePointer; frame : Pointer); nostackframe; assembler;
asm
{$if FPC_VERSION >= 30200}
.code 32
{$endif}
  // the address is of the faulting instruction, and sigreturn will
  //  skip it -> start with a nop
  nop
  push {r0,r1,r2,r3}
  bl GetHandleErrorAddrFrameAddr
  // overwrite last stack slot with new return address
  str r0, [sp,#12]
  // lr := addr
  ldr lr, [sp,#4]
  pop {r0,r1,r2,pc}
.text
end;
{$endif not CPUTHUMB}

{$if FPC_VERSION >= 30200}
Procedure SignalToHandleErrorAddrFrame_Thumb(Errno : longint;addr : CodePointer; frame : Pointer); nostackframe; assembler;
asm
.thumb_func
.code 16
  // the address is of the faulting instruction, and sigreturn will
  // skip it -> start with a nop
  nop
  push {r0,r1,r2,r3}
  bl GetHandleErrorAddrFrameAddr
  // overwrite last stack slot with new return address
  str r0, [sp,#12]
  // lr := addr
  ldr r0, [sp,#4]
  mov lr, r0
  pop {r0,r1,r2,pc}
.text
end;
{$endif}

procedure SignalToRunerror(Sig: longint; { _a2,_a3,_a4 : dword; } SigContext: PSigInfo; uContext : PuContext); public name '_FPC_DEFAULTSIGHANDLER'; cdecl;
var
  res : word;
begin
  res:=0;
  case sig of
    SIGFPE :
        begin
          { don't know how to find the different causes, maybe via xer? }
          res := 207;
          SysResetFPU;
        end;
    SIGILL:
{$ifdef FPC_SYSTEM_FPC_MOVE}
        if in_edsp_test then
          begin
            res:=0;
            cpu_has_edsp:=false;
            inc(uContext^.uc_mcontext.arm_pc,4);
          end
        else
{$endif FPC_SYSTEM_FPC_MOVE}
          res:=216;
    SIGSEGV :
        res:=216;
    SIGBUS:
        res:=214;
    SIGINT:
        res:=217;
    SIGQUIT:
        res:=233;
  end;
  { give runtime error at the position where the signal was raised }
  if res<>0 then
    begin
      ucontext^.uc_mcontext.arm_r0:=res;
      ucontext^.uc_mcontext.arm_r1:=uContext^.uc_mcontext.arm_pc;
      ucontext^.uc_mcontext.arm_r2:=uContext^.uc_mcontext.arm_fp;
{$if FPC_VERSION >= 30200}
{$ifndef CPUTHUMB}
      if (ucontext^.uc_mcontext.arm_cpsr and (1 shl 5))=0 then
        begin
          ucontext^.uc_mcontext.arm_pc:=ptruint(@SignalToHandleErrorAddrFrame_ARM);
        end
      else
{$endif not CPUTHUMB}
        begin
          ucontext^.uc_mcontext.arm_pc:=ptruint(@SignalToHandleErrorAddrFrame_Thumb);
        end;
{$else}
      ucontext^.uc_mcontext.arm_pc:=ptruint(@SignalToHandleErrorAddrFrame_ARM);
{$endif}
    end;
end;


