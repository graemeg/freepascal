{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt,
    member of the Free Pascal development team.

    Syscall implementation for Linux powerpc 

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{No debugging for syslinux include !}
{$IFDEF SYS_LINUX}
  {$UNDEF SYSCALL_DEBUG}
{$ENDIF SYS_LINUX}


{ This variable is needed in syscall.inc assembler code }
var
  fpc_threadvar_relocate_proc : TRelocateThreadVarHandler; public name 'FPC_THREADVAR_RELOCATE';
{$define THREADVAR_RELOCATED_ALREADY_DEFINED}

{*****************************************************************************
                     --- Main:The System Call Self ---
*****************************************************************************}

function FpSysCall(sysnr:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL0'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  temp, retaddress: longint;
asm
  mr  r0,r3
  sc
  bns   .LDone
  lis r10,(fpc_threadvar_relocate_proc)@ha
  lwz   r10,(fpc_threadvar_relocate_proc)@l(r10)
  cmpwi r10,0
  bne   .LThreaded
  lis   r4,(Errno+4)@ha
  stw   r3,(Errno+4)@l(r4)
  b     .LFailed
.LThreaded:
  stw   r3,temp
  mflr  r3
  mtctr r10
  lis   r4,(errno)@ha
  stw   r3,retaddress
  lwz   r3,(errno)@l(r4)
  bctrl
  lwz   r4,temp
  lwz   r5,retaddress
  stw   r4,0(r3)
  mtlr  r5
.LFailed:
  li    r3,-1
.LDone:
end;

function FpSysCall(sysnr,param1:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL1'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  temp, retaddress: longint;
asm
  mr  r0,r3
  mr  r3,r4
  sc
  bns   .LDone
  lis   r10,(fpc_threadvar_relocate_proc)@ha
  lwz   r10,(fpc_threadvar_relocate_proc)@l(r10)
  cmpwi r10,0
  bne   .LThreaded
  lis   r4,(Errno+4)@ha
  stw   r3,(Errno+4)@l(r4)
  b     .LFailed
.LThreaded:
  stw   r3,temp
  mflr  r3
  mtctr r10
  lis   r4,(errno)@ha
  stw   r3,retaddress
  lwz   r3,(errno)@l(r4)
  bctrl
  lwz   r4,temp
  lwz   r5,retaddress
  stw   r4,0(r3)
  mtlr  r5
.LFailed:
  li    r3,-1
.LDone:
end;


function FpSysCall(sysnr,param1,param2:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL2'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  temp, retaddress: longint;
asm
  mr  r0,r3
  mr  r3,r4
  mr  r4,r5
  sc
  bns   .LDone
  lis   r10,(fpc_threadvar_relocate_proc)@ha
  lwz   r10,(fpc_threadvar_relocate_proc)@l(r10)
  cmpwi r10,0
  bne   .LThreaded
  lis   r4,(Errno+4)@ha
  stw   r3,(Errno+4)@l(r4)
  b     .LFailed
.LThreaded:
  stw   r3,temp
  mflr  r3
  mtctr r10
  lis   r4,(errno)@ha
  stw   r3,retaddress
  lwz   r3,(errno)@l(r4)
  bctrl
  lwz   r4,temp
  lwz   r5,retaddress
  stw   r4,0(r3)
  mtlr  r5
.LFailed:
  li    r3,-1
.LDone:
end;

function FpSysCall(sysnr,param1,param2,param3:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL3'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  temp, retaddress: longint;
asm
  mr  r0,r3
  mr  r3,r4
  mr  r4,r5
  mr  r5,r6
  sc
  bns   .LDone
  lis   r10,(fpc_threadvar_relocate_proc)@ha
  lwz   r10,(fpc_threadvar_relocate_proc)@l(r10)
  cmpwi r10,0
  bne   .LThreaded
  lis   r4,(Errno+4)@ha
  stw   r3,(Errno+4)@l(r4)
  b     .LFailed
.LThreaded:
  stw   r3,temp
  mflr  r3
  mtctr r10
  lis   r4,(errno)@ha
  stw   r3,retaddress
  lwz   r3,(errno)@l(r4)
  bctrl
  lwz   r4,temp
  lwz   r5,retaddress
  stw   r4,0(r3)
  mtlr  r5
.LFailed:
  li    r3,-1
.LDone:
end;


function FpSysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL4'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  temp, retaddress: longint;
asm
  mr  r0,r3
  mr  r3,r4
  mr  r4,r5
  mr  r5,r6
  mr  r6,r7
  sc
  bns   .LDone
  lis   r10,(fpc_threadvar_relocate_proc)@ha
  lwz   r10,(fpc_threadvar_relocate_proc)@l(r10)
  cmpwi r10,0
  bne   .LThreaded
  lis   r4,(Errno+4)@ha
  stw   r3,(Errno+4)@l(r4)
  b     .LFailed
.LThreaded:
  stw   r3,temp
  mflr  r3
  mtctr r10
  lis   r4,(errno)@ha
  stw   r3,retaddress
  lwz   r3,(errno)@l(r4)
  bctrl
  lwz   r4,temp
  lwz   r5,retaddress
  stw   r4,0(r3)
  mtlr  r5
.LFailed:
  li    r3,-1
.LDone:
end;

function FpSysCall(sysnr,param1,param2,param3,param4,param5:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL5'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  temp, retaddress: longint;
asm
  mr  r0,r3
  mr  r3,r4
  mr  r4,r5
  mr  r5,r6
  mr  r6,r7
  mr  r7,r8
  sc
  bns   .LDone
  lis   r10,(fpc_threadvar_relocate_proc)@ha
  lwz   r10,(fpc_threadvar_relocate_proc)@l(r10)
  cmpwi r10,0
  bne   .LThreaded
  lis   r4,(Errno+4)@ha
  stw   r3,(Errno+4)@l(r4)
  b     .LFailed
.LThreaded:
  stw   r3,temp
  mflr  r3
  mtctr r10
  lis   r4,(errno)@ha
  stw   r3,retaddress
  lwz   r3,(errno)@l(r4)
  bctrl
  lwz   r4,temp
  lwz   r5,retaddress
  stw   r4,0(r3)
  mtlr  r5
.LFailed:
  li    r3,-1
.LDone:
end;


function FpSysCall(sysnr,param1,param2,param3,param4,param5,param6:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL6'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  temp, retaddress: longint;
asm
  mr  r0,r3
  mr  r3,r4
  mr  r4,r5
  mr  r5,r6
  mr  r6,r7
  mr  r7,r8
  mr  r8,r9
  sc
  bns   .LDone
  lis   r10,(fpc_threadvar_relocate_proc)@ha
  lwz   r10,(fpc_threadvar_relocate_proc)@l(r10)
  cmpwi r10,0
  bne   .LThreaded
  lis   r4,(Errno+4)@ha
  stw   r3,(Errno+4)@l(r4)
  b     .LFailed
.LThreaded:
  stw   r3,temp
  mflr  r3
  mtctr r10
  lis   r4,(errno)@ha
  stw   r3,retaddress
  lwz   r3,(errno)@l(r4)
  bctrl
  lwz   r4,temp
  lwz   r5,retaddress
  stw   r4,0(r3)
  mtlr  r5
.LFailed:
  li    r3,-1
.LDone:
end;



