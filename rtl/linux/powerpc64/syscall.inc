{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Thomas Schatzl,
    member of the Free Pascal development team.

    Linux powerpc syscall wrapper implementation 

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
  { workaround for incomplete linkage area reservation by FPC 3.0.0 and earlier,
    remove after 3.0.2 is the minimum bootstrap version }
  dummy: array[1..4] of ptruint;
  retaddress: ptruint;
asm
  mr r0, r3
  sc
  bns .LDone
  mflr r4
  std r4, retaddress
  bl seterrno
  nop
  ld r4, retaddress
  li r3, -1
  mtlr r4
.LDone:
end;

function FpSysCall(sysnr,param1:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL1'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  { workaround for incomplete linkage area reservation by FPC 3.0.0 and earlier,
    remove after 3.0.2 is the minimum bootstrap version }
  dummy: array[1..4] of ptruint;
  retaddress: ptruint;
asm
  mr r0, r3
  mr r3, r4
  sc
  bns .LDone
  mflr r4
  std r4, retaddress
  bl seterrno
  nop
  ld r4, retaddress
  li r3, -1
  mtlr r4
.LDone:
end;

function FpSysCall(sysnr,param1,param2:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL2'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  { workaround for incomplete linkage area reservation by FPC 3.0.0 and earlier,
    remove after 3.0.2 is the minimum bootstrap version }
  dummy: array[1..4] of ptruint;
  retaddress: ptruint;
asm
  mr r0, r3
  mr r3, r4
  mr r4, r5
  sc
  bns .LDone
  mflr r4
  std r4, retaddress
  bl seterrno
  nop
  ld r4, retaddress
  li r3, -1
  mtlr r4
.LDone:
end;


function FpSysCall(sysnr,param1,param2,param3:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL3'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  { workaround for incomplete linkage area reservation by FPC 3.0.0 and earlier,
    remove after 3.0.2 is the minimum bootstrap version }
  dummy: array[1..4] of ptruint;
  retaddress: ptruint;
asm
  mr r0, r3
  mr r3, r4
  mr r4, r5
  mr r5, r6
  sc
  bns .LDone
  mflr r4
  std r4, retaddress
  bl seterrno
  nop
  ld r4, retaddress
  li r3, -1
  mtlr r4
.LDone:
end;


function FpSysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL4'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  { workaround for incomplete linkage area reservation by FPC 3.0.0 and earlier,
    remove after 3.0.2 is the minimum bootstrap version }
  dummy: array[1..4] of ptruint;
  retaddress: ptruint;
asm
  mr r0, r3
  mr r3, r4
  mr r4, r5
  mr r5, r6
  mr r6, r7
  sc
  bns .LDone
  mflr r4
  std r4, retaddress
  bl seterrno
  nop
  ld r4, retaddress
  li r3, -1
  mtlr r4
.LDone:
end;


function FpSysCall(sysnr,param1,param2,param3,param4,param5:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL5'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  { workaround for incomplete linkage area reservation by FPC 3.0.0 and earlier,
    remove after 3.0.2 is the minimum bootstrap version }
  dummy: array[1..4] of ptruint;
  retaddress: ptruint;
asm
  mr r0, r3
  mr r3, r4
  mr r4, r5
  mr r5, r6
  mr r6, r7
  mr r7, r8
  sc
  bns .LDone
  mflr r4
  std r4, retaddress
  bl seterrno
  nop
  ld r4, retaddress
  li r3, -1
  mtlr r4
.LDone:
end;


function FpSysCall(sysnr,param1,param2,param3,param4,param5,param6:TSysParam):TSysResult; assembler;[public,alias:'FPC_SYSCALL6'];
{
  This function puts the registers in place, does the call, and then
  copies back the registers as they are after the SysCall.
}
var
  { workaround for incomplete linkage area reservation by FPC 3.0.0 and earlier,
    remove after 3.0.2 is the minimum bootstrap version }
  dummy: array[1..4] of ptruint;
  retaddress: ptruint;
asm
  mr r0, r3
  mr r3, r4
  mr r4, r5
  mr r5, r6
  mr r6, r7
  mr r7, r8
  mr r8, r9
  sc
  bns .LDone
  mflr r4
  std r4, retaddress
  bl seterrno
  nop
  ld r4, retaddress
  li r3, -1
  mtlr r4
.LDone:
end;
