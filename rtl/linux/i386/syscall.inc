{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt,
    member of the Free Pascal development team.

    The syscalls for the new RTL, moved to platform dependant dir.
    Old linux calling convention is still kept.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$ASMMODE ATT}

{*************************SYSENTER CODE********************************}

{ included by system.pp in linux rtl }
const
  AT_NULL         = 0;
{* Pointer to the global system page used for system calls and other
   nice things.  *}
  AT_SYSINFO      = 32;
  AT_SYSINFO_EHDR = 33;

type
  TAuxiliaryValue = cuInt32;
<<<<<<< HEAD

  TInternalUnion = record
    a_val: cuint32;           //* Integer value */
      {* We use to have pointer elements added here.  We cannot do that,
         though, since it does not work when using 32-bit definitions
         on 64-bit platforms and vice versa.  *}
  end;

  Elf32_auxv_t = record
    a_type: cuint32;              //* Entry type */
    a_un: TInternalUnion;
  end;
  TElf32AuxiliaryVector = Elf32_auxv_t;
  PElf32AuxiliaryVector = ^TElf32AuxiliaryVector;
<<<<<<< HEAD

var
  psysinfo: LongWord = 0;

procedure InitSyscallIntf;
var
  ep: PPChar;
  auxv: PElf32AuxiliaryVector;
begin

  psysinfo := 0;
  ep := envp;
  while ep^ <> nil do
    Inc(ep);

  Inc(ep);

  auxv := PElf32AuxiliaryVector(ep);

  while auxv^.a_type <> AT_NULL do
    begin
      if auxv^.a_type = AT_SYSINFO then begin
        psysinfo := auxv^.a_un.a_val;
        if psysinfo <> 0 then
          sysenter_supported := 1; // descision factor in asm syscall routines
        Break;
      end;
      Inc(auxv);
    end;
end;

{***********************SYSENTER CODE END******************************}

Procedure fpc_geteipasebx;[external name 'fpc_geteipasebx'];

{$ifdef FPC_PROFILE}
procedure mcount; external name 'mcount';
{$endif FPC_PROFILE}

function FpSysCall(sysnr:TSysParam):TSysResult; assembler; register;  [public,alias:'FPC_SYSCALL0'];
{ Var sysnr located in register eax }
asm
  {$ifdef FPC_PROFILE}
        push  %eax
        call  mcount
        pop   %eax
 {$endif FPC_PROFILE}
        push  %ebx
        push  %ecx
  {$ifdef FPC_PIC}
        call  fpc_geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
        movl  sysenter_supported@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        cmp   $0, %ecx
  {$else FPC_PIC}
        cmp   $0, sysenter_supported
  {$endif FPC_PIC}
        jne   .LSysEnter
        int   $0x80
        jmp   .LTail
  .LSysEnter:
  {$ifdef FPC_PIC}
        movl  psysinfo@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
	call  *%ecx
  {$else FPC_PIC}
        call  psysinfo
  {$endif FPC_PIC}
  .LTail:
	pop   %ecx
	pop   %ebx
        cmpl  $-4095,%eax
        jb    .LSyscOK
        negl  %eax
        call  seterrno
        movl  $-1,%eax
  .LSyscOK:
end;


function FpSysCall(sysnr,param1 : TSysParam):TSysResult; assembler; register; [public,alias:'FPC_SYSCALL1'];
{ Var sysnr located in register eax
  Var param1 located in register edx }
asm
  {$ifdef FPC_PROFILE}
        push  %eax
        push  %edx
        call  mcount
        pop   %edx
        pop   %eax
  {$endif FPC_PROFILE}
        push  %ebx
        push  %ecx
  {$ifdef FPC_PIC}
        call  fpc_geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
        movl  sysenter_supported@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        cmp   $0, %ecx
  {$else FPC_PIC}
        cmp   $0, sysenter_supported
  {$endif FPC_PIC}

        jne   .LSysEnter
        movl  %edx,%ebx		// param1
        int   $0x80
        jmp   .LTail
  .LSysEnter:
  {$ifdef FPC_PIC}
        movl  psysinfo@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        movl  %edx,%ebx		// param1
	call  *%ecx
  {$else FPC_PIC}
        movl  %edx,%ebx		// param1
        call  psysinfo
  {$endif FPC_PIC}
  .LTail:
	pop   %ecx
	pop   %ebx
=======

var
  psysinfo: LongWord = 0;

=======

  TInternalUnion = record
    a_val: cuint32;           //* Integer value */
      {* We use to have pointer elements added here.  We cannot do that,
         though, since it does not work when using 32-bit definitions
         on 64-bit platforms and vice versa.  *}
  end;

  Elf32_auxv_t = record
    a_type: cuint32;              //* Entry type */
    a_un: TInternalUnion;
  end;
  TElf32AuxiliaryVector = Elf32_auxv_t;
  PElf32AuxiliaryVector = ^TElf32AuxiliaryVector;

var
  psysinfo: LongWord = 0;

>>>>>>> origin/fixes_2_2
procedure InitSyscallIntf;
var
  ep: PPChar;
  auxv: PElf32AuxiliaryVector;
begin

  psysinfo := 0;
  ep := envp;
  while ep^ <> nil do
    Inc(ep);

  Inc(ep);

  auxv := PElf32AuxiliaryVector(ep);

  repeat
    if auxv^.a_type = AT_SYSINFO then begin
      psysinfo := auxv^.a_un.a_val;
      if psysinfo <> 0 then
        sysenter_supported := 1; // descision factor in asm syscall routines
      Break;
    end;
    Inc(auxv);
  until auxv^.a_type = AT_NULL;
end;

{***********************SYSENTER CODE END******************************}
<<<<<<< HEAD

Procedure fpc_geteipasebx;[external name 'fpc_geteipasebx'];

=======

Procedure fpc_geteipasebx;[external name 'fpc_geteipasebx'];

>>>>>>> origin/fixes_2_2
function FpSysCall(sysnr:TSysParam):TSysResult; assembler; register; [public,alias:'FPC_SYSCALL0'];
{ Var sysnr located in register eax }
asm
//      movl  sysnr,%eax
        cmp  $0, sysenter_supported
        jne  .LSysEnter
        int $0x80
        jmp   .LTail
  .LSysEnter:
        call psysinfo
  .LTail:
        cmpl  $-4095,%eax
        jb    .LSyscOK
        negl  %eax
        call  seterrno
        movl  $-1,%eax
  .LSyscOK:
end;

function FpSysCall(sysnr,param1 : TSysParam):TSysResult; assembler; register; [public,alias:'FPC_SYSCALL1'];
{ Var sysnr located in register eax
  Var param1 located in register edx }
asm
        movl  %ebx,%ecx
//      movl  sysnr,%eax
        movl  %edx,%ebx
        cmp   $0, sysenter_supported
        jne   .LSysEnter
        int   $0x80
        jmp   .LTail
  .LSysEnter:
        call psysinfo
  .LTail:
        movl  %ecx,%ebx
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        cmpl  $-4095,%eax
        jb    .LSyscOK
        negl  %eax
        call  seterrno
        movl  $-1,%eax
  .LSyscOK:
end;

function FpSysCall(sysnr,param1,param2 : TSysParam):TSysResult; assembler; register; [public,alias:'FPC_SYSCALL2'];
{ Var sysnr located in register eax
  Var param1 located in register edx
  Var param2 located in register ecx }
asm
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef FPC_PROFILE}
        push  %eax
        push  %edx
        push  %ecx
        call  mcount
        pop   %ecx
        pop   %edx
        pop   %eax
  {$endif FPC_PROFILE}
        push  %ebx
        push  %edx
        push  %ecx
  {$ifdef FPC_PIC}
        call  fpc_geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
        movl  sysenter_supported@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        cmp   $0, %ecx
  {$else FPC_PIC}
        cmp   $0, sysenter_supported
  {$endif FPC_PIC}
        jne   .LSysEnter
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        int   $0x80
        jmp   .LTail
  .LSysEnter:
  {$ifdef FPC_PIC}
        movl  psysinfo@GOT(%ebx),%ecx
        movl  %edx,%ebx		// param1
        movl  (%ecx),%edx
	pop   %ecx		// param2
	call  *%edx
  {$else FPC_PIC}
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        call  psysinfo
  {$endif FPC_PIC}
  .LTail:
	pop   %edx
	pop   %ebx
=======
=======
>>>>>>> origin/fixes_2_2
        push  %ebx
//      movl  sysnr,%eax
        movl  %edx,%ebx
//      movl  param2,%ecx
        cmp   $0, sysenter_supported
        jne   .LSysEnter
        int   $0x80
        jmp   .LTail
  .LSysEnter:
        call psysinfo
  .LTail:
        pop   %ebx
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        cmpl  $-4095,%eax
        jb    .LSyscOK
        negl  %eax
        call  seterrno
        movl  $-1,%eax
  .LSyscOK:
end;

function FpSysCall(sysnr,param1,param2,param3:TSysParam):TSysResult; assembler; register; [public,alias:'FPC_SYSCALL3'];
{ Var sysnr located in register eax
  Var param1 located in register edx
  Var param2 located in register ecx
  Var param3 located at ebp+20 }
<<<<<<< HEAD
<<<<<<< HEAD
var
  _psysinfo: LongWord;
asm
  {$ifdef FPC_PROFILE}
        push  %eax
        push  %edx
        push  %ecx
        call  mcount
        pop   %ecx
        pop   %edx
        pop   %eax
  {$endif FPC_PROFILE}
        push  %ebx
        push  %edx
        push  %ecx
  {$ifdef FPC_PIC}
        call  fpc_geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
        movl  sysenter_supported@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        cmp   $0, %ecx
  {$else FPC_PIC}
        cmp   $0, sysenter_supported
  {$endif FPC_PIC}
        jne   .LSysEnter
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        int   $0x80
        jmp   .LTail
  .LSysEnter:
  {$ifdef FPC_PIC}
        movl  psysinfo@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        movl  %ecx,_psysinfo
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        call  _psysinfo 
  {$else FPC_PIC}
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        call psysinfo
  {$endif FPC_PIC}
  .LTail:
	pop   %edx
	pop   %ebx
=======
=======
>>>>>>> origin/fixes_2_2
asm
        push  %ebx
//      movl  sysnr,%eax
        movl  %edx,%ebx
//      movl  param2,%ecx
        movl  param3,%edx
        cmp   $0, sysenter_supported
        jne   .LSysEnter
        int   $0x80
        jmp   .LTail
  .LSysEnter:
        call psysinfo
  .LTail:
        pop   %ebx
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        cmpl  $-4095,%eax
        jb    .LSyscOK
        negl  %eax
        call  seterrno
        movl  $-1,%eax
  .LSyscOK:
end;

function FpSysCall(sysnr,param1,param2,param3,param4:TSysParam):TSysResult; assembler; register; [public,alias:'FPC_SYSCALL4'];
{ Var sysnr located in register eax
  Var param1 located in register edx
  Var param2 located in register ecx
  Var param3 located at ebp+20
  Var param4 located at ebp+16 }
<<<<<<< HEAD
<<<<<<< HEAD
var
  _psysinfo: LongWord;
asm
  {$ifdef FPC_PROFILE}
        push  %eax
        push  %edx
        push  %ecx
        call  mcount
        pop   %ecx
        pop   %edx
        pop   %eax
  {$endif FPC_PROFILE}
        push  %ebx
        push  %esi
        push  %ecx
  {$ifdef FPC_PIC}
        call  fpc_geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
        movl  sysenter_supported@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        cmp   $0, %ecx
  {$else FPC_PIC}
        cmp   $0, sysenter_supported
  {$endif FPC_PIC}
        jne  .LSysEnter
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        movl  param4,%esi	// param4
        int   $0x80
        jmp   .LTail
  .LSysEnter:
  {$ifdef FPC_PIC}
        movl  psysinfo@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        movl  %ecx,_psysinfo
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        movl  param4,%esi	// param4
        call  _psysinfo 
  {$else FPC_PIC}
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        movl  param4,%esi	// param4
        call  psysinfo
  {$endif FPC_PIC}
  .LTail:
  	pop   %esi
	pop   %ebx
=======
=======
>>>>>>> origin/fixes_2_2
asm
        push  %ebx
        push  %esi
//      movl  sysnr,%eax
        movl  %edx,%ebx
//      movl  param2,%ecx
        movl  param3,%edx
        movl  param4,%esi
        cmp   $0, sysenter_supported
        jne   .LSysEnter
        int   $0x80
        jmp  .LTail
  .LSysEnter:
        call psysinfo
  .LTail:
        pop   %esi
        pop   %ebx
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        cmpl  $-4095,%eax
        jb    .LSyscOK
        negl  %eax
        call  seterrno
        movl  $-1,%eax
  .LSyscOK:
end;

function FpSysCall(sysnr,param1,param2,param3,param4,param5 : TSysParam):TSysResult; assembler; register; [public,alias:'FPC_SYSCALL5'];
{ Var sysnr located in register eax
  Var param1 located in register edx
  Var param2 located in register ecx
  Var param3 located at ebp+20
  Var param4 located at ebp+16
  Var param5 located at ebp+12 }
<<<<<<< HEAD
<<<<<<< HEAD
var
  _psysinfo: LongWord;
asm
  {$ifdef FPC_PROFILE}
        push  %eax
        push  %edx
        push  %ecx
        call  mcount
        pop   %ecx
        pop   %edx
        pop   %eax
  {$endif FPC_PROFILE}
        push  %ebx
        push  %edx
        push  %esi
        push  %edi
        push  %ecx
  {$ifdef FPC_PIC}
        call  fpc_geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
        movl  sysenter_supported@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        cmp   $0, %ecx
  {$else FPC_PIC}
        cmp   $0, sysenter_supported
  {$endif FPC_PIC}
        jne   .LSysEnter
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        movl  param4,%esi	// param4
	movl  param5,%edi	// param5
        int $0x80
        jmp   .LTail
  .LSysEnter:
  {$ifdef FPC_PIC}
        movl  psysinfo@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        movl  %ecx,_psysinfo
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        movl  param4,%esi	// param4
	movl  param5,%edi	// param5
        call  _psysinfo 
  {$else FPC_PIC}
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        movl  param4,%esi	// param4
	movl  param5,%edi	// param5
        call psysinfo
  {$endif FPC_PIC}
  .LTail:
  	pop   %edi
  	pop   %esi
	pop   %edx
	pop   %ebx
=======
=======
>>>>>>> origin/fixes_2_2
asm
        push  %ebx
        push  %esi
        push  %edi
//      movl  sysnr,%eax
        movl  %edx,%ebx
//      movl  param2,%ecx
        movl  param3,%edx
        movl  param4,%esi
        movl  param5,%edi
        cmp   $0, sysenter_supported
        jne   .LSysEnter
        int   $0x80
        jmp   .LTail
  .LSysEnter:
        call psysinfo
  .LTail:
        pop   %edi
        pop   %esi
        pop   %ebx
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        cmpl  $-4095,%eax
        jb    .LSyscOK
        negl  %eax
        call  seterrno
        movl  $-1,%eax
  .LSyscOK:
end;

function FpSysCall(sysnr,param1,param2,param3,param4,param5,param6: TSysParam):TSysResult; assembler; register; [public,alias:'FPC_SYSCALL6'];
{ Var sysnr located in register eax
  Var param1 located in register edx
  Var param2 located in register ecx
  Var param3 located at ebp+20
  Var param4 located at ebp+16
  Var param5 located at ebp+12
  Var param6 located at ebp+8 }
<<<<<<< HEAD
<<<<<<< HEAD
var
  _psysinfo: LongWord;
asm
  {$ifdef FPC_PROFILE}
        push  %eax
        push  %edx
        push  %ecx
        call  mcount
        pop   %ecx
        pop   %edx
        pop   %eax
  {$endif FPC_PROFILE}
        push  %ebx
	push  %edx
        push  %esi
        push  %edi
	push  %ebp
        push  %ecx
  {$ifdef FPC_PIC}
        call  fpc_geteipasebx
        addl  $_GLOBAL_OFFSET_TABLE_,%ebx
        movl  sysenter_supported@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        cmp   $0, %ecx
  {$else FPC_PIC}
        cmp   $0, sysenter_supported
  {$endif FPC_PIC}
        jne   .LSysEnter
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        movl  param4,%esi	// param4
	movl  param5,%edi	// param5
	movl  param6,%ebp	// param6
        int   $0x80
        jmp   .LTail
  .LSysEnter:
  {$ifdef FPC_PIC}
        movl  psysinfo@GOT(%ebx),%ecx
        movl  (%ecx),%ecx
        movl  %ecx,_psysinfo
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        movl  param4,%esi	// param4
	movl  param5,%edi	// param5
	movl  param6,%ebp	// param6
        call  _psysinfo 
  {$else FPC_PIC}
        movl  %edx,%ebx		// param1
	pop   %ecx		// param2
        movl  param3,%edx	// param3
        movl  param4,%esi	// param4
	movl  param5,%edi	// param5
	movl  param6,%ebp	// param6
        call psysinfo
  {$endif FPC_PIC}
  .LTail:
	pop   %ebp
  	pop   %edi
  	pop   %esi
	pop   %edx
	pop   %ebx
=======
=======
>>>>>>> origin/fixes_2_2
asm
        push  %ebx
        push  %esi
        push  %edi
        push  %ebp
//      movl  sysnr,%eax
        movl  %edx,%ebx
//      movl  param2,%ecx
        movl  param3,%edx
        movl  param4,%esi
        movl  param5,%edi
        movl  param6,%ebp
        cmp   $0, sysenter_supported
        jne   .LSysEnter
        int   $0x80
        jmp   .LTail
  .LSysEnter:
        call psysinfo
  .LTail:
        pop   %ebp
        pop   %edi
        pop   %esi
        pop   %ebx
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        cmpl  $-4095,%eax
        jb    .LSyscOK
        negl  %eax
        call  seterrno
        movl  $-1,%eax
  .LSyscOK:
end;

{No debugging for syslinux include !}
{$IFDEF SYS_LINUX}
  {$UNDEF SYSCALL_DEBUG}
{$ENDIF SYS_LINUX}
