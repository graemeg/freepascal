{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1998-2000 Florian P Klaempfl
    Copyright (c) 2002 Marco van de Voort
    member of the Free Pascal development team.

    This include implements the actual system call for the
    intel BeOS 80x86 platform.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

// Under BeOS, we use stdcall for this line because the default calling convention in 1.9 
// is register instead of stdcall. But assembler is already written, so i used the stdcall
// calling convention !
function Do_SysCall( callnr : longint; var regs : SysCallArgs ): longint; stdcall; assembler; [public, alias : 'FPC_SYSCALL'];
{
  This routine sets up the parameters on the stack, all the parameters 
  are in reverse order on the stack (like C parameter passing).
}
asm
  { load the parameters... }
  movl  regs,%eax
  movl  24(%eax),%ebx
  pushl %ebx
  movl  20(%eax),%ebx
  pushl %ebx 
  movl  16(%eax),%ebx
  pushl %ebx
  movl  12(%eax),%ebx
  pushl %ebx
  movl  8(%eax),%ebx
  pushl %ebx
  movl  4(%eax),%ebx
  pushl %ebx
  movl  0(%eax),%ebx
  pushl %ebx
  { set the call number }
  movl  callnr,%eax
  call  sys_call
  addl  $28,%esp
end;

// Under BeOS, we use stdcall for this line because the default calling convention in 1.9 
// is register instead of stdcall. But assembler is already written, so i used the stdcall
// calling convention ! Maybe don't needed here. But to be sure...
Function SysCall( callnr:longint;var args : SysCallArgs ):longint; stdcall;
{
  This function serves as an interface to do_SysCall.
  If the SysCall returned a negative number, it returns -1, and puts the
  SysCall result in errno. Otherwise, it returns the SysCall return value
}
var
 funcresult : longint;
begin
  funcresult := do_SysCall(callnr, args);
  if funcresult < 0 then
   begin
     errno := funcresult;
     SysCall := - 1;
   end
  else
   begin
     SysCall := funcresult;
     errno := 0;
   end;
end;
