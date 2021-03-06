{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by the Free Pascal development team.

    SetJmp/Longjmp declarations

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

type
   jmp_buf = record
    ra,s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,sp: dword;
{$if defined(FPUFD) or defined(FPUD)}
    fcsr,dummy : dword;
    f8,f9,f18,f19,f20,f21,
    f22,f23,f24,f25,f26,f27: qword;
{$endif FPUFD or FPUD}
{$if defined(FPUF)}
    fcsr : dword;
    f8,f9,f18,f19,f20,f21,
    f22,f23,f24,f25,f26,f27: longword;
{$endif FPUF}
   end;
   pjmp_buf = ^jmp_buf;

function setjmp(var S : jmp_buf) : longint;[external name 'FPC_SETJMP'];
procedure longjmp(var S : jmp_buf;value : longint);[external name 'FPC_LONGJMP'];
