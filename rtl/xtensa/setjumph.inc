{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by the Free Pascal development team.

    SetJmp/Longjmp declarations

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

type
   jmp_buf = packed record
   end;
   pjmp_buf = ^jmp_buf;

function setjmp(var S : jmp_buf) : longint;[external name 'FPC_SETJMP'];
procedure longjmp(var S : jmp_buf;value : longint);[external name 'FPC_LONGJMP'];


