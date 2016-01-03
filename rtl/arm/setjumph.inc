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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$if defined(FPUVFPV2) or defined(FPUVFPV3) or defined(FPUVFPV3_D16)}
=======
{$if defined(FPUVFPV2) or defined(FPUVFPV3)}
>>>>>>> graemeg/cpstrnew
=======
{$if defined(FPUVFPV2) or defined(FPUVFPV3)}
>>>>>>> graemeg/cpstrnew
=======
{$if defined(FPUVFPV2) or defined(FPUVFPV3)}
>>>>>>> graemeg/cpstrnew
=======
{$if defined(FPUVFPV2) or defined(FPUVFPV3)}
>>>>>>> origin/cpstrnew
      d8,d9,d10,d11,d12,d13,d14,d15: double;
{$endif}
      v1,v2,v3,v4,v5,v6,sl,fp,sp,pc : dword;
   end;
   pjmp_buf = ^jmp_buf;

function setjmp(var S : jmp_buf) : longint;[external name 'FPC_SETJMP'];
procedure longjmp(var S : jmp_buf;value : longint);[external name 'FPC_LONGJMP'];


