{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 Marco van de Voort
    member of the Free Pascal development team.

      syscall headers for *Beos

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

Type
  TSysResult = longint; // all platforms, cint=32-bit.
                        // On platforms with off_t =64-bit, people should
                        // use int64, and typecast all calls that don't
                        // return off_t to cint.

// I don't think this is going to work on several platforms
// 64-bit machines don't have only 64-bit params.

  TSysParam  = longint;
  
type
     SysCallArgs = packed record
       param: array[1..8] of longint; // cint but not defined in unix.pp
     End;

{$IFDEF FPC_USE_LIBC}
//var
//  Errno : cint;
  
{$else}
//var
//  Errno : cint;

{$ENDIF}
procedure sys_call; external name 'sys_call'; // BeOS
//begin
//end;

  
//function Do_SysCall( callnr : longint; var regs : SysCallArgs ): longint; external name 'FPC_SYSCALL';//forward;
//Function SysCall( callnr:longint;var args : SysCallArgs ):longint; external name 'sys_call';//forward;
