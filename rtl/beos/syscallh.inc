{
    Copyright (c) 2002 by Marco van de Voort

    Header for syscall in system unit for i386 *BSD.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
<<<<<<< HEAD
<<<<<<< HEAD
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301, USA.
=======
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
>>>>>>> graemeg/fixes_2_2
=======
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
>>>>>>> origin/fixes_2_2

 ****************************************************************************

}

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
