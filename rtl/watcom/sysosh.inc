<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    This file implements all the base types and limits required
    for a minimal POSIX compliant subset required to port the compiler
    to a new OS.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{Platform specific information}
type
  THandle = Longint;
  TThreadID = THandle;

   PRTLCriticalSection = ^TRTLCriticalSection;
   TRTLCriticalSection = record
     Locked: boolean
   end;



<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001 by Free Pascal development team

    This file implements all the base types and limits required
    for a minimal POSIX compliant subset required to port the compiler
    to a new OS.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{Platform specific information}
type
  THandle = Longint;
  TThreadID = THandle;

   PRTLCriticalSection = ^TRTLCriticalSection;
   TRTLCriticalSection = record
     Locked: boolean
   end;



<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
