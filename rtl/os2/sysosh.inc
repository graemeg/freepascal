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
  TThreadID = cardinal;
  TOSTimestamp = Longint;

  { the fields of this record are os dependent  }
  { and they shouldn't be used in a program     }
  { only the type TCriticalSection is important }
  PRTLCriticalSection = ^TRTLCriticalSection;
  TRTLCriticalSection = packed record
    DebugInfo : pointer;
    LockCount : longint;
    RecursionCount : longint;
    OwningThread : DWord;
    LockSemaphore : DWord;
    Reserved : DWord;
  end;



