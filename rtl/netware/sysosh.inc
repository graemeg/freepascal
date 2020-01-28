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


type
  THandle = DWord;
  TThreadID = THandle;
  TOSTimestamp = Longint;
  
   { the fields of this record are os dependent  }
   { and they shouldn't be used in a program     }
   { only the type TCriticalSection is important }
   PRTLCriticalSection = ^TRTLCriticalSection;
   TRTLCriticalSection = packed record
      SemaHandle : LONGINT;
      SemaIsOpen : BOOLEAN;
   end;

{Delphi/Windows compatible priority constants, they are also defined for Unix and Win32}
const
   THREAD_PRIORITY_IDLE          = -15;
   THREAD_PRIORITY_LOWEST        = -2;
   THREAD_PRIORITY_BELOW_NORMAL  = -1;
   THREAD_PRIORITY_NORMAL        = 0;
   THREAD_PRIORITY_ABOVE_NORMAL  = 1;
   THREAD_PRIORITY_HIGHEST       = 2;
   THREAD_PRIORITY_TIME_CRITICAL = 15;



