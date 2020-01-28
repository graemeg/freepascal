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
  { fd are int in C also for 64bit targets (x86_64) }
  THandle = Longint;
  TThreadID = THandle;
  TOSTimestamp = Int64;

  { pthread_mutex_t }
  PRTLCriticalSection = ^TRTLCriticalSection;
  TRTLCriticalSection = record
    __m_reserved: longint;
    __m_count: longint;
    __m_owner: pointer;
    __m_kind:  longint;
    __m_lock:  record
       __status: sizeint;
      __spinlock: longint;
    end;
  end;
