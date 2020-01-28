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
  THandle = Longint;
  TThreadID = Cardinal;
  TOSTimestamp = Int64;
  
  PRTLCriticalSection = ^TRTLCriticalSection;
  TRTLCriticalSection = record
    __pthread_mutex_flags : record
         __pthread_mutex_flag1 : word;
         __pthread_mutex_flag2 : byte;
         __pthread_mutex_ceiling : byte;
         __pthread_mutex_type : word;
         __pthread_mutex_magic : word;
      end;
    __pthread_mutex_lock : record
        case longint of
           0 : ( __pthread_mutex_lock64 : record
                __pthread_mutex_pad : array[0..7] of byte;
             end );
           1 : ( __pthread_mutex_lock32 : record
                __pthread_ownerpid : dword;
                __pthread_lockword : dword;
             end );
           2 : ( __pthread_mutex_owner64 : qword );
        end;
    __pthread_mutex_data : qword;
  end;


