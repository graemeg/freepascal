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
{$ifdef cpu64}
      __mt_word: array[0..7] of int64;
{$else}
      __mt_word: array[0..12] of longint;
{$endif}
  end;


