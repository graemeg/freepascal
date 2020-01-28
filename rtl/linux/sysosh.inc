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
  
  { pthread_t is defined as an "unsigned long" }
  TThreadID = PtrUInt;

  TOSTimestamp = Int64;

  PRTLCriticalSection = ^TRTLCriticalSection;
{$macro on}
{$define MUTEXTYPENAME := TRTLCriticalSection}
{$i pmutext.inc}
{$undef MUTEXTYPENAME}
{$macro off}

  {$define HAS_ENTRYINFORMATION_OS}
  TEntryInformationOS = record
    argc: longint;
    argv: ppchar;
    envp: ppchar;
    stkptr: pointer;
    stklen: sizeuint;
    haltproc: procedure(e:longint);cdecl;
  end;


