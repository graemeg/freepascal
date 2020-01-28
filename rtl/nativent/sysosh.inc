{
    Basic Native NT stuff

    This file is part of the Free Pascal run time library.
    Copyright (c) 2009 by Sven Barth

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ Platform specific information }
type
  THandle = PtrUInt;
  TThreadID = THandle;
  TOSTimestamp = Longint;

  { the fields of this record are os dependent  }
  { and they shouldn't be used in a program     }
  { only the type TCriticalSection is important }
  PRTLCriticalSection = ^TRTLCriticalSection;
  TRTLCriticalSection = packed record
    DebugInfo : pointer;
    LockCount : longint;
    RecursionCount : longint;
    OwningThread : THandle;
    LockSemaphore : THandle;
    SpinCount : LongWord; // PtrUInt?
  end;

var
  { the following variables are only set if apptype=native and the RTL is
    compiled with -dKMODE (device driver)
    they are exported with their real types in unit DDK }
  // real type: PNtUnicodeString; only valid during PascalMain
  SysRegistryPath: Pointer = Nil;
  // real type: PDriverObject; only valid during PascalMain
  SysDriverObject: Pointer = Nil;

type
  TTagString = String[4];

{$ifdef KMODE}
const
  DefaultPoolTag = 'fpc';

var
  // tells the heap whether to use paged memory or not
  HeapUsePagedPool: Boolean = True;
  // the tag is a four byte string to identify the memory allocated by our
  // driver, which must not be empty
  HeapPoolTag: TTagString = DefaultPoolTag;
{$endif KMODE}
