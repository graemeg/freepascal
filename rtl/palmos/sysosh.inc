{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2017 by Free Pascal development team

    OS specific headers

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{Platform specific information}
type
{$ifdef CPU64}
  THandle = Int64;
{$else CPU64}
  THandle = Longint;
{$endif CPU64}
  TThreadID = THandle;
  TOSTimestamp = Longint;

  PRTLCriticalSection = ^TRTLCriticalSection;
  TRTLCriticalSection = record
   Locked: boolean
  end;
