{
    This file is part of the Free Pascal test suite.
    Copyright (c) 1999-2002 by the Free Pascal development team.

    Used to avoid getting pop up windows
    under Windows Operation Systems for critical errors

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit win32err;

interface

implementation

{ mswindows is defined for win32 and win64 targets }
{$ifdef mswindows}
uses
  windows;
{$endif mswindows}

begin
{$ifdef mswindows}
  SetErrorMode(
    SEM_FAILCRITICALERRORS or
    SEM_NOGPFAULTERRORBOX or
    SEM_NOOPENFILEERRORBOX);
{$endif mswindows}
end.
