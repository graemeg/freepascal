{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2011 by Jonas Maebe
    member of the Free Pascal development team.

    This file implements some Java-specific system unit routines

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************
}

type
  TOSTimestamp = Longint;

  FpcRunTimeError = class(JLException)
    errornr: longint;
    constructor create(l: longint);
  end;
