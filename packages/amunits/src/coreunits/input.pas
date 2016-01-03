{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit input;

INTERFACE

uses exec;


const

    IND_ADDHANDLER      = CMD_NONSTD + 0;
    IND_REMHANDLER      = CMD_NONSTD + 1;
    IND_WRITEEVENT      = CMD_NONSTD + 2;
    IND_SETTHRESH       = CMD_NONSTD + 3;
    IND_SETPERIOD       = CMD_NONSTD + 4;
    IND_SETMPORT        = CMD_NONSTD + 5;
    IND_SETMTYPE        = CMD_NONSTD + 6;
    IND_SETMTRIG        = CMD_NONSTD + 7;

VAR InputBase : pDevice;

FUNCTION PeekQualifier : WORD;

IMPLEMENTATION

FUNCTION PeekQualifier : WORD;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L InputBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

END. (* UNIT INPUT *)



