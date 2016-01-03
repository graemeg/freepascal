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

{
        Keyboard device command definitions
}

unit keyboard;

INTERFACE

uses exec;


Const
    KBD_READEVENT               = CMD_NONSTD + 0;
    KBD_READMATRIX              = CMD_NONSTD + 1;
    KBD_ADDRESETHANDLER         = CMD_NONSTD + 2;
    KBD_REMRESETHANDLER         = CMD_NONSTD + 3;
    KBD_RESETHANDLERDONE        = CMD_NONSTD + 4;

IMPLEMENTATION

end.
