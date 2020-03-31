{
    Copyright (c) 1998-2003 by Carl Eric Codere and Peter Vreman

    Handles the common Xtensa assembler reader routines

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit raxtensa;

{$i fpcdefs.inc}

  interface

    uses
      cpubase,
      aasmtai,aasmdata,
      rautils;

    type
      TXtensaOperand=class(TOperand)
      end;

      TXtensaInstruction=class(TInstruction)
      end;

  implementation

    uses
      aasmcpu;

end.
