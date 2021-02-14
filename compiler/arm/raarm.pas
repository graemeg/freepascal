{
    Copyright (c) 1998-2003 by Carl Eric Codere and Peter Vreman

    Handles the common arm assembler reader routines

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
unit raarm;

{$i fpcdefs.inc}

  interface

    uses
      cpubase,
      aasmtai,aasmdata,
      rautils;

    type
      TARMOperand=class(TOperand)
      end;

      TARMInstruction=class(TInstruction)
        oppostfix : toppostfix;
        wideformat : boolean; // For wide(32bit) instructions of the thumb-2 instruction set
        function ConcatInstruction(p:TAsmList) : tai;override;
      end;

  implementation

    uses
      globals,
      cpuinfo,
      aasmcpu;

    function TARMInstruction.ConcatInstruction(p:TAsmList) : tai;
      begin
        result:=inherited ConcatInstruction(p);
        (result as taicpu).oppostfix:=oppostfix;
        if wideformat then
          include((result as taicpu).flags,cf_wideformat)
        else
          exclude((result as taicpu).flags,cf_wideformat);
        { GNU As assumes implicit S postfix for some instructions in thumb mode }
        if (current_settings.instructionset=is_thumb) and (((result as taicpu).oppostfix=PF_None) and ((opcode=A_MOV) and ((result as taicpu).oper[1]^.typ=top_const)) or (opcode=A_MVN)) then
          (result as taicpu).oppostfix:=PF_S
      end;


end.
