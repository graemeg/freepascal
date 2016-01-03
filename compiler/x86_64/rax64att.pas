{
    Copyright (c) 1998-2002 by Carl Eric Codere and Peter Vreman

    Does the parsing for the i386 GNU AS styled inline assembler.

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
Unit rax64att;

{$i fpcdefs.inc}

  interface

    uses
      raatt,rax86att,aasmtai;

    type
      tx8664attreader = class(tx86attreader)
        actsehdirective: TAsmSehDirective;
        procedure handleopcode;override;
        function is_targetdirective(const s:string):boolean;override;
        procedure handletargetdirective;override;
      end;


  implementation

    uses
      cutils,globtype,rabase,systems,rax86,aasmcpu,cgbase,procinfo,symconst,verbose;

    procedure tx8664attreader.handleopcode;
      var
        instr : Tx86Instruction;
      begin
        instr:=Tx86attInstruction.Create(Tx86Operand);
        BuildOpcode(instr);
        instr.AddReferenceSizes;
        instr.SetInstructionOpsize;
        instr.CheckOperandSizes;
<<<<<<< HEAD
<<<<<<< HEAD
        instr.FixupOpcode;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        instr.ConcatInstruction(curlist);
        instr.Free;
      end;

    const
      { x86_64 subset of SEH directives. .seh_proc and .seh_endproc excluded
        because they are generated automatically when needed. }
      recognized_directives: set of TAsmSehDirective=[
        ash_endprologue,ash_handler,ash_handlerdata,
        ash_setframe,ash_stackalloc,ash_pushreg,
        ash_savereg,ash_savexmm,ash_pushframe
      ];

      { max offset and bitmask for .seh_savereg and .seh_setframe }
      maxoffset: array[boolean] of aint=(high(dword), 240);
      modulo: array[boolean] of integer=(7, 15);

    function tx8664attreader.is_targetdirective(const s:string):boolean;
      var
        i: TAsmSehDirective;
      begin
        result:=false;
        if target_info.system<>system_x86_64_win64 then
          exit;

        for i:=low(TAsmSehDirective) to high(TAsmSehDirective) do
          begin
            if not (i in recognized_directives) then
              continue;
            if s=sehdirectivestr[i] then
              begin
                actsehdirective:=i;
                result:=true;
                break;
              end;
          end;
        { allow SEH directives only in pure assember routines }
        if result and not (po_assembler in current_procinfo.procdef.procoptions) then
          begin
            Message(asmr_e_seh_in_pure_asm_only);
            result:=false;
          end;
      end;

    procedure tx8664attreader.handletargetdirective;
      var
        hreg: TRegister;
        hnum: aint;
        flags: integer;
        ai: tai_seh_directive;
        hs: string;
        err: boolean;
      begin
        if actasmtoken<>AS_TARGET_DIRECTIVE then
          InternalError(2011100201);
        Consume(AS_TARGET_DIRECTIVE);
        Include(current_procinfo.flags,pi_has_unwind_info);

        case actsehdirective of
          { TODO: .seh_pushframe is supposed to have a boolean parameter,
                  but GAS 2.21 does not support it. }
          ash_endprologue,
          ash_pushframe,
          ash_handlerdata:
            curlist.concat(cai_seh_directive.create(actsehdirective));

          ash_handler:
            begin
              hs:=actasmpattern;
              Consume(AS_ID);
              flags:=0;
              err:=false;
              while actasmtoken=AS_COMMA do
                begin
                  Consume(AS_COMMA);
                  if actasmtoken=AS_AT then
                    begin
                      Consume(AS_AT);
                      if actasmtoken=AS_ID then
                        begin
                          uppervar(actasmpattern);
                          if actasmpattern='EXCEPT' then
                            flags:=flags or 1
                          else if actasmpattern='UNWIND' then
                            flags:=flags or 2
                          else
                            err:=true;
                          Consume(AS_ID);
                        end
                      else
                        err:=true;
                    end
                  else
                    err:=true;
                  if err then
                    begin
                      Message(asmr_e_syntax_error);
                      RecoverConsume(false);
                      exit;
                    end;
                end;

              ai:=cai_seh_directive.create_name(ash_handler,hs);
              ai.data.flags:=flags;
              curlist.concat(ai);
            end;
          ash_stackalloc:
            begin
              hnum:=BuildConstExpression(false,false);
              if (hnum<0) or (hnum>high(dword)) or ((hnum and 7)<>0) then
                Message1(asmr_e_bad_seh_directive_offset,sehdirectivestr[ash_stackalloc])
              else
                curlist.concat(cai_seh_directive.create_offset(ash_stackalloc,hnum));
            end;
          ash_pushreg:
            begin
              hreg:=actasmregister;
              Consume(AS_REGISTER);
              if (getregtype(hreg)<>R_INTREGISTER) or (getsubreg(hreg)<>R_SUBQ) then
                Message1(asmr_e_bad_seh_directive_register,sehdirectivestr[ash_pushreg])
              else
                curlist.concat(cai_seh_directive.create_reg(ash_pushreg,hreg));
            end;
          ash_setframe,
          ash_savereg:
            begin
              hreg:=actasmregister;
              Consume(AS_REGISTER);
              if (getregtype(hreg)<>R_INTREGISTER) or (getsubreg(hreg)<>R_SUBQ) then
                Message1(asmr_e_bad_seh_directive_register,sehdirectivestr[actsehdirective]);
              Consume(AS_COMMA);
              hnum:=BuildConstExpression(false,false);
              if (hnum<0) or (hnum>maxoffset[actsehdirective=ash_setframe]) or
                ((hnum mod modulo[actsehdirective=ash_setframe])<>0) then
                Message1(asmr_e_bad_seh_directive_offset,sehdirectivestr[actsehdirective])
              else
                curlist.concat(cai_seh_directive.create_reg_offset(actsehdirective,hreg,hnum));
            end;
          ash_savexmm:
            begin
              hreg:=actasmregister;
              Consume(AS_REGISTER);
              if (getregtype(hreg)<>R_MMREGISTER) then
                Message1(asmr_e_bad_seh_directive_register,sehdirectivestr[ash_savexmm]);
              Consume(AS_COMMA);
              hnum:=BuildConstExpression(false,false);
              if (hnum<0) or (hnum>high(dword)) or ((hnum and 15)<>0) then
                Message1(asmr_e_bad_seh_directive_offset,sehdirectivestr[ash_savexmm])
              else
                curlist.concat(cai_seh_directive.create_reg_offset(actsehdirective,hreg,hnum));
            end;

        else
          InternalError(2011100202);
        end;
        if actasmtoken<>AS_SEPARATOR then
          Consume(AS_SEPARATOR);
      end;


const
  asmmode_x86_64_gas_info : tasmmodeinfo =
          (
            id    : asmmode_x86_64_gas;
            idtxt : 'GAS';
            casmreader : tx8664attreader;
          );

  { Added to be compatible with i386 }
  asmmode_x86_64_att_info : tasmmodeinfo =
          (
            id    : asmmode_x86_64_att;
            idtxt : 'ATT';
            casmreader : tx8664attreader;
          );

  asmmode_x86_64_standard_info : tasmmodeinfo =
          (
            id    : asmmode_standard;
            idtxt : 'STANDARD';
            casmreader : tx8664attreader;
          );

initialization
  RegisterAsmMode(asmmode_x86_64_gas_info);
  RegisterAsmMode(asmmode_x86_64_att_info);
  RegisterAsmMode(asmmode_x86_64_standard_info);
end.
