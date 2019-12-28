{
    Copyright (c) 2016 by Jeppe Johansen

    Does the parsing for the RiscV64 GNU AS styled inline assembler.

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
unit rarv64gas;

{$I fpcdefs.inc}

  interface

    uses
      raatt, rarvgas, rarv,
      cpubase;

    type
      trv64attreader = class(trvattreader)
        actmemoryordering: TMemoryOrdering;
        function is_register(const s: string): boolean; override;
        function is_asmopcode(const s: string):boolean;override;
        procedure handleopcode;override;
        procedure BuildReference(oper : trvoperand);
        procedure BuildOperand(oper : trvoperand);
        procedure BuildOpCode(instr : trvinstruction);
        procedure ReadAt(oper : trvoperand);
        procedure ReadSym(oper : trvoperand);
     end;

  implementation

    uses
      { helpers }
      cutils,
      { global }
      globtype,globals,verbose,
      systems,
      { aasm }
      aasmbase,aasmtai,aasmdata,aasmcpu,
      { symtable }
      symconst,symsym,symdef,
      { parser }
      procinfo,
      rabase,rautils,
      cgbase,cgobj,cgrv
      ;

    procedure trv64attreader.ReadSym(oper : trvoperand);
      var
         tempstr, mangledname : string;
         typesize,l,k : aint;
      begin
        tempstr:=actasmpattern;
        Consume(AS_ID);
        { typecasting? }
        if (actasmtoken=AS_LPAREN) and
           SearchType(tempstr,typesize) then
         begin
           oper.hastype:=true;
           Consume(AS_LPAREN);
           BuildOperand(oper);
           Consume(AS_RPAREN);
           if oper.opr.typ in [OPR_REFERENCE,OPR_LOCAL] then
             oper.SetSize(typesize,true);
         end
        else
         if not oper.SetupVar(tempstr,false) then
          Message1(sym_e_unknown_id,tempstr);
        { record.field ? }
        if actasmtoken=AS_DOT then
         begin
           BuildRecordOffsetSize(tempstr,l,k,mangledname,false);
           if (mangledname<>'') then
             Message(asmr_e_invalid_reference_syntax);
           inc(oper.opr.ref.offset,l);
         end;
      end;


    procedure trv64attreader.ReadAt(oper : trvoperand);
      begin
        { check for ...@ }
        if actasmtoken=AS_AT then
          begin
            if (oper.opr.ref.symbol=nil) and
               (oper.opr.ref.offset = 0) then
              Message(asmr_e_invalid_reference_syntax);
            Consume(AS_AT);
            if actasmtoken=AS_ID then
              begin
                {if upper(actasmpattern)='L' then
                  oper.opr.ref.refaddr:=addr_low
                else if upper(actasmpattern)='HI' then
                  oper.opr.ref.refaddr:=addr_high
                else if upper(actasmpattern)='HA' then
                  oper.opr.ref.refaddr:=addr_higha
                else}
                  Message(asmr_e_invalid_reference_syntax);
                Consume(AS_ID);
              end
            else
              Message(asmr_e_invalid_reference_syntax);
          end;
      end;


    procedure trv64attreader.BuildReference(oper: trvoperand);

      procedure Consume_RParen;
        begin
          if actasmtoken <> AS_RPAREN then
           Begin
             Message(asmr_e_invalid_reference_syntax);
             RecoverConsume(true);
           end
          else
           begin
             Consume(AS_RPAREN);
             if not (actasmtoken in [AS_COMMA,AS_SEPARATOR,AS_END]) then
              Begin
                Message(asmr_e_invalid_reference_syntax);
                RecoverConsume(true);
              end;
           end;
        end;

      var
        l : aint;
        relsym: string;
        asmsymtyp: tasmsymtype;
        isflags: tindsymflags;

      begin
        Consume(AS_LPAREN);
        Case actasmtoken of
          AS_INTNUM,
          AS_MINUS,
          AS_PLUS:
            Begin
              { offset(offset) is invalid }
              If oper.opr.Ref.Offset <> 0 Then
               Begin
                 Message(asmr_e_invalid_reference_syntax);
                 RecoverConsume(true);
               End
              Else
               Begin
                 oper.opr.Ref.Offset:=BuildConstExpression(false,true);
                 Consume(AS_RPAREN);
                 if actasmtoken=AS_AT then
                   ReadAt(oper);
               end;
              exit;
            End;
          AS_REGISTER: { (reg ...  }
            Begin
              if ((oper.opr.typ=OPR_REFERENCE) and (oper.opr.ref.base<>NR_NO)) or
                 ((oper.opr.typ=OPR_LOCAL) and (oper.opr.localsym.localloc.loc<>LOC_REGISTER)) then
                message(asmr_e_cannot_index_relative_var);
              oper.opr.ref.base:=actasmregister;
              Consume(AS_REGISTER);
              Consume_RParen;
            end; {end case }
          AS_ID:
            Begin
              ReadSym(oper);
              case actasmtoken of
                AS_PLUS:
                  begin
                    { add a constant expression? }
                    l:=BuildConstExpression(true,true);
                    case oper.opr.typ of
                      OPR_CONSTANT :
                        inc(oper.opr.val,l);
                      OPR_LOCAL :
                        inc(oper.opr.localsymofs,l);
                      OPR_REFERENCE :
                        inc(oper.opr.ref.offset,l);
                      else
                        internalerror(200309202);
                    end;
                  end;
                AS_MINUS:
                  begin
                    Consume(AS_MINUS);
                    BuildConstSymbolExpression(false,true,false,l,relsym,asmsymtyp);
                    if (relsym<>'') then
                      begin
                        if (oper.opr.typ = OPR_REFERENCE) then
                          oper.opr.ref.relsymbol:=current_asmdata.RefAsmSymbol(relsym,AT_DATA)
                        else
                          begin
                            Message(asmr_e_invalid_reference_syntax);
                            RecoverConsume(false);
                          end
                      end
                    else
                      begin
                        case oper.opr.typ of
                          OPR_CONSTANT :
                            dec(oper.opr.val,l);
                          OPR_LOCAL :
                            dec(oper.opr.localsymofs,l);
                          OPR_REFERENCE :
                            dec(oper.opr.ref.offset,l);
                          else
                            internalerror(2007092601);
                        end;
                      end;
                  end;
                else
                  ;
              end;
              Consume(AS_RPAREN);
              if actasmtoken=AS_AT then
                ReadAt(oper);
            End;
          AS_COMMA: { (, ...  can either be scaling, or index }
            Begin
              Consume(AS_COMMA);
              { Index }
              if (actasmtoken=AS_REGISTER) then
                Begin
                  oper.opr.ref.index:=actasmregister;
                  Consume(AS_REGISTER);
                  { check for scaling ... }
                  Consume_RParen;
                end
              else
                begin
                  Message(asmr_e_invalid_reference_syntax);
                  RecoverConsume(false);
                end;
            end;
        else
          Begin
            Message(asmr_e_invalid_reference_syntax);
            RecoverConsume(false);
          end;
        end;
      end;


    procedure trv64attreader.BuildOperand(oper: trvoperand);
      var
        expr : string;
        typesize,l : aint;


        procedure AddLabelOperand(hl:tasmlabel);
          begin
            if not(actasmtoken in [AS_PLUS,AS_MINUS,AS_LPAREN]) and
               is_calljmp(actopcode) then
             begin
               oper.opr.typ:=OPR_SYMBOL;
               oper.opr.symbol:=hl;
             end
            else
             begin
               oper.InitRef;
               oper.opr.ref.symbol:=hl;
             end;
          end;


        procedure MaybeRecordOffset;
          var
            mangledname: string;
            hasdot  : boolean;
            l,
            toffset,
            tsize   : aint;
          begin
            if not(actasmtoken in [AS_DOT,AS_PLUS,AS_MINUS]) then
             exit;
            l:=0;
            hasdot:=(actasmtoken=AS_DOT);
            if hasdot then
              begin
                if expr<>'' then
                  begin
                    BuildRecordOffsetSize(expr,toffset,tsize,mangledname,false);
                    if (oper.opr.typ<>OPR_CONSTANT) and
                       (mangledname<>'') then
                      Message(asmr_e_wrong_sym_type);
                    inc(l,toffset);
                    oper.SetSize(tsize,true);
                  end;
              end;
            if actasmtoken in [AS_PLUS,AS_MINUS] then
              inc(l,BuildConstExpression(true,false));
            case oper.opr.typ of
              OPR_LOCAL :
                begin
                  { don't allow direct access to fields of parameters, because that
                    will generate buggy code. Allow it only for explicit typecasting }
                  if hasdot and
                     (not oper.hastype) and
                     (tabstractvarsym(oper.opr.localsym).owner.symtabletype=parasymtable) and
                     (current_procinfo.procdef.proccalloption<>pocall_register) then
                    Message(asmr_e_cannot_access_field_directly_for_parameters);
                  inc(oper.opr.localsymofs,l)
                end;
              OPR_CONSTANT :
                if (mangledname<>'') then
                  begin
                    if (oper.opr.val<>0) then
                      Message(asmr_e_wrong_sym_type);
                    oper.opr.typ:=OPR_SYMBOL;
                    oper.opr.symbol:=current_asmdata.DefineAsmSymbol(mangledname,AB_EXTERNAL,AT_FUNCTION,voidcodepointertype);
                  end
                else
                  inc(oper.opr.val,l);
              OPR_REFERENCE :
                inc(oper.opr.ref.offset,l);
              OPR_SYMBOL:
                Message(asmr_e_invalid_symbol_ref);
              else
                internalerror(200309221);
            end;
          end;


        function MaybeBuildReference:boolean;
          { Try to create a reference, if not a reference is found then false
            is returned }
          begin
            MaybeBuildReference:=true;
            case actasmtoken of
              AS_INTNUM,
              AS_MINUS,
              AS_PLUS:
                Begin
                  oper.opr.ref.offset:=BuildConstExpression(True,False);
                  if actasmtoken<>AS_LPAREN then
                    Message(asmr_e_invalid_reference_syntax)
                  else
                    BuildReference(oper);
                end;
              AS_LPAREN:
                BuildReference(oper);
              AS_ID: { only a variable is allowed ... }
                Begin
                  ReadSym(oper);
                  case actasmtoken of
                    AS_END,
                    AS_SEPARATOR,
                    AS_COMMA: ;
                    AS_LPAREN:
                      BuildReference(oper);
                  else
                    Begin
                      Message(asmr_e_invalid_reference_syntax);
                      Consume(actasmtoken);
                    end;
                  end; {end case }
                end;
              else
               MaybeBuildReference:=false;
            end; { end case }
          end;


        function is_fenceflag(hs : string): boolean;
          var
            i: longint;
            flags: TFenceFlags;
          begin
            is_fenceflag := false;

            flags:=[];
            hs:=lower(hs);

            if (actopcode in [A_FENCE]) and (length(hs) >= 1) then
              begin
                for i:=1 to length(hs) do
                  begin
                    case hs[i] of
                      'i':
                        Include(flags,ffi);
                      'o':
                        Include(flags,ffo);
                      'r':
                        Include(flags,ffr);
                      'w':
                        Include(flags,ffw);
                    else
                      exit;
                    end;
                  end;
                oper.opr.typ := OPR_FENCEFLAGS;
                oper.opr.fenceflags := flags;
                exit(true);
              end;
          end;


      var
        tempreg : tregister;
        hl : tasmlabel;
        ofs : aint;
        refaddr: trefaddr;
        entered_paren: Boolean;
      Begin
        expr:='';
        entered_paren:=false;

        refaddr:=addr_full;
        if actasmtoken=AS_MOD then
          begin
            consume(AS_MOD);

            if actasmtoken<>AS_ID then
              begin
                Message(asmr_e_invalid_reference_syntax);
                RecoverConsume(false);
              end
            else
              begin
                if lower(actasmpattern)='pcrel_hi' then
                  refaddr:=addr_pcrel_hi20
                else if lower(actasmpattern)='pcrel_lo' then
                  refaddr:=addr_pcrel_lo12
                else if lower(actasmpattern)='hi' then
                  refaddr:=addr_hi20
                else if lower(actasmpattern)='lo' then
                  refaddr:=addr_lo12
                else
                  begin
                    Message(asmr_e_invalid_reference_syntax);
                    RecoverConsume(false);
                  end;

                consume(AS_ID);
                consume(AS_LPAREN);
                entered_paren:=true;
              end;
          end;

        case actasmtoken of
          AS_LPAREN: { Memory reference or constant expression }
            Begin
              oper.InitRef;
              BuildReference(oper);
            end;

          AS_INTNUM,
          AS_MINUS,
          AS_PLUS:
            Begin
              { Constant memory offset }
              { This must absolutely be followed by (  }
              oper.InitRef;
              oper.opr.ref.offset:=BuildConstExpression(True,False);
              if actasmtoken<>AS_LPAREN then
                begin
                  ofs:=oper.opr.ref.offset;
                  BuildConstantOperand(oper);
                  inc(oper.opr.val,ofs);
                end
              else
                BuildReference(oper);
            end;

          AS_DOT,
          AS_ID: { A constant expression, or a Variable ref.  }
            Begin
              if is_fenceflag(actasmpattern) then
                begin
                  consume(AS_ID);
                end
              else
              { Local Label ? }
              if is_locallabel(actasmpattern) then
               begin
                 CreateLocalLabel(actasmpattern,hl,false);
                 Consume(AS_ID);
                 AddLabelOperand(hl);
               end
              else
               { Check for label }
               if SearchLabel(actasmpattern,hl,false) then
                begin
                  Consume(AS_ID);
                  AddLabelOperand(hl);
                end
              else
               { probably a variable or normal expression }
               { or a procedure (such as in CALL ID)      }
               Begin
                 { is it a constant ? }
                 if SearchIConstant(actasmpattern,l) then
                  Begin
                    if not (oper.opr.typ in [OPR_NONE,OPR_CONSTANT]) then
                     Message(asmr_e_invalid_operand_type);
                    BuildConstantOperand(oper);
                  end
                 else
                  begin
                    expr:=actasmpattern;
                    Consume(AS_ID);
                    { typecasting? }
                    if (actasmtoken=AS_LPAREN) and
                       SearchType(expr,typesize) then
                     begin
                       oper.hastype:=true;
                       Consume(AS_LPAREN);
                       BuildOperand(oper);
                       Consume(AS_RPAREN);
                       if oper.opr.typ in [OPR_REFERENCE,OPR_LOCAL] then
                         oper.SetSize(typesize,true);
                     end
                    else
                     begin
                       if oper.SetupVar(expr,false) then
                         ReadAt(oper)
                       else
                        Begin
                          { look for special symbols ... }
                          if expr= '__HIGH' then
                            begin
                              consume(AS_LPAREN);
                              if not oper.setupvar('high'+actasmpattern,false) then
                                Message1(sym_e_unknown_id,'high'+actasmpattern);
                              consume(AS_ID);
                              consume(AS_RPAREN);
                            end
                          else
                           if expr = '__RESULT' then
                            oper.SetUpResult
                          else
                           if expr = '__SELF' then
                            oper.SetupSelf
                          else
                           if expr = '__OLDEBP' then
                            oper.SetupOldEBP
                          else
                            Message1(sym_e_unknown_id,expr);
                        end;
                     end;
                  end;
                  if actasmtoken=AS_DOT then
                    MaybeRecordOffset;
                  { add a constant expression? }
                  if (actasmtoken=AS_PLUS) then
                   begin
                     l:=BuildConstExpression(true,entered_paren);
                     case oper.opr.typ of
                       OPR_CONSTANT :
                         inc(oper.opr.val,l);
                       OPR_LOCAL :
                         inc(oper.opr.localsymofs,l);
                       OPR_REFERENCE :
                         inc(oper.opr.ref.offset,l);
                       else
                         internalerror(200309202);
                     end;
                   end
               end;
              { Do we have a indexing reference, then parse it also }
              if actasmtoken=AS_LPAREN then
                BuildReference(oper);
            end;

          AS_REGISTER: { Register, a variable reference or a constant reference  }
            Begin
              { save the type of register used. }
              tempreg:=actasmregister;
              Consume(AS_REGISTER);
              if (actasmtoken in [AS_END,AS_SEPARATOR,AS_COMMA]) then
                  begin
                    if not (oper.opr.typ in [OPR_NONE,OPR_REGISTER]) then
                      Message(asmr_e_invalid_operand_type);
                    oper.opr.typ:=OPR_REGISTER;
                    oper.opr.reg:=tempreg;
                  end
              else
                Message(asmr_e_syn_operand);
            end;
          AS_END,
          AS_SEPARATOR,
          AS_COMMA: ;
        else
          Begin
            Message(asmr_e_syn_operand);
            Consume(actasmtoken);
          end;
        end; { end case }

        if refaddr<>addr_full then
          begin
            if oper.opr.typ<>OPR_REFERENCE then
              oper.InitRef;

            oper.opr.ref.refaddr:=refaddr;
            Consume(AS_RPAREN);
          end
        else if (oper.opr.typ=OPR_REFERENCE) and
           (oper.opr.ref.refaddr=addr_no) and
           assigned(oper.opr.ref.symbol) then
          oper.opr.ref.refaddr:=addr_full;
      end;


{*****************************************************************************
                                trv64attreader
*****************************************************************************}

    procedure trv64attreader.BuildOpCode(instr : trvinstruction);
      var
        operandnum : longint;
      Begin
        { opcode }
        if (actasmtoken<>AS_OPCODE) then
         Begin
           Message(asmr_e_invalid_or_missing_opcode);
           RecoverConsume(true);
           exit;
         end;
        { Fill the instr object with the current state }
        with instr do
          begin
            Opcode:=ActOpcode;
            condition:=ActCondition;
            ordering:=actmemoryordering;
          end;

        { We are reading operands, so opcode will be an AS_ID }
        operandnum:=1;
        Consume(AS_OPCODE);
        { Zero operand opcode ?  }
        if actasmtoken in [AS_SEPARATOR,AS_END] then
         begin
           operandnum:=0;
           exit;
         end;
        { Read the operands }
        repeat
          case actasmtoken of
            AS_COMMA: { Operand delimiter }
              Begin
                if operandnum>Max_Operands then
                  Message(asmr_e_too_many_operands)
                else
                  begin
                    { condition operands doesn't set the operand but write to the
                      condition field of the instruction
                    }
                    if instr.Operands[operandnum].opr.typ<>OPR_NONE then
                      Inc(operandnum);
                  end;
                Consume(AS_COMMA);
              end;
            AS_SEPARATOR,
            AS_END : { End of asm operands for this opcode  }
              begin
                break;
              end;
          else
            BuildOperand(instr.Operands[operandnum] as trvoperand);
          end; { end case }
        until false;
        if (operandnum=1) and (instr.Operands[operandnum].opr.typ=OPR_NONE) then
          dec(operandnum);
        instr.Ops:=operandnum;
      end;

    function trv64attreader.is_register(const s: string): boolean;
      type
        treg2str = record
          name : string[3];
          reg : tregister;
        end;

      const
        extraregs : array[0..31] of treg2str = (
          (name: 'A0'; reg : NR_X10),
          (name: 'A1'; reg : NR_X11),
          (name: 'A2'; reg : NR_X12),
          (name: 'A3'; reg : NR_X13),
          (name: 'A4'; reg : NR_X14),
          (name: 'A5'; reg : NR_X15),
          (name: 'A6'; reg : NR_X16),
          (name: 'A7'; reg : NR_X17),
          (name: 'RA'; reg : NR_X1),
          (name: 'SP'; reg : NR_X2),
          (name: 'GP'; reg : NR_X3),
          (name: 'TP'; reg : NR_X4),
          (name: 'T0'; reg : NR_X5),
          (name: 'T1'; reg : NR_X6),
          (name: 'T2'; reg : NR_X7),
          (name: 'S0'; reg : NR_X8),
          (name: 'FP'; reg : NR_X8),
          (name: 'S1'; reg : NR_X9),
          (name: 'S2'; reg : NR_X18),
          (name: 'S3'; reg : NR_X19),
          (name: 'S4'; reg : NR_X20),
          (name: 'S5'; reg : NR_X21),
          (name: 'S6'; reg : NR_X22),
          (name: 'S7'; reg : NR_X23),
          (name: 'S8'; reg : NR_X24),
          (name: 'S9'; reg : NR_X25),
          (name: 'S10';reg : NR_X26),
          (name: 'S11';reg : NR_X27),
          (name: 'T3'; reg : NR_X28),
          (name: 'T4'; reg : NR_X29),
          (name: 'T5'; reg : NR_X30),
          (name: 'T6'; reg : NR_X31)
          );

      var
        i : longint;

      begin
        result:=inherited is_register(s);
        { reg found?
          possible aliases are always 2 char
        }
        if result or (not (length(s) in [2,3])) then
          exit;
        for i:=low(extraregs) to high(extraregs) do
          begin
            if s=extraregs[i].name then
              begin
                actasmregister:=extraregs[i].reg;
                result:=true;
                actasmtoken:=AS_REGISTER;
                exit;
              end;
          end;
      end;


    function trv64attreader.is_asmopcode(const s: string):boolean;
      var
        cond  : tasmcond;
        hs, postfix : string;
        l: longint;
      Begin
        { making s a value parameter would break other assembler readers }
        hs:=s;
        is_asmopcode:=false;

        { clear op code }
        actopcode:=A_None;
        { clear condition }
        fillchar(actcondition,sizeof(actcondition),0);

        { check for direction hint }
        actopcode := tasmop(ptruint(iasmops.find(hs)));
        if actopcode <> A_NONE then
          begin
            actasmtoken:=AS_OPCODE;
            is_asmopcode:=true;
            exit;
          end;
        { not found, check branch instructions }
        if hs[1]='B' then
          begin
            { we can search here without an extra table which is sorted by string length
              because we take the whole remaining string without the leading B }
            actopcode := A_Bxx;
            for cond:=low(TAsmCond) to high(TAsmCond) do
              if copy(hs,2,length(s)-1)=uppercond2str[cond] then
                begin
                  actcondition:=cond;
                  actasmtoken:=AS_OPCODE;
                  is_asmopcode:=true;
                  exit;
                end;
          end;

        { check atomic instructions }
        if (pos('AMO',hs)=1) or
           (pos('LR', hs)=1) or
           (pos('SC', hs)=1) then
          begin
            l := length(hs)-1;
            while l>1 do
              begin
                actopcode := tasmop(ptruint(iasmops.find(copy(hs,1,l))));
                if actopcode <> A_None then
                  begin
                    postfix := copy(hs,l+1,length(hs)-l);

                    if postfix='.AQRL' then actmemoryordering:=[moAq,moRl]
                    else if postfix='.RL' then actmemoryordering:=[moRl]
                    else if postfix='.AQ' then actmemoryordering:=[moAq]
                    else
                      exit;

                    actasmtoken:=AS_OPCODE;
                    is_asmopcode:=true;
                    exit;
                  end;
                dec(l);
              end;
          end;
      end;


    procedure trv64attreader.handleopcode;
      var
        instr : trvinstruction;
      begin
        instr:=trvinstruction.Create(trvoperand);
        BuildOpcode(instr);
        instr.condition := actcondition;
        {
        instr.AddReferenceSizes;
        instr.SetInstructionOpsize;
        instr.CheckOperandSizes;
        }
        instr.ConcatInstruction(curlist);
        instr.Free;
        actmemoryordering:=[];
      end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

    const
      asmmode_rv64_standard_info : tasmmodeinfo =
              (
                id    : asmmode_standard;
                idtxt : 'STANDARD';
                casmreader : trv64attreader;
              );

initialization
  RegisterAsmMode(asmmode_rv64_standard_info);
end.

