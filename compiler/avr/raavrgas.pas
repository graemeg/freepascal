{
    Copyright (c) 1998-2008 by Carl Eric Codere and Peter Vreman

    Does the parsing for the ARM GNU AS styled inline assembler.

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
Unit raavrgas;

{$i fpcdefs.inc}

  Interface

    uses
      raatt,raavr,
      cpubase;

    type
      tavrattreader = class(tattreader)
        function is_asmopcode(const s: string):boolean;override;
        function is_register(const s:string):boolean;override;
        procedure handleopcode;override;
        procedure BuildReference(oper : tavroperand);
        procedure BuildOperand(oper : tavroperand);
        procedure BuildOpCode(instr : tavrinstruction);
        procedure ReadSym(oper : tavroperand);
        procedure ConvertCalljmp(instr : tavrinstruction);
      end;


  Implementation

    uses
      { helpers }
      cutils,
      { global }
      globtype,globals,verbose,
      systems,
      { aasm }
      cpuinfo,aasmbase,aasmtai,aasmdata,aasmcpu,
      { symtable }
      symconst,symbase,symtype,symsym,symtable,symdef,
      { parser }
      scanner,
      procinfo,
      itcpugas,
      rabase,rautils,
      cgbase,cgutils,cgobj,paramgr
      ;


    function tavrattreader.is_register(const s:string):boolean;
      type
        treg2str = record
          name : string[2];
          reg : tregister;
        end;

      const
        extraregs : array[0..5] of treg2str = (
          (name: 'XL'; reg : NR_R26),
          (name: 'XH'; reg : NR_R27),
          (name: 'YL'; reg : NR_R28),
          (name: 'YH'; reg : NR_R29),
          (name: 'ZL'; reg : NR_R30),
          (name: 'ZH'; reg : NR_R31)
        );

      var
        i : longint;

      begin
        result:=inherited is_register(s);
        { reg found?
          possible aliases are always 2 char
        }
        if result or (not (length(s) in [1,2])) then
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


    procedure tavrattreader.ReadSym(oper : tavroperand);
      var
        tempstr, mangledname : string;
        typesize : tcgint;
        l,k : tcgint;
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


    Procedure tavrattreader.BuildReference(oper : tavroperand);

      procedure Consume_RParen;
        begin
          if actasmtoken<>AS_RPAREN then
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


      procedure read_index;
        begin
          Consume(AS_COMMA);
          if actasmtoken=AS_REGISTER then
            Begin
              oper.opr.ref.index:=actasmregister;
              Consume(AS_REGISTER);
            end
          else if actasmtoken=AS_HASH then
            begin
              Consume(AS_HASH);
              inc(oper.opr.ref.offset,BuildConstExpression(false,true));
            end;
        end;


      begin
        Consume(AS_LPAREN);
        if actasmtoken=AS_REGISTER then
          begin
            oper.opr.ref.base:=actasmregister;
            Consume(AS_REGISTER);
            { can either be a register or a right parenthesis }
            { (reg)        }
            if actasmtoken=AS_LPAREN then
             Begin
               Consume_RParen;
               exit;
             end;
            if actasmtoken=AS_PLUS then
              begin
                consume(AS_PLUS);
                oper.opr.ref.addressmode:=AM_POSTINCREMENT;
              end;
          end {end case }
        else
          Begin
            Message(asmr_e_invalid_reference_syntax);
            RecoverConsume(false);
          end;
      end;


    Procedure tavrattreader.BuildOperand(oper : tavroperand);
      var
        expr : string;
        typesize,l : tcgint;


        procedure AddLabelOperand(hl:tasmlabel);
          begin
            if not(actasmtoken in [AS_PLUS,AS_MINUS,AS_LPAREN]) { and
               is_calljmp(actopcode) } then
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
            tsize   : tcgint;
          begin
            if not(actasmtoken in [AS_DOT,AS_PLUS,AS_MINUS]) then
             exit;
            l:=0;
            mangledname:='';
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
                     (not oper.hastype) then
                     checklocalsubscript(oper.opr.localsym);
                  inc(oper.opr.localsymofs,l)
                end;
              OPR_CONSTANT :
                inc(oper.opr.val,l);
              OPR_REFERENCE :
                if (mangledname<>'') then
                  begin
                    if (oper.opr.val<>0) then
                      Message(asmr_e_wrong_sym_type);
                    oper.opr.typ:=OPR_SYMBOL;
                    oper.opr.symbol:=current_asmdata.RefAsmSymbol(mangledname,AT_FUNCTION);
                  end
                else
                  inc(oper.opr.val,l);
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
                  case actasmtoken of
                    AS_LPAREN:
                      BuildReference(oper);
                    AS_COMMA:
                      exit;
                    else
                      Message(asmr_e_invalid_reference_syntax)
                  end;
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


      var
        tempreg : tregister;
        hl : tasmlabel;
        ofs : longint;
        tempstr : string;
        tempsymtyp : tasmsymtype;
      Begin
        expr:='';
        case actasmtoken of
          AS_LBRACKET: { Memory reference or constant expression }
            Begin
              oper.InitRef;
              BuildReference(oper);
            end;

          AS_INTNUM,
          AS_MINUS,
          AS_PLUS,
          AS_NOT:
            Begin
              if (actasmtoken=AS_MINUS) and
                 (actopcode in [A_LD,A_ST]) then
                begin
                  { Special handling of predecrement addressing }
                  oper.InitRef;
                  oper.opr.ref.addressmode:=AM_PREDRECEMENT;

                  consume(AS_MINUS);

                  if actasmtoken=AS_REGISTER then
                    begin
                      oper.opr.ref.base:=actasmregister;
                      consume(AS_REGISTER);
                    end
                  else
                    begin
                      Message(asmr_e_invalid_reference_syntax);
                      RecoverConsume(false);
                    end;
                end
              else
                begin
                  { Constant memory offset }
                  { This must absolutely be followed by (  }
                  oper.InitRef;
                  oper.opr.ref.offset:=BuildConstExpression(True,False);

                  { absolute memory addresss? }
                  if ((actopcode = A_LDS) and (actasmtoken <> AS_SEPARATOR)) or
                     ((actopcode = A_STS) and (actasmtoken <> AS_COMMA)) then
                    begin
                      if not(MaybeBuildReference) then
                        Message(asmr_e_invalid_reference_syntax);
                    end
                  else
                    begin
                      ofs:=oper.opr.ref.offset;
                      BuildConstantOperand(oper);
                      inc(oper.opr.val,ofs);
                    end;
                end;
            end;

          AS_ID: { A constant expression, or a Variable ref.  }
            Begin
              if (actasmpattern='LO8') or (actasmpattern='HI8') then
                begin
                  { Low or High part of a constant (or constant
                    memory location) }
                  oper.InitRef;
                  if actasmpattern='LO8' then
                    oper.opr.ref.refaddr:=addr_lo8
                  else
                    oper.opr.ref.refaddr:=addr_hi8;
                  Consume(actasmtoken);
                  Consume(AS_LPAREN);
                  BuildConstSymbolExpression(false, true,false,l,tempstr,tempsymtyp);
                  if not assigned(oper.opr.ref.symbol) then
                    oper.opr.ref.symbol:=current_asmdata.RefAsmSymbol(tempstr,tempsymtyp)
                  else
                    Message(asmr_e_cant_have_multiple_relocatable_symbols);
                  case oper.opr.typ of
                    OPR_CONSTANT :
                      inc(oper.opr.val,l);
                    OPR_LOCAL :
                      inc(oper.opr.localsymofs,l);
                    OPR_REFERENCE :
                      inc(oper.opr.ref.offset,l);
                    else
                      internalerror(2003092012);
                  end;
                  Consume(AS_RPAREN);
                end
              { Local Label ? }
              else if is_locallabel(actasmpattern) then
               begin
                 CreateLocalLabel(actasmpattern,hl,false);
                 Consume(AS_ID);
                 AddLabelOperand(hl);
               end
              { Check for label }
              else if SearchLabel(actasmpattern,hl,false) then
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
                       if not(oper.SetupVar(expr,false)) then
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
                     l:=BuildConstExpression(true,false);
                     case oper.opr.typ of
                       OPR_CONSTANT :
                         inc(oper.opr.val,l);
                       OPR_LOCAL :
                         inc(oper.opr.localsymofs,l);
                       OPR_REFERENCE :
                         inc(oper.opr.ref.offset,l);
                       else
                         internalerror(2003092013);
                     end;
                   end
               end;
              { Do we have a indexing reference, then parse it also }
              if actasmtoken=AS_LPAREN then
                BuildReference(oper);
            end;

          { Register, a variable reference or a constant reference  }
          AS_REGISTER:
            Begin
              { save the type of register used. }
              tempreg:=actasmregister;
              Consume(AS_REGISTER);
              if (actasmtoken=AS_PLUS) then
                begin
                  oper.opr.typ:=OPR_REFERENCE;

                  reference_reset_base(oper.opr.ref,tempreg,0,ctempposinvalid,1,[]);

                  { add a constant expression? }
                  if actasmtoken=AS_PLUS then
                    begin
                      consume(AS_PLUS);
                      if actasmtoken in [AS_INTNUM,AS_ID] then
                        begin
                          l:=BuildConstExpression(true,false);
                          inc(oper.opr.ref.offset,l);
                        end
                      else
                        oper.opr.ref.addressmode:=AM_POSTINCREMENT;
                    end;
                end
              else if (actasmtoken in [AS_END,AS_SEPARATOR,AS_COMMA]) then
                Begin
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
      end;


{*****************************************************************************
                                tavrattreader
*****************************************************************************}

    procedure tavrattreader.BuildOpCode(instr : tavrinstruction);
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
                  Inc(operandnum);
                Consume(AS_COMMA);
              end;
            AS_SEPARATOR,
            AS_END : { End of asm operands for this opcode  }
              begin
                break;
              end;
          else
            BuildOperand(instr.Operands[operandnum] as tavroperand);
          end; { end case }
        until false;
        instr.Ops:=operandnum;
      end;


    function tavrattreader.is_asmopcode(const s: string):boolean;
(*
      const
        { sorted by length so longer postfixes will match first }
        postfix2strsorted : array[1..19] of string[2] = (
          'EP','SB','BT','SH',
          'IA','IB','DA','DB','FD','FA','ED','EA',
          'B','D','E','P','T','H','S');
*)
      var
        j : longint;
        hs : string;
        maxlen : longint;
        icond : tasmcond;
      Begin
        { making s a value parameter would break other assembler readers }
        hs:=s;
        is_asmopcode:=false;

        { clear op code }
        actopcode:=A_None;

        actcondition:=C_None;

        { first, handle B else BLS is read wrong }
        if ((copy(hs,1,2)='BR') and (length(hs)=4)) then
          begin
            for icond:=low(tasmcond) to high(tasmcond) do
              begin
                if copy(hs,2,3)=uppercond2str[icond] then
                  begin
                    actopcode:=A_BRxx;
                    actasmtoken:=AS_OPCODE;
                    actcondition:=icond;
                    is_asmopcode:=true;
                    exit;
                  end;
              end;
          end;
        maxlen:=max(length(hs),5);
        actopcode:=A_NONE;
        for j:=maxlen downto 1 do
          begin
            actopcode:=tasmop(PtrUInt(iasmops.Find(copy(hs,1,j))));
            if actopcode<>A_NONE then
              begin
                actasmtoken:=AS_OPCODE;
                { strip op code }
                delete(hs,1,j);
                break;
              end;
          end;
        if actopcode=A_NONE then
          exit;
        { search for condition, conditions are always 2 chars }
        if length(hs)>1 then
          begin
            for icond:=low(tasmcond) to high(tasmcond) do
              begin
                if copy(hs,1,2)=uppercond2str[icond] then
                  begin
                    actcondition:=icond;
                    { strip condition }
                    delete(hs,1,2);
                    break;
                  end;
              end;
          end;
        { if we stripped all postfixes, it's a valid opcode }
        is_asmopcode:=length(hs)=0;
      end;


    procedure tavrattreader.ConvertCalljmp(instr : tavrinstruction);
      var
        newopr : toprrec;
      begin
        if instr.Operands[1].opr.typ=OPR_REFERENCE then
          begin
            newopr.typ:=OPR_SYMBOL;
            newopr.symbol:=instr.Operands[1].opr.ref.symbol;
            newopr.symofs:=instr.Operands[1].opr.ref.offset;
            if (instr.Operands[1].opr.ref.base<>NR_NO) or
              (instr.Operands[1].opr.ref.index<>NR_NO) then
              Message(asmr_e_syn_operand);
            instr.Operands[1].opr:=newopr;
          end;
      end;


    procedure tavrattreader.handleopcode;
      var
        instr : tavrinstruction;
      begin
        instr:=tavrinstruction.Create(tavroperand);
        BuildOpcode(instr);
{        if is_calljmp(instr.opcode) then
          ConvertCalljmp(instr); }
        {
        instr.AddReferenceSizes;
        instr.SetInstructionOpsize;
        instr.CheckOperandSizes;
        }
        instr.ConcatInstruction(curlist);
        instr.Free;
      end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

const
  asmmode_avr_att_info : tasmmodeinfo =
          (
            id    : asmmode_avr_gas;
            idtxt : 'GAS';
            casmreader : tavrattreader;
          );

  asmmode_avr_standard_info : tasmmodeinfo =
          (
            id    : asmmode_standard;
            idtxt : 'STANDARD';
            casmreader : tavrattreader;
          );

initialization
  RegisterAsmMode(asmmode_avr_att_info);
  RegisterAsmMode(asmmode_avr_standard_info);
end.
