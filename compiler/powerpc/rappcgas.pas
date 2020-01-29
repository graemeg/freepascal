{
    Copyright (c) 1998-2002 by Carl Eric Codere and Peter Vreman

    Does the parsing for the PowerPC GNU AS styled inline assembler.

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
Unit rappcgas;

{$i fpcdefs.inc}

  Interface

    uses
      raatt,rappc;

    type
      tppcattreader = class(tattreader)
        function is_asmopcode(const s: string):boolean;override;
        procedure handleopcode;override;
        procedure BuildReference(oper : tppcoperand);
        procedure BuildOperand(oper : tppcoperand);
        procedure BuildOpCode(instr : tppcinstruction);
        procedure ReadAt(oper : tppcoperand);
        procedure ReadSym(oper : tppcoperand);
        procedure ConvertCalljmp(instr : tppcinstruction);
      end;


  Implementation

    uses
      { helpers }
      cutils,
      { global }
      globtype,globals,verbose,
      systems,
      { aasm }
      cpubase,aasmbase,aasmtai,aasmdata,aasmcpu,
      { symtable }
      symconst,symsym,symdef,
      { parser }
      procinfo,
      rabase,rautils,
      cgbase,cgobj,cgppc,paramgr
      ;

    procedure tppcattreader.ReadSym(oper : tppcoperand);
      var
         tempstr, mangledname : string;
         typesize,l,k : tcgint;
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


    procedure tppcattreader.ReadAt(oper : tppcoperand);
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
                if upper(actasmpattern)='L' then
                  oper.opr.ref.refaddr:=addr_low
                else if upper(actasmpattern)='HI' then
                  oper.opr.ref.refaddr:=addr_high
                else if upper(actasmpattern)='HA' then
                  oper.opr.ref.refaddr:=addr_higha
                else
                  Message(asmr_e_invalid_reference_syntax);
                Consume(AS_ID);
              end
            else
              Message(asmr_e_invalid_reference_syntax);
          end;
      end;


    Procedure tppcattreader.BuildReference(oper : tppcoperand);

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
        l : tcgint;
        relsym: string;
        asmsymtyp: tasmsymtype;

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
                 { cast explicitly to avoid range check errors }
                 oper.opr.Ref.Offset:=ASizeInt(BuildConstExpression(false,true));
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
              { can either be a register or a right parenthesis }
              { (reg)        }
              if actasmtoken=AS_RPAREN then
               Begin
                 { detect RTOC-based symbol accesses }
                 if assigned(oper.opr.ref.symbol) and
                    (oper.opr.ref.base=NR_RTOC) and
                    (oper.opr.ref.offset=0) then
                   begin
                     { replace global symbol reference with TOC entry name
                       for AIX }
                     if target_info.system in systems_aix then
                       tcgppcgen(cg).get_aix_toc_sym(nil,oper.opr.ref.symbol.name,asmsym2indsymflags(oper.opr.ref.symbol),oper.opr.ref,true);
                     oper.opr.ref.refaddr:=addr_pic_no_got;
                   end;
                 Consume_RParen;
                 exit;
               end;
              { (reg,reg ..  }
              Consume(AS_COMMA);
              if (actasmtoken=AS_REGISTER) and
                 (oper.opr.Ref.Offset = 0) then
               Begin
                 oper.opr.ref.index:=actasmregister;
                 Consume(AS_REGISTER);
                 Consume_RParen;
               end
              else
               Begin
                 Message(asmr_e_invalid_reference_syntax);
                 RecoverConsume(false);
               end;
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
                          oper.opr.ref.relsymbol:=current_asmdata.RefAsmSymbol(relsym,asmsymtyp)
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


    Procedure tppcattreader.BuildOperand(oper : tppcoperand);
      var
        expr : string;
        typesize,l : tcgint;


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


      var
        tempreg : tregister;
        hl : tasmlabel;
        ofs : tcgint;
      Begin
        expr:='';
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

          AS_ID: { A constant expression, or a Variable ref.  }
            Begin
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
                     l:=BuildConstExpression(true,false);
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
                if is_condreg(tempreg) and
                   ((actopcode = A_BC) or
                    (actopcode = A_BCCTR) or
                    (actopcode = A_BCLR) or
                    (actopcode = A_TW) or
                    (actopcode = A_TWI)) then
                  begin
                    { it isn't a real operand, everything is stored in the condition }
                    oper.opr.typ:=OPR_NONE;
                    actcondition.cr := getsupreg(tempreg);
                  end
                else
                  begin
                    if not (oper.opr.typ in [OPR_NONE,OPR_REGISTER]) then
                      Message(asmr_e_invalid_operand_type);
                    oper.opr.typ:=OPR_REGISTER;
                    oper.opr.reg:=tempreg;
                  end
              else if is_condreg(tempreg) then
                begin
                  if not(actcondition.cond in [C_T..C_DZF]) then
                    Message(asmr_e_syn_operand);
                  if actasmtoken=AS_STAR then
                    begin
                      consume(AS_STAR);
                      if (actasmtoken=AS_INTNUM) then
                        begin
                          consume(AS_INTNUM);
                          if actasmtoken=AS_PLUS then
                            begin
                              consume(AS_PLUS);
                              if (actasmtoken=AS_ID) then
                                begin
                                  oper.opr.typ:=OPR_NONE;
                                  if actasmpattern='LT' then
                                    actcondition.crbit:=(getsupreg(tempreg)-(RS_CR0))*4
                                  else if actasmpattern='GT' then
                                    actcondition.crbit:=(getsupreg(tempreg)-(RS_CR0))*4+1
                                  else if actasmpattern='EQ' then
                                    actcondition.crbit:=(getsupreg(tempreg)-(RS_CR0))*4+2
                                  else if actasmpattern='SO' then
                                    actcondition.crbit:=(getsupreg(tempreg)-(RS_CR0))*4+3
                                  else
                                    Message(asmr_e_syn_operand);
                                  consume(AS_ID);
                                end
                              else
                                Message(asmr_e_syn_operand);
                            end
                          else
                            Message(asmr_e_syn_operand);
                        end
                      else
                        Message(asmr_e_syn_operand);
                    end
                  else
                    Message(asmr_e_syn_operand);
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
                                tppcattreader
*****************************************************************************}

    procedure tppcattreader.BuildOpCode(instr : tppcinstruction);
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
            BuildOperand(instr.Operands[operandnum] as tppcoperand);
          end; { end case }
        until false;
        if (operandnum=1) and (instr.Operands[operandnum].opr.typ=OPR_NONE) then
          dec(operandnum);
        instr.Ops:=operandnum;
      end;


    function tppcattreader.is_asmopcode(const s: string):boolean;
      var
        cond  : tasmcondflag;
        hs : string;

      Begin
        { making s a value parameter would break other assembler readers }
        hs:=s;
        is_asmopcode:=false;

        { clear op code }
        actopcode:=A_None;
        { clear condition }
        fillchar(actcondition,sizeof(actcondition),0);

        { check for direction hint }
        if hs[length(s)]='-' then
          begin
            dec(ord(hs[0]));
            actcondition.dirhint:=DH_Minus;
          end
        else if hs[length(s)]='+' then
          begin
            dec(ord(hs[0]));
            actcondition.dirhint:=DH_Plus;
          end;
	actopcode := tasmop(ptruint(iasmops.find(hs)));
        if actopcode <> A_NONE then
          begin
            if actcondition.dirhint<>DH_None then
              message1(asmr_e_unknown_opcode,actasmpattern);
            actasmtoken:=AS_OPCODE;
            is_asmopcode:=true;
            exit;
          end;
        { not found, check branch instructions }
        if hs[1]='B' then
          begin
            { we can search here without an extra table which is sorted by string length
              because we take the whole remaining string without the leading B }
            if copy(hs,length(s)-1,2)='LR' then
              begin
                actopcode := A_BCLR;
                setlength(hs,length(hs)-2)
              end
            else if copy(hs,length(s)-2,3)='CTR' then
              begin
                actopcode := A_BCCTR;
                setlength(hs,length(hs)-3)
              end
            else
              actopcode := A_BC;
            for cond:=low(TAsmCondFlag) to high(TAsmCondFlag) do
              if copy(hs,2,length(s)-1)=UpperAsmCondFlag2Str[cond] then
                begin
                  actcondition.simple:=true;
                  actcondition.cond:=cond;
                  if (cond in [C_LT,C_LE,C_EQ,C_GE,C_GT,C_NL,C_NE,C_NG,C_SO,C_NS,C_UN,C_NU]) then
                    actcondition.cr := RS_CR0;
                  actasmtoken:=AS_OPCODE;
                  is_asmopcode:=true;
                  exit;
                end;
          end;
      end;

    procedure tppcattreader.ConvertCalljmp(instr : tppcinstruction);
      begin
        if instr.Operands[1].opr.typ = OPR_CONSTANT then
          begin
            if (instr.operands[1].opr.val > 31) or
               (instr.operands[2].opr.typ <> OPR_CONSTANT) or
               (instr.operands[2].opr.val > 31) or
               not(instr.operands[3].opr.typ in [OPR_REFERENCE,OPR_SYMBOL]) then
              Message(asmr_e_syn_operand);
            { BO/BI notation }
            instr.condition.simple := false;
            instr.condition.bo := instr.operands[1].opr.val;
            instr.condition.bi := instr.operands[2].opr.val;
            instr.operands[1].free;
            instr.operands[2].free;
            instr.operands[2] := nil;
            instr.operands[1] := instr.operands[3];
            instr.operands[3] := nil;
            instr.ops := 1;
          end;
        if instr.Operands[1].opr.typ = OPR_REFERENCE then
          begin
            instr.Operands[1].opr.ref.refaddr:=addr_full;
            if (instr.Operands[1].opr.ref.base<>NR_NO) or
               (instr.Operands[1].opr.ref.index<>NR_NO) then
              Message(asmr_e_syn_operand);
            if use_dotted_functions and
               assigned(instr.Operands[1].opr.ref.symbol) then
              instr.Operands[1].opr.ref.symbol:=current_asmdata.DefineAsmSymbol('.'+instr.Operands[1].opr.ref.symbol.name,instr.Operands[1].opr.ref.symbol.bind,AT_FUNCTION,voidcodepointertype);
          end;
        { regular name is toc entry, .-based name is actual code }
        if use_dotted_functions and
           (instr.Operands[1].opr.typ = OPR_SYMBOL) and
           (instr.Operands[1].opr.symbol.typ=AT_FUNCTION) then
          instr.Operands[1].opr.symbol:=current_asmdata.DefineAsmSymbol('.'+instr.Operands[1].opr.symbol.name,instr.Operands[1].opr.symbol.bind,AT_FUNCTION,voidcodepointertype);
      end;


    procedure tppcattreader.handleopcode;
      var
        instr : tppcinstruction;
      begin
        instr:=TPPCInstruction.Create(TPPCOperand);
        BuildOpcode(instr);
        instr.condition := actcondition;
        if is_calljmp(instr.opcode) then
          ConvertCalljmp(instr);
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
  asmmode_ppc_att_info : tasmmodeinfo =
          (
            id    : asmmode_ppc_gas;
            idtxt : 'GAS';
            casmreader : tppcattreader;
          );

  asmmode_ppc_standard_info : tasmmodeinfo =
          (
            id    : asmmode_standard;
            idtxt : 'STANDARD';
            casmreader : tppcattreader;
          );

initialization
  RegisterAsmMode(asmmode_ppc_att_info);
  RegisterAsmMode(asmmode_ppc_standard_info);
end.
