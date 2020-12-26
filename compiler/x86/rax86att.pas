{
    Copyright (c) 1998-2002 by Carl Eric Codere and Peter Vreman

    Does the parsing for the x86 GNU AS styled inline assembler.

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
Unit rax86att;

{$i fpcdefs.inc}

Interface

  uses
    cpubase,
    raatt,rax86, rautils;

  type

    { tx86attreader }

    tx86attreader = class(tattreader)
      ActOpsize : topsize;
      function is_asmopcode(const s: string):boolean;override;
      procedure handleopcode;override;
      procedure BuildReference(oper : tx86operand);
      procedure BuildOperand(oper : tx86operand);
      procedure BuildOpCode(instr : tx86instruction);
      procedure handlepercent;override;
     protected
      procedure MaybeGetPICModifier(var oper: tx86operand);
    end;

    { Tx86attInstruction }

    Tx86attInstruction = class(Tx86Instruction)
      procedure AddReferenceSizes; override;
      procedure FixupOpcode;override;
    end;

Implementation

    uses
      { helpers }
      cutils,
      { global }
      globtype,verbose,
      systems,
      { aasm }
      aasmbase,aasmdata,aasmcpu,
      { symtable }
      symconst,symsym,symdef,
      { parser }
      scanner,
      procinfo,
      itcpugas,
      paramgr,
      cgbase
      ;

    { Tx86attInstruction }


    procedure Tx86attInstruction.AddReferenceSizes;
    var
      i: integer;
    begin
      if (Opsize <> S_NO) and
         (MemRefInfo(opcode).ExistsSSEAVX) and
         (MemRefInfo(opcode).MemRefSize in MemRefMultiples) then
      begin
        for i := 1 to ops do
        begin
          if operands[i].Opr.Typ in [OPR_REFERENCE, OPR_LOCAL] then
          begin
            if (tx86operand(operands[i]).opsize = S_NO) then
             tx86operand(operands[i]).opsize := Opsize;
          end;
        end;
      end;

      inherited AddReferenceSizes;
    end;

    procedure Tx86attInstruction.FixupOpcode;
      begin
        case opcode of
          A_MOVQ:
            begin
              { May be either real 'movq' or a generic 'mov' with 'q' suffix. Convert to mov
                if source is a constant, or if neither operand is an mmx/xmm register }
{$ifdef x86_64}
              if (ops=2) and
                (
                  (operands[1].opr.typ=OPR_CONSTANT) or not
                    (
                      ((operands[1].opr.typ=OPR_REGISTER) and
                        (getregtype(operands[1].opr.reg) in [R_MMXREGISTER,R_MMREGISTER])) or
                      ((operands[2].opr.typ=OPR_REGISTER) and
                        (getregtype(operands[2].opr.reg) in [R_MMXREGISTER,R_MMREGISTER]))
                    )
                ) then
                opcode:=A_MOV;
{$endif x86_64}
            end;
          else
            ;
        end;
      end;

    { Tx86attReader }

    procedure tx86attreader.handlepercent;
      var
        len : longint;
      begin
        len:=1;
        actasmpattern[len]:='%';
        c:=current_scanner.asmgetchar;
        { to be a register there must be a letter and not a number }
        if c in ['0'..'9'] then
         begin
           actasmtoken:=AS_MOD;
         end
        else
         begin
           while c in ['a'..'z','A'..'Z','0'..'9'] do
            Begin
              inc(len);
              actasmpattern[len]:=c;
              c:=current_scanner.asmgetchar;
            end;
           actasmpattern[0]:=chr(len);
           uppervar(actasmpattern);
           if (actasmpattern = '%ST') and (c='(') then
            Begin
              actasmpattern:=actasmpattern+c;
              c:=current_scanner.asmgetchar;
              if c in ['0'..'9'] then
               actasmpattern:=actasmpattern + c
              else
               Message(asmr_e_invalid_fpu_register);
              c:=current_scanner.asmgetchar;
              if c <> ')' then
               Message(asmr_e_invalid_fpu_register)
              else
               Begin
                 actasmpattern:=actasmpattern + c;
                 c:=current_scanner.asmgetchar; { let us point to next character. }
               end;
            end;
           if is_register(actasmpattern) then
            exit;
           Message(asmr_e_invalid_register);
           actasmtoken:=raatt.AS_NONE;
         end;
      end;


    Procedure tx86attreader.BuildReference(oper : tx86operand);

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


      procedure Consume_Scale;
        var
          l : aint;
        begin
          { we have to process the scaling }
          l:=BuildConstExpression(false,true);
          if ((l = 2) or (l = 4) or (l = 8) or (l = 1)) then
           oper.opr.ref.scalefactor:=l
          else
           Begin
             Message(asmr_e_wrong_scale_factor);
             oper.opr.ref.scalefactor:=0;
           end;
        end;


      begin
        oper.InitRef;
        Consume(AS_LPAREN);
        Case actasmtoken of
          AS_INTNUM,
          AS_MINUS,
          AS_PLUS: { absolute offset, such as fs:(0x046c) }
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
                 Consume_RParen;
               end;
              exit;
            End;
          AS_REGISTER: { (reg ...  }
            Begin
              { Check if there is already a base (mostly ebp,esp) than this is
                not allowed, because it will give crashing code }
              if ((oper.opr.typ=OPR_REFERENCE) and (oper.opr.ref.base<>NR_NO)) or
                 ((oper.opr.typ=OPR_LOCAL) and (oper.opr.localsym.localloc.loc<>LOC_REGISTER)) then
                message(asmr_e_cannot_index_relative_var);
              oper.opr.ref.base:=actasmregister;
{$ifdef x86_64}
              { non-GOT based RIP-relative accesses are also position-independent }
              if (oper.opr.ref.base=NR_RIP) and
                 (oper.opr.ref.refaddr<>addr_pic) then
                oper.opr.ref.refaddr:=addr_pic_no_got;
{$endif x86_64}
              Consume(AS_REGISTER);
              { can either be a register or a right parenthesis }
              { (reg)        }
              if actasmtoken=AS_RPAREN then
               Begin
                 Consume_RParen;
                 exit;
               end;
              { (reg,reg ..  }
              Consume(AS_COMMA);
              if actasmtoken=AS_REGISTER then
               Begin
                 oper.opr.ref.index:=actasmregister;
                 Consume(AS_REGISTER);
                 { check for scaling ... }
                 case actasmtoken of
                   AS_RPAREN:
                     Begin
                       Consume_RParen;
                       exit;
                     end;
                   AS_COMMA:
                     Begin
                       Consume(AS_COMMA);
                       Consume_Scale;
                       Consume_RParen;
                     end;
                 else
                   Begin
                     Message(asmr_e_invalid_reference_syntax);
                     RecoverConsume(false);
                   end;
                 end; { end case }
               end
              else
               Begin
                 Message(asmr_e_invalid_reference_syntax);
                 RecoverConsume(false);
               end;
            end; {end case }
          AS_COMMA: { (, ...  can either be scaling, or index }
            Begin
              Consume(AS_COMMA);
              { Index }
              if (actasmtoken=AS_REGISTER) then
               Begin
                 oper.opr.ref.index:=actasmregister;
                 Consume(AS_REGISTER);
                 { check for scaling ... }
                 case actasmtoken of
                   AS_RPAREN:
                     Begin
                       Consume_RParen;
                       exit;
                     end;
                   AS_COMMA:
                     Begin
                       Consume(AS_COMMA);
                       Consume_Scale;
                       Consume_RParen;
                     end;
                 else
                   Begin
                     Message(asmr_e_invalid_reference_syntax);
                     RecoverConsume(false);
                   end;
                 end; {end case }
               end
              { Scaling }
              else
               Begin
                 Consume_Scale;
                 Consume_RParen;
                 exit;
               end;
            end;
        else
          Begin
            Message(asmr_e_invalid_reference_syntax);
            RecoverConsume(false);
          end;
        end;
      end;


    Procedure tx86attreader.MaybeGetPICModifier(var oper: tx86operand);
      var
        relsym: string;
        asmsymtyp: tasmsymtype;
        l: tcgint;
        sym: tasmsymbol;
      begin
        case actasmtoken of
          AS_AT:
            begin
              { darwin/i386 needs a relsym instead, and we can't }
              { generate this automatically                      }
              if (target_info.system in [system_i386_darwin,system_i386_iphonesim]) then
                Message(asmr_e_invalid_reference_syntax);
              consume(AS_AT);
              if actasmtoken=AS_ID then
                begin
{$ifdef x86_64}
                  if (actasmpattern='GOTPCREL') or
		     (actasmpattern='PLT') then
{$endif x86_64}
{$ifdef i386}
                  if actasmpattern='GOT' then
{$endif i386}
{$ifdef i8086}
                  if actasmpattern='GOT' then
{$endif i8086}
                    begin
                      case oper.opr.typ of
                        OPR_SYMBOL:
                          begin
                            sym:=oper.opr.symbol;
                            if oper.opr.symofs<>0 then
                              Message(asmr_e_invalid_reference_syntax);
                            oper.opr.typ:=OPR_REFERENCE;
                            fillchar(oper.opr.ref,sizeof(oper.opr.ref),0);
                            oper.opr.ref.symbol:=sym;
                          end;
                        OPR_REFERENCE:
                          begin
                            { ok }
                          end;
                        else
                          Message(asmr_e_invalid_reference_syntax)
                      end;
                      oper.opr.ref.refaddr:=addr_pic;
                      consume(AS_ID);
                    end
                  else
                    Message(asmr_e_invalid_reference_syntax);
                end
              else
                Message(asmr_e_invalid_reference_syntax);
            end;
          AS_MINUS:
            begin
              { relsym? }
              Consume(AS_MINUS);
              BuildConstSymbolExpression(true,true,false,l,relsym,asmsymtyp);
              if (relsym<>'') then
                if not assigned(oper.opr.ref.relsymbol) then
                  oper.opr.ref.relsymbol:=current_asmdata.RefAsmSymbol(relsym,asmsymtyp)
                else
                  Message(asmr_e_invalid_reference_syntax)
              else
                dec(oper.opr.ref.offset,l);
            end;
          else
            ;
        end;
      end;


    Procedure tx86attreader.BuildOperand(oper : tx86operand);
      var
        tempstr,
        expr : string;
        typesize,l,k : tcgint;


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

                   case oper.opr.typ of
                     OPR_REFERENCE: oper.opr.varsize := tsize;
                         OPR_LOCAL: oper.opr.localvarsize := tsize;
                     else
                       ;
                   end;

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
                  inc(oper.opr.localsymofs,l);
                  inc(oper.opr.localconstoffset,l);
                end;
              OPR_CONSTANT :
                if (mangledname<>'') then
                  begin
                    if (oper.opr.val<>0) then
                      Message(asmr_e_wrong_sym_type);
                    oper.opr.typ:=OPR_SYMBOL;
                    oper.opr.symbol:=current_asmdata.RefAsmSymbol(mangledname,AT_FUNCTION);
                  end
                else
                  inc(oper.opr.val,l);
              OPR_REFERENCE :
                begin
                  inc(oper.opr.ref.offset,l);
                  inc(oper.opr.constoffset,l);
                end;
              OPR_SYMBOL:
                Message(asmr_e_invalid_symbol_ref);
              else
                internalerror(200309221);
            end;
          end;


        function MaybeBuildReference:boolean;
          { Try to create a reference, if not a reference is found then false
            is returned }
          var
            mangledname: string;
          begin
            MaybeBuildReference:=true;
            case actasmtoken of
              AS_INTNUM:
                Begin
                  { allow %segmentregister:number }
                  if oper.opr.ref.segment<>NR_NO then
                    begin
                      // already done before calling oper.InitRef;
                      if oper.opr.Ref.Offset <> 0 Then
                        Message(asmr_e_invalid_reference_syntax)
                      else
                        begin
                          oper.opr.Ref.Offset:=BuildConstExpression(true,false);
                          if actasmtoken=AS_LPAREN then
                            BuildReference(oper)
                          else if (oper.opr.ref.segment <> NR_FS) and
                            (oper.opr.ref.segment <> NR_GS) then
                           Message(asmr_w_general_segment_with_constant);
                        end;
                    end
                  else
                    begin
                      oper.opr.ref.offset:=BuildConstExpression(True,False);
                      BuildReference(oper);
                    end;
                end;
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

                     case oper.opr.typ of
                       OPR_REFERENCE: oper.opr.varsize := k;
                           OPR_LOCAL: oper.opr.localvarsize := k;
                       else
                         ;
                     end;
                   end;
                  MaybeGetPICModifier(oper);
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
        hl      : tasmlabel;
      Begin
        expr:='';
        case actasmtoken of
          AS_LPAREN: { Memory reference or constant expression }
            Begin
              oper.InitRef;
              BuildReference(oper);
            end;

          AS_DOLLAR: { Constant expression  }
            Begin
              Consume(AS_DOLLAR);
              BuildConstantOperand(oper);
            end;

          AS_INTNUM,
          AS_MINUS,
          AS_PLUS:
            Begin
              { Constant memory offset }
              { This must absolutely be followed by (  }
              oper.InitRef;
              oper.opr.ref.offset:=asizeint(BuildConstExpression(True,False));
              if actasmtoken<>AS_LPAREN then
                Message(asmr_e_invalid_reference_syntax)
              else
                BuildReference(oper);
            end;

          AS_STAR: { Call from memory address }
            Begin
              Consume(AS_STAR);
              if actasmtoken=AS_REGISTER then
               begin
                 oper.opr.typ:=OPR_REGISTER;
                 oper.opr.reg:=actasmregister;
                 oper.SetSize(tcgsize2size[reg_cgsize(actasmregister)],true);
                 Consume(AS_REGISTER);
               end
              else
               begin
                 oper.InitRef;
                 if not MaybeBuildReference then
                  Message(asmr_e_syn_operand);
               end;
              { this is only allowed for call's and jmp's }
              if not is_calljmp(actopcode) then
               Message(asmr_e_syn_operand);
            end;

          AS_ID: { A constant expression, or a Variable ref.  }
            Begin
              { Local Label ? }
              if is_locallabel(actasmpattern) then
               begin
                 CreateLocalLabel(actasmpattern,hl,false);
                 Consume(AS_ID);
                 AddLabelOperand(hl);
                 MaybeGetPICModifier(oper);
               end
              else
               { Check for label }
               if SearchLabel(actasmpattern,hl,false) then
                begin
                  Consume(AS_ID);
                  AddLabelOperand(hl);
                  MaybeGetPICModifier(oper);
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
                         MaybeGetPICModifier(oper)
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
                  if oper.opr.typ<>OPR_NONE Then
                    begin
                      if (actasmtoken=AS_DOT) then
                        MaybeRecordOffset;
                      { add a constant expression? }
                      if (actasmtoken=AS_PLUS) then
                        begin
                          l:=BuildConstExpression(true,false);
                          if errorcount=0 then
                            case oper.opr.typ of
                              OPR_CONSTANT :
                                inc(oper.opr.val,l);
                              OPR_LOCAL :
                                begin
                                  inc(oper.opr.localsymofs,l);
                                  inc(oper.opr.localconstoffset, l);
                                end;
                              OPR_REFERENCE :
                                begin
                                  inc(oper.opr.ref.offset,l);
                                  inc(oper.opr.constoffset, l);
                                end;
                              else
                                internalerror(2003092011);
                            end;
                        end;
                    end;
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
              if actasmtoken = AS_COLON then
               Begin
                 Consume(AS_COLON);
                 oper.InitRef;
                 if not is_segment_reg(tempreg) then
                   Message(asmr_e_invalid_seg_override);
{$ifdef x86_64}
                 if (tempreg=NR_CS) or (tempreg=NR_DS) or (tempreg=NR_SS) or (tempreg=NR_ES) then
                   Message1(asmr_w_segment_override_ignored_in_64bit_mode,gas_regname(tempreg));
{$endif x86_64}
                 oper.opr.ref.segment:=tempreg;
                 { This must absolutely be followed by a reference }
                 if not MaybeBuildReference then
                  Begin
                    Message(asmr_e_invalid_seg_override);
                    Consume(actasmtoken);
                  end;
               end
              { Simple register  }
              else if (actasmtoken in [AS_END,AS_SEPARATOR,AS_COMMA]) then
                Begin
                  if not (oper.opr.typ in [OPR_NONE,OPR_REGISTER]) then
                    Message(asmr_e_invalid_operand_type);
                  oper.opr.typ:=OPR_REGISTER;
                  oper.opr.reg:=tempreg;
                  oper.SetSize(tcgsize2size[reg_cgsize(oper.opr.reg)],true);
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


    procedure tx86attreader.BuildOpCode(instr : tx86instruction);
      var
        operandnum : longint;
        PrefixOp,OverrideOp: tasmop;
        di_param, si_param: ShortInt;
      Begin
        PrefixOp:=A_None;
        OverrideOp:=A_None;
        { prefix seg opcode / prefix opcode }
        repeat
          if is_prefix(actopcode) then
            begin
              PrefixOp:=ActOpcode;
              with instr do
                begin
                  opcode:=ActOpcode;
                  condition:=ActCondition;
                  opsize:=ActOpsize;
                  ConcatInstruction(curlist);
                end;
              Consume(AS_OPCODE);
            end
          else
            if is_override(actopcode) then
              begin
                OverrideOp:=ActOpcode;
                with instr do
                  begin
                    opcode:=ActOpcode;
                    condition:=ActCondition;
                    opsize:=ActOpsize;
                    ConcatInstruction(curlist);
                  end;
                Consume(AS_OPCODE);
              end
            else
              break;
          { allow for newline as in gas styled syntax }
          while actasmtoken=AS_SEPARATOR do
            Consume(AS_SEPARATOR);
        until (actasmtoken<>AS_OPCODE);
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
            opsize:=ActOpsize;
          end;

        { Valid combination of prefix/override and instruction ?  }

        if (prefixop<>A_NONE) and (NOT CheckPrefix(PrefixOp,actopcode)) then
           Message1(asmr_e_invalid_prefix_and_opcode,actasmpattern);

        if (overrideop<>A_NONE) and (NOT CheckOverride(OverrideOp,ActOpcode)) then
          Message1(asmr_e_invalid_override_and_opcode,actasmpattern);
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
                if operandnum > Max_Operands then
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
            BuildOperand(instr.Operands[operandnum] as tx86operand);
          end; { end case }
        until false;
        instr.Ops:=operandnum;
        { handle string instructions with parameters }
        with instr do
          if is_x86_parameterless_string_op(opcode) and
             (Ops>=1) and (Ops<=2) then
            begin
              if opcode=A_MOVSD then
                begin
                  { distinguish between MOVS and the SSE MOVSD instruction:
                    MOVS must have memory 2 reference operands (there's no need
                    to distinguish from SSE CMPSD, because the SSE version has 3
                    arguments and we've already checked that above) }
                  if (Ops=2) and (operands[1].opr.typ=OPR_REFERENCE) and (operands[2].opr.typ=OPR_REFERENCE) then
                    begin
                      opcode:=A_MOVS;
                      opsize:=S_L;
                    end;
                end
              else
                begin
                  opsize:=get_x86_string_op_size(opcode);
                  opcode:=x86_param2paramless_string_op(opcode);
                end;
            end;
        { Check for invalid ES: overrides }
        if is_x86_parameterized_string_op(instr.opcode) then
          begin
            si_param:=get_x86_string_op_si_param(instr.opcode);
            if si_param<>-1 then
              begin
                si_param:=x86_parameterized_string_op_param_count(instr.opcode)-si_param;
                if si_param<=operandnum then
                  with instr.operands[si_param] do
                    if (opr.typ=OPR_REFERENCE) then
                      begin
                        if not((((opr.ref.index<>NR_NO) and
                                 (opr.ref.base=NR_NO) and
                                 (getregtype(opr.ref.index)=R_INTREGISTER) and
                                 (getsupreg(opr.ref.index)=RS_ESI)) or
                                ((opr.ref.index=NR_NO) and
                                 (opr.ref.base<>NR_NO) and
                                 (getregtype(opr.ref.base)=R_INTREGISTER) and
                                 (getsupreg(opr.ref.base)=RS_ESI))) and
                               (opr.ref.offset=0) and
                               (opr.ref.scalefactor<=1) and
                               (opr.ref.refaddr=addr_no) and
                               (opr.ref.symbol=nil) and
                               (opr.ref.relsymbol=nil)) then
{$if defined(i8086)}
                          Message1(asmr_w_invalid_reference,'(%si)');
{$elseif defined(i386)}
                          Message1(asmr_w_invalid_reference,'(%esi)');
{$elseif defined(x86_64)}
                          Message1(asmr_w_invalid_reference,'(%rsi)');
{$endif}
                      end;
              end;
            di_param:=get_x86_string_op_di_param(instr.opcode);
            if di_param<>-1 then
              begin
                di_param:=x86_parameterized_string_op_param_count(instr.opcode)-di_param;
                if di_param<=operandnum then
                  with instr.operands[di_param] do
                    if (opr.typ=OPR_REFERENCE) then
                      begin
                        if (opr.ref.segment<>NR_NO) and
                           (opr.ref.segment<>NR_ES) then
                          Message(asmr_e_cannot_override_es_segment);
                        if not((((opr.ref.index<>NR_NO) and
                                 (opr.ref.base=NR_NO) and
                                 (getregtype(opr.ref.index)=R_INTREGISTER) and
                                 (getsupreg(opr.ref.index)=RS_EDI)) or
                                ((opr.ref.index=NR_NO) and
                                 (opr.ref.base<>NR_NO) and
                                 (getregtype(opr.ref.base)=R_INTREGISTER) and
                                 (getsupreg(opr.ref.base)=RS_EDI))) and
                               (opr.ref.offset=0) and
                               (opr.ref.scalefactor<=1) and
                               (opr.ref.refaddr=addr_no) and
                               (opr.ref.symbol=nil) and
                               (opr.ref.relsymbol=nil)) then
{$if defined(i8086)}
                          Message1(asmr_w_invalid_reference,'(%di)');
{$elseif defined(i386)}
                          Message1(asmr_w_invalid_reference,'(%edi)');
{$elseif defined(x86_64)}
                          Message1(asmr_w_invalid_reference,'(%rdi)');
{$endif}
                      end;
              end;
            { if two memory parameters, check whether their address sizes are equal }
            if (si_param<>-1) and (di_param<>-1) and
               (si_param<=operandnum) and (di_param<=operandnum) and
               (instr.operands[si_param].opr.typ=OPR_REFERENCE) and
               (instr.operands[di_param].opr.typ=OPR_REFERENCE) then
              begin
                if get_ref_address_size(instr.operands[si_param].opr.ref)<>
                   get_ref_address_size(instr.operands[di_param].opr.ref) then
                  Message(asmr_e_address_sizes_do_not_match);
              end;
          end;
      end;


    function tx86attreader.is_asmopcode(const s: string):boolean;
      var
        cond : string[4];
        cnd  : tasmcond;
        len,
        j,
        sufidx,
        suflen : longint;
      Begin
        is_asmopcode:=FALSE;

        actopcode:=A_None;
        actcondition:=C_None;
        actopsize:=S_NO;

        { search for all possible suffixes }
        for sufidx:=low(att_sizesuffixstr) to high(att_sizesuffixstr) do
         begin
           suflen:=length(att_sizesuffixstr[sufidx]);
           len:=length(s)-suflen;
           if copy(s,len+1,suflen)=att_sizesuffixstr[sufidx] then
            begin
              { Search opcodes }
              if len>0 then
                begin
                  actopcode:=tasmop(PtrUInt(iasmops.Find(copy(s,1,len))));

                  { movsd needs special handling because it has two namings in at&t syntax (movsl for string handling and
                    movsd for the sse instruction) while only one in intel syntax (movsd, both string and sse)
                    this cannot be expressed by the instruction table format so we have to hack around this here }
                  if (actopcode = A_NONE) and (upper(s) = 'MOVSD') then
                    actopcode := A_MOVSD;
                  { cmpsd also needs special handling for pretty much the same reasons as movsd }
                  if (actopcode = A_NONE) and (upper(s) = 'CMPSD') then
                    actopcode := A_CMPSD;
                  { disambiguation between A_MOVS (movsb/movsw/movsl/movsq) and
                    A_MOVSX (movsbw/movsbl/movswl/movsbq/movswq) }
                  if (actopcode = A_MOVS) and (suflen=2) then
                    actopcode := A_MOVSX;

                  { two-letter suffix is allowed by just a few instructions (movsx,movzx),
                    and it is always required whenever allowed }
                  if (gas_needsuffix[actopcode]=attsufINTdual) xor (suflen=2) then
                    continue;

                  if actopcode<>A_NONE then
                    begin
                      if gas_needsuffix[actopcode]=attsufFPU then
                       actopsize:=att_sizefpusuffix[sufidx]
                      else if gas_needsuffix[actopcode]=attsufFPUint then
                       actopsize:=att_sizefpuintsuffix[sufidx]
                      else if gas_needsuffix[actopcode]=attsufMM then
                       actopsize:=att_sizemmsuffix[sufidx]
                      else if gas_needsuffix[actopcode]=attsufMMX then
                       actopsize:=att_sizemmXsuffix[sufidx]
                      else
                       actopsize:=att_sizesuffix[sufidx];
                      { only accept suffix from the same category that the opcode belongs to }
                      if (actopsize<>S_NO) or (suflen=0) then
                        begin
                          actasmtoken:=AS_OPCODE;
                          is_asmopcode:=TRUE;
                          exit;
                        end;
                    end;
                end;
              { not found, check condition opcodes }
              j:=0;
              while (j<CondAsmOps) do
               begin
                 if Copy(s,1,Length(CondAsmOpStr[j]))=CondAsmOpStr[j] then
                  begin
                    cond:=Copy(s,Length(CondAsmOpStr[j])+1,len-Length(CondAsmOpStr[j]));
                    if cond<>'' then
                     begin
                       for cnd:=low(TasmCond) to high(TasmCond) do
                        if Cond=Upper(cond2str[cnd]) then
                         begin
                           actopcode:=CondASmOp[j];
                           { conditional instructions (cmovcc, setcc) use only INT suffixes;
                             other stuff like fcmovcc is represented as group of individual instructions }
                           if gas_needsuffix[actopcode]=attsufINT then
                            actopsize:=att_sizesuffix[sufidx];
                           { only accept suffix from the same category that the opcode belongs to }
                           if (actopsize<>S_NO) or (suflen=0) then
                             begin
                               actcondition:=cnd;
                               actasmtoken:=AS_OPCODE;
                               is_asmopcode:=TRUE;
                               exit;
                             end;
                         end;
                     end;
                  end;
                 inc(j);
               end;
           end;
         end;
      end;


    procedure tx86attreader.handleopcode;
      var
        instr : Tx86Instruction;
      begin
        instr:=Tx86attInstruction.Create(Tx86Operand);
        BuildOpcode(instr);
        instr.AddReferenceSizes;
        instr.SetInstructionOpsize;
        instr.CheckOperandSizes;
        instr.FixupOpcode;
        instr.ConcatInstruction(curlist);
        instr.Free;
      end;



end.
