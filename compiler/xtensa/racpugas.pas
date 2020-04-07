{
    Copyright (c) 1998-2002 by Carl Eric Codere and Peter Vreman

    Does the parsing for the Xtensa GNU AS styled inline assembler.

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
Unit racpugas;

{$i fpcdefs.inc}

  Interface

    uses
      raatt,raxtensa,
      cpubase;

    type

      txtensaattreader = class(tattreader)
       function is_asmopcode(const s: string):boolean;override;
        function is_register(const s:string):boolean;override;
//        function is_targetdirective(const s: string): boolean; override;
        procedure handleopcode;override;
        procedure BuildReference(oper : TXtensaOperand);
        procedure BuildOperand(oper : TXtensaOperand);
        procedure BuildSpecialreg(oper : TXtensaOperand);
        procedure BuildOpCode(instr : TXtensaInstruction);
        procedure ReadSym(oper : TXtensaOperand);
        procedure ConvertCalljmp(instr : TXtensaInstruction);
        procedure HandleTargetDirective; override;
      end;

  Implementation

    uses
      { helpers }
      cutils,
      { global }
      globtype,globals,verbose,
      systems,aasmbase,aasmtai,aasmdata,aasmcpu,
      { symtable }
      symconst,symsym,symdef,
      procinfo,
      rabase,rautils,
      cgbase,cgutils,paramgr;


    function txtensaattreader.is_register(const s:string):boolean;
      type
        treg2str = record
          name : string[3];
          reg : tregister;
        end;

      const
        extraregs : array[0..0] of treg2str = (
          (name: 'A1'; reg : NR_A0)
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


    procedure txtensaattreader.ReadSym(oper : TXtensaOperand);
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


    Procedure txtensaattreader.BuildReference(oper : TXtensaOperand);

      procedure do_error;
        begin
          Message(asmr_e_invalid_reference_syntax);
          RecoverConsume(false);
        end;


      procedure test_end(require_rbracket : boolean);
        begin
          if require_rbracket then begin
            if not(actasmtoken=AS_RBRACKET) then
              begin
                do_error;
                exit;
              end
            else
              Consume(AS_RBRACKET);
          end;
          if not(actasmtoken in [AS_SEPARATOR,AS_end]) then
            do_error
          else
            begin
{$IFDEF debugasmreader}
              writeln('TEST_end_FINAL_OK. Created the following ref:');
              writeln('oper.opr.ref.shiftimm=',oper.opr.ref.shiftimm);
              writeln('oper.opr.ref.shiftmode=',ord(oper.opr.ref.shiftmode));
              writeln('oper.opr.ref.index=',ord(oper.opr.ref.index));
              writeln('oper.opr.ref.base=',ord(oper.opr.ref.base));
              writeln('oper.opr.ref.signindex=',ord(oper.opr.ref.signindex));
              writeln('oper.opr.ref.addressmode=',ord(oper.opr.ref.addressmode));
              writeln;
{$endIF debugasmreader}
            end;
        end;


      procedure read_index_shift(require_rbracket : boolean);
        var
          shift : aint;
        begin
          case actasmtoken of
            AS_COMMA :
              begin
                Consume(AS_COMMA);
                if not(actasmtoken=AS_ID) then
                  do_error;
              end;
            AS_RBRACKET :
              if require_rbracket then
                test_end(require_rbracket)
              else
                begin
                  do_error;
                  exit;
                end;
            AS_SEPARATOR,AS_END :
              if not require_rbracket then
                test_end(false)
               else
                 do_error;
            else
              begin
                do_error;
                exit;
              end;
          end;
        end;


      procedure read_index(require_rbracket : boolean);
        var
          recname : string;
          o_int,s_int : tcgint;
        begin
          case actasmtoken of
            AS_REGISTER :
              begin
                oper.opr.ref.index:=actasmregister;
                Consume(AS_REGISTER);
                read_index_shift(require_rbracket);
                exit;
              end;
            AS_PLUS,AS_MINUS :
              begin
                if actasmtoken=AS_PLUS then
                  begin
                    Consume(AS_PLUS);
                  end;
                if actasmtoken=AS_REGISTER then
                  begin
                    oper.opr.ref.index:=actasmregister;
                    Consume(AS_REGISTER);
                    read_index_shift(require_rbracket);
                    exit;
                  end
                else
                  begin
                    do_error;
                    exit;
                  end;
                test_end(require_rbracket);
                exit;
              end;
            AS_HASH : // constant
              begin
                Consume(AS_HASH);
                o_int := BuildConstExpression(false,true);
                if (o_int>4095) or (o_int<-4095) then
                  begin
                    Message(asmr_e_constant_out_of_bounds);
                    RecoverConsume(false);
                    exit;
                  end
                else
                  begin
                    inc(oper.opr.ref.offset,o_int);
                    test_end(require_rbracket);
                    exit;
                  end;
              end;
            AS_ID :
              begin
                recname := actasmpattern;
                Consume(AS_ID);
                BuildRecordOffsetSize(recname,o_int,s_int,recname,false);
                if (o_int>4095)or(o_int<-4095) then
                  begin
                    Message(asmr_e_constant_out_of_bounds);
                    RecoverConsume(false);
                    exit;
                  end
                else
                  begin
                    inc(oper.opr.ref.offset,o_int);
                    test_end(require_rbracket);
                    exit;
                  end;
              end;
            AS_AT:
              begin
                do_error;
                exit;
              end;
            AS_DOT : // local label
              begin
                BuildConstExpression(true,false);
                test_end(require_rbracket);
                exit;
              end;
            AS_RBRACKET :
              begin
                if require_rbracket then
                  begin
                    test_end(require_rbracket);
                    exit;
                  end
                else
                  begin
                    do_error; // unexpected rbracket
                    exit;
                  end;
              end;
            AS_SEPARATOR,AS_end :
              begin
                if not require_rbracket then
                  begin
                    test_end(false);
                    exit;
                  end
                else
                  begin
                    do_error;
                    exit;
                  end;
              end;
            else
              begin
                // unexpected token
                do_error;
                exit;
              end;
          end; // case
        end;


      procedure try_prepostindexed;
        begin
          Consume(AS_RBRACKET);
          case actasmtoken of
            AS_COMMA :
              begin // post-indexed
                Consume(AS_COMMA);
                read_index(false);
                exit;
              end;
            AS_NOT :
              begin   // pre-indexed
                Consume(AS_NOT);
                test_end(false);
                exit;
              end;
            else
              begin
                test_end(false);
                exit;
              end;
          end; // case
        end;

      var
        lab : TASMLABEL;
      begin
        Consume(AS_LBRACKET);
        if actasmtoken=AS_REGISTER then
          begin
            oper.opr.ref.base:=actasmregister;
            Consume(AS_REGISTER);
            case actasmtoken of
              AS_RBRACKET :
                begin
                  try_prepostindexed;
                  exit;
                end;
              AS_COMMA :
                begin
                  Consume(AS_COMMA);
                  read_index(true);
                  exit;
                end;
              else
                begin
                  Message(asmr_e_invalid_reference_syntax);
                  RecoverConsume(false);
                end;
            end;
          end
        else
{
  if base isn't a register, r15=PC is implied base, so it must be a local label.
  pascal constants don't make sense, because implied r15
  record offsets probably don't make sense, too (a record offset of code?)

  TODO: However, we could make the Stackpointer implied.

}

          Begin
            case actasmtoken of
              AS_ID :
                begin
                  // TODO: Stackpointer implied,
                   Message(asmr_e_invalid_reference_syntax);
                   RecoverConsume(false);
                   exit;
                end;
              else
                begin
                  Message(asmr_e_invalid_reference_syntax);
                  RecoverConsume(false);
                  exit;
                end;
            end;
          end;
      end;


    Procedure txtensaattreader.BuildOperand(oper : TXtensaOperand);
      var
        expr : string;
        typesize,l : tcgint;


        procedure AddLabelOperand(hl:tasmlabel);
          begin
            oper.opr.typ:=OPR_SYMBOL;
            oper.opr.symbol:=hl;
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
                    AS_end,
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


        procedure BuildDirectRef;

          function GetConstLabel(const symname: string; ofs: aint): TAsmLabel;
            var
              hp: tai;
              newconst: tai_const;
              lab: TAsmLabel;
            begin
              if symname<>'' then
                newconst:=tai_const.Createname(symname,ofs)
              else
                newconst:=tai_const.Create_32bit(ofs);

              hp:=tai(current_procinfo.aktlocaldata.First);
              while assigned(hp) do
                begin
                  if hp.typ=ait_const then
                    begin
                      if (tai_const(hp).sym=newconst.sym) and
                         (tai_const(hp).value=newconst.value) and
                         assigned(hp.Previous) and
                         (tai(hp.previous).typ=ait_label) then
                        begin
                          newconst.Free;
                          result:=tai_label(hp.Previous).labsym;
                          exit;
                        end;
                    end;

                  hp:=tai(hp.Next);
                end;

              current_asmdata.getjumplabel(lab);
              current_procinfo.aktlocaldata.concat(tai_align.create(4));
              current_procinfo.aktlocaldata.concat(tai_label.create(lab));
              current_procinfo.aktlocaldata.concat(newconst);
              result:=lab;
            end;

          var
            symtype: TAsmsymtype;
            sym: string;
            val: tcgint;
          begin
            case actasmtoken of
              AS_INTNUM,
              AS_ID:
                begin
                  BuildConstSymbolExpression(true,false,false,val,sym,symtype);

                  if symtype=AT_NONE then
                    sym:='';

                  reference_reset(oper.opr.ref,4,[]);
                  oper.opr.ref.symbol:=GetConstLabel(sym,val);
                end;
              else
                ;
            end;
          end;


      function getregsetindex(reg: tregister): integer;
        begin
          if getsubreg(reg)=R_SUBFS then
            begin
              result:=getsupreg(reg)*2;
              if result>32 then
                result:=result-63;
            end
          else
            result:=getsupreg(reg);
        end;

      var
        tempreg : tregister;
        ireg : tsuperregister;
        regtype: tregistertype;
        subreg: tsubregister;
        hl : tasmlabel;
        {ofs : longint;}
        registerset : tcpuregisterset;
      Begin
        expr:='';
        case actasmtoken of
          AS_LBRACKET: { Memory reference or constant expression }
            Begin
              oper.InitRef;
              BuildReference(oper);
            end;

          AS_MINUS,
          AS_PLUS,
          AS_INTNUM: { Constant expression  }
            Begin
              BuildConstantOperand(oper);
            end;
          AS_ID: { A constant expression, or a Variable ref.  }
            Begin
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
                         internalerror(200309202);
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
              if (actasmtoken in [AS_end,AS_SEPARATOR,AS_COMMA]) then
                Begin
                  if not (oper.opr.typ in [OPR_NONE,OPR_REGISTER]) then
                    Message(asmr_e_invalid_operand_type);
                  oper.opr.typ:=OPR_REGISTER;
                  oper.opr.reg:=tempreg;
                end
              else
                Message(asmr_e_syn_operand);
            end;

          AS_end,
          AS_SEPARATOR,
          AS_COMMA: ;
        else
          Begin
            Message(asmr_e_syn_operand);
            Consume(actasmtoken);
          end;
        end; { end case }
      end;

    procedure txtensaattreader.BuildSpecialreg(oper: TXtensaOperand);
      var
        hs, reg : String;
        ch : char;
        i, t : longint;
        hreg : tregister;
        flags : tspecialregflags;
      begin
        hreg:=NR_NO;
        case actasmtoken of
          AS_REGISTER:
            begin
              oper.opr.typ:=OPR_REGISTER;
              oper.opr.reg:=actasmregister;
              Consume(AS_REGISTER);
            end;
          else
            Message(asmr_e_invalid_operand_type);
        end;
      end;


{*****************************************************************************
                                tarmattreader
*****************************************************************************}

    procedure txtensaattreader.BuildOpCode(instr : TXtensaInstruction);
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
        if actasmtoken in [AS_SEPARATOR,AS_end] then
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
            AS_end : { End of asm operands for this opcode  }
              begin
                break;
              end;
          else
            BuildOperand(instr.Operands[operandnum] as TXtensaOperand);
          end; { end case }
        until false;
        instr.Ops:=operandnum;
      end;


    function txtensaattreader.is_asmopcode(const s: string):boolean;

      //const
      //  { sorted by length so longer postfixes will match first }
      //  postfix2strsorted : array[1..70] of string[9] = (
      //    '.F32.S32','.F32.U32','.S32.F32','.U32.F32','.F64.S32','.F64.U32','.S32.F64','.U32.F64',
      //    '.F32.S16','.F32.U16','.S16.F32','.U16.F32','.F64.S16','.F64.U16','.S16.F64','.U16.F64',
      //    '.F32.F64','.F64.F32',
      //    '.I16','.I32','.I64','.S16','.S32','.S64','.U16','.U32','.U64','.F32','.F64',
      //    'IAD','DBD','FDD','EAD','IAS','DBS','FDS','EAS','IAX','DBX','FDX','EAX',
      //    '.16','.32','.64','.I8','.S8','.U8','.P8',
      //    'EP','SB','BT','SH','IA','IB','DA','DB','FD','FA','ED','EA',
      //    '.8','S','D','E','P','X','R','B','H','T');
      //
      //  postfixsorted : array[1..70] of TOpPostfix = (
      //    PF_F32S32,PF_F32U32,PF_S32F32,PF_U32F32,PF_F64S32,PF_F64U32,PF_S32F64,PF_U32F64,
      //    PF_F32S16,PF_F32U16,PF_S16F32,PF_U16F32,PF_F64S16,PF_F64U16,PF_S16F64,PF_U16F64,
      //    PF_F32F64,PF_F64F32,
      //    PF_I16,PF_I32,
      //    PF_I64,PF_S16,PF_S32,PF_S64,PF_U16,PF_U32,PF_U64,PF_F32,
      //    PF_F64,PF_IAD,PF_DBD,PF_FDD,PF_EAD,
      //    PF_IAS,PF_DBS,PF_FDS,PF_EAS,PF_IAX,
      //    PF_DBX,PF_FDX,PF_EAX,PF_16,PF_32,
      //    PF_64,PF_I8,PF_S8,PF_U8,PF_P8,
      //    PF_EP,PF_SB,PF_BT,PF_SH,PF_IA,
      //    PF_IB,PF_DA,PF_DB,PF_FD,PF_FA,
      //    PF_ED,PF_EA,PF_8,PF_S,PF_D,PF_E,
      //    PF_P,PF_X,PF_R,PF_B,PF_H,PF_T);

      var
        j, j2 : longint;
        hs,hs2 : string;
        maxlen : longint;
        icond : tasmcond;
      Begin
        { making s a value parameter would break other assembler readers }
        hs:=s;
        is_asmopcode:=false;

        { clear op code }
        actopcode:=A_None;

        actcondition:=C_None;

        if hs[1]='B' then
          begin
            for icond:=low(tasmcond) to high(tasmcond) do
              begin
                if copy(hs,2,length(hs)-1)=uppercond2str[icond] then
                  begin
                    actopcode:=A_Bcc;
                    actasmtoken:=AS_OPCODE;
                    actcondition:=icond;
                    is_asmopcode:=true;
                    exit;
                  end;
              end;
          end;
        maxlen:=min(length(hs),7);
        actopcode:=A_NONE;
        j2:=maxlen;
        hs2:=hs;
        while j2>=1 do
          begin
            hs:=hs2;
            while j2>=1 do
              begin
                actopcode:=tasmop(PtrUInt(iasmops.Find(copy(hs,1,j2))));
                if actopcode<>A_NONE then
                  begin
                    actasmtoken:=AS_OPCODE;
                    { strip op code }
                    delete(hs,1,j2);
                    dec(j2);
                    break;
                  end;
                dec(j2);
              end;

            if actopcode=A_NONE then
              exit;

              begin
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
                { check for postfix }
                //if (length(hs)>0) and (actoppostfix=PF_None) then
                //  begin
                //    for j:=low(postfixsorted) to high(postfixsorted) do
                //      begin
                //        if copy(hs,1,length(postfix2strsorted[j]))=postfix2strsorted[j] then
                //          begin
                //            actoppostfix:=postfixsorted[j];
                //            { strip postfix }
                //            delete(hs,1,length(postfix2strsorted[j]));
                //            break;
                //          end;
                //      end;
                //  end;
              end;
            { check for format postfix }
            //if length(hs)>0 then
            //  begin
            //    if copy(hs,1,2) = '.W' then
            //      begin
            //        actwideformat:=true;
            //        delete(hs,1,2);
            //      end;
            //  end;
            { if we stripped all postfixes, it's a valid opcode }
            is_asmopcode:=length(hs)=0;
            if is_asmopcode = true then
              break;
          end;
      end;


    procedure txtensaattreader.ConvertCalljmp(instr : TXtensaInstruction);
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


    procedure txtensaattreader.HandleTargetDirective;
      var
        symname,
        symval  : String;
        val     : tcgint;
        symtyp  : TAsmsymtype;
      begin
        case actasmpattern of
          '.thumb_set':
            begin
              consume(AS_TARGET_DIRECTIVE);
              BuildConstSymbolExpression(true,false,false, val,symname,symtyp);
              Consume(AS_COMMA);
              BuildConstSymbolExpression(true,false,false, val,symval,symtyp);

              curList.concat(tai_symbolpair.create(spk_thumb_set,symname,symval));
            end;
          '.code':
            begin
              consume(AS_TARGET_DIRECTIVE);
              val:=BuildConstExpression(false,false);
              if not(val in [16,32]) then
                Message(asmr_e_invalid_code_value);
              curList.concat(tai_directive.create(asd_code,tostr(val)));
            end;
          '.thumb_func':
            begin
              consume(AS_TARGET_DIRECTIVE);
              curList.concat(tai_directive.create(asd_thumb_func,''));
            end
          else
            inherited HandleTargetDirective;
        end;
      end;


    procedure txtensaattreader.handleopcode;
      var
        instr : TXtensaInstruction;
      begin
        instr:=TXtensaInstruction.Create(TXtensaOperand);
        BuildOpcode(instr);
        if is_calljmp(instr.opcode) then
          ConvertCalljmp(instr);
        {
        instr.AddReferenceSizes;
        instr.SetInstructionOpsize;
        instr.CheckOperandSizes;
        }
        instr.ConcatInstruction(curlist);
        instr.Free;
//        actoppostfix:=PF_None;
//        actwideformat:=false;
      end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

const
  asmmode_xtensa_att_info : tasmmodeinfo =
          (
            id    : asmmode_xtensa_gas;
            idtxt : 'DIVIDED';
            casmreader : txtensaattreader;
          );

  asmmode_xtensa_standard_info : tasmmodeinfo =
          (
            id    : asmmode_standard;
            idtxt : 'STANDARD';
            casmreader : txtensaattreader;
          );

initialization
  RegisterAsmMode(asmmode_xtensa_att_info);
  RegisterAsmMode(asmmode_xtensa_standard_info);
end.

