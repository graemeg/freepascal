{
    Copyright (c) 1998-2002 by Carl Eric Codere and Peter Vreman

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
Unit raarmgas;

{$i fpcdefs.inc}

  Interface

    uses
      raatt,raarm,
      cpubase;

    type

      tarmattreader = class(tattreader)
        actoppostfix : TOpPostfix;
        actwideformat : boolean;
        function is_asmopcode(const s: string):boolean;override;
        function is_register(const s:string):boolean;override;
        function is_targetdirective(const s: string): boolean; override;
        procedure handleopcode;override;
        procedure BuildReference(oper : tarmoperand);
        procedure BuildOperand(oper : tarmoperand);
        procedure BuildSpecialreg(oper : tarmoperand);
        function TryBuildShifterOp(oper : tarmoperand) : boolean;
        procedure BuildOpCode(instr : tarminstruction);
        procedure ReadSym(oper : tarmoperand);
        procedure ConvertCalljmp(instr : tarminstruction);
        procedure HandleTargetDirective; override;
      protected
        function is_unified: boolean; virtual;
      end;

      tarmunifiedattreader = class(tarmattreader)
      protected
        function is_unified: boolean; override;
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


    function tarmunifiedattreader.is_unified: boolean;
      begin
        result:=true;
      end;


    function tarmattreader.is_register(const s:string):boolean;
      type
        treg2str = record
          name : string[3];
          reg : tregister;
        end;

      const
        extraregs : array[0..19+16] of treg2str = (
          (name: 'A1'; reg : NR_R0),
          (name: 'A2'; reg : NR_R1),
          (name: 'A3'; reg : NR_R2),
          (name: 'A4'; reg : NR_R3),
          (name: 'V1'; reg : NR_R4),
          (name: 'V2'; reg : NR_R5),
          (name: 'V3'; reg : NR_R6),
          (name: 'V4'; reg : NR_R7),
          (name: 'V5'; reg : NR_R8),
          (name: 'V6'; reg : NR_R9),
          (name: 'V7'; reg : NR_R10),
          (name: 'V8'; reg : NR_R11),
          (name: 'WR'; reg : NR_R7),
          (name: 'SB'; reg : NR_R9),
          (name: 'SL'; reg : NR_R10),
          (name: 'FP'; reg : NR_R11),
          (name: 'IP'; reg : NR_R12),
          (name: 'SP'; reg : NR_R13),
          (name: 'LR'; reg : NR_R14),
          (name: 'PC'; reg : NR_R15),

          (name: 'C0'; reg : NR_CR0),
          (name: 'C1'; reg : NR_CR1),
          (name: 'C2'; reg : NR_CR2),
          (name: 'C3'; reg : NR_CR3),
          (name: 'C4'; reg : NR_CR4),
          (name: 'C5'; reg : NR_CR5),
          (name: 'C6'; reg : NR_CR6),
          (name: 'C7'; reg : NR_CR7),
          (name: 'C8'; reg : NR_CR8),
          (name: 'C9'; reg : NR_CR9),
          (name: 'C10'; reg : NR_CR10),
          (name: 'C11'; reg : NR_CR11),
          (name: 'C12'; reg : NR_CR12),
          (name: 'C13'; reg : NR_CR13),
          (name: 'C14'; reg : NR_CR14),
          (name: 'C15'; reg : NR_CR15)
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


    function tarmattreader.is_targetdirective(const s: string): boolean;
      begin
        case s of
          '.force_thumb',
          '.thumb_func',
          '.code',
          '.thumb_set':
            result:=true
          else
            Result:=inherited is_targetdirective(s);
        end;
      end;


    procedure tarmattreader.ReadSym(oper : tarmoperand);
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


    Procedure tarmattreader.BuildReference(oper : tarmoperand);

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
            if (actasmtoken=AS_NOT) then
              begin
                oper.opr.ref.addressmode:=AM_PREINDEXED;
                Consume(AS_NOT);
              end;
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


      function is_shifter_ref_operation(var a : tshiftmode) : boolean;
        begin
          a := SM_NONE;
          if      (actasmpattern='LSL') then
            a := SM_LSL
          else if (actasmpattern='LSR') then
            a := SM_LSR
          else if (actasmpattern='ASR') then
            a := SM_ASR
          else if (actasmpattern='ROR') then
            a := SM_ROR
          else if (actasmpattern='RRX') then
            a := SM_RRX;
          is_shifter_ref_operation := not(a=SM_NONE);
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
                if is_shifter_ref_operation(oper.opr.ref.shiftmode) then
                  begin
                    Consume(AS_ID);
                    if not(oper.opr.ref.shiftmode=SM_RRX) then
                      begin
                        if not(actasmtoken=AS_HASH) then
                          do_error;
                        Consume(AS_HASH);
                        shift := BuildConstExpression(false,true);
                        if (shift<0) or (shift>32) then
                          do_error;
                        oper.opr.ref.shiftimm := shift;
                        test_end(require_rbracket);
                      end
                    else
                      test_end(require_rbracket);
                   end
                 else
                   begin
                     do_error;
                     exit;
                   end;
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
                  end
                else
                  begin
                    oper.opr.ref.signindex := -1;
                    Consume(AS_MINUS);
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
                oper.opr.ref.signindex := BuildConstExpression(true,false);
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
                oper.opr.ref.addressmode:=AM_POSTINDEXED;
                read_index(false);
                exit;
              end;
            AS_NOT :
              begin   // pre-indexed
                Consume(AS_NOT);
                oper.opr.ref.addressmode:=AM_PREINDEXED;
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
        oper.opr.ref.addressmode:=AM_OFFSET; // assume "neither PRE nor POST inc"
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
                  if is_locallabel(actasmpattern) then
                    begin
                      CreateLocalLabel(actasmpattern,lab,false);
                      oper.opr.ref.symbol := lab;
                      oper.opr.ref.base := NR_PC;
                      Consume(AS_ID);
                      test_end(true);
                      exit;
                    end
                  else
                    begin
                      // TODO: Stackpointer implied,
                      Message(asmr_e_invalid_reference_syntax);
                      RecoverConsume(false);
                      exit;
                    end;
                end;
              else
                begin // elsecase
                  Message(asmr_e_invalid_reference_syntax);
                  RecoverConsume(false);
                  exit;
                end;
            end;
          end;
      end;


    function tarmattreader.TryBuildShifterOp(oper : tarmoperand) : boolean;

      procedure handlepara(sm : tshiftmode);
        begin
          consume(AS_ID);
          fillchar(oper.opr,sizeof(oper.opr),0);
          oper.opr.typ:=OPR_SHIFTEROP;
          oper.opr.shifterop.shiftmode:=sm;
          if sm<>SM_RRX then
            begin
              case actasmtoken of
                AS_REGISTER:
                  begin
                    oper.opr.shifterop.rs:=actasmregister;
                    consume(AS_REGISTER);
                  end;
                AS_HASH:
                  begin
                    consume(AS_HASH);
                    oper.opr.shifterop.shiftimm:=BuildConstExpression(false,false);
                  end;
                else
                  Message(asmr_e_illegal_shifterop_syntax);
              end;
            end;
        end;

      begin
        result:=true;
        if (actasmtoken=AS_ID) then
          begin
            if (actasmpattern='LSL') then
              handlepara(SM_LSL)
            else if (actasmpattern='LSR') then
              handlepara(SM_LSR)
            else if (actasmpattern='ASR') then
              handlepara(SM_ASR)
            else if (actasmpattern='ROR') then
              handlepara(SM_ROR)
            else if (actasmpattern='RRX') then
              handlepara(SM_RRX)
            else
              result:=false;
          end
        else
          result:=false;
      end;


    Procedure tarmattreader.BuildOperand(oper : tarmoperand);
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
               oper.opr.ref.base:=NR_PC;
               if (actasmtoken in [AS_PLUS, AS_MINUS]) then
                 begin
                   l:=BuildConstExpression(true,false);
                   oper.opr.ref.offset:=l;
                 end;
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


        function is_ConditionCode(hs: string): boolean;
          var icond: tasmcond;
          begin
            is_ConditionCode := false;

            case actopcode of
              A_IT,A_ITE,A_ITT,
              A_ITEE,A_ITTE,A_ITET,A_ITTT,
              A_ITEEE,A_ITTEE,A_ITETE,A_ITTTE,A_ITEET,A_ITTET,A_ITETT,A_ITTTT:
                begin
                  { search for condition, conditions are always 2 chars }
                  if length(hs)>1 then
                    begin
                      for icond:=low(tasmcond) to high(tasmcond) do
                        begin
                          if copy(hs,1,2)=uppercond2str[icond] then
                            begin
                              //actcondition:=icond;
                              oper.opr.typ := OPR_COND;
                              oper.opr.cc := icond;
                              exit(true);
                            end;
                        end;
                    end;
                end;
              else
               ;
            end;
          end;


        function is_modeflag(hs : string): boolean;
          var
            i: longint;
            flags: tcpumodeflags;
          begin
            is_modeflag := false;

            flags:=[];
            hs:=lower(hs);

            if (actopcode in [A_CPSID,A_CPSIE]) and (length(hs) >= 1) then
              begin
                for i:=1 to length(hs) do
                  begin
                    case hs[i] of
                      'a':
                        Include(flags,mfA);
                      'f':
                        Include(flags,mfF);
                      'i':
                        Include(flags,mfI);
                    else
                      exit;
                    end;
                  end;
                oper.opr.typ := OPR_MODEFLAGS;
                oper.opr.flags := flags;
                exit(true);
              end;
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
                  oper.opr.ref.base:=NR_PC;
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

          AS_HASH: { Constant expression  }
            Begin
              Consume(AS_HASH);
              BuildConstantOperand(oper);
            end;

          AS_EQUAL:
            begin
              case actopcode of
                A_LDRBT,A_LDRB,A_LDR,A_LDRH,A_LDRSB,A_LDRSH,A_LDRT,
                A_LDREX,A_LDREXB,A_LDREXD,A_LDREXH:
                  begin
                    consume(AS_EQUAL);
                    oper.InitRef;
                    BuildDirectRef;
                  end;
              else
                Message(asmr_e_invalid_opcode_and_operand);
              end;
            end;

          (*
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
          *)
          AS_ID: { A constant expression, or a Variable ref.  }
            Begin
              if is_modeflag(actasmpattern) then
                begin
                  consume(AS_ID);
                end
              else
              { Condition code? }
              if is_conditioncode(actasmpattern) then
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
                         internalerror(2003092021);
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
              else if (actasmtoken=AS_NOT) and (actopcode in [A_LDM,A_STM,A_FLDM,A_FSTM,A_VLDM,A_VSTM,A_SRS,A_RFE]) then
                begin
                  consume(AS_NOT);
                  oper.opr.typ:=OPR_REFERENCE;
                  oper.opr.ref.addressmode:=AM_PREINDEXED;
                  oper.opr.ref.index:=tempreg;
                end
              else
                Message(asmr_e_syn_operand);
            end;

          { Registerset }
          AS_LSBRACKET:
            begin
              consume(AS_LSBRACKET);
              registerset:=[];
              regtype:=R_INVALIDREGISTER;
              subreg:=R_SUBNONE;
              while actasmtoken<>AS_RSBRACKET do
                begin
                  if actasmtoken=AS_REGISTER then
                    begin
                      include(registerset,getregsetindex(actasmregister));
                      if regtype<>R_INVALIDREGISTER then
                        begin
                          if (getregtype(actasmregister)<>regtype) or
                             (getsubreg(actasmregister)<>subreg) then
                            Message(asmr_e_mixing_regtypes);
                        end
                      else
                        begin
                          regtype:=getregtype(actasmregister);
                          subreg:=getsubreg(actasmregister);
                        end;
                      tempreg:=actasmregister;
                      consume(AS_REGISTER);
                      if actasmtoken=AS_MINUS then
                        begin
                          consume(AS_MINUS);
                          for ireg:=getregsetindex(tempreg) to getregsetindex(actasmregister) do
                            include(registerset,ireg);
                          consume(AS_REGISTER);
                        end;
                    end
                  else
                    consume(AS_REGISTER);
                  if actasmtoken=AS_COMMA then
                    consume(AS_COMMA)
                  else
                    break;
                end;
              consume(AS_RSBRACKET);
              oper.opr.typ:=OPR_REGSET;
              oper.opr.regtype:=regtype;
              oper.opr.subreg:=subreg;
              oper.opr.regset:=registerset;
              if actasmtoken=AS_XOR then
                begin
                  consume(AS_XOR);
                  oper.opr.usermode:=true;
                end
              else
                oper.opr.usermode:=false;
              if (registerset=[]) then
                Message(asmr_e_empty_regset);
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

    procedure tarmattreader.BuildSpecialreg(oper: tarmoperand);
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
          AS_ID:
            begin
              t := pos('_', actasmpattern);
              if t > 0 then
                begin
                  hs:=lower(actasmpattern);
                  reg:=copy(hs, 1, t-1);
                  delete(hs, 1, t);

                  if length(hs) < 1 then
                    Message(asmr_e_invalid_operand_type);

                  if reg = 'cpsr' then
                    hreg:=NR_CPSR
                  else if reg='spsr' then
                    hreg:=NR_SPSR
                  else
                    Message(asmr_e_invalid_register);

                  flags:=[];
                  for i := 1 to length(hs) do
                    begin
                      ch:=hs[i];
                      if ch='c' then
                        include(flags, srC)
                      else if ch='x' then
                        include(flags, srX)
                      else if ch='f' then
                        include(flags, srF)
                      else if ch='s' then
                        include(flags, srS)
                      else
                        message(asmr_e_invalid_operand_type);
                    end;

                  oper.opr.typ:=OPR_SPECIALREG;
                  oper.opr.specialreg:=hreg;
                  oper.opr.specialregflags:=flags;

                  consume(AS_ID);
                end
              else
                Message(asmr_e_invalid_operand_type); // Otherwise it would have been seen as a AS_REGISTER
            end;
          else
            Message(asmr_e_invalid_operand_type);
        end;
      end;


{*****************************************************************************
                                tarmattreader
*****************************************************************************}

    procedure tarmattreader.BuildOpCode(instr : tarminstruction);
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
            oppostfix:=actoppostfix;
            wideformat:=actwideformat;
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
                if ((instr.opcode in [A_MOV,A_MVN,A_CMP,A_CMN,A_TST,A_TEQ,
                                      A_UXTB,A_UXTH,A_UXTB16,
                                      A_SXTB,A_SXTH,A_SXTB16]) and
                    (operandnum=2)) or
                  ((operandnum=3) and not(instr.opcode in [A_UMLAL,A_UMULL,A_SMLAL,A_SMULL,A_MLA,A_UMAAL,A_MLS,
                                                           A_SMLABB,A_SMLABT,A_SMLATB,A_SMLATT,A_SMMLA,A_SMMLS,A_SMLAD,A_SMLALD,A_SMLSD,
                                                           A_SMLALBB,A_SMLALBT,A_SMLALTB,A_SMLALTT,A_SMLSLD,
                                                           A_SMLAWB,A_SMLAWT,
                                                           A_MRC,A_MCR,A_MCRR,A_MRRC,A_MRC2,A_MCR2,A_MCRR2,A_MRRC2,
                                                           A_STREXD,A_STRD,
                                                           A_USADA8,
                                                           A_VMOV,
                                                           A_SBFX,A_UBFX,A_BFI])) then
                  begin
                    Consume(AS_COMMA);
                    if not(TryBuildShifterOp(instr.Operands[operandnum+1] as tarmoperand)) then
                      Message(asmr_e_illegal_shifterop_syntax);
                    Inc(operandnum);
                  end
                else
                  begin
                    if operandnum>Max_Operands then
                      Message(asmr_e_too_many_operands)
                    else
                      Inc(operandnum);
                    Consume(AS_COMMA);
                  end;
              end;
            AS_SEPARATOR,
            AS_end : { End of asm operands for this opcode  }
              begin
                break;
              end;
          else
            if ((instr.opcode = A_MRS) and (operandnum = 2)) or
               ((instr.opcode = A_MSR) and (operandnum = 1)) then
              BuildSpecialreg(instr.Operands[operandnum] as tarmoperand)
            else
              BuildOperand(instr.Operands[operandnum] as tarmoperand);
          end; { end case }
        until false;
        instr.Ops:=operandnum;
      end;


    function tarmattreader.is_asmopcode(const s: string):boolean;

      const
        { sorted by length so longer postfixes will match first }
        postfix2strsorted : array[1..70] of string[9] = (
          '.F32.S32','.F32.U32','.S32.F32','.U32.F32','.F64.S32','.F64.U32','.S32.F64','.U32.F64',
          '.F32.S16','.F32.U16','.S16.F32','.U16.F32','.F64.S16','.F64.U16','.S16.F64','.U16.F64',
          '.F32.F64','.F64.F32',
          '.I16','.I32','.I64','.S16','.S32','.S64','.U16','.U32','.U64','.F32','.F64',
          'IAD','DBD','FDD','EAD','IAS','DBS','FDS','EAS','IAX','DBX','FDX','EAX',
          '.16','.32','.64','.I8','.S8','.U8','.P8',
          'EP','SB','BT','SH','IA','IB','DA','DB','FD','FA','ED','EA',
          '.8','S','D','E','P','X','R','B','H','T');

        postfixsorted : array[1..70] of TOpPostfix = (
          PF_F32S32,PF_F32U32,PF_S32F32,PF_U32F32,PF_F64S32,PF_F64U32,PF_S32F64,PF_U32F64,
          PF_F32S16,PF_F32U16,PF_S16F32,PF_U16F32,PF_F64S16,PF_F64U16,PF_S16F64,PF_U16F64,
          PF_F32F64,PF_F64F32,
          PF_I16,PF_I32,
          PF_I64,PF_S16,PF_S32,PF_S64,PF_U16,PF_U32,PF_U64,PF_F32,
          PF_F64,PF_IAD,PF_DBD,PF_FDD,PF_EAD,
          PF_IAS,PF_DBS,PF_FDS,PF_EAS,PF_IAX,
          PF_DBX,PF_FDX,PF_EAX,PF_16,PF_32,
          PF_64,PF_I8,PF_S8,PF_U8,PF_P8,
          PF_EP,PF_SB,PF_BT,PF_SH,PF_IA,
          PF_IB,PF_DA,PF_DB,PF_FD,PF_FA,
          PF_ED,PF_EA,PF_8,PF_S,PF_D,PF_E,
          PF_P,PF_X,PF_R,PF_B,PF_H,PF_T);

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

        { first, handle B else BLS is read wrong }
        if ((hs[1]='B') and (length(hs)=3)) then
          begin
            for icond:=low(tasmcond) to high(tasmcond) do
              begin
                if copy(hs,2,3)=uppercond2str[icond] then
                  begin
                    actopcode:=A_B;
                    actasmtoken:=AS_OPCODE;
                    actcondition:=icond;
                    is_asmopcode:=true;
                    exit;
                  end;
              end;
          end;
        maxlen:=min(length(hs),6);
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

            if is_unified then
              begin
                { check for postfix }
                if (length(hs)>0) and (actoppostfix=PF_None) then
                  begin
                    for j:=low(postfixsorted) to high(postfixsorted) do
                      begin
                        if copy(hs,1,length(postfix2strsorted[j]))=postfix2strsorted[j] then
                          begin
                            if not ((length(hs)-length(postfix2strsorted[j])) in [0,2,4]) then
                              continue;

                            actoppostfix:=postfixsorted[j];
                            { strip postfix }
                            delete(hs,1,length(postfix2strsorted[j]));
                            break;
                          end;
                      end;
                  end;
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
                if (length(hs)>0) and (actoppostfix=PF_None) then
                  begin
                    for j:=low(postfixsorted) to high(postfixsorted) do
                      begin
                        if copy(hs,1,length(postfix2strsorted[j]))=postfix2strsorted[j] then
                          begin
                            if not ((length(hs)-length(postfix2strsorted[j])) = 0) then
                              continue;

                            actoppostfix:=postfixsorted[j];
                            { strip postfix }
                            delete(hs,1,length(postfix2strsorted[j]));
                            break;
                          end;
                      end;
                  end;
              end
            else
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
                if (length(hs)>0) and (actoppostfix=PF_None) then
                  begin
                    for j:=low(postfixsorted) to high(postfixsorted) do
                      begin
                        if copy(hs,1,length(postfix2strsorted[j]))=postfix2strsorted[j] then
                          begin
                            actoppostfix:=postfixsorted[j];
                            { strip postfix }
                            delete(hs,1,length(postfix2strsorted[j]));
                            break;
                          end;
                      end;
                  end;
              end;
            { check for format postfix }
            if length(hs)>0 then
              begin
                if copy(hs,1,2) = '.W' then
                  begin
                    actwideformat:=true;
                    delete(hs,1,2);
                  end;
              end;
            { if we stripped all postfixes, it's a valid opcode }
            is_asmopcode:=length(hs)=0;
            if is_asmopcode = true then
              break;
          end;
      end;


    procedure tarmattreader.ConvertCalljmp(instr : tarminstruction);
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


    procedure tarmattreader.HandleTargetDirective;
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
            end;
          '.force_thumb':
            begin
              consume(AS_TARGET_DIRECTIVE);
              curList.concat(tai_directive.create(asd_force_thumb,''));
            end
          else
            inherited HandleTargetDirective;
        end;
      end;


    function tarmattreader.is_unified: boolean;
      begin
        result:=false;
      end;


    procedure tarmattreader.handleopcode;
      var
        instr : tarminstruction;
      begin
        instr:=TarmInstruction.Create(TarmOperand);
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
        actoppostfix:=PF_None;
        actwideformat:=false;
      end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

const
  asmmode_arm_att_info : tasmmodeinfo =
          (
            id    : asmmode_arm_gas;
            idtxt : 'DIVIDED';
            casmreader : tarmattreader;
          );

  asmmode_arm_att_unified_info : tasmmodeinfo =
          (
            id    : asmmode_arm_gas_unified;
            idtxt : 'UNIFIED';
            casmreader : tarmunifiedattreader;
          );

  asmmode_arm_standard_info : tasmmodeinfo =
          (
            id    : asmmode_standard;
            idtxt : 'STANDARD';
            casmreader : tarmattreader;
          );

initialization
  RegisterAsmMode(asmmode_arm_att_info);
  RegisterAsmMode(asmmode_arm_att_unified_info);
  RegisterAsmMode(asmmode_arm_standard_info);
end.
