{
    Copyright (c) 1998-2000 by Carl Eric Codere

    This unit does the parsing process for the motorola inline assembler

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
unit ra68kmot;

{$i fpcdefs.inc}

{**********************************************************************}
{ WARNING                                                              }
{**********************************************************************}
{  Any modification in the order or removal of terms in the tables     }
{  in m68k.pas and asmo68k.pas  will BREAK the code in this unit,      }
{  unless the appropriate changes are made to this unit. Addition      }
{  of terms though, will not change the code herein.                   }
{**********************************************************************}

{---------------------------------------------------------------------------}
{ LEFT TO DO                                                                }
{---------------------------------------------------------------------------}
{  o Add support for sized indexing such as in d0.l                         }
{      presently only (an,dn) is supported for indexing --                  }
{        size defaults to LONG.                                             }
{  o Add support for MC68020 opcodes.                                       }
{  o Add support for MC68020 adressing modes.                               }
{  o Add operand checking with m68k opcode table in ConcatOpCode            }
{  o Add Floating point support                                             }
{---------------------------------------------------------------------------}

  interface


    uses
      cutils,
      globtype,cclasses,cpubase,
      symconst,
      aasmbase,
      rabase,rasm,ra68k,rautils;

    type
      tasmtoken = (
        AS_NONE,AS_LABEL,AS_LLABEL,AS_STRING,AS_HEXNUM,AS_OCTALNUM,
        AS_BINNUM,AS_COMMA,AS_LBRACKET,AS_RBRACKET,AS_LPAREN,
        AS_RPAREN,AS_COLON,AS_DOT,AS_PLUS,AS_MINUS,AS_STAR,AS_INTNUM,
        AS_SEPARATOR,AS_ID,AS_REGISTER,AS_OPCODE,AS_SLASH,AS_APPT,AS_REALNUM,
        AS_ALIGN,
          {------------------ Assembler directives --------------------}
        AS_DB,AS_DW,AS_DD,AS_XDEF,AS_END,
          {------------------ Assembler Operators  --------------------}
        AS_MOD,AS_SHL,AS_SHR,AS_NOT,AS_AND,AS_OR,AS_XOR);

      tasmtokenset = set of tasmtoken;
      tasmkeyword = string[10];

      tm68kmotreader = class(tasmreader)
         actasmtoken    : tasmtoken;
         prevasmtoken   : tasmtoken;
         procedure SetupTables;
         function Assemble: tlinkedlist;override;
         function is_asmopcode(const s: string) : boolean;
         Function is_asmdirective(const s: string):boolean;
         function is_register(const s:string):boolean;
         procedure GetToken;
         function consume(t : tasmtoken):boolean;
         function try_to_consume(t : tasmtoken):boolean;
         procedure consume_all_until(tokens : tasmtokenset);
         function findopcode(const s: string; var opsize: topsize): tasmop;
         Function BuildExpression(allow_symbol : boolean; asmsym : pshortstring) : tcgint;
         Procedure BuildConstant(constsize: tcgint);
         Procedure BuildRealConstant(typ : tfloattype);
         Procedure BuildScaling(const oper:tm68koperand);
         Function BuildRefExpression: tcgint;
         procedure BuildReference(const oper:tm68koperand);
         procedure BuildRegList(const oper:tm68koperand);
         procedure BuildRegPair(const oper:tm68koperand);
         Procedure BuildOperand(const oper:tm68koperand);
         Procedure BuildOpCode(instr:Tm68kinstruction);
      end;


Implementation

    uses
       { global }
       globals,verbose,
       systems,
       { aasm }
       cpuinfo,aasmtai,aasmdata,aasmcpu,
       cgbase,cgutils,
       { symtable }
       symbase,symtype,symsym,symdef,symtable,
       { pass 1 }
       nbas,
       { parser }
       scanner,ag68kgas,
       itcpugas
       ;

const
   firstdirective = AS_DB;
   lastdirective  = AS_END;
   firstoperator  = AS_MOD;
   lastoperator   = AS_XOR;

   _count_asmdirectives = longint(lastdirective)-longint(firstdirective);
   _count_asmoperators  = longint(lastoperator)-longint(firstoperator);

   _asmdirectives : array[0.._count_asmdirectives] of tasmkeyword =
    ('DC.B','DC.W','DC.L','XDEF','END');

    { problems with shl,shr,not,and,or and xor, they are }
    { context sensitive.                                 }
    _asmoperators : array[0.._count_asmoperators] of tasmkeyword = (
    'MOD','SHL','SHR','NOT','AND','OR','XOR');

   token2str : array[tasmtoken] of tasmkeyword=(
        'NONE','LABEL','LLABEL','STRING','HEXNUM','OCTALNUM',
        'BINNUM',',','[',']','(',
        ')',':','.','+','-','*','INTNUM',
        'SEPARATOR','ID','REGISTER','OPCODE','/','APPT','REALNUM',
        'ALIGN',
          {------------------ Assembler directives --------------------}
        'DB','DW','DD','XDEF','END',
          {------------------ Assembler Operators  --------------------}
        'MOD','SHL','SHR','NOT','AND','OR','XOR');

const
  firsttoken : boolean = TRUE;
  operandnum : byte = 0;

    procedure tm68kmotreader.SetupTables;
      { creates uppercased symbol tables for speed access }
      var
        i : tasmop;
      Begin
        { opcodes }
        iasmops:=TFPHashList.create;
        for i:=firstop to lastop do
          iasmops.Add(upper(gas_op2str[i]),Pointer(PtrInt(i)));
      end;


  {---------------------------------------------------------------------}
  {                     Routines for the tokenizing                     }
  {---------------------------------------------------------------------}

    function tm68kmotreader.is_asmopcode(const s: string):boolean;
      var
        hs : string;
        j : byte;
      begin
        is_asmopcode:=false;
        { first of all we remove the suffix }
        j:=pos('.',s);
        if j>0 then
          hs:=copy(s,1,j-1)
        else
          hs:=s;

        { Search opcodes }
        actopcode:=tasmop(PtrUInt(iasmops.Find(hs)));
        { Also filter the helper opcodes, they can't be valid
          while reading an assembly source }
        case actopcode of
          A_NONE, A_DBXX, A_SXX, A_BXX, A_FBXX:
            begin
            end;
          else
            begin
              actasmtoken:=AS_OPCODE;
              result:=TRUE;
              exit;
            end;
          end;
      end;



   Function tm68kmotreader.is_asmdirective(const s: string):boolean;
   var
    i:byte;
   begin
     result:=false;
     for i:=0 to _count_asmdirectives do
     begin
        if s=_asmdirectives[i] then
        begin
           actasmtoken := tasmtoken(longint(firstdirective)+i);
           result:=true;
           exit;
        end;
     end;
   end;


    function tm68kmotreader.is_register(const s:string):boolean;
      begin
        result:=false;
        actasmregister:=std_regnum_search(lower(s));
        if actasmregister<>NR_NO then
          begin
            result:=true;
            actasmtoken:=AS_REGISTER;
          end;
        { reg found?
          possible aliases are always 2 char
        }
        if result or (length(s)<>2) then
          exit;
        if lower(s)='sp' then
          actasmregister:=NR_STACK_POINTER_REG;
        if lower(s)='fp' then
          actasmregister:=NR_FRAME_POINTER_REG;
        if actasmregister<>NR_NO then
          begin
            result:=true;
            actasmtoken:=AS_REGISTER;
          end;
      end;


  Procedure tm68kmotreader.GetToken;
  {*********************************************************************}
  { FUNCTION GetToken: tinteltoken;                                     }
  {  Description: This routine returns intel assembler tokens and       }
  {  does some minor syntax error checking.                             }
  {*********************************************************************}
  var
   token: tasmtoken;
   forcelabel: boolean;
  begin
    c:=scanner.c;
    forcelabel := FALSE;
    actasmpattern :='';
    {* INIT TOKEN TO NOTHING *}
    token := AS_NONE;
    { while space and tab , continue scan... }
    while c in [' ',#9] do
     c:=current_scanner.asmgetchar;

    if not (c in [#10,#13,'{',';','(','/']) then
     current_scanner.gettokenpos;
    { Possiblities for first token in a statement:                }
    {   Local Label, Label, Directive, Prefix or Opcode....       }
    if firsttoken and not (c in [#10,#13,'{',';','(','/']) then
    begin

      firsttoken := FALSE;
      if c = '@' then
      begin
        token := AS_LLABEL;   { this is a local label }
        { Let us point to the next character }
        c := current_scanner.asmgetchar;
      end;



      while c in ['A'..'Z','a'..'z','0'..'9','_','@','.'] do
      begin
         { if there is an at_sign, then this must absolutely be a label }
         if c = '@' then forcelabel:=TRUE;
         actasmpattern := actasmpattern + c;
         c := current_scanner.asmgetchar;
      end;

      uppervar(actasmpattern);

      if c = ':' then
      begin
           if token = AS_NONE then
             token := AS_LABEL;
           { let us point to the next character }
           c := current_scanner.asmgetchar;
           actasmtoken := token;
           exit;
      end;

      { Are we trying to create an identifier with }
      { an at-sign...?                             }
      if forcelabel then
       Message(asmr_e_none_label_contain_at);

      If is_asmopcode(actasmpattern) then
       exit;
      if is_asmdirective(actasmpattern) then
        exit
      else
        begin
          actasmtoken := AS_NONE;
          Message1(asmr_e_invalid_or_missing_opcode,actasmpattern);
        end;
    end
    else { else firsttoken }
    { Here we must handle all possible cases                              }
    begin
      case c of

         '@':   { possiblities : - local label reference , such as in jmp @local1 }
                {                - @Result, @Code or @Data special variables.     }
                            begin
                             actasmpattern := c;
                             c:= current_scanner.asmgetchar;
                             while c in  ['A'..'Z','a'..'z','0'..'9','_','@','.'] do
                             begin
                               actasmpattern := actasmpattern + c;
                               c := current_scanner.asmgetchar;
                             end;
                             uppervar(actasmpattern);
                             actasmtoken := AS_ID;
                             exit;
                            end;
      { identifier, register, opcode, prefix or directive }
         'A'..'Z','a'..'z','_': begin
                             actasmpattern := c;
                             c:= current_scanner.asmgetchar;
                             while c in  ['A'..'Z','a'..'z','0'..'9','_','.'] do
                             begin
                               actasmpattern := actasmpattern + c;
                               c := current_scanner.asmgetchar;
                             end;
                             uppervar(actasmpattern);

                             { this isn't the first token, so it can't be an
                               opcode }
                             { Actually, it's possible, since @label: OPCODE foo,bar
                               is valid and was supported in 0.99/1.0 FPC for 68k,
                               the amunits package is full of such code. (KB) }
                             if is_asmopcode(actasmpattern) then
                               exit;
                             if is_register(actasmpattern) then
                               exit;
                             if is_asmdirective(actasmpattern) then
                               exit;
                             { this is surely an identifier }
                             actasmtoken := AS_ID;
                             exit;
                          end;
           { override operator... not supported }
           '&':       begin
                         c:=current_scanner.asmgetchar;
                         actasmtoken := AS_AND;
                      end;
           { string or character }
           '''' :
                      begin
                         actasmpattern:='';
                         while true do
                         begin
                           if c = '''' then
                           begin
                              c:=current_scanner.asmgetchar;
                              if c=#10 then
                              begin
                                 Message(scan_f_string_exceeds_line);
                                 break;
                              end;
                              repeat
                                  if c=''''then
                                   begin
                                       c:=current_scanner.asmgetchar;
                                       if c='''' then
                                        begin
                                               actasmpattern:=actasmpattern+'''';
                                               c:=current_scanner.asmgetchar;
                                               if c=#10 then
                                               begin
                                                    Message(scan_f_string_exceeds_line);
                                                    break;
                                               end;
                                        end
                                        else break;
                                   end
                                   else
                                   begin
                                          actasmpattern:=actasmpattern+c;
                                          c:=current_scanner.asmgetchar;
                                          if c=#10 then
                                            begin
                                               Message(scan_f_string_exceeds_line);
                                               break
                                            end;
                                   end;
                              until false; { end repeat }
                           end
                           else break; { end if }
                         end; { end while }
                   token:=AS_STRING;
                   actasmtoken := token;
                   exit;
                 end;
           '$' :  begin
                    c:=current_scanner.asmgetchar;
                    while c in ['0'..'9','A'..'F','a'..'f'] do
                    begin
                      actasmpattern := actasmpattern + c;
                      c := current_scanner.asmgetchar;
                    end;
                   actasmtoken := AS_HEXNUM;
                   exit;
                  end;
           ',' : begin
                   actasmtoken := AS_COMMA;
                   c:=current_scanner.asmgetchar;
                   exit;
                 end;
           '(' : begin
                   c:=current_scanner.asmgetchar;
                   if c='*' then
                     begin
                       current_scanner.skipoldtpcomment(true);
                       GetToken;
                     end
                   else
                     actasmtoken:=AS_LPAREN;
                   exit;
                 end;
           ')' : begin
                   actasmtoken := AS_RPAREN;
                   c:=current_scanner.asmgetchar;
                   exit;
                 end;
           ':' : begin
                   actasmtoken := AS_COLON;
                   c:=current_scanner.asmgetchar;
                   exit;
                 end;
{           '.' : begin
                   actasmtoken := AS_DOT;
                   c:=current_scanner.asmgetchar;
                   exit;
                 end; }
           '+' : begin
                   actasmtoken := AS_PLUS;
                   c:=current_scanner.asmgetchar;
                   exit;
                 end;
           '-' : begin
                   actasmtoken := AS_MINUS;
                   c:=current_scanner.asmgetchar;
                   exit;
                 end;
           '*' : begin
                   actasmtoken := AS_STAR;
                   c:=current_scanner.asmgetchar;
                   exit;
                 end;
           '/' : begin
                   c:=current_scanner.asmgetchar;
                   if c='/' then
                     begin
                       current_scanner.skipdelphicomment;
                       GetToken;
                     end
                   else
                     actasmtoken := AS_SLASH;
                   exit;
                 end;
           '<' : begin
                   c := current_scanner.asmgetchar;
                   { invalid characters }
                   if c <> '<' then
                    Message(asmr_e_invalid_char_smaller);
                   { still assume << }
                   actasmtoken := AS_SHL;
                   c := current_scanner.asmgetchar;
                   exit;
                 end;
           '>' : begin
                   c := current_scanner.asmgetchar;
                   { invalid characters }
                   if c <> '>' then
                    Message(asmr_e_invalid_char_greater);
                   { still assume << }
                   actasmtoken := AS_SHR;
                   c := current_scanner.asmgetchar;
                   exit;
                 end;
           '|' : begin
                   actasmtoken := AS_OR;
                   c := current_scanner.asmgetchar;
                   exit;
                 end;
           '^' : begin
                  actasmtoken := AS_XOR;
                  c := current_scanner.asmgetchar;
                  exit;
                 end;
           '#' : begin
                  actasmtoken:=AS_APPT;
                  c:=current_scanner.asmgetchar;
                  exit;
                 end;
           '%' : begin
                   c:=current_scanner.asmgetchar;
                   while c in ['0','1'] do
                   begin
                     actasmpattern := actasmpattern + c;
                     c := current_scanner.asmgetchar;
                   end;
                   actasmtoken := AS_BINNUM;
                   exit;
                 end;
           { integer number }
           '0'..'9': begin
                        actasmpattern := c;
                        c := current_scanner.asmgetchar;
                        while c in ['0'..'9'] do
                          begin
                             actasmpattern := actasmpattern + c;
                             c:= current_scanner.asmgetchar;
                          end;
                        actasmtoken := AS_INTNUM;
                        exit;
                     end;
         ';' : begin
                  repeat
                     c:=current_scanner.asmgetchar;
                  until c=#10;
                  firsttoken := TRUE;
                  actasmtoken:=AS_SEPARATOR;
               end;

         '{' : begin
                 current_scanner.skipcomment(true);
                 GetToken;
               end;

         #13,#10 : begin
                            current_scanner.linebreak;
                            c:=current_scanner.asmgetchar;
                            firsttoken := TRUE;
                            actasmtoken:=AS_SEPARATOR;
                           end;
            else
             current_scanner.illegal_char(c);

      end; { end case }
    end; { end else if }
  end;


  {---------------------------------------------------------------------}
  {                     Routines for the parsing                        }
  {---------------------------------------------------------------------}

    function tm68kmotreader.consume(t : tasmtoken):boolean;
      begin
        Consume:=true;
        if t<>actasmtoken then
         begin
           Message2(scan_f_syn_expected,token2str[t],token2str[actasmtoken]);
           Consume:=false;
         end;
        repeat
          gettoken;
        until actasmtoken<>AS_NONE;
      end;

    function tm68kmotreader.try_to_consume(t : tasmtoken):boolean;
      begin
        try_to_consume:=t=actasmtoken;
        if try_to_consume then
          Consume(actasmtoken);
      end;

    procedure tm68kmotreader.consume_all_until(tokens : tasmtokenset);
      begin
         while not (actasmtoken in tokens) do
           Consume(actasmtoken);
      end;



   function tm68kmotreader.findopcode(const s: string; var opsize: topsize): tasmop;
  {*********************************************************************}
  { FUNCTION findopcode(s: string): tasmop;                             }
  {  Description: Determines if the s string is a valid opcode          }
  {  if so returns correct tasmop token.                                }
  {*********************************************************************}
   var
     j: longint;
   begin
     j:=pos('.',s);
     if (j <> 0) and (j < length(s)) then
     begin
       case s[j+1] of
       { For the motorola only opsize size is used to }
       { determine the size of the operands.          }
       'B': opsize := S_B;
       'W': opsize := S_W;
       'L': opsize := S_L;
       'S': opsize := S_FS;
       'D': opsize := S_FD;
       'X': opsize := S_FX;
       else
         Message1(asmr_e_unknown_opcode,s);
       end;
     end;
     result:=actopcode;
   end;




    Function tm68kmotreader.BuildExpression(allow_symbol : boolean; asmsym : pshortstring) : tcgint;
  {*********************************************************************}
  { FUNCTION BuildExpression: tcgint                                    }
  {  Description: This routine calculates a constant expression to      }
  {  a given value. The return value is the value calculated from       }
  {  the expression.                                                    }
  { The following tokens (not strings) are recognized:                  }
  {    (,),SHL,SHR,/,*,NOT,OR,XOR,AND,MOD,+/-,numbers,ID to constants.  }
  {*********************************************************************}
  { ENTRY: On entry the token should be any valid expression token.     }
  { EXIT:  On Exit the token points to either COMMA or SEPARATOR        }
  { ERROR RECOVERY: Tries to find COMMA or SEPARATOR token by consuming }
  {  invalid tokens.                                                    }
  {*********************************************************************}
  var expr: string;
      hs, tempstr: string;
      sym : tsym;
      srsymtable : TSymtable;
      hl : tasmlabel;
      l : tcgint;
      errorflag: boolean;
  begin
    BuildExpression:=0;
    errorflag := FALSE;
    expr := '';
    tempstr := '';
    if allow_symbol then
      asmsym^:='';
    Repeat
      Case actasmtoken of
      AS_LPAREN: begin
                  Consume(AS_LPAREN);
                  expr := expr + '(';
                end;
      AS_RPAREN: begin
                  Consume(AS_RPAREN);
                  expr := expr + ')';
                end;
      AS_SHL:    begin
                  Consume(AS_SHL);
                  expr := expr + '<';
                end;
      AS_SHR:    begin
                  Consume(AS_SHR);
                  expr := expr + '>';
                end;
      AS_SLASH:  begin
                  Consume(AS_SLASH);
                  expr := expr + '/';
                end;
      AS_MOD:    begin
                  Consume(AS_MOD);
                  expr := expr + '%';
                end;
      AS_STAR:   begin
                  Consume(AS_STAR);
                  expr := expr + '*';
                end;
      AS_PLUS:   begin
                  Consume(AS_PLUS);
                  expr := expr + '+';
                end;
      AS_MINUS:  begin
                  Consume(AS_MINUS);
                  expr := expr + '-';
                end;
      AS_AND:    begin
                  Consume(AS_AND);
                  expr := expr + '&';
                end;
      AS_NOT:    begin
                  Consume(AS_NOT);
                  expr := expr + '~';
                end;
      AS_XOR:    begin
                  Consume(AS_XOR);
                  expr := expr + '^';
                end;
      AS_OR:     begin
                  Consume(AS_OR);
                  expr := expr + '|';
                end;
      AS_ID:    begin
                  if SearchIConstant(actasmpattern,l) then
                  begin
                    str(l, tempstr);
                    expr := expr + tempstr;
                    Consume(AS_ID);
                  End else
                  if not allow_symbol then
                  begin
                    Message(asmr_e_syn_constant);
                    l := 0;
                  End else
                  begin
                    hs:='';
                    if (expr[Length(expr)]='+') then
                      Delete(expr,Length(expr),1)
                    else if expr<>'' then
                      begin
                        Message(asmr_e_invalid_constant_expression);
                        break;
                      End;
                    tempstr:=actasmpattern;
                    consume(AS_ID);
                    if (length(tempstr)>1) and (tempstr[1]='@') then
                      begin
                        CreateLocalLabel(tempstr,hl,false);
                        hs:=hl.name
                      end
                    else if SearchLabel(tempstr,hl,false) then
                      hs:=hl.name
                    else
                      begin
                        asmsearchsym(tempstr,sym,srsymtable);
                        if assigned(sym) then
                         begin
                           case sym.typ of
                             paravarsym,
                             localvarsym :
                               begin
                                 Message(asmr_e_no_local_or_para_allowed);
                                 hs:=tabstractvarsym(sym).mangledname;
                               end;
                             staticvarsym :
                                   hs:=tstaticvarsym(sym).mangledname;
                             procsym :
                               begin
                                 if tprocsym(sym).procdeflist.count>1 then
                                      Message(asmr_w_calling_overload_func);
                                 hs:=tprocdef(tprocsym(sym).procdeflist[0]).mangledname;
                               end;
                             typesym :
                               begin
                                 if not(ttypesym(sym).typedef.typ in [recorddef,objectdef]) then
                                      Message(asmr_e_wrong_sym_type);
                               end;
                             else
                               Message(asmr_e_wrong_sym_type);
                           end;
                        end
                        else
                           Message1(sym_e_unknown_id,tempstr);
                      end;
                     { symbol found? }
                     if hs<>'' then
                      begin
                        if asmsym^='' then
                         asmsym^:=hs
                        else
                         Message(asmr_e_cant_have_multiple_relocatable_symbols);
                      end;
                  end;
                end;
      AS_INTNUM:  begin
                   expr := expr + actasmpattern;
                   Consume(AS_INTNUM);
                  end;
      AS_BINNUM:  begin
                      tempstr := tostr(ParseVal(actasmpattern,2));
                      if tempstr = '' then
                       Message(asmr_e_error_converting_binary);
                      expr:=expr+tempstr;
                      Consume(AS_BINNUM);
                  end;

      AS_HEXNUM: begin
                    tempstr := tostr(ParseVal(actasmpattern,16));
                    if tempstr = '' then
                     Message(asmr_e_error_converting_hexadecimal);
                    expr:=expr+tempstr;
                    Consume(AS_HEXNUM);
                end;
      AS_OCTALNUM: begin
                    tempstr := tostr(ParseVal(actasmpattern,8));
                    if tempstr = '' then
                     Message(asmr_e_error_converting_octal);
                    expr:=expr+tempstr;
                    Consume(AS_OCTALNUM);
                  end;
      { go to next term }
      AS_COMMA: begin
                  if not ErrorFlag then
                    BuildExpression := CalculateExpression(expr)
                  else
                    BuildExpression := 0;
                  Exit;
               end;
      { go to next symbol }
      AS_SEPARATOR: begin
                      if not ErrorFlag then
                        BuildExpression := CalculateExpression(expr)
                      else
                        BuildExpression := 0;
                      Exit;
                   end;
      else
        begin
          { only write error once. }
          if not errorflag then
           Message(asmr_e_invalid_constant_expression);
          { consume tokens until we find COMMA or SEPARATOR }
          Consume(actasmtoken);
          errorflag := TRUE;
        End;
      end;
    Until false;
  end;


  Procedure tm68kmotreader.BuildRealConstant(typ : tfloattype);
  {*********************************************************************}
  { PROCEDURE BuilRealConst                                             }
  {  Description: This routine calculates a constant expression to      }
  {  a given value. The return value is the value calculated from       }
  {  the expression.                                                    }
  { The following tokens (not strings) are recognized:                  }
  {    +/-,numbers and real numbers                                     }
  {*********************************************************************}
  { ENTRY: On entry the token should be any valid expression token.     }
  { EXIT:  On Exit the token points to either COMMA or SEPARATOR        }
  { ERROR RECOVERY: Tries to find COMMA or SEPARATOR token by consuming }
  {  invalid tokens.                                                    }
  {*********************************************************************}
  var expr: string;
      r : extended;
      code : word;
      negativ : boolean;
      errorflag: boolean;
  begin
    errorflag := FALSE;
    Repeat
    negativ:=false;
    expr := '';
    if actasmtoken=AS_PLUS then Consume(AS_PLUS)
    else if actasmtoken=AS_MINUS then
      begin
         negativ:=true;
         consume(AS_MINUS);
      end;
    Case actasmtoken of
      AS_INTNUM:  begin
                   expr := actasmpattern;
                   Consume(AS_INTNUM);
                 end;
      AS_REALNUM:  begin
                   expr := actasmpattern;
                   { in ATT syntax you have 0d in front of the real }
                   { should this be forced ?  yes i think so, as to }
                   { conform to gas as much as possible.            }
                   if (expr[1]='0') and (upper(expr[2])='D') then
                     expr:=copy(expr,3,255);
                   Consume(AS_REALNUM);
                 end;
      AS_BINNUM:  begin
                      { checking for real constants with this should use  }
                      { real DECODING otherwise the compiler will crash!  }
                      Message(asmr_e_invalid_float_expr);
                      expr:='0.0';
                      Consume(AS_BINNUM);
                 end;

      AS_HEXNUM: begin
                      { checking for real constants with this should use  }
                      { real DECODING otherwise the compiler will crash!  }
                    Message(asmr_e_invalid_float_expr);
                    expr:='0.0';
                    Consume(AS_HEXNUM);
                end;
      AS_OCTALNUM: begin
                      { checking for real constants with this should use    }
                      { real DECODING otherwise the compiler will crash!    }
                      { xxxToDec using reals could be a solution, but the   }
                      { problem is that these will crash the m68k compiler  }
                      { when compiling -- because of lack of good fpu       }
                      { support.                                           }
                    Message(asmr_e_invalid_float_expr);
                    expr:='0.0';
                    Consume(AS_OCTALNUM);
                  end;
         else
           begin
             { only write error once. }
             if not errorflag then
              Message(asmr_e_invalid_float_expr);
             { consume tokens until we find COMMA or SEPARATOR }
             Consume(actasmtoken);
             errorflag := TRUE;
           End;

         end;
      { go to next term }
      if (actasmtoken=AS_COMMA) or (actasmtoken=AS_SEPARATOR) then
        begin
          if negativ then expr:='-'+expr;
          val(expr,r,code);
          if code<>0 then
            begin
               r:=0;
               Message(asmr_e_invalid_float_expr);
               ConcatRealConstant(curlist,r,typ);
            End
          else
            begin
              ConcatRealConstant(curlist,r,typ);
            End;
        end
      else
        Message(asmr_e_invalid_float_expr);
    Until actasmtoken=AS_SEPARATOR;
  end;


  procedure tm68kmotreader.BuildConstant(constsize: tcgint);
  {*********************************************************************}
  { PROCEDURE BuildConstant                                             }
  {  Description: This routine takes care of parsing a DB,DD,or DW      }
  {  line and adding those to the assembler node. Expressions, range-   }
  {  checking are fully taken care of.                                  }
  {*********************************************************************}
  { EXIT CONDITION:  On exit the routine should point to AS_SEPARATOR.  }
  {*********************************************************************}
  var
    expr: string;
    value : tcgint;
  begin
    repeat
      case actasmtoken of
        AS_STRING:
            begin
              expr:=actasmpattern;
              Consume(AS_STRING);
              if (constsize <> 1) or (length(expr) > 1) then
                Message(asmr_e_string_not_allowed_as_const);

              if not (actasmtoken in [AS_COMMA, AS_SEPARATOR]) then
                Message(asmr_e_invalid_string_expression);

              ConcatString(curlist,expr);
            end;
        AS_ID,
        AS_INTNUM,AS_BINNUM,
        AS_OCTALNUM,AS_HEXNUM,
        { These terms can start an assembler expression }
        AS_PLUS,AS_MINUS,AS_LPAREN,AS_NOT:
            begin
              value:=BuildExpression(false,nil);
              ConcatConstant(curlist,value,constsize);
            end;
        AS_COMMA:
            begin
              Consume(AS_COMMA);
            end;
        AS_SEPARATOR: ;
      else
        begin
          Message(asmr_e_syntax_error);
        end;
      end; { end case }
    until actasmtoken = AS_SEPARATOR;
  end;


  Procedure TM68kMotreader.BuildScaling(const oper:tm68koperand);
  {*********************************************************************}
  {  Takes care of parsing expression starting from the scaling value   }
  {  up to and including possible field specifiers.                     }
  { EXIT CONDITION:  On exit the routine should point to  AS_SEPARATOR  }
  { or AS_COMMA. On entry should point to the AS_STAR  token.           }
  {*********************************************************************}
  var str:string;
      l: longint;
      code: integer;
  begin
     str:='';
     Consume(AS_STAR);
     if (oper.opr.ref.scalefactor <> 0)
     and (oper.opr.ref.scalefactor <> 1) then
      Message(asmr_e_wrong_base_index);
     case actasmtoken of
        AS_INTNUM: str := actasmpattern;
        AS_HEXNUM: str := Tostr(ParseVal(actasmpattern,16));
        AS_BINNUM: str := Tostr(ParseVal(actasmpattern,2));
        AS_OCTALNUM: str := Tostr(ParseVal(actasmpattern,8));
     else
        Message(asmr_e_syntax_error);
     end;
     val(str, l, code);
     if code <> 0 then
      Message(asmr_e_wrong_scale_factor);
     if ((l = 2) or (l = 4) or (l = 8) or (l = 1)) and (code = 0) then
     begin
        oper.opr.ref.scalefactor := l;
     end
     else
     begin
        Message(asmr_e_wrong_scale_factor);
        oper.opr.ref.scalefactor := 0;
     end;
     if oper.opr.ref.index = NR_NO then
     begin
        Message(asmr_e_wrong_base_index);
        oper.opr.ref.scalefactor := 0;
     end;
    { Consume the scaling number }
    Consume(actasmtoken);
    if actasmtoken = AS_RPAREN then
        Consume(AS_RPAREN)
    else
       Message(asmr_e_wrong_scale_factor);
    { // .Field.Field ... or separator/comma // }
    if actasmtoken in [AS_COMMA,AS_SEPARATOR] then
    begin
    end
    else
     Message(asmr_e_syntax_error);
  end;


  Function TM68kMotreader.BuildRefExpression: tcgint;
  {*********************************************************************}
  { FUNCTION BuildRefExpression: tcgint                                 }
  {  Description: This routine calculates a constant expression to      }
  {  a given value. The return value is the value calculated from       }
  {  the expression.                                                    }
  { The following tokens (not strings) are recognized:                  }
  {    SHL,SHR,/,*,NOT,OR,XOR,AND,MOD,+/-,numbers,ID to constants.      }
  {*********************************************************************}
  { ENTRY: On entry the token should be any valid expression token.     }
  { EXIT:  On Exit the token points to the LPAREN token.                }
  { ERROR RECOVERY: Tries to find COMMA or SEPARATOR token by consuming }
  {  invalid tokens.                                                    }
  {*********************************************************************}
  var
    tempstr: string;
    expr: string;
    l : tcgint;
    errorflag : boolean;
  begin
    BuildRefExpression := 0;
    errorflag := FALSE;
    expr := '';

    repeat
      tempstr := '';
      case actasmtoken of
        AS_RPAREN:
            Message(asmr_e_syntax_error);
        AS_SHL:
            tempstr := '<';
        AS_SHR:
            tempstr := '>';
        AS_SLASH:
            tempstr := '/';
        AS_MOD:
            tempstr := '%';
        AS_STAR:
            tempstr := '*';
        AS_PLUS:
            tempstr := '+';
        AS_MINUS:
            tempstr := '-';
        AS_AND:
            tempstr := '&';
        AS_NOT:
            tempstr := '~';
        AS_XOR:
            tempstr := '^';
        AS_OR:
            tempstr := '|';

        { End of reference }
        AS_LPAREN:
            begin
              if not ErrorFlag then
                BuildRefExpression := CalculateExpression(expr);
              { no longer in an expression }
              exit;
            end;
        AS_ID:
            begin
              if not SearchIConstant(actasmpattern,l) then
                begin
                  Message(asmr_e_syn_constant);
                  l := 0;
                end;
              str(l, tempstr);
            end;
        AS_INTNUM:
            tempstr := actasmpattern;
        AS_BINNUM:
            begin
              tempstr := Tostr(ParseVal(actasmpattern,2));
              if tempstr = '' then
                Message(asmr_e_error_converting_binary);
            end;
        AS_HEXNUM:
            begin
              tempstr := Tostr(ParseVal(actasmpattern,16));
              if tempstr = '' then
                Message(asmr_e_error_converting_hexadecimal);
            end;
        AS_OCTALNUM:
            begin
              tempstr := Tostr(ParseVal(actasmpattern,8));
              if tempstr = '' then
                Message(asmr_e_error_converting_octal);
            end;
        else
          begin
            if actasmtoken in [AS_COMMA,AS_SEPARATOR] then
              begin
                { no longer in an expression }
                if not ErrorFlag then
                  BuildRefExpression := CalculateExpression(expr);
                exit;
              end;

            { write error only once. }
            if not errorflag then
              Message(asmr_e_invalid_constant_expression);
            { consume tokens until we find COMMA or SEPARATOR }
            errorflag := true;
        end;
      end;
      if tempstr <> '' then
        expr := expr + tempstr;
      Consume(actasmtoken);
    until false;
  end;



  {*********************************************************************}
  { PROCEDURE BuildBracketExpression                                    }
  {  Description: This routine builds up an expression after a LPAREN   }
  {  token is encountered.                                              }
  {   On entry actasmtoken should be equal to AS_LPAREN                 }
  {*********************************************************************}
  { EXIT CONDITION:  On exit the routine should point to either the     }
  {       AS_COMMA or AS_SEPARATOR token.                               }
  {*********************************************************************}
  procedure TM68kMotreader.BuildReference(const oper:tm68koperand);
    var
      l:tcgint;
      code: integer;
      str: string;
    begin
       str:='';
       Consume(AS_LPAREN);
       case actasmtoken of
         { // (reg ... // }
         AS_REGISTER:
           begin
             oper.opr.ref.base := actasmregister;
             Consume(AS_REGISTER);
             { can either be a register or a right parenthesis }
             { // (reg)       // }
             { // (reg)+      // }
             if actasmtoken=AS_RPAREN then
               begin
                 Consume(AS_RPAREN);
                 if actasmtoken = AS_PLUS then
                 begin
                   if (oper.opr.ref.direction <> dir_none) then
                    Message(asmr_e_no_inc_and_dec_together)
                   else
                     oper.opr.ref.direction := dir_inc;
                   Consume(AS_PLUS);
                 end;
                 if not (actasmtoken in [AS_COMMA,AS_SEPARATOR]) then
                   begin
                     Message(asmr_e_invalid_reference_syntax);
                     { error recovery ... }
                     while actasmtoken <> AS_SEPARATOR do
                        Consume(actasmtoken);
                   end;
                   exit;
               end;
              { // (reg,reg .. // }
              Consume(AS_COMMA);
              if actasmtoken = AS_REGISTER then
                begin
                  oper.opr.ref.index :=
                    actasmregister;
                  Consume(AS_REGISTER);
                  { check for scaling ... }
                  case actasmtoken of
                    AS_RPAREN:
                       begin
                         Consume(AS_RPAREN);
                         if not (actasmtoken in [AS_COMMA,AS_SEPARATOR]) then
                         begin
                           { error recovery ... }
                           Message(asmr_e_invalid_reference_syntax);
                           while actasmtoken <> AS_SEPARATOR do
                             Consume(actasmtoken);
                         end;
                         exit;
                       end;
                    AS_STAR:
                       begin
                         BuildScaling(oper);
                       end;
                    else
                      begin
                        Message(asmr_e_invalid_reference_syntax);
                        while (actasmtoken <> AS_SEPARATOR) do
                          Consume(actasmtoken);
                      end;
                  end; { end case }
                end
              else
                begin
                   Message(asmr_e_invalid_reference_syntax);
                  while (actasmtoken <> AS_SEPARATOR) do
                      Consume(actasmtoken);
                end;
           end;
         AS_HEXNUM,AS_OCTALNUM,   { direct address }
         AS_BINNUM,AS_INTNUM:
           begin
             case actasmtoken of
               AS_INTNUM: str := actasmpattern;
               AS_HEXNUM: str := Tostr(ParseVal(actasmpattern,16));
               AS_BINNUM: str := Tostr(ParseVal(actasmpattern,2));
               AS_OCTALNUM: str := Tostr(ParseVal(actasmpattern,8));
              else
                Message(asmr_e_syntax_error);
             end;
             Consume(actasmtoken);
             val(str, l, code);
             if code <> 0 then
               Message(asmr_e_invalid_reference_syntax)
             else
               oper.opr.ref.offset := l;
             Consume(AS_RPAREN);
             if not (actasmtoken in [AS_COMMA,AS_SEPARATOR]) then
             begin
               { error recovery ... }
               Message(asmr_e_invalid_reference_syntax);
               while actasmtoken <> AS_SEPARATOR do
                 Consume(actasmtoken);
             end;
             exit;
           end;
         else
           begin
             Message(asmr_e_invalid_reference_syntax);
             while (actasmtoken <> AS_SEPARATOR) do
               Consume(actasmtoken);
           end;
       end;
    end;


  procedure tm68kmotreader.BuildRegList(const oper:tm68koperand);
  {*********************************************************************}
  { EXIT CONDITION:  On exit the routine should point to either the     }
  {       AS_COMMA or AS_SEPARATOR token.                               }
  {*********************************************************************}
  var
    i: Tsuperregister;
    reg_one, reg_two: tregister;
    rs_one, rs_two: tsuperregister;
    addrregset,dataregset,fpuregset: tcpuregisterset;
    errorflag, first: boolean;
  begin
    dataregset := [];
    addrregset := [];
    fpuregset := [];

    { 1., try to consume a register list
      2., if successful, add the register list
      3., if not possible, then we have a standalone register, add
      4., repeat until we dont have a slash any more }

    errorflag:=false;
    first:=true;
    repeat
      reg_one:=actasmregister;
      rs_one:=getsupreg(reg_one);
      if not (first or try_to_consume(AS_REGISTER)) then
        begin
          errorflag:=true;
          break;
        end;
      first:=false;

      { try to consume a register list }
      if try_to_consume(AS_MINUS) then
        begin
          reg_two:=actasmregister;
          rs_two:=getsupreg(reg_two);
          if (not try_to_consume(AS_REGISTER)) or
             (rs_one >= rs_two) or
             (getregtype(reg_one) <> getregtype(reg_two)) then
            begin
              errorflag:=true;
              break;
            end;
        end
      else
        begin
          { nope, we have a single element "list" }
          reg_two:=reg_one;
          rs_two:=getsupreg(reg_two);
        end;

      case getregtype(reg_one) of
        R_INTREGISTER:
          for i:=getsupreg(reg_one) to getsupreg(reg_two) do
            include(dataregset,i);
        R_ADDRESSREGISTER:
          for i:=getsupreg(reg_one) to getsupreg(reg_two) do
            include(addrregset,i);
        R_FPUREGISTER:
          for i:=getsupreg(reg_one) to getsupreg(reg_two) do
            include(fpuregset,i);
        else
          internalerror(201611041);
      end;

    until not try_to_consume(AS_SLASH);

    errorflag:=errorflag or
               (((dataregset <> []) or (addrregset <> [])) and (fpuregset <> [])) or
               (not (actasmtoken in [AS_SEPARATOR,AS_COMMA]));

    if errorflag then
      begin
        Message(asmr_e_invalid_reg_list_in_movem_or_fmovem);
        consume_all_until([AS_SEPARATOR,AS_COMMA]);
      end
    else
      begin
        oper.opr.typ:= OPR_REGSET;
        oper.opr.regsetdata := dataregset;
        oper.opr.regsetaddr := addrregset;
        oper.opr.regsetfpu  := fpuregset;
      end;
  end;


  procedure tm68kmotreader.BuildRegPair(const oper:tm68koperand);
  {*********************************************************************}
  { EXIT CONDITION:  On exit the routine should point to either the     }
  {       AS_COMMA or AS_SEPARATOR token.                               }
  {*********************************************************************}
  begin
    oper.opr.typ   := OPR_REGPAIR;
    oper.opr.reghi := actasmregister;
    Consume(AS_COLON);
    if not try_to_consume(AS_REGISTER) then
      begin
        Message(asmr_e_invalid_reg_list_for_opcode);
        consume_all_until([AS_SEPARATOR,AS_COMMA]);
      end
    else
      oper.opr.reglo:=actasmregister;
  end;


  Procedure TM68kMotreader.BuildOperand(const oper:tm68koperand);
  {*********************************************************************}
  { EXIT CONDITION:  On exit the routine should point to either the     }
  {       AS_COMMA or AS_SEPARATOR token.                               }
  {*********************************************************************}
  var
    expr: string;
    tempstr: string;
    lab: tasmlabel;
    l : tcgint;
    hl: tasmlabel;
    p: pointer;
  begin
   case actasmtoken of
   { // Memory reference //  }
     AS_LPAREN:
               begin
                  Oper.InitRef;
                  BuildReference(oper);
               end;
   { // Constant expression //  }
     AS_APPT:  begin
                      Consume(AS_APPT);
                      if not (oper.opr.typ in [OPR_NONE,OPR_CONSTANT]) then
                         Message(asmr_e_invalid_operand_type);
                      { identifiers are handled by BuildExpression }
                      oper.opr.typ := OPR_CONSTANT;
                      l:=BuildExpression(true,@tempstr);
                      oper.opr.val :=aint(l);
                      if tempstr<>'' then
                        begin
                          l:=oper.opr.val;
                          oper.opr.typ := OPR_SYMBOL;
                          oper.opr.symofs := l;
                          oper.opr.symbol := current_asmdata.RefAsmSymbol(tempstr,AT_FUNCTION);
                        end;
                 end;
   { // Constant memory offset .              // }
     AS_HEXNUM,AS_INTNUM,
     AS_BINNUM,AS_OCTALNUM,AS_PLUS:
                   begin
                      Oper.InitRef;
                      oper.opr.ref.offset:=BuildRefExpression;
                      if actasmtoken = AS_LPAREN then
                        BuildReference(oper);
                   end;
   { // A constant expression, or a Variable ref. // }
     AS_ID:  begin
              if actasmpattern[1] = '@' then
              { // Label or Special symbol reference // }
              begin
                 if actasmpattern = '@RESULT' then
                    oper.SetUpResult
                 else
                 if actasmpattern = 'SELF' then
                    oper.SetUpSelf
                 else
                 if (actasmpattern = '@CODE') or (actasmpattern = '@DATA') then
                    Message(asmr_w_CODE_and_DATA_not_supported)
                 else
                  begin
                    delete(actasmpattern,1,1);
                    if actasmpattern = '' then
                     Message(asmr_e_null_label_ref_not_allowed);
                    CreateLocalLabel(actasmpattern,lab,false);
                    oper.opr.typ := OPR_SYMBOL;
                    oper.opr.symbol := lab;
                    oper.opr.symofs := 0;
                  end;
                Consume(AS_ID);
                if not (actasmtoken in [AS_SEPARATOR,AS_COMMA]) then
                  Message(asmr_e_syntax_error);
              end
              { probably a variable or normal expression }
              { or a procedure (such as in CALL ID)      }
              else
               begin
                 { is it a constant ? }
                 if SearchIConstant(actasmpattern,l) then
                   begin
                     Oper.InitRef;
                     oper.opr.ref.offset:=BuildRefExpression;
                     if actasmtoken = AS_LPAREN then
                       BuildReference(oper);
                   end
                 else { is it a label variable ? }

                     { // ID[ , ID.Field.Field or simple ID // }
                     { check if this is a label, if so then }
                     { emit it as a label.                  }
                     if SearchLabel(actasmpattern,hl,false) then
                       begin
                         oper.opr.typ := OPR_SYMBOL;
                         oper.opr.symbol := hl;
                         oper.opr.symofs := 0;
                         Consume(AS_ID);
                         if not (actasmtoken in [AS_SEPARATOR,AS_COMMA]) then
                          Message(asmr_e_syntax_error);

                       end
                      else begin
                       expr:=actasmpattern;
                       Consume(AS_ID);
                       { typecasting? }
                       if SearchType(expr,l) then
                        begin
                          oper.hastype:=true;
                          oper.typesize:=l;
                          case actasmtoken of
                            AS_LPAREN :
                              begin
                                { Support Type([Reference]) }
                                Consume(AS_LPAREN);
                                BuildOperand(oper{,true});
                                Consume(AS_RPAREN);
                              end;
                            AS_LBRACKET :
                              begin
                                { Support Var.Type[Index] }
                                { Convert @label.Byte[1] to reference }
                                if oper.opr.typ=OPR_SYMBOL then
                                  oper.initref;
                              end;
                            else
                              ;
                          end;
                        end
                       else
                        begin
                          if actasmtoken = AS_LPAREN then
                            oper.initref;
                          if not oper.SetupVar(expr,false) then
                            begin
                              { not a variable, check special variables.. }
                              if expr = 'SELF' then
                                oper.SetupSelf
                              else begin
                                Message1(sym_e_unknown_id,expr);
                              end;
                              expr:='';
                            end;
                         end;
//                       Message1(sym_e_unknown_id,actasmpattern);
                      end;

                       case actasmtoken of
                         AS_LPAREN: { indexing }
                           BuildReference(oper);
                         AS_SEPARATOR,AS_COMMA: begin
                         end;
                       else
                         Message(asmr_e_syntax_error);
                       end;

                   end;
               end;

   { // Pre-decrement mode reference or constant mem offset.   // }
     AS_MINUS:    begin
                   Consume(AS_MINUS);
                   if actasmtoken = AS_LPAREN then
                   begin
                     Oper.InitRef;
                     { indicate pre-decrement mode }
                     oper.opr.ref.direction := dir_dec;
                     BuildReference(oper);
                   end
                   else
                   if actasmtoken in [AS_OCTALNUM,AS_HEXNUM,AS_BINNUM,AS_INTNUM] then
                   begin
                      Oper.InitRef;
                      oper.opr.ref.offset:=BuildRefExpression;
                      { negate because was preceded by a negative sign! }
                      oper.opr.ref.offset:=-oper.opr.ref.offset;
                      BuildReference(oper);
                   end
                   else
                   begin
                    Message(asmr_e_syntax_error);
                    while not (actasmtoken in [AS_SEPARATOR,AS_COMMA]) do
                       Consume(actasmtoken);
                   end;
                  end;
   { // Register, a variable reference or a constant reference // }
     AS_REGISTER: begin
                   Consume(AS_REGISTER);
                   case actasmtoken of
                     AS_SEPARATOR, AS_COMMA:
                       begin
                         { // Simple register // }
                         if not (oper.opr.typ in [OPR_NONE,OPR_REGISTER]) then
                           Message(asmr_e_invalid_operand_type);
                         oper.opr.typ := OPR_REGISTER;
                         oper.opr.reg := actasmregister;
                       end;
                     AS_SLASH, AS_MINUS:
                       { // Register listing // }
                       BuildRegList(oper);
                     AS_COLON:
                       { // Register pair // }
                       BuildRegPair(oper);
                     else
                       Message(asmr_e_invalid_register);
                   end;
                 end;
     AS_SEPARATOR, AS_COMMA: ;
    else
     begin
      Message(asmr_e_invalid_opcode_and_operand);
      Consume(actasmtoken);
     end;
  end; { end case }
 end;


  Procedure TM68kmotReader.BuildOpCode(instr:Tm68kinstruction);
  {*********************************************************************}
  { PROCEDURE BuildOpcode;                                              }
  {  Description: Parses the intel opcode and operands, and writes it   }
  {  in the TInstruction object.                                        }
  {*********************************************************************}
  { EXIT CONDITION:  On exit the routine should point to AS_SEPARATOR.  }
  { On ENTRY: Token should point to AS_OPCODE                           }
  {*********************************************************************}
  var
      operandnum : longint;
  begin
    { //  opcode                          // }
    { allow for newline as in gas styled syntax }
    { under DOS you get two AS_SEPARATOR !! }
    while actasmtoken=AS_SEPARATOR do
      Consume(AS_SEPARATOR);
    if (actasmtoken <> AS_OPCODE) then
    begin
      Message(asmr_e_invalid_or_missing_opcode);
      { error recovery }
      While not (actasmtoken in [AS_SEPARATOR,AS_COMMA]) do
         Consume(actasmtoken);
      exit;
    end
    else
    begin
      Instr.opcode := findopcode(actasmpattern,instr.opsize);
      Consume(AS_OPCODE);
      { // Zero operand opcode ? // }
      if actasmtoken = AS_SEPARATOR then
        exit
      else
       operandnum := 1;
    end;

    While actasmtoken <> AS_SEPARATOR do
    begin
       case actasmtoken of
         { //  Operand delimiter // }
         AS_COMMA: begin
                  if operandnum > Max_Operands then
                    Message(asmr_e_too_many_operands)
                  else
                    Inc(operandnum);
                  Consume(AS_COMMA);
                end;
         { // End of asm operands for this opcode // }
         AS_SEPARATOR: ;
       else
         BuildOperand(Instr.Operands[operandnum] as tm68koperand);
     end; { end case }
    end; { end while }
    instr.Ops:=operandnum;
  end;




    function tm68kmotreader.Assemble: tlinkedlist;
      var
        hl: tasmlabel;
        instr : TM68kInstruction;
      begin
        //Message(asmr_d_start_reading);
        firsttoken := TRUE;
        operandnum := 0;
        { sets up all opcode and register tables in uppercase }
        if not _asmsorted then
          begin
            SetupTables;
            _asmsorted := TRUE;
          end;
        curlist:=TAsmList.Create;
        gettoken;
        while actasmtoken<>AS_END do
          begin
            case actasmtoken of
              AS_LLABEL:
                begin
                  if CreateLocalLabel(actasmpattern,hl,true) then
                    ConcatLabel(curlist,hl);
                  Consume(AS_LLABEL);
                end;
              AS_LABEL:
                begin
                  { when looking for Pascal labels, these must }
                  { be in uppercase.                           }
                  if SearchLabel(upper(actasmpattern),hl,true) then
                    ConcatLabel(curlist,hl)
                  else
                    Message1(asmr_e_unknown_label_identifier,actasmpattern);
                  Consume(AS_LABEL);
                end;
              AS_DW:
                begin
                  Consume(AS_DW);
                  BuildConstant(sizeof(word));
                end;
              AS_DB:
                begin
                  Consume(AS_DB);
                  BuildConstant(sizeof(byte));
                end;
              AS_DD:
                begin
                  Consume(AS_DD);
                  BuildConstant(sizeof(dword));
                end;
              AS_XDEF:
                begin
                  Consume(AS_XDEF);
                  if actasmtoken=AS_ID then
                    ConcatPublic(curlist,actasmpattern);
                  Consume(AS_ID);
                  if actasmtoken<>AS_SEPARATOR then
                   Consume(AS_SEPARATOR);
                end;
              AS_ALIGN:
                begin
                  Message(asmr_w_align_not_supported);
                  while actasmtoken <> AS_SEPARATOR do
                   Consume(actasmtoken);
                end;
              AS_OPCODE:
                begin
                  instr:=TM68kInstruction.Create(tm68koperand);
                  BuildOpcode(instr);
//                  instr.AddReferenceSizes;
//                  instr.SetInstructionOpsize;
//                  instr.CheckOperandSizes;
                  if instr.labeled then
                     instr.ConcatLabeledInstr(curlist)
                  else begin
                    instr.ConcatInstruction(curlist);
                  end;
                  instr.Free;

                end;
              AS_SEPARATOR:
                begin
                  Consume(AS_SEPARATOR);
                  { let us go back to the first operand }
                  operandnum := 0;
                end;
              AS_END:
                { end assembly block }
                ;
              else
                begin
                  Message(asmr_e_syntax_error);
                  { error recovery }
                  Consume(actasmtoken);
                end;
            end; { end case }
          end; { end while }

        { Check LocalLabelList }
        checklocallabels;

        assemble:=curlist;
        //Message(asmr_d_finish_reading);
      end;


{*****************************************************************************
                               Initialize
*****************************************************************************}

const
  asmmode_m68k_mot_info : tasmmodeinfo =
          (
            id    : asmmode_m68k_mot;
            idtxt : 'MOTOROLA';
            casmreader : tm68kmotreader;
          );

  asmmode_m68k_standard_info : tasmmodeinfo =
          (
            id    : asmmode_standard;
            idtxt : 'STANDARD';
            casmreader : tm68kmotreader;
          );

begin
  RegisterAsmMode(asmmode_m68k_mot_info);
  RegisterAsmMode(asmmode_m68k_standard_info);
end.
