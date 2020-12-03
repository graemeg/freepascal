{
    Copyright (c) 1998-2008 by Carl Eric Codere and Peter Vreman

    Does the parsing for the Z80 styled inline assembler.

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
Unit raz80asm;

{$i fpcdefs.inc}

  Interface

    uses
      cclasses,
      globtype,
      rasm,raz80,
      aasmbase,cpubase;

    type
      tasmtoken = (
        AS_NONE,AS_LABEL,AS_LLABEL,AS_STRING,AS_INTNUM,
        AS_REALNUM,AS_COMMA,AS_LPAREN,
        AS_RPAREN,AS_COLON,AS_DOT,AS_PLUS,AS_MINUS,AS_STAR,
        AS_SEPARATOR,AS_ID,AS_REGISTER,AS_OPCODE,AS_CONDITION,AS_SLASH,AS_DOLLAR,
        AS_HASH,AS_LSBRACKET,AS_RSBRACKET,AS_LBRACKET,AS_RBRACKET,
        AS_EQUAL,
        {------------------ Assembler directives --------------------}
        AS_DEFB,AS_DEFW,AS_AREA,AS_END,
        {------------------ Assembler Operators  --------------------}
        AS_TYPE,AS_OFFSET,AS_SIZEOF,AS_VMTOFFSET,AS_MOD,AS_SHL,AS_SHR,AS_NOT,AS_AND,AS_OR,AS_XOR,AS_NOR,AS_AT,
        AS_RELTYPE, // common token for relocation types
        {------------------ Target-specific directive ---------------}
        AS_TARGET_DIRECTIVE
        );
      tasmkeyword = string[10];

    const
      { These tokens should be modified accordingly to the modifications }
      { in the different enumerations.                                   }
      firstdirective = AS_DEFB;
      lastdirective  = AS_END;
      token2str : array[tasmtoken] of tasmkeyword=(
        '','Label','LLabel','string','integer',
        'float',',','(',
        ')',':','.','+','-','*',
        ';','identifier','register','opcode','condition','/','$',
        '#','{','}','[',']',
        '=',
        'defb','defw','area','END',
        'TYPE','OFFSET','SIZEOF','VMTOFFSET','%','<<','>>','!','&','|','^','~','@','reltype',
        'directive');

    type
      { input flags for BuildConstSymbolExpression }
      tconstsymbolexpressioninputflag = (
        cseif_needofs,
        cseif_isref,
        cseif_startingminus,
        { allows using full reference-like syntax for constsymbol expressions,
          for example:
          Rec.Str[5]  ->  Rec.Str+5 }
        cseif_referencelike
      );
      tconstsymbolexpressioninputflags = set of tconstsymbolexpressioninputflag;
      { output flags for BuildConstSymbolExpression }
      tconstsymbolexpressionoutputflag = (
        cseof_isseg,
        cseof_is_farproc_entry,
        cseof_hasofs
      );
      tconstsymbolexpressionoutputflags = set of tconstsymbolexpressionoutputflag;

      { tz80reader }

      tz80reader = class(tasmreader)
        actasmcond : TAsmCond;
        actasmpattern_origcase : string;
        actasmtoken   : tasmtoken;
        prevasmtoken  : tasmtoken;
        inexpression : boolean;
        procedure SetupTables;
        procedure GetToken;
        function consume(t : tasmtoken):boolean;
        procedure RecoverConsume(allowcomma:boolean);
        procedure AddReferences(dest,src : tz80operand);
        function is_locallabel(const s:string):boolean;
        function is_asmopcode(const s: string):boolean;
        Function is_asmdirective(const s: string):boolean;
        function is_register(const s:string):boolean;
        function is_condition(const s:string):boolean;
        function is_targetdirective(const s: string):boolean;
        procedure BuildRecordOffsetSize(const expr: string;out offset:tcgint;out size:tcgint; out mangledname: string; needvmtofs: boolean; out hastypecast: boolean);
        procedure BuildConstSymbolExpression(in_flags: tconstsymbolexpressioninputflags;out value:tcgint;out asmsym:string;out asmsymtyp:TAsmsymtype;out size:tcgint;out out_flags:tconstsymbolexpressionoutputflags);
        function BuildConstExpression:longint;
        function BuildRefConstExpression(out size:tcgint;startingminus:boolean=false):longint;
        procedure BuildConstantOperand(oper: tz80operand);
        procedure BuildReference(oper : tz80operand);
        procedure BuildOperand(oper: tz80operand;istypecast:boolean);
        procedure BuildOpCode(instr:TZ80Instruction);
        procedure BuildConstant(constsize: byte);
        procedure handleopcode;
        procedure ConvertCalljmp(instr : tz80instruction);
        function Assemble: tlinkedlist;override;
      end;


  Implementation

    uses
      { helpers }
      cutils,
      { global }
      globals,verbose,
      systems,
      { aasm }
      cpuinfo,aasmtai,aasmdata,aasmcpu,
      { symtable }
      symconst,symbase,symtype,symsym,symtable,symdef,symutil,
      { parser }
      scanner,pbase,
      procinfo,
      rabase,rautils,
      cgbase,cgutils,cgobj
      ;


{*****************************************************************************
                                tz80reader
*****************************************************************************}


    procedure tz80reader.SetupTables;
      var
        i: TAsmOp;
      begin
        iasmops:=TFPHashList.create;
        for i:=firstop to lastop do
          iasmops.Add(upper(std_op2str[i]),Pointer(PtrInt(i)));
      end;


    procedure tz80reader.GetToken;
      var
        len: Integer;
        srsym : tsym;
        srsymtable : TSymtable;
        can_be_condition : Boolean;
      begin
        c:=scanner.c;
        { certain instructions can have a condition, as an operand. We need to set this flag,
          because 'C' can be either a register, or a condition, depending on the context }
        can_be_condition:=(actasmtoken=AS_OPCODE) and (actopcode in [A_JP,A_JR,A_JRJP,A_CALL,A_RET]);
        { save old token and reset new token }
        prevasmtoken:=actasmtoken;
        actasmtoken:=AS_NONE;
        { reset }
        actasmpattern:='';
        { while space and tab , continue scan... }
        while c in [' ',#9] do
          c:=current_scanner.asmgetchar;
        { get token pos }
        if not (c in [#10,#13,'{',';','/','(']) then
          current_scanner.gettokenpos;
        { Local Label, Label, Directive, Prefix or Opcode }
        if firsttoken and not(c in [#10,#13,'{',';','/','(']) then
          begin
            firsttoken:=FALSE;
            len:=0;
            { directive }
            if c = '.' then
              begin
                inc(len);
                actasmpattern[len]:=c;
                { Let us point to the next character }
                c:=current_scanner.asmgetchar;
                while c in ['A'..'Z','a'..'z','0'..'9','_','$'] do
                  begin
                    inc(len);
                    actasmpattern[len]:=c;
                    c:=current_scanner.asmgetchar;
                  end;
                actasmpattern[0]:=chr(len);
                { must be a directive }
                if is_asmdirective(actasmpattern) then
                 exit;
                if is_targetdirective(actasmpattern) then
                  begin
                    actasmtoken:=AS_TARGET_DIRECTIVE;
                    exit;
                  end;
                Message1(asmr_e_not_directive_or_local_symbol,actasmpattern);
              end;
            { only opcodes, global and local labels are allowed now. }
            while c in ['A'..'Z','a'..'z','0'..'9','_','@'] do
              begin
                inc(len);
                actasmpattern[len]:=c;
                c:=current_scanner.asmgetchar;
              end;
            actasmpattern[0]:=chr(len);
            actasmpattern_origcase:=actasmpattern;
            { Label ? }
            if c = ':' then
              begin
                { Local label ? }
                if is_locallabel(actasmpattern) then
                  actasmtoken:=AS_LLABEL
                else
                  actasmtoken:=AS_LABEL;
                { let us point to the next character }
                c:=current_scanner.asmgetchar;
                firsttoken:=true;
                exit;
              end;
            { Directive ? }
            if is_asmdirective(actasmpattern) then
              exit;
            { Opcode ? }
            if is_asmopcode(upper(actasmpattern)) then
              begin
                uppervar(actasmpattern);
                exit;
              end;
            { End of assemblerblock ? }
            if upper(actasmpattern) = 'END' then
              begin
                actasmtoken:=AS_END;
                exit;
              end;
            message1(asmr_e_unknown_opcode,actasmpattern);
            actasmtoken:=AS_NONE;
          end
        else { else firsttoken }
          { Here we must handle all possible cases }
          begin
            case c of
              '.' :  { possiblities : - local label reference , such as in jmp @local1 }
                     {               - field of object/record                         }
                     {               - directive.                                     }
                begin
                  if (prevasmtoken in [AS_ID,AS_RPAREN]) then
                   begin
                     c:=current_scanner.asmgetchar;
                     actasmtoken:=AS_DOT;
                     exit;
                   end;
                  actasmpattern:=c;
                  c:=current_scanner.asmgetchar;
                  while c in  ['A'..'Z','a'..'z','0'..'9','_','$'] do
                   begin
                     actasmpattern:=actasmpattern + c;
                     c:=current_scanner.asmgetchar;
                   end;
                  if is_asmdirective(actasmpattern) then
                   exit;
                  if is_targetdirective(actasmpattern) then
                    begin
                      actasmtoken:=AS_TARGET_DIRECTIVE;
                      exit;
                    end;
                  { local label references and directives }
                  { are case sensitive                    }
                  actasmtoken:=AS_ID;
                  exit;
                end;

           { identifier, register, prefix or directive }
              '_','A'..'Z','a'..'z':
                begin
                  len:=0;
                  while c in ['A'..'Z','a'..'z','0'..'9','_','$'] do
                   begin
                     inc(len);
                     actasmpattern[len]:=c;
                     c:=current_scanner.asmgetchar;
                   end;
                  actasmpattern[0]:=chr(len);
                  actasmpattern_origcase:=actasmpattern;
                  uppervar(actasmpattern);
                  { check for end which is a reserved word unlike the opcodes }
                  if actasmpattern = 'END' then
                    begin
                      actasmtoken:=AS_END;
                      exit;
                    end;
                  if actasmpattern = 'TYPE' then
                    begin
                      actasmtoken:=AS_TYPE;
                      exit;
                    end;
                  if actasmpattern = 'OFFSET' then
                    begin
                      actasmtoken:=AS_OFFSET;
                      exit;
                    end;
                  if actasmpattern = 'SIZEOF' then
                    begin
                      actasmtoken:=AS_SIZEOF;
                      exit;
                    end;
                  if actasmpattern = 'VMTOFFSET' then
                    begin
                      actasmtoken:=AS_VMTOFFSET;
                      exit;
                    end;
                  if can_be_condition and is_condition(actasmpattern) then
                    begin
                      actasmtoken:=AS_CONDITION;
                      exit;
                    end;
                  if is_register(actasmpattern) then
                    begin
                      actasmtoken:=AS_REGISTER;
                      { is it an alternate register? }
                      if (c='''') and is_register(actasmpattern+'''') then
                        begin
                          actasmpattern:=actasmpattern+c;
                          c:=current_scanner.asmgetchar;
                        end;
                      exit;
                    end;
                  { if next is a '.' and this is a unitsym then we also need to
                    parse the identifier }
                  if (c='.') then
                   begin
                     searchsym(actasmpattern,srsym,srsymtable);
                     if assigned(srsym) and
                        (srsym.typ=unitsym) and
                        (srsym.owner.symtabletype in [staticsymtable,globalsymtable]) and
                        srsym.owner.iscurrentunit then
                      begin
                        actasmpattern:=actasmpattern+c;
                        c:=current_scanner.asmgetchar;
                        while c in  ['A'..'Z','a'..'z','0'..'9','_','$'] do
                         begin
                           actasmpattern:=actasmpattern + upcase(c);
                           c:=current_scanner.asmgetchar;
                         end;
                      end;
                   end;
                  actasmtoken:=AS_ID;
                  exit;
                end;

              //'%' : { register or modulo }
              //  handlepercent;

              '1'..'9': { integer number }
                begin
                  len:=0;
                  while c in ['0'..'9'] do
                   Begin
                     inc(len);
                     actasmpattern[len]:=c;
                     c:=current_scanner.asmgetchar;
                   end;
                  actasmpattern[0]:=chr(len);
                  actasmpattern:=tostr(ParseVal(actasmpattern,10));
                  actasmtoken:=AS_INTNUM;
                  exit;
                end;
              '0' : { octal,hexa,real or binary number. }
                begin
                  actasmpattern:=c;
                  c:=current_scanner.asmgetchar;
                  case upcase(c) of
                    'B': { binary }
                      Begin
                        c:=current_scanner.asmgetchar;
                        while c in ['0','1'] do
                         Begin
                           actasmpattern:=actasmpattern + c;
                           c:=current_scanner.asmgetchar;
                         end;
                        actasmpattern:=tostr(ParseVal(actasmpattern,2));
                        actasmtoken:=AS_INTNUM;
                        exit;
                      end;
                    'D': { real }
                      Begin
                        c:=current_scanner.asmgetchar;
                        { get ridd of the 0d }
                        if (c in ['+','-']) then
                         begin
                           actasmpattern:=c;
                           c:=current_scanner.asmgetchar;
                         end
                        else
                         actasmpattern:='';
                        while c in ['0'..'9'] do
                         Begin
                           actasmpattern:=actasmpattern + c;
                           c:=current_scanner.asmgetchar;
                         end;
                        if c='.' then
                         begin
                           actasmpattern:=actasmpattern + c;
                           c:=current_scanner.asmgetchar;
                           while c in ['0'..'9'] do
                            Begin
                              actasmpattern:=actasmpattern + c;
                              c:=current_scanner.asmgetchar;
                            end;
                           if upcase(c) = 'E' then
                            begin
                              actasmpattern:=actasmpattern + c;
                              c:=current_scanner.asmgetchar;
                              if (c in ['+','-']) then
                               begin
                                 actasmpattern:=actasmpattern + c;
                                 c:=current_scanner.asmgetchar;
                               end;
                              while c in ['0'..'9'] do
                               Begin
                                 actasmpattern:=actasmpattern + c;
                                 c:=current_scanner.asmgetchar;
                               end;
                            end;
                           actasmtoken:=AS_REALNUM;
                           exit;
                         end
                        else
                         begin
                           Message1(asmr_e_invalid_float_const,actasmpattern+c);
                           actasmtoken:=AS_NONE;
                         end;
                      end;
                    'X': { hexadecimal }
                      Begin
                        c:=current_scanner.asmgetchar;
                        while c in ['0'..'9','a'..'f','A'..'F'] do
                         Begin
                           actasmpattern:=actasmpattern + c;
                           c:=current_scanner.asmgetchar;
                         end;
                        actasmpattern:=tostr(ParseVal(actasmpattern,16));
                        actasmtoken:=AS_INTNUM;
                        exit;
                      end;
                    '1'..'7': { octal }
                      begin
                        actasmpattern:=actasmpattern + c;
                        while c in ['0'..'7'] do
                         Begin
                           actasmpattern:=actasmpattern + c;
                           c:=current_scanner.asmgetchar;
                         end;
                        actasmpattern:=tostr(ParseVal(actasmpattern,8));
                        actasmtoken:=AS_INTNUM;
                        exit;
                      end;
                    else { octal number zero value...}
                      Begin
                        actasmpattern:=tostr(ParseVal(actasmpattern,8));
                        actasmtoken:=AS_INTNUM;
                        exit;
                      end;
                  end; { end case }
                end;

              '&' :
                begin
                  c:=current_scanner.asmgetchar;
                  actasmtoken:=AS_AND;
                end;

              '''' : { char }
                begin
                  actasmpattern:='';
                  repeat
                    c:=current_scanner.asmgetchar;
                    case c of
                      '\' :
                        begin
                          { copy also the next char so \" is parsed correctly }
                          actasmpattern:=actasmpattern+c;
                          c:=current_scanner.asmgetchar;
                          actasmpattern:=actasmpattern+c;
                        end;
                      '''' :
                        begin
                          c:=current_scanner.asmgetchar;
                          break;
                        end;
                      #10,#13:
                        Message(scan_f_string_exceeds_line);
                      else
                        actasmpattern:=actasmpattern+c;
                    end;
                  until false;
                  actasmpattern:=EscapeToPascal(actasmpattern);
                  actasmtoken:=AS_STRING;
                  exit;
                end;

              '"' : { string }
                begin
                  actasmpattern:='';
                  repeat
                    c:=current_scanner.asmgetchar;
                    case c of
                      '\' :
                        begin
                          { copy also the next char so \" is parsed correctly }
                          actasmpattern:=actasmpattern+c;
                          c:=current_scanner.asmgetchar;
                          actasmpattern:=actasmpattern+c;
                        end;
                      '"' :
                        begin
                          c:=current_scanner.asmgetchar;
                          break;
                        end;
                      #10,#13:
                        Message(scan_f_string_exceeds_line);
                      else
                        actasmpattern:=actasmpattern+c;
                    end;
                  until false;
                  actasmpattern:=EscapeToPascal(actasmpattern);
                  actasmtoken:=AS_STRING;
                  exit;
                end;

              //'$' :
              //  begin
              //    handledollar;
              //    exit;
              //  end;

              '#' :
                begin
                  actasmtoken:=AS_HASH;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              '[' :
                begin
                  actasmtoken:=AS_LBRACKET;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              ']' :
                begin
                  actasmtoken:=AS_RBRACKET;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              '{' :
                begin
 {$ifdef arm}
                  // the arm assembler uses { ... } for register sets
                  // but compiler directives {$... } are still allowed
                  c:=current_scanner.asmgetchar;
                  if c<>'$' then
                    actasmtoken:=AS_LSBRACKET
                  else
                    begin
                      current_scanner.skipcomment(false);
                      GetToken;
                    end;
 {$else arm}
                  current_scanner.skipcomment(true);
                  GetToken;
 {$endif arm}
                  exit;
                end;

 {$ifdef arm}
              '}' :
                begin
                  actasmtoken:=AS_RSBRACKET;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              '=' :
                begin
                  actasmtoken:=AS_EQUAL;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;
 {$endif arm}

              ',' :
                begin
                  actasmtoken:=AS_COMMA;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              '<' :
                begin
                  actasmtoken:=AS_SHL;
                  c:=current_scanner.asmgetchar;
                  if c = '<' then
                   c:=current_scanner.asmgetchar;
                  exit;
                end;

              '>' :
                begin
                  actasmtoken:=AS_SHL;
                  c:=current_scanner.asmgetchar;
                  if c = '>' then
                   c:=current_scanner.asmgetchar;
                  exit;
                end;

              '|' :
                begin
                  actasmtoken:=AS_OR;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              '^' :
                begin
                  actasmtoken:=AS_XOR;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;


              '(' :
                begin
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

              ')' :
                begin
                  actasmtoken:=AS_RPAREN;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              ':' :
                begin
                  actasmtoken:=AS_COLON;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              '+' :
                begin
                  actasmtoken:=AS_PLUS;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              '-' :
                begin
                  actasmtoken:=AS_MINUS;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              '*' :
                begin
                  actasmtoken:=AS_STAR;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              '/' :
                begin
                  c:=current_scanner.asmgetchar;
                  if c='/' then
                    begin
                      current_scanner.skipdelphicomment;
                      GetToken;
                    end
                  else
                    actasmtoken:=AS_SLASH;
                  exit;
                end;

              '!', '~' :
                begin
                  actasmtoken:=AS_NOT;
                  c:=current_scanner.asmgetchar;
                  exit;
                end;

              '@' : { possiblities : - local label reference , such as in jmp @local1 }
                    {                - @Result, @Code or @Data special variables.     }
                begin
                  actasmpattern:=c;
                  c:=current_scanner.asmgetchar;
                  while c in  ['A'..'Z','a'..'z','0'..'9','_','@','$','&','?'] do
                   begin
                     actasmpattern:=actasmpattern + c;
                     c:=current_scanner.asmgetchar;
                   end;
                  actasmpattern_origcase:=actasmpattern;
                  uppervar(actasmpattern);
                  actasmtoken:=AS_ID;
                  exit;
                end;

              #13,#10:
                begin
                  current_scanner.linebreak;
                  c:=current_scanner.asmgetchar;
                  firsttoken:=TRUE;
                  actasmtoken:=AS_SEPARATOR;
                  exit;
                end;

              ';' :
                begin
                  c:=current_scanner.asmgetchar;
                  firsttoken:=TRUE;
                  actasmtoken:=AS_SEPARATOR;
                  exit;
                end;

              else
                current_scanner.illegal_char(c);
            end;
          end;
      end;


    function tz80reader.consume(t: tasmtoken): boolean;
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


    procedure tz80reader.RecoverConsume(allowcomma: boolean);
      begin
        while not (actasmtoken in [AS_SEPARATOR,AS_END]) do
          begin
            if allowcomma and (actasmtoken=AS_COMMA) then
             break;
            Consume(actasmtoken);
          end;
      end;


    procedure tz80reader.AddReferences(dest, src: tz80operand);

      procedure AddRegister(reg:tregister;scalefactor:byte);
        begin
          if reg=NR_NO then
            exit;
          if (dest.opr.ref.base=NR_NO) and (scalefactor=1) then
            begin
              dest.opr.ref.base:=reg;
              exit;
            end;
          if dest.opr.ref.index=NR_NO then
            begin
              dest.opr.ref.index:=reg;
              dest.opr.ref.scalefactor:=scalefactor;
              exit;
            end;
          if dest.opr.ref.index=reg then
            begin
              Inc(dest.opr.ref.scalefactor,scalefactor);
              exit;
            end;
          Message(asmr_e_multiple_index);
        end;

      var
        tmplocal: TOprRec;
        segreg: TRegister;
      begin
        case dest.opr.typ of
          OPR_REFERENCE:
            begin
              case src.opr.typ of
                OPR_REFERENCE:
                  begin
                    AddRegister(src.opr.ref.base,1);
                    AddRegister(src.opr.ref.index,src.opr.ref.scalefactor);
                    Inc(dest.opr.ref.offset,src.opr.ref.offset);
                    Inc(dest.opr.constoffset,src.opr.constoffset);
                    dest.haslabelref:=dest.haslabelref or src.haslabelref;
                    dest.hasproc:=dest.hasproc or src.hasproc;
                    dest.hasvar:=dest.hasvar or src.hasvar;
                    if assigned(src.opr.ref.symbol) then
                      begin
                        if assigned(dest.opr.ref.symbol) then
                          Message(asmr_e_cant_have_multiple_relocatable_symbols);
                        dest.opr.ref.symbol:=src.opr.ref.symbol;
                      end;
                    if assigned(src.opr.ref.relsymbol) then
                      begin
                        if assigned(dest.opr.ref.relsymbol) then
                          Message(asmr_e_cant_have_multiple_relocatable_symbols);
                        dest.opr.ref.relsymbol:=src.opr.ref.relsymbol;
                      end;
                    if dest.opr.ref.refaddr=addr_no then
                      dest.opr.ref.refaddr:=src.opr.ref.refaddr;
                  end;
                OPR_LOCAL:
                  begin
                    tmplocal:=src.opr;
                    if dest.opr.ref.base<>NR_NO then
                      begin
                        if tmplocal.localindexreg=NR_NO then
                          begin
                            tmplocal.localindexreg:=dest.opr.ref.base;
                            tmplocal.localscale:=0;
                          end
                        else if tmplocal.localindexreg=dest.opr.ref.base then
                          tmplocal.localscale:=Min(tmplocal.localscale,1)+1
                        else
                          Message(asmr_e_multiple_index);
                      end;
                    if dest.opr.ref.index<>NR_NO then
                      begin
                        if tmplocal.localindexreg=NR_NO then
                          begin
                            tmplocal.localindexreg:=dest.opr.ref.index;
                            tmplocal.localscale:=dest.opr.ref.scalefactor;
                          end
                        else if tmplocal.localindexreg=dest.opr.ref.index then
                          tmplocal.localscale:=Min(tmplocal.localscale,1)+Min(dest.opr.ref.scalefactor,1)
                        else
                          Message(asmr_e_multiple_index);
                      end;
                    Inc(tmplocal.localconstoffset,dest.opr.constoffset);
                    Inc(tmplocal.localsymofs,dest.opr.ref.offset);
                    dest.opr:=tmplocal;
                  end;
                else
                  internalerror(2018030711);
              end;
            end;
          OPR_LOCAL:
            begin
              case src.opr.typ of
                OPR_REFERENCE:
                  begin
                    if src.opr.ref.base<>NR_NO then
                      begin
                        if dest.opr.localindexreg=NR_NO then
                          begin
                            dest.opr.localindexreg:=src.opr.ref.base;
                            dest.opr.localscale:=0;
                          end
                        else if dest.opr.localindexreg=src.opr.ref.base then
                          dest.opr.localscale:=Min(dest.opr.localscale,1)+1
                        else
                          Message(asmr_e_multiple_index);
                      end;
                    if src.opr.ref.index<>NR_NO then
                      begin
                        if dest.opr.localindexreg=NR_NO then
                          begin
                            dest.opr.localindexreg:=src.opr.ref.index;
                            dest.opr.localscale:=src.opr.ref.scalefactor;
                          end
                        else if dest.opr.localindexreg=src.opr.ref.index then
                          dest.opr.localscale:=Min(dest.opr.localscale,1)+Min(src.opr.ref.scalefactor,1)
                        else
                          Message(asmr_e_multiple_index);
                      end;
                    Inc(dest.opr.localconstoffset,src.opr.constoffset);
                    Inc(dest.opr.localsymofs,src.opr.ref.offset);
                  end;
                OPR_LOCAL:
                  Message(asmr_e_no_local_or_para_allowed);
                else
                  internalerror(2018030713);
              end;
            end;
          else
            internalerror(2018030712);
        end;
      end;


    function tz80reader.is_locallabel(const s: string): boolean;
      begin
        is_locallabel:=(length(s)>1) and (s[1]='@');
      end;


    function tz80reader.is_asmopcode(const s: string):boolean;
      begin
        actcondition:=C_None;
        actopcode:=tasmop(PtrUInt(iasmops.Find(s)));
        if actopcode<>A_NONE then
          begin
            actasmtoken:=AS_OPCODE;
            is_asmopcode:=true;
          end
        else
          is_asmopcode:=false;
      end;


    function tz80reader.is_asmdirective(const s: string): boolean;
      var
        i : tasmtoken;
        hs : string;
      begin
        hs:=lower(s);
        for i:=firstdirective to lastdirective do
         if hs=token2str[i] then
          begin
            actasmtoken:=i;
            is_asmdirective:=true;
            exit;
          end;
        is_asmdirective:=false;
      end;


    function tz80reader.is_register(const s:string):boolean;
      begin
        is_register:=false;
        actasmregister:=std_regnum_search(lower(s));
        if actasmregister<>NR_NO then
          begin
            is_register:=true;
            actasmtoken:=AS_REGISTER;
          end;
      end;


    function tz80reader.is_condition(const s: string): boolean;
      var
        condstr: string;
        cond: TAsmCond;
      begin
        is_condition:=false;
        actasmcond:=C_None;
        condstr:=lower(s);
        for cond in TAsmCond do
          if (cond<>C_None) and (cond2str[cond]=condstr) then
            begin
              is_condition:=true;
              actasmtoken:=AS_CONDITION;
              actasmcond:=cond;
              exit;
            end;
      end;


    function tz80reader.is_targetdirective(const s: string): boolean;
      begin
        result:=false;
      end;

    procedure tz80reader.BuildRecordOffsetSize(const expr: string; out
        offset: tcgint; out size: tcgint; out mangledname: string;
        needvmtofs: boolean; out hastypecast: boolean);
      var
        s: string;
      Begin
        offset:=0;
        size:=0;
        mangledname:='';
        hastypecast:=false;
        s:=expr;
        while (actasmtoken=AS_DOT) do
         begin
           Consume(AS_DOT);
           if actasmtoken in [AS_ID,AS_REGISTER] then
             begin
               s:=s+'.'+actasmpattern;
               consume(actasmtoken);
             end
           else
            begin
              Consume(AS_ID);
              RecoverConsume(true);
              break;
            end;
         end;
        if not GetRecordOffsetSize(s,offset,size,mangledname,needvmtofs,hastypecast) then
          Message(asmr_e_building_record_offset);
      end;

    procedure tz80reader.BuildConstSymbolExpression(
        in_flags: tconstsymbolexpressioninputflags; out value: tcgint; out
        asmsym: string; out asmsymtyp: TAsmsymtype; out size: tcgint; out
        out_flags: tconstsymbolexpressionoutputflags);
      var
        tempstr,expr,hs,mangledname : string;
        parenlevel : longint;
        l,k : tcgint;
        hasparen,
        errorflag,
        needvmtofs : boolean;
        prevtok : tasmtoken;
        hl : tasmlabel;
        hssymtyp : Tasmsymtype;
        def : tdef;
        sym : tsym;
        srsymtable : TSymtable;
        hastypecast : boolean;
      Begin
        { reset }
        value:=0;
        asmsym:='';
        asmsymtyp:=AT_DATA;
        size:=0;
        out_flags:=[];
        errorflag:=FALSE;
        tempstr:='';
        expr:='';
        if cseif_startingminus in in_flags then
          expr:='-';
        inexpression:=TRUE;
        parenlevel:=0;
        sym:=nil;
        needvmtofs:=FALSE;
        Repeat
          { Support ugly delphi constructs like: [ECX].1+2[EDX] }
          if (cseif_isref in in_flags) and (actasmtoken=AS_LBRACKET) then
            break;
          if (cseif_referencelike in in_flags) and
             (actasmtoken in [AS_LBRACKET,AS_RBRACKET]) then
            case actasmtoken of
              AS_LBRACKET:
                begin
                  Consume(AS_LBRACKET);
                  if (length(expr)>0) and
                     not (expr[length(expr)] in ['+','-']) then
                    expr:=expr+'+';
                  expr:=expr+'[';
                end;
              AS_RBRACKET:
                begin
                  Consume(AS_RBRACKET);
                  expr:=expr+']';
                end;
              else
                ;
            end;
          Case actasmtoken of
            AS_LPAREN:
              Begin
                Consume(AS_LPAREN);
                expr:=expr + '(';
                inc(parenlevel);
              end;
            AS_RPAREN:
              Begin
                { Keep the AS_PAREN in actasmtoken, it is maybe a typecast }
                if parenlevel=0 then
                  break;
                Consume(AS_RPAREN);
                expr:=expr + ')';
                dec(parenlevel);
              end;
            AS_SHL:
              Begin
                Consume(AS_SHL);
                expr:=expr + '<';
              end;
            AS_SHR:
              Begin
                Consume(AS_SHR);
                expr:=expr + '>';
              end;
            AS_SLASH:
              Begin
                Consume(AS_SLASH);
                expr:=expr + '/';
              end;
            AS_MOD:
              Begin
                Consume(AS_MOD);
                expr:=expr + '%';
              end;
            AS_STAR:
              Begin
                Consume(AS_STAR);
                if (cseif_isref in in_flags) and (actasmtoken=AS_REGISTER) then
                 break;
                expr:=expr + '*';
              end;
            AS_PLUS:
              Begin
                Consume(AS_PLUS);
                if (cseif_isref in in_flags) and ((actasmtoken=AS_REGISTER) or (actasmtoken=AS_LBRACKET)) then
                 break;
                expr:=expr + '+';
              end;
            AS_MINUS:
              Begin
                Consume(AS_MINUS);
                expr:=expr + '-';
              end;
            AS_AND:
              Begin
                Consume(AS_AND);
                expr:=expr + '&';
              end;
            AS_NOT:
              Begin
                Consume(AS_NOT);
                expr:=expr + '~';
              end;
            AS_XOR:
              Begin
                Consume(AS_XOR);
                expr:=expr + '^';
              end;
            AS_OR:
              Begin
                Consume(AS_OR);
                expr:=expr + '|';
              end;
            AS_INTNUM:
              Begin
                expr:=expr + actasmpattern;
                Consume(AS_INTNUM);
              end;
{$ifdef i8086}
            AS_SEG:
              begin
                include(out_flags,cseof_isseg);
                Consume(actasmtoken);
                if actasmtoken<>AS_ID then
                 Message(asmr_e_seg_without_identifier);
              end;
{$endif i8086}
            AS_VMTOFFSET,
            AS_OFFSET:
              begin
                if (actasmtoken = AS_OFFSET) then
                  begin
                    include(in_flags,cseif_needofs);
                    include(out_flags,cseof_hasofs);
                  end
                else
                  needvmtofs:=true;
                Consume(actasmtoken);
                if actasmtoken<>AS_ID then
                 Message(asmr_e_offset_without_identifier);
              end;
            AS_SIZEOF,
            AS_TYPE:
              begin
                l:=0;
                hasparen:=false;
                Consume(actasmtoken);
                if actasmtoken=AS_LPAREN then
                  begin
                    hasparen:=true;
                    Consume(AS_LPAREN);
                  end;
                if actasmtoken<>AS_ID then
                 Message(asmr_e_type_without_identifier)
                else
                 begin
                   tempstr:=actasmpattern;
                   Consume(AS_ID);
                   if actasmtoken=AS_DOT then
                     begin
                       BuildRecordOffsetSize(tempstr,k,l,mangledname,false,hastypecast);
                       if mangledname<>'' then
                         { procsym }
                         Message(asmr_e_wrong_sym_type);
                       if hastypecast then

                     end
                   else
                    begin
                      asmsearchsym(tempstr,sym,srsymtable);
                      if assigned(sym) then
                       begin
                         case sym.typ of
                           staticvarsym,
                           localvarsym,
                           paravarsym :
                             l:=tabstractvarsym(sym).getsize;
                           typesym :
                             l:=ttypesym(sym).typedef.size;
                           else
                             Message(asmr_e_wrong_sym_type);
                         end;
                       end
                      else
                       Message1(sym_e_unknown_id,tempstr);
                    end;
                 end;
                str(l, tempstr);
                expr:=expr + tempstr;
                if hasparen then
                  Consume(AS_RPAREN);
              end;
            //AS_PTR :
            //  begin
            //    { Support ugly delphi constructs like <constant> PTR [ref] }
            //    break;
            //  end;
            AS_STRING:
              begin
                l:=0;
                case Length(actasmpattern) of
                 1 :
                  l:=ord(actasmpattern[1]);
                 2 :
                  l:=ord(actasmpattern[2]) + ord(actasmpattern[1]) shl 8;
                 3 :
                  l:=ord(actasmpattern[3]) +
                     Ord(actasmpattern[2]) shl 8 + ord(actasmpattern[1]) shl 16;
                 4 :
                  l:=ord(actasmpattern[4]) + ord(actasmpattern[3]) shl 8 +
                     Ord(actasmpattern[2]) shl 16 + ord(actasmpattern[1]) shl 24;
                else
                  Message1(asmr_e_invalid_string_as_opcode_operand,actasmpattern);
                end;
                str(l, tempstr);
                expr:=expr + tempstr;
                Consume(AS_STRING);
              end;
            AS_ID:
              begin
                hs:='';
                hssymtyp:=AT_DATA;
                def:=nil;
                tempstr:=actasmpattern;
                prevtok:=prevasmtoken;
                { stop parsing a constant expression if we find an opcode after a
                  non-operator like "db $66 mov eax,ebx" }
                if (prevtok in [AS_ID,AS_INTNUM,AS_RPAREN]) and
                   is_asmopcode(actasmpattern) then
                  break;
                consume(AS_ID);
                if (tempstr='@CODE') or (tempstr='@DATA') then
                 begin
                   if asmsym='' then
                     begin
                       asmsym:=tempstr;
                       asmsymtyp:=AT_SECTION;
                     end
                   else
                    Message(asmr_e_cant_have_multiple_relocatable_symbols);
                 end
                else if SearchIConstant(tempstr,l) then
                 begin
                   str(l, tempstr);
                   expr:=expr + tempstr;
                 end
                else
                 begin
                   if is_locallabel(tempstr) then
                    begin
                      CreateLocalLabel(tempstr,hl,false);
                      hs:=hl.name;
                      hssymtyp:=AT_FUNCTION;
                    end
                   else
                    if SearchLabel(tempstr,hl,false) then
                      begin
                        hs:=hl.name;
                        hssymtyp:=AT_FUNCTION;
                      end
                   else
                    begin
                      asmsearchsym(tempstr,sym,srsymtable);
                      if assigned(sym) then
                       begin
                         case sym.typ of
                           staticvarsym :
                             begin
                               hs:=tstaticvarsym(sym).mangledname;
                               def:=tstaticvarsym(sym).vardef;
                             end;
                           localvarsym,
                           paravarsym :
                             begin
                               Message(asmr_e_no_local_or_para_allowed);
                             end;
                           procsym :
                             begin
                               if Tprocsym(sym).ProcdefList.Count>1 then
                                Message(asmr_w_calling_overload_func);
                               hs:=tprocdef(tprocsym(sym).ProcdefList[0]).mangledname;
{$ifdef i8086}
                               if is_proc_far(tprocdef(tprocsym(sym).ProcdefList[0]))
                                  and not (po_interrupt in tprocdef(tprocsym(sym).ProcdefList[0]).procoptions) then
                                 include(out_flags,cseof_is_farproc_entry)
                               else
                                 exclude(out_flags,cseof_is_farproc_entry);
{$endif i8086}
                               hssymtyp:=AT_FUNCTION;
                             end;
                           typesym :
                             begin
                               if not(ttypesym(sym).typedef.typ in [recorddef,objectdef]) then
                                Message(asmr_e_wrong_sym_type);
                               size:=ttypesym(sym).typedef.size;
                             end;
                           fieldvarsym :
                             begin
                               tempstr:=upper(tdef(sym.owner.defowner).GetTypeName)+'.'+tempstr;
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
                      if asmsym='' then
                        begin
                          asmsym:=hs;
                          asmsymtyp:=hssymtyp;
                        end
                      else
                       Message(asmr_e_cant_have_multiple_relocatable_symbols);
                      if (expr='') or (expr[length(expr)]='+') then
                       begin
                         { don't remove the + if there could be a record field }
                         if actasmtoken<>AS_DOT then
                          delete(expr,length(expr),1);
                       end
                      else if (cseif_needofs in in_flags) then
                        begin
                          if (prevtok<>AS_OFFSET) then
                            Message(asmr_e_need_offset);
                        end
                      else
                        Message(asmr_e_only_add_relocatable_symbol);
                    end;
                   if (actasmtoken=AS_DOT) or
                      (assigned(sym) and
                       is_normal_fieldvarsym(sym)) then
                     begin
                      BuildRecordOffsetSize(tempstr,l,size,hs,needvmtofs,hastypecast);
                      if hs <> '' then
                        hssymtyp:=AT_FUNCTION
                      else
                        begin
                          str(l, tempstr);
                          expr:=expr + tempstr;
                        end
                    end
                   else if (actasmtoken<>AS_DOT) and
                           assigned(sym) and
                           (sym.typ=typesym) and
                           (ttypesym(sym).typedef.typ in [recorddef,objectdef]) then
                     begin
                       { just a record type (without being followed by dot)
                         evaluates to 0. Ugly, but TP7 compatible. }
                       expr:=expr+'0';
                     end
                   else
                    begin
                      if (expr='') or (expr[length(expr)] in ['+','-','/','*']) then
                       delete(expr,length(expr),1);
                    end;
                   if (actasmtoken=AS_LBRACKET) and
                      assigned(def) and
                      (def.typ=arraydef) then
                     begin
                       consume(AS_LBRACKET);
                       l:=BuildConstExpression;
                       if l<tarraydef(def).lowrange then
                         begin
                           Message(asmr_e_constant_out_of_bounds);
                           l:=0;
                         end
                       else
                         l:=(l-tarraydef(def).lowrange)*tarraydef(def).elesize;
                       str(l, tempstr);
                       expr:=expr + '+' + tempstr;
                       consume(AS_RBRACKET);
                     end;
                 end;
                { check if there are wrong operator used like / or mod etc. }
                if (hs<>'') and not(actasmtoken in [AS_MINUS,AS_PLUS,AS_COMMA,AS_SEPARATOR,AS_END,AS_RBRACKET]) then
                 Message(asmr_e_only_add_relocatable_symbol);
              end;
            //AS_ALIGN,
            AS_DEFB,
            AS_DEFW,
            AS_END,
            AS_RBRACKET,
            AS_SEPARATOR,
            AS_COMMA,
            AS_COLON:
              break;
          else
            begin
              { write error only once. }
              if not errorflag then
                Message(asmr_e_invalid_constant_expression);
              { consume tokens until we find COMMA or SEPARATOR }
              Consume(actasmtoken);
              errorflag:=TRUE;
            end;
          end;
        Until false;
        { calculate expression }
        if not ErrorFlag then
          value:=CalculateExpression(expr)
        else
          value:=0;
        { no longer in an expression }
        inexpression:=FALSE;
      end;


    function tz80reader.BuildConstExpression: longint;
      var
        l,size : tcgint;
        hs : string;
        hssymtyp : TAsmsymtype;
        out_flags : tconstsymbolexpressionoutputflags;
      begin
        BuildConstSymbolExpression([],l,hs,hssymtyp,size,out_flags);
        if hs<>'' then
         Message(asmr_e_relocatable_symbol_not_allowed);
        BuildConstExpression:=l;
      end;


    function tz80reader.BuildRefConstExpression(out size: tcgint;
        startingminus: boolean): longint;
      var
        l : tcgint;
        hs : string;
        hssymtyp : TAsmsymtype;
        in_flags : tconstsymbolexpressioninputflags;
        out_flags : tconstsymbolexpressionoutputflags;
      begin
        in_flags:=[cseif_isref];
        if startingminus then
          include(in_flags,cseif_startingminus);
        BuildConstSymbolExpression(in_flags,l,hs,hssymtyp,size,out_flags);
        if hs<>'' then
         Message(asmr_e_relocatable_symbol_not_allowed);
        BuildRefConstExpression:=l;
      end;


    procedure tz80reader.BuildConstantOperand(oper: tz80operand);
      var
        l,size : tcgint;
        tempstr : string;
        tempsymtyp : tasmsymtype;
        cse_out_flags : tconstsymbolexpressionoutputflags;
      begin
        if not (oper.opr.typ in [OPR_NONE,OPR_CONSTANT]) then
          Message(asmr_e_invalid_operand_type);
        BuildConstSymbolExpression([cseif_needofs],l,tempstr,tempsymtyp,size,cse_out_flags);
        if tempstr<>'' then
          begin
            oper.opr.typ:=OPR_SYMBOL;
            oper.opr.symofs:=l;
            oper.opr.symbol:=current_asmdata.RefAsmSymbol(tempstr,tempsymtyp);
            oper.opr.symseg:=cseof_isseg in cse_out_flags;
            oper.opr.sym_farproc_entry:=cseof_is_farproc_entry in cse_out_flags;
          end
        else
          if oper.opr.typ=OPR_NONE then
            begin
              oper.opr.typ:=OPR_CONSTANT;
              oper.opr.val:=l;
            end
          else
            inc(oper.opr.val,l);
      end;


    procedure tz80reader.BuildReference(oper: tz80operand);
      var
        scale : byte;
        k,l,size : tcgint;
        tempstr,hs : string;
        tempsymtyp : tasmsymtype;
        code : integer;
        hreg : tregister;
        GotStar,GotOffset,HadVar,
        GotPlus,Negative,BracketlessReference : boolean;
        hl : tasmlabel;
        hastypecast: boolean;
        tmpoper: tz80operand;
        cse_in_flags: tconstsymbolexpressioninputflags;
        cse_out_flags: tconstsymbolexpressionoutputflags;
      begin
        if actasmtoken=AS_LPAREN then
          begin
            Consume(AS_LPAREN);
            BracketlessReference:=false;
          end
        else
          BracketlessReference:=true;
        if not(oper.opr.typ in [OPR_LOCAL,OPR_REFERENCE]) then
          oper.InitRef;
        GotStar:=false;
        GotPlus:=true;
        GotOffset:=false;
        Negative:=false;
        Scale:=0;
        repeat
          if GotOffset and (actasmtoken<>AS_ID) then
            Message(asmr_e_invalid_reference_syntax);

          Case actasmtoken of
            AS_ID, { Constant reference expression OR variable reference expression }
            AS_VMTOFFSET:
              Begin
                if not GotPlus then
                  Message(asmr_e_invalid_reference_syntax);
                GotStar:=false;
                GotPlus:=false;
                if (actasmtoken = AS_VMTOFFSET) or
                   (SearchIConstant(actasmpattern,l) or
                    SearchRecordType(actasmpattern)) then
                 begin
                   l:=BuildRefConstExpression(size,negative);
                   if size<>0 then
                     oper.SetSize(size,false);
                   negative:=false;   { "l" was negated if necessary }
                   GotPlus:=(prevasmtoken=AS_PLUS);
                   GotStar:=(prevasmtoken=AS_STAR);
                   case oper.opr.typ of
                     OPR_LOCAL :
                       begin
                         if GotStar then
                           Message(asmr_e_invalid_reference_syntax);
                         Inc(oper.opr.localsymofs,l);
                       end;
                     OPR_REFERENCE :
                       begin
                         if GotStar then
                          oper.opr.ref.scalefactor:=l
                         else
                           Inc(oper.opr.ref.offset,l);
                       end;
                     else
                       internalerror(2019050715);
                   end;
                 end
                else
                 Begin
                   if negative and not oper.hasvar then
                     Message(asmr_e_only_add_relocatable_symbol)
                   else if oper.hasvar and not GotOffset and
                           (not negative or assigned(oper.opr.ref.relsymbol)) then
                     Message(asmr_e_cant_have_multiple_relocatable_symbols);
                   HadVar:=oper.hasvar and GotOffset;
                   tempstr:=actasmpattern;
                   Consume(AS_ID);
                   { typecasting? }
                   if (actasmtoken=AS_LPAREN) and
                      SearchType(tempstr,l) then
                    begin
                      oper.hastype:=true;
                      oper.typesize:=l;
                      Consume(AS_LPAREN);
                      BuildOperand(oper,true);
                      Consume(AS_RPAREN);
                    end
                   else
                    if is_locallabel(tempstr) then
                      begin
                        CreateLocalLabel(tempstr,hl,false);
                        oper.InitRef;
                        oper.haslabelref:=true;
                        if not negative then
                          begin
                            oper.opr.ref.symbol:=hl;
                            oper.hasvar:=true;
                          end
                        else
                          oper.opr.ref.relsymbol:=hl;
{$ifdef i8086}
                        if oper.opr.ref.segment=NR_NO then
                          oper.opr.ref.segment:=NR_CS;
{$endif i8086}
                      end
                   else
                    if oper.SetupVar(tempstr,GotOffset) then
                     begin
                       { convert OPR_LOCAL register para into a reference base }
                       if (oper.opr.typ=OPR_LOCAL) and
                          AsmRegisterPara(oper.opr.localsym) then
                         oper.InitRefConvertLocal
                       else
                         begin
{$ifdef x86_64}
                           if actasmtoken=AS_WRT then
                             begin
                               if (oper.opr.typ=OPR_REFERENCE) then
                                 begin
                                   Consume(AS_WRT);
                                   Consume(AS___GOTPCREL);
                                   if (oper.opr.ref.base<>NR_NO) or
                                      (oper.opr.ref.index<>NR_NO) or
                                      (oper.opr.ref.offset<>0) then
                                     Message(asmr_e_wrong_gotpcrel_intel_syntax);
                                   if tf_no_pic_supported in target_info.flags then
                                     Message(asmr_e_no_gotpcrel_support);
                                   oper.opr.ref.refaddr:=addr_pic;
                                   oper.opr.ref.base:=NR_RIP;
                                 end
                               else
                                 message(asmr_e_invalid_reference_syntax);
                             end;
{$endif x86_64}
                         end;
                     end
                   else
                     Message1(sym_e_unknown_id,tempstr);
                   { record.field ? }
                   if actasmtoken=AS_DOT then
                    begin
                      BuildRecordOffsetSize(tempstr,l,k,hs,false,hastypecast);
                      if (hs<>'') then
                        Message(asmr_e_invalid_symbol_ref);
                      case oper.opr.typ of
                        OPR_LOCAL :
                          inc(oper.opr.localsymofs,l);
                        OPR_REFERENCE :
                          inc(oper.opr.ref.offset,l);
                        else
                          internalerror(2019050716);
                      end;
                      if hastypecast then
                       oper.hastype:=true;
                      oper.SetSize(k,false);
                    end;
                   if GotOffset then
                    begin
                      if oper.hasvar and (oper.opr.ref.base=current_procinfo.framepointer) then
                       begin
                         if (oper.opr.typ=OPR_REFERENCE) then
                           oper.opr.ref.base:=NR_NO;
                         oper.hasvar:=hadvar;
                       end
                      else
                       begin
                         if oper.hasvar and hadvar then
                          Message(asmr_e_cant_have_multiple_relocatable_symbols);
                         { should we allow ?? }
                       end;
                    end;
                 end;
                GotOffset:=false;
              end;

            AS_PLUS :
              Begin
                Consume(AS_PLUS);
                Negative:=false;
                GotPlus:=true;
                GotStar:=false;
                Scale:=0;
              end;

            AS_DOT :
              Begin
                { Handle like a + }
                Consume(AS_DOT);
                Negative:=false;
                GotPlus:=true;
                GotStar:=false;
                Scale:=0;
              end;

            AS_MINUS :
              begin
                Consume(AS_MINUS);
                Negative:=true;
                GotPlus:=true;
                GotStar:=false;
                Scale:=0;
              end;

            AS_STAR : { Scaling, with eax*4 order }
              begin
                Consume(AS_STAR);
                hs:='';
                l:=0;
                case actasmtoken of
                  AS_ID,
                  AS_LPAREN :
                    l:=BuildConstExpression;
                  AS_INTNUM:
                    Begin
                      hs:=actasmpattern;
                      Consume(AS_INTNUM);
                    end;
                  AS_REGISTER :
                    begin
                      case oper.opr.typ of
                        OPR_REFERENCE :
                          begin
                            if oper.opr.ref.scalefactor=0 then
                              begin
                                if scale<>0 then
                                  begin
                                    oper.opr.ref.scalefactor:=scale;
                                    scale:=0;
                                  end
                                else
                                 Message(asmr_e_wrong_scale_factor);
                              end
                            else
                              Message(asmr_e_invalid_reference_syntax);
                          end;
                        OPR_LOCAL :
                          begin
                            if oper.opr.localscale=0 then
                              begin
                                if scale<>0 then
                                  begin
                                    oper.opr.localscale:=scale;
                                    scale:=0;
                                  end
                                else
                                 Message(asmr_e_wrong_scale_factor);
                              end
                            else
                              Message(asmr_e_invalid_reference_syntax);
                          end;
                        else
                          internalerror(2019050719);
                      end;
                    end;
                  else
                    Message(asmr_e_invalid_reference_syntax);
                end;
                if actasmtoken<>AS_REGISTER then
                  begin
                    if hs<>'' then
                      val(hs,l,code);
                    case oper.opr.typ of
                      OPR_REFERENCE :
                        oper.opr.ref.scalefactor:=l;
                      OPR_LOCAL :
                        oper.opr.localscale:=l;
                      else
                        internalerror(2019050717);
                    end;
                    if l>9 then
                      Message(asmr_e_wrong_scale_factor);
                  end;
                GotPlus:=false;
                GotStar:=false;
              end;

            AS_REGISTER :
              begin
                hreg:=actasmregister;

                Consume(AS_REGISTER);

                if not((GotPlus and (not Negative)) or
                       GotStar) then
                  Message(asmr_e_invalid_reference_syntax);
                { this register will be the index:
                   1. just read a *
                   2. next token is a *
                   3. base register is already used }
                case oper.opr.typ of
                  OPR_LOCAL :
                    begin
                      if (oper.opr.localindexreg<>NR_NO) then
                        Message(asmr_e_multiple_index);
                      oper.opr.localindexreg:=hreg;
                      if scale<>0 then
                        begin
                          oper.opr.localscale:=scale;
                          scale:=0;
                        end;
                    end;
                  OPR_REFERENCE :
                    begin
                      if (GotStar) or
                         (actasmtoken=AS_STAR) or
                         (oper.opr.ref.base<>NR_NO) then
                       begin
                         if (oper.opr.ref.index<>NR_NO) then
                          Message(asmr_e_multiple_index);
                         oper.opr.ref.index:=hreg;
                         if scale<>0 then
                           begin
                             oper.opr.ref.scalefactor:=scale;
                             scale:=0;
                           end;
                       end
                      else
                        begin
                          oper.opr.ref.base:=hreg;
{$ifdef x86_64}
                          { non-GOT based RIP-relative accesses are also position-independent }
                          if (oper.opr.ref.base=NR_RIP) and
                             (oper.opr.ref.refaddr<>addr_pic) then
                            oper.opr.ref.refaddr:=addr_pic_no_got;
{$endif x86_64}
                        end;
                    end;
                  else
                    internalerror(2019050718);
                end;
                GotPlus:=false;
                GotStar:=false;
              end;

            AS_OFFSET :
              begin
                Consume(AS_OFFSET);
                GotOffset:=true;
              end;

            AS_TYPE,
            AS_NOT,
            AS_STRING,
            AS_INTNUM,
            AS_LPAREN : { Constant reference expression }
              begin
                if not GotPlus and not GotStar then
                  Message(asmr_e_invalid_reference_syntax);
                cse_in_flags:=[cseif_needofs,cseif_isref];
                if GotPlus and negative then
                  include(cse_in_flags,cseif_startingminus);
                BuildConstSymbolExpression(cse_in_flags,l,tempstr,tempsymtyp,size,cse_out_flags);
                { already handled by BuildConstSymbolExpression(); must be
                  handled there to avoid [reg-1+1] being interpreted as
                  [reg-(1+1)] }
                negative:=false;

                if tempstr<>'' then
                 begin
                   if GotStar then
                    Message(asmr_e_only_add_relocatable_symbol);
                   if not assigned(oper.opr.ref.symbol) then
                     begin
                       oper.opr.ref.symbol:=current_asmdata.RefAsmSymbol(tempstr,tempsymtyp);
{$ifdef i8086}
                       if cseof_isseg in cse_out_flags then
                         begin
                           if not (oper.opr.ref.refaddr in [addr_fardataseg,addr_dgroup]) then
                             oper.opr.ref.refaddr:=addr_seg;
                         end
                       else if (tempsymtyp=AT_FUNCTION) and (oper.opr.ref.segment=NR_NO) then
                         oper.opr.ref.segment:=NR_CS;
{$endif i8086}
                     end
                   else
                    Message(asmr_e_cant_have_multiple_relocatable_symbols);
                 end;
                case oper.opr.typ of
                  OPR_REFERENCE :
                    begin
                      if GotStar then
                       oper.opr.ref.scalefactor:=l
                      else if (prevasmtoken = AS_STAR) then
                       begin
                         if scale<>0 then
                           scale:=l*scale
                         else
                           scale:=l;
                       end
                      else
                      begin
                        Inc(oper.opr.ref.offset,l);
                        Inc(oper.opr.constoffset,l);
                      end;
                    end;
                  OPR_LOCAL :
                    begin
                      if GotStar then
                       oper.opr.localscale:=l
                      else if (prevasmtoken = AS_STAR) then
                       begin
                         if scale<>0 then
                           scale:=l*scale
                         else
                           scale:=l;
                       end
                      else
                        Inc(oper.opr.localsymofs,l);
                    end;
                  else
                    internalerror(2019050714);
                end;
                GotPlus:=(prevasmtoken=AS_PLUS) or
                         (prevasmtoken=AS_MINUS);
                if GotPlus then
                  negative := prevasmtoken = AS_MINUS;
                GotStar:=(prevasmtoken=AS_STAR);
              end;

            //AS_LBRACKET :
            //  begin
            //    if (GotPlus and Negative) or GotStar then
            //      Message(asmr_e_invalid_reference_syntax);
            //    tmpoper:=Tz80Operand.create;
            //    BuildReference(tmpoper);
            //    AddReferences(oper,tmpoper);
            //    tmpoper.Free;
            //    GotPlus:=false;
            //    GotStar:=false;
            //  end;

            AS_RPAREN :
              begin
                if GotPlus or GotStar or BracketlessReference then
                  Message(asmr_e_invalid_reference_syntax);

                Consume(AS_RPAREN);



                if actasmtoken=AS_LPAREN then
                  begin
                    tmpoper:=Tz80Operand.create;
                    BuildReference(tmpoper);
                    AddReferences(oper,tmpoper);
                    tmpoper.Free;
                  end;
                break;
              end;

            AS_SEPARATOR,
            AS_END,
            AS_COMMA:
              begin
                if not BracketlessReference then
                  begin
                    Message(asmr_e_invalid_reference_syntax);
                    RecoverConsume(true);
                  end;
                break;
              end;

            else
              Begin
                Message(asmr_e_invalid_reference_syntax);
                RecoverConsume(true);
                break;
              end;
          end;
        until false;
      end;


    procedure tz80reader.BuildOperand(oper: tz80operand; istypecast: boolean);

      procedure AddLabelOperand(hl:tasmlabel);
      begin
        if (oper.opr.typ=OPR_NONE) and
           is_calljmp(actopcode) then
         begin
           oper.opr.typ:=OPR_SYMBOL;
           oper.opr.symbol:=hl;
         end
        else
         begin
           oper.InitRef;
           oper.opr.ref.symbol:=hl;
           oper.haslabelref:=true;
         end;
      end;

      var
        l: tcgint;
        tsize: tcgint;
        expr: string;
        hl: tasmlabel;
      begin
        repeat
          case actasmtoken of
            AS_OFFSET,
            AS_SIZEOF,
            AS_VMTOFFSET,
            AS_TYPE,
            AS_NOT,
            AS_STRING,
            AS_PLUS,
            AS_MINUS,
//            AS_LPAREN,
            AS_INTNUM :
              begin
                case oper.opr.typ of
                  OPR_REFERENCE :
                    begin
                      l := BuildRefConstExpression(tsize);
                      if tsize<>0 then
                        oper.SetSize(tsize,false);
                      inc(oper.opr.ref.offset,l);
                      inc(oper.opr.constoffset,l);
                    end;
                  OPR_LOCAL :
                    begin
                      l := BuildConstExpression;
                      inc(oper.opr.localsymofs,l);
                      inc(oper.opr.localconstoffset,l);
                    end;

                  OPR_NONE,
                  OPR_CONSTANT :
                    BuildConstantOperand(oper);
                  else
                    Message(asmr_e_invalid_operand_type);
                end;
              end;

            AS_LPAREN:
              begin
                BuildReference(oper);
              end;

            AS_ID : { A constant expression, or a Variable ref. }
              Begin
                { Label or Special symbol reference? }
                if actasmpattern[1] = '@' then
                 Begin
                   if actasmpattern = '@RESULT' then
                    Begin
                      oper.SetupResult;
                      Consume(AS_ID);
                      expr:='result';
                    end
                   else
                    if (actasmpattern = '@CODE') or (actasmpattern = '@DATA') then
                     begin
                       Message(asmr_w_CODE_and_DATA_not_supported);
                       Consume(AS_ID);
                     end
                   else
                    { Local Label }
                    begin
                      CreateLocalLabel(actasmpattern,hl,false);
                      Consume(AS_ID);
                      AddLabelOperand(hl);
                    end;
                 end
                else
                { support result for delphi modes }
                 if (m_objpas in current_settings.modeswitches) and (actasmpattern='RESULT') then
                  begin
                    oper.SetUpResult;
                    Consume(AS_ID);
                    expr:='result';
                  end
                { probably a variable or normal expression }
                { or a procedure (such as in CALL ID)      }
                else
                 Begin
                   { is it a constant ? }
                   if SearchIConstant(actasmpattern,l) then
                    Begin
                      case oper.opr.typ of
                        OPR_REFERENCE :
                          begin
                            l := BuildRefConstExpression(tsize);
                            if tsize<>0 then
                              oper.SetSize(tsize,false);
                            inc(oper.opr.ref.offset,l);
                            inc(oper.opr.constoffset,l);
                          end;

                        OPR_LOCAL :
                          begin
                            l := BuildRefConstExpression(tsize);
                            if tsize<>0 then
                              oper.SetSize(tsize,false);
                            inc(oper.opr.localsymofs,l);
                            inc(oper.opr.localconstoffset,l);
                          end;
                        OPR_NONE,
                        OPR_CONSTANT :
                          BuildConstantOperand(oper);
                        else
                          Message(asmr_e_invalid_operand_type);
                      end;
                    end
                   else
                    { Check for pascal label }
                    if SearchLabel(actasmpattern,hl,false) then
                     begin
                       Consume(AS_ID);
                       AddLabelOperand(hl);
                     end
                    else
                    { is it a normal variable ? }
                     Begin
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
                                BuildOperand(oper,true);
                                { Delphi also supports Type(Register) and
                                  interprets it the same as Type([Register]).  }
                                if (oper.opr.typ = OPR_REGISTER) then
                                  { This also sets base to the register.  }
                                  oper.InitRef;
                                Consume(AS_RPAREN);
                              end;
                            //AS_LBRACKET :
                            //  begin
                            //    { Support Var.Type[Index] }
                            //    { Convert @label.Byte[1] to reference }
                            //    if oper.opr.typ=OPR_SYMBOL then
                            //      oper.initref;
                            //  end;
                            else
                              ;
                          end;
                        end
                       else
                        begin
                          if not oper.SetupVar(expr,false) then
                            Begin
                              { not a variable, check special variables.. }
                              if expr = 'SELF' then
                                begin
                                  oper.SetupSelf;
                                  expr:='self';
                                end
                              else
                                begin
                                  Message1(sym_e_unknown_id,expr);
                                  expr:='';
                                end;
                            end;
                          { indexed access to variable? }
                          //if actasmtoken=AS_LBRACKET then
                          //  begin
                          //    { ... then the operand size is not known anymore }
                          //    oper.size:=OS_NO;
                          //    BuildReference(oper);
                          //  end;
                        end;
                     end;
                 end;
              end;

            AS_REGISTER : { Register, a variable reference or a constant reference }
              begin
                Consume(AS_REGISTER);

                { Simple register }
                if (oper.opr.typ <> OPR_NONE) then
                  Message(asmr_e_syn_operand);
                oper.opr.typ:=OPR_REGISTER;
                oper.opr.reg:=actasmregister;
                oper.SetSize(tcgsize2size[reg_cgsize(oper.opr.reg)],true);
              end;

            AS_SEPARATOR,
            AS_END,
            AS_COMMA:
              begin
                break;
              end;

            else
              begin
                Message(asmr_e_syn_operand);
                RecoverConsume(true);
                break;
              end;
          end;
        until false;
      end;


    procedure tz80reader.BuildOpCode(instr: TZ80Instruction);
      var
        operandnum: Integer;
      begin
        instr.opcode:=actopcode;
        operandnum:=1;
        Consume(AS_OPCODE);
        { Zero operand opcode ?  }
        if actasmtoken in [AS_SEPARATOR,AS_END] then
          exit;
        { Condition (e.g. 'NC' in 'JP NC, label') }
        if actasmtoken=AS_CONDITION then
          begin
            instr.condition:=actasmcond;
            Consume(AS_CONDITION);
            if actasmtoken in [AS_SEPARATOR,AS_END] then
              exit;
            if actasmtoken=AS_COMMA then
              Consume(AS_COMMA);
          end;
        { Read Operands }
        repeat
          case actasmtoken of
            { End of asm operands for this opcode }
            AS_END,
            AS_SEPARATOR :
              break;

            { Operand delimiter }
            AS_COMMA :
              begin
                { should have something before the comma }
                if instr.operands[operandnum].opr.typ=OPR_NONE then
                  Message(asmr_e_syntax_error);
                if operandnum >= max_operands then
                  Message(asmr_e_too_many_operands)
                else
                  Inc(operandnum);
                Consume(AS_COMMA);
              end;
            else
              BuildOperand(instr.Operands[operandnum] as tz80operand,false);
          end;
        until false;
        instr.ops:=operandnum;
      end;


    procedure tz80reader.BuildConstant(constsize: byte);
      var
       asmsymtyp : TAsmSymType;
       asmsym,
       expr : string;
       value, sz : tcgint;
       inflags : tconstsymbolexpressioninputflags;
       outflags : tconstsymbolexpressionoutputflags;
      begin
        repeat
          case actasmtoken of
            AS_STRING:
              Begin
                expr:=actasmpattern;
                if length(expr) > 1 then
                 Message(asmr_e_string_not_allowed_as_const);
                Consume(AS_STRING);
                case actasmtoken of
                  AS_COMMA: Consume(AS_COMMA);
                  AS_END,
                  AS_SEPARATOR: ;
                else
                  Message(asmr_e_invalid_string_expression);
                end; { end case }
                ConcatString(curlist,expr);
              end;
            AS_INTNUM,
            AS_PLUS,
            AS_MINUS,
            AS_LPAREN,
            AS_TYPE,
            AS_SIZEOF,
            AS_NOT,
            AS_VMTOFFSET,
            AS_ID :
              begin
                inflags:=[];
                BuildConstSymbolExpression(inflags,value,asmsym,asmsymtyp,sz,outflags);
                if asmsym<>'' then
                  begin
                    if constsize<>sizeof(pint) then
                      Message(asmr_w_32bit_const_for_address);
                     ConcatConstSymbol(curlist,asmsym,'',asmsymtyp,value,constsize,true)
                  end
                else
                  ConcatConstant(curlist,value,constsize);
              end;
            AS_COMMA:
              Consume(AS_COMMA);
            AS_END,
            AS_SEPARATOR:
              break;
            else
              begin
                Message(asmr_e_syn_constant);
                RecoverConsume(false);
              end
          end; { end case }
        until false;
      end;

    procedure tz80reader.handleopcode;
      var
        instr: TZ80Instruction;
      begin
        instr:=TZ80Instruction.create(TZ80Operand);
        BuildOpcode(instr);
        with instr do
          begin
            //CheckNonCommutativeOpcodes;
            //AddReferenceSizes;
            //SetInstructionOpsize;
            //CheckOperandSizes;
            ConcatInstruction(curlist);
          end;
        instr.Free;
      end;


    procedure tz80reader.ConvertCalljmp(instr : tz80instruction);
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


    function tz80reader.Assemble: tlinkedlist;
      var
        hl: tasmlabel;
        sectionname: String;
        section: tai_section;
      begin
        Message1(asmr_d_start_reading,'Z80');
        firsttoken:=TRUE;
        { sets up all opcode and register tables in uppercase }
        if not _asmsorted then
          begin
            SetupTables;
            _asmsorted:=TRUE;
          end;
        curlist:=TAsmList.Create;

        { we might need to know which parameters are passed in registers }
        if not parse_generic then
          current_procinfo.generate_parameter_info;

        { start tokenizer }
        gettoken;
        { main loop }
        repeat
          case actasmtoken of
            AS_LLABEL:
              Begin
                if CreateLocalLabel(actasmpattern,hl,true) then
                  ConcatLabel(curlist,hl);
                Consume(AS_LLABEL);
              end;

            AS_LABEL:
              Begin
                if SearchLabel(upper(actasmpattern),hl,true) then
                  begin
                    if hl.is_public then
                      ConcatPublic(curlist,actasmpattern_origcase);
                    ConcatLabel(curlist,hl);
                  end
                else
                 Message1(asmr_e_unknown_label_identifier,actasmpattern);
                Consume(AS_LABEL);
              end;

            AS_END:
              begin
                break; { end assembly block }
              end;

            AS_SEPARATOR:
              begin
                Consume(AS_SEPARATOR);
              end;

            AS_DEFB :
              Begin
                inexpression:=true;
                Consume(AS_DEFB);
                BuildConstant(1);
                inexpression:=false;
              end;

            AS_DEFW :
              Begin
                inexpression:=true;
                Consume(AS_DEFW);
                BuildConstant(2);
                inexpression:=false;
              end;

            AS_AREA :
              begin
                Consume(AS_AREA);
                sectionname:=actasmpattern;
                {secflags:=[];
                secprogbits:=SPB_None;}
                Consume(AS_STRING);
                {if actasmtoken=AS_COMMA then
                  begin
                    Consume(AS_COMMA);
                    if actasmtoken=AS_STRING then
                      begin
                        case actasmpattern of
                          'a':
                            Include(secflags,SF_A);
                          'w':
                            Include(secflags,SF_W);
                          'x':
                            Include(secflags,SF_X);
                          '':
                            ;
                          else
                            Message(asmr_e_syntax_error);
                        end;
                        Consume(AS_STRING);
                        if actasmtoken=AS_COMMA then
                          begin
                            Consume(AS_COMMA);
                            if (actasmtoken=AS_MOD) or (actasmtoken=AS_AT) then
                              begin
                                Consume(actasmtoken);
                                if actasmtoken=AS_ID then
                                  begin
                                    case actasmpattern of
                                      'PROGBITS':
                                        secprogbits:=SPB_PROGBITS;
                                      'NOBITS':
                                        secprogbits:=SPB_NOBITS;
                                      'NOTE':
                                        secprogbits:=SPB_NOTE;
                                      else
                                        Message(asmr_e_syntax_error);
                                    end;
                                    Consume(AS_ID);
                                  end
                                else
                                  Message(asmr_e_syntax_error);
                              end
                            else
                              Message(asmr_e_syntax_error);
                          end;
                      end
                    else
                      Message(asmr_e_syntax_error);
                  end;}

                //curList.concat(tai_section.create(sec_user, actasmpattern, 0));
                section:=new_section(curlist, sec_user, sectionname, 0);
                //section.secflags:=secflags;
                //section.secprogbits:=secprogbits;
              end;

            AS_OPCODE:
              begin
                HandleOpCode;
              end;

            else
              begin
                Message(asmr_e_syntax_error);
                RecoverConsume(false);
              end;
          end;
        until false;
        { check that all referenced local labels are defined }
        checklocallabels;
        { Return the list in an asmnode }
        assemble:=curlist;
        Message1(asmr_d_finish_reading,'Z80');
      end;


{*****************************************************************************
                                     Initialize
*****************************************************************************}

const
{  asmmode_z80_att_info : tasmmodeinfo =
          (
            id    : asmmode_z80_gas;
            idtxt : 'GAS';
            casmreader : tz80attreader;
          );}

  asmmode_z80_standard_info : tasmmodeinfo =
          (
            id    : asmmode_standard;
            idtxt : 'STANDARD';
            casmreader : tz80reader;
          );

initialization
//  RegisterAsmMode(asmmode_z80_att_info);
  RegisterAsmMode(asmmode_z80_standard_info);
end.
