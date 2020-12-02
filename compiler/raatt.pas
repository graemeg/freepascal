{
    Copyright (c) 1998-2002 by Carl Eric Codere and Peter Vreman

    Does the parsing for the GAS styled inline assembler.

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
unit raatt;

{$i fpcdefs.inc}

  interface

    uses
      { common }
      cutils,cclasses,
      { global }
      globtype,
      { aasm }
      cpubase,cpuinfo,aasmbase,aasmtai,aasmdata,aasmcpu,
      { assembler reader }
      rabase,
      rasm,
      rautils,
      { symtable }
      symconst,
      { cg }
      cgbase;

    type
      tasmtoken = (
        AS_NONE,AS_LABEL,AS_LLABEL,AS_STRING,AS_INTNUM,
        AS_REALNUM,AS_COMMA,AS_LPAREN,
        AS_RPAREN,AS_COLON,AS_DOT,AS_PLUS,AS_MINUS,AS_STAR,
        AS_SEPARATOR,AS_ID,AS_REGISTER,AS_OPCODE,AS_SLASH,AS_DOLLAR,
        AS_HASH,AS_LSBRACKET,AS_RSBRACKET,AS_LBRACKET,AS_RBRACKET,
        AS_EQUAL,
        {------------------ Assembler directives --------------------}
        AS_DB,AS_DW,AS_DD,AS_DQ,AS_GLOBAL,
        AS_ALIGN,AS_BALIGN,AS_P2ALIGN,AS_ASCII,
        AS_ASCIIZ,AS_LCOMM,AS_COMM,AS_SINGLE,AS_DOUBLE,AS_EXTENDED,AS_CEXTENDED,
        AS_DATA,AS_TEXT,AS_INIT,AS_FINI,AS_RVA,
        AS_SET,AS_WEAK,AS_SECTION,AS_END,
        {------------------ Assembler Operators  --------------------}
        AS_TYPE,AS_SIZEOF,AS_VMTOFFSET,AS_MOD,AS_SHL,AS_SHR,AS_NOT,AS_AND,AS_OR,AS_XOR,AS_NOR,AS_AT,
        AS_RELTYPE, // common token for relocation types
        {------------------ Target-specific directive ---------------}
        AS_TARGET_DIRECTIVE
        );

        tasmkeyword = string[10];

    const
      { These tokens should be modified accordingly to the modifications }
      { in the different enumerations.                                   }
      firstdirective = AS_DB;
      lastdirective  = AS_END;

      token2str : array[tasmtoken] of tasmkeyword=(
        '','Label','LLabel','string','integer',
        'float',',','(',
        ')',':','.','+','-','*',
        ';','identifier','register','opcode','/','$',
        '#','{','}','[',']',
        '=',
        '.byte','.word','.long','.quad','.globl',
        '.align','.balign','.p2align','.ascii',
        '.asciz','.lcomm','.comm','.single','.double','.tfloat','.tcfloat',
        '.data','.text','.init','.fini','.rva',
        '.set','.weak','.section','END',
        'TYPE','SIZEOF','VMTOFFSET','%','<<','>>','!','&','|','^','~','@','reltype',
        'directive');

    type
       tattreader = class(tasmreader)
         actasmtoken    : tasmtoken;
         prevasmtoken   : tasmtoken;
         procedure SetupTables;
         procedure BuildConstant(constsize: byte);
         procedure BuildConstantOperand(oper : toperand);
         procedure BuildRealConstant(typ : tfloattype);
         procedure BuildStringConstant(asciiz: boolean);
         procedure BuildRva;
         procedure BuildRecordOffsetSize(const expr: string;var offset:tcgint;var size:tcgint; var mangledname: string; needvmtofs: boolean);
         procedure BuildConstSymbolExpression(allowref,betweenbracket,needofs:boolean;var value:tcgint;var asmsym:string;var asmsymtyp:TAsmsymtype);
         function BuildConstExpression(allowref,betweenbracket:boolean): tcgint;
         function Assemble: tlinkedlist;override;
         procedure handleopcode;virtual;abstract;
         function is_asmopcode(const s: string) : boolean;virtual;abstract;
         Function is_asmdirective(const s: string):boolean;
         function is_register(const s:string):boolean;virtual;
         function is_locallabel(const s: string):boolean;
         function is_targetdirective(const s: string): boolean;virtual;
         procedure GetToken;
         function consume(t : tasmtoken):boolean;
         procedure RecoverConsume(allowcomma:boolean);
         procedure handlepercent;virtual;
         procedure handledollar;virtual;
         procedure HandleTargetDirective;virtual;
       end;


  implementation

    uses
      { globals }
      verbose,systems,
      { input }
      scanner, pbase,
      { symtable }
      symbase,symtype,symsym,symdef,symtable,
{$ifdef x86}
      rax86,
{$endif x86}
      itcpugas,
      procinfo;


    procedure tattreader.SetupTables;
      var
        i : tasmop;
      Begin
        iasmops:=TFPHashList.create;
        for i:=firstop to lastop do
          iasmops.Add(upper(gas_op2str[i]),Pointer(PtrInt(i)));
      end;


    function tattreader.is_asmdirective(const s: string):boolean;
      var
        i : tasmtoken;
        hs : string;
      Begin
        { GNU as is also not casesensitive with this }
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


    function tattreader.is_register(const s:string):boolean;
      begin
        is_register:=false;
        actasmregister:=gas_regnum_search(lower(s));
        if actasmregister<>NR_NO then
          begin
            is_register:=true;
            actasmtoken:=AS_REGISTER;
          end;
      end;


    function tattreader.is_locallabel(const s: string):boolean;
      begin
        is_locallabel:=(length(s)>=2) and (s[1]='.') and (s[2]='L');
      end;


    procedure tattreader.handledollar;
      begin
        c:=current_scanner.asmgetchar;
        actasmtoken:=AS_DOLLAR;
      end;

    procedure tattreader.handlepercent;
      begin
        c:=current_scanner.asmgetchar;
        actasmtoken:=AS_MOD;
      end;

    function tattreader.is_targetdirective(const s: string): boolean;
      begin
        result:=false;
      end;

    procedure tattreader.handletargetdirective;
      begin
      end;

    procedure tattreader.GetToken;
      var
        len : longint;
        srsym : tsym;
        srsymtable : TSymtable;
      begin
        c:=scanner.c;
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
           { directive or local label }
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
              { this is a local label... }
              if (c=':') and is_locallabel(actasmpattern) then
               Begin
                 { local variables are case sensitive }
                 actasmtoken:=AS_LLABEL;
                 c:=current_scanner.asmgetchar;
                 firsttoken:=true;
                 exit;
               end
              { must be a directive }
              else
               Begin
                 { directives are case sensitive!! }
                 if is_asmdirective(actasmpattern) then
                  exit;
                 if is_targetdirective(actasmpattern) then
                   begin
                     actasmtoken:=AS_TARGET_DIRECTIVE;
                     exit;
                   end;
                 Message1(asmr_e_not_directive_or_local_symbol,actasmpattern);
               end;
            end;
           { only opcodes and global labels are allowed now. }
           while c in ['A'..'Z','a'..'z','0'..'9','_'] do
            begin
              inc(len);
              actasmpattern[len]:=c;
              c:=current_scanner.asmgetchar;
            end;
           actasmpattern[0]:=chr(len);
           { Label ? }
           if c = ':' then
            begin
              actasmtoken:=AS_LABEL;
              { let us point to the next character }
              c:=current_scanner.asmgetchar;
              firsttoken:=true;
              exit;
            end;
{$if defined(POWERPC) or defined(POWERPC64)}
           { some PowerPC instructions can have the postfix -, + or .
             this code could be moved to is_asmopcode but I think
             it's better to ifdef it here (FK)
           }
           case c of
             '.', '-', '+':
               begin
                 actasmpattern:=actasmpattern+c;
                 c:=current_scanner.asmgetchar;
               end
           end;
{$endif POWERPC}
{$if defined(ARM)}
           {
             Thumb-2 instructions can have a .W postfix to indicate 32bit instructions,
             Also in unified syntax sizes and types are indicated with something like a .<dt> prefix for example
           }
           case c of
             '.':
               begin
                 if len>1 then
                   begin
                     while c in ['A'..'Z','a'..'z','0'..'9','_','.'] do
                       begin
                         inc(len);
                         actasmpattern[len]:=c;
                         c:=current_scanner.asmgetchar;
                       end;
                     actasmpattern[0]:=chr(len);
                   end;
                 {actasmpattern:=actasmpattern+c;
                 c:=current_scanner.asmgetchar;

                 if upcase(c) = 'W' then
                   begin
                     actasmpattern:=actasmpattern+c;
                     c:=current_scanner.asmgetchar;
                   end
                 else
                   internalerror(2010122301);}
               end
           end;
{$endif ARM}
{$ifdef aarch64}
           { b.cond, ldX.arrangement }
           case c of
             '.':
               begin
                 repeat
                   actasmpattern:=actasmpattern+c;
                   c:=current_scanner.asmgetchar;
                 until not(c in ['a'..'z','A'..'Z','0'..'9']);
               end;
           end;
{$endif aarch64}
{$if defined(riscv32) or defined(riscv64)}
           {
             amo* instructions contain a postfix with size, and optionally memory ordering
             fence* can contain memory type identifier
             floating point instructions contain size, and optionally rounding mode
           }
           case c of
             '.':
               begin
                 repeat
                   actasmpattern:=actasmpattern+c;
                   c:=current_scanner.asmgetchar;
                 until not(c in ['a'..'z','A'..'Z','.']);
               end;
           end;
{$endif riscv}
{$ifdef xtensa}
           {
             Xtensa can have multiple postfixes
             MULA.DD.LL.LDDEC
             or postfixes with numbers
             RSR.CCOMPARE2
           }
           case c of
             '.':
               begin
                 repeat
                   actasmpattern:=actasmpattern+c;
                   c:=current_scanner.asmgetchar;
                 until not(c in ['a'..'z','A'..'Z', '0'..'9', '.']);
               end;
           end;
{$endif xtensa}
           { Opcode ? }
           If is_asmopcode(upper(actasmpattern)) then
            Begin
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
                 uppervar(actasmpattern);
{$ifdef x86}
                 { only x86 architectures have instruction prefixes }

                 { Opcode, can only be when the previous was a prefix }
                 If is_prefix(actopcode) and is_asmopcode(actasmpattern) then
                  Begin
                    uppervar(actasmpattern);
                    exit;
                  end;
{$endif x86}
                 { check for end which is a reserved word unlike the opcodes }
                 if actasmpattern = 'END' then
                  Begin
                    actasmtoken:=AS_END;
                    exit;
                  end;
                 if actasmpattern = 'TYPE' then
                  Begin
                    actasmtoken:=AS_TYPE;
                    exit;
                  end;
                 if actasmpattern = 'SIZEOF' then
                  Begin
                    actasmtoken:=AS_SIZEOF;
                    exit;
                  end;
                 if actasmpattern = 'VMTOFFSET' then
                  Begin
                    actasmtoken:=AS_VMTOFFSET;
                    exit;
                  end;
                 if is_register(actasmpattern) then
                   begin
                     actasmtoken:=AS_REGISTER;
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

             '%' : { register or modulo }
               handlepercent;

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

             '$' :
               begin
                 handledollar;
                 exit;
               end;

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
{$if defined(arm) or defined(aarch64)}
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
{$else arm or aarch64}
                 current_scanner.skipcomment(true);
                 GetToken;
{$endif arm}
                 exit;
               end;

{$if defined(arm) or defined(aarch64)}
             '}' :
               begin
                 actasmtoken:=AS_RSBRACKET;
                 c:=current_scanner.asmgetchar;
                 exit;
               end;
{$endif arm or aarch64}
{$ifdef arm}
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

             '@' :
               begin
                 actasmtoken:=AS_AT;
                 c:=current_scanner.asmgetchar;
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


    function tattreader.consume(t : tasmtoken):boolean;
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


    procedure tattreader.RecoverConsume(allowcomma:boolean);
      begin
        While not (actasmtoken in [AS_SEPARATOR,AS_END]) do
         begin
           if allowcomma and (actasmtoken=AS_COMMA) then
            break;
           Consume(actasmtoken);
         end;
      end;


    Procedure tattreader.BuildConstant(constsize: byte);
      var
       asmsymtyp : TAsmSymType;
       asmsym,
       expr: string;
       value : tcgint;
      Begin
        Repeat
          Case actasmtoken of
            AS_STRING:
              Begin
                expr:=actasmpattern;
                if length(expr) > 1 then
                 Message(asmr_e_string_not_allowed_as_const);
                Consume(AS_STRING);
                Case actasmtoken of
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
              Begin
                BuildConstSymbolExpression(false,false,false,value,asmsym,asmsymtyp);
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
       Until false;
      end;


    Procedure tattreader.BuildRealConstant(typ : tfloattype);
      var
        expr : string;
        r : bestreal;
        code : integer;
        negativ : boolean;
        errorflag: boolean;
      Begin
        errorflag:=FALSE;
        Repeat
          negativ:=false;
          expr:='';
          if actasmtoken=AS_PLUS then
            Consume(AS_PLUS)
          else
           if actasmtoken=AS_MINUS then
            begin
              negativ:=true;
              consume(AS_MINUS);
            end;
          Case actasmtoken of
            AS_INTNUM:
              Begin
                expr:=actasmpattern;
                Consume(AS_INTNUM);
                if negativ then
                 expr:='-'+expr;
                val(expr,r,code);
                if code<>0 then
                 Begin
                   r:=0;
                   Message(asmr_e_invalid_float_expr);
                 End;
                ConcatRealConstant(curlist,r,typ);
              end;
            AS_REALNUM:
              Begin
                expr:=actasmpattern;
                Consume(AS_REALNUM);
                { in ATT syntax you have 0d in front of the real }
                { should this be forced ?  yes i think so, as to }
                { conform to gas as much as possible.            }
                if (expr[1]='0') and (upper(expr[2])='D') then
                 Delete(expr,1,2);
                if negativ then
                 expr:='-'+expr;
                val(expr,r,code);
                if code<>0 then
                 Begin
                   r:=0;
                   Message(asmr_e_invalid_float_expr);
                 End;
                ConcatRealConstant(curlist,r,typ);
              end;
            AS_COMMA:
              begin
                Consume(AS_COMMA);
              end;
            AS_END,
            AS_SEPARATOR:
              begin
                break;
              end;
         else
           Begin
             Consume(actasmtoken);
             if not errorflag then
              Message(asmr_e_invalid_float_expr);
             errorflag:=TRUE;
           end;
         end;
       Until false;
      end;


    Procedure tattreader.BuildStringConstant(asciiz: boolean);
      var
        expr: string;
        errorflag : boolean;
      Begin
        errorflag:=FALSE;
        Repeat
          Case actasmtoken of
            AS_STRING:
              Begin
                expr:=actasmpattern;
                if asciiz then
                  expr:=expr+#0;
                ConcatString(curlist,expr);
                Consume(AS_STRING);
              end;
            AS_COMMA:
              begin
                Consume(AS_COMMA);
              end;
            AS_END,
            AS_SEPARATOR:
              begin
                break;
              end;
         else
           Begin
             Consume(actasmtoken);
             if not errorflag then
              Message(asmr_e_invalid_string_expression);
             errorflag:=TRUE;
           end;
         end;
       Until false;
      end;


   Function tattreader.Assemble: tlinkedlist;
     Var
       hl         : tasmlabel;
       commname,
       symname,
       symval     ,sectionname: string;
       lasTSec    : TAsmSectiontype;
       l1,
       l2,
       symofs     : tcgint;
       symtyp     : TAsmsymtype;
       section    : tai_section;
       secflags   : TSectionFlags;
       secprogbits : TSectionProgbits;
     Begin
       Message1(asmr_d_start_reading,'GNU AS');
       firsttoken:=TRUE;
       { sets up all opcode and register tables in uppercase }
       if not _asmsorted then
        Begin
          SetupTables;
          _asmsorted:=TRUE;
        end;
       curlist:=TAsmList.Create;

       { we might need to know which parameters are passed in registers }
       if not parse_generic then
         current_procinfo.generate_parameter_info;

       lasTSec:=sec_code;
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
                     ConcatPublic(curlist,actasmpattern);
                   ConcatLabel(curlist,hl);
                 end
               else
                Message1(asmr_e_unknown_label_identifier,actasmpattern);
               Consume(AS_LABEL);
             end;

           AS_DW:
             Begin
               Consume(AS_DW);
               BuildConstant(2);
             end;

           AS_DATA:
             Begin
               new_section(curList,sec_data,lower(current_procinfo.procdef.mangledname),0);
               lasTSec:=sec_data;
               Consume(AS_DATA);
             end;

           AS_TEXT:
             Begin
               new_section(curList,sec_code,lower(current_procinfo.procdef.mangledname),0);
               lasTSec:=sec_code;
               Consume(AS_TEXT);
             end;

           AS_INIT:
             Begin
               new_section(curList,sec_init,lower(current_procinfo.procdef.mangledname),0);
               lasTSec:=sec_init;
               Consume(AS_INIT);
             end;

           AS_FINI:
             Begin
               new_section(curList,sec_fini,lower(current_procinfo.procdef.mangledname),0);
               lasTSec:=sec_fini;
               Consume(AS_FINI);
             end;

           AS_DB:
             Begin
               Consume(AS_DB);
               BuildConstant(1);
             end;

           AS_DD:
             Begin
               Consume(AS_DD);
               BuildConstant(4);
             end;

           AS_DQ:
             Begin
               Consume(AS_DQ);
{$ifdef cpu64bitaddr}
               BuildConstant(8);
{$else cpu64bitaddr}
               BuildRealConstant(s64comp);
{$endif cpu64bitaddr}
             end;

           AS_SINGLE:
             Begin
               Consume(AS_SINGLE);
               BuildRealConstant(s32real);
             end;

           AS_DOUBLE:
             Begin
               Consume(AS_DOUBLE);
               BuildRealConstant(s64real);
             end;

           AS_EXTENDED:
             Begin
               Consume(AS_EXTENDED);
               BuildRealConstant(s80real);
             end;

           AS_CEXTENDED:
             Begin
               Consume(AS_CEXTENDED);
               BuildRealConstant(sc80real);
             end;

           AS_GLOBAL:
             Begin
               Consume(AS_GLOBAL);
               if actasmtoken=AS_ID then
                 ConcatPublic(curlist,actasmpattern);
               Consume(AS_ID);
               if actasmtoken<>AS_SEPARATOR then
                Consume(AS_SEPARATOR);
             end;

           AS_ALIGN:
             Begin
               Consume(AS_ALIGN);
               l1:=BuildConstExpression(false,false);
               if (target_info.system in [system_i386_GO32V2]) then
                 begin
                    l2:=1;
                    if (l1>=0) and (l1<=16) then
                      while (l1>0) do
                        begin
                          l2:=2*l2;
                          dec(l1);
                        end;
                    l1:=l2;
                 end;
               ConcatAlign(curlist,l1);
               Message(asmr_n_align_is_target_specific);
               if actasmtoken<>AS_SEPARATOR then
                Consume(AS_SEPARATOR);
             end;

           AS_BALIGN:
             Begin
               Consume(AS_BALIGN);
               ConcatAlign(curlist,BuildConstExpression(false,false));
               if actasmtoken<>AS_SEPARATOR then
                Consume(AS_SEPARATOR);
             end;

           AS_P2ALIGN:
             Begin
               Consume(AS_P2ALIGN);
               l1:=BuildConstExpression(false,false);
               l2:=1;
               if (l1>=0) and (l1<=16) then
                 while (l1>0) do
                   begin
                      l2:=2*l2;
                      dec(l1);
                   end;
               l1:=l2;
               ConcatAlign(curlist,l1);
               if actasmtoken<>AS_SEPARATOR then
                Consume(AS_SEPARATOR);
             end;

           AS_ASCIIZ:
             Begin
               Consume(AS_ASCIIZ);
               BuildStringConstant(TRUE);
             end;

           AS_ASCII:
             Begin
               Consume(AS_ASCII);
               BuildStringConstant(FALSE);
             end;

           AS_LCOMM:
             Begin
               Consume(AS_LCOMM);
               commname:=actasmpattern;
               Consume(AS_ID);
               Consume(AS_COMMA);
               symofs:=BuildConstExpression(false,false);
               curList.concat(Tai_datablock.Create(commname,symofs,carraydef.getreusable(u8inttype,symofs),AT_DATA));
               if actasmtoken<>AS_SEPARATOR then
                Consume(AS_SEPARATOR);
             end;

           AS_COMM:
             Begin
               Consume(AS_COMM);
               commname:=actasmpattern;
               Consume(AS_ID);
               Consume(AS_COMMA);
               symofs:=BuildConstExpression(false,false);
               curList.concat(Tai_datablock.Create_global(commname,symofs,carraydef.getreusable(u8inttype,symofs),AT_DATA));
               if actasmtoken<>AS_SEPARATOR then
                Consume(AS_SEPARATOR);
             end;

           AS_OPCODE:
             Begin
               HandleOpCode;
             end;

           AS_SEPARATOR:
             Begin
               Consume(AS_SEPARATOR);
             end;

           AS_RVA:
             begin
               { .rva generally applies to systems with COFF output format,
                 not just Windows. }
               if not (target_info.system in systems_all_windows) then
                 Message1(asmr_e_unsupported_directive,token2str[AS_RVA]);
               Consume(AS_RVA);
               BuildRva;
             end;

           AS_SET:
             begin
               Consume(AS_SET);
               BuildConstSymbolExpression(true,false,false, symofs,symname,symtyp);
               Consume(AS_COMMA);
               BuildConstSymbolExpression(true,false,false, symofs,symval,symtyp);

               curList.concat(tai_symbolpair.create(spk_set,symname,symval));
             end;

           AS_WEAK:
             begin
               Consume(AS_WEAK);
               BuildConstSymbolExpression(true,false,false, l1,symname,symtyp);
               { ideally, we should look up the symbol here (or later) and
                 depending on whether it's external or net, generate
                 asd_weak_reference or asd_weak_definition -- this is different
                 on some targets) }
               curList.concat(tai_directive.create(asd_weak_definition,symname));
             end;

           AS_SECTION:
             begin
               Consume(AS_SECTION);
               sectionname:=actasmpattern;
               secflags:=[];
               secprogbits:=SPB_None;
               Consume(AS_STRING);
               if actasmtoken=AS_COMMA then
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
                 end;

               //curList.concat(tai_section.create(sec_user, actasmpattern, 0));
               section:=new_section(curlist, sec_user, sectionname, 0);
               section.secflags:=secflags;
               section.secprogbits:=secprogbits;
             end;

           AS_TARGET_DIRECTIVE:
             HandleTargetDirective;

           AS_END:
             begin
               break; { end assembly block }
             end;

           else
             Begin
               Message(asmr_e_syntax_error);
               RecoverConsume(false);
             end;
         end;
       until false;
       { check that all referenced local labels are defined }
       checklocallabels;
       { are we back in the code section? }
       if lasTSec<>sec_code then
        begin
          Message(asmr_w_assembler_code_not_returned_to_text);
          new_section(curList,sec_code,lower(current_procinfo.procdef.mangledname),0);
        end;
       { Return the list in an asmnode }
       assemble:=curlist;
       Message1(asmr_d_finish_reading,'GNU AS');
     end;


{*****************************************************************************
                               Parsing Helpers
*****************************************************************************}

    Procedure tattreader.BuildRecordOffsetSize(const expr: string;var offset:tcgint;var size:tcgint; var mangledname: string; needvmtofs: boolean);
      { Description: This routine builds up a record offset after a AS_DOT }
      { token is encountered.                                              }
      { On entry actasmtoken should be equal to AS_DOT                     }
      var
        s : string;
        hastypecast: boolean;
      Begin
        offset:=0;
        size:=0;
        s:=expr;
        while (actasmtoken=AS_DOT) do
         begin
           Consume(AS_DOT);

           { a record field could have the same name as a register }
           if actasmtoken in [AS_ID,AS_REGISTER] then
             begin
               s:=s+'.'+actasmpattern;
               consume(actasmtoken)
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


    procedure tattreader.BuildConstSymbolExpression(allowref,betweenbracket,needofs:boolean;var value:tcgint;var asmsym:string;var asmsymtyp:TAsmsymtype);
      var
        hssymtyp : TAsmSymType;
        hs,tempstr,expr,mangledname : string;
        parenlevel : longint;
        l,k : tcgint;
        errorflag : boolean;
        prevtok : tasmtoken;
        sym : tsym;
        srsymtable : TSymtable;
        hl  : tasmlabel;
      Begin
        asmsym:='';
        asmsymtyp:=AT_DATA;
        value:=0;
        errorflag:=FALSE;
        tempstr:='';
        expr:='';
        parenlevel:=0;
        Repeat
          Case actasmtoken of
            AS_LPAREN:
              Begin
                { Exit if ref? }
                if allowref and (prevasmtoken in [AS_INTNUM,AS_ID]) then
                 break;
                Consume(AS_LPAREN);
                expr:=expr + '(';
                inc(parenlevel);
              end;
            AS_RBRACKET:
              begin
                if betweenbracket then
                  break;
                { write error only once. }
                if not errorflag then
                  Message(asmr_e_invalid_constant_expression);
                { consume tokens until we find COMMA or SEPARATOR }
                Consume(actasmtoken);
                errorflag:=TRUE;
              end;
            AS_RPAREN:
              Begin
                { end of ref ? }
                if (parenlevel=0) and betweenbracket then
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
                expr:=expr + '*';
              end;
            AS_PLUS:
              Begin
                Consume(AS_PLUS);
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
            AS_DOLLAR:
              begin
                Consume(AS_DOLLAR);
                if actasmtoken<>AS_ID then
                 Message(asmr_e_dollar_without_identifier);
              end;
            AS_STRING:
              Begin
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
            AS_SIZEOF,
            AS_TYPE:
              begin
                l:=0;
                Consume(actasmtoken);
                if actasmtoken<>AS_ID then
                 Message(asmr_e_type_without_identifier)
                else
                 begin
                   tempstr:=actasmpattern;
                   Consume(AS_ID);
                   if actasmtoken=AS_DOT then
                    begin
                      BuildRecordOffsetSize(tempstr,k,l,mangledname,false);
                      if mangledname<>'' then
                        Message(asmr_e_wrong_sym_type);
                    end
                   else
                    begin
                      searchsym(tempstr,sym,srsymtable);
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
              end;
            AS_VMTOFFSET:
              begin
                Consume(actasmtoken);
                if actasmtoken<>AS_ID then
                  Message(asmr_e_type_without_identifier)
                else
                  begin
                    tempstr:=actasmpattern;
                    consume(AS_ID);
                    BuildRecordOffsetSize(tempstr,k,l,mangledname,true);
                    if (mangledname <> '') then
                      Message(asmr_e_wrong_sym_type);
                    str(k,tempstr);
                    expr := expr + tempstr;
                  end
              end;
            AS_ID:
              Begin
                hs:='';
                hssymtyp:=AT_DATA;
                tempstr:=actasmpattern;
                prevtok:=prevasmtoken;
                consume(AS_ID);
                if SearchIConstant(tempstr,l) then
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
                      hssymtyp:=AT_LABEL;
                    end
                   else
                    if SearchLabel(tempstr,hl,false) then
                      begin
                        hs:=hl.name;
                        hssymtyp:=AT_FUNCTION;
                      end
                   else
                    begin
                      searchsym(tempstr,sym,srsymtable);
                      if assigned(sym) then
                       begin
                         case sym.typ of
                           staticvarsym :
                             begin
                               { we always assume in asm statements that     }
                               { that the variable is valid.                 }
                               tabstractvarsym(sym).varstate:=vs_readwritten;
                               inc(tabstractvarsym(sym).refs);
                               { variable can't be placed in a register }
                               tabstractvarsym(sym).varregable:=vr_none;
                               { and anything may happen with its address }
                               tabstractvarsym(sym).addr_taken:=true;

                               hs:=tstaticvarsym(sym).mangledname;
                             end;
                           localvarsym,
                           paravarsym :
                             Message(asmr_e_no_local_or_para_allowed);
                           procsym :
                             begin
                               if Tprocsym(sym).ProcdefList.Count>1 then
                                Message(asmr_w_calling_overload_func);
                               hs:=tprocdef(tprocsym(sym).ProcdefList[0]).mangledname;
                               hssymtyp:=AT_FUNCTION;
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
                      if needofs and (prevtok<>AS_DOLLAR) then
                       Message(asmr_e_need_dollar);
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
                      else
                       Message(asmr_e_only_add_relocatable_symbol);
                    end;
                   if actasmtoken=AS_DOT then
                    begin
                      BuildRecordOffsetSize(tempstr,l,k,hs,false);
                      if (hs<>'') then
                        hssymtyp:=AT_FUNCTION
                      else
                        begin
                          str(l, tempstr);
                          expr:=expr + tempstr;
                        end
                    end
                   else
                    begin
                      if (expr='') or (expr[length(expr)] in ['+','-','/','*']) then
                       delete(expr,length(expr),1);
                    end;
                 end;
                { check if there are wrong operator used like / or mod etc. }
                if (hs<>'') and
                   not(actasmtoken in [AS_MINUS,AS_PLUS,AS_COMMA,AS_SEPARATOR,
                                       AS_LPAREN,AS_RPAREN,AS_RBRACKET,AS_END]) then
                 Message(asmr_e_only_add_relocatable_symbol);
              end;
            AS_END,
            AS_SEPARATOR,
            AS_COMMA:
              break;
          else
            Begin
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
      end;


    function tattreader.BuildConstExpression(allowref,betweenbracket:boolean): tcgint;
      var
        l : tcgint;
        hs : string;
        hssymtyp : TAsmSymType;
      begin
        BuildConstSymbolExpression(allowref,betweenbracket,false,l,hs,hssymtyp);
        if hs<>'' then
         Message(asmr_e_relocatable_symbol_not_allowed);
        BuildConstExpression:=l;
      end;


    Procedure tattreader.BuildConstantOperand(oper : toperand);
      var
        l : tcgint;
        tempstr : string;
        tempsymtyp : TAsmSymType;
      begin
        BuildConstSymbolExpression(false,false,true,l,tempstr,tempsymtyp);
        if tempstr<>'' then
         begin
           oper.opr.typ:=OPR_SYMBOL;
           oper.opr.symofs:=l;
           oper.opr.symbol:=current_asmdata.RefAsmSymbol(tempstr,tempsymtyp);
         end
        else
         begin
           oper.opr.typ:=OPR_CONSTANT;
           { cast properly to avoid a range check error }
{$if defined(AVR) or defined(i8086)}
           oper.opr.val:=longint(l);
{$else}
           oper.opr.val:=aint(l);
{$endif}
         end;
      end;

    procedure tattreader.BuildRva;
      var
       asmsymtyp : TAsmSymType;
       asmsym: string;
       value : tcgint;
       ai:tai_const;
      begin
        repeat
          case actasmtoken of
            AS_INTNUM,
            AS_PLUS,
            AS_MINUS,
            AS_LPAREN,
            AS_ID :
              Begin
                BuildConstSymbolExpression(false,false,false,value,asmsym,asmsymtyp);
                if asmsym<>'' then
                 begin
                   ai:=tai_const.create_type_sym(aitconst_rva_symbol,current_asmdata.RefAsmSymbol(asmsym,asmsymtyp));
                   ai.value:=value;
                   curlist.concat(ai);
                 end
                else
                 Message(asmr_e_invalid_symbol_ref);
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

end.
