{
    Copyright (c) 1998-2002 by Carl Eric Codere and Peter Vreman

    This unit implements some support routines for assembler parsing
    independent of the processor

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

 **********************************************************************}
Unit RAUtils;

{$i fpcdefs.inc}

Interface

Uses
  cutils,cclasses,
  globtype,aasmbase,aasmtai,aasmdata,cpubase,cpuinfo,cgbase,cgutils,
  symconst,symbase,symtype,symdef,symsym,symcpu;

Const
  RPNMax = 10;             { I think you only need 4, but just to be safe }
  OpMax  = 25;

Function SearchLabel(const s: string; var hl: tasmlabel;emit:boolean): boolean;


{---------------------------------------------------------------------
                 Instruction management
---------------------------------------------------------------------}

type
  TOprType=(OPR_NONE,OPR_CONSTANT,OPR_SYMBOL,OPR_LOCAL,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            OPR_REFERENCE,OPR_REGISTER,OPR_COND,OPR_REGSET,OPR_SHIFTEROP,OPR_MODEFLAGS,OPR_SPECIALREG);
=======
            OPR_REFERENCE,OPR_REGISTER,OPR_REGLIST,OPR_COND,OPR_REGSET,OPR_SHIFTEROP,OPR_MODEFLAGS);
>>>>>>> graemeg/cpstrnew
=======
            OPR_REFERENCE,OPR_REGISTER,OPR_REGLIST,OPR_COND,OPR_REGSET,OPR_SHIFTEROP,OPR_MODEFLAGS);
>>>>>>> graemeg/cpstrnew
=======
            OPR_REFERENCE,OPR_REGISTER,OPR_REGLIST,OPR_COND,OPR_REGSET,OPR_SHIFTEROP,OPR_MODEFLAGS);
>>>>>>> graemeg/cpstrnew
=======
            OPR_REFERENCE,OPR_REGISTER,OPR_REGLIST,OPR_COND,OPR_REGSET,OPR_SHIFTEROP,OPR_MODEFLAGS);
>>>>>>> origin/cpstrnew

  TOprRec = record
    case typ:TOprType of
      OPR_NONE      : ();
      OPR_CONSTANT  : (val:aint);
      OPR_SYMBOL    : (symbol:tasmsymbol;symofs:aint;symseg:boolean;sym_farproc_entry:boolean);
      OPR_REFERENCE : (varsize:asizeint; constoffset: asizeint;ref_farproc_entry:boolean;ref:treference);
      OPR_LOCAL     : (localvarsize, localconstoffset: asizeint;localsym:tabstractnormalvarsym;localsymofs:aint;localindexreg:tregister;localscale:byte;localgetoffset,localforceref:boolean);
      OPR_REGISTER  : (reg:tregister);
{$ifdef m68k}
      OPR_REGSET   : (regsetdata,regsetaddr,regsetfpu : tcpuregisterset);
{$endif m68k}
{$ifdef powerpc}
      OPR_COND      : (cond : tasmcond);
{$endif powerpc}
{$ifdef POWERPC64}
      OPR_COND      : (cond : tasmcond);
{$endif POWERPC64}
{$ifdef arm}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      OPR_REGSET    : (regset : tcpuregisterset; regtype: tregistertype; subreg: tsubregister; usermode: boolean);
      OPR_SHIFTEROP : (shifterop : tshifterop);
      OPR_COND      : (cc : tasmcond);
      OPR_MODEFLAGS : (flags : tcpumodeflags);
      OPR_SPECIALREG: (specialreg : tregister; specialregflags : tspecialregflags);
=======
      OPR_REGSET    : (regset : tcpuregisterset; regtype: tregistertype; subreg: tsubregister);
      OPR_SHIFTEROP : (shifterop : tshifterop);
      OPR_COND      : (cc : tasmcond);
      OPR_MODEFLAGS : (flags : tcpumodeflags);
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      OPR_REGSET    : (regset : tcpuregisterset; regtype: tregistertype; subreg: tsubregister);
      OPR_SHIFTEROP : (shifterop : tshifterop);
      OPR_COND      : (cc : tasmcond);
      OPR_MODEFLAGS : (flags : tcpumodeflags);
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endif arm}
{$ifdef aarch64}
      OPR_SHIFTEROP : (shifterop : tshifterop);
      OPR_COND      : (cc : tasmcond);
{$endif aarch64}
  end;

  TOperand = class
    opr    : TOprRec;
    typesize : byte;
    hastype,          { if the operand has typecasted variable }
    hasvar : boolean; { if the operand is loaded with a variable }
    size   : TCGSize;
    constructor create;virtual;
    destructor  destroy;override;
    Procedure SetSize(_size:longint;force:boolean);virtual;
    Procedure SetCorrectSize(opcode:tasmop);virtual;
    Function  SetupResult:boolean;virtual;
    Function  SetupSelf:boolean;
    Function  SetupOldEBP:boolean;
    Function  SetupVar(const s:string;GetOffset : boolean): Boolean;
    Function  CheckOperand: boolean; virtual;
    Procedure InitRef;
  end;
  TCOperand = class of TOperand;

  TInstruction = class
    operands  : array[1..max_operands] of toperand;
    opcode    : tasmop;
    condition : tasmcond;
    ops       : byte;
    labeled   : boolean;
    filepos  : tfileposinfo;
    constructor create(optype : tcoperand);virtual;
    destructor  destroy;override;
    { converts the instruction to an instruction how it's used by the assembler writer
<<<<<<< HEAD
<<<<<<< HEAD
      and concats it to the passed list. The newly created item is returned if the
=======
      and concats it to the passed list. The newly created item is returned if the 
>>>>>>> graemeg/fixes_2_2
=======
      and concats it to the passed list. The newly created item is returned if the 
>>>>>>> origin/fixes_2_2
      instruction was valid, otherwise nil is returned }
    function ConcatInstruction(p:TAsmList) : tai;virtual;
  end;

  {---------------------------------------------------------------------}
  {                   Expression parser types                           }
  {---------------------------------------------------------------------}

   TExprOperator = record
    ch: char;           { operator }
    is_prefix: boolean; { was it a prefix, possible prefixes are +,- and not }
   end;

  {**********************************************************************}
  { The following operators are supported:                              }
  {  '+' : addition                                                     }
  {  '-' : subtraction                                                  }
  {  '*' : multiplication                                               }
  {  '/' : modulo division                                              }
  {  '^' : exclusive or                                                 }
  {  '<' : shift left                                                   }
  {  '>' : shift right                                                  }
  {  '&' : bitwise and                                                  }
  {  '|' : bitwise or                                                   }
  {  '~' : bitwise complement                                           }
  {  '%' : modulo division                                              }
  {  nnn: longint numbers                                               }
  {  ( and ) parenthesis                                                }
  {**********************************************************************}

  TExprParse = class
    public
     Constructor create;
     Destructor Destroy;override;
     Function Evaluate(Expr:  String): aint;
     Function Priority(_Operator: Char): aint;
    private
     RPNStack   : Array[1..RPNMax] of aint;        { Stack For RPN calculator }
     RPNTop     : aint;
     OpStack    : Array[1..OpMax] of TExprOperator;    { Operator stack For conversion }
     OpTop      : aint;
     Procedure RPNPush(Num: aint);
     Function RPNPop: aint;
     Procedure RPNCalc(const token: String; prefix: boolean);
     Procedure OpPush(_Operator: char; prefix: boolean);
     { In reality returns TExprOperaotr }
     Procedure OpPop(var _Operator:TExprOperator);
  end;

  { Evaluate an expression string to a aint }
  Function CalculateExpression(const expression: string): aint;

  {---------------------------------------------------------------------}
  {                     String routines                                 }
  {---------------------------------------------------------------------}

Function ParseVal(const S:String;base:byte):aint;
Function PadZero(Var s: String; n: byte): Boolean;
Function EscapeToPascal(const s:string): string;

{---------------------------------------------------------------------
                     Symbol helper routines
---------------------------------------------------------------------}

procedure AsmSearchSym(const s:string;var srsym:tsym;var srsymtable:TSymtable);
Function GetRecordOffsetSize(s:string;Var Offset: aint;var Size:aint; var mangledname: string; needvmtofs: boolean):boolean;
Function SearchType(const hs:string;var size:aint): Boolean;
Function SearchRecordType(const s:string): boolean;
Function SearchIConstant(const s:string; var l:aint): boolean;


{---------------------------------------------------------------------
                  Instruction generation routines
---------------------------------------------------------------------}

  Procedure ConcatLabel(p: TAsmList;var l : tasmlabel);
  Procedure ConcatConstant(p : TAsmList;value: aint; constsize:byte);
  Procedure ConcatConstSymbol(p : TAsmList;const sym:string;symtyp:tasmsymtype;l:aint);
  Procedure ConcatRealConstant(p : TAsmList;value: bestreal; real_typ : tfloattype);
  Procedure ConcatString(p : TAsmList;s:string);
  procedure ConcatAlign(p:TAsmList;l:aint);
  Procedure ConcatPublic(p:TAsmList;const s : string);
  Procedure ConcatLocal(p:TAsmList;const s : string);


Implementation

uses
  SysUtils,
  defutil,systems,verbose,globals,
  symtable,paramgr,
  aasmcpu,
  procinfo;

{*************************************************************************
                              TExprParse
*************************************************************************}

Constructor TExprParse.create;
Begin
end;


Procedure TExprParse.RPNPush(Num : aint);
{ Add an operand to the top of the RPN stack }
begin
  if RPNTop < RPNMax then
   begin
     Inc(RPNTop);
     RPNStack[RPNTop]:=Num;
   end
  else
   Message(asmr_e_expr_illegal);
end;


Function TExprParse.RPNPop : aint;
{ Get the operand at the top of the RPN stack }
begin
  RPNPop:=0;
  if RPNTop > 0 then
   begin
     RPNPop:=RPNStack[RPNTop];
     Dec(RPNTop);
   end
  else
   Message(asmr_e_expr_illegal);
end;


Procedure TExprParse.RPNCalc(const Token : String; prefix:boolean);                       { RPN Calculator }
Var
  Temp  : aint;
  n1,n2 : aint;
  LocalError : Integer;
begin
  { Handle operators }
  if (Length(Token) = 1) and (Token[1] in ['+', '-', '*', '/','&','|','%','^','~','<','>']) then
   Case Token[1] of
    '+' :
      Begin
        if not prefix then
         RPNPush(RPNPop + RPNPop);
      end;
    '-' :
      Begin
        if prefix then
         RPNPush(-(RPNPop))
        else
         begin
           n1:=RPNPop;
           n2:=RPNPop;
           RPNPush(n2 - n1);
         end;
      end;
    '*' : RPNPush(RPNPop * RPNPop);
    '&' :
      begin
        n1:=RPNPop;
        n2:=RPNPop;
        RPNPush(n2 and n1);
      end;
    '|' :
      begin
        n1:=RPNPop;
        n2:=RPNPop;
        RPNPush(n2 or n1);
      end;
    '~' : RPNPush(NOT RPNPop);
    '<' :
      begin
        n1:=RPNPop;
        n2:=RPNPop;
        RPNPush(n2 SHL n1);
      end;
    '>' :
      begin
        n1:=RPNPop;
        n2:=RPNPop;
        RPNPush(n2 SHR n1);
      end;
    '%' :
      begin
        Temp:=RPNPop;
        if Temp <> 0 then
         RPNPush(RPNPop mod Temp)
        else
         begin
           Message(asmr_e_expr_zero_divide);
           { push 1 for error recovery }
           RPNPush(1);
         end;
      end;
    '^' : RPNPush(RPNPop XOR RPNPop);
    '/' :
      begin
        Temp:=RPNPop;
        if Temp <> 0 then
         RPNPush(RPNPop div Temp)
        else
         begin
           Message(asmr_e_expr_zero_divide);
           { push 1 for error recovery }
           RPNPush(1);
         end;
      end;
   end
  else
   begin
     { Convert String to number and add to stack }
      Val(Token, Temp, LocalError);
     if LocalError = 0 then
      RPNPush(Temp)
     else
      begin
        Message(asmr_e_expr_illegal);
        { push 1 for error recovery }
        RPNPush(1);
      end;
   end;
end;


Procedure TExprParse.OpPush(_Operator : char;prefix: boolean);
{ Add an operator onto top of the stack }
begin
  if OpTop < OpMax then
   begin
     Inc(OpTop);
     OpStack[OpTop].ch:=_Operator;
     OpStack[OpTop].is_prefix:=prefix;
   end
  else
   Message(asmr_e_expr_illegal);
end;


Procedure TExprParse.OpPop(var _Operator:TExprOperator);
{ Get operator at the top of the stack }
begin
  if OpTop > 0 then
   begin
     _Operator:=OpStack[OpTop];
     Dec(OpTop);
   end
  else
   Message(asmr_e_expr_illegal);
end;


Function TExprParse.Priority(_Operator : Char) : aint;
{ Return priority of operator }
{ The greater the priority, the higher the precedence }
begin
  Priority:=0;
  Case _Operator OF
    '(' :
      Priority:=0;
    '|','^','~' :             // the lowest priority: OR, XOR, NOT
      Priority:=0;
    '&' :                     // bigger priority: AND
      Priority:=1;
    '+', '-' :                // bigger priority: +, -
      Priority:=2;
    '*', '/','%','<','>' :   // the highest priority: *, /, MOD, SHL, SHR
      Priority:=3;
    else
      Message(asmr_e_expr_illegal);
  end;
end;


Function TExprParse.Evaluate(Expr : String):aint;
Var
  I     : longint;
  Token : String;
  opr   : TExprOperator;
begin
  Evaluate:=0;
  { Reset stacks }
  OpTop :=0;
  RPNTop:=0;
  Token :='';
  { nothing to do ? }
  if Expr='' then
   exit;
  For I:=1 to Length(Expr) DO
   begin
     if Expr[I] in ['0'..'9'] then
      begin       { Build multi-digit numbers }
        Token:=Token + Expr[I];
        if I = Length(Expr) then          { Send last one to calculator }
         RPNCalc(Token,false);
      end
     else
      if Expr[I] in ['+', '-', '*', '/', '(', ')','^','&','|','%','~','<','>'] then
       begin
         if Token <> '' then
          begin        { Send last built number to calc. }
            RPNCalc(Token,false);
            Token:='';
          end;

         Case Expr[I] OF
          '(' : OpPush('(',false);
          ')' : begin
                  While (OpTop>0) and (OpStack[OpTop].ch <> '(') DO
                   Begin
                     OpPop(opr);
                     RPNCalc(opr.ch,opr.is_prefix);
                   end;
                  OpPop(opr);                          { Pop off and ignore the '(' }
                end;
  '+','-','~' : Begin
                  { workaround for -2147483648 }
                  if (expr[I]='-') and (expr[i+1] in ['0'..'9']) then
                   begin
                     token:='-';
                     expr[i]:='+';
                   end;
                  { if start of expression then surely a prefix }
                  { or if previous char was also an operator    }
                  if (I = 1) or (not (Expr[I-1] in ['0'..'9',')'])) then
                    OpPush(Expr[I],true)
                  else
                    Begin
                    { Evaluate all higher priority operators }
                      While (OpTop > 0) AND (Priority(Expr[I]) <= Priority(OpStack[OpTop].ch)) DO
                       Begin
                         OpPop(opr);
                         RPNCalc(opr.ch,opr.is_prefix);
                       end;
                      OpPush(Expr[I],false);
                    End;
                end;
     '*', '/',
  '^','|','&',
  '%','<','>' : begin
                  While (OpTop > 0) and (Priority(Expr[I]) <= Priority(OpStack[OpTop].ch)) DO
                   Begin
                     OpPop(opr);
                     RPNCalc(opr.ch,opr.is_prefix);
                   end;
                  OpPush(Expr[I],false);
                end;
         end; { Case }
       end
     else
      Message(asmr_e_expr_illegal);  { Handle bad input error }
   end;

{ Pop off the remaining operators }
  While OpTop > 0 do
   Begin
     OpPop(opr);
     RPNCalc(opr.ch,opr.is_prefix);
   end;

{ The result is stored on the top of the stack }
  Evaluate:=RPNPop;
end;


Destructor TExprParse.Destroy;
Begin
end;


Function CalculateExpression(const expression: string): aint;
var
  expr: TExprParse;
Begin
  expr:=TExprParse.create;
  CalculateExpression:=expr.Evaluate(expression);
  expr.Free;
end;


{*************************************************************************}
{                         String conversions/utils                        }
{*************************************************************************}

Function EscapeToPascal(const s:string): string;
{ converts a C styled string - which contains escape }
{ characters to a pascal style string.               }
var
  i,len : asizeint;
  hs    : string;
  temp  : string;
  c     : char;
Begin
  hs:='';
  len:=0;
  i:=0;
  while (i<length(s)) and (len<255) do
   begin
     Inc(i);
     if (s[i]='\') and (i<length(s)) then
      Begin
        inc(i);
        case s[i] of
         '\' :
           c:='\';
         'b':
           c:=#8;
         'f':
           c:=#12;
         'n':
           c:=#10;
         'r':
           c:=#13;
         't':
           c:=#9;
         '"':
           c:='"';
         '0'..'7':
           Begin
             temp:=s[i];
             temp:=temp+s[i+1];
             temp:=temp+s[i+2];
             inc(i,2);
             c:=chr(ParseVal(temp,8));
           end;
         'x':
           Begin
             temp:=s[i+1];
             temp:=temp+s[i+2];
             inc(i,2);
             c:=chr(ParseVal(temp,16));
           end;
         else
           Begin
             Message1(asmr_e_escape_seq_ignored,s[i]);
             c:=s[i];
           end;
        end;
      end
     else
      c:=s[i];
     inc(len);
     hs[len]:=c;
   end;
  hs[0]:=chr(len);
  EscapeToPascal:=hs;
end;


Function ParseVal(const S:String;base:byte):aint;
{ Converts a decimal string to aint }
var
  code : integer;
  errmsg : word;
  prefix : string[2];
Begin
  case base of
    2 :
      begin
        errmsg:=asmr_e_error_converting_binary;
        prefix:='%';
      end;
    8 :
      begin
        errmsg:=asmr_e_error_converting_octal;
        prefix:='&';
      end;
    10 :
      begin
        errmsg:=asmr_e_error_converting_decimal;
        prefix:='';
      end;
    16 :
      begin
        errmsg:=asmr_e_error_converting_hexadecimal;
        prefix:='$';
      end;
    else
      internalerror(200501202);
  end;
  val(prefix+s,result,code);
  if code<>0 then
    begin
      val(prefix+s,aword(result),code);
      if code<>0 then
        begin
          Message1(errmsg,s);
          result:=0;
        end;
    end;
end;


Function PadZero(Var s: String; n: byte): Boolean;
Begin
  PadZero:=TRUE;
  { Do some error checking first }
  if Length(s) = n then
    exit
  else
  if Length(s) > n then
  Begin
    PadZero:=FALSE;
    delete(s,n+1,length(s));
    exit;
  end
  else
    PadZero:=TRUE;
  { Fill it up with the specified character }
  fillchar(s[length(s)+1],n-1,#0);
  s[0]:=chr(n);
end;


{****************************************************************************
                                   TOperand
****************************************************************************}

constructor TOperand.Create;
begin
  size:=OS_NO;
  hastype:=false;
  hasvar:=false;
  FillChar(Opr,sizeof(Opr),0);
end;


destructor TOperand.destroy;
begin
end;


Procedure TOperand.SetSize(_size:longint;force:boolean);
begin
  if force or
     ((size = OS_NO) and (_size<=16)) then
   Begin
     case _size of
        1 : size:=OS_8;
        2 : size:=OS_16{ could be S_IS};
        4 : size:=OS_32{ could be S_IL or S_FS};
        8 : size:=OS_64{ could be S_D or S_FL};
       10 : size:=OS_F80;
       16 : size:=OS_128;
     end;
   end;
end;


Procedure TOperand.SetCorrectSize(opcode:tasmop);
begin
end;


function TOperand.SetupResult:boolean;

begin
  SetupResult:=false;
  { replace by correct offset. }
  with current_procinfo.procdef do
    if (not is_void(returndef)) then
      begin
        if (m_tp7 in current_settings.modeswitches) and
          not (df_generic in defoptions) and
          (not paramanager.ret_in_param(returndef,current_procinfo.procdef)) then
          begin
            message(asmr_e_cannot_use_RESULT_here);
            exit;
          end;
        SetupResult:=setupvar('result',false)
      end
    else
      message(asmr_e_void_function);
end;


Function TOperand.SetupSelf:boolean;
Begin
  SetupSelf:=false;
  if assigned(current_structdef) then
    SetupSelf:=setupvar('self',false)
  else
    Message(asmr_e_cannot_use_SELF_outside_a_method);
end;


Function TOperand.SetupOldEBP:boolean;
Begin
  SetupOldEBP:=false;
  if current_procinfo.procdef.parast.symtablelevel>normal_function_level then
    SetupOldEBP:=setupvar('parentframe',false)
  else
    Message(asmr_e_cannot_use_OLDEBP_outside_nested_procedure);
end;


Function TOperand.SetupVar(const s:string;GetOffset : boolean): Boolean;

  function symtable_has_localvarsyms(st:TSymtable):boolean;
  var
    sym : tsym;
    i   : longint;
  begin
    result:=false;
    for i:=0 to st.SymList.Count-1 do
      begin
        sym:=tsym(st.SymList[i]);
        if sym.typ=localvarsym then
          begin
            result:=true;
            exit;
          end;
      end;
  end;

  procedure setconst(l:aint);
  begin
    { We return the address of the field, just like Delphi/TP }
    case opr.typ of
      OPR_NONE :
        begin
          opr.typ:=OPR_CONSTANT;
          opr.val:=l;
        end;
      OPR_CONSTANT :
        inc(opr.val,l);
      OPR_REFERENCE :
        inc(opr.ref.offset,l);
      OPR_LOCAL :
        inc(opr.localsymofs,l);
      else
        Message(asmr_e_invalid_operand_type);
    end;
  end;


  procedure setvarsize(sym: tabstractvarsym);
  var
    harrdef: tarraydef;
    l: asizeint;
  begin
    case sym.vardef.typ of
      orddef,
      enumdef,
      pointerdef,
      procvardef,
      floatdef :
        SetSize(sym.getsize,false);
      arraydef :
        begin
          { for arrays try to get the element size, take care of
            multiple indexes }
          harrdef:=tarraydef(sym.vardef);

          { calc array size }
          if is_special_array(harrdef) then
             l := -1
           else
             l := harrdef.size;

          case opr.typ of
            OPR_REFERENCE: opr.varsize := l;
                OPR_LOCAL: opr.localvarsize := l;
          end;


          while assigned(harrdef.elementdef) and
                (harrdef.elementdef.typ=arraydef) do
           harrdef:=tarraydef(harrdef.elementdef);
          if not is_packed_array(harrdef) then
            SetSize(harrdef.elesize,false)
           else
               if (harrdef.elepackedbitsize mod 8) = 0 then
                 SetSize(harrdef.elepackedbitsize div 8,false);
        end;
      recorddef:
        case opr.typ of
          OPR_REFERENCE: opr.varsize := sym.getsize;
              OPR_LOCAL: opr.localvarsize := sym.getsize;
        end;
    end;
  end;

{ search and sets up the correct fields in the Instr record }
{ for the NON-constant identifier passed to the routine.    }
{ if not found returns FALSE.                               }
var
  sym : tsym;
  srsymtable : TSymtable;
  indexreg : tregister;
  plist : ppropaccesslistitem;
Begin
  SetupVar:=false;
  asmsearchsym(s,sym,srsymtable);
  if sym = nil then
   exit;
  if sym.typ=absolutevarsym then
    begin
      if (tabsolutevarsym(sym).abstyp=tovar) then
        begin
          { Only support simple loads }
          plist:=tabsolutevarsym(sym).ref.firstsym;
          if assigned(plist) and
             (plist^.sltype=sl_load) then
            sym:=plist^.sym
          else
            begin
              Message(asmr_e_unsupported_symbol_type);
              exit;
            end;
        end
      else
        begin
          Message(asmr_e_unsupported_symbol_type);
          exit;
        end;
    end;
  case sym.typ of
    fieldvarsym :
      begin
        if not tabstractrecordsymtable(sym.owner).is_packed then
          setconst(tfieldvarsym(sym).fieldoffset)
        else if tfieldvarsym(sym).fieldoffset mod 8 = 0 then
          setconst(tfieldvarsym(sym).fieldoffset div 8)
        else
          Message(asmr_e_packed_element);
        setvarsize(tabstractvarsym(sym));
        hasvar:=true;
        SetupVar:=true;
      end;
    staticvarsym,
    localvarsym,
    paravarsym :
      begin
        { we always assume in asm statements that     }
        { that the variable is valid.                 }
        tabstractvarsym(sym).varstate:=vs_readwritten;
        inc(tabstractvarsym(sym).refs);
        { variable can't be placed in a register }
        tabstractvarsym(sym).varregable:=vr_none;
        { and anything may happen with its address }
        tabstractvarsym(sym).addr_taken:=true;
        case sym.typ of
          staticvarsym :
            begin
              initref;
              opr.ref.symbol:=current_asmdata.RefAsmSymbol(tstaticvarsym(sym).mangledname);
            end;
          paravarsym,
          localvarsym :
            begin
              if opr.typ=OPR_REFERENCE then
                begin
                  indexreg:=opr.ref.base;
                  if opr.ref.index<>NR_NO then
                    begin
                      if indexreg=NR_NO then
                        indexreg:=opr.ref.index
                      else
                        Message(asmr_e_multiple_index);
                    end;
                end
              else
                indexreg:=NR_NO;
              opr.typ:=OPR_LOCAL;
              if assigned(current_procinfo.parent) and
                 not(po_inline in current_procinfo.procdef.procoptions) and
                 (sym.owner<>current_procinfo.procdef.localst) and
                 (sym.owner<>current_procinfo.procdef.parast) and
                 (current_procinfo.procdef.localst.symtablelevel>normal_function_level) and
                 symtable_has_localvarsyms(current_procinfo.procdef.localst) then
                message1(asmr_e_local_para_unreachable,s);
              opr.localsym:=tabstractnormalvarsym(sym);
              opr.localsymofs:=0;
              opr.localindexreg:=indexreg;
              opr.localscale:=0;
              opr.localgetoffset:=GetOffset;
              if paramanager.push_addr_param(tabstractvarsym(sym).varspez,tabstractvarsym(sym).vardef,current_procinfo.procdef.proccalloption) then
<<<<<<< HEAD
                SetSize(sizeof(pint),false);
=======
                SetSize(sizeof(aint),false);
            end;
        end;
        case tabstractvarsym(sym).vardef.typ of
          orddef,
          enumdef,
          pointerdef,
          floatdef :
            SetSize(tabstractvarsym(sym).getsize,false);
          arraydef :
            begin
              { for arrays try to get the element size, take care of
                multiple indexes }
              harrdef:=tarraydef(tabstractvarsym(sym).vardef);
              while assigned(harrdef.elementdef) and
                    (harrdef.elementdef.typ=arraydef) do
               harrdef:=tarraydef(harrdef.elementdef);
              if not is_packed_array(harrdef) then
                SetSize(harrdef.elesize,false)
               else
                 begin
                   if (harrdef.elepackedbitsize mod 8) = 0 then
                     SetSize(harrdef.elepackedbitsize div 8,false);
                 end;
>>>>>>> graemeg/fixes_2_2
            end;
        end;
        setvarsize(tabstractvarsym(sym));
        hasvar:=true;
        SetupVar:=true;
        Exit;
      end;
    constsym :
      begin
        if tconstsym(sym).consttyp=constord then
         begin
           setconst(tconstsym(sym).value.valueord.svalue);
           SetupVar:=true;
           Exit;
         end;
      end;
    typesym :
      begin
        if ttypesym(sym).typedef.typ in [recorddef,objectdef] then
         begin
           setconst(0);
           SetupVar:=TRUE;
           Exit;
         end;
      end;
    procsym :
      begin
        if Tprocsym(sym).ProcdefList.Count>1 then
          Message(asmr_w_calling_overload_func);
        case opr.typ of
          OPR_REFERENCE:
            begin
              opr.ref.symbol:=current_asmdata.RefAsmSymbol(tprocdef(tprocsym(sym).ProcdefList[0]).mangledname);
{$ifdef i8086}
              opr.ref_farproc_entry:=is_proc_far(tprocdef(tprocsym(sym).ProcdefList[0]))
                        and not (po_interrupt in tprocdef(tprocsym(sym).ProcdefList[0]).procoptions);
{$endif i8086}
            end;
          OPR_NONE:
            begin
              opr.typ:=OPR_SYMBOL;
              opr.symbol:=current_asmdata.RefAsmSymbol(tprocdef(tprocsym(sym).ProcdefList[0]).mangledname);
{$ifdef i8086}
              opr.sym_farproc_entry:=is_proc_far(tprocdef(tprocsym(sym).ProcdefList[0]))
                        and not (po_interrupt in tprocdef(tprocsym(sym).ProcdefList[0]).procoptions);
{$endif i8086}
              opr.symofs:=0;
            end;
        else
          Message(asmr_e_invalid_operand_type);
        end;
        hasvar:=true;
        SetupVar:=TRUE;
        Exit;
      end;
{$ifdef i8086}
    labelsym :
      begin
        case opr.typ of
          OPR_REFERENCE:
            begin
              opr.ref.symbol:=current_asmdata.RefAsmSymbol(tlabelsym(sym).mangledname);
              if opr.ref.segment=NR_NO then
                opr.ref.segment:=NR_CS;
            end;
          else
            begin
              Message(asmr_e_unsupported_symbol_type);
              exit;
            end;
        end;
        hasvar:=true;
        SetupVar:=TRUE;
        Exit;
      end
{$endif i8086}
    else
      begin
        Message(asmr_e_unsupported_symbol_type);
        exit;
      end;
  end;
end;


procedure TOperand.InitRef;
{*********************************************************************}
{  Description: This routine first check if the opcode is of     }
{  type OPR_NONE, or OPR_REFERENCE , if not it gives out an error.    }
{  If the operandtype = OPR_NONE or <> OPR_REFERENCE then it sets up  }
{  the operand type to OPR_REFERENCE, as well as setting up the ref   }
{  to point to the default segment.                                   }
{*********************************************************************}
var
  l : aint;
  hsymofs : aint;
  hsymbol : tasmsymbol;
  reg : tregister;
  hsym_farprocentry: Boolean;
Begin
  case opr.typ of
    OPR_REFERENCE :
      exit;
    OPR_CONSTANT :
      begin
        l:=opr.val;
        opr.typ:=OPR_REFERENCE;
        Fillchar(opr.ref,sizeof(treference),0);
        opr.Ref.Offset:=l;
        opr.varsize:=0;
        opr.constoffset:=0;
        opr.ref_farproc_entry:=false;
      end;
    OPR_NONE :
      begin
        opr.typ:=OPR_REFERENCE;
        opr.varsize:=0;
        opr.constoffset:=0;
        opr.ref_farproc_entry:=false;
        Fillchar(opr.ref,sizeof(treference),0);
      end;
    OPR_REGISTER :
      begin
        reg:=opr.reg;
        opr.typ:=OPR_REFERENCE;
        opr.varsize:=0;
        opr.constoffset:=0;
        opr.ref_farproc_entry:=false;
        Fillchar(opr.ref,sizeof(treference),0);
        opr.Ref.base:=reg;
      end;
    OPR_SYMBOL :
      begin
        hsymbol:=opr.symbol;
        hsymofs:=opr.symofs;
        hsym_farprocentry:=opr.sym_farproc_entry;
        opr.typ:=OPR_REFERENCE;
        opr.varsize:=0;
        opr.constoffset:=0;
        Fillchar(opr.ref,sizeof(treference),0);
        opr.ref.symbol:=hsymbol;
        opr.ref.offset:=hsymofs;
        opr.ref_farproc_entry:=hsym_farprocentry;
      end;
    else
      begin
        Message(asmr_e_invalid_operand_type);
        { Recover }
        opr.typ:=OPR_REFERENCE;
        opr.varsize:=0;
        opr.constoffset:=0;
        opr.ref_farproc_entry:=false;
        Fillchar(opr.ref,sizeof(treference),0);
      end;
  end;
end;

Function TOperand.CheckOperand: boolean;
{*********************************************************************}
{  Description: This routine checks if the operand is of              }
{  valid, and returns false if it isn't. Does nothing by default.     }
{*********************************************************************}
begin
  result:=true;
end;


{****************************************************************************
                                 TInstruction
****************************************************************************}

constructor TInstruction.create(optype : tcoperand);
  var
    i : longint;
  Begin
    { these field are set to 0 anyways by the constructor helper (FK)
    Opcode:=A_NONE;
    Condition:=C_NONE;
    Ops:=0;
    }
    filepos:=current_filepos;
    for i:=1 to max_operands do
      Operands[i]:=optype.create;
    Labeled:=false;
  end;


destructor TInstruction.destroy;
var
  i : longint;
Begin
  for i:=1 to max_operands do
   Operands[i].free;
end;


  function TInstruction.ConcatInstruction(p:TAsmList) : tai;
    var
      ai   : taicpu;
      i : longint;
    begin
      for i:=1 to Ops do
        operands[i].CheckOperand;

      ai:=taicpu.op_none(opcode);
      ai.fileinfo:=filepos;
      ai.Ops:=Ops;
      ai.Allocate_oper(Ops);
      for i:=1 to Ops do
        with operands[i].opr do
          begin
            case typ of
              OPR_CONSTANT :
                ai.loadconst(i-1,val);
              OPR_REGISTER:
                ai.loadreg(i-1,reg);
              OPR_SYMBOL:
                ai.loadsymbol(i-1,symbol,symofs);
              OPR_LOCAL :
                ai.loadlocal(i-1,localsym,localsymofs,localindexreg,
                             localscale,localgetoffset,localforceref);
              OPR_REFERENCE:
                ai.loadref(i-1,ref);
{$ifdef m68k}
              OPR_REGSET:
                ai.loadregset(i-1,regsetdata,regsetaddr,regsetfpu);
{$endif}
{$ifdef ARM}
              OPR_REGSET:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                ai.loadregset(i-1,regtype,subreg,regset,usermode);
              OPR_MODEFLAGS:
                ai.loadmodeflags(i-1,flags);
              OPR_SPECIALREG:
                ai.loadspecialreg(i-1,specialreg,specialregflags);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                ai.loadregset(i-1,regtype,subreg,regset);
              OPR_SHIFTEROP:
                ai.loadshifterop(i-1,shifterop);
              OPR_COND:
                ai.loadconditioncode(i-1,cc);
              OPR_MODEFLAGS:
                ai.loadmodeflags(i-1,flags);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endif ARM}
{$if defined(arm) or defined(aarch64)}
             OPR_SHIFTEROP:
               ai.loadshifterop(i-1,shifterop);
             OPR_COND:
               ai.loadconditioncode(i-1,cc);
{$endif arm or aarch64}
              { ignore wrong operand }
              OPR_NONE:
                ;
              else
                internalerror(200501051);
            end;
          end;
     ai.SetCondition(condition);
     { Concat the opcode or give an error }
      if assigned(ai) then
         p.concat(ai)
      else
       Message(asmr_e_invalid_opcode_and_operand);
      result:=ai;
    end;


{****************************************************************************
                      Symbol table helper routines
****************************************************************************}

procedure AsmSearchSym(const s:string;var srsym:tsym;var srsymtable:TSymtable);
var
  i : integer;
begin
  i:=pos('.',s);
  { allow unit.identifier }
  if i>0 then
    begin
      searchsym(Copy(s,1,i-1),srsym,srsymtable);
      if assigned(srsym) then
       begin
         if (srsym.typ=unitsym) and
            (srsym.owner.symtabletype in [staticsymtable,globalsymtable]) and
            srsym.owner.iscurrentunit then
           searchsym_in_module(tunitsym(srsym).module,Copy(s,i+1,255),srsym,srsymtable)
         else
           begin
             srsym:=nil;
             srsymtable:=nil;
           end;
       end;
    end
  else
    searchsym(s,srsym,srsymtable);
end;


Function SearchType(const hs:string;var size:aint): Boolean;
var
  srsym : tsym;
  srsymtable : TSymtable;
begin
  result:=false;
  size:=0;
  asmsearchsym(hs,srsym,srsymtable);
  if assigned(srsym) and
     (srsym.typ=typesym) then
    begin
      size:=ttypesym(srsym).typedef.size;
      result:=true;
    end;
end;



Function SearchRecordType(const s:string): boolean;
var
  srsym : tsym;
  srsymtable : TSymtable;
Begin
  SearchRecordType:=false;
{ Check the constants in symtable }
  asmsearchsym(s,srsym,srsymtable);
  if srsym <> nil then
   Begin
     case srsym.typ of
       typesym :
         begin
           if ttypesym(srsym).typedef.typ in [recorddef,objectdef] then
            begin
              SearchRecordType:=true;
              exit;
            end;
         end;
       fieldvarsym :
         begin
           if (tfieldvarsym(srsym).vardef.typ in [recorddef,objectdef]) then
             begin
               SearchRecordType:=true;
               exit;
             end;
         end;
     end;
   end;
end;


Function SearchIConstant(const s:string; var l:aint): boolean;
{**********************************************************************}
{  Description: Searches for a CONSTANT of name s in either the local  }
{  symbol list, then in the global symbol list, and returns the value  }
{  of that constant in l. Returns TRUE if successfull, if not found,   }
{  or if the constant is not of correct type, then returns FALSE       }
{ Remarks: Also handle TRUE and FALSE returning in those cases 1 and 0 }
{  respectively.                                                       }
{**********************************************************************}
var
  srsym : tsym;
  srsymtable : TSymtable;
Begin
  SearchIConstant:=false;
{ check for TRUE or FALSE reserved words first }
  if s = 'TRUE' then
   Begin
     SearchIConstant:=TRUE;
     l:=1;
     exit;
   end;
  if s = 'FALSE' then
   Begin
     SearchIConstant:=TRUE;
     l:=0;
     exit;
   end;
{ Check the constants in symtable }
  asmsearchsym(s,srsym,srsymtable);
  if srsym <> nil then
   Begin
     case srsym.typ of
       constsym :
         begin
           if tconstsym(srsym).consttyp=constord then
            Begin
              l:=aint(tconstsym(srsym).value.valueord.svalue);
              SearchIConstant:=TRUE;
              exit;
            end;
         end;
       enumsym:
         Begin
           l:=tenumsym(srsym).value;
           SearchIConstant:=TRUE;
           exit;
         end;
     end;
   end;
end;


Function GetRecordOffsetSize(s:string;Var Offset: aint;var Size:aint; var mangledname: string; needvmtofs: boolean):boolean;
{ search and returns the offset and size of records/objects of the base }
{ with field name setup in field.                              }
{ returns FALSE if not found.                                  }
{ used when base is a variable or a typed constant name.       }
var
  st   : TSymtable;
  harrdef : tarraydef;
  sym  : tsym;
  srsymtable : TSymtable;
  i    : longint;
  base : string;
  procdef: tprocdef;
Begin
  GetRecordOffsetSize:=FALSE;
  Offset:=0;
  Size:=0;
  mangledname:='';
  i:=pos('.',s);
  if i=0 then
   i:=255;
  base:=Copy(s,1,i-1);
  delete(s,1,i);
  if base='SELF' then
   st:=current_structdef.symtable
  else
   begin
     asmsearchsym(base,sym,srsymtable);
     st:=nil;
     { we can start with a var,type,typedconst }
     if assigned(sym) then
       case sym.typ of
         staticvarsym,
         localvarsym,
         paravarsym :
           st:=Tabstractvarsym(sym).vardef.GetSymtable(gs_record);
         typesym :
           st:=Ttypesym(sym).typedef.GetSymtable(gs_record);
       end
     else
       s:='';
   end;
  { now walk all recordsymtables }
  while assigned(st) and (s<>'') do
   begin
     { load next field in base }
     i:=pos('.',s);
     if i=0 then
      i:=255;
     base:=Copy(s,1,i-1);
     delete(s,1,i);
     sym:=search_struct_member(tabstractrecorddef(st.defowner),base);
     if not assigned(sym) then
      begin
        GetRecordOffsetSize:=false;
        exit;
      end;
     st:=nil;
     case sym.typ of
       fieldvarsym :
         with Tfieldvarsym(sym) do
           begin
             if not tabstractrecordsymtable(sym.owner).is_packed then
               inc(Offset,fieldoffset)
             else if tfieldvarsym(sym).fieldoffset mod 8 = 0 then
               inc(Offset,fieldoffset div 8)
             else
               Message(asmr_e_packed_element);
             size:=getsize;
             case vardef.typ of
               arraydef :
                 begin
                   { for arrays try to get the element size, take care of
                     multiple indexes }
                   harrdef:=tarraydef(vardef);
                   while assigned(harrdef.elementdef) and
                         (harrdef.elementdef.typ=arraydef) do
                    harrdef:=tarraydef(harrdef.elementdef);
                   if not is_packed_array(harrdef) then
                     size:=harrdef.elesize
                   else
                     begin
                       if (harrdef.elepackedbitsize mod 8) <> 0 then
                         Message(asmr_e_packed_element);
                       size := (harrdef.elepackedbitsize + 7) div 8;
                     end;
                 end;
               recorddef :
                 st:=trecorddef(vardef).symtable;
               objectdef :
                 st:=tobjectdef(vardef).symtable;
             end;
           end;
       procsym:
         begin
           st:=nil;
           if Tprocsym(sym).ProcdefList.Count>1 then
             Message(asmr_w_calling_overload_func);
           procdef:=tprocdef(tprocsym(sym).ProcdefList[0]);
           if (not needvmtofs) then
             begin
               mangledname:=procdef.mangledname;
             end
           else
             begin
               { can only get the vmtoffset of virtual methods }
               if not(po_virtualmethod in procdef.procoptions) or
                   is_objectpascal_helper(procdef.struct) then
                 Message1(asmr_e_no_vmtoffset_possible,FullTypeName(procdef,nil))
               else
                 begin
                   { size = sizeof(target_system_pointer) }
                   size:=sizeof(pint);
                   offset:=tobjectdef(procdef.struct).vmtmethodoffset(procdef.extnumber)
                 end;
             end;
           { if something comes after the procsym, it's invalid assembler syntax }
           GetRecordOffsetSize:=(s='');
           exit;
         end;
     end;
   end;
   { Support Field.Type as typecasting }
   if (st=nil) and (s<>'') then
     begin
       asmsearchsym(s,sym,srsymtable);
       if assigned(sym) and (sym.typ=typesym) then
         begin
           size:=ttypesym(sym).typedef.size;
           s:=''
         end;
     end;
   GetRecordOffsetSize:=(s='');
end;


Function SearchLabel(const s: string; var hl: tasmlabel;emit:boolean): boolean;
var
  sym : tsym;
  srsymtable : TSymtable;
  hs  : string;
Begin
  hl:=nil;
  SearchLabel:=false;
{ Check for pascal labels, which are case insensetive }
  hs:=upper(s);
  asmsearchsym(hs,sym,srsymtable);
  if sym=nil then
   exit;
  case sym.typ of
    labelsym :
      begin
        if symtablestack.top.symtablelevel<>srsymtable.symtablelevel then
          begin
            Tlabelsym(sym).nonlocal:=true;
            if emit then
              exclude(current_procinfo.procdef.procoptions,po_inline);
          end;
        if not(assigned(tlabelsym(sym).asmblocklabel)) then
          if Tlabelsym(sym).nonlocal then
            current_asmdata.getglobaljumplabel(tlabelsym(sym).asmblocklabel)
          else
            current_asmdata.getjumplabel(tlabelsym(sym).asmblocklabel);
        hl:=tlabelsym(sym).asmblocklabel;
        if emit then
<<<<<<< HEAD
<<<<<<< HEAD
          begin
            if tlabelsym(sym).defined then
              Message(sym_e_label_already_defined);
            tlabelsym(sym).defined:=true
          end
=======
          tlabelsym(sym).defined:=true
>>>>>>> graemeg/fixes_2_2
=======
          tlabelsym(sym).defined:=true
>>>>>>> origin/fixes_2_2
        else
          tlabelsym(sym).used:=true;
        SearchLabel:=true;
      end;
  end;
end;


 {*************************************************************************}
 {                   Instruction Generation Utilities                      }
 {*************************************************************************}


   Procedure ConcatString(p : TAsmList;s:string);
  {*********************************************************************}
  { PROCEDURE ConcatString(s:string);                                   }
  {  Description: This routine adds the character chain pointed to in   }
  {  s to the instruction linked list.                                  }
  {*********************************************************************}
  Begin
     p.concat(Tai_string.Create(s));
  end;


Procedure ConcatConstant(p: TAsmList; value: aint; constsize:byte);
{*********************************************************************}
{ PROCEDURE ConcatConstant(value: aint; maxvalue: aint);        }
{  Description: This routine adds the value constant to the current   }
{  instruction linked list.                                           }
{   maxvalue -> indicates the size of the data to initialize:         }
{                  $ff -> create a byte node.                         }
{                  $ffff -> create a word node.                       }
{                  $ffffffff -> create a dword node.                  }
{*********************************************************************}
var
  rangelo,rangehi : int64;
Begin
  case constsize of
    1 :
      begin
        p.concat(Tai_const.Create_8bit(byte(value)));
        rangelo:=low(shortint);
        rangehi:=high(byte);
      end;
    2 :
      begin
        p.concat(Tai_const.Create_16bit(word(value)));
        rangelo:=low(smallint);
        rangehi:=high(word);
      end;
    4 :
      begin
        p.concat(Tai_const.Create_32bit(longint(value)));
        rangelo:=low(longint);
        rangehi:=high(cardinal);
      end;
    8 :
      begin
        p.concat(Tai_const.Create_64bit(int64(value)));
        rangelo:=0;
        rangehi:=0;
      end;
    else
      internalerror(200405011);
  end;
  { check for out of bounds }
  if (rangelo<>0) and
     ((value>rangehi) or (value<rangelo)) then
    Message(asmr_e_constant_out_of_bounds);
end;


  Procedure ConcatConstSymbol(p : TAsmList;const sym:string;symtyp:tasmsymtype;l:aint);
  begin
    p.concat(Tai_const.Createname(sym,l));
  end;


  Procedure ConcatRealConstant(p : TAsmList;value: bestreal; real_typ : tfloattype);
  {***********************************************************************}
  { PROCEDURE ConcatRealConstant(value: bestreal; real_typ : tfloattype); }
  {  Description: This routine adds the value constant to the current     }
  {  instruction linked list.                                             }
  {   real_typ -> indicates the type of the real data to initialize:      }
  {                  s32real -> create a single node.                     }
  {                  s64real -> create a double node.                     }
  {                  s80real -> create an extended node.                  }
  {                  s64bit ->  create a  comp node.                      }
  {                  f32bit ->  create a  fixed node. (not used normally) }
  {***********************************************************************}
    Begin
       case real_typ of
          s32real : p.concat(tai_realconst.create_s32real(value));
          s64real :
{$ifdef ARM}
           if is_double_hilo_swapped then
<<<<<<< HEAD
<<<<<<< HEAD
             p.concat(tai_realconst.create_s64real_hiloswapped(value))
=======
=======
>>>>>>> origin/fixes_2_2
             p.concat(Tai_real_64bit.Create_hiloswapped(value))
>>>>>>> graemeg/fixes_2_2
           else
{$endif ARM}
<<<<<<< HEAD
             p.concat(tai_realconst.create_s64real(value));
          s80real : p.concat(tai_realconst.create_s80real(value,s80floattype.size));
          sc80real : p.concat(tai_realconst.create_s80real(value,sc80floattype.size));
          s64comp : p.concat(tai_realconst.create_s64compreal(trunc(value)));
          else
            internalerror(2014050608);
=======
             p.concat(Tai_real_64bit.Create(value));
          s80real : p.concat(Tai_real_80bit.Create(value,s80floattype.size));
          sc80real : p.concat(Tai_real_80bit.Create(value,sc80floattype.size));
          s64comp : p.concat(Tai_comp_64bit.Create(trunc(value)));
>>>>>>> graemeg/cpstrnew
       end;
    end;

   Procedure ConcatLabel(p: TAsmList;var l : tasmlabel);
  {*********************************************************************}
  { PROCEDURE ConcatLabel                                               }
  {  Description: This routine either emits a label or a labeled        }
  {  instruction to the linked list of instructions.                    }
  {*********************************************************************}
   begin
     p.concat(Tai_label.Create(l));
   end;

   procedure ConcatAlign(p:TAsmList;l:aint);
  {*********************************************************************}
  { PROCEDURE ConcatPublic                                              }
  {  Description: This routine emits an global   definition to the      }
  {  linked list of instructions.(used by AT&T styled asm)              }
  {*********************************************************************}
   begin
     p.concat(Tai_align.Create(l));
   end;

   procedure ConcatPublic(p:TAsmList;const s : string);
  {*********************************************************************}
  { PROCEDURE ConcatPublic                                              }
  {  Description: This routine emits an global   definition to the      }
  {  linked list of instructions.(used by AT&T styled asm)              }
  {*********************************************************************}
   begin
       p.concat(Tai_symbol.Createname_global(s,AT_LABEL,0));
   end;

   procedure ConcatLocal(p:TAsmList;const s : string);
  {*********************************************************************}
  { PROCEDURE ConcatLocal                                               }
  {  Description: This routine emits an local    definition to the      }
  {  linked list of instructions.                                       }
  {*********************************************************************}
   begin
       p.concat(Tai_symbol.Createname(s,AT_LABEL,0));
   end;


end.
