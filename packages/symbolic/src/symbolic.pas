unit Symbolic;
{
    $ id:                                                       $
    Copyright (c) 2000 by Marco van de Voort(marco@freepascal.org)
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright. (LGPL)

    Base types for expression trees, and some small procs to create them.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

Seems not to have memory leaks atm. If you experience them, check procedure
newcalc first.
}

interface

{$ifdef FPC}
 {$Mode ObjFpc}
{$ENDIF}

Uses Math,Classes,Sysutils;

Const
      VLIWIncr       = 40;    { Initial size and increment of VLIW array}
      DelphiMaxOps   = 5000;  { Unused for FPC. Max records in VLIW array
                                FPC:  2 Gb/sizeof(vliwevalword).}

Type {Should be somewhere in the JCLMath or even in JCLtypes unit}
     {$ifdef FPC}
      ArbFloat = float; {Float is set to mathtype used by FPC Math unit}
      ArbInt   = longint;
     {$else}
      ArbFloat = extended;
      ArbInt   = Integer;
     {$endif}

     calcop=(addo,subo,mulo,dvdo,powo);     {real operators}

     FuncOp=(cosx,sinx,tanx,sqrx,sqrtx,expx,lnx,invx,minus,cotanx,arcsinx,arccosx,
             arctanx,sinhx,coshx,tanhx,arcsinhx,arccoshx,arctanhx,log10x,
             log2x,lnxpix,faculx,arctan2x,stepx,powerx,hypotx,lognx,unknown0,
             unknown1,unknown2,unknown3,unknown4);
       {functions, both one and two parameter ones. Including pseudo function
          minus}

CONST UnknownTokens : array[0..4] OF FuncOp =(unknown0,unknown1,unknown2,
                                              unknown3,unknown4);
TYPE
     Operation=(VarNode,ConstNode,iconstnode,CalcNode,FuncNode,func2node,VLIWVar,CustomNode);
     TFlagsEnum=(ExprIsConstant);  {ExprIsConstant signals that this node of
                                   the tree and deeper can evaluate to a single
                                   float constant}

     TFlags    = SET OF TFlagsEnum;

     pnode =^treenode;
     treenode=record
       Flags : TFlags;
       case nodetype:operation of
         iconstnode: (ivalue:ArbInt);
         VarNode:    (variable:string[11]);
         VLIWVar:    (vliwindex:ArbInt);                       {^float?}
         ConstNode:  (value:ArbFloat);
         CalcNode:   (op:calcop;left,right:pnode);
         FuncNode:   (fun:funcop;son:pnode);
         Func2Node:  (fun2:funcop;son2left,son2right:pnode);
         CustomNode: (Indent:Longint);
       end;

      ERPNStack       = Class(Exception);     {All RPN stack problems category}
      EIError         = Class(Exception);     {All internal errors. Most often
                                                these are raised when unknown
                                                function enumerations are found}
      EDiv0           = Class(Exception);     {Division by zero, but RPN, not processor!}

      TBaseExpression = class
                        protected
                         ExprTree    : pnode;
                         function NewConst(value:ArbFloat):pnode;
                         function NewiConst(value:ArbInt):pnode;
                         function NewCalc(op:calcop;left,right:pnode):pnode;
                         function CopyTree(p :pnode):pnode;
                         function NewFunc(fun:funcop;son:pnode):pnode; overload;
                         function NewFunc(fun:funcop;son,son2:pnode):pnode; overload;
                         function NewVar(variable:string):pnode;
                         procedure DisposeExpr(p:pnode);
                         end;

      EParserStack = class(ERPNStack);    {RPN stack under/overflow.}
      EParserIE    = class(EIError);              {Internal error}

    TBaseExprParser= class(TBaseExpression)
                    public
                     function InFixToParseTree(Expr : String;VAR RPNexpr: String):pnode; virtual;
                     function ParseTreeToRPN  (expr:pnode):string; virtual;
                     function ParseTreeToInfix(expr:pnode):string; virtual;
                    end;

    TEvaluator= CLASS;

    EFaculNotInt = Class(exception);    {Faculty on a real value deviating from an integer value by more than 0.01}
    EExprIE      = Class(EIerror);
    ENotInt      = Class(exception);
    ENotFloat    = Class(Exception);

    TExpression = class(TBaseExprParser)
                    protected
                     InfixClean     : Boolean;
                     InfixCache     : String;
                     Evaluator      : TEvaluator;
                     EvaluatorUpToDate : Boolean;
                     function    GetInfix:String;
                     function    GetRPN:String;
                     procedure Simpleop(expr:TExpression;oper:calcop);
                     function  Simpleopwithresult(expr:TExpression;oper:calcop):TExpression;
                     Function  IntDerive(const derivvariable:String;theexpr:pnode):pnode;
                     Function  GetIntValue:LongInt;
                     Procedure SetIntValue(val:Longint);
                     Function  GetFloatValue:ArbFloat;
                     Procedure SetFloatValue(val:ArbFloat);
                     Procedure UpdateConstants; {Kind of integrity check}
                    public
                     SimplificationLevel : Longint;
                     CONSTRUCTOR Create(Infix:String);
                     CONSTRUCTOR EmptyCreate;
                     DESTRUCTOR Destroy; override;

                     Procedure   SetNewInfix(Infix:String);
                     Function    Derive(derivvariable:String):TExpression;
                     procedure   SymbolSubst(ToSubst,SubstWith:String);
                     function    SymbolicValueNames:TStringList;
                     function    Taylor(Degree:ArbInt;const x,x0:String):TExpression;
                     function    Newton(x:String):TExpression;

                     procedure   SimplifyConstants;

                     function add(Expr:TExpression):TExpression;
                     function dvd(Expr:TExpression):TExpression;
                     function mul(Expr:TExpression):TExpression;
                     function power(Expr:TExpression):TExpression;
                     function sub(Expr:TExpression):TExpression;

                     procedure Addto(Expr:TExpression);
                     procedure Divby(Expr:TExpression);
                     procedure RaiseTo(Expr:TExpression);
                     procedure SubFrom(Expr:TExpression);
                     procedure Times(Expr:texpression);
                     property  InfixExpr: string read GetInfix write SetNewInfix;
                     property  RpnExpr: string read GetRPN;
                     property  ValueAsInteger:longint read GetIntValue write SetIntvalue; {Default?}
                     property  ValueAsFloat:arbfloat   read GetFloatValue write SetFloatValue;
                    end;


    VLIWWordtype=  (avariable,anoperation, afunction,
                   afconstant, aiconstant,placeholder);

     { RPN operators or functions with two arguments are the same.}
     vliwop2=(addv,subv,mulv,dvdv,powv,arctan2v,stepv,hypotv,lognv);

       pArbFloat       = ^ArbFloat;
       {$ifdef FPC}
        pVliwArr       = ^VLIWEvalWord;
       {$else} {Delphi doesn't allow referencing of ^simpletype as array,
                  but does allow it for ^ array of VLIWEvalWord}
        TVLIWArr       = array[0..DelphiMaxOps] of VLiwEvalWord;
        pVliwArr       = ^TVliwArr;
       {$ENDIF}

       pVLIWEvalWord  = ^VLIWEvalWord;
        VLIWEvalword  = record	
                         case VLIWEntity :  VLIWWordType OF
                          AVariable  : (IndexOfVar : ArbInt);
                          AnOperation: (op:vliwop2);       {2 arguments}
                          AFunction  : (fun1:funcop);   {functions with one argument}
                          AiConstant : (ivalue:ArbInt);
                          AfConstant : (value:ArbFloat);
                          placeholder: (IndexOfConstant:ArbInt) ;
                         end;

     TEvaluatorNotEnoughVariables=class(Exception);    {Not enough variables passed to Evaluate}
     TEvaluatorStackException    =class(ERPNStack); {RPN Stack over/under flow}
     TEvaluatorBadConstant       =class(Exception);    {Constant value not specified}
     TEvaluatorIE                =class(Exception);    {Internal error. Probably something out of sync.}

     TEvaluator     = Class {Only needs the notion of a pnode }
                      Private
                       VariableName     : TStringList;
                       ConstantValue    : TList;
                       ConstantNames    : TStringList;
                       MaxStack,
                       VLIWCount,
                       VLIWAlloc        : ArbInt;
                       VLIWRPNExpr      : pVLIWArr;
                      public
                       function    Evaldepth:longint;
                       PROCEDURE   SetConstant(Name:String;Value:ArbFloat);
                       CONSTRUCTOR Create(VariableList:TStringList;Expression:pnode);
                       CONSTRUCTOR Create(VariableList:TStringList;Expression:TExpression);
                       DESTRUCTOR  Destroy; override;
                       Procedure   TreeToVLIWRPN(expr:pnode);
                       function    Evaluate(const variables:Array of ArbFloat):ArbFloat;
                       {$IFDEF DebugDump}
                         procedure debugger;
                         procedure WriteVliw(p:VLIWEvalWord);

                       {$ENDIF}
                       end;

{
 Structures used to index a pnode tree to identify terms.

     PTerms = ^TTerms;
     PtermNode=^TTermNode;
     TtermNode= record
                 NrTerms :ArbInt;
                 Terms  : Array[0..499] of PNode;
                end;
     TTerms = record
                 NrTerms : ArbInt;
                 Terms: Array[0..499] of PtermNode;
                end;
}
const InfixOperatorName   : array[addo..powo] of char= ('+','-','*','/','^');
      FunctionNames    : array[cosx..lognx] of string[8]=(
             'cos','sin','tan','sqr','sqrt','exp','ln','inv','-',
             'cotan','arcsin','arccos','arctan','sinh',
             'cosh','tanh','arcsinh','arccosh','arctanh',
             'log10','log2','lnxp1','!','arctan2',
             'step','power','hypot','logn');
     FunctionNamesUpper: array[cosx..lognx] of string[8]=(
             'COS','SIN','TAN','SQR','SQRT','EXP','LN','INV','-',
             'COTAN','ARCSIN','ARCCOS','ARCTAN','SINH',
             'COSH','TANH','ARCSINH','ARCCOSH','ARCTANH',
             'LOG10','LOG2','LNXP1','!','ARCTAN2',
             'STEP','POWER','HYPOT','LOGN');
     LenFunctionNames : array[cosx..lognx] of longint=
             (3,3,3,3,4,3,2,3,1,5,6,6,6,4,4,4,7,7,7,5,4,5,1,7,4,5,5,4);

{$I exprstrs.inc}

function QuickEvaluate(formula:ansistring;variablenames : array of ansistring;variablevalues:array of const):Double;

implementation


{newconst and newiconst are overloaded in FPC}

function TBaseExpression.NewConst(value:ArbFloat):pnode;
{Generate a new node for a floating point constant}

var t : pnode;

begin
 new(t);
 t^.nodetype:=constnode;
 t^.value:=value;
 t^.Flags:=[ExprIsConstant];
 NewConst:=T;
end;

function TBaseExpression.NewiConst(value:ArbInt):pnode;
{Generate a new node for integer constant}

var t : pnode;

begin
 new(t);
 t^.nodetype:=iconstnode;
 t^.ivalue:=value;
 t^.Flags:=[ExprIsConstant];
 NewiConst:=T;
end;

procedure TBaseExpression.DisposeExpr(p:pnode);
{Recursively kill expression tree}

begin
 IF p<>NIL THEN
  begin
   case p^.nodetype of
    CalcNode : begin
                 DisposeExpr(p^.right);
                 DisposeExpr(p^.left);
               end;
    FuncNode : DisposeExpr(p^.son);
    end;
   Dispose(p);
  end;
end;

function TBaseExpression.NewCalc(op:calcop;left,right:pnode):pnode;
{Create NewCalc node. Left and Right may be nil because
to avoid introducing empty nodes, the deriv()
function may return NIL's, which are to be treated as newiconst(0);

Also one of the functions most likely to have memory leaks
}

function isintegerone(testme:pnode) : boolean;
begin
 Isintegerone:=(testme^.nodetype=iconstnode) and (testme^.ivalue=1);
end;

var t : pnode;

begin
  if op=powo then
   begin
    if right=NIL then                   {x^0 =1 for every X}
     begin
      DisposeExpr(left);
      newcalc:=newiconst(1);
      exit;
     end;
    if left=NIL THEN                    { 0^y = 0 except for y=0, but that is}
     begin                              { covered above}
      DisposeExpr(right);
      NewCalc:=NIL;
      exit;
     end;
    if IsIntegerone(left) then           {x^1 =x}
     begin
      DisposeExpr(left);
      NewCalc:=right;
      exit;
     end;
    If IsIntegerone(right) then             { 1^y=1}
     begin
      DisposeExpr(left);
      NewCalc:=right;
      exit;
     end;
   end; {generate a plain power node for all other cases}
  if left=NIL then
   begin
    if (right=nil) or (op=mulo) or (op=dvdo) then     { 0*0, 0*t or  0/t =0}
     begin                              { We have no way to check T for nul}
      IF Right<>NIL then
       DisposeExpr(Right);
      NewCalc:=NIL;
      exit;
     end;
    if op=addo then   {  Don't generate a calc node for 0+x, but return x}
     begin
      NewCalc:=right;
      exit;
     end;
    new(t);
    t^.nodetype:=funcnode; { 0-x = minus(x) }
    t^.fun:=minus;
    t^.son:=right;
    t^.flags:=[];
    if ExprIsConstant in t^.son^.flags then
     include(t^.flags,ExprIsConstant);
    NewCalc:=T;
    exit;
   end;
  if right=NIL then
   begin
    if (left=nil) or (op=mulo) or (op=dvdo) then     { 0*0, 0*t or  0/t =0}
     begin
      IF left<>NIL then
       disposeExpr(Left);
      NewCalc:=Nil;
      exit;
     end;
    NewCalc:=Left;      { for x-0 or x+0, simply return 0}
    exit;
   end;

 If ((op=mulo) or (op=dvdo)) and isintegerone(right) then  { simplify t*1 and t/1}
  begin
   DisposeExpr(right);
   NewCalc:=Left;
   exit;
  end;
 if (op=mulo) and isintegerone(left) then                 { simplify 1*t}
   begin
   DisposeExpr(left);
   NewCalc:=right;
   exit;
  end;
 new(t);
 t^.nodetype:=calcnode;
 t^.op:=op;
 t^.left:=left;
 t^.right:=right;
 t^.Flags:=[];
 if (ExprIsConstant In T^.Left^.Flags) and (ExprIsConstant In T^.Right^.Flags) then
  include(t^.flags,ExprIsConstant);
 newcalc:=t;
end;

function TBaseExpression.CopyTree(p :pnode):pnode;

var newtree : pnode;

begin
 new(newtree);
 move(p^,Newtree^,sizeof(treenode));
 if newtree^.nodetype=CalcNode then
  begin
   newtree^.left:=CopyTree(p^.left);
   newtree^.right:=CopyTree(p^.right);
  end
 else
  if newtree^.nodetype=FuncNode then
   newtree^.son:=CopyTree(p^.son);
 CopyTree:=NewTree;
end;

function TBaseExpression.NewFunc(fun:funcop;son:pnode):pnode;

var t : pnode;

begin
 IF son<>nil then
  begin
   new(t);
   t^.nodetype:=funcnode;
   t^.fun:=fun;
   t^.son:=son;
   t^.flags:=[];
   if ExprIsConstant IN son^.flags then
    Include(t^.flags,ExprIsConstant);
   NewFunc:=T;
  end
 else
  NewFunc:=NIL;
end;

function TBaseExpression.NewFunc(fun:funcop;son,son2:pnode):pnode;

var t : pnode;

begin
 new(t);
 t^.nodetype:=func2node;
 t^.fun:=fun;
 t^.son2Left:=son;
 t^.son2Right:=son2;
 t^.flags:=[];
 if(ExprIsConstant IN son^.flags) and (ExprIsConstant IN son2^.flags) then
  Include(t^.flags,ExprIsConstant);
 NewFunc:=T;
end;

{function TBaseExpression.NewFunc(fun:funcop;unknownIdent:longint):pnode;

var t : pnode;

begin
 new(t);
 t^.nodetype:=func2node;
 t^.fun:=fun;
 t^.son2Left:=son;
 t^.son2Right:=son2;
 t^.flags:=[];
 if(ExprIsConstant IN son^.flags) and (ExprIsConstant IN son2^.flags) then
  Include(t^.flags,ExprIsConstant);
 NewFunc:=T;
end;}



function TBaseExpression.NewVar(variable:string):pnode;

var p :pnode;

begin
 new(p);
 p^.nodetype:=varnode;
 p^.variable:=variable;
 p^.Flags:=[];
 newvar:=p;
end;

{$I parsexpr.inc} {Parser categories}
{$I symbexpr.inc} {standard symbolic manip}
{$I teval.inc}
{$I rearrang.inc}


function QuickEvaluate(formula:ansistring;variablenames : array of ansistring;variablevalues:array of const):Double;

VAR Expr    : TExpression;
    SymVars,VarName : TStringList;
    I,j       : Longint;
    Eval    : TEvaluator;
    Vars    : Array[0..1] OF ArbFloat;
    x : double;
begin
 Expr:=TExpression.Create(formula);
 try
   SymVars:=Expr.SymbolicValueNames;
   try
     VarName:=TStringList.Create;
       try
         Eval:=TEvaluator.Create(Varname,Expr);
         try 
           if high(variablenames)>=0 then
             begin
               for i:=low(variablenames) to high(variablenames) do
                 begin
                   j:=symvars.indexof(variablenames[i]);
                   if j<>-1 then
                     begin
                       case variablevalues[i].vtype of
                         vtinteger : x:=variablevalues[i].vinteger;
                         vtextended: x:=variablevalues[i].vextended^;		
                       else
                         raise exception.CreateFmt(SEvalUnknownParameterType,[variablenames[i]]);
                         end;
                       Eval.SetConstant(variablenames[i],x);
                       symvars.objects[j]:=tobject(1);
                     end;
                 end;
                 
             end; 
           i:=0;
           while (i<symvars.count) and (symvars.objects[i]=tobject(1)) do inc(i);
           if i<symvars.count then
             begin
               writeln(i, ' ',symvars.count);
               raise Exception.CreateFmt(SEvalUndefinedVar,[symvars[i]]);
             end;
           result:=Eval.Evaluate([]);
         finally 
           varname:=nil;
           eval.free;
           end; 
       finally
         VarName.free;
         end
   finally
     SymVars.free;
     end;
 finally
   expr.free;
   end;
end;

end.
{
  $Log$
  Revision 1.1  2002/12/15 21:01:26  marco
  Initial revision

}
