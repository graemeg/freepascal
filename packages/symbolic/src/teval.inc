{
    $ id:                                                       $
    Copyright (c) 2000 by Marco van de Voort (marco@freepascal.org)
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright. (LGPL)

    Evaluator class implementation. Evaluates a parsetree expression in
    a way optimized for fast repeated evaluations of the same expression
    with different variables and constants.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************
}

{$IFDEF DebugDump}
procedure TEvaluator.WriteVliw(p:VLIWEvalWord); forward;
{$ENDIF}

Procedure TEvalInternalError(A,B:ArbInt);

VAR S,S2 : String;

begin
 Str(ORD(A),S);
 Str(ORD(B),S2);
 Raise TEvaluatorIE.Create(SEvalIE+S+' '+S2);
end;


CONSTRUCTOR TEvaluator.Create(VariableList:TStringList;Expression:pnode);
{Constructor. Stringlist to set the order of variables in the function while
xconverting the pnode tree to a TEvaluator structure. This avoids any string
parsing during a real evaluation, and moves all stringparsing to the setup.

So for Func(x,y,z) Variablelist contains ('x','y','z') in that order.
}

begin
 VariableName:=VariableList;
 ConstantNames:=TStringList.Create;
 ConstantValue:=TList.Create;
 Getmem(VLIWRPnExpr,SIZEOF(VLIWEvalWord)*VLIWIncr);
 VLIWCount:=0;
 VLIWAlloc:=VLIWIncr;
 MaxStack :=0;
 TreeToVLIWRPN(Expression);
end;

CONSTRUCTOR TEvaluator.Create(VariableList:TStringList;Expression:TExpression);
{Overloaded, same as other constructor. (which it even calls), except that
it has a TExpression as argument.

Besides that it gets the pnode from the TExpression, it sets the
TExpression.Evaluator to self, and a flag to set in the TExpression that its
assiociated TEvaluator is up to date with the TExpression.
}

begin
 Self.Create(VariableList,Expression.ExprTree);
 Expression.Evaluator:=Self;
 Expression.EvaluatorUpToDate:=TRUE;
end;

DESTRUCTOR TEvaluator.Destroy;

VAR I       : LONGINT;
    TmpList : Tlist;

begin
 VariableName.Free;
 ConstantNames.Free;
 IF ConstantValue.Count>0 THEN
  FOR I:=0 to ConstantValue.Count -1 DO
   begin
    TmpList:=TList(ConstantValue[I]);
    TmpList.Free;
   end;
 ConstantValue.Free;
 If VLIWAlloc>0 THEN
  FreeMem(VLIWRPNExpr,VLIWAlloc*SIZEOF(VLIWEvalWord));
 inherited Destroy;
end;

PROCEDURE   TEvaluator.SetConstant(Name:String;Value:ArbFloat);

Var Ind,I    : Longint;
    TmpList  : TList;

begin
 Ind:=ConstantNames.IndexOf(Name);
 If Ind<>-1 THEN
  begin
   TmpList:=TList(ConstantValue[Ind]);
   I:=TmpList.Count;
   If I>0 Then
    For I:=0 TO TmpList.Count-1 DO
     begin
      PVLIWEvalWord(TmpList[I])^.VLIWEntity:=AfConstant;
      PVLIWEvalWord(TmpList[I])^.Value:=Value;
     end;
   end;
end;

procedure TEvaluator.TreeToVLIWRPN(expr:pnode);

procedure CheckVLIWArr;

begin
 if VLIWCount=VLIWAlloc then
  begin
   ReAllocMem(VLIWRPNExpr,(VLIWAlloc+VLIWIncr)*SIZEOF(VLIWEvalWord));
   Inc(VLIWAlloc,VLIWIncr);
  end;
end;

procedure searchTree(Tree:pnode);

var Ind : ArbInt;
    TmpList : TList;

begin
 if tree<>nil then
 case Tree^.nodetype of
  VarNode  : begin
              {some variable or constant. First: Variable?}
               Ind:=VariableName.IndexOf(Tree^.Variable);
               If Ind<>-1 then
                begin {We have identified a variable}
                 CheckVLIWArr; {Make sure there is at least room for one variable}
                 WITH VLIWRPNExpr[VLIWCount] do
                  begin
                   VLIWEntity:=AVariable;
                   IndexOfVar:=Ind;
                  end;
                  {$IFDEF DebugDump}
                  WriteVliw(VLIWRPNExpr[VLIWCOUNT]);
                  {$ENDIF}
                  inc(VLIWCount);
                end
               else
                begin {We have a constant}
                 ind:=ConstantNames.IndexOf(Tree^.Variable);
                 if Ind=-1 then
                  begin {That doesn't exist. Make sure it exists}
                   ConstantNames.Add(Tree^.Variable);
                   TmpList:=TList.Create;
                   ConstantValue.Add(TmpList);
                  end
                 else
                  begin
                   TmpList:=tlist(ConstantValue[Ind]);
                  end;

                 {Create the VLIW record}
                 CheckVLIWArr;

                 WITH VLIWRPNExpr[VLIWCount] do
                  begin
                   VLIWEntity:=Placeholder;
                   IndexOfConstant:=255;
                  end;
                  {$IFDEF DebugDump}
                  WriteVliw(VLIWRPNExpr[VLIWCOUNT]);
                  {$ENDIF}

                 {Store a pointer to the VLIW record to be able to change the
                  constant}
                 TmpList.Add(pointer(VLIWCount)); {Can't pick pointer here, due to realloc}
                 inc(VLIWCount);
                end; {Ind<>-1}
              end;
  ConstNode: begin

              CheckVLIWArr;
              WITH VLIWRPNExpr[VLIWCount] do
               begin
                VLIWEntity:=AfConstant;
                Value:=tree^.value;
               end;
               {$IFDEF DebugDump}
                  WriteVliw(VLIWRPNExpr[VLIWCOUNT]);
                  {$ENDIF}

              inc(VLIWCount);
             end;
 iconstnode: begin
               CheckVLIWArr;
              WITH VLIWRPNExpr[VLIWCount] do
               begin
                VLIWEntity:=AiConstant;
                IValue:=tree^.ivalue;
               end;
               {$IFDEF DebugDump}
                  WriteVliw(VLIWRPNExpr[VLIWCOUNT]);
                  {$ENDIF}

              inc(VLIWCount);
             end;
 CalcNode  : begin

              CheckVLIWArr;
              WITH VLIWRPNExpr[VLIWCount] do
               begin
                VLIWEntity:=AnOperation;
                op:=vliwop2(ord(Tree^.op));
               end;
               {$IFDEF DebugDump}
                  WriteVliw(VLIWRPNExpr[VLIWCOUNT]);
                  {$ENDIF}

              inc(VLIWCount);
              SearchTree(tree^.left);
              SearchTree(tree^.right);
             end;
 FuncNode:    begin

               CheckVLIWArr;
               WITH VLIWRPNExpr[VLIWCount] do
                begin
                 VLIWEntity:=AFunction;
                 fun1:=Tree^.fun;
                end;
                {$IFDEF DebugDump}
                  WriteVliw(VLIWRPNExpr[VLIWCOUNT]);
                  {$ENDIF}

               inc(VLIWCount);
               SearchTree(tree^.son);

              end;
 Func2Node:   begin

               CheckVLIWArr;
               WITH VLIWRPNExpr[VLIWCount] do
                begin
                 VLIWEntity:=AnOperation;
                 if tree^.fun2=powerx then
                  op:=VLIWOp2(powo)
                 else
                  if tree^.fun2 >powerx then
                   op:=vliwop2(ord(powv)+ord(tree^.fun2)-ord(arctan2x))
                  else
                   op:=vliwop2(1+ord(powv)+ord(tree^.fun2)-ord(arctan2x))
                end;
                {$IFDEF DebugDump}
                  WriteVliw(VLIWRPNExpr[VLIWCOUNT]);
                  {$ENDIF}

               inc(VLIWCount);
               SearchTree(tree^.son2left);
               SearchTree(tree^.son2right);
              end
           else
            TEvalInternalError(4,ORD(Tree^.nodetype ));
    end;
end;

Procedure FixLists;
{We added constants as VLIWCount indexes. To speed up we convert them to
pointers. We couldn't do that directly as a consequence of the ReAlloc.}

VAR I,J : Longint;
    TmpList : TList;

begin
 I:=ConstantValue.Count;
 IF I>0 THEN
  FOR J:=0 TO I-1 DO
   begin
    TmpList:=TList(ConstantValue[J]);
    IF (Tmplist<>NIL) and (TmpList.Count>0) then
      for I:=0 TO TmpList.Count-1 DO
       TmpList[I]:=@VLIWRPNExpr[longint(TmpList[I])];
    end;
end;

begin
 VLIWCount:=0;
 SearchTree(expr);
 FixLists;
end;

function TEvaluator.Evaluate(const variables:Array of ArbFloat):ArbFloat;
{The one that does the work}

CONST StackDepth=50;

 var TheArray   : pVLIWEvalWord;
     VLIWRecs   : Longint;
     RPNStack   : ARRAY[0..StackDepth] OF ArbFloat;
     I,
     RPNPointer : Longint;
//     S          : ansiString;

procedure push(Val:ArbFloat); {$IFDEF FPC} InLine; {$endif}

begin
 IF RPNPointer=StackDepth THEN
   RAISE TEvaluatorStackException.Create(SEvalStackDepthExceeded);
 RPNStack[RpnPointer]:=Val;
 INC(RPNPointer);
end;

begin
 VLIWRecs:=VariableName.Count;
 if (High(Variables)+1)<>VLIWRecs then
  Raise TEvaluatorNotEnoughVariables.Create(SeValBadNumberOfVars);
 RPNPointer:=0;
 VliwRecs:=VliwCount-1;
 TheArray:=@VLIWRPNExpr[VLIWRecs];
 REPEAT
  {$IFDEF DebugMe}
  Writeln(VliwRecs,' ',ord(TheArray^.VLIWEntity));
  {$ENDIF}
  TheArray:=@VLIWRPNExpr[VLIWRecs];
  CASE TheArray^.VLIWEntity OF
   AVariable :  begin
                 {$IFDEF DebugMe}
                  Writeln('var:', TheArray^.IndexOfVar);
                 {$ENDIF}
                 Push(Variables[TheArray^.IndexOfVar]);
                end;

   AfConstant : begin
                 {$IFDEF DebugMe}
                  Writeln('FP value:', TheArray^.value);
                 {$ENDIF}
                 Push(TheArray^.Value);
                end;
   AiConstant : begin
                 {$IFDEF DebugMe}
                  Writeln('Int value:', TheArray^.ivalue);
                 {$ENDIF}
                 Push(TheArray^.iValue);
                end;
   Placeholder: begin
 //                RAISE TEvaluatorBadConstant.Create(ConstantNames[TheArray^.IndexOfConstant]);
                end;
   AnOperation: begin
                 {$IFDEF DebugMe}
                  Writeln('Operator value:', ord(TheArray^.op));
                 {$ENDIF}
                Case TheArray^.Op of
                     addv : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=RPNStack[RPNPointer]+RPNStack[RPNPointer-1];
                            end;
                     subv : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=RPNStack[RPNPointer]-RPNStack[RPNPointer-1];
                            end;
                     mulv : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=RPNStack[RPNPointer]*RPNStack[RPNPointer-1];
                            end;
                     dvdv : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=RPNStack[RPNPointer]/RPNStack[RPNPointer-1];
                            end;
                     powv : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=Power(RPNStack[RPNPointer],RPNStack[RPNPointer-1]);
                            end;
                 arctan2v : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=ArcTan2(RPNStack[RPNPointer],RPNStack[RPNPointer-1]);
                            end;
                   stepv  : begin
                             Dec(RPNPointer);
                             If RPNStack[RPNPointer-1]>RPNStack[RPNPOINTER] THEN
                              RPNStack[RPNPointer-1]:=1.0
                             else
                              RPNStack[RPNPointer-1]:=0.0;
                            end;
                  hypotv  : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=hypot(RPNStack[RPNPointer],RPNStack[RPNPointer-1]);
                            end;
                   lognv  :  begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=logn(RPNStack[RPNPointer],RPNStack[RPNPointer-1]);
                            end;
                     else
                      TEvalInternalError(1,ORD(TheArray^.op));
                     end;
                   end;
        AFunction : begin
                      {$IFDEF DebugMe}
                       Writeln('function value:', ord(TheArray^.fun1));
                      {$ENDIF}

                     Case TheArray^.Fun1 of
                      cosx: RPNStack[RPNPointer-1]:=cos(RPNStack[RPNPointer-1]);
                      sinx: RPNStack[RPNPointer-1]:=sin(RPNStack[RPNPointer-1]);
                      tanx: RPNStack[RPNPointer-1]:=tan(RPNStack[RPNPointer-1]);
                      sqrx: RPNStack[RPNPointer-1]:=sqr(RPNStack[RPNPointer-1]);
                     sqrtx: RPNStack[RPNPointer-1]:=sqrt(RPNStack[RPNPointer-1]);
                      expx: RPNStack[RPNPointer-1]:=exp(RPNStack[RPNPointer-1]);
                       lnx: RPNStack[RPNPointer-1]:=ln(RPNStack[RPNPointer-1]);
                      invx: RPNStack[RPNPointer-1]:=1/RPNStack[RPNPointer-1];
                     minus: RPNStack[RPNPointer-1]:=-RPNStack[RPNPointer-1];
                    cotanx: RPNStack[RPNPointer-1]:=cotan(RPNStack[RPNPointer-1]);
                   arcsinx: RPNStack[RPNPointer-1]:=arcsin(RPNStack[RPNPointer-1]);
                   arccosx: RPNStack[RPNPointer-1]:=arccos(RPNStack[RPNPointer-1]);
                   arctanx: RPNStack[RPNPointer-1]:=arctan(RPNStack[RPNPointer-1]);
                     sinhx: RPNStack[RPNPointer-1]:=sinh(RPNStack[RPNPointer-1]);
                     coshx: RPNStack[RPNPointer-1]:=cosh(RPNStack[RPNPointer-1]);
                     tanhx: RPNStack[RPNPointer-1]:=tanh(RPNStack[RPNPointer-1]);
                  arcsinhx: RPNStack[RPNPointer-1]:=ArcSinh(RPNStack[RPNPointer-1]);
                  arccoshx: RPNStack[RPNPointer-1]:=ArcCosh(RPNStack[RPNPointer-1]);
                  arctanhx: RPNStack[RPNPointer-1]:=ArcTanh(RPNStack[RPNPointer-1]);
                    log10x: RPNStack[RPNPointer-1]:=Log10(RPNStack[RPNPointer-1]);
                     log2x: RPNStack[RPNPointer-1]:=Log2(RPNStack[RPNPointer-1]);
                    lnxpix: RPNStack[RPNPointer-1]:=lnxp1(RPNStack[RPNPointer-1]);
                    else
                     TEvalInternalError(2,ORD(TheArray^.fun1));
                   end;
                  end;
      else
       TEvalInternalError(3,ORD(TheArray^.VLIWEntity));
  end;
 {$Ifdef DebugDump}
 Writeln('RecordNo: ',VliwRecs);
 IF RPNPointer>0 then
  begin
   Writeln('RPN stack');
   for I:=0 TO RpnPointer-1 DO
    Writeln(I:2,' ',RpnStack[I]);
  end;
 {$Endif}
 dec(TheArray);
 dec(VliwRecs);
 UNTIL VliwRecs<0;
 Result:=RPNStack[0];
end;

{
function TEvaluator.i387Evaluate(const variables:Array of ArbFloat):ArbFloat;
{This should become the really *cool* one in time.
Still experimental though.

Current status:
- Can be entirely FP, but isn't allowed to use more that 4 stack-pos then.
- Math's ARCCOS ARCCOSH ARCSIN ARCSINH ARCTAN2 ARCTANH COSH COTAN HYPOT LNXP1 LOG10
        LOG2 LOGN POWER SINH TAN TANH
     and System.Exp are forbidden because they use stackroom internally.
  This is a problem, because specially Exp() is much too common.
 }

CONST StackDepth=50;

 var TheArray   : pVLIWEvalWord;
     VLIWRecs   : Longint;
     RPNStack   : ARRAY[0..StackDepth] OF ArbFloat;
     I,
     RPNPointer : Longint;

procedure push(Val:ArbFloat); {$IFDEF FPC} InLine; {$endif}

begin
 IF RPNPointer=StackDepth THEN
   RAISE TEvaluatorStackException.Create(SEvalStackDepthExceeded);
 RPNStack[RpnPointer]:=Val;
 INC(RPNPointer);
end;

begin
 VLIWRecs:=VariableName.Count;
 if (High(Variables)+1)<>VLIWRecs then
  Raise TEvaluatorNotEnoughVariables.Create(SeValBadNumberOfVars);
 RPNPointer:=0;
 VliwRecs:=VliwCount-1;
 TheArray:=@VLIWRPNExpr[VLIWRecs];
 REPEAT
  {$IFDEF DebugMe}
  Writeln(VliwRecs,' ',ord(TheArray^.VLIWEntity));
  {$ENDIF}
  TheArray:=@VLIWRPNExpr[VLIWRecs];
  CASE TheArray^.VLIWEntity OF
   AVariable :  begin
                 {$IFDEF DebugMe}
                  Writeln('var:', TheArray^.IndexOfVar);
                 {$ENDIF}
                 Push(Variables[TheArray^.IndexOfVar]);
                end;

   AfConstant : begin
                 {$IFDEF DebugMe}
                  Writeln('FP value:', TheArray^.value);
                 {$ENDIF}
                 Push(TheArray^.Value);
                end;
   AiConstant : begin
                 {$IFDEF DebugMe}
                  Writeln('Int value:', TheArray^.ivalue);
                 {$ENDIF}
                 Push(TheArray^.iValue);
                end;
   Placeholder: begin
 //                RAISE TEvaluatorBadConstant.Create(ConstantNames[TheArray^.IndexOfConstant]);
                end;
   AnOperation: begin
                 {$IFDEF DebugMe}
                  Writeln('Operator value:', ord(TheArray^.op));
                 {$ENDIF}
                Case TheArray^.Op of
                     addv : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=RPNStack[RPNPointer]+RPNStack[RPNPointer-1];
                            end;
                     subv : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=RPNStack[RPNPointer]-RPNStack[RPNPointer-1];
                            end;
                     mulv : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=RPNStack[RPNPointer]*RPNStack[RPNPointer-1];
                            end;
                     dvdv : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=RPNStack[RPNPointer]/RPNStack[RPNPointer-1];
                            end;
                     powv : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=Power(RPNStack[RPNPointer],RPNStack[RPNPointer-1]);
                            end;
                 arctan2v : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=ArcTan2(RPNStack[RPNPointer],RPNStack[RPNPointer-1]);
                            end;
                   stepv  : begin
                             Dec(RPNPointer);
                             If RPNStack[RPNPointer-1]>RPNStack[RPNPOINTER] THEN
                              RPNStack[RPNPointer-1]:=1.0
                             else
                              RPNStack[RPNPointer-1]:=0.0;
                            end;
                  hypotv  : begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=hypot(RPNStack[RPNPointer],RPNStack[RPNPointer-1]);
                            end;
                   lognv  :  begin
                             Dec(RPNPointer);
                             RPNStack[RPNPointer-1]:=logn(RPNStack[RPNPointer],RPNStack[RPNPointer-1]);
                            end;
                     else
                      TEvalInternalError(1,ORD(TheArray^.op));
                     end;
                   end;
        AFunction : begin
                      {$IFDEF DebugMe}
                       Writeln('function value:', ord(TheArray^.fun1));
                      {$ENDIF}

                     Case TheArray^.Fun1 of
                      cosx: RPNStack[RPNPointer-1]:=cos(RPNStack[RPNPointer-1]);
                      sinx: RPNStack[RPNPointer-1]:=sin(RPNStack[RPNPointer-1]);
                      tanx: RPNStack[RPNPointer-1]:=tan(RPNStack[RPNPointer-1]);
                      sqrx: RPNStack[RPNPointer-1]:=sqr(RPNStack[RPNPointer-1]);
                     sqrtx: RPNStack[RPNPointer-1]:=sqrt(RPNStack[RPNPointer-1]);
                      expx: RPNStack[RPNPointer-1]:=exp(RPNStack[RPNPointer-1]);
                       lnx: RPNStack[RPNPointer-1]:=ln(RPNStack[RPNPointer-1]);
                      invx: RPNStack[RPNPointer-1]:=1/RPNStack[RPNPointer-1];
                     minus: RPNStack[RPNPointer-1]:=-RPNStack[RPNPointer-1];
                    cotanx: RPNStack[RPNPointer-1]:=cotan(RPNStack[RPNPointer-1]);
                   arcsinx: RPNStack[RPNPointer-1]:=arcsin(RPNStack[RPNPointer-1]);
                   arccosx: RPNStack[RPNPointer-1]:=arccos(RPNStack[RPNPointer-1]);
                   arctanx: RPNStack[RPNPointer-1]:=arctan(RPNStack[RPNPointer-1]);
                     sinhx: RPNStack[RPNPointer-1]:=sinh(RPNStack[RPNPointer-1]);
                     coshx: RPNStack[RPNPointer-1]:=cosh(RPNStack[RPNPointer-1]);
                     tanhx: RPNStack[RPNPointer-1]:=tanh(RPNStack[RPNPointer-1]);
                  arcsinhx: RPNStack[RPNPointer-1]:=ArcSinh(RPNStack[RPNPointer-1]);
                  arccoshx: RPNStack[RPNPointer-1]:=ArcCosh(RPNStack[RPNPointer-1]);
                  arctanhx: RPNStack[RPNPointer-1]:=ArcTanh(RPNStack[RPNPointer-1]);
                    log10x: RPNStack[RPNPointer-1]:=Log10(RPNStack[RPNPointer-1]);
                     log2x: RPNStack[RPNPointer-1]:=Log2(RPNStack[RPNPointer-1]);
                    lnxpix: RPNStack[RPNPointer-1]:=lnxp1(RPNStack[RPNPointer-1]);
                    else
                     TEvalInternalError(2,ORD(TheArray^.fun1));
                   end;
                  end;
      else
       TEvalInternalError(3,ORD(TheArray^.VLIWEntity));
  end;
 {$Ifdef DebugDump}
 Writeln('RecordNo: ',VliwRecs);
 IF RPNPointer>0 then
  begin
   Writeln('RPN stack');
   for I:=0 TO RpnPointer-1 DO
    Writeln(I:2,' ',RpnStack[I]);
  end;
 {$Endif}
 dec(TheArray);
 dec(VliwRecs);
 UNTIL VliwRecs<0;
 Result:=RPNStack[0];
end;
}

function TEvaluator.Evaldepth:longint;
{estimate stackdepth}

var  TheArray   : pVLIWEvalWord;
     VLIWRecs   : Longint;
     Deepest    : Longint;
     RPNPointer : Longint;

begin
 RPNPointer:=0;
 Deepest:=0;
 VliwRecs:=VliwCount-1;
 TheArray:=@VLIWRPNExpr[VLIWRecs];
 REPEAT
  TheArray:=@VLIWRPNExpr[VLIWRecs];
  CASE TheArray^.VLIWEntity OF
   AVariable,
   afconstant,
   aiconstant,          {a placeholder always changes into a push}
   placeholder : Inc(rpnpointer);
   AnOperation : Dec(rpnpointer); {take two args, put one back}
 {  AFunction :  Doesn't do anything}
  end;
   If Deepest<RPNPointer then
    Deepest:=RPNPointer;
 dec(TheArray);
 dec(VliwRecs);
 UNTIL VliwRecs<0;
 Result:=deepest;
end;

{$IFDEF DebugDump}

CONST VLIWOPNames : array[addv..lognv] of String[9] =
                                       ('add','sub','mul','dd','pow',
                                        'arctan2','step','hypot','logn');

procedure TEvaluator.WriteVliw(p:VLIWEvalWord);

begin
 Write('writevliw ',(ord(p.vliwentity)-ORD(AVariable)):2,' ');
  CASE p.VLIWEntity OF
   AVariable :  Writeln('variable : ', VariableName[p.IndexOfVar]);
  AfConstant :  Writeln('FP value : ', p.value);
  AiConstant :  Writeln('Int value: ', p.ivalue);
   Placeholder: begin
                 writeln('placeholder');
                end;
   AnOperation: begin
                 Write('Operator  : ');

                 IF not (p.OP IN [addv..lognv]) then
                  Writeln('Bad OPERATOR!')
                 ELSE
                  Writeln(VLIWOpNames[p.op]);
                 end;
    AFunction : begin
                   Write('Function: ');
                   IF not (p.fun1 IN [cosx..lognx]) then
                    Writeln('xBad function')
                   ELSE
                    Writeln(FunctionNames[p.fun1]);
                    end;
      else
         Writeln('xBAD Entity');
  end;
end;


procedure TEvaluator.debugger;
{Dump the VLIWArray in textual form for debugging}

 var TheArray   : pVLIWEvalWord;
     VLIWRecs   : Longint;

{$IFNDEF GoUp}
 {$DEFINE GoDown}
{$ENDIF}

begin
 VLIWRecs:=VariableName.Count;
 Writeln('Variables : ',VLIWRecs);
 Writeln('Constants : ',ConstantNames.Count);

 VliwRecs:=VliwCount-1;
 Writeln('VLIWCount : ',VLIWCOUNT);
 {$IFDEF GoDown}
 TheArray:=@VLIWRPNExpr[VLIWRecs-1];
 {$ELSE}
  TheArray:=VLIWRPNExpr;
 {$ENDIF}
 REPEAT
  {$IFDEF GoDown}
   Writeln(VliwRecs,' ',ord(TheArray^.VLIWEntity));
  {$ELSE}
   Writeln(VLIWCount-VliwRecs,' ',ord(TheArray^.VLIWEntity));
  {$ENDIF}
  Writeln('------------------------------------------------------');
  WriteVliw(TheArray^);
 {$IFDEF GoDown}
  dec(TheArray);
 {$ELSE}
  INC(TheArray);
 {$ENDIF}
 dec(VliwRecs);
 UNTIL VliwRecs<0;
end;

{$ENDIF}

{
  $Log$
  Revision 1.1  2002/12/15 21:01:28  marco
  Initial revision

}