{
    $id:                                                       $
    Copyright (c) 2000 by Marco van de Voort (marco@freepascal.org)
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright. (LGPL)

    TExpression class which does symbolic manipulation.

    Derivation routine based on 20 lines of conceptual pseudo code
    provided by Osmo Ronkanen.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

Problems:

- Often untested. I used my HP48g to check some complex derivatives,
   but more thorough checking and errorhandling is necessary.
- RPN to Infix adds ()'s when not necessary. Should be made aware of precedence.
 (partially fixed)
}

{Should be moved to a math unit. Calculate x! with a 23 degree polynomal for
 ArbFloat values. From Numlib.


Works for extended only. Redefine TC1 and TC3 for your numeric type
if you want to use something else.}

type Float10Arb =ARRAY[0..9] OF BYTE;

const

   TC1 : Float10Arb = (0,0,$00,$00,$00,$00,0,128,192,63);         {Eps}
   TC3 : Float10Arb = (1,0,0,0,0,0,0,0,0,0);                      {3.64519953188247460E-4951}


var {Looks ugly, but is quite handy.}
    macheps  : ArbFloat absolute TC1;  { macheps = r - 1,  with r
                                        the smallest ArbFloat > 1}
    midget   : ArbFloat absolute TC3;  { the smallest positive ArbFloat}


function spepol(x: ArbFloat; var a: ArbFloat; n: ArbInt): ArbFloat;
var   pa : ^ArbFloat; {FPC extension. Uses ^ some array of ArbFloat in TP}
       i : ArbInt;
    polx : ArbFloat;
begin
  pa:=@a;
  polx:=0;
  for i:=n downto 0 do
    polx:=polx*x+pa[i]; {and pa^[i] here}
  spepol:=polx
end {spepol};


function spegam(x: ArbFloat): ArbFloat;
const

    tmax = 170;
    a: array[0..23] of ArbFloat =
    ( 8.86226925452758013e-1,  1.61691987244425092e-2,
      1.03703363422075456e-1, -1.34118505705967765e-2,
      9.04033494028101968e-3, -2.42259538436268176e-3,
      9.15785997288933120e-4, -2.96890121633200000e-4,
      1.00928148823365120e-4, -3.36375833240268800e-5,
      1.12524642975590400e-5, -3.75499034136576000e-6,
      1.25281466396672000e-6, -4.17808776355840000e-7,
      1.39383522590720000e-7, -4.64774927155200000e-8,
      1.53835215257600000e-8, -5.11961333760000000e-9,
      1.82243164160000000e-9, -6.13513953280000000e-10,
      1.27679856640000000e-10,-4.01499750400000000e-11,
      4.26560716800000000e-11,-1.46381209600000000e-11);

var tvsmall, t, g: ArbFloat;
             m, i: ArbInt;
begin
  if sizeof(ArbFloat) = 6
  then
    tvsmall:=2*midget
  else
    tvsmall:=midget;
  t:=abs(x);
  if t > tmax
  then
    RunError(407);
  if t < macheps
  then
    begin
      if t < tvsmall
      then
        RunError(407);
      spegam:=1/x
    end
  else  { abs(x) >= macheps }
    begin
      m:=trunc(x);
      if x > 0
      then
        begin
          t:=x-m; m:=m-1; g:=1;
          if m<0
          then
            g:=g/x
          else
            if m>0
            then
              for i:=1 to m do
                g:=(x-i)*g
        end
      else { x < 0 }
        begin
          t:=x-m+1;
          if t=1
          then
            RunError(407);
          m:=1-m;
          g:=x;
          for i:=1 to m do
            g:=(i+x)*g;
          g:=1/g
        end;
      spegam:=spepol(2*t-1, a[0], sizeof(a) div sizeof(ArbFloat) - 1)*g
    end { abs(x) >= macheps }
end; {spegam}


Procedure ExprInternalError(A,B:ArbInt);

VAR S,S2 : String;

begin
 Str(ORD(A),S);
 Str(ORD(B),S2);
 Raise EExprIE.Create(SExprIE+S+' '+S2);
end;

CONSTRUCTOR TExpression.Create(Infix:String);

var dummy:String;

begin
 ExprTree:=NIL;

 if (Infix<>'') then
  ExprTree:=InfixToParseTree(Infix,Dummy);
 InfixCache:=Infix;
 InfixClean:=True;                      {Current pnode status' infix
                                            representation is in infixcache}
end;

CONSTRUCTOR TExpression.EmptyCreate;

begin
 ExprTree:=Nil;
 InfixClean:=false;
end;

Procedure TExpression.SetNewInfix(Infix:String);

var dummy:String;

begin
 if Assigned(ExprTree) Then
  Dispose(ExprTree);
 if infix<>'' then
  ExprTree:=InFixToParseTree(Infix,Dummy)
 else
  ExprTree:=NIL;
 InfixClean:=True;
 InfixCache:=Infix;
end;

Destructor TExpression.Destroy;

begin
 If assigned(ExprTree) then
  DisposeExpr(ExprTree);
 inherited Destroy;
end;

function TExpression.GetRPN :String;

begin
 if ExprTree=NIL Then
  Result:='0'
 else
  Result:=ParseTreeToRpn(ExprTree);
end;

function TExpression.GetInfix:String;

begin
 if Not InfixClean then
  begin
   If ExprTree=NIL THEN
    InfixCache:='0'
   else
    InfixCache:=ParseTreeToInfix(ExprTree);
   InfixClean:=True;
  end;
 Result:=InfixCache;
end;

Function  TExpression.GetIntValue:LongInt;

begin
 SimplifyConstants;
 If ExprTree^.NodeType<>Iconstnode then
  Raise ENotInt.Create(SExprNotInt);
 result:=ExprTree^.ivalue;
end;

Procedure TExpression.SetIntValue(val:Longint);

begin
 if ExprTree<> NIL then
  DisposeExpr(ExprTree);
 New(ExprTree);
 ExprTree^.NodeType:=iconstnode;
 ExprTree^.Ivalue:=Val;
end;

Function  TExpression.GetFloatValue:ArbFloat;

begin
 If ExprTree^.NodeType<>constnode then
  Raise ENotFloat.Create(SExprNotFloat);
 result:=ExprTree^.value;
end;

Procedure TExpression.SetFloatValue(val:ArbFloat);

begin
 if ExprTree<> NIL then
  DisposeExpr(ExprTree);
 New(ExprTree);
 ExprTree^.NodeType:=constnode;
 ExprTree^.value:=Val;
end;

procedure TExpression.Simpleop(expr:TExpression;oper:calcop);

begin
 exprtree:=NewCalc(oper,exprtree,CopyTree(expr.exprtree));
 InFixCache:='garbadge';
 InfixClean:=False;
end;

function TExpression.Simpleopwithresult(expr:TExpression;oper:calcop):TExpression;

var tmp:pnode;

begin
 result.EmptyCreate;
 result.SimplificationLevel:=simplificationlevel;
 result.exprtree:=NewCalc(Oper,CopyTree(ExprTree),CopyTree(Expr.ExprTree));
end;

procedure TExpression.Addto(Expr:TExpression);

begin
 simpleop(expr,addo);
end;


procedure TExpression.SubFrom(Expr:TExpression);

begin
 simpleop(expr,subo);
end;

procedure TExpression.Times(Expr:texpression);

begin
 simpleop(expr,mulo);
end;

procedure TExpression.Divby(Expr:TExpression);

begin
 simpleop(expr,dvdo);
end;


procedure TExpression.RaiseTo(Expr:TExpression);

begin
 simpleop(expr,powo);
end;


function TExpression.add(Expr:TExpression):TExpression;


begin
 result:=Simpleopwithresult(expr,addo);
end;

function TExpression.sub(Expr:TExpression):TExpression;


begin
 result:=Simpleopwithresult(expr,subo);
end;

function TExpression.dvd(Expr:TExpression):TExpression;

begin
 result:=Simpleopwithresult(expr,dvdo);
end;


function TExpression.mul(Expr:TExpression):TExpression;

begin
 result:=Simpleopwithresult(expr,mulo);
end;

Function  TExpression.IntDerive(const derivvariable:String;theexpr:pnode):pnode;

function Deriv(t:pnode):pnode;
{Derive subexpression T. Returns NIL if subexpression derives to 0, to avoid
unnecessary (de)allocations. This is the reason why NewCalc is so big.}

var x     : ArbFloat;
    p1,p2 : pnode;

begin
  Deriv:=nil;
  if (t=nil) then               {Early out}
   exit;
  with t^ do begin
   case nodetype of
     VarNode:    if upcase(variable)=derivvariable then
                  Deriv:=NewiConst(ArbInt(1))
                 else
                  Deriv:=NIL;
     ConstNode : Deriv:=NIL;
     IConstNode: Deriv:=NIL;
     CalcNode:  begin
                  case op of
                   addo,
                   subo:  Deriv:=NewCalc(op,Deriv(left),Deriv(right));
                   mulo:  Deriv:=NewCalc(addo,
                                 NewCalc(mulo,Deriv(left),copyTree(right)),
                                 NewCalc(mulo,Deriv(right),copytree(left)));

                   dvdo:  Deriv:=NewCalc(dvdo,
                                  NewCalc(subo,
                                   NewCalc(mulo,Deriv(left),copyTree(right)),
                                   NewCalc(mulo,Deriv(right),copytree(left))),
                                   NewFunc(sqrx,CopyTree(right)));
                  powo: begin
                         p1:=Deriv(Right);
                         if P1<>NIL then
                          p1:=NewCalc(mulo,p1,NewFunc(Lnx,CopyTree(Left))); { ln(l)*r'}
                         p2:=Deriv(Left);
                         if P2<>NIL then
                          p2:=Newcalc(Mulo,CopyTree(Right),newcalc(mulo,p2,
                                newfunc(invx,CopyTree(left))));
                         IF (P1<>nil) and (p2<>nil) then
                          deriv:=newcalc(mulo,CopyTree(t),newcalc(addo,p1,p2))
                         else
                          if (P1=NIL) and (P2=NIL) then {Simplify first to avoid this!}
                           deriv:=NIL
                          else
                           begin
                            if P1=NIL THEN { one of both is constant}
                             P1:=P2;       {The appopriate term is now in P1}
                            deriv:=newcalc(mulo,CopyTree(t),p1);
                           end;
                        end;
                     end;
                  end;
     FuncNode: begin
                 case fun of
                   invx:  Deriv:=NewCalc(dvdo,
					  NewFunc(Minus,Deriv(son)),
		          		  NewFunc(sqrx,CopyTree(son)));

                   minus: Deriv:=NewFunc(minus,Deriv(son));
                   sinx:  Deriv:=NewCalc(Mulo,
                            NewFunc(cosx,Copytree(son)),
                            Deriv(son));
                   cosx:  deriv:=NewCalc(mulo,
                            NewFunc(minus,NewFunc(sinx,copytree(son))),
                            Deriv(son));
                   tanx:  deriv:=Newcalc(dvdo,deriv(son),
                            newfunc(sqrx,newfunc(cosx,copytree(son))));
                   sqrx:  deriv:=newcalc(mulo, newiconst(2),
                                  newcalc(mulo,copytree(son),deriv(son)));
                                  { dx*1 /(2*sqrt(x)) }
                   sqrtx: deriv:=newcalc(mulo, deriv(son),newcalc(dvdo,newiconst(1),
                                       newcalc(mulo,newiconst(2),newfunc(sqrtx,copytree(son)))));
                   lnx :  deriv:=newcalc(mulo,newcalc(dvdo,newiconst(1),CopyTree(son)),
                                             deriv(son)); { dln(x)=x' * 1/x}
                   expx:  deriv:=newcalc(mulo,newfunc(expx,copytree(son)),deriv(son));
                 cotanx:  deriv:=newfunc(minus,Newcalc(dvdo,deriv(son),  { -dx/sqr(sin(x))}
                            newfunc(sqrx,newfunc(sinx,copytree(son)))));
                  coshx:  deriv:=newcalc(mulo,newfunc(sinhx,copytree(son)),deriv(son));
                  sinhx:  deriv:=newcalc(mulo,newfunc(coshx,copytree(son)),deriv(son));
               arcsinhx, {according to HP48?}
                arcsinx:  deriv:=newcalc(dvdo,deriv(son),newfunc(sqrtx,newcalc(subo,newiconst(1),
                                newfunc(sqrx,copytree(son)))));
                arccosx:  deriv:=newfunc(minus,newcalc(dvdo,deriv(son),
                                        newfunc(sqrtx,newcalc(subo,newiconst(1),newfunc(sqrx,copytree(son))))));
                arctanx:  deriv:=newcalc(dvdo,deriv(son),newcalc(addo,newiconst(1),newfunc(sqrx,copytree(son))));
                 log10x:  deriv:=newcalc(mulo,newcalc(dvdo,newconst(0.434294481902),CopyTree(son)),
                                             deriv(son)); { dlog10(x)=x' * log10(e)/x}
                  log2x:  deriv:=newcalc(mulo,newcalc(dvdo,newconst(1.44269504089),CopyTree(son)),
                                             deriv(son)); { dlog2(x)=x' * log2(e)/x}
                  stepx:   ;  {Should raise exception, not easily derivatable}
                  tanhx:  deriv:=newcalc(dvdo,deriv(son),newfunc(sqrx,newfunc(coshx,copytree(son))));
               arctanhx:  deriv:=newcalc(dvdo,deriv(son),newfunc(sqrtx,newcalc(addo,newiconst(1),
                                           newfunc(sqrx,copytree(son)))));
               arccoshx:  deriv:=NewCalc(dvdo,deriv(son),newcalc(mulo,newcalc(subo,newfunc(sqrtx,copytree(son)),newiconst(1)),
                                                                    newcalc(addo,newfunc(sqrtx,copytree(son)),newiconst(1))));
     lnxpix,arctan2x,
     hypotx,lognx : ; {Should also raise exceptions, not implemented yet}
                    end;
                  end;
     Func2Node: begin
                  if son2left^.nodetype=constnode then
                   x:=son2left^.value
                  else
                   x:=son2left^.ivalue;
                  case fun of
                 lognx:  deriv:=newcalc(mulo,newcalc(dvdo,newconst(logn(x,2.71828182846)),
                                 CopyTree(son2right)),deriv(son2right));
                                 { dlogn(x)=x' * log(n,e)/x}
                 Powerx: begin
                         p1:=Deriv(Son2Right);
                         if P1<>NIL then
                          p1:=NewCalc(mulo,p1,NewFunc(Lnx,CopyTree(Son2Left))); { ln(l)*r'}
                         p2:=Deriv(Son2Left);
                         if P2<>NIL then
                          p2:=Newcalc(Mulo,CopyTree(Son2Right),newcalc(mulo,p2,
                                newfunc(invx,CopyTree(Son2Left))));
                         IF (P1<>nil) and (p2<>nil) then
                          deriv:=newcalc(mulo,CopyTree(t),newcalc(addo,p1,p2))
                         else
                          if (P1=NIL) and (P2=NIL) then {Simplify first to avoid this!}
                           deriv:=NIL
                          else
                           begin
                            if P1=NIL THEN { one of both is constant}
                             P1:=P2;       {The appopriate term is now in P1}
                            deriv:=newcalc(mulo,CopyTree(t),p1);
                           end;
                        end;
                     end;
                end;
           end;
    end; {WITH}
end;

begin
 Result:=Deriv(theexpr);
end;

function TExpression.power(Expr:TExpression):TExpression;


begin
 result:=Simpleopwithresult(expr,powo);
end;


Function  TExpression.Derive(derivvariable:String):TExpression;

var tmpvar : Pnode;
    DerivObj: TExpression;

begin
 derivvariable:=upcase(derivvariable);
 Tmpvar:=intDerive(derivvariable,exprtree);

 DerivObj:=TExpression.emptycreate;
 If tmpvar=NIL then
  derivobj.ExprTree:=NewIconst(0)
 else
  derivobj.exprtree:=tmpvar;
 derivobj.simplificationlevel:=simplificationlevel;
 DerivObj.InfixClean:=False;
 result:=derivobj;
end;

function ipower(x,y:ArbInt):ArbInt;

var tmpval : ArbInt;

begin
 if y<0 then
  ; {exception}
 if y=0 then
   result:=1
 else
  begin
   result:=x;
   if y<>1 then
    for tmpval:=2 to y do
     result:=result*x;
  end;
end;

function ifacul(x:ArbInt):ArbInt;

var tmpval : ArbInt;

begin
 if x<0 then
  ; {exception}
 if x=0 then
   result:=1
 else
  begin
   result:=1;
   if x<>1 then
    for tmpval:=2 to x do
     result:=result*tmpval;
  end;
end;

function EvaluateFunction(funcname:funcop;param:ArbFloat):ArbFloat;

var Intermed : integer;

begin
      case funcname of
       cosx : result:=Cos(param);
       sinx : result:=sin(param);
       tanx : result:=tan(param);
       sqrx : result:=sqr(param);
      sqrtx : result:=sqrt(param);
       expx : result:=exp(param);
        lnx : result:=ln(param);
     cotanx : result:=cotan(param);
    arcsinx : result:=arcsin(param);
    arccosx : result:=arccos(param);
    arctanx : result:=arctan(param);
      sinhx : result:=sinh(param);
      coshx : result:=cosh(param);
      tanhx : result:=tanh(param);
   arcsinhx : result:=arcsinh(param);
   arccoshx : result:=arccosh(param);
   arctanhx : result:=arctanh(param);
     log10x : result:=log10(param);
      log2x : result:=log2(param);
     lnxpix : result:=lnxp1(param);
     faculx : result:=spegam(param+1.0);
         else
          ExprInternalError(2,ord(funcname));
      end;
   If Result<1E-4900 then {Uncertainty in sinus(0.0)}
    Result:=0;
end;


procedure TExpression.SimplifyConstants;

//procedure internalsimplify (expr:pnode;InCalc:Boolean;parent:pnode);
procedure internalsimplify (expr:pnode);

function isconst(p:pnode):boolean;

begin
 isconst:=(p^.nodetype=iconstnode) or (p^.nodetype=constnode);
end;

function isconstnil(p:pnode):boolean;
begin
 IsConstNil:=false;
 if (p^.nodetype=iconstnode) and (P^.ivalue=0) then
  IsConstNil:=True;
 If (p^.nodetype=constnode) and (P^.value=0) then
  IsConstNil:=True
end;

var val1,val2 : ArbFloat;
    ival1,
    ival2 : ArbInt;

function setupoperation(operat:calcop;simlevel:longint;Postprocess:boolean;param2func:boolean):longint;

function dosimple(mode:longint;theleft,theright:pnode):longint;

begin
 If Mode >3 then
  ;
 result:=0;
 if mode=0 then
  exit;
        if (theright^.nodetype=iconstnode) and (theleft^.nodetype=iconstnode) then
         begin
          if mode=3 then
           begin
            result:=2;
             val2:=theright^.value;
             val1:=theleft^.value;
           end
          else
           begin
            result:=1;
            ival2:=theright^.ivalue;
            ival1:=theleft^.Ivalue;
           end;
         end;
        if (theright^.nodetype=constnode) and (theleft^.nodetype=constnode) then
         begin
          result:=2;
          val2:=theright^.value;
          val1:=theleft^.value;
         end;
    if mode>1 then
     begin
       if result=0 then
        begin
         if (theright^.nodetype=constnode) and (theleft^.nodetype=iconstnode) then
         begin
          result:=3;
          val2:=theright^.value;
          val1:=theleft^.ivalue;
         end;
        if (theright^.nodetype=iconstnode) and (theleft^.nodetype=constnode) then
         begin
          result:=4;
          val2:=theright^.ivalue;
          val1:=theleft^.value;
         end;
      end;
    end;
end;

begin
 Result:=0;
 if SimplificationLevel<>0 then
  if param2func then
   result:=DoSimple(SimLevel,expr^.son2left,expr^.son2right)
  else
   result:=DoSimple(SimLevel,expr^.left,expr^.right);

 with expr^ do
  begin
 IF (result>0) and PostProcess then
  begin
   if (operat<>dvdo) then { Divide is special case. If
                                           integer x/y produces a fraction
                                           we want to be able to roll back}
    begin
     if Param2func then
      begin
       dispose(son2right);
       dispose(son2left);
      end
     else
      begin
       dispose(right);
       dispose(left);
      end;
     if result=1 then
      nodetype:=iconstnode
     else
      nodetype:=constnode;
     flags:=[ExprIsConstant];
    end;
   end;
  end;
end;

procedure Checkvarnode(p:pnode);

var treal:arbfloat;
    error:integer;
    tint :Integer;

begin
   TrimLeft(P^.variable);
   TrimRight(p^.variable);
   Val(p^.variable, treal, Error);
   IF (error=0) then {Conversion to real succeeds. Numeric}
    begin
      p^.flags:=[ExprIsConstant];
      if (Pos('.',p^.variable)=0) and (Pos('E',p^.variable)=0) Then
       begin
        Val(p^.variable,tint,Error);
        If error=0 then
         begin
          p^.nodetype:=iconstnode;
          p^.ivalue:=tint;
         end
        else
         begin
          p^.nodetype:=constnode;
          p^.value:=treal;
         end;
       end
      else
       begin
        p^.nodetype:=constnode;
        p^.value:=treal;
       end;
    end;
end;

var tmpval : ArbInt;
    invdummy: pnode;

begin
 case expr^.nodetype of
  VarNode  : CheckVarnode(expr);   {sometimes a numeric value can slip in as constant.
                        (e.g. as people pass it as symbol to taylor or
                        "subst" methods}

  calcnode : begin
              //If not
              internalsimplify(expr^.left); {Reduce left and right as much as possible}
              internalsimplify(expr^.right);
            if isconst(expr^.left) and isconst(expr^.right) then
             begin
              TmpVal:=Setupoperation(expr^.op,SimplificationLevel,true,false);
              if tmpval>0 then
               with expr^ do
               case op of
                addo :
                       if tmpval=1 then
                        ivalue:=ival1+ival2
                       else
                        value:=val1+val2;
               subo  : if tmpval=1 then
                        ivalue:=ival1-ival2
                       else
                        value:=val1-val2;
               mulo  : if tmpval=1 then
                        ivalue:=ival1*ival2
                       else
                        value:=val1*val2;

               dvdo  : begin
                       if tmpval=1 then
                        begin
                         tmpval:=ival1 div ival2;
                         if (tmpval*ival2)=ival1 then {no rounding, OK!}
                          begin
                           Dispose(expr^.right);
                           Dispose(Expr^.left);
                           nodetype:=iconstnode;
                           ivalue:=tmpval;
                          end; {ELSE do nothing}
                        end
                        else
                         begin
                          dispose(expr^.right);
                          dispose(expr^.left);
                          nodetype:=constnode;
                          value:=val1 / val2;
                         end;
                        flags:=[ExprIsConstant];
                        end;
               powo :  If tmpval=1 then
                        begin
                         if ival2<0 then {integer x^-y -> inv (x^y)}
                          begin
                           expr^.nodetype:=funcnode;
                           expr^.son:=NewIConst(IPower(Ival1,-Ival2));
                          end
                         else
                          ivalue:=ipower(ival1,ival2);
                        end
                      else
                        value:=exp(val2*ln(val1));
                 else
                  ExprInternalError(1,ord(Expr^.op));
                end; {case}
            end {if}
           else {At least one node is symbolic, or both types are wrong}
            begin
             With Expr^ do
              if IsConstNil(Left) then
               begin
                Dispose(Left);
               case op of
                addo : begin
                        InvDummy:=Right;
                        Expr^:=Right^;
                        Dispose(InvDummy);
                       end;
                subo: begin
                        invdummy:=right;
                        NodeType:=funcNode;
                        Fun:=Minus;
                        son:=invdummy;
                        Flags:=Son^.Flags;
                      end;
           mulo,powo,dvdo : begin
                        Dispose(Right);
                        nodetype:=IconstNode;
                        ivalue:=0;
                        Flags:=[ExprIsConstant];
                       end;
                   end;
                 end
               else
                if IsConstNil(Right) then
                 begin
                  if expr^.op<>dvdo then {Leave tree for DVD intact because of exception}
                   Dispose(Right);
                 case expr^.op of
       addo,subo : begin
                    InvDummy:=left;
                    Expr^:=left^;
                    Dispose(InvDummy);
                   end;
           mulo  : begin
                    Dispose(Left);
                    nodetype:=IconstNode;
                    Flags:=[ExprIsConstant];
                    ivalue:=0;
                   end;
           powo  : begin
                    Dispose(Left);
                    nodetype:=IconstNode;
                    Flags:=[ExprIsConstant];
                    ivalue:=1;
                   end;
           dvdo  : Raise EDiv0.Create(SExprInvSimp);
             else
                 ExprInternalError(6,ord(Expr^.op));
              end;
            end;
          end;
          With Expr^ Do
           Begin
            IF (nodetype=calcnode) and (Op in [Mulo,Addo]) then
             begin  {Commutative operator rearrangements, move constants to left}
              if (ExprIsConstant IN Right^.flags) and NOT (ExprIsConstant IN Left^.flags) then
              begin
               InvDummy:=Right;
               Right:=Left;
               Left:=InvDummy;
              end;
             IF (right^.nodetype=calcnode) and (right^.Op in [Mulo,Addo]) then
              begin
              end;
             end;
           End;
          end; {case calcnode}

  funcnode: begin
             internalSimplify(expr^.son);
             Case Expr^.fun of
              Minus : if IsConst(expr^.son) then
                       begin
                        InvDummy:=Expr^.Son;
                        expr^:=InvDummy^;
                        if InvDummy^.Nodetype=IconstNode then
                         expr^.ivalue:=-expr^.ivalue
                        else
                         expr^.value:=-expr^.value;
                        dispose(InvDummy);
                       end;
             invx   : begin
                       InvDummy:=Expr^.son;
                       If InvDummy^.nodeType=ConstNode Then
                        begin
                         if InvDummy^.Value=0.0 then
                          Raise EDiv0.Create(SExprInvMsg);
                         Expr^.NodeType:=ConstNode;
                         Expr^.Value:=1/InvDummy^.Value;
                         Dispose(InvDummy);
                        end
                       else
                        if InvDummy^.nodetype=iconstnode then
                         begin
                          if InvDummy^.iValue=0 then
                           Raise EDiv0.Create(SExprinvmsg);
                          If (InvDummy^.iValue=1) or (InvDummy^.iValue=-1) then
                           begin
                            expr^.NodeType:=Iconstnode;
                            Expr^.iValue:=InvDummy^.iValue;
                            Dispose(InvDummy);
                           end;
                         end;
                      end;
                 else {IE check in EvaluateFunction}
                  if (expr^.son^.nodetype=constnode) and (Expr^.fun<>faculx) then {Other functions, only func(real) is simplified}
                   begin
                    val1:=EvaluateFunction(expr^.fun,Expr^.son^.value);
                    dispose(expr^.son);
                    expr^.nodetype:=constnode;
                    expr^.value:=val1;
                   end;
                end; {Case 2}
           end;

  Func2Node : begin
               internalSimplify(expr^.son2left);
               internalSimplify(expr^.son2right);
               case expr^.fun2 of
                powerx : begin
                        TmpVal:=Setupoperation(powo,SimplificationLevel,true,true);
                        if TmpVal>1 then
                         begin
                          If tmpval=1 then
                           begin
                            if ival2<0 then {integer x^-y -> inv (x^y)}
                             begin
                              new(invdummy);
                              invdummy^.nodetype:=iconstnode;
                              invdummy^.ivalue:=ipower(ival1,-ival2);
                              expr^.nodetype:=funcnode;
                              expr^.son:=invdummy;
                             end
                           else
                            expr^.ivalue:=ipower(ival1,ival2);
                           end;
                          end;
                       end;
               stepx : begin
                       {N/I yet}
                       end;
             arctan2x : begin
                         TmpVal:=Setupoperation(powo,SimplificationLevel,false,true);
                         if tmpval>1 then {1 is integer, which we don't do}
                          begin
                           dispose(expr^.right);
                           dispose(expr^.left);
                           expr^.nodetype:=constnode;
                           expr^.value:=arctan2(ival2,ival1);
                          end;
                        end;
             hypotx   :begin
                         TmpVal:=Setupoperation(powo,SimplificationLevel,false,true);
                         if tmpval>1 then {1 is integer, which we don't do}
                          begin
                           dispose(expr^.right);
                           dispose(expr^.left);
                           expr^.nodetype:=constnode;
                           expr^.value:=hypot(ival2,ival1);
                          end;
                        end;
           lognx:      begin
                         TmpVal:=Setupoperation(powo,SimplificationLevel,false,true);
                         if tmpval>1 then {1 is integer, which we don't do}
                          begin
                           dispose(expr^.right);
                           dispose(expr^.left);
                           expr^.nodetype:=constnode;
                           expr^.value:=hypot(ival2,ival1);
                          end;
                        end;
               else
                ExprInternalError(3,ORD(expr^.fun2));
             end;
            end;
{         else
           ExprInternalError(4,ORD(expr^.nodetype));}
       end; {Case 1}
end;

begin
 internalsimplify(exprtree);
 InfixClean:=False; {Maybe changed}
end;

procedure TExpression.SymbolSubst(ToSubst,SubstWith:String);

procedure InternalSubst(expr:Pnode);

begin
 if Expr<>NIL THEN
  case Expr^.NodeType of
   VarNode:   if Expr^.Variable=ToSubst then
               Expr^.Variable:=SubstWith;
   calcnode:  begin
               InternalSubst(Expr^.left);
               InternalSubst(Expr^.right);
              end;
   funcnode:  InternalSubst(Expr^.son);
   func2node: begin
               InternalSubst(Expr^.son2left);
               InternalSubst(Expr^.son2right);
              end;
            end;
end;

begin
 InternalSubst(ExprTree);
end;

function TExpression.SymbolicValueNames:TStringList;

var TheList:TStringList;

procedure InternalSearch(expr:Pnode);

begin
 if Expr<>NIL THEN                      {NIL shouldn't be allowed, and signals corruption. IE? Let it die?}
  case Expr^.NodeType of
   VarNode:  begin
              Expr^.Variable:=UpCase(Expr^.Variable);
              TheList.Add(Expr^.Variable);
             end;
   calcnode:  begin
               InternalSearch(Expr^.left);
               InternalSearch(Expr^.right);
              end;
   funcnode:  InternalSearch(Expr^.son);
   func2node: begin
               InternalSearch(Expr^.son2left);
               InternalSearch(Expr^.son2right);
              end;
            end;
end;

begin
 TheList:=TStringList.Create;
 TheList.Sorted:=TRUE;
 Thelist.Duplicates:=DupIgnore;
 InternalSearch(ExprTree);
 Result:=TheList;
end;

function TExpression.Taylor(Degree:ArbInt;const x,x0:String):TExpression;
{Taylor(x,x0)=sum(m,0,degree, d(f)/d(x))(x0)/ m! * (x-x0)^m)

=   f(x0)+ (x-x0)/1! * df/d(x) + (x-x0)^2  /  2! * d^2(f)/d^2(x) +
       (x-x0)^3  /  3! * d^3(f)/d^3(x) + ....
}

Var TaylorPol    : TExpression;   { The result expression}
    Root,
    Tmp,Tmp2,
    tmp3,tmp4,tmp5 : pnode;       { Always have a nice storage of pointers.
                                    Used to hold all intermediate results}
    I,facul        : ArbInt;      { Loop counters and faculty term}

begin
 TaylorPol:=TExpression.EmptyCreate;      {New expression}
 TaylorPol.ExprTree:=CopyTree(ExprTree);  {make a copy of the parsetree}
 TaylorPol.SymbolSubst(X,X0);             {subst x by x0. All occurances
                                          of  f() refer to x0, not x}
 if Degree>0 then                         {First term only? Or nonsense (negative?)}
                                          {Then ready. 0th term only.}
  begin                                   {Start preparing easy creation of higher terms}
   tmp2:=newcalc(subo,newvar(x),
                      newvar(x0));        {tmp2= x-x0 needed separately for first term}
   tmp4:=Newiconst(5);                    {exponent for x-x0, "a" need to keep a reference}
   tmp3:=newcalc(powo,tmp2,tmp4);         {tmp3= (x-x0)^a}
   tmp5:=newiconst(5);                    {faculty value, "b"=m! also keep a reference for later modification }
   tmp3:=Newcalc(dvdo,tmp3,tmp5);         {tmp3=  (x-x0)^a / b    a and b can be changed}
   facul:=1;                              {Calculate faculty as we go along. Start with 1!=1}
   root:=TaylorPol.ExprTree;              {0th term}
   tmp:=root;                             {term that needs to be differentiated per iteration}
   for i:=1 to Degree do
    begin
     facul:=Facul*i;                      {Next faculty value 1!*1 =1 btw :_)}
     tmp:=intderive(x0,tmp);              {Differentiate f^n(x0) to f^(n+1)(x0)}
     If I=1 then                          {first term is special case. No power }
       tmp2:=NewCalc(mulo,CopyTree(tmp2),tmp) {or faculty needed (^1 and 1!)}
      else
       begin
        tmp5^.Ivalue:=facul;              {Higher terms. Set faculty}
        tmp4^.ivalue:=i;                  {Set exponent (=a) of (x-x0)^a}
        tmp2:=NewCalc(mulo,CopyTree(tmp3),tmp); {multiplicate derivative with (x-x0)^a/b term}
       end;
      root:=NewCalc(addo,root,tmp2);      {String all terms together}
     end;
   DisposeExpr(tmp3);                     {Is only CopyTree()'d, not in new expression}
   TaylorPol.Exprtree:=root;              {Set result}
  end;
 Result:=TaylorPol;
end;

function TExpression.Newton(x:String):TExpression;
{
             f(x)
Newton(x)=x- ----
             f'(x)
}

Var NewtonExpr     : TExpression;   { The result expression}
    Root,
    Tmp,Tmp2,
    tmp3,tmp4,tmp5 : pnode;       { Always have a nice storage of pointers.
                                    Used to hold all intermediate results}
    I,facul        : ArbInt;      { Loop counters and faculty term}

begin
 NewtonExpr:=TExpression.EmptyCreate;      {New expression}

 {Should I test for constant expr here?}

 Tmp:=CopyTree(ExprTree);                 {make a copy of the parsetree
                                            for the f(x) term}
 Tmp2:=intDerive(x,tmp);                  { calc f'(x)}
 Tmp3:=NewVar(x);                         { Create (x)}
 Tmp:=Newcalc(dvdo,tmp,tmp2);             { f(x)/f'(x)}
 tmp:=Newcalc(subo,tmp3,tmp);             { x- f(x)/f'(x)}

 {We built the above expression using a copy of the tree.
     So no pointers into self.exprtree exist. We can now safely assign it
     to the new exprtree}
 NewtonExpr.ExprTree:=tmp;
 NewtonExpr.SimplifyConstants;           {Simplify if f'(x)=constant, and
                                          kill 0*y(x) terms}
 Result:=NewtonExpr;
end;


{
  $Log$
  Revision 1.1  2002/12/15 21:01:26  marco
  Initial revision

}
