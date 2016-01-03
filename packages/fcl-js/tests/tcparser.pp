unit tcparser;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testutils, testregistry, jsParser, jstree, jsbase;

type

  { TTestJSParser }

  TTestJSParser= class(TTestCase)
  Private
    FSource : TStringStream;
    FParser : TJSParser;
    FSE : TJSSourceElements;
    FToFree: TJSElement;
  protected
    procedure SetUp; override; 
    procedure TearDown; override;
    Procedure CreateParser(Const ASource : string);
    Procedure CheckClass(E : TJSElement; C : TJSElementClass);
    Procedure AssertEquals(Const AMessage : String; Expected, Actual : TJSType); overload;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure AssertIdentifier(Msg : String; El : TJSElement; Const AName : TJSString);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Function  GetSourceElements : TJSSourceElements;
    Function  GetVars : TJSElementNodes;
    Function  GetStatements : TJSElementNodes;
    Function  GetFunctions : TJSElementNodes;
    Function  GetFirstStatement : TJSElement;
    Function  GetFirstVar : TJSElement;
    Function  GetExpressionStatement : TJSExpressionStatement;
  published
    procedure TestEmpty;
    procedure TestSimple;
    procedure TestSimpleExpressionNumericalLiteral;
    procedure TestSimpleExpressionStringLiteral;
    procedure TestSimpleExpressionBooleanLiteralFalse;
    procedure TestSimpleExpressionBooleanLiteralTrue;
    procedure TestSimpleExpressionIdentifier;
    procedure TestSimpleExpressionNull;
    procedure TestAssignExpressionNumerical;
    procedure TestAssignExpressionNull;
    procedure TestAssignExpressionString;
    procedure TestAssignExpressionBooleanFalse;
    procedure TestAssignExpressionBooleanTrue;
    procedure TestAssignExpressionIdent;
    procedure TestAssignExpressionPlus;
    procedure TestAssignExpressionMinus;
    procedure TestAssignExpressionDiv;
    procedure TestAssignExpressionMul;
    procedure TestAssignExpressionMod;
    procedure TestAssignExpressionAnd;
    procedure TestAssignExpressionOr;
    procedure TestAssignExpressionXOr;
    procedure TestAssignExpressionLShift;
    procedure TestAssignExpressionRShift;
    procedure TestAssignExpressionURShift;
    procedure TestExpressionPlus;
    procedure TestExpressionSub;
    procedure TestExpressionMul;
    procedure TestExpressionDiv;
    procedure TestExpressionMod;
    procedure TestExpressionLShift;
    procedure TestExpressionRShift;
    procedure TestExpressionURShift;
    procedure TestExpressionPostPlusPlus;
    procedure TestExpressionPostMinusMinus;
    procedure TestExpressionPreMinusMinus;
    procedure TestExpressionPrePlusPlus;
    procedure TestExpressionPrecedenceMulPlus;
    procedure TestExpressionPrecedencePlusMul;
    procedure TestExpressionPrecedenceMulMinus;
    procedure TestExpressionPrecedenceMinusMul;
    procedure TestExpressionPrecedenceDivPlus;
    procedure TestExpressionPrecedencePlusDiv;
    procedure TestExpressionPrecedenceModPlus;
    procedure TestExpressionPrecedencePlusMod;
    procedure TestExpressionPrecedencePlusPostPlusPlus;
    procedure TestExpressionPrecedencePlusPostMinusMinus;
    procedure TestExpressionPrecedenceMulPostMinusMinus;
    procedure TestExpressionPrecedenceMulPostPlusPlus;
    procedure TestExpressionPrecedenceMulPreMinusMinus;
    procedure TestExpressionPrecedenceMulPrePlusPlus;
    procedure TestExpressionPrecedencePlusPreMinusMinus;
    procedure TestExpressionPrecedencePlusPrePlusPlus;
    procedure TestExpressionPrecedencePlusInv;
    procedure TestExpressionPrecedenceMulInv;
    procedure TestExpressionPrecedenceMulNot;
    procedure TestExpressionPrecedencePlusNot;
    procedure TestExpressionPrecedenceBraceMulPlus;
    procedure TestExpressionPrecedenceBracePlusMul;
    procedure TestExpressionFunction;
    procedure TestFunctionCallNoArgs;
    procedure TestFunctionCallOneArg;
    procedure TestFunctionCallTwoArgs;
    procedure TestArrayExpressionNumericalArgs;
    procedure TestArrayExpressionStringArgs;
    procedure TestArrayExpressionIdentArgs;
    Procedure TestVarDeclarationSimple;
    procedure TestVarDeclarationDouble;
    procedure TestVarDeclarationSimpleInit;
    procedure TestVarDeclarationDoubleInit;
    procedure TestBlockEmpty;
    procedure TestBlockEmptyStatement;
    procedure TestBlockSimpleStatement;
    procedure TestFunctionDeclarationEmpty;
    procedure TestFunctionDeclarationWithArgs;
    procedure TestFunctionDeclarationWithBody;
    procedure TestIfSimple;
    procedure TestIfElseSimple;
    procedure TestIfEmptyBlock;
    procedure TestIfEmptyBlockElse;
    procedure TestWhileSimple;
    procedure TestWhileBlock;
    procedure TestDoWhileSimple;
    procedure TestDoWhileBlock;
    procedure TestForEmpty;
    procedure TestForEmptyBody;
    procedure TestForSimpleBody;
    procedure TestTryCatch;
    procedure TestTryCatchFinally;
    procedure TestTryFinally;
    procedure TestThrow;
    procedure TestReturn;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure TestAssignment;
    procedure TestNew;
    procedure TestLabeledStatement;
    procedure TestContinue;
    procedure TestContinueTarget;
    procedure TestBreak;
    procedure TestBreakTarget;
    procedure TestSwitchEmpty;
    procedure TestSwitchOne;
    procedure TestSwitchTwo;
    procedure TestSwitchTwoDefault;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;

implementation

uses typinfo;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TTestJSParser.AssertEquals(Const AMessage: String; Expected,
=======
procedure TTestJSParser.AssertEquals(const AMessage: String; Expected,
>>>>>>> graemeg/cpstrnew
=======
procedure TTestJSParser.AssertEquals(const AMessage: String; Expected,
>>>>>>> graemeg/cpstrnew
=======
procedure TTestJSParser.AssertEquals(const AMessage: String; Expected,
>>>>>>> graemeg/cpstrnew
=======
procedure TTestJSParser.AssertEquals(const AMessage: String; Expected,
>>>>>>> origin/cpstrnew
  Actual: TJSType);

Var
  NE,NA : String;

begin
  NE:=GetEnumName(TypeInfo(TJSType),Ord(Expected));
  NA:=GetEnumName(TypeInfo(TJSType),Ord(Actual));
  AssertEquals(AMessage,NE,NA);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TTestJSParser.AssertIdentifier(Msg: String; El: TJSElement;
  Const AName: TJSString);

Var
  L : TJSPrimaryExpressionIdent;
  S1,S2 : String;
begin
  AssertNotNull(Msg+' have TJSPrimaryExpressionIdent element',El);
  CheckClass(El,TJSPrimaryExpressionIdent);
  L:=TJSPrimaryExpressionIdent(el);
  S1:=L.Name;
  S2:=Aname;
  AssertEquals(Msg+'Identifier has correct name',S2,S1);
end;

Function TTestJSParser.GetFirstStatement: TJSElement;
=======
function TTestJSParser.GetFirstStatement: TJSElement;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetFirstStatement: TJSElement;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetFirstStatement: TJSElement;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetFirstStatement: TJSElement;
>>>>>>> origin/cpstrnew

Var
  E : TJSElementNodes;
  N : TJSElement;
  X : TJSExpressionStatement;

begin
  E:=GetStatements;
  AssertNotNull('Have statements',E);
  AssertEquals('1 statement',1,E.Count);
  Result:=E.Nodes[0].Node;
  AssertNotNull('First statement assigned',Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TTestJSParser.GetFirstVar: TJSElement;
=======
function TTestJSParser.GetFirstVar: TJSElement;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetFirstVar: TJSElement;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetFirstVar: TJSElement;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetFirstVar: TJSElement;
>>>>>>> origin/cpstrnew
Var
  E : TJSElementNodes;
  N : TJSElement;
  X : TJSExpressionStatement;
begin
  E:=GetVars;
  AssertNotNull('Have statements',E);
  Writeln('Count : ',E.Count);
  If (E.Count=0) then
    Fail('Zero variables defined');
  Result:=E.Nodes[0].Node;
  AssertNotNull('First variable declaration',Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TTestJSParser.GetExpressionStatement: TJSExpressionStatement;
=======
function TTestJSParser.GetExpressionStatement: TJSExpressionStatement;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetExpressionStatement: TJSExpressionStatement;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetExpressionStatement: TJSExpressionStatement;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetExpressionStatement: TJSExpressionStatement;
>>>>>>> origin/cpstrnew

Var
  N : TJSElement;
  X : TJSExpressionStatement;

begin
  N:=GetFirstStatement;
  CheckClass(N,TJSExpressionStatement);
  Result:=TJSExpressionStatement(N);
end;


procedure TTestJSParser.TestSimple;

Var
  E : TJSElementNodes;
  N : TJSElement;
  X : TJSExpressionStatement;

begin
  CreateParser('1;');
  E:=GetStatements;
  AssertNotNull('Have statements',E);
  AssertEquals('1 statement',1,E.Count);
  N:=E.Nodes[0].Node;
  AssertNotNull('First statement assigned',N);
  AssertNotNull('First statement assigned',N);
  CheckClass(N,TJSExpressionStatement);
  X:=TJSExpressionStatement(N);
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSLiteral);
end;

procedure TTestJSParser.TestSimpleExpressionNumericalLiteral;
Var
  X : TJSExpressionStatement;

begin
  CreateParser('1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSLiteral);
  AssertNotNull('Expression value assigned',TJSLiteral(X.A).Value);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(X.A).Value.ValueType);
  AssertEquals('Expression value correct', 1.0,TJSLiteral(X.A).Value.AsNumber);
end;

procedure TTestJSParser.TestSimpleExpressionStringLiteral;

Var
  X : TJSExpressionStatement;

begin
  CreateParser('"string";');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSLiteral);
  AssertNotNull('Expression value assigned',TJSLiteral(X.A).Value);
  AssertEquals('Expression value type correct', jstString,TJSLiteral(X.A).Value.ValueType);
  AssertEquals('Expression value correct', 'string',TJSLiteral(X.A).Value.AsString);
end;

procedure TTestJSParser.TestSimpleExpressionBooleanLiteralFalse;

Var
  X : TJSExpressionStatement;

begin
  CreateParser('false;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSLiteral);
  AssertNotNull('Expression value assigned',TJSLiteral(X.A).Value);
  AssertEquals('Expression value type correct', jstBoolean,TJSLiteral(X.A).Value.ValueType);
  AssertEquals('Expression value correct', False, TJSLiteral(X.A).Value.AsBoolean);
end;

procedure TTestJSParser.TestSimpleExpressionIdentifier;

Var
  X : TJSExpressionStatement;

begin
  CreateParser('Something;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression value assigned','Something',TJSPrimaryExpressionIdent(X.A).Name);
=======
  AssertEquals('Expression value assigned','Something',TJSPrimaryExpressionIdent(X.A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression value assigned','Something',TJSPrimaryExpressionIdent(X.A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression value assigned','Something',TJSPrimaryExpressionIdent(X.A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression value assigned','Something',TJSPrimaryExpressionIdent(X.A).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestSimpleExpressionNull;

Var
  X : TJSExpressionStatement;
begin
    CreateParser('null;');
    X:=GetExpressionStatement;
    AssertNotNull('Expression statement assigned',X.A);
    CheckClass(X.A,TJSLiteral);
    AssertNotNull('Expression value assigned',TJSLiteral(X.A).Value);
    AssertEquals('Expression value type correct', jstNull,TJSLiteral(X.A).Value.ValueType);
    AssertEquals('Expression value correct', True, TJSLiteral(X.A).Value.IsNull);
end;

procedure TTestJSParser.TestAssignExpressionNumerical;

Var
  X : TJSExpressionStatement;
  SA : TJSSimpleAssignStatement;
begin
  CreateParser('a=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSSimpleAssignStatement);
  SA:=TJSSimpleAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionString;

Var
  X : TJSExpressionStatement;
  SA : TJSSimpleAssignStatement;
begin
  CreateParser('a="string";');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSSimpleAssignStatement);
  SA:=TJSSimpleAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstString,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 'string', TJSLiteral(SA.Expr).Value.AsString);
end;

procedure TTestJSParser.TestAssignExpressionBooleanFalse;

Var
  X : TJSExpressionStatement;
  SA : TJSSimpleAssignStatement;
begin
  CreateParser('a=false;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSSimpleAssignStatement);
  SA:=TJSSimpleAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstBoolean,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', False, TJSLiteral(SA.Expr).Value.AsBoolean);
end;

procedure TTestJSParser.TestAssignExpressionBooleanTrue;
Var
  X : TJSExpressionStatement;
  SA : TJSSimpleAssignStatement;
begin
  CreateParser('a=true;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSSimpleAssignStatement);
  SA:=TJSSimpleAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstBoolean,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', True, TJSLiteral(SA.Expr).Value.AsBoolean);
end;

procedure TTestJSParser.TestAssignExpressionNull;

Var
  X : TJSExpressionStatement;
  SA : TJSSimpleAssignStatement;
begin
  CreateParser('a=null;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSSimpleAssignStatement);
  SA:=TJSSimpleAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNull,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', True, TJSLiteral(SA.Expr).Value.IsNull);
end;

procedure TTestJSParser.TestAssignExpressionIdent;
Var
  X : TJSExpressionStatement;
  SA : TJSSimpleAssignStatement;
begin
  CreateParser('a=b;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSSimpleAssignStatement);
  SA:=TJSSimpleAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSPrimaryExpressionIdent);
  AssertEquals('Expression value type correct', 'b',TJSPrimaryExpressionIdent(SA.Expr).Name);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSPrimaryExpressionIdent);
  AssertEquals('Expression value type correct', 'b',TJSPrimaryExpressionIdent(SA.Expr).AString);
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
end;

procedure TTestJSParser.TestAssignExpressionPlus;

Var
  X : TJSExpressionStatement;
  SA : TJSAddEqAssignStatement;

begin
  CreateParser('a+=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAddEqAssignStatement);
  SA:=TJSAddEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionMinus;

Var
  X : TJSExpressionStatement;
  SA : TJSSubEqAssignStatement;

begin
  CreateParser('a-=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSSubEqAssignStatement);
  SA:=TJSSubEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionMul;

Var
  X : TJSExpressionStatement;
  SA : TJSMulEqAssignStatement;

begin
  CreateParser('a*=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMulEqAssignStatement);
  SA:=TJSMulEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionDiv;

Var
  X : TJSExpressionStatement;
  SA : TJSDivEqAssignStatement;

begin
  CreateParser('a/=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSDivEqAssignStatement);
  SA:=TJSDivEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionMod;

Var
  X : TJSExpressionStatement;
  SA : TJSModEqAssignStatement;

begin
  CreateParser('a%=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSModEqAssignStatement);
  SA:=TJSModEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionAnd;

Var
  X : TJSExpressionStatement;
  SA : TJSAndEqAssignStatement;

begin
  CreateParser('a&=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAndEqAssignStatement);
  SA:=TJSAndEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionOr;

Var
  X : TJSExpressionStatement;
  SA : TJSOrEqAssignStatement;

begin
  CreateParser('a|=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSOrEqAssignStatement);
  SA:=TJSOrEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionXOr;

Var
  X : TJSExpressionStatement;
  SA : TJSXOrEqAssignStatement;

begin
  CreateParser('a^=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSXOrEqAssignStatement);
  SA:=TJSXOrEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionLShift;

Var
  X : TJSExpressionStatement;
  SA : TJSLShiftEqAssignStatement;

begin
  CreateParser('a<<=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSLShiftEqAssignStatement);
  SA:=TJSLShiftEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionRShift;

Var
  X : TJSExpressionStatement;
  SA : TJSRShiftEqAssignStatement;

begin
  CreateParser('a>>=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSRShiftEqAssignStatement);
  SA:=TJSRShiftEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestAssignExpressionURShift;

Var
  X : TJSExpressionStatement;
  SA : TJSURShiftEqAssignStatement;

begin
  CreateParser('a>>>=1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSURShiftEqAssignStatement);
  SA:=TJSURShiftEqAssignStatement(X.A);
  AssertNotNull('Assignment LHS assigned',SA.LHS);
  CheckClass(SA.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).Name);
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Expression LHS name correct', 'a',TJSPrimaryExpressionIdent(SA.LHS).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Assignment Expression assigned',SA.Expr);
  CheckClass(SA.EXPR,TJSLiteral);
  AssertEquals('Expression value type correct', jstNumber,TJSLiteral(SA.Expr).Value.ValueType);
  AssertEquals('Expression value correct', 1.0, TJSLiteral(SA.Expr).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPlus;

Var
  X : TJSExpressionStatement;
  E : TJSAdditiveExpressionPlus;

begin
  CreateParser('1+2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSLiteral);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Expression left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression left operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
  AssertNotNull('Expression right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Expression right operand value correct', 2.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionSub;
Var
  X : TJSExpressionStatement;
  E : TJSAdditiveExpressionMinus;

begin
  CreateParser('1 - 2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionMinus);
  E:=TJSAdditiveExpressionMinus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSLiteral);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Expression left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression left operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
  AssertNotNull('Expression right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Expression right operand value correct', 2.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionMul;

Var
  X : TJSExpressionStatement;
  E : TJSMultiplicativeExpressionMul;

begin
  CreateParser('1*2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionMul);
  E:=TJSMultiplicativeExpressionMul(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSLiteral);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Expression left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression left operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
  AssertNotNull('Expression right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Expression right operand value correct', 2.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionDiv;
Var
  X : TJSExpressionStatement;
  E : TJSMultiplicativeExpressionDiv;

begin
  CreateParser('1/2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionDiv);
  E:=TJSMultiplicativeExpressionDiv(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSLiteral);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Expression left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression left operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
  AssertNotNull('Expression right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Expression right operand value correct', 2.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionMod;
Var
  X : TJSExpressionStatement;
  E : TJSMultiplicativeExpressionMod;

begin
  CreateParser('1%2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionMod);
  E:=TJSMultiplicativeExpressionMod(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSLiteral);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Expression left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression left operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
  AssertNotNull('Expression right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Expression right operand value correct', 2.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionLShift;
Var
  X : TJSExpressionStatement;
  E : TJSLShiftExpression;

begin
  CreateParser('1 << 2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSLShiftExpression);
  E:=TJSLShiftExpression(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSLiteral);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Expression left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression left operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
  AssertNotNull('Expression right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Expression right operand value correct', 2.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionRShift;
Var
  X : TJSExpressionStatement;
  E : TJSRShiftExpression;

begin
  CreateParser('1 >> 2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSRShiftExpression);
  E:=TJSRShiftExpression(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSLiteral);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Expression left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression left operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
  AssertNotNull('Expression right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Expression right operand value correct', 2.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionURShift;
Var
  X : TJSExpressionStatement;
  E : TJSURShiftExpression;

begin
  CreateParser('1 >>> 2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSURShiftExpression);
  E:=TJSURShiftExpression(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSLiteral);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Expression left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression left operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
  AssertNotNull('Expression right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Expression left operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Expression right operand value correct', 2.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPostPlusPlus;
Var
  X : TJSExpressionStatement;
  E : TJSUnaryPostPlusPlusExpression;

begin
  CreateParser('1++;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSUnaryPostPlusPlusExpression);
  E:=TJSUnaryPostPlusPlusExpression(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Expression  operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPostMinusMinus;
Var
  X : TJSExpressionStatement;
  E : TJSUnaryPostMinusMinusExpression;

begin
  CreateParser('1--;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSUnaryPostMinusMinusExpression);
  E:=TJSUnaryPostMinusMinusExpression(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Expression  operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrePlusPlus;

Var
  X : TJSExpressionStatement;
  E : TJSUnaryPrePlusPlusExpression;

begin
  CreateParser('++1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSUnaryPrePlusPlusExpression);
  E:=TJSUnaryPrePlusPlusExpression(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Expression  operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
end;


procedure TTestJSParser.TestExpressionPreMinusMinus;

Var
  X : TJSExpressionStatement;
  E : TJSUnaryPreMinusMinusExpression;

begin
  CreateParser('--1;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSUnaryPreMinusMinusExpression);
  E:=TJSUnaryPreMinusMinusExpression(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Expression  operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Expression operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression operand value correct', 1.0, TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceMulPlus;

Var
  X : TJSExpressionStatement;
  E : TJSAdditiveExpressionPlus;
  R : TJSMultiplicativeExpressionMul;
begin
  CreateParser('2 * 3 + 4;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSMultiplicativeExpressionMul);
  R:=TJSMultiplicativeExpressionMul(E.A);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  AssertNotNull('Multiplication right operand assigned',R.B);
  CheckClass(R.A,TJSLiteral);
  CheckClass(R.B,TJSLiteral);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertEquals('Multiplication right operand type correct', jstNumber, TJSLiteral(R.B).Value.ValueType);
  AssertEquals('Multiplication right operand value correct', 3.0, TJSLiteral(R.B).Value.AsNumber);
  AssertNotNull('Addition right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Expression right operand value correct', 4.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceBraceMulPlus;

Var
  X : TJSExpressionStatement;
  E : TJSMultiplicativeExpressionMul;
  R : TJSAdditiveExpressionPlus;

begin
  CreateParser('2 * (3 + 4);');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionMul);
  E:=TJSMultiplicativeExpressionMul(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSAdditiveExpressionPlus);
  R:=TJSAdditiveExpressionPlus(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  AssertNotNull('Multiplication right operand assigned',R.B);
  CheckClass(R.A,TJSLiteral);
  CheckClass(R.B,TJSLiteral);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 2.0, TJSLiteral(E.A).Value.AsNumber);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Multiplication right operand value correct', 3.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Addition right operand value assigned',TJSLiteral(R.B).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(R.B).Value.ValueType);
  AssertEquals('Addition right operand value correct', 4.0,TJSLiteral(R.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceBracePlusMul;

Var
  X : TJSExpressionStatement;
  E : TJSMultiplicativeExpressionMul;
  R : TJSAdditiveExpressionPlus;

begin
  CreateParser('(3 + 4)*2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionMul);
  E:=TJSMultiplicativeExpressionMul(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSAdditiveExpressionPlus);
  R:=TJSAdditiveExpressionPlus(E.A);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  AssertNotNull('Multiplication right operand assigned',R.B);
  CheckClass(R.A,TJSLiteral);
  CheckClass(R.B,TJSLiteral);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 2.0, TJSLiteral(E.B).Value.AsNumber);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Multiplication right operand value correct', 3.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Addition right operand value assigned',TJSLiteral(R.B).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(R.B).Value.ValueType);
  AssertEquals('Addition right operand value correct', 4.0,TJSLiteral(R.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionFunction;
Var
  X : TJSExpressionStatement;
  A  : TJSSimpleAssignStatement;
begin
  CreateParser('a = function () {};');
  X:=GetExpressionStatement;
  CheckClass(X.A,TJSSimpleAssignStatement);
  A:=TJSSimpleAssignStatement(X.A);
  AssertNotNull('Have left operand',A.LHS);
  CheckClass(A.LHS,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Correct name for assignment LHS ','a',TJSPrimaryExpressionIdent(A.LHS).Name);
=======
  AssertEquals('Correct name for assignment LHS ','a',TJSPrimaryExpressionIdent(A.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Correct name for assignment LHS ','a',TJSPrimaryExpressionIdent(A.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Correct name for assignment LHS ','a',TJSPrimaryExpressionIdent(A.LHS).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Correct name for assignment LHS ','a',TJSPrimaryExpressionIdent(A.LHS).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestExpressionPrecedencePlusMul;

Var
  X : TJSExpressionStatement;
  E : TJSAdditiveExpressionPlus;
  R : TJSMultiplicativeExpressionMul;
begin
  CreateParser('4 + 2 * 3;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSMultiplicativeExpressionMul);
  R:=TJSMultiplicativeExpressionMul(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  AssertNotNull('Multiplication right operand assigned',R.B);
  CheckClass(R.A,TJSLiteral);
  CheckClass(R.B,TJSLiteral);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertEquals('Multiplication right operand type correct', jstNumber, TJSLiteral(R.B).Value.ValueType);
  AssertEquals('Multiplication right operand value correct', 3.0, TJSLiteral(R.B).Value.AsNumber);
  AssertNotNull('Addition left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Addition left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceMulMinus;

Var
  X : TJSExpressionStatement;
  E : TJSAdditiveExpressionMinus;
  R : TJSMultiplicativeExpressionMul;

begin
  CreateParser('2 * 3 - 4;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionMinus);
  E:=TJSAdditiveExpressionMinus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSMultiplicativeExpressionMul);
  R:=TJSMultiplicativeExpressionMul(E.A);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  AssertNotNull('Multiplication right operand assigned',R.B);
  CheckClass(R.A,TJSLiteral);
  CheckClass(R.B,TJSLiteral);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertEquals('Multiplication right operand type correct', jstNumber, TJSLiteral(R.B).Value.ValueType);
  AssertEquals('Multiplication right operand value correct', 3.0, TJSLiteral(R.B).Value.AsNumber);
  AssertNotNull('subtraction right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('subtraction right operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('subtraction right operand value correct', 4.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceMinusMul;

Var
  X : TJSExpressionStatement;
  E : TJSAdditiveExpressionMinus;
  R : TJSMultiplicativeExpressionMul;
begin
  CreateParser('4 - 2 * 3;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionMinus);
  E:=TJSAdditiveExpressionMinus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSMultiplicativeExpressionMul);
  R:=TJSMultiplicativeExpressionMul(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  AssertNotNull('Multiplication right operand assigned',R.B);
  CheckClass(R.A,TJSLiteral);
  CheckClass(R.B,TJSLiteral);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertEquals('Multiplication right operand type correct', jstNumber, TJSLiteral(R.B).Value.ValueType);
  AssertEquals('Multiplication right operand value correct', 3.0, TJSLiteral(R.B).Value.AsNumber);
  AssertNotNull('Subtraction left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Subtraction left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Subtraction left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceDivPlus;

Var
  X : TJSExpressionStatement;
  E : TJSAdditiveExpressionPlus;
  R : TJSMultiplicativeExpressionDiv;
begin
  CreateParser('2 / 3 + 4;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSMultiplicativeExpressionDiv);
  R:=TJSMultiplicativeExpressionDiv(E.A);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('Div left operand assigned',R.A);
  AssertNotNull('Div right operand assigned',R.B);
  CheckClass(R.A,TJSLiteral);
  CheckClass(R.B,TJSLiteral);
  AssertEquals('Div left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Div left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertEquals('Div right operand type correct', jstNumber, TJSLiteral(R.B).Value.ValueType);
  AssertEquals('Div right operand value correct', 3.0, TJSLiteral(R.B).Value.AsNumber);
  AssertNotNull('Addition right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Addition right operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Addition right operand value correct', 4.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedencePlusDiv;

Var
  X : TJSExpressionStatement;
  E : TJSAdditiveExpressionPlus;
  R : TJSMultiplicativeExpressionDiv;

begin
  CreateParser('4 + 2 / 3;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSMultiplicativeExpressionDiv);
  R:=TJSMultiplicativeExpressionDiv(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Div left operand assigned',R.A);
  AssertNotNull('Div right operand assigned',R.B);
  CheckClass(R.A,TJSLiteral);
  CheckClass(R.B,TJSLiteral);
  AssertEquals('Div left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Div left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertEquals('Div right operand type correct', jstNumber, TJSLiteral(R.B).Value.ValueType);
  AssertEquals('Div right operand value correct', 3.0, TJSLiteral(R.B).Value.AsNumber);
  AssertNotNull('Addition left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Addition left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceModPlus;

Var
  X : TJSExpressionStatement;
  E : TJSAdditiveExpressionPlus;
  R : TJSMultiplicativeExpressionMod;
begin
  CreateParser('2 % 3 + 4;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.A,TJSMultiplicativeExpressionMod);
  R:=TJSMultiplicativeExpressionMod(E.A);
  CheckClass(E.B,TJSLiteral);
  AssertNotNull('mod left operand assigned',R.A);
  AssertNotNull('mod right operand assigned',R.B);
  CheckClass(R.A,TJSLiteral);
  CheckClass(R.B,TJSLiteral);
  AssertEquals('mod left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('mod left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertEquals('mod right operand type correct', jstNumber, TJSLiteral(R.B).Value.ValueType);
  AssertEquals('mod right operand value correct', 3.0, TJSLiteral(R.B).Value.AsNumber);
  AssertNotNull('Addition right operand value assigned',TJSLiteral(E.B).Value);
  AssertEquals('Addition right operand type correct', jstNumber, TJSLiteral(E.B).Value.ValueType);
  AssertEquals('Addition right operand value correct', 4.0,TJSLiteral(E.B).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedencePlusMod;

Var
  X : TJSExpressionStatement;
  E : TJSAdditiveExpressionPlus;
  R : TJSMultiplicativeExpressionMod;

begin
  CreateParser('4 + 2 % 3;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSMultiplicativeExpressionMod);
  R:=TJSMultiplicativeExpressionMod(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Mod left operand assigned',R.A);
  AssertNotNull('Mod right operand assigned',R.B);
  CheckClass(R.A,TJSLiteral);
  CheckClass(R.B,TJSLiteral);
  AssertEquals('Mod left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Mod left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertEquals('Mod right operand type correct', jstNumber, TJSLiteral(R.B).Value.ValueType);
  AssertEquals('Mod right operand value correct', 3.0, TJSLiteral(R.B).Value.AsNumber);
  AssertNotNull('Addition right operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Addition left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedencePlusPostPlusPlus;

Var
  X : TJSExpressionStatement;
  R : TJSUnaryPostPlusPlusExpression;
  E : TJSAdditiveExpressionPlus;

begin
  CreateParser('4 + 2++;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSUnaryPostPlusPlusExpression);
  R:=TJSUnaryPostPlusPlusExpression(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('++ operand assigned',R.A);
  CheckClass(R.A,TJSLiteral);
  AssertEquals('++ operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('++ operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Addition left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Addition left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedencePlusPostMinusMinus;
Var
  X : TJSExpressionStatement;
  R : TJSUnaryPostMinusMinusExpression;
  E : TJSAdditiveExpressionPlus;

begin
  CreateParser('4 + 2--;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSUnaryPostMinusMinusExpression);
  R:=TJSUnaryPostMinusMinusExpression(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('-- operand assigned',R.A);
  CheckClass(R.A,TJSLiteral);
  AssertEquals('-- operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('-- operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Addition left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Addition left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceMulPostPlusPlus;

Var
  X : TJSExpressionStatement;
  R : TJSUnaryPostPlusPlusExpression;
  E : TJSMultiplicativeExpressionMul;

begin
  CreateParser('4 * 2++;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionMul);
  E:=TJSMultiplicativeExpressionMul(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSUnaryPostPlusPlusExpression);
  R:=TJSUnaryPostPlusPlusExpression(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('++operand assigned',R.A);
  CheckClass(R.A,TJSLiteral);
  AssertEquals('++ operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('++ operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Multiplication left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceMulPostMinusMinus;
Var
  X : TJSExpressionStatement;
  R : TJSUnaryPostMinusMinusExpression;
  E : TJSMultiplicativeExpressionMul;

begin
  CreateParser('4 * 2--;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionMul);
  E:=TJSMultiplicativeExpressionMul(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSUnaryPostMinusMinusExpression);
  R:=TJSUnaryPostMinusMinusExpression(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('-- operand assigned',R.A);
  CheckClass(R.A,TJSLiteral);
  AssertEquals('-- operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('-- operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Multiplication left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedencePlusPrePlusPlus;

Var
  X : TJSExpressionStatement;
  R : TJSUnaryPrePlusPlusExpression;
  E : TJSAdditiveExpressionPlus;

begin
  CreateParser('4 + ++2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSUnaryPrePlusPlusExpression);
  R:=TJSUnaryPrePlusPlusExpression(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  CheckClass(R.A,TJSLiteral);
  AssertEquals('++ operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('++ operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Addition left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Addition left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedencePlusInv;

Var
  X : TJSExpressionStatement;
  R : TJSUnaryInvExpression;
  E : TJSAdditiveExpressionPlus;

begin
    CreateParser('4 + ~2;');
    X:=GetExpressionStatement;
    AssertNotNull('Expression statement assigned',X.A);
    CheckClass(X.A,TJSAdditiveExpressionPlus);
    E:=TJSAdditiveExpressionPlus(X.A);
    AssertNotNull('Expression left operand assigned',E.A);
    AssertNotNull('Expression right operand assigned',E.B);
    CheckClass(E.B,TJSUnaryInvExpression);
    R:=TJSUnaryInvExpression(E.B);
    CheckClass(E.A,TJSLiteral);
    AssertNotNull('Multiplication left operand assigned',R.A);
    CheckClass(R.A,TJSLiteral);
    AssertEquals('inv operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
    AssertEquals('inv operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
    AssertNotNull('Addition left operand value assigned',TJSLiteral(E.A).Value);
    AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
    AssertEquals('Addition left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceMulInv;

Var
  X : TJSExpressionStatement;
  R : TJSUnaryInvExpression;
  E : TJSMultiplicativeExpressionMul;

begin
  CreateParser('4 * ~2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionMul);
  E:=TJSMultiplicativeExpressionMul(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSUnaryInvExpression);
  R:=TJSUnaryInvExpression(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  CheckClass(R.A,TJSLiteral);
  AssertEquals('Inv operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Inv operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Multiplication left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedencePlusNot;

Var
  X : TJSExpressionStatement;
  R : TJSUnaryNotExpression;
  E : TJSAdditiveExpressionPlus;

begin
    CreateParser('4 + !2;');
    X:=GetExpressionStatement;
    AssertNotNull('Expression statement assigned',X.A);
    CheckClass(X.A,TJSAdditiveExpressionPlus);
    E:=TJSAdditiveExpressionPlus(X.A);
    AssertNotNull('Expression left operand assigned',E.A);
    AssertNotNull('Expression right operand assigned',E.B);
    CheckClass(E.B,TJSUnaryNotExpression);
    R:=TJSUnaryNotExpression(E.B);
    CheckClass(E.A,TJSLiteral);
    AssertNotNull('Multiplication left operand assigned',R.A);
    CheckClass(R.A,TJSLiteral);
    AssertEquals('Not operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
    AssertEquals('Not operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
    AssertNotNull('Addition left operand value assigned',TJSLiteral(E.A).Value);
    AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
    AssertEquals('Addition left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestFunctionCallNoArgs;

Var
  X : TJSExpressionStatement;
  C : TJSCallExpression;

begin
  CreateParser('abc();');
  X:=GetExpressionStatement;
  CheckClass(X.A,TJSCallExpression);
  C:=TJSCallExpression(X.A);
  AssertEquals('No arguments',0,C.Args.Elements.Count);
  AssertNotNull('Call function expression',C.Expr);
  CheckClass(C.Expr,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).Name);
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestFunctionCallOneArg;

Var
  X : TJSExpressionStatement;
  C : TJSCallExpression;
  E : TJSelement;

begin
  CreateParser('abc(d);');
  X:=GetExpressionStatement;
  CheckClass(X.A,TJSCallExpression);
  C:=TJSCallExpression(X.A);
  AssertNotNull('Call function expression',C.Expr);
  CheckClass(C.Expr,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).Name);
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> origin/cpstrnew
  AssertEquals('1 argument',1,C.Args.Elements.Count);
  E:=C.Args.Elements[0].Expr;
  AssertNotNull('First argument expression',E);
  CheckClass(E,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('First argument name correct','d',TJSPrimaryExpressionIdent(E).Name);
=======
  AssertEquals('First argument name correct','d',TJSPrimaryExpressionIdent(E).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('First argument name correct','d',TJSPrimaryExpressionIdent(E).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('First argument name correct','d',TJSPrimaryExpressionIdent(E).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('First argument name correct','d',TJSPrimaryExpressionIdent(E).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestFunctionCallTwoArgs;

Var
  X : TJSExpressionStatement;
  C : TJSCallExpression;
  E : TJSelement;

begin
  CreateParser('abc(d,e);');
  X:=GetExpressionStatement;
  CheckClass(X.A,TJSCallExpression);
  C:=TJSCallExpression(X.A);
  AssertNotNull('Call function expression',C.Expr);
  CheckClass(C.Expr,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).Name);
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Function name correct','abc',TJSPrimaryExpressionIdent(C.Expr).AString);
>>>>>>> origin/cpstrnew
  AssertEquals('2 arguments',2,C.Args.Elements.Count);
  E:=C.Args.Elements[0].Expr;
  AssertNotNull('First argument expression',E);
  CheckClass(E,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('First argument name correct','d',TJSPrimaryExpressionIdent(E).Name);
  E:=C.Args.Elements[1].Expr;
  AssertNotNull('Second argument expression',E);
  CheckClass(E,TJSPrimaryExpressionIdent);
  AssertEquals('Second argument name correct','e',TJSPrimaryExpressionIdent(E).Name);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AssertEquals('First argument name correct','d',TJSPrimaryExpressionIdent(E).AString);
  E:=C.Args.Elements[1].Expr;
  AssertNotNull('Second argument expression',E);
  CheckClass(E,TJSPrimaryExpressionIdent);
  AssertEquals('Second argument name correct','e',TJSPrimaryExpressionIdent(E).AString);
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
end;

procedure TTestJSParser.TestArrayExpressionNumericalArgs;
Var
  X : TJSExpressionStatement;
  B : TJSBracketMemberExpression;

begin
  CreateParser('A[1];');
  X:=GetExpressionStatement;
  CheckClass(X.A,TJSBracketMemberExpression);
  B:=TJSBracketMemberExpression(X.A);
  CheckClass(B.Name,TJSLiteral);
  AssertEquals('Member name operand type correct', jstNumber, TJSLiteral(B.Name).Value.ValueType);
  AssertEquals('Member name operand value correct', 1.0, TJSLiteral(B.Name).Value.AsNumber);
  CheckClass(B.Mexpr,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).Name);
=======
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestArrayExpressionStringArgs;
Var
  X : TJSExpressionStatement;
  B : TJSBracketMemberExpression;

begin
  CreateParser('A["propname"];');
  X:=GetExpressionStatement;
  CheckClass(X.A,TJSBracketMemberExpression);
  B:=TJSBracketMemberExpression(X.A);
  CheckClass(B.Name,TJSLiteral);
  AssertEquals('Member name operand type correct', jstString, TJSLiteral(B.Name).Value.ValueType);
  AssertEquals('Member name operand value correct', 'propname', TJSLiteral(B.Name).Value.AsString);
  CheckClass(B.Mexpr,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).Name);
=======
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestArrayExpressionIdentArgs;

Var
  X : TJSExpressionStatement;
  B : TJSBracketMemberExpression;

begin
  CreateParser('A[B];');
  X:=GetExpressionStatement;
  CheckClass(X.A,TJSBracketMemberExpression);
  B:=TJSBracketMemberExpression(X.A);
  CheckClass(B.Name,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Member name identifier correct', 'B', TJSPrimaryExpressionIdent(B.Name).Name);
  CheckClass(B.Mexpr,TJSPrimaryExpressionIdent);
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).Name);
end;

Procedure TTestJSParser.TestVarDeclarationSimple;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AssertEquals('Member name identifier correct', 'B', TJSPrimaryExpressionIdent(B.Name).AString);
  CheckClass(B.Mexpr,TJSPrimaryExpressionIdent);
  AssertEquals('Array name correct','A',TJSPrimaryExpressionIdent(B.Mexpr).AString);
end;

procedure TTestJSParser.TestVarDeclarationSimple;
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

Var
  X : TJSELement;
  V : TJSVarDeclaration;
begin
  CreateParser('var a;');
  X:=GetFirstVar;
  CheckClass(X,TJSVarDeclaration);
  V:=TJSVarDeclaration(X);
//  AssertNotNull('Variable statement assigned',(X));
  AssertEquals('variable name correct registered', 'a', V.Name);
  AssertNull('No initialization expression', V.Init);
end;

procedure TTestJSParser.TestVarDeclarationDouble;

Var
  X : TJSELement;
  V : TJSVarDeclaration;

begin
  CreateParser('var a, b ;');
  AssertEquals('2 variables declared',2,GetVars.Count);
  X:=GetFirstVar;
  CheckClass(X,TJSVarDeclaration);
  V:=TJSVarDeclaration(X);
//  AssertNotNull('Variable statement assigned',(X));
  AssertEquals('variable name correct registered', 'a', V.name);
  X:=GetVars.Nodes[1].Node;
  CheckClass(X,TJSVarDeclaration);
  V:=TJSVarDeclaration(X);
  AssertEquals('variable name correct registered', 'b', V.Name);
  AssertNull('No initialization expression', V.Init);
end;

procedure TTestJSParser.TestVarDeclarationSimpleInit;

Var
  X : TJSELement;
  V : TJSVarDeclaration;
begin
  CreateParser('var a = b;');
  X:=GetFirstVar;
  CheckClass(X,TJSVarDeclaration);
  V:=TJSVarDeclaration(X);
//  AssertNotNull('Variable statement assigned',(X));
  AssertEquals('variable name correct registered', 'a', V.Name);
  AssertNotNull('Initialization expression present', V.Init);
  CheckClass(V.Init,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Member name identifier correct', 'b', TJSPrimaryExpressionIdent(V.init).Name);
=======
  AssertEquals('Member name identifier correct', 'b', TJSPrimaryExpressionIdent(V.init).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Member name identifier correct', 'b', TJSPrimaryExpressionIdent(V.init).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Member name identifier correct', 'b', TJSPrimaryExpressionIdent(V.init).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Member name identifier correct', 'b', TJSPrimaryExpressionIdent(V.init).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestVarDeclarationDoubleInit;

Var
  X : TJSELement;
  V : TJSVarDeclaration;
begin
  CreateParser('var a, c = b;');
  AssertEquals('2 variables declared',2,GetVars.Count);
  X:=GetFirstVar;
  CheckClass(X,TJSVarDeclaration);
  V:=TJSVarDeclaration(X);
//  AssertNotNull('Variable statement assigned',(X));
  AssertEquals('variable name correct registered', 'a', V.Name);
  AssertNull('No initialization expression', V.Init);
  X:=GetVars.Nodes[1].Node;
  CheckClass(X,TJSVarDeclaration);
  V:=TJSVarDeclaration(X);
  AssertEquals('variable name correct registered', 'c', V.Name);
  AssertNotNull('No initialization expression', V.Init);
  CheckClass(V.Init,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Member name identifier correct', 'b', TJSPrimaryExpressionIdent(V.init).Name);
=======
  AssertEquals('Member name identifier correct', 'b', TJSPrimaryExpressionIdent(V.init).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Member name identifier correct', 'b', TJSPrimaryExpressionIdent(V.init).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Member name identifier correct', 'b', TJSPrimaryExpressionIdent(V.init).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Member name identifier correct', 'b', TJSPrimaryExpressionIdent(V.init).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestBlockEmpty;

Var
  E : TJSSourceElements;
  X : TJSElement;

begin
  CreateParser('{}');
  E:=GetSourceElements;
  AssertEquals('1 statement in block',1,E.Statements.Count);
  X:=E.Statements.Nodes[0].Node;
  CheckClass(X,TJSEmptyBlockStatement);
end;

procedure TTestJSParser.TestBlockEmptyStatement;

Var
  E : TJSSourceElements;
  X : TJSElement;

begin
  CreateParser('{;}');
  E:=GetSourceElements;
  AssertEquals('1 statement in block',1,E.Statements.Count);
  X:=E.Statements.Nodes[0].Node;
  CheckClass(X,TJSEmptyStatement);
end;

procedure TTestJSParser.TestBlockSimpleStatement;

Var
  E : TJSSourceElements;
  X : TJSElement;

begin
  CreateParser('{a;}');
  E:=GetSourceElements;
  AssertEquals('1 statement in block',1,E.Statements.Count);
  X:=E.Statements.Nodes[0].Node;
  CheckClass(X,TJSExpressionStatement);
  CheckNotNull(TJSExpressionStatement(X).A);
  CheckClass(TJSExpressionStatement(X).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(X).A).Name)
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(X).A).AString)
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(X).A).AString)
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(X).A).AString)
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(X).A).AString)
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestFunctionDeclarationEmpty;

Var
  E : TJSSourceElements;
  N : TJSElement;
  FD : TJSFunctionDeclarationStatement;

begin
  CreateParser('function a () {}');
  E:=GetSourceElements;
  AssertEquals('1 function defined',1,E.functions.Count);
  N:=E.Functions.Nodes[0].Node;
  AssertNotNull('Function element defined ',N);
  CheckClass(N,TJSFunctionDeclarationStatement);
  FD:=TJSFunctionDeclarationStatement(N);
  AssertNotNull('Function definition assigned',FD.AFunction);
  AssertEquals('Function name OK','a',FD.AFunction.Name);
  AssertNotNull('Function body assigned', FD.AFunction.Body);
  AssertEquals('No parameters',0,FD.AFunction.Params.Count);
  N:=FD.AFunction.Body;
  CheckClass(N,TJSFunctionBody);
  AssertNotNull('Function body has element',TJSFunctionBody(N).A);
  CheckClass(TJSFunctionBody(N).A,  TJSSourceElements);
  E:=TJSSourceElements(TJSFunctionBody(N).A);
  AssertEquals('0 statement in functionbody elements',0,E.Statements.Count);
//  TJSEmptyBlockStatement
end;

procedure TTestJSParser.TestFunctionDeclarationWithArgs;

Var
  E : TJSSourceElements;
  N : TJSElement;
  FD : TJSFunctionDeclarationStatement;

begin
  CreateParser('function a (b,c) {}');
  E:=GetSourceElements;
  AssertEquals('1 function defined',1,E.functions.Count);
  N:=E.Functions.Nodes[0].Node;
  AssertNotNull('Function element defined ',N);
  CheckClass(N,TJSFunctionDeclarationStatement);
  FD:=TJSFunctionDeclarationStatement(N);
  AssertNotNull('Function definition assigned',FD.AFunction);
  AssertEquals('Function name OK','a',FD.AFunction.Name);
  AssertNotNull('Function body assigned', FD.AFunction.Body);
  AssertEquals('2 parameters',2,FD.AFunction.Params.Count);
  AssertEquals('1st parameter','b',FD.AFunction.Params[0]);
  AssertEquals('2nd parameter','c',FD.AFunction.Params[1]);
  N:=FD.AFunction.Body;
  CheckClass(N,TJSFunctionBody);
  AssertNotNull('Function body has element',TJSFunctionBody(N).A);
  CheckClass(TJSFunctionBody(N).A,  TJSSourceElements);
  E:=TJSSourceElements(TJSFunctionBody(N).A);
  AssertEquals('0 statement in functionbody elements',0,E.Statements.Count);
//  TJSEmptyBlockStatement
end;

procedure TTestJSParser.TestFunctionDeclarationWithBody;

Var
  E : TJSSourceElements;
  N : TJSElement;
  FD : TJSFunctionDeclarationStatement;

begin
  CreateParser('function a () { b; }');
  E:=GetSourceElements;
  AssertEquals('1 function defined',1,E.functions.Count);
  N:=E.Functions.Nodes[0].Node;
  AssertNotNull('Function element defined ',N);
  CheckClass(N,TJSFunctionDeclarationStatement);
  FD:=TJSFunctionDeclarationStatement(N);
  AssertNotNull('Function definition assigned',FD.AFunction);
  AssertEquals('Function name OK','a',FD.AFunction.Name);
  AssertNotNull('Function body assigned', FD.AFunction.Body);
  AssertEquals('2 parameters',0,FD.AFunction.Params.Count);
  N:=FD.AFunction.Body;
  CheckClass(N,TJSFunctionBody);
  AssertNotNull('Function body has element',TJSFunctionBody(N).A);
  CheckClass(TJSFunctionBody(N).A,  TJSSourceElements);
  E:=TJSSourceElements(TJSFunctionBody(N).A);
  AssertEquals('1 statement in functionbody elements',1,E.Statements.Count);
  N:=E.Statements.Nodes[0].Node;
  CheckClass(N,TJSExpressionStatement);
  CheckNotNull(TJSExpressionStatement(N).A);
  CheckClass(TJSExpressionStatement(N).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(N).A).Name);
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(N).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(N).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(N).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(N).A).AString);
>>>>>>> origin/cpstrnew
//  TJSEmptyBlockStatement
end;

procedure TTestJSParser.TestIfSimple;

Var
  E : TJSElement;
  I : TJSIfStatement;

begin
  CreateParser('if (a) b;');
  E:=GetFirstStatement;
  CheckClass(E,TJSIfStatement);
  I:=TJSIfStatement(E);
  AssertNotNull('Statement condition assigned',I.Cond);
  CheckClass(I.Cond,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> origin/cpstrnew
  AssertNull('Statement false branch assigned',I.BFalse);
  AssertNotNull('Statement true branch assigned',I.Btrue);
  CheckClass(I.Btrue,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(I.BTrue).A);
  CheckClass(TJSExpressionStatement(I.BTrue).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.Btrue).A).Name);
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.Btrue).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.Btrue).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.Btrue).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.Btrue).A).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestIfEmptyBlock;

Var
  E : TJSElement;
  I : TJSIfStatement;

begin
  CreateParser('if (a) {}');
  E:=GetFirstStatement;
  CheckClass(E,TJSIfStatement);
  I:=TJSIfStatement(E);
  AssertNotNull('Statement condition assigned',I.Cond);
  CheckClass(I.Cond,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> origin/cpstrnew
  AssertNull('Statement false branch assigned',I.BFalse);
  AssertNotNull('Statement true branch assigned',I.Btrue);
  CheckClass(I.Btrue,TJSEmptyBlockStatement);
end;

procedure TTestJSParser.TestIfEmptyBlockElse;

Var
  E : TJSElement;
  I : TJSIfStatement;

begin
  CreateParser('if (a) {} else b;');
  E:=GetFirstStatement;
  CheckClass(E,TJSIfStatement);
  I:=TJSIfStatement(E);
  AssertNotNull('Statement condition assigned',I.Cond);
  CheckClass(I.Cond,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Statement false branch assigned',I.BFalse);
  AssertNotNull('Statement true branch assigned',I.Btrue);
  CheckClass(I.Btrue,TJSEmptyBlockStatement);
end;

procedure TTestJSParser.TestWhileSimple;
Var
  E : TJSElement;
  W : TJSWhileStatement;

begin
  CreateParser('while (a) b;');
  E:=GetFirstStatement;
  CheckClass(E,TJSWhileStatement);
  W:=TJSWhileStatement(E);
  AssertNotNull('Statement condition assigned',W.Cond);
  CheckClass(W.Cond,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Statement condition assigned',W.body);
  CheckClass(W.Body,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(W.Body).A);
  CheckClass(TJSExpressionStatement(W.Body).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).Name);
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestWhileBlock;

Var
  E : TJSElement;
  W : TJSWhileStatement;
//  B : TJSBlockStatement;

begin
  CreateParser('while (a) {b;}');
  E:=GetFirstStatement;
  CheckClass(E,TJSWhileStatement);
  W:=TJSWhileStatement(E);
  AssertNotNull('Statement condition assigned',W.Cond);
  CheckClass(W.Cond,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Statement condition assigned',W.body);
  CheckClass(W.Body,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(W.Body).A);
  CheckClass(TJSExpressionStatement(W.Body).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).Name);
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestDoWhileSimple;

Var
  E : TJSElement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  W : TJSDoWhileStatement;
=======
  W : TJSWhileStatement;
>>>>>>> graemeg/cpstrnew
=======
  W : TJSWhileStatement;
>>>>>>> graemeg/cpstrnew
=======
  W : TJSWhileStatement;
>>>>>>> graemeg/cpstrnew
=======
  W : TJSWhileStatement;
>>>>>>> origin/cpstrnew
//  B : TJSBlockStatement;

begin
  CreateParser('do b; while (a);');
  E:=GetFirstStatement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  CheckClass(E,TJSDoWhileStatement);
  W:=TJSDoWhileStatement(E);
  AssertNotNull('Statement condition assigned',W.Cond);
  CheckClass(W.Cond,TJSPrimaryExpressionIdent);
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).Name);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  CheckClass(E,TJSWhileStatement);
  W:=TJSWhileStatement(E);
  AssertNotNull('Statement condition assigned',W.Cond);
  CheckClass(W.Cond,TJSPrimaryExpressionIdent);
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).AString);
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
  AssertNotNull('Statement condition assigned',W.body);
  CheckClass(W.Body,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(W.Body).A);
  CheckClass(TJSExpressionStatement(W.Body).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).Name);
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestDoWhileBlock;

Var
  E : TJSElement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  W : TJSDoWhileStatement;
=======
  W : TJSWhileStatement;
>>>>>>> graemeg/cpstrnew
=======
  W : TJSWhileStatement;
>>>>>>> graemeg/cpstrnew
=======
  W : TJSWhileStatement;
>>>>>>> graemeg/cpstrnew
=======
  W : TJSWhileStatement;
>>>>>>> origin/cpstrnew
//  B : TJSBlockStatement;

begin
  CreateParser('do {b;} while (a);');
  E:=GetFirstStatement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  CheckClass(E,TJSDoWhileStatement);
  W:=TJSDoWhileStatement(E);
  AssertNotNull('Statement condition assigned',W.Cond);
  CheckClass(W.Cond,TJSPrimaryExpressionIdent);
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).Name);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  CheckClass(E,TJSWhileStatement);
  W:=TJSWhileStatement(E);
  AssertNotNull('Statement condition assigned',W.Cond);
  CheckClass(W.Cond,TJSPrimaryExpressionIdent);
  AssertEquals('a',TJSPrimaryExpressionIdent(W.Cond).AString);
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
  AssertNotNull('Statement condition assigned',W.body);
  CheckClass(W.Body,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(W.Body).A);
  CheckClass(TJSExpressionStatement(W.Body).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).Name);
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(W.Body).A).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestForEmpty;

Var
  E : TJSElement;
  F : TJSForStatement;

begin
  CreateParser('for (;;) a;');
  E:=GetFirstStatement;
  CheckClass(E,TJSForStatement);
  F:=TJSForStatement(E);
  AssertNull('Statement condition not assigned',F.Cond);
  AssertNull('Statement init not assigned',F.Init);
  AssertNull('Statement step not assigned',F.Incr);
  CheckClass(F.Body,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(F.Body).A);
  CheckClass(TJSExpressionStatement(F.Body).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestForEmptyBody;

Var
  E : TJSElement;
  F : TJSForStatement;

begin
  CreateParser('for (;;) {a;}');
  E:=GetFirstStatement;
  CheckClass(E,TJSForStatement);
  F:=TJSForStatement(E);
  AssertNull('Statement condition not assigned',F.Cond);
  AssertNull('Statement init not assigned',F.Init);
  AssertNull('Statement step not assigned',F.Incr);
  CheckClass(F.Body,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(F.Body).A);
  CheckClass(TJSExpressionStatement(F.Body).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestForSimpleBody;

Var
  E : TJSElement;
  F : TJSForStatement;

begin
  CreateParser('for (a;b;c) {d;}');
  E:=GetFirstStatement;
  CheckClass(E,TJSForStatement);
  F:=TJSForStatement(E);
  AssertNotNull('Statement condition not assigned',F.Cond);
  AssertNotNull('Statement init not assigned',F.Init);
  AssertNotNull('Statement step not assigned',F.Incr);
  CheckClass(F.Init,TJSPrimaryExpressionIdent);
  AssertNotNull('Expression statement expression',TJSPrimaryExpressionIdent(F.Init));
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(F.Init).Name);
  CheckClass(F.Incr,TJSPrimaryExpressionIdent);
  AssertNotNull('Expression statement expression',TJSPrimaryExpressionIdent(F.Incr));
  AssertEquals('c',TJSPrimaryExpressionIdent(F.Incr).Name);
  CheckClass(F.Cond,TJSPrimaryExpressionIdent);
  AssertNotNull('Expression statement expression',TJSPrimaryExpressionIdent(F.Cond));
  AssertEquals('b',TJSPrimaryExpressionIdent(F.cond).Name);
  CheckClass(F.Body,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(F.Body).A);
  CheckClass(TJSExpressionStatement(F.Body).A,TJSPrimaryExpressionIdent);
  AssertEquals('d',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).Name);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AssertEquals('a',TJSPrimaryExpressionIdent(F.Init).AString);
  CheckClass(F.Incr,TJSPrimaryExpressionIdent);
  AssertNotNull('Expression statement expression',TJSPrimaryExpressionIdent(F.Incr));
  AssertEquals('c',TJSPrimaryExpressionIdent(F.Incr).AString);
  CheckClass(F.Cond,TJSPrimaryExpressionIdent);
  AssertNotNull('Expression statement expression',TJSPrimaryExpressionIdent(F.Cond));
  AssertEquals('b',TJSPrimaryExpressionIdent(F.cond).AString);
  CheckClass(F.Body,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(F.Body).A);
  CheckClass(TJSExpressionStatement(F.Body).A,TJSPrimaryExpressionIdent);
  AssertEquals('d',TJSPrimaryExpressionIdent(TJSExpressionStatement(F.Body).A).AString);
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
end;

procedure TTestJSParser.TestTryCatch;

Var
  E : TJSElement;
  T : TJSTryCatchStatement;

begin
  CreateParser('try {a;} catch (e) {b;}');
  E:=GetFirstStatement;
  CheckClass(E,TJSTryCatchStatement);
  T:=TJSTryCatchStatement(E);
  CheckClass(T.Block,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(T.Block).A);
  CheckClass(TJSExpressionStatement(T.Block).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> origin/cpstrnew
  CheckClass(T.BCatch,TJSExpressionStatement);
  AssertEquals('Except object identifier name','e',T.Ident);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(T.BCatch).A);
  CheckClass(TJSExpressionStatement(T.BCatch).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BCatch).A).Name);
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BCatch).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BCatch).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BCatch).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BCatch).A).AString);
>>>>>>> origin/cpstrnew
  AssertNull('No Finally expression',T.BFinally);
end;

procedure TTestJSParser.TestTryCatchFinally;

Var
  E : TJSElement;
  T : TJSTryCatchFinallyStatement;

begin
  CreateParser('try {a;} catch (e) {b;} finally {c;}');
  E:=GetFirstStatement;
  CheckClass(E,TJSTryCatchFinallyStatement);
  T:=TJSTryCatchFinallyStatement(E);
  CheckClass(T.Block,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(T.Block).A);
  CheckClass(TJSExpressionStatement(T.Block).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> origin/cpstrnew
  AssertEquals('Except object identifier name','e',T.Ident);
  CheckClass(T.BCatch,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(T.BCatch).A);
  CheckClass(TJSExpressionStatement(T.BCatch).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BCatch).A).Name);
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BCatch).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BCatch).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BCatch).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BCatch).A).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Finally expression',T.BFinally);
  CheckClass(T.BFinally,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(T.BFinally).A);
  CheckClass(TJSExpressionStatement(T.BFinally).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BFinally).A).Name);
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BFinally).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BFinally).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BFinally).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BFinally).A).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestTryFinally;

Var
  E : TJSElement;
  T : TJSTryFinallyStatement;

begin
  CreateParser('try {a;} finally {c;}');
  E:=GetFirstStatement;
  CheckClass(E,TJSTryFinallyStatement);
  T:=TJSTryFinallyStatement(E);
  CheckClass(T.Block,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(T.Block).A);
  CheckClass(TJSExpressionStatement(T.Block).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.Block).A).AString);
>>>>>>> origin/cpstrnew
  AssertNull('No catch',T.BCatch);
  AssertNotNull('Finally expression',T.BFinally);
  AssertNotNull('Finally expression',TJSExpressionStatement(T.BFinally).A);
  CheckClass(TJSExpressionStatement(T.BFinally).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BFinally).A).Name);
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BFinally).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BFinally).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BFinally).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(T.BFinally).A).AString);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestThrow;
Var
  E : TJSElement;
  T : TJSThrowStatement;

begin
  CreateParser('throw a;');
  E:=GetFirstStatement;
  CheckClass(E,TJSThrowStatement);
  T:=TJSThrowStatement(E);
  AssertNotNull('Have throw object',T.A);
  CheckClass(T.A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('Correct identifier','a',TJSPrimaryExpressionIdent(T.A).Name);
=======
  AssertEquals('Correct identifier','a',TJSPrimaryExpressionIdent(T.A).AsTring);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Correct identifier','a',TJSPrimaryExpressionIdent(T.A).AsTring);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Correct identifier','a',TJSPrimaryExpressionIdent(T.A).AsTring);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('Correct identifier','a',TJSPrimaryExpressionIdent(T.A).AsTring);
>>>>>>> origin/cpstrnew
end;

procedure TTestJSParser.TestReturn;

Var
  E : TJSSourceElements;
  N : TJSElement;
  FD : TJSFunctionDeclarationStatement;

begin
  CreateParser('function a () { return b; }');
  E:=GetSourceElements;
  AssertEquals('1 function defined',1,E.functions.Count);
  N:=E.Functions.Nodes[0].Node;
  AssertNotNull('Function element defined ',N);
  CheckClass(N,TJSFunctionDeclarationStatement);
  FD:=TJSFunctionDeclarationStatement(N);
  AssertNotNull('Function definition assigned',FD.AFunction);
  AssertEquals('Function name OK','a',FD.AFunction.Name);
  AssertNotNull('Function body assigned', FD.AFunction.Body);
  AssertEquals('No parameters',0,FD.AFunction.Params.Count);
  N:=FD.AFunction.Body;
  CheckClass(N,TJSFunctionBody);
  AssertNotNull('Function body has element',TJSFunctionBody(N).A);
  CheckClass(TJSFunctionBody(N).A,  TJSSourceElements);
  E:=TJSSourceElements(TJSFunctionBody(N).A);
  AssertEquals('1 statement in functionbody elements',1,E.Statements.Count);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TTestJSParser.TestAssignment;
Var
  E : TJSElement;
  ES : TJSExpressionStatement;
  A : TJSSimpleAssignStatement;

begin
  CreateParser('a=b;');
  E:=GetFirstStatement;
  CheckClass(E,TJSExpressionStatement);
  ES:=TJSExpressionStatement(E);
  CheckClass(ES.A,TJSSimpleAssignStatement);
  A:=TJSSimpleAssignStatement(ES.A);
  AssertNotNull('Have LHS',A.LHS);
  CheckClass(A.LHS,TJSPrimaryExpressionIdent);
  AssertEquals('Have LHS name','a',TJSPrimaryExpressionIdent(A.LHS).Name);
  CheckClass(A.Expr,TJSPrimaryExpressionIdent);
  AssertEquals('Have RHS name','b',TJSPrimaryExpressionIdent(A.Expr).Name);
end;

procedure TTestJSParser.TestNew;
Var
  E : TJSElement;
  ES : TJSExpressionStatement;
  A : TJSSimpleAssignStatement;
  N : TJSNewMemberExpression;
  L : TJSLiteral;

begin
  CreateParser('a = new b(123)');
  E:=GetFirstStatement;
  CheckClass(E,TJSExpressionStatement);
  ES:=TJSExpressionStatement(E);
  CheckClass(ES.A,TJSSimpleAssignStatement);
  A:=TJSSimpleAssignStatement(ES.A);
  CheckClass(A.LHS,TJSPrimaryExpressionIdent);
  AssertEquals('Have LHS name','a',TJSPrimaryExpressionIdent(A.LHS).Name);
  CheckClass(A.Expr,TJSNewMemberExpression);
  N:=TJSNewMemberExpression(A.Expr);
  AssertNotNull('Have LHS name',N.Mexpr);
  CheckClass(N.Mexpr,TJSPrimaryExpressionIdent);
  AssertEquals('Have LHS name','b',TJSPrimaryExpressionIdent(N.Mexpr).Name);
  AssertNotNull('Have arguments',N.Args);
  AssertEquals('One argument',1,N.Args.Elements.Count);
  AssertNotNull('Have argument 0',N.Args.Elements[0].Expr);
  CheckClass(N.Args.Elements[0].Expr,TJSLiteral);
  L:=TJSLiteral(N.Args.Elements[0].Expr);
  AssertNotNull('Expression value assigned',L.Value);
  AssertEquals('Expression value type correct', jstNumber,L.Value.ValueType);
  AssertEquals('Expression value correct', 123,L.Value.AsNumber);

end;

procedure TTestJSParser.TestLabeledStatement;
Var
  E : TJSElement;
  ES : TJSExpressionStatement;
  A : TJSSimpleAssignStatement;
  N : TJSNewMemberExpression;
  L : TJSLiteral;
  LS : TJSLabeledStatement;

begin
  CreateParser('loc: a = new b(123)');
  E:=GetFirstStatement;
  CheckClass(E,TJSLabeledStatement);
  LS:=TJSLabeledStatement(E);
  AssertNotNull('Have label',LS.TheLabel);
  AssertEquals('Have correct label','loc',LS.TheLabel.Name);
  CheckClass(LS.A,TJSExpressionStatement);
  ES:=TJSExpressionStatement(LS.A);
  CheckClass(ES.A,TJSSimpleAssignStatement);
  A:=TJSSimpleAssignStatement(ES.A);
  CheckClass(A.LHS,TJSPrimaryExpressionIdent);
  AssertEquals('Have LHS name','a',TJSPrimaryExpressionIdent(A.LHS).Name);
  CheckClass(A.Expr,TJSNewMemberExpression);
  N:=TJSNewMemberExpression(A.Expr);
  AssertNotNull('Have LHS name',N.Mexpr);
  CheckClass(N.Mexpr,TJSPrimaryExpressionIdent);
  AssertEquals('Have LHS name','b',TJSPrimaryExpressionIdent(N.Mexpr).Name);
  AssertNotNull('Have arguments',N.Args);
  AssertEquals('One argument',1,N.Args.Elements.Count);
  AssertNotNull('Have argument 0',N.Args.Elements[0].Expr);
  CheckClass(N.Args.Elements[0].Expr,TJSLiteral);
  L:=TJSLiteral(N.Args.Elements[0].Expr);
  AssertNotNull('Expression value assigned',L.Value);
  AssertEquals('Expression value type correct', jstNumber,L.Value.ValueType);
  AssertEquals('Expression value correct', 123,L.Value.AsNumber);
end;

procedure TTestJSParser.TestContinue;

Var
  E : TJSElement;
  C : TJSContinueStatement;

begin
  CreateParser('while (true) continue;');
  E:=GetFirstStatement;
  CheckClass(E,TJSWhileStatement);
  E:=TJSWhileStatement(E).Body;
  CheckClass(E,TJSContinueStatement);
  C:=TJSContinueStatement(E);
  AssertEquals('Have correct (empty) label','',C.TargetName);
end;

procedure TTestJSParser.TestContinueTarget;

Var
  E : TJSElement;
  C : TJSContinueStatement;

begin
  CreateParser('a: while (true) continue a;');
  E:=GetFirstStatement;
  CheckClass(E,TJSLabeledStatement);
  E:=TJSLabeledStatement(E).A;
  CheckClass(E,TJSWhileStatement);
  E:=TJSWhileStatement(E).Body;
  CheckClass(E,TJSContinueStatement);
  C:=TJSContinueStatement(E);
  AssertEquals('Have correct  label','a',C.TargetName);
end;

procedure TTestJSParser.TestBreakTarget;
Var
  E : TJSElement;
  C : TJSBreakStatement;

begin
  CreateParser('a: while (true) break a;');
  E:=GetFirstStatement;
  CheckClass(E,TJSLabeledStatement);
  E:=TJSLabeledStatement(E).A;
  CheckClass(E,TJSWhileStatement);
  E:=TJSWhileStatement(E).Body;
  CheckClass(E,TJSBreakStatement);
  C:=TJSBreakStatement(E);
  AssertEquals('Have correct  label','a',C.TargetName);
end;

procedure TTestJSParser.TestSwitchEmpty;
Var
  E : TJSElement;
  S : TJSSwitchStatement;
  P : TJSPrimaryExpressionIdent;

begin
  CreateParser('switch (a) {}');
  E:=GetFirstStatement;
  CheckClass(E,TJSSwitchStatement);
  S:=TJSSwitchStatement(E);
  AssertNotNull('Have condition',S.Cond);
  AssertNull('Have no default',S.TheDefault);
  AssertIdentifier('Case condition',S.Cond,'a');
  S:=TJSSwitchStatement(E);
  AssertEquals('No cases',0,S.Cases.Count)
end;

procedure TTestJSParser.TestSwitchOne;
Var
  E : TJSElement;
  S : TJSSwitchStatement;
  P : TJSPrimaryExpressionIdent;
  C : TJSCaseElement;
begin
  CreateParser('switch (a) { case c : {}}');
  E:=GetFirstStatement;
  CheckClass(E,TJSSwitchStatement);
  S:=TJSSwitchStatement(E);
  AssertNotNull('Have condition',S.Cond);
  AssertNull('Have no default',S.TheDefault);
  AssertIdentifier('Case condition',S.Cond,'a');
  S:=TJSSwitchStatement(E);
  AssertEquals('1 case',1,S.Cases.Count);
  C:=TJSCaseElement(S.Cases[0]);
  AssertIdentifier('Case expression',C.Expr,'c');
  CheckClass(C.Body,TJSEmptyBlockStatement);
end;

procedure TTestJSParser.TestSwitchTwo;
Var
  E : TJSElement;
  S : TJSSwitchStatement;
  P : TJSPrimaryExpressionIdent;
  C : TJSCaseElement;
begin
  CreateParser('switch (a) { case c: {}'+sLineBreak+' case d: {}}');
  E:=GetFirstStatement;
  CheckClass(E,TJSSwitchStatement);
  S:=TJSSwitchStatement(E);
  AssertNotNull('Have condition',S.Cond);
  AssertNull('Have no default',S.TheDefault);
  AssertIdentifier('Case condition',S.Cond,'a');
  S:=TJSSwitchStatement(E);
  AssertEquals('2 cases',2,S.Cases.Count);
  C:=TJSCaseElement(S.Cases[0]);
  AssertIdentifier('Case expression',C.Expr,'c');
  CheckClass(C.Body,TJSEmptyBlockStatement);
  C:=TJSCaseElement(S.Cases[1]);
  AssertIdentifier('Case expression',C.Expr,'d');
  CheckClass(C.Body,TJSEmptyBlockStatement);
end;

procedure TTestJSParser.TestSwitchTwoDefault;
Var
  E : TJSElement;
  S : TJSSwitchStatement;
  P : TJSPrimaryExpressionIdent;
  C : TJSCaseElement;
begin
  CreateParser('switch (a) { case c: {} case d: {} default: {}}');
  E:=GetFirstStatement;
  CheckClass(E,TJSSwitchStatement);
  S:=TJSSwitchStatement(E);
  AssertNotNull('Have condition',S.Cond);
  AssertNotNull('Have default',S.TheDefault);
  AssertIdentifier('Case condition',S.Cond,'a');
  S:=TJSSwitchStatement(E);
  AssertEquals('2 cases',3,S.Cases.Count);
  C:=TJSCaseElement(S.Cases[0]);
  AssertIdentifier('Case expression',C.Expr,'c');
  CheckClass(C.Body,TJSEmptyBlockStatement);
  C:=TJSCaseElement(S.Cases[1]);
  AssertIdentifier('Case expression',C.Expr,'d');
  CheckClass(C.Body,TJSEmptyBlockStatement);
  C:=TJSCaseElement(S.Cases[2]);
  CheckClass(C.Body,TJSEmptyBlockStatement);
  AssertSame('Default',C,S.TheDefault);
end;

procedure TTestJSParser.TestBreak;
Var
  E : TJSElement;
  C : TJSBreakStatement;

begin
  CreateParser('while (true) break;');
  E:=GetFirstStatement;
  CheckClass(E,TJSWhileStatement);
  E:=TJSWhileStatement(E).Body;
  CheckClass(E,TJSBreakStatement);
  C:=TJSBreakStatement(E);
  AssertEquals('Have correct (empty) label','',C.TargetName);
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TTestJSParser.TestIfElseSimple;

Var
  E : TJSElement;
  I : TJSIfStatement;

begin
  CreateParser('if (a) b; else c;');
  E:=GetFirstStatement;
  CheckClass(E,TJSIfStatement);
  I:=TJSIfStatement(E);
  AssertNotNull('Statement condition assigned',I.Cond);
  CheckClass(I.Cond,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).Name);
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('a',TJSPrimaryExpressionIdent(I.Cond).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Statement condition assigned',I.Btrue);
  CheckClass(I.Btrue,TJSExpressionStatement);
  AssertNotNull('Expression statement expression',TJSExpressionStatement(I.BTrue).A);
  CheckClass(TJSExpressionStatement(I.BTrue).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.Btrue).A).Name);
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.Btrue).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.Btrue).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.Btrue).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('b',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.Btrue).A).AString);
>>>>>>> origin/cpstrnew
  AssertNotNull('Else Statement condition assigned',I.BFalse);
  CheckClass(I.BFalse,TJSExpressionStatement);
  AssertNotNull('Else statement expression',TJSExpressionStatement(I.BFalse).A);
  CheckClass(TJSExpressionStatement(I.BFalse).A,TJSPrimaryExpressionIdent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.BFalse).A).Name);
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.BFalse).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.BFalse).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.BFalse).A).AString);
>>>>>>> graemeg/cpstrnew
=======
  AssertEquals('c',TJSPrimaryExpressionIdent(TJSExpressionStatement(I.BFalse).A).AString);
>>>>>>> origin/cpstrnew
end;


procedure TTestJSParser.TestExpressionPrecedenceMulNot;

Var
  X : TJSExpressionStatement;
  R : TJSUnaryNotExpression;
  E : TJSMultiplicativeExpressionMul;

begin
  CreateParser('4 * !2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionMul);
  E:=TJSMultiplicativeExpressionMul(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSUnaryNotExpression);
  R:=TJSUnaryNotExpression(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  CheckClass(R.A,TJSLiteral);
  AssertEquals('Not operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Not operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Multiplication left operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedencePlusPreMinusMinus;
Var
  X : TJSExpressionStatement;
  R : TJSUnaryPreMinusMinusExpression;
  E : TJSAdditiveExpressionPlus;

begin
  CreateParser('4 + --2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSAdditiveExpressionPlus);
  E:=TJSAdditiveExpressionPlus(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSUnaryPreMinusMinusExpression);
  R:=TJSUnaryPreMinusMinusExpression(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  CheckClass(R.A,TJSLiteral);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Addition right operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression right operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceMulPrePlusPlus;

Var
  X : TJSExpressionStatement;
  R : TJSUnaryPrePlusPlusExpression;
  E : TJSMultiplicativeExpressionMul;

begin
  CreateParser('4 * ++2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionMul);
  E:=TJSMultiplicativeExpressionMul(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSUnaryPrePlusPlusExpression);
  R:=TJSUnaryPrePlusPlusExpression(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  CheckClass(R.A,TJSLiteral);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Addition right operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression right operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestExpressionPrecedenceMulPreMinusMinus;
Var
  X : TJSExpressionStatement;
  R : TJSUnaryPreMinusMinusExpression;
  E : TJSMultiplicativeExpressionMul;

begin
  CreateParser('4 * --2;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSMultiplicativeExpressionMul);
  E:=TJSMultiplicativeExpressionMul(X.A);
  AssertNotNull('Expression left operand assigned',E.A);
  AssertNotNull('Expression right operand assigned',E.B);
  CheckClass(E.B,TJSUnaryPreMinusMinusExpression);
  R:=TJSUnaryPreMinusMinusExpression(E.B);
  CheckClass(E.A,TJSLiteral);
  AssertNotNull('Multiplication left operand assigned',R.A);
  CheckClass(R.A,TJSLiteral);
  AssertEquals('Multiplication left operand type correct', jstNumber, TJSLiteral(R.A).Value.ValueType);
  AssertEquals('Multiplication left operand value correct', 2.0, TJSLiteral(R.A).Value.AsNumber);
  AssertNotNull('Addition right operand value assigned',TJSLiteral(E.A).Value);
  AssertEquals('Addition left operand type correct', jstNumber, TJSLiteral(E.A).Value.ValueType);
  AssertEquals('Expression right operand value correct', 4.0,TJSLiteral(E.A).Value.AsNumber);
end;

procedure TTestJSParser.TestSimpleExpressionBooleanLiteralTrue;

Var
  X : TJSExpressionStatement;

begin
  CreateParser('true;');
  X:=GetExpressionStatement;
  AssertNotNull('Expression statement assigned',X.A);
  CheckClass(X.A,TJSLiteral);
  AssertNotNull('Expression value assigned',TJSLiteral(X.A).Value);
  AssertEquals('Expression value type correct', jstBoolean,TJSLiteral(X.A).Value.ValueType);
  AssertEquals('Expression value correct', True, TJSLiteral(X.A).Value.AsBoolean);
end;


procedure TTestJSParser.TestEmpty;

Var
  E : TJSElement;
  FB : TJSFunctionBody;
  SE : TJSSourceElements;

begin
  CreateParser('var a;');
  E:=FParser.Parse;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  try
    CheckClass(E,TJSFunctionBody);
    FB:=TJSFunctionBody(E);
    AssertNotNull(FB.A);
    CheckClass(FB.A,TJSSourceElements);
    SE:=TJSSourceElements(FB.A);
    AssertEquals('1 variable declaration ',1,SE.Vars.Count);
    CheckClass(FB.A,TJSSourceElements);
  finally
    E.Free;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  CheckClass(E,TJSFunctionBody);
  FB:=TJSFunctionBody(E);
  AssertNotNull(FB.A);
  CheckClass(FB.A,TJSSourceElements);
  SE:=TJSSourceElements(FB.A);
  AssertEquals('1 variable declaration ',1,SE.Vars.Count);
  CheckClass(FB.A,TJSSourceElements);
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
end;

procedure TTestJSParser.SetUp; 
begin
  FParser:=Nil;
  FSource:=Nil;
end; 

procedure TTestJSParser.TearDown; 
begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FreeAndNil(FToFree);
=======
//  FreeAndNil(FToFree);
>>>>>>> graemeg/cpstrnew
=======
//  FreeAndNil(FToFree);
>>>>>>> graemeg/cpstrnew
=======
//  FreeAndNil(FToFree);
>>>>>>> graemeg/cpstrnew
=======
//  FreeAndNil(FToFree);
>>>>>>> origin/cpstrnew
  FreeAndNil(FParser);
  FReeAndNil(FSource);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TTestJSParser.CreateParser(Const ASource: string);
=======
procedure TTestJSParser.CreateParser(const ASource: string);
>>>>>>> graemeg/cpstrnew
=======
procedure TTestJSParser.CreateParser(const ASource: string);
>>>>>>> graemeg/cpstrnew
=======
procedure TTestJSParser.CreateParser(const ASource: string);
>>>>>>> graemeg/cpstrnew
=======
procedure TTestJSParser.CreateParser(const ASource: string);
>>>>>>> origin/cpstrnew
begin
  FSource:=TStringStream.Create(ASource);
  FParser:=TJSParser.Create(FSource);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Procedure TTestJSParser.CheckClass(E: TJSElement; C: TJSElementClass);
=======
procedure TTestJSParser.CheckClass(E: TJSElement; C: TJSElementClass);
>>>>>>> graemeg/cpstrnew
=======
procedure TTestJSParser.CheckClass(E: TJSElement; C: TJSElementClass);
>>>>>>> graemeg/cpstrnew
=======
procedure TTestJSParser.CheckClass(E: TJSElement; C: TJSElementClass);
>>>>>>> graemeg/cpstrnew
=======
procedure TTestJSParser.CheckClass(E: TJSElement; C: TJSElementClass);
>>>>>>> origin/cpstrnew
begin
  AssertEquals(C,E.ClassType);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TTestJSParser.GetSourceElements: TJSSourceElements;
=======
function TTestJSParser.GetSourceElements: TJSSourceElements;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetSourceElements: TJSSourceElements;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetSourceElements: TJSSourceElements;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetSourceElements: TJSSourceElements;
>>>>>>> origin/cpstrnew

Var
  E : TJSElement;
  FB : TJSFunctionBody;

begin
  If Not Assigned(FSE) then
    begin
    AssertNotNull('Parser assigned',FParser);
    E:=FParser.Parse;
    CheckClass(E,TJSFunctionBody);
    FB:=TJSFunctionBody(E);
    AssertNotNull(FB.A);
    CheckClass(FB.A,TJSSourceElements);
    FSE:=TJSSourceElements(FB.A);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FToFree:=E;
    end;
  Result:=FSE;
end;

Function TTestJSParser.GetVars: TJSElementNodes;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end;
  Result:=FSE;
  FToFree:=E;
end;

function TTestJSParser.GetVars: TJSElementNodes;
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
begin
  Result:=GetSourceElements.Vars;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TTestJSParser.GetStatements: TJSElementNodes;
=======
function TTestJSParser.GetStatements: TJSElementNodes;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetStatements: TJSElementNodes;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetStatements: TJSElementNodes;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetStatements: TJSElementNodes;
>>>>>>> origin/cpstrnew
begin
  Result:=GetSourceElements.Statements;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function TTestJSParser.GetFunctions: TJSElementNodes;
=======
function TTestJSParser.GetFunctions: TJSElementNodes;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetFunctions: TJSElementNodes;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetFunctions: TJSElementNodes;
>>>>>>> graemeg/cpstrnew
=======
function TTestJSParser.GetFunctions: TJSElementNodes;
>>>>>>> origin/cpstrnew
begin
  Result:=GetSourceElements.Functions;
end;


initialization

  RegisterTest(TTestJSParser); 
end.

