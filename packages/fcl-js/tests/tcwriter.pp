unit tcwriter;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, jsbase, jstree, jswriter;

type

  { TTestJSWriter }

  TTestJSWriter = class(TTestCase)
  private
    FElement: TJSElement;
    FTextWriter: TBufferWriter;
    FWriter: TJSWriter;
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    Procedure WriteElement(JS : TJSElement); // Set element in Element, write. Freed on teardown
    Procedure AssertResult(Const Msg, Result : String); // Compare result;
    Procedure AssertResult(Const Msg : string; Result : UnicodeString); // Compare result;
    Procedure AssertWrite(Const Msg, Result : String; AElement : TJSElement); // Call writelement, compare result;
    Procedure AssertWrite(Const Msg : string; Result : UnicodeString; AElement : TJSElement); // Call writelement, compare result;
    Function CreateIdent(Const AName : String) : TJSPrimaryExpressionIdent;
    Function CreateLiteral(Const AValue : TJSString) : TJSLiteral;
    Function CreateLiteral(Const AValue : Integer) : TJSLiteral;
    Function CreateLiteral(Const AValue : Boolean) : TJSLiteral;
    Property TextWriter : TBufferWriter Read FTextWriter;
    Property Writer : TJSWriter Read FWriter;
    Property Element : TJSElement read FElement;
  end;

  TTestTestJSWriter = Class(TTestJSWriter)
  published
    procedure TestEmpty;
  end;

  { TTestLiteralWriter }

  TTestLiteralWriter= class(TTestJSWriter)
  published
    Procedure TestInteger;
    Procedure TestBooleanTrue;
    Procedure TestBooleanFalse;
    Procedure TestUndefined;
    Procedure TestNull;
    Procedure TestString;
    Procedure TestStringQuote;
    Procedure TestStringBackslash;
    Procedure TestStringslash;
    Procedure TestStringsBack;
    Procedure TestStringsTab;
    Procedure TestStringsLineFeed;
    Procedure TestStringsFormFeed;
    Procedure TestStringsCarriageReturn;
    Procedure TestArrayEmpty;
    Procedure TestArrayEmptyCompact;
    Procedure TestArrayOneElement;
    Procedure TestArrayOneElementCompact;
    Procedure TestArrayOneElementIndent;
    Procedure TestArrayTwoElements;
    Procedure TestArrayTwoElementsCompact;
    Procedure TestArrayTwoElementsCompact2;
    Procedure TestArrayThreeElementsCompact;
    Procedure TestObjectEmpty;
    Procedure TestObjectEmptyCompact;
    Procedure TestObjectOneElement;
    Procedure TestObjectOneElementCompact;
    Procedure TestObjectOneElementIndent;
    Procedure TestObjectOneElementCompactQuoted;
    Procedure TestObjectTwoElements;
    Procedure TestObjectTwoElementCompact;
    Procedure TestObjectTwoElementCompact2;
    Procedure TestObjectTwoElementCompactQuoted;
    Procedure TestObjectThreeElementsCompact;
  end;

  { TTestStatementWriter }

  TTestStatementWriter = class(TTestJSWriter)
  Public
    Procedure TestAssignment(Const Msg : String; AClass : TJSAssignStatementClass; Result : String;ACompact : Boolean);
    Function CreateAssignment(AClass : TJSAssignStatementClass) : TJSAssignStatement;
    Function CreateStatementListOneElement : TJSStatementList;
    Function CreateStatementListTwoElement2 : TJSStatementList;
  published
    Procedure TestEmptyStatement;
    Procedure TestEmptyStatementComment;
    Procedure TestEmptyStatementBlock;
    Procedure TestEmptyStatementBlockIndent;
    Procedure TestEmptyStatementBlockCompact;
    Procedure TestVarDeclaration;
    Procedure TestVarDeclarationInit;
    Procedure TestVarListDeclaration;
    Procedure TestVarListDeclarationInit;
    Procedure TestVarDeclarationStatement;
    Procedure TestVarListDeclarationStatement;
    Procedure TestVarListDeclarationStatement2Vars;
    Procedure TestVarListDeclarationStatement3Vars;
    Procedure TestReturnStatement;
    Procedure TestLabeledStatement;
    Procedure TestLabeledStatementCompact;
    Procedure TestContinueStatement;
    Procedure TestContinueTargetStatement;
    Procedure TestBreakStatement;
    Procedure TestBreakTargetStatement;
    Procedure TestAssignmentStatementSimple;
    Procedure TestAssignmentStatementSimpleCompact;
    Procedure TestAssignmentStatementAdd;
    Procedure TestAssignmentStatementAddCompact;
    Procedure TestAssignmentStatementSubtract;
    Procedure TestAssignmentStatementSubtractCompact;
    Procedure TestAssignmentStatementMultiply;
    Procedure TestAssignmentStatementMultiplyCompact;
    Procedure TestAssignmentStatementDivide;
    Procedure TestAssignmentStatementDivideCompact;
    Procedure TestAssignmentStatementShift;
    Procedure TestAssignmentStatementShiftCompact;
    Procedure TestAssignmentStatementRShift;
    Procedure TestAssignmentStatementRShiftCompact;
    Procedure TestAssignmentStatementURShift;
    Procedure TestAssignmentStatementURShiftCompact;
    Procedure TestAssignmentStatementMod;
    Procedure TestAssignmentStatementModCompact;
    Procedure TestAssignmentStatementBinaryOr;
    Procedure TestAssignmentStatementBinaryOrCompact;
    Procedure TestAssignmentStatementBinaryXOr;
    Procedure TestAssignmentStatementBinaryXOrCompact;
    Procedure TestAssignmentStatementBinaryAnd;
    Procedure TestAssignmentStatementBinaryAndCompact;
    Procedure TestForStatementEmpty;
    Procedure TestForStatementFull;
    Procedure TestForStatementFull1;
    Procedure TestForStatementCompact;
    Procedure TestForStatement2loops2inits;
    Procedure TestForInStatement;
    Procedure TestWhileStatement;
    Procedure TestDoWhileStatement;
    Procedure TestSwitchStatementEmpty;
    Procedure TestSwitchStatementEmptyCompact;
    Procedure TestSwitchStatementOneElement;
    Procedure TestSwitchStatementOneElementCompact;
    Procedure TestSwitchStatementTwoElements;
    Procedure TestSwitchStatementTwoElementsCompact;
    Procedure TestSwitchStatementTwoElementsDefault;
    Procedure TestSwitchStatementTwoElementsDefaultCompact;
    Procedure TestSwitchStatementTwoElementsOneEmpty;
    Procedure TestSwitchStatementTwoElementsOneEmptyCompact;
    Procedure TestIfThen;
    Procedure TestIfThenElse;
    Procedure TestStatementListEmpty;
    Procedure TestStatementListEmptyCompact;
    Procedure TestStatementListOneStatement;
    Procedure TestStatementListOneStatementCompact;
    Procedure TestStatementListTwoStatements;
    Procedure TestStatementListTwoStatementsCompact;
    Procedure TestStatementListTree4;
    Procedure TestStatementListFor;
    Procedure TestEmptyFunctionDef;
    Procedure TestEmptyFunctionDefCompact;
    Procedure TestFunctionDefParams;
    Procedure TestFunctionDefParamsCompact;
    Procedure TestFunctionDefBody1;
    Procedure TestFunctionDefBody1Compact;
    Procedure TestFunctionDefBody2;
    Procedure TestFunctionDefBody2Compact;
    Procedure TestTryCatch;
    Procedure TestTryCatchCompact;
    Procedure TestTryFinally;
    Procedure TestTryFinallyCompact;
    Procedure TestTryCatchFinally;
    Procedure TestTryCatchFinallyCompact;
    Procedure TestWith;
    Procedure TestWithCompact;
    Procedure TestSourceElements;
    Procedure TestSourceElementsCompact;
  end;

  { TTestExpressionWriter }

  TTestExpressionWriter = class(TTestJSWriter)
  Protected
    Procedure TestUnary(Const Msg : String; AClass : TJSUnaryClass; Result : String);
    Procedure TestBinary(Const Msg : String; AClass : TJSBinaryClass; Result : String; ACompact : Boolean);
    Procedure TestBinaryNested(Const Msg : String; AClass : TJSBinaryClass; Result : String; ACompact : Boolean);
  Published
    Procedure TestIdent;
    Procedure TestThis;
    Procedure TestThrowStatement;
    Procedure TestUnaryDelete;
    Procedure TestUnaryVoid;
    Procedure TestUnaryTypeOf;
    Procedure TestPrefixPlusPLus;
    Procedure TestPrefixMinusMinus;
    Procedure TestUnaryMinus;
    Procedure TestUnaryPlus;
    Procedure TestUnaryInv;
    Procedure TestUnaryNot;
    Procedure TestPostPlusPLus;
    Procedure TestPostMinusMinus;
    Procedure TestBinaryLogicalOr;
    Procedure TestBinaryLogicalOrCompact;
    Procedure TestBinaryLogicalOrNested;
    Procedure TestBinaryLogicalAnd;
    Procedure TestBinaryLogicalAndCompact;
    Procedure TestBinaryLogicalAndNested;
    Procedure TestBinaryBitwiseOr;
    Procedure TestBinaryBitwiseOrCompact;
    Procedure TestBinaryBitwiseAnd;
    Procedure TestBinaryBitwiseAndCompact;
    Procedure TestBinaryBitwiseXOr;
    Procedure TestBinaryBitwiseXOrCompact;
    Procedure TestBinaryEQ;
    Procedure TestBinaryEQCompact;
    Procedure TestBinaryNE;
    Procedure TestBinaryNECompact;
    Procedure TestBinarySEQ;
    Procedure TestBinarySEQCompact;
    Procedure TestBinarySNE;
    Procedure TestBinarySNECompact;
    Procedure TestBinaryLT;
    Procedure TestBinaryLTCompact;
    Procedure TestBinaryGT;
    Procedure TestBinaryGTCompact;
    Procedure TestBinaryLE;
    Procedure TestBinaryLECompact;
    Procedure TestBinaryGE;
    Procedure TestBinaryGECompact;
    Procedure TestBinaryIN;
    Procedure TestBinaryINCompact;
    Procedure TestBinaryInstanceOf;
    Procedure TestBinaryInstanceOfCompact;
    Procedure TestBinaryLShift;
    Procedure TestBinaryLShiftOfCompact;
    Procedure TestBinaryRShift;
    Procedure TestBinaryRShiftOfCompact;
    Procedure TestBinaryURShift;
    Procedure TestBinaryURShiftOfCompact;
    Procedure TestBinaryPlus;
    Procedure TestBinaryPlusCompact;
    Procedure TestBinaryPlusNested;
    Procedure TestBinaryMinus;
    Procedure TestBinaryMinusCompact;
    Procedure TestBinaryMinusNested;
    Procedure TestBinaryMultiply;
    Procedure TestBinaryMultiplyCompact;
    Procedure TestBinaryMultiplyNested;
    Procedure TestBinaryDivide;
    Procedure TestBinaryDivideCompact;
    Procedure TestBinaryMod;
    Procedure TestBinaryModCompact;
    Procedure TestBinaryComma;
    Procedure TestBinaryCommaCompact;
    Procedure TestBinaryCallDiv;
    Procedure TestDotMember;
    Procedure TestArgMember;
    Procedure TestNewMember;
    Procedure TestNewMemberCompact;
    Procedure TestNewMemberNoArgs;
    Procedure TestCall;
    Procedure TestCallCompact;
    Procedure TestCallCompact2;
    Procedure TestCallNoArgs;
    Procedure TestConditional;
    Procedure TestRegularExpressionLiteral;
    Procedure TestRegularExpressionLiteralFlags;
  end;

implementation

{ TTestExpressionWriter }

procedure TTestExpressionWriter.TestUnary(const Msg: String;
  AClass: TJSUnaryClass; Result: String);
Var
  U : TJSUnary;

begin
  U:=AClass.Create(0,0);
  U.A:=CreateIdent('a');
  AssertWrite(Msg,Result,U);
end;

procedure TTestExpressionWriter.TestBinary(const Msg: String;
  AClass: TJSBinaryClass; Result: String; ACompact: Boolean);
Var
  U : TJSBinary;

begin
  if ACompact then
    Writer.Options:=Writer.Options+[woCompact];
  U:=AClass.Create(0,0);
  U.A:=CreateIdent('a');
  U.B:=CreateIdent('b');
  AssertWrite(Msg,Result,U);
end;

procedure TTestExpressionWriter.TestBinaryNested(const Msg: String;
  AClass: TJSBinaryClass; Result: String; ACompact: Boolean);
var
  U: TJSBinary;
begin
  if ACompact then
    Writer.Options:=Writer.Options+[woCompact];
  U:=AClass.Create(0,0);
  U.A:=AClass.Create(0,0);
  TJSBinary(U.A).A:=CreateIdent('a');
  TJSBinary(U.A).B:=CreateIdent('b');
  U.B:=AClass.Create(0,0);
  TJSBinary(U.B).A:=CreateIdent('c');
  TJSBinary(U.B).B:=CreateIdent('d');
  AssertWrite(Msg,Result,U);
end;

procedure TTestExpressionWriter.TestIdent;

begin
  AssertWrite('ABC','ABC',CreateIdent('ABC'));
end;

procedure TTestExpressionWriter.TestThis;
begin
  AssertWrite('this','this',TJSPrimaryExpressionThis.Create(0,0));
end;

procedure TTestExpressionWriter.TestThrowStatement;

begin
  TestUnary('Throw expresssion',TJSThrowStatement,'throw a');
end;

procedure TTestExpressionWriter.TestUnaryDelete;
begin
  TestUnary('Delete expresssion',TJSUnaryDeleteExpression,'delete a');
end;

procedure TTestExpressionWriter.TestUnaryVoid;
begin
  TestUnary('Void expresssion',TJSUnaryVoidExpression,'void a');
end;

procedure TTestExpressionWriter.TestUnaryTypeOf;
begin
  TestUnary('typeof expresssion',TJSUnaryTypeOfExpression,'typeof a');
end;

procedure TTestExpressionWriter.TestPrefixPlusPLus;
begin
  TestUnary('prefix ++ expresssion',TJSUnaryPrePlusPlusExpression,'++a');
end;

procedure TTestExpressionWriter.TestPrefixMinusMinus;
begin
  TestUnary('prefix -- expresssion',TJSUnaryPreMinusMinusExpression,'--a');
end;

procedure TTestExpressionWriter.TestUnaryMinus;
begin
  TestUnary('unary - expresssion',TJSUnaryMinusExpression,'-a');
end;

procedure TTestExpressionWriter.TestUnaryPlus;
begin
  TestUnary('unary + expresssion',TJSUnaryPlusExpression,'+a');
end;

procedure TTestExpressionWriter.TestUnaryInv;
begin
  TestUnary('unary invert expresssion',TJSUnaryInvExpression,'~a');
end;

procedure TTestExpressionWriter.TestUnaryNot;
begin
  TestUnary('unary not expresssion',TJSUnaryNotExpression,'!a');
end;

procedure TTestExpressionWriter.TestPostPlusPLus;
begin
  TestUnary('postfix ++ expresssion',TJSUnaryPostPlusPlusExpression,'a++');
end;

procedure TTestExpressionWriter.TestPostMinusMinus;
begin
  TestUnary('postfix -- expresssion',TJSUnaryPostMinusMinusExpression,'a--');
end;

procedure TTestExpressionWriter.TestBinaryLogicalOr;
begin
  TestBinary('logical or',TJSLogicalOrExpression,'(a || b)',False);
end;

procedure TTestExpressionWriter.TestBinaryLogicalOrCompact;
begin
  TestBinary('logical or',TJSLogicalOrExpression,'(a||b)',True);
end;

procedure TTestExpressionWriter.TestBinaryLogicalOrNested;
begin
  TestBinaryNested('logical or',TJSLogicalOrExpression,'(a||b||c||d)',True);
end;

procedure TTestExpressionWriter.TestBinaryLogicalAnd;
begin
  TestBinary('logical or',TJSLogicalAndExpression,'(a && b)',False);
end;

procedure TTestExpressionWriter.TestBinaryLogicalAndCompact;
begin
  TestBinary('logical or',TJSLogicalAndExpression,'(a&&b)',True);
end;

procedure TTestExpressionWriter.TestBinaryLogicalAndNested;
begin
  TestBinaryNested('logical and',TJSLogicalAndExpression,'(a&&b&&c&&d)',True);
end;

procedure TTestExpressionWriter.TestBinaryBitwiseOr;
begin
  TestBinary('Bitwise or',TJSBitwiseOrExpression,'(a | b)',False);
end;

procedure TTestExpressionWriter.TestBinaryBitwiseOrCompact;
begin
  TestBinary('Bitwise or',TJSBitwiseOrExpression,'(a|b)',True);
end;

procedure TTestExpressionWriter.TestBinaryBitwiseAnd;
begin
  TestBinary('Bitwise and',TJSBitwiseAndExpression,'(a & b)',False);
end;

procedure TTestExpressionWriter.TestBinaryBitwiseAndCompact;
begin
  TestBinary('Bitwise and',TJSBitwiseAndExpression,'(a&b)',True);
end;

procedure TTestExpressionWriter.TestBinaryBitwiseXOr;
begin
  TestBinary('Bitwise xor',TJSBitwiseXOrExpression,'(a ^ b)',False);
end;

procedure TTestExpressionWriter.TestBinaryBitwiseXOrCompact;
begin
  TestBinary('Bitwise xor',TJSBitwiseXOrExpression,'(a^b)',True);
end;

procedure TTestExpressionWriter.TestBinaryEQ;
begin
  TestBinary('Equal',TJSEqualityExpressionEQ,'(a == b)',False);
end;

procedure TTestExpressionWriter.TestBinaryEQCompact;
begin
  TestBinary('Equal',TJSEqualityExpressionEQ,'(a==b)',True);
end;

procedure TTestExpressionWriter.TestBinaryNE;
begin
  TestBinary('Not Equal',TJSEqualityExpressionNE,'(a != b)',False);
end;

procedure TTestExpressionWriter.TestBinaryNECompact;
begin
  TestBinary('Not Equal',TJSEqualityExpressionNE,'(a!=b)',True);
end;

procedure TTestExpressionWriter.TestBinarySEQ;
begin
  TestBinary('Strictly Equal',TJSEqualityExpressionSEQ,'(a === b)',False);
end;

procedure TTestExpressionWriter.TestBinarySEQCompact;
begin
  TestBinary('Strictly Equal',TJSEqualityExpressionSEQ,'(a===b)',True);
end;

procedure TTestExpressionWriter.TestBinarySNE;
begin
  TestBinary('Strictly Equal',TJSEqualityExpressionSNE,'(a !== b)',False);
end;

procedure TTestExpressionWriter.TestBinarySNECompact;
begin
  TestBinary('Strictly Equal',TJSEqualityExpressionSNE,'(a!==b)',True);
end;

procedure TTestExpressionWriter.TestBinaryLT;
begin
  TestBinary('Less than',TJSRelationalExpressionLT,'(a < b)',False);
end;

procedure TTestExpressionWriter.TestBinaryLTCompact;
begin
  TestBinary('Less than',TJSRelationalExpressionLT,'(a<b)',True);
end;

procedure TTestExpressionWriter.TestBinaryGT;
begin
  TestBinary('Greater than',TJSRelationalExpressionGT,'(a > b)',False);
end;

procedure TTestExpressionWriter.TestBinaryGTCompact;
begin
  TestBinary('Greater than',TJSRelationalExpressionGT,'(a>b)',True);
end;

procedure TTestExpressionWriter.TestBinaryLE;
begin
  TestBinary('Less than or equal',TJSRelationalExpressionLE,'(a <= b)',False);
end;

procedure TTestExpressionWriter.TestBinaryLECompact;
begin
  TestBinary('Less than or equal',TJSRelationalExpressionLE,'(a<=b)',True);
end;

procedure TTestExpressionWriter.TestBinaryGE;
begin
  TestBinary('Greater than or equal',TJSRelationalExpressionGE,'(a >= b)',False);
end;

procedure TTestExpressionWriter.TestBinaryGECompact;
begin
  TestBinary('Greater than or equal',TJSRelationalExpressionGE,'(a>=b)',True);
end;

procedure TTestExpressionWriter.TestBinaryIN;
begin
  TestBinary('Prop in Object',TJSRelationalExpressionIN,'(a in b)',False);
end;

procedure TTestExpressionWriter.TestBinaryINCompact;
begin
  TestBinary('Prop in Object',TJSRelationalExpressionIN,'(a in b)',True);
end;

procedure TTestExpressionWriter.TestBinaryInstanceOf;
begin
  TestBinary('A instanceof Object',TJSRelationalExpressionInStanceOf,'(a instanceof b)',False);
end;

procedure TTestExpressionWriter.TestBinaryInstanceOfCompact;
begin
  TestBinary('A instanceof Object',TJSRelationalExpressionInStanceOf,'(a instanceof b)',true);
end;

procedure TTestExpressionWriter.TestBinaryLShift;
begin
  TestBinary('A lshift B',TJSLShiftExpression,'(a << b)',False);
end;

procedure TTestExpressionWriter.TestBinaryLShiftOfCompact;
begin
  TestBinary('A lshift B',TJSLShiftExpression,'(a<<b)',True);
end;

procedure TTestExpressionWriter.TestBinaryRShift;
begin
  TestBinary('A rshift B',TJSRShiftExpression,'(a >> b)',False);
end;

procedure TTestExpressionWriter.TestBinaryRShiftOfCompact;
begin
  TestBinary('A rshift B',TJSRShiftExpression,'(a>>b)',True);
end;

procedure TTestExpressionWriter.TestBinaryURShift;
begin
  TestBinary('A urshift B',TJSURShiftExpression,'(a >>> b)',False);
end;

procedure TTestExpressionWriter.TestBinaryURShiftOfCompact;
begin
  TestBinary('A urshift B',TJSURShiftExpression,'(a>>>b)',True);
end;

procedure TTestExpressionWriter.TestBinaryPlus;
begin
  TestBinary('A plus B',TJSAdditiveExpressionPlus,'(a + b)',False);
end;

procedure TTestExpressionWriter.TestBinaryPlusCompact;
begin
  TestBinary('A plus B',TJSAdditiveExpressionPlus,'(a+b)',True);
end;

procedure TTestExpressionWriter.TestBinaryPlusNested;
begin
  TestBinaryNested('(A+B)+(C+D)',TJSAdditiveExpressionPlus,'(a+b+(c+d))',True);
end;

procedure TTestExpressionWriter.TestBinaryMinus;
begin
  TestBinary('A minus B',TJSAdditiveExpressionMinus,'(a - b)',False);
end;

procedure TTestExpressionWriter.TestBinaryMinusCompact;
begin
  TestBinary('A minus B',TJSAdditiveExpressionMinus,'(a-b)',True);
end;

procedure TTestExpressionWriter.TestBinaryMinusNested;
begin
  TestBinaryNested('(A-B)-(C-D)',TJSAdditiveExpressionMinus,'(a-b-(c-d))',True);
end;

procedure TTestExpressionWriter.TestBinaryMultiply;
begin
  TestBinary('A multiply B',TJSMultiplicativeExpressionMul,'(a * b)',False);
end;

procedure TTestExpressionWriter.TestBinaryMultiplyCompact;
begin
  TestBinary('A multiply B',TJSMultiplicativeExpressionMul,'(a*b)',True);
end;

procedure TTestExpressionWriter.TestBinaryMultiplyNested;
begin
  TestBinaryNested('(A*B)*(C*D)',TJSMultiplicativeExpressionMul,'(a*b*(c*d))',True);
end;

procedure TTestExpressionWriter.TestBinaryDivide;
begin
  TestBinary('A divide B',TJSMultiplicativeExpressionDiv,'(a / b)',False);
end;

procedure TTestExpressionWriter.TestBinaryDivideCompact;
begin
  TestBinary('A divide B',TJSMultiplicativeExpressionDiv,'(a/b)',True);
end;

procedure TTestExpressionWriter.TestBinaryMod;
begin
  TestBinary('A mod B',TJSMultiplicativeExpressionMod,'(a % b)',False);
end;

procedure TTestExpressionWriter.TestBinaryModCompact;
begin
  TestBinary('A mod B',TJSMultiplicativeExpressionMod,'(a%b)',True);
end;

procedure TTestExpressionWriter.TestBinaryComma;
begin
  TestBinary('A comma B',TJSCommaExpression,'(a, b)',False);
end;

procedure TTestExpressionWriter.TestBinaryCommaCompact;
begin
  TestBinary('A comma B',TJSCommaExpression,'(a,b)',True);
end;

procedure TTestExpressionWriter.TestBinaryCallDiv;
var
  aDiv: TJSMultiplicativeExpressionDiv;
  LeftMul: TJSMultiplicativeExpressionMul;
  LeftCall: TJSCallExpression;
  RightSub: TJSAdditiveExpressionMinus;
  Expr: String;
begin
  // (2*f(3))/(a-4)
  aDiv:=TJSMultiplicativeExpressionDiv.Create(0,0);
  // (2*f(3))
  LeftMul:=TJSMultiplicativeExpressionMul.Create(0,0);
  aDiv.A:=LeftMul;
  // 2
  LeftMul.A:=CreateLiteral(2);
  // f(3)
  LeftCall:=TJSCallExpression.Create(0,0);
  LeftMul.B:=LeftCall;
  LeftCall.Expr:=CreateIdent('f');
  LeftCall.Args:=TJSArguments.Create(0,0);
  LeftCall.AddArg(CreateLiteral(3));
  // (a-4)
  RightSub:=TJSAdditiveExpressionMinus.Create(0,0);
  aDiv.B:=RightSub;
  RightSub.A:=CreateIdent('a');
  RightSub.B:=CreateLiteral(4);

  Expr:='((2 * f(3)) / (a - 4))';
  AssertWrite('keep needed brackets of '+Expr,Expr,aDiv);
end;

procedure TTestExpressionWriter.TestDotMember;
Var
  U : TJSDotMemberExpression;

begin
  U:=TJSDotMemberExpression.Create(0,0);
  U.Mexpr:=CreateIdent('a');
  U.Name:='b';
  AssertWrite('member b of object a (a.b)','a.b',U);
end;

procedure TTestExpressionWriter.TestArgMember;
Var
  U : TJSBracketMemberExpression;

begin
  U:=TJSBracketMemberExpression.Create(0,0);
  U.Mexpr:=CreateIdent('a');
  U.Name:=CreateIdent('b');
  AssertWrite('member b of object a (a[b])','a[b]',U);
end;

procedure TTestExpressionWriter.TestNewMember;
Var
  U : TJSNewMemberExpression;

begin
  U:=TJSNewMemberExpression.Create(0,0);
  U.Mexpr:=CreateIdent('a');;
  U.Args:=TJSArguments.Create(0,0);
  U.Args.Elements.AddElement;
  U.Args.Elements[0].Expr:=CreateLiteral(123);
  AssertWrite('member b of object a (a[b])','new a(123)',U);
end;

procedure TTestExpressionWriter.TestNewMemberCompact;

Var
  U : TJSNewMemberExpression;

begin
  Writer.Options:=Writer.Options+[woCompact];
  U:=TJSNewMemberExpression.Create(0,0);
  U.Mexpr:=CreateIdent('a');
  U.Args:=TJSArguments.Create(0,0);
  U.Args.Elements.AddElement;
  U.Args.Elements[0].Expr:=CreateLiteral(123);
  AssertWrite('new a(123)','new a(123)',U);
end;

procedure TTestExpressionWriter.TestNewMemberNoArgs;
Var
  U : TJSNewMemberExpression;

begin
  U:=TJSNewMemberExpression.Create(0,0);
  U.Mexpr:=CreateIdent('a');
  AssertWrite('new a()','new a()',U);
end;

procedure TTestExpressionWriter.TestCall;
Var
  U : TJSCallExpression;

begin
  U:=TJSCallExpression.Create(0,0);
  U.Expr:=CreateIdent('a');
  U.Args:=TJSArguments.Create(0,0);
  U.Args.Elements.AddElement;
  U.Args.Elements[0].Expr:=CreateLiteral(123);
  AssertWrite('call a(123)',
     'a(123)',U);
end;

procedure TTestExpressionWriter.TestCallCompact;
Var
  U : TJSCallExpression;

begin
  Writer.Options:=Writer.Options+[woCompact];
  U:=TJSCallExpression.Create(0,0);
  U.Expr:=CreateIdent('a');
  U.Args:=TJSArguments.Create(0,0);
  U.Args.Elements.AddElement;
  U.Args.Elements[0].Expr:=CreateLiteral(123);
  AssertWrite('call a(123)','a(123)',U);
end;

procedure TTestExpressionWriter.TestCallCompact2;
Var
  U : TJSCallExpression;

begin
  Writer.Options:=Writer.Options+[woCompactArguments];
  U:=TJSCallExpression.Create(0,0);
  U.Expr:=CreateIdent('a');
  U.Args:=TJSArguments.Create(0,0);
  U.Args.Elements.AddElement;
  U.Args.Elements[0].Expr:=CreateLiteral(123);
  U.Args.Elements.AddElement;
  U.Args.Elements[1].Expr:=CreateLiteral(456);
  AssertWrite('call a(123,456)','a(123,456)',U);

end;

procedure TTestExpressionWriter.TestCallNoArgs;
Var
  U : TJSCallExpression;

begin
  U:=TJSCallExpression.Create(0,0);
  U.Expr:=CreateIdent('a');
  AssertWrite('call a()','a()',U);
end;

procedure TTestExpressionWriter.TestConditional;
Var
  U : TJSConditionalExpression;

begin
  U:=TJSConditionalExpression.Create(0,0);
  U.A:=CreateIdent('a');
  U.B:=CreateIdent('b');
  U.C:=CreateIdent('c');
  AssertWrite('a ? b : c','(a ? b : c)',U);
end;

procedure TTestExpressionWriter.TestRegularExpressionLiteral;

Var
  S : TJSRegularExpressionLiteral;
begin
  S:=TJSRegularExpressionLiteral.Create(0,0);
  S.Pattern.AsString:='a';
  AssertWrite('/a/','/a/',S);
end;

procedure TTestExpressionWriter.TestRegularExpressionLiteralFlags;
Var
  S : TJSRegularExpressionLiteral;
begin
  S:=TJSRegularExpressionLiteral.Create(0,0);
  S.Pattern.AsString:='a';
  S.PatternFlags.AsString:='g';
  AssertWrite('/a/g','/a/g',S);
end;

{ ---------------------------------------------------------------------
  TTestStatementWriter
  ---------------------------------------------------------------------}

procedure TTestStatementWriter.TestAssignment(const Msg: String;
  AClass: TJSAssignStatementClass; Result: String; ACompact: Boolean);
Var
  U : TJSAssignStatement;
begin
  if ACompact then
    Writer.Options:=Writer.Options+[woCompact];
  U:=CreateAssignment(AClass);
  AssertWrite(Msg,Result,U);
end;

function TTestStatementWriter.CreateAssignment(AClass: TJSAssignStatementClass
  ): TJSAssignStatement;
begin
  if AClass=Nil then
     AClass := TJSSimpleAssignStatement;
  Result:=AClass.Create(0,0);
  Result.LHS:=CreateIdent('a');
  Result.Expr:=CreateIdent('b');
end;

function TTestStatementWriter.CreateStatementListOneElement: TJSStatementList;
begin
  Result:=TJSStatementList.Create(0,0);
  Result.A:=CreateAssignment(nil);
end;

function TTestStatementWriter.CreateStatementListTwoElement2: TJSStatementList;
begin
  Result:=TJSStatementList.Create(0,0);
  Result.A:=CreateAssignment(nil);
  Result.B:=CreateAssignment(nil);
end;

procedure TTestStatementWriter.TestEmptyStatement;

begin
  AssertWrite('Empty statement','',TJSEmptyStatement.Create(0,0));
end;

procedure TTestStatementWriter.TestEmptyStatementComment;
begin
  Writer.Options:=[woEmptyStatementAsComment,woUseUTF8];
  AssertWrite('Empty statement as comment','/* Empty statement */',TJSEmptyStatement.Create(0,0));
end;

procedure TTestStatementWriter.TestEmptyStatementBlock;
begin
  AssertWrite('Empty statement block','{'+sLineBreak+'}',TJSEmptyBlockStatement.Create(0,0));
end;

procedure TTestStatementWriter.TestEmptyStatementBlockIndent;
begin
  Writer.IndentSize:=2;
  Writer.Indent;
  AssertWrite('Empty statement block','  {'+sLineBreak+'  }',TJSEmptyBlockStatement.Create(0,0));
end;

procedure TTestStatementWriter.TestEmptyStatementBlockCompact;
begin
  Writer.Options:=[woCompact,woUseUTF8];
  AssertWrite('Empty statement block','{}',TJSEmptyBlockStatement.Create(0,0));
end;

procedure TTestStatementWriter.TestVarDeclaration;

Var
  V : TJSVarDeclaration;
begin
  V:=TJSVarDeclaration.Create(0,0);
  V.Name:='a';
  AssertWrite('simple var','a',V);
end;

procedure TTestStatementWriter.TestVarDeclarationInit;
Var
  V : TJSVarDeclaration;
begin
  V:=TJSVarDeclaration.Create(0,0);
  V.Name:='a';
  V.Init:=CreateLiteral(1);
  AssertWrite('simple var, init ','a = 1',V);
end;

procedure TTestStatementWriter.TestVarListDeclaration;
Var
  B,L : TJSVariableDeclarationList;
  V : TJSVarDeclaration;

begin
  L:=TJSVariableDeclarationList.Create(0,0);
  V:=TJSVarDeclaration.Create(0,0);
  V.Name:='a';
  L.A:=V;
  B:=TJSVariableDeclarationList.Create(0,0);
  V:=TJSVarDeclaration.Create(0,0);
  V.Name:='b';
  B.A:=V;
  V.Init:=CreateLiteral(1);
  L.B:=B;
  AssertWrite('simple var list ','a, b = 1',L);
end;

procedure TTestStatementWriter.TestVarListDeclarationInit;
Var
  B,L : TJSVariableDeclarationList;
  V : TJSVarDeclaration;


begin
  L:=TJSVariableDeclarationList.Create(0,0);
  V:=TJSVarDeclaration.Create(0,0);;
  V.Name:='a';
  L.A:=V;
  B:=TJSVariableDeclarationList.Create(0,0);
  V:=TJSVarDeclaration.Create(0,0);;
  V.Name:='b';
  B.A:=V;
  L.B:=B;
  AssertWrite('simple var list ','a, b',L);
end;

procedure TTestStatementWriter.TestVarDeclarationStatement;

Var
  S : TJSVariableStatement;
  V : TJSVarDeclaration;
begin
  S:=TJSVariableStatement.Create(0,0);
  V:=TJSVarDeclaration.Create(0,0);
  S.A:=V;
  V.Name:='a';
  AssertWrite('simple var','var a',S);
end;

procedure TTestStatementWriter.TestVarListDeclarationStatement;

Var
  S : TJSVariableStatement;
  V : TJSVarDeclaration;
  L : TJSVariableDeclarationList;

begin
  S:=TJSVariableStatement.Create(0,0);
  L:=TJSVariableDeclarationList.Create(0,0);
  V:=TJSVarDeclaration.Create(0,0);
  L.A:=V;
  S.A:=L;
  V.Name:='a';
  AssertWrite('simple var','var a',S);
end;

procedure TTestStatementWriter.TestVarListDeclarationStatement2Vars;
Var
  S : TJSVariableStatement;
  V : TJSVarDeclaration;
  L : TJSVariableDeclarationList;

begin
  S:=TJSVariableStatement.Create(0,0);
  L:=TJSVariableDeclarationList.Create(0,0);
  S.A:=L;
  V:=TJSVarDeclaration.Create(0,0);
  L.A:=V;
  V.Name:='a';
  L.B:=TJSVariableDeclarationList.Create(0,0);
  L:=TJSVariableDeclarationList(L.B);
  V:=TJSVarDeclaration.Create(0,0);
  L.A:=V;
  V.Name:='b';
  AssertWrite('simple 2 vars','var a, b',S);
end;

procedure TTestStatementWriter.TestVarListDeclarationStatement3Vars;
Var
  S : TJSVariableStatement;
  V : TJSVarDeclaration;
  L : TJSVariableDeclarationList;

begin
  S:=TJSVariableStatement.Create(0,0);
  L:=TJSVariableDeclarationList.Create(0,0);
  S.A:=L;
  V:=TJSVarDeclaration.Create(0,0);
  L.A:=V;
  V.Name:='a';
  V.Init:=CreateLiteral(1);
  L.B:=TJSVariableDeclarationList.Create(0,0);
  L:=TJSVariableDeclarationList(L.B);
  V:=TJSVarDeclaration.Create(0,0);
  L.A:=V;
  V.Name:='b';
  V.Init:=CreateLiteral(2);
  V:=TJSVarDeclaration.Create(0,0);
  L.B:=V;
  V.Name:='c';
  V.Init:=CreateLiteral(3);
  AssertWrite('simple 3 vars','var a = 1, b = 2, c = 3',S);
end;

procedure TTestStatementWriter.TestReturnStatement;
Var
  S : TJSReturnStatement;

begin
  S:=TJSReturnStatement.Create(0,0);
  S.Expr:=CreateIdent('a');
  AssertWrite('simple return','return a',S);
end;

procedure TTestStatementWriter.TestLabeledStatement;
Var
  LS : TJSLabeledStatement;
  S : TJSReturnStatement;

begin
  LS:=TJSLabeledStatement.Create(0,0);
  LS.TheLabel:=TJSLabel.Create;
  LS.TheLabel.Name:='loc';
  S:=TJSReturnStatement.Create(0,0);
  S.Expr:=CreateIDent('a');
  LS.A:=S;
  AssertWrite('simple return','loc:'+sLineBreak+'return a',LS);
end;

procedure TTestStatementWriter.TestLabeledStatementCompact;
Var
  LS : TJSLabeledStatement;
  S : TJSReturnStatement;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  LS:=TJSLabeledStatement.Create(0,0);
  LS.TheLabel:=TJSLabel.Create;
  LS.TheLabel.Name:='loc';
  S:=TJSReturnStatement.Create(0,0);
  S.Expr:=CreateIdent('a');
  LS.A:=S;
  AssertWrite('simple return','loc: return a',LS);
end;

procedure TTestStatementWriter.TestContinueStatement;

Var
  S : TJSContinueStatement;

begin
  S:=TJSContinueStatement.Create(0,0);
  AssertWrite('simple continue','continue',S);
end;

procedure TTestStatementWriter.TestContinueTargetStatement;

Var
  S : TJSContinueStatement;

begin
  S:=TJSContinueStatement.Create(0,0);
  S.TargetName:='a';
  AssertWrite('continue a','continue a',S);
end;

procedure TTestStatementWriter.TestBreakStatement;

Var
  S : TJSBreakStatement;

begin
  S:=TJSBreakStatement.Create(0,0);
  AssertWrite('simple break','break',S);
end;

procedure TTestStatementWriter.TestBreakTargetStatement;
Var
  S : TJSBreakStatement;

begin
  S:=TJSBreakStatement.Create(0,0);
  S.TargetName:='a';
  AssertWrite('simple break a','break a',S);
end;

procedure TTestStatementWriter.TestAssignmentStatementSimple;
begin
  TestAssignment('Simple assignment',TJSSimpleAssignStatement,'a = b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementSimpleCompact;
begin
  TestAssignment('Simple assignment',TJSSimpleAssignStatement,'a=b',True);
end;

procedure TTestStatementWriter.TestAssignmentStatementAdd;
begin
  TestAssignment('Add assignment',TJSAddEqAssignStatement,'a += b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementAddCompact;
begin
  TestAssignment('Add assignment',TJSAddEqAssignStatement,'a+=b',True);
end;

procedure TTestStatementWriter.TestAssignmentStatementSubtract;
begin
  TestAssignment('Subtract assignment',TJSSubEqAssignStatement,'a -= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementSubtractCompact;
begin
  TestAssignment('Subtract assignment',TJSSubEqAssignStatement,'a-=b',True);
end;

procedure TTestStatementWriter.TestAssignmentStatementMultiply;
begin
  TestAssignment('Multiply assignment',TJSMulEqAssignStatement,'a *= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementMultiplyCompact;
begin
  TestAssignment('Multiply assignment',TJSMulEqAssignStatement,'a*=b',True);
end;

procedure TTestStatementWriter.TestAssignmentStatementDivide;
begin
  TestAssignment('Divide assignment',TJSDivEqAssignStatement,'a /= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementDivideCompact;
begin
  TestAssignment('Divide assignment',TJSDivEqAssignStatement,'a/=b',True);
end;

procedure TTestStatementWriter.TestAssignmentStatementShift;
begin
  TestAssignment('Shift assignment',TJSLShiftEqAssignStatement,'a <<= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementShiftCompact;
begin
  TestAssignment('Shift assignment',TJSLShiftEqAssignStatement,'a<<=b',True);
end;

procedure TTestStatementWriter.TestAssignmentStatementRShift;
begin
  TestAssignment('RShift assignment',TJSRShiftEqAssignStatement,'a >>= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementRShiftCompact;
begin
  TestAssignment('RShift assignment',TJSRShiftEqAssignStatement,'a>>=b',True);
end;

procedure TTestStatementWriter.TestAssignmentStatementURShift;
begin
  TestAssignment('URShift assignment',TJSURShiftEqAssignStatement,'a >>>= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementURShiftCompact;
begin
  TestAssignment('URShift assignment',TJSURShiftEqAssignStatement,'a>>>=b',True);
end;

procedure TTestStatementWriter.TestAssignmentStatementMod;
begin
  TestAssignment('Mod assignment',TJSModEqAssignStatement,'a %= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementModCompact;
begin
  TestAssignment('Mod assignment',TJSModEqAssignStatement,'a%=b',True);
end;

procedure TTestStatementWriter.TestAssignmentStatementBinaryOr;
begin
  TestAssignment('Binary or assignment',TJSOrEqAssignStatement,'a |= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementBinaryOrCompact;
begin
  TestAssignment('Binary or assignment',TJSOrEqAssignStatement,'a |= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementBinaryXOr;
begin
  TestAssignment('Binary xor assignment',TJSXOrEqAssignStatement,'a ^= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementBinaryXOrCompact;
begin
  TestAssignment('Binary xor assignment',TJSXOrEqAssignStatement,'a^=b',True);
end;

procedure TTestStatementWriter.TestAssignmentStatementBinaryAnd;
begin
  TestAssignment('Binary and assignment',TJSAndEqAssignStatement,'a &= b',False);
end;

procedure TTestStatementWriter.TestAssignmentStatementBinaryAndCompact;
begin
  TestAssignment('Binary and assignment',TJSAndEqAssignStatement,'a&=b',True);
end;

procedure TTestStatementWriter.TestForStatementEmpty;

Var
  S : TJSForStatement;
begin
  S:=TJSForStatement.Create(0,0);
  S.Body:=TJSEmptyBlockStatement.Create(0,0);
  AssertWrite('neverending for','for (; ; ) {'+sLineBreak+'}',S);
end;

procedure TTestStatementWriter.TestForStatementFull;

Var
  S : TJSForStatement;
  UPP : TJSUnaryPostPlusPlusExpression;
  CL : TJSRelationalExpressionLT;
  sa : TJSSimpleAssignStatement;

begin
  SA:=TJSSimpleAssignStatement.Create(0,0);
  SA.LHS:=CreateIdent('i');
  SA.Expr:=CreateLiteral(0);
  UPP:=TJSUnaryPostPlusPlusExpression.Create(0,0);
  UPP.A:=CreateIdent('i');
  CL:=TJSRelationalExpressionLT.Create(0,0);
  CL.A:=CreateIdent('i');
  CL.B:=CreateLiteral(10);
  S:=TJSForStatement.Create(0,0);
  S.Init:=SA;
  S.Incr:=UPP;
  S.Cond:=CL;
  S.Body:=TJSEmptyBlockStatement.Create(0,0);
  AssertWrite('for i:=0 to 9','for (i = 0; i < 10; i++) {'+sLineBreak+'}',S);
end;

procedure TTestStatementWriter.TestForStatementFull1;

Var
  S : TJSForStatement;
  UPP : TJSUnaryPostPlusPlusExpression;
  CL : TJSRelationalExpressionLT;
  sa : TJSSimpleAssignStatement;

begin
  SA:=TJSSimpleAssignStatement.Create(0,0);
  SA.LHS:=CreateIdent('i');
  SA.Expr:=CreateLiteral(0);
  UPP:=TJSUnaryPostPlusPlusExpression.Create(0,0);
  UPP.A:=CreateIdent('i');
  CL:=TJSRelationalExpressionLT.Create(0,0);
  CL.A:=CreateIdent('i');
  CL.B:=CreateLiteral(10);
  S:=TJSForStatement.Create(0,0);
  S.Init:=SA;
  S.Incr:=UPP;
  S.Cond:=CL;
  S.Body:=CreateStatementListOneElement;
  AssertWrite('for i:=0 to 9',
     'for (i = 0; i < 10; i++) {'+sLineBreak
    +'a = b;'+sLineBreak
    +'}',S);
end;

procedure TTestStatementWriter.TestForStatementCompact;
Var
  S : TJSForStatement;
  UPP : TJSUnaryPostPlusPlusExpression;
  CL : TJSRelationalExpressionLT;
  sa : TJSSimpleAssignStatement;

begin
  SA:=TJSSimpleAssignStatement.Create(0,0);
  SA.LHS:=CreateIdent('i');
  SA.Expr:=CreateLiteral(0);
  UPP:=TJSUnaryPostPlusPlusExpression.Create(0,0);
  UPP.A:=CreateIdent('i');
  CL:=TJSRelationalExpressionLT.Create(0,0);
  CL.A:=CreateIdent('i');
  CL.B:=CreateLiteral(10);
  S:=TJSForStatement.Create(0,0);
  S.Init:=SA;
  S.Incr:=UPP;
  S.Cond:=CL;
  S.Body:=TJSEmptyBlockStatement.Create(0,0);
  Writer.Options:=[woCompact,woUseUTF8];
  AssertWrite('for i:=0 to 9','for (i=0; i<10; i++) {}',S);
end;

procedure TTestStatementWriter.TestForStatement2loops2inits;
var
  L: TJSStatementList;

  function CreateIdent(aName: string): TJSPrimaryExpressionIdent;
  begin
    Result:=TJSPrimaryExpressionIdent.Create(0,0);
    Result.Name:=TJSString(aName);
  end;

  function CreateNumber(i: TJSNumber): TJSLiteral;
  begin
    Result:=TJSLiteral.Create(0,0);
    Result.Value.AsNumber:=i;
  end;

  function CreateAssignSt(LHS, Expr: TJSElement): TJSSimpleAssignStatement;
  begin
    Result:=TJSSimpleAssignStatement.Create(0,0);
    Result.LHS:=LHS;
    Result.Expr:=Expr;
  end;

  function CreateFor(LoopVar, LoopEndVar: string; StartExpr, EndExpr: TJSElement;
    Up: boolean; Target: string): TJSForStatement;
  var
    V: TJSVariableStatement;
    C: TJSCommaExpression;
  begin
    Result:=TJSForStatement.Create(0,0);
    V:=TJSVariableStatement.Create(0,0);
    Result.Init:=V;
    C:=TJSCommaExpression.Create(0,0);
    V.A:=C;
    C.A:=CreateAssignSt(CreateIdent(LoopVar),StartExpr);
    C.B:=CreateAssignSt(CreateIdent(LoopEndVar),EndExpr);

    if Up then
      Result.Cond:=TJSRelationalExpressionLE.Create(0,0)
    else
      Result.Cond:=TJSRelationalExpressionGE.Create(0,0);
    TJSRelationalExpression(Result.Cond).A:=CreateIdent(LoopVar);
    TJSRelationalExpression(Result.Cond).B:=CreateIdent(LoopEndVar);

    if Up then
      Result.Incr:=TJSUnaryPostPlusPlusExpression.Create(0,0)
    else
      Result.Incr:=TJSUnaryPostMinusMinusExpression.Create(0,0);
    TJSUnaryExpression(Result.Incr).A:=CreateIdent(LoopVar);

    Result.Body:=CreateAssignSt(CreateIdent(Target),CreateIdent(LoopVar));
  end;

begin
  L:=TJSStatementList.Create(0,0);
  L.A:=CreateFor('$loop1','$loopend2',CreateNumber(3),CreateNumber(5),true,'$mod.i');
  L.B:=CreateFor('$loop3','$loopend4',CreateNumber(10),CreateNumber(7),false,'$mod.i');
  AssertWrite('for i:=3 to 5 do ; for i:=10 downto 7 do ;',
    '{'+LineEnding
    +'for (var $loop1 = 3, $loopend2 = 5; $loop1 <= $loopend2; $loop1++) $mod.i = $loop1;'+LineEnding
    +'for (var $loop3 = 10, $loopend4 = 7; $loop3 >= $loopend4; $loop3--) $mod.i = $loop3;'+LineEnding
    +'}'
    ,L);
end;

procedure TTestStatementWriter.TestForInStatement;

Var
  S : TJSForInStatement;

begin
  S:=TJSForInStatement.Create(0,0);
  S.LHS:=CreateIdent('a');
  S.List:=CreateIdent('b');
  S.Body:=TJSEmptyBlockStatement.Create(0,0);
  AssertWrite('for a in b','for (a in b) {'+sLineBreak+'}',S);
end;

procedure TTestStatementWriter.TestWhileStatement;
Var
  S : TJSWhileStatement;

begin
  S:=TJSWhileStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  S.Body:=TJSEmptyBlockStatement.Create(0,0);
  AssertWrite('while a ','while (a) {'+sLineBreak+'}',S);
end;

procedure TTestStatementWriter.TestDoWhileStatement;

Var
  S : TJSDoWhileStatement;

begin
  S:=TJSDoWhileStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  S.Body:=TJSEmptyBlockStatement.Create(0,0);
  AssertWrite('do while a ','do {'+sLineBreak+'} while (a)',S);
end;

procedure TTestStatementWriter.TestSwitchStatementEmpty;
Var
  S : TJSSwitchStatement;

begin
  S:=TJSSwitchStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  AssertWrite('switch ','switch (a) {'+sLineBreak+'}',S);
end;

procedure TTestStatementWriter.TestSwitchStatementEmptyCompact;

Var
  S : TJSSwitchStatement;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSSwitchStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  AssertWrite('switch ','switch (a) {}',S);
end;

procedure TTestStatementWriter.TestSwitchStatementOneElement;

Var
  S : TJSSwitchStatement;
  C : TJSCaseElement;
begin
//  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSSwitchStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('c');
  AssertWrite('switch ','switch (a) {'+sLineBreak+'case c:'+sLineBreak+'{'+sLineBreak+'}'+sLineBreak+'}',S);
end;

procedure TTestStatementWriter.TestSwitchStatementOneElementCompact;
Var
  S : TJSSwitchStatement;
  C : TJSCaseElement;
begin
  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSSwitchStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('c');
  AssertWrite('switch ','switch (a) {case c: {}}',S);
end;

procedure TTestStatementWriter.TestSwitchStatementTwoElements;
Var
  S : TJSSwitchStatement;
  C : TJSCaseElement;
begin
//  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSSwitchStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('c');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('d');
  AssertWrite('switch ','switch (a) {'+sLineBreak+'case c:'+sLineBreak+'{'+sLineBreak+'}'+sLineBreak+'case d:'+sLineBreak+'{'+sLineBreak+'}'+sLineBreak+'}',S);
end;

procedure TTestStatementWriter.TestSwitchStatementTwoElementsCompact;

Var
  S : TJSSwitchStatement;
  C : TJSCaseElement;
begin
  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSSwitchStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('c');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('d');
  AssertWrite('switch ','switch (a) {case c: {} case d: {}}',S);
end;

procedure TTestStatementWriter.TestSwitchStatementTwoElementsDefault;
Var
  S : TJSSwitchStatement;
  C : TJSCaseElement;
begin
  S:=TJSSwitchStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('c');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('d');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  S.TheDefault:=C;
  AssertWrite('switch ','switch (a) {'+sLineBreak+'case c:'+sLineBreak+'{'+sLineBreak+'}'+sLineBreak+'case d:'+sLineBreak+'{'+sLineBreak+'}'+sLineBreak+'default:'+sLineBreak+'{'+sLineBreak+'}'+sLineBreak+'}',S);
end;

procedure TTestStatementWriter.TestSwitchStatementTwoElementsDefaultCompact;
Var
  S : TJSSwitchStatement;
  C : TJSCaseElement;
begin
  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSSwitchStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('c');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('d');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  S.TheDefault:=C;
  AssertWrite('switch ','switch (a) {case c: {} case d: {} default: {}}',S);
end;

procedure TTestStatementWriter.TestSwitchStatementTwoElementsOneEmpty;
Var
  S : TJSSwitchStatement;
  C : TJSCaseElement;
begin
  S:=TJSSwitchStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  C:=S.Cases.AddCase;
  C.Expr:=CreateIdent('c');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('d');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  S.TheDefault:=C;
  AssertWrite('switch ',
     'switch (a) {'+sLineBreak
    +'case c:'+sLineBreak
    +'case d:'+sLineBreak
    +'{'+sLineBreak
    +'}'+sLineBreak
    +'default:'+sLineBreak
    +'{'+sLineBreak
    +'}'+sLineBreak
    +'}',S);
end;

procedure TTestStatementWriter.TestSwitchStatementTwoElementsOneEmptyCompact;
Var
  S : TJSSwitchStatement;
  C : TJSCaseElement;
begin
  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSSwitchStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  C:=S.Cases.AddCase;
  C.Expr:=CreateIdent('c');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  C.Expr:=CreateIdent('d');
  C:=S.Cases.AddCase;
  C.Body:=TJSEmptyBlockStatement.Create(0,0);;
  S.TheDefault:=C;
  AssertWrite('switch ','switch (a) {case c: case d: {} default: {}}',S);
end;

procedure TTestStatementWriter.TestIfThen;
Var
  S : TJSIfStatement;

begin
//  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSIfStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  S.btrue:=TJSEmptyBlockStatement.Create(0,0);
  AssertWrite('if then','if (a) {'+sLineBreak+'}',S);
end;

procedure TTestStatementWriter.TestIfThenElse;
Var
  S : TJSIfStatement;

begin
//  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSIfStatement.Create(0,0);
  S.Cond:=CreateIdent('a');
  S.btrue:=TJSEmptyBlockStatement.Create(0,0);
  S.bfalse:=TJSEmptyBlockStatement.Create(0,0);
  AssertWrite('if then',
     'if (a) {'+sLineBreak
    +'} else {'+sLineBreak
    +'}',S);
end;

procedure TTestStatementWriter.TestStatementListEmpty;
Var
  S : TJSStatementList;

begin
//  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSStatementList.Create(0,0);
  AssertWrite('Statement list','{'+sLineBreak+'}',S);
end;

procedure TTestStatementWriter.TestStatementListEmptyCompact;
Var
  S : TJSStatementList;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSStatementList.Create(0,0);
  AssertWrite('Statement list','{}',S);
end;

procedure TTestStatementWriter.TestStatementListOneStatement;
Var
  S : TJSStatementList;
begin
//  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSStatementList.Create(0,0);
  S.A:=CreateAssignment(nil);
  AssertWrite('Statement list',
     '{'+sLineBreak
    +'a = b;'+sLineBreak
    +'}',S);
end;

procedure TTestStatementWriter.TestStatementListOneStatementCompact;

Var
  S : TJSStatementList;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSStatementList.Create(0,0);
  S.A:=CreateAssignment(nil);
  AssertWrite('Statement list','{a=b}',S);
end;

procedure TTestStatementWriter.TestStatementListTwoStatements;
Var
  S : TJSStatementList;

begin
//  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSStatementList.Create(0,0);
  S.A:=CreateAssignment(nil);
  S.B:=CreateAssignment(nil);
  AssertWrite('Statement list',
     '{'+sLineBreak
    +'a = b;'+sLineBreak
    +'a = b;'+sLineBreak
    +'}',S);
end;

procedure TTestStatementWriter.TestStatementListTwoStatementsCompact;
Var
  S : TJSStatementList;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSStatementList.Create(0,0);
  S.A:=CreateAssignment(nil);
  S.B:=CreateAssignment(nil);
  AssertWrite('Statement list','{a=b; a=b}',S);
end;

procedure TTestStatementWriter.TestStatementListTree4;
var
  S1, S11, S12: TJSStatementList;
begin
  Writer.Options:=[woUseUTF8];
  S1:=TJSStatementList.Create(0,0);
  S11:=TJSStatementList.Create(0,0);
  S1.A:=S11;
  S12:=TJSStatementList.Create(0,0);
  S1.B:=S12;
  S11.A:=CreateAssignment(nil);
  S11.B:=CreateAssignment(nil);
  S12.A:=CreateAssignment(nil);
  S12.B:=CreateAssignment(nil);
  AssertWrite('Statement list',
     '{'+sLineBreak
    +'a = b;'+sLineBreak
    +'a = b;'+sLineBreak
    +'a = b;'+sLineBreak
    +'a = b;'+sLineBreak
    +'}',S1);
end;

procedure TTestStatementWriter.TestStatementListFor;
Var
  S : TJSStatementList;
begin
  // Writer.Options:=[woCompact,woUseUTF8];
  S:=TJSStatementList.Create(0,0);
  S.A:=TJSForStatement.Create(0,0);
  TJSForStatement(S.A).Body:=TJSEmptyBlockStatement.Create(0,0);
  AssertWrite('Statement list',
     '{'+sLineBreak
    +'for (; ; ) {'+sLineBreak
    +'};'+sLineBreak
    +'}',S);
end;

procedure TTestStatementWriter.TestEmptyFunctionDef;

Var
  FD : TJSFunctionDeclarationStatement;

begin
  FD:=TJSFunctionDeclarationStatement.Create(0,0);
  FD.AFunction:=TJSFuncDef.Create;
  FD.AFunction.Name:='a';
  AssertWrite('Empty function',
     'function a() {'+sLineBreak
    +'}',FD);
end;

procedure TTestStatementWriter.TestEmptyFunctionDefCompact;

Var
  FD : TJSFunctionDeclarationStatement;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  FD:=TJSFunctionDeclarationStatement.Create(0,0);
  FD.AFunction:=TJSFuncDef.Create;
  FD.AFunction.Name:='a';
  AssertWrite('Empty function, compact','function a() {}',FD);
end;

procedure TTestStatementWriter.TestFunctionDefParams;
Var
  FD : TJSFunctionDeclarationStatement;

begin
//  Writer.Options:=[woCompact,woUseUTF8];
  FD:=TJSFunctionDeclarationStatement.Create(0,0);
  FD.AFunction:=TJSFuncDef.Create;
  FD.AFunction.Name:='a';
  FD.AFunction.Params.Add('b');
  FD.AFunction.Params.Add('c');
  FD.AFunction.Params.Add('d');

  AssertWrite('Empty function, 3 params',
     'function a(b, c, d) {'+sLineBreak
    +'}',FD);
end;

procedure TTestStatementWriter.TestFunctionDefParamsCompact;

Var
  FD : TJSFunctionDeclarationStatement;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  FD:=TJSFunctionDeclarationStatement.Create(0,0);
  FD.AFunction:=TJSFuncDef.Create;
  FD.AFunction.Name:='a';
  FD.AFunction.Params.Add('b');
  FD.AFunction.Params.Add('c');
  FD.AFunction.Params.Add('d');
  AssertWrite('Empty function, 3 params, compact','function a(b,c,d) {}',FD);
end;

procedure TTestStatementWriter.TestFunctionDefBody1;

Var
  FD : TJSFunctionDeclarationStatement;
  R : TJSReturnStatement;

begin
  Writer.IndentSize:=2;
  // Writer.Options:=[woCompact,woUseUTF8];
  FD:=TJSFunctionDeclarationStatement.Create(0,0);
  FD.AFunction:=TJSFuncDef.Create;
  FD.AFunction.Name:='a';
  FD.AFunction.Body:=TJSFunctionBody.Create(0,0);
  R:=TJSReturnStatement.Create(0,0);
  R.Expr:=CreateLiteral(0);
  FD.AFunction.Body.A:=R;
  AssertWrite('1 statement, ',
     'function a() {'+sLineBreak
    +'  return 0;'+sLineBreak
    +'}',FD);
end;

procedure TTestStatementWriter.TestFunctionDefBody1Compact;
Var
  FD : TJSFunctionDeclarationStatement;
  R : TJSReturnStatement;

begin
  Writer.IndentSize:=2;
  Writer.Options:=[woCompact,woUseUTF8];
  FD:=TJSFunctionDeclarationStatement.Create(0,0);
  FD.AFunction:=TJSFuncDef.Create;
  FD.AFunction.Name:='a';
  FD.AFunction.Body:=TJSFunctionBody.Create(0,0);
  R:=TJSReturnStatement.Create(0,0);
  R.Expr:=CreateLiteral(0);
  FD.AFunction.Body.A:=R;
  AssertWrite('1 statement, compact','function a() {return 0; }',FD);
end;

procedure TTestStatementWriter.TestFunctionDefBody2;
Var
  FD : TJSFunctionDeclarationStatement;
  R : TJSReturnStatement;
  L : TJSStatementList;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
//  Writer.Options:=[woCompact,woUseUTF8];
  FD:=TJSFunctionDeclarationStatement.Create(0,0);
  FD.AFunction:=TJSFuncDef.Create;
  FD.AFunction.Name:='a';
  FD.AFunction.Body:=TJSFunctionBody.Create(0,0);
  FD.AFunction.Params.Add('b');
  R:=TJSReturnStatement.Create(0,0);
  R.Expr:=CreateIdent('b');
  L:=TJSStatementList.Create(0,0);
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  L.A:=A;
  L.B:=R;
  FD.AFunction.Body.A:=L;
  AssertWrite('Function, 2 statements',
     'function a(b) {'+sLineBreak
    +'  b = b * 10;'+sLineBreak
    +'  return b;'+sLineBreak
    +'}',FD);
end;

procedure TTestStatementWriter.TestFunctionDefBody2Compact;
Var
  FD : TJSFunctionDeclarationStatement;
  R : TJSReturnStatement;
  L : TJSStatementList;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
  Writer.Options:=[woCompact,woUseUTF8];
  FD:=TJSFunctionDeclarationStatement.Create(0,0);
  FD.AFunction:=TJSFuncDef.Create;
  FD.AFunction.Name:='a';
  FD.AFunction.Body:=TJSFunctionBody.Create(0,0);
  FD.AFunction.Params.Add('b');
  R:=TJSReturnStatement.Create(0,0);
  R.Expr:=CreateIdent('b');
  L:=TJSStatementList.Create(0,0);
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  L.A:=A;
  L.B:=R;
  FD.AFunction.Body.A:=L;
  AssertWrite('Function, 2 statements, compact','function a(b) {b=b*10; return b}',FD);
end;

procedure TTestStatementWriter.TestTryCatch;

Var
  T : TJSTryCatchStatement;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
  T:=TJSTryCatchStatement.Create(0,0);
  T.Ident:='e';
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  T.Block:=A;
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  A.Expr:=CreateLiteral(1);
  T.BCatch:=A;
  AssertWrite('Try catch',
     'try {'+sLineBreak
    +'  b = b * 10'+sLineBreak
    +'} catch (e) {'+sLineBreak
    +'  b = 1'+sLineBreak
    +'}',T);
end;

procedure TTestStatementWriter.TestTryCatchCompact;
Var
  T : TJSTryCatchStatement;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
  Writer.Options:=[woCompact,woUseUTF8];
  T:=TJSTryCatchStatement.Create(0,0);
  T.Ident:='e';
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  T.Block:=A;
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  A.Expr:=CreateLiteral(1);
  T.BCatch:=A;
  AssertWrite('Try catch compact','try {b=b*10} catch (e) {b=1}',T);
end;

procedure TTestStatementWriter.TestTryFinally;

Var
  T : TJSTryFinallyStatement;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
  T:=TJSTryFinallyStatement.Create(0,0);
  T.Ident:='e';
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  T.Block:=A;
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  A.Expr:=CreateLiteral(1);
  T.BFinally:=A;
  AssertWrite('Try finally ',
    'try {'+sLineBreak
   +'  b = b * 10'+sLineBreak
   +'} finally {'+sLineBreak
   +'  b = 1'+sLineBreak
   +'}',T);
end;

procedure TTestStatementWriter.TestTryFinallyCompact;

Var
  T : TJSTryFinallyStatement;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
  Writer.Options:=[woCompact,woUseUTF8];
  T:=TJSTryFinallyStatement.Create(0,0);
  T.Ident:='e';
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  T.Block:=A;
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  A.Expr:=CreateLiteral(1);
  T.BFinally:=A;
  AssertWrite('Try finally compact','try {b=b*10} finally {b=1}',T);
end;

procedure TTestStatementWriter.TestTryCatchFinally;
Var
  T : TJSTryCatchFinallyStatement;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
  T:=TJSTryCatchFinallyStatement.Create(0,0);
  T.Ident:='e';
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  T.Block:=A;
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  A.Expr:=CreateLiteral(10);
  T.BCatch:=A;
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  A.Expr:=CreateLiteral(1);
  T.BFinally:=A;
  AssertWrite('Try finally ',
     'try {'+sLineBreak
    +'  b = b * 10'+sLineBreak
    +'} catch (e) {'+sLineBreak
    +'  b = 10'+sLineBreak
    +'} finally {'+sLineBreak
    +'  b = 1'+sLineBreak+'}',T);
end;

procedure TTestStatementWriter.TestTryCatchFinallyCompact;
Var
  T : TJSTryCatchFinallyStatement;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
  Writer.Options:=[woCompact,woUseUTF8];
  T:=TJSTryCatchFinallyStatement.Create(0,0);
  T.Ident:='e';
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  T.Block:=A;
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  A.Expr:=CreateLiteral(10);
  T.BCatch:=A;
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  A.Expr:=CreateLiteral(1);
  T.BFinally:=A;
  AssertWrite('Try finally ','try {b=b*10} catch (e) {b=10} finally {b=1}',T);
end;

procedure TTestStatementWriter.TestWith;
Var
  T : TJSWithStatement;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
//  Writer.Options:=[woCompact,woUseUTF8];
  T:=TJSWithStatement.Create(0,0);
  T.A:=CreateIdent('e');
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  T.B:=A;
  AssertWrite('With statement ','with (e)'+slineBreak+'  b = b * 10',T);
end;

procedure TTestStatementWriter.TestWithCompact;
Var
  T : TJSWithStatement;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
  Writer.Options:=[woCompact,woUseUTF8];
  T:=TJSWithStatement.Create(0,0);
  T.A:=CreateIdent('e');
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  T.B:=A;
  AssertWrite('With statement ','with (e) b=b*10',T);
end;

procedure TTestStatementWriter.TestSourceElements;
Var
  T : TJSSourceElements;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
  //  Writer.Options:=[woCompact,woUseUTF8];
  T:=TJSSourceElements.Create(0,0);
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  T.Statements.AddNode.Node:=A;
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('c');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('c');
  M.B:=CreateLiteral(2);
  A.Expr:=M;
  T.Statements.AddNode.Node:=A;
  AssertWrite('Statement lists ','b = b * 10;'+sLineBreak+'c = c * 2;'+sLineBreak,T);
end;

procedure TTestStatementWriter.TestSourceElementsCompact;
Var
  T : TJSSourceElements;
  A : TJSAssignStatement;
  M : TJSMultiplicativeExpressionMul;

begin
  Writer.IndentSize:=2;
  Writer.Options:=[woCompact,woUseUTF8];
  T:=TJSSourceElements.Create(0,0);
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('b');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('b');
  M.B:=CreateLiteral(10);
  A.Expr:=M;
  T.Statements.AddNode.Node:=A;
  A:=TJSSimpleAssignStatement.Create(0,0);
  A.LHS:=CreateIdent('c');
  M:=TJSMultiplicativeExpressionMul.Create(0,0);
  M.A:=CreateIdent('c');
  M.B:=CreateLiteral(2);
  A.Expr:=M;
  T.Statements.AddNode.Node:=A;
  AssertWrite('Statement lists compact','b=b*10; c=c*2;',T);
end;

{ ---------------------------------------------------------------------
  TTestLiteralWriter
  ---------------------------------------------------------------------}

Procedure TTestLiteralWriter.TestInteger;

begin
  AssertWrite('1','1',CreateLiteral(1));
end;

Procedure TTestLiteralWriter.TestBooleanTrue;

Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.Asboolean:=True;
  AssertWrite('true','true',L);
end;

Procedure TTestLiteralWriter.TestBooleanFalse;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.Asboolean:=False;
  AssertWrite('false','false',L);
end;

Procedure TTestLiteralWriter.TestUndefined;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  AssertWrite('undefined','undefined',L);
end;

Procedure TTestLiteralWriter.TestNull;

Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.IsNull:=True;
  AssertWrite('null','null',L);
end;

Procedure TTestLiteralWriter.TestString;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.AsString:='abcd';
  AssertWrite('abcd','"abcd"',L);
end;

Procedure TTestLiteralWriter.TestStringQuote;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.AsString:='ab"cd';
  AssertWrite('ab"cd','''ab"cd''',L);
end;

Procedure TTestLiteralWriter.TestStringBackslash;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.AsString:='ab\cd';
  AssertWrite('ab\cd','"ab\\cd"',L);
end;

Procedure TTestLiteralWriter.TestStringslash;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.AsString:='ab/cd';
  AssertWrite('ab/cd','"ab\/cd"',L);
end;

Procedure TTestLiteralWriter.TestStringsBack;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.AsString:='ab'#8'cd';
  AssertWrite('ab'#8'cd','"ab\bcd"',L);
end;

Procedure TTestLiteralWriter.TestStringsTab;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.AsString:='ab'#9'cd';
  AssertWrite('ab'#9'cd','"ab\tcd"',L);
end;

Procedure TTestLiteralWriter.TestStringsLineFeed;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.AsString:='ab'#10'cd';
  AssertWrite('ab'#10'cd','"ab\ncd"',L);
end;

Procedure TTestLiteralWriter.TestStringsFormFeed;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.AsString:='ab'#12'cd';
  AssertWrite('ab'#12'cd','"ab\fcd"',L);
end;

Procedure TTestLiteralWriter.TestStringsCarriageReturn;
Var
  L : TJSLiteral;
begin
  L:=TJSLiteral.Create(0,0,'');
  L.Value.AsString:='ab'#13'cd';
  AssertWrite('ab'#13'cd','"ab\rcd"',L);
end;

Procedure TTestLiteralWriter.TestArrayEmpty;

Var
  L : TJSArrayLiteral;

begin
  L:=TJSArrayLiteral.Create(0,0);
  AssertWrite('Empty array ','[]',L); // Always
end;

Procedure TTestLiteralWriter.TestArrayEmptyCompact;
Var
  L : TJSArrayLiteral;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  L:=TJSArrayLiteral.Create(0,0);
  AssertWrite('Empty array ','[]',L);
end;

Procedure TTestLiteralWriter.TestArrayOneElement;
Var
  L : TJSArrayLiteral;
  I : TJSLiteral;

begin
  L:=TJSArrayLiteral.Create(0,0);
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  L.Elements.AddElement.Expr:=I;
  AssertWrite('Empty array ','[1]',L);
end;

Procedure TTestLiteralWriter.TestArrayOneElementCompact;

Var
  L : TJSArrayLiteral;
  I : TJSLiteral;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  L:=TJSArrayLiteral.Create(0,0);
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  L.Elements.AddElement.Expr:=I;
  AssertWrite('Empty array ','[1]',L);
end;

Procedure TTestLiteralWriter.TestArrayOneElementIndent;
Var
  L : TJSArrayLiteral;
  I : TJSLiteral;

begin
  Writer.IndentSize:=2;
  L:=TJSArrayLiteral.Create(0,0);
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  L.Elements.AddElement.Expr:=I;
  AssertWrite('Empty array ','[1]',L);
end;

Procedure TTestLiteralWriter.TestArrayTwoElements;

Var
  L : TJSArrayLiteral;
  I : TJSLiteral;

begin
  L:=TJSArrayLiteral.Create(0,0);
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  L.Elements.AddElement.Expr:=I;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=2;
  L.Elements.AddElement.Expr:=I;
  AssertWrite('Empty array ','[1, 2]',L);
end;

Procedure TTestLiteralWriter.TestArrayTwoElementsCompact;
Var
  L : TJSArrayLiteral;
  I : TJSLiteral;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  L:=TJSArrayLiteral.Create(0,0);
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  L.Elements.AddElement.Expr:=I;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=2;
  L.Elements.AddElement.Expr:=I;
  AssertWrite('Empty array ','[1,2]',L);
end;

Procedure TTestLiteralWriter.TestArrayTwoElementsCompact2;
Var
  L : TJSArrayLiteral;
  I : TJSLiteral;

begin
  Writer.Options:=[woCompactArrayLiterals,woUseUTF8];
  L:=TJSArrayLiteral.Create(0,0);
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  L.Elements.AddElement.Expr:=I;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=2;
  L.Elements.AddElement.Expr:=I;
  AssertWrite('Empty array ','[1,2]',L);
end;

Procedure TTestLiteralWriter.TestArrayThreeElementsCompact;
Var
  L : TJSArrayLiteral;
  I : TJSLiteral;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  L:=TJSArrayLiteral.Create(0,0);
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  L.Elements.AddElement.Expr:=I;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=2;
  L.Elements.AddElement.Expr:=I;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=3;
  L.Elements.AddElement.Expr:=I;
  AssertWrite('Empty array ','[1,2,3]',L);
end;

Procedure TTestLiteralWriter.TestObjectEmpty;

Var
  L : TJSObjectLiteral;

begin
  L:=TJSObjectLiteral.Create(0,0);
  AssertWrite('Empty object ','{}',L); // Always
end;

Procedure TTestLiteralWriter.TestObjectEmptyCompact;
Var
  L : TJSObjectLiteral;

begin
  Writer.Options:=[woCompact,woUseUTF8];
  L:=TJSObjectLiteral.Create(0,0);
  AssertWrite('Empty object ','{}',L); // Always
end;

Procedure TTestLiteralWriter.TestObjectOneElement;

Var
  L : TJSObjectLiteral;
  E : TJSObjectLiteralElement;
  I : TJSLiteral;

begin
  L:=TJSObjectLiteral.Create(0,0);
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  E.Expr:=I;
  E.Name:='abc';
  AssertWrite('Empty object ','{'+slineBreak+'abc: 1'+sLineBreak+'}',L);
end;

Procedure TTestLiteralWriter.TestObjectOneElementCompact;

Var
  L : TJSObjectLiteral;
  E : TJSObjectLiteralElement;
  I : TJSLiteral;

begin
  L:=TJSObjectLiteral.Create(0,0);
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  E.Expr:=I;
  E.Name:='abc';
  Writer.Options:=[woCompact,woUseUTF8];
  AssertWrite('Empty object ','{abc: 1}',L);
end;

Procedure TTestLiteralWriter.TestObjectOneElementIndent;

Var
  L : TJSObjectLiteral;
  E : TJSObjectLiteralElement;
  I : TJSLiteral;
begin
  L:=TJSObjectLiteral.Create(0,0);
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  E.Expr:=I;
  E.Name:='abc';
  Writer.IndentSize:=2;
  AssertWrite('Empty object ','{'+slineBreak+'  abc: 1'+sLineBreak+'}',L);
end;

Procedure TTestLiteralWriter.TestObjectOneElementCompactQuoted;
Var
  L : TJSObjectLiteral;
  E : TJSObjectLiteralElement;
  I : TJSLiteral;

begin
  L:=TJSObjectLiteral.Create(0,0);
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  E.Expr:=I;
  E.Name:='abc';
  Writer.Options:=[woCompact,woUseUTF8,woQuoteElementNames];
  AssertWrite('Empty object ','{"abc": 1}',L);
end;

Procedure TTestLiteralWriter.TestObjectTwoElements;
Var
  L : TJSObjectLiteral;
  E : TJSObjectLiteralElement;
  I : TJSLiteral;

begin
  L:=TJSObjectLiteral.Create(0,0);
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  E.Expr:=I;
  E.Name:='abc';
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=2;
  E.Expr:=I;
  E.Name:='efg';
  AssertWrite('Empty object ','{'+slineBreak+'abc: 1,'+sLineBreak+'efg: 2'+slineBreak+'}',L);
end;

Procedure TTestLiteralWriter.TestObjectTwoElementCompact;

Var
  L : TJSObjectLiteral;
  E : TJSObjectLiteralElement;
  I : TJSLiteral;

begin
  L:=TJSObjectLiteral.Create(0,0);
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  E.Expr:=I;
  E.Name:='abc';
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=2;
  E.Expr:=I;
  E.Name:='efg';
  Writer.Options:=[woCompact,woUseUTF8];
  AssertWrite('Empty object ','{abc: 1, efg: 2}',L);
end;

Procedure TTestLiteralWriter.TestObjectTwoElementCompact2;
Var
  L : TJSObjectLiteral;
  E : TJSObjectLiteralElement;
  I : TJSLiteral;

begin
  L:=TJSObjectLiteral.Create(0,0);
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  E.Expr:=I;
  E.Name:='abc';
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=2;
  E.Expr:=I;
  E.Name:='efg';
  Writer.Options:=[woCompactObjectLiterals,woUseUTF8];
  AssertWrite('Empty object ','{abc: 1, efg: 2}',L);
end;

Procedure TTestLiteralWriter.TestObjectTwoElementCompactQuoted;
Var
  L : TJSObjectLiteral;
  E : TJSObjectLiteralElement;
  I : TJSLiteral;

begin
  L:=TJSObjectLiteral.Create(0,0);
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  E.Expr:=I;
  E.Name:='abc';
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=2;
  E.Expr:=I;
  E.Name:='efg';
  Writer.Options:=[woCompact,woUseUTF8,woQuoteElementNames];
  AssertWrite('Empty object ','{"abc": 1, "efg": 2}',L);
end;

Procedure TTestLiteralWriter.TestObjectThreeElementsCompact;
Var
  L : TJSObjectLiteral;
  E : TJSObjectLiteralElement;
  I : TJSLiteral;

begin
  L:=TJSObjectLiteral.Create(0,0);
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=1;
  E.Expr:=I;
  E.Name:='abc';
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=2;
  E.Expr:=I;
  E.Name:='efg';
  E:=L.Elements.AddElement;
  I:=TJSLiteral.Create(0,0);
  I.Value.AsNumber:=3;
  E.Expr:=I;
  E.Name:='hij';
  Writer.Options:=[woCompact,woUseUTF8];
  AssertWrite('Empty object ','{abc: 1, efg: 2, hij: 3}',L);
end;

{ ---------------------------------------------------------------------
  TTestJSWriter
  ---------------------------------------------------------------------}

procedure TTestJSWriter.SetUp;
begin
  FTextWriter:=TBufferWriter.Create(120);
  FWriter:=TJSWriter.Create(FTextWriter);
end;

procedure TTestJSWriter.TearDown;
begin
  FreeAndNil(FWriter);
  FreeAndNil(FTextWriter);
  FreeAndNil(FElement);
end;

Procedure TTestJSWriter.WriteElement(JS: TJSElement);
begin
  FElement:=JS;
  FWriter.WriteJS(JS);
end;

Procedure TTestJSWriter.AssertResult(Const Msg, Result: String);

Var
  S : AnsiString;
  p: Integer;
begin
  S:=FTextWriter.AsString;
  if S=Result then exit;
  p:=1;
  while (p<=length(S)) and (p<=length(Result)) and (S[p]=Result[p]) do inc(p);
  if p>length(S) then
    AssertEquals(Msg+' (actual too short)',Result,S)
  else if p>length(Result) then
    AssertEquals(Msg+' (actual too long)',Result,S)
  else
    AssertEquals(Msg+' (diff at '+IntToStr(p)+' "'+S[p]+'")',Result,S);
end;

Procedure TTestJSWriter.AssertResult(Const Msg: string; Result: UnicodeString);

Var
  S : UnicodeString;
  p: Integer;
begin
  S:=FTextWriter.AsUnicodeString;
  if S=Result then exit;
  p:=1;
  while (p<=length(S)) and (p<=length(Result)) and (S[p]=Result[p]) do inc(p);
  if p>length(S) then
    AssertEquals(Msg+' (actual too short)',String(Result),String(S))
  else if p>length(Result) then
    AssertEquals(Msg+' (actual too long)',String(Result),String(S))
  else
    AssertEquals(Msg+' (diff at '+IntToStr(p)+' "'+String(S[p])+'")',String(Result),String(S));
end;

Procedure TTestJSWriter.AssertWrite(Const Msg, Result: String;
  AElement: TJSElement);
begin
  WriteElement(AElement);
  AssertResult(Msg,Result);
end;

Procedure TTestJSWriter.AssertWrite(Const Msg: string; Result: UnicodeString;
  AElement: TJSElement);
begin
  WriteElement(AElement);
  AssertResult(Msg,Result);
end;

Function TTestJSWriter.CreateIdent(Const AName: String): TJSPrimaryExpressionIdent;
begin
  Result:=TJSPrimaryExpressionIdent.Create(0,0);
  Result.Name:=TJSString(AName);
end;

Function TTestJSWriter.CreateLiteral(Const AValue: TJSString): TJSLiteral;
begin
  Result:=TJSLiteral.Create(0,0);
  Result.Value.AsString:=Avalue;
end;

Function TTestJSWriter.CreateLiteral(Const AValue: Integer): TJSLiteral;
begin
  Result:=TJSLiteral.Create(0,0);
  Result.Value.AsNumber:=Avalue;
end;

Function TTestJSWriter.CreateLiteral(Const AValue: Boolean): TJSLiteral;
begin
  Result:=TJSLiteral.Create(0,0);
  Result.Value.AsBoolean:=Avalue;
end;

{ ---------------------------------------------------------------------
  TTestTestJSWriter
  ---------------------------------------------------------------------}

procedure TTestTestJSWriter.TestEmpty;
begin
  AssertNotNull('Have text writer',TextWriter);
  AssertNotNull('Have JS writer',Writer);
  AssertNull('Have no element',Element);
  AssertSame('Correct text writer for js writer',TextWriter,Writer.Writer);
  AssertEquals('No indent',0,Writer.IndentSize);
  if not (Writer.Options=[woUseUTF8]) then
    Fail('Options are not using UTF8');
end;


Initialization
  RegisterTests([TTestTestJSWriter,TTestLiteralWriter,TTestExpressionWriter,TTestStatementWriter]);
end.

