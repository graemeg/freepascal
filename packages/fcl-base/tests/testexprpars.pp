{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2008 Michael Van Canneyt.
    
    File which provides examples and all testcases for the expression parser.
    It needs fcl-fpcunit to work.
    
    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit testexprpars;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testutils, testregistry, math, fpexprpars;

type

  { TTestExpressionScanner }

  TTestExpressionScanner = class(TTestCase)
  Private
    FP : TFPExpressionScanner;
    FInvalidString : String;
    procedure DoInvalidNumber(AString: String);
    procedure TestIdentifier(const ASource, ATokenName: string);
    procedure TestInvalidNumber;
  protected
    procedure SetUp; override; 
    procedure TearDown; override;
    Procedure AssertEquals(Msg : string; AExpected, AActual : TTokenType); overload;
    Procedure TestString(Const AString : String; AToken : TTokenType);
  published
    procedure TestCreate;
    procedure TestSetSource;
    Procedure TestWhiteSpace;
    Procedure TestTokens;
    Procedure TestNumber;
    Procedure TestInvalidCharacter;
    Procedure TestUnterminatedString;
    Procedure TestQuotesInString;
    Procedure TestIdentifiers;
  end;

  { TMyFPExpressionParser }

  TMyFPExpressionParser = Class(TFPExpressionParser)
  Public
    Procedure BuildHashList;
    Property ExprNode;
    Property Scanner;
    Property Dirty;
  end;

  { TTestBaseParser }

  TTestBaseParser = class(TTestCase)
  private
    procedure DoCheck;
  Protected
    FDestroyCalled : Integer;
    FCheckNode : TFPExprNode;
    procedure AssertNodeType(Msg: String; AClass: TClass; ANode: TFPExprNode); overload;
    procedure AssertEquals(Msg: String; AResultType : TResultType; ANode: TFPExprNode); overload;
    procedure AssertEquals(Msg: String; AExpected,AActual : TResultType); overload;
    Function CreateBoolNode(ABoolean: Boolean) : TFPExprNode;
    Function CreateIntNode(AInteger: Integer) : TFPExprNode;
    Function CreateFloatNode(AFloat : TExprFloat) : TFPExprNode;
    Function CreateStringNode(Astring : String) : TFPExprNode;
    Function CreateDateTimeNode(ADateTime : TDateTime) : TFPExprNode;
    Procedure AssertNodeOK(FN : TFPExprNode);
    Procedure AssertNodeNotOK(Const Msg : String; FN : TFPExprNode);
    Procedure Setup; override;
  end;

  { TMyDestroyNode }

  TMyDestroyNode = Class(TFPConstExpression)
    FTest : TTestBaseParser;
  Public
    Constructor CreateTest(ATest : TTestBaseParser);
    Destructor Destroy; override;
  end;

  { TTestDestroyNode }

  TTestDestroyNode =  Class(TTestBaseParser)
  Published
    Procedure TestDestroy;
  end;

  { TTestConstExprNode }

  TTestConstExprNode = Class(TTestBaseParser)
  private
    FN : TFPConstExpression;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    procedure TestCreateFloat;
    procedure TestCreateBoolean;
    procedure TestCreateDateTime;
    procedure TestCreateString;
  end;

  { TTestNegateExprNode }

  TTestNegateExprNode = Class(TTestBaseParser)
  Private
    FN : TFPNegateOperation;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    procedure TestCreateFloat;
    procedure TestCreateOther1;
    procedure TestCreateOther2;
    Procedure TestDestroy;
  end;

  { TTestBinaryAndNode }

  TTestBinaryAndNode = Class(TTestBaseParser)
  Private
    FN : TFPBinaryAndOperation;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    procedure TestCreateBoolean;
    procedure TestCreateBooleanInteger;
    procedure TestCreateString;
    procedure TestCreateFloat;
    procedure TestCreateDateTime;
    Procedure TestDestroy;
  end;

  { TTestNotNode }

  TTestNotNode = Class(TTestBaseParser)
  Private
    FN : TFPNotNode;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    procedure TestCreateBoolean;
    procedure TestCreateString;
    procedure TestCreateFloat;
    procedure TestCreateDateTime;
    Procedure TestDestroy;
  end;

  { TTestBinaryOrNode }

  TTestBinaryOrNode = Class(TTestBaseParser)
  Private
    FN : TFPBinaryOrOperation;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    procedure TestCreateBoolean;
    procedure TestCreateBooleanInteger;
    procedure TestCreateString;
    procedure TestCreateFloat;
    procedure TestCreateDateTime;
    Procedure TestDestroy;
  end;

  { TTestBinaryXOrNode }

  TTestBinaryXOrNode = Class(TTestBaseParser)
  Private
    FN : TFPBinaryXOrOperation;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    procedure TestCreateBoolean;
    procedure TestCreateBooleanInteger;
    procedure TestCreateString;
    procedure TestCreateFloat;
    procedure TestCreateDateTime;
    Procedure TestDestroy;
  end;

  { TTestIfOperation }

  TTestIfOperation = Class(TTestBaseParser)
  Private
    FN : TIfOperation;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    procedure TestCreateBoolean;
    procedure TestCreateBoolean2;
    procedure TestCreateString;
    procedure TestCreateFloat;
    procedure TestCreateDateTime;
    procedure TestCreateBooleanInteger;
    procedure TestCreateBooleanInteger2;
    procedure TestCreateBooleanString;
    procedure TestCreateBooleanString2;
    procedure TestCreateBooleanDateTime;
    procedure TestCreateBooleanDateTime2;
    Procedure TestDestroy;
  end;

  { TTestCaseOperation }

  TTestCaseOperation = Class(TTestBaseParser)
  Private
    FN : TCaseOperation;
  Protected
    Function CreateArgs(Args : Array of Const) : TExprArgumentArray;
    Procedure TearDown; override;
  Published
    Procedure TestCreateOne;
    procedure TestCreateTwo;
    procedure TestCreateThree;
    procedure TestCreateOdd;
    procedure TestCreateNoExpression;
    procedure TestCreateWrongLabel;
    procedure TestCreateWrongValue;
    procedure TestIntegerTag;
    procedure TestIntegerTagDefault;
    procedure TestStringTag;
    procedure TestStringTagDefault;
    procedure TestFloatTag;
    procedure TestFloatTagDefault;
    procedure TestBooleanTag;
    procedure TestBooleanTagDefault;
    procedure TestDateTimeTag;
    procedure TestDateTimeTagDefault;
    procedure TestIntegerValue;
    procedure TestIntegerValueDefault;
    procedure TestStringValue;
    procedure TestStringValueDefault;
    procedure TestFloatValue;
    procedure TestFloatValueDefault;
    procedure TestBooleanValue;
    procedure TestBooleanValueDefault;
    procedure TestDateTimeValue;
    procedure TestDateTimeValueDefault;
    Procedure TestDestroy;
  end;

  { TTestBooleanNode }

  TTestBooleanNode = Class(TTestBaseParser)
  Protected
    Procedure TestNode(B : TFPBooleanResultOperation; AResult : Boolean);
  end;

  { TTestEqualNode }

  TTestEqualNode = Class(TTestBooleanNode)
  Private
    FN : TFPBooleanResultOperation;
  Protected
    Procedure TearDown; override;
    Class Function NodeClass : TFPBooleanResultOperationClass; virtual;
    Class Function ExpectedResult : Boolean; virtual;
    Class Function OperatorString : String; virtual;
  Published
    Procedure TestCreateIntegerEqual;
    procedure TestCreateIntegerUnEqual;
    Procedure TestCreateFloatEqual;
    procedure TestCreateFloatUnEqual;
    Procedure TestCreateStringEqual;
    procedure TestCreateStringUnEqual;
    Procedure TestCreateBooleanEqual;
    procedure TestCreateBooleanUnEqual;
    Procedure TestCreateDateTimeEqual;
    procedure TestCreateDateTimeUnEqual;
    Procedure TestDestroy;
    Procedure TestWrongTypes1;
    procedure TestWrongTypes2;
    procedure TestWrongTypes3;
    procedure TestWrongTypes4;
    procedure TestWrongTypes5;
    Procedure TestAsString;
  end;

  { TTestUnEqualNode }

  TTestUnEqualNode = Class(TTestEqualNode)
  Protected
    Class Function NodeClass : TFPBooleanResultOperationClass; override;
    Class Function ExpectedResult : Boolean; override;
    Class Function OperatorString : String; override;
  end;

  { TTestLessThanNode }

  TTestLessThanNode = Class(TTestBooleanNode)
  Private
    FN : TFPBooleanResultOperation;
  Protected
    Class Function NodeClass : TFPBooleanResultOperationClass; virtual;
    Class Function Larger : Boolean; virtual;
    Class Function AllowEqual : Boolean; virtual;
    Class Function OperatorString : String; virtual;
    Procedure TearDown; override;
  Published
    Procedure TestCreateIntegerEqual;
    procedure TestCreateIntegerSmaller;
    procedure TestCreateIntegerLarger;
    Procedure TestCreateFloatEqual;
    procedure TestCreateFloatSmaller;
    procedure TestCreateFloatLarger;
    Procedure TestCreateDateTimeEqual;
    procedure TestCreateDateTimeSmaller;
    procedure TestCreateDateTimeLarger;
    Procedure TestCreateStringEqual;
    procedure TestCreateStringSmaller;
    procedure TestCreateStringLarger;
    Procedure TestWrongTypes1;
    procedure TestWrongTypes2;
    procedure TestWrongTypes3;
    procedure TestWrongTypes4;
    procedure TestWrongTypes5;
    Procedure TestNoBoolean1;
    Procedure TestNoBoolean2;
    Procedure TestNoBoolean3;
    Procedure TestAsString;
  end;

  { TTestLessThanEqualNode }

  TTestLessThanEqualNode = Class(TTestLessThanNode)
  protected
    Class Function NodeClass : TFPBooleanResultOperationClass; override;
    Class Function AllowEqual : Boolean; override;
    Class Function OperatorString : String; override;
  end;

  { TTestLargerThanNode }

  TTestLargerThanNode = Class(TTestLessThanNode)
  protected
    Class Function NodeClass : TFPBooleanResultOperationClass; override;
    Class Function Larger : Boolean; override;
    Class Function OperatorString : String; override;
  end;
  { TTestLargerThanEqualNode }

  TTestLargerThanEqualNode = Class(TTestLargerThanNode)
  protected
    Class Function NodeClass : TFPBooleanResultOperationClass; override;
    Class Function AllowEqual : Boolean; override;
    Class Function OperatorString : String; override;
  end;

  { TTestAddNode }

  TTestAddNode = Class(TTestBaseParser)
  Private
    FN : TFPAddOperation;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    Procedure TestCreateFloat;
    Procedure TestCreateDateTime;
    Procedure TestCreateString;
    Procedure TestCreateBoolean;
    Procedure TestDestroy;
    Procedure TestAsString;
  end;

  { TTestSubtractNode }

  TTestSubtractNode = Class(TTestBaseParser)
  Private
    FN : TFPSubtractOperation;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    Procedure TestCreateFloat;
    Procedure TestCreateDateTime;
    Procedure TestCreateString;
    Procedure TestCreateBoolean;
    Procedure TestDestroy;
    Procedure TestAsString;
  end;

  { TTestMultiplyNode }

  TTestMultiplyNode = Class(TTestBaseParser)
  Private
    FN : TFPMultiplyOperation;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    Procedure TestCreateFloat;
    Procedure TestCreateDateTime;
    Procedure TestCreateString;
    Procedure TestCreateBoolean;
    Procedure TestDestroy;
    Procedure TestAsString;
  end;

  { TTestPowerNode }

  TTestPowerNode = Class(TTestBaseParser)
  Private
    FN : TFPPowerOperation;
    FE : TFPExpressionParser;
  Protected
    Procedure Setup; override;
    Procedure TearDown; override;
    procedure Calc(AExpr: String; Expected: Double = NaN);
  Published
    Procedure TestCreateInteger;
    Procedure TestCreateFloat;
    Procedure TestCreateDateTime;
    Procedure TestCreateString;
    Procedure TestCreateBoolean;
    Procedure TestDestroy;
    Procedure TestAsString;
    Procedure TestCalc;
  end;

  { TTestDivideNode }

  TTestDivideNode = Class(TTestBaseParser)
  Private
    FN : TFPDivideOperation;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    Procedure TestCreateFloat;
    Procedure TestCreateDateTime;
    Procedure TestCreateString;
    Procedure TestCreateBoolean;
    Procedure TestDestroy;
    Procedure TestAsString;
  end;

  { TTestIntToFloatNode }

  TTestIntToFloatNode = Class(TTestBaseParser)
  Private
    FN : TIntToFloatNode;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    Procedure TestCreateFloat;
    Procedure TestDestroy;
    Procedure TestAsString;
  end;

  { TTestIntToDateTimeNode }

  TTestIntToDateTimeNode = Class(TTestBaseParser)
  Private
    FN : TIntToDateTimeNode;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    Procedure TestCreateFloat;
    Procedure TestDestroy;
    Procedure TestAsString;
  end;

  { TTestFloatToDateTimeNode }

  TTestFloatToDateTimeNode = Class(TTestBaseParser)
  Private
    FN : TFloatToDateTimeNode;
  Protected
    Procedure TearDown; override;
  Published
    Procedure TestCreateInteger;
    Procedure TestCreateFloat;
    Procedure TestDestroy;
    Procedure TestAsString;
  end;

  { TTestExpressionParser }
  TTestExpressionParser = class(TTestBaseParser)
  Private
    FP : TMyFPExpressionParser;
    FTestExpr : String;
    procedure DoAddInteger(var Result: TFPExpressionResult;
      const Args: TExprParameterArray);
    procedure DoDeleteString(var Result: TFPExpressionResult;
      const Args: TExprParameterArray);
    procedure DoEchoBoolean(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
    procedure DoEchoDate(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
    procedure DoEchoFloat(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
    procedure DoEchoCurrency(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
    procedure DoEchoInteger(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
    procedure DoEchoString(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
    procedure DoGetDate(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
    procedure DoParse;
    procedure TestParser(AExpr: string);
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    Procedure AssertLeftRight(N : TFPExprNode; LeftClass,RightClass : TClass);
    Procedure AssertOperand(N : TFPExprNode; OperandClass : TClass);
    Procedure AssertResultType(RT : TResultType);
    Procedure AssertResult(F : TExprFloat);
    Procedure AssertCurrencyResult(C : Currency);
    Procedure AssertResult(I : Int64);
    Procedure AssertResult(S : String);
    Procedure AssertResult(B : Boolean);
    Procedure AssertDateTimeResult(D : TDateTime);
  end;

  { TTestParserExpressions }

  TTestParserExpressions = Class(TTestExpressionParser)
  private
  Published
    Procedure TestCreate;
    Procedure TestNumberValues;
    Procedure TestSimpleNodeFloat;
    procedure TestSimpleNodeInteger;
    procedure TestSimpleNodeBooleanTrue;
    procedure TestSimpleNodeBooleanFalse;
    procedure TestSimpleNodeString;
    procedure TestSimpleNegativeInteger;
    procedure TestSimpleNegativeFloat;
    procedure TestSimpleAddInteger;
    procedure TestSimpleAddFloat;
    procedure TestSimpleAddIntegerFloat;
    procedure TestSimpleAddFloatInteger;
    procedure TestSimpleAddString;
    procedure TestSimpleSubtractInteger;
    procedure TestSimpleSubtractFloat;
    procedure TestSimpleSubtractIntegerFloat;
    procedure TestSimpleSubtractFloatInteger;
    procedure TestSimpleMultiplyFloat;
    procedure TestSimpleMultiplyInteger;
    procedure TestSimpleDivideFloat;
    procedure TestSimpleDivideInteger;
    procedure TestSimpleBooleanAnd;
    procedure TestSimpleIntegerAnd;
    procedure TestSimpleBooleanOr;
    procedure TestSimpleIntegerOr;
    procedure TestSimpleBooleanNot;
    procedure TestSimpleIntegerNot;
    procedure TestSimpleAddSeries;
    procedure TestSimpleMultiplySeries;
    procedure TestSimpleAddMultiplySeries;
    procedure TestSimpleAddAndSeries;
    procedure TestSimpleAddOrSeries;
    procedure TestSimpleOrNotSeries;
    procedure TestSimpleAndNotSeries;
    procedure TestDoubleAddMultiplySeries;
    procedure TestDoubleSubtractMultiplySeries;
    procedure TestSimpleIfInteger;
    procedure TestSimpleIfString;
    procedure TestSimpleIfFloat;
    procedure TestSimpleIfBoolean;
    procedure TestSimpleIfDateTime;
    procedure TestSimpleIfOperation;
    procedure TestSimpleBrackets;
    procedure TestSimpleBrackets2;
    procedure TestSimpleBracketsLeft;
    procedure TestSimpleBracketsRight;
    procedure TestSimpleBracketsDouble;
  end;

  TTestParserBooleanOperations = Class(TTestExpressionParser)
  Published
    Procedure TestEqualInteger;
    procedure TestUnEqualInteger;
    procedure TestEqualFloat;
    procedure TestEqualFloat2;
    procedure TestUnEqualFloat;
    procedure TestEqualString;
    procedure TestEqualString2;
    procedure TestUnEqualString;
    procedure TestUnEqualString2;
    Procedure TestEqualBoolean;
    procedure TestUnEqualBoolean;
    procedure TestLessThanInteger;
    procedure TestLessThanInteger2;
    procedure TestLessThanEqualInteger;
    procedure TestLessThanEqualInteger2;
    procedure TestLessThanFloat;
    procedure TestLessThanFloat2;
    procedure TestLessThanEqualFloat;
    procedure TestLessThanEqualFloat2;
    procedure TestLessThanString;
    procedure TestLessThanString2;
    procedure TestLessThanEqualString;
    procedure TestLessThanEqualString2;
    procedure TestGreaterThanInteger;
    procedure TestGreaterThanInteger2;
    procedure TestGreaterThanEqualInteger;
    procedure TestGreaterThanEqualInteger2;
    procedure TestGreaterThanFloat;
    procedure TestGreaterThanFloat2;
    procedure TestGreaterThanEqualFloat;
    procedure TestGreaterThanEqualFloat2;
    procedure TestGreaterThanString;
    procedure TestGreaterThanString2;
    procedure TestGreaterThanEqualString;
    procedure TestGreaterThanEqualString2;
    procedure EqualAndSeries;
    procedure EqualAndSeries2;
    procedure EqualOrSeries;
    procedure EqualOrSeries2;
    procedure UnEqualAndSeries;
    procedure UnEqualAndSeries2;
    procedure UnEqualOrSeries;
    procedure UnEqualOrSeries2;
    procedure LessThanAndSeries;
    procedure LessThanAndSeries2;
    procedure LessThanOrSeries;
    procedure LessThanOrSeries2;
    procedure GreaterThanAndSeries;
    procedure GreaterThanAndSeries2;
    procedure GreaterThanOrSeries;
    procedure GreaterThanOrSeries2;
    procedure LessThanEqualAndSeries;
    procedure LessThanEqualAndSeries2;
    procedure LessThanEqualOrSeries;
    procedure LessThanEqualOrSeries2;
    procedure GreaterThanEqualAndSeries;
    procedure GreaterThanEqualAndSeries2;
    procedure GreaterThanEqualOrSeries;
    procedure GreaterThanEqualOrSeries2;
  end;

  { TTestParserOperands }

  TTestParserOperands = Class(TTestExpressionParser)
  private
  Published
    Procedure MissingOperand1;
    procedure MissingOperand2;
    procedure MissingOperand3;
    procedure MissingOperand4;
    procedure MissingOperand5;
    procedure MissingOperand6;
    procedure MissingOperand7;
    procedure MissingOperand8;
    procedure MissingOperand9;
    procedure MissingOperand10;
    procedure MissingOperand11;
    procedure MissingOperand12;
    procedure MissingOperand13;
    procedure MissingOperand14;
    procedure MissingOperand15;
    procedure MissingOperand16;
    procedure MissingOperand17;
    procedure MissingOperand18;
    procedure MissingOperand19;
    procedure MissingOperand20;
    procedure MissingOperand21;
    procedure MissingBracket1;
    procedure MissingBracket2;
    procedure MissingBracket3;
    procedure MissingBracket4;
    procedure MissingBracket5;
    procedure MissingBracket6;
    procedure MissingBracket7;
    procedure MissingArgument1;
    procedure MissingArgument2;
    procedure MissingArgument3;
    procedure MissingArgument4;
    procedure MissingArgument5;
    procedure MissingArgument6;
    procedure MissingArgument7;
  end;

  { TTestParserTypeMatch }

  TTestParserTypeMatch = Class(TTestExpressionParser)
  Private
    Procedure AccessString;
    Procedure AccessInteger;
    Procedure AccessFloat;
    Procedure AccessDateTime;
    Procedure AccessBoolean;
  Published
    Procedure TestTypeMismatch1;
    procedure TestTypeMismatch2;
    procedure TestTypeMismatch3;
    procedure TestTypeMismatch4;
    procedure TestTypeMismatch5;
    procedure TestTypeMismatch6;
    procedure TestTypeMismatch7;
    procedure TestTypeMismatch8;
    procedure TestTypeMismatch9;
    procedure TestTypeMismatch10;
    procedure TestTypeMismatch11;
    procedure TestTypeMismatch12;
    procedure TestTypeMismatch13;
    procedure TestTypeMismatch14;
    procedure TestTypeMismatch15;
    procedure TestTypeMismatch16;
    procedure TestTypeMismatch17;
    procedure TestTypeMismatch18;
    procedure TestTypeMismatch19;
    procedure TestTypeMismatch20;
    procedure TestTypeMismatch21;
    procedure TestTypeMismatch22;
    procedure TestTypeMismatch23;
    procedure TestTypeMismatch24;
  end;

  { TTestParserVariables }

  TTestParserVariables = Class(TTestExpressionParser)
  private
    FAsWrongType : TResultType;
    FEventName: String;
    FBoolValue : Boolean;
    FTest33 : TFPExprIdentifierDef;
    procedure DoGetBooleanVar(var Res: TFPExpressionResult; ConstRef AName: ShortString);
    procedure DoGetBooleanVarWrong(var Res: TFPExpressionResult; ConstRef AName: ShortString);
    procedure TestAccess(Skip: TResultType);
    procedure TestAccess(Skip: TResultTypes);
  Protected
    procedure DoTestVariable33;
    procedure AddVariabletwice;
    procedure UnknownVariable;
    Procedure ReadWrongType;
    procedure WriteWrongType;
    Procedure DoDummy(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
  Published
    Procedure TestVariableAssign;
    Procedure TestVariableAssignAgain;
    Procedure TestVariable1;
    procedure TestVariable2;
    procedure TestVariable3;
    procedure TestVariable4;
    procedure TestVariable5;
    procedure TestVariable6;
    procedure TestVariable7;
    procedure TestVariable8;
    procedure TestVariable9;
    procedure TestVariable10;
    procedure TestVariable11;
    procedure TestVariable12;
    procedure TestVariable13;
    procedure TestVariable14;
    procedure TestVariable15;
    procedure TestVariable16;
    procedure TestVariable17;
    procedure TestVariable18;
    procedure TestVariable19;
    procedure TestVariable20;
    procedure TestVariable21;
    procedure TestVariable22;
    procedure TestVariable23;
    procedure TestVariable24;
    procedure TestVariable25;
    procedure TestVariable26;
    procedure TestVariable27;
    procedure TestVariable28;
    procedure TestVariable29;
    procedure TestVariable30;
    procedure TestVariable31;
    procedure TestVariable32;
    procedure TestVariable33;
    procedure TestVariable34;
    procedure TestVariable35;
    procedure TestVariable36;
  end;

  { TTestParserFunctions }
  TTestParserFunctions = Class(TTestExpressionParser)
  private
    FAccessAs : TResultType;
    procedure ExprAveOf(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure ExprMaxOf(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure ExprMinOf(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure ExprStdDevOf(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    procedure ExprSumOf(var Result: TFPExpressionResult; const Args: TExprParameterArray);
    Procedure TryRead;
    procedure TryWrite;
  Published
    Procedure TestFunction1;
    procedure TestFunction2;
    procedure TestFunction3;
    procedure TestFunction4;
    procedure TestFunction5;
    procedure TestFunction6;
    procedure TestFunction7;
    procedure TestFunction8;
    procedure TestFunction9;
    procedure TestFunction10;
    procedure TestFunction11;
    procedure TestFunction12;
    procedure TestFunction13;
    procedure TestFunction14;
    procedure TestFunction15;
    procedure TestFunction16;
    procedure TestFunction17;
    procedure TestFunction18;
    procedure TestFunction19;
    procedure TestFunction20;
    procedure TestFunction21;
    procedure TestFunction22;
    procedure TestFunction23;
    procedure TestFunction24;
    procedure TestFunction25;
    procedure TestFunction26;
    procedure TestFunction27;
    procedure TestFunction28;
    procedure TestFunction29;
    procedure TestFunction30;
    procedure TestFunction31;
    procedure TestFunction32;
    procedure TestFunction33;
    procedure TestVarArgs1;
    procedure TestVarArgs2;
    procedure TestVarArgs3;
    procedure TestVarArgs4;
    procedure TestVarArgs5;
  end;




  { TAggregateNode }

  TAggregateNode = Class(TFPExprNode)
  Public
    InitCount : Integer;
    UpdateCount : Integer;
    Class Function IsAggregate: Boolean; override;
    Function NodeType: TResultType; override;
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
    procedure GetNodeValue(var Result: TFPExpressionResult); override;
  end;

  { TTestParserAggregate }

  TTestParserAggregate = Class(TTestExpressionParser)
  private
    FVarValue : Integer;
    FLeft : TAggregateNode;
    FRight : TAggregateNode;
    FFunction : TFPExprIdentifierDef;
    FFunction2 : TFPExprIdentifierDef;
  Protected
    Procedure Setup; override;
    Procedure TearDown; override;
  public
    procedure GetVar(var Result: TFPExpressionResult; ConstRef AName: ShortString);
  Published
    Procedure TestIsAggregate;
    Procedure TestHasAggregate;
    Procedure TestBinaryAggregate;
    Procedure TestUnaryAggregate;
    Procedure TestCountAggregate;
    Procedure TestSumAggregate;
    Procedure TestSumAggregate2;
    Procedure TestSumAggregate3;
    Procedure TestAvgAggregate;
    Procedure TestAvgAggregate2;
    Procedure TestAvgAggregate3;
  end;
  { TTestBuiltinsManager }

  TTestBuiltinsManager = Class(TTestExpressionParser)
  private
    FM : TExprBuiltInManager;
  Protected
    procedure Setup; override;
    procedure Teardown; override;
  Published
    procedure TestCreate;
    procedure TestVariable1;
    procedure TestVariable2;
    procedure TestVariable3;
    procedure TestVariable4;
    procedure TestVariable5;
    procedure TestVariable6;
    procedure TestVariable7;
    procedure TestFunction1;
    procedure TestFunction2;
    procedure TestDelete;
    procedure TestRemove;
  end;

  TTestBuiltins = Class(TTestExpressionParser)
  private
    FValue : Integer;
    FM : TExprBuiltInManager;
    FExpr : String;
    procedure DoAverage(Var Result : TFPExpressionResult; ConstRef AName : ShortString);
    procedure DoSeries(var Result: TFPExpressionResult; ConstRef AName: ShortString);
  Protected
    procedure Setup; override;
    procedure Teardown; override;
    Procedure SetExpression(Const AExpression : String);
    Procedure AssertVariable(Const ADefinition : String; AResultType : TResultType);
    Procedure AssertFunction(Const ADefinition,AResultType,ArgumentTypes : String; ACategory : TBuiltinCategory);
    procedure AssertExpression(Const AExpression : String; AResult : Int64);
    procedure AssertExpression(Const AExpression : String; Const AResult : String);
    procedure AssertExpression(Const AExpression : String; Const AResult : TExprFloat);
    procedure AssertExpression(Const AExpression : String; Const AResult : Boolean);
    procedure AssertDateTimeExpression(Const AExpression : String; Const AResult : TDateTime);
    procedure AssertAggregateExpression(Const AExpression : String; AResult : Int64; AUpdateCount : integer);
    procedure AssertAggregateExpression(Const AExpression : String; AResult : TExprFloat; AUpdateCount : integer);
    procedure AssertAggregateCurrExpression(Const AExpression : String; AResult : Currency; AUpdateCount : integer);
  Published
    procedure TestRegister;
    Procedure TestVariablepi;
    Procedure TestFunctioncos;
    Procedure TestFunctionsin;
    Procedure TestFunctionarctan;
    Procedure TestFunctionabs;
    Procedure TestFunctionsqr;
    Procedure TestFunctionsqrt;
    Procedure TestFunctionexp;
    Procedure TestFunctionln;
    Procedure TestFunctionlog;
    Procedure TestFunctionfrac;
    Procedure TestFunctionint;
    Procedure TestFunctionround;
    Procedure TestFunctiontrunc;
    Procedure TestFunctionlength;
    Procedure TestFunctioncopy;
    Procedure TestFunctiondelete;
    Procedure TestFunctionpos;
    Procedure TestFunctionlowercase;
    Procedure TestFunctionuppercase;
    Procedure TestFunctionstringreplace;
    Procedure TestFunctioncomparetext;
    Procedure TestFunctiondate;
    Procedure TestFunctiontime;
    Procedure TestFunctionnow;
    Procedure TestFunctiondayofweek;
    Procedure TestFunctionextractyear;
    Procedure TestFunctionextractmonth;
    Procedure TestFunctionextractday;
    Procedure TestFunctionextracthour;
    Procedure TestFunctionextractmin;
    Procedure TestFunctionextractsec;
    Procedure TestFunctionextractmsec;
    Procedure TestFunctionencodedate;
    Procedure TestFunctionencodetime;
    Procedure TestFunctionencodedatetime;
    Procedure TestFunctionshortdayname;
    Procedure TestFunctionshortmonthname;
    Procedure TestFunctionlongdayname;
    Procedure TestFunctionlongmonthname;
    Procedure TestFunctionformatdatetime;
    Procedure TestFunctionshl;
    Procedure TestFunctionshr;
    Procedure TestFunctionIFS;
    Procedure TestFunctionIFF;
    Procedure TestFunctionIFD;
    Procedure TestFunctionIFI;
    Procedure TestFunctioninttostr;
    Procedure TestFunctionstrtoint;
    Procedure TestFunctionstrtointdef;
    Procedure TestFunctionfloattostr;
    Procedure TestFunctionstrtofloat;
    Procedure TestFunctionstrtofloatdef;
    Procedure TestFunctionbooltostr;
    Procedure TestFunctionstrtobool;
    Procedure TestFunctionstrtobooldef;
    Procedure TestFunctiondatetostr;
    Procedure TestFunctiontimetostr;
    Procedure TestFunctionstrtodate;
    Procedure TestFunctionstrtodatedef;
    Procedure TestFunctionstrtotime;
    Procedure TestFunctionstrtotimedef;
    Procedure TestFunctionstrtodatetime;
    Procedure TestFunctionstrtodatetimedef;
    Procedure TestFunctionAggregateSum;
    Procedure TestFunctionAggregateSumFloat;
    Procedure TestFunctionAggregateSumCurrency;
    Procedure TestFunctionAggregateCount;
    Procedure TestFunctionAggregateAvg;
    Procedure TestFunctionAggregateMin;
    Procedure TestFunctionAggregateMax;
  end;

implementation

uses typinfo;

{ TTestParserAggregate }

procedure TTestParserAggregate.Setup;
begin
  inherited Setup;
  FVarValue:=0;
  FFunction:=TFPExprIdentifierDef.Create(Nil);
  FFunction.Name:='Count';
  FFunction2:=TFPExprIdentifierDef.Create(Nil);
  FFunction2.Name:='MyVar';
  FFunction2.ResultType:=rtInteger;
  FFunction2.IdentifierType:=itVariable;
  FFunction2.OnGetVariableValue:=@GetVar;
  FLeft:=TAggregateNode.Create;
  FRight:=TAggregateNode.Create;
end;

procedure TTestParserAggregate.TearDown;
begin
  FreeAndNil(FFunction);
  FreeAndNil(FLeft);
  FreeAndNil(FRight);
  inherited TearDown;
end;

procedure TTestParserAggregate.GetVar(var Result: TFPExpressionResult; ConstRef
  AName: ShortString);
begin
  Result.ResultType:=FFunction2.ResultType;
  Case Result.ResultType of
    rtInteger : Result.ResInteger:=FVarValue;
    rtFloat : Result.ResFloat:=FVarValue / 2;
    rtCurrency : Result.ResCurrency:=FVarValue / 2;
  end;
end;

procedure TTestParserAggregate.TestIsAggregate;
begin
  AssertEquals('ExprNode',False,TFPExprNode.IsAggregate);
  AssertEquals('TAggregateExpr',True,TAggregateExpr.IsAggregate);
  AssertEquals('TAggregateExpr',False,TFPBinaryOperation.IsAggregate);
end;

procedure TTestParserAggregate.TestHasAggregate;

Var
  N :  TFPExprNode;

begin
  N:=TFPExprNode.Create;
  try
    AssertEquals('ExprNode',False,N.HasAggregate);
  finally
    N.Free;
  end;
  N:=TAggregateExpr.Create;
  try
    AssertEquals('ExprNode',True,N.HasAggregate);
  finally
    N.Free;
  end;
end;

procedure TTestParserAggregate.TestBinaryAggregate;

Var
  B :  TFPBinaryOperation;

begin
  B:=TFPBinaryOperation.Create(Fleft,TFPConstExpression.CreateInteger(1));
  try
    FLeft:=Nil;
    AssertEquals('Binary',True,B.HasAggregate);
  finally
    B.Free;
  end;
  B:=TFPBinaryOperation.Create(TFPConstExpression.CreateInteger(1),FRight);
  try
    FRight:=Nil;
    AssertEquals('Binary',True,B.HasAggregate);
  finally
    B.Free;
  end;
end;

procedure TTestParserAggregate.TestUnaryAggregate;
Var
  B : TFPUnaryOperator;

begin
  B:=TFPUnaryOperator.Create(Fleft);
  try
    FLeft:=Nil;
    AssertEquals('Unary',True,B.HasAggregate);
  finally
    B.Free;
  end;
end;

procedure TTestParserAggregate.TestCountAggregate;

Var
  C : TAggregateCount;
  I : Integer;
  R : TFPExpressionResult;

begin
  FFunction.ResultType:=rtInteger;
  FFunction.ParameterTypes:='';
  C:=TAggregateCount.CreateFunction(FFunction,Nil);
  try
    C.Check;
    C.InitAggregate;
    For I:=1 to 11 do
      C.UpdateAggregate;
    C.GetNodeValue(R);
    AssertEquals('Correct type',rtInteger,R.ResultType);
    AssertEquals('Correct value',11,R.ResInteger);
  finally
    C.Free;
  end;
end;

procedure TTestParserAggregate.TestSumAggregate;

Var
  C : TAggregateSum;
  V : TFPExprVariable;
  I : Integer;
  R : TFPExpressionResult;
  A : TExprArgumentArray;

begin
  FFunction.ResultType:=rtInteger;
  FFunction.ParameterTypes:='I';
  FFunction.Name:='SUM';
  FFunction2.ResultType:=rtInteger;
  C:=Nil;
  V:=TFPExprVariable.CreateIdentifier(FFunction2);
  try
    SetLength(A,1);
    A[0]:=V;
    C:=TAggregateSum.CreateFunction(FFunction,A);
    C.Check;
    C.InitAggregate;
    For I:=1 to 10 do
      begin
      FVarValue:=I;
      C.UpdateAggregate;
      end;
    C.GetNodeValue(R);
    AssertEquals('Correct type',rtInteger,R.ResultType);
    AssertEquals('Correct value',55,R.ResInteger);
  finally
    C.Free;
  end;
end;

procedure TTestParserAggregate.TestSumAggregate2;
Var
  C : TAggregateSum;
  V : TFPExprVariable;
  I : Integer;
  R : TFPExpressionResult;
  A : TExprArgumentArray;

begin
  FFunction.ResultType:=rtFloat;
  FFunction.ParameterTypes:='F';
  FFunction.Name:='SUM';
  FFunction2.ResultType:=rtFloat;
  C:=Nil;
  V:=TFPExprVariable.CreateIdentifier(FFunction2);
  try
    SetLength(A,1);
    A[0]:=V;
    C:=TAggregateSum.CreateFunction(FFunction,A);
    C.Check;
    C.InitAggregate;
    For I:=1 to 10 do
      begin
      FVarValue:=I;
      C.UpdateAggregate;
      end;
    C.GetNodeValue(R);
    AssertEquals('Correct type',rtFloat,R.ResultType);
    AssertEquals('Correct value',55/2,R.ResFloat,0.1);
  finally
    C.Free;
  end;
end;

procedure TTestParserAggregate.TestSumAggregate3;
Var
  C : TAggregateSum;
  V : TFPExprVariable;
  I : Integer;
  R : TFPExpressionResult;
  A : TExprArgumentArray;

begin
  FFunction.ResultType:=rtCurrency;
  FFunction.ParameterTypes:='F';
  FFunction.Name:='SUM';
  FFunction2.ResultType:=rtCurrency;
  C:=Nil;
  V:=TFPExprVariable.CreateIdentifier(FFunction2);
  try
    SetLength(A,1);
    A[0]:=V;
    C:=TAggregateSum.CreateFunction(FFunction,A);
    C.Check;
    C.InitAggregate;
    For I:=1 to 10 do
      begin
      FVarValue:=I;
      C.UpdateAggregate;
      end;
    C.GetNodeValue(R);
    AssertEquals('Correct type',rtCurrency,R.ResultType);
    AssertEquals('Correct value',55/2,R.ResCurrency,0.1);
  finally
    C.Free;
  end;
end;

procedure TTestParserAggregate.TestAvgAggregate;

Var
  C : TAggregateAvg;
  V : TFPExprVariable;
  I : Integer;
  R : TFPExpressionResult;
  A : TExprArgumentArray;

begin
  FFunction.ResultType:=rtInteger;
  FFunction.ParameterTypes:='F';
  FFunction.Name:='AVG';
  FFunction2.ResultType:=rtInteger;
  C:=Nil;
  V:=TFPExprVariable.CreateIdentifier(FFunction2);
  try
    SetLength(A,1);
    A[0]:=V;
    C:=TAggregateAvg.CreateFunction(FFunction,A);
    C.Check;
    C.InitAggregate;
    For I:=1 to 10 do
      begin
      FVarValue:=I;
      C.UpdateAggregate;
      end;
    C.GetNodeValue(R);
    AssertEquals('Correct type',rtFloat,R.ResultType);
    AssertEquals('Correct value',5.5,R.ResFloat,0.1);
  finally
    C.Free;
  end;
end;

procedure TTestParserAggregate.TestAvgAggregate2;

Var
  C : TAggregateAvg;
  V : TFPExprVariable;
  I : Integer;
  R : TFPExpressionResult;
  A : TExprArgumentArray;

begin
  FFunction.ResultType:=rtInteger;
  FFunction.ParameterTypes:='F';
  FFunction.Name:='AVG';
  FFunction2.ResultType:=rtFloat;
  C:=Nil;
  V:=TFPExprVariable.CreateIdentifier(FFunction2);
  try
    SetLength(A,1);
    A[0]:=V;
    C:=TAggregateAvg.CreateFunction(FFunction,A);
    C.Check;
    C.InitAggregate;
    For I:=1 to 10 do
      begin
      FVarValue:=I;
      C.UpdateAggregate;
      end;
    C.GetNodeValue(R);
    AssertEquals('Correct type',rtFloat,R.ResultType);
    AssertEquals('Correct value',5.5/2,R.ResFloat,0.1);
  finally
    C.Free;
  end;
end;

procedure TTestParserAggregate.TestAvgAggregate3;
Var
  C : TAggregateAvg;
  V : TFPExprVariable;
  R : TFPExpressionResult;
  A : TExprArgumentArray;

begin
  FFunction.ResultType:=rtInteger;
  FFunction.ParameterTypes:='F';
  FFunction.Name:='AVG';
  FFunction2.ResultType:=rtFloat;
  C:=Nil;
  V:=TFPExprVariable.CreateIdentifier(FFunction2);
  try
    SetLength(A,1);
    A[0]:=V;
    C:=TAggregateAvg.CreateFunction(FFunction,A);
    C.Check;
    C.InitAggregate;
    C.GetNodeValue(R);
    AssertEquals('Correct type',rtFloat,R.ResultType);
    AssertEquals('Correct value',0.0,R.ResFloat,0.1);
  finally
    C.Free;
  end;
end;

{ TAggregateNode }

class function TAggregateNode.IsAggregate: Boolean;
begin
  Result:=True
end;

function TAggregateNode.NodeType: TResultType;
begin
  Result:=rtInteger;
end;

procedure TAggregateNode.InitAggregate;
begin
  inherited InitAggregate;
  inc(InitCount)
end;

procedure TAggregateNode.UpdateAggregate;
begin
  inherited UpdateAggregate;
  inc(UpdateCount);
end;

procedure TAggregateNode.GetNodeValue(var Result: TFPExpressionResult);
begin
  Result.ResultType:=rtInteger;
  Result.ResInteger:=updateCount;
end;

procedure TTestExpressionScanner.TestCreate;
begin
  AssertEquals('Empty source','',FP.Source);
  AssertEquals('Pos is zero',0,FP.Pos);
  AssertEquals('CurrentChar is zero',#0,FP.CurrentChar);
  AssertEquals('Current token type is EOF',ttEOF,FP.TokenType);
  AssertEquals('Current token is empty','',FP.Token);
end;

procedure TTestExpressionScanner.TestSetSource;
begin
  FP.Source:='Abc';
  FP.Source:='';
  AssertEquals('Empty source','',FP.Source);
  AssertEquals('Pos is zero',0,FP.Pos);
  AssertEquals('CurrentChar is zero',#0,FP.CurrentChar);
  AssertEquals('Current token type is EOF',ttEOF,FP.TokenType);
  AssertEquals('Current token is empty','',FP.Token);
end;

procedure TTestExpressionScanner.TestWhiteSpace;
begin
  TestString('  ',ttEOF);
end;

procedure TTestExpressionScanner.TestTokens;

Const
  TestStrings : Array[TTokenType] of String
  (*
  TTokenType = (ttPlus, ttMinus, ttLessThan, ttLargerThan, ttEqual, ttDiv,
                ttMod, ttMul, ttLeft, ttRight, ttLessThanEqual,
                ttLargerThanEqual, ttunequal, ttNumber, ttString, ttIdentifier,
                ttComma, ttAnd, ttOr, ttXor, ttTrue, ttFalse, ttNot, ttif,
                ttCase, ttPower, ttEOF); // keep ttEOF last

  *)
    = ('+','-','<','>','=','/',
       'mod','*','(',')','<=',
       '>=', '<>','1','''abc''','abc',
       ',','and', 'or','xor','true','false','not',
       'if','case','^','');

var
  t : TTokenType;

begin
  For T:=Low(TTokenType) to High(TTokenType) do
    TestString(TestStrings[t],t);
end;

procedure TTestExpressionScanner.TestInvalidNumber;

begin
  TestString(FInvalidString,ttNumber);
end;

procedure TTestExpressionScanner.DoInvalidNumber(AString : String);

begin
  FInvalidString:=AString;
  AssertException('Invalid number "'+AString+'" ',EExprScanner,@TestInvalidNumber);
end;

procedure TTestExpressionScanner.TestNumber;
begin
  TestString('123',ttNumber);
  TestString('$FF',ttNumber);
  TestString('&77',ttNumber);
  TestString('%11111111',ttNumber);
  TestString('123.4',ttNumber);
  TestString('123.E4',ttNumber);
  TestString('1.E4',ttNumber);
  TestString('1e-2',ttNumber);
  DoInValidNumber('$GG');
  DoInvalidNumber('&88');
  DoInvalidNumber('%22');
  DoInvalidNumber('1..1');
  DoInvalidNumber('1.E--1');
//  DoInvalidNumber('.E-1');
end;


procedure TTestExpressionScanner.TestInvalidCharacter;
begin
  DoInvalidNumber('~');
  DoInvalidNumber('#');
  DoInvalidNumber('$');
end;

procedure TTestExpressionScanner.TestUnterminatedString;
begin
  DoInvalidNumber('''abc');
end;

procedure TTestExpressionScanner.TestQuotesInString;
begin
  TestString('''That''''s it''',ttString);
  TestString('''''''s it''',ttString);
  TestString('''s it''''''',ttString);
end;

procedure TTestExpressionScanner.TestIdentifier(Const ASource,ATokenName : string);

begin
  FP.Source:=ASource;
  AssertEquals('Token type',ttIdentifier,FP.GetToken);
  AssertEquals('Token name',ATokenName,FP.Token);
end;

procedure TTestExpressionScanner.TestIdentifiers;
begin
  TestIdentifier('a','a');
  TestIdentifier(' a','a');
  TestIdentifier('a ','a');
  TestIdentifier('a^b','a');
  TestIdentifier('a-b','a');
  TestIdentifier('a.b','a.b');
  TestIdentifier('"a b"','a b');
  TestIdentifier('c."a b"','c.a b');
  TestIdentifier('c."ab"','c.ab');
end;

procedure TTestExpressionScanner.SetUp; 
begin
  FP:=TFPExpressionScanner.Create;
end;

procedure TTestExpressionScanner.TearDown; 
begin
  FreeAndNil(FP);
end;

procedure TTestExpressionScanner.AssertEquals(Msg: string; AExpected,
  AActual: TTokenType);

Var
  S1,S2 : String;

begin
  S1:=TokenName(AExpected);
  S2:=GetEnumName(TypeInfo(TTokenType),Ord(AActual));
  AssertEquals(Msg,S1,S2);
end;

procedure TTestExpressionScanner.TestString(const AString: String;
  AToken: TTokenType);
begin
  FP.Source:=AString;
  AssertEquals('String "'+AString+'" results in token '+TokenName(AToken),AToken,FP.GetToken);
  If Not (FP.TokenType in [ttString,ttEOF]) then
    AssertEquals('String "'+AString+'" results in token string '+TokenName(AToken),AString,FP.Token)
  else if FP.TokenType=ttString then
    AssertEquals('String "'+AString+'" results in token string '+TokenName(AToken),
                  StringReplace(AString,'''''','''',[rfreplaceAll]),
                  ''''+FP.Token+'''');
end;

{ TTestBaseParser }

procedure TTestBaseParser.DoCheck;
begin
  FCheckNode.Check;
end;

procedure TTestBaseParser.AssertNodeType(Msg: String; AClass: TClass;
  ANode: TFPExprNode);
begin
  AssertNotNull(Msg+': Not null',ANode);
  AssertEquals(Msg+': Class OK',AClass,ANode.ClassType);
end;

procedure TTestBaseParser.AssertEquals(Msg: String; AResultType: TResultType;
  ANode: TFPExprNode);
begin
  AssertNotNull(Msg+': Node not null',ANode);
  AssertEquals(Msg,AResultType,Anode.NodeType);
end;

procedure TTestBaseParser.AssertEquals(Msg: String; AExpected,
  AActual: TResultType);

begin
  AssertEquals(Msg,ResultTypeName(AExpected),ResultTypeName(AActual));
end;

function TTestBaseParser.CreateIntNode(AInteger: Integer): TFPExprNode;
begin
  Result:=TFPConstExpression.CreateInteger(AInteger);
end;

function TTestBaseParser.CreateFloatNode(AFloat: TExprFloat): TFPExprNode;
begin
  Result:=TFPConstExpression.CreateFloat(AFloat);
end;

function TTestBaseParser.CreateStringNode(Astring: String): TFPExprNode;
begin
  Result:=TFPConstExpression.CreateString(AString);
end;

function TTestBaseParser.CreateDateTimeNode(ADateTime: TDateTime): TFPExprNode;
begin
  Result:=TFPConstExpression.CreateDateTime(ADateTime);
end;

procedure TTestBaseParser.AssertNodeOK(FN: TFPExprNode);

Var
  B : Boolean;
  Msg : String;

begin
  AssertNotNull('Node to test OK',FN);
  B:=False;
  try
    FN.Check;
    B:=True;
  except
    On E : Exception do
      Msg:=E.Message;
  end;
  If Not B then
    Fail(Format('Node %s not OK: %s',[FN.ClassName,Msg]));
end;

procedure TTestBaseParser.AssertNodeNotOK(const MSg : String; FN: TFPExprNode);
begin
  FCheckNode:=FN;
  AssertException(Msg,EExprParser,@DoCheck);
end;

function TTestBaseParser.CreateBoolNode(ABoolean: Boolean): TFPExprNode;
begin
  Result:=TFPConstExpression.CreateBoolean(ABoolean);
end;

procedure TTestBaseParser.Setup;
begin
  inherited Setup;
  FDestroyCalled:=0;
end;


{ TTestConstExprNode }

procedure TTestConstExprNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestConstExprNode.TestCreateInteger;
begin
  FN:=TFPConstExpression.CreateInteger(1);
  AssertEquals('Correct type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',1,FN.ConstValue.ResInteger);
  AssertEquals('Correct result',1,FN.NodeValue.ResInteger);
  AssertEquals('AsString ok','1',FN.AsString);
end;

procedure TTestConstExprNode.TestCreateFloat;

Var
  F : Double;
  C : Integer;

begin
  FN:=TFPConstExpression.CreateFloat(2.34);
  AssertEquals('Correct type',rtFloat,FN.NodeType);
  AssertEquals('Correct result',2.34,FN.ConstValue.ResFloat);
  AssertEquals('Correct result',2.34,FN.NodeValue.ResFloat);
  Val(FN.AsString,F,C);
  AssertEquals('Correct conversion',0,C);
  AssertEquals('AsString ok',2.34,F,0.001);
end;

procedure TTestConstExprNode.TestCreateBoolean;
begin
  FN:=TFPConstExpression.CreateBoolean(True);
  AssertEquals('Correct type',rtBoolean,FN.NodeType);
  AssertEquals('Correct result',True,FN.ConstValue.ResBoolean);
  AssertEquals('Correct result',True,FN.NodeValue.ResBoolean);
  AssertEquals('AsString ok','True',FN.AsString);
  FreeAndNil(FN);
  FN:=TFPConstExpression.CreateBoolean(False);
  AssertEquals('AsString ok','False',FN.AsString);
end;

procedure TTestConstExprNode.TestCreateDateTime;

Var
  D : TDateTime;
  S : String;

begin
  D:=Now;
  FN:=TFPConstExpression.CreateDateTime(D);
  AssertEquals('Correct type',rtDateTime,FN.NodeType);
  AssertEquals('Correct result',D,FN.ConstValue.ResDateTime);
  AssertEquals('Correct result',D,FN.NodeValue.ResDateTime);
  S:=''''+FormatDateTime('cccc',D)+'''';
  AssertEquals('AsString ok',S,FN.AsString);
end;

procedure TTestConstExprNode.TestCreateString;

Var
  S : String;

begin
  S:='Ohlala';
  FN:=TFPConstExpression.CreateString(S);
  AssertEquals('Correct type',rtString,FN.NodeType);
  AssertEquals('Correct result',S,FN.ConstValue.ResString);
  AssertEquals('Correct result',S,FN.NodeValue.ResString);
  AssertEquals('AsString ok',''''+S+'''',FN.AsString);
end;

{ TTestNegateExprNode }

procedure TTestNegateExprNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestNegateExprNode.TestCreateInteger;

begin
  FN:=TFPNegateOperation.Create(CreateIntNode(23));
  AssertEquals('Negate has correct type',rtInteger,FN.NodeType);
  AssertEquals('Negate has correct result',-23,FN.NodeValue.Resinteger);
  AssertEquals('Negate has correct string','-23',FN.AsString);
  AssertNodeOK(FN);
end;


procedure TTestNegateExprNode.TestCreateFloat;

Var
  S : String;

begin
  FN:=TFPNegateOperation.Create(CreateFloatNode(1.23));
  AssertEquals('Negate has correct type',rtFloat,FN.NodeType);
  AssertEquals('Negate has correct result',-1.23,FN.NodeValue.ResFloat);
  Str(TExprFloat(-1.23),S);
  AssertEquals('Negate has correct string',S,FN.AsString);
  AssertNodeOK(FN);
end;

procedure TTestNegateExprNode.TestCreateOther1;

begin
  FN:=TFPNegateOperation.Create(TFPConstExpression.CreateString('1.23'));
  AssertNodeNotOK('Negate does not accept string',FN);
end;

procedure TTestNegateExprNode.TestCreateOther2;

begin
  FN:=TFPNegateOperation.Create(TFPConstExpression.CreateBoolean(True));
  AssertNodeNotOK('Negate does not accept boolean',FN)
end;

procedure TTestNegateExprNode.TestDestroy;
begin
  FN:=TFPNegateOperation.Create(TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Operand Destroy called',1,self.FDestroyCalled)
end;

{ TTestDestroyNode }

procedure TTestDestroyNode.TestDestroy;

Var
  FN : TMyDestroyNode;

begin
  AssertEquals('Destroy not called yet',0,self.FDestroyCalled);
  FN:=TMyDestroyNode.CreateTest(Self);
  FN.Free;
  AssertEquals('Destroy called',1,self.FDestroyCalled)
end;

{ TMyDestroyNode }

constructor TMyDestroyNode.CreateTest(ATest: TTestBaseParser);
begin
  FTest:=ATest;
  Inherited CreateInteger(1);
end;

destructor TMyDestroyNode.Destroy;
begin
  Inc(FTest.FDestroyCalled);
  inherited Destroy;
end;

{ TTestBinaryAndNode }

procedure TTestBinaryAndNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestBinaryAndNode.TestCreateInteger;
begin
  FN:=TFPBinaryAndOperation.Create(CreateIntNode(3),CreateIntNode(2));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',2,FN.NodeValue.ResInteger);
end;

procedure TTestBinaryAndNode.TestCreateBoolean;
begin
  FN:=TFPBinaryAndOperation.Create(CreateBoolNode(True),CreateBoolNode(True));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtBoolean,FN.NodeType);
  AssertEquals('Correct result',True,FN.NodeValue.ResBoolean);
end;

procedure TTestBinaryAndNode.TestCreateBooleanInteger;
begin
  FN:=TFPBinaryAndOperation.Create(CreateBoolNode(True),CreateIntNode(0));
  AssertNodeNotOK('Different node types',FN);
end;

procedure TTestBinaryAndNode.TestCreateString;
begin
  FN:=TFPBinaryAndOperation.Create(CreateStringNode('True'),CreateStringNode('True'));
  AssertNodeNotOK('String node type',FN);
end;

procedure TTestBinaryAndNode.TestCreateFloat;
begin
  FN:=TFPBinaryAndOperation.Create(CreateFloatNode(1.23),CreateFloatNode(1.23));
  AssertNodeNotOK('float node type',FN);
end;

procedure TTestBinaryAndNode.TestCreateDateTime;
begin
  FN:=TFPBinaryAndOperation.Create(CreateDateTimeNode(Now),CreateDateTimeNode(Now));
  AssertNodeNotOK('DateTime node type',FN);
end;

procedure TTestBinaryAndNode.TestDestroy;
begin
  FN:=TFPBinaryAndOperation.Create(TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',2,self.FDestroyCalled)
end;

{ TTestBinaryOrNode }

procedure TTestBinaryOrNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestBinaryOrNode.TestCreateInteger;
begin
  FN:=TFPBinaryOrOperation.Create(CreateIntNode(1),CreateIntNode(2));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',3,FN.NodeValue.ResInteger);
end;

procedure TTestBinaryOrNode.TestCreateBoolean;
begin
  FN:=TFPBinaryOrOperation.Create(CreateBoolNode(True),CreateBoolNode(False));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtBoolean,FN.NodeType);
  AssertEquals('Correct result',True,FN.NodeValue.ResBoolean);
end;

procedure TTestBinaryOrNode.TestCreateBooleanInteger;
begin
  FN:=TFPBinaryOrOperation.Create(CreateBoolNode(True),CreateIntNode(0));
  AssertNodeNotOK('Different node types',FN);
end;

procedure TTestBinaryOrNode.TestCreateString;
begin
  FN:=TFPBinaryOrOperation.Create(CreateStringNode('True'),CreateStringNode('True'));
  AssertNodeNotOK('String node type',FN);
end;

procedure TTestBinaryOrNode.TestCreateFloat;
begin
  FN:=TFPBinaryOrOperation.Create(CreateFloatNode(1.23),CreateFloatNode(1.23));
  AssertNodeNotOK('float node type',FN);
end;

procedure TTestBinaryOrNode.TestCreateDateTime;
begin
  FN:=TFPBinaryOrOperation.Create(CreateDateTimeNode(Now),CreateDateTimeNode(Now));
  AssertNodeNotOK('DateTime node type',FN);
end;

procedure TTestBinaryOrNode.TestDestroy;
begin
  FN:=TFPBinaryOrOperation.Create(TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',2,self.FDestroyCalled)
end;

{ TTestBinaryXorNode }

procedure TTestBinaryXorNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestBinaryXorNode.TestCreateInteger;
begin
  FN:=TFPBinaryXorOperation.Create(CreateIntNode(1),CreateIntNode(2));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',3,FN.NodeValue.ResInteger);
end;

procedure TTestBinaryXorNode.TestCreateBoolean;
begin
  FN:=TFPBinaryXorOperation.Create(CreateBoolNode(True),CreateBoolNode(True));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtBoolean,FN.NodeType);
  AssertEquals('Correct result',False,FN.NodeValue.ResBoolean);
end;

procedure TTestBinaryXorNode.TestCreateBooleanInteger;
begin
  FN:=TFPBinaryXorOperation.Create(CreateBoolNode(True),CreateIntNode(0));
  AssertNodeNotOK('Different node types',FN);
end;

procedure TTestBinaryXorNode.TestCreateString;
begin
  FN:=TFPBinaryXorOperation.Create(CreateStringNode('True'),CreateStringNode('True'));
  AssertNodeNotOK('String node type',FN);
end;

procedure TTestBinaryXorNode.TestCreateFloat;
begin
  FN:=TFPBinaryXorOperation.Create(CreateFloatNode(1.23),CreateFloatNode(1.23));
  AssertNodeNotOK('float node type',FN);
end;

procedure TTestBinaryXorNode.TestCreateDateTime;
begin
  FN:=TFPBinaryXorOperation.Create(CreateDateTimeNode(Now),CreateDateTimeNode(Now));
  AssertNodeNotOK('DateTime node type',FN);
end;

procedure TTestBinaryXorNode.TestDestroy;
begin
  FN:=TFPBinaryXorOperation.Create(TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',2,self.FDestroyCalled)
end;

{ TTestBooleanNode }

procedure TTestBooleanNode.TestNode(B: TFPBooleanResultOperation;
  AResult: Boolean);
begin
  AssertEquals(Format('Test %s(%s,%s) result',[B.ClassName,B.Left.AsString,B.Right.AsString]),Aresult,B.NodeValue.resBoolean);
end;

{ TTestEqualNode }

procedure TTestEqualNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

class function TTestEqualNode.NodeClass: TFPBooleanResultOperationClass;
begin
  Result:=TFPEqualOperation;
end;

class function TTestEqualNode.ExpectedResult: Boolean;
begin
  Result:=True
end;

class function TTestEqualNode.OperatorString: String;
begin
  Result:='=';
end;

procedure TTestEqualNode.TestCreateIntegerEqual;
begin
  FN:=NodeClass.Create(CreateIntNode(1),CreateIntNode(1));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,ExpectedResult);
end;

procedure TTestEqualNode.TestCreateIntegerUnEqual;
begin
  FN:=NodeClass.Create(CreateIntNode(2),CreateIntNode(1));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Not ExpectedResult);
end;

procedure TTestEqualNode.TestCreateFloatEqual;
begin
  FN:=NodeClass.Create(CreateFloatNode(1.23),CreateFloatNode(1.23));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,ExpectedResult);
end;

procedure TTestEqualNode.TestCreateFloatUnEqual;
begin
  FN:=NodeClass.Create(CreateFloatNode(1.23),CreateFloatNode(1.34));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Not ExpectedResult);
end;

procedure TTestEqualNode.TestCreateStringEqual;
begin
  FN:=NodeClass.Create(CreateStringNode('now'),CreateStringNode('now'));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,ExpectedResult);
end;

procedure TTestEqualNode.TestCreateStringUnEqual;
begin
  FN:=NodeClass.Create(CreateStringNode('now'),CreateStringNode('then'));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Not ExpectedResult);
end;

procedure TTestEqualNode.TestCreateBooleanEqual;
begin
  FN:=NodeClass.Create(CreateBoolNode(True),CreateBoolNode(True));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,ExpectedResult);
end;

procedure TTestEqualNode.TestCreateBooleanUnEqual;
begin
  FN:=NodeClass.Create(CreateBoolNode(False),CreateBoolNode(True));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Not ExpectedResult);
end;

procedure TTestEqualNode.TestCreateDateTimeEqual;

Var
  D : TDateTime;

begin
  D:=Now;
  FN:=NodeClass.Create(CreateDateTimeNode(D),CreateDateTimeNode(D));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,ExpectedResult);
end;

procedure TTestEqualNode.TestCreateDateTimeUnEqual;

Var
  D : TDateTime;

begin
  D:=Now;
  FN:=NodeClass.Create(CreateDateTimeNode(D),CreateDateTimeNode(D-1));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Not ExpectedResult);
end;


procedure TTestEqualNode.TestDestroy;
begin
  FN:=NodeClass.Create(TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',2,self.FDestroyCalled)
end;

procedure TTestEqualNode.TestWrongTypes1;
begin
  FN:=NodeClass.Create(CreateIntNode(3),CreateStringNode('1.23'));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestEqualNode.TestWrongTypes2;
begin
  FN:=NodeClass.Create(CreateDateTimeNode(3),CreateStringNode('1.23'));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestEqualNode.TestWrongTypes3;
begin
  FN:=NodeClass.Create(CreateFloatNode(1.3),CreateStringNode('1.23'));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestEqualNode.TestWrongTypes4;
begin
  FN:=NodeClass.Create(CreateBoolNode(False),CreateStringNode('1.23'));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestEqualNode.TestWrongTypes5;
begin
  FN:=NodeClass.Create(CreateFloatNode(1),CreateIntNode(1));
  AssertNodeNotOk('Wrong Types',FN);
end;


procedure TTestEqualNode.TestAsString;
begin
  FN:=NodeClass.Create(CreateIntNode(1),CreateIntNode(2));
  AssertEquals('Asstring works ok','1 '+OPeratorString+' 2',FN.AsString);
end;

{ TTestUnEqualNode }

class function TTestUnEqualNode.NodeClass: TFPBooleanResultOperationClass;
begin
  Result:=TFPUnEqualOperation;
end;

class function TTestUnEqualNode.ExpectedResult: Boolean;
begin
  Result:=False;
end;

class function TTestUnEqualNode.OperatorString: String;
begin
  Result:='<>';
end;

{ TTestLessThanNode }

class function TTestLessThanNode.NodeClass: TFPBooleanResultOperationClass;
begin
  Result:=TFPLessThanOperation;
end;

class function TTestLessThanNode.Larger: Boolean;
begin
  Result:=False;
end;

class function TTestLessThanNode.AllowEqual: Boolean;
begin
  Result:=False;
end;

class function TTestLessThanNode.OperatorString: String;
begin
  Result:='<';
end;

procedure TTestLessThanNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestLessThanNode.TestCreateIntegerEqual;
begin
  FN:=NodeClass.Create(CreateIntNode(1),CreateIntNode(1));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,AllowEqual);
end;

procedure TTestLessThanNode.TestCreateIntegerSmaller;
begin
  FN:=NodeClass.Create(CreateIntNode(1),CreateIntNode(2));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Not Larger);
end;

procedure TTestLessThanNode.TestCreateIntegerLarger;
begin
  FN:=NodeClass.Create(CreateIntNode(2),CreateIntNode(1));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Larger);
end;

procedure TTestLessThanNode.TestCreateFloatEqual;
begin
  FN:=NodeClass.Create(CreateFloatNode(1.23),CreateFloatNode(1.23));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,AllowEqual);
end;

procedure TTestLessThanNode.TestCreateFloatSmaller;
begin
  FN:=NodeClass.Create(CreateFloatNode(1.23),CreateFloatNode(4.56));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Not Larger);
end;

procedure TTestLessThanNode.TestCreateFloatLarger;
begin
  FN:=NodeClass.Create(CreateFloatNode(4.56),CreateFloatNode(1.23));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Larger);
end;

procedure TTestLessThanNode.TestCreateDateTimeEqual;

Var
  D : TDateTime;

begin
  D:=Now;
  FN:=NodeClass.Create(CreateDateTimeNode(D),CreateDateTimeNode(D));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,AllowEqual);
end;

procedure TTestLessThanNode.TestCreateDateTimeSmaller;

Var
  D : TDateTime;

begin
  D:=Now;
  FN:=NodeClass.Create(CreateDateTimeNode(D),CreateDateTimeNode(D+1));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Not larger);
end;

procedure TTestLessThanNode.TestCreateDateTimeLarger;

Var
  D : TDateTime;

begin
  D:=Now;
  FN:=NodeClass.Create(CreateDateTimeNode(D),CreateDateTimeNode(D-1));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,larger);
end;

procedure TTestLessThanNode.TestCreateStringEqual;
begin
  FN:=NodeClass.Create(CreateStringNode('now'),CreateStringNode('now'));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,AllowEqual);
end;

procedure TTestLessThanNode.TestCreateStringSmaller;
begin
  FN:=NodeClass.Create(CreateStringNode('now'),CreateStringNode('then'));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Not Larger);
end;

procedure TTestLessThanNode.TestCreateStringLarger;
begin
  FN:=NodeClass.Create(CreateStringNode('then'),CreateStringNode('now'));
  AssertNodeOk(FN);
  AssertEquals('Boolean result',rtBoolean,FN.NodeType);
  TestNode(FN,Larger);
end;

procedure TTestLessThanNode.TestWrongTypes1;
begin
  FN:=NodeClass.Create(CreateIntNode(3),CreateStringNode('1.23'));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestLessThanNode.TestWrongTypes2;
begin
  FN:=NodeClass.Create(CreateDateTimeNode(3),CreateStringNode('1.23'));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestLessThanNode.TestWrongTypes3;
begin
  FN:=NodeClass.Create(CreateFloatNode(1.3),CreateStringNode('1.23'));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestLessThanNode.TestWrongTypes4;
begin
  FN:=NodeClass.Create(CreateBoolNode(False),CreateStringNode('1.23'));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestLessThanNode.TestWrongTypes5;
begin
  FN:=NodeClass.Create(CreateFloatNode(1.23),CreateIntNode(1));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestLessThanNode.TestNoBoolean1;
begin
  FN:=NodeClass.Create(CreateBoolNode(False),CreateIntNode(1));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestLessThanNode.TestNoBoolean2;
begin
  FN:=NodeClass.Create(CreateIntNode(1),CreateBoolNode(False));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestLessThanNode.TestNoBoolean3;
begin
  FN:=NodeClass.Create(CreateBoolNode(False),CreateBoolNode(False));
  AssertNodeNotOk('Wrong Types',FN);
end;

procedure TTestLessThanNode.TestAsString;
begin
  FN:=NodeClass.Create(CreateIntNode(1),CreateIntNode(2));
  AssertEquals('Asstring works ok','1 '+OPeratorString+' 2',FN.AsString);
end;

{ TTestLessThanEqualNode }

class function TTestLessThanEqualNode.NodeClass: TFPBooleanResultOperationClass;
begin
  Result:=TFPLessThanEqualOperation;
end;

class function TTestLessThanEqualNode.AllowEqual: Boolean;
begin
  Result:=True;
end;

class function TTestLessThanEqualNode.OperatorString: String;
begin
  Result:='<=';
end;

{ TTestLargerThanNode }

class function TTestLargerThanNode.NodeClass: TFPBooleanResultOperationClass;
begin
  Result:=TFPGreaterThanOperation;
end;

class function TTestLargerThanNode.Larger: Boolean;
begin
  Result:=True;
end;

class function TTestLargerThanNode.OperatorString: String;
begin
  Result:='>';
end;

{ TTestLargerThanEqualNode }

class function TTestLargerThanEqualNode.NodeClass: TFPBooleanResultOperationClass;
begin
  Result:=TFPGreaterThanEqualOperation;
end;

class function TTestLargerThanEqualNode.AllowEqual: Boolean;
begin
  Result:=True;
end;

class function TTestLargerThanEqualNode.OperatorString: String;
begin
  Result:='>=';
end;

{ TTestAddNode }

procedure TTestAddNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestAddNode.TestCreateInteger;
begin
  FN:=TFPAddOperation.Create(CreateIntNode(1),CreateIntNode(2));
  AssertEquals('Add has correct type',rtInteger,FN.NodeType);
  AssertEquals('Add has correct result',3,FN.NodeValue.ResInteger);
end;

procedure TTestAddNode.TestCreateFloat;
begin
  FN:=TFPAddOperation.Create(CreateFloatNode(1.23),CreateFloatNode(4.56));
  AssertEquals('Add has correct type',rtFloat,FN.NodeType);
  AssertEquals('Add has correct result',5.79,FN.NodeValue.ResFloat);
end;

procedure TTestAddNode.TestCreateDateTime;

Var
  D,T : TDateTime;

begin
  D:=Date;
  T:=Time;
  FN:=TFPAddOperation.Create(CreateDateTimeNode(D),CreateDateTimeNode(T));
  AssertEquals('Add has correct type',rtDateTime,FN.NodeType);
  AssertEquals('Add has correct result',D+T,FN.NodeValue.ResDateTime);
end;

procedure TTestAddNode.TestCreateString;
begin
  FN:=TFPAddOperation.Create(CreateStringNode('alo'),CreateStringNode('ha'));
  AssertEquals('Add has correct type',rtString,FN.NodeType);
  AssertEquals('Add has correct result','aloha',FN.NodeValue.ResString);
end;

procedure TTestAddNode.TestCreateBoolean;
begin
  FN:=TFPAddOperation.Create(CreateBoolNode(True),CreateBoolNode(False));
  AssertNodeNotOK('No boolean addition',FN);
end;

procedure TTestAddNode.TestDestroy;
begin
  FN:=TFPAddOperation.Create(TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',2,self.FDestroyCalled)
end;

procedure TTestAddNode.TestAsString;
begin
  FN:=TFPAddOperation.Create(CreateIntNode(1),CreateIntNode(2));
  AssertEquals('Asstring works ok','1 + 2',FN.AsString);
end;

{ TTestSubtractNode }

procedure TTestSubtractNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestSubtractNode.TestCreateInteger;
begin
  FN:=TFPSubtractOperation.Create(CreateIntNode(4),CreateIntNode(1));
  AssertEquals('Subtract has correct type',rtInteger,FN.NodeType);
  AssertEquals('Subtract has correct result',3,FN.NodeValue.ResInteger);
end;

procedure TTestSubtractNode.TestCreateFloat;
begin
  FN:=TFPSubtractOperation.Create(CreateFloatNode(4.56),CreateFloatNode(1.23));
  AssertEquals('Subtract has correct type',rtFloat,FN.NodeType);
  AssertEquals('Subtract has correct result',3.33,FN.NodeValue.ResFloat);
end;

procedure TTestSubtractNode.TestCreateDateTime;

Var
  D,T : TDateTime;

begin
  D:=Date;
  T:=Time;
  FN:=TFPSubtractOperation.Create(CreateDateTimeNode(D+T),CreateDateTimeNode(T));
  AssertEquals('Subtract has correct type',rtDateTime,FN.NodeType);
  AssertEquals('Subtract has correct result',D,FN.NodeValue.ResDateTime);
end;

procedure TTestSubtractNode.TestCreateString;
begin
  FN:=TFPSubtractOperation.Create(CreateStringNode('alo'),CreateStringNode('ha'));
  AssertNodeNotOK('No string Subtract',FN);
end;

procedure TTestSubtractNode.TestCreateBoolean;
begin
  FN:=TFPSubtractOperation.Create(CreateBoolNode(True),CreateBoolNode(False));
  AssertNodeNotOK('No boolean Subtract',FN);
end;

procedure TTestSubtractNode.TestDestroy;
begin
  FN:=TFPSubtractOperation.Create(TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',2,self.FDestroyCalled)
end;

procedure TTestSubtractNode.TestAsString;
begin
  FN:=TFPSubtractOperation.Create(CreateIntNode(1),CreateIntNode(2));
  AssertEquals('Asstring works ok','1 - 2',FN.AsString);
end;

{ TTestMultiplyNode }

procedure TTestMultiplyNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestMultiplyNode.TestCreateInteger;
begin
  FN:=TFPMultiplyOperation.Create(CreateIntNode(4),CreateIntNode(2));
  AssertEquals('multiply has correct type',rtInteger,FN.NodeType);
  AssertEquals('multiply has correct result',8,FN.NodeValue.ResInteger);
end;

procedure TTestMultiplyNode.TestCreateFloat;
begin
  FN:=TFPMultiplyOperation.Create(CreateFloatNode(2.0),CreateFloatNode(1.23));
  AssertEquals('multiply has correct type',rtFloat,FN.NodeType);
  AssertEquals('multiply has correct result',2.46,FN.NodeValue.ResFloat);
end;

procedure TTestMultiplyNode.TestCreateDateTime;

Var
  D,T : TDateTime;

begin
  D:=Date;
  T:=Time;
  FN:=TFPMultiplyOperation.Create(CreateDateTimeNode(D+T),CreateDateTimeNode(T));
  AssertNodeNotOK('No datetime multiply',FN);
end;

procedure TTestMultiplyNode.TestCreateString;
begin
  FN:=TFPMultiplyOperation.Create(CreateStringNode('alo'),CreateStringNode('ha'));
  AssertNodeNotOK('No string multiply',FN);
end;

procedure TTestMultiplyNode.TestCreateBoolean;
begin
  FN:=TFPMultiplyOperation.Create(CreateBoolNode(True),CreateBoolNode(False));
  AssertNodeNotOK('No boolean multiply',FN);
end;

procedure TTestMultiplyNode.TestDestroy;
begin
  FN:=TFPMultiplyOperation.Create(TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',2,self.FDestroyCalled)
end;

procedure TTestMultiplyNode.TestAsString;
begin
  FN:=TFPMultiplyOperation.Create(CreateIntNode(1),CreateIntNode(2));
  AssertEquals('Asstring works ok','1 * 2',FN.AsString);
end;


{ TTestPowerNode }

procedure TTestPowerNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestPowerNode.Setup;
begin
  inherited ;
  FE:=TFpExpressionParser.Create(Nil);
  FE.Builtins := [bcMath];
end;

procedure TTestPowerNode.Calc(AExpr: String; Expected: Double =NaN);
const
  EPS = 1e-9;
var
  res: TFpExpressionResult;
  x: Double;
begin
  FE.Expression := AExpr;
  res:=FE.Evaluate;
  x:= ArgToFloat(res);
  if not IsNaN(Expected) then 
    AssertEquals('Expression '+AExpr+' result',Expected,X,Eps);
end;

procedure TTestPowerNode.TestCalc;

begin
  Calc('2^2', Power(2, 2));
  Calc('2^-2', Power(2, -2));
  Calc('2^(-2)', Power(2, -2));
  Calc('sqrt(3)^2', Power(sqrt(3), 2));
  Calc('-sqrt(3)^2', -Power(sqrt(3), 2));
  Calc('-2^2', -Power(2, 2));
  Calc('(-2.0)^2', Power(-2.0, 2));
  Calc('(-2.0)^-2', Power(-2.0, -2));
  // Odd integer exponent
  Calc('2^3', Power(2, 3));
  Calc('-2^3', -Power(2, 3));
  Calc('-2^-3', -Power(2, -3));
  Calc('-2^(-3)', -Power(2, -3));
  Calc('(-2.0)^3', Power(-2.0, 3));
  Calc('(-2.0)^-3', Power(-2.0, -3));
  // Fractional exponent
  Calc('10^2.5', power(10, 2.5));
  Calc('10^-2.5', Power(10, -2.5));
  // Expressions
  Calc('(1+1)^3', Power(1+1, 3));
  Calc('1+2^3', 1 + Power(2, 3));
  calc('2^3+1', Power(2, 3) + 1);
  Calc('2^3*2', Power(2, 3) * 2);
  Calc('2^3*-2', Power(2, 3) * -2);
  Calc('2^(1+1)', Power(2, 1+1));
  Calc('2^-(1+1)', Power(2, -(1+1)));
  WriteLn;
  // Special cases
  Calc('0^0', power(0, 0));
  calc('0^1', power(0, 1));
  Calc('0^2.5', Power(0, 2.5));
  calc('2.5^0', power(2.5, 0));
  calc('2^3^4', 2417851639229258349412352);  // according to Wolfram Alpha, 2^(3^4)

  // These expressions should throw expections

  //Calc('(-10)^2.5', NaN);  // base must be positive in case of fractional exponent
  //Calc('0^-2', NaN);       // is 1/0^2 = 1/0
end;

procedure TTestPowerNode.TestCreateInteger;
begin
  FN:=TFPPowerOperation.Create(CreateIntNode(4),CreateIntNode(2));
  AssertEquals('Power has correct type',rtfloat,FN.NodeType);
  AssertEquals('Power has correct result',16.0,FN.NodeValue.ResFloat);
end;

procedure TTestPowerNode.TestCreateFloat;
begin
  FN:=TFPPowerOperation.Create(CreateFloatNode(2.0),CreateFloatNode(3.0));
  AssertEquals('Power has correct type',rtFloat,FN.NodeType);
  AssertEquals('Power has correct result',8.0,FN.NodeValue.ResFloat);
end;

procedure TTestPowerNode.TestCreateDateTime;

Var
  D,T : TDateTime;

begin
  D:=Date;
  T:=Time;
  FN:=TFPPowerOperation.Create(CreateDateTimeNode(D+T),CreateDateTimeNode(T));
  AssertNodeNotOK('No datetime Power',FN);
end;

procedure TTestPowerNode.TestCreateString;
begin
  FN:=TFPPowerOperation.Create(CreateStringNode('alo'),CreateStringNode('ha'));
  AssertNodeNotOK('No string Power',FN);
end;

procedure TTestPowerNode.TestCreateBoolean;
begin
  FN:=TFPPowerOperation.Create(CreateBoolNode(True),CreateBoolNode(False));
  AssertNodeNotOK('No boolean Power',FN);
end;

procedure TTestPowerNode.TestDestroy;
begin
  FN:=TFPPowerOperation.Create(TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',2,self.FDestroyCalled)
end;

procedure TTestPowerNode.TestAsString;
begin
  FN:=TFPPowerOperation.Create(CreateIntNode(1),CreateIntNode(2));
  AssertEquals('Asstring works ok','1^2',FN.AsString);
end;


{ TTestDivideNode }

procedure TTestDivideNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestDivideNode.TestCreateInteger;
begin
  FN:=TFPDivideOperation.Create(CreateIntNode(4),CreateIntNode(2));
  AssertEquals('Divide has correct type',rtfloat,FN.NodeType);
  AssertEquals('Divide has correct result',2.0,FN.NodeValue.ResFloat);
end;

procedure TTestDivideNode.TestCreateFloat;
begin
  FN:=TFPDivideOperation.Create(CreateFloatNode(9.0),CreateFloatNode(3.0));
  AssertEquals('Divide has correct type',rtFloat,FN.NodeType);
  AssertEquals('Divide has correct result',3.0,FN.NodeValue.ResFloat);
end;

procedure TTestDivideNode.TestCreateDateTime;

Var
  D,T : TDateTime;

begin
  D:=Date;
  T:=Time;
  FN:=TFPDivideOperation.Create(CreateDateTimeNode(D+T),CreateDateTimeNode(T));
  AssertNodeNotOK('No datetime division',FN);
end;

procedure TTestDivideNode.TestCreateString;
begin
  FN:=TFPDivideOperation.Create(CreateStringNode('alo'),CreateStringNode('ha'));
  AssertNodeNotOK('No string division',FN);
end;

procedure TTestDivideNode.TestCreateBoolean;
begin
  FN:=TFPDivideOperation.Create(CreateBoolNode(True),CreateBoolNode(False));
  AssertNodeNotOK('No boolean division',FN);
end;

procedure TTestDivideNode.TestDestroy;
begin
  FN:=TFPDivideOperation.Create(TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',2,self.FDestroyCalled)
end;

procedure TTestDivideNode.TestAsString;
begin
  FN:=TFPDivideOperation.Create(CreateIntNode(1),CreateIntNode(2));
  AssertEquals('Asstring works ok','1 / 2',FN.AsString);
end;

{ TTestIntToFloatNode }

procedure TTestIntToFloatNode.TearDown;
begin
  FreeAndNil(Fn);
  inherited TearDown;
end;

procedure TTestIntToFloatNode.TestCreateInteger;
begin
  FN:=TIntToFloatNode.Create(CreateIntNode(4));
  AssertEquals('Convert has correct type',rtfloat,FN.NodeType);
  AssertEquals('Convert has correct result',4.0,FN.NodeValue.ResFloat);
end;

procedure TTestIntToFloatNode.TestCreateFloat;
begin
  FN:=TIntToFloatNode.Create(CreateFloatNode(4.0));
  AssertNodeNotOK('No float allowed',FN);
end;

procedure TTestIntToFloatNode.TestDestroy;
begin
  FN:=TIntToFloatNode.Create(TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',1,self.FDestroyCalled)
end;

procedure TTestIntToFloatNode.TestAsString;
begin
  FN:=TIntToFloatNode.Create(CreateIntNode(4));
  AssertEquals('Convert has correct asstring','4',FN.AsString);
end;

{ TTestIntToDateTimeNode }

procedure TTestIntToDateTimeNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestIntToDateTimeNode.TestCreateInteger;
begin
  FN:=TIntToDateTimeNode.Create(CreateIntNode(Round(Date)));
  AssertEquals('Convert has correct type',rtDateTime,FN.NodeType);
  AssertEquals('Convert has correct result',Date,FN.NodeValue.ResDateTime);
end;

procedure TTestIntToDateTimeNode.TestCreateFloat;
begin
  FN:=TIntToDateTimeNode.Create(CreateFloatNode(4.0));
  AssertNodeNotOK('No float allowed',FN);
end;

procedure TTestIntToDateTimeNode.TestDestroy;
begin
  FN:=TIntToDateTimeNode.Create(TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',1,self.FDestroyCalled)
end;

procedure TTestIntToDateTimeNode.TestAsString;
begin
  FN:=TIntToDateTimeNode.Create(CreateIntNode(4));
  AssertEquals('Convert has correct asstring','4',FN.AsString);
end;

{ TTestFloatToDateTimeNode }

procedure TTestFloatToDateTimeNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestFloatToDateTimeNode.TestCreateInteger;
begin
  FN:=TFloatToDateTimeNode.Create(CreateIntNode(4));
  AssertNodeNotOK('No int allowed',FN);
end;

procedure TTestFloatToDateTimeNode.TestCreateFloat;

Var
  T : TExprFloat;

begin
  T:=Time;
  FN:=TFloatToDateTimeNode.Create(CreateFloatNode(T));
  AssertEquals('Convert has correct type',rtDateTime,FN.NodeType);
  AssertEquals('Convert has correct result',T,FN.NodeValue.ResDateTime);
end;

procedure TTestFloatToDateTimeNode.TestDestroy;
begin
  FN:=TFloatToDateTimeNode.Create(TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for left and right nodes',1,self.FDestroyCalled)
end;

procedure TTestFloatToDateTimeNode.TestAsString;

Var
  S : String;

begin
  FN:=TFloatToDateTimeNode.Create(CreateFloatNode(1.2));
  Str(TExprFloat(1.2),S);
  AssertEquals('Convert has correct asstring',S,FN.AsString);
end;

{ TMyFPExpressionParser }

procedure TMyFPExpressionParser.BuildHashList;
begin
  CreateHashList;
end;

{ TTestExpressionParser }

procedure TTestExpressionParser.SetUp;
begin
  inherited SetUp;
  FP:=TMyFPExpressionParser.Create(Nil);
end;

procedure TTestExpressionParser.TearDown;
begin
  FreeAndNil(FP);
  inherited TearDown;
end;

procedure TTestExpressionParser.DoParse;

begin
  FP.Expression:=FTestExpr;
end;

procedure TTestExpressionParser.TestParser(AExpr : string);

begin
  FTestExpr:=AExpr;
  AssertException(Format('Wrong expression: "%s"',[AExpr]),EExprParser,@DoParse);
end;

procedure TTestExpressionParser.AssertLeftRight(N: TFPExprNode; LeftClass,
  RightClass: TClass);
begin
  AssertNotNull('Binary node not null',N);
  If Not N.InheritsFrom(TFPBinaryOperation) then
    Fail(N.ClassName+' does not descend from TFPBinaryOperation');
  AssertNotNull('Left node assigned',TFPBinaryOperation(N).Left);
  AssertNotNull('Right node assigned',TFPBinaryOperation(N).Right);
  AssertEquals('Left node correct class ',LeftClass, TFPBinaryOperation(N).Left.ClassType);
  AssertEquals('Right node correct class ',RightClass, TFPBinaryOperation(N).Right.ClassType);
end;

procedure TTestExpressionParser.AssertOperand(N: TFPExprNode;
  OperandClass: TClass);
begin
  AssertNotNull('Unary node not null',N);
  If Not N.InheritsFrom(TFPUnaryOperator) then
    Fail(N.ClassName+' does not descend from TFPUnaryOperator');
  AssertNotNull('Operand assigned',TFPUnaryOperator(N).Operand);
  AssertEquals('Operand node correct class ',OperandClass, TFPUnaryOperator(N).Operand.ClassType);
end;

procedure TTestExpressionParser.AssertResultType(RT: TResultType);
begin
  AssertEquals('Result type is '+ResultTypeName(rt),rt,FP.ExprNode);
  AssertEquals('Result type is '+ResultTypeName(rt),rt,FP.ResultType);
end;

procedure TTestExpressionParser.AssertResult(F: TExprFloat);
begin
  AssertEquals('Correct float result',F,FP.ExprNode.NodeValue.ResFloat);
  AssertEquals('Correct float result',F,FP.Evaluate.ResFloat);
end;

procedure TTestExpressionParser.AssertCurrencyResult(C: Currency);
begin
  AssertEquals('Correct currency result',C,FP.ExprNode.NodeValue.ResCurrency);
  AssertEquals('Correct currency result',C,FP.Evaluate.ResCurrency);
end;

procedure TTestExpressionParser.AssertResult(I: Int64);
begin
  AssertEquals('Correct integer result',I,FP.ExprNode.NodeValue.ResInteger);
  AssertEquals('Correct integer result',I,FP.Evaluate.ResInteger);
end;

procedure TTestExpressionParser.AssertResult(S: String);
begin
  AssertEquals('Correct string result',S,FP.ExprNode.NodeValue.ResString);
  AssertEquals('Correct string result',S,FP.Evaluate.ResString);
end;

procedure TTestExpressionParser.AssertResult(B: Boolean);
begin
  AssertEquals('Correct boolean result',B,FP.ExprNode.NodeValue.ResBoolean);
  AssertEquals('Correct boolean result',B,FP.Evaluate.ResBoolean);
end;

procedure TTestExpressionParser.AssertDateTimeResult(D: TDateTime);
begin
  AssertEquals('Correct datetime result',D,FP.ExprNode.NodeValue.ResDateTime);
  AssertEquals('Correct boolean result',D,FP.Evaluate.ResDateTime);
end;
//TTestParserExpressions
procedure TTestParserExpressions.TestCreate;
begin
  AssertEquals('Expression is empty','',FP.Expression);
  AssertNotNull('Identifiers assigned',FP.Identifiers);
  AssertEquals('No identifiers',0,FP.Identifiers.Count);
end;

procedure TTestParserExpressions.TestNumberValues;

  Procedure DoTest(E :  String; V : integer);

  var
    res: TFPExpressionResult;

  begin
    FP.Expression:=E;
    res := FP.Evaluate;
    AssertTrue('Expression '+E+': Result is a number', Res.ResultType in [rtInteger,rtFloat]);
    AssertTrue('Expression '+E+': Correct value', ArgToFloat(res)=V);
  end;


begin
  // Decimal numbers
     DoTest('1', 1);
     DoTest('1E2', 100);
     DoTest('1.0/1E-2', 100);
  // DoTest('200%', 2);
     WriteLn;
     // Hex numbers
     DoTest('$0001', 1);
     DoTest('-$01', -1);
     DoTest('$A', 10);
     DoTest('$FF', 255);
     DoTest('$fe', 254);
     DoTest('$FFFF', $FFFF);
     DoTest('1E2', 100);
     DoTest('$E', 14);
     DoTest('$D+1E2', 113);
     DoTest('$0A-$0B', -1);
     // Hex and variables
     FP.Identifiers.AddVariable('a', rtInteger, '1');
     FP.Identifiers.AddVariable('b', rtInteger, '$B');
     DoTest('a', 1);
     DoTest('b', $B);
     DoTest('$A+a', 11);
     DoTest('$B-b', 0);
     WriteLn;
     // Octal numbers
     DoTest('&10', 8);
     DoTest('&10+10', 18);
     // Mixed hex and octal expression
     DoTest('&10-$0008', 0);
     WriteLn;
     // Binary numbers
     DoTest('%1', 1);
     DoTest('%11', 3);
     DoTest('%1000', 8);

end;


procedure TTestParserExpressions.TestSimpleNodeFloat;
begin
  FP.Expression:='123.4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPConstExpression, FP.ExprNode);
  AssertResultType(rtFloat);
  AssertResult(123.4);
end;

procedure TTestParserExpressions.TestSimpleNodeInteger;
begin
  FP.Expression:='1234';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPConstExpression, FP.ExprNode);
  AssertResultType(rtInteger);
  AssertResult(1234);
end;

procedure TTestParserExpressions.TestSimpleNodeBooleanTrue;
begin
  FP.Expression:='true';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPConstExpression, FP.ExprNode);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserExpressions.TestSimpleNodeBooleanFalse;
begin
  FP.Expression:='False';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPConstExpression, FP.ExprNode);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserExpressions.TestSimpleNodeString;
begin
  FP.Expression:='''A string''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPConstExpression, FP.ExprNode);
  AssertResultType(rtString);
  AssertResult('A string');
end;

procedure TTestParserExpressions.TestSimpleNegativeInteger;
begin
  FP.Expression:='-1234';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPNegateOperation, FP.ExprNode);
  AssertNodeType('Constant expression',TFPConstExpression, TFPNegateOperation(FP.ExprNode).Operand);
  AssertResultType(rtInteger);
  AssertResult(-1234);
end;

procedure TTestParserExpressions.TestSimpleNegativeFloat;
begin
  FP.Expression:='-1.234';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPNegateOperation, FP.ExprNode);
  AssertNodeType('Constant expression',TFPConstExpression, TFPNegateOperation(FP.ExprNode).Operand);
  AssertResultType(rtFloat);
  AssertResult(-1.234);
end;

procedure TTestParserExpressions.TestSimpleAddInteger;
begin
  FP.Expression:='4+1';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPAddOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(5);
end;

procedure TTestParserExpressions.TestSimpleAddFloat;
begin
  FP.Expression:='1.2+3.4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPAddOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtFloat);
  AssertResult(4.6);
end;

procedure TTestParserExpressions.TestSimpleAddIntegerFloat;
begin
  FP.Expression:='1+3.4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPAddOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TIntToFLoatNode,TFPConstExpression);
  AssertResultType(rtFloat);
  AssertResult(4.4);
end;

procedure TTestParserExpressions.TestSimpleAddFloatInteger;
begin
  FP.Expression:='3.4 + 1';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPAddOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TIntToFLoatNode);
  AssertResultType(rtFloat);
  AssertResult(4.4);
end;

procedure TTestParserExpressions.TestSimpleAddString;
begin
  FP.Expression:='''alo''+''ha''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPAddOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtString);
  AssertResult('aloha');
end;

procedure TTestParserExpressions.TestSimpleSubtractInteger;
begin
  FP.Expression:='4-1';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPSubtractOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(3);
end;

procedure TTestParserExpressions.TestSimpleSubtractFloat;
begin
  FP.Expression:='3.4-1.2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPSubtractOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtFloat);
  AssertResult(2.2);
end;

procedure TTestParserExpressions.TestSimpleSubtractIntegerFloat;
begin
  FP.Expression:='3-1.2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPSubtractOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TIntToFloatNode,TFPConstExpression);
  AssertResultType(rtFloat);
  AssertResult(1.8);
end;

procedure TTestParserExpressions.TestSimpleSubtractFloatInteger;
begin
  FP.Expression:='3.3-2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPSubtractOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TIntToFloatNode);
  AssertResultType(rtFloat);
  AssertResult(1.3);
end;

procedure TTestParserExpressions.TestSimpleMultiplyInteger;
begin
  FP.Expression:='4*2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPMultiplyOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(8);
end;

procedure TTestParserExpressions.TestSimpleMultiplyFloat;
begin
  FP.Expression:='3.4*1.5';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPMultiplyOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtFloat);
  AssertResult(5.1);
end;

procedure TTestParserExpressions.TestSimpleDivideInteger;
begin
  FP.Expression:='4/2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPDivideOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtFloat);
  AssertResult(2.0);
end;

procedure TTestParserExpressions.TestSimpleDivideFloat;
begin
  FP.Expression:='5.1/1.5';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPDivideOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtFloat);
  AssertResult(3.4);
end;

procedure TTestParserExpressions.TestSimpleBooleanAnd;
begin
  FP.Expression:='true and true';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserExpressions.TestSimpleIntegerAnd;
begin
  FP.Expression:='3 and 1';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(1);
end;

procedure TTestParserExpressions.TestSimpleBooleanOr;
begin
  FP.Expression:='false or true';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserExpressions.TestSimpleIntegerOr;
begin
  FP.Expression:='2 or 1';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(3);
end;

procedure TTestParserExpressions.TestSimpleBooleanNot;
begin
  FP.Expression:='not false';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Not node',TFPNotNode, FP.ExprNode);
  AssertOperand(FP.ExprNode,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(true);
end;

procedure TTestParserExpressions.TestSimpleIntegerNot;
begin
  FP.Expression:='Not 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Not node',TFPNotNode, FP.ExprNode);
  AssertOperand(FP.ExprNode,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(Not Int64(3));
end;

procedure TTestParserExpressions.TestSimpleAddSeries;
begin
  FP.Expression:='1 + 2 + 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPAddOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPAddOperation,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(6);
end;

procedure TTestParserExpressions.TestSimpleMultiplySeries;
begin
  FP.Expression:='2 * 3 * 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPMultiplyOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPMultiplyOperation,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(24);
end;

procedure TTestParserExpressions.TestSimpleAddMultiplySeries;
begin
  FP.Expression:='2 * 3 + 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPAddOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPMultiplyOperation,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(10);
end;

procedure TTestParserExpressions.TestSimpleAddAndSeries;
begin
  // 2 and (3+4)
  FP.Expression:='2 and 3 + 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPAddOperation);
  AssertResultType(rtInteger);
  AssertResult(2);
end;

procedure TTestParserExpressions.TestSimpleAddOrSeries;
begin
  // 2 or (3+4)
  FP.Expression:='2 or 3 + 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPAddOperation);
  AssertResultType(rtInteger);
  AssertResult(7);
end;

procedure TTestParserExpressions.TestSimpleOrNotSeries;
begin
  FP.Expression:='Not 1 or 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPNotNode,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult((Not Int64(1)) or Int64(3));
end;

procedure TTestParserExpressions.TestSimpleAndNotSeries;
begin
  FP.Expression:='Not False and False';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPNotNode,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserExpressions.TestDoubleAddMultiplySeries;
begin
  FP.Expression:='2 * 3 + 4 * 5';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPAddOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPMultiplyOperation,TFPMultiplyOperation);
  AssertResultType(rtInteger);
  AssertResult(26);
end;

procedure TTestParserExpressions.TestDoubleSubtractMultiplySeries;
begin
  FP.Expression:='4 * 5 - 2 * 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPSubtractOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPMultiplyOperation,TFPMultiplyOperation);
  AssertResultType(rtInteger);
  AssertResult(14);
end;

procedure TTestParserExpressions.TestSimpleIfInteger;
begin
  FP.Expression:='If(True,1,2)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('If operation',TIfOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(1);
end;

procedure TTestParserExpressions.TestSimpleIfString;
begin
  FP.Expression:='If(True,''a'',''b'')';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('If operation',TIfOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtString);
  AssertResult('a');
end;

procedure TTestParserExpressions.TestSimpleIfFloat;
begin
  FP.Expression:='If(True,1.2,3.4)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('If operation',TIfOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtFloat);
  AssertResult(1.2);
end;

procedure TTestParserExpressions.TestSimpleIfBoolean;
begin
  FP.Expression:='If(True,False,True)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('If operation',TIfOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserExpressions.TestSimpleIfDateTime;
begin
  FP.Identifiers.AddDateTimeVariable('a',Date);
  FP.Identifiers.AddDateTimeVariable('b',Date-1);
  FP.Expression:='If(True,a,b)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('If operation',TIfOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPExprVariable,TFPExprVariable);
  AssertResultType(rtDateTime);
  AssertResult(Date);
end;

procedure TTestParserExpressions.TestSimpleIfOperation;
begin
  FP.Expression:='If(True,''a'',''b'')+''c''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertResultType(rtString);
  AssertResult('ac');
end;

procedure TTestParserExpressions.TestSimpleBrackets;
begin
  FP.Expression:='(4 + 2)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPAddOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(6);
end;

procedure TTestParserExpressions.TestSimpleBrackets2;
begin
  FP.Expression:='(4 * 2)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPMultiplyOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(8);
end;

procedure TTestParserExpressions.TestSimpleBracketsLeft;
begin
  FP.Expression:='(4 + 2) * 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPMultiplyOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPAddOperation,TFPConstExpression);
  AssertResultType(rtInteger);
  AssertResult(18);
end;

procedure TTestParserExpressions.TestSimpleBracketsRight;
begin
  FP.Expression:='3 * (4 + 2)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPMultiplyOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPAddOperation);
  AssertResultType(rtInteger);
  AssertResult(18);
end;

procedure TTestParserExpressions.TestSimpleBracketsDouble;
begin
  FP.Expression:='(3 + 4) * (4 + 2)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPMultiplyOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPAddOperation,TFPAddOperation);
  AssertResultType(rtInteger);
  AssertResult(42);
end;

//TTestParserBooleanOperations

procedure TTestParserBooleanOperations.TestEqualInteger;
begin
  FP.Expression:='1 = 2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestUnEqualInteger;
begin
  FP.Expression:='1 <> 2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPUnEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestEqualFloat;
begin
  FP.Expression:='1.2 = 2.3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestEqualFloat2;
begin
  FP.Expression:='1.2 = 1.2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestUnEqualFloat;
begin
  FP.Expression:='1.2 <> 2.3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPUnEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;
procedure TTestParserBooleanOperations.TestEqualString;
begin
  FP.Expression:='''1.2'' = ''2.3''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestEqualString2;
begin
  FP.Expression:='''1.2'' = ''1.2''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestUnEqualString;
begin
  FP.Expression:='''1.2'' <> ''2.3''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPUnEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestUnEqualString2;
begin
  FP.Expression:='''aa'' <> ''AA''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPUnEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestEqualBoolean;
begin
  FP.Expression:='False = True';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestUnEqualBoolean;
begin
  FP.Expression:='False <> True';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPUnEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestLessThanInteger;
begin
  FP.Expression:='1 < 2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestLessThanInteger2;
begin
  FP.Expression:='2 < 2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestLessThanEqualInteger;
begin
  FP.Expression:='3 <= 2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestLessThanEqualInteger2;
begin
  FP.Expression:='2 <= 2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestLessThanFloat;
begin
  FP.Expression:='1.2 < 2.3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestLessThanFloat2;
begin
  FP.Expression:='2.2 < 2.2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestLessThanEqualFloat;
begin
  FP.Expression:='3.1 <= 2.1';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestLessThanEqualFloat2;
begin
  FP.Expression:='2.1 <= 2.1';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestLessThanString;
begin
  FP.Expression:='''1'' < ''2''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestLessThanString2;
begin
  FP.Expression:='''2'' < ''2''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestLessThanEqualString;
begin
  FP.Expression:='''3'' <= ''2''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestLessThanEqualString2;
begin
  FP.Expression:='''2'' <= ''2''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPLessThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;


procedure TTestParserBooleanOperations.TestGreaterThanInteger;
begin
  FP.Expression:='1 > 2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestGreaterThanInteger2;
begin
  FP.Expression:='2 > 2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestGreaterThanEqualInteger;
begin
  FP.Expression:='3 >= 2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestGreaterThanEqualInteger2;
begin
  FP.Expression:='2 >= 2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestGreaterThanFloat;
begin
  FP.Expression:='1.2 > 2.3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestGreaterThanFloat2;
begin
  FP.Expression:='2.2 > 2.2';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestGreaterThanEqualFloat;
begin
  FP.Expression:='3.1 >= 2.1';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestGreaterThanEqualFloat2;
begin
  FP.Expression:='2.1 >= 2.1';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestGreaterThanString;
begin
  FP.Expression:='''1'' > ''2''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestGreaterThanString2;
begin
  FP.Expression:='''2'' > ''2''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.TestGreaterThanEqualString;
begin
  FP.Expression:='''3'' >= ''2''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.TestGreaterThanEqualString2;
begin
  FP.Expression:='''2'' >= ''2''';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPGreaterThanEqualOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPConstExpression,TFPConstExpression);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.EqualAndSeries;
begin
  // (1=2) and (3=4)
  FP.Expression:='1 = 2 and 3 = 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPEqualOperation,TFPEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.EqualAndSeries2;
begin
  // (1=2) and (3=4)
  FP.Expression:='1 = 1 and 3 = 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPEqualOperation,TFPEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.EqualOrSeries;
begin
  // (1=2) or (3=4)
  FP.Expression:='1 = 2 or 3 = 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPEqualOperation,TFPEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.EqualOrSeries2;
begin
  // (1=1) or (3=4)
  FP.Expression:='1 = 1 or 3 = 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPEqualOperation,TFPEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.UnEqualAndSeries;
begin
  // (1<>2) and (3<>4)
  FP.Expression:='1 <> 2 and 3 <> 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPUnEqualOperation,TFPUnEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.UnEqualAndSeries2;
begin
  // (1<>2) and (3<>4)
  FP.Expression:='1 <> 1 and 3 <> 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPUnEqualOperation,TFPUnEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.UnEqualOrSeries;
begin
  // (1<>2) or (3<>4)
  FP.Expression:='1 <> 2 or 3 <> 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPUnEqualOperation,TFPUnEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.UnEqualOrSeries2;
begin
  // (1<>1) or (3<>4)
  FP.Expression:='1 <> 1 or 3 <> 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPUnEqualOperation,TFPUnEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.LessThanAndSeries;
begin
  // (1<2) and (3<4)
  FP.Expression:='1 < 2 and 3 < 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPLessThanOperation,TFPLessThanOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.LessThanAndSeries2;
begin
  // (1<2) and (3<4)
  FP.Expression:='1 < 1 and 3 < 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPLessThanOperation,TFPLessThanOperation);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.LessThanOrSeries;
begin
  // (1<2) or (3<4)
  FP.Expression:='1 < 2 or 3 < 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPLessThanOperation,TFPLessThanOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.LessThanOrSeries2;
begin
  // (1<1) or (3<4)
  FP.Expression:='1 < 1 or 3 < 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPLessThanOperation,TFPLessThanOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.GreaterThanAndSeries;
begin
  // (1>2) and (3>4)
  FP.Expression:='1 > 2 and 3 > 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPGreaterThanOperation,TFPGreaterThanOperation);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.GreaterThanAndSeries2;
begin
  // (1>2) and (3>4)
  FP.Expression:='1 > 1 and 3 > 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPGreaterThanOperation,TFPGreaterThanOperation);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.GreaterThanOrSeries;
begin
  // (1>2) or (3>4)
  FP.Expression:='1 > 2 or 3 > 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPGreaterThanOperation,TFPGreaterThanOperation);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.GreaterThanOrSeries2;
begin
  // (1>1) or (3>4)
  FP.Expression:='1 > 1 or 3 > 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPGreaterThanOperation,TFPGreaterThanOperation);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.LessThanEqualAndSeries;
begin
  // (1<=2) and (3<=4)
  FP.Expression:='1 <= 2 and 3 <= 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPLessThanEqualOperation,TFPLessThanEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.LessThanEqualAndSeries2;
begin
  // (1<=2) and (3<=4)
  FP.Expression:='1 <= 1 and 3 <= 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPLessThanEqualOperation,TFPLessThanEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.LessThanEqualOrSeries;
begin
  // (1<=2) or (3<=4)
  FP.Expression:='1 <= 2 or 3 <= 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPLessThanEqualOperation,TFPLessThanEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.LessThanEqualOrSeries2;
begin
  // (1<=1) or (3<=4)
  FP.Expression:='1 <= 1 or 3 <= 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPLessThanEqualOperation,TFPLessThanEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.GreaterThanEqualAndSeries;
begin
  // (1>=2) and (3>=4)
  FP.Expression:='1 >= 2 and 3 >= 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPGreaterThanEqualOperation,TFPGreaterThanEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.GreaterThanEqualAndSeries2;
begin
  // (1>=2) and (3>=4)
  FP.Expression:='1 >= 1 and 3 >= 3';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryAndOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPGreaterThanEqualOperation,TFPGreaterThanEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserBooleanOperations.GreaterThanEqualOrSeries;
begin
  // (1>=2) or (3>=4)
  FP.Expression:='1 >= 2 or 3 >= 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPGreaterThanEqualOperation,TFPGreaterThanEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(False);
end;

procedure TTestParserBooleanOperations.GreaterThanEqualOrSeries2;
begin
  // (1>=1) or (3>=4)
  FP.Expression:='1 >= 1 or 3 >= 4';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPBinaryOrOperation, FP.ExprNode);
  AssertLeftRight(FP.ExprNode,TFPGreaterThanEqualOperation,TFPGreaterThanEqualOperation);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

//TTestParserOperands
procedure TTestParserOperands.MissingOperand1;
begin
  TestParser('1+');
end;

procedure TTestParserOperands.MissingOperand2;
begin
  TestParser('*1');
end;

procedure TTestParserOperands.MissingOperand3;
begin
  TestParser('1*');
end;

procedure TTestParserOperands.MissingOperand4;
begin
  TestParser('1+');
end;

procedure TTestParserOperands.MissingOperand5;
begin
  TestParser('1 and');
end;

procedure TTestParserOperands.MissingOperand6;
begin
  TestParser('1 or');
end;

procedure TTestParserOperands.MissingOperand7;
begin
  TestParser('and 1');
end;

procedure TTestParserOperands.MissingOperand8;
begin
  TestParser('or 1');
end;

procedure TTestParserOperands.MissingOperand9;
begin
  TestParser('1-');
end;

procedure TTestParserOperands.MissingOperand10;
begin
  TestParser('1 = ');
end;

procedure TTestParserOperands.MissingOperand11;
begin
  TestParser('= 1');
end;

procedure TTestParserOperands.MissingOperand12;
begin
  TestParser('1 <> ');
end;

procedure TTestParserOperands.MissingOperand13;
begin
  TestParser('<> 1');
end;

procedure TTestParserOperands.MissingOperand14;
begin
  TestParser('1 >= ');
end;

procedure TTestParserOperands.MissingOperand15;
begin
  TestParser('>= 1');
end;

procedure TTestParserOperands.MissingOperand16;
begin
  TestParser('1 <= ');
end;

procedure TTestParserOperands.MissingOperand17;
begin
  TestParser('<= 1');
end;

procedure TTestParserOperands.MissingOperand18;
begin
  TestParser('1 < ');
end;

procedure TTestParserOperands.MissingOperand19;
begin
  TestParser('< 1');
end;

procedure TTestParserOperands.MissingOperand20;
begin
  TestParser('1 > ');
end;

procedure TTestParserOperands.MissingOperand21;
begin
  TestParser('> 1');
end;

procedure TTestParserOperands.MissingBracket1;
begin
  TestParser('(1+3');
end;

procedure TTestParserOperands.MissingBracket2;
begin
  TestParser('1+3)');
end;

procedure TTestParserOperands.MissingBracket3;
begin
  TestParser('(1+3))');
end;

procedure TTestParserOperands.MissingBracket4;
begin
  TestParser('((1+3)');
end;

procedure TTestParserOperands.MissingBracket5;
begin
  TestParser('((1+3) 4');
end;

procedure TTestParserOperands.MissingBracket6;
begin
  TestParser('IF(true,1,2');
end;

procedure TTestParserOperands.MissingBracket7;
begin
  TestParser('case(1,1,2,4');
end;

procedure TTestParserOperands.MissingArgument1;
begin
  TestParser('IF(true,1)');
end;

procedure TTestParserOperands.MissingArgument2;
begin
  TestParser('IF(True)');
end;

procedure TTestParserOperands.MissingArgument3;
begin
  TestParser('case(1)');
end;

procedure TTestParserOperands.MissingArgument4;
begin
  TestParser('case(1,2)');
end;

procedure TTestParserOperands.MissingArgument5;

begin
  TestParser('case(1,2,3)');
end;

procedure TTestParserOperands.MissingArgument6;

begin
  TestParser('IF(true,1,2,3)');
end;

procedure TTestParserOperands.MissingArgument7;

begin
  TestParser('case(0,1,2,3,4,5,6)');
end;

procedure TTestParserTypeMatch.AccessString;
begin
  FP.AsString;
end;

procedure TTestParserTypeMatch.AccessInteger;
begin
  FP.AsInteger;
end;

procedure TTestParserTypeMatch.AccessFloat;
begin
  FP.AsFloat;
end;

procedure TTestParserTypeMatch.AccessDateTime;
begin
  FP.AsDateTime;
end;

procedure TTestParserTypeMatch.AccessBoolean;
begin
  FP.AsBoolean;
end;

//TTestParserTypeMatch
procedure TTestParserTypeMatch.TestTypeMismatch1;
begin
  TestParser('1+''string''');
end;

procedure TTestParserTypeMatch.TestTypeMismatch2;
begin
  TestParser('1+True');
end;

procedure TTestParserTypeMatch.TestTypeMismatch3;
begin
  TestParser('True+''string''');
end;

procedure TTestParserTypeMatch.TestTypeMismatch4;
begin
  TestParser('1.23+''string''');
end;

procedure TTestParserTypeMatch.TestTypeMismatch5;
begin
  TestParser('1.23+true');
end;

procedure TTestParserTypeMatch.TestTypeMismatch6;
begin
  TestParser('1.23 and true');
end;

procedure TTestParserTypeMatch.TestTypeMismatch7;
begin
  TestParser('1.23 or true');
end;

procedure TTestParserTypeMatch.TestTypeMismatch8;
begin
  TestParser('''string'' or true');
end;

procedure TTestParserTypeMatch.TestTypeMismatch9;
begin
  TestParser('''string'' and true');
end;

procedure TTestParserTypeMatch.TestTypeMismatch10;
begin
  TestParser('1.23 or 1');
end;

procedure TTestParserTypeMatch.TestTypeMismatch11;
begin
  TestParser('1.23 and 1');
end;

procedure TTestParserTypeMatch.TestTypeMismatch12;
begin
  TestParser('''astring'' = 1');
end;

procedure TTestParserTypeMatch.TestTypeMismatch13;
begin
  TestParser('true = 1');
end;

procedure TTestParserTypeMatch.TestTypeMismatch14;
begin
  TestParser('true * 1');
end;

procedure TTestParserTypeMatch.TestTypeMismatch15;
begin
  TestParser('''astring'' * 1');
end;

procedure TTestParserTypeMatch.TestTypeMismatch16;
begin
  TestParser('If(1,1,1)');
end;

procedure TTestParserTypeMatch.TestTypeMismatch17;
begin
  TestParser('If(True,1,''3'')');
end;

procedure TTestParserTypeMatch.TestTypeMismatch18;
begin
  TestParser('case(1,1,''3'',1)');
end;

procedure TTestParserTypeMatch.TestTypeMismatch19;
begin
  TestParser('case(1,1,1,''3'')');
end;

procedure TTestParserTypeMatch.TestTypeMismatch20;
begin
  FP.Expression:='1';
  AssertException('Accessing integer as string',EExprParser,@AccessString);
end;

procedure TTestParserTypeMatch.TestTypeMismatch21;
begin
  FP.Expression:='''a''';
  AssertException('Accessing string as integer',EExprParser,@AccessInteger);
end;

procedure TTestParserTypeMatch.TestTypeMismatch22;
begin
  FP.Expression:='''a''';
  AssertException('Accessing string as float',EExprParser,@AccessFloat);
end;

procedure TTestParserTypeMatch.TestTypeMismatch23;
begin
  FP.Expression:='''a''';
  AssertException('Accessing string as boolean',EExprParser,@AccessBoolean);
end;

procedure TTestParserTypeMatch.TestTypeMismatch24;
begin
  FP.Expression:='''a''';
  AssertException('Accessing string as datetime',EExprParser,@AccessDateTime);
end;

//TTestParserVariables

Procedure GetDate(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=Date;
end;

procedure TTestParserVariables.TestVariable1;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddVariable('a',rtBoolean,'True');
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Variable has correct resulttype',rtBoolean,I.ResultType);
  AssertEquals('Variable has correct value','True',I.Value);
end;

procedure TTestParserVariables.TestVariable2;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddBooleanVariable('a',False);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Variable has correct resulttype',rtBoolean,I.ResultType);
  AssertEquals('Variable has correct value','False',I.Value);
end;

procedure TTestParserVariables.TestVariable3;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddIntegerVariable('a',123);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Variable has correct resulttype',rtInteger,I.ResultType);
  AssertEquals('Variable has correct value','123',I.Value);
end;

procedure TTestParserVariables.TestVariable4;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFloatVariable('a',1.23);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Variable has correct resulttype',rtFloat,I.ResultType);
  AssertEquals('Variable has correct value',FloatToStr(1.23),I.Value);
end;

procedure TTestParserVariables.TestVariable5;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddStringVariable('a','1.23');
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Variable has correct resulttype',rtString,I.ResultType);
  AssertEquals('Variable has correct value','1.23',I.Value);
end;

procedure TTestParserVariables.TestVariable6;
Var
  I : TFPExprIdentifierDef;
  D : TDateTime;

begin
  D:=Now;
  I:=FP.Identifiers.AddDateTimeVariable('a',D);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Variable has correct resulttype',rtDateTime,I.ResultType);
  AssertEquals('Variable has correct value',FormatDateTime('yyyy-mm-dd hh:nn:ss',D),I.Value);
end;

procedure TTestParserVariables.AddVariabletwice;

begin
  FP.Identifiers.AddDateTimeVariable('a',Now);
end;

procedure TTestParserVariables.UnknownVariable;
begin
  FP.Identifiers.IdentifierByName('unknown');
end;

procedure TTestParserVariables.ReadWrongType;

Var
  Res : TFPExpressioNResult;

begin
  AssertEquals('Only one identifier',1,FP.Identifiers.Count);
  Case FAsWrongType of
    rtBoolean  : res.ResBoolean:=FP.Identifiers[0].AsBoolean;
    rtString   : res.ResString:=FP.Identifiers[0].AsString;
    rtInteger  : Res.ResInteger:=FP.Identifiers[0].AsInteger;
    rtFloat    : Res.ResFloat:=FP.Identifiers[0].AsFloat;
    rtCurrency : Res.ResCurrency:=FP.Identifiers[0].AsCurrency;
    rtDateTime : Res.ResDateTime:=FP.Identifiers[0].AsDateTime;
  end;
end;

procedure TTestParserVariables.WriteWrongType;

Var
  Res : TFPExpressioNResult;

begin
  AssertEquals('Only one identifier',1,FP.Identifiers.Count);
  Case FAsWrongType of
    rtBoolean  : FP.Identifiers[0].AsBoolean:=res.ResBoolean;
    rtString   : FP.Identifiers[0].AsString:=res.ResString;
    rtInteger  : FP.Identifiers[0].AsInteger:=Res.ResInteger;
    rtFloat    : FP.Identifiers[0].AsFloat:=Res.ResFloat;
    rtCurrency : FP.Identifiers[0].AsCurrency:=Res.ResCurrency;
    rtDateTime : FP.Identifiers[0].AsDateTime:=Res.ResDateTime;
  end;
end;

procedure TTestParserVariables.DoDummy(var Result: TFPExpressionResult;
  const Args: TExprParameterArray);
begin
  // Do nothing;
end;

procedure TTestParserVariables.TestVariableAssign;

Var
  I,J : TFPExprIdentifierDef;

begin
  I:=TFPExprIdentifierDef.Create(Nil);
  try
    J:=TFPExprIdentifierDef.Create(Nil);
    try
      I.Name:='Aname';
      I.ParameterTypes:='ISDBF';
      I.ResultType:=rtFloat;
      I.Value:='1.23';
      I.OnGetFunctionValue:=@DoDummy;
      I.OnGetFunctionValueCallBack:=@GetDate;
      J.Assign(I);
      AssertEquals('Names match',I.Name,J.Name);
      AssertEquals('Parametertypes match',I.ParameterTypes,J.ParameterTypes);
      AssertEquals('Values match',I.Value,J.Value);
      AssertEquals('Result types match',Ord(I.ResultType),Ord(J.ResultType));
      AssertSame('Callbacks match',Pointer(I.OnGetFunctionValueCallBack),Pointer(J.OnGetFunctionValueCallback));
      If (I.OnGetFunctionValue)<>(J.OnGetFunctionValue) then
        Fail('OnGetFUnctionValue as Method does not match');
    finally
      J.Free;
    end;
  finally
    I.Free;
  end;
end;

procedure TTestParserVariables.TestVariableAssignAgain;

Var
  I,J : TFPBuiltinExprIdentifierDef;

begin
  I:=TFPBuiltinExprIdentifierDef.Create(Nil);
  try
    J:=TFPBuiltinExprIdentifierDef.Create(Nil);
    try
      I.Name:='Aname';
      I.ParameterTypes:='ISDBF';
      I.ResultType:=rtFloat;
      I.Value:='1.23';
      I.OnGetFunctionValue:=@DoDummy;
      I.OnGetFunctionValueCallBack:=@GetDate;
      I.Category:=bcUser;
      J.Assign(I);
      AssertEquals('Names match',I.Name,J.Name);
      AssertEquals('Parametertypes match',I.ParameterTypes,J.ParameterTypes);
      AssertEquals('Values match',I.Value,J.Value);
      AssertEquals('Result types match',Ord(I.ResultType),Ord(J.ResultType));
      AssertEquals('Categories match',Ord(I.Category),Ord(J.Category));
      AssertSame('Callbacks match',Pointer(I.OnGetFunctionValueCallBack),Pointer(J.OnGetFunctionValueCallback));
      If (I.OnGetFunctionValue)<>(J.OnGetFunctionValue) then
        Fail('OnGetFUnctionValue as Method does not match');
    finally
      J.Free;
    end;
  finally
    I.Free;
  end;
end;

procedure TTestParserVariables.TestVariable7;

Var
  I : TFPExprIdentifierDef;
  D : TDateTime;

begin
  D:=Now;
  I:=FP.Identifiers.AddDateTimeVariable('a',D);
  AssertNotNull('Addvariable returns result',I);
  AssertException('Cannot add same name twice',EExprParser,@AddVariabletwice);
end;

procedure TTestParserVariables.TestVariable8;

begin
  FP.Identifiers.AddIntegerVariable('a',123);
  FP.Identifiers.AddIntegerVariable('b',123);
  AssertEquals('List is dirty',True,FP.Dirty);
  FP.BuildHashList;
  FP.Identifiers.Delete(0);
  AssertEquals('List is dirty',True,FP.Dirty);
end;

procedure TTestParserVariables.TestVariable9;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddIntegerVariable('a',123);
  AssertNotNull('Addvariable returns result',I);
  FP.Expression:='a';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPExprVariable, FP.ExprNode);
  AssertResultType(rtInteger);
  AssertResult(123);
end;

procedure TTestParserVariables.TestVariable10;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddStringVariable('a','a123');
  AssertNotNull('Addvariable returns result',I);
  FP.Expression:='a';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPExprVariable, FP.ExprNode);
  AssertResultType(rtString);
  AssertResult('a123');
end;

procedure TTestParserVariables.TestVariable11;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFloatVariable('a',1.23);
  AssertNotNull('Addvariable returns result',I);
  FP.Expression:='a';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPExprVariable, FP.ExprNode);
  AssertResultType(rtFloat);
  AssertResult(1.23);
end;

procedure TTestParserVariables.TestVariable36;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddCurrencyVariable('a',1.23);
  AssertNotNull('Addvariable returns result',I);
  FP.Expression:='a';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPExprVariable, FP.ExprNode);
  AssertResultType(rtCurrency);
  AssertCurrencyResult(1.23);
end;

procedure TTestParserVariables.TestVariable12;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddBooleanVariable('a',True);
  AssertNotNull('Addvariable returns result',I);
  FP.Expression:='a';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPExprVariable, FP.ExprNode);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserVariables.TestVariable13;

Var
  I : TFPExprIdentifierDef;
  D : TDateTime;

begin
  D:=Date;
  I:=FP.Identifiers.AddDateTimeVariable('a',D);
  AssertNotNull('Addvariable returns result',I);
  FP.Expression:='a';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPExprVariable, FP.ExprNode);
  AssertResultType(rtDateTime);
  AssertDateTimeResult(D);
end;

procedure TTestParserVariables.TestVariable14;

Var
  I,S : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddIntegerVariable('a',1);
  FP.BuildHashList;
  S:=FP.IdentifierByName('a');
  AssertSame('Identifier found',I,S);
end;

procedure TTestParserVariables.TestVariable15;

Var
  I,S : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddIntegerVariable('a',1);
  AssertNotNull('Addvariable returns result',I);
  FP.BuildHashList;
  S:=FP.IdentifierByName('A');
  AssertSame('Identifier found',I,S);
end;

procedure TTestParserVariables.TestVariable16;

Var
  I,S : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddIntegerVariable('a',1);
  AssertNotNull('Addvariable returns result',I);
  FP.BuildHashList;
  S:=FP.IdentifierByName('B');
  AssertNull('Identifier not found',S);
end;

procedure TTestParserVariables.TestVariable17;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddIntegerVariable('a',1);
  AssertNotNull('Addvariable returns result',I);
  FP.BuildHashList;
  AssertException('Identifier not found',EExprParser,@unknownvariable);
end;

procedure TTestParserVariables.TestVariable18;

Var
  I,S : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddIntegerVariable('a',1);
  AssertNotNull('Addvariable returns result',I);
  S:=FP.Identifiers.FindIdentifier('B');
  AssertNull('Identifier not found',S);
end;

procedure TTestParserVariables.TestVariable19;

Var
  I,S : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddIntegerVariable('a',1);
  S:=FP.Identifiers.FindIdentifier('a');
  AssertSame('Identifier found',I,S);
end;

procedure TTestParserVariables.TestVariable20;

Var
  I,S : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddIntegerVariable('a',1);
  S:=FP.Identifiers.FindIdentifier('A');
  AssertSame('Identifier found',I,S);
end;

procedure TTestParserVariables.TestAccess(Skip : TResultType);

begin
  TestAccess([Skip]);
end;

procedure TTestParserVariables.TestAccess(Skip : TResultTypes);

Var
  rt : TResultType;

begin
  For rt:=Low(TResultType) to High(TResultType) do
    if Not (rt in skip) then
      begin
      FasWrongType:=rt;
      AssertException('Acces as '+ResultTypeName(rt),EExprParser,@ReadWrongtype);
      end;
  For rt:=Low(TResultType) to High(TResultType) do
    if Not (rt in skip) then
      begin
      FasWrongType:=rt;
      AssertException('Acces as '+ResultTypeName(rt),EExprParser,@WriteWrongtype);
      end;
end;

procedure TTestParserVariables.TestVariable21;
begin
  FP.IDentifiers.AddIntegerVariable('a',1);
  TestAccess([rtInteger]);
end;

procedure TTestParserVariables.TestVariable22;
begin
  FP.IDentifiers.AddFloatVariable('a',1.0);
  TestAccess([rtFloat]);
end;

procedure TTestParserVariables.TestVariable35;

begin
  FP.IDentifiers.AddCurrencyVariable('a',1.0);
  TestAccess([rtCurrency]);
end;

procedure TTestParserVariables.TestVariable23;
begin
  FP.IDentifiers.AddStringVariable('a','1.0');
  TestAccess(rtString);
end;

procedure TTestParserVariables.TestVariable24;
begin
  FP.IDentifiers.AddBooleanVariable('a',True);
  TestAccess(rtBoolean);
end;

procedure TTestParserVariables.TestVariable25;

begin
  FP.IDentifiers.AddDateTimeVariable('a',Date);
  TestAccess(rtDateTime);
end;

procedure TTestParserVariables.TestVariable26;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.IDentifiers.AddStringVariable('a','1.0');
  I.AsString:='12';
  AssertEquals('Correct value','12',I.AsString);
end;

procedure TTestParserVariables.TestVariable27;
Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.IDentifiers.AddIntegerVariable('a',10);
  I.Asinteger:=12;
  AssertEquals('Correct value',12,I.AsInteger);
end;

procedure TTestParserVariables.TestVariable28;
Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.IDentifiers.AddFloatVariable('a',1.0);
  I.AsFloat:=1.2;
  AssertEquals('Correct value',1.2,I.AsFloat);
end;

procedure TTestParserVariables.TestVariable29;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.IDentifiers.AddDateTimeVariable('a',Now);
  I.AsDateTime:=Date-1;
  AssertEquals('Correct value',Date-1,I.AsDateTime);
end;

procedure TTestParserVariables.TestVariable30;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddBooleanVariable('a',True);
  I.AsBoolean:=False;
  AssertEquals('Correct value',False,I.AsBoolean);
end;

procedure TTestParserVariables.DoGetBooleanVar(var Res: TFPExpressionResult;
  ConstRef AName: ShortString);

begin
  FEventName:=AName;
  Res.ResBoolean:=FBoolValue;
end;

procedure TTestParserVariables.TestVariable31;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddVariable('a',rtBoolean,@DoGetBooleanVar);
  AssertEquals('Correct name','a',i.Name);
  AssertEquals('Correct type',Ord(rtBoolean),Ord(i.ResultType));
  AssertSame(TMethod(I.OnGetVariableValue).Code,TMethod(@DoGetBooleanVar).Code);
  FBoolValue:=True;
  FEventName:='';
  AssertEquals('Correct value 1',True,I.AsBoolean);
  AssertEquals('Correct name passed','a',FEventName);
  FBoolValue:=False;
  FEventName:='';
  AssertEquals('Correct value 2',False,I.AsBoolean);
  AssertEquals('Correct name passed','a',FEventName);
end;

Var
  FVarCallBackName:String;
  FVarBoolValue : Boolean;

procedure DoGetBooleanVar2(var Res: TFPExpressionResult; ConstRef AName: ShortString);

begin
  FVarCallBackName:=AName;
  Res.ResBoolean:=FVarBoolValue;
end;

procedure TTestParserVariables.DoGetBooleanVarWrong(var Res: TFPExpressionResult; ConstRef AName: ShortString);

begin
  FEventName:=AName;
  Res.ResultType:=rtInteger;
  Res.ResInteger:=33;
end;

procedure TTestParserVariables.TestVariable32;
Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddVariable('a',rtBoolean,@DoGetBooleanVar2);
  AssertEquals('Correct name','a',i.Name);
  AssertEquals('Correct type',Ord(rtBoolean),Ord(i.ResultType));
  AssertSame(I.OnGetVariableValueCallBack,@DoGetBooleanVar2);
  FVarBoolValue:=True;
  FVarCallBackName:='';
  AssertEquals('Correct value 1',True,I.AsBoolean);
  AssertEquals('Correct name passed','a',FVarCallBackName);
  FVarBoolValue:=False;
  FVarCallBackName:='';
  AssertEquals('Correct value 2',False,I.AsBoolean);
  AssertEquals('Correct name passed','a',FVarCallBackName);
end;

procedure TTestParserVariables.DoTestVariable33;

Var
  B : Boolean;

begin
  B:=FTest33.AsBoolean;
  AssertTrue(B in [true,False])
end;

procedure TTestParserVariables.TestVariable33;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddVariable('a',rtBoolean,@DoGetBooleanVarWrong);
  FTest33:=I;
  AssertException('Changing type results in exception',EExprParser,@DoTestVariable33);
  AssertEquals('Type is unchanged',Ord(rtBoolean),Ord(i.ResultType));
end;


procedure DoGetBooleanVar2Wrong(var Res: TFPExpressionResult; ConstRef AName: ShortString);

begin
  FVarCallBackName:=AName;
  Res.ResultType:=rtInteger;
  Res.ResInteger:=34;
end;

procedure TTestParserVariables.TestVariable34;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddVariable('a',rtBoolean,@DoGetBooleanVar2Wrong);
  FTest33:=I;
  AssertException('Changing type results in exception',EExprParser,@DoTestVariable33);
  AssertEquals('Type is unchanged',Ord(rtBoolean),Ord(i.ResultType));
end;



Procedure EchoDate(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=Args[0].resDateTime;
end;

Procedure EchoInteger(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resInteger:=Args[0].resInteger;
end;

Procedure EchoBoolean(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resBoolean:=Args[0].resBoolean;
end;

Procedure EchoFloat(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resFloat:=Args[0].resFloat;
end;

Procedure EchoCurrency(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resCurrency:=Args[0].resCurrency;
end;

Procedure EchoString(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=Args[0].resString;
end;

Procedure TTestExpressionParser.DoEchoDate(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=Args[0].resDateTime;
end;

Procedure TTestExpressionParser.DoEchoInteger(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resInteger:=Args[0].resInteger;
end;

Procedure TTestExpressionParser.DoEchoBoolean(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resBoolean:=Args[0].resBoolean;
end;

Procedure TTestExpressionParser.DoEchoFloat(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resFloat:=Args[0].resFloat;
end;

Procedure TTestExpressionParser.DoEchoCurrency(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resCurrency:=Args[0].resCurrency;
end;

Procedure TTestExpressionParser.DoEchoString(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=Args[0].resString;
end;

procedure TTestExpressionParser.DoGetDate(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
begin
  Result.ResDatetime:=Date;
end;

procedure TTestExpressionParser.DoAddInteger(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
begin
  Result.Resinteger:=Args[0].ResInteger+Args[1].ResInteger;
end;

procedure TTestExpressionParser.DoDeleteString(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
begin
  Result.ResString:=Args[0].ResString;
  Delete(Result.ResString,Args[1].ResInteger,Args[2].ResInteger);
end;

procedure TTestParserFunctions.TryRead;

Var
  Res : TFPExpressioNResult;

begin
  AssertEquals('Only one identifier',1,FP.Identifiers.Count);
  Case FAccessAs of
    rtBoolean  : res.ResBoolean:=FP.Identifiers[0].AsBoolean;
    rtString   : res.ResString:=FP.Identifiers[0].AsString;
    rtInteger  : Res.ResInteger:=FP.Identifiers[0].AsInteger;
    rtFloat    : Res.ResFloat:=FP.Identifiers[0].AsFloat;
    rtCurrency : Res.ResCurrency:=FP.Identifiers[0].AsCurrency;
    rtDateTime : Res.ResDateTime:=FP.Identifiers[0].AsDateTime;
  end;
end;

procedure TTestParserFunctions.TryWrite;

Var
  Res : TFPExpressioNResult;

begin
  Res:=Default(TFPExpressioNResult);
  AssertEquals('Only one identifier',1,FP.Identifiers.Count);
  Case FAccessAs of
    rtBoolean  : FP.Identifiers[0].AsBoolean:=res.ResBoolean;
    rtString   : FP.Identifiers[0].AsString:=res.ResString;
    rtInteger  : FP.Identifiers[0].AsInteger:=Res.ResInteger;
    rtFloat    : FP.Identifiers[0].AsFloat:=Res.ResFloat;
    rtCurrency : FP.Identifiers[0].AsCurrency:=Res.ResCurrency;
    rtDateTime : FP.Identifiers[0].AsDateTime:=Res.ResDateTime;
  end;
end;

// TTestParserFunctions
procedure TTestParserFunctions.TestFunction1;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('Date','D','',@GetDate);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtDateTime,I.ResultType);
  AssertSame('Function has correct address',Pointer(@GetDate),Pointer(I.OnGetFunctionValueCallBack));
  FaccessAs:=rtDateTime;
  AssertException('No read access',EExprParser,@TryRead);
  AssertException('No write access',EExprParser,@TryWrite);
end;

procedure TTestParserFunctions.TestFunction2;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoDate','D','D',@EchoDate);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtDateTime,I.ResultType);
  AssertSame('Function has correct address',Pointer(@EchoDate),Pointer(I.OnGetFunctionValueCallBack));
end;

procedure TTestParserFunctions.TestFunction3;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoInteger','I','I',@EchoInteger);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtInteger,I.ResultType);
  AssertSame('Function has correct address',Pointer(@EchoInteger),Pointer(I.OnGetFunctionValueCallBack));
  FaccessAs:=rtInteger;
  AssertException('No read access',EExprParser,@TryRead);
  AssertException('No write access',EExprParser,@TryWrite);
end;

procedure TTestParserFunctions.TestFunction4;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoBoolean','B','B',@EchoBoolean);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtBoolean,I.ResultType);
  AssertSame('Function has correct address',Pointer(@EchoBoolean),Pointer(I.OnGetFunctionValueCallBack));
  FaccessAs:=rtBoolean;
  AssertException('No read access',EExprParser,@TryRead);
  AssertException('No write access',EExprParser,@TryWrite);
end;

procedure TTestParserFunctions.TestFunction5;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoFloat','F','F',@EchoFloat);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtFloat,I.ResultType);
  AssertSame('Function has correct address',Pointer(@EchoFloat),Pointer(I.OnGetFunctionValueCallBack));
  FaccessAs:=rtfloat;
  AssertException('No read access',EExprParser,@TryRead);
  AssertException('No write access',EExprParser,@TryWrite);
end;

procedure TTestParserFunctions.TestFunction30;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoCurrency','C','C',@EchoCurrency);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtCurrency,I.ResultType);
  AssertSame('Function has correct address',Pointer(@EchoCurrency),Pointer(I.OnGetFunctionValueCallBack));
  FaccessAs:=rtCurrency;
  AssertException('No read access',EExprParser,@TryRead);
  AssertException('No write access',EExprParser,@TryWrite);
end;

procedure TTestParserFunctions.TestFunction6;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoString','S','S',@EchoString);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtString,I.ResultType);
  AssertSame('Function has correct address',Pointer(@EchoString),Pointer(I.OnGetFunctionValueCallBack));
  FaccessAs:=rtString;
  AssertException('No read access',EExprParser,@TryRead);
  AssertException('No write access',EExprParser,@TryWrite);
end;

procedure TTestParserFunctions.TestFunction7;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoDate','D','D',@DoEchoDate);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtDateTime,I.ResultType);
//  AssertSame('Function has correct address',TMethod(@Self.DoEchoDate),TMethod(I.OnGetFunctionValue));
end;

procedure TTestParserFunctions.TestFunction8;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoInteger','I','I',@DOEchoInteger);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtInteger,I.ResultType);
//  AssertSame('Function has correct address',Pointer(@EchoInteger),Pointer(I.OnGetFunctionValueCallBack));
end;

procedure TTestParserFunctions.TestFunction9;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoBoolean','B','B',@DoEchoBoolean);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtBoolean,I.ResultType);
//  AssertSame('Function has correct address',Pointer(@EchoBoolean),Pointer(I.OnGetFunctionValueCallBack));
end;

procedure TTestParserFunctions.TestFunction10;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoFloat','F','F',@DoEchoFloat);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtFloat,I.ResultType);
//  AssertSame('Function has correct address',Pointer(@EchoFloat),Pointer(I.OnGetFunctionValueCallBack));
end;

procedure TTestParserFunctions.TestFunction31;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoCurrency','C','C',@DoEchoCurrency);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtCurrency,I.ResultType);
//  AssertSame('Function has correct address',Pointer(@EchoFloat),Pointer(I.OnGetFunctionValueCallBack));
end;

procedure TTestParserFunctions.TestFunction11;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoString','S','S',@DoEchoString);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtString,I.ResultType);
//  AssertSame('Function has correct address',Pointer(@EchoString),Pointer(I.OnGetFunctionValueCallBack));
end;

procedure TTestParserFunctions.TestFunction12;

Var
  I : TFPExprIdentifierDef;
  D : TDateTime;

begin
  D:=Date;
  I:=FP.Identifiers.AddFunction('Date','D','',@GetDate);
  AssertNotNull('Addvariable returns result',I);
  FP.Expression:='Date';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionCallBack, FP.ExprNode);
  AssertResultType(rtDateTime);
  AssertDateTimeResult(D);
end;

procedure TTestParserFunctions.TestFunction13;

Var
  I : TFPExprIdentifierDef;
  D : TDateTime;

begin
  D:=Date;
  I:=FP.Identifiers.AddDateTimeVariable('a',D);
  AssertNotNull('Addvariable returns result',I);
  I:=FP.Identifiers.AddFunction('EchoDate','D','D',@EchoDate);
  AssertNotNull('Addvariable returns result',I);
  FP.Expression:='EchoDate(a)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionCallBack, FP.ExprNode);
  AssertResultType(rtDateTime);
  AssertDateTimeResult(D);
end;

procedure TTestParserFunctions.TestFunction14;
Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoInteger','I','I',@EchoInteger);
  AssertNotNull('Addvariable returns result',I);
  FP.Expression:='EchoInteger(13)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionCallBack, FP.ExprNode);
  AssertResultType(rtInteger);
  AssertResult(13);
end;

procedure TTestParserFunctions.TestFunction15;
Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoBoolean','B','B',@EchoBoolean);
  AssertNotNull('Addvariable returns result',I);
  FP.Expression:='EchoBoolean(True)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionCallBack, FP.ExprNode);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserFunctions.TestFunction16;
Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoFloat','F','F',@EchoFloat);
  AssertNotNull('Have identifier',I);
  FP.Expression:='EchoFloat(1.234)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionCallBack, FP.ExprNode);
  AssertResultType(rtFloat);
  AssertResult(1.234);
end;

procedure TTestParserFunctions.TestFunction32;

Var
  I : TFPExprIdentifierDef;

begin
  // Note there will be an implicit conversion float-> currency as the const will be a float
  I:=FP.Identifiers.AddFunction('EchoCurrency','C','C',@EchoCurrency);
  AssertNotNull('Have identifier',I);
  FP.Expression:='EchoCurrency(1.234)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionCallBack, FP.ExprNode);
  AssertResultType(rtCurrency);
  AssertCurrencyResult(1.234);
end;

procedure TTestParserFunctions.TestFunction33;
Var
  I : TFPExprIdentifierDef;

begin
  // Note there will be no conversion
  I:=FP.Identifiers.AddCurrencyVariable('a',1.234);
  AssertNotNull('Have identifier',I);
  I:=FP.Identifiers.AddFunction('EchoCurrency','C','C',@EchoCurrency);
  AssertNotNull('Have identifier',I);
  FP.Expression:='EchoCurrency(a)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionCallBack, FP.ExprNode);
  AssertResultType(rtCurrency);
  AssertCurrencyResult(1.234);
end;

procedure TTestParserFunctions.ExprMaxOf(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
var
  mx: Double;
  arg: TFPExpressionResult;
begin
  mx := -MaxDouble;
  for arg in Args do
    mx := math.Max(mx, ArgToFloat(arg));
  result.ResFloat:= mx;
end;

procedure TTestParserFunctions.ExprMinOf(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
var
  mn: Double;
  arg: TFPExpressionResult;
begin
  mn := MaxDouble;
  for arg in Args do
    mn := math.Min(mn, ArgToFloat(arg));
  result.ResFloat:= mn;
end;

procedure TTestParserFunctions.ExprSumOf(var Result: TFPExpressionResult; Const Args: TExprParameterArray);
var
  sum: Double;
  arg: TFPExpressionResult;
begin
  sum := 0;
  for arg in Args do
    sum := sum + ArgToFloat(arg);
  Result.ResFloat := sum;
end;

procedure TTestParserFunctions.ExprAveOf(var Result: TFPExpressionResult; const Args: TExprParameterArray);
var
  sum: Double;
  arg: TFPExpressionResult;
begin
  if Length(Args) = 0 then
    raise EExprParser.Create('At least 1 value needed for calculation of average');
  sum := 0;
  for arg in Args do
    sum := sum + ArgToFloat(arg);
  Result.ResFloat := sum / Length(Args);
end;

procedure TTestParserFunctions.ExprStdDevOf(var Result: TFPExpressionResult; const Args: TExprParameterArray);
var
  sum, ave: Double;
  arg: TFPExpressionResult;
begin
  if Length(Args) < 2 then
    raise EExprParser.Create('At least 2 values needed for calculation of standard deviation');
  sum := 0;
  for arg in Args do
    sum := sum + ArgToFloat(arg);
  ave := sum / Length(Args);
  sum := 0;
  for arg in Args do
    sum := sum + sqr(ArgToFloat(arg) - ave);
  Result.ResFloat := sqrt(sum / (Length(Args) - 1));
end;

procedure TTestParserFunctions.TestVarArgs1;
begin
 // FP.BuiltIns := [bcMath];
  FP.Identifiers.AddFunction('MaxOf', 'F', 'F+', @ExprMaxOf);
  FP.Expression := 'MaxOf(-1,2,3,4.1)';
  AssertEquals('Result',4.1,FP.Evaluate.ResFloat,0.1);
end;

procedure TTestParserFunctions.TestVarArgs2;
begin
  FP.Identifiers.AddFunction('MinOf', 'F', 'F+', @ExprMinOf);
  FP.Expression := 'MinOf(-1,2,3,4.1)';
  AssertEquals('Result',-1,FP.Evaluate.ResFloat,0.1);
end;

procedure TTestParserFunctions.TestVarArgs3;
begin
  FP.Identifiers.AddFunction('SumOf', 'F', 'F+', @ExprSumOf);
  FP.Expression := 'SumOf(-1,2,3,4.1)';
  AssertEquals('Result',8.1,FP.Evaluate.ResFloat,0.1);
end;

procedure TTestParserFunctions.TestVarArgs4;
begin
  FP.Identifiers.AddFunction('AveOf', 'F', 'F+', @ExprAveOf);
  FP.Expression := 'AveOf(-1,2,3,4.1)';
  AssertEquals('Result',2.025,FP.Evaluate.ResFloat,0.001);
end;

procedure TTestParserFunctions.TestVarArgs5;
begin
  FP.Identifiers.AddFunction('StdDevOf', 'F', 'F+', @ExprStdDevOf);
  FP.Expression := 'StdDevOf(-1,2,3,4.1)';
  AssertEquals('Result',2.191,FP.Evaluate.ResFloat,0.001);
end;

procedure TTestParserFunctions.TestFunction17;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoString','S','S',@EchoString);
  AssertNotNull('Have identifier',I);
  FP.Expression:='EchoString(''Aloha'')';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionCallBack, FP.ExprNode);
  AssertResultType(rtString);
  AssertResult('Aloha');
end;


procedure TTestParserFunctions.TestFunction18;

Var
  I : TFPExprIdentifierDef;
  D : TDateTime;

begin
  D:=Date;
  I:=FP.Identifiers.AddDateTimeVariable('a',D);
  AssertNotNull('Have identifier',I);
  I:=FP.Identifiers.AddFunction('EchoDate','D','D',@DoEchoDate);
  AssertNotNull('Have identifier',I);
  FP.Expression:='EchoDate(a)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtDateTime);
  AssertDateTimeResult(D);
end;

procedure TTestParserFunctions.TestFunction19;
Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoInteger','I','I',@DoEchoInteger);
  AssertNotNull('Have identifier',I);
  FP.Expression:='EchoInteger(13)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtInteger);
  AssertResult(13);
end;

procedure TTestParserFunctions.TestFunction20;
Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoBoolean','B','B',@DoEchoBoolean);
  AssertNotNull('Have identifier',I);
  FP.Expression:='EchoBoolean(True)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtBoolean);
  AssertResult(True);
end;

procedure TTestParserFunctions.TestFunction21;
Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoFloat','F','F',@DoEchoFloat);
  AssertNotNull('Have identifier',I);
  FP.Expression:='EchoFloat(1.234)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtFloat);
  AssertResult(1.234);
end;

procedure TTestParserFunctions.TestFunction22;
Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('EchoString','S','S',@DoEchoString);
  AssertNotNull('Have identifier',I);
  FP.Expression:='EchoString(''Aloha'')';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtString);
  AssertResult('Aloha');
end;

procedure TTestParserFunctions.TestFunction23;

Var
  I : TFPExprIdentifierDef;
  D : TDateTime;

begin
  D:=Date;
  I:=FP.Identifiers.AddFunction('Date','D','',@DoGetDate);
  AssertNotNull('Have identifier',I);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtDateTime,I.ResultType);
  FP.Expression:='Date';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtDateTime);
  AssertDateTimeResult(D);
end;

procedure TTestParserFunctions.TestFunction24;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('AddInteger','I','II',@DoAddInteger);
  AssertNotNull('Have identifier',I);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtInteger,I.ResultType);
  FP.Expression:='AddInteger(1,2)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtInteger);
  AssertResult(3);
end;

procedure TTestParserFunctions.TestFunction25;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('Delete','S','SII',@DoDeleteString);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Have identifier',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtString,I.ResultType);
  FP.Expression:='Delete(''ABCDEFGHIJ'',3,2)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtString);
  AssertResult('ABEFGHIJ');
end;

procedure TTestParserFunctions.TestFunction26;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('AddInteger','I','II',@DoAddInteger);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtInteger,I.ResultType);
  FP.Expression:='AddInteger(1,2+3)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtInteger);
  AssertResult(6);
end;

procedure TTestParserFunctions.TestFunction27;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('AddInteger','I','II',@DoAddInteger);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtInteger,I.ResultType);
  FP.Expression:='AddInteger(1+2,3*4)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtInteger);
  AssertResult(15);
end;

procedure TTestParserFunctions.TestFunction28;

Var
  I : TFPExprIdentifierDef;

begin
  I:=FP.Identifiers.AddFunction('AddInteger','I','II',@DoAddInteger);
  AssertEquals('List is dirty',True,FP.Dirty);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FP.Identifiers.Count);
  AssertSame('Result equals variable added',I,FP.Identifiers[0]);
  AssertEquals('Function has correct resulttype',rtInteger,I.ResultType);
  FP.Expression:='AddInteger(3 and 2,3*4)';
  AssertNotNull('Have result node',FP.ExprNode);
  AssertNodeType('Constant expression',TFPFunctionEventHandler, FP.ExprNode);
  AssertResultType(rtInteger);
  AssertResult(14);
end;

procedure TTestParserFunctions.TestFunction29;

Var
  I : TFPExprIdentifierDef;

begin
  // Test type mismatch
  I:=FP.Identifiers.AddFunction('AddInteger','I','II',@DoAddInteger);
  AssertNotNull('Addvariable returns result',I);
  TestParser('AddInteger(3 and 2,''s'')');
end;

{ TTestBuiltinsManager }

procedure TTestBuiltinsManager.Setup;
begin
  inherited Setup;
  FM:=TExprBuiltInManager.Create(Nil);
end;

procedure TTestBuiltinsManager.Teardown;
begin
  FreeAndNil(FM);
  inherited Teardown;
end;

procedure TTestBuiltinsManager.TestCreate;
begin
  AssertEquals('Have no builtin expressions',0,FM.IdentifierCount);
end;

procedure TTestBuiltinsManager.TestVariable1;

Var
  I : TFPBuiltinExprIdentifierDef;

begin
  I:=FM.AddVariable(bcuser,'a',rtBoolean,'True');
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FM.IdentifierCount);
  AssertSame('Result equals variable added',I,FM.Identifiers[0]);
  AssertEquals('Variable has correct category',ord(bcUser),Ord(I.Category));
  AssertEquals('Variable has correct resulttype',rtBoolean,I.ResultType);
  AssertEquals('Variable has correct value','True',I.Value);
end;

procedure TTestBuiltinsManager.TestVariable2;

Var
  I : TFPBuiltinExprIdentifierDef;

begin
  I:=FM.AddBooleanVariable(bcUser,'a',False);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FM.IdentifierCount);
  AssertSame('Result equals variable added',I,FM.Identifiers[0]);
  AssertEquals('Variable has correct category',ord(bcUser),Ord(I.Category));
  AssertEquals('Variable has correct resulttype',rtBoolean,I.ResultType);
  AssertEquals('Variable has correct value','False',I.Value);
end;

procedure TTestBuiltinsManager.TestVariable3;

Var
  I : TFPBuiltinExprIdentifierDef;

begin
  I:=FM.AddIntegerVariable(bcUser,'a',123);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FM.IdentifierCount);
  AssertSame('Result equals variable added',I,FM.Identifiers[0]);
  AssertEquals('Variable has correct category',ord(bcUser),Ord(I.Category));
  AssertEquals('Variable has correct resulttype',rtInteger,I.ResultType);
  AssertEquals('Variable has correct value','123',I.Value);
end;

procedure TTestBuiltinsManager.TestVariable4;

Var
  I : TFPBuiltinExprIdentifierDef;

begin
  I:=FM.AddFloatVariable(bcUser,'a',1.23);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FM.IdentifierCount);
  AssertSame('Result equals variable added',I,FM.Identifiers[0]);
  AssertEquals('Variable has correct category',ord(bcUser),Ord(I.Category));
  AssertEquals('Variable has correct resulttype',rtFloat,I.ResultType);
  AssertEquals('Variable has correct value',FloatToStr(1.23),I.Value);
end;

procedure TTestBuiltinsManager.TestVariable7;

Var
  I : TFPBuiltinExprIdentifierDef;

begin
  I:=FM.AddCurrencyVariable(bcUser,'a',1.23);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FM.IdentifierCount);
  AssertSame('Result equals variable added',I,FM.Identifiers[0]);
  AssertEquals('Variable has correct category',ord(bcUser),Ord(I.Category));
  AssertEquals('Variable has correct resulttype',rtCurrency,I.ResultType);
  AssertEquals('Variable has correct value',CurrToStr(1.23),I.Value);
end;

procedure TTestBuiltinsManager.TestVariable5;

Var
  I : TFPBuiltinExprIdentifierDef;

begin
  I:=FM.AddStringVariable(bcUser,'a','1.23');
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FM.IdentifierCount);
  AssertSame('Result equals variable added',I,FM.Identifiers[0]);
  AssertEquals('Variable has correct category',ord(bcUser),Ord(I.Category));
  AssertEquals('Variable has correct resulttype',rtString,I.ResultType);
  AssertEquals('Variable has correct value','1.23',I.Value);
end;

procedure TTestBuiltinsManager.TestVariable6;
Var
  I : TFPBuiltinExprIdentifierDef;
  D : TDateTime;

begin
  D:=Now;
  I:=FM.AddDateTimeVariable(bcUser,'a',D);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FM.IdentifierCount);
  AssertSame('Result equals variable added',I,FM.Identifiers[0]);
  AssertEquals('Variable has correct category',ord(bcUser),Ord(I.Category));
  AssertEquals('Variable has correct resulttype',rtDateTime,I.ResultType);
  AssertEquals('Variable has correct value',FormatDateTime('yyyy-mm-dd hh:nn:ss',D),I.Value);
end;

procedure TTestBuiltinsManager.TestFunction1;

Var
  I : TFPBuiltinExprIdentifierDef;

begin
  I:=FM.AddFunction(bcUser,'Date','D','',@GetDate);
  AssertNotNull('Addvariable returns result',I);
  AssertEquals('One variable added',1,FM.IdentifierCount);
  AssertSame('Result equals variable added',I,FM.Identifiers[0]);
  AssertEquals('Variable has correct category',ord(bcUser),Ord(I.Category));
  AssertEquals('Function has correct resulttype',rtDateTime,I.ResultType);
  AssertSame('Function has correct address',Pointer(@GetDate),Pointer(I.OnGetFunctionValueCallBack));
end;

procedure TTestBuiltinsManager.TestFunction2;

Var
  I,I2 : TFPBuiltinExprIdentifierDef;
  ind : Integer;

begin
  FM.AddFunction(bcUser,'EchoDate','D','D',@EchoDate);
  I:=FM.AddFunction(bcUser,'Echo','D','D',@EchoDate);
  FM.AddFunction(bcUser,'DoEcho','D','D',@EchoDate);
  ind:=FM.IndexOfIdentifier('Echo');
  AssertEquals('Found identifier',1,ind);
  I2:=FM.FindIdentifier('Echo');
  AssertNotNull('FindIdentifier returns result',I2);
  AssertSame('Findidentifier returns correct result',I,I2);
  ind:=FM.IndexOfIdentifier('NoNoNo');
  AssertEquals('Found no such identifier',-1,ind);
  I2:=FM.FindIdentifier('NoNoNo');
  AssertNull('FindIdentifier returns no result',I2);
end;

procedure TTestBuiltinsManager.TestDelete;

begin
  FM.AddFunction(bcUser,'EchoDate','D','D',@EchoDate);
  FM.AddFunction(bcUser,'EchoDate2','D','D',@EchoDate);
  FM.AddFunction(bcUser,'EchoDate3','D','D',@EchoDate);
  AssertEquals('Count before',3,FM.IdentifierCount);
  FM.Delete(2);
  AssertEquals('Count after',2,FM.IdentifierCount);
  AssertEquals('No more',-1,FM.IndexOfIdentifier('EchoDate3'));
  AssertEquals('Left 1',0,FM.IndexOfIdentifier('EchoDate'));
  AssertEquals('Left 2',1,FM.IndexOfIdentifier('EchoDate2'));
end;

procedure TTestBuiltinsManager.TestRemove;
begin
  FM.AddFunction(bcUser,'EchoDate','D','D',@EchoDate);
  FM.AddFunction(bcUser,'EchoDate2','D','D',@EchoDate);
  FM.AddFunction(bcUser,'EchoDate3','D','D',@EchoDate);
  AssertEquals('Count before',3,FM.IdentifierCount);
  AssertEquals('Result ',1,FM.Remove('EchoDate2'));
  AssertEquals('Count after',2,FM.IdentifierCount);
  AssertEquals('No more',-1,FM.IndexOfIdentifier('EchoDate2'));
  AssertEquals('Left 1',0,FM.IndexOfIdentifier('EchoDate'));
  AssertEquals('Left 2',1,FM.IndexOfIdentifier('EchoDate3'));
  AssertEquals('Result ',-1,FM.Remove('Nono'));
end;

{ TTestBuiltins }

procedure TTestBuiltins.Setup;
begin
  inherited Setup;
  FM:=TExprBuiltInManager.Create(Nil);
  FValue:=0;
end;

procedure TTestBuiltins.Teardown;
begin
  FreeAndNil(FM);
  inherited Teardown;
end;

procedure TTestBuiltins.SetExpression(const AExpression: String);

Var
  Msg : String;

begin
  Msg:='';
  try
    FP.Expression:=AExpression;
  except
    On E : Exception do
      Msg:=E.message;
  end;
  If (Msg<>'') then
    Fail('Parsing of expression "'+AExpression+'" failed :'+Msg);
end;

procedure TTestBuiltins.AssertVariable(const ADefinition: String;
  AResultType: TResultType);

Var
  I : TFPBuiltinExprIdentifierDef;

begin
  I:=FM.FindIdentifier(ADefinition);
  AssertNotNull('Definition '+ADefinition+' is present.',I);
  AssertEquals('Correct result type',AResultType,I.ResultType);
end;

procedure TTestBuiltins.AssertFunction(const ADefinition, AResultType,
  ArgumentTypes: String; ACategory : TBuiltinCategory);

Var
  I : TFPBuiltinExprIdentifierDef;

begin
  I:=FM.FindIdentifier(ADefinition);
  AssertEquals('Correct result type for test',1,Length(AResultType));
  AssertNotNull('Definition '+ADefinition+' is present.',I);
  AssertEquals(ADefinition+' has correct parameter types',ArgumentTypes,I.ParameterTypes);
  AssertEquals(ADefinition+' has correct result type',CharToResultType(AResultType[1]),I.ResultType);
  AssertEquals(ADefinition+' has correct category',Ord(ACategory),Ord(I.Category));
end;

procedure TTestBuiltins.AssertExpression(const AExpression: String;
  AResult: Int64);

begin
  FP.BuiltIns:=AllBuiltIns;
  SetExpression(AExpression);
  AssertResult(AResult);
end;

procedure TTestBuiltins.AssertExpression(const AExpression: String;
  const AResult: String);
begin
  FP.BuiltIns:=AllBuiltIns;
  SetExpression(AExpression);
  AssertResult(AResult);
end;

procedure TTestBuiltins.AssertExpression(const AExpression: String;
  const AResult: TExprFloat);
begin
  FP.BuiltIns:=AllBuiltIns;
  SetExpression(AExpression);
  AssertResult(AResult);
end;

procedure TTestBuiltins.AssertExpression(const AExpression: String;
  const AResult: Boolean);
begin
  FP.BuiltIns:=AllBuiltIns;
  SetExpression(AExpression);
  AssertResult(AResult);
end;

procedure TTestBuiltins.AssertDateTimeExpression(const AExpression: String;
  const AResult: TDateTime);
begin
  FP.BuiltIns:=AllBuiltIns;
  SetExpression(AExpression);
  AssertDatetimeResult(AResult);
end;

procedure TTestBuiltins.AssertAggregateExpression(const AExpression: String;
  AResult: Int64; AUpdateCount: integer);
begin
  FP.BuiltIns:=AllBuiltIns;
  SetExpression(AExpression);
  AssertEquals('Has aggregate',True,FP.ExprNode.HasAggregate);
  FP.InitAggregate;
  While AUpdateCount>0 do
    begin
    FP.UpdateAggregate;
    Dec(AUpdateCount);
    end;
  AssertResult(AResult);
end;

procedure TTestBuiltins.AssertAggregateExpression(const AExpression: String;
  AResult: TExprFloat; AUpdateCount: integer);
begin
  FP.BuiltIns:=AllBuiltIns;
  SetExpression(AExpression);
  AssertEquals('Has aggregate',True,FP.ExprNode.HasAggregate);
  FP.InitAggregate;
  While AUpdateCount>0 do
    begin
    FP.UpdateAggregate;
    Dec(AUpdateCount);
    end;
  AssertResult(AResult);
end;

procedure TTestBuiltins.AssertAggregateCurrExpression(Const AExpression : String; AResult : Currency; AUpdateCount : integer);

begin
  FP.BuiltIns:=AllBuiltIns;
  SetExpression(AExpression);
  AssertEquals('Has aggregate',True,FP.ExprNode.HasAggregate);
  FP.InitAggregate;
  While AUpdateCount>0 do
    begin
    FP.UpdateAggregate;
    Dec(AUpdateCount);
    end;
  AssertCurrencyResult(AResult);
end;

procedure TTestBuiltins.TestRegister;

begin
  RegisterStdBuiltins(FM);
  Assertvariable('pi',rtFloat);
  AssertFunction('cos','F','F',bcMath);
  AssertFunction('sin','F','F',bcMath);
  AssertFunction('arctan','F','F',bcMath);
  AssertFunction('abs','F','F',bcMath);
  AssertFunction('sqr','F','F',bcMath);
  AssertFunction('sqrt','F','F',bcMath);
  AssertFunction('exp','F','F',bcMath);
  AssertFunction('ln','F','F',bcMath);
  AssertFunction('log','F','F',bcMath);
  AssertFunction('frac','F','F',bcMath);
  AssertFunction('int','F','F',bcMath);
  AssertFunction('round','I','F',bcMath);
  AssertFunction('trunc','I','F',bcMath);
  AssertFunction('length','I','S',bcStrings);
  AssertFunction('copy','S','SII',bcStrings);
  AssertFunction('delete','S','SII',bcStrings);
  AssertFunction('pos','I','SS',bcStrings);
  AssertFunction('lowercase','S','S',bcStrings);
  AssertFunction('uppercase','S','S',bcStrings);
  AssertFunction('stringreplace','S','SSSBB',bcStrings);
  AssertFunction('comparetext','I','SS',bcStrings);
  AssertFunction('date','D','',bcDateTime);
  AssertFunction('time','D','',bcDateTime);
  AssertFunction('now','D','',bcDateTime);
  AssertFunction('dayofweek','I','D',bcDateTime);
  AssertFunction('extractyear','I','D',bcDateTime);
  AssertFunction('extractmonth','I','D',bcDateTime);
  AssertFunction('extractday','I','D',bcDateTime);
  AssertFunction('extracthour','I','D',bcDateTime);
  AssertFunction('extractmin','I','D',bcDateTime);
  AssertFunction('extractsec','I','D',bcDateTime);
  AssertFunction('extractmsec','I','D',bcDateTime);
  AssertFunction('encodedate','D','III',bcDateTime);
  AssertFunction('encodetime','D','IIII',bcDateTime);
  AssertFunction('encodedatetime','D','IIIIIII',bcDateTime);
  AssertFunction('shortdayname','S','I',bcDateTime);
  AssertFunction('shortmonthname','S','I',bcDateTime);
  AssertFunction('longdayname','S','I',bcDateTime);
  AssertFunction('longmonthname','S','I',bcDateTime);
  AssertFunction('shl','I','II',bcBoolean);
  AssertFunction('shr','I','II',bcBoolean);
  AssertFunction('IFS','S','BSS',bcBoolean);
  AssertFunction('IFF','F','BFF',bcBoolean);
  AssertFunction('IFD','D','BDD',bcBoolean);
  AssertFunction('IFI','I','BII',bcBoolean);
  AssertFunction('inttostr','S','I',bcConversion);
  AssertFunction('strtoint','I','S',bcConversion);
  AssertFunction('strtointdef','I','SI',bcConversion);
  AssertFunction('floattostr','S','F',bcConversion);
  AssertFunction('strtofloat','F','S',bcConversion);
  AssertFunction('strtofloatdef','F','SF',bcConversion);
  AssertFunction('booltostr','S','B',bcConversion);
  AssertFunction('strtobool','B','S',bcConversion);
  AssertFunction('strtobooldef','B','SB',bcConversion);
  AssertFunction('datetostr','S','D',bcConversion);
  AssertFunction('timetostr','S','D',bcConversion);
  AssertFunction('strtodate','D','S',bcConversion);
  AssertFunction('strtodatedef','D','SD',bcConversion);
  AssertFunction('strtotime','D','S',bcConversion);
  AssertFunction('strtotimedef','D','SD',bcConversion);
  AssertFunction('strtodatetime','D','S',bcConversion);
  AssertFunction('strtodatetimedef','D','SD',bcConversion);
  AssertFunction('formatfloat','S','SF',bcConversion);
  AssertFunction('formatdatetime','S','SD',bcConversion);
  AssertFunction('sum','F','F',bcAggregate);
  AssertFunction('count','I','',bcAggregate);
  AssertFunction('avg','F','F',bcAggregate);
  AssertFunction('min','F','F',bcAggregate);
  AssertFunction('max','F','F',bcAggregate);
  AssertEquals('Correct number of identifiers',70,FM.IdentifierCount);
end;

procedure TTestBuiltins.TestVariablepi;
begin
  AssertExpression('pi',Pi);
end;

procedure TTestBuiltins.TestFunctioncos;
begin
  AssertExpression('cos(0.5)',Cos(0.5));
  AssertExpression('cos(0.75)',Cos(0.75));
end;

procedure TTestBuiltins.TestFunctionsin;
begin
  AssertExpression('sin(0.5)',sin(0.5));
  AssertExpression('sin(0.75)',sin(0.75));
end;

procedure TTestBuiltins.TestFunctionarctan;
begin
  AssertExpression('arctan(0.5)',arctan(0.5));
  AssertExpression('arctan(0.75)',arctan(0.75));
end;

procedure TTestBuiltins.TestFunctionabs;
begin
  AssertExpression('abs(0.5)',0.5);
  AssertExpression('abs(-0.75)',0.75);
end;

procedure TTestBuiltins.TestFunctionsqr;
begin
  AssertExpression('sqr(0.5)',sqr(0.5));
  AssertExpression('sqr(-0.75)',sqr(0.75));
end;

procedure TTestBuiltins.TestFunctionsqrt;
begin
  AssertExpression('sqrt(0.5)',sqrt(0.5));
  AssertExpression('sqrt(0.75)',sqrt(0.75));
end;

procedure TTestBuiltins.TestFunctionexp;
begin
  AssertExpression('exp(1.0)',exp(1));
  AssertExpression('exp(0.0)',1.0);
end;

procedure TTestBuiltins.TestFunctionln;
begin
  AssertExpression('ln(0.5)',ln(0.5));
  AssertExpression('ln(1.5)',ln(1.5));
end;

procedure TTestBuiltins.TestFunctionlog;
begin
  AssertExpression('log(0.5)',ln(0.5)/ln(10.0));
  AssertExpression('log(1.5)',ln(1.5)/ln(10.0));
  AssertExpression('log(10.0)',1.0);
end;

procedure TTestBuiltins.TestFunctionfrac;
begin
  AssertExpression('frac(0.5)',frac(0.5));
  AssertExpression('frac(1.5)',frac(1.5));
end;

procedure TTestBuiltins.TestFunctionint;
begin
  AssertExpression('int(0.5)',int(0.5));
  AssertExpression('int(1.5)',int(1.5));
end;

procedure TTestBuiltins.TestFunctionround;
begin
  AssertExpression('round(0.5)',round(0.5));
  AssertExpression('round(1.55)',round(1.55));
end;

procedure TTestBuiltins.TestFunctiontrunc;
begin
  AssertExpression('trunc(0.5)',trunc(0.5));
  AssertExpression('trunc(1.55)',trunc(1.55));
end;

procedure TTestBuiltins.TestFunctionlength;
begin
  AssertExpression('length(''123'')',3);
end;

procedure TTestBuiltins.TestFunctioncopy;
begin
  AssertExpression('copy(''123456'',2,4)','2345');
end;

procedure TTestBuiltins.TestFunctiondelete;
begin
  AssertExpression('delete(''123456'',2,4)','16');
end;

procedure TTestBuiltins.TestFunctionpos;
begin
  AssertExpression('pos(''234'',''123456'')',2);
end;

procedure TTestBuiltins.TestFunctionlowercase;
begin
  AssertExpression('lowercase(''AbCdEf'')','abcdef');
end;

procedure TTestBuiltins.TestFunctionuppercase;
begin
  AssertExpression('uppercase(''AbCdEf'')','ABCDEF');
end;

procedure TTestBuiltins.TestFunctionstringreplace;
begin
  // last options are replaceall, ignorecase
  AssertExpression('stringreplace(''AbCdEf'',''C'',''Z'',false,false)','AbZdEf');
  AssertExpression('stringreplace(''AbCdEf'',''c'',''Z'',false,false)','AbCdEf');
  AssertExpression('stringreplace(''AbCdEf'',''c'',''Z'',false,true)','AbZdEf');
  AssertExpression('stringreplace(''AbCdEfC'',''C'',''Z'',false,false)','AbZdEfC');
  AssertExpression('stringreplace(''AbCdEfC'',''C'',''Z'',True,false)','AbZdEfZ');
end;

procedure TTestBuiltins.TestFunctioncomparetext;
begin
  AssertExpression('comparetext(''AbCdEf'',''AbCdEf'')',0);
  AssertExpression('comparetext(''AbCdEf'',''ABCDEF'')',0);
  AssertExpression('comparetext(''AbCdEf'',''FEDCBA'')',comparetext('AbCdEf','FEDCBA'));
end;

procedure TTestBuiltins.TestFunctiondate;
begin
  AssertExpression('date',date);
end;

procedure TTestBuiltins.TestFunctiontime;
begin
  AssertExpression('time',time);
end;

procedure TTestBuiltins.TestFunctionnow;
begin
  AssertExpression('now',now);
end;

procedure TTestBuiltins.TestFunctiondayofweek;
begin
  FP.Identifiers.AddDateTimeVariable('D',Date);
  AssertExpression('dayofweek(d)',DayOfWeek(date));
end;

procedure TTestBuiltins.TestFunctionextractyear;

Var
  Y,M,D : Word;

begin
  DecodeDate(Date,Y,M,D);
  FP.Identifiers.AddDateTimeVariable('D',Date);
  AssertExpression('extractyear(d)',Y);
end;

procedure TTestBuiltins.TestFunctionextractmonth;

Var
  Y,M,D : Word;

begin
  FP.Identifiers.AddDateTimeVariable('D',Date);
  DecodeDate(Date,Y,M,D);
  AssertExpression('extractmonth(d)',M);
end;

procedure TTestBuiltins.TestFunctionextractday;

Var
  Y,M,D : Word;

begin
  DecodeDate(Date,Y,M,D);
  FP.Identifiers.AddDateTimeVariable('D',Date);
  AssertExpression('extractday(d)',D);
end;

procedure TTestBuiltins.TestFunctionextracthour;

Var
  T : TDateTime;
  H,m,s,ms : Word;

begin
  T:=Time;
  DecodeTime(T,h,m,s,ms);
  FP.Identifiers.AddDateTimeVariable('T',T);
  AssertExpression('extracthour(t)',h);
end;

procedure TTestBuiltins.TestFunctionextractmin;
Var
  T : TDateTime;
  H,m,s,ms : Word;

begin
  T:=Time;
  DecodeTime(T,h,m,s,ms);
  FP.Identifiers.AddDateTimeVariable('T',T);
  AssertExpression('extractmin(t)',m);
end;

procedure TTestBuiltins.TestFunctionextractsec;
Var
  T : TDateTime;
  H,m,s,ms : Word;

begin
  T:=Time;
  DecodeTime(T,h,m,s,ms);
  FP.Identifiers.AddDateTimeVariable('T',T);
  AssertExpression('extractsec(t)',s);
end;

procedure TTestBuiltins.TestFunctionextractmsec;
Var
  T : TDateTime;
  H,m,s,ms : Word;

begin
  T:=Time;
  DecodeTime(T,h,m,s,ms);
  FP.Identifiers.AddDateTimeVariable('T',T);
  AssertExpression('extractmsec(t)',ms);
end;

procedure TTestBuiltins.TestFunctionencodedate;
begin
  AssertExpression('encodedate(2008,10,11)',EncodeDate(2008,10,11));
end;

procedure TTestBuiltins.TestFunctionencodetime;
begin
  AssertExpression('encodetime(14,10,11,0)',EncodeTime(14,10,11,0));
end;

procedure TTestBuiltins.TestFunctionencodedatetime;
begin
  AssertExpression('encodedatetime(2008,12,13,14,10,11,0)',EncodeDate(2008,12,13)+EncodeTime(14,10,11,0));
end;

procedure TTestBuiltins.TestFunctionshortdayname;
begin
  AssertExpression('shortdayname(1)',ShortDayNames[1]);
  AssertExpression('shortdayname(7)',ShortDayNames[7]);
end;

procedure TTestBuiltins.TestFunctionshortmonthname;
begin
  AssertExpression('shortmonthname(1)',ShortMonthNames[1]);
  AssertExpression('shortmonthname(12)',ShortMonthNames[12]);
end;

procedure TTestBuiltins.TestFunctionlongdayname;
begin
  AssertExpression('longdayname(1)',longDayNames[1]);
  AssertExpression('longdayname(7)',longDayNames[7]);
end;

procedure TTestBuiltins.TestFunctionlongmonthname;
begin
  AssertExpression('longmonthname(1)',longMonthNames[1]);
  AssertExpression('longmonthname(12)',longMonthNames[12]);
end;

procedure TTestBuiltins.TestFunctionformatdatetime;
begin
  AssertExpression('FormatDateTime(''cccc'',Date)',FormatDateTime('cccc',Date));
end;

procedure TTestBuiltins.TestFunctionshl;

Var
  I : Int64;

begin
  AssertExpression('shl(12,3)',12 shl 3);
  I:=12 shl 30;
  AssertExpression('shl(12,30)',I);
end;

procedure TTestBuiltins.TestFunctionshr;
begin
  AssertExpression('shr(12,2)',12 shr 2);
end;

procedure TTestBuiltins.TestFunctionIFS;
begin
  AssertExpression('ifs(true,''string1'',''string2'')','string1');
  AssertExpression('ifs(false,''string1'',''string2'')','string2');
end;

procedure TTestBuiltins.TestFunctionIFF;
begin
  AssertExpression('iff(true,1.0,2.0)',1.0);
  AssertExpression('iff(false,1.0,2.0)',2.0);
end;

procedure TTestBuiltins.TestFunctionIFD;
begin
  FP.Identifiers.AddDateTimeVariable('A',Date);
  FP.Identifiers.AddDateTimeVariable('B',Date-1);
  AssertExpression('ifd(true,A,B)',Date);
  AssertExpression('ifd(false,A,B)',Date-1);
end;

procedure TTestBuiltins.TestFunctionIFI;
begin
  AssertExpression('ifi(true,1,2)',1);
  AssertExpression('ifi(false,1,2)',2);
end;

procedure TTestBuiltins.TestFunctioninttostr;
begin
  AssertExpression('inttostr(2)','2');
end;

procedure TTestBuiltins.TestFunctionstrtoint;
begin
  AssertExpression('strtoint(''2'')',2);
end;

procedure TTestBuiltins.TestFunctionstrtointdef;
begin
  AssertExpression('strtointdef(''abc'',2)',2);
end;

procedure TTestBuiltins.TestFunctionfloattostr;
begin
  AssertExpression('floattostr(1.23)',Floattostr(1.23));
end;

procedure TTestBuiltins.TestFunctionstrtofloat;

Var
  S : String;

begin
  S:='1.23';
  S[2]:=DecimalSeparator;
  AssertExpression('strtofloat('''+S+''')',1.23);
end;

procedure TTestBuiltins.TestFunctionstrtofloatdef;

begin
  AssertExpression('strtofloatdef(''abc'',1.23)',1.23);
end;

procedure TTestBuiltins.TestFunctionbooltostr;
begin
  AssertExpression('strtofloatdef(''abc'',1.23)',1.23);
end;

procedure TTestBuiltins.TestFunctionstrtobool;
begin
  AssertExpression('strtobool(''0'')',false);
end;

procedure TTestBuiltins.TestFunctionstrtobooldef;
begin
  AssertExpression('strtobooldef(''XYZ'',True)',True);
end;

procedure TTestBuiltins.TestFunctiondatetostr;
begin
  FP.Identifiers.AddDateTimeVariable('A',Date);
  AssertExpression('DateToStr(A)',DateToStr(Date));
end;

procedure TTestBuiltins.TestFunctiontimetostr;

Var
  T : TDateTime;

begin
  T:=Time;
  FP.Identifiers.AddDateTimeVariable('A',T);
  AssertExpression('TimeToStr(A)',TimeToStr(T));
end;

procedure TTestBuiltins.TestFunctionstrtodate;

begin
  FP.Identifiers.AddStringVariable('S',DateToStr(Date));
  AssertExpression('StrToDate(S)',Date);
end;

procedure TTestBuiltins.TestFunctionstrtodatedef;
begin
  FP.Identifiers.AddDateTimeVariable('A',Date);
  AssertExpression('StrToDateDef(''S'',A)',Date);
end;

procedure TTestBuiltins.TestFunctionstrtotime;

Var
  T : TDateTime;

begin
  T:=Time;
  FP.Identifiers.AddStringVariable('S',TimeToStr(T));
  AssertExpression('StrToTime(S)',T);
end;

procedure TTestBuiltins.TestFunctionstrtotimedef;
Var
  T : TDateTime;

begin
  T:=Time;
  FP.Identifiers.AddDateTimeVariable('S',T);
  AssertExpression('StrToTimeDef(''q'',S)',T);
end;

procedure TTestBuiltins.TestFunctionstrtodatetime;

Var
  T : TDateTime;
  S : String;

begin
  T:=Now;
  S:=DateTimetostr(T);
  AssertExpression('StrToDateTime('''+S+''')',T);
end;

procedure TTestBuiltins.TestFunctionstrtodatetimedef;

Var
  T : TDateTime;
  S : String;

begin
  T:=Now;
  S:=DateTimetostr(T);
  FP.Identifiers.AddDateTimeVariable('S',T);
  AssertExpression('StrToDateTimeDef('''+S+''',S)',T);
end;

procedure TTestBuiltins.TestFunctionAggregateSum;
begin
  FP.Identifiers.AddIntegerVariable('S',2);
  AssertAggregateExpression('sum(S)',10,5);
end;

procedure TTestBuiltins.TestFunctionAggregateSumFloat;
begin
  FP.Identifiers.AddFloatVariable('S',2.0);
  AssertAggregateExpression('sum(S)',10.0,5);
end;

procedure TTestBuiltins.TestFunctionAggregateSumCurrency;
begin
  FP.Identifiers.AddCurrencyVariable('S',2.0);
  AssertAggregateCurrExpression('sum(S)',Currency(10.0),5);
end;

procedure TTestBuiltins.TestFunctionAggregateCount;
begin
  AssertAggregateExpression('count',5,5);
end;


procedure TTestBuiltins.DoAverage(var Result: TFPExpressionResult; ConstRef
  AName: ShortString);

begin
  Inc(FValue);
  Result.ResInteger:=FValue;
  Result.ResultType:=rtInteger;
end;

procedure TTestBuiltins.DoSeries(var Result: TFPExpressionResult; ConstRef
  AName: ShortString);

Const
  Values : Array[1..10] of double =
  (1.3,1.8,1.1,9.9,1.4,2.4,5.8,6.5,7.8,8.1);


begin
  Inc(FValue);
  Result.ResFloat:=Values[FValue];
  Result.ResultType:=rtFloat;
end;

procedure TTestBuiltins.TestFunctionAggregateAvg;
begin
  FP.Identifiers.AddVariable('S',rtInteger,@DoAverage);
  AssertAggregateExpression('avg(S)',5.5,10);
end;

procedure TTestBuiltins.TestFunctionAggregateMin;
begin
  FP.Identifiers.AddVariable('S',rtFloat,@DoSeries);
  AssertAggregateExpression('Min(S)',1.1,10);
end;

procedure TTestBuiltins.TestFunctionAggregateMax;
begin
  FP.Identifiers.AddVariable('S',rtFloat,@DoSeries);
  AssertAggregateExpression('Max(S)',9.9,10);
end;

{ TTestNotNode }

procedure TTestNotNode.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestNotNode.TestCreateInteger;
begin
  FN:=TFPNotNode.Create(CreateIntNode(3));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',Not(Int64(3)),FN.NodeValue.ResInteger);
end;

procedure TTestNotNode.TestCreateBoolean;
begin
  FN:=TFPNotNode.Create(CreateBoolNode(True));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtBoolean,FN.NodeType);
  AssertEquals('Correct result',False,FN.NodeValue.ResBoolean);
end;

procedure TTestNotNode.TestCreateString;
begin
  FN:=TFPNotNode.Create(CreateStringNode('True'));
  AssertNodeNotOK('String node type',FN);
end;

procedure TTestNotNode.TestCreateFloat;
begin
  FN:=TFPNotNode.Create(CreateFloatNode(1.23));
  AssertNodeNotOK('String node type',FN);
end;

procedure TTestNotNode.TestCreateDateTime;
begin
  FN:=TFPNotNode.Create(CreateDateTimeNode(Now));
  AssertNodeNotOK('String node type',FN);
end;

procedure TTestNotNode.TestDestroy;
begin
  FN:=TFPNotNode.Create(TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for operand',1,self.FDestroyCalled)
end;

{ TTestIfOperation }

procedure TTestIfOperation.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestIfOperation.TestCreateInteger;
begin
  FN:=TIfOperation.Create(CreateIntNode(1),CreateIntNode(2),CreateIntNode(3));
  AssertNodeNotOK('First argument wrong',FN);
end;

procedure TTestIfOperation.TestCreateBoolean;
begin
  FN:=TIfOperation.Create(CreateBoolNode(True),CreateIntNode(2),CreateIntNode(3));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',2,FN.NodeValue.ResInteger);
end;

procedure TTestIfOperation.TestCreateBoolean2;
begin
  FN:=TIfOperation.Create(CreateBoolNode(False),CreateIntNode(2),CreateIntNode(3));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',3,FN.NodeValue.ResInteger);
end;

procedure TTestIfOperation.TestCreateBooleanInteger;
begin
  FN:=TIfOperation.Create(CreateBoolNode(False),CreateIntNode(2),CreateBoolNode(False));
  AssertNodeNotOK('Arguments differ in type',FN);
end;

procedure TTestIfOperation.TestCreateBooleanInteger2;
begin
  FN:=TIfOperation.Create(CreateBoolNode(True),CreateIntNode(2),CreateIntNode(3));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',2,FN.NodeValue.ResInteger);
end;

procedure TTestIfOperation.TestCreateBooleanString;
begin
  FN:=TIfOperation.Create(CreateBoolNode(True),CreateStringNode('2'),CreateStringNode('3'));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','2',FN.NodeValue.ResString);
end;

procedure TTestIfOperation.TestCreateBooleanString2;
begin
  FN:=TIfOperation.Create(CreateBoolNode(False),CreateStringNode('2'),CreateStringNode('3'));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','3',FN.NodeValue.ResString);
end;

procedure TTestIfOperation.TestCreateBooleanDateTime;
begin
  FN:=TIfOperation.Create(CreateBoolNode(True),CreateDateTimeNode(Date),CreateDateTimeNode(Date-1));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtDateTime,FN.NodeType);
  AssertEquals('Correct result',Date,FN.NodeValue.ResDateTime);
end;

procedure TTestIfOperation.TestCreateBooleanDateTime2;
begin
  FN:=TIfOperation.Create(CreateBoolNode(False),CreateDateTimeNode(Date),CreateDateTimeNode(Date-1));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtDateTime,FN.NodeType);
  AssertEquals('Correct result',Date-1,FN.NodeValue.ResDateTime);
end;

procedure TTestIfOperation.TestCreateString;
begin
  FN:=TIfOperation.Create(CreateStringNode('1'),CreateIntNode(2),CreateIntNode(3));
  AssertNodeNotOK('First argument wrong',FN);
end;

procedure TTestIfOperation.TestCreateFloat;
begin
  FN:=TIfOperation.Create(CreateFloatNode(2.0),CreateIntNode(2),CreateIntNode(3));
  AssertNodeNotOK('First argument wrong',FN);
end;

procedure TTestIfOperation.TestCreateDateTime;
begin
  FN:=TIfOperation.Create(CreateDateTimeNode(Date),CreateIntNode(2),CreateIntNode(3));
  AssertNodeNotOK('First argument wrong',FN);
end;

procedure TTestIfOperation.TestDestroy;
begin
  FN:=TIfOperation.Create(TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self),TMyDestroyNode.CreateTest(Self));
  FreeAndNil(FN);
  AssertEquals('Destroy called for operand',3,self.FDestroyCalled)
end;

{ TTestCaseOperation }

function TTestCaseOperation.CreateArgs(
  Args: array of const): TExprArgumentArray;

Var
  I : Integer;

begin
  Result:=Default(TExprArgumentArray);
  SetLength(Result,High(Args)-Low(Args)+1);
  For I:=Low(Args) to High(Args) do
    Result[I]:=Args[i].VObject as TFPExprNode;
end;

procedure TTestCaseOperation.TearDown;
begin
  FreeAndNil(FN);
  inherited TearDown;
end;

procedure TTestCaseOperation.TestCreateOne;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateBoolNode(False)]));
  AssertNodeNotOK('Too little arguments',FN);
end;

procedure TTestCaseOperation.TestCreateTwo;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateBoolNode(False),CreateBoolNode(False)]));
  AssertNodeNotOK('Too little arguments',FN);
end;

procedure TTestCaseOperation.TestCreateThree;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateBoolNode(False),CreateBoolNode(False),CreateBoolNode(False)]));
  AssertNodeNotOK('Too little arguments',FN);
end;

procedure TTestCaseOperation.TestCreateOdd;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateBoolNode(False),CreateBoolNode(False),
                                        CreateBoolNode(False),CreateBoolNode(False),
                                        CreateBoolNode(False)]));
  AssertNodeNotOK('Odd number of arguments',FN);
end;

procedure TTestCaseOperation.TestCreateNoExpression;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateBoolNode(False),
                                        CreateBoolNode(False),
                                        TFPBinaryOrOperation.Create(CreateBoolNode(False),CreateBoolNode(False)),
                                        CreateBoolNode(False)]));
  AssertNodeNotOK('Label is not a constant expression',FN);
end;

procedure TTestCaseOperation.TestCreateWrongLabel;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(1),CreateBoolNode(False),
                                        CreateIntNode(1),CreateBoolNode(False),
                                        CreateBoolNode(True),CreateBoolNode(False)]));
  AssertNodeNotOK('Wrong label',FN);
end;

procedure TTestCaseOperation.TestCreateWrongValue;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(1),CreateBoolNode(False),
                                        CreateIntNode(1),CreateBoolNode(False),
                                        CreateIntNode(2),CreateIntNode(1)]));
  AssertNodeNotOK('Wrong value',FN);
end;

procedure TTestCaseOperation.TestIntegerTag;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(1),CreateStringNode('many'),
                                        CreateIntNode(1),CreateStringNode('one'),
                                        CreateIntNode(2),CreateStringNode('two')]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','one',FN.NodeValue.ResString);
end;

procedure TTestCaseOperation.TestIntegerTagDefault;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(3),CreateStringNode('many'),
                                        CreateIntNode(1),CreateStringNode('one'),
                                        CreateIntNode(2),CreateStringNode('two')]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','many',FN.NodeValue.ResString);
end;

procedure TTestCaseOperation.TestStringTag;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateStringNode('one'),CreateIntNode(3),
                                        CreateStringNode('one'),CreateIntNode(1),
                                        CreateStringNode('two'),CreateIntNode(2)]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',1,FN.NodeValue.ResInteger);
end;

procedure TTestCaseOperation.TestStringTagDefault;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateStringNode('many'),CreateIntNode(3),
                                        CreateStringNode('one'),CreateIntNode(1),
                                        CreateStringNode('two'),CreateIntNode(2)]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',3,FN.NodeValue.ResInteger);
end;

procedure TTestCaseOperation.TestFloatTag;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateFloatNode(1.0),CreateStringNode('many'),
                                        CreateFloatNode(1.0),CreateStringNode('one'),
                                        CreateFloatNode(2.0),CreateStringNode('two')]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','one',FN.NodeValue.ResString);
end;

procedure TTestCaseOperation.TestFloatTagDefault;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateFloatNode(3.0),CreateStringNode('many'),
                                        CreateFloatNode(1.0),CreateStringNode('one'),
                                        CreateFloatNode(2.0),CreateStringNode('two')]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','many',FN.NodeValue.ResString);
end;

procedure TTestCaseOperation.TestBooleanTag;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateBoolNode(True),CreateStringNode('unknown'),
                                        CreateBoolNode(True),CreateStringNode('one'),
                                        CreateBoolNode(False),CreateStringNode('two')]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','one',FN.NodeValue.ResString);
end;

procedure TTestCaseOperation.TestBooleanTagDefault;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateBoolNode(True),CreateStringNode('unknown'),
                                        CreateBoolNode(False),CreateStringNode('two')]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','unknown',FN.NodeValue.ResString);
end;

procedure TTestCaseOperation.TestDateTimeTag;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateDateTimeNode(Date),CreateStringNode('later'),
                                        CreateDateTimeNode(Date),CreateStringNode('today'),
                                        CreateDateTimeNode(Date+1),CreateStringNode('tomorrow')]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','today',FN.NodeValue.ResString);
end;

procedure TTestCaseOperation.TestDateTimeTagDefault;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateDateTimeNode(Date+2),CreateStringNode('later'),
                                        CreateDateTimeNode(Date),CreateStringNode('today'),
                                        CreateDateTimeNode(Date+1),CreateStringNode('tomorrow')]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','later',FN.NodeValue.ResString);
end;

procedure TTestCaseOperation.TestIntegerValue;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(1),CreateIntNode(0),
                                        CreateIntNode(1),CreateIntNode(-1),
                                        CreateIntNode(2),CreateIntNode(-2)]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',-1,FN.NodeValue.ResInteger);
end;

procedure TTestCaseOperation.TestIntegerValueDefault;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(3),CreateIntNode(0),
                                        CreateIntNode(1),CreateIntNode(-1),
                                        CreateIntNode(2),CreateIntNode(-2)]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtInteger,FN.NodeType);
  AssertEquals('Correct result',0,FN.NodeValue.ResInteger);
end;

procedure TTestCaseOperation.TestStringValue;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(1),CreateStringNode('many'),
                                        CreateIntNode(1),CreateStringNode('one'),
                                        CreateIntNode(2),CreateStringNode('two')]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','one',FN.NodeValue.ResString);
end;

procedure TTestCaseOperation.TestStringValueDefault;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(3),CreateStringNode('many'),
                                        CreateIntNode(1),CreateStringNode('one'),
                                        CreateIntNode(2),CreateStringNode('two')]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtString,FN.NodeType);
  AssertEquals('Correct result','many',FN.NodeValue.ResString);
end;

procedure TTestCaseOperation.TestFloatValue;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(1),CreateFloatNode(0.0),
                                        CreateIntNode(1),CreateFloatNode(2.0),
                                        CreateIntNode(2),CreateFloatNode(1.0)]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtFloat,FN.NodeType);
  AssertEquals('Correct result',2.0,FN.NodeValue.ResFloat);
end;

procedure TTestCaseOperation.TestFloatValueDefault;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(3),CreateFloatNode(0.0),
                                        CreateIntNode(1),CreateFloatNode(2.0),
                                        CreateIntNode(2),CreateFloatNode(1.0)]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtFloat,FN.NodeType);
  AssertEquals('Correct result',0.0,FN.NodeValue.ResFloat);
end;

procedure TTestCaseOperation.TestBooleanValue;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(1),CreateBoolNode(False),
                                        CreateIntNode(1),CreateBoolNode(True),
                                        CreateIntNode(2),CreateBoolNode(False)]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtBoolean,FN.NodeType);
  AssertEquals('Correct result',True,FN.NodeValue.ResBoolean);
end;

procedure TTestCaseOperation.TestBooleanValueDefault;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(3),CreateBoolNode(False),
                                        CreateIntNode(1),CreateBoolNode(True),
                                        CreateIntNode(2),CreateBoolNode(False)]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtBoolean,FN.NodeType);
  AssertEquals('Correct result',False,FN.NodeValue.ResBoolean);
end;

procedure TTestCaseOperation.TestDateTimeValue;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(1),CreateDateTimeNode(Date+1),
                                        CreateIntNode(1),CreateDateTimeNode(Date),
                                        CreateIntNode(2),CreateDateTimeNode(Date-1)]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtDateTime,FN.NodeType);
  AssertEquals('Correct result',Date,FN.NodeValue.ResDateTime);
end;

procedure TTestCaseOperation.TestDateTimeValueDefault;
begin
  FN:=TCaseOperation.Create(CreateArgs([CreateIntNode(3),CreateDateTimeNode(Date+1),
                                        CreateIntNode(1),CreateDateTimeNode(Date),
                                        CreateIntNode(2),CreateDateTimeNode(Date-1)]));
  AssertNodeOK(FN);
  AssertEquals('Correct node type',rtDateTime,FN.NodeType);
  AssertEquals('Correct result',Date+1,FN.NodeValue.ResDateTime);
end;

procedure TTestCaseOperation.TestDestroy;
begin
  FN:=TCaseOperation.Create(CreateArgs([TMyDestroyNode.CreateTest(Self),
                                        TMyDestroyNode.CreateTest(Self),
                                        TMyDestroyNode.CreateTest(Self),
                                        TMyDestroyNode.CreateTest(Self)]));
  FreeAndNil(FN);
  AssertEquals('Destroy called for operand',4,self.FDestroyCalled)
end;

initialization

  RegisterTests('ExprPars',[TTestExpressionScanner, TTestDestroyNode,
                 TTestConstExprNode,TTestNegateExprNode,
                 TTestBinaryAndNode,TTestBinaryOrNode,TTestBinaryXOrNode,
                 TTestNotNode,TTestEqualNode,TTestUnEqualNode,
                 TTestIfOperation,TTestCaseOperation,
                 TTestLessThanNode,TTestLessThanEqualNode,
                 TTestLargerThanNode,TTestLargerThanEqualNode,
                 TTestAddNode,TTestSubtractNode,
                 TTestMultiplyNode,TTestDivideNode,TTestPowerNode,
                 TTestIntToFloatNode,TTestIntToDateTimeNode,
                 TTestFloatToDateTimeNode,
                 TTestParserExpressions, TTestParserBooleanOperations,
                 TTestParserOperands, TTestParserTypeMatch,
                 TTestParserVariables,TTestParserFunctions,
                 TTestParserAggregate,
                 TTestBuiltinsManager,TTestBuiltins]);
end.

