{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2008 Michael Van Canneyt.

    Expression parser, supports variables, functions and
    float/integer/string/boolean/datetime operations.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
unit fpexprpars;

interface

uses
  Classes, SysUtils, contnrs;

Type
  // tokens
  TTokenType = (ttPlus, ttMinus, ttLessThan, ttLargerThan, ttEqual, ttDiv,
                ttMod, ttMul, ttLeft, ttRight, ttLessThanEqual,
                ttLargerThanEqual, ttunequal, ttNumber, ttString, ttIdentifier,
                ttComma, ttAnd, ttOr, ttXor, ttTrue, ttFalse, ttNot, ttif,
                ttCase, ttPower, ttEOF); // keep ttEOF last

  TExprFloat = Double;

Const
  ttDelimiters = [ttPlus, ttMinus, ttLessThan, ttLargerThan, ttEqual, ttDiv,
                  ttMul, ttLeft, ttRight, ttLessThanEqual, ttLargerThanEqual,
                  ttunequal, ttPower];
  ttComparisons = [ttLargerThan,ttLessthan,
                   ttLargerThanEqual,ttLessthanEqual,
                   ttEqual,ttUnequal];

Type

  TFPExpressionParser = Class;
  TExprBuiltInManager = Class;
  TFPExprFunction = Class;
  TFPExprFunctionClass = Class of TFPExprFunction;

  TNumberKind = (nkDecimal, nkHex, nkOctal, nkBinary);

  { TFPExpressionScanner }

  TFPExpressionScanner = Class(TObject)
    FSource : String;
    LSource,
    FPos : Integer;
    FChar : PChar;
    FToken : String;
    FTokenType : TTokenType;
  private
    function GetCurrentChar: Char;
    procedure ScanError(Msg: String);
  protected
    procedure SetSource(const AValue: String); virtual;
    function DoIdentifier: TTokenType;
    function DoNumber(AKind: TNumberKind): TTokenType;
    function DoDelimiter: TTokenType;
    function DoString: TTokenType;
    Function NextPos : Char; // inline;
    procedure SkipWhiteSpace; // inline;
    function IsWordDelim(C : Char) : Boolean; // inline;
    function IsDelim(C : Char) : Boolean; // inline;
    function IsDigit(C : Char; AKind: TNumberKind) : Boolean; // inline;
    function IsAlpha(C : Char) : Boolean; // inline;
  public
    Constructor Create;
    Function GetToken : TTokenType;
    Property Token : String Read FToken;
    Property TokenType : TTokenType Read FTokenType;
    Property Source : String Read FSource Write SetSource;
    Property Pos : Integer Read FPos;
    Property CurrentChar : Char Read GetCurrentChar;
  end;

  EExprScanner = Class(Exception);

  TResultType = (rtBoolean,rtInteger,rtFloat,rtDateTime,rtString,rtCurrency);
  TResultTypes = set of TResultType;

  TFPExpressionResult = record
    ResString   : String;
    Case ResultType : TResultType of
      rtBoolean  : (ResBoolean  : Boolean);
      rtInteger  : (ResInteger  : Int64);
      rtFloat    : (ResFloat    : TExprFloat);
      rtCurrency : (ResCurrency : Currency);
      rtDateTime : (ResDateTime : TDatetime);
      rtString   : ();
  end;
  PFPExpressionResult = ^TFPExpressionResult;
  TExprParameterArray = Array of TFPExpressionResult;

  { TFPExprNode }

  TFPExprNode = Class(TObject)
  Protected
    Procedure CheckNodeType(Anode : TFPExprNode; Allowed : TResultTypes);
    // A procedure with var saves an implicit try/finally in each node
    // A marked difference in execution speed.
    Procedure GetNodeValue(var Result : TFPExpressionResult); virtual; abstract;
  Public
    Procedure Check; virtual; abstract;
    Procedure InitAggregate; virtual;
    Procedure UpdateAggregate; virtual;
    Class Function IsAggregate : Boolean; virtual;
    Function HasAggregate : Boolean; virtual;
    Function NodeType : TResultType; virtual; abstract;
    Function NodeValue : TFPExpressionResult;
    Function AsString : string; virtual; abstract;
  end;
  TExprArgumentArray = Array of TFPExprNode;

  { TFPBinaryOperation }

  TFPBinaryOperation = Class(TFPExprNode)
  private
    FLeft: TFPExprNode;
    FRight: TFPExprNode;
  Protected
    Procedure CheckSameNodeTypes;
  Public
    Constructor Create(ALeft,ARight : TFPExprNode);
    Destructor Destroy; override;
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
    Function HasAggregate : Boolean; override;
    Procedure Check; override;
    Property left : TFPExprNode Read FLeft;
    Property Right : TFPExprNode Read FRight;
  end;
  TFPBinaryOperationClass = Class of TFPBinaryOperation;


  { TFPBooleanOperation }

  TFPBooleanOperation = Class(TFPBinaryOperation)
  Public
    Procedure Check; override;
    Function NodeType : TResultType; override;
  end;
  { TFPBinaryAndOperation }

  TFPBinaryAndOperation = Class(TFPBooleanOperation)
  Protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Function AsString : string ; override;
  end;

  { TFPBinaryOrOperation }

  TFPBinaryOrOperation = Class(TFPBooleanOperation)
  Protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Function AsString : string ; override;
  end;

  { TFPBinaryXOrOperation }

  TFPBinaryXOrOperation = Class(TFPBooleanOperation)
  Protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Function AsString : string ; override;
  end;

  { TFPBooleanResultOperation }

  TFPBooleanResultOperation = Class(TFPBinaryOperation)
  Public
    Procedure Check; override;
    Function NodeType : TResultType; override;
  end;
  TFPBooleanResultOperationClass = Class of TFPBooleanResultOperation;


  { TFPEqualOperation }

  TFPEqualOperation = Class(TFPBooleanResultOperation)
  Protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Function AsString : string ; override;
  end;

  { TFPUnequalOperation }

  TFPUnequalOperation = Class(TFPEqualOperation)
  Protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Function AsString : string ; override;
  end;

  { TFPOrderingOperation }

  TFPOrderingOperation = Class(TFPBooleanResultOperation)
  Public
    Procedure Check; override;
  end;

  { TFPLessThanOperation }

  TFPLessThanOperation = Class(TFPOrderingOperation)
  Protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Function AsString : string ; override;
  end;

  { TFPGreaterThanOperation }

  TFPGreaterThanOperation = Class(TFPOrderingOperation)
  Protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Function AsString : string ; override;
  end;

  { TFPLessThanEqualOperation }

  TFPLessThanEqualOperation = Class(TFPGreaterThanOperation)
  Protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Function AsString : string ; override;
  end;


  { TFPGreaterThanEqualOperation }

  TFPGreaterThanEqualOperation = Class(TFPLessThanOperation)
  Protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Function AsString : string ; override;
  end;

  { TIfOperation }

  TIfOperation = Class(TFPBinaryOperation)
  private
    FCondition: TFPExprNode;
  protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Procedure Check; override;
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
    Function HasAggregate : Boolean; override;
    Function NodeType : TResultType; override;
    Constructor Create(ACondition,ALeft,ARight : TFPExprNode);
    Destructor destroy; override;
    Function AsString : string ; override;
    Property Condition : TFPExprNode Read FCondition;
  end;

  { TCaseOperation }

  TCaseOperation = Class(TFPExprNode)
  private
    FArgs : TExprArgumentArray;
    FCondition: TFPExprNode;
  protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Procedure Check; override;
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
    function HasAggregate: Boolean; override;
    Function NodeType : TResultType; override;
    Constructor Create(Args : TExprArgumentArray);
    Destructor destroy; override;
    Function AsString : string ; override;
    Property Condition : TFPExprNode Read FCondition;
  end;

  { TMathOperation }

  TMathOperation = Class(TFPBinaryOperation)
  Public
    Procedure Check; override;
    Function NodeType : TResultType; override;
  end;

  { TFPAddOperation }

  TFPAddOperation = Class(TMathOperation)
  Protected
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
    Function AsString : string ; override;
  end;

  { TFPSubtractOperation }

  TFPSubtractOperation = Class(TMathOperation)
  Public
    Procedure Check; override;
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
    Function AsString : string ; override;
  end;

  { TFPMultiplyOperation }

  TFPMultiplyOperation = Class(TMathOperation)
  Public
    Procedure check; override;
    Function AsString : string ; override;
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  end;

  { TFPDivideOperation }

  TFPDivideOperation = Class(TMathOperation)
  Public
    Procedure Check; override;
    Function AsString : string ; override;
    Function NodeType : TResultType; override;
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  end;

  { TFPModuloOperation }

  TFPModuloOperation = Class(TMathOperation)
  Public
    Procedure Check; override;
    Function AsString : string ; override;
    Function NodeType : TResultType; override;
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  end;


  { TFPPowerOperation }
  TFPPowerOperation = class(TMathOperation)
  public
    Procedure Check; override;
    Function AsString : string ; override;
    Function NodeType : TResultType; override;
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  end;


  { TFPUnaryOperator }

  TFPUnaryOperator = Class(TFPExprNode)
  private
    FOperand: TFPExprNode;
  Public
    Constructor Create(AOperand : TFPExprNode);
    Destructor Destroy; override;
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
    Function HasAggregate : Boolean; override;
    Procedure Check; override;
    Property Operand : TFPExprNode Read FOperand;
  end;

  { TFPConvertNode }

  TFPConvertNode = Class(TFPUnaryOperator)
    Function AsString : String; override;
  end;

  { TFPNotNode }

  TFPNotNode = Class(TFPUnaryOperator)
  Public
    Procedure Check; override;
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
    Function AsString : String; override;
  end;

  TIntConvertNode = Class(TFPConvertNode)
  Public
    Procedure Check; override;
  end;

  { TIntToFloatNode }

  TIntToFloatNode = Class(TIntConvertNode)
  Public
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
  end;

  { TIntToCurrencyNode }

  TIntToCurrencyNode = Class(TIntConvertNode)
  Public
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
  end;

  { TIntToDateTimeNode }

  TIntToDateTimeNode = Class(TIntConvertNode)
  Public
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
  end;

  { TFloatToDateTimeNode }

  TFloatToDateTimeNode = Class(TFPConvertNode)
  Public
    Procedure Check; override;
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
  end;

  { TFloatToCurrencyNode }

  TFloatToCurrencyNode = Class(TFPConvertNode)
  Public
    Procedure Check; override;
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
  end;

  { TCurrencyToDateTimeNode }

  TCurrencyToDateTimeNode = Class(TFPConvertNode)
  Public
    Procedure Check; override;
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
  end;

  { TCurrencyToFloatNode }

  TCurrencyToFloatNode = Class(TFPConvertNode)
  Public
    Procedure Check; override;
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
  end;


  { TFPNegateOperation }

  TFPNegateOperation = Class(TFPUnaryOperator)
  Public
    Procedure Check; override;
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
    Function AsString : String; override;
  end;

  { TFPConstExpression }

  TFPConstExpression = Class(TFPExprnode)
  private
    FValue : TFPExpressionResult;
  public
    Constructor CreateString(AValue : String);
    Constructor CreateInteger(AValue : Int64);
    Constructor CreateDateTime(AValue : TDateTime);
    Constructor CreateFloat(AValue : TExprFloat);
    Constructor CreateBoolean(AValue : Boolean);
    constructor CreateCurrency(AValue: Currency);
    Procedure Check; override;
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
    Function AsString : string ; override;
   // For inspection
    Property ConstValue : TFPExpressionResult read FValue;
  end;


  TIdentifierType = (itVariable,itFunctionCallBack,itFunctionHandler,itFunctionNode);
  TFPExprFunctionCallBack = Procedure (Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
  TFPExprFunctionEvent = Procedure (Var Result : TFPExpressionResult; Const Args : TExprParameterArray) of object;
  TFPExprVariableCallBack = Procedure (Var Result : TFPExpressionResult; ConstRef AName : ShortString);
  TFPExprVariableEvent = Procedure (Var Result : TFPExpressionResult; ConstRef AName : ShortString) of Object;

  { TFPExprIdentifierDef }

  TFPExprIdentifierDef = Class(TCollectionItem)
  private
    FNodeType: TFPExprFunctionClass;
    FOnGetVarValue: TFPExprVariableEvent;
    FOnGetVarValueCB: TFPExprVariableCallBack;
    FStringValue : String;
    FValue : TFPExpressionResult;
    FArgumentTypes: String;
    FIDType: TIdentifierType;
    FName: ShortString;
    FVariableArgumentCount: Boolean;
    FOnGetValue: TFPExprFunctionEvent;
    FOnGetValueCB: TFPExprFunctionCallBack;
    function GetAsBoolean: Boolean;
    function GetAsDateTime: TDateTime;
    function GetAsFloat: TExprFloat;
    function GetAsCurrency : Currency;
    function GetAsInteger: Int64;
    function GetAsString: String;
    function GetResultType: TResultType;
    function GetValue: String;
    procedure SetArgumentTypes(const AValue: String);
    procedure SetAsBoolean(const AValue: Boolean);
    procedure SetAsDateTime(const AValue: TDateTime);
    procedure SetAsFloat(const AValue: TExprFloat);
    procedure SetAsCurrency(const AValue: Currency);
    procedure SetAsInteger(const AValue: Int64);
    procedure SetAsString(const AValue: String);
    procedure SetName(const AValue: ShortString);
    procedure SetResultType(const AValue: TResultType);
    procedure SetValue(const AValue: String);
  Protected
    Procedure CheckResultType(Const AType : TResultType);
    Procedure CheckVariable;
    Procedure FetchValue;
  Public
    Function ArgumentCount : Integer;
    Procedure Assign(Source : TPersistent); override;
    Function EventBasedVariable : Boolean; Inline;
    Property AsFloat : TExprFloat Read GetAsFloat Write SetAsFloat;
    Property AsCurrency : Currency Read GetAsCurrency Write SetAsCurrency;
    Property AsInteger : Int64 Read GetAsInteger Write SetAsInteger;
    Property AsString : String Read GetAsString Write SetAsString;
    Property AsBoolean : Boolean Read GetAsBoolean Write SetAsBoolean;
    Property AsDateTime : TDateTime Read GetAsDateTime Write SetAsDateTime;
    Property OnGetFunctionValueCallBack : TFPExprFunctionCallBack Read FOnGetValueCB Write FOnGetValueCB;
    Property OnGetVariableValueCallBack : TFPExprVariableCallBack Read FOnGetVarValueCB Write FOnGetVarValueCB;
  Published
    Property IdentifierType : TIdentifierType Read FIDType Write FIDType;
    Property Name : ShortString Read FName Write SetName;
    Property Value : String Read GetValue Write SetValue;
    Property ParameterTypes : String Read FArgumentTypes Write SetArgumentTypes;
    Property ResultType : TResultType Read GetResultType Write SetResultType;
    Property OnGetFunctionValue : TFPExprFunctionEvent Read FOnGetValue Write FOnGetValue;
    Property OnGetVariableValue : TFPExprVariableEvent Read FOnGetVarValue Write FOnGetVarValue;
    Property NodeType : TFPExprFunctionClass Read FNodeType Write FNodeType;
    property VariableArgumentCount: Boolean read FVariableArgumentCount write FVariableArgumentCount;
  end;


  TBuiltInCategory = (bcStrings,bcDateTime,bcMath,bcBoolean,bcConversion,bcData,bcVaria,bcUser,bcAggregate);
  TBuiltInCategories = Set of TBuiltInCategory;

  { TFPBuiltInExprIdentifierDef }

  TFPBuiltInExprIdentifierDef = Class(TFPExprIdentifierDef)
  private
    FCategory: TBuiltInCategory;
  Public
    Procedure Assign(Source : TPersistent); override;
  Published
    Property Category : TBuiltInCategory Read FCategory Write FCategory;
  end;

  { TFPExprIdentifierDefs }

  TFPExprIdentifierDefs = Class(TCollection)
  private
    FParser: TFPExpressionParser;
    function GetI(AIndex : Integer): TFPExprIdentifierDef;
    procedure SetI(AIndex : Integer; const AValue: TFPExprIdentifierDef);
  Protected
    procedure Update(Item: TCollectionItem); override;
    Property Parser: TFPExpressionParser Read FParser;
  Public
    Function IndexOfIdentifier(Const AName : ShortString) : Integer;
    Function FindIdentifier(Const AName : ShortString) : TFPExprIdentifierDef;
    Function IdentifierByName(Const AName : ShortString) : TFPExprIdentifierDef;
    Function AddVariable(Const AName : ShortString; AResultType : TResultType; ACallback : TFPExprVariableCallBack) : TFPExprIdentifierDef;
    Function AddVariable(Const AName : ShortString; AResultType : TResultType; ACallback : TFPExprVariableEvent) : TFPExprIdentifierDef;
    Function AddVariable(Const AName : ShortString; AResultType : TResultType; AValue : String) : TFPExprIdentifierDef;
    Function AddBooleanVariable(Const AName : ShortString; AValue : Boolean) : TFPExprIdentifierDef;
    Function AddIntegerVariable(Const AName : ShortString; AValue : Integer) : TFPExprIdentifierDef;
    Function AddFloatVariable(Const AName : ShortString; AValue : TExprFloat) : TFPExprIdentifierDef;
    Function AddCurrencyVariable(Const AName : ShortString; AValue : Currency) : TFPExprIdentifierDef;
    Function AddStringVariable(Const AName : ShortString; AValue : String) : TFPExprIdentifierDef;
    Function AddDateTimeVariable(Const AName : ShortString; AValue : TDateTime) : TFPExprIdentifierDef;
    Function AddFunction(Const AName : ShortString; Const AResultType : Char; Const AParamTypes : String; ACallBack : TFPExprFunctionCallBack) : TFPExprIdentifierDef;
    Function AddFunction(Const AName : ShortString; Const AResultType : Char; Const AParamTypes : String; ACallBack : TFPExprFunctionEvent) : TFPExprIdentifierDef;
    Function AddFunction(Const AName : ShortString; Const AResultType : Char; Const AParamTypes : String; ANodeClass : TFPExprFunctionClass) : TFPExprIdentifierDef;
    property Identifiers[AIndex : Integer] : TFPExprIdentifierDef Read GetI Write SetI; Default;
  end;

  { TFPExprIdentifierNode }

  TFPExprIdentifierNode = Class(TFPExprNode)
  Private
    FID : TFPExprIdentifierDef;
    PResult : PFPExpressionResult;
    FResultType : TResultType;
  public
    Constructor CreateIdentifier(AID : TFPExprIdentifierDef);
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
    Property Identifier : TFPExprIdentifierDef Read FID;
  end;

  { TFPExprVariable }

  TFPExprVariable = Class(TFPExprIdentifierNode)
    Procedure Check; override;
    function AsString: string; override;
  end;

  { TFPExprFunction }

  TFPExprFunction = Class(TFPExprIdentifierNode)
  private
    FArgumentNodes : TExprArgumentArray;
    FargumentParams : TExprParameterArray;
  Protected
    Procedure CalcParams;
    function ConvertArgument(aIndex: Integer; aNode: TFPExprNode; aType: TResultType): TFPExprNode; virtual;
  Public
    Procedure Check; override;
    Constructor CreateFunction(AID : TFPExprIdentifierDef; Const Args : TExprArgumentArray); virtual;
    Destructor Destroy; override;
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
    Function HasAggregate : Boolean; override;
    Property ArgumentNodes : TExprArgumentArray Read FArgumentNodes;
    Property ArgumentParams : TExprParameterArray Read FArgumentParams;
    Function AsString : String; override;
  end;

  { TAggregateExpr }

  TAggregateExpr = Class(TFPExprFunction)
  Protected
    FResult : TFPExpressionResult;
  public
    Class Function IsAggregate : Boolean; override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
  end;

  { TAggregateMin }

  TAggregateMin = Class(TAggregateExpr)
  Public
    FFirst: Boolean;
  Public
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
  end;

  { TAggregateMax }

  TAggregateMax = Class(TAggregateExpr)
  Public
    FFirst: Boolean;
  Public
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
  end;

  { TAggregateSum }

  TAggregateSum = Class(TAggregateExpr)
  Public
    function ConvertArgument(aIndex: Integer; aNode: TFPExprNode; aType: TResultType): TFPExprNode; override;
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
  end;

  { TAggregateAvg }

  TAggregateAvg = Class(TAggregateSum)
  Protected
    FCount : Integer;
  Public
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
  end;

  { TAggregateCount }

  TAggregateCount = Class(TAggregateExpr)
  Public
    Procedure InitAggregate; override;
    Procedure UpdateAggregate; override;
  end;

  { TFPFunctionCallBack }

  TFPFunctionCallBack = Class(TFPExprFunction)
  Private
    FCallBack : TFPExprFunctionCallBack;
  Public
    Constructor CreateFunction(AID : TFPExprIdentifierDef; Const Args : TExprArgumentArray); override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
    Property CallBack : TFPExprFunctionCallBack Read FCallBack;
  end;

  { TFPFunctionEventHandler }

  TFPFunctionEventHandler = Class(TFPExprFunction)
  Private
    FCallBack : TFPExprFunctionEvent;
  Public
    Constructor CreateFunction(AID : TFPExprIdentifierDef; Const Args : TExprArgumentArray); override;
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
    Property CallBack : TFPExprFunctionEvent Read FCallBack;
  end;

  { TFPExpressionParser }

  TFPExpressionParser = class(TComponent)
  private
    FBuiltIns: TBuiltInCategories;
    FExpression: String;
    FScanner : TFPExpressionScanner;
    FExprNode : TFPExprNode;
    FIdentifiers : TFPExprIdentifierDefs;
    FHashList : TFPHashObjectlist;
    FDirty : Boolean;
    procedure CheckEOF;
    function GetAsBoolean: Boolean;
    function GetAsDateTime: TDateTime;
    function GetAsFloat: TExprFloat;
    function GetAsCurrency: Currency;
    function GetAsInteger: Int64;
    function GetAsString: String;
    function MatchNodes(Todo, Match: TFPExprNode): TFPExprNode;
    procedure CheckNodes(var Left, Right: TFPExprNode);
    procedure SetBuiltIns(const AValue: TBuiltInCategories);
    procedure SetIdentifiers(const AValue: TFPExprIdentifierDefs);
  Protected
    procedure ParserError(Msg: String);
    procedure SetExpression(const AValue: String); virtual;
    Procedure CheckResultType(Const Res :TFPExpressionResult; AType : TResultType); inline;
    Procedure CheckResultTypes(Const Res :TFPExpressionResult; ATypes : TResultTypes); inline;
    Class function ConvertNode(Todo: TFPExprNode; ToType: TResultType): TFPExprNode;
    class Function BuiltinsManager : TExprBuiltInManager;
    Function Level1 : TFPExprNode;
    Function Level2 : TFPExprNode;
    Function Level3 : TFPExprNode;
    Function Level4 : TFPExprNode;
    Function Level5 : TFPExprNode;
    Function Level6 : TFPExprNode;
    Function Level7 : TFPExprNode;
    Function Primitive : TFPExprNode;
    function GetToken: TTokenType;
    Function TokenType : TTokenType;
    Function CurrentToken : String;
    Procedure CreateHashList;
    Property Scanner : TFPExpressionScanner Read FScanner;
    Property ExprNode : TFPExprNode Read FExprNode;
    Property Dirty : Boolean Read FDirty;
  public
    Constructor Create(AOwner :TComponent); override;
    Destructor Destroy; override;
    Function IdentifierByName(const AName : ShortString) : TFPExprIdentifierDef; virtual;
    Procedure Clear;
    Procedure EvaluateExpression(Out Result : TFPExpressionResult);
    function ExtractNode(var N: TFPExprNode): Boolean;
    Function Evaluate : TFPExpressionResult;
    Function ResultType : TResultType;
    Function HasAggregate : Boolean;
    Procedure InitAggregate;
    Procedure UpdateAggregate;
    Property AsFloat : TExprFloat Read GetAsFloat;
    Property AsCurrency : Currency Read GetAsCurrency;
    Property AsInteger : Int64 Read GetAsInteger;
    Property AsString : String Read GetAsString;
    Property AsBoolean : Boolean Read GetAsBoolean;
    Property AsDateTime : TDateTime Read GetAsDateTime;
  Published
    // The Expression to parse
    property Expression : String read FExpression write SetExpression;
    Property Identifiers : TFPExprIdentifierDefs Read FIdentifiers Write SetIdentifiers;
    Property BuiltIns : TBuiltInCategories Read FBuiltIns Write SetBuiltIns;
  end;
  TFPExpressionParserClass = Class of TFPExpressionParser;

  { TExprBuiltInManager }

  TExprBuiltInManager = Class(TComponent)
  Private
    FDefs : TFPExprIdentifierDefs;
    function GetCount: Integer;
    function GetI(AIndex : Integer): TFPBuiltInExprIdentifierDef;
  protected
    Property Defs : TFPExprIdentifierDefs Read FDefs;
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Function IndexOfIdentifier(Const AName : ShortString) : Integer;
    Function FindIdentifier(Const AName : ShortString) : TFPBuiltinExprIdentifierDef;
    Function IdentifierByName(Const AName : ShortString) : TFPBuiltinExprIdentifierDef;
    Function AddVariable(Const ACategory : TBuiltInCategory; Const AName : ShortString; AResultType : TResultType; AValue : String) : TFPBuiltInExprIdentifierDef;
    Function AddBooleanVariable(Const ACategory : TBuiltInCategory; Const AName : ShortString; AValue : Boolean) : TFPBuiltInExprIdentifierDef;
    Function AddIntegerVariable(Const ACategory : TBuiltInCategory; Const AName : ShortString; AValue : Integer) : TFPBuiltInExprIdentifierDef;
    Function AddFloatVariable(Const ACategory : TBuiltInCategory; Const AName : ShortString; AValue : TExprFloat) : TFPBuiltInExprIdentifierDef;
    Function AddCurrencyVariable(Const ACategory : TBuiltInCategory; Const AName : ShortString; AValue : Currency) : TFPBuiltInExprIdentifierDef;
    Function AddStringVariable(Const ACategory : TBuiltInCategory; Const AName : ShortString; AValue : String) : TFPBuiltInExprIdentifierDef;
    Function AddDateTimeVariable(Const ACategory : TBuiltInCategory; Const AName : ShortString; AValue : TDateTime) : TFPBuiltInExprIdentifierDef;
    Function AddFunction(Const ACategory : TBuiltInCategory; Const AName : ShortString; Const AResultType : Char; Const AParamTypes : String; ACallBack : TFPExprFunctionCallBack) : TFPBuiltInExprIdentifierDef;
    Function AddFunction(Const ACategory : TBuiltInCategory; Const AName : ShortString; Const AResultType : Char; Const AParamTypes : String; ACallBack : TFPExprFunctionEvent) : TFPBuiltInExprIdentifierDef;
    Function AddFunction(Const ACategory : TBuiltInCategory; Const AName : ShortString; Const AResultType : Char; Const AParamTypes : String; ANodeClass : TFPExprFunctionClass) : TFPBuiltInExprIdentifierDef;
    Procedure Delete(AIndex: Integer);
    Function Remove(aIdentifier : String) : Integer;
    Property IdentifierCount : Integer Read GetCount;
    Property Identifiers[AIndex : Integer] :TFPBuiltInExprIdentifierDef Read GetI;
  end;

  EExprParser = Class(Exception);

Const
  AllBuiltIns = [bcStrings,bcDateTime,bcMath,bcBoolean,bcConversion,bcData,bcVaria,bcUser,bcAggregate];

Function TokenName (AToken : TTokenType) : String;
Function ResultTypeName (AResult : TResultType) : String;
Function CharToResultType(C : Char) : TResultType;
Function BuiltinIdentifiers : TExprBuiltInManager;
Procedure RegisterStdBuiltins(AManager : TExprBuiltInManager; Categories : TBuiltInCategories = AllBuiltIns);
function ArgToFloat(Arg: TFPExpressionResult): TExprFloat;



implementation

uses typinfo;

{ TFPExpressionParser }

const
  cNull=#0;
  cSingleQuote = '''';
  cHexIdentifier = '$';
  cOctalIdentifier = '&';
  cBinaryIdentifier = '%';

  Digits        = ['0'..'9','.'];
  HexDigits     = ['0'..'9', 'A'..'F', 'a'..'f'];
  OctalDigits   = ['0'..'7'];
  BinaryDigits  = ['0', '1'];
  WhiteSpace    = [' ',#13,#10,#9];
  Operators     = ['+','-','<','>','=','/','*','^'];
  Delimiters    = Operators+[',','(',')'];
  Symbols       = ['%']+Delimiters;
  WordDelimiters = WhiteSpace + Symbols;

var
  FileFormatSettings: TFormatSettings;

Resourcestring
  SBadQuotes        = 'Unterminated string';
  SUnknownDelimiter = 'Unknown delimiter character: "%s"';
  SErrUnknownCharacter = 'Unknown character at pos %d: "%s"';
  SErrUnexpectedEndOfExpression = 'Unexpected end of expression';
  SErrUnknownComparison = 'Internal error: Unknown comparison';
  SErrUnknownBooleanOp = 'Internal error: Unknown boolean operation';
  SErrBracketExpected = 'Expected ) bracket at position %d, but got %s';
  SerrUnknownTokenAtPos = 'Unknown token at pos %d : %s';
  SErrLeftBracketExpected = 'Expected ( bracket at position %d, but got %s';
  SErrInvalidFloat = '%s is not a valid floating-point value';
  SErrUnknownIdentifier = 'Unknown identifier: %s';
  SErrInExpression = 'Cannot evaluate: error in expression';
  SErrInExpressionEmpty = 'Cannot evaluate: empty expression';
  SErrCommaExpected =  'Expected comma (,) at position %d, but got %s';
  SErrInvalidNumberChar = 'Unexpected character in number : %s';
  SErrInvalidNumber = 'Invalid numerical value : %s';
  SErrUnterminatedIdentifier = 'Unterminated quoted identifier: %s';
  SErrNoOperand = 'No operand for unary operation %s';
  SErrNoleftOperand = 'No left operand for binary operation %s';
  SErrNoRightOperand = 'No right operand for binary operation %s';
  SErrNoNegation = 'Cannot negate expression of type %s : %s';
  SErrNoNOTOperation = 'Cannot perform "not" on expression of type %s: %s';
  SErrTypesDoNotMatch = 'Type mismatch: %s<>%s for expressions "%s" and "%s".';
  SErrTypesIncompatible = 'Incompatible types: %s<>%s for expressions "%s" and "%s".';
  SErrNoNodeToCheck = 'Internal error: No node to check !';
  SInvalidNodeType = 'Node type (%s) not in allowed types (%s) for expression: %s';
  SErrUnterminatedExpression = 'Badly terminated expression. Found token at position %d : %s';
  SErrDuplicateIdentifier = 'An identifier with name "%s" already exists.';
  SErrInvalidResultCharacter = '"%s" is not a valid return type indicator';
  ErrInvalidArgumentCount = 'Invalid argument count for function %s';
  SErrInvalidArgumentType = 'Invalid type for argument %d: Expected %s, got %s';
  SErrInvalidResultType = 'Invalid result type: %s';
  SErrNotVariable = 'Identifier %s is not a variable';
  SErrInactive = 'Operation not allowed while an expression is active';
  SErrIFNeedsBoolean = 'First argument to IF must be of type boolean: %s';
  SErrCaseNeeds3 = 'Case statement needs to have at least 4 arguments';
  SErrCaseEvenCount = 'Case statement needs to have an even number of arguments';
  SErrCaseLabelNotAConst = 'Case label %d "%s" is not a constant expression';
  SErrCaseLabelType = 'Case label %d "%s" needs type %s, but has type %s';
  SErrCaseValueType = 'Case value %d "%s" needs type %s, but has type %s';

{ ---------------------------------------------------------------------
  Auxiliary functions
  ---------------------------------------------------------------------}

Procedure RaiseParserError(Msg : String);
begin
  Raise EExprParser.Create(Msg);
end;

Procedure RaiseParserError(Fmt : String; Args : Array of const);
begin
  Raise EExprParser.CreateFmt(Fmt,Args);
end;

function TokenName(AToken: TTokenType): String;

begin
  Result:=GetEnumName(TypeInfo(TTokenType),Ord(AToken));
end;

function ResultTypeName(AResult: TResultType): String;

begin
  Result:=GetEnumName(TypeInfo(TResultType),Ord(AResult));
end;

function CharToResultType(C: Char): TResultType;
begin
  Case Upcase(C) of
    'S' : Result:=rtString;
    'D' : Result:=rtDateTime;
    'B' : Result:=rtBoolean;
    'I' : Result:=rtInteger;
    'F' : Result:=rtFloat;
    'C' : Result:=rtCurrency;
  else
    RaiseParserError(SErrInvalidResultCharacter,[C]);
  end;
end;

Var
  BuiltIns : TExprBuiltInManager;

function BuiltinIdentifiers: TExprBuiltInManager;

begin
  If (BuiltIns=Nil) then
    BuiltIns:=TExprBuiltInManager.Create(Nil);
  Result:=BuiltIns;
end;

Procedure FreeBuiltIns;

begin
  FreeAndNil(Builtins);
end;

{ TFloatToCurrencyNode }

procedure TFloatToCurrencyNode.Check;
begin
  CheckNodeType(Operand,[rtFloat]);
end;

function TFloatToCurrencyNode.NodeType: TResultType;
begin
  Result:=rtCurrency;
end;

procedure TFloatToCurrencyNode.GetNodeValue(var Result: TFPExpressionResult);
begin
  Operand.GetNodeValue(Result);
  Result.ResultType:=rtCurrency;
  Result.ResCurrency:=Result.ResFloat;
end;

{ TIntToCurrencyNode }

function TIntToCurrencyNode.NodeType: TResultType;
begin
  Result:=rtCurrency;
end;

procedure TIntToCurrencyNode.GetNodeValue(var Result: TFPExpressionResult);
begin
  Operand.GetNodeValue(Result);
  Result.ResCurrency:=Result.ResInteger;
  Result.ResultType:=rtCurrency;
end;

{ TFPModuloOperation }

procedure TFPModuloOperation.Check;
begin
  CheckNodeType(Left,[rtInteger]);
  CheckNodeType(Right,[rtInteger]);
  inherited Check;
end;

function TFPModuloOperation.AsString: string;
begin
  Result:=Left.AsString+' mod '+Right.asString;
end;

function TFPModuloOperation.NodeType: TResultType;
begin
  Result:=rtInteger;
end;

procedure TFPModuloOperation.GetNodeValue(var Result: TFPExpressionResult);

Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  Result.ResInteger:=Result.ResInteger mod RRes.ResInteger;
  Result.ResultType:=rtInteger;
end;

{ TAggregateMax }

procedure TAggregateMax.InitAggregate;
begin
  inherited InitAggregate;
  FFirst:=True;
  FResult.ResultType:=FArgumentNodes[0].NodeType;
  Case FResult.ResultType of
    rtFloat : FResult.resFloat:=0.0;
    rtCurrency : FResult.resCurrency:=0.0;
    rtInteger : FResult.resInteger:=0;
  end;
end;

procedure TAggregateMax.UpdateAggregate;

Var
  OK : Boolean;
  N : TFPExpressionResult;

begin
  FArgumentNodes[0].GetNodeValue(N);
  if FFirst then
    begin
    FResult.ResultType:=N.ResultType;
    FFirst:=False;
    OK:=True;
    end
  else
    Case N.ResultType of
      rtFloat: OK:=N.ResFloat>FResult.ResFloat;
      rtCurrency: OK:=N.ResCurrency>FResult.ResCurrency;
      rtinteger: OK:=N.ResInteger>FResult.ResFloat;
    end;
  if OK then
    Case N.ResultType of
      rtFloat: FResult.ResFloat:=N.ResFloat;
      rtinteger: FResult.ResFloat:=N.ResInteger;
      rtCurrency: FResult.ResCurrency:=N.ResCurrency;
    end;
end;

{ TAggregateMin }

procedure TAggregateMin.InitAggregate;
begin
  inherited InitAggregate;
  FFirst:=True;
  FResult.ResultType:=FArgumentNodes[0].NodeType;
  Case FResult.ResultType of
    rtFloat : FResult.resFloat:=0.0;
    rtCurrency : FResult.resCurrency:=0.0;
    rtInteger : FResult.resInteger:=0;
  end;
end;

procedure TAggregateMin.UpdateAggregate;

Var
  OK : Boolean;
  N : TFPExpressionResult;

begin
  FArgumentNodes[0].GetNodeValue(N);
  if FFirst then
    begin
    FFirst:=False;
    OK:=True;
    end
  else
    Case N.ResultType of
      rtFloat: OK:=N.ResFloat<FResult.ResFloat;
      rtCurrency: OK:=N.ResCurrency<FResult.ResCurrency;
      rtinteger: OK:=N.ResInteger<FResult.ResFloat;
    end;
  if OK then
    Case FResult.ResultType of
      rtFloat: FResult.ResFloat:=N.ResFloat;
      rtCurrency: FResult.ResCurrency:=N.ResCurrency;
      rtinteger: FResult.ResFloat:=N.ResInteger;
    end;
  inherited UpdateAggregate;
end;

{ TAggregateAvg }

procedure TAggregateAvg.InitAggregate;
begin
  inherited InitAggregate;
end;

procedure TAggregateAvg.UpdateAggregate;
begin
  inherited UpdateAggregate;
  Inc(FCount);
end;

procedure TAggregateAvg.GetNodeValue(var Result: TFPExpressionResult);
begin
  inherited GetNodeValue(Result);
  Result.ResultType:=FResult.ResultType;
  if FCount=0 then
    Case FResult.ResultType of
      rtInteger:
        begin
        Result.ResultType:=rtFloat;
        Result.ResFloat:=0.0;
        end;
      rtFloat:
        Result.ResFloat:=0.0;
      rtCurrency:
        Result.ResCurrency:=0.0;
    end
  else
    Case FResult.ResultType of
      rtInteger:
        begin
        Result.ResultType:=rtFloat;
        Result.ResFloat:=FResult.ResInteger/FCount;
        end;
      rtFloat:
        Result.ResFloat:=FResult.ResFloat/FCount;
      rtCurrency:
        Result.ResCurrency:=FResult.ResCurrency/FCount;
    end;
end;

{ TAggregateCount }

procedure TAggregateCount.InitAggregate;
begin
  FResult.ResultType:=rtInteger;
  FResult.ResInteger:=0;
end;

procedure TAggregateCount.UpdateAggregate;
begin
  Inc(FResult.ResInteger);
end;

{ TAggregateExpr }

class function TAggregateExpr.IsAggregate: Boolean;
begin
  Result:=True;
end;

procedure TAggregateExpr.GetNodeValue(var Result: TFPExpressionResult);
begin
  Result:=FResult;
end;

{ TAggregateSum }

function TAggregateSum.ConvertArgument(aIndex: Integer; aNode: TFPExprNode; aType: TResultType): TFPExprNode;
begin
  if not (aNode.NodeType in [rtFloat,rtInteger,rtCurrency]) then
    RaiseParserError(SErrInvalidArgumentType,[aIndex,ResultTypeName(aType),ResultTypeName(aNode.NodeType)]);
  Result:=aNode;
end;

procedure TAggregateSum.InitAggregate;

begin
  FResult.ResultType:=FArgumentNodes[0].NodeType;
  Case FResult.ResultType of
    rtFloat: FResult.ResFloat:=0.0;
    rtCurrency : FResult.ResCurrency:=0.0;
    rtinteger: FResult.ResInteger:=0;
  end;
end;

procedure TAggregateSum.UpdateAggregate;

Var
  R : TFPExpressionResult;

begin
  FArgumentNodes[0].GetNodeValue(R);
  Case FResult.ResultType of
    rtFloat: FResult.ResFloat:=FResult.ResFloat+R.ResFloat;
    rtCurrency: FResult.ResCurrency:=FResult.ResCurrency+R.ResCurrency;
    rtinteger: FResult.ResInteger:=FResult.ResInteger+R.ResInteger;
  end;
end;

{ ---------------------------------------------------------------------
  TFPExpressionScanner
  ---------------------------------------------------------------------}

function TFPExpressionScanner.IsAlpha(C: Char): Boolean;
begin
  Result := C in ['A'..'Z', 'a'..'z'];
end;

constructor TFPExpressionScanner.Create;
begin
  Source:='';
end;


procedure TFPExpressionScanner.SetSource(const AValue: String);
begin
  FSource:=AValue;
  LSource:=Length(FSource);
  FTokenType:=ttEOF;
  If LSource=0 then
    FPos:=0
  else
    FPos:=1;
  FChar:=Pchar(FSource);
  FToken:='';
end;

function TFPExpressionScanner.NextPos: Char;
begin
  Inc(FPos);
  Inc(FChar);
  Result:=FChar^;
end;


function TFPExpressionScanner.IsWordDelim(C: Char): Boolean;
begin
  Result:=C in WordDelimiters;
end;

function TFPExpressionScanner.IsDelim(C: Char): Boolean;
begin
  Result:=C in Delimiters;
end;

function TFPExpressionScanner.IsDigit(C: Char; AKind: TNumberKind): Boolean;
begin
  case AKind of
    nkDecimal: Result := C in Digits;
    nkHex    : Result := C in HexDigits;
    nkOctal  : Result := C in OctalDigits;
    nkBinary : Result := C in BinaryDigits;
  end;
end;

Procedure TFPExpressionScanner.SkipWhiteSpace;

begin
  While (FChar^ in WhiteSpace) and (FPos<=LSource) do
    NextPos;
end;

Function TFPExpressionScanner.DoDelimiter : TTokenType;

Var
  B : Boolean;
  C,D : Char;

begin
  C:=FChar^;
  FToken:=C;
  B:=C in ['<','>'];
  D:=C;
  C:=NextPos;

  if B and (C in ['=','>']) then
    begin
    FToken:=FToken+C;
    NextPos;
    If (D='>') then
      Result:=ttLargerThanEqual
    else if (C='>') then
      Result:=ttUnequal
    else
      Result:=ttLessThanEqual;
    end
  else
    Case D of
      '+' : Result := ttPlus;
      '-' : Result := ttMinus;
      '<' : Result := ttLessThan;
      '>' : Result := ttLargerThan;
      '=' : Result := ttEqual;
      '/' : Result := ttDiv;
      '*' : Result := ttMul;
      '(' : Result := ttLeft;
      ')' : Result := ttRight;
      ',' : Result := ttComma;
      '^' : Result := ttPower;
    else
      ScanError(Format(SUnknownDelimiter,[D]));
    end;

end;

Procedure TFPExpressionScanner.ScanError(Msg : String);

begin
  Raise EExprScanner.Create(Msg)
end;

Function TFPExpressionScanner.DoString : TTokenType;

  Function TerminatingChar(C : Char) : boolean;

  begin
    Result:=(C=cNull) or
            ((C=cSingleQuote) and
              Not ((FPos<LSource) and (FSource[FPos+1]=cSingleQuote)));
  end;


Var
  C : Char;

begin
  FToken := '';
  C:=NextPos;
  while not TerminatingChar(C) do
    begin
    FToken:=FToken+C;
    If C=cSingleQuote then
      NextPos;
    C:=NextPos;
    end;
  if (C=cNull) then
    ScanError(SBadQuotes);
  Result := ttString;
  FTokenType:=Result;
  NextPos;
end;

function TFPExpressionScanner.GetCurrentChar: Char;
begin
  If FChar<>Nil then
    Result:=FChar^
  else
    Result:=#0;
end;

procedure Val(const S: string; out V: TExprFloat; out Code: Integer);
var
  L64: Int64;
begin
  if (S <> '') and (S[1] in ['&', '$', '%']) then
  begin
    System.Val(S, L64, Code);
    if Code = 0 then
      V := L64
  end
  else
    System.Val(S, V, Code);
end;

Function TFPExpressionScanner.DoNumber(AKind: TNumberKind) : TTokenType;

Var
  C : Char;
  X : TExprFloat;
  I : Integer;
  prevC: Char;

  function ValidDigit(C: Char; AKind: TNumberKind): Boolean;
  begin
    Result := IsDigit(C, AKind);
    if (not Result) then
      case AKind of
        nkDecimal:
          Result := ((FToken <> '') and (UpCase(C)='E')) or
                    ((FToken <> '') and (C in ['+','-']) and (prevC='E'));
        nkHex:
          Result := (C = cHexIdentifier) and (prevC = #0);
        nkOctal:
          Result := (C = cOctalIdentifier) and (prevC = #0);
        nkBinary:
          Result := (C = cBinaryIdentifier) and (prevC = #0);
      end;
  end;

begin
  C:=CurrentChar;
  prevC := #0;
  while (C <> cNull) do
  begin
    if IsWordDelim(C) then
      case AKind of
        nkDecimal:
          if not (prevC in ['E','-','+']) then break;
        nkHex, nkOctal:
          break;
        nkBinary:
          if (prevC <> #0) then break;   // allow '%' as first char
      end;
    if not ValidDigit(C, AKind) then
      ScanError(Format(SErrInvalidNumberChar,[C]));
    FToken := FToken+C;
    prevC := Upcase(C);
    C:=NextPos;
    end;
  Val(FToken,X,I);
  If (I<>0) then
    ScanError(Format(SErrInvalidNumber,[FToken]));
  Result:=ttNumber;
end;

Function TFPExpressionScanner.DoIdentifier : TTokenType;

Var
  C : Char;
  S : String;
begin
  C:=CurrentChar;
  while (not IsWordDelim(C)) and (C<>cNull) do
    begin
    if (C<>'"') then
      FToken:=FToken+C
    else
      begin
      C:=NextPos;
      While Not (C in [cNull,'"']) do
        begin
        FToken:=FToken+C;
        C:=NextPos;
        end;
      if (C<>'"') then
        ScanError(Format(SErrUnterminatedIdentifier,[FToken]));
      end;
    C:=NextPos;
    end;
  S:=LowerCase(Token);
  If (S='or') then
    Result:=ttOr
  else if (S='xor') then
    Result:=ttXOr
  else if (S='and') then
    Result:=ttAnd
  else if (S='true') then
    Result:=ttTrue
  else if (S='false') then
    Result:=ttFalse
  else if (S='not') then
    Result:=ttnot
  else if (S='if') then
    Result:=ttif
  else if (S='case') then
    Result:=ttcase
  else if (S='mod') then
    Result:=ttMod
  else
    Result:=ttIdentifier;
end;

Function TFPExpressionScanner.GetToken : TTokenType;

Var
  C : Char;

begin
  FToken := '';
  SkipWhiteSpace;
  C:=FChar^;
  if c=cNull then
    Result:=ttEOF
  else if IsDelim(C) then
    Result:=DoDelimiter
  else if (C=cSingleQuote) then
    Result:=DoString
  else if (C=cHexIdentifier) then
    Result := DoNumber(nkHex)
  else if (C=cOctalIdentifier) then
    Result := DoNumber(nkOctal)
  else if (C=cBinaryIdentifier) then
    Result := DoNumber(nkBinary)
  else if IsDigit(C, nkDecimal) then
    Result:=DoNumber(nkDecimal)
  else if IsAlpha(C) or (C='"') then
    Result:=DoIdentifier
  else
    ScanError(Format(SErrUnknownCharacter,[FPos,C]))  ;
  FTokenType:=Result;
end;

{ ---------------------------------------------------------------------
  TFPExpressionParser
  ---------------------------------------------------------------------}

function TFPExpressionParser.TokenType: TTokenType;

begin
  Result:=FScanner.TokenType;
end;

function TFPExpressionParser.CurrentToken: String;
begin
  Result:=FScanner.Token;
end;

procedure TFPExpressionParser.CreateHashList;

Var
  ID : TFPExpridentifierDef;
  BID : TFPBuiltinExpridentifierDef;
  I  : Integer;
  M : TExprBuiltinManager;

begin
  FHashList.Clear;
  // Builtins
  M:=BuiltinsManager;
  If (FBuiltins<>[]) and Assigned(M) then
    For I:=0 to M.IdentifierCount-1 do
      begin
      BID:=M.Identifiers[I];
      If BID.Category in FBuiltins then
        FHashList.Add(LowerCase(BID.Name),BID);
      end;
  // User
  For I:=0 to FIdentifiers.Count-1 do
    begin
    ID:=FIdentifiers[i];
    FHashList.Add(LowerCase(ID.Name),ID);
    end;
  FDirty:=False;
end;

function TFPExpressionParser.IdentifierByName(const AName: ShortString): TFPExprIdentifierDef;
begin
  If FDirty then
    CreateHashList;
  Result:=TFPExprIdentifierDef(FHashList.Find(LowerCase(AName)));
end;

procedure TFPExpressionParser.Clear;
begin
  FExpression:='';
  FHashList.Clear;
  FExprNode.Free;
end;

constructor TFPExpressionParser.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FIdentifiers:=TFPExprIdentifierDefs.Create(TFPExprIdentifierDef);
  FIdentifiers.FParser:=Self;
  FScanner:=TFPExpressionScanner.Create;
  FHashList:=TFPHashObjectList.Create(False);
end;

destructor TFPExpressionParser.Destroy;
begin
  FreeAndNil(FHashList);
  FreeAndNil(FExprNode);
  FreeAndNil(FIdentifiers);
  FreeAndNil(FScanner);
  inherited Destroy;
end;

function TFPExpressionParser.GetToken: TTokenType;

begin
  Result:=FScanner.GetToken;
end;

procedure TFPExpressionParser.CheckEOF;

begin
  If (TokenType=ttEOF) then
    ParserError(SErrUnexpectedEndOfExpression);
end;

procedure TFPExpressionParser.SetIdentifiers(const AValue: TFPExprIdentifierDefs
  );
begin
  FIdentifiers.Assign(AValue)
end;

procedure TFPExpressionParser.EvaluateExpression(Out Result: TFPExpressionResult);
begin
  If (FExpression='') then
    ParserError(SErrInExpressionEmpty);
  if not Assigned(FExprNode) then
    ParserError(SErrInExpression);
  FExprNode.GetNodeValue(Result);
end;

function TFPExpressionParser.ExtractNode(Var N : TFPExprNode) : Boolean;
begin
  Result:=Assigned(FExprNode);
  if Result then
    begin
    N:=FExprNode;
    FExprNode:=Nil;
    FExpression:='';
    end;
end;

procedure TFPExpressionParser.ParserError(Msg: String);
begin
  Raise EExprParser.Create(Msg);
end;

Class function TFPExpressionParser.ConvertNode(Todo : TFPExprNode; ToType : TResultType): TFPExprNode;
begin
  Result:=ToDo;
  Case ToDo.NodeType of
    rtInteger :
      Case ToType of
        rtFloat    : Result:=TIntToFloatNode.Create(Result);
        rtCurrency : Result:=TIntToCurrencyNode.Create(Result);
        rtDateTime : Result:=TIntToDateTimeNode.Create(Result);
      end;
    rtFloat :
      Case ToType of
        rtCurrency : Result:=TFloatToCurrencyNode.Create(Result);
        rtDateTime : Result:=TFloatToDateTimeNode.Create(Result);
      end;
    rtCurrency :
      Case ToType of
        rtFloat    : Result:=TCurrencyToFloatNode.Create(Result);
        rtDateTime : Result:=TCurrencyToDateTimeNode.Create(Result);
      end;
  end;
end;

function TFPExpressionParser.GetAsBoolean: Boolean;

var
  Res: TFPExpressionResult;

begin
  EvaluateExpression(Res);
  CheckResultType(Res,rtBoolean);
  Result:=Res.ResBoolean;
end;

function TFPExpressionParser.GetAsDateTime: TDateTime;
var
  Res: TFPExpressionResult;

begin
  EvaluateExpression(Res);
  CheckResultType(Res,rtDateTime);
  Result:=Res.ResDatetime;
end;

function TFPExpressionParser.GetAsFloat: TExprFloat;

var
  Res: TFPExpressionResult;

begin
  EvaluateExpression(Res);
  CheckResultTypes(Res,[rtFloat,rtCurrency,rtInteger]);
  case Res.ResultType of
    rtInteger : Result:=Res.ResInteger;
    rtFloat : Result:=Res.ResFloat;
    rtCurrency : Result:=res.ResCurrency;
  end;
end;

function TFPExpressionParser.GetAsCurrency: Currency;
var
  Res: TFPExpressionResult;

begin
  EvaluateExpression(Res);
  CheckResultTypes(Res,[rtFloat,rtCurrency,rtInteger]);
  case Res.ResultType of
    rtInteger : Result:=Res.ResInteger;
    rtFloat : Result:=Res.ResFloat;
    rtCurrency : Result:=res.ResCurrency;
  end;
end;

function TFPExpressionParser.GetAsInteger: Int64;

var
  Res: TFPExpressionResult;

begin
  EvaluateExpression(Res);
  CheckResultType(Res,rtInteger);
  Result:=Res.ResInteger;
end;

function TFPExpressionParser.GetAsString: String;

var
  Res: TFPExpressionResult;

begin
  EvaluateExpression(Res);
  CheckResultType(Res,rtString);
  Result:=Res.ResString;
end;

{
  Checks types of todo and match. If ToDO can be converted to it matches
  the type of match, then a node is inserted.
  For binary operations, this function is called for both operands.
}

function TFPExpressionParser.MatchNodes(Todo,Match : TFPExprNode): TFPExprNode;

Var
  FromType,ToType : TResultType;

begin
  Result:=Todo;
  FromType:=Todo.NodeType;
  ToType:=Match.NodeType;
  If (FromType<>ToType) then
    Case FromType of
    rtInteger:
      if (ToType in [rtFloat,rtCurrency,rtDateTime]) then
        Result:=ConvertNode(Todo,toType);
    rtFloat:
      if (ToType in [rtCurrency,rtDateTime]) then
        Result:=ConvertNode(Todo,toType);
    rtCurrency:
      if (ToType in [rtFloat,rtDateTime]) then
        Result:=ConvertNode(Todo,toType);
    end;
end;

{
  if the result types differ, they are converted to a common type if possible.
}

procedure TFPExpressionParser.CheckNodes(var Left, Right: TFPExprNode);

begin
  Left:=MatchNodes(Left,Right);
  Right:=MatchNodes(Right,Left);
end;

procedure TFPExpressionParser.SetBuiltIns(const AValue: TBuiltInCategories);
begin
  if FBuiltIns=AValue then exit;
  FBuiltIns:=AValue;
  FDirty:=True;
end;

function TFPExpressionParser.Level1: TFPExprNode;

var
  tt: TTokenType;
  Right : TFPExprNode;

begin
{$ifdef debugexpr}Writeln('Level 1 ',TokenName(TokenType),': ',CurrentToken);{$endif debugexpr}
  if TokenType = ttNot then
    begin
    GetToken;
    CheckEOF;
    Right:=Level2;
    Result:=TFPNotNode.Create(Right);
    end
  else
    Result:=Level2;
  Try
    while (TokenType in [ttAnd,ttOr,ttXor]) do
      begin
      tt:=TokenType;
      GetToken;
      CheckEOF;
      Right:=Level2;
      Case tt of
        ttOr  : Result:=TFPBinaryOrOperation.Create(Result,Right);
        ttAnd : Result:=TFPBinaryAndOperation.Create(Result,Right);
        ttXor : Result:=TFPBinaryXorOperation.Create(Result,Right);
      Else
        ParserError(SErrUnknownBooleanOp)
      end;
      end;
  Except
    Result.Free;
    Raise;
  end;
end;

function TFPExpressionParser.Level2: TFPExprNode;

var
  Right : TFPExprNode;
  tt : TTokenType;
  C : TFPBinaryOperationClass;

begin
{$ifdef debugexpr}  Writeln('Level 2 ',TokenName(TokenType),': ',CurrentToken);{$endif debugexpr}
  Result:=Level3;
  try
    if (TokenType in ttComparisons) then
      begin
      tt:=TokenType;
      GetToken;
      CheckEOF;
      Right:=Level3;
      CheckNodes(Result,Right);
      Case tt of
        ttLessthan         : C:=TFPLessThanOperation;
        ttLessthanEqual    : C:=TFPLessThanEqualOperation;
        ttLargerThan       : C:=TFPGreaterThanOperation;
        ttLargerThanEqual  : C:=TFPGreaterThanEqualOperation;
        ttEqual            : C:=TFPEqualOperation;
        ttUnequal          : C:=TFPUnequalOperation;
      Else
        ParserError(SErrUnknownComparison)
      end;
      Result:=C.Create(Result,Right);
      end;
  Except
    Result.Free;
    Raise;
  end;
end;

function TFPExpressionParser.Level3: TFPExprNode;

var
  tt : TTokenType;
  right : TFPExprNode;

begin
{$ifdef debugexpr}  Writeln('Level 3 ',TokenName(TokenType),': ',CurrentToken);{$endif debugexpr}
  Result:=Level4;
  try
    while TokenType in [ttPlus,ttMinus] do
      begin
      tt:=TokenType;
      GetToken;
      CheckEOF;
      Right:=Level4;
      CheckNodes(Result,Right);
      Case tt of
        ttPlus  : Result:=TFPAddOperation.Create(Result,Right);
        ttMinus : Result:=TFPSubtractOperation.Create(Result,Right);
      end;
      end;
  Except
    Result.Free;
    Raise;
  end;
end;




function TFPExpressionParser.Level4: TFPExprNode;

var
  tt : TTokenType;
  right : TFPExprNode;

begin
{$ifdef debugexpr}  Writeln('Level 4 ',TokenName(TokenType),': ',CurrentToken);{$endif debugexpr}
  Result:=Level5;
  try
    while (TokenType in [ttMul,ttDiv,ttMod]) do
      begin
      tt:=TokenType;
      GetToken;
      Right:=Level5;
      CheckNodes(Result,Right);
      Case tt of
        ttMul : Result:=TFPMultiplyOperation.Create(Result,Right);
        ttDiv : Result:=TFPDivideOperation.Create(Result,Right);
        ttMod : Result:=TFPModuloOperation.Create(Result,Right);
      end;
      end;
  Except
    Result.Free;
    Raise;
  end;
end;

function TFPExpressionParser.Level5: TFPExprNode;

Var
  B : Boolean;

begin
{$ifdef debugexpr}  Writeln('Level 5 ',TokenName(TokenType),': ',CurrentToken);{$endif debugexpr}
  B:=False;
  if (TokenType in [ttPlus,ttMinus]) then
    begin
    B:=TokenType=ttMinus;
    GetToken;
    end;
  Result:=Level6;
  If B then
    Result:=TFPNegateOperation.Create(Result);
end;

function TFPExpressionParser.Level6: TFPExprNode;
var
  right: TFPExprNode;
begin
{$ifdef debugexpr} Writeln('Level 6 ',TokenName(TokenType),': ',CurrentToken);{$endif debugexpr}
  Result := Level7;
  try
    while (TokenType = ttPower) do
    begin
      GetToken;
      right := Level5;           // Accept '(', unary '+', '-' as next tokens
      CheckNodes(Result, right);
      Result := TFPPowerOperation.Create(Result, right);
    end;
  except
    Result.Free;
    Raise;
  end;
end;

function TFPExpressionParser.Level7: TFPExprNode;
begin
{$ifdef debugexpr}  Writeln('Level 7 ',TokenName(TokenType),': ',CurrentToken);{$endif debugexpr}
  if (TokenType=ttLeft) then
    begin
    GetToken;
    Result:=Level1;
    try
      if (TokenType<>ttRight) then
        ParserError(Format(SErrBracketExpected,[SCanner.Pos,CurrentToken]));
      GetToken;
    Except
      Result.Free;
      Raise;
    end;
    end
  else
    Result:=Primitive;
end;

function TFPExpressionParser.Primitive: TFPExprNode;

Var
  I : Int64;
  C : Integer;
  X : TExprFloat;
  ACount : Integer;
  IFF : Boolean;
  IFC : Boolean;
  ID : TFPExprIdentifierDef;
  Args : TExprArgumentArray;
  AI : Integer;

begin
{$ifdef debugexpr}  Writeln('Primitive : ',TokenName(TokenType),': ',CurrentToken);{$endif debugexpr}
  SetLength(Args,0);
  if (TokenType=ttNumber) then
    begin
    if TryStrToInt64(CurrentToken,I) then
      Result:=TFPConstExpression.CreateInteger(I)
    else
      begin
      Val(CurrentToken,X,C);
      If (C=0) then
        Result:=TFPConstExpression.CreateFloat(X)
      else
        ParserError(Format(SErrInvalidFloat,[CurrentToken]));
      end;
    end
  else if (TokenType=ttString) then
    Result:=TFPConstExpression.CreateString(CurrentToken)
  else if (TokenType in [ttTrue,ttFalse]) then
    Result:=TFPConstExpression.CreateBoolean(TokenType=ttTrue)
  else if Not (TokenType in [ttIdentifier,ttIf,ttcase]) then
    ParserError(Format(SerrUnknownTokenAtPos,[Scanner.Pos,CurrentToken]))
  else
    begin
    IFF:=TokenType=ttIf;
    IFC:=TokenType=ttCase;
    if Not (IFF or IFC) then
      begin
      ID:=self.IdentifierByName(CurrentToken);
      If (ID=Nil) then
        ParserError(Format(SErrUnknownIdentifier,[CurrentToken]))
      end;
    // Determine number of arguments
    if Iff then
      ACount:=3
    else if IfC then
      ACount:=-4
    else if (ID.IdentifierType in [itFunctionCallBack,itFunctionHandler,itFunctionNode]) then
      ACount:=ID.ArgumentCount
    else
      ACount:=0;
    // Parse arguments.
    // Negative is for variable number of arguments, where Abs(value) is the minimum number of arguments
    If (ACount<>0) then
      begin
      GetToken;
      If (TokenType<>ttLeft) then
         ParserError(Format(SErrLeftBracketExpected,[Scanner.Pos,CurrentToken]));
      SetLength(Args,Abs(ACount));
      AI:=0;
      Try
        Repeat
          GetToken;
          // Check if we must enlarge the argument array
          If (ACount<0) and (AI=Length(Args)) then
            begin
            SetLength(Args,AI+1);
            Args[AI]:=Nil;
            end;
          Args[AI]:=Level1;
          Inc(AI);
          If (TokenType<>ttComma) then
            If (AI<Abs(ACount)) then
              ParserError(Format(SErrCommaExpected,[Scanner.Pos,CurrentToken]))
        Until (AI=ACount) or ((ACount<0) and (TokenType=ttRight));
        If TokenType<>ttRight then
          ParserError(Format(SErrBracketExpected,[Scanner.Pos,CurrentToken]));
      except
        On E : Exception do
          begin
          Dec(AI);
          While (AI>=0) do
            begin
            FreeAndNil(Args[Ai]);
            Dec(AI);
            end;
          Raise;
          end;
      end;
      end;
    If Iff then
      Result:=TIfOperation.Create(Args[0],Args[1],Args[2])
    else If IfC then
      Result:=TCaseOperation.Create(Args)
    else
      Case ID.IdentifierType of
        itVariable         : Result:= TFPExprVariable.CreateIdentifier(ID);
        itFunctionCallBack : Result:= TFPFunctionCallback.CreateFunction(ID,Args);
        itFunctionHandler  : Result:= TFPFunctionEventHandler.CreateFunction(ID,Args);
        itFunctionNode     : Result:= ID.NodeType.CreateFunction(ID,Args);
      end;
    end;
  GetToken;
end;


procedure TFPExpressionParser.SetExpression(const AValue: String);
begin
  if FExpression=AValue then exit;
  FExpression:=AValue;
  FScanner.Source:=AValue;
  If Assigned(FExprNode) then
    FreeAndNil(FExprNode);
  If (FExpression<>'') then
    begin
    GetToken;
    FExprNode:=Level1;
    If (TokenType<>ttEOF) then
      ParserError(Format(SErrUnterminatedExpression,[Scanner.Pos,CurrentToken]));
    FExprNode.Check;
    end
  else
    FExprNode:=Nil;
end;

procedure TFPExpressionParser.CheckResultType(const Res: TFPExpressionResult;
  AType: TResultType); inline;
begin
  If (Res.ResultType<>AType) then
    RaiseParserError(SErrInvalidResultType,[ResultTypeName(Res.ResultType)]);
end;

procedure TFPExpressionParser.CheckResultTypes(const Res: TFPExpressionResult; ATypes: TResultTypes);
begin
  If Not (Res.ResultType in ATypes) then
    RaiseParserError(SErrInvalidResultType,[ResultTypeName(Res.ResultType)]);
end;

class function TFPExpressionParser.BuiltinsManager: TExprBuiltInManager;
begin
  Result:=BuiltinIdentifiers;
end;


function TFPExpressionParser.Evaluate: TFPExpressionResult;
begin
  EvaluateExpression(Result);
end;

function TFPExpressionParser.ResultType: TResultType;
begin
  if not Assigned(FExprNode) then
    ParserError(SErrInExpression);
  Result:=FExprNode.NodeType;
end;

function TFPExpressionParser.HasAggregate: Boolean;
begin
  Result:=Assigned(FExprNode) and FExprNode.HasAggregate;
end;

procedure TFPExpressionParser.InitAggregate;
begin
  If Assigned(FExprNode) then
    FExprNode.InitAggregate;
end;

procedure TFPExpressionParser.UpdateAggregate;
begin
  If Assigned(FExprNode) then
    FExprNode.UpdateAggregate;
end;

{ ---------------------------------------------------------------------
  TFPExprIdentifierDefs
  ---------------------------------------------------------------------}

function TFPExprIdentifierDefs.GetI(AIndex : Integer): TFPExprIdentifierDef;
begin
  Result:=TFPExprIdentifierDef(Items[AIndex]);
end;

procedure TFPExprIdentifierDefs.SetI(AIndex : Integer;
  const AValue: TFPExprIdentifierDef);
begin
  Items[AIndex]:=AValue;
end;

procedure TFPExprIdentifierDefs.Update(Item: TCollectionItem);
begin
  If Assigned(FParser) then
    FParser.FDirty:=True;
end;

function TFPExprIdentifierDefs.IndexOfIdentifier(const AName: ShortString
  ): Integer;
begin
  Result:=Count-1;
  While (Result>=0) And (CompareText(GetI(Result).Name,AName)<>0) do
    Dec(Result);
end;

function TFPExprIdentifierDefs.FindIdentifier(const AName: ShortString
  ): TFPExprIdentifierDef;

Var
  I : Integer;

begin
  I:=IndexOfIdentifier(AName);
  If (I=-1) then
    Result:=Nil
  else
    Result:=GetI(I);
end;

function TFPExprIdentifierDefs.IdentifierByName(const AName: ShortString
  ): TFPExprIdentifierDef;
begin
  Result:=FindIdentifier(AName);
  if (Result=Nil) then
    RaiseParserError(SErrUnknownIdentifier,[AName]);
end;

function TFPExprIdentifierDefs.AddVariable(const AName: ShortString;
  AResultType: TResultType; ACallback: TFPExprVariableCallBack
  ): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=AResultType;
  Result.OnGetVariableValueCallBack:=ACallBack
end;

function TFPExprIdentifierDefs.AddVariable(const AName: ShortString;
  AResultType: TResultType; ACallback: TFPExprVariableEvent
  ): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=AResultType;
  Result.OnGetVariableValue:=ACallBack
end;

function TFPExprIdentifierDefs.AddVariable(const AName: ShortString;
  AResultType: TResultType; AValue: String): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=AResultType;
  Result.Value:=AValue;
end;

function TFPExprIdentifierDefs.AddBooleanVariable(const AName: ShortString;
  AValue: Boolean): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=rtBoolean;
  Result.FValue.ResBoolean:=AValue;
end;

function TFPExprIdentifierDefs.AddIntegerVariable(const AName: ShortString;
  AValue: Integer): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=rtInteger;
  Result.FValue.ResInteger:=AValue;
end;

function TFPExprIdentifierDefs.AddFloatVariable(const AName: ShortString;
  AValue: TExprFloat): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=rtFloat;
  Result.FValue.ResFloat:=AValue;
end;

function TFPExprIdentifierDefs.AddCurrencyVariable(const AName: ShortString; AValue: Currency): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=rtCurrency;
  Result.FValue.ResCurrency:=AValue;
end;

function TFPExprIdentifierDefs.AddStringVariable(const AName: ShortString;
  AValue: String): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=rtString;
  Result.FValue.ResString:=AValue;
end;

function TFPExprIdentifierDefs.AddDateTimeVariable(const AName: ShortString;
  AValue: TDateTime): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=rtDateTime;
  Result.FValue.ResDateTime:=AValue;
end;

function TFPExprIdentifierDefs.AddFunction(const AName: ShortString;
  const AResultType: Char; const AParamTypes: String;
  ACallBack: TFPExprFunctionCallBack): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.Name:=Aname;
  Result.IdentifierType:=itFunctionCallBack;
  if (AParamTypes <> '') and (AParamTypes[Length(AParamTypes)] = '+') then begin
    Result.ParameterTypes := Copy(AParamTypes, 1, Length(AParamTypes)-1);
    Result.FVariableArgumentCount := true;
  end else
    Result.ParameterTypes := AParamTypes;
  Result.ResultType:=CharToResultType(AResultType);
  Result.FOnGetValueCB:=ACallBack;
end;

function TFPExprIdentifierDefs.AddFunction(const AName: ShortString;
  const AResultType: Char; const AParamTypes: String;
  ACallBack: TFPExprFunctionEvent): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.Name:=Aname;
  Result.IdentifierType:=itFunctionHandler;
  if (AParamTypes <> '') and (AParamTypes[Length(AParamTypes)] = '+') then begin
    Result.ParameterTypes := Copy(AParamTypes, 1, Length(AParamTypes)-1);
    Result.FVariableArgumentCount := true;
  end else
    Result.ParameterTypes := AParamTypes;
  Result.ResultType:=CharToResultType(AResultType);
  Result.FOnGetValue:=ACallBack;
end;

function TFPExprIdentifierDefs.AddFunction(const AName: ShortString;
  const AResultType: Char; const AParamTypes: String;
  ANodeClass: TFPExprFunctionClass): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.Name:=Aname;
  Result.IdentifierType:=itFunctionNode;
  if (AParamTypes <> '') and (AParamTypes[Length(AParamTypes)] = '+') then begin
    Result.ParameterTypes := Copy(AParamTypes, 1, Length(AParamTypes)-1);
    Result.FVariableArgumentCount := true;
  end else
    Result.ParameterTypes := AParamTypes;
  Result.ResultType:=CharToResultType(AResultType);
  Result.FNodeType:=ANodeClass;
end;


{ ---------------------------------------------------------------------
  TFPExprIdentifierDef
  ---------------------------------------------------------------------}

procedure TFPExprIdentifierDef.SetName(const AValue: ShortString);
begin
  if FName=AValue then exit;
  If (AValue<>'') then
    If Assigned(Collection) and (TFPExprIdentifierDefs(Collection).IndexOfIdentifier(AValue)<>-1) then
      RaiseParserError(SErrDuplicateIdentifier,[AValue]);
  FName:=AValue;
end;

procedure TFPExprIdentifierDef.SetResultType(const AValue: TResultType);

begin
  If AValue<>FValue.ResultType then
    begin
    FValue.ResultType:=AValue;
    SetValue(FStringValue);
    end;
end;

procedure TFPExprIdentifierDef.SetValue(const AValue: String);
begin
  FStringValue:=AValue;
  If (AValue<>'') then
    Case FValue.ResultType of
      rtBoolean  : FValue.ResBoolean:=FStringValue='True';
      rtInteger  : FValue.ResInteger:=StrToInt(AValue);
      rtFloat    : FValue.ResFloat:=StrToFloat(AValue, FileFormatSettings);
      rtCurrency : FValue.ResFloat:=StrToCurr(AValue, FileFormatSettings);
      rtDateTime : FValue.ResDateTime:=StrToDateTime(AValue, FileFormatSettings);
      rtString   : FValue.ResString:=AValue;
    end
  else
    Case FValue.ResultType of
      rtBoolean  : FValue.ResBoolean:=False;
      rtInteger  : FValue.ResInteger:=0;
      rtFloat    : FValue.ResFloat:=0.0;
      rtCurrency : FValue.ResCurrency:=0.0;
      rtDateTime : FValue.ResDateTime:=0;
      rtString   : FValue.ResString:='';
    end
end;

procedure TFPExprIdentifierDef.CheckResultType(const AType: TResultType);
begin
  If FValue.ResultType<>AType then
    RaiseParserError(SErrInvalidResultType,[ResultTypeName(AType)])
end;

procedure TFPExprIdentifierDef.CheckVariable;
begin
  If Identifiertype<>itvariable then
    RaiseParserError(SErrNotVariable,[Name]);
  if EventBasedVariable then
    FetchValue;
end;

function TFPExprIdentifierDef.ArgumentCount: Integer;
begin
  if FVariableArgumentCount then
    Result := -Length(FArgumentTypes)
  else
    Result:=Length(FArgumentTypes);
end;

procedure TFPExprIdentifierDef.Assign(Source: TPersistent);

Var
  EID : TFPExprIdentifierDef;

begin
  if (Source is TFPExprIdentifierDef) then
    begin
    EID:=Source as TFPExprIdentifierDef;
    FStringValue:=EID.FStringValue;
    FValue:=EID.FValue;
    FArgumentTypes:=EID.FArgumentTypes;
    FVariableArgumentCount := EID.FVariableArgumentCount;
    FIDType:=EID.FIDType;
    FName:=EID.FName;
    FOnGetValue:=EID.FOnGetValue;
    FOnGetValueCB:=EID.FOnGetValueCB;
    FOnGetVarValue:=EID.FOnGetVarValue;
    FOnGetVarValueCB:=EID.FOnGetVarValueCB;
    end
  else
    inherited Assign(Source);
end;

procedure TFPExprIdentifierDef.SetArgumentTypes(const AValue: String);

Var
  I : integer;

begin
  if FArgumentTypes=AValue then exit;
  For I:=1 to Length(AValue) do
    CharToResultType(AValue[i]);
  FArgumentTypes:=AValue;
end;

procedure TFPExprIdentifierDef.SetAsBoolean(const AValue: Boolean);
begin
  CheckVariable;
  CheckResultType(rtBoolean);
  FValue.ResBoolean:=AValue;
end;

procedure TFPExprIdentifierDef.SetAsDateTime(const AValue: TDateTime);
begin
  CheckVariable;
  CheckResultType(rtDateTime);
  FValue.ResDateTime:=AValue;
end;

procedure TFPExprIdentifierDef.SetAsFloat(const AValue: TExprFloat);
begin
  CheckVariable;
  CheckResultType(rtFloat);
  FValue.ResFloat:=AValue;
end;

procedure TFPExprIdentifierDef.SetAsCurrency(const AValue: Currency);
begin
  CheckVariable;
  CheckResultType(rtCurrency);
  FValue.ResCurrency:=AValue;
end;

procedure TFPExprIdentifierDef.SetAsInteger(const AValue: Int64);
begin
  CheckVariable;
  CheckResultType(rtInteger);
  FValue.ResInteger:=AValue;
end;

procedure TFPExprIdentifierDef.SetAsString(const AValue: String);
begin
  CheckVariable;
  CheckResultType(rtString);
  FValue.resString:=AValue;
end;

function TFPExprIdentifierDef.GetValue: String;
begin
  Case FValue.ResultType of
    rtBoolean  : If FValue.ResBoolean then
                   Result:='True'
                 else
                   Result:='False';
    rtInteger  : Result:=IntToStr(FValue.ResInteger);
    rtFloat    : Result:=FloatToStr(FValue.ResFloat, FileFormatSettings);
    rtCurrency : Result:=CurrToStr(FValue.ResCurrency, FileFormatSettings);
    rtDateTime : Result:=FormatDateTime('cccc',FValue.ResDateTime, FileFormatSettings);
    rtString   : Result:=FValue.ResString;
  end;
end;

procedure TFPExprIdentifierDef.FetchValue;

Var
  RT,RT2 : TResultType;
  I : Integer;

begin
  RT:=FValue.ResultType;
  if Assigned(FOnGetVarValue) then
    FOnGetVarValue(FValue,FName)
  else
    FOnGetVarValueCB(FValue,FName);
  RT2:=FValue.ResultType;
  if RT2<>RT then
    begin
    // Automatically convert integer to float.
    if (rt2=rtInteger) and (rt=rtFloat) then
      begin
      FValue.ResultType:=RT;
      I:=FValue.resInteger;
      FValue.resFloat:=I;
      end
    else
      begin
      // Restore
      FValue.ResultType:=RT;
      Raise EExprParser.CreateFmt('Value handler for variable %s returned wrong type, expected "%s", got "%s"',[
        FName,
        GetEnumName(TypeInfo(TResultType),Ord(rt)),
        GetEnumName(TypeInfo(TResultType),Ord(rt2))
        ]);
      end;
    end;
end;

function TFPExprIdentifierDef.EventBasedVariable: Boolean;
begin
  Result:=Assigned(FOnGetVarValue) or Assigned(FOnGetVarValueCB);
end;

function TFPExprIdentifierDef.GetResultType: TResultType;
begin
  Result:=FValue.ResultType;
end;

function TFPExprIdentifierDef.GetAsFloat: TExprFloat;
begin
  CheckResultType(rtFloat);
  CheckVariable;
  Result:=FValue.ResFloat;
end;

function TFPExprIdentifierDef.GetAsCurrency: Currency;
begin
  CheckResultType(rtCurrency);
  CheckVariable;
  Result:=FValue.ResCurrency;
end;

function TFPExprIdentifierDef.GetAsBoolean: Boolean;
begin
  CheckResultType(rtBoolean);
  CheckVariable;
  Result:=FValue.ResBoolean;
end;

function TFPExprIdentifierDef.GetAsDateTime: TDateTime;
begin
  CheckResultType(rtDateTime);
  CheckVariable;
  Result:=FValue.ResDateTime;
end;

function TFPExprIdentifierDef.GetAsInteger: Int64;
begin
  CheckResultType(rtInteger);
  CheckVariable;
  Result:=FValue.ResInteger;
end;

function TFPExprIdentifierDef.GetAsString: String;
begin
  CheckResultType(rtString);
  CheckVariable;
  Result:=FValue.ResString;
end;

{ ---------------------------------------------------------------------
  TExprBuiltInManager
  ---------------------------------------------------------------------}

function TExprBuiltInManager.GetCount: Integer;
begin
  Result:=FDefs.Count;
end;

function TExprBuiltInManager.GetI(AIndex : Integer
  ): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs[Aindex])
end;

constructor TExprBuiltInManager.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FDefs:=TFPExprIdentifierDefs.Create(TFPBuiltInExprIdentifierDef)
end;

destructor TExprBuiltInManager.Destroy;
begin
  FreeAndNil(FDefs);
  inherited Destroy;
end;

function TExprBuiltInManager.IndexOfIdentifier(const AName: ShortString
  ): Integer;
begin
  Result:=FDefs.IndexOfIdentifier(AName);
end;

function TExprBuiltInManager.FindIdentifier(const AName: ShortString
  ): TFPBuiltinExprIdentifierDef;
begin
  Result:=TFPBuiltinExprIdentifierDef(FDefs.FindIdentifier(AName));
end;

function TExprBuiltInManager.IdentifierByName(const AName: ShortString
  ): TFPBuiltinExprIdentifierDef;
begin
  Result:=TFPBuiltinExprIdentifierDef(FDefs.IdentifierByName(AName));
end;

function TExprBuiltInManager.AddVariable(const ACategory: TBuiltInCategory;
  const AName: ShortString; AResultType: TResultType; AValue: String
  ): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs.Addvariable(AName,AResultType,AValue));
  Result.Category:=ACategory;
end;

function TExprBuiltInManager.AddBooleanVariable(
  const ACategory: TBuiltInCategory; const AName: ShortString; AValue: Boolean
  ): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs.AddBooleanvariable(AName,AValue));
  Result.Category:=ACategory;
end;

function TExprBuiltInManager.AddIntegerVariable(
  const ACategory: TBuiltInCategory; const AName: ShortString; AValue: Integer
  ): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs.AddIntegerVariable(AName,AValue));
  Result.Category:=ACategory;
end;

function TExprBuiltInManager.AddFloatVariable(
  const ACategory: TBuiltInCategory; const AName: ShortString;
  AValue: TExprFloat): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs.AddFloatVariable(AName,AValue));
  Result.Category:=ACategory;
end;

function TExprBuiltInManager.AddCurrencyVariable(const ACategory: TBuiltInCategory; const AName: ShortString; AValue: Currency
  ): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs.AddCurrencyVariable(AName,AValue));
  Result.Category:=ACategory;
end;

function TExprBuiltInManager.AddStringVariable(
  const ACategory: TBuiltInCategory; const AName: ShortString; AValue: String
  ): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs.AddStringVariable(AName,AValue));
  Result.Category:=ACategory;
end;

function TExprBuiltInManager.AddDateTimeVariable(
  const ACategory: TBuiltInCategory; const AName: ShortString; AValue: TDateTime
  ): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs.AddDateTimeVariable(AName,AValue));
  Result.Category:=ACategory;
end;

function TExprBuiltInManager.AddFunction(const ACategory: TBuiltInCategory;
  const AName: ShortString; const AResultType: Char; const AParamTypes: String;
  ACallBack: TFPExprFunctionCallBack): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs.AddFunction(AName,AResultType,AParamTypes,ACallBack));
  Result.Category:=ACategory;
end;

function TExprBuiltInManager.AddFunction(const ACategory: TBuiltInCategory;
  const AName: ShortString; const AResultType: Char; const AParamTypes: String;
  ACallBack: TFPExprFunctionEvent): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs.AddFunction(AName,AResultType,AParamTypes,ACallBack));
  Result.Category:=ACategory;
end;

function TExprBuiltInManager.AddFunction(const ACategory: TBuiltInCategory;
  const AName: ShortString; const AResultType: Char; const AParamTypes: String;
  ANodeClass: TFPExprFunctionClass): TFPBuiltInExprIdentifierDef;
begin
  Result:=TFPBuiltInExprIdentifierDef(FDefs.AddFunction(AName,AResultType,AParamTypes,ANodeClass));
  Result. Category:=ACategory;
end;

procedure TExprBuiltInManager.Delete(AIndex: Integer);
begin
  FDefs.Delete(AIndex);
end;

function TExprBuiltInManager.Remove(aIdentifier: String): Integer;
begin
  Result:=IndexOfIdentifier(aIdentifier);
  if Result<>-1 then
    Delete(Result);
end;


{ ---------------------------------------------------------------------
  Various Nodes
  ---------------------------------------------------------------------}

{ TFPBinaryOperation }

procedure TFPBinaryOperation.CheckSameNodeTypes;

Var
  LT,RT : TResultType;


begin
  LT:=Left.NodeType;
  RT:=Right.NodeType;
  if (RT<>LT) then
    RaiseParserError(SErrTypesDoNotMatch,[ResultTypeName(LT),ResultTypeName(RT),Left.AsString,Right.AsString])
end;

constructor TFPBinaryOperation.Create(ALeft, ARight: TFPExprNode);
begin
  FLeft:=ALeft;
  FRight:=ARight;
end;

destructor TFPBinaryOperation.Destroy;
begin
  FreeAndNil(FLeft);
  FreeAndNil(FRight);
  inherited Destroy;
end;

procedure TFPBinaryOperation.InitAggregate;
begin
  inherited InitAggregate;
  if Assigned(Left) then
    Left.InitAggregate;
  if Assigned(Right) then
    Right.InitAggregate;
end;

procedure TFPBinaryOperation.UpdateAggregate;
begin
  inherited UpdateAggregate;
  if Assigned(Left) then
    Left.UpdateAggregate;
  if Assigned(Right) then
    Right.UpdateAggregate;
end;

function TFPBinaryOperation.HasAggregate: Boolean;
begin
  Result:=inherited HasAggregate;
  if Assigned(Left) then
    Result:=Result or Left.HasAggregate;
  if Assigned(Right) then
    Result:=Result or Right.HasAggregate;
end;

procedure TFPBinaryOperation.Check;
begin
  If Not Assigned(Left) then
    RaiseParserError(SErrNoLeftOperand,[classname]);
  If Not Assigned(Right) then
    RaiseParserError(SErrNoRightOperand,[classname]);
end;

{ TFPUnaryOperator }

constructor TFPUnaryOperator.Create(AOperand: TFPExprNode);
begin
  FOperand:=AOperand;
end;

destructor TFPUnaryOperator.Destroy;
begin
  FreeAndNil(FOperand);
  inherited Destroy;
end;

procedure TFPUnaryOperator.InitAggregate;
begin
  inherited InitAggregate;
  if Assigned(FOperand) then
    FOperand.InitAggregate;

end;

procedure TFPUnaryOperator.UpdateAggregate;
begin
  inherited UpdateAggregate;
  if Assigned(FOperand) then
    FOperand.UpdateAggregate;
end;

function TFPUnaryOperator.HasAggregate: Boolean;
begin
  Result:=inherited HasAggregate;
  if Assigned(FOperand) then
    Result:=Result or FOperand.HasAggregate;
end;

procedure TFPUnaryOperator.Check;
begin
  If Not Assigned(Operand) then
    RaiseParserError(SErrNoOperand,[Self.className]);
end;

{ TFPConstExpression }

constructor TFPConstExpression.CreateString(AValue: String);
begin
  FValue.ResultType:=rtString;
  FValue.ResString:=AValue;
end;

constructor TFPConstExpression.CreateInteger(AValue: Int64);
begin
  FValue.ResultType:=rtInteger;
  FValue.ResInteger:=AValue;
end;

constructor TFPConstExpression.CreateDateTime(AValue: TDateTime);
begin
  FValue.ResultType:=rtDateTime;
  FValue.ResDateTime:=AValue;
end;

constructor TFPConstExpression.CreateFloat(AValue: TExprFloat);
begin
  Inherited create;
  FValue.ResultType:=rtFloat;
  FValue.ResFloat:=AValue;
end;

constructor TFPConstExpression.CreateCurrency(AValue: Currency);
begin
  Inherited create;
  FValue.ResultType:=rtCurrency;
  FValue.ResCurrency:=AValue;
end;

constructor TFPConstExpression.CreateBoolean(AValue: Boolean);
begin
  FValue.ResultType:=rtBoolean;
  FValue.ResBoolean:=AValue;
end;

procedure TFPConstExpression.Check;
begin
  // Nothing to check;
end;

function TFPConstExpression.NodeType: TResultType;
begin
  Result:=FValue.ResultType;
end;

Procedure TFPConstExpression.GetNodeValue(var Result : TFPExpressionResult);
begin
  Result:=FValue;
end;

function TFPConstExpression.AsString: string ;
begin
  Case NodeType of
    rtString  : Result:=''''+FValue.resString+'''';
    rtInteger : Result:=IntToStr(FValue.resInteger);
    rtDateTime : Result:=''''+FormatDateTime('cccc',FValue.resDateTime)+'''';
    rtBoolean : If FValue.ResBoolean then Result:='True' else Result:='False';
    rtFloat : Str(FValue.ResFloat,Result);
    rtCurrency : Str(FValue.ResCurrency,Result);
  end;
end;


{ TFPNegateOperation }

procedure TFPNegateOperation.Check;
begin
  Inherited;
  If Not (Operand.NodeType in [rtInteger,rtFloat,rtCurrency]) then
    RaiseParserError(SErrNoNegation,[ResultTypeName(Operand.NodeType),Operand.AsString])
end;

function TFPNegateOperation.NodeType: TResultType;
begin
  Result:=Operand.NodeType;
end;

Procedure TFPNegateOperation.GetNodeValue(var Result : TFPExpressionResult);
begin
  Operand.GetNodeValue(Result);
  Case Result.ResultType of
    rtInteger : Result.resInteger:=-Result.ResInteger;
    rtFloat : Result.resFloat:=-Result.ResFloat;
    rtCurrency : Result.resCurrency:=-Result.ResCurrency;
  end;
end;

function TFPNegateOperation.AsString: String;
begin
  Result:='-'+TrimLeft(Operand.AsString);
end;

{ TFPBinaryAndOperation }

procedure TFPBooleanOperation.Check;
begin
  inherited Check;
  CheckNodeType(Left,[rtInteger,rtBoolean]);
  CheckNodeType(Right,[rtInteger,rtBoolean]);
  CheckSameNodeTypes;
end;

function TFPBooleanOperation.NodeType: TResultType;
begin
  Result:=Left.NodeType;
end;

Procedure TFPBinaryAndOperation.GetNodeValue(var Result : TFPExpressionResult);

Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  Case Result.ResultType of
    rtBoolean : Result.resBoolean:=Result.ResBoolean and RRes.ResBoolean;
    rtInteger : Result.resInteger:=Result.ResInteger and RRes.ResInteger;
  end;
end;

function TFPBinaryAndOperation.AsString: string;
begin
  Result:=Left.AsString+' and '+Right.AsString;
end;

{ TFPExprNode }

procedure TFPExprNode.CheckNodeType(Anode: TFPExprNode; Allowed: TResultTypes);

Var
  S : String;
  A : TResultType;

begin
  If (Anode=Nil) then
    RaiseParserError(SErrNoNodeToCheck);
  If Not (ANode.NodeType in Allowed) then
    begin
    S:='';
    For A:=Low(TResultType) to High(TResultType) do
      If A in Allowed then
        begin
        If S<>'' then
          S:=S+',';
        S:=S+ResultTypeName(A);
        end;
    RaiseParserError(SInvalidNodeType,[ResultTypeName(ANode.NodeType),S,ANode.AsString]);
    end;
end;

procedure TFPExprNode.InitAggregate;
begin
  // Do nothing
end;

procedure TFPExprNode.UpdateAggregate;
begin
  // Do nothing
end;

function TFPExprNode.HasAggregate: Boolean;
begin
  Result:=IsAggregate;
end;

class function TFPExprNode.IsAggregate: Boolean;
begin
  Result:=False;
end;

function TFPExprNode.NodeValue: TFPExpressionResult;
begin
  GetNodeValue(Result);
end;

{ TFPBinaryOrOperation }

function TFPBinaryOrOperation.AsString: string;
begin
  Result:=Left.AsString+' or '+Right.AsString;
end;

Procedure TFPBinaryOrOperation.GetNodeValue(var Result : TFPExpressionResult);

Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  Case Result.ResultType of
    rtBoolean : Result.resBoolean:=Result.ResBoolean or RRes.ResBoolean;
    rtInteger : Result.resInteger:=Result.ResInteger or RRes.ResInteger;
  end;
end;

{ TFPBinaryXOrOperation }

function TFPBinaryXOrOperation.AsString: string;
begin
  Result:=Left.AsString+' xor '+Right.AsString;
end;

Procedure TFPBinaryXOrOperation.GetNodeValue(var Result : TFPExpressionResult);
Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  Case Result.ResultType of
    rtBoolean : Result.resBoolean:=Result.ResBoolean xor RRes.ResBoolean;
    rtInteger : Result.resInteger:=Result.ResInteger xor RRes.ResInteger;
  end;
end;

{ TFPNotNode }

procedure TFPNotNode.Check;
begin
  If Not (Operand.NodeType in [rtInteger,rtBoolean]) then
    RaiseParserError(SErrNoNotOperation,[ResultTypeName(Operand.NodeType),Operand.AsString])
end;

function TFPNotNode.NodeType: TResultType;
begin
  Result:=Operand.NodeType;
end;

procedure TFPNotNode.GetNodeValue(var Result: TFPExpressionResult);
begin
  Operand.GetNodeValue(Result);
  Case result.ResultType of
    rtInteger : Result.resInteger:=Not Result.resInteger;
    rtBoolean : Result.resBoolean:=Not Result.resBoolean;
  end
end;

function TFPNotNode.AsString: String;
begin
  Result:='not '+Operand.AsString;
end;

{ TIfOperation }

constructor TIfOperation.Create(ACondition, ALeft, ARight: TFPExprNode);
begin
  Inherited Create(ALeft,ARight);
  FCondition:=ACondition;
end;

destructor TIfOperation.destroy;
begin
  FreeAndNil(FCondition);
  inherited destroy;
end;

procedure TIfOperation.GetNodeValue(var Result: TFPExpressionResult);

begin
  FCondition.GetNodeValue(Result);
  If Result.ResBoolean then
    Left.GetNodeValue(Result)
  else
    Right.GetNodeValue(Result)
end;

procedure TIfOperation.Check;
begin
  inherited Check;
  if (Condition.NodeType<>rtBoolean) then
    RaiseParserError(SErrIFNeedsBoolean,[Condition.AsString]);
  CheckSameNodeTypes;
end;

procedure TIfOperation.InitAggregate;
begin
  inherited InitAggregate;
  If Assigned(FCondition) then
    fCondition.InitAggregate;
end;

procedure TIfOperation.UpdateAggregate;
begin
  inherited UpdateAggregate;
  If Assigned(FCondition) then
    FCondition.UpdateAggregate;
end;

function TIfOperation.HasAggregate: Boolean;
begin
  Result:=inherited HasAggregate;
  if Assigned(Condition) then
    Result:=Result or Condition.HasAggregate;
end;

function TIfOperation.NodeType: TResultType;
begin
  Result:=Left.NodeType;
end;

function TIfOperation.AsString: string;
begin
  Result:=Format('if(%s , %s , %s)',[Condition.AsString,Left.AsString,Right.AsString]);
end;

{ TCaseOperation }

procedure TCaseOperation.GetNodeValue(var Result: TFPExpressionResult);

Var
  I,L : Integer;
  B : Boolean;
  RT,RV : TFPExpressionResult;

begin
  FArgs[0].GetNodeValue(RT);
  L:=Length(FArgs);
  I:=2;
  B:=False;
  While (Not B) and (I<L) do
    begin
    FArgs[i].GetNodeValue(RV);
    Case RT.ResultType of
      rtBoolean  : B:=RT.ResBoolean=RV.ResBoolean;
      rtInteger  : B:=RT.ResInteger=RV.ResInteger;
      rtFloat    : B:=RT.ResFloat=RV.ResFloat;
      rtCurrency : B:=RT.resCurrency=RV.resCurrency;
      rtDateTime : B:=RT.ResDateTime=RV.ResDateTime;
      rtString   : B:=RT.ResString=RV.ResString;
    end;
    If Not B then
      Inc(I,2);
    end;
  // Set result type.
  Result.ResultType:=FArgs[1].NodeType;
  If B then
    FArgs[I+1].GetNodeValue(Result)
  else if ((L mod 2)=0) then
    FArgs[1].GetNodeValue(Result);
end;

procedure TCaseOperation.Check;

Var
  T,V : TResultType;
  I : Integer;
  N : TFPExprNode;

begin
  If (Length(FArgs)<3) then
    RaiseParserError(SErrCaseNeeds3);
  If ((Length(FArgs) mod 2)=1) then
    RaiseParserError(SErrCaseEvenCount);
  T:=FArgs[0].NodeType;
  V:=FArgs[1].NodeType;
  For I:=2 to Length(Fargs)-1 do
    begin
    N:=FArgs[I];
    // Even argument types (labels) must equal tag.
    If ((I mod 2)=0) then
      begin
      If Not (N is TFPConstExpression) then
        RaiseParserError(SErrCaseLabelNotAConst,[I div 2,N.AsString]);
      If (N.NodeType<>T) then
        RaiseParserError(SErrCaseLabelType,[I div 2,N.AsString,ResultTypeName(T),ResultTypeName(N.NodeType)]);
      end
    else // Odd argument types (values) must match first.
      begin
      If (N.NodeType<>V) then
        RaiseParserError(SErrCaseValueType,[(I-1)div 2,N.AsString,ResultTypeName(V),ResultTypeName(N.NodeType)]);
      end
    end;
end;

procedure TCaseOperation.InitAggregate;

Var
  I : Integer;

begin
  inherited InitAggregate;
  if Assigned(FCondition) then
    FCondition.InitAggregate;
  For I:=0 to Length(Fargs)-1 do
    FArgs[i].InitAggregate;
end;

procedure TCaseOperation.UpdateAggregate;
Var
  I : Integer;
begin
  inherited UpdateAggregate;
  if Assigned(FCondition) then
    FCondition.UpdateAggregate;
  For I:=0 to Length(Fargs)-1 do
    FArgs[i].InitAggregate;
end;

Function  TCaseOperation.HasAggregate : Boolean;

Var
  I,L : Integer;
begin
  Result:=inherited HasAggregate;
  L:=Length(Fargs);
  I:=0;
  While (Not Result) and (I<L) do
    begin
    Result:=Result or FArgs[i].HasAggregate;
    Inc(I)
    end;
end;

function TCaseOperation.NodeType: TResultType;
begin
  Result:=FArgs[1].NodeType;
end;

constructor TCaseOperation.Create(Args: TExprArgumentArray);
begin
  Fargs:=Args;
end;

destructor TCaseOperation.destroy;

Var
  I : Integer;

begin
  For I:=0 to Length(FArgs)-1 do
    FreeAndNil(Fargs[I]);
  inherited destroy;
end;

function TCaseOperation.AsString: string;

Var
  I : integer;

begin
  Result:='';
  For I:=0 to Length(FArgs)-1 do
    begin
    If (Result<>'') then
      Result:=Result+', ';
    Result:=Result+FArgs[i].AsString;
    end;
  Result:='Case('+Result+')';
end;

{ TFPBooleanResultOperation }

procedure TFPBooleanResultOperation.Check;
begin
  inherited Check;
  CheckSameNodeTypes;
end;

function TFPBooleanResultOperation.NodeType: TResultType;
begin
  Result:=rtBoolean;
end;

{ TFPEqualOperation }

function TFPEqualOperation.AsString: string;
begin
  Result:=Left.AsString+' = '+Right.AsString;
end;

Procedure TFPEqualOperation.GetNodeValue(var Result : TFPExpressionResult);

Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  Case Result.ResultType of
    rtBoolean  : Result.resBoolean:=Result.ResBoolean=RRes.ResBoolean;
    rtInteger  : Result.resBoolean:=Result.ResInteger=RRes.ResInteger;
    rtFloat    : Result.resBoolean:=Result.ResFloat=RRes.ResFloat;
    rtCurrency : Result.resBoolean:=Result.resCurrency=RRes.resCurrency;
    rtDateTime : Result.resBoolean:=Result.ResDateTime=RRes.ResDateTime;
    rtString   : Result.resBoolean:=Result.ResString=RRes.ResString;
  end;
  Result.ResultType:=rtBoolean;
end;

{ TFPUnequalOperation }

function TFPUnequalOperation.AsString: string;
begin
  Result:=Left.AsString+' <> '+Right.AsString;
end;

Procedure TFPUnequalOperation.GetNodeValue(var Result : TFPExpressionResult);
begin
  Inherited GetNodeValue(Result);
  Result.ResBoolean:=Not Result.ResBoolean;
end;


{ TFPLessThanOperation }

function TFPLessThanOperation.AsString: string;
begin
  Result:=Left.AsString+' < '+Right.AsString;
end;

procedure TFPLessThanOperation.GetNodeValue(var Result : TFPExpressionResult);
Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  Case Result.ResultType of
    rtInteger  : Result.resBoolean:=Result.ResInteger<RRes.ResInteger;
    rtFloat    : Result.resBoolean:=Result.ResFloat<RRes.ResFloat;
    rtCurrency : Result.resBoolean:=Result.resCurrency<RRes.resCurrency;
    rtDateTime : Result.resBoolean:=Result.ResDateTime<RRes.ResDateTime;
    rtString   : Result.resBoolean:=Result.ResString<RRes.ResString;
  end;
  Result.ResultType:=rtBoolean;
end;

{ TFPGreaterThanOperation }

function TFPGreaterThanOperation.AsString: string;
begin
  Result:=Left.AsString+' > '+Right.AsString;
end;

Procedure TFPGreaterThanOperation.GetNodeValue(var Result : TFPExpressionResult);

Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  Case Result.ResultType of
    rtInteger : case Right.NodeType of
                  rtInteger : Result.resBoolean:=Result.ResInteger>RRes.ResInteger;
                  rtFloat : Result.resBoolean:=Result.ResInteger>RRes.ResFloat;
                  rtCurrency : Result.resBoolean:=Result.ResInteger>RRes.resCurrency;
                end;
    rtFloat   : case Right.NodeType of
                  rtInteger : Result.resBoolean:=Result.ResFloat>RRes.ResInteger;
                  rtFloat : Result.resBoolean:=Result.ResFloat>RRes.ResFloat;
                  rtCurrency : Result.resBoolean:=Result.ResFloat>RRes.ResCurrency;
                end;
    rtCurrency   : case Right.NodeType of
                  rtInteger : Result.resBoolean:=Result.ResCurrency>RRes.ResInteger;
                  rtFloat : Result.resBoolean:=Result.ResCurrency>RRes.ResFloat;
                  rtCurrency : Result.resBoolean:=Result.ResCurrency>RRes.ResCurrency;
                end;
    rtDateTime : Result.resBoolean:=Result.ResDateTime>RRes.ResDateTime;
    rtString   : Result.resBoolean:=Result.ResString>RRes.ResString;
  end;
  Result.ResultType:=rtBoolean;
end;

{ TFPGreaterThanEqualOperation }

function TFPGreaterThanEqualOperation.AsString: string;
begin
  Result:=Left.AsString+' >= '+Right.AsString;
end;

Procedure TFPGreaterThanEqualOperation.GetNodeValue(var Result : TFPExpressionResult);
begin
  Inherited GetNodeValue(Result);
  Result.ResBoolean:=Not Result.ResBoolean;
end;

{ TFPLessThanEqualOperation }

function TFPLessThanEqualOperation.AsString: string;
begin
  Result:=Left.AsString+' <= '+Right.AsString;
end;

Procedure TFPLessThanEqualOperation.GetNodeValue(var Result : TFPExpressionResult);
begin
  Inherited GetNodeValue(Result);
  Result.ResBoolean:=Not Result.ResBoolean;
end;

{ TFPOrderingOperation }

procedure TFPOrderingOperation.Check;

Const
  AllowedTypes =[rtInteger,rtfloat,rtCurrency,rtDateTime,rtString];

begin
  CheckNodeType(Left,AllowedTypes);
  CheckNodeType(Right,AllowedTypes);
  inherited Check;
end;

{ TMathOperation }

procedure TMathOperation.Check;

Const
  AllowedTypes =[rtInteger,rtfloat,rtCurrency,rtDateTime,rtString];

begin
  inherited Check;
  CheckNodeType(Left,AllowedTypes);
  CheckNodeType(Right,AllowedTypes);
  CheckSameNodeTypes;
end;

function TMathOperation.NodeType: TResultType;
begin
  Result:=Left.NodeType;
end;

{ TFPAddOperation }

function TFPAddOperation.AsString: string;
begin
  Result:=Left.AsString+' + '+Right.asString;
end;

Procedure TFPAddOperation.GetNodeValue(var Result : TFPExpressionResult);

Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  case Result.ResultType of
    rtInteger  : Result.ResInteger:=Result.ResInteger+RRes.ResInteger;
    rtString   : Result.ResString:=Result.ResString+RRes.ResString;
    rtDateTime : Result.ResDateTime:=Result.ResDateTime+RRes.ResDateTime;
    rtFloat    : Result.ResFloat:=Result.ResFloat+RRes.ResFloat;
    rtCurrency : Result.ResCurrency:=Result.ResCurrency+RRes.ResCurrency;
  end;
  Result.ResultType:=NodeType;
end;

{ TFPSubtractOperation }

procedure TFPSubtractOperation.check;

Const
  AllowedTypes =[rtInteger,rtfloat,rtCurrency,rtDateTime];

begin
  CheckNodeType(Left,AllowedTypes);
  CheckNodeType(Right,AllowedTypes);
  inherited check;
end;

function TFPSubtractOperation.AsString: string;
begin
  Result:=Left.AsString+' - '+Right.asString;
end;

Procedure TFPSubtractOperation.GetNodeValue(var Result : TFPExpressionResult);

Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  case Result.ResultType of
    rtInteger  : Result.ResInteger:=Result.ResInteger-RRes.ResInteger;
    rtDateTime : Result.ResDateTime:=Result.ResDateTime-RRes.ResDateTime;
    rtFloat    : Result.ResFloat:=Result.ResFloat-RRes.ResFloat;
    rtCurrency : Result.resCurrency:=Result.resCurrency-RRes.ResCurrency;
  end;
end;

{ TFPMultiplyOperation }

procedure TFPMultiplyOperation.check;

Const
  AllowedTypes =[rtInteger,rtCurrency,rtfloat];

begin
  CheckNodeType(Left,AllowedTypes);
  CheckNodeType(Right,AllowedTypes);
  Inherited;
end;

function TFPMultiplyOperation.AsString: string;
begin
  Result:=Left.AsString+' * '+Right.asString;
end;

Procedure TFPMultiplyOperation.GetNodeValue(var Result : TFPExpressionResult);
Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  case Result.ResultType of
    rtInteger  : Result.ResInteger:=Result.ResInteger*RRes.ResInteger;
    rtFloat    : Result.ResFloat:=Result.ResFloat*RRes.ResFloat;
    rtCurrency : Result.ResCurrency:=Result.ResCurrency*RRes.ResCurrency;
  end;
end;

{ TFPDivideOperation }

procedure TFPDivideOperation.check;
Const
  AllowedTypes =[rtInteger,rtCurrency,rtfloat];

begin
  CheckNodeType(Left,AllowedTypes);
  CheckNodeType(Right,AllowedTypes);
  inherited check;
end;

function TFPDivideOperation.AsString: string;
begin
  Result:=Left.AsString+' / '+Right.asString;
end;

function TFPDivideOperation.NodeType: TResultType;

begin
  if (Left.NodeType=rtCurrency) and (Right.NodeType=rtCurrency) then
    Result:=rtCurrency
  else
    Result:=rtFloat;
end;

Procedure TFPDivideOperation.GetNodeValue(var Result : TFPExpressionResult);

Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  case Result.ResultType of
    rtInteger  : Result.ResFloat:=Result.ResInteger/RRes.ResInteger;
    rtFloat    : Result.ResFloat:=Result.ResFloat/RRes.ResFloat;
    rtCurrency :
        if NodeType=rtCurrency then
          Result.ResCurrency:=Result.ResCurrency/RRes.ResCurrency
        else
          Result.ResFloat:=Result.ResFloat/RRes.ResFloat;
  end;
  Result.ResultType:=NodeType;
end;

{ TFPPowerOperation }

procedure TFPPowerOperation.Check;
const
  AllowedTypes = [rtInteger, rtCurrency, rtFloat];
begin
  CheckNodeType(Left, AllowedTypes);
  CheckNodeType(Right, AllowedTypes);
end;

function TFPPowerOperation.AsString: String;
begin
  Result := Left.AsString + '^' + Right.AsString;
end;

function TFPPowerOperation.NodeType: TResultType;
begin
  Result := rtFloat;
end;

function power(base,exponent: TExprFloat): TExprFloat;
// Adapted from unit "math"
var
  ex: Integer;
begin
  if Exponent = 0.0 then
    result := 1.0
  else if (base = 0.0) and (exponent > 0.0) then
    result := 0.0
  else if (base < 0.0) and (frac(exponent) = 0.0) then
  begin
    ex := round(exponent);
    result := exp( exponent * ln(-base));
    if odd(ex) then result := -result;
  end
  else
    result := exp( exponent * ln(base) );
end;

procedure TFPPowerOperation.GetNodeValue(var Result: TFPExpressionResult);
var
  RRes: TFPExpressionResult;
begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  Result.ResFloat := power(ArgToFloat(Result), ArgToFloat(RRes));
  Result.ResultType := rtFloat;
end;

{ TFPConvertNode }

function TFPConvertNode.AsString: String;
begin
  Result:=Operand.AsString;
end;

{ TIntToFloatNode }

procedure TIntConvertNode.Check;
begin
  inherited Check;
  CheckNodeType(Operand,[rtInteger])
end;

function TIntToFloatNode.NodeType: TResultType;
begin
  Result:=rtFloat;
end;

Procedure TIntToFloatNode.GetNodeValue(var Result : TFPExpressionResult);
begin
  Operand.GetNodeValue(Result);
  Result.ResFloat:=Result.ResInteger;
  Result.ResultType:=rtFloat;
end;


{ TIntToDateTimeNode }

function TIntToDateTimeNode.NodeType: TResultType;
begin
  Result:=rtDatetime;
end;

procedure TIntToDateTimeNode.GetNodeValue(var Result : TFPExpressionResult);
begin
  Operand.GetnodeValue(Result);
  Result.ResDateTime:=Result.ResInteger;
  Result.ResultType:=rtDateTime;
end;

{ TFloatToDateTimeNode }

procedure TFloatToDateTimeNode.Check;
begin
  inherited Check;
  CheckNodeType(Operand,[rtFloat]);
end;

function TFloatToDateTimeNode.NodeType: TResultType;
begin
  Result:=rtDateTime;
end;

Procedure TFloatToDateTimeNode.GetNodeValue(var Result : TFPExpressionResult);
begin
  Operand.GetNodeValue(Result);
  Result.ResDateTime:=Result.ResFloat;
  Result.ResultType:=rtDateTime;
end;

{ TCurrencyToDateTimeNode }

procedure TCurrencyToDateTimeNode.Check;
begin
  inherited Check;
  CheckNodeType(Operand,[rtCurrency]);
end;

function TCurrencyToDateTimeNode.NodeType: TResultType;
begin
  Result:=rtDateTime;
end;

Procedure TCurrencyToDateTimeNode.GetNodeValue(var Result : TFPExpressionResult);
begin
  Operand.GetNodeValue(Result);
  Result.ResDateTime:=Result.ResCurrency;
  Result.ResultType:=rtDateTime;
end;

{ TCurrencyToFloatNode }

procedure TCurrencyToFloatNode.Check;
begin
  inherited Check;
  CheckNodeType(Operand,[rtCurrency]);
end;

function TCurrencyToFloatNode.NodeType: TResultType;
begin
  Result:=rtFloat;
end;

Procedure TCurrencyToFloatNode.GetNodeValue(var Result : TFPExpressionResult);
begin
  Operand.GetNodeValue(Result);
  Result.ResFloat:=Result.ResCurrency;
  Result.ResultType:=rtFloat;
end;

{ TFPExprIdentifierNode }

constructor TFPExprIdentifierNode.CreateIdentifier(AID: TFPExprIdentifierDef);
begin
  Inherited Create;
  FID:=AID;
  PResult:=@FID.FValue;
  FResultType:=FID.ResultType;
end;

function TFPExprIdentifierNode.NodeType: TResultType;
begin
  Result:=FResultType;
end;

Procedure TFPExprIdentifierNode.GetNodeValue(var Result : TFPExpressionResult);
begin
  if Identifier.EventBasedVariable then
    Identifier.FetchValue;
  Result:=PResult^;
  Result.ResultType:=FResultType;
end;

{ TFPExprVariable }

procedure TFPExprVariable.Check;
begin
  // Do nothing;
end;

function TFPExprVariable.AsString: string;
begin
  Result:=FID.Name;
end;

{ TFPExprFunction }

procedure TFPExprFunction.CalcParams;

Var
  I : Integer;

begin
  For I:=0 to Length(FArgumentParams)-1 do
    begin
    FArgumentNodes[i].GetNodeValue(FArgumentParams[i]);
    end;
end;

Function TFPExprFunction.ConvertArgument(aIndex : Integer; aNode : TFPExprNode; aType : TResultType) : TFPExprNode;

Var
  N : TFPExprNode;

begin
  // Automatically convert integers to floats for float/currency parameters
  N:=TFPExpressionParser.ConvertNode(aNode,aType);
  if (aNode=N) then
    // No conversion was performed, raise error
    RaiseParserError(SErrInvalidArgumentType,[aIndex,ResultTypeName(aType),ResultTypeName(aNode.NodeType)]);
  Result:=N;
end;

function TFPExprFunction.HasAggregate: Boolean;
var
  I: Integer;
begin
  Result := true;
  if IsAggregate then
    exit;
  For I:=0 to Length(FArgumentNodes)-1 do
    if FArgumentNodes[I].HasAggregate then
      exit;
  Result := false;
end;

procedure TFPExprFunction.Check;

Var
  I : Integer;
  rtp,rta : TResultType;

begin
  If (Length(FArgumentNodes)<>FID.ArgumentCount) and not FID.VariableArgumentCount then
    RaiseParserError(ErrInvalidArgumentCount,[FID.Name]);
  For I:=0 to Length(FArgumentNodes)-1 do
    begin
    if (i < Length(FID.ParameterTypes)) then
      rtp := CharToResultType(FID.ParameterTypes[i+1])
    else if FID.VariableArgumentCount then
      rtp := CharToResultType(FID.ParameterTypes[Length(FID.ParameterTypes)]);
    rta:=FArgumentNodes[i].NodeType;
    If (rtp<>rta) then
      FArgumentNodes[i]:=ConvertArgument(I+1,FArgumentNodes[i],rtp);
    end;
end;

constructor TFPExprFunction.CreateFunction(AID: TFPExprIdentifierDef; const Args: TExprArgumentArray);

begin
  Inherited CreateIdentifier(AID);
  FArgumentNodes:=Args;
  SetLength(FArgumentParams,Length(Args));
end;

destructor TFPExprFunction.Destroy;

Var
  I : Integer;

begin
  For I:=0 to Length(FArgumentNodes)-1 do
    FreeAndNil(FArgumentNodes[I]);
  inherited Destroy;
end;

procedure TFPExprFunction.InitAggregate;
var
  I: Integer;
begin
  For I:=0 to Length(FArgumentNodes)-1 do
    FArgumentNodes[i].InitAggregate;
end;

procedure TFPExprFunction.UpdateAggregate;
var
  I: Integer;
begin
  For I:=0 to Length(FArgumentNodes)-1 do
    FArgumentNodes[i].UpdateAggregate;
end;

function TFPExprFunction.AsString: String;

Var
  S : String;
  I : Integer;

begin
  S:='';
  For I:=0 to length(FArgumentNodes)-1 do
    begin
    If (S<>'') then
      S:=S+',';
    S:=S+FArgumentNodes[I].AsString;
    end;
  If (S<>'') then
    S:='('+S+')';
  Result:=FID.Name+S;
end;

{ TFPFunctionCallBack }

constructor TFPFunctionCallBack.CreateFunction(AID: TFPExprIdentifierDef;
  Const Args : TExprArgumentArray);
begin
  Inherited;
  FCallBack:=AID.OnGetFunctionValueCallBack;
end;

Procedure TFPFunctionCallBack.GetNodeValue(var Result : TFPExpressionResult);
begin
  If Length(FArgumentParams)>0 then
    CalcParams;

  FCallBack(Result,FArgumentParams);
  Result.ResultType:=NodeType;
end;

{ TFPFunctionEventHandler }

constructor TFPFunctionEventHandler.CreateFunction(AID: TFPExprIdentifierDef;
  Const Args : TExprArgumentArray);
begin
  Inherited;
  FCallBack:=AID.OnGetFunctionValue;
end;

Procedure TFPFunctionEventHandler.GetNodeValue(var Result : TFPExpressionResult);
begin
  If Length(FArgumentParams)>0 then
    CalcParams;
  FCallBack(Result,FArgumentParams);
  Result.ResultType:=NodeType;
end;

{ ---------------------------------------------------------------------
  Standard Builtins support
  ---------------------------------------------------------------------}

{ Template for builtin.

Procedure MyCallback (Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
end;

}

function ArgToFloat(Arg: TFPExpressionResult): TExprFloat;
// Utility function for the built-in math functions. Accepts also integers
// in place of the floating point arguments. To be called in builtins or
// user-defined callbacks having float results.
begin
  if Arg.ResultType = rtInteger then
    result := Arg.resInteger
  else if Arg.ResultType = rtCurrency then
    result := Arg.resCurrency
  else
    result := Arg.resFloat;
end;

// Math builtins

Procedure BuiltInCos(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=Cos(ArgToFloat(Args[0]));
end;

Procedure BuiltInSin(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=Sin(ArgToFloat(Args[0]));
end;

Procedure BuiltInArcTan(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=Arctan(ArgToFloat(Args[0]));
end;

Procedure BuiltInAbs(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=Abs(ArgToFloat(Args[0]));
end;

Procedure BuiltInSqr(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=Sqr(ArgToFloat(Args[0]));
end;

Procedure BuiltInSqrt(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=Sqrt(ArgToFloat(Args[0]));
end;

Procedure BuiltInExp(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=Exp(ArgToFloat(Args[0]));
end;

Procedure BuiltInLn(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=Ln(ArgToFloat(Args[0]));
end;

Const
  L10 = ln(10);

Procedure BuiltInLog(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=Ln(ArgToFloat(Args[0]))/L10;
end;

Procedure BuiltInRound(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resInteger:=Round(ArgToFloat(Args[0]));
end;

Procedure BuiltInTrunc(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resInteger:=Trunc(ArgToFloat(Args[0]));
end;

Procedure BuiltInInt(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=Int(ArgToFloat(Args[0]));
end;

Procedure BuiltInFrac(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resFloat:=frac(ArgToFloat(Args[0]));
end;

// String builtins

Procedure BuiltInLength(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resInteger:=Length(Args[0].resString);
end;

Procedure BuiltInCopy(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resString:=Copy(Args[0].resString,Args[1].resInteger,Args[2].resInteger);
end;

Procedure BuiltInDelete(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resString:=Args[0].resString;
  Delete(Result.resString,Args[1].resInteger,Args[2].resInteger);
end;

Procedure BuiltInPos(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resInteger:=Pos(Args[0].resString,Args[1].resString);
end;

Procedure BuiltInUppercase(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resString:=Uppercase(Args[0].resString);
end;

Procedure BuiltInLowercase(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resString:=Lowercase(Args[0].resString);
end;

Procedure BuiltInStringReplace(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

Var
  F : TReplaceFlags;

begin
  F:=[];
  If Args[3].resBoolean then
    Include(F,rfReplaceAll);
  If Args[4].resBoolean then
    Include(F,rfIgnoreCase);
  Result.resString:=StringReplace(Args[0].resString,Args[1].resString,Args[2].resString,f);
end;

Procedure BuiltInCompareText(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resInteger:=CompareText(Args[0].resString,Args[1].resString);
end;

// Date/Time builtins

Procedure BuiltInDate(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resDateTime:=Date;
end;

Procedure BuiltInTime(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resDateTime:=Time;
end;

Procedure BuiltInNow(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resDateTime:=Now;
end;

Procedure BuiltInDayofWeek(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  Result.resInteger:=DayOfWeek(Args[0].resDateTime);
end;

Procedure BuiltInExtractYear(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

Var
  Y,M,D : Word;

begin
  DecodeDate(Args[0].resDateTime,Y,M,D);
  Result.resInteger:=Y;
end;

Procedure BuiltInExtractMonth(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

Var
  Y,M,D : Word;

begin
  DecodeDate(Args[0].resDateTime,Y,M,D);
  Result.resInteger:=M;
end;

Procedure BuiltInExtractDay(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

Var
  Y,M,D : Word;

begin
  DecodeDate(Args[0].resDateTime,Y,M,D);
  Result.resInteger:=D;
end;

Procedure BuiltInExtractHour(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

Var
  H,M,S,MS : Word;

begin
  DecodeTime(Args[0].resDateTime,H,M,S,MS);
  Result.resInteger:=H;
end;

Procedure BuiltInExtractMin(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

Var
  H,M,S,MS : Word;

begin
  DecodeTime(Args[0].resDateTime,H,M,S,MS);
  Result.resInteger:=M;
end;

Procedure BuiltInExtractSec(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

Var
  H,M,S,MS : Word;

begin
  DecodeTime(Args[0].resDateTime,H,M,S,MS);
  Result.resInteger:=S;
end;

Procedure BuiltInExtractMSec(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

Var
  H,M,S,MS : Word;

begin
  DecodeTime(Args[0].resDateTime,H,M,S,MS);
  Result.resInteger:=MS;
end;

Procedure BuiltInEncodedate(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=Encodedate(Args[0].resInteger,Args[1].resInteger,Args[2].resInteger);
end;

Procedure BuiltInEncodeTime(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=EncodeTime(Args[0].resInteger,Args[1].resInteger,Args[2].resInteger,Args[3].resInteger);
end;

Procedure BuiltInEncodeDateTime(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=EncodeDate(Args[0].resInteger,Args[1].resInteger,Args[2].resInteger)
                     +EncodeTime(Args[3].resInteger,Args[4].resInteger,Args[5].resInteger,Args[6].resInteger);
end;

Procedure BuiltInShortDayName(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=DefaultFormatSettings.ShortDayNames[Args[0].resInteger];
end;

Procedure BuiltInShortMonthName(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=DefaultFormatSettings.ShortMonthNames[Args[0].resInteger];
end;
Procedure BuiltInLongDayName(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=DefaultFormatSettings.LongDayNames[Args[0].resInteger];
end;

Procedure BuiltInLongMonthName(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=DefaultFormatSettings.LongMonthNames[Args[0].resInteger];
end;

Procedure BuiltInFormatDateTime(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=FormatDateTime(Args[0].resString,Args[1].resDateTime);
end;


// Conversion
Procedure BuiltInIntToStr(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=IntToStr(Args[0].resinteger);
end;

Procedure BuiltInStrToInt(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resInteger:=StrToInt(Args[0].resString);
end;

Procedure BuiltInStrToIntDef(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resInteger:=StrToIntDef(Args[0].resString,Args[1].resInteger);
end;

Procedure BuiltInFloatToStr(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=FloatToStr(Args[0].resFloat);
end;

Procedure BuiltInStrToFloat(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resFloat:=StrToFloat(Args[0].resString);
end;

Procedure BuiltInStrToFloatDef(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resFloat:=StrToFloatDef(Args[0].resString,Args[1].resFloat);
end;

Procedure BuiltInDateToStr(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=DateToStr(Args[0].resDateTime);
end;

Procedure BuiltInTimeToStr(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=TimeToStr(Args[0].resDateTime);
end;

Procedure BuiltInStrToDate(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=StrToDate(Args[0].resString);
end;

Procedure BuiltInStrToDateDef(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=StrToDateDef(Args[0].resString,Args[1].resDateTime);
end;

Procedure BuiltInStrToTime(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=StrToTime(Args[0].resString);
end;

Procedure BuiltInStrToTimeDef(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=StrToTimeDef(Args[0].resString,Args[1].resDateTime);
end;

Procedure BuiltInStrToDateTime(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=StrToDateTime(Args[0].resString);
end;

Procedure BuiltInStrToDateTimeDef(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resDateTime:=StrToDateTimeDef(Args[0].resString,Args[1].resDateTime);
end;

procedure BuiltInFormatFloat(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
  result.ResString := FormatFloat(Args[0].resString, Args[1].ResFloat);
end;

Procedure BuiltInBoolToStr(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resString:=BoolToStr(Args[0].resBoolean);
end;

Procedure BuiltInStrToBool(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resBoolean:=StrToBool(Args[0].resString);
end;

Procedure BuiltInStrToBoolDef(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resBoolean:=StrToBoolDef(Args[0].resString,Args[1].resBoolean);
end;

// Boolean
Procedure BuiltInShl(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resInteger:=Args[0].resInteger shl Args[1].resInteger
end;

Procedure BuiltInShr(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  Result.resInteger:=Args[0].resInteger shr Args[1].resInteger
end;

Procedure BuiltinIFS(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  If Args[0].resBoolean then
    Result.resString:=Args[1].resString
  else
    Result.resString:=Args[2].resString
end;

Procedure BuiltinIFI(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  If Args[0].resBoolean then
    Result.resinteger:=Args[1].resinteger
  else
    Result.resinteger:=Args[2].resinteger
end;

Procedure BuiltinIFF(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  If Args[0].resBoolean then
    Result.resfloat:=Args[1].resfloat
  else
    Result.resfloat:=Args[2].resfloat
end;

Procedure BuiltinIFD(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
  If Args[0].resBoolean then
    Result.resDateTime:=Args[1].resDateTime
  else
    Result.resDateTime:=Args[2].resDateTime
end;

procedure RegisterStdBuiltins(AManager: TExprBuiltInManager;  Categories: TBuiltInCategories = AllBuiltIns);

begin
  With AManager do
    begin
    if bcMath in Categories then
      begin
      AddFloatVariable(bcMath,'pi',Pi);
      // Math functions
      AddFunction(bcMath,'cos','F','F',@BuiltinCos);
      AddFunction(bcMath,'sin','F','F',@BuiltinSin);
      AddFunction(bcMath,'arctan','F','F',@BuiltinArctan);
      AddFunction(bcMath,'abs','F','F',@BuiltinAbs);
      AddFunction(bcMath,'sqr','F','F',@BuiltinSqr);
      AddFunction(bcMath,'sqrt','F','F',@BuiltinSqrt);
      AddFunction(bcMath,'exp','F','F',@BuiltinExp);
      AddFunction(bcMath,'ln','F','F',@BuiltinLn);
      AddFunction(bcMath,'log','F','F',@BuiltinLog);
      AddFunction(bcMath,'frac','F','F',@BuiltinFrac);
      AddFunction(bcMath,'int','F','F',@BuiltinInt);
      AddFunction(bcMath,'round','I','F',@BuiltinRound);
      AddFunction(bcMath,'trunc','I','F',@BuiltinTrunc);
      end;
    if bcStrings in Categories then
      begin
      // String
      AddFunction(bcStrings,'length','I','S',@BuiltinLength);
      AddFunction(bcStrings,'copy','S','SII',@BuiltinCopy);
      AddFunction(bcStrings,'delete','S','SII',@BuiltinDelete);
      AddFunction(bcStrings,'pos','I','SS',@BuiltinPos);
      AddFunction(bcStrings,'lowercase','S','S',@BuiltinLowercase);
      AddFunction(bcStrings,'uppercase','S','S',@BuiltinUppercase);
      AddFunction(bcStrings,'stringreplace','S','SSSBB',@BuiltinStringReplace);
      AddFunction(bcStrings,'comparetext','I','SS',@BuiltinCompareText);
      end;
    if bcDateTime in Categories then
      begin
      // Date/Time
      AddFunction(bcDateTime,'date','D','',@BuiltinDate);
      AddFunction(bcDateTime,'time','D','',@BuiltinTime);
      AddFunction(bcDateTime,'now','D','',@BuiltinNow);
      AddFunction(bcDateTime,'dayofweek','I','D',@BuiltinDayofweek);
      AddFunction(bcDateTime,'extractyear','I','D',@BuiltinExtractYear);
      AddFunction(bcDateTime,'extractmonth','I','D',@BuiltinExtractMonth);
      AddFunction(bcDateTime,'extractday','I','D',@BuiltinExtractDay);
      AddFunction(bcDateTime,'extracthour','I','D',@BuiltinExtractHour);
      AddFunction(bcDateTime,'extractmin','I','D',@BuiltinExtractMin);
      AddFunction(bcDateTime,'extractsec','I','D',@BuiltinExtractSec);
      AddFunction(bcDateTime,'extractmsec','I','D',@BuiltinExtractMSec);
      AddFunction(bcDateTime,'encodedate','D','III',@BuiltinEncodedate);
      AddFunction(bcDateTime,'encodetime','D','IIII',@BuiltinEncodeTime);
      AddFunction(bcDateTime,'encodedatetime','D','IIIIIII',@BuiltinEncodeDateTime);
      AddFunction(bcDateTime,'shortdayname','S','I',@BuiltinShortDayName);
      AddFunction(bcDateTime,'shortmonthname','S','I',@BuiltinShortMonthName);
      AddFunction(bcDateTime,'longdayname','S','I',@BuiltinLongDayName);
      AddFunction(bcDateTime,'longmonthname','S','I',@BuiltinLongMonthName);
      end;
    if bcBoolean in Categories then
      begin
      // Boolean
      AddFunction(bcBoolean,'shl','I','II',@BuiltinShl);
      AddFunction(bcBoolean,'shr','I','II',@BuiltinShr);
      AddFunction(bcBoolean,'IFS','S','BSS',@BuiltinIFS);
      AddFunction(bcBoolean,'IFF','F','BFF',@BuiltinIFF);
      AddFunction(bcBoolean,'IFD','D','BDD',@BuiltinIFD);
      AddFunction(bcBoolean,'IFI','I','BII',@BuiltinIFI);
      end;
    if (bcConversion in Categories) then
      begin
      // Conversion
      AddFunction(bcConversion,'inttostr','S','I',@BuiltInIntToStr);
      AddFunction(bcConversion,'strtoint','I','S',@BuiltInStrToInt);
      AddFunction(bcConversion,'strtointdef','I','SI',@BuiltInStrToIntDef);
      AddFunction(bcConversion,'floattostr','S','F',@BuiltInFloatToStr);
      AddFunction(bcConversion,'strtofloat','F','S',@BuiltInStrToFloat);
      AddFunction(bcConversion,'strtofloatdef','F','SF',@BuiltInStrToFloatDef);
      AddFunction(bcConversion,'booltostr','S','B',@BuiltInBoolToStr);
      AddFunction(bcConversion,'strtobool','B','S',@BuiltInStrToBool);
      AddFunction(bcConversion,'strtobooldef','B','SB',@BuiltInStrToBoolDef);
      AddFunction(bcConversion,'datetostr','S','D',@BuiltInDateToStr);
      AddFunction(bcConversion,'timetostr','S','D',@BuiltInTimeToStr);
      AddFunction(bcConversion,'strtodate','D','S',@BuiltInStrToDate);
      AddFunction(bcConversion,'strtodatedef','D','SD',@BuiltInStrToDateDef);
      AddFunction(bcConversion,'strtotime','D','S',@BuiltInStrToTime);
      AddFunction(bcConversion,'strtotimedef','D','SD',@BuiltInStrToTimeDef);
      AddFunction(bcConversion,'strtodatetime','D','S',@BuiltInStrToDateTime);
      AddFunction(bcConversion,'strtodatetimedef','D','SD',@BuiltInStrToDateTimeDef);
      AddFunction(bcConversion,'formatfloat','S','SF',@BuiltInFormatFloat);
      AddFunction(bcConversion,'formatdatetime','S','SD',@BuiltinFormatDateTime);
      end;
    if bcAggregate in Categories then
      begin
      AddFunction(bcAggregate,'count','I','',TAggregateCount);
      AddFunction(bcAggregate,'sum','F','F',TAggregateSum);
      AddFunction(bcAggregate,'avg','F','F',TAggregateAvg);
      AddFunction(bcAggregate,'min','F','F',TAggregateMin);
      AddFunction(bcAggregate,'max','F','F',TAggregateMax);
      end;
    end;
end;

{ TFPBuiltInExprIdentifierDef }

procedure TFPBuiltInExprIdentifierDef.Assign(Source: TPersistent);
begin
  inherited Assign(Source);
  If Source is TFPBuiltInExprIdentifierDef then
    FCategory:=(Source as TFPBuiltInExprIdentifierDef).Category;
end;

procedure InitFileFormatSettings;
begin
  FileFormatSettings := DefaultFormatSettings;
  FileFormatSettings.DecimalSeparator := '.';
  FileFormatSettings.DateSeparator := '-';
  FileFormatSettings.TimeSeparator := ':';
  FileFormatsettings.ShortDateFormat := 'yyyy-mm-dd';
  FileFormatSettings.LongTimeFormat := 'hh:nn:ss';
end;

initialization
  RegisterStdBuiltins(BuiltinIdentifiers);
  InitFileFormatSettings;

finalization
  FreeBuiltins;
end.

