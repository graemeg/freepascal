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
                ttMul, ttLeft, ttRight, ttLessThanEqual, ttLargerThanEqual,
                ttunequal, ttNumber, ttString, ttIdentifier,
                ttComma, ttand, ttOr,ttXor,ttTrue,ttFalse,ttnot,ttif,
                ttCase,ttEOF);

  TExprFloat = Double;

Const
  ttDelimiters = [ttPlus, ttMinus, ttLessThan, ttLargerThan, ttEqual, ttDiv,
                  ttMul, ttLeft, ttRight, ttLessThanEqual, ttLargerThanEqual,
                  ttunequal];
  ttComparisons = [ttLargerThan,ttLessthan,
                   ttLargerThanEqual,ttLessthanEqual,
                   ttEqual,ttUnequal];

Type

  TFPExpressionParser = Class;
  TExprBuiltInManager = Class;

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
    function DoNumber: TTokenType;
    function DoDelimiter: TTokenType;
    function DoString: TTokenType;
    Function NextPos : Char; // inline;
    procedure SkipWhiteSpace; // inline;
    function IsWordDelim(C : Char) : Boolean; // inline;
    function IsDelim(C : Char) : Boolean; // inline;
    function IsDigit(C : Char) : Boolean; // inline;
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

  TResultType = (rtBoolean,rtInteger,rtFloat,rtDateTime,rtString);
  TResultTypes = set of TResultType;

  TFPExpressionResult = record
    ResString   : String;
    Case ResultType : TResultType of
      rtBoolean  : (ResBoolean  : Boolean);
      rtInteger  : (ResInteger  : Int64);
      rtFloat    : (ResFloat    : TExprFloat);
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
<<<<<<< HEAD
<<<<<<< HEAD
  Public
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Procedure Check; override;
    Function NodeType : TResultType; override;
=======
    Procedure Check; override;
    Function NodeType : TResultType; override;
  Public
>>>>>>> graemeg/fixes_2_2
=======
    Procedure Check; override;
    Function NodeType : TResultType; override;
  Public
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Procedure Check; override;
    Function NodeType : TResultType; override;
=======
    Procedure Check; override;
    Function NodeType : TResultType; override;
  Public
>>>>>>> graemeg/fixes_2_2
=======
    Procedure Check; override;
    Function NodeType : TResultType; override;
  Public
>>>>>>> origin/fixes_2_2
    Constructor Create(Args : TExprArgumentArray);
    Destructor destroy; override;
    Function AsString : string ; override;
    Property Condition : TFPExprNode Read FCondition;
  end;

  { TMathOperation }

  TMathOperation = Class(TFPBinaryOperation)
<<<<<<< HEAD
<<<<<<< HEAD
  Public
=======
  protected
>>>>>>> graemeg/fixes_2_2
=======
  protected
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Procedure Check; override;
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
=======
=======
>>>>>>> origin/fixes_2_2
  Protected
    Procedure check; override;
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  Public
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    Function AsString : string ; override;
  end;

  { TFPMultiplyOperation }

  TFPMultiplyOperation = Class(TMathOperation)
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Procedure check; override;
=======
  Protected
    Procedure check; override;
  Public
>>>>>>> graemeg/fixes_2_2
=======
  Protected
    Procedure check; override;
  Public
>>>>>>> origin/fixes_2_2
    Function AsString : string ; override;
    Procedure GetNodeValue(var Result : TFPExpressionResult); override;
  end;

  { TFPDivideOperation }

  TFPDivideOperation = Class(TMathOperation)
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Procedure Check; override;
=======
  Protected
    Procedure check; override;
  Public
>>>>>>> graemeg/fixes_2_2
=======
  Protected
    Procedure check; override;
  Public
>>>>>>> origin/fixes_2_2
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
    Procedure Check; override;
    Property Operand : TFPExprNode Read FOperand;
  end;

  { TFPConvertNode }

  TFPConvertNode = Class(TFPUnaryOperator)
    Function AsString : String; override;
  end;

  { TFPNotNode }

  TFPNotNode = Class(TFPUnaryOperator)
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Procedure Check; override;
=======
  Protected
    Procedure Check; override;
  Public
>>>>>>> graemeg/fixes_2_2
=======
  Protected
    Procedure Check; override;
  Public
>>>>>>> origin/fixes_2_2
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
    Function AsString : String; override;
  end;

  TIntConvertNode = Class(TFPConvertNode)
<<<<<<< HEAD
<<<<<<< HEAD
  Public
=======
  Protected
>>>>>>> graemeg/fixes_2_2
=======
  Protected
>>>>>>> origin/fixes_2_2
    Procedure Check; override;
  end;

  { TIntToFloatNode }
  TIntToFloatNode = Class(TIntConvertNode)
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
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Procedure Check; override;
=======
  Protected
    Procedure Check; override;
  Public
>>>>>>> graemeg/fixes_2_2
=======
  Protected
    Procedure Check; override;
  Public
>>>>>>> origin/fixes_2_2
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
    Procedure Check; override;
    Function NodeType : TResultType;  override;
    Procedure GetNodeValue(var Result : TFPExpressionResult);  override;
    Function AsString : string ; override;
   // For inspection
    Property ConstValue : TFPExpressionResult read FValue;
  end;


  TIdentifierType = (itVariable,itFunctionCallBack,itFunctionHandler);
  TFPExprFunctionCallBack = Procedure (Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
  TFPExprFunctionEvent = Procedure (Var Result : TFPExpressionResult; Const Args : TExprParameterArray) of object;

  { TFPExprIdentifierDef }

  TFPExprIdentifierDef = Class(TCollectionItem)
  private
    FStringValue : String;
    FValue : TFPExpressionResult;
    FArgumentTypes: String;
    FIDType: TIdentifierType;
    FName: ShortString;
    FOnGetValue: TFPExprFunctionEvent;
    FOnGetValueCB: TFPExprFunctionCallBack;
    function GetAsBoolean: Boolean;
    function GetAsDateTime: TDateTime;
    function GetAsFloat: TExprFloat;
    function GetAsInteger: Int64;
    function GetAsString: String;
    function GetResultType: TResultType;
    function GetValue: String;
    procedure SetArgumentTypes(const AValue: String);
    procedure SetAsBoolean(const AValue: Boolean);
    procedure SetAsDateTime(const AValue: TDateTime);
    procedure SetAsFloat(const AValue: TExprFloat);
    procedure SetAsInteger(const AValue: Int64);
    procedure SetAsString(const AValue: String);
    procedure SetName(const AValue: ShortString);
    procedure SetResultType(const AValue: TResultType);
    procedure SetValue(const AValue: String);
  Protected
    Procedure CheckResultType(Const AType : TResultType);
    Procedure CheckVariable;
  Public
    Function ArgumentCount : Integer;
    Procedure Assign(Source : TPersistent); override;
    Property AsFloat : TExprFloat Read GetAsFloat Write SetAsFloat;
    Property AsInteger : Int64 Read GetAsInteger Write SetAsInteger;
    Property AsString : String Read GetAsString Write SetAsString;
    Property AsBoolean : Boolean Read GetAsBoolean Write SetAsBoolean;
    Property AsDateTime : TDateTime Read GetAsDateTime Write SetAsDateTime;
    Property OnGetFunctionValueCallBack : TFPExprFunctionCallBack Read FOnGetValueCB Write FOnGetValueCB;
  Published
    Property IdentifierType : TIdentifierType Read FIDType Write FIDType;
    Property Name : ShortString Read FName Write SetName;
    Property Value : String Read GetValue Write SetValue;
    Property ParameterTypes : String Read FArgumentTypes Write SetArgumentTypes;
    Property ResultType : TResultType Read GetResultType Write SetResultType;
    Property OnGetFunctionValue : TFPExprFunctionEvent Read FOnGetValue Write FOnGetValue;
  end;


  TBuiltInCategory = (bcStrings,bcDateTime,bcMath,bcBoolean,bcConversion,bcData,bcVaria,bcUser);
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
    Function AddVariable(Const AName : ShortString; AResultType : TResultType; AValue : String) : TFPExprIdentifierDef;
    Function AddBooleanVariable(Const AName : ShortString; AValue : Boolean) : TFPExprIdentifierDef;
    Function AddIntegerVariable(Const AName : ShortString; AValue : Integer) : TFPExprIdentifierDef;
    Function AddFloatVariable(Const AName : ShortString; AValue : TExprFloat) : TFPExprIdentifierDef;
    Function AddStringVariable(Const AName : ShortString; AValue : String) : TFPExprIdentifierDef;
    Function AddDateTimeVariable(Const AName : ShortString; AValue : TDateTime) : TFPExprIdentifierDef;
    Function AddFunction(Const AName : ShortString; Const AResultType : Char; Const AParamTypes : String; ACallBack : TFPExprFunctionCallBack) : TFPExprIdentifierDef;
    Function AddFunction(Const AName : ShortString; Const AResultType : Char; Const AParamTypes : String; ACallBack : TFPExprFunctionEvent) : TFPExprIdentifierDef;
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
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Procedure Check; override;
=======
    Procedure Check; override;
  Public
>>>>>>> graemeg/fixes_2_2
=======
    Procedure Check; override;
  Public
>>>>>>> origin/fixes_2_2
    Constructor CreateFunction(AID : TFPExprIdentifierDef; Const Args : TExprArgumentArray); virtual;
    Destructor Destroy; override;
    Property ArgumentNodes : TExprArgumentArray Read FArgumentNodes;
    Property ArgumentParams : TExprParameterArray Read FArgumentParams;
    Function AsString : String; override;
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
    function ConvertNode(Todo: TFPExprNode; ToType: TResultType): TFPExprNode;
    function GetAsBoolean: Boolean;
    function GetAsDateTime: TDateTime;
    function GetAsFloat: TExprFloat;
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
    class Function BuiltinsManager : TExprBuiltInManager;
    Function Level1 : TFPExprNode;
    Function Level2 : TFPExprNode;
    Function Level3 : TFPExprNode;
    Function Level4 : TFPExprNode;
    Function Level5 : TFPExprNode;
    Function Level6 : TFPExprNode;
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
<<<<<<< HEAD
<<<<<<< HEAD
    Function IdentifierByName(const AName : ShortString) : TFPExprIdentifierDef; virtual;
=======
    Function IdentifierByName(AName : ShortString) : TFPExprIdentifierDef;
>>>>>>> graemeg/fixes_2_2
=======
    Function IdentifierByName(AName : ShortString) : TFPExprIdentifierDef;
>>>>>>> origin/fixes_2_2
    Procedure Clear;
    Procedure EvaluateExpression(Var Result : TFPExpressionResult);
    Function Evaluate : TFPExpressionResult;
    Function ResultType : TResultType;
    Property AsFloat : TExprFloat Read GetAsFloat;
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
    Function AddStringVariable(Const ACategory : TBuiltInCategory; Const AName : ShortString; AValue : String) : TFPBuiltInExprIdentifierDef;
    Function AddDateTimeVariable(Const ACategory : TBuiltInCategory; Const AName : ShortString; AValue : TDateTime) : TFPBuiltInExprIdentifierDef;
    Function AddFunction(Const ACategory : TBuiltInCategory; Const AName : ShortString; Const AResultType : Char; Const AParamTypes : String; ACallBack : TFPExprFunctionCallBack) : TFPBuiltInExprIdentifierDef;
    Function AddFunction(Const ACategory : TBuiltInCategory; Const AName : ShortString; Const AResultType : Char; Const AParamTypes : String; ACallBack : TFPExprFunctionEvent) : TFPBuiltInExprIdentifierDef;
    Property IdentifierCount : Integer Read GetCount;
    Property Identifiers[AIndex : Integer] :TFPBuiltInExprIdentifierDef Read GetI;
  end;

  EExprParser = Class(Exception);


Function TokenName (AToken : TTokenType) : String;
Function ResultTypeName (AResult : TResultType) : String;
Function CharToResultType(C : Char) : TResultType;
Function BuiltinIdentifiers : TExprBuiltInManager;
Procedure RegisterStdBuiltins(AManager : TExprBuiltInManager);
<<<<<<< HEAD
<<<<<<< HEAD
function ArgToFloat(Arg: TFPExpressionResult): TExprFloat;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Const
  AllBuiltIns = [bcStrings,bcDateTime,bcMath,bcBoolean,bcConversion,bcData,bcVaria,bcUser];


implementation

uses typinfo;

{ TFPExpressionParser }

const
  cNull=#0;
  cSingleQuote = '''';

  Digits        = ['0'..'9','.'];
  WhiteSpace    = [' ',#13,#10,#9];
  Operators     = ['+','-','<','>','=','/','*'];
  Delimiters    = Operators+[',','(',')'];
  Symbols       = ['%','^']+Delimiters;
  WordDelimiters = WhiteSpace + Symbols;

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
  SErrNoOperand = 'No operand for unary operation %s';
  SErrNoleftOperand = 'No left operand for binary operation %s';
<<<<<<< HEAD
<<<<<<< HEAD
  SErrNoRightOperand = 'No right operand for binary operation %s';
=======
  SErrNoRightOperand = 'No left operand for binary operation %s';
>>>>>>> graemeg/fixes_2_2
=======
  SErrNoRightOperand = 'No left operand for binary operation %s';
>>>>>>> origin/fixes_2_2
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

Function TokenName (AToken : TTokenType) : String;

begin
  Result:=GetEnumName(TypeInfo(TTokenType),Ord(AToken));
end;

Function ResultTypeName (AResult : TResultType) : String;

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
  else
    RaiseParserError(SErrInvalidResultCharacter,[C]);
  end;
end;

Var
  BuiltIns : TExprBuiltInManager;

Function BuiltinIdentifiers : TExprBuiltInManager;

begin
  If (BuiltIns=Nil) then
    BuiltIns:=TExprBuiltInManager.Create(Nil);
  Result:=BuiltIns;
end;

Procedure FreeBuiltIns;

begin
  FreeAndNil(Builtins);
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

function TFPExpressionScanner.IsDigit(C: Char): Boolean;
begin
  Result:=C in Digits;
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

Function TFPExpressionScanner.DoNumber : TTokenType;

Var
  C : Char;
  X : TExprFloat;
  I : Integer;
<<<<<<< HEAD
<<<<<<< HEAD
  prevC: Char;

begin
  C:=CurrentChar;
  prevC := #0;
  while (not IsWordDelim(C) or (prevC='E')) and (C<>cNull) do
    begin
    If Not ( IsDigit(C)
             or ((FToken<>'') and (Upcase(C)='E'))
             or ((FToken<>'') and (C in ['+','-']) and (prevC='E'))
           )
    then
      ScanError(Format(SErrInvalidNumberChar,[C]));
    FToken := FToken+C;
    prevC := Upcase(C);
=======
=======
>>>>>>> origin/fixes_2_2

begin
  C:=CurrentChar;
  while (not IsWordDelim(C)) and (C<>cNull) do
    begin
    If Not (IsDigit(C) or ((FToken<>'') and (Upcase(C)='E'))) then
      ScanError(Format(SErrInvalidNumberChar,[C]));
    FToken := FToken+C;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
    FToken:=FToken+C;
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
  else if IsDigit(C) then
    Result:=DoNumber
  else if IsAlpha(C) then
    Result:=DoIdentifier
  else
    ScanError(Format(SErrUnknownCharacter,[FPos,C]))  ;
  FTokenType:=Result;
end;

{ ---------------------------------------------------------------------
  TFPExpressionParser
  ---------------------------------------------------------------------}

Function TFPExpressionParser.TokenType : TTokenType;

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

<<<<<<< HEAD
<<<<<<< HEAD
function TFPExpressionParser.IdentifierByName(const AName: ShortString): TFPExprIdentifierDef;
=======
function TFPExpressionParser.IdentifierByName(AName: ShortString): TFPExprIdentifierDef;
>>>>>>> graemeg/fixes_2_2
=======
function TFPExpressionParser.IdentifierByName(AName: ShortString): TFPExprIdentifierDef;
>>>>>>> origin/fixes_2_2
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

Function TFPExpressionParser.GetToken : TTokenType;

begin
  Result:=FScanner.GetToken;
end;

Procedure TFPExpressionParser.CheckEOF;

begin
  If (TokenType=ttEOF) then
    ParserError(SErrUnexpectedEndOfExpression);
end;

procedure TFPExpressionParser.SetIdentifiers(const AValue: TFPExprIdentifierDefs
  );
begin
  FIdentifiers.Assign(AValue)
end;

procedure TFPExpressionParser.EvaluateExpression(var Result: TFPExpressionResult);
begin
  If (FExpression='') then
    ParserError(SErrInExpressionEmpty);
  if not Assigned(FExprNode) then
    ParserError(SErrInExpression);
  FExprNode.GetNodeValue(Result);
end;

procedure TFPExpressionParser.ParserError(Msg: String);
begin
  Raise EExprParser.Create(Msg);
end;

function TFPExpressionParser.ConvertNode(Todo : TFPExprNode; ToType : TResultType): TFPExprNode;


begin
  Result:=ToDo;
  Case ToDo.NodeType of
    rtInteger :
      Case ToType of
        rtFloat    : Result:=TIntToFloatNode.Create(Result);
        rtDateTime : Result:=TIntToDateTimeNode.Create(Result);
      end;
    rtFloat :
      Case ToType of
        rtDateTime : Result:=TFloatToDateTimeNode.Create(Result);
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
  CheckResultType(Res,rtFloat);
  Result:=Res.ResFloat;
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
  TT,MT : TResultType;

begin
  Result:=Todo;
  TT:=Todo.NodeType;
  MT:=Match.NodeType;
  If (TT<>MT) then
    begin
    if (TT=rtInteger) then
      begin
      if (MT in [rtFloat,rtDateTime]) then
        Result:=ConvertNode(Todo,MT);
      end
    else if (TT=rtFloat) then
      begin
      if (MT=rtDateTime) then
        Result:=ConvertNode(Todo,rtDateTime);
      end;
    end;
end;

{
  if the result types differ, they are converted to a common type if possible.
}

Procedure TFPExpressionParser.CheckNodes(Var Left,Right : TFPExprNode);

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

Function TFPExpressionParser.Level1 : TFPExprNode;

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
    while (TokenType in [ttMul,ttDiv]) do
      begin
      tt:=TokenType;
      GetToken;
      Right:=Level5;
      CheckNodes(Result,Right);
      Case tt of
        ttMul : Result:=TFPMultiplyOperation.Create(Result,Right);
        ttDiv : Result:=TFPDivideOperation.Create(Result,Right);
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
begin
{$ifdef debugexpr}  Writeln('Level 6 ',TokenName(TokenType),': ',CurrentToken);{$endif debugexpr}
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
      If (I=0) then
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
    else if (ID.IdentifierType in [itFunctionCallBack,itFunctionHandler]) then
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
  Result:=FExprNode.NodeType;;
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

function TFPExprIdentifierDefs.AddVariable(Const AName: ShortString;
  AResultType: TResultType; AValue: String): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=AResultType;
  Result.Value:=AValue;
end;

function TFPExprIdentifierDefs.AddBooleanVariable(Const AName: ShortString; AValue: Boolean
  ): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=rtBoolean;
  Result.FValue.ResBoolean:=AValue;
end;

function TFPExprIdentifierDefs.AddIntegerVariable(Const AName: ShortString; AValue: Integer
  ): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=rtInteger;
  Result.FValue.ResInteger:=AValue;
end;

function TFPExprIdentifierDefs.AddFloatVariable(Const AName: ShortString; AValue: TExprFloat
  ): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=rtFloat;
  Result.FValue.ResFloat:=AValue;
end;

function TFPExprIdentifierDefs.AddStringVariable(Const AName: ShortString; AValue: String
  ): TFPExprIdentifierDef;
begin
  Result:=Add as TFPExprIdentifierDef;
  Result.IdentifierType:=itVariable;
  Result.Name:=AName;
  Result.ResultType:=rtString;
  Result.FValue.ResString:=AValue;
end;

function TFPExprIdentifierDefs.AddDateTimeVariable(Const AName: ShortString; AValue: TDateTime
  ): TFPExprIdentifierDef;
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
  Result.ParameterTypes:=AParamTypes;
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
  Result.ParameterTypes:=AParamTypes;
  Result.ResultType:=CharToResultType(AResultType);
  Result.FOnGetValue:=ACallBack;
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
      rtFloat    : FValue.ResFloat:=StrToFloat(AValue);
      rtDateTime : FValue.ResDateTime:=StrToDateTime(AValue);
      rtString   : FValue.ResString:=AValue;
    end
  else
    Case FValue.ResultType of
      rtBoolean  : FValue.ResBoolean:=False;
      rtInteger  : FValue.ResInteger:=0;
      rtFloat    : FValue.ResFloat:=0.0;
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
end;

function TFPExprIdentifierDef.ArgumentCount: Integer;
begin
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
    FIDType:=EID.FIDType;
    FName:=EID.FName;
    FOnGetValue:=EID.FOnGetValue;
    FOnGetValueCB:=EID.FOnGetValueCB;
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
    rtFloat    : Result:=FloatToStr(FValue.ResFloat);
    rtDateTime : Result:=FormatDateTime('cccc',FValue.ResDateTime);
    rtString   : Result:=FValue.ResString;
  end;
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
  end;
end;


{ TFPNegateOperation }

procedure TFPNegateOperation.Check;
begin
  Inherited;
  If Not (Operand.NodeType in [rtInteger,rtFloat]) then
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
      rtFloat    : B:=RT.ResFloat=RV.ResFLoat;
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
    rtFloat    : Result.resBoolean:=Result.ResFloat=RRes.ResFLoat;
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
    rtFloat    : Result.resBoolean:=Result.ResFloat<RRes.ResFLoat;
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
                end;
    rtFloat   : case Right.NodeType of
                  rtInteger : Result.resBoolean:=Result.ResFloat>RRes.ResInteger;
                  rtFloat : Result.resBoolean:=Result.ResFloat>RRes.ResFLoat;
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
  AllowedTypes =[rtInteger,rtfloat,rtDateTime,rtString];

begin
  CheckNodeType(Left,AllowedTypes);
  CheckNodeType(Right,AllowedTypes);
  inherited Check;
end;

{ TMathOperation }

procedure TMathOperation.Check;

Const
  AllowedTypes =[rtInteger,rtfloat,rtDateTime,rtString];

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
    rtFloat    : Result.ResFLoat:=Result.ResFLoat+RRes.ResFLoat;
  end;
  Result.ResultType:=NodeType;
end;

{ TFPSubtractOperation }

procedure TFPSubtractOperation.check;

Const
  AllowedTypes =[rtInteger,rtfloat,rtDateTime];

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
    rtFloat    : Result.ResFLoat:=Result.ResFLoat-RRes.ResFLoat;
  end;
end;

{ TFPMultiplyOperation }

procedure TFPMultiplyOperation.check;

Const
  AllowedTypes =[rtInteger,rtfloat];

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
    rtFloat    : Result.ResFLoat:=Result.ResFLoat*RRes.ResFLoat;
  end;
end;

{ TFPDivideOperation }

procedure TFPDivideOperation.check;
Const
  AllowedTypes =[rtInteger,rtfloat];

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
  Result:=rtFLoat;
end;

Procedure TFPDivideOperation.GetNodeValue(var Result : TFPExpressionResult);

Var
  RRes : TFPExpressionResult;

begin
  Left.GetNodeValue(Result);
  Right.GetNodeValue(RRes);
  case Result.ResultType of
    rtInteger  : Result.ResFloat:=Result.ResInteger/RRes.ResInteger;
    rtFloat    : Result.ResFLoat:=Result.ResFLoat/RRes.ResFLoat;
  end;
  Result.ResultType:=rtFloat;
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
    FArgumentNodes[i].GetNodeValue(FArgumentParams[i]);
end;

procedure TFPExprFunction.Check;

Var
  I : Integer;
  rtp,rta : TResultType;

begin
  If Length(FArgumentNodes)<>FID.ArgumentCount then
    RaiseParserError(ErrInvalidArgumentCount,[FID.Name]);
  For I:=0 to Length(FArgumentNodes)-1 do
    begin
    rtp:=CharToResultType(FID.ParameterTypes[i+1]);
    rta:=FArgumentNodes[i].NodeType;
<<<<<<< HEAD
<<<<<<< HEAD
    If (rtp<>rta) then begin

      // Automatically convert integers to floats in functions that return
      // a float
      if (rta = rtInteger) and (rtp = rtFloat) then begin
        FArgumentNodes[i] := TIntToFloatNode(FArgumentNodes[i]);
        exit;
      end;

      RaiseParserError(SErrInvalidArgumentType,[I+1,ResultTypeName(rtp),ResultTypeName(rta)])
    end;
    end;
=======
    If (rtp<>rta) then
      RaiseParserError(SErrInvalidArgumentType,[I+1,ResultTypeName(rtp),ResultTypeName(rta)])
    end;
>>>>>>> graemeg/fixes_2_2
=======
    If (rtp<>rta) then
      RaiseParserError(SErrInvalidArgumentType,[I+1,ResultTypeName(rtp),ResultTypeName(rta)])
    end;
>>>>>>> origin/fixes_2_2
end;

constructor TFPExprFunction.CreateFunction(AID: TFPExprIdentifierDef;
  const Args: TExprArgumentArray);
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

<<<<<<< HEAD
<<<<<<< HEAD
function ArgToFloat(Arg: TFPExpressionResult): TExprFloat;
// Utility function for the built-in math functions. Accepts also integers
// in place of the floating point arguments. To be called in builtins or
// user-defined callbacks having float results.
begin
  if Arg.ResultType = rtInteger then
    result := Arg.resInteger
  else
    result := Arg.resFloat;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
// Math builtins

Procedure BuiltInCos(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=Cos(ArgToFloat(Args[0]));
=======
  Result.resFloat:=Cos(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=Cos(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInSin(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=Sin(ArgToFloat(Args[0]));
=======
  Result.resFloat:=Sin(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=Sin(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInArcTan(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=Arctan(ArgToFloat(Args[0]));
=======
  Result.resFloat:=Arctan(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=Arctan(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInAbs(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=Abs(ArgToFloat(Args[0]));
=======
  Result.resFloat:=Abs(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=Abs(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInSqr(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=Sqr(ArgToFloat(Args[0]));
=======
  Result.resFloat:=Sqr(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=Sqr(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInSqrt(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=Sqrt(ArgToFloat(Args[0]));
=======
  Result.resFloat:=Sqrt(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=Sqrt(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInExp(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=Exp(ArgToFloat(Args[0]));
=======
  Result.resFloat:=Exp(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=Exp(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInLn(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=Ln(ArgToFloat(Args[0]));
=======
  Result.resFloat:=Ln(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=Ln(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Const
  L10 = ln(10);

Procedure BuiltInLog(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=Ln(ArgToFloat(Args[0]))/L10;
=======
  Result.resFloat:=Ln(Args[0].resFloat)/L10;
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=Ln(Args[0].resFloat)/L10;
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInRound(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resInteger:=Round(ArgToFloat(Args[0]));
=======
  Result.resInteger:=Round(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resInteger:=Round(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInTrunc(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resInteger:=Trunc(ArgToFloat(Args[0]));
=======
  Result.resInteger:=Trunc(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resInteger:=Trunc(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInInt(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=Int(ArgToFloat(Args[0]));
=======
  Result.resFloat:=Int(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=Int(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInFrac(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resFloat:=frac(ArgToFloat(Args[0]));
=======
  Result.resFloat:=frac(Args[0].resFloat);
>>>>>>> graemeg/fixes_2_2
=======
  Result.resFloat:=frac(Args[0].resFloat);
>>>>>>> origin/fixes_2_2
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
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resString:=DefaultFormatSettings.ShortDayNames[Args[0].resInteger];
=======
  Result.resString:=ShortDayNames[Args[0].resInteger];
>>>>>>> graemeg/fixes_2_2
=======
  Result.resString:=ShortDayNames[Args[0].resInteger];
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInShortMonthName(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resString:=DefaultFormatSettings.ShortMonthNames[Args[0].resInteger];
=======
  Result.resString:=ShortMonthNames[Args[0].resInteger];
>>>>>>> graemeg/fixes_2_2
=======
  Result.resString:=ShortMonthNames[Args[0].resInteger];
>>>>>>> origin/fixes_2_2
end;
Procedure BuiltInLongDayName(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resString:=DefaultFormatSettings.LongDayNames[Args[0].resInteger];
=======
  Result.resString:=LongDayNames[Args[0].resInteger];
>>>>>>> graemeg/fixes_2_2
=======
  Result.resString:=LongDayNames[Args[0].resInteger];
>>>>>>> origin/fixes_2_2
end;

Procedure BuiltInLongMonthName(Var Result : TFPExpressionResult; Const Args : TExprParameterArray);

begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result.resString:=DefaultFormatSettings.LongMonthNames[Args[0].resInteger];
=======
  Result.resString:=LongMonthNames[Args[0].resInteger];
>>>>>>> graemeg/fixes_2_2
=======
  Result.resString:=LongMonthNames[Args[0].resInteger];
>>>>>>> origin/fixes_2_2
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

Procedure RegisterStdBuiltins(AManager : TExprBuiltInManager);

begin
  With AManager do
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
    // String
    AddFunction(bcStrings,'length','I','S',@BuiltinLength);
    AddFunction(bcStrings,'copy','S','SII',@BuiltinCopy);
    AddFunction(bcStrings,'delete','S','SII',@BuiltinDelete);
    AddFunction(bcStrings,'pos','I','SS',@BuiltinPos);
    AddFunction(bcStrings,'lowercase','S','S',@BuiltinLowercase);
    AddFunction(bcStrings,'uppercase','S','S',@BuiltinUppercase);
    AddFunction(bcStrings,'stringreplace','S','SSSBB',@BuiltinStringReplace);
    AddFunction(bcStrings,'comparetext','I','SS',@BuiltinCompareText);
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
    AddFunction(bcDateTime,'formatdatetime','S','SD',@BuiltinFormatDateTime);
    // Boolean
    AddFunction(bcBoolean,'shl','I','II',@BuiltinShl);
    AddFunction(bcBoolean,'shr','I','II',@BuiltinShr);
    AddFunction(bcBoolean,'IFS','S','BSS',@BuiltinIFS);
    AddFunction(bcBoolean,'IFF','F','BFF',@BuiltinIFF);
    AddFunction(bcBoolean,'IFD','D','BDD',@BuiltinIFD);
    AddFunction(bcBoolean,'IFI','I','BII',@BuiltinIFI);
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
    end;
end;

{ TFPBuiltInExprIdentifierDef }

procedure TFPBuiltInExprIdentifierDef.Assign(Source: TPersistent);
begin
  inherited Assign(Source);
  If Source is TFPBuiltInExprIdentifierDef then
    FCategory:=(Source as TFPBuiltInExprIdentifierDef).Category;
end;

initialization
  RegisterStdBuiltins(BuiltinIdentifiers);

finalization
  FreeBuiltins;
end.
