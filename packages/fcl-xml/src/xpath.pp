{
    This file is part of the Free Component Library

    Implementation of the XML Path Language (XPath) for Free Pascal
    Copyright (c) 2000 - 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$MODE objfpc}
{$H+}

unit XPath;

interface

uses SysUtils, Classes, DOM;

resourcestring
  { XPath variables type names }
  SNodeSet = 'node set';
  SBoolean = 'boolean';
  SNumber = 'number';
  SString = 'string';

  { Variable errors }
  SVarNoConversion = 'Conversion from %s to %s not possible';

  { Scanner errors }
  SScannerUnclosedString = 'String literal was not closed';
  SScannerInvalidChar = 'Invalid character';
  SScannerMalformedQName = 'Expected "*" or local part after colon';
  SScannerExpectedVarName = 'Expected variable name after "$"';

  { Parser errors }
  SParserExpectedLeftBracket = 'Expected "("';
  SParserExpectedRightBracket = 'Expected ")"';
  SParserBadAxisName = 'Invalid axis name';
  SParserBadNodeType = 'Invalid node type';
  SParserExpectedRightSquareBracket = 'Expected "]" after predicate';
  SParserInvalidPrimExpr = 'Invalid primary expression';
  SParserGarbageAfterExpression = 'Unrecognized input after expression';
  SParserInvalidNodeTest = 'Invalid node test (syntax error)';


  { Evaluation errors }
  SEvalUnknownFunction = 'Unknown function: "%s"';
  SEvalUnknownVariable = 'Unknown variable: "%s"';
  SEvalInvalidArgCount = 'Invalid number of function arguments';

type

  TXPathContext = class;
  TXPathEnvironment = class;
  TXPathVariable = class;


{ XPath lexical scanner }

  TXPathToken = (               // [28] - [38]
    tkInvalid,
    tkEndOfStream,
    tkIdentifier,
    tkNSNameTest,               // NCName:*
    tkString,
    tkNumber,
    tkVariable,                 // $QName
    tkLeftBracket,              // "("
    tkRightBracket,             // ")"
    tkAsterisk,                 // "*"
    tkPlus,                     // "+"
    tkComma,                    // ","
    tkMinus,                    // "-"
    tkDot,                      // "."
    tkDotDot,                   // ".."
    tkSlash,                    // "/"
    tkSlashSlash,               // "//"
    tkColonColon,               // "::"
    tkLess,                     // "<"
    tkLessEqual,                // "<="
    tkEqual,                    // "="
    tkNotEqual,                 // "!="
    tkGreater,                  // ">"
    tkGreaterEqual,             // ">="
    tkAt,                       // "@"
    tkLeftSquareBracket,        // "["
    tkRightSquareBracket,       // "]"
    tkPipe                      // "|"
  );

  TXPathKeyword = (
    // axis names
    xkNone, xkAncestor,  xkAncestorOrSelf,  xkAttribute,  xkChild,
    xkDescendant, xkDescendantOrSelf, xkFollowing, xkFollowingSibling,
    xkNamespace, xkParent, xkPreceding, xkPrecedingSibling, xkSelf,
    // node tests
    xkComment, xkText, xkProcessingInstruction, xkNode,
    // operators
    xkAnd, xkOr, xkDiv, xkMod,
    // standard functions
    xkLast, xkPosition, xkCount, xkId, xkLocalName, xkNamespaceUri,
    xkName, xkString, xkConcat, xkStartsWith, xkContains,
    xkSubstringBefore, xkSubstringAfter, xkSubstring,
    xkStringLength, xkNormalizeSpace, xkTranslate, xkBoolean,
    xkNot, xkTrue, xkFalse, xkLang, xkNumber, xkSum, xkFloor,
    xkCeiling, xkRound
  );

{ XPath expression parse tree }

  TXPathExprNode = class
  protected
    function EvalPredicate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): Boolean;
  public
    function Evaluate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): TXPathVariable; virtual; abstract;
  end;

  TXPathNodeArray = array of TXPathExprNode;

  TXPathConstantNode = class(TXPathExprNode)
  private
    FValue: TXPathVariable;
  public
    constructor Create(AValue: TXPathVariable);
    destructor Destroy; override;
    function Evaluate(AContext: TXPathContext;
       AEnvironment: TXPathEnvironment): TXPathVariable; override;
  end;


  TXPathVariableNode = class(TXPathExprNode)
  private
    FName: DOMString;
  public
    constructor Create(const AName: DOMString);
    function Evaluate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): TXPathVariable; override;
  end;


  TXPathFunctionNode = class(TXPathExprNode)
  private
    FName: DOMString;
    FArgs: TXPathNodeArray;
  public
    constructor Create(const AName: DOMString; const Args: TXPathNodeArray);
    destructor Destroy; override;
    function Evaluate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): TXPathVariable; override;
  end;


  TXPathNegationNode = class(TXPathExprNode)
  private
    FOperand: TXPathExprNode;
  public
    constructor Create(AOperand: TXPathExprNode);
    destructor Destroy; override;
    function Evaluate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): TXPathVariable; override;
  end;

  // common ancestor for binary operations

  TXPathBinaryNode = class(TXPathExprNode)
  protected
    FOperand1, FOperand2: TXPathExprNode;
  public
    destructor Destroy; override;   
  end;

  // Node for (binary) mathematical operation

  TXPathMathOp = (opAdd, opSubtract, opMultiply, opDivide, opMod);

  TXPathMathOpNode = class(TXPathBinaryNode)
  private
    FOperator: TXPathMathOp;
  public
    constructor Create(AOperator: TXPathMathOp;
      AOperand1, AOperand2: TXPathExprNode);
    function Evaluate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): TXPathVariable; override;
  end;

  // Node for comparison operations

  TXPathCompareOp = (opEqual, opNotEqual, opLess, opLessEqual, opGreater,
    opGreaterEqual);

  TXPathCompareNode = class(TXPathBinaryNode)
  private
    FOperator: TXPathCompareOp;
  public
    constructor Create(AOperator: TXPathCompareOp;
      AOperand1, AOperand2: TXPathExprNode);
    function Evaluate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): TXPathVariable; override;
  end;


  // Node for boolean operations (and, or)

  TXPathBooleanOp = (opOr, opAnd);

  TXPathBooleanOpNode = class(TXPathBinaryNode)
  private
    FOperator: TXPathBooleanOp;
  public
    constructor Create(AOperator: TXPathBooleanOp;
      AOperand1, AOperand2: TXPathExprNode);
    function Evaluate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): TXPathVariable; override;
  end;


  // Node for unions (see [18])

  TXPathUnionNode = class(TXPathBinaryNode)
  public
    constructor Create(AOperand1, AOperand2: TXPathExprNode);
    function Evaluate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): TXPathVariable; override;
  end;


  TNodeSet = TFPList;

  // Filter node (for [20])

  TXPathFilterNode = class(TXPathExprNode)
  private
    FLeft: TXPathExprNode;
    FPredicates: TXPathNodeArray;
    procedure ApplyPredicates(Nodes: TNodeSet; AEnvironment: TXPathEnvironment);
  public
    constructor Create(AExpr: TXPathExprNode);
    destructor Destroy; override;
    function Evaluate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): TXPathVariable; override;
  end;


  // Node for location paths

  TAxis = (axisInvalid, axisAncestor, axisAncestorOrSelf, axisAttribute,
    axisChild, axisDescendant, axisDescendantOrSelf, axisFollowing,
    axisFollowingSibling, axisNamespace, axisParent, axisPreceding,
    axisPrecedingSibling, axisSelf, axisRoot);

  TNodeTestType = (ntAnyPrincipal, ntName, ntTextNode,
    ntCommentNode, ntPINode, ntAnyNode);

  TStep = class(TXPathFilterNode)
  private
    procedure SelectNodes(ANode: TDOMNode; out ResultNodes: TNodeSet);
  public
    Axis: TAxis;
    NodeTestType: TNodeTestType;
    NodeTestString: DOMString;
<<<<<<< HEAD
    NSTestString: DOMString;
    constructor Create(aAxis: TAxis; aTest: TNodeTestType);
=======
    Predicates: TList;
  end;

  TXPathLocationPathNode = class(TXPathExprNode)
  private
    FFirstStep: TStep;
    FIsAbsolutePath: Boolean;
  public
    constructor Create(AIsAbsolutePath: Boolean);
    destructor destroy;override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function Evaluate(AContext: TXPathContext;
      AEnvironment: TXPathEnvironment): TXPathVariable; override;
  end;

{ Exceptions }

  EXPathEvaluationError = class(Exception);

  procedure EvaluationError(const Msg: String);
  procedure EvaluationError(const Msg: String; const Args: array of const);


type

{ XPath variables and results classes }

  TXPathVariable = class
  protected
    FRefCount: Integer;
    procedure Error(const Msg: String; const Args: array of const);
  public
    class function TypeName: String; virtual; abstract;
    procedure Release;
    function AsNodeSet: TNodeSet; virtual;
    function AsBoolean: Boolean; virtual; abstract;
    function AsNumber: Extended; virtual; abstract;
    function AsText: DOMString; virtual; abstract;
  end;

  TXPathNodeSetVariable = class(TXPathVariable)
  private
    FValue: TNodeSet;
  public
    constructor Create(AValue: TNodeSet);
    destructor Destroy; override;
    class function TypeName: String; override;
    function AsNodeSet: TNodeSet; override;
    function AsText: DOMString; override;
    function AsBoolean: Boolean; override;
    function AsNumber: Extended; override;
    property Value: TNodeSet read FValue;
  end;

  TXPathBooleanVariable = class(TXPathVariable)
  private
    FValue: Boolean;
  public
    constructor Create(AValue: Boolean);
    class function TypeName: String; override;
    function AsBoolean: Boolean; override;
    function AsNumber: Extended; override;
    function AsText: DOMString; override;
    property Value: Boolean read FValue;
  end;

  TXPathNumberVariable = class(TXPathVariable)
  private
    FValue: Extended;
  public
    constructor Create(AValue: Extended);
    class function TypeName: String; override;
    function AsBoolean: Boolean; override;
    function AsNumber: Extended; override;
    function AsText: DOMString; override;
    property Value: Extended read FValue;
  end;

  TXPathStringVariable = class(TXPathVariable)
  private
    FValue: DOMString;
  public
    constructor Create(const AValue: DOMString);
    class function TypeName: String; override;
    function AsBoolean: Boolean; override;
    function AsNumber: Extended; override;
    function AsText: DOMString; override;
    property Value: DOMString read FValue;
  end;

  TXPathNSResolver = class
  protected
    FNode: TDOMNode;
  public
    constructor Create(aNode: TDOMNode);
    function LookupNamespaceURI(const aPrefix: DOMString): DOMString; virtual;
  end;

  TXPathNSResolver = TDOMNode {!!! experimental};

  TXPathNSResolver = TDOMNode {!!! experimental};

  TXPathNSResolver = TDOMNode {!!! experimental};

  TXPathNSResolver = TDOMNode {!!! experimental};

{ XPath lexical scanner }

  TXPathScanner = class
  private
    FExpressionString, FCurData: DOMPChar;
    FCurToken: TXPathToken;
    FCurTokenString: DOMString;
    FTokenStart: DOMPChar;
    FTokenLength: Integer;
    FPrefixLength: Integer;
    FTokenId: TXPathKeyword;
    FResolver: TXPathNSResolver;
    procedure Error(const Msg: String);
    procedure ParsePredicates(var Dest: TXPathNodeArray);
    function ParseStep: TStep;          // [4]
    function ParseNodeTest(axis: TAxis): TStep; // [7]
    function ParsePrimaryExpr: TXPathExprNode; // [15]
    function ParseFunctionCall: TXPathExprNode; // [16]
    function ParseUnionExpr: TXPathExprNode;   // [18]
    function ParsePathExpr: TXPathExprNode;    // [19]
    function ParseFilterExpr: TXPathExprNode;  // [20]
    function ParseOrExpr: TXPathExprNode;      // [21]
    function ParseAndExpr: TXPathExprNode;     // [22]
    function ParseEqualityExpr: TXPathExprNode;    // [23]
    function ParseRelationalExpr: TXPathExprNode;  // [24]
    function ParseAdditiveExpr: TXPathExprNode;    // [25]
    function ParseMultiplicativeExpr: TXPathExprNode;  // [26]
    function ParseUnaryExpr: TXPathExprNode;   // [27]
    function GetToken: TXPathToken;
    function ScanQName: Boolean;
  public
    constructor Create(const AExpressionString: DOMString);
    function NextToken: TXPathToken;
    function PeekToken: TXPathToken;
    function SkipToken(tok: TXPathToken): Boolean;
    property CurToken: TXPathToken read FCurToken;
    property CurTokenString: DOMString read FCurTokenString;
  end;


{ XPath context }

  TXPathContext = class
  public
    ContextNode: TDOMNode;
    ContextPosition: Integer;
    ContextSize: Integer;

    constructor Create(AContextNode: TDOMNode;
      AContextPosition, AContextSize: Integer);
  end;


{ XPath environments (not defined in XPath standard: an environment contains
  the variables and functions, which are part of the context in the official
  standard). }

  TXPathVarList = TFPList;

  TXPathFunction = function(Context: TXPathContext; Args: TXPathVarList):
    TXPathVariable of object;

  TXPathEnvironment = class
  private
    FFunctions: TFPList;
    FVariables: TFPList;
    function GetFunctionCount: Integer;
    function GetVariableCount: Integer;
    function GetFunction(Index: Integer): TXPathFunction;
    function GetFunction(const AName: String): TXPathFunction;
    function GetVariable(Index: Integer): TXPathVariable;
    function GetVariable(const AName: String): TXPathVariable;
  protected
    // XPath Core Function Library:
    function xpLast(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpPosition(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpCount(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpId(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpLocalName(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpNamespaceURI(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpName(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpString(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpConcat(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpStartsWith(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpContains(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpSubstringBefore(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpSubstringAfter(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpSubstring(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpStringLength(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpNormalizeSpace(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpTranslate(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpBoolean(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpNot(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpTrue(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpFalse(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpLang(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpNumber(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpSum(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpFloor(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpCeiling(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
    function xpRound(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
  public
    constructor Create;
    destructor Destroy; override;
    function GetFunctionIndex(const AName: String): Integer;
    function GetVariableIndex(const AName: String): Integer;
    procedure AddFunction(const AName: String; AFunction: TXPathFunction);
    procedure AddVariable(const AName: String; AVariable: TXPathVariable);
    procedure RemoveFunction(Index: Integer);
    procedure RemoveFunction(const AName: String);
    procedure RemoveVariable(Index: Integer);
    procedure RemoveVariable(const AName: String);
    property FunctionCount: Integer read GetFunctionCount;
    property VariableCount: Integer read GetVariableCount;
    property Functions[Index: Integer]: TXPathFunction read GetFunction;
    property FunctionsByName[const AName: String]: TXPathFunction
       read GetFunction;
    property Variables[Index: Integer]: TXPathVariable read GetVariable;
    property VariablesByName[const AName: String]: TXPathVariable read GetVariable;
  end;


{ XPath expressions }

  TXPathExpression = class
  private
    FRootNode: TXPathExprNode;
  public
    { CompleteExpresion specifies wether the parser should check for gargabe
      after the recognised part. True => Throw exception if there is garbage }
<<<<<<< HEAD
    constructor Create(AScanner: TXPathScanner; CompleteExpression: Boolean;
      AResolver: TXPathNSResolver = nil);
    destructor Destroy; override;
=======
    constructor Create(AScanner: TXPathScanner; CompleteExpression: Boolean);
    destructor destroy;override;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    function Evaluate(AContextNode: TDOMNode): TXPathVariable;
    function Evaluate(AContextNode: TDOMNode;
      AEnvironment: TXPathEnvironment): TXPathVariable;
  end;


function EvaluateXPathExpression(const AExpressionString: DOMString;
  AContextNode: TDOMNode; AResolver: TXPathNSResolver = nil): TXPathVariable;


// ===================================================================
// ===================================================================

implementation

uses Math, xmlutils;

{$i xpathkw.inc}

const
  AxisNameKeywords = [xkAncestor..xkSelf];
  AxisNameMap: array[xkAncestor..xkSelf] of TAxis = (
    axisAncestor, axisAncestorOrSelf, axisAttribute, axisChild,
    axisDescendant, axisDescendantOrSelf, axisFollowing,
    axisFollowingSibling, axisNamespace, axisParent, axisPreceding,
    axisPrecedingSibling, axisSelf
  );
  NodeTestKeywords = [xkComment..xkNode];
  NodeTestMap: array[xkComment..xkNode] of TNodeTestType = (
    ntCommentNode, ntTextNode, ntPINode, ntAnyNode
  );

  FunctionKeywords = [xkLast..xkRound];

{ Helper functions }

function NodeToText(Node: TDOMNode): DOMString;
var
  Child: TDOMNode;
begin
  case Node.NodeType of
    DOCUMENT_NODE, DOCUMENT_FRAGMENT_NODE{, ELEMENT_NODE}:
      begin
        SetLength(Result, 0);
        Child := Node.FirstChild;
        while Assigned(Child) do
        begin
	  if Result <> '' then
	    Result := Result + LineEnding;
          Result := Result + NodeToText(Child);
          Child := Child.NextSibling;
        end;
      end;
    ELEMENT_NODE:
      Result := Node.TextContent;
    ATTRIBUTE_NODE, PROCESSING_INSTRUCTION_NODE, COMMENT_NODE, TEXT_NODE,
      CDATA_SECTION_NODE, ENTITY_REFERENCE_NODE:
      Result := Node.NodeValue;
  end;
  // !!!: What to do with 'namespace nodes'?
end;

function StrToNumber(const s: DOMString): Extended;
var
  Code: Integer;
begin
  Val(s, Result, Code);
{$push}
<<<<<<< HEAD
{$r-,q-}
=======
{$r-}
>>>>>>> origin/fixes_2.4
  if Code <> 0 then
    Result := NaN;
{$pop}
end;

procedure TranslateWideString(var S: DOMString; const SrcPat, DstPat: DOMString);
var
  I, J, L: Integer;
  P, Start: DOMPChar;
begin
  UniqueString(S);
  L := Length(DstPat);
  P := DOMPChar(S);
  if Length(SrcPat) > L then  // may remove some chars
  begin
    Start := P;
    for I := 1 to Length(S) do
    begin
      J := Pos(S[I], SrcPat);
      if J > 0 then
      begin
        if J <= L then
        begin
          P^ := DstPat[J];
          Inc(P);
        end;
      end
      else
      begin
        P^ := S[I];
        Inc(P);
      end;
    end;
    SetLength(S, P-Start);
  end
  else  // no char removal possible
    for I := 1 to Length(S) do
    begin
      J := Pos(S[I], SrcPat);
      if J > 0 then
        P^ := DstPat[J]
      else
        P^ := S[I];
      Inc(P);  
    end;
end;

function GetNodeLanguage(aNode: TDOMNode): DOMString;
var
  Attr: TDomAttr;
begin
  Result := '';
  if aNode = nil then
    Exit;
  case aNode.NodeType of
    ELEMENT_NODE: begin
      Attr := TDomElement(aNode).GetAttributeNode('xml:lang');
      if Assigned(Attr) then
        Result := Attr.Value
      else
        Result := GetNodeLanguage(aNode.ParentNode);
    end;
    TEXT_NODE, CDATA_SECTION_NODE, ENTITY_REFERENCE_NODE,
    PROCESSING_INSTRUCTION_NODE, COMMENT_NODE:
      Result := GetNodeLanguage(aNode.ParentNode);
    ATTRIBUTE_NODE:
      Result := GetNodeLanguage(TDOMAttr(aNode).OwnerElement);
  end;
end;

procedure AddNodes(var Dst: TXPathNodeArray; const Src: array of TXPathExprNode;
  var Count: Integer);
var
  L: Integer;
begin
  if Count > 0 then
  begin
    L := Length(Dst);
    SetLength(Dst, L + Count);
    Move(Src[0], Dst[L], Count*sizeof(TObject));
    Count := 0;
  end;
end;

{ XPath parse tree classes }

function TXPathExprNode.EvalPredicate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): Boolean;
var
  resvar: TXPathVariable;
begin
  resvar := Evaluate(AContext, AEnvironment);
  try
    if resvar.InheritsFrom(TXPathNumberVariable) then
      Result := resvar.AsNumber = AContext.ContextPosition   // TODO: trunc/round?
    else
      Result := resvar.AsBoolean;
  finally
    resvar.Release;
  end;
end;

constructor TXPathConstantNode.Create(AValue: TXPathVariable);
begin
  inherited Create;
  FValue := AValue;
end;

destructor TXPathConstantNode.Destroy;
begin
  FValue.Release;
  inherited Destroy;
end;

function TXPathConstantNode.Evaluate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): TXPathVariable;
begin
  Result := FValue;
  Inc(Result.FRefCount);
end;


constructor TXPathVariableNode.Create(const AName: DOMString);
begin
  inherited Create;
  FName := AName;
end;

function TXPathVariableNode.Evaluate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): TXPathVariable;
begin
  Result := AEnvironment.VariablesByName[FName];
  if not Assigned(Result) then
    EvaluationError(SEvalUnknownVariable, [FName]);
end;


constructor TXPathFunctionNode.Create(const AName: DOMString; const Args: TXPathNodeArray);
begin
  inherited Create;
  FName := AName;
  FArgs := Args;
end;

destructor TXPathFunctionNode.Destroy;
var
  i: Integer;
begin
  for i := Low(FArgs) to High(FArgs) do
    FArgs[i].Free;
  inherited Destroy;
end;

function TXPathFunctionNode.Evaluate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): TXPathVariable;
var
  Fn: TXPathFunction;
  Args: TXPathVarList;
  i: Integer;
begin
  Fn := AEnvironment.FunctionsByName[FName];
  if not Assigned(Fn) then
    EvaluationError(SEvalUnknownFunction, [FName]);

  Args := TXPathVarList.Create;
  try
    for i := Low(FArgs) to High(FArgs) do
      Args.Add(FArgs[i].Evaluate(AContext, AEnvironment));
    Result := Fn(AContext, Args);
    for i := Low(FArgs) to High(FArgs) do
      TXPathVariable(Args[i]).Release;
  finally
    Args.Free;
  end;
end;


constructor TXPathNegationNode.Create(AOperand: TXPathExprNode);
begin
  inherited Create;
  FOperand := AOperand;
end;

destructor TXPathNegationNode.Destroy;
begin
  FOperand.Free;
  inherited Destroy;
end;

function TXPathNegationNode.Evaluate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): TXPathVariable;
var
  OpResult: TXPathVariable;
begin
  OpResult := FOperand.Evaluate(AContext, AEnvironment);
  try
    Result := TXPathNumberVariable.Create(-OpResult.AsNumber);
  finally
    OpResult.Release;
  end;
end;

destructor TXPathBinaryNode.Destroy;
begin
  FOperand1.Free;
  FOperand2.Free;
  inherited Destroy;
end;

constructor TXPathMathOpNode.Create(AOperator: TXPathMathOp;
  AOperand1, AOperand2: TXPathExprNode);
begin
  inherited Create;
  FOperator := AOperator;
  FOperand1 := AOperand1;
  FOperand2 := AOperand2;
end;

function TXPathMathOpNode.Evaluate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): TXPathVariable;
var
  Op1Result, Op2Result: TXPathVariable;
  Op1, Op2, NumberResult: Extended;
begin
  Op1Result := FOperand1.Evaluate(AContext, AEnvironment);
  try
    Op2Result := FOperand2.Evaluate(AContext, AEnvironment);
    try
      Op1 := Op1Result.AsNumber;
      Op2 := Op2Result.AsNumber;
      case FOperator of
        opAdd:
          NumberResult := Op1 + Op2;
        opSubtract:
          NumberResult := Op1 - Op2;
        opMultiply:
          NumberResult := Op1 * Op2;
        opDivide:
          NumberResult := Op1 / Op2;
        opMod: if IsNan(Op1) or IsNan(Op2) then
{$push}
<<<<<<< HEAD
{$r-,q-}
=======
{$r-}
>>>>>>> origin/fixes_2.4
          NumberResult := NaN
{$pop}
        else
          NumberResult := Trunc(Op1) mod Trunc(Op2);
      end;
    finally
      Op2Result.Release;
    end;
  finally
    Op1Result.Release;
  end;
  Result := TXPathNumberVariable.Create(NumberResult);
end;

const
  reverse: array[TXPathCompareOp] of TXPathCompareOp = (
    opEqual, opNotEqual,
    opGreaterEqual, //opLess
    opGreater,      //opLessEqual
    opLessEqual,    //opGreater
    opLess          //opGreaterEqual
  );
  
function CmpNumbers(const n1, n2: Extended; op: TXPathCompareOp): Boolean;
begin
  result := (op = opNotEqual);
  if IsNan(n1) or IsNan(n2) then
    Exit;    // NaNs are not equal
  case op of
    // TODO: should CompareValue() be used here?
    opLess:         result := n1 < n2;
    opLessEqual:    result := n1 <= n2;
    opGreater:      result := n1 > n2;
    opGreaterEqual: result := n1 >= n2;
  else
    if IsInfinite(n1) or IsInfinite(n2) then
      result := n1 = n2
    else
      result := SameValue(n1, n2);
    result := result xor (op = opNotEqual);
  end;
end;

function CmpStrings(const s1, s2: DOMString; op: TXPathCompareOp): Boolean;
begin
  case op of
    opEqual:    result := s1 = s2;
    opNotEqual: result := s1 <> s2;
  else
    result := CmpNumbers(StrToNumber(s1), StrToNumber(s2), op);
  end;
end;

function CmpNodesetWithString(ns: TNodeSet; const s: DOMString; op: TXPathCompareOp): Boolean;
var
  i: Integer;
begin
  Result := True;
  for i := 0 to ns.Count - 1 do
  begin
    if CmpStrings(NodeToText(TDOMNode(ns[i])), s, op) then
      exit;
  end;
  Result := False;
end;

function CmpNodesetWithNumber(ns: TNodeSet; const n: Extended; op: TXPathCompareOp): Boolean;
var
  i: Integer;
begin
  Result := True;
  for i := 0 to ns.Count - 1 do
  begin
    if CmpNumbers(StrToNumber(NodeToText(TDOMNode(ns[i]))), n, op) then
      exit;
  end;
  Result := False;
end;

function CmpNodesetWithBoolean(ns: TNodeSet; b: Boolean; op: TXPathCompareOp): Boolean;
begin
// TODO: handles only equality
  result := ((ns.Count <> 0) = b) xor (op = opNotEqual);
end;

function CmpNodesets(ns1, ns2: TNodeSet; op: TXPathCompareOp): Boolean;
var
  i, j: Integer;
  s: DOMString;
begin
  Result := True;
  for i := 0 to ns1.Count - 1 do
  begin
    s := NodeToText(TDOMNode(ns1[i]));
    for j := 0 to ns2.Count - 1 do
    if CmpStrings(s, NodeToText(TDOMNode(ns2[j])), op) then
      exit;
  end;
  Result := False;
end;

constructor TXPathCompareNode.Create(AOperator: TXPathCompareOp;
  AOperand1, AOperand2: TXPathExprNode);
begin
  inherited Create;
  FOperator := AOperator;
  FOperand1 := AOperand1;
  FOperand2 := AOperand2;
end;

function TXPathCompareNode.Evaluate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): TXPathVariable;
var
  Op1, Op2: TXPathVariable;
  BoolResult: Boolean;
  nsnum: Integer;
begin
  Op1 := FOperand1.Evaluate(AContext, AEnvironment);
  try
    Op2 := FOperand2.Evaluate(AContext, AEnvironment);
    try
      nsnum := ord(Op1 is TXPathNodeSetVariable) or
       (ord(Op2 is TXPathNodeSetVariable) shl 1);
      case nsnum of
        0: begin  // neither op is a nodeset
          if (FOperator in [opEqual, opNotEqual]) then
          begin
            if (Op1 is TXPathBooleanVariable) or (Op2 is TXPathBooleanVariable) then
              BoolResult := (Op1.AsBoolean = Op2.AsBoolean) xor (FOperator = opNotEqual)
            else if (Op1 is TXPathNumberVariable) or (Op2 is TXPathNumberVariable) then
              BoolResult := CmpNumbers(Op1.AsNumber, Op2.AsNumber, FOperator)
            else
              BoolResult := (Op1.AsText = Op2.AsText) xor (FOperator = opNotEqual);
          end
          else
            BoolResult := CmpNumbers(Op1.AsNumber, Op2.AsNumber, FOperator);
        end;

        1: // Op1 is nodeset
          if Op2 is TXPathNumberVariable then
            BoolResult := CmpNodesetWithNumber(Op1.AsNodeSet, Op2.AsNumber, FOperator)
          else if Op2 is TXPathStringVariable then
            BoolResult := CmpNodesetWithString(Op1.AsNodeSet, Op2.AsText, FOperator)
          else
            BoolResult := CmpNodesetWithBoolean(Op1.AsNodeSet, Op2.AsBoolean, FOperator);

        2: // Op2 is nodeset
          if Op1 is TXPathNumberVariable then
            BoolResult := CmpNodesetWithNumber(Op2.AsNodeSet, Op1.AsNumber, reverse[FOperator])
          else if Op1 is TXPathStringVariable then
            BoolResult := CmpNodesetWithString(Op2.AsNodeSet, Op1.AsText, reverse[FOperator])
          else
            BoolResult := CmpNodesetWithBoolean(Op2.AsNodeSet, Op1.AsBoolean, reverse[FOperator]);

      else  // both ops are nodesets
        BoolResult := CmpNodesets(Op1.AsNodeSet, Op2.AsNodeSet, FOperator);
      end;
    finally
      Op2.Release;
    end;
  finally
    Op1.Release;
  end;
  Result := TXPathBooleanVariable.Create(BoolResult);
end;

constructor TXPathBooleanOpNode.Create(AOperator: TXPathBooleanOp;
  AOperand1, AOperand2: TXPathExprNode);
begin
  inherited Create;
  FOperator := AOperator;
  FOperand1 := AOperand1;
  FOperand2 := AOperand2;
end;

function TXPathBooleanOpNode.Evaluate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): TXPathVariable;
var
  res: Boolean;
  Op1, Op2: TXPathVariable;
begin
  { don't evaluate second arg if result is determined by first one }
  Op1 := FOperand1.Evaluate(AContext, AEnvironment);
  try
    res := Op1.AsBoolean;
  finally
    Op1.Release;
  end;
  if not (((FOperator = opAnd) and (not res)) or ((FOperator = opOr) and res)) then
  begin
    Op2 := FOperand2.Evaluate(AContext, AEnvironment);
    try
      case FOperator of
        opAnd: res := res and Op2.AsBoolean;
        opOr:  res := res or Op2.AsBoolean;
      end;
    finally
      Op2.Release;
    end;
  end;
  Result := TXPathBooleanVariable.Create(res);
end;

constructor TXPathUnionNode.Create(AOperand1, AOperand2: TXPathExprNode);
begin
  inherited Create;
  FOperand1 := AOperand1;
  FOperand2 := AOperand2;
end;

function TXPathUnionNode.Evaluate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): TXPathVariable;
var
  Op1Result, Op2Result: TXPathVariable;
  NodeSet, NodeSet2: TNodeSet;
  CurNode: Pointer;
  i: Integer;
begin
{ TODO: result must be sorted by document order, i.e. 'a|b' yields the
  same nodeset as 'b|a' }
  Op1Result := FOperand1.Evaluate(AContext, AEnvironment);
  try
    Op2Result := FOperand2.Evaluate(AContext, AEnvironment);
    try
      NodeSet := Op1Result.AsNodeSet;
      NodeSet2 := Op2Result.AsNodeSet;
      for i := 0 to NodeSet2.Count - 1 do
      begin
        CurNode := NodeSet2[i];
        if NodeSet.IndexOf(CurNode) < 0 then
          NodeSet.Add(CurNode);
      end;
    finally
      Op2Result.Release;
    end;
  finally
    Result := Op1Result;
  end;
end;


constructor TXPathFilterNode.Create(AExpr: TXPathExprNode);
begin
  inherited Create;
  FLeft := AExpr;
end;

destructor TXPathFilterNode.Destroy;
var
  i: Integer;
begin
  FLeft.Free;
  for i := 0 to High(FPredicates) do
    FPredicates[i].Free;
  inherited Destroy;
end;

function TXPathFilterNode.Evaluate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): TXPathVariable;
var
  NodeSet: TNodeSet;
begin
  Result := FLeft.Evaluate(AContext, AEnvironment);
  NodeSet := Result.AsNodeSet;
  ApplyPredicates(NodeSet, AEnvironment);
end;


constructor TStep.Create(aAxis: TAxis; aTest: TNodeTestType);
begin
  Axis := aAxis;
  NodeTestType := aTest;
end;

procedure TStep.SelectNodes(ANode: TDOMNode; out ResultNodes: TNodeSet);
var
  Node, Node2: TDOMNode;
  Attr: TDOMNamedNodeMap;
  i: Integer;
<<<<<<< HEAD
=======
begin
  for i := 0 to Predicates.Count - 1 do
    TXPathExprNode(Predicates[i]).Free;
  Predicates.Free;
  inherited destroy;
end;

constructor TXPathLocationPathNode.Create(AIsAbsolutePath: Boolean);
begin
  inherited Create;
  FIsAbsolutePath := AIsAbsolutePath;
end;
>>>>>>> graemeg/fixes_2_2

  procedure DoNodeTest(Node: TDOMNode);
  begin
    case NodeTestType of
      ntAnyPrincipal:
        // !!!: Probably this isn't ready for namespace support yet
        if (Axis <> axisAttribute) and
          (Node.NodeType <> ELEMENT_NODE) then
          exit;
      ntName:
        if NSTestString <> '' then
        begin
          if Node.namespaceURI <> NSTestString then
            exit;
          if (NodeTestString <> '') and (Node.localName <> NodeTestString) then
            exit;
        end
        else if Node.NodeName <> NodeTestString then
          exit;
      ntTextNode:
        if not Node.InheritsFrom(TDOMText) then
          exit;
      ntCommentNode:
        if Node.NodeType <> COMMENT_NODE then
          exit;
      ntPINode:
        if (Node.NodeType <> PROCESSING_INSTRUCTION_NODE) or
         ((NodeTestString <> '') and (Node.nodeName <> NodeTestString)) then
          exit;
    end;
    if ResultNodes.IndexOf(Node) < 0 then
      ResultNodes.Add(Node);
  end;

  procedure AddDescendants(CurNode: TDOMNode);
  var
    Child: TDOMNode;
  begin
    Child := CurNode.FirstChild;
    while Assigned(Child) do
    begin
      DoNodeTest(Child);
      AddDescendants(Child);
      Child := Child.NextSibling;
    end;
  end;

  procedure AddDescendantsReverse(CurNode: TDOMNode);
  var
    Child: TDOMNode;
  begin
    Child := CurNode.LastChild;
    while Assigned(Child) do
    begin
      AddDescendantsReverse(Child);
      DoNodeTest(Child);
      Child := Child.PreviousSibling;
    end;
  end;

begin
  ResultNodes := TNodeSet.Create;
  case Axis of
    axisAncestor:
      begin
        // TODO: same check needed for XPATH_NAMESPACE_NODE
        if ANode.nodeType = ATTRIBUTE_NODE then
          Node := TDOMAttr(ANode).ownerElement
        else
          Node := ANode.ParentNode;
        while Assigned(Node) do
        begin
          DoNodeTest(Node);
          Node := Node.ParentNode;
        end;
      end;
    axisAncestorOrSelf:
      begin
        DoNodeTest(ANode);
        // TODO: same check needed for XPATH_NAMESPACE_NODE
        if ANode.nodeType = ATTRIBUTE_NODE then
          Node := TDOMAttr(ANode).ownerElement
        else
          Node := ANode.ParentNode;
        while Assigned(Node) do
        begin
          DoNodeTest(Node);
          Node := Node.ParentNode;
        end;
      end;
    axisAttribute:
      begin
        Attr := ANode.Attributes;
        if Assigned(Attr) then
          for i := 0 to Attr.Length - 1 do
            DoNodeTest(Attr[i]);
      end;
    axisChild:
      begin
        Node := ANode.FirstChild;
        while Assigned(Node) do
        begin
          DoNodeTest(Node);
          Node := Node.NextSibling;
        end;
      end;
    axisDescendant:
      AddDescendants(ANode);
    axisDescendantOrSelf:
      begin
        DoNodeTest(ANode);
        AddDescendants(ANode);
      end;
    axisFollowing:
      begin
        Node := ANode;
        repeat
          Node2 := Node.NextSibling;
          while Assigned(Node2) do
          begin
            DoNodeTest(Node2);
            AddDescendants(Node2);
            Node2 := Node2.NextSibling;
          end;
          Node := Node.ParentNode;
        until not Assigned(Node);
      end;
    axisFollowingSibling:
      begin
        Node := ANode.NextSibling;
        while Assigned(Node) do
        begin
          DoNodeTest(Node);
          Node := Node.NextSibling;
        end;
      end;
    {axisNamespace: !!!: Not supported yet}
    axisParent:
      if ANode.NodeType=ATTRIBUTE_NODE then
      begin
        if Assigned(TDOMAttr(ANode).OwnerElement) then
          DoNodeTest(TDOMAttr(ANode).OwnerElement);
      end
      else if Assigned(ANode.ParentNode) then
        DoNodeTest(ANode.ParentNode);
    axisPreceding:
      begin
        Node := ANode;
        repeat
          Node2 := Node.PreviousSibling;
          while Assigned(Node2) do
          begin
            AddDescendantsReverse(Node2);
            DoNodeTest(Node2);
            Node2 := Node2.PreviousSibling;
          end;
          Node := Node.ParentNode;
        until not Assigned(Node);
      end;
    axisPrecedingSibling:
      begin
        Node := ANode.PreviousSibling;
        while Assigned(Node) do
        begin
          DoNodeTest(Node);
          Node := Node.PreviousSibling;
        end;
      end;
    axisSelf:
      DoNodeTest(ANode);
    axisRoot:
      if ANode.nodeType = DOCUMENT_NODE then
        ResultNodes.Add(ANode)
      else
        ResultNodes.Add(ANode.ownerDocument);
  end;
end;

{ Filter the nodes of this step using the predicates: The current
  node set is filtered, nodes not passing the filter are replaced
  by nil. After one filter has been applied, Nodes is packed, and
  the next filter will be processed. }

procedure TXPathFilterNode.ApplyPredicates(Nodes: TNodeSet; AEnvironment: TXPathEnvironment);
var
  i, j: Integer;
  NewContext: TXPathContext;
begin
  for i := 0 to High(FPredicates) do
  begin
    NewContext := TXPathContext.Create(nil, 0, Nodes.Count);
    try
      for j := 0 to Nodes.Count - 1 do
      begin
        NewContext.ContextPosition := j+1;
        NewContext.ContextNode := TDOMNode(Nodes[j]);
        if not FPredicates[i].EvalPredicate(NewContext, AEnvironment) then
          Nodes[j] := nil;
      end;
      Nodes.Pack;
    finally
      NewContext.Free;
    end;
  end;
end;

function TStep.Evaluate(AContext: TXPathContext;
  AEnvironment: TXPathEnvironment): TXPathVariable;
var
  ResultNodeSet: TNodeSet;
  LeftResult: TXPathVariable;
  i: Integer;

  procedure EvaluateStep(AContextNode: TDOMNode);
  var
    StepNodes: TFPList;
    Node: TDOMNode;
    i: Integer;
  begin
    SelectNodes(AContextNode, StepNodes);
    try
      ApplyPredicates(StepNodes, AEnvironment);
      if Axis in [axisAncestor, axisAncestorOrSelf,
        axisPreceding, axisPrecedingSibling] then
      for i := StepNodes.Count - 1 downto 0 do
      begin
        Node := TDOMNode(StepNodes[i]);
        if ResultNodeSet.IndexOf(Node) < 0 then
          ResultNodeSet.Add(Node);
      end
      else for i := 0 to StepNodes.Count - 1 do
      begin
        Node := TDOMNode(StepNodes[i]);
        if ResultNodeSet.IndexOf(Node) < 0 then
          ResultNodeSet.Add(Node);
      end;
    finally
      StepNodes.Free;
    end;
  end;

begin
  ResultNodeSet := TNodeSet.Create;
  try
    if Assigned(FLeft) then
    begin
      LeftResult := FLeft.Evaluate(AContext, AEnvironment);
      try
        with LeftResult.AsNodeSet do
          for i := 0 to Count-1 do
            EvaluateStep(TDOMNode(Items[i]));
      finally
        LeftResult.Release;
      end;
    end
    else  
      EvaluateStep(AContext.ContextNode);
  except
    ResultNodeSet.Free;
    raise;
  end;
  Result := TXPathNodeSetVariable.Create(ResultNodeSet);
end;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
destructor TXPathLocationPathNode.destroy;
var tmp:TStep;
begin
 while FFirstStep<>nil do begin
  tmp:=FFirstStep.NextStep;
  FFirstStep.free;
  FFirstStep:=tmp;
 end;
end;

>>>>>>> graemeg/fixes_2_2
{ Exceptions }

procedure EvaluationError(const Msg: String);
begin
  raise EXPathEvaluationError.Create(Msg) at get_caller_addr(get_frame), get_caller_frame(get_frame);
end;

procedure EvaluationError(const Msg: String; const Args: array of const);
begin
  raise EXPathEvaluationError.CreateFmt(Msg, Args)
    at get_caller_addr(get_frame), get_caller_frame(get_frame);
end;


{ TXPathVariable and derived classes}

procedure TXPathVariable.Release;
begin
  if FRefCount <= 0 then
    Free
  else
    Dec(FRefCount);
end;

function TXPathVariable.AsNodeSet: TNodeSet;
begin
  Error(SVarNoConversion, [TypeName, TXPathNodeSetVariable.TypeName]);
  Result := nil;
end;

procedure TXPathVariable.Error(const Msg: String; const Args: array of const);
begin
  raise Exception.CreateFmt(Msg, Args) at get_caller_addr(get_frame), get_caller_frame(get_frame);
end;


constructor TXPathNodeSetVariable.Create(AValue: TNodeSet);
begin
  inherited Create;
  FValue := AValue;
end;

destructor TXPathNodeSetVariable.Destroy;
begin
  FValue.Free;
  inherited Destroy;
end;

class function TXPathNodeSetVariable.TypeName: String;
begin
  Result := SNodeSet;
end;

function TXPathNodeSetVariable.AsNodeSet: TNodeSet;
begin
  Result := FValue;
end;

function TXPathNodeSetVariable.AsText: DOMString;
begin
  if FValue.Count = 0 then
    Result := ''
  else
    Result := NodeToText(TDOMNode(FValue.First));
end;

function TXPathNodeSetVariable.AsBoolean: Boolean;
begin
  Result := FValue.Count <> 0;
end;

function TXPathNodeSetVariable.AsNumber: Extended;
begin
  Result := StrToNumber(AsText);
end;

constructor TXPathBooleanVariable.Create(AValue: Boolean);
begin
  inherited Create;
  FValue := AValue;
end;

class function TXPathBooleanVariable.TypeName: String;
begin
  Result := SBoolean;
end;

function TXPathBooleanVariable.AsBoolean: Boolean;
begin
  Result := FValue;
end;

function TXPathBooleanVariable.AsNumber: Extended;
begin
  if FValue then
    Result := 1
  else
    Result := 0;
end;

function TXPathBooleanVariable.AsText: DOMString;
begin
  if FValue then
    Result := 'true'    // Do not localize!
  else
    Result := 'false';  // Do not localize!
end;


constructor TXPathNumberVariable.Create(AValue: Extended);
begin
  inherited Create;
  FValue := AValue;
end;

class function TXPathNumberVariable.TypeName: String;
begin
  Result := SNumber;
end;

function TXPathNumberVariable.AsBoolean: Boolean;
begin
  Result := not (IsNan(FValue) or IsZero(FValue));
end;

function TXPathNumberVariable.AsNumber: Extended;
begin
  Result := FValue;
end;

function TXPathNumberVariable.AsText: DOMString;
var
  frec: TFloatRec;
  i, nd, reqlen: Integer;
  P: DOMPChar;
begin
  FloatToDecimal(frec, FValue, fvExtended, 17, 9999);

  if frec.Exponent = -32768 then
  begin
    Result := 'NaN';          // do not localize
    Exit;
  end
  else if frec.Exponent = 32767 then
  begin
    if frec.Negative then
      Result := '-Infinity'   // do not localize
    else
      Result := 'Infinity';   // do not localize
    Exit;  
  end
  else if frec.Digits[0] = #0 then
  begin
    Result := '0';
    Exit;
  end
  else
  begin
    nd := StrLen(@frec.Digits[0]);
    reqlen := nd + ord(frec.Negative);  // maybe minus sign
    if frec.Exponent > nd then
      Inc(reqlen, frec.Exponent - nd)   // add this much zeroes at the right
    else if frec.Exponent < nd then
    begin
      Inc(reqlen);                      // decimal point
      if frec.Exponent <= 0 then
        Inc(reqlen, 1 - frec.Exponent); // zeroes at the left + one more for the int part
    end;
    SetLength(Result, reqlen);
    P := DOMPChar(Result);
    if frec.Negative then
    begin
      P^ := '-';
      Inc(P);
    end;
    if frec.Exponent <= 0 then          // value less than 1, put zeroes at left
    begin
      for i := 0 to 1-frec.Exponent do
        P[i] := '0';
      P[1] := '.';
      for i := 0 to nd-1 do
        P[i+2-frec.Exponent] := WideChar(ord(frec.Digits[i]));
    end
    else if frec.Exponent > nd then    // large integer, put zeroes at right
    begin
      for i := 0 to nd-1 do
        P[i] := WideChar(ord(frec.Digits[i]));
      for i := nd to reqlen-1-ord(frec.Negative) do
        P[i] := '0';
    end
    else  // 0 < exponent <= digits, insert decimal point into middle
    begin
      for i := 0 to frec.Exponent-1 do
        P[i] := WideChar(ord(frec.Digits[i]));
      if frec.Exponent < nd then
      begin
        P[frec.Exponent] := '.';
        for i := frec.Exponent to nd-1 do
          P[i+1] := WideChar(ord(frec.Digits[i]));
      end;
    end;
  end;
end;


constructor TXPathStringVariable.Create(const AValue: DOMString);
begin
  inherited Create;
  FValue := AValue;
end;

class function TXPathStringVariable.TypeName: String;
begin
  Result := SString;
end;

function TXPathStringVariable.AsBoolean: Boolean;
begin
  Result := Length(FValue) > 0;
end;

function TXPathStringVariable.AsNumber: Extended;
begin
  Result := StrToNumber(FValue);
end;

function TXPathStringVariable.AsText: DOMString;
begin
  Result := FValue;
end;


{ XPath lexical scanner }

constructor TXPathScanner.Create(const AExpressionString: DOMString);
begin
  inherited Create;
  FExpressionString := DOMPChar(AExpressionString);
  FCurData := FExpressionString;
  NextToken;
end;

function TXPathScanner.PeekToken: TXPathToken;
var
  save: DOMPChar;
begin
  save := FCurData;
  Result := GetToken;
  FCurData := save;
end;

function TXPathScanner.NextToken: TXPathToken;
begin
  Result := GetToken;
  FCurToken := Result;
  if Result in [tkIdentifier, tkNSNameTest, tkNumber, tkString, tkVariable] then
    SetString(FCurTokenString, FTokenStart, FTokenLength);
  if Result = tkIdentifier then
    FTokenId := LookupXPathKeyword(FTokenStart, FTokenLength)
  else
    FTokenId := xkNone;
end;

function TXPathScanner.SkipToken(tok: TXPathToken): Boolean; { inline? }
begin
  Result := (FCurToken = tok);
  if Result then
    NextToken;
end;

// TODO: no surrogate pairs/XML 1.1 support yet
function TXPathScanner.ScanQName: Boolean;
var
  p: DOMPChar;
begin
  FPrefixLength := 0;
  p := FCurData;
  repeat
    if (Byte(p^) in NamingBitmap[NamePages[hi(Word(p^))]]) then
      Inc(p)
    else
    begin
      // either the first char of name is bad (it may be a colon),
      // or a colon is not followed by a valid NameStartChar
      Result := False;
      Break;
    end;

    while Byte(p^) in NamingBitmap[NamePages[$100+hi(Word(p^))]] do
      Inc(p);

    Result := True;
    if (p^ <> ':') or (p[1] = ':') or (FPrefixLength > 0) then
      Break;
    // first colon, and not followed by another one -> remember its position
    FPrefixLength := p-FTokenStart;
    Inc(p);
  until False;
  FCurData := p;
  FTokenLength := p-FTokenStart;
end;

function TXPathScanner.GetToken: TXPathToken;

  procedure GetNumber(HasDot: Boolean);
  begin
    FTokenLength := 1;
    while ((FCurData[1] >= '0') and (FCurData[1] <= '9')) or ((FCurData[1] = '.') and not HasDot) do
    begin
      Inc(FCurData);
      Inc(FTokenLength);
      if FCurData[0] = '.' then
        HasDot := True;
    end;
    Result := tkNumber;
  end;

var
  Delim: WideChar;
begin
  // Skip whitespace
  while (FCurData[0] < #255) and (char(ord(FCurData[0])) in [#9, #10, #13, ' ']) do
    Inc(FCurData);

  FTokenStart := FCurData;
  FTokenLength := 0;
  Result := tkInvalid;

  case FCurData[0] of
    #0:
      Result := tkEndOfStream;
    '!':
      if FCurData[1] = '=' then
      begin
        Inc(FCurData);
        Result := tkNotEqual;
      end;
    '"', '''':
      begin
        Delim := FCurData^;
        Inc(FCurData);
        FTokenStart := FCurData;
        while FCurData[0] <> Delim do
        begin
          if FCurData[0] = #0 then
            Error(SScannerUnclosedString);
          Inc(FCurData);
        end;
        FTokenLength := FCurData-FTokenStart;
        Result := tkString;
      end;
    '$':
      begin
        Inc(FCurData);
        Inc(FTokenStart);
        if ScanQName then
          Result := tkVariable
        else
          Error(SScannerExpectedVarName);
        Exit;
      end;
    '(':
      Result := tkLeftBracket;
    ')':
      Result := tkRightBracket;
    '*':
      Result := tkAsterisk;
    '+':
      Result := tkPlus;
    ',':
      Result := tkComma;
    '-':
      Result := tkMinus;
    '.':
      if FCurData[1] = '.' then
      begin
        Inc(FCurData);
        Result := tkDotDot;
      end else if (FCurData[1] >= '0') and (FCurData[1] <= '9') then
        GetNumber(True)
      else
        Result := tkDot;
    '/':
      if FCurData[1] = '/' then
      begin
        Inc(FCurData);
        Result := tkSlashSlash;
      end else
        Result := tkSlash;
    '0'..'9':
      GetNumber(False);
    ':':
      if FCurData[1] = ':' then
      begin
        Inc(FCurData);
        Result := tkColonColon;
      end;
    '<':
      if FCurData[1] = '=' then
      begin
        Inc(FCurData);
        Result := tkLessEqual;
      end else
        Result := tkLess;
    '=':
      Result := tkEqual;
    '>':
      if FCurData[1] = '=' then
      begin
        Inc(FCurData);
        Result := tkGreaterEqual;
      end else
        Result := tkGreater;
    '@':
      Result := tkAt;
    '[':
      Result := tkLeftSquareBracket;
    ']':
      Result := tkRightSquareBracket;
    '|':
      Result := tkPipe;
  else
    if ScanQName then
    begin
      Result := tkIdentifier;
      Exit;
    end
    else if FPrefixLength > 0 then
    begin
      if FCurData^ = '*'  then
      begin
        Inc(FCurData);
        Dec(FTokenLength);        // exclude ':'
        Result := tkNSNameTest;
        Exit;
      end
      else
        Error(SScannerMalformedQName);
    end;
  end;

  if Result = tkInvalid then
    Error(SScannerInvalidChar);
  // We have processed at least one character now; eat it:
  if Result > tkEndOfStream then
    Inc(FCurData);
end;

procedure TXPathScanner.Error(const Msg: String);
begin
  raise Exception.Create(Msg) at get_caller_addr(get_frame), get_caller_frame(get_frame);
end;

procedure TXPathScanner.ParsePredicates(var Dest: TXPathNodeArray);
var
  Buffer: array[0..15] of TXPathExprNode;
  I: Integer;
begin
  I := 0;
  // accumulate nodes in local buffer, then add all at once
  // this reduces amount of ReallocMem's
  while SkipToken(tkLeftSquareBracket) do
  begin
    Buffer[I] := ParseOrExpr;
    Inc(I);
    if I > High(Buffer) then
      AddNodes(Dest, Buffer, I);  // will reset I to zero
    if not SkipToken(tkRightSquareBracket) then
      Error(SParserExpectedRightSquareBracket);
  end;
  AddNodes(Dest, Buffer, I);
end;

function TXPathScanner.ParseStep: TStep;  // [4]
var
  Axis: TAxis;
begin
  if CurToken = tkDot then          // [12] Abbreviated step, first case
  begin
    NextToken;
    Result := TStep.Create(axisSelf, ntAnyNode);
  end
  else if CurToken = tkDotDot then  // [12] Abbreviated step, second case
  begin
    NextToken;
    Result := TStep.Create(axisParent, ntAnyNode);
  end
  else		// Parse [5] AxisSpecifier
  begin
    if CurToken = tkAt then         // [13] AbbreviatedAxisSpecifier
    begin
      Axis := axisAttribute;
      NextToken;
    end
    else if (CurToken = tkIdentifier) and (PeekToken = tkColonColon) then  // [5] AxisName '::'
    begin
      if FTokenId in AxisNameKeywords then
        Axis := AxisNameMap[FTokenId]
      else
        Error(SParserBadAxisName);
      NextToken;  // skip identifier and the '::'
      NextToken;
    end
    else
      Axis := axisChild;

    Result := ParseNodeTest(Axis);
    ParsePredicates(Result.FPredicates);
  end;
end;

function TXPathScanner.ParseNodeTest(Axis: TAxis): TStep; // [7]
var
  nodeType: TNodeTestType;
  nodeName: DOMString;
  nsURI: DOMString;
begin
  nodeName := '';
  nsURI := '';
  if CurToken = tkAsterisk then   // [37] NameTest, first case
  begin
    nodeType := ntAnyPrincipal;
    NextToken;
  end
  else if CurToken = tkNSNameTest then // [37] NameTest, second case
  begin
    if Assigned(FResolver) then
      nsURI := FResolver.lookupNamespaceURI(CurTokenString);
    if nsURI = '' then
      // !! localization disrupted by DOM exception specifics
      raise EDOMNamespace.Create('TXPathScanner.ParseStep');
    NextToken;
    nodeType := ntName;
  end
  else if CurToken = tkIdentifier then
  begin
    // Check for case [38] NodeType
    if PeekToken = tkLeftBracket then
    begin
      if FTokenId in NodeTestKeywords then
      begin
        nodeType := NodeTestMap[FTokenId];
        if FTokenId = xkProcessingInstruction then
        begin
          NextToken;
          if NextToken = tkString then
          begin
            nodeName := CurTokenString;
            NextToken;
          end;
        end
        else
        begin
          NextToken;
          NextToken;
        end;
        if CurToken <> tkRightBracket then
          Error(SParserExpectedRightBracket);
        NextToken;
      end
      else
        Error(SParserBadNodeType);
    end
    else  // [37] NameTest, third case
    begin
      nodeType := ntName;
      if FPrefixLength > 0 then
      begin
        if Assigned(FResolver) then
          nsURI := FResolver.lookupNamespaceURI(Copy(CurTokenString, 1, FPrefixLength));
        if nsURI = '' then
          raise EDOMNamespace.Create('TXPathScanner.ParseStep');
        nodeName := Copy(CurTokenString, FPrefixLength+2, MaxInt);
      end
      else
        nodeName := CurTokenString;
      NextToken;
    end;
  end
  else
    Error(SParserInvalidNodeTest);

  Result := TStep.Create(Axis, nodeType);
  Result.NodeTestString := nodeName;
  Result.NSTestString := nsURI;
end;

function TXPathScanner.ParsePrimaryExpr: TXPathExprNode;  // [15]
begin
  case CurToken of
    tkVariable:         // [36] Variable reference
        Result := TXPathVariableNode.Create(CurTokenString);
    tkLeftBracket:
      begin
        NextToken;
        Result := ParseOrExpr;
        if CurToken <> tkRightBracket then
          Error(SParserExpectedRightBracket);
      end;
    tkString:         // [29] Literal
      Result := TXPathConstantNode.Create(
        TXPathStringVariable.Create(CurTokenString));
    tkNumber:         // [30] Number
      Result := TXPathConstantNode.Create(
        TXPathNumberVariable.Create(StrToNumber(CurTokenString)));
    tkIdentifier:     // [16] Function call
      Result := ParseFunctionCall;
  else
    Error(SParserInvalidPrimExpr);
    Result := nil; // satisfy compiler
  end;
  NextToken;
end;

function TXPathScanner.ParseFunctionCall: TXPathExprNode;
var
  Name: DOMString;
  Args: TXPathNodeArray;
  Buffer: array[0..15] of TXPathExprNode;
  I: Integer;
begin
  Name := CurTokenString;
  I := 0;
  if NextToken <> tkLeftBracket then
    Error(SParserExpectedLeftBracket);
  NextToken;
  // Parse argument list
  if CurToken <> tkRightBracket then
  repeat
    Buffer[I] := ParseOrExpr;
    Inc(I);
    if I > High(Buffer) then
      AddNodes(Args, Buffer, I);
  until not SkipToken(tkComma);
  if CurToken <> tkRightBracket then
    Error(SParserExpectedRightBracket);

  AddNodes(Args, Buffer, I);
  Result := TXPathFunctionNode.Create(Name, Args);
end;

function TXPathScanner.ParseUnionExpr: TXPathExprNode;  // [18]
begin
  Result := ParsePathExpr;
  while SkipToken(tkPipe) do
    Result := TXPathUnionNode.Create(Result, ParsePathExpr);
end;

function AddStep(Left: TXPathExprNode; Right: TStep): TXPathExprNode;
begin
  Right.FLeft := Left;
  Result := Right;
end;

function TXPathScanner.ParsePathExpr: TXPathExprNode;  // [19]
var
  tok: TXPathToken;
begin
  Result := nil;
  // Try to detect whether a LocationPath [1] or a FilterExpr [20] follows
  if ((CurToken = tkIdentifier) and (PeekToken = tkLeftBracket) and
    not (FTokenId in NodeTestKeywords)) or
    (CurToken in [tkVariable, tkLeftBracket, tkString, tkNumber]) then
  begin
    // second, third or fourth case of [19]
    Result := ParseFilterExpr;
    if SkipToken(tkSlash) then { do nothing }
    else if SkipToken(tkSlashSlash) then
      Result := AddStep(Result, TStep.Create(axisDescendantOrSelf, ntAnyNode))
    else
      Exit;
  end
  else if CurToken in [tkSlash, tkSlashSlash] then
  begin
    tok := CurToken;
    NextToken;
    Result := TStep.Create(axisRoot, ntAnyNode);
    if tok = tkSlashSlash then
      Result := AddStep(Result, TStep.Create(axisDescendantOrSelf, ntAnyNode))
    else if not (CurToken in [tkDot, tkDotDot, tkAt, tkAsterisk, tkIdentifier, tkNSNameTest]) then
      Exit;  // allow '/' alone
  end;

  // Continue with parsing of [3] RelativeLocationPath
  repeat
    Result := AddStep(Result, ParseStep);
    if CurToken = tkSlashSlash then
    begin
      NextToken;
      // Found abbreviated step ("//" for "descendant-or-self::node()")
      Result := AddStep(Result, TStep.Create(axisDescendantOrSelf, ntAnyNode));
    end
    else if not SkipToken(tkSlash) then
      break;
  until False;
end;

function TXPathScanner.ParseFilterExpr: TXPathExprNode;  // [20]
begin
  Result := ParsePrimaryExpr;
  // Parse predicates
  if CurToken = tkLeftSquareBracket then
  begin
    Result := TXPathFilterNode.Create(Result);
    ParsePredicates(TXPathFilterNode(Result).FPredicates);
  end;
end;

function TXPathScanner.ParseOrExpr: TXPathExprNode;  // [21]
begin
  Result := ParseAndExpr;
  while FTokenId = xkOr do
  begin
    NextToken;
    Result := TXPathBooleanOpNode.Create(opOr, Result, ParseAndExpr);
  end;
end;

function TXPathScanner.ParseAndExpr: TXPathExprNode;  // [22]
begin
  Result := ParseEqualityExpr;
  while FTokenId = xkAnd do
  begin
    NextToken;
    Result := TXPathBooleanOpNode.Create(opAnd, Result, ParseEqualityExpr);
  end;
end;

function TXPathScanner.ParseEqualityExpr: TXPathExprNode;  // [23]
var
  op: TXPathCompareOp;
begin
  Result := ParseRelationalExpr;
  repeat
    case CurToken of
      tkEqual:    op := opEqual;
      tkNotEqual: op := opNotEqual;
    else
      Break;
    end;
    NextToken;
    Result := TXPathCompareNode.Create(op, Result, ParseRelationalExpr);
  until False;
end;

function TXPathScanner.ParseRelationalExpr: TXPathExprNode;  // [24]
var
  op: TXPathCompareOp;
begin
  Result := ParseAdditiveExpr;
  repeat
    case CurToken of
      tkLess:      op := opLess;
      tkLessEqual: op := opLessEqual;
      tkGreater:   op := opGreater;
      tkGreaterEqual: op := opGreaterEqual;
    else
      Break;
    end;
    NextToken;
    Result := TXPathCompareNode.Create(op, Result, ParseAdditiveExpr);
  until False;
end;

function TXPathScanner.ParseAdditiveExpr: TXPathExprNode;  // [25]
var
  op: TXPathMathOp;
begin
  Result := ParseMultiplicativeExpr;
  repeat
    case CurToken of
      tkPlus: op := opAdd;
      tkMinus: op := opSubtract;
    else
      Break;
    end;
    NextToken;
    Result := TXPathMathOpNode.Create(op, Result, ParseMultiplicativeExpr);
  until False;
end;

function TXPathScanner.ParseMultiplicativeExpr: TXPathExprNode;  // [26]
var
  op: TXPathMathOp;
begin
  Result := ParseUnaryExpr;
  repeat
    case CurToken of
      tkAsterisk:
        op := opMultiply;
      tkIdentifier:
        if FTokenId = xkDiv then
          op := opDivide
        else if FTokenId = xkMod then
          op := opMod
        else
          break;
    else
      break;
    end;
    NextToken;
    Result := TXPathMathOpNode.Create(op, Result, ParseUnaryExpr);
  until False;
end;

function TXPathScanner.ParseUnaryExpr: TXPathExprNode;  // [27]
var
  NegCount: Integer;
begin
  NegCount := 0;
  while SkipToken(tkMinus) do
    Inc(NegCount);
  Result := ParseUnionExpr;

  if Odd(NegCount) then
    Result := TXPathNegationNode.Create(Result);
end;

{ TXPathContext }

constructor TXPathContext.Create(AContextNode: TDOMNode;
  AContextPosition, AContextSize: Integer);
begin
  inherited Create;
  ContextNode := AContextNode;
  ContextPosition := AContextPosition;
  ContextSize := AContextSize;
end;


{ TXPathEnvironment }

type
  PFunctionInfo = ^TFunctionInfo;
  TFunctionInfo = record
    Name: String;
    Fn: TXPathFunction;
  end;

  PVariableInfo = ^TVariableInfo;
  TVariableInfo = record
    Name: String;
    Variable: TXPathVariable;
  end;


constructor TXPathEnvironment.Create;
begin
  inherited Create;
  FFunctions := TFPList.Create;
  FVariables := TFPList.Create;

  // Add the functions of the XPath Core Function Library

  // Node set functions
  AddFunction('last', @xpLast);
  AddFunction('position', @xpPosition);
  AddFunction('count', @xpCount);
  AddFunction('id', @xpId);
  AddFunction('local-name', @xpLocalName);
  AddFunction('namespace-uri', @xpNamespaceURI);
  AddFunction('name', @xpName);
  // String functions
  AddFunction('string', @xpString);
  AddFunction('concat', @xpConcat);
  AddFunction('starts-with', @xpStartsWith);
  AddFunction('contains', @xpContains);
  AddFunction('substring-before', @xpSubstringBefore);
  AddFunction('substring-after', @xpSubstringAfter);
  AddFunction('substring', @xpSubstring);
  AddFunction('string-length', @xpStringLength);
  AddFunction('normalize-space', @xpNormalizeSpace);
  AddFunction('translate', @xpTranslate);
  // Boolean functions
  AddFunction('boolean', @xpBoolean);
  AddFunction('not', @xpNot);
  AddFunction('true', @xpTrue);
  AddFunction('false', @xpFalse);
  AddFunction('lang', @xpLang);
  // Number functions
  AddFunction('number', @xpNumber);
  AddFunction('sum', @xpSum);
  AddFunction('floor', @xpFloor);
  AddFunction('ceiling', @xpCeiling);
  AddFunction('round', @xpRound);
end;

destructor TXPathEnvironment.Destroy;
var
  i: Integer;
  FunctionInfo: PFunctionInfo;
  VariableInfo: PVariableInfo;
begin
  for i := 0 to FFunctions.Count - 1 do
  begin
    FunctionInfo := PFunctionInfo(FFunctions[i]);
    FreeMem(FunctionInfo);
  end;
  FFunctions.Free;
  for i := 0 to FVariables.Count - 1 do
  begin
    VariableInfo := PVariableInfo(FVariables[i]);
    FreeMem(VariableInfo);
  end;
  FVariables.Free;
  inherited Destroy;
end;

function TXPathEnvironment.GetFunctionIndex(const AName: String): Integer;
var
  i: Integer;
begin
  for i := 0 to FFunctions.Count - 1 do
    if PFunctionInfo(FFunctions[i])^.Name = AName then
    begin
      Result := i;
      exit;
    end;
  Result := -1;
end;

function TXPathEnvironment.GetVariableIndex(const AName: String): Integer;
var
  i: Integer;
begin
  for i := 0 to FVariables.Count - 1 do
    if PVariableInfo(FFunctions[i])^.Name = AName then
    begin
      Result := i;
      exit;
    end;
  Result := -1;
end;

procedure TXPathEnvironment.AddFunction(const AName: String; AFunction: TXPathFunction);
var
  NewFunctionInfo: PFunctionInfo;
begin
  // !!!: Prevent the addition of duplicate functions
  New(NewFunctionInfo);
  NewFunctionInfo^.Name := AName;
  NewFunctionInfo^.Fn := AFunction;
  FFunctions.Add(NewFunctionInfo);
end;

procedure TXPathEnvironment.AddVariable(const AName: String; AVariable: TXPathVariable);
var
  NewVariableInfo: PVariableInfo;
begin
  // !!!: Prevent the addition of duplicate variables
  New(NewVariableInfo);
  NewVariableInfo^.Name := AName;
  NewVariableInfo^.Variable := AVariable;
  FVariables.Add(NewVariableInfo);
end;

procedure TXPathEnvironment.RemoveFunction(Index: Integer);
var
  FunctionInfo: PFunctionInfo;
begin
  FunctionInfo := PFunctionInfo(FFunctions[Index]);
  Dispose(FunctionInfo);
  FFunctions.Delete(Index);
end;

procedure TXPathEnvironment.RemoveFunction(const AName: String);
var
  i: Integer;
begin
  for i := 0 to FFunctions.Count - 1 do
    if PFunctionInfo(FFunctions[i])^.Name = AName then
    begin
      RemoveFunction(i);
      exit;
    end;
end;

procedure TXPathEnvironment.RemoveVariable(Index: Integer);
var
  VariableInfo: PVariableInfo;
begin
  VariableInfo := PVariableInfo(FVariables[Index]);
  Dispose(VariableInfo);
  FVariables.Delete(Index);
end;

procedure TXPathEnvironment.RemoveVariable(const AName: String);
var
  Index: Integer;
begin
  Index := GetVariableIndex(AName);
  if Index >= 0 then
    RemoveVariable(Index);
end;

function TXPathEnvironment.GetFunctionCount: Integer;
begin
  Result := FFunctions.Count;
end;

function TXPathEnvironment.GetVariableCount: Integer;
begin
  Result := FVariables.Count;
end;

function TXPathEnvironment.GetFunction(Index: Integer): TXPathFunction;
begin
  Result := PFunctionInfo(FFunctions[Index])^.Fn;
end;

function TXPathEnvironment.GetFunction(const AName: String): TXPathFunction;
var
  i: Integer;
begin
  for i := 0 to FFunctions.Count - 1 do
    if PFunctionInfo(FFunctions[i])^.Name = AName then
    begin
      Result := PFunctionInfo(FFunctions[i])^.Fn;
      exit;
    end;
  Result := nil;
end;

function TXPathEnvironment.GetVariable(Index: Integer): TXPathVariable;
begin
  Result := PVariableInfo(FVariables[Index])^.Variable;
end;

function TXPathEnvironment.GetVariable(const AName: String): TXPathVariable;
var
  i: Integer;
begin
  for i := 0 to FVariables.Count - 1 do
    if PFunctionInfo(FVariables[i])^.Name = AName then
    begin
      Result := PVariableInfo(FVariables[i])^.Variable;
      exit;
    end;
  Result := nil;
end;

function TXPathEnvironment.xpLast(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
begin
  if Args.Count <> 0 then
    EvaluationError(SEvalInvalidArgCount);
  Result := TXPathNumberVariable.Create(Context.ContextSize);
end;

function TXPathEnvironment.xpPosition(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
begin
  if Args.Count <> 0 then
    EvaluationError(SEvalInvalidArgCount);
  Result := TXPathNumberVariable.Create(Context.ContextPosition);
end;

function TXPathEnvironment.xpCount(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
begin
  if Args.Count <> 1 then
    EvaluationError(SEvalInvalidArgCount);
  Result := TXPathNumberVariable.Create(TXPathVariable(Args[0]).AsNodeSet.Count);
end;

function TXPathEnvironment.xpId(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  i: Integer;
  ResultSet: TNodeSet;
  TheArg: TXPathVariable;
  doc: TDOMDocument;

  procedure AddId(ns: TNodeSet; const s: DOMString);
  var
    Head, Tail, L: Integer;
    Token: DOMString;
    Element: TDOMNode;
  begin
    Head := 1;
    L := Length(s);

    while Head <= L do
    begin
      while (Head <= L) and IsXmlWhiteSpace(s[Head]) do
        Inc(Head);

      Tail := Head;
      while (Tail <= L) and not IsXmlWhiteSpace(s[Tail]) do
        Inc(Tail);
      SetString(Token, @s[Head], Tail - Head);
      Element := doc.GetElementById(Token);
      if Assigned(Element) then
        ns.Add(Element);

      Head := Tail;
    end;
  end;

begin
  if Args.Count <> 1 then
    EvaluationError(SEvalInvalidArgCount);
  // TODO: probably have doc as member of Context
  if Context.ContextNode.NodeType = DOCUMENT_NODE then
    doc := TDOMDocument(Context.ContextNode)
  else
    doc := Context.ContextNode.OwnerDocument;

  ResultSet := TNodeSet.Create;
  TheArg := TXPathVariable(Args[0]);
  if TheArg is TXPathNodeSetVariable then
  begin
    with TheArg.AsNodeSet do
      for i := 0 to Count-1 do
        AddId(ResultSet, NodeToText(TDOMNode(Items[i])));
  end
  else
    AddId(ResultSet, TheArg.AsText);
  Result := TXPathNodeSetVariable.Create(ResultSet);
end;

function TXPathEnvironment.xpLocalName(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  n: TDOMNode;
  NodeSet: TNodeSet;
  s: DOMString;
begin
  if Args.Count > 1 then
    EvaluationError(SEvalInvalidArgCount);
  n := nil;
  if Args.Count = 0 then
    n := Context.ContextNode
  else
  begin
    NodeSet := TXPathVariable(Args[0]).AsNodeSet;
    if NodeSet.Count > 0 then
      n := TDOMNode(NodeSet[0]);
  end;
  s := '';
  if Assigned(n) then
  begin
    case n.NodeType of
      ELEMENT_NODE,ATTRIBUTE_NODE:
        with TDOMNode_NS(n) do
          s := Copy(NSI.QName^.Key, NSI.PrefixLen+1, MaxInt);
      PROCESSING_INSTRUCTION_NODE:
        s := TDOMProcessingInstruction(n).Target;
      // TODO: NAMESPACE_NODE: must return prefix part
    end;
  end;
  Result := TXPathStringVariable.Create(s);
end;

function TXPathEnvironment.xpNamespaceURI(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  n: TDOMNode;
  NodeSet: TNodeSet;
  s: DOMString;
begin
  if Args.Count > 1 then
    EvaluationError(SEvalInvalidArgCount);
  n := nil;
  if Args.Count = 0 then
    n := Context.ContextNode
  else
  begin
    NodeSet := TXPathVariable(Args[0]).AsNodeSet;
    if NodeSet.Count > 0 then
      n := TDOMNode(NodeSet[0]);
  end;
  if Assigned(n) then
    s := n.namespaceUri
  else
    s := '';
  Result := TXPathStringVariable.Create(s);
end;

function TXPathEnvironment.xpName(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  n: TDOMNode;
  NodeSet: TNodeSet;
  s: DOMString;
begin
  if Args.Count > 1 then
    EvaluationError(SEvalInvalidArgCount);
  n := nil;
  if Args.Count = 0 then
    n := Context.ContextNode
  else
  begin
    NodeSet := TXPathVariable(Args[0]).AsNodeSet;
    if NodeSet.Count > 0 then
      n := TDOMNode(NodeSet[0]);
  end;
  s := '';
  if Assigned(n) then
  begin
    case n.NodeType of
      ELEMENT_NODE,ATTRIBUTE_NODE:
        s := TDOMNode_NS(n).NSI.QName^.Key;
      PROCESSING_INSTRUCTION_NODE:
        s := TDOMProcessingInstruction(n).Target;
      // TODO: NAMESPACE_NODE: must return prefix part
    end;
  end;
  Result := TXPathStringVariable.Create(s);
end;

function TXPathEnvironment.xpString(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  s: DOMString;
begin
  if Args.Count > 1 then
    EvaluationError(SEvalInvalidArgCount);
  if Args.Count = 0 then
    s := NodeToText(Context.ContextNode)
  else
    s := TXPathVariable(Args[0]).AsText;
  Result := TXPathStringVariable.Create(s);
end;

function TXPathEnvironment.xpConcat(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  i: Integer;
  s: DOMString;
begin
  if Args.Count < 2 then
    EvaluationError(SEvalInvalidArgCount);
  SetLength(s, 0);
  for i := 0 to Args.Count - 1 do
    s := s + TXPathVariable(Args[i]).AsText;
  Result := TXPathStringVariable.Create(s);
end;

function TXPathEnvironment.xpStartsWith(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  s1, s2: DOMString;
  res: Boolean;
begin
  if Args.Count <> 2 then
    EvaluationError(SEvalInvalidArgCount);
  s1 := TXPathVariable(Args[0]).AsText;
  s2 := TXPathVariable(Args[1]).AsText;
  if s2 = '' then
    res := True
  else
    res := Pos(s2, s1) = 1;
  Result := TXPathBooleanVariable.Create(res);
end;

function TXPathEnvironment.xpContains(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  s1, s2: DOMString;
  res: Boolean;
begin
  if Args.Count <> 2 then
    EvaluationError(SEvalInvalidArgCount);
  s1 := TXPathVariable(Args[0]).AsText;
  s2 := TXPathVariable(Args[1]).AsText;
  if s2 = '' then
    res := True
  else
    res := Pos(s2, s1) <> 0;
  Result := TXPathBooleanVariable.Create(res);
end;

function TXPathEnvironment.xpSubstringBefore(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  s, substr: DOMString;
begin
  if Args.Count <> 2 then
    EvaluationError(SEvalInvalidArgCount);
  s := TXPathVariable(Args[0]).AsText;
  substr := TXPathVariable(Args[1]).AsText;
  Result := TXPathStringVariable.Create(Copy(s, 1, Pos(substr, s)-1));
end;

function TXPathEnvironment.xpSubstringAfter(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  s, substr: DOMString;
  i: Integer;
begin
  if Args.Count <> 2 then
    EvaluationError(SEvalInvalidArgCount);
  s := TXPathVariable(Args[0]).AsText;
  substr := TXPathVariable(Args[1]).AsText;
  i := Pos(substr, s);
  if i <> 0 then
    Result := TXPathStringVariable.Create(Copy(s, i + Length(substr), MaxInt))
  else
    Result := TXPathStringVariable.Create('');
end;

function TXPathEnvironment.xpSubstring(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  s: DOMString;
  i, n1, n2: Integer;
  e1, e2: Extended;
  empty: Boolean;
begin
  if (Args.Count < 2) or (Args.Count > 3) then
    EvaluationError(SEvalInvalidArgCount);
  s := TXPathVariable(Args[0]).AsText;
  e1 := TXPathVariable(Args[1]).AsNumber;
  n1 := 1;  // satisfy compiler
  n2 := MaxInt;
  empty := IsNaN(e1) or IsInfinite(e1);
  if not empty then
    n1 := floor(0.5 + e1);
  if Args.Count = 3 then
  begin
    e2 := TXPathVariable(Args[2]).AsNumber;
    if IsNaN(e2) or (IsInfinite(e2) and (e2 < 0)) then
      empty := True
    else if not IsInfinite(e2) then
      n2 := floor(0.5 + e2);
  end;
  i := Max(n1, 1);
  if empty then
    n2 := -1
  else if n2 < MaxInt then
    n2 := n2 + (n1 - i);
  Result := TXPathStringVariable.Create(Copy(s, i, n2));
end;

function TXPathEnvironment.xpStringLength(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  s: DOMString;
begin
  if Args.Count > 1 then
    EvaluationError(SEvalInvalidArgCount);
  if Args.Count = 0 then
    s := NodeToText(Context.ContextNode)
  else
    s := TXPathVariable(Args[0]).AsText;
  Result := TXPathNumberVariable.Create(Length(s));
end;

function TXPathEnvironment.xpNormalizeSpace(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  s: DOMString;
  p: DOMPChar;
  i: Integer;
begin
  if Args.Count > 1 then
    EvaluationError(SEvalInvalidArgCount);
  if Args.Count = 0 then
    s := NodeToText(Context.ContextNode)
  else
    s := TXPathVariable(Args[0]).AsText;
  UniqueString(s);
  p := DOMPChar(s);
  for i := 1 to Length(s) do
  begin
    if (p^ = #10) or (p^ = #13) or (p^ = #9) then
      p^ := #32;
    Inc(p);  
  end;
  NormalizeSpaces(s);
  Result := TXPathStringVariable.Create(s);
end;

function TXPathEnvironment.xpTranslate(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  S: DOMString;
begin
  if Args.Count <> 3 then
    EvaluationError(SEvalInvalidArgCount);
  S := TXPathVariable(Args[0]).AsText;
  TranslateWideString(S, TXPathVariable(Args[1]).AsText, TXPathVariable(Args[2]).AsText);
  Result := TXPathStringVariable.Create(S);
end;

function TXPathEnvironment.xpBoolean(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
begin
  if Args.Count <> 1 then
    EvaluationError(SEvalInvalidArgCount);
  Result := TXPathBooleanVariable.Create(TXPathVariable(Args[0]).AsBoolean);
end;

function TXPathEnvironment.xpNot(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
begin
  if Args.Count <> 1 then
    EvaluationError(SEvalInvalidArgCount);
  Result := TXPathBooleanVariable.Create(not TXPathVariable(Args[0]).AsBoolean);
end;

function TXPathEnvironment.xpTrue(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
begin
  if Args.Count <> 0 then
    EvaluationError(SEvalInvalidArgCount);
  Result := TXPathBooleanVariable.Create(True);
end;

function TXPathEnvironment.xpFalse(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
begin
  if Args.Count <> 0 then
    EvaluationError(SEvalInvalidArgCount);
  Result := TXPathBooleanVariable.Create(False);
end;

function TXPathEnvironment.xpLang(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  L: Integer;
  TheArg, NodeLang: DOMString;
  res: Boolean;
begin
  if Args.Count <> 1 then
    EvaluationError(SEvalInvalidArgCount);
  TheArg := TXPathVariable(Args[0]).AsText;
  NodeLang := GetNodeLanguage(Context.ContextNode);

  L := Length(TheArg);
  res := (L <= Length(NodeLang)) and
    (WStrLIComp(DOMPChar(NodeLang), DOMPChar(TheArg), L) = 0) and
    ((L = Length(NodeLang)) or (NodeLang[L+1] = '-'));

  Result := TXPathBooleanVariable.Create(res);
end;

function TXPathEnvironment.xpNumber(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
begin
  if Args.Count > 1 then
    EvaluationError(SEvalInvalidArgCount);
  if Args.Count = 0 then
    Result := TXPathNumberVariable.Create(StrToNumber(NodeToText(Context.ContextNode)))
  else
    Result := TXPathNumberVariable.Create(TXPathVariable(Args[0]).AsNumber);
end;

function TXPathEnvironment.xpSum(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  i: Integer;
  ns: TNodeSet;
  sum: Extended;
begin
  if Args.Count <> 1 then
    EvaluationError(SEvalInvalidArgCount);
  ns := TXPathVariable(Args[0]).AsNodeSet;
  sum := 0.0;
  for i := 0 to ns.Count-1 do
    sum := sum + StrToNumber(NodeToText(TDOMNode(ns[i])));
  Result := TXPathNumberVariable.Create(sum);
end;

function TXPathEnvironment.xpFloor(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  n: Extended;
begin
  if Args.Count <> 1 then
    EvaluationError(SEvalInvalidArgCount);
  n := TXPathVariable(Args[0]).AsNumber;
  if not IsNan(n) then
    n := floor(n);
  Result := TXPathNumberVariable.Create(n);
end;

function TXPathEnvironment.xpCeiling(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  n: Extended;
begin
  if Args.Count <> 1 then
    EvaluationError(SEvalInvalidArgCount);
  n := TXPathVariable(Args[0]).AsNumber;
  if not IsNan(n) then
    n := ceil(n);
  Result := TXPathNumberVariable.Create(n);
end;

function TXPathEnvironment.xpRound(Context: TXPathContext; Args: TXPathVarList): TXPathVariable;
var
  num: Extended;
begin
  if Args.Count <> 1 then
    EvaluationError(SEvalInvalidArgCount);
  num := TXPathVariable(Args[0]).AsNumber;
  if not (IsNan(num) or IsInfinite(num)) then
    num := floor(0.5 + num);
  Result := TXPathNumberVariable.Create(num);
end;

{ TXPathNSResolver }

constructor TXPathNSResolver.Create(aNode: TDOMNode);
begin
  inherited Create;
  FNode := aNode;
end;

function TXPathNSResolver.LookupNamespaceURI(const aPrefix: DOMString): DOMString;
begin
  if assigned(FNode) then
    result := FNode.LookupNamespaceURI(aPrefix)
  else
    result := '';
end;

{ TXPathExpression }

constructor TXPathExpression.Create(AScanner: TXPathScanner;
  CompleteExpression: Boolean; AResolver: TXPathNSResolver);
begin
  inherited Create;
  AScanner.FResolver := AResolver;
  FRootNode := AScanner.ParseOrExpr;
  if CompleteExpression and (AScanner.CurToken <> tkEndOfStream) then
    EvaluationError(SParserGarbageAfterExpression);
end;

function TXPathExpression.Evaluate(AContextNode: TDOMNode): TXPathVariable;
var
  Environment: TXPathEnvironment;
begin
  Environment := TXPathEnvironment.Create;
  try
    Result := Evaluate(AContextNode, Environment);
  finally
    Environment.Free;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
destructor TXPathExpression.Destroy;
begin
  FRootNode.Free;
  inherited Destroy;
=======
destructor TXPathExpression.destroy;
begin
 FRootNode.free;
>>>>>>> graemeg/fixes_2_2
=======
destructor TXPathExpression.destroy;
begin
 FRootNode.free;
>>>>>>> origin/fixes_2_2
end;

function TXPathExpression.Evaluate(AContextNode: TDOMNode;
  AEnvironment: TXPathEnvironment): TXPathVariable;
var
  Context: TXPathContext;
  mask: TFPUExceptionMask;
begin
  if Assigned(FRootNode) then
  begin
    mask := GetExceptionMask;
    SetExceptionMask(mask + [exInvalidOp, exZeroDivide]);
    Context := TXPathContext.Create(AContextNode, 1, 1);
    try
      Result := FRootNode.Evaluate(Context, AEnvironment);
    finally
      Context.Free;
      SetExceptionMask(mask);
    end;
  end else
    Result := nil;
end;

function EvaluateXPathExpression(const AExpressionString: DOMString;
  AContextNode: TDOMNode; AResolver: TXPathNSResolver): TXPathVariable;
var
  Scanner: TXPathScanner;
  Expression: TXPathExpression;
begin
  Scanner := TXPathScanner.Create(AExpressionString);
  try
    Expression := TXPathExpression.Create(Scanner, True, AResolver);
    try
      Result := Expression.Evaluate(AContextNode);
    finally
      Expression.Free;
    end;
  finally
    Scanner.Free;
  end;
end;

end.
