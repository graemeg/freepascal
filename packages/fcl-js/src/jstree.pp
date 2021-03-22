{ ********************************************************************* 
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2016 Michael Van Canneyt.
       
    Javascript syntax tree definitions
            
    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.
                   
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
                                
  **********************************************************************}
unit jstree;

{$mode objfpc}{$H+}
{ $DEFINE NOCLASSES}

interface

uses
{$IFNDEF NOCLASSES}  Classes, {$ENDIF} SysUtils, jsbase, jstoken;

Type
{$IFDEF NOCLASSES}

  { TStrings }
{$M+}
  TStrings = Class(TObject)
  private
    FCount: Integer;
    FStrings : Array of String;
    function GetCount: Integer;
    function GetS(AIndex : Integer): String;
    procedure SetS(AIndex : Integer; AValue: String);
  Public
    Function Add(Const S : String) : Integer;
    Procedure Assign(Source : TStrings);
    Property Strings[AIndex : Integer] : String Read GetS Write SetS; default;
    Property Count : Integer Read GetCount;
  end;
  TStringList = TStrings;
  EListError = Class(Exception);

  TCollection = Class;

  { TCollectionItem }

  TCollectionItem = Class
  Private
    FCollection : TCollection;
  Public
    Constructor Create(ACollection : TCollection);
    Destructor Destroy; override;
  end;
  TCollectionItemClass = Class of TCollectionItem;

  { TCollection }

  TCollection = Class
  private
    FCount: Integer;
    FItems : Array of TCollectionItem;
    FItemClass : TCollectionItemClass;
    function GetCount: Integer;
    function GetI(AIndex : Integer): TCollectionItem;
  public
    Constructor Create(AItemClass : TCollectionItemClass);
    Destructor Destroy; override;
    Procedure Clear;
    Procedure Remove(AItem : TCollectionItem);
    Function Add : TCollectionItem;
    Property Items[AIndex : Integer] : TCollectionItem Read GetI;default;
    Property Count : Integer Read GetCount;
  end;

{$M-}
{$ENDIF}
  TJSElementFlag = (elIsConst,elIsConstValid);
  TJSElementFlags = set of TJSElementFlag;

  TJSFunctionBody = Class;

  { TJSElement }

  TJSObject = Class(TObject);


  { TJSLabelSet }

  TJSLabelSet = Class(TJSObject)
  private
    FCont: Boolean;
    FNext: TJSLabelSet;
    FTarget: Cardinal;
  Public
    Property Target : Cardinal Read FTarget Write FTarget;
    Property Next : TJSLabelSet Read FNext Write FNext; // Linked list
    Property Continuable : Boolean Read FCont Write FCont;
  end;

  { TJSLabel }

  TJSLabel = Class(TJSObject)
  private
    FLabelSet: TJSLabelSet;
    FLocationLine: Integer;
    FLocationPos: Integer;
    FLocationSource: String;
    FName: String;
    FNext: TJSLabel;
  Public
    Property Name : String Read FName Write FName;
    Property LabelSet : TJSLabelSet Read FLabelSet Write FLabelSet;
    Property LocationSource : String Read FLocationSource Write FLocationSource;
    Property LocationLine : Integer Read FLocationLine Write FLocationLine;
    Property LocationPos : Integer Read FLocationPos Write FLocationPos;
    Property Next : TJSLabel Read FNext Write FNext;
  end;

  TJSString = jsbase.TJSString; // beware of jstoken.tjsString

  { TJSFuncDef - part of TJSFunctionDeclarationStatement, e.g. 'function Name(Params)Body' }

  TJSFuncDef = Class(TJSObject)
  private
    FBody: TJSFunctionBody;
    FIsAsync: Boolean;
    FIsEmpty: Boolean;
    FName: TJSString;
    FParams: TStrings;
    procedure SetParams(const AValue: TStrings);
  Public
    Constructor Create;
    Destructor Destroy; override;
    Property Params : TStrings Read FParams Write SetParams;
    Property Body : TJSFunctionBody Read FBody Write FBody; // can be nil
    Property Name : TJSString Read FName Write FName;
    Property IsEmpty : Boolean Read FIsEmpty Write FIsEmpty;
    Property IsAsync : Boolean Read FIsAsync Write FIsAsync;
  end;

  { TJSElement }

  TJSElement = Class(TJSObject)
  private
    FFlags: TJSElementFlags;
    FLine: Integer;
    FColumn: Integer;
    FSource: String;
  Public
    Constructor Create(ALine,AColumn : Integer; Const ASource : String = ''); virtual;
    Procedure AssignPosition(El: TJSElement); virtual;
    Property Source : String Read FSource Write FSource;
    Property Line : Integer Read FLine Write FLine;
    Property Column : Integer Read FColumn Write FColumn;
    Property Flags : TJSElementFlags Read FFlags Write FFlags;
  end;
  TJSElementClass = Class of TJSElement;
  TJSElementArray = array of TJSElement;

  { TJSEmptyBlockStatement - empty curly brackets }

  TJSEmptyBlockStatement = Class(TJSElement);

  { TJSEmptyStatement - a dummy placeholder, needs sometimes a single semicolon }

  TJSEmptyStatement = Class(TJSElement);

  { TJSLiteral }

  TJSLiteral = Class(TJSElement)
  private
    FValue: TJSValue;
  Public
    Constructor Create(ALine,AColumn : Integer; Const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Value : TJSValue Read FValue Write FValue;
  end;

  { TJSRegularExpressionLiteral - /Pattern/PatternFlags }

  TJSRegularExpressionLiteral = Class(TJSElement)
  private
    FPattern: TJSValue;
    FPatternFlags: TJSValue;
    FArgv : Array[0..1] of TJSValue;
    function GetA(AIndex : integer): TJSValue;
    procedure SetA(AIndex : integer; const AValue: TJSValue);
  Public
    Constructor Create(ALine,AColumn : Integer; Const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Pattern : TJSValue Read FPattern Write FPattern;
    Property PatternFlags : TJSValue Read FPatternFlags Write FPatternFlags;
    Property Argv[AIndex : integer] : TJSValue Read GetA Write SetA;
  end;

  TJSPrimaryExpression = Class(TJSElement);

  TJSPrimaryExpressionThis = Class(TJSPrimaryExpression); // 'this'

  { TJSPrimaryExpressionIdent }

  TJSPrimaryExpressionIdent = Class(TJSPrimaryExpression)
  private
    FName: TJSString;
  Public
    Property Name : TJSString Read FName Write FName;
  end;

  { TJSArrayLiteralElement - an item of a TJSArrayLiteralElements }

  TJSArrayLiteralElement = Class(TCollectionItem)
  private
    FExpr: TJSelement;
    FElementIndex: Integer;
  Public
    Destructor Destroy; override;
    Property Expr : TJSElement Read FExpr Write FExpr;
    Property ElementIndex : Integer Read FElementIndex Write FElementIndex;
  end;

  { TJSArrayLiteralElements - Elements property of TJSArrayLiteral }

  TJSArrayLiteralElements = Class(TCollection)
  private
    function GetE(AIndex : Integer): TJSArrayLiteralElement;
  Public
    Function AddElement : TJSArrayLiteralElement;
    Property Elements[AIndex : Integer] : TJSArrayLiteralElement Read GetE ; default;
  end;

  { TJSArrayLiteral - [element1,...] or Args of a function }

  TJSArrayLiteral = Class(TJSElement)
  private
    FElements: TJSArrayLiteralElements;
  Public
    Constructor Create(ALine,AColumn : Integer; const ASource : String = ''); override;
    procedure AddElement(El: TJSElement);
    Destructor Destroy; override;
    Property Elements : TJSArrayLiteralElements Read FElements;
  end;

  { TJSObjectLiteralElement - an item of TJSObjectLiteralElements }

  TJSObjectLiteralElement = Class(TCollectionItem)
  private
    FExpr: TJSelement;
    FName: TJSString;
  Public
    Destructor Destroy; override;
    Property Expr : TJSElement Read FExpr Write FExpr;
    Property Name : TJSString Read FName Write FName;
  end;

  { TJSObjectLiteralElements - Elements property of TJSObjectLiteral }

  TJSObjectLiteralElements = Class(TCollection)
  private
    function GetE(AIndex : Integer): TJSObjectLiteralElement;
  Public
    Function AddElement : TJSObjectLiteralElement;
    Property Elements[AIndex : Integer] : TJSObjectLiteralElement Read GetE ; default;
  end;

  { TJSObjectLiteral }

  TJSObjectLiteral = Class(TJSElement)
  private
    FElements: TJSObjectLiteralElements;
  Public
    Constructor Create(ALine,AColumn : Integer; const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Elements : TJSObjectLiteralElements Read FElements;
  end;

  { TJSArguments - (element1,...) }

  TJSArguments = Class(TJSArrayLiteral);

  { TJSMemberExpression - base class }

  TJSMemberExpression = Class(TJSElement)
  private
    FMexpr: TJSElement;
  Public
    Destructor Destroy; override;
    Property MExpr : TJSElement Read FMexpr Write FMexpr;
  end;

  { TJSNewMemberExpression - e.g. 'new MExpr(Args)' }

  TJSNewMemberExpression = Class(TJSMemberExpression)
  private
    FArgs: TJSArguments;
  Public
    Destructor Destroy; override;
    procedure AddArg(El: TJSElement);
    Property Args : TJSArguments Read FArgs Write FArgs;
  end;

  { TJSDotMemberExpression - e.g. 'MExpr.Name' }

  TJSDotMemberExpression = Class(TJSMemberExpression)
  private
    FName: TJSString;
  Public
    Property Name : TJSString Read FName Write FName;
  end;

  { TJSBracketMemberExpression - e.g. 'MExpr[Name]' }

  TJSBracketMemberExpression = Class(TJSMemberExpression)
  private
    FName: TJSElement;
  Public
    Destructor Destroy; override;
    Property Name : TJSElement Read FName Write FName;
  end;

  { TJSCallExpression - e.g. 'Expr(Args)'}

  TJSCallExpression = Class(TJSElement)
  private
    FArgs: TJSArguments;
    FExpr: TJSElement;
  Public
    Destructor Destroy; override;
    procedure AddArg(El: TJSElement);
    procedure InsertArg(Index: integer; El: TJSElement);
    Property Expr : TJSElement Read FExpr Write FExpr;
    Property Args : TJSArguments Read FArgs Write FArgs;
  end;

  { TJSUnary - e.g. 'PrefixOperator A PostFixOperator', '--i' }

  TJSUnary = Class(TJSElement)
  private
    FA: TJSElement;
  Public
    Class function PrefixOperatorToken : tjsToken; virtual;
    Class function PostFixOperatorToken : tjsToken; virtual;
    Class function PrefixOperator : String;
    Class function PostFixOperator : String;
    Destructor Destroy; override;
    Property A : TJSElement Read FA Write FA;
  end;
  TJSUnaryClass = class of TJSUnary;

  { TJSVariableStatement - e.g. 'var A' }

  TJSVariableStatement = Class(TJSUnary);

  { TJSExpressionStatement - A; }

  TJSExpressionStatement = Class(TJSUnary);

  { TJSThrowStatement - e.g. 'throw A' }

  TJSThrowStatement = Class(TJSUnary)
  Public
    Class function PrefixOperatorToken : tjsToken; Override;
  end;

  TJSUnaryExpression = Class(TJSUnary);

  { TJSUnaryDeleteExpression - e.g. 'delete A' }

  TJSUnaryDeleteExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryVoidExpression - e.g. 'void A' }

  TJSUnaryVoidExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryTypeOfExpression - e.g. 'typeof A' }

  TJSUnaryTypeOfExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSAwaitExpression - e.g. 'await A' }

  TJSAwaitExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; Override;
  end;

  { TJSUnaryPrePlusPlusExpression - e.g. '++A' }

  TJSUnaryPrePlusPlusExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryPreMinusMinusExpression - e.g. '--A' }

  TJSUnaryPreMinusMinusExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryPlusExpression - e.g. '+A' }

  TJSUnaryPlusExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryMinusExpression - e.g. '-A' }

  TJSUnaryMinusExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryInvExpression - e.g. '~A' }

  TJSUnaryInvExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryNotExpression - e.g. '!A' }

  TJSUnaryNotExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryPostPlusPlusExpression - e.g. 'A++' }

  TJSUnaryPostPlusPlusExpression = Class(TJSUnaryExpression)
  Public
    Class function PostFixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryPostMinusMinusExpression - e.g. 'A--' }

  TJSUnaryPostMinusMinusExpression = Class(TJSUnaryExpression)
  Public
    Class function PostFixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryBracketsExpression - e.g. '(A)' }

  TJSUnaryBracketsExpression = Class(TJSUnaryExpression)
  Public
    Class function PrefixOperatorToken : tjsToken; override;
    Class function PostFixOperatorToken : tjsToken; override;
  end;

  { TJSBinary - base class }

  TJSBinary = Class(TJSElement)
  private
    FA: TJSElement;
    FB: TJSElement;
  Public
    Destructor Destroy; override;
    Property A : TJSElement Read FA Write FA;
    Property B : TJSElement Read FB Write FB;
  end;
  TJSBinaryClass = Class of TJSBinary;

  { TJSStatementList - a list of statements enclosed in curly brackets }

  TJSStatementList = Class(TJSBinary); // A->first statement, B->next in list, chained.

  { TJSVariableDeclarationList }

  TJSVariableDeclarationList = Class(TJSBinary); // A->first variable, B->next in list, chained.

  { TJSWithStatement - with(A)do B; }

  TJSWithStatement = Class(TJSBinary); // A-> with expression, B->statement(s)

  { TJSBinaryExpression - e.g. A operator B }

  TJSBinaryExpression = Class(TJSBinary)
  Public
    Class function OperatorToken : tjsToken; virtual;
    Class function OperatorString : string;
    Class Function AllowCompact : Boolean; virtual;
  end;

  { TJSLogicalOrExpression - e.g. A || B }

  TJSLogicalOrExpression = Class (TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSLogicalAndExpression - e.g. A && B }

  TJSLogicalAndExpression = Class (TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSBitwiseAndExpression - e.g. A & B }

  TJSBitwiseAndExpression = Class (TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSBitwiseOrExpression - e.g. A | B }

  TJSBitwiseOrExpression = Class (TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSBitwiseXOrExpression - e.g. A ^ B }

  TJSBitwiseXOrExpression = Class (TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  TJSEqualityExpression = Class (TJSBinaryExpression);

  { TJSEqualityExpressionEQ - e.g. A == B }

  TJSEqualityExpressionEQ = Class(TJSEqualityExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSEqualityExpressionNE - e.g. A != B }

  TJSEqualityExpressionNE = Class(TJSEqualityExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSEqualityExpressionSEQ strict equal - e.g. A === B }

  TJSEqualityExpressionSEQ = Class(TJSEqualityExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSEqualityExpressionSNE not strict equal - e.g. A !== B }

  TJSEqualityExpressionSNE = Class(TJSEqualityExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  TJSRelationalExpression = Class(TJSBinaryExpression);

  { TJSRelationalExpressionLT lower than - e.g. A < B }

  TJSRelationalExpressionLT = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRelationalExpressionGT greater than - e.g. A > B }

  TJSRelationalExpressionGT = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRelationalExpressionLE lower equal - e.g. A <= B }

  TJSRelationalExpressionLE = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRelationalExpressionGE greater equal - e.g. A >= B }

  TJSRelationalExpressionGE = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRelationalExpressionIn - e.g. A in B }

  TJSRelationalExpressionIn = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
    Class Function AllowCompact : Boolean; override;
  end;

  { TJSRelationalExpressionInstanceOf - e.g. A instanceof B }

  TJSRelationalExpressionInstanceOf = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
    Class Function AllowCompact : Boolean; override;
  end;

  TJSShiftExpression = Class(TJSBinaryExpression);

  { TJSLShiftExpression - e.g. A << B }

  TJSLShiftExpression = Class(TJSShiftExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRShiftExpression right shift keep sign - e.g. A >> B }

  TJSRShiftExpression = Class(TJSShiftExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSURShiftExpression right shift unsigned, insert zeroes - e.g. A >>> B }

  TJSURShiftExpression = Class(TJSShiftExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  TJSAdditiveExpression = Class(TJSBinaryExpression);

  { TJSAdditiveExpressionPlus - e.g. A + B }

  TJSAdditiveExpressionPlus = Class(TJSAdditiveExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSAdditiveExpressionMinus - e.g. A - B }

  TJSAdditiveExpressionMinus = Class(TJSAdditiveExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  TJSMultiplicativeExpression = Class(TJSBinaryExpression);

  { TJSMultiplicativeExpressionMul - e.g. A * B }

  TJSMultiplicativeExpressionMul = Class(TJSMultiplicativeExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSMultiplicativeExpressionDiv - e.g. A / B }

  TJSMultiplicativeExpressionDiv = Class(TJSMultiplicativeExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSMultiplicativeExpressionMod - e.g. A % B }

  TJSMultiplicativeExpressionMod = Class(TJSMultiplicativeExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSCommaExpression - e.g. A , B }

  TJSCommaExpression = Class(TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSConditionalExpression - e.g. A ? B :C }

  TJSConditionalExpression = Class(TJSElement)
  private
    FA: TJSElement;
    FB: TJSElement;
    FC: TJSElement;
  Public
    Destructor Destroy; override;
    Property A : TJSElement Read FA Write FA;
    Property B : TJSElement Read FB Write FB;
    Property C : TJSElement Read FC Write FC;
  end;

  { TJSAssignStatement - e.g. LHS operator Expr }

  TJSAssignStatement = Class(TJSElement)
  private
    FExpr: TJSElement;
    FLHS: TJSElement;
  Public
    Destructor Destroy; override;
    Class function OperatorToken : tjsToken; virtual;
    Class function OperatorString : String;
    Property Expr : TJSElement Read FExpr Write FExpr;
    Property LHS : TJSElement Read FLHS Write FLHS;
  end;

  TJSAssignStatementClass = Class of TJSAssignStatement;

  { TJSSimpleAssignStatement - e.g. LHS=Expr }

  TJSSimpleAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSMulEqAssignStatement - e.g. LHS*=Expr }

  TJSMulEqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSDivEqAssignStatement - e.g. LHS/=Expr }

  TJSDivEqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSModEqAssignStatement - e.g. LHS%=Expr }

  TJSModEqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSAddEqAssignStatement - e.g. LHS+=Expr }

  TJSAddEqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSSubEqAssignStatement - e.g. LHS-=Expr }

  TJSSubEqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSLShiftEqAssignStatement - e.g. LHS<<=Expr }

  TJSLShiftEqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRShiftEqAssignStatement - e.g. LHS>>=Expr keep sign }

  TJSRShiftEqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSURShiftEqAssignStatement - e.g. LHS>>>=Expr unsigned, insert zeroes }

  TJSURShiftEqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSANDEqAssignStatement - e.g. LHS&=Expr }

  TJSANDEqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSOREqAssignStatement - e.g. LHS|=Expr }

  TJSOREqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSXOREqAssignStatement - e.g. LHS^=Expr }

  TJSXOREqAssignStatement = Class(TJSAssignStatement)
  Public
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSVarDeclaration - e.g. Name=Init }

  TJSVarDeclaration = Class(TJSElement)
  private
    FInit: TJSElement;
    FName: String;
  Public
    Destructor Destroy; override;
    Property Name : String Read FName Write FName;
    Property Init : TJSElement Read FInit Write FInit;
  end;

  { TJSIfStatement - e.g. if (Cond) btrue else bfalse }

  TJSIfStatement = Class(TJSElement)
  private
    FBFalse: TJSElement;
    FBTrue: TJSElement;
    FCond: TJSElement;
  Public
    Destructor Destroy; override;
    Property Cond : TJSElement Read FCond Write FCond;
    Property BTrue : TJSElement Read FBTrue Write FBTrue;
    Property BFalse : TJSElement Read FBFalse Write FBFalse;
  end;

  { TJSTargetStatement
    - base class for statements targetable by continue and break
    - TargetName can be empty }

  TJSTargetStatement = Class(TJSElement)
  private
    FTarget: Cardinal;
    FTargetName: TJSString;
  Public
    Property Target : Cardinal Read FTarget Write FTarget;
    Property TargetName : TJSString Read FTargetName Write FTargetName;
  end;

  { TJSBodyStatement - base class }

  TJSBodyStatement = Class(TJSTargetStatement)
  private
    FBody: TJSElement;
  Public
    Destructor Destroy; override;
    Property Body : TJSElement Read FBody Write FBody;
  end;

  { TJSCondLoopStatement - base class for do..while and while..do }

  TJSCondLoopStatement = Class(TJSBodyStatement)
  private
    FCond: TJSElement;
  Public
    Destructor Destroy; override;
    Property Cond : TJSElement Read FCond Write FCond;
  end;

  { TJSWhileStatement - e.g. 'while(Cond) Body' }

  TJSWhileStatement = Class(TJSCondLoopStatement);

  { TJSDoWhileStatement - e.g. 'do Body while(Cond)' }

  TJSDoWhileStatement = Class(TJSWhileStatement);

  { TJSForStatement - e.g. 'for(Init;Cond;Incr) Body' }

  TJSForStatement = Class(TJSCondLoopStatement)
  private
    FIncr: TJSElement;
    FInit: TJSElement;
  Public
    Destructor Destroy; override;
    Property Incr : TJSElement Read FIncr Write FIncr;
    Property Init : TJSElement Read FInit Write FInit;
  end;

  { TJSForInStatement - e.g. 'for(LHS in List) Body' }

  TJSForInStatement = Class(TJSBodyStatement)
  private
    FLhs: TJSElement;
    FList: TJSElement;
  Public
    Destructor Destroy; override;
    Property LHS : TJSElement Read FLHS Write FLHS;
    Property List : TJSElement Read FList Write FList;
  end;

  { TJSContinueStatement - e.g. 'continue'}

  TJSContinueStatement = Class(TJSTargetStatement);

  { TJSBreakStatement - e.g. 'break' }

  TJSBreakStatement = Class(TJSTargetStatement);

  { TJSReturn - e.g. 'return Expr'}

  TJSReturnStatement = Class(TJSElement)
  private
    FExpr: TJSElement;
  Public
    Destructor Destroy; override;
    Property Expr : TJSElement Read FExpr Write FExpr;
  end;

  { TJSCaseElement - element of TJSCaseElements, e.g. 'case Expr: Body' }

  TJSCaseElement = Class(TCollectionItem)
  private
    FBody: TJSElement;
    FExpr: TJSelement;
  Public
    Destructor Destroy; override;
    Property Expr : TJSelement Read FExpr Write FExpr;
    Property Body : TJSElement Read FBody Write FBody;
  end;

  { TJSCaseElements - Cases property of TJSSwitch }

  TJSCaseElements = Class(TCollection)
  private
    function GetC(AIndex : Integer): TJSCaseElement;
  Public
    Function AddCase : TJSCaseElement;
    Property Cases[AIndex : Integer] : TJSCaseElement Read GetC ;default;
  end;

  { TJSSwitch - e.g. switch(Cond) Cases }

  TJSSwitchStatement = Class(TJSTargetStatement)
  private
    FCases: TJSCaseElements;
    FCond: TJSelement;
    FDefault: TJSCaseElement;
  Public
    Constructor Create(ALine,AColumn : Integer; const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Cond : TJSElement Read FCond Write FCond;
    Property Cases : TJSCaseElements Read FCases;
    Property TheDefault : TJSCaseElement Read FDefault Write FDefault; // one of Cases
  end;

  { TJSLabeledStatement - e.g. 'TheLabel : A' }

  TJSLabeledStatement = Class(TJSUnary)
  private
    FLabel: TJSLabel;
    FTarget: Integer;
  Public
    Destructor Destroy; override;
    Property Target: Integer Read FTarget Write FTarget;
    Property TheLabel : TJSLabel Read FLabel Write Flabel;
  end;

  { TJSTryStatement - e.g. 'try Block catch(Ident) BCatch finally BFinally' }

  TJSTryStatement = Class(TJSElement)
  private
    FBCatch: TJSElement;
    FBFinally: TJSElement;
    FBlock: TJSElement;
    FIdent: TJSString;
  Public
    Destructor Destroy; override;
    Property Block : TJSElement Read FBlock Write FBlock;
    Property BCatch : TJSElement Read FBCatch Write FBCatch;
    Property BFinally : TJSElement Read FBFinally Write FBFinally;
    Property Ident : TJSString Read FIdent Write FIDent;
  end;

  TJSTryCatchFinallyStatement = Class(TJSTryStatement);
  TJSTryCatchStatement = Class(TJSTryStatement);
  TJSTryFinallyStatement = Class(TJSTryStatement);


  { TJSFunctionDeclarationStatement - same as TJSFuncDef, except as a TJSElement }

  TJSFunctionDeclarationStatement = Class(TJSElement)
  private
    FFuncDef: TJSFuncDef;
  Public
    Destructor Destroy; override;
    Property AFunction : TJSFuncDef Read FFuncDef Write FFuncDef;
  end;

  { TJSFunctionBody - the statement block of a function }

  TJSFunctionBody = Class(TJSUnary)
  private
    FIsProgram: Boolean;
  Public
    Property isProgram : Boolean Read FIsProgram Write FIsProgram;
  end;

  { TJSElementNode - element of TJSElementNodes }

  TJSElementNode = Class(TCollectionItem)
  private
    FNode: TJSElement;
  Public
    Destructor Destroy; override;
    Property Node : TJSElement Read FNode Write FNode;
  end;

  { TJSElementNodes - see TJSSourceElements }

  TJSElementNodes = Class(TCollection)
  private
    function GetN(AIndex : Integer): TJSElementNode;
  Public
    Function AddNode : TJSElementNode;
    Function InsertNode(Index: integer) : TJSElementNode;
    Property Nodes[AIndex : Integer] : TJSElementNode Read GetN ; default;
  end;

  { TJSSourceElements - a list of elements, every element ends in semicolon,
    first Vars, then Functions, finally Statements }

  TJSSourceElements = Class(TJSElement)
  private
    FFunctions: TJSElementNodes;
    FStatements: TJSElementNodes;
    FVars: TJSElementNodes;
  Public
    Constructor Create(ALine,AColumn : Integer; const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Vars : TJSElementNodes Read FVars;
    Property Functions : TJSElementNodes Read FFunctions;
    Property Statements : TJSElementNodes Read FStatements;
  end;

implementation

{$IFDEF NOCLASSES}
{ TCollectionItem }

Constructor TCollectionItem.Create(ACollection: TCollection);
begin
  FCollection:=ACollection;
end;

Destructor TCollectionItem.Destroy;
begin
  if Assigned(FCollection) then
    FCollection.Remove(Self);
  inherited Destroy;
end;

{ TCollection }

function TCollection.GetI(AIndex : Integer): TCollectionItem;
begin
  if (AIndex>=0) and (AIndex<FCount) then
    Result:=FItems[AIndex]
  else
    Raise EListError.CreateFmt('Collection index (%d) out of bounds.',[AIndex]);
end;

function TCollection.GetCount: Integer;
begin
  Result:=FCount;
end;

Procedure TCollection.Remove(AItem: TCollectionItem);

Var
  I,J : Integer;

begin
  if (AItem=Nil)  then exit;
  I:=Count-1;
  While (I>=0) and (FItems[I]<>AItem) do
    Dec(i);
  For J:=I to Count-2 do
    FItems[I]:=FItems[i+1];
  Dec(FCount);
end;

Constructor TCollection.Create(AItemClass: TCollectionItemClass);
begin
  FItemClass:=AItemClass;
end;

Destructor TCollection.Destroy;
begin
  Clear;
  inherited Destroy;
end;

Procedure TCollection.Clear;

Var
  I : Integer;

begin
  For I:=0 to Count-1 do
    begin
    FItems[i].FCollection:=Nil;
    FItems[i].Destroy;
    end;
  FCount:=0;
  SetLength(Fitems,0);
end;

Function TCollection.Add: TCollectionItem;
Var
  NL : Integer;
begin
  If FCount=Length(FItems) then
    begin
    NL:=Length(FItems)*3 div 2;
    if NL=0 then NL:=10;
    SetLength(FItems,NL);
    end;
  Result:=FItemClass.Create(Self);
  FItems[FCount]:=Result;
  Inc(FCount);
end;

{ TStrings }

function TStrings.GetCount: Integer;
begin
  Result:=FCount;
end;

function TStrings.GetS(AIndex : Integer): String;
begin
  if (AIndex>=0) and (AIndex<FCount) then
    Result:=FStrings[AIndex]
  else
    Raise EListError.CreateFmt('List index (%d) out of bounds.',[AIndex]);
end;

procedure TStrings.SetS(AIndex : Integer; AValue: String);
begin
  if (AIndex>=0) and (AIndex<=FCount) then
    begin
    if (AIndex=FCount) then
      Add(AValue)
    else
      FStrings[AIndex]:=AValue;
    end
  else
    Raise EListError.CreateFmt('List index (%d) out of bounds.',[AIndex]);
end;

Function TStrings.Add(Const S: String): Integer;

Var
  NL : Integer;
begin
  If FCount=Length(FStrings) then
    begin
    NL:=Length(FStrings)*3 div 2;
    if NL=0 then NL:=10;
    SetLength(FStrings,NL);
    end;
  FStrings[FCount]:=S;
  Inc(FCount);
end;

Procedure TStrings.Assign(Source: TStrings);

Var
  I : Integer;

begin
  SetLength(FStrings,Length(Source.FStrings));
  FCount:=Source.FCount;
  For I:=0 to FCount-1 do
    FStrings[i]:=Source.FStrings[i];
end;
{$ENDIF NOCLASSES}

{ TJSXOREqAssignStatement }

Class function TJSXOREqAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsXOREq;
end;

{ TJSOREqAssignStatement }

Class function TJSOREqAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsOREQ;
end;

{ TJSANDEqAssignStatement }

Class function TJSANDEqAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsAndEq;
end;

{ TJSURShiftEqAssignStatement }

Class function TJSURShiftEqAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsURSHIFTEQ;
end;

{ TJSRShiftEqAssignStatement }

Class function TJSRShiftEqAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsRSHIFTEQ;
end;

{ TJSLShiftEqAssignStatement }

Class function TJSLShiftEqAssignStatement.OperatorToken: tjsToken;
begin
   Result:=tjsLSHIFTEQ;
end;

{ TJSSubEqAssignStatement }

Class function TJSSubEqAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsMINUSEQ;
end;

{ TJSAddEqAssignStatement }

Class function TJSAddEqAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsPLUSEQ;
end;

{ TJSModEqAssignStatement }

Class function TJSModEqAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsMODEQ;
end;

{ TJSDivEqAssignStatement }

Class function TJSDivEqAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsDIVEQ;
end;

{ TJSMulEqAssignStatement }

Class function TJSMulEqAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsMULEQ;
end;

{ TJSSimpleAssignStatement }

Class function TJSSimpleAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsAssign;
end;

{ TJSLabeledStatement }

Destructor TJSLabeledStatement.Destroy;
begin
  FreeAndNil(Flabel);
  inherited Destroy;
end;

{ TJSCommaExpression }

Class function TJSCommaExpression.OperatorToken: tjsToken;
begin
  Result:=tjsComma;
end;

{ TJSMultiplicativeExpressionMod }

Class function TJSMultiplicativeExpressionMod.OperatorToken: tjsToken;
begin
  Result:=tjsMod;
end;

{ TJSMultiplicativeExpressionDiv }

Class function TJSMultiplicativeExpressionDiv.OperatorToken: tjsToken;
begin
  Result:=tjsDiv;
end;

{ TJSMultiplicativeExpressionMul }

Class function TJSMultiplicativeExpressionMul.OperatorToken: tjsToken;
begin
  Result:=tjsMul;
end;

{ TJSAdditiveExpressionMinus }

Class function TJSAdditiveExpressionMinus.OperatorToken: tjsToken;
begin
  Result:=tjsMinus;
end;

{ TJSAdditiveExpressionPlus }

Class function TJSAdditiveExpressionPlus.OperatorToken: tjsToken;
begin
  Result:=tjsPlus;
end;

{ TJSURShiftExpression }

Class function TJSURShiftExpression.OperatorToken: tjsToken;
begin
  Result:=tjsURshift;
end;

{ TJSRShiftExpression }

Class function TJSRShiftExpression.OperatorToken: tjsToken;
begin
  Result:=tjsRSHIFT;
end;

{ TJSLShiftExpression }

Class function TJSLShiftExpression.OperatorToken: tjsToken;
begin
  Result:=tjsLSHIFT;
end;

{ TJSRelationalExpressionInstanceOf }

Class function TJSRelationalExpressionInstanceOf.OperatorToken: tjsToken;
begin
  Result:=tjsInstanceOf;
end;

Class Function TJSRelationalExpressionInstanceOf.AllowCompact: Boolean;
begin
  Result:=False;
end;

{ TJSRelationalExpressionIn }

Class function TJSRelationalExpressionIn.OperatorToken: tjsToken;
begin
  Result:=tjsIn;
end;

Class Function TJSRelationalExpressionIn.AllowCompact: Boolean;
begin
  Result:=False;
end;

{ TJSRelationalExpressionGE }

Class function TJSRelationalExpressionGE.OperatorToken: tjsToken;
begin
  Result:=tjsGE;
end;

{ TJSRelationalExpressionLE }

Class function TJSRelationalExpressionLE.OperatorToken: tjsToken;
begin
  Result:=tjsLE;
end;

{ TJSRelationalExpressionGT }

Class function TJSRelationalExpressionGT.OperatorToken: tjsToken;
begin
  Result:=tjsGT;
end;

{ TJSRelationalExpressionLT }

Class function TJSRelationalExpressionLT.OperatorToken: tjsToken;
begin
  Result:=tjsLT;
end;

{ TJSEqualityExpressionSNE }

Class function TJSEqualityExpressionSNE.OperatorToken: tjsToken;
begin
  Result:=tjsSNE;
end;

{ TJSEqualityExpressionSEQ }

Class function TJSEqualityExpressionSEQ.OperatorToken: tjsToken;
begin
  Result:=tjsSEQ;
end;

{ TJSEqualityExpressionNE }

Class function TJSEqualityExpressionNE.OperatorToken: tjsToken;
begin
  Result:=tjsNE;
end;

{ TJSEqualityExpressionEQ }

Class function TJSEqualityExpressionEQ.OperatorToken: tjsToken;
begin
  Result:=tjsEQ;
end;

{ TJSBinaryExpression }

Class function TJSBinaryExpression.OperatorToken: tjsToken;
begin
  Result:=tjsUnknown
end;

Class function TJSBinaryExpression.OperatorString: string;

Var
  T : TJSToken;
begin
  T:=OperatorToken;
  if (T<>tjsUnknown) then
    begin
    Result:=TokenInfos[T]
    end
  else
    Result:='';
end;

Class Function TJSBinaryExpression.AllowCompact: Boolean;
begin
  Result:=True
end;

{ TJSBitwiseXOrExpression }

Class function TJSBitwiseXOrExpression.OperatorToken : tjsToken;
begin
  Result:=tjsXor
end;

{ TJSBitwiseOrExpression }

Class function TJSBitwiseOrExpression.OperatorToken : tjsToken;
begin
  Result:=tjsOr
end;

{ TJSBitwiseAndExpression }

Class function TJSBitwiseAndExpression.OperatorToken : tjsToken;
begin
  Result:=tjsAnd
end;

{ TJSLogicalAndExpression }

Class function TJSLogicalAndExpression.OperatorToken : tjsToken;
begin
  Result:=tjsAndAnd
end;

{ TJSLogicalOrExpression }

Class function TJSLogicalOrExpression.OperatorToken : tjsToken;
begin
  Result:=tjsOrOr
end;

{ TJSUnaryVoidExpression }

Class function TJSUnaryVoidExpression.PrefixOperatorToken : tjsToken;
begin
  Result:=tjsvoid;
end;

{ TJSThrowStatement }

Class function TJSThrowStatement.PrefixOperatorToken: tjsToken;
begin
  Result:=tjsThrow;
end;

{ TJSUnaryBracketsExpression }

class function TJSUnaryBracketsExpression.PrefixOperatorToken: tjsToken;
begin
  Result:=tjsBraceOpen;
end;

class function TJSUnaryBracketsExpression.PostFixOperatorToken: tjsToken;
begin
  Result:=tjsBraceClose;
end;

{ TJSUnaryPostMinusMinusExpression }

Class function TJSUnaryPostMinusMinusExpression.PostFixOperatorToken : tjsToken;
begin
  Result:=tjsMinusMinus;
end;

{ TJSUnaryPostPlusPlusExpression }

Class function TJSUnaryPostPlusPlusExpression.PostFixOperatorToken : tjsToken;
begin
  Result:=tjsPlusPlus;
end;

{ TJSUnaryNotExpression }

Class function TJSUnaryNotExpression.PrefixOperatorToken : tjsToken;
begin
  Result:=tjsNot;
end;

{ TJSUnaryInvExpression }

Class function TJSUnaryInvExpression.PrefixOperatorToken : tjsToken;
begin
  Result:=tjsinv;
end;

{ TJSUnaryMinusExpression }

Class function TJSUnaryMinusExpression.PrefixOperatorToken : tjsToken;
begin
  Result:=tjsMinus;
end;

{ TJSUnaryPlusExpression }

Class function TJSUnaryPlusExpression.PrefixOperatorToken : tjsToken;
begin
  Result:=tjsPlus;
end;

{ TJSUnaryPreMinusMinusExpression }

Class function TJSUnaryPreMinusMinusExpression.PrefixOperatorToken : tjsToken;
begin
  Result:=tjsMinusMinus;
end;

{ TJSUnaryPrePlusPlusExpression }

Class function TJSUnaryPrePlusPlusExpression.PrefixOperatorToken : tjsToken;
begin
  Result:=tjsPlusPlus;
end;

{ TJSUnaryTypeOfExpression }

Class function TJSUnaryTypeOfExpression.PrefixOperatorToken : tjsToken;
begin
  Result:=tjsTypeOf;
end;

{ TJSAwaitExpression }

class function TJSAwaitExpression.PrefixOperatorToken: tjsToken;
begin
  Result:=tjsAwait;
end;

{ TJSUnaryDeleteExpression }

Class function TJSUnaryDeleteExpression.PrefixOperatorToken : tjsToken;
begin
  Result:=tjsdelete;
end;

{ TJSElement }

constructor TJSElement.Create(ALine, AColumn: Integer; const ASource: String);
begin
  FLine:=ALine;
  FColumn:=AColumn;
  FSource:=ASource;
end;

procedure TJSElement.AssignPosition(El: TJSElement);
begin
  Source:=El.Source;
  Line:=El.Line;
  Column:=El.Column;
end;

{ TJSRegularExpressionLiteral }

function TJSRegularExpressionLiteral.GetA(AIndex : integer): TJSValue;
begin
  Result:=FArgv[AIndex];
end;

procedure TJSRegularExpressionLiteral.SetA(AIndex : integer;
  const AValue: TJSValue);
begin
  FArgv[AIndex]:=Avalue;
end;

constructor TJSRegularExpressionLiteral.Create(ALine, AColumn: Integer;
  const ASource: String);
begin
  inherited Create(ALine, AColumn, ASource);
  FPattern:=TJSValue.Create;
  FPatternFlags:=TJSValue.Create;
end;

destructor TJSRegularExpressionLiteral.Destroy;
begin
  FreeAndNil(FPattern);
  FreeAndNil(FPatternFlags);
  Inherited Destroy;
end;

{ TJSArrayLiteralElements }

function TJSArrayLiteralElements.GetE(AIndex : Integer): TJSArrayLiteralElement;
begin
  Result:=TJSArrayLiteralElement(Items[AIndex]);
end;

function TJSArrayLiteralElements.AddElement: TJSArrayLiteralElement;
begin
  Result:=TJSArrayLiteralElement(Add);
end;

{ TJSArrayLiteral }

constructor TJSArrayLiteral.Create(ALine, AColumn: Integer; const ASource: String);
begin
  inherited Create(ALine, AColumn, ASource);
  FElements:=TJSArrayLiteralElements.Create(TJSArrayLiteralElement);
end;

procedure TJSArrayLiteral.AddElement(El: TJSElement);
var
  ArrEl: TJSArrayLiteralElement;
begin
  ArrEl:=Elements.AddElement;
  ArrEl.ElementIndex:=Elements.Count-1;
  ArrEl.Expr:=El;
end;

destructor TJSArrayLiteral.Destroy;
begin
  FreeAndNil(FElements);
  inherited Destroy;
end;

{ TJSObjectLiteralElements }

function TJSObjectLiteralElements.GetE(AIndex : Integer
  ): TJSObjectLiteralElement;
begin
  Result:=TJSObjectLiteralElement(Items[AIndex]);
end;


function TJSObjectLiteralElements.AddElement: TJSObjectLiteralElement;
begin
  Result:=TJSObjectLiteralElement(Add);
end;

{ TJSObjectLiteral }

constructor TJSObjectLiteral.Create(ALine, AColumn: Integer; const ASource: String = '');
begin
  inherited Create(ALine, AColumn, ASource);
  FElements:=TJSObjectLiteralElements.Create(TJSObjectLiteralElement);
end;

destructor TJSObjectLiteral.Destroy;
begin
  FreeAndNil(FElements);
  inherited Destroy;
end;

{ TJSObjectLiteralElement }

destructor TJSObjectLiteralElement.Destroy;
begin
  FreeAndNil(Fexpr);
  inherited Destroy;
end;

{ TJSArrayLiteralElement }

destructor TJSArrayLiteralElement.Destroy;
begin
  FreeAndNil(FExpr);
  inherited Destroy;
end;

{ TJSNewMemberExpression }

destructor TJSNewMemberExpression.Destroy;
begin
  FreeAndNil(FArgs);
  inherited Destroy;
end;

procedure TJSNewMemberExpression.AddArg(El: TJSElement);
begin
  if Args=nil then
    Args:=TJSArguments.Create(Line,Column,Source);
  Args.Elements.AddElement.Expr:=El;
end;

{ TJSMemberExpression }

destructor TJSMemberExpression.Destroy;
begin
  FreeAndNil(FMExpr);
  inherited Destroy;
end;

{ TJSCallExpression }

destructor TJSCallExpression.Destroy;
begin
  FreeAndNil(FExpr);
  FreeAndNil(FArgs);
  inherited Destroy;
end;

procedure TJSCallExpression.AddArg(El: TJSElement);
begin
  Args.Elements.AddElement.Expr:=El;
end;

procedure TJSCallExpression.InsertArg(Index: integer; El: TJSElement);
var
  NewEl: TJSArrayLiteralElement;
begin
  NewEl:=TJSArrayLiteralElement(Args.Elements.Insert(Index));
  NewEl.Expr:=El;
end;

{ TJSUnary }

Class function TJSUnary.PrefixOperatorToken: tjsToken;
begin
  Result:=tjsUnknown;
end;

Class function TJSUnary.PostFixOperatorToken: tjsToken;
begin
  Result:=tjsUnknown;
end;

Class function TJSUnary.PrefixOperator: String;

Var
  T : TJSToken;

begin
  T:=PrefixOperatorToken;
  if (T=tjsUnknown) then
    Result:=''
  else
    begin
    Result:=TokenInfos[t];
    if t in [tjsTypeOf,tjsVoid,tjsDelete,tjsThrow,tjsAwait] then
      Result:=Result+' ';
    end;
end;

Class function TJSUnary.PostFixOperator: String;

Var
  T : TJSToken;

begin
  T:=PostFixOperatorToken;
  if (T=tjsUnknown) then
    Result:=''
  else
    Result:=TokenInfos[t];
end;

Destructor TJSUnary.Destroy;
begin
  FreeAndNil(FA);
  inherited Destroy;
end;

{ TJSBinary }

destructor TJSBinary.Destroy;
var
  El: TJSElement;
  BinCnt: Integer;
  Bins: TJSElementArray;
  SubBin: TJSBinary;
begin
  if FA is TJSBinary then
    begin
    // free El binary chains without stack
    El:=FA;
    SetLength(Bins{%H-},8);
    BinCnt:=0;
    while El is TJSBinary do
      begin
      SubBin:=TJSBinary(El);
      if BinCnt=length(Bins) then
        SetLength(Bins,BinCnt*2);
      Bins[BinCnt]:=SubBin;
      inc(BinCnt);
      El:=SubBin.FA;
      end;
    while BinCnt>0 do
      begin
      dec(BinCnt);
      SubBin:=TJSBinary(Bins[BinCnt]);
      FreeAndNil(SubBin.FA);
      FreeAndNil(SubBin.FB);
      end;
    end;

  FreeAndNil(FA);
  FreeAndNil(FB);
  inherited Destroy;
end;

{ TJSConditionalExpression }

destructor TJSConditionalExpression.Destroy;
begin
  FreeAndNil(FB);
  FreeAndNil(FA);
  FreeAndNil(FC);
  inherited Destroy;
end;

{ TJSAssign }

Destructor TJSAssignStatement.Destroy;
begin
  FreeAndNil(FLHS);
  FreeAndNil(FExpr);
  inherited Destroy;
end;

Class function TJSAssignStatement.OperatorToken: tjsToken;
begin
  Result:=tjsUNknown;
end;

Class function TJSAssignStatement.OperatorString: String;

Var
  t :  TJSToken;
begin
  T:=OperatorToken;
  if (tjsUNknown<>t) then
    Result:=TokenInfos[t]
  else
    Result:='';
end;

{ TJSVarDeclaration }


destructor TJSVarDeclaration.Destroy;
begin
  FreeAndNil(FInit);
  inherited Destroy;
end;

{ TJSIfStatement }

destructor TJSIfStatement.Destroy;
begin
  FreeAndNil(FCond);
  FreeAndNil(FBTrue);
  FreeAndNil(FBFalse);
  inherited Destroy;
end;

{ TJSBodyStatement }

destructor TJSBodyStatement.Destroy;
begin
  FreeAndNil(FBody);
  inherited Destroy;
end;

{ TJSCondLoopStatement }

destructor TJSCondLoopStatement.Destroy;
begin
  FreeAndNil(FCond);
  inherited Destroy;
end;

{ TJSForStatement }

destructor TJSForStatement.Destroy;
begin
  FreeAndNil(FIncr);
  FreeAndNil(FInit);
  inherited Destroy;
end;

{ TJSForInStatement }

destructor TJSForInStatement.Destroy;
begin
  FreeAndNil(FList);
  FreeAndNil(FLHS);
  inherited Destroy;
end;


{ TJSReturn }

destructor TJSReturnStatement.Destroy;
begin
  FreeAndNil(FExpr);
  inherited Destroy;
end;

{ TJSCaseElement }

destructor TJSCaseElement.Destroy;
begin
  FreeAndNil(FExpr);
  FreeAndNil(FBody);
  inherited Destroy;
end;

{ TJSSwitch }

constructor TJSSwitchStatement.Create(ALine, AColumn: Integer; const ASource: String);
begin
  inherited Create(ALine, AColumn, ASource);
  FCases:=TJSCaseElements.Create(TJSCaseElement);
end;

destructor TJSSwitchStatement.Destroy;
begin
  FreeAndNil(FCases);
  FreeAndNil(FCond);
  inherited Destroy;
end;

{ TJSCaseElements }

function TJSCaseElements.GetC(AIndex : Integer): TJSCaseElement;
begin
  Result:=TJSCaseElement(Items[AIndex]);
end;


function TJSCaseElements.AddCase: TJSCaseElement;
begin
  Result:=TJSCaseElement(Add);
end;

{ TJSTryStatement }

destructor TJSTryStatement.Destroy;
begin
  FreeAndNil(FBlock);
  FreeAndNil(FBCatch);
  FreeAndNil(FBFinally);
  inherited Destroy;
end;

{ TJSSourceElements }

constructor TJSSourceElements.Create(ALine, AColumn: Integer; const ASource: String
  );
begin
  inherited Create(ALine, AColumn, ASource);
  FStatements:=TJSElementNodes.Create(TJSElementNode);
  FFunctions:=TJSElementNodes.Create(TJSElementNode);
  FVars:=TJSElementNodes.Create(TJSElementNode);
end;

destructor TJSSourceElements.Destroy;

Var
  i : integer;

begin
  FreeAndNil(FStatements);
  FreeAndNil(FFunctions);
  // Vars are owned by their statements, and those have been freed
  For I:=0 to FVars.Count-1 do
    FVars.Nodes[i].Node:=nil;
  FreeAndNil(FVars);
  inherited Destroy;
end;

{ TJSElementNodes }

function TJSElementNodes.GetN(AIndex : Integer): TJSElementNode;
begin
  Result:=TJSElementNode(Items[Aindex])
end;

function TJSElementNodes.AddNode: TJSElementNode;
begin
  Result:=TJSElementNode(Add);
end;

function TJSElementNodes.InsertNode(Index: integer): TJSElementNode;
begin
  Result:=TJSElementNode(Insert(Index));
end;

{ TJSFunction }

destructor TJSFunctionDeclarationStatement.Destroy;
begin
  FreeAndNil(FFuncDef);
  inherited Destroy;
end;

{ TJSElementNode }

destructor TJSElementNode.Destroy;
begin
  FreeAndNil(FNode);
  inherited Destroy;
end;

{ TJSFuncDef }

procedure TJSFuncDef.SetParams(const AValue: TStrings);
begin
  if FParams=AValue then exit;
  FParams.Assign(AValue);
end;

constructor TJSFuncDef.Create;
begin
  FParams:=TStringList.Create;
end;

destructor TJSFuncDef.Destroy;
begin
  FreeAndNil(FBody);
  FreeAndNil(FParams);
  inherited Destroy;
end;

{ TJSBracketMemberExpression }

destructor TJSBracketMemberExpression.Destroy;
begin
  FreeAndNil(FName);
  inherited Destroy;
end;

{ TJSLiteral }

constructor TJSLiteral.Create(ALine, AColumn: Integer; const ASource: String);
begin
  FValue:=TJSValue.Create;
  inherited Create(ALine, AColumn, ASource);
end;

destructor TJSLiteral.Destroy;
begin
  FreeAndNil(FValue);
  Inherited;
end;

end.

