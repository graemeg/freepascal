unit jstree;

{$mode objfpc}{$H+}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ $DEFINE NOCLASSES}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
  Classes, SysUtils, jsbase;

Type
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, jsbase;

Type
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, jsbase;

Type
>>>>>>> graemeg/cpstrnew
=======
  Classes, SysUtils, jsbase;

Type
>>>>>>> origin/cpstrnew
  TJSElementFlag = (elIsConst,elIsConstValid);
  TJSElementFlags = set of TJSElementFlag;

  TJSFunctionBody = Class;

  { TJSElement }

  TJSObject = Class(TObject);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    { TJSLabelSet }

    TJSLabelSet = Class(TJSObject)
    private
      FCOnt: Boolean;
      FNext: TJSLabelSet;
      FTarget: Cardinal;
    Public
      Property Target : Cardinal Read FTarget Write FTarget;
      Property Next : TJSLabelSet Read FNext Write FNext; // Linked list
      Property Continuable : Boolean Read FCOnt Write FCont;
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

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { TJSFuncDef }

  TJSFuncDef = Class(TJSObject)
  private
    FBody: TJSFunctionBody;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FIsEmpty: Boolean;
    FName: String;
    FParams: TStrings;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FCache: TJSObject;
    FCommon: TJSObject;
    FIsEmpty: Boolean;
    FName: String;
    FNext: TJSFuncDef;
    FParams: TStrings;
    FSec: TObject;
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
    procedure SetParams(const AValue: TStrings);
  Public
    Constructor Create;
    Destructor Destroy; override;
    Property Params : TStrings Read FParams Write SetParams;
    Property Body : TJSFunctionBody Read FBody Write FBody;
    Property Name : String Read FName Write FName;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property IsEmpty : Boolean Read FIsEmpty Write FIsEmpty;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Property Common : TJSObject Read FCommon Write FCommon;
    Property Cache : TJSObject Read FCache write FCache;
    Property Next : TJSFuncDef Read FNext Write FNext;
    Property IsEmpty : Boolean Read FIsEmpty Write FIsEmpty;
    Property SecurityDomain : TObject Read FSec Write FSec;
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

  TJSString = WideString;

  TJSElement = Class (TJSObject)
  private
    FFlags: TJSElementFlags;
    FLine: Integer;
    FRow: Integer;
    FSource: String;
  Public
    Constructor Create(ALine,ARow : Integer; Const ASource : String = ''); virtual;
    Property Source : String Read FSource Write FSource;
    Property Row : Integer Read FRow Write FRow;
    Property Line : Integer Read FLine Write FLine;
    Property Flags : TJSElementFlags Read FFlags Write FFlags;
  end;
  TJSElementClass = Class of TJSElement;

  { TJSEmptyBlockStatement }

  TJSEmptyBlockStatement = Class(TJSElement);
  TJSEmptyStatement = Class(TJSElement);

  { TJSLiteral }

  TJSLiteral = Class(TJSElement)
  private
    FValue: TJSValue;
  Public
    Constructor Create(ALine,ARow : Integer; Const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Value : TJSValue Read FValue Write FValue;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
(*  { TJSStringLiteral }
=======
  { TJSStringLiteral }
>>>>>>> graemeg/cpstrnew
=======
  { TJSStringLiteral }
>>>>>>> graemeg/cpstrnew
=======
  { TJSStringLiteral }
>>>>>>> graemeg/cpstrnew
=======
  { TJSStringLiteral }
>>>>>>> origin/cpstrnew

  TJSStringLiteral = Class(TJSElement)
  private
    FValue: TJSString;
  Public
    Property Value : TJSString Read FValue Write FValue;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
*)
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
  { TJSRegularExpressionLiteral }

  TJSRegularExpressionLiteral = Class(TJSElement)
  private
    FPattern: TJSValue;
    FPatternFlags: TJSValue;
    FArgv : Array[0..1] of TJSValue;
    function GetA(AIndex : integer): TJSValue;
    procedure SetA(AIndex : integer; const AValue: TJSValue);
  Public
    Constructor Create(ALine,ARow : Integer; Const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Pattern : TJSValue Read FPattern Write FPattern;
    Property PatternFlags : TJSValue Read FPatternFlags Write FPatternFlags;
    Property Argv[AIndex : integer] : TJSValue Read GetA Write SetA;
  end;

  { TJSPrimaryExpressionIdent }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TJSPrimaryExpression = Class(TJSElement);

  TJSPrimaryExpressionIdent = Class(TJSPrimaryExpression)
  private
    FName: TJSString;
  Public
    Property Name : TJSString Read FName Write FName;
  end;
  TJSPrimaryExpressionThis = Class(TJSPrimaryExpression);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  TJSPrimaryExpressionIdent = Class(TJSElement)
  private
    FString: TJSString;
  Public
    Property AString : TJSString Read FString Write FString;
  end;
  TJSPrimaryExpressionThis = Class(TJSElement);
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

  { TJSArrayLiteralElement }

  TJSArrayLiteralElement = Class(TCollectionItem)
  private
    FExpr: TJSelement;
    FFindex: Integer;
  Public
    Destructor Destroy; override;
    Property Expr : TJSelement Read FExpr Write FExpr;
    Property ElementIndex : Integer Read FFindex Write FFIndex;
  end;

  { TJSArrayLiteralElements }

  TJSArrayLiteralElements = Class(TCollection)
  private
    function GetE(AIndex : Integer): TJSArrayLiteralElement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Function AddElement : TJSArrayLiteralElement;
    Property Elements[AIndex : Integer] : TJSArrayLiteralElement Read GetE ; default;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure SetE(AIndex : Integer; const AValue: TJSArrayLiteralElement);
  Public
    Function AddElement : TJSArrayLiteralElement;
    Property Elements[AIndex : Integer] : TJSArrayLiteralElement Read GetE Write SetE; default;
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

  { TJSArrayLiteral }

  TJSArrayLiteral = Class(TJSElement)
  private
    FElements: TJSArrayLiteralElements;
  Public
    Constructor Create(ALine,ARow : Integer; const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Elements : TJSArrayLiteralElements Read FElements;
  end;

  { TJSObjectLiteralElement }

  TJSObjectLiteralElement = Class(TCollectionItem)
  private
    FExpr: TJSelement;
    FName: TJSString;
  Public
    Destructor Destroy; override;
    Property Expr : TJSelement Read FExpr Write FExpr;
    Property Name : TJSString Read FName Write FName;
  end;

  { TJSObjectLiteralElements }

  TJSObjectLiteralElements = Class(TCollection)
  private
    function GetE(AIndex : Integer): TJSObjectLiteralElement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Function AddElement : TJSObjectLiteralElement;
    Property Elements[AIndex : Integer] : TJSObjectLiteralElement Read GetE ; default;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure SetE(AIndex : Integer; const AValue: TJSObjectLiteralElement);
  Public
    Function AddElement : TJSObjectLiteralElement;
    Property Elements[AIndex : Integer] : TJSObjectLiteralElement Read GetE Write SetE; default;
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

  { TJSObjectLiteral }

  TJSObjectLiteral = Class(TJSElement)
  private
    FElements: TJSObjectLiteralElements;
  Public
    Constructor Create(ALine,ARow : Integer; const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Elements : TJSObjectLiteralElements Read FElements;
  end;

  { TJSArguments }

  TJSArguments = Class(TJSArrayLiteral);

  { TJSMemberExpression }

  TJSMemberExpression = Class(TJSElement)
  private
    FMexpr: TJSElement;
  Public
    Destructor Destroy; override;
    Property Mexpr : TJSElement Read FMexpr Write FMexpr;
  end;

  { TJSNewMemberExpression }

  TJSNewMemberExpression = Class(TJSMemberExpression)
  private
    FArgs: TJSArguments;
  Public
    Destructor Destroy; override;
    Property Args : TJSArguments Read FArgs Write FArgs;
  end;

  { TJSDotMemberExpression }

  TJSDotMemberExpression = Class(TJSMemberExpression)
  private
    FName: TJSString;
  Public
    Property Name : TJSString Read FName Write FName;
  end;

  { TJSBracketMemberExpression }

  TJSBracketMemberExpression = Class(TJSMemberExpression)
  private
    FName: TJSElement;
  Public
    Destructor Destroy; override;
    Property Name : TJSElement Read FName Write FName;
  end;

  { TJSCallExpression }

  TJSCallExpression = Class(TJSElement)
  private
    FArgs: TJSArguments;
    FExpr: TJSElement;
  Public
    Destructor Destroy; override;
    Property Expr : TJSElement Read FExpr Write FExpr;
    Property Args : TJSArguments Read FArgs Write FArgs;
  end;

  { TJSUnary }

  TJSUnary = Class(TJSElement)
  private
    FA: TJSElement;
  Public
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Class function PrefixOperatorToken : tjsToken; virtual;
    Class function PostFixOperatorToken : tjsToken; virtual;
    Class function PrefixOperator : String;
    Class function PostFixOperator : String;
    Destructor Destroy; override;
    Property A : TJSElement Read FA Write FA;
  end;
  TJSUnaryClass = class of TJSUnary;
=======
    Destructor Destroy; override;
    Property A : TJSElement Read FA Write FA;
  end;
>>>>>>> graemeg/cpstrnew
=======
    Destructor Destroy; override;
    Property A : TJSElement Read FA Write FA;
  end;
>>>>>>> graemeg/cpstrnew
=======
    Destructor Destroy; override;
    Property A : TJSElement Read FA Write FA;
  end;
>>>>>>> graemeg/cpstrnew
=======
    Destructor Destroy; override;
    Property A : TJSElement Read FA Write FA;
  end;
>>>>>>> origin/cpstrnew

  { TJSVariableStatement }
  TJSVariableStatement = Class(TJSUnary);
  TJSExpressionStatement = Class(TJSUnary);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  { TJSThrowStatement }

  TJSThrowStatement = Class(TJSUnary)
    Class function PrefixOperatorToken : tjsToken; Override;
  end;

  TJSUnaryExpression = Class(TJSUnary);

  { TJSUnaryDeleteExpression }

  TJSUnaryDeleteExpression = Class(TJSUnaryExpression)
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryVoidExpression }

  TJSUnaryVoidExpression = Class(TJSUnaryExpression)
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryTypeOfExpression }

  TJSUnaryTypeOfExpression = Class(TJSUnaryExpression)
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryPrePlusPlusExpression }

  TJSUnaryPrePlusPlusExpression = Class(TJSUnaryExpression)
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryPreMinusMinusExpression }

  TJSUnaryPreMinusMinusExpression = Class(TJSUnaryExpression)
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryPlusExpression }

  TJSUnaryPlusExpression = Class(TJSUnaryExpression)
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryMinusExpression }

  TJSUnaryMinusExpression = Class(TJSUnaryExpression)
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryInvExpression }

  TJSUnaryInvExpression = Class(TJSUnaryExpression)
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryNotExpression }

  TJSUnaryNotExpression = Class(TJSUnaryExpression)
    Class function PrefixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryPostPlusPlusExpression }

  TJSUnaryPostPlusPlusExpression = Class(TJSUnaryExpression)
    Class function PostFixOperatorToken : tjsToken; override;
  end;

  { TJSUnaryPostMinusMinusExpression }

  TJSUnaryPostMinusMinusExpression = Class(TJSUnaryExpression)
    Class function PostFixOperatorToken : tjsToken; override;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TJSThrowStatement = Class(TJSUnary);
  TJSUnaryExpression = Class(TJSUnary);
  TJSUnaryDeleteExpression = Class(TJSUnaryExpression);
  TJSUnaryVoidExpression = Class(TJSUnaryExpression);
  TJSUnaryTypeOfExpression = Class(TJSUnaryExpression);
  TJSUnaryPrePlusPlusExpression = Class(TJSUnaryExpression);
  TJSUnaryPreMinusMinusExpression = Class(TJSUnaryExpression);
  TJSUnaryPlusExpression = Class(TJSUnaryExpression);
  TJSUnaryMinusExpression = Class(TJSUnaryExpression);
  TJSUnaryInvExpression = Class(TJSUnaryExpression);
  TJSUnaryNotExpression = Class(TJSUnaryExpression);
  TJSUnaryPostPlusPlusExpression = Class(TJSUnaryExpression);
  TJSUnaryPostMinusMinusExpression = Class(TJSUnaryExpression);
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



  { TJSBinary }

  TJSBinary = Class(TJSElement)
  private
    FA: TJSElement;
    FB: TJSElement;
  Public
    Destructor Destroy; override;
    Property A : TJSElement Read FA Write FA;
    Property B : TJSElement Read FB Write FB;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TJSBinaryClass = Class of TJSBinary;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  { TJSStatementList }

  TJSStatementList = Class(TJSBinary); // A->first statement, B->next in list, chained.
  TJSVariableDeclarationList = Class(TJSBinary);
  TJSWithStatement = Class(TJSBinary); // A-> with expression, B->statement(s)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  { TJSBinaryExpression }

  TJSBinaryExpression = Class(TJSBinary)
    Class function OperatorToken : tjsToken; virtual;
    Class function OperatorString : string;
    Class Function AllowCompact : Boolean; virtual;
  end;

  { TJSLogicalOrExpression }

  TJSLogicalOrExpression = Class (TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSLogicalAndExpression }

  TJSLogicalAndExpression = Class (TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSBitwiseAndExpression }

  TJSBitwiseAndExpression = Class (TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSBitwiseOrExpression }

  TJSBitwiseOrExpression = Class (TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSBitwiseXOrExpression }

  TJSBitwiseXOrExpression = Class (TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  TJSEqualityExpression = Class (TJSBinaryExpression);

  { TJSEqualityExpressionEQ }

  TJSEqualityExpressionEQ = Class(TJSEqualityExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSEqualityExpressionNE }

  TJSEqualityExpressionNE = Class(TJSEqualityExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSEqualityExpressionSEQ }

  TJSEqualityExpressionSEQ = Class(TJSEqualityExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSEqualityExpressionSNE }

  TJSEqualityExpressionSNE = Class(TJSEqualityExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  TJSRelationalExpression = Class(TJSBinaryExpression);

  { TJSRelationalExpressionLT }

  TJSRelationalExpressionLT = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRelationalExpressionGT }

  TJSRelationalExpressionGT = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRelationalExpressionLE }

  TJSRelationalExpressionLE = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRelationalExpressionGE }

  TJSRelationalExpressionGE = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRelationalExpressionIn }

  TJSRelationalExpressionIn = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
    Class Function AllowCompact : Boolean; override;
  end;

  { TJSRelationalExpressionInstanceOf }

  TJSRelationalExpressionInstanceOf = Class(TJSRelationalExpression)
    Class function OperatorToken : tjsToken; override;
    Class Function AllowCompact : Boolean; override;
  end;

  TJSShiftExpression = Class(TJSBinaryExpression);

  { TJSLShiftExpression }

  TJSLShiftExpression = Class(TJSShiftExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRShiftExpression }

  TJSRShiftExpression = Class(TJSShiftExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSURShiftExpression }

  TJSURShiftExpression = Class(TJSShiftExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  TJSAdditiveExpression = Class(TJSBinaryExpression);

  { TJSAdditiveExpressionPlus }

  TJSAdditiveExpressionPlus = Class(TJSAdditiveExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSAdditiveExpressionMinus }

  TJSAdditiveExpressionMinus = Class(TJSAdditiveExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  TJSMultiplicativeExpression = Class(TJSBinaryExpression);

  { TJSMultiplicativeExpressionMul }

  TJSMultiplicativeExpressionMul = Class(TJSMultiplicativeExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSMultiplicativeExpressionDiv }

  TJSMultiplicativeExpressionDiv = Class(TJSMultiplicativeExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSMultiplicativeExpressionMod }

  TJSMultiplicativeExpressionMod = Class(TJSMultiplicativeExpression)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSCommaExpression }

  TJSCommaExpression = Class(TJSBinaryExpression)
    Class function OperatorToken : tjsToken; override;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TJSLogicalOrExpression = Class (TJSBinary);
  TJSLogicalAndExpression = Class (TJSBinary);
  TJSBitwiseAndExpression = Class (TJSBinary);
  TJSBitwiseOrExpression = Class (TJSBinary);
  TJSBitwiseXOrExpression = Class (TJSBinary);
  TJSEqualityExpression = Class (TJSBinary);
  TJSEqualityExpressionEQ = Class(TJSEqualityExpression);
  TJSEqualityExpressionNE = Class(TJSEqualityExpression);
  TJSEqualityExpressionSEQ = Class(TJSEqualityExpression);
  TJSEqualityExpressionSNE = Class(TJSEqualityExpression);
  TJSRelationalExpression = Class(TJSBinary);
  TJSRelationalExpressionLT = Class(TJSRelationalExpression);
  TJSRelationalExpressionGT = Class(TJSRelationalExpression);
  TJSRelationalExpressionLE = Class(TJSRelationalExpression);
  TJSRelationalExpressionGE = Class(TJSRelationalExpression);
  TJSRelationalExpressionIn = Class(TJSRelationalExpression);
  TJSRelationalExpressionInstanceOf = Class(TJSRelationalExpression);
  TJSShiftExpression = Class(TJSBinary);
  TJSLShiftExpression = Class(TJSShiftExpression);
  TJSRShiftExpression = Class(TJSShiftExpression);
  TJSURShiftExpression = Class(TJSShiftExpression);
  TJSAdditiveExpression = Class(TJSBinary);
  TJSAdditiveExpressionPlus = Class(TJSAdditiveExpression);
  TJSAdditiveExpressionMinus = Class(TJSAdditiveExpression);
  TJSMultiplicativeExpression = Class(TJSBinary);
  TJSMultiplicativeExpressionMul = Class(TJSMultiplicativeExpression);
  TJSMultiplicativeExpressionDiv = Class(TJSMultiplicativeExpression);
  TJSMultiplicativeExpressionMod = Class(TJSMultiplicativeExpression);
  TJSCommaExpression = Class(TJSBinary);
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

  { TJSConditionalExpression }

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

  { TJSAssignStatement }

  TJSAssignStatement = Class(TJSElement)
  private
    FExpr: TJSElement;
    FLHS: TJSElement;
  Public
    Destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Class function OperatorToken : tjsToken; virtual;
    Class function OperatorString : String;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Property Expr : TJSElement Read FExpr Write FExpr;
    Property LHS : TJSElement Read FLHS Write FLHS;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TJSAssignStatementClass = Class of TJSAssignStatement;

  { TJSSimpleAssignStatement }

  TJSSimpleAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSMulEqAssignStatement }

  TJSMulEqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSDivEqAssignStatement }

  TJSDivEqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSModEqAssignStatement }

  TJSModEqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSAddEqAssignStatement }

  TJSAddEqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSSubEqAssignStatement }

  TJSSubEqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSLShiftEqAssignStatement }

  TJSLShiftEqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSRShiftEqAssignStatement }

  TJSRShiftEqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSURShiftEqAssignStatement }

  TJSURShiftEqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSANDEqAssignStatement }

  TJSANDEqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSOREqAssignStatement }

  TJSOREqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;

  { TJSXOREqAssignStatement }

  TJSXOREqAssignStatement = Class(TJSAssignStatement)
    Class function OperatorToken : tjsToken; override;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TJSSimpleAssignStatement = Class(TJSAssignStatement);
  TJSMulEqAssignStatement = Class(TJSAssignStatement);
  TJSDivEqAssignStatement = Class(TJSAssignStatement);
  TJSModEqAssignStatement = Class(TJSAssignStatement);
  TJSAddEqAssignStatement = Class(TJSAssignStatement);
  TJSSubEqAssignStatement = Class(TJSAssignStatement);
  TJSLShiftEqAssignStatement = Class(TJSAssignStatement);
  TJSRShiftEqAssignStatement = Class(TJSAssignStatement);
  TJSURShiftEqAssignStatement = Class(TJSAssignStatement);
  TJSANDEqAssignStatement = Class(TJSAssignStatement);
  TJSOREqAssignStatement = Class(TJSAssignStatement);
  TJSXOREqAssignStatement = Class(TJSAssignStatement);
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

  { TJSVarDeclaration }

  TJSVarDeclaration = Class(TJSElement)
  private
    FInit: TJSElement;
    FName: String;
  Public
    Destructor Destroy; override;
    Property Name : String Read FName Write FName;
    Property Init : TJSElement Read FInit Write FInit;
  end;

  { TJSIfStatement }

  TJSIfStatement = Class(TJSElement)
  private
    FBFalse: TJSElement;
    FBTrue: TJSElement;
    FCond: TJSElement;
  Public
    Destructor Destroy; override;
    Property Cond : TJSElement Read FCond Write FCond;
    Property btrue : TJSElement Read FBTrue Write FBTrue;
    Property bfalse : TJSElement Read FBFalse Write FBFalse;
  end;

  { TJSWhileStatement }

  { TJSTargetStatement }

  TJSTargetStatement = Class(TJSElement)
  private
    FTarget: Cardinal;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FTargetName: TJSString;
  Public
    Property Target : Cardinal Read FTarget Write FTarget;
    Property TargetName : TJSString Read FTargetName Write FTargetName;
=======
  Public
    Property Target : Cardinal Read FTarget Write FTarget;
>>>>>>> graemeg/cpstrnew
=======
  Public
    Property Target : Cardinal Read FTarget Write FTarget;
>>>>>>> graemeg/cpstrnew
=======
  Public
    Property Target : Cardinal Read FTarget Write FTarget;
>>>>>>> graemeg/cpstrnew
=======
  Public
    Property Target : Cardinal Read FTarget Write FTarget;
>>>>>>> origin/cpstrnew
  end;

  { TJSBodyStatement }

  TJSBodyStatement = Class(TJSTargetStatement)
  private
    FBody: TJSElement;
  Public
    Destructor Destroy; override;
    Property body : TJSElement Read FBody Write FBody;
  end;

  { TJSCondLoopStatement }

  TJSCondLoopStatement = Class(TJSBodyStatement)
  private
    FCond: TJSElement;
  Public
    Destructor Destroy; override;
    Property Cond : TJSElement Read FCond Write FCond;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TJSWhileStatement = Class(TJSCondLoopStatement);
  TJSDoWhileStatement = Class(TJSWhileStatement);
=======
  TJSWhileStatement = Class(TJSCondLoopStatement)
  end;
>>>>>>> graemeg/cpstrnew
=======
  TJSWhileStatement = Class(TJSCondLoopStatement)
  end;
>>>>>>> graemeg/cpstrnew
=======
  TJSWhileStatement = Class(TJSCondLoopStatement)
  end;
>>>>>>> graemeg/cpstrnew
=======
  TJSWhileStatement = Class(TJSCondLoopStatement)
  end;
>>>>>>> origin/cpstrnew

  { TJSForStatement }

  TJSForStatement = Class(TJSCondLoopStatement)
  private
    FIncr: TJSElement;
    FInit: TJSElement;
  Public
    Destructor Destroy; override;
    Property Incr : TJSElement Read FIncr Write FIncr;
    Property Init : TJSElement Read FInit Write FInit;
  end;

  { TJSForInStatement }

  TJSForInStatement = Class(TJSBodyStatement)
  private
    FLhs: TJSElement;
    FList: TJSElement;
  Public
    Destructor Destroy; override;
    Property LHS : TJSElement Read FLHS Write FLHS;
    Property List : TJSElement Read FList Write FList;
  end;

  TJSContinueStatement = Class(TJSTargetStatement);

  TJSBreakStatement = Class(TJSTargetStatement);

  { TJSReturn }

  TJSReturnStatement = Class(TJSElement)
  private
    FExpr: TJSElement;
  Public
    Destructor Destroy; override;
    Property Expr : TJSElement Read FExpr Write FExpr;
  end;

  { TJSCaseElement }

  TJSCaseElement = Class(TCollectionItem)
  private
    FBody: TJSElement;
    FExpr: TJSelement;
  Public
    Destructor Destroy; override;
    Property Expr : TJSelement Read FExpr Write FExpr;
    Property Body : TJSElement Read FBody Write FBody;
  end;

  { TJSCaseElements }

  TJSCaseElements = Class(TCollection)
  private
    function GetC(AIndex : Integer): TJSCaseElement;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Function AddCase : TJSCaseElement;
    Property Cases[AIndex : Integer] : TJSCaseElement Read GetC ;default;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure SetC(AIndex : Integer; const AValue: TJSCaseElement);
  Public
    Function AddCase : TJSCaseElement;
    Property Cases[AIndex : Integer] : TJSCaseElement Read GetC Write SetC;
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

  { TJSSwitch }

  TJSSwitchStatement = Class(TJSTargetStatement)
  private
    FCases: TJSCaseElements;
    FCond: TJSelement;
    FDefault: TJSCaseElement;
  Public
    Constructor Create(ALine,ARow : Integer; const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Cond : TJSelement Read FCond Write FCond;
    Property Cases : TJSCaseElements Read FCases;
    Property TheDefault : TJSCaseelement Read FDefault Write FDefault;
  end;

  { TJSLabeledStatement }

  TJSLabeledStatement = Class(TJSUnary)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FLabel: TJSLabel;
    FTarget: Integer;
  Public
    Destructor Destroy; override;
    Property target: Integer Read FTarget Write FTarget;
    Property TheLabel : TJSLabel Read FLabel Write Flabel;
=======
    FTarget: Integer;
  Public
    Property target: Integer Read FTarget Write FTarget;
>>>>>>> graemeg/cpstrnew
=======
    FTarget: Integer;
  Public
    Property target: Integer Read FTarget Write FTarget;
>>>>>>> graemeg/cpstrnew
=======
    FTarget: Integer;
  Public
    Property target: Integer Read FTarget Write FTarget;
>>>>>>> graemeg/cpstrnew
=======
    FTarget: Integer;
  Public
    Property target: Integer Read FTarget Write FTarget;
>>>>>>> origin/cpstrnew
  end;

  { TJSTryStatement }

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


  { TJSFunction }

  TJSFunctionDeclarationStatement = Class(TJSelement)
  private
    FFuncDef: TJSFuncDef;
  Public
    Destructor Destroy; override;
    Property AFunction : TJSFuncDef Read FFuncDef Write FFuncDef;
  end;

  { TJSFunctionBody }

  TJSFunctionBody = Class(TJSUnary)
  private
    FisProgram: Boolean;
  Public
    Property isProgram : Boolean Read FisProgram Write FIsProgram;
  end;

  { TJSElementNode }

  TJSElementNode = Class(TCollectionItem)
  private
    FNode: TJSElement;
  Public
    Destructor Destroy; override;
    Property Node : TJSElement Read FNode Write FNode;
  end;

  { TJSElementNodes }

  TJSElementNodes = Class(TCollection)
  private
    function GetN(AIndex : Integer): TJSElementNode;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Public
    Function AddNode : TJSElementNode;
    Property Nodes[AIndex : Integer] : TJSElementNode Read GetN ; default;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure SetN(AIndex : Integer; const AValue: TJSElementNode);
  Public
    Function AddNode : TJSElementNode;
    Property Nodes[AIndex : Integer] : TJSElementNode Read GetN Write SetN; default;
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

  { TJSSourceElements }
  TJSSourceElements = Class(TJSElement)
  private
    FFunctions: TJSElementNodes;
    FStatements: TJSElementNodes;
    FVars: TJSElementNodes;
  Public
    Constructor Create(ALine,ARow : Integer; const ASource : String = ''); override;
    Destructor Destroy; override;
    Property Statements : TJSElementNodes Read FStatements;
    Property functions : TJSElementNodes Read FFunctions;
    Property Vars : TJSElementNodes Read FVars;
  end;


implementation

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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

{ TJSUnaryDeleteExpression }

Class function TJSUnaryDeleteExpression.PrefixOperatorToken : tjsToken;
begin
  Result:=tjsdelete;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
{ TJSElement }

constructor TJSElement.Create(ALine, ARow: Integer; Const ASource: String = '');
begin
  FLine:=ALine;
  FRow:=ARow;
  FSource:=ASource;
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

constructor TJSRegularExpressionLiteral.Create(ALine, ARow: Integer;
  const ASource: String);
begin
  inherited Create(ALine, ARow, ASource);
  FPattern:=TJSValue.Create;
  FPatternFlags:=TJSValue.Create;
end;

destructor TJSRegularExpressionLiteral.Destroy;
begin
  FreeAndNil(FPattern);
  FreeAndNil(FPatternFlags);
  Inherited Destroy;
=======
{ TJSElement }

constructor TJSElement.Create(ALine, ARow: Integer; Const ASource: String = '');
begin
  FLine:=ALine;
  FRow:=ARow;
  FSource:=ASource;
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

constructor TJSRegularExpressionLiteral.Create(ALine, ARow: Integer;
  const ASource: String);
begin
  inherited Create(ALine, ARow, ASource);
  FPattern:=TJSValue.Create;
  FPatternFlags:=TJSValue.Create;
end;

destructor TJSRegularExpressionLiteral.Destroy;
begin
  FreeAndNil(FPattern);
  FreeAndNil(FPatternFlags);
  Inherited Destroy;
>>>>>>> origin/cpstrnew
end;

{ TJSArrayLiteralElements }

function TJSArrayLiteralElements.GetE(AIndex : Integer): TJSArrayLiteralElement;
begin
  Result:=TJSArrayLiteralElement(Items[AIndex]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TJSArrayLiteralElements.SetE(AIndex : Integer;
  const AValue: TJSArrayLiteralElement);
begin
  Items[AIndex]:=AValue;
end;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TJSArrayLiteralElements.AddElement: TJSArrayLiteralElement;
begin
  Result:=TJSArrayLiteralElement(Add);
end;

{ TJSArrayLiteral }

constructor TJSArrayLiteral.Create(ALine, ARow: Integer; Const ASource: String = '');
begin
  inherited Create(ALine, ARow, ASource);
  FElements:=TJSArrayLiteralElements.Create(TJSArrayLiteralElement);
end;

destructor TJSArrayLiteral.Destroy;
begin
  FreeAndNil(FElements);
  inherited Destroy;
<<<<<<< HEAD
=======
{ TJSElement }

constructor TJSElement.Create(ALine, ARow: Integer; Const ASource: String = '');
begin
  FLine:=ALine;
  FRow:=ARow;
  FSource:=ASource;
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

constructor TJSRegularExpressionLiteral.Create(ALine, ARow: Integer;
  const ASource: String);
begin
  inherited Create(ALine, ARow, ASource);
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

procedure TJSArrayLiteralElements.SetE(AIndex : Integer;
  const AValue: TJSArrayLiteralElement);
begin
  Items[AIndex]:=AValue;
end;

function TJSArrayLiteralElements.AddElement: TJSArrayLiteralElement;
begin
  Result:=TJSArrayLiteralElement(Add);
end;

{ TJSArrayLiteral }

constructor TJSArrayLiteral.Create(ALine, ARow: Integer; Const ASource: String = '');
begin
  inherited Create(ALine, ARow, ASource);
  FElements:=TJSArrayLiteralElements.Create(TJSArrayLiteralElement);
end;

destructor TJSArrayLiteral.Destroy;
begin
  FreeAndNil(FElements);
  inherited Destroy;
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
end;

{ TJSObjectLiteralElements }

function TJSObjectLiteralElements.GetE(AIndex : Integer
  ): TJSObjectLiteralElement;
begin
  Result:=TJSObjectLiteralElement(Items[AIndex]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TJSObjectLiteralElements.SetE(AIndex : Integer;
  const AValue: TJSObjectLiteralElement);
begin
  Items[AIndex]:=AValue;
end;
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

function TJSObjectLiteralElements.AddElement: TJSObjectLiteralElement;
begin
  Result:=TJSObjectLiteralElement(Add);
end;

{ TJSObjectLiteral }

constructor TJSObjectLiteral.Create(ALine, ARow: Integer; const ASource: String = '');
begin
  inherited Create(ALine, ARow, ASource);
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
  FreeAndNil(Fexpr);
  inherited Destroy;
end;

{ TJSNewMemberExpression }

destructor TJSNewMemberExpression.Destroy;
begin
  FreeAndNil(FArgs);
  inherited Destroy;
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

{ TJSUnary }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
    if t in [tjsTypeOf,tjsVoid,tjsDelete,tjsThrow] then
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
=======
destructor TJSUnary.Destroy;
>>>>>>> graemeg/cpstrnew
=======
destructor TJSUnary.Destroy;
>>>>>>> graemeg/cpstrnew
=======
destructor TJSUnary.Destroy;
>>>>>>> graemeg/cpstrnew
=======
destructor TJSUnary.Destroy;
>>>>>>> origin/cpstrnew
begin
  FreeAndNil(FA);
  inherited Destroy;
end;

{ TJSBinary }

destructor TJSBinary.Destroy;
begin
  FreeAndNil(FB);
  FreeAndNil(FA);
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Destructor TJSAssignStatement.Destroy;
=======
destructor TJSAssignStatement.Destroy;
>>>>>>> graemeg/cpstrnew
=======
destructor TJSAssignStatement.Destroy;
>>>>>>> graemeg/cpstrnew
=======
destructor TJSAssignStatement.Destroy;
>>>>>>> graemeg/cpstrnew
=======
destructor TJSAssignStatement.Destroy;
>>>>>>> origin/cpstrnew
begin
  FreeAndNil(FLHS);
  FreeAndNil(FExpr);
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

constructor TJSSwitchStatement.Create(ALine, ARow: Integer; const ASource: String);
begin
  inherited Create(ALine, ARow, ASource);
  FCases:=TJSCaseElements.Create(TJSCaseElement);
end;

destructor TJSSwitchStatement.Destroy;
begin
  FreeAndNil(FCases);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  FreeAndNil(FDefault);
>>>>>>> graemeg/cpstrnew
=======
  FreeAndNil(FDefault);
>>>>>>> graemeg/cpstrnew
=======
  FreeAndNil(FDefault);
>>>>>>> graemeg/cpstrnew
=======
  FreeAndNil(FDefault);
>>>>>>> origin/cpstrnew
  FreeAndNil(FCond);
  inherited Destroy;
end;

{ TJSCaseElements }

function TJSCaseElements.GetC(AIndex : Integer): TJSCaseElement;
begin
  Result:=TJSCaseElement(Items[AIndex]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TJSCaseElements.SetC(AIndex : Integer; const AValue: TJSCaseElement);
begin
  Items[AIndex]:=AValue;
end;
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

constructor TJSSourceElements.Create(ALine, ARow: Integer; const ASource: String
  );
begin
  inherited Create(ALine, ARow, ASource);
  FStatements:=TJSElementNodes.Create(TJSElementNode);
  FFunctions:=TJSElementNodes.Create(TJSElementNode);
  FVars:=TJSElementNodes.Create(TJSElementNode);
end;

destructor TJSSourceElements.Destroy;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

Var
  i : integer;

begin
  FreeAndNil(FStatements);
  FreeAndNil(FFunctions);
  // Vars are owned by their statements, and those have been freed
  For I:=0 to FVars.Count-1 do
    FVars.Nodes[i].Node:=nil;
=======
begin
  FreeAndNil(FStatements);
  FreeAndNil(FFunctions);
>>>>>>> graemeg/cpstrnew
=======
begin
  FreeAndNil(FStatements);
  FreeAndNil(FFunctions);
>>>>>>> graemeg/cpstrnew
=======
begin
  FreeAndNil(FStatements);
  FreeAndNil(FFunctions);
>>>>>>> graemeg/cpstrnew
=======
begin
  FreeAndNil(FStatements);
  FreeAndNil(FFunctions);
>>>>>>> origin/cpstrnew
  FreeAndNil(FVars);
  inherited Destroy;
end;

{ TJSElementNodes }

function TJSElementNodes.GetN(AIndex : Integer): TJSElementNode;
begin
  Result:=TJSElementNode(Items[Aindex])
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TJSElementNodes.SetN(AIndex : Integer; const AValue: TJSElementNode);
begin
  Items[AIndex]:=Avalue;
end;
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

function TJSElementNodes.AddNode: TJSElementNode;
begin
  Result:=TJSElementNode(Add);
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FreeAndNil(FNode);
=======
  //FreeAndNil(FNode);
>>>>>>> graemeg/cpstrnew
=======
  //FreeAndNil(FNode);
>>>>>>> graemeg/cpstrnew
=======
  //FreeAndNil(FNode);
>>>>>>> graemeg/cpstrnew
=======
  //FreeAndNil(FNode);
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FreeAndNil(FBody);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
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

constructor TJSLiteral.Create(ALine, ARow: Integer; const ASource: String);
begin
  FValue:=TJSValue.Create;
  inherited Create(ALine, ARow, ASource);
end;

destructor TJSLiteral.Destroy;
begin
  FreeAndNil(FValue);
  Inherited;
end;

end.

