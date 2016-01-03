{
    This file is part of the Free Component Library

    Pascal parse tree classes
    Copyright (c) 2000-2005 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}
{$h+}

unit PasTree;

interface

uses Classes;

resourcestring
  // Parse tree node type names
  SPasTreeElement = 'generic element';
  SPasTreeSection = 'unit section';
  SPasTreeModule = 'module';
  SPasTreeUnit = 'unit';
  SPasTreeProgram = 'program';
  SPasTreePackage = 'package';
  SPasTreeResString = 'resource string';
  SPasTreeType = 'generic type';
  SPasTreePointerType = 'pointer type';
  SPasTreeAliasType = 'alias type';
  SPasTreeTypeAliasType = '"type" alias type';
  SPasTreeClassOfType = '"class of" type';
  SPasTreeRangeType = 'range type';
  SPasTreeArrayType = 'array type';
  SPasTreeFileType = 'file type';
  SPasTreeEnumValue = 'enumeration value';
  SPasTreeEnumType = 'enumeration type';
  SPasTreeSetType = 'set type';
  SPasTreeRecordType = 'record type';
  SPasStringType = 'string type';
  SPasTreeObjectType = 'object';
  SPasTreeClassType = 'class';
  SPasTreeInterfaceType = 'interface';
  SPasTreeGenericType = 'generic class';
  SPasTreeSpecializedType = 'specialized class type';
  SPasClassHelperType = 'Class helper type';
  SPasRecordHelperType = 'Record helper type';
  SPasTreeArgument = 'argument';
  SPasTreeProcedureType = 'procedure type';
  SPasTreeResultElement = 'function result';
  SPasTreeConstructorType = 'constructor type';
  SPasTreeDestructorType = 'destructor type';
  SPasTreeFunctionType = 'function type';
  SPasTreeUnresolvedTypeRef = 'unresolved type reference';
  SPasTreeVariable = 'variable';
  SPasTreeConst = 'constant';
  SPasTreeProperty = 'property';
  SPasTreeOverloadedProcedure = 'overloaded procedure';
  SPasTreeProcedure = 'procedure';
  SPasTreeFunction = 'function';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  SPasTreeOperator = 'operator';
  SPasTreeClassOperator = 'class operator';
  SPasTreeClassProcedure = 'class procedure';
  SPasTreeClassFunction = 'class function';
  SPasTreeClassConstructor = 'class constructor';
  SPasTreeClassDestructor = 'class destructor';
=======
  SPasTreeClassProcedure = 'class procedure';
  SPasTreeClassFunction = 'class function';
>>>>>>> graemeg/cpstrnew
=======
  SPasTreeClassProcedure = 'class procedure';
  SPasTreeClassFunction = 'class function';
>>>>>>> graemeg/cpstrnew
=======
  SPasTreeClassProcedure = 'class procedure';
  SPasTreeClassFunction = 'class function';
>>>>>>> graemeg/cpstrnew
=======
  SPasTreeClassProcedure = 'class procedure';
  SPasTreeClassFunction = 'class function';
>>>>>>> origin/cpstrnew
  SPasTreeConstructor = 'constructor';
  SPasTreeDestructor = 'destructor';
  SPasTreeProcedureImpl = 'procedure/function implementation';
  SPasTreeConstructorImpl = 'constructor implementation';
  SPasTreeDestructorImpl = 'destructor implementation';

type

  // Visitor pattern.
  TPassTreeVisitor = class;

  TPasElementBase = class
    procedure Accept(Visitor: TPassTreeVisitor); virtual; abstract;
  end;


  TPasModule = class;

  TPasMemberVisibility = (visDefault, visPrivate, visProtected, visPublic,
    visPublished, visAutomated,
    visStrictPrivate, visStrictProtected);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TCallingConvention = (ccDefault,ccRegister,ccPascal,ccCDecl,ccStdCall,ccOldFPCCall,ccSafeCall,ccSysCall);
  TPackMode = (pmNone,pmPacked,pmBitPacked);
=======
  TCallingConvention = (ccDefault,ccRegister,ccPascal,ccCDecl,ccStdCall,ccOldFPCCall,ccSafeCall);
>>>>>>> graemeg/cpstrnew
=======
  TCallingConvention = (ccDefault,ccRegister,ccPascal,ccCDecl,ccStdCall,ccOldFPCCall,ccSafeCall);
>>>>>>> graemeg/cpstrnew
=======
  TCallingConvention = (ccDefault,ccRegister,ccPascal,ccCDecl,ccStdCall,ccOldFPCCall,ccSafeCall);
>>>>>>> graemeg/cpstrnew
=======
  TCallingConvention = (ccDefault,ccRegister,ccPascal,ccCDecl,ccStdCall,ccOldFPCCall,ccSafeCall);
>>>>>>> origin/cpstrnew

  TPasMemberVisibilities = set of TPasMemberVisibility;
<<<<<<< HEAD
<<<<<<< HEAD
  TPasMemberHint = (hDeprecated,hLibrary,hPlatform,hExperimental,hUnimplemented);
=======
  TPasMemberHint = (hDeprecated,hLibrary,hPlatform);
>>>>>>> graemeg/fixes_2_2
=======
  TPasMemberHint = (hDeprecated,hLibrary,hPlatform);
>>>>>>> origin/fixes_2_2
  TPasMemberHints = set of TPasMemberHint; 

  TPTreeElement = class of TPasElement;

  { TPasElement }

  TPasElement = class(TPasElementBase)
  private
    FData: TObject;
    FDocComment: String;
    FRefCount: LongWord;
    FName: string;
    FParent: TPasElement;
    FHints : TPasMemberHints;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FHintMessage : String;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  protected
    procedure ProcessHints(const ASemiColonPrefix: boolean; var AResult: string); virtual;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  public
    SourceFilename: string;
    SourceLinenumber: Integer;
    Visibility: TPasMemberVisibility;
  public
    constructor Create(const AName: string; AParent: TPasElement); virtual;
    procedure AddRef;
    procedure Release;
    function FullPath: string;
    function ParentPath: string;
    function FullName: string; virtual;         // Name including parent's names
    function PathName: string; virtual;         // = Module.Name + FullName
    function GetModule: TPasModule;
    function ElementTypeName: string; virtual;
    Function HintsString : String;
    function GetDeclaration(full : Boolean) : string; virtual;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure Accept(Visitor: TPassTreeVisitor); override;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    property RefCount: LongWord read FRefCount;
    property Name: string read FName write FName;
    property Parent: TPasElement read FParent;
    Property Hints : TPasMemberHints Read FHints Write FHints;
<<<<<<< HEAD
<<<<<<< HEAD
    Property CustomData : TObject Read FData Write FData;
    Property HintMessage : String Read FHintMessage Write FHintMessage;
    Property DocComment : String Read FDocComment Write FDocComment;
  end;

  TPasExprKind = (pekIdent, pekNumber, pekString, pekSet, pekNil, pekBoolConst, pekRange,
     pekUnary, pekBinary, pekFuncParams, pekArrayParams, pekListOfExp, pekInherited, pekSelf);

  TExprOpCode = (eopNone,
                 eopAdd,eopSubtract,eopMultiply,eopDivide, eopDiv,eopMod, eopPower,// arithmetic
                 eopShr,eopShl, // bit operations
                 eopNot,eopAnd,eopOr,eopXor, // logical/bit
                 eopEqual, eopNotEqual,  // Logical
                 eopLessThan,eopGreaterThan, eopLessthanEqual,eopGreaterThanEqual, // ordering
                 eopIn,eopIs,eopAs, eopSymmetricaldifference, // Specials
                 eopAddress, eopDeref, // Pointers
                 eopSubIdent); // SomeRec.A, A is subIdent of SomeRec

  { TPasExpr }

  TPasExpr = class(TPasElement)
    Kind      : TPasExprKind;
    OpCode    : TexprOpcode;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TexprOpcode); virtual; overload;
=======
=======
>>>>>>> origin/fixes_2_2
  end;

  TPasSection = class(TPasElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    procedure AddUnitToUsesList(const AUnitName: string);
  public
    UsesList: TList;            // TPasUnresolvedTypeRef or TPasModule elements
    Declarations, ResStrings, Types, Consts, Classes,
    Functions, Variables, Properties: TList;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  TUnaryExpr = class(TPasExpr)
    Operand   : TPasExpr;
    constructor Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
=======
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    InterfaceSection, ImplementationSection: TPasSection;
    PackageName: string;
>>>>>>> graemeg/fixes_2_2
  end;

  { TBinaryExpr }

  TBinaryExpr = class(TPasExpr)
    left      : TPasExpr;
    right     : TPasExpr;
    constructor Create(AParent : TPasElement; xleft, xright: TPasExpr; AOpCode: TExprOpCode); overload;
    constructor CreateRange(AParent : TPasElement; xleft, xright: TPasExpr); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
  end;

  TPrimitiveExpr = class(TPasExpr)
    Value     : AnsiString;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : Ansistring); overload;
    function GetDeclaration(full : Boolean) : string; override;
=======
    function ElementTypeName: string; override;
  public
    Modules: TList;     // List of TPasModule objects
  end;

  TPasResString = class(TPasElement)
  public
    function ElementTypeName: string; override;
    function GetDeclaration(full : Boolean) : string; Override;
  public
    Value: string;
>>>>>>> graemeg/fixes_2_2
  end;
  
  TBoolConstExpr = class(TPasExpr)
    Value     : Boolean;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

<<<<<<< HEAD
  { TNilExpr }

  TNilExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TInheritedExpr }

  TInheritedExpr = class(TPasExpr)
  Public
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
=======
  TPasPointerType = class(TPasType)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : Boolean): string; override;
  public
    DestType: TPasType;
  end;

  TPasAliasType = class(TPasType)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : Boolean): string; override;
  public
    DestType: TPasType;
>>>>>>> graemeg/fixes_2_2
  end;

  { TSelfExpr }

  TSelfExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TParamsExpr }

  TParamsExpr = class(TPasExpr)
    Value     : TPasExpr;
    Params    : array of TPasExpr;
    {pekArray, pekFuncCall, pekSet}
    constructor Create(AParent : TPasElement; AKind: TPasExprKind); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
    procedure AddParam(xp: TPasExpr);
  end;

  { TRecordValues }

<<<<<<< HEAD
  TRecordValuesItem = record
    Name      : AnsiString;
    ValueExp  : TPasExpr;
=======
  TPasRangeType = class(TPasType)
  public
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    RangeStart, RangeEnd: string;
>>>>>>> graemeg/fixes_2_2
  end;

  TRecordValues = class(TPasExpr)
    Fields    : array of TRecordValuesItem;
    constructor Create(AParent : TPasElement); overload;
    destructor Destroy; override;
<<<<<<< HEAD
    procedure AddField(const AName: AnsiString; Value: TPasExpr);
    function GetDeclaration(full : Boolean) : string; override;
=======
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    IndexRange : string;
    IsPacked : Boolean;          // 12/04/04 - Dave - Added
    ElType: TPasType;
>>>>>>> graemeg/fixes_2_2
  end;

  { TArrayValues }

  TArrayValues = class(TPasExpr)
    Values    : array of TPasExpr;
    constructor Create(AParent : TPasElement); overload;
    destructor Destroy; override;
<<<<<<< HEAD
    procedure AddValues(AValue: TPasExpr);
    function GetDeclaration(full : Boolean) : string; override;
  end;

<<<<<<< HEAD
=======
  TPasExprKind = (pekIdent, pekNumber, pekString, pekSet, pekNil, pekBoolConst, pekRange,
     pekUnary, pekBinary, pekFuncParams, pekArrayParams, pekListOfExp, pekInherited, pekSelf);

  TExprOpCode = (eopNone,
                 eopAdd,eopSubtract,eopMultiply,eopDivide, eopDiv,eopMod, eopPower,// arithmetic
                 eopShr,eopSHl, // bit operations
                 eopNot,eopAnd,eopOr,eopXor, // logical/bit
                 eopEqual, eopNotEqual,  // Logical
                 eopLessThan,eopGreaterThan, eopLessthanEqual,eopGreaterThanEqual, // ordering
                 eopIn,eopIs,eopAs, eopSymmetricaldifference, // Specials
                 eopAddress, eopDeref, // Pointers
                 eopSubIdent); // SomeRec.A, A is subIdent of SomeRec

  { TPasExpr }

  TPasExpr = class(TPasElement)
    Kind      : TPasExprKind;
    OpCode    : TexprOpcode;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TexprOpcode); virtual; overload;
=======
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    ElType: TPasType;
  end;

  TPasEnumValue = class(TPasElement)
  public
    function ElementTypeName: string; override;
  public
    IsValueUsed: Boolean;
    Value: Integer;
    AssignedValue : string;
>>>>>>> graemeg/fixes_2_2
  end;

  TUnaryExpr = class(TPasExpr)
    Operand   : TPasExpr;
    constructor Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
=======
     function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    Procedure GetEnumNames(Names : TStrings);
  public
    Values: TList;      // List of TPasEnumValue objects
>>>>>>> graemeg/fixes_2_2
  end;

  { TBinaryExpr }

  TBinaryExpr = class(TPasExpr)
    left      : TPasExpr;
    right     : TPasExpr;
    constructor Create(AParent : TPasElement; xleft, xright: TPasExpr; AOpCode: TExprOpCode); overload;
    constructor CreateRange(AParent : TPasElement; xleft, xright: TPasExpr); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
=======
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    EnumType: TPasType;
>>>>>>> graemeg/fixes_2_2
  end;

  TPrimitiveExpr = class(TPasExpr)
    Value     : AnsiString;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : Ansistring); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;
  
  TBoolConstExpr = class(TPasExpr)
    Value     : Boolean;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

<<<<<<< HEAD
  { TNilExpr }

  TNilExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TInheritedExpr }

  TInheritedExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
=======
  TPasVariant = class(TPasElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
  public
    Values: TStringList;
    Members: TPasRecordType;
  end;

  TPasRecordType = class(TPasType)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    IsPacked: Boolean;
    IsBitPacked : Boolean;
    Members: TList;     // array of TPasVariable elements
    VariantName: string;
    VariantType: TPasType;
    Variants: TList;	// array of TPasVariant elements, may be nil!
>>>>>>> graemeg/fixes_2_2
  end;

  { TSelfExpr }

  TSelfExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TParamsExpr }

  TParamsExpr = class(TPasExpr)
    Value     : TPasExpr;
    Params    : array of TPasExpr;
    {pekArray, pekFuncCall, pekSet}
    constructor Create(AParent : TPasElement; AKind: TPasExprKind); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
    procedure AddParam(xp: TPasExpr);
=======
    function ElementTypeName: string; override;
  public
    ObjKind: TPasObjKind;
    AncestorType: TPasType;     // TPasClassType or TPasUnresolvedTypeRef
    IsPacked: Boolean;        // 12/04/04 - Dave - Added
    IsForward : Boolean;
    Members: TList;     // array of TPasElement objects
    InterfaceGUID : string; // 15/06/07 - Inoussa
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  { TRecordValues }

<<<<<<< HEAD
  TRecordValuesItem = record
    Name      : AnsiString;
    ValueExp  : TPasExpr;
=======
  TPasArgument = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    Access: TArgumentAccess;
    ArgType: TPasType;
    Value: string;
>>>>>>> graemeg/fixes_2_2
  end;

  TRecordValues = class(TPasExpr)
    Fields    : array of TRecordValuesItem;
    constructor Create(AParent : TPasElement); overload;
    destructor Destroy; override;
<<<<<<< HEAD
    procedure AddField(const AName: AnsiString; Value: TPasExpr);
    function GetDeclaration(full : Boolean) : string; override;
=======
    class function TypeName: string; virtual;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure GetArguments(List : TStrings);
    function CreateArgument(const AName, AUnresolvedTypeName: string):TPasArgument;
  public
    IsOfObject: Boolean;
    Args: TList;        // List of TPasArgument objects
>>>>>>> graemeg/fixes_2_2
  end;

<<<<<<< HEAD
=======
  TPasExprKind = (pekIdent, pekNumber, pekString, pekSet, pekNil, pekBoolConst, pekRange,
     pekUnary, pekBinary, pekFuncParams, pekArrayParams, pekListOfExp, pekInherited, pekSelf);

  TExprOpCode = (eopNone,
                 eopAdd,eopSubtract,eopMultiply,eopDivide, eopDiv,eopMod, eopPower,// arithmetic
                 eopShr,eopSHl, // bit operations
                 eopNot,eopAnd,eopOr,eopXor, // logical/bit
                 eopEqual, eopNotEqual,  // Logical
                 eopLessThan,eopGreaterThan, eopLessthanEqual,eopGreaterThanEqual, // ordering
                 eopIn,eopIs,eopAs, eopSymmetricaldifference, // Specials
                 eopAddress, eopDeref, // Pointers
                 eopSubIdent); // SomeRec.A, A is subIdent of SomeRec

  { TPasExpr }

  TPasExpr = class(TPasElement)
    Kind      : TPasExprKind;
    OpCode    : TexprOpcode;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TexprOpcode); virtual; overload;
  end;

  TUnaryExpr = class(TPasExpr)
    Operand   : TPasExpr;
    constructor Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
=======
    function ElementTypeName : string; override;
  public
    ResultType: TPasType;
>>>>>>> graemeg/fixes_2_2
  end;

  { TBinaryExpr }

  TBinaryExpr = class(TPasExpr)
    left      : TPasExpr;
    right     : TPasExpr;
    constructor Create(AParent : TPasElement; xleft, xright: TPasExpr; AOpCode: TExprOpCode); overload;
    constructor CreateRange(AParent : TPasElement; xleft, xright: TPasExpr); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
=======
    class function TypeName: string; override;
    function ElementTypeName: string; override;
    function GetDeclaration(Full : boolean) : string; override;
  public
    ResultEl: TPasResultElement;
>>>>>>> graemeg/fixes_2_2
  end;

  TPrimitiveExpr = class(TPasExpr)
    Value     : AnsiString;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : Ansistring); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;
  
  TBoolConstExpr = class(TPasExpr)
    Value     : Boolean;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TNilExpr }

  TNilExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

<<<<<<< HEAD
  { TInheritedExpr }

  TInheritedExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
=======
  TPasVariable = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    VarType: TPasType;
    Value: string;
    Modifiers : string;
    AbsoluteLocation : String;
>>>>>>> graemeg/fixes_2_2
  end;

  { TSelfExpr }

  TSelfExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TParamsExpr }

  TParamsExpr = class(TPasExpr)
    Value     : TPasExpr;
    Params    : array of TPasExpr;
    {pekArray, pekFuncCall, pekSet}
    constructor Create(AParent : TPasElement; AKind: TPasExprKind); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
    procedure AddParam(xp: TPasExpr);
=======
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    Args: TList;        // List of TPasArgument objects
    IndexValue, ReadAccessorName, WriteAccessorName,ImplementsName,
      StoredAccessorName, DefaultValue: string;
    IsDefault, IsNodefault: Boolean;
>>>>>>> graemeg/fixes_2_2
  end;

<<<<<<< HEAD
=======
  TPasExprKind = (pekIdent, pekNumber, pekString, pekSet, pekNil, pekBoolConst, pekRange,
     pekUnary, pekBinary, pekFuncParams, pekArrayParams, pekListOfExp, pekInherited, pekSelf);

  TExprOpCode = (eopNone,
                 eopAdd,eopSubtract,eopMultiply,eopDivide, eopDiv,eopMod, eopPower,// arithmetic
                 eopShr,eopSHl, // bit operations
                 eopNot,eopAnd,eopOr,eopXor, // logical/bit
                 eopEqual, eopNotEqual,  // Logical
                 eopLessThan,eopGreaterThan, eopLessthanEqual,eopGreaterThanEqual, // ordering
                 eopIn,eopIs,eopAs, eopSymmetricaldifference, // Specials
                 eopAddress, eopDeref, // Pointers
                 eopSubIdent); // SomeRec.A, A is subIdent of SomeRec

  { TPasExpr }

  TPasExpr = class(TPasElement)
    Kind      : TPasExprKind;
    OpCode    : TexprOpcode;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TexprOpcode); virtual; overload;
  end;

  TUnaryExpr = class(TPasExpr)
    Operand   : TPasExpr;
    constructor Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
=======
    function ElementTypeName: string; override;
    function TypeName: string; override;
  public
    Overloads: TList;           // List of TPasProcedure nodes
>>>>>>> graemeg/fixes_2_2
  end;

  { TBinaryExpr }

  TBinaryExpr = class(TPasExpr)
    left      : TPasExpr;
    right     : TPasExpr;
    constructor Create(AParent : TPasElement; xleft, xright: TPasExpr; AOpCode: TExprOpCode); overload;
    constructor CreateRange(AParent : TPasElement; xleft, xright: TPasExpr); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
=======
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetDeclaration(full: Boolean): string; override;
    procedure GetModifiers(List: TStrings);
  public
    ProcType: TPasProcedureType;
    IsVirtual, IsDynamic, IsAbstract, IsOverride,
      IsOverload, IsMessage, isReintroduced, isStatic: Boolean;
>>>>>>> graemeg/fixes_2_2
  end;

  TPrimitiveExpr = class(TPasExpr)
    Value     : AnsiString;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : Ansistring); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;
  
  TBoolConstExpr = class(TPasExpr)
    Value     : Boolean;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TNilExpr }

  TNilExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TInheritedExpr }

  TInheritedExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TSelfExpr }

  TSelfExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TParamsExpr }

  TParamsExpr = class(TPasExpr)
    Value     : TPasExpr;
    Params    : array of TPasExpr;
    {pekArray, pekFuncCall, pekSet}
    constructor Create(AParent : TPasElement; AKind: TPasExprKind); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
    procedure AddParam(xp: TPasExpr);
=======
    function ElementTypeName: string; override;
    function TypeName: string; virtual;
  public
    ProcType: TPasProcedureType;
    Locals: TList;
    Body: TPasImplBlock;
>>>>>>> graemeg/fixes_2_2
  end;

>>>>>>> graemeg/cpstrnew
  { TRecordValues }

  TRecordValuesItem = record
    Name      : AnsiString;
    ValueExp  : TPasExpr;
  end;

  TRecordValues = class(TPasExpr)
    Fields    : array of TRecordValuesItem;
    constructor Create(AParent : TPasElement); overload;
    destructor Destroy; override;
    procedure AddField(const AName: AnsiString; Value: TPasExpr);
    function GetDeclaration(full : Boolean) : string; override;
  end;

<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
  { TArrayValues }

  TArrayValues = class(TPasExpr)
    Values    : array of TPasExpr;
    constructor Create(AParent : TPasElement); overload;
    destructor Destroy; override;
    procedure AddValues(AValue: TPasExpr);
    function GetDeclaration(full : Boolean) : string; override;
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
  TPasExprKind = (pekIdent, pekNumber, pekString, pekSet, pekNil, pekBoolConst, pekRange,
     pekUnary, pekBinary, pekFuncParams, pekArrayParams, pekListOfExp, pekInherited, pekSelf);

  TExprOpCode = (eopNone,
                 eopAdd,eopSubtract,eopMultiply,eopDivide, eopDiv,eopMod, eopPower,// arithmetic
                 eopShr,eopSHl, // bit operations
                 eopNot,eopAnd,eopOr,eopXor, // logical/bit
                 eopEqual, eopNotEqual,  // Logical
                 eopLessThan,eopGreaterThan, eopLessthanEqual,eopGreaterThanEqual, // ordering
                 eopIn,eopIs,eopAs, eopSymmetricaldifference, // Specials
                 eopAddress, eopDeref, // Pointers
                 eopSubIdent); // SomeRec.A, A is subIdent of SomeRec

  { TPasExpr }

  TPasExpr = class(TPasElement)
    Kind      : TPasExprKind;
    OpCode    : TexprOpcode;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TexprOpcode); virtual; overload;
  end;

  TUnaryExpr = class(TPasExpr)
    Operand   : TPasExpr;
    constructor Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  public
    Commands: TStrings;
>>>>>>> graemeg/fixes_2_2
  end;

  { TBinaryExpr }

  TBinaryExpr = class(TPasExpr)
    left      : TPasExpr;
    right     : TPasExpr;
    constructor Create(AParent : TPasElement; xleft, xright: TPasExpr; AOpCode: TExprOpCode); overload;
    constructor CreateRange(AParent : TPasElement; xleft, xright: TPasExpr); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
  end;

  TPrimitiveExpr = class(TPasExpr)
    Value     : AnsiString;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : Ansistring); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;
  
  TBoolConstExpr = class(TPasExpr)
    Value     : Boolean;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TNilExpr }

  TNilExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
=======
=======
>>>>>>> origin/fixes_2_2
  public
    Condition: string;
    IfBranch, ElseBranch: TPasImplElement;
  end;

  TPasImplForLoop = class(TPasImplElement)
  public
    destructor Destroy; override;
  public
    Variable: TPasVariable;
    StartValue, EndValue: string;
    Body: TPasImplElement;
  end;

  TPasImplBlock = class(TPasImplElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function AddCommand(const ACommand: string): TPasImplCommand;
    function AddCommands: TPasImplCommands;
    function AddIfElse(const ACondition: string): TPasImplIfElse;
    function AddForLoop(AVar: TPasVariable;
      const AStartValue, AEndValue: string): TPasImplForLoop;
  public
    Elements: TList;    // TPasImplElement objects
>>>>>>> graemeg/fixes_2_2
  end;

  { TInheritedExpr }

  TInheritedExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TSelfExpr }

  TSelfExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TParamsExpr }

  TParamsExpr = class(TPasExpr)
    Value     : TPasExpr;
    Params    : array of TPasExpr;
    {pekArray, pekFuncCall, pekSet}
    constructor Create(AParent : TPasElement; AKind: TPasExprKind); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
    procedure AddParam(xp: TPasExpr);
  end;

  { TRecordValues }

<<<<<<< HEAD
  TRecordValuesItem = record
    Name      : AnsiString;
    ValueExp  : TPasExpr;
  end;
=======
{$IFNDEF FPC}
  const
    LineEnding = sLineBreak;
{$ENDIF}
>>>>>>> origin/fixes_2_2

  TRecordValues = class(TPasExpr)
    Fields    : array of TRecordValuesItem;
    constructor Create(AParent : TPasElement); overload;
    destructor Destroy; override;
    procedure AddField(const AName: AnsiString; Value: TPasExpr);
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TArrayValues }

  TArrayValues = class(TPasExpr)
    Values    : array of TPasExpr;
    constructor Create(AParent : TPasElement); overload;
    destructor Destroy; override;
    procedure AddValues(AValue: TPasExpr);
    function GetDeclaration(full : Boolean) : string; override;
  end;

>>>>>>> origin/cpstrnew
  { TPasDeclarations }

  TPasDeclarations = class(TPasElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  public
    Declarations, ResStrings, Types, Consts, Classes,
    Functions, Variables, Properties, ExportSymbols: TFPList;
  end;

  { TPasSection }

  TPasSection = class(TPasDeclarations)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddUnitToUsesList(const AUnitName: string);
  public
    UsesList: TFPList;            // TPasUnresolvedTypeRef or TPasModule elements
  end;

  { TInterfaceSection }

  TInterfaceSection = class(TPasSection)
  end;

  { TImplementationSection }

  TImplementationSection = class(TPasSection)
  end;

  TProgramSection = class(TImplementationSection)
  end;

  TLibrarySection = class(TImplementationSection)
  end;

<<<<<<< HEAD
=======
  { TPasSection }

  TPasSection = class(TPasDeclarations)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddUnitToUsesList(const AUnitName: string);
  public
    UsesList: TList;            // TPasUnresolvedTypeRef or TPasModule elements
  end;

=======
  public
=======
  public
>>>>>>> graemeg/cpstrnew
=======
  public
>>>>>>> origin/cpstrnew
    Declarations, ResStrings, Types, Consts, Classes,
    Functions, Variables, Properties: TList;
  end;

  { TPasSection }

  TPasSection = class(TPasDeclarations)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddUnitToUsesList(const AUnitName: string);
  public
    UsesList: TList;            // TPasUnresolvedTypeRef or TPasModule elements
  end;

<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { TInterfaceSection }

  TInterfaceSection = class(TPasSection)
  end;

  { TImplementationSection }

  TImplementationSection = class(TPasSection)
  end;

  TProgramSection = class(TPasSection)
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
  TInitializationSection = class;
  TFinalizationSection = class;

  { TPasModule }

  TPasModule = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    InterfaceSection: TInterfaceSection;
    ImplementationSection: TImplementationSection;
    InitializationSection: TInitializationSection;
    FinalizationSection: TFinalizationSection;
    PackageName: string;
    Filename   : String;  // the IN filename, only written when not empty.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  end;

  { TPasProgram }

  { TPasUnitModule }

  TPasUnitModule = Class(TPasModule)
    function ElementTypeName: string; override;
  end;

  TPasProgram = class(TPasModule)
  Public
    destructor Destroy; override;
    function ElementTypeName: string; override;
  Public
    ProgramSection: TProgramSection;
    InputFile,OutPutFile : String;
  end;

  { TPasLibrary }

  TPasLibrary = class(TPasModule)
  Public
    destructor Destroy; override;
    function ElementTypeName: string; override;
  Public
    LibrarySection: TLibrarySection;
    InputFile,OutPutFile : String;
=======
>>>>>>> graemeg/cpstrnew
  end;
=======
  end;

  { TPasProgram }

  TPasProgram = class(TPasModule);
>>>>>>> graemeg/cpstrnew

  { TPasPackage }

=======
  end;

>>>>>>> graemeg/cpstrnew
=======
  end;

>>>>>>> origin/cpstrnew
  { TPasProgram }

  TPasProgram = class(TPasModule);

  { TPasPackage }

  TPasPackage = class(TPasElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
  public
    Modules: TFPList;     // List of TPasModule objects
  end;

  { TPasResString }

  TPasResString = class(TPasElement)
  public
    Destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : Boolean) : string; Override;
  public
    Expr: TPasExpr;
  end;

  { TPasType }

  TPasType = class(TPasElement)
  public
    function ElementTypeName: string; override;
  end;

  { TPasPointerType }

  TPasPointerType = class(TPasType)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : Boolean): string; override;
  public
    DestType: TPasType;
  end;

  { TPasAliasType }

  TPasAliasType = class(TPasType)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : Boolean): string; override;
  public
    DestType: TPasType;
  end;

  { TPasTypeAliasType }

  TPasTypeAliasType = class(TPasAliasType)
  public
    function ElementTypeName: string; override;
  end;

  { TPasClassOfType }

  TPasClassOfType = class(TPasAliasType)
  public
    function ElementTypeName: string; override;
    function GetDeclaration(full: boolean) : string; override;
  end;


  { TPasRangeType }

  TPasRangeType = class(TPasType)
  public
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    RangeExpr : TBinaryExpr;
    Destructor Destroy; override;
    Function RangeStart : String;
    Function RangeEnd : String;
  end;

  { TPasArrayType }

  TPasArrayType = class(TPasType)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    IndexRange : string;
    PackMode : TPackMode;
    ElType: TPasType;
    Function IsPacked : Boolean;
  end;

  { TPasFileType }

  TPasFileType = class(TPasType)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    ElType: TPasType;
  end;

  { TPasEnumValue }

  TPasEnumValue = class(TPasElement)
  public
    function ElementTypeName: string; override;
  public
    Value: TPasExpr;
    Destructor Destroy; override;
    Function AssignedValue : string;
  end;

  { TPasEnumType }

  TPasEnumType = class(TPasType)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
     function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    Procedure GetEnumNames(Names : TStrings);
  public
    Values: TFPList;      // List of TPasEnumValue objects
  end;

  { TPasSetType }

  TPasSetType = class(TPasType)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    EnumType: TPasType;
  end;

  TPasRecordType = class;

  { TPasVariant }

  TPasVariant = class(TPasElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    Values: TFPList;
    Members: TPasRecordType;
  end;

  { TPasRecordType }

  TPasRecordType = class(TPasType)
  private
    procedure GetMembers(S: TStrings);
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    PackMode : TPackMode;
    Members: TFPList;     // array of TPasVariable elements
    VariantName: string;
    VariantType: TPasType;
    Variants: TFPList;	// array of TPasVariant elements, may be nil!
    Function IsPacked: Boolean;
    Function IsBitPacked : Boolean;
    Function IsAdvancedRecord : Boolean;
  end;

  TPasGenericTemplateType = Class(TPasElement);
  TPasObjKind = (okObject, okClass, okInterface, okGeneric, okSpecialize,
                 okClassHelper,okRecordHelper,okTypeHelper);

  { TPasClassType }

  { TPasClassType }

  { TPasClassType }

  { TPasClassType }

  { TPasClassType }

  TPasClassType = class(TPasType)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
  public
    PackMode : TPackMode;
    ObjKind: TPasObjKind;
    AncestorType: TPasType;     // TPasClassType or TPasUnresolvedTypeRef
    HelperForType: TPasType;     // TPasClassType or TPasUnresolvedTypeRef
    IsForward : Boolean;
    IsShortDefinition: Boolean;//class(anchestor); without end
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    GUIDExpr : TPasExpr;
    Members: TFPList;     // array of TPasElement objects
    ClassVars: TFPList;   // class vars
    Modifiers: TStringList;
    Interfaces : TFPList;
    GenericTemplateTypes : TFPList;
    Function FindMember(MemberClass : TPTreeElement; Const MemberName : String) : TPasElement;
    Function FindMemberInAncestors(MemberClass : TPTreeElement; Const MemberName : String) : TPasElement;
    Function IsPacked : Boolean;
    Function InterfaceGUID : string;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Members: TList;     // array of TPasElement objects
    InterfaceGUID : string; // 15/06/07 - Inoussa

    ClassVars: TList;   // class vars
    Modifiers: TStringList;
    Interfaces : TList;
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



  TArgumentAccess = (argDefault, argConst, argVar, argOut, argConstRef);

  { TPasArgument }

  { TPasArgument }

  { TPasArgument }

  { TPasArgument }

  { TPasArgument }

  TPasArgument = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    Access: TArgumentAccess;
    ArgType: TPasType;
    ValueExpr: TPasExpr;
    Function Value : String;
  end;

  { TPasProcedureType }

  TPasProcedureType = class(TPasType)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    class function TypeName: string; virtual;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure GetArguments(List : TStrings);
    function CreateArgument(const AName, AUnresolvedTypeName: string):TPasArgument;
  public
    IsOfObject: Boolean;
    IsNested : Boolean;
    Args: TFPList;        // List of TPasArgument objects
    CallingConvention : TCallingConvention;
  end;

  { TPasResultElement }

  TPasResultElement = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName : string; override;
  public
    ResultType: TPasType;
  end;

  { TPasFunctionType }

  TPasFunctionType = class(TPasProcedureType)
  public
    destructor Destroy; override;
    class function TypeName: string; override;
    function ElementTypeName: string; override;
    function GetDeclaration(Full : boolean) : string; override;
  public
    ResultEl: TPasResultElement;
  end;

  TPasUnresolvedSymbolRef = class(TPasType)
  end;

  TPasUnresolvedTypeRef = class(TPasUnresolvedSymbolRef)
  public
    // Typerefs cannot be parented! -> AParent _must_ be NIL
    constructor Create(const AName: string; AParent: TPasElement); override;
    function ElementTypeName: string; override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  { TPasUnresolvedUnitRef }

  TPasUnresolvedUnitRef = Class(TPasUnresolvedSymbolRef)
    function ElementTypeName: string; override;
  Public
    FileName : string;
  end;

  { TPasStringType }

  TPasStringType = class(TPasUnresolvedTypeRef)
  public
    LengthExpr : String;
    function ElementTypeName: string; override;
  end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { TPasTypeRef }

  TPasTypeRef = class(TPasUnresolvedTypeRef)
  public
  public
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
    // function GetDeclaration(full : Boolean): string; override;
>>>>>>> graemeg/cpstrnew
    RefType: TPasType;
  end;

  { TPasVariable }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TVariableModifier = (vmCVar, vmExternal, vmPublic, vmExport, vmClass);
  TVariableModifiers = set of TVariableModifier;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  TPasVariable = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    VarType: TPasType;
    VarModifiers : TVariableModifiers;
    LibraryName,ExportName : string;
    Modifiers : string;
    AbsoluteLocation : String;
    Expr: TPasExpr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Function Value : String;
  end;

  { TPasExportSymbol }

  TPasExportSymbol = class(TPasElement)
    ExportName : TPasExpr;
    Exportindex : TPasExpr;
    Destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  end;

  { TPasConst }

  TPasConst = class(TPasVariable)
  public
  public
    function ElementTypeName: string; override;
  end;

  { TPasProperty }

  TPasProperty = class(TPasVariable)
  Public
    FResolvedType : TPasType;
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    IndexExpr,
    DefaultExpr : TPasExpr;
    Args: TFPList;        // List of TPasArgument objects
    ReadAccessorName, WriteAccessorName,ImplementsName,
      StoredAccessorName: string;
    IsClass, IsDefault, IsNodefault: Boolean;
    Function ResolvedType : TPasType;
    Function IndexValue : String;
    Function DefaultValue : string;
  end;

  { TPasProcedureBase }

  TPasProcedureBase = class(TPasElement)
  public
    function TypeName: string; virtual; abstract;
  end;

  { TPasOverloadedProc }

  TPasOverloadedProc = class(TPasProcedureBase)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function TypeName: string; override;
  public
    Overloads: TFPList;           // List of TPasProcedure nodes
  end;

  TProcedureModifier = (pmVirtual, pmDynamic, pmAbstract, pmOverride,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        pmExport, pmOverload, pmMessage, pmReintroduce,
                        pmStatic,pmInline,pmAssembler,pmVarargs, pmPublic,
                        pmCompilerProc,pmExternal,pmForward);
  TProcedureModifiers = Set of TProcedureModifier;
  TProcedureMessageType = (pmtNone,pmtInteger,pmtString);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                        pmExported, pmOverload, pmMessage, pmReintroduce,
                        pmStatic,pmInline,pmAssembler,pmVarargs,
                        pmCompilerProc,pmExternal,pmExtdecl,pmForward);
  TProcedureModifiers = Set of TProcedureModifier;
  TProcedureMessageType = (pmtInteger,pmtString);
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
                        
  TProcedureBody = class;

  TPasProcedure = class(TPasProcedureBase)
  Private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FModifiers : TProcedureModifiers;
    FMessageName : String;
    FMessageType : TProcedureMessageType;
    function GetCallingConvention: TCallingConvention;
    procedure SetCallingConvention(AValue: TCallingConvention);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    FCallingConvention : TCallingConvention;
    FModifiers : TProcedureModifiers;
    FMessageName : String;
    FMessageType : TProcedureMessageType;
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
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetDeclaration(full: Boolean): string; override;
    procedure GetModifiers(List: TStrings);
  public
    ProcType : TPasProcedureType;
    Body : TProcedureBody;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    PublicName,
    LibrarySymbolName,
    LibraryExpr : TPasExpr;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Procedure AddModifier(AModifier : TProcedureModifier);
    Function IsVirtual : Boolean;
    Function IsDynamic : Boolean;
    Function IsAbstract : Boolean;
    Function IsOverride : Boolean;
    Function IsExported : Boolean;
    Function IsExternal : Boolean;
    Function IsOverload : Boolean;
    Function IsMessage: Boolean;
    Function IsReintroduced : Boolean;
    Function IsStatic : Boolean;
    Function IsForward: Boolean;
    Property Modifiers : TProcedureModifiers Read FModifiers Write FModifiers;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Property CallingConvention : TCallingConvention Read GetCallingConvention Write SetCallingConvention;
=======
    Property CallingConvention : TCallingConvention Read FCallingConvention Write FCallingConvention;
>>>>>>> graemeg/cpstrnew
=======
    Property CallingConvention : TCallingConvention Read FCallingConvention Write FCallingConvention;
>>>>>>> graemeg/cpstrnew
=======
    Property CallingConvention : TCallingConvention Read FCallingConvention Write FCallingConvention;
>>>>>>> graemeg/cpstrnew
=======
    Property CallingConvention : TCallingConvention Read FCallingConvention Write FCallingConvention;
>>>>>>> origin/cpstrnew
    Property MessageName : String Read FMessageName Write FMessageName;
    property MessageType : TProcedureMessageType Read FMessageType Write FMessageType;
  end;

  TPasFunction = class(TPasProcedure)
  private
    function GetFT: TPasFunctionType;
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetDeclaration (full : boolean) : string; override;
    Property FuncType : TPasFunctionType Read GetFT;
  end;

  { TPasOperator }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TOperatorType = (otUnknown,otImplicit,otExplicit,otMul,otPlus, otMinus, otDivision,otLessThan, otEqual,
                   otGreaterThan, otAssign,otNotEqual,otLessEqualThan,otGreaterEqualThan,otPower,
                   otSymmetricalDifference, otInc, otDec, otMod, otNegative, otPositive, otBitWiseOr, otDiv,
                   otLeftShift, otLogicalOr, otBitwiseAnd, otbitwiseXor,otLogicalAnd,otLogicalNot,otLogicalXor,
                   otRightShift);
  TOperatorTypes = set of TOperatorType;

  TPasOperator = class(TPasFunction)
  private
    FOperatorType: TOperatorType;
    FTokenBased: Boolean;
    function NameSuffix: String;
  public
    Class Function OperatorTypeToToken(T : TOperatorType) : String;
    Class Function OperatorTypeToOperatorName(T: TOperatorType) : String;
    Class Function TokenToOperatorType(S : String) : TOperatorType;
    Class Function NameToOperatorType(S : String) : TOperatorType;
    Procedure CorrectName;
    // For backwards compatibility the old name can still be used to search on.
    function GetOperatorDeclaration(Full: Boolean): string;
    Function OldName(WithPath : Boolean) : String;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  TPasOperator = class(TPasProcedure)
  public
>>>>>>> graemeg/cpstrnew
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetDeclaration (full : boolean) : string; override;
    Property OperatorType : TOperatorType Read FOperatorType Write FOperatorType;
    // True if the declaration was using a token instead of a
    Property TokenBased : Boolean Read FTokenBased Write FTokenBased;
  end;

Type
  { TPasClassOperator }

  TPasClassOperator = class(TPasOperator)
    function TypeName: string; override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { TPasConstructor }

  TPasConstructor = class(TPasProcedure)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  { TPasClassConstructor }

  TPasClassConstructor  = class(TPasConstructor)
=======
  { TPasDestructor }

  TPasDestructor = class(TPasProcedure)
>>>>>>> graemeg/cpstrnew
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { TPasDestructor }

  TPasDestructor = class(TPasProcedure)
=======
  { TPasClassProcedure }

  TPasClassProcedure = class(TPasProcedure)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

  { TPasClassFunction }

  TPasClassFunction = class(TPasProcedure)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

  TPasImplBlock = class;

  { TProcedureBody - the var+type+const+begin, without the header, child of TPasProcedure }

  TProcedureBody = class(TPasDeclarations)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
  public
    Labels: TList;
    Body: TPasImplBlock;
  end;

  { TPasProcedureImpl - used by mkxmlrpc, not by pparser }

  TPasProcedureImpl = class(TPasElement)
>>>>>>> graemeg/cpstrnew
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  { TPasClassDestructor }

  TPasClassDestructor  = class(TPasDestructor)
=======
  { TPasConstructorImpl - used by mkxmlrpc, not by pparser }

  TPasConstructorImpl = class(TPasProcedureImpl)
>>>>>>> graemeg/cpstrnew
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { TPasClassProcedure }

  TPasClassProcedure = class(TPasProcedure)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

  { TPasClassFunction }

  TPasClassFunction = class(TPasProcedure)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

  TPasImplBlock = class;

  { TProcedureBody - the var+type+const+begin, without the header, child of TPasProcedure }

  TProcedureBody = class(TPasDeclarations)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
  public
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

    Labels: TFPList;
=======
    Labels: TList;
>>>>>>> graemeg/cpstrnew
=======
    Labels: TList;
>>>>>>> graemeg/cpstrnew
=======
    Labels: TList;
>>>>>>> origin/cpstrnew
    Body: TPasImplBlock;
  end;

  { TPasProcedureImpl - used by mkxmlrpc, not by pparser }

  TPasProcedureImpl = class(TPasElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function TypeName: string; virtual;
  public
    ProcType: TPasProcedureType;
    Locals: TFPList;
    Body: TPasImplBlock;
  end;

  { TPasConstructorImpl - used by mkxmlrpc, not by pparser }

  TPasConstructorImpl = class(TPasProcedureImpl)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  { TPasDestructorImpl - used by mkxmlrpc, not by pparser }

  TPasDestructorImpl = class(TPasProcedureImpl)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

  { TPasImplElement - implementation element }

  TPasImplElement = class(TPasElement)
  end;

  { TPasImplCommand }

  TPasImplCommand = class(TPasImplElement)
  public
    Command: string;
  end;

  { TPasImplCommands - used by mkxmlrpc, not used by pparser }

  TPasImplCommands = class(TPasImplElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
  public
    Commands: TStrings;
  end;

  { TPasLabels }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  TPasLabels = class(TPasImplElement)
=======

  TPasLabels = class(TPasImplElement)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  TPasLabels = class(TPasImplElement)
  public
    Labels  : TStrings;
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
  end;

  TPasImplBeginBlock = class;
  TPasImplRepeatUntil = class;
  TPasImplIfElse = class;
  TPasImplWhileDo = class;
  TPasImplWithDo = class;
  TPasImplCaseOf = class;
  TPasImplForLoop = class;
  TPasImplTry = class;
  TPasImplExceptOn = class;
  TPasImplRaise = class;
  TPasImplAssign = class;
  TPasImplSimple = class;
  TPasImplLabelMark = class;

  { TPasImplBlock }

  TPasImplBlock = class(TPasImplElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); virtual;
    function AddCommand(const ACommand: string): TPasImplCommand;
    function AddCommands: TPasImplCommands; // used by mkxmlrpc, not by pparser
    function AddBeginBlock: TPasImplBeginBlock;
    function AddRepeatUntil: TPasImplRepeatUntil;
    function AddIfElse(const ACondition: string): TPasImplIfElse;
    function AddWhileDo(const ACondition: string): TPasImplWhileDo;
    function AddWithDo(const Expression: string): TPasImplWithDo;
    function AddCaseOf(const Expression: string): TPasImplCaseOf;
    function AddForLoop(AVar: TPasVariable;
      const AStartValue, AEndValue: string): TPasImplForLoop;
    function AddForLoop(const AVarName, AStartValue, AEndValue: string;
      ADownTo: Boolean = false): TPasImplForLoop;
    function AddTry: TPasImplTry;
    function AddExceptOn(const VarName, TypeName: string): TPasImplExceptOn;
    function AddRaise: TPasImplRaise;
    function AddLabelMark(const Id: string): TPasImplLabelMark;
    function AddAssign(left, right: TPasExpr): TPasImplAssign;
    function AddSimple(exp: TPasExpr): TPasImplSimple;
    function CloseOnSemicolon: boolean; virtual;
  public
    Elements: TList;    // TPasImplElement objects
  end;

  { TPasImplStatement }

  TPasImplStatement = class(TPasImplBlock)
  public
    function CloseOnSemicolon: boolean; override;
  end;

  { TPasImplBeginBlock }

  TPasImplBeginBlock = class(TPasImplBlock)
  end;

  { TInitializationSection }

  TInitializationSection = class(TPasImplBlock)
  end;

  { TFinalizationSection }

  TFinalizationSection = class(TPasImplBlock)
  end;

  { TPasImplRepeatUntil }

  TPasImplRepeatUntil = class(TPasImplBlock)
  public
    Condition: string;
  end;

  { TPasImplIfElse }

  TPasImplIfElse = class(TPasImplBlock)
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  public
    Labels  : TStrings;
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  end;

  TPasImplBeginBlock = class;
  TPasImplRepeatUntil = class;
  TPasImplIfElse = class;
  TPasImplWhileDo = class;
  TPasImplWithDo = class;
  TPasImplCaseOf = class;
  TPasImplForLoop = class;
  TPasImplTry = class;
  TPasImplExceptOn = class;
  TPasImplRaise = class;
  TPasImplAssign = class;
  TPasImplSimple = class;
  TPasImplLabelMark = class;

  { TPasImplBlock }

  TPasImplBlock = class(TPasImplElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); virtual;
    function AddCommand(const ACommand: string): TPasImplCommand;
    function AddCommands: TPasImplCommands; // used by mkxmlrpc, not by pparser
    function AddBeginBlock: TPasImplBeginBlock;
    function AddRepeatUntil: TPasImplRepeatUntil;
    function AddIfElse(const ACondition: string): TPasImplIfElse;
    function AddWhileDo(const ACondition: string): TPasImplWhileDo;
    function AddWithDo(const Expression: string): TPasImplWithDo;
    function AddCaseOf(const Expression: string): TPasImplCaseOf;
    function AddForLoop(AVar: TPasVariable;
      const AStartValue, AEndValue: string): TPasImplForLoop;
    function AddForLoop(const AVarName, AStartValue, AEndValue: string;
      ADownTo: Boolean = false): TPasImplForLoop;
    function AddTry: TPasImplTry;
    function AddExceptOn(const VarName, TypeName: string): TPasImplExceptOn;
    function AddRaise: TPasImplRaise;
    function AddLabelMark(const Id: string): TPasImplLabelMark;
    function AddAssign(left, right: TPasExpr): TPasImplAssign;
    function AddSimple(exp: TPasExpr): TPasImplSimple;
    function CloseOnSemicolon: boolean; virtual;
=======
    procedure AddElement(Element: TPasImplElement); override;
    function CloseOnSemicolon: boolean; override;
  public
    Condition: string;
    IfBranch: TPasImplElement;
    ElseBranch: TPasImplElement; // can be nil
  end;

  { TPasImplWhileDo }
=======
    procedure AddElement(Element: TPasImplElement); override;
    function CloseOnSemicolon: boolean; override;
  public
    Condition: string;
    IfBranch: TPasImplElement;
    ElseBranch: TPasImplElement; // can be nil
=======
    procedure AddElement(Element: TPasImplElement); override;
    function CloseOnSemicolon: boolean; override;
  public
    Condition: string;
    IfBranch: TPasImplElement;
    ElseBranch: TPasImplElement; // can be nil
  end;

  { TPasImplWhileDo }

  TPasImplWhileDo = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    Condition: string;
    Body: TPasImplElement;
  end;

  { TPasImplWithDo }

  TPasImplWithDo = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expression: string);
  public
    Expressions: TStrings;
    Body: TPasImplElement;
  end;

  TPasImplCaseStatement = class;
  TPasImplCaseElse = class;

  { TPasImplCaseOf }

  TPasImplCaseOf = class(TPasImplBlock)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    function AddCase(const Expression: string): TPasImplCaseStatement;
    function AddElse: TPasImplCaseElse;
  public
    Expression: string;
    ElseBranch: TPasImplCaseElse;
  end;

  { TPasImplCaseStatement }

  TPasImplCaseStatement = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expr: string);
  public
    Expressions: TStrings;
    Body: TPasImplElement;
  end;

  { TPasImplCaseElse }

  TPasImplCaseElse = class(TPasImplBlock)
  end;

  { TPasImplForLoop }

  TPasImplForLoop = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    Variable: TPasVariable;
    VariableName, StartValue, EndValue: string;
    Down: boolean; // downto
    Body: TPasImplElement;
  end;

  { TPasImplAssign }

  TPasImplAssign = class (TPasImplStatement)
  public
    left  : TPasExpr;
    right : TPasExpr;
  end;

  { TPasImplSimple }

  TPasImplSimple = class (TPasImplStatement)
  public
    expr  : TPasExpr;
  end;

  TPasImplTryHandler = class;
  TPasImplTryFinally = class;
  TPasImplTryExcept = class;
  TPasImplTryExceptElse = class;

  { TPasImplTry }

  TPasImplTry = class(TPasImplBlock)
  public
    destructor Destroy; override;
    function AddFinally: TPasImplTryFinally;
    function AddExcept: TPasImplTryExcept;
    function AddExceptElse: TPasImplTryExceptElse;
  public
    FinallyExcept: TPasImplTryHandler;
    ElseBranch: TPasImplTryExceptElse;
  end;

  TPasImplTryHandler = class(TPasImplBlock)
  end;

  { TPasImplTryFinally }

  TPasImplTryFinally = class(TPasImplTryHandler)
  end;

  { TPasImplTryExcept }

  TPasImplTryExcept = class(TPasImplTryHandler)
  end;

  { TPasImplTryExceptElse }

  TPasImplTryExceptElse = class(TPasImplTryHandler)
  end;

  { TPasImplExceptOn }

  TPasImplExceptOn = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    VariableName, TypeName: string;
    Body: TPasImplElement;
  end;

  { TPasImplRaise }

  TPasImplRaise = class(TPasImplStatement)
  end;

  { TPassTreeVisitor }

  TPassTreeVisitor = class
    procedure Visit(obj: TPasElement); virtual;
  end;

  TPasImplLabelMark = class(TPasImplElement)
  public
    LabelId:  AnsiString;
>>>>>>> origin/cpstrnew
  end;

  { TPasImplWhileDo }

<<<<<<< HEAD
  TPasImplWhileDo = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    Condition: string;
    Body: TPasImplElement;
  end;

  { TPasImplWithDo }

  TPasImplWithDo = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expression: string);
  public
    Expressions: TStrings;
    Body: TPasImplElement;
  end;

  TPasImplCaseStatement = class;
  TPasImplCaseElse = class;
=======
  VisibilityNames: array[TPasMemberVisibility] of string = (
    'default', 'private', 'protected', 'public', 'published', 'automated','strict private', 'strict protected');

  ObjKindNames: array[TPasObjKind] of string = (
    'object', 'class', 'interface');
  
  OpcodeStrings : Array[TExprOpCode] of string = 
       ('','+','-','*','/','div','mod','**',
        'shr','shl',
        'not','and','or','xor',
        '=','<>',
        '<','>','<=','>=',
        'in','is','as','><',
        '@','^',
        '.');

  cPasMemberHint : array[TPasMemberHint] of string =
      ( 'deprecated', 'library', 'platform', 'experimental', 'unimplemented' );
>>>>>>> origin/cpstrnew

  { TPasImplCaseOf }

  TPasImplCaseOf = class(TPasImplBlock)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    function AddCase(const Expression: string): TPasImplCaseStatement;
    function AddElse: TPasImplCaseElse;
  public
    Expression: string;
    ElseBranch: TPasImplCaseElse;
  end;

  { TPasImplCaseStatement }

  TPasImplCaseStatement = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expr: string);
  public
    Expressions: TStrings;
    Body: TPasImplElement;
  end;

  { TPasImplCaseElse }

  TPasImplCaseElse = class(TPasImplBlock)
  end;

  { TPasImplForLoop }

  TPasImplForLoop = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    Variable: TPasVariable;
    VariableName, StartValue, EndValue: string;
    Down: boolean; // downto
    Body: TPasImplElement;
  end;

  { TPasImplAssign }

  TPasImplAssign = class (TPasImplStatement)
  public
    left  : TPasExpr;
    right : TPasExpr;
  end;

  { TPasImplSimple }

  TPasImplSimple = class (TPasImplStatement)
  public
    expr  : TPasExpr;
  end;

  TPasImplTryHandler = class;
  TPasImplTryFinally = class;
  TPasImplTryExcept = class;
  TPasImplTryExceptElse = class;

  { TPasImplTry }

  TPasImplTry = class(TPasImplBlock)
  public
    destructor Destroy; override;
    function AddFinally: TPasImplTryFinally;
    function AddExcept: TPasImplTryExcept;
    function AddExceptElse: TPasImplTryExceptElse;
  public
    FinallyExcept: TPasImplTryHandler;
    ElseBranch: TPasImplTryExceptElse;
  end;

  TPasImplTryHandler = class(TPasImplBlock)
  end;

  { TPasImplTryFinally }

  TPasImplTryFinally = class(TPasImplTryHandler)
  end;

  { TPasImplTryExcept }

  TPasImplTryExcept = class(TPasImplTryHandler)
  end;

  { TPasImplTryExceptElse }

  TPasImplTryExceptElse = class(TPasImplTryHandler)
  end;

  { TPasImplExceptOn }

  TPasImplExceptOn = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    VariableName, TypeName: string;
    Body: TPasImplElement;
  end;

  { TPasImplRaise }

  TPasImplRaise = class(TPasImplStatement)
  end;

  { TPassTreeVisitor }

  TPassTreeVisitor = class
    procedure Visit(obj: TPasElement); virtual;
  end;

  TPasImplLabelMark = class(TPasImplElement)
  public
    LabelId:  AnsiString;
  end;
>>>>>>> graemeg/cpstrnew

  TPasImplWhileDo = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    Condition: string;
    Body: TPasImplElement;
  end;

<<<<<<< HEAD
  { TPasImplWithDo }

  TPasImplWithDo = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expression: string);
>>>>>>> graemeg/cpstrnew
  public
    Expressions: TStrings;
    Body: TPasImplElement;
  end;

  TPasImplCaseStatement = class;
  TPasImplCaseElse = class;
=======
  VisibilityNames: array[TPasMemberVisibility] of string = (
    'default', 'private', 'protected', 'public', 'published', 'automated','strict private', 'strict protected');

  ObjKindNames: array[TPasObjKind] of string = (
    'object', 'class', 'interface');
  
  OpcodeStrings : Array[TExprOpCode] of string = 
       ('','+','-','*','/','div','mod','**',
        'shr','shl',
        'not','and','or','xor',
        '=','<>',
        '<','>','<=','>=',
        'in','is','as','><',
        '@','^',
        '.');

  cPasMemberHint : array[TPasMemberHint] of string =
      ( 'deprecated', 'library', 'platform', 'experimental', 'unimplemented' );
>>>>>>> graemeg/cpstrnew

  { TPasImplCaseOf }

  TPasImplCaseOf = class(TPasImplBlock)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    function AddCase(const Expression: string): TPasImplCaseStatement;
    function AddElse: TPasImplCaseElse;
  public
    Expression: string;
    ElseBranch: TPasImplCaseElse;
  end;

  { TPasImplCaseStatement }

  TPasImplCaseStatement = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expr: string);
  public
    Expressions: TStrings;
    Body: TPasImplElement;
  end;

<<<<<<< HEAD
  { TPasImplCaseElse }
=======
function TPasElement.ElementTypeName: string; begin Result := SPasTreeElement end;
function TPasDeclarations.ElementTypeName: string; begin Result := SPasTreeSection end;
function TPasModule.ElementTypeName: string; begin Result := SPasTreeModule end;
function TPasPackage.ElementTypeName: string; begin Result := SPasTreePackage end;
function TPasResString.ElementTypeName: string; begin Result := SPasTreeResString end;
function TPasType.ElementTypeName: string; begin Result := SPasTreeType end;
function TPasPointerType.ElementTypeName: string; begin Result := SPasTreePointerType end;
function TPasAliasType.ElementTypeName: string; begin Result := SPasTreeAliasType end;
function TPasTypeAliasType.ElementTypeName: string; begin Result := SPasTreeTypeAliasType end;
function TPasClassOfType.ElementTypeName: string; begin Result := SPasTreeClassOfType end;
function TPasRangeType.ElementTypeName: string; begin Result := SPasTreeRangeType end;
function TPasArrayType.ElementTypeName: string; begin Result := SPasTreeArrayType end;
function TPasFileType.ElementTypeName: string; begin Result := SPasTreeFileType end;
function TPasEnumValue.ElementTypeName: string; begin Result := SPasTreeEnumValue end;
function TPasEnumType.ElementTypeName: string; begin Result := SPasTreeEnumType end;
function TPasSetType.ElementTypeName: string; begin Result := SPasTreeSetType end;
function TPasRecordType.ElementTypeName: string; begin Result := SPasTreeRecordType end;
function TPasArgument.ElementTypeName: string; begin Result := SPasTreeArgument end;
function TPasProcedureType.ElementTypeName: string; begin Result := SPasTreeProcedureType end;
function TPasResultElement.ElementTypeName: string; begin Result := SPasTreeResultElement end;
function TPasFunctionType.ElementTypeName: string; begin Result := SPasTreeFunctionType end;
function TPasUnresolvedTypeRef.ElementTypeName: string; begin Result := SPasTreeUnresolvedTypeRef end;
function TPasVariable.ElementTypeName: string; begin Result := SPasTreeVariable end;
function TPasConst.ElementTypeName: string; begin Result := SPasTreeConst end;
function TPasProperty.ElementTypeName: string; begin Result := SPasTreeProperty end;
function TPasOverloadedProc.ElementTypeName: string; begin Result := SPasTreeOverloadedProcedure end;
function TPasProcedure.ElementTypeName: string; begin Result := SPasTreeProcedure end;
function TPasFunction.ElementTypeName: string; begin Result := SPasTreeFunction end;
function TPasClassProcedure.ElementTypeName: string; begin Result := SPasTreeClassProcedure; end;
function TPasClassFunction.ElementTypeName: string; begin Result := SPasTreeClassFunction; end;
function TPasOperator.ElementTypeName: string; begin Result := SPasTreeFunction end;
function TPasConstructor.ElementTypeName: string; begin Result := SPasTreeConstructor end;
function TPasDestructor.ElementTypeName: string; begin Result := SPasTreeDestructor end;
function TPasProcedureImpl.ElementTypeName: string; begin Result := SPasTreeProcedureImpl end;
function TPasConstructorImpl.ElementTypeName: string; begin Result := SPasTreeConstructorImpl end;
function TPasDestructorImpl.ElementTypeName: string; begin Result := SPasTreeDestructorImpl end;
>>>>>>> graemeg/cpstrnew

  TPasImplCaseElse = class(TPasImplBlock)
  end;

  { TPasImplForLoop }

<<<<<<< HEAD
<<<<<<< HEAD
  TPasImplForLoop = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    Variable: TPasVariable;
    VariableName, StartValue, EndValue: string;
    Down: boolean; // downto
    Body: TPasImplElement;
  end;
=======

{ All other stuff: }

procedure TPasElement.ProcessHints(const ASemiColonPrefix: boolean; var AResult: string);
var
  h: TPasMemberHint;
begin
  if Hints <> [] then
  begin
    if ASemiColonPrefix then
      AResult := AResult + ';';
    for h := Low(TPasMemberHint) to High(TPasMemberHint) do
    begin
      if h in Hints then
        AResult := AResult + ' ' + cPasMemberHint[h] + ';'
    end;
  end;
end;

constructor TPasElement.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create;
  FName := AName;
  FParent := AParent;
end;
>>>>>>> origin/cpstrnew

  { TPasImplAssign }
=======

{ All other stuff: }

procedure TPasElement.ProcessHints(const ASemiColonPrefix: boolean; var AResult: string);
var
  h: TPasMemberHint;
begin
<<<<<<< HEAD
  if Hints <> [] then
=======
  Result := Name;
  p := Parent;
  while Assigned(p) and not p.InheritsFrom(TPasDeclarations) do
>>>>>>> origin/cpstrnew
  begin
    if ASemiColonPrefix then
      AResult := AResult + ';';
    for h := Low(TPasMemberHint) to High(TPasMemberHint) do
    begin
      if h in Hints then
        AResult := AResult + ' ' + cPasMemberHint[h] + ';'
    end;
  end;
end;
>>>>>>> graemeg/cpstrnew

  TPasImplAssign = class (TPasImplStatement)
  public
    left  : TPasExpr;
    right : TPasExpr;
  end;

  { TPasImplSimple }

  TPasImplSimple = class (TPasImplStatement)
  public
    expr  : TPasExpr;
  end;

<<<<<<< HEAD
  TPasImplTryHandler = class;
  TPasImplTryFinally = class;
  TPasImplTryExcept = class;
  TPasImplTryExceptElse = class;

  { TPasImplTry }

  TPasImplTry = class(TPasImplBlock)
  public
    destructor Destroy; override;
    function AddFinally: TPasImplTryFinally;
    function AddExcept: TPasImplTryExcept;
    function AddExceptElse: TPasImplTryExceptElse;
  public
    FinallyExcept: TPasImplTryHandler;
    ElseBranch: TPasImplTryExceptElse;
=======
function TPasElement.FullName: string;
var
  p: TPasElement;
begin
  Result := Name;
  p := Parent;
  while Assigned(p) and not p.InheritsFrom(TPasDeclarations) do
  begin
    if (p.ClassType <> TPasOverloadedProc) and (Length(p.Name) > 0) then
      if Length(Result) > 0 then
        Result := p.Name + '.' + Result
      else
        Result := p.Name;
    p := p.Parent;
>>>>>>> graemeg/cpstrnew
  end;

  TPasImplTryHandler = class(TPasImplBlock)
  end;

  { TPasImplTryFinally }

  TPasImplTryFinally = class(TPasImplTryHandler)
  end;

<<<<<<< HEAD
  { TPasImplTryExcept }

  TPasImplTryExcept = class(TPasImplTryHandler)
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  { TPasImplStatement }
=======
  { TPasImplTryExceptElse }

  TPasImplTryExceptElse = class(TPasImplTryHandler)
  end;

  { TPasImplExceptOn }

  TPasImplExceptOn = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    VariableName, TypeName: string;
    Body: TPasImplElement;
  end;

  { TPasImplRaise }
=======
procedure TPasElement.Accept(Visitor: TPassTreeVisitor);
begin
  Visitor.Visit(Self);
end;

constructor TPasDeclarations.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Declarations := TList.Create;
  ResStrings := TList.Create;
  Types := TList.Create;
  Consts := TList.Create;
  Classes := TList.Create;
  Functions := TList.Create;
  Variables := TList.Create;
  Properties := TList.Create;
end;

destructor TPasDeclarations.Destroy;
var
  i: Integer;
begin
  Variables.Free;
  Functions.Free;
  Classes.Free;
  Consts.Free;
  Types.Free;
  ResStrings.Free;
  Properties.Free;
  for i := 0 to Declarations.Count - 1 do
    TPasElement(Declarations[i]).Release;
  Declarations.Free;

  inherited Destroy;
end;

destructor TPasModule.Destroy;
begin
  if Assigned(InterfaceSection) then
    InterfaceSection.Release;
  if Assigned(ImplementationSection) then
    ImplementationSection.Release;
  inherited Destroy;
end;
>>>>>>> origin/cpstrnew

  TPasImplRaise = class(TPasImplStatement)
  end;

  { TPassTreeVisitor }

  TPassTreeVisitor = class
    procedure Visit(obj: TPasElement); virtual;
  end;

  TPasImplLabelMark = class(TPasImplElement)
  public
    LabelId:  AnsiString;
  end;
>>>>>>> graemeg/cpstrnew

  TPasImplStatement = class(TPasImplBlock)
  public
    function CloseOnSemicolon: boolean; override;
  end;

<<<<<<< HEAD
  { TPasImplBeginBlock }

  TPasImplBeginBlock = class(TPasImplBlock)
  end;

  { TInitializationSection }
=======
  VisibilityNames: array[TPasMemberVisibility] of string = (
    'default', 'private', 'protected', 'public', 'published', 'automated','strict private', 'strict protected');

  ObjKindNames: array[TPasObjKind] of string = (
    'object', 'class', 'interface');
  
  OpcodeStrings : Array[TExprOpCode] of string = 
       ('','+','-','*','/','div','mod','**',
        'shr','shl',
        'not','and','or','xor',
        '=','<>',
        '<','>','<=','>=',
        'in','is','as','><',
        '@','^',
        '.');

  cPasMemberHint : array[TPasMemberHint] of string =
      ( 'deprecated', 'library', 'platform', 'experimental', 'unimplemented' );
>>>>>>> graemeg/cpstrnew

  TInitializationSection = class(TPasImplBlock)
  end;

  { TFinalizationSection }

  TFinalizationSection = class(TPasImplBlock)
  end;

  { TPasImplRepeatUntil }

  TPasImplRepeatUntil = class(TPasImplBlock)
  public
    Condition: string;
  end;

  { TPasImplIfElse }

  TPasImplIfElse = class(TPasImplBlock)
>>>>>>> graemeg/cpstrnew
  public
    Labels  : TStrings;
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
<<<<<<< HEAD
  end;

  TPasImplBeginBlock = class;
  TPasImplRepeatUntil = class;
  TPasImplIfElse = class;
  TPasImplWhileDo = class;
  TPasImplWithDo = class;
  TPasImplCaseOf = class;
  TPasImplForLoop = class;
  TPasImplTry = class;
  TPasImplExceptOn = class;
  TPasImplRaise = class;
  TPasImplAssign = class;
  TPasImplSimple = class;
  TPasImplLabelMark = class;

  { TPasImplBlock }

  TPasImplBlock = class(TPasImplElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); virtual;
    function AddCommand(const ACommand: string): TPasImplCommand;
    function AddCommands: TPasImplCommands; // used by mkxmlrpc, not by pparser
    function AddBeginBlock: TPasImplBeginBlock;
    function AddRepeatUntil: TPasImplRepeatUntil;
    function AddIfElse(const ACondition: TPasExpr): TPasImplIfElse;
    function AddWhileDo(const ACondition: TPasExpr): TPasImplWhileDo;
    function AddWithDo(const Expression: TPasExpr): TPasImplWithDo;
    function AddCaseOf(const Expression: TPasExpr): TPasImplCaseOf;
    function AddForLoop(AVar: TPasVariable;
      const AStartValue, AEndValue: TPasExpr): TPasImplForLoop;
    function AddForLoop(const AVarName : String; AStartValue, AEndValue: TPasExpr;
      ADownTo: Boolean = false): TPasImplForLoop;
    function AddTry: TPasImplTry;
    function AddExceptOn(const VarName, TypeName: TPasExpr): TPasImplExceptOn;
    function AddRaise: TPasImplRaise;
    function AddLabelMark(const Id: string): TPasImplLabelMark;
    function AddAssign(left, right: TPasExpr): TPasImplAssign;
    function AddSimple(exp: TPasExpr): TPasImplSimple;
    function CloseOnSemicolon: boolean; virtual;
  public
    Elements: TFPList;    // TPasImplElement objects
  end;

  { TPasImplStatement }
=======
    procedure AddElement(Element: TPasImplElement); override;
    function CloseOnSemicolon: boolean; override;
  public
    Condition: string;
    IfBranch: TPasImplElement;
    ElseBranch: TPasImplElement; // can be nil
  end;

  { TPasImplWhileDo }

  TPasImplWhileDo = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    Condition: string;
    Body: TPasImplElement;
  end;

  { TPasImplWithDo }

  TPasImplWithDo = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expression: string);
  public
    Expressions: TStrings;
    Body: TPasImplElement;
  end;

  TPasImplCaseStatement = class;
  TPasImplCaseElse = class;

  { TPasImplCaseOf }

  TPasImplCaseOf = class(TPasImplBlock)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    function AddCase(const Expression: string): TPasImplCaseStatement;
    function AddElse: TPasImplCaseElse;
  public
    Expression: string;
    ElseBranch: TPasImplCaseElse;
  end;

  { TPasImplCaseStatement }

  TPasImplCaseStatement = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expr: string);
  public
    Expressions: TStrings;
    Body: TPasImplElement;
  end;

  { TPasImplCaseElse }

  TPasImplCaseElse = class(TPasImplBlock)
  end;

  { TPasImplForLoop }

  TPasImplForLoop = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    Variable: TPasVariable;
    VariableName, StartValue, EndValue: string;
    Down: boolean; // downto
    Body: TPasImplElement;
  end;

  { TPasImplAssign }

  TPasImplAssign = class (TPasImplStatement)
  public
    left  : TPasExpr;
    right : TPasExpr;
  end;

  { TPasImplSimple }

  TPasImplSimple = class (TPasImplStatement)
  public
    expr  : TPasExpr;
  end;

  TPasImplTryHandler = class;
  TPasImplTryFinally = class;
  TPasImplTryExcept = class;
  TPasImplTryExceptElse = class;

  { TPasImplTry }

  TPasImplTry = class(TPasImplBlock)
  public
    destructor Destroy; override;
    function AddFinally: TPasImplTryFinally;
    function AddExcept: TPasImplTryExcept;
    function AddExceptElse: TPasImplTryExceptElse;
  public
    FinallyExcept: TPasImplTryHandler;
    ElseBranch: TPasImplTryExceptElse;
  end;

  TPasImplTryHandler = class(TPasImplBlock)
  end;

  { TPasImplTryFinally }

  TPasImplTryFinally = class(TPasImplTryHandler)
  end;

  { TPasImplTryExcept }

  TPasImplTryExcept = class(TPasImplTryHandler)
  end;

  { TPasImplTryExceptElse }

  TPasImplTryExceptElse = class(TPasImplTryHandler)
  end;

  { TPasImplExceptOn }

  TPasImplExceptOn = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    VariableName, TypeName: string;
    Body: TPasImplElement;
  end;

  { TPasImplRaise }

  TPasImplRaise = class(TPasImplStatement)
  end;

  { TPassTreeVisitor }

  TPassTreeVisitor = class
    procedure Visit(obj: TPasElement); virtual;
  end;

  TPasImplLabelMark = class(TPasImplElement)
  public
    LabelId:  AnsiString;
  end;
>>>>>>> graemeg/cpstrnew

  TPasImplStatement = class(TPasImplBlock)
  public
    function CloseOnSemicolon: boolean; override;
  end;

<<<<<<< HEAD
  { TPasImplBeginBlock }

  TPasImplBeginBlock = class(TPasImplBlock)
  end;

  { TInitializationSection }
=======
  VisibilityNames: array[TPasMemberVisibility] of string = (
    'default', 'private', 'protected', 'public', 'published', 'automated','strict private', 'strict protected');

  ObjKindNames: array[TPasObjKind] of string = (
    'object', 'class', 'interface');
  
  OpcodeStrings : Array[TExprOpCode] of string = 
       ('','+','-','*','/','div','mod','**',
        'shr','shl',
        'not','and','or','xor',
        '=','<>',
        '<','>','<=','>=',
        'in','is','as','><',
        '@','^',
        '.');

  cPasMemberHint : array[TPasMemberHint] of string =
      ( 'deprecated', 'library', 'platform', 'experimental', 'unimplemented' );
>>>>>>> graemeg/cpstrnew

  TInitializationSection = class(TPasImplBlock)
  end;

  { TFinalizationSection }

  TFinalizationSection = class(TPasImplBlock)
  end;

  { TPasImplAsmStatement }

<<<<<<< HEAD
  TPasImplAsmStatement = class (TPasImplStatement)
  private
    FTokens: TStrings;
  Public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    Property Tokens : TStrings Read FTokens;
  end;
=======
function TPasElement.ElementTypeName: string; begin Result := SPasTreeElement end;
function TPasDeclarations.ElementTypeName: string; begin Result := SPasTreeSection end;
function TPasModule.ElementTypeName: string; begin Result := SPasTreeModule end;
function TPasPackage.ElementTypeName: string; begin Result := SPasTreePackage end;
function TPasResString.ElementTypeName: string; begin Result := SPasTreeResString end;
function TPasType.ElementTypeName: string; begin Result := SPasTreeType end;
function TPasPointerType.ElementTypeName: string; begin Result := SPasTreePointerType end;
function TPasAliasType.ElementTypeName: string; begin Result := SPasTreeAliasType end;
function TPasTypeAliasType.ElementTypeName: string; begin Result := SPasTreeTypeAliasType end;
function TPasClassOfType.ElementTypeName: string; begin Result := SPasTreeClassOfType end;
function TPasRangeType.ElementTypeName: string; begin Result := SPasTreeRangeType end;
function TPasArrayType.ElementTypeName: string; begin Result := SPasTreeArrayType end;
function TPasFileType.ElementTypeName: string; begin Result := SPasTreeFileType end;
function TPasEnumValue.ElementTypeName: string; begin Result := SPasTreeEnumValue end;
function TPasEnumType.ElementTypeName: string; begin Result := SPasTreeEnumType end;
function TPasSetType.ElementTypeName: string; begin Result := SPasTreeSetType end;
function TPasRecordType.ElementTypeName: string; begin Result := SPasTreeRecordType end;
function TPasArgument.ElementTypeName: string; begin Result := SPasTreeArgument end;
function TPasProcedureType.ElementTypeName: string; begin Result := SPasTreeProcedureType end;
function TPasResultElement.ElementTypeName: string; begin Result := SPasTreeResultElement end;
function TPasFunctionType.ElementTypeName: string; begin Result := SPasTreeFunctionType end;
function TPasUnresolvedTypeRef.ElementTypeName: string; begin Result := SPasTreeUnresolvedTypeRef end;
function TPasVariable.ElementTypeName: string; begin Result := SPasTreeVariable end;
function TPasConst.ElementTypeName: string; begin Result := SPasTreeConst end;
function TPasProperty.ElementTypeName: string; begin Result := SPasTreeProperty end;
function TPasOverloadedProc.ElementTypeName: string; begin Result := SPasTreeOverloadedProcedure end;
function TPasProcedure.ElementTypeName: string; begin Result := SPasTreeProcedure end;
function TPasFunction.ElementTypeName: string; begin Result := SPasTreeFunction end;
function TPasClassProcedure.ElementTypeName: string; begin Result := SPasTreeClassProcedure; end;
function TPasClassFunction.ElementTypeName: string; begin Result := SPasTreeClassFunction; end;
function TPasOperator.ElementTypeName: string; begin Result := SPasTreeFunction end;
function TPasConstructor.ElementTypeName: string; begin Result := SPasTreeConstructor end;
function TPasDestructor.ElementTypeName: string; begin Result := SPasTreeDestructor end;
function TPasProcedureImpl.ElementTypeName: string; begin Result := SPasTreeProcedureImpl end;
function TPasConstructorImpl.ElementTypeName: string; begin Result := SPasTreeConstructorImpl end;
function TPasDestructorImpl.ElementTypeName: string; begin Result := SPasTreeDestructorImpl end;
>>>>>>> graemeg/cpstrnew

  { TPasImplRepeatUntil }

  TPasImplRepeatUntil = class(TPasImplBlock)
  public
    ConditionExpr : TPasExpr;
    destructor Destroy; override;
    Function Condition: string;
  end;

  { TPasImplIfElse }

<<<<<<< HEAD
<<<<<<< HEAD
  TPasImplIfElse = class(TPasImplBlock)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    function CloseOnSemicolon: boolean; override;
  public
    ConditionExpr : TPasExpr;
    IfBranch: TPasImplElement;
    ElseBranch: TPasImplElement; // can be nil
    Function Condition: string;
  end;

  { TPasImplWhileDo }
=======
=======

{ All other stuff: }

procedure TPasElement.ProcessHints(const ASemiColonPrefix: boolean; var AResult: string);
var
  h: TPasMemberHint;
begin
  if Hints <> [] then
  begin
    if ASemiColonPrefix then
      AResult := AResult + ';';
    for h := Low(TPasMemberHint) to High(TPasMemberHint) do
    begin
      if h in Hints then
        AResult := AResult + ' ' + cPasMemberHint[h] + ';'
    end;
  end;
end;
>>>>>>> graemeg/cpstrnew

{ All other stuff: }

procedure TPasElement.ProcessHints(const ASemiColonPrefix: boolean; var AResult: string);
var
  h: TPasMemberHint;
begin
  if Hints <> [] then
  begin
    if ASemiColonPrefix then
      AResult := AResult + ';';
    for h := Low(TPasMemberHint) to High(TPasMemberHint) do
    begin
      if h in Hints then
        AResult := AResult + ' ' + cPasMemberHint[h] + ';'
    end;
  end;
end;
>>>>>>> graemeg/cpstrnew

  TPasImplWhileDo = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    ConditionExpr : TPasExpr;
    Body: TPasImplElement;
    function Condition: string;
  end;

  { TPasImplWithDo }

  TPasImplWithDo = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expression: TPasExpr);
  public
    Expressions: TFPList;
    Body: TPasImplElement;
  end;

<<<<<<< HEAD
  TPasImplCaseStatement = class;
  TPasImplCaseElse = class;

  { TPasImplCaseOf }

  TPasImplCaseOf = class(TPasImplBlock)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    function AddCase(const Expression: TPasExpr): TPasImplCaseStatement;
    function AddElse: TPasImplCaseElse;
  public
    CaseExpr : TPasExpr;
    ElseBranch: TPasImplCaseElse;
    function Expression: string;
=======
function TPasElement.FullName: string;
var
  p: TPasElement;
begin
  Result := Name;
  p := Parent;
  while Assigned(p) and not p.InheritsFrom(TPasDeclarations) do
  begin
    if (p.ClassType <> TPasOverloadedProc) and (Length(p.Name) > 0) then
      if Length(Result) > 0 then
        Result := p.Name + '.' + Result
      else
        Result := p.Name;
    p := p.Parent;
>>>>>>> graemeg/cpstrnew
  end;

  { TPasImplCaseStatement }

  TPasImplCaseStatement = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expr: TPasExpr);
  public
    Expressions: TFPList;
    Body: TPasImplElement;
  end;

  { TPasImplCaseElse }

  TPasImplCaseElse = class(TPasImplBlock)
  end;

  { TPasImplForLoop }
  TLoopType = (ltNormal,ltDown,ltIn);
  TPasImplForLoop = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    Variable: TPasVariable;
    StartExpr : TPasExpr;
    EndExpr : TPasExpr;
    VariableName : String;
    LoopType : TLoopType;
    Body: TPasImplElement;
    Function Down: boolean; // downto, backward compatibility
    Function StartValue : String;
    Function EndValue: string;
  end;

  { TPasImplAssign }
  TAssignKind = (akDefault,akAdd,akMinus,akMul,akDivision);
  TPasImplAssign = class (TPasImplStatement)
  public
    left  : TPasExpr;
    right : TPasExpr;
    Kind : TAssignKind;
    Destructor Destroy; override;
  end;

<<<<<<< HEAD
  { TPasImplSimple }

  TPasImplSimple = class (TPasImplStatement)
  public
    expr  : TPasExpr;
    Destructor Destroy; override;
  end;

  TPasImplTryHandler = class;
  TPasImplTryFinally = class;
  TPasImplTryExcept = class;
  TPasImplTryExceptElse = class;

  { TPasImplTry }

  TPasImplTry = class(TPasImplBlock)
  public
    destructor Destroy; override;
    function AddFinally: TPasImplTryFinally;
    function AddExcept: TPasImplTryExcept;
    function AddExceptElse: TPasImplTryExceptElse;
  public
    FinallyExcept: TPasImplTryHandler;
    ElseBranch: TPasImplTryExceptElse;
  end;

  TPasImplTryHandler = class(TPasImplBlock)
  end;

  { TPasImplTryFinally }

  TPasImplTryFinally = class(TPasImplTryHandler)
  end;

  { TPasImplTryExcept }

  TPasImplTryExcept = class(TPasImplTryHandler)
  end;

  { TPasImplTryExceptElse }

  TPasImplTryExceptElse = class(TPasImplTryHandler)
  end;

  { TPasImplExceptOn }

  TPasImplExceptOn = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
  public
    VarExpr,TypeExpr : TPasExpr;
    Body: TPasImplElement;
    Function VariableName : String;
    Function TypeName: string;
  end;

  { TPasImplRaise }

  TPasImplRaise = class(TPasImplStatement)
  public
    destructor Destroy; override;
  Public
    ExceptObject,
    ExceptAddr : TPasExpr;
  end;

  { TPassTreeVisitor }

  TPassTreeVisitor = class
    procedure Visit(obj: TPasElement); virtual;
  end;

  TPasImplLabelMark = class(TPasImplElement)
  public
    LabelId:  AnsiString;
  end;

const
  AccessNames: array[TArgumentAccess] of string[9] = ('', 'const ', 'var ', 'out ','constref ');
  AllVisibilities: TPasMemberVisibilities =
     [visDefault, visPrivate, visProtected, visPublic,
      visPublished, visAutomated];

  VisibilityNames: array[TPasMemberVisibility] of string = (
    'default', 'private', 'protected', 'public', 'published', 'automated','strict private', 'strict protected');

  ObjKindNames: array[TPasObjKind] of string = (
    'object', 'class', 'interface','class','class','class helper','record helper','type helper');
  
  OpcodeStrings : Array[TExprOpCode] of string = 
       ('','+','-','*','/','div','mod','**',
        'shr','shl',
        'not','and','or','xor',
        '=','<>',
        '<','>','<=','>=',
        'in','is','as','><',
        '@','^',
        '.');


  UnaryOperators = [otImplicit,otExplicit,otAssign,otNegative,otPositive];

  OperatorTokens : Array[TOperatorType] of string
       =  ('','','','*','+','-','/','<','=',
           '>',':=','<>','<=','>=','**',
           '><','Inc','Dec','mod','-','+','Or','div',
           'shl','or','and','xor','and','not','xor',
           'shr');
  OperatorNames : Array[TOperatorType] of string
       =  ('','implicit','explicit','multiply','add','subtract','divide','lessthan','equal',
           'greaterthan','assign','notequal','lessthanorequal','greaterthanorequal','power',
           'symmetricaldifference','inc','dec','modulus','negative','positive','bitwiseor','intdivide',
           'leftshift','logicalor','bitwiseand','bitwisexor','logicaland','logicalnot','logicalxor',
           'rightshift');

  cPasMemberHint : array[TPasMemberHint] of string =
      ( 'deprecated', 'library', 'platform', 'experimental', 'unimplemented' );
  cCallingConventions : array[TCallingConvention] of string =
      ( '', 'Register','Pascal','CDecl','StdCall','OldFPCCall','SafeCall','SysCall');

  ModifierNames : Array[TProcedureModifier] of string
                = ('virtual', 'dynamic','abstract', 'override',
                   'export', 'overload', 'message', 'reintroduce',
                   'static','inline','assembler','varargs', 'public',
                   'compilerproc','external','forward');

implementation

uses SysUtils;

{ TPasClassOperator }

function TPasClassOperator.TypeName: string;
begin
  Result:='class operator';
end;

<<<<<<< HEAD
{ TPasImplAsmStatement }

constructor TPasImplAsmStatement.Create(const AName: string;
  AParent: TPasElement);
=======
procedure TPasElement.Accept(Visitor: TPassTreeVisitor);
begin
  Visitor.Visit(Self);
end;

constructor TPasDeclarations.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Declarations := TList.Create;
  ResStrings := TList.Create;
  Types := TList.Create;
  Consts := TList.Create;
  Classes := TList.Create;
  Functions := TList.Create;
  Variables := TList.Create;
  Properties := TList.Create;
end;

destructor TPasDeclarations.Destroy;
var
  i: Integer;
begin
  Variables.Free;
  Functions.Free;
  Classes.Free;
  Consts.Free;
  Types.Free;
  ResStrings.Free;
  Properties.Free;
  for i := 0 to Declarations.Count - 1 do
    TPasElement(Declarations[i]).Release;
  Declarations.Free;

  inherited Destroy;
end;

destructor TPasModule.Destroy;
>>>>>>> graemeg/cpstrnew
begin
  inherited Create(AName, AParent);
  FTokens:=TStringList.Create;
end;

destructor TPasImplAsmStatement.Destroy;
=======
procedure TPasElement.Accept(Visitor: TPassTreeVisitor);
begin
  Visitor.Visit(Self);
end;

constructor TPasDeclarations.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Declarations := TList.Create;
  ResStrings := TList.Create;
  Types := TList.Create;
  Consts := TList.Create;
  Classes := TList.Create;
  Functions := TList.Create;
  Variables := TList.Create;
  Properties := TList.Create;
end;

destructor TPasDeclarations.Destroy;
var
  i: Integer;
>>>>>>> graemeg/cpstrnew
begin
  FreeAndNil(FTokens);
  inherited Destroy;
end;

<<<<<<< HEAD
{ TPasClassConstructor }

function TPasClassConstructor.TypeName: string;
begin
  Result:='class '+ inherited TypeName;
end;

{ TPasImplRaise }

destructor TPasImplRaise.Destroy;
begin
  FreeAndNil(ExceptObject);
  FreeAndNil(ExceptAddr);
  Inherited;
end;

{ TPasImplRepeatUntil }

destructor TPasImplRepeatUntil.Destroy;
=======
  inherited Destroy;
end;

destructor TPasModule.Destroy;
>>>>>>> graemeg/cpstrnew
begin
  FreeAndNil(ConditionExpr);
  inherited Destroy;
end;

function TPasImplRepeatUntil.Condition: string;
begin
  If Assigned(ConditionExpr) then
    Result:=ConditionExpr.GetDeclaration(True)
  else
    Result:='';
end;

{ TPasImplSimple }

destructor TPasImplSimple.Destroy;
begin
  FreeAndNil(Expr);
  inherited Destroy;
end;

{ TPasImplAssign }

destructor TPasImplAssign.Destroy;
begin
  FreeAndNil(Left);
  FreeAndNil(Right);
  inherited Destroy;
end;

{ TPasExportSymbol }

destructor TPasExportSymbol.Destroy;
begin
  FreeAndNil(ExportName);
  FreeAndNil(ExportIndex);
  inherited Destroy;
end;

function TPasExportSymbol.ElementTypeName: string;
begin
  Result:='Export'
end;

function TPasExportSymbol.GetDeclaration(full: boolean): string;
begin
  Result:=Name;
  if (ExportName<>Nil) then
    Result:=Result+' name '+ExportName.GetDeclaration(Full)
  else if (ExportIndex<>Nil) then
    Result:=Result+' index '+ExportIndex.GetDeclaration(Full);
end;

{ TPasUnresolvedUnitRef }

function TPasUnresolvedUnitRef.ElementTypeName: string;
begin
  Result:=SPasTreeUnit;
end;

{ TPasLibrary }

destructor TPasLibrary.Destroy;
begin
  FreeAndNil(LibrarySection);
  inherited Destroy;
end;

function TPasLibrary.ElementTypeName: string;
begin
  Result:=inherited ElementTypeName;
end;

{ TPasProgram }

destructor TPasProgram.Destroy;
begin
  FreeAndNil(ProgramSection);
  inherited Destroy;
end;

function TPasProgram.ElementTypeName: string;
begin
  Result:=inherited ElementTypeName;
end;

{ TPasUnitModule }

function TPasUnitModule.ElementTypeName: string;
begin
  Result:=SPasTreeUnit;
end;

{ TPasStringType }


{$IFNDEF FPC}
  const
    LineEnding = sLineBreak;
{$ENDIF}

{ Parse tree element type name functions }

function TPasElement.ElementTypeName: string; begin Result := SPasTreeElement end;

Function TPasElement.HintsString: String;

Var
  H : TPasmemberHint;

begin
<<<<<<< HEAD
  Result:='';
  For H := Low(TPasmemberHint) to High(TPasMemberHint) do
    if H in Hints then
      begin
      If (Result<>'') then
        Result:=Result+'; ';
      Result:=Result+cPasMemberHint[h];
      end;
=======
  inherited Create(AName, AParent);
  IsPacked := False;                     // 12/04/04 - Dave - Added
  IsShortDefinition := False;
  Members := TList.Create;
  Modifiers := TStringList.Create;
  ClassVars := TList.Create;
  Interfaces:= TList.Create;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
end;

function TPasDeclarations.ElementTypeName: string; begin Result := SPasTreeSection end;
function TPasModule.ElementTypeName: string; begin Result := SPasTreeModule end;
function TPasPackage.ElementTypeName: string; begin Result := SPasTreePackage end;
function TPasResString.ElementTypeName: string; begin Result := SPasTreeResString end;
function TPasType.ElementTypeName: string; begin Result := SPasTreeType end;
function TPasPointerType.ElementTypeName: string; begin Result := SPasTreePointerType end;
function TPasAliasType.ElementTypeName: string; begin Result := SPasTreeAliasType end;
function TPasTypeAliasType.ElementTypeName: string; begin Result := SPasTreeTypeAliasType end;
function TPasClassOfType.ElementTypeName: string; begin Result := SPasTreeClassOfType end;
function TPasRangeType.ElementTypeName: string; begin Result := SPasTreeRangeType end;
function TPasArrayType.ElementTypeName: string; begin Result := SPasTreeArrayType end;
function TPasFileType.ElementTypeName: string; begin Result := SPasTreeFileType end;
function TPasEnumValue.ElementTypeName: string; begin Result := SPasTreeEnumValue end;

destructor TPasEnumValue.Destroy;
begin
<<<<<<< HEAD
  FreeAndNil(Value);
=======
  for i := 0 to Members.Count - 1 do
    TPasElement(Members[i]).Release;
  Members.Free;
  if Assigned(AncestorType) then
    AncestorType.Release;
  Modifiers.Free;
  ClassVars.Free;
  Interfaces.Free;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
  inherited Destroy;
end;

function TPasEnumValue.AssignedValue: string;
begin
  If Assigned(Value) then
    Result:=Value.GetDeclaration(True)
  else
    Result:='';
end;

function TPasEnumType.ElementTypeName: string; begin Result := SPasTreeEnumType end;
function TPasSetType.ElementTypeName: string; begin Result := SPasTreeSetType end;
function TPasRecordType.ElementTypeName: string; begin Result := SPasTreeRecordType end;
function TPasArgument.ElementTypeName: string; begin Result := SPasTreeArgument end;
function TPasProcedureType.ElementTypeName: string; begin Result := SPasTreeProcedureType end;
function TPasResultElement.ElementTypeName: string; begin Result := SPasTreeResultElement end;
function TPasFunctionType.ElementTypeName: string; begin Result := SPasTreeFunctionType end;
function TPasUnresolvedTypeRef.ElementTypeName: string; begin Result := SPasTreeUnresolvedTypeRef end;
function TPasVariable.ElementTypeName: string; begin Result := SPasTreeVariable end;
function TPasConst.ElementTypeName: string; begin Result := SPasTreeConst end;
function TPasProperty.ElementTypeName: string; begin Result := SPasTreeProperty end;
function TPasOverloadedProc.ElementTypeName: string; begin Result := SPasTreeOverloadedProcedure end;
function TPasProcedure.ElementTypeName: string; begin Result := SPasTreeProcedure end;

function TPasFunction.GetFT: TPasFunctionType;
begin
  Result:=ProcType as TPasFunctionType;
end;

function TPasFunction.ElementTypeName: string; begin Result := SPasTreeFunction end;
function TPasClassProcedure.ElementTypeName: string; begin Result := SPasTreeClassProcedure; end;
function TPasClassConstructor.ElementTypeName: string; begin Result := SPasTreeClassConstructor; end;
function TPasClassDestructor.ElementTypeName: string; begin Result := SPasTreeClassDestructor; end;

function TPasClassDestructor.TypeName: string;
begin
<<<<<<< HEAD
  Result:='destructor';
=======
  for i := 0 to Args.Count - 1 do
    TPasArgument(Args[i]).Release;
  FreeAndNil(Args);
  inherited Destroy;
>>>>>>> graemeg/cpstrnew
end;

function TPasClassFunction.ElementTypeName: string; begin Result := SPasTreeClassFunction; end;

class function TPasOperator.OperatorTypeToToken(T: TOperatorType): String;
begin
  Result:=OperatorTokens[T];
end;

class function TPasOperator.OperatorTypeToOperatorName(T: TOperatorType
  ): String;
begin
  Result:=OperatorNames[T];
end;

class function TPasOperator.TokenToOperatorType(S: String): TOperatorType;
begin
  Result:=High(TOperatorType);
  While (Result>otUnknown) and (CompareText(S,OperatorTokens[Result])<>0) do
    Result:=Pred(Result);
end;

class function TPasOperator.NameToOperatorType(S: String): TOperatorType;
begin
  Result:=High(TOperatorType);
  While (Result>otUnknown) and (CompareText(S,OperatorNames[Result])<>0) do
    Result:=Pred(Result);
end;

Function TPasOperator.NameSuffix : String;

Var
  I : Integer;

begin
  Result:='(';
  if Assigned(ProcType) and Assigned(ProcType.Args) then
  for i:=0 to ProcType.Args.Count-1 do
    begin
    if i>0 then
      Result:=Result+',';
    Result:=Result+TPasArgument(ProcType.Args[i]).ArgType.Name;
    end;
  Result:=Result+')';
  if Assigned(TPasFunctionType(ProcType)) and
     Assigned(TPasFunctionType(ProcType).ResultEl) and
     Assigned(TPasFunctionType(ProcType).ResultEl.ResultType) then
    Result:=Result+':'+TPasFunctionType(ProcType).ResultEl.ResultType.Name;
end;

procedure TPasOperator.CorrectName;

begin
  Name:=OperatorNames[OperatorType]+NameSuffix;
end;

function TPasOperator.OldName(WithPath : Boolean): String;

Var
  I : Integer;
  S : String;
begin
  Result:=TypeName+' '+OperatorTokens[OperatorType];
  Result := Result + '(';
  if Assigned(ProcType) then
    begin
    for i := 0 to ProcType.Args.Count - 1 do
      begin
      if i > 0 then
        Result := Result + ', ';
      Result := Result + TPasArgument(ProcType.Args[i]).ArgType.Name;
      end;
    Result := Result + '): ' + TPasFunctionType(ProcType).ResultEl.ResultType.Name;
    If WithPath then
      begin
      S:=Self.ParentPath;
      if (S<>'') then
        Result:=S+'.'+Result;
      end;
    end;
end;

function TPasOperator.ElementTypeName: string;
begin
<<<<<<< HEAD
  Result := SPasTreeOperator
=======
  { Attention, in derived classes, VarType isn't necessarily set!
    (e.g. in Constants) }
  if Assigned(VarType) then
    VarType.Release;
  if Assigned(Expr) then
    Expr.Release;
  inherited Destroy;
>>>>>>> graemeg/cpstrnew
end;

function TPasConstructor.ElementTypeName: string; begin Result := SPasTreeConstructor end;
function TPasDestructor.ElementTypeName: string; begin Result := SPasTreeDestructor end;
function TPasProcedureImpl.ElementTypeName: string; begin Result := SPasTreeProcedureImpl end;
function TPasConstructorImpl.ElementTypeName: string; begin Result := SPasTreeConstructorImpl end;
function TPasDestructorImpl.ElementTypeName: string; begin Result := SPasTreeDestructorImpl end;
function TPasStringType.ElementTypeName: string; begin Result:=SPasStringType;end;

function TPasClassType.ElementTypeName: string;
begin
  case ObjKind of
    okObject: Result := SPasTreeObjectType;
    okClass: Result := SPasTreeClassType;
    okInterface: Result := SPasTreeInterfaceType;
    okGeneric : Result := SPasTreeGenericType;
    okSpecialize : Result := SPasTreeSpecializedType;
    okClassHelper : Result:=SPasClassHelperType;
    okRecordHelper : Result:=SPasRecordHelperType;
  end;
end;

function TPasClassType.FindMember(MemberClass: TPTreeElement; const MemberName: String): TPasElement;

Var
  I : Integer;

begin
//  Writeln('Looking for ',MemberName,'(',MemberClass.ClassName,') in ',Name);
  Result:=Nil;
  I:=0;
  While (Result=Nil) and (I<Members.Count) do
    begin
    Result:=TPasElement(Members[i]);
    if (Result.ClassType<>MemberClass) or (CompareText(Result.Name,MemberName)<>0) then
      Result:=Nil;
    Inc(I);
    end;
end;

function TPasClassType.FindMemberInAncestors(MemberClass: TPTreeElement;
  const MemberName: String): TPasElement;

  Function A (C : TPasClassType) : TPasClassType;

  begin
    if C.AncestorType is TPasClassType then
      result:=TPasClassType(C.AncestorType)
    else
      result:=Nil;
  end;

Var
  C : TPasClassType;

begin
  Result:=Nil;
  C:=A(Self);
  While (Result=Nil) and (C<>Nil) do
    begin
    Result:=C.FindMember(MemberClass,MemberName);
    C:=A(C);
    end;
end;

function TPasClassType.InterfaceGUID: string;
begin
  If Assigned(GUIDExpr) then
    Result:=GUIDExpr.GetDeclaration(True)
  else
    Result:=''
end;

function TPasClassType.IsPacked: Boolean;
begin
  Result:=PackMode<>pmNone;
end;



{ All other stuff: }

procedure TPasElement.ProcessHints(const ASemiColonPrefix: boolean; var AResult: string);
var
  S : String;
begin
<<<<<<< HEAD
  if Hints <> [] then
    begin
    if ASemiColonPrefix then
      AResult := AResult + ';';
    S:=HintsString;
    if (S<>'') then
      AResult:=AResult+' '+S;
    if ASemiColonPrefix then
      AResult:=AResult+';';
    end;
=======
  if Assigned(ProcType) then
    ProcType.Release;
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
>>>>>>> graemeg/cpstrnew
end;

constructor TPasElement.Create(const AName: string; AParent: TPasElement);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  inherited Create;
  FName := AName;
  FParent := AParent;
end;

procedure TPasElement.AddRef;
begin
  Inc(FRefCount);
end;

{ $define debugrefcount}

procedure TPasElement.Release;

{$ifdef debugrefcount}
Var
  Cn : String;
  {$endif}

=======
  Result := 'procedure';
end;

=======
  Result := 'procedure';
end;

>>>>>>> graemeg/cpstrnew
constructor TPasProcedureImpl.Create(const AName: string; AParent: TPasElement);
>>>>>>> graemeg/cpstrnew
begin
{$ifdef debugrefcount}
  CN:=ClassName;
  CN:=CN+' '+IntToStr(FRefCount);
  If Assigned(Parent) then
    CN:=CN+' ('+Parent.ClassName+')';
  Writeln('Release : ',Cn);
{$endif}
  if FRefCount = 0 then
    Free
  else
    Dec(FRefCount);
{$ifdef debugrefcount}  Writeln('Released : ',Cn); {$endif}
end;

function TPasElement.FullPath: string;

var
  p: TPasElement;

begin
  Result := '';
  p := Parent;
  while Assigned(p) and not p.InheritsFrom(TPasDeclarations) do
  begin
    if (not (p is TPasOverloadedProc)) and (Length(p.Name) > 0) then
      if Length(Result) > 0 then
        Result := p.Name + '.' + Result
      else
        Result := p.Name;
    p := p.Parent;
  end;
end;

function TPasElement.FullName: string;


begin
  Result := FullPath;
  if Result<>'' then
    Result:=Result+'.'+Name
  else
    Result:=Name;
end;

function TPasElement.ParentPath: string;

var
  p: TPasElement;
begin
  Result:='';
  p := Parent;
  while Assigned(p) do
  begin
    if (Not (p is TPasOverloadedProc)) and (Length(p.Name) > 0) then
      if Length(Result) > 0 then
        Result := p.Name + '.' + Result
      else
        Result := p.Name;
    p := p.Parent;
  end;
end;

function TPasElement.PathName: string;

begin
  Result := ParentPath;
  if Result<>'' then
    Result:=Result+'.'+Name
  else
    Result:=Name;
end;

function TPasElement.GetModule: TPasModule;

Var
  p : TPaselement;
begin
  if self is  TPasPackage then
    Result := nil
  else
    begin
    P:=Self;
    While (P<>Nil) and Not (P is TPasModule) do
      P:=P.Parent;
    Result:=TPasModule(P);
    end;
end;

function TPasElement.GetDeclaration(full: Boolean): string;

begin
  if Full then
    Result := Name
  else
    Result := '';
end;

procedure TPasElement.Accept(Visitor: TPassTreeVisitor);
begin
  Visitor.Visit(Self);
end;

<<<<<<< HEAD
constructor TPasDeclarations.Create(const AName: string; AParent: TPasElement);
=======
procedure TPasImplIfElse.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if IfBranch=nil then
    IfBranch:=Element
  else if ElseBranch=nil then
    ElseBranch:=Element
  else
    raise Exception.Create('TPasImplIfElse.AddElement if and else already set - please report this bug');
end;

<<<<<<< HEAD
=======
procedure TPasImplIfElse.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if IfBranch=nil then
    IfBranch:=Element
  else if ElseBranch=nil then
    ElseBranch:=Element
  else
    raise Exception.Create('TPasImplIfElse.AddElement if and else already set - please report this bug');
end;

>>>>>>> graemeg/cpstrnew
function TPasImplIfElse.CloseOnSemicolon: boolean;
begin
  Result:=ElseBranch<>nil;
end;

destructor TPasImplForLoop.Destroy;
begin
  if Assigned(Variable) then
    Variable.Release;
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplForLoop.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element
  else
    raise Exception.Create('TPasImplForLoop.AddElement body already set - please report this bug');
end;

constructor TPasImplBlock.Create(const AName: string; AParent: TPasElement);
>>>>>>> graemeg/cpstrnew
begin
  inherited Create(AName, AParent);
  Declarations := TFPList.Create;
  ResStrings := TFPList.Create;
  Types := TFPList.Create;
  Consts := TFPList.Create;
  Classes := TFPList.Create;
  Functions := TFPList.Create;
  Variables := TFPList.Create;
  Properties := TFPList.Create;
  ExportSymbols := TFPList.Create;
end;

=======
procedure TPasElement.Accept(Visitor: TPassTreeVisitor);
begin
  Visitor.Visit(Self);
end;

constructor TPasDeclarations.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Declarations := TList.Create;
  ResStrings := TList.Create;
  Types := TList.Create;
  Consts := TList.Create;
  Classes := TList.Create;
  Functions := TList.Create;
  Variables := TList.Create;
  Properties := TList.Create;
end;

>>>>>>> graemeg/cpstrnew
destructor TPasDeclarations.Destroy;
var
  i: Integer;
begin
<<<<<<< HEAD
  ExportSymbols.Free;
=======
>>>>>>> graemeg/cpstrnew
  Variables.Free;
  Functions.Free;
  Classes.Free;
  Consts.Free;
  Types.Free;
  ResStrings.Free;
  Properties.Free;
  for i := 0 to Declarations.Count - 1 do
    TPasElement(Declarations[i]).Release;
  Declarations.Free;

<<<<<<< HEAD
  inherited Destroy;
end;

<<<<<<< HEAD
<<<<<<< HEAD
destructor TPasModule.Destroy;
begin
  if Assigned(InterfaceSection) then
    InterfaceSection.Release;
  if Assigned(ImplementationSection) then
    ImplementationSection.Release;
 FreeAndNil(InitializationSection);
 FreeAndNil(FinalizationSection);
 inherited Destroy;
end;

=======
=======
>>>>>>> graemeg/cpstrnew
procedure TPasImplBlock.AddElement(Element: TPasImplElement);
begin
  Elements.Add(Element);
end;

function TPasImplBlock.AddCommand(const ACommand: string): TPasImplCommand;
begin
  Result := TPasImplCommand.Create('', Self);
  Result.Command := ACommand;
  AddElement(Result);
end;

function TPasImplBlock.AddCommands: TPasImplCommands;
begin
  Result := TPasImplCommands.Create('', Self);
  AddElement(Result);
<<<<<<< HEAD
=======
end;

function TPasImplBlock.AddBeginBlock: TPasImplBeginBlock;
begin
  Result := TPasImplBeginBlock.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddRepeatUntil: TPasImplRepeatUntil;
begin
  Result := TPasImplRepeatUntil.Create('', Self);
  AddElement(Result);
>>>>>>> graemeg/cpstrnew
end;

function TPasImplBlock.AddBeginBlock: TPasImplBeginBlock;
begin
  Result := TPasImplBeginBlock.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddRepeatUntil: TPasImplRepeatUntil;
begin
  Result := TPasImplRepeatUntil.Create('', Self);
  AddElement(Result);
end;
>>>>>>> graemeg/cpstrnew

constructor TPasPackage.Create(const AName: string; AParent: TPasElement);
begin
<<<<<<< HEAD
  if (Length(AName) > 0) and (AName[1] <> '#') then
    inherited Create('#' + AName, AParent)
  else
    inherited Create(AName, AParent);
  Modules := TFPList.Create;
=======
  Result := TPasImplIfElse.Create('', Self);
  Result.Condition := ACondition;
  AddElement(Result);
<<<<<<< HEAD
=======
end;

function TPasImplBlock.AddWhileDo(const ACondition: string): TPasImplWhileDo;
begin
  Result := TPasImplWhileDo.Create('', Self);
  Result.Condition := ACondition;
  AddElement(Result);
end;

function TPasImplBlock.AddWithDo(const Expression: string): TPasImplWithDo;
begin
  Result := TPasImplWithDo.Create('', Self);
  Result.AddExpression(Expression);
  AddElement(Result);
end;

function TPasImplBlock.AddCaseOf(const Expression: string): TPasImplCaseOf;
begin
  Result := TPasImplCaseOf.Create('', Self);
  Result.Expression := Expression;
  AddElement(Result);
>>>>>>> graemeg/cpstrnew
end;

function TPasImplBlock.AddWhileDo(const ACondition: string): TPasImplWhileDo;
begin
  Result := TPasImplWhileDo.Create('', Self);
  Result.Condition := ACondition;
  AddElement(Result);
end;

function TPasImplBlock.AddWithDo(const Expression: string): TPasImplWithDo;
begin
  Result := TPasImplWithDo.Create('', Self);
  Result.AddExpression(Expression);
  AddElement(Result);
end;

function TPasImplBlock.AddCaseOf(const Expression: string): TPasImplCaseOf;
begin
  Result := TPasImplCaseOf.Create('', Self);
  Result.Expression := Expression;
  AddElement(Result);
>>>>>>> graemeg/cpstrnew
end;

destructor TPasPackage.Destroy;
var
  i: Integer;
begin
<<<<<<< HEAD
  for i := 0 to Modules.Count - 1 do
    TPasModule(Modules[i]).Release;
  Modules.Free;
  inherited Destroy;
=======
  Result := TPasImplForLoop.Create('', Self);
  Result.Variable := AVar;
  Result.StartValue := AStartValue;
  Result.EndValue := AEndValue;
  AddElement(Result);
<<<<<<< HEAD
=======
end;

function TPasImplBlock.AddForLoop(const AVarName, AStartValue,
  AEndValue: string; ADownTo: Boolean): TPasImplForLoop;
begin
  Result := TPasImplForLoop.Create('', Self);
  Result.VariableName := AVarName;
  Result.StartValue := AStartValue;
  Result.EndValue := AEndValue;
  Result.Down := ADownTo;
  AddElement(Result);
end;

function TPasImplBlock.AddTry: TPasImplTry;
begin
  Result := TPasImplTry.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddExceptOn(const VarName, TypeName: string
  ): TPasImplExceptOn;
begin
  Result:=TPasImplExceptOn.Create('',Self);
  Result.VariableName:=VarName;
  Result.TypeName:=TypeName;
  AddElement(Result);
end;

function TPasImplBlock.AddRaise: TPasImplRaise;
begin
  Result:=TPasImplRaise.Create('',Self);
  AddElement(Result);
end;

function TPasImplBlock.AddLabelMark(const Id: string): TPasImplLabelMark;
begin
  Result:=TPasImplLabelMark.Create('', Self);
  Result.LabelId:=Id;
  AddElement(Result);
end;

function TPasImplBlock.AddAssign(left,right:TPasExpr):TPasImplAssign;
begin
  Result:=TPasImplAssign.Create('', Self);
  Result.left:=left;
  Result.right:=right;
  AddElement(Result);
end;

function TPasImplBlock.AddSimple(exp:TPasExpr):TPasImplSimple;
begin
  Result:=TPasImplSimple.Create('', Self);
  Result.expr:=exp;
  AddElement(Result);
end;

function TPasImplBlock.CloseOnSemicolon: boolean;
begin
  Result:=false;
>>>>>>> graemeg/cpstrnew
end;

function TPasImplBlock.AddForLoop(const AVarName, AStartValue,
  AEndValue: string; ADownTo: Boolean): TPasImplForLoop;
begin
  Result := TPasImplForLoop.Create('', Self);
  Result.VariableName := AVarName;
  Result.StartValue := AStartValue;
  Result.EndValue := AEndValue;
  Result.Down := ADownTo;
  AddElement(Result);
end;

function TPasImplBlock.AddTry: TPasImplTry;
begin
  Result := TPasImplTry.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddExceptOn(const VarName, TypeName: string
  ): TPasImplExceptOn;
begin
  Result:=TPasImplExceptOn.Create('',Self);
  Result.VariableName:=VarName;
  Result.TypeName:=TypeName;
  AddElement(Result);
end;

function TPasImplBlock.AddRaise: TPasImplRaise;
begin
  Result:=TPasImplRaise.Create('',Self);
  AddElement(Result);
end;

function TPasImplBlock.AddLabelMark(const Id: string): TPasImplLabelMark;
begin
  Result:=TPasImplLabelMark.Create('', Self);
  Result.LabelId:=Id;
  AddElement(Result);
end;

function TPasImplBlock.AddAssign(left,right:TPasExpr):TPasImplAssign;
begin
  Result:=TPasImplAssign.Create('', Self);
  Result.left:=left;
  Result.right:=right;
  AddElement(Result);
end;

function TPasImplBlock.AddSimple(exp:TPasExpr):TPasImplSimple;
begin
  Result:=TPasImplSimple.Create('', Self);
  Result.expr:=exp;
  AddElement(Result);
end;

function TPasImplBlock.CloseOnSemicolon: boolean;
begin
  Result:=false;
>>>>>>> graemeg/cpstrnew
end;


destructor TPasPointerType.Destroy;
begin
  if Assigned(DestType) then
    DestType.Release;
  inherited Destroy;
end;


destructor TPasAliasType.Destroy;
begin
  if Assigned(DestType) then
    DestType.Release;
  inherited Destroy;
end;


destructor TPasArrayType.Destroy;
begin
  if Assigned(ElType) then
    ElType.Release;
  inherited Destroy;
end;

destructor TPasFileType.Destroy;
begin
  if Assigned(ElType) then
    ElType.Release;
  inherited Destroy;
end;


constructor TPasEnumType.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Values := TFPList.Create;
end;

destructor TPasEnumType.Destroy;
var
  i: Integer;
begin
  for i := 0 to Values.Count - 1 do
    TPasEnumValue(Values[i]).Release;
  Values.Free;
  inherited Destroy;
end;

procedure TPasEnumType.GetEnumNames(Names: TStrings);
var
  i: Integer;
begin
  with Values do
  begin
    for i := 0 to Count - 2 do
      Names.Add(TPasEnumValue(Items[i]).Name + ',');
    if Count > 0 then
      Names.Add(TPasEnumValue(Items[Count - 1]).Name);
  end;
end;


destructor TPasSetType.Destroy;
begin
  if Assigned(EnumType) then
    begin
    EnumType.Release;
    end;
  inherited Destroy;
end;


constructor TPasVariant.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Values := TFPList.Create;
end;

destructor TPasVariant.Destroy;

Var
  I : Integer;

begin
  For I:=0 to Values.Count-1 do
    TObject(Values[i]).Free;
  Values.Free;
  if Assigned(Members) then
    Members.Release;
  inherited Destroy;
end;

function TPasVariant.GetDeclaration(full: boolean): string;

Var
  i : Integer;
  S : TStrings;

begin
  Result:='';
  For I:=0 to Values.Count-1 do
    begin
    if (Result<>'') then
      Result:=Result+', ';
    Result:=Result+TPasElement(Values[i]).GetDeclaration(False);
    Result:=Result+': ('+sLineBreak;
    S:=TStringList.Create;
    try
      Members.GetMembers(S);
      Result:=Result+S.Text;
    finally
      S.Free;
    end;
    Result:=Result+');';
    end;
end;


constructor TPasRecordType.Create(const AName: string; AParent: TPasElement);
begin
<<<<<<< HEAD
  inherited Create(AName, AParent);
  Members := TFPList.Create;
=======
  S:=TStringList.Create;
  T:=TStringList.Create;
  Try
    Temp:='record';
    If IsPacked then
      if IsBitPacked then
        Temp:='bitpacked '+Temp
      else
        Temp:='packed '+Temp;
    If Full then
      Temp:=Name+' = '+Temp;
    S.Add(Temp);
    For I:=0 to Members.Count-1 do
      begin
      Temp:=TPasVariable(Members[i]).GetDeclaration(True);
      If Pos(LineEnding,Temp)>0 then
        begin
        T.Text:=Temp;
        For J:=0 to T.Count-1 do
          if J=T.Count-1 then
            S.Add('  '+T[J]+';')
          else
            S.Add('  '+T[J])
        end
      else
        S.Add('  '+Temp+';');
      end;
    S.Add('end');
    Result:=S.Text;
    ProcessHints(False, Result);
  finally
    S.free;
    T.free;
  end;
>>>>>>> graemeg/cpstrnew
end;

destructor TPasRecordType.Destroy;
var
  i: Integer;
begin
  for i := 0 to Members.Count - 1 do
    TPasVariable(Members[i]).Release;
  Members.Free;

  if Assigned(VariantType) then
    VariantType.Release;

  if Assigned(Variants) then
  begin
    for i := 0 to Variants.Count - 1 do
      TPasVariant(Variants[i]).Release;
    Variants.Free;
  end;

  inherited Destroy;
end;


constructor TPasClassType.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  PackMode:=pmNone;                     // 12/04/04 - Dave - Added
  IsShortDefinition := False;
  Members := TFPList.Create;
  Modifiers := TStringList.Create;
  ClassVars := TFPList.Create;
  Interfaces:= TFPList.Create;
  GenericTemplateTypes:=TFPList.Create;

end;

destructor TPasClassType.Destroy;
var
  i: Integer;
begin
  for i := 0 to Members.Count - 1 do
    TPasElement(Members[i]).Release;
  for i := 0 to Interfaces.Count - 1 do
    TPasElement(Interfaces[i]).Release;
  Members.Free;
  if Assigned(AncestorType) then
    AncestorType.Release;
  if Assigned(HelperForType) then
    HelperForType.Release;
  FreeAndNil(GUIDExpr);
  Modifiers.Free;
  ClassVars.Free;
  Interfaces.Free;
  for i := 0 to GenericTemplateTypes.Count - 1 do
    TPasElement(GenericTemplateTypes[i]).Release;
  GenericTemplateTypes.Free;
  inherited Destroy;
end;


destructor TPasArgument.Destroy;
begin
  if Assigned(ArgType) then
    ArgType.Release;
  FreeAndNil(ValueExpr);
  inherited Destroy;
end;


constructor TPasProcedureType.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Args := TFPList.Create;
end;

destructor TPasProcedureType.Destroy;
var
  i: Integer;
begin
  for i := 0 to Args.Count - 1 do
    TPasArgument(Args[i]).Release;
  FreeAndNil(Args);
  inherited Destroy;
end;

class function TPasProcedureType.TypeName: string;
begin
  Result := 'procedure';
end;

function TPasProcedureType.CreateArgument(const AName,
  AUnresolvedTypeName: string): TPasArgument;
begin
  Result := TPasArgument.Create(AName, Self);
  Args.Add(Result);
  Result.ArgType := TPasUnresolvedTypeRef.Create(AUnresolvedTypeName, Result);
end;


destructor TPasResultElement.Destroy;
begin
  if Assigned(ResultType) then
    ResultType.Release;
  inherited Destroy;
end;


destructor TPasFunctionType.Destroy;
begin
  if Assigned(ResultEl) then
    ResultEl.Release;
  inherited Destroy;
end;


class function TPasFunctionType.TypeName: string;
begin
  Result := 'function';
end;


constructor TPasUnresolvedTypeRef.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, nil);
end;


destructor TPasVariable.Destroy;
begin
//  FreeAndNil(Expr);
  { Attention, in derived classes, VarType isn't necessarily set!
    (e.g. in Constants) }
  if Assigned(VarType) then
    VarType.Release;
  if Assigned(Expr) then
    Expr.Release;
  inherited Destroy;
end;


constructor TPasProperty.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Args := TFPList.Create;
end;

destructor TPasProperty.Destroy;
var
  i: Integer;
begin
  for i := 0 to Args.Count - 1 do
    TPasArgument(Args[i]).Release;
  Args.Free;
  FreeAndNil(DefaultExpr);
  FreeAndNil(IndexExpr);
  inherited Destroy;
end;


constructor TPasOverloadedProc.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Overloads := TFPList.Create;
end;

destructor TPasOverloadedProc.Destroy;
var
  i: Integer;
begin
  for i := 0 to Overloads.Count - 1 do
    TPasProcedure(Overloads[i]).Release;
  Overloads.Free;
  inherited Destroy;
end;

function TPasOverloadedProc.TypeName: string;
begin
  if Assigned(TPasProcedure(Overloads[0]).ProcType) then
    Result := TPasProcedure(Overloads[0]).ProcType.TypeName
  else
    SetLength(Result, 0);
end;

function TPasProcedure.GetCallingConvention: TCallingConvention;
begin
  Result:=ccDefault;
  if Assigned(ProcType) then
    Result:=ProcType.CallingConvention;
end;

procedure TPasProcedure.SetCallingConvention(AValue: TCallingConvention);
begin
  if Assigned(ProcType) then
    ProcType.CallingConvention:=AValue;
end;

destructor TPasProcedure.Destroy;
begin
  if Assigned(ProcType) then
    ProcType.Release;
  if Assigned(Body) then
    Body.Release;
  FreeAndNil(PublicName);
  FreeAndNil(LibraryExpr);
  FreeAndNil(LibrarySymbolName);
  inherited Destroy;
end;

function TPasProcedure.TypeName: string;
begin
  Result := 'procedure';
end;

constructor TPasProcedureImpl.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Locals := TFPList.Create;
end;

destructor TPasProcedureImpl.Destroy;
var
  i: Integer;
begin
  if Assigned(Body) then
    Body.Release;

  for i := 0 to Locals.Count - 1 do
    TPasElement(Locals[i]).Release;
  Locals.Free;

  if Assigned(ProcType) then
    ProcType.Release;

  inherited Destroy;
end;

function TPasProcedureImpl.TypeName: string;
begin
  Result := ProcType.TypeName;
end;


function TPasConstructorImpl.TypeName: string;
begin
  Result := 'constructor';
end;

function TPasDestructorImpl.TypeName: string;
begin
  Result := 'destructor';
end;


constructor TPasImplCommands.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Commands := TStringList.Create;
end;

destructor TPasImplCommands.Destroy;
begin
  Commands.Free;
  inherited Destroy;
end;


destructor TPasImplIfElse.Destroy;
begin
  FreeAndNil(ConditionExpr);
  if Assigned(IfBranch) then
    IfBranch.Release;
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

procedure TPasImplIfElse.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if IfBranch=nil then
    begin
    IfBranch:=Element;
    element.AddRef;
    end
  else if ElseBranch=nil then
    begin
    ElseBranch:=Element;
    Element.AddRef;
    end
  else
    raise Exception.Create('TPasImplIfElse.AddElement if and else already set - please report this bug');
end;

function TPasImplIfElse.CloseOnSemicolon: boolean;
begin
  Result:=ElseBranch<>nil;
end;

function TPasImplIfElse.Condition: string;
begin
  If Assigned(ConditionExpr) then
    Result:=ConditionExpr.GetDeclaration(True);
end;

destructor TPasImplForLoop.Destroy;
begin
  FreeAndNil(StartExpr);
  FreeAndNil(EndExpr);
  if Assigned(Variable) then
    Variable.Release;
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplForLoop.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    begin
    Body:=Element;
    Body.AddRef;
    end
  else
    raise Exception.Create('TPasImplForLoop.AddElement body already set - please report this bug');
end;

function TPasImplForLoop.Down: boolean;
begin
  Result:=(LoopType=ltDown);
end;

function TPasImplForLoop.StartValue: String;
begin
  If Assigned(StartExpr) then
    Result:=StartExpr.GetDeclaration(true)
  else
    Result:='';
end;

function TPasImplForLoop.EndValue: string;
begin
  If Assigned(EndExpr) then
    Result:=EndExpr.GetDeclaration(true)
  else
    Result:='';
end;

constructor TPasImplBlock.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Elements := TFPList.Create;
end;

destructor TPasImplBlock.Destroy;
var
  i: Integer;
begin
  for i := 0 to Elements.Count - 1 do
    TPasImplElement(Elements[i]).Release;
  Elements.Free;
  inherited Destroy;
end;

procedure TPasImplBlock.AddElement(Element: TPasImplElement);
begin
  Elements.Add(Element);
end;

function TPasImplBlock.AddCommand(const ACommand: string): TPasImplCommand;
begin
  Result := TPasImplCommand.Create('', Self);
  Result.Command := ACommand;
  AddElement(Result);
end;

function TPasImplBlock.AddCommands: TPasImplCommands;
begin
  Result := TPasImplCommands.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddBeginBlock: TPasImplBeginBlock;
begin
  Result := TPasImplBeginBlock.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddRepeatUntil: TPasImplRepeatUntil;
begin
  Result := TPasImplRepeatUntil.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddIfElse(const ACondition: TPasExpr): TPasImplIfElse;
begin
  Result := TPasImplIfElse.Create('', Self);
  Result.ConditionExpr := ACondition;
  AddElement(Result);
end;

function TPasImplBlock.AddWhileDo(const ACondition: TPasExpr): TPasImplWhileDo;
begin
  Result := TPasImplWhileDo.Create('', Self);
  Result.ConditionExpr := ACondition;
  AddElement(Result);
end;

function TPasImplBlock.AddWithDo(const Expression: TPasExpr): TPasImplWithDo;
begin
  Result := TPasImplWithDo.Create('', Self);
  Result.AddExpression(Expression);
  AddElement(Result);
end;

function TPasImplBlock.AddCaseOf(const Expression: TPasExpr): TPasImplCaseOf;
begin
  Result := TPasImplCaseOf.Create('', Self);
  Result.CaseExpr:= Expression;
  AddElement(Result);
end;

function TPasImplBlock.AddForLoop(AVar: TPasVariable; const AStartValue,
  AEndValue: TPasExpr): TPasImplForLoop;
begin
  Result := TPasImplForLoop.Create('', Self);
  Result.Variable := AVar;
  Result.StartExpr := AStartValue;
  Result.EndExpr:= AEndValue;
  AddElement(Result);
end;

function TPasImplBlock.AddForLoop(const AVarName: String; AStartValue,
  AEndValue: TPasExpr; ADownTo: Boolean): TPasImplForLoop;
begin
  Result := TPasImplForLoop.Create('', Self);
  Result.VariableName := AVarName;
  Result.StartExpr := AStartValue;
  Result.EndExpr := AEndValue;
  if ADownto then
    Result.Looptype := ltDown;
  AddElement(Result);
end;

function TPasImplBlock.AddTry: TPasImplTry;
begin
  Result := TPasImplTry.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddExceptOn(const VarName, TypeName: TPasExpr
  ): TPasImplExceptOn;
begin
  Result:=TPasImplExceptOn.Create('',Self);
  Result.VarExpr:=VarName;
  Result.TypeExpr:=TypeName;
  AddElement(Result);
end;

function TPasImplBlock.AddRaise: TPasImplRaise;
begin
  Result:=TPasImplRaise.Create('',Self);
  AddElement(Result);
end;

function TPasImplBlock.AddLabelMark(const Id: string): TPasImplLabelMark;
begin
  Result:=TPasImplLabelMark.Create('', Self);
  Result.LabelId:=Id;
  AddElement(Result);
end;

function TPasImplBlock.AddAssign(left,right:TPasExpr):TPasImplAssign;
begin
  Result:=TPasImplAssign.Create('', Self);
  Result.left:=left;
  Result.right:=right;
  AddElement(Result);
end;

function TPasImplBlock.AddSimple(exp:TPasExpr):TPasImplSimple;
begin
  Result:=TPasImplSimple.Create('', Self);
  Result.expr:=exp;
  AddElement(Result);
end;

function TPasImplBlock.CloseOnSemicolon: boolean;
begin
  Result:=false;
end;



{ ---------------------------------------------------------------------

  ---------------------------------------------------------------------}

function TPasModule.GetDeclaration(full : boolean): string;
begin
  Result := 'Unit ' + Name;
end;

{
function TPas.GetDeclaration : string;
begin
  Result:=Name;
end;
}

function TPasResString.GetDeclaration (full : boolean) : string;
begin
  Result:=Expr.GetDeclaration(true);
  If Full Then
    begin
    Result:=Name+' = '+Result;
    ProcessHints(False,Result);
    end;
end;

destructor TPasResString.Destroy;
begin
  If Assigned(Expr) then
    Expr.Release;
  inherited Destroy;
end;

function TPasPointerType.GetDeclaration (full : boolean) : string;
begin
  Result:='^'+DestType.Name;
  If Full then
    begin
    Result:=Name+' = '+Result;
    ProcessHints(False,Result);
    end;
end;

function TPasAliasType.GetDeclaration (full : boolean) : string;
begin
  Result:=DestType.Name;
  If Full then
    begin
    Result:=Name+' = '+Result;
    ProcessHints(False,Result);
    end;
end;

function TPasClassOfType.GetDeclaration (full : boolean) : string;
begin
  Result:='Class of '+DestType.Name;
  If Full then
    begin
    Result:=Name+' = '+Result;
    ProcessHints(False,Result);
    end;
end;

function TPasRangeType.GetDeclaration (full : boolean) : string;
begin
  Result:=RangeStart+'..'+RangeEnd;
  If Full then
    begin
    Result:=Name+' = '+Result;
    ProcessHints(False,Result);
    end;
end;

destructor TPasRangeType.Destroy;
begin
  FreeAndNil(RangeExpr);
  inherited Destroy;
end;

function TPasRangeType.RangeStart: String;
begin
  Result:=RangeExpr.Left.GetDeclaration(False);
end;

function TPasRangeType.RangeEnd: String;
begin
  Result:=RangeExpr.Right.GetDeclaration(False);
end;

function TPasArrayType.GetDeclaration (full : boolean) : string;
begin
  Result:='Array';
  If (IndexRange<>'') then
    Result:=Result+'['+IndexRange+']';
  Result:=Result+' of ';
  If IsPacked then
     Result := 'packed '+Result;      // 12/04/04 Dave - Added
  If Assigned(Eltype) then
    Result:=Result+ElType.Name
  else
    Result:=Result+'const';
  If Full Then
    begin
    Result:=Name+' = '+Result;
    ProcessHints(False,Result);
    end;
end;

function TPasArrayType.IsPacked: Boolean;
begin
  Result:=PackMode=pmPacked;
end;

function TPasFileType.GetDeclaration (full : boolean) : string;
begin
  Result:='File';
  If Assigned(Eltype) then
    Result:=Result+' of '+ElType.Name;
  If Full Then
    begin
    Result:=Name+' = '+Result;
    ProcessHints(False,Result);
    end;
end;

Function IndentStrings(S : TStrings; indent : Integer) : string;

Var
  I,CurrLen,CurrPos : Integer;


begin
  Result:='';
  CurrLen:=0;
  CurrPos:=0;
  For I:=0 to S.Count-1 do
    begin
    CurrLen:=Length(S[i]);
    If (CurrLen+CurrPos)>72 then
      begin
      Result:=Result+LineEnding+StringOfChar(' ',Indent);
      CurrPos:=Indent;
      end;
    Result:=Result+S[i];
    CurrPos:=CurrPos+CurrLen;
    end;
end;

function TPasEnumType.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;

begin
  S:=TStringList.Create;
  Try
    If Full then
      S.Add(Name+' = (')
    else
      S.Add('(');
    GetEnumNames(S);
    S[S.Count-1]:=S[S.Count-1]+')';
    If Full then
      Result:=IndentStrings(S,Length(Name)+4)
    else
      Result:=IndentStrings(S,1);
    if Full then
      ProcessHints(False,Result);
  finally
    S.Free;
  end;
end;

function TPasSetType.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;
  i : Integer;

begin
  If (EnumType is TPasEnumType) and (EnumType.Name='') then
    begin
    S:=TStringList.Create;
    Try
      If Full then
        S.Add(Name+'= Set of (')
      else
        S.Add('Set of (');
      TPasEnumType(EnumType).GetEnumNames(S);
      S[S.Count-1]:=S[S.Count-1]+')';
      I:=Pos('(',S[0]);
      Result:=IndentStrings(S,i);
    finally
      S.Free;
    end;
    end
  else
    begin
    Result:='Set of '+EnumType.Name;
    If Full then
      Result:=Name+' = '+Result;
    end;
  If Full then
    ProcessHints(False,Result);
end;

procedure TPasRecordType.GetMembers(S: TStrings);

Var
  T : TStringList;
  temp : string;
  I,J : integer;
  E : TPasElement;
  CV : TPasMemberVisibility ;

begin
  T:=TStringList.Create;
  try

  CV:=visDefault;
  For I:=0 to Members.Count-1 do
    begin
    E:=TPasElement(Members[i]);
    if E.Visibility<>CV then
      begin
      CV:=E.Visibility;
      if CV<>visDefault then
        S.Add(VisibilityNames[CV]);
      end;
    Temp:=E.GetDeclaration(True);
    If E is TPasProperty then
      Temp:='property '+Temp;
    If Pos(LineEnding,Temp)>0 then
      begin
      T.Text:=Temp;
      For J:=0 to T.Count-1 do
        if J=T.Count-1 then
          S.Add('  '+T[J]+';')
        else
          S.Add('  '+T[J])
      end
    else
      S.Add('  '+Temp+';');
    end;
  if Variants<>nil then
    begin
    temp:='case ';
    if (VariantName<>'') then
      temp:=Temp+variantName+' : ';
    if (VariantType<>Nil) then
      temp:=temp+VariantType.Name;
    S.Add(temp+' of');
    T.Clear;
    For I:=0 to Variants.Count-1 do
      T.Add(TPasVariant(Variants[i]).GetDeclaration(True));
    S.AddStrings(T);
    end;
  finally
    T.Free;
  end;
end;

function TPasRecordType.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;
  temp : string;

begin
  S:=TStringList.Create;
  Try
    Temp:='record';
    If IsPacked then
      if IsBitPacked then
        Temp:='bitpacked '+Temp
      else
        Temp:='packed '+Temp;
    If Full then
      Temp:=Name+' = '+Temp;
    S.Add(Temp);
    GetMembers(S);
    S.Add('end');
    Result:=S.Text;
    ProcessHints(False, Result);
  finally
    S.free;
  end;
end;

function TPasRecordType.IsPacked: Boolean;
begin
  Result:=(PackMode <> pmNone);
end;

function TPasRecordType.IsBitPacked: Boolean;
begin
  Result:=(PackMode=pmBitPacked)
end;

function TPasRecordType.IsAdvancedRecord: Boolean;

Var
  I : Integer;

begin
  Result:=False;
  I:=0;
  While (Not Result) and (I<Members.Count) do
    begin
    Result:=TPasElement(Members[i]).InheritsFrom(TPasProcedureBase) or
            TPasElement(Members[i]).InheritsFrom(TPasProperty);
    Inc(I);
    end;
end;

procedure TPasProcedureType.GetArguments(List : TStrings);

Var
  T : string;
  I : Integer;

begin
  For I:=0 to Args.Count-1 do
    begin
    T:=AccessNames[TPasArgument(Args[i]).Access];
    T:=T+TPasArgument(Args[i]).GetDeclaration(True);
    If I=0 then
      T:='('+T;
    If I<Args.Count-1 then
      List.Add(T+';')
    else
      List.Add(T+')');
    end;
end;

function TPasProcedureType.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;

begin
  S:=TStringList.Create;
  Try
    If Full then
      S.Add(Format('%s = ',[Name]));
    S.Add(TypeName);
    GetArguments(S);
    If IsOfObject then
      S.Add(' of object')
    else if IsNested then
      S.Add(' is nested');
    If Full then
      Result:=IndentStrings(S,Length(S[0])+Length(S[1])+1)
    else
      Result:=IndentStrings(S,Length(S[0])+1);
  finally
    S.Free;
  end;
end;

function TPasFunctionType.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;
  T : string;

begin
  S:=TStringList.Create;
  Try
    If Full then
      S.Add(Format('%s = ',[Name]));
    S.Add(TypeName);
    GetArguments(S);
    If Assigned(ResultEl) then
      begin
      T:=' : ';
      If (ResultEl.ResultType.Name<>'') then
        T:=T+ResultEl.ResultType.Name
      else
        T:=T+ResultEl.ResultType.GetDeclaration(False);
      S.Add(T);
      end;
    If IsOfObject then
      S.Add(' of object');
    If Full then
      Result:=IndentStrings(S,Length(S[0])+Length(S[1])+1)
    else
      Result:=IndentStrings(S,Length(S[0])+1);
  finally
    S.Free;
  end;
end;

function TPasVariable.GetDeclaration (full : boolean) : string;

Const
 Seps : Array[Boolean] of Char = ('=',':');

begin
  If Assigned(VarType) then
    begin
    If VarType.Name='' then
      Result:=VarType.GetDeclaration(False)
    else
      Result:=VarType.Name;
    Result:=Result+Modifiers;
    if (Value<>'') then
      Result:=Result+' = '+Value;
    end
  else
    Result:=Value;
  If Full then
    begin
    Result:=Name+' '+Seps[Assigned(VarType)]+' '+Result;
    Result:=Result+HintsString;
    end;
end;



function TPasVariable.Value: String;
begin
  If Assigned(Expr) then
    Result:=Expr.GetDeclaration(True)
end;

function TPasProperty.GetDeclaration (full : boolean) : string;

Var
  S : string;
  I : Integer;

begin
  If Assigned(VarType) then
    begin
    If VarType.Name='' then
      Result:=VarType.GetDeclaration(False)
    else
      Result:=VarType.Name;
    end
  else if Assigned(Expr) then
    Result:=Expr.GetDeclaration(True);
  S:='';
  If Assigned(Args) and (Args.Count>0) then
    begin
    For I:=0 to Args.Count-1 do
      begin
      If (S<>'') then
        S:=S+';';
      S:=S+TPasElement(Args[i]).GetDeclaration(true);
      end;
    end;
  If S<>'' then
    S:='['+S+']'
  else
    S:=' ';
  If Full then
    begin
    Result:=Name+S+': '+Result;
    If (ImplementsName<>'') then
       Result:=Result+' implements '+ImplementsName;
    end;   
  If IsDefault then
    Result:=Result+'; default';
  ProcessHints(True, Result);
end;

function TPasProperty.ResolvedType: TPasType;

  Function GC(P : TPasProperty) : TPasClassType;

  begin
    if Assigned(P) and Assigned(P.Parent) and (P.Parent is TPasClassType) then
      Result:=P.Parent as TPasClassType
    else
      Result:=Nil;
  end;


Var
  P : TPasProperty;
  C : TPasClassType;

begin
  Result:=FResolvedType;
  if Result=Nil then
    Result:=VarType;
  P:=Self;
  While (Result=Nil) and (P<>Nil) do
    begin
    C:=GC(P);
//    Writeln('Looking for ',Name,' in ancestor ',C.Name);
    P:=TPasProperty(C.FindMemberInAncestors(TPasProperty,Name));
    if Assigned(P) then
      begin
//      Writeln('Found ',Name,' in ancestor : ',P.Name);
      Result:=P.ResolvedType;
      end
    end;
end;

function TPasProperty.IndexValue: String;
begin
  If Assigned(IndexExpr) then
    Result:=IndexExpr.GetDeclaration(true)
  else
    Result:='';
end;

function TPasProperty.DefaultValue: string;
begin
  If Assigned(DefaultExpr) then
    Result:=DefaultExpr.GetDeclaration(true)
  else
    Result:='';
end;

Procedure TPasProcedure.GetModifiers(List : TStrings);

  Procedure DoAdd(B : Boolean; S : string);

  begin
    if B then
      List.add('; '+S);
  end;

begin
  Doadd(IsVirtual,' Virtual');
  DoAdd(IsDynamic,' Dynamic');
  DoAdd(IsOverride,' Override');
  DoAdd(IsAbstract,' Abstract');
  DoAdd(IsOverload,' Overload');
  DoAdd(IsReintroduced,' Reintroduce');
  DoAdd(IsStatic,' Static');
  DoAdd(IsMessage,' Message');
end;

Procedure TPasProcedure.AddModifier(AModifier : TProcedureModifier);

begin
  Include(FModifiers,AModifier);
end;

Function TPasProcedure.IsVirtual : Boolean;
begin
  Result:=pmVirtual in FModifiers;
end;

Function TPasProcedure.IsDynamic : Boolean;
begin
  Result:=pmDynamic in FModifiers;
end;

Function TPasProcedure.IsAbstract : Boolean;
begin
  Result:=pmAbstract in FModifiers;
end;

Function TPasProcedure.IsOverride : Boolean;
begin
  Result:=pmOverride in FModifiers;
end;

Function TPasProcedure.IsExported : Boolean;
begin
  Result:=pmExport in FModifiers;
end;

function TPasProcedure.IsExternal: Boolean;
begin
  Result:=pmExternal in FModifiers;
end;

Function TPasProcedure.IsOverload : Boolean;
begin
  Result:=pmOverload in FModifiers;
end;

Function TPasProcedure.IsMessage: Boolean;
begin
  Result:=pmMessage in FModifiers;
end;

Function TPasProcedure.IsReintroduced : Boolean;
begin
  Result:=pmReintroduce in FModifiers;
end;

Function TPasProcedure.IsStatic : Boolean;

begin
  Result:=pmStatic in FModifiers;
end;

function TPasProcedure.IsForward: Boolean;
begin
  Result:=pmForward in FModifiers;
end;

function TPasProcedure.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;
begin
  S:=TStringList.Create;
  try
    If Full then
      S.Add(TypeName+' '+Name);
    ProcType.GetArguments(S);
    GetModifiers(S);
    Result:=IndentStrings(S,Length(S[0]));
  finally
    S.Free;
  end;
end;

function TPasFunction.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;
  T : string;

begin
  S:=TStringList.Create;
  try
    If Full then
      S.Add(TypeName+' '+Name);
    ProcType.GetArguments(S);
    If Assigned((Proctype as TPasFunctionType).ResultEl) then
      With TPasFunctionType(ProcType).ResultEl.ResultType do
        begin
        T:=' : ';
        If (Name<>'') then
          T:=T+Name
        else
          T:=T+GetDeclaration(False);
        S.Add(T);
        end;
    GetModifiers(S);
    Result:=IndentStrings(S,Length(S[0]));
  finally
    S.Free;
  end;
end;

function TPasFunction.TypeName: string;
begin
  Result:='function';
end;

function TPasOperator.GetOperatorDeclaration(Full : Boolean) : string;

begin
  if Full then
    begin
    Result:=FullPath;
    if (Result<>'') then
      Result:=Result+'.';
    end
  else
    Result:='';
  if TokenBased then
    Result:=Result+TypeName+' '+OperatorTypeToToken(OperatorType)
  else
    Result:=Result+TypeName+' '+OperatorTypeToOperatorName(OperatorType);
end;

function TPasOperator.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;
  T : string;

begin
  S:=TStringList.Create;
  try
    If Full then
      S.Add(GetOperatorDeclaration(Full));
    ProcType.GetArguments(S);
    If Assigned((Proctype as TPasFunctionType).ResultEl) then
      With TPasFunctionType(ProcType).ResultEl.ResultType do
        begin
        T:=' : ';
        If (Name<>'') then
          T:=T+Name
        else
          T:=T+GetDeclaration(False);
        S.Add(T);
        end;
    GetModifiers(S);
    Result:=IndentStrings(S,Length(S[0]));

  finally
    S.Free;
  end;
end;

function TPasOperator.TypeName: string;
begin
  Result:='operator';
end;

function TPasClassProcedure.TypeName: string;
begin
  Result:='class procedure';
end;

function TPasClassFunction.TypeName: string;
begin
  Result:='class function';
end;

function TPasConstructor.TypeName: string;
begin
  Result:='constructor';
end;

function TPasDestructor.TypeName: string;
begin
  Result:='destructor';
end;

function TPasArgument.GetDeclaration (full : boolean) : string;
begin
  If Assigned(ArgType) then
    begin
    If ArgType.Name<>'' then
      Result:=ArgType.Name
    else
      Result:=ArgType.GetDeclaration(False);
    If Full then
      Result:=Name+': '+Result;
    end
  else If Full then
    Result:=Name
  else
    Result:='';
end;

function TPasArgument.Value: String;
begin
  If Assigned(ValueExpr) then
    Result:=ValueExpr.GetDeclaration(true)
  else
    Result:='';
end;



{ TPassTreeVisitor }

procedure TPassTreeVisitor.Visit(obj: TPasElement);
begin
  // Needs to be implemented by descendents.
end;

{ TPasSection }

constructor TPasSection.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  UsesList := TFPList.Create;
end;

destructor TPasSection.Destroy;
var
  i: Integer;
begin
  for i := 0 to UsesList.Count - 1 do
    TPasType(UsesList[i]).Release;
  UsesList.Free;

  inherited Destroy;
end;

procedure TPasSection.AddUnitToUsesList(const AUnitName: string);
begin
  UsesList.Add(TPasUnresolvedTypeRef.Create(AUnitName, Self));
end;

{ TProcedureBody }

constructor TProcedureBody.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Labels:=TFPList.Create;
end;

destructor TProcedureBody.Destroy;
begin
  FreeAndNil(Labels);
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

{ TPasImplWhileDo }

destructor TPasImplWhileDo.Destroy;
begin
  FreeAndNil(ConditionExpr);
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplWhileDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    begin
    Body:=Element;
    Body.AddRef;
    end
  else
    raise Exception.Create('TPasImplWhileDo.AddElement body already set - please report this bug');
end;

function TPasImplWhileDo.Condition: string;
begin
  If Assigned(ConditionExpr) then
    Result:=ConditionExpr.GetDeclaration(True);
end;

{ TPasImplCaseOf }

destructor TPasImplCaseOf.Destroy;
begin
  FreeAndNil(CaseExpr);
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

procedure TPasImplCaseOf.AddElement(Element: TPasImplElement);
begin
  if (ElseBranch<>Nil) and (Element=ElseBranch) then
    ElseBranch.AddRef;
  inherited AddElement(Element);
end;

function TPasImplCaseOf.AddCase(const Expression: TPasExpr
  ): TPasImplCaseStatement;
begin
  Result:=TPasImplCaseStatement.Create('',Self);
  Result.AddExpression(Expression);
  AddElement(Result);
end;

function TPasImplCaseOf.AddElse: TPasImplCaseElse;
begin
  Result:=TPasImplCaseElse.Create('',Self);
  ElseBranch:=Result;
  AddElement(Result);
end;

function TPasImplCaseOf.Expression: string;
begin
  if Assigned(CaseExpr) then
    Result:=CaseExpr.GetDeclaration(True)
  else
    Result:='';
end;

{ TPasImplCaseStatement }

constructor TPasImplCaseStatement.Create(const AName: string;
  AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Expressions:=TFPList.Create;
end;

destructor TPasImplCaseStatement.Destroy;

Var
  I : integer;

begin
  For I:=0 to Expressions.Count-1 do
    TPasExpr(Expressions[i]).Free;
  FreeAndNil(Expressions);
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplCaseStatement.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    begin
    Body:=Element;
    Body.AddRef;
    end
end;

procedure TPasImplCaseStatement.AddExpression(const Expr: TPasExpr);
begin
  Expressions.Add(Expr);
end;

{ TPasImplWithDo }

constructor TPasImplWithDo.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Expressions:=TFPList.Create;
end;

destructor TPasImplWithDo.Destroy;
Var
  I : Integer;
begin
  if Assigned(Body) then
    Body.Release;
  For I:=0 to Expressions.Count-1 do
    TObject(Expressions[i]).Free;
  FreeAndNil(Expressions);
  inherited Destroy;
end;

procedure TPasImplWithDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    begin
    Body:=Element;
    Body.AddRef;
    end;
end;

procedure TPasImplWithDo.AddExpression(const Expression: TPasExpr);
begin
  Expressions.Add(Expression);
end;

{ TPasImplTry }

destructor TPasImplTry.Destroy;
begin
  if Assigned(FinallyExcept) then
    FinallyExcept.Release;
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

function TPasImplTry.AddFinally: TPasImplTryFinally;
begin
  Result:=TPasImplTryFinally.Create('',Self);
  FinallyExcept:=Result;
end;

function TPasImplTry.AddExcept: TPasImplTryExcept;
begin
  Result:=TPasImplTryExcept.Create('',Self);
  FinallyExcept:=Result;
end;

function TPasImplTry.AddExceptElse: TPasImplTryExceptElse;
begin
  Result:=TPasImplTryExceptElse.Create('',Self);
  ElseBranch:=Result;
end;

{ TPasImplExceptOn }

destructor TPasImplExceptOn.Destroy;
begin
  FreeAndNil(VarExpr);
  FreeAndNil(TypeExpr);
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplExceptOn.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    begin
    Body:=Element;
    Body.AddRef;
    end;
end;

function TPasImplExceptOn.VariableName: String;
begin
  If assigned(VarExpr) then
    Result:=VarExpr.GetDeclaration(True)
  else
    Result:='';
end;

function TPasImplExceptOn.TypeName: string;
begin
  If assigned(TypeExpr) then
    Result:=TypeExpr.GetDeclaration(True)
  else
    Result:='';
end;

{ TPasImplStatement }

function TPasImplStatement.CloseOnSemicolon: boolean;
begin
  Result:=true;
end;

{ TPasExpr }

constructor TPasExpr.Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TexprOpcode);
begin
  inherited Create(ClassName, AParent);
  Kind:=AKind;
  OpCode:=AOpCode;
end;

{ TPrimitiveExpr }

function TPrimitiveExpr.GetDeclaration(Full : Boolean):AnsiString;
begin
  Result:=Value;
end;

constructor TPrimitiveExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : Ansistring);
begin
  inherited Create(AParent,AKind, eopNone);
  Value:=AValue;
end;

{ TBoolConstExpr }

constructor TBoolConstExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean);
begin
  inherited Create(AParent,AKind, eopNone);
  Value:=ABoolValue;
end;

Function TBoolConstExpr.GetDeclaration(Full: Boolean):AnsiString;

begin
  If Value then
    Result:='True'
  else
    Result:='False';  
end;



{ TUnaryExpr }

Function TUnaryExpr.GetDeclaration(Full : Boolean):AnsiString;

begin
  Result:=OpCodeStrings[Opcode];
  If Assigned(Operand) then
    Result:=Result+Operand.GetDeclaration(Full);
end;

constructor TUnaryExpr.Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode);
begin
  inherited Create(AParent,pekUnary, AOpCode);
  Operand:=AOperand;
end;

destructor TUnaryExpr.Destroy;
begin
  Operand.Free;
end;

{ TBinaryExpr }

function TBinaryExpr.GetDeclaration(Full : Boolean):AnsiString;
  function OpLevel(op: TPasExpr): Integer;
  begin
    case op.OpCode of
      eopNot,eopAddress:
        Result := 4;
      eopMultiply, eopDivide, eopDiv, eopMod, eopAnd, eopShl,
      eopShr, eopAs, eopPower:
        Result := 3;
      eopAdd, eopSubtract, eopOr, eopXor:
        Result := 2;
      eopEqual, eopNotEqual, eopLessThan, eopLessthanEqual, eopGreaterThan,
      eopGreaterThanEqual, eopIn, eopIs:
        Result := 1;
    else
      Result := 5; // Numbers and Identifiers
    end;
  end;
var op: string;
begin
  If Kind=pekRange then
    Result:='..'
  else
    Result:=' '+OpcodeStrings[Opcode]+' ';
  If Assigned(Left) then
  begin
    op := Left.GetDeclaration(Full);
    if OpLevel(Left) < OpLevel(Self) then
      Result := '(' + op + ')' + Result
    else
      Result := op + Result;
  end;
  If Assigned(Right) then
  begin
    op := Right.GetDeclaration(Full);
    if (OpLevel(Right) < 5) and (OpLevel(Right) >= OpLevel(Self)) then
      Result := Result + '(' + op + ')'
    else
      Result := Result + op;
  end;
end;


constructor TBinaryExpr.Create(AParent : TPasElement; xleft,xright:TPasExpr; AOpCode:TExprOpCode);
begin
  inherited Create(AParent,pekBinary, AOpCode);
  left:=xleft;
  right:=xright;
end;

constructor TBinaryExpr.CreateRange(AParent : TPasElement; xleft,xright:TPasExpr);
begin
  inherited Create(AParent,pekRange, eopNone);
  left:=xleft;
  right:=xright;
end;

destructor TBinaryExpr.Destroy;
begin
  left.Free;
  right.Free;
  inherited Destroy;
end;

{ TParamsExpr }

Function TParamsExpr.GetDeclaration(Full: Boolean) : Ansistring;

Var
  I : Integer;

begin
  Result := '';
  For I:=0 to High(Params) do
    begin
    If (Result<>'')  then
      Result:=Result+', ';
    Result:=Result+Params[I].GetDeclaration(Full);  
    end;
  if Kind = pekSet then
    Result := '[' + Result + ']'
  else
    Result := '(' + Result + ')';
end;

procedure TParamsExpr.AddParam(xp:TPasExpr);
var
  i : Integer;
begin
  i:=Length(Params);
  SetLength(Params, i+1);
  Params[i]:=xp;
end;

constructor TParamsExpr.Create(AParent : TPasElement; AKind: TPasExprKind);
begin
  inherited Create(AParent,AKind, eopNone)
end;

destructor TParamsExpr.Destroy;
var
  i : Integer;
begin
  FreeAndNil(Value);
  for i:=0 to length(Params)-1 do Params[i].Free;
  inherited Destroy;
end;

{ TRecordValues }

Function TRecordValues.GetDeclaration(Full : Boolean):AnsiString;

Var
  I : Integer;
begin
  Result := '';
  For I:=0 to High(Fields) do
    begin
    If Result<>'' then
      Result:=Result+'; ';
    Result:=Result+Fields[I].Name+': '+Fields[i].ValueExp.getDeclaration(Full);
    end;
<<<<<<< HEAD
  Result:='('+Result+')';
=======
  If S<>'' then
    S:='['+S+']'
  else
    S:=' ';
  If Full then
    begin
    Result:=Name+S+': '+Result;
    If (ImplementsName<>'') then
       Result:=Result+' implements '+ImplementsName;
    end;   
  If IsDefault then
    Result:=Result+'; default';
  ProcessHints(True, Result);
>>>>>>> graemeg/cpstrnew
end;

constructor TRecordValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone);
end;

destructor TRecordValues.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Fields)-1 do Fields[i].ValueExp.Free;
  inherited Destroy;
end;

procedure TRecordValues.AddField(const AName:AnsiString;Value:TPasExpr);
var
  i : Integer;
begin
  i:=length(Fields);
  SetLength(Fields, i+1);
  Fields[i].Name:=AName;
  Fields[i].ValueExp:=Value;
end;

{ TNilExpr }

Function TNilExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Nil';
end;

<<<<<<< HEAD
{ TInheritedExpr }
=======
Procedure TPasProcedure.AddModifier(AModifier : TProcedureModifier);

begin
  Include(FModifiers,AModifier);
end;

Function TPasProcedure.IsVirtual : Boolean;
begin
  Result:=pmVirtual in FModifiers;
end;

Function TPasProcedure.IsDynamic : Boolean;
begin
  Result:=pmDynamic in FModifiers;
end;

Function TPasProcedure.IsAbstract : Boolean;
begin
  Result:=pmAbstract in FModifiers;
end;

Function TPasProcedure.IsOverride : Boolean;
begin
  Result:=pmOverride in FModifiers;
end;

Function TPasProcedure.IsExported : Boolean;
begin
  Result:=pmExported in FModifiers;
end;

function TPasProcedure.IsExternal: Boolean;
begin
  Result:=pmExternal in FModifiers;
end;

Function TPasProcedure.IsOverload : Boolean;
begin
  Result:=pmOverload in FModifiers;
end;

Function TPasProcedure.IsMessage: Boolean;
begin
  Result:=pmMessage in FModifiers;
end;

Function TPasProcedure.IsReintroduced : Boolean;
begin
  Result:=pmReintroduce in FModifiers;
end;

Function TPasProcedure.IsStatic : Boolean;

begin
  Result:=pmStatic in FModifiers;
end;

function TPasProcedure.IsForward: Boolean;
begin
  Result:=pmForward in FModifiers;
end;

function TPasProcedure.GetDeclaration (full : boolean) : string;
>>>>>>> graemeg/cpstrnew

function TInheritedExpr.GetDeclaration(full: Boolean): string;
begin
  Result:='Inherited';
end;

{ TSelfExpr }

Function TSelfExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Self';
end;

{ TArrayValues }

Function TArrayValues.GetDeclaration(Full: Boolean):AnsiString;

Var
  I : Integer;

begin
  Result := '';
  For I:=0 to High(Values) do
    begin
    If Result<>'' then
      Result:=Result+', ';
    Result:=Result+Values[i].getDeclaration(Full);
    end;
  Result:='('+Result+')';
end;

<<<<<<< HEAD
constructor TArrayValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone)
end;
=======
function TPasFunction.TypeName: string;
begin
  Result:='function';
end;

function TPasOperator.GetDeclaration (full : boolean) : string;
>>>>>>> graemeg/cpstrnew

destructor TArrayValues.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Values)-1 do Values[i].Free;
  inherited Destroy;
end;

procedure TArrayValues.AddValues(AValue:TPasExpr);
var
  i : Integer;
begin
  i:=length(Values);
  SetLength(Values, i+1);
  Values[i]:=AValue;
end;

<<<<<<< HEAD
{ TNilExpr }
=======
function TPasOperator.TypeName: string;
begin
  Result:='operator';
end;

function TPasClassProcedure.TypeName: string;
begin
  Result:='class procedure';
end;

function TPasClassFunction.TypeName: string;
begin
  Result:='class function';
end;

function TPasConstructor.TypeName: string;
begin
  Result:='constructor';
end;

function TPasDestructor.TypeName: string;
begin
  Result:='destructor';
end;
>>>>>>> graemeg/cpstrnew

constructor TNilExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekNil, eopNone);
end;

{ TInheritedExpr }

constructor TInheritedExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekInherited, eopNone);
end;


{ TSelfExpr }

constructor TSelfExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekSelf, eopNone);
end;

{ TPasLabels }

constructor TPasLabels.Create(const AName:string;AParent:TPasElement);
begin
  inherited Create(AName,AParent);
  Labels := TStringList.Create;
end;

destructor TPasLabels.Destroy;
begin
  Labels.Free;
  inherited Destroy;
end;



{ TPassTreeVisitor }

procedure TPassTreeVisitor.Visit(obj: TPasElement);
begin
  // Needs to be implemented by descendents.
end;

{ TPasSection }

constructor TPasSection.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  UsesList := TList.Create;
end;

destructor TPasSection.Destroy;
var
  i: Integer;
begin
  for i := 0 to UsesList.Count - 1 do
    TPasType(UsesList[i]).Release;
  UsesList.Free;

  inherited Destroy;
end;

procedure TPasSection.AddUnitToUsesList(const AUnitName: string);
begin
  UsesList.Add(TPasUnresolvedTypeRef.Create(AUnitName, Self));
end;

{ TProcedureBody }

constructor TProcedureBody.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Labels:=TList.Create;
end;

destructor TProcedureBody.Destroy;
begin
  FreeAndNil(Labels);
  inherited Destroy;
end;

{ TPasImplWhileDo }

destructor TPasImplWhileDo.Destroy;
begin
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplWhileDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element
  else
    raise Exception.Create('TPasImplWhileDo.AddElement body already set - please report this bug');
end;

{ TPasImplCaseOf }

destructor TPasImplCaseOf.Destroy;
begin
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

procedure TPasImplCaseOf.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
end;

function TPasImplCaseOf.AddCase(const Expression: string
  ): TPasImplCaseStatement;
begin
  Result:=TPasImplCaseStatement.Create('',Self);
  Result.AddExpression(Expression);
  AddElement(Result);
end;

function TPasImplCaseOf.AddElse: TPasImplCaseElse;
begin
  Result:=TPasImplCaseElse.Create('',Self);
  ElseBranch:=Result;
  AddElement(Result);
end;

{ TPasImplCaseStatement }

constructor TPasImplCaseStatement.Create(const AName: string;
  AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Expressions:=TStringList.Create;
end;

destructor TPasImplCaseStatement.Destroy;
begin
  FreeAndNil(Expressions);
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplCaseStatement.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
end;

procedure TPasImplCaseStatement.AddExpression(const Expr: string);
begin
  Expressions.Add(Expr);
end;

{ TPasImplWithDo }

constructor TPasImplWithDo.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Expressions:=TStringList.Create;
end;

destructor TPasImplWithDo.Destroy;
begin
  if Assigned(Body) then
    Body.Release;
  FreeAndNil(Expressions);
  inherited Destroy;
end;

procedure TPasImplWithDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
end;

procedure TPasImplWithDo.AddExpression(const Expression: string);
begin
  Expressions.Add(Expression);
end;

{ TPasImplTry }

destructor TPasImplTry.Destroy;
begin
  if Assigned(FinallyExcept) then
    FinallyExcept.Release;
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

function TPasImplTry.AddFinally: TPasImplTryFinally;
begin
  Result:=TPasImplTryFinally.Create('',Self);
  FinallyExcept:=Result;
end;

function TPasImplTry.AddExcept: TPasImplTryExcept;
begin
  Result:=TPasImplTryExcept.Create('',Self);
  FinallyExcept:=Result;
end;

function TPasImplTry.AddExceptElse: TPasImplTryExceptElse;
begin
  Result:=TPasImplTryExceptElse.Create('',Self);
  ElseBranch:=Result;
end;

{ TPasImplExceptOn }

destructor TPasImplExceptOn.Destroy;
begin
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplExceptOn.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
end;

{ TPasImplStatement }

function TPasImplStatement.CloseOnSemicolon: boolean;
=======
  inherited Destroy;
end;

destructor TPasModule.Destroy;
>>>>>>> graemeg/cpstrnew
begin
  Result:=true;
end;

{ TPasExpr }

constructor TPasExpr.Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TexprOpcode);
begin
  Create(ClassName, AParent);
  Kind:=AKind;
  OpCode:=AOpCode;
end;

{ TPrimitiveExpr }

function TPrimitiveExpr.GetDeclaration(Full : Boolean):AnsiString;
begin
  Result:=Value;
end;

constructor TPrimitiveExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : Ansistring);
begin
  inherited Create(AParent,AKind, eopNone);
  Value:=AValue;
end;

{ TBoolConstExpr }

constructor TBoolConstExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean);
begin
  inherited Create(AParent,AKind, eopNone);
  Value:=ABoolValue;
end;

Function TBoolConstExpr.GetDeclaration(Full: Boolean):AnsiString;

begin
  If Value then
    Result:='True'
  else
    Result:='False';  
end;



{ TUnaryExpr }

Function TUnaryExpr.GetDeclaration(Full : Boolean):AnsiString;

begin
  Result:=OpCodeStrings[Opcode];
  If Assigned(Operand) then
    Result:=Result+Operand.GetDeclaration(Full);
end;

constructor TUnaryExpr.Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode);
begin
  inherited Create(AParent,pekUnary, AOpCode);
  Operand:=AOperand;
end;

destructor TUnaryExpr.Destroy;
begin
  Operand.Free;
end;

{ TBinaryExpr }

function TBinaryExpr.GetDeclaration(Full : Boolean):AnsiString;

begin
  If Kind=pekRange then
    Result:='..'
  else
    Result:=' '+OpcodeStrings[Opcode]+' ';
  If Assigned(Left) then
    Result:=Left.GetDeclaration(Full)+Result;
  If Assigned(Right) then
    Result:=Result +Right.GetDeclaration(Full);
end;


constructor TBinaryExpr.Create(AParent : TPasElement; xleft,xright:TPasExpr; AOpCode:TExprOpCode);
begin
  inherited Create(AParent,pekBinary, AOpCode);
  left:=xleft;
  right:=xright;
end;

constructor TBinaryExpr.CreateRange(AParent : TPasElement; xleft,xright:TPasExpr);
begin
  inherited Create(AParent,pekRange, eopNone);
  left:=xleft;
  right:=xright;
end;

destructor TBinaryExpr.Destroy;
begin
  left.Free;
  right.Free;
  inherited Destroy;
end;

{ TParamsExpr }

Function TParamsExpr.GetDeclaration(Full: Boolean) : Ansistring;

Var
  I : Integer;

begin
  For I:=0 to High(Params) do
    begin
    If (Result<>'')  then
      Result:=Result+', ';
    Result:=Result+Params[I].GetDeclaration(Full);  
    end;  
  Result:='('+Result+')';
end;

procedure TParamsExpr.AddParam(xp:TPasExpr);
var
  i : Integer;
begin
  i:=Length(Params);
  SetLength(Params, i+1);
  Params[i]:=xp;
end;

constructor TParamsExpr.Create(AParent : TPasElement; AKind: TPasExprKind);
begin
  inherited Create(AParent,AKind, eopNone)
end;

destructor TParamsExpr.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Params)-1 do Params[i].Free;
  inherited Destroy;
end;

{ TRecordValues }

Function TRecordValues.GetDeclaration(Full : Boolean):AnsiString;

Var
  I : Integer;
begin
<<<<<<< HEAD
  For I:=0 to High(Fields) do
    begin
    If Result='' then
      Result:=Result+'; ';
    Result:=Result+Fields[I].Name+': '+Fields[i].ValueExp.getDeclaration(Full);
    end;
<<<<<<< HEAD
  Result:='('+Result+')';
=======
  If S<>'' then
    S:='['+S+']'
  else
    S:=' ';
  If Full then
    begin
    Result:=Name+S+': '+Result;
    If (ImplementsName<>'') then
       Result:=Result+' implements '+ImplementsName;
    end;   
  If IsDefault then
    Result:=Result+'; default';
  ProcessHints(True, Result);
>>>>>>> graemeg/cpstrnew
=======
  inherited Create(AName, AParent);
  IsPacked := False;                     // 12/04/04 - Dave - Added
  IsShortDefinition := False;
  Members := TList.Create;
  Modifiers := TStringList.Create;
  ClassVars := TList.Create;
  Interfaces:= TList.Create;
>>>>>>> graemeg/cpstrnew
end;

constructor TRecordValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone);
end;

destructor TRecordValues.Destroy;
var
  i : Integer;
begin
<<<<<<< HEAD
  for i:=0 to length(Fields)-1 do Fields[i].ValueExp.Free;
=======
  for i := 0 to Members.Count - 1 do
    TPasElement(Members[i]).Release;
  Members.Free;
  if Assigned(AncestorType) then
    AncestorType.Release;
  Modifiers.Free;
  ClassVars.Free;
  Interfaces.Free;
>>>>>>> graemeg/cpstrnew
  inherited Destroy;
end;

procedure TRecordValues.AddField(const AName:AnsiString;Value:TPasExpr);
var
  i : Integer;
begin
  i:=length(Fields);
  SetLength(Fields, i+1);
  Fields[i].Name:=AName;
  Fields[i].ValueExp:=Value;
end;

{ TNilExpr }

Function TNilExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Nil';
end;

<<<<<<< HEAD
{ TInheritedExpr }
=======
Procedure TPasProcedure.AddModifier(AModifier : TProcedureModifier);

begin
  Include(FModifiers,AModifier);
end;

Function TPasProcedure.IsVirtual : Boolean;
begin
<<<<<<< HEAD
  Result:=pmVirtual in FModifiers;
=======
  for i := 0 to Args.Count - 1 do
    TPasArgument(Args[i]).Release;
  FreeAndNil(Args);
  inherited Destroy;
>>>>>>> graemeg/cpstrnew
end;

Function TPasProcedure.IsDynamic : Boolean;
begin
  Result:=pmDynamic in FModifiers;
end;

Function TPasProcedure.IsAbstract : Boolean;
begin
  Result:=pmAbstract in FModifiers;
end;

Function TPasProcedure.IsOverride : Boolean;
begin
  Result:=pmOverride in FModifiers;
end;

Function TPasProcedure.IsExported : Boolean;
begin
  Result:=pmExported in FModifiers;
end;

function TPasProcedure.IsExternal: Boolean;
begin
  Result:=pmExternal in FModifiers;
end;

Function TPasProcedure.IsOverload : Boolean;
begin
  Result:=pmOverload in FModifiers;
end;

Function TPasProcedure.IsMessage: Boolean;
begin
  Result:=pmMessage in FModifiers;
end;

Function TPasProcedure.IsReintroduced : Boolean;
begin
  Result:=pmReintroduce in FModifiers;
end;

Function TPasProcedure.IsStatic : Boolean;

begin
  Result:=pmStatic in FModifiers;
end;

function TPasProcedure.IsForward: Boolean;
begin
  Result:=pmForward in FModifiers;
end;

function TPasProcedure.GetDeclaration (full : boolean) : string;
>>>>>>> graemeg/cpstrnew

Function TInheritedExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
<<<<<<< HEAD
  Result:='Inherited';
=======
  { Attention, in derived classes, VarType isn't necessarily set!
    (e.g. in Constants) }
  if Assigned(VarType) then
    VarType.Release;
  if Assigned(Expr) then
    Expr.Release;
  inherited Destroy;
>>>>>>> graemeg/cpstrnew
end;

{ TSelfExpr }

Function TSelfExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Self';
end;

{ TArrayValues }

Function TArrayValues.GetDeclaration(Full: Boolean):AnsiString;

Var
  I : Integer;

begin
  For I:=0 to High(Values) do
    begin
    If Result='' then
      Result:=Result+', ';
    Result:=Result+Values[i].getDeclaration(Full);
    end;
  Result:='('+Result+')';
end;

<<<<<<< HEAD
constructor TArrayValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone)
end;
=======
function TPasFunction.TypeName: string;
begin
  Result:='function';
end;

function TPasOperator.GetDeclaration (full : boolean) : string;
>>>>>>> graemeg/cpstrnew

destructor TArrayValues.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Values)-1 do Values[i].Free;
  inherited Destroy;
end;

procedure TArrayValues.AddValues(AValue:TPasExpr);
var
  i : Integer;
begin
  i:=length(Values);
  SetLength(Values, i+1);
  Values[i]:=AValue;
end;

<<<<<<< HEAD
{ TNilExpr }
=======
function TPasOperator.TypeName: string;
begin
  Result:='operator';
end;

function TPasClassProcedure.TypeName: string;
begin
  Result:='class procedure';
end;

function TPasClassFunction.TypeName: string;
begin
  Result:='class function';
end;

function TPasConstructor.TypeName: string;
begin
  Result:='constructor';
end;

function TPasDestructor.TypeName: string;
begin
  Result:='destructor';
end;
>>>>>>> graemeg/cpstrnew

constructor TNilExpr.Create(AParent : TPasElement);
begin
<<<<<<< HEAD
  inherited Create(AParent,pekNil, eopNone);
end;

{ TInheritedExpr }

constructor TInheritedExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekInherited, eopNone);
end;

{ TSelfExpr }

constructor TSelfExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekSelf, eopNone);
=======
  if Assigned(ProcType) then
    ProcType.Release;
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
>>>>>>> graemeg/cpstrnew
end;

{ TPasLabels }

constructor TPasLabels.Create(const AName:string;AParent:TPasElement);
begin
<<<<<<< HEAD
  inherited Create(AName,AParent);
  Labels := TStringList.Create;
end;

destructor TPasLabels.Destroy;
begin
  Labels.Free;
  inherited Destroy;
end;



{ TPassTreeVisitor }

procedure TPassTreeVisitor.Visit(obj: TPasElement);
begin
  // Needs to be implemented by descendents.
end;

{ TPasSection }

constructor TPasSection.Create(const AName: string; AParent: TPasElement);
=======
  Result := 'procedure';
end;

constructor TPasProcedureImpl.Create(const AName: string; AParent: TPasElement);
>>>>>>> graemeg/cpstrnew
begin
  inherited Create(AName, AParent);
  UsesList := TList.Create;
end;

destructor TPasSection.Destroy;
var
  i: Integer;
begin
  for i := 0 to UsesList.Count - 1 do
    TPasType(UsesList[i]).Release;
  UsesList.Free;

  inherited Destroy;
end;

procedure TPasSection.AddUnitToUsesList(const AUnitName: string);
begin
  UsesList.Add(TPasUnresolvedTypeRef.Create(AUnitName, Self));
end;

{ TProcedureBody }

constructor TProcedureBody.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Labels:=TList.Create;
end;

destructor TProcedureBody.Destroy;
begin
  FreeAndNil(Labels);
  inherited Destroy;
end;

{ TPasImplWhileDo }

destructor TPasImplWhileDo.Destroy;
begin
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplWhileDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element
  else
    raise Exception.Create('TPasImplWhileDo.AddElement body already set - please report this bug');
end;

{ TPasImplCaseOf }

destructor TPasImplCaseOf.Destroy;
begin
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

<<<<<<< HEAD
procedure TPasImplCaseOf.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
=======
procedure TPasImplIfElse.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if IfBranch=nil then
    IfBranch:=Element
  else if ElseBranch=nil then
    ElseBranch:=Element
  else
    raise Exception.Create('TPasImplIfElse.AddElement if and else already set - please report this bug');
end;

function TPasImplIfElse.CloseOnSemicolon: boolean;
begin
  Result:=ElseBranch<>nil;
>>>>>>> graemeg/cpstrnew
end;

function TPasImplCaseOf.AddCase(const Expression: string
  ): TPasImplCaseStatement;
begin
  Result:=TPasImplCaseStatement.Create('',Self);
  Result.AddExpression(Expression);
  AddElement(Result);
end;

function TPasImplCaseOf.AddElse: TPasImplCaseElse;
begin
  Result:=TPasImplCaseElse.Create('',Self);
  ElseBranch:=Result;
  AddElement(Result);
end;

<<<<<<< HEAD
{ TPasImplCaseStatement }
=======
procedure TPasImplForLoop.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element
  else
    raise Exception.Create('TPasImplForLoop.AddElement body already set - please report this bug');
end;
>>>>>>> graemeg/cpstrnew

constructor TPasImplCaseStatement.Create(const AName: string;
  AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Expressions:=TStringList.Create;
end;

destructor TPasImplCaseStatement.Destroy;
begin
  FreeAndNil(Expressions);
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

<<<<<<< HEAD
procedure TPasImplCaseStatement.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
=======
procedure TPasImplBlock.AddElement(Element: TPasImplElement);
begin
  Elements.Add(Element);
end;

function TPasImplBlock.AddCommand(const ACommand: string): TPasImplCommand;
begin
  Result := TPasImplCommand.Create('', Self);
  Result.Command := ACommand;
  AddElement(Result);
>>>>>>> graemeg/cpstrnew
end;

procedure TPasImplCaseStatement.AddExpression(const Expr: string);
begin
<<<<<<< HEAD
  Expressions.Add(Expr);
=======
  Result := TPasImplCommands.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddBeginBlock: TPasImplBeginBlock;
begin
  Result := TPasImplBeginBlock.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddRepeatUntil: TPasImplRepeatUntil;
begin
  Result := TPasImplRepeatUntil.Create('', Self);
  AddElement(Result);
>>>>>>> graemeg/cpstrnew
end;

{ TPasImplWithDo }

constructor TPasImplWithDo.Create(const AName: string; AParent: TPasElement);
begin
<<<<<<< HEAD
  inherited Create(AName, AParent);
  Expressions:=TStringList.Create;
=======
  Result := TPasImplIfElse.Create('', Self);
  Result.Condition := ACondition;
  AddElement(Result);
end;

function TPasImplBlock.AddWhileDo(const ACondition: string): TPasImplWhileDo;
begin
  Result := TPasImplWhileDo.Create('', Self);
  Result.Condition := ACondition;
  AddElement(Result);
end;

function TPasImplBlock.AddWithDo(const Expression: string): TPasImplWithDo;
begin
  Result := TPasImplWithDo.Create('', Self);
  Result.AddExpression(Expression);
  AddElement(Result);
end;

function TPasImplBlock.AddCaseOf(const Expression: string): TPasImplCaseOf;
begin
  Result := TPasImplCaseOf.Create('', Self);
  Result.Expression := Expression;
  AddElement(Result);
>>>>>>> graemeg/cpstrnew
end;

destructor TPasImplWithDo.Destroy;
begin
<<<<<<< HEAD
  if Assigned(Body) then
    Body.Release;
  FreeAndNil(Expressions);
  inherited Destroy;
=======
  Result := TPasImplForLoop.Create('', Self);
  Result.Variable := AVar;
  Result.StartValue := AStartValue;
  Result.EndValue := AEndValue;
  AddElement(Result);
end;

function TPasImplBlock.AddForLoop(const AVarName, AStartValue,
  AEndValue: string; ADownTo: Boolean): TPasImplForLoop;
begin
  Result := TPasImplForLoop.Create('', Self);
  Result.VariableName := AVarName;
  Result.StartValue := AStartValue;
  Result.EndValue := AEndValue;
  Result.Down := ADownTo;
  AddElement(Result);
end;

function TPasImplBlock.AddTry: TPasImplTry;
begin
  Result := TPasImplTry.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddExceptOn(const VarName, TypeName: string
  ): TPasImplExceptOn;
begin
  Result:=TPasImplExceptOn.Create('',Self);
  Result.VariableName:=VarName;
  Result.TypeName:=TypeName;
  AddElement(Result);
end;

function TPasImplBlock.AddRaise: TPasImplRaise;
begin
  Result:=TPasImplRaise.Create('',Self);
  AddElement(Result);
end;

function TPasImplBlock.AddLabelMark(const Id: string): TPasImplLabelMark;
begin
  Result:=TPasImplLabelMark.Create('', Self);
  Result.LabelId:=Id;
  AddElement(Result);
end;

function TPasImplBlock.AddAssign(left,right:TPasExpr):TPasImplAssign;
begin
  Result:=TPasImplAssign.Create('', Self);
  Result.left:=left;
  Result.right:=right;
  AddElement(Result);
end;

function TPasImplBlock.AddSimple(exp:TPasExpr):TPasImplSimple;
begin
  Result:=TPasImplSimple.Create('', Self);
  Result.expr:=exp;
  AddElement(Result);
end;

function TPasImplBlock.CloseOnSemicolon: boolean;
begin
  Result:=false;
>>>>>>> graemeg/cpstrnew
end;

procedure TPasImplWithDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
end;

procedure TPasImplWithDo.AddExpression(const Expression: string);
begin
  Expressions.Add(Expression);
end;

{ TPasImplTry }

destructor TPasImplTry.Destroy;
begin
  if Assigned(FinallyExcept) then
    FinallyExcept.Release;
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

function TPasImplTry.AddFinally: TPasImplTryFinally;
begin
  Result:=TPasImplTryFinally.Create('',Self);
  FinallyExcept:=Result;
end;

function TPasImplTry.AddExcept: TPasImplTryExcept;
begin
  Result:=TPasImplTryExcept.Create('',Self);
  FinallyExcept:=Result;
end;

function TPasImplTry.AddExceptElse: TPasImplTryExceptElse;
begin
  Result:=TPasImplTryExceptElse.Create('',Self);
  ElseBranch:=Result;
end;

{ TPasImplExceptOn }

destructor TPasImplExceptOn.Destroy;
begin
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplExceptOn.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
end;

{ TPasImplStatement }

function TPasImplStatement.CloseOnSemicolon: boolean;
begin
  Result:=true;
end;

{ TPasExpr }

constructor TPasExpr.Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TexprOpcode);
begin
  Create(ClassName, AParent);
  Kind:=AKind;
  OpCode:=AOpCode;
end;

{ TPrimitiveExpr }

function TPrimitiveExpr.GetDeclaration(Full : Boolean):AnsiString;
begin
  Result:=Value;
end;

constructor TPrimitiveExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : Ansistring);
begin
  inherited Create(AParent,AKind, eopNone);
  Value:=AValue;
end;

{ TBoolConstExpr }

constructor TBoolConstExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean);
begin
  inherited Create(AParent,AKind, eopNone);
  Value:=ABoolValue;
end;

Function TBoolConstExpr.GetDeclaration(Full: Boolean):AnsiString;

begin
  If Value then
    Result:='True'
  else
    Result:='False';  
end;



{ TUnaryExpr }

Function TUnaryExpr.GetDeclaration(Full : Boolean):AnsiString;

begin
  Result:=OpCodeStrings[Opcode];
  If Assigned(Operand) then
    Result:=Result+Operand.GetDeclaration(Full);
end;

constructor TUnaryExpr.Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode);
begin
  inherited Create(AParent,pekUnary, AOpCode);
  Operand:=AOperand;
end;

destructor TUnaryExpr.Destroy;
begin
  Operand.Free;
end;

{ TBinaryExpr }

function TBinaryExpr.GetDeclaration(Full : Boolean):AnsiString;

begin
<<<<<<< HEAD
  If Kind=pekRange then
    Result:='..'
  else
    Result:=' '+OpcodeStrings[Opcode]+' ';
  If Assigned(Left) then
    Result:=Left.GetDeclaration(Full)+Result;
  If Assigned(Right) then
    Result:=Result +Right.GetDeclaration(Full);
=======
  S:=TStringList.Create;
  T:=TStringList.Create;
  Try
    Temp:='record';
    If IsPacked then
      if IsBitPacked then
        Temp:='bitpacked '+Temp
      else
        Temp:='packed '+Temp;
    If Full then
      Temp:=Name+' = '+Temp;
    S.Add(Temp);
    For I:=0 to Members.Count-1 do
      begin
      Temp:=TPasVariable(Members[i]).GetDeclaration(True);
      If Pos(LineEnding,Temp)>0 then
        begin
        T.Text:=Temp;
        For J:=0 to T.Count-1 do
          if J=T.Count-1 then
            S.Add('  '+T[J]+';')
          else
            S.Add('  '+T[J])
        end
      else
        S.Add('  '+Temp+';');
      end;
    S.Add('end');
    Result:=S.Text;
    ProcessHints(False, Result);
  finally
    S.free;
    T.free;
  end;
>>>>>>> graemeg/cpstrnew
end;


constructor TBinaryExpr.Create(AParent : TPasElement; xleft,xright:TPasExpr; AOpCode:TExprOpCode);
begin
  inherited Create(AParent,pekBinary, AOpCode);
  left:=xleft;
  right:=xright;
end;

constructor TBinaryExpr.CreateRange(AParent : TPasElement; xleft,xright:TPasExpr);
begin
  inherited Create(AParent,pekRange, eopNone);
  left:=xleft;
  right:=xright;
end;

destructor TBinaryExpr.Destroy;
begin
  left.Free;
  right.Free;
  inherited Destroy;
end;

{ TParamsExpr }

Function TParamsExpr.GetDeclaration(Full: Boolean) : Ansistring;

Var
  I : Integer;

begin
<<<<<<< HEAD
  For I:=0 to High(Params) do
    begin
    If (Result<>'')  then
      Result:=Result+', ';
    Result:=Result+Params[I].GetDeclaration(Full);  
    end;  
  Result:='('+Result+')';
=======
  inherited Create(AName, AParent);
  IsPacked := False;                     // 12/04/04 - Dave - Added
  IsShortDefinition := False;
  Members := TList.Create;
  Modifiers := TStringList.Create;
  ClassVars := TList.Create;
  Interfaces:= TList.Create;
>>>>>>> origin/cpstrnew
end;

procedure TParamsExpr.AddParam(xp:TPasExpr);
var
  i : Integer;
begin
<<<<<<< HEAD
  i:=Length(Params);
  SetLength(Params, i+1);
  Params[i]:=xp;
=======
  for i := 0 to Members.Count - 1 do
    TPasElement(Members[i]).Release;
  Members.Free;
  if Assigned(AncestorType) then
    AncestorType.Release;
  Modifiers.Free;
  ClassVars.Free;
  Interfaces.Free;
  inherited Destroy;
>>>>>>> origin/cpstrnew
end;

constructor TParamsExpr.Create(AParent : TPasElement; AKind: TPasExprKind);
begin
  inherited Create(AParent,AKind, eopNone)
end;

destructor TParamsExpr.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Params)-1 do Params[i].Free;
  inherited Destroy;
end;

{ TRecordValues }

Function TRecordValues.GetDeclaration(Full : Boolean):AnsiString;

Var
  I : Integer;
begin
  For I:=0 to High(Fields) do
    begin
    If Result='' then
      Result:=Result+'; ';
    Result:=Result+Fields[I].Name+': '+Fields[i].ValueExp.getDeclaration(Full);
    end;
  Result:='('+Result+')';
end;

constructor TRecordValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone);
end;

destructor TRecordValues.Destroy;
var
  i : Integer;
begin
<<<<<<< HEAD
  for i:=0 to length(Fields)-1 do Fields[i].ValueExp.Free;
=======
  for i := 0 to Args.Count - 1 do
    TPasArgument(Args[i]).Release;
  FreeAndNil(Args);
>>>>>>> origin/cpstrnew
  inherited Destroy;
end;

procedure TRecordValues.AddField(const AName:AnsiString;Value:TPasExpr);
var
  i : Integer;
begin
  i:=length(Fields);
  SetLength(Fields, i+1);
  Fields[i].Name:=AName;
  Fields[i].ValueExp:=Value;
end;

{ TNilExpr }

Function TNilExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Nil';
end;

{ TInheritedExpr }

Function TInheritedExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Inherited';
end;

{ TSelfExpr }

Function TSelfExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Self';
end;

{ TArrayValues }

Function TArrayValues.GetDeclaration(Full: Boolean):AnsiString;

Var
  I : Integer;

begin
  For I:=0 to High(Values) do
    begin
    If Result='' then
      Result:=Result+', ';
    Result:=Result+Values[i].getDeclaration(Full);
    end;
<<<<<<< HEAD
  Result:='('+Result+')';
=======
  If S<>'' then
    S:='['+S+']'
  else
    S:=' ';
  If Full then
    begin
    Result:=Name+S+': '+Result;
    If (ImplementsName<>'') then
       Result:=Result+' implements '+ImplementsName;
    end;   
  If IsDefault then
    Result:=Result+'; default';
  ProcessHints(True, Result);
>>>>>>> graemeg/cpstrnew
end;

constructor TArrayValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone)
end;

destructor TArrayValues.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Values)-1 do Values[i].Free;
  inherited Destroy;
end;

procedure TArrayValues.AddValues(AValue:TPasExpr);
var
  i : Integer;
begin
  i:=length(Values);
  SetLength(Values, i+1);
  Values[i]:=AValue;
end;

<<<<<<< HEAD
{ TNilExpr }
=======
Procedure TPasProcedure.AddModifier(AModifier : TProcedureModifier);

begin
  Include(FModifiers,AModifier);
end;

Function TPasProcedure.IsVirtual : Boolean;
begin
  Result:=pmVirtual in FModifiers;
end;

Function TPasProcedure.IsDynamic : Boolean;
begin
  Result:=pmDynamic in FModifiers;
end;

Function TPasProcedure.IsAbstract : Boolean;
begin
  Result:=pmAbstract in FModifiers;
end;

Function TPasProcedure.IsOverride : Boolean;
begin
<<<<<<< HEAD
  Result:=pmOverride in FModifiers;
end;

Function TPasProcedure.IsExported : Boolean;
begin
  Result:=pmExported in FModifiers;
=======
  { Attention, in derived classes, VarType isn't necessarily set!
    (e.g. in Constants) }
  if Assigned(VarType) then
    VarType.Release;
  if Assigned(Expr) then
    Expr.Release;
  inherited Destroy;
>>>>>>> origin/cpstrnew
end;

function TPasProcedure.IsExternal: Boolean;
begin
  Result:=pmExternal in FModifiers;
end;

Function TPasProcedure.IsOverload : Boolean;
begin
  Result:=pmOverload in FModifiers;
end;

Function TPasProcedure.IsMessage: Boolean;
begin
  Result:=pmMessage in FModifiers;
end;

Function TPasProcedure.IsReintroduced : Boolean;
begin
  Result:=pmReintroduce in FModifiers;
end;

Function TPasProcedure.IsStatic : Boolean;

begin
  Result:=pmStatic in FModifiers;
end;

function TPasProcedure.IsForward: Boolean;
begin
  Result:=pmForward in FModifiers;
end;

function TPasProcedure.GetDeclaration (full : boolean) : string;
>>>>>>> graemeg/cpstrnew

constructor TNilExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekNil, eopNone);
end;

{ TInheritedExpr }

constructor TInheritedExpr.Create(AParent : TPasElement);
begin
<<<<<<< HEAD
  inherited Create(AParent,pekInherited, eopNone);
end;

<<<<<<< HEAD
{ TSelfExpr }
=======
function TPasFunction.TypeName: string;
begin
  Result:='function';
end;

function TPasOperator.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;
  T : string;
>>>>>>> graemeg/cpstrnew

constructor TSelfExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekSelf, eopNone);
end;

<<<<<<< HEAD
{ TPasLabels }
=======
function TPasOperator.TypeName: string;
begin
  Result:='operator';
end;

function TPasClassProcedure.TypeName: string;
begin
  Result:='class procedure';
end;

function TPasClassFunction.TypeName: string;
begin
  Result:='class function';
end;

function TPasConstructor.TypeName: string;
begin
  Result:='constructor';
end;

function TPasDestructor.TypeName: string;
begin
  Result:='destructor';
=======
  if Assigned(ProcType) then
    ProcType.Release;
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
>>>>>>> origin/cpstrnew
end;
>>>>>>> graemeg/cpstrnew

constructor TPasLabels.Create(const AName:string;AParent:TPasElement);
begin
<<<<<<< HEAD
  inherited Create(AName,AParent);
  Labels := TStringList.Create;
end;

destructor TPasLabels.Destroy;
begin
  Labels.Free;
  inherited Destroy;
end;



{ TPassTreeVisitor }

procedure TPassTreeVisitor.Visit(obj: TPasElement);
begin
  // Needs to be implemented by descendents.
end;

{ TPasSection }

constructor TPasSection.Create(const AName: string; AParent: TPasElement);
=======
  Result := 'procedure';
end;

constructor TPasProcedureImpl.Create(const AName: string; AParent: TPasElement);
>>>>>>> origin/cpstrnew
begin
  inherited Create(AName, AParent);
  UsesList := TList.Create;
end;

destructor TPasSection.Destroy;
var
  i: Integer;
begin
  for i := 0 to UsesList.Count - 1 do
    TPasType(UsesList[i]).Release;
  UsesList.Free;

  inherited Destroy;
end;

procedure TPasSection.AddUnitToUsesList(const AUnitName: string);
begin
  UsesList.Add(TPasUnresolvedTypeRef.Create(AUnitName, Self));
end;

{ TProcedureBody }

constructor TProcedureBody.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Labels:=TList.Create;
end;

<<<<<<< HEAD
destructor TProcedureBody.Destroy;
begin
  FreeAndNil(Labels);
  inherited Destroy;
end;
=======
{$IFNDEF FPC}
  const
    LineEnding = sLineBreak;
{$ENDIF}
>>>>>>> graemeg/fixes_2_2

{ TPasImplWhileDo }

destructor TPasImplWhileDo.Destroy;
begin
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplWhileDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element
  else
    raise Exception.Create('TPasImplWhileDo.AddElement body already set - please report this bug');
end;

{ TPasImplCaseOf }

destructor TPasImplCaseOf.Destroy;
begin
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

<<<<<<< HEAD
procedure TPasImplCaseOf.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
=======
procedure TPasImplIfElse.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if IfBranch=nil then
    IfBranch:=Element
  else if ElseBranch=nil then
    ElseBranch:=Element
  else
    raise Exception.Create('TPasImplIfElse.AddElement if and else already set - please report this bug');
end;

function TPasImplIfElse.CloseOnSemicolon: boolean;
begin
  Result:=ElseBranch<>nil;
>>>>>>> origin/cpstrnew
end;

function TPasImplCaseOf.AddCase(const Expression: string
  ): TPasImplCaseStatement;
begin
  Result:=TPasImplCaseStatement.Create('',Self);
  Result.AddExpression(Expression);
  AddElement(Result);
end;

function TPasImplCaseOf.AddElse: TPasImplCaseElse;
begin
  Result:=TPasImplCaseElse.Create('',Self);
  ElseBranch:=Result;
  AddElement(Result);
end;

<<<<<<< HEAD
{ TPasImplCaseStatement }
=======
procedure TPasImplForLoop.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element
  else
    raise Exception.Create('TPasImplForLoop.AddElement body already set - please report this bug');
end;
>>>>>>> origin/cpstrnew

constructor TPasImplCaseStatement.Create(const AName: string;
  AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Expressions:=TStringList.Create;
end;

destructor TPasImplCaseStatement.Destroy;
begin
  FreeAndNil(Expressions);
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

<<<<<<< HEAD
procedure TPasImplCaseStatement.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
=======
procedure TPasImplBlock.AddElement(Element: TPasImplElement);
begin
  Elements.Add(Element);
end;

function TPasImplBlock.AddCommand(const ACommand: string): TPasImplCommand;
begin
  Result := TPasImplCommand.Create('', Self);
  Result.Command := ACommand;
  AddElement(Result);
>>>>>>> origin/cpstrnew
end;

procedure TPasImplCaseStatement.AddExpression(const Expr: string);
begin
<<<<<<< HEAD
  Expressions.Add(Expr);
=======
  Result := TPasImplCommands.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddBeginBlock: TPasImplBeginBlock;
begin
  Result := TPasImplBeginBlock.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddRepeatUntil: TPasImplRepeatUntil;
begin
  Result := TPasImplRepeatUntil.Create('', Self);
  AddElement(Result);
>>>>>>> origin/cpstrnew
end;

{ TPasImplWithDo }

constructor TPasImplWithDo.Create(const AName: string; AParent: TPasElement);
begin
<<<<<<< HEAD
  inherited Create(AName, AParent);
<<<<<<< HEAD
  Expressions:=TStringList.Create;
=======
  Result := TPasImplIfElse.Create('', Self);
  Result.Condition := ACondition;
  AddElement(Result);
=======
  UsesList := TList.Create;
  Declarations := TList.Create;
  ResStrings := TList.Create;
  Types := TList.Create;
  Consts := TList.Create;
  Classes := TList.Create;
  Functions := TList.Create;
  Variables := TList.Create;
  Properties := TList.Create;
>>>>>>> graemeg/fixes_2_2
end;

function TPasImplBlock.AddWhileDo(const ACondition: string): TPasImplWhileDo;
begin
<<<<<<< HEAD
  Result := TPasImplWhileDo.Create('', Self);
  Result.Condition := ACondition;
  AddElement(Result);
end;

function TPasImplBlock.AddWithDo(const Expression: string): TPasImplWithDo;
begin
  Result := TPasImplWithDo.Create('', Self);
  Result.AddExpression(Expression);
  AddElement(Result);
end;
=======
  Variables.Free;
  Functions.Free;
  Classes.Free;
  Consts.Free;
  Types.Free;
  ResStrings.Free;
  Properties.Free;
  for i := 0 to Declarations.Count - 1 do
    TPasElement(Declarations[i]).Release;
  Declarations.Free;
>>>>>>> graemeg/fixes_2_2

function TPasImplBlock.AddCaseOf(const Expression: string): TPasImplCaseOf;
begin
  Result := TPasImplCaseOf.Create('', Self);
  Result.Expression := Expression;
  AddElement(Result);
>>>>>>> origin/cpstrnew
end;

destructor TPasImplWithDo.Destroy;
begin
<<<<<<< HEAD
  if Assigned(Body) then
    Body.Release;
  FreeAndNil(Expressions);
  inherited Destroy;
=======
  Result := TPasImplForLoop.Create('', Self);
  Result.Variable := AVar;
  Result.StartValue := AStartValue;
  Result.EndValue := AEndValue;
  AddElement(Result);
end;

function TPasImplBlock.AddForLoop(const AVarName, AStartValue,
  AEndValue: string; ADownTo: Boolean): TPasImplForLoop;
begin
  Result := TPasImplForLoop.Create('', Self);
  Result.VariableName := AVarName;
  Result.StartValue := AStartValue;
  Result.EndValue := AEndValue;
  Result.Down := ADownTo;
  AddElement(Result);
end;

function TPasImplBlock.AddTry: TPasImplTry;
begin
  Result := TPasImplTry.Create('', Self);
  AddElement(Result);
end;

function TPasImplBlock.AddExceptOn(const VarName, TypeName: string
  ): TPasImplExceptOn;
begin
  Result:=TPasImplExceptOn.Create('',Self);
  Result.VariableName:=VarName;
  Result.TypeName:=TypeName;
  AddElement(Result);
end;

function TPasImplBlock.AddRaise: TPasImplRaise;
begin
  Result:=TPasImplRaise.Create('',Self);
  AddElement(Result);
end;

function TPasImplBlock.AddLabelMark(const Id: string): TPasImplLabelMark;
begin
  Result:=TPasImplLabelMark.Create('', Self);
  Result.LabelId:=Id;
  AddElement(Result);
end;

function TPasImplBlock.AddAssign(left,right:TPasExpr):TPasImplAssign;
begin
  Result:=TPasImplAssign.Create('', Self);
  Result.left:=left;
  Result.right:=right;
  AddElement(Result);
end;

function TPasImplBlock.AddSimple(exp:TPasExpr):TPasImplSimple;
begin
  Result:=TPasImplSimple.Create('', Self);
  Result.expr:=exp;
  AddElement(Result);
end;

function TPasImplBlock.CloseOnSemicolon: boolean;
begin
  Result:=false;
>>>>>>> origin/cpstrnew
end;

procedure TPasImplWithDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
end;

procedure TPasImplWithDo.AddExpression(const Expression: string);
begin
  Expressions.Add(Expression);
end;

{ TPasImplTry }

destructor TPasImplTry.Destroy;
begin
  if Assigned(FinallyExcept) then
    FinallyExcept.Release;
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

function TPasImplTry.AddFinally: TPasImplTryFinally;
begin
  Result:=TPasImplTryFinally.Create('',Self);
  FinallyExcept:=Result;
end;

function TPasImplTry.AddExcept: TPasImplTryExcept;
begin
  Result:=TPasImplTryExcept.Create('',Self);
  FinallyExcept:=Result;
end;

function TPasImplTry.AddExceptElse: TPasImplTryExceptElse;
begin
  Result:=TPasImplTryExceptElse.Create('',Self);
  ElseBranch:=Result;
end;

{ TPasImplExceptOn }

destructor TPasImplExceptOn.Destroy;
begin
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplExceptOn.AddElement(Element: TPasImplElement);
begin
<<<<<<< HEAD
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
=======
  inherited Create(AName, AParent);
  UsesList := TList.Create;
  Declarations := TList.Create;
  ResStrings := TList.Create;
  Types := TList.Create;
  Consts := TList.Create;
  Classes := TList.Create;
  Functions := TList.Create;
  Variables := TList.Create;
  Properties := TList.Create;
>>>>>>> origin/fixes_2_2
end;

{ TPasImplStatement }

function TPasImplStatement.CloseOnSemicolon: boolean;
begin
<<<<<<< HEAD
  Result:=true;
end;

{ TPasExpr }
=======
  Variables.Free;
  Functions.Free;
  Classes.Free;
  Consts.Free;
  Types.Free;
  ResStrings.Free;
  Properties.Free;
  for i := 0 to Declarations.Count - 1 do
    TPasElement(Declarations[i]).Release;
  Declarations.Free;
>>>>>>> origin/fixes_2_2

constructor TPasExpr.Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TexprOpcode);
begin
  Create(ClassName, AParent);
  Kind:=AKind;
  OpCode:=AOpCode;
end;

{ TPrimitiveExpr }

function TPrimitiveExpr.GetDeclaration(Full : Boolean):AnsiString;
begin
  Result:=Value;
end;

constructor TPrimitiveExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : Ansistring);
begin
  inherited Create(AParent,AKind, eopNone);
  Value:=AValue;
end;

{ TBoolConstExpr }

constructor TBoolConstExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean);
begin
  inherited Create(AParent,AKind, eopNone);
  Value:=ABoolValue;
end;

Function TBoolConstExpr.GetDeclaration(Full: Boolean):AnsiString;

begin
  If Value then
    Result:='True'
  else
    Result:='False';  
end;



{ TUnaryExpr }

Function TUnaryExpr.GetDeclaration(Full : Boolean):AnsiString;

begin
  Result:=OpCodeStrings[Opcode];
  If Assigned(Operand) then
    Result:=Result+Operand.GetDeclaration(Full);
end;

constructor TUnaryExpr.Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode);
begin
  inherited Create(AParent,pekUnary, AOpCode);
  Operand:=AOperand;
end;

destructor TUnaryExpr.Destroy;
begin
  Operand.Free;
end;

{ TBinaryExpr }

function TBinaryExpr.GetDeclaration(Full : Boolean):AnsiString;

begin
<<<<<<< HEAD
  If Kind=pekRange then
    Result:='..'
  else
    Result:=' '+OpcodeStrings[Opcode]+' ';
  If Assigned(Left) then
    Result:=Left.GetDeclaration(Full)+Result;
  If Assigned(Right) then
    Result:=Result +Right.GetDeclaration(Full);
=======
  S:=TStringList.Create;
  T:=TStringList.Create;
  Try
    Temp:='record';
    If IsPacked then
      if IsBitPacked then
        Temp:='bitpacked '+Temp
      else
        Temp:='packed '+Temp;
    If Full then
      Temp:=Name+' = '+Temp;
    S.Add(Temp);
    For I:=0 to Members.Count-1 do
      begin
      Temp:=TPasVariable(Members[i]).GetDeclaration(True);
      If Pos(LineEnding,Temp)>0 then
        begin
        T.Text:=Temp;
        For J:=0 to T.Count-1 do
          if J=T.Count-1 then
            S.Add('  '+T[J]+';')
          else
            S.Add('  '+T[J])
        end
      else
        S.Add('  '+Temp+';');
      end;
    S.Add('end');
    Result:=S.Text;
    ProcessHints(False, Result);
  finally
    S.free;
    T.free;
  end;
>>>>>>> origin/cpstrnew
end;


constructor TBinaryExpr.Create(AParent : TPasElement; xleft,xright:TPasExpr; AOpCode:TExprOpCode);
begin
  inherited Create(AParent,pekBinary, AOpCode);
  left:=xleft;
  right:=xright;
end;

constructor TBinaryExpr.CreateRange(AParent : TPasElement; xleft,xright:TPasExpr);
begin
  inherited Create(AParent,pekRange, eopNone);
  left:=xleft;
  right:=xright;
end;

destructor TBinaryExpr.Destroy;
begin
  left.Free;
  right.Free;
  inherited Destroy;
end;

{ TParamsExpr }

Function TParamsExpr.GetDeclaration(Full: Boolean) : Ansistring;

Var
  I : Integer;

begin
  For I:=0 to High(Params) do
    begin
    If (Result<>'')  then
      Result:=Result+', ';
    Result:=Result+Params[I].GetDeclaration(Full);  
    end;  
  Result:='('+Result+')';
end;

procedure TParamsExpr.AddParam(xp:TPasExpr);
var
  i : Integer;
begin
  i:=Length(Params);
  SetLength(Params, i+1);
  Params[i]:=xp;
end;

constructor TParamsExpr.Create(AParent : TPasElement; AKind: TPasExprKind);
begin
  inherited Create(AParent,AKind, eopNone)
end;

destructor TParamsExpr.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Params)-1 do Params[i].Free;
  inherited Destroy;
end;

{ TRecordValues }

Function TRecordValues.GetDeclaration(Full : Boolean):AnsiString;

Var
  I : Integer;
begin
  For I:=0 to High(Fields) do
    begin
    If Result='' then
      Result:=Result+'; ';
    Result:=Result+Fields[I].Name+': '+Fields[i].ValueExp.getDeclaration(Full);
    end;
  Result:='('+Result+')';
end;

constructor TRecordValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone);
end;

destructor TRecordValues.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Fields)-1 do Fields[i].ValueExp.Free;
  inherited Destroy;
end;

procedure TRecordValues.AddField(const AName:AnsiString;Value:TPasExpr);
var
  i : Integer;
begin
  i:=length(Fields);
  SetLength(Fields, i+1);
  Fields[i].Name:=AName;
  Fields[i].ValueExp:=Value;
end;

{ TNilExpr }

Function TNilExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Nil';
end;

{ TInheritedExpr }

Function TInheritedExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Inherited';
end;

{ TSelfExpr }

Function TSelfExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Self';
end;

{ TArrayValues }

Function TArrayValues.GetDeclaration(Full: Boolean):AnsiString;

Var
  I : Integer;

begin
  For I:=0 to High(Values) do
    begin
    If Result='' then
      Result:=Result+', ';
    Result:=Result+Values[i].getDeclaration(Full);
    end;
<<<<<<< HEAD
  Result:='('+Result+')';
=======
  If S<>'' then
    S:='['+S+']'
  else
    S:=' ';
  If Full then
    begin
    Result:=Name+S+': '+Result;
    If (ImplementsName<>'') then
       Result:=Result+' implements '+ImplementsName;
    end;   
  If IsDefault then
    Result:=Result+'; default';
  ProcessHints(True, Result);
>>>>>>> origin/cpstrnew
end;

constructor TArrayValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone)
end;

destructor TArrayValues.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Values)-1 do Values[i].Free;
  inherited Destroy;
end;

procedure TArrayValues.AddValues(AValue:TPasExpr);
var
  i : Integer;
begin
  i:=length(Values);
  SetLength(Values, i+1);
  Values[i]:=AValue;
end;

<<<<<<< HEAD
{ TNilExpr }
=======
Procedure TPasProcedure.AddModifier(AModifier : TProcedureModifier);

begin
  Include(FModifiers,AModifier);
end;

Function TPasProcedure.IsVirtual : Boolean;
begin
  Result:=pmVirtual in FModifiers;
end;

Function TPasProcedure.IsDynamic : Boolean;
begin
  Result:=pmDynamic in FModifiers;
end;

Function TPasProcedure.IsAbstract : Boolean;
begin
  Result:=pmAbstract in FModifiers;
end;

Function TPasProcedure.IsOverride : Boolean;
begin
  Result:=pmOverride in FModifiers;
end;

Function TPasProcedure.IsExported : Boolean;
begin
  Result:=pmExported in FModifiers;
end;

function TPasProcedure.IsExternal: Boolean;
begin
  Result:=pmExternal in FModifiers;
end;

Function TPasProcedure.IsOverload : Boolean;
begin
  Result:=pmOverload in FModifiers;
end;

Function TPasProcedure.IsMessage: Boolean;
begin
  Result:=pmMessage in FModifiers;
end;

Function TPasProcedure.IsReintroduced : Boolean;
begin
  Result:=pmReintroduce in FModifiers;
end;

Function TPasProcedure.IsStatic : Boolean;

begin
  Result:=pmStatic in FModifiers;
end;

function TPasProcedure.IsForward: Boolean;
begin
  Result:=pmForward in FModifiers;
end;

function TPasProcedure.GetDeclaration (full : boolean) : string;
>>>>>>> origin/cpstrnew

constructor TNilExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekNil, eopNone);
end;

{ TInheritedExpr }

constructor TInheritedExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekInherited, eopNone);
end;

<<<<<<< HEAD
{ TSelfExpr }
=======
function TPasFunction.TypeName: string;
begin
  Result:='function';
end;

function TPasOperator.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;
  T : string;
>>>>>>> origin/cpstrnew

constructor TSelfExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekSelf, eopNone);
end;

<<<<<<< HEAD
{ TPasLabels }
=======
function TPasOperator.TypeName: string;
begin
  Result:='operator';
end;

function TPasClassProcedure.TypeName: string;
begin
  Result:='class procedure';
end;

function TPasClassFunction.TypeName: string;
begin
  Result:='class function';
end;

function TPasConstructor.TypeName: string;
begin
  Result:='constructor';
end;

function TPasDestructor.TypeName: string;
begin
  Result:='destructor';
end;
>>>>>>> origin/cpstrnew

constructor TPasLabels.Create(const AName:string;AParent:TPasElement);
begin
  inherited Create(AName,AParent);
  Labels := TStringList.Create;
end;

destructor TPasLabels.Destroy;
begin
  Labels.Free;
  inherited Destroy;
end;



{ TPassTreeVisitor }

procedure TPassTreeVisitor.Visit(obj: TPasElement);
begin
  // Needs to be implemented by descendents.
end;

{ TPasSection }

constructor TPasSection.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  UsesList := TList.Create;
end;

destructor TPasSection.Destroy;
var
  i: Integer;
begin
  for i := 0 to UsesList.Count - 1 do
    TPasType(UsesList[i]).Release;
  UsesList.Free;

  inherited Destroy;
end;

procedure TPasSection.AddUnitToUsesList(const AUnitName: string);
begin
  UsesList.Add(TPasUnresolvedTypeRef.Create(AUnitName, Self));
end;

{ TProcedureBody }

constructor TProcedureBody.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Labels:=TList.Create;
end;

destructor TProcedureBody.Destroy;
begin
  FreeAndNil(Labels);
  inherited Destroy;
end;

{ TPasImplWhileDo }

destructor TPasImplWhileDo.Destroy;
begin
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplWhileDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element
  else
    raise Exception.Create('TPasImplWhileDo.AddElement body already set - please report this bug');
end;

{ TPasImplCaseOf }

destructor TPasImplCaseOf.Destroy;
begin
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

procedure TPasImplCaseOf.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
end;

function TPasImplCaseOf.AddCase(const Expression: string
  ): TPasImplCaseStatement;
begin
  Result:=TPasImplCaseStatement.Create('',Self);
  Result.AddExpression(Expression);
  AddElement(Result);
end;

function TPasImplCaseOf.AddElse: TPasImplCaseElse;
begin
  Result:=TPasImplCaseElse.Create('',Self);
  ElseBranch:=Result;
  AddElement(Result);
end;

{ TPasImplCaseStatement }

constructor TPasImplCaseStatement.Create(const AName: string;
  AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Expressions:=TStringList.Create;
end;

destructor TPasImplCaseStatement.Destroy;
begin
  FreeAndNil(Expressions);
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplCaseStatement.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
end;

procedure TPasImplCaseStatement.AddExpression(const Expr: string);
begin
  Expressions.Add(Expr);
end;

{ TPasImplWithDo }

constructor TPasImplWithDo.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Expressions:=TStringList.Create;
end;

destructor TPasImplWithDo.Destroy;
begin
  if Assigned(Body) then
    Body.Release;
  FreeAndNil(Expressions);
  inherited Destroy;
end;

procedure TPasImplWithDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
end;

procedure TPasImplWithDo.AddExpression(const Expression: string);
begin
  Expressions.Add(Expression);
end;

{ TPasImplTry }

destructor TPasImplTry.Destroy;
begin
  if Assigned(FinallyExcept) then
    FinallyExcept.Release;
  if Assigned(ElseBranch) then
    ElseBranch.Release;
  inherited Destroy;
end;

function TPasImplTry.AddFinally: TPasImplTryFinally;
begin
  Result:=TPasImplTryFinally.Create('',Self);
  FinallyExcept:=Result;
end;

function TPasImplTry.AddExcept: TPasImplTryExcept;
begin
  Result:=TPasImplTryExcept.Create('',Self);
  FinallyExcept:=Result;
end;

function TPasImplTry.AddExceptElse: TPasImplTryExceptElse;
begin
  Result:=TPasImplTryExceptElse.Create('',Self);
  ElseBranch:=Result;
end;

{ TPasImplExceptOn }

destructor TPasImplExceptOn.Destroy;
begin
  if Assigned(Body) then
    Body.Release;
  inherited Destroy;
end;

procedure TPasImplExceptOn.AddElement(Element: TPasImplElement);
begin
<<<<<<< HEAD
<<<<<<< HEAD
  inherited AddElement(Element);
  if Body=nil then
    Body:=Element;
=======
=======
>>>>>>> origin/fixes_2_2
  Result:='Array';
  If (IndexRange<>'') then
    Result:=Result+'['+IndexRange+']';
  Result:=Result+' of ';
  If IsPacked then
     Result := 'packed '+Result;      // 12/04/04 Dave - Added
  If Assigned(Eltype) then
    Result:=Result+ElType.Name
  else
    Result:=Result+'const';
  If Full Then
    Result:=Name+' = '+Result;
>>>>>>> graemeg/fixes_2_2
end;

{ TPasImplStatement }

function TPasImplStatement.CloseOnSemicolon: boolean;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  Result:=true;
=======
=======
>>>>>>> origin/fixes_2_2
  Result:='File';
  If Assigned(Eltype) then
    Result:=Result+' of '+ElType.Name;
  If Full Then
    Result:=Name+' = '+Result;
>>>>>>> graemeg/fixes_2_2
end;

{ TPasExpr }

constructor TPasExpr.Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TexprOpcode);
begin
  Create(ClassName, AParent);
  Kind:=AKind;
  OpCode:=AOpCode;
end;

{ TPrimitiveExpr }

function TPrimitiveExpr.GetDeclaration(Full : Boolean):AnsiString;
begin
  Result:=Value;
end;

constructor TPrimitiveExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : Ansistring);
begin
  inherited Create(AParent,AKind, eopNone);
  Value:=AValue;
end;

{ TBoolConstExpr }

constructor TBoolConstExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const ABoolValue : Boolean);
begin
  inherited Create(AParent,AKind, eopNone);
  Value:=ABoolValue;
end;

Function TBoolConstExpr.GetDeclaration(Full: Boolean):AnsiString;

begin
  If Value then
    Result:='True'
  else
    Result:='False';  
end;



{ TUnaryExpr }

Function TUnaryExpr.GetDeclaration(Full : Boolean):AnsiString;

begin
<<<<<<< HEAD
  Result:=OpCodeStrings[Opcode];
  If Assigned(Operand) then
    Result:=Result+Operand.GetDeclaration(Full);
end;

constructor TUnaryExpr.Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode);
begin
  inherited Create(AParent,pekUnary, AOpCode);
  Operand:=AOperand;
end;

destructor TUnaryExpr.Destroy;
begin
  Operand.Free;
end;

{ TBinaryExpr }

function TBinaryExpr.GetDeclaration(Full : Boolean):AnsiString;

begin
  If Kind=pekRange then
    Result:='..'
  else
    Result:=' '+OpcodeStrings[Opcode]+' ';
  If Assigned(Left) then
    Result:=Left.GetDeclaration(Full)+Result;
  If Assigned(Right) then
    Result:=Result +Right.GetDeclaration(Full);
=======
  S:=TStringList.Create;
  T:=TstringList.Create;
  Try
    Temp:='record';
    If IsPacked then
      if IsBitPacked then
        Temp:='bitpacked '+Temp
      else
        Temp:='packed '+Temp;
    If Full then
      Temp:=Name+' = '+Temp;
    S.Add(Temp);
    For I:=0 to Members.Count-1 do
      begin
      Temp:=TPasVariable(Members[i]).GetDeclaration(True);
      If Pos(LineEnding,Temp)>0 then
        begin
        T.Text:=Temp;
        For J:=0 to T.Count-1 do
          if J=T.Count-1 then
            S.Add('  '+T[J]+';')
          else
            S.Add('  '+T[J])
        end
      else
        S.Add('  '+Temp+';');
      end;
    S.Add('end');
    Result:=S.Text;
  finally
    S.free;
    T.free;
  end;
>>>>>>> graemeg/fixes_2_2
end;


constructor TBinaryExpr.Create(AParent : TPasElement; xleft,xright:TPasExpr; AOpCode:TExprOpCode);
begin
  inherited Create(AParent,pekBinary, AOpCode);
  left:=xleft;
  right:=xright;
end;

constructor TBinaryExpr.CreateRange(AParent : TPasElement; xleft,xright:TPasExpr);
begin
  inherited Create(AParent,pekRange, eopNone);
  left:=xleft;
  right:=xright;
end;

destructor TBinaryExpr.Destroy;
begin
  left.Free;
  right.Free;
  inherited Destroy;
end;

{ TParamsExpr }

Function TParamsExpr.GetDeclaration(Full: Boolean) : Ansistring;

Var
  I : Integer;

begin
  For I:=0 to High(Params) do
    begin
    If (Result<>'')  then
      Result:=Result+', ';
    Result:=Result+Params[I].GetDeclaration(Full);  
    end;  
  Result:='('+Result+')';
end;

procedure TParamsExpr.AddParam(xp:TPasExpr);
var
  i : Integer;
begin
  i:=Length(Params);
  SetLength(Params, i+1);
  Params[i]:=xp;
end;

constructor TParamsExpr.Create(AParent : TPasElement; AKind: TPasExprKind);
begin
  inherited Create(AParent,AKind, eopNone)
end;

destructor TParamsExpr.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Params)-1 do Params[i].Free;
  inherited Destroy;
end;

{ TRecordValues }

Function TRecordValues.GetDeclaration(Full : Boolean):AnsiString;

Var
  I : Integer;
begin
  For I:=0 to High(Fields) do
    begin
    If Result='' then
      Result:=Result+'; ';
    Result:=Result+Fields[I].Name+': '+Fields[i].ValueExp.getDeclaration(Full);
    end;
  Result:='('+Result+')';
end;

constructor TRecordValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone);
end;

destructor TRecordValues.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Fields)-1 do Fields[i].ValueExp.Free;
  inherited Destroy;
end;

procedure TRecordValues.AddField(const AName:AnsiString;Value:TPasExpr);
var
  i : Integer;
begin
  i:=length(Fields);
  SetLength(Fields, i+1);
  Fields[i].Name:=AName;
  Fields[i].ValueExp:=Value;
end;

{ TNilExpr }

Function TNilExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Nil';
end;

{ TInheritedExpr }

Function TInheritedExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Inherited';
end;

{ TSelfExpr }

Function TSelfExpr.GetDeclaration(Full :Boolean):AnsiString;
begin
  Result:='Self';
end;

{ TArrayValues }

Function TArrayValues.GetDeclaration(Full: Boolean):AnsiString;

Var
  I : Integer;

begin
  For I:=0 to High(Values) do
    begin
    If Result='' then
      Result:=Result+', ';
    Result:=Result+Values[i].getDeclaration(Full);
    end;
<<<<<<< HEAD
  Result:='('+Result+')';
=======
  If S<>'' then
    S:='['+S+']'
  else
    S:=' ';
  If Full then
    begin
    Result:=Name+S+': '+Result;
    If (ImplementsName<>'') then
       Result:=Result+' implements '+ImplementsName;
    end;   
  If IsDefault then
    Result:=Result+'; default'
>>>>>>> graemeg/fixes_2_2
end;

constructor TArrayValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone)
end;

destructor TArrayValues.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Values)-1 do Values[i].Free;
  inherited Destroy;
end;

procedure TArrayValues.AddValues(AValue:TPasExpr);
var
  i : Integer;
begin
  i:=length(Values);
  SetLength(Values, i+1);
  Values[i]:=AValue;
end;

{ TNilExpr }

constructor TNilExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekNil, eopNone);
end;

{ TInheritedExpr }

constructor TInheritedExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekInherited, eopNone);
end;

{ TSelfExpr }

constructor TSelfExpr.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekSelf, eopNone);
end;

{ TPasLabels }

constructor TPasLabels.Create(const AName:string;AParent:TPasElement);
begin
  inherited Create(AName,AParent);
  Labels := TStringList.Create;
end;

destructor TPasLabels.Destroy;
begin
  Labels.Free;
  inherited Destroy;
end;

end.
