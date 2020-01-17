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

{$if defined(debugrefcount) or defined(VerbosePasTreeMem) or defined(VerbosePasResolver)}
  {$define EnablePasTreeGlobalRefCount}
{$endif}

interface

uses Classes;

resourcestring
  // Parse tree node type names
  SPasTreeElement = 'generic element';
  SPasTreeSection = 'unit section';
  SPasTreeProgramSection = 'program section';
  SPasTreeLibrarySection = 'library section';
  SPasTreeInterfaceSection = 'interface section';
  SPasTreeImplementationSection = 'implementation section';
  SPasTreeUsesUnit = 'uses unit';
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
  SPasTreeSpecializedType = 'specialized class type';
  SPasTreeSpecializedExpr = 'specialize expr';
  SPasClassHelperType = 'class helper type';
  SPasRecordHelperType = 'record helper type';
  SPasTypeHelperType = 'type helper type';
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
  SPasTreeOperator = 'operator';
  SPasTreeClassOperator = 'class operator';
  SPasTreeClassProcedure = 'class procedure';
  SPasTreeClassFunction = 'class function';
  SPasTreeClassConstructor = 'class constructor';
  SPasTreeClassDestructor = 'class destructor';
  SPasTreeConstructor = 'constructor';
  SPasTreeDestructor = 'destructor';
  SPasTreeAnonymousProcedure = 'anonymous procedure';
  SPasTreeAnonymousFunction = 'anonymous function';
  SPasTreeProcedureImpl = 'procedure/function implementation';
  SPasTreeConstructorImpl = 'constructor implementation';
  SPasTreeDestructorImpl = 'destructor implementation';

type

  // Visitor pattern.
  TPassTreeVisitor = class;

  { TPasElementBase }

  TPasElementBase = class
  private
    FData: TObject;
  protected
    procedure Accept(Visitor: TPassTreeVisitor); virtual;
  public
    Property CustomData: TObject Read FData Write FData;
  end;
  TPasElementBaseClass = class of TPasElementBase;


  TPasModule = class;

  TPasMemberVisibility = (visDefault, visPrivate, visProtected, visPublic,
    visPublished, visAutomated,
    visStrictPrivate, visStrictProtected);

  TCallingConvention = (ccDefault,ccRegister,ccPascal,ccCDecl,ccStdCall,
                        ccOldFPCCall,ccSafeCall,ccSysCall);
  TProcTypeModifier = (ptmOfObject,ptmIsNested,ptmStatic,ptmVarargs,ptmReferenceTo);
  TProcTypeModifiers = set of TProcTypeModifier;
  TPackMode = (pmNone,pmPacked,pmBitPacked);

  TPasMemberVisibilities = set of TPasMemberVisibility;
  TPasMemberHint = (hDeprecated,hLibrary,hPlatform,hExperimental,hUnimplemented);
  TPasMemberHints = set of TPasMemberHint; 

  TPasElement = class;
  TPTreeElement = class of TPasElement;

  TOnForEachPasElement = procedure(El: TPasElement; arg: pointer) of object;

  { TPasElement }

  TPasElement = class(TPasElementBase)
  private
    FDocComment: String;
    FRefCount: LongWord;
    FName: string;
    FParent: TPasElement;
    FHints : TPasMemberHints;
    FHintMessage : String;
    {$ifdef pas2js}
    FPasElementId: NativeInt;
    class var FLastPasElementId: NativeInt;
    {$endif}
    {$ifdef EnablePasTreeGlobalRefCount}
    class var FGlobalRefCount: NativeInt;
    {$endif}
  protected
    procedure ProcessHints(const ASemiColonPrefix: boolean; var AResult: string); virtual;
    procedure SetParent(const AValue: TPasElement); virtual;
  public
    SourceFilename: string;
    SourceLinenumber: Integer;
    SourceEndLinenumber: Integer;
    Visibility: TPasMemberVisibility;
    {$IFDEF CheckPasTreeRefCount}
  public
    RefIds: TStringList;
    NextRefEl, PrevRefEl: TPasElement;
    class var FirstRefEl, LastRefEl: TPasElement;
    procedure ChangeRefId(const OldId, NewId: string);
    {$ENDIF}
    constructor Create(const AName: string; AParent: TPasElement); virtual;
    destructor Destroy; override;
    procedure AddRef{$IFDEF CheckPasTreeRefCount}(const aId: string){$ENDIF};
    procedure Release{$IFDEF CheckPasTreeRefCount}(const aId: string){$ENDIF};
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); virtual;
    procedure ForEachChildCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer; Child: TPasElement; CheckParent: boolean); virtual;
    function FullPath: string;                  // parent's names, until parent is not TPasDeclarations
    function ParentPath: string;                // parent's names
    function FullName: string; virtual;         // FullPath + Name
    function PathName: string; virtual;         // = Module.Name + ParentPath
    function GetModule: TPasModule;
    function ElementTypeName: string; virtual;
    Function HintsString : String;
    function GetDeclaration(full : Boolean) : string; virtual;
    procedure Accept(Visitor: TPassTreeVisitor); override;
    procedure ClearTypeReferences(aType: TPasElement); virtual;
    function HasParent(aParent: TPasElement): boolean;
    property RefCount: LongWord read FRefCount;
    property Name: string read FName write FName;
    property Parent: TPasElement read FParent Write SetParent;
    property Hints : TPasMemberHints Read FHints Write FHints;
    property HintMessage : String Read FHintMessage Write FHintMessage;
    property DocComment : String Read FDocComment Write FDocComment;
    {$ifdef pas2js}
    property PasElementId: NativeInt read FPasElementId; // global unique id
    {$endif}
    {$ifdef EnablePasTreeGlobalRefCount}
    class property GlobalRefCount: NativeInt read FGlobalRefCount write FGlobalRefCount;
    {$endif}
  end;
  TPasElementArray = array of TPasElement;

  TPasExprKind = (pekIdent, pekNumber, pekString, pekSet, pekNil, pekBoolConst,
     pekRange, pekUnary, pekBinary, pekFuncParams, pekArrayParams, pekListOfExp,
     pekInherited, pekSelf, pekSpecialize, pekProcedure);

  TExprOpCode = (eopNone,
                 eopAdd,eopSubtract,eopMultiply,eopDivide{/}, eopDiv{div},eopMod, eopPower,// arithmetic
                 eopShr,eopShl, // bit operations
                 eopNot,eopAnd,eopOr,eopXor, // logical/bit
                 eopEqual, eopNotEqual,  // Logical
                 eopLessThan,eopGreaterThan, eopLessthanEqual,eopGreaterThanEqual, // ordering
                 eopIn,eopIs,eopAs, eopSymmetricaldifference, // Specials
                 eopAddress, eopDeref, eopMemAddress, // Pointers  eopMemAddress=**
                 eopSubIdent); // SomeRec.A, A is subIdent of SomeRec

  { TPasExpr }

  TPasExpr = class(TPasElement)
    Kind      : TPasExprKind;
    OpCode    : TExprOpCode;
    format1,format2 : TPasExpr; // write, writeln, str
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; AOpCode: TExprOpCode); virtual; overload;
    destructor Destroy; override;
  end;

  { TUnaryExpr }

  TUnaryExpr = class(TPasExpr)
    Operand   : TPasExpr;
    constructor Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  end;

  { TBinaryExpr }

  TBinaryExpr = class(TPasExpr)
    left      : TPasExpr;
    right     : TPasExpr;
    constructor Create(AParent : TPasElement; xleft, xright: TPasExpr; AOpCode: TExprOpCode); overload;
    constructor CreateRange(AParent : TPasElement; xleft, xright: TPasExpr); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    class function IsRightSubIdent(El: TPasElement): boolean;
  end;

  { TPrimitiveExpr }

  TPrimitiveExpr = class(TPasExpr)
    Value     : String;
    constructor Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : string); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;
  
  { TBoolConstExpr }

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
  Public
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  { TSelfExpr }

  TSelfExpr = class(TPasExpr)
    constructor Create(AParent : TPasElement); overload;
    function GetDeclaration(full : Boolean) : string; override;
  end;

  TPasExprArray = array of TPasExpr;

  { TParamsExpr - source position is the opening bracket }

  TParamsExpr = class(TPasExpr)
    Value     : TPasExpr;
    Params    : TPasExprArray;
    // Kind: pekArrayParams, pekFuncParams, pekSet
    constructor Create(AParent : TPasElement; AKind: TPasExprKind); overload;
    function GetDeclaration(full : Boolean) : string; override;
    destructor Destroy; override;
    procedure AddParam(xp: TPasExpr);
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  end;

  { TRecordValues }

  TRecordValuesItem = record
    Name      : String;
    NameExp   : TPrimitiveExpr;
    ValueExp  : TPasExpr;
  end;
  PRecordValuesItem = ^TRecordValuesItem;
  TRecordValuesItemArray = array of TRecordValuesItem;

  TRecordValues = class(TPasExpr)
    Fields    : TRecordValuesItemArray;
    constructor Create(AParent : TPasElement); overload;
    destructor Destroy; override;
    procedure AddField(AName: TPrimitiveExpr; Value: TPasExpr);
    function GetDeclaration(full : Boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  end;

  { TArrayValues }

  TArrayValues = class(TPasExpr)
    Values    : TPasExprArray;
    constructor Create(AParent : TPasElement); overload;
    destructor Destroy; override;
    procedure AddValues(AValue: TPasExpr);
    function GetDeclaration(full : Boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  end;

  { TPasDeclarations - base class of TPasSection, TProcedureBody }

  TPasDeclarations = class(TPasElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Declarations: TFPList; // list of TPasElement
    // Declarations contains all the following:
    Attributes, // TPasAttributes
    Classes,    // TPasClassType, TPasRecordType
    Consts,     // TPasConst
    ExportSymbols,// TPasExportSymbol
    Functions,  // TPasProcedure
    Properties, // TPasProperty
    ResStrings, // TPasResString
    Types,      // TPasType, except TPasClassType, TPasRecordType
    Variables   // TPasVariable, not descendants
      : TFPList;
  end;

  { TPasUsesUnit - Parent is TPasSection }

  TPasUsesUnit = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Expr: TPasExpr; // name expression
    InFilename: TPrimitiveExpr; // Kind=pekString, can be nil
    Module: TPasElement; // TPasUnresolvedUnitRef or TPasModule
  end;
  TPasUsesClause = array of TPasUsesUnit;

  { TPasSection }

  TPasSection = class(TPasDeclarations)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function AddUnitToUsesList(const AUnitName: string; aName: TPasExpr = nil;
      InFilename: TPrimitiveExpr = nil; aModule: TPasElement = nil;
      UsesUnit: TPasUsesUnit = nil): TPasUsesUnit;
    function ElementTypeName: string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure ReleaseUsedUnits;
  public
    UsesList: TFPList; // kept for compatibility, see TPasUsesUnit.Module
    UsesClause: TPasUsesClause;
    PendingUsedIntf: TPasUsesUnit; // <>nil while resolving a uses cycle
  end;
  TPasSectionClass = class of TPasSection;

  { TInterfaceSection }

  TInterfaceSection = class(TPasSection)
  public
    function ElementTypeName: string; override;
  end;

  { TImplementationSection }

  TImplementationSection = class(TPasSection)
  public
    function ElementTypeName: string; override;
  end;

  { TProgramSection }

  TProgramSection = class(TImplementationSection)
  public
    function ElementTypeName: string; override;
  end;

  { TLibrarySection }

  TLibrarySection = class(TImplementationSection)
  public
    function ElementTypeName: string; override;
  end;

  TPasImplCommandBase = class;
  TInitializationSection = class;
  TFinalizationSection = class;

  { TPasModule }

  TPasModule = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure ReleaseUsedUnits; virtual;
  public
    GlobalDirectivesSection: TPasImplCommandBase; // not used by pparser
    InterfaceSection: TInterfaceSection;
    ImplementationSection: TImplementationSection;
    InitializationSection: TInitializationSection; // in TPasProgram the begin..end.
    FinalizationSection: TFinalizationSection;
    PackageName: string;
    Filename   : String;  // the IN filename, only written when not empty.
  end;

  { TPasUnitModule }

  TPasUnitModule = Class(TPasModule)
    function ElementTypeName: string; override;
  end;

  { TPasProgram }

  TPasProgram = class(TPasModule)
  Public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure ReleaseUsedUnits; override;
  Public
    ProgramSection: TProgramSection;
    InputFile,OutPutFile : String;
    // Note: the begin..end. block is in the InitializationSection
  end;

  { TPasLibrary }

  TPasLibrary = class(TPasModule)
  Public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure ReleaseUsedUnits; override;
  Public
    LibrarySection: TLibrarySection;
    InputFile,OutPutFile : String;
  end;

  { TPasPackage }

  TPasPackage = class(TPasElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Modules: TFPList;     // List of TPasModule objects
  end;

  { TPasResString }

  TPasResString = class(TPasElement)
  public
    Destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : Boolean) : string; Override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Expr: TPasExpr;
  end;

  { TPasType }

  TPasType = class(TPasElement)
  Protected
    Function FixTypeDecl(aDecl: String) : String;
  public
    function ElementTypeName: string; override;
  end;
  TPasTypeArray = array of TPasType;

  { TPasAliasType }

  TPasAliasType = class(TPasType)
  protected
    procedure SetParent(const AValue: TPasElement); override;
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : Boolean): string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure ClearTypeReferences(aType: TPasElement); override;
  public
    DestType: TPasType;
    Expr: TPasExpr;
  end;

  { TPasPointerType - todo: change it TPasAliasType }

  TPasPointerType = class(TPasType)
  protected
    procedure SetParent(const AValue: TPasElement); override;
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : Boolean): string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure ClearTypeReferences(aType: TPasElement); override;
  public
    DestType: TPasType;
  end;

  { TPasTypeAliasType }

  TPasTypeAliasType = class(TPasAliasType)
  public
    function ElementTypeName: string; override;
  end;

  { TPasGenericTemplateType - type param of a generic }

  TPasGenericTemplateType = Class(TPasType)
  public
    destructor Destroy; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure AddConstraint(El: TPasElement);
  Public
    TypeConstraint: String deprecated; // deprecated in fpc 3.3.1
    Constraints: TPasElementArray; // list of TPasExpr or TPasType, can be nil!
  end;

  { TPasGenericType - abstract base class for all types which can be generics }

  TPasGenericType = class(TPasType)
  private
    procedure ClearChildReferences(El: TPasElement; arg: pointer);
  protected
    procedure SetParent(const AValue: TPasElement); override;
  public
    GenericTemplateTypes: TFPList; // list of TPasGenericTemplateType, can be nil
    destructor Destroy; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure SetGenericTemplates(AList: TFPList); virtual;
  end;

  { TPasSpecializeType DestType<Params> }

  TPasSpecializeType = class(TPasAliasType)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full: boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Params: TFPList; // list of TPasType or TPasExpr
  end;

  { TInlineSpecializeExpr - A<B,C> }

  TInlineSpecializeExpr = class(TPasExpr)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : Boolean): string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    NameExpr: TPasExpr;
    Params: TFPList; // list of TPasType
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
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    RangeExpr : TBinaryExpr; // Kind=pekRange
    Destructor Destroy; override;
    Function RangeStart : String;
    Function RangeEnd : String;
  end;

  { TPasArrayType }

  TPasArrayType = class(TPasGenericType)
  protected
    procedure SetParent(const AValue: TPasElement); override;
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
  public
    IndexRange : string; // only valid if Parser po_arrayrangeexpr disabled
    Ranges: TPasExprArray; // only valid if Parser po_arrayrangeexpr enabled
    PackMode : TPackMode;
    ElType: TPasType; // nil means array-of-const
    function IsGenericArray : Boolean;
    function IsPacked : Boolean;
    procedure AddRange(Range: TPasExpr);
  end;

  { TPasFileType }

  TPasFileType = class(TPasType)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    ElType: TPasType;
  end;

  { TPasEnumValue - Parent is TPasEnumType }

  TPasEnumValue = class(TPasElement)
  public
    function ElementTypeName: string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
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
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Values: TFPList;      // List of TPasEnumValue
  end;

  { TPasSetType }

  TPasSetType = class(TPasType)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    EnumType: TPasType;
    IsPacked : Boolean;
  end;

  TPasRecordType = class;

  { TPasVariant }

  TPasVariant = class(TPasElement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Values: TFPList; // list of TPasElement
    Members: TPasRecordType;
  end;

  { TPasMembersType - base type for TPasRecordType and TPasClassType }

  TPasMembersType = class(TPasGenericType)
  public
    PackMode: TPackMode;
    Members: TFPList;
    Constructor Create(const AName: string; AParent: TPasElement); override;
    Destructor Destroy; override;
    Function IsPacked: Boolean;
    Function IsBitPacked : Boolean;
    Procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  end;

  { TPasRecordType }

  TPasRecordType = class(TPasMembersType)
  private
    procedure GetMembers(S: TStrings);
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    VariantEl: TPasElement; // nil or TPasVariable or TPasType
    Variants: TFPList;	// list of TPasVariant elements, may be nil!
    Function IsAdvancedRecord : Boolean;
  end;

  TPasObjKind = (
    okObject, okClass, okInterface,
    // okGeneric  removed in FPC 3.3.1  check instead GenericTemplateTypes<>nil
    // okSpecialize removed in FPC 3.1.1
    okClassHelper,okRecordHelper,okTypeHelper,
    okDispInterface);
const
  okWithFields = [okObject, okClass];
  okAllHelpers = [okClassHelper,okRecordHelper,okTypeHelper];
  okWithClassFields = okWithFields+okAllHelpers;

type

  TPasClassInterfaceType = (
    citCom, // default
    citCorba
    );

  { TPasClassType }

  TPasClassType = class(TPasMembersType)
  protected
    procedure SetParent(const AValue: TPasElement); override;
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    ObjKind: TPasObjKind;
    AncestorType: TPasType;   // TPasClassType or TPasUnresolvedTypeRef or TPasAliasType or TPasTypeAliasType
                              // Note: AncestorType can be nil even though it has a default ancestor
    HelperForType: TPasType;  // any type, except helper
    IsForward: Boolean;
    IsExternal : Boolean;
    IsShortDefinition: Boolean;//class(anchestor); without end
    GUIDExpr : TPasExpr;
    Modifiers: TStringList;
    Interfaces : TFPList; // list of TPasType
    ExternalNameSpace : String;
    ExternalName : String;
    InterfaceType: TPasClassInterfaceType;
    Function FindMember(MemberClass : TPTreeElement; Const MemberName : String) : TPasElement;
    Function FindMemberInAncestors(MemberClass : TPTreeElement; Const MemberName : String) : TPasElement;
    Function InterfaceGUID : string;
    Function IsSealed : Boolean;
    Function IsAbstract : Boolean;
    Function HasModifier(const aModifier: String): Boolean;
  end;

  TArgumentAccess = (argDefault, argConst, argVar, argOut, argConstRef);

  { TPasArgument }

  TPasArgument = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure ClearTypeReferences(aType: TPasElement); override;
  public
    Access: TArgumentAccess;
    ArgType: TPasType; // can be nil, when Access<>argDefault
    ValueExpr: TPasExpr; // the default value
    Function Value : String;
  end;

  { TPasProcedureType }

  TPasProcedureType = class(TPasGenericType)
  private
    function GetIsNested: Boolean;
    function GetIsOfObject: Boolean;
    function GetIsReference: Boolean;
    procedure SetIsNested(const AValue: Boolean);
    procedure SetIsOfObject(const AValue: Boolean);
    procedure SetIsReference(AValue: Boolean);
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    class function TypeName: string; virtual;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure GetArguments(List : TStrings);
    function CreateArgument(const AName, AUnresolvedTypeName: string): TPasArgument; // not used by TPasParser
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Args: TFPList;        // List of TPasArgument objects
    CallingConvention: TCallingConvention;
    Modifiers: TProcTypeModifiers;
    VarArgsType: TPasType;
    property IsOfObject: Boolean read GetIsOfObject write SetIsOfObject;
    property IsNested : Boolean read GetIsNested write SetIsNested;
    property IsReferenceTo : Boolean Read GetIsReference write SetIsReference;
  end;
  TPasProcedureTypeClass = class of TPasProcedureType;

  { TPasResultElement }

  TPasResultElement = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure ClearTypeReferences(aType: TPasElement); override;
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
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
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

  { TPasUnresolvedUnitRef }

  TPasUnresolvedUnitRef = Class(TPasUnresolvedSymbolRef)
  public
    FileName : string;
    function ElementTypeName: string; override;
  end;

  { TPasStringType - e.g. string[len] }

  TPasStringType = class(TPasUnresolvedTypeRef)
  public
    LengthExpr : String;
    function ElementTypeName: string; override;
  end;

  { TPasTypeRef  - not used by TPasParser }

  TPasTypeRef = class(TPasUnresolvedTypeRef)
  public
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    RefType: TPasType;
  end;

  { TPasVariable }
  TVariableModifier = (vmCVar, vmExternal, vmPublic, vmExport, vmClass, vmStatic);
  TVariableModifiers = set of TVariableModifier;

  TPasVariable = class(TPasElement)
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure ClearTypeReferences(aType: TPasElement); override;
  public
    VarType: TPasType;
    VarModifiers : TVariableModifiers;
    LibraryName : TPasExpr; // libname of modifier external
    ExportName : TPasExpr; // symbol name of modifier external, export and public
    Modifiers : string;
    AbsoluteLocation : String deprecated; // deprecated in fpc 3.1.1
    AbsoluteExpr: TPasExpr;
    Expr: TPasExpr;
    Function Value : String;
  end;

  { TPasExportSymbol }

  TPasExportSymbol = class(TPasElement)
  public
    ExportName : TPasExpr;
    ExportIndex : TPasExpr;
    Destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  end;

  { TPasConst }

  TPasConst = class(TPasVariable)
  public
    IsConst: boolean; // true iff untyped const or typed with $WritableConst off
    function ElementTypeName: string; override;
  end;

  { TPasProperty }

  TPasProperty = class(TPasVariable)
  private
    FArgs: TFPList;
    FResolvedType : TPasType;
    function GetIsClass: boolean;
    procedure SetIsClass(AValue: boolean);
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function GetDeclaration(full : boolean) : string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    IndexExpr: TPasExpr;
    ReadAccessor: TPasExpr;
    WriteAccessor: TPasExpr;
    DispIDExpr : TPasExpr;   // Can be nil.
    Implements: TPasExprArray;
    StoredAccessor: TPasExpr;
    DefaultExpr: TPasExpr;
    ReadAccessorName: string; // not used by resolver
    WriteAccessorName: string; // not used by resolver
    ImplementsName: string; // not used by resolver
    StoredAccessorName: string; // not used by resolver
    DispIDReadOnly,
    IsDefault, IsNodefault: Boolean;
    property Args: TFPList read FArgs; // List of TPasArgument objects
    property IsClass: boolean read GetIsClass write SetIsClass;
    Function ResolvedType : TPasType;
    Function IndexValue : String;
    Function DefaultValue : string;
  end;

  { TPasAttributes }

  TPasAttributes = class(TPasElement)
  public
    destructor Destroy; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure AddCall(Expr: TPasExpr);
  public
    Calls: TPasExprArray;
  end;

  TProcType = (ptProcedure, ptFunction,
               ptOperator, ptClassOperator,
               ptConstructor, ptDestructor,
               ptClassProcedure, ptClassFunction,
               ptClassConstructor, ptClassDestructor,
               ptAnonymousProcedure, ptAnonymousFunction);

  { TPasProcedureBase }

  TPasProcedureBase = class(TPasElement)
  public
    function TypeName: string; virtual; abstract;
  end;

  { TPasOverloadedProc - not used by resolver }

  TPasOverloadedProc = class(TPasProcedureBase)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function TypeName: string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Overloads: TFPList;           // List of TPasProcedure nodes
  end;

  { TPasProcedure }

  TProcedureModifier = (pmVirtual, pmDynamic, pmAbstract, pmOverride,
                        pmExport, pmOverload, pmMessage, pmReintroduce,
                        pmInline,pmAssembler, pmPublic,
                        pmCompilerProc,pmExternal,pmForward, pmDispId, 
                        pmNoReturn, pmFar, pmFinal);
  TProcedureModifiers = Set of TProcedureModifier;
  TProcedureMessageType = (pmtNone,pmtInteger,pmtString);

  { TProcedureNamePart }

  TProcedureNamePart = class
    Name: string;
    Templates: TFPList; // optional list of TPasGenericTemplateType, can be nil!
  end;
  TProcedureNameParts = TFPList; // list of TProcedureNamePart
                        
  TProcedureBody = class;

  { TPasProcedure - named procedure, not anonymous }

  TPasProcedure = class(TPasProcedureBase)
  Private
    FModifiers : TProcedureModifiers;
    FMessageName : String;
    FMessageType : TProcedureMessageType;
    function GetCallingConvention: TCallingConvention;
    procedure SetCallingConvention(AValue: TCallingConvention);
  public
    destructor Destroy; override;
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetDeclaration(full: Boolean): string; override;
    procedure GetModifiers(List: TStrings);
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    PublicName, // e.g. public PublicName;
    LibrarySymbolName,
    LibraryExpr : TPasExpr; // e.g. external LibraryExpr name LibrarySymbolName;
    DispIDExpr :  TPasExpr;
    MessageExpr: TPasExpr;
    AliasName : String;
    ProcType : TPasProcedureType;
    Body : TProcedureBody;
    NameParts: TProcedureNameParts; // only used for generic aka parametrized functions
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
    Function GetProcTypeEnum: TProcType; virtual;
    procedure SetNameParts(Parts: TProcedureNameParts);
    Property Modifiers : TProcedureModifiers Read FModifiers Write FModifiers;
    Property CallingConvention : TCallingConvention Read GetCallingConvention Write SetCallingConvention;
    Property MessageName : String Read FMessageName Write FMessageName;
    property MessageType : TProcedureMessageType Read FMessageType Write FMessageType;
  end;
  TPasProcedureClass = class of TPasProcedure;

  TArrayOfPasProcedure = array of TPasProcedure;

  { TPasFunction - named function, not anonymous function}

  TPasFunction = class(TPasProcedure)
  private
    function GetFT: TPasFunctionType; inline;
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
    Property FuncType : TPasFunctionType Read GetFT;
    function GetProcTypeEnum: TProcType; override;
  end;

  { TPasOperator }
  TOperatorType = (
    otUnknown,
    otImplicit, otExplicit,
    otMul, otPlus, otMinus, otDivision,
    otLessThan, otEqual, otGreaterThan,
    otAssign, otNotEqual, otLessEqualThan, otGreaterEqualThan,
    otPower, otSymmetricalDifference,
    otInc, otDec,
    otMod,
    otNegative, otPositive,
    otBitWiseOr,
    otDiv,
    otLeftShift,
    otLogicalOr,
    otBitwiseAnd, otbitwiseXor,
    otLogicalAnd, otLogicalNot, otLogicalXor,
    otRightShift,
    otEnumerator, otIn
    );
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
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetProcTypeEnum: TProcType; override;
    function GetDeclaration (full : boolean) : string; override;
    Property OperatorType : TOperatorType Read FOperatorType Write FOperatorType;
    // True if the declaration was using a token instead of an identifier
    Property TokenBased : Boolean Read FTokenBased Write FTokenBased;
  end;

  { TPasClassOperator }

  TPasClassOperator = class(TPasOperator)
    function TypeName: string; override;
    function GetProcTypeEnum: TProcType; override;
  end;


  { TPasConstructor }

  TPasConstructor = class(TPasProcedure)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetProcTypeEnum: TProcType; override;
  end;

  { TPasClassConstructor }

  TPasClassConstructor  = class(TPasConstructor)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetProcTypeEnum: TProcType; override;
  end;

  { TPasDestructor }

  TPasDestructor = class(TPasProcedure)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetProcTypeEnum: TProcType; override;
  end;

  { TPasClassDestructor }

  TPasClassDestructor  = class(TPasDestructor)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetProcTypeEnum: TProcType; override;
  end;

  { TPasClassProcedure }

  TPasClassProcedure = class(TPasProcedure)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetProcTypeEnum: TProcType; override;
  end;

  { TPasClassFunction }

  TPasClassFunction = class(TPasFunction)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetProcTypeEnum: TProcType; override;
  end;

  { TPasAnonymousProcedure - parent is TProcedureExpr }

  TPasAnonymousProcedure = class(TPasProcedure)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
    function GetProcTypeEnum: TProcType; override;
  end;

  { TPasAnonymousFunction - parent is TProcedureExpr and ProcType is TPasFunctionType}

  TPasAnonymousFunction = class(TPasAnonymousProcedure)
  private
    function GetFT: TPasFunctionType; inline;
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
    Property FuncType : TPasFunctionType Read GetFT;
    function GetProcTypeEnum: TProcType; override;
  end;

  { TProcedureExpr }

  TProcedureExpr = class(TPasExpr)
  public
    Proc: TPasAnonymousProcedure;
    constructor Create(AParent: TPasElement); overload;
    destructor Destroy; override;
    function GetDeclaration(full: Boolean): string; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  end;

  { TPasMethodResolution }

  TPasMethodResolution = class(TPasElement)
  public
    destructor Destroy; override;
  public
    ProcClass: TPasProcedureClass;
    InterfaceName: TPasExpr;
    InterfaceProc: TPasExpr;
    ImplementationProc: TPasExpr;
  end;

  TPasImplBlock = class;

  { TProcedureBody - the var+type+const+begin, without the header, child of TPasProcedure }

  TProcedureBody = class(TPasDeclarations)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
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
    IsClassMethod: boolean;
  end;

  { TPasConstructorImpl - used by mkxmlrpc, not by pparser }

  TPasConstructorImpl = class(TPasProcedureImpl)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

  { TPasDestructorImpl - used by mkxmlrpc, not by pparser }

  TPasDestructorImpl = class(TPasProcedureImpl)
  public
    function ElementTypeName: string; override;
    function TypeName: string; override;
  end;

  { TPasImplElement - implementation element }

  TPasImplElement = class(TPasElement)
  end;

  { TPasImplCommandBase }

  TPasImplCommandBase = class(TPasImplElement)
  public
    SemicolonAtEOL: boolean;
    constructor Create(const AName: string; AParent: TPasElement); override;
  end;

  { TPasImplCommand - currently used as empty statement, e.g. if then else ; }

  TPasImplCommand = class(TPasImplCommandBase)
  public
    Command: string; // never set by TPasParser
  end;

  { TPasImplCommands - used by mkxmlrpc, not used by pparser }

  TPasImplCommands = class(TPasImplCommandBase)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
  public
    Commands: TStrings;
  end;

  { TPasLabels }

  TPasLabels = class(TPasImplElement)
  public
    Labels: TStrings;
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
    function AddIfElse(const ACondition: TPasExpr): TPasImplIfElse;
    function AddWhileDo(const ACondition: TPasExpr): TPasImplWhileDo;
    function AddWithDo(const Expression: TPasExpr): TPasImplWithDo;
    function AddCaseOf(const Expression: TPasExpr): TPasImplCaseOf;
    function AddForLoop(AVar: TPasVariable;
      const AStartValue, AEndValue: TPasExpr): TPasImplForLoop;
    function AddForLoop(AVarName : TPasExpr; AStartValue, AEndValue: TPasExpr;
      ADownTo: Boolean = false): TPasImplForLoop;
    function AddTry: TPasImplTry;
    function AddExceptOn(const VarName, TypeName: string): TPasImplExceptOn;
    function AddExceptOn(const VarName: string; VarType: TPasType): TPasImplExceptOn;
    function AddExceptOn(const VarEl: TPasVariable): TPasImplExceptOn;
    function AddExceptOn(const TypeEl: TPasType): TPasImplExceptOn;
    function AddRaise: TPasImplRaise;
    function AddLabelMark(const Id: string): TPasImplLabelMark;
    function AddAssign(left, right: TPasExpr): TPasImplAssign;
    function AddSimple(exp: TPasExpr): TPasImplSimple;
    function CloseOnSemicolon: boolean; virtual;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Elements: TFPList;    // list of TPasImplElement
  end;
  TPasImplBlockClass = class of TPasImplBlock;

  { TPasImplStatement - base class }

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

  { TPasImplAsmStatement }

  TPasImplAsmStatement = class (TPasImplStatement)
  private
    FTokens: TStrings;
  Public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    Property Tokens : TStrings Read FTokens;
  end;

  { TPasImplRepeatUntil }

  TPasImplRepeatUntil = class(TPasImplBlock)
  public
    ConditionExpr : TPasExpr;
    destructor Destroy; override;
    Function Condition: string;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  end;

  { TPasImplIfElse }

  TPasImplIfElse = class(TPasImplBlock)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    function CloseOnSemicolon: boolean; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    ConditionExpr: TPasExpr;
    IfBranch: TPasImplElement;
    ElseBranch: TPasImplElement; // can be nil
    Function Condition: string;
  end;

  { TPasImplWhileDo }

  TPasImplWhileDo = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
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
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Expressions: TFPList; // list of TPasExpr
    Body: TPasImplElement;
  end;

  TPasImplCaseStatement = class;
  TPasImplCaseElse = class;

  { TPasImplCaseOf - Elements are TPasImplCaseStatement }

  TPasImplCaseOf = class(TPasImplBlock)
  public
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    function AddCase(const Expression: TPasExpr): TPasImplCaseStatement;
    function AddElse: TPasImplCaseElse;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    CaseExpr : TPasExpr;
    ElseBranch: TPasImplCaseElse; // this is also in Elements
    function Expression: string;
  end;

  { TPasImplCaseStatement }

  TPasImplCaseStatement = class(TPasImplStatement)
  public
    constructor Create(const AName: string; AParent: TPasElement); override;
    destructor Destroy; override;
    procedure AddElement(Element: TPasImplElement); override;
    procedure AddExpression(const Expr: TPasExpr);
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    Expressions: TFPList; // list of TPasExpr
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
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    VariableName : TPasExpr;
    LoopType : TLoopType;
    StartExpr : TPasExpr;
    EndExpr : TPasExpr; // if LoopType=ltIn this is nil
    Body: TPasImplElement;
    Variable: TPasVariable; // not used by TPasParser
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
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  end;

  { TPasImplSimple }

  TPasImplSimple = class (TPasImplStatement)
  public
    Expr  : TPasExpr;
    Destructor Destroy; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
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
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  public
    FinallyExcept: TPasImplTryHandler; // not in Elements
    ElseBranch: TPasImplTryExceptElse; // not in Elements
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
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
    procedure ClearTypeReferences(aType: TPasElement); override;
  public
    VarEl: TPasVariable; // can be nil
    TypeEl : TPasType; // if VarEl<>nil then TypeEl=VarEl.VarType
    Body: TPasImplElement;
    Function VariableName : String;
    Function TypeName: string;
  end;

  { TPasImplRaise }

  TPasImplRaise = class(TPasImplStatement)
  public
    destructor Destroy; override;
    procedure ForEachCall(const aMethodCall: TOnForEachPasElement;
      const Arg: Pointer); override;
  Public
    ExceptObject,
    ExceptAddr : TPasExpr;
  end;

  { TPasImplLabelMark }

  TPasImplLabelMark = class(TPasImplElement)
  public
    LabelId: String;
  end;

  { TPassTreeVisitor }

  TPassTreeVisitor = class
  public
    procedure Visit(obj: TPasElement); virtual;
  end;

const
  AccessNames: array[TArgumentAccess] of string{$ifdef fpc}[9]{$endif} = ('', 'const ', 'var ', 'out ','constref ');
  AccessDescriptions: array[TArgumentAccess] of string{$ifdef fpc}[9]{$endif} = ('default', 'const', 'var', 'out','constref');
  AllVisibilities: TPasMemberVisibilities =
     [visDefault, visPrivate, visProtected, visPublic,
      visPublished, visAutomated];

  VisibilityNames: array[TPasMemberVisibility] of string = (
    'default','private', 'protected', 'public', 'published', 'automated',
    'strict private', 'strict protected');

  ObjKindNames: array[TPasObjKind] of string = (
    'object', 'class', 'interface',
    'class helper','record helper','type helper',
    'dispinterface');

  InterfaceTypeNames: array[TPasClassInterfaceType] of string = (
    'COM',
    'Corba'
    );

  ExprKindNames : Array[TPasExprKind] of string = (
      'Ident',
      'Number',
      'String',
      'Set',
      'Nil',
      'BoolConst',
      'Range',
      'Unary',
      'Binary',
      'FuncParams',
      'ArrayParams',
      'ListOfExp',
      'Inherited',
      'Self',
      'Specialize',
      'Procedure');

  OpcodeStrings : Array[TExprOpCode] of string = (
        '','+','-','*','/','div','mod','**',
        'shr','shl',
        'not','and','or','xor',
        '=','<>',
        '<','>','<=','>=',
        'in','is','as','><',
        '@','^','@@',
        '.');


  UnaryOperators = [otImplicit,otExplicit,otAssign,otNegative,otPositive,otEnumerator];

  OperatorTokens : Array[TOperatorType] of string
       =  ('','','','*','+','-','/','<','=',
           '>',':=','<>','<=','>=','**',
           '><','Inc','Dec','mod','-','+','Or','div',
           'shl','or','and','xor','and','not','xor',
           'shr','enumerator','in');
  OperatorNames : Array[TOperatorType] of string
       =  ('','implicit','explicit','multiply','add','subtract','divide','lessthan','equal',
           'greaterthan','assign','notequal','lessthanorequal','greaterthanorequal','power',
           'symmetricaldifference','inc','dec','modulus','negative','positive','bitwiseor','intdivide',
           'leftshift','logicalor','bitwiseand','bitwisexor','logicaland','logicalnot','logicalxor',
           'rightshift','enumerator','in');

  AssignKindNames : Array[TAssignKind] of string = (':=','+=','-=','*=','/=' );

  cPasMemberHint : Array[TPasMemberHint] of string =
      ( 'deprecated', 'library', 'platform', 'experimental', 'unimplemented' );
  cCallingConventions : Array[TCallingConvention] of string =
      ( '', 'Register','Pascal','CDecl','StdCall','OldFPCCall','SafeCall','SysCall');
  ProcTypeModifiers : Array[TProcTypeModifier] of string =
      ('of Object', 'is nested','static','varargs','reference to');

  ModifierNames : Array[TProcedureModifier] of string
                = ('virtual', 'dynamic','abstract', 'override',
                   'export', 'overload', 'message', 'reintroduce',
                   'inline','assembler','public',
                   'compilerproc','external','forward','dispid',
                   'noreturn','far','final');

  VariableModifierNames : Array[TVariableModifier] of string
     = ('cvar', 'external', 'public', 'export', 'class', 'static');

procedure ReleaseAndNil(var El: TPasElement {$IFDEF CheckPasTreeRefCount}; const Id: string{$ENDIF}); overload;
procedure ReleaseGenericTemplateTypes(var GenericTemplateTypes: TFPList{$IFDEF CheckPasTreeRefCount}; const Id: string{$ENDIF});
procedure ReleaseElementList(ElList: TFPList{$IFDEF CheckPasTreeRefCount}; const Id: string{$ENDIF});
function GenericTemplateTypesAsString(List: TFPList): string;
procedure ReleaseProcNameParts(var NameParts: TProcedureNameParts);

{$IFDEF HasPTDumpStack}
procedure PTDumpStack;
function GetPTDumpStack: string;
{$ENDIF}

implementation

uses SysUtils;

procedure ReleaseAndNil(var El: TPasElement {$IFDEF CheckPasTreeRefCount}; const Id: string{$ENDIF});
begin
  if El=nil then exit;
  {$IFDEF VerbosePasTreeMem}writeln('ReleaseAndNil ',El.Name,' ',El.ClassName);{$ENDIF}
  El.Release{$IFDEF CheckPasTreeRefCount}(Id){$ENDIF};
  El:=nil;
end;

procedure ReleaseGenericTemplateTypes(var GenericTemplateTypes: TFPList{$IFDEF CheckPasTreeRefCount}; const Id: string{$ENDIF});
var
  i: Integer;
  El: TPasElement;
begin
  if GenericTemplateTypes=nil then exit;
  for i := 0 to GenericTemplateTypes.Count - 1 do
    begin
    El:=TPasElement(GenericTemplateTypes[i]);
    El.Parent:=nil;
    El.Release{$IFDEF CheckPasTreeRefCount}(Id){$ENDIF};
    end;
  FreeAndNil(GenericTemplateTypes);
end;

procedure ReleaseElementList(ElList: TFPList{$IFDEF CheckPasTreeRefCount}; const Id: string{$ENDIF});
var
  i: Integer;
  El: TPasElement;
begin
  if ElList=nil then exit;
  for i := 0 to ElList.Count - 1 do
    begin
    El:=TPasElement(ElList[i]);
    if El<>nil then
      El.Release{$IFDEF CheckPasTreeRefCount}(Id){$ENDIF};
    end;
  ElList.Clear;
end;

function GenericTemplateTypesAsString(List: TFPList): string;
var
  i, j: Integer;
  T: TPasGenericTemplateType;
begin
  Result:='';
  for i:=0 to List.Count-1 do
    begin
    if i>0 then
      Result:=Result+',';
    T:=TPasGenericTemplateType(List[i]);
    Result:=Result+T.Name;
    if length(T.Constraints)>0 then
      begin
      Result:=Result+':';
      for j:=0 to length(T.Constraints)-1 do
        begin
        if j>0 then
          Result:=Result+',';
        Result:=Result+T.GetDeclaration(false);
        end;
      end;
    end;
  Result:='<'+Result+'>';
end;

procedure ReleaseProcNameParts(var NameParts: TProcedureNameParts);
var
  El: TPasElement;
  i, j: Integer;
  Part: TProcedureNamePart;
begin
  if NameParts=nil then exit;
  for i := NameParts.Count-1 downto 0 do
    begin
    Part:=TProcedureNamePart(NameParts[i]);
    if Part.Templates<>nil then
      begin
      for j:=0 to Part.Templates.Count-1 do
        begin
        El:=TPasGenericTemplateType(Part.Templates[j]);
        El.Parent:=nil;
        El.Release{$IFDEF CheckPasTreeRefCount}('TPasProcedure.NameParts'){$ENDIF};
        end;
      Part.Templates.Free;
      Part.Templates:=nil;
      end;
    NameParts.Delete(i);
    Part.Free;
    end;
  NameParts.Free;
  NameParts:=nil;
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

{ TPasGenericType }

procedure TPasGenericType.ClearChildReferences(El: TPasElement; arg: pointer);
begin
  El.ClearTypeReferences(Self);
  if arg=nil then ;
end;

procedure TPasGenericType.SetParent(const AValue: TPasElement);
begin
  if (AValue=nil) and (Parent<>nil) then
    begin
    // parent is cleared
    // -> clear all child references to this array (releasing loops)
    ForEachCall(@ClearChildReferences,nil);
    end;
  inherited SetParent(AValue);
end;

destructor TPasGenericType.Destroy;
begin
  ReleaseGenericTemplateTypes(GenericTemplateTypes{$IFDEF CheckPasTreeRefCount},'TPasGenericType'{$ENDIF});
  inherited Destroy;
end;

procedure TPasGenericType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  if GenericTemplateTypes<>nil then
    for i:=0 to GenericTemplateTypes.Count-1 do
      ForEachChildCall(aMethodCall,Arg,TPasElement(GenericTemplateTypes[i]),false);
end;

procedure TPasGenericType.SetGenericTemplates(AList: TFPList);
var
  I: Integer;
  El: TPasElement;
begin
  if GenericTemplateTypes=nil then
    GenericTemplateTypes:=TFPList.Create;
  For I:=0 to AList.Count-1 do
    begin
    El:=TPasElement(AList[i]);
    El.Parent:=Self;
    GenericTemplateTypes.Add(El);
    end;
  AList.Clear;
end;

{ TPasGenericTemplateType }

destructor TPasGenericTemplateType.Destroy;
var
  i: Integer;
begin
  for i:=0 to length(Constraints)-1 do
    Constraints[i].Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  Constraints:=nil;
  inherited Destroy;
end;

function TPasGenericTemplateType.GetDeclaration(full: boolean): string;
var
  i: Integer;
begin
  Result:=inherited GetDeclaration(full);
  if length(Constraints)>0 then
    begin
    Result:=Result+': ';
    for i:=0 to length(Constraints)-1 do
      begin
      if i>0 then
        Result:=Result+',';
      Result:=Result+Constraints[i].GetDeclaration(false);
      end;
    end;
end;

procedure TPasGenericTemplateType.ForEachCall(
  const aMethodCall: TOnForEachPasElement; const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to length(Constraints)-1 do
    ForEachChildCall(aMethodCall,Arg,Constraints[i],false);
end;

procedure TPasGenericTemplateType.AddConstraint(El: TPasElement);
var
  l: Integer;
begin
  l:=Length(Constraints);
  SetLength(Constraints,l+1);
  Constraints[l]:=El;
end;

{$IFDEF HasPTDumpStack}
procedure PTDumpStack;
begin
  {AllowWriteln}
  writeln(GetPTDumpStack);
  {AllowWriteln-}
end;

function GetPTDumpStack: string;
var
  bp: Pointer;
  addr: Pointer;
  oldbp: Pointer;
  CurAddress: Shortstring;
begin
  Result:='';
  { retrieve backtrace info }
  bp:=get_caller_frame(get_frame);
  while bp<>nil do begin
    addr:=get_caller_addr(bp);
    CurAddress:=BackTraceStrFunc(addr);
    Result:=Result+CurAddress+LineEnding;
    oldbp:=bp;
    bp:=get_caller_frame(bp);
    if (bp<=oldbp) or (bp>(StackBottom + StackLength)) then
      bp:=nil;
  end;
end;
{$ENDIF}

{ TPasAttributes }

destructor TPasAttributes.Destroy;
var
  i: Integer;
begin
  for i:=0 to length(Calls)-1 do
    Calls[i].Release{$IFDEF CheckPasTreeRefCount}('TPasAttributes.Destroy'){$ENDIF};
  inherited Destroy;
end;

procedure TPasAttributes.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to length(Calls)-1 do
    ForEachChildCall(aMethodCall,Arg,Calls[i],false);
end;

procedure TPasAttributes.AddCall(Expr: TPasExpr);
var
  i : Integer;
begin
  i:=Length(Calls);
  SetLength(Calls, i+1);
  Calls[i]:=Expr;
end;

{ TPasMethodResolution }

destructor TPasMethodResolution.Destroy;
begin
  ReleaseAndNil(TPasElement(InterfaceName){$IFDEF CheckPasTreeRefCount},'TPasMethodResolution.InterfaceName'{$ENDIF});
  ReleaseAndNil(TPasElement(InterfaceProc){$IFDEF CheckPasTreeRefCount},'TPasMethodResolution.InterfaceProc'{$ENDIF});
  ReleaseAndNil(TPasElement(ImplementationProc){$IFDEF CheckPasTreeRefCount},'TPasMethodResolution.ImplementationProc'{$ENDIF});
  inherited Destroy;
end;

{ TPasImplCommandBase }

constructor TPasImplCommandBase.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  SemicolonAtEOL := true;
end;

{ TInlineSpecializeExpr }

constructor TInlineSpecializeExpr.Create(const AName: string;
  AParent: TPasElement);
begin
  if AName='' then ;
  inherited Create(AParent, pekSpecialize, eopNone);
  Params:=TFPList.Create;
end;

destructor TInlineSpecializeExpr.Destroy;
var
  i: Integer;
begin
  TPasElement(NameExpr).Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
  for i:=0 to Params.Count-1 do
    TPasElement(Params[i]).Release{$IFDEF CheckPasTreeRefCount}('TInlineSpecializeExpr.Params'){$ENDIF};
  FreeAndNil(Params);
  inherited Destroy;
end;

function TInlineSpecializeExpr.ElementTypeName: string;
begin
  Result:=SPasTreeSpecializedExpr;
end;

function TInlineSpecializeExpr.GetDeclaration(full: Boolean): string;
var
  i: Integer;
begin
  Result:='specialize '+NameExpr.GetDeclaration(false)+'<';
  for i:=0 to Params.Count-1 do
    begin
    if i>0 then
      Result:=Result+',';
    Result:=Result+TPasElement(Params[i]).GetDeclaration(false);
    end;
  Result:=Result+'>';
  if full then ;
end;

procedure TInlineSpecializeExpr.ForEachCall(
  const aMethodCall: TOnForEachPasElement; const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,NameExpr,false);
  for i:=0 to Params.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Params[i]),true);
end;

{ TPasSpecializeType }

constructor TPasSpecializeType.Create(const AName: string; AParent: TPasElement
  );
begin
  inherited Create(AName, AParent);
  Params:=TFPList.Create;
end;

destructor TPasSpecializeType.Destroy;
var
  i: Integer;
begin
  for i:=0 to Params.Count-1 do
    TPasElement(Params[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasSpecializeType.Params'){$ENDIF};
  FreeAndNil(Params);
  inherited Destroy;
end;

function TPasSpecializeType.ElementTypeName: string;
begin
  Result:=SPasTreeSpecializedType;
end;

function TPasSpecializeType.GetDeclaration(full: boolean): string;
var
  i: Integer;
begin
  Result:='specialize '+DestType.Name+'<';
  for i:=0 to Params.Count-1 do
    begin
    if i>0 then
      Result:=Result+',';
    Result:=Result+TPasElement(Params[i]).GetDeclaration(false);
    end;
  If Full and (Name<>'') then
    begin
    Result:=Name+' = '+Result;
    ProcessHints(False,Result);
    end;
end;

procedure TPasSpecializeType.ForEachCall(
  const aMethodCall: TOnForEachPasElement; const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to Params.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Params[i]),true);
end;

{ TInterfaceSection }

function TInterfaceSection.ElementTypeName: string;
begin
  Result:=SPasTreeInterfaceSection;
end;

{ TLibrarySection }

function TLibrarySection.ElementTypeName: string;
begin
  Result:=SPasTreeLibrarySection;
end;

{ TProgramSection }

function TProgramSection.ElementTypeName: string;
begin
  Result:=SPasTreeProgramSection;
end;

{ TImplementationSection }

function TImplementationSection.ElementTypeName: string;
begin
  Result:=SPasTreeImplementationSection;
end;

{ TPasUsesUnit }

destructor TPasUsesUnit.Destroy;
begin
  ReleaseAndNil(TPasElement(Expr){$IFDEF CheckPasTreeRefCount},'TPasUsesUnit.Expr'{$ENDIF});
  ReleaseAndNil(TPasElement(InFilename){$IFDEF CheckPasTreeRefCount},'TPasUsesUnit.InFilename'{$ENDIF});
  ReleaseAndNil(TPasElement(Module){$IFDEF CheckPasTreeRefCount},'TPasUsesUnit.Module'{$ENDIF});
  inherited Destroy;
end;

function TPasUsesUnit.ElementTypeName: string;
begin
  Result := SPasTreeUsesUnit;
end;

procedure TPasUsesUnit.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,Expr,false);
  ForEachChildCall(aMethodCall,Arg,InFilename,false);
  ForEachChildCall(aMethodCall,Arg,Module,true);
end;

{ TPasElementBase }

procedure TPasElementBase.Accept(Visitor: TPassTreeVisitor);
begin
  if Visitor=nil then ;
end;

{ TPasTypeRef }

procedure TPasTypeRef.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,RefType,true);
end;

{ TPasClassOperator }

function TPasClassOperator.TypeName: string;
begin
  Result:='class operator';
end;

function TPasClassOperator.GetProcTypeEnum: TProcType;
begin
  Result:=ptClassOperator;
end;

{ TPasImplAsmStatement }

constructor TPasImplAsmStatement.Create(const AName: string;
  AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  FTokens:=TStringList.Create;
end;

destructor TPasImplAsmStatement.Destroy;
begin
  FreeAndNil(FTokens);
  inherited Destroy;
end;

{ TPasClassConstructor }

function TPasClassConstructor.TypeName: string;
begin
  Result:='class '+ inherited TypeName;
end;

function TPasClassConstructor.GetProcTypeEnum: TProcType;
begin
  Result:=ptClassConstructor;
end;

{ TPasAnonymousProcedure }

function TPasAnonymousProcedure.ElementTypeName: string;
begin
  Result:=SPasTreeAnonymousProcedure;
end;

function TPasAnonymousProcedure.TypeName: string;
begin
  Result:='anonymous procedure';
end;

function TPasAnonymousProcedure.GetProcTypeEnum: TProcType;
begin
  Result:=ptAnonymousProcedure;
end;

{ TPasAnonymousFunction }

function TPasAnonymousFunction.GetFT: TPasFunctionType;
begin
  Result:=ProcType as TPasFunctionType;
end;

function TPasAnonymousFunction.ElementTypeName: string;
begin
  Result := SPasTreeAnonymousFunction;
end;

function TPasAnonymousFunction.TypeName: string;
begin
  Result:='anonymous function';
end;

function TPasAnonymousFunction.GetProcTypeEnum: TProcType;
begin
  Result:=ptAnonymousFunction;
end;

{ TProcedureExpr }

constructor TProcedureExpr.Create(AParent: TPasElement);
begin
  inherited Create(AParent,pekProcedure,eopNone);
end;

destructor TProcedureExpr.Destroy;
begin
  ReleaseAndNil(TPasElement(Proc){$IFDEF CheckPasTreeRefCount},'TProcedureExpr.Proc'{$ENDIF});
  inherited Destroy;
end;

function TProcedureExpr.GetDeclaration(full: Boolean): string;
begin
  if Proc<>nil then
    Result:=Proc.GetDeclaration(full)
  else
    Result:='procedure-expr';
end;

procedure TProcedureExpr.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,Proc,false);
end;

{ TPasImplRaise }

destructor TPasImplRaise.Destroy;
begin
  ReleaseAndNil(TPasElement(ExceptObject){$IFDEF CheckPasTreeRefCount},'TPasImplRaise.ExceptObject'{$ENDIF});
  ReleaseAndNil(TPasElement(ExceptAddr){$IFDEF CheckPasTreeRefCount},'TPasImplRaise.ExceptAddr'{$ENDIF});
  inherited Destroy;
end;

procedure TPasImplRaise.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,ExceptObject,false);
  ForEachChildCall(aMethodCall,Arg,ExceptAddr,false);
end;

{ TPasImplRepeatUntil }

destructor TPasImplRepeatUntil.Destroy;
begin
  ReleaseAndNil(TPasElement(ConditionExpr){$IFDEF CheckPasTreeRefCount},'TPasImplRepeatUntil.ConditionExpr'{$ENDIF});
  inherited Destroy;
end;

function TPasImplRepeatUntil.Condition: string;
begin
  If Assigned(ConditionExpr) then
    Result:=ConditionExpr.GetDeclaration(True)
  else
    Result:='';
end;

procedure TPasImplRepeatUntil.ForEachCall(
  const aMethodCall: TOnForEachPasElement; const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,ConditionExpr,false);
end;

{ TPasImplSimple }

destructor TPasImplSimple.Destroy;
begin
  ReleaseAndNil(TPasElement(Expr){$IFDEF CheckPasTreeRefCount},'TPasImplSimple.Expr'{$ENDIF});
  inherited Destroy;
end;

procedure TPasImplSimple.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,Expr,false);
end;

{ TPasImplAssign }

destructor TPasImplAssign.Destroy;
begin
  ReleaseAndNil(TPasElement(Left){$IFDEF CheckPasTreeRefCount},'TPasImplAssign.left'{$ENDIF});
  ReleaseAndNil(TPasElement(Right){$IFDEF CheckPasTreeRefCount},'TPasImplAssign.right'{$ENDIF});
  inherited Destroy;
end;

procedure TPasImplAssign.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,left,false);
  ForEachChildCall(aMethodCall,Arg,right,false);
end;

{ TPasExportSymbol }

destructor TPasExportSymbol.Destroy;
begin
  ReleaseAndNil(TPasElement(ExportName){$IFDEF CheckPasTreeRefCount},'TPasExportSymbol.ExportName'{$ENDIF});
  ReleaseAndNil(TPasElement(ExportIndex){$IFDEF CheckPasTreeRefCount},'TPasExportSymbol.ExportIndex'{$ENDIF});
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

procedure TPasExportSymbol.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,ExportName,false);
  ForEachChildCall(aMethodCall,Arg,ExportIndex,false);
end;

{ TPasUnresolvedUnitRef }

function TPasUnresolvedUnitRef.ElementTypeName: string;
begin
  Result:=SPasTreeUnit;
end;

{ TPasLibrary }

destructor TPasLibrary.Destroy;
begin
  ReleaseAndNil(TPasElement(LibrarySection){$IFDEF CheckPasTreeRefCount},'TPasLibrary.LibrarySection'{$ENDIF});
  inherited Destroy;
end;

function TPasLibrary.ElementTypeName: string;
begin
  Result:=inherited ElementTypeName;
end;

procedure TPasLibrary.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  ForEachChildCall(aMethodCall,Arg,LibrarySection,false);
  inherited ForEachCall(aMethodCall, Arg);
end;

procedure TPasLibrary.ReleaseUsedUnits;
begin
  if LibrarySection<>nil then
    LibrarySection.ReleaseUsedUnits;
  inherited ReleaseUsedUnits;
end;

{ TPasProgram }

destructor TPasProgram.Destroy;
begin
  {$IFDEF VerbosePasTreeMem}writeln('TPasProgram.Destroy ProgramSection');{$ENDIF}
  ReleaseAndNil(TPasElement(ProgramSection){$IFDEF CheckPasTreeRefCount},'TPasProgram.ProgramSection'{$ENDIF});
  {$IFDEF VerbosePasTreeMem}writeln('TPasProgram.Destroy inherited');{$ENDIF}
  inherited Destroy;
  {$IFDEF VerbosePasTreeMem}writeln('TPasProgram.Destroy END');{$ENDIF}
end;

function TPasProgram.ElementTypeName: string;
begin
  Result:=inherited ElementTypeName;
end;

procedure TPasProgram.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  ForEachChildCall(aMethodCall,Arg,ProgramSection,false);
  inherited ForEachCall(aMethodCall, Arg);
end;

procedure TPasProgram.ReleaseUsedUnits;
begin
  if ProgramSection<>nil then
    ProgramSection.ReleaseUsedUnits;
  inherited ReleaseUsedUnits;
end;

{ TPasUnitModule }

function TPasUnitModule.ElementTypeName: string;
begin
  Result:=SPasTreeUnit;
end;

{ Parse tree element type name functions }
function TPasElement.ElementTypeName: string; begin Result := SPasTreeElement end;

function TPasElement.HintsString: String;

Var
  H : TPasmemberHint;

begin
  Result:='';
  For H := Low(TPasmemberHint) to High(TPasMemberHint) do
    if H in Hints then
      begin
      If (Result<>'') then
        Result:=Result+'; ';
      Result:=Result+cPasMemberHint[h];
      end;
end;

function TPasDeclarations.ElementTypeName: string; begin Result := SPasTreeSection end;

procedure TPasDeclarations.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to Declarations.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Declarations[i]),false);
end;

function TPasModule.ElementTypeName: string; begin Result := SPasTreeModule end;
function TPasPackage.ElementTypeName: string; begin Result := SPasTreePackage end;

procedure TPasPackage.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to Modules.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasModule(Modules[i]),true);
end;

function TPasResString.ElementTypeName: string; begin Result := SPasTreeResString; end;

function TPasType.FixTypeDecl(aDecl: String): String;
begin
  Result:=aDecl;
  if (Name<>'') then
    Result:=Name+' = '+Result;
  ProcessHints(false,Result);
end;

function TPasType.ElementTypeName: string; begin Result := SPasTreeType; end;
function TPasPointerType.ElementTypeName: string; begin Result := SPasTreePointerType; end;
function TPasAliasType.ElementTypeName: string; begin Result := SPasTreeAliasType; end;
function TPasTypeAliasType.ElementTypeName: string; begin Result := SPasTreeTypeAliasType; end;
function TPasClassOfType.ElementTypeName: string; begin Result := SPasTreeClassOfType; end;
function TPasRangeType.ElementTypeName: string; begin Result := SPasTreeRangeType; end;
function TPasArrayType.ElementTypeName: string; begin Result := SPasTreeArrayType; end;
function TPasFileType.ElementTypeName: string; begin Result := SPasTreeFileType; end;
function TPasEnumValue.ElementTypeName: string; begin Result := SPasTreeEnumValue; end;

procedure TPasEnumValue.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,Value,false);
end;

destructor TPasEnumValue.Destroy;
begin
  ReleaseAndNil(TPasElement(Value){$IFDEF CheckPasTreeRefCount},'TPasEnumValue.Value'{$ENDIF});
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

procedure TPasResultElement.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,ResultType,true);
end;

procedure TPasResultElement.ClearTypeReferences(aType: TPasElement);
begin
  if ResultType=aType then
    ReleaseAndNil(TPasElement(ResultType){$IFDEF CheckPasTreeRefCount},'TPasResultElement.ResultType'{$ENDIF});
end;

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

function TPasFunction.ElementTypeName: string; begin Result := SPasTreeFunction; end;
function TPasClassProcedure.ElementTypeName: string; begin Result := SPasTreeClassProcedure; end;
function TPasClassConstructor.ElementTypeName: string; begin Result := SPasTreeClassConstructor; end;
function TPasClassDestructor.ElementTypeName: string; begin Result := SPasTreeClassDestructor; end;

function TPasClassDestructor.TypeName: string;
begin
  Result:='destructor';
end;

function TPasClassDestructor.GetProcTypeEnum: TProcType;
begin
  Result:=ptClassDestructor;
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
  Result := SPasTreeOperator
end;

function TPasConstructor.ElementTypeName: string; begin Result := SPasTreeConstructor end;
function TPasDestructor.ElementTypeName: string; begin Result := SPasTreeDestructor end;
function TPasProcedureImpl.ElementTypeName: string; begin Result := SPasTreeProcedureImpl end;
function TPasConstructorImpl.ElementTypeName: string; begin Result := SPasTreeConstructorImpl end;
function TPasDestructorImpl.ElementTypeName: string; begin Result := SPasTreeDestructorImpl end;
function TPasStringType.ElementTypeName: string; begin Result:=SPasStringType;end;


{ All other stuff: }

procedure TPasElement.ProcessHints(const ASemiColonPrefix: boolean; var AResult: string);
var
  S : String;
begin
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
end;

procedure TPasElement.SetParent(const AValue: TPasElement);
begin
  FParent:=AValue;
end;

{$IFDEF CheckPasTreeRefCount}
procedure TPasElement.ChangeRefId(const OldId, NewId: string);
var
  i: Integer;
begin
  i:=RefIds.IndexOf(OldId);
  if i<0 then
    begin
    {AllowWriteln}
    writeln('ERROR: TPasElement.ChangeRefId ',Name,':',ClassName,' Old="'+OldId+'" New="'+NewId+'" Old not found');
    writeln(RefIds.Text);
    {AllowWriteln-}
    raise Exception.Create('');
    end;
  RefIds.Delete(i);
  RefIds.Add(NewId);
end;
{$ENDIF}

constructor TPasElement.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create;
  FName := AName;
  FParent := AParent;
  {$ifdef pas2js}
  inc(FLastPasElementId);
  FPasElementId:=FLastPasElementId;
  //writeln('TPasElement.Create ',Name,':',ClassName,' ID=[',FPasElementId,']');
  {$endif}
  {$ifdef EnablePasTreeGlobalRefCount}
  Inc(FGlobalRefCount);
  {$endif}
  {$IFDEF CheckPasTreeRefCount}
  RefIds:=TStringList.Create;
  PrevRefEl:=LastRefEl;
  if LastRefEl<>nil then
    LastRefEl.NextRefEl:=Self
  else
    FirstRefEl:=Self;
  LastRefEl:=Self;
  {$ENDIF}
end;

destructor TPasElement.Destroy;
begin
  if (FRefCount>0) and (FRefCount<high(FRefCount)) then
    begin
    {$if defined(debugrefcount) or defined(VerbosePasTreeMem)}writeln('TPasElement.Destroy ',Name,':',ClassName);{$ENDIF}
    {$IFDEF CheckPasTreeRefCount}
    if (FRefCount>0) and (FRefCount<high(FRefCount)) then
      begin
      {AllowWriteln}
      writeln('TPasElement.Destroy ',Name,':',ClassName,' RefIds.Count=',RefIds.Count);
      writeln(RefIds.Text);
      {AllowWriteln-}
      end;
    FreeAndNil(RefIds);
    {$ENDIF}
    raise Exception.Create('');
    end;
  {$IFDEF CheckPasTreeRefCount}
  FreeAndNil(RefIds);
  // remove from global chain
  if FirstRefEl=Self then FirstRefEl:=NextRefEl;
  if LastRefEl=Self then LastRefEl:=PrevRefEl;
  if PrevRefEl<>nil then
    PrevRefEl.NextRefEl:=NextRefEl;
  if NextRefEl<>nil then
    NextRefEl.PrevRefEl:=PrevRefEl;
  PrevRefEl:=nil;
  NextRefEl:=nil;
  {$ENDIF}
  FParent:=nil;
  {$ifdef EnablePasTreeGlobalRefCount}
  Dec(FGlobalRefCount);
  {$endif}
  inherited Destroy;
end;

procedure TPasElement.AddRef{$IFDEF CheckPasTreeRefCount}(const aId: string){$ENDIF};
begin
  {$ifdef EnablePasTreeGlobalRefCount}
  Inc(FGlobalRefCount);
  {$endif}
  Inc(FRefCount);
  {$IFDEF CheckPasTreeRefCount}
  if SameText(aId,'CreateElement') and (RefIds.IndexOf('CreateElement')>=0) then
    begin
    {AllowWriteln}
    writeln('TPasElement.AddRef ',Name,':',ClassName,' RefCount=',RefCount,' RefIds={',RefIds.Text,'}');
    {AllowWriteln-}
    {$IFDEF HasPTDumpStack}
    PTDumpStack;
    {$ENDIF}
    Halt;
    end;
  RefIds.Add(aId);
  {$ENDIF}
end;

procedure TPasElement.Release{$IFDEF CheckPasTreeRefCount}(const aId: string){$ENDIF};
{$if defined(debugrefcount) or defined(VerbosePasTreeMem)}
Var
  Cn : String;
  {$endif}
{$IFDEF CheckPasTreeRefCount}
var i: integer;
{$ENDIF}
begin
  {$if defined(debugrefcount) or defined(VerbosePasTreeMem)}
  {AllowWriteln}
  CN:=ClassName+' '+Name;
  CN:=CN+' '+IntToStr(FRefCount);
  //If Assigned(Parent) then
  //  CN:=CN+' ('+Parent.ClassName+')';
  Writeln('TPasElement.Release : ',Cn);
  {AllowWriteln-}
  {$endif}
  {$IFDEF CheckPasTreeRefCount}
  i:=RefIds.IndexOf(aId);
  if i<0 then
    RefIds.Add('remove:'+aId)
  else
    RefIds.Delete(i);
  {$ENDIF}
  if FRefCount = 0 then
    begin
    FRefCount:=High(FRefCount);
    {$ifdef pas2js}
    Destroy;
    {$else}
    Free;
    {$endif}
    end
  else if FRefCount=High(FRefCount) then
    begin
    {$if defined(debugrefcount) or defined(VerbosePasTreeMem)}
    Writeln('TPasElement.Released OUCH: ',Cn);
    {$endif}
    {$if defined(VerbosePasResolver) or defined(VerbosePCUFiler)}
    Writeln('TPasElement.Released : ',ClassName,' ',Name);
    {$endif}
    raise Exception.Create('');
    end
  else
    begin
    Dec(FRefCount);
    {$ifdef EnablePasTreeGlobalRefCount}
    Dec(FGlobalRefCount);
    {$endif}
    end;
{$if defined(debugrefcount) or defined(VerbosePasTreeMem)}  Writeln('TPasElement.Released : ',Cn); {$endif}
end;

procedure TPasElement.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  aMethodCall(Self,Arg);
end;

procedure TPasElement.ForEachChildCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer; Child: TPasElement; CheckParent: boolean);
begin
  if (Child=nil) then exit;
  if CheckParent and (not Child.HasParent(Self)) then exit;
  Child.ForEachCall(aMethodCall,Arg);
end;

function TPasElement.FullPath: string;

var
  p: TPasElement;

begin
  Result := '';
  p := Parent;
  while Assigned(p) and not p.InheritsFrom(TPasDeclarations) do
  begin
    if (p.Name<>'') and (Not (p is TPasOverloadedProc)) then
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
    if (p.Name<>'') and (Not (p is TPasOverloadedProc)) then
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

procedure TPasElement.ClearTypeReferences(aType: TPasElement);
begin
  if aType=nil then ;
end;

function TPasElement.HasParent(aParent: TPasElement): boolean;
var
  El: TPasElement;
begin
  El:=Parent;
  while El<>nil do
    begin
    if El=aParent then exit(true);
    El:=El.Parent;
    end;
  Result:=false;
end;

constructor TPasDeclarations.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  Declarations := TFPList.Create;
  Attributes := TFPList.Create;
  Classes := TFPList.Create;
  Consts := TFPList.Create;
  ExportSymbols := TFPList.Create;
  Functions := TFPList.Create;
  Properties := TFPList.Create;
  ResStrings := TFPList.Create;
  Types := TFPList.Create;
  Variables := TFPList.Create;
end;

destructor TPasDeclarations.Destroy;
var
  i: Integer;
  Child: TPasElement;
begin
  {$IFDEF VerbosePasTreeMem}writeln('TPasDeclarations.Destroy START');{$ENDIF}
  FreeAndNil(Variables);
  FreeAndNil(Types);
  FreeAndNil(ResStrings);
  FreeAndNil(Properties);
  FreeAndNil(Functions);
  FreeAndNil(ExportSymbols);
  FreeAndNil(Consts);
  FreeAndNil(Classes);
  FreeAndNil(Attributes);
  {$IFDEF VerbosePasTreeMem}writeln('TPasDeclarations.Destroy Declarations');{$ENDIF}
  for i := 0 to Declarations.Count - 1 do
    begin
    Child:=TPasElement(Declarations[i]);
    Child.Parent:=nil;
    Child.Release{$IFDEF CheckPasTreeRefCount}('TPasDeclarations.Children'){$ENDIF};
    end;
  FreeAndNil(Declarations);

  {$IFDEF VerbosePasTreeMem}writeln('TPasDeclarations.Destroy inherited');{$ENDIF}
  inherited Destroy;
  {$IFDEF VerbosePasTreeMem}writeln('TPasDeclarations.Destroy END');{$ENDIF}
end;

destructor TPasModule.Destroy;
begin
  {$IFDEF VerbosePasTreeMem}writeln('TPasModule.Destroy ReleaseUsedUnits');{$ENDIF}
  ReleaseUsedUnits;
  {$IFDEF VerbosePasTreeMem}writeln('TPasModule.Destroy global directives');{$ENDIF}
  ReleaseAndNil(TPasElement(GlobalDirectivesSection){$IFDEF CheckPasTreeRefCount},'TPasModule.GlobalDirectivesSection'{$ENDIF});
  {$IFDEF VerbosePasTreeMem}writeln('TPasModule.Destroy interface');{$ENDIF}
  ReleaseAndNil(TPasElement(InterfaceSection){$IFDEF CheckPasTreeRefCount},'TPasModule.InterfaceSection'{$ENDIF});
  {$IFDEF VerbosePasTreeMem}writeln('TPasModule.Destroy implementation');{$ENDIF}
  ReleaseAndNil(TPasElement(ImplementationSection){$IFDEF CheckPasTreeRefCount},'TPasModule.ImplementationSection'{$ENDIF});
  {$IFDEF VerbosePasTreeMem}writeln('TPasModule.Destroy initialization');{$ENDIF}
  ReleaseAndNil(TPasElement(InitializationSection){$IFDEF CheckPasTreeRefCount},'TPasModule.InitializationSection'{$ENDIF});
  {$IFDEF VerbosePasTreeMem}writeln('TPasModule.Destroy finalization');{$ENDIF}
  ReleaseAndNil(TPasElement(FinalizationSection){$IFDEF CheckPasTreeRefCount},'TPasModule.FinalizationSection'{$ENDIF});
  {$IFDEF VerbosePasTreeMem}writeln('TPasModule.Destroy inherited');{$ENDIF}
  inherited Destroy;
  {$IFDEF VerbosePasTreeMem}writeln('TPasModule.Destroy END');{$ENDIF}
end;


constructor TPasPackage.Create(const AName: string; AParent: TPasElement);
begin
  if (Length(AName) > 0) and (AName[1] <> '#') then
    inherited Create('#' + AName, AParent)
  else
    inherited Create(AName, AParent);
  Modules := TFPList.Create;
end;

destructor TPasPackage.Destroy;
var
  i: Integer;
begin
  for i := 0 to Modules.Count - 1 do
    TPasModule(Modules[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasPackage.Modules'){$ENDIF};
  FreeAndNil(Modules);
  inherited Destroy;
end;

procedure TPasPointerType.SetParent(const AValue: TPasElement);
begin
  if (AValue=nil) and (Parent<>nil) and (DestType<>nil)
      and ((DestType.Parent=Parent) or (DestType=Self)) then
    begin
    // DestType in same type section can create a loop
    // -> break loop when type section is closed
    DestType.Release{$IFDEF CheckPasTreeRefCount}('TPasPointerType.DestType'){$ENDIF};
    DestType:=nil;
    end;
  inherited SetParent(AValue);
end;

destructor TPasPointerType.Destroy;
begin
  ReleaseAndNil(TPasElement(DestType){$IFDEF CheckPasTreeRefCount},'TPasPointerType.DestType'{$ENDIF});
  inherited Destroy;
end;

procedure TPasAliasType.SetParent(const AValue: TPasElement);
begin
  if (AValue=nil) and (Parent<>nil) and (DestType<>nil)
      and ((DestType.Parent=Parent) or (DestType=Self)) then
    begin
    // DestType in same type section can create a loop
    // -> break loop when type section is closed
    DestType.Release{$IFDEF CheckPasTreeRefCount}('TPasAliasType.DestType'){$ENDIF};
    DestType:=nil;
    end;
  inherited SetParent(AValue);
end;

destructor TPasAliasType.Destroy;
begin
  ReleaseAndNil(TPasElement(DestType){$IFDEF CheckPasTreeRefCount},'TPasAliasType.DestType'{$ENDIF});
  ReleaseAndNil(TPasElement(Expr){$IFDEF CheckPasTreeRefCount},'TPasAliasType.Expr'{$ENDIF});
  inherited Destroy;
end;

procedure TPasArrayType.SetParent(const AValue: TPasElement);
var
  CurArr: TPasArrayType;
begin
  if (AValue=nil) and (Parent<>nil) then
    begin
    // parent is cleared
    // -> clear all references to this array (releasing loops)
    CurArr:=Self;
    while CurArr.ElType is TPasArrayType do
      begin
      if CurArr.ElType=Self then
        begin
        ReleaseAndNil(TPasElement(CurArr.ElType){$IFDEF CheckPasTreeRefCount},'TPasClassType.AncestorType'{$ENDIF});
        break;
        end;
      CurArr:=TPasArrayType(CurArr.ElType);
      end;
    end;
  inherited SetParent(AValue);
end;

destructor TPasArrayType.Destroy;
var
  i: Integer;
begin
  for i:=0 to length(Ranges)-1 do
    Ranges[i].Release{$IFDEF CheckPasTreeRefCount}('TPasArrayType.Ranges'){$ENDIF};
  ReleaseAndNil(TPasElement(ElType){$IFDEF CheckPasTreeRefCount},'TPasArrayType.ElType'{$ENDIF});
  inherited Destroy;
end;

destructor TPasFileType.Destroy;
begin
  ReleaseAndNil(TPasElement(ElType){$IFDEF CheckPasTreeRefCount},'TPasFileType.ElType'{$ENDIF});
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
    TPasEnumValue(Values[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasEnumType.Values'){$ENDIF};
  FreeAndNil(Values);
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

procedure TPasEnumType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to Values.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasEnumValue(Values[i]),false);
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
    TPasElement(Values[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasVariant.Values'){$ENDIF};
  FreeAndNil(Values);
  ReleaseAndNil(TPasElement(Members){$IFDEF CheckPasTreeRefCount},'TPasVariant.Members'{$ENDIF});
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
    if Full then ;
    end;
end;

procedure TPasVariant.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to Values.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Values[i]),false);
  ForEachChildCall(aMethodCall,Arg,Members,false);
end;

{ TPasRecordType }

constructor TPasRecordType.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
end;

destructor TPasRecordType.Destroy;
var
  i: Integer;
begin
  ReleaseAndNil(TPasElement(VariantEl){$IFDEF CheckPasTreeRefCount},'TPasRecordType.VariantEl'{$ENDIF});

  if Assigned(Variants) then
  begin
    for i := 0 to Variants.Count - 1 do
      TPasVariant(Variants[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasRecordType.Variants'){$ENDIF};
    FreeAndNil(Variants);
  end;

  inherited Destroy;
end;

{ TPasClassType }

procedure TPasClassType.SetParent(const AValue: TPasElement);
begin
  if (AValue=nil) and (Parent<>nil) then
    begin
    // parent is cleared
    // -> clear all references to this class (releasing loops)
    if AncestorType=Self then
      ReleaseAndNil(TPasElement(AncestorType){$IFDEF CheckPasTreeRefCount},'TPasClassType.AncestorType'{$ENDIF});
    if HelperForType=Self then
      ReleaseAndNil(TPasElement(HelperForType){$IFDEF CheckPasTreeRefCount},'TPasClassType.HelperForType'{$ENDIF});
    end;
  inherited SetParent(AValue);
end;

constructor TPasClassType.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  IsShortDefinition := False;
  Modifiers := TStringList.Create;
  Interfaces:= TFPList.Create;
end;

destructor TPasClassType.Destroy;
var
  i: Integer;
begin
  for i := 0 to Interfaces.Count - 1 do
    TPasElement(Interfaces[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasClassType.Interfaces'){$ENDIF};
  FreeAndNil(Interfaces);
  ReleaseAndNil(TPasElement(AncestorType){$IFDEF CheckPasTreeRefCount},'TPasClassType.AncestorType'{$ENDIF});
  ReleaseAndNil(TPasElement(HelperForType){$IFDEF CheckPasTreeRefCount},'TPasClassType.HelperForType'{$ENDIF});
  ReleaseAndNil(TPasElement(GUIDExpr){$IFDEF CheckPasTreeRefCount},'TPasClassType.GUIDExpr'{$ENDIF});
  FreeAndNil(Modifiers);
  inherited Destroy;
end;

function TPasClassType.ElementTypeName: string;
begin
  case ObjKind of
    okObject: Result := SPasTreeObjectType;
    okClass: Result := SPasTreeClassType;
    okInterface: Result := SPasTreeInterfaceType;
    okClassHelper : Result:=SPasClassHelperType;
    okRecordHelper : Result:=SPasRecordHelperType;
    okTypeHelper : Result:=SPasTypeHelperType;
  else
    Result:='ObjKind('+IntToStr(ord(ObjKind))+')';
  end;
end;

procedure TPasClassType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);

  ForEachChildCall(aMethodCall,Arg,AncestorType,true);
  for i:=0 to Interfaces.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Interfaces[i]),true);
  ForEachChildCall(aMethodCall,Arg,HelperForType,true);
  ForEachChildCall(aMethodCall,Arg,GUIDExpr,false);
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

function TPasClassType.IsSealed: Boolean;
begin
  Result:=HasModifier('sealed');
end;

function TPasClassType.IsAbstract: Boolean;
begin
  Result:=HasModifier('abstract');
end;

function TPasClassType.HasModifier(const aModifier: String): Boolean;
var
  i: Integer;
begin
  for i:=0 to Modifiers.Count-1 do
    if CompareText(aModifier,Modifiers[i])=0 then
      exit(true);
  Result:=false;
end;

{ TPasArgument }

destructor TPasArgument.Destroy;
begin
  ReleaseAndNil(TPasElement(ArgType){$IFDEF CheckPasTreeRefCount},'TPasArgument.ArgType'{$ENDIF});
  ReleaseAndNil(TPasElement(ValueExpr){$IFDEF CheckPasTreeRefCount},'TPasArgument.ValueExpr'{$ENDIF});
  inherited Destroy;
end;

{ TPasProcedureType }

function TPasProcedureType.GetIsNested: Boolean;
begin
  Result:=ptmIsNested in Modifiers;
end;

function TPasProcedureType.GetIsOfObject: Boolean;
begin
  Result:=ptmOfObject in Modifiers;
end;

function TPasProcedureType.GetIsReference: Boolean;
begin
  Result:=ptmReferenceTo in Modifiers;
end;

procedure TPasProcedureType.SetIsNested(const AValue: Boolean);
begin
  if AValue then
    Include(Modifiers,ptmIsNested)
  else
    Exclude(Modifiers,ptmIsNested);
end;

procedure TPasProcedureType.SetIsOfObject(const AValue: Boolean);
begin
  if AValue then
    Include(Modifiers,ptmOfObject)
  else
    Exclude(Modifiers,ptmOfObject);
end;

procedure TPasProcedureType.SetIsReference(AValue: Boolean);
begin
  if AValue then
    Include(Modifiers,ptmReferenceTo)
  else
    Exclude(Modifiers,ptmReferenceTo);
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
    TPasArgument(Args[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasProcedureType.Args'){$ENDIF};
  FreeAndNil(Args);
  ReleaseAndNil(TPasElement(VarArgsType){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
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
  if AUnresolvedTypeName<>'' then
    Result.ArgType := TPasUnresolvedTypeRef.Create(AUnresolvedTypeName, Result);
end;

procedure TPasProcedureType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to Args.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Args[i]),false);
  ForEachChildCall(aMethodCall,Arg,VarArgsType,false);
end;

{ TPasResultElement }

destructor TPasResultElement.Destroy;
begin
  if Assigned(ResultType) then
    ReleaseAndNil(TPasElement(ResultType){$IFDEF CheckPasTreeRefCount},'TPasResultElement.ResultType'{$ENDIF});
  inherited Destroy;
end;

destructor TPasFunctionType.Destroy;
begin
  ReleaseAndNil(TPasElement(ResultEl){$IFDEF CheckPasTreeRefCount},'TPasFunctionType.ResultEl'{$ENDIF});
  inherited Destroy;
end;


class function TPasFunctionType.TypeName: string;
begin
  Result := 'function';
end;


constructor TPasUnresolvedTypeRef.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, nil);
  if AParent=nil then ;
end;


destructor TPasVariable.Destroy;
begin
//  FreeAndNil(Expr);
  { Attention, in derived classes, VarType isn't necessarily set!
    (e.g. in Constants) }
  ReleaseAndNil(TPasElement(VarType){$IFDEF CheckPasTreeRefCount},'TPasVariable.VarType'{$ENDIF});
  ReleaseAndNil(TPasElement(Expr){$IFDEF CheckPasTreeRefCount},'TPasVariable.Expr'{$ENDIF});
  ReleaseAndNil(TPasElement(LibraryName){$IFDEF CheckPasTreeRefCount},'TPasVariable.LibraryName'{$ENDIF});
  ReleaseAndNil(TPasElement(ExportName){$IFDEF CheckPasTreeRefCount},'TPasVariable.ExportName'{$ENDIF});
  ReleaseAndNil(TPasElement(AbsoluteExpr){$IFDEF CheckPasTreeRefCount},'TPasVariable.AbsoluteExpr'{$ENDIF});
  inherited Destroy;
end;

function TPasProperty.GetIsClass: boolean;
begin
  Result:=vmClass in VarModifiers;
end;

procedure TPasProperty.SetIsClass(AValue: boolean);
begin
   if AValue then
    Include(VarModifiers,vmClass)
  else
    Exclude(VarModifiers,vmClass);
end;

constructor TPasProperty.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  FArgs := TFPList.Create;
end;

destructor TPasProperty.Destroy;
var
  i: Integer;
begin
  for i := 0 to Args.Count - 1 do
    TPasArgument(Args[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasProperty.Args'){$ENDIF};
  FreeAndNil(FArgs);
  ReleaseAndNil(TPasElement(IndexExpr){$IFDEF CheckPasTreeRefCount},'TPasProperty.IndexExpr'{$ENDIF});
  ReleaseAndNil(TPasElement(ReadAccessor){$IFDEF CheckPasTreeRefCount},'TPasProperty.ReadAccessor'{$ENDIF});
  ReleaseAndNil(TPasElement(WriteAccessor){$IFDEF CheckPasTreeRefCount},'TPasProperty.WriteAccessor'{$ENDIF});
  for i := 0 to length(Implements) - 1 do
    TPasExpr(Implements[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasProperty.Implements'){$ENDIF};
  SetLength(Implements,0);
  ReleaseAndNil(TPasElement(StoredAccessor){$IFDEF CheckPasTreeRefCount},'TPasProperty.StoredAccessor'{$ENDIF});
  ReleaseAndNil(TPasElement(DefaultExpr){$IFDEF CheckPasTreeRefCount},'TPasProperty.DefaultExpr'{$ENDIF});
  ReleaseAndNil(TPasElement(DispIDExpr){$IFDEF CheckPasTreeRefCount},'TPasProperty.DispIDExpr'{$ENDIF});
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
    TPasProcedure(Overloads[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasOverloadedProc.Overloads'){$ENDIF};
  FreeAndNil(Overloads);
  inherited Destroy;
end;

function TPasOverloadedProc.TypeName: string;
begin
  if Assigned(TPasProcedure(Overloads[0]).ProcType) then
    Result := TPasProcedure(Overloads[0]).ProcType.TypeName
  else
    SetLength(Result, 0);
end;

procedure TPasOverloadedProc.ForEachCall(
  const aMethodCall: TOnForEachPasElement; const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to Overloads.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasProcedure(Overloads[i]),false);
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
  ReleaseAndNil(TPasElement(PublicName){$IFDEF CheckPasTreeRefCount},'TPasProcedure.PublicName'{$ENDIF});
  ReleaseAndNil(TPasElement(LibraryExpr){$IFDEF CheckPasTreeRefCount},'TPasProcedure.LibraryExpr'{$ENDIF});
  ReleaseAndNil(TPasElement(LibrarySymbolName){$IFDEF CheckPasTreeRefCount},'TPasProcedure.LibrarySymbolName'{$ENDIF});
  ReleaseAndNil(TPasElement(DispIDExpr){$IFDEF CheckPasTreeRefCount},'TPasProcedure.DispIDExpr'{$ENDIF});
  ReleaseAndNil(TPasElement(MessageExpr){$IFDEF CheckPasTreeRefCount},'TPasProcedure.MessageExpr'{$ENDIF});
  ReleaseAndNil(TPasElement(ProcType){$IFDEF CheckPasTreeRefCount},'TPasProcedure.ProcType'{$ENDIF});
  ReleaseAndNil(TPasElement(Body){$IFDEF CheckPasTreeRefCount},'TPasProcedure.Body'{$ENDIF});
  ReleaseProcNameParts(NameParts);
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
  ReleaseAndNil(TPasElement(Body){$IFDEF CheckPasTreeRefCount},'TPasProcedureImpl.Body'{$ENDIF});

  for i := 0 to Locals.Count - 1 do
    TPasElement(Locals[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasProcedureImpl.Locals'){$ENDIF};
  FreeAndNil(Locals);

  ReleaseAndNil(TPasElement(ProcType){$IFDEF CheckPasTreeRefCount},'TPasProcedureImpl.ProcType'{$ENDIF});

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
  FreeAndNil(Commands);
  inherited Destroy;
end;


destructor TPasImplIfElse.Destroy;
begin
  ReleaseAndNil(TPasElement(ConditionExpr){$IFDEF CheckPasTreeRefCount},'TPasImplIfElse.ConditionExpr'{$ENDIF});
  ReleaseAndNil(TPasElement(IfBranch){$IFDEF CheckPasTreeRefCount},'TPasImplIfElse.IfBranch'{$ENDIF});
  ReleaseAndNil(TPasElement(ElseBranch){$IFDEF CheckPasTreeRefCount},'TPasImplIfElse.ElseBranch'{$ENDIF});
  inherited Destroy;
end;

procedure TPasImplIfElse.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if IfBranch=nil then
    begin
    IfBranch:=Element;
    Element.AddRef{$IFDEF CheckPasTreeRefCount}('TPasImplIfElse.IfBranch'){$ENDIF};
    end
  else if ElseBranch=nil then
    begin
    ElseBranch:=Element;
    Element.AddRef{$IFDEF CheckPasTreeRefCount}('TPasImplIfElse.ElseBranch'){$ENDIF};
    end
  else
    raise Exception.Create('TPasImplIfElse.AddElement if and else already set - please report this bug');
end;

function TPasImplIfElse.CloseOnSemicolon: boolean;
begin
  Result:=ElseBranch<>nil;
end;

procedure TPasImplIfElse.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  ForEachChildCall(aMethodCall,Arg,ConditionExpr,false);
  if Elements.IndexOf(IfBranch)<0 then
    ForEachChildCall(aMethodCall,Arg,IfBranch,false);
  if Elements.IndexOf(ElseBranch)<0 then
    ForEachChildCall(aMethodCall,Arg,ElseBranch,false);
  inherited ForEachCall(aMethodCall, Arg);
end;

function TPasImplIfElse.Condition: string;
begin
  If Assigned(ConditionExpr) then
    Result:=ConditionExpr.GetDeclaration(True);
end;

destructor TPasImplForLoop.Destroy;
begin
  ReleaseAndNil(TPasElement(VariableName){$IFDEF CheckPasTreeRefCount},'TPasImplForLoop.VariableName'{$ENDIF});
  ReleaseAndNil(TPasElement(StartExpr){$IFDEF CheckPasTreeRefCount},'TPasImplForLoop.StartExpr'{$ENDIF});
  ReleaseAndNil(TPasElement(EndExpr){$IFDEF CheckPasTreeRefCount},'TPasImplForLoop.EndExpr'{$ENDIF});
  ReleaseAndNil(TPasElement(Variable){$IFDEF CheckPasTreeRefCount},'TPasImplForLoop.Variable'{$ENDIF});
  ReleaseAndNil(TPasElement(Body){$IFDEF CheckPasTreeRefCount},'TPasImplForLoop.Body'{$ENDIF});
  inherited Destroy;
end;

procedure TPasImplForLoop.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    begin
    Body:=Element;
    Body.AddRef{$IFDEF CheckPasTreeRefCount}('TPasImplForLoop.Body'){$ENDIF};
    end
  else
    raise Exception.Create('TPasImplForLoop.AddElement body already set - please report this bug');
end;

procedure TPasImplForLoop.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  ForEachChildCall(aMethodCall,Arg,VariableName,false);
  ForEachChildCall(aMethodCall,Arg,Variable,false);
  ForEachChildCall(aMethodCall,Arg,StartExpr,false);
  ForEachChildCall(aMethodCall,Arg,EndExpr,false);
  if Elements.IndexOf(Body)<0 then
    ForEachChildCall(aMethodCall,Arg,Body,false);
  inherited ForEachCall(aMethodCall, Arg);
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
    TPasImplElement(Elements[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasImplBlock.Elements'){$ENDIF};
  FreeAndNil(Elements);
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

function TPasImplBlock.AddForLoop(AVarName: TPasExpr; AStartValue,
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

function TPasImplBlock.AddExceptOn(const VarName, TypeName: string
  ): TPasImplExceptOn;
begin
  Result:=AddExceptOn(VarName,TPasUnresolvedTypeRef.Create(TypeName,nil));
end;

function TPasImplBlock.AddExceptOn(const VarName: string; VarType: TPasType
  ): TPasImplExceptOn;
var
  V: TPasVariable;
begin
  V:=TPasVariable.Create(VarName,nil);
  V.VarType:=VarType;
  Result:=AddExceptOn(V);
end;

function TPasImplBlock.AddExceptOn(const VarEl: TPasVariable): TPasImplExceptOn;
begin
  Result:=TPasImplExceptOn.Create('',Self);
  Result.VarEl:=VarEl;
  Result.TypeEl:=VarEl.VarType;
  Result.TypeEl.AddRef{$IFDEF CheckPasTreeRefCount}('TPasImplExceptOn.TypeEl'){$ENDIF};
  AddElement(Result);
end;

function TPasImplBlock.AddExceptOn(const TypeEl: TPasType): TPasImplExceptOn;
begin
  Result:=TPasImplExceptOn.Create('',Self);
  Result.TypeEl:=TypeEl;
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
  Result.Expr:=exp;
  AddElement(Result);
end;

function TPasImplBlock.CloseOnSemicolon: boolean;
begin
  Result:=false;
end;

procedure TPasImplBlock.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to Elements.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Elements[i]),false);
end;



{ ---------------------------------------------------------------------

  ---------------------------------------------------------------------}

function TPasModule.GetDeclaration(full : boolean): string;
begin
  Result := 'Unit ' + Name;
  if full then ;
end;

procedure TPasModule.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,InterfaceSection,false);
  ForEachChildCall(aMethodCall,Arg,ImplementationSection,false);
  ForEachChildCall(aMethodCall,Arg,InitializationSection,false);
  ForEachChildCall(aMethodCall,Arg,FinalizationSection,false);
end;

procedure TPasModule.ReleaseUsedUnits;
begin
  if InterfaceSection<>nil then
    InterfaceSection.ReleaseUsedUnits;
  if ImplementationSection<>nil then
    ImplementationSection.ReleaseUsedUnits;
end;

function TPasResString.GetDeclaration(full: Boolean): string;
begin
  Result:=Expr.GetDeclaration(true);
  If Full Then
    begin
    Result:=Name+' = '+Result;
    ProcessHints(False,Result);
    end;
end;

procedure TPasResString.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,Expr,false);
end;

destructor TPasResString.Destroy;
begin
  ReleaseAndNil(TPasElement(Expr){$IFDEF CheckPasTreeRefCount},'TPasResString.Expr'{$ENDIF});
  inherited Destroy;
end;

function TPasPointerType.GetDeclaration(full: Boolean): string;
begin
  Result:='^'+DestType.Name;
  If Full then
    begin
    Result:=Name+' = '+Result;
    ProcessHints(False,Result);
    end;
end;

procedure TPasPointerType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,DestType,true);
end;

procedure TPasPointerType.ClearTypeReferences(aType: TPasElement);
begin
  if DestType=aType then
    ReleaseAndNil(TPasElement(DestType){$IFDEF CheckPasTreeRefCount},'TPasPointerType.DestType'{$ENDIF});
end;

function TPasAliasType.GetDeclaration(full: Boolean): string;
begin
  Result:=DestType.Name;
  If Full then
    Result:=FixTypeDecl(Result);
end;

procedure TPasAliasType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,DestType,true);
  ForEachChildCall(aMethodCall,Arg,Expr,false);
end;

procedure TPasAliasType.ClearTypeReferences(aType: TPasElement);
begin
  if DestType=aType then
    ReleaseAndNil(TPasElement(DestType){$IFDEF CheckPasTreeRefCount},'TPasAliasType.DestType'{$ENDIF});
end;

function TPasClassOfType.GetDeclaration (full : boolean) : string;
begin
  Result:='Class of '+DestType.Name;
  If Full then
    Result:=FixTypeDecl(Result);
end;

function TPasRangeType.GetDeclaration (full : boolean) : string;
begin
  Result:=RangeStart+'..'+RangeEnd;
  If Full then
    Result:=FixTypeDecl(Result);
end;

procedure TPasRangeType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,RangeExpr,false);
end;

destructor TPasRangeType.Destroy;
begin
  ReleaseAndNil(TPasElement(RangeExpr){$IFDEF CheckPasTreeRefCount},'TPasRangeType.RangeExpr'{$ENDIF});
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
  if Full then
    begin
    if GenericTemplateTypes<>nil then
      Result:=Result+GenericTemplateTypesAsString(GenericTemplateTypes)+' = '+Result
    else
      Result:=Result+' = '+Result;
    end;
  If (IndexRange<>'') then
    Result:=Result+'['+IndexRange+']';
  Result:=Result+' of ';
  If IsPacked then
    Result := 'packed '+Result;      // 12/04/04 Dave - Added
  If Assigned(Eltype) then
    Result:=Result+ElType.Name
  else
    Result:=Result+'const';
end;

function TPasArrayType.IsGenericArray: Boolean;
begin
  Result:=GenericTemplateTypes<>nil;
end;

function TPasArrayType.IsPacked: Boolean;
begin
  Result:=PackMode=pmPacked;
end;

procedure TPasArrayType.AddRange(Range: TPasExpr);
var
  i: Integer;
begin
  i:=Length(Ranges);
  SetLength(Ranges, i+1);
  Ranges[i]:=Range;
end;

function TPasFileType.GetDeclaration (full : boolean) : string;
begin
  Result:='File';
  If Assigned(Eltype) then
    Result:=Result+' of '+ElType.Name;
  If Full Then
    Result:=FixTypeDecl(Result);
end;

procedure TPasFileType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,ElType,true);
end;

function TPasEnumType.GetDeclaration (full : boolean) : string;

Var
  S : TStringList;

begin
  S:=TStringList.Create;
  Try
    If Full and (Name<>'') then
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

destructor TPasSetType.Destroy;
begin
  ReleaseAndNil(TPasElement(EnumType){$IFDEF CheckPasTreeRefCount},'TPasSetType.EnumType'{$ENDIF});
  inherited Destroy;
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
      If Full and (Name<>'') then
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

procedure TPasSetType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,EnumType,true);
end;

{ TPasMembersType }

constructor TPasMembersType.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  PackMode:=pmNone;
  Members := TFPList.Create;
  GenericTemplateTypes:=TFPList.Create;
end;

destructor TPasMembersType.Destroy;
var
  i: Integer;
  El: TPasElement;
begin
  for i := 0 to Members.Count - 1 do
    begin
    El:=TPasElement(Members[i]);
    El.Parent:=nil;
    El.Release{$IFDEF CheckPasTreeRefCount}('TPasMembersType.Members'){$ENDIF};
    end;
  FreeAndNil(Members);

  ReleaseGenericTemplateTypes(GenericTemplateTypes
    {$IFDEF CheckPasTreeRefCount},'TPasMembersType.GenericTemplateTypes'{$ENDIF});

  inherited Destroy;
end;

function TPasMembersType.IsPacked: Boolean;
begin
  Result:=(PackMode <> pmNone);
end;

function TPasMembersType.IsBitPacked: Boolean;
begin
  Result:=(PackMode=pmBitPacked)
end;

procedure TPasMembersType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to Members.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Members[i]),false);
end;

{ TPasRecordType }

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
    if (VariantEl is TPasVariable) then
      temp:=Temp+VariantEl.Name+' : '+TPasVariable(VariantEl).VarType.Name
    else if (VariantEl<>Nil) then
      temp:=temp+VariantEl.Name;
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
    If Full and (Name<>'') then
      begin
      if GenericTemplateTypes.Count>0 then
        Temp:=Name+GenericTemplateTypesAsString(GenericTemplateTypes)+' = '+Temp
      else
        Temp:=Name+' = '+Temp;
      end;
    S.Add(Temp);
    GetMembers(S);
    S.Add('end');
    Result:=S.Text;
    if Full then
      ProcessHints(False, Result);
  finally
    S.free;
  end;
end;

procedure TPasRecordType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,VariantEl,true);
  if Variants<>nil then
    for i:=0 to Variants.Count-1 do
      ForEachChildCall(aMethodCall,Arg,TPasElement(Variants[i]),false);
end;

function TPasRecordType.IsAdvancedRecord: Boolean;

Var
  I : Integer;
  Member: TPasElement;

begin
  Result:=False;
  I:=0;
  While (Not Result) and (I<Members.Count) do
    begin
    Member:=TPasElement(Members[i]);
    if (Member.Visibility<>visPublic) then exit(true);
    if (Member.ClassType<>TPasVariable) then exit(true);
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
      List.Add(T+'; ')
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

function TPasFunctionType.GetDeclaration(Full: boolean): string;

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

procedure TPasFunctionType.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,ResultEl,false);
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

procedure TPasVariable.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,VarType,true);
  ForEachChildCall(aMethodCall,Arg,Expr,false);
  ForEachChildCall(aMethodCall,Arg,LibraryName,false);
  ForEachChildCall(aMethodCall,Arg,ExportName,false);
  ForEachChildCall(aMethodCall,Arg,AbsoluteExpr,false);
end;

procedure TPasVariable.ClearTypeReferences(aType: TPasElement);
begin
  if VarType=aType then
    ReleaseAndNil(TPasElement(VarType){$IFDEF CheckPasTreeRefCount},'TPasVariable.VarType'{$ENDIF});
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

procedure TPasProperty.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,IndexExpr,false);
  for i:=0 to Args.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Args[i]),false);
  ForEachChildCall(aMethodCall,Arg,ReadAccessor,false);
  ForEachChildCall(aMethodCall,Arg,WriteAccessor,false);
  for i:=0 to length(Implements)-1 do
    ForEachChildCall(aMethodCall,Arg,Implements[i],false);
  ForEachChildCall(aMethodCall,Arg,StoredAccessor,false);
  ForEachChildCall(aMethodCall,Arg,DefaultExpr,false);
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

procedure TPasProcedure.GetModifiers(List: TStrings);

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

procedure TPasProcedure.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i, j: Integer;
  Templates: TFPList;
begin
  inherited ForEachCall(aMethodCall, Arg);
  if NameParts<>nil then
    for i:=0 to NameParts.Count-1 do
      begin
      Templates:=TProcedureNamePart(NameParts[i]).Templates;
      if Templates<>nil then
        for j:=0 to Templates.Count-1 do
          ForEachChildCall(aMethodCall,Arg,TPasElement(Templates[j]),false);
      end;
  ForEachChildCall(aMethodCall,Arg,ProcType,false);
  ForEachChildCall(aMethodCall,Arg,PublicName,false);
  ForEachChildCall(aMethodCall,Arg,LibraryExpr,false);
  ForEachChildCall(aMethodCall,Arg,LibrarySymbolName,false);
  ForEachChildCall(aMethodCall,Arg,MessageExpr,false);
  ForEachChildCall(aMethodCall,Arg,Body,false);
end;

procedure TPasProcedure.AddModifier(AModifier: TProcedureModifier);
begin
  Include(FModifiers,AModifier);
end;

function TPasProcedure.IsVirtual: Boolean;
begin
  Result:=pmVirtual in FModifiers;
end;

function TPasProcedure.IsDynamic: Boolean;
begin
  Result:=pmDynamic in FModifiers;
end;

function TPasProcedure.IsAbstract: Boolean;
begin
  Result:=pmAbstract in FModifiers;
end;

function TPasProcedure.IsOverride: Boolean;
begin
  Result:=pmOverride in FModifiers;
end;

function TPasProcedure.IsExported: Boolean;
begin
  Result:=pmExport in FModifiers;
end;

function TPasProcedure.IsExternal: Boolean;
begin
  Result:=pmExternal in FModifiers;
end;

function TPasProcedure.IsOverload: Boolean;
begin
  Result:=pmOverload in FModifiers;
end;

function TPasProcedure.IsMessage: Boolean;
begin
  Result:=pmMessage in FModifiers;
end;

function TPasProcedure.IsReintroduced: Boolean;
begin
  Result:=pmReintroduce in FModifiers;
end;

function TPasProcedure.IsStatic: Boolean;

begin
  Result:=ptmStatic in ProcType.Modifiers;
end;

function TPasProcedure.IsForward: Boolean;
begin
  Result:=pmForward in FModifiers;
end;

function TPasProcedure.GetProcTypeEnum: TProcType;
begin
  Result:=ptProcedure;
end;

procedure TPasProcedure.SetNameParts(Parts: TProcedureNameParts);
var
  i, j: Integer;
  El: TPasElement;
begin
  if NameParts<>nil then
    ReleaseProcNameParts(NameParts);
  NameParts:=TFPList.Create;
  NameParts.Assign(Parts);
  Parts.Clear;
  for i:=0 to NameParts.Count-1 do
    with TProcedureNamePart(NameParts[i]) do
      if Templates<>nil then
        for j:=0 to Templates.Count-1 do
          begin
          El:=TPasElement(Templates[j]);
          El.Parent:=Self;
          end;
end;

function TPasProcedure.GetDeclaration(full: Boolean): string;
Var
  S : TStringList;
  T: String;
  i: Integer;
begin
  S:=TStringList.Create;
  try
    If Full then
      begin
      T:=TypeName;
      if NameParts<>nil then
        begin
        T:=T+' ';
        for i:=0 to NameParts.Count-1 do
          begin
          if i>0 then
            T:=T+'.';
          with TProcedureNamePart(NameParts[i]) do
            begin
            T:=T+Name;
            if Templates<>nil then
              T:=T+GenericTemplateTypesAsString(Templates);
            end;
          end;
        end
      else if Name<>'' then
        T:=T+' '+Name;
      S.Add(T);
      end;
    ProcType.GetArguments(S);
    If (ProcType is TPasFunctionType)
        and Assigned(TPasFunctionType(Proctype).ResultEl) then
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

function TPasFunction.GetProcTypeEnum: TProcType;
begin
  Result:=ptFunction;
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

function TPasOperator.GetProcTypeEnum: TProcType;
begin
  Result:=ptOperator;
end;

function TPasClassProcedure.TypeName: string;
begin
  Result:='class procedure';
end;

function TPasClassProcedure.GetProcTypeEnum: TProcType;
begin
  Result:=ptClassProcedure;
end;

function TPasClassFunction.TypeName: string;
begin
  Result:='class function';
end;

function TPasClassFunction.GetProcTypeEnum: TProcType;
begin
  Result:=ptClassFunction;
end;

function TPasConstructor.TypeName: string;
begin
  Result:='constructor';
end;

function TPasConstructor.GetProcTypeEnum: TProcType;
begin
  Result:=ptConstructor;
end;

function TPasDestructor.TypeName: string;
begin
  Result:='destructor';
end;

function TPasDestructor.GetProcTypeEnum: TProcType;
begin
  Result:=ptDestructor;
end;

function TPasArgument.GetDeclaration (full : boolean) : string;
begin
  If Assigned(ArgType) then
    begin
    If ArgType.Name<>'' then
      Result:=ArgType.Name
    else
      Result:=ArgType.GetDeclaration(False);
    If Full and (Name<>'') then
      Result:=Name+': '+Result;
    end
  else If Full then
    Result:=Name
  else
    Result:='';
end;

procedure TPasArgument.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,ArgType,true);
  ForEachChildCall(aMethodCall,Arg,ValueExpr,false);
end;

procedure TPasArgument.ClearTypeReferences(aType: TPasElement);
begin
  if ArgType=aType then
    ReleaseAndNil(TPasElement(ArgType){$IFDEF CheckPasTreeRefCount},'TPasArgument.ArgType'{$ENDIF});
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
  if Obj=nil then ;
end;

{ TPasSection }

constructor TPasSection.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
  UsesList := TFPList.Create;
end;

destructor TPasSection.Destroy;
begin
  ReleaseUsedUnits;
  FreeAndNil(UsesList);

  {$IFDEF VerbosePasTreeMem}writeln('TPasSection.Destroy inherited');{$ENDIF}
  inherited Destroy;
  {$IFDEF VerbosePasTreeMem}writeln('TPasSection.Destroy END');{$ENDIF}
end;

function TPasSection.AddUnitToUsesList(const AUnitName: string;
  aName: TPasExpr; InFilename: TPrimitiveExpr; aModule: TPasElement;
  UsesUnit: TPasUsesUnit): TPasUsesUnit;
var
  l: Integer;
begin
  if (InFilename<>nil) and (InFilename.Kind<>pekString) then
    raise Exception.Create('');
  if aModule=nil then
    aModule:=TPasUnresolvedUnitRef.Create(AUnitName, Self);
  l:=length(UsesClause);
  SetLength(UsesClause,l+1);
  if UsesUnit=nil then
    begin
    UsesUnit:=TPasUsesUnit.Create(AUnitName,Self);
    if aName<>nil then
      begin
      Result.SourceFilename:=aName.SourceFilename;
      Result.SourceLinenumber:=aName.SourceLinenumber;
      end;
    end;
  UsesClause[l]:=UsesUnit;
  UsesUnit.Expr:=aName;
  UsesUnit.InFilename:=InFilename;
  UsesUnit.Module:=aModule;
  Result:=UsesUnit;

  UsesList.Add(aModule);
  aModule.AddRef{$IFDEF CheckPasTreeRefCount}('TPasSection.UsesList'){$ENDIF};
end;

function TPasSection.ElementTypeName: string;
begin
  Result := SPasTreeSection;
end;

procedure TPasSection.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to length(UsesClause)-1 do
    ForEachChildCall(aMethodCall,Arg,UsesClause[i],false);
end;

procedure TPasSection.ReleaseUsedUnits;
var
  i: Integer;
begin
  {$IFDEF VerbosePasTreeMem}writeln('TPasSection.Destroy UsesList');{$ENDIF}
  for i := 0 to UsesList.Count - 1 do
    TPasType(UsesList[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasSection.UsesList'){$ENDIF};
  UsesList.Clear;
  {$IFDEF VerbosePasTreeMem}writeln('TPasSection.Destroy UsesClause');{$ENDIF}
  for i := 0 to length(UsesClause) - 1 do
    UsesClause[i].Release{$IFDEF CheckPasTreeRefCount}('TPasSection.UsesClause'){$ENDIF};
  SetLength(UsesClause,0);

  PendingUsedIntf:=nil; // not release
end;

{ TProcedureBody }

constructor TProcedureBody.Create(const AName: string; AParent: TPasElement);
begin
  inherited Create(AName, AParent);
end;

destructor TProcedureBody.Destroy;
begin
  ReleaseAndNil(TPasElement(Body){$IFDEF CheckPasTreeRefCount},'TProcedureBody.Body'{$ENDIF});
  inherited Destroy;
end;

procedure TProcedureBody.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,Body,false);
end;

{ TPasImplWhileDo }

destructor TPasImplWhileDo.Destroy;
begin
  ReleaseAndNil(TPasElement(ConditionExpr){$IFDEF CheckPasTreeRefCount},'TPasImplWhileDo.ConditionExpr'{$ENDIF});
  ReleaseAndNil(TPasElement(Body){$IFDEF CheckPasTreeRefCount},'TPasImplWhileDo.Body'{$ENDIF});
  inherited Destroy;
end;

procedure TPasImplWhileDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    begin
    Body:=Element;
    Body.AddRef{$IFDEF CheckPasTreeRefCount}('TPasImplWhileDo.Body'){$ENDIF};
    end
  else
    raise Exception.Create('TPasImplWhileDo.AddElement body already set');
end;

procedure TPasImplWhileDo.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  ForEachChildCall(aMethodCall,Arg,ConditionExpr,false);
  if Elements.IndexOf(Body)<0 then
    ForEachChildCall(aMethodCall,Arg,Body,false);
  inherited ForEachCall(aMethodCall, Arg);
end;

function TPasImplWhileDo.Condition: string;
begin
  If Assigned(ConditionExpr) then
    Result:=ConditionExpr.GetDeclaration(True);
end;

{ TPasImplCaseOf }

destructor TPasImplCaseOf.Destroy;
begin
  ReleaseAndNil(TPasElement(CaseExpr){$IFDEF CheckPasTreeRefCount},'TPasImplCaseOf.CaseExpr'{$ENDIF});
  ReleaseAndNil(TPasElement(ElseBranch){$IFDEF CheckPasTreeRefCount},'TPasImplCaseOf.ElseBranch'{$ENDIF});
  inherited Destroy;
end;

procedure TPasImplCaseOf.AddElement(Element: TPasImplElement);
begin
  if (ElseBranch<>Nil) and (Element=ElseBranch) then
    ElseBranch.AddRef{$IFDEF CheckPasTreeRefCount}('TPasImplCaseOf.ElseBranch'){$ENDIF};
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

procedure TPasImplCaseOf.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  ForEachChildCall(aMethodCall,Arg,CaseExpr,false);
  if Elements.IndexOf(ElseBranch)<0 then
    ForEachChildCall(aMethodCall,Arg,ElseBranch,false);
  inherited ForEachCall(aMethodCall, Arg);
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
    TPasExpr(Expressions[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasImplCaseStatement.CaseExpr'){$ENDIF};
  FreeAndNil(Expressions);
  ReleaseAndNil(TPasElement(Body){$IFDEF CheckPasTreeRefCount},'TPasImplCaseStatement.Body'{$ENDIF});
  inherited Destroy;
end;

procedure TPasImplCaseStatement.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    begin
    Body:=Element;
    Body.AddRef{$IFDEF CheckPasTreeRefCount}('TPasImplCaseStatement.Body'){$ENDIF};
    end
  else
    raise Exception.Create('TPasImplCaseStatement.AddElement body already set');
end;

procedure TPasImplCaseStatement.AddExpression(const Expr: TPasExpr);
begin
  Expressions.Add(Expr);
  Expr.Parent:=Self;
end;

procedure TPasImplCaseStatement.ForEachCall(
  const aMethodCall: TOnForEachPasElement; const Arg: Pointer);
var
  i: Integer;
begin
  for i:=0 to Expressions.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Expressions[i]),false);
  if Elements.IndexOf(Body)<0 then
    ForEachChildCall(aMethodCall,Arg,Body,false);
  inherited ForEachCall(aMethodCall, Arg);
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
  ReleaseAndNil(TPasElement(Body){$IFDEF CheckPasTreeRefCount},'TPasImplWithDo.Body'{$ENDIF});
  For I:=0 to Expressions.Count-1 do
    TPasExpr(Expressions[i]).Release{$IFDEF CheckPasTreeRefCount}('TPasImplWithDo.Expressions'){$ENDIF};
  FreeAndNil(Expressions);
  inherited Destroy;
end;

procedure TPasImplWithDo.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    begin
    Body:=Element;
    Body.AddRef{$IFDEF CheckPasTreeRefCount}('TPasImplWithDo.Body'){$ENDIF};
    end
  else
    raise Exception.Create('TPasImplWithDo.AddElement body already set');
end;

procedure TPasImplWithDo.AddExpression(const Expression: TPasExpr);
begin
  Expressions.Add(Expression);
end;

procedure TPasImplWithDo.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  for i:=0 to Expressions.Count-1 do
    ForEachChildCall(aMethodCall,Arg,TPasElement(Expressions[i]),false);
  if Elements.IndexOf(Body)<0 then
    ForEachChildCall(aMethodCall,Arg,Body,false);
  inherited ForEachCall(aMethodCall, Arg);
end;

{ TPasImplTry }

destructor TPasImplTry.Destroy;
begin
  ReleaseAndNil(TPasElement(FinallyExcept){$IFDEF CheckPasTreeRefCount},'TPasImplTry.FinallyExcept'{$ENDIF});
  ReleaseAndNil(TPasElement(ElseBranch){$IFDEF CheckPasTreeRefCount},'TPasImplTry.ElseBranch'{$ENDIF});
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

procedure TPasImplTry.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,FinallyExcept,false);
  ForEachChildCall(aMethodCall,Arg,ElseBranch,false);
end;

{ TPasImplExceptOn }

destructor TPasImplExceptOn.Destroy;
begin
  ReleaseAndNil(TPasElement(VarEl){$IFDEF CheckPasTreeRefCount},'TPasImplExceptOn.VarEl'{$ENDIF});
  ReleaseAndNil(TPasElement(TypeEl){$IFDEF CheckPasTreeRefCount},'TPasImplExceptOn.TypeEl'{$ENDIF});
  ReleaseAndNil(TPasElement(Body){$IFDEF CheckPasTreeRefCount},'TPasImplExceptOn.Body'{$ENDIF});
  inherited Destroy;
end;

procedure TPasImplExceptOn.AddElement(Element: TPasImplElement);
begin
  inherited AddElement(Element);
  if Body=nil then
    begin
    Body:=Element;
    Body.AddRef{$IFDEF CheckPasTreeRefCount}('TPasImplExceptOn.Body'){$ENDIF};
    end;
end;

procedure TPasImplExceptOn.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  ForEachChildCall(aMethodCall,Arg,VarEl,false);
  ForEachChildCall(aMethodCall,Arg,TypeEl,true);
  if Elements.IndexOf(Body)<0 then
    ForEachChildCall(aMethodCall,Arg,Body,false);
  inherited ForEachCall(aMethodCall, Arg);
end;

procedure TPasImplExceptOn.ClearTypeReferences(aType: TPasElement);
begin
  if TypeEl=aType then
    ReleaseAndNil(TPasElement(TypeEl){$IFDEF CheckPasTreeRefCount},'TPasImplExceptOn.TypeEl'{$ENDIF});
end;

function TPasImplExceptOn.VariableName: String;
begin
  If assigned(VarEl) then
    Result:=VarEl.Name
  else
    Result:='';
end;

function TPasImplExceptOn.TypeName: string;
begin
  If assigned(TypeEl) then
    Result:=TypeEl.GetDeclaration(True)
  else
    Result:='';
end;

{ TPasImplStatement }

function TPasImplStatement.CloseOnSemicolon: boolean;
begin
  Result:=true;
end;

{ TPasExpr }

constructor TPasExpr.Create(AParent: TPasElement; AKind: TPasExprKind;
  AOpCode: TExprOpCode);
begin
  inherited Create(ClassName, AParent);
  Kind:=AKind;
  OpCode:=AOpCode;
end;

destructor TPasExpr.Destroy;
begin
  ReleaseAndNil(TPasElement(Format1){$IFDEF CheckPasTreeRefCount},'TPasExpr.format1'{$ENDIF});
  ReleaseAndNil(TPasElement(Format2){$IFDEF CheckPasTreeRefCount},'TPasExpr.format2'{$ENDIF});
  inherited Destroy;
end;

{ TPrimitiveExpr }

function TPrimitiveExpr.GetDeclaration(full: Boolean): string;
begin
  Result:=Value;
  if full then ;
end;

constructor TPrimitiveExpr.Create(AParent : TPasElement; AKind: TPasExprKind; const AValue : string);
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

function TBoolConstExpr.GetDeclaration(full: Boolean): string;

begin
  If Value then
    Result:='True'
  else
    Result:='False';
  if full then ;
end;



{ TUnaryExpr }

function TUnaryExpr.GetDeclaration(full: Boolean): string;

Const
  WordOpcodes = [eopDiv,eopMod,eopshr,eopshl,eopNot,eopAnd,eopOr,eopXor];

begin
  Result:=OpCodeStrings[Opcode];
  if OpCode in WordOpCodes  then
    Result:=Result+' ';
  If Assigned(Operand) then
    Result:=Result+' '+Operand.GetDeclaration(Full);
end;

constructor TUnaryExpr.Create(AParent : TPasElement; AOperand: TPasExpr; AOpCode: TExprOpCode);
begin
  inherited Create(AParent,pekUnary, AOpCode);
  Operand:=AOperand;
  Operand.Parent:=Self;
end;

destructor TUnaryExpr.Destroy;
begin
  ReleaseAndNil(TPasElement(Operand){$IFDEF CheckPasTreeRefCount},'TUnaryExpr.Operand'{$ENDIF});
  inherited Destroy;
end;

procedure TUnaryExpr.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,Operand,false);
end;

{ TBinaryExpr }

function TBinaryExpr.GetDeclaration(full: Boolean): string;
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
    begin
    Result:=OpcodeStrings[Opcode];
    if Not (OpCode in [eopAddress,eopDeref,eopSubIdent]) then
      Result:=' '+Result+' ';
    end;
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
    if OpLevel(Left) < OpLevel(Self) then
      Result := Result + '(' + op + ')'
    else
      Result := Result + op;
  end;
end;


constructor TBinaryExpr.Create(AParent : TPasElement; xleft,xright:TPasExpr; AOpCode:TExprOpCode);
begin
  inherited Create(AParent,pekBinary, AOpCode);
  left:=xleft;
  left.Parent:=Self;
  right:=xright;
  right.Parent:=Self;
end;

constructor TBinaryExpr.CreateRange(AParent : TPasElement; xleft,xright:TPasExpr);
begin
  inherited Create(AParent,pekRange, eopNone);
  left:=xleft;
  left.Parent:=Self;
  right:=xright;
  right.Parent:=Self;
end;

destructor TBinaryExpr.Destroy;
begin
  ReleaseAndNil(TPasElement(left){$IFDEF CheckPasTreeRefCount},'TBinaryExpr.left'{$ENDIF});
  ReleaseAndNil(TPasElement(right){$IFDEF CheckPasTreeRefCount},'TBinaryExpr.right'{$ENDIF});
  inherited Destroy;
end;

procedure TBinaryExpr.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,left,false);
  ForEachChildCall(aMethodCall,Arg,right,false);
end;

class function TBinaryExpr.IsRightSubIdent(El: TPasElement): boolean;
var
  Bin: TBinaryExpr;
begin
  if (El=nil) or not (El.Parent is TBinaryExpr) then exit(false);
  Bin:=TBinaryExpr(El.Parent);
  Result:=(Bin.right=El) and (Bin.OpCode=eopSubIdent);
end;

{ TParamsExpr }

function TParamsExpr.GetDeclaration(full: Boolean): string;

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
  if full and Assigned(Value) then
    Result:=Value.GetDeclaration(True)+Result;
end;

procedure TParamsExpr.AddParam(xp:TPasExpr);
var
  i : Integer;
begin
  i:=Length(Params);
  SetLength(Params, i+1);
  Params[i]:=xp;
end;

procedure TParamsExpr.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  ForEachChildCall(aMethodCall,Arg,Value,false);
  for i:=0 to Length(Params)-1 do
    ForEachChildCall(aMethodCall,Arg,Params[i],false);
end;

constructor TParamsExpr.Create(AParent : TPasElement; AKind: TPasExprKind);
begin
  inherited Create(AParent,AKind, eopNone);
end;

destructor TParamsExpr.Destroy;
var
  i : Integer;
begin
  ReleaseAndNil(TPasElement(Value){$IFDEF CheckPasTreeRefCount},'TParamsExpr.Value'{$ENDIF});
  for i:=0 to length(Params)-1 do
    Params[i].Release{$IFDEF CheckPasTreeRefCount}('TParamsExpr.Params'){$ENDIF};
  inherited Destroy;
end;

{ TRecordValues }

function TRecordValues.GetDeclaration(full: Boolean): string;

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
  Result:='('+Result+')';
end;

procedure TRecordValues.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to length(Fields)-1 do
    with Fields[i] do
      begin
      if NameExp<>nil then
        ForEachChildCall(aMethodCall,Arg,NameExp,false);
      if ValueExp<>nil then
        ForEachChildCall(aMethodCall,Arg,ValueExp,false);
      end;
end;

constructor TRecordValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone);
end;

destructor TRecordValues.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Fields)-1 do
    begin
    Fields[i].NameExp.Release{$IFDEF CheckPasTreeRefCount}('TRecordValues.Fields.NameExpr'){$ENDIF};
    Fields[i].ValueExp.Release{$IFDEF CheckPasTreeRefCount}('TRecordValues.Fields.ValueExp'){$ENDIF};
    end;
  Fields:=nil;
  inherited Destroy;
end;

procedure TRecordValues.AddField(AName: TPrimitiveExpr; Value: TPasExpr);
var
  i : Integer;
begin
  i:=length(Fields);
  SetLength(Fields, i+1);
  Fields[i].Name:=AName.Value;
  Fields[i].NameExp:=AName;
  AName.Parent:=Self;
  Fields[i].ValueExp:=Value;
  Value.Parent:=Self;
end;

{ TNilExpr }

function TNilExpr.GetDeclaration(full: Boolean): string;
begin
  Result:='Nil';
  if full then ;
end;

{ TInheritedExpr }

function TInheritedExpr.GetDeclaration(full: Boolean): string;
begin
  Result:='Inherited';
  if full then ;
end;

{ TSelfExpr }

function TSelfExpr.GetDeclaration(full: Boolean): string;
begin
  Result:='Self';
  if full then ;
end;

{ TArrayValues }

function TArrayValues.GetDeclaration(full: Boolean): string;

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

procedure TArrayValues.ForEachCall(const aMethodCall: TOnForEachPasElement;
  const Arg: Pointer);
var
  i: Integer;
begin
  inherited ForEachCall(aMethodCall, Arg);
  for i:=0 to length(Values)-1 do
    ForEachChildCall(aMethodCall,Arg,Values[i],false);
end;

constructor TArrayValues.Create(AParent : TPasElement);
begin
  inherited Create(AParent,pekListOfExp, eopNone);
end;

destructor TArrayValues.Destroy;
var
  i : Integer;
begin
  for i:=0 to length(Values)-1 do
    Values[i].Release{$IFDEF CheckPasTreeRefCount}('TArrayValues.Values'){$ENDIF};
  Values:=nil;
  inherited Destroy;
end;

procedure TArrayValues.AddValues(AValue:TPasExpr);
var
  i : Integer;
begin
  i:=length(Values);
  SetLength(Values, i+1);
  Values[i]:=AValue;
  AValue.Parent:=Self;
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
  FreeAndNil(Labels);
  inherited Destroy;
end;

end.
