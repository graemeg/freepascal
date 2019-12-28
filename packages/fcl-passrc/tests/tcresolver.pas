{
  Examples:
    ./testpassrc --suite=TTestResolver.TestEmpty
}
(*
  CheckReferenceDirectives:
    {#a} label "a", labels all elements at the following token
    {@a} reference "a", search at next token for an element e with
           TResolvedReference(e.CustomData).Declaration points to an element
           labeled "a".
    {=a} is "a", search at next token for a TPasAliasType t with t.DestType
           points to an element labeled "a"
*)
unit tcresolver;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, contnrs, strutils, fpcunit, testregistry,
  PasTree, PScanner, PParser, PasResolver, PasResolveEval,
  tcbaseparser;

type
  TSrcMarkerKind = (
    mkLabel,
    mkResolverReference,
    mkDirectReference
    );
  PSrcMarker = ^TSrcMarker;
  TSrcMarker = record
    Kind: TSrcMarkerKind;
    Filename: string;
    Row: cardinal;
    StartCol, EndCol: integer; // token start, end column
    Identifier: string;
    Next: PSrcMarker;
  end;

const
  SrcMarker: array[TSrcMarkerKind] of char = (
    '#', // mkLabel
    '@', // mkResolverReference
    '='  // mkDirectReference
    );
type
  TOnFindUnit = function(Sender: TPasResolver;
    const aUnitName, InFilename: String;
    NameExpr, InFileExpr: TPasExpr): TPasModule of object;
  TOnContinueParsing = procedure(Sender: TPasResolver) of object;

  { TTestEnginePasResolver }

  TTestEnginePasResolver = class(TPasResolver)
  private
    FFilename: string;
    FModule: TPasModule;
    FOnFindUnit: TOnFindUnit;
    FParser: TPasParser;
    FStreamResolver: TStreamResolver;
    FScanner: TPascalScanner;
    FSource: string;
    procedure SetModule(AValue: TPasModule);
  public
    constructor Create;
    destructor Destroy; override;
    procedure ReleaseUsedUnits;
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility;
      const ASrcPos: TPasSourcePos; TypeParams: TFPList = nil): TPasElement;
      overload; override;
    function FindUnit(const AName, InFilename: String; NameExpr,
      InFileExpr: TPasExpr): TPasModule; override;
    procedure UsedInterfacesFinished(Section: TPasSection); override;
    property OnFindUnit: TOnFindUnit read FOnFindUnit write FOnFindUnit;
    property Filename: string read FFilename write FFilename;
    property StreamResolver: TStreamResolver read FStreamResolver write FStreamResolver;
    property Scanner: TPascalScanner read FScanner write FScanner;
    property Parser: TPasParser read FParser write FParser;
    property Source: string read FSource write FSource;
    property Module: TPasModule read FModule write SetModule;
  end;

  { TTestResolverMessage }

  TTestResolverMessage = class
  public
    Id: int64;
    MsgType: TMessageType;
    MsgNumber: integer;
    Msg: string;
    SourcePos: TPasSourcePos;
  end;

  TTestResolverReferenceData = record
    Filename: string;
    Row: integer;
    StartCol: integer;
    EndCol: integer;
    Found: TFPList; // list of TPasElement at this token
  end;
  PTestResolverReferenceData = ^TTestResolverReferenceData;

  TSystemUnitPart = (
    supTObject,
    supTVarRec
    );
  TSystemUnitParts = set of TSystemUnitPart;

  { TCustomTestResolver }

  TCustomTestResolver = Class(TTestParser)
  Private
    {$IF defined(VerbosePasResolver) or defined(VerbosePasResolverMem)}
    FStartElementRefCount: int64;
    {$ENDIF}
    FFirstStatement: TPasImplBlock;
    FModules: TObjectList;// list of TTestEnginePasResolver
    FResolverEngine: TTestEnginePasResolver;
    FResolverMsgs: TObjectList; // list of TTestResolverMessage
    FResolverGoodMsgs: TFPList; // list of TTestResolverMessage marked as expected
    function GetModuleCount: integer;
    function GetModules(Index: integer): TTestEnginePasResolver;
    function GetMsgCount: integer;
    function GetMsgs(Index: integer): TTestResolverMessage;
    procedure OnPasResolverContinueParsing(Sender: TPasResolver);
    function OnPasResolverFindUnit(SrcResolver: TPasResolver;
      const aUnitName, InFilename: String; NameExpr, InFileExpr: TPasExpr): TPasModule;
    procedure OnFindReference(El: TPasElement; FindData: pointer);
    procedure OnCheckElementParent(El: TPasElement; arg: pointer);
    procedure FreeSrcMarkers;
    procedure OnPasResolverLog(Sender: TObject; const Msg: String);
    procedure OnScannerDirective(Sender: TObject; Directive, Param: String;
      var Handled: boolean);
    procedure OnScannerLog(Sender: TObject; const Msg: String);
  Protected
    FirstSrcMarker, LastSrcMarker: PSrcMarker;
    Procedure SetUp; override;
    Procedure TearDown; override;
    procedure CreateEngine(var TheEngine: TPasTreeContainer); override;
    procedure ParseModule; override;
    procedure ParseProgram; virtual;
    procedure ParseUnit; virtual;
    procedure CheckReferenceDirectives; virtual;
    procedure CheckResolverHint(MsgType: TMessageType; MsgNumber: integer;
      Msg: string; Marker: PSrcMarker = nil); virtual;
    procedure CheckResolverUnexpectedHints(WithSourcePos: boolean = false); virtual;
    procedure CheckResolverException(Msg: string; MsgNumber: integer);
    procedure CheckParserException(Msg: string; MsgNumber: integer);
    procedure CheckAccessMarkers; virtual;
    procedure CheckParamsExpr_pkSet_Markers; virtual;
    procedure CheckAttributeMarkers; virtual;
    procedure GetSrc(Index: integer; out SrcLines: TStringList; out aFilename: string);
    function FindElementsAt(aFilename: string; aLine, aStartCol, aEndCol: integer): TFPList;// list of TPasElement
    function FindElementsAt(aMarker: PSrcMarker; ErrorOnNoElements: boolean = true): TFPList;// list of TPasElement
    function FindSrcLabel(const Identifier: string): PSrcMarker;
    function FindElementsAtSrcLabel(const Identifier: string; ErrorOnNoElements: boolean = true): TFPList;// list of TPasElement
    procedure WriteSources(const aFilename: string; aRow, aCol: integer);
    procedure RaiseErrorAtSrc(Msg: string; const aFilename: string; aRow, aCol: integer);
    procedure RaiseErrorAtSrcMarker(Msg: string; aMarker: PSrcMarker);
    procedure HandleError(CurEngine: TTestEnginePasResolver; E: Exception);
  Public
    constructor Create; override;
    destructor Destroy; override;
    function FindModuleWithFilename(aFilename: string): TTestEnginePasResolver;
    function AddModule(aFilename: string): TTestEnginePasResolver;
    function AddModuleWithSrc(aFilename, Src: string): TTestEnginePasResolver;
    function AddModuleWithIntfImplSrc(aFilename, InterfaceSrc,
      ImplementationSrc: string): TTestEnginePasResolver;
    procedure AddSystemUnit(Parts: TSystemUnitParts = []);
    procedure StartProgram(NeedSystemUnit: boolean; SystemUnitParts: TSystemUnitParts = []);
    procedure StartUnit(NeedSystemUnit: boolean);
    property Modules[Index: integer]: TTestEnginePasResolver read GetModules;
    property ModuleCount: integer read GetModuleCount;
    property ResolverEngine: TTestEnginePasResolver read FResolverEngine;
    property MsgCount: integer read GetMsgCount;
    property Msgs[Index: integer]: TTestResolverMessage read GetMsgs;
  end;

  { TTestResolver }

  TTestResolver = Class(TCustomTestResolver)
  Published
    Procedure TestEmpty;

    // alias
    Procedure TestAliasType;
    Procedure TestAlias2Type;
    Procedure TestAliasTypeRefs;
    Procedure TestAliasOfVarFail;
    Procedure TestAliasType_UnitPrefix;
    Procedure TestAliasType_UnitPrefix_CycleFail;
    Procedure TestAliasTypeNotFoundPosition;
    Procedure TestTypeAliasType;

    // vars, const
    Procedure TestVarLongint;
    Procedure TestVarInteger;
    Procedure TestConstInteger;
    Procedure TestConstInteger2;
    Procedure TestDuplicateVar;
    Procedure TestVarInitConst;
    Procedure TestVarOfVarFail;
    Procedure TestConstOfVarFail;
    Procedure TestTypedConstWrongExprFail;
    Procedure TestVarWrongExprFail;
    Procedure TestArgWrongExprFail;
    Procedure TestTypedConstInConstExprFail;
    Procedure TestVarExternal;
    Procedure TestVarNoSemicolonBeginFail;
    Procedure TestConstIntOperators;
    Procedure TestConstBitwiseOps;
    Procedure TestConstExternal;
    Procedure TestIntegerTypeCast;
    Procedure TestConstFloatOperators;
    Procedure TestFloatTypeCast;
    Procedure TestCurrency;
    Procedure TestWritableConst;
    Procedure TestWritableConst_AssignFail;
    Procedure TestWritableConst_PassVarFail;

    // boolean
    Procedure TestBoolTypeCast;
    Procedure TestConstBoolOperators;
    Procedure TestBoolSet_Const;
    Procedure TestBool_ForIn;
    Procedure TestBool_Assert;
    Procedure TestBool_AssertSysutils;

    // integer range
    Procedure TestIntegerRange;
    Procedure TestIntegerRangeHighLowerLowFail;
    Procedure TestIntegerRangeLowHigh;
    Procedure TestAssignIntRangeWarning;
    Procedure TestByteRangeWarning;
    Procedure TestByteRangeWarningOff;
    Procedure TestCustomIntRangeWarning;
    Procedure TestIntSet_Const;
    Procedure TestIntSet_ConstDuplicateElement;
    Procedure TestInt_ForIn;

    // strings
    Procedure TestChar_BuiltInProcs;
    Procedure TestString_BuiltInProcs;
    Procedure TestString_Element;
    Procedure TestStringElement_MissingArgFail;
    Procedure TestStringElement_IndexNonIntFail;
    Procedure TestStringElement_AsVarArgFail;
    Procedure TestString_DoubleQuotesFail;
    Procedure TestString_ShortstringType;
    Procedure TestConstStringOperators;
    Procedure TestConstUnicodeStringOperators;
    Procedure TestCharSet_Const;
    Procedure TestCharSet_Custom;
    Procedure TestCharAssignStringFail;
    Procedure TestChar_ForIn;

    // enums and sets
    Procedure TestEnums;
    Procedure TestEnumRangeFail;
    Procedure TestEnumDotValueFail;
    Procedure TestSets;
    Procedure TestSetOperators;
    Procedure TestEnumParams;
    Procedure TestSetParams;
    Procedure TestSetFunctions;
    Procedure TestEnumHighLow;
    Procedure TestEnumOrd;
    Procedure TestEnumPredSucc;
    Procedure TestEnum_EqualNilFail;
    Procedure TestEnum_CastIntegerToEnum;
    Procedure TestEnum_Str;
    Procedure TestConstEnumOperators;
    Procedure TestEnumSetConstRange;
    Procedure TestEnumSet_AnonymousEnumtype;
    Procedure TestEnumSet_AnonymousEnumtypeName;
    Procedure TestEnumSet_Const;
    Procedure TestSet_IntRange_Const;
    Procedure TestSet_Byte_Const;
    Procedure TestEnumRange;
    Procedure TestEnum_ForIn;
    Procedure TestEnum_ForInRangeFail;
    Procedure TestEnum_ScopedEnums;
    Procedure TestEnum_ScopedEnumsFail;

    // operators
    Procedure TestPrgAssignment;
    Procedure TestPrgProcVar;
    Procedure TestUnitProcVar;
    Procedure TestAssignIntegers;
    Procedure TestAssignString;
    Procedure TestAssignIntToStringFail;
    Procedure TestAssignStringToIntFail;
    Procedure TestIntegerOperators;
    Procedure TestIntegerBoolFail;
    Procedure TestBooleanOperators;
    Procedure TestStringOperators;
    Procedure TestWideCharOperators;
    Procedure TestFloatOperators;
    Procedure TestCAssignments;
    Procedure TestTypeCastBaseTypes;
    Procedure TestTypeCastAliasBaseTypes;
    Procedure TestTypeCastStrToIntFail;
    Procedure TestTypeCastStrToCharFail;
    Procedure TestTypeCastIntToStrFail;
    Procedure TestTypeCastDoubleToStrFail;
    Procedure TestTypeCastDoubleToIntFail;
    Procedure TestTypeCastDoubleToBoolFail;
    Procedure TestTypeCastBooleanToDoubleFail;
    Procedure TestAssign_Access;
    Procedure TestAssignedIntFail;

    // misc built-in functions
    Procedure TestHighLow;
    Procedure TestStr_BaseTypes;
    Procedure TestStr_StringFail;
    Procedure TestStr_CharFail;
    Procedure TestIncDec;
    Procedure TestIncStringFail;
    Procedure TestTypeInfo;
    Procedure TestTypeInfo_FailRTTIDisabled;

    // statements
    Procedure TestForLoop;
    Procedure TestForLoop_NestedSameVarFail;
    Procedure TestForLoop_AssignVarFail;
    Procedure TestForLoop_PassVarFail;
    Procedure TestStatements;
    Procedure TestCaseOfInt;
    Procedure TestCaseOfIntExtConst;
    Procedure TestCaseIntDuplicateFail;
    Procedure TestCaseOfStringDuplicateFail;
    Procedure TestCaseOfStringRangeDuplicateFail;
    Procedure TestCaseOfBaseType;
    Procedure TestCaseOfExprNonOrdFail;
    Procedure TestCaseOfIncompatibleValueFail;
    Procedure TestTryStatement;
    Procedure TestTryExceptOnNonTypeFail;
    Procedure TestTryExceptOnNonClassFail;
    Procedure TestRaiseNonVarFail;
    Procedure TestRaiseNonClassFail;
    Procedure TestRaiseDescendant;
    Procedure TestStatementsRefs;
    Procedure TestRepeatUntilNonBoolFail;
    Procedure TestWhileDoNonBoolFail;
    Procedure TestIfThen;
    Procedure TestIfThenNonBoolFail;
    Procedure TestIfAssignMissingSemicolonFail;
    Procedure TestForLoopVarNonVarFail;
    Procedure TestForLoopStartIncompFail;
    Procedure TestForLoopEndIncompFail;
    Procedure TestSimpleStatement_VarFail;
    Procedure TestLabelStatementFail;
    Procedure TestLabelStatementDelphiFail;

    // units
    Procedure TestUnitForwardOverloads;
    Procedure TestUnitIntfInitialization;
    Procedure TestUnitUseSystem;
    Procedure TestUnitUseIntf;
    Procedure TestUnitUseImplFail;
    Procedure TestUnit_DuplicateUsesFail;
    Procedure TestUnit_DuplicateUsesIntfImplFail;
    Procedure TestUnit_NestedFail;
    Procedure TestUnitUseDotted;
    Procedure TestUnit_ProgramDefaultNamespace;
    Procedure TestUnit_DottedIdentifier;
    Procedure TestUnit_DottedPrg;
    Procedure TestUnit_DottedUnit;
    Procedure TestUnit_DottedExpr;
    Procedure TestUnit_DuplicateDottedUsesFail;
    Procedure TestUnit_DuplicateUsesDiffNameFail;
    Procedure TestUnit_Unit1DotUnit2Fail;
    Procedure TestUnit_InFilename;
    Procedure TestUnit_InFilenameAliasDelphiFail;
    Procedure TestUnit_InFilenameInUnitDelphiFail;
    Procedure TestUnit_MissingUnitErrorPos;
    Procedure TestUnit_UnitNotFoundErrorPos;
    Procedure TestUnit_AccessIndirectUsedUnitFail;
    Procedure TestUnit_Intf1Impl2Intf1;

    // procs
    Procedure TestProcParam;
    Procedure TestProcParamAccess;
    Procedure TestProcParamConstRef;
    Procedure TestFunctionResult;
    Procedure TestProcedureResultFail;
    Procedure TestProc_ArgVarPrecisionLossFail;
    Procedure TestProc_ArgVarTypeAliasObjFPC;
    Procedure TestProc_ArgVarTypeAliasDelphi;
    Procedure TestProc_ArgVarTypeAliasDelphiMismatchFail;
    Procedure TestProc_ArgMissingSemicolonFail;
    Procedure TestProcOverload;
    Procedure TestProcOverloadImplDuplicateFail;
    Procedure TestProcOverloadImplDuplicate2Fail;
    Procedure TestProcOverloadOtherUnit;
    Procedure TestProcOverloadWithBaseTypes;
    Procedure TestProcOverloadWithBaseTypes2;
    Procedure TestProcOverloadWithDefaultArgs;
    Procedure TestProcOverloadNearestHigherPrecision;
    Procedure TestProcOverloadForLoopIntDouble;
    Procedure TestProcOverloadStringArgCount;
    Procedure TestProcCallLowPrecision;
    Procedure TestProcOverloadUntyped;
    Procedure TestProcOverloadMultiLowPrecisionFail;
    Procedure TestProcOverload_TypeAlias;
    Procedure TestProcOverload_TypeAliasLiteralFail;
    Procedure TestProcOverloadWithClassTypes;
    Procedure TestProcOverloadWithInhClassTypes;
    Procedure TestProcOverloadWithInhAliasClassTypes;
    Procedure TestProcOverloadWithInterfaces;
    Procedure TestProcOverloadBaseTypeOtherUnit;
    Procedure TestProcOverloadBaseProcNoHint;
    Procedure TestProcOverload_UnitOrderFail;
    Procedure TestProcOverload_UnitSameSignature;
    Procedure TestProcOverloadDelphiMissingNextOverload;
    Procedure TestProcOverloadDelphiMissingPrevOverload;
    Procedure TestProcOverloadDelphiUnit;
    Procedure TestProcOverloadDelphiUnitNoOverloadFail;
    Procedure TestProcOverloadObjFPCUnitWithoutOverloadMod;
    Procedure TestProcOverloadDelphiWithObjFPC;
    Procedure TestProcOverloadDelphiOverride;
    Procedure TestProcDuplicate;
    Procedure TestNestedProc;
    Procedure TestNestedProc_ResultString;
    Procedure TestFuncAssignFail;
    Procedure TestForwardProc;
    Procedure TestForwardProcUnresolved;
    Procedure TestNestedForwardProc;
    Procedure TestNestedForwardProcUnresolved;
    Procedure TestForwardProcFuncMismatch;
    Procedure TestForwardFuncResultMismatch;
    Procedure TestUnitIntfProc;
    Procedure TestUnitIntfProcUnresolved;
    Procedure TestUnitIntfMismatchArgName;
    Procedure TestProcOverloadIsNotFunc;
    Procedure TestProcCallMissingParams;
    Procedure TestProcArgDefaultValue;
    Procedure TestProcArgDefaultValueTypeMismatch;
    Procedure TestProcPassConstToVar;
    Procedure TestBuiltInProcCallMissingParams;
    Procedure TestAssignFunctionResult;
    Procedure TestAssignProcResultFail;
    Procedure TestFunctionResultInCondition;
    Procedure TestExit;
    Procedure TestBreak;
    Procedure TestContinue;
    Procedure TestProcedureExternal;
    Procedure TestProc_UntypedParam_Forward;
    Procedure TestProc_Varargs;
    Procedure TestProc_VarargsOfT;
    Procedure TestProc_VarargsOfTMismatch;
    Procedure TestProc_ParameterExprAccess;
    Procedure TestProc_FunctionResult_DeclProc;
    Procedure TestProc_TypeCastFunctionResult;
    Procedure TestProc_ImplicitCalls;
    Procedure TestProc_Absolute;
    Procedure TestProc_LocalInit;
    Procedure TestProc_ExtNamePropertyFail;

    // anonymous procs
    Procedure TestAnonymousProc_Assign;
    Procedure TestAnonymousProc_AssignSemicolonFail;
    Procedure TestAnonymousProc_Assign_ReferenceToMissingFail;
    Procedure TestAnonymousProc_Assign_WrongParamListFail;
    Procedure TestAnonymousProc_Arg;
    Procedure TestAnonymousProc_ArgSemicolonFail;
    Procedure TestAnonymousProc_EqualFail;
    Procedure TestAnonymousProc_ConstFail;
    Procedure TestAnonymousProc_Assembler;
    Procedure TestAnonymousProc_NameFail;
    Procedure TestAnonymousProc_StatementFail;
    Procedure TestAnonymousProc_Typecast_ObjFPC;
    Procedure TestAnonymousProc_Typecast_Delphi;
    Procedure TestAnonymousProc_TypecastToResultFail;
    Procedure TestAnonymousProc_With;
    Procedure TestAnonymousProc_ExceptOn;
    Procedure TestAnonymousProc_Nested;
    Procedure TestAnonymousProc_ForLoop;

    // record
    Procedure TestRecord;
    Procedure TestRecordVariant;
    Procedure TestRecordVariantNested;
    Procedure TestRecord_WriteConstParamFail;
    Procedure TestRecord_WriteConstParam_WithFail;
    Procedure TestRecord_WriteNestedConstParamFail;
    Procedure TestRecord_WriteNestedConstParamWithFail;
    Procedure TestRecord_TypeCast;
    Procedure TestRecord_NewDispose;
    Procedure TestRecord_Const;
    Procedure TestRecord_Const_DuplicateFail;
    Procedure TestRecord_Const_ExprMismatchFail;
    Procedure TestRecord_Const_MissingHint;
    Procedure TestRecord_Const_UntypedFail;
    Procedure TestRecord_Const_NestedRecord;
    Procedure TestRecord_Const_Variant;
    Procedure TestRecord_Default;
    Procedure TestRecord_VarExternal;
    Procedure TestRecord_VarSelfFail;

    // advanced record
    Procedure TestAdvRecord;
    Procedure TestAdvRecord_Private;
    Procedure TestAdvRecord_StrictPrivate;
    Procedure TestAdvRecord_StrictPrivateFail;
    Procedure TestAdvRecord_MethodImplMissingFail;
    Procedure TestAdvRecord_VarConst;
    Procedure TestAdvRecord_RecVal_ConstFail;
    Procedure TestAdvRecord_RecVal_ClassVarFail;
    Procedure TestAdvRecord_LocalForwardType;
    Procedure TestAdvRecord_Constructor_NewInstance;
    Procedure TestAdvRecord_ConstructorNoParamsFail;
    Procedure TestAdvRecord_ClassConstructor;
    Procedure TestAdvRecord_ClassConstructorParamsFail;
    Procedure TestAdvRecord_ClassConstructor_CallFail;
    Procedure TestAdvRecord_ClassConstructorDuplicateFail;
    Procedure TestAdvRecord_NestedRecordType;
    Procedure TestAdvRecord_NestedArgConstFail;
    Procedure TestAdvRecord_Property;
    Procedure TestAdvRecord_ClassProperty;
    Procedure TestAdvRecord_PropertyDefault;
    Procedure TestAdvRecord_RecordAsFuncResult;
    Procedure TestAdvRecord_InheritedFail;
    Procedure TestAdvRecord_ForInEnumerator;
    Procedure TestAdvRecord_InFunctionFail;
    Procedure TestAdvRecord_SubClass;

    // class
    Procedure TestClass;
    Procedure TestClassDefaultInheritance;
    Procedure TestClassTripleInheritance;
    Procedure TestClassInheritanceCycleFail;
    Procedure TestClassDefaultVisibility;
    Procedure TestClassForward;
    Procedure TestClassForwardAsAncestorFail;
    Procedure TestClassForwardNotResolved;
    Procedure TestClassForwardDuplicateFail;
    Procedure TestClassForwardDelphiFail;
    Procedure TestClassForwardObjFPCProgram;
    Procedure TestClassForwardObjFPCUnit;
    Procedure TestClass_Method;
    Procedure TestClass_ConstructorMissingDotFail;
    Procedure TestClass_MethodImplDuplicateFail;
    Procedure TestClass_MethodWithoutClassFail;
    Procedure TestClass_MethodInOtherUnitFail;
    Procedure TestClass_MethodWithParams;
    Procedure TestClass_MethodUnresolvedPrg;
    Procedure TestClass_MethodUnresolvedUnit;
    Procedure TestClass_MethodAbstract;
    Procedure TestClass_MethodAbstractWithoutVirtualFail;
    Procedure TestClass_MethodAbstractHasBodyFail;
    Procedure TestClass_MethodUnresolvedWithAncestor;
    Procedure TestClass_ProcFuncMismatch;
    Procedure TestClass_MethodOverload;
    Procedure TestClass_MethodInvalidOverload;
    Procedure TestClass_MethodOverride;
    Procedure TestClass_MethodOverride2;
    Procedure TestClass_MethodOverrideFixCase;
    Procedure TestClass_MethodOverrideSameResultType;
    Procedure TestClass_MethodOverrideDiffResultTypeFail;
    Procedure TestClass_MethodOverrideDiffVarName;
    Procedure TestClass_MethodOverloadMissingInDelphi;
    Procedure TestClass_MethodOverloadAncestor;
    Procedure TestClass_MethodOverloadUnit;
    Procedure TestClass_HintMethodHidesNonVirtualMethod;
    Procedure TestClass_HintMethodHidesNonVirtualMethodWithoutBody_NoHint;
    Procedure TestClass_NoHintMethodHidesPrivateMethod;
    Procedure TestClass_MethodReintroduce;
    Procedure TestClass_MethodOverloadArrayOfTClass;
    Procedure TestClass_ConstructorHidesAncestorWarning;
    Procedure TestClass_ConstructorOverride;
    Procedure TestClass_ConstructorAccessHiddenAncestorFail;
    Procedure TestClass_ConstructorNoteAbstractMethods;
    Procedure TestClass_ConstructorNoNoteAbstractMethods;
    Procedure TestClass_MethodScope;
    Procedure TestClass_IdentifierSelf;
    Procedure TestClassCallInherited;
    Procedure TestClassCallInheritedNoParamsAbstractFail;
    Procedure TestClassCallInheritedWithParamsAbstractFail;
    Procedure TestClassCallInheritedConstructor;
    Procedure TestClassCallInheritedNested;
    Procedure TestClassCallInheritedAs;
    Procedure TestClassAssignNil;
    Procedure TestClassAssign;
    Procedure TestClassNilAsParam;
    Procedure TestClass_Operators_Is_As;
    Procedure TestClass_OperatorIsOnNonTypeFail;
    Procedure TestClass_OperatorAsOnNonDescendantFail;
    Procedure TestClass_OperatorAsOnNonTypeFail;
    Procedure TestClassAsFuncResult;
    Procedure TestClassTypeCast;
    Procedure TestClassTypeCastUnrelatedFail;
    Procedure TestClass_TypeCastSelf;
    Procedure TestClass_TypeCaseMultipleParamsFail;
    Procedure TestClass_TypeCastAssign;
    Procedure TestClass_AccessMemberViaClassFail;
    Procedure TestClass_FuncReturningObjectMember;
    Procedure TestClass_StaticWithoutClassFail;
    Procedure TestClass_SelfInStaticFail;
    Procedure TestClass_SelfDotInStaticFail;
    Procedure TestClass_PrivateProtectedInSameUnit;
    Procedure TestClass_PrivateInMainBeginFail;
    Procedure TestClass_PrivateInDescendantFail;
    Procedure TestClass_ProtectedInDescendant;
    Procedure TestClass_StrictPrivateInMainBeginFail;
    Procedure TestClass_StrictProtectedInMainBeginFail;
    Procedure TestClass_Constructor_NewInstance;
    Procedure TestClass_Destructor_FreeInstance;
    Procedure TestClass_ConDestructor_CallInherited;
    Procedure TestClass_Constructor_Inherited;
    Procedure TestClass_SubObject;
    Procedure TestClass_WithClassInstance;
    Procedure TestClass_ProcedureExternal;
    Procedure TestClass_ReintroducePublicVarFail;
    Procedure TestClass_ReintroducePrivateVar;
    Procedure TestClass_ReintroduceProc;
    Procedure TestClass_UntypedParam_TypeCast;
    Procedure TestClass_Sealed;
    Procedure TestClass_SealedDescendFail;
    Procedure TestClass_Abstract;
    Procedure TestClass_AbstractCreateFail;
    Procedure TestClass_VarExternal;
    Procedure TestClass_WarnOverrideLowerVisibility;
    Procedure TestClass_Const;
    Procedure TestClass_ClassMissingVarFail;
    Procedure TestClass_ClassConstFail;
    Procedure TestClass_Enumerator;
    Procedure TestClass_EnumeratorFunc;
    Procedure TestClass_ForInPropertyStaticArray;
    Procedure TestClass_TypeAlias;
    Procedure TestClass_Message;
    Procedure TestClass_Message_MissingParamFail;

    // published
    Procedure TestClass_PublishedClassVarFail;
    Procedure TestClass_PublishedClassPropertyFail;
    Procedure TestClass_PublishedClassFunctionFail;
    Procedure TestClass_PublishedOverloadFail;

    // nested class
    Procedure TestNestedClass;
    Procedure TestNestedClass_Forward;
    procedure TestNestedClass_StrictPrivateFail;
    procedure TestNestedClass_AccessStrictPrivate;
    procedure TestNestedClass_AccessParent;
    procedure TestNestedClass_BodyAccessParentVarFail;
    procedure TestNestedClass_PropertyAccessParentVarFail;

    // external class
    Procedure TestExternalClass;
    Procedure TestExternalClass_Descendant;
    Procedure TestExternalClass_HintMethodHidesNonVirtualMethodExact;

    // class of
    Procedure TestClassOf;
    Procedure TestClassOfAlias;
    Procedure TestClassOfNonClassFail;
    Procedure TestClassOfAssignClassOfFail;
    Procedure TestClassOfIsOperatorFail;
    Procedure TestClassOfAsOperatorFail;
    Procedure TestClassOfIsOperator;
    Procedure TestClass_ClassVar;
    Procedure TestClassOfDotClassVar;
    Procedure TestClassOfDotVarFail;
    Procedure TestClassOfDotClassProc;
    Procedure TestClassOfDotProcFail;
    Procedure TestClassOfDotClassProperty;
    Procedure TestClassOfDotPropertyFail;
    Procedure TestClass_ClassProcSelf;
    Procedure TestClass_ClassProcSelfTypeCastFail;
    Procedure TestClass_ClassMembers;
    Procedure TestClassOf_AsFail;
    Procedure TestClassOf_MemberAsFail;
    Procedure TestClassOf_IsFail;
    Procedure TestClass_TypeCast;
    Procedure TestClassOf_AlwaysForward;
    Procedure TestClassOf_ClassOfBeforeClass_FuncResult;
    Procedure TestClassOf_Const;
    Procedure TestClassOf_Const2;

    // property
    Procedure TestProperty1;
    Procedure TestPropertyAccessorNotInFront;
    Procedure TestPropertyReadAndWriteMissingFail;
    Procedure TestPropertyReadAccessorVarWrongType;
    Procedure TestPropertyReadAccessorProcNotFunc;
    Procedure TestPropertyReadAccessorFuncWrongResult;
    Procedure TestPropertyReadAccessorFuncWrongArgCount;
    Procedure TestPropertyReadAccessorFunc;
    Procedure TestPropertyReadAccessorStrictPrivate;
    Procedure TestPropertyReadAccessorNonClassFail;
    Procedure TestPropertyWriteAccessorVarWrongType;
    Procedure TestPropertyWriteAccessorFuncNotProc;
    Procedure TestPropertyWriteAccessorProcWrongArgCount;
    Procedure TestPropertyWriteAccessorProcWrongArg;
    Procedure TestPropertyWriteAccessorProcWrongArgType;
    Procedure TestPropertyWriteAccessorProc;
    Procedure TestPropertyTypeless;
    Procedure TestPropertyTypelessNoAncestorFail;
    Procedure TestPropertyStoredAccessor;
    Procedure TestPropertyStoredAccessorVarWrongType;
    Procedure TestPropertyStoredAccessorProcNotFunc;
    Procedure TestPropertyStoredAccessorFuncWrongResult;
    Procedure TestPropertyStoredAccessorFuncWrongArgCount;
    Procedure TestPropertyIndexSpec;
    Procedure TestPropertyIndexSpec_ReadAccessorWrongArgCount;
    Procedure TestPropertyIndexSpec_ReadAccessorWrongIndexArgType;
    Procedure TestPropertyDefaultValue;
    Procedure TestPropertyAssign;
    Procedure TestPropertyAssignReadOnlyFail;
    Procedure TestProperty_PassAsParam;
    Procedure TestPropertyReadNonReadableFail;
    Procedure TestPropertyArgs1;
    Procedure TestPropertyArgs2;
    Procedure TestPropertyArgsWithDefaultsFail;
    Procedure TestPropertyArgs_StringConstDefault;
    Procedure TestClassProperty;
    Procedure TestClassPropertyNonStaticFail;
    Procedure TestClassPropertyNonStaticAllow;
    Procedure TestArrayProperty;
    Procedure TestArrayProperty_PassImplicitCallClassFunc;
    Procedure TestProperty_WrongTypeAsIndexFail;
    Procedure TestProperty_Option_ClassPropertyNonStatic;
    Procedure TestDefaultProperty;
    Procedure TestDefaultPropertyIncVisibility;
    Procedure TestProperty_MissingDefault;
    Procedure TestProperty_DefaultDotFail;

    // class interfaces
    Procedure TestClassInterface;
    Procedure TestClassInterfaceForward;
    Procedure TestClassInterfaceVarFail;
    Procedure TestClassInterfaceConstFail;
    Procedure TestClassInterfaceClassMethodFail;
    Procedure TestClassInterfaceNestedTypeFail;
    Procedure TestClassInterfacePropertyStoredFail;
    Procedure TestClassInterface_ConstructorFail;
    Procedure TestClassInterface_DelphiClassAncestorIntfFail;
    Procedure TestClassInterface_ObjFPCClassAncestorIntf;
    Procedure TestClassInterface_MethodVirtualFail;
    Procedure TestClassInterface_Overloads;
    Procedure TestClassInterface_OverloadHint;
    Procedure TestClassInterface_OverloadNoHint;
    Procedure TestClassInterface_IntfListClassFail;
    Procedure TestClassInterface_IntfListDuplicateFail;
    Procedure TestClassInterface_MissingMethodFail;
    Procedure TestClassInterface_MissingAncestorMethodFail;
    Procedure TestClassInterface_DefaultProperty;
    Procedure TestClassInterface_MethodResolution;
    Procedure TestClassInterface_MethodResolutionDuplicateFail;
    Procedure TestClassInterface_DelegationIntf;
    Procedure TestClassInterface_Delegation_DuplPropFail;
    Procedure TestClassInterface_Delegation_MethodResFail;
    Procedure TestClassInterface_DelegationClass;
    Procedure TestClassInterface_DelegationFQN;
    Procedure TestClassInterface_Assign;
    Procedure TestClassInterface_AssignObjVarIntfVarFail;
    Procedure TestClassInterface_AssignDescendentFail;
    Procedure TestClassInterface_Args;
    Procedure TestClassInterface_Enumerator;
    Procedure TestClassInterface_PassTypecastClassToIntfAsVarParamFail;
    Procedure TestClassInterface_PassTypecastIntfToClassAsVarParamFail;
    Procedure TestClassInterface_GUID;

    // with
    Procedure TestWithBlock1;
    Procedure TestWithBlock2;
    Procedure TestWithBlockFuncResult;
    Procedure TestWithBlockConstructor;

    // arrays
    Procedure TestDynArrayOfLongint;
    Procedure TestDynArrayOfSelfFail;
    Procedure TestStaticArray;
    Procedure TestStaticArrayOfChar;
    Procedure TestStaticArrayOfCharDelphi;
    Procedure TestStaticArrayOfRangeElCheckFail;
    Procedure TestArrayOfArray;
    Procedure TestArrayOfArray_NameAnonymous;
    Procedure TestFunctionReturningArray;
    Procedure TestArray_LowHigh;
    Procedure TestArray_LowVarFail;
    Procedure TestArray_AssignDiffElTypeFail;
    Procedure TestArray_AssignSameSignatureDelphiFail;
    Procedure TestArray_Assigned;
    Procedure TestPropertyOfTypeArray;
    Procedure TestArrayElementFromFuncResult_AsParams;
    Procedure TestArrayEnumTypeRange;
    Procedure TestArrayEnumTypeConstNotEnoughValuesFail1;
    Procedure TestArrayEnumTypeConstNotEnoughValuesFail2;
    Procedure TestArrayEnumTypeConstWrongTypeFail;
    Procedure TestArrayEnumTypeConstNonConstFail;
    Procedure TestArrayEnumTypeSetLengthFail;
    Procedure TestArrayEnumCustomRange;
    Procedure TestArray_DynArrayConstObjFPC;
    Procedure TestArray_DynArrayConstDelphi;
    Procedure TestArray_DynArrAssignStaticDelphiFail;
    Procedure TestArray_Static_Const;
    Procedure TestArray_Record_Const;
    Procedure TestArray_MultiDim_Const;
    Procedure TestArray_AssignNilToStaticArrayFail1;
    Procedure TestArray_SetLengthProperty;
    Procedure TestStaticArray_SetlengthFail;
    Procedure TestArray_PassArrayElementToVarParam;
    Procedure TestArray_OpenArrayOfString;
    Procedure TestArray_OpenArrayOfString_IntFail;
    Procedure TestArray_OpenArrayOverride;
    Procedure TestArray_OpenArrayAsDynArraySetLengthFail;
    Procedure TestArray_OpenArrayAsDynArray;
    Procedure TestArray_OpenArrayDelphi;
    Procedure TestArray_OpenArrayChar;
    Procedure TestArray_CopyConcat;
    Procedure TestStaticArray_CopyConcat;// ToDo
    Procedure TestArray_CopyMismatchFail;
    Procedure TestArray_InsertDeleteAccess;
    Procedure TestArray_InsertArray;
    Procedure TestStaticArray_InsertFail;
    Procedure TestStaticArray_DeleteFail;
    Procedure TestArray_InsertItemMismatchFail;
    Procedure TestArray_TypeCast;
    Procedure TestArray_TypeCastWrongElTypeFail;
    Procedure TestArray_ConstDynArrayWrite;
    Procedure TestArray_ConstOpenArrayWriteFail;
    Procedure TestArray_ForIn;
    Procedure TestArray_Arg_AnonymousStaticFail;
    Procedure TestArray_Arg_AnonymousMultiDimFail;

    // array of const
    Procedure TestArrayOfConst;
    Procedure TestArrayOfConst_PassDynArrayOfIntFail;
    Procedure TestArrayOfConst_AssignNilFail;
    Procedure TestArrayOfConst_SetLengthFail;

    // static arrays
    Procedure TestArrayIntRange_OutOfRange;
    Procedure TestArrayIntRange_OutOfRangeError;
    Procedure TestArrayCharRange_OutOfRange;

    // procedure types
    Procedure TestProcTypesAssignObjFPC;
    Procedure TestMethodTypesAssignObjFPC;
    Procedure TestProcTypeCall;
    Procedure TestProcType_FunctionFPC;
    Procedure TestProcType_FunctionDelphi;
    Procedure TestProcType_ProcedureDelphi;
    Procedure TestProcType_MethodFPC;
    Procedure TestProcType_MethodDelphi;
    Procedure TestAssignProcToMethodFail;
    Procedure TestAssignMethodToProcFail;
    Procedure TestAssignProcToFunctionFail;
    Procedure TestAssignProcWrongArgsFail;
    Procedure TestAssignProcWrongArgAccessFail;
    Procedure TestProcType_AssignNestedProcFail;
    Procedure TestArrayOfProc;
    Procedure TestProcType_Assigned;
    Procedure TestProcType_TNotifyEvent;
    Procedure TestProcType_TNotifyEvent_NoAtFPC_Fail1;
    Procedure TestProcType_TNotifyEvent_NoAtFPC_Fail2;
    Procedure TestProcType_TNotifyEvent_NoAtFPC_Fail3;
    Procedure TestProcType_WhileListCompare;
    Procedure TestProcType_IsNested;
    Procedure TestProcType_IsNested_AssignProcFail;
    Procedure TestProcType_ReferenceTo;
    Procedure TestProcType_AllowNested;
    Procedure TestProcType_AllowNestedOfObject;
    Procedure TestProcType_AsArgOtherUnit;
    Procedure TestProcType_Property;
    Procedure TestProcType_PropertyCallWrongArgFail;
    Procedure TestProcType_Typecast;
    Procedure TestProcType_InsideFunction;
    Procedure TestProcType_PassProcToUntyped;

    // pointer
    Procedure TestPointer;
    Procedure TestPointer_AnonymousSetFail;
    Procedure TestPointer_AssignPointerToClassFail;
    Procedure TestPointer_TypecastToMethodTypeFail;
    Procedure TestPointer_TypecastFromMethodTypeFail;
    Procedure TestPointer_TypecastMethod_proMethodAddrAsPointer;
    Procedure TestPointer_OverloadSignature;
    Procedure TestPointer_Assign;
    Procedure TestPointerTyped;
    Procedure TestPointerTypedForwardMissingFail;
    Procedure TestPointerTyped_CycleFail;
    Procedure TestPointerTyped_AssignMismatchFail;
    Procedure TestPointerTyped_AddrAddrFail;
    Procedure TestPointerTyped_RecordObjFPC;
    Procedure TestPointerTyped_RecordDelphi;
    Procedure TestPointerTyped_Arithmetic;

    // resourcestrings
    Procedure TestResourcestring;
    Procedure TestResourcestringAssignFail;
    Procedure TestResourcestringLocalFail;
    Procedure TestResourcestringInConstFail;
    Procedure TestResourcestringPassVarArgFail;

    // hints
    Procedure TestHint_ElementHints;
    Procedure TestHint_ElementHintsMsg;
    Procedure TestHint_ElementHintsAlias;
    Procedure TestHint_ElementHints_WarnOff_SymbolDeprecated;
    Procedure TestHint_Garbage;

    // helpers
    Procedure TestClassHelper;
    Procedure TestClassHelper_AncestorIsNotHelperForDescendantFail;
    Procedure TestClassHelper_HelperForParentFail;
    Procedure TestClassHelper_ForInterfaceFail;
    Procedure TestClassHelper_FieldFail;
    Procedure TestClassHelper_AbstractFail;
    Procedure TestClassHelper_VirtualObjFPCFail;
    Procedure TestClassHelper_VirtualDelphiFail;
    Procedure TestClassHelper_DestructorFail;
    Procedure TestClassHelper_ClassRefersToTypeHelperOfAncestor;
    Procedure TestClassHelper_InheritedObjFPC;
    Procedure TestClassHelper_InheritedObjFPC2;
    Procedure TestClassHelper_InheritedObjFPCStrictPrivateFail;
    Procedure TestClassHelper_InheritedClassObjFPC;
    Procedure TestClassHelper_InheritedDelphi;
    Procedure TestClassHelper_NestedInheritedParentFail;
    Procedure TestClassHelper_AccessFields;
    Procedure TestClassHelper_HelperDotClassMethodFail;
    Procedure TestClassHelper_WithHelperFail;
    Procedure TestClassHelper_AsTypeFail;
    Procedure TestClassHelper_ClassMethod;
    Procedure TestClassHelper_Enumerator;
    Procedure TestClassHelper_FromUnitInterface;
    Procedure TestClassHelper_Constructor_NewInstance;
    Procedure TestClassHelper_ReintroduceHides_CallFail;
    Procedure TestClassHelper_DefaultProperty;
    Procedure TestClassHelper_DefaultClassProperty;
    Procedure TestClassHelper_MultiHelpers;
    Procedure TestRecordHelper;
    Procedure TestRecordHelper_ForByteFail;
    Procedure TestRecordHelper_ClassNonStaticFail;
    Procedure TestRecordHelper_InheritedObjFPC;
    Procedure TestRecordHelper_Constructor_NewInstance;
    Procedure TestTypeHelper;
    Procedure TestTypeHelper_HelperForProcTypeFail;
    Procedure TestTypeHelper_DefaultPropertyFail;
    Procedure TestTypeHelper_Enum;
    Procedure TestTypeHelper_EnumDotValueFail;
    Procedure TestTypeHelper_EnumHelperDotProcFail;
    Procedure TestTypeHelper_Set;
    Procedure TestTypeHelper_Enumerator;
    Procedure TestTypeHelper_String;
    Procedure TestTypeHelper_StringOtherUnit;
    Procedure TestTypeHelper_Boolean;
    Procedure TestTypeHelper_Double;
    Procedure TestTypeHelper_DoubleAlias;
    Procedure TestTypeHelper_Constructor_NewInstance;
    Procedure TestTypeHelper_Interface;
    Procedure TestTypeHelper_Interface_ConstructorFail;
    Procedure TestTypeHelper_TypeAliasType;

    // attributes
    Procedure TestAttributes_Globals;
    Procedure TestAttributes_NonConstParam_Fail;
    Procedure TestAttributes_UnknownAttrWarning;
    Procedure TestAttributes_Members;
  end;

function LinesToStr(Args: array of const): string;

implementation

function LinesToStr(Args: array of const): string;
var
  s: String;
  i: Integer;
begin
  s:='';
  for i:=Low(Args) to High(Args) do
    case Args[i].VType of
      vtChar:         s += Args[i].VChar+LineEnding;
      vtString:       s += Args[i].VString^+LineEnding;
      vtPChar:        s += Args[i].VPChar+LineEnding;
      vtWideChar:     s += AnsiString(Args[i].VWideChar)+LineEnding;
      vtPWideChar:    s += AnsiString(Args[i].VPWideChar)+LineEnding;
      vtAnsiString:   s += AnsiString(Args[i].VAnsiString)+LineEnding;
      vtWidestring:   s += AnsiString(WideString(Args[i].VWideString))+LineEnding;
      vtUnicodeString:s += AnsiString(UnicodeString(Args[i].VUnicodeString))+LineEnding;
    end;
  Result:=s;
end;

{ TTestEnginePasResolver }

procedure TTestEnginePasResolver.SetModule(AValue: TPasModule);
begin
  if FModule=AValue then Exit;
  if Module<>nil then
    Module.Release{$IFDEF CheckPasTreeRefCount}('TTestEnginePasResolver.Module'){$ENDIF};
  FModule:=AValue;
  {$IFDEF CheckPasTreeRefCount}
  if Module<>nil then
    Module.ChangeRefId('CreateElement','TTestEnginePasResolver.Module');
  {$ENDIF}
end;

constructor TTestEnginePasResolver.Create;
begin
  inherited Create;
  StoreSrcColumns:=true;
end;

destructor TTestEnginePasResolver.Destroy;
begin
  FStreamResolver:=nil;
  FreeAndNil(FParser);
  FreeAndNil(FScanner);
  inherited Destroy;
  Module:=nil;
end;

procedure TTestEnginePasResolver.ReleaseUsedUnits;
begin
  if Module<>nil then
    Module.ReleaseUsedUnits;
end;

function TTestEnginePasResolver.CreateElement(AClass: TPTreeElement;
  const AName: String; AParent: TPasElement; AVisibility: TPasMemberVisibility;
  const ASrcPos: TPasSourcePos; TypeParams: TFPList): TPasElement;
begin
  Result:=inherited CreateElement(AClass, AName, AParent, AVisibility, ASrcPos, TypeParams);
  if (FModule=nil) and AClass.InheritsFrom(TPasModule) then
    Module:=TPasModule(Result);
end;

function TTestEnginePasResolver.FindUnit(const AName, InFilename: String;
  NameExpr, InFileExpr: TPasExpr): TPasModule;
begin
  Result:=OnFindUnit(Self,AName,InFilename,NameExpr,InFileExpr);
end;

procedure TTestEnginePasResolver.UsedInterfacesFinished(Section: TPasSection);
begin
  if Section=nil then ;
  // do not parse recursively
  // using a queue
end;

{ TCustomTestResolver }

procedure TCustomTestResolver.SetUp;
begin
  {$IF defined(VerbosePasResolver) or defined(VerbosePasResolverMem)}
  FStartElementRefCount:=TPasElement.GlobalRefCount;
  {$ENDIF}
  FModules:=TObjectList.Create(true);
  inherited SetUp;
  Parser.Options:=Parser.Options+[po_ResolveStandardTypes];
  Scanner.OnDirective:=@OnScannerDirective;
  Scanner.OnLog:=@OnScannerLog;
end;

procedure TCustomTestResolver.TearDown;
{$IFDEF CheckPasTreeRefCount}
var El: TPasElement;
{$ENDIF}
var i: Integer;
begin
  FResolverMsgs.Clear;
  FResolverGoodMsgs.Clear;
  {$IFDEF VerbosePasResolverMem}
  writeln('TTestResolver.TearDown START FreeSrcMarkers');
  {$ENDIF}
  FreeSrcMarkers;
  {$IFDEF VerbosePasResolverMem}
  writeln('TTestResolver.TearDown ResolverEngine.Clear');
  {$ENDIF}
  if ResolverEngine.Parser=Parser then
    ResolverEngine.Parser:=nil;
  ResolverEngine.Clear;
  if FModules<>nil then
    begin
    {$IFDEF VerbosePasResolverMem}
    writeln('TTestResolver.TearDown FModules');
    {$ENDIF}
    for i:=0 to FModules.Count-1 do
      TTestEnginePasResolver(FModules[i]).ReleaseUsedUnits;
    FModules.OwnsObjects:=false;
    FModules.Remove(ResolverEngine); // remove reference
    FModules.OwnsObjects:=true;
    FreeAndNil(FModules);// free all other modules
    end;
  {$IFDEF VerbosePasResolverMem}
  writeln('TTestResolver.TearDown inherited');
  {$ENDIF}
  if Module<>nil then
    Module.AddRef{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF}; // for the Release in ancestor TTestParser
  inherited TearDown;
  FResolverEngine:=nil;
  {$IF defined(VerbosePasResolver) or defined(VerbosePasResolverMem)}
  if FStartElementRefCount<>TPasElement.GlobalRefCount then
    begin
    writeln('TCustomTestResolver.TearDown GlobalRefCount Was='+IntToStr(FStartElementRefCount)+' Now='+IntToStr(TPasElement.GlobalRefCount));
    {$IFDEF CheckPasTreeRefCount}
    El:=TPasElement.FirstRefEl;
    if El=nil then
      writeln('  TPasElement.FirstRefEl=nil');
    while El<>nil do
      begin
      writeln('  ',GetObjName(El),' RefIds.Count=',El.RefIds.Count,':');
      for i:=0 to El.RefIds.Count-1 do
        writeln('    ',El.RefIds[i]);
      El:=El.NextRefEl;
      end;
    {$ENDIF}
    //Halt;
    Fail('TCustomTestResolver.TearDown GlobalRefCount Was='+IntToStr(FStartElementRefCount)+' Now='+IntToStr(TPasElement.GlobalRefCount));
    end;
  {$ENDIF}
  {$IFDEF VerbosePasResolverMem}
  writeln('TTestResolver.TearDown END');
  {$ENDIF}
end;

procedure TCustomTestResolver.CreateEngine(var TheEngine: TPasTreeContainer);
begin
  FResolverEngine:=AddModule(MainFilename);
  TheEngine:=ResolverEngine;
end;

procedure TCustomTestResolver.ParseModule;
var
  Section: TPasSection;
  i: Integer;
  CurResolver: TTestEnginePasResolver;
  Found: Boolean;
begin
  if ResolverEngine.Parser=nil then
    ResolverEngine.Parser:=Parser;

  inherited ParseModule;
  repeat
    Found:=false;
    for i:=0 to ModuleCount-1 do
      begin
      CurResolver:=Modules[i];
      if CurResolver.Parser=nil then continue;
      if not CurResolver.Parser.CanParseContinue(Section) then
        continue;
      {$IFDEF VerbosePasResolver}
      writeln('TCustomTestResolver.ParseModule continue parsing section=',GetObjName(Section),' of ',CurResolver.Filename);
      {$ENDIF}
      Found:=true;
      CurResolver.Parser.ParseContinue;
      break;
      end;
  until not Found;

  for i:=0 to ModuleCount-1 do
    begin
    CurResolver:=Modules[i];
    if CurResolver.Parser=nil then
      begin
      if CurResolver.CurrentParser<>nil then
        Fail(CurResolver.Filename+' Parser<>CurrentParser Parser="'+GetObjName(CurResolver.Parser)+'" CurrentParser='+GetObjName(CurResolver.CurrentParser));
      continue;
      end;
    if CurResolver.Parser.CurModule<>nil then
      begin
      Section:=CurResolver.Parser.GetLastSection;
      {$IFDEF VerbosePasResolver}
      writeln('TCustomTestResolver.ParseModule module not finished "',GetObjName(CurResolver.RootElement),'" LastSection=',GetObjName(Section)+' PendingUsedIntf='+GetObjName(Section.PendingUsedIntf));
      if (Section<>nil) and (Section.PendingUsedIntf<>nil) then
        writeln('TCustomTestResolver.ParseModule PendingUsedIntf=',GetObjName(Section.PendingUsedIntf.Module));
      {$ENDIF}
      Fail('module not finished "'+GetObjName(CurResolver.RootElement)+'"');
      end;
    end;
end;

procedure TCustomTestResolver.ParseProgram;
var
  aFilename: String;
  aRow, aCol: Integer;
begin
  FFirstStatement:=nil;
  try
    ParseModule;
  except
    on E: EParserError do
      begin
      aFilename:=E.Filename;
      aRow:=E.Row;
      aCol:=E.Column;
      WriteSources(aFilename,aRow,aCol);
      writeln('ERROR: TTestResolver.ParseProgram Parser: '+E.ClassName+':'+E.Message,
        ' Scanner at'
        +' '+aFilename+'('+IntToStr(aRow)+','+IntToStr(aCol)+')'
        +' Line="'+Scanner.CurLine+'"');
      Fail(E.Message);
      end;
    on E: EPasResolve do
      begin
      aFilename:=Scanner.CurFilename;
      aRow:=Scanner.CurRow;
      aCol:=Scanner.CurColumn;
      if E.PasElement<>nil then
        begin
        aFilename:=E.PasElement.SourceFilename;
        ResolverEngine.UnmangleSourceLineNumber(E.PasElement.SourceLinenumber,aRow,aCol);
        end;
      WriteSources(aFilename,aRow,aCol);
      writeln('ERROR: TTestResolver.ParseProgram PasResolver: '+E.ClassName+':'+E.Message
        +' at '+aFilename+'('+IntToStr(aRow)+','+IntToStr(aCol)+')');
      Fail(E.Message);
      end;
    on E: Exception do
      begin
      writeln('ERROR: TTestResolver.ParseProgram Exception: '+E.ClassName+':'+E.Message);
      Fail(E.Message);
      end;
  end;
  TAssert.AssertSame('Has resolver',ResolverEngine,Parser.Engine);
  AssertEquals('Has program',TPasProgram,Module.ClassType);
  AssertNotNull('Has program section',PasProgram.ProgramSection);
  AssertNotNull('Has initialization section',PasProgram.InitializationSection);
  if (PasProgram.InitializationSection.Elements.Count>0) then
    if TObject(PasProgram.InitializationSection.Elements[0]) is TPasImplBlock then
      FFirstStatement:=TPasImplBlock(PasProgram.InitializationSection.Elements[0]);
  CheckReferenceDirectives;
end;

procedure TCustomTestResolver.ParseUnit;
begin
  FFirstStatement:=nil;
  try
    ParseModule;
  except
    on E: EParserError do
      begin
      writeln('ERROR: TTestResolver.ParseUnit Parser: '+E.ClassName+':'+E.Message
        +' File='+Scanner.CurFilename
        +' LineNo='+IntToStr(Scanner.CurRow)
        +' Col='+IntToStr(Scanner.CurColumn)
        +' Line="'+Scanner.CurLine+'"'
        );
      Fail(E.Message);
      end;
    on E: EPasResolve do
      begin
      writeln('ERROR: TTestResolver.ParseUnit PasResolver: '+E.ClassName+':'+E.Message
        +' File='+Scanner.CurFilename
        +' LineNo='+IntToStr(Scanner.CurRow)
        +' Col='+IntToStr(Scanner.CurColumn)
        +' Line="'+Scanner.CurLine+'"'
        );
      Fail(E.Message);
      end;
    on E: Exception do
      begin
      writeln('ERROR: TTestResolver.ParseUnit Exception: '+E.ClassName+':'+E.Message);
      Fail(E.Message);
      end;
  end;
  TAssert.AssertSame('Has resolver',ResolverEngine,Parser.Engine);
  AssertEquals('Has unit',TPasModule,Module.ClassType);
  AssertNotNull('Has interface section',Module.InterfaceSection);
  AssertNotNull('Has implementation section',Module.ImplementationSection);
  if (Module.InitializationSection<>nil)
  and (Module.InitializationSection.Elements.Count>0) then
    if TObject(Module.InitializationSection.Elements[0]) is TPasImplBlock then
      FFirstStatement:=TPasImplBlock(Module.InitializationSection.Elements[0]);
  CheckReferenceDirectives;
end;

procedure TCustomTestResolver.CheckReferenceDirectives;
var
  Filename: string;
  LineNumber: Integer;
  SrcLine: String;
  CommentStartP, CommentEndP: PChar;

  procedure RaiseError(Msg: string; p: PChar);
  begin
    RaiseErrorAtSrc(Msg,Filename,LineNumber,p-PChar(SrcLine)+1);
  end;

  procedure AddMarker(Marker: PSrcMarker);
  begin
    if LastSrcMarker<>nil then
      LastSrcMarker^.Next:=Marker
    else
      FirstSrcMarker:=Marker;
    LastSrcMarker:=Marker;
  end;

  function AddMarker(Kind: TSrcMarkerKind; const aFilename: string;
    aLine, aStartCol, aEndCol: integer; const Identifier: string): PSrcMarker;
  begin
    New(Result);
    Result^.Kind:=Kind;
    Result^.Filename:=aFilename;
    Result^.Row:=aLine;
    Result^.StartCol:=aStartCol;
    Result^.EndCol:=aEndCol;
    Result^.Identifier:=Identifier;
    Result^.Next:=nil;
    //writeln('AddMarker Line="',SrcLine,'" Identifier=',Identifier,' Col=',aStartCol,'-',aEndCol,' "',copy(SrcLine,aStartCol,aEndCol-aStartCol),'"');
    AddMarker(Result);
  end;

  function AddMarkerForTokenBehindComment(Kind: TSrcMarkerKind;
    const Identifier: string): PSrcMarker;
  var
    TokenStart, p: PChar;
  begin
    p:=CommentEndP;
    ReadNextPascalToken(p,TokenStart,false,false);
    Result:=AddMarker(Kind,Filename,LineNumber,
      CommentEndP-PChar(SrcLine)+1,p-PChar(SrcLine)+1,Identifier);
  end;

  function ReadIdentifier(var p: PChar): string;
  var
    StartP: PChar;
  begin
    if not (p^ in ['a'..'z','A'..'Z','_']) then
      RaiseError('identifier expected',p);
    StartP:=p;
    inc(p);
    while p^ in ['a'..'z','A'..'Z','_','0'..'9'] do inc(p);
    Result:='';
    SetLength(Result,p-StartP);
    Move(StartP^,Result[1],length(Result));
  end;

  procedure AddLabel;
  var
    Identifier: String;
    p: PChar;
  begin
    p:=CommentStartP+2;
    Identifier:=ReadIdentifier(p);
    //writeln('TTestResolver.CheckReferenceDirectives.AddLabel ',Identifier);
    if FindSrcLabel(Identifier)<>nil then
      RaiseError('duplicate label "'+Identifier+'"',p);
    AddMarkerForTokenBehindComment(mkLabel,Identifier);
  end;

  procedure AddResolverReference;
  var
    Identifier: String;
    p: PChar;
  begin
    p:=CommentStartP+2;
    Identifier:=ReadIdentifier(p);
    //writeln('TTestResolver.CheckReferenceDirectives.AddReference ',Identifier);
    AddMarkerForTokenBehindComment(mkResolverReference,Identifier);
  end;

  procedure AddDirectReference;
  var
    Identifier: String;
    p: PChar;
  begin
    p:=CommentStartP+2;
    Identifier:=ReadIdentifier(p);
    //writeln('TTestResolver.CheckReferenceDirectives.AddDirectReference ',Identifier);
    AddMarkerForTokenBehindComment(mkDirectReference,Identifier);
  end;

  procedure ParseCode(SrcLines: TStringList; aFilename: string);
  var
    p: PChar;
    IsDirective: Boolean;
  begin
    //writeln('TTestResolver.CheckReferenceDirectives.ParseCode File=',aFilename);
    Filename:=aFilename;
    // parse code, find all labels
    LineNumber:=0;
    while LineNumber<SrcLines.Count do
      begin
      inc(LineNumber);
      SrcLine:=SrcLines[LineNumber-1];
      if SrcLine='' then continue;
      //writeln('TTestResolver.CheckReferenceDirectives Line=',SrcLine);
      p:=PChar(SrcLine);
      repeat
        case p^ of
          #0: if (p-PChar(SrcLine)=length(SrcLine)) then break;
          '{':
            begin
            CommentStartP:=p;
            inc(p);
            IsDirective:=p^ in ['#','@','='];

            // skip to end of comment
            repeat
              case p^ of
              #0:
                if (p-PChar(SrcLine)=length(SrcLine)) then
                  begin
                  // multi line comment
                  if IsDirective then
                    RaiseError('directive missing closing bracket',CommentStartP);
                  repeat
                    inc(LineNumber);
                    if LineNumber>SrcLines.Count then exit;
                    SrcLine:=SrcLines[LineNumber-1];
                    //writeln('TTestResolver.CheckReferenceDirectives Comment Line=',SrcLine);
                  until SrcLine<>'';
                  p:=PChar(SrcLine);
                  continue;
                  end;
              '}':
                begin
                inc(p);
                break;
                end;
              end;
              inc(p);
            until false;

            CommentEndP:=p;
            case CommentStartP[1] of
            '#': AddLabel;
            '@': AddResolverReference;
            '=': AddDirectReference;
            end;
            p:=CommentEndP;
            continue;

            end;
          '/':
            if p[1]='/' then
              break; // rest of line is comment -> skip
        end;
        inc(p);
      until false;
      end;
  end;

  procedure CheckResolverReference(aMarker: PSrcMarker);
  // check if one element at {@a} has a TResolvedReference to an element labeled {#a}
  var
    aLabel: PSrcMarker;
    ReferenceElements, LabelElements: TFPList;
    i, j, aLine, aCol: Integer;
    El, Ref, LabelEl: TPasElement;
  begin
    //writeln('CheckResolverReference searching reference: ',aMarker^.Filename,' Line=',aMarker^.Row,' Col=',aMarker^.StartCol,'-',aMarker^.EndCol,' Label="',aMarker^.Identifier,'"');
    aLabel:=FindSrcLabel(aMarker^.Identifier);
    if aLabel=nil then
      RaiseErrorAtSrc('label "'+aMarker^.Identifier+'" not found',aMarker^.Filename,aMarker^.Row,aMarker^.StartCol);

    LabelElements:=nil;
    ReferenceElements:=nil;
    try
      LabelElements:=FindElementsAt(aLabel);
      ReferenceElements:=FindElementsAt(aMarker);

      for i:=0 to ReferenceElements.Count-1 do
        begin
        El:=TPasElement(ReferenceElements[i]);
        Ref:=nil;
        if El.CustomData is TResolvedReference then
          Ref:=TResolvedReference(El.CustomData).Declaration
        else if El.CustomData is TPasPropertyScope then
          Ref:=TPasPropertyScope(El.CustomData).AncestorProp
        else if El.CustomData is TPasSpecializeTypeData then
          Ref:=TPasSpecializeTypeData(El.CustomData).SpecializedType;
        if Ref<>nil then
          for j:=0 to LabelElements.Count-1 do
            begin
            LabelEl:=TPasElement(LabelElements[j]);
            if Ref=LabelEl then
              exit; // success
            end;
        end;

      // failure write candidates
      for i:=0 to ReferenceElements.Count-1 do
        begin
        El:=TPasElement(ReferenceElements[i]);
        write('Reference candidate for "',aMarker^.Identifier,'" at reference ',aMarker^.Filename,'(',aMarker^.Row,',',aMarker^.StartCol,'-',aMarker^.EndCol,')');
        write(' El=',GetObjName(El));
        if EL is TPrimitiveExpr then
          begin
           writeln('CheckResolverReference ',TPrimitiveExpr(El).Value);
          end;
        Ref:=nil;
        if El.CustomData is TResolvedReference then
          Ref:=TResolvedReference(El.CustomData).Declaration
        else if El.CustomData is TPasPropertyScope then
          Ref:=TPasPropertyScope(El.CustomData).AncestorProp
        else if El.CustomData is TPasSpecializeTypeData then
          Ref:=TPasSpecializeTypeData(El.CustomData).SpecializedType;
        if Ref<>nil then
          begin
          write(' Decl=',GetObjName(Ref));
          ResolverEngine.UnmangleSourceLineNumber(Ref.SourceLinenumber,aLine,aCol);
          write(',',Ref.SourceFilename,'(',aLine,',',aCol,')');
          end
        else
          write(' has no TResolvedReference. El.CustomData=',GetObjName(El.CustomData));
        writeln;
        end;
      for i:=0 to LabelElements.Count-1 do
        begin
        El:=TPasElement(LabelElements[i]);
        write('Label candidate for "',aLabel^.Identifier,'" at reference ',aLabel^.Filename,'(',aLabel^.Row,',',aLabel^.StartCol,'-',aLabel^.EndCol,')');
        write(' El=',GetObjName(El));
        writeln;
        end;

      RaiseErrorAtSrcMarker('wrong resolved reference "'+aMarker^.Identifier+'"',aMarker);
    finally
      LabelElements.Free;
      ReferenceElements.Free;
    end;
  end;

  procedure CheckDirectReference(aMarker: PSrcMarker);
  // check if one element at {=a} is a TPasAliasType pointing to an element labeled {#a}
  var
    aLabel: PSrcMarker;
    ReferenceElements, LabelElements: TFPList;
    i, LabelLine, LabelCol, j: Integer;
    El, LabelEl: TPasElement;
    DeclEl, TypeEl: TPasType;
  begin
    //writeln('CheckDirectReference searching pointer: ',aMarker^.Filename,' Line=',aMarker^.Row,' Col=',aMarker^.StartCol,'-',aMarker^.EndCol,' Label="',aMarker^.Identifier,'"');
    aLabel:=FindSrcLabel(aMarker^.Identifier);
    if aLabel=nil then
      RaiseErrorAtSrcMarker('label "'+aMarker^.Identifier+'" not found',aMarker);

    LabelElements:=nil;
    ReferenceElements:=nil;
    try
      //writeln('CheckDirectReference finding elements at label ...');
      LabelElements:=FindElementsAt(aLabel);
      //writeln('CheckDirectReference finding elements at reference ...');
      ReferenceElements:=FindElementsAt(aMarker);

      for i:=0 to ReferenceElements.Count-1 do
        begin
        El:=TPasElement(ReferenceElements[i]);
        //writeln('CheckDirectReference ',i,'/',ReferenceElements.Count,' ',GetTreeDbg(El,2));
        if El.ClassType=TPasVariable then
          begin
          if TPasVariable(El).VarType=nil then
            begin
            //writeln('CheckDirectReference Var without Type: ',GetObjName(El),' El.Parent=',GetObjName(El.Parent));
            AssertNotNull('TPasVariable(El='+El.Name+').VarType',TPasVariable(El).VarType);
            end;
          TypeEl:=TPasVariable(El).VarType;
          for j:=0 to LabelElements.Count-1 do
            begin
            LabelEl:=TPasElement(LabelElements[j]);
            if TypeEl=LabelEl then
              exit; // success
            end;
          end
        else if El is TPasAliasType then
          begin
          DeclEl:=TPasAliasType(El).DestType;
          ResolverEngine.UnmangleSourceLineNumber(DeclEl.SourceLinenumber,LabelLine,LabelCol);
          if (aLabel^.Filename=DeclEl.SourceFilename)
          and (integer(aLabel^.Row)=LabelLine)
          and (aLabel^.StartCol<=LabelCol)
          and (aLabel^.EndCol>=LabelCol) then
            exit; // success
          end
        else if El.ClassType=TPasArgument then
          begin
          TypeEl:=TPasArgument(El).ArgType;
          for j:=0 to LabelElements.Count-1 do
            begin
            LabelEl:=TPasElement(LabelElements[j]);
            if TypeEl=LabelEl then
              exit; // success
            end;
          end;
        end;
      // failed -> show candidates
      writeln('CheckDirectReference failed: Labels:');
      for j:=0 to LabelElements.Count-1 do
        begin
        LabelEl:=TPasElement(LabelElements[j]);
        writeln('  Label ',GetObjName(LabelEl),' at ',ResolverEngine.GetElementSourcePosStr(LabelEl));
        end;
      writeln('CheckDirectReference failed: References:');
      for i:=0 to ReferenceElements.Count-1 do
        begin
        El:=TPasElement(ReferenceElements[i]);
        writeln('  Reference ',GetObjName(El),' at ',ResolverEngine.GetElementSourcePosStr(El));
        //if EL is TPasVariable then
        //  writeln('CheckDirectReference ',GetObjPath(TPasVariable(El).VarType),' ',ResolverEngine.GetElementSourcePosStr(TPasVariable(EL).VarType));
        end;
      RaiseErrorAtSrcMarker('wrong direct reference "'+aMarker^.Identifier+'"',aMarker);
    finally
      LabelElements.Free;
      ReferenceElements.Free;
    end;
  end;

var
  aMarker: PSrcMarker;
  i: Integer;
  SrcLines: TStringList;
begin
  Module.ForEachCall(@OnCheckElementParent,nil);
  //writeln('TTestResolver.CheckReferenceDirectives find all markers');
  // find all markers
  for i:=0 to Resolver.Streams.Count-1 do
    begin
    GetSrc(i,SrcLines,Filename);
    ParseCode(SrcLines,Filename);
    SrcLines.Free;
    end;

  //writeln('TTestResolver.CheckReferenceDirectives check references');
  // check references
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    case aMarker^.Kind of
    mkResolverReference: CheckResolverReference(aMarker);
    mkDirectReference: CheckDirectReference(aMarker);
    end;
    aMarker:=aMarker^.Next;
    end;
  //writeln('TTestResolver.CheckReferenceDirectives COMPLETE');
end;

procedure TCustomTestResolver.CheckResolverHint(MsgType: TMessageType;
  MsgNumber: integer; Msg: string; Marker: PSrcMarker);
var
  i: Integer;
  Item: TTestResolverMessage;
  Expected,Actual: string;
begin
  //writeln('TCustomTestResolver.CheckResolverHint MsgCount=',MsgCount);
  for i:=0 to MsgCount-1 do
    begin
    Item:=Msgs[i];
    if (Item.MsgNumber<>MsgNumber) or (Item.Msg<>Msg) then continue;
    if (Marker<>nil) then
      begin
      if Item.SourcePos.Row<>Marker^.Row then continue;
      if (integer(Item.SourcePos.Column)<Marker^.StartCol)
          or (integer(Item.SourcePos.Column)>Marker^.EndCol) then continue;
      end;
    // found
    FResolverGoodMsgs.Add(Item);
    str(Item.MsgType,Actual);
    str(MsgType,Expected);
    AssertEquals('MsgType',Expected,Actual);
    exit;
    end;

  // needed message missing -> show emitted messages
  WriteSources('',0,0);
  for i:=0 to MsgCount-1 do
    begin
    Item:=Msgs[i];
    write('TCustomTestResolver.CheckResolverHint ',i,'/',MsgCount,' ',Item.MsgType,
      ' ('+IntToStr(Item.MsgNumber),')');
    if Marker<>nil then
      write(' '+ExtractFileName(Item.SourcePos.FileName),'(',Item.SourcePos.Row,',',Item.SourcePos.Column,')');
    writeln(' {',Item.Msg,'}');
    end;
  str(MsgType,Expected);
  Actual:='Missing '+Expected+' ('+IntToStr(MsgNumber)+')';
  if Marker<>nil then
    Actual:=Actual+' '+ExtractFileName(Marker^.Filename)+'('+IntToStr(Marker^.Row)+','+IntToStr(Marker^.StartCol)+'..'+IntToStr(Marker^.EndCol)+')';
  Actual:=Actual+' '+Msg;
  Fail(Actual);
end;

procedure TCustomTestResolver.CheckResolverUnexpectedHints(
  WithSourcePos: boolean);
var
  i: Integer;
  s, Txt: String;
  Msg: TTestResolverMessage;
begin
  for i:=0 to MsgCount-1 do
    begin
    Msg:=Msgs[i];
    if FResolverGoodMsgs.IndexOf(Msg)>=0 then continue;
    s:='';
    str(Msg.MsgType,s);
    Txt:='Unexpected resolver message found ['+IntToStr(Msg.Id)+'] '
      +s+': ('+IntToStr(Msg.MsgNumber)+')';
    if WithSourcePos then
      Txt:=Txt+' '+ExtractFileName(Msg.SourcePos.FileName)+'('+IntToStr(Msg.SourcePos.Row)+','+IntToStr(Msg.SourcePos.Column)+')';
    Txt:=Txt+' {'+Msg.Msg+'}';
    Fail(Txt);
    end;
end;

procedure TCustomTestResolver.CheckResolverException(Msg: string; MsgNumber: integer);
var
  ok: Boolean;
  Full: String;
begin
  ok:=false;
  try
    ParseModule;
  except
    on E: EPasResolve do
      begin
      AssertEquals('Expected {'+Msg+'}, but got msg {'+E.Message+'} number',
        MsgNumber,E.MsgNumber);
      Full:=E.Message+' at '+E.SourcePos.FileName+' ('+IntToStr(E.SourcePos.Row)+','+IntToStr(E.SourcePos.Column)+')';
      if (Msg<>E.Message) and (Msg<>E.MsgPattern) and (Msg<>Full) then
        begin
        {$IFDEF VerbosePasResolver}
        writeln('TCustomTestResolver.CheckResolverException E.MsgPattern={',E.MsgPattern,'} E.Message={',E.Message,'} Full={',Full,'}');
        {$ENDIF}
        AssertEquals('Expected message ('+IntToStr(MsgNumber)+')',
          '{'+Msg+'}','{'+E.Message+'} OR {'+E.MsgPattern+'} OR {'+Full+'}');
        end;
      ok:=true;
      end;
  end;
  AssertEquals('Missing resolver error {'+Msg+'} ('+IntToStr(MsgNumber)+')',true,ok);
end;

procedure TCustomTestResolver.CheckParserException(Msg: string; MsgNumber: integer);
var
  ok: Boolean;
begin
  ok:=false;
  try
    ParseModule;
  except
    on E: EParserError do
      begin
      if (Parser.LastMsg<>Msg) and (Parser.LastMsgPattern<>Msg) and (E.Message<>Msg) then
        Fail('Expected msg {'+Msg+'}, but got {'+Parser.LastMsg+'} OR pattern {'+Parser.LastMsgPattern+'} OR E.Message {'+E.Message+'}');
      AssertEquals('Expected {'+Msg+'}, but got msg {'+E.Message+'} number',
        MsgNumber,Parser.LastMsgNumber);
      ok:=true;
      end;
  end;
  AssertEquals('Missing parser error '+Msg+' ('+IntToStr(MsgNumber)+')',true,ok);
end;

procedure TCustomTestResolver.CheckAccessMarkers;
const
  AccessNames: array[TResolvedRefAccess] of string = (
    'none',
    'read',
    'assign',
    'readandassign',
    'var',
    'out',
    'paramtest'
    );
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualAccess, ExpectedAccess: TResolvedRefAccess;
  i, j: Integer;
  El, El2: TPasElement;
  Ref: TResolvedReference;
  p: SizeInt;
  AccessPostfix: String;
begin
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.CheckAccessMarkers ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    p:=RPos('_',aMarker^.Identifier);
    if p>1 then
      begin
      AccessPostfix:=copy(aMarker^.Identifier,p+1);
      ExpectedAccess:=High(TResolvedRefAccess);
      repeat
        if CompareText(AccessPostfix,AccessNames[ExpectedAccess])=0 then break;
        if ExpectedAccess=Low(TResolvedRefAccess) then
          RaiseErrorAtSrcMarker('unknown access postfix of reference at "#'+aMarker^.Identifier+'"',aMarker);
        ExpectedAccess:=Pred(ExpectedAccess);
      until false;

      Elements:=FindElementsAt(aMarker);
      try
        ActualAccess:=rraNone;
        for i:=0 to Elements.Count-1 do
          begin
          El:=TPasElement(Elements[i]);
          //writeln('TTestResolver.CheckAccessMarkers ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
          if not (El.CustomData is TResolvedReference) then continue;
          Ref:=TResolvedReference(El.CustomData);
          if ActualAccess<>rraNone then
            begin
            //writeln('TTestResolver.CheckAccessMarkers multiple references at "#'+aMarker^.Identifier+'":');
            for j:=0 to Elements.Count-1 do
              begin
              El2:=TPasElement(Elements[i]);
              if not (El2.CustomData is TResolvedReference) then continue;
              //writeln('TTestResolver.CheckAccessMarkers ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
              Ref:=TResolvedReference(El.CustomData);
              //writeln('  ',j,'/',Elements.Count,' Element=',GetObjName(El2),' ',AccessNames[Ref.Access],' Declaration="',El2.GetDeclaration(true),'"');
              end;
            RaiseErrorAtSrcMarker('multiple references at "#'+aMarker^.Identifier+'"',aMarker);
            end;
          ActualAccess:=Ref.Access;
          if ActualAccess=rraNone then
            RaiseErrorAtSrcMarker('missing Access in reference at "#'+aMarker^.Identifier+'"',aMarker);
          end;
        if ActualAccess<>ExpectedAccess then
          RaiseErrorAtSrcMarker('expected "'+AccessNames[ExpectedAccess]+'" at "#'+aMarker^.Identifier+'", but got "'+AccessNames[ActualAccess]+'"',aMarker);
      finally
        Elements.Free;
      end;
      end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TCustomTestResolver.CheckParamsExpr_pkSet_Markers;
// e.g. {#a_set}  {#b_array}
var
  aMarker: PSrcMarker;
  p: SizeInt;
  AccessPostfix: String;
  Elements: TFPList;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
  ParamsExpr: TParamsExpr;
  NeedArray: Boolean;
begin
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.CheckParamsExpr_pkSet_Markers ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    p:=RPos('_',aMarker^.Identifier);
    if p>1 then
      begin
      AccessPostfix:=copy(aMarker^.Identifier,p+1);
      if SameText(AccessPostfix,'set') then
        NeedArray:=false
      else if SameText(AccessPostfix,'array') then
        NeedArray:=true
      else
        RaiseErrorAtSrcMarker('unknown set/array postfix of [] expression at "#'+aMarker^.Identifier+'"',aMarker);

      Elements:=FindElementsAt(aMarker);
      try
        ParamsExpr:=nil;
        for i:=0 to Elements.Count-1 do
          begin
          El:=TPasElement(Elements[i]);
          //writeln('TTestResolver.CheckParamsExpr_pkSet_Markers ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
          if El.ClassType<>TParamsExpr then continue;
          if ParamsExpr<>nil then
            RaiseErrorAtSrcMarker('multiple paramsexpr found at "#'+aMarker^.Identifier+'"',aMarker);

          ParamsExpr:=TParamsExpr(El);

          if NeedArray then
            begin
            if not (El.CustomData is TResolvedReference) then
              RaiseErrorAtSrcMarker('array expr has no TResolvedReference at "#'+aMarker^.Identifier+'"',aMarker);
            Ref:=TResolvedReference(El.CustomData);
            if not (Ref.Declaration is TPasArrayType) then
              RaiseErrorAtSrcMarker('array expr Ref.Decl is not TPasArrayType (is '+GetObjName(Ref.Declaration)+') at "#'+aMarker^.Identifier+'"',aMarker);
            end
          else
            begin
            if not (El.CustomData is TResolvedReference) then
              continue; // good
            Ref:=TResolvedReference(El.CustomData);
            if Ref.Declaration is TPasArrayType then
              RaiseErrorAtSrcMarker('set expr Ref.Decl is '+GetObjName(Ref.Declaration)+' at "#'+aMarker^.Identifier+'"',aMarker);
            end;
          end;
        if TParamsExpr=nil then
          RaiseErrorAtSrcMarker('missing paramsexpr at "#'+aMarker^.Identifier+'"',aMarker);
      finally
        Elements.Free;
      end;
      end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TCustomTestResolver.CheckAttributeMarkers;
// check markers of the form {#Attr__ClassMarker__ConstructorMarker[__OptionalName]}
var
  aMarker, ClassMarker, ConstructorMarker: PSrcMarker;
  Elements: TFPList;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
  s, ClassMarkerName, ConstructorMarkerName: String;
  p: SizeInt;
  ExpectedClass: TPasClassType;
  ExpectedConstrucor, ActualConstructor: TPasConstructor;
begin
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    s:=aMarker^.Identifier;
    if SameText(LeftStr(s,6),'Attr__') then
      begin
      //writeln('TCustomTestResolver.CheckAttributeMarkers ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
      Delete(s,1,6);
      p:=Pos('__',s);
      if p<1 then
        RaiseErrorAtSrcMarker('missing second __ at "#'+aMarker^.Identifier+'"',aMarker);
      ClassMarkerName:=LeftStr(s,p-1);
      Delete(s,1,p+1);
      p:=Pos('__',s);
      if p<1 then
        ConstructorMarkerName:=s
      else
        ConstructorMarkerName:=copy(s,1,p-1);

      // find attribute class at ClassMarkerName
      ClassMarker:=FindSrcLabel(ClassMarkerName);
      if ClassMarker=nil then
        RaiseErrorAtSrcMarker('ClassMarker "'+ClassMarkerName+'" not found at "#'+aMarker^.Identifier+'"',aMarker);
      ExpectedClass:=nil;
      Elements:=FindElementsAt(ClassMarker);
      try
        for i:=0 to Elements.Count-1 do
          begin
          El:=TPasElement(Elements[i]);
          if El is TPasClassType then
            begin
            ExpectedClass:=TPasClassType(El);
            break;
            end;
          end;
        if ExpectedClass=nil then
          RaiseErrorAtSrcMarker('ClassMarker "'+ClassMarkerName+'" at "#'+aMarker^.Identifier+'" has no TPasClassType',aMarker);
      finally
        Elements.Free;
      end;

      // find constructor at ConstructorMarkerName
      ConstructorMarker:=FindSrcLabel(ConstructorMarkerName);
      if ConstructorMarker=nil then
        RaiseErrorAtSrcMarker('ConstructorMarker "'+ConstructorMarkerName+'" not found at "#'+aMarker^.Identifier+'"',aMarker);
      ExpectedConstrucor:=nil;
      Elements:=FindElementsAt(ConstructorMarker);
      try
        for i:=0 to Elements.Count-1 do
          begin
          El:=TPasElement(Elements[i]);
          if El is TPasConstructor then
            begin
            ExpectedConstrucor:=TPasConstructor(El);
            break;
            end;
          end;
        if ExpectedConstrucor=nil then
          RaiseErrorAtSrcMarker('ConstructorMarker "'+ConstructorMarkerName+'" at "#'+aMarker^.Identifier+'" has no TPasConstructor',aMarker);
      finally
        Elements.Free;
      end;

      Elements:=FindElementsAt(aMarker);
      try
        for i:=0 to Elements.Count-1 do
          begin
          El:=TPasElement(Elements[i]);
          //writeln('TCustomTestResolver.CheckAttributeMarkers ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
          if not (El.CustomData is TResolvedReference) then continue;
          Ref:=TResolvedReference(El.CustomData);
          if Ref.Declaration<>ExpectedClass then
            RaiseErrorAtSrcMarker('Ref.Declaration at "#'+aMarker^.Identifier+'", expected "'+ExpectedClass.FullName+'" but found "'+Ref.Declaration.FullName+'", El='+GetObjName(El),aMarker);
          if not (Ref.Context is TResolvedRefCtxAttrProc) then
            RaiseErrorAtSrcMarker('Ref.Context at "#'+aMarker^.Identifier+'", expected "TResolvedRefCtxAttrConstructor" but found "'+GetObjName(Ref.Context)+'", El='+GetObjName(El),aMarker);
          ActualConstructor:=TResolvedRefCtxAttrProc(Ref.Context).Proc;
          if ActualConstructor<>ExpectedConstrucor then
            RaiseErrorAtSrcMarker('Ref.Context.Proc at "#'+aMarker^.Identifier+'", expected "'+ExpectedConstrucor.FullName+'" but found "'+ActualConstructor.FullName+'", El='+GetObjName(El),aMarker);
          break;
          end;
      finally
        Elements.Free;
      end;
      end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TCustomTestResolver.GetSrc(Index: integer; out SrcLines: TStringList; out
  aFilename: string);
var
  aStream: TStream;
begin
  SrcLines:=TStringList.Create;
  aStream:=Resolver.Streams.Objects[Index] as TStream;
  aStream.Position:=0;
  SrcLines.LoadFromStream(aStream);
  aFilename:=Resolver.Streams[Index];
end;

function TCustomTestResolver.FindElementsAt(aFilename: string; aLine, aStartCol,
  aEndCol: integer): TFPList;
var
  ok: Boolean;
  FoundRefs: TTestResolverReferenceData;
  i: Integer;
  CurResolver: TTestEnginePasResolver;
begin
  //writeln('TCustomTestResolver.FindElementsAt START "',aFilename,'" Line=',aLine,' Col=',aStartCol,'-',aEndCol);
  FoundRefs:=Default(TTestResolverReferenceData);
  FoundRefs.Filename:=aFilename;
  FoundRefs.Row:=aLine;
  FoundRefs.StartCol:=aStartCol;
  FoundRefs.EndCol:=aEndCol;
  FoundRefs.Found:=TFPList.Create;
  ok:=false;
  try
    // find all markers
    Module.ForEachCall(@OnFindReference,@FoundRefs);
    for i:=0 to ModuleCount-1 do
      begin
      CurResolver:=Modules[i];
      if CurResolver.Module=Module then continue;
      //writeln('TCustomTestResolver.FindElementsAt ',CurResolver.Filename);
      CurResolver.Module.ForEachCall(@OnFindReference,@FoundRefs);
      end;
    ok:=true;
  finally
    if not ok then
      FreeAndNil(FoundRefs.Found);
  end;
  Result:=FoundRefs.Found;
  FoundRefs.Found:=nil;
end;

function TCustomTestResolver.FindElementsAt(aMarker: PSrcMarker;
  ErrorOnNoElements: boolean): TFPList;
begin
  Result:=FindElementsAt(aMarker^.Filename,aMarker^.Row,aMarker^.StartCol,aMarker^.EndCol);
  if ErrorOnNoElements and ((Result=nil) or (Result.Count=0)) then
    RaiseErrorAtSrcMarker('marker '+SrcMarker[aMarker^.Kind]+aMarker^.Identifier+' has no elements',aMarker);
end;

function TCustomTestResolver.FindSrcLabel(const Identifier: string): PSrcMarker;
begin
  Result:=FirstSrcMarker;
  while Result<>nil do
    begin
    if (Result^.Kind=mkLabel)
    and (CompareText(Result^.Identifier,Identifier)=0) then
      exit;
    Result:=Result^.Next;
    end;
end;

function TCustomTestResolver.FindElementsAtSrcLabel(const Identifier: string;
  ErrorOnNoElements: boolean): TFPList;
var
  SrcLabel: PSrcMarker;
begin
  SrcLabel:=FindSrcLabel(Identifier);
  if SrcLabel=nil then
    Fail('missing label "'+Identifier+'"');
  Result:=FindElementsAt(SrcLabel,ErrorOnNoElements);
end;

procedure TCustomTestResolver.WriteSources(const aFilename: string; aRow,
  aCol: integer);
var
  IsSrc: Boolean;
  i, j: Integer;
  SrcLines: TStringList;
  SrcFilename, Line: string;
begin
  for i:=0 to Resolver.Streams.Count-1 do
    begin
    GetSrc(i,SrcLines,SrcFilename);
    IsSrc:=ExtractFilename(SrcFilename)=ExtractFileName(aFilename);
    writeln('Testcode:-File="',SrcFilename,'"----------------------------------:');
    for j:=1 to SrcLines.Count do
      begin
      Line:=SrcLines[j-1];
      if IsSrc and (j=aRow) then
        begin
        write('*');
        Line:=LeftStr(Line,aCol-1)+'|'+copy(Line,aCol,length(Line));
        end;
      writeln(Format('%:4d: ',[j]),Line);
      end;
    SrcLines.Free;
    end;
end;

procedure TCustomTestResolver.RaiseErrorAtSrc(Msg: string; const aFilename: string;
  aRow, aCol: integer);
var
  s: String;
begin
  WriteSources(aFilename,aRow,aCol);
  s:='[TTestResolver.RaiseErrorAtSrc] '+aFilename+'('+IntToStr(aRow)+','+IntToStr(aCol)+') Error: '+Msg;
  writeln('ERROR: ',s);
  Fail(s);
end;

procedure TCustomTestResolver.RaiseErrorAtSrcMarker(Msg: string; aMarker: PSrcMarker);
begin
  RaiseErrorAtSrc(Msg,aMarker^.Filename,aMarker^.Row,aMarker^.StartCol);
end;

procedure TCustomTestResolver.HandleError(CurEngine: TTestEnginePasResolver;
  E: Exception);
var
  ErrFilename: String;
  ErrRow, ErrCol: Integer;
begin
  ErrFilename:=CurEngine.Scanner.CurFilename;
  ErrRow:=CurEngine.Scanner.CurRow;
  ErrCol:=CurEngine.Scanner.CurColumn;
  writeln('ERROR: TCustomTestResolver.HandleError during parsing: '+E.ClassName+':'+E.Message
    +' File='+ErrFilename
    +' LineNo='+IntToStr(ErrRow)
    +' Col='+IntToStr(ErrCol)
    +' Line="'+CurEngine.Scanner.CurLine+'"'
    );
  WriteSources(ErrFilename,ErrRow,ErrCol);
  Fail(E.Message);
end;

constructor TCustomTestResolver.Create;
begin
  inherited Create;
  FResolverMsgs:=TObjectList.Create(true);
  FResolverGoodMsgs:=TFPList.Create;
end;

destructor TCustomTestResolver.Destroy;
begin
  FreeAndNil(FResolverMsgs);
  FreeAndNil(FResolverGoodMsgs);
  inherited Destroy;
end;

function TCustomTestResolver.FindModuleWithFilename(aFilename: string
  ): TTestEnginePasResolver;
var
  i: Integer;
begin
  for i:=0 to ModuleCount-1 do
    if CompareText(Modules[i].Filename,aFilename)=0 then
      exit(Modules[i]);
  Result:=nil;
end;

function TCustomTestResolver.AddModule(aFilename: string): TTestEnginePasResolver;
begin
  //writeln('TTestResolver.AddModule ',aFilename);
  if FindModuleWithFilename(aFilename)<>nil then
    Fail('TTestResolver.AddModule: file "'+aFilename+'" already exists');
  Result:=TTestEnginePasResolver.Create;
  Result.Filename:=aFilename;
  Result.AddObjFPCBuiltInIdentifiers;
  Result.OnFindUnit:=@OnPasResolverFindUnit;
  Result.OnLog:=@OnPasResolverLog;
  FModules.Add(Result);
end;

function TCustomTestResolver.AddModuleWithSrc(aFilename, Src: string
  ): TTestEnginePasResolver;
begin
  Result:=AddModule(aFilename);
  Result.Source:=Src;
end;

function TCustomTestResolver.AddModuleWithIntfImplSrc(aFilename, InterfaceSrc,
  ImplementationSrc: string): TTestEnginePasResolver;
var
  Src: String;
begin
  Src:='unit '+ExtractFileUnitName(aFilename)+';'+LineEnding;
  Src+=LineEnding;
  Src+='interface'+LineEnding;
  Src+=LineEnding;
  Src+=InterfaceSrc;
  Src+='implementation'+LineEnding;
  Src+=LineEnding;
  Src+=ImplementationSrc;
  Src+='end.'+LineEnding;
  Result:=AddModuleWithSrc(aFilename,Src);
end;

procedure TCustomTestResolver.AddSystemUnit(Parts: TSystemUnitParts);
var
  Intf, Impl: TStringList;
begin
  Intf:=TStringList.Create;
  // interface
  Intf.Add('type');
  Intf.Add('  integer=longint;');
  Intf.Add('  sizeint=int64;');
    //'const',
    //'  LineEnding = #10;',
    //'  DirectorySeparator = ''/'';',
    //'  DriveSeparator = '''';',
    //'  AllowDirectorySeparators : set of char = [''\'',''/''];',
    //'  AllowDriveSeparators : set of char = [];',
  if supTObject in Parts then
    begin
    Intf.AddStrings([
    'type',
    '  TClass = class of TObject;',
    '  TObject = class',
    '    constructor Create;',
    '    destructor Destroy; virtual;',
    '    class function ClassType: TClass; assembler;',
    '    class function ClassName: String; assembler;',
    '    class function ClassNameIs(const Name: string): boolean;',
    '    class function ClassParent: TClass; assembler;',
    '    class function InheritsFrom(aClass: TClass): boolean; assembler;',
    '    class function UnitName: String; assembler;',
    '    procedure AfterConstruction; virtual;',
    '    procedure BeforeDestruction;virtual;',
    '    function Equals(Obj: TObject): boolean; virtual;',
    '    function ToString: String; virtual;',
    '  end;']);
    end;
  if supTVarRec in Parts then
    begin
    Intf.AddStrings([
    'const',
    '  vtInteger       = 0;',
    '  vtBoolean       = 1;',
    'type',
    '  PVarRec = ^TVarRec;',
    '  TVarRec = record',
    '    case VType : sizeint of',
    '    vtInteger       : (VInteger: Longint);',
    '    vtBoolean       : (VBoolean: Boolean);',
    '  end;']);
    end;
  Intf.Add('var');
  Intf.Add('  ExitCode: Longint = 0;');

  // implementation
  Impl:=TStringList.Create;
  if supTObject in Parts then
    begin
    Impl.AddStrings([
      '// needed by ClassNameIs, the real SameText is in SysUtils',
      'function SameText(const s1, s2: String): Boolean; assembler;',
      'asm',
      'end;',
      'constructor TObject.Create; begin end;',
      'destructor TObject.Destroy; begin end;',
      'class function TObject.ClassType: TClass; assembler;',
      'asm',
      'end;',
      'class function TObject.ClassName: String; assembler;',
      'asm',
      'end;',
      'class function TObject.ClassNameIs(const Name: string): boolean;',
      'begin',
      '  Result:=SameText(Name,ClassName);',
      'end;',
      'class function TObject.ClassParent: TClass; assembler;',
      'asm',
      'end;',
      'class function TObject.InheritsFrom(aClass: TClass): boolean; assembler;',
      'asm',
      'end;',
      'class function TObject.UnitName: String; assembler;',
      'asm',
      'end;',
      'procedure TObject.AfterConstruction; begin end;',
      'procedure TObject.BeforeDestruction; begin end;',
      'function TObject.Equals(Obj: TObject): boolean;',
      'begin',
      '  Result:=Obj=Self;',
      'end;',
      'function TObject.ToString: String;',
      'begin',
      '  Result:=ClassName;',
      'end;'
      ]);
    end;

  try
    AddModuleWithIntfImplSrc('system.pp',Intf.Text,Impl.Text);
  finally
    Intf.Free;
    Impl.Free;
  end;
end;

procedure TCustomTestResolver.StartProgram(NeedSystemUnit: boolean;
  SystemUnitParts: TSystemUnitParts);
begin
  if NeedSystemUnit then
    AddSystemUnit(SystemUnitParts)
  else
    Parser.ImplicitUses.Clear;
  Add('program '+ExtractFileUnitName(MainFilename)+';');
end;

procedure TCustomTestResolver.StartUnit(NeedSystemUnit: boolean);
begin
  if NeedSystemUnit then
    AddSystemUnit
  else
    Parser.ImplicitUses.Clear;
  Add('unit '+ExtractFileUnitName(MainFilename)+';');
end;

function TCustomTestResolver.OnPasResolverFindUnit(SrcResolver: TPasResolver;
  const aUnitName, InFilename: String; NameExpr, InFileExpr: TPasExpr
  ): TPasModule;

  function InitUnit(CurEngine: TTestEnginePasResolver): TPasModule;
  begin
    if CurEngine.Module<>nil then
      Fail('InitUnit '+GetObjName(CurEngine.Module));
    CurEngine.StreamResolver:=Resolver;
    //writeln('TTestResolver.OnPasResolverFindUnit SOURCE=',CurEngine.Source);
    CurEngine.StreamResolver.AddStream(CurEngine.FileName,
                                    TStringStream.Create(CurEngine.Source));
    CurEngine.Scanner:=TPascalScanner.Create(CurEngine.StreamResolver);
    CurEngine.Scanner.CurrentBoolSwitches:=[bsHints,bsNotes,bsWarnings];
    CurEngine.Parser:=TPasParser.Create(CurEngine.Scanner,
                                        CurEngine.StreamResolver,CurEngine);
    CurEngine.Parser.Options:=CurEngine.Parser.Options+[po_StopOnUnitInterface];
    if CompareText(ExtractFileUnitName(CurEngine.Filename),'System')=0 then
      CurEngine.Parser.ImplicitUses.Clear;
    CurEngine.Scanner.OpenFile(CurEngine.Filename);
    try
      CurEngine.Parser.NextToken;
      CurEngine.Parser.ParseUnit(CurEngine.FModule);
    except
      on E: Exception do
        HandleError(CurEngine,E);
    end;
    //writeln('TTestResolver.OnPasResolverFindUnit END ',CurUnitName);
    Result:=CurEngine.Module;
  end;

  function FindUnit(const aUnitName: String): TPasModule;
  var
    i: Integer;
    CurEngine: TTestEnginePasResolver;
    CurUnitName: String;
  begin
    {$IFDEF VerboseUnitSearch}
    writeln('TTestResolver.OnPasResolverFindUnit START Unit="',aUnitName,'"');
    {$ENDIF}
    Result:=nil;
    for i:=0 to ModuleCount-1 do
      begin
      CurEngine:=Modules[i];
      CurUnitName:=ExtractFileUnitName(CurEngine.Filename);
      {$IFDEF VerboseUnitSearch}
      writeln('TTestResolver.OnPasResolverFindUnit Checking ',i,'/',ModuleCount,' ',CurEngine.Filename,' ',CurUnitName);
      {$ENDIF}
      if CompareText(aUnitName,CurUnitName)=0 then
        begin
        Result:=CurEngine.Module;
        {$IFDEF VerboseUnitSearch}
        writeln('TTestResolver.OnPasResolverFindUnit Found unit "',CurEngine.Filename,'" Module=',GetObjName(Result));
        {$ENDIF}
        if Result<>nil then exit;
        {$IFDEF VerboseUnitSearch}
        writeln('TTestResolver.OnPasResolverFindUnit PARSING unit "',CurEngine.Filename,'"');
        {$ENDIF}
        Result:=InitUnit(CurEngine);
        exit;
        end;
      end;
  end;

  function GetResolver(aFilename: string): boolean;
  var
    CurEngine: TTestEnginePasResolver;
    aModule: TPasModule;
  begin
    {$IFDEF VerbosePasResolver}
    writeln('TCustomTestResolver.OnPasResolverFindUnit searching file "',aFilename,'"');
    {$ENDIF}
    CurEngine:=FindModuleWithFilename(aFilename);
    if CurEngine=nil then exit(false);
    aModule:=InitUnit(CurEngine);
    if aModule=nil then exit(false);
    OnPasResolverFindUnit:=aModule;
    Result:=true;
  end;

var
  aFilename: String;
begin
  if SrcResolver=nil then ;
  if NameExpr=nil then ;
  if InFilename<>'' then
    begin
    // uses IN parameter
    {$IFDEF VerbosePasResolver}
    writeln('TCustomTestResolver.OnPasResolverFindUnit searching IN-file "',InFilename,'"');
    {$ENDIF}
    if SrcResolver<>ResolverEngine then
      SrcResolver.RaiseMsg(20180222004753,100000,'in-file only allowed in program',
         [],InFileExpr);

    aFilename:=InFilename;
    DoDirSeparators(aFilename);
    if FilenameIsAbsolute(aFilename) then
      if GetResolver(aFilename) then exit;
    aFilename:=ExtractFilePath(ResolverEngine.Filename)+aFilename;
    if GetResolver(aFilename) then exit;
    SrcResolver.RaiseMsg(20180222004311,100001,'in-file ''%s'' not found',
      [InFilename],InFileExpr);
    end;

  if (Pos('.',aUnitName)<1) and (ResolverEngine.DefaultNameSpace<>'') then
    begin
    // first search in default program namespace
    {$IFDEF VerbosePasResolver}
    writeln('TCustomTestResolver.OnPasResolverFindUnit searching "',aUnitName,'" in default program/library namespace "',ResolverEngine.DefaultNameSpace,'"');
    {$ENDIF}
    Result:=FindUnit(ResolverEngine.DefaultNameSpace+'.'+aUnitName);
    if Result<>nil then exit;
    end;
  Result:=FindUnit(aUnitName);
  if Result<>nil then exit;
  {$IFDEF VerbosePasResolver}
  writeln('TTestResolver.OnPasResolverFindUnit missing unit "',aUnitName,'"');
  {$ENDIF}
end;

procedure TCustomTestResolver.OnFindReference(El: TPasElement; FindData: pointer);
var
  Data: PTestResolverReferenceData absolute FindData;
  Line, Col: integer;
begin
  ResolverEngine.UnmangleSourceLineNumber(El.SourceLinenumber,Line,Col);
  //writeln('TTestResolver.OnFindReference ',El.SourceFilename,' Line=',Line,',Col=',Col,' ',GetObjName(El),' SearchFile=',Data^.Filename,',Line=',Data^.Row,',Col=',Data^.StartCol,'-',Data^.EndCol);
  if (Data^.Filename=El.SourceFilename)
  and (Data^.Row=Line)
  and (Data^.StartCol<=Col)
  and (Data^.EndCol>=Col)
  then
    Data^.Found.Add(El);
end;

procedure TCustomTestResolver.OnCheckElementParent(El: TPasElement; arg: pointer);
var
  SubEl: TPasElement;
  i: Integer;

  procedure E(Msg: string);
  var
    s: String;
  begin
    s:='TTestResolver.OnCheckElementParent El='+GetTreeDbg(El)+' '+
      ResolverEngine.GetElementSourcePosStr(El)+' '+Msg;
    writeln('ERROR: ',s);
    Fail(s);
  end;

begin
  if arg=nil then ;
  //writeln('TTestResolver.OnCheckElementParent ',GetObjName(El));
  if El=nil then exit;
  if El.Parent=El then
    E('El.Parent=El='+GetObjName(El));
  if El is TBinaryExpr then
    begin
    if (TBinaryExpr(El).left<>nil) and (TBinaryExpr(El).left.Parent<>El) then
      E('TBinaryExpr(El).left.Parent='+GetObjName(TBinaryExpr(El).left.Parent)+'<>El');
    if (TBinaryExpr(El).right<>nil) and (TBinaryExpr(El).right.Parent<>El) then
      E('TBinaryExpr(El).right.Parent='+GetObjName(TBinaryExpr(El).right.Parent)+'<>El');
    end
  else if El is TParamsExpr then
    begin
    if (TParamsExpr(El).Value<>nil) and (TParamsExpr(El).Value.Parent<>El) then
      E('TParamsExpr(El).Value.Parent='+GetObjName(TParamsExpr(El).Value.Parent)+'<>El');
    for i:=0 to length(TParamsExpr(El).Params)-1 do
      if TParamsExpr(El).Params[i].Parent<>El then
        E('TParamsExpr(El).Params[i].Parent='+GetObjName(TParamsExpr(El).Params[i].Parent)+'<>El');
    end
  else if El is TProcedureExpr then
    begin
    if (TProcedureExpr(El).Proc<>nil) and (TProcedureExpr(El).Proc.Parent<>El) then
      E('TProcedureExpr(El).Proc.Parent='+GetObjName(TProcedureExpr(El).Proc.Parent)+'<>El');
    end
  else if El is TPasDeclarations then
    begin
    for i:=0 to TPasDeclarations(El).Declarations.Count-1 do
      begin
      SubEl:=TPasElement(TPasDeclarations(El).Declarations[i]);
      if SubEl.Parent<>El then
        E('SubEl=TPasElement(TPasDeclarations(El).Declarations[i])='+GetObjName(SubEl)+' SubEl.Parent='+GetObjName(SubEl.Parent)+'<>El');
      end;
    end
  else if El is TPasImplBlock then
    begin
    for i:=0 to TPasImplBlock(El).Elements.Count-1 do
      begin
      SubEl:=TPasElement(TPasImplBlock(El).Elements[i]);
      if SubEl.Parent<>El then
        E('TPasElement(TPasImplBlock(El).Elements[i]).Parent='+GetObjName(SubEl.Parent)+'<>El');
      end;
    end
  else if El is TPasImplWithDo then
    begin
    for i:=0 to TPasImplWithDo(El).Expressions.Count-1 do
      begin
      SubEl:=TPasExpr(TPasImplWithDo(El).Expressions[i]);
      if SubEl.Parent<>El then
        E('TPasExpr(TPasImplWithDo(El).Expressions[i]).Parent='+GetObjName(SubEl.Parent)+'<>El');
      end;
    end
  else if El is TPasProcedure then
    begin
    if TPasProcedure(El).ProcType.Parent<>El then
      E('TPasProcedure(El).ProcType.Parent='+GetObjName(TPasProcedure(El).ProcType.Parent)+'<>El');
    end
  else if El is TPasProcedureType then
    begin
    for i:=0 to TPasProcedureType(El).Args.Count-1 do
      if TPasArgument(TPasProcedureType(El).Args[i]).Parent<>El then
        E('TPasArgument(TPasProcedureType(El).Args[i]).Parent='+GetObjName(TPasArgument(TPasProcedureType(El).Args[i]).Parent)+'<>El');
    end;
end;

procedure TCustomTestResolver.FreeSrcMarkers;
var
  aMarker, Last: PSrcMarker;
begin
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    Last:=aMarker;
    aMarker:=aMarker^.Next;
    Dispose(Last);
    end;
  FirstSrcMarker:=nil;
  LastSrcMarker:=nil;
end;

procedure TCustomTestResolver.OnPasResolverLog(Sender: TObject;
  const Msg: String);
var
  aResolver: TTestEnginePasResolver;
  Item: TTestResolverMessage;
begin
  aResolver:=Sender as TTestEnginePasResolver;
  Item:=TTestResolverMessage.Create;
  Item.Id:=aResolver.LastMsgId;
  Item.MsgType:=aResolver.LastMsgType;
  Item.MsgNumber:=aResolver.LastMsgNumber;
  Item.Msg:=Msg;
  Item.SourcePos:=aResolver.LastSourcePos;
  {$IFDEF VerbosePasResolver}
  writeln('TCustomTestResolver.OnPasResolverLog ',GetObjName(Sender),' ',Item.MsgType,' (',Item.MsgNumber,') {',Msg,'}');
  {$ENDIF}
  FResolverMsgs.Add(Item);
end;

procedure TCustomTestResolver.OnScannerDirective(Sender: TObject; Directive,
  Param: String; var Handled: boolean);
var
  aScanner: TPascalScanner;
begin
  if Handled then exit;
  aScanner:=Sender as TPascalScanner;
  aScanner.LastMsgType:=mtError;
  aScanner.LastMsg:='unknown directive "'+Directive+'"';
  aScanner.LastMsgPattern:=aScanner.LastMsg;
  aScanner.LastMsgArgs:=nil;
  raise EScannerError.Create(aScanner.LastMsg);
  if Param='' then ;
end;

procedure TCustomTestResolver.OnScannerLog(Sender: TObject; const Msg: String);
var
  aScanner: TPascalScanner;
begin
  aScanner:=TPascalScanner(Sender);
  if aScanner=nil then exit;
  {$IFDEF VerbosePasResolver}
  writeln('TCustomTestResolver.OnScannerLog ',GetObjName(Sender),' ',aScanner.LastMsgType,' ',aScanner.LastMsgNumber,' Msg="', Msg,'"');
  {$ENDIF}
end;

function TCustomTestResolver.GetModules(Index: integer): TTestEnginePasResolver;
begin
  Result:=TTestEnginePasResolver(FModules[Index]);
end;

function TCustomTestResolver.GetMsgCount: integer;
begin
  Result:=FResolverMsgs.Count;
end;

function TCustomTestResolver.GetMsgs(Index: integer): TTestResolverMessage;
begin
  Result:=TTestResolverMessage(FResolverMsgs[Index]);
end;

procedure TCustomTestResolver.OnPasResolverContinueParsing(Sender: TPasResolver
  );
var
  CurEngine: TTestEnginePasResolver;
begin
  CurEngine:=Sender as TTestEnginePasResolver;
  {$IFDEF VerbosePasResolver}
  writeln('TCustomTestResolver.OnPasResolverContinueParsing "',CurEngine.Module.Name,'"...');
  {$ENDIF}
  try
    CurEngine.Parser.ParseContinue;
  except
    on E: Exception do
      HandleError(CurEngine,E);
  end;
end;

function TCustomTestResolver.GetModuleCount: integer;
begin
  Result:=FModules.Count;
end;

{ TTestResolver }

procedure TTestResolver.TestEmpty;
begin
  StartProgram(false);
  Add('begin');
  ParseProgram;
  AssertEquals('No statements',0,PasProgram.InitializationSection.Elements.Count);
end;

procedure TTestResolver.TestAliasType;
var
  El: TPasElement;
  T: TPasAliasType;
begin
  StartProgram(false);
  Add('type');
  Add('  tint=longint;');
  Add('begin');
  ParseProgram;
  AssertEquals('1 declaration',1,PasProgram.ProgramSection.Declarations.Count);
  El:=TPasElement(PasProgram.ProgramSection.Declarations[0]);
  AssertEquals('Type',TPasAliasType,El.ClassType);
  T:=TPasAliasType(El);
  AssertEquals('Type tint','tint',T.Name);
  AssertEquals('Type built-in',TPasUnresolvedSymbolRef,T.DestType.ClassType);
  AssertEquals('longint type','longint',lowercase(T.DestType.Name));
end;

procedure TTestResolver.TestAlias2Type;
var
  El: TPasElement;
  T1, T2: TPasAliasType;
  DestT1, DestT2: TPasType;
begin
  StartProgram(false);
  Add('type');
  Add('  tint1=longint;');
  Add('  tint2=tint1;');
  Add('begin');
  ParseProgram;
  AssertEquals('2 declaration',2,PasProgram.ProgramSection.Declarations.Count);

  El:=TPasElement(PasProgram.ProgramSection.Declarations[0]);
  AssertEquals('Type',TPasAliasType,El.ClassType);
  T1:=TPasAliasType(El);
  AssertEquals('Type tint1','tint1',T1.Name);
  DestT1:=T1.DestType;
  AssertEquals('built-in',TPasUnresolvedSymbolRef,DestT1.ClassType);
  AssertEquals('built-in longint','longint',lowercase(DestT1.Name));

  El:=TPasElement(PasProgram.ProgramSection.Declarations[1]);
  AssertEquals('Type',TPasAliasType,El.ClassType);
  T2:=TPasAliasType(El);
  AssertEquals('Type tint2','tint2',T2.Name);
  DestT2:=T2.DestType;
  AssertEquals('points to alias type',TPasAliasType,DestT2.ClassType);
  AssertEquals('points to tint1','tint1',DestT2.Name);
end;

procedure TTestResolver.TestAliasTypeRefs;
begin
  StartProgram(false);
  Add('type');
  Add('  {#a}a=longint;');
  Add('  {#b}{=a}b=a;');
  Add('var');
  Add('  {=a}c: a;');
  Add('  {=b}d: b;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestAliasOfVarFail;
begin
  StartProgram(false);
  Add('var');
  Add('  a: char;');
  Add('type');
  Add('  t=a;');
  Add('begin');
  CheckParserException('Expected type, but got variable',PParser.nParserExpectedTypeButGot);
end;

procedure TTestResolver.TestAliasType_UnitPrefix;
begin
  StartUnit(false);
  Add('interface');
  Add('type');
  Add('  {#a}a=longint;');
  Add('  {#b}{=a}b=afile.a;');
  Add('var');
  Add('  {=a}c: a;');
  Add('  {=b}d: b;');
  Add('implementation');
  ParseUnit;
end;

procedure TTestResolver.TestAliasType_UnitPrefix_CycleFail;
begin
  StartUnit(false);
  Add('interface');
  Add('type');
  Add('  {#a}a=afile.a;');
  Add('implementation');
  CheckResolverException('identifier not found "a"',nIdentifierNotFound);
end;

procedure TTestResolver.TestAliasTypeNotFoundPosition;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TColor = NotThere;');
  CheckResolverException('identifier not found "NotThere"',nIdentifierNotFound);
  // TColor element was not created yet, so LastElement must be nil
  AssertNull('ResolverEngine.LastElement',ResolverEngine.LastElement);
  with ResolverEngine.LastSourcePos do
    begin
    //writeln('TTestResolver.TestAliasTypeNotFoundPosition ',FileName,' ',Row,' ',Col);
    //WriteSources(FileName,Row,Column);
    AssertEquals('ResolverEngine.LastSourcePos.Filename','afile.pp',FileName);
    AssertEquals('ResolverEngine.LastSourcePos.Row',4,Row);
    AssertEquals('ResolverEngine.LastSourcePos.Column',20,Column);
    end;
end;

procedure TTestResolver.TestTypeAliasType;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#integer}integer = longint;',
  '  {#tcolor}TColor = type integer;',
  'var',
  '  {=integer}i: integer;',
  '  {=tcolor}c: TColor;',
  'begin',
  '  c:=i;',
  '  i:=c;',
  '  i:=integer(c);',
  '  c:=TColor(i);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestVarLongint;
var
  El: TPasElement;
  V1: TPasVariable;
  DestT1: TPasType;
begin
  StartProgram(false);
  Add('var');
  Add('  v1:longint;');
  Add('begin');
  ParseProgram;
  AssertEquals('1 declaration',1,PasProgram.ProgramSection.Declarations.Count);

  El:=TPasElement(PasProgram.ProgramSection.Declarations[0]);
  AssertEquals('var',TPasVariable,El.ClassType);
  V1:=TPasVariable(El);
  AssertEquals('var v1','v1',V1.Name);
  DestT1:=V1.VarType;
  AssertEquals('built-in',TPasUnresolvedSymbolRef,DestT1.ClassType);
  AssertEquals('built-in longint','longint',lowercase(DestT1.Name));
end;

procedure TTestResolver.TestVarInteger;
var
  El: TPasElement;
  V1: TPasVariable;
  DestT1: TPasType;
begin
  StartProgram(true);
  Add('var');
  Add('  v1:integer;'); // defined in system.pp
  Add('begin');
  ParseProgram;
  AssertEquals('1 declaration',1,PasProgram.ProgramSection.Declarations.Count);

  El:=TPasElement(PasProgram.ProgramSection.Declarations[0]);
  AssertEquals('var',TPasVariable,El.ClassType);
  V1:=TPasVariable(El);
  AssertEquals('var v1','v1',V1.Name);
  DestT1:=V1.VarType;
  AssertNotNull('v1 type',DestT1);
  AssertEquals('built-in',TPasAliasType,DestT1.ClassType);
  AssertEquals('built-in integer','integer',DestT1.Name);
  AssertNull('v1 no expr',V1.Expr);
end;

procedure TTestResolver.TestConstInteger;
var
  El: TPasElement;
  C1: TPasConst;
  DestT1: TPasType;
  ExprC1: TPrimitiveExpr;
begin
  StartProgram(true);
  Add('const');
  Add('  c1: integer=3;'); // defined in system.pp
  Add('begin');
  ParseProgram;
  AssertEquals('1 declaration',1,PasProgram.ProgramSection.Declarations.Count);

  El:=TPasElement(PasProgram.ProgramSection.Declarations[0]);
  AssertEquals('const',TPasConst,El.ClassType);
  C1:=TPasConst(El);
  AssertEquals('const c1','c1',C1.Name);
  DestT1:=C1.VarType;
  AssertNotNull('c1 type',DestT1);
  AssertEquals('built-in',TPasAliasType,DestT1.ClassType);
  AssertEquals('built-in integer','integer',DestT1.Name);
  ExprC1:=TPrimitiveExpr(C1.Expr);
  AssertNotNull('c1 expr',ExprC1);
  AssertEquals('c1 expr primitive',TPrimitiveExpr,ExprC1.ClassType);
  AssertEquals('c1 expr value','3',ExprC1.Value);
end;

procedure TTestResolver.TestConstInteger2;
begin
  StartProgram(false);
  Add('const');
  Add('  c1 = 3');
  Add('  c2: longint=c1;');
  Add('begin');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestDuplicateVar;
begin
  StartProgram(false);
  Add('var a: longint;');
  Add('var a: string;');
  Add('begin');
  CheckResolverException(sDuplicateIdentifier,nDuplicateIdentifier);
end;

procedure TTestResolver.TestVarInitConst;
begin
  StartProgram(false);
  Add('const {#c}c=1;');
  Add('var a: longint = {@c}c;');
  Add('begin');
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestVarOfVarFail;
begin
  StartProgram(false);
  Add('var');
  Add('  a: char;');
  Add('  b: a;');
  Add('begin');
  CheckParserException('Expected type, but got variable',PParser.nParserExpectedTypeButGot);
end;

procedure TTestResolver.TestConstOfVarFail;
begin
  StartProgram(false);
  Add('var');
  Add('  a: longint;');
  Add('const');
  Add('  b: a = 1;');
  Add('begin');
  CheckParserException('Expected type, but got variable',PParser.nParserExpectedTypeButGot);
end;

procedure TTestResolver.TestTypedConstWrongExprFail;
begin
  StartProgram(false);
  Add('const');
  Add('  a: string = 1;');
  Add('begin');
  CheckResolverException('Incompatible types: got "Longint" expected "String"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestVarWrongExprFail;
begin
  StartProgram(false);
  Add('var');
  Add('  a: string = 1;');
  Add('begin');
  CheckResolverException('Incompatible types: got "Longint" expected "String"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestArgWrongExprFail;
begin
  StartProgram(false);
  Add('procedure ProcA(a: string = 1);');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckResolverException('Incompatible types: got "Longint" expected "String"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestTypedConstInConstExprFail;
begin
  StartProgram(false);
  Add('const');
  Add('  a: longint = 3;');
  Add('  b: longint = a;');
  Add('begin');
  CheckResolverException('Constant expression expected',nConstantExpressionExpected);
end;

procedure TTestResolver.TestVarExternal;
begin
  StartProgram(false);
  Add('var');
  Add('  NaN: double; external name ''Global.Nan'';');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestVarNoSemicolonBeginFail;
begin
  StartProgram(false);
  Add('procedure DoIt; begin end;');
  Add('var');
  Add('  i: longint');
  Add('begin');
  Add('  doit;');
  CheckParserException('Expected ";"',
    nParserExpectTokenError);
end;

procedure TTestResolver.TestConstIntOperators;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  'const',
  '  a:byte=1+2;',
  '  b:shortint=1-2;',
  '  c:word=2*3;',
  '  d:smallint=5 div 2;',
  '  e:longword=5 mod 2;',
  '  f:longint=5 shl 2;',
  '  g:qword=5 shr 2;',
  '  h:boolean=5=2;',
  '  i:boolean=5<>2;',
  '  j:boolean=5<2;',
  '  k:boolean=5>2;',
  '  l:boolean=5<=2;',
  '  m:boolean=5>=2;',
  '  n:longword=5 and 2;',
  '  o:longword=5 or 2;',
  '  p:longword=5 xor 2;',
  '  q:longword=not (5 or not 2);',
  '  r=low(word)+high(int64);',
  '  s=low(longint)+high(integer);',
  '  t=succ(2)+pred(2);',
  '  lo1:byte=lo(word($1234));',
  '  hi1:byte=hi(word($1234));',
  '  lo2:word=lo(longword($1234CDEF));',
  '  hi2:word=hi(longword($1234CDEF));',
  '  lo3:word=lo(LongInt(-$1234CDEF));',
  '  hi3:word=hi(LongInt(-$1234CDEF));',
  '  lo4:byte=lo(byte($34));',
  '  hi4:byte=hi(byte($34));',
  '  lo5:byte=lo(shortint(-$34));',
  '  hi5:byte=hi(shortint(-$34));',
  '  lo6:longword=lo($123456789ABCDEF0);',
  '  hi6:longword=hi($123456789ABCDEF0);',
  '  lo7:longword=lo(-$123456789ABCDEF0);',
  '  hi7:longword=hi(-$123456789ABCDEF0);',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestConstBitwiseOps;
begin
  StartProgram(false);
  Add([
  'const',
  '  a=3;',
  '  b=not a;',
  '  c=not word(a);',
  '  d=1 shl 2;',
  '  e=13 shr 1;',
  '  f=13 and 5;',
  '  g=10 or 5;',
  '  h=5 xor 7;',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestConstExternal;
begin
  Parser.Options:=Parser.Options+[po_ExtConstWithoutExpr];
  StartProgram(false);
  Add([
  'const',
  '  PI: double; external name ''Global.PI'';',
  '  Tau = 2*PI;',
  '  TauD: double = 2*PI;',
  'var',
  '  d: double = PI;',
  '  e: double = PI+Tau;',
  'begin',
  '  d:=pi+tau;']);
  ParseProgram;
  // ToDo: fail on const Tau = 2*Var
end;

procedure TTestResolver.TestIntegerTypeCast;
begin
  StartProgram(false);
  Add([
  'const',
  '  a=longint(-11);',
  '  b=not shortint(-12);',
  '  c=word(-2);',
  '  d=word(longword(-3));',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestConstFloatOperators;
begin
  StartProgram(false);
  Add([
  'const',
  '  a=4/2 + 6.1/3 + 8.1/4.1 + 10/5.1;',
  '  b=(1.1+1) + (2.1+3.1) + (4+5.1);',
  '  c=(1.1-1) + (2.1-3.1) + (4-5.1);',
  '  d=4*2 + 6.1*3 + 8.1*4.1 + 10*5.1;',
  '  e=a=b;',
  '  f=a<>b;',
  '  g=a>b;',
  '  h=a>=b;',
  '  i=a<b;',
  '  j=a<=b;',
  '  k=(1.1<1) or (2.1<3.1) or (4<5.1);',
  '  l=(1.1=1) or (2.1=3.1) or (4=5.1);',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestFloatTypeCast;
begin
  StartProgram(false);
  Add([
  'const',
  '  a=-123456890123456789012345;',
  '  b: double=-123456890123456789012345;',
  '  c=single(double(-123456890123456789012345));',
  '  d=single(-1);',
  '  e=single(word(-1));',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestCurrency;
begin
  StartProgram(false);
  Add([
  'const',
  '  a: currency = -922337203685477.5808;',
  '  b: currency = 922337203685477.5807;',
  '  c=double(currency(-123456890123456));',
  '  d=currency(-1);',
  '  e=currency(word(-1));',
  'var',
  '  i: longint = 1;',
  '  i64: int64;',
  '  f: double;',
  'begin',
  '  a:=i;',
  '  a:=i+a;',
  '  a:=a+i;',
  '  a:=-a+b;',
  '  a:=a*b;',
  '  a:=a/b;',
  '  a:=a/1.23;',
  '  a:=1.2345;',
  '  a:=a-i;',
  '  a:=i-a;',
  '  a:=a*i;',
  '  a:=i*a;',
  '  a:=a/i;',
  '  a:=i/a;',
  '  a:=i64;',
  '  a:=currency(i64);',
  //'  i64:=a;', not allowed
  '  i64:=int64(a);', // truncates a
  '  a:=f;',
  '  a:=currency(f);',
  '  f:=a;',
  '  f:=double(a);',
  '']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestWritableConst;
begin
  StartProgram(false);
  Add([
  '{$writeableconst off}',
  'const i: longint = 3;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestWritableConst_AssignFail;
begin
  StartProgram(false);
  Add([
  '{$writeableconst off}',
  'const i: longint = 3;',
  'begin',
  '  i:=4;',
  '']);
  CheckResolverException(sCantAssignValuesToConstVariable,nCantAssignValuesToConstVariable);
end;

procedure TTestResolver.TestWritableConst_PassVarFail;
begin
  StartProgram(false);
  Add([
  '{$writeableconst off}',
  'const i: longint = 3;',
  'procedure DoIt(var j: longint); external;',
  'begin',
  '  DoIt(i);',
  '']);
  CheckResolverException(sCantAssignValuesToConstVariable,nCantAssignValuesToConstVariable);
end;

procedure TTestResolver.TestBoolTypeCast;
begin
  StartProgram(false);
  Add('var');
  Add('  a: boolean = boolean(0);');
  Add('  b: boolean = boolean(1);');
  Add('begin');
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestConstBoolOperators;
begin
  StartProgram(false);
  Add([
  'const',
  '  a=true and false;',
  '  b=true or false;',
  '  c=true xor false;',
  '  d=not b;',
  '  e=a=b;',
  '  f=a<>b;',
  '  g=low(boolean) or high(boolean);',
  '  h=succ(false) or pred(true);',
  '  i=ord(false)+ord(true);',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestBoolSet_Const;
begin
  StartProgram(false);
  Add([
  'const',
  '  s1 = [true];',
  '  s2 = [false,true];',
  '  s3 = [false..true];',
  '  s7 = [true]*s2;',
  '  s8 = s2-s1;',
  '  s9 = s1+s2;',
  '  s10 = s1><s2;',
  '  s11 = s2=s3;',
  '  s12 = s2<>s3;',
  '  s13 = s2<=s3;',
  '  s14 = s2>=s3;',
  '  s15 = true in s2;',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestBool_ForIn;
begin
  StartProgram(false);
  Add([
  'type',
  //'  TBoolRg = false..true;',
  '  TSetOfBool = set of boolean;',
  //'  TSetOfBoolRg = set of TBoolRg;',
  'var',
  '  b: boolean;',
  //'  br: TBoolRg;',
  'begin',
  '  for b in boolean do;',
  //'  for b in TBoolRg do;',
  '  for b in TSetOfBool do;',
  //'  for b in TSetOfBoolRg do;',
  //'  for br in TBoolRg do;',
  //'  for br in TSetOfBoolRg do;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestBool_Assert;
begin
  StartProgram(false);
  Add([
  'var',
  '  b : boolean;',
  '  s: string;',
  'begin',
  '  Assert(true);',
  '  Assert(b);',
  '  Assert(b,''error'');',
  '  Assert(false,''error''+s);',
  '  Assert(not b);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestBool_AssertSysutils;
begin
  AddModuleWithIntfImplSrc('SysUtils.pas',
    LinesToStr([
    'type',
    '  TObject = class',
    '    constructor Create;',
    '  end;',
    '  EAssertionFailed = class',
    '    constructor Create(s: string);',
    '  end;',
    '']),
    LinesToStr([
    'constructor TObject.Create;',
    'begin end;',
    'constructor EAssertionFailed.Create(s: string);',
    'begin end;',
    '']) );

  StartProgram(true);
  Add([
  'uses sysutils;',
  'procedure DoIt;',
  'var',
  '  b: boolean;',
  '  s: string;',
  'begin',
  '  {$Assertions on}',
  '  Assert(b);',
  '  Assert(b,s);',
  'end;',
  'begin',
  '  DoIt;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestIntegerRange;
begin
  StartProgram(false);
  Add([
  'const',
  '  MinInt = -1;',
  '  MaxInt = +1;',
  'type',
  '  {#TMyInt}TMyInt = MinInt..MaxInt;',
  '  TInt2 = 1..3;',
  'var',
  '  i: TMyInt;',
  '  i2: TInt2;',
  'begin',
  '  i:=i2;',
  '  if i=i2 then ;']);
  ParseProgram;
end;

procedure TTestResolver.TestIntegerRangeHighLowerLowFail;
begin
  StartProgram(false);
  Add('const');
  Add('  MinInt = -1;');
  Add('  MaxInt = +1;');
  Add('type');
  Add('  {#TMyInt}TMyInt = MaxInt..MinInt;');
  Add('begin');
  CheckResolverException(sHighRangeLimitLTLowRangeLimit,
    nHighRangeLimitLTLowRangeLimit);
end;

procedure TTestResolver.TestIntegerRangeLowHigh;
begin
  StartProgram(false);
  Add([
  'const',
  '  MinInt = -1;',
  '  MaxInt = +10;',
  'type',
  '  {#TMyInt}TMyInt = MinInt..MaxInt;',
  'const',
  '  a = low(TMyInt)+High(TMyInt);',
  'var',
  '  i: TMyInt;',
  'begin',
  '  i:=low(i)+high(i);']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestAssignIntRangeWarning;
begin
  StartProgram(false);
  Add([
  'type TMyInt = 1..2;',
  'var i: TMyInt;',
  'begin',
  '  i:=3;']);
  ParseProgram;
  CheckResolverHint(mtWarning,nRangeCheckEvaluatingConstantsVMinMax,
    'range check error while evaluating constants (3 is not between 1 and 2)');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestByteRangeWarning;
begin
  StartProgram(false);
  Add([
  'var b:byte=300;',
  'begin']);
  ParseProgram;
  CheckResolverHint(mtWarning,nRangeCheckEvaluatingConstantsVMinMax,
    'range check error while evaluating constants (300 is not between 0 and 255)');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestByteRangeWarningOff;
begin
  StartProgram(false);
  Add([
  '{$warnings off}',
  'var b:byte=300;',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestCustomIntRangeWarning;
begin
  StartProgram(false);
  Add([
  'const i:1..2 = 3;',
  'begin']);
  ParseProgram;
  CheckResolverHint(mtWarning,nRangeCheckEvaluatingConstantsVMinMax,
    'range check error while evaluating constants (3 is not between 1 and 2)');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestIntSet_Const;
begin
  StartProgram(false);
  Add([
  'const',
  '  s1 = [1];',
  '  s2 = [1,2];',
  '  s3 = [1..3];',
  '  s4 = [1..2,4..5,6];',
  '  s5 = [low(shortint)..high(shortint)];',
  '  s6 = [succ(low(shortint))..pred(high(shortint))];',
  '  s7 = [1..3]*[2..4];',
  '  s8 = [1..5]-[2,5];',
  '  s9 = [1,3..4]+[2,5];',
  '  s10 = [1..3]><[2..5];',
  '  s11 = s2=s3;',
  '  s12 = s2<>s3;',
  '  s13 = s2<=s3;',
  '  s14 = s2>=s3;',
  '  s15 = 1 in s2;',
  'var',
  '  w: word;',
  'begin',
  '  if w in [1..12] then ;',
  '']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestIntSet_ConstDuplicateElement;
begin
  StartProgram(false);
  Add([
  'const',
  '  s1 = [1,1..2];',
  'begin']);
  CheckResolverException(sRangeCheckInSetConstructor,nRangeCheckInSetConstructor);
end;

procedure TTestResolver.TestInt_ForIn;
begin
  StartProgram(false);
  Add([
  'type',
  '  TIntRg = 2..4;',
  '  TSetOfInt = set of byte;',
  '  TSetOfIntRg = set of TIntRg;',
  'var',
  '  i: longint;',
  '  ir: TIntRg;',
  'begin',
  '  for i in longint do;',
  '  for i in TIntRg do;',
  '  for i in TSetOfInt do;',
  '  for i in TSetOfIntRg do;',
  '  for ir in TIntRg do;',
  '  for ir in TSetOfIntRg do;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestChar_BuiltInProcs;
begin
  StartProgram(false);
  Add([
  'var',
  '  c: char;',
  '  i: longint;',
  'begin',
  '  i:=ord(c);',
  '  c:=chr(i);',
  '  c:=pred(c);',
  '  c:=succ(c);',
  '  c:=low(c);',
  '  c:=high(c);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestString_BuiltInProcs;
begin
  StartProgram(false);
  Add([
  'var',
  '  s: string;',
  'begin',
  '  SetLength({#a_var}s,3);',
  '  SetLength({#b_var}s,length({#c_read}s));',
  '  s:=concat(''a'',s);',
  '']);
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestString_Element;
begin
  StartProgram(false);
  Add([
  'var',
  '  s: string;',
  '  c: char;',
  'begin',
  '  if s[1]=s then ;',
  '  if s=s[2] then ;',
  '  if s[3+4]=c then ;',
  '  if c=s[5] then ;',
  '  c:=s[6];',
  '  s[7]:=c;',
  '  s[8]:=''a'';',
  '  s[9+1]:=''b'';',
  '  s[10]:='''''''';',
  '  s[11]:=^g;',
  '  s[12]:=^H;']);
  ParseProgram;
end;

procedure TTestResolver.TestStringElement_MissingArgFail;
begin
  StartProgram(false);
  Add('var s: string;');
  Add('begin');
  Add('  if s[]=s then ;');
  CheckResolverException('Missing parameter character index',nMissingParameterX);
end;

procedure TTestResolver.TestStringElement_IndexNonIntFail;
begin
  StartProgram(false);
  Add('var s: string;');
  Add('begin');
  Add('  if s[true]=s then ;');
  CheckResolverException('Incompatible types: got "Boolean" expected "integer"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestStringElement_AsVarArgFail;
begin
  StartProgram(false);
  Add('procedure DoIt(var c: char);');
  Add('begin');
  Add('end;');
  Add('var s: string;');
  Add('begin');
  Add('  DoIt(s[1]);');
  CheckResolverException('Variable identifier expected',
    nVariableIdentifierExpected);
end;

procedure TTestResolver.TestString_DoubleQuotesFail;
begin
  StartProgram(false);
  Add('var s: string;');
  Add('begin');
  Add('  s:="abc" + "def";');
  CheckParserException('Invalid character ''"''',PScanner.nErrInvalidCharacter);
end;

procedure TTestResolver.TestString_ShortstringType;
begin
  StartProgram(false);
  Add([
  'type t = string[12];',
  'var',
  '  s: t;',
  'begin',
  '  s:=''abc'';',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestConstStringOperators;
begin
  StartProgram(false);
  Add([
  'const',
  '  a=''o''+''x''+''''+''ab'';',
  '  b=#65#66;',
  '  c=a=b;',
  '  d=a<>b;',
  '  e=a<b;',
  '  f=a<=b;',
  '  g=a>b;',
  '  h=a>=b;',
  '  i=a[1];',
  '  j=length(a);',
  '  k=chr(97);',
  '  l=ord(a[1]);',
  '  m=low(char)+high(char);',
  '  n = string(''A'');',
  '  o = UnicodeString(''A'');',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestConstUnicodeStringOperators;
begin
  ResolverEngine.ExprEvaluator.DefaultStringCodePage:=CP_UTF8;
  StartProgram(false);
  Add([
  'const',
  '  a=''大''+''学'';',
  '  b=#22823+#23398;',
  '  c=a=b;',
  '  d=a<>b;',
  '  e=a<b;',
  '  f=a<=b;',
  '  g=a>b;',
  '  h=a>=b;',
  '  i=b[1];',
  '  j=length(b);',
  '  k=chr(22823);',
  '  l=ord(b[1]);',
  '  m=low(widechar)+high(widechar);',
  '  n=#65#22823;',
  '  ascii=#65;',
  '  o=ascii+b;',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestCharSet_Const;
begin
  StartProgram(false);
  Add([
  'const',
  '  s1 = [''a''];',
  '  s2 = [''a'',''b''];',
  '  s3 = [''a''..''c''];',
  '  s4 = [''a''..''b'',''d''..''e'',''f''];',
  '  s5 = [low(Char)..high(Char)];',
  '  s6 = [succ(low(Char))..pred(high(Char))];',
  '  s7 = [''a''..''c'']*[''b''..''d''];',
  '  s8 = [''a''..''e'']-[''b'',''e''];',
  '  s9 = [''a'',''c''..''d'']+[''b'',''e''];',
  '  s10 = [''a''..''c'']><[''b''..''e''];',
  '  s11 = [''a'',''b'']=[''a''..''b''];',
  '  s12 = [''a'',''b'']<>[''a''..''b''];',
  '  s13 = [''a'',''b'']<=[''a''..''b''];',
  '  s14 = [''a'',''b'']>=[''a''..''b''];',
  '  s15 = ''a'' in [''a'',''b''];',
  '  s16 = [#0..#127,#22823..#23398];',
  '  s17 = #22823 in s16;',
  'var c: char;',
  'begin',
  '  if c in s3 then ;']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestCharSet_Custom;
begin
  StartProgram(false);
  Add([
  'type',
  '  TCharRg = ''a''..''z'';',
  '  TSetOfCharRg = set of TCharRg;',
  '  TCharRg2 = ''m''..''p'';',
  'const',
  '  crg: TCharRg = ''b'';',
  'var',
  '  c: char;',
  '  crg2: TCharRg2;',
  '  s: TSetOfCharRg;',
  'begin',
  '  c:=crg;',
  '  crg:=c;',
  '  crg2:=crg;',
  '  if c=crg then ;',
  '  if crg=c then ;',
  '  if crg=crg2 then ;',
  '  if c in s then ;',
  '  if crg2 in s then ;',
  '']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestCharAssignStringFail;
begin
  StartProgram(false);
  Add([
  'var',
  '  c: char;',
  '  s: string;',
  'begin',
  '  c:=s;']);
  CheckResolverException('Incompatible types: got "String" expected "Char"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestChar_ForIn;
begin
  StartProgram(false);
  Add([
  'type',
  '  TCharRg = ''a''..''z'';',
  '  TSetOfChar = set of char;',
  '  TSetOfCharRg = set of TCharRg;',
  'const Foo = ''foo'';',
  'var',
  '  c: char;',
  '  cr: TCharRg;',
  '  s: string;',
  '  a: array of char;',
  '  b: array[1..3] of char;',
  '  soc: TSetOfChar;',
  '  socr: TSetOfCharRg;',
  'begin',
  '  for c in foo do;',
  '  for c in s do;',
  '  for c in a do;',
  '  for c in b do;',
  '  for c in char do;',
  '  for c in TCharRg do;',
  '  for c in TSetOfChar do;',
  '  for c in TSetOfCharRg do;',
  '  for c in soc do;',
  '  for c in socr do;',
  '  for c in [''A''..''C''] do ;',
  '  for cr in TCharRg do;',
  '  for cr in TSetOfCharRg do;',
  '  for cr in socr do;',
  //'  for cr in [''b''..''d''] do ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestEnums;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TFlag}TFlag = ({#Red}Red, {#Green}Green, {#Blue}Blue);',
  '  {#TAlias}TAlias = TFlag;',
  'var',
  '  {#f}{=TFlag}f: TFlag;',
  '  {#v}{=TFlag}v: TFlag = Green;',
  '  {#i}i: longint;',
  'begin',
  '  {@f}f:={@Red}Red;',
  '  {@f}f:={@v}v;',
  '  if {@f}f={@Red}Red then ;',
  '  if {@f}f={@v}v then ;',
  '  if {@f}f>{@v}v then ;',
  '  if {@f}f<{@v}v then ;',
  '  if {@f}f>={@v}v then ;',
  '  if {@f}f<={@v}v then ;',
  '  if {@f}f<>{@v}v then ;',
  '  if ord({@f}f)<>ord({@Red}Red) then ;',
  '  {@f}f:={@TFlag}TFlag.{@Red}Red;',
  '  {@f}f:={@TFlag}TFlag({@i}i);',
  '  {@i}i:=longint({@f}f);',
  '  {@f}f:={@TAlias}TAlias.{@Green}Green;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestEnumRangeFail;
begin
  StartProgram(false);
  Add([
  'type TFlag = (a,b,c);',
  'const all = a..c;',
  'begin']);
  CheckParserException('Const ranges are not allowed',nParserNoConstRangeAllowed);
end;

procedure TTestResolver.TestEnumDotValueFail;
begin
  StartProgram(false);
  Add([
  'type TFlag = (a,b,c);',
  'var f: TFlag;',
  'begin',
  '  f:=f.a;']);
  CheckResolverException('illegal qualifier "." after "f:TFlag"',nIllegalQualifierAfter);
end;

procedure TTestResolver.TestSets;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TFlag}TFlag = ({#Red}Red, {#Green}Green, {#Blue}Blue, {#Gray}Gray, {#Black}Black, {#White}White);');
  Add('  {#TFlags}TFlags = set of TFlag;');
  Add('  {#TChars}TChars = set of Char;');
  Add('  {#TMyInt}TMyInt = 0..17;');
  Add('  {#TMyInts}TMyInts = set of TMyInt;');
  Add('  {#TMyBools}TMyBools = set of boolean;');
  Add('const');
  Add('  {#Colors}Colors = [{@Red}Red..{@Blue}Blue];');
  Add('  {#ExtColors}ExtColors = {@Colors}Colors+[{@White}White,{@Black}Black];');
  Add('var');
  Add('  {#f}{=TFlag}f: TFlag;');
  Add('  {#s}{=TFlags}s: TFlags;');
  Add('  {#t}{=TFlags}t: TFlags = [Green,Gray];');
  Add('  {#Chars}{=TChars}Chars: TChars;');
  Add('  {#MyInts}{=TMyInts}MyInts: TMyInts;');
  Add('  {#MyBools}{=TMyBools}MyBools: TMyBools;');
  Add('begin');
  Add('  {@s}s:=[];');
  Add('  {@s}s:={@t}t;');
  Add('  {@s}s:=[{@Red}Red];');
  Add('  {@s}s:=[{@Red}Red,{@Blue}Blue];');
  Add('  {@s}s:=[{@Gray}Gray..{@White}White];');
  Add('  {@MyInts}MyInts:=[1];');
  Add('  {@MyInts}MyInts:=[1,2];');
  Add('  {@MyInts}MyInts:=[1..2];');
  Add('  {@MyInts}MyInts:=[1..2,3];');
  Add('  {@MyInts}MyInts:=[1..2,3..4];');
  Add('  {@MyInts}MyInts:=[1,2..3];');
  Add('  {@MyBools}MyBools:=[false];');
  Add('  {@MyBools}MyBools:=[false,true];');
  Add('  {@MyBools}MyBools:=[false..true];');
  ParseProgram;
end;

procedure TTestResolver.TestSetOperators;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TFlag}TFlag = ({#Red}Red, {#Green}Green, {#Blue}Blue, {#Gray}Gray, {#Black}Black, {#White}White);');
  Add('  {#TFlags}TFlags = set of TFlag;');
  Add('  {#TChars}TChars = set of Char;');
  Add('  {#TMyInt}TMyInt = 0..17;');
  Add('  {#TMyInts}TMyInts = set of TMyInt;');
  Add('  {#TMyBools}TMyBools = set of boolean;');
  Add('const');
  Add('  {#Colors}Colors = [{@Red}Red..{@Blue}Blue];');
  Add('  {#ExtColors}ExtColors = {@Colors}Colors+[{@White}White,{@Black}Black];');
  Add('var');
  Add('  {#f}{=TFlag}f: TFlag;');
  Add('  {#s}{=TFlags}s: TFlags;');
  Add('  {#t}{=TFlags}t: TFlags = [Green,Gray];');
  Add('  {#Chars}{=TChars}Chars: TChars;');
  Add('  {#MyInts}{=TMyInts}MyInts: TMyInts;');
  Add('  {#MyBools}{=TMyBools}MyBools: TMyBools;');
  Add('begin');
  Add('  {@s}s:=[];');
  Add('  {@s}s:=[{@Red}Red]+[{@Blue}Blue,{@Gray}Gray];');
  Add('  {@s}s:=[{@Blue}Blue,{@Gray}Gray]-[{@Blue}Blue];');
  Add('  {@s}s:={@t}t+[];');
  Add('  {@s}s:=[{@Red}Red]+{@s}s;');
  Add('  {@s}s:={@s}s+[{@Red}Red];');
  Add('  {@s}s:=[{@Red}Red]-{@s}s;');
  Add('  {@s}s:={@s}s-[{@Red}Red];');
  Add('  Include({@s}s,{@Blue}Blue);');
  Add('  Include({@s}s,{@f}f);');
  Add('  Exclude({@s}s,{@Blue}Blue);');
  Add('  Exclude({@s}s,{@f}f);');
  Add('  {@s}s:={@s}s+[{@f}f];');
  Add('  if {@Green}Green in {@s}s then ;');
  Add('  if {@Blue}Blue in {@Colors}Colors then ;');
  Add('  if {@f}f in {@ExtColors}ExtColors then ;');
  Add('  {@s}s:={@s}s * {@Colors}Colors;');
  Add('  {@s}s:={@Colors}Colors * {@s}s;');
  Add('  {@s}s:={@ExtColors}ExtColors * {@Colors}Colors;');
  Add('  {@s}s:=Colors >< {@ExtColors}ExtColors;');
  Add('  {@s}s:={@s}s >< {@ExtColors}ExtColors;');
  Add('  {@s}s:={@ExtColors}ExtColors >< s;');
  Add('  {@s}s:={@s}s >< {@s}s;');
  Add('  if ''p'' in [''a''..''z''] then ; ');
  Add('  if ''p'' in [''a''..''z'',''A''..''Z'',''0''..''9'',''_''] then ; ');
  Add('  if ''p'' in {@Chars}Chars then ; ');
  Add('  if 7 in {@MyInts}MyInts then ; ');
  Add('  if 7 in [1+2,(3*4)+5,(-2+6)..(8-3)] then ; ');
  Add('  if [red,blue]*s=[red,blue] then ;');
  Add('  if {@s}s = t then;');
  Add('  if {@s}s = {@Colors}Colors then;');
  Add('  if {@Colors}Colors = s then;');
  Add('  if {@s}s <> t then;');
  Add('  if {@s}s <> {@Colors}Colors then;');
  Add('  if {@Colors}Colors <> s then;');
  Add('  if {@s}s <= t then;');
  Add('  if {@s}s <= {@Colors}Colors then;');
  Add('  if {@Colors}Colors <= s then;');
  Add('  if {@s}s >= t then;');
  Add('  if {@s}s >= {@Colors}Colors then;');
  Add('  if {@Colors}Colors >= {@s}s then;');
  ParseProgram;
end;

procedure TTestResolver.TestEnumParams;
begin
  StartProgram(false);
  Add('type');
  Add('  TFlag = (red, green, blue);');
  Add('function {#A1}FuncA: TFlag;');
  Add('begin');
  Add('  Result:=red;');
  Add('end;');
  Add('function {#A2}FuncA(f: TFlag): TFlag;');
  Add('begin');
  Add('  Result:=f;');
  Add('end;');
  Add('var');
  Add('  f: TFlag;');
  Add('begin');
  Add('  f:={@A1}FuncA;');
  Add('  f:={@A1}FuncA();');
  Add('  f:={@A2}FuncA(f);');
  ParseProgram;
end;

procedure TTestResolver.TestSetParams;
begin
  StartProgram(false);
  Add('type');
  Add('  TFlag = (red, green, blue);');
  Add('  TFlags = set of TFlag;');
  Add('function {#A1}FuncA: TFlags;');
  Add('begin');
  Add('  Result:=[red];');
  Add('  Include(Result,green);');
  Add('  Exclude(Result,blue);');
  Add('end;');
  Add('function {#A2}FuncA(f: TFlags): TFlags;');
  Add('begin');
  Add('  Include(f,green);');
  Add('  Result:=f;');
  Add('end;');
  Add('var');
  Add('  f: TFlags;');
  Add('begin');
  Add('  f:={@A1}FuncA;');
  Add('  f:={@A1}FuncA();');
  Add('  f:={@A2}FuncA(f);');
  Add('  f:={@A2}FuncA([green]);');
  ParseProgram;
end;

procedure TTestResolver.TestSetFunctions;
begin
  StartProgram(false);
  Add('type');
  Add('  TFlag = (red, green, blue);');
  Add('  TFlags = set of TFlag;');
  Add('var');
  Add('  e: TFlag;');
  Add('  s: TFlags;');
  Add('begin');
  Add('  e:=Low(TFlags);');
  Add('  e:=Low(s);');
  Add('  e:=High(TFlags);');
  Add('  e:=High(s);');
  ParseProgram;
end;

procedure TTestResolver.TestEnumHighLow;
begin
  StartProgram(false);
  Add('type');
  Add('  TFlag = (red, green, blue);');
  Add('var f: TFlag;');
  Add('begin');
  Add('  for f:=low(TFlag) to high(TFlag) do ;');
  ParseProgram;
end;

procedure TTestResolver.TestEnumOrd;
begin
  StartProgram(false);
  Add('type');
  Add('  TFlag = (red, green, blue);');
  Add('var');
  Add('  f: TFlag;');
  Add('  i: longint;');
  Add('begin');
  Add('  i:=ord(f);');
  Add('  i:=ord(green);');
  Add('  if i=ord(f) then ;');
  Add('  if ord(f)=i then ;');
  ParseProgram;
end;

procedure TTestResolver.TestEnumPredSucc;
begin
  StartProgram(false);
  Add('type');
  Add('  TFlag = (red, green, blue);');
  Add('var');
  Add('  f: TFlag;');
  Add('begin');
  Add('  f:=Pred(f);');
  Add('  if Pred(green)=Pred(TFlag.Blue) then;');
  Add('  f:=Succ(f);');
  Add('  if Succ(green)=Succ(TFlag.Blue) then;');
  ParseProgram;
end;

procedure TTestResolver.TestEnum_EqualNilFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TFlag = (red, green);');
  Add('var');
  Add('  f: TFlag;');
  Add('begin');
  Add('  if f=nil then ;');
  CheckResolverException('Incompatible types: got "TFlag" expected "Pointer"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestEnum_CastIntegerToEnum;
begin
  StartProgram(false);
  Add('type');
  Add('  TFlag = (red, green, blue);');
  Add('var');
  Add('  f: TFlag;');
  Add('  i: longint;');
  Add('begin');
  Add('  f:=TFlag(1);');
  Add('  f:=TFlag({#a_read}i);');
  Add('  if TFlag({#b_read}i)=TFlag(1) then;');
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestEnum_Str;
begin
  StartProgram(false);
  Add([
  'type',
  '  TFlag = (red, green, blue);',
  'var',
  '  f: TFlag;',
  '  i: longint;',
  '  aString: string;',
  'begin',
  '  aString:=str(f);',
  '  aString:=str(f:3);',
  '  str(f,aString);',
  '  writestr(astring,f,i);',
  '  val(aString,f,i);']);
  ParseProgram;
end;

procedure TTestResolver.TestConstEnumOperators;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red,blue,green);',
  'const',
  '  a=ord(red);',
  '  b=succ(low(TEnum));',
  '  c=pred(high(TEnum));',
  '  d=TEnum(0);',
  '  e=TEnum(2);',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestEnumSetConstRange;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red,blue,green);',
  '  TEnums = set of TEnum;',
  'const',
  '  teAny = [low(TEnum)..high(TEnum)];',
  '  teRedBlue = [low(TEnum)..pred(high(TEnum))];',
  'var',
  '  e: TEnum;',
  '  s: TEnums;',
  'begin',
  '  if blue in teAny then;',
  '  if blue in teAny+[e] then;',
  '  if blue in teAny+teRedBlue then;',
  '  s:=teAny;',
  '  s:=teAny+[e];',
  '  s:=[e]+teAny;',
  '  s:=teAny+teRedBlue;',
  '  s:=teAny+teRedBlue+[e];',
  '']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestEnumSet_AnonymousEnumtype;
begin
  StartProgram(false);
  Add('type');
  Add('  TFlags = set of (red, green);');
  Add('const');
  Add('  favorite = red;');
  Add('var');
  Add('  f: TFlags;');
  Add('  i: longint;');
  Add('begin');
  Add('  Include(f,red);');
  Add('  Include(f,favorite);');
  Add('  i:=ord(red);');
  Add('  i:=ord(favorite);');
  Add('  i:=ord(low(TFlags));');
  Add('  i:=ord(low(f));');
  Add('  i:=ord(low(favorite));');
  Add('  i:=ord(high(TFlags));');
  Add('  i:=ord(high(f));');
  Add('  i:=ord(high(favorite));');
  Add('  f:=[green,favorite];');
  ParseProgram;
end;

procedure TTestResolver.TestEnumSet_AnonymousEnumtypeName;
begin
  ResolverEngine.AnonymousElTypePostfix:='$enum';
  StartProgram(false);
  Add('type');
  Add('  TFlags = set of (red, green);');
  Add('const');
  Add('  favorite = red;');
  Add('var');
  Add('  f: TFlags;');
  Add('  i: longint;');
  Add('begin');
  Add('  Include(f,red);');
  Add('  Include(f,favorite);');
  Add('  i:=ord(red);');
  Add('  i:=ord(favorite);');
  Add('  i:=ord(low(TFlags));');
  Add('  i:=ord(low(f));');
  Add('  i:=ord(low(favorite));');
  Add('  i:=ord(high(TFlags));');
  Add('  i:=ord(high(f));');
  Add('  i:=ord(high(favorite));');
  Add('  f:=[green,favorite];');
  ParseProgram;
end;

procedure TTestResolver.TestEnumSet_Const;
begin
  StartProgram(false);
  Add([
  'type',
  '  TFlag = (a,b,c,d,e,f);',
  'const',
  '  s1 = [a];',
  '  s2 = [a,b];',
  '  s3 = [a..c];',
  '  s4 = [a..b,d..e,f];',
  '  s5 = [low(TFlag)..high(TFlag)];',
  '  s6 = [succ(low(TFlag))..pred(high(TFlag))];',
  '  s7 = [a..c]*[b..d];',
  '  s8 = [a..e]-[b,e];',
  '  s9 = [a,c..d]+[b,e];',
  '  s10 = [a..c]><[b..e];',
  '  s11 = [a,b]=[a..b];',
  '  s12 = [a,b]<>[a..b];',
  '  s13 = [a,b]<=[a..b];',
  '  s14 = [a,b]>=[a..b];',
  '  s15 = a in [a,b];',
  'var',
  '  Flag: TFlag;',
  'begin',
  '  if Flag in [b,c] then ;']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestSet_IntRange_Const;
begin
  StartProgram(false);
  Add([
  'type',
  '  TIntRg = 2..6;',
  '  TFiveSet = set of TIntRg;',
  'const',
  '  Three = 3;',
  '  a: TFiveSet = [2..Three,5]+[4];',
  '  b = low(TIntRg)+high(TIntRg);',
  '  c = [low(TIntRg)..high(TIntRg)];',
  'var',
  '  s: TFiveSet;',
  'begin',
  '  s:= {#s1_set}[];',
  '  s:= {#s2_set}[3];',
  '  s:= {#s3_set}[3..4];',
  '  s:= {#s4_set}[Three];',
  '  if 3 in a then ;',
  '  s:=c;']);
  ParseProgram;
  CheckParamsExpr_pkSet_Markers;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestSet_Byte_Const;
begin
  StartProgram(false);
  Add([
  'type',
  '  TIntRg = byte;',
  '  TFiveSet = set of TIntRg;',
  'const',
  '  Three = 3;',
  '  a: TFiveSet = [2..Three,5]+[4];',
  '  b = low(TIntRg)+high(TIntRg);',
  '  c = [low(TIntRg)..high(TIntRg)];',
  'var',
  '  s: TFiveSet;',
  'begin',
  '  s:= {#s1_set}[];',
  '  s:= {#s2_set}[3];',
  '  s:= {#s3_set}[3..4];',
  '  s:= {#s4_set}[Three];',
  '  if 3 in a then ;',
  '  s:=c;',
  //'  Include(s,Three);', // ToDo
  '']);
  ParseProgram;
  CheckParamsExpr_pkSet_Markers;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestEnumRange;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (a,b,c,d,e);',
  '  TEnumRg = b..d;',
  '  TEnumRg2 = c..e;',
  '  TSetOfEnumRg = set of TEnumRg;',
  'const',
  '  c1: TEnumRg = c;',
  '  c2: TEnumRg = succ(low(TEnumRg));',
  '  c3: TEnumRg = pred(high(TEnumRg));',
  '  c4: TEnumRg = TEnumRg(2);',
  '  c5: TEnumRg2 = e;',
  'var',
  '  er: TEnumRg;',
  '  er2: TEnumRg2;',
  '  Enum: TEnum;',
  '  i: longint;',
  '  sr: TSetOfEnumRg;',
  'begin',
  '  er:=d;',
  '  Enum:=er;',
  '  if Enum=er then ;',
  '  if er=Enum then ;',
  '  if er=c then ;',
  '  if c=er then ;',
  '  if er=er2 then ;',
  '  er:=er2;',
  '  i:=ord(er);',
  '  er:=TEnumRg(i);',
  '  i:=longint(er);',
  '  if b in sr then ;',
  '  if er in sr then ;',
  '']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestEnum_ForIn;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red,green,blue);',
  '  TEnumRg = green..blue;',
  '  TSetOfEnum = set of TEnum;',
  '  TSetOfEnumRg = set of TEnumRg;',
  '  TArrOfEnum = array[TEnum] of byte;',
  '  TArrOfEnumRg = array[TEnumRg] of byte;',
  'var',
  '  e: TEnum;',
  '  er: TEnumRg;',
  'begin',
  '  for e in TEnum do;',
  '  for e in TEnumRg do;',
  '  for e in TSetOfEnum do;',
  '  for e in TSetOfEnumRg do;',
  '  for e in [] do;',
  '  for e in [red..green] do;',
  '  for e in [green,blue] do;',
  '  for e in TArrOfEnum do;',
  '  for e in TArrOfEnumRg do;',
  '  for er in TEnumRg do;',
  '  for er in TSetOfEnumRg do;',
  '  for er in [green..blue] do;',
  '  for er in TArrOfEnumRg do;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestEnum_ForInRangeFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red,green,blue);',
  'var',
  '  e: TEnum;',
  'begin',
  '  for e in red..green do;',
  '']);
  CheckResolverException('Cannot find an enumerator for the type "range.."',nCannotFindEnumeratorForType);
end;

procedure TTestResolver.TestEnum_ScopedEnums;
begin
  StartProgram(false);
  Add([
  'type',
  '  {$scopedenums on}',
  '  TEnum = (red, green);',
  '  TFlags = set of (red,blue);',
  '  ',
  'var e: TEnum;',
  '  f: TFlags;',
  'begin',
  '  e:=TEnum.red;',
  '  if red in f then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestEnum_ScopedEnumsFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  {$ScopedEnums on}',
  '  TEnum = (red, green);',
  'var e: TEnum;',
  'begin',
  '  e:=red;'
  ]);
  CheckResolverException(sIdentifierNotFound,nIdentifierNotFound);
end;

procedure TTestResolver.TestPrgAssignment;
var
  El: TPasElement;
  V1: TPasVariable;
  ImplAssign: TPasImplAssign;
  Ref1: TPrimitiveExpr;
  Resolver1: TResolvedReference;
begin
  StartProgram(false);
  Add('var');
  Add('  v1:longint;');
  Add('begin');
  Add('  v1:=3;');
  ParseProgram;
  AssertEquals('1 declaration',1,PasProgram.ProgramSection.Declarations.Count);

  El:=TPasElement(PasProgram.ProgramSection.Declarations[0]);
  AssertEquals('var',TPasVariable,El.ClassType);
  V1:=TPasVariable(El);
  AssertEquals('var v1','v1',V1.Name);

  AssertEquals('1 statement',1,PasProgram.InitializationSection.Elements.Count);
  AssertEquals('Assignment statement',TPasImplAssign,FFirstStatement.ClassType);
  ImplAssign:=FFirstStatement as TPasImplAssign;
  AssertEquals('Normal assignment',akDefault,ImplAssign.Kind);
  AssertExpression('Right side is constant',ImplAssign.Right,pekNumber,'3');
  AssertExpression('Left side is variable',ImplAssign.Left,pekIdent,'v1');
  AssertEquals('Left side is variable, primitive',TPrimitiveExpr,ImplAssign.Left.ClassType);
  Ref1:=TPrimitiveExpr(ImplAssign.Left);
  AssertNotNull('variable has customdata',Ref1.CustomData);
  AssertEquals('variable has resolver',TResolvedReference,Ref1.CustomData.ClassType);
  Resolver1:=TResolvedReference(Ref1.CustomData);
  AssertSame('variable resolver element',Resolver1.Element,Ref1);
  AssertSame('variable resolver declaration v1',Resolver1.Declaration,V1);
end;

procedure TTestResolver.TestPrgProcVar;
begin
  StartProgram(false);
  Add('procedure Proc1;');
  Add('type');
  Add('  t1=longint;');
  Add('var');
  Add('  v1:t1;');
  Add('begin');
  Add('end;');
  Add('begin');
  ParseProgram;
  AssertEquals('1 declaration',1,PasProgram.ProgramSection.Declarations.Count);
end;

procedure TTestResolver.TestUnitProcVar;
var
  El: TPasElement;
  IntfProc1, ImplProc1: TPasProcedure;
  IntfType1, ProcSubType1: TPasAliasType;
  ImplVar1, ProcSubVar1: TPasVariable;
  ImplVar1Type, ProcSubVar1Type: TPasType;
begin
  StartUnit(false);
  Add('interface');
  Add('');
  Add('type t1=string; // unit scope');
  Add('procedure Proc1;');
  Add('');
  Add('implementation');
  Add('');
  Add('procedure Proc1;');
  Add('type t1=longint; // local proc scope');
  Add('var  v1:t1; // using local t1');
  Add('begin');
  Add('end;');
  Add('var  v2:t1; // using interface t1');
  ParseUnit;

  // interface
  AssertEquals('2 intf declarations',2,Module.InterfaceSection.Declarations.Count);
  El:=TPasElement(Module.InterfaceSection.Declarations[0]);
  AssertEquals('intf type',TPasAliasType,El.ClassType);
  IntfType1:=TPasAliasType(El);
  AssertEquals('intf type t1','t1',IntfType1.Name);

  El:=TPasElement(Module.InterfaceSection.Declarations[1]);
  AssertEquals('intf proc',TPasProcedure,El.ClassType);
  IntfProc1:=TPasProcedure(El);
  AssertEquals('intf proc Proc1','Proc1',IntfProc1.Name);

  // implementation
  AssertEquals('2 impl declarations',2,Module.ImplementationSection.Declarations.Count);
  El:=TPasElement(Module.ImplementationSection.Declarations[0]);
  AssertEquals('impl proc',TPasProcedure,El.ClassType);
  ImplProc1:=TPasProcedure(El);
  AssertEquals('impl proc Proc1','Proc1',ImplProc1.Name);

  El:=TPasElement(Module.ImplementationSection.Declarations[1]);
  AssertEquals('impl var',TPasVariable,El.ClassType);
  ImplVar1:=TPasVariable(El);
  AssertEquals('impl var v2','v2',ImplVar1.Name);
  ImplVar1Type:=TPasType(ImplVar1.VarType);
  AssertSame('impl var type is intf t1',IntfType1,ImplVar1Type);

  // proc
  AssertEquals('2 proc sub declarations',2,ImplProc1.Body.Declarations.Count);

  // proc sub type t1
  El:=TPasElement(ImplProc1.Body.Declarations[0]);
  AssertEquals('proc sub type',TPasAliasType,El.ClassType);
  ProcSubType1:=TPasAliasType(El);
  AssertEquals('proc sub type t1','t1',ProcSubType1.Name);

  // proc sub var v1
  El:=TPasElement(ImplProc1.Body.Declarations[1]);
  AssertEquals('proc sub var',TPasVariable,El.ClassType);
  ProcSubVar1:=TPasVariable(El);
  AssertEquals('proc sub var v1','v1',ProcSubVar1.Name);
  ProcSubVar1Type:=TPasType(ProcSubVar1.VarType);
  AssertSame('proc sub var type is proc sub t1',ProcSubType1,ProcSubVar1Type);
end;

procedure TTestResolver.TestAssignIntegers;
begin
  StartProgram(false);
  Add('var');
  Add('  {#vbyte}vbyte:byte;');
  Add('  {#vshortint}vshortint:shortint;');
  Add('  {#vword}vword:word;');
  Add('  {#vsmallint}vsmallint:smallint;');
  Add('  {#vlongword}vlongword:longword;');
  Add('  {#vlongint}vlongint:longint;');
  Add('  {#vqword}vqword:qword;');
  Add('  {#vint64}vint64:int64;');
  Add('  {#vcomp}vcomp:comp;');
  Add('begin');
  Add('  {@vbyte}vbyte:=0;');
  Add('  {@vbyte}vbyte:=255;');
  Add('  {@vshortint}vshortint:=0;');
  Add('  {@vshortint}vshortint:=-128;');
  Add('  {@vshortint}vshortint:= 127;');
  Add('  {@vword}vword:=0;');
  Add('  {@vword}vword:=+$ffff;');
  Add('  {@vsmallint}vsmallint:=0;');
  Add('  {@vsmallint}vsmallint:=-$8000;');
  Add('  {@vsmallint}vsmallint:= $7fff;');
  Add('  {@vlongword}vlongword:=0;');
  Add('  {@vlongword}vlongword:=$ffffffff;');
  Add('  {@vlongint}vlongint:=0;');
  Add('  {@vlongint}vlongint:=-$80000000;');
  Add('  {@vlongint}vlongint:= $7fffffff;');
  Add('  {@vlongint}vlongint:={@vbyte}vbyte;');
  Add('  {@vlongint}vlongint:={@vshortint}vshortint;');
  Add('  {@vlongint}vlongint:={@vword}vword;');
  Add('  {@vlongint}vlongint:={@vsmallint}vsmallint;');
  Add('  {@vlongint}vlongint:={@vlongint}vlongint;');
  Add('  {@vint64}vint64:=0;');
  Add('  {@vint64}vint64:=-$8000000000000000;');
  Add('  {@vint64}vint64:= $7fffffffffffffff;');
  Add('  {@vqword}vqword:=0;');
  Add('  {@vqword}vqword:=$ffffffffffffffff;');
  Add('  {@vcomp}vcomp:=0;');
  Add('  {@vcomp}vcomp:=-$8000000000000000;');
  Add('  {@vcomp}vcomp:= $7fffffffffffffff;');
  ParseProgram;
end;

procedure TTestResolver.TestAssignString;
begin
  StartProgram(false);
  Add('var');
  Add('  vstring:string;');
  Add('  vchar:char;');
  Add('begin');
  Add('  vstring:='''';');
  Add('  vstring:=''abc'';');
  Add('  vstring:=''a'';');
  Add('  vchar:=''c'';');
  Add('  vchar:=vstring[1];');
  ParseProgram;
end;

procedure TTestResolver.TestAssignIntToStringFail;
begin
  StartProgram(false);
  Add('var');
  Add('  vstring:string;');
  Add('begin');
  Add('  vstring:=2;');
  CheckResolverException('Incompatible types: got "Longint" expected "String"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestAssignStringToIntFail;
begin
  StartProgram(false);
  Add('var');
  Add('  v:longint;');
  Add('begin');
  Add('  v:=''A'';');
  CheckResolverException('Incompatible types: got "Char" expected "Longint"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestIntegerOperators;
begin
  StartProgram(false);
  Add('var');
  Add('  i,j,k:longint;');
  Add('begin');
  Add('  i:=1;');
  Add('  i:=1+2;');
  Add('  i:=1+2+3;');
  Add('  i:=1-2;');
  Add('  i:=j;');
  Add('  i:=j+1;');
  Add('  i:=-j+1;');
  Add('  i:=j+k;');
  Add('  i:=-j+k;');
  Add('  i:=j*k;');
  Add('  i:=j**k;');
  Add('  i:=10**3;');
  Add('  i:=j div k;');
  Add('  i:=10 div 3;');
  Add('  i:=j mod k;');
  Add('  i:=10 mod 3;');
  Add('  i:=j shl k;');
  Add('  i:=j shr k;');
  Add('  i:=j and k;');
  Add('  i:=j or k;');
  Add('  i:=j and not k;');
  Add('  i:=(j+k) div 3;');
  Add('  if i=j then;');
  Add('  if i<>j then;');
  Add('  if i>j then;');
  Add('  if i>=j then;');
  Add('  if i<j then;');
  Add('  if i<=j then;');
  Add('  i:=lo($1234);');
  Add('  i:=lo($1234CDEF);');
  Add('  i:=hi($1234);');
  Add('  i:=hi($1234CDEF);');
  ParseProgram;
end;

procedure TTestResolver.TestIntegerBoolFail;
begin
  StartProgram(false);
  Add([
  'var i: longint;',
  'begin',
  '  i:=3 * false;']);
  CheckResolverException('Operator is not overloaded: "Longint" * "Boolean"',
    nOperatorIsNotOverloadedAOpB);
end;

procedure TTestResolver.TestBooleanOperators;
begin
  StartProgram(false);
  Add('var');
  Add('  i,j,k:boolean;');
  Add('begin');
  Add('  i:=false;');
  Add('  i:=true;');
  Add('  i:=j and k;');
  Add('  i:=j or k;');
  Add('  i:=j or not k;');
  Add('  i:=(not j) or k;');
  Add('  i:=j or false;');
  Add('  i:=j and true;');
  Add('  i:=j xor k;');
  Add('  i:=j=k;');
  Add('  i:=j<>k;');
  ParseProgram;
end;

procedure TTestResolver.TestStringOperators;
begin
  StartProgram(false);
  Add([
  'var',
  '  i,j:string;',
  '  k:char;',
  '  w:widechar;',
  'begin',
  '  i:='''';',
  '  i:=''''+'''';',
  '  i:=k+'''';',
  '  i:=''''+k;',
  '  i:=''a''+j;',
  '  i:=''abc''+j;',
  '  k:=#65;',
  '  k:=#$42;',
  '  k:=''a'';',
  '  k:='''''''';',
  '  k:=j[1];',
  '  k:=char(#10);',
  '  w:=k;',
  '  w:=#66;',
  '  w:=#6666;',
  '  w:=widechar(#10);',
  '  w:=widechar(#$E0000);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestWideCharOperators;
begin
  ResolverEngine.ExprEvaluator.DefaultStringCodePage:=CP_UTF8;
  ResolverEngine.BaseTypeChar:=btWideChar;
  ResolverEngine.BaseTypeString:=btUnicodeString;
  StartProgram(false);
  Add('var');
  Add('  k:char;');
  Add('  w:widechar;');
  Add('begin');
  Add('  w:=k;');
  Add('  w:=#66;');
  Add('  w:=#6666;');
  Add('  w:=''ä'';');
  ParseProgram;
end;

procedure TTestResolver.TestFloatOperators;
begin
  StartProgram(false);
  Add('var');
  Add('  i,j,k:double;');
  Add('  o,p:longint;');
  Add('begin');
  Add('  i:=1;');
  Add('  i:=1+2;');
  Add('  i:=1+2+3;');
  Add('  i:=1-2;');
  Add('  i:=j;');
  Add('  i:=j+1;');
  Add('  i:=-j+1;');
  Add('  i:=j+k;');
  Add('  i:=-j+k;');
  Add('  i:=j*k;');
  Add('  i:=10/3;');
  Add('  i:=10.0/3;');
  Add('  i:=10/3.0;');
  Add('  i:=10.0/3.0;');
  Add('  i:=j/k;');
  Add('  i:=o/p;');
  Add('  i:=10**3;');
  Add('  i:=10.0**3;');
  Add('  i:=10.0**3.0;');
  Add('  i:=10**3.0;');
  Add('  i:=j**k;');
  Add('  i:=o**p;');
  Add('  i:=(j+k)/3;');
  ParseProgram;
end;

procedure TTestResolver.TestCAssignments;
begin
  StartProgram(false);
  Parser.Options:=Parser.Options+[po_cassignments];
  Add('Type');
  Add('  TFlag = (Flag1,Flag2);');
  Add('  TFlags = set of TFlag;');
  Add('var');
  Add('  i: longint;');
  Add('  c: char;');
  Add('  s: string;');
  Add('  d: double;');
  Add('  f: TFlag;');
  Add('  fs: TFlags;');
  Add('begin');
  Add('  i+=1;');
  Add('  i-=2;');
  Add('  i*=3;');
  Add('  s+=''A'';');
  Add('  s:=c;');
  Add('  d+=4;');
  Add('  d-=5;');
  Add('  d*=6;');
  Add('  d/=7;');
  Add('  d+=8.5;');
  Add('  d-=9.5;');
  Add('  d*=10.5;');
  Add('  d/=11.5;');
  Add('  fs+=[f];');
  Add('  fs-=[f];');
  Add('  fs*=[f];');
  Add('  fs+=[Flag1];');
  Add('  fs-=[Flag1];');
  Add('  fs*=[Flag1];');
  Add('  fs+=[Flag1,Flag2];');
  Add('  fs-=[Flag1,Flag2];');
  Add('  fs*=[Flag1,Flag2];');
  ParseProgram;
end;

procedure TTestResolver.TestTypeCastBaseTypes;
begin
  StartProgram(false);
  Add([
  'var',
  '  si: smallint;',
  '  i: longint;',
  '  fs: single;',
  '  d: double;',
  '  b: boolean;',
  '  c: char;',
  '  s: string;',
  'begin',
  '  d:=double({#a_read}i);',
  '  i:=shortint({#b_read}i);',
  '  i:=longint({#c_read}si);',
  '  d:=double({#d_read}d);',
  '  fs:=single({#e_read}d);',
  '  d:=single({#f_read}d);',
  '  b:=longbool({#g_read}b);',
  '  b:=bytebool({#i_read}longbool({#h_read}b));',
  '  d:=double({#j_read}i)/2.5;',
  '  b:=boolean({#k_read}i);',
  '  i:=longint({#l_read}b);',
  '  d:=double({#m_read}i);',
  '  c:=char({#n_read}c);',
  '  c:=char({#o_read}i);',
  '  c:=char(65);',
  '  s:=string({#p_read}s);',
  '  s:=string({#q_read}c);',
  '']);
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestTypeCastAliasBaseTypes;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TCaption = string;');
  Add('  TYesNo = boolean;');
  Add('  TFloat = double;');
  Add('  TChar = char;');
  Add('var');
  Add('  i: longint;');
  Add('  s: string;');
  Add('  b: boolean;');
  Add('  d: double;');
  Add('  c: char;');
  Add('begin');
  Add('  i:=integer({#a_read}i);');
  Add('  i:=integer({#h_read}b);');
  Add('  s:=TCaption({#b_read}s);');
  Add('  s:=TCaption({#g_read}c);');
  Add('  b:=TYesNo({#c_read}b);');
  Add('  b:=TYesNo({#d_read}i);');
  Add('  d:=TFloat({#e_read}d);');
  Add('  c:=TChar({#f_read}c);');
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestTypeCastStrToIntFail;
begin
  StartProgram(false);
  Add('var');
  Add('  s: string;');
  Add('  i: longint;');
  Add('begin');
  Add('  i:=longint(s);');
  CheckResolverException(sIllegalTypeConversionTo,nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestTypeCastStrToCharFail;
begin
  StartProgram(false);
  Add('var');
  Add('  s: string;');
  Add('  c: char;');
  Add('begin');
  Add('  c:=char(s);');
  CheckResolverException(sIllegalTypeConversionTo,nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestTypeCastIntToStrFail;
begin
  StartProgram(false);
  Add('var');
  Add('  s: string;');
  Add('  i: longint;');
  Add('begin');
  Add('  s:=string(i);');
  CheckResolverException(sIllegalTypeConversionTo,nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestTypeCastDoubleToStrFail;
begin
  StartProgram(false);
  Add('var');
  Add('  s: string;');
  Add('  d: double;');
  Add('begin');
  Add('  s:=string(d);');
  CheckResolverException(sIllegalTypeConversionTo,nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestTypeCastDoubleToIntFail;
begin
  StartProgram(false);
  Add('var');
  Add('  i: longint;');
  Add('  d: double;');
  Add('begin');
  Add('  i:=longint(d);');
  CheckResolverException(sIllegalTypeConversionTo,nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestTypeCastDoubleToBoolFail;
begin
  StartProgram(false);
  Add('var');
  Add('  b: boolean;');
  Add('  d: double;');
  Add('begin');
  Add('  b:=longint(d);');
  CheckResolverException(sIllegalTypeConversionTo,nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestTypeCastBooleanToDoubleFail;
begin
  StartProgram(false);
  Add('var');
  Add('  b: boolean;');
  Add('  d: double;');
  Add('begin');
  Add('  d:=double(b);');
  CheckResolverException(sIllegalTypeConversionTo,nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestAssign_Access;
begin
  StartProgram(false);
  Parser.Options:=Parser.Options+[po_cassignments];
  Add('var i: longint;');
  Add('begin');
  Add('  {#a1_assign}i:={#a2_read}i;');
  Add('  {#b1_readandassign}i+={#b2_read}i;');
  Add('  {#c1_readandassign}i-={#c2_read}i;');
  Add('  {#d1_readandassign}i*={#d2_read}i;');
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestAssignedIntFail;
begin
  StartProgram(false);
  Add('var i: longint;');
  Add('begin');
  Add('  if Assigned(i) then ;');
  CheckResolverException('Incompatible type arg no. 1: Got "Longint", expected "class or array"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestHighLow;
begin
  StartProgram(false);
  Add('var');
  Add('  bo: boolean;');
  Add('  by: byte;');
  Add('  ch: char;');
  Add('begin');
  Add('  for bo:=low(boolean) to high(boolean) do;');
  Add('  for by:=low(byte) to high(byte) do;');
  Add('  for ch:=low(char) to high(char) do;');
  ParseProgram;
end;

procedure TTestResolver.TestStr_BaseTypes;
begin
  StartProgram(false);
  Add('var');
  Add('  b: boolean;');
  Add('  i: longint;');
  Add('  i64: int64;');
  Add('  s: single;');
  Add('  d: double;');
  Add('  aString: string;');
  Add('  r: record end;');
  Add('begin');
  Add('  Str(b,{#a_var}aString);');
  Add('  Str(b:1,aString);');
  Add('  Str(b:i,aString);');
  Add('  Str(i,aString);');
  Add('  Str(i:2,aString);');
  Add('  Str(i:i64,aString);');
  Add('  Str(i64,aString);');
  Add('  Str(i64:3,aString);');
  Add('  Str(i64:i,aString);');
  Add('  Str(s,aString);');
  Add('  Str(d,aString);');
  Add('  Str(d:4,aString);');
  Add('  Str(d:4:5,aString);');
  Add('  Str(d:4:i,aString);');
  Add('  aString:=Str(b);');
  Add('  aString:=Str(i:3);');
  Add('  aString:=Str(d:3:4);');
  Add('  aString:=Str(b,i,d);');
  Add('  aString:=Str(s,''foo'');');
  Add('  aString:=Str(i,{#assign_read}aString);');
  Add('  while true do Str(i,{#whiledo_var}aString);');
  Add('  repeat Str(i,{#repeat_var}aString); until true;');
  Add('  if true then Str(i,{#ifthen_var}aString) else Str(i,{#ifelse_var}aString);');
  Add('  for i:=0 to 0 do Str(i,{#fordo_var}aString);');
  Add('  with r do Str(i,{#withdo_var}aString);');
  Add('  case Str(s,''caseexpr'') of');
  Add('  ''bar'': Str(i,{#casest_var}aString);');
  Add('  else Str(i,{#caseelse_var}aString);');
  Add('  end;');
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestStr_StringFail;
begin
  StartProgram(false);
  Add('var');
  Add('  aString: string;');
  Add('begin');
  Add('  Str(aString,aString);');
  CheckResolverException('Incompatible type arg no. 1: Got "String", expected "boolean, integer, enum value"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestStr_CharFail;
begin
  StartProgram(false);
  Add('var');
  Add('  c: char;');
  Add('  aString: string;');
  Add('begin');
  Add('  Str(c,aString);');
  CheckResolverException('Incompatible type arg no. 1: Got "Char", expected "boolean, integer, enum value"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestIncDec;
begin
  StartProgram(false);
  Add('var');
  Add('  i: longint;');
  Add('begin');
  Add('  inc({#a_var}i);');
  Add('  inc({#b_var}i,2);');
  Add('  dec({#c_var}i);');
  Add('  dec({#d_var}i,3);');
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestIncStringFail;
begin
  StartProgram(false);
  Add('var');
  Add('  i: string;');
  Add('begin');
  Add('  inc(i);');
  CheckResolverException('Incompatible type arg no. 1: Got "String", expected "integer"',nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestTypeInfo;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  TRec = record',
  '    v: integer;',
  '  end;',
  '  TClass = class of TObject;',
  '  TObject = class',
  '    class function ClassType: TClass; virtual; abstract;',
  '  end;',
  'var',
  '  i: integer;',
  '  s: string;',
  '  p: pointer;',
  '  r: TRec;',
  '  o: TObject;',
  '  c: TClass;',
  'begin',
  '  p:=typeinfo(integer);',
  '  p:=typeinfo(longint);',
  '  p:=typeinfo(i);',
  '  p:=typeinfo(s);',
  '  p:=typeinfo(p);',
  '  p:=typeinfo(r.v);',
  '  p:=typeinfo(TObject.ClassType);',
  '  p:=typeinfo(o.ClassType);',
  '  p:=typeinfo(o);',
  '  p:=typeinfo(c);',
  '  p:=typeinfo(c.ClassType);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeInfo_FailRTTIDisabled;
begin
  StartProgram(false);
  Add([
  '{$modeswitch OmitRTTI}',
  'type',
  '  TObject = class',
  '  end;',
  'var o: TObject;',
  'begin',
  '  if typeinfo(o)=nil then ;',
  '']);
  CheckResolverException(sSymbolCannotBePublished,nSymbolCannotBePublished);
end;

procedure TTestResolver.TestForLoop;
begin
  StartProgram(false);
  Add('var');
  Add('  {#v1}v1,{#v2}v2,{#v3}v3:longint;');
  Add('begin');
  Add('  for {@v1}v1:=');
  Add('    {@v2}v2');
  Add('    to {@v3}v3 do ;');
  ParseProgram;
end;

procedure TTestResolver.TestForLoop_NestedSameVarFail;
begin
  StartProgram(false);
  Add([
  'var i: byte;',
  'begin',
  '  for i:=1 to 2 do',
  '    for i:=1 to 2 do ;',
  '']);
  CheckResolverException('Illegal assignment to for-loop variable "i"',nIllegalAssignmentToForLoopVar);
end;

procedure TTestResolver.TestForLoop_AssignVarFail;
begin
  StartProgram(false);
  Add([
  'var i: byte;',
  'begin',
  '  for i:=1 to 2 do',
  '    i:=3;',
  '']);
  CheckResolverException('Illegal assignment to for-loop variable "i"',nIllegalAssignmentToForLoopVar);
end;

procedure TTestResolver.TestForLoop_PassVarFail;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(var i: byte); external;',
  'var i: byte;',
  'begin',
  '  for i:=1 to 2 do',
  '    DoIt(i);',
  '']);
  CheckResolverException('Illegal assignment to for-loop variable "i"',nIllegalAssignmentToForLoopVar);
end;

procedure TTestResolver.TestStatements;
begin
  StartProgram(false);
  Add([
  'var',
  '  v1,v2,v3:longint;',
  'begin',
  '  v1:=1;',
  '  v2:=v1+v1*v1+v1 div v1;',
  '  v3:=-v1;',
  '  repeat',
  '    v1:=v1+1;',
  '  until v1>=5;',
  '  while v1>=0 do',
  '    v1:=v1-v2;',
  '  for v1:=v2 to v3 do v2:=v1;',
  '  if v1<v2 then v3:=v1 else v3:=v2;',
  '']);
  ParseProgram;
  AssertEquals('3 declarations',3,PasProgram.ProgramSection.Declarations.Count);
end;

procedure TTestResolver.TestCaseOfInt;
begin
  StartProgram(false);
  Add('const');
  Add('  {#c1}c1=1;');
  Add('  {#c2}c2=2;');
  Add('  {#c3}c3=3;');
  Add('  {#c4}c4=4;');
  Add('  {#c5}c5=5;');
  Add('  {#c6}c6=6;');
  Add('var');
  Add('  {#v1}v1,{#v2}v2,{#v3}v3:longint;');
  Add('begin');
  Add('  Case {@v1}v1+{@v2}v2 of');
  Add('  {@c1}c1:');
  Add('    {@v2}v2:={@v3}v3;');
  Add('  {@c2}c2,{@c3}c3: ;');
  Add('  {@c4}c4..5: ;');
  Add('  {@c5}c5+{@c6}c6: ;');
  Add('  else');
  Add('    {@v1}v1:=3;');
  Add('  end;');
  ParseProgram;
end;

procedure TTestResolver.TestCaseOfIntExtConst;
begin
  Parser.Options:=Parser.Options+[po_ExtConstWithoutExpr];
  StartProgram(false);
  Add([
  'const e: longint; external;',
  'var i: longint;',
  'begin',
  '  case i of',
  '  2: ;',
  '  e: ;',
  '  1: ;',
  '  end;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestCaseIntDuplicateFail;
begin
  StartProgram(false);
  Add([
  'var i: longint;',
  'begin',
  '  case i of',
  '  2: ;',
  '  1..3: ;',
  '  end;',
  '']);
  CheckResolverException('Duplicate case value "1..3", other at afile.pp(5,3)',nDuplicateCaseValueXatY);
end;

procedure TTestResolver.TestCaseOfStringDuplicateFail;
begin
  StartProgram(false);
  Add([
  'var s: string;',
  'begin',
  '  case s of',
  '  ''a''#10''bc'': ;',
  '  ''A''#10''BC'': ;',
  '  ''a''#10''bc'': ;',
  '  end;',
  '']);
  CheckResolverException('Duplicate case value "string", other at afile.pp(5,3)',nDuplicateCaseValueXatY);
end;

procedure TTestResolver.TestCaseOfStringRangeDuplicateFail;
begin
  StartProgram(false);
  Add([
  'var s: string;',
  'begin',
  '  case s of',
  '  ''c'': ;',
  '  ''a''..''z'': ;',
  '  end;',
  '']);
  CheckResolverException('Duplicate case value "string", other at afile.pp(5,3)',nDuplicateCaseValueXatY);
end;

procedure TTestResolver.TestCaseOfBaseType;
begin
  StartProgram(false);
  Add([
  'type',
  '  TFlag = (red,green,blue);',
  'var',
  '  i: longint;',
  '  f: TFlag;',
  '  b: boolean;',
  '  c: char;',
  '  s: string;',
  'begin',
  '  case i of',
  '  1: ;',
  '  2..3: ;',
  '  4,5..6,7: ;',
  '  else',
  '  end;',
  '  case f of',
  '  red: ;',
  '  green..blue: ;',
  '  end;',
  '  case b of',
  '  true: ;',
  '  false: ;',
  '  end;',
  '  case c of',
  '  #0: ;',
  '  #10,#13: ;',
  '  ''0''..''9'',''a''..''z'': ;',
  '  end;',
  '  case s of',
  '  #10: ;',
  '  ''abc'': ;',
  '  ''a''..''z'': ;',
  '  end;']);
  ParseProgram;
end;

procedure TTestResolver.TestCaseOfExprNonOrdFail;
begin
  StartProgram(false);
  Add('begin');
  Add('  case longint of');
  Add('  1: ;');
  Add('  end;');
  CheckResolverException('ordinal expression expected, but Longint found',
    nXExpectedButYFound);
end;

procedure TTestResolver.TestCaseOfIncompatibleValueFail;
begin
  StartProgram(false);
  Add('var i: longint;');
  Add('begin');
  Add('  case i of');
  Add('  ''1'': ;');
  Add('  end;');
  CheckResolverException('Incompatible types: got "Char" expected "Longint"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestTryStatement;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class end;');
  Add('  {#Exec}Exception = class end;');
  Add('var');
  Add('  {#v1}v1,{#e1}e:longint;');
  Add('begin');
  Add('  try');
  Add('    {@v1}v1:={@e1}e;');
  Add('  finally');
  Add('    {@v1}v1:={@e1}e;');
  Add('  end;');
  Add('  try');
  Add('    {@v1}v1:={@e1}e;');
  Add('  except');
  Add('    {@v1}v1:={@e1}e;');
  Add('    raise;');
  Add('  end;');
  Add('  try');
  Add('    {@v1}v1:={@e1}e;');
  Add('  except');
  Add('    on {#e2}{=Exec}E: Exception do');
  Add('      if {@e2}e=nil then raise;');
  Add('    on {#e3}{=Exec}E: Exception do');
  Add('      raise {@e3}e;');
  Add('    else');
  Add('      {@v1}v1:={@e1}e;');
  Add('  end;');
  ParseProgram;
end;

procedure TTestResolver.TestTryExceptOnNonTypeFail;
begin
  StartProgram(false);
  Add('type TObject = class end;');
  Add('var E: TObject;');
  Add('begin');
  Add('  try');
  Add('  except');
  Add('    on E do ;');
  Add('  end;');
  CheckParserException('Expected type, but got variable',PParser.nParserExpectedTypeButGot);
end;

procedure TTestResolver.TestTryExceptOnNonClassFail;
begin
  StartProgram(false);
  Add('begin');
  Add('  try');
  Add('  except');
  Add('    on longint do ;');
  Add('  end;');
  CheckResolverException('class expected, but Longint found',nXExpectedButYFound);
end;

procedure TTestResolver.TestRaiseNonVarFail;
begin
  StartProgram(false);
  Add('type TObject = class end;');
  Add('begin');
  Add('  raise TObject;');
  CheckResolverException('variable expected, but class found',nXExpectedButYFound);
end;

procedure TTestResolver.TestRaiseNonClassFail;
begin
  StartProgram(false);
  Add('var');
  Add('  E: longint;');
  Add('begin');
  Add('  raise E;');
  CheckResolverException('class expected, but Longint found',nXExpectedButYFound);
end;

procedure TTestResolver.TestRaiseDescendant;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualNewInstance: Boolean;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create(Msg: string); external name ''ext'';',
  '  end;',
  '  Exception = class end;',
  '  EConvertError = class(Exception) end;',
  'function AssertConv(Msg: string = ''msg''): EConvertError;',
  'begin',
  '  Result:=EConvertError.{#ass}Create(Msg);',
  'end;',
  'begin',
  '  raise Exception.{#a}Create(''foo'');',
  '  raise EConvertError.{#b}Create(''bar'');',
  '  raise AssertConv(''c'');',
  '  raise AssertConv;',
  '']);
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestRaiseDescendant ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualNewInstance:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestRaiseDescendant ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if not (Ref.Declaration is TPasProcedure) then continue;
        //writeln('TTestResolver.TestRaiseDescendant ',GetObjName(Ref.Declaration),' rrfNewInstance=',rrfNewInstance in Ref.Flags);
        if (Ref.Declaration is TPasConstructor) then
          ActualNewInstance:=rrfNewInstance in Ref.Flags;
        break;
        end;
      if not ActualNewInstance then
        RaiseErrorAtSrcMarker('expected newinstance at "#'+aMarker^.Identifier+', but got normal call"',aMarker);
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestStatementsRefs;
begin
  StartProgram(false);
  Add('var');
  Add('  {#v1}v1,{#v2}v2,{#v3}v3:longint;');
  Add('begin');
  Add('  {@v1}v1:=1;');
  Add('  {@v2}v2:=');
  Add('    {@v1}v1+');
  Add('    {@v1}v1*{@v1}v1');
  Add('    +{@v1}v1 div {@v1}v1;');
  Add('  {@v3}v3:=');
  Add('    -{@v1}v1;');
  Add('  repeat');
  Add('    {@v1}v1:=');
  Add('      {@v1}v1+1;');
  Add('  until {@v1}v1>=5;');
  Add('  while {@v1}v1>=0 do');
  Add('    {@v1}v1');
  Add('    :={@v1}v1-{@v2}v2;');
  Add('  if {@v1}v1<{@v2}v2 then');
  Add('    {@v3}v3:={@v1}v1');
  Add('  else {@v3}v3:=');
  Add('    {@v2}v2;');
  ParseProgram;
  AssertEquals('3 declarations',3,PasProgram.ProgramSection.Declarations.Count);
end;

procedure TTestResolver.TestRepeatUntilNonBoolFail;
begin
  StartProgram(false);
  Add('begin');
  Add('  repeat');
  Add('  until 3;');
  CheckResolverException('Boolean expected, but Longint found',nXExpectedButYFound);
end;

procedure TTestResolver.TestWhileDoNonBoolFail;
begin
  StartProgram(false);
  Add('begin');
  Add('  while 3 do ;');
  CheckResolverException('Boolean expected, but Longint found',nXExpectedButYFound);
end;

procedure TTestResolver.TestIfThen;
begin
  StartProgram(false);
  Add([
  'var b: boolean;',
  'begin',
  '  if b then ;',
  '  if b then else ;']);
  ParseProgram;
end;

procedure TTestResolver.TestIfThenNonBoolFail;
begin
  StartProgram(false);
  Add('begin');
  Add('  if 3 then ;');
  CheckResolverException('Boolean expected, but Longint found',nXExpectedButYFound);
end;

procedure TTestResolver.TestIfAssignMissingSemicolonFail;
begin
  StartProgram(false);
  Add([
  'var',
  '  v:longint;',
  'begin',
  '  if true then v:=1',
  '  v:=2']);
  CheckParserException('Expected "Semicolon"',nParserExpectTokenError);
end;

procedure TTestResolver.TestForLoopVarNonVarFail;
begin
  StartProgram(false);
  Add('const i = 3;');
  Add('begin');
  Add('  for i:=1 to 2 do ;');
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestForLoopStartIncompFail;
begin
  StartProgram(false);
  Add('var i: char;');
  Add('begin');
  Add('  for i:=1 to 2 do ;');
  CheckResolverException('Incompatible types: got "Longint" expected "Char"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestForLoopEndIncompFail;
begin
  StartProgram(false);
  Add('var i: longint;');
  Add('begin');
  Add('  for i:=1 to ''2'' do ;');
  CheckResolverException('Incompatible types: got "Char" expected "Longint"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestSimpleStatement_VarFail;
begin
  StartProgram(false);
  Add('var i: longint;');
  Add('begin');
  Add('  i;');
  CheckResolverException('Illegal expression',nIllegalExpression);
end;

procedure TTestResolver.TestLabelStatementFail;
begin
  StartProgram(false);
  Add('var i: longint;');
  Add('begin');
  Add('  i: i;');
  CheckParserException('Expected ";"',nParserExpectTokenError);
end;

procedure TTestResolver.TestLabelStatementDelphiFail;
begin
  StartProgram(false);
  Add('{$mode delphi}');
  Add('{$goto off}');
  Add('var i: longint;');
  Add('begin');
  Add('  i: i;');
  CheckParserException('Expected ";"',nParserExpectTokenError);
end;

procedure TTestResolver.TestUnitForwardOverloads;
begin
  StartUnit(false);
  Add([
  'interface',
  'procedure {#ADecl}DoIt(vI: longint);',
  'procedure {#BDecl}DoIt(vI, vJ: longint);',
  'implementation',
  'procedure {#EDecl}DoIt(vI, vJ, vK, vL, vM: longint); forward;',
  'procedure {#C}DoIt(vI, vJ, vK: longint); begin end;',
  'procedure {#AImpl}DoIt(vi: longint); begin end;',
  'procedure {#D}DoIt(vI, vJ, vK, vL: longint); begin end;',
  'procedure {#BImpl}DoIt(vi, vj: longint); begin end;',
  'procedure {#EImpl}DoIt(vi, vj, vk, vl, vm: longint); begin end;',
  'begin',
  '  {@ADecl}DoIt(1);',
  '  {@BDecl}DoIt(2,3);',
  '  {@C}DoIt(4,5,6);',
  '  {@D}DoIt(7,8,9,10);',
  '  {@EDecl}DoIt(11,12,13,14,15);']);
  ParseUnit;
end;

procedure TTestResolver.TestUnitIntfInitialization;
var
  El, DeclEl, OtherUnit: TPasElement;
  LocalVar: TPasVariable;
  Assign1, Assign2, Assign3: TPasImplAssign;
  Prim1, Prim2: TPrimitiveExpr;
  BinExp: TBinaryExpr;
begin
  StartUnit(true);
  Add('interface');
  Add('var exitCOde: string;');
  Add('implementation');
  Add('initialization');
  Add('  ExitcodE:=''1'';');
  Add('  afile.eXitCode:=''2'';');
  Add('  System.exiTCode:=3;');
  ParseUnit;

  // interface
  AssertEquals('1 intf declaration',1,Module.InterfaceSection.Declarations.Count);
  El:=TPasElement(Module.InterfaceSection.Declarations[0]);
  AssertEquals('local var',TPasVariable,El.ClassType);
  LocalVar:=TPasVariable(El);
  AssertEquals('local var exitcode','exitCOde',LocalVar.Name);

  // initialization
  AssertEquals('3 initialization statements',3,Module.InitializationSection.Elements.Count);

  // check direct assignment to local var
  El:=TPasElement(Module.InitializationSection.Elements[0]);
  AssertEquals('direct assign',TPasImplAssign,El.ClassType);
  Assign1:=TPasImplAssign(El);
  AssertEquals('direct assign left',TPrimitiveExpr,Assign1.left.ClassType);
  Prim1:=TPrimitiveExpr(Assign1.left);
  AssertNotNull(Prim1.CustomData);
  AssertEquals('direct assign left ref',TResolvedReference,Prim1.CustomData.ClassType);
  DeclEl:=TResolvedReference(Prim1.CustomData).Declaration;
  AssertSame('direct assign local var',LocalVar,DeclEl);

  // check indirect assignment to local var: "afile.eXitCode"
  El:=TPasElement(Module.InitializationSection.Elements[1]);
  AssertEquals('indirect assign',TPasImplAssign,El.ClassType);
  Assign2:=TPasImplAssign(El);
  AssertEquals('indirect assign left',TBinaryExpr,Assign2.left.ClassType);
  BinExp:=TBinaryExpr(Assign2.left);
  AssertEquals('indirect assign first token',TPrimitiveExpr,BinExp.left.ClassType);
  Prim1:=TPrimitiveExpr(BinExp.left);
  AssertEquals('indirect assign first token','afile',Prim1.Value);
  AssertNotNull(Prim1.CustomData);
  AssertEquals('indirect assign unit ref resolved',TResolvedReference,Prim1.CustomData.ClassType);
  DeclEl:=TResolvedReference(Prim1.CustomData).Declaration;
  AssertSame('indirect assign unit ref',Module,DeclEl);

  AssertEquals('indirect assign dot',eopSubIdent,BinExp.OpCode);

  AssertEquals('indirect assign second token',TPrimitiveExpr,BinExp.right.ClassType);
  Prim2:=TPrimitiveExpr(BinExp.right);
  AssertEquals('indirect assign second token','eXitCode',Prim2.Value);
  AssertNotNull(Prim2.CustomData);
  AssertEquals('indirect assign var ref resolved',TResolvedReference,Prim2.CustomData.ClassType);
  AssertEquals('indirect assign left ref',TResolvedReference,Prim2.CustomData.ClassType);
  DeclEl:=TResolvedReference(Prim2.CustomData).Declaration;
  AssertSame('indirect assign local var',LocalVar,DeclEl);

  // check assignment to "system.ExitCode"
  El:=TPasElement(Module.InitializationSection.Elements[2]);
  AssertEquals('other unit assign',TPasImplAssign,El.ClassType);
  Assign3:=TPasImplAssign(El);
  AssertEquals('other unit assign left',TBinaryExpr,Assign3.left.ClassType);
  BinExp:=TBinaryExpr(Assign3.left);
  AssertEquals('othe unit assign first token',TPrimitiveExpr,BinExp.left.ClassType);
  Prim1:=TPrimitiveExpr(BinExp.left);
  AssertEquals('other unit assign first token','System',Prim1.Value);
  AssertNotNull(Prim1.CustomData);
  AssertEquals('other unit assign unit ref resolved',TResolvedReference,Prim1.CustomData.ClassType);
  DeclEl:=TResolvedReference(Prim1.CustomData).Declaration;
  OtherUnit:=DeclEl;
  AssertEquals('other unit assign unit ref',TPasUsesUnit,DeclEl.ClassType);
  AssertEquals('other unit assign unit ref system','system',lowercase(DeclEl.Name));

  AssertEquals('other unit assign dot',eopSubIdent,BinExp.OpCode);

  AssertEquals('other unit assign second token',TPrimitiveExpr,BinExp.right.ClassType);
  Prim2:=TPrimitiveExpr(BinExp.right);
  AssertEquals('other unit assign second token','exiTCode',Prim2.Value);
  AssertNotNull(Prim2.CustomData);
  AssertEquals('other unit assign var ref resolved',TResolvedReference,Prim2.CustomData.ClassType);
  AssertEquals('other unit assign left ref',TResolvedReference,Prim2.CustomData.ClassType);
  DeclEl:=TResolvedReference(Prim2.CustomData).Declaration;
  AssertEquals('other unit assign var',TPasVariable,DeclEl.ClassType);
  AssertEquals('other unit assign var exitcode','exitcode',lowercase(DeclEl.Name));
  AssertSame('other unit assign var exitcode',(OtherUnit as TPasUsesUnit).Module,DeclEl.GetModule);
end;

procedure TTestResolver.TestUnitUseSystem;
begin
  StartProgram(true);
  Add('type number = system.integer;');
  Add('begin');
  Add('  if ExitCode=2 then ;');
  ParseProgram;
end;

procedure TTestResolver.TestUnitUseIntf;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'type TListCallBack = procedure;',
    'var i: longint;',
    'procedure DoIt;',
    '']),
    LinesToStr([
    'procedure DoIt; begin end;']));

  StartProgram(true);
  Add('uses unit2;');
  Add('type TListCB = unit2.tlistcallback;');
  Add('begin');
  Add('  if i=2 then');
  Add('    DoIt;');
  ParseProgram;
end;

procedure TTestResolver.TestUnitUseImplFail;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    '']),
    LinesToStr([
    'procedure DoIt; begin end;']));

  StartProgram(true);
  Add('uses unit2;');
  Add('begin');
  Add('  DoIt;');
  CheckResolverException('identifier not found "DoIt"',nIdentifierNotFound);
end;

procedure TTestResolver.TestUnit_DuplicateUsesFail;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add('uses unit2, unit2;');
  Add('begin');
  Add('  i:=3;');
  CheckParserException('Duplicate identifier "unit2"',
    nParserDuplicateIdentifier);
end;

procedure TTestResolver.TestUnit_DuplicateUsesIntfImplFail;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'type number = longint;']),
    LinesToStr([
    '']));

  StartUnit(true);
  Add([
  'interface',
  'uses unit2;',
  'var j: number;',
  'implementation',
  'uses unit2;',
  'initialization',
  '  if number(3) then ;',
  '']);
  CheckParserException('Duplicate identifier "unit2" at token ";" in file afile.pp at line 6 column 11',
    nParserDuplicateIdentifier);
end;

procedure TTestResolver.TestUnit_NestedFail;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i2: longint;']),
    LinesToStr([
    '']));

  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'uses unit2;',
    'var j1: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  'uses unit1;',
  'begin',
  '  if j1=0 then ;',
  '  if i2=0 then ;',
  '']);
  CheckResolverException('identifier not found "i2"',nIdentifierNotFound);
end;

procedure TTestResolver.TestUnitUseDotted;
begin
  AddModuleWithIntfImplSrc('ns1.unit2.pp',
    LinesToStr([
    'var i2: longint;']),
    LinesToStr([
    '']));

  AddModuleWithIntfImplSrc('ns2.ns2A.unit1.pp',
    LinesToStr([
    'uses ns1.unit2;',
    'var j1: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  'uses ns2.ns2A.unit1;',
  'begin',
  '  if j1=0 then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestUnit_ProgramDefaultNamespace;
begin
  MainFilename:='ns1.main1.pas';

  AddModuleWithIntfImplSrc('ns1.unit2.pp',
    LinesToStr([
    'var i2: longint;']),
    LinesToStr([
    '']));

  AddModuleWithIntfImplSrc('ns1.unit1.pp',
    LinesToStr([
    'uses unit2;',
    'var j1: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  'uses unit1;',
  'begin',
  '  if j1=0 then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestUnit_DottedIdentifier;
begin
  MainFilename:='unitdots.main1.pas';

  AddModuleWithIntfImplSrc('unitdots.unit1.pp',
    LinesToStr([
    'type TColor = longint;',
    'var i1: longint;']),
    LinesToStr([
    '']));

  AddModuleWithIntfImplSrc('unitdots.pp',
    LinesToStr([
    'type TBright = longint;',
    'var j1: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  'uses unitdots.unit1, unitdots;',
  'type',
  '  TPrgBright = unitdots.tbright;',
  '  TPrgColor = unitdots.unit1.tcolor;',
  '  TStrange = unitdots.main1.tprgcolor;',
  'var k1: longint;',
  'begin',
  '  if unitdots.main1.k1=0 then ;',
  '  if unitdots.j1=0 then ;',
  '  if unitdots.unit1.i1=0 then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestUnit_DottedPrg;
begin
  MainFilename:='unitdots.main1.pas';

  AddModuleWithIntfImplSrc('unitdots.unit1.pp',
    LinesToStr([
    'type TColor = longint;',
    'var i1: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  'uses UnIt1;',
  'type',
  '  TPrgColor = UNIT1.tcolor;',
  '  TStrange = UnitDots.Main1.tprgcolor;',
  'var k1: longint;',
  'begin',
  '  if unitdots.main1.k1=0 then ;',
  '  if unit1.i1=0 then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestUnit_DottedUnit;
begin
  MainFilename:='unitdots.unit1.pas';
  StartUnit(false);
  Add([
  'interface',
  'var k1: longint;',
  'implementation',
  'initialization',
  '  if unitDots.Unit1.k1=0 then ;',
  '']);
  ParseUnit;
end;

procedure TTestResolver.TestUnit_DottedExpr;
begin
  MainFilename:='unitdots1.sub1.main1.pas';

  AddModuleWithIntfImplSrc('unitdots2.sub2.unit2.pp',
    LinesToStr([
    'procedure DoIt; external name ''$DoIt'';']),
    LinesToStr([
    '']));

  AddModuleWithIntfImplSrc('unitdots3.sub3.unit3.pp',
    LinesToStr([
    'procedure DoSome;']),
    LinesToStr([
    'uses unitdots2.sub2.unit2;',
    'procedure DoSome;',
    'begin',
    '  unitdots2.sub2.unit2.doit;',
    'end;']));

  StartProgram(true);
  Add([
  'uses unitdots3.sub3.unit3;',
  'begin',
  '  unitdots3.sub3.unit3.dosome;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestUnit_DuplicateDottedUsesFail;
begin
  AddModuleWithIntfImplSrc('ns.unit2.pp',
    LinesToStr([
    'var i: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add('uses ns.unit2, ns.unit2;');
  Add('begin');
  Add('  i:=3;');
  CheckParserException('Duplicate identifier "ns.unit2"',
    nParserDuplicateIdentifier);
end;

procedure TTestResolver.TestUnit_DuplicateUsesDiffNameFail;
begin
  MainFilename:='unitdots.main1.pas';
  AddModuleWithIntfImplSrc('unitdots.unit1.pp',
    LinesToStr([
    'var j1: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  'uses unitdots.unit1, unit1;',
  'var k1: longint;',
  'begin',
  '  if unitdots.main1.k1=0 then ;',
  '  if unit1.j1=0 then ;',
  '  if unitdots.unit1.j1=0 then ;',
  '']);
  CheckParserException('Duplicate identifier "unit1" at token ";" in file unitdots.main1.pas at line 2 column 27',
    nParserDuplicateIdentifier);
end;

procedure TTestResolver.TestUnit_Unit1DotUnit2Fail;
begin
  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'var i1: longint;']),
    LinesToStr([
    '']));

  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'uses unit1;',
    'var j1: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  'uses unit2;',
  'begin',
  '  if unit2.unit1.i1=0 then ;',
  '']);
  CheckResolverException('identifier not found "unit1"',
    nIdentifierNotFound);
end;

procedure TTestResolver.TestUnit_InFilename;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i1: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  'uses foo in ''unit2.pp'';',
  'begin',
  '  if foo.i1=0 then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestUnit_InFilenameAliasDelphiFail;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i1: longint;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  '{$mode delphi}',
  'uses foo in ''unit2.pp'';',
  'begin',
  '  if foo.i1=0 then ;',
  '']);
  CheckResolverException('foo expected, but unit2 found',nXExpectedButYFound);
end;

procedure TTestResolver.TestUnit_InFilenameInUnitDelphiFail;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i1: longint;']),
    LinesToStr([
    '']));

  StartUnit(true);
  Add([
  '{$mode delphi}',
  'interface',
  'uses unit2 in ''unit2.pp'';',
  'implementation',
  '']);
  CheckParserException('Expected ";"',nParserExpectTokenError);
end;

procedure TTestResolver.TestUnit_MissingUnitErrorPos;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var j1: longint;']),
    LinesToStr([
    '']));
  StartProgram(true);
  Add([
  'uses unit2, ;',
  'begin']);
  CheckParserException('Expected "Identifier" at token ";" in file afile.pp at line 2 column 13',
    nParserExpectTokenError);
end;

procedure TTestResolver.TestUnit_UnitNotFoundErrorPos;
begin
  StartProgram(true);
  Add([
  'uses foo   ;',
  'begin']);
  CheckResolverException('can''t find unit "foo" at afile.pp (2,6)',nCantFindUnitX);
end;

procedure TTestResolver.TestUnit_AccessIndirectUsedUnitFail;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'var i2: longint;']),
    LinesToStr([
    '']));

  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'uses unit2;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  'uses unit1;',
  'begin',
  '  if unit2.i2=0 then ;',
  '']);
  CheckResolverException('identifier not found "unit2"',nIdentifierNotFound);
end;

procedure TTestResolver.TestUnit_Intf1Impl2Intf1;
begin
  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'type number = longint;']),
    LinesToStr([
    'uses afile;',
    'procedure DoIt;',
    'begin',
    '  i:=3;',
    'end;']));

  StartUnit(true);
  Add([
  'interface',
  'uses unit1;',
  'var i: number;',
  'implementation']);
  ParseUnit;
end;

procedure TTestResolver.TestProcParam;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('procedure Proc1(a: integer);');
  Add('begin');
  Add('  a:=3;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestProcParamAccess;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('procedure DoIt(vI: integer; const vJ: integer; var vK: integer);');
  Add('var vL: integer;');
  Add('begin');
  Add('  vi:=vi+1;');
  Add('  vl:=vj+1;');
  Add('  vk:=vk+1;');
  Add('  vl:=vl+1;');
  Add('  DoIt(vi,vi,vi);');
  Add('  DoIt(vj,vj,vl);');
  Add('  DoIt(vk,vk,vk);');
  Add('  DoIt(vl,vl,vl);');
  Add('end;');
  Add('var i: integer;');
  Add('begin');
  Add('  DoIt(i,i,i);');
  Add('  DoIt(1,1,i);');
  ParseProgram;
end;

procedure TTestResolver.TestProcParamConstRef;
begin
  StartProgram(false);
  Add([
  'procedure Run(constref a: word);',
  'begin',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestFunctionResult;
begin
  StartProgram(false);
  Add('function Func1: longint;');
  Add('begin');
  Add('  Result:=3;');
  Add('  Func1:=4; ');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestProcedureResultFail;
begin
  StartProgram(false);
  Add('procedure A: longint; begin end;');
  Add('begin');
  CheckParserException('Expected ";"',
    nParserExpectTokenError);
end;

procedure TTestResolver.TestProc_ArgVarPrecisionLossFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TColor = type longint;',
  '  TByte = byte;',
  'procedure DoColor(var c: TColor); external;',
  'var',
  '  b: TByte;',
  'begin',
  '  DoColor(TColor(b));',
  '']);
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestProc_ArgVarTypeAliasObjFPC;
begin
  StartProgram(false);
  Add([
  'type',
  '  TColor = type longint;',
  'procedure DoColor(var c: TColor); external;',
  'procedure TakeColor(c: TColor); external;',
  'procedure DoInt(var i: longint); external;',
  'var',
  '  i: longint;',
  '  c: TColor;',
  'begin',
  '  DoColor(c);',
  '  DoColor(longint(c));',
  '  DoColor(i);',
  '  DoColor(TColor(i));',
  '  TakeColor(c);',
  '  TakeColor(longint(c));',
  '  TakeColor(i);',
  '  TakeColor(TColor(i));',
  '  DoInt(i);',
  '  DoInt(TColor(i));',
  '  DoInt(c);',
  '  DoInt(longint(c));',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProc_ArgVarTypeAliasDelphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TColor = type longint;',
  'procedure DoColor(var c: TColor); external;',
  'procedure TakeColor(c: TColor); external;',
  'procedure DoInt(var i: longint); external;',
  'var',
  '  i: longint;',
  '  c: TColor;',
  'begin',
  '  DoColor(c);',
  '  DoColor(TColor(i));',
  '  TakeColor(i);',
  '  TakeColor(longint(c));',
  '  DoInt(i);',
  '  DoInt(longint(c));',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProc_ArgVarTypeAliasDelphiMismatchFail;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TColor = type longint;',
  'procedure DoColor(var c: TColor); external;',
  'var',
  '  i: longint;',
  'begin',
  '  DoColor(i);',
  '']);
  CheckResolverException('Incompatible type arg no. 1: Got "Longint", expected "TColor". Var param must match exactly.',
    nIncompatibleTypeArgNoVarParamMustMatchExactly);
end;

procedure TTestResolver.TestProc_ArgMissingSemicolonFail;
begin
  StartProgram(false);
  Add([
  'type TScalar = double;',
  'procedure SinCos (var sinus: TScalar var cosinus: TScalar);',
  'begin end;',
  'begin']);
  CheckParserException('Expected ";" at token "var" in file afile.pp at line 3 column 38',nParserExpectTokenError);
end;

procedure TTestResolver.TestProcOverload;
var
  El: TPasElement;
begin
  StartProgram(false);
  Add('function Func1(i: longint; j: longint = 0): longint; overload;');
  Add('begin');
  Add('  Result:=1;');
  Add('end;');
  Add('function Func1(s: string): longint; overload;');
  Add('begin');
  Add('  Result:=2;');
  Add('end;');
  Add('begin');
  Add('  Func1(3);');
  ParseProgram;
  AssertEquals('2 declarations',2,PasProgram.ProgramSection.Declarations.Count);

  El:=TPasElement(PasProgram.ProgramSection.Declarations[0]);
  AssertEquals('is function',TPasFunction,El.ClassType);

  AssertEquals('1 statement',1,PasProgram.InitializationSection.Elements.Count);
end;

procedure TTestResolver.TestProcOverloadImplDuplicateFail;
begin
  StartUnit(false);
  Add([
  'interface',
  'procedure DoIt(d: double);',
  'implementation',
  'procedure DoIt(d: double); begin end;',
  'procedure DoIt(d: double); begin end;',
  'end.']);
  CheckResolverException('Duplicate identifier "DoIt" at afile.pp(5,15)',nDuplicateIdentifier);
end;

procedure TTestResolver.TestProcOverloadImplDuplicate2Fail;
begin
  StartUnit(false);
  Add([
  'interface',
  'implementation',
  'procedure DoIt(d: double); begin end;',
  'procedure DoIt(d: double); begin end;',
  'end.']);
  CheckResolverException('Duplicate identifier "DoIt" at afile.pp(4,15)',nDuplicateIdentifier);
end;

procedure TTestResolver.TestProcOverloadOtherUnit;
begin
  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'procedure DoIt(d: double);',
    '']),
    LinesToStr([
    'procedure DoIt(d: double); begin end;',
    '']));

  StartUnit(true);
  Add([
  'interface',
  'implementation',
  'procedure DoIt(d: double); begin end;',
  'end.']);
  ParseUnit;
end;

procedure TTestResolver.TestProcOverloadWithBaseTypes;
begin
  StartProgram(false);
  Add('function {#A}Func1(i: longint; j: longint = 0): longint; overload;');
  Add('begin');
  Add('  Result:=1;');
  Add('end;');
  Add('function {#B}Func1(s: string): longint; overload;');
  Add('begin');
  Add('  Result:=2;');
  Add('end;');
  Add('begin');
  Add('  {@A}Func1(3);');
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadWithBaseTypes2;
begin
  StartProgram(false);
  Add('procedure {#byte}DoIt(p: byte); external;  var by: byte;');
  Add('procedure {#shortint}DoIt(p: shortint); external;  var shi: shortint;');
  Add('procedure {#word}DoIt(p: word); external;  var w: word;');
  Add('procedure {#smallint}DoIt(p: smallint); external;  var smi: smallint;');
  Add('procedure {#longword}DoIt(p: longword); external;  var lw: longword;');
  Add('procedure {#longint}DoIt(p: longint); external;  var li: longint;');
  Add('procedure {#qword}DoIt(p: qword); external;  var qw: qword;');
  Add('procedure {#int64}DoIt(p: int64); external;  var i6: int64;');
  Add('procedure {#comp}DoIt(p: comp); external;  var co: comp;');
  Add('procedure {#boolean}DoIt(p: boolean); external;  var bo: boolean;');
  Add('procedure {#char}DoIt(p: char); external;  var ch: char;');
  Add('procedure {#widechar}DoIt(p: widechar); external;  var wc: widechar;');
  Add('procedure {#string}DoIt(p: string); external;  var st: string;');
  Add('procedure {#widestring}DoIt(p: widestring); external;  var ws: widestring;');
  Add('procedure {#shortstring}DoIt(p: shortstring); external;  var ss: shortstring;');
  Add('procedure {#unicodestring}DoIt(p: unicodestring); external;  var us: unicodestring;');
  Add('procedure {#rawbytestring}DoIt(p: rawbytestring); external;  var rs: rawbytestring;');
  Add('begin');
  Add('  {@byte}DoIt(by);');
  Add('  {@shortint}DoIt(shi);');
  Add('  {@word}DoIt(w);');
  Add('  {@smallint}DoIt(smi);');
  Add('  {@longword}DoIt(lw);');
  Add('  {@longint}DoIt(li);');
  Add('  {@qword}DoIt(qw);');
  Add('  {@int64}DoIt(i6);');
  Add('  {@comp}DoIt(co);');
  Add('  {@boolean}DoIt(bo);');
  Add('  {@char}DoIt(ch);');
  Add('  {@widechar}DoIt(wc);');
  Add('  {@string}DoIt(st);');
  Add('  {@widestring}DoIt(ws);');
  Add('  {@shortstring}DoIt(ss);');
  Add('  {@unicodestring}DoIt(us);');
  Add('  {@rawbytestring}DoIt(rs);');
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadWithDefaultArgs;
begin
  StartProgram(false);
  Add([
  'type float = type single;',
  'type integer = longint;',
  'procedure {#float}DoIt(s: float); external;',
  'procedure {#longint}DoIt(i: integer; Scale: float = 1.0); external;',
  'var i: integer;',
  'begin',
  '  {@float}DoIt(1.0);',
  '  {@longint}DoIt(2);',
  '  {@longint}DoIt(i);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadNearestHigherPrecision;
begin
  StartProgram(false);
  Add([
  'procedure {#longint}DoIt(i: longint); external;',
  'procedure DoIt(i: int64); external;',
  'var w: word;',
  'begin',
  '  {@longint}DoIt(w);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadForLoopIntDouble;
begin
  StartProgram(false);
  Add([
  'function {#int}Max(a,b: longint): longint; external; overload;',
  'function {#double}Max(a,b: double): double; external; overload;',
  'var',
  '  i: longint;',
  '  S: string;',
  'begin',
  '  for i:=0 to Max(length(s),1) do ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadStringArgCount;
begin
  StartProgram(false);
  Add([
  'function {#a}StrToDate(const a: String): double; begin end;',
  'function {#b}StrToDate(const a: String; const b: string): double; begin end;',
  'function {#c}StrToDate(const a: String; const b: string; c: char): double; begin end;',
  'var d: double;',
  'begin',
  '  d:={@a}StrToDate('''');',
  '  d:={@b}StrToDate('''','''');',
  '  d:={@c}StrToDate('''','''',''x'');',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcCallLowPrecision;
begin
  StartProgram(false);
  Add([
  'procedure {#longint}DoIt(i: longint); external;',
  'var i: int64;',
  'begin',
  '  {@longint}DoIt(i);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadUntyped;
begin
  StartProgram(false);
  Add([
  'procedure {#a}DoIt(a, b: longint); external;',
  'procedure {#b}DoIt(const a; b: longint); external;',
  'var',
  '  a: longint;',
  '  b: boolean;',
  'begin',
  '  {@a}DoIt(a,a);',
  '  {@b}DoIt(b,a);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadMultiLowPrecisionFail;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(i: longint); external;',
  'procedure DoIt(w: longword); external;',
  'var i: int64;',
  'begin',
  '  DoIt(i);',
  '']);
  CheckResolverException('Can''t determine which overloaded function to call, afile.pp(3,15), afile.pp(2,15)',
    nCantDetermineWhichOverloadedFunctionToCall);
end;

procedure TTestResolver.TestProcOverload_TypeAlias;
begin
  StartProgram(false);
  Add([
  'type',
  '  TValue = type longint;',
  '  TAliasValue = TValue;',
  '  TColor = type TAliasValue;',
  '  TAliasColor = TColor;',
  'procedure {#a}DoIt(i: TAliasValue); external;',
  'procedure {#b}DoIt(i: TAliasColor); external;',
  'procedure {#c}Fly(var i: TAliasValue); external;',
  'procedure {#d}Fly(var i: TAliasColor); external;',
  'var',
  '  v: TAliasValue;',
  '  c: TAliasColor;',
  'begin',
  '  {@a}DoIt(v);',
  '  {@a}DoIt(TAliasValue(c));',
  '  {@a}DoIt(TValue(c));',
  '  {@b}DoIt(c);',
  '  {@b}DoIt(TAliasColor(v));',
  '  {@b}DoIt(TColor(v));',
  '  {@c}Fly(v);',
  '  {@c}Fly(TAliasValue(c));',
  '  {@c}Fly(TValue(c));',
  '  {@d}Fly(c);',
  '  {@d}Fly(TAliasColor(v));',
  '  {@d}Fly(TColor(v));',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverload_TypeAliasLiteralFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = word;',
  '  TValue = type word;',
  '  TAliasValue = TValue;',
  'procedure DoIt(i: integer); external;',
  'procedure DoIt(i: TAliasValue); external;',
  'begin',
  '  DoIt(1);',
  '']);
  CheckResolverException('Can''t determine which overloaded function to call, afile.pp(7,15), afile.pp(6,15)',
    nCantDetermineWhichOverloadedFunctionToCall);
end;

procedure TTestResolver.TestProcOverloadWithClassTypes;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class end;');
  Add('  {#TA}TClassA = class end;');
  Add('  {#TB}TClassB = class end;');
  Add('procedure {#DoA}DoIt({=TA}p: TClassA); overload;');
  Add('begin');
  Add('end;');
  Add('procedure {#DoB}DoIt({=TB}p: TClassB); overload;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#A}{=TA}A: TClassA;');
  Add('  {#B}{=TB}B: TClassB;');
  Add('begin');
  Add('  {@DoA}DoIt({@A}A);');
  Add('  {@DoB}DoIt({@B}B);');
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadWithInhClassTypes;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class end;');
  Add('  {#TA}TClassA = class end;');
  Add('  {#TB}TClassB = class(TClassA) end;');
  Add('  {#TC}TClassC = class(TClassB) end;');
  Add('procedure {#DoA}DoIt({=TA}p: TClassA); overload;');
  Add('begin');
  Add('end;');
  Add('procedure {#DoB}DoIt({=TB}p: TClassB); overload;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#A}{=TA}A: TClassA;');
  Add('  {#B}{=TB}B: TClassB;');
  Add('  {#C}{=TC}C: TClassC;');
  Add('begin');
  Add('  {@DoA}DoIt({@A}A);');
  Add('  {@DoB}DoIt({@B}B);');
  Add('  {@DoB}DoIt({@C}C);');
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadWithInhAliasClassTypes;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TOBJ}TObject = class end;',
  '  {#TA}TClassA = class end;',
  '  {#TB}{=TA}TClassB = TClassA;',
  '  {#TC}TClassC = class(TClassB) end;',
  'procedure {#DoA}DoIt({=TA}p: TClassA); overload;',
  'begin',
  'end;',
  'procedure {#DoC}DoIt({=TC}p: TClassC); overload;',
  'begin',
  'end;',
  'var',
  '  {#A}{=TA}A: TClassA;',
  '  {#B}{=TB}B: TClassB;',
  '  {#C}{=TC}C: TClassC;',
  'begin',
  '  {@DoA}DoIt({@A}A);',
  '  {@DoA}DoIt({@B}B);',
  '  {@DoC}DoIt({@C}C);']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadWithInterfaces;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  {#IUnk}IUnknown = interface end;',
  '  {#IBird}IBird = interface(IUnknown) end;',
  '  {#TObj}TObject = class end;',
  '  {#TBird}TBird = class(IBird) end;',
  'procedure {#DoA}DoIt(o: TObject); overload; begin end;',
  'procedure {#DoB}DoIt(b: IBird); overload; begin end;',
  'var',
  '  o: TObject;',
  '  b: TBird;',
  '  i: IBird;',
  'begin',
  '  {@DoA}DoIt(o);',
  '  {@DoA}DoIt(b);',
  '  {@DoB}DoIt(i);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadBaseTypeOtherUnit;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'procedure Val(var d: double);',
    '']),
    LinesToStr([
    'procedure Val(var d: double); begin end;',
    'procedure Val(var i: integer); begin end;',
    '']));

  StartProgram(true);
  Add('uses unit2;');
  Add('var');
  Add('  d: double;');
  Add('  i: integer;');
  Add('begin');
  Add('  Val(d);');
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadBaseProcNoHint;
begin
  StartProgram(false);
  Add([
  'function Copy(s: string): string; overload;',
  'begin end;',
  'var',
  '  A: array of longint;',
  '  s: string;',
  'begin',
  '  A:=Copy(A,1);',
  '  s:=copy(s)']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestProcOverload_UnitOrderFail;
begin
  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'procedure Val(d: string);',
    '']),
    LinesToStr([
    'procedure Val(d: string); begin end;',
    '']));
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'procedure Val(d: double);',
    '']),
    LinesToStr([
    'procedure Val(d: double); begin end;',
    '']));

  StartProgram(true);
  Add([
  'uses unit1, unit2;',
  'var',
  '  s: string;',
  'begin',
  '  Val(s);']);
  CheckResolverException(sIncompatibleTypeArgNo,nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestProcOverload_UnitSameSignature;
begin
  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'procedure Val(d: string);',
    '']),
    LinesToStr([
    'procedure Val(d: string); begin end;',
    '']));
  StartProgram(true);
  Add([
  'uses unit1;',
  'procedure Val(d: string);',
  'begin',
  'end;',
  'var',
  '  s: string;',
  'begin',
  '  Val(s);']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadDelphiMissingNextOverload;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'procedure DoIt(i: longint); overload;',
  'begin end;',
  'procedure DoIt(s: string);',
  'begin end;',
  'begin']);
  CheckResolverException(sOverloadedProcMissesOverload,nOverloadedProcMissesOverload);
end;

procedure TTestResolver.TestProcOverloadDelphiMissingPrevOverload;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'procedure DoIt(i: longint); ',
  'begin end;',
  'procedure DoIt(s: string); overload;',
  'begin end;',
  'begin']);
  CheckResolverException(sPreviousDeclMissesOverload,nPreviousDeclMissesOverload);
end;

procedure TTestResolver.TestProcOverloadDelphiUnit;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    '{$mode delphi}',
    'procedure DoIt(s: string); overload;',
    'procedure DoIt(b: boolean); overload;',
    '']),
    LinesToStr([
    'procedure DoIt(s: string); begin end;',
    'procedure DoIt(b: boolean); begin end;',
    '']));

  StartProgram(true);
  Add([
  '{$mode delphi}',
  'uses unit2;',
  'procedure DoIt(i: longint); overload;',
  'begin end;',
  'begin',
  '  DoIt(3);',
  '  DoIt(true);',
  '  DoIt(''foo'');',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadDelphiUnitNoOverloadFail;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    '{$mode delphi}',
    'procedure DoIt(b: boolean);',
    '']),
    LinesToStr([
    'procedure DoIt(b: boolean); begin end;',
    '']));

  StartProgram(true);
  Add([
  '{$mode delphi}',
  'uses unit2;',
  'procedure DoIt(i: longint); overload;',
  'begin end;',
  'begin',
  '  DoIt(true);',
  '']);
  CheckResolverException(sIncompatibleTypeArgNo,nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestProcOverloadObjFPCUnitWithoutOverloadMod;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    '{$mode objfpc}',
    'procedure DoIt(s: string);',
    'procedure DoIt(b: boolean);',
    '']),
    LinesToStr([
    'procedure DoIt(s: string); begin end;',
    'procedure DoIt(b: boolean); begin end;',
    '']));

  StartProgram(true);
  Add([
  '{$mode objfpc}',
  'uses unit2;',
  'procedure DoIt(i: longint); overload;',
  'begin end;',
  'begin',
  '  DoIt(3);',
  '  DoIt(true);',
  '  DoIt(''foo'');',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadDelphiWithObjFPC;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    '{$mode objfpc}',
    'procedure DoIt(s: string);',
    'procedure DoIt(b: boolean);',
    '']),
    LinesToStr([
    'procedure DoIt(s: string); begin end;',
    'procedure DoIt(b: boolean); begin end;',
    '']));

  StartProgram(true);
  Add([
  '{$mode delphi}',
  'uses unit2;',
  'begin',
  '  DoIt(true);',
  '  DoIt(''foo'');',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcOverloadDelphiOverride;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class end;',
  '  TBird = class',
  '    function {#a}GetValue: longint; overload; virtual;',
  '    function {#b}GetValue(AValue: longint): longint; overload; virtual;',
  '  end;',
  '  TEagle = class(TBird)',
  '    function {#c}GetValue: longint; overload; override;',
  '    function {#d}GetValue(AValue: longint): longint; overload; override;',
  '  end;',
  '  TBear = class',
  '    procedure DoIt;',
  '  end;',
  'function TBird.GetValue: longint;',
  'begin',
  '  if 3={@a}GetValue then ;',
  '  if 4={@b}GetValue(5) then ;',
  'end;',
  'function TBird.GetValue(AValue: longint): longint;',
  'begin',
  'end;',
  'function TEagle.GetValue: longint;',
  'begin',
  '  if 13={@c}GetValue then ;',
  '  if 14={@d}GetValue(15) then ;',
  '  if 15=inherited {@a}GetValue then ;',
  '  if 16=inherited {@b}GetValue(17) then ;',
  'end;',
  'function TEagle.GetValue(AValue: longint): longint;',
  'begin',
  'end;',
  'procedure TBear.DoIt;',
  'var',
  '  e: TEagle;',
  'begin',
  '  if 23=e.{@c}GetValue then ;',
  '  if 24=e.{@d}GetValue(25) then ;',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestProcDuplicate;
begin
  StartProgram(false);
  Add('type integer = longint;');
  Add('procedure ProcA(i: longint);');
  Add('begin');
  Add('end;');
  Add('procedure ProcA(i: integer);');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckResolverException(sDuplicateIdentifier,nDuplicateIdentifier);
end;

procedure TTestResolver.TestNestedProc;
begin
  StartProgram(false);
  Add('function DoIt({#a1}a,{#d1}d: longint): longint;');
  Add('var');
  Add('  {#b1}b: longint;');
  Add('  {#c1}c: longint;');
  Add('  function {#Nesty1}Nesty({#a2}a: longint): longint; ');
  Add('  var {#b2}b: longint;');
  Add('  begin');
  Add('    Result:={@a2}a');
  Add('      +{@b2}b');
  Add('      +{@c1}c');
  Add('      +{@d1}d;');
  Add('    Nesty:=3;');
  Add('    DoIt:=4;');
  Add('  end;');
  Add('begin');
  Add('  Result:={@a1}a');
  Add('      +{@b1}b');
  Add('      +{@c1}c;');
  Add('  DoIt:=5;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestNestedProc_ResultString;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add([
  'function DoIt: string;',
  '  function Sub: char;',
  '  begin',
  '    {#a1}DoIt:=#65;',
  '    {#a2}DoIt[1]:=#66;',
  '    {#a3}DoIt;',
  '  end;',
  'begin',
  '  {#b1}DoIt:=#67;',
  '  {#b2}DoIt[2]:=#68;',
  '  {#b3}DoIt;',
  'end;',
  'begin']);
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestNestedProc_ResultString ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestNestedProc_ResultString ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        //writeln('TTestResolver.TestNestedProc_ResultString ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' Decl=',GetObjName(Ref.Declaration));
        case aMarker^.Identifier of
        'a1','a2','b1','b2':
          if not (Ref.Declaration is TPasResultElement) then
            RaiseErrorAtSrcMarker('expected FuncResult at "#'+aMarker^.Identifier+', but was "'+GetObjName(Ref.Declaration)+'"',aMarker);
        'a3','b3':
          if not (Ref.Declaration is TPasFunction) then
            RaiseErrorAtSrcMarker('expected TPasFunction at "#'+aMarker^.Identifier+', but was "'+GetObjName(Ref.Declaration)+'"',aMarker);
        end;
        end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestFuncAssignFail;
begin
  StartProgram(false);
  Add([
  'function DoIt: boolean;',
  'begin',
  'end;',
  'begin',
  '  DoIt:=true;']);
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestForwardProc;
begin
  StartProgram(false);
  Add('procedure {#A_forward}FuncA(i: longint); forward;');
  Add('procedure {#B}FuncB(i: longint);');
  Add('begin');
  Add('  {@A_forward}FuncA(i);');
  Add('end;');
  Add('procedure {#A}FuncA(i: longint);');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  {@A_forward}FuncA(3);');
  Add('  {@B}FuncB(3);');
  ParseProgram;
end;

procedure TTestResolver.TestForwardProcUnresolved;
begin
  StartProgram(false);
  Add('procedure FuncA(i: longint); forward;');
  Add('begin');
  CheckResolverException(sForwardProcNotResolved,nForwardProcNotResolved);
end;

procedure TTestResolver.TestNestedForwardProc;
begin
  StartProgram(false);
  Add('procedure {#A}FuncA;');
  Add('  procedure {#B_forward}ProcB(i: longint); forward;');
  Add('  procedure {#C}ProcC(i: longint);');
  Add('  begin');
  Add('    {@B_forward}ProcB(i);');
  Add('  end;');
  Add('  procedure {#B}ProcB(i: longint);');
  Add('  begin');
  Add('  end;');
  Add('begin');
  Add('  {@B_forward}ProcB(3);');
  Add('  {@C}ProcC(3);');
  Add('end;');
  Add('begin');
  Add('  {@A}FuncA;');
  ParseProgram;
end;

procedure TTestResolver.TestNestedForwardProcUnresolved;
begin
  StartProgram(false);
  Add('procedure FuncA;');
  Add('  procedure ProcB(i: longint); forward;');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckResolverException(sForwardProcNotResolved,nForwardProcNotResolved);
end;

procedure TTestResolver.TestForwardProcFuncMismatch;
begin
  StartProgram(false);
  Add('procedure DoIt; forward;');
  Add('function DoIt: longint;');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckResolverException('procedure expected, but function found',nXExpectedButYFound);
end;

procedure TTestResolver.TestForwardFuncResultMismatch;
begin
  StartProgram(false);
  Add('function DoIt: longint; forward;');
  Add('function DoIt: string;');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckResolverException('Result type mismatch, expected Longint, but found String',
    nResultTypeMismatchExpectedButFound);
end;

procedure TTestResolver.TestUnitIntfProc;
begin
  StartUnit(false);
  Add('interface');
  Add('procedure {#A_forward}FuncA({#Bar}Bar: longint);');
  Add('implementation');
  Add('procedure {#A}FuncA(bar: longint);');
  Add('begin');
  Add('  if {@Bar}bar=3 then ;');
  Add('end;');
  Add('initialization');
  Add('  {@A_forward}FuncA(3);');
  ParseUnit;
end;

procedure TTestResolver.TestUnitIntfProcUnresolved;
begin
  StartUnit(false);
  Add('interface');
  Add('procedure {#A_forward}FuncA(i: longint);');
  Add('implementation');
  Add('initialization');
  CheckResolverException(sForwardProcNotResolved,nForwardProcNotResolved);
end;

procedure TTestResolver.TestUnitIntfMismatchArgName;
begin
  StartUnit(false);
  Add('interface');
  Add('procedure {#A_forward}ProcA(i: longint);');
  Add('implementation');
  Add('procedure {#A}ProcA(j: longint);');
  Add('begin');
  Add('end;');
  CheckResolverException('function header "ProcA" doesn''t match forward : var name changes i => j',
    nFunctionHeaderMismatchForwardVarName);
end;

procedure TTestResolver.TestProcOverloadIsNotFunc;
begin
  StartUnit(false);
  Add('interface');
  Add('var ProcA: longint;');
  Add('procedure {#A_Decl}ProcA(i: longint);');
  Add('implementation');
  Add('procedure {#A_Impl}ProcA(i: longint);');
  Add('begin');
  Add('end;');
  CheckResolverException(sDuplicateIdentifier,nDuplicateIdentifier);
end;

procedure TTestResolver.TestProcCallMissingParams;
begin
  StartProgram(false);
  Add('procedure Proc1(a: longint);');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  Proc1;');
  CheckResolverException('Wrong number of parameters specified for call to "Proc1"',
    nWrongNumberOfParametersForCallTo);
end;

procedure TTestResolver.TestProcArgDefaultValue;
begin
  StartProgram(false);
  Add('const {#DefA}DefA = 3;');
  Add('procedure Proc1(a: longint = {@DefA}DefA);');
  Add('begin');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestProcArgDefaultValueTypeMismatch;
begin
  StartProgram(false);
  Add('procedure Proc1(a: string = 3);');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckResolverException('Incompatible types: got "Longint" expected "String"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestProcPassConstToVar;
begin
  StartProgram(false);
  Add('procedure DoSome(var i: longint); begin end;');
  Add('procedure DoIt(const i: longint);');
  Add('begin');
  Add('  DoSome(i);');
  Add('end;');
  Add('begin');
  CheckResolverException('Variable identifier expected',
    nVariableIdentifierExpected);
end;

procedure TTestResolver.TestBuiltInProcCallMissingParams;
begin
  StartProgram(false);
  Add('begin');
  Add('  length;');
  CheckResolverException('Wrong number of parameters specified for call to "function Length(const String or Array): sizeint"',
    nWrongNumberOfParametersForCallTo);
end;

procedure TTestResolver.TestAssignFunctionResult;
begin
  StartProgram(false);
  Add('function {#F1}F1: longint;');
  Add('begin');
  Add('end;');
  Add('function {#F2}F2: longint;');
  Add('begin');
  Add('end;');
  Add('var {#i}i: longint;');
  Add('begin');
  Add('  {@i}i:={@F1}F1();');
  Add('  {@i}i:={@F1}F1()+{@F2}F2();');
  Add('  {@i}i:={@F1}F1;');
  Add('  {@i}i:={@F1}F1+{@F2}F2;');
  ParseProgram;
end;

procedure TTestResolver.TestAssignProcResultFail;
begin
  StartProgram(false);
  Add('procedure {#P}P;');
  Add('begin');
  Add('end;');
  Add('var {#i}i: longint;');
  Add('begin');
  Add('  {@i}i:={@P}P();');
  CheckResolverException('Incompatible types: got "Procedure/Function" expected "Longint"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestFunctionResultInCondition;
begin
  StartProgram(false);
  Add('function {#F1}F1: longint;');
  Add('begin');
  Add('end;');
  Add('function {#F2}F2: boolean;');
  Add('begin');
  Add('end;');
  Add('var {#i}i: longint;');
  Add('begin');
  Add('  if {@F2}F2 then ;');
  Add('  if {@i}i={@F1}F1() then ;');
  ParseProgram;
end;

procedure TTestResolver.TestExit;
begin
  StartProgram(false);
  Add('procedure ProcA;');
  Add('begin');
  Add('  exit;');
  Add('end;');
  Add('function FuncB: longint;');
  Add('begin');
  Add('  exit;');
  Add('  exit(3);');
  Add('end;');
  Add('function FuncC: string;');
  Add('begin');
  Add('  exit;');
  Add('  exit(''a'');');
  Add('  exit(''abc'');');
  Add('end;');
  Add('begin');
  Add('  exit;');
  Add('  exit(4);');
  ParseProgram;
end;

procedure TTestResolver.TestBreak;
begin
  StartProgram(false);
  Add('var i: longint;');
  Add('begin');
  Add('  repeat');
  Add('    break;');
  Add('  until false;');
  Add('  while true do');
  Add('    break;');
  Add('  for i:=0 to 1 do');
  Add('    break;');
  ParseProgram;
end;

procedure TTestResolver.TestContinue;
begin
  StartProgram(false);
  Add('var i: longint;');
  Add('begin');
  Add('  repeat');
  Add('    continue;');
  Add('  until false;');
  Add('  while true do');
  Add('    continue;');
  Add('  for i:=0 to 1 do');
  Add('    continue;');
  ParseProgram;
end;

procedure TTestResolver.TestProcedureExternal;
begin
  StartProgram(false);
  Add('procedure {#ProcA}ProcA; external ''ExtProcA'';');
  Add('function {#FuncB}FuncB: longint; external ''ExtFuncB'';');
  Add('function {#FuncC}FuncC(d: double): string; external ''ExtFuncC'';');
  Add('var');
  Add('  i: longint;');
  Add('  s: string;');
  Add('begin');
  Add('  {@ProcA}ProcA;');
  Add('  i:={@FuncB}FuncB;');
  Add('  i:={@FuncB}FuncB();');
  Add('  s:={@FuncC}FuncC(1.2);');
  ParseProgram;
end;

procedure TTestResolver.TestProc_UntypedParam_Forward;
begin
  StartProgram(false);
  Add('procedure {#ProcA}ProcA(var {#A}A); forward;');
  Add('procedure {#ProcB}ProcB(const {#B}B); forward;');
  Add('procedure {#ProcC}ProcC(out {#C}C); forward;');
  //Add('procedure {#ProcD}ProcD(constref {#D}D); forward;');
  Add('procedure ProcA(var A);');
  Add('begin');
  Add('end;');
  Add('procedure ProcB(const B);');
  Add('begin');
  Add('end;');
  Add('procedure ProcC(out C);');
  Add('begin');
  Add('end;');
  //Add('procedure ProcD(constref D);');
  //Add('begin');
  //Add('end;');
  Add('var i: longint;');
  Add('begin');
  Add('  {@ProcA}ProcA(i);');
  Add('  {@ProcB}ProcB(i);');
  Add('  {@ProcC}ProcC(i);');
  //Add('  {@ProcD}ProcD(i);');
  ParseProgram;
end;

procedure TTestResolver.TestProc_Varargs;
begin
  StartProgram(false);
  Add('procedure ProcA(i:longint); varargs; external;');
  Add('procedure ProcB; varargs; external;');
  Add('procedure ProcC(i: longint = 17); varargs; external;');
  Add('begin');
  Add('  ProcA(1);');
  Add('  ProcA(1,2);');
  Add('  ProcA(1,2.0);');
  Add('  ProcA(1,2,3);');
  Add('  ProcA(1,''2'');');
  Add('  ProcA(2,'''');');
  Add('  ProcA(3,false);');
  Add('  ProcB;');
  Add('  ProcB();');
  Add('  ProcB(4);');
  Add('  ProcB(''foo'');');
  Add('  ProcC;');
  Add('  ProcC();');
  Add('  ProcC(4);');
  Add('  ProcC(5,''foo'');');
  ParseProgram;
end;

procedure TTestResolver.TestProc_VarargsOfT;
begin
  StartProgram(false);
  Add([
  'procedure ProcA(i:longint); varargs of word; external;',
  'procedure ProcB; varargs of boolean; external;',
  'procedure ProcC(i: longint = 17); varargs of double; external;',
  'begin',
  '  ProcA(1);',
  '  ProcA(2,3);',
  '  ProcA(4,5,6);',
  '  ProcB;',
  '  ProcB();',
  '  ProcB(false);',
  '  ProcB(true,false);',
  '  ProcC;',
  '  ProcC();',
  '  ProcC(7);',
  '  ProcC(8,9.3);',
  '  ProcC(8,9.3,1.3);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProc_VarargsOfTMismatch;
begin
  StartProgram(false);
  Add([
  'procedure ProcA(i:longint); varargs of word; external;',
  'begin',
  '  ProcA(1,false);',
  '']);
  CheckResolverException('Incompatible type arg no. 2: Got "Boolean", expected "Word"',nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestProc_ParameterExprAccess;
begin
  StartProgram(false);
  Add('type');
  Add('  TRec = record');
  Add('    a: longint;');
  Add('  end;');
  Add('procedure DoIt(i: longint; const j: longint; var k: longint; out l: longint);');
  Add('begin');
  Add('  DoIt({#loc1_read}i,{#loc2_read}i,{#loc3_var}i,{#loc4_out}i);');
  Add('end;');
  Add('var');
  Add('  r: TRec;');
  Add('begin');
  Add('  DoIt({#r1_read}r.{#r_a1_read}a,');
  Add('    {#r2_read}r.{#r_a2_read}a,');
  Add('    {#r3_read}r.{#r_a3_var}a,');
  Add('    {#r4_read}r.{#r_a4_out}a);');
  Add('  with r do');
  Add('    DoIt({#w_a1_read}a,');
  Add('      {#w_a2_read}a,');
  Add('      {#w_a3_var}a,');
  Add('      {#w_a4_out}a);');
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestProc_FunctionResult_DeclProc;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
  ResultEl: TPasResultElement;
  Proc: TPasProcedure;
  ProcScope: TPasProcedureScope;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    function MethodA: longint;');
  Add('  end;');
  Add('function FuncA: longint; forward;');
  Add('function TObject.MethodA: longint;');
  Add('begin');
  Add('  {#MethodA_Result}Result:=1;');
  Add('end;');
  Add('function FuncA: longint;');
  Add('  function SubFuncA: longint; forward;');
  Add('  function SubFuncB: longint;');
  Add('  begin');
  Add('    {#SubFuncB_Result}Result:=2;');
  Add('  end;');
  Add('  function SubFuncA: longint;');
  Add('  begin');
  Add('    {#SubFuncA_Result}Result:=3;');
  Add('  end;');
  Add('begin');
  Add('  {#FuncA_Result}Result:=4;');
  Add('end;');
  Add('begin');
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestProc_FunctionResult_DeclProc ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestProc_FunctionResult_DeclProc ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        //writeln('TTestResolver.TestProc_FunctionResult_DeclProc ',GetObjName(Ref.Declaration));
        if not (Ref.Declaration is TPasResultElement) then continue;
        ResultEl:=TPasResultElement(Ref.Declaration);
        Proc:=ResultEl.Parent.Parent as TPasProcedure;
        ProcScope:=Proc.CustomData as TPasProcedureScope;
        if ProcScope.DeclarationProc<>nil then
          RaiseErrorAtSrcMarker('expected Result to resolve to declaration at "#'+aMarker^.Identifier+', but was implproc"',aMarker);
        break;
        end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestProc_TypeCastFunctionResult;
begin
  StartProgram(false);
  Add('function GetIt: longint; begin end;');
  Add('var s: smallint;');
  Add('begin');
  Add('  s:=smallint(GetIt);');
  ParseProgram;
end;

procedure TTestResolver.TestProc_ImplicitCalls;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualImplicitCallWithoutParams: Boolean;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add([
  'function b: longint;',
  'begin',
  'end;',
  'function GetStr: string;',
  'begin',
  'end;',
  'var',
  '  a: longint;',
  '  s: string;',
  '  arr: array of longint;',
  'begin',
  '  Inc(a,{#b1}b);',
  '  Dec(a,{#b2}b);',
  '  str({#b3}b,s);',
  '  SetLength(arr,{#b4}b);',
  '  Insert({#b5}b,arr,{#b6}b);',
  '  Delete(arr,{#b7}b,{#b8}b);',
  '  a:=length({#b9}GetStr);',
  '']);
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestProc_IncWithImplicitCall ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualImplicitCallWithoutParams:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestProc_IncWithImplicitCall ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if not (Ref.Declaration is TPasProcedure) then continue;
        //writeln('TTestResolver.TestProc_IncWithImplicitCall ',GetObjName(Ref.Declaration),' rrfNewInstance=',rrfNewInstance in Ref.Flags);
        ActualImplicitCallWithoutParams:=rrfImplicitCallWithoutParams in Ref.Flags;
        break;
        end;
      if not ActualImplicitCallWithoutParams then
        RaiseErrorAtSrcMarker('expected implicit call at "#'+aMarker^.Identifier+', but got function ref"',aMarker);
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestProc_Absolute;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(p: Pointer);',
  'var',
  '  s: string absolute p;',
  '  t: array of char absolute s;',
  'begin',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestProc_LocalInit;
begin
  StartProgram(false);
  Add([
  'type TBytes = array of byte;',
  'procedure DoIt;',
  'const c = 4;',
  'var',
  '  w: word = c;',
  '  b: byte = 1+c;',
  '  p: pointer = nil;',
  '  buf: TBytes = nil;',
  'begin',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestProc_ExtNamePropertyFail;
begin
  StartProgram(false);
  Add([
  'procedure Foo; external name ''});'' property;',
  'begin']);
  CheckParserException('Expected ";" at token "property" in file afile.pp at line 2 column 36',
    nParserExpectTokenError);
end;

procedure TTestResolver.TestAnonymousProc_Assign;
begin
  StartProgram(false);
  Add([
  'type',
  '  TFunc = reference to function(x: word): word;',
  'var Func: TFunc;',
  'procedure DoIt(a: word);',
  'begin',
  '  Func:=function(b:word): word',
  '  begin',
  '    Result:=a+b;',
  '    exit(b);',
  '    exit(Result);',
  '  end;',// test semicolon
  '  a:=3;',
  'end;',
  'begin',
  '  Func:=function(c:word):word begin',
  '    Result:=3+c;',
  '    exit(c);',
  '    exit(Result);',
  '  end;']);
  ParseProgram;
end;

procedure TTestResolver.TestAnonymousProc_AssignSemicolonFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  'procedure DoIt(a: word);',
  'var p: TProc;',
  'begin',
  '  p:=procedure; begin end;',
  '  a:=3;',
  'end;',
  'begin']);
  CheckParserException('Expected "begin" at token ";" in file afile.pp at line 7 column 15',
    nParserExpectTokenError);
end;

procedure TTestResolver.TestAnonymousProc_Assign_ReferenceToMissingFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = procedure;',
  'procedure DoIt;',
  'var p: TProc;',
  'begin',
  '  p:=procedure(w: word) begin end;',
  'end;',
  'begin']);
  CheckResolverException('procedural type modifier "reference to" mismatch',
    nXModifierMismatchY);
end;

procedure TTestResolver.TestAnonymousProc_Assign_WrongParamListFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  'procedure DoIt;',
  'var p: TProc;',
  'begin',
  '  p:=procedure(w: word) begin end;',
  'end;',
  'begin']);
  CheckResolverException('Incompatible types, got 0 parameters, expected 1',
    nIncompatibleTypesGotParametersExpected);
end;

procedure TTestResolver.TestAnonymousProc_Arg;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  '  TFunc = reference to function(x: word): word;',
  'procedure DoMore(f,g: TProc);',
  'begin',
  'end;',
  'procedure DoIt(f: TFunc);',
  'begin',
  '  DoIt(function(b:word): word',
  '    begin',
  '      Result:=1+b;',
  '    end);',
  '  DoMore(procedure begin end, procedure begin end);',
  'end;',
  'begin',
  '  DoMore(procedure begin end, procedure begin end);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestAnonymousProc_ArgSemicolonFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  'procedure DoIt(p: TProc);',
  'begin',
  'end;',
  'begin',
  '  DoIt(procedure begin end;);']);
  CheckParserException('Expected "," at token ";" in file afile.pp at line 8 column 27',
    nParserExpectTokenError);
end;

procedure TTestResolver.TestAnonymousProc_EqualFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TFunc = reference to function(x: word): word;',
  'procedure DoIt(f: TFunc);',
  'var w: word;',
  'begin',
  '  if w=function(b:word): word',
  '    begin',
  '      Result:=1+b;',
  '    end then ;',
  'end;',
  'begin']);
  CheckResolverException('Incompatible types: got "Procedure/Function" expected "Word"',nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestAnonymousProc_ConstFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  'const',
  '  p: TProc = procedure begin end;',
  'begin']);
  CheckParserException('Identifier expected at token "procedure" in file afile.pp at line 5 column 14',nParserExpectedIdentifier);
end;

procedure TTestResolver.TestAnonymousProc_Assembler;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  '  TProcB = reference to procedure cdecl;',
  'procedure DoIt(p: TProc);',
  'var b: TProcB;',
  'begin',
  '  p:=procedure assembler asm end;',
  '  p:=procedure() assembler asm end;',
  '  b:=procedure() cdecl assembler asm end;',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestAnonymousProc_NameFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  'procedure DoIt(p: TProc);',
  'begin',
  '  p:=procedure Bla() begin end;',
  'end;',
  'begin']);
  CheckParserException(SParserSyntaxError,nParserSyntaxError);
end;

procedure TTestResolver.TestAnonymousProc_StatementFail;
begin
  StartProgram(false);
  Add([
  'procedure DoIt;',
  'begin',
  '  procedure () begin end;',
  'end;',
  'begin']);
  CheckParserException(SParserSyntaxError,nParserSyntaxError);
end;

procedure TTestResolver.TestAnonymousProc_Typecast_ObjFPC;
begin
  StartProgram(false);
  Add([
  '{$mode ObjFPC}',
  'type',
  '  TProc = reference to procedure(w: word);',
  '  TArr = array of word;',
  '  TFuncArr = reference to function: TArr;',
  'procedure DoIt(p: TProc);',
  'var',
  '  w: word;',
  '  a: TArr;',
  'begin',
  '  p:=TProc(procedure(b: smallint) begin end);',
  '  a:=TFuncArr(function: TArr begin end)();',
  '  w:=TFuncArr(function: TArr begin end)()[3];',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestAnonymousProc_Typecast_Delphi;
begin
  StartProgram(false);
  Add([
  '{$mode Delphi}',
  'type',
  '  TProc = reference to procedure(w: word);',
  '  TArr = array of word;',
  '  TFuncArr = reference to function: TArr;',
  'procedure DoIt(p: TProc);',
  'var',
  '  w: word;',
  '  a: TArr;',
  'begin',
  '  p:=TProc(procedure(b: smallint) begin end);',
  '  a:=TFuncArr(function: TArr begin end)();',
  '  w:=TFuncArr(function: TArr begin end)()[3];',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestAnonymousProc_TypecastToResultFail;
begin
  StartProgram(false);
  Add([
  'procedure DoIt;',
  'var i: longint;',
  'begin',
  '  i:=longint(function(b: byte): byte begin end);',
  'end;',
  'begin']);
  CheckResolverException('Illegal type conversion: "Procedure/Function" to "Longint"',
    nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestAnonymousProc_With;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure(w: word);',
  '  TObject = class end;',
  '  TBird = class',
  '    {#bool}b: boolean;',
  '  end;',
  'procedure DoIt({#i}i: longint);',
  'var',
  '  {#p}p: TProc;',
  '  {#bird}bird: TBird;',
  'begin',
  '  with {@bird}bird do',
  '    {@p}p:=procedure({#w}w: word)',
  '      begin',
  '        {@bool}b:=true;',
  '        {@bool}b:=({@w}w+{@i}i)>2;',
  '      end;',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestAnonymousProc_ExceptOn;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  '  TObject = class end;',
  '  Exception = class',
  '    {#bool}b: boolean;',
  '  end;',
  'procedure DoIt;',
  'var',
  '  {#p}p: TProc;',
  'begin',
  '  try',
  '  except',
  '    on {#E}E: Exception do',
  '    {@p}p:=procedure',
  '      begin',
  '        {@E}E.{@bool}b:=true;',
  '      end;',
  '  end;',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestAnonymousProc_Nested;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  '  TObject = class',
  '    i: byte;',
  '    procedure DoIt;',
  '  end;',
  'procedure TObject.DoIt;',
  'var',
  '  p: TProc;',
  '  procedure Sub;',
  '  begin',
  '    p:=procedure',
  '      begin',
  '        i:=3;',
  '        Self.i:=4;',
  '        p:=procedure',
  '            procedure SubSub;',
  '            begin',
  '              i:=13;',
  '              Self.i:=14;',
  '            end;',
  '          begin',
  '            i:=13;',
  '            Self.i:=14;',
  '          end;',
  '      end;',
  '  end;',
  'begin',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestAnonymousProc_ForLoop;
begin
  StartProgram(false);
  Add([
  'type TProc = reference to procedure;',
  'procedure Foo(p: TProc);',
  'begin',
  'end;',
  'procedure DoIt;',
  'var i: word;',
  '  a: word;',
  'begin',
  '  for i:=1 to 10 do begin',
  '    Foo(procedure begin a:=3; end);',
  '  end;',
  'end;',
  'begin',
  '  DoIt;']);
  ParseProgram;
end;

procedure TTestResolver.TestRecord;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TRec}TRec = record');
  Add('    {#Size}Size: longint;');
  Add('  end;');
  Add('var');
  Add('  {#r}{=TRec}r: TRec;');
  Add('begin');
  Add('  {@r}r.{@Size}Size:=3;');
  ParseProgram;
end;

procedure TTestResolver.TestRecordVariant;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TRec}TRec = record');
  Add('    {#Size}Size: longint;');
  Add('    case {#vari}vari: longint of');
  Add('    0: ({#b}b: longint)');
  Add('  end;');
  Add('var');
  Add('  {#r}{=TRec}r: TRec;');
  Add('begin');
  Add('  {@r}r.{@Size}Size:=3;');
  Add('  {@r}r.{@vari}vari:=4;');
  Add('  {@r}r.{@b}b:=5;');
  ParseProgram;
end;

procedure TTestResolver.TestRecordVariantNested;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TRec}TRec = record',
  '    {#Size}Size: longint;',
  '    case {#vari}vari: longint of',
  '    0: ({#b}b: longint)',
  '    1: ({#c}c:',
  '          record',
  '            {#d}d: longint;',
  '            case {#e}e: longint of',
  '            0: ({#f}f: longint)',
  '          end)',
  '  end;',
  'var',
  '  {#r}{=TRec}r: TRec;',
  'begin',
  '  {@r}r.{@Size}Size:=3;',
  '  {@r}r.{@vari}vari:=4;',
  '  {@r}r.{@b}b:=5;',
  '  {@r}r.{@c}c.{@d}d:=6;',
  '  {@r}r.{@c}c.{@e}e:=7;',
  '  {@r}r.{@c}c.{@f}f:=8;']);
  ParseProgram;
end;

procedure TTestResolver.TestRecord_WriteConstParamFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TSmall = record');
  Add('    Size: longint;');
  Add('  end;');
  Add('procedure DoIt(const S: TSmall);');
  Add('begin');
  Add('  S.Size:=3;');
  Add('end;');
  Add('begin');
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestRecord_WriteConstParam_WithFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TSmall = record');
  Add('    Size: longint;');
  Add('  end;');
  Add('procedure DoIt(const S: TSmall);');
  Add('begin');
  Add('  with S do Size:=3;');
  Add('end;');
  Add('begin');
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestRecord_WriteNestedConstParamFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TSmall = record');
  Add('    Size: longint;');
  Add('  end;');
  Add('  TBig = record');
  Add('    Small: TSmall;');
  Add('  end;');
  Add('procedure DoIt(const B: TBig);');
  Add('begin');
  Add('  B.Small.Size:=3;');
  Add('end;');
  Add('begin');
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestRecord_WriteNestedConstParamWithFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TSmall = record');
  Add('    Size: longint;');
  Add('  end;');
  Add('  TBig = record');
  Add('    Small: TSmall;');
  Add('  end;');
  Add('procedure DoIt(const B: TBig);');
  Add('begin');
  Add('  with B do with Small do Size:=3;');
  Add('end;');
  Add('begin');
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestRecord_TypeCast;
begin
  StartProgram(false);
  Add([
  'type',
  '  TAnimal = record',
  '    Size: longint;',
  '  end;',
  '  TBird = record',
  '    Length: longint;',
  '  end;',
  'var',
  '  a: TAnimal;',
  '  b: TBird;',
  'begin',
  '  b:=TBird(a);',
  '  TAnimal(b).Size:=TBird(a).Length;',
  '  ']);
  ParseProgram;
end;

procedure TTestResolver.TestRecord_NewDispose;
begin
  StartProgram(false);
  Add([
  'type',
  '  TBird = record',
  '    Length: longint;',
  '  end;',
  '  PBird = ^TBird;',
  'var',
  '  p: PBird;',
  '  q: ^TBird;',
  'begin',
  '  New(p);',
  '  Dispose(p);',
  '  New(q);',
  '  Dispose(q);',
  '  ']);
  ParseProgram;
end;

procedure TTestResolver.TestRecord_Const;
begin
  StartProgram(false);
  Add([
  'type',
  '  TPoint = record x, y: longint; end;',
  'const r: TPoint = (x:1; y:2);',
  'type',
  '  TPasSourcePos = Record',
  '    FileName: String;',
  '    Row, Column: LongWord;',
  '  end;',
  'const',
  '  DefPasSourcePos: TPasSourcePos = (Filename:''''; Row:0; Column:0);',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestRecord_Const_DuplicateFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TPoint = record x, y: longint; end;',
  'const r: TPoint = (x:1; x:2);',
  'begin',
  '']);
  CheckResolverException('Duplicate identifier "x" at afile.pp(4,20)',nDuplicateIdentifier);
end;

procedure TTestResolver.TestRecord_Const_ExprMismatchFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TPoint = record x, y: longint; end;',
  'const r: TPoint = (x:1; x:2);',
  'begin',
  '']);
  CheckResolverException('Duplicate identifier "x" at afile.pp(4,20)',nDuplicateIdentifier);
end;

procedure TTestResolver.TestRecord_Const_MissingHint;
begin
  StartProgram(false);
  Add([
  'type',
  '  TPoint = record x, y: longint; end;',
  'const r: TPoint = (x:1);',
  'begin',
  '']);
  ParseProgram;
  CheckResolverHint(mtHint,nMissingFieldsX,'Missing fields: "y"');
end;

procedure TTestResolver.TestRecord_Const_UntypedFail;
begin
  StartProgram(false);
  Add([
  'const r = (x:1);',
  'begin',
  '']);
  CheckResolverException('Syntax error, "const" expected but "record values" found',nSyntaxErrorExpectedButFound);
end;

procedure TTestResolver.TestRecord_Const_NestedRecord;
begin
  StartProgram(false);
  Add([
  'type',
  '  TPoint = record x, y: longint; end;',
  '  TSrc = record',
  '    Id: longint;',
  '    XY: TPoint',
  '  end;',
  'const r: TSrc = (Id:1; XY: (x:2; y:3));',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestRecord_Const_Variant;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TRec}TRec = record',
  '    {#Size}Size: longint;',
  '    case {#vari}vari: longint of',
  '    0: ({#b}b: longint);',
  '    1: ({#c}c:',
  '          record',
  '            {#d}d: longint;',
  '            case {#e}e: longint of',
  '            0: ({#f}f: longint)',
  '          end)',
  '  end;',
  'const',
  '  {#r}r: TRec = (',
  '    {@Size}Size:2;',
  '    {@c}c:(',
  '      {@d}d:3;',
  '      {@f}f:4',
  '    )',
  '  );',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestRecord_Default;
begin
  StartProgram(false);
  Add([
  'type',
  '  TPoint = record x, y: longint; end;',
  'var',
  '  i: longint;',
  '  r: TPoint;',
  'begin',
  '  i:=Default(longint);',
  '  r:=Default(r);',
  '  r:=Default(TPoint);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestRecord_VarExternal;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TRec = record',
  '    Id: longint external name ''$Id'';',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestRecord_VarSelfFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TRec = record',
  '    r: Trec;',
  '  end;',
  'begin']);
  CheckResolverException('type "TRec" is not yet completely defined',nTypeXIsNotYetCompletelyDefined);
end;

procedure TTestResolver.TestAdvRecord;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    procedure DoIt;',
  '  end;',
  'procedure TRec.DoIt;',
  'begin',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_Private;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '  private',
  '    a: byte;',
  '  public',
  '    b: byte;',
  '  end;',
  'var',
  '  r: TRec;',
  'begin',
  '  r.a:=r.b;']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_StrictPrivate;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '  strict private',
  '    FSize: longword;',
  '    function GetSize: longword;',
  '  public',
  '    property Size: longword read GetSize write FSize;',
  '  end;',
  'function TRec.GetSize: longword;',
  'begin',
  '  FSize:=GetSize;',
  'end;',
  'var',
  '  r: TRec;',
  'begin',
  '  r.Size:=r.Size;']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_StrictPrivateFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '  strict private',
  '    A: word;',
  '  end;',
  'var',
  '  r: TRec;',
  'begin',
  '  r.a:=r.a;']);
  CheckResolverException('Can''t access strict private member A',nCantAccessXMember);
end;

procedure TTestResolver.TestAdvRecord_MethodImplMissingFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    procedure SetSize(Value: word);',
  '  end;',
  'begin',
  '']);
  CheckResolverException(sForwardProcNotResolved,nForwardProcNotResolved);
end;

procedure TTestResolver.TestAdvRecord_VarConst;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '  type TInt = word;',
  '  const',
  '    C1 = 3;',
  '    C2: TInt = 4;',
  '  var',
  '    V1: TInt;',
  '    V2: TInt;',
  '  class var',
  '    VC: TInt;',
  '    CA: array[1..C1] of TInt;',
  '  procedure DoIt;',
  '  end;',
  'procedure TRec.DoIt;',
  'begin',
  '  C2:=Self.C2;',
  '  V1:=VC;',
  '  Self.V1:=Self.VC;',
  '  VC:=V1;',
  '  Self.VC:=Self.V1;',
  'end;',
  'var',
  '  r: TRec;',
  'begin',
  '  trec.C2:=trec.C2;',
  '  r.V1:=r.VC;',
  '  r.V1:=trec.VC;',
  '  r.VC:=r.V1;',
  '  trec.VC:=trec.c1;',
  '  trec.ca[1]:=trec.c2;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_RecVal_ConstFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    V1: word;',
  '  const',
  '    C1 = 3;',
  '  end;',
  'var',
  '  r: TRec = (V1:2; C1: 4);',
  'begin',
  '']);
  CheckResolverException(sIdentifierXIsNotAnInstanceField,nIdentifierXIsNotAnInstanceField);
end;

procedure TTestResolver.TestAdvRecord_RecVal_ClassVarFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    V1: word;',
  '  class var',
  '    C1: word;',
  '  end;',
  'var',
  '  r: TRec = (V1:2; C1: 4);',
  'begin',
  '']);
  CheckResolverException(sIdentifierXIsNotAnInstanceField,nIdentifierXIsNotAnInstanceField);
end;

procedure TTestResolver.TestAdvRecord_LocalForwardType;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '  type',
  '    PInt = ^TInt;',
  '    TInt = word;',
  '  var i: PInt;',
  '  end;',
  'var',
  '  r: TRec;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_Constructor_NewInstance;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualNewInstance: Boolean;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    constructor Create(w: word);',
  '    class function DoSome: TRec; static;',
  '  end;',
  'constructor TRec.Create(w: word);',
  'begin',
  '  {#a}Create(1); // normal call',
  '  TRec.{#b}Create(2); // new instance',
  'end;',
  'class function TRec.DoSome: TRec;',
  'begin',
  '  Result:={#c}Create(3); // new instance',
  'end;',
  'var',
  '  r: TRec;',
  'begin',
  '  TRec.{#p}Create(4); // new object',
  '  r:=TRec.{#q}Create(5); // new object',
  '  with TRec do begin',
  '    {#r}Create(6); // new object',
  '    r:={#s}Create(7); // new object',
  '  end;',
  '  r.{#t}Create(8); // normal call',
  '  r:=r.{#u}Create(9); // normal call',
  '  with r do begin',
  '    {#v}Create(10); // normal call',
  '    r:={#w}Create(11); // normal call',
  '  end;',
  '']);
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestAdvRecord_Constructor_NewInstance ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualNewInstance:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestAdvRecord_Constructor_NewInstance ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if not (Ref.Declaration is TPasProcedure) then continue;
        //writeln('TTestResolver.TestAdvRecord_Constructor_NewInstance ',GetObjName(Ref.Declaration),' rrfNewInstance=',rrfNewInstance in Ref.Flags);
        if (Ref.Declaration is TPasConstructor) then
          ActualNewInstance:=rrfNewInstance in Ref.Flags;
        if rrfImplicitCallWithoutParams in Ref.Flags then
          RaiseErrorAtSrcMarker('unexpected implicit call at "#'+aMarker^.Identifier+' ref"',aMarker);
        break;
        end;
      case aMarker^.Identifier of
      'a','t','u','v','w':// should be normal call
        if ActualNewInstance then
          RaiseErrorAtSrcMarker('expected normal call at "#'+aMarker^.Identifier+', but got newinstance"',aMarker);
      else // should be newinstance
        if not ActualNewInstance then
          RaiseErrorAtSrcMarker('expected newinstance at "#'+aMarker^.Identifier+', but got normal call"',aMarker);
      end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestAdvRecord_ConstructorNoParamsFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    constructor Create(w: word = 3);',
  '  end;',
  'constructor TRec.Create(w: word);',
  'begin',
  'end;',
  'begin',
  '']);
  CheckResolverException(sParameterlessConstructorsNotAllowedInRecords,
    nParameterlessConstructorsNotAllowedInRecords);
end;

procedure TTestResolver.TestAdvRecord_ClassConstructor;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    class var w: word;',
  '    class procedure {#a}Create; static;',
  '    class constructor Create;', // name clash is allowed!
  '  end;',
  'class constructor TRec.Create;',
  'begin',
  '  w:=w+1;',
  'end;',
  'class procedure TRec.Create;',
  'begin',
  '  w:=w+1;',
  'end;',
  'begin',
  '  TRec.{@a}Create;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_ClassConstructorParamsFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    class constructor Create(w: word);',
  '  end;',
  'class constructor TRec.Create(w: word);',
  'begin',
  'end;',
  'begin',
  '']);
  CheckResolverException('class constructor cannot have parameters',nXCannotHaveParameters);
end;

procedure TTestResolver.TestAdvRecord_ClassConstructor_CallFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    class constructor Create;',
  '  end;',
  'class constructor TRec.Create;',
  'begin',
  'end;',
  'begin',
  '  TRec.Create;',
  '']);
  CheckResolverException('identifier not found "Create"',nIdentifierNotFound);
end;

procedure TTestResolver.TestAdvRecord_ClassConstructorDuplicateFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    class constructor Create;',
  '    class constructor Init;',
  '  end;',
  'class constructor TRec.Create;',
  'begin',
  'end;',
  'class constructor TRec.Init;',
  'begin',
  'end;',
  'begin',
  '']);
  CheckResolverException('Multiple class constructor in record TRec: Create and Init',
    nMultipleXinTypeYNameZCAandB);
end;

procedure TTestResolver.TestAdvRecord_NestedRecordType;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '  type',
  '    TSub = record',
  '      x: word;',
  '      class var y: word;',
  '      procedure DoSub;',
  '    end;',
  '  var',
  '    Sub: TSub;',
  '    procedure DoIt(const r: TRec);',
  '  end;',
  'procedure TRec.TSub.DoSub;',
  'begin',
  '  x:=3;',
  'end;',
  'procedure TRec.DoIt(const r: TRec);',
  'begin',
  '  Sub.x:=4;',
  '  r.Sub.y:=Sub.x;', // class var y is writable, even though r.Sub is not
  'end;',
  'var r: TRec;',
  'begin',
  '  r.sub.x:=4;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_NestedArgConstFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '  type',
  '    TSub = record',
  '      x: word;',
  '    end;',
  '  var',
  '    Sub: TSub;',
  '    procedure DoIt(const r: TRec);',
  '  end;',
  'procedure TRec.DoIt(const r: TRec);',
  'begin',
  '  r.Sub.x:=4;',
  'end;',
  'begin',
  '']);
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestAdvRecord_Property;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '  private',
  '    FSize: word;',
  '    function SizeStored: boolean;',
  '    function GetWidth: word;',
  '    procedure SetWidth(Value: word);',
  '  public',
  '    property Size: word read FSize write FSize stored SizeStored default 3;',
  '    property Width: word read GetWidth write SetWidth;',
  '  end;',
  'function TRec.SizeStored: boolean;',
  'begin',
  'end;',
  'function TRec.GetWidth: word;',
  'begin',
  '  Result:=FSize;',
  'end;',
  'procedure TRec.SetWidth(Value: word);',
  'begin',
  '  FSize:=Value;',
  'end;',
  'var r: TRec;',
  'begin',
  '  r.Size:=r.Size;',
  '  r.Width:=r.Width;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_ClassProperty;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '  private',
  '    class var FSize: word;',
  '    class function GetWidth: word; static;',
  '    class procedure SetWidth(Value: word); static;',
  '  public',
  '    class property Size: word read FSize write FSize;',
  '    class property Width: word read GetWidth write SetWidth;',
  '  end;',
  'class function TRec.GetWidth: word;',
  'begin',
  '  Result:=FSize;',
  'end;',
  'class procedure TRec.SetWidth(Value: word);',
  'begin',
  '  FSize:=Value;',
  'end;',
  'begin',
  '  TRec.Size:=TRec.Size;',
  '  TRec.Width:=TRec.Width;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_PropertyDefault;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '  private',
  '    function GetItems(Index: word): word;',
  '    procedure SetItems(Index: word; Value: word);',
  '  public',
  '    property Items[Index: word]: word read GetItems write SetItems; default;',
  '  end;',
  '  TGlob = record',
  '  private',
  '    class function GetSizes(Index: word): word; static;',
  '    class procedure SetSizes(Index: word; Value: word); static;',
  '  public',
  '    class property Sizes[Index: word]: word read GetSizes write SetSizes; default;',
  '  end;',
  'function TRec.GetItems(Index: word): word;',
  'begin',
  'end;',
  'procedure TRec.SetItems(Index: word; Value: word);',
  'begin',
  'end;',
  'class function TGlob.GetSizes(Index: word): word;',
  'begin',
  'end;',
  'class procedure TGlob.SetSizes(Index: word; Value: word);',
  'begin',
  'end;',
  'var',
  '  r: TRec;',
  '  g: TGlob;',
  'begin',
  '  r[1]:=r[2];',
  '  TGlob[1]:=TGlob[2];',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_RecordAsFuncResult;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  {#A}TRec = record',
  '     {#A_i}i: longint;',
  '     class function {#A_CreateA}Create: TRec; static;',
  '     class function {#A_CreateB}Create(i: longint): TRec; static;',
  '  end;',
  'function {#F}F: TRec;',
  'begin',
  '  Result:=default(TRec);',
  'end;',
  'class function TRec.Create: TRec;',
  'begin',
  '  Result:=default(TRec);',
  'end;',
  'class function TRec.Create(i: longint): TRec;',
  'begin',
  '  Result:=default(TRec);',
  '  Result.i:=i;',
  'end;',
  'var',
  '  {#v}{=A}v: TRec;',
  'begin',
  '  {@v}v:={@F}F;',
  '  {@v}v:={@F}F();',
  '  if {@v}v={@F}F then ;',
  '  if {@v}v={@F}F() then ;',
  '  {@v}v:={@A}TRec.{@A_CreateA}Create;',
  '  {@v}v:={@A}TRec.{@A_CreateA}Create();',
  '  {@v}v:={@A}TRec.{@A_CreateB}Create(3);',
  '  {@A}TRec.{@A_CreateA}Create . {@A_i}i:=4;',
  '  {@A}TRec.{@A_CreateA}Create().{@A_i}i:=5;',
  '  {@A}TRec.{@A_CreateB}Create(3).{@A_i}i:=6;']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_InheritedFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    procedure DoIt;',
  '  end;',
  'procedure TRec.DoIt;',
  'begin',
  '  inherited;',
  'end;',
  'begin',
  '']);
  CheckResolverException('The use of "inherited" is not allowed in a record',
    nTheUseOfXisNotAllowedInARecord);
end;

procedure TTestResolver.TestAdvRecord_ForInEnumerator;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'type',
  '  TObject = class end;',
  '  TItem = TObject;',
  '  TEnumerator = class',
  '    FCurrent: TItem;',
  '    property Current: TItem read FCurrent;',
  '    function MoveNext: boolean;',
  '  end;',
  '  TBird = record',
  '    function GetEnumerator: TEnumerator;',
  '  end;',
  'function TEnumerator.MoveNext: boolean;',
  'begin',
  'end;',
  'function TBird.GetEnumerator: TEnumerator;',
  'begin',
  'end;',
  'var',
  '  b: TBird;',
  '  i: TItem;',
  '  {#i2}i2: TItem;',
  'begin',
  '  for i in b do {@i2}i2:=i;']);
  ParseProgram;
end;

procedure TTestResolver.TestAdvRecord_InFunctionFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  'procedure DoIt;',
  'type',
  '  TBird = record',
  '    class var i: word;',
  '  end;',
  'var',
  '  b: TBird;',
  'begin',
  'end;',
  'begin']);
  CheckParserException(sErrRecordVariablesNotAllowed,nErrRecordVariablesNotAllowed);
end;

procedure TTestResolver.TestAdvRecord_SubClass;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TObject = class end;',
  '  TPoint = record',
  '  type',
  '    TBird = class',
  '      procedure DoIt;',
  '      class procedure Glob;',
  '    end;',
  '    procedure DoIt(b: TBird);',
  '  end;',
  'procedure TPoint.TBird.DoIt;',
  'begin',
  'end;',
  'class procedure TPoint.TBird.Glob;',
  'begin',
  'end;',
  'procedure TPoint.DoIt(b: TBird);',
  'begin',
  'end;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClass;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    {#B}b: longint;');
  Add('  end;');
  Add('var');
  Add('  {#C}{=TOBJ}c: TObject;');
  Add('begin');
  Add('  {@C}c.{@b}b:=3;');
  ParseProgram;
end;

procedure TTestResolver.TestClassDefaultInheritance;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    {#OBJ_b}b: longint;');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    {#A_c}c: longint;');
  Add('  end;');
  Add('var');
  Add('  {#V}{=A}v: TClassA;');
  Add('begin');
  Add('  {@V}v.{@A_c}c:=2;');
  Add('  {@V}v.{@OBJ_b}b:=3;');
  ParseProgram;
end;

procedure TTestResolver.TestClassTripleInheritance;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    {#OBJ_a}a: longint;');
  Add('    {#OBJ_b}b: longint;');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    {#A_c}c: longint;');
  Add('  end;');
  Add('  {#B}TClassB = class(TClassA)');
  Add('    {#B_d}d: longint;');
  Add('  end;');
  Add('var');
  Add('  {#V}{=B}v: TClassB;');
  Add('begin');
  Add('  {@V}v.{@B_d}d:=1;');
  Add('  {@V}v.{@A_c}c:=2;');
  Add('  {@V}v.{@OBJ_B}b:=3;');
  Add('  {@V}v.{@Obj_a}a:=4;');
  ParseProgram;
end;

procedure TTestResolver.TestClassInheritanceCycleFail;
begin
  StartProgram(false);
  Add([
  'type A = class(A)',
  'begin']);
  CheckResolverException(sAncestorCycleDetected,nAncestorCycleDetected);
end;

procedure TTestResolver.TestClassDefaultVisibility;
var
  Elements: TFPList;
  El: TPasElement;
  aMarker: PSrcMarker;
  i: Integer;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    {#B}b: longint;',
  '  end;',
  '  {$M+}',
  '  TPersistent = class',
  '    {#C}c: longint;',
  '  end;',
  '  {$M-}',
  '  TPic = class',
  '    {#D}d: longint;',
  '  end;',
  '  TComponent = class(TPersistent)',
  '    {#E}e: longint;',
  '  end;',
  '  TControl = class(TComponent)',
  '    {#F}f: longint;',
  '  end;',
  'begin']);
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestClassDefaultVisibility',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestClassDefaultVisibility ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El is TPasVariable) then continue;
        case aMarker^.Identifier of
        'B','D':
          if El.Visibility<>visPublic then
            RaiseErrorAtSrcMarker('expected visPublic at #'+aMarker^.Identifier+', but got '+VisibilityNames[El.Visibility],aMarker);
        else
          if El.Visibility<>visPublished then
            RaiseErrorAtSrcMarker('expected visPublished at #'+aMarker^.Identifier+', but got '+VisibilityNames[El.Visibility],aMarker);
        end;
        break;
        end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestClassForward;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  {#B_forward}TClassB = class;');
  Add('  {#A}TClassA = class');
  Add('    {#A_b}{=B_forward}b: TClassB;');
  Add('  end;');
  Add('  {#B}TClassB = class(TClassA)');
  Add('    {#B_a}a: longint;');
  Add('    {#B_d}d: longint;');
  Add('  end;');
  Add('var');
  Add('  {#V}{=B}v: TClassB;');
  Add('begin');
  Add('  {@V}v.{@B_d}d:=1;');
  Add('  {@V}v.{@B_a}a:=2;');
  Add('  {@V}v.{@A_b}b:=nil;');
  Add('  {@V}v.{@A_b}b.{@B_a}a:=3;');
  ParseProgram;
end;

procedure TTestResolver.TestClassForwardAsAncestorFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class;');
  Add('  TBird = class end;');
  Add('  TObject = class');
  Add('  end;');
  Add('var');
  Add('  v: TBird;');
  Add('begin');
  CheckResolverException('Can''t use forward declaration "TObject" as ancestor',
    nCantUseForwardDeclarationAsAncestor);
end;

procedure TTestResolver.TestClassForwardNotResolved;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  TClassB = class;');
  Add('var');
  Add('  v: TClassB;');
  Add('begin');
  CheckResolverException(sForwardTypeNotResolved,
    nForwardTypeNotResolved);
end;

procedure TTestResolver.TestClassForwardDuplicateFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class;',
  '  TObject = class;',
  '  TObject = class',
  '  end;',
  'begin']);
  CheckResolverException('Duplicate identifier "TObject" at afile.pp(3,10)',nDuplicateIdentifier);
end;

procedure TTestResolver.TestClassForwardDelphiFail;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class end;',
  '  TBird = class;',
  'const k = 1;',
  'type',
  '  TBird = class',
  '  end;',
  'begin']);
  CheckResolverException('Forward type not resolved "TBird"',nForwardTypeNotResolved);
end;

procedure TTestResolver.TestClassForwardObjFPCProgram;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  'type',
  '  TObject = class end;',
  '  TBird = class;',
  'const k = 1;',
  'type',
  '  TBird = class',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassForwardObjFPCUnit;
begin
  StartUnit(false);
  Add([
  '{$mode objfpc}',
  'interface',
  'type',
  '  TObject = class end;',
  '  TBird = class;',
  'const k = 1;',
  'type',
  '  TBird = class',
  '  end;',
  'implementation',
  'type',
  '  TEagle = class;',
  'const c = 1;',
  'type',
  '  TEagle = class',
  '  end;',
  '']);
  ParseUnit;
end;

procedure TTestResolver.TestClass_Method;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    procedure {#A_ProcA_Decl}ProcA;');
  Add('  end;');
  Add('procedure TClassA.ProcA;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#V}{=A}v: TClassA;');
  Add('begin');
  Add('  {@V}v.{@A_ProcA_Decl}ProcA;');
  ParseProgram;
end;

procedure TTestResolver.TestClass_ConstructorMissingDotFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  'constructor Create; begin end;',
  'begin',
  '']);
  CheckResolverException('full method name expected, but short name found',
    nXExpectedButYFound);
end;

procedure TTestResolver.TestClass_MethodImplDuplicateFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoIt;',
  '  end;',
  'procedure TObject.DoIt; begin end;',
  'procedure TObject.DoIt; begin end;',
  'begin',
  '']);
  CheckResolverException('Duplicate identifier "TObject.DoIt" at afile.pp(6,23) at afile.pp (7,23)',
    nDuplicateIdentifier);
end;

procedure TTestResolver.TestClass_MethodWithoutClassFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('procedure TClassA.ProcA;');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckResolverException('class "TClassA" not found in this module',nClassXNotFoundInThisModule);
end;

procedure TTestResolver.TestClass_MethodInOtherUnitFail;
begin
  AddModuleWithIntfImplSrc('unit1.pas',
    LinesToStr([
    'type',
    '  TObject = class',
    '  public',
    '  end;',
    '']),
    '');

  StartProgram(true);
  Add([
  'uses unit1;',
  'procedure TObject.DoIt;',
  'begin',
  'end;',
  'begin']);
  CheckResolverException('class "TObject" not found in this module',
    nClassXNotFoundInThisModule);
end;

procedure TTestResolver.TestClass_MethodWithParams;
begin
  StartProgram(false);
  Add('type');
  Add('  {#A}TObject = class');
  Add('    procedure {#ProcA_Decl}ProcA({#Bar}Bar: longint);');
  Add('  end;');
  Add('procedure tobject.proca(bar: longint);');
  Add('begin');
  Add('  if {@Bar}bar=3 then ;');
  Add('end;');
  Add('var');
  Add('  {#V}{=A}Obj: TObject;');
  Add('begin');
  Add('  {@V}Obj.{@ProcA_Decl}ProcA(4);');
  ParseProgram;
end;

procedure TTestResolver.TestClass_MethodUnresolvedPrg;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  TClassA = class');
  Add('    procedure ProcA;');
  Add('  end;');
  Add('begin');
  CheckResolverException(sForwardProcNotResolved,nForwardProcNotResolved);
end;

procedure TTestResolver.TestClass_MethodUnresolvedUnit;
begin
  StartUnit(false);
  Add('interface');
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  TClassA = class');
  Add('    procedure ProcA;');
  Add('  end;');
  Add('implementation');
  CheckResolverException(sForwardProcNotResolved,nForwardProcNotResolved);
end;

procedure TTestResolver.TestClass_MethodAbstract;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure ProcA; virtual; abstract;');
  Add('  end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_MethodAbstractWithoutVirtualFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure ProcA; abstract;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Invalid procedure modifier abstract without virtual',nInvalidXModifierY);
end;

procedure TTestResolver.TestClass_MethodAbstractHasBodyFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure ProcA; virtual; abstract;');
  Add('  end;');
  Add('procedure TObject.ProcA;');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckResolverException(sAbstractMethodsMustNotHaveImplementation,
    nAbstractMethodsMustNotHaveImplementation);
end;

procedure TTestResolver.TestClass_MethodUnresolvedWithAncestor;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure ProcA; virtual; abstract;');
  Add('  end;');
  Add('  TClassA = class');
  Add('    procedure ProcA;');
  Add('  end;');
  Add('begin');
  CheckResolverException(sForwardProcNotResolved,nForwardProcNotResolved);
end;

procedure TTestResolver.TestClass_ProcFuncMismatch;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure DoIt;');
  Add('  end;');
  Add('function TObject.DoIt: longint;');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckResolverException('procedure expected, but function found',
    nXExpectedButYFound);
end;

procedure TTestResolver.TestClass_MethodOverload;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure DoIt;');
  Add('    procedure DoIt(i: longint);');
  Add('    procedure DoIt(s: string);');
  Add('  end;');
  Add('procedure TObject.DoIt;');
  Add('begin');
  Add('end;');
  Add('procedure TObject.DoIt(i: longint);');
  Add('begin');
  Add('end;');
  Add('procedure TObject.DoIt(s: string);');
  Add('begin');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_MethodInvalidOverload;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure DoIt(i: longint);');
  Add('    procedure DoIt(k: longint);');
  Add('  end;');
  Add('procedure TObject.DoIt(i: longint);');
  Add('begin');
  Add('end;');
  Add('procedure TObject.DoIt(k: longint);');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckResolverException(sDuplicateIdentifier,nDuplicateIdentifier);
end;

procedure TTestResolver.TestClass_MethodOverride;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure {#TOBJ_ProcA}ProcA; virtual; abstract;');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    procedure {#A_ProcA}ProcA; override;');
  Add('  end;');
  Add('procedure TClassA.ProcA;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#V}{=A}v: TClassA;');
  Add('begin');
  Add('  {@V}v.{@A_ProcA}ProcA;');
  ParseProgram;
end;

procedure TTestResolver.TestClass_MethodOverride2;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure {#TOBJ_ProcA}ProcA; virtual; abstract;');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    procedure {#A_ProcA}ProcA; override;');
  Add('  end;');
  Add('  {#B}TClassB = class');
  Add('    procedure {#B_ProcA}ProcA; override;');
  Add('  end;');
  Add('procedure TClassA.ProcA;');
  Add('begin');
  Add('end;');
  Add('procedure TClassB.ProcA;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#V}{=B}v: TClassB;');
  Add('begin');
  Add('  {@V}v.{@B_ProcA}ProcA;');
  ParseProgram;
end;

procedure TTestResolver.TestClass_MethodOverrideFixCase;

  procedure CheckOverrideName(aLabel: string);
  var
    Elements: TFPList;
    i: Integer;
    El: TPasElement;
    Scope: TPasProcedureScope;
  begin
    Elements:=FindElementsAtSrcLabel(aLabel);
    try
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        if not (El is TPasProcedure) then continue;
        Scope:=El.CustomData as TPasProcedureScope;
        if Scope.OverriddenProc=nil then
          Fail('Scope.OverriddenProc=nil');
        AssertEquals('Proc Name and Proc.Scope.OverriddenProc.Name',El.Name,Scope.OverriddenProc.Name);
        end;
    finally
      Elements.Free;
    end;
  end;

begin
  ResolverEngine.Options:=ResolverEngine.Options+[proFixCaseOfOverrides];
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure {#TOBJ_ProcA}ProcA; virtual; abstract;');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    procedure {#A_ProcA}proca; override;');
  Add('  end;');
  Add('  {#B}TClassB = class');
  Add('    procedure {#B_ProcA}prOca; override;');
  Add('  end;');
  Add('procedure tclassa.proca;');
  Add('begin');
  Add('end;');
  Add('procedure tclassb.proca;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#V}{=B}v: TClassB;');
  Add('begin');
  Add('  {@V}v.{@B_ProcA}ProcA;');
  ParseProgram;
  CheckOverrideName('A_ProcA');
  CheckOverrideName('B_ProcA');
end;

procedure TTestResolver.TestClass_MethodOverrideSameResultType;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'type',
    '  TObject = class',
    '  public',
    '    function ProcA(const s: string): string; virtual; abstract;',
    '  end;',
    '']),
    LinesToStr([
    ''])
    );

  StartProgram(true);
  Add('uses unit2;');
  Add('type');
  Add('  TCar = class');
  Add('  public');
  Add('    function ProcA(const s: string): string; override;');
  Add('  end;');
  Add('function TCar.ProcA(const s: string): string; begin end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_MethodOverrideDiffResultTypeFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  public');
  Add('    function ProcA(const s: string): string; virtual; abstract;');
  Add('  end;');
  Add('  TCar = class');
  Add('  public');
  Add('    function ProcA(const s: string): longint; override;');
  Add('  end;');
  Add('function TCar.ProcA(const s: string): longint; begin end;');
  Add('begin');
  CheckResolverException('Result type mismatch, expected String, but found Longint',
    nResultTypeMismatchExpectedButFound);
end;

procedure TTestResolver.TestClass_MethodOverrideDiffVarName;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoIt(aName: string); virtual; abstract;',
  '  end;',
  '  TCar = class',
  '    procedure DoIt(aCaption: string); override;',
  '  end;',
  'procedure TCar.DoIt(aCaption: string); begin end;',
  'begin'
  ]);
  ParseProgram;
end;

procedure TTestResolver.TestClass_MethodOverloadMissingInDelphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class',
  '    procedure DoIt(i: longint); virtual; abstract;',
  '    procedure DoIt(s: string); virtual; abstract;',
  '  end;',
  'begin'
  ]);
  CheckResolverException(sPreviousDeclMissesOverload,nPreviousDeclMissesOverload);
end;

procedure TTestResolver.TestClass_MethodOverloadAncestor;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure {#A1}DoIt;');
  Add('    procedure {#B1}DoIt(i: longint);');
  Add('  end;');
  Add('  TCar = class');
  Add('    procedure {#A2}DoIt;');
  Add('    procedure {#B2}DoIt(i: longint);');
  Add('  end;');
  Add('procedure TObject.DoIt; begin end;');
  Add('procedure TObject.DoIt(i: longint); begin end;');
  Add('procedure TCar.DoIt;');
  Add('begin');
  Add('  {@A2}DoIt;');
  Add('  {@B2}DoIt(1);');
  Add('  inherited {@A1}DoIt;');
  Add('  inherited {@B1}DoIt(2);');
  Add('end;');
  Add('procedure TCar.DoIt(i: longint); begin end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_MethodOverloadUnit;
begin
  StartProgram(true);
  Add([
  'type',
  '  TObject = class',
  '    procedure Copy(s: string);',
  '  end;',
  'procedure TObject.Copy(s: string);',
  'var a: array of longint;',
  'begin',
  '  a:=system.Copy(a,1,3);',
  'end;',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClass_HintMethodHidesNonVirtualMethod;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoIt(p: pointer);',
  '  end;',
  '  TBird = class',
  '    procedure DoIt(i: longint);',
  '  end;',
  'procedure TObject.DoIt(p: pointer);',
  'begin',
  '  if p=nil then ;',
  'end;',
  'procedure TBird.DoIt(i: longint); begin end;',
  'var b: TBird;',
  'begin',
  '  b.DoIt(3);']);
  ParseProgram;
  CheckResolverHint(mtHint,nFunctionHidesIdentifier_NonVirtualMethod,
   'function hides identifier at "afile.pp(4,19)". Use overload or reintroduce');
end;

procedure TTestResolver.
  TestClass_HintMethodHidesNonVirtualMethodWithoutBody_NoHint;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'type',
    '  TObject = class',
    '  public',
    '    procedure DoIt(p: pointer);',
    '  end;',
    '']),
    LinesToStr([
    'procedure TObject.DoIt(p: pointer);',
    'begin',
    'end;',
    '']) );

  StartProgram(true);
  Add([
  'uses unit2;',
  'type',
  '  TBird = class',
  '    procedure DoIt(i: longint);',
  '  end;',
  'procedure TBird.DoIt(i: longint); begin end;',
  'var b: TBird;',
  'begin',
  '  b.DoIt(3);']);
  ParseProgram;
  CheckResolverUnexpectedHints(true);
end;

procedure TTestResolver.TestClass_NoHintMethodHidesPrivateMethod;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'type',
    '  TObject = class',
    '  private',
    '    procedure DoIt(p: pointer);',
    '  end;',
    '']),
    LinesToStr([
    'procedure TObject.DoIt(p: pointer);',
    'begin',
    '  if p=nil then ;',
    'end;',
    '']) );
  StartProgram(true);
  Add([
  'uses unit2;',
  'type',
  '  TAnimal = class',
  '  strict private',
  '    procedure Fly(p: pointer);',
  '  end;',
  '  TBird = class(TAnimal)',
  '    procedure DoIt(i: longint);',
  '    procedure Fly(b: boolean);',
  '  end;',
  'procedure TAnimal.Fly(p: pointer);',
  'begin',
  '  if p=nil then ;',
  'end;',
  'procedure TBird.DoIt(i: longint); begin end;',
  'procedure TBird.Fly(b: boolean); begin end;',
  'var b: TBird;',
  'begin',
  '  b.DoIt(3);']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClass_MethodReintroduce;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoIt(p: pointer); virtual; abstract;',
  '  end;',
  '  TBird = class',
  '    procedure DoIt(i: longint); virtual; abstract; reintroduce;',
  '    procedure DoIt(s: string); virtual; abstract;',
  '  end;',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClass_MethodOverloadArrayOfTClass;
begin
  StartProgram(false);
  Add([
  'type',
  '  TClass = class of TObject;',
  '  TObject = class',
  '    constructor {#A}Builder(AClass: TClass; AName: string); reintroduce; overload; virtual;',
  '    constructor {#B}Builder(AClass: TClass); reintroduce; overload; virtual;',
  '    constructor {#C}Builder(AClassArray: Array of TClass); reintroduce; overload; virtual;',
  '    constructor {#D}Builder(AName: string); reintroduce; overload; virtual;',
  '    constructor {#E}Builder; reintroduce; overload; virtual;',
  '    class var ClassName: string;',
  '  end;',
  '  TTestCase = class end;',
  'constructor TObject.Builder(AClass: TClass; AName: string);',
  'begin',
  '  Builder(AClass);',
  'end;',
  'constructor TObject.Builder(AClass: TClass);',
  'begin',
  '  Builder(AClass.ClassName);',
  'end;',
  'constructor TObject.Builder(AClassArray: Array of TClass);',
  'var',
  '  i: longint;',
  'begin',
  '  Builder;',
  '  for i := Low(AClassArray) to High(AClassArray) do',
  '    if Assigned(AClassArray[i]) then ;',
  'end;',
  'constructor TObject.Builder(AName: string);',
  'begin',
  '  Builder();',
  'end;',
  'constructor TObject.Builder;',
  'begin',
  'end;',
  'var',
  '  o: TObject;',
  'begin',
  '  o.{@A}Builder(TTestCase,''first'');',
  '  o.{@B}Builder(TTestCase);',
  '  o.{@C}Builder([]);',
  '  o.{@C}Builder([TTestCase]);',
  '  o.{@C}Builder([TObject,TTestCase]);',
  '  o.{@D}Builder(''fourth'');',
  '  o.{@E}Builder();',
  '  o.{@E}Builder;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClass_ConstructorHidesAncestorWarning;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create(o: tobject); virtual; abstract;',
  '  end;',
  '  TBird = class',
  '    constructor Create(s: string); virtual; abstract;',
  '  end;',
  'begin',
  '']);
  ParseProgram;
  CheckResolverHint(mtWarning,nMethodHidesMethodOfBaseType,
    'Method "Create" hides method of base type "TObject" at afile.pp(4,23)');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClass_ConstructorOverride;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create(o: tobject); virtual;',
  '  end;',
  '  TBird = class',
  '    constructor Create(o: tobject); override;',
  '  end;',
  '  TEagle = class(TBird)',
  '    constructor Create(o: tobject); override;',
  '  end;',
  'constructor tobject.Create(o: tobject); begin end;',
  'constructor tbird.Create(o: tobject); begin end;',
  'constructor teagle.Create(o: tobject); begin end;',
  'var o: TEagle;',
  'begin',
  '  o:=TEagle.Create(nil);',
  '  o:=TEagle.Create(o);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClass_ConstructorAccessHiddenAncestorFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create(o: tobject);',
  '  end;',
  '  TBird = class',
  '    constructor Create(i: longint); reintroduce;',
  '  end;',
  'constructor tobject.Create(o: tobject); begin end;',
  'constructor tbird.Create(i: longint); begin end;',
  'var o: TBird;',
  'begin',
  '  o:=TBird.Create(nil);',
  '']);
  CheckResolverException('Incompatible type arg no. 1: Got "Nil", expected "Longint"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestClass_ConstructorNoteAbstractMethods;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoIt; virtual; abstract;',
  '    constructor Create; virtual;',
  '  end;',
  'constructor TObject.Create;',
  'begin',
  'end;',
  'begin',
  '  TObject.Create;']);
  ParseProgram;
  CheckResolverHint(mtWarning,nConstructingClassXWithAbstractMethodY,'Constructing a class "TObject" with abstract method "DoIt"');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClass_ConstructorNoNoteAbstractMethods;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoIt; virtual; abstract;',
  '    constructor Create;',
  '  end;',
  '  TClass = class of TObject;',
  'constructor TObject.Create;',
  'begin',
  'end;',
  'var c: TClass;',
  'begin',
  '  c.Create;',
  '  with c do Create;',
  '']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClass_MethodScope;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    {#A_A}A: longint;');
  Add('    procedure {#A_ProcB}ProcB;');
  Add('  end;');
  Add('procedure TClassA.ProcB;');
  Add('begin');
  Add('  {@A_A}A:=3;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_IdentifierSelf;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    {#C}C: longint;');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    {#B}B: longint;');
  Add('    procedure {#A_ProcB}ProcB;');
  Add('  end;');
  Add('procedure TClassA.ProcB;');
  Add('begin');
  Add('  {@B}B:=1;');
  Add('  {@C}C:=2;');
  Add('  Self.{@B}B:=3;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClassCallInherited;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure {#TOBJ_ProcA}ProcA(vI: longint); virtual;',
  '    procedure {#TOBJ_ProcB}ProcB(vJ: longint); virtual;',
  '  end;',
  '  {#A}TClassA = class',
  '    procedure {#A_ProcA}ProcA({#i1}vI: longint); override;',
  '    procedure {#A_ProcB}ProcB(vJ: longint); override;',
  '    procedure {#A_ProcC}ProcC; virtual;',
  '  end;',
  'procedure TObject.ProcA(vi: longint);',
  'begin',
  '  inherited; // ignore, do not raise error',
  'end;',
  'procedure TObject.ProcB(vj: longint);',
  'begin',
  'end;',
  'procedure TClassA.ProcA(vi: longint);',
  'begin',
  '  {@A_ProcA}ProcA({@i1}vI);',
  '  {@TOBJ_ProcA}inherited;',
  '  inherited {@TOBJ_ProcA}ProcA({@i1}vI);',
  '  {@A_ProcB}ProcB({@i1}vI);',
  '  inherited {@TOBJ_ProcB}ProcB({@i1}vI);',
  'end;',
  'procedure TClassA.ProcB(vJ: longint);',
  'begin',
  'end;',
  'procedure TClassA.ProcC;',
  'begin',
  '  inherited; // ignore, do not raise error',
  'end;',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClassCallInheritedNoParamsAbstractFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure ProcA; virtual; abstract;');
  Add('  end;');
  Add('  TClassA = class');
  Add('    procedure ProcA; override;');
  Add('  end;');
  Add('procedure TClassA.ProcA;');
  Add('begin');
  Add('  inherited;');
  Add('end;');
  Add('begin');
  CheckResolverException('Abstract methods cannot be called directly',
    nAbstractMethodsCannotBeCalledDirectly);
end;

procedure TTestResolver.TestClassCallInheritedWithParamsAbstractFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure ProcA(c: char); virtual; abstract;');
  Add('  end;');
  Add('  TClassA = class');
  Add('    procedure ProcA(c: char); override;');
  Add('  end;');
  Add('procedure TClassA.ProcA(c: char);');
  Add('begin');
  Add('  inherited ProcA(c);');
  Add('end;');
  Add('begin');
  CheckResolverException('Abstract methods cannot be called directly',
    nAbstractMethodsCannotBeCalledDirectly);
end;

procedure TTestResolver.TestClassCallInheritedConstructor;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    constructor {#TOBJ_CreateA}Create(vI: longint); virtual;');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    constructor {#A_CreateA}Create({#i1}vI: longint); override;');
  Add('  end;');
  Add('constructor TObject.Create(vI: longint);');
  Add('begin');
  Add('  inherited; // ignore and do not raise error');
  Add('end;');
  Add('constructor TClassA.Create(vI: longint);');
  Add('begin');
  Add('  {@A_CreateA}Create({@i1}vI);');
  Add('  {@TOBJ_CreateA}inherited;');
  Add('  inherited {@TOBJ_CreateA}Create({@i1}vI);');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClassCallInheritedNested;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function DoIt: longint; virtual;',
  '  end;',
  '  TBird = class',
  '    function DoIt: longint; override;',
  '  end;',
  'function tobject.doit: longint;',
  'begin',
  'end;',
  'function tbird.doit: longint;',
  '  procedure Sub;',
  '  begin',
  '    inherited;',
  '    inherited DoIt;',
  '    if inherited DoIt=4 then ;',
  '  end;',
  'begin',
  '  Sub;',
  '  inherited;',
  '  inherited DoIt;',
  '  if inherited DoIt=14 then ;',
  '  with Self do inherited;',
  '  with Self do inherited DoIt;',
  'end;',
  'begin',
   '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassCallInheritedAs;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function GetSome: TObject; virtual;',
  '  end;',
  '  TBird = class',
  '    function GetIt: TBird;',
  '  end;',
  'function TObject.GetSome: TObject;',
  'begin',
  'end;',
  'function TBird.GetIt: TBird;',
  'begin',
  '  Result:=inherited GetSome as TBird;',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassAssignNil;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    {#FSub}FSub: TClassA;');
  Add('    property {#Sub}Sub: TClassA read {@FSub}FSub write {@FSub}FSub;');
  Add('  end;');
  Add('var');
  Add('  {#v}{=A}v: TClassA;');
  Add('begin');
  Add('  {@v}v:=nil;');
  Add('  if {@v}v=nil then ;');
  Add('  if nil={@v}v then ;');
  Add('  if {@v}v<>nil then ;');
  Add('  if nil<>{@v}v then ;');
  Add('  {@v}v.{@FSub}FSub:=nil;');
  Add('  if {@v}v.{@FSub}FSub=nil then ;');
  Add('  if {@v}v.{@FSub}FSub<>nil then ;');
  Add('  {@v}v.{@Sub}Sub:=nil;');
  Add('  if {@v}v.{@Sub}Sub=nil then ;');
  Add('  if {@v}v.{@Sub}Sub<>nil then ;');
  ParseProgram;
end;

procedure TTestResolver.TestClassAssign;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    {#FSub}FSub: TClassA;');
  Add('    property {#Sub}Sub: TClassA read {@FSub}FSub write {@FSub}FSub;');
  Add('  end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#v}{=A}v: TClassA;');
  Add('  {#p}{=A}p: TClassA;');
  Add('begin');
  Add('  {@o}o:={@v}v;');
  Add('  {@v}v:={@p}p;');
  Add('  if {@v}v={@p}p then ;');
  Add('  if {@v}v={@o}o then ;');
  Add('  if {@o}o={@o}o then ;');
  Add('  if {@o}o={@v}v then ;');
  Add('  if {@v}v<>{@p}p then ;');
  Add('  if {@v}v<>{@o}o then ;');
  Add('  if {@o}o<>{@o}o then ;');
  Add('  if {@o}o<>{@v}v then ;');
  Add('  {@v}v.{@FSub}FSub:={@p}p;');
  Add('  {@p}p:={@v}v.{@FSub}FSub;');
  Add('  {@o}o:={@v}v.{@FSub}FSub;');
  Add('  {@v}v.{@Sub}Sub:={@p}p;');
  Add('  {@p}p:={@v}v.{@Sub}Sub;');
  Add('  {@o}o:={@v}v.{@Sub}Sub;');
  ParseProgram;
end;

procedure TTestResolver.TestClassNilAsParam;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('  end;');
  Add('procedure ProcP(o: TObject);');
  Add('begin end;');
  Add('begin');
  Add('  ProcP(nil);');
  ParseProgram;
end;

procedure TTestResolver.TestClass_Operators_Is_As;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    {#Sub}Sub: TClassA;');
  Add('  end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#v}{=A}v: TClassA;');
  Add('begin');
  Add('  if {@o}o is {@A}TClassA then;');
  Add('  if {@v}v is {@A}TClassA then;');
  Add('  if {@v}v is {@TOBJ}TObject then;');
  Add('  if {@v}v.{@Sub}Sub is {@A}TClassA then;');
  Add('  {@v}v:={@o}o as {@A}TClassA;');
  ParseProgram;
end;

procedure TTestResolver.TestClass_OperatorIsOnNonTypeFail;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('  end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#v}{=A}v: TClassA;');
  Add('begin');
  Add('  if {@o}o is {@v}v then;');
  CheckResolverException('class type expected, but class found',
    nXExpectedButYFound);
end;

procedure TTestResolver.TestClass_OperatorAsOnNonDescendantFail;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('  end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#v}{=A}v: TClassA;');
  Add('begin');
  Add('  {@o}o:={@v}v as {@TObj}TObject;');
  CheckResolverException('Types are not related: "TClassA" and "class TObject" at afile.pp (11,16)',nTypesAreNotRelatedXY);
end;

procedure TTestResolver.TestClass_OperatorAsOnNonTypeFail;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('  end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#v}{=A}v: TClassA;');
  Add('begin');
  Add('  {@o}o:={@v}v as {@o}o;');
  CheckResolverException('class expected, but o found',
    nXExpectedButYFound);
end;

procedure TTestResolver.TestClassAsFuncResult;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TOBJ}TObject = class',
  '  end;',
  '  {#A}TClassA = class',
  '     {#A_i}i: longint;',
  '     constructor {#A_CreateA}Create;',
  '     constructor {#A_CreateB}Create(i: longint);',
  '  end;',
  'function {#F}F: TClassA;',
  'begin',
  '  Result:=nil;',
  'end;',
  'constructor TClassA.Create;',
  'begin',
  'end;',
  'constructor TClassA.Create(i: longint);',
  'begin',
  'end;',
  'var',
  '  {#o}{=TOBJ}o: TObject;',
  '  {#v}{=A}v: TClassA;',
  'begin',
  '  {@o}o:={@F}F;',
  '  {@o}o:={@F}F();',
  '  {@v}v:={@F}F;',
  '  {@v}v:={@F}F();',
  '  if {@o}o={@F}F then ;',
  '  if {@o}o={@F}F() then ;',
  '  if {@v}v={@F}F then ;',
  '  if {@v}v={@F}F() then ;',
  '  {@v}v:={@A}TClassA.{@A_CreateA}Create;',
  '  {@v}v:={@A}TClassA.{@A_CreateA}Create();',
  '  {@v}v:={@A}TClassA.{@A_CreateB}Create(3);',
  '  {@A}TClassA.{@A_CreateA}Create.{@A_i}i:=3;',
  '  {@A}TClassA.{@A_CreateA}Create().{@A_i}i:=3;',
  '  {@A}TClassA.{@A_CreateB}Create(3).{@A_i}i:=3;']);
  ParseProgram;
end;

procedure TTestResolver.TestClassTypeCast;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    id: longint;');
  Add('  end;');
  Add('procedure ProcA(var a: TClassA);');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#v}{=A}v: TClassA;');
  Add('begin');
  Add('  {@o}o:={@v}v;');
  Add('  {@o}o:=TObject({@o}o);');
  Add('  {@v}v:=TClassA({@o}o);');
  Add('  {@v}v:=TClassA(TObject({@o}o));');
  Add('  {@v}v:=TClassA({@v}v);');
  Add('  {@v}v:=v as TClassA;');
  Add('  {@v}v:=o as TClassA;');
  Add('  ProcA({@v}v);');
  Add('  ProcA(TClassA({@o}o));');
  Add('  if TClassA({@o}o).id=3 then ;');
  Add('  if (o as TClassA).id=3 then ;');
  Add('  o:=TObject(nil);');
  ParseProgram;
end;

procedure TTestResolver.TestClassTypeCastUnrelatedFail;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    id: longint;');
  Add('  end;');
  Add('  {#B}TClassB = class');
  Add('    Name: string;');
  Add('  end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#va}{=A}va: TClassA;');
  Add('  {#vb}{=B}vb: TClassB;');
  Add('begin');
  Add('  {@vb}vb:=TClassB({@va}va);');
  CheckResolverException('Illegal type conversion: "TClassA" to "class TClassB"',
    nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestClass_TypeCastSelf;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    constructor Create;');
  Add('    procedure ProcA;');
  Add('  end;');
  Add('  TClassA = class');
  Add('    id: longint;');
  Add('  end;');
  Add('constructor TObject.Create;');
  Add('begin');
  Add('  TClassA(Self).id:=3;');
  Add('  if TClassA(Self).id=4 then;');
  Add('  if 5=TClassA(Self).id then;');
  Add('end;');
  Add('procedure TObject.ProcA;');
  Add('begin');
  Add('  TClassA(Self).id:=3;');
  Add('  if TClassA(Self).id=4 then;');
  Add('  if 5=TClassA(Self).id then;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_TypeCaseMultipleParamsFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    i: longint;');
  Add('  end;');
  Add('var o: TObject;');
  Add('begin');
  Add('  o.i:=TObject(o,o).i;');
  CheckResolverException('wrong number of parameters for type cast to TObject',
    nWrongNumberOfParametersForTypeCast);
end;

procedure TTestResolver.TestClass_TypeCastAssign;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  TCar = class');
  Add('  end;');
  Add('procedure DoIt(a: TCar; const b: TCar; var c: TCar; out d: TCar); begin end;');
  Add('var');
  Add('  o: TObject;');
  Add('  c: TCar;');
  Add('begin');
  Add('  TCar({#a_assign}o):=nil;');
  Add('  TCar({#b_assign}o):=c;');
  Add('  DoIt(TCar({#c1_read}o),TCar({#c2_read}o),TCar({#c3_var}o),TCar({#c4_out}o));');
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestClass_AccessMemberViaClassFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    i: longint;');
  Add('  end;');
  Add('begin');
  Add('  if TObject.i=7 then ;');
  CheckResolverException(sInstanceMemberXInaccessible,
    nInstanceMemberXInaccessible);
end;

procedure TTestResolver.TestClass_FuncReturningObjectMember;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    i: longint;');
  Add('  end;');
  Add('function FuncO: TObject;');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  FuncO.i:=3;');
  Add('  if FuncO.i=4 then ;');
  Add('  if 5=FuncO.i then ;');
  ParseProgram;
end;

procedure TTestResolver.TestClass_StaticWithoutClassFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure ProcA; static;');
  Add('  end;');
  Add('procedure TObject.ProcA; begin end;');
  Add('begin');
  CheckResolverException('Invalid procedure modifier static',
    nInvalidXModifierY);
end;

procedure TTestResolver.TestClass_SelfInStaticFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class procedure ProcA; static;');
  Add('  end;');
  Add('class procedure TObject.ProcA;');
  Add('begin');
  Add('  if Self=nil then ;');
  Add('end;');
  Add('begin');
  CheckResolverException('identifier not found "Self"',nIdentifierNotFound);
end;

procedure TTestResolver.TestClass_SelfDotInStaticFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class var FLeft: word;');
  Add('    class function DoIt: word; static;');
  Add('    class property Left: word read FLeft;');
  Add('  end;');
  Add('class function TObject.DoIt: word;');
  Add('begin');
  Add('  Result:=Self.Left;');
  Add('end;');
  Add('begin');
  CheckResolverException('identifier not found "Self"',nIdentifierNotFound);
end;

procedure TTestResolver.TestClass_PrivateProtectedInSameUnit;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  strict private {#vstrictprivate}vstrictprivate: longint;');
  Add('  strict protected {#vstrictprotected}vstrictprotected: longint;');
  Add('  private {#vprivate}vprivate: longint;');
  Add('  protected {#vprotected}vprotected: longint;');
  Add('  public {#vpublic}vpublic: longint;');
  Add('    procedure ProcA;');
  Add('  automated {#vautomated}vautomated: longint;');
  Add('  published {#vpublished}vpublished: longint;');
  Add('  end;');
  Add('procedure TObject.ProcA;');
  Add('begin');
  Add('  if {@vstrictprivate}vstrictprivate=1 then ;');
  Add('  if {@vstrictprotected}vstrictprotected=2 then ;');
  Add('  if {@vprivate}vprivate=3 then ;');
  Add('  if {@vprotected}vprotected=4 then ;');
  Add('  if {@vpublic}vpublic=5 then ;');
  Add('  if {@vautomated}vautomated=6 then ;');
  Add('  if {@vpublished}vpublished=7 then ;');
  Add('end;');
  Add('var');
  Add('  o: TObject;');
  Add('begin');
  Add('  if o.vprivate=10 then ;');
  Add('  if o.vprotected=11 then ;');
  Add('  if o.vpublic=12 then ;');
  Add('  if o.vautomated=13 then ;');
  Add('  if o.vpublished=14 then ;');
  ParseProgram;
end;

procedure TTestResolver.TestClass_PrivateInMainBeginFail;
begin
  AddModuleWithSrc('unit1.pas',
    LinesToStr([
      'unit unit1;',
      'interface',
      'type',
      '  TObject = class',
      '  private v: longint;',
      '  end;',
      'implementation',
      'end.'
      ]));
  StartProgram(true);
  Add('uses unit1;');
  Add('var');
  Add('  o: TObject;');
  Add('begin');
  Add('  if o.v=3 then ;');
  CheckResolverException('Can''t access private member v',
    nCantAccessXMember);
end;

procedure TTestResolver.TestClass_PrivateInDescendantFail;
begin
  AddModuleWithSrc('unit1.pas',
    LinesToStr([
      'unit unit1;',
      'interface',
      'type',
      '  TObject = class',
      '  private v: longint;',
      '  end;',
      'implementation',
      'end.'
      ]));
  StartProgram(true);
  Add('uses unit1;');
  Add('type');
  Add('  TClassA = class(TObject)');
  Add('    procedure ProcA;');
  Add('  end;');
  Add('procedure TClassA.ProcA;');
  Add('begin');
  Add('  if v=3 then ;');
  Add('end;');
  Add('begin');
  CheckResolverException('Can''t access private member v',
    nCantAccessXMember);
end;

procedure TTestResolver.TestClass_ProtectedInDescendant;
begin
  AddModuleWithSrc('unit1.pas',
    LinesToStr([
      'unit unit1;',
      'interface',
      'type',
      '  TObject = class',
      '  protected vprotected: longint;',
      '  strict protected vstrictprotected: longint;',
      '  end;',
      'implementation',
      'end.'
      ]));
  StartProgram(true);
  Add([
  'uses unit1;',
  'type',
  '  TClassA = class(TObject)',
  '    procedure ProcA;',
  '  end;',
  '  TClassB = class(TObject)',
  '    procedure ProcB;',
  '  end;',
  'procedure TClassA.ProcA;',
  'begin',
  '  if vprotected=3 then ;',
  '  if vstrictprotected=4 then ;',
  '  if self.vprotected=5 then;',
  '  if self.vstrictprotected=6 then;',
  '  with self do if vprotected=7 then;',
  '  with self do if vstrictprotected=8 then;',
  'end;',
  'procedure TClassB.ProcB;',
  'var A: TClassA;',
  'begin',
  '  if A.vprotected=9 then;',
  '  with A do if vprotected=10 then;',
  'end;',
  'var A: TClassA;',
  'begin',
  '  A.vprotected:=11;',
  '  with A do vprotected:=12;',
  '  // error: A.vstrictprotected:=13; ']);
  ParseProgram;
end;

procedure TTestResolver.TestClass_StrictPrivateInMainBeginFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  strict private v: longint;');
  Add('  end;');
  Add('var');
  Add('  o: TObject;');
  Add('begin');
  Add('  if o.v=3 then ;');
  CheckResolverException('Can''t access strict private member v',
    nCantAccessXMember);
end;

procedure TTestResolver.TestClass_StrictProtectedInMainBeginFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  strict protected v: longint;');
  Add('  end;');
  Add('var');
  Add('  o: TObject;');
  Add('begin');
  Add('  if o.v=3 then ;');
  CheckResolverException('Can''t access strict protected member v',
    nCantAccessXMember);
end;

procedure TTestResolver.TestClass_Constructor_NewInstance;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
  ActualNewInstance, ActualImplicitCallWithoutParams: Boolean;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create;',
  '    class function DoSome: TObject;',
  '  end;',
  'constructor TObject.Create;',
  'begin',
  '  {#a}Create; // normal call',
  '  TObject.{#b}Create; // new instance',
  'end;',
  'class function TObject.DoSome: TObject;',
  'begin',
  '  Result:={#c}Create; // new instance',
  'end;',
  'var',
  '  o: TObject;',
  'begin',
  '  TObject.{#p}Create; // new object',
  '  o:=TObject.{#q}Create; // new object',
  '  o.{#r}Create; // normal call',
  '  o:=o.{#s}Create; // normal call',
  '']);
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestClass_Constructor_NewInstance ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualNewInstance:=false;
      ActualImplicitCallWithoutParams:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestClass_Constructor_NewInstance ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if not (Ref.Declaration is TPasProcedure) then continue;
        //writeln('TTestResolver.TestClass_Constructor_NewInstance ',GetObjName(Ref.Declaration),' rrfNewInstance=',rrfNewInstance in Ref.Flags);
        if (Ref.Declaration is TPasConstructor) then
          ActualNewInstance:=rrfNewInstance in Ref.Flags;
        ActualImplicitCallWithoutParams:=rrfImplicitCallWithoutParams in Ref.Flags;
        break;
        end;
      if not ActualImplicitCallWithoutParams then
        RaiseErrorAtSrcMarker('expected implicit call at "#'+aMarker^.Identifier+', but got function ref"',aMarker);
      case aMarker^.Identifier of
      'a','r','s':// should be normal call
        if ActualNewInstance then
          RaiseErrorAtSrcMarker('expected normal call at "#'+aMarker^.Identifier+', but got newinstance"',aMarker);
      else // should be newinstance
        if not ActualNewInstance then
          RaiseErrorAtSrcMarker('expected newinstance at "#'+aMarker^.Identifier+', but got normal call"',aMarker);
      end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestClass_Destructor_FreeInstance;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
  ActualFreeInstance, ActualImplicitCallWithoutParams: Boolean;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    destructor Destroy; virtual;');
  Add('  end;');
  Add('  TChild = class(TObject)');
  Add('    destructor DestroyOther;');
  Add('  end;');
  Add('destructor TObject.Destroy;');
  Add('begin');
  Add('end;');
  Add('destructor TChild.DestroyOther;');
  Add('begin');
  Add('  {#a}Destroy; // free instance');
  Add('  inherited {#b}Destroy; // normal call');
  Add('end;');
  Add('var');
  Add('  c: TChild;');
  Add('begin');
  Add('  c.{#c}Destroy; // free instance');
  Add('  c.{#d}DestroyOther; // free instance');
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestClass_Destructor_FreeInstance ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualFreeInstance:=false;
      ActualImplicitCallWithoutParams:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestClass_Destructor_FreeInstance ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if not (Ref.Declaration is TPasProcedure) then continue;
        //writeln('TTestResolver.TestClass_Destructor_FreeInstance ',GetObjName(Ref.Declaration),' rrfNewInstance=',rrfNewInstance in Ref.Flags);
        if (Ref.Declaration is TPasDestructor) then
          ActualFreeInstance:=rrfFreeInstance in Ref.Flags;
        ActualImplicitCallWithoutParams:=rrfImplicitCallWithoutParams in Ref.Flags;
        break;
        end;
      if not ActualImplicitCallWithoutParams then
        RaiseErrorAtSrcMarker('expected implicit call at "#'+aMarker^.Identifier+', but got function ref"',aMarker);
      case aMarker^.Identifier of
      'b':// should be normal call
        if ActualFreeInstance then
          RaiseErrorAtSrcMarker('expected normal call at "#'+aMarker^.Identifier+', but got freeinstance"',aMarker);
      else // should be freeinstance
        if not ActualFreeInstance then
          RaiseErrorAtSrcMarker('expected freeinstance at "#'+aMarker^.Identifier+', but got normal call"',aMarker);
      end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestClass_ConDestructor_CallInherited;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    constructor Create;');
  Add('    destructor Destroy; virtual;');
  Add('  end;');
  Add('  TChild = class(TObject)');
  Add('    constructor Create;');
  Add('    destructor Destroy; override;');
  Add('  end;');
  Add('constructor TObject.Create;');
  Add('begin');
  Add('end;');
  Add('destructor TObject.Destroy;');
  Add('begin');
  Add('end;');
  Add('constructor TChild.Create;');
  Add('begin');
  Add('  {#c}inherited; // normal call');
  Add('end;');
  Add('destructor TChild.Destroy;');
  Add('begin');
  Add('  {#d}inherited; // normal call');
  Add('end;');
  Add('begin');
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestClass_ConDestructor_Inherited ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestClass_ConDestructor_Inherited ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if not (Ref.Declaration is TPasProcedure) then continue;
        //writeln('TTestResolver.TestClass_ConDestructor_Inherited ',GetObjName(Ref.Declaration),' rrfNewInstance=',rrfNewInstance in Ref.Flags);
        if rrfNewInstance in Ref.Flags then
          RaiseErrorAtSrcMarker('expected normal call at "#'+aMarker^.Identifier+', but got newinstance"',aMarker);
        if rrfFreeInstance in Ref.Flags then
          RaiseErrorAtSrcMarker('expected normal call at "#'+aMarker^.Identifier+', but got freeinstance"',aMarker);
        break;
        end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestClass_Constructor_Inherited;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    constructor Create;');
  Add('    destructor Destroy;');
  Add('    procedure DoIt;');
  Add('  end;');
  Add('  {#TClassA}TClassA = class');
  Add('    Sub: TObject;');
  Add('  end;');
  Add('constructor TObject.Create; begin end;');
  Add('destructor TObject.Destroy; begin end;');
  Add('procedure TObject.DoIt; begin end;');
  Add('var a: TClassA;');
  Add('begin');
  Add('  a:=TClassA.Create;');
  Add('  a.DoIt;');
  Add('  a.Destroy;');
  Add('  if TClassA.Create.Sub=nil then ;');
  Add('  with TClassA.Create do Sub:=nil;');
  Add('  with TClassA do a:=Create;');
  Add('  with TClassA do Create.Sub:=nil;');
  ParseProgram;
end;

procedure TTestResolver.TestClass_SubObject;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    {#Sub}Sub: TObject;');
  Add('    procedure DoIt(p: longint);');
  Add('    function GetIt(p: longint): TObject;');
  Add('  end;');
  Add('procedure TObject.DoIt(p: longint); begin end;');
  Add('function TObject.GetIt(p: longint): TObject; begin end;');
  Add('var o: TObject;');
  Add('begin');
  Add('  o.Sub:=nil;');
  Add('  o.Sub.Sub:=nil;');
  Add('  if o.Sub=nil then ;');
  Add('  if o.Sub=o.Sub.Sub then ;');
  Add('  o.Sub.DoIt(3);');
  Add('  o.Sub.GetIt(4);');
  Add('  o.Sub.GetIt(5).DoIt(6);');
  Add('  o.Sub.GetIt(7).Sub.DoIt(8);');
  ParseProgram;
end;

procedure TTestResolver.TestClass_WithClassInstance;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualRefWith: Boolean;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FInt: longint;');
  Add('    FObj: TObject;');
  Add('    FArr: array of longint;');
  Add('    constructor Create;');
  Add('    function GetSize: longint;');
  Add('    procedure SetSize(Value: longint);');
  Add('    function GetItems(Index: longint): longint;');
  Add('    procedure SetItems(Index, Value: longint);');
  Add('    property Size: longint read GetSize write SetSize;');
  Add('    property Items[Index: longint]: longint read GetItems write SetItems;');
  Add('  end;');
  Add('constructor TObject.Create; begin end;');
  Add('function TObject.GetSize: longint; begin end;');
  Add('procedure TObject.SetSize(Value: longint); begin end;');
  Add('function TObject.GetItems(Index: longint): longint; begin end;');
  Add('procedure TObject.SetItems(Index, Value: longint); begin end;');
  Add('var');
  Add('  Obj: TObject;');
  Add('  i: longint;');
  Add('begin');
  Add('  with TObject.Create do begin');
  Add('    {#A}FInt:=3;');
  Add('    i:={#B}FInt;');
  Add('    i:={#C}GetSize;');
  Add('    i:={#D}GetSize();');
  Add('    {#E}SetSize(i);');
  Add('    i:={#F}Size;');
  Add('    {#G}Size:=i;');
  Add('    i:={#H}Items[i];');
  Add('    {#I}Items[i]:=i;');
  Add('    i:={#J}FArr[i];');
  Add('    {#K}FArr[i]:=i;');
  Add('  end;');
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestClass_WithClassInstance ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualRefWith:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestClass_WithClassInstance ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if Ref.WithExprScope=nil then continue;
        ActualRefWith:=true;
        break;
        end;
      if not ActualRefWith then
        RaiseErrorAtSrcMarker('expected Ref.WithExprScope<>nil at "#'+aMarker^.Identifier+', but got nil"',aMarker);
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestClass_ProcedureExternal;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure DoIt; external ''somewhere'';');
  Add('  end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_ReintroducePublicVarFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  public');
  Add('    Some: longint;');
  Add('  end;');
  Add('  TCar = class(tobject)');
  Add('  public');
  Add('    Some: longint;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Duplicate identifier "Some" at afile.pp(5,5)',nDuplicateIdentifier);
end;

procedure TTestResolver.TestClass_ReintroducePrivateVar;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  strict private');
  Add('    Some: longint;');
  Add('  end;');
  Add('  TCar = class(tobject)');
  Add('  public');
  Add('    Some: longint;');
  Add('  end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_ReintroduceProc;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  strict private');
  Add('    Some: longint;');
  Add('  end;');
  Add('  TMobile = class');
  Add('  strict private');
  Add('    Some: string;');
  Add('  end;');
  Add('  TCar = class(tmobile)');
  Add('    procedure {#A}Some;');
  Add('    procedure {#B}Some(vA: longint);');
  Add('  end;');
  Add('procedure tcar.some;');
  Add('begin');
  Add('  {@A}Some;');
  Add('  {@B}Some(1);');
  Add('end;');
  Add('procedure tcar.some(va: longint); begin end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_UntypedParam_TypeCast;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class end;');
  Add('procedure {#ProcA}ProcA(var {#A}A);');
  Add('begin');
  Add('  TObject({@A}A):=TObject({@A}A);');
  Add('  if TObject({@A}A)=nil then ;');
  Add('  if nil=TObject({@A}A) then ;');
  Add('end;');
  Add('procedure {#ProcB}ProcB(const {#B}B);');
  Add('begin');
  Add('  if TObject({@B}B)=nil then ;');
  Add('  if nil=TObject({@B}B) then ;');
  Add('end;');
  Add('var o: TObject;');
  Add('begin');
  Add('  {@ProcA}ProcA(o);');
  Add('  {@ProcB}ProcB(o);');
  ParseProgram;
end;

procedure TTestResolver.TestClass_Sealed;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class sealed');
  Add('  end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_SealedDescendFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class sealed');
  Add('  end;');
  Add('  TNop = class(TObject)');
  Add('  end;');
  Add('begin');
  CheckResolverException(sCannotCreateADescendantOfTheSealedXY,
    nCannotCreateADescendantOfTheSealedXY);
end;

procedure TTestResolver.TestClass_Abstract;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  '  TNop = class abstract(TObject)',
  '  end;',
  '  TBird = class(TNop)',
  '    constructor Create(w: word);',
  '  end;',
  'constructor TObject.Create;',
  'begin',
  'end;',
  'constructor TBird.Create(w: word);',
  'begin',
  '  inherited Create;',
  'end;',
  'begin',
  '  TBird.Create;']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClass_AbstractCreateFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  '  TNop = class abstract(TObject)',
  '  end;',
  'constructor TObject.Create;',
  'begin',
  'end;',
  'begin',
  '  TNop.Create;']);
  ParseProgram;
  CheckResolverHint(mtWarning,nCreatingAnInstanceOfAbstractClassY,
    'Creating an instance of abstract class "TNop"');
end;

procedure TTestResolver.TestClass_VarExternal;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('    Id: longint external name ''$Id'';');
  Add('    Data: longint external name ''$Data'';');
  Add('  end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_WarnOverrideLowerVisibility;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  strict protected');
  Add('    procedure DoStrictProtected; virtual; abstract;');
  Add('  protected');
  Add('    procedure DoProtected; virtual; abstract;');
  Add('  public');
  Add('    procedure DoPublic; virtual; abstract;');
  Add('  published');
  Add('    procedure DoPublished; virtual; abstract;');
  Add('  end;');
  Add('  TBird = class(TObject)');
  Add('  private');
  Add('    procedure DoStrictProtected; override;');
  Add('    procedure DoProtected; override;');
  Add('  protected');
  Add('    procedure DoPublic; override;');
  Add('    procedure DoPublished; override;');
  Add('  end;');
  Add('procedure TBird.DoStrictProtected; begin end;');
  Add('procedure TBird.DoProtected; begin end;');
  Add('procedure TBird.DoPublic; begin end;');
  Add('procedure TBird.DoPublished; begin end;');
  Add('begin');
  ParseProgram;
  CheckResolverHint(mtNote,nVirtualMethodXHasLowerVisibility,
    'Virtual method "DoStrictProtected" has a lower visibility (private) than parent class TObject (strict protected)');
  CheckResolverHint(mtNote,nVirtualMethodXHasLowerVisibility,
    'Virtual method "DoProtected" has a lower visibility (private) than parent class TObject (protected)');
  CheckResolverHint(mtNote,nVirtualMethodXHasLowerVisibility,
    'Virtual method "DoPublic" has a lower visibility (protected) than parent class TObject (public)');
  CheckResolverHint(mtNote,nVirtualMethodXHasLowerVisibility,
    'Virtual method "DoPublished" has a lower visibility (protected) than parent class TObject (published)');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClass_Const;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  TClass = class of TObject;',
  '  TObject = class',
  '  strict private const',
  '    Prefix = ''binary'';',
  '    PrefixLength = Length(Prefix);',
  '  public',
  '    const cI: integer = 3;',
  '    procedure DoIt;',
  '    class procedure DoMore;',
  '  end;',
  'procedure tobject.doit;',
  'begin',
  '  if cI=4 then;',
  '  if 5=cI then;',
  '  if Self.cI=6 then;',
  '  if 7=Self.cI then;',
  '  with Self do begin',
  '    if cI=11 then;',
  '    if 12=cI then;',
  '  end;',
  'end;',
  'class procedure tobject.domore;',
  'begin',
  '  if cI=8 then;',
  '  if Self.cI=9 then;',
  '  if 10=cI then;',
  '  if 11=Self.cI then;',
  '  with Self do begin',
  '    if cI=13 then;',
  '    if 14=cI then;',
  '  end;',
  'end;',
  'var',
  '  Obj: TObject;',
  '  Cla: TClass;',
  'begin',
  '  if TObject.cI=21 then ;',
  '  if Obj.cI=22 then ;',
  '  if Cla.cI=23 then ;',
  '  with obj do if ci=24 then;',
  '  with TObject do if ci=25 then;',
  '  with Cla do if ci=26 then;']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClass_ClassMissingVarFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class c: word;',
  '  end;',
  'begin']);
  CheckParserException('Expected "Procedure" or "Function"',nParserExpectToken2Error);
end;

procedure TTestResolver.TestClass_ClassConstFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class const c = 1;',
  '  end;',
  'begin']);
  CheckParserException(sParserExpectToken2Error,nParserExpectToken2Error);
end;

procedure TTestResolver.TestClass_Enumerator;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TItem = TObject;',
  '  TEnumerator = class',
  '    FCurrent: TItem;',
  '    property Current: TItem read FCurrent;',
  '    function MoveNext: boolean;',
  '  end;',
  '  TBird = class',
  '    function GetEnumerator: TEnumerator;',
  '  end;',
  'function TEnumerator.MoveNext: boolean;',
  'begin',
  'end;',
  'function TBird.GetEnumerator: TEnumerator;',
  'begin',
  'end;',
  'var',
  '  b: TBird;',
  '  i: TItem;',
  '  {#i2}i2: TItem;',
  'begin',
  '  for i in b do {@i2}i2:=i;']);
  ParseProgram;
end;

procedure TTestResolver.TestClass_EnumeratorFunc;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TItem = longint;',
  '  TEnumerator = class',
  '    FCurrent: TItem;',
  '    property Current: TItem read FCurrent;',
  '    function MoveNext: boolean;',
  '    function GetEnumerator: TEnumerator;',
  '  end;',
  'function TEnumerator.MoveNext: boolean;',
  'begin',
  'end;',
  'function TEnumerator.GetEnumerator: TEnumerator;',
  'begin',
  'end;',
  'function GetIt: TEnumerator;',
  'begin',
  'end;',
  'var',
  '  i, i2: TItem;',
  'begin',
  '  for i in GetIt do i2:=i;']);
  ParseProgram;
end;

procedure TTestResolver.TestClass_ForInPropertyStaticArray;
begin
  StartProgram(false);
  Add([
  'type',
  '  TMonthNameArray = array [1..12] of string;',
  '  TMonthNames = TMonthNameArray;',
  '  TObject = class',
  '  private',
  '    function GetLongMonthNames: TMonthNames; virtual; abstract;',
  '  public',
  '    Property LongMonthNames : TMonthNames Read GetLongMonthNames;',
  '  end;',
  'var f: TObject;',
  '  Month: string;',
  'begin',
  '  for Month in f.LongMonthNames do ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClass_TypeAlias;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TBird = type TObject;',
  'var',
  '  o: TObject;',
  '  b: TBird;',
  'begin',
  '  o:=b;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClass_Message;
begin
  StartProgram(false);
  Add([
  'const',
  '  FlyId = 2;',
  '  RunStr = ''Fast'';',
  'type',
  '  TObject = class',
  '    procedure Fly(var msg); message 3+FlyId;',
  '    procedure Run(var msg); virtual; abstract; message ''prefix''+RunStr;',
  '  end;',
  'procedure TObject.Fly(var msg);',
  'begin',
  'end;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClass_Message_MissingParamFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure Fly; message 3;',
  '  end;',
  'procedure TObject.Fly;',
  'begin',
  'end;',
  'begin',
  '']);
  CheckResolverException(sMessageHandlersInvalidParams,nMessageHandlersInvalidParams);
end;

procedure TTestResolver.TestClass_PublishedClassVarFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  published');
  Add('    class var Id: longint;');
  Add('  end;');
  Add('begin');
  CheckResolverException(sSymbolCannotBePublished,nSymbolCannotBePublished);
end;

procedure TTestResolver.TestClass_PublishedClassPropertyFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class var FA: longint;');
  Add('  published');
  Add('    class property A: longint read FA;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Invalid published property modifier "class"',
    nInvalidXModifierY);
end;

procedure TTestResolver.TestClass_PublishedClassFunctionFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  published');
  Add('    class procedure DoIt;');
  Add('  end;');
  Add('class procedure TObject.DoIt; begin end;');
  Add('begin');
  CheckResolverException(sSymbolCannotBePublished,nSymbolCannotBePublished);
end;

procedure TTestResolver.TestClass_PublishedOverloadFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  published');
  Add('    procedure DoIt;');
  Add('    procedure DoIt(i: longint);');
  Add('  end;');
  Add('procedure TObject.DoIt; begin end;');
  Add('procedure TObject.DoIt(i: longint); begin end;');
  Add('begin');
  CheckResolverException(sDuplicatePublishedMethodXAtY,nDuplicatePublishedMethodXAtY);
end;

procedure TTestResolver.TestNestedClass;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TBear = class',
  '  type',
  '    TNumber = byte;',
  '    TLeg = class',
  '      constructor Create(i: TNumber);',
  '      function {#Walk}Walk(i: TNumber): TLeg;',
  '    end;',
  '    procedure Move(i: TNumber);',
  '  end;',
  'procedure TBear.Move(i: TNumber);',
  'var Leg: TLeg;',
  'begin',
  '  Leg:=TLeg.Create(i);',
  '  Leg:=TBear.TLeg.Create(i);',
  'end;',
  'constructor tBear.tLeg.Create(i: TNumber);',
  'begin',
  '  {@Walk}Walk(i);',
  '  Self.{@Walk}Walk(i);',
  'end;',
  'function tBear.tLeg.walk(i: TNumber): TLeg;',
  'begin',
  '  Result:=Walk(3);',
  'end;',
  'var Leg: TBear.TLeg;',
  'begin',
  '  Leg:=TBear.TLeg.Create(2);',
  '  Leg:=Leg.Walk(3);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestNestedClass_Forward;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  type',
  '    TArm = class;',
  '    TLeg = class',
  '      procedure Send(Arm: TArm);',
  '    end;',
  '    TArm = class',
  '      i: byte;',
  '    end;',
  '  end;',
  'procedure tObject.tLeg.send(Arm: TArm);',
  'begin',
  '  Arm.i:=3;',
  'end;',
  'var',
  '  Leg: TObject.TLeg;',
  '  Arm: TObject.TArm;',
  'begin',
  '  Leg.Send(Arm);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestNestedClass_StrictPrivateFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  strict private type',
  '    TArm = class',
  '      i: byte;',
  '    end;',
  '  end;',
  'var',
  '  Arm: TObject.TArm;',
  'begin',
  '']);
  CheckResolverException('Can''t access strict private member TArm',nCantAccessXMember);
end;

procedure TTestResolver.TestNestedClass_AccessStrictPrivate;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  public type',
  '    TWing = class',
  '      procedure Fly;',
  '    end;',
  '  strict private',
  '    class var i: longint;',
  '  end;',
  'procedure TObject.TWing.Fly;',
  'begin',
  '  i:=3;',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestNestedClass_AccessParent;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TObject = class',
  '  end;',
  '  TLimb = class',
  '    {#tlimb_d}d: longint;',
  '  end;',
  '  TAnt = boolean;',
  '  TBird = class',
  '  public type',
  '    TBody = class',
  '    public type',
  '      TAnt = word;',
  '      TWing = class(TLimb)',
  '        {#ant}ant: TAnt;',
  '        procedure Fly(i: longint);',
  '      end;',
  '    public',
  '      class var {#tbody_a}a, {#tbody_b}b, {#tbody_d}d, {#tbody_e}e: longint;',
  '    end;',
  '  public',
  '    class var {#tbird_a}a, {#tbird_b}b, {#tbird_c}c, {#tbird_d}d, {#tbird_e}e: longint;',
  '  end;',
  'var {#intf_a}a, {#intf_d}d: longint;',
  'implementation',
  'var {#impl_e}e: longint;',
  'procedure TBird.TBody.TWing.Fly(i: longint);',
  'begin',
  '  {@ant}ant:=2;',
  '  {@intf_a}a:=3;',
  '  {@tbody_b}b:=4;',
  '  {@tbird_c}c:=5;',
  '  {@tlimb_d}d:=6;',
  '  {@impl_e}e:=7;',
  'end;',
  '']);
  ParseUnit;
end;

procedure TTestResolver.TestNestedClass_BodyAccessParentVarFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TBird = class',
  '  public type',
  '    TWing = class',
  '      procedure Fly;',
  '    end;',
  '  public',
  '    var i: longint;',
  '  end;',
  'procedure TBird.TWing.Fly;',
  'begin',
  '  i:=3;',
  'end;',
  'begin']);
  CheckResolverException('Instance member "i" inaccessible here',nInstanceMemberXInaccessible);
end;

procedure TTestResolver.TestNestedClass_PropertyAccessParentVarFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TBird = class',
  '    fSize: word;',
  '  public type',
  '    TWing = class',
  '      property Size: word read fSize;',
  '    end;',
  '  end;',
  'begin']);
  CheckResolverException('identifier not found "fSize"',nIdentifierNotFound);
end;

procedure TTestResolver.TestExternalClass;
begin
  StartProgram(false);
  Add('type');
  Add('{$modeswitch externalclass}');
  Add('  TExtA = class external ''namespace'' name ''symbol''');
  Add('    Id: longint;');
  Add('  end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestExternalClass_Descendant;
begin
  StartProgram(false);
  Add('type');
  Add('{$modeswitch externalclass}');
  Add('  TExtA = class external ''namespace'' name ''symbol''');
  Add('    Id: longint;');
  Add('  end;');
  Add('  TExtB = class external ''namespace'' name ''symbol''(TExtA)');
  Add('  end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestExternalClass_HintMethodHidesNonVirtualMethodExact;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSObject = class external name ''JSObject''',
  '    procedure DoIt(p: pointer);',
  '  end;',
  '  TBird = class external name ''Bird''(TJSObject)',
  '    procedure DoIt(p: pointer);',
  '  end;',
  'procedure TJSObject.DoIt(p: pointer);',
  'begin',
  '  if p=nil then ;',
  'end;',
  'procedure TBird.DoIt(p: pointer); begin end;',
  'var b: TBird;',
  'begin',
  '  b.DoIt(nil);']);
  ParseProgram;
  CheckResolverHint(mtHint,nMethodHidesNonVirtualMethodExactly,
   'method hides identifier at "afile.pp(5,19)". Use reintroduce');
end;

procedure TTestResolver.TestClassOf;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TClass}{=TObj}TClass = class of TObject;',
  '  {#TOBJ}TObject = class',
  '    ClassType: TClass; ',
  '  end;',
  'type',
  '  {#TMobile}TMobile = class',
  '  end;',
  '  {#TMobiles}{=TMobile}TMobiles = class of TMobile;',
  'type',
  '  {#TCars}{=TCar}TCars = class of TCar;',
  '  {#TShips}{=TShip}TShips = class of TShip;',
  '  {#TCar}TCar = class(TMobile)',
  '  end;',
  '  {#TShip}TShip = class(TMobile)',
  '  end;',
  'var',
  '  o: TObject;',
  '  c: TClass;',
  '  mobile: TMobile;',
  '  mobiletype: TMobiles;',
  '  car: TCar;',
  '  cartype: TCars;',
  '  ship: TShip;',
  '  shiptype: TShips;',
  '  p: pointer;',
  'begin',
  '  c:=nil;',
  '  c:=o.ClassType;',
  '  if c=nil then;',
  '  if nil=c then;',
  '  if c=o.ClassType then ;',
  '  if c<>o.ClassType then ;',
  '  if Assigned(o) then ;',
  '  if Assigned(o.ClassType) then ;',
  '  if Assigned(c) then ;',
  '  mobiletype:=TMobile;',
  '  mobiletype:=TCar;',
  '  mobiletype:=TShip;',
  '  mobiletype:=cartype;',
  '  if mobiletype=nil then ;',
  '  if nil=mobiletype then ;',
  '  if mobiletype=TShip then ;',
  '  if TShip=mobiletype then ;',
  '  if mobiletype<>TShip then ;',
  '  if mobile is mobiletype then ;',
  '  if car is mobiletype then ;',
  '  if mobile is cartype then ;',
  '  p:=c;',
  '  if p=c then ;',
  '  if c=p then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassOfAlias;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TBird = TObject;',
  '  TBirds = class of TBird;',
  '  TEagles = TBirds;',
  'var',
  '  o: TBird;',
  '  c: TEagles;',
  'begin',
  '  c:=TObject;',
  '  c:=TBird;',
  '  if c=TObject then ;',
  '  if c=TBird then ;',
  '  if o is c then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassOfNonClassFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TCars = class of longint;');
  Add('begin');
  CheckResolverException('Incompatible types: got "Longint" expected "class"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestClassOfAssignClassOfFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TClass = class of TObject;',
  'var c: TClass;',
  'begin',
  '  c:=TClass;']);
  CheckResolverException('Incompatible types: got "type class-of" expected "class of TObject"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestClassOfIsOperatorFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class end;');
  Add('  TCar = class end;');
  Add('  TCars = class of TCar;');
  Add('var cars: TCars;');
  Add('begin');
  Add('  if cars is TCars then ;');
  CheckResolverException('left side of is-operator expects a class, but got "class of"',
    nLeftSideOfIsOperatorExpectsAClassButGot);
end;

procedure TTestResolver.TestClassOfAsOperatorFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class end;');
  Add('  TCar = class end;');
  Add('  TCars = class of TCar;');
  Add('var');
  Add('  o: TObject;');
  Add('  cars: TCars;');
  Add('begin');
  Add('  cars:=cars as TCars;');
  CheckResolverException('Operator is not overloaded: "TCars" as "class of TCars"',
    nOperatorIsNotOverloadedAOpB);
end;

procedure TTestResolver.TestClassOfIsOperator;
begin
  StartProgram(false);
  ResolverEngine.Options:=ResolverEngine.Options+[proClassOfIs];
  Add('type');
  Add('  TObject = class end;');
  Add('  TClass = class of TObject;');
  Add('  TCar = class end;');
  Add('  TCars = class of TCar;');
  Add('var C: TClass;');
  Add('  D: TCars;');
  Add('begin');
  Add('  if C is TCar then;');
  Add('  if C is TCars then;');
  Add('  if C is D then ;');
  ParseProgram;
end;

procedure TTestResolver.TestClass_ClassVar;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class var GlobalId: longint;');
  Add('  end;');
  Add('  TObjectClass = class of TObject;');
  Add('var');
  Add('  o: TObject;');
  Add('  oc: TObjectClass;');
  Add('begin');
  Add('  o.GlobalId:=3;');
  Add('  if o.GlobalId=4 then ;');
  Add('  if 5=o.GlobalId then ;');
  Add('  TObject.GlobalId:=6;');
  Add('  if TObject.GlobalId=7 then ;');
  Add('  if 8=TObject.GlobalId then ;');
  Add('  oc.GlobalId:=9;');
  Add('  if oc.GlobalId=10 then ;');
  Add('  if 11=oc.GlobalId then ;');
  ParseProgram;
end;

procedure TTestResolver.TestClassOfDotClassVar;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class var Id: longint;');
  Add('  end;');
  Add('  TObjectClass = class of TObject;');
  Add('var');
  Add('  oc: TObjectClass;');
  Add('begin');
  Add('  oc.Id:=3;');
  Add('  if oc.Id=4 then ;');
  Add('  if 5=oc.Id then ;');
  Add('  TObject.Id:=3;');
  Add('  if TObject.Id=4 then ;');
  Add('  if 5=TObject.Id then ;');
  ParseProgram;
end;

procedure TTestResolver.TestClassOfDotVarFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    Id: longint;');
  Add('  end;');
  Add('  TObjectClass = class of TObject;');
  Add('var');
  Add('  oc: TObjectClass;');
  Add('begin');
  Add('  oc.Id:=3;');
  CheckResolverException(sInstanceMemberXInaccessible,
    nInstanceMemberXInaccessible);
end;

procedure TTestResolver.TestClassOfDotClassProc;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class procedure ProcA;');
  Add('    class function FuncB: longint;');
  Add('    class procedure ProcC(i: longint);');
  Add('    class function FuncD(i: longint): longint;');
  Add('  end;');
  Add('  TObjectClass = class of TObject;');
  Add('class procedure TObject.ProcA; begin end;');
  Add('class function TObject.FuncB: longint; begin end;');
  Add('class procedure TObject.ProcC(i: longint); begin end;');
  Add('class function TObject.FuncD(i: longint): longint; begin end;');
  Add('var');
  Add('  o: TObject;');
  Add('  oc: TObjectClass;');
  Add('begin');
  Add('  o.ProcA;');
  Add('  oc.ProcA;');
  Add('  TObject.ProcA;');
  Add('  o.FuncB;');
  Add('  o.FuncB();');
  Add('  oc.FuncB;');
  Add('  oc.FuncB();');
  Add('  TObject.FuncB;');
  Add('  TObject.FuncB();');
  Add('  if oc.FuncB=3 then ;');
  Add('  if oc.FuncB()=4 then ;');
  Add('  if 5=oc.FuncB then ;');
  Add('  if 6=oc.FuncB() then ;');
  Add('  oc.ProcC(7);');
  Add('  TObject.ProcC(8);');
  Add('  oc.FuncD(7);');
  Add('  TObject.FuncD(8);');
  Add('  if oc.FuncD(9)=10 then ;');
  Add('  if 11=oc.FuncD(12) then ;');
  Add('  if TObject.FuncD(13)=14 then ;');
  Add('  if 15=TObject.FuncD(16) then ;');
  ParseProgram;
end;

procedure TTestResolver.TestClassOfDotProcFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure ProcA;');
  Add('  end;');
  Add('  TObjectClass = class of TObject;');
  Add('procedure TObject.ProcA; begin end;');
  Add('var');
  Add('  oc: TObjectClass;');
  Add('begin');
  Add('  oc.ProcA;');
  CheckResolverException(sInstanceMemberXInaccessible,
    nInstanceMemberXInaccessible);
end;

procedure TTestResolver.TestClassOfDotClassProperty;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class var FA: longint;');
  Add('    class function GetA: longint; static;');
  Add('    class procedure SetA(Value: longint); static;');
  Add('    class property A1: longint read FA write SetA;');
  Add('    class property A2: longint read GetA write FA;');
  Add('  end;');
  Add('  TObjectClass = class of TObject;');
  Add('class function TObject.GetA: longint; begin end;');
  Add('class procedure TObject.SetA(Value: longint); begin end;');
  Add('var');
  Add('  o: TObject;');
  Add('  oc: TObjectClass;');
  Add('begin');
  Add('  o.A1:=3;');
  Add('  if o.A1=4 then ;');
  Add('  if 5=o.A1 then ;');
  Add('  oc.A1:=6;');
  Add('  if oc.A1=7 then ;');
  Add('  if 8=oc.A1 then ;');
  Add('  TObject.A1:=9;');
  Add('  if TObject.A1=10 then ;');
  Add('  if 11=TObject.A1 then ;');
  ParseProgram;
end;

procedure TTestResolver.TestClassOfDotPropertyFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FA: longint;');
  Add('    property A: longint read FA;');
  Add('  end;');
  Add('  TObjectClass = class of TObject;');
  Add('var');
  Add('  oc: TObjectClass;');
  Add('begin');
  Add('  if oc.A=3 then ;');
  CheckResolverException(sInstanceMemberXInaccessible,
    nInstanceMemberXInaccessible);
end;

procedure TTestResolver.TestClass_ClassProcSelf;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class var GlobalId: longint;');
  Add('    class procedure ProcA;');
  Add('  end;');
  Add('  TClass = class of TObject;');
  Add('class procedure TObject.ProcA;');
  Add('var c: TClass;');
  Add('begin');
  Add('  if Self=nil then ;');
  Add('  if Self.GlobalId=3 then ;');
  Add('  if 4=Self.GlobalId then ;');
  Add('  Self.GlobalId:=5;');
  Add('  c:=Self;');
  Add('  c:=TClass(Self);');
  Add('  if Self=c then ;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClass_ClassProcSelfTypeCastFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class procedure ProcA;');
  Add('  end;');
  Add('class procedure TObject.ProcA;');
  Add('begin');
  Add('  if TObject(Self)=nil then ;');
  Add('end;');
  Add('begin');
  CheckResolverException('Illegal type conversion: "Self" to "class TObject"',
    nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestClass_ClassMembers;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  TMobile = class');
  Add('  public');
  Add('    MobileId: longint;');
  Add('    class var LastVal: longint;');
  Add('    constructor Create; virtual;');
  Add('    class procedure ClProcA;');
  Add('    class function ClFuncB: longint;');
  Add('    class function StFuncC: longint; static;');
  Add('    class property ClMobileId: longint read StFuncC write LastVal;');
  Add('  end;');
  Add('  TMobiles = class of TMobile;');
  Add('  TCars = class of TCar;');
  Add('  TCar = class(TMobile)');
  Add('  public');
  Add('    CarId: longint;');
  Add('    class var LastCarVal: longint;');
  Add('    constructor Create; override;');
  Add('  end;');
  Add('constructor TMobile.Create;');
  Add('begin');
  Add('  Self.MobileId:=7;');
  Add('  LastVal:=LastVal+ClMobileId+1;');
  Add('  ClMobileId:=MobileId+3;');
  Add('  TCar(Self).CarId:=4;');
  Add('end;');
  Add('class procedure TMobile.ClProcA;');
  Add('var');
  Add('  m: TMobiles;');
  Add('begin');
  Add('  LastVal:=9;');
  Add('  Self.LastVal:=ClFuncB+ClMobileId;');
  Add('  m:=Self;');
  Add('  if m=Self then ;');
  Add('end;');
  Add('class function TMobile.ClFuncB: longint;');
  Add('begin');
  Add('  if LastVal=3 then ;');
  Add('  Result:=Self.LastVal-ClMobileId;');
  Add('end;');
  Add('class function TMobile.StFuncC: longint;');
  Add('begin');
  Add('  Result:=LastVal;');
  Add('  // Forbidden: no Self in static methods');
  Add('end;');
  Add('');
  Add('constructor TCar.Create;');
  Add('begin');
  Add('  inherited Create;');
  Add('  Self.CarId:=8;');
  Add('  TMobile(Self).LastVal:=5;');
  Add('  if TMobile(Self).LastVal=25 then ;');
  Add('end;');
  Add('');
  Add('var');
  Add('  car: TCar;');
  Add('  cartype: TCars;');
  Add('begin');
  Add('  car:=TCar.Create;');
  Add('  car.MobileId:=10;');
  Add('  car.ClProcA;');
  Add('  exit;');
  Add('  car.ClMobileId:=11;');
  Add('  if car.ClFuncB=16 then ;');
  Add('  if 17=car.ClFuncB then ;');
  Add('  cartype:=TCar;');
  Add('  cartype.LastVal:=18;');
  Add('  if cartype.LastVal=19 then ;');
  Add('  if 20=cartype.LastVal then ;');
  ParseProgram;
end;

procedure TTestResolver.TestClassOf_AsFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TClass = class of TObject;');
  Add('  TObject = class');
  Add('  end;');
  Add('var');
  Add('  c: tclass;');
  Add('begin');
  Add('  c:=c as TClass;');
  CheckResolverException('Operator is not overloaded: "TClass" as "class of TClass"',
    nOperatorIsNotOverloadedAOpB);
end;

procedure TTestResolver.TestClassOf_MemberAsFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TClass = class of TObject;');
  Add('  TObject = class');
  Add('    c: tclass;');
  Add('  end;');
  Add('var o: TObject;');
  Add('begin');
  Add('  o.c:=o.c as TClass;');
  CheckResolverException('Operator is not overloaded: "TClass" as "class of TClass"',nOperatorIsNotOverloadedAOpB);
end;

procedure TTestResolver.TestClassOf_IsFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TClass = class of TObject;');
  Add('  TObject = class');
  Add('  end;');
  Add('var');
  Add('  c: tclass;');
  Add('begin');
  Add('  if c is TObject then;');
  CheckResolverException('left side of is-operator expects a class, but got "class of"',
    nLeftSideOfIsOperatorExpectsAClassButGot);
end;

procedure TTestResolver.TestClass_TypeCast;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class procedure {#TObject_DoIt}DoIt;');
  Add('  end;');
  Add('  TClass = class of TObject;');
  Add('  TMobile = class');
  Add('    class procedure {#TMobile_DoIt}DoIt;');
  Add('  end;');
  Add('  TMobileClass = class of TMobile;');
  Add('  TCar = class(TMobile)');
  Add('    class procedure {#TCar_DoIt}DoIt;');
  Add('  end;');
  Add('  TCarClass = class of TCar;');
  Add('class procedure TObject.DoIt;');
  Add('begin');
  Add('  TClass(Self).{@TObject_DoIt}DoIt;');
  Add('  TMobileClass(Self).{@TMobile_DoIt}DoIt;');
  Add('end;');
  Add('class procedure TMobile.DoIt;');
  Add('begin');
  Add('  TClass(Self).{@TObject_DoIt}DoIt;');
  Add('  TMobileClass(Self).{@TMobile_DoIt}DoIt;');
  Add('  TCarClass(Self).{@TCar_DoIt}DoIt;');
  Add('end;');
  Add('class procedure TCar.DoIt; begin end;');
  Add('var');
  Add('  ObjC: TClass;');
  Add('  MobileC: TMobileClass;');
  Add('  CarC: TCarClass;');
  Add('begin');
  Add('  ObjC.{@TObject_DoIt}DoIt;');
  Add('  MobileC.{@TMobile_DoIt}DoIt;');
  Add('  CarC.{@TCar_DoIt}DoIt;');
  Add('  TClass(ObjC).{@TObject_DoIt}DoIt;');
  Add('  TMobileClass(ObjC).{@TMobile_DoIt}DoIt;');
  Add('  TCarClass(ObjC).{@TCar_DoIt}DoIt;');
  Add('  TClass(MobileC).{@TObject_DoIt}DoIt;');
  Add('  TMobileClass(MobileC).{@TMobile_DoIt}DoIt;');
  Add('  TCarClass(MobileC).{@TCar_DoIt}DoIt;');
  Add('  TClass(CarC).{@TObject_DoIt}DoIt;');
  Add('  TMobileClass(CarC).{@TMobile_DoIt}DoIt;');
  Add('  TCarClass(CarC).{@TCar_DoIt}DoIt;');
  ParseProgram;
end;

procedure TTestResolver.TestClassOf_AlwaysForward;
begin
  AddModuleWithIntfImplSrc('unit2.pp',
    LinesToStr([
    'type',
    '  TObject = class',
    '  end;',
    '  TCar = class',
    '  end;',
    '  TCarry = TCar;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add('uses unit2;');
  Add('type');
  Add('  {#C}{=A}TCars = class of TCarry;');
  Add('  {#A}TCarry = class');
  Add('    class var {#B}B: longint;');
  Add('  end;');
  Add('begin');
  Add('  {@C}TCars.{@B}B:=3;');
  ParseProgram;
end;

procedure TTestResolver.TestClassOf_ClassOfBeforeClass_FuncResult;
begin
  StartProgram(false);
  Add('type');
  Add('  TClass = class of TObject;');
  Add('  TObject = class');
  Add('  end;');
  Add('function GetClass: TClass;');
  Add('begin');
  Add('  Result:=TObject;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestClassOf_Const;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TBird = TObject;',
  '  TBirds = class of TBird;',
  '  TEagles = TBirds;',
  '  THawk = class(TBird);',
  'const',
  '  Hawk: TEagles = THawk;',
  '  DefaultBirdClasses : Array [1..2] of TEagles = (',
  '    TBird,',
  '    THawk',
  '  );',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassOf_Const2;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TFieldType = (fta,ftb);',
  '  TField = Class;',
  '  TFieldClass = class of TField;',
  '  TField = Class(TObject);',
  '  TFieldA = Class(TField);',
  '  TFieldB = Class(TField);',
  'Const',
  '  DefaultFieldClasses : Array [TFieldType] of TFieldClass = (TFieldA,TFieldB);',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestProperty1;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  {#TOBJ}TObject = class');
  Add('  end;');
  Add('  {#A}TClassA = class');
  Add('    {#FB}FB: integer;');
  Add('    property {#B}B: longint read {@FB}FB write {@FB}FB;');
  Add('  end;');
  Add('var');
  Add('  {#v}{=A}v: TClassA;');
  Add('begin');
  Add('  {@v}v.{@b}b:=3;');
  ParseProgram;
end;

procedure TTestResolver.TestPropertyAccessorNotInFront;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    property B: longint read FB;');
  Add('    FB: longint;');
  Add('  end;');
  Add('begin');
  CheckResolverException('identifier not found "FB"',nIdentifierNotFound);
end;

procedure TTestResolver.TestPropertyReadAndWriteMissingFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    property B: longint;',
  '  end;',
  'begin']);
  CheckResolverException(sPropertyMustHaveReadOrWrite,nPropertyMustHaveReadOrWrite);
end;

procedure TTestResolver.TestPropertyReadAccessorVarWrongType;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FB: string;');
  Add('    property B: longint read FB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Incompatible types: got "Longint" expected "String"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestPropertyReadAccessorProcNotFunc;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure GetB;');
  Add('    property B: longint read GetB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('function expected, but procedure found',nXExpectedButYFound);
end;

procedure TTestResolver.TestPropertyReadAccessorFuncWrongResult;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    function GetB: string;');
  Add('    property B: longint read GetB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('function result Longint expected, but String found',
    nXExpectedButYFound);
end;

procedure TTestResolver.TestPropertyReadAccessorFuncWrongArgCount;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    function GetB(i: longint): longint;');
  Add('    property B: longint read GetB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Wrong number of parameters specified for call to "GetB"',
    nWrongNumberOfParametersForCallTo);
end;

procedure TTestResolver.TestPropertyReadAccessorFunc;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    function {#GetB}GetB: longint;');
  Add('    property {#B}B: longint read {@GetB}GetB;');
  Add('  end;');
  Add('function TObject.GetB: longint;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('begin');
  Add('  if {@o}o.{@B}B=3 then ;');
  ParseProgram;
end;

procedure TTestResolver.TestPropertyReadAccessorStrictPrivate;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  strict private',
  '    FSize: word;',
  '    property Size: word read FSize;',
  '  strict protected',
  '    FName: string;',
  '    property Name: string read FName;',
  '  end;',
  '  TBird = class',
  '  strict protected',
  '    property Caption: string read FName;',
  '  end;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestPropertyReadAccessorNonClassFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    FSize: word;',
  '    class property Size: word read FSize;',
  '  end;',
  'begin',
  '']);
  CheckResolverException('class var expected, but var found',nXExpectedButYFound);
end;

procedure TTestResolver.TestPropertyWriteAccessorVarWrongType;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FB: string;');
  Add('    property B: longint write FB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Incompatible types: got "Longint" expected "String"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestPropertyWriteAccessorFuncNotProc;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    function SetB: longint;');
  Add('    property B: longint write SetB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('procedure expected, but function found',nXExpectedButYFound);
end;

procedure TTestResolver.TestPropertyWriteAccessorProcWrongArgCount;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure SetB;');
  Add('    property B: longint write SetB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Wrong number of parameters specified for call to "SetB"',
    nWrongNumberOfParametersForCallTo);
end;

procedure TTestResolver.TestPropertyWriteAccessorProcWrongArg;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure SetB(var Value: longint);');
  Add('    property B: longint write SetB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Incompatible type arg no. 1: Got "var", expected "const"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestPropertyWriteAccessorProcWrongArgType;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure SetB(Value: string);');
  Add('    property B: longint write SetB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Incompatible type arg no. 1: Got "String", expected "Longint"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestPropertyWriteAccessorProc;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    procedure {#SetB}SetB(Value: longint);');
  Add('    property {#B}B: longint write {@SetB}SetB;');
  Add('  end;');
  Add('procedure TObject.SetB(Value: longint);');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('begin');
  Add('  {@o}o.{@B}B:=3;');
  ParseProgram;
end;

procedure TTestResolver.TestPropertyTypeless;
begin
  StartProgram(false);
  Add([
  'type',
  '  {#TOBJ}TObject = class',
  '    {#FB}FB: longint;',
  '    property {#TOBJ_B}B: longint write {@FB}FB;',
  '    property {#TOBJ_D}D: longint write {@FB}FB;',
  '  end;',
  '  {#TA}TClassA = class',
  '    {#FC}FC: longint;',
  '    property {#TA_B}{@TOBJ_B}B write {@FC}FC;',
  '  end;',
  '  {#TB}TClassB = class(TClassA)',
  '  published',
  '    property {#TB_D}{@TOBJ_D}D;',
  '  end;',
  'var',
  '  {#v}{=TA}v: TClassA;',
  'begin',
  '  {@v}v.{@TA_B}B:=3;',
  '  {@v}v.{@TObj_D}D:=4;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestPropertyTypelessNoAncestorFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  TClassA = class');
  Add('    property B;');
  Add('  end;');
  Add('begin');
  CheckResolverException(sNoPropertyFoundToOverride,
    nNoPropertyFoundToOverride);
end;

procedure TTestResolver.TestPropertyStoredAccessor;
begin
  StartProgram(false);
  Add('const StoreB = true;');
  Add('type');
  Add('  TObject = class');
  Add('    FBird: longint;');
  Add('    VStored: boolean;');
  Add('    function IsBirdStored: boolean; virtual; abstract;');
  Add('    property Bird: longint read FBird stored VStored;');
  Add('    property B: longint read FBird stored IsBirdStored;');
  Add('    property Eagle: longint read FBird stored StoreB;');
  Add('    property Hawk: longint read FBird stored false;');
  Add('  end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestPropertyStoredAccessorVarWrongType;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FB: longint;');
  Add('    BStored: longint;');
  Add('    property B: longint read FB stored BStored;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Incompatible types: got "Longint" expected "Boolean"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestPropertyStoredAccessorProcNotFunc;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FB: longint;');
  Add('    procedure GetB;');
  Add('    property B: longint read FB stored GetB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('function expected, but procedure found',nXExpectedButYFound);
end;

procedure TTestResolver.TestPropertyStoredAccessorFuncWrongResult;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FB: longint;');
  Add('    function GetB: string;');
  Add('    property B: longint read FB stored GetB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('function: boolean expected, but function:String found',
    nXExpectedButYFound);
end;

procedure TTestResolver.TestPropertyStoredAccessorFuncWrongArgCount;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FB: longint;');
  Add('    function GetB(i: longint): boolean;');
  Add('    property B: longint read FB stored GetB;');
  Add('  end;');
  Add('begin');
  CheckResolverException('Wrong number of parameters specified for call to "GetB"',
    nWrongNumberOfParametersForCallTo);
end;

procedure TTestResolver.TestPropertyIndexSpec;
begin
  StartProgram(false);
  Add([
  'const',
  '  CB = true or false;',
  '  CI = 1+2;',
  'type',
  '  TEnum = (red, blue);',
  '  TObject = class',
  '    FB: boolean;',
  '    function GetIntBool(Index: longint): boolean; virtual; abstract;',
  '    procedure SetIntBool(Index: longint; b: boolean); virtual; abstract;',
  '    function GetBoolBool(Index: boolean): boolean; virtual; abstract;',
  '    procedure SetBoolBool(Index: boolean; b: boolean); virtual; abstract;',
  '    function GetEnumBool(Index: TEnum): boolean; virtual; abstract;',
  '    procedure SetEnumBool(Index: TEnum; b: boolean); virtual; abstract;',
  '    function GetStrIntBool(A: String; I: longint): boolean; virtual; abstract;',
  '    procedure SetStrIntBool(A: String; I: longint; b: boolean); virtual; abstract;',
  '    property B1: boolean index 1 read GetIntBool write SetIntBool stored GetIntBool;',
  '    property B2: boolean index CI read GetIntBool write SetIntBool stored GetIntBool;',
  '    property B3: boolean index false read GetBoolBool write SetBoolBool stored GetBoolBool;',
  '    property B4: boolean index CB read GetBoolBool write SetBoolBool stored GetBoolBool;',
  '    property B5: boolean index red read GetEnumBool write SetEnumBool stored GetEnumBool;',
  '    property B6: boolean index TEnum.blue read GetEnumBool write SetEnumBool stored GetEnumBool;',
  '    property B7: boolean index 1 read GetIntBool write FB stored FB;',
  '    property I1[A: String]: boolean index 2 read GetStrIntBool write SetStrIntBool;',
  '  end;',
  '  TBird = class',
  '    function GetIntBoolOvr(Index: longint): boolean; virtual; abstract;',
  '    property B1 index 3;',
  '    property B2 read GetIntBoolOvr;',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestPropertyIndexSpec_ReadAccessorWrongArgCount;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function GetB: boolean; virtual; abstract;',
  '    property B: boolean index 1 read GetB;',
  '  end;',
  'begin']);
  CheckResolverException('Wrong number of parameters specified for call to "GetB"',
    nWrongNumberOfParametersForCallTo);
end;

procedure TTestResolver.TestPropertyIndexSpec_ReadAccessorWrongIndexArgType;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function GetB(S: string): boolean; virtual; abstract;',
  '    property B: boolean index 1 read GetB;',
  '  end;',
  'begin']);
  CheckResolverException('Incompatible type arg no. 1: Got "Longint", expected "String"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestPropertyDefaultValue;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red, blue, green, white, grey, black);',
  '  TEnumRg = blue..grey;',
  '  TSet = set of TEnum;',
  'const',
  '  CB = true or false;',
  '  CI = 1+2;',
  '  CS = [red,blue];',
  'type',
  '  TObject = class',
  '    FB: boolean;',
  '    property B1: boolean read FB default true;',
  '    property B2: boolean read FB default CB;',
  '    property B3: boolean read FB default afile.cb;',
  '    FI: longint;',
  '    property I1: longint read FI default 2;',
  '    property I2: longint read FI default CI;',
  '    FE: TEnum;',
  '    property E1: TEnum read FE default red;',
  '    property E2: TEnum read FE default TEnum.blue;',
  '    FEnumRg: TEnumRg;',
  '    property EnumRg1: TEnumRg read FEnumRg default white;',
  '    FSet: TSet;',
  '    property Set1: TSet read FSet default [];',
  '    property Set2: TSet read FSet default [red];',
  '    property Set3: TSet read FSet default [red,blue];',
  '    property Set4: TSet read FSet default CS;',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestPropertyArgs1;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    function GetB(Index: longint): boolean;');
  Add('    procedure SetB(Index: longint; Value: boolean);');
  Add('    property B[Index: longint]: boolean read GetB write SetB;');
  Add('  end;');
  Add('function TObject.GetB(Index: longint): boolean;');
  Add('begin');
  Add('end;');
  Add('procedure TObject.SetB(Index: longint; Value: boolean);');
  Add('begin');
  Add('end;');
  Add('var o: TObject;');
  Add('begin');
  Add('  o.B[3]:=true;');
  Add('  if o.B[4] then;');
  Add('  if o.B[5]=true then;');
  Add('  if false=o.B[6] then;');
  ParseProgram;
end;

procedure TTestResolver.TestPropertyArgs2;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    function GetB(Index: longint; const ID: string): longint;');
  Add('    procedure SetB(Index: longint; const ID: string; Value: longint);');
  Add('    property B[Index: longint; const ID: string]: longint read GetB write SetB;');
  Add('  end;');
  Add('function TObject.GetB(Index: longint; const ID: string): longint;');
  Add('begin');
  Add('end;');
  Add('procedure TObject.SetB(Index: longint; const ID: string; Value: longint);');
  Add('begin');
  Add('end;');
  Add('var o: TObject;');
  Add('begin');
  Add('  o.B[3,''abc'']:=7;');
  Add('  if o.B[4,'''']=8 then;');
  Add('  if 9=o.B[6,''d''] then;');
  ParseProgram;
end;

procedure TTestResolver.TestPropertyArgsWithDefaultsFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    function GetB(Index: longint): boolean;');
  Add('    procedure SetB(Index: longint; Value: boolean);');
  Add('    property B[Index: longint = 0]: boolean read GetB write SetB;');
  Add('  end;');
  Add('function TObject.GetB(Index: longint): boolean;');
  Add('begin');
  Add('end;');
  Add('procedure TObject.SetB(Index: longint; Value: boolean);');
  Add('begin');
  Add('end;');
  Add('begin');
  CheckParserException('Property arguments can not have default values',
    PParser.nParserPropertyArgumentsCanNotHaveDefaultValues);
end;

procedure TTestResolver.TestPropertyArgs_StringConstDefault;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function GetItems(const s: string): byte; virtual; abstract;',
  '    procedure SetItems(const s: string; b: byte); virtual; abstract;',
  '    property Items[s: string]: byte read GetItems write SetItems;',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassProperty;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class function GetStatic: word; static;',
  '    class procedure SetStatic(Value: word); static;',
  '    class property StaticP: word read GetStatic write SetStatic;',
  '  end;',
  'class function TObject.GetStatic: word;',
  'begin',
  '  StaticP:=StaticP;',
  'end;',
  'class procedure TObject.SetStatic(Value: word);',
  'begin',
  'end;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassPropertyNonStaticFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class function GetNonStatic: word;',
  '    class property NonStatic: word read GetNonStatic;',
  '  end;',
  'class function TObject.GetNonStatic: word;',
  'begin',
  'end;',
  'begin',
  '']);
  CheckResolverException(sClassPropertyAccessorMustBeStatic,nClassPropertyAccessorMustBeStatic);
end;

procedure TTestResolver.TestClassPropertyNonStaticAllow;
begin
  ResolverEngine.Options:=ResolverEngine.Options+[proClassPropertyNonStatic];
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class function GetStatic: word; static;',
  '    class procedure SetStatic(Value: word); static;',
  '    class property StaticP: word read GetStatic write SetStatic;',
  '    class function GetNonStatic: word;',
  '    class procedure SetNonStatic(Value: word);',
  '    class property NonStatic: word read GetNonStatic write SetNonStatic;',
  '  end;',
  '  TClass = class of TObject;',
  'class function TObject.GetStatic: word;',
  'begin',
  '  StaticP:=StaticP;',
  '  NonStatic:=NonStatic;',
  'end;',
  'class procedure TObject.SetStatic(Value: word);',
  'begin',
  'end;',
  'class function TObject.GetNonStatic: word;',
  'begin',
  '  StaticP:=StaticP;',
  '  NonStatic:=NonStatic;',
  'end;',
  'class procedure TObject.SetNonStatic(Value: word);',
  'begin',
  'end;',
  'var',
  '  c: TClass;',
  '  o: TObject;',
  'begin',
  '  c.STaticP:=c.StaticP;',
  '  o.STaticP:=o.StaticP;',
  '  c.NonStatic:=c.NonStatic;',
  '  o.NonStatic:=o.NonStatic;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestArrayProperty;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    {#FItems}FItems: array of string;');
  Add('    function {#GetItems}GetItems(Index: longint): string;');
  Add('    procedure {#SetItems}SetItems(Index: longint; Value: string);');
  Add('    procedure DoIt;');
  Add('    property {#Items}Items[Index: longint]: string read {@GetItems}getitems write {@SetItems}setitems;');
  Add('  end;');
  Add('function tobject.getitems(index: longint): string;');
  Add('begin');
  Add('  Result:={@FItems}fitems[index];');
  Add('end;');
  Add('procedure tobject.setitems(index: longint; value: string);');
  Add('begin');
  Add('  {@FItems}fitems[index]:=value;');
  Add('end;');
  Add('procedure tobject.doit;');
  Add('begin');
  Add('  {@Items}items[1]:={@Items}items[2];');
  Add('  self.{@Items}items[3]:=self.{@Items}items[4];');
  Add('end;');
  Add('var Obj: tobject;');
  Add('begin');
  Add('  obj.{@Items}Items[11]:=obj.{@Items}Items[12];');
  ParseProgram;
end;

procedure TTestResolver.TestArrayProperty_PassImplicitCallClassFunc;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualImplicitCallWithoutParams, ExpectedImplicitCallWithoutParams: Boolean;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function GetItems(s: string): string;',
  '    property Items[s: string]: string read GetItems; default;',
  '    class function Desc: string; virtual; abstract;',
  '  end;',
  'function TObject.GetItems(s: string): string;',
  'begin',
  '  Result:=Items[{#a_implicit}Desc];',
  '  Result:=Items[{#b_direct}Desc()];',
  '  Result:=Items[Self.{#c_implicit}Desc];',
  '  Result:=Items[Self.{#d_direct}Desc()];',
  'end;',
  'var b: TObject;',
  '  s: string;',
  'begin',
  '  s:=b.Items[b.{#m_implicit}Desc];',
  '  s:=b.Items[b.{#n_direct}Desc()];',
  '  s:=b.Items[TObject.{#o_implicit}Desc];',
  '  s:=b.Items[TObject.{#p_direct}Desc()];',
  '  s:=b[b.{#q_implicit}Desc];',
  '  s:=b[b.{#r_direct}Desc()];',
  '  s:=b[TObject.{#s_implicit}Desc];',
  '  s:=b[TObject.{#t_direct}Desc()];',
  '']);
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestArrayProperty_PassImplicitCallClassFunc ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualImplicitCallWithoutParams:=false;
      Ref:=nil;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestArrayProperty_PassImplicitCallClassFunc ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if Ref.Declaration is TPasProcedure then
          break
        else
          Ref:=nil;
        end;
      if Ref=nil then
        RaiseErrorAtSrcMarker('missing proc ref at "#'+aMarker^.Identifier+'"',aMarker);
      ActualImplicitCallWithoutParams:=rrfImplicitCallWithoutParams in Ref.Flags;
      ExpectedImplicitCallWithoutParams:=RightStr(aMarker^.Identifier,length('_implicit'))='_implicit';
      if ActualImplicitCallWithoutParams<>ExpectedImplicitCallWithoutParams then
        RaiseErrorAtSrcMarker('wrong implicit call at "#'+aMarker^.Identifier
          +', ExpectedImplicitCall='+BoolToStr(ExpectedImplicitCallWithoutParams,true)+'"',aMarker);
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestProperty_WrongTypeAsIndexFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    function GetItems(Index: string): string;');
  Add('    property Items[Index: string]: string read getitems;');
  Add('  end;');
  Add('function tobject.getitems(index: string): string;');
  Add('begin');
  Add('end;');
  Add('var Obj: tobject;');
  Add('begin');
  Add('  obj.Items[3]:=''4'';');
  CheckResolverException('Incompatible type arg no. 1: Got "Longint", expected "String"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestProperty_Option_ClassPropertyNonStatic;
begin
  ResolverEngine.Options:=ResolverEngine.Options+[proClassPropertyNonStatic];
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class function GetB: longint;');
  Add('    class procedure SetB(Value: longint);');
  Add('    class property B: longint read GetB write SetB;');
  Add('  end;');
  Add('class function TObject.GetB: longint;');
  Add('begin');
  Add('end;');
  Add('class procedure TObject.SetB(Value: longint);');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  TObject.B:=4;');
  Add('  if TObject.B=6 then;');
  Add('  if 7=TObject.B then;');
  ParseProgram;
end;

procedure TTestResolver.TestDefaultProperty;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function GetB(Index: longint): longint;',
  '    procedure SetB(Index: longint; Value: longint);',
  '    property B[Index: longint]: longint read GetB write SetB; default;',
  '  end;',
  'function TObject.GetB(Index: longint): longint;',
  'begin',
  'end;',
  'procedure TObject.SetB(Index: longint; Value: longint);',
  'begin',
  '  if Value=Self[Index] then ;',
  '  Self[Index]:=Value;',
  'end;',
  'var o: TObject;',
  'begin',
  '  o[3]:=4;',
  '  if o[5]=6 then;',
  '  if 7=o[8] then;']);
  ParseProgram;
end;

procedure TTestResolver.TestDefaultPropertyIncVisibility;
begin
  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'type',
    '  TNumber = longint;',
    '  TInteger = longint;',
    '  TObject = class',
    '  private',
    '    function GetItems(Index: TNumber): TInteger; virtual; abstract;',
    '    procedure SetItems(Index: TInteger; Value: TNumber); virtual; abstract;',
    '  protected',
    '    property Items[Index: TNumber]: longint read GetItems write SetItems;',
    '  end;']),
    LinesToStr([
    '']));

  StartProgram(true);
  Add([
  'uses unit1;',
  'type',
  '  TBird = class',
  '  public',
  '    property Items;',
  '  end;',
  'procedure DoIt(i: TInteger);',
  'begin',
  'end;',
  'var b: TBird;',
  'begin',
  '  b.Items[1]:=2;',
  '  b.Items[3]:=b.Items[4];',
  '  DoIt(b.Items[5]);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProperty_MissingDefault;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('var o: TObject;');
  Add('begin');
  Add('  if o[5]=6 then;');
  CheckResolverException('illegal qualifier "[" after "TObject"',
    nIllegalQualifierAfter);
end;

procedure TTestResolver.TestProperty_DefaultDotFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function GetItems(Index: byte): byte;',
  '    property Items[Index: byte]: byte read GetItems; default;',
  '  end;',
  'function TObject.GetItems(Index: byte): byte; begin end;',
  'var o: TObject;',
  'begin',
  '  if o.Items.i=6 then;',
  '']);
  CheckResolverException('illegal qualifier "." after "Items:array property"',
    nIllegalQualifierAfter);
end;

procedure TTestResolver.TestClassInterface;
begin
  StartProgram(false);
  Add([
  'type',
  '  {$interfaces corba}',
  '  ICorbaIntf = interface',
  '  end;',
  '  {$interfaces com}',
  '  IUnknown = interface',
  '  end;',
  '  IInterface = IUnknown;',
  '  IComIntf = interface',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterfaceForward;
begin
  StartProgram(false);
  Add([
  'type',
  '  IBird = interface;',
  '  TObject = class',
  '    Bird: IBird;',
  '  end;',
  '  IUnknown = interface',
  '  end;',
  '  IBird = interface(IUnknown)',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterfaceVarFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    i: longint;',
  '  end;',
  'begin']);
  CheckParserException('Fields are not allowed in interface',nParserNoFieldsAllowed);
end;

procedure TTestResolver.TestClassInterfaceConstFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    const i = 3;',
  '  end;',
  'begin']);
  CheckParserException('CONST is not allowed in interface',nParserXNotAllowedInY);
end;

procedure TTestResolver.TestClassInterfaceClassMethodFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    class procedure DoIt;',
  '  end;',
  'begin']);
  CheckParserException('CLASS is not allowed in interface',nParserXNotAllowedInY);
end;

procedure TTestResolver.TestClassInterfaceNestedTypeFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    type l = longint;',
  '  end;',
  'begin']);
  CheckParserException('TYPE is not allowed in interface',nParserXNotAllowedInY);
end;

procedure TTestResolver.TestClassInterfacePropertyStoredFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    function GetSize: longint;',
  '    property Size: longint read GetSize stored false;',
  '  end;',
  'begin']);
  CheckParserException('STORED is not allowed in interface',nParserXNotAllowedInY);
end;

procedure TTestResolver.TestClassInterface_ConstructorFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    constructor Create;',
  '  end;',
  'begin']);
  CheckParserException('constructor is not allowed in interface',nParserXNotAllowedInY);
end;

procedure TTestResolver.TestClassInterface_DelphiClassAncestorIntfFail;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  IInterface = interface',
  '  end;',
  '  TObject = class(IInterface)',
  '  end;',
  'begin']);
  CheckResolverException('class type expected, but interface type found',nXExpectedButYFound);
end;

procedure TTestResolver.TestClassInterface_ObjFPCClassAncestorIntf;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  'type',
  '  IUnknown = interface',
  '  end;',
  '  TObject = class(IUnknown)',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterface_MethodVirtualFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt; virtual;',
  '  end;',
  'begin']);
  CheckParserException('Fields are not allowed in interface',nParserNoFieldsAllowed);
end;

procedure TTestResolver.TestClassInterface_Overloads;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt(i: longint);',
  '    procedure DoIt(s: string);',
  '  end;',
  '  IBird = interface',
  '    procedure DoIt(b: boolean); overload;',
  '  end;',
  '  TObject = class end;',
  '  TBird = class(TObject,IBird)',
  '    procedure DoIt(i: longint); virtual; abstract;',
  '    procedure DoIt(s: string); virtual; abstract;',
  '    procedure DoIt(b: boolean); virtual; abstract;',
  '  end;',
  'var i: IBird;',
  'begin',
  '  i.DoIt(3);',
  '  i.DoIt(''abc'');',
  '  i.DoIt(true);',
  '']);
  ParseProgram;
  CheckResolverUnexpectedHints();
end;

procedure TTestResolver.TestClassInterface_OverloadHint;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '  end;',
  '  IBird = interface',
  '    procedure DoIt;',
  '  end;',
  'begin']);
  ParseProgram;
  CheckResolverHint(mtHint,nMethodHidesNonVirtualMethodExactly,
    'method hides identifier at "afile.pp(4,19)". Use reintroduce');
end;

procedure TTestResolver.TestClassInterface_OverloadNoHint;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '    procedure DoIt(i: longint);',
  '  end;',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestClassInterface_IntfListClassFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TAnimal = class',
  '  end;',
  '  TBird = class(TObject,TAnimal)',
  '  end;',
  'begin']);
  CheckResolverException('interface type expected, but class type found',nXExpectedButYFound);
end;

procedure TTestResolver.TestClassInterface_IntfListDuplicateFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IA = interface',
  '  end;',
  '  IB = IA;',
  '  TObject = class(IA,IB)',
  '  end;',
  'begin']);
  CheckResolverException('Duplicate identifier "IB" at 1',nDuplicateIdentifier);
end;

procedure TTestResolver.TestClassInterface_MissingMethodFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '  end;',
  '  TObject = class(IUnknown)',
  '  end;',
  'begin']);
  CheckResolverException('No matching implementation for interface method "procedure IUnknown.DoIt of Object" found',
    nNoMatchingImplForIntfMethodXFound);
end;

procedure TTestResolver.TestClassInterface_MissingAncestorMethodFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '  end;',
  '  IBird = interface',
  '  end;',
  '  TObject = class(IBird)',
  '  end;',
  'begin']);
  CheckResolverException('No matching implementation for interface method "procedure IUnknown.DoIt of Object" found',
    nNoMatchingImplForIntfMethodXFound);
end;

procedure TTestResolver.TestClassInterface_DefaultProperty;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IA = interface',
  '    function GetItems(Index: longint): boolean;',
  '    procedure SetItems(Index: longint; Value: boolean);',
  '    property Items[IndeX: longint]: boolean read GetItems write SetItems; default;',
  '  end;',
  '  IB = IA;',
  '  TObject = class(IB)',
  '  strict private',
  '    function GetItems(Index: longint): boolean; virtual; abstract;',
  '    procedure SetItems(Index: longint; Value: boolean); virtual; abstract;',
  '  end;',
  'var',
  '  a: IA;',
  '  b: IB;',
  'begin',
  '  a[1]:=a[2];',
  '  b[3]:=b[4];']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterface_MethodResolution;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt(i: longint);',
  '    procedure DoIt(s: string);',
  '    function DoIt(b: boolean): boolean;',
  '    function GetIt: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    procedure IUnknown.DoIt = DoSome;',
  '    function IUnknown.GetIt = GetIt;',
  '    procedure DoSome(i: longint); virtual; abstract;',
  '    procedure DoSome(s: string); virtual; abstract;',
  '    function GetIt: longint; virtual; abstract;',
  '    function DoIt(b: boolean): boolean; virtual; abstract;',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterface_MethodResolutionDuplicateFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    procedure IUnknown.DoIt = DoSome;',
  '    procedure IUnknown.DoIt = DoMore;',
  '    procedure DoSome; virtual; abstract;',
  '    procedure DoMore; virtual; abstract;',
  '  end;',
  'begin']);
  CheckResolverException('Duplicate identifier "procedure IUnknown.DoIt" at afile.pp(7,14) at afile.pp (8,24)',nDuplicateIdentifier);
end;

procedure TTestResolver.TestClassInterface_DelegationIntf;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '  end;',
  '  IBird = interface',
  '  end;',
  '  TObject = class(IUnknown, IBird)',
  '    function GetI: IBird; virtual; abstract;',
  '    property MyI: IBird read GetI implements IUnknown, IBird;',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterface_Delegation_DuplPropFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '  end;',
  '  IBird = interface',
  '  end;',
  '  TObject = class(IUnknown, IBird)',
  '    function GetI: IBird; virtual; abstract;',
  '    property MyI: IBird read GetI implements IBird;',
  '    property MyJ: IBird read GetI implements IBird;',
  '  end;',
  'begin']);
  CheckResolverException('Duplicate implements for interface "IBird" at afile.pp(10,17)',
    nDuplicateImplementsForIntf);
end;

procedure TTestResolver.TestClassInterface_Delegation_MethodResFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '  end;',
  '  IBird = interface',
  '  end;',
  '  TObject = class(IUnknown, IBird)',
  '    function GetI: IBird; virtual; abstract;',
  '    procedure IBird.DoIt = DoSome;',
  '    procedure DoSome; virtual; abstract;',
  '    property MyI: IBird read GetI implements IBird;',
  '  end;',
  'begin']);
  CheckResolverException('Cannot mix method resolution and delegation at afile.pp(12,17)',
    nCannotMixMethodResolutionAndDelegationAtX);
end;

procedure TTestResolver.TestClassInterface_DelegationClass;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '  end;',
  '  IBird = interface',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(IBird)',
  '    procedure DoIt; virtual; abstract;',
  '  end;',
  '  TEagle = class(IBird)',
  '    FBird: TBird;',
  '    property Bird: TBird read FBird implements IBird;',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterface_DelegationFQN;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(IUnknown)',
  '    procedure DoIt; virtual; abstract;',
  '  end;',
  '  TEagle = class(IUnknown)',
  '    FBird: TBird;',
  '    property Bird: TBird read FBird implements afile.IUnknown;',
  '  end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterface_Assign;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IBird = interface',
  '    procedure Fly;',
  '  end;',
  '  IEagle = interface(IBird)',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(IBird)',
  '    procedure Fly; virtual; abstract;',
  '  end;',
  '  TAlbatros = class(TBird)',
  '  end;',
  'var',
  '  i: IUnknown = nil;',
  '  e: IEagle;',
  '  b: IBird;',
  '  oBird,oBird2: TBird;',
  '  o: TObject;',
  '  a: TAlbatros;',
  '  p: pointer;',
  'begin',
  '  if Assigned(i) then ;',
  '  if TypeInfo(i)=nil then ;',
  '  i:=nil;',
  '  i:=i;',
  '  i:=e;',
  '  if i=nil then ;',
  '  if i=e then ;',
  '  if e=i then ;',
  '  e:=IEagle(i);',
  '  if i is IEagle then ;',
  '  e:=i as IEagle;',
  '  b:=oBird;',
  '  b:=a;',
  '  i:=IBird(oBird);', // FPC needs GUID
  '  oBird2:=TBird(i);', // not supported by FPC
  '  oBird2:=TBird(e);', // not supported by FPC
  '  i:=o as IBird;', // FPC needs GUID
  '  oBird2:=i as TBird;',
  '  oBird2:=e as TBird;',
  '  if o is IBird then ;', // FPC needs GUID
  '  if i is TBird then ;',
  '  if e is TBird then ;',
  '  p:=i;',
  '  if p=i then ;',
  '  if i=p then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterface_AssignObjVarIntfVarFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '  end;',
  '  TObject = class(IUnknown)',
  '  end;',
  'var',
  '  i: IUnknown;',
  '  o: TObject;',
  'begin',
  '  o:=i;',
  '']);
  CheckResolverException('Incompatible types: got "IUnknown" expected "TObject"',nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestClassInterface_AssignDescendentFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IBird = interface',
  '  end;',
  '  TObject = class(IBird)',
  '  end;',
  'var',
  '  i: IUnknown;',
  '  o: TObject;',
  'begin',
  '  i:=o;',
  '']);
  CheckResolverException('Incompatible types: got "TObject" expected "IUnknown"',nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestClassInterface_Args;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IBird = interface',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(IBird)',
  '  end;',
  'function GetIt(var u; i: IBird; const j: IBird): IBird;',
  'begin',
  '  Result:=IBird(u);',
  '  Result:=i;',
  '  Result:=j;',
  'end;',
  'procedure Change(var i: IBird; out j: IBird);',
  'begin',
  '  i:=GetIt(i,i,i);',
  'end;',
  'var',
  '  i: IBird;',
  '  o: TBird;',
  'begin',
  '  i:=GetIt(i,i,i);',
  '  Change(i,i);',
  '  GetIt(i,o,o);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterface_Enumerator;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TItem = TObject;',
  '  TEnumerator = class',
  '    FCurrent: TItem;',
  '    property Current: TItem read FCurrent;',
  '    function MoveNext: boolean;',
  '  end;',
  '  IUnknown = interface end;',
  '  IEnumerator = interface',
  '    function GetCurrent: TItem;',
  '    property Current: TItem read GetCurrent;',
  '    function MoveNext: boolean;',
  '  end;',
  '  IEnumerable = interface',
  '    function GetEnumerator: IEnumerator;',
  '  end;',
  '  IBird = interface',
  '    function GetEnumerator: TEnumerator;',
  '  end;',
  'function TEnumerator.MoveNext: boolean;',
  'begin',
  'end;',
  'var',
  '  e: IEnumerable;',
  '  b: IBird;',
  '  i: TItem;',
  '  {#i2}i2: TItem;',
  'begin',
  '  for i in e do {@i2}i2:=i;',
  '  for i in b do {@i2}i2:=i;']);
  ParseProgram;
end;

procedure TTestResolver.TestClassInterface_PassTypecastClassToIntfAsVarParamFail;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface end;',
  '  TObject = class end;',
  '  TBall = class(IUnknown) end;',
  'procedure DoIt(var i: IUnknown); begin end;',
  'var b: TBall;',
  'begin',
  '  DoIt(IUnknown(b));']);
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.
  TestClassInterface_PassTypecastIntfToClassAsVarParamFail;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface end;',
  '  TObject = class end;',
  '  TBall = class(IUnknown) end;',
  'procedure DoIt(var i: IUnknown); begin end;',
  'var i: IUnknown;',
  'begin',
  '  DoIt(TBall(i));']);
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestClassInterface_GUID;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '    [''{F31DB68F-3010-D355-4EBA-CDD4EF4A737C}'']',
  '  end;',
  '  TObject = class end;',
  '  TGUID = record D1,D2,D3,D4: word; end;',
  '  TAliasGUID = TGUID;',
  '  TGUIDString = type string;',
  '  TAliasGUIDString = TGUIDString;',
  'procedure {#A}DoIt(const g: TAliasGUID); overload;',
  'begin end;',
  'procedure {#B}DoIt(const s: TAliasGUIDString); overload;',
  'begin end;',
  'var',
  '  i: IUnknown;',
  '  g: TAliasGUID = ''{D91C9AF4-3C93-420F-A303-BF5BA82BFD23}'';',
  '  s: TAliasGUIDString;',
  'begin',
  '  {@A}DoIt(IUnknown);',
  '  {@A}DoIt(i);',
  '  g:=i;',
  '  g:=IUnknown;',
  '  g:=''{D91C9AF4-3C93-420F-A303-BF5BA82BFD23}'';',
  '  s:=g;',
  '  s:=IUnknown;',
  '  s:=i;',
  '  {@B}DoIt(s);',
  '  if s=IUnknown then ;',
  '  if IUnknown=s then ;',
  '  if s=i then ;',
  '  if i=s then ;',
  '  if g=IUnknown then ;',
  '  if IUnknown=g then ;',
  '  if g=i then ;',
  '  if i=g then ;',
  '  if s=g then ;',
  '  if g=s then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestPropertyAssign;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FB: longint;');
  Add('    property B: longint read FB write FB;');
  Add('  end;');
  Add('var');
  Add('  o: TObject;');
  Add('  i: longint;');
  Add('begin');
  Add('  {#a1_read}o.{#a2_assign}B:=i;');
  Add('  i:={#b1_read}o.{#b2_read}B;');
  Add('  if i={#c1_read}o.{#c2_read}B then ;');
  Add('  if {#d1_read}o.{#d2_read}B=3 then ;');
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestPropertyAssignReadOnlyFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FB: longint;');
  Add('    property B: longint read FB;');
  Add('  end;');
  Add('var');
  Add('  o: TObject;');
  Add('begin');
  Add('  o.B:=3;');
  CheckResolverException('No member is provided to access property',nPropertyNotWritable);
end;

procedure TTestResolver.TestProperty_PassAsParam;
begin
  ResolverEngine.Options:=ResolverEngine.Options+[proPropertyAsVarParam];
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FA: longint;');
  Add('    property A: longint read FA write FA;');
  Add('  end;');
  Add('procedure DoIt(i: longint; const j: longint; var k: longint; out l: longint);');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  o: TObject;');
  Add('begin');
  Add('  DoIt({#o1_read}o.{#o_a1_read}a,');
  Add('    {#o2_read}o.{#o_a2_read}a,');
  Add('    {#o3_read}o.{#o_a3_var}a,');
  Add('    {#o4_read}o.{#o_a4_out}a);');
  Add('  with o do');
  Add('    DoIt({#w_a1_read}a,');
  Add('      {#w_a2_read}a,');
  Add('      {#w_a3_var}a,');
  Add('      {#w_a4_out}a);');
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestPropertyReadNonReadableFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FB: longint;');
  Add('    property B: longint write FB;');
  Add('  end;');
  Add('var');
  Add('  o: TObject;');
  Add('begin');
  Add('  if o.B=3 then;');
  CheckResolverException('not readable',nNotReadable);
end;

procedure TTestResolver.TestWithBlock1;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    {#TOBJ_A}A: longint;');
  Add('  end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#a}a: longint;');
  Add('begin');
  Add('  {@a}a:=1;');
  Add('  with {@o}o do');
  Add('    {@TOBJ_A}a:=2;');
  ParseProgram;
end;

procedure TTestResolver.TestWithBlock2;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    {#TOBJ_i}i: longint;');
  Add('  end;');
  Add('  {#TA}TClassA = class');
  Add('    {#TA_j}j: longint;');
  Add('    {#TA_b}{=TA}b: TClassA;');
  Add('  end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#a}{=TA}a: TClassA;');
  Add('  {#i}i: longint;');
  Add('begin');
  Add('  {@i}i:=1;');
  Add('  with {@o}o do');
  Add('    {@TOBJ_i}i:=2;');
  Add('  {@i}i:=1;');
  Add('  with {@o}o,{@a}a do begin');
  Add('    {@TOBJ_i}i:=3;');
  Add('    {@TA_j}j:=4;');
  Add('    {@TA_b}b:={@a}a;');
  Add('  end;');
  ParseProgram;
end;

procedure TTestResolver.TestWithBlockFuncResult;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    {#TOBJ_i}i: longint;');
  Add('  end;');
  Add('  {#TA}TClassA = class');
  Add('    {#TA_j}j: longint;');
  Add('    {#TA_b}{=TA}b: TClassA;');
  Add('  end;');
  Add('function {#GiveA}Give: TClassA;');
  Add('begin');
  Add('end;');
  Add('function {#GiveB}Give(i: longint): TClassA;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#a}{=TA}a: TClassA;');
  Add('  {#i}i: longint;');
  Add('begin');
  Add('  with {@GiveA}Give do {@TOBJ_i}i:=3;');
  Add('  with {@GiveA}Give() do {@TOBJ_i}i:=3;');
  Add('  with {@GiveB}Give(2) do {@TOBJ_i}i:=3;');
  ParseProgram;
end;

procedure TTestResolver.TestWithBlockConstructor;
begin
  StartProgram(false);
  Add('type');
  Add('  {#TOBJ}TObject = class');
  Add('    {#TOBJ_i}i: longint;');
  Add('  end;');
  Add('  {#TA}TClassA = class');
  Add('    {#TA_j}j: longint;');
  Add('    {#TA_b}{=TA}b: TClassA;');
  Add('    constructor {#A_CreateA}Create;');
  Add('    constructor {#A_CreateB}Create(i: longint);');
  Add('  end;');
  Add('constructor TClassA.Create;');
  Add('begin');
  Add('end;');
  Add('constructor TClassA.Create(i: longint);');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  {#o}{=TOBJ}o: TObject;');
  Add('  {#a}{=TA}a: TClassA;');
  Add('  {#i}i: longint;');
  Add('begin');
  Add('  with TClassA.{@A_CreateA}Create do {@TOBJ_i}i:=3;');
  Add('  with TClassA.{@A_CreateA}Create() do {@TOBJ_i}i:=3;');
  Add('  with TClassA.{@A_CreateB}Create(2) do {@TOBJ_i}i:=3;');
  ParseProgram;
end;

procedure TTestResolver.TestDynArrayOfLongint;
begin
  StartProgram(false);
  Add('type TIntArray = array of longint;');
  Add('var a: TIntArray;');
  Add('begin');
  Add('  a:=nil;');
  Add('  if a=nil then ;');
  Add('  if nil=a then ;');
  Add('  SetLength(a,3);');
  Add('  a[0]:=1;');
  Add('  a[1]:=length(a);');
  Add('  a[2]:=a[0];');
  Add('  if a[3]=a[4] then ;');
  Add('  a[a[5]]:=a[a[6]];');
  ParseProgram;
end;

procedure TTestResolver.TestDynArrayOfSelfFail;
begin
  StartProgram(false);
  Add('type TIntArray = array of TIntArray;');
  Add('begin');
  CheckResolverException(sIllegalExpression,nIllegalExpression);
end;

procedure TTestResolver.TestStaticArray;
begin
  StartProgram(false);
  Add('type');
  Add('  TArrA = array[1..2] of longint;');
  Add('  TArrB = array[char] of boolean;');
  Add('  TArrC = array[byte,''a''..''z''] of longint;');
  Add('const');
  Add('  ArrA: TArrA = (3,4);');
  Add('var');
  Add('  a: TArrA;');
  Add('  b: TArrB;');
  Add('  c: TArrC;');
  Add('begin');
  Add('  a[1]:=1;');
  Add('  if a[2]=low(a) then ;');
  Add('  b[''x'']:=true;');
  Add('  if b[''y''] then ;');
  Add('  c[3,''f'']:=1;');
  Add('  if c[4,''g'']=a[1] then ;');
  ParseProgram;
end;

procedure TTestResolver.TestStaticArrayOfChar;
begin
  ResolverEngine.ExprEvaluator.DefaultStringCodePage:=CP_UTF8;
  StartProgram(false);
  Add([
  'type',
  '  TArrA = array[1..3] of char;',
  'const',
  '  A: TArrA = (''p'',''a'',''p'');', // duplicate allowed, this bracket is not a set
  '  B: TArrA = ''pas'';',
  '  Three = length(TArrA);',
  '  C: array[1..Three] of char = ''pas'';',
  '  D = ''pp'';',
  '  E: array[length(D)..Three] of char = D;',
  '  F: array[1..2] of widechar = ''äö'';',
  '  G: array[1..2] of char = ''ä'';',
  '  H: array[1..4] of char = ''äö'';',
  '  I: array[1..4] of char = ''ä''+''ö'';',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestStaticArrayOfCharDelphi;
begin
  ResolverEngine.ExprEvaluator.DefaultStringCodePage:=CP_UTF8;
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TArrA = array[1..3] of char;',
  'const',
  '  A: TArrA = (''p'',''a'',''p'');', // duplicate allowed, this bracket is not a set
  '  B: TArrA = ''pas'';',
  '  Three = length(TArrA);',
  '  C: array[1..Three] of char = ''pas'';',
  '  D = ''pp'';',
  '  E: array[length(D)..Three] of char = D;',
  '  F: array[1..2] of widechar = ''äö'';',
  '  G: array[1..2] of char = ''ä'';',
  '  H: array[1..4] of char = ''äö'';',
  '  I: array[1..4] of char = ''ä''+''ö'';',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestStaticArrayOfRangeElCheckFail;
begin
  StartProgram(false);
  Add('var');
  Add('  A: array[1..2] of shortint = (1,300);');
  Add('begin');
  ParseProgram;
  CheckResolverHint(mtWarning,nRangeCheckEvaluatingConstantsVMinMax,
    'range check error while evaluating constants (300 is not between -128 and 127)');
end;

procedure TTestResolver.TestArrayOfArray;
begin
  StartProgram(false);
  Add('type');
  Add('  TArrA = array[byte] of longint;');
  Add('  TArrB = array[smallint] of TArrA;');
  Add('  TArrC = array of array of longint;');
  Add('var');
  Add('  b: TArrB;');
  Add('  c: TArrC;');
  Add('begin');
  Add('  b[1][2]:=5;');
  Add('  b[1,2]:=5;');
  Add('  if b[2,1]=b[0,1] then ;');
  Add('  c[3][4]:=c[5,6];');
  Add('  Setlength(c[3],7);');
  Add('  Setlength(c,8,9);');
  ParseProgram;
end;

procedure TTestResolver.TestArrayOfArray_NameAnonymous;
begin
  ResolverEngine.AnonymousElTypePostfix:='$array';
  StartProgram(false);
  Add('type');
  Add('  TArrA = array of array of longint;');
  Add('var');
  Add('  a: TArrA;');
  Add('begin');
  Add('  a[1][2]:=5;');
  Add('  a[1,2]:=5;');
  Add('  if a[2,1]=a[0,1] then ;');
  Add('  a[3][4]:=a[5,6];');
  ParseProgram;
end;

procedure TTestResolver.TestFunctionReturningArray;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArrA = array[1..20] of longint;',
  '  TArrB = array of TArrA;',
  'function FuncC: TArrB;',
  'begin',
  '  SetLength(Result,3);',
  'end;',
  'begin',
  '  FuncC[2,4]:=6;',
  '  FuncC()[1,3]:=5;']);
  ParseProgram;
end;

procedure TTestResolver.TestArray_LowHigh;
begin
  StartProgram(false);
  Add('type');
  Add('  TArrA = array[char] of longint;');
  Add('  TArrB = array of TArrA;');
  Add('var');
  Add('  c: char;');
  Add('  i: longint;');
  Add('begin');
  Add('  for c:=low(TArrA) to High(TArrA) do ;');
  Add('  for i:=low(TArrB) to High(TArrB) do ;');
  ParseProgram;
end;

procedure TTestResolver.TestArray_LowVarFail;
begin
  StartProgram(false);
  Add([
  'var a: array of longint;',
  'const l = length(a);',
  'begin']);
  CheckResolverException(sConstantExpressionExpected,nConstantExpressionExpected);
end;

procedure TTestResolver.TestArray_AssignDiffElTypeFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TArrA = array of longint;');
  Add('  TArrB = array of byte;');
  Add('var');
  Add('  a: TArrA;');
  Add('  b: TArrB;');
  Add('begin');
  Add('  a:=b;');
  CheckResolverException('Incompatible types: got "array of Longint" expected "array of Byte"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestArray_AssignSameSignatureDelphiFail;
begin
  StartProgram(false);
  Add('{$mode delphi}');
  Add('type');
  Add('  TArrA = array of longint;');
  Add('  TArrB = array of longint;');
  Add('var');
  Add('  a: TArrA;');
  Add('  b: TArrB;');
  Add('begin');
  Add('  a:=b;');
  CheckResolverException('Incompatible types: got "TArrB" expected "TArrA"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestArray_Assigned;
begin
  StartProgram(false);
  Add('var a: array of longint;');
  Add('begin');
  Add('  if Assigned(a) then ;');
  ParseProgram;
end;

procedure TTestResolver.TestPropertyOfTypeArray;
begin
  StartProgram(false);
  Add('type');
  Add('  TArray = array of longint;');
  Add('  TObject = class');
  Add('    FItems: TArray;');
  Add('    function GetItems: TArray;');
  Add('    procedure SetItems(Value: TArray);');
  Add('    property Items: TArray read FItems write FItems;');
  Add('    property Numbers: TArray read GetItems write SetItems;');
  Add('  end;');
  Add('function TObject.GetItems: TArray;');
  Add('begin');
  Add('  Result:=FItems;');
  Add('end;');
  Add('procedure TObject.SetItems(Value: TArray);');
  Add('begin');
  Add('  FItems:=Value;');
  Add('end;');
  Add('var Obj: TObject;');
  Add('begin');
  Add('  Obj.Items[3]:=4;');
  Add('  if Obj.Items[5]=6 then;');
  Add('  Obj.Numbers[7]:=8;');
  Add('  if Obj.Numbers[9]=10 then;');
  ParseProgram;
end;

procedure TTestResolver.TestArrayElementFromFuncResult_AsParams;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualImplicitCall: Boolean;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add('type Integer = longint;');
  Add('type TArrayInt = array of integer;');
  Add('function GetArr(vB: integer = 0): tarrayint;');
  Add('begin');
  Add('end;');
  Add('procedure DoIt(vG: integer);');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  doit({#a}getarr[1+1]);');
  Add('  doit({#b}getarr()[2+1]);');
  Add('  doit({#b}getarr(7)[3+1]);');
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestArrayElementFromFuncResult_AsParams ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualImplicitCall:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestArrayElementFromFuncResult_AsParams ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if rrfImplicitCallWithoutParams in Ref.Flags then
          ActualImplicitCall:=true;
        break;
        end;
      case aMarker^.Identifier of
      'a':
        if not ActualImplicitCall then
          RaiseErrorAtSrcMarker('expected rrfImplicitCallWithoutParams at "#'+aMarker^.Identifier+'"',aMarker);
      else
        if ActualImplicitCall then
          RaiseErrorAtSrcMarker('expected no rrfImplicitCallWithoutParams at "#'+aMarker^.Identifier+'"',aMarker);
      end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestArrayEnumTypeRange;
begin
  StartProgram(false);
  Add('type');
  Add('  TEnum = (red,blue);');
  Add('  TEnumArray = array[TEnum] of longint;');
  Add('var');
  Add('  e: TEnum;');
  Add('  i: longint;');
  Add('  a: TEnumArray;');
  Add('  names: array[TEnum] of string = (''red'',''blue'');');
  Add('begin');
  Add('  e:=low(a);');
  Add('  e:=high(a);');
  Add('  i:=a[red];');
  Add('  a[e]:=a[e];');
  ParseProgram;
end;

procedure TTestResolver.TestArrayEnumTypeConstNotEnoughValuesFail1;
begin
  StartProgram(false);
  Add('type');
  Add('  TEnum = (red,blue);');
  Add('var');
  Add('  a: array[TEnum] of string = (''red'');');
  Add('begin');
  CheckResolverException('Expect 2 array elements, but found 1',nExpectXArrayElementsButFoundY);
end;

procedure TTestResolver.TestArrayEnumTypeConstNotEnoughValuesFail2;
begin
  StartProgram(false);
  Add('type');
  Add('  TEnum = (red,blue,green);');
  Add('var');
  Add('  a: array[TEnum] of string = (''red'',''blue'');');
  Add('begin');
  CheckResolverException('Expect 3 array elements, but found 2',nExpectXArrayElementsButFoundY);
end;

procedure TTestResolver.TestArrayEnumTypeConstWrongTypeFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TEnum = (red,blue);');
  Add('var');
  Add('  a: array[TEnum] of string = (1,2);');
  Add('begin');
  CheckResolverException('Incompatible types: got "Longint" expected "String"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestArrayEnumTypeConstNonConstFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TEnum = (red,blue);');
  Add('var');
  Add('  s: string;');
  Add('  a: array[TEnum] of string = (''red'',s);');
  Add('begin');
  CheckResolverException('Constant expression expected',
    nConstantExpressionExpected);
end;

procedure TTestResolver.TestArrayEnumTypeSetLengthFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TEnum = (red,blue);');
  Add('var');
  Add('  a: array[TEnum] of longint;');
  Add('begin');
  Add('  SetLength(a,1);');
  CheckResolverException('Incompatible type arg no. 1: Got "static array[] of Longint", expected "string or dynamic array variable"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestArrayEnumCustomRange;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red,blue,green);',
  '  TEnumRg = blue..green;',
  '  TEnumArray = array[TEnumRg] of longint;',
  '  TEnumArray2 = array[blue..green] of longint;',
  'var',
  '  e: TEnum;',
  '  r: TEnumRg;',
  '  i: longint;',
  '  a: TEnumArray;',
  '  b: array[TEnum] of longint;',
  '  c: TEnumArray2;',
  '  names: array[TEnumRg] of string = (''blue'',''green'');',
  'begin',
  '  r:=low(a);',
  '  r:=high(a);',
  '  i:=a[red];',
  '  a[e]:=a[e];',
  '  a[r]:=a[r];',
  '  b[r]:=b[r];',
  '  r:=low(c);',
  '  r:=high(c);',
  '  i:=c[red];',
  '  c[e]:=c[e];',
  '  c[r]:=c[r];',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestArray_DynArrayConstObjFPC;
begin
  Parser.Options:=Parser.Options+[po_cassignments];
  StartProgram(false);
  Add([
  '{$modeswitch arrayoperators}',
  'type',
  '  integer = longint;',
  '  TArrInt = array of integer;',
  '  TArrStr = array of string;',
  'const',
  '  Ints: TArrInt = (1,2,3);',
  '  Aliases: TarrStr = (''foo'',''b'');',
  '  OneInt: TArrInt = (7);',
  '  OneInt2: array of integer = (7);',
  '  Chars: array of char = ''aoc'';',
  '  Names: array of string = (''a'',''foo'');',
  '  NameCount = low(Names)+high(Names)+length(Names);',
  'procedure DoIt(Ints: TArrInt);',
  'begin',
  'end;',
  'var i: integer;',
  'begin',
  '  Ints:= {#a_array}[1,i];',
  '  Ints:= {#b1_array}[1,1]+ {#b2_array}[2]+ {#b3_array}[i];',
  '  Ints:= {#c_array}[i]+ {#d_array}[2,2];',
  '  Ints:=Ints+ {#e_array}[1];',
  '  Ints:= {#f_array}[1]+Ints;',
  '  Ints:=Ints+OneInt+OneInt2;',
  '  Ints+= {#g_array}[i];',
  '  Ints+= {#h_array}[1,1];',
  '  DoIt( {#i_array}[1,1]);',
  '  DoIt( {#j_array}[i]);',
  '']);
  ParseProgram;
  CheckParamsExpr_pkSet_Markers;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestArray_DynArrayConstDelphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'const c= {#c_set}[1,2];',
  'type',
  '  integer = longint;',
  '  TArrInt = array of integer;',
  '  TArrStr = array of string;',
  '  TArrInt2 = array of TArrInt;',
  '  TSetOfEnum = set of (red,blue);',
  '  TArrOfSet = array of TSetOfEnum;',
  'const',
  '  Ints: TArrInt = {#ints_array}[1,2,1];',
  '  Aliases: TarrStr = {#aliases_array}[''foo'',''b'',''b''];',
  '  OneInt: TArrInt = {#oneint_array}[7];',
  '  TwoInt: array of integer = {#twoint1_array}[7]+{#twoint2_array}[8];',
  '  Chars: array of char = ''aoc'';',
  '  Names: array of string = {#names_array}[''a'',''a''];',
  '  NameCount = low(Names)+high(Names)+length(Names);',
  'procedure {#DoArrOfSet}DoIt(const s: TArrOfSet); overload; begin end;',
  'procedure {#DoArrOfArrInt}DoIt(const a: TArrInt2); overload; begin end;',
  'begin',
  '  {@DoArrOfSet}DoIt( {#a1_array}[ {#a2_set}[blue], {#a3_set}[red] ]);',
  '  {@DoArrOfArrInt}DoIt( {#b1_array}[ {#b2_array}[1], {#b3_array}[2] ]);',
  '']);
  ParseProgram;
  CheckParamsExpr_pkSet_Markers;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestArray_DynArrAssignStaticDelphiFail;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TIntArr = array[1..3] of longint;',
  'var',
  '  dyn: array of longint;',
  '  sta: TIntArr;',
  'begin',
  '  dyn:=sta;']);
  CheckResolverException('Incompatible types: got "static array" expected "dynamic array"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestArray_Static_Const;
begin
  StartProgram(false);
  Add([
  'type',
  '  TIntArr = array[1..3] of longint;',
  'const',
  '  a = low(TIntArr)+high(TIntArr);',
  '  b: array[1..3] of longint = (10,11,12);',
  '  c: array[boolean] of TIntArr = ((21,22,23),(31,32,33));',
  'begin']);
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestArray_Record_Const;
begin
  StartProgram(false);
  Add([
  'type',
  '  TPoint = record x, y: longint; end;',
  '  TDynArray = array of TPoint;',
  '  TStaticArray = array[1..2] of TPoint;',
  '  TRecArr = record',
  '    DA: TDynArray;',
  '    SA: TStaticArray;',
  '  end;',
  'const',
  '  sa: TStaticArray = ( (x:2; y:3), (x:12;y:14) );',
  '  da: TDynArray = ( (x:22; y:23), (x:32;y:34) );',
  '  ra: TRecArr = (',
  '    DA: ( (x:42; y:43), (x:44;y:45) );',
  '    SA: ( (x:51; y:52), (x:53;y:54) );',
  '  );',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestArray_MultiDim_Const;
begin
  StartProgram(false);
  Add([
  '{$modeswitch arrayoperators}',
  'type',
  '  TDynArray = array of longint;',
  '  TDynArray2 = array of TDynArray;',
  '  TArrOfArr = array[1..2] of TDynArray;',
  '  TMultiDimArr = array[1..2,3..4] of longint;',
  'const',
  '  AoA: TArrOfArr = ( (1,2), (2,3) );',
  '  MultiDimArr: TMultiDimArr = ( (11,12), (13,14) );',
  '  A2: TDynArray2 = ( (1,2), (2,3) );',
  'var',
  '  A: TDynArray;',
  'procedure DoIt(const a: TDynArray2); begin end;',
  'var i: longint;',
  'begin',
  '  AoA:= {#a1_array}[ {#a2_array}[1], {#a3_array}[i] ];',
  '  AoA:= {#b1_array}[ {#b2_array}[i], A ];',
  '  AoA:= {#c1_array}[ {#c2_array}[i,2], {#c3_array}[2,i] ];',
  '  MultiDimArr:= {#d1_array}[ {#d2_array}[11,12], [13,14] ];',
  '  A2:= {#e1_array}[ {#e2_array}[1,2], {#e3_array}[2,3], {#e4_array}[i] ];',
  '  DoIt( {#f1_array}[ {#f2_array}[i,32], {#f3_array}[32,i] ]);',
  '  A2:= A2+ {#g1_array}[A];',
  '  A2:= {#h1_array}[A]+A2;',
  '']);
  ParseProgram;
  CheckParamsExpr_pkSet_Markers;
end;

procedure TTestResolver.TestArray_AssignNilToStaticArrayFail1;
begin
  StartProgram(false);
  Add('type');
  Add('  TEnum = (red,blue);');
  Add('var');
  Add('  a: array[TEnum] of longint;');
  Add('begin');
  Add('  a:=nil;');
  CheckResolverException('Incompatible types: got "Nil" expected "static array"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestArray_SetLengthProperty;
begin
  ResolverEngine.Options:=ResolverEngine.Options+[proPropertyAsVarParam];
  StartProgram(false);
  Add('type');
  Add('  TArrInt = array of longint;');
  Add('  TObject = class');
  Add('    function GetColors: TArrInt; external name ''GetColors'';');
  Add('    procedure SetColors(const Value: TArrInt); external name ''SetColors'';');
  Add('    property Colors: TArrInt read GetColors write SetColors;');
  Add('  end;');
  Add('procedure DoIt(var i: longint; out j: longint; const k: longint); begin end;');
  Add('var Obj: TObject;');
  Add('begin');
  Add('  SetLength(Obj.Colors,2);');
  Add('  DoIt(Obj.Colors[1],Obj.Colors[2],Obj.Colors[3]);');
  ParseProgram;
end;

procedure TTestResolver.TestStaticArray_SetlengthFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TArrInt = array[1..3] of longint;');
  Add('var a: TArrInt;');
  Add('begin');
  Add('  SetLength(a,2);');
  CheckResolverException(sIncompatibleTypeArgNo,nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestArray_PassArrayElementToVarParam;
begin
  StartProgram(false);
  Add('type');
  Add('  TArrInt = array of longint;');
  Add('procedure DoIt(var i: longint; out j: longint; const k: longint); begin end;');
  Add('var a: TArrInt;');
  Add('begin');
  Add('  DoIt(a[1],a[2],a[3]);');
  ParseProgram;
end;

procedure TTestResolver.TestArray_OpenArrayOfString;
begin
  StartProgram(false);
  Add([
  'type TArrStr = array of string;',
  'procedure DoIt(const a: array of String);',
  'var',
  '  i: longint;',
  '  s: string;',
  'begin',
  '  for i:=low(a) to high(a) do s:=a[length(a)-i-1];',
  'end;',
  'const arr: array[0..1] of string = (''A'', ''B'');',
  'var s: string;',
  'begin',
  '  DoIt([]);',
  '  DoIt([s,''foo'','''',s+s]);',
  '  DoIt(arr);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestArray_OpenArrayOfString_IntFail;
begin
  StartProgram(false);
  Add('procedure DoIt(const a: array of String);');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  DoIt([1]);');
  CheckResolverException('Incompatible types: got "Longint" expected "String"',nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestArray_OpenArrayOverride;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  Exception = class');
  Add('    constructor CreateFmt(const Msg: string; const Args: array of string); virtual;');
  Add('  end;');
  Add('  ESome = class(Exception)');
  Add('    constructor CreateFmt(const Msg: string; const Args: array of string); override;');
  Add('  end;');
  Add('constructor Exception.CreateFmt(const Msg: string; const Args: array of string);');
  Add('begin end;');
  Add('constructor ESome.CreateFmt(const Msg: string; const Args: array of string);');
  Add('begin');
  Add('  inherited CreateFmt(Msg,Args);');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestArray_OpenArrayAsDynArraySetLengthFail;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(a: array of byte);',
  'begin',
  '  SetLength(a,3);',
  'end;',
  'begin']);
  CheckResolverException('Incompatible type arg no. 1: Got "open array of Byte", expected "string or dynamic array variable"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestArray_OpenArrayAsDynArray;
begin
  ResolverEngine.Options:=ResolverEngine.Options+[proOpenAsDynArrays];
  StartProgram(false);
  Add([
  '{$modeswitch arrayoperators}',
  'type TArrStr = array of string;',
  'procedure DoStr(const a: TArrStr); forward;',
  'procedure DoIt(a: array of String);',
  'var',
  '  i: longint;',
  '  s: string;',
  'begin',
  '  SetLength(a,3);',
  '  DoStr(a);',
  '  DoStr(a+[s]);',
  '  DoStr([s]+a);',
  'end;',
  'procedure DoStr(const a: TArrStr);',
  'var s: string;',
  'begin',
  '  DoIt(a);',
  '  DoIt(a+[s]);',
  '  DoIt([s]+a);',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestArray_OpenArrayDelphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TDynArrInt = array of byte;',
  '  TStaArrInt = array[1..2] of byte;',
  'procedure Fly(var a: array of byte);',
  'begin',
  '  Fly(a);',
  'end;',
  'procedure DoIt(a: array of byte);',
  'var',
  '  d: TDynArrInt;',
  '  s: TStaArrInt;',
  'begin',
  '  DoIt(a);',
  '  // d:=s; forbidden in delphi', // see TestArray_DynArrAssignStaticDelphiFail
  '  // d:=a; forbidden in delphi',
  '  DoIt(d);',
  '  DoIt(s);',
  '  Fly(a);',
  '  Fly(d);', // dyn array can be passed to a var open array
  'end;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestArray_OpenArrayChar;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'Function CharInSet(Ch: Char;Const CSet : array of char) : Boolean;',
  'begin',
  'end;',
  'var Key: Char;',
  'begin',
  '  if CharInSet(Key, [^V, ^X, ^C]) then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestArray_CopyConcat;
begin
  StartProgram(false);
  Add([
  '{$modeswitch arrayoperators}',
  'type',
  '  integer = longint;',
  '  TArrayInt = array of integer;',
  '  TFlag = (red, blue);',
  '  TArrayFlag = array of TFlag;',
  'function Get(A: TArrayInt): TArrayInt; begin end;',
  'var',
  '  i: integer;',
  '  A: TArrayInt;',
  '  FA: TArrayFlag;',
  'begin',
  '  A:=Copy(A);',
  '  A:=Copy(A,1);',
  '  A:=Copy(A,2,3);',
  '  A:=Copy(Get(A),2,3);',
  '  Get(Copy(A));',
  '  A:=Concat(A);',
  '  A:=Concat(A,Get(A));',
  '  A:=Copy( {#a_array}[1]);',
  '  A:=Copy( {#b1_array}[1]+ {#b2_array}[2,3]);',
  '  A:=Concat( {#c_array}[1]);',
  '  A:=Concat( {#d1_array}[1], {#d2_array}[2,3]);',
  '  FA:=concat([red]);',
  '  FA:=concat([red],FA);',
  '']);
  ParseProgram;
  CheckParamsExpr_pkSet_Markers;
end;

procedure TTestResolver.TestStaticArray_CopyConcat;
begin
  exit;
  //ResolverEngine.Options:=ResolverEngine.Options+[proStaticArrayCopy,proStaticArrayConcat];
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  TArrayInt = array of integer;',
  '  TThreeInts = array[1..3] of integer;',
  'function Get(A: TThreeInts): TThreeInts; begin end;',
  'var',
  '  i: integer;',
  '  A: TArrayInt;',
  '  S: TThreeInts;',
  'begin',
  '  A:=Copy(S);',
  '  A:=Copy(S,1);',
  '  A:=Copy(S,2,3);',
  '  A:=Copy(Get(S),2,3);',
  '  A:=Concat(S,Get(S));']);
  ParseProgram;
end;

procedure TTestResolver.TestArray_CopyMismatchFail;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TArrayInt = array of integer;');
  Add('  TArrayStr = array of string;');
  Add('var');
  Add('  i: integer;');
  Add('  A: TArrayInt;');
  Add('  B: TArrayStr;');
  Add('begin');
  Add('  A:=Copy(B);');
  CheckResolverException('Incompatible types: got "array of integer" expected "array of String"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestArray_InsertDeleteAccess;
begin
  StartProgram(false);
  Add([
  '{$modeswitch arrayoperators}',
  'type',
  '  integer = longint;',
  '  TArrayInt = array of integer;',
  '  TArrArrInt = array of TArrayInt;',
  'var',
  '  i: integer;',
  '  A: TArrayInt;',
  '  A2: TArrArrInt;',
  'begin',
  '  Insert({#a1_read}i+1,{#a2_var}A,{#a3_read}i+2);',
  '  Insert([i],A2,i+2);',
  '  Insert(A+[1],A2,i+2);',
  '  Delete({#b1_var}A,{#b2_read}i+3,{#b3_read}i+4);']);
  ParseProgram;
  CheckAccessMarkers;
end;

procedure TTestResolver.TestArray_InsertArray;
begin
  StartProgram(false);
  Add([
  '{$modeswitch arrayoperators}',
  'type',
  '  integer = longint;',
  '  TArrayInt = array of integer;',
  '  TArrArrInt = array of TArrayInt;',
  '  TCol = (red,blue);',
  '  TSetCol = set of TCol;',
  '  TArrayCol = array of TCol;',
  '  TArrArrCol = array of TArrayCol;',
  '  TArrSetCol = array of TSetCol;',
  'var',
  '  i: integer;',
  '  ArrInt: TArrayInt;',
  '  ArrArrInt: TArrArrInt;',
  '  ArrArrCol: TArrArrCol;',
  '  ArrSetCol: TArrSetCol;',
  'begin',
  '  Insert( {#a_array}[1], ArrArrInt, i+2);',
  '  Insert( {#b_array}[i], ArrArrInt, 3);',
  '  Insert( ArrInt+ {#c_array}[1], ArrArrInt, 4);',
  '  Insert( {#d_set}[red], ArrSetCol, 5);',
  '  Insert( {#e_array}[red], ArrArrCol, 6);',
  '']);
  ParseProgram;
  CheckParamsExpr_pkSet_Markers;
end;

procedure TTestResolver.TestStaticArray_InsertFail;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TArrayInt = array[1..3] of integer;');
  Add('var');
  Add('  i: integer;');
  Add('  A: TArrayInt;');
  Add('begin');
  Add('  Insert(1,A,i);');
  CheckResolverException(sIncompatibleTypeArgNo,nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestStaticArray_DeleteFail;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TArrayInt = array[1..3] of integer;');
  Add('var');
  Add('  i: integer;');
  Add('  A: TArrayInt;');
  Add('begin');
  Add('  Delete(A,i,1);');
  CheckResolverException(sIncompatibleTypeArgNo,nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestArray_InsertItemMismatchFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TCaption = string;');
  Add('  TArrayCap = array of TCaption;');
  Add('var');
  Add('  i: longint;');
  Add('  A: TArrayCap;');
  Add('begin');
  Add('  Insert(i,{#a2_var}A,2);');
  CheckResolverException('Incompatible types: got "Longint" expected "String"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestArray_TypeCast;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TArrIntA = array of integer;');
  Add('  TArrIntB = array of longint;');
  Add('  TArrIntC = array of integer;');
  Add('var');
  Add('  a: TArrIntA;');
  Add('  b: TArrIntB;');
  Add('  c: TArrIntC;');
  Add('begin');
  Add('  a:=TArrIntA(a);');
  Add('  a:=TArrIntA(b);');
  Add('  a:=TArrIntA(c);');
  ParseProgram;
end;

procedure TTestResolver.TestArray_TypeCastWrongElTypeFail;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TArrInt = array of integer;');
  Add('  TArrStr = array of string;');
  Add('var');
  Add('  a: TArrInt;');
  Add('  s: TArrStr;');
  Add('begin');
  Add('  a:=TArrInt(s);');
  CheckResolverException('Illegal type conversion: "TArrStr" to "TArrInt"',
    nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestArray_ConstDynArrayWrite;
begin
  StartProgram(false);
  Add('type');
  Add('  TArrInt = array of longint;');
  Add('Procedure DoIt(const a: tarrint);');
  Add('begin');
  Add('  a[2]:=3;'); // FPC allows this for dynamic arrays
  Add('end;');
  Add('begin');
  ParseProgram;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestArray_ConstOpenArrayWriteFail;
begin
  StartProgram(false);
  Add('Procedure DoIt(const a: array of longint);');
  Add('begin');
  Add('  a[2]:=3;');
  Add('end;');
  Add('begin');
  CheckResolverException('Variable identifier expected',nVariableIdentifierExpected);
end;

procedure TTestResolver.TestArray_ForIn;
begin
  StartProgram(false);
  Add([
  '{$modeswitch arrayoperators}',
  'var',
  '  a: array of longint;',
  '  s: array[1,2] of longint;',
  '  i: longint;',
  'begin',
  '  for i in a do ;',
  '  for i in s do ;',
  '  for i in a+ {#a_array}[1] do ;',
  '  for i in {#b1_set}[1]+ {#b2_set}[2] do ;',
  '  for i in {#c_set}[1,2] do ;',
  '']);
  ParseProgram;
  CheckParamsExpr_pkSet_Markers;
end;

procedure TTestResolver.TestArray_Arg_AnonymousStaticFail;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(args: array[1..2] of word);',
  'begin',
  'end;',
  'begin']);
  CheckParserException('Expected "of"',nParserExpectTokenError);
end;

procedure TTestResolver.TestArray_Arg_AnonymousMultiDimFail;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(args: array of array of word);',
  'begin',
  'end;',
  'begin']);
  CheckParserException(SParserExpectedIdentifier,nParserExpectedIdentifier);
end;

procedure TTestResolver.TestArrayOfConst;
begin
  StartProgram(true,[supTVarRec]);
  Add([
  'type',
  '  TArrOfVarRec = array of TVarRec;',
  'procedure DoIt(args: array of const);',
  'var',
  '  i: longint;',
  '  v: TVarRec;',
  '  a: TArrOfVarRec;',
  '  sa: array[1..2] of TVarRec;',
  'begin',
  '  DoIt(args);',
  '  DoIt(a);',
  '  DoIt([]);',
  '  DoIt([1]);',
  '  DoIt([i]);',
  '  DoIt([true,''foo'',''c'',1.3,nil,@DoIt]);',
  '  for i:=low(args) to high(args) do begin',
  '    v:=args[i];',
  '    case args[i].VType of',
  '    vtInteger: if length(args)=args[i].VInteger then ;',
  '    end;',
  '  end;',
  '  for v in Args do ;',
  '  args:=sa;',
  'end;',
  'begin']);
  ParseProgram;
end;

procedure TTestResolver.TestArrayOfConst_PassDynArrayOfIntFail;
begin
  StartProgram(true,[supTVarRec]);
  Add([
  'type',
  '  TArr = array of word;',
  'procedure DoIt(args: array of const);',
  'begin',
  'end;',
  'var a: TArr;',
  'begin',
  '  DoIt(a)']);
  CheckResolverException('Incompatible type arg no. 1: Got "TArr", expected "array of const"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestArrayOfConst_AssignNilFail;
begin
  StartProgram(true,[supTVarRec]);
  Add([
  'type',
  '  TArr = array of word;',
  'procedure DoIt(args: array of const);',
  'begin',
  '  args:=nil;',
  'end;',
  'begin']);
  CheckResolverException('Incompatible types: got "Nil" expected "array of const"',nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestArrayOfConst_SetLengthFail;
begin
  StartProgram(true,[supTVarRec]);
  Add([
  'type',
  '  TArr = array of word;',
  'procedure DoIt(args: array of const);',
  'begin',
  '  SetLength(args,2);',
  'end;',
  'begin']);
  CheckResolverException('Incompatible type arg no. 1: Got "array of const", expected "string or dynamic array variable"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestArrayIntRange_OutOfRange;
begin
  StartProgram(false);
  Add([
  'type TArr = array[1..2] of longint;',
  'var a: TArr;',
  'begin',
  '  a[0]:=3;',
  '']);
  ParseProgram;
  CheckResolverHint(mtWarning,nRangeCheckEvaluatingConstantsVMinMax,
    'range check error while evaluating constants (0 is not between 1 and 2)');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestArrayIntRange_OutOfRangeError;
begin
  StartProgram(false);
  Add([
  '{$R+}',
  'type TArr = array[1..2] of longint;',
  'var a: TArr;',
  'begin',
  '  a[0]:=3;',
  '']);
  CheckResolverException('range check error while evaluating constants (0 is not between 1 and 2)',
    nRangeCheckEvaluatingConstantsVMinMax);
end;

procedure TTestResolver.TestArrayCharRange_OutOfRange;
begin
  StartProgram(false);
  Add([
  'type TArr = array[''a''..''b''] of longint;',
  'var a: TArr;',
  'begin',
  '  a[''0'']:=3;',
  '']);
  ParseProgram;
  CheckResolverHint(mtWarning,nRangeCheckEvaluatingConstantsVMinMax,
    'range check error while evaluating constants (''0'' is not between ''a'' and ''b'')');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestProcTypesAssignObjFPC;
begin
  StartProgram(false);
  Add('type');
  Add('  TProcedure = procedure;');
  Add('  TFunctionInt = function:longint;');
  Add('  TFunctionIntFunc = function:TFunctionInt;');
  Add('  TFunctionIntFuncFunc = function:TFunctionIntFunc;');
  Add('function GetNumber: longint;');
  Add('begin');
  Add('  Result:=3;');
  Add('end;');
  Add('function GetNumberFunc: TFunctionInt;');
  Add('begin');
  Add('  Result:=@GetNumber;');
  Add('end;');
  Add('function GetNumberFuncFunc: TFunctionIntFunc;');
  Add('begin');
  Add('  Result:=@GetNumberFunc;');
  Add('end;');
  Add('var');
  Add('  i: longint;');
  Add('  f: TFunctionInt;');
  Add('  ff: TFunctionIntFunc;');
  Add('begin');
  Add('  i:=GetNumber; // omit ()');
  Add('  i:=GetNumber();');
  Add('  i:=GetNumberFunc()();');
  Add('  i:=GetNumberFuncFunc()()();');
  Add('  if i=GetNumberFunc()() then ;');
  Add('  if GetNumberFunc()()=i then ;');
  Add('  if i=GetNumberFuncFunc()()() then ;');
  Add('  if GetNumberFuncFunc()()()=i then ;');
  Add('  f:=nil;');
  Add('  if f=nil then ;');
  Add('  if nil=f then ;');
  Add('  if Assigned(f) then ;');
  Add('  f:=f;');
  Add('  f:=@GetNumber;');
  Add('  f:=GetNumberFunc; // not in Delphi');
  Add('  f:=GetNumberFunc(); // not in Delphi');
  Add('  f:=GetNumberFuncFunc()();');
  Add('  if f=f then ;');
  Add('  if i=f then ;');
  Add('  if i=f() then ;');
  Add('  if f()=i then ;');
  Add('  if f()=f() then ;');
  Add('  if f=@GetNumber then ;');
  Add('  if @GetNumber=f then ;');
  Add('  if f=GetNumberFunc then ;');
  Add('  if f=GetNumberFunc() then ;');
  Add('  if f=GetNumberFuncFunc()() then ;');
  Add('  ff:=nil;');
  Add('  if ff=nil then ;');
  Add('  if nil=ff then ;');
  Add('  ff:=ff;');
  Add('  if ff=ff then ;');
  Add('  ff:=@GetNumberFunc;');
  Add('  ff:=GetNumberFuncFunc; // not in Delphi');
  Add('  ff:=GetNumberFuncFunc();');
  ParseProgram;
end;

procedure TTestResolver.TestMethodTypesAssignObjFPC;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class;');
  Add('  TNotifyEvent = procedure(Sender: TObject) of object;');
  Add('  TObject = class');
  Add('    FOnClick: TNotifyEvent;');
  Add('    procedure SetOnClick(const Value: TNotifyEvent);');
  Add('    procedure Notify(Sender: TObject);');
  Add('    property OnClick: TNotifyEvent read FOnClick write SetOnClick;');
  Add('  end;');
  Add('procedure TObject.SetOnClick(const Value: TNotifyEvent);');
  Add('begin');
  Add('  if FOnClick=Value then exit;');
  Add('  FOnClick:=Value;');
  Add('end;');
  Add('procedure TObject.Notify(Sender: TObject);');
  Add('begin');
  Add('  if Assigned(OnClick) and (OnClick<>@Notify) then begin');
  Add('    OnClick(Sender);');
  Add('    OnClick(Self);');
  Add('    Self.OnClick(nil);');
  Add('  end;');
  Add('  if OnClick=@Self.Notify then ;');
  Add('  if Self.OnClick=@Self.Notify then ;');
  Add('end;');
  Add('var o: TObject;');
  Add('begin');
  Add('  o.OnClick:=@o.Notify;');
  Add('  o.OnClick(nil);');
  Add('  o.OnClick(o);');
  Add('  o.SetOnClick(@o.Notify);');
  ParseProgram;
end;

procedure TTestResolver.TestProcTypeCall;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualImplicitCallWithoutParams: Boolean;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add('type');
  Add('  TFuncInt = function(vI: longint = 1):longint;');
  Add('  TFuncFuncInt = function(vI: longint = 1): TFuncInt;');
  Add('procedure DoI(vI: longint); begin end;');
  Add('procedure DoFConst(const vI: tfuncint); begin end;');
  Add('procedure DoFVar(var vI: tfuncint); begin end;');
  Add('procedure DoFDefault(vI: tfuncint); begin end;');
  Add('var');
  Add('  i: longint;');
  Add('  f: tfuncint;');
  Add('begin');
  Add('  {#a}f;');
  Add('  {#b}f();');
  Add('  {#c}f(2);');
  Add('  i:={#d}f;');
  Add('  i:={#e}f();');
  Add('  i:={#f}f(2);');
  Add('  doi({#g}f);');
  Add('  doi({#h}f());');
  Add('  doi({#i}f(2));');
  Add('  dofconst({#j}f);');
  Add('  if Assigned({#k}f) then;');
  Add('  if {#l}f=nil then;');
  Add('  if nil={#m}f then;');
  ParseProgram;

  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestProcTypeCall ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualImplicitCallWithoutParams:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestProcTypeCall ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        //writeln('TTestResolver.TestProcTypeCall ',GetObjName(Ref.Declaration),' rrfImplicitCallWithoutParams=',rrfImplicitCallWithoutParams in Ref.Flags);
        if rrfImplicitCallWithoutParams in Ref.Flags then
          ActualImplicitCallWithoutParams:=true;
        break;
        end;
      case aMarker^.Identifier of
      'a','d','g':
        if not ActualImplicitCallWithoutParams then
          RaiseErrorAtSrcMarker('expected implicit call at "#'+aMarker^.Identifier+'"',aMarker);
      else
        if ActualImplicitCallWithoutParams then
          RaiseErrorAtSrcMarker('expected no implicit call at "#'+aMarker^.Identifier+'"',aMarker);
      end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestProcType_FunctionFPC;
begin
  StartProgram(false);
  Add('type');
  Add('  TFuncInt = function(vA: longint = 1): longint;');
  Add('function DoIt(vI: longint): longint;');
  Add('begin end;');
  Add('var');
  Add('  b: boolean;');
  Add('  vP, vQ: tfuncint;');
  Add('begin');
  Add('  vp:=nil;');
  Add('  vp:=vp;');
  Add('  vp:=@doit;'); // ok in fpc and delphi
  //Add('  vp:=doit;'); // illegal in fpc, ok in delphi
  Add('  vp;'); // ok in fpc and delphi
  Add('  vp();');
  Add('  vp(2);');
  Add('  b:=vp=nil;'); // ok in fpc, illegal in delphi
  Add('  b:=nil=vp;'); // ok in fpc, illegal in delphi
  Add('  b:=vp=vq;'); // in fpc compare proctypes, in delphi compare results
  Add('  b:=vp=@doit;'); // ok in fpc, illegal in delphi
  Add('  b:=@doit=vp;'); // ok in fpc, illegal in delphi
  //Add('  b:=vp=3;'); // illegal in fpc, ok in delphi
  Add('  b:=4=vp;'); // illegal in fpc, ok in delphi
  Add('  b:=vp<>nil;'); // ok in fpc, illegal in delphi
  Add('  b:=nil<>vp;'); // ok in fpc, illegal in delphi
  Add('  b:=vp<>vq;'); // in fpc compare proctypes, in delphi compare results
  Add('  b:=vp<>@doit;'); // ok in fpc, illegal in delphi
  Add('  b:=@doit<>vp;'); // ok in fpc, illegal in delphi
  //Add('  b:=vp<>5;'); // illegal in fpc, ok in delphi
  Add('  b:=6<>vp;'); // illegal in fpc, ok in delphi
  Add('  b:=Assigned(vp);');
  //Add('  doit(vp);'); // illegal in fpc, ok in delphi
  Add('  doit(vp());'); // ok in fpc and delphi
  Add('  doit(vp(2));'); // ok in fpc and delphi
  ParseProgram;
end;

procedure TTestResolver.TestProcType_FunctionDelphi;
begin
  StartProgram(false);
  Add('{$mode Delphi}');
  Add('type');
  Add('  TFuncInt = function(vA: longint = 1): longint;');
  Add('function DoIt(vI: longint): longint;');
  Add('begin end;');
  Add('var');
  Add('  b: boolean;');
  Add('  vP, vQ: tfuncint;');
  Add('  ');
  Add('begin');
  Add('  vp:=nil;');
  Add('  vp:=vp;');
  Add('  vp:=@doit;'); // ok in fpc and delphi
  Add('  vp:=doit;'); // illegal in fpc, ok in delphi
  Add('  vp;'); // ok in fpc and delphi
  Add('  vp();');
  Add('  vp(2);');
  //Add('  b:=vp=nil;'); // ok in fpc, illegal in delphi
  //Add('  b:=nil=vp;'); // ok in fpc, illegal in delphi
  Add('  b:=vp=vq;'); // in fpc compare proctypes, in delphi compare results
  //Add('  b:=vp=@doit;'); // ok in fpc, illegal in delphi
  //Add('  b:=@doit=vp;'); // ok in fpc, illegal in delphi
  Add('  b:=vp=3;'); // illegal in fpc, ok in delphi
  Add('  b:=4=vp;'); // illegal in fpc, ok in delphi
  //Add('  b:=vp<>nil;'); // ok in fpc, illegal in delphi
  //Add('  b:=nil<>vp;'); // ok in fpc, illegal in delphi
  Add('  b:=vp<>vq;'); // in fpc compare proctypes, in delphi compare results
  //Add('  b:=vp<>@doit;'); // ok in fpc, illegal in delphi
  //Add('  b:=@doit<>vp;'); // ok in fpc, illegal in delphi
  Add('  b:=vp<>5;'); // illegal in fpc, ok in delphi
  Add('  b:=6<>vp;'); // illegal in fpc, ok in delphi
  Add('  b:=Assigned(vp);');
  Add('  doit(vp);'); // illegal in fpc, ok in delphi
  Add('  doit(vp());'); // ok in fpc and delphi
  Add('  doit(vp(2));'); // ok in fpc and delphi  *)
  ParseProgram;
end;

procedure TTestResolver.TestProcType_ProcedureDelphi;
begin
  StartProgram(false);
  Add('{$mode Delphi}');
  Add('type');
  Add('  TProc = procedure;');
  Add('procedure Doit;');
  Add('begin end;');
  Add('var');
  Add('  b: boolean;');
  Add('  vP, vQ: tproc;');
  Add('begin');
  Add('  vp:=nil;');
  Add('  vp:=vp;');
  Add('  vp:=vq;');
  Add('  vp:=@doit;'); // ok in fpc and delphi, Note that in Delphi type of @F is Pointer, while in FPC it is the proc type
  Add('  vp:=doit;'); // illegal in fpc, ok in delphi
  //Add('  vp:=@doit;'); // illegal in fpc, ok in delphi (because Delphi treats @F as Pointer), not supported by resolver
  Add('  vp;'); // ok in fpc and delphi
  Add('  vp();');

  // equal
  //Add('  b:=vp=nil;'); // ok in fpc, illegal in delphi
  Add('  b:=@@vp=nil;'); // ok in fpc delphi mode, ok in delphi
  //Add('  b:=nil=vp;'); // ok in fpc, illegal in delphi
  Add('  b:=nil=@@vp;'); // ok in fpc delphi mode, ok in delphi
  Add('  b:=@@vp=@@vq;'); // ok in fpc delphi mode, ok in Delphi
  //Add('  b:=vp=vq;'); // in fpc compare proctypes, in delphi compare results
  //Add('  b:=vp=@doit;'); // ok in fpc, illegal in delphi
  Add('  b:=@@vp=@doit;'); // ok in fpc delphi mode, ok in delphi
  //Add('  b:=@doit=vp;'); // ok in fpc, illegal in delphi
  Add('  b:=@doit=@@vp;'); // ok in fpc delphi mode, ok in delphi

  // unequal
  //Add('  b:=vp<>nil;'); // ok in fpc, illegal in delphi
  Add('  b:=@@vp<>nil;'); // ok in fpc mode delphi, ok in delphi
  //Add('  b:=nil<>vp;'); // ok in fpc, illegal in delphi
  Add('  b:=nil<>@@vp;'); // ok in fpc mode delphi, ok in delphi
  //Add('  b:=vp<>vq;'); // in fpc compare proctypes, in delphi compare results
  Add('  b:=@@vp<>@@vq;'); // ok in fpc mode delphi, ok in delphi
  //Add('  b:=vp<>@doit;'); // ok in fpc, illegal in delphi
  Add('  b:=@@vp<>@doit;'); // ok in fpc mode delphi, illegal in delphi
  //Add('  b:=@doit<>vp;'); // ok in fpc, illegal in delphi
  Add('  b:=@doit<>@@vp;'); // ok in fpc mode delphi, illegal in delphi

  Add('  b:=Assigned(vp);');
  ParseProgram;
end;

procedure TTestResolver.TestProcType_MethodFPC;
begin
  StartProgram(false);
  Add('type');
  Add('  TFuncInt = function(vA: longint = 1): longint of object;');
  Add('  TObject = class');
  Add('    function DoIt(vA: longint = 1): longint;');
  Add('  end;');
  Add('function tobject.doit(vA: longint): longint;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  Obj: TObject;');
  Add('  vP: tfuncint;');
  Add('  b: boolean;');
  Add('begin');
  Add('  vp:=@obj.doit;'); // ok in fpc and delphi
  //Add('  vp:=obj.doit;'); // illegal in fpc, ok in delphi
  Add('  vp;'); // ok in fpc and delphi
  Add('  vp();');
  Add('  vp(2);');
  Add('  b:=vp=@obj.doit;'); // ok in fpc, illegal in delphi
  Add('  b:=@obj.doit=vp;'); // ok in fpc, illegal in delphi
  Add('  b:=vp<>@obj.doit;'); // ok in fpc, illegal in delphi
  Add('  b:=@obj.doit<>vp;'); // ok in fpc, illegal in delphi
  ParseProgram;
end;

procedure TTestResolver.TestProcType_MethodDelphi;
begin
  StartProgram(false);
  Add('{$mode delphi}');
  Add('type');
  Add('  TFuncInt = function(vA: longint = 1): longint of object;');
  Add('  TObject = class');
  Add('    function DoIt(vA: longint = 1): longint;');
  Add('  end;');
  Add('function tobject.doit(vA: longint): longint;');
  Add('begin');
  Add('end;');
  Add('var');
  Add('  Obj: TObject;');
  Add('  vP: tfuncint;');
  Add('  b: boolean;');
  Add('begin');
  Add('  vp:=@obj.doit;'); // ok in fpc and delphi
  Add('  vp:=obj.doit;'); // illegal in fpc, ok in delphi
  Add('  vp;'); // ok in fpc and delphi
  Add('  vp();');
  Add('  vp(2);');
  //Add('  b:=vp=@obj.doit;'); // ok in fpc, illegal in delphi
  //Add('  b:=@obj.doit=vp;'); // ok in fpc, illegal in delphi
  //Add('  b:=vp<>@obj.doit;'); // ok in fpc, illegal in delphi
  //Add('  b:=@obj.doit<>vp;'); // ok in fpc, illegal in delphi
  ParseProgram;
end;

procedure TTestResolver.TestAssignProcToMethodFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class end;');
  Add('  TNotifyEvent = procedure(Sender: TObject) of object;');
  Add('procedure ProcA(Sender: TObject);');
  Add('begin end;');
  Add('var n: TNotifyEvent;');
  Add('begin');
  Add('  n:=@ProcA;');
  CheckResolverException('procedural type modifier "of Object" mismatch',
    nXModifierMismatchY);
end;

procedure TTestResolver.TestAssignMethodToProcFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure ProcA(Sender: TObject);');
  Add('  end;');
  Add('  TNotifyProc = procedure(Sender: TObject);');
  Add('procedure TObject.ProcA(Sender: TObject);');
  Add('begin end;');
  Add('var');
  Add('  n: TNotifyProc;');
  Add('  o: TObject;');
  Add('begin');
  Add('  n:=@o.ProcA;');
  CheckResolverException('procedural type modifier "of Object" mismatch',
    nXModifierMismatchY);
end;

procedure TTestResolver.TestAssignProcToFunctionFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TFuncInt = function(i: longint): longint;');
  Add('procedure ProcA(i: longint);');
  Add('begin end;');
  Add('var p: TFuncInt;');
  Add('begin');
  Add('  p:=@ProcA;');
  CheckResolverException(
    'Incompatible types: got "procedural type" expected "functional type"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestAssignProcWrongArgsFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TProcInt = procedure(i: longint);');
  Add('procedure ProcA(i: string);');
  Add('begin end;');
  Add('var p: TProcInt;');
  Add('begin');
  Add('  p:=@ProcA;');
  CheckResolverException('Incompatible type arg no. 1: Got "Longint", expected "String"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestAssignProcWrongArgAccessFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TProcInt = procedure(i: longint);');
  Add('procedure ProcA(const i: longint);');
  Add('begin end;');
  Add('var p: TProcInt;');
  Add('begin');
  Add('  p:=@ProcA;');
  CheckResolverException('Incompatible type arg no. 1: Got "access modifier const", expected "default"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestProcType_AssignNestedProcFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TProcInt = procedure(i: longint);');
  Add('procedure ProcA;');
  Add('var p: TProcInt;');
  Add('  procedure SubProc(i: longint);');
  Add('  begin');
  Add('  end;');
  Add('begin');
  Add('  p:=@SubProc;');
  Add('end;');
  Add('begin');
  CheckResolverException('procedural type modifier "is nested" mismatch',
    nXModifierMismatchY);
end;

procedure TTestResolver.TestArrayOfProc;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TNotifyProc = function(Sender: TObject = nil): longint;',
  '  TProcArray = array of TNotifyProc;',
  'function ProcA(Sender: TObject): longint;',
  'begin end;',
  'procedure DoIt(const a: TProcArray);',
  'begin end;',
  'var',
  '  a: TProcArray;',
  '  p: TNotifyProc;',
  'begin',
  '  a[0]:=@ProcA;',
  '  if a[1]=@ProcA then ;',
  '  if @ProcA=a[2] then ;',
  // '  a[3];', ToDo
  '  a[3](nil);',
  '  if a[4](nil)=5 then ;',
  '  if 6=a[7](nil) then ;',
  '  a[8]:=a[9];',
  '  p:=a[10];',
  '  a[11]:=p;',
  '  if a[12]=p then ;',
  '  if p=a[13] then ;',
  '  DoIt([@ProcA]);',
  '  DoIt([nil]);',
  '  DoIt([nil,@ProcA]);',
  '  DoIt([p]);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcType_Assigned;
begin
  StartProgram(false);
  Add('type');
  Add('  TFuncInt = function(i: longint): longint;');
  Add('function ProcA(i: longint): longint;');
  Add('begin end;');
  Add('var');
  Add('  a: array of TFuncInt;');
  Add('  p: TFuncInt;');
  Add('begin');
  Add('  if Assigned(p) then ;');
  Add('  if Assigned(a[1]) then ;');
  ParseProgram;
end;

procedure TTestResolver.TestProcType_TNotifyEvent;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TNotifyEvent = procedure(Sender: TObject) of object;');
  Add('  TButton = class(TObject)');
  Add('  private');
  Add('    FOnClick: TNotifyEvent;');
  Add('  published');
  Add('    property OnClick: TNotifyEvent read FOnClick write FOnClick;');
  Add('  end;');
  Add('  TApplication = class(TObject)');
  Add('    procedure BtnClickHandler(Sender: TObject); external name ''BtnClickHandler'';');
  Add('  end;');
  Add('var ');
  Add('  App: TApplication;');
  Add('  Button1: TButton;');
  Add('begin');
  Add('  Button1.OnClick := @App.BtnClickHandler;');
  ParseProgram;
end;

procedure TTestResolver.TestProcType_TNotifyEvent_NoAtFPC_Fail1;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TNotifyEvent = procedure(Sender: TObject) of object;');
  Add('  TButton = class(TObject)');
  Add('  private');
  Add('    FOnClick: TNotifyEvent;');
  Add('  published');
  Add('    property OnClick: TNotifyEvent read FOnClick write FOnClick;');
  Add('  end;');
  Add('  TApplication = class(TObject)');
  Add('    procedure BtnClickHandler(Sender: TObject); external name ''BtnClickHandler'';');
  Add('  end;');
  Add('var ');
  Add('  App: TApplication;');
  Add('  Button1: TButton;');
  Add('begin');
  Add('  Button1.OnClick := App.BtnClickHandler;');
  CheckResolverException(
    'Wrong number of parameters specified for call to "BtnClickHandler"',
    nWrongNumberOfParametersForCallTo);
end;

procedure TTestResolver.TestProcType_TNotifyEvent_NoAtFPC_Fail2;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TNotifyEvent = procedure(Sender: TObject) of object;');
  Add('  TButton = class(TObject)');
  Add('  private');
  Add('    FOnClick: TNotifyEvent;');
  Add('  published');
  Add('    property OnClick: TNotifyEvent read FOnClick write FOnClick;');
  Add('  end;');
  Add('  TApplication = class(TObject)');
  Add('    procedure BtnClickHandler(Sender: TObject); external name ''BtnClickHandler'';');
  Add('  end;');
  Add('var ');
  Add('  App: TApplication;');
  Add('  Button1: TButton;');
  Add('begin');
  Add('  Button1.OnClick := App.BtnClickHandler();');
  CheckResolverException(
    'Wrong number of parameters specified for call to "procedure BtnClickHandler(TObject) of Object"',
    nWrongNumberOfParametersForCallTo);
end;

procedure TTestResolver.TestProcType_TNotifyEvent_NoAtFPC_Fail3;
begin
  StartProgram(true,[supTObject]);
  Add('type');
  Add('  TNotifyEvent = procedure(Sender: TObject) of object;');
  Add('  TButton = class(TObject)');
  Add('  private');
  Add('    FOnClick: TNotifyEvent;');
  Add('  published');
  Add('    property OnClick: TNotifyEvent read FOnClick write FOnClick;');
  Add('  end;');
  Add('  TApplication = class(TObject)');
  Add('    procedure BtnClickHandler(Sender: TObject); external name ''BtnClickHandler'';');
  Add('  end;');
  Add('var ');
  Add('  App: TApplication;');
  Add('  Button1: TButton;');
  Add('begin');
  Add('  Button1.OnClick := @App.BtnClickHandler();');
  CheckResolverException(
    'Wrong number of parameters specified for call to "procedure BtnClickHandler(TObject) of Object"',
    nWrongNumberOfParametersForCallTo);
end;

procedure TTestResolver.TestProcType_WhileListCompare;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TArrInt = array of Integer;');
  Add('  TListCompare = function(Item1, Item2: Integer): integer;');
  Add('procedure Sort(P: Integer; const List: TArrInt; const Compare: TListCompare);');
  Add('begin');
  Add('  while Compare(P,List[0])>0 do ;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestProcType_IsNested;
begin
  StartProgram(false);
  Add('{$modeswitch nestedprocvars}');
  Add('type');
  Add('  integer = longint;');
  Add('  TNestedProc = procedure(i: integer) is nested;');
  Add('procedure DoIt(i: integer);');
  Add('var p: TNestedProc;');
  Add('  procedure Sub(i: integer);');
  Add('  var SubP: TNestedProc;');
  Add('    procedure SubSub(i: integer);');
  Add('    begin');
  Add('      p:=@Sub;');
  Add('      p:=@SubSub;');
  Add('      SubP:=@Sub;');
  Add('      SubP:=@SubSub;');
  Add('    end;');
  Add('  begin');
  Add('    p:=@Sub;');
  Add('    p:=@SubSub;');
  Add('    SubP:=@Sub;');
  Add('    SubP:=@SubSub;');
  Add('  end;');
  Add('begin');
  Add('  p:=@Sub;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestProcType_IsNested_AssignProcFail;
begin
  StartProgram(false);
  Add('{$modeswitch nestedprocvars}');
  Add('type');
  Add('  integer = longint;');
  Add('  TNestedProc = procedure(i: integer) is nested;');
  Add('procedure DoIt(i: integer); begin end;');
  Add('var p: TNestedProc;');
  Add('begin');
  Add('  p:=@DoIt;');
  CheckResolverException('procedural type modifier "is nested" mismatch',nXModifierMismatchY);
end;

procedure TTestResolver.TestProcType_ReferenceTo;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProcRef = reference to procedure(i: longint = 0);',
  '  TFuncRef = reference to function(i: longint = 0): longint;',
  '  TObject = class',
  '    function Grow(s: longint): longint;',
  '  end;',
  'var',
  '  p: TProcRef;',
  '  f: TFuncRef;',
  'function tobject.Grow(s: longint): longint;',
  '  function GrowSub(i: longint): longint;',
  '  begin',
  '    f:=@Grow;',
  '    f:=@GrowSub;',
  '    f;',
  '    f();',
  '    f(1);',
  '  end;',
  'begin',
  '  f:=@Grow;',
  '  f:=@GrowSub;',
  '  f;',
  '  f();',
  '  f(1);',
  'end;',
  'procedure DoIt(i: longint);',
  'begin',
  'end;',
  'function GetIt(i: longint): longint;',
  '  function Sub(i: longint): longint;',
  '  begin',
  '    p:=@DoIt;',
  '    f:=@GetIt;',
  '    f:=@Sub;',
  '  end;',
  'begin',
  '  p:=@DoIt;',
  '  f:=@GetIt;',
  '  f;',
  '  f();',
  '  f(1);',
  'end;',
  'begin',
  '  p:=@DoIt;',
  '  f:=@GetIt;',
  '  f;',
  '  f();',
  '  f(1);',
  '  p:=TProcRef(f);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcType_AllowNested;
begin
  ResolverEngine.Options:=ResolverEngine.Options+[proProcTypeWithoutIsNested];
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TProc = procedure(i: integer);');
  Add('procedure DoIt(i: integer);');
  Add('var p: TProc;');
  Add('  procedure Sub(i: integer);');
  Add('  var SubP: TProc;');
  Add('    procedure SubSub(i: integer);');
  Add('    begin');
  Add('      p:=@DoIt;');
  Add('      p:=@Sub;');
  Add('      p:=@SubSub;');
  Add('      SubP:=@DoIt;');
  Add('      SubP:=@Sub;');
  Add('      SubP:=@SubSub;');
  Add('    end;');
  Add('  begin');
  Add('    p:=@DoIt;');
  Add('    p:=@Sub;');
  Add('    p:=@SubSub;');
  Add('    SubP:=@DoIt;');
  Add('    SubP:=@Sub;');
  Add('    SubP:=@SubSub;');
  Add('  end;');
  Add('begin');
  Add('  p:=@DoIt;');
  Add('  p:=@Sub;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestProcType_AllowNestedOfObject;
begin
  ResolverEngine.Options:=ResolverEngine.Options+[proProcTypeWithoutIsNested];
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TMethodProc = procedure(i: integer) of object;');
  Add('  TObject = class');
  Add('    procedure DoIt(i: integer);');
  Add('  end;');
  Add('procedure TObject.DoIt(i: integer);');
  Add('var p: TMethodProc;');
  Add('  procedure Sub(i: integer);');
  Add('  var SubP: TMethodProc;');
  Add('    procedure SubSub(i: integer);');
  Add('    begin');
  Add('      p:=@DoIt;');
  Add('      p:=@Sub;');
  Add('      p:=@SubSub;');
  Add('      SubP:=@DoIt;');
  Add('      SubP:=@Sub;');
  Add('      SubP:=@SubSub;');
  Add('    end;');
  Add('  begin');
  Add('    p:=@DoIt;');
  Add('    p:=@Sub;');
  Add('    p:=@SubSub;');
  Add('    SubP:=@DoIt;');
  Add('    SubP:=@Sub;');
  Add('    SubP:=@SubSub;');
  Add('  end;');
  Add('begin');
  Add('  p:=@DoIt;');
  Add('  p:=@Sub;');
  Add('end;');
  Add('begin');
  ParseProgram;
end;

procedure TTestResolver.TestProcType_AsArgOtherUnit;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'type',
    '  JSInteger = longint;',
    '  TObject = class;',
    '  TJSArrayCallBack = function (element : JSInteger) : Boolean;',
    '  TObject = class',
    '  public',
    '    procedure forEach(const aCallBack : TJSArrayCallBack); virtual; abstract;',
    '  end;',
    '']),
    '');
  StartProgram(true);
  Add('uses unit2;');
  Add('function showElement(el : JSInteger) : boolean  ;');
  Add('begin');
  Add('  result:=true;');
  Add('end;');
  Add('var a: TObject;');
  Add('begin');
  Add('  a.forEach(@ShowElement);');
  ParseProgram;
end;

procedure TTestResolver.TestProcType_Property;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TNotifyEvent = procedure(Sender: TObject) of object;',
  '  TControl = class',
  '    FOnClick: TNotifyEvent;',
  '    property OnClick: TNotifyEvent read FOnClick write FOnClick;',
  '    procedure Click(Sender: TObject);',
  '  end;',
  '  TButton = class(TControl)',
  '    property OnClick;',
  '  end;',
  'procedure TControl.Click(Sender: TObject);',
  'begin',
  '  if Assigned(OnClick) then ;',
  '  OnClick:=@Click;',
  '  OnClick(Sender);',
  '  Self.OnClick(Sender);',
  '  with Self do OnClick(Sender);',
  'end;',
  'var',
  '  Ctrl: TControl;',
  '  Btn: TButton;',
  'begin',
  '  if Assigned(Ctrl.OnClick) then ;',
  '  Ctrl.OnClick(Ctrl);',
  '  with Ctrl do OnClick(Ctrl);',
  '  if Assigned(Btn.OnClick) then ;',
  '  Btn.OnClick(Btn);',
  '  with Btn do OnClick(Btn);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcType_PropertyCallWrongArgFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class end;');
  Add('  TNotifyEvent = procedure(Sender: TObject) of object;');
  Add('  TControl = class');
  Add('    FOnClick: TNotifyEvent;');
  Add('    property OnClick: TNotifyEvent read FOnClick write FOnClick;');
  Add('  end;');
  Add('var Btn: TControl;');
  Add('begin');
  Add('  Btn.OnClick(3);');
  CheckResolverException('Incompatible type arg no. 1: Got "Longint", expected "TObject"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestProcType_Typecast;
begin
  StartProgram(false);
  Add('type');
  Add('  TNotifyEvent = procedure(Sender: Pointer) of object;');
  Add('  TEvent = procedure of object;');
  Add('  TProcA = procedure(i: longint);');
  Add('  TFuncB = function(i, j: longint): longint;');
  Add('var');
  Add('  Notify: TNotifyEvent;');
  Add('  Event: TEvent;');
  Add('  ProcA: TProcA;');
  Add('  FuncB: TFuncB;');
  Add('  p: pointer;');
  Add('begin');
  Add('  Notify:=TNotifyEvent(Event);');
  Add('  Event:=TEvent(Event);');
  Add('  Event:=TEvent(Notify);');
  Add('  ProcA:=TProcA(FuncB);');
  Add('  FuncB:=TFuncB(FuncB);');
  Add('  FuncB:=TFuncB(ProcA);');
  Add('  ProcA:=TProcA(p);');
  Add('  FuncB:=TFuncB(p);');
  ParseProgram;
end;

procedure TTestResolver.TestProcType_InsideFunction;
begin
  StartProgram(false);
  Add([
  'function GetIt: longint;',
  'type TGetter = function: longint;',
  'var',
  '  p: Pointer;',
  'begin',
  '  Result:=TGetter(p)();',
  'end;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestProcType_PassProcToUntyped;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualImplicitCallWithoutParams: Boolean;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEvent = procedure of object;',
  '  TFunc = function: longint of object;',
  'procedure DoIt; varargs; begin end;',
  'procedure DoSome(const a; var b; c: pointer); begin end;',
  'var',
  '  E: TEvent;',
  '  F: TFunc;',
  'begin',
  '  DoIt({#a1}E,{#a2}F);',
  '  DoSome({#b1}E,{#b2}E,{#b3}E);',
  '  DoSome({#c1}F,{#c2}F,{#c3}F);',
  '']);
  ParseProgram;

  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestProcType_PassProcToUntyped ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualImplicitCallWithoutParams:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestProcType_PassProcToUntyped ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        //writeln('TTestResolver.TestProcType_PassProcToUntyped ',GetObjName(Ref.Declaration),' rrfImplicitCallWithoutParams=',rrfImplicitCallWithoutParams in Ref.Flags);
        if rrfImplicitCallWithoutParams in Ref.Flags then
          ActualImplicitCallWithoutParams:=true;
        break;
        end;
      if ActualImplicitCallWithoutParams then
        RaiseErrorAtSrcMarker('expected no implicit call at "#'+aMarker^.Identifier+'"',aMarker);
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestPointer;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TClass = class of TObject;',
  '  TMyPtr = pointer;',
  '  TArrInt = array of longint;',
  '  TFunc = function: longint;',
  'procedure DoIt; begin end;',
  'var',
  '  p: TMyPtr;',
  '  Obj: TObject;',
  '  Cl: TClass;',
  '  a: tarrint;',
  '  f: TFunc;',
  '  s: string;',
  '  u: unicodestring;',
  'begin',
  '  p:=nil;',
  '  if p=nil then;',
  '  if nil=p then;',
  '  if Assigned(p) then;',
  '  p:=obj;',
  '  p:=cl;',
  '  p:=a;',
  '  p:=Pointer(f);',
  '  p:=@DoIt;',
  '  p:=Pointer(@DoIt);',
  '  obj:=TObject(p);',
  '  cl:=TClass(p);',
  '  a:=TArrInt(p);',
  '  p:=Pointer(a);',
  '  p:=Pointer(s);',
  '  s:=String(p);',
  '  p:=pointer(u);',
  '  u:=UnicodeString(p);']);
  ParseProgram;
end;

procedure TTestResolver.TestPointer_AnonymousSetFail;
begin
  StartProgram(false);
  Add([
  'type p = ^(red, green);',
  'begin']);
  CheckParserException('Expected "Identifier" at token "(" in file afile.pp at line 2 column 11',
    nParserExpectTokenError);
end;

procedure TTestResolver.TestPointer_AssignPointerToClassFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class end;');
  Add('var');
  Add('  Obj: TObject;');
  Add('  p: pointer;');
  Add('begin');
  Add('  obj:=p;');
  CheckResolverException('Incompatible types: got "Pointer" expected "TObject"',
    nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestPointer_TypecastToMethodTypeFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TEvent = procedure of object;');
  Add('var');
  Add('  p: pointer;');
  Add('  e: TEvent;');
  Add('begin');
  Add('  e:=TEvent(p);');
  CheckResolverException('Illegal type conversion: "Pointer" to "procedure type of Object"',
    nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestPointer_TypecastFromMethodTypeFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TEvent = procedure of object;');
  Add('var');
  Add('  p: pointer;');
  Add('  e: TEvent;');
  Add('begin');
  Add('  p:=Pointer(e);');
  CheckResolverException('Illegal type conversion: "procedural type of Object" to "Pointer"',
    nIllegalTypeConversionTo);
end;

procedure TTestResolver.TestPointer_TypecastMethod_proMethodAddrAsPointer;
begin
  ResolverEngine.Options:=ResolverEngine.Options+[proMethodAddrAsPointer];
  StartProgram(false);
  Add('type');
  Add('  TEvent = procedure of object;');
  Add('var');
  Add('  p: pointer;');
  Add('  e: TEvent;');
  Add('begin');
  Add('  e:=TEvent(p);');
  Add('  p:=Pointer(e);');
  ParseProgram;
end;

procedure TTestResolver.TestPointer_OverloadSignature;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class end;');
  Add('  TClass = class of TObject;');
  Add('  TBird = class(TObject) end;');
  Add('  TBirds = class of TBird;');
  Add('procedure {#pointer}DoIt(p: Pointer); begin end;');
  Add('procedure {#tobject}DoIt(o: TObject); begin end;');
  Add('procedure {#tclass}DoIt(c: TClass); begin end;');
  Add('var');
  Add('  p: pointer;');
  Add('  o: TObject;');
  Add('  c: TClass;');
  Add('  b: TBird;');
  Add('  bc: TBirds;');
  Add('begin');
  Add('  {@pointer}DoIt(p);');
  Add('  {@tobject}DoIt(o);');
  Add('  {@tclass}DoIt(c);');
  Add('  {@tobject}DoIt(b);');
  Add('  {@tclass}DoIt(bc);');
  ParseProgram;
end;

procedure TTestResolver.TestPointer_Assign;
begin
  StartProgram(false);
  Add([
  'type',
  '  TPtr = pointer;',
  '  TClass = class of TObject;',
  '  TObject = class end;',
  'var',
  '  p: TPtr;',
  '  o: TObject;',
  '  c: TClass;',
  'begin',
  '  p:=o;',
  '  if p=o then ;',
  '  if o=p then ;',
  '  p:=c;',
  '  if p=c then ;',
  '  if c=p then ;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestPointerTyped;
begin
  StartProgram(false);
  Add([
  'type',
  '  PBoolean = ^boolean;',
  '  PPInteger = ^PInteger;',
  '  PInteger = ^integer;',
  '  integer = longint;',
  'var',
  '  i: integer;',
  '  p1: PInteger;',
  '  p2: ^Integer;',
  '  p3: ^PInteger;',
  '  a: array of integer;',
  'begin',
  '  p1:=@i;',
  '  p1:=p2;',
  '  p2:=@i;',
  '  p3:=@p1;',
  '  p1:=@a[1];',
  '  p1^:=i;',
  '  i:=(@i)^;',
  '  i:=p1^;',
  '  i:=p2^;',
  '  i:=p3^^;',
  '  i:=PInteger(p3)^;',
  '  if p1=@i then ;',
  '  if @i=p1 then ;',
  '  if p1=p2 then ;',
  '  if p2=p1 then ;',
  '  if p2=@i then ;',
  '  if @i=p2 then ;',
  '  if p1=@a[2] then ;',
  '  if @a[3]=p1 then ;',
  '  if i=p1^ then ;',
  '  if p1^=i then ;',
  '  i:=p1[1];',
  '  i:=(@i)[1];',
  '  i:=p2[2];',
  '  i:=p3[3][4];',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestPointerTypedForwardMissingFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  PInteger = ^integer;',
  'var',
  '  i: integer;',
  'begin',
  '']);
  CheckResolverException('identifier not found "integer"',nIdentifierNotFound);
end;

procedure TTestResolver.TestPointerTyped_CycleFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  PInteger = ^integer;',
  '  integer = PInteger;',
  'var',
  '  i: integer;',
  '  p1: PInteger;',
  'begin',
  '']);
  CheckResolverException(sTypeCycleFound,nTypeCycleFound);
end;

procedure TTestResolver.TestPointerTyped_AssignMismatchFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  PInt = ^longint;',
  '  PBool = ^boolean;',
  'var',
  '  pi: Pint;',
  '  pb: PBool;',
  'begin',
  '  pi:=pb;',
  '']);
  CheckResolverException('Incompatible types: got "PBool" expected "PInt"',nIncompatibleTypesGotExpected);
end;

procedure TTestResolver.TestPointerTyped_AddrAddrFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  PInt = ^longint;',
  '  PPInt = ^PInt;',
  'var',
  '  i: longint;',
  '  p: PPint;',
  'begin',
  '  p:=@(@i);',
  '']);
  CheckResolverException('illegal qualifier "@" in front of "Pointer"',nIllegalQualifierInFrontOf);
end;

procedure TTestResolver.TestPointerTyped_RecordObjFPC;
begin
  StartProgram(false);
  Add([
  'type',
  '  PRec = ^TRec;',
  '  TRec = record x: longint; end;',
  'var',
  '  r: TRec;',
  '  p: PRec;',
  '  i: longint;',
  '  Ptr: pointer;',
  'begin',
  '  p:=@r;',
  '  i:=p^.x;',
  '  p^.x:=i;',
  '  if i=p^.x then;',
  '  if p^.x=i then;',
  '  ptr:=p;',
  '  p:=PRec(ptr);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestPointerTyped_RecordDelphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  PRec = ^TRec;',
  '  TRec = record x: longint; end;',
  'procedure DoIt(const p: PRec);',
  'begin',
  '  p.x:=p.x;',
  '  with p^ do',
  '    x:=x;',
  'end;',
  'var',
  '  r: TRec;',
  '  p: PRec;',
  '  i: longint;',
  'begin',
  '  i:=p.x;',
  '  p.x:=i;',
  '  if i=p.x then;',
  '  if p.x=i then;',
  '  DoIt(@r);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestPointerTyped_Arithmetic;
begin
  StartProgram(false);
  Add([
  'type',
  '  PInt = ^longint;',
  'var',
  '  i: longint;',
  '  p: PInt;',
  'begin',
  '  inc(p);',
  '  inc(p,2);',
  '  p:=p+3;',
  '  p:=4+p;',
  '  p:=@i+5;',
  '  p:=6+@i;',
  '  i:=(p+7)^;',
  '  i:=(@i+8)^;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestResourcestring;
begin
  StartProgram(false);
  Add([
  'const Foo = ''foo'';',
  'Resourcestring',
  '  Bar = foo;',
  '  Red = ''Red'';',
  '  r = ''Rd''+foo;',
  'procedure DoIt(s: string; const h: string); begin end;',
  'begin',
  '  if bar=red then ;',
  '  if bar=''a'' then ;',
  '  doit(r,r);',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestResourcestringAssignFail;
begin
  StartProgram(false);
  Add([
  'Resourcestring Foo = ''bar'';',
  'begin',
  '  Foo:=''a'';',
  '']);
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestResourcestringLocalFail;
begin
  StartProgram(false);
  Add([
  'procedure DoIt;',
  'Resourcestring Foo = ''bar'';',
  'begin end;',
  'begin;',
  '']);
  CheckParserException(SParserResourcestringsMustBeGlobal,nParserResourcestringsMustBeGlobal);
end;

procedure TTestResolver.TestResourcestringInConstFail;
begin
  StartProgram(false);
  Add([
  'Resourcestring Foo = ''foo'';',
  'const Bar = ''Prefix''+Foo;',
  'begin',
  '']);
  CheckResolverException(sConstantExpressionExpected,nConstantExpressionExpected);
end;

procedure TTestResolver.TestResourcestringPassVarArgFail;
begin
  StartProgram(false);
  Add([
  'Resourcestring Bar = ''foo'';',
  'procedure DoIt(var s: string); begin end;',
  'begin',
  '  doit(bar);',
  '']);
  CheckResolverException(sVariableIdentifierExpected,nVariableIdentifierExpected);
end;

procedure TTestResolver.TestHint_ElementHints;
begin
  StartProgram(false);
  Add([
  'type',
  '  TDeprecated = longint deprecated;',
  '  TLibrary = longint library;',
  '  TPlatform = longint platform;',
  '  TExperimental = longint experimental;',
  '  TUnimplemented = longint unimplemented;',
  'var',
  '  vDeprecated: TDeprecated;',
  '  vLibrary: TLibrary;',
  '  vPlatform: TPlatform;',
  '  vExperimental: TExperimental;',
  '  vUnimplemented: TUnimplemented;',
  'begin',
  '']);
  ParseProgram;
  CheckResolverHint(mtWarning,nSymbolXIsDeprecated,'Symbol "TDeprecated" is deprecated');
  CheckResolverHint(mtWarning,nSymbolXBelongsToALibrary,'Symbol "TLibrary" belongs to a library');
  CheckResolverHint(mtWarning,nSymbolXIsNotPortable,'Symbol "TPlatform" is not portable');
  CheckResolverHint(mtWarning,nSymbolXIsExperimental,'Symbol "TExperimental" is experimental');
  CheckResolverHint(mtWarning,nSymbolXIsNotImplemented,'Symbol "TUnimplemented" is not implemented');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestHint_ElementHintsMsg;
begin
  StartProgram(false);
  Add([
  'type',
  '  TDeprecated = longint deprecated ''foo'';',
  'var',
  '  vDeprecated: TDeprecated;',
  'begin',
  '']);
  ParseProgram;
  CheckResolverHint(mtWarning,nSymbolXIsDeprecatedY,'Symbol "TDeprecated" is deprecated: ''foo''');
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestHint_ElementHintsAlias;
var
  aMarker: PSrcMarker;
begin
  StartProgram(false);
  Add([
  'type',
  '  TPlatform = longint platform;',
  '  {#a}TAlias = TPlatform;',
  'var',
  '  {#b}vB: TPlatform;',
  '  {#c}vC: TAlias;',
  'function {#d}DoIt: TPlatform;',
  'begin',
  '  Result:=0;',
  'end;',
  'function {#e}DoSome: TAlias;',
  'begin',
  '  Result:=0;',
  'end;',
  'begin',
  '']);
  ParseProgram;
  //WriteSources('afile.pp',3,4);

  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestHint_ElementHintsAlias Marker "',aMarker^.Identifier,'" ',aMarker^.StartCol,'..',aMarker^.EndCol);
    CheckResolverHint(mtWarning,nSymbolXIsNotPortable,'Symbol "TPlatform" is not portable',aMarker);
    aMarker:=aMarker^.Next;
    end;

  CheckResolverUnexpectedHints(true);
end;

procedure TTestResolver.TestHint_ElementHints_WarnOff_SymbolDeprecated;
begin
  StartProgram(false);
  Add([
  '{$warn symbol_deprecated off}',
  'var',
  '  i: byte deprecated;',
  'begin',
  '  if i=3 then ;']);
  ParseProgram;
  CheckResolverUnexpectedHints(true);
end;

procedure TTestResolver.TestHint_Garbage;
begin
  StartProgram(false);
  Add([
  'begin',
  'end.']);
  ParseProgram;
  CheckResolverHint(mtHint,nTextAfterFinalIgnored,sTextAfterFinalIgnored);
  CheckResolverUnexpectedHints(true);
end;

procedure TTestResolver.TestClassHelper;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TObjectHelper = class helper for TObject',
  '  type T = word;',
  '  const',
  '    c: T = 3;',
  '    k: T = 4;',
  '  class var',
  '    v: T;',
  '    w: T;',
  '  end;',
  '  TBird = class(TObject)',
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '  end;',
  '  TExtObjHelper = class helper(TObjectHelper) for TBird',
  '  end;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_AncestorIsNotHelperForDescendantFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject)',
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '  end;',
  '  TFish = class(TObject)',
  '  end;',
  '  THelper = class helper(TBirdHelper) for TFish',
  '  end;',
  'begin',
  '']);
  CheckResolverException('Derived class helper must extend a subclass of "TBird" or the class itself',
    nDerivedXMustExtendASubClassY);
end;

procedure TTestResolver.TestClassHelper_HelperForParentFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject)',
  '  type',
  '    TBirdHelper = class helper for TBird',
  '    end;',
  '  end;',
  'begin',
  '']);
  CheckResolverException(sTypeXIsNotYetCompletelyDefined,
    nTypeXIsNotYetCompletelyDefined);
end;

procedure TTestResolver.TestClassHelper_ForInterfaceFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  IUnknown = interface',
  '    procedure DoIt;',
  '  end;',
  '  TBirdHelper = class helper for IUnknown',
  '  end;',
  'begin',
  '']);
  CheckResolverException('class type expected, but IUnknown found',
    nXExpectedButYFound);
end;

procedure TTestResolver.TestClassHelper_FieldFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    F: word;',
  '  end;',
  'begin',
  '']);
  CheckParserException('Fields are not allowed in class helper',
    nParserNoFieldsAllowed);
end;

procedure TTestResolver.TestClassHelper_AbstractFail;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    procedure DoIt; virtual; abstract;',
  '  end;',
  'begin',
  '']);
  CheckResolverException('Invalid class helper procedure modifier abstract',
    nInvalidXModifierY);
end;

procedure TTestResolver.TestClassHelper_VirtualObjFPCFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    procedure DoIt; virtual;',
  '  end;',
  'procedure TObjHelper.DoIt;',
  'begin end;',
  'begin',
  '']);
  CheckResolverException('Invalid class helper procedure modifier virtual',
    nInvalidXModifierY);
end;

procedure TTestResolver.TestClassHelper_VirtualDelphiFail;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    procedure DoIt; virtual;',
  '  end;',
  'procedure TObjHelper.DoIt;',
  'begin end;',
  'begin',
  '']);
  CheckResolverException('Invalid class helper procedure modifier virtual',
    nInvalidXModifierY);
end;

procedure TTestResolver.TestClassHelper_DestructorFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    destructor Destroyer;',
  '  end;',
  'destructor TObjHelper.Destroyer;',
  'begin end;',
  'begin',
  '']);
  CheckParserException('destructor is not allowed in class helper',
    nParserXNotAllowedInY);
end;

procedure TTestResolver.TestClassHelper_ClassRefersToTypeHelperOfAncestor;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '  type',
  '    TInt = word;',
  '    function GetSize: TInt;',
  '  end;',
  '  TAnt = class',
  '    procedure SetSize(Value: TInt);',
  '    property Size: TInt read GetSize write SetSize;',
  '  end;',
  'function Tobjhelper.getSize: TInt;',
  'begin',
  'end;',
  'procedure TAnt.SetSize(Value: TInt);',
  'begin',
  'end;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_InheritedObjFPC;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure {#TObject_Fly}Fly;',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    procedure {#TObjHelper_Fly}Fly;',
  '  end;',
  '  TBird = class',
  '    procedure {#TBird_Fly}Fly;',
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '    procedure {#TBirdHelper_Fly}Fly;',
  '    procedure {#TBirdHelper_Walk}Walk;',
  '  end;',
  '  TEagleHelper = class helper(TBirdHelper) for TBird',
  '    procedure {#TEagleHelper_Fly}Fly;',
  '    procedure {#TEagleHelper_Walk}Walk;',
  '  end;',
  'procedure Tobject.fly;',
  'begin',
  '  inherited;', // ignore
  'end;',
  'procedure Tobjhelper.fly;',
  'begin',
  '  {@TObject_Fly}inherited;',
  '  inherited {@TObject_Fly}Fly;',
  'end;',
  'procedure Tbird.fly;',
  'begin',
  '  {@TObjHelper_Fly}inherited;',
  '  inherited {@TObjHelper_Fly}Fly;',
  'end;',
  'procedure Tbirdhelper.fly;',
  'begin',
  '  {@TBird_Fly}inherited;',
  '  inherited {@TBird_Fly}Fly;',
  'end;',
  'procedure Tbirdhelper.walk;',
  'begin',
  'end;',
  'procedure teagleHelper.fly;',
  'begin',
  '  {@TBird_Fly}inherited;',
  '  inherited {@TBird_Fly}Fly;',
  'end;',
  'procedure teagleHelper.walk;',
  'begin',
  '  {@TBirdHelper_Walk}inherited;',
  '  inherited {@TBirdHelper_Walk}Walk;',
  'end;',
  'var',
  '  o: TObject;',
  '  b: TBird;',
  'begin',
  '  o.{@TObjHelper_Fly}Fly;',
  '  b.{@TEagleHelper_Fly}Fly;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_InheritedObjFPC2;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure {#TObject_Fly}Fly;',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    procedure {#TObjHelper_Walk}Walk;',
  '  end;',
  '  TBird = class',
  '    procedure {#TBird_Fly}Fly;',
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '    procedure {#TBirdHelper_Walk}Walk;',
  '  end;',
  '  TEagleHelper = class helper(TBirdHelper) for TBird',
  '    procedure {#TEagleHelper_Walk}Walk;',
  '  end;',
  'procedure Tobject.fly;',
  'begin',
  '  inherited;', // ignore
  'end;',
  'procedure Tobjhelper.walk;',
  'begin',
  '  inherited;', // ignore
  'end;',
  'procedure Tbird.fly;',
  'begin',
  '  {@TObject_Fly}inherited;', // no helper, search further in ancestor
  '  inherited {@TObject_Fly}Fly;', // no helper, search further in ancestor
  'end;',
  'procedure Tbirdhelper.walk;',
  'begin',
  '  {@TObjHelper_Walk}inherited;',
  '  inherited {@TObjHelper_Walk}Walk;',
  'end;',
  'procedure teagleHelper.walk;',
  'begin',
  '  {@TObjHelper_Walk}inherited;',
  '  inherited {@TObjHelper_Walk}Walk;',
  'end;',
  'begin',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_InheritedObjFPCStrictPrivateFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  strict private i: word;',
  '  end;',
  '  THelper = class helper for TObject',
  '    property a: word read i;',
  '  end;',
  'begin',
  '']);
  CheckResolverException('Can''t access strict private member i',nCantAccessXMember);
end;

procedure TTestResolver.TestClassHelper_InheritedClassObjFPC;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class procedure {#TObject_Fly}Fly;',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    class procedure {#TObjHelper_Fly}Fly;',
  '  end;',
  '  TBird = class',
  '    class procedure {#TBird_Fly}Fly;',
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '    class procedure {#TBirdHelper_Fly}Fly;',
  '    class procedure {#TBirdHelper_Walk}Walk;',
  '  end;',
  '  TEagleHelper = class helper(TBirdHelper) for TBird',
  '    class procedure {#TEagleHelper_Fly}Fly;',
  '    class procedure {#TEagleHelper_Walk}Walk;',
  '  end;',
  'class procedure Tobject.fly;',
  'begin',
  '  inherited;', // ignore
  'end;',
  'class procedure Tobjhelper.fly;',
  'begin',
  '  {@TObject_Fly}inherited;',
  '  inherited {@TObject_Fly}Fly;',
  'end;',
  'class procedure Tbird.fly;',
  'begin',
  '  {@TObjHelper_Fly}inherited;',
  '  inherited {@TObjHelper_Fly}Fly;',
  'end;',
  'class procedure Tbirdhelper.fly;',
  'begin',
  '  {@TBird_Fly}inherited;',
  '  inherited {@TBird_Fly}Fly;',
  'end;',
  'class procedure Tbirdhelper.walk;',
  'begin',
  'end;',
  'class procedure teagleHelper.fly;',
  'begin',
  '  {@TBird_Fly}inherited;',
  '  inherited {@TBird_Fly}Fly;',
  'end;',
  'class procedure teagleHelper.walk;',
  'begin',
  '  {@TBirdHelper_Walk}inherited;',
  '  inherited {@TBirdHelper_Walk}Walk;',
  'end;',
  'var',
  '  o: TObject;',
  '  b: TBird;',
  'begin',
  '  o.{@TObjHelper_Fly}Fly;',
  '  TObject.{@TObjHelper_Fly}Fly;',
  '  b.{@TEagleHelper_Fly}Fly;',
  '  TBird.{@TEagleHelper_Fly}Fly;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_InheritedDelphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class',
  '    procedure {#TObject_Fly}Fly;',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    procedure {#TObjHelper_Fly}Fly;',
  '  end;',
  '  TBird = class',
  '    procedure {#TBird_Fly}Fly;',
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '    procedure {#TBirdHelper_Fly}Fly;',
  '    procedure {#TBirdHelper_Walk}Walk;',
  '  end;',
  '  TEagleHelper = class helper(TBirdHelper) for TBird',
  '    procedure {#TEagleHelper_Fly}Fly;',
  '    procedure {#TEagleHelper_Walk}Walk;',
  '  end;',
  'procedure Tobject.fly;',
  'begin',
  '  inherited;', // ignore
  'end;',
  'procedure Tobjhelper.fly;',
  'begin',
  '  inherited;', // ignore
  '  inherited {@TObject_Fly}Fly;',
  'end;',
  'procedure Tbird.fly;',
  'begin',
  '  {@TObjHelper_Fly}inherited;',
  '  inherited {@TObjHelper_Fly}Fly;',
  'end;',
  'procedure Tbirdhelper.fly;',
  'begin',
  '  {@TObjHelper_Fly}inherited;',// skip helperfortype too
  '  inherited {@TBird_Fly}Fly;',
  'end;',
  'procedure Tbirdhelper.walk;',
  'begin',
  'end;',
  'procedure teagleHelper.fly;',
  'begin',
  '  {@TObjHelper_Fly}inherited;',// skip helperfortype too
  '  inherited {@TBird_Fly}Fly;',
  'end;',
  'procedure teagleHelper.walk;',
  'begin',
  '  inherited;', // ignore
  '  inherited {@TBirdHelper_Walk}Walk;',
  'end;',
  'var',
  '  o: TObject;',
  '  b: TBird;',
  'begin',
  '  o.{@TObjHelper_Fly}Fly;',
  '  b.{@TEagleHelper_Fly}Fly;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_NestedInheritedParentFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TBird = class',
  '    procedure Fly;',
  '  type',
  '    TBirdHelper = class helper for TObject',
  '      procedure Fly;',
  '    end;',
  '  end;',
  'procedure TBird.fly;',
  'begin',
  'end;',
  'procedure TBird.Tbirdhelper.fly;',
  'begin',
  '  inherited Fly;',
  'end;',
  'begin',
  '']);
  CheckResolverException('identifier not found "Fly"',nIdentifierNotFound);
end;

procedure TTestResolver.TestClassHelper_AccessFields;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TBird = class',
  '    Size: word;',
  '    FItems: array of word;',
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '    procedure Fly;',
  '  end;',
  'procedure TBirdHelper.Fly;',
  'begin',
  '  Size:=FItems[0];',
  '  Self.Size:=Self.FItems[0];',
  'end;',
  'var',
  '  b: TBird;',
  'begin',
  '  b.Fly;',
  '  b.Fly()',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_HelperDotClassMethodFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  THelper = class helper for TObject',
  '    class procedure Fly;',
  '  end;',
  'class procedure THelper.Fly;',
  'begin',
  'end;',
  'begin',
  '  THelper.Fly;',
  '']);
  CheckResolverException(sHelpersCannotBeUsedAsTypes,nHelpersCannotBeUsedAsTypes);
end;

procedure TTestResolver.TestClassHelper_WithHelperFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  THelper = class helper for TObject',
  '  end;',
  'begin',
  '  with THelper do ;',
  '']);
  CheckResolverException(sHelpersCannotBeUsedAsTypes,nHelpersCannotBeUsedAsTypes);
end;

procedure TTestResolver.TestClassHelper_AsTypeFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  THelper = class helper for TObject',
  '  end;',
  'var h: THelper;',
  'begin',
  '']);
  CheckResolverException(sHelpersCannotBeUsedAsTypes,nHelpersCannotBeUsedAsTypes);
end;

procedure TTestResolver.TestClassHelper_ClassMethod;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  THelper = class helper for TObject',
  '    class procedure Fly(w: word = 1);',
  '    class procedure Run(w: word = 1); static;',
  '  end;',
  'class procedure THelper.Fly(w: word = 1);',
  'begin',
  '  Fly;',
  '  Fly();',
  '  Run;',
  '  Run();',
  '  Self.Fly;',
  '  Self.Fly();',
  '  Self.Run;',
  '  Self.Run();',
  '  with Self do begin',
  '    Fly;',
  '    Fly();',
  '    Run;',
  '    Run();',
  '  end;',
  'end;',
  'class procedure THelper.Run(w: word = 1);',
  'begin',
  '  Fly;',
  '  Fly();',
  '  Run;',
  '  Run();',
  'end;',
  'var o: TObject;',
  'begin',
  '  o.Fly;',
  '  o.Fly();',
  '  o.Run;',
  '  o.Run();',
  '  with o do begin',
  '    Fly;',
  '    Fly();',
  '    Run;',
  '    Run();',
  '  end;',
  '  TObject.Fly;',
  '  TObject.Fly();',
  '  TObject.Run;',
  '  TObject.Run();',
  '  with TObject do begin',
  '    Fly;',
  '    Fly();',
  '    Run;',
  '    Run();',
  '  end;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_Enumerator;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TItem = TObject;',
  '  TEnumerator = class',
  '    FCurrent: TItem;',
  '    property Current: TItem read FCurrent;',
  '    function MoveNext: boolean;',
  '  end;',
  '  TBird = class',
  '    FItems: array of TItem;',
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '    function GetEnumerator: TEnumerator;',
  '  end;',
  'function TEnumerator.MoveNext: boolean;',
  'begin',
  'end;',
  'function TBirdHelper.GetEnumerator: TEnumerator;',
  'begin',
  '  Result.FCurrent:=FItems[0];',
  '  Result.FCurrent:=Self.FItems[0];',
  'end;',
  'var',
  '  b: TBird;',
  '  i: TItem;',
  '  {#i2}i2: TItem;',
  'begin',
  '  for i in b do {@i2}i2:=i;']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_FromUnitInterface;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'type',
    '  TObject = class',
    '  public',
    '    Id: word;',
    '  end;',
    '  TObjHelper = class helper for TObject',
    '    property Size: word read ID write ID;',
    '  end;',
    '']),
    '');
  AddModuleWithIntfImplSrc('unit3.pas',
    LinesToStr([
    'uses unit2;',
    'type',
    '  TObjHelper = class helper for TObject',
    '    property Size: word read ID write ID;',
    '  end;',
    '']),
    '');
  StartProgram(true);
  Add([
  'uses unit2, unit3;',
  'var o: TObject;',
  'begin',
  '  o.Size:=o.Size;']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_Constructor_NewInstance;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
  ActualNewInstance, ActualImplicitCallWithoutParams: Boolean;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  THelper = class helper for TObject',
  '    constructor Create;',
  '    class function DoSome: TObject;',
  '  end;',
  'constructor THelper.Create;',
  'begin',
  '  {#a}Create; // normal call',
  '  TObject.{#b}Create; // new instance',
  'end;',
  'class function THelper.DoSome: TObject;',
  'begin',
  '  Result:={#c}Create; // new instance',
  'end;',
  'var',
  '  o: TObject;',
  'begin',
  '  TObject.{#p}Create; // new object',
  '  o:=TObject.{#q}Create; // new object',
  '  with TObject do begin',
  '    {#r}Create; // new object',
  '    o:={#s}Create; // new object',
  '  end;',
  '  o.{#t}Create; // normal call',
  '  o:=o.{#u}Create; // normal call',
  '  with o do begin',
  '    {#v}Create; // normal call',
  '    o:={#w}Create; // normal call',
  '  end;',
  '']);
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestClassHelper_Constructor_NewInstance ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualNewInstance:=false;
      ActualImplicitCallWithoutParams:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestClassHelper_Constructor_NewInstance ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if not (Ref.Declaration is TPasProcedure) then continue;
        //writeln('TTestResolver.TestClassHelper_Constructor_NewInstance ',GetObjName(Ref.Declaration),' rrfNewInstance=',rrfNewInstance in Ref.Flags);
        if (Ref.Declaration is TPasConstructor) then
          ActualNewInstance:=rrfNewInstance in Ref.Flags;
        ActualImplicitCallWithoutParams:=rrfImplicitCallWithoutParams in Ref.Flags;
        break;
        end;
      if not ActualImplicitCallWithoutParams then
        RaiseErrorAtSrcMarker('expected implicit call at "#'+aMarker^.Identifier+', but got function ref"',aMarker);
      case aMarker^.Identifier of
      'a','t','u','v','w':// should be normal call
        if ActualNewInstance then
          RaiseErrorAtSrcMarker('expected normal call at "#'+aMarker^.Identifier+', but got newinstance"',aMarker);
      else // should be newinstance
        if not ActualNewInstance then
          RaiseErrorAtSrcMarker('expected newinstance at "#'+aMarker^.Identifier+', but got normal call"',aMarker);
      end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestClassHelper_ReintroduceHides_CallFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create(o: tobject);',
  '  end;',
  '  TBird = class helper for TObject',
  '    constructor Create(i: longint); reintroduce;',
  '  end;',
  'constructor tobject.Create(o: tobject); begin end;',
  'constructor tbird.Create(i: longint); begin end;',
  'var o: TObject;',
  'begin',
  '  o:=TObject.Create(nil);',
  '']);
  CheckResolverException('Incompatible type arg no. 1: Got "Nil", expected "Longint"',
    nIncompatibleTypeArgNo);
end;

procedure TTestResolver.TestClassHelper_DefaultProperty;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function GetB(Index: longint): longint;',
  '    procedure SetB(Index: longint; Value: longint);',
  '  end;',
  '  THelper = class helper for TObject',
  '    property B[Index: longint]: longint read GetB write SetB; default;',
  '  end;',
  'function TObject.GetB(Index: longint): longint;',
  'begin',
  'end;',
  'procedure TObject.SetB(Index: longint; Value: longint);',
  'begin',
  '  if Value=Self[Index] then ;',
  '  Self[Index]:=Value;',
  'end;',
  'var o: TObject;',
  'begin',
  '  o[3]:=4;',
  '  if o[5]=6 then;',
  '  if 7=o[8] then;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_DefaultClassProperty;
begin
  StartProgram(false);
  Add([
  'type',
  '  TClass = class of TObject;',
  '  TObject = class',
  '    class function GetB(Index: longint): longint; static;',
  '    class procedure SetB(Index: longint; Value: longint); static;',
  '  end;',
  '  THelper = class helper for TObject',
  '    class property B[Index: longint]: longint read GetB write SetB; default;',
  '  end;',
  'class function TObject.GetB(Index: longint): longint;',
  'begin',
  'end;',
  'class procedure TObject.SetB(Index: longint; Value: longint);',
  'begin',
  '  if Value=TObject[Index] then ;',
  '  TObject[Index]:=Value;',
  'end;',
  'var c: TClass;',
  'begin',
  '  c[3]:=4;',
  '  if c[5]=6 then;',
  '  if 7=c[8] then;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestClassHelper_MultiHelpers;
begin
  StartProgram(false);
  Add([
  '{$modeswitch multihelpers}',
  'type',
  '  TObject = class',
  '  end;',
  '  TFlyHelper = class helper for TObject',
  '    procedure {#Fly}Fly;',
  '    procedure {#FlyMove}Move;',
  '  end;',
  '  TRunHelper = class helper for TObject',
  '    procedure {#Run}Run;',
  '    procedure {#RunMove}Move;',
  '    procedure {#RunBack}Back;',
  '  end;',
  '  TSwimHelper = class helper for TObject',
  '    procedure {#Swim}Swim;',
  '    procedure {#SwimBack}Back;',
  '  end;',
  'procedure TFlyHelper.Fly; begin end;',
  'procedure TFlyHelper.Move; begin end;',
  'procedure TRunHelper.Run; begin end;',
  'procedure TRunHelper.Move; begin end;',
  'procedure TRunHelper.Back; begin end;',
  'procedure TSwimHelper.Swim; begin end;',
  'procedure TSwimHelper.Back; begin end;',
  'var o: TObject;',
  'begin',
  '  o.{@Fly}Fly;',
  '  o.{@Run}Run;',
  '  o.{@Swim}Swim;',
  '  o.{@RunMove}Move;',
  '  o.{@SwimBack}Back;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestRecordHelper;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TProc = procedure of object;',
  '  TRec = record',
  '    x: word;',
  '  end;',
  '  TRecHelper = record helper for TRec',
  '  type T = word;',
  '  const',
  '    c: T = 3;',
  '    k: T = 4;',
  '  class var',
  '    v: T;',
  '    w: T;',
  '    procedure Fly;',
  '  end;',
  '  TAnt = word;',
  '  TAntHelper = record helper for TAnt',
  '  end;',
  'procedure TRecHelper.Fly;',
  'var',
  '  r: TRec;',
  '  p: TProc;',
  'begin',
  '  Self:=r;',
  '  r:=Self;',
  '  c:=v+x;',
  '  x:=k+w;',
  '  p:=Fly;',
  'end;',
  'var',
  '  r: TRec;',
  '  p: TProc;',
  'begin',
  '  p:=r.Fly;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestRecordHelper_ForByteFail;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  'type',
  '  TRecHelper = record helper for byte',
  '    class var Glob: word;',
  '  end;',
  'begin',
  '']);
  CheckResolverException('Type "Byte" cannot be extended by a record helper',nTypeXCannotBeExtendedByARecordHelper);
end;

procedure TTestResolver.TestRecordHelper_ClassNonStaticFail;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TRec = record',
  '    x: word;',
  '  end;',
  '  TRecHelper = record helper for TRec',
  '    class procedure Fly;',
  '  end;',
  'class procedure TRecHelper.Fly;',
  'begin',
  'end;',
  'begin',
  '']);
  CheckResolverException('Class methods must be static in record helper',nClassMethodsMustBeStaticInX);
end;

procedure TTestResolver.TestRecordHelper_InheritedObjFPC;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  '{$modeswitch advancedrecords}',
  'type',
  '  TRec = record',
  '    procedure {#TRec_Fly}Fly;',
  '  end;',
  '  TRecHelper = record helper for TRec',
  '    procedure {#TRecHelper_Fly}Fly;',
  '    procedure {#TRecHelper_Walk}Walk;',
  '    procedure {#TRecHelper_Run}Run;',
  '  end;',
  '  TEagleHelper = record helper(TRecHelper) for TRec',
  '    procedure {#TEagleHelper_Fly}Fly;',
  '    procedure {#TEagleHelper_Run}Run;',
  '  end;',
  'procedure TRec.fly;',
  'begin',
  'end;',
  'procedure TRechelper.fly;',
  'begin',
  '  {@TRec_Fly}inherited;',
  '  inherited {@TRec_Fly}Fly;',
  'end;',
  'procedure TRechelper.walk;',
  'begin',
  '  inherited;', // ignore
  'end;',
  'procedure TRechelper.run;',
  'begin',
  '  inherited;', // ignore
  'end;',
  'procedure teagleHelper.fly;',
  'begin',
  '  {@TRec_Fly}inherited;',
  '  inherited {@TRec_Fly}Fly;',
  'end;',
  'procedure teagleHelper.run;',
  'begin',
  '  {@TRecHelper_Run}inherited;',
  '  inherited {@TRecHelper_Run}Run;',
  'end;',
  'var',
  '  r: TRec;',
  'begin',
  '  r.{@TEagleHelper_Fly}Fly;',
  '  r.{@TRecHelper_Walk}Walk;',
  '  r.{@TEagleHelper_Run}Run;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestRecordHelper_Constructor_NewInstance;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualNewInstance: Boolean;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add([
  '{$modeswitch advancedrecords}',
  '{$modeswitch typehelpers}',
  'type',
  '  TRec = record',
  '    constructor Create(w: word);',
  '    class function DoSome: TRec; static;',
  '  end;',
  'constructor TRec.Create(w: word);',
  'begin',
  '  {#a}Create(1); // normal call',
  '  TRec.{#b}Create(2); // new instance',
  'end;',
  'class function TRec.DoSome: TRec;',
  'begin',
  '  Result:={#c}Create(3); // new instance',
  'end;',
  'var',
  '  r: TRec;',
  'begin',
  '  TRec.{#p}Create(4); // new object',
  '  r:=TRec.{#q}Create(5); // new object',
  '  with TRec do begin',
  '    {#r}Create(6); // new object',
  '    r:={#s}Create(7); // new object',
  '  end;',
  '  r.{#t}Create(8); // normal call',
  '  r:=r.{#u}Create(9); // normal call',
  '  with r do begin',
  '    {#v}Create(10); // normal call',
  '    r:={#w}Create(11); // normal call',
  '  end;',
  '']);
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestAdvRecord_Constructor_NewInstance ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualNewInstance:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestAdvRecord_Constructor_NewInstance ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if not (Ref.Declaration is TPasProcedure) then continue;
        //writeln('TTestResolver.TestAdvRecord_Constructor_NewInstance ',GetObjName(Ref.Declaration),' rrfNewInstance=',rrfNewInstance in Ref.Flags);
        if (Ref.Declaration is TPasConstructor) then
          ActualNewInstance:=rrfNewInstance in Ref.Flags;
        if rrfImplicitCallWithoutParams in Ref.Flags then
          RaiseErrorAtSrcMarker('unexpected implicit call at "#'+aMarker^.Identifier+' ref"',aMarker);
        break;
        end;
      case aMarker^.Identifier of
      'a','t','u','v','w':// should be normal call
        if ActualNewInstance then
          RaiseErrorAtSrcMarker('expected normal call at "#'+aMarker^.Identifier+', but got newinstance"',aMarker);
      else // should be newinstance
        if not ActualNewInstance then
          RaiseErrorAtSrcMarker('expected newinstance at "#'+aMarker^.Identifier+', but got normal call"',aMarker);
      end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestTypeHelper;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TStringHelper = type helper for string',
  '  end;',
  '  TCaption = string;',
  '  TCapHelper = type helper(TStringHelper) for TCaption',
  '    procedure Fly;',
  '  end;',
  '  TProc = procedure of object;',
  'procedure TCapHelper.Fly; begin end;',
  'var',
  '  c: TCaption;',
  '  p: TProc;',
  'begin',
  '  c.Fly;',
  '  p:=@c.Fly;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeHelper_HelperForProcTypeFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TProc = procedure;',
  '  THelper = type helper for TProc',
  '  end;',
  'begin',
  '']);
  CheckResolverException('Type "TProc" cannot be extended by a type helper',
    nTypeXCannotBeExtendedByATypeHelper);
end;

procedure TTestResolver.TestTypeHelper_DefaultPropertyFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TStringHelper = type helper for string',
  '  end;',
  '  TCaption = string;',
  '  TCapHelper = type helper(TStringHelper) for TCaption',
  '    function GetItems(Index: boolean): boolean;',
  '    property Items[Index: boolean]: boolean read GetItems; default;',
  '  end;',
  'function TCapHelper.GetItems(Index: boolean): boolean; begin end;',
  'begin',
  '']);
  CheckResolverException('Default property not allowed in helper for TCaption',
    nDefaultPropertyNotAllowedInHelperForX);
end;

procedure TTestResolver.TestTypeHelper_Enum;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TFlag = (Red, Green, Blue);',
  '  THelper = type helper for TFlag',
  '    function toString: string;',
  '    class procedure Fly; static;',
  '  end;',
  'function THelper.toString: string;',
  'begin',
  '  Self:=Red;',
  '  if Self=TFlag.Blue then ;',
  '  Result:=str(Self);',
  'end;',
  'class procedure THelper.Fly;',
  'begin',
  'end;',
  'var',
  '  f: TFlag;',
  'begin',
  '  f.toString;',
  '  green.toString;',
  '  TFlag.green.toString;',
  '  TFlag.Fly;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeHelper_EnumDotValueFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TFlag = (Red, Green, Blue);',
  '  THelper = type helper for TFlag',
  '  end;',
  'var',
  '  f: TFlag;',
  'begin',
  '  f:=f.red;',
  '']);
  CheckResolverException('identifier not found "red"',nIdentifierNotFound);
end;

procedure TTestResolver.TestTypeHelper_EnumHelperDotProcFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TFlag = (Red, Green, Blue);',
  '  THelper = type helper for TFlag',
  '    procedure Fly;',
  '  end;',
  'procedure THelper.Fly;',
  'begin',
  'end;',
  'begin',
  '  TFlag.Fly;',
  '']);
  CheckResolverException('Instance member "Fly" inaccessible here',
    nInstanceMemberXInaccessible);
end;

procedure TTestResolver.TestTypeHelper_Set;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TEnum = (Red, Green, Blue);',
  '  TSetOfEnum = set of TEnum;',
  '  THelper = type helper for TSetOfEnum',
  '    procedure Fly;',
  '    class procedure Run; static;',
  '  end;',
  'procedure THelper.Fly;',
  'begin',
  '  Self:=[];',
  '  Self:=[green];',
  '  Include(Self,blue);',
  'end;',
  'class procedure THelper.Run;',
  'begin',
  'end;',
  'var s: TSetOfEnum;',
  'begin',
  '  s.Fly;',
  //'  with s do Fly;',
  '  TSetOfEnum.Run;',
  //'  with TSetOfEnum do Run;',
  //'  [green].Fly', not supported
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeHelper_Enumerator;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TObject = class end;',
  '  TItem = byte;',
  '  TEnumerator = class',
  '    FCurrent: TItem;',
  '    property Current: TItem read FCurrent;',
  '    function MoveNext: boolean;',
  '  end;',
  '  TWordHelper = type helper for Word',
  '    function GetEnumerator: TEnumerator;',
  '  end;',
  'function TEnumerator.MoveNext: boolean;',
  'begin',
  'end;',
  'function TWordHelper.GetEnumerator: TEnumerator;',
  'begin',
  '  if Self=2 then ;',
  '  Self:=Self+3;',
  'end;',
  'var',
  '  w: word;',
  '  i: TItem;',
  '  {#i2}i2: TItem;',
  'begin',
  '  w.GetEnumerator;',
  '  for i in w do {@i2}i2:=i;']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeHelper_String;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TStringHelper = type helper for String',
  '    procedure DoIt;',
  '  end;',
  '  TCharHelper = type helper for char',
  '    procedure Fly;',
  '  end;',
  'procedure TStringHelper.DoIt;',
  'begin',
  '  Self[1]:=Self[2];',
  'end;',
  'procedure TCharHelper.Fly;',
  'begin',
  '  Self:=''c'';',
  '  Self:=Self;',
  'end;',
  'begin',
  '  ''abc''.DoIt;',
  '  ''xyz''.DoIt();',
  '  ''c''.Fly;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeHelper_StringOtherUnit;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    '{$modeswitch typehelpers}',
    'type',
    '  TStringHelper = type helper for String',
    '    procedure DoIt;',
    '  end;',
    '  TCharHelper = type helper for char',
    '    procedure Fly;',
    '  end;',
    '']),
    LinesToStr([
    'procedure TStringHelper.DoIt;',
    'begin',
    '  Self[1]:=Self[2];',
    'end;',
    'procedure TCharHelper.Fly;',
    'begin',
    '  Self:=''c'';',
    '  Self:=Self;',
    'end;',
    '']));
  StartProgram(true);
  Add([
  'uses unit2;',
  'var s: string;',
  'begin',
  '  ''abc''.DoIt;',
  '  ''xyz''.DoIt();',
  '  ''c''.Fly;',
  '  s.DoIt;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeHelper_Boolean;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for boolean',
  '    procedure DoIt;',
  '  end;',
  'procedure THelper.DoIt;',
  'begin',
  '  Self:=not Self;',
  'end;',
  'begin',
  '  false.DoIt;',
  '  true.DoIt();']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeHelper_Double;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  Float = double;',
  '  THelper = type helper for float',
  '    const NPI = 3.141592;',
  '    function ToStr: String;',
  '  end;',
  'function THelper.ToStr: String;',
  'begin',
  'end;',
  'var',
  '  a,b: Float;',
  '  s: string;',
  'begin',
  '  s:=(a * b.NPI).ToStr;',
  '  s:=(a * float.NPI).ToStr;',
  '  s:=float.NPI.ToStr;',
  '  s:=3.2.ToStr;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeHelper_DoubleAlias;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  Float = type double;',
  '  THelper = type helper for float',
  '    const NPI = 3.141592;',
  '    function ToStr: String;',
  '  end;',
  'function THelper.ToStr: String;',
  'begin',
  'end;',
  'var',
  '  a,b: Float;',
  '  s: string;',
  'begin',
  '  s:=(a * b.NPI).ToStr;',
  '  s:=(a * float.NPI).ToStr;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeHelper_Constructor_NewInstance;
var
  aMarker: PSrcMarker;
  Elements: TFPList;
  ActualNewInstance: Boolean;
  i: Integer;
  El: TPasElement;
  Ref: TResolvedReference;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TInt = type word;',
  '  THelper = type helper for TInt',
  '    constructor Create(w: TInt);',
  '    class function DoSome: TInt; static;',
  '  end;',
  'constructor THelper.Create(w: TInt);',
  'begin',
  '  {#a}Create(1); // normal call',
  '  TInt.{#b}Create(2); // new instance',
  'end;',
  'class function THelper.DoSome: TInt;',
  'begin',
  '  Result:={#c}Create(3); // new instance',
  'end;',
  'var',
  '  r: TInt;',
  'begin',
  '  TInt.{#p}Create(4); // new object',
  '  r:=TInt.{#q}Create(5); // new object',
  '  with TInt do begin',
  '    {#r}Create(6); // new object',
  '    r:={#s}Create(7); // new object',
  '  end;',
  '  r.{#t}Create(8); // normal call',
  '  r:=r.{#u}Create(9); // normal call',
  '  with r do begin',
  '    {#v}Create(10); // normal call',
  '    r:={#w}Create(11); // normal call',
  '  end;',
  '']);
  ParseProgram;
  aMarker:=FirstSrcMarker;
  while aMarker<>nil do
    begin
    //writeln('TTestResolver.TestAdvRecord_Constructor_NewInstance ',aMarker^.Identifier,' ',aMarker^.StartCol,' ',aMarker^.EndCol);
    Elements:=FindElementsAt(aMarker);
    try
      ActualNewInstance:=false;
      for i:=0 to Elements.Count-1 do
        begin
        El:=TPasElement(Elements[i]);
        //writeln('TTestResolver.TestAdvRecord_Constructor_NewInstance ',aMarker^.Identifier,' ',i,'/',Elements.Count,' El=',GetObjName(El),' ',GetObjName(El.CustomData));
        if not (El.CustomData is TResolvedReference) then continue;
        Ref:=TResolvedReference(El.CustomData);
        if not (Ref.Declaration is TPasProcedure) then continue;
        //writeln('TTestResolver.TestAdvRecord_Constructor_NewInstance ',GetObjName(Ref.Declaration),' rrfNewInstance=',rrfNewInstance in Ref.Flags);
        if (Ref.Declaration is TPasConstructor) then
          ActualNewInstance:=rrfNewInstance in Ref.Flags;
        if rrfImplicitCallWithoutParams in Ref.Flags then
          RaiseErrorAtSrcMarker('unexpected implicit call at "#'+aMarker^.Identifier+' ref"',aMarker);
        break;
        end;
      case aMarker^.Identifier of
      'a','t','u','v','w':// should be normal call
        if ActualNewInstance then
          RaiseErrorAtSrcMarker('expected normal call at "#'+aMarker^.Identifier+', but got newinstance"',aMarker);
      else // should be newinstance
        if not ActualNewInstance then
          RaiseErrorAtSrcMarker('expected newinstance at "#'+aMarker^.Identifier+', but got normal call"',aMarker);
      end;
    finally
      Elements.Free;
    end;
    aMarker:=aMarker^.Next;
    end;
end;

procedure TTestResolver.TestTypeHelper_Interface;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  IUnknown = interface',
  '    function GetSizes(Index: word): word;',
  '    procedure SetSizes(Index: word; value: word);',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function GetSizes(Index: word): word; virtual; abstract;',
  '    procedure SetSizes(Index: word; value: word); virtual; abstract;',
  '  end;',
  '  THelper = type helper for IUnknown',
  '    procedure Fly;',
  '    class procedure Run; static;',
  '    property Sizes[Index: word]: word read GetSizes write SetSizes; default;',
  '  end;',
  'var',
  '  i: IUnknown;',
  '  o: TObject;',
  'procedure THelper.Fly;',
  'begin',
  '  i:=Self;',
  '  o:=Self as TObject;',
  '  Self:=nil;',
  '  Self:=i;',
  '  Self:=o;',
  'end;',
  'class procedure THelper.Run;',
  'begin',
  'end;',
  'begin',
  '  i.Fly;',
  '  i.Fly();',
  '  i.Run;',
  '  i.Run();',
  '  i.Sizes[3]:=i.Sizes[4];',
  '  i[5]:=i[6];',
  '  IUnknown.Run;',
  '  IUnknown.Run();',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestTypeHelper_Interface_ConstructorFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  IUnknown = interface',
  '  end;',
  '  THelper = type helper for IUnknown',
  '    constructor Fly;',
  '  end;',
  'constructor THelper.Fly;',
  'begin',
  'end;',
  'begin',
  '']);
  CheckResolverException('constructor is not supported',nXIsNotSupported);
end;

procedure TTestResolver.TestTypeHelper_TypeAliasType;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TEnum = type longint;',
  '  TIntHelper = type helper for longint',
  '    procedure Run;',
  '  end;',
  '  TEnumHelper = type helper for TEnum',
  '    procedure Fly;',
  '  end;',
  'procedure TIntHelper.Run;',
  'begin',
  'end;',
  'procedure TEnumHelper.Fly;',
  'begin',
  'end;',
  'var',
  '  e: TEnum;',
  '  i: longint;',
  'begin',
  '  i.Run;',
  '  e.Fly;',
  '  with i do Run;',
  '  with e do Fly;',
  '']);
  ParseProgram;
end;

procedure TTestResolver.TestAttributes_Globals;
begin
  StartProgram(false);
  Add([
  '{$modeswitch prefixedattributes}',
  'type',
  '  TObject = class',
  '    constructor {#TObject_Create}Create;',
  '  end;',
  '  {#Custom}TCustomAttribute = class',
  '  end;',
  '  {#Red}RedAttribute = class(TCustomAttribute)',
  '    constructor {#Red_A}Create(Id: word = 3; Deep: boolean = false); overload;',
  '    constructor {#Red_B}Create(Size: double); overload;',
  '  end;',
  '  Red = word;',
  'constructor TObject.Create; begin end;',
  'constructor RedAttribute.Create(Id: word; Deep: boolean); begin end;',
  'constructor RedAttribute.Create(Size: double); begin end;',
  'var',
  '  [{#Attr__Custom__TObject_Create}TCustom]',
  '  [{#Attr__Red__Red_A__1}Red,afile.{#Attr__Red__Red_A__2}Red]',
  '  o: TObject;',
  'const',
  '  [{#Attr__Red__Red_B}RedAttribute(1.3)]',
  '  c = 3;',
  'begin',
  '']);
  ParseProgram;
  CheckAttributeMarkers;
  CheckResolverUnexpectedHints;
end;

procedure TTestResolver.TestAttributes_NonConstParam_Fail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch prefixedattributes}',
  'type',
  '  TObject = class',
  '    constructor Create(w: word);',
  '  end;',
  '  TCustomAttribute = class',
  '  end;',
  'constructor TObject.Create(w: word);',
  'begin',
  'end;',
  'var',
  '  w: word;',
  '  [TCustom(w)]',
  '  o: TObject;',
  'begin',
  '']);
  CheckResolverException(sConstantExpressionExpected,nConstantExpressionExpected);
end;

procedure TTestResolver.TestAttributes_UnknownAttrWarning;
begin
  StartProgram(false);
  Add([
  '{$modeswitch prefixedattributes}',
  'type',
  '  TObject = class',
  '  end;',
  '  TCustomAttribute = class',
  '  end;',
  'var',
  '  [Red]',
  '  o: TObject;',
  'begin',
  '']);
  ParseProgram;
  CheckResolverHint(mtWarning,nUnknownCustomAttributeX,'Unknown custom attribute "Red"');
end;

procedure TTestResolver.TestAttributes_Members;
begin
  StartProgram(false);
  Add([
  '{$modeswitch prefixedattributes}',
  'type',
  '  TObject = class',
  '    constructor {#create}Create;',
  '  end;',
  '  {#custom}TCustomAttribute = class',
  '  end;',
  '  TMyClass = class',
  '    [{#attr__custom__create__cl}TCustom]',
  '    Field: word;',
  '  end;',
  '  TMyRecord = record',
  '    [{#attr__custom__create__rec}TCustom]',
  '    Field: word;',
  '  end;',
  'constructor TObject.Create;',
  'begin',
  'end;',
  'begin',
  '']);
  ParseProgram;
  CheckAttributeMarkers;
end;

initialization
  RegisterTests([TTestResolver]);

end.

