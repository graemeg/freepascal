{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2018 by Michael Van Canneyt

    Unit tests for Pascal-to-Javascript converter class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

 Examples:
    ./testpas2js --suite=TTestModule.TestEmptyProgram
    ./testpas2js --suite=TTestModule.TestEmptyUnit
}
unit TCModules;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, contnrs,
  jstree, jswriter, jsbase,
  PasTree, PScanner, PasResolver, PParser, PasResolveEval,
  FPPas2Js;

const
  // default parser+scanner options
  po_tcmodules = po_Pas2js+[po_KeepScannerError];
  co_tcmodules = [];
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
    Row: integer;
    StartCol, EndCol: integer; // token start, end column
    Identifier: string;
    Next: PSrcMarker;
  end;

  TSystemUnitPart = (
    supTObject,
    supTVarRec,
    supTypeInfo,
    supTInterfacedObject,
    supWriteln
    );
  TSystemUnitParts = set of TSystemUnitPart;

  { TTestHintMessage }

  TTestHintMessage = class
  public
    Id: int64;
    MsgType: TMessageType;
    MsgNumber: integer;
    Msg: string;
    SourcePos: TPasSourcePos;
  end;

  { TTestPasParser }

  TTestPasParser = Class(TPasParser)
  end;

  TOnFindUnit = function(const aUnitName: String): TPasModule of object;

  { TTestEnginePasResolver }

  TTestEnginePasResolver = class(TPas2JsResolver)
  private
    FFilename: string;
    FModule: TPasModule;
    FOnFindUnit: TOnFindUnit;
    FParser: TTestPasParser;
    FStreamResolver: TStreamResolver;
    FScanner: TPas2jsPasScanner;
    FSource: string;
  public
    destructor Destroy; override;
    function FindUnit(const AName, InFilename: String; NameExpr,
      InFileExpr: TPasExpr): TPasModule; override;
    procedure UsedInterfacesFinished(Section: TPasSection); override;
    property OnFindUnit: TOnFindUnit read FOnFindUnit write FOnFindUnit;
    property Filename: string read FFilename write FFilename;
    property StreamResolver: TStreamResolver read FStreamResolver write FStreamResolver;
    property Scanner: TPas2jsPasScanner read FScanner write FScanner;
    property Parser: TTestPasParser read FParser write FParser;
    property Source: string read FSource write FSource;
    property Module: TPasModule read FModule;
  end;

  { TCustomTestModule }

  TCustomTestModule = Class(TTestCase)
  private
    FConverter: TPasToJSConverter;
    FEngine: TTestEnginePasResolver;
    FExpectedErrorClass: ExceptClass;
    FExpectedErrorMsg: string;
    FExpectedErrorNumber: integer;
    FFilename: string;
    FFileResolver: TStreamResolver;
    FHub: TPas2JSResolverHub;
    FJSImplementationUses: TJSArrayLiteral;
    FJSInitBody: TJSFunctionBody;
    FJSImplentationUses: TJSArrayLiteral;
    FJSInterfaceUses: TJSArrayLiteral;
    FJSModule: TJSSourceElements;
    FJSModuleSrc: TJSSourceElements;
    FJSSource: TStringList;
    FModule: TPasModule;
    FJSModuleCallArgs: TJSArguments;
    FModules: TObjectList;// list of TTestEnginePasResolver
    FParser: TTestPasParser;
    FPasProgram: TPasProgram;
    FPasLibrary: TPasLibrary;
    FHintMsgs: TObjectList; // list of TTestHintMessage
    FHintMsgsGood: TFPList; // list of TTestHintMessage marked as expected
    FJSRegModuleCall: TJSCallExpression;
    FScanner: TPas2jsPasScanner;
    FSkipTests: boolean;
    FSource: TStringList;
    FFirstPasStatement: TPasImplBlock;
    FWithTypeInfo: boolean;
    {$IFDEF EnablePasTreeGlobalRefCount}
    FElementRefCountAtSetup: int64;
    {$ENDIF}
    function GetMsgCount: integer;
    function GetMsgs(Index: integer): TTestHintMessage;
    function GetResolverCount: integer;
    function GetResolvers(Index: integer): TTestEnginePasResolver;
    function OnPasResolverFindUnit(const aUnitName: String): TPasModule;
    procedure OnParserLog(Sender: TObject; const Msg: String);
    procedure OnPasResolverLog(Sender: TObject; const Msg: String);
    procedure OnScannerLog(Sender: TObject; const Msg: String);
    procedure SetWithTypeInfo(const AValue: boolean);
  protected
    procedure SetUp; override;
    function CreateConverter: TPasToJSConverter; virtual;
    function LoadUnit(const aUnitName: String): TPasModule;
    procedure InitScanner(aScanner: TPas2jsPasScanner); virtual;
    procedure TearDown; override;
    Procedure Add(Line: string); virtual;
    Procedure Add(const Lines: array of string);
    Procedure StartParsing; virtual;
    procedure ParseModuleQueue; virtual;
    procedure ParseModule; virtual;
    procedure ParseProgram; virtual;
    procedure ParseLibrary; virtual;
    procedure ParseUnit; virtual;
  protected
    function FindModuleWithFilename(aFilename: string): TTestEnginePasResolver; virtual;
    function AddModule(aFilename: string): TTestEnginePasResolver; virtual;
    function AddModuleWithSrc(aFilename, Src: string): TTestEnginePasResolver; virtual;
    function AddModuleWithIntfImplSrc(aFilename, InterfaceSrc,
      ImplementationSrc: string): TTestEnginePasResolver; virtual;
    procedure AddSystemUnit(Parts: TSystemUnitParts = []); virtual;
    procedure StartProgram(NeedSystemUnit: boolean; SystemUnitParts: TSystemUnitParts = []); virtual;
    procedure StartLibrary(NeedSystemUnit: boolean; SystemUnitParts: TSystemUnitParts = []); virtual;
    procedure StartUnit(NeedSystemUnit: boolean; SystemUnitParts: TSystemUnitParts = []); virtual;
    procedure ConvertModule; virtual;
    procedure ConvertProgram; virtual;
    procedure ConvertLibrary; virtual;
    procedure ConvertUnit; virtual;
    function ConvertJSModuleToString(El: TJSElement): string; virtual;
    procedure CheckDottedIdentifier(Msg: string; El: TJSElement; DottedName: string);
    function GetDottedIdentifier(El: TJSElement): string;
    procedure CheckSource(Msg,Statements: String; InitStatements: string = '';
      ImplStatements: string = ''); virtual;
    procedure CheckDiff(Msg, Expected, Actual: string); virtual;
    procedure CheckUnit(Filename, ExpectedSrc: string); virtual;
    procedure CheckHint(MsgType: TMessageType; MsgNumber: integer;
      Msg: string; Marker: PSrcMarker = nil); virtual;
    procedure CheckResolverUnexpectedHints(WithSourcePos: boolean = false); virtual;
    procedure SetExpectedScannerError(Msg: string; MsgNumber: integer);
    procedure SetExpectedParserError(Msg: string; MsgNumber: integer);
    procedure SetExpectedPasResolverError(Msg: string; MsgNumber: integer);
    procedure SetExpectedConverterError(Msg: string; MsgNumber: integer);
    function IsErrorExpected(E: Exception): boolean;
    procedure HandleScannerError(E: EScannerError);
    procedure HandleParserError(E: EParserError);
    procedure HandlePasResolveError(E: EPasResolve);
    procedure HandlePas2JSError(E: EPas2JS);
    procedure HandleException(E: Exception);
    procedure FailException(E: Exception);
    procedure WriteSources(const aFilename: string; aRow, aCol: integer);
    function IndexOfResolver(const Filename: string): integer;
    function GetResolver(const Filename: string): TTestEnginePasResolver;
    function GetDefaultNamespace: string;
    property PasProgram: TPasProgram Read FPasProgram;
    property PasLibrary: TPasLibrary Read FPasLibrary;
    property Resolvers[Index: integer]: TTestEnginePasResolver read GetResolvers;
    property ResolverCount: integer read GetResolverCount;
    property Engine: TTestEnginePasResolver read FEngine;
    property Filename: string read FFilename;
    Property Module: TPasModule Read FModule;
    property FirstPasStatement: TPasImplBlock read FFirstPasStatement;
    property Converter: TPasToJSConverter read FConverter;
    property JSSource: TStringList read FJSSource;
    property JSModule: TJSSourceElements read FJSModule;
    property JSRegModuleCall: TJSCallExpression read FJSRegModuleCall;
    property JSModuleCallArgs: TJSArguments read FJSModuleCallArgs;
    property JSImplementationUses: TJSArrayLiteral read FJSImplementationUses;
    property JSInterfaceUses: TJSArrayLiteral read FJSInterfaceUses;
    property JSModuleSrc: TJSSourceElements read FJSModuleSrc;
    property JSInitBody: TJSFunctionBody read FJSInitBody;
    property ExpectedErrorClass: ExceptClass read FExpectedErrorClass write FExpectedErrorClass;
    property ExpectedErrorMsg: string read FExpectedErrorMsg write FExpectedErrorMsg;
    property ExpectedErrorNumber: integer read FExpectedErrorNumber write FExpectedErrorNumber;
    property SkipTests: boolean read FSkipTests write FSkipTests;
  public
    constructor Create; override;
    destructor Destroy; override;
    property Hub: TPas2JSResolverHub read FHub;
    property Source: TStringList read FSource;
    property FileResolver: TStreamResolver read FFileResolver;
    property Scanner: TPas2jsPasScanner read FScanner;
    property Parser: TTestPasParser read FParser;
    property MsgCount: integer read GetMsgCount;
    property Msgs[Index: integer]: TTestHintMessage read GetMsgs;
    property WithTypeInfo: boolean read FWithTypeInfo write SetWithTypeInfo;
  end;

  { TTestModule }

  TTestModule = class(TCustomTestModule)
  Published
    Procedure TestReservedWords;

    // program, units, includes
    Procedure TestEmptyProgram;
    Procedure TestEmptyProgramUseStrict;
    Procedure TestEmptyUnit;
    Procedure TestEmptyUnitUseStrict;
    Procedure TestDottedUnitNames;
    Procedure TestDottedUnitNameImpl;
    Procedure TestDottedUnitExpr;
    Procedure Test_ModeFPCFail;
    Procedure Test_ModeSwitchCBlocksFail;
    Procedure TestUnit_UseSystem;
    Procedure TestUnit_Intf1Impl2Intf1;
    Procedure TestIncludeVersion;

    // vars/const
    Procedure TestVarInt;
    Procedure TestVarBaseTypes;
    Procedure TestBaseTypeSingleFail;
    Procedure TestBaseTypeExtendedFail;
    Procedure TestConstBaseTypes;
    Procedure TestUnitImplVars;
    Procedure TestUnitImplConsts;
    Procedure TestUnitImplRecord;
    Procedure TestRenameJSNameConflict;
    Procedure TestLocalConst;
    Procedure TestVarExternal;
    Procedure TestVarExternalOtherUnit;
    Procedure TestVarAbsoluteFail;
    Procedure TestConstExternal;

    // numbers
    Procedure TestDouble;
    Procedure TestInteger;
    Procedure TestIntegerRange;
    Procedure TestIntegerTypecasts;
    Procedure TestInteger_BitwiseShrNativeInt;
    Procedure TestInteger_BitwiseShlNativeInt;
    Procedure TestInteger_SystemFunc;
    Procedure TestCurrency;
    Procedure TestForBoolDo;
    Procedure TestForIntDo;
    Procedure TestForIntInDo;

    // strings
    Procedure TestCharConst;
    Procedure TestChar_Compare;
    Procedure TestChar_BuiltInProcs;
    Procedure TestStringConst;
    Procedure TestStringConst_InvalidUTF16;
    Procedure TestStringConstSurrogate;
    Procedure TestString_Length;
    Procedure TestString_Compare;
    Procedure TestString_SetLength;
    Procedure TestString_CharAt;
    Procedure TestStringHMinusFail;
    Procedure TestStr;
    Procedure TestBaseType_AnsiStringFail;
    Procedure TestBaseType_WideStringFail;
    Procedure TestBaseType_ShortStringFail;
    Procedure TestBaseType_RawByteStringFail;
    Procedure TestTypeShortstring_Fail;
    Procedure TestCharSet_Custom;
    Procedure TestWideChar;
    Procedure TestForCharDo;
    Procedure TestForCharInDo;

    // alias types
    Procedure TestAliasTypeRef;
    Procedure TestTypeCast_BaseTypes;
    Procedure TestTypeCast_AliasBaseTypes;

    // functions
    Procedure TestEmptyProc;
    Procedure TestProcOneParam;
    Procedure TestFunctionWithoutParams;
    Procedure TestProcedureWithoutParams;
    Procedure TestPrgProcVar;
    Procedure TestProcTwoArgs;
    Procedure TestProc_DefaultValue;
    Procedure TestUnitProcVar;
    Procedure TestImplProc;
    Procedure TestFunctionResult;
    Procedure TestNestedProc;
    Procedure TestNestedProc_ResultString;
    Procedure TestForwardProc;
    Procedure TestNestedForwardProc;
    Procedure TestAssignFunctionResult;
    Procedure TestFunctionResultInCondition;
    Procedure TestFunctionResultInForLoop;
    Procedure TestFunctionResultInTypeCast;
    Procedure TestExit;
    Procedure TestExit_ResultInFinally;
    Procedure TestBreak;
    Procedure TestBreakAsVar;
    Procedure TestContinue;
    Procedure TestProc_External;
    Procedure TestProc_ExternalOtherUnit;
    Procedure TestProc_Asm;
    Procedure TestProc_AsmSubBlock;
    Procedure TestProc_Assembler;
    Procedure TestProc_VarParam;
    Procedure TestProc_VarParamString;
    Procedure TestProc_VarParamV;
    Procedure TestProc_Overload;
    Procedure TestProc_OverloadForward;
    Procedure TestProc_OverloadIntfImpl;
    Procedure TestProc_OverloadNested;
    Procedure TestProc_OverloadNestedForward;
    Procedure TestProc_OverloadUnitCycle;
    Procedure TestProc_Varargs;
    Procedure TestProc_ConstOrder;
    Procedure TestProc_DuplicateConst;
    Procedure TestProc_LocalVarAbsolute;
    Procedure TestProc_LocalVarInit;
    Procedure TestProc_ReservedWords;
    Procedure TestProc_ConstRefWord;

    // anonymous functions
    Procedure TestAnonymousProc_Assign_ObjFPC;
    Procedure TestAnonymousProc_Assign_Delphi;
    Procedure TestAnonymousProc_Arg;
    Procedure TestAnonymousProc_Typecast;
    Procedure TestAnonymousProc_With;
    Procedure TestAnonymousProc_ExceptOn;
    Procedure TestAnonymousProc_Nested;
    Procedure TestAnonymousProc_NestedAssignResult;
    Procedure TestAnonymousProc_Class;
    Procedure TestAnonymousProc_ForLoop;
    Procedure TestAnonymousProc_AsmDelphi;

    // enums, sets
    Procedure TestEnum_Name;
    Procedure TestEnum_Number;
    Procedure TestEnum_ConstFail;
    Procedure TestEnum_Functions;
    Procedure TestEnumRg_Functions;
    Procedure TestEnum_AsParams;
    Procedure TestEnumRange_Array;
    Procedure TestEnum_ForIn;
    Procedure TestEnum_ScopedNumber;
    Procedure TestEnum_InFunction;
    Procedure TestEnum_Name_Anonymous_Unit;
    Procedure TestSet_Enum;
    Procedure TestSet_Operators;
    Procedure TestSet_Operator_In;
    Procedure TestSet_Functions;
    Procedure TestSet_PassAsArgClone;
    Procedure TestSet_AsParams;
    Procedure TestSet_Property;
    Procedure TestSet_EnumConst;
    Procedure TestSet_IntConst;
    Procedure TestSet_IntRange;
    Procedure TestSet_AnonymousEnumType;
    Procedure TestSet_AnonymousEnumTypeChar; // ToDo
    Procedure TestSet_ConstEnum;
    Procedure TestSet_ConstChar;
    Procedure TestSet_ConstInt;
    Procedure TestSet_InFunction;
    Procedure TestSet_ForIn;

    // statements
    Procedure TestNestBegin;
    Procedure TestIncDec;
    Procedure TestLoHiFpcMode;
    Procedure TestLoHiDelphiMode;
    Procedure TestAssignments;
    Procedure TestArithmeticOperators1;
    Procedure TestMultiAdd;
    Procedure TestLogicalOperators;
    Procedure TestBitwiseOperators;
    Procedure TestBitwiseOperatorsLongword;
    Procedure TestFunctionInt;
    Procedure TestFunctionString;
    Procedure TestIfThen;
    Procedure TestForLoop;
    Procedure TestForLoopInsideFunction;
    Procedure TestForLoop_ReadVarAfter;
    Procedure TestForLoop_Nested;
    Procedure TestRepeatUntil;
    Procedure TestAsmBlock;
    Procedure TestAsmPas_Impl; // ToDo
    Procedure TestTryFinally;
    Procedure TestTryExcept;
    Procedure TestTryExcept_ReservedWords;
    Procedure TestIfThenRaiseElse;
    Procedure TestCaseOf;
    Procedure TestCaseOf_UseSwitch;
    Procedure TestCaseOfNoElse;
    Procedure TestCaseOfNoElse_UseSwitch;
    Procedure TestCaseOfRange;
    Procedure TestCaseOfString;
    Procedure TestCaseOfChar;
    Procedure TestCaseOfExternalClassConst;
    Procedure TestDebugger;

    // arrays
    Procedure TestArray_Dynamic;
    Procedure TestArray_Dynamic_Nil;
    Procedure TestArray_DynMultiDimensional;
    Procedure TestArray_DynamicAssign;
    Procedure TestArray_StaticInt;
    Procedure TestArray_StaticBool;
    Procedure TestArray_StaticChar;
    Procedure TestArray_StaticMultiDim;
    Procedure TestArray_StaticInFunction;
    Procedure TestArray_StaticMultiDimEqualNotImplemented;
    Procedure TestArrayOfRecord;
    Procedure TestArray_StaticRecord;
    Procedure TestArrayOfSet;
    Procedure TestArray_DynAsParam;
    Procedure TestArray_StaticAsParam;
    Procedure TestArrayElement_AsParams;
    Procedure TestArrayElementFromFuncResult_AsParams;
    Procedure TestArrayEnumTypeRange;
    Procedure TestArray_SetLengthOutArg;
    Procedure TestArray_SetLengthProperty;
    Procedure TestArray_SetLengthMultiDim;
    Procedure TestArray_SetLengthDynOfStatic;
    Procedure TestArray_OpenArrayOfString;
    Procedure TestArray_ArrayOfCharAssignString; // ToDo
    Procedure TestArray_ConstRef;
    Procedure TestArray_Concat;
    Procedure TestArray_Copy;
    Procedure TestArray_InsertDelete;
    Procedure TestArray_DynArrayConstObjFPC;
    Procedure TestArray_DynArrayConstDelphi;
    Procedure TestArray_ArrayLitAsParam;
    Procedure TestArray_ArrayLitMultiDimAsParam;
    Procedure TestArray_ArrayLitStaticAsParam;
    Procedure TestArray_ForInArrOfString;
    Procedure TestExternalClass_TypeCastArrayToExternalClass;
    Procedure TestExternalClass_TypeCastArrayFromExternalClass;
    Procedure TestArrayOfConst_TVarRec;
    Procedure TestArrayOfConst_PassBaseTypes;
    Procedure TestArrayOfConst_PassObj;

    // record
    Procedure TestRecord_Empty;
    Procedure TestRecord_Var;
    Procedure TestRecord_VarExternal;
    Procedure TestRecord_WithDo;
    Procedure TestRecord_Assign;
    Procedure TestRecord_AsParams;
    Procedure TestRecord_ConstRef;
    Procedure TestRecordElement_AsParams;
    Procedure TestRecordElementFromFuncResult_AsParams;
    Procedure TestRecordElementFromWith_AsParams;
    Procedure TestRecord_Equal;
    Procedure TestRecord_JSValue;
    Procedure TestRecord_VariantFail;
    Procedure TestRecord_FieldArray;
    Procedure TestRecord_Const;
    Procedure TestRecord_TypecastFail;
    Procedure TestRecord_InFunction;
    Procedure TestRecord_AnonymousFail;

    // advanced record
    Procedure TestAdvRecord_Function;
    Procedure TestAdvRecord_Property;
    Procedure TestAdvRecord_PropertyDefault;
    Procedure TestAdvRecord_Property_ClassMethod;
    Procedure TestAdvRecord_Const;
    Procedure TestAdvRecord_ExternalField;
    Procedure TestAdvRecord_SubRecord;
    Procedure TestAdvRecord_SubClass;
    Procedure TestAdvRecord_SubInterfaceFail;
    Procedure TestAdvRecord_Constructor;
    Procedure TestAdvRecord_ClassConstructor_Program;
    Procedure TestAdvRecord_ClassConstructor_Unit;

    // classes
    Procedure TestClass_TObjectDefaultConstructor;
    Procedure TestClass_TObjectConstructorWithParams;
    Procedure TestClass_TObjectConstructorWithDefaultParam;
    Procedure TestClass_Var;
    Procedure TestClass_Method;
    Procedure TestClass_Implementation;
    Procedure TestClass_Inheritance;
    Procedure TestClass_TypeAlias;
    Procedure TestClass_AbstractMethod;
    Procedure TestClass_CallInherited_ProcNoParams;
    Procedure TestClass_CallInherited_WithParams;
    Procedure TestClasS_CallInheritedConstructor;
    Procedure TestClass_ClassVar_Assign;
    Procedure TestClass_CallClassMethod;
    Procedure TestClass_CallClassMethodStatic; // ToDo
    Procedure TestClass_Property;
    Procedure TestClass_Property_ClassMethod;
    Procedure TestClass_Property_Indexed;
    Procedure TestClass_Property_IndexSpec;
    Procedure TestClass_PropertyOfTypeArray;
    Procedure TestClass_PropertyDefault;
    Procedure TestClass_PropertyDefault_TypecastToOtherDefault;
    //Procedure TestClass_PropertyDefault;
    Procedure TestClass_PropertyOverride;
    Procedure TestClass_PropertyIncVisibility;
    Procedure TestClass_Assigned;
    Procedure TestClass_WithClassDoCreate;
    Procedure TestClass_WithClassInstDoProperty;
    Procedure TestClass_WithClassInstDoPropertyWithParams;
    Procedure TestClass_WithClassInstDoFunc;
    Procedure TestClass_TypeCast;
    Procedure TestClass_TypeCastUntypedParam;
    Procedure TestClass_Overloads;
    Procedure TestClass_OverloadsAncestor;
    Procedure TestClass_OverloadConstructor;
    Procedure TestClass_OverloadDelphiOverride;
    Procedure TestClass_ReintroduceVarDelphi;
    Procedure TestClass_ReintroducedVar;
    Procedure TestClass_RaiseDescendant;
    Procedure TestClass_ExternalMethod;
    Procedure TestClass_ExternalVirtualNameMismatchFail;
    Procedure TestClass_ExternalOverrideFail;
    Procedure TestClass_ExternalVar;
    Procedure TestClass_Const;
    Procedure TestClass_ConstEnum;
    Procedure TestClass_LocalConstDuplicate_Prg;
    Procedure TestClass_LocalConstDuplicate_Unit;
    // ToDo: Procedure TestAdvRecord_LocalConstDuplicate;
    Procedure TestClass_LocalVarSelfFail;
    Procedure TestClass_ArgSelfFail;
    Procedure TestClass_NestedProcSelf;
    Procedure TestClass_NestedProcSelf2;
    Procedure TestClass_NestedProcClassSelf;
    Procedure TestClass_NestedProcCallInherited;
    Procedure TestClass_TObjectFree;
    Procedure TestClass_TObjectFree_VarArg;
    Procedure TestClass_TObjectFreeNewInstance;
    Procedure TestClass_TObjectFreeLowerCase;
    Procedure TestClass_TObjectFreeFunctionFail;
    Procedure TestClass_TObjectFreePropertyFail;
    Procedure TestClass_ForIn;
    Procedure TestClass_DispatchMessage;
    Procedure TestClass_Message_DuplicateIntFail;
    Procedure TestClass_DispatchMessage_WrongFieldNameFail;

    // class of
    Procedure TestClassOf_Create;
    Procedure TestClassOf_Call;
    Procedure TestClassOf_Assign;
    Procedure TestClassOf_Is;
    Procedure TestClassOf_Compare;
    Procedure TestClassOf_ClassVar;
    Procedure TestClassOf_ClassMethod;
    Procedure TestClassOf_ClassProperty;
    Procedure TestClassOf_ClassMethodSelf;
    Procedure TestClassOf_TypeCast;
    Procedure TestClassOf_ImplicitFunctionCall;
    Procedure TestClassOf_Const;

    // nested class
    Procedure TestNestedClass_Alias;
    Procedure TestNestedClass_Record;
    Procedure TestNestedClass_Class;

    // external class
    Procedure TestExternalClass_Var;
    Procedure TestExternalClass_Const;
    Procedure TestExternalClass_Dollar;
    Procedure TestExternalClass_DuplicateVarFail;
    Procedure TestExternalClass_Method;
    Procedure TestExternalClass_ClassMethod;
    Procedure TestExternalClass_ClassMethodStatic;
    Procedure TestExternalClass_FunctionResultInTypeCast;
    Procedure TestExternalClass_NonExternalOverride;
    Procedure TestExternalClass_OverloadHint;
    Procedure TestExternalClass_SameNamePublishedProperty;
    Procedure TestExternalClass_Property;
    Procedure TestExternalClass_PropertyDate;
    Procedure TestExternalClass_ClassProperty;
    Procedure TestExternalClass_ClassOf;
    Procedure TestExternalClass_ClassOtherUnit;
    Procedure TestExternalClass_Is;
    Procedure TestExternalClass_As;
    Procedure TestExternalClass_DestructorFail;
    Procedure TestExternalClass_New;
    Procedure TestExternalClass_ClassOf_New;
    Procedure TestExternalClass_FuncClassOf_New;
    Procedure TestExternalClass_New_PasClassFail;
    Procedure TestExternalClass_New_PasClassBracketsFail;
    Procedure TestExternalClass_NewExtName;
    Procedure TestExternalClass_Constructor;
    Procedure TestExternalClass_ConstructorBrackets;
    Procedure TestExternalClass_LocalConstSameName;
    Procedure TestExternalClass_ReintroduceOverload;
    Procedure TestExternalClass_Inherited;
    Procedure TestExternalClass_PascalAncestorFail;
    Procedure TestExternalClass_NewInstance;
    Procedure TestExternalClass_NewInstance_NonVirtualFail;
    Procedure TestExternalClass_NewInstance_FirstParamNotString_Fail;
    Procedure TestExternalClass_NewInstance_SecondParamTyped_Fail;
    Procedure TestExternalClass_JSFunctionPasDescendant;
    Procedure TestExternalClass_PascalProperty;
    Procedure TestExternalClass_TypeCastToRootClass;
    Procedure TestExternalClass_TypeCastToJSObject;
    Procedure TestExternalClass_TypeCastStringToExternalString;
    Procedure TestExternalClass_TypeCastToJSFunction;
    Procedure TestExternalClass_TypeCastDelphiUnrelated;
    Procedure TestExternalClass_CallClassFunctionOfInstanceFail;
    Procedure TestExternalClass_BracketAccessor;
    Procedure TestExternalClass_BracketAccessor_Call;
    Procedure TestExternalClass_BracketAccessor_2ParamsFail;
    Procedure TestExternalClass_BracketAccessor_ReadOnly;
    Procedure TestExternalClass_BracketAccessor_WriteOnly;
    Procedure TestExternalClass_BracketAccessor_MultiType;
    Procedure TestExternalClass_BracketAccessor_Index;
    Procedure TestExternalClass_ForInJSObject;
    Procedure TestExternalClass_ForInJSArray;
    Procedure TestExternalClass_IncompatibleArgDuplicateIdentifier;

    // class interfaces
    Procedure TestClassInterface_Corba;
    Procedure TestClassInterface_ProcExternalFail;
    Procedure TestClassInterface_Overloads;
    Procedure TestClassInterface_DuplicateGUIInIntfListFail;
    Procedure TestClassInterface_DuplicateGUIInAncestorFail;
    Procedure TestClassInterface_AncestorImpl;
    Procedure TestClassInterface_ImplReintroduce;
    Procedure TestClassInterface_MethodResolution;
    Procedure TestClassInterface_AncestorMoreInterfaces;
    Procedure TestClassInterface_MethodOverride;
    Procedure TestClassInterface_Corba_Delegation;
    Procedure TestClassInterface_Corba_DelegationStatic;
    Procedure TestClassInterface_Corba_Operators;
    Procedure TestClassInterface_Corba_Args;
    Procedure TestClassInterface_Corba_ForIn;
    Procedure TestClassInterface_COM_AssignVar;
    Procedure TestClassInterface_COM_AssignArg;
    Procedure TestClassInterface_COM_FunctionResult;
    Procedure TestClassInterface_COM_InheritedFuncResult;
    Procedure TestClassInterface_COM_IsAsTypeCasts;
    Procedure TestClassInterface_COM_PassAsArg;
    Procedure TestClassInterface_COM_PassToUntypedParam;
    Procedure TestClassInterface_COM_FunctionInExpr;
    Procedure TestClassInterface_COM_Property;
    Procedure TestClassInterface_COM_IntfProperty;
    Procedure TestClassInterface_COM_Delegation;
    Procedure TestClassInterface_COM_With;
    Procedure TestClassInterface_COM_ForIn;
    Procedure TestClassInterface_COM_ArrayOfIntfFail;
    Procedure TestClassInterface_COM_RecordIntfFail;
    Procedure TestClassInterface_COM_UnitInitialization;
    Procedure TestClassInterface_GUID;
    Procedure TestClassInterface_GUIDProperty;

    // helpers
    Procedure TestClassHelper_ClassVar;
    Procedure TestClassHelper_Method_AccessInstanceFields;
    Procedure TestClassHelper_Method_Call;
    Procedure TestClassHelper_Method_Nested_Call;
    Procedure TestClassHelper_ClassMethod_Call;
    Procedure TestClassHelper_ClassOf;
    Procedure TestClassHelper_MethodRefObjFPC;
    Procedure TestClassHelper_Constructor;
    Procedure TestClassHelper_InheritedObjFPC;
    Procedure TestClassHelper_Property;
    Procedure TestClassHelper_Property_Array;
    Procedure TestClassHelper_Property_Array_Default;
    Procedure TestClassHelper_Property_Array_DefaultDefault;
    Procedure TestClassHelper_ClassProperty;
    Procedure TestClassHelper_ClassPropertyStatic;
    Procedure TestClassHelper_ClassProperty_Array;
    Procedure TestClassHelper_ForIn;
    Procedure TestClassHelper_PassProperty;
    Procedure TestExtClassHelper_ClassVar;
    Procedure TestExtClassHelper_Method_Call;
    Procedure TestExtClassHelper_ClassMethod_MissingStatic;
    Procedure TestRecordHelper_ClassVar;
    Procedure TestRecordHelper_Method_Call;
    Procedure TestRecordHelper_Constructor;
    Procedure TestTypeHelper_ClassVar;
    Procedure TestTypeHelper_PassResultElement;
    Procedure TestTypeHelper_PassArgs;
    Procedure TestTypeHelper_PassVarConst;
    Procedure TestTypeHelper_PassFuncResult;
    Procedure TestTypeHelper_PassPropertyField;
    Procedure TestTypeHelper_PassPropertyGetter;
    Procedure TestTypeHelper_PassClassPropertyField;
    Procedure TestTypeHelper_PassClassPropertyGetterStatic;
    Procedure TestTypeHelper_PassClassPropertyGetterNonStatic;
    Procedure TestTypeHelper_Property;
    Procedure TestTypeHelper_Property_Array;
    Procedure TestTypeHelper_ClassProperty;
    Procedure TestTypeHelper_ClassProperty_Array;
    Procedure TestTypeHelper_ClassMethod;
    Procedure TestTypeHelper_ExtClassMethodFail;
    Procedure TestTypeHelper_Constructor;
    Procedure TestTypeHelper_Word;
    Procedure TestTypeHelper_Boolean;
    Procedure TestTypeHelper_WordBool;
    Procedure TestTypeHelper_Double;
    Procedure TestTypeHelper_NativeInt;
    Procedure TestTypeHelper_StringChar;
    Procedure TestTypeHelper_JSValue;
    Procedure TestTypeHelper_Array;
    Procedure TestTypeHelper_EnumType;
    Procedure TestTypeHelper_SetType;
    Procedure TestTypeHelper_InterfaceType;
    Procedure TestTypeHelper_NestedSelf;

    // proc types
    Procedure TestProcType;
    Procedure TestProcType_Arg;
    Procedure TestProcType_FunctionFPC;
    Procedure TestProcType_FunctionDelphi;
    Procedure TestProcType_ProcedureDelphi;
    Procedure TestProcType_AsParam;
    Procedure TestProcType_MethodFPC;
    Procedure TestProcType_MethodDelphi;
    Procedure TestProcType_PropertyFPC;
    Procedure TestProcType_PropertyDelphi;
    Procedure TestProcType_WithClassInstDoPropertyFPC;
    Procedure TestProcType_Nested;
    Procedure TestProcType_NestedOfObject;
    Procedure TestProcType_ReferenceToProc;
    Procedure TestProcType_ReferenceToMethod;
    Procedure TestProcType_Typecast;
    Procedure TestProcType_PassProcToUntyped;
    Procedure TestProcType_PassProcToArray;
    Procedure TestProcType_SafeCallObjFPC;
    Procedure TestProcType_SafeCallDelphi;

    // pointer
    Procedure TestPointer;
    Procedure TestPointer_Proc;
    Procedure TestPointer_AssignRecordFail;
    Procedure TestPointer_AssignStaticArrayFail;
    Procedure TestPointer_TypeCastJSValueToPointer;
    Procedure TestPointer_NonRecordFail;
    Procedure TestPointer_AnonymousArgTypeFail;
    Procedure TestPointer_AnonymousVarTypeFail;
    Procedure TestPointer_AnonymousResultTypeFail;
    Procedure TestPointer_AddrOperatorFail;
    Procedure TestPointer_ArrayParamsFail;
    Procedure TestPointer_PointerAddFail;
    Procedure TestPointer_IncPointerFail;
    Procedure TestPointer_Record;
    Procedure TestPointer_RecordArg;

    // jsvalue
    Procedure TestJSValue_AssignToJSValue;
    Procedure TestJSValue_TypeCastToBaseType;
    Procedure TestJSValue_TypecastToJSValue;
    Procedure TestJSValue_Equal;
    Procedure TestJSValue_If;
    Procedure TestJSValue_Not;
    Procedure TestJSValue_Enum;
    Procedure TestJSValue_ClassInstance;
    Procedure TestJSValue_ClassOf;
    Procedure TestJSValue_ArrayOfJSValue;
    Procedure TestJSValue_ArrayLit;
    Procedure TestJSValue_Params;
    Procedure TestJSValue_UntypedParam;
    Procedure TestJSValue_FuncResultType;
    Procedure TestJSValue_ProcType_Assign;
    Procedure TestJSValue_ProcType_Equal;
    Procedure TestJSValue_ProcType_Param;
    Procedure TestJSValue_AssignToPointerFail;
    Procedure TestJSValue_OverloadDouble;
    Procedure TestJSValue_OverloadNativeInt;
    Procedure TestJSValue_OverloadWord;
    Procedure TestJSValue_OverloadString;
    Procedure TestJSValue_OverloadChar;
    Procedure TestJSValue_OverloadPointer;
    Procedure TestJSValue_ForIn;

    // RTTI
    Procedure TestRTTI_IntRange;
    Procedure TestRTTI_Double;
    Procedure TestRTTI_ProcType;
    Procedure TestRTTI_ProcType_ArgFromOtherUnit;
    Procedure TestRTTI_EnumAndSetType;
    Procedure TestRTTI_EnumRange;
    Procedure TestRTTI_AnonymousEnumType;
    Procedure TestRTTI_StaticArray;
    Procedure TestRTTI_DynArray;
    Procedure TestRTTI_ArrayNestedAnonymous;
    Procedure TestRTTI_PublishedMethodOverloadFail;
    Procedure TestRTTI_PublishedMethodHideNoHint;
    Procedure TestRTTI_PublishedMethodExternalFail;
    Procedure TestRTTI_PublishedClassPropertyFail;
    Procedure TestRTTI_PublishedClassFieldFail;
    Procedure TestRTTI_PublishedFieldExternalFail;
    Procedure TestRTTI_Class_Field;
    Procedure TestRTTI_Class_Method;
    Procedure TestRTTI_Class_MethodArgFlags;
    Procedure TestRTTI_Class_Property;
    Procedure TestRTTI_Class_PropertyParams;
    Procedure TestRTTI_Class_OtherUnit_TypeAlias;
    Procedure TestRTTI_Class_OmitRTTI;
    Procedure TestRTTI_Class_Field_AnonymousArrayOfSelfClass;
    Procedure TestRTTI_IndexModifier;
    Procedure TestRTTI_StoredModifier;
    Procedure TestRTTI_DefaultValue;
    Procedure TestRTTI_DefaultValueSet;
    Procedure TestRTTI_DefaultValueRangeType;
    Procedure TestRTTI_DefaultValueInherit;
    Procedure TestRTTI_OverrideMethod;
    Procedure TestRTTI_ReintroduceMethod;
    Procedure TestRTTI_OverloadProperty;
    // ToDo: array argument
    Procedure TestRTTI_ClassForward;
    Procedure TestRTTI_ClassOf;
    Procedure TestRTTI_Record;
    Procedure TestRTTI_RecordAnonymousArray;
    Procedure TestRTTI_Record_ClassVarType;
    Procedure TestRTTI_LocalTypes;
    Procedure TestRTTI_TypeInfo_BaseTypes;
    Procedure TestRTTI_TypeInfo_Type_BaseTypes;
    Procedure TestRTTI_TypeInfo_LocalFail;
    Procedure TestRTTI_TypeInfo_ExtTypeInfoClasses1;
    Procedure TestRTTI_TypeInfo_ExtTypeInfoClasses2;
    Procedure TestRTTI_TypeInfo_ExtTypeInfoClasses3;
    Procedure TestRTTI_TypeInfo_FunctionClassType;
    Procedure TestRTTI_TypeInfo_MixedUnits_PointerAndClass;
    Procedure TestRTTI_Interface_Corba;
    Procedure TestRTTI_Interface_COM;
    Procedure TestRTTI_ClassHelper;
    Procedure TestRTTI_ExternalClass;
    Procedure TestRTTI_Unit;

    // Resourcestring
    Procedure TestResourcestringProgram;
    Procedure TestResourcestringUnit;
    Procedure TestResourcestringImplementation;

    // Attributes
    Procedure TestAttributes_Members;
    Procedure TestAttributes_Types;
    Procedure TestAttributes_HelperConstructor_Fail;

    // Assertions, checks
    procedure TestAssert;
    procedure TestAssert_SysUtils;
    procedure TestObjectChecks;
    procedure TestOverflowChecks_Int;
    procedure TestRangeChecks_AssignInt;
    procedure TestRangeChecks_AssignIntRange;
    procedure TestRangeChecks_AssignEnum;
    procedure TestRangeChecks_AssignEnumRange;
    procedure TestRangeChecks_AssignChar;
    procedure TestRangeChecks_AssignCharRange;
    procedure TestRangeChecks_ArrayIndex;
    procedure TestRangeChecks_ArrayOfRecIndex;
    procedure TestRangeChecks_StringIndex;
    procedure TestRangeChecks_TypecastInt;
    procedure TestRangeChecks_TypeHelperInt;

    // Async/AWait
    Procedure TestAsync_Proc;
    Procedure TestAsync_CallResultIsPromise;
    Procedure TestAsync_ConstructorFail;
    Procedure TestAsync_PropertyGetterFail;
    Procedure TestAwait_NonPromiseWithTypeFail;
    Procedure TestAwait_AsyncCallTypeMismatch;
    Procedure TestAWait_OutsideAsyncFail;
    Procedure TestAWait_IntegerFail;
    Procedure TestAWait_ExternalClassPromise;
    Procedure TestAWait_JSValue;
    Procedure TestAWait_Result;
    Procedure TestAWait_ResultPromiseMissingTypeFail; // await(AsyncCallResultPromise) needs T
    Procedure TestAsync_AnonymousProc;
    Procedure TestAsync_ProcType;
    Procedure TestAsync_ProcTypeAsyncModMismatchFail;
    Procedure TestAsync_Inherited;
    Procedure TestAsync_ClassInterface;
    Procedure TestAsync_ClassInterface_AsyncMissmatchFail;

    // Library
    Procedure TestLibrary_Empty;
    Procedure TestLibrary_ExportFunc; // ToDo
    // ToDo: test delayed specialization init
    // ToDO: analyzer
  end;

function LinesToStr(Args: array of const): string;
function ExtractFileUnitName(aFilename: string): string;
function JSToStr(El: TJSElement): string;
function CheckSrcDiff(Expected, Actual: string; out Msg: string): boolean;

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

function ExtractFileUnitName(aFilename: string): string;
var
  p: Integer;
begin
  Result:=ExtractFileName(aFilename);
  if Result='' then exit;
  for p:=length(Result) downto 1 do
    case Result[p] of
    '/','\': exit;
    '.':
      begin
      Delete(Result,p,length(Result));
      exit;
      end;
    end;
end;

function JSToStr(El: TJSElement): string;
var
  aWriter: TBufferWriter;
  aJSWriter: TJSWriter;
begin
  aJSWriter:=nil;
  aWriter:=TBufferWriter.Create(1000);
  try
    aJSWriter:=TJSWriter.Create(aWriter);
    aJSWriter.IndentSize:=2;
    aJSWriter.WriteJS(El);
    Result:=aWriter.AsString;
  finally
    aJSWriter.Free;
    aWriter.Free;
  end;
end;

function CheckSrcDiff(Expected, Actual: string; out Msg: string): boolean;
// search diff, ignore changes in spaces
const
  SpaceChars = [#9,#10,#13,' '];
var
  ExpectedP, ActualP: PChar;

  function FindLineEnd(p: PChar): PChar;
  begin
    Result:=p;
    while not (Result^ in [#0,#10,#13]) do inc(Result);
  end;

  function FindLineStart(p, MinP: PChar): PChar;
  begin
    while (p>MinP) and not (p[-1] in [#10,#13]) do dec(p);
    Result:=p;
  end;

  procedure SkipLineEnd(var p: PChar);
  begin
    if p^ in [#10,#13] then
    begin
      if (p[1] in [#10,#13]) and (p^<>p[1]) then
        inc(p,2)
      else
        inc(p);
    end;
  end;

  function HasSpecialChar(s: string): boolean;
  var
    i: Integer;
  begin
    for i:=1 to length(s) do
      if s[i] in [#0..#31,#127..#255] then
        exit(true);
    Result:=false;
  end;

  function HashSpecialChars(s: string): string;
  var
    i: Integer;
  begin
    Result:='';
    for i:=1 to length(s) do
      if s[i] in [#0..#31,#127..#255] then
        Result:=Result+'#'+hexstr(ord(s[i]),2)
      else
        Result:=Result+s[i];
  end;

  procedure DiffFound;
  var
    ActLineStartP, ActLineEndP, p, StartPos: PChar;
    ExpLine, ActLine: String;
    i, LineNo, DiffLineNo: Integer;
  begin
    writeln('Diff found "',Msg,'". Lines:');
    // write correct lines
    p:=PChar(Expected);
    LineNo:=0;
    DiffLineNo:=0;
    repeat
      StartPos:=p;
      while not (p^ in [#0,#10,#13]) do inc(p);
      ExpLine:=copy(Expected,StartPos-PChar(Expected)+1,p-StartPos);
      SkipLineEnd(p);
      inc(LineNo);
      if (p<=ExpectedP) and (p^<>#0) then
      begin
        writeln('= ',ExpLine);
      end else begin
        // diff line
        if DiffLineNo=0 then DiffLineNo:=LineNo;
        // write actual line
        ActLineStartP:=FindLineStart(ActualP,PChar(Actual));
        ActLineEndP:=FindLineEnd(ActualP);
        ActLine:=copy(Actual,ActLineStartP-PChar(Actual)+1,ActLineEndP-ActLineStartP);
        writeln('- ',ActLine);
        if HasSpecialChar(ActLine) then
          writeln('- ',HashSpecialChars(ActLine));
        // write expected line
        writeln('+ ',ExpLine);
        if HasSpecialChar(ExpLine) then
          writeln('- ',HashSpecialChars(ExpLine));
        // write empty line with pointer ^
        for i:=1 to 2+ExpectedP-StartPos do write(' ');
        writeln('^');
        Msg:='expected "'+ExpLine+'", but got "'+ActLine+'".';
        CheckSrcDiff:=false;
        // write up to three following actual lines to get some context
        for i:=1 to 3 do begin
          ActLineStartP:=ActLineEndP;
          SkipLineEnd(ActLineStartP);
          if ActLineStartP^=#0 then break;
          ActLineEndP:=FindLineEnd(ActLineStartP);
          ActLine:=copy(Actual,ActLineStartP-PChar(Actual)+1,ActLineEndP-ActLineStartP);
          writeln('~ ',ActLine);
        end;
        exit;
      end;
    until p^=#0;

    writeln('DiffFound Actual:-----------------------');
    writeln(Actual);
    writeln('DiffFound Expected:---------------------');
    writeln(Expected);
    writeln('DiffFound ------------------------------');
    Msg:='diff found, but lines are the same, internal error';
    CheckSrcDiff:=false;
  end;

var
  IsSpaceNeeded: Boolean;
  LastChar, Quote: Char;
begin
  Result:=true;
  Msg:='';
  if Expected='' then Expected:=' ';
  if Actual='' then Actual:=' ';
  ExpectedP:=PChar(Expected);
  ActualP:=PChar(Actual);
  repeat
    //writeln('TTestModule.CheckDiff Exp="',ExpectedP^,'" Act="',ActualP^,'"');
    case ExpectedP^ of
    #0:
      begin
      // check that rest of Actual has only spaces
      while ActualP^ in SpaceChars do inc(ActualP);
      if ActualP^<>#0 then
        begin
        DiffFound;
        exit;
        end;
      exit(true);
      end;
    ' ',#9,#10,#13:
      begin
      // skip space in Expected
      IsSpaceNeeded:=false;
      if ExpectedP>PChar(Expected) then
        LastChar:=ExpectedP[-1]
      else
        LastChar:=#0;
      while ExpectedP^ in SpaceChars do inc(ExpectedP);
      if (LastChar in ['a'..'z','A'..'Z','0'..'9','_','$'])
          and (ExpectedP^ in ['a'..'z','A'..'Z','0'..'9','_','$']) then
        IsSpaceNeeded:=true;
      if IsSpaceNeeded and (not (ActualP^ in SpaceChars)) then
        begin
        DiffFound;
        exit;
        end;
      while ActualP^ in SpaceChars do inc(ActualP);
      end;
    '''','"':
      begin
      while ActualP^ in SpaceChars do inc(ActualP);
      if ExpectedP^<>ActualP^ then
        begin
        DiffFound;
        exit;
        end;
      Quote:=ExpectedP^;
      repeat
        inc(ExpectedP);
        inc(ActualP);
        if ExpectedP^<>ActualP^ then
          begin
          DiffFound;
          exit;
          end;
        if (ExpectedP^ in [#0,#10,#13]) then
          break
        else if (ExpectedP^=Quote) then
          begin
          inc(ExpectedP);
          inc(ActualP);
          break;
          end;
      until false;
      end;
    else
      while ActualP^ in SpaceChars do inc(ActualP);
      if ExpectedP^<>ActualP^ then
        begin
        DiffFound;
        exit;
        end;
      inc(ExpectedP);
      inc(ActualP);
    end;
  until false;
end;

{ TTestEnginePasResolver }

destructor TTestEnginePasResolver.Destroy;
begin
  FreeAndNil(FStreamResolver);
  FreeAndNil(FParser);
  FreeAndNil(FScanner);
  FreeAndNil(FStreamResolver);
  if Module<>nil then
    begin
    Module.Release{$IFDEF CheckPasTreeRefCount}('CreateElement'){$ENDIF};
    FModule:=nil;
    end;
  inherited Destroy;
end;

function TTestEnginePasResolver.FindUnit(const AName, InFilename: String;
  NameExpr, InFileExpr: TPasExpr): TPasModule;
begin
  Result:=nil;
  if InFilename<>'' then
    RaiseNotYetImplemented(20180224101926,InFileExpr,'Use testcase tcunitsearch instead');
  if Assigned(OnFindUnit) then
    Result:=OnFindUnit(AName);
  if NameExpr=nil then ;
end;

procedure TTestEnginePasResolver.UsedInterfacesFinished(Section: TPasSection);
begin
  // do not parse recursively
  // parse via the queue
  if Section=nil then ;
end;

{ TCustomTestModule }

function TCustomTestModule.GetMsgCount: integer;
begin
  Result:=FHintMsgs.Count;
end;

function TCustomTestModule.GetMsgs(Index: integer): TTestHintMessage;
begin
  Result:=TTestHintMessage(FHintMsgs[Index]);
end;

function TCustomTestModule.GetResolverCount: integer;
begin
  Result:=FModules.Count;
end;

function TCustomTestModule.GetResolvers(Index: integer
  ): TTestEnginePasResolver;
begin
  Result:=TTestEnginePasResolver(FModules[Index]);
end;

function TCustomTestModule.OnPasResolverFindUnit(const aUnitName: String
  ): TPasModule;
var
  DefNamespace: String;
begin
  //writeln('TTestModule.OnPasResolverFindUnit START Unit="',aUnitName,'"');
  if (Pos('.',aUnitName)<1) then
    begin
    DefNamespace:=GetDefaultNamespace;
    if DefNamespace<>'' then
      begin
      Result:=LoadUnit(DefNamespace+'.'+aUnitName);
      if Result<>nil then exit;
      end;
    end;
  Result:=LoadUnit(aUnitName);
  if Result<>nil then exit;
  {$IFDEF VerbosePas2JS}
  writeln('TTestModule.OnPasResolverFindUnit missing unit "',aUnitName,'"');
  {$ENDIF}
  Fail('can''t find unit "'+aUnitName+'"');
end;

procedure TCustomTestModule.OnParserLog(Sender: TObject; const Msg: String);
var
  aParser: TPasParser;
  Item: TTestHintMessage;
begin
  aParser:=Sender as TPasParser;
  Item:=TTestHintMessage.Create;
  Item.Id:=aParser.LastMsgNumber;
  Item.MsgType:=aParser.LastMsgType;
  Item.MsgNumber:=aParser.LastMsgNumber;
  Item.Msg:=Msg;
  Item.SourcePos:=aParser.Scanner.CurSourcePos;
  {$IFDEF VerbosePas2JS}
  writeln('TCustomTestModule.OnParserLog ',GetObjName(Sender),' ',Item.MsgType,' (',Item.MsgNumber,') {',Msg,'}');
  {$ENDIF}
  FHintMsgs.Add(Item);
end;

procedure TCustomTestModule.OnPasResolverLog(Sender: TObject; const Msg: String
  );
var
  aResolver: TTestEnginePasResolver;
  Item: TTestHintMessage;
begin
  aResolver:=Sender as TTestEnginePasResolver;
  Item:=TTestHintMessage.Create;
  Item.Id:=aResolver.LastMsgId;
  Item.MsgType:=aResolver.LastMsgType;
  Item.MsgNumber:=aResolver.LastMsgNumber;
  Item.Msg:=Msg;
  Item.SourcePos:=aResolver.LastSourcePos;
  {$IFDEF VerbosePas2JS}
  writeln('TCustomTestModule.OnPasResolverLog ',GetObjName(Sender),' ',Item.MsgType,' (',Item.MsgNumber,') {',Msg,'}');
  {$ENDIF}
  FHintMsgs.Add(Item);
end;

procedure TCustomTestModule.OnScannerLog(Sender: TObject; const Msg: String);
var
  Item: TTestHintMessage;
  aScanner: TPas2jsPasScanner;
begin
  aScanner:=Sender as TPas2jsPasScanner;
  Item:=TTestHintMessage.Create;
  Item.Id:=aScanner.LastMsgNumber;
  Item.MsgType:=aScanner.LastMsgType;
  Item.MsgNumber:=aScanner.LastMsgNumber;
  Item.Msg:=Msg;
  Item.SourcePos:=aScanner.CurSourcePos;
  {$IFDEF VerbosePas2JS}
  writeln('TCustomTestModule.OnScannerLog ',GetObjName(Sender),' ',Item.MsgType,' (',Item.MsgNumber,') {',Msg,'}');
  {$ENDIF}
  FHintMsgs.Add(Item);
end;

procedure TCustomTestModule.SetWithTypeInfo(const AValue: boolean);
begin
  if FWithTypeInfo=AValue then Exit;
  FWithTypeInfo:=AValue;
  if AValue then
    Converter.Options:=Converter.Options-[coNoTypeInfo]
  else
    Converter.Options:=Converter.Options+[coNoTypeInfo];
end;

function TCustomTestModule.LoadUnit(const aUnitName: String): TPasModule;
var
  i: Integer;
  CurEngine: TTestEnginePasResolver;
  CurUnitName: String;
begin
  //writeln('TTestModule.FindUnit START Unit="',aUnitName,'"');
  Result:=nil;
  if (Module.ClassType=TPasModule)
      and (CompareText(Module.Name,aUnitName)=0) then
    exit(Module);

  for i:=0 to ResolverCount-1 do
    begin
    CurEngine:=Resolvers[i];
    CurUnitName:=ExtractFileUnitName(CurEngine.Filename);
    //writeln('TTestModule.FindUnit Checking ',i,'/',ResolverCount,' ',CurEngine.Filename,' ',CurUnitName);
    if CompareText(aUnitName,CurUnitName)=0 then
      begin
      Result:=CurEngine.Module;
      if Result<>nil then exit;
      //writeln('TTestModule.FindUnit PARSING unit "',CurEngine.Filename,'"');
      FileResolver.FindSourceFile(aUnitName);

      CurEngine.StreamResolver:=TStreamResolver.Create;
      CurEngine.StreamResolver.OwnsStreams:=True;
      //writeln('TTestModule.FindUnit SOURCE=',CurEngine.Source);
      CurEngine.StreamResolver.AddStream(CurEngine.FileName,TStringStream.Create(CurEngine.Source));
      CurEngine.Scanner:=TPas2jsPasScanner.Create(CurEngine.StreamResolver);
      InitScanner(CurEngine.Scanner);
      CurEngine.Parser:=TTestPasParser.Create(CurEngine.Scanner,CurEngine.StreamResolver,CurEngine);
      CurEngine.Parser.Options:=po_tcmodules;
      if CompareText(CurUnitName,'System')=0 then
        CurEngine.Parser.ImplicitUses.Clear;
      CurEngine.Scanner.OpenFile(CurEngine.Filename);
      try
        CurEngine.Parser.NextToken;
        CurEngine.Parser.ParseUnit(CurEngine.FModule);
      except
        on E: Exception do
          HandleException(E);
      end;
      //writeln('TTestModule.FindUnit END ',CurUnitName);
      Result:=CurEngine.Module;
      exit;
      end;
    end;
end;

procedure TCustomTestModule.SetUp;
begin
  {$IFDEF EnablePasTreeGlobalRefCount}
  FElementRefCountAtSetup:=TPasElement.GlobalRefCount;
  {$ENDIF}

  if FModules<>nil then
    begin
    writeln('TCustomTestModule.SetUp FModules<>nil');
    Halt;
    end;

  inherited SetUp;
  FSkipTests:=false;
  FWithTypeInfo:=false;
  FSource:=TStringList.Create;

  FHub:=TPas2JSResolverHub.Create(Self);
  FModules:=TObjectList.Create(true);

  FFilename:='test1.pp';
  FFileResolver:=TStreamResolver.Create;
  FFileResolver.OwnsStreams:=True;

  FScanner:=TPas2jsPasScanner.Create(FFileResolver);
  InitScanner(FScanner);

  FEngine:=AddModule(Filename);
  FEngine.Scanner:=FScanner;
  FScanner.Resolver:=FEngine;

  FParser:=TTestPasParser.Create(FScanner,FFileResolver,FEngine);
  FParser.OnLog:=@OnParserLog;
  FEngine.Parser:=FParser;
  Parser.Options:=po_tcmodules;

  FModule:=Nil;
  FConverter:=CreateConverter;

  FExpectedErrorClass:=nil;
end;

function TCustomTestModule.CreateConverter: TPasToJSConverter;
var
  Options: TPasToJsConverterOptions;
begin
  Result:=TPasToJSConverter.Create;
  Options:=co_tcmodules;
  if WithTypeInfo then
    Exclude(Options,coNoTypeInfo)
  else
    Include(Options,coNoTypeInfo);
  Result.Options:=Options;
  Result.Globals:=TPasToJSConverterGlobals.Create(Result);
end;

procedure TCustomTestModule.InitScanner(aScanner: TPas2jsPasScanner);
begin
  aScanner.AllowedModeSwitches:=msAllPas2jsModeSwitches;
  aScanner.ReadOnlyModeSwitches:=msAllPas2jsModeSwitchesReadOnly;
  aScanner.CurrentModeSwitches:=OBJFPCModeSwitches*msAllPas2jsModeSwitches+msAllPas2jsModeSwitchesReadOnly;

  aScanner.AllowedBoolSwitches:=bsAllPas2jsBoolSwitches;
  aScanner.ReadOnlyBoolSwitches:=bsAllPas2jsBoolSwitchesReadOnly;
  aScanner.CurrentBoolSwitches:=bsAllPas2jsBoolSwitchesReadOnly+[bsHints,bsNotes,bsWarnings,bsWriteableConst];

  aScanner.AllowedValueSwitches:=vsAllPas2jsValueSwitches;
  aScanner.ReadOnlyValueSwitches:=vsAllPas2jsValueSwitchesReadOnly;

  aScanner.OnLog:=@OnScannerLog;

  aScanner.CompilerVersion:='Comp.Ver.tcmodules';
end;

procedure TCustomTestModule.TearDown;
{$IFDEF CheckPasTreeRefCount}
var
  El: TPasElement;
{$ENDIF}
var
  i: Integer;
  CurModule: TPasModule;
begin
  FHintMsgs.Clear;
  FHintMsgsGood.Clear;
  FSkipTests:=false;
  FWithTypeInfo:=false;
  FJSRegModuleCall:=nil;
  FJSModuleCallArgs:=nil;
  FJSImplentationUses:=nil;
  FJSInterfaceUses:=nil;
  FJSModuleSrc:=nil;
  FJSInitBody:=nil;
  FreeAndNil(FJSSource);
  FreeAndNil(FJSModule);
  FreeAndNil(FConverter);
  Engine.Clear;
  FreeAndNil(FSource);
  FreeAndNil(FFileResolver);
  if FModules<>nil then
    begin
    for i:=0 to FModules.Count-1 do
      begin
      CurModule:=TTestEnginePasResolver(FModules[i]).Module;
      if CurModule=nil then continue;
      //writeln('TCustomTestModule.TearDown ReleaseUsedUnits ',CurModule.Name,' ',CurModule.RefCount,' ',CurModule.RefIds.Text);
      CurModule.ReleaseUsedUnits;
      end;
    if FModule<>nil then
      FModule.ReleaseUsedUnits;
    for i:=0 to FModules.Count-1 do
      begin
      CurModule:=TTestEnginePasResolver(FModules[i]).Module;
      if CurModule=nil then continue;
      //writeln('TCustomTestModule.TearDown UsesReleased ',CurModule.Name,' ',CurModule.RefCount,' ',CurModule.RefIds.Text);
      end;
    FreeAndNil(FModules);
    ReleaseAndNil(TPasElement(FModule){$IFDEF CheckPasTreeRefCount},'CreateElement'{$ENDIF});
    FEngine:=nil;
    end;
  FreeAndNil(FHub);

  inherited TearDown;
  {$IFDEF EnablePasTreeGlobalRefCount}
  if FElementRefCountAtSetup<>TPasElement.GlobalRefCount then
    begin
    writeln('TCustomTestModule.TearDown GlobalRefCount Was='+IntToStr(FElementRefCountAtSetup)+' Now='+IntToStr(TPasElement.GlobalRefCount));
    {$IFDEF CheckPasTreeRefCount}
    El:=TPasElement.FirstRefEl;
    while El<>nil do
      begin
      writeln('  ',GetObjName(El),' RefIds.Count=',El.RefIds.Count,':');
      for i:=0 to El.RefIds.Count-1 do
        writeln('    ',El.RefIds[i]);
      El:=El.NextRefEl;
      end;
    {$ENDIF}
    Halt;
    Fail('TCustomTestModule.TearDown Was='+IntToStr(FElementRefCountAtSetup)+' Now='+IntToStr(TPasElement.GlobalRefCount));
    end;
  {$ENDIF}
end;

procedure TCustomTestModule.Add(Line: string);
begin
  Source.Add(Line);
end;

procedure TCustomTestModule.Add(const Lines: array of string);
var
  i: Integer;
begin
  for i:=low(Lines) to high(Lines) do
    Add(Lines[i]);
end;

procedure TCustomTestModule.StartParsing;
var
  Src: String;
begin
  Src:=Source.Text;
  FEngine.Source:=Src;
  FileResolver.AddStream(FileName,TStringStream.Create(Src));
  Scanner.OpenFile(FileName);
  Writeln('// Test : ',Self.TestName);
  Writeln(Src);
end;

procedure TCustomTestModule.ParseModuleQueue;
var
  i: Integer;
  CurResolver: TTestEnginePasResolver;
  Found: Boolean;
  Section: TPasSection;
begin
  // parse til exception or all modules finished
  while not SkipTests do
    begin
    Found:=false;
    for i:=0 to ResolverCount-1 do
      begin
      CurResolver:=Resolvers[i];
      if CurResolver.CurrentParser=nil then continue;
      if not CurResolver.CurrentParser.CanParseContinue(Section) then
        continue;
      CurResolver.Parser.ParseContinue;
      Found:=true;
      break;
      end;
    if not Found then break;
    end;

  for i:=0 to ResolverCount-1 do
    begin
    CurResolver:=Resolvers[i];
    if CurResolver.Parser=nil then
      begin
      if CurResolver.CurrentParser<>nil then
        Fail('TCustomTestModule.ParseModuleQueue '+CurResolver.Filename+' '+GetObjName(CurResolver.Parser)+'=Parser<>CurrentParser='+GetObjName(CurResolver.CurrentParser));
      continue;
      end;
    if CurResolver.Parser.CurModule<>nil then
      Fail('TCustomTestModule.ParseModuleQueue '+CurResolver.Filename+' NOT FINISHED CurModule='+GetObjName(CurResolver.Parser.CurModule));
    end;
end;

procedure TCustomTestModule.ParseModule;
begin
  if SkipTests then exit;
  FFirstPasStatement:=nil;
  try
    StartParsing;
    Parser.ParseMain(FModule);
    ParseModuleQueue;
  except
    on E: Exception do
      HandleException(E);
  end;
  if SkipTests then exit;

  AssertNotNull('Module resulted in Module',Module);
  AssertEquals('modulename',lowercase(ChangeFileExt(FFileName,'')),lowercase(Module.Name));
  TAssert.AssertSame('Has resolver',Engine,Parser.Engine);
end;

procedure TCustomTestModule.ParseProgram;
begin
  if SkipTests then exit;
  ParseModule;
  if SkipTests then exit;
  AssertEquals('Has program',TPasProgram,Module.ClassType);
  FPasProgram:=TPasProgram(Module);
  AssertNotNull('Has program section',PasProgram.ProgramSection);
  AssertNotNull('Has initialization section',PasProgram.InitializationSection);
  if (PasProgram.InitializationSection.Elements.Count>0) then
    if TObject(PasProgram.InitializationSection.Elements[0]) is TPasImplBlock then
      FFirstPasStatement:=TPasImplBlock(PasProgram.InitializationSection.Elements[0]);
end;

procedure TCustomTestModule.ParseLibrary;
var
  Init: TInitializationSection;
begin
  if SkipTests then exit;
  ParseModule;
  if SkipTests then exit;
  AssertEquals('Has library',TPasLibrary,Module.ClassType);
  FPasLibrary:=TPasLibrary(Module);
  AssertNotNull('Has library section',PasLibrary.LibrarySection);
  Init:=PasLibrary.InitializationSection;
  if (Init<>nil) and (Init.Elements.Count>0) then
    if TObject(Init.Elements[0]) is TPasImplBlock then
      FFirstPasStatement:=TPasImplBlock(PasLibrary.InitializationSection.Elements[0]);
end;

procedure TCustomTestModule.ParseUnit;
begin
  if SkipTests then exit;
  ParseModule;
  if SkipTests then exit;
  AssertEquals('Has unit (TPasModule)',TPasModule,Module.ClassType);
  AssertNotNull('Has interface section',Module.InterfaceSection);
  AssertNotNull('Has implementation section',Module.ImplementationSection);
  if (Module.InitializationSection<>nil)
      and (Module.InitializationSection.Elements.Count>0)
      and (TObject(Module.InitializationSection.Elements[0]) is TPasImplBlock) then
    FFirstPasStatement:=TPasImplBlock(Module.InitializationSection.Elements[0]);
end;

function TCustomTestModule.FindModuleWithFilename(aFilename: string
  ): TTestEnginePasResolver;
var
  i: Integer;
begin
  for i:=0 to ResolverCount-1 do
    if CompareText(Resolvers[i].Filename,aFilename)=0 then
      exit(Resolvers[i]);
  Result:=nil;
end;

function TCustomTestModule.AddModule(aFilename: string
  ): TTestEnginePasResolver;
begin
  //writeln('TTestModuleConverter.AddModule ',aFilename);
  if FindModuleWithFilename(aFilename)<>nil then
    Fail('TTestModuleConverter.AddModule: file "'+aFilename+'" already exists');
  Result:=TTestEnginePasResolver.Create;
  Result.Filename:=aFilename;
  Result.AddObjFPCBuiltInIdentifiers(btAllJSBaseTypes,bfAllJSBaseProcs);
  Result.OnFindUnit:=@OnPasResolverFindUnit;
  Result.OnLog:=@OnPasResolverLog;
  Result.Hub:=Hub;
  FModules.Add(Result);
end;

function TCustomTestModule.AddModuleWithSrc(aFilename, Src: string
  ): TTestEnginePasResolver;
begin
  Result:=AddModule(aFilename);
  Result.Source:=Src;
end;

function TCustomTestModule.AddModuleWithIntfImplSrc(aFilename, InterfaceSrc,
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

procedure TCustomTestModule.AddSystemUnit(Parts: TSystemUnitParts);
var
  Intf, Impl: TStringList;
begin
  Intf:=TStringList.Create;
  if supTInterfacedObject in Parts then Include(Parts,supTObject);

  // unit interface
  if [supTVarRec,supTypeInfo]*Parts<>[] then
    Intf.Add('{$modeswitch externalclass}');
  Intf.Add('type');
  Intf.Add('  integer=longint;');
  Intf.Add('  sizeint=nativeint;');
    //'const',
    //'  LineEnding = #10;',
    //'  DirectorySeparator = ''/'';',
    //'  DriveSeparator = '''';',
    //'  AllowDirectorySeparators : set of char = [''\'',''/''];',
    //'  AllowDriveSeparators : set of char = [];',
  if supTObject in Parts then
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
  if supTInterfacedObject in Parts then
    Intf.AddStrings([
    '  {$Interfaces COM}',
    '  IUnknown = interface',
    '    [''{00000000-0000-0000-C000-000000000046}'']',
    //'    function QueryInterface(const iid: TGuid; out obj): Integer;',
    '    function _AddRef: Integer;',
    '    function _Release: Integer;',
    '  end;',
    '  IInterface = IUnknown;',
    '  TInterfacedObject = class(TObject,IUnknown)',
    '  protected',
    '    fRefCount: Integer;',
    '    { implement methods of IUnknown }',
    //'    function QueryInterface(const iid: TGuid; out obj): Integer; virtual;',
    '    function _AddRef: Integer; virtual;',
    '    function _Release: Integer; virtual;',
    '  end;',
    '  TInterfacedClass = class of TInterfacedObject;',
    '',
    '']);
  if supTVarRec in Parts then
    Intf.AddStrings([
    'const',
    '  vtInteger       = 0;',
    '  vtBoolean       = 1;',
    '  vtJSValue       = 19;',
    'type',
    '  PVarRec = ^TVarRec;',
    '  TVarRec = record',
    '    VType : byte;',
    '    VJSValue: JSValue;',
    '    vInteger: longint external name ''VJSValue'';',
    '    vBoolean: boolean external name ''VJSValue'';',
    '  end;',
    '  TVarRecArray = array of TVarRec;',
    'function VarRecs: TVarRecArray; varargs;',
    '']);
  if supTypeInfo in Parts then
    begin
    Intf.AddStrings([
    'type',
    '  TTypeKind = (',
    '    tkUnknown,  // 0',
    '    tkInteger,  // 1',
    '    tkChar,     // 2 in Delphi/FPC tkWChar, tkUChar',
    '    tkString,   // 3 in Delphi/FPC tkSString, tkWString or tkUString',
    '    tkEnumeration, // 4',
    '    tkSet,      // 5',
    '    tkDouble,   // 6',
    '    tkBool,     // 7',
    '    tkProcVar,  // 8  function or procedure',
    '    tkMethod,   // 9  proc var of object',
    '    tkArray,    // 10 static array',
    '    tkDynArray, // 11',
    '    tkRecord,   // 12',
    '    tkClass,    // 13',
    '    tkClassRef, // 14',
    '    tkPointer,  // 15',
    '    tkJSValue,  // 16',
    '    tkRefToProcVar, // 17  variable of procedure type',
    '    tkInterface, // 18',
    '    //tkObject,',
    '    //tkSString,tkLString,tkAString,tkWString,',
    '    //tkVariant,',
    '    //tkWChar,',
    '    //tkInt64,',
    '    //tkQWord,',
    '    //tkInterfaceRaw,',
    '    //tkUString,tkUChar,',
    '    tkHelper,   // 19',
    '    //tkFile,',
    '    tkExtClass  // 20',
    '    );',
    '  TTypeKinds = set of TTypeKind;',
    '  TTypeInfo = class external name ''rtl.tTypeInfo'' end;',
    '  TTypeInfoInteger = class external name ''rtl.tTypeInfoInteger''(TTypeInfo)',
    '  end;',
    '  TTypeInfoEnum = class external name ''rtl.tTypeInfoEnum''(TTypeInfoInteger) end;',
    '  TTypeInfoSet = class external name ''rtl.tTypeInfoSet''(TTypeInfo) end;',
    '  TTypeInfoStaticArray = class external name ''rtl.tTypeInfoStaticArray''(TTypeInfo) end;',
    '  TTypeInfoDynArray = class external name ''rtl.tTypeInfoDynArray''(TTypeInfo) end;',
    '  TTypeInfoProcVar = class external name ''rtl.tTypeInfoProcVar''(TTypeInfo) end;',
    '  TTypeInfoMethodVar = class external name ''rtl.tTypeInfoMethodVar''(TTypeInfoProcVar) end;',
    '  TTypeInfoClass = class external name ''rtl.tTypeInfoClass''(TTypeInfo) end;',
    '  TTypeInfoClassRef = class external name ''rtl.tTypeInfoClassRef''(TTypeInfo) end;',
    '  TTypeInfoExtClass = class external name ''rtl.tTypeInfoExtClass''(TTypeInfo) end;',
    '  TTypeInfoRecord = class external name ''rtl.tTypeInfoRecord''(TTypeInfo) end;',
    '  TTypeInfoPointer = class external name ''rtl.tTypeInfoPointer''(TTypeInfo) end;',
    '  TTypeInfoHelper = class external name ''rtl.tTypeInfoHelper''(TTypeInfo) end;',
    '  TTypeInfoInterface = class external name ''rtl.tTypeInfoInterface''(TTypeInfo) end;',
    '']);
    end;
  if supWriteln in Parts then
    Intf.Add('procedure writeln; varargs; external name ''console.log'';');

  Intf.Add('var');
  Intf.Add('  ExitCode: Longint = 0;');

  // unit implementation
  Impl:=TStringList.Create;
  if supTObject in Parts then
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
  if supTInterfacedObject in Parts then
    Impl.AddStrings([
    //'function TInterfacedObject.QueryInterface(const iid: TGuid; out obj): Integer;',
    //'begin',
    //'end;',
    'function TInterfacedObject._AddRef: Integer;',
    'begin',
    'end;',
    'function TInterfacedObject._Release: Integer;',
    'begin',
    'end;',
    '']);
  if supTVarRec in Parts then
    Impl.AddStrings([
    'function VarRecs: TVarRecArray; varargs;',
    'var',
    '  v: PVarRec;',
    'begin',
    '  v^.VType:=1;',
    '  v^.VJSValue:=2;',
    'end;',
    '']);

  try
    AddModuleWithIntfImplSrc('system.pp',Intf.Text,Impl.Text);
  finally
    Intf.Free;
    Impl.Free;
  end;
end;

procedure TCustomTestModule.StartProgram(NeedSystemUnit: boolean;
  SystemUnitParts: TSystemUnitParts);
begin
  if NeedSystemUnit then
    AddSystemUnit(SystemUnitParts)
  else
    Parser.ImplicitUses.Clear;
  Add('program '+ExtractFileUnitName(Filename)+';');
  Add('');
end;

procedure TCustomTestModule.StartLibrary(NeedSystemUnit: boolean;
  SystemUnitParts: TSystemUnitParts);
begin
  if NeedSystemUnit then
    AddSystemUnit(SystemUnitParts)
  else
    Parser.ImplicitUses.Clear;
  Add('library '+ExtractFileUnitName(Filename)+';');
  Add('');
end;

procedure TCustomTestModule.StartUnit(NeedSystemUnit: boolean;
  SystemUnitParts: TSystemUnitParts);
begin
  if NeedSystemUnit then
    AddSystemUnit(SystemUnitParts)
  else
    Parser.ImplicitUses.Clear;
  Add('unit Test1;');
  Add('');
end;

procedure TCustomTestModule.ConvertModule;

  procedure CheckUsesList(UsesName: String; Arg: TJSArrayLiteralElement;
    out UsesLit: TJSArrayLiteral);
  var
    i: Integer;
    Item: TJSElement;
    Lit: TJSLiteral;
  begin
    UsesLit:=nil;
    AssertNotNull(UsesName+' uses section',Arg.Expr);
    if (Arg.Expr.ClassType=TJSLiteral) and TJSLiteral(Arg.Expr).Value.IsNull then
      exit; // null is ok
    AssertEquals(UsesName+' uses section param is array',TJSArrayLiteral,Arg.Expr.ClassType);
    FJSInterfaceUses:=TJSArrayLiteral(Arg.Expr);
    for i:=0 to FJSInterfaceUses.Elements.Count-1 do
      begin
      Item:=FJSInterfaceUses.Elements.Elements[i].Expr;
      AssertNotNull(UsesName+' uses section item['+IntToStr(i)+'].Expr',Item);
      AssertEquals(UsesName+' uses section item['+IntToStr(i)+'] is lit',TJSLiteral,Item.ClassType);
      Lit:=TJSLiteral(Item);
      AssertEquals(UsesName+' uses section item['+IntToStr(i)+'] is string lit',
        ord(jsbase.jstString),ord(Lit.Value.ValueType));
      end;
  end;

  procedure CheckFunctionParam(ParamName: string; Arg: TJSArrayLiteralElement;
    out Src: TJSSourceElements);
  var
    FunDecl: TJSFunctionDeclarationStatement;
    FunDef: TJSFuncDef;
    FunBody: TJSFunctionBody;
  begin
    Src:=nil;
    AssertNotNull(ParamName,Arg.Expr);
    AssertEquals(ParamName+' Arg.Expr type',TJSFunctionDeclarationStatement,Arg.Expr.ClassType);
    FunDecl:=Arg.Expr as TJSFunctionDeclarationStatement;
    AssertNotNull(ParamName+' FunDecl.AFunction',FunDecl.AFunction);
    AssertEquals(ParamName+' FunDecl.AFunction type',TJSFuncDef,FunDecl.AFunction.ClassType);
    FunDef:=FunDecl.AFunction as TJSFuncDef;
    AssertEquals(ParamName+' name empty','',String(FunDef.Name));
    AssertNotNull(ParamName+' body',FunDef.Body);
    AssertEquals(ParamName+' body type',TJSFunctionBody,FunDef.Body.ClassType);
    FunBody:=FunDef.Body as TJSFunctionBody;
    AssertNotNull(ParamName+' body.A',FunBody.A);
    AssertEquals(ParamName+' body.A type',TJSSourceElements,FunBody.A.ClassType);
    Src:=FunBody.A as TJSSourceElements;
  end;

var
  ModuleNameExpr: TJSLiteral;
  InitFunction: TJSFunctionDeclarationStatement;
  InitAssign: TJSSimpleAssignStatement;
  InitName: String;
  LastNode: TJSElement;
  Arg: TJSArrayLiteralElement;
begin
  if SkipTests then exit;
  try
    FJSModule:=FConverter.ConvertPasElement(Module,Engine) as TJSSourceElements;
  except
    on E: Exception do
      HandleException(E);
  end;
  if SkipTests then exit;
  if ExpectedErrorClass<>nil then
    Fail('Missing '+ExpectedErrorClass.ClassName+' error {'+ExpectedErrorMsg+'} ('+IntToStr(ExpectedErrorNumber)+')');

  FJSSource:=TStringList.Create;
  FJSSource.Text:=ConvertJSModuleToString(JSModule);
  {$IFDEF VerbosePas2JS}
  writeln('TTestModule.ConvertModule JS:');
  write(FJSSource.Text);
  {$ENDIF}

  // rtl.module(...
  AssertEquals('jsmodule has one statement - the call',1,JSModule.Statements.Count);
  AssertNotNull('register module call',JSModule.Statements.Nodes[0].Node);
  AssertEquals('register module call',TJSCallExpression,JSModule.Statements.Nodes[0].Node.ClassType);
  FJSRegModuleCall:=JSModule.Statements.Nodes[0].Node as TJSCallExpression;
  AssertNotNull('register module rtl.module expr',JSRegModuleCall.Expr);
  AssertNotNull('register module rtl.module args',JSRegModuleCall.Args);
  AssertEquals('rtl.module args',TJSArguments,JSRegModuleCall.Args.ClassType);
  FJSModuleCallArgs:=JSRegModuleCall.Args as TJSArguments;

  // parameter 'unitname'
  if JSModuleCallArgs.Elements.Count<1 then
    Fail('rtl.module first param unit missing');
  Arg:=JSModuleCallArgs.Elements.Elements[0];
  AssertNotNull('module name param',Arg.Expr);
  ModuleNameExpr:=Arg.Expr as TJSLiteral;
  AssertEquals('module name param is string',ord(jstString),ord(ModuleNameExpr.Value.ValueType));
  if Module is TPasProgram then
    AssertEquals('module name','program',String(ModuleNameExpr.Value.AsString))
  else if Module is TPasLibrary then
    AssertEquals('module name','library',String(ModuleNameExpr.Value.AsString))
  else
    AssertEquals('module name',Module.Name,String(ModuleNameExpr.Value.AsString));

  // main uses section
  if JSModuleCallArgs.Elements.Count<2 then
    Fail('rtl.module second param main uses missing');
  Arg:=JSModuleCallArgs.Elements.Elements[1];
  CheckUsesList('interface',Arg,FJSInterfaceUses);

  // program/library/interface function()
  if JSModuleCallArgs.Elements.Count<3 then
    Fail('rtl.module third param intf-function missing');
  Arg:=JSModuleCallArgs.Elements.Elements[2];
  CheckFunctionParam('module intf-function',Arg,FJSModuleSrc);

  // search for $mod.$init or $mod.$main - the last statement
  if (Module is TPasProgram) or (Module is TPasLibrary) then
    begin
    InitName:='$main';
    AssertEquals('$mod.'+InitName+' function 1',true,JSModuleSrc.Statements.Count>0);
    end
  else
    InitName:='$init';
  FJSInitBody:=nil;
  if JSModuleSrc.Statements.Count>0 then
    begin
    LastNode:=JSModuleSrc.Statements.Nodes[JSModuleSrc.Statements.Count-1].Node;
    if LastNode is TJSSimpleAssignStatement then
      begin
      InitAssign:=LastNode as TJSSimpleAssignStatement;
      if GetDottedIdentifier(InitAssign.LHS)='$mod.'+InitName then
        begin
        InitFunction:=InitAssign.Expr as TJSFunctionDeclarationStatement;
        FJSInitBody:=InitFunction.AFunction.Body as TJSFunctionBody;
        end
      else if (Module is TPasProgram) or (Module is TPasLibrary) then
        CheckDottedIdentifier('init function',InitAssign.LHS,'$mod.'+InitName);
      end;
    end;

  // optional: implementation uses section
  if JSModuleCallArgs.Elements.Count<4 then
    exit;
  Arg:=JSModuleCallArgs.Elements.Elements[3];
  CheckUsesList('implementation',Arg,FJSImplentationUses);
end;

procedure TCustomTestModule.ConvertProgram;
begin
  Add('end.');
  ParseProgram;
  ConvertModule;
end;

procedure TCustomTestModule.ConvertLibrary;
begin
  Add('end.');
  ParseLibrary;
  ConvertModule;
end;

procedure TCustomTestModule.ConvertUnit;
begin
  Add('end.');
  ParseUnit;
  ConvertModule;
end;

function TCustomTestModule.ConvertJSModuleToString(El: TJSElement): string;
begin
  Result:=tcmodules.JSToStr(El);
end;

procedure TCustomTestModule.CheckDottedIdentifier(Msg: string; El: TJSElement;
  DottedName: string);
begin
  if DottedName='' then
    begin
    AssertNull(Msg,El);
    end
  else
    begin
    AssertNotNull(Msg,El);
    AssertEquals(Msg,DottedName,GetDottedIdentifier(El));
    end;
end;

function TCustomTestModule.GetDottedIdentifier(El: TJSElement): string;
begin
  if El=nil then
    Result:=''
  else if El is TJSPrimaryExpressionIdent then
    Result:=String(TJSPrimaryExpressionIdent(El).Name)
  else if El is TJSDotMemberExpression then
    Result:=GetDottedIdentifier(TJSDotMemberExpression(El).MExpr)+'.'+String(TJSDotMemberExpression(El).Name)
  else
    AssertEquals('GetDottedIdentifier',TJSPrimaryExpressionIdent,El.ClassType);
end;

procedure TCustomTestModule.CheckSource(Msg, Statements: String;
  InitStatements: string; ImplStatements: string);
var
  ActualSrc, ExpectedSrc, InitName: String;
begin
  ActualSrc:=JSToStr(JSModuleSrc);
  if coUseStrict in Converter.Options then
    ExpectedSrc:='"use strict";'+LineEnding
  else
    ExpectedSrc:='';
  ExpectedSrc:=ExpectedSrc+'var $mod = this;'+LineEnding;
  ExpectedSrc:=ExpectedSrc+Statements;

  // unit implementation
  if (Trim(ImplStatements)<>'') then
    ExpectedSrc:=ExpectedSrc+LineEnding
      +'$mod.$implcode = function () {'+LineEnding
      +ImplStatements
      +'};'+LineEnding;

  // program main or unit initialization
  if (Module is TPasProgram) or (Trim(InitStatements)<>'') then
    begin
    if (Module is TPasProgram) or (Module is TPasLibrary) then
      InitName:='$main'
    else
      InitName:='$init';
    ExpectedSrc:=ExpectedSrc+LineEnding
      +'$mod.'+InitName+' = function () {'+LineEnding
      +InitStatements
      +'};'+LineEnding;
    end;

  //writeln('TCustomTestModule.CheckSource ExpectedIntf="',ExpectedSrc,'"');
  //writeln('TTestModule.CheckSource InitStatements="',Trim(InitStatements),'"');
  CheckDiff(Msg,ExpectedSrc,ActualSrc);
end;

procedure TCustomTestModule.CheckDiff(Msg, Expected, Actual: string);
// search diff, ignore changes in spaces
var
  s: string;
begin
  if CheckSrcDiff(Expected,Actual,s) then exit;
  Fail(Msg+': '+s);
end;

procedure TCustomTestModule.CheckUnit(Filename, ExpectedSrc: string);
var
  aResolver: TTestEnginePasResolver;
  aConverter: TPasToJSConverter;
  aJSModule: TJSSourceElements;
  ActualSrc: String;
begin
  aResolver:=GetResolver(Filename);
  AssertNotNull('missing resolver of unit '+Filename,aResolver);
  AssertNotNull('missing resolver.module of unit '+Filename,aResolver.Module);
  {$IFDEF VerbosePas2JS}
  writeln('CheckUnit '+Filename+' converting ...');
  {$ENDIF}
  aConverter:=CreateConverter;
  aJSModule:=nil;
  try
    try
      aJSModule:=aConverter.ConvertPasElement(aResolver.Module,aResolver) as TJSSourceElements;
    except
      on E: Exception do
        HandleException(E);
    end;
    ActualSrc:=ConvertJSModuleToString(aJSModule);
    {$IFDEF VerbosePas2JS}
    writeln('TTestModule.CheckUnit ',Filename,' Pas:');
    write(aResolver.Source);
    writeln('TTestModule.CheckUnit ',Filename,' JS:');
    write(ActualSrc);
    {$ENDIF}
    CheckDiff('Converted unit: "'+ChangeFileExt(Filename,'.js')+'"',ExpectedSrc,ActualSrc);
  finally
    aJSModule.Free;
    aConverter.Free;
  end;
end;

procedure TCustomTestModule.CheckHint(MsgType: TMessageType;
  MsgNumber: integer; Msg: string; Marker: PSrcMarker);
var
  i: Integer;
  Item: TTestHintMessage;
  Expected,Actual: string;
begin
  //writeln('TCustomTestModule.CheckHint MsgCount=',MsgCount);
  for i:=0 to MsgCount-1 do
    begin
    Item:=Msgs[i];
    if (Item.MsgNumber<>MsgNumber) or (Item.Msg<>Msg) then continue;
    if (Marker<>nil) then
      begin
      if Item.SourcePos.Row<>cardinal(Marker^.Row) then continue;
      if (Item.SourcePos.Column<cardinal(Marker^.StartCol))
          or (Item.SourcePos.Column>cardinal(Marker^.EndCol)) then continue;
      end;
    // found
    FHintMsgsGood.Add(Item);
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
    write('TCustomTestModule.CheckHint ',i,'/',MsgCount,' ',Item.MsgType,
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

procedure TCustomTestModule.CheckResolverUnexpectedHints(WithSourcePos: boolean
  );
var
  i: Integer;
  s, Txt: String;
  Msg: TTestHintMessage;
begin
  for i:=0 to MsgCount-1 do
    begin
    Msg:=Msgs[i];
    if FHintMsgsGood.IndexOf(Msg)>=0 then continue;
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

procedure TCustomTestModule.SetExpectedScannerError(Msg: string;
  MsgNumber: integer);
begin
  ExpectedErrorClass:=EScannerError;
  ExpectedErrorMsg:=Msg;
  ExpectedErrorNumber:=MsgNumber;
end;

procedure TCustomTestModule.SetExpectedParserError(Msg: string;
  MsgNumber: integer);
begin
  ExpectedErrorClass:=EParserError;
  ExpectedErrorMsg:=Msg;
  ExpectedErrorNumber:=MsgNumber;
end;

procedure TCustomTestModule.SetExpectedPasResolverError(Msg: string;
  MsgNumber: integer);
begin
  ExpectedErrorClass:=EPasResolve;
  ExpectedErrorMsg:=Msg;
  ExpectedErrorNumber:=MsgNumber;
end;

procedure TCustomTestModule.SetExpectedConverterError(Msg: string;
  MsgNumber: integer);
begin
  ExpectedErrorClass:=EPas2JS;
  ExpectedErrorMsg:=Msg;
  ExpectedErrorNumber:=MsgNumber;
end;

function TCustomTestModule.IsErrorExpected(E: Exception): boolean;
var
  MsgNumber: Integer;
  Msg: String;
begin
  Result:=false;
  if (ExpectedErrorClass=nil) or (ExpectedErrorClass<>E.ClassType) then exit;
  Msg:=E.Message;
  if E is EPas2JS then
    MsgNumber:=EPas2JS(E).MsgNumber
  else if E is EPasResolve then
    MsgNumber:=EPasResolve(E).MsgNumber
  else if E is EParserError then
    MsgNumber:=Parser.LastMsgNumber
  else if E is EScannerError then
    begin
    MsgNumber:=Scanner.LastMsgNumber;
    Msg:=Scanner.LastMsg;
    end
  else
    MsgNumber:=0;
  Result:=(MsgNumber=ExpectedErrorNumber) and (Msg=ExpectedErrorMsg);
  if Result then
    SkipTests:=true;
end;

procedure TCustomTestModule.HandleScannerError(E: EScannerError);
begin
  if IsErrorExpected(E) then exit;
  WriteSources(Scanner.CurFilename,Scanner.CurRow,Scanner.CurColumn);
  writeln('ERROR: TCustomTestModule.HandleScannerError '+E.ClassName+':'+E.Message
    +' '+Scanner.CurFilename
    +'('+IntToStr(Scanner.CurRow)+','+IntToStr(Scanner.CurColumn)+')');
  FailException(E);
end;

procedure TCustomTestModule.HandleParserError(E: EParserError);
begin
  if IsErrorExpected(E) then exit;
  WriteSources(E.Filename,E.Row,E.Column);
  writeln('ERROR: TCustomTestModule.HandleParserError '+E.ClassName+':'+E.Message
    +' '+E.Filename+'('+IntToStr(E.Row)+','+IntToStr(E.Column)+')'
    +' MainModuleScannerLine="'+Scanner.CurLine+'"'
    );
  FailException(E);
end;

procedure TCustomTestModule.HandlePasResolveError(E: EPasResolve);
var
  P: TPasSourcePos;
begin
  if IsErrorExpected(E) then exit;
  P:=E.SourcePos;
  WriteSources(P.FileName,P.Row,P.Column);
  writeln('ERROR: TCustomTestModule.HandlePasResolveError '+E.ClassName+':'+E.Message
    +' '+P.FileName+'('+IntToStr(P.Row)+','+IntToStr(P.Column)+')');
  FailException(E);
end;

procedure TCustomTestModule.HandlePas2JSError(E: EPas2JS);
var
  Row, Col: integer;
begin
  if IsErrorExpected(E) then exit;
  Engine.UnmangleSourceLineNumber(E.PasElement.SourceLinenumber,Row,Col);
  WriteSources(E.PasElement.SourceFilename,Row,Col);
  writeln('ERROR: TCustomTestModule.HandlePas2JSError '+E.ClassName+':'+E.Message
    +' '+E.PasElement.SourceFilename
    +'('+IntToStr(Row)+','+IntToStr(Col)+')');
  FailException(E);
end;

procedure TCustomTestModule.HandleException(E: Exception);
begin
  if E is EScannerError then
    HandleScannerError(EScannerError(E))
  else if E is EParserError then
    HandleParserError(EParserError(E))
  else if E is EPasResolve then
    HandlePasResolveError(EPasResolve(E))
  else if E is EPas2JS then
    HandlePas2JSError(EPas2JS(E))
  else
    begin
    if IsErrorExpected(E) then exit;
    if not (E is EAssertionFailedError) then
      begin
      WriteSources('',0,0);
      writeln('ERROR: TCustomTestModule.HandleException '+E.ClassName+':'+E.Message);
      end;
    FailException(E);
    end;
end;

procedure TCustomTestModule.FailException(E: Exception);
var
  MsgNumber: Integer;
begin
  if ExpectedErrorClass<>nil then
  begin
    if FExpectedErrorClass=E.ClassType then
    begin
      if E is EPas2JS then
        MsgNumber:=EPas2JS(E).MsgNumber
      else if E is EPasResolve then
        MsgNumber:=EPasResolve(E).MsgNumber
      else if E is EParserError then
        MsgNumber:=Parser.LastMsgNumber
      else if E is EScannerError then
        MsgNumber:=Scanner.LastMsgNumber
      else
        MsgNumber:=0;
      AssertEquals('Expected error message ('+IntToStr(ExpectedErrorNumber)+')','{'+ExpectedErrorMsg+'}','{'+E.Message+'}');
      AssertEquals('Expected {'+ExpectedErrorMsg+'}, but got msg {'+E.Message+'} number',
        ExpectedErrorNumber,MsgNumber);
    end else begin
      AssertEquals('Wrong exception class',ExpectedErrorClass.ClassName,E.ClassName);
    end;
  end;
  Fail(E.Message);
end;

procedure TCustomTestModule.WriteSources(const aFilename: string; aRow,
  aCol: integer);
var
  IsSrc: Boolean;
  i, j: Integer;
  SrcLines: TStringList;
  Line: string;
  aModule: TTestEnginePasResolver;
begin
  writeln('TCustomTestModule.WriteSources File="',aFilename,'" Row=',aRow,' Col=',aCol);
  for i:=0 to ResolverCount-1 do
    begin
    aModule:=Resolvers[i];
    SrcLines:=TStringList.Create;
    try
      SrcLines.Text:=aModule.Source;
      IsSrc:=ExtractFilename(aModule.Filename)=ExtractFileName(aFilename);
      writeln('Testcode:-File="',aModule.Filename,'"----------------------------------:');
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
    finally
      SrcLines.Free;
    end;
    end;
end;

function TCustomTestModule.IndexOfResolver(const Filename: string): integer;
var
  i: Integer;
begin
  for i:=0 to ResolverCount-1 do
    if Filename=Resolvers[i].Filename then exit(i);
  Result:=-1;
end;

function TCustomTestModule.GetResolver(const Filename: string
  ): TTestEnginePasResolver;
var
  i: Integer;
begin
  i:=IndexOfResolver(Filename);
  if i<0 then exit(nil);
  Result:=Resolvers[i];
end;

function TCustomTestModule.GetDefaultNamespace: string;
var
  C: TClass;
begin
  Result:='';
  if FModule=nil then exit;
  C:=FModule.ClassType;
  if (C=TPasProgram) or (C=TPasLibrary) or (C=TPasPackage) then
    Result:=Engine.DefaultNameSpace;
end;

constructor TCustomTestModule.Create;
begin
  inherited Create;
  FHintMsgs:=TObjectList.Create(true);
  FHintMsgsGood:=TFPList.Create;
end;

destructor TCustomTestModule.Destroy;
begin
  FreeAndNil(FHintMsgs);
  FreeAndNil(FHintMsgsGood);
  inherited Destroy;
end;

{ TTestModule }

procedure TTestModule.TestReservedWords;
var
  i: integer;
begin
  for i:=low(JSReservedWords) to High(JSReservedWords)-1 do
    if CompareStr(JSReservedWords[i],JSReservedWords[i+1])>=0 then
      Fail('20170203135442 '+JSReservedWords[i]+' >= '+JSReservedWords[i+1]);
  for i:=low(JSReservedGlobalWords) to High(JSReservedGlobalWords)-1 do
    if CompareStr(JSReservedGlobalWords[i],JSReservedGlobalWords[i+1])>=0 then
      Fail('20170203135443 '+JSReservedGlobalWords[i]+' >= '+JSReservedGlobalWords[i+1]);
end;

procedure TTestModule.TestEmptyProgram;
begin
  StartProgram(false);
  Add('begin');
  ConvertProgram;
  CheckSource('TestEmptyProgram','','');
end;

procedure TTestModule.TestEmptyProgramUseStrict;
begin
  Converter.Options:=Converter.Options+[coUseStrict];
  StartProgram(false);
  Add('begin');
  ConvertProgram;
  CheckSource('TestEmptyProgramUseStrict','','');
end;

procedure TTestModule.TestEmptyUnit;
begin
  StartUnit(false);
  Add('interface');
  Add('implementation');
  ConvertUnit;
  CheckSource('TestEmptyUnit',
    LinesToStr([
    ]),
    '');
end;

procedure TTestModule.TestEmptyUnitUseStrict;
begin
  Converter.Options:=Converter.Options+[coUseStrict];
  StartUnit(false);
  Add('interface');
  Add('implementation');
  ConvertUnit;
  CheckSource('TestEmptyUnitUseStrict',
    LinesToStr([
    ''
    ]),
    '');
end;

procedure TTestModule.TestDottedUnitNames;
begin
  AddModuleWithIntfImplSrc('NS1.Unit2.pas',
    LinesToStr([
    'var iV: longint;'
    ]),
    '');

  FFilename:='ns1.test1.pp';
  StartProgram(true);
  Add('uses unIt2;');
  Add('var');
  Add('  i: longint;');
  Add('begin');
  Add('  i:=iv;');
  Add('  i:=uNit2.iv;');
  Add('  i:=Ns1.TEst1.i;');
  ConvertProgram;
  CheckSource('TestDottedUnitNames',
    LinesToStr([
    'this.i = 0;',
    '']),
    LinesToStr([ // this.$init
    '$mod.i = pas["NS1.Unit2"].iV;',
    '$mod.i = pas["NS1.Unit2"].iV;',
    '$mod.i = $mod.i;',
    '']) );
end;

procedure TTestModule.TestDottedUnitNameImpl;
begin
  AddModuleWithIntfImplSrc('TEST.UnitA.pas',
    LinesToStr([
    'type',
    '  TObject = class end;',
    '  TTestA = class',
    '  end;'
    ]),
    LinesToStr(['uses TEST.UnitB;'])
    );
  AddModuleWithIntfImplSrc('TEST.UnitB.pas',
    LinesToStr([
    'uses TEST.UnitA;',
    'type TTestB = class(TTestA);'
    ]),
    ''
    );
  StartProgram(true);
  Add('uses TEST.UnitA;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestDottedUnitNameImpl',
    LinesToStr([
    '']),
    LinesToStr([ // this.$init
    '']) );
  CheckUnit('TEST.UnitA.pas',
    LinesToStr([
    'rtl.module("TEST.UnitA", ["system"], function () {',
    '  var $mod = this;',
    '  rtl.createClass(this, "TObject", null, function () {',
    '    this.$init = function () {',
    '    };',
    '    this.$final = function () {',
    '    };',
    '  });',
    '  rtl.createClass(this, "TTestA", this.TObject, function () {',
    '  });',
    '}, ["TEST.UnitB"]);'
    ]));
  CheckUnit('TEST.UnitB.pas',
    LinesToStr([
    'rtl.module("TEST.UnitB", ["system","TEST.UnitA"], function () {',
    '  var $mod = this;',
    '  rtl.createClass(this, "TTestB", pas["TEST.UnitA"].TTestA, function () {',
    '  });',
    '});'
    ]));
end;

procedure TTestModule.TestDottedUnitExpr;
begin
  AddModuleWithIntfImplSrc('NS2.SubNs2.Unit2.pas',
    LinesToStr([
    'procedure DoIt;'
    ]),
    'procedure DoIt; begin end;');

  FFilename:='Ns1.SubNs1.Test1.pp';
  StartProgram(true);
  Add('uses Ns2.sUbnS2.unIt2;');
  Add('var');
  Add('  i: longint;');
  Add('begin');
  Add('  ns2.subns2.unit2.doit;');
  Add('  i:=Ns1.SubNS1.TEst1.i;');
  ConvertProgram;
  CheckSource('TestDottedUnitExpr',
    LinesToStr([
    'this.i = 0;',
    '']),
    LinesToStr([ // this.$init
    'pas["NS2.SubNs2.Unit2"].DoIt();',
    '$mod.i = $mod.i;',
    '']) );
end;

procedure TTestModule.Test_ModeFPCFail;
begin
  StartProgram(false);
  Add('{$mode FPC}');
  Add('begin');
  SetExpectedScannerError('Invalid mode: "FPC"',nErrInvalidMode);
  ConvertProgram;
end;

procedure TTestModule.Test_ModeSwitchCBlocksFail;
begin
  StartProgram(false);
  Add('{$modeswitch cblocks-}');
  Add('begin');
  ConvertProgram;
  CheckHint(mtWarning,nErrInvalidModeSwitch,'Warning: test1.pp(3,23) : Invalid mode switch: "cblocks"');
  CheckResolverUnexpectedHints();
end;

procedure TTestModule.TestUnit_UseSystem;
begin
  StartUnit(true);
  Add([
  'interface',
  'var i: integer;',
  'implementation']);
  ConvertUnit;
  CheckSource('TestUnit_UseSystem',
    LinesToStr([
    'this.i = 0;',
    '']),
    LinesToStr([
    '']) );
end;

procedure TTestModule.TestUnit_Intf1Impl2Intf1;
begin
  AddModuleWithIntfImplSrc('unit1.pp',
    LinesToStr([
    'type number = longint;']),
    LinesToStr([
    'uses test1;',
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
  ConvertUnit;
  CheckSource('TestUnit_Intf1Impl2Intf1',
    LinesToStr([
    'this.i = 0;',
    '']),
    LinesToStr([
    '']) );
end;

procedure TTestModule.TestIncludeVersion;
begin
  StartProgram(false);
  Add([
  'var',
  '  s: string;',
  '  i: word;',
  'begin',
  '  s:={$I %line%};',
  '  i:={$I %linenum%};',
  '  s:={$I %currentroutine%};',
  '  s:={$I %pas2jsversion%};',
  '  s:={$I %pas2jstarget%};',
  '  s:={$I %pas2jstargetos%};',
  '  s:={$I %pas2jstargetcpu%};',
  '  s:={$I %file%};',
  '']);
  ConvertProgram;
  CheckSource('TestIncludeVersion',
    LinesToStr([
    'this.s="";',
    'this.i = 0;']),
    LinesToStr([
    '$mod.s = "7";',
    '$mod.i = 8;',
    '$mod.s = "<anonymous>";',
    '$mod.s = "Comp.Ver.tcmodules";',
    '$mod.s = "Browser";',
    '$mod.s = "Browser";',
    '$mod.s = "ECMAScript5";',
    '$mod.s = "test1.pp";',
    '']));
end;

procedure TTestModule.TestVarInt;
begin
  StartProgram(false);
  Add('var MyI: longint;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestVarInt','this.MyI=0;','');
end;

procedure TTestModule.TestVarBaseTypes;
begin
  StartProgram(false);
  Add('var');
  Add('  i: longint;');
  Add('  s: string;');
  Add('  c: char;');
  Add('  b: boolean;');
  Add('  d: double;');
  Add('  i2: longint = 3;');
  Add('  s2: string = ''foo'';');
  Add('  c2: char = ''4'';');
  Add('  b2: boolean = true;');
  Add('  d2: double = 5.6;');
  Add('  i3: longint = $707;');
  Add('  i4: nativeint = 9007199254740991;');
  Add('  i5: nativeint = -9007199254740991-1;');
  Add('  i6: nativeint =   $fffffffffffff;');
  Add('  i7: nativeint = -$fffffffffffff-1;');
  Add('  i8: byte = 00;');
  Add('  u8: nativeuint =  $fffffffffffff;');
  Add('  u9: nativeuint =  $0000000000000;');
  Add('  u10: nativeuint = $00ff00;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestVarBaseTypes',
    LinesToStr([
    'this.i = 0;',
    'this.s = "";',
    'this.c = "";',
    'this.b = false;',
    'this.d = 0.0;',
    'this.i2 = 3;',
    'this.s2 = "foo";',
    'this.c2 = "4";',
    'this.b2 = true;',
    'this.d2 = 5.6;',
    'this.i3 = 0x707;',
    'this.i4 = 9007199254740991;',
    'this.i5 = -9007199254740991-1;',
    'this.i6 = 0xfffffffffffff;',
    'this.i7 =-0xfffffffffffff-1;',
    'this.i8 = 0;',
    'this.u8 = 0xfffffffffffff;',
    'this.u9 = 0x0;',
    'this.u10 = 0xff00;'
    ]),
    '');
end;

procedure TTestModule.TestBaseTypeSingleFail;
begin
  StartProgram(false);
  Add('var s: single;');
  SetExpectedPasResolverError('identifier not found "single"',PasResolveEval.nIdentifierNotFound);
  ConvertProgram;
end;

procedure TTestModule.TestBaseTypeExtendedFail;
begin
  StartProgram(false);
  Add('var e: extended;');
  SetExpectedPasResolverError('identifier not found "extended"',PasResolveEval.nIdentifierNotFound);
  ConvertProgram;
end;

procedure TTestModule.TestConstBaseTypes;
begin
  StartProgram(false);
  Add('const');
  Add('  i: longint = 3;');
  Add('  s: string = ''foo'';');
  Add('  c: char = ''4'';');
  Add('  b: boolean = true;');
  Add('  d: double = 5.6;');
  Add('  e = low(word);');
  Add('  f = high(word);');
  Add('begin');
  ConvertProgram;
  CheckSource('TestVarBaseTypes',
    LinesToStr([
    'this.i=3;',
    'this.s="foo";',
    'this.c="4";',
    'this.b=true;',
    'this.d=5.6;',
    'this.e = 0;',
    'this.f = 65535;'
    ]),
    '');
end;

procedure TTestModule.TestAliasTypeRef;
begin
  StartProgram(false);
  Add('type');
  Add('  a=longint;');
  Add('  b=a;');
  Add('var');
  Add('  c: A;');
  Add('  d: B;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestAliasTypeRef',
    LinesToStr([ // statements
    'this.c = 0;',
    'this.d = 0;'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestTypeCast_BaseTypes;
begin
  StartProgram(false);
  Add([
  'var',
  '  i: longint;',
  '  b: boolean;',
  '  d: double;',
  '  s: string;',
  '  c: char;',
  'begin',
  '  i:=longint(i);',
  '  i:=longint(b);',
  '  b:=boolean(b);',
  '  b:=boolean(i);',
  '  d:=double(d);',
  '  d:=double(i);',
  '  s:=string(s);',
  '  s:=string(c);',
  '  c:=char(c);',
  '  c:=char(i);',
  '  c:=char(65);',
  '  c:=char(#10);',
  '  c:=char(#$E000);',
  '']);
  ConvertProgram;
  CheckSource('TestAliasTypeRef',
    LinesToStr([ // statements
    'this.i = 0;',
    'this.b = false;',
    'this.d = 0.0;',
    'this.s = "";',
    'this.c = "";',
    '']),
    LinesToStr([ // this.$main
    '$mod.i = $mod.i;',
    '$mod.i = ($mod.b ? 1 : 0);',
    '$mod.b = $mod.b;',
    '$mod.b = $mod.i != 0;',
    '$mod.d = $mod.d;',
    '$mod.d = $mod.i;',
    '$mod.s = $mod.s;',
    '$mod.s = $mod.c;',
    '$mod.c = $mod.c;',
    '$mod.c = String.fromCharCode($mod.i);',
    '$mod.c = "A";',
    '$mod.c = "\n";',
    '$mod.c = "";',
    '']));
end;

procedure TTestModule.TestTypeCast_AliasBaseTypes;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TYesNo = boolean;');
  Add('  TFloat = double;');
  Add('  TCaption = string;');
  Add('  TChar = char;');
  Add('var');
  Add('  i: integer;');
  Add('  b: TYesNo;');
  Add('  d: TFloat;');
  Add('  s: TCaption;');
  Add('  c: TChar;');
  Add('begin');
  Add('  i:=integer(i);');
  Add('  i:=integer(b);');
  Add('  b:=TYesNo(b);');
  Add('  b:=TYesNo(i);');
  Add('  d:=TFloat(d);');
  Add('  d:=TFloat(i);');
  Add('  s:=TCaption(s);');
  Add('  s:=TCaption(c);');
  Add('  c:=TChar(c);');
  ConvertProgram;
  CheckSource('TestAliasTypeRef',
    LinesToStr([ // statements
    'this.i = 0;',
    'this.b = false;',
    'this.d = 0.0;',
    'this.s = "";',
    'this.c = "";',
    '']),
    LinesToStr([ // this.$main
    '$mod.i = $mod.i;',
    '$mod.i = ($mod.b ? 1 : 0);',
    '$mod.b = $mod.b;',
    '$mod.b = $mod.i != 0;',
    '$mod.d = $mod.d;',
    '$mod.d = $mod.i;',
    '$mod.s = $mod.s;',
    '$mod.s = $mod.c;',
    '$mod.c = $mod.c;',
    '']));
end;

procedure TTestModule.TestEmptyProc;
begin
  StartProgram(false);
  Add('procedure Test;');
  Add('begin');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestEmptyProc',
    LinesToStr([ // statements
    'this.Test = function () {',
    '};'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestProcOneParam;
begin
  StartProgram(false);
  Add('procedure ProcA(i: longint);');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  PROCA(3);');
  ConvertProgram;
  CheckSource('TestProcOneParam',
    LinesToStr([ // statements
    'this.ProcA = function (i) {',
    '};'
    ]),
    LinesToStr([ // this.$main
    '$mod.ProcA(3);'
    ]));
end;

procedure TTestModule.TestFunctionWithoutParams;
begin
  StartProgram(false);
  Add('function FuncA: longint;');
  Add('begin');
  Add('end;');
  Add('var i: longint;');
  Add('begin');
  Add('  I:=FUNCA();');
  Add('  I:=FUNCA;');
  Add('  FUNCA();');
  Add('  FUNCA;');
  ConvertProgram;
  CheckSource('TestProcWithoutParams',
    LinesToStr([ // statements
    'this.FuncA = function () {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    'this.i=0;'
    ]),
    LinesToStr([ // this.$main
    '$mod.i=$mod.FuncA();',
    '$mod.i=$mod.FuncA();',
    '$mod.FuncA();',
    '$mod.FuncA();'
    ]));
end;

procedure TTestModule.TestProcedureWithoutParams;
begin
  StartProgram(false);
  Add('procedure ProcA;');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  PROCA();');
  Add('  PROCA;');
  ConvertProgram;
  CheckSource('TestProcWithoutParams',
    LinesToStr([ // statements
    'this.ProcA = function () {',
    '};'
    ]),
    LinesToStr([ // this.$main
    '$mod.ProcA();',
    '$mod.ProcA();'
    ]));
end;

procedure TTestModule.TestIncDec;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(var i: longint);',
  'begin',
  '  inc(i);',
  '  inc(i,2);',
  'end;',
  'var',
  '  Bar: longint;',
  'begin',
  '  inc(bar);',
  '  inc(bar,2);',
  '  dec(bar);',
  '  dec(bar,3);',
  '']);
  ConvertProgram;
  CheckSource('TestIncDec',
    LinesToStr([ // statements
    'this.DoIt = function (i) {',
    '  i.set(i.get()+1);',
    '  i.set(i.get()+2);',
    '};',
    'this.Bar = 0;'
    ]),
    LinesToStr([ // this.$main
    '$mod.Bar+=1;',
    '$mod.Bar+=2;',
    '$mod.Bar-=1;',
    '$mod.Bar-=3;'
    ]));
end;

procedure TTestModule.TestLoHiFpcMode;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  'const',
  '  LoByte1 = Lo(Word($1234));',
  '  HiByte1 = Hi(Word($1234));',
  '  LoByte2 = Lo(SmallInt($1234));',
  '  HiByte2 = Hi(SmallInt($1234));',
  '  LoWord1 = Lo($1234CDEF);',
  '  HiWord1 = Hi($1234CDEF);',
  '  LoWord2 = Lo(-$1234CDEF);',
  '  HiWord2 = Hi(-$1234CDEF);',
  '  lo4:byte=lo(byte($34));',
  '  hi4:byte=hi(byte($34));',
  '  lo5:byte=lo(shortint(-$34));',
  '  hi5:byte=hi(shortint(-$34));',
  '  lo6:longword=lo($123456789ABCD);',
  '  hi6:longword=hi($123456789ABCD);',
  '  lo7:longword=lo(-$123456789ABCD);',
  '  hi7:longword=hi(-$123456789ABCD);',
  'var',
  '  b: Byte;',
  '  ss: shortint;',
  '  w: Word;',
  '  si: SmallInt;',
  '  lw: LongWord;',
  '  li: LongInt;',
  '  b2: Byte;',
  '  ni: nativeint;',
  'begin',
  '  w := $1234;',
  '  ss := -$12;',
  '  b := lo(ss);',
  '  b := HI(ss);',
  '  b := lo(w);',
  '  b := HI(w);',
  '  b2 := lo(b);',
  '  b2 := hi(b);',
  '  lw := $1234CDEF;',
  '  w := lo(lw);',
  '  w := hi(lw);',
  '  ni := $123456789ABCD;',
  '  lw := lo(ni);',
  '  lw := hi(ni);',
  '']);
  ConvertProgram;
  CheckSource('TestLoHiFpcMode',
    LinesToStr([ // statements
    'this.LoByte1 = 0x1234 & 0xFF;',
    'this.HiByte1 = (0x1234 >> 8) & 0xFF;',
    'this.LoByte2 = 0x1234 & 0xFF;',
    'this.HiByte2 = (0x1234 >> 8) & 0xFF;',
    'this.LoWord1 = 0x1234CDEF & 0xFFFF;',
    'this.HiWord1 = (0x1234CDEF >> 16) & 0xFFFF;',
    'this.LoWord2 = -0x1234CDEF & 0xFFFF;',
    'this.HiWord2 = (-0x1234CDEF >> 16) & 0xFFFF;',
    'this.lo4 = 0x34 & 0xF;',
    'this.hi4 = (0x34 >> 4) & 0xF;',
    'this.lo5 = (((-0x34 & 255) << 24) >> 24) & 0xFF;',
    'this.hi5 = ((((-0x34 & 255) << 24) >> 24) >> 8) & 0xFF;',
    'this.lo6 = 0x123456789ABCD >>> 0;',
    'this.hi6 = 74565 >>> 0;',
    'this.lo7 = -0x123456789ABCD >>> 0;',
    'this.hi7 = Math.floor(-0x123456789ABCD / 4294967296) >>> 0;',
    'this.b = 0;',
    'this.ss = 0;',
    'this.w = 0;',
    'this.si = 0;',
    'this.lw = 0;',
    'this.li = 0;',
    'this.b2 = 0;',
    'this.ni = 0;',
    '']),
    LinesToStr([ // this.$main
    '$mod.w = 0x1234;',
    '$mod.ss = -0x12;',
    '$mod.b = $mod.ss & 0xFF;',
    '$mod.b = ($mod.ss >> 8) & 0xFF;',
    '$mod.b = $mod.w & 0xFF;',
    '$mod.b = ($mod.w >> 8) & 0xFF;',
    '$mod.b2 = $mod.b & 0xF;',
    '$mod.b2 = ($mod.b >> 4) & 0xF;',
    '$mod.lw = 0x1234CDEF;',
    '$mod.w = $mod.lw & 0xFFFF;',
    '$mod.w = ($mod.lw >> 16) & 0xFFFF;',
    '$mod.ni = 0x123456789ABCD;',
    '$mod.lw = $mod.ni >>> 0;',
    '$mod.lw = Math.floor($mod.ni / 4294967296) >>> 0;',
    '']));
end;

procedure TTestModule.TestLoHiDelphiMode;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'const',
  '  LoByte1 = Lo(Word($1234));',
  '  HiByte1 = Hi(Word($1234));',
  '  LoByte2 = Lo(SmallInt($1234));',
  '  HiByte2 = Hi(SmallInt($1234));',
  '  LoByte3 = Lo($1234CDEF);',
  '  HiByte3 = Hi($1234CDEF);',
  '  LoByte4 = Lo(-$1234CDEF);',
  '  HiByte4 = Hi(-$1234CDEF);',
  'var',
  '  b: Byte;',
  '  w: Word;',
  '  si: SmallInt;',
  '  lw: LongWord;',
  '  li: LongInt;',
  'begin',
  '  w := $1234;',
  '  b := lo(w);',
  '  b := HI(w);',
  '  lw := $1234CDEF;',
  '  b := lo(lw);',
  '  b := hi(lw);',
  '']);
  ConvertProgram;
  CheckSource('TestLoHiDelphiMode',
    LinesToStr([ // statements
    'this.LoByte1 = 0x1234 & 0xFF;',
    'this.HiByte1 = (0x1234 >> 8) & 0xFF;',
    'this.LoByte2 = 0x1234 & 0xFF;',
    'this.HiByte2 = (0x1234 >> 8) & 0xFF;',
    'this.LoByte3 = 0x1234CDEF & 0xFF;',
    'this.HiByte3 = (0x1234CDEF >> 8) & 0xFF;',
    'this.LoByte4 = -0x1234CDEF & 0xFF;',
    'this.HiByte4 = (-0x1234CDEF >> 8) & 0xFF;',
    'this.b = 0;',
    'this.w = 0;',
    'this.si = 0;',
    'this.lw = 0;',
    'this.li = 0;'
    ]),
    LinesToStr([ // this.$main
    '$mod.w = 0x1234;',
    '$mod.b = $mod.w & 0xFF;',
    '$mod.b = ($mod.w >> 8) & 0xFF;',
    '$mod.lw = 0x1234CDEF;',
    '$mod.b = $mod.lw & 0xFF;',
    '$mod.b = ($mod.lw >> 8) & 0xFF;'
    ]));
end;

procedure TTestModule.TestAssignments;
begin
  StartProgram(false);
  Parser.Options:=Parser.Options+[po_cassignments];
  Add('var');
  Add('  Bar:longint;');
  Add('begin');
  Add('  bar:=3;');
  Add('  bar+=4;');
  Add('  bar-=5;');
  Add('  bar*=6;');
  ConvertProgram;
  CheckSource('TestAssignments',
    LinesToStr([ // statements
    'this.Bar = 0;'
    ]),
    LinesToStr([ // this.$main
    '$mod.Bar=3;',
    '$mod.Bar+=4;',
    '$mod.Bar-=5;',
    '$mod.Bar*=6;'
    ]));
end;

procedure TTestModule.TestArithmeticOperators1;
begin
  StartProgram(false);
  Add('var');
  Add('  vA,vB,vC:longint;');
  Add('begin');
  Add('  va:=1;');
  Add('  vb:=va+va;');
  Add('  vb:=va div vb;');
  Add('  vb:=va mod vb;');
  Add('  vb:=va+va*vb+va div vb;');
  Add('  vc:=-va;');
  Add('  va:=va-vb;');
  Add('  vb:=va;');
  Add('  if va<vb then vc:=va else vc:=vb;');
  ConvertProgram;
  CheckSource('TestArithmeticOperators1',
    LinesToStr([ // statements
    'this.vA = 0;',
    'this.vB = 0;',
    'this.vC = 0;'
    ]),
    LinesToStr([ // this.$main
    '$mod.vA = 1;',
    '$mod.vB = $mod.vA + $mod.vA;',
    '$mod.vB = rtl.trunc($mod.vA / $mod.vB);',
    '$mod.vB = $mod.vA % $mod.vB;',
    '$mod.vB = $mod.vA + ($mod.vA * $mod.vB) + rtl.trunc($mod.vA / $mod.vB);',
    '$mod.vC = -$mod.vA;',
    '$mod.vA = $mod.vA - $mod.vB;',
    '$mod.vB = $mod.vA;',
    'if ($mod.vA < $mod.vB){ $mod.vC = $mod.vA } else $mod.vC = $mod.vB;'
    ]));
end;

procedure TTestModule.TestMultiAdd;
begin
  StartProgram(false);
  Add([
  'function Fly: string; external name ''fly'';',
  'function TryEncodeDate(Year, Month, Day: Word): Boolean;',
  'var',
  '  Date: double;',
  'begin',
  '  Result:=(Year>0) and (Year<10000) and',
  '          (Month >= 1) and (Month<=12) and',
  '          (Day>0) and (Day<=31);',
  '  Date := (146097*Year) SHR 2 + (1461*Year) SHR 2 + (153*LongWord(Month)+2) DIV 5 + LongWord(Day);',
  'end;',
  'var s: string;',
  'begin',
  '  s:=''a''+''b''+''c''+''d'';',
  '  s:=s+Fly+''e'';',
  '  s:=Fly+Fly+Fly;',
  '']);
  ConvertProgram;
  CheckSource('TestMultiAdd',
    LinesToStr([ // statements
    'this.TryEncodeDate = function (Year, Month, Day) {',
    '  var Result = false;',
    '  var date = 0.0;',
    '  Result = (Year > 0) && (Year < 10000) && (Month >= 1) && (Month <= 12) && (Day > 0) && (Day <= 31);',
    '  date = ((146097 * Year) >>> 2) + ((1461 * Year) >>> 2) + rtl.trunc(((153 * Month) + 2) / 5) + Day;',
    '  return Result;',
    '};',
    'this.s = "";',
    '']),
    LinesToStr([ // this.$main
    '$mod.s = "a" + "b" + "c" + "d";',
    '$mod.s = $mod.s + fly() + "e";',
    '$mod.s = fly() + fly() + fly();',
    '']));
end;

procedure TTestModule.TestLogicalOperators;
begin
  StartProgram(false);
  Add('var');
  Add('  vA,vB,vC:boolean;');
  Add('begin');
  Add('  va:=vb and vc;');
  Add('  va:=vb or vc;');
  Add('  va:=vb xor vc;');
  Add('  va:=true and vc;');
  Add('  va:=(vb and vc) or (va and vb);');
  Add('  va:=not vb;');
  ConvertProgram;
  CheckSource('TestLogicalOperators',
    LinesToStr([ // statements
    'this.vA = false;',
    'this.vB = false;',
    'this.vC = false;'
    ]),
    LinesToStr([ // this.$main
    '$mod.vA = $mod.vB && $mod.vC;',
    '$mod.vA = $mod.vB || $mod.vC;',
    '$mod.vA = $mod.vB ^ $mod.vC;',
    '$mod.vA = true && $mod.vC;',
    '$mod.vA = ($mod.vB && $mod.vC) || ($mod.vA && $mod.vB);',
    '$mod.vA = !$mod.vB;'
    ]));
end;

procedure TTestModule.TestBitwiseOperators;
begin
  StartProgram(false);
  Add([
  'var',
  '  vA,vB,vC:longint;',
  '  X,Y,Z: nativeint;',
  'begin',
  '  va:=vb and vc;',
  '  va:=vb or vc;',
  '  va:=vb xor vc;',
  '  va:=vb shl vc;',
  '  va:=vb shr vc;',
  '  va:=3 and vc;',
  '  va:=(vb and vc) or (va and vb);',
  '  va:=not vb;',
  '  X:=Y and Z;',
  '  X:=Y and va;',
  '  X:=Y or Z;',
  '  X:=Y or va;',
  '  X:=Y xor Z;',
  '  X:=Y xor va;',
  '']);
  ConvertProgram;
  CheckSource('TestBitwiseOperators',
    LinesToStr([ // statements
    'this.vA = 0;',
    'this.vB = 0;',
    'this.vC = 0;',
    'this.X = 0;',
    'this.Y = 0;',
    'this.Z = 0;',
    '']),
    LinesToStr([ // this.$main
    '$mod.vA = $mod.vB & $mod.vC;',
    '$mod.vA = $mod.vB | $mod.vC;',
    '$mod.vA = $mod.vB ^ $mod.vC;',
    '$mod.vA = $mod.vB << $mod.vC;',
    '$mod.vA = $mod.vB >>> $mod.vC;',
    '$mod.vA = 3 & $mod.vC;',
    '$mod.vA = ($mod.vB & $mod.vC) | ($mod.vA & $mod.vB);',
    '$mod.vA = ~$mod.vB;',
    '$mod.X = rtl.and($mod.Y, $mod.Z);',
    '$mod.X = $mod.Y & $mod.vA;',
    '$mod.X = rtl.or($mod.Y, $mod.Z);',
    '$mod.X = rtl.or($mod.Y, $mod.vA);',
    '$mod.X = rtl.xor($mod.Y, $mod.Z);',
    '$mod.X = rtl.xor($mod.Y, $mod.vA);',
    '']));
end;

procedure TTestModule.TestBitwiseOperatorsLongword;
begin
  StartProgram(false);
  Add([
  'var',
  '  a,b,c:longword;',
  '  i: longint;',
  'begin',
  '  a:=$12345678;',
  '  b:=$EDCBA987;',
  '  c:=not a;',
  '  c:=a and b;',
  '  c:=a and $ffff0000;',
  '  c:=a or b;',
  '  c:=a or $ff00ff00;',
  '  c:=a xor b;',
  '  c:=a xor $f0f0f0f0;',
  '  c:=a shl 1;',
  '  c:=a shl 16;',
  '  c:=a shl 24;',
  '  c:=a shl b;',
  '  c:=a shr 1;',
  '  c:=a shr 16;',
  '  c:=a shr 24;',
  '  c:=a shr b;',
  '  c:=(b and c) or (a and b);',
  '  c:=i and a;',
  '  c:=i or a;',
  '  c:=i xor a;',
  '']);
  ConvertProgram;
  CheckSource('TestBitwiseOperatorsLongword',
    LinesToStr([ // statements
    'this.a = 0;',
    'this.b = 0;',
    'this.c = 0;',
    'this.i = 0;',
    '']),
    LinesToStr([ // this.$main
    '$mod.a = 0x12345678;',
    '$mod.b = 0xEDCBA987;',
    '$mod.c = rtl.lw(~$mod.a);',
    '$mod.c = rtl.lw($mod.a & $mod.b);',
    '$mod.c = rtl.lw($mod.a & 0xffff0000);',
    '$mod.c = rtl.lw($mod.a | $mod.b);',
    '$mod.c = rtl.lw($mod.a | 0xff00ff00);',
    '$mod.c = rtl.lw($mod.a ^ $mod.b);',
    '$mod.c = rtl.lw($mod.a ^ 0xf0f0f0f0);',
    '$mod.c = rtl.lw($mod.a << 1);',
    '$mod.c = rtl.lw($mod.a << 16);',
    '$mod.c = rtl.lw($mod.a << 24);',
    '$mod.c = rtl.lw($mod.a << $mod.b);',
    '$mod.c = rtl.lw($mod.a >>> 1);',
    '$mod.c = rtl.lw($mod.a >>> 16);',
    '$mod.c = rtl.lw($mod.a >>> 24);',
    '$mod.c = rtl.lw($mod.a >>> $mod.b);',
    '$mod.c = rtl.lw(rtl.lw($mod.b & $mod.c) | rtl.lw($mod.a & $mod.b));',
    '$mod.c = $mod.i & $mod.a;',
    '$mod.c = $mod.i | $mod.a;',
    '$mod.c = $mod.i ^ $mod.a;',
    '']));
end;

procedure TTestModule.TestPrgProcVar;
begin
  StartProgram(false);
  Add('procedure Proc1;');
  Add('type');
  Add('  t1=longint;');
  Add('var');
  Add('  vA:t1;');
  Add('begin');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestPrgProcVar',
    LinesToStr([ // statements
    'this.Proc1 = function () {',
    '  var vA=0;',
    '};'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestUnitProcVar;
begin
  StartUnit(false);
  Add('interface');
  Add('');
  Add('type tA=string; // unit scope');
  Add('procedure Proc1;');
  Add('');
  Add('implementation');
  Add('');
  Add('procedure Proc1;');
  Add('type tA=longint; // local proc scope');
  Add('var  v1:tA; // using local tA');
  Add('begin');
  Add('end;');
  Add('var  v2:tA; // using interface tA');
  ConvertUnit;
  CheckSource('TestUnitProcVar',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    'this.Proc1 = function () {',
    '  var v1 = 0;',
    '};',
    '']),
    // this.$init
    '',
    // implementation
    LinesToStr([
    '$impl.v2 = "";',
    '']));
end;

procedure TTestModule.TestImplProc;
begin
  StartUnit(false);
  Add('interface');
  Add('');
  Add('procedure Proc1;');
  Add('');
  Add('implementation');
  Add('');
  Add('procedure Proc1; begin end;');
  Add('procedure Proc2; begin end;');
  Add('initialization');
  Add('  Proc1;');
  Add('  Proc2;');
  ConvertUnit;
  CheckSource('TestImplProc',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    'this.Proc1 = function () {',
    '};',
    '']),
    LinesToStr([ // this.$init
    '$mod.Proc1();',
    '$impl.Proc2();',
    '']),
    LinesToStr([ // implementation
    '$impl.Proc2 = function () {',
    '};',
    ''])
    );
end;

procedure TTestModule.TestFunctionResult;
begin
  StartProgram(false);
  Add('function Func1: longint;');
  Add('begin');
  Add('  Result:=3;');
  Add('  Func1:=4;');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestFunctionResult',
    LinesToStr([ // statements
    'this.Func1 = function () {',
    '  var Result = 0;',
    '  Result = 3;',
    '  Result = 4;',
    '  return Result;',
    '};'
    ]),
    '');
end;

procedure TTestModule.TestNestedProc;
begin
  StartProgram(false);
  Add([
  'var vInUnit: longint;',
  'function DoIt(pA,pD: longint): longint;',
  'var',
  '  vB: longint;',
  '  vC: longint;',
  '  function Nesty(pA: longint): longint; ',
  '  var vB: longint;',
  '  begin',
  '    Result:=pa+vb+vc+pd+vInUnit;',
  '    nesty:=3;',
  '    doit:=4;',
  '    exit;',
  '  end;',
  'begin',
  '  Result:=pa+vb+vc;',
  '  doit:=6;',
  '  exit;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestNestedProc',
    LinesToStr([ // statements
    'this.vInUnit = 0;',
    'this.DoIt = function (pA, pD) {',
    '  var Result = 0;',
    '  var vB = 0;',
    '  var vC = 0;',
    '  function Nesty(pA) {',
    '    var Result$1 = 0;',
    '    var vB = 0;',
    '    Result$1 = pA + vB + vC + pD + $mod.vInUnit;',
    '    Result$1 = 3;',
    '    Result = 4;',
    '    return Result$1;',
    '    return Result$1;',
    '  };',
    '  Result = pA + vB + vC;',
    '  Result = 6;',
    '  return Result;',
    '  return Result;',
    '};'
    ]),
    '');
end;

procedure TTestModule.TestNestedProc_ResultString;
begin
  StartProgram(false);
  Add([
  'function DoIt: string;',
  '  function Nesty: string; ',
  '  begin',
  '    nesty:=#65#66;',
  '    nesty[1]:=#67;',
  '    doit:=#68;',
  '    doit[2]:=#69;',
  '  end;',
  'begin',
  '  doit:=#70;',
  '  doit[3]:=#71;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestNestedProc_ResultString',
    LinesToStr([ // statements
    'this.DoIt = function () {',
    '  var Result = "";',
    '  function Nesty() {',
    '    var Result$1 = "";',
    '    Result$1 = "AB";',
    '    Result$1 = rtl.setCharAt(Result$1, 0, "C");',
    '    Result = "D";',
    '    Result = rtl.setCharAt(Result, 1, "E");',
    '    return Result$1;',
    '  };',
    '  Result = "F";',
    '  Result = rtl.setCharAt(Result, 2, "G");',
    '  return Result;',
    '};'
    ]),
    '');
end;

procedure TTestModule.TestForwardProc;
begin
  StartProgram(false);
  Add('procedure FuncA(Bar: longint); forward;');
  Add('procedure FuncB(Bar: longint);');
  Add('begin');
  Add('  funca(bar);');
  Add('end;');
  Add('procedure funca(bar: longint);');
  Add('begin');
  Add('  if bar=3 then ;');
  Add('end;');
  Add('begin');
  Add('  funca(4);');
  Add('  funcb(5);');
  ConvertProgram;
  CheckSource('TestForwardProc',
    LinesToStr([ // statements'
    'this.FuncB = function (Bar) {',
    '  $mod.FuncA(Bar);',
    '};',
    'this.FuncA = function (Bar) {',
    '  if (Bar === 3);',
    '};'
    ]),
    LinesToStr([
    '$mod.FuncA(4);',
    '$mod.FuncB(5);'
    ])
    );
end;

procedure TTestModule.TestNestedForwardProc;
begin
  StartProgram(false);
  Add('procedure FuncA;');
  Add('  procedure FuncB(i: longint); forward;');
  Add('  procedure FuncC(i: longint);');
  Add('  begin');
  Add('    funcb(i);');
  Add('  end;');
  Add('  procedure FuncB(i: longint);');
  Add('  begin');
  Add('    if i=3 then ;');
  Add('  end;');
  Add('begin');
  Add('  funcc(4)');
  Add('end;');
  Add('begin');
  Add('  funca;');
  ConvertProgram;
  CheckSource('TestNestedForwardProc',
    LinesToStr([ // statements'
    'this.FuncA = function () {',
    '  function FuncC(i) {',
    '    FuncB(i);',
    '  };',
    '  function FuncB(i) {',
    '    if (i === 3);',
    '  };',
    '  FuncC(4);',
    '};'
    ]),
    LinesToStr([
    '$mod.FuncA();'
    ])
    );
end;

procedure TTestModule.TestAssignFunctionResult;
begin
  StartProgram(false);
  Add('function Func1: longint;');
  Add('begin');
  Add('end;');
  Add('var i: longint;');
  Add('begin');
  Add('  i:=func1();');
  Add('  i:=func1()+func1();');
  ConvertProgram;
  CheckSource('TestAssignFunctionResult',
    LinesToStr([ // statements
     'this.Func1 = function () {',
     '  var Result = 0;',
     '  return Result;',
     '};',
     'this.i = 0;'
    ]),
    LinesToStr([
    '$mod.i = $mod.Func1();',
    '$mod.i = $mod.Func1() + $mod.Func1();'
    ]));
end;

procedure TTestModule.TestFunctionResultInCondition;
begin
  StartProgram(false);
  Add('function Func1: longint;');
  Add('begin');
  Add('end;');
  Add('function Func2: boolean;');
  Add('begin');
  Add('end;');
  Add('var i: longint;');
  Add('begin');
  Add('  if func2 then ;');
  Add('  if i=func1() then ;');
  Add('  if i=func1 then ;');
  ConvertProgram;
  CheckSource('TestFunctionResultInCondition',
    LinesToStr([ // statements
     'this.Func1 = function () {',
     '  var Result = 0;',
     '  return Result;',
     '};',
     'this.Func2 = function () {',
     '  var Result = false;',
     '  return Result;',
     '};',
     'this.i = 0;'
    ]),
    LinesToStr([
    'if ($mod.Func2());',
    'if ($mod.i === $mod.Func1());',
    'if ($mod.i === $mod.Func1());'
    ]));
end;

procedure TTestModule.TestFunctionResultInForLoop;
begin
  StartProgram(false);
  Add([
  'function Func1(a: array of longint): longint;',
  'begin',
  '  for Result:=High(a) downto Low(a) do if a[Result]=0 then exit;',
  '  for Result in a do if a[Result]=0 then exit;',
  'end;',
  'begin',
  '  Func1([1,2,3])']);
  ConvertProgram;
  CheckSource('TestFunctionResultInForLoop',
    LinesToStr([ // statements
    'this.Func1 = function (a) {',
    '  var Result = 0;',
    '  for (var $l = rtl.length(a) - 1; $l >= 0; $l--) {',
    '    Result = $l;',
    '    if (a[Result] === 0) return Result;',
    '  };',
    '  for (var $in = a, $l1 = 0, $end = rtl.length($in) - 1; $l1 <= $end; $l1++) {',
    '    Result = $in[$l1];',
    '    if (a[Result] === 0) return Result;',
    '  };',
    '  return Result;',
    '};',
    '']),
    LinesToStr([
    '$mod.Func1([1, 2, 3]);'
    ]));
end;

procedure TTestModule.TestFunctionResultInTypeCast;
begin
  StartProgram(false);
  Add([
  'function GetInt: longint;',
  'begin',
  'end;',
  'begin',
  '  if Byte(GetInt)=0 then ;',
  '']);
  ConvertProgram;
  CheckSource('TestFunctionResultInTypeCast',
    LinesToStr([ // statements
    'this.GetInt = function () {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    '']),
    LinesToStr([
    'if (($mod.GetInt() & 255) === 0) ;'
    ]));
end;

procedure TTestModule.TestExit;
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
  Add('  exit(1);');
  ConvertProgram;
  CheckSource('TestExit',
    LinesToStr([ // statements
    'this.ProcA = function () {',
    '  return;',
    '};',
    'this.FuncB = function () {',
    '  var Result = 0;',
    '  return Result;',
    '  return 3;',
    '  return Result;',
    '};',
    'this.FuncC = function () {',
    '  var Result = "";',
    '  return Result;',
    '  return "a";',
    '  return "abc";',
    '  return Result;',
    '};'
    ]),
    LinesToStr([
    'return;',
    'return 1;',
    '']));
end;

procedure TTestModule.TestExit_ResultInFinally;
begin
  StartProgram(false);
  Add([
  'function Run: word;',
  'begin',
  '  try',
  '    exit(3);', // no Result in finally -> use return 3
  '  finally',
  '  end;',
  'end;',
  'function Fly: word;',
  'begin',
  '  try',
  '    exit(3);',
  '  finally',
  '    if Result>0 then ;',
  '  end;',
  'end;',
  'function Jump: word;',
  'begin',
  '  try',
  '    try',
  '      exit(4);',
  '    finally',
  '    end;',
  '  finally',
  '    if Result>0 then ;',
  '  end;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestExit_ResultInFinally',
    LinesToStr([ // statements
    'this.Run = function () {',
    '  var Result = 0;',
    '  try {',
    '    return 3;',
    '  } finally {',
    '  };',
    '  return Result;',
    '};',
    'this.Fly = function () {',
    '  var Result = 0;',
    '  try {',
    '    Result = 3;',
    '    return Result;',
    '  } finally {',
    '    if (Result > 0) ;',
    '  };',
    '  return Result;',
    '};',
    'this.Jump = function () {',
    '  var Result = 0;',
    '  try {',
    '    try {',
    '      Result = 4;',
    '      return Result;',
    '    } finally {',
    '    };',
    '  } finally {',
    '    if (Result > 0) ;',
    '  };',
    '  return Result;',
    '};',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestBreak;
begin
  StartProgram(false);
  Add([
  'var',
  '  i: longint;',
  'begin',
  '  repeat',
  '    break;',
  '  until true;',
  '  while true do',
  '    break;',
  '  for i:=1 to 2 do',
  '    break;']);
  ConvertProgram;
  CheckSource('TestBreak',
    LinesToStr([ // statements
    'this.i = 0;'
    ]),
    LinesToStr([
    'do {',
    '  break;',
    '} while (!true);',
    'while (true) break;',
    'for ($mod.i = 1; $mod.i <= 2; $mod.i++) break;',
    '']));
end;

procedure TTestModule.TestBreakAsVar;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(break: boolean);',
  'begin',
  '  if break then ;',
  'end;',
  'var',
  '  break: boolean;',
  'begin',
  '  if break then ;']);
  ConvertProgram;
  CheckSource('TestBreakAsVar',
    LinesToStr([ // statements
    'this.DoIt = function (Break) {',
    '  if (Break) ;',
    '};',
    'this.Break = false;',
    '']),
    LinesToStr([
    'if($mod.Break) ;',
    '']));
end;

procedure TTestModule.TestContinue;
begin
  StartProgram(false);
  Add('var i: longint;');
  Add('begin');
  Add('  repeat');
  Add('    continue;');
  Add('  until true;');
  Add('  while true do');
  Add('    continue;');
  Add('  for i:=1 to 2 do');
  Add('    continue;');
  ConvertProgram;
  CheckSource('TestContinue',
    LinesToStr([ // statements
    'this.i = 0;'
    ]),
    LinesToStr([
    'do {',
    '  continue;',
    '} while (!true);',
    'while (true) continue;',
    'for ($mod.i = 1; $mod.i <= 2; $mod.i++) continue;',
    '']));
end;

procedure TTestModule.TestProc_External;
begin
  StartProgram(false);
  Add('procedure Foo; external name ''console.log'';');
  Add('function Bar: longint; external name ''get.item'';');
  Add('function Bla(s: string): longint; external name ''apply.something'';');
  Add('var');
  Add('  i: longint;');
  Add('begin');
  Add('  Foo;');
  Add('  i:=Bar;');
  Add('  i:=Bla(''abc'');');
  ConvertProgram;
  CheckSource('TestProc_External',
    LinesToStr([ // statements
    'this.i = 0;'
    ]),
    LinesToStr([
    'console.log();',
    '$mod.i = get.item();',
    '$mod.i = apply.something("abc");'
    ]));
end;

procedure TTestModule.TestProc_ExternalOtherUnit;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'procedure Now; external name ''Date.now'';',
    'procedure DoIt;'
    ]),
    'procedure doit; begin end;');

  StartUnit(true);
  Add('interface');
  Add('uses unit2;');
  Add('implementation');
  Add('begin');
  Add('  now;');
  Add('  now();');
  Add('  uNit2.now;');
  Add('  uNit2.now();');
  Add('  doit;');
  Add('  uNit2.doit;');
  ConvertUnit;
  CheckSource('TestProc_ExternalOtherUnit',
    LinesToStr([
    '']),
    LinesToStr([
    'Date.now();',
    'Date.now();',
    'Date.now();',
    'Date.now();',
    'pas.unit2.DoIt();',
    'pas.unit2.DoIt();',
    '']));
end;

procedure TTestModule.TestProc_Asm;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'function DoIt: longint;',
  'begin;',
  '  asm',
  '  { a:{ b:{}, c:[]}, d:''1'' };',
  '  end;',
  '  asm console.log(); end;',
  '  asm',
  '    s = "'' ";',
  '    s = ''" '';',
  '    s = s + "world" + "''";',
  '    // end',
  '    s = ''end'';',
  '    s = "end";',
  '    s = "foo\"bar";',
  '    s = ''a\''b'';',
  '    s =  `${expr}\`-"-''-`;',
  '    s = `multi',
  'line`;',
  '  end;',
  'end;',
  'procedure Fly;',
  'asm',
  '  return;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestProc_Asm',
    LinesToStr([ // statements
    'this.DoIt = function () {',
    '  var Result = 0;',
    '  { a:{ b:{}, c:[]}, d:''1'' };',
    '  console.log();',
    '  s = "'' ";',
    '  s = ''" '';',
    '  s = s + "world" + "''";',
    '  // end',
    '  s = ''end'';',
    '  s = "end";',
    '  s = "foo\"bar";',
    '  s = ''a\''b'';',
    '  s =  `${expr}\`-"-''-`;',
    '  s = `multi',
    'line`;',
    '  return Result;',
    '};',
    'this.Fly = function () {',
    '  return;',
    '};',
    '']),
    LinesToStr([
    ''
    ]));
end;

procedure TTestModule.TestProc_AsmSubBlock;
begin
  StartProgram(true,[supTObject]);
  Add([
  '{$mode delphi}',
  'type',
  '  TBird = class end;',
  'procedure Run(w: word);',
  'begin;',
  '  if true then asm console.log(); end;',
  '  if w>3 then asm',
  '    var a = w+1;',
  '    w = a+3;',
  '  end;',
  '  while (w>7) do asm',
  '    w+=3; w*=2;',
  '  end;',
  '  try',
  '  except',
  '    on E: TBird do',
  '      asm console.log(E); end;',
  '    on E: TObject do',
  '      asm var i=3; i--; end;',
  '    else asm Fly; High; end;',
  '  end;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestProc_AsmSubBlock',
    LinesToStr([ // statements
    'rtl.createClass(this, "TBird", pas.system.TObject, function () {',
    '});',
    'this.Run = function (w) {',
    '  if (true) console.log();',
    '  if (w > 3) {',
    '    var a = w+1;',
    '    w = a+3;',
    '  };',
    '  while (w > 7) {',
    '    w+=3; w*=2;',
    '  };',
    '  try {} catch ($e) {',
    '    if ($mod.TBird.isPrototypeOf($e)) {',
    '      var E = $e;',
    '      console.log(E);',
    '    } else if (pas.system.TObject.isPrototypeOf($e)) {',
    '      var E = $e;',
    '      var i=3; i--;',
    '    } else {',
    '      Fly; High;',
    '    }',
    '  };',
    '};',
    '']),
    LinesToStr([
    ''
    ]));
end;

procedure TTestModule.TestProc_Assembler;
begin
  StartProgram(false);
  Add('function DoIt: longint; assembler;');
  Add('asm');
  Add('{ a:{ b:{}, c:[]}, d:''1'' };');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestProc_Assembler',
    LinesToStr([ // statements
    'this.DoIt = function () {',
    '  { a:{ b:{}, c:[]}, d:''1'' };',
    '};'
    ]),
    LinesToStr([
    ''
    ]));
end;

procedure TTestModule.TestProc_VarParam;
begin
  StartProgram(false);
  Add('type integer = longint;');
  Add('procedure DoIt(vG: integer; const vH: integer; var vI: integer);');
  Add('var vJ: integer;');
  Add('begin');
  Add('  vg:=vg+1;');
  Add('  vj:=vh+2;');
  Add('  vi:=vi+3;');
  Add('  doit(vg,vg,vg);');
  Add('  doit(vh,vh,vj);');
  Add('  doit(vi,vi,vi);');
  Add('  doit(vj,vj,vj);');
  Add('end;');
  Add('var i: integer;');
  Add('begin');
  Add('  doit(i,i,i);');
  ConvertProgram;
  CheckSource('TestProc_VarParam',
    LinesToStr([ // statements
    'this.DoIt = function (vG,vH,vI) {',
    '  var vJ = 0;',
    '  vG = vG + 1;',
    '  vJ = vH + 2;',
    '  vI.set(vI.get()+3);',
    '  $mod.DoIt(vG, vG, {',
    '    get: function () {',
    '      return vG;',
    '    },',
    '    set: function (v) {',
    '      vG = v;',
    '    }',
    '  });',
    '  $mod.DoIt(vH, vH, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '  $mod.DoIt(vI.get(), vI.get(), vI);',
    '  $mod.DoIt(vJ, vJ, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '};',
    'this.i = 0;'
    ]),
    LinesToStr([
    '$mod.DoIt($mod.i,$mod.i,{',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '});'
    ]));
end;

procedure TTestModule.TestProc_VarParamString;
begin
  StartProgram(false);
  Add(['type TCaption = string;',
  'procedure DoIt(vA: TCaption; var vB: TCaption; out vC: TCaption);',
  'var c: char;',
  'begin',
  '  va[1]:=c;',
  '  vb[2]:=c;',
  '  vc[3]:=c;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestProc_VarParamString',
    LinesToStr([ // statements
    'this.DoIt = function (vA,vB,vC) {',
    '  var c = "";',
    '  vA = rtl.setCharAt(vA, 0, c);',
    '  vB.set(rtl.setCharAt(vB.get(), 1, c));',
    '  vC.set(rtl.setCharAt(vC.get(), 2, c));',
    '};',
    '']),
    LinesToStr([
    ]));
end;

procedure TTestModule.TestProc_VarParamV;
begin
  StartProgram(false);
  Add([
  'procedure Inc2(var i: longint);',
  'begin',
  '  i:=i+2;',
  'end;',
  'procedure DoIt(v: longint);',
  'var p: array of longint;',
  'begin',
  '  Inc2(v);',
  '  Inc2(p[v]);',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestProc_VarParamV',
    LinesToStr([ // statements
    'this.Inc2 = function (i) {',
    '  i.set(i.get()+2);',
    '};',
    'this.DoIt = function (v) {',
    '  var p = [];',
    '  $mod.Inc2({get: function () {',
    '    return v;',
    '  }, set: function (w) {',
    '    v = w;',
    '  }});',
    '  $mod.Inc2({',
    '    a: v,',
    '    p: p,',
    '    get: function () {',
    '        return this.p[this.a];',
    '      },',
    '    set: function (v) {',
    '        this.p[this.a] = v;',
    '      }',
    '  });',
    '};',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestProc_Overload;
begin
  StartProgram(false);
  Add('procedure DoIt(vI: longint); begin end;');
  Add('procedure DoIt(vI, vJ: longint); begin end;');
  Add('procedure DoIt(vD: double); begin end;');
  Add('begin');
  Add('  DoIt(1);');
  Add('  DoIt(2,3);');
  Add('  DoIt(4.5);');
  ConvertProgram;
  CheckSource('TestProcedureOverload',
    LinesToStr([ // statements
    'this.DoIt = function (vI) {',
    '};',
    'this.DoIt$1 = function (vI, vJ) {',
    '};',
    'this.DoIt$2 = function (vD) {',
    '};',
    '']),
    LinesToStr([
    '$mod.DoIt(1);',
    '$mod.DoIt$1(2, 3);',
    '$mod.DoIt$2(4.5);',
    '']));
end;

procedure TTestModule.TestProc_OverloadForward;
begin
  StartProgram(false);
  Add('procedure DoIt(vI: longint); forward;');
  Add('procedure DoIt(vI, vJ: longint); begin end;');
  Add('procedure doit(vi: longint); begin end;');
  Add('begin');
  Add('  doit(1);');
  Add('  doit(2,3);');
  ConvertProgram;
  CheckSource('TestProcedureOverloadForward',
    LinesToStr([ // statements
    'this.DoIt$1 = function (vI, vJ) {',
    '};',
    'this.DoIt = function (vI) {',
    '};',
    '']),
    LinesToStr([
    '$mod.DoIt(1);',
    '$mod.DoIt$1(2, 3);',
    '']));
end;

procedure TTestModule.TestProc_OverloadIntfImpl;
begin
  StartUnit(false);
  Add('interface');
  Add('procedure DoIt(vI: longint);');
  Add('procedure DoIt(vI, vJ: longint);');
  Add('implementation');
  Add('procedure DoIt(vI, vJ, vK, vL, vM: longint); forward;');
  Add('procedure DoIt(vI, vJ, vK: longint); begin end;');
  Add('procedure DoIt(vi: longint); begin end;');
  Add('procedure DoIt(vI, vJ, vK, vL: longint); begin end;');
  Add('procedure DoIt(vi, vj: longint); begin end;');
  Add('procedure DoIt(vi, vj, vk, vl, vm: longint); begin end;');
  Add('begin');
  Add('  doit(1);');
  Add('  doit(2,3);');
  Add('  doit(4,5,6);');
  Add('  doit(7,8,9,10);');
  Add('  doit(11,12,13,14,15);');
  ConvertUnit;
  CheckSource('TestProcedureOverloadUnit',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    'this.DoIt = function (vI) {',
    '};',
    'this.DoIt$1 = function (vI, vJ) {',
    '};',
    '']),
    LinesToStr([ // this.$init
    '$mod.DoIt(1);',
    '$mod.DoIt$1(2, 3);',
    '$impl.DoIt$3(4,5,6);',
    '$impl.DoIt$4(7,8,9,10);',
    '$impl.DoIt$2(11,12,13,14,15);',
    '']),
    LinesToStr([ // implementation
    '$impl.DoIt$3 = function (vI, vJ, vK) {',
    '};',
    '$impl.DoIt$4 = function (vI, vJ, vK, vL) {',
    '};',
    '$impl.DoIt$2 = function (vI, vJ, vK, vL, vM) {',
    '};',
    '']));
end;

procedure TTestModule.TestProc_OverloadNested;
begin
  StartProgram(false);
  Add([
  'procedure doit(vA: longint);',
  '  procedure DoIt(vA, vB: longint); overload;',
  '  begin',
  '    doit(1);',
  '    doit(1,2);',
  '  end;',
  '  procedure doit(vA, vB, vC: longint);',
  '  begin',
  '    doit(1);',
  '    doit(1,2);',
  '    doit(1,2,3);',
  '  end;',
  'begin',
  '  doit(1);',
  '  doit(1,2);',
  '  doit(1,2,3);',
  'end;',
  'begin // main',
  '  doit(1);']);
  ConvertProgram;
  CheckSource('TestProcedureOverloadNested',
    LinesToStr([ // statements
    'this.doit = function (vA) {',
    '  function DoIt$1(vA, vB) {',
    '    $mod.doit(1);',
    '    DoIt$1(1, 2);',
    '  };',
    '  function doit$2(vA, vB, vC) {',
    '    $mod.doit(1);',
    '    DoIt$1(1, 2);',
    '    doit$2(1, 2, 3);',
    '  };',
    '  $mod.doit(1);',
    '  DoIt$1(1, 2);',
    '  doit$2(1, 2, 3);',
    '};',
    '']),
    LinesToStr([
    '$mod.doit(1);',
    '']));
end;

procedure TTestModule.TestProc_OverloadNestedForward;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(vA: longint); overload; forward;',
  'procedure DoIt(vB, vC: longint); overload;',
  'begin // 2 param overload',
  '  doit(1);',
  '  doit(1,2);',
  'end;',
  'procedure doit(vA: longint);',
  '  procedure DoIt(vA, vB, vC: longint); overload; forward;',
  '  procedure DoIt(vA, vB, vC, vD: longint); overload;',
  '  begin // 4 param overload',
  '    doit(1);',
  '    doit(1,2);',
  '    doit(1,2,3);',
  '    doit(1,2,3,4);',
  '  end;',
  '  procedure doit(vA, vB, vC: longint);',
  '    procedure DoIt(vA, vB, vC, vD, vE: longint); overload; forward;',
  '    procedure DoIt(vA, vB, vC, vD, vE, vF: longint); overload;',
  '    begin // 6 param overload',
  '      doit(1);',
  '      doit(1,2);',
  '      doit(1,2,3);',
  '      doit(1,2,3,4);',
  '      doit(1,2,3,4,5);',
  '      doit(1,2,3,4,5,6);',
  '    end;',
  '    procedure doit(vA, vB, vC, vD, vE: longint);',
  '    begin // 5 param overload',
  '      doit(1);',
  '      doit(1,2);',
  '      doit(1,2,3);',
  '      doit(1,2,3,4);',
  '      doit(1,2,3,4,5);',
  '      doit(1,2,3,4,5,6);',
  '    end;',
  '  begin // 3 param overload',
  '    doit(1);',
  '    doit(1,2);',
  '    doit(1,2,3);',
  '    doit(1,2,3,4);',
  '    doit(1,2,3,4,5);',
  '    doit(1,2,3,4,5,6);',
  '  end;',
  'begin // 1 param overload',
  '  doit(1);',
  '  doit(1,2);',
  '  doit(1,2,3);',
  '  doit(1,2,3,4);',
  'end;',
  'begin // main',
  '  doit(1);',
  '  doit(1,2);']);
  ConvertProgram;
  CheckSource('TestProc_OverloadNestedForward',
    LinesToStr([ // statements
    'this.DoIt$1 = function (vB, vC) {',
    '  $mod.DoIt(1);',
    '  $mod.DoIt$1(1, 2);',
    '};',
    'this.DoIt = function (vA) {',
    '  function DoIt$3(vA, vB, vC, vD) {',
    '    $mod.DoIt(1);',
    '    $mod.DoIt$1(1, 2);',
    '    DoIt$2(1, 2, 3);',
    '    DoIt$3(1, 2, 3, 4);',
    '  };',
    '  function DoIt$2(vA, vB, vC) {',
    '    function DoIt$5(vA, vB, vC, vD, vE, vF) {',
    '      $mod.DoIt(1);',
    '      $mod.DoIt$1(1, 2);',
    '      DoIt$2(1, 2, 3);',
    '      DoIt$3(1, 2, 3, 4);',
    '      DoIt$4(1, 2, 3, 4, 5);',
    '      DoIt$5(1, 2, 3, 4, 5, 6);',
    '    };',
    '    function DoIt$4(vA, vB, vC, vD, vE) {',
    '      $mod.DoIt(1);',
    '      $mod.DoIt$1(1, 2);',
    '      DoIt$2(1, 2, 3);',
    '      DoIt$3(1, 2, 3, 4);',
    '      DoIt$4(1, 2, 3, 4, 5);',
    '      DoIt$5(1, 2, 3, 4, 5, 6);',
    '    };',
    '    $mod.DoIt(1);',
    '    $mod.DoIt$1(1, 2);',
    '    DoIt$2(1, 2, 3);',
    '    DoIt$3(1, 2, 3, 4);',
    '    DoIt$4(1, 2, 3, 4, 5);',
    '    DoIt$5(1, 2, 3, 4, 5, 6);',
    '  };',
    '  $mod.DoIt(1);',
    '  $mod.DoIt$1(1, 2);',
    '  DoIt$2(1, 2, 3);',
    '  DoIt$3(1, 2, 3, 4);',
    '};',
    '']),
    LinesToStr([
    '$mod.DoIt(1);',
    '$mod.DoIt$1(1, 2);',
    '']));
end;

procedure TTestModule.TestProc_OverloadUnitCycle;
begin
  AddModuleWithIntfImplSrc('Unit2.pas',
    LinesToStr([
    'type',
    '  TObject = class',
    '    procedure DoIt(b: boolean); virtual; abstract;',
    '    procedure DoIt(i: longint); virtual; abstract;',
    '  end;',
    '']),
    'uses test1;');
  StartUnit(true);
  Add([
  'interface',
  'uses unit2;',
  'type',
  '  TEagle = class(TObject)',
  '    procedure DoIt(b: boolean); override;',
  '    procedure DoIt(i: longint); override;',
  '  end;',
  'implementation',
  'procedure TEagle.DoIt(b: boolean); begin end;',
  'procedure TEagle.DoIt(i: longint); begin end;',
  '']);
  ConvertUnit;
  CheckSource('TestProc_OverloadUnitCycle',
    LinesToStr([ // statements
    'rtl.createClass(this, "TEagle", pas.Unit2.TObject, function () {',
    '  this.DoIt = function (b) {',
    '  };',
    '  this.DoIt$1 = function (i) {',
    '  };',
    '});',
    '']),
    '',
    LinesToStr([
    '']));
end;

procedure TTestModule.TestProc_Varargs;
begin
  StartProgram(false);
  Add([
  'procedure ProcA(i:longint); varargs; external name ''ProcA'';',
  'procedure ProcB; varargs; external name ''ProcB'';',
  'procedure ProcC(i: longint = 17); varargs; external name ''ProcC'';',
  'function GetIt: longint; begin end;',
  'begin',
  '  ProcA(1);',
  '  ProcA(1,2);',
  '  ProcA(1,2.0);',
  '  ProcA(1,2,3);',
  '  ProcA(1,''2'');',
  '  ProcA(2,'''');',
  '  ProcA(3,false);',
  '  ProcB;',
  '  ProcB();',
  '  ProcB(4);',
  '  ProcB(''foo'');',
  '  ProcC;',
  '  ProcC();',
  '  ProcC(4);',
  '  ProcC(5,''foo'');',
  '  ProcB(GetIt);',
  '  ProcB(GetIt());',
  '  ProcB(GetIt,GetIt());']);
  ConvertProgram;
  CheckSource('TestProc_Varargs',
    LinesToStr([ // statements
    'this.GetIt = function () {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    '']),
    LinesToStr([
    'ProcA(1);',
    'ProcA(1, 2);',
    'ProcA(1, 2.0);',
    'ProcA(1, 2, 3);',
    'ProcA(1, "2");',
    'ProcA(2, "");',
    'ProcA(3, false);',
    'ProcB();',
    'ProcB();',
    'ProcB(4);',
    'ProcB("foo");',
    'ProcC(17);',
    'ProcC(17);',
    'ProcC(4);',
    'ProcC(5, "foo");',
    'ProcB($mod.GetIt());',
    'ProcB($mod.GetIt());',
    'ProcB($mod.GetIt(), $mod.GetIt());',
    '']));
end;

procedure TTestModule.TestProc_ConstOrder;
begin
  StartProgram(false);
  Add([
  'const A = 3;',
  'const B = A+1;',
  'procedure DoIt;',
  'const C = A+1;',
  'const D = B+1;',
  'const E = D+C+B+A;',
  'begin',
  'end;',
  'begin'
  ]);
  ConvertProgram;
  CheckSource('TestProc_ConstOrder',
    LinesToStr([ // statements
    'this.A = 3;',
    'this.B = 3 + 1;',
    'var C = 3 + 1;',
    'var D = 4 + 1;',
    'var E = 5 + 4 + 4 + 3;',
    'this.DoIt = function () {',
    '};',
    '']),
    LinesToStr([
    ''
    ]));
end;

procedure TTestModule.TestProc_DuplicateConst;
begin
  StartProgram(false);
  Add([
  'const A = 1;',
  'procedure DoIt;',
  'const A = 2;',
  '  procedure SubIt;',
  '  const A = 21;',
  '  begin',
  '  end;',
  'begin',
  'end;',
  'procedure DoSome;',
  'const A = 3;',
  'begin',
  'end;',
  'begin'
  ]);
  ConvertProgram;
  CheckSource('TestProc_DuplicateConst',
    LinesToStr([ // statements
    'this.A = 1;',
    'var A$1 = 2;',
    'var A$2 = 21;',
    'this.DoIt = function () {',
    '  function SubIt() {',
    '  };',
    '};',
    'var A$3 = 3;',
    'this.DoSome = function () {',
    '};',
    '']),
    LinesToStr([
    ''
    ]));
end;

procedure TTestModule.TestProc_LocalVarAbsolute;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    Index: longint;',
  '    procedure DoAbs(Item: pointer);',
  '  end;',
  'procedure TObject.DoAbs(Item: pointer);',
  'var',
  '  o: TObject absolute Item;',
  'begin',
  '  if o.Index<o.Index then o.Index:=o.Index;',
  'end;',
  'procedure DoIt(i: longint; p: pointer);',
  'var',
  '  d: double absolute i;',
  '  s: string absolute d;',
  '  oi: TObject absolute i;',
  '  op: TObject absolute p;',
  'begin',
  '  if d=d then d:=d;',
  '  if s=s then s:=s;',
  '  if oi.Index<oi.Index then oi.Index:=oi.Index;',
  '  if op.Index=op.Index then op.Index:=op.Index;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestProc_LocalVarAbsolute',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.Index = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoAbs = function (Item) {',
    '    if (Item.Index < Item.Index) Item.Index = Item.Index;',
    '  };',
    '});',
    'this.DoIt = function (i, p) {',
    '  if (i === i) i = i;',
    '  if (i === i) i = i;',
    '  if (i.Index < i.Index) i.Index = i.Index;',
    '  if (p.Index === p.Index) p.Index = p.Index;',
    '};'
    ]),
    LinesToStr([
    ]));
end;

procedure TTestModule.TestProc_LocalVarInit;
begin
  StartProgram(false);
  Add([
  'type TBytes = array of byte;',
  'procedure DoIt;',
  'const c = 4;',
  'var',
  '  b: byte = 1;',
  '  w: word = 2+c;',
  '  p: pointer = nil;',
  '  Buffer: TBytes = nil;',
  'begin',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestProc_LocalVarInit',
    LinesToStr([ // statements
    'var c = 4;',
    'this.DoIt = function () {',
    '  var b = 1;',
    '  var w = 2 + 4;',
    '  var p = null;',
    '  var Buffer = [];',
    '};',
    '']),
    LinesToStr([
    ]));
end;

procedure TTestModule.TestProc_ReservedWords;
begin
  StartProgram(false);
  Add([
  'procedure Date(ArrayBuffer: longint);',
  'const',
  '  NaN: longint = 3;',
  'var',
  '  &Boolean: longint;',
  '  procedure Error(ArrayBuffer: longint);',
  '  begin',
  '  end;',
  'begin',
  '  Nan:=&bOolean;',
  'end;',
  'begin',
  '  Date(1);']);
  ConvertProgram;
  CheckSource('TestProc_ReservedWords',
    LinesToStr([ // statements
    'var naN = 3;',
    'this.Date = function (arrayBuffer) {',
    '  var boolean = 0;',
    '  function error(arrayBuffer) {',
    '  };',
    '  naN = boolean;',
    '};',
    '']),
    LinesToStr([
    '  $mod.Date(1);'
    ]));
end;

procedure TTestModule.TestProc_ConstRefWord;
begin
  StartProgram(false);
  Add([
  'procedure Run(constref w: word);',
  'var l: word;',
  'begin',
  '  l:=w;',
  '  Run(w);',
  '  Run(l);',
  'end;',
  'procedure Fly(a: word; var b: word; out c: word; const d: word; constref e: word);',
  'begin',
  '  Run(a);',
  '  Run(b);',
  '  Run(c);',
  '  Run(d);',
  '  Run(e);',
  'end;',
  'begin',
  '  Run(1);']);
  ConvertProgram;
  CheckHint(mtWarning,nConstRefNotForXAsConst,'ConstRef not yet implemented for Word. Treating as Const');
  CheckSource('TestProc_ConstRefWord',
    LinesToStr([ // statements
    'this.Run = function (w) {',
    '  var l = 0;',
    '  l = w;',
    '  $mod.Run(w);',
    '  $mod.Run(l);',
    '};',
    'this.Fly = function (a, b, c, d, e) {',
    '  $mod.Run(a);',
    '  $mod.Run(b.get());',
    '  $mod.Run(c.get());',
    '  $mod.Run(d);',
    '  $mod.Run(e);',
    '};',
    '']),
    LinesToStr([
    '$mod.Run(1);'
    ]));
end;

procedure TTestModule.TestAnonymousProc_Assign_ObjFPC;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
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
  ConvertProgram;
  CheckSource('TestAnonymousProc_Assign_ObjFPC',
    LinesToStr([ // statements
    'this.Func = null;',
    'this.DoIt = function (a) {',
    '  $mod.Func = function (b) {',
    '    var Result = 0;',
    '    Result = a + b;',
    '    return b;',
    '    return Result;',
    '    return Result;',
    '  };',
    '  a = 3;',
    '};',
    '']),
    LinesToStr([
    '$mod.Func = function (c) {',
    '  var Result = 0;',
    '  Result = 3 + c;',
    '  return c;',
    '  return Result;',
    '  return Result;',
    '};',
    '']));
end;

procedure TTestModule.TestAnonymousProc_Assign_Delphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TProc = reference to procedure(x: word);',
  'procedure DoIt(a: word);',
  'var Proc: TProc;',
  'begin',
  '  Proc:=procedure(b:word) begin end;',
  'end;',
  'var Proc: TProc;',
  'begin',
  '  Proc:=procedure(c:word) begin end;',
  '']);
  ConvertProgram;
  CheckSource('TestAnonymousProc_Assign_Delphi',
    LinesToStr([ // statements
    'this.DoIt = function (a) {',
    '  var Proc = null;',
    '  Proc = function (b) {',
    '  };',
    '};',
    'this.Proc = null;',
    '']),
    LinesToStr([
    '$mod.Proc = function (c) {',
    '};',
    '']));
end;

procedure TTestModule.TestAnonymousProc_Arg;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  '  TFunc = reference to function(x: word): word;',
  'procedure DoMore(f,g: TProc);',
  'begin',
  'end;',
  'procedure DoOdd(v: jsvalue);',
  'begin',
  'end;',
  'procedure DoIt(f: TFunc);',
  'begin',
  '  DoIt(function(b:word): word',
  '    begin',
  '      Result:=1+b;',
  '    end);',
  '  DoMore(procedure begin end, procedure begin end);',
  '  DoOdd(procedure begin end);',
  'end;',
  'begin',
  '  DoMore(procedure begin end,',
  '    procedure assembler asm',
  '      console.log("c");',
  '    end);',
  '']);
  ConvertProgram;
  CheckSource('TestAnonymousProc_Arg',
    LinesToStr([ // statements
    'this.DoMore = function (f, g) {',
    '};',
    'this.DoOdd = function (v) {',
    '};',
    'this.DoIt = function (f) {',
    '  $mod.DoIt(function (b) {',
    '    var Result = 0;',
    '    Result = 1 + b;',
    '    return Result;',
    '  });',
    '  $mod.DoMore(function () {',
    '  }, function () {',
    '  });',
    '  $mod.DoOdd(function () {',
    '  });',
    '};',
    '']),
    LinesToStr([
    '$mod.DoMore(function () {',
    '}, function () {',
    '  console.log("c");',
    '});',
    '']));
end;

procedure TTestModule.TestAnonymousProc_Typecast;
begin
  StartProgram(false);
  Add([
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
  ConvertProgram;
  CheckSource('TestAnonymousProc_Typecast',
    LinesToStr([ // statements
    'this.DoIt = function (p) {',
    '  var w = 0;',
    '  var a = [];',
    '  p = function (b) {',
    '  };',
    '  a = function () {',
    '    var Result = [];',
    '    return Result;',
    '  }();',
    '  w = function () {',
    '    var Result = [];',
    '    return Result;',
    '  }()[3];',
    '};',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestAnonymousProc_With;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure(w: word);',
  '  TObject = class',
  '    b: boolean;',
  '  end;',
  'var',
  '  p: TProc;',
  '  bird: TObject;',
  'begin',
  '  with bird do',
  '    p:=procedure(w: word)',
  '      begin',
  '        b:=w>2;',
  '      end;',
  '']);
  ConvertProgram;
  CheckSource('TestAnonymousProc_With',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.b = false;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.p = null;',
    'this.bird = null;',
    '']),
    LinesToStr([
    'var $with = $mod.bird;',
    '$mod.p = function (w) {',
    '  $with.b = w > 2;',
    '};',
    '']));
end;

procedure TTestModule.TestAnonymousProc_ExceptOn;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  '  TObject = class',
  '    b: boolean;',
  '  end;',
  'procedure DoIt;',
  'var',
  '  p: TProc;',
  'begin',
  '  try',
  '  except',
  '    on E: TObject do',
  '    p:=procedure',
  '      begin',
  '        E.b:=true;',
  '      end;',
  '  end;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestAnonymousProc_ExceptOn',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.b = false;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.DoIt = function () {',
    '  var p = null;',
    '  try {} catch ($e) {',
    '    if ($mod.TObject.isPrototypeOf($e)) {',
    '      var E = $e;',
    '      p = function () {',
    '        E.b = true;',
    '      };',
    '    } else throw $e',
    '  };',
    '};',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestAnonymousProc_Nested;
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
  ConvertProgram;
  CheckSource('TestAnonymousProc_Nested',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.i = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '    var $Self = this;',
    '    var p = null;',
    '    function Sub() {',
    '      p = function () {',
    '        $Self.i = 3;',
    '        $Self.i = 4;',
    '        p = function () {',
    '          function SubSub() {',
    '            $Self.i = 13;',
    '            $Self.i = 14;',
    '          };',
    '          $Self.i = 13;',
    '          $Self.i = 14;',
    '        };',
    '      };',
    '    };',
    '  };',
    '});',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestAnonymousProc_NestedAssignResult;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  'function DoIt: TProc;',
  '  function Sub: TProc;',
  '  begin',
  '    Result:=procedure',
  '      begin',
  '        Sub:=procedure',
  '            procedure SubSub;',
  '            begin',
  '              Result:=nil;',
  '              Sub:=nil;',
  '              DoIt:=nil;',
  '            end;',
  '          begin',
  '            Result:=nil;',
  '            Sub:=nil;',
  '            DoIt:=nil;',
  '          end;',
  '      end;',
  '  end;',
  'begin',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestAnonymousProc_NestedAssignResult',
    LinesToStr([ // statements
    'this.DoIt = function () {',
    '  var Result = null;',
    '  function Sub() {',
    '    var Result$1 = null;',
    '    Result$1 = function () {',
    '      Result$1 = function () {',
    '        function SubSub() {',
    '          Result$1 = null;',
    '          Result$1 = null;',
    '          Result = null;',
    '        };',
    '        Result$1 = null;',
    '        Result$1 = null;',
    '        Result = null;',
    '      };',
    '    };',
    '    return Result$1;',
    '  };',
    '  return Result;',
    '};',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestAnonymousProc_Class;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProc = reference to procedure;',
  '  TEvent = procedure of object;',
  '  TObject = class',
  '    Size: word;',
  '    function GetIt: TProc;',
  '    procedure DoIt; virtual; abstract;',
  '  end;',
  'function TObject.GetIt: TProc;',
  'begin',
  '  Result:=procedure',
  '    var p: TEvent;',
  '    begin',
  '      Size:=Size;',
  '      Size:=Self.Size;',
  '      p:=@DoIt;',
  '      p:=@Self.DoIt;',
  '    end;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestAnonymousProc_Class',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.Size = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetIt = function () {',
    '    var $Self = this;',
    '    var Result = null;',
    '    Result = function () {',
    '      var p = null;',
    '      $Self.Size = $Self.Size;',
    '      $Self.Size = $Self.Size;',
    '      p = rtl.createCallback($Self, "DoIt");',
    '      p = rtl.createCallback($Self, "DoIt");',
    '    };',
    '    return Result;',
    '  };',
    '});',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestAnonymousProc_ForLoop;
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
  ConvertProgram;
  CheckSource('TestAnonymousProc_ForLoop',
    LinesToStr([ // statements
    'this.Foo = function (p) {',
    '};',
    'this.DoIt = function () {',
    '  var i = 0;',
    '  var a = 0;',
    '  for (i = 1; i <= 10; i++) {',
    '    $mod.Foo(function () {',
    '      a = 3;',
    '    });',
    '  };',
    '};',
    '']),
    LinesToStr([
    '$mod.DoIt();'
    ]));
end;

procedure TTestModule.TestAnonymousProc_AsmDelphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TProc = reference to procedure;',
  '  TFunc = reference to function(x: word): word;',
  'procedure Run;',
  'asm',
  'end;',
  'procedure Walk(p: TProc; f: TFunc);',
  'begin',
  '  Walk(procedure asm end, function(b:word): word asm return 1+b; end);',
  'end;',
  'begin',
  '  Walk(procedure',
  '    asm',
  '      console.log("a");',
  '    end,',
  '    function(x: word): word asm',
  '      console.log("c");',
  '    end);',
  '']);
  ConvertProgram;
  CheckSource('TestAnonymousProc_AsmDelphi',
    LinesToStr([ // statements
    'this.Run = function () {',
    '};',
    'this.Walk = function (p, f) {',
    '  $mod.Walk(function () {',
    '  }, function (b) {',
    '    return 1+b;',
    '  });',
    '};',
    '']),
    LinesToStr([
    '$mod.Walk(function () {',
    '  console.log("a");',
    '}, function (x) {',
    '  console.log("c");',
    '});',
    '']));
end;

procedure TTestModule.TestEnum_Name;
begin
  StartProgram(false);
  Add('type TMyEnum = (Red, Green, Blue);');
  Add('var e: TMyEnum;');
  Add('var f: TMyEnum = Blue;');
  Add('begin');
  Add('  e:=green;');
  Add('  e:=default(TMyEnum);');
  ConvertProgram;
  CheckSource('TestEnum_Name',
    LinesToStr([ // statements
    'this.TMyEnum = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1,',
    '  "2":"Blue",',
    '  Blue:2',
    '  };',
    'this.e = 0;',
    'this.f = this.TMyEnum.Blue;'
    ]),
    LinesToStr([
    '$mod.e=$mod.TMyEnum.Green;',
    '$mod.e=$mod.TMyEnum.Red;'
    ]));
end;

procedure TTestModule.TestEnum_Number;
begin
  Converter.Options:=Converter.Options+[coEnumNumbers];
  StartProgram(false);
  Add('type TMyEnum = (Red, Green);');
  Add('var');
  Add('  e: TMyEnum;');
  Add('  f: TMyEnum = Green;');
  Add('  i: longint;');
  Add('begin');
  Add('  e:=green;');
  Add('  i:=longint(e);');
  ConvertProgram;
  CheckSource('TestEnumNumber',
    LinesToStr([ // statements
    'this.TMyEnum = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1',
    '  };',
    'this.e = 0;',
    'this.f = 1;',
    'this.i = 0;'
    ]),
    LinesToStr([
    '$mod.e=1;',
    '$mod.i=$mod.e;'
    ]));
end;

procedure TTestModule.TestEnum_ConstFail;
begin
  StartProgram(false);
  Add([
  'type TMyEnum = (Red = 100, Green = 101);',
  'var',
  '  e: TMyEnum;',
  '  f: TMyEnum = Green;',
  'begin',
  '  e:=green;']);
  SetExpectedPasResolverError('not yet implemented: Red:TPasEnumValue [20180126202434] "enum const"',3002);
  ConvertProgram;
end;

procedure TTestModule.TestEnum_Functions;
begin
  StartProgram(false);
  Add([
  'type TMyEnum = (Red, Green);',
  'procedure DoIt(var e: TMyEnum; var i: word);',
  'var',
  '  v: longint;',
  '  s: string;',
  'begin',
  '  val(s,e,v);',
  '  val(s,e,i);',
  'end;',
  'var',
  '  e: TMyEnum;',
  '  i: longint;',
  '  s: string;',
  '  b: boolean;',
  'begin',
  '  i:=ord(red);',
  '  i:=ord(green);',
  '  i:=ord(e);',
  '  i:=ord(b);',
  '  e:=low(tmyenum);',
  '  e:=low(e);',
  '  b:=low(boolean);',
  '  e:=high(tmyenum);',
  '  e:=high(e);',
  '  b:=high(boolean);',
  '  e:=pred(green);',
  '  e:=pred(e);',
  '  b:=pred(b);',
  '  e:=succ(red);',
  '  e:=succ(e);',
  '  b:=succ(b);',
  '  e:=tmyenum(1);',
  '  e:=tmyenum(i);',
  '  s:=str(e);',
  '  str(e,s);',
  '  str(red,s);',
  '  s:=str(e:3);',
  '  writestr(s,e:3,red);',
  '  val(s,e,i);',
  '  i:=longint(e);']);
  ConvertProgram;
  CheckSource('TestEnum_Functions',
    LinesToStr([ // statements
    'this.TMyEnum = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1',
    '  };',
    'this.DoIt = function (e, i) {',
    '  var v = 0;',
    '  var s = "";',
    '  e.set(rtl.valEnum(s, $mod.TMyEnum, function (w) {',
    '    v = w;',
    '  }));',
    '  e.set(rtl.valEnum(s, $mod.TMyEnum, i.set));',
    '};',
    'this.e = 0;',
    'this.i = 0;',
    'this.s = "";',
    'this.b = false;',
    '']),
    LinesToStr([
    '$mod.i=$mod.TMyEnum.Red;',
    '$mod.i=$mod.TMyEnum.Green;',
    '$mod.i=$mod.e;',
    '$mod.i=$mod.b+0;',
    '$mod.e=$mod.TMyEnum.Red;',
    '$mod.e=$mod.TMyEnum.Red;',
    '$mod.b=false;',
    '$mod.e=$mod.TMyEnum.Green;',
    '$mod.e=$mod.TMyEnum.Green;',
    '$mod.b=true;',
    '$mod.e=$mod.TMyEnum.Green-1;',
    '$mod.e=$mod.e-1;',
    '$mod.b=false;',
    '$mod.e=$mod.TMyEnum.Red+1;',
    '$mod.e=$mod.e+1;',
    '$mod.b=true;',
    '$mod.e=1;',
    '$mod.e=$mod.i;',
    '$mod.s = $mod.TMyEnum[$mod.e];',
    '$mod.s = $mod.TMyEnum[$mod.e];',
    '$mod.s = $mod.TMyEnum[$mod.TMyEnum.Red];',
    '$mod.s = rtl.spaceLeft($mod.TMyEnum[$mod.e], 3);',
    '$mod.s = rtl.spaceLeft($mod.TMyEnum[$mod.e], 3)+$mod.TMyEnum[$mod.TMyEnum.Red];',
    '$mod.e = rtl.valEnum($mod.s, $mod.TMyEnum, function (v) {',
    '  $mod.i = v;',
    '});',
    '$mod.i=$mod.e;',
    '']));
end;

procedure TTestModule.TestEnumRg_Functions;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (Red, Green, Blue);',
  '  TEnumRg = Green..Blue;',
  'procedure DoIt(var e: TEnumRg; var i: word);',
  'var',
  '  v: longint;',
  '  s: string;',
  'begin',
  '  val(s,e,v);',
  '  val(s,e,i);',
  'end;',
  'var',
  '  e: TEnumRg;',
  '  i: longint;',
  '  s: string;',
  'begin',
  '  i:=ord(green);',
  '  i:=ord(e);',
  '  e:=low(tenumrg);',
  '  e:=low(e);',
  '  e:=high(tenumrg);',
  '  e:=high(e);',
  '  e:=pred(blue);',
  '  e:=pred(e);',
  '  e:=succ(green);',
  '  e:=succ(e);',
  '  e:=tenumrg(1);',
  '  e:=tenumrg(i);',
  '  s:=str(e);',
  '  str(e,s);',
  '  str(red,s);',
  '  s:=str(e:3);',
  '  writestr(s,e:3,blue);',
  '  val(s,e,i);',
  '  i:=longint(e);']);
  ConvertProgram;
  CheckSource('TestEnumRg_Functions',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1,',
    '  "2":"Blue",',
    '  Blue:2',
    '  };',
    'this.DoIt = function (e, i) {',
    '  var v = 0;',
    '  var s = "";',
    '  e.set(rtl.valEnum(s, $mod.TEnum, function (w) {',
    '    v = w;',
    '  }));',
    '  e.set(rtl.valEnum(s, $mod.TEnum, i.set));',
    '};',
    'this.e = this.TEnum.Green;',
    'this.i = 0;',
    'this.s = "";',
    '']),
    LinesToStr([
    '$mod.i=$mod.TEnum.Green;',
    '$mod.i=$mod.e;',
    '$mod.e=$mod.TEnum.Green;',
    '$mod.e=$mod.TEnum.Green;',
    '$mod.e=$mod.TEnum.Blue;',
    '$mod.e=$mod.TEnum.Blue;',
    '$mod.e=$mod.TEnum.Blue-1;',
    '$mod.e=$mod.e-1;',
    '$mod.e=$mod.TEnum.Green+1;',
    '$mod.e=$mod.e+1;',
    '$mod.e=1;',
    '$mod.e=$mod.i;',
    '$mod.s = $mod.TEnum[$mod.e];',
    '$mod.s = $mod.TEnum[$mod.e];',
    '$mod.s = $mod.TEnum[$mod.TEnum.Red];',
    '$mod.s = rtl.spaceLeft($mod.TEnum[$mod.e], 3);',
    '$mod.s = rtl.spaceLeft($mod.TEnum[$mod.e], 3)+$mod.TEnum[$mod.TEnum.Blue];',
    '$mod.e = rtl.valEnum($mod.s, $mod.TEnum, function (v) {',
    '  $mod.i = v;',
    '});',
    '$mod.i=$mod.e;',
    '']));
end;

procedure TTestModule.TestEnum_AsParams;
begin
  StartProgram(false);
  Add('type TEnum = (Red,Blue);');
  Add('procedure DoIt(vG: TEnum; const vH: TEnum; var vI: TEnum);');
  Add('var vJ: TEnum;');
  Add('begin');
  Add('  vg:=vg;');
  Add('  vj:=vh;');
  Add('  vi:=vi;');
  Add('  doit(vg,vg,vg);');
  Add('  doit(vh,vh,vj);');
  Add('  doit(vi,vi,vi);');
  Add('  doit(vj,vj,vj);');
  Add('end;');
  Add('var i: TEnum;');
  Add('begin');
  Add('  doit(i,i,i);');
  ConvertProgram;
  CheckSource('TestEnum_AsParams',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "Red",',
    '  Red: 0,',
    '  "1": "Blue",',
    '  Blue: 1',
    '};',
    'this.DoIt = function (vG,vH,vI) {',
    '  var vJ = 0;',
    '  vG = vG;',
    '  vJ = vH;',
    '  vI.set(vI.get());',
    '  $mod.DoIt(vG, vG, {',
    '    get: function () {',
    '      return vG;',
    '    },',
    '    set: function (v) {',
    '      vG = v;',
    '    }',
    '  });',
    '  $mod.DoIt(vH, vH, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '  $mod.DoIt(vI.get(), vI.get(), vI);',
    '  $mod.DoIt(vJ, vJ, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '};',
    'this.i = 0;'
    ]),
    LinesToStr([
    '$mod.DoIt($mod.i,$mod.i,{',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '});'
    ]));
end;

procedure TTestModule.TestEnumRange_Array;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (Red, Green, Blue);',
  '  TEnumRg = green..blue;',
  '  TArr = array[TEnumRg] of byte;',
  '  TArr2 = array[green..blue] of byte;',
  'var',
  '  a: TArr;',
  '  b: TArr = (3,4);',
  '  c: TArr2 = (5,6);',
  'begin',
  '  a[green] := b[blue];',
  '  c[green] := c[blue];',
  '']);
  ConvertProgram;
  CheckSource('TestEnumRange_Array',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "Red",',
    '  Red: 0,',
    '  "1": "Green",',
    '  Green: 1,',
    '  "2": "Blue",',
    '  Blue: 2',
    '};',
    'this.a = rtl.arraySetLength(null, 0, 2);',
    'this.b = [3, 4];',
    'this.c = [5, 6];',
    '']),
    LinesToStr([
    '  $mod.a[$mod.TEnum.Green - 1] = $mod.b[$mod.TEnum.Blue - 1];',
    '  $mod.c[$mod.TEnum.Green - 1] = $mod.c[$mod.TEnum.Blue - 1];',
    '']));
end;

procedure TTestModule.TestEnum_ForIn;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (Red, Green, Blue);',
  '  TEnumRg = green..blue;',
  '  TArr = array[TEnum] of byte;',
  '  TArrRg = array[TEnumRg] of byte;',
  'var',
  '  e: TEnum;',
  '  a1: TArr = (3,4,5);',
  '  a2: TArrRg = (11,12);',
  '  b: byte;',
  'begin',
  '  for e in TEnum do ;',
  '  for e in TEnumRg do ;',
  '  for e in TArr do ;',
  '  for e in TArrRg do ;',
  '  for b in a1 do ;',
  '  for b in a2 do ;',
  '']);
  ConvertProgram;
  CheckSource('TestEnum_ForIn',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "Red",',
    '  Red: 0,',
    '  "1": "Green",',
    '  Green: 1,',
    '  "2": "Blue",',
    '  Blue: 2',
    '};',
    'this.e = 0;',
    'this.a1 = [3, 4, 5];',
    'this.a2 = [11, 12];',
    'this.b = 0;',
    '']),
    LinesToStr([
    '  for ($mod.e = 0; $mod.e <= 2; $mod.e++) ;',
    '  for ($mod.e = 1; $mod.e <= 2; $mod.e++) ;',
    '  for ($mod.e = 0; $mod.e <= 2; $mod.e++) ;',
    '  for ($mod.e = 1; $mod.e <= 2; $mod.e++) ;',
    '  for (var $in = $mod.a1, $l = 0, $end = rtl.length($in) - 1; $l <= $end; $l++) $mod.b = $in[$l];',
    '  for (var $in1 = $mod.a2, $l1 = 0, $end1 = rtl.length($in1) - 1; $l1 <= $end1; $l1++) $mod.b = $in1[$l1];',
    '']));
end;

procedure TTestModule.TestEnum_ScopedNumber;
begin
  Converter.Options:=Converter.Options+[coEnumNumbers];
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (Red, Green);',
  'var',
  '  e: TEnum;',
  'begin',
  '  e:=TEnum.Green;',
  '']);
  ConvertProgram;
  CheckSource('TestEnum_ScopedNumber',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "Red",',
    '  Red: 0,',
    '  "1": "Green",',
    '  Green: 1',
    '};',
    'this.e = 0;',
    '']),
    LinesToStr([
    '$mod.e = 1;']));
end;

procedure TTestModule.TestEnum_InFunction;
begin
  StartProgram(false);
  Add([
  'const TEnum = 3;',
  'procedure DoIt;',
  'type',
  '  TEnum = (Red, Green, Blue);',
  '  procedure Sub;',
  '  type',
  '    TEnumSub = (Left, Right);',
  '  var',
  '    es: TEnumSub;',
  '  begin',
  '    es:=Left;',
  '  end;',
  'var',
  '  e, e2: TEnum;',
  'begin',
  '  if e in [red,blue] then e2:=e;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestEnum_InFunction',
    LinesToStr([ // statements
    'this.TEnum = 3;',
    'var TEnum$1 = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1,',
    '  "2":"Blue",',
    '  Blue:2',
    '  };',
    'var TEnumSub = {',
    '  "0": "Left",',
    '  Left: 0,',
    '  "1": "Right",',
    '  Right: 1',
    '};',
    'this.DoIt = function () {',
    '  function Sub() {',
    '    var es = 0;',
    '    es = TEnumSub.Left;',
    '  };',
    '  var e = 0;',
    '  var e2 = 0;',
    '  if (e in rtl.createSet(TEnum$1.Red, TEnum$1.Blue)) e2 = e;',
    '};',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestEnum_Name_Anonymous_Unit;
begin
  StartUnit(true);
  Add([
  'interface',
  'var color: (red, green);',
  'implementation',
  'initialization',
  '  color:=green;',
  '']);
  ConvertUnit;
  CheckSource('TestEnum_Name_Anonymous_Unit',
    LinesToStr([
    'this.color$a = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "green",',
    '  green: 1',
    '};',
    'this.color = 0;',
    '']),
    LinesToStr([ // this.$init
    '$mod.color = $mod.color$a.green;',
    '']),
    LinesToStr([ // implementation
    '']) );
end;

procedure TTestModule.TestSet_Enum;
begin
  StartProgram(false);
  Add([
  'type',
  '  TColor = (Red, Green, Blue);',
  '  TColors = set of TColor;',
  'var',
  '  c: TColor;',
  '  s: TColors;',
  '  t: TColors = [];',
  '  u: TColors = [Red];',
  'begin',
  '  s:=[];',
  '  s:=[Green];',
  '  s:=[Green,Blue];',
  '  s:=[Red..Blue];',
  '  s:=[Red,Green..Blue];',
  '  s:=[Red,c];',
  '  s:=t;',
  '  s:=default(TColors);',
  '']);
  ConvertProgram;
  CheckSource('TestSet',
    LinesToStr([ // statements
    'this.TColor = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1,',
    '  "2":"Blue",',
    '  Blue:2',
    '  };',
    'this.c = 0;',
    'this.s = {};',
    'this.t = {};',
    'this.u = rtl.createSet(this.TColor.Red);'
    ]),
    LinesToStr([
    '$mod.s={};',
    '$mod.s=rtl.createSet($mod.TColor.Green);',
    '$mod.s=rtl.createSet($mod.TColor.Green,$mod.TColor.Blue);',
    '$mod.s=rtl.createSet(null,$mod.TColor.Red,$mod.TColor.Blue);',
    '$mod.s=rtl.createSet($mod.TColor.Red,null,$mod.TColor.Green,$mod.TColor.Blue);',
    '$mod.s=rtl.createSet($mod.TColor.Red,$mod.c);',
    '$mod.s=rtl.refSet($mod.t);',
    '$mod.s={};',
    '']));
end;

procedure TTestModule.TestSet_Operators;
begin
  StartProgram(false);
  Add('type');
  Add('  TColor = (Red, Green, Blue);');
  Add('  TColors = set of tcolor;');
  Add('var');
  Add('  vC: TColor;');
  Add('  vS: TColors;');
  Add('  vT: TColors;');
  Add('  vU: TColors;');
  Add('  B: boolean;');
  Add('begin');
  Add('  include(vs,green);');
  Add('  exclude(vs,vc);');
  Add('  vs:=vt+vu;');
  Add('  vs:=vt+[red];');
  Add('  vs:=[red]+vt;');
  Add('  vs:=[red]+[green];');
  Add('  vs:=vt-vu;');
  Add('  vs:=vt-[red];');
  Add('  vs:=[red]-vt;');
  Add('  vs:=[red]-[green];');
  Add('  vs:=vt*vu;');
  Add('  vs:=vt*[red];');
  Add('  vs:=[red]*vt;');
  Add('  vs:=[red]*[green];');
  Add('  vs:=vt><vu;');
  Add('  vs:=vt><[red];');
  Add('  vs:=[red]><vt;');
  Add('  vs:=[red]><[green];');
  Add('  b:=vt=vu;');
  Add('  b:=vt=[red];');
  Add('  b:=[red]=vt;');
  Add('  b:=[red]=[green];');
  Add('  b:=vt<>vu;');
  Add('  b:=vt<>[red];');
  Add('  b:=[red]<>vt;');
  Add('  b:=[red]<>[green];');
  Add('  b:=vt<=vu;');
  Add('  b:=vt<=[red];');
  Add('  b:=[red]<=vt;');
  Add('  b:=[red]<=[green];');
  Add('  b:=vt>=vu;');
  Add('  b:=vt>=[red];');
  Add('  b:=[red]>=vt;');
  Add('  b:=[red]>=[green];');
  ConvertProgram;
  CheckSource('TestSet_Operators',
    LinesToStr([ // statements
    'this.TColor = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1,',
    '  "2":"Blue",',
    '  Blue:2',
    '  };',
    'this.vC = 0;',
    'this.vS = {};',
    'this.vT = {};',
    'this.vU = {};',
    'this.B = false;'
    ]),
    LinesToStr([
    '$mod.vS = rtl.includeSet($mod.vS,$mod.TColor.Green);',
    '$mod.vS = rtl.excludeSet($mod.vS,$mod.vC);',
    '$mod.vS = rtl.unionSet($mod.vT, $mod.vU);',
    '$mod.vS = rtl.unionSet($mod.vT, rtl.createSet($mod.TColor.Red));',
    '$mod.vS = rtl.unionSet(rtl.createSet($mod.TColor.Red), $mod.vT);',
    '$mod.vS = rtl.unionSet(rtl.createSet($mod.TColor.Red), rtl.createSet($mod.TColor.Green));',
    '$mod.vS = rtl.diffSet($mod.vT, $mod.vU);',
    '$mod.vS = rtl.diffSet($mod.vT, rtl.createSet($mod.TColor.Red));',
    '$mod.vS = rtl.diffSet(rtl.createSet($mod.TColor.Red), $mod.vT);',
    '$mod.vS = rtl.diffSet(rtl.createSet($mod.TColor.Red), rtl.createSet($mod.TColor.Green));',
    '$mod.vS = rtl.intersectSet($mod.vT, $mod.vU);',
    '$mod.vS = rtl.intersectSet($mod.vT, rtl.createSet($mod.TColor.Red));',
    '$mod.vS = rtl.intersectSet(rtl.createSet($mod.TColor.Red), $mod.vT);',
    '$mod.vS = rtl.intersectSet(rtl.createSet($mod.TColor.Red), rtl.createSet($mod.TColor.Green));',
    '$mod.vS = rtl.symDiffSet($mod.vT, $mod.vU);',
    '$mod.vS = rtl.symDiffSet($mod.vT, rtl.createSet($mod.TColor.Red));',
    '$mod.vS = rtl.symDiffSet(rtl.createSet($mod.TColor.Red), $mod.vT);',
    '$mod.vS = rtl.symDiffSet(rtl.createSet($mod.TColor.Red), rtl.createSet($mod.TColor.Green));',
    '$mod.B = rtl.eqSet($mod.vT, $mod.vU);',
    '$mod.B = rtl.eqSet($mod.vT, rtl.createSet($mod.TColor.Red));',
    '$mod.B = rtl.eqSet(rtl.createSet($mod.TColor.Red), $mod.vT);',
    '$mod.B = rtl.eqSet(rtl.createSet($mod.TColor.Red), rtl.createSet($mod.TColor.Green));',
    '$mod.B = rtl.neSet($mod.vT, $mod.vU);',
    '$mod.B = rtl.neSet($mod.vT, rtl.createSet($mod.TColor.Red));',
    '$mod.B = rtl.neSet(rtl.createSet($mod.TColor.Red), $mod.vT);',
    '$mod.B = rtl.neSet(rtl.createSet($mod.TColor.Red), rtl.createSet($mod.TColor.Green));',
    '$mod.B = rtl.leSet($mod.vT, $mod.vU);',
    '$mod.B = rtl.leSet($mod.vT, rtl.createSet($mod.TColor.Red));',
    '$mod.B = rtl.leSet(rtl.createSet($mod.TColor.Red), $mod.vT);',
    '$mod.B = rtl.leSet(rtl.createSet($mod.TColor.Red), rtl.createSet($mod.TColor.Green));',
    '$mod.B = rtl.geSet($mod.vT, $mod.vU);',
    '$mod.B = rtl.geSet($mod.vT, rtl.createSet($mod.TColor.Red));',
    '$mod.B = rtl.geSet(rtl.createSet($mod.TColor.Red), $mod.vT);',
    '$mod.B = rtl.geSet(rtl.createSet($mod.TColor.Red), rtl.createSet($mod.TColor.Green));',
    '']));
end;

procedure TTestModule.TestSet_Operator_In;
begin
  StartProgram(false);
  Add([
  'type',
  '  TColor = (Red, Green, Blue);',
  '  TColors = set of tcolor;',
  '  TColorRg = green..blue;',
  'var',
  '  vC: tcolor;',
  '  vT: tcolors;',
  '  B: boolean;',
  '  rg: TColorRg;',
  'begin',
  '  b:=red in vt;',
  '  b:=vc in vt;',
  '  b:=green in [red..blue];',
  '  b:=vc in [red..blue];',
  '  ',
  '  if red in vt then ;',
  '  while vC in vt do ;',
  '  repeat',
  '  until vC in vt;',
  '  if rg in [green..blue] then ;',
  '']);
  ConvertProgram;
  CheckSource('TestSet_Operator_In',
    LinesToStr([ // statements
    'this.TColor = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1,',
    '  "2":"Blue",',
    '  Blue:2',
    '  };',
    'this.vC = 0;',
    'this.vT = {};',
    'this.B = false;',
    'this.rg = this.TColor.Green;',
    '']),
    LinesToStr([
    '$mod.B = $mod.TColor.Red in $mod.vT;',
    '$mod.B = $mod.vC in $mod.vT;',
    '$mod.B = $mod.TColor.Green in rtl.createSet(null, $mod.TColor.Red, $mod.TColor.Blue);',
    '$mod.B = $mod.vC in rtl.createSet(null, $mod.TColor.Red, $mod.TColor.Blue);',
    'if ($mod.TColor.Red in $mod.vT) ;',
    'while ($mod.vC in $mod.vT) {',
    '};',
    'do {',
    '} while (!($mod.vC in $mod.vT));',
    'if ($mod.rg in rtl.createSet(null, $mod.TColor.Green, $mod.TColor.Blue)) ;',
    '']));
end;

procedure TTestModule.TestSet_Functions;
begin
  StartProgram(false);
  Add('type');
  Add('  TMyEnum = (Red, Green);');
  Add('  TMyEnums = set of TMyEnum;');
  Add('var');
  Add('  e: TMyEnum;');
  Add('  s: TMyEnums;');
  Add('begin');
  Add('  e:=Low(TMyEnums);');
  Add('  e:=Low(s);');
  Add('  e:=High(TMyEnums);');
  Add('  e:=High(s);');
  ConvertProgram;
  CheckSource('TestSetFunctions',
    LinesToStr([ // statements
    'this.TMyEnum = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1',
    '  };',
    'this.e = 0;',
    'this.s = {};'
    ]),
    LinesToStr([
    '$mod.e=$mod.TMyEnum.Red;',
    '$mod.e=$mod.TMyEnum.Red;',
    '$mod.e=$mod.TMyEnum.Green;',
    '$mod.e=$mod.TMyEnum.Green;',
    '']));
end;

procedure TTestModule.TestSet_PassAsArgClone;
begin
  StartProgram(false);
  Add('type');
  Add('  TMyEnum = (Red, Green);');
  Add('  TMyEnums = set of TMyEnum;');
  Add('procedure DoDefault(s: tmyenums); begin end;');
  Add('procedure DoConst(const s: tmyenums); begin end;');
  Add('var');
  Add('  aSet: tmyenums;');
  Add('begin');
  Add('  dodefault(aset);');
  Add('  doconst(aset);');
  ConvertProgram;
  CheckSource('TestSetFunctions',
    LinesToStr([ // statements
    'this.TMyEnum = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1',
    '  };',
    'this.DoDefault = function (s) {',
    '};',
    'this.DoConst = function (s) {',
    '};',
    'this.aSet = {};'
    ]),
    LinesToStr([
    '$mod.DoDefault(rtl.refSet($mod.aSet));',
    '$mod.DoConst($mod.aSet);',
    '']));
end;

procedure TTestModule.TestSet_AsParams;
begin
  StartProgram(false);
  Add([
  'type TEnum = (Red,Blue);',
  'type TEnums = set of TEnum;',
  'function DoIt(vG: TEnums; const vH: TEnums; var vI: TEnums): TEnums;',
  'var vJ: TEnums;',
  'begin',
  '  Include(vg,red);',
  '  Include(result,blue);',
  '  vg:=vg;',
  '  vj:=vh;',
  '  vi:=vi;',
  '  doit(vg,vg,vg);',
  '  doit(vh,vh,vj);',
  '  doit(vi,vi,vi);',
  '  doit(vj,vj,vj);',
  'end;',
  'var i: TEnums;',
  'begin',
  '  doit(i,i,i);']);
  ConvertProgram;
  CheckSource('TestSet_AsParams',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "Red",',
    '  Red: 0,',
    '  "1": "Blue",',
    '  Blue: 1',
    '};',
    'this.DoIt = function (vG,vH,vI) {',
    '  var Result = {};',
    '  var vJ = {};',
    '  vG = rtl.includeSet(vG, $mod.TEnum.Red);',
    '  Result = rtl.includeSet(Result, $mod.TEnum.Blue);',
    '  vG = rtl.refSet(vG);',
    '  vJ = rtl.refSet(vH);',
    '  vI.set(rtl.refSet(vI.get()));',
    '  $mod.DoIt(rtl.refSet(vG), vG, {',
    '    get: function () {',
    '      return vG;',
    '    },',
    '    set: function (v) {',
    '      vG = v;',
    '    }',
    '  });',
    '  $mod.DoIt(rtl.refSet(vH), vH, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '  $mod.DoIt(rtl.refSet(vI.get()), vI.get(), vI);',
    '  $mod.DoIt(rtl.refSet(vJ), vJ, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '  return Result;',
    '};',
    'this.i = {};'
    ]),
    LinesToStr([
    '$mod.DoIt(rtl.refSet($mod.i),$mod.i,{',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '});'
    ]));
end;

procedure TTestModule.TestSet_Property;
begin
  StartProgram(false);
  Add('type');
  Add('  TEnum = (Red,Blue);');
  Add('  TEnums = set of TEnum;');
  Add('  TObject = class');
  Add('    function GetColors: TEnums; external name ''GetColors'';');
  Add('    procedure SetColors(const Value: TEnums); external name ''SetColors'';');
  Add('    property Colors: TEnums read GetColors write SetColors;');
  Add('  end;');
  Add('procedure DoIt(i: TEnums; const j: TEnums; var k: TEnums; out l: TEnums);');
  Add('begin end;');
  Add('var Obj: TObject;');
  Add('begin');
  Add('  Include(Obj.Colors,Red);');
  Add('  Exclude(Obj.Colors,Red);');
  //Add('  DoIt(Obj.Colors,Obj.Colors,Obj.Colors,Obj.Colors);');
  ConvertProgram;
  CheckSource('TestSet_Property',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "Red",',
    '  Red: 0,',
    '  "1": "Blue",',
    '  Blue: 1',
    '};',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.DoIt = function (i, j, k, l) {',
    '};',
    'this.Obj = null;',
    '']),
    LinesToStr([
    '$mod.Obj.SetColors(rtl.includeSet($mod.Obj.GetColors(), $mod.TEnum.Red));',
    '$mod.Obj.SetColors(rtl.excludeSet($mod.Obj.GetColors(), $mod.TEnum.Red));',
    '']));
end;

procedure TTestModule.TestSet_EnumConst;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (Red,Blue);',
  '  TEnums = set of TEnum;',
  'const',
  '  Orange = red;',
  'var',
  '  Enum: tenum;',
  '  Enums: tenums;',
  'begin',
  '  Include(enums,orange);',
  '  Exclude(enums,orange);',
  '  if orange in enums then;',
  '  if orange in [orange,red] then;']);
  ConvertProgram;
  CheckSource('TestSet_EnumConst',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "Red",',
    '  Red: 0,',
    '  "1": "Blue",',
    '  Blue: 1',
    '};',
    'this.Orange = this.TEnum.Red;',
    'this.Enum = 0;',
    'this.Enums = {};',
    '']),
    LinesToStr([
    '$mod.Enums = rtl.includeSet($mod.Enums, $mod.TEnum.Red);',
    '$mod.Enums = rtl.excludeSet($mod.Enums, $mod.TEnum.Red);',
    'if ($mod.TEnum.Red in $mod.Enums) ;',
    'if ($mod.TEnum.Red in rtl.createSet($mod.TEnum.Red, $mod.TEnum.Red)) ;',
    '']));
end;

procedure TTestModule.TestSet_IntConst;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnums = set of Byte;',
  'const',
  '  Orange = 0;',
  'var',
  '  Enum: byte;',
  '  Enums: tenums;',
  'begin',
  '  Enums:=[];',
  '  Enums:=[0];',
  '  Enums:=[1..2];',
  //'  Include(enums,orange);',
  //'  Exclude(enums,orange);',
  '  if orange in enums then;',
  '  if orange in [orange,1] then;']);
  ConvertProgram;
  CheckSource('TestSet_IntConst',
    LinesToStr([ // statements
    'this.Orange = 0;',
    'this.Enum = 0;',
    'this.Enums = {};',
    '']),
    LinesToStr([
    '$mod.Enums = {};',
    '$mod.Enums = rtl.createSet(0);',
    '$mod.Enums = rtl.createSet(null, 1, 2);',
    'if (0 in $mod.Enums) ;',
    'if (0 in rtl.createSet(0, 1)) ;',
    '']));
end;

procedure TTestModule.TestSet_IntRange;
begin
  StartProgram(false);
  Add([
  'type',
  '  TRange = 1..3;',
  '  TEnums = set of TRange;',
  'const',
  '  Orange = 2;',
  'var',
  '  Enum: byte;',
  '  Enums: TEnums;',
  'begin',
  '  Enums:=[];',
  '  Enums:=[1];',
  '  Enums:=[2..3];',
  '  Include(enums,orange);',
  '  Exclude(enums,orange);',
  '  if orange in enums then;',
  '  if orange in [orange,1] then;']);
  ConvertProgram;
  CheckSource('TestSet_IntRange',
    LinesToStr([ // statements
    'this.Orange = 2;',
    'this.Enum = 0;',
    'this.Enums = {};',
    '']),
    LinesToStr([
    '$mod.Enums = {};',
    '$mod.Enums = rtl.createSet(1);',
    '$mod.Enums = rtl.createSet(null, 2, 3);',
    '$mod.Enums = rtl.includeSet($mod.Enums, 2);',
    '$mod.Enums = rtl.excludeSet($mod.Enums, 2);',
    'if (2 in $mod.Enums) ;',
    'if (2 in rtl.createSet(2, 1)) ;',
    '']));
end;

procedure TTestModule.TestSet_AnonymousEnumType;
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
  ConvertProgram;
  CheckSource('TestSet_AnonymousEnumType',
    LinesToStr([ // statements
    'this.TFlags$a = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "green",',
    '  green: 1',
    '};',
    'this.favorite = this.TFlags$a.red;',
    'this.f = {};',
    'this.i = 0;',
    '']),
    LinesToStr([
    '$mod.f = rtl.includeSet($mod.f, $mod.TFlags$a.red);',
    '$mod.f = rtl.includeSet($mod.f, $mod.TFlags$a.red);',
    '$mod.i = $mod.TFlags$a.red;',
    '$mod.i = $mod.TFlags$a.red;',
    '$mod.i = $mod.TFlags$a.red;',
    '$mod.i = $mod.TFlags$a.red;',
    '$mod.i = $mod.TFlags$a.red;',
    '$mod.i = $mod.TFlags$a.green;',
    '$mod.i = $mod.TFlags$a.green;',
    '$mod.i = $mod.TFlags$a.green;',
    '$mod.f = rtl.createSet($mod.TFlags$a.green, $mod.TFlags$a.red);',
    '']));
end;

procedure TTestModule.TestSet_AnonymousEnumTypeChar;
begin
  exit;
  StartProgram(false);
  Add([
  'type',
  '  TAtoZ = ''A''..''Z'';',
  '  TSetOfAZ = set of TAtoZ;',
  'var',
  '  c: char;',
  '  a: TAtoZ;',
  '  s: TSetOfAZ = [''P'',''A''];',
  '  i: longint;',
  'begin',
  '  Include(s,''S'');',
  '  Include(s,c);',
  '  Include(s,a);',
  '  c:=low(TAtoZ);',
  '  i:=ord(low(TAtoZ));',
  '  a:=high(TAtoZ);',
  '  a:=high(TSetOfAtoZ);',
  '  s:=[a,c,''M''];',
  '']);
  ConvertProgram;
  CheckSource('TestSet_AnonymousEnumTypeChar',
    LinesToStr([ // statements
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestSet_ConstEnum;
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
  '  if e in [red,blue] then;',
  '  s:=teAny;',
  '  s:=teAny+[e];',
  '  s:=[e]+teAny;',
  '  s:=teAny+teRedBlue;',
  '  s:=teAny+teRedBlue+[e];',
  '']);
  ConvertProgram;
  CheckSource('TestSet_ConstEnum',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1,',
    '  "2": "green",',
    '  green: 2',
    '};',
    'this.teAny = rtl.createSet(null, this.TEnum.red, this.TEnum.green);',
    'this.teRedBlue = rtl.createSet(null, this.TEnum.red, this.TEnum.green - 1);',
    'this.e = 0;',
    'this.s = {};',
    '']),
    LinesToStr([
    'if ($mod.TEnum.blue in $mod.teAny) ;',
    'if ($mod.TEnum.blue in rtl.unionSet($mod.teAny, rtl.createSet($mod.e))) ;',
    'if ($mod.TEnum.blue in rtl.unionSet($mod.teAny, $mod.teRedBlue)) ;',
    'if ($mod.e in rtl.createSet($mod.TEnum.red, $mod.TEnum.blue)) ;',
    '$mod.s = rtl.refSet($mod.teAny);',
    '$mod.s = rtl.unionSet($mod.teAny, rtl.createSet($mod.e));',
    '$mod.s = rtl.unionSet(rtl.createSet($mod.e), $mod.teAny);',
    '$mod.s = rtl.unionSet($mod.teAny, $mod.teRedBlue);',
    '$mod.s = rtl.unionSet(rtl.unionSet($mod.teAny, $mod.teRedBlue), rtl.createSet($mod.e));',
    '']));
end;

procedure TTestModule.TestSet_ConstChar;
begin
  StartProgram(false);
  Add([
  'const',
  '  LowChars = [''a''..''z''];',
  '  Chars = LowChars+[''A''..''Z''];',
  '  sc = [''А'', ''Я''];',
  'var',
  '  c: char;',
  '  s: string;',
  'begin',
  '  if c in lowchars then ;',
  '  if ''a'' in lowchars then ;',
  '  if s[1] in lowchars then ;',
  '  if c in chars then ;',
  '  if c in [''a''..''z'',''_''] then ;',
  '  if ''b'' in [''a''..''z'',''_''] then ;',
  '  if ''Я'' in sc then ;',
  '  if 3=ord('' '') then ;',
  '']);
  ConvertProgram;
  CheckSource('TestSet_ConstChar',
    LinesToStr([ // statements
    'this.LowChars = rtl.createSet(null, 97, 122);',
    'this.Chars = rtl.unionSet(this.LowChars, rtl.createSet(null, 65, 90));',
    'this.sc = rtl.createSet(1040, 1071);',
    'this.c = "";',
    'this.s = "";',
    '']),
    LinesToStr([
    'if ($mod.c.charCodeAt() in $mod.LowChars) ;',
    'if (97 in $mod.LowChars) ;',
    'if ($mod.s.charCodeAt(0) in $mod.LowChars) ;',
    'if ($mod.c.charCodeAt() in $mod.Chars) ;',
    'if ($mod.c.charCodeAt() in rtl.createSet(null, 97, 122, 95)) ;',
    'if (98 in rtl.createSet(null, 97, 122, 95)) ;',
    'if (1071 in $mod.sc) ;',
    'if (3 === 32) ;',
    '']));
end;

procedure TTestModule.TestSet_ConstInt;
begin
  StartProgram(false);
  Add([
  'const',
  '  Months = [1..12];',
  '  Mirror = [-12..-1]+Months;',
  'var',
  '  i: smallint;',
  'begin',
  '  if 3 in Months then;',
  '  if i in Months+[i] then;',
  '  if i in Months+Mirror then;',
  '  if i in [4..6,8] then;',
  '']);
  ConvertProgram;
  CheckSource('TestSet_ConstInt',
    LinesToStr([ // statements
    'this.Months = rtl.createSet(null, 1, 12);',
    'this.Mirror = rtl.unionSet(rtl.createSet(null, -12, -1), this.Months);',
    'this.i = 0;',
    '']),
    LinesToStr([
    'if (3 in $mod.Months) ;',
    'if ($mod.i in rtl.unionSet($mod.Months, rtl.createSet($mod.i))) ;',
    'if ($mod.i in rtl.unionSet($mod.Months, $mod.Mirror)) ;',
    'if ($mod.i in rtl.createSet(null, 4, 6, 8)) ;',
    '']));
end;

procedure TTestModule.TestSet_InFunction;
begin
  StartProgram(false);
  Add([
  'const',
  '  TEnum = 3;',
  '  TSetOfEnum = 4;',
  '  TSetOfAno = 5;',
  'procedure DoIt;',
  'type',
  '  TEnum = (red, blue);',
  '  TSetOfEnum = set of TEnum;',
  '  TSetOfAno = set of (up,down);',
  'var',
  '  e: TEnum;',
  '  se: TSetOfEnum;',
  '  sa: TSetOfAno;',
  'begin',
  '  se:=[e];',
  '  sa:=[up];',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestSet_InFunction',
    LinesToStr([ // statements
    'this.TEnum = 3;',
    'this.TSetOfEnum = 4;',
    'this.TSetOfAno = 5;',
    'var TEnum$1 = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'var TSetOfAno$a = {',
    '  "0": "up",',
    '  up: 0,',
    '  "1": "down",',
    '  down: 1',
    '};',
    'this.DoIt = function () {',
    '  var e = 0;',
    '  var se = {};',
    '  var sa = {};',
    '  se = rtl.createSet(e);',
    '  sa = rtl.createSet(TSetOfAno$a.up);',
    '};',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestSet_ForIn;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (Red, Green, Blue);',
  '  TEnumRg = green..blue;',
  '  TSetOfEnum = set of TEnum;',
  '  TSetOfEnumRg = set of TEnumRg;',
  'var',
  '  e, e2: TEnum;',
  '  er: TEnum;',
  '  s: TSetOfEnum;',
  'begin',
  '  for e in TSetOfEnum do ;',
  '  for e in TSetOfEnumRg do ;',
  '  for e in [] do e2:=e;',
  '  for e in [red..green] do e2:=e;',
  '  for e in [green,blue] do e2:=e;',
  '  for e in [red,blue] do e2:=e;',
  '  for e in s do e2:=e;',
  '  for er in TSetOfEnumRg do ;',
  '']);
  ConvertProgram;
  CheckSource('TestSet_ForIn',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0":"Red",',
    '  Red:0,',
    '  "1":"Green",',
    '  Green:1,',
    '  "2":"Blue",',
    '  Blue:2',
    '  };',
    'this.e = 0;',
    'this.e2 = 0;',
    'this.er = 0;',
    'this.s = {};',
    '']),
    LinesToStr([
    'for ($mod.e = 0; $mod.e <= 2; $mod.e++) ;',
    'for ($mod.e = 1; $mod.e <= 2; $mod.e++) ;',
    'for ($mod.e = 0; $mod.e <= 1; $mod.e++) $mod.e2 = $mod.e;',
    'for ($mod.e = 1; $mod.e <= 2; $mod.e++) $mod.e2 = $mod.e;',
    'for ($mod.e in rtl.createSet($mod.TEnum.Red, $mod.TEnum.Blue)) $mod.e2 = $mod.e;',
    'for (var $l in $mod.s){',
    '  $mod.e = +$l;',
    '  $mod.e2 = $mod.e;',
    '};',
    'for ($mod.er = 1; $mod.er <= 2; $mod.er++) ;',
    '']));
end;

procedure TTestModule.TestNestBegin;
begin
  StartProgram(false);
  Add('begin');
  Add('  begin');
  Add('    begin');
  Add('    end;');
  Add('    begin');
  Add('      if true then ;');
  Add('    end;');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestNestBegin',
    '',
    'if (true) ;');
end;

procedure TTestModule.TestUnitImplVars;
begin
  StartUnit(false);
  Add('interface');
  Add('implementation');
  Add('var');
  Add('  V1:longint;');
  Add('  V2:longint = 3;');
  Add('  V3:string = ''abc'';');
  ConvertUnit;
  CheckSource('TestUnitImplVars',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    '']),
    '', // this.$init
    LinesToStr([ // implementation
    '$impl.V1 = 0;',
    '$impl.V2 = 3;',
    '$impl.V3 = "abc";',
    '']) );
end;

procedure TTestModule.TestUnitImplConsts;
begin
  StartUnit(false);
  Add('interface');
  Add('implementation');
  Add('const');
  Add('  v1 = 3;');
  Add('  v2:longint = 4;');
  Add('  v3:string = ''abc'';');
  ConvertUnit;
  CheckSource('TestUnitImplConsts',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    '']),
    '', // this.$init
    LinesToStr([ // implementation
    '$impl.v1 = 3;',
    '$impl.v2 = 4;',
    '$impl.v3 = "abc";',
    '']) );
end;

procedure TTestModule.TestUnitImplRecord;
begin
  StartUnit(false);
  Add('interface');
  Add('implementation');
  Add('type');
  Add('  TMyRecord = record');
  Add('    i: longint;');
  Add('  end;');
  Add('var aRec: TMyRecord;');
  Add('initialization');
  Add('  arec.i:=3;');
  ConvertUnit;
  CheckSource('TestUnitImplRecord',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    '']),
    // this.$init
    '$impl.aRec.i = 3;',
    LinesToStr([ // implementation
    'rtl.recNewT($impl, "TMyRecord", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return this.i === b.i;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    return this;',
    '  };',
    '});',
    '$impl.aRec = $impl.TMyRecord.$new();',
    '']) );
end;

procedure TTestModule.TestRenameJSNameConflict;
begin
  StartProgram(false);
  Add('var apply: longint;');
  Add('var bind: longint;');
  Add('var call: longint;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestRenameJSNameConflict',
    LinesToStr([ // statements
    'this.Apply = 0;',
    'this.Bind = 0;',
    'this.Call = 0;'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestLocalConst;
begin
  StartProgram(false);
  Add('procedure DoIt;');
  Add('const');
  Add('  cA: longint = 1;');
  Add('  cB = 2;');
  Add('  procedure Sub;');
  Add('  const');
  Add('    csA = 3;');
  Add('    cB: double = 4;');
  Add('  begin');
  Add('    cb:=cb+csa;');
  Add('    ca:=ca+csa+5;');
  Add('  end;');
  Add('begin');
  Add('  ca:=ca+cb+6;');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestLocalConst',
    LinesToStr([
    'var cA = 1;',
    'var cB = 2;',
    'var csA = 3;',
    'var cB$1 = 4;',
    'this.DoIt = function () {',
    '  function Sub() {',
    '    cB$1 = cB$1 + 3;',
    '    cA = cA + 3 + 5;',
    '  };',
    '  cA = cA + 2 + 6;',
    '};'
    ]),
    LinesToStr([
    ]));
end;

procedure TTestModule.TestVarExternal;
begin
  StartProgram(false);
  Add('var');
  Add('  NaN: double; external name ''Global.NaN'';');
  Add('  d: double;');
  Add('begin');
  Add('  d:=NaN;');
  ConvertProgram;
  CheckSource('TestVarExternal',
    LinesToStr([
    'this.d = 0.0;'
    ]),
    LinesToStr([
    '$mod.d = Global.NaN;'
    ]));
end;

procedure TTestModule.TestVarExternalOtherUnit;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'var NaN: double; external name ''Global.NaN'';',
    'var iV: longint;'
    ]),
    '');

  StartUnit(true);
  Add('interface');
  Add('uses unit2;');
  Add('implementation');
  Add('var');
  Add('  d: double;');
  Add('  i: longint; external name ''$i'';');
  Add('begin');
  Add('  d:=nan;');
  Add('  d:=uNit2.nan;');
  Add('  d:=test1.d;');
  Add('  i:=iv;');
  Add('  i:=uNit2.iv;');
  Add('  i:=test1.i;');
  ConvertUnit;
  CheckSource('TestVarExternalOtherUnit',
    LinesToStr([
    'var $impl = $mod.$impl;',
    '']),
    LinesToStr([ // this.$init
    '$impl.d = Global.NaN;',
    '$impl.d = Global.NaN;',
    '$impl.d = $impl.d;',
    '$i = pas.unit2.iV;',
    '$i = pas.unit2.iV;',
    '$i = $i;',
    '']),
    LinesToStr([ // implementation
    '$impl.d = 0.0;',
    '']) );
end;

procedure TTestModule.TestVarAbsoluteFail;
begin
  StartProgram(false);
  Add([
  'var',
  '  a: longint;',
  '  b: longword absolute a;',
  'begin']);
  SetExpectedPasResolverError('Invalid variable modifier "absolute"',nInvalidVariableModifier);
  ConvertProgram;
end;

procedure TTestModule.TestConstExternal;
begin
  StartProgram(false);
  Add([
  'const',
  '  PI: double; external name ''Global.PI'';',
  '  Tau = 2*pi;',
  'var d: double;',
  'begin',
  '  d:=pi;',
  '  d:=tau+pi;']);
  ConvertProgram;
  CheckSource('TestConstExternal',
    LinesToStr([
    'this.Tau = 2*Global.PI;',
    'this.d = 0.0;'
    ]),
    LinesToStr([
    '$mod.d = Global.PI;',
    '$mod.d = $mod.Tau + Global.PI;'
    ]));
end;

procedure TTestModule.TestDouble;
begin
  StartProgram(false);
  Add([
  'type',
  '  TDateTime = double;',
  'const',
  '  a = TDateTime(2.7);',
  '  b = a + TDateTime(1.7);',
  '  c = 0.9 + 0.1;',
  '  f0_1 = 0.1;',
  '  f0_3 = 0.3;',
  '  fn0_1 = -0.1;',
  '  fn0_3 = -0.3;',
  '  fn0_003 = -0.003;',
  '  fn0_123456789 = -0.123456789;',
  '  fn300_0 = -300.0;',
  '  fn123456_0 = -123456.0;',
  '  fn1234567_8 = -1234567.8;',
  '  fn12345678_9 = -12345678.9;',
  '  f1_0En12 = 1E-12;',
  '  fn1_0En12 = -1E-12;',
  '  maxdouble = 1.7e+308;',
  '  mindouble = -1.7e+308;',
  '  MinSafeIntDouble  = -$1fffffffffffff;',
  '  MinSafeIntDouble2 = -$20000000000000-1;',
  '  MaxSafeIntDouble =   $1fffffffffffff;',
  '  DZeroResolution = 1E-12;',
  '  Minus1 = -1E-12;',
  '  EPS = 1E-9;',
  '  DELTA = 0.001;',
  '  Big = 129.789E+100;',
  '  Test0_15 = 0.15;',
  '  Test999 = 2.9999999999999;',
  '  Test111999 = 211199999999999000.0;',
  '  TestMinus111999 = -211199999999999000.0;',
  'var',
  '  d: double = b;',
  'begin',
  '  d:=1.0;',
  '  d:=1.0/3.0;',
  '  d:=1/3;',
  '  d:=5.0E-324;',
  '  d:=1.7E308;',
  '  d:=001.00E00;',
  '  d:=002.00E001;',
  '  d:=003.000E000;',
  '  d:=-004.00E-00;',
  '  d:=-005.00E-001;',
  '  d:=10**3;',
  '  d:=10 mod 3;',
  '  d:=10 div 3;',
  '  d:=c;',
  '  d:=f0_1;',
  '  d:=f0_3;',
  '  d:=fn0_1;',
  '  d:=fn0_3;',
  '  d:=fn0_003;',
  '  d:=fn0_123456789;',
  '  d:=fn300_0;',
  '  d:=fn123456_0;',
  '  d:=fn1234567_8;',
  '  d:=fn12345678_9;',
  '  d:=f1_0En12;',
  '  d:=fn1_0En12;',
  '  d:=maxdouble;',
  '  d:=mindouble;',
  '  d:=MinSafeIntDouble;',
  '  d:=double(MinSafeIntDouble);',
  '  d:=MinSafeIntDouble2;',
  '  d:=double(MinSafeIntDouble2);',
  '  d:=MaxSafeIntDouble;',
  '  d:=default(double);',
  '']);
  ConvertProgram;
  CheckSource('TestDouble',
    LinesToStr([
    'this.a = 2.7;',
    'this.b = 2.7 + 1.7;',
    'this.c = 0.9 + 0.1;',
    'this.f0_1 = 0.1;',
    'this.f0_3 = 0.3;',
    'this.fn0_1 = -0.1;',
    'this.fn0_3 = -0.3;',
    'this.fn0_003 = -0.003;',
    'this.fn0_123456789 = -0.123456789;',
    'this.fn300_0 = -300.0;',
    'this.fn123456_0 = -123456.0;',
    'this.fn1234567_8 = -1234567.8;',
    'this.fn12345678_9 = -12345678.9;',
    'this.f1_0En12 = 1E-12;',
    'this.fn1_0En12 = -1E-12;',
    'this.maxdouble = 1.7e+308;',
    'this.mindouble = -1.7e+308;',
    'this.MinSafeIntDouble = -0x1fffffffffffff;',
    'this.MinSafeIntDouble2 = -0x20000000000000 - 1;',
    'this.MaxSafeIntDouble = 0x1fffffffffffff;',
    'this.DZeroResolution = 1E-12;',
    'this.Minus1 = -1E-12;',
    'this.EPS = 1E-9;',
    'this.DELTA = 0.001;',
    'this.Big = 129.789E+100;',
    'this.Test0_15 = 0.15;',
    'this.Test999 = 2.9999999999999;',
    'this.Test111999 = 211199999999999000.0;',
    'this.TestMinus111999 = -211199999999999000.0;',
    'this.d = 4.4;'
    ]),
    LinesToStr([
    '$mod.d = 1.0;',
    '$mod.d = 1.0 / 3.0;',
    '$mod.d = 1 / 3;',
    '$mod.d = 5.0E-324;',
    '$mod.d = 1.7E308;',
    '$mod.d = 1.00E0;',
    '$mod.d = 2.00E1;',
    '$mod.d = 3.000E0;',
    '$mod.d = -4.00E-0;',
    '$mod.d = -5.00E-1;',
    '$mod.d = Math.pow(10, 3);',
    '$mod.d = 10 % 3;',
    '$mod.d = rtl.trunc(10 / 3);',
    '$mod.d = 1;',
    '$mod.d = 0.1;',
    '$mod.d = 0.3;',
    '$mod.d = -0.1;',
    '$mod.d = -0.3;',
    '$mod.d = -0.003;',
    '$mod.d = -0.123456789;',
    '$mod.d = -300;',
    '$mod.d = -123456;',
    '$mod.d = -1234567.8;',
    '$mod.d = -1.23456789E7;',
    '$mod.d = 1E-12;',
    '$mod.d = -1E-12;',
    '$mod.d = 1.7E308;',
    '$mod.d = -1.7E308;',
    '$mod.d = -9007199254740991;',
    '$mod.d = -9007199254740991;',
    '$mod.d = -9.007199254740992E15;',
    '$mod.d = -9.007199254740992E15;',
    '$mod.d = 9007199254740991;',
    '$mod.d = 0.0;',
    '']));
end;

procedure TTestModule.TestInteger;
begin
  StartProgram(false);
  Add([
  'const',
  '  MinInt = low(NativeInt);',
  '  MaxInt = high(NativeInt);',
  'type',
  '  {#TMyInt}TMyInt = MinInt..MaxInt;',
  'const',
  '  a = low(TMyInt)+High(TMyInt);',
  'var',
  '  i: TMyInt;',
  'begin',
  '  i:=-MinInt;',
  '  i:=default(TMyInt);',
  '  i:=low(i)+high(i);',
  '']);
  ConvertProgram;
  CheckSource('TestIntegerRange',
    LinesToStr([
    'this.MinInt = -9007199254740991;',
    'this.MaxInt = 9007199254740991;',
    'this.a = -9007199254740991 + 9007199254740991;',
    'this.i = 0;',
    '']),
    LinesToStr([
    '$mod.i = - -9007199254740991;',
    '$mod.i = -9007199254740991;',
    '$mod.i = -9007199254740991 + 9007199254740991;',
    '']));
end;

procedure TTestModule.TestIntegerRange;
begin
  StartProgram(false);
  Add([
  'const',
  '  MinInt = -1;',
  '  MaxInt = +1;',
  'type',
  '  {#TMyInt}TMyInt = MinInt..MaxInt;',
  '  TInt2 = 1..3;',
  'const',
  '  a = low(TMyInt)+High(TMyInt);',
  '  b = low(TInt2)+High(TInt2);',
  '  s1 = [1];',
  '  s2 = [1,2];',
  '  s3 = [1..3];',
  '  s4 = [low(shortint)..high(shortint)];',
  '  s5 = [succ(low(shortint))..pred(high(shortint))];',
  '  s6 = 1 in s2;',
  'var',
  '  i: TMyInt;',
  '  i2: TInt2;',
  'begin',
  '  i:=i2;',
  '  i:=default(TMyInt);',
  '  if i=i2 then ;',
  '  i:=ord(i2);',
  '']);
  ConvertProgram;
  CheckSource('TestIntegerRange',
    LinesToStr([
    'this.MinInt = -1;',
    'this.MaxInt = +1;',
    'this.a = -1 + 1;',
    'this.b = 1 + 3;',
    'this.s1 = rtl.createSet(1);',
    'this.s2 = rtl.createSet(1, 2);',
    'this.s3 = rtl.createSet(null, 1, 3);',
    'this.s4 = rtl.createSet(null, -128, 127);',
    'this.s5 = rtl.createSet(null, -128 + 1, 127 - 1);',
    'this.s6 = 1 in this.s2;',
    'this.i = 0;',
    'this.i2 = 0;',
    '']),
    LinesToStr([
    '$mod.i = $mod.i2;',
    '$mod.i = -1;',
    'if ($mod.i === $mod.i2) ;',
    '$mod.i = $mod.i2;',
    '']));
end;

procedure TTestModule.TestIntegerTypecasts;
begin
  StartProgram(false);
  Add([
  'var',
  '  i: nativeint;',
  '  b: byte;',
  '  sh: shortint;',
  '  w: word;',
  '  sm: smallint;',
  '  lw: longword;',
  '  li: longint;',
  'begin',
  '  b:=byte(i);',
  '  sh:=shortint(i);',
  '  w:=word(i);',
  '  sm:=smallint(i);',
  '  lw:=longword(i);',
  '  li:=longint(i);',
  '']);
  ConvertProgram;
  CheckSource('TestIntegerTypecasts',
    LinesToStr([
    'this.i = 0;',
    'this.b = 0;',
    'this.sh = 0;',
    'this.w = 0;',
    'this.sm = 0;',
    'this.lw = 0;',
    'this.li = 0;',
    '']),
    LinesToStr([
    '$mod.b = $mod.i & 255;',
    '$mod.sh = (($mod.i & 255) << 24) >> 24;',
    '$mod.w = $mod.i & 65535;',
    '$mod.sm = (($mod.i & 65535) << 16) >> 16;',
    '$mod.lw = $mod.i >>> 0;',
    '$mod.li = $mod.i & 0xFFFFFFFF;',
    '']));
end;

procedure TTestModule.TestInteger_BitwiseShrNativeInt;
begin
  StartProgram(false);
  Add([
  'var',
  '  i,j: nativeint;',
  'begin',
  '  i:=i shr 0;',
  '  i:=i shr 1;',
  '  i:=i shr 3;',
  '  i:=i shr 54;',
  '  i:=j shr i;',
  '']);
  ConvertProgram;
  CheckResolverUnexpectedHints;
  CheckSource('TestInteger_BitwiseShrNativeInt',
    LinesToStr([
    'this.i = 0;',
    'this.j = 0;',
    '']),
    LinesToStr([
    '$mod.i = $mod.i;',
    '$mod.i = Math.floor($mod.i / 2);',
    '$mod.i = Math.floor($mod.i / 8);',
    '$mod.i = 0;',
    '$mod.i = rtl.shr($mod.j, $mod.i);',
    '']));
end;

procedure TTestModule.TestInteger_BitwiseShlNativeInt;
begin
  StartProgram(false);
  Add([
  'var',
  '  i: nativeint;',
  'begin',
  '  i:=i shl 0;',
  '  i:=i shl 54;',
  '  i:=123456789012 shl 1;',
  '  i:=i shl 1;',
  '']);
  ConvertProgram;
  CheckResolverUnexpectedHints;
  CheckSource('TestInteger_BitwiseShrNativeInt',
    LinesToStr([
    'this.i = 0;',
    '']),
    LinesToStr([
    '$mod.i = $mod.i;',
    '$mod.i = 0;',
    '$mod.i = 246913578024;',
    '$mod.i = rtl.shl($mod.i, 1);',
    '']));
end;

procedure TTestModule.TestInteger_SystemFunc;
begin
  StartProgram(true);
  Add([
  'var',
  '  i: byte;',
  '  s: string;',
  'begin',
  '  system.inc(i);',
  '  system.str(i,s);',
  '  s:=system.str(i);',
  '  i:=system.low(i);',
  '  i:=system.high(i);',
  '  i:=system.pred(i);',
  '  i:=system.succ(i);',
  '  i:=system.ord(i);',
  '']);
  ConvertProgram;
  CheckResolverUnexpectedHints;
  CheckSource('TestInteger_SystemFunc',
    LinesToStr([
    'this.i = 0;',
    'this.s = "";',
    '']),
    LinesToStr([
    '$mod.i += 1;',
    '$mod.s = "" + $mod.i;',
    '$mod.s = "" + $mod.i;',
    '$mod.i = 0;',
    '$mod.i = 255;',
    '$mod.i = $mod.i - 1;',
    '$mod.i = $mod.i + 1;',
    '$mod.i = $mod.i;',
    '']));
end;

procedure TTestModule.TestCurrency;
begin
  StartProgram(false);
  Add([
  'type',
  '  TCoin = currency;',
  'const',
  '  a = TCoin(2.7);',
  '  b = a + TCoin(1.7);',
  '  MinSafeIntCurrency: TCoin = -92233720368.5477;',
  '  MaxSafeIntCurrency: TCoin =  92233720368.5477;',
  'var',
  '  c: TCoin = b;',
  '  i: nativeint;',
  '  d: double;',
  '  j: jsvalue;',
  'function DoIt(c: currency): currency; begin end;',
  'function GetIt(d: double): double; begin end;',
  'procedure Write(v: jsvalue); begin end;',
  'begin',
  '  c:=1.0;',
  '  c:=0.1;',
  '  c:=1.0/3.0;',
  '  c:=1/3;',
  '  c:=a;',
  '  d:=c;',
  '  c:=d;',
  '  c:=currency(c);',
  '  c:=currency(d);',
  '  d:=double(c);',
  '  c:=i;',
  '  c:=currency(i);',
  //'  i:=c;', not allowed
  '  i:=nativeint(c);',
  '  c:=c+a;',
  '  c:=-c-a;',
  '  c:=d+c;',
  '  c:=c+d;',
  '  c:=d-c;',
  '  c:=c-d;',
  '  c:=c*a;',
  '  c:=a*c;',
  '  c:=d*c;',
  '  c:=c*d;',
  '  c:=c/a;',
  '  c:=a/c;',
  '  c:=d/c;',
  '  c:=c/d;',
  '  c:=c**a;',
  '  c:=a**c;',
  '  c:=d**c;',
  '  c:=c**d;',
  '  if c=c then ;',
  '  if c=a then ;',
  '  if a=c then ;',
  '  if d=c then ;',
  '  if c=d then ;',
  '  c:=DoIt(c);',
  '  c:=DoIt(i);',
  '  c:=DoIt(d);',
  '  c:=GetIt(c);',
  '  j:=c;',
  '  Write(c);',
  '  c:=default(currency);',
  '  j:=str(c);',
  '  j:=str(c:0:3);',
  '']);
  ConvertProgram;
  CheckSource('TestCurrency',
    LinesToStr([
    'this.a = 27000;',
    'this.b = this.a + 17000;',
    'this.MinSafeIntCurrency = -92233720368.5477;',
    'this.MaxSafeIntCurrency = 92233720368.5477;',
    'this.c = this.b;',
    'this.i = 0;',
    'this.d = 0.0;',
    'this.j = undefined;',
    'this.DoIt = function (c) {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    'this.GetIt = function (d) {',
    '  var Result = 0.0;',
    '  return Result;',
    '};',
    'this.Write = function (v) {',
    '};',
    '']),
    LinesToStr([
    '$mod.c = 10000;',
    '$mod.c = 1000;',
    '$mod.c = rtl.trunc((1.0 / 3.0) * 10000);',
    '$mod.c = rtl.trunc((1 / 3) * 10000);',
    '$mod.c = $mod.a;',
    '$mod.d = $mod.c / 10000;',
    '$mod.c = rtl.trunc($mod.d * 10000);',
    '$mod.c = $mod.c;',
    '$mod.c = $mod.d * 10000;',
    '$mod.d = $mod.c / 10000;',
    '$mod.c = $mod.i * 10000;',
    '$mod.c = $mod.i * 10000;',
    '$mod.i = rtl.trunc($mod.c / 10000);',
    '$mod.c = $mod.c + $mod.a;',
    '$mod.c = -$mod.c - $mod.a;',
    '$mod.c = ($mod.d * 10000) + $mod.c;',
    '$mod.c = $mod.c + ($mod.d * 10000);',
    '$mod.c = ($mod.d * 10000) - $mod.c;',
    '$mod.c = $mod.c - ($mod.d * 10000);',
    '$mod.c = ($mod.c * $mod.a) / 10000;',
    '$mod.c = ($mod.a * $mod.c) / 10000;',
    '$mod.c = $mod.d * $mod.c;',
    '$mod.c = $mod.c * $mod.d;',
    '$mod.c = rtl.trunc(($mod.c / $mod.a) * 10000);',
    '$mod.c = rtl.trunc(($mod.a / $mod.c) * 10000);',
    '$mod.c = rtl.trunc($mod.d / $mod.c);',
    '$mod.c = rtl.trunc($mod.c / $mod.d);',
    '$mod.c = rtl.trunc(Math.pow($mod.c / 10000, $mod.a / 10000) * 10000);',
    '$mod.c = rtl.trunc(Math.pow($mod.a / 10000, $mod.c / 10000) * 10000);',
    '$mod.c = rtl.trunc(Math.pow($mod.d, $mod.c / 10000) * 10000);',
    '$mod.c = rtl.trunc(Math.pow($mod.c / 10000, $mod.d) * 10000);',
    'if ($mod.c === $mod.c) ;',
    'if ($mod.c === $mod.a) ;',
    'if ($mod.a === $mod.c) ;',
    'if (($mod.d * 10000) === $mod.c) ;',
    'if ($mod.c === ($mod.d * 10000)) ;',
    '$mod.c = $mod.DoIt($mod.c);',
    '$mod.c = $mod.DoIt($mod.i * 10000);',
    '$mod.c = $mod.DoIt($mod.d * 10000);',
    '$mod.c = rtl.trunc($mod.GetIt($mod.c / 10000) * 10000);',
    '$mod.j = $mod.c / 10000;',
    '$mod.Write($mod.c / 10000);',
    '$mod.c = 0;',
    '$mod.j = rtl.floatToStr($mod.c / 10000);',
    '$mod.j = rtl.floatToStr($mod.c / 10000, 0, 3);',
    '']));
end;

procedure TTestModule.TestForBoolDo;
begin
  StartProgram(false);
  Add([
  'var b: boolean;',
  'begin',
  '  for b:=false to true do ;',
  '  for b:=b downto false do ;',
  '  for b in boolean do ;',
  '']);
  ConvertProgram;
  CheckSource('TestForBoolDo',
    LinesToStr([ // statements
    'this.b = false;']),
    LinesToStr([ // this.$main
    'for (var $l = 0; $l <= 1; $l++) $mod.b = $l !== 0;',
    'for (var $l1 = +$mod.b; $l1 >= 0; $l1--) $mod.b = $l1 !== 0;',
    'for (var $l2 = 0; $l2 <= 1; $l2++) $mod.b = $l2 !== 0;',
    '']));
end;

procedure TTestModule.TestForIntDo;
begin
  StartProgram(false);
  Add([
  'var i: longint;',
  'begin',
  '  for i:=3 to 5 do ;',
  '  for i:=i downto 2 do ;',
  '  for i in byte do ;',
  '']);
  ConvertProgram;
  CheckSource('TestForIntDo',
    LinesToStr([ // statements
    'this.i = 0;']),
    LinesToStr([ // this.$main
    'for ($mod.i = 3; $mod.i <= 5; $mod.i++) ;',
    'for (var $l = $mod.i; $l >= 2; $l--) $mod.i = $l;',
    'for (var $l1 = 0; $l1 <= 255; $l1++) $mod.i = $l1;',
    '']));
end;

procedure TTestModule.TestForIntInDo;
begin
  StartProgram(false);
  Add([
  'type',
  '  TSetOfInt = set of byte;',
  '  TIntRg = 3..7;',
  '  TSetOfIntRg = set of TIntRg;',
  'var',
  '  i,i2: longint;',
  '  a1: array of byte;',
  '  a2: array[1..3] of byte;',
  '  soi: TSetOfInt;',
  '  soir: TSetOfIntRg;',
  '  ir: TIntRg;',
  'begin',
  '  for i in byte do ;',
  '  for i in a1 do ;',
  '  for i in a2 do ;',
  '  for i in [11..13] do ;',
  '  for i in TSetOfInt do ;',
  '  for i in TIntRg do ;',
  '  for i in soi do i2:=i;',
  '  for i in TSetOfIntRg do ;',
  '  for i in soir do ;',
  '  for ir in TIntRg do ;',
  '  for ir in TSetOfIntRg do ;',
  '  for ir in soir do ;',
  '']);
  ConvertProgram;
  CheckSource('TestForIntInDo',
    LinesToStr([ // statements
    'this.i = 0;',
    'this.i2 = 0;',
    'this.a1 = [];',
    'this.a2 = rtl.arraySetLength(null, 0, 3);',
    'this.soi = {};',
    'this.soir = {};',
    'this.ir = 0;',
    '']),
    LinesToStr([ // this.$main
    'for (var $l = 0; $l <= 255; $l++) $mod.i = $l;',
    'for (var $in = $mod.a1, $l1 = 0, $end = rtl.length($in) - 1; $l1 <= $end; $l1++) $mod.i = $in[$l1];',
    'for (var $in1 = $mod.a2, $l2 = 0, $end1 = rtl.length($in1) - 1; $l2 <= $end1; $l2++) $mod.i = $in1[$l2];',
    'for (var $l3 = 11; $l3 <= 13; $l3++) $mod.i = $l3;',
    'for (var $l4 = 0; $l4 <= 255; $l4++) $mod.i = $l4;',
    'for (var $l5 = 3; $l5 <= 7; $l5++) $mod.i = $l5;',
    'for (var $l6 in $mod.soi) {',
    '  $mod.i = +$l6;',
    '  $mod.i2 = $mod.i;',
    '};',
    'for (var $l7 = 3; $l7 <= 7; $l7++) $mod.i = $l7;',
    'for (var $l8 in $mod.soir) $mod.i = +$l8;',
    'for (var $l9 = 3; $l9 <= 7; $l9++) $mod.ir = $l9;',
    'for (var $l10 = 3; $l10 <= 7; $l10++) $mod.ir = $l10;',
    'for (var $l11 in $mod.soir) $mod.ir = +$l11;',
    '']));
end;

procedure TTestModule.TestCharConst;
begin
  StartProgram(false);
  Add([
  'const',
  '  a = #$00F3;',
  '  c: char = ''1'';',
  '  wc: widechar = ''ä'';',
  'begin',
  '  c:=#0;',
  '  c:=#1;',
  '  c:=#9;',
  '  c:=#10;',
  '  c:=#13;',
  '  c:=#31;',
  '  c:=#32;',
  '  c:=#$A;',
  '  c:=#$0A;',
  '  c:=#$b;',
  '  c:=#$0b;',
  '  c:=^A;',
  '  c:=''"'';',
  '  c:=default(char);',
  '  c:=#$00E4;', // ä
  '  c:=''ä'';',
  '  c:=#$E4;', // ä
  '  c:=#$D800;', // invalid UTF-16
  '  c:=#$DFFF;', // invalid UTF-16
  '  c:=#$FFFF;', // last UCS-2
  '  c:=high(c);', // last UCS-2
  '  c:=#269;',
  '']);
  ConvertProgram;
  CheckSource('TestCharConst',
    LinesToStr([
    'this.a="ó";',
    'this.c="1";',
    'this.wc="ä";'
    ]),
    LinesToStr([
    '$mod.c="\x00";',
    '$mod.c="\x01";',
    '$mod.c="\t";',
    '$mod.c="\n";',
    '$mod.c="\r";',
    '$mod.c="\x1F";',
    '$mod.c=" ";',
    '$mod.c="\n";',
    '$mod.c="\n";',
    '$mod.c="\x0B";',
    '$mod.c="\x0B";',
    '$mod.c="\x01";',
    '$mod.c=''"'';',
    '$mod.c="\x00";',
    '$mod.c = "ä";',
    '$mod.c = "ä";',
    '$mod.c = "ä";',
    '$mod.c="\uD800";',
    '$mod.c="\uDFFF";',
    '$mod.c="\uFFFF";',
    '$mod.c="\uFFFF";',
    '$mod.c = "č";',
    '']));
end;

procedure TTestModule.TestChar_Compare;
begin
  StartProgram(false);
  Add('var');
  Add('  c: char;');
  Add('  b: boolean;');
  Add('begin');
  Add('  b:=c=''1'';');
  Add('  b:=''2''=c;');
  Add('  b:=''3''=''4'';');
  Add('  b:=c<>''5'';');
  Add('  b:=''6''<>c;');
  Add('  b:=c>''7'';');
  Add('  b:=''8''>c;');
  Add('  b:=c>=''9'';');
  Add('  b:=''A''>=c;');
  Add('  b:=c<''B'';');
  Add('  b:=''C''<c;');
  Add('  b:=c<=''D'';');
  Add('  b:=''E''<=c;');
  ConvertProgram;
  CheckSource('TestChar_Compare',
    LinesToStr([
    'this.c="";',
    'this.b = false;'
    ]),
    LinesToStr([
    '$mod.b = $mod.c === "1";',
    '$mod.b = "2" === $mod.c;',
    '$mod.b = "3" === "4";',
    '$mod.b = $mod.c !== "5";',
    '$mod.b = "6" !== $mod.c;',
    '$mod.b = $mod.c > "7";',
    '$mod.b = "8" > $mod.c;',
    '$mod.b = $mod.c >= "9";',
    '$mod.b = "A" >= $mod.c;',
    '$mod.b = $mod.c < "B";',
    '$mod.b = "C" < $mod.c;',
    '$mod.b = $mod.c <= "D";',
    '$mod.b = "E" <= $mod.c;',
    '']));
end;

procedure TTestModule.TestChar_BuiltInProcs;
begin
  StartProgram(false);
  Add([
  'var',
  '  c: char;',
  '  i: longint;',
  '  s: string;',
  'begin',
  '  i:=ord(c);',
  '  i:=ord(s[i]);',
  '  c:=chr(i);',
  '  c:=pred(c);',
  '  c:=succ(c);',
  '  c:=low(c);',
  '  c:=high(c);',
  '  i:=byte(c);',
  '  i:=word(c);',
  '  i:=longint(c);',
  '']);
  ConvertProgram;
  CheckSource('TestChar_BuiltInProcs',
    LinesToStr([
    'this.c = "";',
    'this.i = 0;',
    'this.s = "";'
    ]),
    LinesToStr([
    '$mod.i = $mod.c.charCodeAt();',
    '$mod.i = $mod.s.charCodeAt($mod.i-1);',
    '$mod.c = String.fromCharCode($mod.i);',
    '$mod.c = String.fromCharCode($mod.c.charCodeAt() - 1);',
    '$mod.c = String.fromCharCode($mod.c.charCodeAt() + 1);',
    '$mod.c = "\x00";',
    '$mod.c = "\uFFFF";',
    '$mod.i = $mod.c.charCodeAt() & 255;',
    '$mod.i = $mod.c.charCodeAt();',
    '$mod.i = $mod.c.charCodeAt() & 0xFFFFFFFF;',
    '']));
end;

procedure TTestModule.TestStringConst;
begin
  StartProgram(false);
  Add([
  '{$H+}',
  'const',
  '  a = #$00F3#$017C;', // first <256, then >=256
  '  b = string(''a'');',
  '  c = string(''ä'');',
  '  d = UnicodeString(''b'');',
  '  e = UnicodeString(''ö'');',
  'var',
  '  s: string = ''abc'';',
  'begin',
  '  s:='''';',
  '  s:=#13#10;',
  '  s:=#9''foo'';',
  '  s:=#$A9;',
  '  s:=''foo''#13''bar'';',
  '  s:=''"'';',
  '  s:=''"''''"'';',
  '  s:=#$20AC;', // euro
  '  s:=#$10437;', // outside BMP
  '  s:=''abc''#$20AC;', // ascii,#
  '  s:=''ä''#$20AC;', // non ascii,#
  '  s:=#$20AC''abc'';', // #, ascii
  '  s:=#$20AC''ä'';', // #, non ascii
  '  s:=default(string);',
  '  s:=concat(s);',
  '  s:=concat(s,''a'',s);',
  '  s:=#250#269;',
  //'  s:=#$2F804;',
  // ToDo: \uD87E\uDC04 -> \u{2F804}
  '']);
  ConvertProgram;
  CheckSource('TestStringConst',
    LinesToStr([
    'this.a = "óż";',
    'this.b = "a";',
    'this.c = "ä";',
    'this.d = "b";',
    'this.e = "ö";',
    'this.s="abc";',
    '']),
    LinesToStr([
    '$mod.s="";',
    '$mod.s="\r\n";',
    '$mod.s="\tfoo";',
    '$mod.s="©";',
    '$mod.s="foo\rbar";',
    '$mod.s=''"'';',
    '$mod.s=''"\''"'';',
    '$mod.s="€";',
    '$mod.s="'#$F0#$90#$90#$B7'";',
    '$mod.s = "abc€";',
    '$mod.s = "ä€";',
    '$mod.s = "€abc";',
    '$mod.s = "€ä";',
    '$mod.s="";',
    '$mod.s = $mod.s;',
    '$mod.s = $mod.s.concat("a", $mod.s);',
    '$mod.s = "úč";',
    '']));
end;

procedure TTestModule.TestStringConst_InvalidUTF16;
begin
  StartProgram(false);
  Add([
  'const',
  '  a: char = #$D87E;',
  '  b: string = #$D87E;',
  '  c: string = #$D87E#43;',
  'begin',
  '  c:=''abc''#$D87E;',
  '  c:=#0#1#2;',
  '  c:=#127;',
  '  c:=#128;',
  '  c:=#255;',
  '  c:=#256;',
  '']);
  ConvertProgram;
  CheckSource('TestStringConst',
    LinesToStr([
    'this.a = "\uD87E";',
    'this.b = "\uD87E";',
    'this.c = "\uD87E+";',
    '']),
    LinesToStr([
    '$mod.c = "abc\uD87E";',
    '$mod.c = "\x00\x01\x02";',
    '$mod.c = "'#127'";',
    '$mod.c = "'#$c2#$80'";',
    '$mod.c = "'#$c3#$BF'";',
    '$mod.c = "'#$c4#$80'";',
    '']));
end;

procedure TTestModule.TestStringConstSurrogate;
begin
  StartProgram(false);
  Add([
  'var',
  '  s: string;',
  'begin',
  '  s:=''😊'';', // 1F60A
  '']);
  ConvertProgram;
  CheckSource('TestStringConstSurrogate',
    LinesToStr([
    'this.s="";'
    ]),
    LinesToStr([
    '$mod.s="😊";'
    ]));
end;

procedure TTestModule.TestString_Length;
begin
  StartProgram(false);
  Add('const c = ''foo'';');
  Add('var');
  Add('  s: string;');
  Add('  i: longint;');
  Add('begin');
  Add('  i:=length(s);');
  Add('  i:=length(s+s);');
  Add('  i:=length(''abc'');');
  Add('  i:=length(c);');
  ConvertProgram;
  CheckSource('TestString_Length',
    LinesToStr([
    'this.c = "foo";',
    'this.s = "";',
    'this.i = 0;',
    '']),
    LinesToStr([
    '$mod.i = $mod.s.length;',
    '$mod.i = ($mod.s+$mod.s).length;',
    '$mod.i = "abc".length;',
    '$mod.i = $mod.c.length;',
    '']));
end;

procedure TTestModule.TestString_Compare;
begin
  StartProgram(false);
  Add('var');
  Add('  s, t: string;');
  Add('  b: boolean;');
  Add('begin');
  Add('  b:=s=t;');
  Add('  b:=s<>t;');
  Add('  b:=s>t;');
  Add('  b:=s>=t;');
  Add('  b:=s<t;');
  Add('  b:=s<=t;');
  ConvertProgram;
  CheckSource('TestString_Compare',
    LinesToStr([ // statements
    'this.s = "";',
    'this.t = "";',
    'this.b =false;'
    ]),
    LinesToStr([ // this.$main
    '$mod.b = $mod.s === $mod.t;',
    '$mod.b = $mod.s !== $mod.t;',
    '$mod.b = $mod.s > $mod.t;',
    '$mod.b = $mod.s >= $mod.t;',
    '$mod.b = $mod.s < $mod.t;',
    '$mod.b = $mod.s <= $mod.t;',
    '']));
end;

procedure TTestModule.TestString_SetLength;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(var s: string);',
  'begin',
  '  SetLength(s,2);',
  'end;',
  'var s: string;',
  'begin',
  '  SetLength(s,3);',
  '']);
  ConvertProgram;
  CheckSource('TestString_SetLength',
    LinesToStr([ // statements
    'this.DoIt = function (s) {',
    '  s.set(rtl.strSetLength(s.get(), 2));',
    '};',
    'this.s = "";',
    '']),
    LinesToStr([ // this.$main
    '$mod.s = rtl.strSetLength($mod.s, 3);'
    ]));
end;

procedure TTestModule.TestString_CharAt;
begin
  StartProgram(false);
  Add([
  'var',
  '  s: string;',
  '  c: char;',
  '  b: boolean;',
  'begin',
  '  b:= s[1] = c;',
  '  b:= c = s[1];',
  '  b:= c <> s[1];',
  '  b:= c > s[1];',
  '  b:= c >= s[1];',
  '  b:= c < s[2];',
  '  b:= c <= s[1];',
  '  s[1] := c;',
  '  s[2+3] := c;']);
  ConvertProgram;
  CheckSource('TestString_CharAt',
    LinesToStr([ // statements
    'this.s = "";',
    'this.c = "";',
    'this.b = false;'
    ]),
    LinesToStr([ // this.$main
    '$mod.b = $mod.s.charAt(0) === $mod.c;',
    '$mod.b = $mod.c === $mod.s.charAt(0);',
    '$mod.b = $mod.c !== $mod.s.charAt(0);',
    '$mod.b = $mod.c > $mod.s.charAt(0);',
    '$mod.b = $mod.c >= $mod.s.charAt(0);',
    '$mod.b = $mod.c < $mod.s.charAt(1);',
    '$mod.b = $mod.c <= $mod.s.charAt(0);',
    '$mod.s = rtl.setCharAt($mod.s, 0, $mod.c);',
    '$mod.s = rtl.setCharAt($mod.s, (2 + 3) - 1, $mod.c);',
    '']));
end;

procedure TTestModule.TestStringHMinusFail;
begin
  StartProgram(false);
  Add([
  '{$H-}',
  'var s: string;',
  'begin']);
  ConvertProgram;
  CheckHint(mtWarning,nWarnIllegalCompilerDirectiveX,'Warning: test1.pp(3,6) : Illegal compiler directive "H-"');
end;

procedure TTestModule.TestStr;
begin
  StartProgram(false);
  Add('var');
  Add('  b: boolean;');
  Add('  i: longint;');
  Add('  d: double;');
  Add('  s: string;');
  Add('begin');
  Add('  str(b,s);');
  Add('  str(i,s);');
  Add('  str(d,s);');
  Add('  str(i:3,s);');
  Add('  str(d:3:2,s);');
  Add('  Str(12.456:12:1,s);');
  Add('  Str(12.456:12,s);');
  Add('  s:=str(b);');
  Add('  s:=str(i);');
  Add('  s:=str(d);');
  Add('  s:=str(i,i);');
  Add('  s:=str(i:3);');
  Add('  s:=str(d:3:2);');
  Add('  s:=str(i:4,i);');
  Add('  s:=str(i,i:5);');
  Add('  s:=str(i:4,i:5);');
  Add('  s:=str(s,s);');
  Add('  s:=str(s,''foo'');');
  ConvertProgram;
  CheckSource('TestStr',
    LinesToStr([ // statements
    'this.b = false;',
    'this.i = 0;',
    'this.d = 0.0;',
    'this.s = "";',
    '']),
    LinesToStr([ // this.$main
    '$mod.s = ""+$mod.b;',
    '$mod.s = ""+$mod.i;',
    '$mod.s = rtl.floatToStr($mod.d);',
    '$mod.s = rtl.spaceLeft(""+$mod.i,3);',
    '$mod.s = rtl.floatToStr($mod.d,3,2);',
    '$mod.s = rtl.floatToStr(12.456,12,1);',
    '$mod.s = rtl.floatToStr(12.456,12);',
    '$mod.s = ""+$mod.b;',
    '$mod.s = ""+$mod.i;',
    '$mod.s = rtl.floatToStr($mod.d);',
    '$mod.s = ""+$mod.i+$mod.i;',
    '$mod.s = rtl.spaceLeft(""+$mod.i,3);',
    '$mod.s = rtl.floatToStr($mod.d,3,2);',
    '$mod.s = rtl.spaceLeft("" + $mod.i, 4) + $mod.i;',
    '$mod.s = "" + $mod.i + rtl.spaceLeft("" + $mod.i, 5);',
    '$mod.s = rtl.spaceLeft("" + $mod.i, 4) + rtl.spaceLeft("" + $mod.i, 5);',
    '$mod.s = $mod.s + $mod.s;',
    '$mod.s = $mod.s + "foo";',
    '']));
end;

procedure TTestModule.TestBaseType_AnsiStringFail;
begin
  StartProgram(false);
  Add('var s: AnsiString');
  SetExpectedPasResolverError('identifier not found "AnsiString"',PasResolveEval.nIdentifierNotFound);
  ConvertProgram;
end;

procedure TTestModule.TestBaseType_WideStringFail;
begin
  StartProgram(false);
  Add('var s: WideString');
  SetExpectedPasResolverError('identifier not found "WideString"',PasResolveEval.nIdentifierNotFound);
  ConvertProgram;
end;

procedure TTestModule.TestBaseType_ShortStringFail;
begin
  StartProgram(false);
  Add('var s: ShortString');
  SetExpectedPasResolverError('identifier not found "ShortString"',PasResolveEval.nIdentifierNotFound);
  ConvertProgram;
end;

procedure TTestModule.TestBaseType_RawByteStringFail;
begin
  StartProgram(false);
  Add('var s: RawByteString');
  SetExpectedPasResolverError('identifier not found "RawByteString"',PasResolveEval.nIdentifierNotFound);
  ConvertProgram;
end;

procedure TTestModule.TestTypeShortstring_Fail;
begin
  StartProgram(false);
  Add('type t = string[12];');
  Add('var s: t;');
  Add('begin');
  SetExpectedPasResolverError('illegal qualifier "["',nIllegalQualifier);
  ConvertProgram;
end;

procedure TTestModule.TestCharSet_Custom;
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
  '  c:=default(TCharRg);',
  '']);
  ConvertProgram;
  CheckSource('TestCharSet_Custom',
    LinesToStr([ // statements
    'this.crg = "b";',
    'this.c = "";',
    'this.crg2 = "m";',
    'this.s = {};',
    '']),
    LinesToStr([ // this.$main
    '$mod.c = $mod.crg;',
    '$mod.crg = $mod.c;',
    '$mod.crg2 = $mod.crg;',
    'if ($mod.c === $mod.crg) ;',
    'if ($mod.crg === $mod.c) ;',
    'if ($mod.crg === $mod.crg2) ;',
    'if ($mod.c.charCodeAt() in $mod.s) ;',
    'if ($mod.crg2.charCodeAt() in $mod.s) ;',
    '$mod.c = "a";',
    '']));
end;

procedure TTestModule.TestWideChar;
begin
  StartProgram(false);
  Add([
  'procedure Fly(var c: char);',
  'begin',
  'end;',
  'procedure Run(var c: widechar);',
  'begin',
  'end;',
  'var',
  '  c: char;',
  '  wc: widechar;',
  '  w: word;',
  'begin',
  '  Fly(wc);',
  '  Run(c);',
  '  wc:=WideChar(w);',
  '  w:=ord(wc);',
  '']);
  ConvertProgram;
  CheckSource('TestWideChar_VarArg',
    LinesToStr([ // statements
    'this.Fly = function (c) {',
    '};',
    'this.Run = function (c) {',
    '};',
    'this.c = "";',
    'this.wc = "";',
    'this.w = 0;',
    '']),
    LinesToStr([ // this.$main
    '$mod.Fly({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.wc;',
    '    },',
    '  set: function (v) {',
    '      this.p.wc = v;',
    '    }',
    '});',
    '$mod.Run({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.c;',
    '    },',
    '  set: function (v) {',
    '      this.p.c = v;',
    '    }',
    '});',
    '$mod.wc = String.fromCharCode($mod.w);',
    '$mod.w = $mod.wc.charCodeAt();',
    '',
    '']));
end;

procedure TTestModule.TestForCharDo;
begin
  StartProgram(false);
  Add([
  'var c: char;',
  'begin',
  '  for c:=''a'' to ''c'' do ;',
  '  for c:=c downto ''a'' do ;',
  '  for c:=''Б'' to ''Я'' do ;',
  '']);
  ConvertProgram;
  CheckSource('TestForCharDo',
    LinesToStr([ // statements
    'this.c = "";']),
    LinesToStr([ // this.$main
    'for (var $l = 97; $l <= 99; $l++) $mod.c = String.fromCharCode($l);',
    'for (var $l1 = $mod.c.charCodeAt(); $l1 >= 97; $l1--) $mod.c = String.fromCharCode($l1);',
    'for (var $l2 = 1041; $l2 <= 1071; $l2++) $mod.c = String.fromCharCode($l2);',
    '']));
end;

procedure TTestModule.TestForCharInDo;
begin
  StartProgram(false);
  Add([
  'type',
  '  TSetOfChar = set of char;',
  '  TCharRg = ''a''..''z'';',
  '  TSetOfCharRg = set of TCharRg;',
  'const Foo = ''foo'';',
  'var',
  '  c,c2: char;',
  '  s: string;',
  '  a1: array of char;',
  '  a2: array[1..3] of char;',
  '  soc: TSetOfChar;',
  '  socr: TSetOfCharRg;',
  '  cr: TCharRg;',
  'begin',
  '  for c in foo do ;',
  '  for c in s do ;',
  '  for c in char do ;',
  '  for c in a1 do ;',
  '  for c in a2 do ;',
  '  for c in [''1''..''3''] do ;',
  '  for c in TSetOfChar do ;',
  '  for c in TCharRg do ;',
  '  for c in soc do c2:=c;',
  '  for c in TSetOfCharRg do ;',
  '  for c in socr do ;',
  '  for cr in TCharRg do ;',
  '  for cr in TSetOfCharRg do ;',
  '  for cr in socr do ;',
  '']);
  ConvertProgram;
  CheckSource('TestForCharInDo',
    LinesToStr([ // statements
    'this.Foo = "foo";',
    'this.c = "";',
    'this.c2 = "";',
    'this.s = "";',
    'this.a1 = [];',
    'this.a2 = rtl.arraySetLength(null, "", 3);',
    'this.soc = {};',
    'this.socr = {};',
    'this.cr = "a";',
    '']),
    LinesToStr([ // this.$main
    'for (var $in = $mod.Foo, $l = 0, $end = $in.length - 1; $l <= $end; $l++) $mod.c = $in.charAt($l);',
    'for (var $in1 = $mod.s, $l1 = 0, $end1 = $in1.length - 1; $l1 <= $end1; $l1++) $mod.c = $in1.charAt($l1);',
    'for (var $l2 = 0; $l2 <= 65535; $l2++) $mod.c = String.fromCharCode($l2);',
    'for (var $in2 = $mod.a1, $l3 = 0, $end2 = rtl.length($in2) - 1; $l3 <= $end2; $l3++) $mod.c = $in2[$l3];',
    'for (var $in3 = $mod.a2, $l4 = 0, $end3 = rtl.length($in3) - 1; $l4 <= $end3; $l4++) $mod.c = $in3[$l4];',
    'for (var $l5 = 49; $l5 <= 51; $l5++) $mod.c = String.fromCharCode($l5);',
    'for (var $l6 = 0; $l6 <= 65535; $l6++) $mod.c = String.fromCharCode($l6);',
    'for (var $l7 = 97; $l7 <= 122; $l7++) $mod.c = String.fromCharCode($l7);',
    'for (var $l8 in $mod.soc) {',
    '  $mod.c = String.fromCharCode($l8);',
    '  $mod.c2 = $mod.c;',
    '};',
    'for (var $l9 = 97; $l9 <= 122; $l9++) $mod.c = String.fromCharCode($l9);',
    'for (var $l10 in $mod.socr) $mod.c = String.fromCharCode($l10);',
    'for (var $l11 = 97; $l11 <= 122; $l11++) $mod.cr = String.fromCharCode($l11);',
    'for (var $l12 = 97; $l12 <= 122; $l12++) $mod.cr = String.fromCharCode($l12);',
    'for (var $l13 in $mod.socr) $mod.cr = String.fromCharCode($l13);',
    '']));
end;

procedure TTestModule.TestProcTwoArgs;
begin
  StartProgram(false);
  Add('procedure Test(a,b: longint);');
  Add('begin');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestProcTwoArgs',
    LinesToStr([ // statements
    'this.Test = function (a,b) {',
    '};'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestProc_DefaultValue;
begin
  StartProgram(false);
  Add('procedure p1(i: longint = 1);');
  Add('begin');
  Add('end;');
  Add('procedure p2(i: longint = 1; c: char = ''a'');');
  Add('begin');
  Add('end;');
  Add('procedure p3(d: double = 1.0; b: boolean = false; s: string = ''abc'');');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  p1;');
  Add('  p1();');
  Add('  p1(11);');
  Add('  p2;');
  Add('  p2();');
  Add('  p2(12);');
  Add('  p2(13,''b'');');
  Add('  p3();');
  ConvertProgram;
  CheckSource('TestProc_DefaultValue',
    LinesToStr([ // statements
    'this.p1 = function (i) {',
    '};',
    'this.p2 = function (i,c) {',
    '};',
    'this.p3 = function (d,b,s) {',
    '};'
    ]),
    LinesToStr([ // this.$main
    '  $mod.p1(1);',
    '  $mod.p1(1);',
    '  $mod.p1(11);',
    '  $mod.p2(1,"a");',
    '  $mod.p2(1,"a");',
    '  $mod.p2(12,"a");',
    '  $mod.p2(13,"b");',
    '  $mod.p3(1.0,false,"abc");'
    ]));
end;

procedure TTestModule.TestFunctionInt;
begin
  StartProgram(false);
  Add('function MyTest(Bar: longint): longint;');
  Add('begin');
  Add('  Result:=2*bar');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestFunctionInt',
    LinesToStr([ // statements
    'this.MyTest = function (Bar) {',
    '  var Result = 0;',
    '  Result = 2*Bar;',
    '  return Result;',
    '};'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestFunctionString;
begin
  StartProgram(false);
  Add('function Test(Bar: string): string;');
  Add('begin');
  Add('  Result:=bar+BAR');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestFunctionString',
    LinesToStr([ // statements
    'this.Test = function (Bar) {',
    '  var Result = "";',
    '  Result = Bar+Bar;',
    '  return Result;',
    '};'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestIfThen;
begin
  StartProgram(false);
  Add([
  'var b: boolean;',
  'begin',
  '  if b then ;',
  '  if b then else ;']);
  ConvertProgram;
  CheckSource('TestIfThen',
    LinesToStr([ // statements
    'this.b = false;',
    '']),
    LinesToStr([ // this.$main
    'if ($mod.b) ;',
    'if ($mod.b) ;',
    '']));
end;

procedure TTestModule.TestForLoop;
begin
  StartProgram(false);
  Add('var');
  Add('  vI, vJ, vN: longint;');
  Add('begin');
  Add('  VJ:=0;');
  Add('  VN:=3;');
  Add('  for VI:=1 to VN do');
  Add('  begin');
  Add('    VJ:=VJ+VI;');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestForLoop',
    LinesToStr([ // statements
    'this.vI = 0;',
    'this.vJ = 0;',
    'this.vN = 0;'
    ]),
    LinesToStr([ // this.$main
    '  $mod.vJ = 0;',
    '  $mod.vN = 3;',
    '  for (var $l = 1, $end = $mod.vN; $l <= $end; $l++) {',
    '    $mod.vI = $l;',
    '    $mod.vJ = $mod.vJ + $mod.vI;',
    '  };',
    '']));
end;

procedure TTestModule.TestForLoopInsideFunction;
begin
  StartProgram(false);
  Add('function SumNumbers(Count: longint): longint;');
  Add('var');
  Add('  vI, vJ: longint;');
  Add('begin');
  Add('  vj:=0;');
  Add('  for vi:=1 to count do');
  Add('  begin');
  Add('    vj:=vj+vi;');
  Add('  end;');
  Add('end;');
  Add('begin');
  Add('  sumnumbers(3);');
  ConvertProgram;
  CheckSource('TestForLoopInsideFunction',
    LinesToStr([ // statements
    'this.SumNumbers = function (Count) {',
    '  var Result = 0;',
    '  var vI = 0;',
    '  var vJ = 0;',
    '  vJ = 0;',
    '  for (var $l = 1, $end = Count; $l <= $end; $l++) {',
    '    vI = $l;',
    '    vJ = vJ + vI;',
    '  };',
    '  return Result;',
    '};'
    ]),
    LinesToStr([ // $mod.$main
    '  $mod.SumNumbers(3);'
    ]));
end;

procedure TTestModule.TestForLoop_ReadVarAfter;
begin
  StartProgram(false);
  Add('var');
  Add('  vI: longint;');
  Add('begin');
  Add('  for vi:=1 to 2 do ;');
  Add('  if vi=3 then ;');
  ConvertProgram;
  CheckSource('TestForLoop',
    LinesToStr([ // statements
    'this.vI = 0;'
    ]),
    LinesToStr([ // this.$main
    '  for ($mod.vI = 1; $mod.vI <= 2; $mod.vI++) ;',
    '  if ($mod.vI===3) ;'
    ]));
end;

procedure TTestModule.TestForLoop_Nested;
begin
  StartProgram(false);
  Add('function SumNumbers(Count: longint): longint;');
  Add('var');
  Add('  vI, vJ, vK: longint;');
  Add('begin');
  Add('  VK:=0;');
  Add('  for VI:=1 to count do');
  Add('  begin');
  Add('    for vj:=1 to vi do');
  Add('    begin');
  Add('      vk:=VK+VI;');
  Add('    end;');
  Add('  end;');
  Add('end;');
  Add('begin');
  Add('  sumnumbers(3);');
  ConvertProgram;
  CheckSource('TestForLoopInFunction',
    LinesToStr([ // statements
    'this.SumNumbers = function (Count) {',
    '  var Result = 0;',
    '  var vI = 0;',
    '  var vJ = 0;',
    '  var vK = 0;',
    '  vK = 0;',
    '  for (var $l = 1, $end = Count; $l <= $end; $l++) {',
    '    vI = $l;',
    '    for (var $l1 = 1, $end1 = vI; $l1 <= $end1; $l1++) {',
    '      vJ = $l1;',
    '      vK = vK + vI;',
    '    };',
    '  };',
    '  return Result;',
    '};'
    ]),
    LinesToStr([ // $mod.$main
    '  $mod.SumNumbers(3);'
    ]));
end;

procedure TTestModule.TestRepeatUntil;
begin
  StartProgram(false);
  Add('var');
  Add('  vI, vJ, vN: longint;');
  Add('begin');
  Add('  vn:=3;');
  Add('  vj:=0;');
  Add('  VI:=0;');
  Add('  repeat');
  Add('    VI:=vi+1;');
  Add('    vj:=VJ+vI;');
  Add('  until vi>=vn');
  ConvertProgram;
  CheckSource('TestRepeatUntil',
    LinesToStr([ // statements
    'this.vI = 0;',
    'this.vJ = 0;',
    'this.vN = 0;'
    ]),
    LinesToStr([ // $mod.$main
    '  $mod.vN = 3;',
    '  $mod.vJ = 0;',
    '  $mod.vI = 0;',
    '  do{',
    '    $mod.vI = $mod.vI + 1;',
    '    $mod.vJ = $mod.vJ + $mod.vI;',
    '  }while(!($mod.vI>=$mod.vN));'
    ]));
end;

procedure TTestModule.TestAsmBlock;
begin
  StartProgram(false);
  Add([
  'var',
  '  vI: longint;',
  'begin',
  '  vi:=1;',
  '  asm',
  '    if (vI===1) {',
  '      vI=2;',
  //'      console.log(''end;'');',  ToDo
  '    }',
  '    if (vI===2){ vI=3; }',
  '  end;',
  '  VI:=4;']);
  ConvertProgram;
  CheckSource('TestAsmBlock',
    LinesToStr([ // statements
    'this.vI = 0;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.vI = 1;',
    'if (vI===1) {',
    '  vI=2;',
    '}',
    'if (vI===2){ vI=3; }',
    ';',
    '$mod.vI = 4;'
    ]));
end;

procedure TTestModule.TestAsmPas_Impl;
begin
  StartUnit(false);
  Add('interface');
  Add('const cIntf: longint = 1;');
  Add('var vIntf: longint;');
  Add('implementation');
  Add('const cImpl: longint = 2;');
  Add('var vImpl: longint;');
  Add('procedure DoIt;');
  Add('const cLoc: longint = 3;');
  Add('var vLoc: longint;');
  Add('begin;');
  Add('  asm');
  //Add('    pas(vIntf)=pas(cIntf);');
  //Add('    pas(vImpl)=pas(cImpl);');
  //Add('    pas(vLoc)=pas(cLoc);');
  Add('  end;');
  Add('end;');
  ConvertUnit;
  CheckSource('TestAsmPas_Impl',
    LinesToStr([
    'var $impl = $mod.$impl;',
    'this.cIntf = 1;',
    'this.vIntf = 0;',
    '']),
    '', // this.$init
    LinesToStr([ // implementation
    '$impl.cImpl = 2;',
    '$impl.vImpl = 0;',
    'var cLoc = 3;',
    '$impl.DoIt = function () {',
    '  var vLoc = 0;',
    '};',
    '']) );
end;

procedure TTestModule.TestTryFinally;
begin
  StartProgram(false);
  Add('var i: longint;');
  Add('begin');
  Add('  try');
  Add('    i:=0; i:=2 div i;');
  Add('  finally');
  Add('    i:=3');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestTryFinally',
    LinesToStr([ // statements
    'this.i = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'try {',
    '  $mod.i = 0;',
    '  $mod.i = rtl.trunc(2 / $mod.i);',
    '} finally {',
    '  $mod.i = 3;',
    '};'
    ]));
end;

procedure TTestModule.TestTryExcept;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  Exception = class Msg: string; end;',
  '  EInvalidCast = class(Exception) end;',
  'var vI: longint;',
  'begin',
  '  try',
  '    vi:=1;',
  '  except',
  '    vi:=2',
  '  end;',
  '  try',
  '    vi:=3;',
  '  except',
  '    raise;',
  '  end;',
  '  try',
  '    VI:=4;',
  '  except',
  '    on einvalidcast do',
  '      raise;',
  '    on E: exception do',
  '      if e.msg='''' then',
  '        raise e;',
  '    else',
  '      vi:=5',
  '  end;',
  '  try',
  '    VI:=6;',
  '  except',
  '    on einvalidcast do ;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestTryExcept',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "Exception", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.Msg = "";',
    '  };',
    '});',
    'rtl.createClass(this, "EInvalidCast", this.Exception, function () {',
    '});',
    'this.vI = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'try {',
    '  $mod.vI = 1;',
    '} catch ($e) {',
    '  $mod.vI = 2;',
    '};',
    'try {',
    '  $mod.vI = 3;',
    '} catch ($e) {',
    '  throw $e;',
    '};',
    'try {',
    '  $mod.vI = 4;',
    '} catch ($e) {',
    '  if ($mod.EInvalidCast.isPrototypeOf($e)){',
    '    throw $e',
    '  } else if ($mod.Exception.isPrototypeOf($e)) {',
    '    var E = $e;',
    '    if (E.Msg === "") throw E;',
    '  } else {',
    '    $mod.vI = 5;',
    '  }',
    '};',
    'try {',
    '  $mod.vI = 6;',
    '} catch ($e) {',
    '  if ($mod.EInvalidCast.isPrototypeOf($e)){' ,
    '  } else throw $e',
    '};',
    '']));
end;

procedure TTestModule.TestTryExcept_ReservedWords;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  Exception = class',
  '    Symbol: string;',
  '  end;',
  'var &try: longint;',
  'begin',
  '  try',
  '    &try:=4;',
  '  except',
  '    on Error: exception do',
  '      if errOR.symBol='''' then',
  '        raise ERRor;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestTryExcept_ReservedWords',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "Exception", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.Symbol = "";',
    '  };',
    '});',
    'this.Try = 0;',
    '']),
    LinesToStr([ // $mod.$main
    'try {',
    '  $mod.Try = 4;',
    '} catch ($e) {',
    '  if ($mod.Exception.isPrototypeOf($e)) {',
    '    var error = $e;',
    '    if (error.Symbol === "") throw error;',
    '  } else throw $e',
    '};',
    '']));
end;

procedure TTestModule.TestIfThenRaiseElse;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  'constructor TObject.Create;',
  'begin',
  'end;',
  'var b: boolean;',
  'begin',
  '  if b then',
  '    raise TObject.Create',
  '  else',
  '    b:=false;',
  '']);
  ConvertProgram;
  CheckSource('TestIfThenRaiseElse',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '});',
    'this.b = false;',
    '']),
    LinesToStr([ // $mod.$main
    'if ($mod.b) {',
    '  throw $mod.TObject.$create("Create")}',
    ' else $mod.b = false;',
    '']));
end;

procedure TTestModule.TestCaseOf;
begin
  StartProgram(false);
  Add([
  'const e: longint; external name ''$e'';',
  'var vI: longint;',
  'begin',
  '  case vi of',
  '  1: ;',
  '  2: vi:=3;',
  '  e: ;',
  '  else',
  '    VI:=4',
  '  end;']);
  ConvertProgram;
  CheckSource('TestCaseOf',
    LinesToStr([ // statements
    'this.vI = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'var $tmp = $mod.vI;',
    'if ($tmp === 1) {}',
    'else if ($tmp === 2) {',
    '  $mod.vI = 3}',
    ' else if ($tmp === $e) {}',
    'else {',
    '  $mod.vI = 4;',
    '};'
    ]));
end;

procedure TTestModule.TestCaseOf_UseSwitch;
begin
  StartProgram(false);
  Converter.UseSwitchStatement:=true;
  Add('var Vi: longint;');
  Add('begin');
  Add('  case vi of');
  Add('  1: ;');
  Add('  2: VI:=3;');
  Add('  else');
  Add('    vi:=4');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestCaseOf_UseSwitch',
    LinesToStr([ // statements
    'this.Vi = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'switch ($mod.Vi) {',
    'case 1:',
    '  break;',
    'case 2:',
    '  $mod.Vi = 3;',
    '  break;',
    'default:',
    '  $mod.Vi = 4;',
    '};'
    ]));
end;

procedure TTestModule.TestCaseOfNoElse;
begin
  StartProgram(false);
  Add('var Vi: longint;');
  Add('begin');
  Add('  case vi of');
  Add('  1: begin vi:=2; VI:=3; end;');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestCaseOfNoElse',
    LinesToStr([ // statements
    'this.Vi = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'var $tmp = $mod.Vi;',
    'if ($tmp === 1) {',
    '  $mod.Vi = 2;',
    '  $mod.Vi = 3;',
    '};'
    ]));
end;

procedure TTestModule.TestCaseOfNoElse_UseSwitch;
begin
  StartProgram(false);
  Converter.UseSwitchStatement:=true;
  Add('var vI: longint;');
  Add('begin');
  Add('  case vi of');
  Add('  1: begin VI:=2; vi:=3; end;');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestCaseOfNoElse_UseSwitch',
    LinesToStr([ // statements
    'this.vI = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'switch ($mod.vI) {',
    'case 1:',
    '  $mod.vI = 2;',
    '  $mod.vI = 3;',
    '  break;',
    '};'
    ]));
end;

procedure TTestModule.TestCaseOfRange;
begin
  StartProgram(false);
  Add('var vI: longint;');
  Add('begin');
  Add('  case vi of');
  Add('  1..3: vi:=14;');
  Add('  4,5: vi:=16;');
  Add('  6..7,9..10: ;');
  Add('  else ;');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestCaseOfRange',
    LinesToStr([ // statements
    'this.vI = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'var $tmp = $mod.vI;',
    'if (($tmp >= 1) && ($tmp <= 3)){',
    '  $mod.vI = 14',
    '} else if (($tmp === 4) || ($tmp === 5)){',
    '  $mod.vI = 16',
    '} else if ((($tmp >= 6) && ($tmp <= 7)) || (($tmp >= 9) && ($tmp <= 10))) ;'
    ]));
end;

procedure TTestModule.TestCaseOfString;
begin
  StartProgram(false);
  Add([
  'var s,h: string;',
  'begin',
  '  case s of',
  '  ''foo'': s:=h;',
  '  ''a''..''z'': h:=s;',
  '  ''ў'', ''ё'': ;',
  '  ''Б''..''Я'': ;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestCaseOfString',
    LinesToStr([ // statements
    'this.s = "";',
    'this.h = "";',
    '']),
    LinesToStr([ // $mod.$main
    'var $tmp = $mod.s;',
    'if ($tmp === "foo") {',
    '  $mod.s = $mod.h}',
    ' else if (($tmp.length === 1) && ($tmp >= "a") && ($tmp <= "z")) {',
    '  $mod.h = $mod.s}',
    ' else if (($tmp === "ў") || ($tmp === "ё")) {}',
    ' else if (($tmp.length === 1) && ($tmp >= "Б") && ($tmp <= "Я")) ;',
    '']));
end;

procedure TTestModule.TestCaseOfChar;
begin
  StartProgram(false);
  Add([
  'var s,h: char;',
  'begin',
  '  case s of',
  '  ''a''..''z'': h:=s;',
  '  ''ä'': ;',
  '  ''ў'', ''ё'': ;',
  '  ''Б''..''Я'': ;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestCaseOfString',
    LinesToStr([ // statements
    'this.s = "";',
    'this.h = "";',
    '']),
    LinesToStr([ // $mod.$main
    'var $tmp = $mod.s;',
    'if (($tmp >= "a") && ($tmp <= "z")) {',
    '  $mod.h = $mod.s}',
    ' else if ($tmp === "ä") {}',
    ' else if (($tmp === "ў") || ($tmp === "ё")) {}',
    ' else if (($tmp >= "Б") && ($tmp <= "Я")) ;',
    '']));
end;

procedure TTestModule.TestCaseOfExternalClassConst;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TBird = class external name ''Bird''',
  '    const e: longint;',
  '  end;',
  'var vI: longint;',
  'begin',
  '  case vi of',
  '  1: vi:=3;',
  '  TBird.e: ;',
  '  end;']);
  ConvertProgram;
  CheckSource('TestCaseOfExternalClassConst',
    LinesToStr([ // statements
    'this.vI = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'var $tmp = $mod.vI;',
    'if ($tmp === 1) {',
    '  $mod.vI = 3}',
    ' else if ($tmp === Bird.e) ;'
    ]));
end;

procedure TTestModule.TestDebugger;
begin
  StartProgram(false);
  Add([
  'procedure DoIt;',
  'begin',
  '  deBugger;',
  '  DeBugger();',
  'end;',
  'begin',
  '  Debugger;']);
  ConvertProgram;
  CheckSource('TestDebugger',
    LinesToStr([ // statements
    'this.DoIt = function () {',
    '  debugger;',
    '  debugger;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    'debugger;',
    '']));
end;

procedure TTestModule.TestArray_Dynamic;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArrayInt = array of longint;',
  'var',
  '  Arr: TArrayInt;',
  '  i: longint;',
  '  b: boolean;',
  'begin',
  '  SetLength(arr,3);',
  '  arr[0]:=4;',
  '  arr[1]:=length(arr)+arr[0];',
  '  arr[i]:=5;',
  '  arr[arr[i]]:=arr[6];',
  '  i:=low(arr);',
  '  i:=high(arr);',
  '  b:=Assigned(arr);',
  '  Arr:=default(TArrayInt);']);
  ConvertProgram;
  CheckSource('TestArray_Dynamic',
    LinesToStr([ // statements
    'this.Arr = [];',
    'this.i = 0;',
    'this.b = false;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Arr = rtl.arraySetLength($mod.Arr,0,3);',
    '$mod.Arr[0] = 4;',
    '$mod.Arr[1] = rtl.length($mod.Arr) + $mod.Arr[0];',
    '$mod.Arr[$mod.i] = 5;',
    '$mod.Arr[$mod.Arr[$mod.i]] = $mod.Arr[6];',
    '$mod.i = 0;',
    '$mod.i = rtl.length($mod.Arr) - 1;',
    '$mod.b = rtl.length($mod.Arr) > 0;',
    '$mod.Arr = [];',
    '']));
end;

procedure TTestModule.TestArray_Dynamic_Nil;
begin
  StartProgram(false);
  Add('type');
  Add('  TArrayInt = array of longint;');
  Add('var');
  Add('  Arr: TArrayInt;');
  Add('procedure DoIt(const i: TArrayInt; j: TArrayInt); begin end;');
  Add('begin');
  Add('  arr:=nil;');
  Add('  if arr=nil then;');
  Add('  if nil=arr then;');
  Add('  if arr<>nil then;');
  Add('  if nil<>arr then;');
  Add('  DoIt(nil,nil);');
  ConvertProgram;
  CheckSource('TestArray_Dynamic',
    LinesToStr([ // statements
    'this.Arr = [];',
    'this.DoIt = function(i,j){',
    '};'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Arr = [];',
    'if (rtl.length($mod.Arr) === 0) ;',
    'if (rtl.length($mod.Arr) === 0) ;',
    'if (rtl.length($mod.Arr) > 0) ;',
    'if (rtl.length($mod.Arr) > 0) ;',
    '$mod.DoIt([],[]);',
    '']));
end;

procedure TTestModule.TestArray_DynMultiDimensional;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArrayInt = array of longint;',
  '  TArrayArrayInt = array of TArrayInt;',
  'var',
  '  Arr: TArrayInt;',
  '  Arr2: TArrayArrayInt;',
  '  i: longint;',
  'begin',
  '  arr2:=nil;',
  '  if arr2=nil then;',
  '  if nil=arr2 then;',
  '  i:=low(arr2);',
  '  i:=low(arr2[1]);',
  '  i:=high(arr2);',
  '  i:=high(arr2[2]);',
  '  arr2[3]:=arr;',
  '  arr2[4][5]:=i;',
  '  i:=arr2[6][7];',
  '  arr2[8,9]:=i;',
  '  i:=arr2[10,11];',
  '  SetLength(arr2,14);',
  '  SetLength(arr2[15],16);']);
  ConvertProgram;
  CheckSource('TestArray_Dynamic',
    LinesToStr([ // statements
    'this.Arr = [];',
    'this.Arr2 = [];',
    'this.i = 0;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Arr2 = [];',
    'if (rtl.length($mod.Arr2) === 0) ;',
    'if (rtl.length($mod.Arr2) === 0) ;',
    '$mod.i = 0;',
    '$mod.i = 0;',
    '$mod.i = rtl.length($mod.Arr2) - 1;',
    '$mod.i = rtl.length($mod.Arr2[2]) - 1;',
    '$mod.Arr2[3] = rtl.arrayRef($mod.Arr);',
    '$mod.Arr2[4][5] = $mod.i;',
    '$mod.i = $mod.Arr2[6][7];',
    '$mod.Arr2[8][9] = $mod.i;',
    '$mod.i = $mod.Arr2[10][11];',
    '$mod.Arr2 = rtl.arraySetLength($mod.Arr2, [], 14);',
    '$mod.Arr2[15] = rtl.arraySetLength($mod.Arr2[15], 0, 16);',
    '']));
end;

procedure TTestModule.TestArray_DynamicAssign;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArrayInt = array of longint;',
  '  TArrayArrayInt = array of TArrayInt;',
  'procedure Run(a: TArrayInt; const b: TArrayInt; constref c: TArrayInt);',
  'begin',
  'end;',
  'procedure Fly(var a: TArrayInt);',
  'begin',
  'end;',
  'var',
  '  Arr: TArrayInt;',
  '  Arr2: TArrayArrayInt;',
  'begin',
  '  arr:=nil;',
  '  arr2:=nil;',
  '  arr2[1]:=nil;',
  '  arr2[2]:=arr;',
  '  Run(arr,arr,arr);',
  '  Fly(arr);',
  '  Run(arr2[4],arr2[5],arr2[6]);',
  '  Fly(arr2[7]);',
  '']);
  ConvertProgram;
  CheckSource('TestArray_DynamicAssign',
    LinesToStr([ // statements
    'this.Run = function (a, b, c) {',
    '};',
    'this.Fly = function (a) {',
    '};',
    'this.Arr = [];',
    'this.Arr2 = [];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Arr = [];',
    '$mod.Arr2 = [];',
    '$mod.Arr2[1] = [];',
    '$mod.Arr2[2] = rtl.arrayRef($mod.Arr);',
    '$mod.Run(rtl.arrayRef($mod.Arr), $mod.Arr, $mod.Arr);',
    '$mod.Fly({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.Arr;',
    '    },',
    '  set: function (v) {',
    '      this.p.Arr = v;',
    '    }',
    '});',
    '$mod.Run(rtl.arrayRef($mod.Arr2[4]), $mod.Arr2[5], $mod.Arr2[6]);',
    '$mod.Fly({',
    '  a: 7,',
    '  p: $mod.Arr2,',
    '  get: function () {',
    '      return this.p[this.a];',
    '    },',
    '  set: function (v) {',
    '      this.p[this.a] = v;',
    '    }',
    '});',
    '']));
end;

procedure TTestModule.TestArray_StaticInt;
begin
  StartProgram(false);
  Add('type');
  Add('  TArrayInt = array[2..4] of longint;');
  Add('var');
  Add('  Arr: TArrayInt;');
  Add('  Arr2: TArrayInt = (5,6,7);');
  Add('  i: longint;');
  Add('  b: boolean;');
  Add('begin');
  Add('  arr[2]:=4;');
  Add('  arr[3]:=arr[2]+arr[3];');
  Add('  arr[i]:=5;');
  Add('  arr[arr[i]]:=arr[high(arr)];');
  Add('  i:=low(arr);');
  Add('  i:=high(arr);');
  Add('  b:=arr[2]=arr[3];');
  Add('  arr:=default(TArrayInt);');
  ConvertProgram;
  CheckSource('TestArray_StaticInt',
    LinesToStr([ // statements
    'this.Arr = rtl.arraySetLength(null,0,3);',
    'this.Arr2 = [5, 6, 7];',
    'this.i = 0;',
    'this.b = false;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Arr[0] = 4;',
    '$mod.Arr[1] = $mod.Arr[0] + $mod.Arr[1];',
    '$mod.Arr[$mod.i-2] = 5;',
    '$mod.Arr[$mod.Arr[$mod.i-2]-2] = $mod.Arr[2];',
    '$mod.i = 2;',
    '$mod.i = 4;',
    '$mod.b = $mod.Arr[0] === $mod.Arr[1];',
    '$mod.Arr = rtl.arraySetLength(null,0,3);',
    '']));
end;

procedure TTestModule.TestArray_StaticBool;
begin
  StartProgram(false);
  Add('type');
  Add('  TBools = array[boolean] of boolean;');
  Add('  TBool2 = array[true..true] of boolean;');
  Add('var');
  Add('  Arr: TBools;');
  Add('  Arr2: TBool2;');
  Add('  Arr3: TBools = (true,false);');
  Add('  b: boolean;');
  Add('begin');
  Add('  b:=low(arr);');
  Add('  b:=high(arr);');
  Add('  arr[true]:=false;');
  Add('  arr[false]:=arr[b] or arr[true];');
  Add('  arr[b]:=true;');
  Add('  arr[arr[b]]:=arr[high(arr)];');
  Add('  b:=arr[false]=arr[true];');
  Add('  b:=low(arr2);');
  Add('  b:=high(arr2);');
  Add('  arr2[true]:=true;');
  Add('  arr2[true]:=arr2[true] and arr2[b];');
  Add('  arr2[b]:=false;');
  ConvertProgram;
  CheckSource('TestArray_StaticBool',
    LinesToStr([ // statements
    'this.Arr = rtl.arraySetLength(null,false,2);',
    'this.Arr2 = rtl.arraySetLength(null,false,1);',
    'this.Arr3 = [true, false];',
    'this.b = false;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.b = false;',
    '$mod.b = true;',
    '$mod.Arr[1] = false;',
    '$mod.Arr[0] = $mod.Arr[+$mod.b] || $mod.Arr[1];',
    '$mod.Arr[+$mod.b] = true;',
    '$mod.Arr[+$mod.Arr[+$mod.b]] = $mod.Arr[1];',
    '$mod.b = $mod.Arr[0] === $mod.Arr[1];',
    '$mod.b = true;',
    '$mod.b = true;',
    '$mod.Arr2[0] = true;',
    '$mod.Arr2[0] = $mod.Arr2[0] && $mod.Arr2[1-$mod.b];',
    '$mod.Arr2[1-$mod.b] = false;',
    '']));
end;

procedure TTestModule.TestArray_StaticChar;
begin
  StartProgram(false);
  Add([
  'type',
  '  TChars = array[char] of char;',
  '  TChars2 = array[''a''..''z''] of char;',
  'var',
  '  Arr: TChars;',
  '  Arr2: TChars2;',
  '  Arr3: array[2..4] of char = (''p'',''a'',''s'');',
  '  Arr4: array[11..13] of char = ''pas'';',
  '  Arr5: array[21..22] of char = ''äö'';',
  '  Arr6: array[31..32] of char = ''ä''+''ö'';',
  '  c: char;',
  '  b: boolean;',
  'begin',
  '  c:=low(arr);',
  '  c:=high(arr);',
  '  arr[''B'']:=''a'';',
  '  arr[''D'']:=arr[c];',
  '  arr[c]:=arr[''d''];',
  '  arr[arr[c]]:=arr[high(arr)];',
  '  b:=arr[low(arr)]=arr[''e''];',
  '  c:=low(arr2);',
  '  c:=high(arr2);',
  '  arr2[''b'']:=''f'';',
  '  arr2[''a'']:=arr2[c];',
  '  arr2[c]:=arr2[''g''];']);
  ConvertProgram;
  CheckSource('TestArray_StaticChar',
    LinesToStr([ // statements
    'this.Arr = rtl.arraySetLength(null, "", 65536);',
    'this.Arr2 = rtl.arraySetLength(null, "", 26);',
    'this.Arr3 = ["p", "a", "s"];',
    'this.Arr4 = ["p", "a", "s"];',
    'this.Arr5 = ["ä", "ö"];',
    'this.Arr6 = ["ä", "ö"];',
    'this.c = "";',
    'this.b = false;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.c = "\x00";',
    '$mod.c = "\uFFFF";',
    '$mod.Arr[66] = "a";',
    '$mod.Arr[68] = $mod.Arr[$mod.c.charCodeAt()];',
    '$mod.Arr[$mod.c.charCodeAt()] = $mod.Arr[100];',
    '$mod.Arr[$mod.Arr[$mod.c.charCodeAt()].charCodeAt()] = $mod.Arr[65535];',
    '$mod.b = $mod.Arr[0] === $mod.Arr[101];',
    '$mod.c = "a";',
    '$mod.c = "z";',
    '$mod.Arr2[1] = "f";',
    '$mod.Arr2[0] = $mod.Arr2[$mod.c.charCodeAt() - 97];',
    '$mod.Arr2[$mod.c.charCodeAt() - 97] = $mod.Arr2[6];',
    '']));
end;

procedure TTestModule.TestArray_StaticMultiDim;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArrayInt = array[1..3] of longint;',
  '  TArrayArrayInt = array[5..6] of TArrayInt;',
  'var',
  '  Arr: TArrayInt;',
  '  Arr2: TArrayArrayInt;',
  '  Arr3: array[boolean] of TArrayInt = ((11,12,13),(21,22,23));',
  '  i: longint;',
  'begin',
  '  i:=low(arr);',
  '  i:=low(arr2);',
  '  i:=low(arr2[5]);',
  '  i:=high(arr);',
  '  i:=high(arr2);',
  '  i:=high(arr2[6]);',
  '  arr2[5]:=arr;',
  '  arr2[6][2]:=i;',
  '  i:=arr2[6][3];',
  '  arr2[6,3]:=i;',
  '  i:=arr2[5,2];',
  '  arr2:=arr2;',// clone multi dim static array
  '  arr3:=arr3;',// clone anonymous multi dim static array
  '']);
  ConvertProgram;
  CheckSource('TestArray_StaticMultiDim',
    LinesToStr([ // statements
    'this.TArrayArrayInt$clone = function (a) {',
    '  var r = [];',
    '  for (var i = 0; i < 2; i++) r.push(a[i].slice(0));',
    '  return r;',
    '};',
    'this.Arr = rtl.arraySetLength(null, 0, 3);',
    'this.Arr2 = rtl.arraySetLength(null, 0, 2, 3);',
    'this.Arr3$a$clone = function (a) {',
    '  var r = [];',
    '  for (var i = 0; i < 2; i++) r.push(a[i].slice(0));',
    '  return r;',
    '};',
    'this.Arr3 = [[11, 12, 13], [21, 22, 23]];',
    'this.i = 0;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.i = 1;',
    '$mod.i = 5;',
    '$mod.i = 1;',
    '$mod.i = 3;',
    '$mod.i = 6;',
    '$mod.i = 3;',
    '$mod.Arr2[0] = $mod.Arr.slice(0);',
    '$mod.Arr2[1][1] = $mod.i;',
    '$mod.i = $mod.Arr2[1][2];',
    '$mod.Arr2[1][2] = $mod.i;',
    '$mod.i = $mod.Arr2[0][1];',
    '$mod.Arr2 = $mod.TArrayArrayInt$clone($mod.Arr2);',
    '$mod.Arr3 = $mod.Arr3$a$clone($mod.Arr3);',
    '']));
end;

procedure TTestModule.TestArray_StaticInFunction;
begin
  StartProgram(false);
  Add([
  'const TArrayInt = 3;',
  'const TArrayArrayInt = 4;',
  'procedure DoIt;',
  'type',
  '  TArrayInt = array[1..3] of longint;',
  '  TArrayArrayInt = array[5..6] of TArrayInt;',
  'var',
  '  Arr: TArrayInt;',
  '  Arr2: TArrayArrayInt;',
  '  Arr3: array[boolean] of TArrayInt = ((11,12,13),(21,22,23));',
  '  i: longint;',
  'begin',
  '  arr2[5]:=arr;',
  '  arr2:=arr2;',// clone multi dim static array
  '  arr3:=arr3;',// clone multi dim anonymous static array
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestArray_StaticInFunction',
    LinesToStr([ // statements
    'this.TArrayInt = 3;',
    'this.TArrayArrayInt = 4;',
    'var TArrayArrayInt$1$clone = function (a) {',
    '  var r = [];',
    '  for (var i = 0; i < 2; i++) r.push(a[i].slice(0));',
    '  return r;',
    '};',
    'var Arr3$a$clone = function (a) {',
    '  var r = [];',
    '  for (var i = 0; i < 2; i++) r.push(a[i].slice(0));',
    '  return r;',
    '};',
    'this.DoIt = function () {',
    '  var Arr = rtl.arraySetLength(null, 0, 3);',
    '  var Arr2 = rtl.arraySetLength(null, 0, 2, 3);',
    '  var Arr3 = [[11, 12, 13], [21, 22, 23]];',
    '  var i = 0;',
    '  Arr2[0] = Arr.slice(0);',
    '  Arr2 = TArrayArrayInt$1$clone(Arr2);',
    '  Arr3 = Arr3$a$clone(Arr3);',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestArray_StaticMultiDimEqualNotImplemented;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArrayInt = array[1..3,1..2] of longint;',
  'var',
  '  a,b: TArrayInt;',
  'begin',
  '  if a=b then ;',
  '']);
  SetExpectedPasResolverError('compare static array is not supported',
    nXIsNotSupported);
  ConvertProgram;
end;

procedure TTestModule.TestArrayOfRecord;
begin
  StartProgram(false);
  Add([
  'type',
  '  TRec = record',
  '    Int: longint;',
  '  end;',
  '  TArrayRec = array of TRec;',
  'procedure DoIt(vd: TRec; const vc: TRec; var vv: TRec);',
  'begin',
  'end;',
  'var',
  '  Arr: TArrayRec;',
  '  r: TRec;',
  '  i: longint;',
  'begin',
  '  SetLength(arr,3);',
  '  arr[0].int:=4;',
  '  arr[1].int:=length(arr)+arr[2].int;',
  '  arr[arr[i].int].int:=arr[5].int;',
  '  arr[7]:=r;',
  '  r:=arr[8];',
  '  i:=low(arr);',
  '  i:=high(arr);',
  '  DoIt(Arr[9],Arr[10],Arr[11]);']);
  ConvertProgram;
  CheckSource('TestArrayOfRecord',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.Int = 0;',
    '  this.$eq = function (b) {',
    '    return this.Int === b.Int;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.Int = s.Int;',
    '    return this;',
    '  };',
    '});',
    'this.DoIt = function (vd, vc, vv) {',
    '};',
    'this.Arr = [];',
    'this.r = this.TRec.$new();',
    'this.i = 0;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Arr = rtl.arraySetLength($mod.Arr,$mod.TRec,3);',
    '$mod.Arr[0].Int = 4;',
    '$mod.Arr[1].Int = rtl.length($mod.Arr)+$mod.Arr[2].Int;',
    '$mod.Arr[$mod.Arr[$mod.i].Int].Int = $mod.Arr[5].Int;',
    '$mod.Arr[7].$assign($mod.r);',
    '$mod.r.$assign($mod.Arr[8]);',
    '$mod.i = 0;',
    '$mod.i = rtl.length($mod.Arr)-1;',
    '$mod.DoIt($mod.TRec.$clone($mod.Arr[9]), $mod.Arr[10], $mod.Arr[11]);',
    '']));
end;

procedure TTestModule.TestArray_StaticRecord;
begin
  StartProgram(false);
  Add([
  'type',
  '  TRec = record',
  '    Int: longint;',
  '  end;',
  '  TArrayRec = array[1..2] of TRec;',
  'var',
  '  Arr: TArrayRec;',
  'begin',
  '  arr[1].int:=length(arr)+low(arr)+high(arr);',
  '']);
  ConvertProgram;
  CheckSource('TestArray_StaticRecord',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.Int = 0;',
    '  this.$eq = function (b) {',
    '    return this.Int === b.Int;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.Int = s.Int;',
    '    return this;',
    '  };',
    '});',
    'this.TArrayRec$clone = function (a) {',
    '  var r = [];',
    '  for (var i = 0; i < 2; i++) r.push($mod.TRec.$clone(a[i]));',
    '  return r;',
    '};',
    'this.Arr = rtl.arraySetLength(null, this.TRec, 2);',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Arr[0].Int = 2 + 1 + 2;']));
end;

procedure TTestModule.TestArrayOfSet;
begin
  StartProgram(false);
  Add([
  'type',
  '  TFlag = (big,small);',
  '  TSetOfFlag = set of tflag;',
  '  TArrayFlag = array of TSetOfFlag;',
  'procedure DoIt(const a: Tarrayflag);',
  'begin',
  'end;',
  'var',
  '  f: TFlag;',
  '  s: TSetOfFlag;',
  '  Arr: TArrayFlag;',
  '  i: longint;',
  'begin',
  '  SetLength(arr,3);',
  '  arr[0]:=s;',
  '  arr[1]:=[big];',
  '  arr[2]:=[big]+s;',
  '  arr[3]:=s+[big];',
  '  arr[4]:=arr[5];',
  '  s:=arr[6];',
  '  i:=low(arr);',
  '  i:=high(arr);',
  '  DoIt(arr);',
  '  DoIt([s]);',
  '  DoIt([[],s]);',
  '  DoIt([s,[]]);',
  '']);
  ConvertProgram;
  CheckSource('TestArrayOfSet',
    LinesToStr([ // statements
    'this.TFlag = {',
    '  "0": "big",',
    '  big: 0,',
    '  "1": "small",',
    '  small: 1',
    '};',
    'this.DoIt = function (a) {',
    '};',
    'this.f = 0;',
    'this.s = {};',
    'this.Arr = [];',
    'this.i = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Arr = rtl.arraySetLength($mod.Arr, {}, 3);',
    '$mod.Arr[0] = rtl.refSet($mod.s);',
    '$mod.Arr[1] = rtl.createSet($mod.TFlag.big);',
    '$mod.Arr[2] = rtl.unionSet(rtl.createSet($mod.TFlag.big), $mod.s);',
    '$mod.Arr[3] = rtl.unionSet($mod.s, rtl.createSet($mod.TFlag.big));',
    '$mod.Arr[4] = rtl.refSet($mod.Arr[5]);',
    '$mod.s = rtl.refSet($mod.Arr[6]);',
    '$mod.i = 0;',
    '$mod.i = rtl.length($mod.Arr) - 1;',
    '$mod.DoIt($mod.Arr);',
    '$mod.DoIt([rtl.refSet($mod.s)]);',
    '$mod.DoIt([{}, rtl.refSet($mod.s)]);',
    '$mod.DoIt([rtl.refSet($mod.s), {}]);',
    '']));
end;

procedure TTestModule.TestArray_DynAsParam;
begin
  StartProgram(false);
  Add([
  'type integer = longint;',
  'type TArrInt = array of integer;',
  'procedure DoIt(vG: TArrInt; const vH: TArrInt; var vI: TArrInt);',
  'var vJ: TArrInt;',
  'begin',
  '  vg:=vg;',
  '  vj:=vh;',
  '  vi:=vi;',
  '  doit(vg,vg,vg);',
  '  doit(vh,vh,vj);',
  '  doit(vi,vi,vi);',
  '  doit(vj,vj,vj);',
  'end;',
  'var i: TArrInt;',
  'begin',
  '  doit(i,i,i);']);
  ConvertProgram;
  CheckSource('TestArray_DynAsParams',
    LinesToStr([ // statements
    'this.DoIt = function (vG,vH,vI) {',
    '  var vJ = [];',
    '  vG = rtl.arrayRef(vG);',
    '  vJ = rtl.arrayRef(vH);',
    '  vI.set(rtl.arrayRef(vI.get()));',
    '  $mod.DoIt(rtl.arrayRef(vG), vG, {',
    '    get: function () {',
    '      return vG;',
    '    },',
    '    set: function (v) {',
    '      vG = v;',
    '    }',
    '  });',
    '  $mod.DoIt(rtl.arrayRef(vH), vH, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '  $mod.DoIt(rtl.arrayRef(vI.get()), vI.get(), vI);',
    '  $mod.DoIt(rtl.arrayRef(vJ), vJ, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '};',
    'this.i = [];'
    ]),
    LinesToStr([
    '$mod.DoIt(rtl.arrayRef($mod.i),$mod.i,{',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '});'
    ]));
end;

procedure TTestModule.TestArray_StaticAsParam;
begin
  StartProgram(false);
  Add([
  'type integer = longint;',
  'type TArrInt = array[1..2] of integer;',
  'procedure DoIt(vG: TArrInt; const vH: TArrInt; var vI: TArrInt);',
  'var vJ: TArrInt;',
  'begin',
  '  vg:=vg;',
  '  vj:=vh;',
  '  vi:=vi;',
  '  doit(vg,vg,vg);',
  '  doit(vh,vh,vj);',
  '  doit(vi,vi,vi);',
  '  doit(vj,vj,vj);',
  'end;',
  'var i: TArrInt;',
  'begin',
  '  doit(i,i,i);']);
  ConvertProgram;
  CheckSource('TestArray_StaticAsParams',
    LinesToStr([ // statements
    'this.DoIt = function (vG,vH,vI) {',
    '  var vJ = rtl.arraySetLength(null, 0, 2);',
    '  vG = vG.slice(0);',
    '  vJ = vH.slice(0);',
    '  vI.set(vI.get().slice(0));',
    '  $mod.DoIt(vG.slice(0), vG, {',
    '    get: function () {',
    '      return vG;',
    '    },',
    '    set: function (v) {',
    '      vG = v;',
    '    }',
    '  });',
    '  $mod.DoIt(vH.slice(0), vH, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '  $mod.DoIt(vI.get().slice(0), vI.get(), vI);',
    '  $mod.DoIt(vJ.slice(0), vJ, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '};',
    'this.i = rtl.arraySetLength(null, 0, 2);'
    ]),
    LinesToStr([
    '$mod.DoIt($mod.i.slice(0),$mod.i,{',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '});'
    ]));
end;

procedure TTestModule.TestArrayElement_AsParams;
begin
  StartProgram(false);
  Add('type integer = longint;');
  Add('type TArrayInt = array of integer;');
  Add('procedure DoIt(vG: Integer; const vH: Integer; var vI: Integer);');
  Add('var vJ: tarrayint;');
  Add('begin');
  Add('  vi:=vi;');
  Add('  doit(vi,vi,vi);');
  Add('  doit(vj[1+1],vj[1+2],vj[1+3]);');
  Add('end;');
  Add('var a: TArrayInt;');
  Add('begin');
  Add('  doit(a[1+4],a[1+5],a[1+6]);');
  ConvertProgram;
  CheckSource('TestArrayElement_AsParams',
    LinesToStr([ // statements
    'this.DoIt = function (vG,vH,vI) {',
    '  var vJ = [];',
    '  vI.set(vI.get());',
    '  $mod.DoIt(vI.get(), vI.get(), vI);',
    '  $mod.DoIt(vJ[1+1], vJ[1+2], {',
    '    a:1+3,',
    '    p:vJ,',
    '    get: function () {',
    '      return this.p[this.a];',
    '    },',
    '    set: function (v) {',
    '      this.p[this.a] = v;',
    '    }',
    '  });',
    '};',
    'this.a = [];'
    ]),
    LinesToStr([
    '$mod.DoIt($mod.a[1+4],$mod.a[1+5],{',
    '  a: 1+6,',
    '  p: $mod.a,',
    '  get: function () {',
    '      return this.p[this.a];',
    '    },',
    '  set: function (v) {',
    '      this.p[this.a] = v;',
    '    }',
    '});'
    ]));
end;

procedure TTestModule.TestArrayElementFromFuncResult_AsParams;
begin
  StartProgram(false);
  Add('type Integer = longint;');
  Add('type TArrayInt = array of integer;');
  Add('function GetArr(vB: integer = 0): tarrayint;');
  Add('begin');
  Add('end;');
  Add('procedure DoIt(vG: integer; const vH: integer; var vI: integer);');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  doit(getarr[1+1],getarr[1+2],getarr[1+3]);');
  Add('  doit(getarr()[2+1],getarr()[2+2],getarr()[2+3]);');
  Add('  doit(getarr(7)[3+1],getarr(8)[3+2],getarr(9)[3+3]);');
  ConvertProgram;
  CheckSource('TestArrayElementFromFuncResult_AsParams',
    LinesToStr([ // statements
    'this.GetArr = function (vB) {',
    '  var Result = [];',
    '  return Result;',
    '};',
    'this.DoIt = function (vG,vH,vI) {',
    '};'
    ]),
    LinesToStr([
    '$mod.DoIt($mod.GetArr(0)[1+1],$mod.GetArr(0)[1+2],{',
    '  a: 1+3,',
    '  p: $mod.GetArr(0),',
    '  get: function () {',
    '      return this.p[this.a];',
    '    },',
    '  set: function (v) {',
    '      this.p[this.a] = v;',
    '    }',
    '});',
    '$mod.DoIt($mod.GetArr(0)[2+1],$mod.GetArr(0)[2+2],{',
    '  a: 2+3,',
    '  p: $mod.GetArr(0),',
    '  get: function () {',
    '      return this.p[this.a];',
    '    },',
    '  set: function (v) {',
    '      this.p[this.a] = v;',
    '    }',
    '});',
    '$mod.DoIt($mod.GetArr(7)[3+1],$mod.GetArr(8)[3+2],{',
    '  a: 3+3,',
    '  p: $mod.GetArr(9),',
    '  get: function () {',
    '      return this.p[this.a];',
    '    },',
    '  set: function (v) {',
    '      this.p[this.a] = v;',
    '    }',
    '});',
    '']));
end;

procedure TTestModule.TestArrayEnumTypeRange;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red,blue);',
  '  TEnumArray = array[TEnum] of longint;',
  'var',
  '  e: TEnum;',
  '  i: longint;',
  '  a: TEnumArray;',
  '  numbers: TEnumArray = (1,2);',
  '  names: array[TEnum] of string = (''red'',''blue'');',
  'begin',
  '  e:=low(a);',
  '  e:=high(a);',
  '  i:=a[red];',
  '  a[e]:=a[e];']);
  ConvertProgram;
  CheckSource('TestArrayEnumTypeRange',
    LinesToStr([ // statements
    '  this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'this.e = 0;',
    'this.i = 0;',
    'this.a = rtl.arraySetLength(null,0,2);',
    'this.numbers = [1, 2];',
    'this.names = ["red", "blue"];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.e = $mod.TEnum.red;',
    '$mod.e = $mod.TEnum.blue;',
    '$mod.i = $mod.a[$mod.TEnum.red];',
    '$mod.a[$mod.e] = $mod.a[$mod.e];',
    '']));
end;

procedure TTestModule.TestArray_SetLengthOutArg;
begin
  StartProgram(false);
  Add([
  'type TArrInt = array of longint;',
  'procedure DoIt(out a: TArrInt);',
  'begin',
  '  SetLength(a,2);',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestArray_SetLengthOutArg',
    LinesToStr([ // statements
    'this.DoIt = function (a) {',
    '  a.set(rtl.arraySetLength(a.get(), 0, 2));',
    '};',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestArray_SetLengthProperty;
begin
  StartProgram(false);
  Add('type');
  Add('  TArrInt = array of longint;');
  Add('  TObject = class');
  Add('    function GetColors: TArrInt; external name ''GetColors'';');
  Add('    procedure SetColors(const Value: TArrInt); external name ''SetColors'';');
  Add('    property Colors: TArrInt read GetColors write SetColors;');
  Add('  end;');
  Add('var Obj: TObject;');
  Add('begin');
  Add('  SetLength(Obj.Colors,2);');
  ConvertProgram;
  CheckSource('TestArray_SetLengthProperty',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.Obj = null;',
    '']),
    LinesToStr([
    '$mod.Obj.SetColors(rtl.arraySetLength($mod.Obj.GetColors(), 0, 2));',
    '']));
end;

procedure TTestModule.TestArray_SetLengthMultiDim;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArrArrInt = array of array of longint;',
  '  TArrStaInt = array of array[1..2] of longint;',
  'var',
  '  a: TArrArrInt;',
  '  b: TArrStaInt;',
  'begin',
  '  SetLength(a,2);',
  '  SetLength(a,3,4);',
  '  SetLength(b,5);',
  '']);
  ConvertProgram;
  CheckSource('TestArray_SetLengthMultiDim',
    LinesToStr([ // statements
    'this.a = [];',
    'this.b = [];',
    '']),
    LinesToStr([
    '$mod.a = rtl.arraySetLength($mod.a, [], 2);',
    '$mod.a = rtl.arraySetLength($mod.a, 0, 3, 4);',
    '$mod.b = rtl.arraySetLength($mod.b, 0, 5, "s", 2);',
    '']));
end;

procedure TTestModule.TestArray_SetLengthDynOfStatic;
begin
  StartProgram(false);
  Add([
  'type',
  '  TStaArr1 = array[1..3] of boolean;',
  //'  TStaArr2 = array[5..6] of TStaArr1;',
  '  TDynArr1StaArr1 = array of TStaArr1;',
  //'  TDynArr1StaArr2 = array of TStaArr2;',
  '  TDynArr2StaArr1 = array of TDynArr1StaArr1;',
  //'  TDynArr2StaArr2 = array of TDynArr1StaArr2;',
  'var',
  '  DynArr1StaArr1: TDynArr1StaArr1;',
  //'  DynArr1StaArr2: TDynArr1StaArr1;',
  '  DynArr2StaArr1: TDynArr2StaArr1;',
  //'  DynArr2StaArr2: TDynArr2StaArr2;',
  'begin',
  '  SetLength(DynArr1StaArr1,11);',
  '  SetLength(DynArr2StaArr1,12);',
  '  SetLength(DynArr2StaArr1[13],14);',
  '  SetLength(DynArr2StaArr1,15,16);',
  //'  SetLength(DynArr1StaArr2,21);',
  //'  SetLength(DynArr2StaArr2,22);',
  //'  SetLength(DynArr2StaArr2[23],24);',
  //'  SetLength(DynArr2StaArr2,25,26);',
  '']);
  ConvertProgram;
  CheckSource('TestArray_DynOfStatic',
    LinesToStr([ // statements
    'this.DynArr1StaArr1 = [];',
    'this.DynArr2StaArr1 = [];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DynArr1StaArr1 = rtl.arraySetLength($mod.DynArr1StaArr1, false, 11, "s", 3);',
    '$mod.DynArr2StaArr1 = rtl.arraySetLength($mod.DynArr2StaArr1, [], 12);',
    '$mod.DynArr2StaArr1[13] = rtl.arraySetLength($mod.DynArr2StaArr1[13], false, 14, "s", 3);',
    '$mod.DynArr2StaArr1 = rtl.arraySetLength(',
    '  $mod.DynArr2StaArr1,',
    '  false,',
    '  15,',
    '  16,',
    '  "s",',
    '  3',
    ');',
    '']));
end;

procedure TTestModule.TestArray_OpenArrayOfString;
begin
  StartProgram(false);
  Add('procedure DoIt(const a: array of String);');
  Add('var');
  Add('  i: longint;');
  Add('  s: string;');
  Add('begin');
  Add('  for i:=low(a) to high(a) do s:=a[length(a)-i-1];');
  Add('end;');
  Add('var s: string;');
  Add('begin');
  Add('  DoIt([]);');
  Add('  DoIt([s,''foo'','''',s+s]);');
  ConvertProgram;
  CheckSource('TestArray_OpenArrayOfString',
    LinesToStr([ // statements
    'this.DoIt = function (a) {',
    '  var i = 0;',
    '  var s = "";',
    '  for (var $l = 0, $end = rtl.length(a) - 1; $l <= $end; $l++) {',
    '    i = $l;',
    '    s = a[rtl.length(a) - i - 1];',
    '  };',
    '};',
    'this.s = "";',
    '']),
    LinesToStr([
    '$mod.DoIt([]);',
    '$mod.DoIt([$mod.s, "foo", "", $mod.s + $mod.s]);',
    '']));
end;

procedure TTestModule.TestArray_ArrayOfCharAssignString;
begin
  StartProgram(false);
  Add([
  'type TArr = array of char;',
  'var',
  '  c: char;',
  '  s: string;',
  '  a: TArr;',
  'procedure Run(const a: array of char);',
  'begin',
  '  Run(c);',
  '  Run(s);',
  'end;',
  'begin',
  '  a:=c;',
  '  a:=s;',
  '  a:=#13;',
  '  a:=''Foo'';',
  '  Run(c);',
  '  Run(s);',
  '']);
  ConvertProgram;
  CheckSource('TestArray_ArrayOfCharAssignString',
    LinesToStr([ // statements
    'this.c = "";',
    'this.s = "";',
    'this.a = [];',
    'this.Run = function (a) {',
    '  $mod.Run($mod.c.split(""));',
    '  $mod.Run($mod.s.split(""));',
    '};',
    '']),
    LinesToStr([
    '$mod.a = $mod.c.split("");',
    '$mod.a = $mod.s.split("");',
    '$mod.a = "\r".split("");',
    '$mod.a = "Foo".split("");',
    '$mod.Run($mod.c.split(""));',
    '$mod.Run($mod.s.split(""));',
    '']));
end;

procedure TTestModule.TestArray_ConstRef;
begin
  StartProgram(false);
  Add([
  'type TArr = array of word;',
  'procedure Run(constref a: TArr);',
  'begin',
  'end;',
  'procedure Fly(a: TArr; var b: TArr; out c: TArr; const d: TArr; constref e: TArr);',
  'var l: TArr;',
  'begin',
  '  Run(l);',
  '  Run(a);',
  '  Run(b);',
  '  Run(c);',
  '  Run(d);',
  '  Run(e);',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckResolverUnexpectedHints();
  CheckSource('TestArray_ConstRef',
    LinesToStr([ // statements
    'this.Run = function (a) {',
    '};',
    'this.Fly = function (a, b, c, d, e) {',
    '  var l = [];',
    '  $mod.Run(l);',
    '  $mod.Run(a);',
    '  $mod.Run(b.get());',
    '  $mod.Run(c.get());',
    '  $mod.Run(d);',
    '  $mod.Run(e);',
    '};',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestArray_Concat;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  TFlag = (big,small);',
  '  TFlags = set of TFlag;',
  '  TRec = record',
  '    i: integer;',
  '  end;',
  '  TArrInt = array of integer;',
  '  TArrRec = array of TRec;',
  '  TArrFlag = array of TFlag;',
  '  TArrSet = array of TFlags;',
  '  TArrJSValue = array of jsvalue;',
  'var',
  '  ArrInt: tarrint;',
  '  ArrRec: tarrrec;',
  '  ArrFlag: tarrflag;',
  '  ArrSet: tarrset;',
  '  ArrJSValue: tarrjsvalue;',
  'begin',
  '  arrint:=concat(arrint);',
  '  arrint:=concat(arrint,arrint);',
  '  arrint:=concat(arrint,arrint,arrint);',
  '  arrrec:=concat(arrrec);',
  '  arrrec:=concat(arrrec,arrrec);',
  '  arrrec:=concat(arrrec,arrrec,arrrec);',
  '  arrset:=concat(arrset);',
  '  arrset:=concat(arrset,arrset);',
  '  arrset:=concat(arrset,arrset,arrset);',
  '  arrjsvalue:=concat(arrjsvalue);',
  '  arrjsvalue:=concat(arrjsvalue,arrjsvalue);',
  '  arrjsvalue:=concat(arrjsvalue,arrjsvalue,arrjsvalue);',
  '  arrint:=concat([1],arrint);',
  '  arrflag:=concat([big]);',
  '  arrflag:=concat([big],arrflag);',
  '  arrflag:=concat(arrflag,[small]);',
  '']);
  ConvertProgram;
  CheckSource('TestArray_Concat',
    LinesToStr([ // statements
    'this.TFlag = {',
    '  "0": "big",',
    '  big: 0,',
    '  "1": "small",',
    '  small: 1',
    '};',
    'rtl.recNewT(this, "TRec", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return this.i === b.i;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    return this;',
    '  };',
    '});',
    'this.ArrInt = [];',
    'this.ArrRec = [];',
    'this.ArrFlag = [];',
    'this.ArrSet = [];',
    'this.ArrJSValue = [];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.ArrInt = rtl.arrayRef($mod.ArrInt);',
    '$mod.ArrInt = rtl.arrayConcatN($mod.ArrInt, $mod.ArrInt);',
    '$mod.ArrInt = rtl.arrayConcatN($mod.ArrInt, $mod.ArrInt, $mod.ArrInt);',
    '$mod.ArrRec = rtl.arrayRef($mod.ArrRec);',
    '$mod.ArrRec = rtl.arrayConcat($mod.TRec, $mod.ArrRec, $mod.ArrRec);',
    '$mod.ArrRec = rtl.arrayConcat($mod.TRec, $mod.ArrRec, $mod.ArrRec, $mod.ArrRec);',
    '$mod.ArrSet = rtl.arrayRef($mod.ArrSet);',
    '$mod.ArrSet = rtl.arrayConcat("refSet", $mod.ArrSet, $mod.ArrSet);',
    '$mod.ArrSet = rtl.arrayConcat("refSet", $mod.ArrSet, $mod.ArrSet, $mod.ArrSet);',
    '$mod.ArrJSValue = rtl.arrayRef($mod.ArrJSValue);',
    '$mod.ArrJSValue = rtl.arrayConcatN($mod.ArrJSValue, $mod.ArrJSValue);',
    '$mod.ArrJSValue = rtl.arrayConcatN($mod.ArrJSValue, $mod.ArrJSValue, $mod.ArrJSValue);',
    '$mod.ArrInt = rtl.arrayConcatN([1], $mod.ArrInt);',
    '$mod.ArrFlag = [$mod.TFlag.big];',
    '$mod.ArrFlag = rtl.arrayConcatN([$mod.TFlag.big], $mod.ArrFlag);',
    '$mod.ArrFlag = rtl.arrayConcatN($mod.ArrFlag, [$mod.TFlag.small]);',
    '']));
end;

procedure TTestModule.TestArray_Copy;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  TFlag = (big,small);',
  '  TFlags = set of TFlag;',
  '  TRec = record',
  '    i: integer;',
  '  end;',
  '  TArrInt = array of integer;',
  '  TArrRec = array of TRec;',
  '  TArrSet = array of TFlags;',
  '  TArrJSValue = array of jsvalue;',
  'var',
  '  ArrInt: tarrint;',
  '  ArrRec: tarrrec;',
  '  ArrSet: tarrset;',
  '  ArrJSValue: tarrjsvalue;',
  'begin',
  '  arrint:=copy(arrint);',
  '  arrint:=copy(arrint,2);',
  '  arrint:=copy(arrint,3,4);',
  '  arrint:=copy([1,1],1,2);',
  '  arrrec:=copy(arrrec);',
  '  arrrec:=copy(arrrec,5);',
  '  arrrec:=copy(arrrec,6,7);',
  '  arrset:=copy(arrset);',
  '  arrset:=copy(arrset,8);',
  '  arrset:=copy(arrset,9,10);',
  '  arrjsvalue:=copy(arrjsvalue);',
  '  arrjsvalue:=copy(arrjsvalue,11);',
  '  arrjsvalue:=copy(arrjsvalue,12,13);',
  '  ']);
  ConvertProgram;
  CheckSource('TestArray_Copy',
    LinesToStr([ // statements
    'this.TFlag = {',
    '  "0": "big",',
    '  big: 0,',
    '  "1": "small",',
    '  small: 1',
    '};',
    'rtl.recNewT(this, "TRec", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return this.i === b.i;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    return this;',
    '  };',
    '});',
    'this.ArrInt = [];',
    'this.ArrRec = [];',
    'this.ArrSet = [];',
    'this.ArrJSValue = [];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.ArrInt = rtl.arrayCopy(0, $mod.ArrInt, 0);',
    '$mod.ArrInt = rtl.arrayCopy(0, $mod.ArrInt, 2);',
    '$mod.ArrInt = rtl.arrayCopy(0, $mod.ArrInt, 3, 4);',
    '$mod.ArrInt = rtl.arrayCopy(0, [1, 1], 1, 2);',
    '$mod.ArrRec = rtl.arrayCopy($mod.TRec, $mod.ArrRec, 0);',
    '$mod.ArrRec = rtl.arrayCopy($mod.TRec, $mod.ArrRec, 5);',
    '$mod.ArrRec = rtl.arrayCopy($mod.TRec, $mod.ArrRec, 6, 7);',
    '$mod.ArrSet = rtl.arrayCopy("refSet", $mod.ArrSet, 0);',
    '$mod.ArrSet = rtl.arrayCopy("refSet", $mod.ArrSet, 8);',
    '$mod.ArrSet = rtl.arrayCopy("refSet", $mod.ArrSet, 9, 10);',
    '$mod.ArrJSValue = rtl.arrayCopy(0, $mod.ArrJSValue, 0);',
    '$mod.ArrJSValue = rtl.arrayCopy(0, $mod.ArrJSValue, 11);',
    '$mod.ArrJSValue = rtl.arrayCopy(0, $mod.ArrJSValue, 12, 13);',
    '']));
end;

procedure TTestModule.TestArray_InsertDelete;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  TFlag = (big,small);',
  '  TFlags = set of TFlag;',
  '  TRec = record',
  '    i: integer;',
  '  end;',
  '  TArrInt = array of integer;',
  '  TArrRec = array of TRec;',
  '  TArrSet = array of TFlags;',
  '  TArrJSValue = array of jsvalue;',
  '  TArrArrInt = array of TArrInt;',
  'var',
  '  ArrInt: tarrint;',
  '  ArrRec: tarrrec;',
  '  ArrSet: tarrset;',
  '  ArrJSValue: tarrjsvalue;',
  '  ArrArrInt: TArrArrInt;',
  'begin',
  '  Insert(1,arrint,2);',
  '  Insert(arrint[3],arrint,4);',
  '  Insert(arrrec[5],arrrec,6);',
  '  Insert(arrset[7],arrset,7);',
  '  Insert(arrjsvalue[8],arrjsvalue,9);',
  '  Insert(10,arrjsvalue,11);',
  '  Insert([23],arrarrint,22);',
  '  Delete(arrint,12,13);',
  '  Delete(arrrec,14,15);',
  '  Delete(arrset,17,18);',
  '  Delete(arrjsvalue,19,10);']);
  ConvertProgram;
  CheckSource('TestArray_InsertDelete',
    LinesToStr([ // statements
    'this.TFlag = {',
    '  "0": "big",',
    '  big: 0,',
    '  "1": "small",',
    '  small: 1',
    '};',
    'rtl.recNewT(this, "TRec", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return this.i === b.i;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    return this;',
    '  };',
    '});',
    'this.ArrInt = [];',
    'this.ArrRec = [];',
    'this.ArrSet = [];',
    'this.ArrJSValue = [];',
    'this.ArrArrInt = [];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.ArrInt = rtl.arrayInsert(1, $mod.ArrInt, 2);',
    '$mod.ArrInt = rtl.arrayInsert($mod.ArrInt[3], $mod.ArrInt, 4);',
    '$mod.ArrRec = rtl.arrayInsert($mod.ArrRec[5], $mod.ArrRec, 6);',
    '$mod.ArrSet = rtl.arrayInsert($mod.ArrSet[7], $mod.ArrSet, 7);',
    '$mod.ArrJSValue = rtl.arrayInsert($mod.ArrJSValue[8], $mod.ArrJSValue, 9);',
    '$mod.ArrJSValue = rtl.arrayInsert(10, $mod.ArrJSValue, 11);',
    '$mod.ArrArrInt = rtl.arrayInsert([23], $mod.ArrArrInt, 22);',
    '$mod.ArrInt.splice(12, 13);',
    '$mod.ArrRec.splice(14, 15);',
    '$mod.ArrSet.splice(17, 18);',
    '$mod.ArrJSValue.splice(19, 10);',
    '']));
end;

procedure TTestModule.TestArray_DynArrayConstObjFPC;
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
  '  OneStr: array of integer = (7);',
  '  Chars: array of char = ''aoc'';',
  '  Names: array of string = (''a'',''foo'');',
  '  NameCount = low(Names)+high(Names)+length(Names);',
  'var i: integer;',
  'begin',
  '  Ints:=[];',
  '  Ints:=[1,1];',
  '  Ints:=[1]+[2];',
  '  Ints:=[2];',
  '  Ints:=[]+ints;',
  '  Ints:=Ints+[];',
  '  Ints:=Ints+OneInt;',
  '  Ints:=Ints+[1,1];',
  '  Ints:=[i,i]+Ints;',
  '  Ints:=[1]+[i]+[3];',
  '']);
  ConvertProgram;
  CheckSource('TestArray_DynArrayConstObjFPC',
    LinesToStr([ // statements
    'this.Ints = [1, 2, 3];',
    'this.Aliases = ["foo", "b"];',
    'this.OneInt = [7];',
    'this.OneStr = [7];',
    'this.Chars = ["a", "o", "c"];',
    'this.Names = ["a", "foo"];',
    'this.NameCount = 0 + (rtl.length(this.Names) - 1) + rtl.length(this.Names);',
    'this.i = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Ints = [];',
    '$mod.Ints = [1, 1];',
    '$mod.Ints = rtl.arrayConcatN([1], [2]);',
    '$mod.Ints = [2];',
    '$mod.Ints = rtl.arrayConcatN([], $mod.Ints);',
    '$mod.Ints = rtl.arrayConcatN($mod.Ints, []);',
    '$mod.Ints = rtl.arrayConcatN($mod.Ints, $mod.OneInt);',
    '$mod.Ints = rtl.arrayConcatN($mod.Ints, [1, 1]);',
    '$mod.Ints = rtl.arrayConcatN([$mod.i, $mod.i], $mod.Ints);',
    '$mod.Ints = rtl.arrayConcatN(rtl.arrayConcatN([1], [$mod.i]), [3]);',
    '']));
end;

procedure TTestModule.TestArray_DynArrayConstDelphi;
begin
  StartProgram(false);
  // Note: const c = [1,1]; defines a set!
  Add([
  '{$mode delphi}',
  'type',
  '  integer = longint;',
  '  TArrInt = array of integer;',
  '  TArrStr = array of string;',
  'const',
  '  Ints: TArrInt = [1,1,2];',
  '  Aliases: TarrStr = [''foo'',''b''];',
  '  OneInt: TArrInt = [7];',
  '  OneStr: array of integer = [7]+[8];',
  '  Chars: array of char = ''aoc'';',
  '  Names: array of string = [''a'',''a''];',
  '  NameCount = low(Names)+high(Names)+length(Names);',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestArray_DynArrayConstDelphi',
    LinesToStr([ // statements
    'this.Ints = [1, 1, 2];',
    'this.Aliases = ["foo", "b"];',
    'this.OneInt = [7];',
    'this.OneStr = rtl.arrayConcatN([7],[8]);',
    'this.Chars = ["a", "o", "c"];',
    'this.Names = ["a", "a"];',
    'this.NameCount = 0 + (rtl.length(this.Names) - 1) + rtl.length(this.Names);',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestArray_ArrayLitAsParam;
begin
  StartProgram(false);
  Add([
  '{$modeswitch arrayoperators}',
  'type',
  '  integer = longint;',
  '  TArrInt = array of integer;',
  '  TArrSet = array of (red,green,blue);',
  'procedure DoOpenInt(const a: array of integer); forward;',
  'procedure DoInt(const a: TArrInt);',
  'begin',
  '  DoInt(a+[1]);',
  '  DoInt([1]+a);',
  '  DoOpenInt(a);',
  '  DoOpenInt(a+[1]);',
  '  DoOpenInt([1]+a);',
  'end;',
  'procedure DoOpenInt(const a: array of integer);',
  'begin',
  '  DoOpenInt(a+[1]);',
  '  DoOpenInt([1]+a);',
  '  DoInt(a);',
  '  DoInt(a+[1]);',
  '  DoInt([1]+a);',
  'end;',
  'procedure DoSet(const a: TArrSet);',
  'begin',
  '  DoSet(a+[red]);',
  '  DoSet([blue]+a);',
  'end;',
  'var',
  '  i: TArrInt;',
  '  s: TArrSet;',
  'begin',
  '  DoInt([1]);',
  '  DoInt([1]+[2]);',
  '  DoInt(i+[1]);',
  '  DoInt([1]+i);',
  '  DoOpenInt([1]);',
  '  DoOpenInt([1]+[2]);',
  '  DoOpenInt(i+[1]);',
  '  DoOpenInt([1]+i);',
  '  DoSet([red]);',
  '  DoSet([blue]+[green]);',
  '  DoSet(s+[blue]);',
  '  DoSet([red]+s);',
  '']);
  ConvertProgram;
  CheckSource('TestArray_ArrayLitAsParam',
    LinesToStr([ // statements
    'this.TArrSet$a = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "green",',
    '  green: 1,',
    '  "2": "blue",',
    '  blue: 2',
    '};',
    'this.DoInt = function (a) {',
    '  $mod.DoInt(rtl.arrayConcatN(a, [1]));',
    '  $mod.DoInt(rtl.arrayConcatN([1], a));',
    '  $mod.DoOpenInt(a);',
    '  $mod.DoOpenInt(rtl.arrayConcatN(a, [1]));',
    '  $mod.DoOpenInt(rtl.arrayConcatN([1], a));',
    '};',
    'this.DoOpenInt = function (a) {',
    '  $mod.DoOpenInt(rtl.arrayConcatN(a, [1]));',
    '  $mod.DoOpenInt(rtl.arrayConcatN([1], a));',
    '  $mod.DoInt(a);',
    '  $mod.DoInt(rtl.arrayConcatN(a, [1]));',
    '  $mod.DoInt(rtl.arrayConcatN([1], a));',
    '};',
    'this.DoSet = function (a) {',
    '  $mod.DoSet(rtl.arrayConcatN(a, [$mod.TArrSet$a.red]));',
    '  $mod.DoSet(rtl.arrayConcatN([$mod.TArrSet$a.blue], a));',
    '};',
    'this.i = [];',
    'this.s = [];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoInt([1]);',
    '$mod.DoInt(rtl.arrayConcatN([1], [2]));',
    '$mod.DoInt(rtl.arrayConcatN($mod.i, [1]));',
    '$mod.DoInt(rtl.arrayConcatN([1], $mod.i));',
    '$mod.DoOpenInt([1]);',
    '$mod.DoOpenInt(rtl.arrayConcatN([1], [2]));',
    '$mod.DoOpenInt(rtl.arrayConcatN($mod.i, [1]));',
    '$mod.DoOpenInt(rtl.arrayConcatN([1], $mod.i));',
    '$mod.DoSet([$mod.TArrSet$a.red]);',
    '$mod.DoSet(rtl.arrayConcatN([$mod.TArrSet$a.blue], [$mod.TArrSet$a.green]));',
    '$mod.DoSet(rtl.arrayConcatN($mod.s, [$mod.TArrSet$a.blue]));',
    '$mod.DoSet(rtl.arrayConcatN([$mod.TArrSet$a.red], $mod.s));',
    '']));
end;

procedure TTestModule.TestArray_ArrayLitMultiDimAsParam;
begin
  StartProgram(false);
  Add([
  '{$modeswitch arrayoperators}',
  'type',
  '  integer = longint;',
  '  TArrInt = array of integer;',
  '  TArrArrInt = array of TArrInt;',
  'procedure DoInt(const a: TArrArrInt);',
  'begin',
  '  DoInt(a+[[1]]);',
  '  DoInt([[1]]+a);',
  '  DoInt(a);',
  'end;',
  'var',
  '  i: TArrInt;',
  '  a: TArrArrInt;',
  'begin',
  '  a:=[[1]];',
  '  a:=[i];',
  '  a:=a+[i];',
  '  a:=[i]+a;',
  '  a:=[[1]+i];',
  '  a:=[[1]+[2]];',
  '  a:=[i+[2]];',
  '  DoInt([[1]]);',
  '  DoInt([[1]+[2],[3,4],[5]]);',
  '  DoInt([i+[1]]+a);',
  '  DoInt([i]+a);',
  '']);
  ConvertProgram;
  CheckSource('TestArray_ArrayLitMultiDimAsParam',
    LinesToStr([ // statements
    'this.DoInt = function (a) {',
    '  $mod.DoInt(rtl.arrayConcatN(a, [[1]]));',
    '  $mod.DoInt(rtl.arrayConcatN([[1]], a));',
    '  $mod.DoInt(a);',
    '};',
    'this.i = [];',
    'this.a = [];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.a = [[1]];',
    '$mod.a = [$mod.i];',
    '$mod.a = rtl.arrayConcatN($mod.a, [$mod.i]);',
    '$mod.a = rtl.arrayConcatN([$mod.i], $mod.a);',
    '$mod.a = [rtl.arrayConcatN([1], $mod.i)];',
    '$mod.a = [rtl.arrayConcatN([1], [2])];',
    '$mod.a = [rtl.arrayConcatN($mod.i, [2])];',
    '$mod.DoInt([[1]]);',
    '$mod.DoInt([rtl.arrayConcatN([1], [2]), [3, 4], [5]]);',
    '$mod.DoInt(rtl.arrayConcatN([rtl.arrayConcatN($mod.i, [1])], $mod.a));',
    '$mod.DoInt(rtl.arrayConcatN([$mod.i], $mod.a));',
    '']));
end;

procedure TTestModule.TestArray_ArrayLitStaticAsParam;
begin
  StartProgram(false);
  Add([
  '{$modeswitch arrayoperators}',
  'type',
  '  integer = longint;',
  '  TArrInt = array[1..2] of integer;',
  '  TArrArrInt = array of TArrInt;',
  'procedure DoInt(const a: TArrArrInt);',
  'begin',
  '  DoInt(a+[[1,2]]);',
  '  DoInt([[1,2]]+a);',
  '  DoInt(a);',
  'end;',
  'var',
  '  i: TArrInt;',
  '  a: TArrArrInt;',
  'begin',
  '  a:=[[1,1]];',
  '  a:=[i];',
  '  a:=a+[i];',
  '  a:=[i]+a;',
  '  DoInt([[1,1]]);',
  '  DoInt([[1,2],[3,4]]);',
  '']);
  ConvertProgram;
  CheckSource('TestArray_ArrayLitStaticAsParam',
    LinesToStr([ // statements
    'this.DoInt = function (a) {',
    '  $mod.DoInt(rtl.arrayConcatN(a, [[1, 2]]));',
    '  $mod.DoInt(rtl.arrayConcatN([[1, 2]], a));',
    '  $mod.DoInt(a);',
    '};',
    'this.i = rtl.arraySetLength(null, 0, 2);',
    'this.a = [];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.a = [[1, 1]];',
    '$mod.a = [$mod.i.slice(0)];',
    '$mod.a = rtl.arrayConcatN($mod.a, [$mod.i.slice(0)]);',
    '$mod.a = rtl.arrayConcatN([$mod.i.slice(0)], $mod.a);',
    '$mod.DoInt([[1, 1]]);',
    '$mod.DoInt([[1, 2], [3, 4]]);',
    '']));
end;

procedure TTestModule.TestArray_ForInArrOfString;
begin
  StartProgram(false);
  Add([
  'type',
  'type',
  '  TMonthNameArray = array [1..12] of string;',
  '  TMonthNames = TMonthNameArray;',
  '  TObject = class',
  '  private',
  '    function GetLongMonthNames: TMonthNames; virtual; abstract;',
  '  public',
  '    Property LongMonthNames : TMonthNames Read GetLongMonthNames;',
  '  end;',
  'var',
  '  f: TObject;',
  '  Month: string;',
  '  Names: array of string = (''a'',''foo'',''bar'');',
  '  i: longint;',
  'begin',
  '  for Month in f.LongMonthNames do ;',
  '  for Month in Names do ;',
  '  for i:=low(Names) to high(Names) do ;',
  '']);
  ConvertProgram;
  CheckSource('TestArray_ForInArrOfString',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.f = null;',
    'this.Month = "";',
    'this.Names = ["a", "foo", "bar"];',
    'this.i = 0;',
    '']),
    LinesToStr([ // $mod.$main
    'for (var $in = $mod.f.GetLongMonthNames(), $l = 0, $end = rtl.length($in) - 1; $l <= $end; $l++) $mod.Month = $in[$l];',
    'for (var $in1 = $mod.Names, $l1 = 0, $end1 = rtl.length($in1) - 1; $l1 <= $end1; $l1++) $mod.Month = $in1[$l1];',
    'for (var $l2 = 0, $end2 = rtl.length($mod.Names) - 1; $l2 <= $end2; $l2++) $mod.i = $l2;',
    '']));
end;

procedure TTestModule.TestExternalClass_TypeCastArrayToExternalClass;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSObject = class external name ''Object''',
  '  end;',
  '  TJSArray = class external name ''Array''',
  '    class function isArray(Value: JSValue) : boolean;',
  '    function concat() : TJSArray; varargs;',
  '  end;',
  'var',
  '  aObj: TJSArray;',
  '  a: array of longint;',
  '  o: TJSObject;',
  'begin',
  '  if TJSArray.isArray(65) then ;',
  '  aObj:=TJSArray(a).concat(a);',
  '  o:=TJSObject(a);']);
  ConvertProgram;
  CheckSource('TestExternalClass_TypeCastArrayToExternalClass',
    LinesToStr([ // statements
    'this.aObj = null;',
    'this.a = [];',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    'if (Array.isArray(65)) ;',
    '$mod.aObj = $mod.a.concat($mod.a);',
    '$mod.o = $mod.a;',
    '']));
end;

procedure TTestModule.TestExternalClass_TypeCastArrayFromExternalClass;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TArrStr = array of string;',
  '  TJSArray = class external name ''Array''',
  '  end;',
  '  TJSObject = class external name ''Object''',
  '  end;',
  'var',
  '  aObj: TJSArray;',
  '  a: TArrStr;',
  '  jo: TJSObject;',
  'begin',
  '  a:=TArrStr(aObj);',
  '  TArrStr(aObj)[1]:=TArrStr(aObj)[2];',
  '  a:=TarrStr(jo);',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_TypeCastArrayFromExternalClass',
    LinesToStr([ // statements
    'this.aObj = null;',
    'this.a = [];',
    'this.jo = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.a = $mod.aObj;',
    '$mod.aObj[1] = $mod.aObj[2];',
    '$mod.a = $mod.jo;',
    '']));
end;

procedure TTestModule.TestArrayOfConst_TVarRec;
begin
  StartProgram(true,[supTVarRec]);
  Add([
  'procedure Say(args: array of const);',
  'var',
  '  i: longint;',
  '  v: TVarRec;',
  'begin',
  '  for i:=low(args) to high(args) do begin',
  '    v:=args[i];',
  '    case v.vtype of',
  '    vtInteger: if length(args)=args[i].vInteger then ;',
  '    end;',
  '  end;',
  '  for v in args do ;',
  '  args:=nil;',
  '  SetLength(args,2);',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestArrayOfConst_TVarRec',
    LinesToStr([ // statements
    'this.Say = function (args) {',
    '  var i = 0;',
    '  var v = pas.system.TVarRec.$new();',
    '  for (var $l = 0, $end = rtl.length(args) - 1; $l <= $end; $l++) {',
    '    i = $l;',
    '    v.$assign(args[i]);',
    '    var $tmp = v.VType;',
    '    if ($tmp === 0) if (rtl.length(args) === args[i].VJSValue) ;',
    '  };',
    '  for (var $in = args, $l1 = 0, $end1 = rtl.length($in) - 1; $l1 <= $end1; $l1++) v = $in[$l1];',
    '  args = [];',
    '  args = rtl.arraySetLength(args, pas.system.TVarRec, 2);',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    ]));
end;

procedure TTestModule.TestArrayOfConst_PassBaseTypes;
begin
  StartProgram(true,[supTVarRec]);
  Add([
  'procedure Say(args: array of const);',
  'begin',
  '  Say(args);',
  'end;',
  'var',
  '  p: Pointer;',
  '  j: jsvalue;',
  '  c: currency;',
  'begin',
  '  Say([]);',
  '  Say([1]);',
  '  Say([''c'',''foo'',nil,true,1.3,p,j,c]);',
  '']);
  ConvertProgram;
  CheckSource('TestArrayOfConst_PassBaseTypes',
    LinesToStr([ // statements
    'this.Say = function (args) {',
    '  $mod.Say(args);',
    '};',
    'this.p = null;',
    'this.j = undefined;',
    'this.c = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Say([]);',
    '$mod.Say(pas.system.VarRecs(0, 1));',
    '$mod.Say(pas.system.VarRecs(',
    '  9,',
    '  "c",',
    '  18,',
    '  "foo",',
    '  5,',
    '  null,',
    '  1,',
    '  true,',
    '  3,',
    '  1.3,',
    '  5,',
    '  $mod.p,',
    '  20,',
    '  $mod.j,',
    '  12,',
    '  $mod.c',
    '  ));',
    '']));
end;

procedure TTestModule.TestArrayOfConst_PassObj;
begin
  StartProgram(true,[supTVarRec]);
  Add([
  '{$interfaces corba}',
  'type',
  '  TObject = class',
  '  end;',
  '  TClass = class of TObject;',
  '  IUnknown = interface',
  '  end;',
  'procedure Say(args: array of const);',
  'begin',
  'end;',
  'var',
  '  o: TObject;',
  '  c: TClass;',
  '  i: IUnknown;',
  'begin',
  '  Say([o,c,TObject]);',
  '  Say([nil,i]);',
  '']);
  ConvertProgram;
  CheckSource('TestArrayOfConst_PassObj',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-B800-000000000000}", [], null);',
    'this.Say = function (args) {',
    '};',
    'this.o = null;',
    'this.c = null;',
    'this.i = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Say(pas.system.VarRecs(',
    '  7,',
    '  $mod.o,',
    '  8,',
    '  $mod.c,',
    '  8,',
    '  $mod.TObject',
    '));',
    '$mod.Say(pas.system.VarRecs(5, null, 14, $mod.i));',
    '']));
end;

procedure TTestModule.TestRecord_Empty;
begin
  StartProgram(false);
  Add([
  'type',
  '  TRecA = record',
  '  end;',
  'var a,b: TRecA;',
  'begin',
  '  if a=b then ;']);
  ConvertProgram;
  CheckSource('TestRecord_Empty',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRecA", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '});',
    'this.a = this.TRecA.$new();',
    'this.b = this.TRecA.$new();',
    '']),
    LinesToStr([ // $mod.$main
    'if ($mod.a.$eq($mod.b)) ;'
    ]));
end;

procedure TTestModule.TestRecord_Var;
begin
  StartProgram(false);
  Add('type');
  Add('  TRecA = record');
  Add('    Bold: longint;');
  Add('  end;');
  Add('var Rec: TRecA;');
  Add('begin');
  Add('  rec.bold:=123');
  ConvertProgram;
  CheckSource('TestRecord_Var',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRecA", function () {',
    '  this.Bold = 0;',
    '  this.$eq = function (b) {',
    '    return this.Bold === b.Bold;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.Bold = s.Bold;',
    '    return this;',
    '  };',
    '});',
    'this.Rec = this.TRecA.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Rec.Bold = 123;'
    ]));
end;

procedure TTestModule.TestRecord_VarExternal;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TRecA = record',
  '    i: byte;',
  '    length_: longint external name ''length'';',
  '  end;',
  'var Rec: TRecA;',
  'begin',
  '  rec.length_ := rec.length_',
  '']);
  ConvertProgram;
  CheckSource('TestRecord_VarExternal',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRecA", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return (this.i === b.i) && (this.length === b.length);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    this.length = s.length;',
    '    return this;',
    '  };',
    '});',
    'this.Rec = this.TRecA.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Rec.length = $mod.Rec.length;'
    ]));
end;

procedure TTestModule.TestRecord_WithDo;
begin
  StartProgram(false);
  Add('type');
  Add('  TRec = record');
  Add('    vI: longint;');
  Add('  end;');
  Add('var');
  Add('  Int: longint;');
  Add('  r: TRec;');
  Add('begin');
  Add('  with r do');
  Add('    int:=vi;');
  Add('  with r do begin');
  Add('    int:=vi;');
  Add('    vi:=int;');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestWithRecordDo',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.vI = 0;',
    '  this.$eq = function (b) {',
    '    return this.vI === b.vI;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.vI = s.vI;',
    '    return this;',
    '  };',
    '});',
    'this.Int = 0;',
    'this.r = this.TRec.$new();',
    '']),
    LinesToStr([ // $mod.$main
    'var $with = $mod.r;',
    '$mod.Int = $with.vI;',
    'var $with1 = $mod.r;',
    '$mod.Int = $with1.vI;',
    '$with1.vI = $mod.Int;'
    ]));
end;

procedure TTestModule.TestRecord_Assign;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red,green);',
  '  TEnums = set of TEnum;',
  '  TSmallRec = record',
  '    N: longint;',
  '  end;',
  '  TBigRec = record',
  '    Int: longint;',
  '    D: double;',
  '    Arr: array of longint;',
  '    Arr2: array[1..2] of longint;',
  '    Small: TSmallRec;',
  '    Enums: TEnums;',
  '  end;',
  'var',
  '  r, s: TBigRec;',
  'begin',
  '  r:=s;',
  '  r:=default(TBigRec);',
  '  r:=default(s);',
  '']);
  ConvertProgram;
  CheckSource('TestRecord_Assign',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "green",',
    '  green: 1',
    '};',
    'rtl.recNewT(this, "TSmallRec", function () {',
    '  this.N = 0;',
    '  this.$eq = function (b) {',
    '    return this.N === b.N;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.N = s.N;',
    '    return this;',
    '  };',
    '});',
    'rtl.recNewT(this, "TBigRec", function () {',
    '  this.Int = 0;',
    '  this.D = 0.0;',
    '  this.$new = function () {',
    '    var r = Object.create(this);',
    '    r.Arr = [];',
    '    r.Arr2 = rtl.arraySetLength(null, 0, 2);',
    '    r.Small = $mod.TSmallRec.$new();',
    '    r.Enums = {};',
    '    return r;',
    '  };',
    '  this.$eq = function (b) {',
    '    return (this.Int === b.Int) && (this.D === b.D) && (this.Arr === b.Arr) && rtl.arrayEq(this.Arr2, b.Arr2) && this.Small.$eq(b.Small) && rtl.eqSet(this.Enums, b.Enums);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.Int = s.Int;',
    '    this.D = s.D;',
    '    this.Arr = rtl.arrayRef(s.Arr);',
    '    this.Arr2 = s.Arr2.slice(0);',
    '    this.Small.$assign(s.Small);',
    '    this.Enums = rtl.refSet(s.Enums);',
    '    return this;',
    '  };',
    '});',
    'this.r = this.TBigRec.$new();',
    'this.s = this.TBigRec.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.r.$assign($mod.s);',
    '$mod.r.$assign($mod.TBigRec.$new());',
    '$mod.r.$assign($mod.TBigRec.$new());',
    '']));
end;

procedure TTestModule.TestRecord_AsParams;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  TRecord = record',
  '    i: integer;',
  '  end;',
  'procedure DoIt(vD: TRecord; const vC: TRecord; var vV: TRecord; var U);',
  'var vL: TRecord;',
  'begin',
  '  vd:=vd;',
  '  vd.i:=vd.i;',
  '  vl:=vc;',
  '  vv:=vv;',
  '  vv.i:=vv.i;',
  '  U:=vl;',
  '  U:=vd;',
  '  U:=vc;',
  '  U:=vv;',
  '  vl:=TRecord(U);',
  '  vd:=TRecord(U);',
  '  vv:=TRecord(U);',
  '  doit(vd,vd,vd,vd);',
  '  doit(vc,vc,vl,vl);',
  '  doit(vv,vv,vv,vv);',
  '  doit(vl,vl,vl,vl);',
  '  TRecord(U).i:=3;',
  'end;',
  'var i: TRecord;',
  'begin',
  '  doit(i,i,i,i);',
  '']);
  ConvertProgram;
  CheckSource('TestRecord_AsParams',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRecord", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return this.i === b.i;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    return this;',
    '  };',
    '});',
    'this.DoIt = function (vD, vC, vV, U) {',
    '  var vL = $mod.TRecord.$new();',
    '  vD.$assign(vD);',
    '  vD.i = vD.i;',
    '  vL.$assign(vC);',
    '  vV.$assign(vV);',
    '  vV.i = vV.i;',
    '  U.$assign(vL);',
    '  U.$assign(vD);',
    '  U.$assign(vC);',
    '  U.$assign(vV);',
    '  vL.$assign(U);',
    '  vD.$assign(U);',
    '  vV.$assign(U);',
    '  $mod.DoIt($mod.TRecord.$clone(vD), vD, vD, vD);',
    '  $mod.DoIt($mod.TRecord.$clone(vC), vC, vL, vL);',
    '  $mod.DoIt($mod.TRecord.$clone(vV), vV, vV, vV);',
    '  $mod.DoIt($mod.TRecord.$clone(vL), vL, vL, vL);',
    '  U.i = 3;',
    '};',
    'this.i = this.TRecord.$new();'
    ]),
    LinesToStr([
    '$mod.DoIt($mod.TRecord.$clone($mod.i), $mod.i, $mod.i, $mod.i);',
    '']));
end;

procedure TTestModule.TestRecord_ConstRef;
begin
  StartProgram(false);
  Add([
  'type TRec = record i: word; end;',
  'procedure Run(constref a: TRec);',
  'begin',
  'end;',
  'procedure Fly(a: TRec; var b: TRec; out c: TRec; const d: TRec; constref e: TRec);',
  'var l: TRec;',
  'begin',
  '  Run(l);',
  '  Run(a);',
  '  Run(b);',
  '  Run(c);',
  '  Run(d);',
  '  Run(e);',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckResolverUnexpectedHints();
  CheckSource('TestRecord_ConstRef',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return this.i === b.i;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    return this;',
    '  };',
    '});',
    'this.Run = function (a) {',
    '};',
    'this.Fly = function (a, b, c, d, e) {',
    '  var l = $mod.TRec.$new();',
    '  $mod.Run(l);',
    '  $mod.Run(a);',
    '  $mod.Run(b);',
    '  $mod.Run(c);',
    '  $mod.Run(d);',
    '  $mod.Run(e);',
    '};',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestRecordElement_AsParams;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TRecord = record');
  Add('    i: integer;');
  Add('  end;');
  Add('procedure DoIt(vG: integer; const vH: integer; var vI: integer);');
  Add('var vJ: TRecord;');
  Add('begin');
  Add('  doit(vj.i,vj.i,vj.i);');
  Add('end;');
  Add('var r: TRecord;');
  Add('begin');
  Add('  doit(r.i,r.i,r.i);');
  ConvertProgram;
  CheckSource('TestRecordElement_AsParams',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRecord", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return this.i === b.i;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    return this;',
    '  };',
    '});',
    'this.DoIt = function (vG,vH,vI) {',
    '  var vJ = $mod.TRecord.$new();',
    '  $mod.DoIt(vJ.i, vJ.i, {',
    '    p: vJ,',
    '    get: function () {',
    '      return this.p.i;',
    '    },',
    '    set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '  });',
    '};',
    'this.r = this.TRecord.$new();'
    ]),
    LinesToStr([
    '$mod.DoIt($mod.r.i,$mod.r.i,{',
    '  p: $mod.r,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '});'
    ]));
end;

procedure TTestModule.TestRecordElementFromFuncResult_AsParams;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TRecord = record');
  Add('    i: integer;');
  Add('  end;');
  Add('function GetRec(vB: integer = 0): TRecord;');
  Add('begin');
  Add('end;');
  Add('procedure DoIt(vG: integer; const vH: integer);');
  Add('begin');
  Add('end;');
  Add('begin');
  Add('  doit(getrec.i,getrec.i);');
  Add('  doit(getrec().i,getrec().i);');
  Add('  doit(getrec(1).i,getrec(2).i);');
  ConvertProgram;
  CheckSource('TestRecordElementFromFuncResult_AsParams',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRecord", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return this.i === b.i;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    return this;',
    '  };',
    '});',
    'this.GetRec = function (vB) {',
    '  var Result = $mod.TRecord.$new();',
    '  return Result;',
    '};',
    'this.DoIt = function (vG, vH) {',
    '};',
    '']),
    LinesToStr([
    '$mod.DoIt($mod.GetRec(0).i,$mod.GetRec(0).i);',
    '$mod.DoIt($mod.GetRec(0).i,$mod.GetRec(0).i);',
    '$mod.DoIt($mod.GetRec(1).i,$mod.GetRec(2).i);',
    '']));
end;

procedure TTestModule.TestRecordElementFromWith_AsParams;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TRecord = record');
  Add('    i: integer;');
  Add('  end;');
  Add('procedure DoIt(vG: integer; const vH: integer; var vI: integer);');
  Add('begin');
  Add('end;');
  Add('var r: trecord;');
  Add('begin');
  Add('  with r do ');
  Add('    doit(i,i,i);');
  ConvertProgram;
  CheckSource('TestRecordElementFromWith_AsParams',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRecord", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return this.i === b.i;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    return this;',
    '  };',
    '});',
    'this.DoIt = function (vG,vH,vI) {',
    '};',
    'this.r = this.TRecord.$new();'
    ]),
    LinesToStr([
    'var $with = $mod.r;',
    '$mod.DoIt($with.i,$with.i,{',
    '  p: $with,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '});',
    '']));
end;

procedure TTestModule.TestRecord_Equal;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TFlag = (red,blue);');
  Add('  TFlags = set of TFlag;');
  Add('  TProc = procedure;');
  Add('  TRecord = record');
  Add('    i: integer;');
  Add('    Event: TProc;');
  Add('    f: TFlags;');
  Add('  end;');
  Add('  TNested = record');
  Add('    r: TRecord;');
  Add('  end;');
  Add('var');
  Add('  b: boolean;');
  Add('  r,s: trecord;');
  Add('begin');
  Add('  b:=r=s;');
  Add('  b:=r<>s;');
  ConvertProgram;
  CheckSource('TestRecord_Equal',
    LinesToStr([ // statements
    'this.TFlag = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'rtl.recNewT(this, "TRecord", function () {',
    '  this.i = 0;',
    '  this.Event = null;',
    '  this.$new = function () {',
    '    var r = Object.create(this);',
    '    r.f = {};',
    '    return r;',
    '  };',
    '  this.$eq = function (b) {',
    '    return (this.i === b.i) && rtl.eqCallback(this.Event, b.Event) && rtl.eqSet(this.f, b.f);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    this.Event = s.Event;',
    '    this.f = rtl.refSet(s.f);',
    '    return this;',
    '  };',
    '});',
    'rtl.recNewT(this, "TNested", function () {',
    '  this.$new = function () {',
    '    var r = Object.create(this);',
    '    r.r = $mod.TRecord.$new();',
    '    return r;',
    '  };',
    '  this.$eq = function (b) {',
    '    return this.r.$eq(b.r);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.r.$assign(s.r);',
    '    return this;',
    '  };',
    '});',
    'this.b = false;',
    'this.r = this.TRecord.$new();',
    'this.s = this.TRecord.$new();',
    '']),
    LinesToStr([
    '$mod.b = $mod.r.$eq($mod.s);',
    '$mod.b = !$mod.r.$eq($mod.s);',
    '']));
end;

procedure TTestModule.TestRecord_JSValue;
begin
  StartProgram(false);
  Add([
  'type',
  '  TRecord = record',
  '    i: longint;',
  '  end;',
  'procedure Fly(d: jsvalue; const c: jsvalue);',
  'begin',
  'end;',
  'procedure Run(d: TRecord; const c: TRecord; var v: TRecord);',
  'begin',
  '  if jsvalue(d) then ;',
  '  if jsvalue(c) then ;',
  '  if jsvalue(v) then ;',
  'end;',
  'var',
  '  Jv: jsvalue;',
  '  Rec: trecord;',
  'begin',
  '  rec:=trecord(jv);',
  '  jv:=rec;',
  '  Fly(rec,rec);',
  '  Fly(@rec,@rec);',
  '  if jsvalue(Rec) then ;',
  '  Run(trecord(jv),trecord(jv),rec);',
  '']);
  ConvertProgram;
  CheckSource('TestRecord_JSValue',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRecord", function () {',
    '  this.i = 0;',
    '  this.$eq = function (b) {',
    '    return this.i === b.i;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    return this;',
    '  };',
    '});',
    'this.Fly = function (d, c) {',
    '};',
    'this.Run = function (d, c, v) {',
    '  if (d) ;',
    '  if (c) ;',
    '  if (v) ;',
    '};',
    'this.Jv = undefined;',
    'this.Rec = this.TRecord.$new();',
    '']),
    LinesToStr([
    '$mod.Rec.$assign(rtl.getObject($mod.Jv));',
    '$mod.Jv = $mod.Rec;',
    '$mod.Fly($mod.TRecord.$clone($mod.Rec), $mod.Rec);',
    '$mod.Fly($mod.Rec, $mod.Rec);',
    'if ($mod.Rec) ;',
    '$mod.Run($mod.TRecord.$clone(rtl.getObject($mod.Jv)), rtl.getObject($mod.Jv), $mod.Rec);',
    '']));
end;

procedure TTestModule.TestRecord_VariantFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TRec = record',
  '    case word of',
  '    0: (b0, b1: Byte);',
  '    1: (i: word);',
  '  end;',
  'begin']);
  SetExpectedPasResolverError('variant record is not supported',
    nXIsNotSupported);
  ConvertProgram;
end;

procedure TTestModule.TestRecord_FieldArray;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArrInt = array[3..4] of longint;',
  '  TArrArrInt = array[3..4] of longint;',
  '  TRec = record',
  '    a: array of longint;',
  '    s: array[1..2] of longint;',
  '    m: array[1..2,3..4] of longint;',
  '    o: TArrArrInt;',
  '  end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRecord_FieldArray',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.$new = function () {',
    '    var r = Object.create(this);',
    '    r.a = [];',
    '    r.s = rtl.arraySetLength(null, 0, 2);',
    '    r.m = rtl.arraySetLength(null, 0, 2, 2);',
    '    r.o = rtl.arraySetLength(null, 0, 2);',
    '    return r;',
    '  };',
    '  this.$eq = function (b) {',
    '    return (this.a === b.a) && rtl.arrayEq(this.s, b.s) && rtl.arrayEq(this.m, b.m) && rtl.arrayEq(this.o, b.o);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.a = rtl.arrayRef(s.a);',
    '    this.s = s.s.slice(0);',
    '    this.m = s.m.slice(0);',
    '    this.o = s.o.slice(0);',
    '    return this;',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRecord_Const;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArrInt = array[3..4] of longint;',
  '  TPoint = record x,y: longint; end;',
  '  TRec = record',
  '    i: longint;',
  '    a: array of longint;',
  '    s: array[1..2] of longint;',
  '    m: array[1..2,3..4] of longint;',
  '    p: TPoint;',
  '  end;',
  '  TPoints = array of TPoint;',
  'const',
  '  r: TRec = (',
  '    i:1;',
  '    a:(2,3);',
  '    s:(4,5);',
  '    m:( (11,12), (13,14) );',
  '    p: (x:21; y:22)',
  '  );',
  '  p: TPoints = ( (x:1;y:2), (x:3;y:4) );',
  'begin']);
  ConvertProgram;
  CheckSource('TestRecord_Const',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TPoint", function () {',
    '  this.x = 0;',
    '  this.y = 0;',
    '  this.$eq = function (b) {',
    '    return (this.x === b.x) && (this.y === b.y);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    this.y = s.y;',
    '    return this;',
    '  };',
    '});',
    'rtl.recNewT(this, "TRec", function () {',
    '  this.i = 0;',
    '  this.$new = function () {',
    '    var r = Object.create(this);',
    '    r.a = [];',
    '    r.s = rtl.arraySetLength(null, 0, 2);',
    '    r.m = rtl.arraySetLength(null, 0, 2, 2);',
    '    r.p = $mod.TPoint.$new();',
    '    return r;',
    '  };',
    '  this.$eq = function (b) {',
    '    return (this.i === b.i) && (this.a === b.a) && rtl.arrayEq(this.s, b.s) && rtl.arrayEq(this.m, b.m) && this.p.$eq(b.p);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    this.a = rtl.arrayRef(s.a);',
    '    this.s = s.s.slice(0);',
    '    this.m = s.m.slice(0);',
    '    this.p.$assign(s.p);',
    '    return this;',
    '  };',
    '});',
    'this.r = this.TRec.$clone({',
    '  i: 1,',
    '  a: [2, 3],',
    '  s: [4, 5],',
    '  m: [[11, 12], [13, 14]],',
    '  p: this.TPoint.$clone({',
    '      x: 21,',
    '      y: 22',
    '    })',
    '});',
    'this.p = [this.TPoint.$clone({',
    '  x: 1,',
    '  y: 2',
    '}), this.TPoint.$clone({',
    '  x: 3,',
    '  y: 4',
    '})];',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRecord_TypecastFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TPoint = record x,y: longint; end;',
  '  TRec = record l: longint end;',
  'var p: TPoint;',
  'begin',
  '  if TRec(p).l=2 then ;']);
  SetExpectedPasResolverError('Illegal type conversion: "TPoint" to "record TRec"',
    nIllegalTypeConversionTo);
  ConvertProgram;
end;

procedure TTestModule.TestRecord_InFunction;
begin
  StartProgram(false);
  Add([
  'var TPoint: longint = 3;',
  'procedure DoIt;',
  'type',
  '  TPoint = record x,y: longint; end;',
  '  TPoints = array of TPoint;',
  'var',
  '  r: TPoint;',
  '  p: TPoints;',
  'begin',
  '  SetLength(p,2);',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRecord_InFunction',
    LinesToStr([ // statements
    'this.TPoint = 3;',
    'var TPoint$1 = rtl.recNewT(null, "", function () {',
    '  this.x = 0;',
    '  this.y = 0;',
    '  this.$eq = function (b) {',
    '    return (this.x === b.x) && (this.y === b.y);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    this.y = s.y;',
    '    return this;',
    '  };',
    '});',
    'this.DoIt = function () {',
    '  var r = TPoint$1.$new();',
    '  var p = [];',
    '  p = rtl.arraySetLength(p, TPoint$1, 2);',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRecord_AnonymousFail;
begin
  StartProgram(false);
  Add([
  'var',
  '  r: record x: word end;',
  'begin']);
  SetExpectedPasResolverError('not yet implemented: :TPasRecordType [20190408224556] "anonymous record type"',
    nNotYetImplemented);
  ConvertProgram;
end;

procedure TTestModule.TestAdvRecord_Function;
begin
  StartProgram(false);
  Parser.Options:=Parser.Options+[po_cassignments];
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TPoint = record',
  '    x,y: word;',
  '    function Add(const apt: TPoint): TPoint;',
  '  end;',
  'function TPoint.Add(const apt: TPoint): TPoint;',
  'begin',
  '  Result:=Self;',
  '  Result.x+=apt.x;',
  '  Result.y:=Result.y+apt.y;',
  '  Self:=apt;',
  'end;',
  'var p,q: TPoint;',
  'begin',
  '  p.add(q);',
  '  p:=default(TPoint);',
  '  p:=q;',
  '']);
  ConvertProgram;
  CheckSource('TestAdvRecord_Function',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TPoint", function () {',
    '  this.x = 0;',
    '  this.y = 0;',
    '  this.$eq = function (b) {',
    '    return (this.x === b.x) && (this.y === b.y);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    this.y = s.y;',
    '    return this;',
    '  };',
    '  this.Add = function (apt) {',
    '    var Result = $mod.TPoint.$new();',
    '    Result.$assign(this);',
    '    Result.x += apt.x;',
    '    Result.y = Result.y + apt.y;',
    '    this.$assign(apt);',
    '    return Result;',
    '  };',
    '});',
    'this.p = this.TPoint.$new();',
    'this.q = this.TPoint.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p.Add($mod.q);',
    '$mod.p.$assign($mod.TPoint.$new());',
    '$mod.p.$assign($mod.q);',
    '']));
end;

procedure TTestModule.TestAdvRecord_Property;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TPoint = record',
  '    x,y: word;',
  '  strict private',
  '    function GetSize: longword;',
  '    procedure SetSize(Value: longword);',
  '  public',
  '    property Size: longword read GetSize write SetSize;',
  '    property Left: word read x write y;',
  '  end;',
  'procedure SetSize(Value: longword); begin end;',// check auto rename
  'function TPoint.GetSize: longword;',
  'begin',
  '  x:=y;',
  '  Size:=Size;',
  '  Left:=Left;',
  'end;',
  'procedure TPoint.SetSize(Value: longword);',
  'begin',
  'end;',
  'var p,q: TPoint;',
  'begin',
  '  p.Size:=q.Size;',
  '  p.Left:=q.Left;',
  '']);
  ConvertProgram;
  CheckSource('TestAdvRecord_Property',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TPoint", function () {',
    '  this.x = 0;',
    '  this.y = 0;',
    '  this.$eq = function (b) {',
    '    return (this.x === b.x) && (this.y === b.y);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    this.y = s.y;',
    '    return this;',
    '  };',
    '  this.GetSize = function () {',
    '    var Result = 0;',
    '    this.x = this.y;',
    '    this.SetSize(this.GetSize());',
    '    this.y = this.x;',
    '    return Result;',
    '  };',
    '  this.SetSize = function (Value) {',
    '  };',
    '});',
    'this.SetSize = function (Value) {',
    '};',
    'this.p = this.TPoint.$new();',
    'this.q = this.TPoint.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p.SetSize($mod.q.GetSize());',
    '$mod.p.y = $mod.q.x;',
    '']));
end;

procedure TTestModule.TestAdvRecord_PropertyDefault;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TPoint = record',
  '  strict private',
  '    function GetItems(Index: word): word;',
  '    procedure SetItems(Index: word; Value: word);',
  '  public',
  '    property Items[Index: word]: word read GetItems write SetItems; default;',
  '  end;',
  'function TPoint.GetItems(Index: word): word;',
  'begin',
  '  Items[index]:=Items[index];',
  '  self.Items[index]:=self.Items[index];',
  'end;',
  'procedure TPoint.SetItems(Index: word; Value: word);',
  'begin',
  'end;',
  'var p: TPoint;',
  'begin',
  '  p[1]:=p[2];',
  '  p.Items[3]:=p.Items[4];',
  '']);
  ConvertProgram;
  CheckSource('TestAdvRecord_PropertyDefault',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TPoint", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  this.GetItems = function (Index) {',
    '    var Result = 0;',
    '    this.SetItems(Index, this.GetItems(Index));',
    '    this.SetItems(Index, this.GetItems(Index));',
    '    return Result;',
    '  };',
    '  this.SetItems = function (Index, Value) {',
    '  };',
    '});',
    'this.p = this.TPoint.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p.SetItems(1, $mod.p.GetItems(2));',
    '$mod.p.SetItems(3, $mod.p.GetItems(4));',
    '']));
end;

procedure TTestModule.TestAdvRecord_Property_ClassMethod;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TRec = record',
  '    class var',
  '      Fx: longint;',
  '      Fy: longint;',
  '    class function GetInt: longint; static;',
  '    class procedure SetInt(Value: longint); static;',
  '    class procedure DoIt; static;',
  '    class property IntA: longint read Fx write Fy;',
  '    class property IntB: longint read GetInt write SetInt;',
  '  end;',
  'class function trec.getint: longint;',
  'begin',
  '  result:=fx;',
  'end;',
  'class procedure trec.setint(value: longint);',
  'begin',
  'end;',
  'class procedure trec.doit;',
  'begin',
  '  IntA:=IntA+1;',
  '  IntB:=IntB+1;',
  'end;',
  'var r: trec;',
  'begin',
  '  trec.inta:=trec.inta+1;',
  '  if trec.intb=2 then;',
  '  trec.intb:=trec.intb+2;',
  '  trec.setint(trec.inta);',
  '  r.inta:=r.inta+1;',
  '  if r.intb=2 then;',
  '  r.intb:=r.intb+2;',
  '  r.setint(r.inta);']);
  ConvertProgram;
  CheckSource('TestAdvRecord_Property_ClassMethod',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.Fx = 0;',
    '  this.Fy = 0;',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  this.GetInt = function () {',
    '    var Result = 0;',
    '    Result = $mod.TRec.Fx;',
    '    return Result;',
    '  };',
    '  this.SetInt = function (Value) {',
    '  };',
    '  this.DoIt = function () {',
    '    $mod.TRec.Fy = $mod.TRec.Fx + 1;',
    '    $mod.TRec.SetInt($mod.TRec.GetInt() + 1);',
    '  };',
    '}, true);',
    'this.r = this.TRec.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TRec.Fy = $mod.TRec.Fx + 1;',
    'if ($mod.TRec.GetInt() === 2) ;',
    '$mod.TRec.SetInt($mod.TRec.GetInt() + 2);',
    '$mod.TRec.SetInt($mod.TRec.Fx);',
    '$mod.TRec.Fy = $mod.r.Fx + 1;',
    'if ($mod.TRec.GetInt() === 2) ;',
    '$mod.TRec.SetInt($mod.TRec.GetInt() + 2);',
    '$mod.TRec.SetInt($mod.r.Fx);',
    '']));
end;

procedure TTestModule.TestAdvRecord_Const;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TArrInt = array[3..4] of longint;',
  '  TPoint = record',
  '    x,y: longint;',
  '    class var Count: nativeint;',
  '  end;',
  '  TRec = record',
  '    i: longint;',
  '    a: array of longint;',
  '    s: array[1..2] of longint;',
  '    m: array[1..2,3..4] of longint;',
  '    p: TPoint;',
  '  end;',
  '  TPoints = array of TPoint;',
  'const',
  '  r: TRec = (',
  '    i:1;',
  '    a:(2,3);',
  '    s:(4,5);',
  '    m:( (11,12), (13,14) );',
  '    p: (x:21)',
  '  );',
  '  p: TPoints = ( (x:1;y:2), (x:3;y:4) );',
  'begin']);
  ConvertProgram;
  CheckSource('TestAdvRecord_Const',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TPoint", function () {',
    '  this.x = 0;',
    '  this.y = 0;',
    '  this.Count = 0;',
    '  this.$eq = function (b) {',
    '    return (this.x === b.x) && (this.y === b.y);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    this.y = s.y;',
    '    return this;',
    '  };',
    '}, true);',
    'rtl.recNewT(this, "TRec", function () {',
    '  this.i = 0;',
    '  this.$new = function () {',
    '    var r = Object.create(this);',
    '    r.a = [];',
    '    r.s = rtl.arraySetLength(null, 0, 2);',
    '    r.m = rtl.arraySetLength(null, 0, 2, 2);',
    '    r.p = $mod.TPoint.$new();',
    '    return r;',
    '  };',
    '  this.$eq = function (b) {',
    '    return (this.i === b.i) && (this.a === b.a) && rtl.arrayEq(this.s, b.s) && rtl.arrayEq(this.m, b.m) && this.p.$eq(b.p);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    this.a = rtl.arrayRef(s.a);',
    '    this.s = s.s.slice(0);',
    '    this.m = s.m.slice(0);',
    '    this.p.$assign(s.p);',
    '    return this;',
    '  };',
    '});',
    'this.r = this.TRec.$clone({',
    '  i: 1,',
    '  a: [2, 3],',
    '  s: [4, 5],',
    '  m: [[11, 12], [13, 14]],',
    '  p: this.TPoint.$clone({',
    '      x: 21,',
    '      y: 0',
    '    })',
    '});',
    'this.p = [this.TPoint.$clone({',
    '  x: 1,',
    '  y: 2',
    '}), this.TPoint.$clone({',
    '  x: 3,',
    '  y: 4',
    '})];',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestAdvRecord_ExternalField;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  '{$modeswitch externalclass}',
  'type',
  '  TCar = record',
  '  public',
  '    Intern: longint external name ''$Intern'';',
  '    Intern2: longint external name ''$Intern2'';',
  '    Bracket: longint external name ''["A B"]'';',
  '    procedure DoIt;',
  '  end;',
  'procedure tcar.doit;',
  'begin',
  '  Intern:=Intern+1;',
  '  Intern2:=Intern2+2;',
  '  Bracket:=Bracket+3;',
  'end;',
  'var Rec: TCar = (intern: 11; intern2: 12; bracket: 13);',
  'begin',
  '  Rec.intern:=Rec.intern+1;',
  '  Rec.intern2:=Rec.intern2+2;',
  '  Rec.Bracket:=Rec.Bracket+3;',
  '  with Rec do begin',
  '    intern:=intern+1;',
  '    intern2:=intern2+2;',
  '    Bracket:=Bracket+3;',
  '  end;']);
  ConvertProgram;
  CheckSource('TestAdvRecord_ExternalField',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TCar", function () {',
    '  this.$eq = function (b) {',
    '    return (this.$Intern === b.$Intern) && (this.$Intern2 === b.$Intern2) && (this["A B"] === b["A B"]);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.$Intern = s.$Intern;',
    '    this.$Intern2 = s.$Intern2;',
    '    this["A B"] = s["A B"];',
    '    return this;',
    '  };',
    '  this.DoIt = function () {',
    '    this.$Intern = this.$Intern + 1;',
    '    this.$Intern2 = this.$Intern2 + 2;',
    '    this["A B"] = this["A B"] + 3;',
    '  };',
    '});',
    'this.Rec = this.TCar.$clone({',
    '  $Intern: 11,',
    '  $Intern2: 12,',
    '  "A B": 13',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Rec.$Intern = $mod.Rec.$Intern + 1;',
    '$mod.Rec.$Intern2 = $mod.Rec.$Intern2 + 2;',
    '$mod.Rec["A B"] = $mod.Rec["A B"] + 3;',
    'var $with = $mod.Rec;',
    '$with.$Intern = $with.$Intern + 1;',
    '$with.$Intern2 = $with.$Intern2 + 2;',
    '$with["A B"] = $with["A B"] + 3;',
    '']));
end;

procedure TTestModule.TestAdvRecord_SubRecord;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TRec = record',
  '  type',
  '    TPoint = record',
  '      x,y: longint;',
  '      class var Count: nativeint;',
  '      procedure DoIt;',
  '      class procedure DoThat; static;',
  '    end;',
  '  var',
  '    i: longint;',
  '    p: TPoint;',
  '    procedure DoSome;',
  '  end;',
  'const',
  '  r: TRec = (',
  '    i:1;',
  '    p: (x:21;y:22)',
  '  );',
  'procedure TRec.DoSome;',
  'begin',
  '  p.x:=p.y+1;',
  '  p.Count:=p.Count+2;',
  'end;',
  'procedure TRec.TPoint.DoIt;',
  'begin',
  '  Count:=Count+3;',
  'end;',
  'class procedure TRec.TPoint.DoThat;',
  'begin',
  '  Count:=Count+4;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestAdvRecord_SubRecord',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  rtl.recNewT(this, "TPoint", function () {',
    '    this.x = 0;',
    '    this.y = 0;',
    '    this.Count = 0;',
    '    this.$eq = function (b) {',
    '      return (this.x === b.x) && (this.y === b.y);',
    '    };',
    '    this.$assign = function (s) {',
    '      this.x = s.x;',
    '      this.y = s.y;',
    '      return this;',
    '    };',
    '    this.DoIt = function () {',
    '      $mod.TRec.TPoint.Count = this.Count + 3;',
    '    };',
    '    this.DoThat = function () {',
    '      $mod.TRec.TPoint.Count = $mod.TRec.TPoint.Count + 4;',
    '    };',
    '  }, true);',
    '  this.i = 0;',
    '  this.$new = function () {',
    '    var r = Object.create(this);',
    '    r.p = this.TPoint.$new();',
    '    return r;',
    '  };',
    '  this.$eq = function (b) {',
    '    return (this.i === b.i) && this.p.$eq(b.p);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.i = s.i;',
    '    this.p.$assign(s.p);',
    '    return this;',
    '  };',
    '  this.DoSome = function () {',
    '    this.p.x = this.p.y + 1;',
    '    this.TPoint.Count = this.p.Count + 2;',
    '  };',
    '}, true);',
    'this.r = this.TRec.$clone({',
    '  i: 1,',
    '  p: this.TRec.TPoint.$clone({',
    '      x: 21,',
    '      y: 22',
    '    })',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestAdvRecord_SubClass;
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
  '  doit;',
  '  self.doit;',
  '  glob;',
  '  self.glob;',
  'end;',
  'class procedure TPoint.TBird.Glob;',
  'begin',
  '  glob;',
  '  self.glob;',
  'end;',
  'procedure TPoint.DoIt(b: TBird);',
  'begin',
  '  b.doit;',
  '  b.glob;',
  '  TBird.glob;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestAdvRecord_SubClass',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.recNewT(this, "TPoint", function () {',
    '  rtl.createClass(this, "TBird", $mod.TObject, function () {',
    '    this.DoIt = function () {',
    '      this.DoIt();',
    '      this.DoIt();',
    '      this.$class.Glob();',
    '      this.$class.Glob();',
    '    };',
    '    this.Glob = function () {',
    '      this.Glob();',
    '      this.Glob();',
    '    };',
    '  }, "TPoint.TBird");',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  this.DoIt = function (b) {',
    '    b.DoIt();',
    '    b.$class.Glob();',
    '    this.TBird.Glob();',
    '  };',
    '}, true);',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestAdvRecord_SubInterfaceFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  IUnknown = interface end;',
  '  TPoint = record',
  '    type IBird = interface end;',
  '  end;',
  'begin',
  '']);
  SetExpectedPasResolverError('not yet implemented: IBird:TPasClassType [20190105143752] "interface inside record"',
    nNotYetImplemented);
  ParseProgram;
end;

procedure TTestModule.TestAdvRecord_Constructor;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TPoint = record',
  '    x,y: longint;',
  '    class procedure Run(w: longint = 13); static;',
  '    constructor Create(ax: longint; ay: longint = -1);',
  '  end;',
  'class procedure tpoint.run(w: longint);',
  'begin',
  '   run;',
  '   run();',
  'end;',
  'constructor tpoint.create(ax,ay: longint);',
  'begin',
  '  x:=ax;',
  '  self.y:=ay;',
  ' run;',
  '  run(ax);',
  'end;',
  'var r: TPoint;',
  'begin',
  '  r:=TPoint.Create(1,2);',
  '  with TPoint do r:=Create(1,2);',
  '  r.Create(3);',
  '  r:=r.Create(4);',
  '']);
  ConvertProgram;
  CheckSource('TestAdvRecord_Constructor',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TPoint", function () {',
    '  this.x = 0;',
    '  this.y = 0;',
    '  this.$eq = function (b) {',
    '    return (this.x === b.x) && (this.y === b.y);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    this.y = s.y;',
    '    return this;',
    '  };',
    '  this.Run = function (w) {',
    '    $mod.TPoint.Run(13);',
    '    $mod.TPoint.Run(13);',
    '  };',
    '  this.Create = function (ax, ay) {',
    '    this.x = ax;',
    '    this.y = ay;',
    '    this.Run(13);',
    '    this.Run(ax);',
    '    return this;',
    '  };',
    '});',
    'this.r = this.TPoint.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.r.$assign($mod.TPoint.$new().Create(1, 2));',
    'var $with = $mod.TPoint;',
    '$mod.r.$assign($with.$new().Create(1, 2));',
    '$mod.r.Create(3, -1);',
    '$mod.r.$assign($mod.r.Create(4, -1));',
    '']));
end;

procedure TTestModule.TestAdvRecord_ClassConstructor_Program;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TPoint = record',
  '    class var x: longint;',
  '    class procedure Fly; static;',
  '    class constructor Init;',
  '  end;',
  'var count: word;',
  'class procedure Tpoint.Fly;',
  'begin',
  'end;',
  'class constructor tpoint.init;',
  'begin',
  '  count:=count+1;',
  '  x:=x+3;',
  '  tpoint.x:=tpoint.x+4;',
  '  fly;',
  '  tpoint.fly;',
  'end;',
  'var r: TPoint;',
  'begin',
  '  r.x:=r.x+10;',
  '  r.Fly;',
  '  r.Fly();',
  '']);
  ConvertProgram;
  CheckSource('TestAdvRecord_ClassConstructor_Program',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TPoint", function () {',
    '  this.x = 0;',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  this.Fly = function () {',
    '  };',
    '}, true);',
    'this.count = 0;',
    'this.r = this.TPoint.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '(function () {',
    '  $mod.count = $mod.count + 1;',
    '  $mod.TPoint.x = $mod.TPoint.x + 3;',
    '  $mod.TPoint.x = $mod.TPoint.x + 4;',
    '  $mod.TPoint.Fly();',
    '  $mod.TPoint.Fly();',
    '})();',
    '$mod.TPoint.x = $mod.r.x + 10;',
    '$mod.TPoint.Fly();',
    '$mod.TPoint.Fly();',
    '']));
end;

procedure TTestModule.TestAdvRecord_ClassConstructor_Unit;
begin
  StartUnit(false);
  Add([
  'interface',
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TPoint = record',
  '    class var x: longint;',
  '    class procedure Fly; static;',
  '    class constructor Init;',
  '  end;',
  'implementation',
  'var count: word;',
  'class procedure Tpoint.Fly;',
  'begin',
  'end;',
  'class constructor tpoint.init;',
  'begin',
  '  count:=count+1;',
  '  x:=3;',
  '  tpoint.x:=4;',
  '  fly;',
  '  tpoint.fly;',
  'end;',
  '']);
  ConvertUnit;
  CheckSource('TestAdvRecord_ClassConstructor_Unit',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    'rtl.recNewT(this, "TPoint", function () {',
    '  this.x = 0;',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  this.Fly = function () {',
    '  };',
    '}, true);',
    '']),
    LinesToStr([ // $mod.$init
    '(function () {',
    '  $impl.count = $impl.count + 1;',
    '  $mod.TPoint.x = 3;',
    '  $mod.TPoint.x = 4;',
    '  $mod.TPoint.Fly();',
    '  $mod.TPoint.Fly();',
    '})();',
    '']),
    LinesToStr([ // $mod.$main
    '$impl.count = 0;',
    '']));
end;

procedure TTestModule.TestClass_TObjectDefaultConstructor;
begin
  StartProgram(false);
  Add(['type',
  '  TObject = class',
  '  public',
  '    constructor Create;',
  '    destructor Destroy;',
  '  end;',
  '  TBird = TObject;',
  'constructor tobject.create;',
  'begin end;',
  'destructor tobject.destroy;',
  'begin end;',
  'var Obj: tobject;',
  'begin',
  '  obj:=tobject.create;',
  '  obj:=tobject.create();',
  '  obj:=tbird.create;',
  '  obj:=tbird.create();',
  '  obj:=obj.create();',
  '  obj.destroy;',
  '']);
  ConvertProgram;
  CheckSource('TestClass_TObjectDefaultConstructor',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function(){',
    '    return this;',
    '  };',
    '  this.Destroy = function(){',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj = $mod.TObject.$create("Create");',
    '$mod.Obj = $mod.TObject.$create("Create");',
    '$mod.Obj = $mod.TObject.$create("Create");',
    '$mod.Obj = $mod.TObject.$create("Create");',
    '$mod.Obj = $mod.Obj.Create();',
    '$mod.Obj.$destroy("Destroy");',
    '']));
end;

procedure TTestModule.TestClass_TObjectConstructorWithParams;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  public');
  Add('    constructor Create(Par: longint);');
  Add('  end;');
  Add('constructor tobject.create(par: longint);');
  Add('begin end;');
  Add('var Obj: tobject;');
  Add('begin');
  Add('  obj:=tobject.create(3);');
  ConvertProgram;
  CheckSource('TestClass_TObjectConstructorWithParams',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function(Par){',
    '    return this;',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj = $mod.TObject.$create("Create",[3]);'
    ]));
end;

procedure TTestModule.TestClass_TObjectConstructorWithDefaultParam;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  public');
  Add('    constructor Create;');
  Add('  end;');
  Add('  TTest = class(TObject)');
  Add('  public');
  Add('    constructor Create(const Par: longint = 1);');
  Add('  end;');
  Add('constructor tobject.create;');
  Add('begin end;');
  Add('constructor ttest.create(const par: longint);');
  Add('begin end;');
  Add('var t: ttest;');
  Add('begin');
  Add('  t:=ttest.create;');
  Add('  t:=ttest.create(2);');
  ConvertProgram;
  CheckSource('TestClass_TObjectConstructorWithDefaultParam',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function(){',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TTest", this.TObject, function () {',
    '  this.Create$1 = function (Par) {',
    '    return this;',
    '  };',
    '});',
    'this.t = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.t = $mod.TTest.$create("Create$1", [1]);',
    '$mod.t = $mod.TTest.$create("Create$1", [2]);'
    ]));
end;

procedure TTestModule.TestClass_Var;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  public',
  '    vI: longint;',
  '    constructor Create(Par: longint);',
  '  end;',
  'constructor tobject.create(par: longint);',
  'begin',
  '  vi:=par+3',
  'end;',
  'var Obj: tobject;',
  'begin',
  '  obj:=tobject.create(4);',
  '  obj.vi:=obj.VI+5;']);
  ConvertProgram;
  CheckSource('TestClass_Var',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.$init = function () {',
    '    this.vI = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function(Par){',
    '    this.vI = Par+3;',
    '    return this;',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj = $mod.TObject.$create("Create",[4]);',
    '$mod.Obj.vI = $mod.Obj.vI + 5;'
    ]));
end;

procedure TTestModule.TestClass_Method;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  public');
  Add('    vI: longint;');
  Add('    Sub: TObject;');
  Add('    constructor Create;');
  Add('    function GetIt(Par: longint): tobject;');
  Add('  end;');
  Add('constructor tobject.create; begin end;');
  Add('function tobject.getit(par: longint): tobject;');
  Add('begin');
  Add('  Self.vi:=par+3;');
  Add('  Result:=self.sub;');
  Add('end;');
  Add('var Obj: tobject;');
  Add('begin');
  Add('  obj:=tobject.create;');
  Add('  obj.getit(4);');
  Add('  obj.sub.sub:=nil;');
  Add('  obj.sub.getit(5);');
  Add('  obj.sub.getit(6).SUB:=nil;');
  Add('  obj.sub.getit(7).GETIT(8);');
  Add('  obj.sub.getit(9).SuB.getit(10);');
  ConvertProgram;
  CheckSource('TestClass_Method',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.$init = function () {',
    '    this.vI = 0;',
    '    this.Sub = null;',
    '  };',
    '  this.$final = function () {',
    '    this.Sub = undefined;',
    '  };',
    '  this.Create = function(){',
    '    return this;',
    '  };',
    '  this.GetIt = function(Par){',
    '    var Result = null;',
    '    this.vI = Par + 3;',
    '    Result = this.Sub;',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj = $mod.TObject.$create("Create");',
    '$mod.Obj.GetIt(4);',
    '$mod.Obj.Sub.Sub=null;',
    '$mod.Obj.Sub.GetIt(5);',
    '$mod.Obj.Sub.GetIt(6).Sub=null;',
    '$mod.Obj.Sub.GetIt(7).GetIt(8);',
    '$mod.Obj.Sub.GetIt(9).Sub.GetIt(10);'
    ]));
end;

procedure TTestModule.TestClass_Implementation;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  'implementation',
  'type',
  '  TIntClass = class',
  '    constructor Create; reintroduce;',
  '    class procedure DoGlob;',
  '  end;',
  'constructor tintclass.create;',
  'begin',
  '  inherited;',
  '  inherited create;',
  '  doglob;',
  'end;',
  'class procedure tintclass.doglob;',
  'begin',
  'end;',
  'constructor tobject.create;',
  'var',
  '  iC: tintclass;',
  'begin',
  '  ic:=tintclass.create;',
  '  tintclass.doglob;',
  '  ic.doglob;',
  'end;',
  'initialization',
  '  tintclass.doglob;',
  '']);
  ConvertUnit;
  CheckSource('TestClass_Implementation',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    var iC = null;',
    '    iC = $impl.TIntClass.$create("Create$1");',
    '    $impl.TIntClass.DoGlob();',
    '    iC.$class.DoGlob();',
    '    return this;',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '$impl.TIntClass.DoGlob();',
    '']),
    LinesToStr([
    'rtl.createClass($impl, "TIntClass", $mod.TObject, function () {',
    '  this.Create$1 = function () {',
    '    $mod.TObject.Create.call(this);',
    '    $mod.TObject.Create.call(this);',
    '    this.$class.DoGlob();',
    '    return this;',
    '  };',
    '  this.DoGlob = function () {',
    '  };',
    '});',
    '']));
end;

procedure TTestModule.TestClass_Inheritance;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  public');
  Add('    constructor Create;');
  Add('  end;');
  Add('  TClassA = class');
  Add('  end;');
  Add('  TClassB = class(TObject)');
  Add('    procedure ProcB;');
  Add('  end;');
  Add('constructor tobject.create; begin end;');
  Add('procedure tclassb.procb; begin end;');
  Add('var');
  Add('  oO: TObject;');
  Add('  oA: TClassA;');
  Add('  oB: TClassB;');
  Add('begin');
  Add('  oO:=tobject.Create;');
  Add('  oA:=tclassa.Create;');
  Add('  ob:=tclassb.Create;');
  Add('  if oo is tclassa then ;');
  Add('  ob:=oo as tclassb;');
  Add('  (oo as tclassb).procb;');
  ConvertProgram;
  CheckSource('TestClass_Inheritance',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this,"TClassA",this.TObject,function(){',
    '});',
    'rtl.createClass(this,"TClassB",this.TObject,function(){',
    '  this.ProcB = function () {',
    '  };',
    '});',
    'this.oO = null;',
    'this.oA = null;',
    'this.oB = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.oO = $mod.TObject.$create("Create");',
    '$mod.oA = $mod.TClassA.$create("Create");',
    '$mod.oB = $mod.TClassB.$create("Create");',
    'if ($mod.TClassA.isPrototypeOf($mod.oO));',
    '$mod.oB = rtl.as($mod.oO, $mod.TClassB);',
    'rtl.as($mod.oO, $mod.TClassB).ProcB();'
    ]));
end;

procedure TTestModule.TestClass_TypeAlias;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IObject = interface',
  '  end;',
  '  IBird = type IObject;',
  '  TObject = class',
  '  end;',
  '  TBird = type TObject;',
  'var',
  '  oObj: TObject;',
  '  oBird: TBird;',
  '  IntfObj: IObject;',
  '  IntfBird: IBird;',
  'begin',
  '  oObj:=oBird;',
  '']);
  ConvertProgram;
  CheckSource('TestClass_TypeAlias',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IObject", "{B92D5841-6F2A-306A-8000-000000000000}", [], null);',
    'rtl.createInterface(this, "IBird", "{4B0D080B-C0F6-387B-AE88-F10981585074}", [], this.IObject);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '});',
    'this.oObj = null;',
    'this.oBird = null;',
    'this.IntfObj = null;',
    'this.IntfBird = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.oObj = $mod.oBird;',
    '']));
end;

procedure TTestModule.TestClass_AbstractMethod;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  public');
  Add('    procedure DoIt; virtual; abstract;');
  Add('  end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestClass_AbstractMethod',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestClass_CallInherited_ProcNoParams;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoAbstract; virtual; abstract;',
  '    procedure DoVirtual; virtual;',
  '    procedure DoIt;',
  '  end;',
  '  TA = class',
  '    procedure doabstract; override;',
  '    procedure dovirtual; override;',
  '    procedure DoSome;',
  '  end;',
  'procedure tobject.dovirtual;',
  'begin',
  '  inherited; // call non existing ancestor -> ignore silently',
  'end;',
  'procedure tobject.doit;',
  'begin',
  'end;',
  'procedure ta.doabstract;',
  'begin',
  '  inherited dovirtual; // call TObject.DoVirtual',
  'end;',
  'procedure ta.dovirtual;',
  'begin',
  '  inherited; // call TObject.DoVirtual',
  '  inherited dovirtual; // call TObject.DoVirtual',
  '  inherited dovirtual(); // call TObject.DoVirtual',
  '  doit;',
  '  doit();',
  'end;',
  'procedure ta.dosome;',
  'begin',
  '  inherited; // call non existing ancestor method -> silently ignore',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestClass_CallInherited_ProcNoParams',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoVirtual = function () {',
    '  };',
    '  this.DoIt = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TA", this.TObject, function () {',
    '  this.DoAbstract = function () {',
    '    $mod.TObject.DoVirtual.call(this);',
    '  };',
    '  this.DoVirtual = function () {',
    '    $mod.TObject.DoVirtual.call(this);',
    '    $mod.TObject.DoVirtual.call(this);',
    '    $mod.TObject.DoVirtual.call(this);',
    '    this.DoIt();',
    '    this.DoIt();',
    '  };',
    '  this.DoSome = function () {',
    '  };',
    '});'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestClass_CallInherited_WithParams;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoAbstract(pA: longint; pB: longint = 0); virtual; abstract;',
  '    procedure DoVirtual(pA: longint; pB: longint = 0); virtual;',
  '    procedure DoIt(pA: longint; pB: longint = 0);',
  '    procedure DoIt2(pA: longint = 1; pB: longint = 2);',
  '    function GetIt(pA: longint = 1; pB: longint = 2): longint;',
  '  end;',
  '  TClassA = class',
  '    procedure DoAbstract(pA: longint; pB: longint = 0); override;',
  '    procedure DoVirtual(pA: longint; pB: longint = 0); override;',
  '    function GetIt(pA: longint = 1; pB: longint = 2): longint;',
  '  end;',
  'procedure tobject.dovirtual(pa: longint; pb: longint = 0);',
  'begin',
  'end;',
  'procedure tobject.doit(pa: longint; pb: longint = 0);',
  'begin',
  'end;',
  'procedure tobject.doit2(pa: longint; pb: longint = 0);',
  'begin',
  'end;',
  'function tobject.getit(pa: longint; pb: longint = 0): longint;',
  'begin',
  'end;',
  'procedure tclassa.doabstract(pa: longint; pb: longint = 0);',
  'begin',
  '  inherited dovirtual(pa,pb); // call TObject.DoVirtual(pA,pB)',
  '  inherited dovirtual(pa); // call TObject.DoVirtual(pA,0)',
  'end;',
  'procedure tclassa.dovirtual(pa: longint; pb: longint = 0);',
  'begin',
  '  inherited; // call TObject.DoVirtual(pA,pB)',
  '  inherited dovirtual(pa,pb); // call TObject.DoVirtual(pA,pB)',
  '  inherited dovirtual(pa); // call TObject.DoVirtual(pA,0)',
  '  doit(pa,pb);',
  '  doit(pa);',
  '  doit2(pa);',
  '  doit2;',
  'end;',
  'function tclassa.getit(pa: longint; pb: longint = 0): longint;',
  'begin',
  '  pa:=inherited;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestClass_CallInherited_WithParams',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoVirtual = function (pA,pB) {',
    '  };',
    '  this.DoIt = function (pA,pB) {',
    '  };',
    '  this.DoIt2 = function (pA,pB) {',
    '  };',
    '  this.GetIt = function (pA, pB) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'rtl.createClass(this, "TClassA", this.TObject, function () {',
    '  this.DoAbstract = function (pA,pB) {',
    '    $mod.TObject.DoVirtual.call(this,pA,pB);',
    '    $mod.TObject.DoVirtual.call(this,pA,0);',
    '  };',
    '  this.DoVirtual = function (pA,pB) {',
    '    $mod.TObject.DoVirtual.apply(this, arguments);',
    '    $mod.TObject.DoVirtual.call(this,pA,pB);',
    '    $mod.TObject.DoVirtual.call(this,pA,0);',
    '    this.DoIt(pA,pB);',
    '    this.DoIt(pA,0);',
    '    this.DoIt2(pA,2);',
    '    this.DoIt2(1,2);',
    '  };',
    '  this.GetIt$1 = function (pA, pB) {',
    '    var Result = 0;',
    '    pA = $mod.TObject.GetIt.apply(this, arguments);',
    '    return Result;',
    '  };',
    '});'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestClasS_CallInheritedConstructor;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    constructor Create; virtual;');
  Add('    constructor CreateWithB(b: boolean);');
  Add('  end;');
  Add('  TA = class');
  Add('    constructor Create; override;');
  Add('    constructor CreateWithC(c: char);');
  Add('    procedure DoIt;');
  Add('    class function DoSome: TObject;');
  Add('  end;');
  Add('constructor tobject.create;');
  Add('begin');
  Add('  inherited; // call non existing ancestor -> ignore silently');
  Add('end;');
  Add('constructor tobject.createwithb(b: boolean);');
  Add('begin');
  Add('  inherited; // call non existing ancestor -> ignore silently');
  Add('  create; // normal call');
  Add('end;');
  Add('constructor ta.create;');
  Add('begin');
  Add('  inherited; // normal call TObject.Create');
  Add('  inherited create; // normal call TObject.Create');
  Add('  inherited createwithb(false); // normal call TObject.CreateWithB');
  Add('end;');
  Add('constructor ta.createwithc(c: char);');
  Add('begin');
  Add('  inherited create; // call TObject.Create');
  Add('  inherited createwithb(true); // call TObject.CreateWithB');
  Add('  doit;');
  Add('  doit();');
  Add('  dosome;');
  Add('end;');
  Add('procedure ta.doit;');
  Add('begin');
  Add('  create; // normal call');
  Add('  createwithb(false); // normal call');
  Add('  createwithc(''c''); // normal call');
  Add('end;');
  Add('class function ta.dosome: TObject;');
  Add('begin');
  Add('  Result:=create; // constructor');
  Add('  Result:=createwithb(true); // constructor');
  Add('  Result:=createwithc(''c''); // constructor');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestClass_CallInheritedConstructor',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '  this.CreateWithB = function (b) {',
    '    this.Create();',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TA", this.TObject, function () {',
    '  this.Create = function () {',
    '    $mod.TObject.Create.call(this);',
    '    $mod.TObject.Create.call(this);',
    '    $mod.TObject.CreateWithB.call(this, false);',
    '    return this;',
    '  };',
    '  this.CreateWithC = function (c) {',
    '    $mod.TObject.Create.call(this);',
    '    $mod.TObject.CreateWithB.call(this, true);',
    '    this.DoIt();',
    '    this.DoIt();',
    '    this.$class.DoSome();',
    '    return this;',
    '  };',
    '  this.DoIt = function () {',
    '    this.Create();',
    '    this.CreateWithB(false);',
    '    this.CreateWithC("c");',
    '  };',
    '  this.DoSome = function () {',
    '    var Result = null;',
    '    Result = this.$create("Create");',
    '    Result = this.$create("CreateWithB", [true]);',
    '    Result = this.$create("CreateWithC", ["c"]);',
    '    return Result;',
    '  };',
    '});'
    ]),
    LinesToStr([ // this.$main
    ''
    ]));
end;

procedure TTestModule.TestClass_ClassVar_Assign;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  public',
  '    class var vI: longint;',
  '    class var Sub: TObject;',
  '    constructor Create;',
  '    class function GetIt(var Par: longint): tobject;',
  '  end;',
  'constructor tobject.create;',
  'begin',
  '  vi:=vi+1;',
  '  Self.vi:=Self.vi+1;',
  '  inc(vi);',
  'end;',
  'class function tobject.getit(var par: longint): tobject;',
  'begin',
  '  vi:=vi+3;',
  '  Self.vi:=Self.vi+4;',
  '  inc(vi);',
  '  Result:=self.sub;',
  '  GetIt(vi);',
  'end;',
  'var Obj: tobject;',
  'begin',
  '  obj:=tobject.create;',
  '  tobject.vi:=3;',
  '  if tobject.vi=4 then ;',
  '  tobject.sub:=nil;',
  '  obj.sub:=nil;',
  '  obj.sub.sub:=nil;']);
  ConvertProgram;
  CheckSource('TestClass_ClassVar_Assign',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.vI = 0;',
    '  this.Sub = null;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function(){',
    '    $mod.TObject.vI = this.vI+1;',
    '    $mod.TObject.vI = this.vI+1;',
    '    $mod.TObject.vI += 1;',
    '    return this;',
    '  };',
    '  this.GetIt = function(Par){',
    '    var Result = null;',
    '    $mod.TObject.vI = this.vI + 3;',
    '    $mod.TObject.vI = this.vI + 4;',
    '    $mod.TObject.vI += 1;',
    '    Result = this.Sub;',
    '    this.GetIt({',
    '      p: $mod.TObject,',
    '      get: function () {',
    '          return this.p.vI;',
    '        },',
    '      set: function (v) {',
    '          this.p.vI = v;',
    '        }',
    '    });',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj = $mod.TObject.$create("Create");',
    '$mod.TObject.vI = 3;',
    'if ($mod.TObject.vI === 4);',
    '$mod.TObject.Sub=null;',
    '$mod.TObject.Sub=null;',
    '$mod.TObject.Sub=null;',
    '']));
end;

procedure TTestModule.TestClass_CallClassMethod;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  public');
  Add('    class var vI: longint;');
  Add('    class var Sub: TObject;');
  Add('    constructor Create;');
  Add('    function GetMore(Par: longint): longint;');
  Add('    class function GetIt(Par: longint): tobject;');
  Add('  end;');
  Add('constructor tobject.create;');
  Add('begin');
  Add('  sub:=getit(3);');
  Add('  vi:=getmore(4);');
  Add('  sub:=Self.getit(5);');
  Add('  vi:=Self.getmore(6);');
  Add('end;');
  Add('function tobject.getmore(par: longint): longint;');
  Add('begin');
  Add('  sub:=getit(11);');
  Add('  vi:=getmore(12);');
  Add('  sub:=self.getit(13);');
  Add('  vi:=self.getmore(14);');
  Add('end;');
  Add('class function tobject.getit(par: longint): tobject;');
  Add('begin');
  Add('  sub:=getit(21);');
  Add('  vi:=sub.getmore(22);');
  Add('  sub:=self.getit(23);');
  Add('  vi:=self.sub.getmore(24);');
  Add('end;');
  Add('var Obj: tobject;');
  Add('begin');
  Add('  obj:=tobject.create;');
  Add('  tobject.getit(5);');
  Add('  obj.getit(6);');
  Add('  obj.sub.getit(7);');
  Add('  obj.sub.getit(8).SUB:=nil;');
  Add('  obj.sub.getit(9).GETIT(10);');
  Add('  obj.sub.getit(11).SuB.getit(12);');
  ConvertProgram;
  CheckSource('TestClass_CallClassMethod',
    LinesToStr([ // statements
    'rtl.createClass(this,"TObject",null,function(){',
    '  this.vI = 0;',
    '  this.Sub = null;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function(){',
    '    $mod.TObject.Sub = this.$class.GetIt(3);',
    '    $mod.TObject.vI = this.GetMore(4);',
    '    $mod.TObject.Sub = this.$class.GetIt(5);',
    '    $mod.TObject.vI = this.GetMore(6);',
    '    return this;',
    '  };',
    '  this.GetMore = function(Par){',
    '    var Result = 0;',
    '    $mod.TObject.Sub = this.$class.GetIt(11);',
    '    $mod.TObject.vI = this.GetMore(12);',
    '    $mod.TObject.Sub = this.$class.GetIt(13);',
    '    $mod.TObject.vI = this.GetMore(14);',
    '    return Result;',
    '  };',
    '  this.GetIt = function(Par){',
    '    var Result = null;',
    '    $mod.TObject.Sub = this.GetIt(21);',
    '    $mod.TObject.vI = this.Sub.GetMore(22);',
    '    $mod.TObject.Sub = this.GetIt(23);',
    '    $mod.TObject.vI = this.Sub.GetMore(24);',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj = $mod.TObject.$create("Create");',
    '$mod.TObject.GetIt(5);',
    '$mod.Obj.$class.GetIt(6);',
    '$mod.Obj.Sub.$class.GetIt(7);',
    '$mod.TObject.Sub=null;',
    '$mod.Obj.Sub.$class.GetIt(9).$class.GetIt(10);',
    '$mod.Obj.Sub.$class.GetIt(11).Sub.$class.GetIt(12);',
    '']));
end;

procedure TTestModule.TestClass_CallClassMethodStatic;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  public',
  '    class function Fly: tobject; static;',
  '  end;',
  'class function tobject.Fly: tobject;',
  'begin',
  '  Result.Fly;',
  '  Result.Fly();',
  '  Fly;',
  '  Fly();',
  '  Fly.Fly;',
  '  Fly.Fly();',
  'end;',
  'var Obj: tobject;',
  'begin',
  '  obj.Fly;',
  '  obj.Fly();',
  '  with obj do begin',
  '    Fly;',
  '    Fly();',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClass_CallClassMethodStatic',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Fly = function () {',
    '    var Result = null;',
    '    $mod.TObject.Fly();',
    '    $mod.TObject.Fly();',
    '    $mod.TObject.Fly();',
    '    $mod.TObject.Fly();',
    '    $mod.TObject.Fly();',
    '    $mod.TObject.Fly();',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.TObject.Fly();',
    '$mod.TObject.Fly();',
    'var $with = $mod.Obj;',
    '$with.Fly();',
    '$with.Fly();',
    '']));
end;

procedure TTestModule.TestClass_Property;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    Fx: longint;');
  Add('    Fy: longint;');
  Add('    function GetInt: longint;');
  Add('    procedure SetInt(Value: longint);');
  Add('    procedure DoIt;');
  Add('    property IntA: longint read Fx write Fy;');
  Add('    property IntB: longint read GetInt write SetInt;');
  Add('  end;');
  Add('function tobject.getint: longint;');
  Add('begin');
  Add('  result:=fx;');
  Add('end;');
  Add('procedure tobject.setint(value: longint);');
  Add('begin');
  Add('  if value=fy then exit;');
  Add('  fy:=value;');
  Add('end;');
  Add('procedure tobject.doit;');
  Add('begin');
  Add('  IntA:=IntA+1;');
  Add('  Self.IntA:=Self.IntA+1;');
  Add('  IntB:=IntB+1;');
  Add('  Self.IntB:=Self.IntB+1;');
  Add('end;');
  Add('var Obj: tobject;');
  Add('begin');
  Add('  obj.inta:=obj.inta+1;');
  Add('  if obj.intb=2 then;');
  Add('  obj.intb:=obj.intb+2;');
  Add('  obj.setint(obj.inta);');
  ConvertProgram;
  CheckSource('TestClass_Property',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.Fx = 0;',
    '    this.Fy = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetInt = function () {',
    '    var Result = 0;',
    '    Result = this.Fx;',
    '    return Result;',
    '  };',
    '  this.SetInt = function (Value) {',
    '    if (Value === this.Fy) return;',
    '    this.Fy = Value;',
    '  };',
    '  this.DoIt = function () {',
    '    this.Fy = this.Fx + 1;',
    '    this.Fy = this.Fx + 1;',
    '    this.SetInt(this.GetInt() + 1);',
    '    this.SetInt(this.GetInt() + 1);',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj.Fy = $mod.Obj.Fx + 1;',
    'if ($mod.Obj.GetInt() === 2);',
    '$mod.Obj.SetInt($mod.Obj.GetInt() + 2);',
    '$mod.Obj.SetInt($mod.Obj.Fx);'
    ]));
end;

procedure TTestModule.TestClass_Property_ClassMethod;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class var Fx: longint;',
  '    class var Fy: longint;',
  '    class function GetInt: longint;',
  '    class procedure SetInt(Value: longint);',
  '  end;',
  '  TBird = class',
  '    class procedure DoIt;',
  '    class property IntA: longint read Fx write Fy;',
  '    class property IntB: longint read GetInt write SetInt;',
  '  end;',
  'class function tobject.getint: longint;',
  'begin',
  '  result:=fx;',
  'end;',
  'class procedure tobject.setint(value: longint);',
  'begin',
  'end;',
  'class procedure tbird.doit;',
  'begin',
  '  FX:=3;',
  '  IntA:=IntA+1;',
  '  Self.IntA:=Self.IntA+1;',
  '  IntB:=IntB+1;',
  '  Self.IntB:=Self.IntB+1;',
  '  with Self do begin',
  '    FX:=11;',
  '    IntA:=IntA+12;',
  '    IntB:=IntB+13;',
  '  end;',
  'end;',
  'var Obj: tbird;',
  'begin',
  '  tbird.fx:=tbird.fx+1;',
  '  tbird.inta:=tbird.inta+1;',
  '  if tbird.intb=2 then;',
  '  tbird.intb:=tbird.intb+2;',
  '  tbird.setint(tbird.inta);',
  '  obj.inta:=obj.inta+1;',
  '  if obj.intb=2 then;',
  '  obj.intb:=obj.intb+2;',
  '  obj.setint(obj.inta);',
  '  with Tbird do begin',
  '    FX:=FY+1;',
  '    inta:=inta+2;',
  '    intb:=intb+3;',
  '  end;',
  '  with Obj do begin',
  '    FX:=FY+1;',
  '    inta:=inta+2;',
  '    intb:=intb+3;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClass_Property_ClassMethod',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.Fx = 0;',
    '  this.Fy = 0;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetInt = function () {',
    '    var Result = 0;',
    '    Result = this.Fx;',
    '    return Result;',
    '  };',
    '  this.SetInt = function (Value) {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt = function () {',
    '    $mod.TObject.Fx = 3;',
    '    $mod.TObject.Fy = this.Fx + 1;',
    '    $mod.TObject.Fy = this.Fx + 1;',
    '    this.SetInt(this.GetInt() + 1);',
    '    this.SetInt(this.GetInt() + 1);',
    '    $mod.TObject.Fx = 11;',
    '    $mod.TObject.Fy = this.Fx + 12;',
    '    this.SetInt(this.GetInt() + 13);',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.TObject.Fx = $mod.TBird.Fx + 1;',
    '$mod.TObject.Fy = $mod.TBird.Fx + 1;',
    'if ($mod.TBird.GetInt() === 2);',
    '$mod.TBird.SetInt($mod.TBird.GetInt() + 2);',
    '$mod.TBird.SetInt($mod.TBird.Fx);',
    '$mod.TObject.Fy = $mod.Obj.Fx + 1;',
    'if ($mod.Obj.$class.GetInt() === 2);',
    '$mod.Obj.$class.SetInt($mod.Obj.$class.GetInt() + 2);',
    '$mod.Obj.$class.SetInt($mod.Obj.Fx);',
    'var $with = $mod.TBird;',
    '$mod.TObject.Fx = $with.Fy + 1;',
    '$mod.TObject.Fy = $with.Fx + 2;',
    '$with.SetInt($with.GetInt() + 3);',
    'var $with1 = $mod.Obj;',
    '$mod.TObject.Fx = $with1.Fy + 1;',
    '$mod.TObject.Fy = $with1.Fx + 2;',
    '$with1.$class.SetInt($with1.$class.GetInt() + 3);',
    '']));
end;

procedure TTestModule.TestClass_Property_Indexed;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    FItems: array of longint;',
  '    function GetItems(Index: longint): longint;',
  '    procedure SetItems(Index: longint; Value: longint);',
  '    procedure DoIt;',
  '    property Items[Index: longint]: longint read getitems write setitems;',
  '  end;',
  'function tobject.getitems(index: longint): longint;',
  'begin',
  '  Result:=fitems[index];',
  'end;',
  'procedure tobject.setitems(index: longint; value: longint);',
  'begin',
  '  fitems[index]:=value;',
  'end;',
  'procedure tobject.doit;',
  'begin',
  '  items[1]:=2;',
  '  items[3]:=items[4];',
  '  self.items[5]:=self.items[6];',
  '  items[items[7]]:=items[items[8]];',
  'end;',
  'var Obj: tobject;',
  'begin',
  '  obj.Items[11]:=obj.Items[12];',
  '']);
  ConvertProgram;
  CheckSource('TestClass_Property_Indexed',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FItems = [];',
    '  };',
    '  this.$final = function () {',
    '    this.FItems = undefined;',
    '  };',
    '  this.GetItems = function (Index) {',
    '    var Result = 0;',
    '    Result = this.FItems[Index];',
    '    return Result;',
    '  };',
    '  this.SetItems = function (Index, Value) {',
    '    this.FItems[Index] = Value;',
    '  };',
    '  this.DoIt = function () {',
    '    this.SetItems(1, 2);',
    '    this.SetItems(3,this.GetItems(4));',
    '    this.SetItems(5,this.GetItems(6));',
    '    this.SetItems(this.GetItems(7), this.GetItems(this.GetItems(8)));',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj.SetItems(11,$mod.Obj.GetItems(12));'
    ]));
end;

procedure TTestModule.TestClass_Property_IndexSpec;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red, blue);',
  '  TObject = class',
  '    function GetIntBool(Index: longint): boolean; virtual; abstract;',
  '    procedure SetIntBool(Index: longint; b: boolean); virtual; abstract;',
  '    function GetEnumBool(Index: TEnum): boolean; virtual; abstract;',
  '    procedure SetEnumBool(Index: TEnum; b: boolean); virtual; abstract;',
  '    function GetStrIntBool(A: String; I: longint): boolean; virtual; abstract;',
  '    procedure SetStrIntBool(A: String; I: longint; b: boolean); virtual; abstract;',
  '    property B1: boolean index 1 read GetIntBool write SetIntBool;',
  '    property B2: boolean index TEnum.blue read GetEnumBool write SetEnumBool;',
  '    property B3: boolean index ord(red) read GetIntBool write SetIntBool;',
  '    property I1[A: String]: boolean index ord(blue) read GetStrIntBool write SetStrIntBool;',
  '  end;',
  'procedure DoIt(b: boolean); begin end;',
  'var',
  '  o: TObject;',
  'begin',
  '  o.B1:=o.B1;',
  '  o.B2:=o.B2;',
  '  o.B3:=o.B3;',
  '  o.I1[''a'']:=o.I1[''b''];',
  '  doit(o.b1);',
  '  doit(o.b2);',
  '  doit(o.i1[''c'']);',
  '']);
  ConvertProgram;
  CheckSource('TestClass_Property_IndexSpec',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.DoIt = function (b) {',
    '};',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.o.SetIntBool(1, $mod.o.GetIntBool(1));',
    '$mod.o.SetEnumBool($mod.TEnum.blue, $mod.o.GetEnumBool($mod.TEnum.blue));',
    '$mod.o.SetIntBool(0, $mod.o.GetIntBool(0));',
    '$mod.o.SetStrIntBool("a", 1, $mod.o.GetStrIntBool("b", 1));',
    '$mod.DoIt($mod.o.GetIntBool(1));',
    '$mod.DoIt($mod.o.GetEnumBool($mod.TEnum.blue));',
    '$mod.DoIt($mod.o.GetStrIntBool("c", 1));',
    '']));
end;

procedure TTestModule.TestClass_PropertyOfTypeArray;
begin
  StartProgram(false);
  Add('type');
  Add('  TArray = array of longint;');
  Add('  TObject = class');
  Add('    FItems: TArray;');
  Add('    function GetItems: tarray;');
  Add('    procedure SetItems(Value: tarray);');
  Add('    property Items: tarray read getitems write setitems;');
  Add('    procedure SetNumbers(const Value: tarray);');
  Add('    property Numbers: tarray write setnumbers;');
  Add('  end;');
  Add('function tobject.getitems: tarray;');
  Add('begin');
  Add('  Result:=fitems;');
  Add('end;');
  Add('procedure tobject.setitems(value: tarray);');
  Add('begin');
  Add('  fitems:=value;');
  Add('  fitems:=nil;');
  Add('  Items:=nil;');
  Add('  Items:=Items;');
  Add('  Items[1]:=2;');
  Add('  fitems[3]:=Items[4];');
  Add('  Items[5]:=Items[6];');
  Add('  Self.Items[7]:=8;');
  Add('  Self.Items[9]:=Self.Items[10];');
  Add('  Items[Items[11]]:=Items[Items[12]];');
  Add('end;');
  Add('procedure tobject.SetNumbers(const Value: tarray);');
  Add('begin;');
  Add('  Numbers:=nil;');
  Add('  Numbers:=Value;');
  Add('  Self.Numbers:=Value;');
  Add('end;');
  Add('var Obj: tobject;');
  Add('begin');
  Add('  obj.items:=nil;');
  Add('  obj.items:=obj.items;');
  Add('  obj.items[11]:=obj.items[12];');
  ConvertProgram;
  CheckSource('TestClass_PropertyOfTypeArray',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FItems = [];',
    '  };',
    '  this.$final = function () {',
    '    this.FItems = undefined;',
    '  };',
    '  this.GetItems = function () {',
    '    var Result = [];',
    '    Result = rtl.arrayRef(this.FItems);',
    '    return Result;',
    '  };',
    '  this.SetItems = function (Value) {',
    '    this.FItems = rtl.arrayRef(Value);',
    '    this.FItems = [];',
    '    this.SetItems([]);',
    '    this.SetItems(rtl.arrayRef(this.GetItems()));',
    '    this.GetItems()[1] = 2;',
    '    this.FItems[3] = this.GetItems()[4];',
    '    this.GetItems()[5] = this.GetItems()[6];',
    '    this.GetItems()[7] = 8;',
    '    this.GetItems()[9] = this.GetItems()[10];',
    '    this.GetItems()[this.GetItems()[11]] = this.GetItems()[this.GetItems()[12]];',
    '  };',
    '  this.SetNumbers = function (Value) {',
    '    this.SetNumbers([]);',
    '    this.SetNumbers(Value);',
    '    this.SetNumbers(Value);',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj.SetItems([]);',
    '$mod.Obj.SetItems($mod.Obj.GetItems());',
    '$mod.Obj.GetItems()[11] = $mod.Obj.GetItems()[12];'
    ]));
end;

procedure TTestModule.TestClass_PropertyDefault;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArray = array of longint;',
  '  TObject = class',
  '  end;',
  '  TBird = class',
  '    FItems: TArray;',
  '    function GetItems(Index: longint): longint;',
  '    procedure SetItems(Index, Value: longint);',
  '    property Items[Index: longint]: longint read getitems write setitems; default;',
  '  end;',
  'function TBird.getitems(index: longint): longint;',
  'begin',
  'end;',
  'procedure TBird.setitems(index, value: longint);',
  'begin',
  '  Self[1]:=2;',
  '  Self[3]:=Self[index];',
  '  Self[index]:=Self[Self[value]];',
  '  Self[Self[4]]:=value;',
  'end;',
  'var',
  '  Bird: TBird;',
  '  Obj: TObject;',
  'begin',
  '  bird[11]:=12;',
  '  bird[13]:=bird[14];',
  '  bird[Bird[15]]:=bird[Bird[15]];',
  '  TBird(obj)[16]:=TBird(obj)[17];',
  '  (obj as tbird)[18]:=19;',
  '']);
  ConvertProgram;
  CheckSource('TestClass_PropertyDefault',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.FItems = [];',
    '  };',
    '  this.$final = function () {',
    '    this.FItems = undefined;',
    '    $mod.TObject.$final.call(this);',
    '  };',
    '  this.GetItems = function (Index) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  this.SetItems = function (Index, Value) {',
    '    this.SetItems(1, 2);',
    '    this.SetItems(3, this.GetItems(Index));',
    '    this.SetItems(Index, this.GetItems(this.GetItems(Value)));',
    '    this.SetItems(this.GetItems(4), Value);',
    '  };',
    '});',
    'this.Bird = null;',
    'this.Obj = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Bird.SetItems(11, 12);',
    '$mod.Bird.SetItems(13, $mod.Bird.GetItems(14));',
    '$mod.Bird.SetItems($mod.Bird.GetItems(15), $mod.Bird.GetItems($mod.Bird.GetItems(15)));',
    '$mod.Obj.SetItems(16, $mod.Obj.GetItems(17));',
    'rtl.as($mod.Obj, $mod.TBird).SetItems(18, 19);',
    '']));
end;

procedure TTestModule.TestClass_PropertyDefault_TypecastToOtherDefault;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TAlphaList = class',
  '    function GetAlphas(Index: boolean): Pointer; virtual; abstract;',
  '    procedure SetAlphas(Index: boolean; Value: Pointer); virtual; abstract;',
  '    property Alphas[Index: boolean]: Pointer read getAlphas write setAlphas; default;',
  '  end;',
  '  TBetaList = class',
  '    function GetBetas(Index: longint): Pointer; virtual; abstract;',
  '    procedure SetBetas(Index: longint; Value: Pointer); virtual; abstract;',
  '    property Betas[Index: longint]: Pointer read getBetas write setBetas; default;',
  '  end;',
  '  TBird = class',
  '    procedure DoIt;',
  '  end;',
  'procedure TBird.DoIt;',
  'var',
  '  List: TAlphaList;',
  'begin',
  '  if TBetaList(List[true])[3]=nil then ;',
  '  TBetaList(List[false])[5]:=nil;',
  'end;',
  'var',
  '  List: TAlphaList;',
  'begin',
  '  if TBetaList(List[true])[3]=nil then ;',
  '  TBetaList(List[false])[5]:=nil;',
  '']);
  ConvertProgram;
  CheckSource('TestClass_PropertyDefault_TypecastToOtherDefault',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TAlphaList", this.TObject, function () {',
    '});',
    'rtl.createClass(this, "TBetaList", this.TObject, function () {',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt = function () {',
    '    var List = null;',
    '    if (List.GetAlphas(true).GetBetas(3) === null) ;',
    '    List.GetAlphas(false).SetBetas(5, null);',
    '  };',
    '});',
    'this.List = null;',
    '']),
    LinesToStr([ // $mod.$main
    'if ($mod.List.GetAlphas(true).GetBetas(3) === null) ;',
    '$mod.List.GetAlphas(false).SetBetas(5, null);',
    '']));
end;

procedure TTestModule.TestClass_PropertyOverride;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TObject = class');
  Add('    FItem: integer;');
  Add('    function GetItem: integer; external name ''GetItem'';');
  Add('    procedure SetItem(Value: integer); external name ''SetItem'';');
  Add('    property Item: integer read getitem write setitem;');
  Add('  end;');
  Add('  TCar = class');
  Add('    FBag: integer;');
  Add('    function GetBag: integer; external name ''GetBag'';');
  Add('    property Item read getbag;');
  Add('  end;');
  Add('var');
  Add('  Obj: tobject;');
  Add('  Car: tcar;');
  Add('begin');
  Add('  Obj.Item:=Obj.Item;');
  Add('  Car.Item:=Car.Item;');
  ConvertProgram;
  CheckSource('TestClass_PropertyOverride',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FItem = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TCar", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.FBag = 0;',
    '  };',
    '});',
    'this.Obj = null;',
    'this.Car = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Obj.SetItem($mod.Obj.GetItem());',
    '$mod.Car.SetItem($mod.Car.GetBag());',
    '']));
end;

procedure TTestModule.TestClass_PropertyIncVisibility;
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
  ConvertProgram;
  CheckSource('TestClass_PropertyIncVisibility',
    LinesToStr([ // statements
    'rtl.createClass(this, "TBird", pas.unit1.TObject, function () {',
    '});',
    'this.DoIt = function (i) {',
    '};',
    'this.b = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.b.SetItems(1, 2);',
    '$mod.b.SetItems(3, $mod.b.GetItems(4));',
    '$mod.DoIt($mod.b.GetItems(5));'
    ]));
end;

procedure TTestModule.TestClass_Assigned;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('var');
  Add('  Obj: tobject;');
  Add('  b: boolean;');
  Add('begin');
  Add('  if Assigned(obj) then ;');
  Add('  b:=Assigned(obj) or false;');
  ConvertProgram;
  CheckSource('TestClass_Assigned',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.Obj = null;',
    'this.b = false;'
    ]),
    LinesToStr([ // $mod.$main
    'if ($mod.Obj != null);',
    '$mod.b = ($mod.Obj != null) || false;'
    ]));
end;

procedure TTestModule.TestClass_WithClassDoCreate;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    aBool: boolean;');
  Add('    Arr: array of boolean;');
  Add('    constructor Create;');
  Add('  end;');
  Add('constructor TObject.Create; begin end;');
  Add('var');
  Add('  Obj: tobject;');
  Add('  b: boolean;');
  Add('begin');
  Add('  with tobject.create do begin');
  Add('    b:=abool;');
  Add('    abool:=b;');
  Add('    b:=arr[1];');
  Add('    arr[2]:=b;');
  Add('  end;');
  Add('  with tobject do');
  Add('    obj:=create;');
  Add('  with obj do begin');
  Add('    create;');
  Add('    b:=abool;');
  Add('    abool:=b;');
  Add('    b:=arr[3];');
  Add('    arr[4]:=b;');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestClass_WithClassDoCreate',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.aBool = false;',
    '    this.Arr = [];',
    '  };',
    '  this.$final = function () {',
    '    this.Arr = undefined;',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '});',
    'this.Obj = null;',
    'this.b = false;'
    ]),
    LinesToStr([ // $mod.$main
    'var $with = $mod.TObject.$create("Create");',
    '$mod.b = $with.aBool;',
    '$with.aBool = $mod.b;',
    '$mod.b = $with.Arr[1];',
    '$with.Arr[2] = $mod.b;',
    'var $with1 = $mod.TObject;',
    '$mod.Obj = $with1.$create("Create");',
    'var $with2 = $mod.Obj;',
    '$with2.Create();',
    '$mod.b = $with2.aBool;',
    '$with2.aBool = $mod.b;',
    '$mod.b = $with2.Arr[3];',
    '$with2.Arr[4] = $mod.b;',
    '']));
end;

procedure TTestModule.TestClass_WithClassInstDoProperty;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    FInt: longint;');
  Add('    constructor Create;');
  Add('    function GetSize: longint;');
  Add('    procedure SetSize(Value: longint);');
  Add('    property Int: longint read FInt write FInt;');
  Add('    property Size: longint read GetSize write SetSize;');
  Add('  end;');
  Add('constructor TObject.Create; begin end;');
  Add('function TObject.GetSize: longint; begin; end;');
  Add('procedure TObject.SetSize(Value: longint); begin; end;');
  Add('var');
  Add('  Obj: tobject;');
  Add('  i: longint;');
  Add('begin');
  Add('  with TObject.Create do begin');
  Add('    i:=int;');
  Add('    int:=i;');
  Add('    i:=size;');
  Add('    size:=i;');
  Add('  end;');
  Add('  with obj do begin');
  Add('    i:=int;');
  Add('    int:=i;');
  Add('    i:=size;');
  Add('    size:=i;');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestClass_WithClassInstDoProperty',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FInt = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '  this.GetSize = function () {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  this.SetSize = function (Value) {',
    '  };',
    '});',
    'this.Obj = null;',
    'this.i = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'var $with = $mod.TObject.$create("Create");',
    '$mod.i = $with.FInt;',
    '$with.FInt = $mod.i;',
    '$mod.i = $with.GetSize();',
    '$with.SetSize($mod.i);',
    'var $with1 = $mod.Obj;',
    '$mod.i = $with1.FInt;',
    '$with1.FInt = $mod.i;',
    '$mod.i = $with1.GetSize();',
    '$with1.SetSize($mod.i);',
    '']));
end;

procedure TTestModule.TestClass_WithClassInstDoPropertyWithParams;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    constructor Create;');
  Add('    function GetItems(Index: longint): longint;');
  Add('    procedure SetItems(Index, Value: longint);');
  Add('    property Items[Index: longint]: longint read GetItems write SetItems;');
  Add('  end;');
  Add('constructor TObject.Create; begin end;');
  Add('function tobject.getitems(index: longint): longint; begin; end;');
  Add('procedure tobject.setitems(index, value: longint); begin; end;');
  Add('var');
  Add('  Obj: tobject;');
  Add('  i: longint;');
  Add('begin');
  Add('  with TObject.Create do begin');
  Add('    i:=Items[1];');
  Add('    Items[2]:=i;');
  Add('  end;');
  Add('  with obj do begin');
  Add('    i:=Items[3];');
  Add('    Items[4]:=i;');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestClass_WithClassInstDoPropertyWithParams',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '  this.GetItems = function (Index) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  this.SetItems = function (Index, Value) {',
    '  };',
    '});',
    'this.Obj = null;',
    'this.i = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'var $with = $mod.TObject.$create("Create");',
    '$mod.i = $with.GetItems(1);',
    '$with.SetItems(2, $mod.i);',
    'var $with1 = $mod.Obj;',
    '$mod.i = $with1.GetItems(3);',
    '$with1.SetItems(4, $mod.i);',
    '']));
end;

procedure TTestModule.TestClass_WithClassInstDoFunc;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    constructor Create;');
  Add('    function GetSize: longint;');
  Add('    procedure SetSize(Value: longint);');
  Add('  end;');
  Add('constructor TObject.Create; begin end;');
  Add('function TObject.GetSize: longint; begin; end;');
  Add('procedure TObject.SetSize(Value: longint); begin; end;');
  Add('var');
  Add('  Obj: tobject;');
  Add('  i: longint;');
  Add('begin');
  Add('  with TObject.Create do begin');
  Add('    i:=GetSize;');
  Add('    i:=GetSize();');
  Add('    SetSize(i);');
  Add('  end;');
  Add('  with obj do begin');
  Add('    i:=GetSize;');
  Add('    i:=GetSize();');
  Add('    SetSize(i);');
  Add('  end;');
  ConvertProgram;
  CheckSource('TestClass_WithClassInstDoFunc',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '  this.GetSize = function () {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  this.SetSize = function (Value) {',
    '  };',
    '});',
    'this.Obj = null;',
    'this.i = 0;'
    ]),
    LinesToStr([ // $mod.$main
    'var $with = $mod.TObject.$create("Create");',
    '$mod.i = $with.GetSize();',
    '$mod.i = $with.GetSize();',
    '$with.SetSize($mod.i);',
    'var $with1 = $mod.Obj;',
    '$mod.i = $with1.GetSize();',
    '$mod.i = $with1.GetSize();',
    '$with1.SetSize($mod.i);',
    '']));
end;

procedure TTestModule.TestClass_TypeCast;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    Next: TObject;');
  Add('    constructor Create;');
  Add('  end;');
  Add('  TControl = class(TObject)');
  Add('    Arr: array of TObject;');
  Add('    function GetIt(vI: longint = 0): TObject;');
  Add('  end;');
  Add('constructor tobject.create; begin end;');
  Add('function tcontrol.getit(vi: longint = 0): tobject; begin end;');
  Add('var');
  Add('  Obj: tobject;');
  Add('begin');
  Add('  obj:=tcontrol(obj).next;');
  Add('  tcontrol(obj):=nil;');
  Add('  obj:=tcontrol(obj);');
  Add('  tcontrol(obj):=tcontrol(tcontrol(obj).getit);');
  Add('  tcontrol(obj):=tcontrol(tcontrol(obj).getit());');
  Add('  tcontrol(obj):=tcontrol(tcontrol(obj).getit(1));');
  Add('  tcontrol(obj):=tcontrol(tcontrol(tcontrol(obj).getit).arr[2]);');
  Add('  obj:=tcontrol(nil);');
  ConvertProgram;
  CheckSource('TestClass_TypeCast',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.Next = null;',
    '  };',
    '  this.$final = function () {',
    '    this.Next = undefined;',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TControl", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.Arr = [];',
    '  };',
    '  this.$final = function () {',
    '    this.Arr = undefined;',
    '    $mod.TObject.$final.call(this);',
    '  };',
    '  this.GetIt = function (vI) {',
    '    var Result = null;',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj = $mod.Obj.Next;',
    '$mod.Obj = null;',
    '$mod.Obj = $mod.Obj;',
    '$mod.Obj = $mod.Obj.GetIt(0);',
    '$mod.Obj = $mod.Obj.GetIt(0);',
    '$mod.Obj = $mod.Obj.GetIt(1);',
    '$mod.Obj = $mod.Obj.GetIt(0).Arr[2];',
    '$mod.Obj = null;',
    '']));
end;

procedure TTestModule.TestClass_TypeCastUntypedParam;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class end;');
  Add('procedure ProcA(var A);');
  Add('begin');
  Add('  TObject(A):=nil;');
  Add('  TObject(A):=TObject(A);');
  Add('  if TObject(A)=nil then ;');
  Add('  if nil=TObject(A) then ;');
  Add('end;');
  Add('procedure ProcB(out A);');
  Add('begin');
  Add('  TObject(A):=nil;');
  Add('  TObject(A):=TObject(A);');
  Add('  if TObject(A)=nil then ;');
  Add('  if nil=TObject(A) then ;');
  Add('end;');
  Add('procedure ProcC(const A);');
  Add('begin');
  Add('  if TObject(A)=nil then ;');
  Add('  if nil=TObject(A) then ;');
  Add('end;');
  Add('var o: TObject;');
  Add('begin');
  Add('  ProcA(o);');
  Add('  ProcB(o);');
  Add('  ProcC(o);');
  ConvertProgram;
  CheckSource('TestClass_TypeCastUntypedParam',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.ProcA = function (A) {',
    '  A.set(null);',
    '  A.set(A.get());',
    '  if (A.get() === null);',
    '  if (null === A.get());',
    '};',
    'this.ProcB = function (A) {',
    '  A.set(null);',
    '  A.set(A.get());',
    '  if (A.get() === null);',
    '  if (null === A.get());',
    '};',
    'this.ProcC = function (A) {',
    '  if (A === null);',
    '  if (null === A);',
    '};',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.ProcA({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.o;',
    '    },',
    '  set: function (v) {',
    '      this.p.o = v;',
    '    }',
    '});',
    '$mod.ProcB({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.o;',
    '    },',
    '  set: function (v) {',
    '      this.p.o = v;',
    '    }',
    '});',
    '$mod.ProcC($mod.o);',
    '']));
end;

procedure TTestModule.TestClass_Overloads;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure DoIt;');
  Add('    procedure DoIt(vI: longint);');
  Add('  end;');
  Add('procedure TObject.DoIt;');
  Add('begin');
  Add('  DoIt;');
  Add('  DoIt(1);');
  Add('end;');
  Add('procedure TObject.DoIt(vI: longint); begin end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestClass_Overloads',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '    this.DoIt();',
    '    this.DoIt$1(1);',
    '  };',
    '  this.DoIt$1 = function (vI) {',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClass_OverloadsAncestor;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class;');
  Add('  TObject = class');
  Add('    procedure DoIt(vA: longint);');
  Add('    procedure DoIt(vA, vB: longint);');
  Add('  end;');
  Add('  TCar = class;');
  Add('  TCar = class');
  Add('    procedure DoIt(vA: longint);');
  Add('    procedure DoIt(vA, vB: longint);');
  Add('  end;');
  Add('procedure tobject.doit(va: longint);');
  Add('begin');
  Add('  doit(1);');
  Add('  doit(1,2);');
  Add('end;');
  Add('procedure tobject.doit(va, vb: longint); begin end;');
  Add('procedure tcar.doit(va: longint);');
  Add('begin');
  Add('  doit(1);');
  Add('  doit(1,2);');
  Add('  inherited doit(1);');
  Add('  inherited doit(1,2);');
  Add('end;');
  Add('procedure tcar.doit(va, vb: longint); begin end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestClass_OverloadsAncestor',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (vA) {',
    '    this.DoIt(1);',
    '    this.DoIt$1(1,2);',
    '  };',
    '  this.DoIt$1 = function (vA, vB) {',
    '  };',
    '});',
    'rtl.createClass(this, "TCar", this.TObject, function () {',
    '  this.DoIt$2 = function (vA) {',
    '    this.DoIt$2(1);',
    '    this.DoIt$3(1, 2);',
    '    $mod.TObject.DoIt.call(this, 1);',
    '    $mod.TObject.DoIt$1.call(this, 1, 2);',
    '  };',
    '  this.DoIt$3 = function (vA, vB) {',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClass_OverloadConstructor;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    constructor Create(vA: longint);');
  Add('    constructor Create(vA, vB: longint);');
  Add('  end;');
  Add('  TCar = class');
  Add('    constructor Create(vA: longint);');
  Add('    constructor Create(vA, vB: longint);');
  Add('  end;');
  Add('constructor tobject.create(va: longint);');
  Add('begin');
  Add('  create(1);');
  Add('  create(1,2);');
  Add('end;');
  Add('constructor tobject.create(va, vb: longint); begin end;');
  Add('constructor tcar.create(va: longint);');
  Add('begin');
  Add('  create(1);');
  Add('  create(1,2);');
  Add('  inherited create(1);');
  Add('  inherited create(1,2);');
  Add('end;');
  Add('constructor tcar.create(va, vb: longint); begin end;');
  Add('begin');
  Add('  tobject.create(1);');
  Add('  tobject.create(1,2);');
  Add('  tcar.create(1);');
  Add('  tcar.create(1,2);');
  ConvertProgram;
  CheckSource('TestClass_OverloadConstructor',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function (vA) {',
    '    this.Create(1);',
    '    this.Create$1(1,2);',
    '    return this;',
    '  };',
    '  this.Create$1 = function (vA, vB) {',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TCar", this.TObject, function () {',
    '  this.Create$2 = function (vA) {',
    '    this.Create$2(1);',
    '    this.Create$3(1, 2);',
    '    $mod.TObject.Create.call(this, 1);',
    '    $mod.TObject.Create$1.call(this, 1, 2);',
    '    return this;',
    '  };',
    '  this.Create$3 = function (vA, vB) {',
    '    return this;',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TObject.$create("Create", [1]);',
    '$mod.TObject.$create("Create$1", [1, 2]);',
    '$mod.TCar.$create("Create$2", [1]);',
    '$mod.TCar.$create("Create$3", [1, 2]);',
    '']));
end;

procedure TTestModule.TestClass_OverloadDelphiOverride;
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
  'var',
  '  e: TEagle;',
  'begin',
  '  if 23=e.{@c}GetValue then ;',
  '  if 24=e.{@d}GetValue(25) then ;']);
  ConvertProgram;
  CheckSource('TestClass_OverloadDelphiOverride',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.GetValue = function () {',
    '    var Result = 0;',
    '    if (3 === this.GetValue()) ;',
    '    if (4 === this.GetValue$1(5)) ;',
    '    return Result;',
    '  };',
    '  this.GetValue$1 = function (AValue) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'rtl.createClass(this, "TEagle", this.TBird, function () {',
    '  this.GetValue = function () {',
    '    var Result = 0;',
    '    if (13 === this.GetValue()) ;',
    '    if (14 === this.GetValue$1(15)) ;',
    '    if (15 === $mod.TBird.GetValue.call(this)) ;',
    '    if (16 === $mod.TBird.GetValue$1.call(this, 17)) ;',
    '    return Result;',
    '  };',
    '  this.GetValue$1 = function (AValue) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'this.e = null;',
    '']),
    LinesToStr([ // $mod.$main
    'if (23 === $mod.e.GetValue()) ;',
    'if (24 === $mod.e.GetValue$1(25)) ;',
    '']));
end;

procedure TTestModule.TestClass_ReintroduceVarDelphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TObject = class end;',
  '  TAnimal = class',
  '  public',
  '    {#animal_a}A: longint;',
  '    function {#animal_b}B: longint;',
  '  end;',
  '  TBird = class(TAnimal)',
  '  public',
  '    {#bird_a}A: double;',
  '    {#bird_b}B: boolean;',
  '  end;',
  '  TEagle = class(TBird)',
  '  public',
  '    function {#eagle_a}A: boolean;',
  '    {#eagle_b}B: double;',
  '  end;',
  'function TAnimal.B: longint;',
  'begin',
  'end;',
  'function TEagle.A: boolean;',
  'begin',
  '  {@eagle_b}B:=3.3;',
  '  {@eagle_a}A();',
  '  TBird(Self).{@bird_b}B:=true;',
  '  TAnimal(Self).{@animal_a}A:=17;',
  '  inherited {@bird_b}B:=inherited {bird_a}A>1;', // Delphi allows only inherited <functionname>
  'end;',
  'var',
  '  e: TEagle;',
  'begin',
  '  e.{@eagle_b}B:=5.3;',
  '  if e.{@eagle_a}A then ;',
  '']);
  ConvertProgram;
  CheckSource('TestClass_ReintroduceVarDelphi',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TAnimal", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.A = 0;',
    '  };',
    '  this.B = function () {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TAnimal, function () {',
    '  this.$init = function () {',
    '    $mod.TAnimal.$init.call(this);',
    '    this.A$1 = 0.0;',
    '    this.B$1 = false;',
    '  };',
    '});',
    'rtl.createClass(this, "TEagle", this.TBird, function () {',
    '  this.$init = function () {',
    '    $mod.TBird.$init.call(this);',
    '    this.B$2 = 0.0;',
    '  };',
    '  this.A$2 = function () {',
    '    var Result = false;',
    '    this.B$2 = 3.3;',
    '    this.A$2();',
    '    this.B$1 = true;',
    '    this.A = 17;',
    '    this.B$1 = this.A$1 > 1;',
    '    return Result;',
    '  };',
    '});',
    'this.e = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.e.B$2 = 5.3;',
    'if ($mod.e.A$2()) ;',
    '']));
end;

procedure TTestModule.TestClass_ReintroducedVar;
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
  Add('    procedure Some;');
  Add('    procedure Some(vA: longint);');
  Add('  end;');
  Add('procedure tcar.some;');
  Add('begin');
  Add('  Some;');
  Add('  Some(1);');
  Add('end;');
  Add('procedure tcar.some(va: longint); begin end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestClass_ReintroducedVar',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.Some = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TMobile", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.Some$1 = "";',
    '  };',
    '});',
    'rtl.createClass(this, "TCar", this.TMobile, function () {',
    '  this.Some$2 = function () {',
    '    this.Some$2();',
    '    this.Some$3(1);',
    '  };',
    '  this.Some$3 = function (vA) {',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClass_RaiseDescendant;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create(Msg: string);',
  '  end;',
  '  Exception = class',
  '  end;',
  '  EConvertError = class(Exception)',
  '  end;',
  'constructor TObject.Create(Msg: string); begin end;',
  'function AssertConv(Msg: string = ''def''): EConvertError; begin end;',
  'begin',
  '  raise Exception.Create(''Bar1'');',
  '  raise EConvertError.Create(''Bar2'');',
  '  raise AssertConv(''Bar2'');',
  '  raise AssertConv;',
  '']);
  ConvertProgram;
  CheckSource('TestClass_RaiseDescendant',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function (Msg) {',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "Exception", this.TObject, function () {',
    '});',
    'rtl.createClass(this, "EConvertError", this.Exception, function () {',
    '});',
    'this.AssertConv = function (Msg) {',
    '  var Result = null;',
    '  return Result;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    'throw $mod.Exception.$create("Create",["Bar1"]);',
    'throw $mod.EConvertError.$create("Create",["Bar2"]);',
    'throw $mod.AssertConv("Bar2");',
    'throw $mod.AssertConv("def");',
    '']));
end;

procedure TTestModule.TestClass_ExternalMethod;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'type',
    '  TObject = class',
    '  public',
    '    procedure Intern; external name ''$DoIntern'';',
    '  end;',
    '']),
    LinesToStr([
    '']));

  StartUnit(true);
  Add('interface');
  Add('uses unit2;');
  Add('type');
  Add('  TCar = class(TObject)');
  Add('  public');
  Add('    procedure Intern2; external name ''$DoIntern2'';');
  Add('    procedure DoIt;');
  Add('  end;');
  Add('implementation');
  Add('procedure tcar.doit;');
  Add('begin');
  Add('  Intern;');
  Add('  Intern();');
  Add('  Intern2;');
  Add('  Intern2();');
  Add('end;');
  Add('var Obj: TCar;');
  Add('begin');
  Add('  obj.intern;');
  Add('  obj.intern();');
  Add('  obj.intern2;');
  Add('  obj.intern2();');
  Add('  obj.doit;');
  Add('  obj.doit();');
  Add('  with obj do begin');
  Add('    Intern;');
  Add('    Intern();');
  Add('    Intern2;');
  Add('    Intern2();');
  Add('  end;');
  ConvertUnit;
  CheckSource('TestClass_ExternalMethod',
    LinesToStr([
    'var $impl = $mod.$impl;',
    'rtl.createClass(this, "TCar", pas.unit2.TObject, function () {',
    '    this.DoIt = function () {',
    '      this.$DoIntern();',
    '      this.$DoIntern();',
    '      this.$DoIntern2();',
    '      this.$DoIntern2();',
    '    };',
    '  });',
    '']),
    LinesToStr([ // this.$init
    '$impl.Obj.$DoIntern();',
    '$impl.Obj.$DoIntern();',
    '$impl.Obj.$DoIntern2();',
    '$impl.Obj.$DoIntern2();',
    '$impl.Obj.DoIt();',
    '$impl.Obj.DoIt();',
    'var $with = $impl.Obj;',
    '$with.$DoIntern();',
    '$with.$DoIntern();',
    '$with.$DoIntern2();',
    '$with.$DoIntern2();',
    '']),
    LinesToStr([ // implementation
    '$impl.Obj = null;',
    '']) );
end;

procedure TTestModule.TestClass_ExternalVirtualNameMismatchFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure DoIt; virtual; external name ''Foo'';');
  Add('  end;');
  Add('begin');
  SetExpectedPasResolverError('Virtual method name must match external',
    nVirtualMethodNameMustMatchExternal);
  ConvertProgram;
end;

procedure TTestModule.TestClass_ExternalOverrideFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure DoIt; virtual; external name ''DoIt'';');
  Add('  end;');
  Add('  TCar = class');
  Add('    procedure DoIt; override; external name ''DoIt'';');
  Add('  end;');
  Add('begin');
  SetExpectedPasResolverError('Invalid procedure modifier override,external',
    nInvalidXModifierY);
  ConvertProgram;
end;

procedure TTestModule.TestClass_ExternalVar;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    '{$modeswitch externalclass}',
    'type',
    '  TObject = class',
    '  public',
    '    Intern: longint external name ''$Intern'';',
    '    Bracket: longint external name ''["A B"]'';',
    '  end;',
    '']),
    LinesToStr([
    '']));

  StartUnit(true);
  Add([
  'interface',
  'uses unit2;',
  '{$modeswitch externalclass}',
  'type',
  '  TCar = class(tobject)',
  '  public',
  '    Intern2: longint external name ''$Intern2'';',
  '    procedure DoIt;',
  '  end;',
  'implementation',
  'procedure tcar.doit;',
  'begin',
  '  Intern:=Intern+1;',
  '  Intern2:=Intern2+2;',
  '  Bracket:=Bracket+3;',
  'end;',
  'var Obj: TCar;',
  'begin',
  '  obj.intern:=obj.intern+1;',
  '  obj.intern2:=obj.intern2+2;',
  '  obj.Bracket:=obj.Bracket+3;',
  '  with obj do begin',
  '    intern:=intern+1;',
  '    intern2:=intern2+2;',
  '    Bracket:=Bracket+3;',
  '  end;']);
  ConvertUnit;
  CheckSource('TestClass_ExternalVar',
    LinesToStr([
    'var $impl = $mod.$impl;',
    'rtl.createClass(this, "TCar", pas.unit2.TObject, function () {',
    '    this.DoIt = function () {',
    '      this.$Intern = this.$Intern + 1;',
    '      this.$Intern2 = this.$Intern2 + 2;',
    '      this["A B"] = this["A B"] + 3;',
    '    };',
    '  });',
    '']),
    LinesToStr([
    '$impl.Obj.$Intern = $impl.Obj.$Intern + 1;',
    '$impl.Obj.$Intern2 = $impl.Obj.$Intern2 + 2;',
    '$impl.Obj["A B"] = $impl.Obj["A B"] + 3;',
    'var $with = $impl.Obj;',
    '$with.$Intern = $with.$Intern + 1;',
    '$with.$Intern2 = $with.$Intern2 + 2;',
    '$with["A B"] = $with["A B"] + 3;',
    '']),
    LinesToStr([ // implementation
    '$impl.Obj = null;',
    '']));
end;

procedure TTestModule.TestClass_Const;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  TClass = class of TObject;',
  '  TObject = class',
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
  ConvertProgram;
  CheckSource('TestClass_Const',
    LinesToStr([
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.cI = 3;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '    if (this.cI === 4) ;',
    '    if (5 === this.cI) ;',
    '    if (this.cI === 6) ;',
    '    if (7 === this.cI) ;',
    '    if (this.cI === 11) ;',
    '    if (12 === this.cI) ;',
    '  };',
    '  this.DoMore = function () {',
    '    if (this.cI === 8) ;',
    '    if (this.cI === 9) ;',
    '    if (10 === this.cI) ;',
    '    if (11 === this.cI) ;',
    '    if (this.cI === 13) ;',
    '    if (14 === this.cI) ;',
    '  };',
    '});',
    'this.Obj = null;',
    'this.Cla = null;',
    '']),
    LinesToStr([
    'if ($mod.TObject.cI === 21) ;',
    'if ($mod.Obj.cI === 22) ;',
    'if ($mod.Cla.cI === 23) ;',
    'var $with = $mod.Obj;',
    'if ($with.cI === 24) ;',
    'var $with1 = $mod.TObject;',
    'if ($with1.cI === 25) ;',
    'var $with2 = $mod.Cla;',
    'if ($with2.cI === 26) ;',
    '']));
end;

procedure TTestModule.TestClass_ConstEnum;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red,blue);',
  '  TObject = class',
  '  end;',
  '  TAnimal = class',
  '  public',
  '    type TSubEnum = (light,dark);',
  '    const a = high(TEnum);',
  '    const b = high(TSubEnum);',
  '  end;',
  '  TBird = class(TAnimal)',
  '  public',
  '    const c = high(TEnum);',
  '    const d = high(TSubEnum);',
  '  end;',
  '  TAnt = class',
  '  public',
  '    const e = high(TEnum);',
  '    const f = high(TBird.TSubEnum);',
  '  end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClass_ConstEnum',
    LinesToStr([
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TAnimal", this.TObject, function () {',
    '  this.TSubEnum = {',
    '    "0": "light",',
    '    light: 0,',
    '    "1": "dark",',
    '    dark: 1',
    '  };',
    '  this.a = $mod.TEnum.blue;',
    '  this.b = this.TSubEnum.dark;',
    '});',
    'rtl.createClass(this, "TBird", this.TAnimal, function () {',
    '  this.c = $mod.TEnum.blue;',
    '  this.d = this.TSubEnum.dark;',
    '});',
    'rtl.createClass(this, "TAnt", this.TObject, function () {',
    '  this.e = $mod.TEnum.blue;',
    '  this.f = $mod.TAnimal.TSubEnum.dark;',
    '});',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestClass_LocalConstDuplicate_Prg;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    const cI: longint = 3;',
  '    procedure Fly;',
  '    procedure Run;',
  '  end;',
  '  TBird = class',
  '    procedure Go;',
  '  end;',
  'procedure tobject.fly;',
  'const cI: word = 4;',
  'begin',
  '  if cI=Self.cI then ;',
  'end;',
  'procedure tobject.run;',
  'const cI: word = 5;',
  'begin',
  '  if cI=Self.cI then ;',
  'end;',
  'procedure tbird.go;',
  'const cI: word = 6;',
  'begin',
  '  if cI=Self.cI then ;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClass_LocalConstDuplicate_Prg',
    LinesToStr([
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.cI = 3;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var cI$1 = 4;',
    '  this.Fly = function () {',
    '    if (cI$1 === this.cI) ;',
    '  };',
    '  var cI$2 = 5;',
    '  this.Run = function () {',
    '    if (cI$2 === this.cI) ;',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  var cI$3 = 6;',
    '  this.Go = function () {',
    '    if (cI$3 === this.cI) ;',
    '  };',
    '});',
    '']),
    LinesToStr([
    '']));
end;

procedure TTestModule.TestClass_LocalConstDuplicate_Unit;
begin
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TObject = class',
  '    const cI: longint = 3;',
  '    procedure Fly;',
  '    procedure Run;',
  '  end;',
  '  TBird = class',
  '    procedure Go;',
  '  end;',
  'implementation',
  'procedure tobject.fly;',
  'const cI: word = 4;',
  'begin',
  '  if cI=Self.cI then ;',
  'end;',
  'procedure tobject.run;',
  'const cI: word = 5;',
  'begin',
  '  if cI=Self.cI then ;',
  'end;',
  'procedure tbird.go;',
  'const cI: word = 6;',
  'begin',
  '  if cI=Self.cI then ;',
  'end;',
  '']);
  ConvertUnit;
  CheckSource('TestClass_LocalConstDuplicate_Unit',
    LinesToStr([
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.cI = 3;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var cI$1 = 4;',
    '  this.Fly = function () {',
    '    if (cI$1 === this.cI) ;',
    '  };',
    '  var cI$2 = 5;',
    '  this.Run = function () {',
    '    if (cI$2 === this.cI) ;',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  var cI$3 = 6;',
    '  this.Go = function () {',
    '    if (cI$3 === this.cI) ;',
    '  };',
    '});',
    '']),
    '',
    '');
end;

procedure TTestModule.TestClass_LocalVarSelfFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  'constructor tobject.create;',
  'var self: longint;',
  'begin',
  'end',
  'begin',
  '']);
  SetExpectedPasResolverError('Duplicate identifier "self" at (0)',nDuplicateIdentifier);
  ConvertProgram;
end;

procedure TTestModule.TestClass_ArgSelfFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoIt(Self: longint);',
  '  end;',
  'procedure tobject.doit(self: longint);',
  'begin',
  'end',
  'begin',
  '']);
  SetExpectedPasResolverError('Duplicate identifier "Self" at test1.pp(5,24)',nDuplicateIdentifier);
  ConvertProgram;
end;

procedure TTestModule.TestClass_NestedProcSelf;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    Key: longint;',
  '    class var State: longint;',
  '    procedure DoIt;',
  '    function GetSize: longint; virtual; abstract;',
  '    procedure SetSize(Value: longint); virtual; abstract;',
  '    property Size: longint read GetSize write SetSize;',
  '  end;',
  'procedure tobject.doit;',
  '  procedure Sub;',
  '  begin',
  '    key:=key+2;',
  '    self.key:=self.key+3;',
  '    state:=state+4;',
  '    self.state:=self.state+5;',
  '    tobject.state:=tobject.state+6;',
  '    size:=size+7;',
  '    self.size:=self.size+8;',
  '  end;',
  'begin',
  '  sub;',
  '  key:=key+12;',
  '  self.key:=self.key+13;',
  '  state:=state+14;',
  '  self.state:=self.state+15;',
  '  tobject.state:=tobject.state+16;',
  '  size:=size+17;',
  '  self.size:=self.size+18;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClass_NestedProcSelf',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.State = 0;',
    '  this.$init = function () {',
    '    this.Key = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '    var $Self = this;',
    '    function Sub() {',
    '      $Self.Key = $Self.Key + 2;',
    '      $Self.Key = $Self.Key + 3;',
    '      $mod.TObject.State = $Self.State + 4;',
    '      $mod.TObject.State = $Self.State + 5;',
    '      $mod.TObject.State = $mod.TObject.State + 6;',
    '      $Self.SetSize($Self.GetSize() + 7);',
    '      $Self.SetSize($Self.GetSize() + 8);',
    '    };',
    '    Sub();',
    '    this.Key = this.Key + 12;',
    '    $Self.Key = $Self.Key + 13;',
    '    $mod.TObject.State = this.State + 14;',
    '    $mod.TObject.State = $Self.State + 15;',
    '    $mod.TObject.State = $mod.TObject.State + 16;',
    '    this.SetSize(this.GetSize() + 17);',
    '    $Self.SetSize($Self.GetSize() + 18);',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClass_NestedProcSelf2;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    Key: longint;',
  '    class var State: longint;',
  '    function GetSize: longint; virtual; abstract;',
  '    procedure SetSize(Value: longint); virtual; abstract;',
  '    property Size: longint read GetSize write SetSize;',
  '  end;',
  '  TBird = class',
  '    procedure DoIt;',
  '  end;',
  'procedure tbird.doit;',
  '  procedure Sub;',
  '  begin',
  '    key:=key+2;',
  '    self.key:=self.key+3;',
  '    state:=state+4;',
  '    self.state:=self.state+5;',
  '    tobject.state:=tobject.state+6;',
  '    size:=size+7;',
  '    self.size:=self.size+8;',
  '  end;',
  'begin',
  '  sub;',
  '  key:=key+12;',
  '  self.key:=self.key+13;',
  '  state:=state+14;',
  '  self.state:=self.state+15;',
  '  tobject.state:=tobject.state+16;',
  '  size:=size+17;',
  '  self.size:=self.size+18;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClass_NestedProcSelf2',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.State = 0;',
    '  this.$init = function () {',
    '    this.Key = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt = function () {',
    '    var $Self = this;',
    '    function Sub() {',
    '      $Self.Key = $Self.Key + 2;',
    '      $Self.Key = $Self.Key + 3;',
    '      $mod.TObject.State = $Self.State + 4;',
    '      $mod.TObject.State = $Self.State + 5;',
    '      $mod.TObject.State = $mod.TObject.State + 6;',
    '      $Self.SetSize($Self.GetSize() + 7);',
    '      $Self.SetSize($Self.GetSize() + 8);',
    '    };',
    '    Sub();',
    '    this.Key = this.Key + 12;',
    '    $Self.Key = $Self.Key + 13;',
    '    $mod.TObject.State = this.State + 14;',
    '    $mod.TObject.State = $Self.State + 15;',
    '    $mod.TObject.State = $mod.TObject.State + 16;',
    '    this.SetSize(this.GetSize() + 17);',
    '    $Self.SetSize($Self.GetSize() + 18);',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClass_NestedProcClassSelf;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class var State: longint;',
  '    class procedure DoIt;',
  '    class function GetSize: longint; virtual; abstract;',
  '    class procedure SetSize(Value: longint); virtual; abstract;',
  '    class property Size: longint read GetSize write SetSize;',
  '  end;',
  'class procedure tobject.doit;',
  '  procedure Sub;',
  '  begin',
  '    state:=state+2;',
  '    self.state:=self.state+3;',
  '    tobject.state:=tobject.state+4;',
  '    size:=size+5;',
  '    self.size:=self.size+6;',
  '    tobject.size:=tobject.size+7;',
  '  end;',
  'begin',
  '  sub;',
  '  state:=state+12;',
  '  self.state:=self.state+13;',
  '  tobject.state:=tobject.state+14;',
  '  size:=size+15;',
  '  self.size:=self.size+16;',
  '  tobject.size:=tobject.size+17;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClass_NestedProcClassSelf',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.State = 0;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '    var $Self = this;',
    '    function Sub() {',
    '      $mod.TObject.State = $Self.State + 2;',
    '      $mod.TObject.State = $Self.State + 3;',
    '      $mod.TObject.State = $mod.TObject.State + 4;',
    '      $Self.SetSize($Self.GetSize() + 5);',
    '      $Self.SetSize($Self.GetSize() + 6);',
    '      $mod.TObject.SetSize($mod.TObject.GetSize() + 7);',
    '    };',
    '    Sub();',
    '    $mod.TObject.State = this.State + 12;',
    '    $mod.TObject.State = $Self.State + 13;',
    '    $mod.TObject.State = $mod.TObject.State + 14;',
    '    this.SetSize(this.GetSize() + 15);',
    '    $Self.SetSize($Self.GetSize() + 16);',
    '    $mod.TObject.SetSize($mod.TObject.GetSize() + 17);',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClass_NestedProcCallInherited;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function DoIt(k: boolean): longint; virtual;',
  '  end;',
  '  TBird = class',
  '    function DoIt(k: boolean): longint; override;',
  '  end;',
  'function tobject.doit(k: boolean): longint;',
  'begin',
  'end;',
  'function tbird.doit(k: boolean): longint;',
  '  procedure Sub;',
  '  begin',
  '    inherited DoIt(true);',
  //'    if inherited DoIt(false)=4 then ;',
  '  end;',
  'begin',
  '  Sub;',
  '  inherited;',
  '  inherited DoIt(true);',
  //'  if inherited DoIt(false)=14 then ;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClass_NestedProcCallInherited',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (k) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt = function (k) {',
    '    var $Self = this;',
    '    var Result = 0;',
    '    function Sub() {',
    '      $mod.TObject.DoIt.call($Self, true);',
    '    };',
    '    Sub();',
    '    $mod.TObject.DoIt.apply(this, arguments);',
    '    $mod.TObject.DoIt.call(this, true);',
    '    return Result;',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClass_TObjectFree;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    Obj: tobject;',
  '    procedure Free;',
  '    procedure Release;',
  '  end;',
  'procedure tobject.free;',
  'begin',
  'end;',
  'procedure tobject.release;',
  'begin',
  '  free;',
  '  if true then free;',
  'end;',
  'function DoIt(o: tobject): tobject;',
  'var l: tobject;',
  'begin',
  '  o.free;',
  '  o.free();',
  '  l.free;',
  '  l.free();',
  '  o.obj.free;',
  '  o.obj.free();',
  '  with o do obj.free;',
  '  with o do obj.free();',
  '  result.Free;',
  '  result.Free();',
  'end;',
  'var o: tobject;',
  '  a: array of tobject;',
  'begin',
  '  o.free;',
  '  o.obj.free;',
  '  a[1+2].free;',
  '']);
  ConvertProgram;
  CheckSource('TestClass_TObjectFree',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.Obj = null;',
    '  };',
    '  this.$final = function () {',
    '    this.Obj = undefined;',
    '  };',
    '  this.Free = function () {',
    '  };',
    '  this.Release = function () {',
    '    this.Free();',
    '    if (true) this.Free();',
    '  };',
    '});',
    'this.DoIt = function (o) {',
    '  var Result = null;',
    '  var l = null;',
    '  o = rtl.freeLoc(o);',
    '  o = rtl.freeLoc(o);',
    '  l = rtl.freeLoc(l);',
    '  l = rtl.freeLoc(l);',
    '  rtl.free(o, "Obj");',
    '  rtl.free(o, "Obj");',
    '  rtl.free(o, "Obj");',
    '  rtl.free(o, "Obj");',
    '  Result = rtl.freeLoc(Result);',
    '  Result = rtl.freeLoc(Result);',
    '  return Result;',
    '};',
    'this.o = null;',
    'this.a = [];',
    '']),
    LinesToStr([ // $mod.$main
    'rtl.free($mod, "o");',
    'rtl.free($mod.o, "Obj");',
    'rtl.free($mod.a, 1 + 2);',
    '']));
end;

procedure TTestModule.TestClass_TObjectFree_VarArg;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    Obj: tobject;',
  '    procedure Free;',
  '  end;',
  'procedure tobject.free;',
  'begin',
  'end;',
  'procedure DoIt(var o: tobject);',
  'begin',
  '  o.free;',
  '  o.free();',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClass_TObjectFree_VarArg',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.Obj = null;',
    '  };',
    '  this.$final = function () {',
    '    this.Obj = undefined;',
    '  };',
    '  this.Free = function () {',
    '  };',
    '});',
    'this.DoIt = function (o) {',
    '  o.set(rtl.freeLoc(o.get()));',
    '  o.set(rtl.freeLoc(o.get()));',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClass_TObjectFreeNewInstance;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create;',
  '    procedure Free;',
  '  end;',
  'constructor TObject.Create; begin end;',
  'procedure tobject.free; begin end;',
  'begin',
  '  with tobject.create do free;',
  '']);
  ConvertProgram;
  CheckSource('TestClass_TObjectFreeNewInstance',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '  this.Free = function () {',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    'var $with = $mod.TObject.$create("Create");',
    '$with=rtl.freeLoc($with);',
    '']));
end;

procedure TTestModule.TestClass_TObjectFreeLowerCase;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    destructor Destroy;',
  '    procedure Free;',
  '  end;',
  'destructor TObject.Destroy; begin end;',
  'procedure tobject.free; begin end;',
  'var o: tobject;',
  'begin',
  '  o.free;',
  '']);
  Converter.UseLowerCase:=true;
  ConvertProgram;
  CheckSource('TestClass_TObjectFreeLowerCase',
    LinesToStr([ // statements
    'rtl.createClass(this, "tobject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.tObjectDestroy = "destroy";',
    '  this.destroy = function () {',
    '  };',
    '  this.free = function () {',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    'rtl.free($mod, "o");',
    '']));
end;

procedure TTestModule.TestClass_TObjectFreeFunctionFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure Free;',
  '    function GetObj: tobject; virtual; abstract;',
  '  end;',
  'procedure tobject.free;',
  'begin',
  'end;',
  'var o: tobject;',
  'begin',
  '  o.getobj.free;',
  '']);
  SetExpectedPasResolverError(sFreeNeedsVar,nFreeNeedsVar);
  ConvertProgram;
end;

procedure TTestModule.TestClass_TObjectFreePropertyFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure Free;',
  '    FObj: TObject;',
  '    property Obj: tobject read FObj write FObj;',
  '  end;',
  'procedure tobject.free;',
  'begin',
  'end;',
  'var o: tobject;',
  'begin',
  '  o.obj.free;',
  '']);
  SetExpectedPasResolverError(sFreeNeedsVar,nFreeNeedsVar);
  ConvertProgram;
end;

procedure TTestModule.TestClass_ForIn;
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
  '  i, i2: TItem;',
  'begin',
  '  for i in b do i2:=i;']);
  ConvertProgram;
  CheckSource('TestClass_ForIn',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TEnumerator", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.FCurrent = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FCurrent = undefined;',
    '    $mod.TObject.$final.call(this);',
    '  };',
    '  this.MoveNext = function () {',
    '    var Result = false;',
    '    return Result;',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.GetEnumerator = function () {',
    '    var Result = null;',
    '    return Result;',
    '  };',
    '});',
    'this.b = null;',
    'this.i = null;',
    'this.i2 = null;'
    ]),
    LinesToStr([ // $mod.$main
    'var $in = $mod.b.GetEnumerator();',
    'try {',
    '  while ($in.MoveNext()){',
    '    $mod.i = $in.FCurrent;',
    '    $mod.i2 = $mod.i;',
    '  }',
    '} finally {',
    '  $in = rtl.freeLoc($in)',
    '};',
    '']));
end;

procedure TTestModule.TestClass_DispatchMessage;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    {$DispatchField DispInt}',
  '    procedure Dispatch(var Msg); virtual; abstract;',
  '    {$DispatchStrField DispStr}',
  '    procedure DispatchStr(var Msg); virtual; abstract;',
  '  end;',
  '  THopMsg = record',
  '    DispInt: longint;',
  '  end;',
  '  TPutMsg = record',
  '    DispStr: string;',
  '  end;',
  '  TBird = class',
  '    procedure Fly(var Msg); virtual; abstract; message 2;',
  '    procedure Run; overload; virtual; abstract;',
  '    procedure Run(var Msg); overload; message ''Fast'';',
  '    procedure Hop(var Msg: THopMsg); virtual; abstract; message 3;',
  '    procedure Put(var Msg: TPutMsg); virtual; abstract; message ''foo'';',
  '  end;',
  'procedure TBird.Run(var Msg);',
  'begin',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClass_Message',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.recNewT(this, "THopMsg", function () {',
    '  this.DispInt = 0;',
    '  this.$eq = function (b) {',
    '    return this.DispInt === b.DispInt;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.DispInt = s.DispInt;',
    '    return this;',
    '  };',
    '});',
    'rtl.recNewT(this, "TPutMsg", function () {',
    '  this.DispStr = "";',
    '  this.$eq = function (b) {',
    '    return this.DispStr === b.DispStr;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.DispStr = s.DispStr;',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.Run$1 = function (Msg) {',
    '  };',
    '  this.$msgint = {',
    '    "2": "Fly",',
    '    "3": "Hop"',
    '  };',
    '  this.$msgstr = {',
    '    Fast: "Run$1",',
    '    foo: "Put"',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClass_Message_DuplicateIntFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure Fly(var Msg); virtual; abstract; message 3;',
  '    procedure Run(var Msg); virtual; abstract; message 1+2;',
  '  end;',
  'begin',
  '']);
  SetExpectedPasResolverError('Duplicate message id "3" at test1.pp(5,56)',nDuplicateMessageIdXAtY);
  ConvertProgram;
end;

procedure TTestModule.TestClass_DispatchMessage_WrongFieldNameFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    {$dispatchfield Msg}',
  '    procedure Dispatch(var Msg); virtual; abstract;',
  '  end;',
  '  TFlyMsg = record',
  '    FlyId: longint;',
  '  end;',
  '  TBird = class',
  '    procedure Fly(var Msg: TFlyMsg); virtual; abstract; message 3;',
  '  end;',
  'begin',
  '']);
  ConvertProgram;
  CheckHint(mtWarning,nDispatchRequiresX,'Dispatch requires record field "Msg"');
end;

procedure TTestModule.TestClassOf_Create;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    constructor Create;');
  Add('  end;');
  Add('  TClass = class of TObject;');
  Add('constructor tobject.create; begin end;');
  Add('var');
  Add('  Obj: tobject;');
  Add('  C: tclass;');
  Add('begin');
  Add('  obj:=C.create;');
  Add('  with c do obj:=create;');
  ConvertProgram;
  CheckSource('TestClassOf_Create',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '});',
    'this.Obj = null;',
    'this.C = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.Obj = $mod.C.$create("Create");',
    'var $with = $mod.C;',
    '$mod.Obj = $with.$create("Create");',
    '']));
end;

procedure TTestModule.TestClassOf_Call;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class procedure DoIt;');
  Add('  end;');
  Add('  TClass = class of TObject;');
  Add('class procedure tobject.doit; begin end;');
  Add('var');
  Add('  C: tclass;');
  Add('begin');
  Add('  c.doit;');
  Add('  with c do doit;');
  ConvertProgram;
  CheckSource('TestClassOf_Call',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '  };',
    '});',
    'this.C = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.C.DoIt();',
    'var $with = $mod.C;',
    '$with.DoIt();',
    '']));
end;

procedure TTestModule.TestClassOf_Assign;
begin
  StartProgram(false);
  Add('type');
  Add('  TClass = class of TObject;');
  Add('  TObject = class');
  Add('    ClassType: TClass; ');
  Add('  end;');
  Add('var');
  Add('  Obj: tobject;');
  Add('  C: tclass;');
  Add('begin');
  Add('  c:=nil;');
  Add('  c:=obj.classtype;');
  ConvertProgram;
  CheckSource('TestClassOf_Assign',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.ClassType = null;',
    '  };',
    '  this.$final = function () {',
    '    this.ClassType = undefined;',
    '  };',
    '});',
    'this.Obj = null;',
    'this.C = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.C = null;',
    '$mod.C = $mod.Obj.ClassType;',
    '']));
end;

procedure TTestModule.TestClassOf_Is;
begin
  StartProgram(false);
  Add('type');
  Add('  TClass = class of TObject;');
  Add('  TObject = class');
  Add('  end;');
  Add('  TCar = class');
  Add('  end;');
  Add('  TCars = class of TCar;');
  Add('var');
  Add('  Obj: tobject;');
  Add('  C: tclass;');
  Add('  Cars: tcars;');
  Add('begin');
  Add('  if c is tcar then ;');
  Add('  if c is tcars then ;');
  ConvertProgram;
  CheckSource('TestClassOf_Is',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TCar", this.TObject, function () {',
    '});',
    'this.Obj = null;',
    'this.C = null;',
    'this.Cars = null;'
    ]),
    LinesToStr([ // $mod.$main
    'if(rtl.is($mod.C,$mod.TCar));',
    'if(rtl.is($mod.C,$mod.TCar));',
    '']));
end;

procedure TTestModule.TestClassOf_Compare;
begin
  StartProgram(false);
  Add('type');
  Add('  TClass = class of TObject;');
  Add('  TObject = class');
  Add('    ClassType: TClass; ');
  Add('  end;');
  Add('var');
  Add('  b: boolean;');
  Add('  Obj: tobject;');
  Add('  C: tclass;');
  Add('begin');
  Add('  b:=c=nil;');
  Add('  b:=nil=c;');
  Add('  b:=c=obj.classtype;');
  Add('  b:=obj.classtype=c;');
  Add('  b:=c=TObject;');
  Add('  b:=TObject=c;');
  Add('  b:=c<>nil;');
  Add('  b:=nil<>c;');
  Add('  b:=c<>obj.classtype;');
  Add('  b:=obj.classtype<>c;');
  Add('  b:=c<>TObject;');
  Add('  b:=TObject<>c;');
  ConvertProgram;
  CheckSource('TestClassOf_Compare',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.ClassType = null;',
    '  };',
    '  this.$final = function () {',
    '    this.ClassType = undefined;',
    '  };',
    '});',
    'this.b = false;',
    'this.Obj = null;',
    'this.C = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.b = $mod.C === null;',
    '$mod.b = null === $mod.C;',
    '$mod.b = $mod.C === $mod.Obj.ClassType;',
    '$mod.b = $mod.Obj.ClassType === $mod.C;',
    '$mod.b = $mod.C === $mod.TObject;',
    '$mod.b = $mod.TObject === $mod.C;',
    '$mod.b = $mod.C !== null;',
    '$mod.b = null !== $mod.C;',
    '$mod.b = $mod.C !== $mod.Obj.ClassType;',
    '$mod.b = $mod.Obj.ClassType !== $mod.C;',
    '$mod.b = $mod.C !== $mod.TObject;',
    '$mod.b = $mod.TObject !== $mod.C;',
    '']));
end;

procedure TTestModule.TestClassOf_ClassVar;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class var id: longint;');
  Add('  end;');
  Add('  TClass = class of TObject;');
  Add('var');
  Add('  C: tclass;');
  Add('begin');
  Add('  C.id:=C.id;');
  ConvertProgram;
  CheckSource('TestClassOf_ClassVar',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.id = 0;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.C = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.TObject.id = $mod.C.id;',
    '']));
end;

procedure TTestModule.TestClassOf_ClassMethod;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class function DoIt(i: longint = 0): longint;');
  Add('  end;');
  Add('  TClass = class of TObject;');
  Add('class function tobject.doit(i: longint = 0): longint; begin end;');
  Add('var');
  Add('  i: longint;');
  Add('  C: tclass;');
  Add('begin');
  Add('  C.DoIt;');
  Add('  C.DoIt();');
  Add('  i:=C.DoIt;');
  Add('  i:=C.DoIt();');
  ConvertProgram;
  CheckSource('TestClassOf_ClassMethod',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (i) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'this.i = 0;',
    'this.C = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.C.DoIt(0);',
    '$mod.C.DoIt(0);',
    '$mod.i = $mod.C.DoIt(0);',
    '$mod.i = $mod.C.DoIt(0);',
    '']));
end;

procedure TTestModule.TestClassOf_ClassProperty;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class var FA: longint;',
  '    class function GetA: longint;',
  '    class procedure SetA(Value: longint);',
  '    class property pA: longint read fa write fa;',
  '    class property pB: longint read geta write seta;',
  '  end;',
  '  TObjectClass = class of tobject;',
  'class function tobject.geta: longint; begin end;',
  'class procedure tobject.seta(value: longint); begin end;',
  'var',
  '  b: boolean;',
  '  Obj: tobject;',
  '  Cla: tobjectclass;',
  'begin',
  '  obj.pa:=obj.pa;',
  '  obj.pb:=obj.pb;',
  '  b:=obj.pa=4;',
  '  b:=obj.pb=obj.pb;',
  '  b:=5=obj.pa;',
  '  cla.pa:=6;',
  '  cla.pa:=cla.pa;',
  '  cla.pb:=cla.pb;',
  '  b:=cla.pa=7;',
  '  b:=cla.pb=cla.pb;',
  '  b:=8=cla.pa;',
  '  tobject.pa:=9;',
  '  tobject.pb:=tobject.pb;',
  '  b:=tobject.pa=10;',
  '  b:=11=tobject.pa;',
  '']);
  ConvertProgram;
  CheckSource('TestClassOf_ClassProperty',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.FA = 0;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetA = function () {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  this.SetA = function (Value) {',
    '  };',
    '});',
    'this.b = false;',
    'this.Obj = null;',
    'this.Cla = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.TObject.FA = $mod.Obj.FA;',
    '$mod.Obj.$class.SetA($mod.Obj.$class.GetA());',
    '$mod.b = $mod.Obj.FA === 4;',
    '$mod.b = $mod.Obj.$class.GetA() === $mod.Obj.$class.GetA();',
    '$mod.b = 5 === $mod.Obj.FA;',
    '$mod.TObject.FA = 6;',
    '$mod.TObject.FA = $mod.Cla.FA;',
    '$mod.Cla.SetA($mod.Cla.GetA());',
    '$mod.b = $mod.Cla.FA === 7;',
    '$mod.b = $mod.Cla.GetA() === $mod.Cla.GetA();',
    '$mod.b = 8 === $mod.Cla.FA;',
    '$mod.TObject.FA = 9;',
    '$mod.TObject.SetA($mod.TObject.GetA());',
    '$mod.b = $mod.TObject.FA === 10;',
    '$mod.b = 11 === $mod.TObject.FA;',
    '']));
end;

procedure TTestModule.TestClassOf_ClassMethodSelf;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class var GlobalId: longint;');
  Add('    class procedure ProcA;');
  Add('  end;');
  Add('class procedure tobject.proca;');
  Add('var b: boolean;');
  Add('begin');
  Add('  b:=self=nil;');
  Add('  b:=self.globalid=3;');
  Add('  b:=4=self.globalid;');
  Add('  self.globalid:=5;');
  Add('  self.proca;');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestClassOf_ClassMethodSelf',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.GlobalId = 0;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.ProcA = function () {',
    '    var b = false;',
    '    b = this === null;',
    '    b = this.GlobalId === 3;',
    '    b = 4 === this.GlobalId;',
    '    $mod.TObject.GlobalId = 5;',
    '    this.ProcA();',
    '  };',
    '});'
    ]),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassOf_TypeCast;
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
  ConvertProgram;
  CheckSource('TestClassOf_TypeCast',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '    this.DoIt();',
    '    this.DoIt$1();',
    '  };',
    '});',
    'rtl.createClass(this, "TMobile", this.TObject, function () {',
    '  this.DoIt$1 = function () {',
    '    this.DoIt();',
    '    this.DoIt$1();',
    '    this.DoIt$2();',
    '  };',
    '});',
    'rtl.createClass(this, "TCar", this.TMobile, function () {',
    '  this.DoIt$2 = function () {',
    '  };',
    '});',
    'this.ObjC = null;',
    'this.MobileC = null;',
    'this.CarC = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.ObjC.DoIt();',
    '$mod.MobileC.DoIt$1();',
    '$mod.CarC.DoIt$2();',
    '$mod.ObjC.DoIt();',
    '$mod.ObjC.DoIt$1();',
    '$mod.ObjC.DoIt$2();',
    '$mod.MobileC.DoIt();',
    '$mod.MobileC.DoIt$1();',
    '$mod.MobileC.DoIt$2();',
    '$mod.CarC.DoIt();',
    '$mod.CarC.DoIt$1();',
    '$mod.CarC.DoIt$2();',
    '']));
end;

procedure TTestModule.TestClassOf_ImplicitFunctionCall;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    function CurNow: longint; ');
  Add('    class function Now: longint; ');
  Add('  end;');
  Add('function TObject.CurNow: longint; begin end;');
  Add('class function TObject.Now: longint; begin end;');
  Add('var');
  Add('  Obj: tobject;');
  Add('  vI: longint;');
  Add('begin');
  Add('  obj.curnow;');
  Add('  vi:=obj.curnow;');
  Add('  tobject.now;');
  Add('  vi:=tobject.now;');
  ConvertProgram;
  CheckSource('TestClassOf_ImplicitFunctionCall',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.CurNow = function () {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  this.Now = function () {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;',
    'this.vI = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Obj.CurNow();',
    '$mod.vI = $mod.Obj.CurNow();',
    '$mod.TObject.Now();',
    '$mod.vI = $mod.TObject.Now();',
    '']));
end;

procedure TTestModule.TestClassOf_Const;
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
  ConvertProgram;
  CheckSource('TestClassOf_Const',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "THawk", this.TObject, function () {',
    '});',
    'this.Hawk = this.THawk;',
    'this.DefaultBirdClasses = [this.TObject, this.THawk];',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestNestedClass_Alias;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    type TNested = type longint;',
  '  end;',
  'type TAlias = type tobject.tnested;',
  'var i: tobject.tnested = 3;',
  'var j: TAlias = 4;',
  'begin',
  '  if typeinfo(TAlias)=nil then ;',
  '  if typeinfo(tobject.tnested)=nil then ;',
  '']);
  ConvertProgram;
  CheckSource('TestNestedClass_Alias',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  $mod.$rtti.$inherited("TObject.TNested", rtl.longint, {});',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.$rtti.$inherited("TAlias", this.$rtti["TObject.TNested"], {});',
    'this.i = 3;',
    'this.j = 4;',
    '']),
    LinesToStr([ // $mod.$main
    'if ($mod.$rtti["TAlias"] === null) ;',
    'if ($mod.$rtti["TObject.TNested"] === null) ;',
    '']));
end;

procedure TTestModule.TestNestedClass_Record;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    type TPoint = record',
  '       x,y: byte;',
  '    end;',
  '    procedure DoIt(t: TPoint);',
  '  end;',
  'procedure tobject.DoIt(t: TPoint);',
  'var p: TPoint;',
  'begin',
  '  t.x:=t.y;',
  '  p:=t;',
  'end;',
  'var',
  '  p: tobject.tpoint = (x:2; y:4);',
  '  o: TObject;',
  'begin',
  '  p:=p;',
  '  o.doit(p);',
  '']);
  ConvertProgram;
  CheckSource('TestNestedClass_Record',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  rtl.recNewT(this, "TPoint", function () {',
    '    this.x = 0;',
    '    this.y = 0;',
    '    this.$eq = function (b) {',
    '      return (this.x === b.x) && (this.y === b.y);',
    '    };',
    '    this.$assign = function (s) {',
    '      this.x = s.x;',
    '      this.y = s.y;',
    '      return this;',
    '    };',
    '    var $r = $mod.$rtti.$Record("TObject.TPoint", {});',
    '    $r.addField("x", rtl.byte);',
    '    $r.addField("y", rtl.byte);',
    '  });',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (t) {',
    '    var p = this.TPoint.$new();',
    '    t.x = t.y;',
    '    p.$assign(t);',
    '  };',
    '});',
    'this.p = this.TObject.TPoint.$clone({',
    '  x: 2,',
    '  y: 4',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p.$assign($mod.p);',
    '$mod.o.DoIt($mod.TObject.TPoint.$clone($mod.p));',
    '']));
end;

procedure TTestModule.TestNestedClass_Class;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  '  TBird = class',
  '    type TLeg = class',
  '      FId: longint;',
  '      constructor Create;',
  '      function Create(i: longint): TLeg;',
  '    end;',
  '    function DoIt(b: TBird): Tleg;',
  '  end;',
  'constructor tbird.tleg.create;',
  'begin',
  '  FId:=3;',
  'end;',
  'function tbird.tleg.Create(i: longint): TLeg;',
  'begin',
  '  Create;',
  '  Result:=TLeg.Create;',
  '  Result:=TBird.TLeg.Create;',
  '  Result:=Create(3);',
  '  FId:=i;',
  'end;',
  'function tbird.DoIt(b: tbird): tleg;',
  'begin',
  '  Result.Create;',
  '  Result:=TLeg.Create;',
  '  Result:=TBird.TLeg.Create;',
  '  Result:=Result.Create(3);',
  'end;',
  'var',
  '  b: Tbird.tleg;',
  'begin',
  '  b.Create;',
  '  b:=TBird.TLeg.Create;',
  '  b:=b.Create(3);',
  '']);
  ConvertProgram;
  CheckSource('TestNestedClass_Class',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  rtl.createClass(this, "TLeg", $mod.TObject, function () {',
    '    this.$init = function () {',
    '      $mod.TObject.$init.call(this);',
    '      this.FId = 0;',
    '    };',
    '    this.Create = function () {',
    '      this.FId = 3;',
    '      return this;',
    '    };',
    '    this.Create$1 = function (i) {',
    '      var Result = null;',
    '      this.Create();',
    '      Result = $mod.TBird.TLeg.$create("Create");',
    '      Result = $mod.TBird.TLeg.$create("Create");',
    '      Result = this.Create$1(3);',
    '      this.FId = i;',
    '      return Result;',
    '    };',
    '  }, "TBird.TLeg");',
    '  this.DoIt = function (b) {',
    '    var Result = null;',
    '    Result.Create();',
    '    Result = this.TLeg.$create("Create");',
    '    Result = $mod.TBird.TLeg.$create("Create");',
    '    Result = Result.Create$1(3);',
    '    return Result;',
    '  };',
    '});',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.b.Create();',
    '$mod.b = $mod.TBird.TLeg.$create("Create");',
    '$mod.b = $mod.b.Create$1(3);',
    '']));
end;

procedure TTestModule.TestExternalClass_Var;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtObj''',
  '    Id: longint external name ''$Id'';',
  '    B: longint;',
  '  end;',
  'var Obj: TExtA;',
  'begin',
  '  obj.id:=obj.id+1;',
  '  obj.B:=obj.B+1;']);
  ConvertProgram;
  CheckSource('TestExternalClass_Var',
    LinesToStr([ // statements
    'this.Obj = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Obj.$Id = $mod.Obj.$Id + 1;',
    '$mod.Obj.B = $mod.Obj.B + 1;',
    '']));
end;

procedure TTestModule.TestExternalClass_Const;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtObj''',
  '    const Two: longint = 2;',
  '    const Three = 3;',
  '    const Id: longint;',
  '  end;',
  '  TExtB = class external name ''ExtB''',
  '    A: TExtA;',
  '  end;',
  'var',
  '  A: texta;',
  '  B: textb;',
  '  i: longint;',
  'begin',
  '  i:=a.two;',
  '  i:=texta.two;',
  '  i:=a.three;',
  '  i:=texta.three;',
  '  i:=a.id;',
  '  i:=texta.id;',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_Const',
    LinesToStr([ // statements
    'this.A = null;',
    'this.B = null;',
    'this.i = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.i = 2;',
    '$mod.i = 2;',
    '$mod.i = 3;',
    '$mod.i = 3;',
    '$mod.i = $mod.A.Id;',
    '$mod.i = ExtObj.Id;',
    '']));
end;

procedure TTestModule.TestExternalClass_Dollar;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''$''',
  '    Id: longint external name ''$'';',
  '    function Bla(i: longint): longint; external name ''$'';',
  '  end;',
  'function dollar(k: longint): longint; external name ''$'';',
  'var Obj: TExtA;',
  'begin',
  '  dollar(1);',
  '  obj.id:=obj.id+2;',
  '  obj.Bla(3);',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_Dollar',
    LinesToStr([ // statements
    'this.Obj = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$(1);',
    '$mod.Obj.$ = $mod.Obj.$ + 2;',
    '$mod.Obj.$(3);',
    '']));
end;

procedure TTestModule.TestExternalClass_DuplicateVarFail;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('    Id: longint external name ''$Id'';');
  Add('  end;');
  Add('  TExtB = class external ''lib'' name ''ExtB''(TExtA)');
  Add('    Id: longint;');
  Add('  end;');
  Add('begin');
  SetExpectedPasResolverError('Duplicate identifier "Id" at test1.pp(6,5)',nDuplicateIdentifier);
  ConvertProgram;
end;

procedure TTestModule.TestExternalClass_Method;
begin
  StartProgram(false);
  Add(['{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtObj''',
  '    procedure DoIt(Id: longint = 1); external name ''$Execute'';',
  '    procedure DoSome(Id: longint = 1);',
  '  end;',
  'var Obj: texta;',
  'begin',
  '  obj.doit;',
  '  obj.doit();',
  '  obj.doit(2);',
  '  with obj do begin',
  '    doit;',
  '    doit();',
  '    doit(3);',
  '  end;']);
  ConvertProgram;
  CheckSource('TestExternalClass_Method',
    LinesToStr([ // statements
    'this.Obj = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Obj.$Execute(1);',
    '$mod.Obj.$Execute(1);',
    '$mod.Obj.$Execute(2);',
    'var $with = $mod.Obj;',
    '$with.$Execute(1);',
    '$with.$Execute(1);',
    '$with.$Execute(3);',
    '']));
end;

procedure TTestModule.TestExternalClass_ClassMethod;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtObj''',
  '    class procedure DoIt(Id: longint = 1); external name ''$Execute'';',
  '  end;',
  '  TExtB = TExtA;',
  'var p: Pointer;',
  'begin',
  '  texta.doit;',
  '  texta.doit();',
  '  texta.doit(2);',
  '  p:=@TExtA.DoIt;',
  '  with texta do begin',
  '    doit;',
  '    doit();',
  '    doit(3);',
  '    p:=@DoIt;',
  '  end;',
  '  textb.doit;',
  '  textb.doit();',
  '  textb.doit(4);',
  '  with textb do begin',
  '    doit;',
  '    doit();',
  '    doit(5);',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_ClassMethod',
    LinesToStr([ // statements
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    'ExtObj.$Execute(1);',
    'ExtObj.$Execute(1);',
    'ExtObj.$Execute(2);',
    '$mod.p = rtl.createCallback(ExtObj, "$Execute");',
    'ExtObj.$Execute(1);',
    'ExtObj.$Execute(1);',
    'ExtObj.$Execute(3);',
    '$mod.p = rtl.createCallback(ExtObj, "$Execute");',
    'ExtObj.$Execute(1);',
    'ExtObj.$Execute(1);',
    'ExtObj.$Execute(4);',
    'ExtObj.$Execute(1);',
    'ExtObj.$Execute(1);',
    'ExtObj.$Execute(5);',
    '']));
end;

procedure TTestModule.TestExternalClass_ClassMethodStatic;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtObj''',
  '    class procedure DoIt(Id: longint = 1); static;',
  '  end;',
  'var p: Pointer;',
  'begin',
  '  texta.doit;',
  '  texta.doit();',
  '  texta.doit(2);',
  '  p:=@TExtA.DoIt;',
  '  with texta do begin',
  '    doit;',
  '    doit();',
  '    doit(3);',
  '    p:=@DoIt;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_ClassMethodStatic',
    LinesToStr([ // statements
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    'ExtObj.DoIt(1);',
    'ExtObj.DoIt(1);',
    'ExtObj.DoIt(2);',
    '$mod.p = ExtObj.DoIt;',
    'ExtObj.DoIt(1);',
    'ExtObj.DoIt(1);',
    'ExtObj.DoIt(3);',
    '$mod.p = ExtObj.DoIt;',
    '']));
end;

procedure TTestModule.TestExternalClass_FunctionResultInTypeCast;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TBird = class external name ''Array''',
  '  end;',
  'function GetPtr: Pointer;',
  'begin',
  'end;',
  'procedure Write(const p);',
  'begin',
  'end;',
  'procedure WriteLn; varargs;',
  'begin',
  'end;',
  'begin',
  '  if TBird(GetPtr)=nil then ;',
  '  Write(GetPtr);',
  '  WriteLn(GetPtr);',
  '  Write(TBird(GetPtr));',
  '  WriteLn(TBird(GetPtr));',
  '']);
  ConvertProgram;
  CheckSource('TestFunctionResultInTypeCast',
    LinesToStr([ // statements
    'this.GetPtr = function () {',
    '  var Result = null;',
    '  return Result;',
    '};',
    'this.Write = function (p) {',
    '};',
    'this.WriteLn = function () {',
    '};',
    '']),
    LinesToStr([
    'if ($mod.GetPtr() === null) ;',
    '$mod.Write($mod.GetPtr());',
    '$mod.WriteLn($mod.GetPtr());',
    '$mod.Write($mod.GetPtr());',
    '$mod.WriteLn($mod.GetPtr());',
    '']));
end;

procedure TTestModule.TestExternalClass_NonExternalOverride;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtObjA''',
  '    procedure ProcA; virtual;',
  '    procedure ProcB; virtual;',
  '  end;',
  '  TExtB = class external name ''ExtObjB'' (TExtA)',
  '  end;',
  '  TExtC = class (TExtB)',
  '    procedure ProcA; override;',
  '  end;',
  'procedure TExtC.ProcA;',
  'begin',
  '  ProcA;',
  '  Self.ProcA;',
  '  ProcB;',
  '  Self.ProcB;',
  'end;',
  'var',
  '  A: texta;',
  '  B: textb;',
  '  C: textc;',
  'begin',
  '  a.proca;',
  '  b.proca;',
  '  c.proca;']);
  ConvertProgram;
  CheckSource('TestExternalClass_NonExternalOverride',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TExtC", ExtObjB, "", function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.ProcA = function () {',
    '    this.ProcA();',
    '    this.ProcA();',
    '    this.ProcB();',
    '    this.ProcB();',
    '  };',
    '});',
    'this.A = null;',
    'this.B = null;',
    'this.C = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.A.ProcA();',
    '$mod.B.ProcA();',
    '$mod.C.ProcA();',
    '']));
end;

procedure TTestModule.TestExternalClass_OverloadHint;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtObjA''',
  '    procedure DoIt;',
  '    procedure DoIt(i: longint);',
  '  end;',
  'begin',
  '']);
  ConvertProgram;
  CheckResolverUnexpectedHints(true);
  CheckSource('TestExternalClass_OverloadHint',
    LinesToStr([ // statements
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestExternalClass_SameNamePublishedProperty;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  JSwiper = class external name ''Swiper''',
  '    constructor New;',
  '  end;',
  '  TObject = class',
  '  private',
  '    FSwiper: JSwiper;',
  '  published',
  '    property Swiper: JSwiper read FSwiper write FSwiper;',
  '  end;',
  'begin',
  '  JSwiper.new;',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_SameNamePublishedProperty',
    LinesToStr([ // statements
    'this.$rtti.$ExtClass("JSwiper", {',
    '  jsclass: "Swiper"',
    '});',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FSwiper = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FSwiper = undefined;',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addProperty("Swiper", 0, $mod.$rtti["JSwiper"], "FSwiper", "FSwiper");',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    'new Swiper();',
    '']));
end;

procedure TTestModule.TestExternalClass_Property;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtA''',
  '    function getYear: longint;',
  '    procedure setYear(Value: longint);',
  '    property Year: longint read getyear write setyear;',
  '  end;',
  '  TExtB = class (TExtA)',
  '    procedure OtherSetYear(Value: longint);',
  '    property year write othersetyear;',
  '  end;',
  'procedure textb.othersetyear(value: longint);',
  'begin',
  '  setYear(Value+4);',
  'end;',
  'var',
  '  A: texta;',
  '  B: textb;',
  'begin',
  '  a.year:=a.year+1;',
  '  b.year:=b.year+2;']);
  ConvertProgram;
  CheckSource('TestExternalClass_NonExternalOverride',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TExtB", ExtA, "", function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.OtherSetYear = function (Value) {',
    '    this.setYear(Value+4);',
    '  };',
    '});',
    'this.A = null;',
    'this.B = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.A.setYear($mod.A.getYear()+1);',
    '$mod.B.OtherSetYear($mod.B.getYear()+2);',
    '']));
end;

procedure TTestModule.TestExternalClass_PropertyDate;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtA''',
  '  end;',
  '  TExtB = class (TExtA)',
  '    FDate: string;',
  '    property Date: string read FDate write FDate;',
  '    property ExtA: string read FDate write FDate;',
  '  end;',
  '  {$M+}',
  '  TObject = class',
  '    FDate: string;',
  '  published',
  '    property Date: string read FDate write FDate;',
  '    property ExtA: string read FDate write FDate;',
  '  end;',
  'var',
  '  B: textb;',
  '  o: TObject;',
  'begin',
  '  b.date:=b.exta;',
  '  o.date:=o.exta;']);
  ConvertProgram;
  CheckSource('TestExternalClass_PropertyDate',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TExtB", ExtA, "", function () {',
    '  this.$init = function () {',
    '    this.FDate = "";',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FDate = "";',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addField("FDate", rtl.string);',
    '  $r.addProperty("Date", 0, rtl.string, "FDate", "FDate");',
    '  $r.addProperty("ExtA", 0, rtl.string, "FDate", "FDate");',
    '});',
    'this.B = null;',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.B.FDate = $mod.B.FDate;',
    '$mod.o.FDate = $mod.o.FDate;',
    '']));
end;

procedure TTestModule.TestExternalClass_ClassProperty;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('    class function getYear: longint;');
  Add('    class procedure setYear(Value: longint);');
  Add('    class property Year: longint read getyear write setyear;');
  Add('  end;');
  Add('  TExtB = class (TExtA)');
  Add('    class function GetCentury: longint;');
  Add('    class procedure SetCentury(Value: longint);');
  Add('    class property Century: longint read getcentury write setcentury;');
  Add('  end;');
  Add('class function textb.getcentury: longint;');
  Add('begin');
  Add('end;');
  Add('class procedure textb.setcentury(value: longint);');
  Add('begin');
  Add('  setyear(value+11);');
  Add('  texta.year:=texta.year+12;');
  Add('  year:=year+13;');
  Add('  textb.century:=textb.century+14;');
  Add('  century:=century+15;');
  Add('end;');
  Add('var');
  Add('  A: texta;');
  Add('  B: textb;');
  Add('begin');
  Add('  texta.year:=texta.year+1;');
  Add('  textb.year:=textb.year+2;');
  Add('  TextA.year:=TextA.year+3;');
  Add('  b.year:=b.year+4;');
  Add('  textb.century:=textb.century+5;');
  Add('  b.century:=b.century+6;');
  ConvertProgram;
  CheckSource('TestExternalClass_ClassProperty',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TExtB", ExtA, "", function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetCentury = function () {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  this.SetCentury = function (Value) {',
    '    this.setYear(Value + 11);',
    '    ExtA.setYear(ExtA.getYear() + 12);',
    '    this.setYear(this.getYear() + 13);',
    '    $mod.TExtB.SetCentury($mod.TExtB.GetCentury() + 14);',
    '    this.SetCentury(this.GetCentury() + 15);',
    '  };',
    '});',
    'this.A = null;',
    'this.B = null;',
    '']),
    LinesToStr([ // $mod.$main
    'ExtA.setYear(ExtA.getYear() + 1);',
    '$mod.TExtB.setYear($mod.TExtB.getYear() + 2);',
    'ExtA.setYear(ExtA.getYear() + 3);',
    '$mod.B.setYear($mod.B.getYear() + 4);',
    '$mod.TExtB.SetCentury($mod.TExtB.GetCentury() + 5);',
    '$mod.B.$class.SetCentury($mod.B.$class.GetCentury() + 6);',
    '']));
end;

procedure TTestModule.TestExternalClass_ClassOf;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('    procedure ProcA; virtual;');
  Add('    procedure ProcB; virtual;');
  Add('  end;');
  Add('  TExtAClass = class of TExtA;');
  Add('  TExtB = class external name ''ExtB'' (TExtA)');
  Add('  end;');
  Add('  TExtBClass = class of TExtB;');
  Add('  TExtC = class (TExtB)');
  Add('    procedure ProcA; override;');
  Add('  end;');
  Add('  TExtCClass = class of TExtC;');
  Add('procedure TExtC.ProcA; begin end;');
  Add('var');
  Add('  A: texta; ClA: TExtAClass;');
  Add('  B: textb; ClB: TExtBClass;');
  Add('  C: textc; ClC: TExtCClass;');
  Add('begin');
  Add('  ClA:=texta;');
  Add('  ClA:=textb;');
  Add('  ClA:=textc;');
  Add('  ClB:=textb;');
  Add('  ClB:=textc;');
  Add('  ClC:=textc;');
  ConvertProgram;
  CheckSource('TestExternalClass_ClassOf',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TExtC", ExtB, "", function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.ProcA = function () {',
    '  };',
    '});',
    'this.A = null;',
    'this.ClA = null;',
    'this.B = null;',
    'this.ClB = null;',
    'this.C = null;',
    'this.ClC = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.ClA = ExtA;',
    '$mod.ClA = ExtB;',
    '$mod.ClA = $mod.TExtC;',
    '$mod.ClB = ExtB;',
    '$mod.ClB = $mod.TExtC;',
    '$mod.ClC = $mod.TExtC;',
    '']));
end;

procedure TTestModule.TestExternalClass_ClassOtherUnit;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    '{$modeswitch externalclass}',
    'type',
    '  TExtA = class external name ''ExtA''',
    '    class var Id: longint;',
    '  end;',
    '']),
    '');

  StartUnit(true);
  Add('interface');
  Add('uses unit2;');
  Add('implementation');
  Add('begin');
  Add('  unit2.texta.id:=unit2.texta.id+1;');
  ConvertUnit;
  CheckSource('TestExternalClass_ClassOtherUnit',
    LinesToStr([
    '']),
    LinesToStr([
    'ExtA.Id = ExtA.Id + 1;',
    '']));
end;

procedure TTestModule.TestExternalClass_Is;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtA''',
  '  end;',
  '  TExtAClass = class of TExtA;',
  '  TExtB = class external name ''ExtB'' (TExtA)',
  '  end;',
  '  TExtBClass = class of TExtB;',
  '  TExtC = class (TExtB)',
  '  end;',
  '  TExtCClass = class of TExtC;',
  'var',
  '  A: texta; ClA: TExtAClass;',
  '  B: textb; ClB: TExtBClass;',
  '  C: textc; ClC: TExtCClass;',
  'begin',
  '  if a is textb then ;',
  '  if a is textc then ;',
  '  if b is textc then ;',
  '  if cla is textb then ;',
  '  if cla is textc then ;',
  '  if clb is textc then ;',
  '  try',
  '  except',
  '  on TExtA do ;',
  '  on e: TExtB do ;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_Is',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TExtC", ExtB, "", function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.A = null;',
    'this.ClA = null;',
    'this.B = null;',
    'this.ClB = null;',
    'this.C = null;',
    'this.ClC = null;',
    '']),
    LinesToStr([ // $mod.$main
    'if (rtl.isExt($mod.A, ExtB)) ;',
    'if ($mod.TExtC.isPrototypeOf($mod.A)) ;',
    'if ($mod.TExtC.isPrototypeOf($mod.B)) ;',
    'if (rtl.isExt($mod.ClA, ExtB)) ;',
    'if (rtl.is($mod.ClA, $mod.TExtC)) ;',
    'if (rtl.is($mod.ClB, $mod.TExtC)) ;',
    'try {} catch ($e) {',
    '  if (rtl.isExt($e,ExtA)) {}',
    '  else if (rtl.isExt($e,ExtB)) {',
    '    var e = $e;',
    '  } else throw $e',
    '};',
    '']));
end;

procedure TTestModule.TestExternalClass_As;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('  end;');
  Add('  TExtB = class external name ''ExtB'' (TExtA)');
  Add('  end;');
  Add('  TExtC = class (TExtB)');
  Add('  end;');
  Add('var');
  Add('  A: texta;');
  Add('  B: textb;');
  Add('  C: textc;');
  Add('begin');
  Add('  b:=a as textb;');
  Add('  c:=a as textc;');
  Add('  c:=b as textc;');
  ConvertProgram;
  CheckSource('TestExternalClass_Is',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TExtC", ExtB, "", function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.A = null;',
    'this.B = null;',
    'this.C = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.B = rtl.asExt($mod.A, ExtB);',
    '$mod.C = rtl.as($mod.A, $mod.TExtC);',
    '$mod.C = rtl.as($mod.B, $mod.TExtC);',
    '']));
end;

procedure TTestModule.TestExternalClass_DestructorFail;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('    destructor Free;');
  Add('  end;');
  SetExpectedPasResolverError('Pascal element not supported: destructor',
    nPasElementNotSupported);
  ConvertProgram;
end;

procedure TTestModule.TestExternalClass_New;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtA''',
  '    constructor New;',
  '    constructor New(i: longint; j: longint = 2);',
  '  end;',
  'var',
  '  A: texta;',
  'begin',
  '  a:=texta.new;',
  '  a:=texta(texta.new);',
  '  a:=texta.new();',
  '  a:=texta.new(1);',
  '  with texta do begin',
  '    a:=new;',
  '    a:=new();',
  '    a:=new(2);',
  '  end;',
  '  a:=test1.texta.new;',
  '  a:=test1.texta.new();',
  '  a:=test1.texta.new(3);',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_New',
    LinesToStr([ // statements
    'this.A = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.A = new ExtA();',
    '$mod.A = new ExtA();',
    '$mod.A = new ExtA();',
    '$mod.A = new ExtA(1,2);',
    '$mod.A = new ExtA();',
    '$mod.A = new ExtA();',
    '$mod.A = new ExtA(2,2);',
    '$mod.A = new ExtA();',
    '$mod.A = new ExtA();',
    '$mod.A = new ExtA(3,2);',
    '']));
end;

procedure TTestModule.TestExternalClass_ClassOf_New;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtAClass = class of TExtA;');
  Add('  TExtA = class external name ''ExtA''');
  Add('    C: TExtAClass;');
  Add('    constructor New;');
  Add('  end;');
  Add('var');
  Add('  A: texta;');
  Add('  C: textaclass;');
  Add('begin');
  Add('  a:=c.new;');
  Add('  a:=c.new();');
  Add('  with C do begin');
  Add('    a:=new;');
  Add('    a:=new();');
  Add('  end;');
  Add('  a:=test1.c.new;');
  Add('  a:=test1.c.new();');
  Add('  a:=A.c.new();');
  ConvertProgram;
  CheckSource('TestExternalClass_ClassOf_New',
    LinesToStr([ // statements
    'this.A = null;',
    'this.C = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.A = new $mod.C();',
    '$mod.A = new $mod.C();',
    'var $with = $mod.C;',
    '$mod.A = new $with();',
    '$mod.A = new $with();',
    '$mod.A = new $mod.C();',
    '$mod.A = new $mod.C();',
    '$mod.A = new $mod.A.C();',
    '']));
end;

procedure TTestModule.TestExternalClass_FuncClassOf_New;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtAClass = class of TExtA;',
  '  TExtA = class external name ''ExtA''',
  '    constructor New;',
  '  end;',
  'function GetCreator: TExtAClass;',
  'begin',
  '  Result:=TExtA;',
  'end;',
  'var',
  '  A: texta;',
  'begin',
  '  a:=getcreator.new;',
  '  a:=getcreator().new;',
  '  a:=getcreator().new();',
  '  a:=getcreator.new();',
  '  with getcreator do begin',
  '    a:=new;',
  '    a:=new();',
  '  end;']);
  ConvertProgram;
  CheckSource('TestExternalClass_FuncClassOf_New',
    LinesToStr([ // statements
    'this.GetCreator = function () {',
    '  var Result = null;',
    '  Result = ExtA;',
    '  return Result;',
    '};',
    'this.A = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.A = new ($mod.GetCreator())();',
    '$mod.A = new ($mod.GetCreator())();',
    '$mod.A = new ($mod.GetCreator())();',
    '$mod.A = new ($mod.GetCreator())();',
    'var $with = $mod.GetCreator();',
    '$mod.A = new $with();',
    '$mod.A = new $with();',
    '']));
end;

procedure TTestModule.TestExternalClass_New_PasClassFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtA''',
  '    constructor New;',
  '  end;',
  '  TBird = class(TExtA)',
  '  end;',
  'begin',
  '  TBird.new;',
  '']);
  SetExpectedPasResolverError(sJSNewNotSupported,nJSNewNotSupported);
  ConvertProgram;
end;

procedure TTestModule.TestExternalClass_New_PasClassBracketsFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtA''',
  '    constructor New;',
  '  end;',
  '  TBird = class(TExtA)',
  '  end;',
  'begin',
  '  TBird.new();',
  '']);
  SetExpectedPasResolverError(sJSNewNotSupported,nJSNewNotSupported);
  ConvertProgram;
end;

procedure TTestModule.TestExternalClass_NewExtName;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtA''',
  '    constructor New; external name ''Other'';',
  '    constructor New(i: longint; j: longint = 2); external name ''A.B'';',
  '  end;',
  'var',
  '  A: texta;',
  'begin',
  '  a:=texta.new;',
  '  a:=texta(texta.new);',
  '  a:=texta.new();',
  '  a:=texta.new(1);',
  '  with texta do begin',
  '    a:=new;',
  '    a:=new();',
  '    a:=new(2);',
  '  end;',
  '  a:=test1.texta.new;',
  '  a:=test1.texta.new();',
  '  a:=test1.texta.new(3);',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_NewExtName',
    LinesToStr([ // statements
    'this.A = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.A = new Other();',
    '$mod.A = new Other();',
    '$mod.A = new Other();',
    '$mod.A = new A.B(1,2);',
    '$mod.A = new Other();',
    '$mod.A = new Other();',
    '$mod.A = new A.B(2,2);',
    '$mod.A = new Other();',
    '$mod.A = new Other();',
    '$mod.A = new A.B(3,2);',
    '']));
end;

procedure TTestModule.TestExternalClass_Constructor;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtA''',
  '    constructor Create;',
  '    constructor Create(i: longint; j: longint = 2);',
  '  end;',
  'var',
  '  A: texta;',
  'begin',
  '  a:=texta.create;',
  '  a:=texta(texta.create);',
  '  a:=texta.create();',
  '  a:=texta.create(1);',
  '  with texta do begin',
  '    a:=create;',
  '    a:=create();',
  '    a:=create(2);',
  '  end;',
  '  a:=test1.texta.create;',
  '  a:=test1.texta.create();',
  '  a:=test1.texta.create(3);',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_Constructor',
    LinesToStr([ // statements
    'this.A = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.A = new ExtA.Create();',
    '$mod.A = new ExtA.Create();',
    '$mod.A = new ExtA.Create();',
    '$mod.A = new ExtA.Create(1,2);',
    '$mod.A = new ExtA.Create();',
    '$mod.A = new ExtA.Create();',
    '$mod.A = new ExtA.Create(2,2);',
    '$mod.A = new ExtA.Create();',
    '$mod.A = new ExtA.Create();',
    '$mod.A = new ExtA.Create(3,2);',
    '']));
end;

procedure TTestModule.TestExternalClass_ConstructorBrackets;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtA''',
  '    constructor Create; external name ''{}'';',
  '  end;',
  'var',
  '  A: texta;',
  'begin',
  '  a:=texta.create;',
  '  a:=texta(texta.create);',
  '  a:=texta.create();',
  '  with texta do begin',
  '    a:=create;',
  '    a:=create();',
  '  end;',
  '  a:=test1.texta.create;',
  '  a:=test1.texta.create();',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_ConstructorBrackets',
    LinesToStr([ // statements
    'this.A = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.A = {};',
    '$mod.A = {};',
    '$mod.A = {};',
    '$mod.A = {};',
    '$mod.A = {};',
    '$mod.A = {};',
    '$mod.A = {};',
    '']));
end;

procedure TTestModule.TestExternalClass_LocalConstSameName;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('    constructor New;');
  Add('  end;');
  Add('function DoIt: longint;');
  Add('const ExtA: longint = 3;');
  Add('begin');
  Add('  Result:=ExtA;');
  Add('end;');
  Add('var');
  Add('  A: texta;');
  Add('begin');
  Add('  a:=texta.new;');
  ConvertProgram;
  CheckSource('TestExternalClass_LocalConstSameName',
    LinesToStr([ // statements
    'var ExtA$1 = 3;',
    'this.DoIt = function () {',
    '  var Result = 0;',
    '  Result = ExtA$1;',
    '  return Result;',
    '};',
    'this.A = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.A = new ExtA();',
    '']));
end;

procedure TTestModule.TestExternalClass_ReintroduceOverload;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('    procedure DoIt;');
  Add('  end;');
  Add('  TMyA = class(TExtA)');
  Add('    procedure DoIt;');
  Add('  end;');
  Add('procedure TMyA.DoIt; begin end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestExternalClass_ReintroduceOverload',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TMyA", ExtA, "", function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt$1 = function () {',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestExternalClass_Inherited;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('    procedure DoIt(i: longint = 1); virtual;');
  Add('    procedure DoSome(j: longint = 2);');
  Add('  end;');
  Add('  TExtB = class external name ''ExtB''(TExtA)');
  Add('  end;');
  Add('  TMyC = class(TExtB)');
  Add('    procedure DoIt(i: longint = 1); override;');
  Add('    procedure DoSome(j: longint = 2); reintroduce;');
  Add('  end;');
  Add('procedure TMyC.DoIt(i: longint);');
  Add('begin');
  Add('  inherited;');
  Add('  inherited DoIt;');
  Add('  inherited DoIt();');
  Add('  inherited DoIt(3);');
  Add('  inherited DoSome;');
  Add('  inherited DoSome();');
  Add('  inherited DoSome(4);');
  Add('end;');
  Add('procedure TMyC.DoSome(j: longint);');
  Add('begin');
  Add('  inherited;');
  Add('end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestExternalClass_ReintroduceOverload',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TMyC", ExtB, "", function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (i) {',
    '    ExtB.DoIt.apply(this, arguments);',
    '    ExtB.DoIt.call(this, 1);',
    '    ExtB.DoIt.call(this, 1);',
    '    ExtB.DoIt.call(this, 3);',
    '    ExtB.DoSome.call(this, 2);',
    '    ExtB.DoSome.call(this, 2);',
    '    ExtB.DoSome.call(this, 4);',
    '  };',
    '  this.DoSome$1 = function (j) {',
    '    ExtB.DoSome.apply(this, arguments);',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestExternalClass_PascalAncestorFail;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TObject = class');
  Add('  end;');
  Add('  TExtA = class external name ''ExtA''(TObject)');
  Add('  end;');
  Add('begin');
  SetExpectedPasResolverError('Ancestor "TObject" is not external',nAncestorIsNotExternal);
  ConvertProgram;
end;

procedure TTestModule.TestExternalClass_NewInstance;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('  end;');
  Add('  TMyB = class(TExtA)');
  Add('  protected');
  Add('    class function NewInstance(fnname: string; const paramarray): TMyB; virtual;');
  Add('  end;');
  Add('class function TMyB.NewInstance(fnname: string; const paramarray): TMyB;');
  Add('begin end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestExternalClass_NewInstance',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TMyB", ExtA, "NewInstance", function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.NewInstance = function (fnname, paramarray) {',
    '    var Result = null;',
    '    return Result;',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestExternalClass_NewInstance_NonVirtualFail;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('  end;');
  Add('  TMyB = class(TExtA)');
  Add('  protected');
  Add('    class function NewInstance(fnname: string; const paramarray): TMyB;');
  Add('  end;');
  Add('class function TMyB.NewInstance(fnname: string; const paramarray): TMyB;');
  Add('begin end;');
  Add('begin');
  SetExpectedPasResolverError(sNewInstanceFunctionMustBeVirtual,nNewInstanceFunctionMustBeVirtual);
  ConvertProgram;
end;

procedure TTestModule.TestExternalClass_NewInstance_FirstParamNotString_Fail;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('  end;');
  Add('  TMyB = class(TExtA)');
  Add('  protected');
  Add('    class function NewInstance(fnname: longint; const paramarray): TMyB; virtual;');
  Add('  end;');
  Add('class function TMyB.NewInstance(fnname: longint; const paramarray): TMyB;');
  Add('begin end;');
  Add('begin');
  SetExpectedPasResolverError('Incompatible type arg no. 1: Got "Longint", expected "String"',
    nIncompatibleTypeArgNo);
  ConvertProgram;
end;

procedure TTestModule.TestExternalClass_NewInstance_SecondParamTyped_Fail;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TExtA = class external name ''ExtA''');
  Add('  end;');
  Add('  TMyB = class(TExtA)');
  Add('  protected');
  Add('    class function NewInstance(fnname: string; const paramarray: string): TMyB; virtual;');
  Add('  end;');
  Add('class function TMyB.NewInstance(fnname: string; const paramarray: string): TMyB;');
  Add('begin end;');
  Add('begin');
  SetExpectedPasResolverError('Incompatible type arg no. 2: Got "type", expected "untyped"',
    nIncompatibleTypeArgNo);
  ConvertProgram;
end;

procedure TTestModule.TestExternalClass_JSFunctionPasDescendant;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSFunction = class external name ''Function''',
  '  end;',
  '  TExtA = class external name ''ExtA''(TJSFunction)',
  '    constructor New(w: word);',
  '  end;',
  '  TBird = class (TExtA)',
  '  public',
  '    Size: word;',
  '    class var Legs: word;',
  '    constructor Create(a: word);',
  '  end;',
  '  TEagle = class (TBird)',
  '  public',
  '    constructor Create(b: word); reintroduce;',
  '  end;',
  'constructor TBird.Create(a: word);',
  'begin',
  '  inherited;',  // silently ignored
  '  inherited New(a);', // this.$func(a)
  'end;',
  'constructor TEagle.Create(b: word);',
  'begin',
  '  inherited Create(b);',
  'end;',
  'var',
  '  Bird: TBird;',
  '  Eagle: TEagle;',
  'begin',
  '  Bird:=TBird.Create(3);',
  '  Eagle:=TEagle.Create(4);',
  '  Bird.Size:=Bird.Size+5;',
  '  Bird.Legs:=Bird.Legs+6;',
  '  Eagle.Size:=Eagle.Size+5;',
  '  Eagle.Legs:=Eagle.Legs+6;',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_JSFunctionPasDescendant',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TBird", ExtA, "", function () {',
    '  this.Legs = 0;',
    '  this.$init = function () {',
    '    this.Size = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function (a) {',
    '    this.$ancestorfunc(a);',
    '    return this;',
    '  };',
    '});',
    'rtl.createClassExt(this, "TEagle", this.TBird, "", function () {',
    '  this.Create$1 = function (b) {',
    '    $mod.TBird.Create.call(this, b);',
    '    return this;',
    '  };',
    '});',
    'this.Bird = null;',
    'this.Eagle = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Bird = $mod.TBird.$create("Create", [3]);',
    '$mod.Eagle = $mod.TEagle.$create("Create$1", [4]);',
    '$mod.Bird.Size = $mod.Bird.Size + 5;',
    '$mod.TBird.Legs = $mod.Bird.Legs + 6;',
    '$mod.Eagle.Size = $mod.Eagle.Size + 5;',
    '$mod.TBird.Legs = $mod.Eagle.Legs + 6;',
    '']));
end;

procedure TTestModule.TestExternalClass_PascalProperty;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TJSElement = class;');
  Add('  TJSNotifyEvent = procedure(Sender: TJSElement) of object;');
  Add('  TJSElement = class external name ''ExtA''');
  Add('  end;');
  Add('  TControl = class(TJSElement)');
  Add('  private');
  Add('    FOnClick: TJSNotifyEvent;');
  Add('    property OnClick: TJSNotifyEvent read FOnClick write FOnClick;');
  Add('    procedure Click(Sender: TJSElement);');
  Add('  end;');
  Add('procedure TControl.Click(Sender: TJSElement);');
  Add('begin');
  Add('  OnClick(Self);');
  Add('end;');
  Add('var');
  Add('  Ctrl: TControl;');
  Add('begin');
  Add('  Ctrl.OnClick:=@Ctrl.Click;');
  Add('  Ctrl.OnClick(Ctrl);');
  ConvertProgram;
  CheckSource('TestExternalClass_PascalProperty',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TControl", ExtA, "", function () {',
    '  this.$init = function () {',
    '    this.FOnClick = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FOnClick = undefined;',
    '  };',
    '  this.Click = function (Sender) {',
    '    this.FOnClick(this);',
    '  };',
    '});',
    'this.Ctrl = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Ctrl.FOnClick = rtl.createCallback($mod.Ctrl, "Click");',
    '$mod.Ctrl.FOnClick($mod.Ctrl);',
    '']));
end;

procedure TTestModule.TestExternalClass_TypeCastToRootClass;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  IUnknown = interface end;',
  '  TObject = class',
  '  end;',
  '  TChild = class',
  '  end;',
  '  TExtRootA = class external name ''ExtRootA''',
  '  end;',
  '  TExtChildA = class external name ''ExtChildA''(TExtRootA)',
  '  end;',
  '  TExtRootB = class external name ''ExtRootB''',
  '  end;',
  '  TExtChildB = class external name ''ExtChildB''(TExtRootB)',
  '  end;',
  'var',
  '  Obj: TObject;',
  '  Child: TChild;',
  '  RootA: TExtRootA;',
  '  ChildA: TExtChildA;',
  '  RootB: TExtRootB;',
  '  ChildB: TExtChildB;',
  '  i: IUnknown;',
  'begin',
  '  obj:=tobject(roota);',
  '  obj:=tobject(childa);',
  '  child:=tchild(tobject(roota));',
  '  roota:=textroota(obj);',
  '  roota:=textroota(child);',
  '  roota:=textroota(rootb);',
  '  roota:=textroota(childb);',
  '  childa:=textchilda(textroota(obj));',
  '  roota:=TExtRootA(i)',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_TypeCastToRootClass',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-B800-000000000000}", [], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TChild", this.TObject, function () {',
    '});',
    'this.Obj = null;',
    'this.Child = null;',
    'this.RootA = null;',
    'this.ChildA = null;',
    'this.RootB = null;',
    'this.ChildB = null;',
    'this.i = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Obj = $mod.RootA;',
    '$mod.Obj = $mod.ChildA;',
    '$mod.Child = $mod.RootA;',
    '$mod.RootA = $mod.Obj;',
    '$mod.RootA = $mod.Child;',
    '$mod.RootA = $mod.RootB;',
    '$mod.RootA = $mod.ChildB;',
    '$mod.ChildA = $mod.Obj;',
    '$mod.RootA = $mod.i;',
    '']));
end;

procedure TTestModule.TestExternalClass_TypeCastToJSObject;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  IUnknown = interface end;',
  '  IBird = interface(IUnknown) end;',
  '  TClass = class of TObject;',
  '  TObject = class',
  '  end;',
  '  TChild = class',
  '  end;',
  '  TJSObject = class external name ''Object''',
  '  end;',
  '  TRec = record end;',
  'var',
  '  Obj: TObject;',
  '  Child: TChild;',
  '  i: IUnknown;',
  '  Bird: IBird;',
  '  j: TJSObject;',
  '  r: TRec;',
  '  c: TClass;',
  'begin',
  '  j:=tjsobject(IUnknown);',
  '  j:=tjsobject(IBird);',
  '  j:=tjsobject(TObject);',
  '  j:=tjsobject(TChild);',
  '  j:=tjsobject(TRec);',
  '  j:=tjsobject(Obj);',
  '  j:=tjsobject(Child);',
  '  j:=tjsobject(i);',
  '  j:=tjsobject(Bird);',
  '  j:=tjsobject(r);',
  '  j:=tjsobject(c);',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_TypeCastToJSObject',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-B800-000000000000}", [], null);',
    'rtl.createInterface(this, "IBird", "{4B0D080B-C0F6-396E-AE88-000B87785074}", [], this.IUnknown);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TChild", this.TObject, function () {',
    '});',
    'rtl.recNewT(this, "TRec", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '});',
    'this.Obj = null;',
    'this.Child = null;',
    'this.i = null;',
    'this.Bird = null;',
    'this.j = null;',
    'this.r = this.TRec.$new();',
    'this.c = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.j = $mod.IUnknown;',
    '$mod.j = $mod.IBird;',
    '$mod.j = $mod.TObject;',
    '$mod.j = $mod.TChild;',
    '$mod.j = $mod.TRec;',
    '$mod.j = $mod.Obj;',
    '$mod.j = $mod.Child;',
    '$mod.j = $mod.i;',
    '$mod.j = $mod.Bird;',
    '$mod.j = $mod.r;',
    '$mod.j = $mod.c;',
    '']));
end;

procedure TTestModule.TestExternalClass_TypeCastStringToExternalString;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TJSString = class external name ''String''');
  Add('    class function fromCharCode() : string; varargs;');
  Add('    function anchor(const aName : string) : string;');
  Add('  end;');
  Add('var');
  Add('  s: string;');
  Add('begin');
  Add('  s:=TJSString.fromCharCode(65,66);');
  Add('  s:=TJSString(s).anchor(s);');
  Add('  s:=TJSString(''foo'').anchor(s);');
  ConvertProgram;
  CheckSource('TestExternalClass_TypeCastStringToExternalString',
    LinesToStr([ // statements
    'this.s = "";',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.s = String.fromCharCode(65, 66);',
    '$mod.s = $mod.s.anchor($mod.s);',
    '$mod.s = "foo".anchor($mod.s);',
    '']));
end;

procedure TTestModule.TestExternalClass_TypeCastToJSFunction;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSObject = class external name ''Object'' end;',
  '  TJSFunction = class external name ''Function''',
  '    function bind(thisArg: TJSObject): TJSFunction; varargs;',
  '    function call(thisArg: TJSObject): JSValue; varargs;',
  '  end;',
  '  TObject = class',
  '    procedure DoIt(i: longint);',
  '  end;',
  '  TFuncInt = function(o: TObject): longint;',
  'function GetIt(o: TObject): longint;',
  '  procedure Sub; begin end;',
  'var',
  '  f: TJSFunction;',
  '  fi: TFuncInt;',
  'begin',
  '  fi:=TFuncInt(f);',
  '  f:=TJSFunction(fi);',
  '  f:=TJSFunction(@GetIt);',
  '  f:=TJSFunction(@GetIt).bind(nil,3);',
  '  f:=TJSFunction(@Sub);',
  '  f:=TJSFunction(@o.doit);',
  '  f:=TJSFunction(fi).bind(nil,4)',
  'end;',
  'procedure TObject.DoIt(i: longint);',
  '  procedure Sub; begin end;',
  'var f: TJSFunction;',
  'begin',
  '  f:=TJSFunction(@DoIt);',
  '  f:=TJSFunction(@DoIt).bind(nil,13);',
  '  f:=TJSFunction(@Sub);',
  '  f:=TJSFunction(@GetIt);',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestExternalClass_TypeCastToJSFunction',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (i) {',
    '    var $Self = this;',
    '    function Sub() {',
    '    };',
    '    var f = null;',
    '    f = this.DoIt;',
    '    f = this.DoIt.bind(null, 13);',
    '    f = Sub;',
    '    f = $mod.GetIt;',
    '  };',
    '});',
    'this.GetIt = function (o) {',
    '  var Result = 0;',
    '  function Sub() {',
    '  };',
    '  var f = null;',
    '  var fi = null;',
    '  fi = f;',
    '  f = fi;',
    '  f = $mod.GetIt;',
    '  f = $mod.GetIt.bind(null, 3);',
    '  f = Sub;',
    '  f = $mod.TObject.DoIt;',
    '  f = fi.bind(null, 4);',
    '  return Result;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestExternalClass_TypeCastDelphiUnrelated;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  '{$modeswitch externalclass}',
  'type',
  '  TJSObject = class external name ''Object'' end;',
  '  TJSWindow = class external name ''Window''(TJSObject)',
  '    procedure Open;',
  '  end;',
  '  TJSEventTarget = class external name ''Event''(TJSObject)',
  '    procedure Execute;',
  '  end;',
  'procedure Fly;',
  'var',
  '  w: TJSWindow;',
  '  e: TJSEventTarget;',
  'begin',
  '  w:=TJSWindow(e);',
  '  e:=TJSEventTarget(w);',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestExternalClass_TypeCastDelphiUnrelated',
    LinesToStr([ // statements
    'this.Fly = function () {',
    '  var w = null;',
    '  var e = null;',
    '  w = e;',
    '  e = w;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestExternalClass_CallClassFunctionOfInstanceFail;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TJSString = class external name ''String''');
  Add('    class function fromCharCode() : string; varargs;');
  Add('  end;');
  Add('var');
  Add('  s: string;');
  Add('  sObj: TJSString;');
  Add('begin');
  Add('  s:=sObj.fromCharCode(65,66);');
  SetExpectedPasResolverError('External class instance cannot access static class function fromCharCode',
    nExternalClassInstanceCannotAccessStaticX);
  ConvertProgram;
end;

procedure TTestModule.TestExternalClass_BracketAccessor;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSArray = class external name ''Array2''',
  '    function GetItems(Index: longint): jsvalue; external name ''[]'';',
  '    procedure SetItems(Index: longint; Value: jsvalue); external name ''[]'';',
  '    property Items[Index: longint]: jsvalue read GetItems write SetItems; default;',
  '  end;',
  'procedure DoIt(vI: JSValue; const vJ: jsvalue; var vK: jsvalue; out vL: jsvalue);',
  'begin end;',
  'var',
  '  Arr: tjsarray;',
  '  s: string;',
  '  i: longint;',
  '  v: jsvalue;',
  'begin',
  '  v:=arr[0];',
  '  v:=arr.items[1];',
  '  arr[2]:=s;',
  '  arr.items[3]:=s;',
  '  arr[4]:=i;',
  '  arr[5]:=arr[6];',
  '  arr.items[7]:=arr.items[8];',
  '  with arr do items[9]:=items[10];',
  '  doit(arr[7],arr[8],arr[9],arr[10]);',
  '  with arr do begin',
  '    v:=GetItems(14);',
  '    setitems(15,16);',
  '  end;',
  '  v:=test1.arr.items[17];',
  '  test1.arr.items[18]:=v;',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_BracketAccessor',
    LinesToStr([ // statements
    'this.DoIt = function (vI, vJ, vK, vL) {',
    '};',
    'this.Arr = null;',
    'this.s = "";',
    'this.i = 0;',
    'this.v = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.v = $mod.Arr[0];',
    '$mod.v = $mod.Arr[1];',
    '$mod.Arr[2] = $mod.s;',
    '$mod.Arr[3] = $mod.s;',
    '$mod.Arr[4] = $mod.i;',
    '$mod.Arr[5] = $mod.Arr[6];',
    '$mod.Arr[7] = $mod.Arr[8];',
    'var $with = $mod.Arr;',
    '$with[9] = $with[10];',
    '$mod.DoIt($mod.Arr[7], $mod.Arr[8], {',
    '  a: 9,',
    '  p: $mod.Arr,',
    '  get: function () {',
    '      return this.p[this.a];',
    '    },',
    '  set: function (v) {',
    '      this.p[this.a] = v;',
    '    }',
    '}, {',
    '  a: 10,',
    '  p: $mod.Arr,',
    '  get: function () {',
    '      return this.p[this.a];',
    '    },',
    '  set: function (v) {',
    '      this.p[this.a] = v;',
    '    }',
    '});',
    'var $with1 = $mod.Arr;',
    '$mod.v = $with1[14];',
    '$with1[15] = 16;',
    '$mod.v = $mod.Arr[17];',
    '$mod.Arr[18] = $mod.v;',
    '']));
end;

procedure TTestModule.TestExternalClass_BracketAccessor_Call;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSArray = class external name ''Array2''',
  '    function GetItems(Index: longint): jsvalue; external name ''[]'';',
  '    procedure SetItems(Index: longint; Value: jsvalue); external name ''[]'';',
  '    property Items[Index: longint]: jsvalue read GetItems write SetItems; default;',
  '  end;',
  '  TMyArr = class(TJSArray)',
  '    procedure DoIt;',
  '  end;',
  'procedure tmyarr.DoIt;',
  'begin',
  '  Items[1]:=Items[2];',
  '  SetItems(3,getItems(4));',
  'end;',
  'var',
  '  Arr: tmyarr;',
  '  s: string;',
  '  i: longint;',
  '  v: jsvalue;',
  'begin',
  '  v:=arr[0];',
  '  v:=arr.items[1];',
  '  arr[2]:=s;',
  '  arr.items[3]:=s;',
  '  arr[4]:=i;',
  '  arr[5]:=arr[6];',
  '  arr.items[7]:=arr.items[8];',
  '  with arr do items[9]:=items[10];',
  '  with arr do begin',
  '    v:=GetItems(14);',
  '    setitems(15,16);',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_BracketAccessor_Call',
    LinesToStr([ // statements
    'rtl.createClassExt(this, "TMyArr", Array2, "", function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '    this[1] = this[2];',
    '    this[3] = this[4];',
    '  };',
    '});',
    'this.Arr = null;',
    'this.s = "";',
    'this.i = 0;',
    'this.v = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.v = $mod.Arr[0];',
    '$mod.v = $mod.Arr[1];',
    '$mod.Arr[2] = $mod.s;',
    '$mod.Arr[3] = $mod.s;',
    '$mod.Arr[4] = $mod.i;',
    '$mod.Arr[5] = $mod.Arr[6];',
    '$mod.Arr[7] = $mod.Arr[8];',
    'var $with = $mod.Arr;',
    '$with[9] = $with[10];',
    'var $with1 = $mod.Arr;',
    '$mod.v = $with1[14];',
    '$with1[15] = 16;',
    '']));
end;

procedure TTestModule.TestExternalClass_BracketAccessor_2ParamsFail;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TJSArray = class external name ''Array2''');
  Add('    function GetItems(Index1, Index2: longint): jsvalue; external name ''[]'';');
  Add('    procedure SetItems(Index1, Index2: longint; Value: jsvalue); external name ''[]'';');
  Add('    property Items[Index1, Index2: longint]: jsvalue read GetItems write SetItems; default;');
  Add('  end;');
  Add('begin');
  SetExpectedPasResolverError(sBracketAccessorOfExternalClassMustHaveOneParameter,
    nBracketAccessorOfExternalClassMustHaveOneParameter);
  ConvertProgram;
end;

procedure TTestModule.TestExternalClass_BracketAccessor_ReadOnly;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TJSArray = class external name ''Array2''');
  Add('    function GetItems(Index: longint): jsvalue; external name ''[]'';');
  Add('    property Items[Index: longint]: jsvalue read GetItems; default;');
  Add('  end;');
  Add('procedure DoIt(vI: JSValue; const vJ: jsvalue);');
  Add('begin end;');
  Add('var');
  Add('  Arr: tjsarray;');
  Add('  v: jsvalue;');
  Add('begin');
  Add('  v:=arr[0];');
  Add('  v:=arr.items[1];');
  Add('  with arr do v:=items[2];');
  Add('  doit(arr[3],arr[4]);');
  ConvertProgram;
  CheckSource('TestExternalClass_BracketAccessor_ReadOnly',
    LinesToStr([ // statements
    'this.DoIt = function (vI, vJ) {',
    '};',
    'this.Arr = null;',
    'this.v = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.v = $mod.Arr[0];',
    '$mod.v = $mod.Arr[1];',
    'var $with = $mod.Arr;',
    '$mod.v = $with[2];',
    '$mod.DoIt($mod.Arr[3], $mod.Arr[4]);',
    '']));
end;

procedure TTestModule.TestExternalClass_BracketAccessor_WriteOnly;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TJSArray = class external name ''Array2''');
  Add('    procedure SetItems(Index: longint; Value: jsvalue); external name ''[]'';');
  Add('    property Items[Index: longint]: jsvalue write SetItems; default;');
  Add('  end;');
  Add('var');
  Add('  Arr: tjsarray;');
  Add('  s: string;');
  Add('  i: longint;');
  Add('  v: jsvalue;');
  Add('begin');
  Add('  arr[2]:=s;');
  Add('  arr.items[3]:=s;');
  Add('  arr[4]:=i;');
  Add('  with arr do items[5]:=i;');
  ConvertProgram;
  CheckSource('TestExternalClass_BracketAccessor_WriteOnly',
    LinesToStr([ // statements
    'this.Arr = null;',
    'this.s = "";',
    'this.i = 0;',
    'this.v = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Arr[2] = $mod.s;',
    '$mod.Arr[3] = $mod.s;',
    '$mod.Arr[4] = $mod.i;',
    'var $with = $mod.Arr;',
    '$with[5] = $mod.i;',
    '']));
end;

procedure TTestModule.TestExternalClass_BracketAccessor_MultiType;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TJSArray = class external name ''Array2''');
  Add('    procedure SetItems(Index: longint; Value: jsvalue); external name ''[]'';');
  Add('    property Items[Index: longint]: jsvalue write SetItems; default;');
  Add('    procedure SetNumbers(Index: longint; Value: longint); external name ''[]'';');
  Add('    property Numbers[Index: longint]: longint write SetNumbers;');
  Add('  end;');
  Add('var');
  Add('  Arr: tjsarray;');
  Add('  s: string;');
  Add('  i: longint;');
  Add('  v: jsvalue;');
  Add('begin');
  Add('  arr[2]:=s;');
  Add('  arr.items[3]:=s;');
  Add('  arr.numbers[4]:=i;');
  Add('  with arr do items[5]:=i;');
  Add('  with arr do numbers[6]:=i;');
  ConvertProgram;
  CheckSource('TestExternalClass_BracketAccessor_MultiType',
    LinesToStr([ // statements
    'this.Arr = null;',
    'this.s = "";',
    'this.i = 0;',
    'this.v = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Arr[2] = $mod.s;',
    '$mod.Arr[3] = $mod.s;',
    '$mod.Arr[4] = $mod.i;',
    'var $with = $mod.Arr;',
    '$with[5] = $mod.i;',
    'var $with1 = $mod.Arr;',
    '$with1[6] = $mod.i;',
    '']));
end;

procedure TTestModule.TestExternalClass_BracketAccessor_Index;
begin
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TJSArray = class external name ''Array2''');
  Add('    function GetItems(Index: longint): jsvalue; external name ''[]'';');
  Add('    procedure SetItems(Index: longint; Value: jsvalue); external name ''[]'';');
  Add('    property Items[Index: longint]: jsvalue read GetItems write SetItems; default;');
  Add('  end;');
  Add('var');
  Add('  Arr: tjsarray;');
  Add('  i: longint;');
  Add('  IntArr: array of longint;');
  Add('  v: jsvalue;');
  Add('begin');
  Add('  v:=arr.items[i];');
  Add('  arr[longint(v)]:=arr.items[intarr[0]];');
  Add('  arr.items[intarr[1]]:=arr[IntArr[2]];');
  ConvertProgram;
  CheckSource('TestExternalClass_BracketAccessor_Index',
    LinesToStr([ // statements
    'this.Arr = null;',
    'this.i = 0;',
    'this.IntArr = [];',
    'this.v = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.v = $mod.Arr[$mod.i];',
    '$mod.Arr[rtl.trunc($mod.v)] = $mod.Arr[$mod.IntArr[0]];',
    '$mod.Arr[$mod.IntArr[1]] = $mod.Arr[$mod.IntArr[2]];',
    '']));
end;

procedure TTestModule.TestExternalClass_ForInJSObject;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSObject = class external name ''Object''',
  '  end;',
  'var',
  '  o: TJSObject;',
  '  key: string;',
  'begin',
  '  for key in o do',
  '    if key=''abc'' then ;',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_ForInJSObject',
    LinesToStr([ // statements
    'this.o = null;',
    'this.key = "";',
    '']),
    LinesToStr([ // $mod.$main
    'for ($mod.key in $mod.o) if ($mod.key === "abc") ;',
    '']));
end;

procedure TTestModule.TestExternalClass_ForInJSArray;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSInt8Array = class external name ''Int8Array''',
  '  private',
  '    flength: NativeInt external name ''length'';',
  '    function getValue(Index: NativeInt): shortint; external name ''[]'';',
  '  public',
  '    property values[Index: NativeInt]: Shortint Read getValue; default;',
  '    property Length: NativeInt read flength;',
  '  end;',
  'var',
  '  a: TJSInt8Array;',
  '  value: shortint;',
  'begin',
  '  for value in a do',
  '    if value=3 then ;',
  '']);
  ConvertProgram;
  CheckSource('TestExternalClass_ForInJSArray',
    LinesToStr([ // statements
    'this.a = null;',
    'this.value = 0;',
    '']),
    LinesToStr([ // $mod.$main
    'for (var $in = $mod.a, $l = 0, $end = rtl.length($in) - 1; $l <= $end; $l++) {',
    '  $mod.value = $in[$l];',
    '  if ($mod.value === 3) ;',
    '};',
    '']));
end;

procedure TTestModule.TestExternalClass_IncompatibleArgDuplicateIdentifier;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    '{$modeswitch externalclass}',
    'type',
    '  TJSBufferSource = class external name ''BufferSource''',
    '  end;',
    'procedure DoIt(s: TJSBufferSource); external name ''DoIt'';',
    '']),
    '');
  AddModuleWithIntfImplSrc('unit3.pas',
    LinesToStr([
    '{$modeswitch externalclass}',
    'type',
    '  TJSBufferSource = class external name ''BufferSource''',
    '  end;',
    '']),
    '');

  StartUnit(true);
  Add([
  'interface',
  'uses unit2, unit3;',
  'procedure DoSome(s: TJSBufferSource);',
  'implementation',
  'procedure DoSome(s: TJSBufferSource);',
  'begin',
  '  DoIt(s);',
  'end;',
  '']);
  SetExpectedPasResolverError('Incompatible type arg no. 1: Got "unit3.TJSBufferSource", expected "unit2.TJSBufferSource"',
    nIncompatibleTypeArgNo);
  ConvertUnit;
end;

procedure TTestModule.TestClassInterface_Corba;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface;',
  '  IUnknown = interface',
  '    [''{00000000-0000-0000-C000-000000000046}'']',
  '  end;',
  '  IInterface = IUnknown;',
  '  IBird = interface(IInterface)',
  '    function GetSize: longint;',
  '    procedure SetSize(i: longint);',
  '    property Size: longint read GetSize write SetSize;',
  '    procedure DoIt(i: longint);',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject,IBird)',
  '    function GetSize: longint; virtual; abstract;',
  '    procedure SetSize(i: longint); virtual; abstract;',
  '    procedure DoIt(i: longint); virtual; abstract;',
  '  end;',
  'var',
  '  BirdIntf: IBird;',
  'begin',
  '  BirdIntf.Size:=BirdIntf.Size;',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_Corba',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{00000000-0000-0000-C000-000000000046}", [], null);',
    'rtl.createInterface(this, "IBird", "{5BD1A53B-69BB-37EE-AF32-BEFB86D85B03}", ["GetSize", "SetSize", "DoIt"], this.IUnknown);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  rtl.addIntf(this, $mod.IBird);',
    '});',
    'this.BirdIntf = null;',
    '']),
    LinesToStr([ // $mod.$main
    '  $mod.BirdIntf.SetSize($mod.BirdIntf.GetSize());',
    '']));
end;

procedure TTestModule.TestClassInterface_ProcExternalFail;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '    procedure DoIt; external name ''foo'';',
  '  end;',
  'begin']);
  SetExpectedParserError(
    'Fields are not allowed in interface at token "Identifier external" in file test1.pp at line 6 column 21',
    nParserNoFieldsAllowed);
  ConvertProgram;
end;

procedure TTestModule.TestClassInterface_Overloads;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  integer = longint;',
  '  IUnknown = interface',
  '    procedure DoIt(i: integer);',
  '    procedure DoIt(s: string);',
  '  end;',
  '  IBird = interface(IUnknown)',
  '    procedure DoIt(b: boolean); overload;',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject,IBird)',
  '    procedure DoIt(o: TObject);',
  '    procedure DoIt(s: string);',
  '    procedure DoIt(i: integer);',
  '    procedure DoIt(b: boolean);',
  '  end;',
  'procedure TBird.DoIt(o: TObject); begin end;',
  'procedure TBird.DoIt(s: string); begin end;',
  'procedure TBird.DoIt(i: integer); begin end;',
  'procedure TBird.DoIt(b: boolean); begin end;',
  'var',
  '  BirdIntf: IBird;',
  'begin',
  '  BirdIntf.DoIt(3);',
  '  BirdIntf.DoIt(''abc'');',
  '  BirdIntf.DoIt(true);',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_Overloads',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-BDC4-8A2AE2C59400}", ["DoIt", "DoIt$1"], null);',
    'rtl.createInterface(this, "IBird", "{8285DD5E-EA3E-396E-AE88-000B86AABF05}", ["DoIt$2"], this.IUnknown);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt = function (o) {',
    '  };',
    '  this.DoIt$1 = function (s) {',
    '  };',
    '  this.DoIt$2 = function (i) {',
    '  };',
    '  this.DoIt$3 = function (b) {',
    '  };',
    '  rtl.addIntf(this, $mod.IBird, {',
    '    DoIt$2: "DoIt$3",',
    '    DoIt: "DoIt$2"',
    '  });',
    '});',
    'this.BirdIntf = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.BirdIntf.DoIt(3);',
    '$mod.BirdIntf.DoIt$1("abc");',
    '$mod.BirdIntf.DoIt$2(true);',
    '']));
end;

procedure TTestModule.TestClassInterface_DuplicateGUIInIntfListFail;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IBird = interface',
  '    [''{4B3BA825-E0EC-4799-A19C-55F714A07959}'']',
  '  end;',
  '  IDog = interface',
  '    [''{4B3BA825-E0EC-4799-A19C-55F714A07959}'']',
  '  end;',
  '  TObject = class(IBird,IDog)',
  '  end;',
  'begin']);
  SetExpectedPasResolverError('Duplicate GUID {4B3BA825-E0EC-4799-A19C-55F714A07959} in IDog and IBird',
    nDuplicateGUIDXInYZ);
  ConvertProgram;
end;

procedure TTestModule.TestClassInterface_DuplicateGUIInAncestorFail;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IAnimal = interface',
  '    [''{4B3BA825-E0EC-4799-A19C-55F714A07959}'']',
  '  end;',
  '  IBird = interface(IAnimal)',
  '  end;',
  '  IHawk = interface(IBird)',
  '    [''{4B3BA825-E0EC-4799-A19C-55F714A07959}'']',
  '  end;',
  'begin']);
  SetExpectedPasResolverError('Duplicate GUID {4B3BA825-E0EC-4799-A19C-55F714A07959} in IHawk and IAnimal',
    nDuplicateGUIDXInYZ);
  ConvertProgram;
end;

procedure TTestModule.TestClassInterface_AncestorImpl;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  integer = longint;',
  '  IUnknown = interface',
  '    procedure DoIt(i: integer);',
  '  end;',
  '  IBird = interface',
  '    procedure Fly(i: integer);',
  '  end;',
  '  TObject = class(IUnknown)',
  '    procedure DoIt(i: integer);',
  '  end;',
  '  TBird = class(IBird)',
  '    procedure Fly(i: integer);',
  '  end;',
  'procedure TObject.DoIt(i: integer); begin end;',
  'procedure TBird.Fly(i: integer); begin end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_AncestorIntf',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-BDC4-8A2800000000}", ["DoIt"], null);',
    'rtl.createInterface(this, "IBird", "{B92D5841-6264-3AE3-BF20-000000000000}", ["Fly"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (i) {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.Fly = function (i) {',
    '  };',
    '  rtl.addIntf(this, $mod.IBird);',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_ImplReintroduce;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  integer = longint;',
  '  IBird = interface',
  '    procedure DoIt(i: integer);',
  '  end;',
  '  TObject = class',
  '    procedure DoIt(i: integer);',
  '  end;',
  '  TBird = class(IBird)',
  '    procedure DoIt(i: integer); virtual; reintroduce;',
  '  end;',
  'procedure TObject.DoIt(i: integer); begin end;',
  'procedure TBird.DoIt(i: integer); begin end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_ImplReintroduce',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IBird", "{B92D5841-6264-3AE2-8594-000000000000}", ["DoIt"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (i) {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt$1 = function (i) {',
    '  };',
    '  rtl.addIntf(this, $mod.IBird, {',
    '    DoIt: "DoIt$1"',
    '  });',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_MethodResolution;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '    procedure Walk(i: longint);',
  '  end;',
  '  IBird = interface(IUnknown)',
  '    procedure Walk(b: boolean); overload;',
  '    procedure Fly(s: string);',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject,IBird)',
  '    procedure IBird.Fly = Move;',
  '    procedure IBird.Walk = Hop;',
  '    procedure Hop(i: longint);',
  '    procedure Move(s: string);',
  '    procedure Hop(b: boolean);',
  '  end;',
  'procedure TBird.Move(s: string); begin end;',
  'procedure TBird.Hop(i: longint); begin end;',
  'procedure TBird.Hop(b: boolean); begin end;',
  'var',
  '  BirdIntf: IBird;',
  'begin',
  '  BirdIntf.Walk(3);',
  '  BirdIntf.Walk(true);',
  '  BirdIntf.Fly(''abc'');',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_MethodResolution',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-BDD7-23D600000000}", ["Walk"], null);',
    'rtl.createInterface(this, "IBird", "{CF8A4986-80F6-396E-AE88-000B86AAE208}", ["Walk$1", "Fly"], this.IUnknown);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.Hop = function (i) {',
    '  };',
    '  this.Move = function (s) {',
    '  };',
    '  this.Hop$1 = function (b) {',
    '  };',
    '  rtl.addIntf(this, $mod.IBird, {',
    '    Walk$1: "Hop$1",',
    '    Fly: "Move",',
    '    Walk: "Hop"',
    '  });',
    '});',
    'this.BirdIntf = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.BirdIntf.Walk(3);',
    '$mod.BirdIntf.Walk$1(true);',
    '$mod.BirdIntf.Fly("abc");',
    '']));
end;

procedure TTestModule.TestClassInterface_AncestorMoreInterfaces;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    procedure Walk;',
  '  end;',
  '  IBird = interface end;',
  '  IDog = interface end;',
  '  TObject = class(IBird,IDog)',
  '    function _AddRef: longint; virtual; abstract;',
  '    procedure Walk; virtual; abstract;',
  '  end;',
  '  TBird = class(IUnknown)',
  '  end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_AncestorLess',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{8F2D5841-758A-322B-BDDF-21CD521DD723}", ["_AddRef", "Walk"], null);',
    'rtl.createInterface(this, "IBird", "{CCE11D4C-6504-3AEE-AE88-000B86AAE675}", [], this.IUnknown);',
    'rtl.createInterface(this, "IDog", "{CCE11D4C-6504-3AEE-AE88-000B8E5FC675}", [], this.IUnknown);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IBird);',
    '  rtl.addIntf(this, $mod.IDog);',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '  rtl.addIntf(this, $mod.IBird);',
    '  rtl.addIntf(this, $mod.IDog);',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_MethodOverride;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '    [''{D6D98E5B-8A10-4FEC-856A-7BFC847FE74B}'']',
  '    procedure Go;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    procedure Go; virtual; abstract;',
  '  end;',
  '  TBird = class',
  '    procedure Go; override;',
  '  end;',
  '  TCat = class(TObject)',
  '    procedure Go; override;',
  '  end;',
  '  TDog = class(TObject, IUnknown)',
  '    procedure Go; override;',
  '  end;',
  'procedure TBird.Go; begin end;',
  'procedure TCat.Go; begin end;',
  'procedure TDog.Go; begin end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_MethodOverride',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D6D98E5B-8A10-4FEC-856A-7BFC847FE74B}", ["Go"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.Go = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'rtl.createClass(this, "TCat", this.TObject, function () {',
    '  this.Go = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'rtl.createClass(this, "TDog", this.TObject, function () {',
    '  this.Go = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_Corba_Delegation;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IBird = interface(IUnknown)',
  '    procedure Fly(s: string);',
  '  end;',
  '  IEagle = interface(IBird)',
  '  end;',
  '  IDove = interface(IBird)',
  '  end;',
  '  ISwallow = interface(IBird)',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject,IBird,IEagle,IDove,ISwallow)',
  '    procedure Fly(s: string); virtual; abstract;',
  '  end;',
  '  TBat = class(IBird,IEagle,IDove,ISwallow)',
  '    FBirdIntf: IBird;',
  '    property BirdIntf: IBird read FBirdIntf implements IBird;',
  '    function GetEagleIntf: IEagle; virtual; abstract;',
  '    property EagleIntf: IEagle read GetEagleIntf implements IEagle;',
  '    FDoveObj: TBird;',
  '    property DoveObj: TBird read FDoveObj implements IDove;',
  '    function GetSwallowObj: TBird; virtual; abstract;',
  '    property SwallowObj: TBird read GetSwallowObj implements ISwallow;',
  '  end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_Corba_Delegation',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-B800-000000000000}", [], null);',
    'rtl.createInterface(this, "IBird", "{478D080B-C0F6-396E-AE88-000B87785B07}", ["Fly"], this.IUnknown);',
    'rtl.createInterface(this, "IEagle", "{489289DE-FDE2-34A6-8288-39119022B1B4}", [], this.IBird);',
    'rtl.createInterface(this, "IDove", "{489289DE-FDE2-34A6-8288-39118EF16074}", [], this.IBird);',
    'rtl.createInterface(this, "ISwallow", "{B89289DE-FDE2-34A6-8288-3911CBDCB359}", [], this.IBird);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  rtl.addIntf(this, $mod.IBird);',
    '  rtl.addIntf(this, $mod.IEagle);',
    '  rtl.addIntf(this, $mod.IDove);',
    '  rtl.addIntf(this, $mod.ISwallow);',
    '});',
    'rtl.createClass(this, "TBat", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.FBirdIntf = null;',
    '    this.FDoveObj = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FBirdIntf = undefined;',
    '    this.FDoveObj = undefined;',
    '    $mod.TObject.$final.call(this);',
    '  };',
    '  this.$intfmaps = {',
    '    "{478D080B-C0F6-396E-AE88-000B87785B07}": function () {',
    '        return this.FBirdIntf;',
    '      },',
    '    "{489289DE-FDE2-34A6-8288-39119022B1B4}": function () {',
    '        return this.GetEagleIntf();',
    '      },',
    '    "{489289DE-FDE2-34A6-8288-39118EF16074}": function () {',
    '        return rtl.getIntfT(this.FDoveObj, $mod.IDove);',
    '      },',
    '    "{B89289DE-FDE2-34A6-8288-3911CBDCB359}": function () {',
    '        return rtl.getIntfT(this.GetSwallowObj(), $mod.ISwallow);',
    '      }',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_Corba_DelegationStatic;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IBird = interface(IUnknown)',
  '    procedure Fly(s: string);',
  '  end;',
  '  IEagle = interface(IBird)',
  '  end;',
  '  IDove = interface(IBird)',
  '  end;',
  '  ISwallow = interface(IBird)',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject,IBird,IEagle,IDove,ISwallow)',
  '    procedure Fly(s: string); virtual; abstract;',
  '  end;',
  '  TBat = class(IBird,IEagle,IDove,ISwallow)',
  '  private',
  '    class var FBirdIntf: IBird;',
  '    class var FDoveObj: TBird;',
  '    class function GetEagleIntf: IEagle; virtual; abstract;',
  '    class function GetSwallowObj: TBird; virtual; abstract;',
  '  protected',
  '    class property BirdIntf: IBird read FBirdIntf implements IBird;',
  '    class property EagleIntf: IEagle read GetEagleIntf implements IEagle;',
  '    class property DoveObj: TBird read FDoveObj implements IDove;',
  '    class property SwallowObj: TBird read GetSwallowObj implements ISwallow;',
  '  end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_Corba_DelegationStatic',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-B800-000000000000}", [], null);',
    'rtl.createInterface(this, "IBird", "{478D080B-C0F6-396E-AE88-000B87785B07}", ["Fly"], this.IUnknown);',
    'rtl.createInterface(this, "IEagle", "{489289DE-FDE2-34A6-8288-39119022B1B4}", [], this.IBird);',
    'rtl.createInterface(this, "IDove", "{489289DE-FDE2-34A6-8288-39118EF16074}", [], this.IBird);',
    'rtl.createInterface(this, "ISwallow", "{B89289DE-FDE2-34A6-8288-3911CBDCB359}", [], this.IBird);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  rtl.addIntf(this, $mod.IBird);',
    '  rtl.addIntf(this, $mod.IEagle);',
    '  rtl.addIntf(this, $mod.IDove);',
    '  rtl.addIntf(this, $mod.ISwallow);',
    '});',
    'rtl.createClass(this, "TBat", this.TObject, function () {',
    '  this.FBirdIntf = null;',
    '  this.FDoveObj = null;',
    '  this.$intfmaps = {',
    '    "{478D080B-C0F6-396E-AE88-000B87785B07}": function () {',
    '        return this.FBirdIntf;',
    '      },',
    '    "{489289DE-FDE2-34A6-8288-39119022B1B4}": function () {',
    '        return this.GetEagleIntf();',
    '      },',
    '    "{489289DE-FDE2-34A6-8288-39118EF16074}": function () {',
    '        return rtl.getIntfT(this.FDoveObj, $mod.IDove);',
    '      },',
    '    "{B89289DE-FDE2-34A6-8288-3911CBDCB359}": function () {',
    '        return rtl.getIntfT(this.GetSwallowObj(), $mod.ISwallow);',
    '      }',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_Corba_Operators;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IBird = interface(IUnknown)',
  '    function GetItems(Index: longint): longint;',
  '    procedure SetItems(Index: longint; Value: longint);',
  '    property Items[Index: longint]: longint read GetItems write SetItems; default;',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject,IBird)',
  '    function GetItems(Index: longint): longint; virtual; abstract;',
  '    procedure SetItems(Index: longint; Value: longint); virtual; abstract;',
  '  end;',
  'var',
  '  IntfVar: IBird = nil;',
  '  IntfVar2: IBird;',
  '  ObjVar: TBird;',
  '  v: JSValue;',
  'begin',
  '  IntfVar:=nil;',
  '  IntfVar[3]:=IntfVar[4];',
  '  if Assigned(IntfVar) then ;',
  '  IntfVar:=IntfVar2;',
  '  IntfVar:=ObjVar;',
  '  if IntfVar=IntfVar2 then ;',
  '  if IntfVar<>IntfVar2 then ;',
  '  if IntfVar is IBird then ;',
  '  if IntfVar is TBird then ;',
  '  if ObjVar is IBird then ;',
  '  IntfVar:=IntfVar2 as IBird;',
  '  ObjVar:=IntfVar2 as TBird;',
  '  IntfVar:=ObjVar as IBird;',
  '  IntfVar:=IBird(IntfVar2);',
  '  ObjVar:=TBird(IntfVar);',
  '  IntfVar:=IBird(ObjVar);',
  '  v:=IntfVar;',
  '  IntfVar:=IBird(v);',
  '  if v is IBird then ;',
  '  v:=JSValue(IntfVar);',
  '  v:=IBird;',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_Corba_Operators',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-B800-000000000000}", [], null);',
    'rtl.createInterface(this, "IBird", "{D53FED90-DE59-3202-B1AE-000B87785B08}", ["GetItems", "SetItems"], this.IUnknown);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  rtl.addIntf(this, $mod.IBird);',
    '});',
    'this.IntfVar = null;',
    'this.IntfVar2 = null;',
    'this.ObjVar = null;',
    'this.v = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.IntfVar = null;',
    '$mod.IntfVar.SetItems(3, $mod.IntfVar.GetItems(4));',
    'if ($mod.IntfVar != null) ;',
    '$mod.IntfVar = $mod.IntfVar2;',
    '$mod.IntfVar = rtl.getIntfT($mod.ObjVar,$mod.IBird);',
    'if ($mod.IntfVar === $mod.IntfVar2) ;',
    'if ($mod.IntfVar !== $mod.IntfVar2) ;',
    'if ($mod.IBird.isPrototypeOf($mod.IntfVar)) ;',
    'if (rtl.intfIsClass($mod.IntfVar, $mod.TBird)) ;',
    'if (rtl.getIntfT($mod.ObjVar, $mod.IBird) !== null) ;',
    '$mod.IntfVar = rtl.as($mod.IntfVar2, $mod.IBird);',
    '$mod.ObjVar = rtl.intfAsClass($mod.IntfVar2, $mod.TBird);',
    '$mod.IntfVar = rtl.getIntfT($mod.ObjVar, $mod.IBird);',
    '$mod.IntfVar = $mod.IntfVar2;',
    '$mod.ObjVar = rtl.intfToClass($mod.IntfVar, $mod.TBird);',
    '$mod.IntfVar = rtl.getIntfT($mod.ObjVar, $mod.IBird);',
    '$mod.v = $mod.IntfVar;',
    '$mod.IntfVar = rtl.getObject($mod.v);',
    'if (rtl.isExt($mod.v, $mod.IBird, 1)) ;',
    '$mod.v = $mod.IntfVar;',
    '$mod.v = $mod.IBird;',
    '']));
end;

procedure TTestModule.TestClassInterface_Corba_Args;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IBird = interface(IUnknown)',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject,IBird)',
  '  end;',
  'procedure DoIt(var u; i: IBird; const j: IBird);',
  'begin',
  '  DoIt(i,i,i);',
  'end;',
  'procedure Change(var i: IBird; out j: IBird);',
  'begin',
  '  DoIt(i,i,i);',
  '  Change(i,i);',
  'end;',
  'var',
  '  i: IBird;',
  '  o: TBird;',
  'begin',
  '  DoIt(i,i,i);',
  '  Change(i,i);',
  '  DoIt(o,o,o);',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_Corba_Args',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-B800-000000000000}", [], null);',
    'rtl.createInterface(this, "IBird", "{4B0D080B-C0F6-396E-AE88-000B87785074}", [], this.IUnknown);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  rtl.addIntf(this, $mod.IBird);',
    '});',
    'this.DoIt = function (u, i, j) {',
    '  $mod.DoIt({',
    '    get: function () {',
    '        return i;',
    '      },',
    '    set: function (v) {',
    '        i = v;',
    '      }',
    '  }, i, i);',
    '};',
    'this.Change = function (i, j) {',
    '  $mod.DoIt(i, i.get(), i.get());',
    '  $mod.Change(i, i);',
    '};',
    'this.i = null;',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '}, $mod.i, $mod.i);',
    '$mod.Change({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '}, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '});',
    '$mod.DoIt({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.o;',
    '    },',
    '  set: function (v) {',
    '      this.p.o = v;',
    '    }',
    '}, rtl.getIntfT($mod.o, $mod.IBird), rtl.getIntfT($mod.o, $mod.IBird));',
    '']));
end;

procedure TTestModule.TestClassInterface_Corba_ForIn;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface end;',
  '  TObject = class',
  '    Id: longint;',
  '  end;',
  '  IEnumerator = interface(IUnknown)',
  '    function GetCurrent: TObject;',
  '    function MoveNext: Boolean;',
  '    property Current: TObject read GetCurrent;',
  '  end;',
  '  IEnumerable = interface(IUnknown)',
  '    function GetEnumerator: IEnumerator;',
  '  end;',
  'var',
  '  o: TObject;',
  '  i: IEnumerable;',
  'begin',
  '  for o in i do o.Id:=3;',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_Corba_ForIn',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-B800-000000000000}", [], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.Id = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createInterface(this, "IEnumerator", "{95D7745D-ED61-3F13-BBE4-07708161999E}", ["GetCurrent", "MoveNext"], this.IUnknown);',
    'rtl.createInterface(this, "IEnumerable", "{8CC9D45D-ED7D-3B73-96B6-290B931BB19E}", ["GetEnumerator"], this.IUnknown);',
    'this.o = null;',
    'this.i = null;',
    '']),
    LinesToStr([ // $mod.$main
    'var $in = $mod.i.GetEnumerator();',
    'while ($in.MoveNext()) {',
    '  $mod.o = $in.GetCurrent();',
    '  $mod.o.Id = 3;',
    '};',
    '']));
end;

procedure TTestModule.TestClassInterface_COM_AssignVar;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '  end;',
  'var',
  '  i: IUnknown;',
  'procedure DoGlobal(o: TObject);',
  'begin',
  '  i:=nil;',
  '  i:=o;',
  '  i:=i;',
  'end;',
  'procedure DoLocal(o: TObject);',
  'const k: IUnknown = nil;',
  'var j: IUnknown;',
  'begin',
  '  k:=o;',
  '  k:=i;',
  '  j:=o;',
  '  j:=i;',
  'end;',
  'var o: TObject;',
  'begin',
  '  i:=nil;',
  '  i:=o;',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_AssignVar',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'this.i = null;',
    'this.DoGlobal = function (o) {',
    '  rtl.setIntfP($mod, "i", null);',
    '  rtl.setIntfP($mod, "i", rtl.queryIntfT(o, $mod.IUnknown), true);',
    '  rtl.setIntfP($mod, "i", $mod.i);',
    '};',
    'var k = null;',
    'this.DoLocal = function (o) {',
    '  var j = null;',
    '  try{',
    '    k = rtl.setIntfL(k, rtl.queryIntfT(o, $mod.IUnknown), true);',
    '    k = rtl.setIntfL(k, $mod.i);',
    '    j = rtl.setIntfL(j, rtl.queryIntfT(o, $mod.IUnknown), true);',
    '    j = rtl.setIntfL(j, $mod.i);',
    '  }finally{',
    '    rtl._Release(j);',
    '  };',
    '};',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    'rtl.setIntfP($mod, "i", null);',
    'rtl.setIntfP($mod, "i", rtl.queryIntfT($mod.o, $mod.IUnknown), true);',
    '']));
end;

procedure TTestModule.TestClassInterface_COM_AssignArg;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '  end;',
  'procedure DoDefault(i, j: IUnknown);',
  'begin',
  '  i:=nil;',
  '  i:=j;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_AssignArg',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'this.DoDefault = function (i, j) {',
    '  rtl._AddRef(i);',
    '  try {',
    '    i = rtl.setIntfL(i, null);',
    '    i = rtl.setIntfL(i, j);',
    '  } finally {',
    '    rtl._Release(i);',
    '  };',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_COM_FunctionResult;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '  end;',
  'function DoDefault(i: IUnknown): IUnknown;',
  'begin',
  '  Result:=i;',
  '  if Result<>nil then exit;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_FunctionResult',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'this.DoDefault = function (i) {',
    '  var Result = null;',
    '  var $ok = false;',
    '  try {',
    '    Result = rtl.setIntfL(Result, i);',
    '    if(Result !== null){',
    '      $ok = true;',
    '      return Result;',
    '    };',
    '    $ok = true;',
    '  } finally {',
    '    if(!$ok) rtl._Release(Result);',
    '  };',
    '  return Result;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_COM_InheritedFuncResult;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '    function GetIntf: IUnknown; virtual;',
  '  end;',
  '  TMouse = class',
  '    function GetIntf: IUnknown; override;',
  '  end;',
  'function TObject.GetIntf: IUnknown; begin end;',
  'function TMouse.GetIntf: IUnknown;',
  'var i: IUnknown;',
  'begin',
  '  inherited;',
  '  inherited GetIntf;',
  '  inherited GetIntf();',
  '  Result:=inherited GetIntf;',
  '  Result:=inherited GetIntf();',
  '  i:=inherited GetIntf;',
  '  i:=inherited GetIntf();',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_InheritedFuncResult',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetIntf = function () {',
    '    var Result = null;',
    '    return Result;',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'rtl.createClass(this, "TMouse", this.TObject, function () {',
    '  this.GetIntf = function () {',
    '    var Result = null;',
    '    var i = null;',
    '    var $ir = rtl.createIntfRefs();',
    '    var $ok = false;',
    '    try {',
    '      $ir.ref(1, $mod.TObject.GetIntf.call(this));',
    '      $ir.ref(2, $mod.TObject.GetIntf.call(this));',
    '      $ir.ref(3, $mod.TObject.GetIntf.call(this));',
    '      Result = rtl.setIntfL(Result, $mod.TObject.GetIntf.call(this), true);',
    '      Result = rtl.setIntfL(Result, $mod.TObject.GetIntf.call(this), true);',
    '      i = rtl.setIntfL(i, $mod.TObject.GetIntf.call(this), true);',
    '      i = rtl.setIntfL(i, $mod.TObject.GetIntf.call(this), true);',
    '      $ok = true;',
    '    } finally {',
    '      $ir.free();',
    '      rtl._Release(i);',
    '      if (!$ok) rtl._Release(Result);',
    '    };',
    '    return Result;',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_COM_IsAsTypeCasts;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '  end;',
  'procedure DoDefault(i, j: IUnknown; o: TObject);',
  'begin',
  '  if i is IUnknown then ;',
  '  if o is IUnknown then ;',
  '  if i is TObject then ;',
  '  i:=j as IUnknown;',
  '  i:=o as IUnknown;',
  '  o:=j as TObject;',
  '  i:=IUnknown(j);',
  '  i:=IUnknown(o);',
  '  o:=TObject(i);',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_IsAsTypeCasts',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'this.DoDefault = function (i, j, o) {',
    '  rtl._AddRef(i);',
    '  try {',
    '    if (rtl.intfIsIntfT(i, $mod.IUnknown)) ;',
    '    if (rtl.queryIntfIsT(o, $mod.IUnknown)) ;',
    '    if (rtl.intfIsClass(i, $mod.TObject)) ;',
    '    i = rtl.setIntfL(i, rtl.intfAsIntfT(j, $mod.IUnknown));',
    '    i = rtl.setIntfL(i, rtl.queryIntfT(o, $mod.IUnknown), true);',
    '    o = rtl.intfAsClass(j, $mod.TObject);',
    '    i = rtl.setIntfL(i, j);',
    '    i = rtl.setIntfL(i, rtl.queryIntfT(o, $mod.IUnknown), true);',
    '    o = rtl.intfToClass(i, $mod.TObject);',
    '  } finally {',
    '    rtl._Release(i);',
    '  };',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_COM_PassAsArg;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '  end;',
  'procedure DoIt(v: IUnknown; const j: IUnknown; var k: IUnknown; out l: IUnknown);',
  'var o: TObject;',
  'begin',
  '  DoIt(v,v,v,v);',
  '  DoIt(o,o,k,k);',
  'end;',
  'procedure DoSome;',
  'var v: IUnknown;',
  'begin',
  '  DoIt(v,v,v,v);',
  'end;',
  'var i: IUnknown;',
  'begin',
  '  DoIt(i,i,i,i);',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_PassAsArg',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'this.DoIt = function (v, j, k, l) {',
    '  var o = null;',
    '  var $ir = rtl.createIntfRefs();',
    '  rtl._AddRef(v);',
    '  try {',
    '    $mod.DoIt(v, v, {',
    '      get: function () {',
    '          return v;',
    '        },',
    '      set: function (w) {',
    '          v = rtl.setIntfL(v, w);',
    '        }',
    '    }, {',
    '      get: function () {',
    '          return v;',
    '        },',
    '      set: function (w) {',
    '          v = rtl.setIntfL(v, w);',
    '        }',
    '    });',
    '    $mod.DoIt($ir.ref(1, rtl.queryIntfT(o, $mod.IUnknown)), $ir.ref(2, rtl.queryIntfT(o, $mod.IUnknown)), k, k);',
    '  } finally {',
    '    $ir.free();',
    '    rtl._Release(v);',
    '  };',
    '};',
    'this.DoSome = function () {',
    '  var v = null;',
    '  try {',
    '    $mod.DoIt(v, v, {',
    '      get: function () {',
    '          return v;',
    '        },',
    '      set: function (w) {',
    '          v = rtl.setIntfL(v, w);',
    '        }',
    '    }, {',
    '      get: function () {',
    '          return v;',
    '        },',
    '      set: function (w) {',
    '          v = rtl.setIntfL(v, w);',
    '        }',
    '    });',
    '  } finally {',
    '    rtl._Release(v);',
    '  };',
    '};',
    'this.i = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt($mod.i, $mod.i, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      rtl.setIntfP(this.p, "i", v);',
    '    }',
    '}, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      rtl.setIntfP(this.p, "i", v);',
    '    }',
    '});',
    '']));
end;

procedure TTestModule.TestClassInterface_COM_PassToUntypedParam;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '  end;',
  'procedure DoIt(out i);',
  'begin end;',
  'procedure DoSome;',
  'var v: IUnknown;',
  'begin',
  '  DoIt(v);',
  'end;',
  'function GetIt: IUnknown;',
  'begin',
  '  DoIt(Result);',
  'end;',
  'var i: IUnknown;',
  'begin',
  '  DoIt(i);',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_PassToUntypedParam',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'this.DoIt = function (i) {',
    '};',
    'this.DoSome = function () {',
    '  var v = null;',
    '  try {',
    '    $mod.DoIt({',
    '      get: function () {',
    '          return v;',
    '        },',
    '      set: function (w) {',
    '          v = w;',
    '        }',
    '    });',
    '  } finally {',
    '    rtl._Release(v);',
    '  };',
    '};',
    'this.GetIt = function () {',
    '  var Result = null;',
    '  var $ok = false;',
    '  try {',
    '    $mod.DoIt({',
    '      get: function () {',
    '          return Result;',
    '        },',
    '      set: function (v) {',
    '          Result = v;',
    '        }',
    '    });',
    '    $ok = true;',
    '  } finally {',
    '    if (!$ok) rtl._Release(Result);',
    '  };',
    '  return Result;',
    '};',
    'this.i = null;',
    '']),
    LinesToStr([ // $mod.$main
    'try {',
    '  $mod.DoIt({',
    '    p: $mod,',
    '    get: function () {',
    '        return this.p.i;',
    '      },',
    '    set: function (v) {',
    '        this.p.i = v;',
    '      }',
    '  });',
    '} finally {',
    '  rtl._Release($mod.i);',
    '};',
    '']));
end;

procedure TTestModule.TestClassInterface_COM_FunctionInExpr;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '  end;',
  'function GetIt: IUnknown;',
  'begin',
  'end;',
  'procedure DoSome;',
  'var v: IUnknown;',
  '  i: longint;',
  'begin',
  '  v:=GetIt;',
  '  v:=GetIt();',
  '  GetIt()._AddRef;',
  '  i:=GetIt()._AddRef;',
  'end;',
  'var v: IUnknown;',
  '  i: longint;',
  'begin',
  '  v:=GetIt;',
  '  v:=GetIt();',
  '  GetIt()._AddRef;',
  '  i:=GetIt()._AddRef;',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_FunctionInExpr',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'this.GetIt = function () {',
    '  var Result = null;',
    '  return Result;',
    '};',
    'this.DoSome = function () {',
    '  var v = null;',
    '  var i = 0;',
    '  var $ir = rtl.createIntfRefs();',
    '  try {',
    '    v = rtl.setIntfL(v, $mod.GetIt(), true);',
    '    v = rtl.setIntfL(v, $mod.GetIt(), true);',
    '    $ir.ref(1, $mod.GetIt())._AddRef();',
    '    i = $ir.ref(2, $mod.GetIt())._AddRef();',
    '  } finally {',
    '    $ir.free();',
    '    rtl._Release(v);',
    '  };',
    '};',
    'this.v = null;',
    'this.i = 0;',
    '']),
    LinesToStr([ // $mod.$main
    'var $ir = rtl.createIntfRefs();',
    'try {',
    '  rtl.setIntfP($mod, "v", $mod.GetIt(), true);',
    '  rtl.setIntfP($mod, "v", $mod.GetIt(), true);',
    '  $ir.ref(1, $mod.GetIt())._AddRef();',
    '  $mod.i = $ir.ref(2, $mod.GetIt())._AddRef();',
    '} finally {',
    '  $ir.free();',
    '};',
    '']));
end;

procedure TTestModule.TestClassInterface_COM_Property;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    FAnt: IUnknown;',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '    function GetBird: IUnknown; virtual; abstract;',
  '    procedure SetBird(Value: IUnknown); virtual; abstract;',
  '    function GetItems(Index: longint): IUnknown; virtual; abstract;',
  '    procedure SetItems(Index: longint; Value: IUnknown); virtual; abstract;',
  '    property Ant: IUnknown read FAnt write FAnt;',
  '    property Bird: IUnknown read GetBird write SetBird;',
  '    property Items[Index: longint]: IUnknown read GetItems write SetItems; default;',
  '  end;',
  'procedure DoIt;',
  'var',
  '  o: TObject;',
  '  v: IUnknown;',
  'begin',
  '  v:=o.Ant;',
  '  o.Ant:=v;',
  '  o.Ant:=o.Ant;',
  '  v:=o.Bird;',
  '  o.Bird:=v;',
  '  o.Bird:=o.Bird;',
  '  v:=o.Items[1];',
  '  o.Items[2]:=v;',
  '  o.Items[3]:=o.Items[4];',
  '  v:=o[5];',
  '  o[6]:=v;',
  '  o[7]:=o[8];',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_Property',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FAnt = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FAnt = undefined;',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'this.DoIt = function () {',
    '  var o = null;',
    '  var v = null;',
    '  var $ir = rtl.createIntfRefs();',
    '  try {',
    '    v = rtl.setIntfL(v, o.FAnt);',
    '    rtl.setIntfP(o, "FAnt", v);',
    '    rtl.setIntfP(o, "FAnt", o.FAnt);',
    '    v = rtl.setIntfL(v, o.GetBird(), true);',
    '    o.SetBird(v);',
    '    o.SetBird($ir.ref(1, o.GetBird()));',
    '    v = rtl.setIntfL(v, o.GetItems(1), true);',
    '    o.SetItems(2, v);',
    '    o.SetItems(3, $ir.ref(2, o.GetItems(4)));',
    '    v = rtl.setIntfL(v, o.GetItems(5), true);',
    '    o.SetItems(6, v);',
    '    o.SetItems(7, $ir.ref(3, o.GetItems(8)));',
    '  } finally {',
    '    $ir.free();',
    '    rtl._Release(v);',
    '  };',
    '};',
    '']),
    LinesToStr([ // $mod.$main

    '']));
end;

procedure TTestModule.TestClassInterface_COM_IntfProperty;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '    function GetBird: IUnknown;',
  '    procedure SetBird(Value: IUnknown);',
  '    function GetItems(Index: longint): IUnknown;',
  '    procedure SetItems(Index: longint; Value: IUnknown);',
  '    property Bird: IUnknown read GetBird write SetBird;',
  '    property Items[Index: longint]: IUnknown read GetItems write SetItems; default;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '    function GetBird: IUnknown; virtual; abstract;',
  '    procedure SetBird(Value: IUnknown); virtual; abstract;',
  '    function GetItems(Index: longint): IUnknown; virtual; abstract;',
  '    procedure SetItems(Index: longint; Value: IUnknown); virtual; abstract;',
  '  end;',
  'procedure DoIt;',
  'var',
  '  o: TObject;',
  '  v: IUnknown;',
  'begin',
  '  v:=v.Items[1];',
  '  v.Items[2]:=v;',
  '  v.Items[3]:=v.Items[4];',
  '  v:=v[5];',
  '  v[6]:=v;',
  '  v[7]:=v[8];',
  '  v[9].Bird.Bird:=v;',
  '  v:=v.Bird[10].Bird',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_IntfProperty',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{385F5482-571B-338C-8130-4E97F330543B}", [',
    '  "_AddRef",',
    '  "_Release",',
    '  "GetBird",',
    '  "SetBird",',
    '  "GetItems",',
    '  "SetItems"',
    '], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'this.DoIt = function () {',
    '  var o = null;',
    '  var v = null;',
    '  var $ir = rtl.createIntfRefs();',
    '  try {',
    '    v = rtl.setIntfL(v, v.GetItems(1), true);',
    '    v.SetItems(2, v);',
    '    v.SetItems(3, $ir.ref(1, v.GetItems(4)));',
    '    v = rtl.setIntfL(v, v.GetItems(5), true);',
    '    v.SetItems(6, v);',
    '    v.SetItems(7, $ir.ref(2, v.GetItems(8)));',
    '    $ir.ref(4, $ir.ref(3, v.GetItems(9)).GetBird()).SetBird(v);',
    '    v = rtl.setIntfL(v, $ir.ref(6, $ir.ref(5, v.GetBird()).GetItems(10)).GetBird(), true);',
    '  } finally {',
    '    $ir.free();',
    '    rtl._Release(v);',
    '  };',
    '};',
    '']),
    LinesToStr([ // $mod.$main

    '']));
end;

procedure TTestModule.TestClassInterface_COM_Delegation;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  IBird = interface(IUnknown)',
  '    procedure Fly(s: string);',
  '  end;',
  '  IEagle = interface(IBird) end;',
  '  IDove = interface(IBird) end;',
  '  ISwallow = interface(IBird) end;',
  '  TObject = class',
  '  end;',
  '  TBird = class(TObject,IBird,IEagle,IDove,ISwallow)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '    procedure Fly(s: string); virtual; abstract;',
  '  end;',
  '  TBat = class(IBird,IEagle,IDove,ISwallow)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '    FBirdIntf: IBird;',
  '    property BirdIntf: IBird read FBirdIntf implements IBird;',
  '    function GetEagleIntf: IEagle; virtual; abstract;',
  '    property EagleIntf: IEagle read GetEagleIntf implements IEagle;',
  '    FDoveObj: TBird;',
  '    property DoveObj: TBird read FDoveObj implements IDove;',
  '    function GetSwallowObj: TBird; virtual; abstract;',
  '    property SwallowObj: TBird read GetSwallowObj implements ISwallow;',
  '  end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_Delegation',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createInterface(this, "IBird", "{CC440C7F-7623-3DEE-AE88-000B86AAF108}", ["Fly"], this.IUnknown);',
    'rtl.createInterface(this, "IEagle", "{4B6A41C9-B020-3D7C-B688-96D19022B1B4}", [], this.IBird);',
    'rtl.createInterface(this, "IDove", "{4B6A41C9-B020-3D7C-B688-96D18EF16074}", [], this.IBird);',
    'rtl.createInterface(this, "ISwallow", "{BB6A41C9-B020-3D7C-B688-96D1CBDCB359}", [], this.IBird);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  rtl.addIntf(this, $mod.IBird);',
    '  rtl.addIntf(this, $mod.IEagle);',
    '  rtl.addIntf(this, $mod.IDove);',
    '  rtl.addIntf(this, $mod.ISwallow);',
    '});',
    'rtl.createClass(this, "TBat", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.FBirdIntf = null;',
    '    this.FDoveObj = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FBirdIntf = undefined;',
    '    this.FDoveObj = undefined;',
    '    $mod.TObject.$final.call(this);',
    '  };',
    '  this.$intfmaps = {',
    '    "{CC440C7F-7623-3DEE-AE88-000B86AAF108}": function () {',
    '        return rtl._AddRef(this.FBirdIntf);',
    '      },',
    '    "{4B6A41C9-B020-3D7C-B688-96D19022B1B4}": function () {',
    '        return this.GetEagleIntf();',
    '      },',
    '    "{4B6A41C9-B020-3D7C-B688-96D18EF16074}": function () {',
    '        return rtl.queryIntfT(this.FDoveObj, $mod.IDove);',
    '      },',
    '    "{BB6A41C9-B020-3D7C-B688-96D1CBDCB359}": function () {',
    '        return rtl.queryIntfT(this.GetSwallowObj(), $mod.ISwallow);',
    '      }',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_COM_With;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '    function GetAnt: IUnknown;',
  '    property Ant: IUnknown read GetAnt;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '    function GetAnt: IUnknown; virtual; abstract;',
  '    property Ant: IUnknown read GetAnt;',
  '  end;',
  'procedure DoIt;',
  'var',
  '  i: IUnknown;',
  'begin',
  '  with i do ',
  '    GetAnt;',
  '  with i.Ant, Ant do ',
  '    GetAnt;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_With',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB00D-C6B6-39FB-BDDF-21CD521DDFA9}", ["_AddRef", "_Release", "GetAnt"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'this.DoIt = function () {',
    '  var i = null;',
    '  var $ir = rtl.createIntfRefs();',
    '  try {',
    '    $ir.ref(1, i.GetAnt());',
    '    var $with = $ir.ref(2, i.GetAnt());',
    '    var $with1 = $ir.ref(3, $with.GetAnt());',
    '    $ir.ref(4, $with1.GetAnt());',
    '  } finally {',
    '    $ir.free();',
    '  };',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassInterface_COM_ForIn;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface end;',
  '  TObject = class',
  '    Id: longint;',
  '  end;',
  '  IEnumerator = interface(IUnknown)',
  '    function GetCurrent: TObject;',
  '    function MoveNext: Boolean;',
  '    property Current: TObject read GetCurrent;',
  '  end;',
  '  IEnumerable = interface(IUnknown)',
  '    function GetEnumerator: IEnumerator;',
  '  end;',
  'var',
  '  o: TObject;',
  '  i: IEnumerable;',
  'begin',
  '  for o in i do o.Id:=3;',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_COM_ForIn',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{B92D5841-758A-322B-B800-000000000000}", [], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.Id = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createInterface(this, "IEnumerator", "{95D7745D-ED61-3F13-BBE4-07708161999E}", ["GetCurrent", "MoveNext"], this.IUnknown);',
    'rtl.createInterface(this, "IEnumerable", "{8CC9D45D-ED7D-3B73-96B6-290B931BB19E}", ["GetEnumerator"], this.IUnknown);',
    'this.o = null;',
    'this.i = null;',
    '']),
    LinesToStr([ // $mod.$main
    'var $in = $mod.i.GetEnumerator();',
    'try {',
    '  while ($in.MoveNext()) {',
    '    $mod.o = $in.GetCurrent();',
    '    $mod.o.Id = 3;',
    '  }',
    '} finally {',
    '  rtl._Release($in)',
    '};',
    '']));
end;

procedure TTestModule.TestClassInterface_COM_ArrayOfIntfFail;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class',
  '  end;',
  '  TArrOfIntf = array of IUnknown;',
  'begin',
  '']);
  SetExpectedPasResolverError('Not supported: array of COM-interface',nNotSupportedX);
  ConvertProgram;
end;

procedure TTestModule.TestClassInterface_COM_RecordIntfFail;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TRec = record',
  '    i: IUnknown;',
  '  end;',
  'begin',
  '']);
  SetExpectedPasResolverError('Not supported: COM-interface as record member',nNotSupportedX);
  ConvertProgram;
end;

procedure TTestModule.TestClassInterface_COM_UnitInitialization;
begin
  StartUnit(false);
  Add([
  '{$interfaces com}',
  'interface',
  'implementation',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint;',
  '  end;',
  'function TObject._AddRef: longint; begin end;',
  'var i: IUnknown;',
  '  o: TObject;',
  'initialization',
  '  i:=nil;',
  '  i:=i;',
  '  i:=o;',
  '  if (o as IUnknown)=nil then ;',
  '']);
  ConvertUnit;
  CheckSource('TestClassInterface_COM_UnitInitialization',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    '']),
    LinesToStr([ // this.$init
    'var $ir = rtl.createIntfRefs();',
    'try {',
    '  rtl.setIntfP($impl, "i", null);',
    '  rtl.setIntfP($impl, "i", $impl.i);',
    '  rtl.setIntfP($impl, "i", rtl.queryIntfT($impl.o, $impl.IUnknown), true);',
    '  if ($ir.ref(1, rtl.queryIntfT($impl.o, $impl.IUnknown)) === null) ;',
    '} finally {',
    '  $ir.free();',
    '};',
    '']),
    LinesToStr([ // implementation
    'rtl.createInterface($impl, "IUnknown", "{B92D5841-758A-322B-BDDF-21CD52180000}", ["_AddRef"], null);',
    'rtl.createClass($impl, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this._AddRef = function () {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  rtl.addIntf(this, $impl.IUnknown);',
    '});',
    '$impl.i = null;',
    '$impl.o = null;',
    ''])
    );
end;

procedure TTestModule.TestClassInterface_GUID;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '    [''{f31db68f-3010-D355-4EBA-CDD4EF4A737C}'']',
  '  end;',
  '  TObject = class end;',
  '  TGUID = record D1, D2, D3, D4: word; end;',
  '  TAliasGUID = TGUID;',
  '  TGUIDString = type string;',
  '  TAliasGUIDString = TGUIDString;',
  'procedure DoConstGUIDIt(const g: TAliasGUID); overload;',
  'begin end;',
  'procedure DoDefGUID(g: TAliasGUID); overload;',
  'begin end;',
  'procedure DoStr(const s: TAliasGUIDString); overload;',
  'begin end;',
  'var',
  '  i: IUnknown;',
  '  g: TAliasGUID = ''{d91c9af4-3C93-420F-A303-BF5BA82BFD23}'';',
  '  s: TAliasGUIDString;',
  'begin',
  '  DoConstGUIDIt(IUnknown);',
  '  DoDefGUID(IUnknown);',
  '  DoStr(IUnknown);',
  '  DoConstGUIDIt(i);',
  '  DoDefGUID(i);',
  '  DoStr(i);',
  '  DoConstGUIDIt(''{D91C9AF4-3c93-420f-A303-BF5BA82BFD23}'');',
  '  DoDefGUID(''{D91C9AF4-3c93-420f-A303-BF5BA82BFD23}'');',
  '  DoStr(g);',
  '  g:=i;',
  '  g:=IUnknown;',
  '  g:=''{D91C9AF4-3C93-420F-A303-bf5ba82bfd23}'';',
  '  s:=i;',
  '  s:=IUnknown;',
  '  s:=g;',
  '  if g=i then ;',
  '  if i=g then ;',
  '  if g=IUnknown then ;',
  '  if IUnknown=g then ;',
  '  if s=i then ;',
  '  if i=s then ;',
  '  if s=IUnknown then ;',
  '  if IUnknown=s then ;',
  '  if s=g then ;',
  '  if g=s then ;',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_GUID',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{F31DB68F-3010-D355-4EBA-CDD4EF4A737C}", [], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.recNewT(this, "TGUID", function () {',
    '  this.D1 = 0;',
    '  this.D2 = 0;',
    '  this.D3 = 0;',
    '  this.D4 = 0;',
    '  this.$eq = function (b) {',
    '    return (this.D1 === b.D1) && (this.D2 === b.D2) && (this.D3 === b.D3) && (this.D4 === b.D4);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.D1 = s.D1;',
    '    this.D2 = s.D2;',
    '    this.D3 = s.D3;',
    '    this.D4 = s.D4;',
    '    return this;',
    '  };',
    '});',
    'this.DoConstGUIDIt = function (g) {',
    '};',
    'this.DoDefGUID = function (g) {',
    '};',
    'this.DoStr = function (s) {',
    '};',
    'this.i = null;',
    'this.g = this.TGUID.$clone({',
    '  D1: 0xD91C9AF4,',
    '  D2: 0x3C93,',
    '  D3: 0x420F,',
    '  D4: [',
    '      0xA3,',
    '      0x03,',
    '      0xBF,',
    '      0x5B,',
    '      0xA8,',
    '      0x2B,',
    '      0xFD,',
    '      0x23',
    '    ]',
    '});',
    'this.s = "";',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoConstGUIDIt(rtl.getIntfGUIDR($mod.IUnknown));',
    '$mod.DoDefGUID($mod.TGUID.$clone(rtl.getIntfGUIDR($mod.IUnknown)));',
    '$mod.DoStr($mod.IUnknown.$guid);',
    '$mod.DoConstGUIDIt(rtl.getIntfGUIDR($mod.i));',
    '$mod.DoDefGUID($mod.TGUID.$clone(rtl.getIntfGUIDR($mod.i)));',
    '$mod.DoStr($mod.i.$guid);',
    '$mod.DoConstGUIDIt(rtl.strToGUIDR("{D91C9AF4-3c93-420f-A303-BF5BA82BFD23}"));',
    '$mod.DoDefGUID(rtl.strToGUIDR("{D91C9AF4-3c93-420f-A303-BF5BA82BFD23}"));',
    '$mod.DoStr(rtl.guidrToStr($mod.g));',
    '$mod.g.$assign(rtl.getIntfGUIDR($mod.i));',
    '$mod.g.$assign(rtl.getIntfGUIDR($mod.IUnknown));',
    '$mod.g.$assign({',
    '  D1: 0xD91C9AF4,',
    '  D2: 0x3C93,',
    '  D3: 0x420F,',
    '  D4: [',
    '      0xA3,',
    '      0x03,',
    '      0xBF,',
    '      0x5B,',
    '      0xA8,',
    '      0x2B,',
    '      0xFD,',
    '      0x23',
    '    ]',
    '});',
    '$mod.s = $mod.i.$guid;',
    '$mod.s = $mod.IUnknown.$guid;',
    '$mod.s = rtl.guidrToStr($mod.g);',
    'if ($mod.g.$eq(rtl.getIntfGUIDR($mod.i))) ;',
    'if ($mod.g.$eq(rtl.getIntfGUIDR($mod.i))) ;',
    'if ($mod.g.$eq(rtl.getIntfGUIDR($mod.IUnknown))) ;',
    'if ($mod.g.$eq(rtl.getIntfGUIDR($mod.IUnknown))) ;',
    'if ($mod.s === $mod.i.$guid) ;',
    'if ($mod.i.$guid === $mod.s) ;',
    'if ($mod.s === $mod.IUnknown.$guid) ;',
    'if ($mod.IUnknown.$guid === $mod.s) ;',
    'if ($mod.g.$eq(rtl.createTGUID($mod.s))) ;',
    'if ($mod.g.$eq(rtl.createTGUID($mod.s))) ;',
    '']));
end;

procedure TTestModule.TestClassInterface_GUIDProperty;
begin
  StartProgram(false);
  Add([
  '{$interfaces corba}',
  'type',
  '  IUnknown = interface',
  '    [''{f31db68f-3010-D355-4EBA-CDD4EF4A737C}'']',
  '  end;',
  '  TGUID = record D1, D2, D3, D4: word; end;',
  '  TAliasGUID = TGUID;',
  '  TGUIDString = type string;',
  '  TAliasGUIDString = TGUIDString;',
  '  TObject = class',
  '    function GetG: TAliasGUID; virtual; abstract;',
  '    procedure SetG(const Value: TAliasGUID); virtual; abstract;',
  '    function GetS: TAliasGUIDString; virtual; abstract;',
  '    procedure SetS(const Value: TAliasGUIDString); virtual; abstract;',
  '    property g: TAliasGUID read GetG write SetG;',
  '    property s: TAliasGUIDString read GetS write SetS;',
  '  end;',
  'var o: TObject;',
  'begin',
  '  o.g:=IUnknown;',
  '  o.g:=''{D91C9AF4-3C93-420F-A303-bf5ba82bfd23}'';',
  '  o.s:=IUnknown;',
  '  o.s:=o.g;',
  '']);
  ConvertProgram;
  CheckSource('TestClassInterface_GUIDProperty',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{F31DB68F-3010-D355-4EBA-CDD4EF4A737C}", [], null);',
    'rtl.recNewT(this, "TGUID", function () {',
    '  this.D1 = 0;',
    '  this.D2 = 0;',
    '  this.D3 = 0;',
    '  this.D4 = 0;',
    '  this.$eq = function (b) {',
    '    return (this.D1 === b.D1) && (this.D2 === b.D2) && (this.D3 === b.D3) && (this.D4 === b.D4);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.D1 = s.D1;',
    '    this.D2 = s.D2;',
    '    this.D3 = s.D3;',
    '    this.D4 = s.D4;',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.o.SetG(rtl.getIntfGUIDR($mod.IUnknown));',
    '$mod.o.SetG({',
    '  D1: 0xD91C9AF4,',
    '  D2: 0x3C93,',
    '  D3: 0x420F,',
    '  D4: [',
    '      0xA3,',
    '      0x03,',
    '      0xBF,',
    '      0x5B,',
    '      0xA8,',
    '      0x2B,',
    '      0xFD,',
    '      0x23',
    '    ]',
    '});',
    '$mod.o.SetS($mod.IUnknown.$guid);',
    '$mod.o.SetS(rtl.guidrToStr($mod.o.GetG()));',
    '']));
end;

procedure TTestModule.TestClassHelper_ClassVar;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  THelper = class helper for TObject',
  '    const',
  '      One = 1;',
  '      Two: word = 2;',
  '    class var',
  '      Glob: word;',
  '    function Foo(w: word): word;',
  '    class function Bar(w: word): word;',
  '  end;',
  'function THelper.foo(w: word): word;',
  'begin',
  '  Result:=w;',
  '  Two:=One+w;',
  '  Glob:=Glob;',
  '  Result:=Self.Glob;',
  '  Self.Glob:=Self.Glob;',
  '  with Self do Glob:=Glob;',
  'end;',
  'class function THelper.bar(w: word): word;',
  'begin',
  '  Result:=w;',
  '  Two:=One;',
  '  Glob:=Glob;',
  '  Self.Glob:=Self.Glob;',
  '  with Self do Glob:=Glob;',
  'end;',
  'var o: TObject;',
  'begin',
  '  tobject.two:=tobject.one;',
  '  tobject.Glob:=tobject.Glob;',
  '  with tobject do begin',
  '    two:=one;',
  '    Glob:=Glob;',
  '  end;',
  '  o.two:=o.one;',
  '  o.Glob:=o.Glob;',
  '  with o do begin',
  '    two:=one;',
  '    Glob:=Glob;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_ClassVar',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.One = 1;',
    '  this.Two = 2;',
    '  this.Glob = 0;',
    '  this.Foo = function (w) {',
    '    var Result = 0;',
    '    Result = w;',
    '    $mod.THelper.Two = 1 + w;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    Result = $mod.THelper.Glob;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    return Result;',
    '  };',
    '  this.Bar = function (w) {',
    '    var Result = 0;',
    '    Result = w;',
    '    $mod.THelper.Two = 1;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    return Result;',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    'var $with = $mod.TObject;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    'var $with1 = $mod.o;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    '']));
end;

procedure TTestModule.TestClassHelper_Method_AccessInstanceFields;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    FSize: word;',
  '    property Size: word read FSize write FSize;',
  '  end;',
  '  THelper = class helper for TObject',
  '    function Foo(w: word = 1): word;',
  '  end;',
  'function THelper.foo(w: word): word;',
  'begin',
  '  Result:=Size;',
  '  Size:=Size+2;',
  '  Self.Size:=Self.Size+3;',
  '  FSize:=FSize+4;',
  '  Self.FSize:=Self.FSize+5;',
  '  with Self do begin',
  '    Size:=Size+6;',
  '    FSize:=FSize+7;',
  '    FSize:=FSize+8;',
  '  end;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_Method_AccessInstanceFields',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FSize = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Foo = function (w) {',
    '    var Result = 0;',
    '    Result = this.FSize;',
    '    this.FSize = this.FSize + 2;',
    '    this.FSize = this.FSize + 3;',
    '    this.FSize = this.FSize + 4;',
    '    this.FSize = this.FSize + 5;',
    '    this.FSize = this.FSize + 6;',
    '    this.FSize = this.FSize + 7;',
    '    this.FSize = this.FSize + 8;',
    '    return Result;',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassHelper_Method_Call;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure Run(w: word = 10);',
  '  end;',
  '  THelper = class helper for TObject',
  '    function Foo(w: word = 1): word;',
  '  end;',
  'procedure TObject.Run(w: word);',
  'var o: TObject;',
  'begin',
  '  Foo;',
  '  Foo();',
  '  Foo(2);',
  '  Self.Foo;',
  '  Self.Foo();',
  '  Self.Foo(3);',
  '  with Self do begin',
  '    Foo;',
  '    Foo();',
  '    Foo(4);',
  '  end;',
  '  with o do Foo(5);',
  'end;',
  'function THelper.foo(w: word): word;',
  'begin',
  '  Run;',
  '  Run();',
  '  Run(11);',
  '  Foo;',
  '  Foo();',
  '  Foo(12);',
  '  Self.Foo;',
  '  Self.Foo();',
  '  Self.Foo(13);',
  '  with Self do begin',
  '    Foo;',
  '    Foo();',
  '    Foo(14);',
  '  end;',
  'end;',
  'var Obj: TObject;',
  'begin',
  '  obj.Foo;',
  '  obj.Foo();',
  '  obj.Foo(21);',
  '  with obj do begin',
  '    Foo;',
  '    Foo();',
  '    Foo(22);',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_Method_Call',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Run = function (w) {',
    '    var o = null;',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 2);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 3);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 4);',
    '    $mod.THelper.Foo.call(o, 5);',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Foo = function (w) {',
    '    var Result = 0;',
    '    this.Run(10);',
    '    this.Run(10);',
    '    this.Run(11);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 12);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 13);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 14);',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Foo.call($mod.Obj, 1);',
    '$mod.THelper.Foo.call($mod.Obj, 1);',
    '$mod.THelper.Foo.call($mod.Obj, 21);',
    'var $with = $mod.Obj;',
    '$mod.THelper.Foo.call($with, 1);',
    '$mod.THelper.Foo.call($with, 1);',
    '$mod.THelper.Foo.call($with, 22);',
    '']));
end;

procedure TTestModule.TestClassHelper_Method_Nested_Call;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure Run(w: word = 10);',
  '  end;',
  '  THelper = class helper for TObject',
  '    function Foo(w: word = 1): word;',
  '  end;',
  'procedure TObject.Run(w: word);',
  '  procedure Sub(Self: TObject);',
  '  begin',
  '    Foo;',
  '    Foo();',
  '    Self.Foo;',
  '    Self.Foo();',
  '    with Self do begin',
  '      Foo;',
  '      Foo();',
  '    end;',
  '  end;',
  'begin',
  'end;',
  'function THelper.foo(w: word): word;',
  '  procedure Sub(Self: TObject);',
  '  begin',
  '    Run;',
  '    Run();',
  '    Foo;',
  '    Foo();',
  '    Self.Foo;',
  '    Self.Foo();',
  '    with Self do begin',
  '      Foo;',
  '      Foo();',
  '    end;',
  '  end;',
  'begin',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_Method_Nested_Call',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Run = function (w) {',
    '    var $Self = this;',
    '    function Sub(Self) {',
    '      $mod.THelper.Foo.call($Self, 1);',
    '      $mod.THelper.Foo.call($Self, 1);',
    '      $mod.THelper.Foo.call(Self, 1);',
    '      $mod.THelper.Foo.call(Self, 1);',
    '      $mod.THelper.Foo.call(Self, 1);',
    '      $mod.THelper.Foo.call(Self, 1);',
    '    };',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Foo = function (w) {',
    '    var $Self = this;',
    '    var Result = 0;',
    '    function Sub(Self) {',
    '      $Self.Run(10);',
    '      $Self.Run(10);',
    '      $mod.THelper.Foo.call($Self, 1);',
    '      $mod.THelper.Foo.call($Self, 1);',
    '      $mod.THelper.Foo.call(Self, 1);',
    '      $mod.THelper.Foo.call(Self, 1);',
    '      $mod.THelper.Foo.call(Self, 1);',
    '      $mod.THelper.Foo.call(Self, 1);',
    '    };',
    '    return Result;',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassHelper_ClassMethod_Call;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class procedure Run(w: word = 10);',
  '  end;',
  '  THelper = class helper for TObject',
  '    class function Foo(w: word = 1): word;',
  '  end;',
  'class procedure TObject.Run(w: word);',
  'begin',
  '  Foo;',
  '  Foo();',
  '  Self.Foo;',
  '  Self.Foo();',
  '  with Self do begin',
  '    Foo;',
  '    Foo();',
  '  end;',
  'end;',
  'class function THelper.foo(w: word): word;',
  'begin',
  '  Run;',
  '  Run();',
  '  Foo;',
  '  Foo();',
  '  Self.Foo;',
  '  Self.Foo();',
  '  with Self do begin',
  '    Foo;',
  '    Foo();',
  '  end;',
  'end;',
  'var',
  '  Obj: TObject;',
  'begin',
  '  obj.Foo;',
  '  obj.Foo();',
  '  with obj do begin',
  '    Foo;',
  '    Foo();',
  '  end;',
  '  tobject.Foo;',
  '  tobject.Foo();',
  '  with tobject do begin',
  '    Foo;',
  '    Foo();',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_ClassMethod_Call',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Run = function (w) {',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Foo = function (w) {',
    '    var Result = 0;',
    '    this.Run(10);',
    '    this.Run(10);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Foo.call($mod.Obj.$class, 1);',
    '$mod.THelper.Foo.call($mod.Obj.$class, 1);',
    'var $with = $mod.Obj;',
    '$mod.THelper.Foo.call($with.$class, 1);',
    '$mod.THelper.Foo.call($with.$class, 1);',
    '$mod.THelper.Foo.call($mod.TObject, 1);',
    '$mod.THelper.Foo.call($mod.TObject, 1);',
    'var $with1 = $mod.TObject;',
    '$mod.THelper.Foo.call($mod.TObject, 1);',
    '$mod.THelper.Foo.call($mod.TObject, 1);',
    '']));
end;

procedure TTestModule.TestClassHelper_ClassOf;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TClass = class of TObject;',
  '  THelper = class helper for TObject',
  '    class function Foo(w: word = 1): word;',
  '  end;',
  'class function THelper.foo(w: word): word;',
  'begin',
  'end;',
  'var',
  '  c: TClass;',
  'begin',
  '  c.Foo;',
  '  c.Foo();',
  '  with c do begin',
  '    Foo;',
  '    Foo();',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_ClassOf',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Foo = function (w) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'this.c = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Foo.call($mod.c, 1);',
    '$mod.THelper.Foo.call($mod.c, 1);',
    'var $with = $mod.c;',
    '$mod.THelper.Foo.call($with, 1);',
    '$mod.THelper.Foo.call($with, 1);',
    '']));
end;

procedure TTestModule.TestClassHelper_MethodRefObjFPC;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  'type',
  '  TObject = class',
  '    procedure DoIt;',
  '  end;',
  '  THelper = class helper for TObject',
  '    procedure Fly(w: word = 1);',
  '    class procedure Glide(w: word = 1);',
  '    class procedure Run(w: word = 1); static;',
  '  end;',
  '  TFly = procedure(w: word) of object;',
  '  TGlide = TFly;',
  '  TRun = procedure(w: word);',
  'var',
  '  f: TFly;',
  '  g: TGlide;',
  '  r: TRun;',
  'procedure TObject.DoIt;',
  'begin',
  '  f:=@fly;',
  '  g:=@glide;',
  '  r:=@run;',
  '  f:=@Self.fly;',
  '  g:=@Self.glide;',
  '  r:=@Self.run;',
  '  with self do begin',
  '    f:=@fly;',
  '    g:=@glide;',
  '    r:=@run;',
  '  end;',
  'end;',
  'procedure THelper.fly(w: word);',
  'begin',
  '  f:=@fly;',
  '  g:=@glide;',
  '  r:=@run;',
  'end;',
  'class procedure THelper.glide(w: word);',
  'begin',
  '  g:=@glide;',
  '  r:=@run;',
  'end;',
  'class procedure THelper.run(w: word);',
  'begin',
  '  g:=@glide;',
  '  r:=@run;',
  'end;',
  'var',
  '  Obj: TObject;',
  'begin',
  '  f:=@obj.fly;',
  '  g:=@obj.glide;',
  '  r:=@obj.run;',
  '  with obj do begin',
  '    f:=@fly;',
  '    g:=@glide;',
  '    r:=@run;',
  '  end;',
  '  g:=@tobject.glide;',
  '  r:=@tobject.run;',
  '  with tobject do begin',
  '    g:=@glide;',
  '    r:=@run;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_MethodRefObjFPC',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '    $mod.f = rtl.createCallback(this, $mod.THelper.Fly);',
    '    $mod.g = rtl.createCallback(this.$class, $mod.THelper.Glide);',
    '    $mod.r = $mod.THelper.Run;',
    '    $mod.f = rtl.createCallback(this, $mod.THelper.Fly);',
    '    $mod.g = rtl.createCallback(this.$class, $mod.THelper.Glide);',
    '    $mod.r = $mod.THelper.Run;',
    '    $mod.f = rtl.createCallback(this, $mod.THelper.Fly);',
    '    $mod.g = rtl.createCallback(this.$class, $mod.THelper.Glide);',
    '    $mod.r = $mod.THelper.Run;',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Fly = function (w) {',
    '    $mod.f = rtl.createCallback(this, $mod.THelper.Fly);',
    '    $mod.g = rtl.createCallback(this.$class, $mod.THelper.Glide);',
    '    $mod.r = $mod.THelper.Run;',
    '  };',
    '  this.Glide = function (w) {',
    '    $mod.g = rtl.createCallback(this, $mod.THelper.Glide);',
    '    $mod.r = $mod.THelper.Run;',
    '  };',
    '  this.Run = function (w) {',
    '    $mod.g = rtl.createCallback($mod.THelper, $mod.THelper.Glide);',
    '    $mod.r = $mod.THelper.Run;',
    '  };',
    '});',
    'this.f = null;',
    'this.g = null;',
    'this.r = null;',
    'this.Obj = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.f = rtl.createCallback($mod.Obj, $mod.THelper.Fly);',
    '$mod.g = rtl.createCallback($mod.Obj.$class, $mod.THelper.Glide);',
    '$mod.r = $mod.THelper.Run;',
    'var $with = $mod.Obj;',
    '$mod.f = rtl.createCallback($with, $mod.THelper.Fly);',
    '$mod.g = rtl.createCallback($with.$class, $mod.THelper.Glide);',
    '$mod.r = $mod.THelper.Run;',
    '$mod.g = rtl.createCallback($mod.TObject, $mod.THelper.Glide);',
    '$mod.r = $mod.THelper.Run;',
    'var $with1 = $mod.TObject;',
    '$mod.g = rtl.createCallback($with1, $mod.THelper.Glide);',
    '$mod.r = $mod.THelper.Run;',
    '']));
end;

procedure TTestModule.TestClassHelper_Constructor;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  '  TClass = class of TObject;',
  '  THelper = class helper for TObject',
  '    constructor NewHlp(w: word);',
  '  end;',
  'var',
  '  obj: TObject;',
  '  c: TClass;',
  'constructor TObject.Create;',
  'begin',
  '  NewHlp(2);', // normal call
  '  tobject.NewHlp(3);', // new instance
  '  c.newhlp(4);', // new instance
  'end;',
  'constructor THelper.NewHlp(w: word);',
  'begin',
  '  create;', // normal call
  '  tobject.create;', // new instance
  '  NewHlp(2);', // normal call
  '  tobject.NewHlp(3);', // new instance
  '  c.newhlp(4);', // new instance
  'end;',
  'begin',
  '  obj.newhlp(2);', // normal call
  '  with Obj do newhlp(12);', // normal call
  '  tobject.newhlp(3);', // new instance
  '  with tobject do newhlp(13);', // new instance
  '  c.newhlp(4);', // new instance
  '  with c do newhlp(14);', // new instance
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_Constructor',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    $mod.THelper.NewHlp.call(this, 2);',
    '    $mod.TObject.$create($mod.THelper.NewHlp, [3]);',
    '    $mod.c.$create($mod.THelper.NewHlp, [4]);',
    '    return this;',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.NewHlp = function (w) {',
    '    this.Create();',
    '    $mod.TObject.$create("Create");',
    '    $mod.THelper.NewHlp.call(this, 2);',
    '    $mod.TObject.$create($mod.THelper.NewHlp, [3]);',
    '    $mod.c.$create($mod.THelper.NewHlp, [4]);',
    '    return this;',
    '  };',
    '});',
    'this.obj = null;',
    'this.c = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.NewHlp.call($mod.obj, 2);',
    'var $with = $mod.obj;',
    '$mod.THelper.NewHlp.call($with, 12);',
    '$mod.TObject.$create($mod.THelper.NewHlp, [3]);',
    'var $with1 = $mod.TObject;',
    '$with1.$create($mod.THelper.NewHlp, [13]);',
    '$mod.c.$create($mod.THelper.NewHlp, [4]);',
    'var $with2 = $mod.c;',
    '$with2.$create($mod.THelper.NewHlp, [14]);',
    '']));
end;

procedure TTestModule.TestClassHelper_InheritedObjFPC;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure Fly;',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    procedure Fly;',
  '  end;',
  '  TBird = class',
  '    procedure Fly;',
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '    procedure Fly;',
  '    procedure Walk(w: word);',
  '  end;',
  '  TEagleHelper = class helper(TBirdHelper) for TBird',
  '    procedure Fly;',
  '    procedure Walk(w: word);',
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
  'procedure Tbirdhelper.walk(w: word);',
  'begin',
  'end;',
  'procedure teagleHelper.fly;',
  'begin',
  '  {@TBird_Fly}inherited;',
  '  inherited {@TBird_Fly}Fly;',
  'end;',
  'procedure teagleHelper.walk(w: word);',
  'begin',
  '  {@TBirdHelper_Walk}inherited;',
  '  inherited {@TBirdHelper_Walk}Walk(3);',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_InheritedObjFPC',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Fly = function () {',
    '  };',
    '});',
    'rtl.createHelper(this, "TObjHelper", null, function () {',
    '  this.Fly = function () {',
    '    $mod.TObject.Fly.call(this);',
    '    $mod.TObject.Fly.call(this);',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.Fly$1 = function () {',
    '    $mod.TObjHelper.Fly.call(this);',
    '    $mod.TObjHelper.Fly.call(this);',
    '  };',
    '});',
    'rtl.createHelper(this, "TBirdHelper", null, function () {',
    '  this.Fly = function () {',
    '    $mod.TBird.Fly$1.call(this);',
    '    $mod.TBird.Fly$1.call(this);',
    '  };',
    '  this.Walk = function (w) {',
    '  };',
    '});',
    'rtl.createHelper(this, "TEagleHelper", this.TBirdHelper, function () {',
    '  this.Fly$1 = function () {',
    '    $mod.TBird.Fly$1.call(this);',
    '    $mod.TBird.Fly$1.call(this);',
    '  };',
    '  this.Walk$1 = function (w) {',
    '    $mod.TBirdHelper.Walk.apply(this, arguments);',
    '    $mod.TBirdHelper.Walk.call(this, 3);',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestClassHelper_Property;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    FSize: word;',
  '    function GetSpeed: word;',
  '    procedure SetSpeed(Value: word);',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    function GetLeft: word;',
  '    procedure SetLeft(Value: word);',
  '    property Size: word read FSize write FSize;',
  '    property Speed: word read GetSpeed write SetSpeed;',
  '    property Left: word read GetLeft write SetLeft;',
  '  end;',
  '  TBird = class',
  '    property NotRight: word read GetLeft write SetLeft;',
  '    procedure DoIt;',
  '  end;',
  'var',
  '  b: TBird;',
  'function Tobject.GetSpeed: word;',
  'begin',
  '  Size:=Size+11;',
  '  Speed:=Speed+12;',
  '  Result:=Left+13;',
  '  Left:=13;',
  '  Left:=Left+13;',
  '  Self.Size:=Self.Size+21;',
  '  Self.Speed:=Self.Speed+22;',
  '  Self.Left:=Self.Left+23;',
  '  with Self do begin',
  '    Size:=Size+31;',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '  end;',
  'end;',
  'procedure Tobject.SetSpeed(Value: word);',
  'begin',
  'end;',
  'function TObjHelper.GetLeft: word;',
  'begin',
  '  Size:=Size+11;',
  '  Speed:=Speed+12;',
  '  Left:=Left+13;',
  '  Self.Size:=Self.Size+21;',
  '  Self.Speed:=Self.Speed+22;',
  '  Self.Left:=Self.Left+23;',
  '  with Self do begin',
  '    Size:=Size+31;',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '  end;',
  'end;',
  'procedure TObjHelper.SetLeft(Value: word);',
  'begin',
  'end;',
  'procedure TBird.DoIt;',
  'begin',
  '  NotRight:=NotRight+11;',
  '  Self.NotRight:=Self.NotRight+21;',
  '  with Self do begin',
  '    NotRight:=NotRight+31;',
  '  end;',
  'end;',
  'begin',
  '  b.Size:=b.Size+11;',
  '  b.Speed:=b.Speed+12;',
  '  b.Left:=b.Left+13;',
  '  b.NotRight:=b.NotRight+14;',
  '  with b do begin',
  '    Size:=Size+31;',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '    NotRight:=NotRight+34;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_Property',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FSize = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetSpeed = function () {',
    '    var Result = 0;',
    '    this.FSize = this.FSize + 11;',
    '    this.SetSpeed(this.GetSpeed() + 12);',
    '    Result = $mod.TObjHelper.GetLeft.call(this) + 13;',
    '    $mod.TObjHelper.SetLeft.call(this, 13);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 13);',
    '    this.FSize = this.FSize + 21;',
    '    this.SetSpeed(this.GetSpeed() + 22);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 23);',
    '    this.FSize = this.FSize + 31;',
    '    this.SetSpeed(this.GetSpeed() + 32);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 33);',
    '    return Result;',
    '  };',
    '  this.SetSpeed = function (Value) {',
    '  };',
    '});',
    'rtl.createHelper(this, "TObjHelper", null, function () {',
    '  this.GetLeft = function () {',
    '    var Result = 0;',
    '    this.FSize = this.FSize + 11;',
    '    this.SetSpeed(this.GetSpeed() + 12);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 13);',
    '    this.FSize = this.FSize + 21;',
    '    this.SetSpeed(this.GetSpeed() + 22);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 23);',
    '    this.FSize = this.FSize + 31;',
    '    this.SetSpeed(this.GetSpeed() + 32);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 33);',
    '    return Result;',
    '  };',
    '  this.SetLeft = function (Value) {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt = function () {',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 11);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 21);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 31);',
    '  };',
    '});',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.b.FSize = $mod.b.FSize + 11;',
    '$mod.b.SetSpeed($mod.b.GetSpeed() + 12);',
    '$mod.TObjHelper.SetLeft.call($mod.b, $mod.TObjHelper.GetLeft.call($mod.b) + 13);',
    '$mod.TObjHelper.SetLeft.call($mod.b, $mod.TObjHelper.GetLeft.call($mod.b) + 14);',
    'var $with = $mod.b;',
    '$with.FSize = $with.FSize + 31;',
    '$with.SetSpeed($with.GetSpeed() + 32);',
    '$mod.TObjHelper.SetLeft.call($with, $mod.TObjHelper.GetLeft.call($with) + 33);',
    '$mod.TObjHelper.SetLeft.call($with, $mod.TObjHelper.GetLeft.call($with) + 34);',
    '']));
end;

procedure TTestModule.TestClassHelper_Property_Array;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function GetSpeed(Index: boolean): word;',
  '    procedure SetSpeed(Index: boolean; Value: word);',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    function GetSize(Index: boolean): word;',
  '    procedure SetSize(Index: boolean; Value: word);',
  '    property Size[Index: boolean]: word read GetSize write SetSize;',
  '    property Speed[Index: boolean]: word read GetSpeed write SetSpeed;',
  '  end;',
  '  TBird = class',
  '    property Items[Index: boolean]: word read GetSize write SetSize;',
  '    procedure DoIt;',
  '  end;',
  'var',
  '  b: TBird;',
  'function Tobject.GetSpeed(Index: boolean): word;',
  'begin',
  '  Result:=Size[false];',
  '  Size[true]:=Size[false]+11;',
  '  Speed[true]:=Speed[false]+12;',
  '  Self.Size[true]:=Self.Size[false]+21;',
  '  Self.Speed[true]:=Self.Speed[false]+22;',
  '  with Self do begin',
  '    Size[true]:=Size[false]+31;',
  '    Speed[true]:=Speed[false]+32;',
  '  end;',
  'end;',
  'procedure Tobject.SetSpeed(Index: boolean; Value: word);',
  'begin',
  'end;',
  'function TObjHelper.GetSize(Index: boolean): word;',
  'begin',
  '  Size[true]:=Size[false]+11;',
  '  Speed[true]:=Speed[false]+12;',
  '  Self.Size[true]:=Self.Size[false]+21;',
  '  Self.Speed[true]:=Self.Speed[false]+22;',
  '  with Self do begin',
  '    Size[true]:=Size[false]+31;',
  '    Speed[true]:=Speed[false]+32;',
  '  end;',
  'end;',
  'procedure TObjHelper.SetSize(Index: boolean; Value: word);',
  'begin',
  'end;',
  'procedure TBird.DoIt;',
  'begin',
  '  Items[true]:=Items[false]+11;',
  '  Self.Items[true]:=Self.Items[false]+21;',
  '  with Self do Items[true]:=Items[false]+31;',
  'end;',
  'begin',
  '  b.Size[true]:=b.Size[false]+11;',
  '  b.Speed[true]:=b.Speed[false]+12;',
  '  b.Items[true]:=b.Items[false]+13;',
  '  with b do begin',
  '    Size[true]:=Size[false]+21;',
  '    Speed[true]:=Speed[false]+22;',
  '    Items[true]:=Items[false]+23;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_Property_Array',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetSpeed = function (Index) {',
    '    var Result = 0;',
    '    Result = $mod.TObjHelper.GetSize.call(this, false);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 11);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 12);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 21);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 22);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 31);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 32);',
    '    return Result;',
    '  };',
    '  this.SetSpeed = function (Index, Value) {',
    '  };',
    '});',
    'rtl.createHelper(this, "TObjHelper", null, function () {',
    '  this.GetSize = function (Index) {',
    '    var Result = 0;',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 11);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 12);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 21);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 22);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 31);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 32);',
    '    return Result;',
    '  };',
    '  this.SetSize = function (Index, Value) {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt = function () {',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 11);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 21);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 31);',
    '  };',
    '});',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TObjHelper.SetSize.call($mod.b, true, $mod.TObjHelper.GetSize.call($mod.b, false) + 11);',
    '$mod.b.SetSpeed(true, $mod.b.GetSpeed(false) + 12);',
    '$mod.TObjHelper.SetSize.call($mod.b, true, $mod.TObjHelper.GetSize.call($mod.b, false) + 13);',
    'var $with = $mod.b;',
    '$mod.TObjHelper.SetSize.call($with, true, $mod.TObjHelper.GetSize.call($with, false) + 21);',
    '$with.SetSpeed(true, $with.GetSpeed(false) + 22);',
    '$mod.TObjHelper.SetSize.call($with, true, $mod.TObjHelper.GetSize.call($with, false) + 23);',
    '']));
end;

procedure TTestModule.TestClassHelper_Property_Array_Default;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    function GetSpeed(Index: boolean): word;',
  '    procedure SetSpeed(Index: boolean; Value: word);',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    property Speed[Index: boolean]: word read GetSpeed write SetSpeed; default;',
  '  end;',
  '  TBird = class',
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '    function GetSize(Index: word): boolean;',
  '    procedure SetSize(Index: word; Value: boolean);',
  '    property Size[Index: word]: boolean read GetSize write SetSize; default;',
  '  end;',
  'function Tobject.GetSpeed(Index: boolean): word;',
  'begin',
  '  Self[true]:=Self[false]+1;',
  'end;',
  'procedure Tobject.SetSpeed(Index: boolean; Value: word);',
  'begin',
  'end;',
  'function TBirdHelper.GetSize(Index: word): boolean;',
  'begin',
  '  Self[1]:=not Self[2];',
  'end;',
  'procedure TBirdHelper.SetSize(Index: word; Value: boolean);',
  'begin',
  'end;',
  'var',
  '  o: TObject;',
  '  b: TBird;',
  'begin',
  '  o[true]:=o[false]+1;',
  '  b[3]:=not b[4];',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_Property_Array_Default',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetSpeed = function (Index) {',
    '    var Result = 0;',
    '    this.SetSpeed(true, this.GetSpeed(false) + 1);',
    '    return Result;',
    '  };',
    '  this.SetSpeed = function (Index, Value) {',
    '  };',
    '});',
    'rtl.createHelper(this, "TObjHelper", null, function () {',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '});',
    'rtl.createHelper(this, "TBirdHelper", null, function () {',
    '  this.GetSize = function (Index) {',
    '    var Result = false;',
    '    $mod.TBirdHelper.SetSize.call(this, 1, !$mod.TBirdHelper.GetSize.call(this, 2));',
    '    return Result;',
    '  };',
    '  this.SetSize = function (Index, Value) {',
    '  };',
    '});',
    'this.o = null;',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.o.SetSpeed(true, $mod.o.GetSpeed(false) + 1);',
    '$mod.TBirdHelper.SetSize.call($mod.b, 3, !$mod.TBirdHelper.GetSize.call($mod.b, 4));',
    '']));
end;

procedure TTestModule.TestClassHelper_Property_Array_DefaultDefault;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    function GetItems(Index: word): TObject;',
  '    procedure SetItems(Index: word; Value: TObject);',
  '    property Items[Index: word]: TObject read GetItems write SetItems; default;',
  '  end;',
  'function Tobjhelper.GetItems(Index: word): TObject;',
  'begin',
  '  Self[1][2]:=Self[3][4];',
  'end;',
  'procedure Tobjhelper.SetItems(Index: word; Value: TObject);',
  'begin',
  'end;',
  'var',
  '  o: TObject;',
  'begin',
  '  o[1][2]:=o[3][4];',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_Property_Array_DefaultDefault',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createHelper(this, "TObjHelper", null, function () {',
    '  this.GetItems = function (Index) {',
    '    var Result = null;',
    '    $mod.TObjHelper.SetItems.call($mod.TObjHelper.GetItems.call(this, 1), 2, $mod.TObjHelper.GetItems.call($mod.TObjHelper.GetItems.call(this, 3), 4));',
    '    return Result;',
    '  };',
    '  this.SetItems = function (Index, Value) {',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TObjHelper.SetItems.call($mod.TObjHelper.GetItems.call($mod.o, 1), 2, $mod.TObjHelper.GetItems.call($mod.TObjHelper.GetItems.call($mod.o, 3), 4));',
    '']));
end;

procedure TTestModule.TestClassHelper_ClassProperty;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class var FSize: word;',
  '    class function GetSpeed: word;',
  '    class procedure SetSpeed(Value: word); virtual; abstract;',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    class function GetLeft: word;',
  '    class procedure SetLeft(Value: word);',
  '    class property Size: word read FSize write FSize;',
  '    class property Speed: word read GetSpeed write SetSpeed;',
  '    class property Left: word read GetLeft write SetLeft;',
  '  end;',
  '  TBird = class',
  '    class property NotRight: word read GetLeft write SetLeft;',
  '    class procedure DoIt;',
  '  end;',
  '  TBirdClass = class of TBird;',
  'class function Tobject.GetSpeed: word;',
  'begin',
  '  Size:=Size+11;',
  '  Speed:=Speed+12;',
  '  Left:=Left+13;',
  '  Self.Size:=Self.Size+21;',
  '  Self.Speed:=Self.Speed+22;',
  '  Self.Left:=Self.Left+23;',
  '  with Self do begin',
  '    Size:=Size+31;',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '  end;',
  'end;',
  'class function TObjHelper.GetLeft: word;',
  'begin',
  '  Size:=Size+11;',
  '  Speed:=Speed+12;',
  '  Left:=Left+13;',
  '  Self.Size:=Self.Size+21;',
  '  Self.Speed:=Self.Speed+22;',
  '  Self.Left:=Self.Left+23;',
  '  with Self do begin',
  '    Size:=Size+31;',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '  end;',
  'end;',
  'class procedure TObjHelper.SetLeft(Value: word);',
  'begin',
  'end;',
  'class procedure TBird.DoIt;',
  'begin',
  '  NotRight:=NotRight+11;',
  '  Self.NotRight:=Self.NotRight+21;',
  '  with Self do NotRight:=NotRight+31;',
  'end;',
  'var',
  '  b: TBird;',
  '  c: TBirdClass;',
  'begin',
  '  b.Size:=b.Size+11;',
  '  b.Speed:=b.Speed+12;',
  '  b.Left:=b.Left+13;',
  '  b.NotRight:=b.NotRight+14;',
  '  with b do begin',
  '    Size:=Size+31;',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '    NotRight:=NotRight+34;',
  '  end;',
  '  c.Size:=c.Size+11;',
  '  c.Speed:=c.Speed+12;',
  '  c.Left:=c.Left+13;',
  '  c.NotRight:=c.NotRight+14;',
  '  with c do begin',
  '    Size:=Size+31;',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '    NotRight:=NotRight+34;',
  '  end;',
  '  tbird.Size:=tbird.Size+11;',
  '  tbird.Speed:=tbird.Speed+12;',
  '  tbird.Left:=tbird.Left+13;',
  '  tbird.NotRight:=tbird.NotRight+14;',
  '  with tbird do begin',
  '    Size:=Size+31;',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '    NotRight:=NotRight+34;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_ClassProperty',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.FSize = 0;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetSpeed = function () {',
    '    var Result = 0;',
    '    $mod.TObject.FSize = this.FSize + 11;',
    '    this.SetSpeed(this.GetSpeed() + 12);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 13);',
    '    $mod.TObject.FSize = this.FSize + 21;',
    '    this.SetSpeed(this.GetSpeed() + 22);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 23);',
    '    $mod.TObject.FSize = this.FSize + 31;',
    '    this.SetSpeed(this.GetSpeed() + 32);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 33);',
    '    return Result;',
    '  };',
    '});',
    'rtl.createHelper(this, "TObjHelper", null, function () {',
    '  this.GetLeft = function () {',
    '    var Result = 0;',
    '    $mod.TObject.FSize = this.FSize + 11;',
    '    this.SetSpeed(this.GetSpeed() + 12);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 13);',
    '    $mod.TObject.FSize = this.FSize + 21;',
    '    this.SetSpeed(this.GetSpeed() + 22);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 23);',
    '    $mod.TObject.FSize = this.FSize + 31;',
    '    this.SetSpeed(this.GetSpeed() + 32);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 33);',
    '    return Result;',
    '  };',
    '  this.SetLeft = function (Value) {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt = function () {',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 11);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 21);',
    '    $mod.TObjHelper.SetLeft.call(this, $mod.TObjHelper.GetLeft.call(this) + 31);',
    '  };',
    '});',
    'this.b = null;',
    'this.c = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TObject.FSize = $mod.b.FSize + 11;',
    '$mod.b.$class.SetSpeed($mod.b.$class.GetSpeed() + 12);',
    '$mod.TObjHelper.SetLeft.call($mod.b.$class, $mod.TObjHelper.GetLeft.call($mod.b.$class) + 13);',
    '$mod.TObjHelper.SetLeft.call($mod.b.$class, $mod.TObjHelper.GetLeft.call($mod.b.$class) + 14);',
    'var $with = $mod.b;',
    '$mod.TObject.FSize = $with.FSize + 31;',
    '$with.$class.SetSpeed($with.$class.GetSpeed() + 32);',
    '$mod.TObjHelper.SetLeft.call($with.$class, $mod.TObjHelper.GetLeft.call($with.$class) + 33);',
    '$mod.TObjHelper.SetLeft.call($with.$class, $mod.TObjHelper.GetLeft.call($with.$class) + 34);',
    '$mod.TObject.FSize = $mod.c.FSize + 11;',
    '$mod.c.SetSpeed($mod.c.GetSpeed() + 12);',
    '$mod.TObjHelper.SetLeft.call($mod.c, $mod.TObjHelper.GetLeft.call($mod.c) + 13);',
    '$mod.TObjHelper.SetLeft.call($mod.c, $mod.TObjHelper.GetLeft.call($mod.c) + 14);',
    'var $with1 = $mod.c;',
    '$mod.TObject.FSize = $with1.FSize + 31;',
    '$with1.SetSpeed($with1.GetSpeed() + 32);',
    '$mod.TObjHelper.SetLeft.call($with1, $mod.TObjHelper.GetLeft.call($with1) + 33);',
    '$mod.TObjHelper.SetLeft.call($with1, $mod.TObjHelper.GetLeft.call($with1) + 34);',
    '$mod.TObject.FSize = $mod.TBird.FSize + 11;',
    '$mod.TBird.SetSpeed($mod.TBird.GetSpeed() + 12);',
    '$mod.TObjHelper.SetLeft.call($mod.TBird, $mod.TObjHelper.GetLeft.call($mod.TBird) + 13);',
    '$mod.TObjHelper.SetLeft.call($mod.TBird, $mod.TObjHelper.GetLeft.call($mod.TBird) + 14);',
    'var $with2 = $mod.TBird;',
    '$mod.TObject.FSize = $with2.FSize + 31;',
    '$with2.SetSpeed($with2.GetSpeed() + 32);',
    '$mod.TObjHelper.SetLeft.call($mod.TBird, $mod.TObjHelper.GetLeft.call($mod.TBird) + 33);',
    '$mod.TObjHelper.SetLeft.call($mod.TBird, $mod.TObjHelper.GetLeft.call($mod.TBird) + 34);',
    '']));
end;

procedure TTestModule.TestClassHelper_ClassPropertyStatic;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class function GetSpeed: word; static;',
  '    class procedure SetSpeed(Value: word); static;',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    class function GetLeft: word; static;',
  '    class procedure SetLeft(Value: word); static;',
  '    class property Speed: word read GetSpeed write SetSpeed;',
  '    class property Left: word read GetLeft write SetLeft;',
  '  end;',
  '  TBird = class',
  '    class property NotRight: word read GetLeft write SetLeft;',
  '    class procedure DoIt; static;',
  '    class procedure DoSome;',
  '  end;',
  '  TBirdClass = class of TBird;',
  'class function Tobject.GetSpeed: word;',
  'begin',
  '  Speed:=Speed+12;',
  '  Left:=Left+13;',
  'end;',
  'class procedure TObject.SetSpeed(Value: word);',
  'begin',
  'end;',
  'class function TObjHelper.GetLeft: word;',
  'begin',
  '  Speed:=Speed+12;',
  '  Left:=Left+13;',
  'end;',
  'class procedure TObjHelper.SetLeft(Value: word);',
  'begin',
  'end;',
  'class procedure TBird.DoIt;',
  'begin',
  '  NotRight:=NotRight+11;',
  'end;',
  'class procedure TBird.DoSome;',
  'begin',
  '  Speed:=Speed+12;',
  '  Left:=Left+13;',
  '  Self.Speed:=Self.Speed+22;',
  '  Self.Left:=Self.Left+23;',
  '  with Self do begin',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '  end;',
  '  NotRight:=NotRight+11;',
  '  Self.NotRight:=Self.NotRight+21;',
  '  with Self do NotRight:=NotRight+31;',
  'end;',
  'var',
  '  b: TBird;',
  '  c: TBirdClass;',
  'begin',
  '  b.Speed:=b.Speed+12;',
  '  b.Left:=b.Left+13;',
  '  b.NotRight:=b.NotRight+14;',
  '  with b do begin',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '    NotRight:=NotRight+34;',
  '  end;',
  '  c.Speed:=c.Speed+12;',
  '  c.Left:=c.Left+13;',
  '  c.NotRight:=c.NotRight+14;',
  '  with c do begin',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '    NotRight:=NotRight+34;',
  '  end;',
  '  tbird.Speed:=tbird.Speed+12;',
  '  tbird.Left:=tbird.Left+13;',
  '  tbird.NotRight:=tbird.NotRight+14;',
  '  with tbird do begin',
  '    Speed:=Speed+32;',
  '    Left:=Left+33;',
  '    NotRight:=NotRight+34;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_ClassPropertyStatic',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetSpeed = function () {',
    '    var Result = 0;',
    '    $mod.TObject.SetSpeed($mod.TObject.GetSpeed() + 12);',
    '    $mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 13);',
    '    return Result;',
    '  };',
    '  this.SetSpeed = function (Value) {',
    '  };',
    '});',
    'rtl.createHelper(this, "TObjHelper", null, function () {',
    '  this.GetLeft = function () {',
    '    var Result = 0;',
    '    $mod.TObject.SetSpeed($mod.TObject.GetSpeed() + 12);',
    '    $mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 13);',
    '    return Result;',
    '  };',
    '  this.SetLeft = function (Value) {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt = function () {',
    '    $mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 11);',
    '  };',
    '  this.DoSome = function () {',
    '    this.SetSpeed(this.GetSpeed() + 12);',
    '    $mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 13);',
    '    this.SetSpeed(this.GetSpeed() + 22);',
    '    $mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 23);',
    '    this.SetSpeed(this.GetSpeed() + 32);',
    '    $mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 33);',
    '    $mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 11);',
    '    $mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 21);',
    '    $mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 31);',
    '  };',
    '});',
    'this.b = null;',
    'this.c = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TObject.SetSpeed($mod.TObject.GetSpeed() + 12);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 13);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 14);',
    'var $with = $mod.b;',
    '$with.SetSpeed($with.GetSpeed() + 32);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 33);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 34);',
    '$mod.TObject.SetSpeed($mod.TObject.GetSpeed() + 12);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 13);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 14);',
    'var $with1 = $mod.c;',
    '$with1.SetSpeed($with1.GetSpeed() + 32);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 33);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 34);',
    '$mod.TObject.SetSpeed($mod.TObject.GetSpeed() + 12);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 13);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 14);',
    'var $with2 = $mod.TBird;',
    '$with2.SetSpeed($with2.GetSpeed() + 32);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 33);',
    '$mod.TObjHelper.SetLeft($mod.TObjHelper.GetLeft() + 34);',
    '']));
end;

procedure TTestModule.TestClassHelper_ClassProperty_Array;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    class function GetSpeed(Index: boolean): word;',
  '    class procedure SetSpeed(Index: boolean; Value: word); virtual; abstract;',
  '  end;',
  '  TObjHelper = class helper for TObject',
  '    class function GetSize(Index: boolean): word;',
  '    class procedure SetSize(Index: boolean; Value: word);',
  '    class property Size[Index: boolean]: word read GetSize write SetSize;',
  '    class property Speed[Index: boolean]: word read GetSpeed write SetSpeed;',
  '  end;',
  '  TBird = class',
  '    class property Items[Index: boolean]: word read GetSize write SetSize;',
  '    class procedure DoIt;',
  '  end;',
  '  TBirdClass = class of TBird;',
  'class function Tobject.GetSpeed(Index: boolean): word;',
  'begin',
  '  Size[true]:=Size[false]+11;',
  '  Speed[true]:=Speed[false]+12;',
  '  Self.Size[true]:=Self.Size[false]+21;',
  '  Self.Speed[true]:=Self.Speed[false]+22;',
  '  with Self do begin',
  '    Size[true]:=Size[false]+31;',
  '    Speed[true]:=Speed[false]+32;',
  '  end;',
  'end;',
  'class function TObjHelper.GetSize(Index: boolean): word;',
  'begin',
  '  Size[true]:=Size[false]+11;',
  '  Speed[true]:=Speed[false]+12;',
  '  Self.Size[true]:=Self.Size[false]+21;',
  '  Self.Speed[true]:=Self.Speed[false]+22;',
  '  with Self do begin',
  '    Size[true]:=Size[false]+31;',
  '    Speed[true]:=Speed[false]+32;',
  '  end;',
  'end;',
  'class procedure TObjHelper.SetSize(Index: boolean; Value: word);',
  'begin',
  'end;',
  'class procedure TBird.DoIt;',
  'begin',
  '  Items[true]:=Items[false]+11;',
  '  Self.Items[true]:=Self.Items[false]+21;',
  '  with Self do Items[true]:=Items[false]+31;',
  'end;',
  'var',
  '  b: TBird;',
  '  c: TBirdClass;',
  'begin',
  '  b.Size[true]:=b.Size[false]+11;',
  '  b.Speed[true]:=b.Speed[false]+12;',
  '  b.Items[true]:=b.Items[false]+13;',
  '  with b do begin',
  '    Size[true]:=Size[false]+21;',
  '    Speed[true]:=Speed[false]+22;',
  '    Items[true]:=Items[false]+23;',
  '  end;',
  '  c.Size[true]:=c.Size[false]+11;',
  '  c.Speed[true]:=c.Speed[false]+12;',
  '  c.Items[true]:=c.Items[false]+13;',
  '  with c do begin',
  '    Size[true]:=Size[false]+21;',
  '    Speed[true]:=Speed[false]+22;',
  '    Items[true]:=Items[false]+23;',
  '  end;',
  '  TBird.Size[true]:=TBird.Size[false]+11;',
  '  TBird.Speed[true]:=TBird.Speed[false]+12;',
  '  TBird.Items[true]:=TBird.Items[false]+13;',
  '  with TBird do begin',
  '    Size[true]:=Size[false]+21;',
  '    Speed[true]:=Speed[false]+22;',
  '    Items[true]:=Items[false]+23;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_ClassProperty_Array',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetSpeed = function (Index) {',
    '    var Result = 0;',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 11);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 12);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 21);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 22);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 31);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 32);',
    '    return Result;',
    '  };',
    '});',
    'rtl.createHelper(this, "TObjHelper", null, function () {',
    '  this.GetSize = function (Index) {',
    '    var Result = 0;',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 11);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 12);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 21);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 22);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 31);',
    '    this.SetSpeed(true, this.GetSpeed(false) + 32);',
    '    return Result;',
    '  };',
    '  this.SetSize = function (Index, Value) {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.DoIt = function () {',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 11);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 21);',
    '    $mod.TObjHelper.SetSize.call(this, true, $mod.TObjHelper.GetSize.call(this, false) + 31);',
    '  };',
    '});',
    'this.b = null;',
    'this.c = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TObjHelper.SetSize.call($mod.b.$class, true, $mod.TObjHelper.GetSize.call($mod.b.$class, false) + 11);',
    '$mod.b.$class.SetSpeed(true, $mod.b.$class.GetSpeed(false) + 12);',
    '$mod.TObjHelper.SetSize.call($mod.b.$class, true, $mod.TObjHelper.GetSize.call($mod.b.$class, false) + 13);',
    'var $with = $mod.b;',
    '$mod.TObjHelper.SetSize.call($with.$class, true, $mod.TObjHelper.GetSize.call($with.$class, false) + 21);',
    '$with.$class.SetSpeed(true, $with.$class.GetSpeed(false) + 22);',
    '$mod.TObjHelper.SetSize.call($with.$class, true, $mod.TObjHelper.GetSize.call($with.$class, false) + 23);',
    '$mod.TObjHelper.SetSize.call($mod.c, true, $mod.TObjHelper.GetSize.call($mod.c, false) + 11);',
    '$mod.c.SetSpeed(true, $mod.c.GetSpeed(false) + 12);',
    '$mod.TObjHelper.SetSize.call($mod.c, true, $mod.TObjHelper.GetSize.call($mod.c, false) + 13);',
    'var $with1 = $mod.c;',
    '$mod.TObjHelper.SetSize.call($with1, true, $mod.TObjHelper.GetSize.call($with1, false) + 21);',
    '$with1.SetSpeed(true, $with1.GetSpeed(false) + 22);',
    '$mod.TObjHelper.SetSize.call($with1, true, $mod.TObjHelper.GetSize.call($with1, false) + 23);',
    '$mod.TObjHelper.SetSize.call($mod.TBird, true, $mod.TObjHelper.GetSize.call($mod.TBird, false) + 11);',
    '$mod.TBird.SetSpeed(true, $mod.TBird.GetSpeed(false) + 12);',
    '$mod.TObjHelper.SetSize.call($mod.TBird, true, $mod.TObjHelper.GetSize.call($mod.TBird, false) + 13);',
    'var $with2 = $mod.TBird;',
    '$mod.TObjHelper.SetSize.call($mod.TBird, true, $mod.TObjHelper.GetSize.call($mod.TBird, false) + 21);',
    '$with2.SetSpeed(true, $with2.GetSpeed(false) + 22);',
    '$mod.TObjHelper.SetSize.call($mod.TBird, true, $mod.TObjHelper.GetSize.call($mod.TBird, false) + 23);',
    '']));
end;

procedure TTestModule.TestClassHelper_ForIn;
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
  '  end;',
  '  TBirdHelper = class helper for TBird',
  '    function GetEnumerator: TEnumerator;',
  '  end;',
  'function TEnumerator.MoveNext: boolean;',
  'begin',
  'end;',
  'function TBirdHelper.GetEnumerator: TEnumerator;',
  'begin',
  'end;',
  'var',
  '  b: TBird;',
  '  i, i2: TItem;',
  'begin',
  '  for i in b do i2:=i;']);
  ConvertProgram;
  CheckSource('TestClassHelper_ForIn',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TEnumerator", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.FCurrent = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FCurrent = undefined;',
    '    $mod.TObject.$final.call(this);',
    '  };',
    '  this.MoveNext = function () {',
    '    var Result = false;',
    '    return Result;',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '});',
    'rtl.createHelper(this, "TBirdHelper", null, function () {',
    '  this.GetEnumerator = function () {',
    '    var Result = null;',
    '    return Result;',
    '  };',
    '});',
    'this.b = null;',
    'this.i = null;',
    'this.i2 = null;'
    ]),
    LinesToStr([ // $mod.$main
    'var $in = $mod.TBirdHelper.GetEnumerator.call($mod.b);',
    'try {',
    '  while ($in.MoveNext()){',
    '    $mod.i = $in.FCurrent;',
    '    $mod.i2 = $mod.i;',
    '  }',
    '} finally {',
    '  $in = rtl.freeLoc($in)',
    '};',
    '']));
end;

procedure TTestModule.TestClassHelper_PassProperty;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    FField: TObject;',
  '    property Field: TObject read FField write FField;',
  '  end;',
  '  THelper = class helper for TObject',
  '    procedure Fly;',
  '    class procedure Run;',
  '    class procedure Jump; static;',
  '  end;',
  'procedure THelper.Fly;',
  'begin',
  '  Field.Fly;',
  '  Field.Run;',
  '  Field.Jump;',
  '  with Field do begin',
  '    Fly;',
  '    Run;',
  '    Jump;',
  '  end;',
  'end;',
  'class procedure THelper.Run;',
  'begin',
  'end;',
  'class procedure THelper.Jump;',
  'begin',
  'end;',
  'var',
  '  b: TObject;',
  'begin',
  '  b.Field.Fly;',
  '  b.Field.Run;',
  '  b.Field.Jump;',
  '  with b do begin',
  '    Field.Run;',
  '    Field.Fly;',
  '    Field.Jump;',
  '  end;',
  '  with b.Field do begin',
  '    Run;',
  '    Fly;',
  '    Jump;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestClassHelper_PassProperty',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FField = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FField = undefined;',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Fly = function () {',
    '    $mod.THelper.Fly.call(this.FField);',
    '    $mod.THelper.Run.call(this.FField.$class);',
    '    $mod.THelper.Jump();',
    '    var $with = this.FField;',
    '    $mod.THelper.Fly.call($with);',
    '    $mod.THelper.Run.call($with.$class);',
    '    $mod.THelper.Jump();',
    '  };',
    '  this.Run = function () {',
    '  };',
    '  this.Jump = function () {',
    '  };',
    '});',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Fly.call($mod.b.FField);',
    '$mod.THelper.Run.call($mod.b.FField.$class);',
    '$mod.THelper.Jump();',
    'var $with = $mod.b;',
    '$mod.THelper.Run.call($with.FField.$class);',
    '$mod.THelper.Fly.call($with.FField);',
    '$mod.THelper.Jump();',
    'var $with1 = $mod.b.FField;',
    '$mod.THelper.Run.call($with1.$class);',
    '$mod.THelper.Fly.call($with1);',
    '$mod.THelper.Jump();',
    '']));
end;

procedure TTestModule.TestExtClassHelper_ClassVar;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtObj''',
  '  end;',
  '  THelper = class helper for TExtA',
  '    const',
  '      One = 1;',
  '      Two: word = 2;',
  '    class var',
  '      Glob: word;',
  '    function Foo(w: word): word;',
  '    class function Bar(w: word): word; static;',
  '  end;',
  'function THelper.foo(w: word): word;',
  'begin',
  '  Result:=w;',
  '  Two:=One+w;',
  '  Glob:=Glob;',
  '  Result:=Self.Glob;',
  '  Self.Glob:=Self.Glob;',
  '  with Self do Glob:=Glob;',
  'end;',
  'class function THelper.bar(w: word): word;',
  'begin',
  '  Result:=w;',
  '  Two:=One;',
  '  Glob:=Glob;',
  'end;',
  'var o: TExtA;',
  'begin',
  '  texta.two:=texta.one;',
  '  texta.Glob:=texta.Glob;',
  '  with texta do begin',
  '    two:=one;',
  '    Glob:=Glob;',
  '  end;',
  '  o.two:=o.one;',
  '  o.Glob:=o.Glob;',
  '  with o do begin',
  '    two:=one;',
  '    Glob:=Glob;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestExtClassHelper_ClassVar',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.One = 1;',
    '  this.Two = 2;',
    '  this.Glob = 0;',
    '  this.Foo = function (w) {',
    '    var Result = 0;',
    '    Result = w;',
    '    $mod.THelper.Two = 1 + w;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    Result = $mod.THelper.Glob;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    return Result;',
    '  };',
    '  this.Bar = function (w) {',
    '    var Result = 0;',
    '    Result = w;',
    '    $mod.THelper.Two = 1;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    return Result;',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    'var $with = $mod.o;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    '']));
end;

procedure TTestModule.TestExtClassHelper_Method_Call;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TFly = function(w: word): word of object;',
  '  TExtA = class external name ''ExtObj''',
  '    procedure Run(w: word = 10);',
  '  end;',
  '  THelper = class helper for TExtA',
  '    function Foo(w: word = 1): word;',
  '    function Fly(w: word = 2): word; external name ''Fly'';',
  '  end;',
  'var p: TFly;',
  'function THelper.foo(w: word): word;',
  'begin',
  '  Run;',
  '  Run();',
  '  Run(11);',
  '  Foo;',
  '  Foo();',
  '  Foo(12);',
  '  Self.Foo;',
  '  Self.Foo();',
  '  Self.Foo(13);',
  '  Fly;',
  '  Fly();',
  '  with Self do begin',
  '    Foo;',
  '    Foo();',
  '    Foo(14);',
  '    Fly;',
  '    Fly();',
  '  end;',
  '  p:=@Fly;',
  'end;',
  'var Obj: TExtA;',
  'begin',
  '  obj.Foo;',
  '  obj.Foo();',
  '  obj.Foo(21);',
  '  obj.Fly;',
  '  obj.Fly();',
  '  with obj do begin',
  '    Foo;',
  '    Foo();',
  '    Foo(22);',
  '    Fly;',
  '    Fly();',
  '  end;',
  '  p:=@obj.Fly;',
  '']);
  ConvertProgram;
  CheckSource('TestExtClassHelper_Method_Call',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Foo = function (w) {',
    '    var Result = 0;',
    '    this.Run(10);',
    '    this.Run(10);',
    '    this.Run(11);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 12);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 13);',
    '    this.Fly(2);',
    '    this.Fly(2);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 14);',
    '    this.Fly(2);',
    '    this.Fly(2);',
    '    $mod.p = rtl.createCallback(this, "Fly");',
    '    return Result;',
    '  };',
    '});',
    'this.p = null;',
    'this.Obj = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Foo.call($mod.Obj, 1);',
    '$mod.THelper.Foo.call($mod.Obj, 1);',
    '$mod.THelper.Foo.call($mod.Obj, 21);',
    '$mod.Obj.Fly(2);',
    '$mod.Obj.Fly(2);',
    'var $with = $mod.Obj;',
    '$mod.THelper.Foo.call($with, 1);',
    '$mod.THelper.Foo.call($with, 1);',
    '$mod.THelper.Foo.call($with, 22);',
    '$with.Fly(2);',
    '$with.Fly(2);',
    '$mod.p = rtl.createCallback($mod.Obj, "Fly");',
    '']));
end;

procedure TTestModule.TestExtClassHelper_ClassMethod_MissingStatic;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TExtA = class external name ''ExtObj''',
  '    procedure Run(w: word = 10);',
  '  end;',
  '  THelper = class helper for TExtA',
  '    class procedure Fly;',
  '  end;',
  'class procedure THelper.Fly;',
  'begin end;',
  'begin',
  '']);
  SetExpectedPasResolverError(sHelperClassMethodForExtClassMustBeStatic,
                              nHelperClassMethodForExtClassMustBeStatic);
  ConvertProgram;
end;

procedure TTestModule.TestRecordHelper_ClassVar;
begin
  StartProgram(false);
  Add([
  'type',
  '  TRec = record',
  '  end;',
  '  THelper = record helper for TRec',
  '    const',
  '      One = 1;',
  '      Two: word = 2;',
  '    class var',
  '      Glob: word;',
  '    function Foo(w: word): word;',
  '    class function Bar(w: word): word; static;',
  '  end;',
  'function THelper.foo(w: word): word;',
  'begin',
  '  Result:=w;',
  '  Two:=One+w;',
  '  Glob:=Glob;',
  '  Result:=Self.Glob;',
  '  Self.Glob:=Self.Glob;',
  '  with Self do Glob:=Glob;',
  '  Self:=Self;',
  'end;',
  'class function THelper.bar(w: word): word;',
  'begin',
  '  Result:=w;',
  '  Two:=One;',
  '  Glob:=Glob;',
  'end;',
  'var r: TRec;',
  'begin',
  '  trec.two:=trec.one;',
  '  trec.Glob:=trec.Glob;',
  '  with trec do begin',
  '    two:=one;',
  '    Glob:=Glob;',
  '  end;',
  '  r.two:=r.one;',
  '  r.Glob:=r.Glob;',
  '  with r do begin',
  '    two:=one;',
  '    Glob:=Glob;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestRecordHelper_ClassVar',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.One = 1;',
    '  this.Two = 2;',
    '  this.Glob = 0;',
    '  this.Foo = function (w) {',
    '    var Result = 0;',
    '    Result = w;',
    '    $mod.THelper.Two = 1 + w;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    Result = $mod.THelper.Glob;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    this.$assign(this);',
    '    return Result;',
    '  };',
    '  this.Bar = function (w) {',
    '    var Result = 0;',
    '    Result = w;',
    '    $mod.THelper.Two = 1;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    return Result;',
    '  };',
    '});',
    'this.r = this.TRec.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    'var $with = $mod.TRec;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    'var $with1 = $mod.r;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    '']));
end;

procedure TTestModule.TestRecordHelper_Method_Call;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TRec = record',
  '    procedure Run(w: word = 10);',
  '  end;',
  '  THelper = record helper for TRec',
  '    function Foo(w: word = 1): word;',
  '  end;',
  'procedure TRec.Run(w: word);',
  'begin',
  '  Foo;',
  '  Foo();',
  '  Foo(2);',
  '  Self.Foo;',
  '  Self.Foo();',
  '  Self.Foo(3);',
  '  with Self do begin',
  '    Foo;',
  '    Foo();',
  '    Foo(4);',
  '  end;',
  'end;',
  'function THelper.foo(w: word): word;',
  'begin',
  '  Run;',
  '  Run();',
  '  Run(11);',
  '  Foo;',
  '  Foo();',
  '  Foo(12);',
  '  Self.Foo;',
  '  Self.Foo();',
  '  Self.Foo(13);',
  '  with Self do begin',
  '    Foo;',
  '    Foo();',
  '    Foo(14);',
  '  end;',
  'end;',
  'var Rec: TRec;',
  'begin',
  '  Rec.Foo;',
  '  Rec.Foo();',
  '  Rec.Foo(21);',
  '  with Rec do begin',
  '    Foo;',
  '    Foo();',
  '    Foo(22);',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestRecordHelper_Method_Call',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  this.Run = function (w) {',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 2);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 3);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 4);',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Foo = function (w) {',
    '    var Result = 0;',
    '    this.Run(10);',
    '    this.Run(10);',
    '    this.Run(11);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 12);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 13);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 1);',
    '    $mod.THelper.Foo.call(this, 14);',
    '    return Result;',
    '  };',
    '});',
    'this.Rec = this.TRec.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Foo.call($mod.Rec, 1);',
    '$mod.THelper.Foo.call($mod.Rec, 1);',
    '$mod.THelper.Foo.call($mod.Rec, 21);',
    'var $with = $mod.Rec;',
    '$mod.THelper.Foo.call($with, 1);',
    '$mod.THelper.Foo.call($with, 1);',
    '$mod.THelper.Foo.call($with, 22);',
    '']));
end;

procedure TTestModule.TestRecordHelper_Constructor;
begin
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TRec = record',
  '    constructor Create(w: word);',
  '  end;',
  '  THelper = record helper for TRec',
  '    constructor NewHlp(w: word);',
  '  end;',
  'var',
  '  Rec: TRec;',
  'constructor TRec.Create(w: word);',
  'begin',
  '  NewHlp(2);', // normal call
  '  trec.NewHlp(3);', // new instance
  'end;',
  'constructor THelper.NewHlp(w: word);',
  'begin',
  '  create(2);', // normal call
  '  trec.create(3);', // new instance
  '  NewHlp(4);', // normal call
  '  trec.NewHlp(5);', // new instance
  'end;',
  'begin',
  '  rec.newhlp(2);', // normal call
  '  with rec do newhlp(12);', // normal call
  '  trec.newhlp(3);', // new instance
  '  with trec do newhlp(13);', // new instance
  '']);
  ConvertProgram;
  CheckSource('TestRecordHelper_Constructor',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  this.Create = function (w) {',
    '    $mod.THelper.NewHlp.call(this, 2);',
    '    $mod.THelper.$new("NewHlp", [3]);',
    '    return this;',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.NewHlp = function (w) {',
    '    this.Create(2);',
    '    $mod.TRec.$new().Create(3);',
    '    $mod.THelper.NewHlp.call(this, 4);',
    '    $mod.THelper.$new("NewHlp", [5]);',
    '    return this;',
    '  };',
    '  this.$new = function (fn, args) {',
    '    return this[fn].apply($mod.TRec.$new(), args);',
    '  };',
    '});',
    'this.Rec = this.TRec.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.NewHlp.call($mod.Rec, 2);',
    'var $with = $mod.Rec;',
    '$mod.THelper.NewHlp.call($with, 12);',
    '$mod.THelper.$new("NewHlp", [3]);',
    'var $with1 = $mod.TRec;',
    '$mod.THelper.$new("NewHlp", [13]);',
    '']));
end;

procedure TTestModule.TestTypeHelper_ClassVar;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for byte',
  '    const',
  '      One = 1;',
  '      Two: word = 2;',
  '    class var',
  '      Glob: word;',
  '    function Foo(w: word): word;',
  '    class function Bar(w: word): word; static;',
  '  end;',
  'function THelper.foo(w: word): word;',
  'begin',
  '  Result:=w;',
  '  Two:=One+w;',
  '  Glob:=Glob;',
  '  Result:=Self.Glob;',
  '  Self.Glob:=Self.Glob;',
  '  with Self do Glob:=Glob;',
  'end;',
  'class function THelper.bar(w: word): word;',
  'begin',
  '  Result:=w;',
  '  Two:=One;',
  '  Glob:=Glob;',
  'end;',
  'var b: byte;',
  'begin',
  '  byte.two:=byte.one;',
  '  byte.Glob:=byte.Glob;',
  '  with byte do begin',
  '    two:=one;',
  '    Glob:=Glob;',
  '  end;',
  '  b.two:=b.one;',
  '  b.Glob:=b.Glob;',
  '  with b do begin',
  '    two:=one;',
  '    Glob:=Glob;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_ClassVar',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.One = 1;',
    '  this.Two = 2;',
    '  this.Glob = 0;',
    '  this.Foo = function (w) {',
    '    var Result = 0;',
    '    Result = w;',
    '    $mod.THelper.Two = 1 + w;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    Result = $mod.THelper.Glob;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    var $with = this.get();',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    return Result;',
    '  };',
    '  this.Bar = function (w) {',
    '    var Result = 0;',
    '    Result = w;',
    '    $mod.THelper.Two = 1;',
    '    $mod.THelper.Glob = $mod.THelper.Glob;',
    '    return Result;',
    '  };',
    '});',
    'this.b = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    'var $with = $mod.b;',
    '$mod.THelper.Two = 1;',
    '$mod.THelper.Glob = $mod.THelper.Glob;',
    '']));
end;

procedure TTestModule.TestTypeHelper_PassResultElement;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    procedure DoIt(e: byte = 123);',
  '    class procedure DoSome(e: byte = 456); static;',
  '  end;',
  'procedure THelper.DoIt(e: byte);',
  'begin',
  'end;',
  'class procedure THelper.DoSome(e: byte);',
  'begin',
  'end;',
  'function Foo(w: word): word;',
  'begin',
  '  Result.DoIt;',
  '  Result.DoIt();',
  '  Result.DoSome;',
  '  Result.DoSome();',
  '  with Result do begin',
  '    DoIt;',
  '    DoIt();',
  '    DoSome;',
  '    DoSome();',
  '  end;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_PassResultElement',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.DoIt = function (e) {',
    '  };',
    '  this.DoSome = function (e) {',
    '  };',
    '});',
    'this.Foo = function (w) {',
    '  var Result = 0;',
    '  $mod.THelper.DoIt.call({',
    '    get: function () {',
    '        return Result;',
    '      },',
    '    set: function (v) {',
    '        Result = v;',
    '      }',
    '  }, 123);',
    '  $mod.THelper.DoIt.call({',
    '    get: function () {',
    '        return Result;',
    '      },',
    '    set: function (v) {',
    '        Result = v;',
    '      }',
    '  }, 123);',
    '  $mod.THelper.DoSome(456);',
    '  $mod.THelper.DoSome(456);',
    '  $mod.THelper.DoIt.call({',
    '    get: function () {',
    '        return Result;',
    '      },',
    '    set: function (v) {',
    '        Result = v;',
    '      }',
    '  }, 123);',
    '  $mod.THelper.DoIt.call({',
    '    get: function () {',
    '        return Result;',
    '      },',
    '    set: function (v) {',
    '        Result = v;',
    '      }',
    '  }, 123);',
    '  $mod.THelper.DoSome(456);',
    '  $mod.THelper.DoSome(456);',
    '  return Result;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestTypeHelper_PassArgs;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    procedure DoIt(e: byte = 123);',
  '  end;',
  'procedure THelper.DoIt(e: byte);',
  'begin',
  'end;',
  'procedure FooDefault(a: word);',
  'begin',
  '  a.DoIt;',
  '  with a do DoIt;',
  'end;',
  'procedure FooConst(const a: word);',
  'begin',
  '  a.DoIt;',
  '  with a do DoIt;',
  'end;',
  'procedure FooVar(var a: word);',
  'begin',
  '  a.DoIt;',
  '  with a do DoIt;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_PassArgs',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.DoIt = function (e) {',
    '  };',
    '});',
    'this.FooDefault = function (a) {',
    '  $mod.THelper.DoIt.call({',
    '    get: function () {',
    '        return a;',
    '      },',
    '    set: function (v) {',
    '        a = v;',
    '      }',
    '  }, 123);',
    '  $mod.THelper.DoIt.call({',
    '    get: function () {',
    '        return a;',
    '      },',
    '    set: function (v) {',
    '        a = v;',
    '      }',
    '  }, 123);',
    '};',
    'this.FooConst = function (a) {',
    '  $mod.THelper.DoIt.call({',
    '    get: function () {',
    '        return a;',
    '      },',
    '    set: function (v) {',
    '        rtl.raiseE("EPropReadOnly");',
    '      }',
    '  }, 123);',
    '  $mod.THelper.DoIt.call({',
    '    get: function () {',
    '        return a;',
    '      },',
    '    set: function () {',
    '        rtl.raiseE("EPropReadOnly");',
    '      }',
    '  }, 123);',
    '};',
    'this.FooVar = function (a) {',
    '  $mod.THelper.DoIt.call(a, 123);',
    '  var $with = a.get();',
    '  $mod.THelper.DoIt.call(a, 123);',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestTypeHelper_PassVarConst;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    procedure DoIt(e: byte = 123);',
  '  end;',
  'procedure THelper.DoIt(e: byte);',
  'begin',
  'end;',
  'var a: word;',
  'const c: word = 2;',
  '{$writeableconst off}',
  'const r: word = 3;',
  'begin',
  '  a.DoIt;',
  '  with a do DoIt;',
  '  c.DoIt;',
  '  with c do DoIt;',
  '  r.DoIt;',
  '  with r do DoIt;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_PassVarConst',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.DoIt = function (e) {',
    '  };',
    '});',
    'this.a = 0;',
    'this.c = 2;',
    'this.r = 3;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.DoIt.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.a;',
    '    },',
    '  set: function (v) {',
    '      this.p.a = v;',
    '    }',
    '}, 123);',
    'var $with = $mod.a;',
    '$mod.THelper.DoIt.call({',
    '  get: function () {',
    '      return $with;',
    '    },',
    '  set: function (v) {',
    '      $with = v;',
    '    }',
    '}, 123);',
    '$mod.THelper.DoIt.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.c;',
    '    },',
    '  set: function (v) {',
    '      this.p.c = v;',
    '    }',
    '}, 123);',
    'var $with1 = $mod.c;',
    '$mod.THelper.DoIt.call({',
    '  get: function () {',
    '      return $with1;',
    '    },',
    '  set: function (v) {',
    '      $with1 = v;',
    '    }',
    '}, 123);',
    '$mod.THelper.DoIt.call({',
    '  get: function () {',
    '      return 3;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}, 123);',
    'var $with2 = 3;',
    '  $mod.THelper.DoIt.call({',
    '    get: function () {',
    '        return $with2;',
    '      },',
    '    set: function () {',
    '        rtl.raiseE("EPropReadOnly");',
    '      }',
    '  }, 123);',
    '']));
end;

procedure TTestModule.TestTypeHelper_PassFuncResult;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    procedure DoIt(e: byte = 123);',
  '  end;',
  'procedure THelper.DoIt(e: byte);',
  'begin',
  'end;',
  'function Foo(b: byte = 1): word;',
  'begin',
  'end;',
  'begin',
  '  Foo.DoIt;',
  '  Foo().DoIt;',
  '  with Foo do DoIt;',
  '  with Foo() do DoIt;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_PassFuncResult',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.DoIt = function (e) {',
    '  };',
    '});',
    'this.Foo = function (b) {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.DoIt.call({',
    '  a: $mod.Foo(1),',
    '  get: function () {',
    '      return this.a;',
    '    },',
    '  set: function (v) {',
    '      this.a = v;',
    '    }',
    '}, 123);',
    '$mod.THelper.DoIt.call({',
    '  a: $mod.Foo(1),',
    '  get: function () {',
    '      return this.a;',
    '    },',
    '  set: function (v) {',
    '      this.a = v;',
    '    }',
    '}, 123);',
    'var $with = $mod.Foo(1);',
    '$mod.THelper.DoIt.call({',
    '  get: function () {',
    '      return $with;',
    '    },',
    '  set: function (v) {',
    '      $with = v;',
    '    }',
    '}, 123);',
    'var $with1 = $mod.Foo(1);',
    '$mod.THelper.DoIt.call({',
    '  get: function () {',
    '      return $with1;',
    '    },',
    '  set: function (v) {',
    '      $with1 = v;',
    '    }',
    '}, 123);',
    '']));
end;

procedure TTestModule.TestTypeHelper_PassPropertyField;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TObject = class',
  '    FField: word;',
  '    procedure SetField(Value: word);',
  '    property Field: word read FField write SetField;',
  '  end;',
  '  THelper = type helper for word',
  '    procedure Fly;',
  '    class procedure Run; static;',
  '  end;',
  'procedure TObject.SetField(Value: word);',
  'begin',
  '  Field.Fly;',
  '  Field.Run;',
  '  Self.Field.Fly;',
  '  Self.Field.Run;',
  '  with Self do begin',
  '    Field.Fly;',
  '    Field.Run;',
  '  end;',
  '  with Self.Field do begin',
  '    Fly;',
  '    Run;',
  '  end;',
  'end;',
  'procedure THelper.Fly;',
  'begin',
  'end;',
  'class procedure THelper.Run;',
  'begin',
  'end;',
  'var',
  '  o: TObject;',
  'begin',
  '  o.Field.Fly;',
  '  o.Field.Run;',
  '  with o do begin',
  '    Field.Fly;',
  '    Field.Run;',
  '  end;',
  '  with o.Field do begin',
  '    Fly;',
  '    Run;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_PassPropertyField',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FField = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.SetField = function (Value) {',
    '    $mod.THelper.Fly.call({',
    '      p: this,',
    '      get: function () {',
    '          return this.p.FField;',
    '        },',
    '      set: function (v) {',
    '          this.p.FField = v;',
    '        }',
    '    });',
    '    $mod.THelper.Run();',
    '    $mod.THelper.Fly.call({',
    '      p: this,',
    '      get: function () {',
    '          return this.p.FField;',
    '        },',
    '      set: function (v) {',
    '          this.p.FField = v;',
    '        }',
    '    });',
    '    $mod.THelper.Run();',
    '    $mod.THelper.Fly.call({',
    '      p: this,',
    '      get: function () {',
    '          return this.p.FField;',
    '        },',
    '      set: function (v) {',
    '          this.p.FField = v;',
    '        }',
    '    });',
    '    $mod.THelper.Run();',
    '    var $with = this.FField;',
    '    $mod.THelper.Fly.call({',
    '      get: function () {',
    '          return $with;',
    '        },',
    '      set: function (v) {',
    '          $with = v;',
    '        }',
    '    });',
    '    $mod.THelper.Run();',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Fly = function () {',
    '  };',
    '  this.Run = function () {',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Fly.call({',
    '  p: $mod.o,',
    '  get: function () {',
    '      return this.p.FField;',
    '    },',
    '  set: function (v) {',
    '      this.p.FField = v;',
    '    }',
    '});',
    '$mod.THelper.Run();',
    'var $with = $mod.o;',
    '$mod.THelper.Fly.call({',
    '  p: $with,',
    '  get: function () {',
    '      return this.p.FField;',
    '    },',
    '  set: function (v) {',
    '      this.p.FField = v;',
    '    }',
    '});',
    '$mod.THelper.Run();',
    'var $with1 = $mod.o.FField;',
    '$mod.THelper.Fly.call({',
    '  get: function () {',
    '      return $with1;',
    '    },',
    '  set: function (v) {',
    '      $with1 = v;',
    '    }',
    '});',
    '$mod.THelper.Run();',
    '']));
end;

procedure TTestModule.TestTypeHelper_PassPropertyGetter;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TObject = class',
  '    FField: word;',
  '    function GetField: word;',
  '    property Field: word read GetField write FField;',
  '  end;',
  '  THelper = type helper for word',
  '    procedure Fly;',
  '    class procedure Run; static;',
  '  end;',
  'function TObject.GetField: word;',
  'begin',
  '  Field.Fly;',
  '  Field.Run;',
  '  Self.Field.Fly;',
  '  Self.Field.Run;',
  '  with Self do begin',
  '    Field.Fly;',
  '    Field.Run;',
  '  end;',
  '  with Self.Field do begin',
  '    Fly;',
  '    Run;',
  '  end;',
  'end;',
  'procedure THelper.Fly;',
  'begin',
  'end;',
  'class procedure THelper.Run;',
  'begin',
  'end;',
  'var',
  '  o: TObject;',
  'begin',
  '  o.Field.Fly;',
  '  o.Field.Run;',
  '  with o do begin',
  '    Field.Fly;',
  '    Field.Run;',
  '  end;',
  '  with o.Field do begin',
  '    Fly;',
  '    Run;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_PassPropertyGetter',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FField = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetField = function () {',
    '    var Result = 0;',
    '    $mod.THelper.Fly.call({',
    '      p: this.GetField(),',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    });',
    '    $mod.THelper.Run();',
    '    $mod.THelper.Fly.call({',
    '      p: this.GetField(),',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    });',
    '    $mod.THelper.Run();',
    '    $mod.THelper.Fly.call({',
    '      p: this.GetField(),',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    });',
    '    $mod.THelper.Run();',
    '    var $with = this.GetField();',
    '    $mod.THelper.Fly.call({',
    '      get: function () {',
    '          return $with;',
    '        },',
    '      set: function (v) {',
    '          $with = v;',
    '        }',
    '    });',
    '    $mod.THelper.Run();',
    '    return Result;',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Fly = function () {',
    '  };',
    '  this.Run = function () {',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Fly.call({',
    '  p: $mod.o.GetField(),',
    '  get: function () {',
    '      return this.p;',
    '    },',
    '  set: function (v) {',
    '      this.p = v;',
    '    }',
    '});',
    '$mod.THelper.Run();',
    'var $with = $mod.o;',
    '$mod.THelper.Fly.call({',
    '  p: $with.GetField(),',
    '  get: function () {',
    '      return this.p;',
    '    },',
    '  set: function (v) {',
    '      this.p = v;',
    '    }',
    '});',
    '$mod.THelper.Run();',
    'var $with1 = $mod.o.GetField();',
    '$mod.THelper.Fly.call({',
    '  get: function () {',
    '      return $with1;',
    '    },',
    '  set: function (v) {',
    '      $with1 = v;',
    '    }',
    '});',
    '$mod.THelper.Run();',
    '']));
end;

procedure TTestModule.TestTypeHelper_PassClassPropertyField;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TObject = class',
  '    class var FField: word;',
  '    class procedure SetField(Value: word);',
  '    class property Field: word read FField write SetField;',
  '  end;',
  '  THelper = type helper for word',
  '    procedure Fly(n: byte);',
  '  end;',
  'class procedure TObject.SetField(Value: word);',
  'begin',
  '  Field.Fly(1);',
  '  Self.Field.Fly(2);',
  '  with Self do Field.Fly(3);',
  '  with Self.Field do Fly(4);',
  '  TObject.Field.Fly(5);',
  '  with TObject do Field.Fly(6);',
  '  with TObject.Field do Fly(7);',
  'end;',
  'procedure THelper.Fly(n: byte);',
  'begin',
  'end;',
  'var',
  '  o: TObject;',
  'begin',
  '  o.Field.Fly(11);',
  '  with o do Field.Fly(12);',
  '  with o.Field do Fly(13);',
  '  TObject.Field.Fly(14);',
  '  with TObject do Field.Fly(15);',
  '  with TObject.Field do Fly(16);',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_PassClassPropertyField',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.FField = 0;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.SetField = function (Value) {',
    '    $mod.THelper.Fly.call({',
    '      p: this,',
    '      get: function () {',
    '          return this.p.FField;',
    '        },',
    '      set: function (v) {',
    '          $mod.TObject.FField = v;',
    '        }',
    '    }, 1);',
    '    $mod.THelper.Fly.call({',
    '      p: this,',
    '      get: function () {',
    '          return this.p.FField;',
    '        },',
    '      set: function (v) {',
    '          $mod.TObject.FField = v;',
    '        }',
    '    }, 2);',
    '    $mod.THelper.Fly.call({',
    '      p: this,',
    '      get: function () {',
    '          return this.p.FField;',
    '        },',
    '      set: function (v) {',
    '          $mod.TObject.FField = v;',
    '        }',
    '    }, 3);',
    '    var $with = this.FField;',
    '    $mod.THelper.Fly.call({',
    '      get: function () {',
    '          return $with;',
    '        },',
    '      set: function (v) {',
    '          $with = v;',
    '        }',
    '    }, 4);',
    '    $mod.THelper.Fly.call({',
    '      p: $mod.TObject,',
    '      get: function () {',
    '          return this.p.FField;',
    '        },',
    '      set: function (v) {',
    '          $mod.TObject.FField = v;',
    '        }',
    '    }, 5);',
    '    var $with1 = $mod.TObject;',
    '    $mod.THelper.Fly.call({',
    '      p: $with1,',
    '      get: function () {',
    '          return this.p.FField;',
    '        },',
    '      set: function (v) {',
    '          $mod.TObject.FField = v;',
    '        }',
    '    }, 6);',
    '    var $with2 = $mod.TObject.FField;',
    '    $mod.THelper.Fly.call({',
    '      get: function () {',
    '          return $with2;',
    '        },',
    '      set: function (v) {',
    '          $with2 = v;',
    '        }',
    '    }, 7);',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Fly = function (n) {',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Fly.call({',
    '  p: $mod.o,',
    '  get: function () {',
    '      return this.p.FField;',
    '    },',
    '  set: function (v) {',
    '      $mod.TObject.FField = v;',
    '    }',
    '}, 11);',
    'var $with = $mod.o;',
    '$mod.THelper.Fly.call({',
    '  p: $with,',
    '  get: function () {',
    '      return this.p.FField;',
    '    },',
    '  set: function (v) {',
    '      $mod.TObject.FField = v;',
    '    }',
    '}, 12);',
    'var $with1 = $mod.o.FField;',
    '$mod.THelper.Fly.call({',
    '  get: function () {',
    '      return $with1;',
    '    },',
    '  set: function (v) {',
    '      $with1 = v;',
    '    }',
    '}, 13);',
    '$mod.THelper.Fly.call({',
    '  p: $mod.TObject,',
    '  get: function () {',
    '      return this.p.FField;',
    '    },',
    '  set: function (v) {',
    '      $mod.TObject.FField = v;',
    '    }',
    '}, 14);',
    'var $with2 = $mod.TObject;',
    '$mod.THelper.Fly.call({',
    '  p: $with2,',
    '  get: function () {',
    '      return this.p.FField;',
    '    },',
    '  set: function (v) {',
    '      $mod.TObject.FField = v;',
    '    }',
    '}, 15);',
    'var $with3 = $mod.TObject.FField;',
    '$mod.THelper.Fly.call({',
    '  get: function () {',
    '      return $with3;',
    '    },',
    '  set: function (v) {',
    '      $with3 = v;',
    '    }',
    '}, 16);',
    '']));
end;

procedure TTestModule.TestTypeHelper_PassClassPropertyGetterStatic;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TObject = class',
  '    class var FField: word;',
  '    class function GetField: word; static;',
  '    class property Field: word read GetField write FField;',
  '  end;',
  '  THelper = type helper for word',
  '    procedure Fly(n: byte);',
  '  end;',
  'class function TObject.GetField: word;',
  'begin',
  '  Field.Fly(1);',
  '  TObject.Field.Fly(5);',
  '  with TObject do Field.Fly(6);',
  '  with TObject.Field do Fly(7);',
  'end;',
  'procedure THelper.Fly(n: byte);',
  'begin',
  'end;',
  'var',
  '  o: TObject;',
  'begin',
  '  o.Field.Fly(11);',
  '  with o do Field.Fly(12);',
  '  with o.Field do Fly(13);',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_PassClassPropertyGetterStatic',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.FField = 0;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetField = function () {',
    '    var Result = 0;',
    '    $mod.THelper.Fly.call({',
    '      p: $mod.TObject.GetField(),',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    }, 1);',
    '    $mod.THelper.Fly.call({',
    '      p: $mod.TObject.GetField(),',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    }, 5);',
    '    var $with = $mod.TObject;',
    '    $mod.THelper.Fly.call({',
    '      p: $with.GetField(),',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    }, 6);',
    '    var $with1 = $mod.TObject.GetField();',
    '    $mod.THelper.Fly.call({',
    '      get: function () {',
    '          return $with1;',
    '        },',
    '      set: function (v) {',
    '          $with1 = v;',
    '        }',
    '    }, 7);',
    '    return Result;',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Fly = function (n) {',
    '  };',
    '});',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Fly.call({',
    '  p: $mod.TObject.GetField(),',
    '  get: function () {',
    '      return this.p;',
    '    },',
    '  set: function (v) {',
    '      this.p = v;',
    '    }',
    '}, 11);',
    'var $with = $mod.o;',
    '$mod.THelper.Fly.call({',
    '  p: $with.GetField(),',
    '  get: function () {',
    '      return this.p;',
    '    },',
    '  set: function (v) {',
    '      this.p = v;',
    '    }',
    '}, 12);',
    'var $with1 = $mod.TObject.GetField();',
    '$mod.THelper.Fly.call({',
    '  get: function () {',
    '      return $with1;',
    '    },',
    '  set: function (v) {',
    '      $with1 = v;',
    '    }',
    '}, 13);',
    '']));
end;

procedure TTestModule.TestTypeHelper_PassClassPropertyGetterNonStatic;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TObject = class',
  '    class var FField: word;',
  '    class function GetField: word;',
  '    class property Field: word read GetField write FField;',
  '  end;',
  '  TClass = class of TObject;',
  '  THelper = type helper for word',
  '    procedure Fly(n: byte);',
  '  end;',
  'class function TObject.GetField: word;',
  'begin',
  '  Field.Fly(1);',
  '  Self.Field.Fly(5);',
  '  with Self do Field.Fly(6);',
  '  with Self.Field do Fly(7);',
  'end;',
  'procedure THelper.Fly(n: byte);',
  'begin',
  'end;',
  'var',
  '  o: TObject;',
  '  c: TClass;',
  'begin',
  '  o.Field.Fly(11);',
  '  with o do Field.Fly(12);',
  '  with o.Field do Fly(13);',
  '  c.Field.Fly(14);',
  '  with c do Field.Fly(15);',
  '  with c.Field do Fly(16);',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_PassClassPropertyGetterNonStatic',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.FField = 0;',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetField = function () {',
    '    var Result = 0;',
    '    $mod.THelper.Fly.call({',
    '      p: this.GetField(),',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    }, 1);',
    '    $mod.THelper.Fly.call({',
    '      p: this.GetField(),',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    }, 5);',
    '    $mod.THelper.Fly.call({',
    '      p: this.GetField(),',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    }, 6);',
    '    var $with = this.GetField();',
    '    $mod.THelper.Fly.call({',
    '      get: function () {',
    '          return $with;',
    '        },',
    '      set: function (v) {',
    '          $with = v;',
    '        }',
    '    }, 7);',
    '    return Result;',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Fly = function (n) {',
    '  };',
    '});',
    'this.o = null;',
    'this.c = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Fly.call({',
    '  p: $mod.o.$class.GetField(),',
    '  get: function () {',
    '      return this.p;',
    '    },',
    '  set: function (v) {',
    '      this.p = v;',
    '    }',
    '}, 11);',
    'var $with = $mod.o;',
    '$mod.THelper.Fly.call({',
    '  p: $with.$class.GetField(),',
    '  get: function () {',
    '      return this.p;',
    '    },',
    '  set: function (v) {',
    '      this.p = v;',
    '    }',
    '}, 12);',
    'var $with1 = $mod.o.$class.GetField();',
    '$mod.THelper.Fly.call({',
    '  get: function () {',
    '      return $with1;',
    '    },',
    '  set: function (v) {',
    '      $with1 = v;',
    '    }',
    '}, 13);',
    '$mod.THelper.Fly.call({',
    '  p: $mod.c.GetField(),',
    '  get: function () {',
    '      return this.p;',
    '    },',
    '  set: function (v) {',
    '      this.p = v;',
    '    }',
    '}, 14);',
    'var $with2 = $mod.c;',
    '$mod.THelper.Fly.call({',
    '  p: $with2.GetField(),',
    '  get: function () {',
    '      return this.p;',
    '    },',
    '  set: function (v) {',
    '      this.p = v;',
    '    }',
    '}, 15);',
    'var $with3 = $mod.c.GetField();',
    '$mod.THelper.Fly.call({',
    '  get: function () {',
    '      return $with3;',
    '    },',
    '  set: function (v) {',
    '      $with3 = v;',
    '    }',
    '}, 16);',
    '']));
end;

procedure TTestModule.TestTypeHelper_Property;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    function GetSize: longint;',
  '    procedure SetSize(Value: longint);',
  '    property Size: longint read GetSize write SetSize;',
  '  end;',
  'function THelper.GetSize: longint;',
  'begin',
  '  Result:=Size+1;',
  '  Size:=2;',
  '  Result:=Self.Size+3;',
  '  Self.Size:=4;',
  '  with Self do begin',
  '    Result:=Size+5;',
  '    Size:=6;',
  '  end;',
  'end;',
  'procedure THelper.SetSize(Value: longint);',
  'begin',
  'end;',
  'var w: word;',
  'begin',
  '  w:=w.Size+7;',
  '  w.Size:=w+8;',
  '  with w do begin',
  '    w:=Size+9;',
  '    Size:=w+10;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_Property',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.GetSize = function () {',
    '    var Result = 0;',
    '    Result = $mod.THelper.GetSize.call(this) + 1;',
    '    $mod.THelper.SetSize.call(this, 2);',
    '    Result = $mod.THelper.GetSize.call(this) + 3;',
    '    $mod.THelper.SetSize.call(this, 4);',
    '    var $with = this.get();',
    '    Result = $mod.THelper.GetSize.call(this) + 5;',
    '    $mod.THelper.SetSize.call(this, 6);',
    '    return Result;',
    '  };',
    '  this.SetSize = function (Value) {',
    '  };',
    '});',
    'this.w = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.w = $mod.THelper.GetSize.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.w;',
    '    },',
    '  set: function (v) {',
    '      this.p.w = v;',
    '    }',
    '}) + 7;',
    '$mod.THelper.SetSize.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.w;',
    '    },',
    '  set: function (v) {',
    '      this.p.w = v;',
    '    }',
    '}, $mod.w + 8);',
    'var $with = $mod.w;',
    '$mod.w = $mod.THelper.GetSize.call({',
    '  get: function () {',
    '      return $with;',
    '    },',
    '  set: function (v) {',
    '      $with = v;',
    '    }',
    '}) + 9;',
    '$mod.THelper.SetSize.call({',
    '  get: function () {',
    '      return $with;',
    '    },',
    '  set: function (v) {',
    '      $with = v;',
    '    }',
    '}, $mod.w + 10);',
    '']));
end;

procedure TTestModule.TestTypeHelper_Property_Array;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    function GetItems(Index: byte): boolean;',
  '    procedure SetItems(Index: byte; Value: boolean);',
  '    property Items[Index: byte]: boolean read GetItems write SetItems;',
  '  end;',
  'function THelper.GetItems(Index: byte): boolean;',
  'begin',
  '  Result:=Items[1];',
  '  Items[2]:=false;',
  '  Result:=Self.Items[3];',
  '  Self.Items[4]:=true;',
  '  with Self do begin',
  '    Result:=Items[5];',
  '    Items[6]:=false;',
  '  end;',
  'end;',
  'procedure THelper.SetItems(Index: byte; Value: boolean);',
  'begin',
  'end;',
  'var',
  '  w: word;',
  '  b: boolean;',
  'begin',
  '  b:=w.Items[1];',
  '  w.Items[2]:=b;',
  '  with w do begin',
  '    b:=Items[3];',
  '    Items[4]:=b;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_Property_Array',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.GetItems = function (Index) {',
    '    var Result = false;',
    '    Result = $mod.THelper.GetItems.call(this, 1);',
    '    $mod.THelper.SetItems.call(this, 2, false);',
    '    Result = $mod.THelper.GetItems.call(this, 3);',
    '    $mod.THelper.SetItems.call(this, 4, true);',
    '    var $with = this.get();',
    '    Result = $mod.THelper.GetItems.call(this, 5);',
    '    $mod.THelper.SetItems.call(this, 6, false);',
    '    return Result;',
    '  };',
    '  this.SetItems = function (Index, Value) {',
    '  };',
    '});',
    'this.w = 0;',
    'this.b = false;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.b = $mod.THelper.GetItems.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.w;',
    '    },',
    '  set: function (v) {',
    '      this.p.w = v;',
    '    }',
    '}, 1);',
    '$mod.THelper.SetItems.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.w;',
    '    },',
    '  set: function (v) {',
    '      this.p.w = v;',
    '    }',
    '}, 2, $mod.b);',
    'var $with = $mod.w;',
    '$mod.b = $mod.THelper.GetItems.call({',
    '  get: function () {',
    '      return $with;',
    '    },',
    '  set: function (v) {',
    '      $with = v;',
    '    }',
    '}, 3);',
    '$mod.THelper.SetItems.call({',
    '  get: function () {',
    '      return $with;',
    '    },',
    '  set: function (v) {',
    '      $with = v;',
    '    }',
    '}, 4, $mod.b);',
    '']));
end;

procedure TTestModule.TestTypeHelper_ClassProperty;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    class function GetSize: longint; static;',
  '    class procedure SetSize(Value: longint); static;',
  '    class property Size: longint read GetSize write SetSize;',
  '  end;',
  'class function THelper.GetSize: longint;',
  'begin',
  '  Result:=Size+1;',
  '  Size:=2;',
  'end;',
  'class procedure THelper.SetSize(Value: longint);',
  'begin',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_ClassProperty',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.GetSize = function () {',
    '    var Result = 0;',
    '    Result = $mod.THelper.GetSize() + 1;',
    '    $mod.THelper.SetSize(2);',
    '    return Result;',
    '  };',
    '  this.SetSize = function (Value) {',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestTypeHelper_ClassProperty_Array;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    class function GetItems(Index: byte): boolean; static;',
  '    class procedure SetItems(Index: byte; Value: boolean); static;',
  '    class property Items[Index: byte]: boolean read GetItems write SetItems;',
  '  end;',
  'class function THelper.GetItems(Index: byte): boolean;',
  'begin',
  '  Result:=Items[1];',
  '  Items[2]:=false;',
  'end;',
  'class procedure THelper.SetItems(Index: byte; Value: boolean);',
  'begin',
  'end;',
  'var',
  '  w: word;',
  '  b: boolean;',
  'begin',
  '  b:=w.Items[1];',
  '  w.Items[2]:=b;',
  '  with w do begin',
  '    b:=Items[3];',
  '    Items[4]:=b;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_ClassProperty_Array',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.GetItems = function (Index) {',
    '    var Result = false;',
    '    Result = $mod.THelper.GetItems(1);',
    '    $mod.THelper.SetItems(2, false);',
    '    return Result;',
    '  };',
    '  this.SetItems = function (Index, Value) {',
    '  };',
    '});',
    'this.w = 0;',
    'this.b = false;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.b = $mod.THelper.GetItems(1);',
    '$mod.THelper.SetItems(2, $mod.b);',
    'var $with = $mod.w;',
    '$mod.b = $mod.THelper.GetItems(3);',
    '$mod.THelper.SetItems(4, $mod.b);',
    '']));
end;

procedure TTestModule.TestTypeHelper_ClassMethod;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    class procedure DoStatic; static;',
  '  end;',
  'class procedure THelper.DoStatic;',
  'begin',
  '  DoStatic;',
  '  DoStatic();',
  'end;',
  'var w: word;',
  'begin',
  '  w.DoStatic;',
  '  w.DoStatic();',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_ClassMethod',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.DoStatic = function () {',
    '    $mod.THelper.DoStatic();',
    '    $mod.THelper.DoStatic();',
    '  };',
    '});',
    'this.w = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.DoStatic();',
    '$mod.THelper.DoStatic();',
    '']));
end;

procedure TTestModule.TestTypeHelper_ExtClassMethodFail;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    procedure Run; external name ''Run'';',
  '  end;',
  'var w: word;',
  'begin',
  '  w.Run;',
  '']);
  SetExpectedPasResolverError('Not supported: external method in type helper',nNotSupportedX);
  ConvertProgram;
end;

procedure TTestModule.TestTypeHelper_Constructor;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    constructor Init(e: longint);',
  '  end;',
  'constructor THelper.Init(e: longint);',
  'begin',
  '  Self:=e;',
  '  Init(e+1);',
  'end;',
  'var w: word;',
  'begin',
  '  w:=word.Init(2);',
  '  w:=w.Init(3);',
  '  with word do w:=Init(4);',
  '  with w do w:=Init(5);',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_Constructor',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Init = function (e) {',
    '    this.set(e);',
    '    $mod.THelper.Init.call(this, e + 1);',
    '    return this.get();',
    '  };',
    '  this.$new = function (fn, args) {',
    '    return this[fn].apply({',
    '      p: 0,',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    }, args);',
    '  };',
    '});',
    'this.w = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.w = $mod.THelper.$new("Init", [2]);',
    '$mod.w = $mod.THelper.Init.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.w;',
    '    },',
    '  set: function (v) {',
    '      this.p.w = v;',
    '    }',
    '}, 3);',
    '$mod.w = $mod.THelper.$new("Init", [4]);',
    'var $with = $mod.w;',
    '$mod.w = $mod.THelper.Init.call({',
    '  get: function () {',
    '      return $with;',
    '    },',
    '  set: function (v) {',
    '      $with = v;',
    '    }',
    '}, 5);',
    '']));
end;

procedure TTestModule.TestTypeHelper_Word;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for word',
  '    procedure DoIt(e: byte = 123);',
  '  end;',
  'procedure THelper.DoIt(e: byte);',
  'begin',
  '  Self:=e;',
  '  Self:=Self+1;',
  '  with Self do Doit;',
  'end;',
  'begin',
  '  word(3).DoIt;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_Word',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.DoIt = function (e) {',
    '    this.set(e);',
    '    this.set(this.get() + 1);',
    '    var $with = this.get();',
    '    $mod.THelper.DoIt.call(this, 123);',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.DoIt.call({',
    '  get: function () {',
    '      return 3;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}, 123);',
    '']));
end;

procedure TTestModule.TestTypeHelper_Boolean;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  Integer = longint;',
  '  THelper = type helper for boolean',
  '    procedure Run(e: wordbool = true);',
  '  end;',
  'procedure THelper.Run(e: wordbool);',
  'begin',
  '  Self:=e;',
  '  Self:=not Self;',
  '  with Self do Run;',
  '  if Integer(Self)=0 then ;',
  'end;',
  'begin',
  '  boolean(3).Run;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_Boolean',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Run = function (e) {',
    '    this.set(e);',
    '    this.set(!this.get());',
    '    var $with = this.get();',
    '    $mod.THelper.Run.call(this, true);',
    '    if ((this.get() ? 1 : 0) === 0) ;',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Run.call({',
    '  a: 3 != 0,',
    '  get: function () {',
    '      return this.a;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}, true);',
    '']));
end;

procedure TTestModule.TestTypeHelper_WordBool;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  Integer = longint;',
  '  THelper = type helper for WordBool',
  '    procedure Run(e: wordbool = true);',
  '  end;',
  'procedure THelper.Run(e: wordbool);',
  'var i: integer;',
  'begin',
  '  i:=Integer(Self);',
  'end;',
  'var w: wordbool;',
  'begin',
  '  w.Run;',
  '  wordbool(3).Run;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_WordBool',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Run = function (e) {',
    '    var i = 0;',
    '    i = (this.get() ? 1 : 0);',
    '  };',
    '});',
    'this.w = false;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Run.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.w;',
    '    },',
    '  set: function (v) {',
    '      this.p.w = v;',
    '    }',
    '}, true);',
    '$mod.THelper.Run.call({',
    '  a: 3 != 0,',
    '  get: function () {',
    '      return this.a;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}, true);',
    '']));
end;

procedure TTestModule.TestTypeHelper_Double;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  Float = type double;',
  '  THelper = type helper for Float',
  '    const NPI = 3.141592;',
  '    function ToStr: String;',
  '  end;',
  'function THelper.ToStr: String;',
  'begin',
  'end;',
  'procedure DoIt(s: string);',
  'begin',
  'end;',
  'var f: Float;',
  'begin',
  '  DoIt(f.toStr);',
  '  DoIt(f.toStr());',
  '  (f*f).toStr;',
  '  DoIt((f*f).toStr);',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_Double',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.NPI = 3.141592;',
    '  this.ToStr = function () {',
    '    var Result = "";',
    '    return Result;',
    '  };',
    '});',
    'this.DoIt = function (s) {',
    '};',
    'this.f = 0.0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt($mod.THelper.ToStr.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.f;',
    '    },',
    '  set: function (v) {',
    '      this.p.f = v;',
    '    }',
    '}));',
    '$mod.DoIt($mod.THelper.ToStr.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.f;',
    '    },',
    '  set: function (v) {',
    '      this.p.f = v;',
    '    }',
    '}));',
    '$mod.THelper.ToStr.call({',
    '  a: $mod.f * $mod.f,',
    '  get: function () {',
    '      return this.a;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '});',
    '$mod.DoIt($mod.THelper.ToStr.call({',
    '  a: $mod.f * $mod.f,',
    '  get: function () {',
    '      return this.a;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}));',
    '']));
end;

procedure TTestModule.TestTypeHelper_NativeInt;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  MaxInt = type nativeint;',
  '  THelperI = type helper for MaxInt',
  '    function ToStr: String;',
  '  end;',
  '  MaxUInt = type nativeuint;',
  '  THelperU = type helper for MaxUInt',
  '    function ToStr: String;',
  '  end;',
  'function THelperI.ToStr: String;',
  'begin',
  '  Result:=str(Self);',
  'end;',
  'function THelperU.ToStr: String;',
  'begin',
  '  Result:=str(Self);',
  'end;',
  'procedure DoIt(s: string);',
  'begin',
  'end;',
  'var i: MaxInt;',
  'begin',
  '  DoIt(i.toStr);',
  '  DoIt(i.toStr());',
  '  (i*i).toStr;',
  '  DoIt((i*i).toStr);',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_NativeInt',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelperI", null, function () {',
    '  this.ToStr = function () {',
    '    var Result = "";',
    '    Result = "" + this.get();',
    '    return Result;',
    '  };',
    '});',
    'rtl.createHelper(this, "THelperU", null, function () {',
    '  this.ToStr = function () {',
    '    var Result = "";',
    '    Result = "" + this.get();',
    '    return Result;',
    '  };',
    '});',
    'this.DoIt = function (s) {',
    '};',
    'this.i = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt($mod.THelperI.ToStr.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '}));',
    '$mod.DoIt($mod.THelperI.ToStr.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '}));',
    '$mod.THelperI.ToStr.call({',
    '  a: $mod.i * $mod.i,',
    '  get: function () {',
    '      return this.a;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '});',
    '$mod.DoIt($mod.THelperI.ToStr.call({',
    '  a: $mod.i * $mod.i,',
    '  get: function () {',
    '      return this.a;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}));',
    '']));
end;

procedure TTestModule.TestTypeHelper_StringChar;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TStringHelper = type helper for string',
  '    procedure DoIt(e: byte = 123);',
  '  end;',
  '  TCharHelper = type helper for char',
  '    procedure Fly;',
  '  end;',
  'procedure TStringHelper.DoIt(e: byte);',
  'begin',
  '  Self[1]:=''c'';',
  '  Self[2]:=Self[3];',
  'end;',
  'procedure TCharHelper.Fly;',
  'begin',
  '  Self:=''c'';',
  'end;',
  'begin',
  '  ''abc''.DoIt;',
  '  ''xyz''.DoIt();',
  '  ''c''.Fly();',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_StringChar',
    LinesToStr([ // statements
    'rtl.createHelper(this, "TStringHelper", null, function () {',
    '  this.DoIt = function (e) {',
    '    this.set(rtl.setCharAt(this.get(), 0, "c"));',
    '    this.set(rtl.setCharAt(this.get(), 1, this.get().charAt(2)));',
    '  };',
    '});',
    'rtl.createHelper(this, "TCharHelper", null, function () {',
    '  this.Fly = function () {',
    '    this.set("c");',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.TStringHelper.DoIt.call({',
    '  get: function () {',
    '      return "abc";',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}, 123);',
    '$mod.TStringHelper.DoIt.call({',
    '  get: function () {',
    '      return "xyz";',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}, 123);',
    '$mod.TCharHelper.Fly.call({',
    '  get: function () {',
    '      return "c";',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '});',
    '']));
end;

procedure TTestModule.TestTypeHelper_JSValue;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TExtValue = type jsvalue;',
  '  THelper = type helper for TExtValue',
  '    function ToStr: String;',
  '  end;',
  'function THelper.ToStr: String;',
  'begin',
  'end;',
  'var',
  '  s: string;',
  '  v: TExtValue;',
  'begin',
  '  s:=v.toStr;',
  '  s:=v.toStr();',
  '  TExtValue(s).toStr;',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_JSValue',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.ToStr = function () {',
    '    var Result = "";',
    '    return Result;',
    '  };',
    '});',
    'this.s = "";',
    'this.v = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.s = $mod.THelper.ToStr.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.v;',
    '    },',
    '  set: function (v) {',
    '      this.p.v = v;',
    '    }',
    '});',
    '$mod.s = $mod.THelper.ToStr.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.v;',
    '    },',
    '  set: function (v) {',
    '      this.p.v = v;',
    '    }',
    '});',
    '$mod.THelper.ToStr.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.s;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '});',
    '']));
end;

procedure TTestModule.TestTypeHelper_Array;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TArrOfBool = array of boolean;',
  '  TArrOfJS = array of jsvalue;',
  '  THelper = type helper for TArrOfBool',
  '    procedure DoIt(e: byte = 123);',
  '  end;',
  'procedure THelper.DoIt(e: byte);',
  'begin',
  '  Self[1]:=true;',
  '  Self[2]:=not Self[3];',
  '  SetLength(Self,4);',
  'end;',
  'var',
  '  b: TArrOfBool;',
  '  j: TArrOfJS;',
  'begin',
  '  b.DoIt;',
  '  TArrOfBool(j).DoIt();',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_Array',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.DoIt = function (e) {',
    '    this.get()[1] = true;',
    '    this.get()[2] = !this.get()[3];',
    '    this.set(rtl.arraySetLength(this.get(), false, 4));',
    '  };',
    '});',
    'this.b = [];',
    'this.j = [];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.DoIt.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.b;',
    '    },',
    '  set: function (v) {',
    '      this.p.b = v;',
    '    }',
    '}, 123);',
    '$mod.THelper.DoIt.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.j;',
    '    },',
    '  set: function (v) {',
    '      this.p.j = v;',
    '    }',
    '}, 123);',
    '']));
end;

procedure TTestModule.TestTypeHelper_EnumType;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TEnum = (red,blue);',
  '  THelper = type helper for TEnum',
  '    procedure DoIt(e: byte = 123);',
  '    class procedure Swing(w: word); static;',
  '  end;',
  'procedure THelper.DoIt(e: byte);',
  'begin',
  '  Self:=red;',
  '  Self:=succ(Self);',
  '  with Self do Doit;',
  'end;',
  'class procedure THelper.Swing(w: word);',
  'begin',
  'end;',
  'var e: TEnum;',
  'begin',
  '  e.DoIt;',
  '  red.DoIt;',
  '  TEnum.blue.DoIt;',
  '  TEnum(1).DoIt;',
  '  TEnum.Swing(3);',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_EnumType',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.DoIt = function (e) {',
    '    this.set($mod.TEnum.red);',
    '    this.set(this.get() + 1);',
    '    var $with = this.get();',
    '    $mod.THelper.DoIt.call(this, 123);',
    '  };',
    '  this.Swing = function (w) {',
    '  };',
    '});',
    'this.e = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.DoIt.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.e;',
    '    },',
    '  set: function (v) {',
    '      this.p.e = v;',
    '    }',
    '}, 123);',
    '$mod.THelper.DoIt.call({',
    '  p: $mod.TEnum,',
    '  get: function () {',
    '      return this.p.red;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}, 123);',
    '$mod.THelper.DoIt.call({',
    '  p: $mod.TEnum,',
    '  get: function () {',
    '      return this.p.blue;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}, 123);',
    '$mod.THelper.DoIt.call({',
    '  get: function () {',
    '      return 1;',
    '    },',
    '  set: function (v) {',
    '      rtl.raiseE("EPropReadOnly");',
    '    }',
    '}, 123);',
    '$mod.THelper.Swing(3);',
    '']));
end;

procedure TTestModule.TestTypeHelper_SetType;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  TEnum = (red,blue);',
  '  TSetOfEnum = set of TEnum;',
  '  THelper = type helper for TSetOfEnum',
  '    procedure DoIt(e: byte = 123);',
  '    constructor Init(e: TEnum);',
  '    constructor InitEmpty;',
  '  end;',
  'procedure THelper.DoIt(e: byte);',
  'begin',
  '  Self:=[];',
  '  Self:=[red];',
  '  Include(Self,blue);',
  'end;',
  'constructor THelper.Init(e: TEnum);',
  'begin',
  '  Self:=[];',
  '  Self:=[e];',
  '  Include(Self,blue);',
  'end;',
  'constructor THelper.InitEmpty;',
  'begin',
  'end;',
  'var s: TSetOfEnum;',
  'begin',
  '  s.DoIt;',
  //'  [red].DoIt;',
  //'  with s do DoIt;',
  //'  with [red,blue] do DoIt;',
  '  s:=TSetOfEnum.Init(blue);',
  '  s:=s.Init(blue);',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_SetType',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.DoIt = function (e) {',
    '    this.set({});',
    '    this.set(rtl.createSet($mod.TEnum.red));',
    '    this.set(rtl.includeSet(this.get(), $mod.TEnum.blue));',
    '  };',
    '  this.Init = function (e) {',
    '    this.set({});',
    '    this.set(rtl.createSet(e));',
    '    this.set(rtl.includeSet(this.get(), $mod.TEnum.blue));',
    '    return this.get();',
    '  };',
    '  this.InitEmpty = function () {',
    '    return this.get();',
    '  };',
    '  this.$new = function (fn, args) {',
    '    return this[fn].apply({',
    '      p: {},',
    '      get: function () {',
    '          return this.p;',
    '        },',
    '      set: function (v) {',
    '          this.p = v;',
    '        }',
    '    }, args);',
    '  };',
    '});',
    'this.s = {};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.DoIt.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.s;',
    '    },',
    '  set: function (v) {',
    '      this.p.s = v;',
    '    }',
    '}, 123);',
    '$mod.s = rtl.refSet($mod.THelper.$new("Init", [$mod.TEnum.blue]));',
    '$mod.s = rtl.refSet($mod.THelper.Init.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.s;',
    '    },',
    '  set: function (v) {',
    '      this.p.s = v;',
    '    }',
    '}, $mod.TEnum.blue));',
    '']));
end;

procedure TTestModule.TestTypeHelper_InterfaceType;
begin
  StartProgram(false);
  Add([
  '{$interfaces com}',
  '{$modeswitch typehelpers}',
  'type',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  '  TObject = class(IUnknown)',
  '    function _AddRef: longint; virtual; abstract;',
  '    function _Release: longint; virtual; abstract;',
  '  end;',
  '  THelper = type helper for IUnknown',
  '    procedure Fly(e: byte = 123);',
  '    class procedure Run; static;',
  '  end;',
  'var',
  '  i: IUnknown;',
  '  o: TObject;',
  'procedure THelper.Fly(e: byte);',
  'begin',
  '  i:=Self;',
  '  o:=Self as TObject;',
  '  Self:=nil;',
  '  Self:=i;',
  '  Self:=o;',
  '  with Self do begin',
  '    Fly;',
  '    Fly();',
  '  end;',
  'end;',
  'class procedure THelper.Run;',
  'var l: IUnknown;',
  'begin',
  '  l.Fly;',
  '  l.Fly();',
  'end;',
  'begin',
  '  i.Fly;',
  '  i.Fly();',
  '  i.Run;',
  '  i.Run();',
  '  IUnknown.Run;',
  '  IUnknown.Run();',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_InterfaceType',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  rtl.addIntf(this, $mod.IUnknown);',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Fly = function (e) {',
    '    var $ir = rtl.createIntfRefs();',
    '    try {',
    '      rtl.setIntfP($mod, "i", this.get());',
    '      $mod.o = rtl.intfAsClass(this.get(), $mod.TObject);',
    '      this.set(null);',
    '      this.set($mod.i);',
    '      this.set($ir.ref(1, rtl.queryIntfT($mod.o, $mod.IUnknown)));',
    '      var $with = this.get();',
    '      $mod.THelper.Fly.call(this, 123);',
    '      $mod.THelper.Fly.call(this, 123);',
    '    } finally {',
    '      $ir.free();',
    '    };',
    '  };',
    '  this.Run = function () {',
    '    var l = null;',
    '    try {',
    '      $mod.THelper.Fly.call({',
    '        get: function () {',
    '            return l;',
    '          },',
    '        set: function (v) {',
    '            l = rtl.setIntfL(l, v);',
    '          }',
    '      }, 123);',
    '      $mod.THelper.Fly.call({',
    '        get: function () {',
    '            return l;',
    '          },',
    '        set: function (v) {',
    '            l = rtl.setIntfL(l, v);',
    '          }',
    '      }, 123);',
    '    } finally {',
    '      rtl._Release(l);',
    '    };',
    '  };',
    '});',
    'this.i = null;',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.Fly.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      rtl.setIntfP(this.p, "i", v);',
    '    }',
    '}, 123);',
    '$mod.THelper.Fly.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      rtl.setIntfP(this.p, "i", v);',
    '    }',
    '}, 123);',
    '$mod.THelper.Run();',
    '$mod.THelper.Run();',
    '$mod.THelper.Run();',
    '$mod.THelper.Run();',
    '']));
end;

procedure TTestModule.TestTypeHelper_NestedSelf;
begin
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  'type',
  '  THelper = type helper for string',
  '    procedure Run(Value: string);',
  '  end;',
  'procedure THelper.Run(Value: string);',
  '  function Sub(i: nativeint): boolean;',
  '  begin',
  '    Result:=Self[i+1]=Value[i];',
  '  end;',
  'begin',
  '  if Self[3]=Value[4] then ;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestTypeHelper_NestedSelf',
    LinesToStr([ // statements
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.Run = function (Value) {',
    '    var $Self = this;',
    '    function Sub(i) {',
    '      var Result = false;',
    '      Result = $Self.get().charAt((i + 1) - 1) === Value.charAt(i - 1);',
    '      return Result;',
    '    };',
    '    if ($Self.get().charAt(2) === Value.charAt(3)) ;',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestProcType;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProcInt = procedure(vI: longint = 1);',
  'procedure DoIt(vJ: longint);',
  'begin end;',
  'var',
  '  b: boolean;',
  '  vP, vQ: tprocint;',
  'begin',
  '  vp:=nil;',
  '  vp:=vp;',
  '  vp:=@doit;',
  '  vp;',
  '  vp();',
  '  vp(2);',
  '  b:=vp=nil;',
  '  b:=nil=vp;',
  '  b:=vp=vq;',
  '  b:=vp=@doit;',
  '  b:=@doit=vp;',
  '  b:=vp<>nil;',
  '  b:=nil<>vp;',
  '  b:=vp<>vq;',
  '  b:=vp<>@doit;',
  '  b:=@doit<>vp;',
  '  b:=Assigned(vp);',
  '  if Assigned(vp) then ;']);
  ConvertProgram;
  CheckSource('TestProcType',
    LinesToStr([ // statements
    'this.DoIt = function(vJ) {',
    '};',
    'this.b = false;',
    'this.vP = null;',
    'this.vQ = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.vP = null;',
    '$mod.vP = $mod.vP;',
    '$mod.vP = $mod.DoIt;',
    '$mod.vP(1);',
    '$mod.vP(1);',
    '$mod.vP(2);',
    '$mod.b = $mod.vP === null;',
    '$mod.b = null === $mod.vP;',
    '$mod.b = rtl.eqCallback($mod.vP,$mod.vQ);',
    '$mod.b = rtl.eqCallback($mod.vP, $mod.DoIt);',
    '$mod.b = rtl.eqCallback($mod.DoIt, $mod.vP);',
    '$mod.b = $mod.vP !== null;',
    '$mod.b = null !== $mod.vP;',
    '$mod.b = !rtl.eqCallback($mod.vP,$mod.vQ);',
    '$mod.b = !rtl.eqCallback($mod.vP, $mod.DoIt);',
    '$mod.b = !rtl.eqCallback($mod.DoIt, $mod.vP);',
    '$mod.b = $mod.vP != null;',
    'if ($mod.vP != null) ;',
    '']));
end;

procedure TTestModule.TestProcType_Arg;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProcInt = procedure(vI: longint = 1);',
  'procedure DoIt(vJ: longint); begin end;',
  'procedure DoSome(vP, vQ: TProcInt);',
  'var',
  '  b: boolean;',
  'begin',
  '  vp:=nil;',
  '  vp:=vp;',
  '  vp:=@doit;',
  '  vp;',
  '  vp();',
  '  vp(2);',
  '  b:=vp=nil;',
  '  b:=nil=vp;',
  '  b:=vp=vq;',
  '  b:=vp=@doit;',
  '  b:=@doit=vp;',
  '  b:=vp<>nil;',
  '  b:=nil<>vp;',
  '  b:=vp<>vq;',
  '  b:=vp<>@doit;',
  '  b:=@doit<>vp;',
  '  b:=Assigned(vp);',
  '  if Assigned(vp) then ;',
  'end;',
  'begin',
  '  DoSome(@DoIt,nil);']);
  ConvertProgram;
  CheckSource('TestProcType_Arg',
    LinesToStr([ // statements
    'this.DoIt = function(vJ) {',
    '};',
    'this.DoSome = function(vP, vQ) {',
    '  var b = false;',
    '  vP = null;',
    '  vP = vP;',
    '  vP = $mod.DoIt;',
    '  vP(1);',
    '  vP(1);',
    '  vP(2);',
    '  b = vP === null;',
    '  b = null === vP;',
    '  b = rtl.eqCallback(vP,vQ);',
    '  b = rtl.eqCallback(vP, $mod.DoIt);',
    '  b = rtl.eqCallback($mod.DoIt, vP);',
    '  b = vP !== null;',
    '  b = null !== vP;',
    '  b = !rtl.eqCallback(vP, vQ);',
    '  b = !rtl.eqCallback(vP, $mod.DoIt);',
    '  b = !rtl.eqCallback($mod.DoIt, vP);',
    '  b = vP != null;',
    '  if (vP != null) ;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoSome($mod.DoIt,null);',
    '']));
end;

procedure TTestModule.TestProcType_FunctionFPC;
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
  ConvertProgram;
  CheckSource('TestProcType_FunctionFPC',
    LinesToStr([ // statements
    'this.DoIt = function(vI) {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    'this.b = false;',
    'this.vP = null;',
    'this.vQ = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.vP = null;',
    '$mod.vP = $mod.vP;',
    '$mod.vP = $mod.DoIt;',
    '$mod.vP(1);',
    '$mod.vP(1);',
    '$mod.vP(2);',
    '$mod.b = $mod.vP === null;',
    '$mod.b = null === $mod.vP;',
    '$mod.b = rtl.eqCallback($mod.vP,$mod.vQ);',
    '$mod.b = rtl.eqCallback($mod.vP, $mod.DoIt);',
    '$mod.b = rtl.eqCallback($mod.DoIt, $mod.vP);',
    '$mod.b = 4 === $mod.vP(1);',
    '$mod.b = $mod.vP !== null;',
    '$mod.b = null !== $mod.vP;',
    '$mod.b = !rtl.eqCallback($mod.vP,$mod.vQ);',
    '$mod.b = !rtl.eqCallback($mod.vP, $mod.DoIt);',
    '$mod.b = !rtl.eqCallback($mod.DoIt, $mod.vP);',
    '$mod.b = 6 !== $mod.vP(1);',
    '$mod.b = $mod.vP != null;',
    '$mod.DoIt($mod.vP(1));',
    '$mod.DoIt($mod.vP(2));',
    '']));
end;

procedure TTestModule.TestProcType_FunctionDelphi;
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
  ConvertProgram;
  CheckSource('TestProcType_FunctionDelphi',
    LinesToStr([ // statements
    'this.DoIt = function(vI) {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    'this.b = false;',
    'this.vP = null;',
    'this.vQ = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.vP = null;',
    '$mod.vP = $mod.vP;',
    '$mod.vP = $mod.DoIt;',
    '$mod.vP = $mod.DoIt;',
    '$mod.vP(1);',
    '$mod.vP(1);',
    '$mod.vP(2);',
    '$mod.b = $mod.vP(1) === $mod.vQ(1);',
    '$mod.b = $mod.vP(1) === 3;',
    '$mod.b = 4 === $mod.vP(1);',
    '$mod.b = $mod.vP(1) !== $mod.vQ(1);',
    '$mod.b = $mod.vP(1) !== 5;',
    '$mod.b = 6 !== $mod.vP(1);',
    '$mod.b = $mod.vP != null;',
    '$mod.DoIt($mod.vP(1));',
    '$mod.DoIt($mod.vP(1));',
    '$mod.DoIt($mod.vP(2));',
    '']));
end;

procedure TTestModule.TestProcType_ProcedureDelphi;
begin
  StartProgram(false);
  Add('{$mode Delphi}');
  Add('type');
  Add('  TProc = procedure;');
  Add('procedure DoIt;');
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

  ConvertProgram;
  CheckSource('TestProcType_ProcedureDelphi',
    LinesToStr([ // statements
    'this.DoIt = function() {',
    '};',
    'this.b = false;',
    'this.vP = null;',
    'this.vQ = null;'
    ]),
    LinesToStr([ // $mod.$main
    '$mod.vP = null;',
    '$mod.vP = $mod.vP;',
    '$mod.vP = $mod.vQ;',
    '$mod.vP = $mod.DoIt;',
    '$mod.vP = $mod.DoIt;',
    '$mod.vP();',
    '$mod.vP();',
    '$mod.b = $mod.vP === null;',
    '$mod.b = null === $mod.vP;',
    '$mod.b = rtl.eqCallback($mod.vP, $mod.vQ);',
    '$mod.b = rtl.eqCallback($mod.vP, $mod.DoIt);',
    '$mod.b = rtl.eqCallback($mod.DoIt, $mod.vP);',
    '$mod.b = $mod.vP !== null;',
    '$mod.b = null !== $mod.vP;',
    '$mod.b = !rtl.eqCallback($mod.vP, $mod.vQ);',
    '$mod.b = !rtl.eqCallback($mod.vP, $mod.DoIt);',
    '$mod.b = !rtl.eqCallback($mod.DoIt, $mod.vP);',
    '$mod.b = $mod.vP != null;',
    '']));
end;

procedure TTestModule.TestProcType_AsParam;
begin
  StartProgram(false);
  Add('type');
  Add('  TFuncInt = function(vA: longint = 1): longint;');
  Add('procedure DoIt(vG: tfuncint; const vH: tfuncint; var vI: tfuncint);');
  Add('var vJ: tfuncint;');
  Add('begin');
  Add('  vg:=vg;');
  Add('  vj:=vh;');
  Add('  vi:=vi;');
  Add('  doit(vg,vg,vg);');
  Add('  doit(vh,vh,vj);');
  Add('  doit(vi,vi,vi);');
  Add('  doit(vj,vj,vj);');
  Add('end;');
  Add('var i: tfuncint;');
  Add('begin');
  Add('  doit(i,i,i);');
  ConvertProgram;
  CheckSource('TestProcType_AsParam',
    LinesToStr([ // statements
    'this.DoIt = function (vG,vH,vI) {',
    '  var vJ = null;',
    '  vG = vG;',
    '  vJ = vH;',
    '  vI.set(vI.get());',
    '  $mod.DoIt(vG, vG, {',
    '    get: function () {',
    '      return vG;',
    '    },',
    '    set: function (v) {',
    '      vG = v;',
    '    }',
    '  });',
    '  $mod.DoIt(vH, vH, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '  $mod.DoIt(vI.get(), vI.get(), vI);',
    '  $mod.DoIt(vJ, vJ, {',
    '    get: function () {',
    '      return vJ;',
    '    },',
    '    set: function (v) {',
    '      vJ = v;',
    '    }',
    '  });',
    '};',
    'this.i = null;'
    ]),
    LinesToStr([
    '$mod.DoIt($mod.i,$mod.i,{',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '});'
    ]));
end;

procedure TTestModule.TestProcType_MethodFPC;
begin
  StartProgram(false);
  Add('type');
  Add('  TFuncInt = function(vA: longint = 1): longint of object;');
  Add('  TObject = class');
  Add('    function DoIt(vA: longint = 1): longint;');
  Add('  end;');
  Add('function TObject.DoIt(vA: longint = 1): longint;');
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
  ConvertProgram;
  CheckSource('TestProcType_MethodFPC',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (vA) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;',
    'this.vP = null;',
    'this.b = false;'
    ]),
    LinesToStr([
    '$mod.vP = rtl.createCallback($mod.Obj, "DoIt");',
    '$mod.vP(1);',
    '$mod.vP(1);',
    '$mod.vP(2);',
    '$mod.b = rtl.eqCallback($mod.vP, rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.b = rtl.eqCallback(rtl.createCallback($mod.Obj, "DoIt"), $mod.vP);',
    '$mod.b = !rtl.eqCallback($mod.vP, rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.b = !rtl.eqCallback(rtl.createCallback($mod.Obj, "DoIt"), $mod.vP);',
    '']));
end;

procedure TTestModule.TestProcType_MethodDelphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  'type',
  '  TFuncInt = function(vA: longint = 1): longint of object;',
  '  TObject = class',
  '    function DoIt(vA: longint = 1): longint;',
  '  end;',
  'function TObject.DoIt(vA: longint = 1): longint;',
  'begin',
  'end;',
  'var',
  '  Obj: TObject;',
  '  vP: tfuncint;',
  '  b: boolean;',
  'begin',
  '  vp:=@obj.doit;', // ok in fpc and delphi
  '  vp:=obj.doit;', // illegal in fpc, ok in delphi
  '  vp;', // ok in fpc and delphi
  '  vp();',
  '  vp(2);',
  //'  b:=vp=@obj.doit;', // ok in fpc, illegal in delphi
  //'  b:=@obj.doit=vp;', // ok in fpc, illegal in delphi
  //'  b:=vp<>@obj.doit;', // ok in fpc, illegal in delphi
  //'  b:=@obj.doit<>vp;'); // ok in fpc, illegal in delphi
  '']);
  ConvertProgram;
  CheckSource('TestProcType_MethodDelphi',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (vA) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;',
    'this.vP = null;',
    'this.b = false;'
    ]),
    LinesToStr([
    '$mod.vP = rtl.createCallback($mod.Obj, "DoIt");',
    '$mod.vP = rtl.createCallback($mod.Obj, "DoIt");',
    '$mod.vP(1);',
    '$mod.vP(1);',
    '$mod.vP(2);',
    '']));
end;

procedure TTestModule.TestProcType_PropertyFPC;
begin
  StartProgram(false);
  Add('type');
  Add('  TFuncInt = function(vA: longint = 1): longint of object;');
  Add('  TObject = class');
  Add('    FOnFoo: TFuncInt;');
  Add('    function DoIt(vA: longint = 1): longint;');
  Add('    function GetFoo: TFuncInt;');
  Add('    procedure SetFoo(const Value: TFuncInt);');
  Add('    function GetEvents(Index: longint): TFuncInt;');
  Add('    procedure SetEvents(Index: longint; const Value: TFuncInt);');
  Add('    property OnFoo: TFuncInt read FOnFoo write FOnFoo;');
  Add('    property OnBar: TFuncInt read GetFoo write SetFoo;');
  Add('    property Events[Index: longint]: TFuncInt read GetEvents write SetEvents; default;');
  Add('  end;');
  Add('function tobject.doit(va: longint = 1): longint; begin end;');
  Add('function tobject.getfoo: tfuncint; begin end;');
  Add('procedure tobject.setfoo(const value: tfuncint); begin end;');
  Add('function tobject.getevents(index: longint): tfuncint; begin end;');
  Add('procedure tobject.setevents(index: longint; const value: tfuncint); begin end;');
  Add('var');
  Add('  Obj: TObject;');
  Add('  vP: tfuncint;');
  Add('  b: boolean;');
  Add('begin');
  Add('  obj.onfoo:=nil;');
  Add('  obj.onbar:=nil;');
  Add('  obj.events[1]:=nil;');
  Add('  obj.onfoo:=obj.onfoo;');
  Add('  obj.onbar:=obj.onbar;');
  Add('  obj.events[2]:=obj.events[3];');
  Add('  obj.onfoo:=@obj.doit;');
  Add('  obj.onbar:=@obj.doit;');
  Add('  obj.events[4]:=@obj.doit;');
  //Add('  obj.onfoo:=obj.doit;'); // delphi
  //Add('  obj.onbar:=obj.doit;'); // delphi
  //Add('  obj.events[4]:=obj.doit;'); // delphi
  Add('  obj.onfoo;');
  Add('  obj.onbar;');
  //Add('  obj.events[5];'); ToDo in pasresolver
  Add('  obj.onfoo();');
  Add('  obj.onbar();');
  Add('  obj.events[6]();');
  Add('  b:=obj.onfoo=nil;');
  Add('  b:=obj.onbar=nil;');
  Add('  b:=obj.events[7]=nil;');
  Add('  b:=obj.onfoo<>nil;');
  Add('  b:=obj.onbar<>nil;');
  Add('  b:=obj.events[8]<>nil;');
  Add('  b:=obj.onfoo=vp;');
  Add('  b:=obj.onbar=vp;');
  Add('  b:=obj.events[9]=vp;');
  Add('  b:=obj.onfoo=obj.onfoo;');
  Add('  b:=obj.onbar=obj.onfoo;');
  Add('  b:=obj.events[10]=obj.onfoo;');
  Add('  b:=obj.onfoo<>obj.onfoo;');
  Add('  b:=obj.onbar<>obj.onfoo;');
  Add('  b:=obj.events[11]<>obj.onfoo;');
  Add('  b:=obj.onfoo=@obj.doit;');
  Add('  b:=obj.onbar=@obj.doit;');
  Add('  b:=obj.events[12]=@obj.doit;');
  Add('  b:=obj.onfoo<>@obj.doit;');
  Add('  b:=obj.onbar<>@obj.doit;');
  Add('  b:=obj.events[12]<>@obj.doit;');
  Add('  b:=Assigned(obj.onfoo);');
  Add('  b:=Assigned(obj.onbar);');
  Add('  b:=Assigned(obj.events[13]);');
  ConvertProgram;
  CheckSource('TestProcType_PropertyFPC',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FOnFoo = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FOnFoo = undefined;',
    '  };',
    '  this.DoIt = function (vA) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    'this.GetFoo = function () {',
    '  var Result = null;',
    '  return Result;',
    '};',
    'this.SetFoo = function (Value) {',
    '};',
    'this.GetEvents = function (Index) {',
    '  var Result = null;',
    '  return Result;',
    '};',
    'this.SetEvents = function (Index, Value) {',
    '};',
    '});',
    'this.Obj = null;',
    'this.vP = null;',
    'this.b = false;'
    ]),
    LinesToStr([
    '$mod.Obj.FOnFoo = null;',
    '$mod.Obj.SetFoo(null);',
    '$mod.Obj.SetEvents(1, null);',
    '$mod.Obj.FOnFoo = $mod.Obj.FOnFoo;',
    '$mod.Obj.SetFoo($mod.Obj.GetFoo());',
    '$mod.Obj.SetEvents(2, $mod.Obj.GetEvents(3));',
    '$mod.Obj.FOnFoo = rtl.createCallback($mod.Obj, "DoIt");',
    '$mod.Obj.SetFoo(rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.Obj.SetEvents(4, rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.Obj.FOnFoo(1);',
    '$mod.Obj.GetFoo();',
    '$mod.Obj.FOnFoo(1);',
    '$mod.Obj.GetFoo()(1);',
    '$mod.Obj.GetEvents(6)(1);',
    '$mod.b = $mod.Obj.FOnFoo === null;',
    '$mod.b = $mod.Obj.GetFoo() === null;',
    '$mod.b = $mod.Obj.GetEvents(7) === null;',
    '$mod.b = $mod.Obj.FOnFoo !== null;',
    '$mod.b = $mod.Obj.GetFoo() !== null;',
    '$mod.b = $mod.Obj.GetEvents(8) !== null;',
    '$mod.b = rtl.eqCallback($mod.Obj.FOnFoo, $mod.vP);',
    '$mod.b = rtl.eqCallback($mod.Obj.GetFoo(), $mod.vP);',
    '$mod.b = rtl.eqCallback($mod.Obj.GetEvents(9), $mod.vP);',
    '$mod.b = rtl.eqCallback($mod.Obj.FOnFoo, $mod.Obj.FOnFoo);',
    '$mod.b = rtl.eqCallback($mod.Obj.GetFoo(), $mod.Obj.FOnFoo);',
    '$mod.b = rtl.eqCallback($mod.Obj.GetEvents(10), $mod.Obj.FOnFoo);',
    '$mod.b = !rtl.eqCallback($mod.Obj.FOnFoo, $mod.Obj.FOnFoo);',
    '$mod.b = !rtl.eqCallback($mod.Obj.GetFoo(), $mod.Obj.FOnFoo);',
    '$mod.b = !rtl.eqCallback($mod.Obj.GetEvents(11), $mod.Obj.FOnFoo);',
    '$mod.b = rtl.eqCallback($mod.Obj.FOnFoo, rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.b = rtl.eqCallback($mod.Obj.GetFoo(), rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.b = rtl.eqCallback($mod.Obj.GetEvents(12), rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.b = !rtl.eqCallback($mod.Obj.FOnFoo, rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.b = !rtl.eqCallback($mod.Obj.GetFoo(), rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.b = !rtl.eqCallback($mod.Obj.GetEvents(12), rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.b = $mod.Obj.FOnFoo != null;',
    '$mod.b = $mod.Obj.GetFoo() != null;',
    '$mod.b = $mod.Obj.GetEvents(13) != null;',
    '']));
end;

procedure TTestModule.TestProcType_PropertyDelphi;
begin
  StartProgram(false);
  Add('{$mode delphi}');
  Add('type');
  Add('  TFuncInt = function(vA: longint = 1): longint of object;');
  Add('  TObject = class');
  Add('    FOnFoo: TFuncInt;');
  Add('    function DoIt(vA: longint = 1): longint;');
  Add('    function GetFoo: TFuncInt;');
  Add('    procedure SetFoo(const Value: TFuncInt);');
  Add('    function GetEvents(Index: longint): TFuncInt;');
  Add('    procedure SetEvents(Index: longint; const Value: TFuncInt);');
  Add('    property OnFoo: TFuncInt read FOnFoo write FOnFoo;');
  Add('    property OnBar: TFuncInt read GetFoo write SetFoo;');
  Add('    property Events[Index: longint]: TFuncInt read GetEvents write SetEvents; default;');
  Add('  end;');
  Add('function tobject.doit(va: longint = 1): longint; begin end;');
  Add('function tobject.getfoo: tfuncint; begin end;');
  Add('procedure tobject.setfoo(const value: tfuncint); begin end;');
  Add('function tobject.getevents(index: longint): tfuncint; begin end;');
  Add('procedure tobject.setevents(index: longint; const value: tfuncint); begin end;');
  Add('var');
  Add('  Obj: TObject;');
  Add('  vP: tfuncint;');
  Add('  b: boolean;');
  Add('begin');
  Add('  obj.onfoo:=nil;');
  Add('  obj.onbar:=nil;');
  Add('  obj.events[1]:=nil;');
  Add('  obj.onfoo:=obj.onfoo;');
  Add('  obj.onbar:=obj.onbar;');
  Add('  obj.events[2]:=obj.events[3];');
  Add('  obj.onfoo:=@obj.doit;');
  Add('  obj.onbar:=@obj.doit;');
  Add('  obj.events[4]:=@obj.doit;');
  Add('  obj.onfoo:=obj.doit;'); // delphi
  Add('  obj.onbar:=obj.doit;'); // delphi
  Add('  obj.events[4]:=obj.doit;'); // delphi
  Add('  obj.onfoo;');
  Add('  obj.onbar;');
  //Add('  obj.events[5];'); ToDo in pasresolver
  Add('  obj.onfoo();');
  Add('  obj.onbar();');
  Add('  obj.events[6]();');
  //Add('  b:=obj.onfoo=nil;'); // fpc
  //Add('  b:=obj.onbar=nil;'); // fpc
  //Add('  b:=obj.events[7]=nil;'); // fpc
  //Add('  b:=obj.onfoo<>nil;'); // fpc
  //Add('  b:=obj.onbar<>nil;'); // fpc
  //Add('  b:=obj.events[8]<>nil;'); // fpc
  Add('  b:=obj.onfoo=vp;');
  Add('  b:=obj.onbar=vp;');
  //Add('  b:=obj.events[9]=vp;'); ToDo in pasresolver
  Add('  b:=obj.onfoo=obj.onfoo;');
  Add('  b:=obj.onbar=obj.onfoo;');
  //Add('  b:=obj.events[10]=obj.onfoo;'); // ToDo in pasresolver
  Add('  b:=obj.onfoo<>obj.onfoo;');
  Add('  b:=obj.onbar<>obj.onfoo;');
  //Add('  b:=obj.events[11]<>obj.onfoo;'); // ToDo in pasresolver
  //Add('  b:=obj.onfoo=@obj.doit;'); // fpc
  //Add('  b:=obj.onbar=@obj.doit;'); // fpc
  //Add('  b:=obj.events[12]=@obj.doit;'); // fpc
  //Add('  b:=obj.onfoo<>@obj.doit;'); // fpc
  //Add('  b:=obj.onbar<>@obj.doit;'); // fpc
  //Add('  b:=obj.events[12]<>@obj.doit;'); // fpc
  Add('  b:=Assigned(obj.onfoo);');
  Add('  b:=Assigned(obj.onbar);');
  Add('  b:=Assigned(obj.events[13]);');
  ConvertProgram;
  CheckSource('TestProcType_PropertyDelphi',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FOnFoo = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FOnFoo = undefined;',
    '  };',
    '  this.DoIt = function (vA) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    'this.GetFoo = function () {',
    '  var Result = null;',
    '  return Result;',
    '};',
    'this.SetFoo = function (Value) {',
    '};',
    'this.GetEvents = function (Index) {',
    '  var Result = null;',
    '  return Result;',
    '};',
    'this.SetEvents = function (Index, Value) {',
    '};',
    '});',
    'this.Obj = null;',
    'this.vP = null;',
    'this.b = false;'
    ]),
    LinesToStr([
    '$mod.Obj.FOnFoo = null;',
    '$mod.Obj.SetFoo(null);',
    '$mod.Obj.SetEvents(1, null);',
    '$mod.Obj.FOnFoo = $mod.Obj.FOnFoo;',
    '$mod.Obj.SetFoo($mod.Obj.GetFoo());',
    '$mod.Obj.SetEvents(2, $mod.Obj.GetEvents(3));',
    '$mod.Obj.FOnFoo = rtl.createCallback($mod.Obj, "DoIt");',
    '$mod.Obj.SetFoo(rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.Obj.SetEvents(4, rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.Obj.FOnFoo = rtl.createCallback($mod.Obj, "DoIt");',
    '$mod.Obj.SetFoo(rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.Obj.SetEvents(4, rtl.createCallback($mod.Obj, "DoIt"));',
    '$mod.Obj.FOnFoo(1);',
    '$mod.Obj.GetFoo();',
    '$mod.Obj.FOnFoo(1);',
    '$mod.Obj.GetFoo()(1);',
    '$mod.Obj.GetEvents(6)(1);',
    '$mod.b = $mod.Obj.FOnFoo(1) === $mod.vP(1);',
    '$mod.b = $mod.Obj.GetFoo() === $mod.vP(1);',
    '$mod.b = $mod.Obj.FOnFoo(1) === $mod.Obj.FOnFoo(1);',
    '$mod.b = $mod.Obj.GetFoo() === $mod.Obj.FOnFoo(1);',
    '$mod.b = $mod.Obj.FOnFoo(1) !== $mod.Obj.FOnFoo(1);',
    '$mod.b = $mod.Obj.GetFoo() !== $mod.Obj.FOnFoo(1);',
    '$mod.b = $mod.Obj.FOnFoo != null;',
    '$mod.b = $mod.Obj.GetFoo() != null;',
    '$mod.b = $mod.Obj.GetEvents(13) != null;',
    '']));
end;

procedure TTestModule.TestProcType_WithClassInstDoPropertyFPC;
begin
  StartProgram(false);
  Add('type');
  Add('  TFuncInt = function(vA: longint = 1): longint of object;');
  Add('  TObject = class');
  Add('    FOnFoo: TFuncInt;');
  Add('    function DoIt(vA: longint = 1): longint;');
  Add('    function GetFoo: TFuncInt;');
  Add('    procedure SetFoo(const Value: TFuncInt);');
  Add('    property OnFoo: TFuncInt read FOnFoo write FOnFoo;');
  Add('    property OnBar: TFuncInt read GetFoo write SetFoo;');
  Add('  end;');
  Add('function tobject.doit(va: longint = 1): longint; begin end;');
  Add('function tobject.getfoo: tfuncint; begin end;');
  Add('procedure tobject.setfoo(const value: tfuncint); begin end;');
  Add('var');
  Add('  Obj: TObject;');
  Add('  vP: tfuncint;');
  Add('  b: boolean;');
  Add('begin');
  Add('with obj do begin');
  Add('  fonfoo:=nil;');
  Add('  onfoo:=nil;');
  Add('  onbar:=nil;');
  Add('  fonfoo:=fonfoo;');
  Add('  onfoo:=onfoo;');
  Add('  onbar:=onbar;');
  Add('  fonfoo:=@doit;');
  Add('  onfoo:=@doit;');
  Add('  onbar:=@doit;');
  //Add('  fonfoo:=doit;'); // delphi
  //Add('  onfoo:=doit;'); // delphi
  //Add('  onbar:=doit;'); // delphi
  Add('  fonfoo;');
  Add('  onfoo;');
  Add('  onbar;');
  Add('  fonfoo();');
  Add('  onfoo();');
  Add('  onbar();');
  Add('  b:=fonfoo=nil;');
  Add('  b:=onfoo=nil;');
  Add('  b:=onbar=nil;');
  Add('  b:=fonfoo<>nil;');
  Add('  b:=onfoo<>nil;');
  Add('  b:=onbar<>nil;');
  Add('  b:=fonfoo=vp;');
  Add('  b:=onfoo=vp;');
  Add('  b:=onbar=vp;');
  Add('  b:=fonfoo=fonfoo;');
  Add('  b:=onfoo=onfoo;');
  Add('  b:=onbar=onfoo;');
  Add('  b:=fonfoo<>fonfoo;');
  Add('  b:=onfoo<>onfoo;');
  Add('  b:=onbar<>onfoo;');
  Add('  b:=fonfoo=@doit;');
  Add('  b:=onfoo=@doit;');
  Add('  b:=onbar=@doit;');
  Add('  b:=fonfoo<>@doit;');
  Add('  b:=onfoo<>@doit;');
  Add('  b:=onbar<>@doit;');
  Add('  b:=Assigned(fonfoo);');
  Add('  b:=Assigned(onfoo);');
  Add('  b:=Assigned(onbar);');
  Add('end;');
  ConvertProgram;
  CheckSource('TestProcType_WithClassInstDoPropertyFPC',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FOnFoo = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FOnFoo = undefined;',
    '  };',
    '  this.DoIt = function (vA) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  this.GetFoo = function () {',
    '    var Result = null;',
    '    return Result;',
    '  };',
    '  this.SetFoo = function (Value) {',
    '  };',
    '});',
    'this.Obj = null;',
    'this.vP = null;',
    'this.b = false;'
    ]),
    LinesToStr([
    'var $with = $mod.Obj;',
    '$with.FOnFoo = null;',
    '$with.FOnFoo = null;',
    '$with.SetFoo(null);',
    '$with.FOnFoo = $with.FOnFoo;',
    '$with.FOnFoo = $with.FOnFoo;',
    '$with.SetFoo($with.GetFoo());',
    '$with.FOnFoo = rtl.createCallback($with, "DoIt");',
    '$with.FOnFoo = rtl.createCallback($with, "DoIt");',
    '$with.SetFoo(rtl.createCallback($with, "DoIt"));',
    '$with.FOnFoo(1);',
    '$with.FOnFoo(1);',
    '$with.GetFoo();',
    '$with.FOnFoo(1);',
    '$with.FOnFoo(1);',
    '$with.GetFoo()(1);',
    '$mod.b = $with.FOnFoo === null;',
    '$mod.b = $with.FOnFoo === null;',
    '$mod.b = $with.GetFoo() === null;',
    '$mod.b = $with.FOnFoo !== null;',
    '$mod.b = $with.FOnFoo !== null;',
    '$mod.b = $with.GetFoo() !== null;',
    '$mod.b = rtl.eqCallback($with.FOnFoo, $mod.vP);',
    '$mod.b = rtl.eqCallback($with.FOnFoo, $mod.vP);',
    '$mod.b = rtl.eqCallback($with.GetFoo(), $mod.vP);',
    '$mod.b = rtl.eqCallback($with.FOnFoo, $with.FOnFoo);',
    '$mod.b = rtl.eqCallback($with.FOnFoo, $with.FOnFoo);',
    '$mod.b = rtl.eqCallback($with.GetFoo(), $with.FOnFoo);',
    '$mod.b = !rtl.eqCallback($with.FOnFoo, $with.FOnFoo);',
    '$mod.b = !rtl.eqCallback($with.FOnFoo, $with.FOnFoo);',
    '$mod.b = !rtl.eqCallback($with.GetFoo(), $with.FOnFoo);',
    '$mod.b = rtl.eqCallback($with.FOnFoo, rtl.createCallback($with, "DoIt"));',
    '$mod.b = rtl.eqCallback($with.FOnFoo, rtl.createCallback($with, "DoIt"));',
    '$mod.b = rtl.eqCallback($with.GetFoo(), rtl.createCallback($with, "DoIt"));',
    '$mod.b = !rtl.eqCallback($with.FOnFoo, rtl.createCallback($with, "DoIt"));',
    '$mod.b = !rtl.eqCallback($with.FOnFoo, rtl.createCallback($with, "DoIt"));',
    '$mod.b = !rtl.eqCallback($with.GetFoo(), rtl.createCallback($with, "DoIt"));',
    '$mod.b = $with.FOnFoo != null;',
    '$mod.b = $with.FOnFoo != null;',
    '$mod.b = $with.GetFoo() != null;',
    '']));
end;

procedure TTestModule.TestProcType_Nested;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProcInt = procedure(vI: longint = 1);',
  'procedure DoIt(vJ: longint);',
  'var aProc: TProcInt;',
  '    b: boolean;',
  '  procedure Sub(vK: longint);',
  '  var aSub: TProcInt;',
  '    procedure SubSub(vK: longint);',
  '    var aSubSub: TProcInt;',
  '    begin;',
  '      aProc:=@DoIt;',
  '      aSub:=@DoIt;',
  '      aSubSub:=@DoIt;',
  '      aProc:=@Sub;',
  '      aSub:=@Sub;',
  '      aSubSub:=@Sub;',
  '      aProc:=@SubSub;',
  '      aSub:=@SubSub;',
  '      aSubSub:=@SubSub;',
  '    end;',
  '  begin;',
  '  end;',
  'begin;',
  '  aProc:=@Sub;',
  '  b:=aProc=@Sub;',
  '  b:=@Sub=aProc;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestProcType_Nested',
    LinesToStr([ // statements
    'this.DoIt = function (vJ) {',
    '  var aProc = null;',
    '  var b = false;',
    '  function Sub(vK) {',
    '    var aSub = null;',
    '    function SubSub(vK) {',
    '      var aSubSub = null;',
    '      aProc = $mod.DoIt;',
    '      aSub = $mod.DoIt;',
    '      aSubSub = $mod.DoIt;',
    '      aProc = Sub;',
    '      aSub = Sub;',
    '      aSubSub = Sub;',
    '      aProc = SubSub;',
    '      aSub = SubSub;',
    '      aSubSub = SubSub;',
    '    };',
    '  };',
    '  aProc = Sub;',
    '  b = rtl.eqCallback(aProc, Sub);',
    '  b = rtl.eqCallback(Sub, aProc);',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestProcType_NestedOfObject;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProcInt = procedure(vI: longint = 1) of object;',
  '  TObject = class',
  '    procedure DoIt(vJ: longint);',
  '  end;',
  'procedure TObject.DoIt(vJ: longint);',
  'var aProc: TProcInt;',
  '    b: boolean;',
  '  procedure Sub(vK: longint);',
  '  var aSub: TProcInt;',
  '    procedure SubSub(vK: longint);',
  '    var aSubSub: TProcInt;',
  '    begin;',
  '      aProc:=@DoIt;',
  '      aSub:=@DoIt;',
  '      aSubSub:=@DoIt;',
  '      aProc:=@Sub;',
  '      aSub:=@Sub;',
  '      aSubSub:=@Sub;',
  '      aProc:=@SubSub;',
  '      aSub:=@SubSub;',
  '      aSubSub:=@SubSub;',
  '    end;',
  '  begin;',
  '  end;',
  'begin;',
  '  aProc:=@Sub;',
  '  b:=aProc=@Sub;',
  '  b:=@Sub=aProc;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestProcType_Nested',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function (vJ) {',
    '    var $Self = this;',
    '    var aProc = null;',
    '    var b = false;',
    '    function Sub(vK) {',
    '      var aSub = null;',
    '      function SubSub(vK) {',
    '        var aSubSub = null;',
    '        aProc = rtl.createCallback($Self, "DoIt");',
    '        aSub = rtl.createCallback($Self, "DoIt");',
    '        aSubSub = rtl.createCallback($Self, "DoIt");',
    '        aProc = Sub;',
    '        aSub = Sub;',
    '        aSubSub = Sub;',
    '        aProc = SubSub;',
    '        aSub = SubSub;',
    '        aSubSub = SubSub;',
    '      };',
    '    };',
    '    aProc = Sub;',
    '    b = rtl.eqCallback(aProc, Sub);',
    '    b = rtl.eqCallback(Sub, aProc);',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestProcType_ReferenceToProc;
begin
  StartProgram(false);
  Add([
  'type',
  '  TProcRef = reference to procedure(i: longint = 0);',
  '  TFuncRef = reference to function(i: longint = 0): longint;',
  'var',
  '  p: TProcRef;',
  '  f: TFuncRef;',
  'procedure DoIt(i: longint);',
  'begin',
  'end;',
  'function GetIt(i: longint): longint;',
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
  ConvertProgram;
  CheckSource('TestProcType_ReferenceToProc',
    LinesToStr([ // statements
    'this.p = null;',
    'this.f = null;',
    'this.DoIt = function (i) {',
    '};',
    'this.GetIt = function (i) {',
    '  var Result = 0;',
    '  $mod.p = $mod.DoIt;',
    '  $mod.f = $mod.GetIt;',
    '  $mod.f(0);',
    '  $mod.f(0);',
    '  $mod.f(1);',
    '  return Result;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.DoIt;',
    '$mod.f = $mod.GetIt;',
    '$mod.f(0);',
    '$mod.f(0);',
    '$mod.f(1);',
    '$mod.p = $mod.f;',
    '']));
end;

procedure TTestModule.TestProcType_ReferenceToMethod;
begin
  StartProgram(false);
  Add([
  'type',
  '  TFuncRef = reference to function(i: longint = 5): longint;',
  '  TObject = class',
  '    function Grow(s: longint): longint;',
  '  end;',
  'var',
  '  f: tfuncref;',
  'function tobject.grow(s: longint): longint;',
  '  function GrowSub(i: longint): longint;',
  '  begin',
  '    f:=@grow;',
  '    f:=@growsub;',
  '  end;',
  'begin',
  '  f:=@grow;',
  '  f:=@growsub;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestProcType_ReferenceToMethod',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Grow = function (s) {',
    '    var $Self = this;',
    '    var Result = 0;',
    '    function GrowSub(i) {',
    '      var Result = 0;',
    '      $mod.f = rtl.createCallback($Self, "Grow");',
    '      $mod.f = GrowSub;',
    '      return Result;',
    '    };',
    '    $mod.f = rtl.createCallback($Self, "Grow");',
    '    $mod.f = GrowSub;',
    '    return Result;',
    '  };',
    '});',
    'this.f = null;',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestProcType_Typecast;
begin
  StartProgram(false);
  Add([
  'type',
  '  TNotifyEvent = procedure(Sender: Pointer) of object;',
  '  TEvent = procedure of object;',
  '  TGetter = function:longint of object;',
  '  TProcA = procedure(i: longint);',
  '  TFuncB = function(i, j: longint): longint;',
  'procedure DoIt(); varargs; begin end;',
  'var',
  '  Notify: tnotifyevent;',
  '  Event: tevent;',
  '  Getter: tgetter;',
  '  ProcA: tproca;',
  '  FuncB: tfuncb;',
  '  p: pointer;',
  'begin',
  '  notify:=tnotifyevent(event);',
  '  event:=tevent(event);',
  '  event:=tevent(notify);',
  '  event:=tevent(getter);',
  '  event:=tevent(proca);',
  '  proca:=tproca(funcb);',
  '  funcb:=tfuncb(funcb);',
  '  funcb:=tfuncb(proca);',
  '  funcb:=tfuncb(getter);',
  '  proca:=tproca(p);',
  '  funcb:=tfuncb(p);',
  '  getter:=tgetter(p);',
  '  p:=pointer(notify);',
  '  p:=notify;',
  '  p:=pointer(proca);',
  '  p:=proca;',
  '  p:=pointer(funcb);',
  '  p:=funcb;',
  '  doit(Pointer(notify),pointer(event),pointer(proca));',
  '']);
  ConvertProgram;
  CheckSource('TestProcType_Typecast',
    LinesToStr([ // statements
    'this.DoIt = function () {',
    '};',
    'this.Notify = null;',
    'this.Event = null;',
    'this.Getter = null;',
    'this.ProcA = null;',
    'this.FuncB = null;',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Notify = $mod.Event;',
    '$mod.Event = $mod.Event;',
    '$mod.Event = $mod.Notify;',
    '$mod.Event = $mod.Getter;',
    '$mod.Event = $mod.ProcA;',
    '$mod.ProcA = $mod.FuncB;',
    '$mod.FuncB = $mod.FuncB;',
    '$mod.FuncB = $mod.ProcA;',
    '$mod.FuncB = $mod.Getter;',
    '$mod.ProcA = $mod.p;',
    '$mod.FuncB = $mod.p;',
    '$mod.Getter = $mod.p;',
    '$mod.p = $mod.Notify;',
    '$mod.p = $mod.Notify;',
    '$mod.p = $mod.ProcA;',
    '$mod.p = $mod.ProcA;',
    '$mod.p = $mod.FuncB;',
    '$mod.p = $mod.FuncB;',
    '$mod.DoIt($mod.Notify, $mod.Event, $mod.ProcA);',
    '']));
end;

procedure TTestModule.TestProcType_PassProcToUntyped;
begin
  StartProgram(false);
  Add([
  'type',
  '  TEvent = procedure of object;',
  '  TFunc = function: longint;',
  'procedure DoIt(); varargs; begin end;',
  'procedure DoSome(const a; var b; p: pointer); begin end;',
  'var',
  '  Event: tevent;',
  '  Func: TFunc;',
  'begin',
  '  doit(event,func);',
  '  dosome(event,event,event);',
  '  dosome(func,func,func);',
  '']);
  ConvertProgram;
  CheckSource('TestProcType_PassProcToUntyped',
    LinesToStr([ // statements
    'this.DoIt = function () {',
    '};',
    'this.DoSome = function (a, b, p) {',
    '};',
    'this.Event = null;',
    'this.Func = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt($mod.Event, $mod.Func);',
    '$mod.DoSome($mod.Event, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.Event;',
    '    },',
    '  set: function (v) {',
    '      this.p.Event = v;',
    '    }',
    '}, $mod.Event);',
    '$mod.DoSome($mod.Func, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.Func;',
    '    },',
    '  set: function (v) {',
    '      this.p.Func = v;',
    '    }',
    '}, $mod.Func);',
    '']));
end;

procedure TTestModule.TestProcType_PassProcToArray;
begin
  StartProgram(false);
  Add([
  'type',
  '  TFunc = function: longint;',
  '  TArrFunc = array of TFunc;',
  'procedure DoIt(Arr: TArrFunc); begin end;',
  'function GetIt: longint; begin end;',
  'var',
  '  Func: tfunc;',
  'begin',
  '  doit([]);',
  '  doit([@GetIt]);',
  '  doit([Func]);',
  '']);
  ConvertProgram;
  CheckSource('TestProcType_PassProcToArray',
    LinesToStr([ // statements
    'this.DoIt = function (Arr) {',
    '};',
    'this.GetIt = function () {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    'this.Func = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt([]);',
    '$mod.DoIt([$mod.GetIt]);',
    '$mod.DoIt([$mod.Func]);',
    '']));
end;

procedure TTestModule.TestProcType_SafeCallObjFPC;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TProc = reference to procedure(i: longint); safecall;',
  '  TEvent = procedure(i: longint) of object; safecall;',
  '  TExtA = class external name ''ExtObj''',
  '    procedure DoIt(Id: longint = 1); external name ''$Execute'';',
  '    procedure DoSome(Id: longint = 1);',
  '    procedure SetOnClick(const e: TEvent);',
  '    property OnClick: TEvent write SetOnClick;',
  '    class procedure Fly(Id: longint = 1); static;',
  '    procedure SetOnShow(const p: TProc);',
  '    property OnShow: TProc write SetOnShow;',
  '  end;',
  'procedure Run(i: longint = 1);',
  'begin',
  'end;',
  'var',
  '  Obj: texta;',
  '  e: TEvent;',
  '  p: TProc;',
  'begin',
  '  e:=e;',
  '  e:=@obj.doit;',
  '  e:=@obj.dosome;',
  '  e:=TEvent(@obj.dosome);', // no safecall
  '  obj.OnClick:=@obj.doit;',
  '  obj.OnClick:=@obj.dosome;',
  '  obj.setonclick(@obj.doit);',
  '  obj.setonclick(@obj.dosome);',
  '  p:=@Run;',
  '  p:=@TExtA.Fly;',
  '  obj.OnShow:=@Run;',
  '  obj.OnShow:=@TExtA.Fly;',
  '  obj.setOnShow(@Run);',
  '  obj.setOnShow(@TExtA.Fly);',
  '  with obj do begin',
  '    e:=@doit;',
  '    e:=@dosome;',
  '    OnClick:=@doit;',
  '    OnClick:=@dosome;',
  '    setonclick(@doit);',
  '    setonclick(@dosome);',
  '    OnShow:=@Run;',
  '    setOnShow(@Run);',
  '  end;']);
  ConvertProgram;
  CheckSource('TestProcType_SafeCallObjFPC',
    LinesToStr([ // statements
    'this.Run = function (i) {',
    '};',
    'this.Obj = null;',
    'this.e = null;',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.e = $mod.e;',
    '$mod.e = rtl.createSafeCallback($mod.Obj, "$Execute");',
    '$mod.e = rtl.createSafeCallback($mod.Obj, "DoSome");',
    '$mod.e = rtl.createCallback($mod.Obj, "DoSome");',
    '$mod.Obj.SetOnClick(rtl.createSafeCallback($mod.Obj, "$Execute"));',
    '$mod.Obj.SetOnClick(rtl.createSafeCallback($mod.Obj, "DoSome"));',
    '$mod.Obj.SetOnClick(rtl.createSafeCallback($mod.Obj, "$Execute"));',
    '$mod.Obj.SetOnClick(rtl.createSafeCallback($mod.Obj, "DoSome"));',
    '$mod.p = rtl.createSafeCallback($mod, "Run");',
    '$mod.p = rtl.createSafeCallback(ExtObj, "Fly");',
    '$mod.Obj.SetOnShow(rtl.createSafeCallback($mod, "Run"));',
    '$mod.Obj.SetOnShow(rtl.createSafeCallback(ExtObj, "Fly"));',
    '$mod.Obj.SetOnShow(rtl.createSafeCallback($mod, "Run"));',
    '$mod.Obj.SetOnShow(rtl.createSafeCallback(ExtObj, "Fly"));',
    'var $with = $mod.Obj;',
    '$mod.e = rtl.createSafeCallback($with, "$Execute");',
    '$mod.e = rtl.createSafeCallback($with, "DoSome");',
    '$with.SetOnClick(rtl.createSafeCallback($with, "$Execute"));',
    '$with.SetOnClick(rtl.createSafeCallback($with, "DoSome"));',
    '$with.SetOnClick(rtl.createSafeCallback($with, "$Execute"));',
    '$with.SetOnClick(rtl.createSafeCallback($with, "DoSome"));',
    '$with.SetOnShow(rtl.createSafeCallback($mod, "Run"));',
    '$with.SetOnShow(rtl.createSafeCallback($mod, "Run"));',
    '']));
end;

procedure TTestModule.TestProcType_SafeCallDelphi;
begin
  StartProgram(false);
  Add([
  '{$mode delphi}',
  '{$modeswitch externalclass}',
  'type',
  '  TProc = reference to procedure(i: longint); safecall;',
  '  TEvent = procedure(i: longint) of object; safecall;',
  '  TExtA = class external name ''ExtObj''',
  '    procedure DoIt(Id: longint = 1); external name ''$Execute'';',
  '    procedure DoSome(Id: longint = 1);',
  '    procedure SetOnClick(const e: TEvent);',
  '    property OnClick: TEvent write SetOnClick;',
  '    class procedure Fly(Id: longint = 1); static;',
  '    procedure SetOnShow(const p: TProc);',
  '    property OnShow: TProc write SetOnShow;',
  '  end;',
  'procedure Run(i: longint = 1);',
  'begin',
  'end;',
  'var',
  '  Obj: texta;',
  '  e: TEvent;',
  '  p: TProc;',
  'begin',
  '  e:=e;',
  '  e:=obj.doit;',
  '  e:=obj.dosome;',
  '  e:=TEvent(@obj.dosome);', // no safecall
  '  obj.OnClick:=obj.doit;',
  '  obj.OnClick:=obj.dosome;',
  '  obj.setonclick(obj.doit);',
  '  obj.setonclick(obj.dosome);',
  '  p:=Run;',
  '  p:=TExtA.Fly;',
  '  obj.OnShow:=Run;',
  '  obj.OnShow:=TExtA.Fly;',
  '  obj.setOnShow(Run);',
  '  obj.setOnShow(TExtA.Fly);',
  '  with obj do begin',
  '    e:=doit;',
  '    e:=dosome;',
  '    OnClick:=doit;',
  '    OnClick:=dosome;',
  '    setonclick(doit);',
  '    setonclick(dosome);',
  '    OnShow:=@Run;',
  '    setOnShow(@Run);',
  '  end;']);
  ConvertProgram;
  CheckSource('TestProcType_SafeCallDelphi',
    LinesToStr([ // statements
    'this.Run = function (i) {',
    '};',
    'this.Obj = null;',
    'this.e = null;',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.e = $mod.e;',
    '$mod.e = rtl.createSafeCallback($mod.Obj, "$Execute");',
    '$mod.e = rtl.createSafeCallback($mod.Obj, "DoSome");',
    '$mod.e = rtl.createCallback($mod.Obj, "DoSome");',
    '$mod.Obj.SetOnClick(rtl.createSafeCallback($mod.Obj, "$Execute"));',
    '$mod.Obj.SetOnClick(rtl.createSafeCallback($mod.Obj, "DoSome"));',
    '$mod.Obj.SetOnClick(rtl.createSafeCallback($mod.Obj, "$Execute"));',
    '$mod.Obj.SetOnClick(rtl.createSafeCallback($mod.Obj, "DoSome"));',
    '$mod.p = rtl.createSafeCallback($mod, "Run");',
    '$mod.p = rtl.createSafeCallback(ExtObj, "Fly");',
    '$mod.Obj.SetOnShow(rtl.createSafeCallback($mod, "Run"));',
    '$mod.Obj.SetOnShow(rtl.createSafeCallback(ExtObj, "Fly"));',
    '$mod.Obj.SetOnShow(rtl.createSafeCallback($mod, "Run"));',
    '$mod.Obj.SetOnShow(rtl.createSafeCallback(ExtObj, "Fly"));',
    'var $with = $mod.Obj;',
    '$mod.e = rtl.createSafeCallback($with, "$Execute");',
    '$mod.e = rtl.createSafeCallback($with, "DoSome");',
    '$with.SetOnClick(rtl.createSafeCallback($with, "$Execute"));',
    '$with.SetOnClick(rtl.createSafeCallback($with, "DoSome"));',
    '$with.SetOnClick(rtl.createSafeCallback($with, "$Execute"));',
    '$with.SetOnClick(rtl.createSafeCallback($with, "DoSome"));',
    '$with.SetOnShow(rtl.createSafeCallback($mod, "Run"));',
    '$with.SetOnShow(rtl.createSafeCallback($mod, "Run"));',
    '']));
end;

procedure TTestModule.TestPointer;
begin
  StartProgram(false);
  Add(['type',
  '  TObject = class end;',
  '  TClass = class of TObject;',
  '  TArrInt = array of longint;',
  'const',
  '  n = nil;',
  'var',
  '  v: jsvalue;',
  '  Obj: tobject;',
  '  C: tclass;',
  '  a: tarrint;',
  '  p: Pointer = nil;',
  '  s: string;',
  'begin',
  '  p:=p;',
  '  p:=nil;',
  '  if p=nil then;',
  '  if nil=p then;',
  '  if Assigned(p) then;',
  '  p:=Pointer(v);',
  '  p:=obj;',
  '  p:=c;',
  '  p:=a;',
  '  p:=tobject;',
  '  obj:=TObject(p);',
  '  c:=TClass(p);',
  '  a:=TArrInt(p);',
  '  p:=n;',
  '  p:=Pointer(a);',
  '  p:=pointer(s);',
  '  s:=string(p);',
  '']);
  ConvertProgram;
  CheckSource('TestPointer',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.n = null;',
    'this.v = undefined;',
    'this.Obj = null;',
    'this.C = null;',
    'this.a = [];',
    'this.p = null;',
    'this.s = "";',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.p;',
    '$mod.p = null;',
    'if ($mod.p === null) ;',
    'if (null === $mod.p) ;',
    'if ($mod.p != null) ;',
    '$mod.p = $mod.v;',
    '$mod.p = $mod.Obj;',
    '$mod.p = $mod.C;',
    '$mod.p = $mod.a;',
    '$mod.p = $mod.TObject;',
    '$mod.Obj = $mod.p;',
    '$mod.C = $mod.p;',
    '$mod.a = $mod.p;',
    '$mod.p = null;',
    '$mod.p = $mod.a;',
    '$mod.p = $mod.s;',
    '$mod.s = $mod.p;',
    '']));
end;

procedure TTestModule.TestPointer_Proc;
begin
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    procedure DoIt; virtual; abstract;');
  Add('  end;');
  Add('procedure DoSome; begin end;');
  Add('var');
  Add('  o: TObject;');
  Add('  p: Pointer;');
  Add('begin');
  Add('  p:=@DoSome;');
  Add('  p:=@o.DoIt;');
  ConvertProgram;
  CheckSource('TestPointer_Proc',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.DoSome = function () {',
    '};',
    'this.o = null;',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.DoSome;',
    '$mod.p = rtl.createCallback($mod.o, "DoIt");',
    '']));
end;

procedure TTestModule.TestPointer_AssignRecordFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TRec = record end;');
  Add('var');
  Add('  p: Pointer;');
  Add('  r: TRec;');
  Add('begin');
  Add('  p:=r;');
  SetExpectedPasResolverError('Incompatible types: got "TRec" expected "Pointer"',
    nIncompatibleTypesGotExpected);
  ConvertProgram;
end;

procedure TTestModule.TestPointer_AssignStaticArrayFail;
begin
  StartProgram(false);
  Add('type');
  Add('  TArr = array[boolean] of longint;');
  Add('var');
  Add('  p: Pointer;');
  Add('  a: TArr;');
  Add('begin');
  Add('  p:=a;');
  SetExpectedPasResolverError('Incompatible types: got "TArr" expected "Pointer"',
    nIncompatibleTypesGotExpected);
  ConvertProgram;
end;

procedure TTestModule.TestPointer_TypeCastJSValueToPointer;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(args: array of jsvalue); begin end;',
  'procedure DoAll; varargs; begin end;',
  'var',
  '  v: jsvalue;',
  'begin',
  '  DoIt([pointer(v)]);',
  '  DoAll(pointer(v));',
  '']);
  ConvertProgram;
  CheckSource('TestPointer_TypeCastJSValueToPointer',
    LinesToStr([ // statements
    'this.DoIt = function (args) {',
    '};',
    'this.DoAll = function () {',
    '};',
    'this.v = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt([$mod.v]);',
    '$mod.DoAll($mod.v);',
    '']));
end;

procedure TTestModule.TestPointer_NonRecordFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  p = ^longint;',
  'begin',
  '']);
  SetExpectedPasResolverError('Not supported: pointer of Longint',nNotSupportedX);
  ConvertProgram;
end;

procedure TTestModule.TestPointer_AnonymousArgTypeFail;
begin
  StartProgram(false);
  Add([
  'procedure DoIt(p: ^longint); begin end;',
  'begin',
  '']);
  SetExpectedPasResolverError('Not supported: pointer',nNotSupportedX);
  ConvertProgram;
end;

procedure TTestModule.TestPointer_AnonymousVarTypeFail;
begin
  StartProgram(false);
  Add([
  'var p: ^longint;',
  'begin',
  '']);
  SetExpectedPasResolverError('Not supported: pointer',nNotSupportedX);
  ConvertProgram;
end;

procedure TTestModule.TestPointer_AnonymousResultTypeFail;
begin
  StartProgram(false);
  Add([
  'function DoIt: ^longint; begin end;',
  'begin',
  '']);
  SetExpectedPasResolverError('Not supported: pointer',nNotSupportedX);
  ConvertProgram;
end;

procedure TTestModule.TestPointer_AddrOperatorFail;
begin
  StartProgram(false);
  Add([
  'var i: longint;',
  'begin',
  '  if @i=nil then ;',
  '']);
  SetExpectedConverterError('illegal qualifier "@" in front of "i:Longint"',nIllegalQualifierInFrontOf);
  ConvertProgram;
end;

procedure TTestModule.TestPointer_ArrayParamsFail;
begin
  StartProgram(false);
  Add([
  'var',
  '  p: Pointer;',
  'begin',
  '  p:=p[1];',
  '']);
  SetExpectedPasResolverError('illegal qualifier "[" after "Pointer"',nIllegalQualifierAfter);
  ConvertProgram;
end;

procedure TTestModule.TestPointer_PointerAddFail;
begin
  StartProgram(false);
  Add([
  'var',
  '  p: Pointer;',
  'begin',
  '  p:=p+1;',
  '']);
  SetExpectedPasResolverError('Operator is not overloaded: "Pointer" + "Longint"',nOperatorIsNotOverloadedAOpB);
  ConvertProgram;
end;

procedure TTestModule.TestPointer_IncPointerFail;
begin
  StartProgram(false);
  Add([
  'var',
  '  p: Pointer;',
  'begin',
  '  inc(p,1);',
  '']);
  SetExpectedPasResolverError('Incompatible type arg no. 1: Got "Pointer", expected "integer"',
    nIncompatibleTypeArgNo);
  ConvertProgram;
end;

procedure TTestModule.TestPointer_Record;
begin
  StartProgram(false);
  Add([
  'type',
  '  TRec = record x: longint; end;',
  '  PRec = ^TRec;',
  'var',
  '  r: TRec;',
  '  p: PRec;',
  '  q: ^TRec;',
  '  Ptr: pointer;',
  'begin',
  '  new(p);',
  '  p:=@r;',
  '  r:=p^;',
  '  r.x:=p^.x;',
  '  p^.x:=r.x;',
  '  if p^.x=3 then ;',
  '  if 4=p^.x then ;',
  '  dispose(p);',
  '  new(q);',
  '  dispose(q);',
  '  Ptr:=p;',
  '  p:=PRec(ptr);',
  '']);
  ConvertProgram;
  CheckSource('TestPointer_Record',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.x = 0;',
    '  this.$eq = function (b) {',
    '    return this.x === b.x;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    return this;',
    '  };',
    '});',
    'this.r = this.TRec.$new();',
    'this.p = null;',
    'this.q = null;',
    'this.Ptr = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.TRec.$new();',
    '$mod.p = $mod.r;',
    '$mod.r.$assign($mod.p);',
    '$mod.r.x = $mod.p.x;',
    '$mod.p.x = $mod.r.x;',
    'if ($mod.p.x === 3) ;',
    'if (4 === $mod.p.x) ;',
    '$mod.p = null;',
    '$mod.q = $mod.TRec.$new();',
    '$mod.q = null;',
    '$mod.Ptr = $mod.p;',
    '$mod.p = $mod.Ptr;',
    '']));
end;

procedure TTestModule.TestPointer_RecordArg;
begin
  StartProgram(false);
  Add([
  '{$modeswitch autoderef}',
  'type',
  '  TRec = record x: longint; end;',
  '  PRec = ^TRec;',
  'function DoIt(const a: PRec; var b: PRec; out c: PRec): TRec;',
  'begin',
  '  a.x:=a.x;',
  '  a^.x:=a^.x;',
  '  with a^ do',
  '    x:=x;',
  'end;',
  'function GetIt(p: PRec): PRec;',
  'begin',
  '  p.x:=p.x;',
  '  p^.x:=p^.x;',
  '  with p^ do',
  '    x:=x;',
  'end;',
  'var',
  '  r: TRec;',
  '  p: PRec;',
  'begin',
  '  p:=GetIt(p);',
  '  p^:=GetIt(@r)^;',
  '  DoIt(p,p,p);',
  '  DoIt(@r,p,p);',
  '']);
  ConvertProgram;
  CheckSource('TestPointer_RecordArg',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.x = 0;',
    '  this.$eq = function (b) {',
    '    return this.x === b.x;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    return this;',
    '  };',
    '});',
    'this.DoIt = function (a, b, c) {',
    '  var Result = $mod.TRec.$new();',
    '  a.x = a.x;',
    '  a.x = a.x;',
    '  a.x = a.x;',
    '  return Result;',
    '};',
    'this.GetIt = function (p) {',
    '  var Result = null;',
    '  p.x = p.x;',
    '  p.x = p.x;',
    '  p.x = p.x;',
    '  return Result;',
    '};',
    'this.r = this.TRec.$new();',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.GetIt($mod.p);',
    '$mod.p.$assign($mod.GetIt($mod.r));',
    '$mod.DoIt($mod.p, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.p;',
    '    },',
    '  set: function (v) {',
    '      this.p.p = v;',
    '    }',
    '}, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.p;',
    '    },',
    '  set: function (v) {',
    '      this.p.p = v;',
    '    }',
    '});',
    '$mod.DoIt($mod.r, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.p;',
    '    },',
    '  set: function (v) {',
    '      this.p.p = v;',
    '    }',
    '}, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.p;',
    '    },',
    '  set: function (v) {',
    '      this.p.p = v;',
    '    }',
    '});',
    '']));
end;

procedure TTestModule.TestJSValue_AssignToJSValue;
begin
  StartProgram(false);
  Add('var');
  Add('  v: jsvalue;');
  Add('  i: longint;');
  Add('  s: string;');
  Add('  b: boolean;');
  Add('  d: double;');
  Add('  p: pointer;');
  Add('begin');
  Add('  v:=v;');
  Add('  v:=1;');
  Add('  v:=i;');
  Add('  v:='''';');
  Add('  v:=''c'';');
  Add('  v:=''foo'';');
  Add('  v:=s;');
  Add('  v:=false;');
  Add('  v:=true;');
  Add('  v:=b;');
  Add('  v:=0.1;');
  Add('  v:=d;');
  Add('  v:=nil;');
  Add('  v:=p;');
  ConvertProgram;
  CheckSource('TestJSValue_AssignToJSValue',
    LinesToStr([ // statements
    'this.v = undefined;',
    'this.i = 0;',
    'this.s = "";',
    'this.b = false;',
    'this.d = 0.0;',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.v = $mod.v;',
    '$mod.v = 1;',
    '$mod.v = $mod.i;',
    '$mod.v = "";',
    '$mod.v = "c";',
    '$mod.v = "foo";',
    '$mod.v = $mod.s;',
    '$mod.v = false;',
    '$mod.v = true;',
    '$mod.v = $mod.b;',
    '$mod.v = 0.1;',
    '$mod.v = $mod.d;',
    '$mod.v = null;',
    '$mod.v = $mod.p;',
    '']));
end;

procedure TTestModule.TestJSValue_TypeCastToBaseType;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TYesNo = boolean;');
  Add('  TFloat = double;');
  Add('  TCaption = string;');
  Add('  TChar = char;');
  Add('var');
  Add('  v: jsvalue;');
  Add('  i: integer;');
  Add('  s: TCaption;');
  Add('  b: TYesNo;');
  Add('  d: TFloat;');
  Add('  c: char;');
  Add('begin');
  Add('  i:=longint(v);');
  Add('  i:=integer(v);');
  Add('  s:=string(v);');
  Add('  s:=TCaption(v);');
  Add('  b:=boolean(v);');
  Add('  b:=TYesNo(v);');
  Add('  d:=double(v);');
  Add('  d:=TFloat(v);');
  Add('  c:=char(v);');
  Add('  c:=TChar(v);');
  ConvertProgram;
  CheckSource('TestJSValue_TypeCastToBaseType',
    LinesToStr([ // statements
    'this.v = undefined;',
    'this.i = 0;',
    'this.s = "";',
    'this.b = false;',
    'this.d = 0.0;',
    'this.c = "";',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.i = rtl.trunc($mod.v);',
    '$mod.i = rtl.trunc($mod.v);',
    '$mod.s = "" + $mod.v;',
    '$mod.s = "" + $mod.v;',
    '$mod.b = !($mod.v == false);',
    '$mod.b = !($mod.v == false);',
    '$mod.d = rtl.getNumber($mod.v);',
    '$mod.d = rtl.getNumber($mod.v);',
    '$mod.c = rtl.getChar($mod.v);',
    '$mod.c = rtl.getChar($mod.v);',
    '']));
end;

procedure TTestModule.TestJSValue_TypecastToJSValue;
begin
  StartProgram(false);
  Add([
  'type',
  '  TArr = array of word;',
  '  TRec = record end;',
  '  TSet = set of boolean;',
  'procedure Fly(v: jsvalue);',
  'begin',
  'end;',
  'var',
  '  a: TArr;',
  '  r: TRec;',
  '  s: TSet;',
  'begin',
  '  Fly(jsvalue(a));',
  '  Fly(jsvalue(r));',
  '  Fly(jsvalue(s));',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_TypecastToJSValue',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '});',
    'this.Fly = function (v) {',
    '};',
    'this.a = [];',
    'this.r = this.TRec.$new();',
    'this.s = {};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Fly($mod.a);',
    '$mod.Fly($mod.r);',
    '$mod.Fly($mod.s);',
    '']));
end;

procedure TTestModule.TestJSValue_Equal;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TYesNo = boolean;');
  Add('  TFloat = double;');
  Add('  TCaption = string;');
  Add('  TChar = char;');
  Add('  TMulti = JSValue;');
  Add('var');
  Add('  v: jsvalue;');
  Add('  i: integer;');
  Add('  s: TCaption;');
  Add('  b: TYesNo;');
  Add('  d: TFloat;');
  Add('  c: char;');
  Add('  m: TMulti;');
  Add('begin');
  Add('  b:=v=v;');
  Add('  b:=v<>v;');
  Add('  b:=v=1;');
  Add('  b:=v<>1;');
  Add('  b:=2=v;');
  Add('  b:=2<>v;');
  Add('  b:=v=i;');
  Add('  b:=i=v;');
  Add('  b:=v=nil;');
  Add('  b:=nil=v;');
  Add('  b:=v=false;');
  Add('  b:=true=v;');
  Add('  b:=v=b;');
  Add('  b:=b=v;');
  Add('  b:=v=s;');
  Add('  b:=s=v;');
  Add('  b:=v=''foo'';');
  Add('  b:=''''=v;');
  Add('  b:=v=d;');
  Add('  b:=d=v;');
  Add('  b:=v=3.4;');
  Add('  b:=5.6=v;');
  Add('  b:=v=c;');
  Add('  b:=c=v;');
  Add('  b:=m=m;');
  Add('  b:=v=m;');
  Add('  b:=m=v;');
  ConvertProgram;
  CheckSource('TestJSValue_Equal',
    LinesToStr([ // statements
    'this.v = undefined;',
    'this.i = 0;',
    'this.s = "";',
    'this.b = false;',
    'this.d = 0.0;',
    'this.c = "";',
    'this.m = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.b = $mod.v == $mod.v;',
    '$mod.b = $mod.v != $mod.v;',
    '$mod.b = $mod.v == 1;',
    '$mod.b = $mod.v != 1;',
    '$mod.b = 2 == $mod.v;',
    '$mod.b = 2 != $mod.v;',
    '$mod.b = $mod.v == $mod.i;',
    '$mod.b = $mod.i == $mod.v;',
    '$mod.b = $mod.v == null;',
    '$mod.b = null == $mod.v;',
    '$mod.b = $mod.v == false;',
    '$mod.b = true == $mod.v;',
    '$mod.b = $mod.v == $mod.b;',
    '$mod.b = $mod.b == $mod.v;',
    '$mod.b = $mod.v == $mod.s;',
    '$mod.b = $mod.s == $mod.v;',
    '$mod.b = $mod.v == "foo";',
    '$mod.b = "" == $mod.v;',
    '$mod.b = $mod.v == $mod.d;',
    '$mod.b = $mod.d == $mod.v;',
    '$mod.b = $mod.v == 3.4;',
    '$mod.b = 5.6 == $mod.v;',
    '$mod.b = $mod.v == $mod.c;',
    '$mod.b = $mod.c == $mod.v;',
    '$mod.b = $mod.m == $mod.m;',
    '$mod.b = $mod.v == $mod.m;',
    '$mod.b = $mod.m == $mod.v;',
    '']));
end;

procedure TTestModule.TestJSValue_If;
begin
  StartProgram(false);
  Add([
  'procedure Fly(var u);',
  'begin',
  '  if jsvalue(u) then ;',
  'end;',
  'var',
  '  v: jsvalue;',
  'begin',
  '  if v then ;',
  '  while v do ;',
  '  repeat until v;',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_If',
    LinesToStr([ // statements
    'this.Fly = function (u) {',
    '  if (u.get()) ;',
    '};',
    'this.v = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    'if ($mod.v) ;',
    'while($mod.v){',
    '};',
    'do{',
    '} while(!$mod.v);',
    '']));
end;

procedure TTestModule.TestJSValue_Not;
begin
  StartProgram(false);
  Add([
  'var',
  '  v: jsvalue;',
  '  b: boolean;',
  'begin',
  '  b:=not v;',
  '  if not v then ;',
  '  while not v do ;',
  '  repeat until not v;',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_If',
    LinesToStr([ // statements
    'this.v = undefined;',
    'this.b = false;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.b=!$mod.v;',
    'if (!$mod.v) ;',
    'while(!$mod.v){',
    '};',
    'do{',
    '} while($mod.v);',
    '']));
end;

procedure TTestModule.TestJSValue_Enum;
begin
  StartProgram(false);
  Add('type');
  Add('  TColor = (red, blue);');
  Add('  TRedBlue = TColor;');
  Add('var');
  Add('  v: jsvalue;');
  Add('  e: TColor;');
  Add('begin');
  Add('  v:=e;');
  Add('  v:=TColor(e);');
  Add('  v:=TRedBlue(e);');
  Add('  e:=TColor(v);');
  Add('  e:=TRedBlue(v);');
  ConvertProgram;
  CheckSource('TestJSValue_Enum',
    LinesToStr([ // statements
    'this.TColor = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'this.v = undefined;',
    'this.e = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.v = $mod.e;',
    '$mod.v = $mod.e;',
    '$mod.v = $mod.e;',
    '$mod.e = $mod.v;',
    '$mod.e = $mod.v;',
    '']));
end;

procedure TTestModule.TestJSValue_ClassInstance;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TBirdObject = TObject;',
  'var',
  '  v: jsvalue;',
  '  o: TObject;',
  'begin',
  '  v:=o;',
  '  v:=TObject(o);',
  '  v:=TBirdObject(o);',
  '  o:=TObject(v);',
  '  o:=TBirdObject(v);',
  '  if v is TObject then ;',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_ClassInstance',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.v = undefined;',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.v = $mod.o;',
    '$mod.v = $mod.o;',
    '$mod.v = $mod.o;',
    '$mod.o = rtl.getObject($mod.v);',
    '$mod.o = rtl.getObject($mod.v);',
    'if (rtl.isExt($mod.v, $mod.TObject, 1)) ;',
    '']));
end;

procedure TTestModule.TestJSValue_ClassOf;
begin
  StartProgram(false);
  Add([
  'type',
  '  TClass = class of TObject;',
  '  TObject = class',
  '  end;',
  '  TBirds = class of TBird;',
  '  TBird = class(TObject) end;',
  'var',
  '  v: jsvalue;',
  '  c: TClass;',
  'begin',
  '  v:=c;',
  '  v:=TObject;',
  '  v:=TClass(c);',
  '  v:=TBirds(c);',
  '  c:=TClass(v);',
  '  c:=TBirds(v);',
  '  if v is TClass then ;',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_ClassOf',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '});',
    'this.v = undefined;',
    'this.c = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.v = $mod.c;',
    '$mod.v = $mod.TObject;',
    '$mod.v = $mod.c;',
    '$mod.v = $mod.c;',
    '$mod.c = rtl.getObject($mod.v);',
    '$mod.c = rtl.getObject($mod.v);',
    'if (rtl.isExt($mod.v, $mod.TObject, 2)) ;',
    '']));
end;

procedure TTestModule.TestJSValue_ArrayOfJSValue;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  TArray = array of JSValue;',
  '  TArrgh = tarray;',
  '  TArrInt = array of integer;',
  'var',
  '  v: jsvalue;',
  '  TheArray: tarray = (1,''2'');',
  '  Arr: tarrgh;',
  '  i: integer;',
  '  ArrInt: tarrint;',
  'begin',
  '  arr:=thearray;',
  '  thearray:=arr;',
  '  setlength(arr,2);',
  '  setlength(thearray,3);',
  '  arr[4]:=v;',
  '  arr[5]:=length(thearray);',
  '  arr[6]:=nil;',
  '  arr[7]:=thearray[8];',
  '  arr[low(arr)]:=high(thearray);',
  '  arr:=arrint;',
  '  arrInt:=tarrint(arr);',
  '  if TheArray = nil then ;',
  '  if nil = TheArray then ;',
  '  if TheArray <> nil then ;',
  '  if nil <> TheArray then ;',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_ArrayOfJSValue',
    LinesToStr([ // statements
    'this.v = undefined;',
    'this.TheArray = [1, "2"];',
    'this.Arr = [];',
    'this.i = 0;',
    'this.ArrInt = [];',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.Arr = rtl.arrayRef($mod.TheArray);',
    '$mod.TheArray = rtl.arrayRef($mod.Arr);',
    '$mod.Arr = rtl.arraySetLength($mod.Arr,undefined,2);',
    '$mod.TheArray = rtl.arraySetLength($mod.TheArray,undefined,3);',
    '$mod.Arr[4] = $mod.v;',
    '$mod.Arr[5] = rtl.length($mod.TheArray);',
    '$mod.Arr[6] = null;',
    '$mod.Arr[7] = $mod.TheArray[8];',
    '$mod.Arr[0] = rtl.length($mod.TheArray) - 1;',
    '$mod.Arr = rtl.arrayRef($mod.ArrInt);',
    '$mod.ArrInt = $mod.Arr;',
    'if (rtl.length($mod.TheArray) === 0) ;',
    'if (rtl.length($mod.TheArray) === 0) ;',
    'if (rtl.length($mod.TheArray) > 0) ;',
    'if (rtl.length($mod.TheArray) > 0) ;',
    '']));
end;

procedure TTestModule.TestJSValue_ArrayLit;
begin
  StartProgram(false);
  Add([
  'type',
  '  TFlag = (big,small);',
  '  TArray = array of JSValue;',
  '  TObject = class end;',
  '  TClass = class of TObject;',
  'var',
  '  v: jsvalue;',
  '  a: TArray;',
  '  o: TObject;',
  'begin',
  '  a:=[];',
  '  a:=[1];',
  '  a:=[1,2];',
  '  a:=[big];',
  '  a:=[1,big];',
  '  a:=[o,nil];',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_ArrayLit',
    LinesToStr([ // statements
    'this.TFlag = {',
    '  "0": "big",',
    '  big: 0,',
    '  "1": "small",',
    '  small: 1',
    '};',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.v = undefined;',
    'this.a = [];',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.a = [];',
    '$mod.a = [1];',
    '$mod.a = [1, 2];',
    '$mod.a = [$mod.TFlag.big];',
    '$mod.a = [1, $mod.TFlag.big];',
    '$mod.a = [$mod.o, null];',
    '']));
end;

procedure TTestModule.TestJSValue_Params;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TYesNo = boolean;');
  Add('  TFloat = double;');
  Add('  TCaption = string;');
  Add('  TChar = char;');
  Add('function DoIt(a: jsvalue; const b: jsvalue; var c: jsvalue; out d: jsvalue): jsvalue;');
  Add('var');
  Add('  l: jsvalue;');
  Add('begin');
  Add('  a:=a;');
  Add('  l:=b;');
  Add('  c:=c;');
  Add('  d:=d;');
  Add('  Result:=l;');
  Add('end;');
  Add('function DoSome(a: jsvalue; const b: jsvalue): jsvalue; begin end;');
  Add('var');
  Add('  v: jsvalue;');
  Add('  i: integer;');
  Add('  b: TYesNo;');
  Add('  d: TFloat;');
  Add('  s: TCaption;');
  Add('  c: TChar;');
  Add('begin');
  Add('  v:=doit(v,v,v,v);');
  Add('  i:=integer(dosome(i,i));');
  Add('  b:=TYesNo(dosome(b,b));');
  Add('  d:=TFloat(dosome(d,d));');
  Add('  s:=TCaption(dosome(s,s));');
  Add('  c:=TChar(dosome(c,c));');
  ConvertProgram;
  CheckSource('TestJSValue_Params',
    LinesToStr([ // statements
    'this.DoIt = function (a, b, c, d) {',
    '  var Result = undefined;',
    '  var l = undefined;',
    '  a = a;',
    '  l = b;',
    '  c.set(c.get());',
    '  d.set(d.get());',
    '  Result = l;',
    '  return Result;',
    '};',
    'this.DoSome = function (a, b) {',
    '  var Result = undefined;',
    '  return Result;',
    '};',
    'this.v = undefined;',
    'this.i = 0;',
    'this.b = false;',
    'this.d = 0.0;',
    'this.s = "";',
    'this.c = "";',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.v = $mod.DoIt($mod.v, $mod.v, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.v;',
    '    },',
    '  set: function (v) {',
    '      this.p.v = v;',
    '    }',
    '}, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.v;',
    '    },',
    '  set: function (v) {',
    '      this.p.v = v;',
    '    }',
    '});',
    '$mod.i = rtl.trunc($mod.DoSome($mod.i, $mod.i));',
    '$mod.b = !($mod.DoSome($mod.b, $mod.b) == false);',
    '$mod.d = rtl.getNumber($mod.DoSome($mod.d, $mod.d));',
    '$mod.s = "" + $mod.DoSome($mod.s, $mod.s);',
    '$mod.c = rtl.getChar($mod.DoSome($mod.c, $mod.c));',
    '']));
end;

procedure TTestModule.TestJSValue_UntypedParam;
begin
  StartProgram(false);
  Add('function DoIt(const a; var b; out c): jsvalue;');
  Add('begin');
  Add('  Result:=a;');
  Add('  Result:=b;');
  Add('  Result:=c;');
  Add('  b:=Result;');
  Add('  c:=Result;');
  Add('end;');
  Add('var i: longint;');
  Add('begin');
  Add('  doit(i,i,i);');
  ConvertProgram;
  CheckSource('TestJSValue_UntypedParam',
    LinesToStr([ // statements
    'this.DoIt = function (a, b, c) {',
    '  var Result = undefined;',
    '  Result = a;',
    '  Result = b.get();',
    '  Result = c.get();',
    '  b.set(Result);',
    '  c.set(Result);',
    '  return Result;',
    '};',
    'this.i = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt($mod.i, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '}, {',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.i;',
    '    },',
    '  set: function (v) {',
    '      this.p.i = v;',
    '    }',
    '});',
    '']));
end;

procedure TTestModule.TestJSValue_FuncResultType;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TJSValueArray = array of JSValue;');
  Add('  TListSortCompare = function(Item1, Item2: JSValue): Integer;');
  Add('procedure Sort(P: JSValue; aList: TJSValueArray; const Compare: TListSortCompare);');
  Add('begin');
  Add('  while Compare(P,aList[0])>0 do ;');
  Add('end;');
  Add('var');
  Add('  Compare: TListSortCompare;');
  Add('  V: JSValue;');
  Add('  i: integer;');
  Add('begin');
  Add('  if Compare(V,V)>0 then ;');
  Add('  if Compare(i,i)>1 then ;');
  Add('  if Compare(nil,false)>2 then ;');
  Add('  if Compare(1,true)>3 then ;');
  ConvertProgram;
  CheckSource('TestJSValue_UntypedParam',
    LinesToStr([ // statements
    'this.Sort = function (P, aList, Compare) {',
    '  while (Compare(P, aList[0]) > 0) {',
    '  };',
    '};',
    'this.Compare = null;',
    'this.V = undefined;',
    'this.i = 0;',
    '']),
    LinesToStr([ // $mod.$main
    'if ($mod.Compare($mod.V, $mod.V) > 0) ;',
    'if ($mod.Compare($mod.i, $mod.i) > 1) ;',
    'if ($mod.Compare(null, false) > 2) ;',
    'if ($mod.Compare(1, true) > 3) ;',
    '']));
end;

procedure TTestModule.TestJSValue_ProcType_Assign;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TObject = class');
  Add('    class function GetGlob: integer;');
  Add('    function Getter: integer;');
  Add('  end;');
  Add('class function TObject.GetGlob: integer;');
  Add('var v1: jsvalue;');
  Add('begin');
  Add('  v1:=@GetGlob;');
  Add('  v1:=@Self.GetGlob;');
  Add('end;');
  Add('function TObject.Getter: integer;');
  Add('var v2: jsvalue;');
  Add('begin');
  Add('  v2:=@Getter;');
  Add('  v2:=@Self.Getter;');
  Add('  v2:=@GetGlob;');
  Add('  v2:=@Self.GetGlob;');
  Add('end;');
  Add('function GetIt(i: integer): integer;');
  Add('var v3: jsvalue;');
  Add('begin');
  Add('  v3:=@GetIt;');
  Add('end;');
  Add('var');
  Add('  V: JSValue;');
  Add('  o: TObject;');
  Add('begin');
  Add('  v:=@GetIt;');
  Add('  v:=@o.Getter;');
  Add('  v:=@o.GetGlob;');
  ConvertProgram;
  CheckSource('TestJSValue_ProcType_Assign',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetGlob = function () {',
    '    var Result = 0;',
    '    var v1 = undefined;',
    '    v1 = rtl.createCallback(this, "GetGlob");',
    '    v1 = rtl.createCallback(this, "GetGlob");',
    '    return Result;',
    '  };',
    '  this.Getter = function () {',
    '    var Result = 0;',
    '    var v2 = undefined;',
    '    v2 = rtl.createCallback(this, "Getter");',
    '    v2 = rtl.createCallback(this, "Getter");',
    '    v2 = rtl.createCallback(this.$class, "GetGlob");',
    '    v2 = rtl.createCallback(this.$class, "GetGlob");',
    '    return Result;',
    '  };',
    '});',
    'this.GetIt = function (i) {',
    '  var Result = 0;',
    '  var v3 = undefined;',
    '  v3 = $mod.GetIt;',
    '  return Result;',
    '};',
    'this.V = undefined;',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.V = $mod.GetIt;',
    '$mod.V = rtl.createCallback($mod.o, "Getter");',
    '$mod.V = rtl.createCallback($mod.o.$class, "GetGlob");',
    '']));
end;

procedure TTestModule.TestJSValue_ProcType_Equal;
begin
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TObject = class');
  Add('    class function GetGlob: integer;');
  Add('    function Getter: integer;');
  Add('  end;');
  Add('class function TObject.GetGlob: integer;');
  Add('var v1: jsvalue;');
  Add('begin');
  Add('  if v1=@GetGlob then;');
  Add('  if v1=@Self.GetGlob then ;');
  Add('end;');
  Add('function TObject.Getter: integer;');
  Add('var v2: jsvalue;');
  Add('begin');
  Add('  if v2=@Getter then;');
  Add('  if v2=@Self.Getter then ;');
  Add('  if v2=@GetGlob then;');
  Add('  if v2=@Self.GetGlob then;');
  Add('end;');
  Add('function GetIt(i: integer): integer;');
  Add('var v3: jsvalue;');
  Add('begin');
  Add('  if v3=@GetIt then;');
  Add('end;');
  Add('var');
  Add('  V: JSValue;');
  Add('  o: TObject;');
  Add('begin');
  Add('  if v=@GetIt then;');
  Add('  if v=@o.Getter then;');
  Add('  if v=@o.GetGlob then;');
  Add('  if @GetIt=v then;');
  Add('  if @o.Getter=v then;');
  Add('  if @o.GetGlob=v then;');
  ConvertProgram;
  CheckSource('TestJSValue_ProcType_Equal',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.GetGlob = function () {',
    '    var Result = 0;',
    '    var v1 = undefined;',
    '    if (rtl.eqCallback(v1, rtl.createCallback(this, "GetGlob"))) ;',
    '    if (rtl.eqCallback(v1, rtl.createCallback(this, "GetGlob"))) ;',
    '    return Result;',
    '  };',
    '  this.Getter = function () {',
    '    var Result = 0;',
    '    var v2 = undefined;',
    '    if (rtl.eqCallback(v2, rtl.createCallback(this, "Getter"))) ;',
    '    if (rtl.eqCallback(v2, rtl.createCallback(this, "Getter"))) ;',
    '    if (rtl.eqCallback(v2, rtl.createCallback(this.$class, "GetGlob"))) ;',
    '    if (rtl.eqCallback(v2, rtl.createCallback(this.$class, "GetGlob"))) ;',
    '    return Result;',
    '  };',
    '});',
    'this.GetIt = function (i) {',
    '  var Result = 0;',
    '  var v3 = undefined;',
    '  if (rtl.eqCallback(v3, $mod.GetIt)) ;',
    '  return Result;',
    '};',
    'this.V = undefined;',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    'if (rtl.eqCallback($mod.V, $mod.GetIt)) ;',
    'if (rtl.eqCallback($mod.V, rtl.createCallback($mod.o, "Getter"))) ;',
    'if (rtl.eqCallback($mod.V, rtl.createCallback($mod.o.$class, "GetGlob"))) ;',
    'if (rtl.eqCallback($mod.GetIt, $mod.V)) ;',
    'if (rtl.eqCallback(rtl.createCallback($mod.o, "Getter"), $mod.V)) ;',
    'if (rtl.eqCallback(rtl.createCallback($mod.o.$class, "GetGlob"), $mod.V)) ;',
    '']));
end;

procedure TTestModule.TestJSValue_ProcType_Param;
begin
  StartProgram(false);
  Add([
  'type',
  '  variant = jsvalue;',
  '  TArrVariant = array of variant;',
  '  TArrVar2 = TArrVariant;',
  '  TFuncInt = function: longint;',
  'function GetIt: longint;',
  'begin',
  'end;',
  'procedure DoIt(p: jsvalue; Arr: TArrVar2);',
  'var v: variant;',
  'begin',
  '  v:=arr[1];',
  'end;',
  'var s: string;',
  'begin',
  '  DoIt(GetIt,[]);',
  '  DoIt(@GetIt,[]);',
  '  DoIt(1,[s,GetIt]);',
  '  DoIt(1,[s,@GetIt]);',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_ProcType_Param',
    LinesToStr([ // statements
    'this.GetIt = function () {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    'this.DoIt = function (p, Arr) {',
    '  var v = undefined;',
    '  v = Arr[1];',
    '};',
    'this.s = "";',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt($mod.GetIt(), []);',
    '$mod.DoIt($mod.GetIt, []);',
    '$mod.DoIt(1, [$mod.s, $mod.GetIt()]);',
    '$mod.DoIt(1, [$mod.s, $mod.GetIt]);',
    '']));
end;

procedure TTestModule.TestJSValue_AssignToPointerFail;
begin
  StartProgram(false);
  Add([
  'var',
  '  v: JSValue;',
  '  p: Pointer;',
  'begin',
  '  p:=v;',
  '']);
  SetExpectedPasResolverError('Incompatible types: got "JSValue" expected "Pointer"',
    nIncompatibleTypesGotExpected);
  ConvertProgram;
end;

procedure TTestModule.TestJSValue_OverloadDouble;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  tdatetime = double;',
  'procedure DoIt(d: double); begin end;',
  'procedure DoIt(v: jsvalue); begin end;',
  'var',
  '  d: double;',
  '  dt: tdatetime;',
  '  i: integer;',
  '  b: byte;',
  '  shi: shortint;',
  '  w: word;',
  '  smi: smallint;',
  '  lw: longword;',
  '  li: longint;',
  '  ni: nativeint;',
  '  nu: nativeuint;',
  'begin',
  '  DoIt(d);',
  '  DoIt(dt);',
  '  DoIt(i);',
  '  DoIt(b);',
  '  DoIt(shi);',
  '  DoIt(w);',
  '  DoIt(smi);',
  '  DoIt(lw);',
  '  DoIt(li);',
  '  DoIt(ni);',
  '  DoIt(nu);',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_OverloadDouble',
    LinesToStr([ // statements
    'this.DoIt = function (d) {',
    '};',
    'this.DoIt$1 = function (v) {',
    '};',
    'this.d = 0.0;',
    'this.dt = 0.0;',
    'this.i = 0;',
    'this.b = 0;',
    'this.shi = 0;',
    'this.w = 0;',
    'this.smi = 0;',
    'this.lw = 0;',
    'this.li = 0;',
    'this.ni = 0;',
    'this.nu = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt($mod.d);',
    '$mod.DoIt($mod.dt);',
    '$mod.DoIt$1($mod.i);',
    '$mod.DoIt$1($mod.b);',
    '$mod.DoIt$1($mod.shi);',
    '$mod.DoIt$1($mod.w);',
    '$mod.DoIt$1($mod.smi);',
    '$mod.DoIt$1($mod.lw);',
    '$mod.DoIt$1($mod.li);',
    '$mod.DoIt$1($mod.ni);',
    '$mod.DoIt$1($mod.nu);',
    '']));
end;

procedure TTestModule.TestJSValue_OverloadNativeInt;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  int53 = nativeint;',
  '  tdatetime = double;',
  'procedure DoIt(n: nativeint); begin end;',
  'procedure DoIt(v: jsvalue); begin end;',
  'var',
  '  d: double;',
  '  dt: tdatetime;',
  '  i: integer;',
  '  b: byte;',
  '  shi: shortint;',
  '  w: word;',
  '  smi: smallint;',
  '  lw: longword;',
  '  li: longint;',
  '  ni: nativeint;',
  '  nu: nativeuint;',
  'begin',
  '  DoIt(d);',
  '  DoIt(dt);',
  '  DoIt(i);',
  '  DoIt(b);',
  '  DoIt(shi);',
  '  DoIt(w);',
  '  DoIt(smi);',
  '  DoIt(lw);',
  '  DoIt(li);',
  '  DoIt(ni);',
  '  DoIt(nu);',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_OverloadNativeInt',
    LinesToStr([ // statements
    'this.DoIt = function (n) {',
    '};',
    'this.DoIt$1 = function (v) {',
    '};',
    'this.d = 0.0;',
    'this.dt = 0.0;',
    'this.i = 0;',
    'this.b = 0;',
    'this.shi = 0;',
    'this.w = 0;',
    'this.smi = 0;',
    'this.lw = 0;',
    'this.li = 0;',
    'this.ni = 0;',
    'this.nu = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt$1($mod.d);',
    '$mod.DoIt$1($mod.dt);',
    '$mod.DoIt($mod.i);',
    '$mod.DoIt($mod.b);',
    '$mod.DoIt($mod.shi);',
    '$mod.DoIt($mod.w);',
    '$mod.DoIt($mod.smi);',
    '$mod.DoIt($mod.lw);',
    '$mod.DoIt($mod.li);',
    '$mod.DoIt($mod.ni);',
    '$mod.DoIt($mod.nu);',
    '']));
end;

procedure TTestModule.TestJSValue_OverloadWord;
begin
  StartProgram(false);
  Add([
  'type',
  '  integer = longint;',
  '  int53 = nativeint;',
  '  tdatetime = double;',
  'procedure DoIt(w: word); begin end;',
  'procedure DoIt(v: jsvalue); begin end;',
  'var',
  '  d: double;',
  '  dt: tdatetime;',
  '  i: integer;',
  '  b: byte;',
  '  shi: shortint;',
  '  w: word;',
  '  smi: smallint;',
  '  lw: longword;',
  '  li: longint;',
  '  ni: nativeint;',
  '  nu: nativeuint;',
  'begin',
  '  DoIt(d);',
  '  DoIt(dt);',
  '  DoIt(i);',
  '  DoIt(b);',
  '  DoIt(shi);',
  '  DoIt(w);',
  '  DoIt(smi);',
  '  DoIt(lw);',
  '  DoIt(li);',
  '  DoIt(ni);',
  '  DoIt(nu);',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_OverloadWord',
    LinesToStr([ // statements
    'this.DoIt = function (w) {',
    '};',
    'this.DoIt$1 = function (v) {',
    '};',
    'this.d = 0.0;',
    'this.dt = 0.0;',
    'this.i = 0;',
    'this.b = 0;',
    'this.shi = 0;',
    'this.w = 0;',
    'this.smi = 0;',
    'this.lw = 0;',
    'this.li = 0;',
    'this.ni = 0;',
    'this.nu = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt$1($mod.d);',
    '$mod.DoIt$1($mod.dt);',
    '$mod.DoIt$1($mod.i);',
    '$mod.DoIt($mod.b);',
    '$mod.DoIt($mod.shi);',
    '$mod.DoIt($mod.w);',
    '$mod.DoIt$1($mod.smi);',
    '$mod.DoIt$1($mod.lw);',
    '$mod.DoIt$1($mod.li);',
    '$mod.DoIt$1($mod.ni);',
    '$mod.DoIt$1($mod.nu);',
    '']));
end;

procedure TTestModule.TestJSValue_OverloadString;
begin
  StartProgram(false);
  Add([
  'type',
  '  uni = string;',
  '  WChar = char;',
  'procedure DoIt(s: string); begin end;',
  'procedure DoIt(v: jsvalue); begin end;',
  'var',
  '  s: string;',
  '  c: char;',
  '  u: uni;',
  'begin',
  '  DoIt(s);',
  '  DoIt(c);',
  '  DoIt(u);',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_OverloadString',
    LinesToStr([ // statements
    'this.DoIt = function (s) {',
    '};',
    'this.DoIt$1 = function (v) {',
    '};',
    'this.s = "";',
    'this.c = "";',
    'this.u = "";',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt($mod.s);',
    '$mod.DoIt($mod.c);',
    '$mod.DoIt($mod.u);',
    '']));
end;

procedure TTestModule.TestJSValue_OverloadChar;
begin
  StartProgram(false);
  Add([
  'type',
  '  uni = string;',
  '  WChar = char;',
  'procedure DoIt(c: char); begin end;',
  'procedure DoIt(v: jsvalue); begin end;',
  'var',
  '  s: string;',
  '  c: char;',
  '  u: uni;',
  'begin',
  '  DoIt(s);',
  '  DoIt(c);',
  '  DoIt(u);',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_OverloadChar',
    LinesToStr([ // statements
    'this.DoIt = function (c) {',
    '};',
    'this.DoIt$1 = function (v) {',
    '};',
    'this.s = "";',
    'this.c = "";',
    'this.u = "";',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt$1($mod.s);',
    '$mod.DoIt($mod.c);',
    '$mod.DoIt$1($mod.u);',
    '']));
end;

procedure TTestModule.TestJSValue_OverloadPointer;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class end;',
  'procedure DoIt(p: pointer); begin end;',
  'procedure DoIt(v: jsvalue); begin end;',
  'var',
  '  o: TObject;',
  'begin',
  '  DoIt(o);',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_OverloadPointer',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.DoIt = function (p) {',
    '};',
    'this.DoIt$1 = function (v) {',
    '};',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt($mod.o);',
    '']));
end;

procedure TTestModule.TestJSValue_ForIn;
begin
  StartProgram(false);
  Add([
  'var',
  '  v: JSValue;',
  '  key: string;',
  'begin',
  '  for key in v do begin',
  '    if key=''abc'' then ;',
  '  end;',
  '']);
  ConvertProgram;
  CheckSource('TestJSValue_ForIn',
    LinesToStr([ // statements
    'this.v = undefined;',
    'this.key = "";',
    '']),
    LinesToStr([ // $mod.$main
    'for ($mod.key in $mod.v) {',
    '  if ($mod.key === "abc") ;',
    '};',
    '']));
end;

procedure TTestModule.TestRTTI_IntRange;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TGraphicsColor = -$7FFFFFFF-1..$7FFFFFFF;',
  '  TColor = type TGraphicsColor;',
  'var',
  '  p: TTypeInfo;',
  '  k: TTypeKind;',
  'begin',
  '  p:=typeinfo(TGraphicsColor);',
  '  p:=typeinfo(TColor);',
  '  k:=GetTypeKind(TGraphicsColor);',
  '  k:=GetTypeKind(TColor);',
  '']);
  ConvertProgram;
  CheckSource('TestRTTI_IntRange',
    LinesToStr([ // statements
    'this.$rtti.$Int("TGraphicsColor", {',
    '  minvalue: -2147483648,',
    '  maxvalue: 2147483647,',
    '  ordtype: 4',
    '});',
    'this.$rtti.$inherited("TColor", this.$rtti["TGraphicsColor"], {});',
    'this.p = null;',
    'this.k = 0;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TGraphicsColor"];',
    '$mod.p = $mod.$rtti["TColor"];',
    '$mod.k = 1;',
    '$mod.k = 1;',
    '']));
end;

procedure TTestModule.TestRTTI_Double;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TFloat = type double;',
  'var',
  '  p: TTypeInfo;',
  'begin',
  '  p:=typeinfo(double);',
  '  p:=typeinfo(TFloat);',
  '']);
  ConvertProgram;
  CheckSource('TestRTTI_Double',
    LinesToStr([ // statements
    'this.$rtti.$inherited("TFloat", rtl.double, {});',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = rtl.double;',
    '$mod.p = $mod.$rtti["TFloat"];',
    '']));
end;

procedure TTestModule.TestRTTI_ProcType;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TProcA = procedure;');
  Add('  TMethodB = procedure of object;');
  Add('  TProcC = procedure; varargs;');
  Add('  TProcD = procedure(i: longint; const j: string; var c: char; out d: double);');
  Add('  TProcE = function: nativeint;');
  Add('  TProcF = function(const p: TProcA): nativeuint;');
  Add('var p: pointer;');
  Add('begin');
  Add('  p:=typeinfo(tproca);');
  ConvertProgram;
  CheckSource('TestRTTI_ProcType',
    LinesToStr([ // statements
    'this.$rtti.$ProcVar("TProcA", {',
    '  procsig: rtl.newTIProcSig([])',
    '});',
    'this.$rtti.$MethodVar("TMethodB", {',
    '  procsig: rtl.newTIProcSig([]),',
    '  methodkind: 0',
    '});',
    'this.$rtti.$ProcVar("TProcC", {',
    '  procsig: rtl.newTIProcSig([], null, 2)',
    '});',
    'this.$rtti.$ProcVar("TProcD", {',
    '  procsig: rtl.newTIProcSig([["i", rtl.longint], ["j", rtl.string, 2], ["c", rtl.char, 1], ["d", rtl.double, 4]])',
    '});',
    'this.$rtti.$ProcVar("TProcE", {',
    '  procsig: rtl.newTIProcSig([], rtl.nativeint)',
    '});',
    'this.$rtti.$ProcVar("TProcF", {',
    '  procsig: rtl.newTIProcSig([["p", this.$rtti["TProcA"], 2]], rtl.nativeuint)',
    '});',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TProcA"];',
    '']));
end;

procedure TTestModule.TestRTTI_ProcType_ArgFromOtherUnit;
begin
  WithTypeInfo:=true;

  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'type',
    '  TObject = class end;'
    ]),
    '');
  StartUnit(true);
  Add('interface');
  Add('uses unit2;');
  Add('type');
  Add('  TProcA = function(o: tobject): tobject;');
  Add('implementation');
  Add('type');
  Add('  TProcB = function(o: tobject): tobject;');
  Add('var p: Pointer;');
  Add('initialization');
  Add('  p:=typeinfo(tproca);');
  Add('  p:=typeinfo(tprocb);');
  ConvertUnit;
  CheckSource('TestRTTI_ProcType_ArgFromOtherUnit',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    'this.$rtti.$ProcVar("TProcA", {',
    '  procsig: rtl.newTIProcSig([["o", pas.unit2.$rtti["TObject"]]], pas.unit2.$rtti["TObject"])',
    '});',
    '']),
    LinesToStr([ // this.$init
    '$impl.p = $mod.$rtti["TProcA"];',
    '$impl.p = $mod.$rtti["TProcB"];',
    '']),
    LinesToStr([ // implementation
    '$mod.$rtti.$ProcVar("TProcB", {',
    '  procsig: rtl.newTIProcSig([["o", pas.unit2.$rtti["TObject"]]], pas.unit2.$rtti["TObject"])',
    '});',
    '$impl.p = null;',
    '']) );
end;

procedure TTestModule.TestRTTI_EnumAndSetType;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TFlag = (light,dark);');
  Add('  TFlags = set of TFlag;');
  Add('  TProc = function(f: TFlags): TFlag;');
  Add('var p: pointer;');
  Add('begin');
  Add('  p:=typeinfo(tflag);');
  Add('  p:=typeinfo(tflags);');
  ConvertProgram;
  CheckSource('TestRTTI_EnumAndType',
    LinesToStr([ // statements
    'this.TFlag = {',
    '  "0": "light",',
    '  light: 0,',
    '  "1": "dark",',
    '  dark: 1',
    '};',
    'this.$rtti.$Enum("TFlag", {',
    '  minvalue: 0,',
    '  maxvalue: 1,',
    '  ordtype: 1,',
    '  enumtype: this.TFlag',
    '});',
    'this.$rtti.$Set("TFlags", {',
    '  comptype: this.$rtti["TFlag"]',
    '});',
    'this.$rtti.$ProcVar("TProc", {',
    '  procsig: rtl.newTIProcSig([["f", this.$rtti["TFlags"]]], this.$rtti["TFlag"])',
    '});',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TFlag"];',
    '$mod.p = $mod.$rtti["TFlags"];',
    '']));
end;

procedure TTestModule.TestRTTI_EnumRange;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TCol = (red,green,blue);',
  '  TColRg = green..blue;',
  '  TSetOfColRg = set of TColRg;',
  'var p: pointer;',
  'begin',
  '  p:=typeinfo(tcolrg);',
  '  p:=typeinfo(tsetofcolrg);',
  '']);
  ConvertProgram;
end;

procedure TTestModule.TestRTTI_AnonymousEnumType;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TFlags = set of (red, green);');
  Add('var');
  Add('  f: TFlags;');
  Add('begin');
  Add('  Include(f,red);');
  ConvertProgram;
  CheckSource('TestRTTI_AnonymousEnumType',
    LinesToStr([ // statements
    'this.TFlags$a = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "green",',
    '  green: 1',
    '};',
    'this.$rtti.$Enum("TFlags$a", {',
    '  minvalue: 0,',
    '  maxvalue: 1,',
    '  ordtype: 1,',
    '  enumtype: this.TFlags$a',
    '});',
    'this.$rtti.$Set("TFlags", {',
    '  comptype: this.$rtti["TFlags$a"]',
    '});',
    'this.f = {};',
    '']),
    LinesToStr([
    '$mod.f = rtl.includeSet($mod.f, $mod.TFlags$a.red);',
    '']));
end;

procedure TTestModule.TestRTTI_StaticArray;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TFlag = (light,dark);');
  Add('  TFlagNames = array[TFlag] of string;');
  Add('  TBoolNames = array[boolean] of string;');
  Add('  TByteArray = array[1..32768] of byte;');
  Add('  TProc = function(f: TBoolNames): TFlagNames;');
  Add('var p: pointer;');
  Add('begin');
  Add('  p:=typeinfo(TFlagNames);');
  Add('  p:=typeinfo(TBoolNames);');
  ConvertProgram;
  CheckSource('TestRTTI_StaticArray',
    LinesToStr([ // statements
    'this.TFlag = {',
    '  "0": "light",',
    '  light: 0,',
    '  "1": "dark",',
    '  dark: 1',
    '};',
    'this.$rtti.$Enum("TFlag", {',
    '  minvalue: 0,',
    '  maxvalue: 1,',
    '  ordtype: 1,',
    '  enumtype: this.TFlag',
    '});',
    'this.$rtti.$StaticArray("TFlagNames", {',
    '  dims: [2],',
    '  eltype: rtl.string',
    '});',
    'this.$rtti.$StaticArray("TBoolNames", {',
    '  dims: [2],',
    '  eltype: rtl.string',
    '});',
    'this.$rtti.$StaticArray("TByteArray", {',
    '  dims: [32768],',
    '  eltype: rtl.byte',
    '});',
    'this.$rtti.$ProcVar("TProc", {',
    '  procsig: rtl.newTIProcSig([["f", this.$rtti["TBoolNames"]]], this.$rtti["TFlagNames"])',
    '});',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TFlagNames"];',
    '$mod.p = $mod.$rtti["TBoolNames"];',
    '']));
end;

procedure TTestModule.TestRTTI_DynArray;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TArrStr = array of string;');
  Add('  TArr2Dim = array of tarrstr;');
  Add('  TProc = function(f: TArrStr): TArr2Dim;');
  Add('var p: pointer;');
  Add('begin');
  Add('  p:=typeinfo(tarrstr);');
  Add('  p:=typeinfo(tarr2dim);');
  ConvertProgram;
  CheckSource('TestRTTI_DynArray',
    LinesToStr([ // statements
    'this.$rtti.$DynArray("TArrStr", {',
    '  eltype: rtl.string',
    '});',
    'this.$rtti.$DynArray("TArr2Dim", {',
    '  eltype: this.$rtti["TArrStr"]',
    '});',
    'this.$rtti.$ProcVar("TProc", {',
    '  procsig: rtl.newTIProcSig([["f", this.$rtti["TArrStr"]]], this.$rtti["TArr2Dim"])',
    '});',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TArrStr"];',
    '$mod.p = $mod.$rtti["TArr2Dim"];',
    '']));
end;

procedure TTestModule.TestRTTI_ArrayNestedAnonymous;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TArr = array of array of longint;');
  Add('var a: TArr;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestRTTI_ArrayNestedAnonymous',
    LinesToStr([ // statements
    'this.$rtti.$DynArray("TArr$a", {',
    '  eltype: rtl.longint',
    '});',
    'this.$rtti.$DynArray("TArr", {',
    '  eltype: this.$rtti["TArr$a"]',
    '});',
    'this.a = [];',
    '']),
    LinesToStr([ // $mod.$main
    ]));
end;

procedure TTestModule.TestRTTI_PublishedMethodOverloadFail;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  published');
  Add('    procedure Proc; virtual; abstract;');
  Add('    procedure Proc(Sender: tobject); virtual; abstract;');
  Add('  end;');
  Add('begin');
  SetExpectedPasResolverError('Duplicate published method "Proc" at test1.pp(6,19)',
    nDuplicatePublishedMethodXAtY);
  ConvertProgram;
end;

procedure TTestModule.TestRTTI_PublishedMethodHideNoHint;
begin
  WithTypeInfo:=true;
  StartUnit(false);
  Add([
  'interface',
  'type',
  '  TObject = class',
  '  end;',
  '  {$M+}',
  '  TBird = class',
  '    procedure Fly;',
  '  end;',
  '  {$M-}',
  'type',
  '  TEagle = class(TBird)',
  '    procedure Fly;',
  '  end;',
  'implementation',
  'procedure TBird.Fly;',
  'begin',
  'end;',
  'procedure TEagle.Fly;',
  'begin',
  'end;',
  '']);
  ConvertUnit;
  CheckSource('TestRTTI_PublishedMethodHideNoHint',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.Fly = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addMethod("Fly", 0, []);',
    '});',
    'rtl.createClass(this, "TEagle", this.TBird, function () {',
    '  this.Fly = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addMethod("Fly", 0, []);',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    ]));
  CheckResolverUnexpectedHints(true);
end;

procedure TTestModule.TestRTTI_PublishedMethodExternalFail;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  published');
  Add('    procedure Proc; external name ''foo'';');
  Add('  end;');
  Add('begin');
  SetExpectedPasResolverError(sPublishedNameMustMatchExternal,
    nPublishedNameMustMatchExternal);
  ConvertProgram;
end;

procedure TTestModule.TestRTTI_PublishedClassPropertyFail;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('    class var FA: longint;');
  Add('  published');
  Add('    class property A: longint read FA;');
  Add('  end;');
  Add('begin');
  SetExpectedPasResolverError('Invalid published property modifier "class"',
    nInvalidXModifierY);
  ConvertProgram;
end;

procedure TTestModule.TestRTTI_PublishedClassFieldFail;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  published');
  Add('    class var FA: longint;');
  Add('  end;');
  Add('begin');
  SetExpectedPasResolverError(sSymbolCannotBePublished,
    nSymbolCannotBePublished);
  ConvertProgram;
end;

procedure TTestModule.TestRTTI_PublishedFieldExternalFail;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TObject = class');
  Add('  published');
  Add('    V: longint; external name ''foo'';');
  Add('  end;');
  Add('begin');
  SetExpectedPasResolverError(sPublishedNameMustMatchExternal,
    nPublishedNameMustMatchExternal);
  ConvertProgram;
end;

procedure TTestModule.TestRTTI_Class_Field;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TObject = class');
  Add('  private');
  Add('    FPropA: string;');
  Add('  published');
  Add('    VarLI: longint;');
  Add('    VarC: char;');
  Add('    VarS: string;');
  Add('    VarD: double;');
  Add('    VarB: boolean;');
  Add('    VarLW: longword;');
  Add('    VarSmI: smallint;');
  Add('    VarW: word;');
  Add('    VarShI: shortint;');
  Add('    VarBy: byte;');
  Add('    VarExt: longint external name ''VarExt'';');
  Add('    ArrA, ArrB: array of byte;');
  Add('  end;');
  Add('var p: pointer;');
  Add('  Obj: tobject;');
  Add('begin');
  Add('  p:=typeinfo(tobject);');
  Add('  p:=typeinfo(p);');
  Add('  p:=typeinfo(obj);');
  ConvertProgram;
  CheckSource('TestRTTI_Class_Field',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FPropA = "";',
    '    this.VarLI = 0;',
    '    this.VarC = "";',
    '    this.VarS = "";',
    '    this.VarD = 0.0;',
    '    this.VarB = false;',
    '    this.VarLW = 0;',
    '    this.VarSmI = 0;',
    '    this.VarW = 0;',
    '    this.VarShI = 0;',
    '    this.VarBy = 0;',
    '    this.ArrA = [];',
    '    this.ArrB = [];',
    '  };',
    '  this.$final = function () {',
    '    this.ArrA = undefined;',
    '    this.ArrB = undefined;',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addField("VarLI", rtl.longint);',
    '  $r.addField("VarC", rtl.char);',
    '  $r.addField("VarS", rtl.string);',
    '  $r.addField("VarD", rtl.double);',
    '  $r.addField("VarB", rtl.boolean);',
    '  $r.addField("VarLW", rtl.longword);',
    '  $r.addField("VarSmI", rtl.smallint);',
    '  $r.addField("VarW", rtl.word);',
    '  $r.addField("VarShI", rtl.shortint);',
    '  $r.addField("VarBy", rtl.byte);',
    '  $r.addField("VarExt", rtl.longint);',
    '  $mod.$rtti.$DynArray("TObject.ArrB$a", {',
    '    eltype: rtl.byte',
    '  });',
    '  $r.addField("ArrA", $mod.$rtti["TObject.ArrB$a"]);',
    '  $r.addField("ArrB", $mod.$rtti["TObject.ArrB$a"]);',
    '});',
    'this.p = null;',
    'this.Obj = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TObject"];',
    '$mod.p = rtl.pointer;',
    '$mod.p = $mod.Obj.$rtti;',
    '']));
end;

procedure TTestModule.TestRTTI_Class_Method;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  private',
  '    procedure Internal; external name ''$intern'';',
  '  published',
  '    procedure Click; virtual; abstract;',
  '    procedure Notify(Sender: TObject); virtual; abstract;',
  '    function GetNotify: boolean; external name ''GetNotify'';',
  '    procedure Println(a,b: longint); varargs; virtual; abstract;',
  '    function Fetch(URL: string): word; async; external name ''Fetch'';',
  '  end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRTTI_Class_Method',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addMethod("Click", 0, []);',
    '  $r.addMethod("Notify", 0, [["Sender", $r]]);',
    '  $r.addMethod("GetNotify", 1, [], rtl.boolean, 4);',
    '  $r.addMethod("Println", 0, [["a", rtl.longint], ["b", rtl.longint]], null, 2);',
    '  $r.addMethod("Fetch", 1, [["URL", rtl.string]], rtl.word, 20);',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_Class_MethodArgFlags;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  published');
  Add('    procedure OpenArray(const Args: array of string); virtual; abstract;');
  Add('    procedure ByRef(var Value: longint; out Item: longint); virtual; abstract;');
  Add('    procedure Untyped(var Value; out Item); virtual; abstract;');
  Add('  end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestRTTI_Class_MethodOpenArray',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '$r.addMethod("OpenArray", 0, [["Args", rtl.string, 10]]);',
    '$r.addMethod("ByRef", 0, [["Value", rtl.longint, 1], ["Item", rtl.longint, 4]]);',
    '$r.addMethod("Untyped", 0, [["Value", null, 1], ["Item", null, 4]]);',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_Class_Property;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TObject = class');
  Add('  private');
  Add('    FColor: longint;');
  Add('    FColorStored: boolean;');
  Add('    procedure SetColor(Value: longint); virtual; abstract;');
  Add('    function GetColor: longint; virtual; abstract;');
  Add('    function GetColorStored: boolean; virtual; abstract;');
  Add('    FExtSize: longint external name ''$extSize'';');
  Add('    FExtSizeStored: boolean external name ''$extSizeStored'';');
  Add('    procedure SetExtSize(Value: longint); external name ''$setSize'';');
  Add('    function GetExtSize: longint; external name ''$getSize'';');
  Add('    function GetExtSizeStored: boolean; external name ''$getExtSizeStored'';');
  Add('  published');
  Add('    property ColorA: longint read FColor;');
  Add('    property ColorB: longint write FColor;');
  Add('    property ColorC: longint read GetColor write SetColor;');
  Add('    property ColorD: longint read FColor write FColor stored FColorStored;');
  Add('    property ExtSizeA: longint read FExtSize write FExtSize;');
  Add('    property ExtSizeB: longint read GetExtSize write SetExtSize stored FExtSizeStored;');
  Add('    property ExtSizeC: longint read FExtSize write FExtSize stored GetExtSizeStored;');
  Add('  end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestRTTI_Class_Property',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FColor = 0;',
    '    this.FColorStored = false;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addProperty("ColorA", 0, rtl.longint, "FColor", "");',
    '  $r.addProperty("ColorB", 0, rtl.longint, "", "FColor");',
    '  $r.addProperty("ColorC", 3, rtl.longint, "GetColor", "SetColor");',
    '  $r.addProperty(',
    '    "ColorD",',
    '    8,',
    '    rtl.longint,',
    '    "FColor",',
    '    "FColor",',
    '    {',
    '      stored: "FColorStored"',
    '    }',
    '  );',
    '  $r.addProperty("ExtSizeA", 0, rtl.longint, "$extSize", "$extSize");',
    '  $r.addProperty(',
    '    "ExtSizeB",',
    '    11,',
    '    rtl.longint,',
    '    "$getSize",',
    '    "$setSize",',
    '    {',
    '      stored: "$extSizeStored"',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "ExtSizeC",',
    '    12,',
    '    rtl.longint,',
    '    "$extSize",',
    '    "$extSize",',
    '    {',
    '      stored: "$getExtSizeStored"',
    '    }',
    '  );',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_Class_PropertyParams;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  integer = longint;');
  Add('  TObject = class');
  Add('  private');
  Add('    function GetItems(i: integer): tobject; virtual; abstract;');
  Add('    procedure SetItems(i: integer; value: tobject); virtual; abstract;');
  Add('    function GetValues(const i: integer; var b: boolean): char; virtual; abstract;');
  Add('    procedure SetValues(const i: integer; var b: boolean; value: char); virtual; abstract;');
  Add('  published');
  Add('    property Items[Index: integer]: tobject read getitems write setitems;');
  Add('    property Values[const keya: integer; var keyb: boolean]: char read getvalues write setvalues;');
  Add('  end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestRTTI_Class_PropertyParams',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addProperty("Items", 3, $r, "GetItems", "SetItems");',
    '  $r.addProperty("Values", 3, rtl.char, "GetValues", "SetValues");',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_Class_OtherUnit_TypeAlias;
begin
  WithTypeInfo:=true;
  AddModuleWithIntfImplSrc('unit1.pas',
    'type TColor = -5..5;',
    '');

  StartProgram(true);
  Add([
  'uses unit1;',
  'type',
  '  TColorAlias = TColor;',
  '  TColorTypeAlias = type TColor;',
  '  TObject = class',
  '  private',
  '    fColor: TColor;',
  '    fAlias: TColorAlias;',
  '    fTypeAlias: TColorTypeAlias;',
  '  published',
  '    property Color: TColor read fcolor;',
  '    property Alias: TColorAlias read falias;',
  '    property TypeAlias: TColorTypeAlias read ftypealias;',
  '  end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestRTTI_Class_OtherUnit_TypeAlias',
    LinesToStr([ // statements
    'this.$rtti.$inherited("TColorTypeAlias", pas.unit1.$rtti["TColor"], {});',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.fColor = 0;',
    '    this.fAlias = 0;',
    '    this.fTypeAlias = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addProperty("Color", 0, pas.unit1.$rtti["TColor"], "fColor", "");',
    '  $r.addProperty("Alias", 0, pas.unit1.$rtti["TColor"], "fAlias", "");',
    '  $r.addProperty("TypeAlias", 0, $mod.$rtti["TColorTypeAlias"], "fTypeAlias", "");',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_Class_OmitRTTI;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  '{$modeswitch omitrtti}',
  'type',
  '  TObject = class',
  '  private',
  '    FA: byte;',
  '  published',
  '    property A: byte read FA write FA;',
  '  end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRTTI_Class_OmitRTTI',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FA = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_Class_Field_AnonymousArrayOfSelfClass;
begin
  WithTypeInfo:=true;
  StartUnit(true,[supTObject]);
  Add([
  'interface',
  'type',
  '  {$M+}',
  '  TBird = class',
  '  published',
  '    Swarm: array of TBird;',
  '  end;',
  'implementation',
  '']);
  ConvertUnit;
  CheckSource('TestRTTI_Class_Field_AnonymousArrayOfSelfClass',
    LinesToStr([ // statements
    'rtl.createClass(this, "TBird", pas.system.TObject, function () {',
    '  this.$init = function () {',
    '    pas.system.TObject.$init.call(this);',
    '    this.Swarm = [];',
    '  };',
    '  this.$final = function () {',
    '    this.Swarm = undefined;',
    '    pas.system.TObject.$final.call(this);',
    '  };',
    '  var $r = this.$rtti;',
    '  $mod.$rtti.$DynArray("TBird.Swarm$a", {',
    '    eltype: $r',
    '  });',
    '  $r.addField("Swarm", $mod.$rtti["TBird.Swarm$a"]);',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_IndexModifier;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red, blue);',
  '  TObject = class',
  '    FB: boolean;',
  '    procedure SetIntBool(Index: longint; b: boolean); virtual; abstract;',
  '    function GetBoolBool(Index: boolean): boolean; virtual; abstract;',
  '    procedure SetBoolBool(Index: boolean; b: boolean); virtual; abstract;',
  '    function GetEnumBool(Index: TEnum): boolean; virtual; abstract;',
  '    function GetStrIntBool(A: String; I: longint): boolean; virtual; abstract;',
  '    procedure SetStrIntBool(A: String; I: longint; b: boolean); virtual; abstract;',
  '  published',
  '    property B1: boolean index 1 read FB write SetIntBool;',
  '    property B2: boolean index TEnum.blue read GetEnumBool write FB;',
  '    property I1[A: String]: boolean index 2 read GetStrIntBool write SetStrIntBool;',
  '  end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRTTI_IndexModifier',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'this.$rtti.$Enum("TEnum", {',
    '  minvalue: 0,',
    '  maxvalue: 1,',
    '  ordtype: 1,',
    '  enumtype: this.TEnum',
    '});',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FB = false;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addProperty(',
    '    "B1",',
    '    18,',
    '    rtl.boolean,',
    '    "FB",',
    '    "SetIntBool",',
    '    {',
    '      index: 1',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "B2",',
    '    17,',
    '    rtl.boolean,',
    '    "GetEnumBool",',
    '    "FB",',
    '    {',
    '      index: $mod.TEnum.blue',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "I1",',
    '    19,',
    '    rtl.boolean,',
    '    "GetStrIntBool",',
    '    "SetStrIntBool",',
    '    {',
    '      index: 2',
    '    }',
    '  );',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_StoredModifier;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'const',
  '  ConstB = true;',
  'type',
  '  TObject = class',
  '  private',
  '    FB: boolean;',
  '    function IsBStored: boolean; virtual; abstract;',
  '  published',
  '    property BoolA: boolean read FB stored true;',
  '    property BoolB: boolean read FB stored false;',
  '    property BoolC: boolean read FB stored FB;',
  '    property BoolD: boolean read FB stored ConstB;',
  '    property BoolE: boolean read FB stored IsBStored;',
  '  end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRTTI_StoredModifier',
    LinesToStr([ // statements
    'this.ConstB = true;',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FB = false;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addProperty("BoolA", 0, rtl.boolean, "FB", "");',
    '  $r.addProperty("BoolB", 4, rtl.boolean, "FB", "");',
    '  $r.addProperty(',
    '    "BoolC",',
    '    8,',
    '    rtl.boolean,',
    '    "FB",',
    '    "",',
    '    {',
    '      stored: "FB"',
    '    }',
    '  );',
    '  $r.addProperty("BoolD", 0, rtl.boolean, "FB", "");',
    '  $r.addProperty(',
    '    "BoolE",',
    '    12,',
    '    rtl.boolean,',
    '    "FB",',
    '    "",',
    '    {',
    '      stored: "IsBStored"',
    '    }',
    '  );',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_DefaultValue;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red, blue);',
  'const',
  '  CB = true or false;',
  '  CI = 1+2;',
  'type',
  '  TObject = class',
  '    FB: boolean;',
  '    FI: longint;',
  '    FE: TEnum;',
  '  published',
  '    property B1: boolean read FB default true;',
  '    property B2: boolean read FB default CB;',
  '    property B3: boolean read FB default test1.cb;',
  '    property I1: longint read FI default 2;',
  '    property I2: longint read FI default CI;',
  '    property E1: TEnum read FE default red;',
  '    property E2: TEnum read FE default TEnum.blue;',
  '  end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRTTI_DefaultValue',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'this.$rtti.$Enum("TEnum", {',
    '  minvalue: 0,',
    '  maxvalue: 1,',
    '  ordtype: 1,',
    '  enumtype: this.TEnum',
    '});',
    'this.CB = true || false;',
    'this.CI = 1 + 2;',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FB = false;',
    '    this.FI = 0;',
    '    this.FE = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addProperty(',
    '    "B1",',
    '    0,',
    '    rtl.boolean,',
    '    "FB",',
    '    "",',
    '    {',
    '      Default: true',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "B2",',
    '    0,',
    '    rtl.boolean,',
    '    "FB",',
    '    "",',
    '    {',
    '      Default: true',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "B3",',
    '    0,',
    '    rtl.boolean,',
    '    "FB",',
    '    "",',
    '    {',
    '      Default: true',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "I1",',
    '    0,',
    '    rtl.longint,',
    '    "FI",',
    '    "",',
    '    {',
    '      Default: 2',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "I2",',
    '    0,',
    '    rtl.longint,',
    '    "FI",',
    '    "",',
    '    {',
    '      Default: 3',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "E1",',
    '    0,',
    '    $mod.$rtti["TEnum"],',
    '    "FE",',
    '    "",',
    '    {',
    '      Default: $mod.TEnum.red',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "E2",',
    '    0,',
    '    $mod.$rtti["TEnum"],',
    '    "FE",',
    '    "",',
    '    {',
    '      Default: $mod.TEnum.blue',
    '    }',
    '  );',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_DefaultValueSet;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TEnum = (red, blue);',
  '  TSet = set of TEnum;',
  'const',
  '  CSet = [red,blue];',
  'type',
  '  TObject = class',
  '    FSet: TSet;',
  '  published',
  '    property Set1: TSet read FSet default [];',
  '    property Set2: TSet read FSet default [red];',
  '    property Set3: TSet read FSet default [red,blue];',
  '    property Set4: TSet read FSet default CSet;',
  '  end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRTTI_DefaultValueSet',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "blue",',
    '  blue: 1',
    '};',
    'this.$rtti.$Enum("TEnum", {',
    '  minvalue: 0,',
    '  maxvalue: 1,',
    '  ordtype: 1,',
    '  enumtype: this.TEnum',
    '});',
    'this.$rtti.$Set("TSet", {',
    '  comptype: this.$rtti["TEnum"]',
    '});',
    'this.CSet = rtl.createSet(this.TEnum.red, this.TEnum.blue);',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FSet = {};',
    '  };',
    '  this.$final = function () {',
    '    this.FSet = undefined;',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addProperty(',
    '    "Set1",',
    '    0,',
    '    $mod.$rtti["TSet"],',
    '    "FSet",',
    '    "",',
    '    {',
    '      Default: {}',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "Set2",',
    '    0,',
    '    $mod.$rtti["TSet"],',
    '    "FSet",',
    '    "",',
    '    {',
    '      Default: rtl.createSet($mod.TEnum.red)',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "Set3",',
    '    0,',
    '    $mod.$rtti["TSet"],',
    '    "FSet",',
    '    "",',
    '    {',
    '      Default: rtl.createSet($mod.TEnum.red, $mod.TEnum.blue)',
    '    }',
    '  );',
    '  $r.addProperty(',
    '    "Set4",',
    '    0,',
    '    $mod.$rtti["TSet"],',
    '    "FSet",',
    '    "",',
    '    {',
    '      Default: $mod.CSet',
    '    }',
    '  );',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_DefaultValueRangeType;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TRg = -1..1;',
  'const',
  '  l = low(TRg);',
  '  h = high(TRg);',
  'type',
  '  TObject = class',
  '    FV: TRg;',
  '  published',
  '    property V1: TRg read FV default -1;',
  '  end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRTTI_DefaultValueRangeType',
    LinesToStr([ // statements
    'this.$rtti.$Int("TRg", {',
    '  minvalue: -1,',
    '  maxvalue: 1,',
    '  ordtype: 0',
    '});',
    'this.l = -1;',
    'this.h = 1;',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FV = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addProperty(',
    '    "V1",',
    '    0,',
    '    $mod.$rtti["TRg"],',
    '    "FV",',
    '    "",',
    '    {',
    '      Default: -1',
    '    }',
    '  );',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_DefaultValueInherit;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    FA, FB: byte;',
  '    property A: byte read FA default 1;',
  '    property B: byte read FB default 2;',
  '  end;',
  '  TBird = class',
  '  published',
  '    property A;',
  '    property B nodefault;',
  '  end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRTTI_DefaultValueInherit',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FA = 0;',
    '    this.FB = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  var $r = this.$rtti;',
    '  $r.addProperty(',
    '    "A",',
    '    0,',
    '    rtl.byte,',
    '    "FA",',
    '    "",',
    '    {',
    '      Default: 1',
    '    }',
    '  );',
    '  $r.addProperty("B", 0, rtl.byte, "FB", "");',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_OverrideMethod;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  published');
  Add('    procedure DoIt; virtual; abstract;');
  Add('  end;');
  Add('  TSky = class');
  Add('  published');
  Add('    procedure DoIt; override;');
  Add('  end;');
  Add('procedure TSky.DoIt; begin end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestRTTI_OverrideMethod',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addMethod("DoIt", 0, []);',
    '});',
    'rtl.createClass(this, "TSky", this.TObject, function () {',
    '  this.DoIt = function () {',
    '  };',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_ReintroduceMethod;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  published',
  '    procedure DoIt;',
  '  end;',
  '  TSky = class',
  '  published',
  '    procedure DoIt; reintroduce;',
  '  end;',
  'procedure TObject.DoIt; begin end;',
  'procedure TSky.DoIt;',
  'begin',
  '  inherited DoIt;',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRTTI_ReintroduceMethod',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addMethod("DoIt", 0, []);',
    '});',
    'rtl.createClass(this, "TSky", this.TObject, function () {',
    '  this.DoIt = function () {',
    '    $mod.TObject.DoIt.call(this);',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addMethod("DoIt", 0, []);',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_OverloadProperty;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TObject = class');
  Add('  protected');
  Add('    FFlag: longint;');
  Add('  published');
  Add('    property Flag: longint read fflag;');
  Add('  end;');
  Add('  TSky = class');
  Add('  published');
  Add('    property FLAG: longint write fflag;');
  Add('  end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestRTTI_OverrideMethod',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FFlag = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addProperty("Flag", 0, rtl.longint, "FFlag", "");',
    '});',
    'rtl.createClass(this, "TSky", this.TObject, function () {',
    '  var $r = this.$rtti;',
    '  $r.addProperty("Flag", 0, rtl.longint, "", "FFlag");',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_ClassForward;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TObject = class end;');
  Add('  tbridge = class;');
  Add('  TProc = function: tbridge;');
  Add('  TOger = class');
  Add('  published');
  Add('    FBridge: tbridge;');
  Add('    procedure SetBridge(Value: tbridge); virtual; abstract;');
  Add('    property Bridge: tbridge read fbridge write setbridge;');
  Add('  end;');
  Add('  TBridge = class');
  Add('    FOger: toger;');
  Add('  end;');
  Add('var p: Pointer;');
  Add(' b: tbridge;');
  Add('begin');
  Add('  p:=typeinfo(tbridge);');
  Add('  p:=typeinfo(b);');
  ConvertProgram;
  CheckSource('TestRTTI_ClassForward',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.$rtti.$Class("TBridge");',
    'this.$rtti.$ProcVar("TProc", {',
    '  procsig: rtl.newTIProcSig([], this.$rtti["TBridge"])',
    '});',
    'rtl.createClass(this, "TOger", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.FBridge = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FBridge = undefined;',
    '    $mod.TObject.$final.call(this);',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addField("FBridge", $mod.$rtti["TBridge"]);',
    '  $r.addMethod("SetBridge", 0, [["Value", $mod.$rtti["TBridge"]]]);',
    '  $r.addProperty("Bridge", 2, $mod.$rtti["TBridge"], "FBridge", "SetBridge");',
    '});',
    'rtl.createClass(this, "TBridge", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.FOger = null;',
    '  };',
    '  this.$final = function () {',
    '    this.FOger = undefined;',
    '    $mod.TObject.$final.call(this);',
    '  };',
    '});',
    'this.p = null;',
    'this.b = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TBridge"];',
    '$mod.p = $mod.b.$rtti;',
    '']));
end;

procedure TTestModule.TestRTTI_ClassOf;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TClass = class of tobject;');
  Add('  TProcA = function: TClass;');
  Add('  TObject = class');
  Add('  published');
  Add('    C: tclass;');
  Add('  end;');
  Add('  tfox = class;');
  Add('  TBird = class end;');
  Add('  TBirds = class of tbird;');
  Add('  TFox = class end;');
  Add('  TFoxes = class of tfox;');
  Add('  TCows = class of TCow;');
  Add('  TCow = class;');
  Add('  TCow = class end;');
  Add('begin');
  ConvertProgram;
  CheckSource('TestRTTI_ClassOf',
    LinesToStr([ // statements
    'this.$rtti.$Class("TObject");',
    'this.$rtti.$ClassRef("TClass", {',
    '  instancetype: this.$rtti["TObject"]',
    '});',
    'this.$rtti.$ProcVar("TProcA", {',
    '  procsig: rtl.newTIProcSig([], this.$rtti["TClass"])',
    '});',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.C = null;',
    '  };',
    '  this.$final = function () {',
    '    this.C = undefined;',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addField("C", $mod.$rtti["TClass"]);',
    '});',
    'this.$rtti.$Class("TFox");',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '});',
    'this.$rtti.$ClassRef("TBirds", {',
    '  instancetype: this.$rtti["TBird"]',
    '});',
    'rtl.createClass(this, "TFox", this.TObject, function () {',
    '});',
    'this.$rtti.$ClassRef("TFoxes", {',
    '  instancetype: this.$rtti["TFox"]',
    '});',
    'this.$rtti.$Class("TCow");',
    'this.$rtti.$ClassRef("TCows", {',
    '  instancetype: this.$rtti["TCow"]',
    '});',
    'rtl.createClass(this, "TCow", this.TObject, function () {',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_Record;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  integer = longint;');
  Add('  TPoint = record');
  Add('    x,y: integer;');
  Add('  end;');
  Add('var p: pointer;');
  Add('  r: tpoint;');
  Add('begin');
  Add('  p:=typeinfo(tpoint);');
  Add('  p:=typeinfo(r);');
  Add('  p:=typeinfo(r.x);');
  ConvertProgram;
  CheckSource('TestRTTI_Record',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TPoint", function () {',
    '  this.x = 0;',
    '  this.y = 0;',
    '  this.$eq = function (b) {',
    '    return (this.x === b.x) && (this.y === b.y);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    this.y = s.y;',
    '    return this;',
    '  };',
    '  var $r = $mod.$rtti.$Record("TPoint", {});',
    '  $r.addField("x", rtl.longint);',
    '  $r.addField("y", rtl.longint);',
    '});',
    'this.p = null;',
    'this.r = this.TPoint.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TPoint"];',
    '$mod.p = $mod.$rtti["TPoint"];',
    '$mod.p = rtl.longint;',
    '']));
end;

procedure TTestModule.TestRTTI_RecordAnonymousArray;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('type');
  Add('  TFloatRec = record');
  Add('    c,d: array of char;');
  // Add('    i: array of array of longint;');
  Add('  end;');
  Add('var p: pointer;');
  Add('  r: tfloatrec;');
  Add('begin');
  Add('  p:=typeinfo(tfloatrec);');
  Add('  p:=typeinfo(r);');
  Add('  p:=typeinfo(r.d);');
  ConvertProgram;
  CheckSource('TestRTTI_Record',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TFloatRec", function () {',
    '  this.$new = function () {',
    '    var r = Object.create(this);',
    '    r.c = [];',
    '    r.d = [];',
    '    return r;',
    '  };',
    '  this.$eq = function (b) {',
    '    return (this.c === b.c) && (this.d === b.d);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.c = rtl.arrayRef(s.c);',
    '    this.d = rtl.arrayRef(s.d);',
    '    return this;',
    '  };',
    '  var $r = $mod.$rtti.$Record("TFloatRec", {});',
    '  $mod.$rtti.$DynArray("TFloatRec.d$a", {',
    '    eltype: rtl.char',
    '  });',
    '  $r.addField("c", $mod.$rtti["TFloatRec.d$a"]);',
    '  $r.addField("d", $mod.$rtti["TFloatRec.d$a"]);',
    '});',
    'this.p = null;',
    'this.r = this.TFloatRec.$new();',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TFloatRec"];',
    '$mod.p = $mod.$rtti["TFloatRec"];',
    '$mod.p = $mod.$rtti["TFloatRec.d$a"];',
    '']));
end;

procedure TTestModule.TestRTTI_Record_ClassVarType;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  '{$modeswitch AdvancedRecords}',
  'type',
  '  TPoint = record',
  '    type TProc = procedure(w: word);',
  '    class var p: TProc;',
  '  end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestRTTI_Record_ClassVarType',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TPoint", function () {',
    '  $mod.$rtti.$ProcVar("TPoint.TProc", {',
    '    procsig: rtl.newTIProcSig([["w", rtl.word]])',
    '  });',
    '  this.p = null;',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  var $r = $mod.$rtti.$Record("TPoint", {});',
    '  $r.addField("p", $mod.$rtti["TPoint.TProc"]);',
    '}, true);',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_LocalTypes;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'procedure DoIt;',
  'type',
  '  integer = longint;',
  '  TPoint = record',
  '    x,y: integer;',
  '  end;',
  'var p: TPoint;',
  'begin',
  'end;',
  'begin']);
  ConvertProgram;
  CheckSource('TestRTTI_LocalTypes',
    LinesToStr([ // statements
    'var TPoint = rtl.recNewT(null, "", function () {',
    '  this.x = 0;',
    '  this.y = 0;',
    '  this.$eq = function (b) {',
    '    return (this.x === b.x) && (this.y === b.y);',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    this.y = s.y;',
    '    return this;',
    '  };',
    '});',
    'this.DoIt = function () {',
    '  var p = TPoint.$new();',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRTTI_TypeInfo_BaseTypes;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TCaption = string;',
  '  TYesNo = boolean;',
  '  TLetter = char;',
  '  TFloat = double;',
  '  TPtr = pointer;',
  '  TShortInt = shortint;',
  '  TByte = byte;',
  '  TSmallInt = smallint;',
  '  TWord = word;',
  '  TInt32 = longint;',
  '  TDWord = longword;',
  '  TValue = jsvalue;',
  'var p: TPtr;',
  'begin',
  '  p:=typeinfo(string);',
  '  p:=typeinfo(tcaption);',
  '  p:=typeinfo(boolean);',
  '  p:=typeinfo(tyesno);',
  '  p:=typeinfo(char);',
  '  p:=typeinfo(tletter);',
  '  p:=typeinfo(double);',
  '  p:=typeinfo(tfloat);',
  '  p:=typeinfo(pointer);',
  '  p:=typeinfo(tptr);',
  '  p:=typeinfo(shortint);',
  '  p:=typeinfo(tshortint);',
  '  p:=typeinfo(byte);',
  '  p:=typeinfo(tbyte);',
  '  p:=typeinfo(smallint);',
  '  p:=typeinfo(tsmallint);',
  '  p:=typeinfo(word);',
  '  p:=typeinfo(tword);',
  '  p:=typeinfo(longword);',
  '  p:=typeinfo(tdword);',
  '  p:=typeinfo(jsvalue);',
  '  p:=typeinfo(tvalue);',
  '']);
  ConvertProgram;
  CheckSource('TestRTTI_TypeInfo_BaseTypes',
    LinesToStr([ // statements
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = rtl.string;',
    '$mod.p = rtl.string;',
    '$mod.p = rtl.boolean;',
    '$mod.p = rtl.boolean;',
    '$mod.p = rtl.char;',
    '$mod.p = rtl.char;',
    '$mod.p = rtl.double;',
    '$mod.p = rtl.double;',
    '$mod.p = rtl.pointer;',
    '$mod.p = rtl.pointer;',
    '$mod.p = rtl.shortint;',
    '$mod.p = rtl.shortint;',
    '$mod.p = rtl.byte;',
    '$mod.p = rtl.byte;',
    '$mod.p = rtl.smallint;',
    '$mod.p = rtl.smallint;',
    '$mod.p = rtl.word;',
    '$mod.p = rtl.word;',
    '$mod.p = rtl.longword;',
    '$mod.p = rtl.longword;',
    '$mod.p = rtl.jsvalue;',
    '$mod.p = rtl.jsvalue;',
    '']));
end;

procedure TTestModule.TestRTTI_TypeInfo_Type_BaseTypes;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  'type',
  '  TCaption = type string;',
  '  TYesNo = type boolean;',
  '  TLetter = type char;',
  '  TFloat = type double;',
  '  TPtr = type pointer;',
  '  TShortInt = type shortint;',
  '  TByte = type byte;',
  '  TSmallInt = type smallint;',
  '  TWord = type word;',
  '  TInt32 = type longint;',
  '  TDWord = type longword;',
  '  TValue = type jsvalue;',
  '  TAliasValue = type TValue;',
  'var',
  '  p: TPtr;',
  '  a: TAliasValue;',
  'begin',
  '  p:=typeinfo(tcaption);',
  '  p:=typeinfo(tyesno);',
  '  p:=typeinfo(tletter);',
  '  p:=typeinfo(tfloat);',
  '  p:=typeinfo(tptr);',
  '  p:=typeinfo(tshortint);',
  '  p:=typeinfo(tbyte);',
  '  p:=typeinfo(tsmallint);',
  '  p:=typeinfo(tword);',
  '  p:=typeinfo(tdword);',
  '  p:=typeinfo(tvalue);',
  '  p:=typeinfo(taliasvalue);',
  '  p:=typeinfo(a);',
  '']);
  ConvertProgram;
  CheckSource('TestRTTI_TypeInfo_Type_BaseTypes',
    LinesToStr([ // statements
    'this.$rtti.$inherited("TCaption", rtl.string, {});',
    'this.$rtti.$inherited("TYesNo", rtl.boolean, {});',
    'this.$rtti.$inherited("TLetter", rtl.char, {});',
    'this.$rtti.$inherited("TFloat", rtl.double, {});',
    'this.$rtti.$inherited("TPtr", rtl.pointer, {});',
    'this.$rtti.$inherited("TShortInt", rtl.shortint, {});',
    'this.$rtti.$inherited("TByte", rtl.byte, {});',
    'this.$rtti.$inherited("TSmallInt", rtl.smallint, {});',
    'this.$rtti.$inherited("TWord", rtl.word, {});',
    'this.$rtti.$inherited("TInt32", rtl.longint, {});',
    'this.$rtti.$inherited("TDWord", rtl.longword, {});',
    'this.$rtti.$inherited("TValue", rtl.jsvalue, {});',
    'this.$rtti.$inherited("TAliasValue", this.$rtti["TValue"], {});',
    'this.p = null;',
    'this.a = undefined;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TCaption"];',
    '$mod.p = $mod.$rtti["TYesNo"];',
    '$mod.p = $mod.$rtti["TLetter"];',
    '$mod.p = $mod.$rtti["TFloat"];',
    '$mod.p = $mod.$rtti["TPtr"];',
    '$mod.p = $mod.$rtti["TShortInt"];',
    '$mod.p = $mod.$rtti["TByte"];',
    '$mod.p = $mod.$rtti["TSmallInt"];',
    '$mod.p = $mod.$rtti["TWord"];',
    '$mod.p = $mod.$rtti["TDWord"];',
    '$mod.p = $mod.$rtti["TValue"];',
    '$mod.p = $mod.$rtti["TAliasValue"];',
    '$mod.p = $mod.$rtti["TAliasValue"];',
    '']));
end;

procedure TTestModule.TestRTTI_TypeInfo_LocalFail;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add('procedure DoIt;');
  Add('type');
  Add('  integer = longint;');
  Add('  TPoint = record');
  Add('    x,y: integer;');
  Add('  end;');
  Add('var p: pointer;');
  Add('begin');
  Add('  p:=typeinfo(tpoint);');
  Add('end;');
  Add('begin');
  SetExpectedPasResolverError(sSymbolCannotBePublished,nSymbolCannotBePublished);
  ConvertProgram;
end;

procedure TTestModule.TestRTTI_TypeInfo_ExtTypeInfoClasses1;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TFlag = (up,down);',
  '  TFlags = set of TFlag;',
  'var',
  '  ti: TTypeInfo;',
  '  tiInt: TTypeInfoInteger;',
  '  tiEnum: TTypeInfoEnum;',
  '  tiSet: TTypeInfoSet;',
  'begin',
  '  ti:=typeinfo(string);',
  '  ti:=typeinfo(boolean);',
  '  ti:=typeinfo(char);',
  '  ti:=typeinfo(double);',
  '  tiInt:=typeinfo(shortint);',
  '  tiInt:=typeinfo(byte);',
  '  tiInt:=typeinfo(smallint);',
  '  tiInt:=typeinfo(word);',
  '  tiInt:=typeinfo(longint);',
  '  tiInt:=typeinfo(longword);',
  '  ti:=typeinfo(jsvalue);',
  '  tiEnum:=typeinfo(tflag);',
  '  tiSet:=typeinfo(tflags);']);
  ConvertProgram;
  CheckSource('TestRTTI_TypeInfo_ExtTypeInfoClasses1',
    LinesToStr([ // statements
    'this.TFlag = {',
    '  "0": "up",',
    '  up: 0,',
    '  "1": "down",',
    '  down: 1',
    '};',
    'this.$rtti.$Enum("TFlag", {',
    '  minvalue: 0,',
    '  maxvalue: 1,',
    '  ordtype: 1,',
    '  enumtype: this.TFlag',
    '});',
    'this.$rtti.$Set("TFlags", {',
    '  comptype: this.$rtti["TFlag"]',
    '});',
    'this.ti = null;',
    'this.tiInt = null;',
    'this.tiEnum = null;',
    'this.tiSet = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.ti = rtl.string;',
    '$mod.ti = rtl.boolean;',
    '$mod.ti = rtl.char;',
    '$mod.ti = rtl.double;',
    '$mod.tiInt = rtl.shortint;',
    '$mod.tiInt = rtl.byte;',
    '$mod.tiInt = rtl.smallint;',
    '$mod.tiInt = rtl.word;',
    '$mod.tiInt = rtl.longint;',
    '$mod.tiInt = rtl.longword;',
    '$mod.ti = rtl.jsvalue;',
    '$mod.tiEnum = $mod.$rtti["TFlag"];',
    '$mod.tiSet = $mod.$rtti["TFlags"];',
    '']));
end;

procedure TTestModule.TestRTTI_TypeInfo_ExtTypeInfoClasses2;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TStaticArr = array[boolean] of string;');
  Add('  TDynArr = array of string;');
  Add('  TProc = procedure;');
  Add('  TMethod = procedure of object;');
  Add('var');
  Add('  StaticArray: TStaticArr;');
  Add('  tiStaticArray: TTypeInfoStaticArray;');
  Add('  DynArray: TDynArr;');
  Add('  tiDynArray: TTypeInfoDynArray;');
  Add('  ProcVar: TProc;');
  Add('  tiProcVar: TTypeInfoProcVar;');
  Add('  MethodVar: TMethod;');
  Add('  tiMethodVar: TTypeInfoMethodVar;');
  Add('begin');
  Add('  tiStaticArray:=typeinfo(StaticArray);');
  Add('  tiStaticArray:=typeinfo(TStaticArr);');
  Add('  tiDynArray:=typeinfo(DynArray);');
  Add('  tiDynArray:=typeinfo(TDynArr);');
  Add('  tiProcVar:=typeinfo(ProcVar);');
  Add('  tiProcVar:=typeinfo(TProc);');
  Add('  tiMethodVar:=typeinfo(MethodVar);');
  Add('  tiMethodVar:=typeinfo(TMethod);');
  ConvertProgram;
  CheckSource('TestRTTI_TypeInfo_ExtTypeInfoClasses2',
    LinesToStr([ // statements
    'this.$rtti.$StaticArray("TStaticArr", {',
    '  dims: [2],',
    '  eltype: rtl.string',
    '});',
    'this.$rtti.$DynArray("TDynArr", {',
    '  eltype: rtl.string',
    '});',
    'this.$rtti.$ProcVar("TProc", {',
    '  procsig: rtl.newTIProcSig([])',
    '});',
    'this.$rtti.$MethodVar("TMethod", {',
    '  procsig: rtl.newTIProcSig([]),',
    '  methodkind: 0',
    '});',
    'this.StaticArray = rtl.arraySetLength(null,"",2);',
    'this.tiStaticArray = null;',
    'this.DynArray = [];',
    'this.tiDynArray = null;',
    'this.ProcVar = null;',
    'this.tiProcVar = null;',
    'this.MethodVar = null;',
    'this.tiMethodVar = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.tiStaticArray = $mod.$rtti["TStaticArr"];',
    '$mod.tiStaticArray = $mod.$rtti["TStaticArr"];',
    '$mod.tiDynArray = $mod.$rtti["TDynArr"];',
    '$mod.tiDynArray = $mod.$rtti["TDynArr"];',
    '$mod.tiProcVar = $mod.$rtti["TProc"];',
    '$mod.tiProcVar = $mod.$rtti["TProc"];',
    '$mod.tiMethodVar = $mod.$rtti["TMethod"];',
    '$mod.tiMethodVar = $mod.$rtti["TMethod"];',
    '']));
end;

procedure TTestModule.TestRTTI_TypeInfo_ExtTypeInfoClasses3;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add('{$modeswitch externalclass}');
  Add('type');
  Add('  TRec = record end;');
  // ToDo: ^TRec
  Add('  TObject = class end;');
  Add('  TClass = class of tobject;');
  Add('var');
  Add('  Rec: trec;');
  Add('  tiRecord: ttypeinforecord;');
  Add('  Obj: tobject;');
  Add('  tiClass: ttypeinfoclass;');
  Add('  aClass: tclass;');
  Add('  tiClassRef: ttypeinfoclassref;');
  // ToDo: ^TRec
  Add('  tiPointer: ttypeinfopointer;');
  Add('begin');
  Add('  tirecord:=typeinfo(trec);');
  Add('  tirecord:=typeinfo(trec);');
  Add('  ticlass:=typeinfo(obj);');
  Add('  ticlass:=typeinfo(tobject);');
  Add('  ticlass:=typeinfo(aclass);');
  Add('  ticlassref:=typeinfo(tclass);');
  ConvertProgram;
  CheckSource('TestRTTI_TypeInfo_ExtTypeInfoClasses3',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  $mod.$rtti.$Record("TRec", {});',
    '});',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.$rtti.$ClassRef("TClass", {',
    '  instancetype: this.$rtti["TObject"]',
    '});',
    'this.Rec = this.TRec.$new();',
    'this.tiRecord = null;',
    'this.Obj = null;',
    'this.tiClass = null;',
    'this.aClass = null;',
    'this.tiClassRef = null;',
    'this.tiPointer = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.tiRecord = $mod.$rtti["TRec"];',
    '$mod.tiRecord = $mod.$rtti["TRec"];',
    '$mod.tiClass = $mod.Obj.$rtti;',
    '$mod.tiClass = $mod.$rtti["TObject"];',
    '$mod.tiClass = $mod.aClass.$rtti;',
    '$mod.tiClassRef = $mod.$rtti["TClass"];',
    '']));
end;

procedure TTestModule.TestRTTI_TypeInfo_FunctionClassType;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TClass = class of tobject;',
  '  TObject = class',
  '    function MyClass: TClass;',
  '    class function ClassType: TClass;',
  '  end;',
  'function TObject.MyClass: TClass;',
  'var t: TTypeInfoClass;',
  'begin',
  '  t:=TypeInfo(Self);',
  '  t:=TypeInfo(Result);',
  '  t:=TypeInfo(TObject);',
  'end;',
  'class function TObject.ClassType: TClass;',
  'var t: TTypeInfoClass;',
  'begin',
  '  t:=TypeInfo(Self);',
  '  t:=TypeInfo(Result);',
  'end;',
  'var',
  '  Obj: TObject;',
  '  t: TTypeInfoClass;',
  'begin',
  '  t:=TypeInfo(TObject.ClassType);',
  '  t:=TypeInfo(Obj.ClassType);',
  '  t:=TypeInfo(Obj.MyClass);',
  '']);
  ConvertProgram;
  CheckSource('TestRTTI_TypeInfo_FunctionClassType',
    LinesToStr([ // statements
    'this.$rtti.$Class("TObject");',
    'this.$rtti.$ClassRef("TClass", {',
    '  instancetype: this.$rtti["TObject"]',
    '});',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.MyClass = function () {',
    '    var Result = null;',
    '    var t = null;',
    '    t = this.$rtti;',
    '    t = Result.$rtti;',
    '    t = $mod.$rtti["TObject"];',
    '    return Result;',
    '  };',
    '  this.ClassType = function () {',
    '    var Result = null;',
    '    var t = null;',
    '    t = this.$rtti;',
    '    t = Result.$rtti;',
    '    return Result;',
    '  };',
    '});',
    'this.Obj = null;',
    'this.t = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.t = $mod.TObject.ClassType().$rtti;',
    '$mod.t = $mod.Obj.$class.ClassType().$rtti;',
    '$mod.t = $mod.Obj.MyClass().$rtti;',
    '']));
end;

procedure TTestModule.TestRTTI_TypeInfo_MixedUnits_PointerAndClass;
begin
  WithTypeInfo:=true;
  AddModuleWithIntfImplSrc('typinfo.pas',
    LinesToStr([
    '{$modeswitch externalclass}',
    'type',
    '  TTypeInfo = class external name ''rtl.tTypeInfo'' end;',
    '  TTypeInfoInteger = class external name ''rtl.tTypeInfoInteger''(TTypeInfo) end;',
    '']),
    '');
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'uses typinfo;',
    'type PTypeInfo = TTypeInfo;', // delphi compatibility code
    'procedure DoPtr(p: PTypeInfo);',
    'procedure DoInfo(t: TTypeInfo);',
    'procedure DoInt(t: TTypeInfoInteger);',
    '']),
    LinesToStr([
    'procedure DoPtr(p: PTypeInfo);',
    'begin end;',
    'procedure DoInfo(t: TTypeInfo);',
    'begin end;',
    'procedure DoInt(t: TTypeInfoInteger);',
    'begin end;',
    '']));
  StartUnit(true);
  Add([
  'interface',
  'uses unit2;', // does not use unit typinfo
  'implementation',
  'var',
  '  i: byte;',
  '  p: pointer;',
  '  t: PTypeInfo;',
  'initialization',
  '  p:=typeinfo(i);',
  '  t:=typeinfo(i);',
  '  if p=t then ;',
  '  if p=typeinfo(i) then ;',
  '  if typeinfo(i)=p then ;',
  '  if t=typeinfo(i) then ;',
  '  if typeinfo(i)=t then ;',
  '  DoPtr(p);',
  '  DoPtr(t);',
  '  DoPtr(typeinfo(i));',
  '  DoInfo(p);',
  '  DoInfo(t);',
  '  DoInfo(typeinfo(i));',
  '  DoInt(typeinfo(i));',
  '']);
  ConvertUnit;
  CheckSource('TestRTTI_TypeInfo_MixedUnits_PointerAndClass',
    LinesToStr([ // statements
    'var $impl = $mod.$impl;',
    '']),
    LinesToStr([ // this.$init
    '$impl.p = rtl.byte;',
    '$impl.t = rtl.byte;',
    'if ($impl.p === $impl.t) ;',
    'if ($impl.p === rtl.byte) ;',
    'if (rtl.byte === $impl.p) ;',
    'if ($impl.t === rtl.byte) ;',
    'if (rtl.byte === $impl.t) ;',
    'pas.unit2.DoPtr($impl.p);',
    'pas.unit2.DoPtr($impl.t);',
    'pas.unit2.DoPtr(rtl.byte);',
    'pas.unit2.DoInfo($impl.p);',
    'pas.unit2.DoInfo($impl.t);',
    'pas.unit2.DoInfo(rtl.byte);',
    'pas.unit2.DoInt(rtl.byte);',
    '']),
    LinesToStr([ // implementation
    '$impl.i = 0;',
    '$impl.p = null;',
    '$impl.t = null;',
    '']) );
end;

procedure TTestModule.TestRTTI_Interface_Corba;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add([
  '{$interfaces corba}',
  '{$modeswitch externalclass}',
  'type',
  '  IUnknown = interface',
  '  end;',
  '  IBird = interface',
  '    function GetItem: longint;',
  '    procedure SetItem(Value: longint);',
  '    property Item: longint read GetItem write SetItem;',
  '  end;',
  'procedure DoIt(t: TTypeInfoInterface); begin end;',
  'var',
  '  i: IBird;',
  '  t: TTypeInfoInterface;',
  'begin',
  '  t:=TypeInfo(IBird);',
  '  t:=TypeInfo(i);',
  '  DoIt(t);',
  '  DoIt(TypeInfo(IBird));',
  '']);
  ConvertProgram;
  CheckSource('TestRTTI_Interface_Corba',
    LinesToStr([ // statements
    'rtl.createInterface(',
    '  this,',
    '  "IUnknown",',
    '  "{B92D5841-758A-322B-B800-000000000000}",',
    '  [],',
    '  null,',
    '  function () {',
    '  }',
    ');',
    'rtl.createInterface(',
    '  this,',
    '  "IBird",',
    '  "{D32D5841-6264-3AE3-A2C9-B91CE922C9B9}",',
    '  ["GetItem", "SetItem"],',
    '  null,',
    '  function () {',
    '    var $r = this.$rtti;',
    '    $r.addMethod("GetItem", 1, [], rtl.longint);',
    '    $r.addMethod("SetItem", 0, [["Value", rtl.longint]]);',
    '    $r.addProperty("Item", 3, rtl.longint, "GetItem", "SetItem");',
    '  }',
    ');',
    'this.DoIt = function (t) {',
    '};    ',
    'this.i = null;',
    'this.t = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.t = $mod.$rtti["IBird"];',
    '$mod.t = $mod.i.$rtti;',
    '$mod.DoIt($mod.t);',
    '$mod.DoIt($mod.$rtti["IBird"]);',
    '']));
end;

procedure TTestModule.TestRTTI_Interface_COM;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add([
  '{$interfaces com}',
  '{$modeswitch externalclass}',
  'type',
  '  TGuid = record end;',
  '  integer = longint;',
  '  IUnknown = interface',
  '    function QueryInterface(const iid: TGuid; out obj): Integer;',
  '    function _AddRef: Integer;',
  '    function _Release: Integer;',
  '  end;',
  '  IBird = interface',
  '    function GetItem: longint;',
  '    procedure SetItem(Value: longint);',
  '    property Item: longint read GetItem write SetItem;',
  '  end;',
  'var',
  '  i: IBird;',
  '  t: TTypeInfoInterface;',
  'begin',
  '  t:=TypeInfo(IBird);',
  '  t:=TypeInfo(i);',
  '']);
  ConvertProgram;
  CheckSource('TestRTTI_Interface_COM',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TGuid", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  $mod.$rtti.$Record("TGuid", {});',
    '});',
    'rtl.createInterface(',
    '  this,',
    '  "IUnknown",',
    '  "{D7ADB00D-1A9B-3EDC-B123-730E661DDFA9}",',
    '  ["QueryInterface", "_AddRef", "_Release"],',
    '  null,',
    '  function () {',
    '    this.$kind = "com";',
    '    var $r = this.$rtti;',
    '    $r.addMethod("QueryInterface", 1, [["iid", $mod.$rtti["TGuid"], 2], ["obj", null, 4]], rtl.longint);',
    '    $r.addMethod("_AddRef", 1, [], rtl.longint);',
    '    $r.addMethod("_Release", 1, [], rtl.longint);',
    '  }',
    ');',
    'rtl.createInterface(',
    '  this,',
    '  "IBird",',
    '  "{9CC77572-0E45-3594-9A88-9E8D865C9E0A}",',
    '  ["GetItem", "SetItem"],',
    '  this.IUnknown,',
    '  function () {',
    '    var $r = this.$rtti;',
    '    $r.addMethod("GetItem", 1, [], rtl.longint);',
    '    $r.addMethod("SetItem", 0, [["Value", rtl.longint]]);',
    '    $r.addProperty("Item", 3, rtl.longint, "GetItem", "SetItem");',
    '  }',
    ');',
    'this.i = null;',
    'this.t = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.t = $mod.$rtti["IBird"];',
    '$mod.t = $mod.i.$rtti;',
    '']));
end;

procedure TTestModule.TestRTTI_ClassHelper;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add([
  '{$interfaces com}',
  '{$modeswitch externalclass}',
  'type',
  '  TObject = class',
  '  end;',
  '  THelper = class helper for TObject',
  '  published',
  '    function GetItem: longint;',
  '    property Item: longint read GetItem;',
  '  end;',
  'function THelper.GetItem: longint;',
  'begin',
  'end;',
  'var',
  '  t: TTypeInfoHelper;',
  'begin',
  '  t:=TypeInfo(THelper);',
  '']);
  ConvertProgram;
  CheckSource('TestRTTI_ClassHelper',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.GetItem = function () {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addMethod("GetItem", 1, [], rtl.longint);',
    '  $r.addProperty("Item", 1, rtl.longint, "GetItem", "");',
    '});',
    'this.t = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.t = $mod.$rtti["THelper"];',
    '']));
end;

procedure TTestModule.TestRTTI_ExternalClass;
begin
  WithTypeInfo:=true;
  StartProgram(true,[supTypeInfo]);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSObject = class external name ''Object''',
  '  end;',
  '  TJSArray = class external name ''Array'' (TJSObject)',
  '  end;',
  'var',
  '  p: Pointer;',
  '  tc: TTypeInfoExtClass;',
  'begin',
  '  p:=typeinfo(TJSArray);']);
  ConvertProgram;
  CheckSource('TestRTTI_ExternalClass',
    LinesToStr([ // statements
    'this.$rtti.$ExtClass("TJSObject", {',
    '  jsclass: "Object"',
    '});',
    'this.$rtti.$ExtClass("TJSArray", {',
    '  ancestor: this.$rtti["TJSObject"],',
    '  jsclass: "Array"',
    '});',
    'this.p = null;',
    'this.tc = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TJSArray"];',
    '']));
end;

procedure TTestModule.TestRTTI_Unit;
begin
  WithTypeInfo:=true;
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    '{$mode delphi}',
    'type',
    '  TWordArray = array of word;',
    '  TArray<T> = array of T;',
    '']),
    '');
  StartUnit(true,[supTypeInfo,supTInterfacedObject]);
  Add([
  '{$mode delphi}',
  'interface',
  'uses unit2;',
  'type',
  '  IBird = interface',
  '    function Swoop: TWordArray;',
  '    function Glide: TArray<word>;',
  '  end;',
  'procedure Fly;',
  'implementation',
  'procedure Fly;',
  'var',
  '  ta: tTypeInfoDynArray;',
  '  ti: tTypeInfoInterface;',
  'begin',
  '  ta:=typeinfo(TWordArray);',
  '  ta:=typeinfo(TArray<word>);',
  '  ti:=typeinfo(IBird);',
  'end;',
  '']);
  ConvertUnit;
  CheckSource('TestRTTI_ExternalClass',
    LinesToStr([ // statements
    'rtl.createInterface(',
    '  this,',
    '  "IBird",',
    '  "{3B98AAAC-6116-3E17-AA85-F16786D85B09}",',
    '  ["Swoop", "Glide"],',
    '  pas.system.IUnknown,',
    '  function () {',
    '    var $r = this.$rtti;',
    '    $r.addMethod("Swoop", 1, [], pas.unit2.$rtti["TWordArray"]);',
    '    $r.addMethod("Glide", 1, [], pas.unit2.$rtti["TArray<System.Word>"]);',
    '  }',
    ');',
    'this.Fly = function () {',
    '  var ta = null;',
    '  var ti = null;',
    '  ta = pas.unit2.$rtti["TWordArray"];',
    '  ta = pas.unit2.$rtti["TArray<System.Word>"];',
    '  ti = $mod.$rtti["IBird"];',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestResourcestringProgram;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'resourcestring Title = ''Nice'';',
    '']),
    '');
  StartProgram(true);
  Add([
  'uses unit2;',
  'const Bar = ''bar'';',
  'resourcestring',
  '  Red = ''red'';',
  '  Foobar = ''fOo''+bar;',
  'var s: string;',
  '  c: char;',
  'begin',
  '  s:=red;',
  '  s:=test1.red;',
  '  s:=Title;',
  '  c:=red[1];',
  '  c:=test1.red[2];',
  '  if red=foobar then ;',
  '  if red[3]=red[4] then ;']);
  ConvertProgram;
  CheckSource('TestResourcestringProgram',
    LinesToStr([ // statements
    'this.Bar = "bar";',
    'this.s = "";',
    'this.c = "";',
    '$mod.$resourcestrings = {',
    '  Red: {',
    '      org: "red"',
    '    },',
    '  Foobar: {',
    '      org: "fOobar"',
    '    }',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.s = rtl.getResStr($mod, "Red");',
    '$mod.s = rtl.getResStr($mod, "Red");',
    '$mod.s = rtl.getResStr(pas.unit2, "Title");',
    '$mod.c = rtl.getResStr($mod, "Red").charAt(0);',
    '$mod.c = rtl.getResStr($mod, "Red").charAt(1);',
    'if (rtl.getResStr($mod, "Red") === rtl.getResStr($mod, "Foobar")) ;',
    'if (rtl.getResStr($mod, "Red").charAt(2) === rtl.getResStr($mod, "Red").charAt(3)) ;',
    '']));
end;

procedure TTestModule.TestResourcestringUnit;
begin
  AddModuleWithIntfImplSrc('unit2.pas',
    LinesToStr([
    'resourcestring Title = ''Nice'';',
    '']),
    '');
  StartUnit(true);
  Add([
  'interface',
  'uses unit2;',
  'const Red = ''rEd'';',
  'resourcestring',
  '  Blue = ''blue'';',
  '  NotRed = ''not''+Red;',
  'var s: string;',
  'implementation',
  'resourcestring',
  '  ImplGreen = ''green'';',
  'initialization',
  '  s:=blue+ImplGreen;',
  '  s:=test1.blue+test1.implgreen;',
  '  s:=blue[1]+implgreen[2];',
  '  s:=Title;',
  '']);
  ConvertUnit;
  CheckSource('TestResourcestringUnit',
    LinesToStr([ // statements
    'this.Red = "rEd";',
    'this.s = "";',
    '$mod.$resourcestrings = {',
    '  Blue: {',
    '      org: "blue"',
    '    },',
    '  NotRed: {',
    '      org: "notrEd"',
    '    },',
    '  ImplGreen: {',
    '      org: "green"',
    '    }',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.s = rtl.getResStr($mod, "Blue") + rtl.getResStr($mod, "ImplGreen");',
    '$mod.s = rtl.getResStr($mod, "Blue") + rtl.getResStr($mod, "ImplGreen");',
    '$mod.s = rtl.getResStr($mod, "Blue").charAt(0) + rtl.getResStr($mod, "ImplGreen").charAt(1);',
    '$mod.s = rtl.getResStr(pas.unit2, "Title");',
    '']));
end;

procedure TTestModule.TestResourcestringImplementation;
begin
  StartUnit(false);
  Add([
  'interface',
  'implementation',
  'resourcestring',
  '  ImplRed = ''red'';']);
  ConvertUnit;
  CheckSource('TestResourcestringImplementation',
    LinesToStr([ // intf statements
    'var $impl = $mod.$impl;']),
    LinesToStr([ // $mod.$init
    '']),
    LinesToStr([ // impl statements
    '$mod.$resourcestrings = {',
    '  ImplRed: {',
    '      org: "red"',
    '    }',
    '};',
    '']));
end;

procedure TTestModule.TestAttributes_Members;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  '{$modeswitch PrefixedAttributes}',
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  '  TCustomAttribute = class',
  '    constructor Create(Id: word);',
  '  end;',
  '  [Missing]',
  '  TBird = class',
  '  published',
  '    [Tcustom]',
  '    FField: word;',
  '    [tcustom(14)]',
  '    property Size: word read FField;',
  '    [Tcustom(15)]',
  '    procedure Fly; virtual; abstract;',
  '  end;',
  '  TRec = record',
  '    [Tcustom,tcustom(14)]',
  '    Size: word;',
  '  end;',
  'constructor TObject.Create; begin end;',
  'constructor TCustomAttribute.Create(Id: word); begin end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestAttributes_Members',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function () {',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TCustomAttribute", this.TObject, function () {',
    '  this.Create$1 = function (Id) {',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.$init = function () {',
    '    $mod.TObject.$init.call(this);',
    '    this.FField = 0;',
    '  };',
    '  var $r = this.$rtti;',
    '  $r.addField("FField", rtl.word, {',
    '    attr: [$mod.TCustomAttribute, "Create"]',
    '  });',
    '  $r.addProperty(',
    '    "Size",',
    '    0,',
    '    rtl.word,',
    '    "FField",',
    '    "",',
    '    {',
    '      attr: [$mod.TCustomAttribute, "Create$1", [14]]',
    '    }',
    '  );',
    '  $r.addMethod("Fly", 0, [], null, 0, {',
    '    attr: [$mod.TCustomAttribute, "Create$1", [15]]',
    '  });',
    '});',
    'rtl.recNewT(this, "TRec", function () {',
    '  this.Size = 0;',
    '  this.$eq = function (b) {',
    '    return this.Size === b.Size;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.Size = s.Size;',
    '    return this;',
    '  };',
    '  var $r = $mod.$rtti.$Record("TRec", {});',
    '  $r.addField("Size", rtl.word, {',
    '    attr: [',
    '        $mod.TCustomAttribute,',
    '        "Create",',
    '        $mod.TCustomAttribute,',
    '        "Create$1",',
    '        [14]',
    '      ]',
    '  });',
    '});',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestAttributes_Types;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  '{$modeswitch PrefixedAttributes}',
  'type',
  '  TObject = class',
  '    constructor Create(Id: word);',
  '  end;',
  '  TCustomAttribute = class',
  '  end;',
  '  [TCustom(1)]',
  '  TMyClass = class',
  '  end;',
  '  [TCustom(11)]',
  '  TMyDescendant = class(TMyClass)',
  '  end;',
  '  [TCustom(2)]',
  '  TRec = record',
  '  end;',
  '  [TCustom(3)]',
  '  TInt = type word;',
  'constructor TObject.Create(Id: word);',
  'begin',
  'end;',
  'var p: pointer;',
  'begin',
  '  p:=typeinfo(TMyClass);',
  '  p:=typeinfo(TRec);',
  '  p:=typeinfo(TInt);',
  '']);
  ConvertProgram;
  CheckSource('TestAttributes_Types',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Create = function (Id) {',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TCustomAttribute", this.TObject, function () {',
    '});',
    'rtl.createClass(this, "TMyClass", this.TObject, function () {',
    '  var $r = this.$rtti;',
    '  $r.attr = [$mod.TCustomAttribute, "Create", [1]];',
    '});',
    'rtl.createClass(this, "TMyDescendant", this.TMyClass, function () {',
    '  var $r = this.$rtti;',
    '  $r.attr = [$mod.TCustomAttribute, "Create", [11]];',
    '});',
    'rtl.recNewT(this, "TRec", function () {',
    '  this.$eq = function (b) {',
    '    return true;',
    '  };',
    '  this.$assign = function (s) {',
    '    return this;',
    '  };',
    '  $mod.$rtti.$Record("TRec", {',
    '    attr: [$mod.TCustomAttribute, "Create", [2]]',
    '  });',
    '});',
    'this.$rtti.$inherited("TInt", rtl.word, {',
    '  attr: [this.TCustomAttribute, "Create", [3]]',
    '});',
    'this.p = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.p = $mod.$rtti["TMyClass"];',
    '$mod.p = $mod.$rtti["TRec"];',
    '$mod.p = $mod.$rtti["TInt"];',
    '']));
end;

procedure TTestModule.TestAttributes_HelperConstructor_Fail;
begin
  WithTypeInfo:=true;
  StartProgram(false);
  Add([
  '{$modeswitch PrefixedAttributes}',
  'type',
  '  TObject = class',
  '    constructor Create;',
  '  end;',
  '  TCustomAttribute = class',
  '  end;',
  '  THelper = class helper for TCustomAttribute',
  '    constructor Create(Id: word);',
  '  end;',
  '  [TCustom(3)]',
  '  TMyInt = word;',
  'constructor TObject.Create; begin end;',
  'constructor THelper.Create(Id: word); begin end;',
  'begin',
  '  if typeinfo(TMyInt)=nil then ;']);
  ConvertProgram;
end;

procedure TTestModule.TestAssert;
begin
  StartProgram(false);
  Add([
  'procedure DoIt;',
  'var',
  '  b: boolean;',
  '  s: string;',
  'begin',
  '  {$Assertions on}',
  '  Assert(b);',
  'end;',
  'begin',
  '  DoIt;',
  '']);
  ConvertProgram;
  CheckSource('TestAssert',
    LinesToStr([ // statements
    'this.DoIt = function () {',
    '  var b = false;',
    '  var s = "";',
    '  if (!b) throw "assert failed";',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt();',
    '']));
end;

procedure TTestModule.TestAssert_SysUtils;
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
  '  Assert(b,''msg'');',
  'end;',
  'begin',
  '  DoIt;',
  '']);
  ConvertProgram;
  CheckSource('TestAssert_SysUtils',
    LinesToStr([ // statements
    'this.DoIt = function () {',
    '  var b = false;',
    '  var s = "";',
    '  if (!b) throw pas.SysUtils.EAssertionFailed.$create("Create");',
    '  if (!b) throw pas.SysUtils.EAssertionFailed.$create("Create$1", ["msg"]);',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.DoIt();',
    '']));
end;

procedure TTestModule.TestObjectChecks;
begin
  Scanner.CurrentBoolSwitches:=Scanner.CurrentBoolSwitches+[bsObjectChecks];
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    procedure DoIt;',
  '  end;',
  '  TClass = class of tobject;',
  '  TBird = class',
  '  end;',
  '  TBirdClass = class of TBird;',
  'var',
  '  o : TObject;',
  '  c: TClass;',
  '  b: TBird;',
  '  bc: TBirdClass;',
  'procedure TObject.DoIt;',
  'begin',
  '  b:=TBird(o);',
  'end;',
  'begin',
  '  o.DoIt;',
  '  b:=TBird(o);',
  '  bc:=TBirdClass(c);',
  '']);
  ConvertProgram;
  CheckSource('TestCheckMethodCall',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.DoIt = function () {',
    '    rtl.checkMethodCall(this,$mod.TObject);',
    '    $mod.b = rtl.asExt($mod.o, $mod.TBird, 1);',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '});',
    'this.o = null;',
    'this.c = null;',
    'this.b = null;',
    'this.bc = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.o.DoIt();',
    '$mod.b = rtl.asExt($mod.o,$mod.TBird, 1);',
    '$mod.bc = rtl.asExt($mod.c, $mod.TBird, 2);',
    '']));
end;

procedure TTestModule.TestOverflowChecks_Int;
begin
  Scanner.CurrentBoolSwitches:=Scanner.CurrentBoolSwitches+[bsOverflowChecks];
  StartProgram(false);
  Add([
  'procedure DoIt;',
  'var',
  '  b: byte;',
  '  n: nativeint;',
  '  u: nativeuint;',
  '  c: currency;',
  'begin',
  '  n:=n+n;',
  '  n:=n-n;',
  '  n:=n+b;',
  '  n:=b-n;',
  '  n:=n*n;',
  '  n:=n*u;',
  '  c:=c+b;',
  '  c:=b+c;',
  '  c:=c*b;',
  '  c:=b*c;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestOverflowChecks_Int',
    LinesToStr([ // statements
    'this.DoIt = function () {',
    '  var b = 0;',
    '  var n = 0;',
    '  var u = 0;',
    '  var c = 0;',
    '  n = rtl.oc(n + n);',
    '  n = rtl.oc(n - n);',
    '  n = rtl.oc(n + b);',
    '  n = rtl.oc(b - n);',
    '  n = rtl.oc(n * n);',
    '  n = rtl.oc(n * u);',
    '  c = rtl.oc(c + (b * 10000));',
    '  c = rtl.oc((b * 10000) + c);',
    '  c = rtl.oc(c * b);',
    '  c = rtl.oc(b * c);',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRangeChecks_AssignInt;
begin
  Scanner.Options:=Scanner.Options+[po_CAssignments];
  StartProgram(false);
  Add([
  '{$R+}',
  'var',
  '  b: byte = 2;',
  '  w: word = 3;',
  'procedure DoIt(p: byte);',
  'begin',
  '  b:=w;',
  '  b+=w;',
  '  b:=1;',
  'end;',
  '{$R-}',
  'procedure DoSome;',
  'begin',
  '  DoIt(w);',
  '  b:=w;',
  '  b:=2;',
  'end;',
  'begin',
  '{$R+}',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_AssignInt',
    LinesToStr([ // statements
    'this.b = 2;',
    'this.w = 3;',
    'this.DoIt = function (p) {',
    '  rtl.rc(p, 0, 255);',
    '  $mod.b = rtl.rc($mod.w,0,255);',
    '  rtl.rc($mod.b += $mod.w, 0, 255);',
    '  $mod.b = 1;',
    '};',
    'this.DoSome = function () {',
    '  $mod.DoIt($mod.w);',
    '  $mod.b = $mod.w;',
    '  $mod.b = 2;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRangeChecks_AssignIntRange;
begin
  Scanner.Options:=Scanner.Options+[po_CAssignments];
  StartProgram(false);
  Add([
  '{$R+}',
  'type Ten = 1..10;',
  'var',
  '  b: Ten = 2;',
  '  w: Ten = 3;',
  'procedure DoIt(p: Ten);',
  'begin',
  '  b:=w;',
  '  b+=w;',
  '  b:=1;',
  'end;',
  '{$R-}',
  'procedure DoSome;',
  'begin',
  '  DoIt(w);',
  '  b:=w;',
  '  b:=2;',
  'end;',
  'begin',
  '{$R+}',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_AssignIntRange',
    LinesToStr([ // statements
    'this.b = 2;',
    'this.w = 3;',
    'this.DoIt = function (p) {',
    '  rtl.rc(p, 1, 10);',
    '  $mod.b = rtl.rc($mod.w, 1, 10);',
    '  rtl.rc($mod.b += $mod.w, 1, 10);',
    '  $mod.b = 1;',
    '};',
    'this.DoSome = function () {',
    '  $mod.DoIt($mod.w);',
    '  $mod.b = $mod.w;',
    '  $mod.b = 2;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRangeChecks_AssignEnum;
begin
  StartProgram(false);
  Add([
  '{$R+}',
  'type TEnum = (red,green);',
  'var',
  '  e: TEnum = red;',
  'procedure DoIt(p: TEnum);',
  'begin',
  '  e:=p;',
  '  p:=TEnum(0);',
  '  p:=succ(e);',
  'end;',
  '{$R-}',
  'procedure DoSome;',
  'begin',
  '  DoIt(e);',
  '  e:=TEnum(1);',
  '  e:=pred(e);',
  'end;',
  'begin',
  '{$R+}',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_AssignEnum',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "green",',
    '  green: 1',
    '};',
    'this.e = this.TEnum.red;',
    'this.DoIt = function (p) {',
    '  rtl.rc(p, 0, 1);',
    '  $mod.e = rtl.rc(p, 0, 1);',
    '  p = 0;',
    '  p = rtl.rc($mod.e + 1, 0, 1);',
    '};',
    'this.DoSome = function () {',
    '  $mod.DoIt($mod.e);',
    '  $mod.e = 1;',
    '  $mod.e = $mod.e - 1;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRangeChecks_AssignEnumRange;
begin
  StartProgram(false);
  Add([
  '{$R+}',
  'type',
  '  TEnum = (red,green);',
  '  TEnumRg = red..green;',
  'var',
  '  e: TEnumRg = red;',
  'procedure DoIt(p: TEnumRg);',
  'begin',
  '  e:=p;',
  '  p:=TEnumRg(0);',
  '  p:=succ(e);',
  'end;',
  '{$R-}',
  'procedure DoSome;',
  'begin',
  '  DoIt(e);',
  '  e:=TEnum(1);',
  '  e:=pred(e);',
  'end;',
  'begin',
  '{$R+}',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_AssignEnumRange',
    LinesToStr([ // statements
    'this.TEnum = {',
    '  "0": "red",',
    '  red: 0,',
    '  "1": "green",',
    '  green: 1',
    '};',
    'this.e = this.TEnum.red;',
    'this.DoIt = function (p) {',
    '  rtl.rc(p, 0, 1);',
    '  $mod.e = rtl.rc(p, 0, 1);',
    '  p = 0;',
    '  p = rtl.rc($mod.e + 1, 0, 1);',
    '};',
    'this.DoSome = function () {',
    '  $mod.DoIt($mod.e);',
    '  $mod.e = 1;',
    '  $mod.e = $mod.e - 1;',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRangeChecks_AssignChar;
begin
  StartProgram(false);
  Add([
  '{$R+}',
  'type',
  '  TLetter = char;',
  'var',
  '  b: TLetter = ''2'';',
  '  w: TLetter = ''3'';',
  'procedure DoIt(p: TLetter);',
  'begin',
  '  b:=w;',
  '  b:=''1'';',
  'end;',
  '{$R-}',
  'procedure DoSome;',
  'begin',
  '  DoIt(w);',
  '  b:=w;',
  '  b:=''2'';',
  'end;',
  'begin',
  '{$R+}',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_AssignChar',
    LinesToStr([ // statements
    'this.b = "2";',
    'this.w = "3";',
    'this.DoIt = function (p) {',
    '  rtl.rcc(p, 0, 65535);',
    '  $mod.b = rtl.rcc($mod.w, 0, 65535);',
    '  $mod.b = "1";',
    '};',
    'this.DoSome = function () {',
    '  $mod.DoIt($mod.w);',
    '  $mod.b = $mod.w;',
    '  $mod.b = "2";',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRangeChecks_AssignCharRange;
begin
  StartProgram(false);
  Add([
  '{$R+}',
  'type TDigit = ''0''..''9'';',
  'var',
  '  b: TDigit = ''2'';',
  '  w: TDigit = ''3'';',
  'procedure DoIt(p: TDigit);',
  'begin',
  '  b:=w;',
  '  b:=''1'';',
  'end;',
  '{$R-}',
  'procedure DoSome;',
  'begin',
  '  DoIt(w);',
  '  b:=w;',
  '  b:=''2'';',
  'end;',
  'begin',
  '{$R+}',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_AssignCharRange',
    LinesToStr([ // statements
    'this.b = "2";',
    'this.w = "3";',
    'this.DoIt = function (p) {',
    '  rtl.rcc(p, 48, 57);',
    '  $mod.b = rtl.rcc($mod.w, 48, 57);',
    '  $mod.b = "1";',
    '};',
    'this.DoSome = function () {',
    '  $mod.DoIt($mod.w);',
    '  $mod.b = $mod.w;',
    '  $mod.b = "2";',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRangeChecks_ArrayIndex;
begin
  StartProgram(false);
  Add([
  '{$R+}',
  'type',
  '  Ten = 1..10;',
  '  TArr = array of Ten;',
  '  TArrArr = array of TArr;',
  '  TArrByte = array[byte] of Ten;',
  '  TArrChar = array[''0''..''9''] of Ten;',
  '  TArrByteChar = array[byte,''0''..''9''] of Ten;',
  '  TObject = class',
  '    A: TArr;',
  '  end;',
  'procedure DoIt;',
  'var',
  '  Arr: TArr;',
  '  ArrArr: TArrArr;',
  '  ArrByte: TArrByte;',
  '  ArrChar: TArrChar;',
  '  ArrByteChar: TArrByteChar;',
  '  i: Ten;',
  '  c: char;',
  '  o: tobject;',
  'begin',
  '  i:=Arr[1];',
  '  i:=ArrByteChar[1,''2''];',
  '  Arr[1]:=Arr[1];',
  '  Arr[i]:=Arr[i];',
  '  ArrByte[3]:=ArrByte[3];',
  '  ArrByte[i]:=ArrByte[i];',
  '  ArrChar[''5'']:=ArrChar[''5''];',
  '  ArrChar[c]:=ArrChar[c];',
  '  ArrByteChar[7,''7'']:=ArrByteChar[7,''7''];',
  '  ArrByteChar[i,c]:=ArrByteChar[i,c];',
  '  o.a[i]:=o.a[i];',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_ArrayIndex',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.A = [];',
    '  };',
    '  this.$final = function () {',
    '    this.A = undefined;',
    '  };',
    '});',
    'this.DoIt = function () {',
    '  var Arr = [];',
    '  var ArrArr = [];',
    '  var ArrByte = rtl.arraySetLength(null, 0, 256);',
    '  var ArrChar = rtl.arraySetLength(null, 0, 10);',
    '  var ArrByteChar = rtl.arraySetLength(null, 0, 256, 10);',
    '  var i = 0;',
    '  var c = "";',
    '  var o = null;',
    '  i = rtl.rc(Arr[1], 1, 10);',
    '  i = rtl.rc(ArrByteChar[1][2], 1, 10);',
    '  Arr[1] = rtl.rc(Arr[1], 1, 10);',
    '  rtl.rcArrW(Arr, i, rtl.rcArrR(Arr, i));',
    '  ArrByte[3] = rtl.rc(ArrByte[3], 1, 10);',
    '  rtl.rcArrW(ArrByte, i, rtl.rcArrR(ArrByte, i));',
    '  ArrChar[5] = rtl.rc(ArrChar[5], 1, 10);',
    '  rtl.rcArrW(ArrChar, c.charCodeAt() - 48, rtl.rcArrR(ArrChar, c.charCodeAt() - 48));',
    '  ArrByteChar[7][7] = rtl.rc(ArrByteChar[7][7], 1, 10);',
    '  rtl.rcArrW(ArrByteChar, i, c.charCodeAt() - 48, rtl.rcArrR(ArrByteChar, i, c.charCodeAt() - 48));',
    '  rtl.rcArrW(o.A, i, rtl.rcArrR(o.A, i));',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRangeChecks_ArrayOfRecIndex;
begin
  StartProgram(false);
  Add([
  '{$R+}',
  'type',
  '  Ten = 1..10;',
  '  TRec = record x: Ten end;',
  '  TArr = array of TRec;',
  '  TArrArr = array of TArr;',
  '  TObject = class',
  '    A: TArr;',
  '  end;',
  'procedure DoIt;',
  'var',
  '  Arr: TArr;',
  '  ArrArr: TArrArr;',
  '  i: Ten;',
  '  o: tobject;',
  'begin',
  '  Arr[1]:=Arr[1];',
  '  Arr[i]:=Arr[i+1];',
  '  o.a[i]:=o.a[i+2];',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_ArrayOfRecIndex',
    LinesToStr([ // statements
    'rtl.recNewT(this, "TRec", function () {',
    '  this.x = 0;',
    '  this.$eq = function (b) {',
    '    return this.x === b.x;',
    '  };',
    '  this.$assign = function (s) {',
    '    this.x = s.x;',
    '    return this;',
    '  };',
    '});',
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.A = [];',
    '  };',
    '  this.$final = function () {',
    '    this.A = undefined;',
    '  };',
    '});',
    'this.DoIt = function () {',
    '  var Arr = [];',
    '  var ArrArr = [];',
    '  var i = 0;',
    '  var o = null;',
    '  Arr[1].$assign(Arr[1]);',
    '  rtl.rcArrR(Arr, i).$assign(rtl.rcArrR(Arr, i + 1));',
    '  rtl.rcArrR(o.A, i).$assign(rtl.rcArrR(o.A, i + 2));',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRangeChecks_StringIndex;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '    S: string;',
  '  end;',
  '{$R+}',
  'procedure DoIt(var h: string);',
  'var',
  '  s: string;',
  '  i: longint;',
  '  c: char;',
  '  o: tobject;',
  'begin',
  '  c:=s[1];',
  '  s[i]:=s[i];',
  '  h[i]:=h[i];',
  '  c:=o.s[i];',
  '  o.s[i]:=c;',
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_StringIndex',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.S = "";',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'this.DoIt = function (h) {',
    '  var s = "";',
    '  var i = 0;',
    '  var c = "";',
    '  var o = null;',
    '  c = rtl.rcc(rtl.rcCharAt(s, 0), 0, 65535);',
    '  s = rtl.rcSetCharAt(s, i - 1, rtl.rcCharAt(s, i - 1));',
    '  h.set(rtl.rcSetCharAt(h.get(), i - 1, rtl.rcCharAt(h.get(), i - 1)));',
    '  c = rtl.rcc(rtl.rcCharAt(o.S, i - 1), 0, 65535);',
    '  o.S = rtl.rcSetCharAt(o.S, i - 1, c);',
    '};',
    '']),
    LinesToStr([ // $mod.$main
    '']));
end;

procedure TTestModule.TestRangeChecks_TypecastInt;
begin
  StartProgram(false);
  Add([
  '{$R+}',
  'var',
  '  i: nativeint;',
  '  b: byte;',
  '  sh: shortint;',
  '  w: word;',
  '  sm: smallint;',
  '  lw: longword;',
  '  li: longint;',
  'begin',
  '  b:=12+byte(i);',
  '  sh:=12+shortint(i);',
  '  w:=12+word(i);',
  '  sm:=12+smallint(i);',
  '  lw:=12+longword(i);',
  '  li:=12+longint(i);',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_TypecastInt',
    LinesToStr([
    'this.i = 0;',
    'this.b = 0;',
    'this.sh = 0;',
    'this.w = 0;',
    'this.sm = 0;',
    'this.lw = 0;',
    'this.li = 0;',
    '']),
    LinesToStr([
    '$mod.b = rtl.rc(12 + rtl.rc($mod.i, 0, 255), 0, 255);',
    '$mod.sh = rtl.rc(12 + rtl.rc($mod.i, -128, 127), -128, 127);',
    '$mod.w = rtl.rc(12 + rtl.rc($mod.i, 0, 65535), 0, 65535);',
    '$mod.sm = rtl.rc(12 + rtl.rc($mod.i, -32768, 32767), -32768, 32767);',
    '$mod.lw = rtl.rc(12 + rtl.rc($mod.i, 0, 4294967295), 0, 4294967295);',
    '$mod.li = rtl.rc(12 + rtl.rc($mod.i, -2147483648, 2147483647), -2147483648, 2147483647);',
    '']));
end;

procedure TTestModule.TestRangeChecks_TypeHelperInt;
begin
  Scanner.Options:=Scanner.Options+[po_CAssignments];
  StartProgram(false);
  Add([
  '{$modeswitch typehelpers}',
  '{$R+}',
  'type',
  '  TObject = class',
  '    FSize: byte;',
  '    property Size: byte read FSize;',
  '  end;',
  '  THelper = type helper for byte',
  '    procedure SetIt(w: word);',
  '  end;',
  'procedure THelper.SetIt(w: word);',
  'begin',
  '  Self:=w;',
  'end;',
  'function GetIt: byte;',
  'begin',
  '  Result.SetIt(2);',
  'end;',
  'var',
  '  b: byte = 3;',
  '  o: TObject;',
  'begin',
  '  b.SetIt(14);',
  '  with b do SetIt(15);',
  '  o.Size.SetIt(16);',
  '']);
  ConvertProgram;
  CheckSource('TestRangeChecks_AssignInt',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '    this.FSize = 0;',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createHelper(this, "THelper", null, function () {',
    '  this.SetIt = function (w) {',
    '    rtl.rc(w, 0, 65535);',
    '    this.set(w);',
    '  };',
    '});',
    'this.GetIt = function () {',
    '  var Result = 0;',
    '  $mod.THelper.SetIt.call({',
    '    get: function () {',
    '        return Result;',
    '      },',
    '    set: function (v) {',
    '        rtl.rc(v, 0, 255);',
    '        Result = v;',
    '      }',
    '  }, 2);',
    '  return Result;',
    '};',
    'this.b = 3;',
    'this.o = null;',
    '']),
    LinesToStr([ // $mod.$main
    '$mod.THelper.SetIt.call({',
    '  p: $mod,',
    '  get: function () {',
    '      return this.p.b;',
    '    },',
    '  set: function (v) {',
    '      rtl.rc(v, 0, 255);',
    '      this.p.b = v;',
    '    }',
    '}, 14);',
    'var $with = $mod.b;',
    '$mod.THelper.SetIt.call({',
    '  get: function () {',
    '      return $with;',
    '    },',
    '  set: function (v) {',
    '      rtl.rc(v, 0, 255);',
    '      $with = v;',
    '    }',
    '}, 15);',
    '$mod.THelper.SetIt.call({',
    '  p: $mod.o,',
    '  get: function () {',
    '      return this.p.FSize;',
    '    },',
    '  set: function (v) {',
    '      rtl.rc(v, 0, 255);',
    '      this.p.FSize = v;',
    '    }',
    '}, 16);',
    '']));
end;

procedure TTestModule.TestAsync_Proc;
begin
  StartProgram(false);
  Add([
  'procedure Fly(w: word = 1); async; forward;',
  'procedure Run(w: word = 2); async;',
  'begin',
  '  Fly(w);',
  '  Fly;',
  '  await(Fly(w));',
  '  await(Fly);',
  'end;',
  'procedure Fly(w: word); ',
  'begin',
  'end;',
  'begin',
  '  Run;',
  '  Run(3);',
  '']);
  CheckResolverUnexpectedHints();
  ConvertProgram;
  CheckSource('TestAsync_Proc',
    LinesToStr([ // statements
    'this.Run = async function (w) {',
    '  $mod.Fly(w);',
    '  $mod.Fly(1);',
    '  await $mod.Fly(w);',
    '  await $mod.Fly(1);',
    '};',
    'this.Fly = async function (w) {',
    '};',
    '']),
    LinesToStr([
    '$mod.Run(2);',
    '$mod.Run(3);',
    '']));
end;

procedure TTestModule.TestAsync_CallResultIsPromise;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TObject = class',
  '  end;',
  '  TJSPromise = class external name ''Promise''',
  '  end;',
  '  TBird = class',
  '    function Fly: word; async; ',
  '  end;',
  'function TBird.Fly: word; async; ',
  'begin',
  '  Result:=3;',
  '  Fly:=4+Result;',
  '  if Result=5 then ;',
  '  exit(6);',
  'end;',
  'function Run: word; async;',
  'begin',
  '  Result:=11+Result;',
  '  inc(Result);',
  'end;',
  'var',
  '  p: TJSPromise;',
  '  o: TBird;',
  'begin',
  '  p:=Run;',
  '  p:=Run();',
  '  if Run=p then ;',
  '  if p=Run then ;',
  '  if Run()=p then ;',
  '  if p=Run() then ;',
  '  p:=o.Fly;',
  '  p:=o.Fly();',
  '  if o.Fly=p then ;',
  '  if o.Fly()=p then ;',
  '  with o do begin',
  '    p:=Fly;',
  '    p:=Fly();',
  '    if Fly=p then ;',
  '    if Fly()=p then ;',
  '  end;',
  '']);
  CheckResolverUnexpectedHints();
  ConvertProgram;
  CheckSource('TestAsync_CallResultIsPromise',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.Fly = async function () {',
    '    var Result = 0;',
    '    Result = 3;',
    '    Result = 4 + Result;',
    '    if (Result === 5) ;',
    '    return 6;',
    '    return Result;',
    '  };',
    '});',
    'this.Run = async function () {',
    '  var Result = 0;',
    '  Result = 11 + Result;',
    '  Result += 1;',
    '  return Result;',
    '};',
    'this.p = null;',
    'this.o = null;',
    '']),
    LinesToStr([
    '$mod.p = $mod.Run();',
    '$mod.p = $mod.Run();',
    'if ($mod.Run() === $mod.p) ;',
    'if ($mod.p === $mod.Run()) ;',
    'if ($mod.Run() === $mod.p) ;',
    'if ($mod.p === $mod.Run()) ;',
    '$mod.p = $mod.o.Fly();',
    '$mod.p = $mod.o.Fly();',
    'if ($mod.o.Fly() === $mod.p) ;',
    'if ($mod.o.Fly() === $mod.p) ;',
    'var $with = $mod.o;',
    '$mod.p = $with.Fly();',
    '$mod.p = $with.Fly();',
    'if ($with.Fly() === $mod.p) ;',
    'if ($with.Fly() === $mod.p) ;',
    '']));
end;

procedure TTestModule.TestAsync_ConstructorFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TBird = class',
  '    constructor Create; async;',
  '  end;',
  'constructor TBird.Create; async;',
  'begin',
  'end;',
  'begin',
  '']);
  SetExpectedPasResolverError('Invalid constructor modifier async',nInvalidXModifierY);
  ConvertProgram;
end;

procedure TTestModule.TestAsync_PropertyGetterFail;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TBird = class',
  '    function GetSize: word; async;',
  '    property Size: word read GetSize;',
  '  end;',
  'function TBird.GetSize: word; async;',
  'begin',
  'end;',
  'begin',
  '']);
  SetExpectedPasResolverError('Invalid property getter modifier async',nInvalidXModifierY);
  ConvertProgram;
end;

procedure TTestModule.TestAwait_NonPromiseWithTypeFail;
begin
  StartProgram(false);
  Add([
  'procedure Run; async;',
  'begin',
  '  await(word,1);',
  'end;',
  'begin',
  '']);
  SetExpectedPasResolverError('Incompatible type arg no. 2: Got "Longint", expected "TJSPromise"',nIncompatibleTypeArgNo);
  ConvertProgram;
end;

procedure TTestModule.TestAwait_AsyncCallTypeMismatch;
begin
  StartProgram(false);
  Add([
  'type',
  '  TObject = class',
  '  end;',
  '  TBird = class',
  '  end;',
  'function Fly: TObject; async;',
  'begin',
  'end;',
  'procedure Run; async;',
  'begin',
  '  await(TBird,Fly);',
  'end;',
  'begin',
  '']);
  SetExpectedPasResolverError('Incompatible type arg no. 2: Got "TObject", expected "TBird"',nIncompatibleTypeArgNo);
  ConvertProgram;
end;

procedure TTestModule.TestAWait_OutsideAsyncFail;
begin
  StartProgram(false);
  Add([
  'procedure Crawl(w: double); ',
  'begin',
  'end;',
  'procedure Run(w: double);',
  'begin',
  '  await(Crawl(w));',
  'end;',
  'begin',
  '  Run(1);']);
  SetExpectedPasResolverError(sAWaitOnlyInAsyncProcedure,nAWaitOnlyInAsyncProcedure);
  ConvertProgram;
end;

procedure TTestModule.TestAWait_IntegerFail;
begin
  StartProgram(false);
  Add([
  'function Run: word;',
  'begin',
  'end;',
  'procedure Fly(w: word); async;',
  'begin',
  '  await(Run());',
  'end;',
  'begin',
  '  Fly(1);']);
  SetExpectedPasResolverError('async function expected, but Result:Word found',nXExpectedButYFound);
  ConvertProgram;
end;

procedure TTestModule.TestAWait_ExternalClassPromise;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSPromise = class external name ''Promise''',
  '  end;',
  '  TJSThenable = class external name ''Thenable''',
  '  end;',
  'function Fly(w: word): TJSPromise;',
  'begin',
  'end;',
  'function Jump(w: word): word; async;',
  'begin',
  'end;',
  'function Eat(w: word): TJSPromise; async;',
  'begin',
  'end;',
  'function Run(d: double): word; async;',
  'var',
  '  p: TJSPromise;',
  'begin',
  '  Result:=await(word,p);', // promise needs type
  '  Result:=await(word,Fly(3));', // promise needs type
  '  Result:=await(Jump(4));', // async non promise must omit the type
  '  Result:=await(word,Jump(5));', // async call can provide fitting type
  '  Result:=await(word,Eat(6));', // promise needs type
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestAWait_ExternalClassPromise',
    LinesToStr([ // statements
    'this.Fly = function (w) {',
    '  var Result = null;',
    '  return Result;',
    '};',
    'this.Jump = async function (w) {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    'this.Eat = async function (w) {',
    '  var Result = null;',
    '  return Result;',
    '};',
    'this.Run = async function (d) {',
    '  var Result = 0;',
    '  var p = null;',
    '  Result = await p;',
    '  Result = await $mod.Fly(3);',
    '  Result = await $mod.Jump(4);',
    '  Result = await $mod.Jump(5);',
    '  Result = await $mod.Eat(6);',
    '  return Result;',
    '};',
    '']),
    LinesToStr([
    ]));
  CheckResolverUnexpectedHints();
end;

procedure TTestModule.TestAWait_JSValue;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSPromise = class external name ''Promise''',
  '  end;',
  'function Fly(w: word): jsvalue; async;',
  'begin',
  'end;',
  'function Run(d: jsvalue; var e): word; async;',
  'begin',
  '  Result:=await(word,d);', // promise needs type
  '  d:=await(Fly(4));', // async non promise must omit the type
  '  Result:=await(word,e);', // promise needs type
  'end;',
  'begin',
  '']);
  ConvertProgram;
  CheckSource('TestAWait_JSValue',
    LinesToStr([ // statements
    'this.Fly = async function (w) {',
    '  var Result = undefined;',
    '  return Result;',
    '};',
    'this.Run = async function (d, e) {',
    '  var Result = 0;',
    '  Result = await d;',
    '  d = await $mod.Fly(4);',
    '  Result = await e.get();',
    '  return Result;',
    '};',
    '']),
    LinesToStr([
    ]));
  CheckResolverUnexpectedHints();
end;

procedure TTestModule.TestAWait_Result;
begin
  StartProgram(false);
  Add([
  '{$modeswitch externalclass}',
  'type',
  '  TJSPromise = class external name ''Promise''',
  '  end;',
  'function Crawl(d: double = 1.3): TJSPromise; ',
  'begin',
  'end;',
  'function Run(d: double = 1.6): word; async;',
  'begin',
  '  Result:=await(word,Crawl);',
  '  Result:=await(word,Crawl(4.5));',
  '  Result:=await(Run);',
  '  Result:=await(Run(6.7));',
  'end;',
  'begin',
  '  Run(1);']);
  ConvertProgram;
  CheckSource('TestAWait_Result',
    LinesToStr([ // statements
    'this.Crawl = function (d) {',
    '  var Result = null;',
    '  return Result;',
    '};',
    'this.Run = async function (d) {',
    '  var Result = 0;',
    '  Result = await $mod.Crawl(1.3);',
    '  Result = await $mod.Crawl(4.5);',
    '  Result = await $mod.Run(1.6);',
    '  Result = await $mod.Run(6.7);',
    '  return Result;',
    '};',
    '']),
    LinesToStr([
    '$mod.Run(1);'
    ]));
  CheckResolverUnexpectedHints();
end;

procedure TTestModule.TestAWait_ResultPromiseMissingTypeFail;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  '{$modeswitch externalclass}',
  'type',
  '  TJSPromise = class external name ''Promise''',
  '  end;',
  'function Run: TJSPromise; async;',
  'begin',
  'end;',
  'procedure Fly(w: word); async;',
  'begin',
  '  await(Run());',
  'end;',
  'begin',
  '  Fly(1);']);
  SetExpectedPasResolverError('Wrong number of parameters specified for call to "function await(aType,TJSPromise):aType"',
    nWrongNumberOfParametersForCallTo);
  ConvertProgram;
end;

procedure TTestModule.TestAsync_AnonymousProc;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  '{$modeswitch externalclass}',
  'type',
  '  TJSPromise = class external name ''Promise''',
  '  end;',
  'type',
  '  TFunc = reference to function(x: double): word; async;',
  'function Crawl(d: double = 1.3): word; async;',
  'begin',
  'end;',
  'var Func: TFunc;',
  'begin',
  '  Func:=function(c:double):word async begin',
  '    Result:=await(Crawl(c));',
  '  end;',
  '  Func:=function(c:double):word async assembler asm',
  '  end;',
  '  ']);
  ConvertProgram;
  CheckSource('TestAsync_AnonymousProc',
    LinesToStr([ // statements
    'this.Crawl = async function (d) {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    'this.Func = null;',
    '']),
    LinesToStr([
    '$mod.Func = async function (c) {',
    '  var Result = 0;',
    '  Result = await $mod.Crawl(c);',
    '  return Result;',
    '};',
    '$mod.Func = async function (c) {',
    '};',
    '']));
  CheckResolverUnexpectedHints();
end;

procedure TTestModule.TestAsync_ProcType;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  'type',
  '  TRefFunc = reference to function(x: double = 1.3): word; async;',
  '  TFunc = function(x: double = 1.1): word; async;',
  '  TProc = procedure(x: longint = 7); async;',
  'function Crawl(d: double): word; async;',
  'begin',
  'end;',
  'procedure Run(e:longint); async;',
  'begin',
  'end;',
  'procedure Fly(p: TProc); async;',
  'begin',
  '  await(p);',
  '  await(p());',
  'end;',
  'var',
  '  RefFunc: TRefFunc;',
  '  Func: TFunc;',
  '  Proc, ProcB: TProc;',
  'begin',
  '  Func:=@Crawl;',
  '  RefFunc:=@Crawl;',
  '  RefFunc:=function(c:double):word async begin',
  '    Result:=await(RefFunc);',
  '    Result:=await(RefFunc());',
  '    Result:=await(Func);',
  '    Result:=await(Func());',
  '    await(Proc);',
  '    await(Proc());',
  '    await(Proc(13));',
  '  end;',
  '  Proc:=@Run;',
  '  if Proc=ProcB then ;',
  '  ']);
  ConvertProgram;
  CheckResolverUnexpectedHints();
  CheckSource('TestAsync_ProcType',
    LinesToStr([ // statements
    'this.Crawl = async function (d) {',
    '  var Result = 0;',
    '  return Result;',
    '};',
    'this.Run = async function (e) {',
    '};',
    'this.Fly = async function (p) {',
    '  await p(7);',
    '  await p(7);',
    '};',
    'this.RefFunc = null;',
    'this.Func = null;',
    'this.Proc = null;',
    'this.ProcB = null;',
    '']),
    LinesToStr([
    '$mod.Func = $mod.Crawl;',
    '$mod.RefFunc = $mod.Crawl;',
    '$mod.RefFunc = async function (c) {',
    '  var Result = 0;',
    '  Result = await $mod.RefFunc(1.3);',
    '  Result = await $mod.RefFunc(1.3);',
    '  Result = await $mod.Func(1.1);',
    '  Result = await $mod.Func(1.1);',
    '  await $mod.Proc(7);',
    '  await $mod.Proc(7);',
    '  await $mod.Proc(13);',
    '  return Result;',
    '};',
    '$mod.Proc = $mod.Run;',
    'if (rtl.eqCallback($mod.Proc, $mod.ProcB)) ;',
    '']));
end;

procedure TTestModule.TestAsync_ProcTypeAsyncModMismatchFail;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  'type',
  '  TRefFunc = reference to function(x: double = 1.3): word;',
  'function Crawl(d: double): word; async;',
  'begin',
  'end;',
  'var',
  '  RefFunc: TRefFunc;',
  'begin',
  '  RefFunc:=@Crawl;',
  '  ']);
  SetExpectedPasResolverError('procedure type modifier "async" mismatch',nXModifierMismatchY);
  ConvertProgram;
end;

procedure TTestModule.TestAsync_Inherited;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  '{$modeswitch externalclass}',
  'type',
  '  TJSPromise = class external name ''Promise''',
  '  end;',
  '  TObject = class',
  '    function Run(w: word = 3): word; async; virtual;',
  '  end;',
  '  TBird = class',
  '    function Run(w: word = 3): word; async; override;',
  '  end;',
  'function TObject.Run(w: word = 3): word; async;',
  'begin',
  'end;',
  'function TBird.Run(w: word = 3): word;', // async modifier not needed in impl
  'var p: TJSPromise;',
  'begin',
  '  p:=inherited;',
  '  p:=inherited Run;',
  '  p:=inherited Run();',
  '  p:=inherited Run(4);',
  '  exit(p);',
  '  exit(inherited);',
  '  exit(inherited Run);',
  '  exit(inherited Run(5));',
  '  exit(6);',
  'end;',
  'begin',
  '  ']);
  ConvertProgram;
  CheckSource('TestAsync_Inherited',
    LinesToStr([ // statements
    'rtl.createClass(this, "TObject", null, function () {',
    '  this.$init = function () {',
    '  };',
    '  this.$final = function () {',
    '  };',
    '  this.Run = async function (w) {',
    '    var Result = 0;',
    '    return Result;',
    '  };',
    '});',
    'rtl.createClass(this, "TBird", this.TObject, function () {',
    '  this.Run = async function (w) {',
    '    var Result = 0;',
    '    var p = null;',
    '    p = $mod.TObject.Run.apply(this, arguments);',
    '    p = $mod.TObject.Run.call(this, 3);',
    '    p = $mod.TObject.Run.call(this, 3);',
    '    p = $mod.TObject.Run.call(this, 4);',
    '    return p;',
    '    return $mod.TObject.Run.apply(this, arguments);',
    '    return $mod.TObject.Run.call(this, 3);',
    '    return $mod.TObject.Run.call(this, 5);',
    '    return 6;',
    '    return Result;',
    '  };',
    '});',
    '']),
    LinesToStr([
    '']));
  CheckResolverUnexpectedHints();
end;

procedure TTestModule.TestAsync_ClassInterface;
begin
  StartProgram(false);
  Add([
  '{$mode objfpc}',
  '{$modeswitch externalclass}',
  'type',
  '  TJSPromise = class external name ''Promise''',
  '  end;',
  '  IUnknown = interface',
  '    function _AddRef: longint;',
  '    function _Release: longint;',
  '  end;',
  'function Say(i: IUnknown): IUnknown; async;',
  'begin',
  'end;',
  'function Run: IUnknown; async;',
  'begin',
  '  Result:=await(Run);',
  '  Result:=await(Run());',
  '  Result:=await(Run) as IUnknown;',
  '  Result:=await(Say(nil));',
  '  Result:=await(Say(await(Run())));',
  '  Result:=await(Say(await(Run()) as IUnknown));',
  '  Result:=await(Say(await(Run()) as IUnknown)) as IUnknown;',
  'end;',
  'procedure Fly;',
  'var p: TJSPromise;',
  'begin',
  '  Run;',
  '  Run();',
  '  p:=Run;',
  '  p:=Run();',
  'end;',
  'begin',
  '  ']);
  ConvertProgram;
  CheckSource('TestAsync_ClassInterface',
    LinesToStr([ // statements
    'rtl.createInterface(this, "IUnknown", "{D7ADB0E1-758A-322B-BDDF-21CD521DDFA9}", ["_AddRef", "_Release"], null);',
    'this.Say = async function (i) {',
    '  var Result = null;',
    '  return Result;',
    '};',
    'this.Run = async function () {',
    '  var Result = null;',
    '  var $ok = false;',
    '  try {',
    '    Result = rtl.setIntfL(Result, await $mod.Run());',
    '    Result = rtl.setIntfL(Result, await $mod.Run());',
    '    Result = rtl.setIntfL(Result, rtl.intfAsIntfT(await $mod.Run(), $mod.IUnknown));',
    '    Result = rtl.setIntfL(Result, await $mod.Say(null));',
    '    Result = rtl.setIntfL(Result, await $mod.Say(await $mod.Run()));',
    '    Result = rtl.setIntfL(Result, await $mod.Say(rtl.intfAsIntfT(await $mod.Run(), $mod.IUnknown)));',
    '    Result = rtl.setIntfL(Result, rtl.intfAsIntfT(await $mod.Say(rtl.intfAsIntfT(await $mod.Run(), $mod.IUnknown)), $mod.IUnknown));',
    '    $ok = true;',
    '  } finally {',
    '    if (!$ok) rtl._Release(Result);',
    '  };',
    '  return Result;',
    '};',
    'this.Fly = function () {',
    '  var p = null;',
    '  $mod.Run();',
    '  $mod.Run();',
    '  p = $mod.Run();',
    '  p = $mod.Run();',
    '};',
    '']),
    LinesToStr([
    '']));
  CheckResolverUnexpectedHints();
end;

procedure TTestModule.TestAsync_ClassInterface_AsyncMissmatchFail;
begin
  StartProgram(true,[supTInterfacedObject]);
  Add([
  '{$mode objfpc}',
  '{$modeswitch externalclass}',
  'type',
  '  TJSPromise = class external name ''Promise''',
  '  end;',
  '  IBird = interface',
  '    procedure Run;',
  '  end;',
  '  TBird = class(TInterfacedObject,IBird)',
  '    procedure Run; async;',
  '  end;',
  'procedure TBird.Run;',
  'begin',
  'end;',
  'begin',
  '  ']);
  SetExpectedPasResolverError('procedure type modifier "async" mismatch',nXModifierMismatchY);
  ConvertProgram;
end;

procedure TTestModule.TestLibrary_Empty;
begin
  StartLibrary(false);
  Add([
  '']);
  ConvertLibrary;
  CheckSource('TestLibrary_Empty',
    LinesToStr([ // statements
    '']),
    LinesToStr([
    '']));
  CheckResolverUnexpectedHints();
end;

procedure TTestModule.TestLibrary_ExportFunc;
begin
  exit;

  StartLibrary(false);
  Add([
  'procedure Run(w: word);',
  'begin',
  'end;',
  'exports',
  '  Run,',
  '  run name ''Foo'';',
  '']);
  ConvertLibrary;
  CheckSource('TestLibrary_ExportFunc',
    LinesToStr([ // statements
    '']),
    LinesToStr([
    '']));
  CheckResolverUnexpectedHints();
end;

Initialization
  RegisterTests([TTestModule]);
end.

